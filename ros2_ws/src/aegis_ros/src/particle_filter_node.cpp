#include <algorithm>
#include <cmath>
#include <deque>
#include <fstream>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "aegis_core/particle_filter.hpp"
#include "aegis_msgs/msg/filter_diagnostics.hpp"

namespace
{

double quaternionToYaw(const geometry_msgs::msg::Quaternion &q)
{
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

}  // namespace

class ParticleFilterNode : public rclcpp::Node
{
public:
  ParticleFilterNode()
  : Node("particle_filter_localization_node"),
    particle_filter_(static_cast<std::size_t>(this->declare_parameter<int>("num_particles", 500)))
  {
    stats_out_ = this->declare_parameter<std::string>("stats_out", "");
    const auto random_seed = static_cast<std::uint32_t>(this->declare_parameter<int64_t>("random_seed", 4242));
    frame_id_ = this->declare_parameter<std::string>("frame_id", "map");
    use_odom_pose_update_ = this->declare_parameter<bool>("use_odom_pose_update", true);
    correction_replay_enabled_ = this->declare_parameter<bool>("correction_replay_enabled", true);
    max_history_seconds_ = this->declare_parameter<double>("max_history_seconds", 5.0);
    process_noise_x_ = this->declare_parameter<double>("process_noise_x", 0.02);
    process_noise_y_ = this->declare_parameter<double>("process_noise_y", 0.02);
    process_noise_theta_ = this->declare_parameter<double>("process_noise_theta", 0.01);
    measurement_noise_x_ = this->declare_parameter<double>("measurement_noise_x", 0.05);
    measurement_noise_y_ = this->declare_parameter<double>("measurement_noise_y", 0.05);
    measurement_noise_theta_ = this->declare_parameter<double>("measurement_noise_theta", 0.03);
    resample_threshold_ratio_ = this->declare_parameter<double>("resample_threshold_ratio", 0.5);

    particle_filter_.setNoiseParameters(
      process_noise_x_,
      process_noise_y_,
      process_noise_theta_,
      measurement_noise_x_,
      measurement_noise_y_,
      measurement_noise_theta_,
      resample_threshold_ratio_);
    particle_filter_.setRandomSeed(random_seed);

    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/aegis/pf_pose", 1000);
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/aegis/pf_path", 1000);
    diagnostics_pub_ = this->create_publisher<aegis_msgs::msg::FilterDiagnostics>("/aegis/diagnostics", 1000);

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom",
      1000,
      std::bind(&ParticleFilterNode::odomCallback, this, std::placeholders::_1));

    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/imu",
      1000,
      std::bind(&ParticleFilterNode::imuCallback, this, std::placeholders::_1));

    correction_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/pose_correction",
      1000,
      std::bind(&ParticleFilterNode::correctionCallback, this, std::placeholders::_1));

    path_msg_.header.frame_id = frame_id_;
  }

  ~ParticleFilterNode() override
  {
    writeStats();
  }

private:
  struct OdomRecord
  {
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
    nav_msgs::msg::Odometry msg;
  };

  struct Snapshot
  {
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
    aegis_core::ParticleFilter filter;
  };

  struct CorrectionRecord
  {
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
    geometry_msgs::msg::PoseWithCovarianceStamped msg;
  };

  std::deque<Snapshot>::iterator findSnapshot(const rclcpp::Time &stamp)
  {
    return std::find_if(
      snapshot_history_.begin(),
      snapshot_history_.end(),
      [&stamp](const Snapshot &snapshot) {
        return snapshot.stamp == stamp;
      });
  }

  std::deque<OdomRecord>::iterator findOdomRecord(const rclcpp::Time &stamp)
  {
    return std::find_if(
      odom_history_.begin(),
      odom_history_.end(),
      [&stamp](const OdomRecord &record) {
        return record.stamp == stamp;
      });
  }

  void writeStats() const
  {
    if (stats_out_.empty()) {
      return;
    }

    std::ofstream handle(stats_out_, std::ios::out | std::ios::trunc);
    if (!handle.is_open()) {
      return;
    }

    handle << "{\n";
    handle << "  \"odom_received\": " << odom_received_count_ << ",\n";
    handle << "  \"imu_received\": " << imu_received_count_ << ",\n";
    handle << "  \"predict_calls\": " << predict_count_ << ",\n";
    handle << "  \"pose_update_calls\": " << pose_update_count_ << ",\n";
    handle << "  \"velocity_update_calls\": " << velocity_update_count_ << ",\n";
    handle << "  \"pose_published\": " << pose_publish_count_ << ",\n";
    handle << "  \"correction_messages_received\": " << correction_received_count_ << ",\n";
    handle << "  \"correction_messages_applied\": " << correction_applied_count_ << ",\n";
    handle << "  \"correction_replays\": " << correction_replay_count_ << ",\n";
    handle << "  \"correction_naive_arrival_updates\": " << correction_naive_arrival_count_ << ",\n";
    handle << "  \"correction_history_rejections\": " << correction_history_rejection_count_ << ",\n";
    handle << "  \"effective_sample_size\": " << last_effective_sample_size_ << "\n";
    handle << "}\n";
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    last_odom_ = *msg;
    got_odom_ = true;
    ++odom_received_count_;

    if (!initialized_) {
      aegis_core::State2D initial_state;
      initial_state.px = msg->pose.pose.position.x;
      initial_state.py = msg->pose.pose.position.y;
      initial_state.theta = quaternionToYaw(msg->pose.pose.orientation);
      initial_state.vx = msg->twist.twist.linear.x;
      initial_state.vy = msg->twist.twist.linear.y;
      initial_state.omega = msg->twist.twist.angular.z;
      particle_filter_.initialize(
        initial_state,
        measurement_noise_x_,
        measurement_noise_y_,
        measurement_noise_theta_);
      initialized_ = true;
      diagnostics_status_ = "OK";
      last_processed_stamp_ = rclcpp::Time(msg->header.stamp);
      odom_history_.push_back(OdomRecord{last_processed_stamp_, *msg});
      snapshot_history_.push_back(Snapshot{last_processed_stamp_, particle_filter_});
      publishPose(msg->header.stamp);
      publishDiagnostics(msg->header.stamp);
      return;
    }

    applyOdomMeasurement(*msg, true);
    odom_history_.push_back(OdomRecord{rclcpp::Time(msg->header.stamp), *msg});
    snapshot_history_.push_back(Snapshot{rclcpp::Time(msg->header.stamp), particle_filter_});
    trimHistory();
    maybeWriteStats();
  }

  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    last_imu_ = *msg;
    got_imu_ = true;
    ++imu_received_count_;
  }

  void correctionCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    ++correction_received_count_;
    if (!initialized_) {
      return;
    }

    if (!correction_replay_enabled_) {
      applyCorrectionMeasurement(*msg, false);
      ++correction_naive_arrival_count_;
      publishPose(toBuiltinTime(last_processed_stamp_));
      publishDiagnostics(toBuiltinTime(last_processed_stamp_));
      maybeWriteStats();
      return;
    }

    const rclcpp::Time correction_stamp(msg->header.stamp);
    if (!odom_history_.empty() && correction_stamp < odom_history_.front().stamp) {
      ++correction_history_rejection_count_;
      return;
    }
    if (correction_stamp > last_processed_stamp_) {
      return;
    }

    processed_corrections_.push_back(CorrectionRecord{correction_stamp, *msg});
    std::stable_sort(
      processed_corrections_.begin(),
      processed_corrections_.end(),
      [](const CorrectionRecord &lhs, const CorrectionRecord &rhs) {
        return lhs.stamp < rhs.stamp;
      });

    replayFromCorrection(correction_stamp);
    maybeWriteStats();
  }

  void applyOdomMeasurement(const nav_msgs::msg::Odometry &msg, bool publish_outputs)
  {
    const rclcpp::Time current_stamp(msg.header.stamp);
    const double dt = std::max(0.0, (current_stamp - last_processed_stamp_).seconds());
    last_processed_stamp_ = current_stamp;

    const double vx = msg.twist.twist.linear.x;
    const double vy = msg.twist.twist.linear.y;
    const double omega = msg.twist.twist.angular.z;

    if (dt > 0.0) {
      particle_filter_.predict(dt, vx, vy, omega);
      ++predict_count_;
    }

    particle_filter_.updateVelocityYawRate(vx, vy, omega);
    ++velocity_update_count_;

    if (use_odom_pose_update_) {
      particle_filter_.updatePose(
        msg.pose.pose.position.x,
        msg.pose.pose.position.y,
        quaternionToYaw(msg.pose.pose.orientation));
      ++pose_update_count_;
    }

    diagnostics_status_ = "OK";
    last_effective_sample_size_ = particle_filter_.effectiveSampleSize();

    if (publish_outputs) {
      publishPose(msg.header.stamp);
      publishDiagnostics(msg.header.stamp);
    }
  }

  void applyCorrectionMeasurement(const geometry_msgs::msg::PoseWithCovarianceStamped &msg, bool publish_outputs)
  {
    const double noise_x = std::sqrt(std::max(msg.pose.covariance[0], 1e-9));
    const double noise_y = std::sqrt(std::max(msg.pose.covariance[7], 1e-9));
    const double noise_theta = std::sqrt(std::max(msg.pose.covariance[35], 1e-9));
    particle_filter_.setMeasurementNoise(noise_x, noise_y, noise_theta);
    particle_filter_.updatePose(
      msg.pose.pose.position.x,
      msg.pose.pose.position.y,
      quaternionToYaw(msg.pose.pose.orientation));
    particle_filter_.setMeasurementNoise(measurement_noise_x_, measurement_noise_y_, measurement_noise_theta_);
    ++pose_update_count_;
    ++correction_applied_count_;
    last_effective_sample_size_ = particle_filter_.effectiveSampleSize();
    diagnostics_status_ = "OK";

    if (publish_outputs) {
      publishDiagnostics(msg.header.stamp);
    }
  }

  void replayFromCorrection(const rclcpp::Time &correction_stamp)
  {
    const auto snapshot_it = findSnapshot(correction_stamp);
    if (snapshot_it == snapshot_history_.end()) {
      ++correction_history_rejection_count_;
      return;
    }

    const auto odom_it = findOdomRecord(correction_stamp);
    if (odom_it == odom_history_.end()) {
      ++correction_history_rejection_count_;
      return;
    }

    particle_filter_ = snapshot_it->filter;
    last_processed_stamp_ = correction_stamp;

    for (const auto &correction : processed_corrections_) {
      if (correction.stamp == correction_stamp) {
        applyCorrectionMeasurement(correction.msg, false);
      }
    }
    snapshot_it->filter = particle_filter_;

    auto replay_it = odom_it;
    auto replay_snapshot_it = snapshot_it;
    ++replay_it;
    ++replay_snapshot_it;
    for (; replay_it != odom_history_.end(); ++replay_it) {
      applyOdomMeasurement(replay_it->msg, false);
      for (const auto &correction : processed_corrections_) {
        if (correction.stamp == replay_it->stamp) {
          applyCorrectionMeasurement(correction.msg, false);
        }
      }
      if (replay_snapshot_it != snapshot_history_.end() && replay_snapshot_it->stamp == replay_it->stamp) {
        replay_snapshot_it->filter = particle_filter_;
        ++replay_snapshot_it;
      }
    }

    ++correction_replay_count_;
    publishPose(toBuiltinTime(last_processed_stamp_));
    publishDiagnostics(toBuiltinTime(last_processed_stamp_));
  }

  builtin_interfaces::msg::Time toBuiltinTime(const rclcpp::Time &stamp) const
  {
    return stamp;
  }

  void trimHistory()
  {
    const rclcpp::Duration max_history = rclcpp::Duration::from_seconds(max_history_seconds_);
    while (!odom_history_.empty() && (last_processed_stamp_ - odom_history_.front().stamp) > max_history) {
      const rclcpp::Time trimmed_stamp = odom_history_.front().stamp;
      odom_history_.pop_front();
      if (!snapshot_history_.empty() && snapshot_history_.front().stamp <= trimmed_stamp) {
        snapshot_history_.pop_front();
      }
      if (odom_history_.empty()) {
        processed_corrections_.clear();
      } else {
        processed_corrections_.erase(
          std::remove_if(
            processed_corrections_.begin(),
            processed_corrections_.end(),
            [this](const CorrectionRecord &correction) {
              return correction.stamp < odom_history_.front().stamp;
            }),
          processed_corrections_.end());
      }
    }
  }

  void publishPose(const builtin_interfaces::msg::Time &stamp)
  {
    const aegis_core::State2D state = particle_filter_.estimateState();

    geometry_msgs::msg::PoseStamped msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = frame_id_;
    msg.pose.position.x = state.px;
    msg.pose.position.y = state.py;
    msg.pose.position.z = 0.0;

    const double half_yaw = state.theta * 0.5;
    msg.pose.orientation.x = 0.0;
    msg.pose.orientation.y = 0.0;
    msg.pose.orientation.z = std::sin(half_yaw);
    msg.pose.orientation.w = std::cos(half_yaw);

    pose_pub_->publish(msg);
    ++pose_publish_count_;

    path_msg_.header.stamp = msg.header.stamp;
    path_msg_.poses.push_back(msg);
    if (path_msg_.poses.size() > 200) {
      path_msg_.poses.erase(path_msg_.poses.begin());
    }
    path_pub_->publish(path_msg_);
  }

  void publishDiagnostics(const builtin_interfaces::msg::Time &stamp)
  {
    aegis_msgs::msg::FilterDiagnostics diagnostics;
    diagnostics.source = "PF";
    diagnostics.timestamp = rclcpp::Time(stamp).seconds();
    diagnostics.status = diagnostics_status_;
    diagnostics.measurement_type = "effective_sample_size";
    diagnostics.accepted = true;
    diagnostics.innovation_norm = last_effective_sample_size_;
    diagnostics.innovation_dim = 0U;
    diagnostics.nis = std::numeric_limits<double>::quiet_NaN();
    diagnostics_pub_->publish(diagnostics);
  }

  void maybeWriteStats() const
  {
    if ((pose_publish_count_ % 500) == 0) {
      writeStats();
    }
  }

  std::string stats_out_;
  std::string frame_id_;
  bool use_odom_pose_update_ = true;
  bool correction_replay_enabled_ = true;
  bool initialized_ = false;
  double max_history_seconds_ = 5.0;
  double process_noise_x_ = 0.02;
  double process_noise_y_ = 0.02;
  double process_noise_theta_ = 0.01;
  double measurement_noise_x_ = 0.05;
  double measurement_noise_y_ = 0.05;
  double measurement_noise_theta_ = 0.03;
  double resample_threshold_ratio_ = 0.5;
  double last_effective_sample_size_ = 0.0;
  std::string diagnostics_status_ = "INIT";
  rclcpp::Time last_processed_stamp_{0, 0, RCL_ROS_TIME};
  bool got_odom_ = false;
  bool got_imu_ = false;
  nav_msgs::msg::Odometry last_odom_;
  sensor_msgs::msg::Imu last_imu_;
  aegis_core::ParticleFilter particle_filter_;
  std::deque<OdomRecord> odom_history_;
  std::deque<Snapshot> snapshot_history_;
  std::vector<CorrectionRecord> processed_corrections_;
  std::size_t odom_received_count_ = 0;
  std::size_t imu_received_count_ = 0;
  std::size_t predict_count_ = 0;
  std::size_t pose_update_count_ = 0;
  std::size_t velocity_update_count_ = 0;
  std::size_t pose_publish_count_ = 0;
  std::size_t correction_received_count_ = 0;
  std::size_t correction_applied_count_ = 0;
  std::size_t correction_replay_count_ = 0;
  std::size_t correction_naive_arrival_count_ = 0;
  std::size_t correction_history_rejection_count_ = 0;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<aegis_msgs::msg::FilterDiagnostics>::SharedPtr diagnostics_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr correction_sub_;
  nav_msgs::msg::Path path_msg_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ParticleFilterNode>());
  rclcpp::shutdown();
  return 0;
}
