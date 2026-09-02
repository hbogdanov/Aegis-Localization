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
#include "aegis_core/ukf.hpp"
#include "aegis_msgs/msg/filter_diagnostics.hpp"

namespace
{

double quaternionToYaw(const geometry_msgs::msg::Quaternion &q)
{
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

aegis_core::Matrix3d covarianceFromPoseMessage(
  const geometry_msgs::msg::PoseWithCovarianceStamped &msg,
  const aegis_core::Matrix3d &fallback)
{
  aegis_core::Matrix3d covariance = fallback;
  const double xx = msg.pose.covariance[0];
  const double yy = msg.pose.covariance[7];
  const double yaw = msg.pose.covariance[35];
  if (xx > 0.0 && yy > 0.0 && yaw > 0.0) {
    covariance.setZero();
    covariance(0, 0) = xx;
    covariance(1, 1) = yy;
    covariance(2, 2) = yaw;
  }
  return covariance;
}

}  // namespace

class UkfNode : public rclcpp::Node
{
public:
  UkfNode()
  : Node("ukf_localization_node")
  {
    stats_out_ = this->declare_parameter<std::string>("stats_out", "");
    frame_id_ = this->declare_parameter<std::string>("frame_id", "map");
    use_odom_pose_update_ = this->declare_parameter<bool>("use_odom_pose_update", true);
    pose_gating_enabled_ = this->declare_parameter<bool>("pose_gating_enabled", false);
    pose_gating_threshold_ = this->declare_parameter<double>("pose_gating_threshold", 9.324146034653893);
    correction_replay_enabled_ = this->declare_parameter<bool>("correction_replay_enabled", true);
    alpha_ = this->declare_parameter<double>("alpha", 1.0);
    beta_ = this->declare_parameter<double>("beta", 2.0);
    kappa_ = this->declare_parameter<double>("kappa", 0.0);
    max_history_seconds_ = this->declare_parameter<double>("max_history_seconds", 5.0);

    auto process_noise = this->declare_parameter<std::vector<double>>(
      "process_noise",
      std::vector<double>{1e-3, 1e-3, 1e-3, 1e-3, 1e-3, 1e-3});
    auto velocity_noise = this->declare_parameter<std::vector<double>>(
      "velocity_yaw_rate_noise",
      std::vector<double>{1e-2, 1e-2, 1e-2});
    auto pose_noise = this->declare_parameter<std::vector<double>>(
      "pose_noise",
      std::vector<double>{0.05, 0.05, 0.1});

    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/aegis/ukf_pose", 1000);
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/aegis/ukf_path", 1000);
    diagnostics_pub_ = this->create_publisher<aegis_msgs::msg::FilterDiagnostics>("/aegis/diagnostics", 1000);

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom",
      1000,
      std::bind(&UkfNode::odomCallback, this, std::placeholders::_1));

    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/imu",
      1000,
      std::bind(&UkfNode::imuCallback, this, std::placeholders::_1));

    correction_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/pose_correction",
      1000,
      std::bind(&UkfNode::correctionCallback, this, std::placeholders::_1));

    path_msg_.header.frame_id = frame_id_;

    configureNoise(process_noise, velocity_noise, pose_noise);
    ukf_.setPoseUpdateGate(pose_gating_enabled_, pose_gating_threshold_);
  }

  ~UkfNode() override
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
    aegis_core::UKF filter;
  };

  struct CorrectionRecord
  {
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
    geometry_msgs::msg::PoseWithCovarianceStamped msg;
    bool diagnostics_pending = false;
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

    const auto &health = ukf_.lastCovarianceHealth();
    handle << "{\n";
    handle << "  \"odom_received\": " << odom_received_count_ << ",\n";
    handle << "  \"imu_received\": " << imu_received_count_ << ",\n";
    handle << "  \"predict_calls\": " << predict_count_ << ",\n";
    handle << "  \"pose_update_attempts\": " << pose_update_attempt_count_ << ",\n";
    handle << "  \"pose_update_calls\": " << pose_update_count_ << ",\n";
    handle << "  \"pose_updates_rejected\": " << pose_update_rejected_count_ << ",\n";
    handle << "  \"velocity_update_calls\": " << velocity_update_count_ << ",\n";
    handle << "  \"pose_published\": " << pose_publish_count_ << ",\n";
    handle << "  \"failed_updates\": " << failed_update_count_ << ",\n";
    handle << "  \"correction_messages_received\": " << correction_received_count_ << ",\n";
    handle << "  \"correction_messages_applied\": " << correction_applied_count_ << ",\n";
    handle << "  \"correction_replays\": " << correction_replay_count_ << ",\n";
    handle << "  \"correction_naive_arrival_updates\": " << correction_naive_arrival_count_ << ",\n";
    handle << "  \"correction_history_rejections\": " << correction_history_rejection_count_ << ",\n";
    handle << "  \"last_covariance_stage\": \"" << health.stage << "\",\n";
    handle << "  \"last_min_eigenvalue\": " << health.min_eigenvalue << ",\n";
    handle << "  \"last_max_eigenvalue\": " << health.max_eigenvalue << ",\n";
    handle << "  \"last_symmetry_error\": " << health.symmetry_error << ",\n";
    handle << "  \"last_condition_number\": " << health.condition_number << ",\n";
    handle << "  \"last_covariance_finite\": " << (health.finite ? "true" : "false") << ",\n";
    handle << "  \"last_covariance_psd\": " << (health.positive_semidefinite ? "true" : "false") << ",\n";
    handle << "  \"pose_gating_enabled\": " << (pose_gating_enabled_ ? "true" : "false") << ",\n";
    handle << "  \"pose_gating_threshold\": " << pose_gating_threshold_ << "\n";
    handle << "}\n";
  }

  void configureNoise(
    const std::vector<double> &process_noise,
    const std::vector<double> &velocity_noise,
    const std::vector<double> &pose_noise)
  {
    aegis_core::Matrix6d Q = aegis_core::Matrix6d::Zero();
    for (size_t i = 0; i < process_noise.size() && i < 6; ++i) {
      Q(i, i) = process_noise[i];
    }
    ukf_.setProcessNoise(Q);

    aegis_core::Matrix3d Rv = aegis_core::Matrix3d::Zero();
    for (size_t i = 0; i < velocity_noise.size() && i < 3; ++i) {
      Rv(i, i) = velocity_noise[i];
    }
    ukf_.setVelocityYawRateNoise(Rv);

    configured_pose_noise_.setZero();
    for (size_t i = 0; i < pose_noise.size() && i < 3; ++i) {
      configured_pose_noise_(i, i) = pose_noise[i];
    }
    ukf_.setPoseNoise(configured_pose_noise_);
    ukf_.setSigmaPointParameters(alpha_, beta_, kappa_);
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
      ukf_.setState(initial_state);
      initialized_ = true;
      diagnostics_status_ = "OK";
      last_processed_stamp_ = rclcpp::Time(msg->header.stamp);
      odom_history_.push_back(OdomRecord{last_processed_stamp_, *msg});
      snapshot_history_.push_back(Snapshot{last_processed_stamp_, ukf_});
      publishPose(msg->header.stamp);
      publishDiagnostics(msg->header.stamp);
      return;
    }

    applyOdomMeasurement(*msg, true);
    odom_history_.push_back(OdomRecord{rclcpp::Time(msg->header.stamp), *msg});
    snapshot_history_.push_back(Snapshot{rclcpp::Time(msg->header.stamp), ukf_});
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
      applyCorrectionMeasurement(*msg, true);
      ++correction_naive_arrival_count_;
      publishPose(toBuiltinTime(last_processed_stamp_));
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

    processed_corrections_.push_back(CorrectionRecord{correction_stamp, *msg, true});
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

    try {
      if (dt > 0.0) {
        ukf_.predict(dt);
        ++predict_count_;
      }

      const double vx = msg.twist.twist.linear.x;
      const double vy = msg.twist.twist.linear.y;
      const double omega = msg.twist.twist.angular.z;

      if (use_odom_pose_update_) {
        ++pose_update_attempt_count_;
        if (ukf_.updatePose(
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            quaternionToYaw(msg.pose.pose.orientation)))
        {
          ++pose_update_count_;
          diagnostics_status_ = "ACCEPTED";
        } else {
          ++pose_update_rejected_count_;
          diagnostics_status_ = "REJECTED_GATE";
        }
        if (publish_outputs) {
          publishDiagnostics(msg.header.stamp);
        }
      }

      ukf_.updateVelocityYawRate(vx, vy, omega);
      ++velocity_update_count_;
      diagnostics_status_ = "ACCEPTED";

      if (publish_outputs) {
        publishPose(msg.header.stamp);
        publishDiagnostics(msg.header.stamp);
      }
    } catch (const std::exception &) {
      ++failed_update_count_;
      throw;
    }
  }

  void applyCorrectionMeasurement(const geometry_msgs::msg::PoseWithCovarianceStamped &msg, bool publish_outputs)
  {
    ++pose_update_attempt_count_;
    ukf_.setPoseNoise(covarianceFromPoseMessage(msg, configured_pose_noise_));
    if (ukf_.updatePose(
        msg.pose.pose.position.x,
        msg.pose.pose.position.y,
        quaternionToYaw(msg.pose.pose.orientation)))
    {
      ++pose_update_count_;
      ++correction_applied_count_;
      diagnostics_status_ = "ACCEPTED";
    } else {
      ++pose_update_rejected_count_;
      diagnostics_status_ = "REJECTED_GATE";
    }
    ukf_.setPoseNoise(configured_pose_noise_);

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

    ukf_ = snapshot_it->filter;
    last_processed_stamp_ = correction_stamp;

    for (auto &correction : processed_corrections_) {
      if (correction.stamp == correction_stamp) {
        // Publish the newly received correction once; historical replay stays silent.
        applyCorrectionMeasurement(correction.msg, correction.diagnostics_pending);
        correction.diagnostics_pending = false;
      }
    }
    snapshot_it->filter = ukf_;

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
        replay_snapshot_it->filter = ukf_;
        ++replay_snapshot_it;
      }
    }

    ++correction_replay_count_;
    publishPose(toBuiltinTime(last_processed_stamp_));
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
    geometry_msgs::msg::PoseStamped msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = frame_id_;

    const auto &state = ukf_.state();
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
    const auto &update = ukf_.lastUpdateDiagnostics();
    aegis_msgs::msg::FilterDiagnostics diagnostics;
    diagnostics.source = "UKF";
    diagnostics.timestamp = rclcpp::Time(stamp).seconds();
    diagnostics.status = diagnostics_status_;
    diagnostics.measurement_type = update.measurement_type;
    diagnostics.accepted = update.available ? update.accepted : false;
    diagnostics.innovation_norm = update.available ? update.innovation.norm() : 0.0;
    diagnostics.innovation_dim = update.available ? static_cast<std::uint32_t>(update.innovation.size()) : 0U;
    diagnostics.nis = update.available ? update.nis : std::numeric_limits<double>::quiet_NaN();
    if (update.available) {
      diagnostics.innovation_vector.assign(
        update.innovation.data(),
        update.innovation.data() + update.innovation.size());
      diagnostics.innovation_covariance.reserve(
        static_cast<std::size_t>(
          update.innovation_covariance.rows() * update.innovation_covariance.cols()));
      for (Eigen::Index row = 0; row < update.innovation_covariance.rows(); ++row) {
        for (Eigen::Index col = 0; col < update.innovation_covariance.cols(); ++col) {
          diagnostics.innovation_covariance.push_back(update.innovation_covariance(row, col));
        }
      }
      diagnostics.state_covariance.reserve(36);
      for (Eigen::Index row = 0; row < update.state_covariance.rows(); ++row) {
        for (Eigen::Index col = 0; col < update.state_covariance.cols(); ++col) {
          diagnostics.state_covariance.push_back(update.state_covariance(row, col));
        }
      }
    }
    diagnostics_pub_->publish(diagnostics);
  }

  void maybeWriteStats() const
  {
    if ((pose_publish_count_ % 500) == 0 || failed_update_count_ > 0) {
      writeStats();
    }
  }

  std::string stats_out_;
  std::string frame_id_;
  bool use_odom_pose_update_ = true;
  bool pose_gating_enabled_ = false;
  bool correction_replay_enabled_ = true;
  double pose_gating_threshold_ = 9.324146034653893;
  bool initialized_ = false;
  double alpha_ = 1.0;
  double beta_ = 2.0;
  double kappa_ = 0.0;
  double max_history_seconds_ = 5.0;
  rclcpp::Time last_processed_stamp_{0, 0, RCL_ROS_TIME};
  bool got_odom_ = false;
  bool got_imu_ = false;
  nav_msgs::msg::Odometry last_odom_;
  sensor_msgs::msg::Imu last_imu_;
  aegis_core::UKF ukf_;
  aegis_core::Matrix3d configured_pose_noise_ = aegis_core::Matrix3d::Zero();
  std::deque<OdomRecord> odom_history_;
  std::deque<Snapshot> snapshot_history_;
  std::vector<CorrectionRecord> processed_corrections_;
  std::size_t odom_received_count_ = 0;
  std::size_t imu_received_count_ = 0;
  std::size_t predict_count_ = 0;
  std::size_t pose_update_attempt_count_ = 0;
  std::size_t pose_update_count_ = 0;
  std::size_t pose_update_rejected_count_ = 0;
  std::size_t velocity_update_count_ = 0;
  std::size_t pose_publish_count_ = 0;
  std::size_t failed_update_count_ = 0;
  std::size_t correction_received_count_ = 0;
  std::size_t correction_applied_count_ = 0;
  std::size_t correction_replay_count_ = 0;
  std::size_t correction_naive_arrival_count_ = 0;
  std::size_t correction_history_rejection_count_ = 0;
  std::string diagnostics_status_ = "INIT";
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
  try {
    rclcpp::spin(std::make_shared<UkfNode>());
    rclcpp::shutdown();
    return 0;
  } catch (const std::exception &exception) {
    std::fprintf(stderr, "ukf_node fatal exception: %s\n", exception.what());
    rclcpp::shutdown();
    return 1;
  }
}
