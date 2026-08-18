#include <cmath>
#include <memory>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <vector>
#include <limits>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "aegis_core/ekf.hpp"
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

class EkfNode : public rclcpp::Node
{
public:
  EkfNode()
  : Node("ekf_localization_node")
  {
    stats_out_ = this->declare_parameter<std::string>("stats_out", "");
    frame_id_ = this->declare_parameter<std::string>("frame_id", "map");
    use_odom_pose_update_ = this->declare_parameter<bool>("use_odom_pose_update", true);

    auto process_noise = this->declare_parameter<std::vector<double>>("process_noise",
      std::vector<double>{1e-3, 1e-3, 1e-3, 1e-3, 1e-3, 1e-3});
    auto velocity_noise = this->declare_parameter<std::vector<double>>("velocity_yaw_rate_noise",
      std::vector<double>{1e-2, 1e-2, 1e-2});
    auto pose_noise = this->declare_parameter<std::vector<double>>("pose_noise",
      std::vector<double>{0.05, 0.05, 0.1});

    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/aegis/ekf_pose", 1000);
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/aegis/ekf_path", 1000);
    diagnostics_pub_ = this->create_publisher<aegis_msgs::msg::FilterDiagnostics>("/aegis/diagnostics", 1000);

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom",
      1000,
      std::bind(&EkfNode::odomCallback, this, std::placeholders::_1));

    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/imu",
      1000,
      std::bind(&EkfNode::imuCallback, this, std::placeholders::_1));

    path_msg_.header.frame_id = frame_id_;

    configureNoise(process_noise, velocity_noise, pose_noise);
  }

  ~EkfNode() override
  {
    writeStats();
  }

private:
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
    handle << "  \"pose_published\": " << pose_publish_count_ << "\n";
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
      ekf_.setState(initial_state);
      initialized_ = true;
      diagnostics_status_ = "OK";
      last_processed_stamp_ = msg->header.stamp;
      publishPose(msg->header.stamp);
      publishDiagnostics(msg->header.stamp);
      maybeWriteStats();
      return;
    }

    updateFromOdom(msg->header.stamp);
  }

  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    last_imu_ = *msg;
    got_imu_ = true;
    ++imu_received_count_;
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
    ekf_.setProcessNoise(Q);

    aegis_core::Matrix3d Rv = aegis_core::Matrix3d::Zero();
    for (size_t i = 0; i < velocity_noise.size() && i < 3; ++i) {
      Rv(i, i) = velocity_noise[i];
    }
    ekf_.setVelocityYawRateNoise(Rv);

    aegis_core::Matrix3d Rp = aegis_core::Matrix3d::Zero();
    for (size_t i = 0; i < pose_noise.size() && i < 3; ++i) {
      Rp(i, i) = pose_noise[i];
    }
    ekf_.setPoseNoise(Rp);
  }

  Eigen::Vector3d computeVelocityInnovation(double vx, double vy, double omega) const
  {
    const auto state_vec = ekf_.state().toVector();
    Eigen::Vector3d predicted;
    predicted << state_vec(3), state_vec(4), state_vec(5);

    Eigen::Vector3d measurement;
    measurement << vx, vy, omega;
    return measurement - predicted;
  }

  void updateFromOdom(const builtin_interfaces::msg::Time &stamp)
  {
    const rclcpp::Time current_stamp(stamp);
    const double dt = std::max(0.0, (current_stamp - last_processed_stamp_).seconds());
    last_processed_stamp_ = current_stamp;

    if (dt > 0.0) {
      ekf_.predict(dt);
      ++predict_count_;
    }

    const double vx = last_odom_.twist.twist.linear.x;
    const double vy = last_odom_.twist.twist.linear.y;
    const double omega = last_odom_.twist.twist.angular.z;

    if (use_odom_pose_update_) {
      const double px = last_odom_.pose.pose.position.x;
      const double py = last_odom_.pose.pose.position.y;
      const double yaw = quaternionToYaw(last_odom_.pose.pose.orientation);
      ekf_.updatePose(px, py, yaw);
      ++pose_update_count_;
      publishDiagnostics(stamp);
    }

    const Eigen::Vector3d innovation = computeVelocityInnovation(vx, vy, omega);
    last_innovation_ = innovation.norm();
    ekf_.updateVelocityYawRate(vx, vy, omega);
    ++velocity_update_count_;
    diagnostics_status_ = "OK";

    publishPose(stamp);
    publishDiagnostics(stamp);
    maybeWriteStats();
  }

  void publishPose(const builtin_interfaces::msg::Time &stamp)
  {
    geometry_msgs::msg::PoseStamped msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = frame_id_;

    const auto &state = ekf_.state();
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
    const auto &update = ekf_.lastUpdateDiagnostics();
    aegis_msgs::msg::FilterDiagnostics diagnostics;
    diagnostics.source = "EKF";
    diagnostics.timestamp = rclcpp::Time(stamp).seconds();
    diagnostics.status = diagnostics_status_;
    diagnostics.measurement_type = update.measurement_type;
    diagnostics.innovation_norm = update.available ? update.innovation.norm() : last_innovation_;
    diagnostics.innovation_dim = update.available ? static_cast<std::uint32_t>(update.innovation.size()) : 0U;
    diagnostics.nis = update.available ? update.nis : std::numeric_limits<double>::quiet_NaN();
    if (update.available) {
      diagnostics.innovation_vector.assign(update.innovation.data(), update.innovation.data() + update.innovation.size());
      diagnostics.innovation_covariance.reserve(
        static_cast<std::size_t>(update.innovation_covariance.rows() * update.innovation_covariance.cols()));
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
    if ((pose_publish_count_ % 500) == 0) {
      writeStats();
    }
  }

  std::string stats_out_;
  std::string frame_id_;
  bool use_odom_pose_update_ = true;
  bool initialized_ = false;
  rclcpp::Time last_processed_stamp_{0, 0, RCL_ROS_TIME};
  bool got_odom_ = false;
  bool got_imu_ = false;
  nav_msgs::msg::Odometry last_odom_;
  sensor_msgs::msg::Imu last_imu_;
  aegis_core::EKF ekf_;
  std::size_t odom_received_count_ = 0;
  std::size_t imu_received_count_ = 0;
  std::size_t predict_count_ = 0;
  std::size_t pose_update_count_ = 0;
  std::size_t velocity_update_count_ = 0;
  std::size_t pose_publish_count_ = 0;
  double last_innovation_ = 0.0;
  std::string diagnostics_status_ = "INIT";
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<aegis_msgs::msg::FilterDiagnostics>::SharedPtr diagnostics_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  nav_msgs::msg::Path path_msg_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EkfNode>());
  rclcpp::shutdown();
  return 0;
}
