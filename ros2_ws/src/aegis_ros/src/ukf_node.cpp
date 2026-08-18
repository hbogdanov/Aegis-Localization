#include <cmath>
#include <chrono>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <memory>
#include <sstream>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "aegis_core/ukf.hpp"

namespace
{

double quaternionToYaw(const geometry_msgs::msg::Quaternion &q)
{
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
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
    alpha_ = this->declare_parameter<double>("alpha", 1.0);
    beta_ = this->declare_parameter<double>("beta", 2.0);
    kappa_ = this->declare_parameter<double>("kappa", 0.0);

    auto process_noise = this->declare_parameter<std::vector<double>>("process_noise",
      std::vector<double>{1e-3, 1e-3, 1e-3, 1e-3, 1e-3, 1e-3});
    auto velocity_noise = this->declare_parameter<std::vector<double>>("velocity_yaw_rate_noise",
      std::vector<double>{1e-2, 1e-2, 1e-2});
    auto pose_noise = this->declare_parameter<std::vector<double>>("pose_noise",
      std::vector<double>{0.05, 0.05, 0.1});

    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/aegis/ukf_pose", 1000);
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/aegis/ukf_path", 1000);

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom",
      1000,
      std::bind(&UkfNode::odomCallback, this, std::placeholders::_1));

    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/imu",
      1000,
      std::bind(&UkfNode::imuCallback, this, std::placeholders::_1));

    path_msg_.header.frame_id = frame_id_;

    configureNoise(process_noise, velocity_noise, pose_noise);
  }

  ~UkfNode() override
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

    const auto &health = ukf_.lastCovarianceHealth();
    handle << "{\n";
    handle << "  \"odom_received\": " << odom_received_count_ << ",\n";
    handle << "  \"imu_received\": " << imu_received_count_ << ",\n";
    handle << "  \"predict_calls\": " << predict_count_ << ",\n";
    handle << "  \"pose_update_calls\": " << pose_update_count_ << ",\n";
    handle << "  \"velocity_update_calls\": " << velocity_update_count_ << ",\n";
    handle << "  \"pose_published\": " << pose_publish_count_ << ",\n";
    handle << "  \"failed_updates\": " << failed_update_count_ << ",\n";
    handle << "  \"last_covariance_stage\": \"" << health.stage << "\",\n";
    handle << "  \"last_min_eigenvalue\": " << health.min_eigenvalue << ",\n";
    handle << "  \"last_max_eigenvalue\": " << health.max_eigenvalue << ",\n";
    handle << "  \"last_symmetry_error\": " << health.symmetry_error << ",\n";
    handle << "  \"last_condition_number\": " << health.condition_number << ",\n";
    handle << "  \"last_covariance_finite\": " << (health.finite ? "true" : "false") << ",\n";
    handle << "  \"last_covariance_psd\": " << (health.positive_semidefinite ? "true" : "false") << "\n";
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

    aegis_core::Matrix3d Rp = aegis_core::Matrix3d::Zero();
    for (size_t i = 0; i < pose_noise.size() && i < 3; ++i) {
      Rp(i, i) = pose_noise[i];
    }
    ukf_.setPoseNoise(Rp);
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
      last_processed_stamp_ = msg->header.stamp;
      publishPose(msg->header.stamp);
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

  void updateFromOdom(const builtin_interfaces::msg::Time &stamp)
  {
    const rclcpp::Time current_stamp(stamp);
    const double dt = std::max(0.0, (current_stamp - last_processed_stamp_).seconds());
    last_processed_stamp_ = current_stamp;

    try {
      if (dt > 0.0) {
        ukf_.predict(dt);
        ++predict_count_;
      }

      const double vx = last_odom_.twist.twist.linear.x;
      const double vy = last_odom_.twist.twist.linear.y;
      const double omega = last_odom_.twist.twist.angular.z;

      if (use_odom_pose_update_) {
        const double px = last_odom_.pose.pose.position.x;
        const double py = last_odom_.pose.pose.position.y;
        const double yaw = quaternionToYaw(last_odom_.pose.pose.orientation);
        ukf_.updatePose(px, py, yaw);
        ++pose_update_count_;
      }

      ukf_.updateVelocityYawRate(vx, vy, omega);
      ++velocity_update_count_;
      ++processed_odom_updates_;
      publishPose(stamp);
      maybeWriteStats();
    } catch (const std::exception &exception) {
      ++failed_update_count_;
      const auto &state = ukf_.state();
      const auto &health = ukf_.lastCovarianceHealth();
      std::ostringstream message;
      message
        << "UKF update failed after " << processed_odom_updates_
        << " processed odom updates; dt=" << dt
        << " state=[px=" << state.px
        << ", py=" << state.py
        << ", theta=" << state.theta
        << ", vx=" << state.vx
        << ", vy=" << state.vy
        << ", omega=" << state.omega
        << "] measurement=[vx=" << last_odom_.twist.twist.linear.x
        << ", vy=" << last_odom_.twist.twist.linear.y
        << ", omega=" << last_odom_.twist.twist.angular.z
        << "] pose_measurement=[px=" << last_odom_.pose.pose.position.x
        << ", py=" << last_odom_.pose.pose.position.y
        << ", yaw=" << quaternionToYaw(last_odom_.pose.pose.orientation)
        << "] covariance_health=[stage=" << health.stage
        << ", min_eigenvalue=" << health.min_eigenvalue
        << ", max_eigenvalue=" << health.max_eigenvalue
        << ", symmetry_error=" << health.symmetry_error
        << ", condition_number=" << health.condition_number
        << ", finite=" << (health.finite ? "true" : "false")
        << ", psd=" << (health.positive_semidefinite ? "true" : "false")
        << "] error=" << exception.what();
      RCLCPP_ERROR(this->get_logger(), "%s", message.str().c_str());
      std::fprintf(stderr, "%s\n", message.str().c_str());
      throw;
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

  void maybeWriteStats() const
  {
    if ((pose_publish_count_ % 500) == 0 || failed_update_count_ > 0) {
      writeStats();
    }
  }

  std::string stats_out_;
  std::string frame_id_;
  bool use_odom_pose_update_ = true;
  bool initialized_ = false;
  double alpha_ = 1.0;
  double beta_ = 2.0;
  double kappa_ = 0.0;
  rclcpp::Time last_processed_stamp_{0, 0, RCL_ROS_TIME};
  bool got_odom_ = false;
  bool got_imu_ = false;
  nav_msgs::msg::Odometry last_odom_;
  sensor_msgs::msg::Imu last_imu_;
  aegis_core::UKF ukf_;
  std::size_t odom_received_count_ = 0;
  std::size_t imu_received_count_ = 0;
  std::size_t predict_count_ = 0;
  std::size_t pose_update_count_ = 0;
  std::size_t velocity_update_count_ = 0;
  std::size_t processed_odom_updates_ = 0;
  std::size_t pose_publish_count_ = 0;
  std::size_t failed_update_count_ = 0;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
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
