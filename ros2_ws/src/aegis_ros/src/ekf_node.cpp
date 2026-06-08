#include <cmath>
#include <memory>
#include <chrono>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "aegis_core/ekf.hpp"
#include "aegis_msgs/msg/filter_diagnostics.hpp"

using namespace std::chrono_literals;

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
    frame_id_ = this->declare_parameter<std::string>("frame_id", "map");
    use_odom_pose_update_ = this->declare_parameter<bool>("use_odom_pose_update", true);

    auto process_noise = this->declare_parameter<std::vector<double>>("process_noise",
      std::vector<double>{1e-3, 1e-3, 1e-3, 1e-3, 1e-3, 1e-3});
    auto velocity_noise = this->declare_parameter<std::vector<double>>("velocity_yaw_rate_noise",
      std::vector<double>{1e-2, 1e-2, 1e-2});
    auto pose_noise = this->declare_parameter<std::vector<double>>("pose_noise",
      std::vector<double>{0.05, 0.05, 0.1});

    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/aegis/ekf_pose", 10);
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/aegis/ekf_path", 10);
    diagnostics_pub_ = this->create_publisher<aegis_msgs::msg::FilterDiagnostics>("/aegis/diagnostics", 10);

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom",
      10,
      std::bind(&EkfNode::odomCallback, this, std::placeholders::_1));

    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/imu",
      10,
      std::bind(&EkfNode::imuCallback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(
      100ms,
      std::bind(&EkfNode::updateAndPublish, this));

    path_msg_.header.frame_id = frame_id_;
    last_update_time_ = this->now();

    configureNoise(process_noise, velocity_noise, pose_noise);
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    last_odom_ = *msg;
    got_odom_ = true;

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
      last_update_time_ = this->now();
    }
  }

  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    last_imu_ = *msg;
    got_imu_ = true;
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

  void updateAndPublish()
  {
    if (!initialized_) {
      last_innovation_ = 0.0;
      diagnostics_status_ = "WAITING_FOR_INIT";
      publishDiagnostics();
      return;
    }

    const rclcpp::Time now = this->now();
    const double dt = std::max(0.0, (now - last_update_time_).seconds());
    last_update_time_ = now;

    if (dt > 0.0) {
      ekf_.predict(dt);
    }

    if (got_odom_ || got_imu_) {
      double vx = 0.0;
      double vy = 0.0;
      double omega = 0.0;

      if (got_odom_) {
        vx = last_odom_.twist.twist.linear.x;
        vy = last_odom_.twist.twist.linear.y;
      }
      if (got_imu_) {
        omega = last_imu_.angular_velocity.z;
      }

      if (got_odom_ && use_odom_pose_update_) {
        const double px = last_odom_.pose.pose.position.x;
        const double py = last_odom_.pose.pose.position.y;
        const double yaw = quaternionToYaw(last_odom_.pose.pose.orientation);
        ekf_.updatePose(px, py, yaw);
      }

      const Eigen::Vector3d innovation = computeVelocityInnovation(vx, vy, omega);
      last_innovation_ = innovation.norm();
      ekf_.updateVelocityYawRate(vx, vy, omega);
      diagnostics_status_ = "OK";
    } else {
      last_innovation_ = 0.0;
      diagnostics_status_ = "NO_MEASUREMENT";
    }

    publishPose();
    publishDiagnostics();
  }

  void publishPose()
  {
    geometry_msgs::msg::PoseStamped msg;
    msg.header.stamp = this->now();
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

    path_msg_.header.stamp = msg.header.stamp;
    path_msg_.poses.push_back(msg);
    if (path_msg_.poses.size() > 200) {
      path_msg_.poses.erase(path_msg_.poses.begin());
    }
    path_pub_->publish(path_msg_);
  }

  void publishDiagnostics()
  {
    aegis_msgs::msg::FilterDiagnostics diagnostics;
    diagnostics.source = "EKF";
    diagnostics.timestamp = this->now().seconds();
    diagnostics.status = diagnostics_status_;
    diagnostics.innovation = last_innovation_;
    diagnostics_pub_->publish(diagnostics);
  }

  std::string frame_id_;
  bool use_odom_pose_update_ = true;
  bool initialized_ = false;
  rclcpp::Time last_update_time_;
  bool got_odom_ = false;
  bool got_imu_ = false;
  nav_msgs::msg::Odometry last_odom_;
  sensor_msgs::msg::Imu last_imu_;
  aegis_core::EKF ekf_;
  double last_innovation_ = 0.0;
  std::string diagnostics_status_ = "INIT";
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<aegis_msgs::msg::FilterDiagnostics>::SharedPtr diagnostics_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  nav_msgs::msg::Path path_msg_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EkfNode>());
  rclcpp::shutdown();
  return 0;
}
