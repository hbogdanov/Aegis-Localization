#include <cmath>
#include <chrono>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "aegis_core/ukf.hpp"

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

class UkfNode : public rclcpp::Node
{
public:
  UkfNode()
  : Node("ukf_localization_node")
  {
    frame_id_ = this->declare_parameter<std::string>("frame_id", "map");
    use_odom_pose_update_ = this->declare_parameter<bool>("use_odom_pose_update", true);
    alpha_ = this->declare_parameter<double>("alpha", 0.1);
    beta_ = this->declare_parameter<double>("beta", 2.0);
    kappa_ = this->declare_parameter<double>("kappa", 0.0);

    auto process_noise = this->declare_parameter<std::vector<double>>("process_noise",
      std::vector<double>{1e-3, 1e-3, 1e-3, 1e-3, 1e-3, 1e-3});
    auto velocity_noise = this->declare_parameter<std::vector<double>>("velocity_yaw_rate_noise",
      std::vector<double>{1e-2, 1e-2, 1e-2});
    auto pose_noise = this->declare_parameter<std::vector<double>>("pose_noise",
      std::vector<double>{0.05, 0.05, 0.1});

    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/aegis/ukf_pose", 10);
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/aegis/ukf_path", 10);

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom",
      10,
      std::bind(&UkfNode::odomCallback, this, std::placeholders::_1));

    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/imu",
      10,
      std::bind(&UkfNode::imuCallback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(
      100ms,
      std::bind(&UkfNode::updateAndPublish, this));

    path_msg_.header.frame_id = frame_id_;
    last_update_time_ = this->now();

    configureNoise(process_noise, velocity_noise, pose_noise);
  }

private:
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
      last_update_time_ = this->now();
    }
  }

  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    last_imu_ = *msg;
    got_imu_ = true;
  }

  void updateAndPublish()
  {
    if (!initialized_) {
      return;
    }

    const rclcpp::Time now = this->now();
    const double dt = std::max(0.0, (now - last_update_time_).seconds());
    last_update_time_ = now;

    if (dt > 0.0) {
      ukf_.predict(dt);
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
        ukf_.updatePose(px, py, yaw);
      }

      ukf_.updateVelocityYawRate(vx, vy, omega);
    }

    publishPose();
  }

  void publishPose()
  {
    geometry_msgs::msg::PoseStamped msg;
    msg.header.stamp = this->now();
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

    path_msg_.header.stamp = msg.header.stamp;
    path_msg_.poses.push_back(msg);
    if (path_msg_.poses.size() > 200) {
      path_msg_.poses.erase(path_msg_.poses.begin());
    }
    path_pub_->publish(path_msg_);
  }

  std::string frame_id_;
  bool use_odom_pose_update_ = true;
  bool initialized_ = false;
  double alpha_ = 0.1;
  double beta_ = 2.0;
  double kappa_ = 0.0;
  rclcpp::Time last_update_time_;
  bool got_odom_ = false;
  bool got_imu_ = false;
  nav_msgs::msg::Odometry last_odom_;
  sensor_msgs::msg::Imu last_imu_;
  aegis_core::UKF ukf_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  nav_msgs::msg::Path path_msg_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<UkfNode>());
  rclcpp::shutdown();
  return 0;
}
