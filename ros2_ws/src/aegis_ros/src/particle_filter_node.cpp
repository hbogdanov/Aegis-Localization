#include <cmath>
#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "aegis_core/particle_filter.hpp"
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

class ParticleFilterNode : public rclcpp::Node
{
public:
  ParticleFilterNode()
  : Node("particle_filter_localization_node"),
    particle_filter_(static_cast<std::size_t>(this->declare_parameter<int>("num_particles", 500)))
  {
    frame_id_ = this->declare_parameter<std::string>("frame_id", "map");
    use_odom_pose_update_ = this->declare_parameter<bool>("use_odom_pose_update", true);
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

    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/aegis/pf_pose", 10);
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/aegis/pf_path", 10);
    diagnostics_pub_ = this->create_publisher<aegis_msgs::msg::FilterDiagnostics>("/aegis/diagnostics", 10);

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom",
      10,
      std::bind(&ParticleFilterNode::odomCallback, this, std::placeholders::_1));

    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/imu",
      10,
      std::bind(&ParticleFilterNode::imuCallback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(100ms, std::bind(&ParticleFilterNode::updateAndPublish, this));

    path_msg_.header.frame_id = frame_id_;
    last_update_time_ = this->now();
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
      particle_filter_.initialize(
        initial_state,
        measurement_noise_x_,
        measurement_noise_y_,
        measurement_noise_theta_);
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

  void updateAndPublish()
  {
    if (!initialized_) {
      diagnostics_status_ = "WAITING_FOR_INIT";
      last_effective_sample_size_ = 0.0;
      publishDiagnostics();
      return;
    }

    const rclcpp::Time now = this->now();
    const double dt = std::max(0.0, (now - last_update_time_).seconds());
    last_update_time_ = now;

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

    if (dt > 0.0) {
      particle_filter_.predict(dt, vx, vy, omega);
    }

    if (got_odom_ || got_imu_) {
      particle_filter_.updateVelocityYawRate(vx, vy, omega);

      if (got_odom_ && use_odom_pose_update_) {
        const double px = last_odom_.pose.pose.position.x;
        const double py = last_odom_.pose.pose.position.y;
        const double yaw = quaternionToYaw(last_odom_.pose.pose.orientation);
        particle_filter_.updatePose(px, py, yaw);
      }

      diagnostics_status_ = "OK";
      last_effective_sample_size_ = particle_filter_.effectiveSampleSize();
    } else {
      diagnostics_status_ = "NO_MEASUREMENT";
      last_effective_sample_size_ = particle_filter_.effectiveSampleSize();
    }

    publishPose();
    publishDiagnostics();
  }

  void publishPose()
  {
    const aegis_core::State2D state = particle_filter_.estimateState();

    geometry_msgs::msg::PoseStamped msg;
    msg.header.stamp = this->now();
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
    diagnostics.source = "PF";
    diagnostics.timestamp = this->now().seconds();
    diagnostics.status = diagnostics_status_;
    diagnostics.innovation = last_effective_sample_size_;
    diagnostics_pub_->publish(diagnostics);
  }

  std::string frame_id_;
  bool use_odom_pose_update_ = true;
  bool initialized_ = false;
  double process_noise_x_ = 0.02;
  double process_noise_y_ = 0.02;
  double process_noise_theta_ = 0.01;
  double measurement_noise_x_ = 0.05;
  double measurement_noise_y_ = 0.05;
  double measurement_noise_theta_ = 0.03;
  double resample_threshold_ratio_ = 0.5;
  double last_effective_sample_size_ = 0.0;
  std::string diagnostics_status_ = "INIT";
  rclcpp::Time last_update_time_;
  bool got_odom_ = false;
  bool got_imu_ = false;
  nav_msgs::msg::Odometry last_odom_;
  sensor_msgs::msg::Imu last_imu_;
  aegis_core::ParticleFilter particle_filter_;
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
  rclcpp::spin(std::make_shared<ParticleFilterNode>());
  rclcpp::shutdown();
  return 0;
}
