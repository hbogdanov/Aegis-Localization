#include <cmath>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
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

    path_msg_.header.frame_id = frame_id_;
  }

  ~ParticleFilterNode() override
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
    handle << "  \"pose_published\": " << pose_publish_count_ << ",\n";
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

  void updateFromOdom(const builtin_interfaces::msg::Time &stamp)
  {
    const rclcpp::Time current_stamp(stamp);
    const double dt = std::max(0.0, (current_stamp - last_processed_stamp_).seconds());
    last_processed_stamp_ = current_stamp;

    const double vx = last_odom_.twist.twist.linear.x;
    const double vy = last_odom_.twist.twist.linear.y;
    const double omega = last_odom_.twist.twist.angular.z;

    if (dt > 0.0) {
      particle_filter_.predict(dt, vx, vy, omega);
      ++predict_count_;
    }

    particle_filter_.updateVelocityYawRate(vx, vy, omega);
    ++velocity_update_count_;

    if (use_odom_pose_update_) {
      const double px = last_odom_.pose.pose.position.x;
      const double py = last_odom_.pose.pose.position.y;
      const double yaw = quaternionToYaw(last_odom_.pose.pose.orientation);
      particle_filter_.updatePose(px, py, yaw);
      ++pose_update_count_;
    }

    diagnostics_status_ = "OK";
    last_effective_sample_size_ = particle_filter_.effectiveSampleSize();

    publishPose(stamp);
    publishDiagnostics(stamp);
    maybeWriteStats();
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
    diagnostics.innovation = last_effective_sample_size_;
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
  std::size_t odom_received_count_ = 0;
  std::size_t imu_received_count_ = 0;
  std::size_t predict_count_ = 0;
  std::size_t pose_update_count_ = 0;
  std::size_t velocity_update_count_ = 0;
  std::size_t pose_publish_count_ = 0;
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
  rclcpp::spin(std::make_shared<ParticleFilterNode>());
  rclcpp::shutdown();
  return 0;
}
