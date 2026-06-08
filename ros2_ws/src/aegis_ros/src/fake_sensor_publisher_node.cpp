#include <chrono>
#include <cmath>
#include <memory>
#include <random>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

using namespace std::chrono_literals;

class FakeSensorPublisher : public rclcpp::Node
{
public:
  FakeSensorPublisher()
  : Node("fake_sensor_publisher")
  , rng_(std::random_device{}())
  {
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu", 10);
    truth_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/ground_truth/pose", 10);

    radius_ = this->declare_parameter<double>("radius", 1.0);
    omega_ = this->declare_parameter<double>("omega", 0.5); // rad/s
    odom_position_noise_std_ = this->declare_parameter<double>("odom_position_noise_std", 0.05);
    odom_velocity_noise_std_ = this->declare_parameter<double>("odom_velocity_noise_std", 0.02);
    imu_yaw_rate_noise_std_ = this->declare_parameter<double>("imu_yaw_rate_noise_std", 0.01);
    dropout_probability_ = this->declare_parameter<double>("dropout_probability", 0.05);

    start_time_ = this->now();
    timer_ = this->create_wall_timer(50ms, std::bind(&FakeSensorPublisher::timerCallback, this));
  }

private:
  double addNoise(double mean, double stddev)
  {
    if (stddev <= 0.0) {
      return mean;
    }
    std::normal_distribution<double> dist(0.0, stddev);
    return mean + dist(rng_);
  }

  void timerCallback()
  {
    rclcpp::Time now = this->now();
    double t = (now - start_time_).seconds();

    double x = radius_ * std::cos(omega_ * t);
    double y = radius_ * std::sin(omega_ * t);
    double theta = omega_ * t + M_PI_2;

    double vx = -radius_ * omega_ * std::sin(omega_ * t);
    double vy = radius_ * omega_ * std::cos(omega_ * t);

    geometry_msgs::msg::PoseStamped truth;
    truth.header.stamp = now;
    truth.header.frame_id = "odom";
    truth.pose.position.x = x;
    truth.pose.position.y = y;
    truth.pose.position.z = 0.0;

    const double half_yaw = theta * 0.5;
    truth.pose.orientation.x = 0.0;
    truth.pose.orientation.y = 0.0;
    truth.pose.orientation.z = std::sin(half_yaw);
    truth.pose.orientation.w = std::cos(half_yaw);
    truth_pub_->publish(truth);

    const bool dropout = std::bernoulli_distribution(dropout_probability_)(rng_);
    if (dropout) {
      return;
    }

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = now;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";

    odom.pose.pose.position.x = addNoise(x, odom_position_noise_std_);
    odom.pose.pose.position.y = addNoise(y, odom_position_noise_std_);
    odom.pose.pose.position.z = 0.0;

    odom.pose.pose.orientation.x = 0.0;
    odom.pose.pose.orientation.y = 0.0;
    odom.pose.pose.orientation.z = std::sin(half_yaw);
    odom.pose.pose.orientation.w = std::cos(half_yaw);

    odom.twist.twist.linear.x = addNoise(vx, odom_velocity_noise_std_);
    odom.twist.twist.linear.y = addNoise(vy, odom_velocity_noise_std_);
    odom.twist.twist.linear.z = 0.0;
    odom.twist.twist.angular.x = 0.0;
    odom.twist.twist.angular.y = 0.0;
    odom.twist.twist.angular.z = addNoise(omega_, imu_yaw_rate_noise_std_);

    sensor_msgs::msg::Imu imu;
    imu.header = odom.header;
    imu.angular_velocity.x = 0.0;
    imu.angular_velocity.y = 0.0;
    imu.angular_velocity.z = addNoise(omega_, imu_yaw_rate_noise_std_);

    odom_pub_->publish(odom);
    imu_pub_->publish(imu);
  }

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr truth_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time start_time_;
  double radius_;
  double omega_;
  double odom_position_noise_std_;
  double odom_velocity_noise_std_;
  double imu_yaw_rate_noise_std_;
  double dropout_probability_;
  std::mt19937_64 rng_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FakeSensorPublisher>());
  rclcpp::shutdown();
  return 0;
}
