#include <chrono>
#include <cmath>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"

using namespace std::chrono_literals;

class FakeSensorPublisher : public rclcpp::Node
{
public:
  FakeSensorPublisher()
  : Node("fake_sensor_publisher")
  {
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu", 10);

    radius_ = this->declare_parameter<double>("radius", 1.0);
    omega_ = this->declare_parameter<double>("omega", 0.5); // rad/s

    start_time_ = this->now();

    timer_ = this->create_wall_timer(50ms, std::bind(&FakeSensorPublisher::timerCallback, this));
  }

private:
  void timerCallback()
  {
    rclcpp::Time now = this->now();
    double t = (now - start_time_).seconds();

    double x = radius_ * std::cos(omega_ * t);
    double y = radius_ * std::sin(omega_ * t);
    double theta = omega_ * t + M_PI_2;

    // velocities in world frame
    double vx = -radius_ * omega_ * std::sin(omega_ * t);
    double vy = radius_ * omega_ * std::cos(omega_ * t);

    // Odometry
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = now;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";

    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;

    double half_yaw = theta * 0.5;
    odom.pose.pose.orientation.x = 0.0;
    odom.pose.pose.orientation.y = 0.0;
    odom.pose.pose.orientation.z = std::sin(half_yaw);
    odom.pose.pose.orientation.w = std::cos(half_yaw);

    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.linear.z = 0.0;
    odom.twist.twist.angular.x = 0.0;
    odom.twist.twist.angular.y = 0.0;
    odom.twist.twist.angular.z = omega_;

    // IMU
    sensor_msgs::msg::Imu imu;
    imu.header = odom.header;
    imu.angular_velocity.x = 0.0;
    imu.angular_velocity.y = 0.0;
    imu.angular_velocity.z = omega_;

    odom_pub_->publish(odom);
    imu_pub_->publish(imu);
  }

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time start_time_;
  double radius_;
  double omega_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FakeSensorPublisher>());
  rclcpp::shutdown();
  return 0;
}
