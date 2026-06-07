#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"

using namespace std::chrono_literals;

class EkfNode : public rclcpp::Node
{
public:
  EkfNode()
  : Node("ekf_localization_node")
  {
    frame_id_ = this->declare_parameter<std::string>("frame_id", "map");
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/aegis/ekf_pose", 10);
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/aegis/ekf_path", 10);

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
      std::bind(&EkfNode::publishPose, this));

    path_msg_.header.frame_id = frame_id_;
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    last_odom_ = *msg;
  }

  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    last_imu_ = *msg;
  }

  void publishPose()
  {
    auto msg = geometry_msgs::msg::PoseStamped();
    msg.header.stamp = this->now();
    msg.header.frame_id = frame_id_;

    // Dummy pose from latest odometry if available.
    if (last_odom_.header.stamp.sec != 0) {
      msg.pose = last_odom_.pose.pose;
    }

    pose_pub_->publish(msg);

    path_msg_.header.stamp = msg.header.stamp;
    path_msg_.poses.push_back(msg);
    if (path_msg_.poses.size() > 200) {
      path_msg_.poses.erase(path_msg_.poses.begin());
    }
    path_pub_->publish(path_msg_);
  }

  std::string frame_id_;
  nav_msgs::msg::Odometry last_odom_;
  sensor_msgs::msg::Imu last_imu_;
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
  rclcpp::spin(std::make_shared<EkfNode>());
  rclcpp::shutdown();
  return 0;
}
