#include <cmath>
#include <memory>
#include <string>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace
{

double quaternionToYaw(const geometry_msgs::msg::Quaternion &q)
{
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

geometry_msgs::msg::Quaternion yawToQuaternion(double yaw)
{
  geometry_msgs::msg::Quaternion q;
  const double half_yaw = yaw * 0.5;
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(half_yaw);
  q.w = std::cos(half_yaw);
  return q;
}

}  // namespace

class GazeboGroundTruthBridgeNode : public rclcpp::Node
{
public:
  GazeboGroundTruthBridgeNode()
  : Node("gazebo_ground_truth_bridge_node")
  {
    frame_id_ = this->declare_parameter<std::string>("frame_id", "map");
    ground_truth_odom_topic_ = this->declare_parameter<std::string>(
      "ground_truth_odom_topic", "/ground_truth/odom");

    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/ground_truth/pose", 10);
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      ground_truth_odom_topic_,
      10,
      std::bind(&GazeboGroundTruthBridgeNode::groundTruthOdomCallback, this, std::placeholders::_1));
  }

private:
  void groundTruthOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    geometry_msgs::msg::PoseStamped truth;
    truth.header.stamp = msg->header.stamp;
    truth.header.frame_id = frame_id_;
    truth.pose.position = msg->pose.pose.position;
    truth.pose.position.z = 0.0;
    truth.pose.orientation = yawToQuaternion(quaternionToYaw(msg->pose.pose.orientation));
    pose_pub_->publish(truth);

    ++published_count_;
    if (published_count_ == 1U) {
      RCLCPP_INFO(
        get_logger(),
        "Publishing Gazebo ground truth from odometry topic '%s'",
        ground_truth_odom_topic_.c_str());
    }
  }

  std::string frame_id_;
  std::string ground_truth_odom_topic_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  std::size_t published_count_ = 0;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GazeboGroundTruthBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
