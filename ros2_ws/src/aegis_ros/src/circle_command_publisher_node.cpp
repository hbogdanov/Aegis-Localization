#include <chrono>
#include <memory>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class CircleCommandPublisherNode : public rclcpp::Node
{
public:
  CircleCommandPublisherNode()
  : Node("circle_command_publisher_node")
  {
    linear_velocity_ = this->declare_parameter<double>("linear_velocity", 0.12);
    angular_velocity_ = this->declare_parameter<double>("angular_velocity", 0.35);

    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    timer_ = this->create_wall_timer(100ms, std::bind(&CircleCommandPublisherNode::publishCommand, this));
  }

private:
  void publishCommand()
  {
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = linear_velocity_;
    cmd.angular.z = angular_velocity_;
    cmd_pub_->publish(cmd);
  }

  double linear_velocity_ = 0.12;
  double angular_velocity_ = 0.35;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CircleCommandPublisherNode>());
  rclcpp::shutdown();
  return 0;
}
