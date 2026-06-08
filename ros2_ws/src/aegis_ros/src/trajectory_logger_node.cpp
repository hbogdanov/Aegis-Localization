#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <string>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

namespace fs = std::filesystem;

class TrajectoryLoggerNode : public rclcpp::Node
{
public:
  TrajectoryLoggerNode()
  : Node("trajectory_logger_node")
  {
    frame_id_ = this->declare_parameter<std::string>("frame_id", "map");
    use_odom_as_ground_truth_ = this->declare_parameter<bool>("use_odom_as_ground_truth", false);

    const fs::path results_path = locateResultsPath();
    fs::create_directories(results_path);

    ekf_file_path_ = results_path / "ekf.csv";
    ukf_file_path_ = results_path / "ukf.csv";
    ground_truth_file_path_ = results_path / "ground_truth.csv";

    openCsv(ekf_file_, ekf_file_path_);
    openCsv(ground_truth_file_, ground_truth_file_path_);

    ekf_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/aegis/ekf_pose",
      10,
      std::bind(&TrajectoryLoggerNode::ekfCallback, this, std::placeholders::_1));

    ukf_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/aegis/ukf_pose",
      10,
      std::bind(&TrajectoryLoggerNode::ukfCallback, this, std::placeholders::_1));

    ground_truth_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/ground_truth/pose",
      10,
      std::bind(&TrajectoryLoggerNode::groundTruthCallback, this, std::placeholders::_1));

    if (use_odom_as_ground_truth_) {
      odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom",
        10,
        std::bind(&TrajectoryLoggerNode::odomCallback, this, std::placeholders::_1));
    }
  }

  ~TrajectoryLoggerNode() override
  {
    if (ekf_file_.is_open()) {
      ekf_file_.close();
    }
    if (ground_truth_file_.is_open()) {
      ground_truth_file_.close();
    }
    if (ukf_file_.is_open()) {
      ukf_file_.close();
    }
  }

private:
  static fs::path locateResultsPath()
  {
    fs::path share_dir = ament_index_cpp::get_package_share_directory("aegis_ros");
    fs::path repo_root = share_dir;
    for (int i = 0; i < 5 && !repo_root.empty(); ++i) {
      repo_root = repo_root.parent_path();
    }
    fs::path metrics_dir = repo_root / "results" / "metrics";
    if (!metrics_dir.empty()) {
      return metrics_dir;
    }
    return fs::current_path() / "results" / "metrics";
  }

  static void openCsv(std::ofstream &stream, const fs::path &path)
  {
    stream.open(path, std::ios::out | std::ios::trunc);
    if (!stream.is_open()) {
      throw std::runtime_error("Failed to open CSV file: " + path.string());
    }
    stream << "timestamp,x,y,yaw\n";
    stream.flush();
  }

  static void ensureCsvOpen(std::ofstream &stream, const fs::path &path)
  {
    if (!stream.is_open()) {
      openCsv(stream, path);
    }
  }

  static double toTimestamp(const rclcpp::Time &stamp)
  {
    return static_cast<double>(stamp.seconds());
  }

  static double quaternionToYaw(const geometry_msgs::msg::Quaternion &q)
  {
    const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
  }

  void ekfCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    if (!ekf_file_.is_open()) {
      return;
    }

    const double timestamp = toTimestamp(msg->header.stamp);
    const double x = msg->pose.position.x;
    const double y = msg->pose.position.y;
    const double yaw = quaternionToYaw(msg->pose.orientation);

    ekf_file_ << std::fixed << std::setprecision(9)
              << timestamp << ','
              << x << ','
              << y << ','
              << yaw << '\n';
    ekf_file_.flush();
  }

  void groundTruthCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    if (!ground_truth_file_.is_open()) {
      return;
    }

    const double timestamp = toTimestamp(msg->header.stamp);
    const double x = msg->pose.position.x;
    const double y = msg->pose.position.y;
    const double yaw = quaternionToYaw(msg->pose.orientation);

    ground_truth_file_ << std::fixed << std::setprecision(9)
                       << timestamp << ','
                       << x << ','
                       << y << ','
                       << yaw << '\n';
    ground_truth_file_.flush();
  }

  void ukfCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    ensureCsvOpen(ukf_file_, ukf_file_path_);

    const double timestamp = toTimestamp(msg->header.stamp);
    const double x = msg->pose.position.x;
    const double y = msg->pose.position.y;
    const double yaw = quaternionToYaw(msg->pose.orientation);

    ukf_file_ << std::fixed << std::setprecision(9)
              << timestamp << ','
              << x << ','
              << y << ','
              << yaw << '\n';
    ukf_file_.flush();
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    if (!ground_truth_file_.is_open()) {
      return;
    }

    const double timestamp = toTimestamp(msg->header.stamp);
    const double x = msg->pose.pose.position.x;
    const double y = msg->pose.pose.position.y;
    const double yaw = quaternionToYaw(msg->pose.pose.orientation);

    ground_truth_file_ << std::fixed << std::setprecision(9)
                       << timestamp << ','
                       << x << ','
                       << y << ','
                       << yaw << '\n';
    ground_truth_file_.flush();
  }

  std::string frame_id_;
  bool use_odom_as_ground_truth_;
  fs::path ekf_file_path_;
  fs::path ukf_file_path_;
  fs::path ground_truth_file_path_;
  std::ofstream ekf_file_;
  std::ofstream ukf_file_;
  std::ofstream ground_truth_file_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr ekf_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr ukf_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr ground_truth_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrajectoryLoggerNode>());
  rclcpp::shutdown();
  return 0;
}
