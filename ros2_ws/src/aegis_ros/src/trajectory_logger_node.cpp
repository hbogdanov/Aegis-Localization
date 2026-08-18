#include <cmath>
#include <chrono>
#include <cstddef>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "aegis_msgs/msg/filter_diagnostics.hpp"
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
    results_dir_ = this->declare_parameter<std::string>("results_dir", "results/metrics");
    use_odom_as_ground_truth_ = this->declare_parameter<bool>("use_odom_as_ground_truth", false);
    log_odom_baseline_ = this->declare_parameter<bool>("log_odom_baseline", false);
    stats_out_ = this->declare_parameter<std::string>("stats_out", "");

    const fs::path results_path = locateResultsPath(results_dir_);
    fs::create_directories(results_path);

    ekf_file_path_ = results_path / "ekf.csv";
    ukf_file_path_ = results_path / "ukf.csv";
    pf_file_path_ = results_path / "pf.csv";
    ground_truth_file_path_ = results_path / "ground_truth.csv";
    odom_file_path_ = results_path / "odom.csv";
    diagnostics_file_path_ = results_path / "filter_diagnostics.csv";

    openCsv(ekf_file_, ekf_file_path_);
    openCsv(ground_truth_file_, ground_truth_file_path_);
    openDiagnosticsCsv(diagnostics_file_, diagnostics_file_path_);
    if (log_odom_baseline_) {
      openCsv(odom_file_, odom_file_path_);
    }

    ekf_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/aegis/ekf_pose",
      1000,
      std::bind(&TrajectoryLoggerNode::ekfCallback, this, std::placeholders::_1));

    ukf_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/aegis/ukf_pose",
      1000,
      std::bind(&TrajectoryLoggerNode::ukfCallback, this, std::placeholders::_1));

    pf_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/aegis/pf_pose",
      1000,
      std::bind(&TrajectoryLoggerNode::pfCallback, this, std::placeholders::_1));

    ground_truth_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/ground_truth/pose",
      1000,
      std::bind(&TrajectoryLoggerNode::groundTruthCallback, this, std::placeholders::_1));

    diagnostics_sub_ = this->create_subscription<aegis_msgs::msg::FilterDiagnostics>(
      "/aegis/diagnostics",
      1000,
      std::bind(&TrajectoryLoggerNode::diagnosticsCallback, this, std::placeholders::_1));

    if (use_odom_as_ground_truth_ || log_odom_baseline_) {
      odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom",
        1000,
        std::bind(&TrajectoryLoggerNode::odomCallback, this, std::placeholders::_1));
    }
  }

  ~TrajectoryLoggerNode() override
  {
    RCLCPP_INFO(
      this->get_logger(),
      "Trajectory logger summary: gt=%zu ekf=%zu ukf=%zu pf=%zu results_dir=%s",
      ground_truth_count_,
      ekf_count_,
      ukf_count_,
      pf_count_,
      ground_truth_file_path_.parent_path().string().c_str());
    if (ekf_file_.is_open()) {
      ekf_file_.close();
    }
    if (ground_truth_file_.is_open()) {
      ground_truth_file_.close();
    }
    if (ukf_file_.is_open()) {
      ukf_file_.close();
    }
    if (pf_file_.is_open()) {
      pf_file_.close();
    }
    if (odom_file_.is_open()) {
      odom_file_.close();
    }
    if (diagnostics_file_.is_open()) {
      diagnostics_file_.close();
    }
    writeStats();
  }

private:
  static bool isRepoRoot(const fs::path &path)
  {
    return fs::exists(path / "README.md") &&
           fs::exists(path / "ros2_ws" / "src" / "aegis_core" / "package.xml");
  }

  static fs::path findRepoRoot(fs::path start)
  {
    while (!start.empty()) {
      if (isRepoRoot(start)) {
        return start;
      }
      const fs::path parent = start.parent_path();
      if (parent == start) {
        break;
      }
      start = parent;
    }

    return {};
  }

  static fs::path locateResultsPath(const std::string &results_dir)
  {
    const fs::path configured_path(results_dir);
    if (configured_path.is_absolute()) {
      return configured_path;
    }

    fs::path repo_root = findRepoRoot(fs::current_path());
    if (repo_root.empty()) {
      repo_root = findRepoRoot(ament_index_cpp::get_package_share_directory("aegis_ros"));
    }

    if (!repo_root.empty()) {
      return repo_root / configured_path;
    }

    return fs::current_path() / configured_path;
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

  static void openDiagnosticsCsv(std::ofstream &stream, const fs::path &path)
  {
    stream.open(path, std::ios::out | std::ios::trunc);
    if (!stream.is_open()) {
      throw std::runtime_error("Failed to open diagnostics CSV file: " + path.string());
    }
    stream << "source,timestamp,status,measurement_type,innovation_norm,innovation_dim,nis,innovation_vector,innovation_covariance,state_covariance\n";
    stream.flush();
  }

  static void maybeFlush(std::ofstream &stream, std::size_t count)
  {
    if (stream.is_open() && count > 0 && (count % 100) == 0) {
      stream.flush();
    }
  }

  static std::string serializeVector(const std::vector<double> &values)
  {
    std::ostringstream stream;
    stream << std::setprecision(17);
    for (std::size_t index = 0; index < values.size(); ++index) {
      if (index > 0) {
        stream << ';';
      }
      stream << values[index];
    }
    return stream.str();
  }

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
    handle << "  \"ground_truth_received\": " << ground_truth_count_ << ",\n";
    handle << "  \"ekf_received\": " << ekf_count_ << ",\n";
    handle << "  \"ukf_received\": " << ukf_count_ << ",\n";
    handle << "  \"pf_received\": " << pf_count_ << ",\n";
    handle << "  \"odom_received\": " << odom_count_ << ",\n";
    handle << "  \"diagnostics_received\": " << diagnostics_count_ << ",\n";
    handle << "  \"log_odom_baseline\": " << (log_odom_baseline_ ? "true" : "false") << ",\n";
    handle << "  \"use_odom_as_ground_truth\": " << (use_odom_as_ground_truth_ ? "true" : "false") << "\n";
    handle << "}\n";
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
    ++ekf_count_;
    maybeFlush(ekf_file_, ekf_count_);
    maybeWriteStats();
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
    ++ground_truth_count_;
    maybeFlush(ground_truth_file_, ground_truth_count_);
    maybeWriteStats();
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
    ++ukf_count_;
    maybeFlush(ukf_file_, ukf_count_);
    maybeWriteStats();
  }

  void pfCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    ensureCsvOpen(pf_file_, pf_file_path_);

    const double timestamp = toTimestamp(msg->header.stamp);
    const double x = msg->pose.position.x;
    const double y = msg->pose.position.y;
    const double yaw = quaternionToYaw(msg->pose.orientation);

    pf_file_ << std::fixed << std::setprecision(9)
             << timestamp << ','
             << x << ','
             << y << ','
             << yaw << '\n';
    ++pf_count_;
    maybeFlush(pf_file_, pf_count_);
    maybeWriteStats();
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    const double timestamp = toTimestamp(msg->header.stamp);
    const double x = msg->pose.pose.position.x;
    const double y = msg->pose.pose.position.y;
    const double yaw = quaternionToYaw(msg->pose.pose.orientation);

    if (use_odom_as_ground_truth_ && ground_truth_file_.is_open()) {
      ground_truth_file_ << std::fixed << std::setprecision(9)
                         << timestamp << ','
                         << x << ','
                         << y << ','
                         << yaw << '\n';
      ++ground_truth_count_;
    }

    if (log_odom_baseline_) {
      ensureCsvOpen(odom_file_, odom_file_path_);
      odom_file_ << std::fixed << std::setprecision(9)
                 << timestamp << ','
                 << x << ','
                 << y << ','
                 << yaw << '\n';
      ++odom_count_;
      maybeFlush(odom_file_, odom_count_);
      maybeWriteStats();
    }
  }

  void diagnosticsCallback(const aegis_msgs::msg::FilterDiagnostics::SharedPtr msg)
  {
    if (!diagnostics_file_.is_open()) {
      return;
    }

    diagnostics_file_ << std::fixed << std::setprecision(9)
                      << msg->source << ','
                      << msg->timestamp << ','
                      << msg->status << ','
                      << msg->measurement_type << ','
                      << msg->innovation_norm << ','
                      << msg->innovation_dim << ','
                      << msg->nis << ','
                      << serializeVector(msg->innovation_vector) << ','
                      << serializeVector(msg->innovation_covariance) << ','
                      << serializeVector(msg->state_covariance) << '\n';
    ++diagnostics_count_;
    maybeFlush(diagnostics_file_, diagnostics_count_);
    maybeWriteStats();
  }

  void maybeWriteStats() const
  {
    const std::size_t total =
      ground_truth_count_ + ekf_count_ + ukf_count_ + pf_count_ + odom_count_ + diagnostics_count_;
    if (total > 0 && (total % 1000) == 0) {
      writeStats();
    }
  }

  std::string frame_id_;
  std::string results_dir_;
  bool use_odom_as_ground_truth_;
  bool log_odom_baseline_ = false;
  std::string stats_out_;
  fs::path ekf_file_path_;
  fs::path ukf_file_path_;
  fs::path pf_file_path_;
  fs::path ground_truth_file_path_;
  fs::path odom_file_path_;
  fs::path diagnostics_file_path_;
  std::ofstream ekf_file_;
  std::ofstream ukf_file_;
  std::ofstream pf_file_;
  std::ofstream ground_truth_file_;
  std::ofstream odom_file_;
  std::ofstream diagnostics_file_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr ekf_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr ukf_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pf_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr ground_truth_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<aegis_msgs::msg::FilterDiagnostics>::SharedPtr diagnostics_sub_;
  std::size_t ekf_count_ = 0;
  std::size_t ukf_count_ = 0;
  std::size_t pf_count_ = 0;
  std::size_t ground_truth_count_ = 0;
  std::size_t odom_count_ = 0;
  std::size_t diagnostics_count_ = 0;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrajectoryLoggerNode>());
  rclcpp::shutdown();
  return 0;
}
