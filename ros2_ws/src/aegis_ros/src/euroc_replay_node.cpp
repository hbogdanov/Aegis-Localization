#include <chrono>
#include <cmath>
#include <cstddef>
#include <fstream>
#include <optional>
#include <sstream>
#include <string>
#include <thread>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

using namespace std::chrono_literals;

namespace
{

struct ProxySample
{
  double timestamp = 0.0;
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
  double vx = 0.0;
  double vy = 0.0;
  double imu_omega_z = 0.0;
};

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

bool parseSample(const std::string & line, ProxySample & sample)
{
  std::stringstream line_stream(line);
  std::string field;
  double values[8];
  std::size_t value_index = 0;
  while (std::getline(line_stream, field, ',')) {
    if (value_index >= 8) {
      return false;
    }
    try {
      values[value_index++] = std::stod(field);
    } catch (const std::exception &) {
      return false;
    }
  }

  if (value_index < 8) {
    return false;
  }

  sample.timestamp = values[0];
  sample.x = values[1];
  sample.y = values[2];
  sample.yaw = values[3];
  sample.vx = values[4];
  sample.vy = values[5];
  sample.imu_omega_z = values[7];
  return true;
}

}  // namespace

class EurocReplayNode : public rclcpp::Node
{
public:
  EurocReplayNode()
  : Node("euroc_replay_node")
  {
    proxy_csv_path_ = this->declare_parameter<std::string>("proxy_csv_path", "");
    sleep_scale_ = this->declare_parameter<double>("sleep_scale", 30.0);
    max_samples_ = this->declare_parameter<int>("max_samples", -1);
    summary_out_ = this->declare_parameter<std::string>("summary_out", "");
    subscriber_wait_seconds_ = this->declare_parameter<double>("subscriber_wait_seconds", 10.0);
    subscriber_settle_seconds_ = this->declare_parameter<double>("subscriber_settle_seconds", 0.5);
    drain_wait_seconds_ = this->declare_parameter<double>("drain_wait_seconds", 2.0);
    expected_odom_subscribers_ = this->declare_parameter<int>("expected_odom_subscribers", 3);
    expected_imu_subscribers_ = this->declare_parameter<int>("expected_imu_subscribers", 3);
    expected_truth_subscribers_ = this->declare_parameter<int>("expected_truth_subscribers", 1);
    require_full_subscribers_ = this->declare_parameter<bool>("require_full_subscribers", true);

    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 1000);
    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu", 1000);
    truth_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/ground_truth/pose", 1000);
  }

  int run()
  {
    if (proxy_csv_path_.empty()) {
      RCLCPP_ERROR(this->get_logger(), "proxy_csv_path parameter is required");
      return 2;
    }

    waitForSubscribers();

    std::ifstream handle(proxy_csv_path_);
    if (!handle.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open proxy CSV: %s", proxy_csv_path_.c_str());
      return 2;
    }

    std::string line;
    std::getline(handle, line);  // skip header

    const auto started_at = std::chrono::steady_clock::now();
    std::optional<double> previous_timestamp;
    int sample_index = 0;

    while (std::getline(handle, line) && rclcpp::ok()) {
      if (max_samples_ >= 0 && sample_index >= max_samples_) {
        break;
      }

      ProxySample sample;
      if (!parseSample(line, sample)) {
        continue;
      }

      if (previous_timestamp.has_value() && sleep_scale_ > 0.0) {
        const double dt = std::max(0.0, sample.timestamp - *previous_timestamp);
        std::this_thread::sleep_for(std::chrono::duration<double>(dt / sleep_scale_));
      }
      previous_timestamp = sample.timestamp;

      publishSample(sample);
      ++sample_index;
    }

    const auto drain_until = std::chrono::steady_clock::now() + std::chrono::duration<double>(drain_wait_seconds_);
    while (rclcpp::ok() && std::chrono::steady_clock::now() < drain_until) {
      rclcpp::spin_some(shared_from_this());
      std::this_thread::sleep_for(20ms);
    }

    const auto elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - started_at).count();
    writeSummary(elapsed);
    RCLCPP_INFO(
      this->get_logger(),
      "EuRoC replay summary: published=%zu first=%.6f last=%.6f",
      published_samples_,
      first_timestamp_.value_or(-1.0),
      last_timestamp_.value_or(-1.0));
    return 0;
  }

private:
  void waitForSubscribers()
  {
    const auto start = std::chrono::steady_clock::now();
    while (rclcpp::ok()) {
      odom_subscribers_ = odom_pub_->get_subscription_count();
      imu_subscribers_ = imu_pub_->get_subscription_count();
      truth_subscribers_ = truth_pub_->get_subscription_count();
      if (
        odom_subscribers_ >= static_cast<std::size_t>(std::max(expected_odom_subscribers_, 0)) &&
        imu_subscribers_ >= static_cast<std::size_t>(std::max(expected_imu_subscribers_, 0)) &&
        truth_subscribers_ >= static_cast<std::size_t>(std::max(expected_truth_subscribers_, 0)))
      {
        RCLCPP_INFO(
          this->get_logger(),
          "Replay subscribers connected: odom=%zu imu=%zu truth=%zu",
          odom_subscribers_,
          imu_subscribers_,
          truth_subscribers_);
        const auto settle_until = std::chrono::steady_clock::now() + std::chrono::duration<double>(subscriber_settle_seconds_);
        while (rclcpp::ok() && std::chrono::steady_clock::now() < settle_until) {
          rclcpp::spin_some(shared_from_this());
          std::this_thread::sleep_for(20ms);
        }
        return;
      }
      if (std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count() >= subscriber_wait_seconds_) {
        const bool full_subscribers_ready =
          odom_subscribers_ >= static_cast<std::size_t>(std::max(expected_odom_subscribers_, 0)) &&
          imu_subscribers_ >= static_cast<std::size_t>(std::max(expected_imu_subscribers_, 0)) &&
          truth_subscribers_ >= static_cast<std::size_t>(std::max(expected_truth_subscribers_, 0));
        if (require_full_subscribers_ && !full_subscribers_ready) {
          std::ostringstream message;
          message
            << "Timed out waiting for full subscriber set; observed odom=" << odom_subscribers_
            << " imu=" << imu_subscribers_
            << " truth=" << truth_subscribers_
            << " expected odom>=" << expected_odom_subscribers_
            << " imu>=" << expected_imu_subscribers_
            << " truth>=" << expected_truth_subscribers_;
          throw std::runtime_error(message.str());
        }
        RCLCPP_WARN(
          this->get_logger(),
          "Timed out waiting for subscribers; continuing with odom=%zu imu=%zu truth=%zu",
          odom_subscribers_,
          imu_subscribers_,
          truth_subscribers_);
        return;
      }
      rclcpp::spin_some(shared_from_this());
      std::this_thread::sleep_for(100ms);
    }
  }

  void publishSample(const ProxySample & sample)
  {
    const rclcpp::Time stamp(static_cast<int64_t>(sample.timestamp * 1e9));
    const auto orientation = yawToQuaternion(sample.yaw);

    geometry_msgs::msg::PoseStamped truth;
    truth.header.stamp = stamp;
    truth.header.frame_id = "map";
    truth.pose.position.x = sample.x;
    truth.pose.position.y = sample.y;
    truth.pose.orientation = orientation;
    truth_pub_->publish(truth);

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";
    odom.pose.pose.position.x = sample.x;
    odom.pose.pose.position.y = sample.y;
    odom.pose.pose.orientation = orientation;
    odom.twist.twist.linear.x = sample.vx;
    odom.twist.twist.linear.y = sample.vy;
    odom.twist.twist.angular.z = sample.imu_omega_z;
    odom_pub_->publish(odom);

    sensor_msgs::msg::Imu imu;
    imu.header.stamp = stamp;
    imu.header.frame_id = "imu_link";
    imu.angular_velocity.z = sample.imu_omega_z;
    imu_pub_->publish(imu);

    if (!first_timestamp_.has_value()) {
      first_timestamp_ = sample.timestamp;
    }
    last_timestamp_ = sample.timestamp;
    ++published_samples_;

    rclcpp::spin_some(shared_from_this());
  }

  void writeSummary(double elapsed_seconds) const
  {
    if (summary_out_.empty()) {
      return;
    }

    std::ofstream handle(summary_out_);
    if (!handle.is_open()) {
      RCLCPP_WARN(this->get_logger(), "Failed to write replay summary to %s", summary_out_.c_str());
      return;
    }

    handle << "{\n";
    handle << "  \"published_samples\": " << published_samples_ << ",\n";
    handle << "  \"first_timestamp\": " << (first_timestamp_.has_value() ? std::to_string(*first_timestamp_) : "null") << ",\n";
    handle << "  \"last_timestamp\": " << (last_timestamp_.has_value() ? std::to_string(*last_timestamp_) : "null") << ",\n";
    handle << "  \"elapsed_wall_seconds\": " << elapsed_seconds << ",\n";
    handle << "  \"subscriber_snapshot\": {\n";
    handle << "    \"odom\": " << odom_subscribers_ << ",\n";
    handle << "    \"imu\": " << imu_subscribers_ << ",\n";
    handle << "    \"truth\": " << truth_subscribers_ << "\n";
    handle << "  }\n";
    handle << "}\n";
  }

  std::string proxy_csv_path_;
  double sleep_scale_ = 30.0;
  int max_samples_ = -1;
  std::string summary_out_;
  double subscriber_wait_seconds_ = 10.0;
  double subscriber_settle_seconds_ = 0.5;
  double drain_wait_seconds_ = 2.0;
  int expected_odom_subscribers_ = 3;
  int expected_imu_subscribers_ = 3;
  int expected_truth_subscribers_ = 1;
  bool require_full_subscribers_ = true;
  std::size_t published_samples_ = 0;
  std::size_t odom_subscribers_ = 0;
  std::size_t imu_subscribers_ = 0;
  std::size_t truth_subscribers_ = 0;
  std::optional<double> first_timestamp_;
  std::optional<double> last_timestamp_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr truth_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<EurocReplayNode>();
  const int exit_code = node->run();
  rclcpp::shutdown();
  return exit_code;
}
