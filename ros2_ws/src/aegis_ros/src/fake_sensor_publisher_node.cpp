#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
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
  , rng_(0)
  , warmup_ticks_remaining_(0)
  , shutdown_grace_ticks_remaining_(0)
  , sample_index_(0)
  {
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu", 10);
    truth_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/ground_truth/pose", 10);

    random_seed_ = static_cast<std::uint32_t>(this->declare_parameter<int64_t>("random_seed", 1337));
    stats_out_ = this->declare_parameter<std::string>("stats_out", "");
    corruption_log_out_ = this->declare_parameter<std::string>("corruption_log_out", "");
    radius_ = this->declare_parameter<double>("radius", 1.0);
    omega_ = this->declare_parameter<double>("omega", 0.5); // rad/s
    duration_seconds_ = this->declare_parameter<double>("duration_seconds", 30.0);
    startup_delay_seconds_ = this->declare_parameter<double>("startup_delay_seconds", 2.0);
    shutdown_grace_seconds_ = this->declare_parameter<double>("shutdown_grace_seconds", 0.25);
    odom_position_noise_std_ = this->declare_parameter<double>("odom_position_noise_std", 0.05);
    odom_velocity_noise_std_ = this->declare_parameter<double>("odom_velocity_noise_std", 0.02);
    imu_yaw_rate_noise_std_ = this->declare_parameter<double>("imu_yaw_rate_noise_std", 0.01);
    dropout_probability_ = this->declare_parameter<double>("dropout_probability", 0.05);
    pose_outlier_probability_ = this->declare_parameter<double>("pose_outlier_probability", 0.0);
    pose_outlier_position_std_ = this->declare_parameter<double>("pose_outlier_position_std", 1.5);
    pose_outlier_yaw_std_ = this->declare_parameter<double>("pose_outlier_yaw_std", 0.75);
    pose_outlier_start_seconds_ = this->declare_parameter<double>("pose_outlier_start_seconds", 0.0);
    rng_.seed(random_seed_);

    if (!corruption_log_out_.empty()) {
      corruption_log_.open(corruption_log_out_, std::ios::out | std::ios::trunc);
      if (corruption_log_.is_open()) {
        corruption_log_ << "timestamp,corrupted\n";
        corruption_log_.flush();
      }
    }

    start_time_ = this->now() + rclcpp::Duration::from_seconds(startup_delay_seconds_);
    max_samples_ = static_cast<std::size_t>(std::floor(duration_seconds_ / kSamplePeriodSeconds)) + 1U;
    warmup_ticks_remaining_ = static_cast<std::size_t>(std::ceil(startup_delay_seconds_ / kSamplePeriodSeconds));
    shutdown_grace_ticks_remaining_ = static_cast<std::size_t>(std::ceil(shutdown_grace_seconds_ / kSamplePeriodSeconds));
    timer_ = this->create_wall_timer(50ms, std::bind(&FakeSensorPublisher::timerCallback, this));
  }

  ~FakeSensorPublisher() override
  {
    if (corruption_log_.is_open()) {
      corruption_log_.close();
    }
    writeStats();
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
    handle << "  \"random_seed\": " << random_seed_ << ",\n";
    handle << "  \"published_truth_samples\": " << sample_index_ << ",\n";
    handle << "  \"dropped_measurements\": " << dropout_count_ << ",\n";
    handle << "  \"corrupted_pose_measurements\": " << pose_outlier_count_ << ",\n";
    handle << "  \"pose_outlier_probability\": " << pose_outlier_probability_ << ",\n";
    handle << "  \"pose_outlier_position_std\": " << pose_outlier_position_std_ << ",\n";
    handle << "  \"pose_outlier_yaw_std\": " << pose_outlier_yaw_std_ << "\n";
    handle << "}\n";
  }

  void timerCallback()
  {
    if (warmup_ticks_remaining_ > 0U) {
      --warmup_ticks_remaining_;
      return;
    }

    if (sample_index_ >= max_samples_) {
      if (shutdown_grace_ticks_remaining_ == 0U) {
        rclcpp::shutdown();
      } else {
        --shutdown_grace_ticks_remaining_;
      }
      return;
    }

    const double t = static_cast<double>(sample_index_) * kSamplePeriodSeconds;
    const rclcpp::Time stamp = start_time_ + rclcpp::Duration::from_seconds(t);

    double x = radius_ * std::cos(omega_ * t);
    double y = radius_ * std::sin(omega_ * t);
    double theta = omega_ * t + M_PI_2;

    double vx = -radius_ * omega_ * std::sin(omega_ * t);
    double vy = radius_ * omega_ * std::cos(omega_ * t);

    geometry_msgs::msg::PoseStamped truth;
    truth.header.stamp = stamp;
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
      ++dropout_count_;
    } else {
      nav_msgs::msg::Odometry odom;
      odom.header.stamp = stamp;
      odom.header.frame_id = "odom";
      odom.child_frame_id = "base_link";

      double measured_x = addNoise(x, odom_position_noise_std_);
      double measured_y = addNoise(y, odom_position_noise_std_);
      double measured_theta = theta;
      const bool outlier_window_open = t >= pose_outlier_start_seconds_;
      const bool pose_outlier = outlier_window_open && std::bernoulli_distribution(pose_outlier_probability_)(rng_);
      if (pose_outlier) {
        measured_x = addNoise(measured_x, pose_outlier_position_std_);
        measured_y = addNoise(measured_y, pose_outlier_position_std_);
        measured_theta = addNoise(measured_theta, pose_outlier_yaw_std_);
        ++pose_outlier_count_;
      }

      odom.pose.pose.position.x = measured_x;
      odom.pose.pose.position.y = measured_y;
      odom.pose.pose.position.z = 0.0;

      const double measured_half_yaw = measured_theta * 0.5;
      odom.pose.pose.orientation.x = 0.0;
      odom.pose.pose.orientation.y = 0.0;
      odom.pose.pose.orientation.z = std::sin(measured_half_yaw);
      odom.pose.pose.orientation.w = std::cos(measured_half_yaw);

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

      imu_pub_->publish(imu);
      odom_pub_->publish(odom);
      if (corruption_log_.is_open()) {
        corruption_log_ << std::fixed << std::setprecision(9)
                        << stamp.seconds() << ','
                        << (pose_outlier ? "true" : "false") << '\n';
      }
    }

    ++sample_index_;
  }

  static constexpr double kSamplePeriodSeconds = 0.05;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr truth_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time start_time_;
  std::uint32_t random_seed_;
  std::string stats_out_;
  std::string corruption_log_out_;
  std::size_t warmup_ticks_remaining_;
  std::size_t shutdown_grace_ticks_remaining_;
  std::size_t sample_index_;
  std::size_t max_samples_;
  std::size_t dropout_count_ = 0;
  std::size_t pose_outlier_count_ = 0;
  double radius_;
  double omega_;
  double duration_seconds_;
  double startup_delay_seconds_;
  double shutdown_grace_seconds_;
  double odom_position_noise_std_;
  double odom_velocity_noise_std_;
  double imu_yaw_rate_noise_std_;
  double dropout_probability_;
  double pose_outlier_probability_;
  double pose_outlier_position_std_;
  double pose_outlier_yaw_std_;
  double pose_outlier_start_seconds_;
  std::mt19937_64 rng_;
  std::ofstream corruption_log_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FakeSensorPublisher>());
  rclcpp::shutdown();
  return 0;
}
