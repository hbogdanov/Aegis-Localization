#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <memory>
#include <random>
#include <deque>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

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
    correction_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/pose_correction", 10);

    random_seed_ = static_cast<std::uint32_t>(this->declare_parameter<int64_t>("random_seed", 1337));
    stats_out_ = this->declare_parameter<std::string>("stats_out", "");
    corruption_log_out_ = this->declare_parameter<std::string>("corruption_log_out", "");
    correction_log_out_ = this->declare_parameter<std::string>("correction_log_out", "");
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
    correction_enabled_ = this->declare_parameter<bool>("correction_enabled", false);
    correction_start_seconds_ = this->declare_parameter<double>("correction_start_seconds", 0.0);
    correction_frequency_hz_ = this->declare_parameter<double>("correction_frequency_hz", 2.0);
    correction_dropout_probability_ = this->declare_parameter<double>("correction_dropout_probability", 0.0);
    correction_latency_seconds_ = this->declare_parameter<double>("correction_latency_seconds", 0.0);
    correction_latency_schedule_seconds_ = parseLatencySchedule(
      this->declare_parameter<std::string>("correction_latency_schedule_seconds", ""));
    correction_max_emissions_ = static_cast<std::size_t>(
      std::max<int64_t>(0, this->declare_parameter<int64_t>("correction_max_emissions", 0)));
    correction_position_noise_std_ = this->declare_parameter<double>("correction_position_noise_std", 0.05);
    correction_yaw_noise_std_ = this->declare_parameter<double>("correction_yaw_noise_std", 0.08);
    correction_outlier_probability_ = this->declare_parameter<double>("correction_outlier_probability", 0.0);
    correction_outlier_position_std_ = this->declare_parameter<double>("correction_outlier_position_std", 1.5);
    correction_outlier_yaw_std_ = this->declare_parameter<double>("correction_outlier_yaw_std", 0.75);
    rng_.seed(random_seed_);

    if (!corruption_log_out_.empty()) {
      corruption_log_.open(corruption_log_out_, std::ios::out | std::ios::trunc);
      if (corruption_log_.is_open()) {
        corruption_log_ << "timestamp,corrupted\n";
        corruption_log_.flush();
      }
    }

    if (!correction_log_out_.empty()) {
      correction_log_.open(correction_log_out_, std::ios::out | std::ios::trunc);
      if (correction_log_.is_open()) {
        correction_log_ << "measurement_timestamp,publish_timestamp,dropped,corrupted\n";
        correction_log_.flush();
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
    if (correction_log_.is_open()) {
      correction_log_.close();
    }
    writeStats();
  }

private:
  struct PendingCorrection
  {
    std::size_t release_index = 0U;
    double publish_timestamp = 0.0;
    bool dropped = false;
    bool corrupted = false;
    geometry_msgs::msg::PoseWithCovarianceStamped msg;
  };

  static std::vector<double> parseLatencySchedule(const std::string &raw)
  {
    std::vector<double> schedule;
    std::string sanitized;
    sanitized.reserve(raw.size());
    for (char ch : raw) {
      if (ch != '[' && ch != ']' && ch != ' ') {
        sanitized.push_back(ch);
      }
    }
    if (sanitized.empty()) {
      return schedule;
    }

    std::stringstream stream(sanitized);
    std::string token;
    while (std::getline(stream, token, ',')) {
      if (token.empty()) {
        continue;
      }
      schedule.push_back(std::stod(token));
    }
    return schedule;
  }

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
    handle << "  \"published_corrections\": " << published_correction_count_ << ",\n";
    handle << "  \"dropped_corrections\": " << dropped_correction_count_ << ",\n";
    handle << "  \"corrupted_corrections\": " << corrupted_correction_count_ << ",\n";
    handle << "  \"enqueued_corrections\": " << enqueued_correction_count_ << ",\n";
    handle << "  \"correction_frequency_hz\": " << correction_frequency_hz_ << ",\n";
    handle << "  \"correction_latency_seconds\": " << correction_latency_seconds_ << ",\n";
    handle << "  \"pose_outlier_probability\": " << pose_outlier_probability_ << ",\n";
    handle << "  \"pose_outlier_position_std\": " << pose_outlier_position_std_ << ",\n";
    handle << "  \"pose_outlier_yaw_std\": " << pose_outlier_yaw_std_ << "\n";
    handle << "}\n";
  }

  bool shouldEmitCorrection(std::size_t sample_index) const
  {
    if (!correction_enabled_ || correction_frequency_hz_ <= 0.0) {
      return false;
    }
    if ((static_cast<double>(sample_index) * kSamplePeriodSeconds) + 1e-9 < correction_start_seconds_) {
      return false;
    }
    if (correction_max_emissions_ > 0U && enqueued_correction_count_ >= correction_max_emissions_) {
      return false;
    }

    const double correction_period = 1.0 / correction_frequency_hz_;
    const std::size_t correction_step = std::max<std::size_t>(
      1U,
      static_cast<std::size_t>(std::llround(correction_period / kSamplePeriodSeconds)));
    return (sample_index % correction_step) == 0U;
  }

  void enqueueCorrection(
    const rclcpp::Time &measurement_stamp,
    double x,
    double y,
    double theta)
  {
    const std::size_t correction_index = enqueued_correction_count_;
    const bool dropped = std::bernoulli_distribution(correction_dropout_probability_)(rng_);
    const bool corrupted = !dropped && std::bernoulli_distribution(correction_outlier_probability_)(rng_);

    double measured_x = addNoise(x, correction_position_noise_std_);
    double measured_y = addNoise(y, correction_position_noise_std_);
    double measured_theta = addNoise(theta, correction_yaw_noise_std_);
    if (corrupted) {
      measured_x = addNoise(measured_x, correction_outlier_position_std_);
      measured_y = addNoise(measured_y, correction_outlier_position_std_);
      measured_theta = addNoise(measured_theta, correction_outlier_yaw_std_);
      ++corrupted_correction_count_;
    }

    geometry_msgs::msg::PoseWithCovarianceStamped correction;
    correction.header.stamp = measurement_stamp;
    correction.header.frame_id = "odom";
    correction.pose.pose.position.x = measured_x;
    correction.pose.pose.position.y = measured_y;
    correction.pose.pose.position.z = 0.0;
    const double half_yaw = measured_theta * 0.5;
    correction.pose.pose.orientation.x = 0.0;
    correction.pose.pose.orientation.y = 0.0;
    correction.pose.pose.orientation.z = std::sin(half_yaw);
    correction.pose.pose.orientation.w = std::cos(half_yaw);
    correction.pose.covariance[0] = correction_position_noise_std_ * correction_position_noise_std_;
    correction.pose.covariance[7] = correction_position_noise_std_ * correction_position_noise_std_;
    correction.pose.covariance[35] = correction_yaw_noise_std_ * correction_yaw_noise_std_;

    double latency_seconds = correction_latency_seconds_;
    if (correction_index < correction_latency_schedule_seconds_.size()) {
      latency_seconds = correction_latency_schedule_seconds_[correction_index];
    }
    const long long latency_steps_raw = std::llround(latency_seconds / kSamplePeriodSeconds);
    const std::size_t latency_steps = static_cast<std::size_t>(std::max<long long>(0LL, latency_steps_raw));
    const std::size_t release_index = sample_index_ + latency_steps;
    const double publish_timestamp = measurement_stamp.seconds() + latency_seconds;
    pending_corrections_.push_back(PendingCorrection{
      release_index,
      publish_timestamp,
      dropped,
      corrupted,
      correction,
    });
    ++enqueued_correction_count_;
  }

  void publishDueCorrections()
  {
    while (!pending_corrections_.empty() && pending_corrections_.front().release_index <= sample_index_) {
      const PendingCorrection pending = pending_corrections_.front();
      pending_corrections_.pop_front();

      if (pending.dropped) {
        ++dropped_correction_count_;
      } else {
        correction_pub_->publish(pending.msg);
        ++published_correction_count_;
      }

      if (correction_log_.is_open()) {
        const rclcpp::Time measurement_stamp(pending.msg.header.stamp);
        correction_log_ << std::fixed << std::setprecision(9)
                        << measurement_stamp.seconds() << ','
                        << pending.publish_timestamp << ','
                        << (pending.dropped ? "true" : "false") << ','
                        << (pending.corrupted ? "true" : "false") << '\n';
      }
    }
  }

  void timerCallback()
  {
    publishDueCorrections();

    if (warmup_ticks_remaining_ > 0U) {
      --warmup_ticks_remaining_;
      return;
    }

    if (sample_index_ >= max_samples_) {
      if (shutdown_grace_ticks_remaining_ == 0U && pending_corrections_.empty()) {
        rclcpp::shutdown();
      } else {
        --shutdown_grace_ticks_remaining_;
        ++sample_index_;
        publishDueCorrections();
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

    if (shouldEmitCorrection(sample_index_)) {
      enqueueCorrection(stamp, x, y, theta);
    }

    ++sample_index_;
    publishDueCorrections();
  }

  static constexpr double kSamplePeriodSeconds = 0.05;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr truth_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr correction_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time start_time_;
  std::uint32_t random_seed_;
  std::string stats_out_;
  std::string corruption_log_out_;
  std::string correction_log_out_;
  std::size_t warmup_ticks_remaining_;
  std::size_t shutdown_grace_ticks_remaining_;
  std::size_t sample_index_;
  std::size_t max_samples_;
  std::size_t dropout_count_ = 0;
  std::size_t pose_outlier_count_ = 0;
  std::size_t published_correction_count_ = 0;
  std::size_t dropped_correction_count_ = 0;
  std::size_t corrupted_correction_count_ = 0;
  std::size_t enqueued_correction_count_ = 0;
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
  bool correction_enabled_ = false;
  double correction_start_seconds_ = 0.0;
  double correction_frequency_hz_ = 2.0;
  double correction_dropout_probability_ = 0.0;
  double correction_latency_seconds_ = 0.0;
  std::vector<double> correction_latency_schedule_seconds_;
  std::size_t correction_max_emissions_ = 0U;
  double correction_position_noise_std_ = 0.05;
  double correction_yaw_noise_std_ = 0.08;
  double correction_outlier_probability_ = 0.0;
  double correction_outlier_position_std_ = 1.5;
  double correction_outlier_yaw_std_ = 0.75;
  std::mt19937_64 rng_;
  std::ofstream corruption_log_;
  std::ofstream correction_log_;
  std::deque<PendingCorrection> pending_corrections_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FakeSensorPublisher>());
  rclcpp::shutdown();
  return 0;
}
