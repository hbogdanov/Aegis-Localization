#include "aegis_core/particle_filter.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>

namespace aegis_core {

namespace
{

constexpr double kPi = 3.14159265358979323846;

}  // namespace

ParticleFilter::ParticleFilter(std::size_t num_particles)
  : particles_(num_particles),
    rng_(std::random_device{}())
{
  const double initial_weight = particles_.empty() ? 0.0 : 1.0 / static_cast<double>(particles_.size());
  for (auto &particle : particles_) {
    particle.weight = initial_weight;
  }
}

void ParticleFilter::setNoiseParameters(
  double process_noise_x,
  double process_noise_y,
  double process_noise_theta,
  double measurement_noise_x,
  double measurement_noise_y,
  double measurement_noise_theta,
  double resample_threshold_ratio)
{
  process_noise_x_ = process_noise_x;
  process_noise_y_ = process_noise_y;
  process_noise_theta_ = process_noise_theta;
  measurement_noise_x_ = measurement_noise_x;
  measurement_noise_y_ = measurement_noise_y;
  measurement_noise_theta_ = measurement_noise_theta;
  resample_threshold_ratio_ = std::clamp(resample_threshold_ratio, 0.0, 1.0);
}

void ParticleFilter::initialize(
  const State2D &mean,
  double std_x,
  double std_y,
  double std_theta)
{
  latest_vx_ = mean.vx;
  latest_vy_ = mean.vy;
  latest_omega_ = mean.omega;

  const double uniform_weight = particles_.empty() ? 0.0 : 1.0 / static_cast<double>(particles_.size());
  for (auto &particle : particles_) {
    particle.px = mean.px + sampleNormal(std_x);
    particle.py = mean.py + sampleNormal(std_y);
    particle.theta = normalizeAngle(mean.theta + sampleNormal(std_theta));
    particle.weight = uniform_weight;
  }

  initialized_ = true;
}

void ParticleFilter::predict(double dt, double vx, double vy, double omega)
{
  if (!initialized_) {
    return;
  }

  latest_vx_ = vx;
  latest_vy_ = vy;
  latest_omega_ = omega;

  for (auto &particle : particles_) {
    particle.px += vx * dt + sampleNormal(process_noise_x_);
    particle.py += vy * dt + sampleNormal(process_noise_y_);
    particle.theta = normalizeAngle(particle.theta + omega * dt + sampleNormal(process_noise_theta_));
  }
}

bool ParticleFilter::updatePose(double px, double py, double yaw)
{
  if (!initialized_ || particles_.empty()) {
    return false;
  }

  std::vector<double> log_weights;
  log_weights.reserve(particles_.size());
  double max_log_weight = -std::numeric_limits<double>::infinity();

  for (const auto &particle : particles_) {
    const double dx = px - particle.px;
    const double dy = py - particle.py;
    const double dtheta = normalizeAngle(yaw - particle.theta);
    const double log_weight =
      gaussianLogLikelihood(dx, measurement_noise_x_) +
      gaussianLogLikelihood(dy, measurement_noise_y_) +
      gaussianLogLikelihood(dtheta, measurement_noise_theta_);
    log_weights.push_back(log_weight);
    max_log_weight = std::max(max_log_weight, log_weight);
  }

  double weight_sum = 0.0;
  for (std::size_t i = 0; i < particles_.size(); ++i) {
    particles_[i].weight = std::exp(log_weights[i] - max_log_weight);
    weight_sum += particles_[i].weight;
  }

  if (weight_sum <= std::numeric_limits<double>::epsilon()) {
    const double uniform_weight = 1.0 / static_cast<double>(particles_.size());
    for (auto &particle : particles_) {
      particle.weight = uniform_weight;
    }
  } else {
    normalizeWeights();
  }

  if (effectiveSampleSize() < resample_threshold_ratio_ * static_cast<double>(particles_.size())) {
    systematicResample();
  }

  return true;
}

bool ParticleFilter::updateVelocityYawRate(double vx, double vy, double omega)
{
  latest_vx_ = vx;
  latest_vy_ = vy;
  latest_omega_ = omega;
  return initialized_;
}

void ParticleFilter::normalizeWeights()
{
  if (particles_.empty()) {
    return;
  }

  const double weight_sum = std::accumulate(
    particles_.begin(),
    particles_.end(),
    0.0,
    [](double total, const Particle &particle) {
      return total + particle.weight;
    });

  if (weight_sum <= std::numeric_limits<double>::epsilon()) {
    const double uniform_weight = 1.0 / static_cast<double>(particles_.size());
    for (auto &particle : particles_) {
      particle.weight = uniform_weight;
    }
    return;
  }

  for (auto &particle : particles_) {
    particle.weight /= weight_sum;
  }
}

void ParticleFilter::systematicResample()
{
  if (particles_.empty()) {
    return;
  }

  normalizeWeights();

  std::vector<Particle> resampled_particles;
  resampled_particles.reserve(particles_.size());

  const double step = 1.0 / static_cast<double>(particles_.size());
  std::uniform_real_distribution<double> dist(0.0, step);
  double target = dist(rng_);
  double cumulative_weight = particles_.front().weight;
  std::size_t index = 0;

  for (std::size_t i = 0; i < particles_.size(); ++i) {
    while (target > cumulative_weight && index + 1 < particles_.size()) {
      ++index;
      cumulative_weight += particles_[index].weight;
    }
    Particle particle = particles_[index];
    particle.weight = step;
    resampled_particles.push_back(particle);
    target += step;
  }

  particles_ = std::move(resampled_particles);
}

double ParticleFilter::effectiveSampleSize() const
{
  if (particles_.empty()) {
    return 0.0;
  }

  double squared_weight_sum = 0.0;
  for (const auto &particle : particles_) {
    squared_weight_sum += particle.weight * particle.weight;
  }

  if (squared_weight_sum <= std::numeric_limits<double>::epsilon()) {
    return 0.0;
  }

  return 1.0 / squared_weight_sum;
}

State2D ParticleFilter::estimateState() const
{
  State2D estimate;
  if (particles_.empty()) {
    return estimate;
  }

  double sin_sum = 0.0;
  double cos_sum = 0.0;
  for (const auto &particle : particles_) {
    estimate.px += particle.weight * particle.px;
    estimate.py += particle.weight * particle.py;
    sin_sum += particle.weight * std::sin(particle.theta);
    cos_sum += particle.weight * std::cos(particle.theta);
  }

  estimate.theta = std::atan2(sin_sum, cos_sum);
  estimate.vx = latest_vx_;
  estimate.vy = latest_vy_;
  estimate.omega = latest_omega_;
  return estimate;
}

std::size_t ParticleFilter::particleCount() const noexcept
{
  return particles_.size();
}

const std::vector<Particle> &ParticleFilter::particles() const noexcept
{
  return particles_;
}

double ParticleFilter::sampleNormal(double stddev)
{
  if (stddev <= 0.0) {
    return 0.0;
  }

  std::normal_distribution<double> dist(0.0, stddev);
  return dist(rng_);
}

double ParticleFilter::gaussianLogLikelihood(double residual, double stddev) const
{
  const double clamped_stddev = std::max(stddev, 1e-9);
  const double variance = clamped_stddev * clamped_stddev;
  return -0.5 * ((residual * residual) / variance + std::log(2.0 * kPi * variance));
}

}  // namespace aegis_core
