#pragma once

#include "aegis_core/state.hpp"
#include "aegis_core/utils.hpp"

#include <cstddef>
#include <random>
#include <vector>

namespace aegis_core {

struct Particle
{
  double px = 0.0;
  double py = 0.0;
  double theta = 0.0;
  double weight = 1.0;
};

class ParticleFilter
{
public:
  explicit ParticleFilter(std::size_t num_particles = 500);
  void setRandomSeed(std::uint32_t seed);

  void setNoiseParameters(
    double process_noise_x,
    double process_noise_y,
    double process_noise_theta,
    double measurement_noise_x,
    double measurement_noise_y,
    double measurement_noise_theta,
    double resample_threshold_ratio);
  void setMeasurementNoise(double measurement_noise_x, double measurement_noise_y, double measurement_noise_theta);

  void initialize(
    const State2D &mean,
    double std_x,
    double std_y,
    double std_theta);

  void predict(double dt, double vx, double vy, double omega);
  bool updatePose(double px, double py, double yaw);
  bool updateVelocityYawRate(double vx, double vy, double omega);

  void normalizeWeights();
  void systematicResample();
  double effectiveSampleSize() const;
  State2D estimateState() const;

  std::size_t particleCount() const noexcept;
  const std::vector<Particle> &particles() const noexcept;

private:
  double sampleNormal(double stddev);
  double gaussianLogLikelihood(double residual, double stddev) const;

  std::vector<Particle> particles_;
  std::mt19937 rng_;
  double process_noise_x_ = 0.02;
  double process_noise_y_ = 0.02;
  double process_noise_theta_ = 0.01;
  double measurement_noise_x_ = 0.05;
  double measurement_noise_y_ = 0.05;
  double measurement_noise_theta_ = 0.03;
  double resample_threshold_ratio_ = 0.5;
  double latest_vx_ = 0.0;
  double latest_vy_ = 0.0;
  double latest_omega_ = 0.0;
  bool initialized_ = false;
};

}  // namespace aegis_core
