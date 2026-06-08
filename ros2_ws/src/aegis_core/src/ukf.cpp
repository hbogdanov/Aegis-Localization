#include "aegis_core/ukf.hpp"

#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace aegis_core {

namespace
{

template<int VectorSize, int SigmaPointCount>
Eigen::Matrix<double, VectorSize, 1> computeWeightedMean(
  const Eigen::Matrix<double, VectorSize, SigmaPointCount> &sigma_points,
  const Eigen::Matrix<double, SigmaPointCount, 1> &weights,
  const std::vector<int> &angle_indices)
{
  Eigen::Matrix<double, VectorSize, 1> mean = Eigen::Matrix<double, VectorSize, 1>::Zero();
  std::array<bool, VectorSize> is_angle{};
  for (const int index : angle_indices) {
    if (index >= 0 && index < VectorSize) {
      is_angle[static_cast<size_t>(index)] = true;
    }
  }

  for (int row = 0; row < VectorSize; ++row) {
    if (is_angle[static_cast<size_t>(row)]) {
      double sin_sum = 0.0;
      double cos_sum = 0.0;
      for (int col = 0; col < SigmaPointCount; ++col) {
        sin_sum += weights(col) * std::sin(sigma_points(row, col));
        cos_sum += weights(col) * std::cos(sigma_points(row, col));
      }
      mean(row) = std::atan2(sin_sum, cos_sum);
    } else {
      for (int col = 0; col < SigmaPointCount; ++col) {
        mean(row) += weights(col) * sigma_points(row, col);
      }
    }
  }

  return mean;
}

template<int VectorSize>
Eigen::Matrix<double, VectorSize, 1> computeResidual(
  const Eigen::Matrix<double, VectorSize, 1> &value,
  const Eigen::Matrix<double, VectorSize, 1> &reference,
  const std::vector<int> &angle_indices)
{
  Eigen::Matrix<double, VectorSize, 1> difference = value - reference;
  for (const int index : angle_indices) {
    if (index >= 0 && index < VectorSize) {
      difference(index) = normalizeAngle(difference(index));
    }
  }
  return difference;
}

}  // namespace

UKF::UKF()
  : state_(),
    process_noise_(Matrix6d::Identity() * 1e-3),
    velocity_yaw_rate_noise_(Matrix3d::Identity() * 1e-2),
    pose_noise_(Matrix3d::Identity() * 1e-2),
    alpha_(1e-1),
    beta_(2.0),
    kappa_(0.0),
    lambda_(0.0),
    weights_mean_(WeightVector::Zero()),
    weights_covariance_(WeightVector::Zero())
{
  computeWeights();
}

UKF::UKF(const State2D &initial_state)
  : UKF()
{
  state_ = initial_state;
}

void UKF::setState(const State2D &initial_state)
{
  state_ = initial_state;
}

const State2D &UKF::state() const noexcept
{
  return state_;
}

void UKF::setProcessNoise(const Matrix6d &Q)
{
  process_noise_ = Q;
}

void UKF::setVelocityYawRateNoise(const Matrix3d &R)
{
  velocity_yaw_rate_noise_ = R;
}

void UKF::setPoseNoise(const Matrix3d &R)
{
  pose_noise_ = R;
}

void UKF::setSigmaPointParameters(double alpha, double beta, double kappa)
{
  alpha_ = alpha;
  beta_ = beta;
  kappa_ = kappa;
  computeWeights();
}

void UKF::computeWeights()
{
  const double scaling = alpha_ * alpha_ * (kStateSize + kappa_);
  lambda_ = scaling - kStateSize;
  const double denominator = kStateSize + lambda_;
  if (denominator <= 0.0) {
    throw std::invalid_argument("UKF sigma point parameters produce a non-positive scaling term");
  }

  weights_mean_.setConstant(1.0 / (2.0 * denominator));
  weights_covariance_.setConstant(1.0 / (2.0 * denominator));
  weights_mean_(0) = lambda_ / denominator;
  weights_covariance_(0) = weights_mean_(0) + (1.0 - alpha_ * alpha_ + beta_);
}

Eigen::LLT<Matrix6d> UKF::computeCovarianceFactor(const Matrix6d &covariance) const
{
  Matrix6d adjusted = covariance;
  double jitter = 1e-9;

  for (int attempt = 0; attempt < 8; ++attempt) {
    Eigen::LLT<Matrix6d> llt(adjusted);
    if (llt.info() == Eigen::Success) {
      return llt;
    }
    adjusted += Matrix6d::Identity() * jitter;
    jitter *= 10.0;
  }

  throw std::runtime_error("UKF covariance is not positive definite");
}

UKF::SigmaPointMatrix UKF::generateSigmaPoints() const
{
  const Eigen::LLT<Matrix6d> llt = computeCovarianceFactor(state_.covariance);
  const Matrix6d factor = llt.matrixL().toDenseMatrix() * std::sqrt(kStateSize + lambda_);

  SigmaPointMatrix sigma_points;
  const Vector6d mean = state_.toVector();
  sigma_points.col(0) = mean;
  for (int i = 0; i < kStateSize; ++i) {
    sigma_points.col(i + 1) = mean + factor.col(i);
    sigma_points.col(i + 1 + kStateSize) = mean - factor.col(i);
  }
  return sigma_points;
}

void UKF::updateStateFromVector(const Vector6d &mean, const Matrix6d &covariance)
{
  state_.fromVector(mean);
  state_.theta = normalizeAngle(state_.theta);
  state_.covariance = covariance;
}

void UKF::predict(double dt)
{
  const SigmaPointMatrix sigma_points = generateSigmaPoints();

  SigmaPointMatrix predicted_sigma_points;
  for (int i = 0; i < kSigmaPointCount; ++i) {
    predicted_sigma_points.col(i) = MotionModel::propagate(sigma_points.col(i), dt);
    predicted_sigma_points(2, i) = normalizeAngle(predicted_sigma_points(2, i));
  }

  const Vector6d predicted_mean = weightedMean(predicted_sigma_points, {2});
  Matrix6d predicted_covariance = process_noise_;
  for (int i = 0; i < kSigmaPointCount; ++i) {
    const Vector6d sigma_point = predicted_sigma_points.col(i);
    const Vector6d diff = residual(sigma_point, predicted_mean, {2});
    predicted_covariance += weights_covariance_(i) * diff * diff.transpose();
  }

  updateStateFromVector(predicted_mean, predicted_covariance);
}

bool UKF::updateVelocityYawRate(double vx, double vy, double omega)
{
  const SigmaPointMatrix sigma_points = generateSigmaPoints();
  MeasurementMatrix<3> measurement_sigma_points;
  for (int i = 0; i < kSigmaPointCount; ++i) {
    measurement_sigma_points.col(i) << sigma_points(3, i), sigma_points(4, i), sigma_points(5, i);
  }

  Eigen::Vector3d measurement;
  measurement << vx, vy, omega;
  return update<3>(measurement, velocity_yaw_rate_noise_, measurement_sigma_points, {});
}

bool UKF::updatePose(double px, double py, double theta)
{
  const SigmaPointMatrix sigma_points = generateSigmaPoints();
  MeasurementMatrix<3> measurement_sigma_points;
  for (int i = 0; i < kSigmaPointCount; ++i) {
    measurement_sigma_points.col(i) << sigma_points(0, i), sigma_points(1, i), sigma_points(2, i);
    measurement_sigma_points(2, i) = normalizeAngle(measurement_sigma_points(2, i));
  }

  Eigen::Vector3d measurement;
  measurement << px, py, normalizeAngle(theta);
  return update<3>(measurement, pose_noise_, measurement_sigma_points, {2});
}

template<int MeasurementSize>
bool UKF::update(
  const Eigen::Matrix<double, MeasurementSize, 1> &measurement,
  const Eigen::Matrix<double, MeasurementSize, MeasurementSize> &noise,
  const MeasurementMatrix<MeasurementSize> &measurement_sigma_points,
  const std::vector<int> &angle_indices)
{
  const SigmaPointMatrix sigma_points = generateSigmaPoints();
  const Vector6d state_mean = state_.toVector();
  const Eigen::Matrix<double, MeasurementSize, 1> measurement_mean =
    weightedMean(measurement_sigma_points, angle_indices);

  Eigen::Matrix<double, MeasurementSize, MeasurementSize> measurement_covariance = noise;
  Eigen::Matrix<double, kStateSize, MeasurementSize> cross_covariance =
    Eigen::Matrix<double, kStateSize, MeasurementSize>::Zero();

  for (int i = 0; i < kSigmaPointCount; ++i) {
    const Vector6d state_sigma_point = sigma_points.col(i);
    const Vector6d state_diff = residual(state_sigma_point, state_mean, {2});
    const Eigen::Matrix<double, MeasurementSize, 1> measurement_sigma_point =
      measurement_sigma_points.col(i);
    const Eigen::Matrix<double, MeasurementSize, 1> measurement_diff =
      residual(measurement_sigma_point, measurement_mean, angle_indices);

    measurement_covariance += weights_covariance_(i) * measurement_diff * measurement_diff.transpose();
    cross_covariance += weights_covariance_(i) * state_diff * measurement_diff.transpose();
  }

  const Eigen::Matrix<double, MeasurementSize, 1> innovation =
    residual(measurement, measurement_mean, angle_indices);
  const Eigen::Matrix<double, kStateSize, MeasurementSize> kalman_gain =
    cross_covariance * measurement_covariance.inverse();
  const Vector6d updated_state = state_mean + kalman_gain * innovation;
  Matrix6d updated_covariance =
    state_.covariance - kalman_gain * measurement_covariance * kalman_gain.transpose();

  updated_covariance = 0.5 * (updated_covariance + updated_covariance.transpose());
  updateStateFromVector(updated_state, updated_covariance);
  return true;
}

template<int VectorSize>
Eigen::Matrix<double, VectorSize, 1> UKF::weightedMean(
  const Eigen::Matrix<double, VectorSize, kSigmaPointCount> &sigma_points,
  const std::vector<int> &angle_indices) const
{
  return computeWeightedMean<VectorSize, kSigmaPointCount>(sigma_points, weights_mean_, angle_indices);
}

template<int VectorSize>
Eigen::Matrix<double, VectorSize, 1> UKF::residual(
  const Eigen::Matrix<double, VectorSize, 1> &value,
  const Eigen::Matrix<double, VectorSize, 1> &reference,
  const std::vector<int> &angle_indices) const
{
  return computeResidual(value, reference, angle_indices);
}

template bool UKF::update<3>(
  const Eigen::Matrix<double, 3, 1> &measurement,
  const Eigen::Matrix<double, 3, 3> &noise,
  const MeasurementMatrix<3> &measurement_sigma_points,
  const std::vector<int> &angle_indices);

template Eigen::Matrix<double, 6, 1> UKF::weightedMean<6>(
  const Eigen::Matrix<double, 6, kSigmaPointCount> &sigma_points,
  const std::vector<int> &angle_indices) const;

template Eigen::Matrix<double, 3, 1> UKF::weightedMean<3>(
  const Eigen::Matrix<double, 3, kSigmaPointCount> &sigma_points,
  const std::vector<int> &angle_indices) const;

template Eigen::Matrix<double, 6, 1> UKF::residual<6>(
  const Eigen::Matrix<double, 6, 1> &value,
  const Eigen::Matrix<double, 6, 1> &reference,
  const std::vector<int> &angle_indices) const;

template Eigen::Matrix<double, 3, 1> UKF::residual<3>(
  const Eigen::Matrix<double, 3, 1> &value,
  const Eigen::Matrix<double, 3, 1> &reference,
  const std::vector<int> &angle_indices) const;

}  // namespace aegis_core
