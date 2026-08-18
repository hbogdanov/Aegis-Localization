#include "aegis_core/ekf.hpp"

#include <stdexcept>

namespace aegis_core {

EKF::EKF()
  : state_(),
    process_noise_(Matrix6d::Identity() * 1e-3),
    velocity_yaw_rate_noise_(Matrix3d::Zero()),
    pose_noise_(Matrix3d::Zero())
{
}

EKF::EKF(const State2D &initial_state)
  : state_(initial_state),
    process_noise_(Matrix6d::Identity() * 1e-3),
    velocity_yaw_rate_noise_(Matrix3d::Zero()),
    pose_noise_(Matrix3d::Zero())
{
}

void EKF::setState(const State2D &initial_state)
{
  state_ = initial_state;
}

const State2D &EKF::state() const noexcept
{
  return state_;
}

void EKF::setProcessNoise(const Matrix6d &Q)
{
  process_noise_ = Q;
}

void EKF::setVelocityYawRateNoise(const Matrix3d &R)
{
  velocity_yaw_rate_noise_ = R;
}

void EKF::setPoseNoise(const Matrix3d &R)
{
  pose_noise_ = R;
}

const EKF::UpdateDiagnostics &EKF::lastUpdateDiagnostics() const noexcept
{
  return last_update_diagnostics_;
}

void EKF::storeUpdateDiagnostics(
  const std::string &measurement_type,
  const Eigen::VectorXd &innovation,
  const Eigen::MatrixXd &innovation_covariance,
  const Matrix6d &state_covariance)
{
  last_update_diagnostics_.measurement_type = measurement_type;
  last_update_diagnostics_.innovation = innovation;
  last_update_diagnostics_.innovation_covariance = innovation_covariance;
  last_update_diagnostics_.state_covariance = state_covariance;
  if (innovation.size() == 0) {
    last_update_diagnostics_.nis = 0.0;
    last_update_diagnostics_.available = false;
    return;
  }

  const Eigen::LDLT<Eigen::MatrixXd> solver(innovation_covariance);
  if (solver.info() != Eigen::Success) {
    throw std::runtime_error("EKF innovation covariance factorization failed");
  }

  const Eigen::VectorXd solved = solver.solve(innovation);
  last_update_diagnostics_.nis = innovation.dot(solved);
  last_update_diagnostics_.available = true;
}

void EKF::predict(double dt)
{
  Eigen::Vector<double, 6> x = state_.toVector();
  Eigen::Matrix<double, 6, 6> F = MotionModel::stateTransition(dt);
  const Matrix6d discrete_process_noise = process_noise_ * std::max(dt, 1e-9);

  x = MotionModel::propagate(x, dt);
  state_.fromVector(x);
  state_.theta = normalizeAngle(state_.theta);

  state_.covariance = F * state_.covariance * F.transpose() + discrete_process_noise;
}

bool EKF::updateVelocityYawRate(double vx, double vy, double omega)
{
  Eigen::Matrix<double, 3, 6> H = Eigen::Matrix<double, 3, 6>::Zero();
  H(0, 3) = 1.0;
  H(1, 4) = 1.0;
  H(2, 5) = 1.0;

  Eigen::Vector3d z;
  z << vx, vy, omega;

  Eigen::Vector3d y = z - H * state_.toVector();
  Eigen::Matrix3d S = H * state_.covariance * H.transpose() + velocity_yaw_rate_noise_;
  const Eigen::LDLT<Eigen::Matrix3d> solver(S);
  if (solver.info() != Eigen::Success) {
    throw std::runtime_error("EKF velocity innovation covariance factorization failed");
  }
  const Eigen::Matrix<double, 6, 3> K = solver.solve(H * state_.covariance).transpose();

  Eigen::Vector<double, 6> x = state_.toVector() + K * y;
  state_.fromVector(x);
  state_.theta = normalizeAngle(state_.theta);

  state_.covariance = (Eigen::Matrix<double, 6, 6>::Identity() - K * H) * state_.covariance;
  storeUpdateDiagnostics("velocity_yaw_rate", y, S, state_.covariance);
  return true;
}

bool EKF::updatePose(double px, double py, double theta)
{
  Eigen::Matrix<double, 3, 6> H = Eigen::Matrix<double, 3, 6>::Zero();
  H(0, 0) = 1.0;
  H(1, 1) = 1.0;
  H(2, 2) = 1.0;

  Eigen::Vector3d z;
  z << px, py, normalizeAngle(theta);

  Eigen::Vector3d x_pred = H * state_.toVector();
  Eigen::Vector3d y;
  y(0) = z(0) - x_pred(0);
  y(1) = z(1) - x_pred(1);
  y(2) = normalizeAngle(z(2) - x_pred(2));

  Eigen::Matrix3d S = H * state_.covariance * H.transpose() + pose_noise_;
  const Eigen::LDLT<Eigen::Matrix3d> solver(S);
  if (solver.info() != Eigen::Success) {
    throw std::runtime_error("EKF pose innovation covariance factorization failed");
  }
  const Eigen::Matrix<double, 6, 3> K = solver.solve(H * state_.covariance).transpose();

  Eigen::Vector<double, 6> x = state_.toVector() + K * y;
  state_.fromVector(x);
  state_.theta = normalizeAngle(state_.theta);

  state_.covariance = (Eigen::Matrix<double, 6, 6>::Identity() - K * H) * state_.covariance;
  storeUpdateDiagnostics("pose", y, S, state_.covariance);
  return true;
}

}  // namespace aegis_core
