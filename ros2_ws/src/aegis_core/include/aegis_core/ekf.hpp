#pragma once

#include "aegis_core/motion_model.hpp"
#include "aegis_core/utils.hpp"

#include <Eigen/Dense>

namespace aegis_core {

using Vector3d = Eigen::Matrix<double, 3, 1>;
using Matrix3d = Eigen::Matrix<double, 3, 3>;

class EKF
{
public:
  EKF();
  explicit EKF(const State2D &initial_state);

  void setState(const State2D &initial_state);
  const State2D &state() const noexcept;

  void setProcessNoise(const Matrix6d &Q);
  void setVelocityYawRateNoise(const Matrix3d &R);
  void setPoseNoise(const Matrix3d &R);

  void predict(double dt);
  bool updateVelocityYawRate(double vx, double vy, double omega);
  bool updatePose(double px, double py, double theta);

private:
  State2D state_;
  Matrix6d process_noise_;
  Matrix3d velocity_yaw_rate_noise_;
  Matrix3d pose_noise_;
};

}  // namespace aegis_core
