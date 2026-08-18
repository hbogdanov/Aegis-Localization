#pragma once

#include "aegis_core/motion_model.hpp"
#include "aegis_core/utils.hpp"

#include <Eigen/Dense>
#include <string>

namespace aegis_core {

using Vector3d = Eigen::Matrix<double, 3, 1>;
using Matrix3d = Eigen::Matrix<double, 3, 3>;

class EKF
{
public:
  struct UpdateDiagnostics
  {
    std::string measurement_type = "none";
    Eigen::VectorXd innovation = Eigen::VectorXd();
    Eigen::MatrixXd innovation_covariance = Eigen::MatrixXd();
    Matrix6d state_covariance = Matrix6d::Zero();
    double nis = 0.0;
    bool available = false;
  };

  EKF();
  explicit EKF(const State2D &initial_state);

  void setState(const State2D &initial_state);
  const State2D &state() const noexcept;

  void setProcessNoise(const Matrix6d &Q);
  void setVelocityYawRateNoise(const Matrix3d &R);
  void setPoseNoise(const Matrix3d &R);
  const UpdateDiagnostics &lastUpdateDiagnostics() const noexcept;

  void predict(double dt);
  bool updateVelocityYawRate(double vx, double vy, double omega);
  bool updatePose(double px, double py, double theta);

private:
  void storeUpdateDiagnostics(
    const std::string &measurement_type,
    const Eigen::VectorXd &innovation,
    const Eigen::MatrixXd &innovation_covariance,
    const Matrix6d &state_covariance);

  State2D state_;
  Matrix6d process_noise_;
  Matrix3d velocity_yaw_rate_noise_;
  Matrix3d pose_noise_;
  UpdateDiagnostics last_update_diagnostics_;
};

}  // namespace aegis_core
