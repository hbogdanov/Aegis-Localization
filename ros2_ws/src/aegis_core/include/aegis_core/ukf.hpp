#pragma once

#include "aegis_core/motion_model.hpp"
#include "aegis_core/utils.hpp"

#include <Eigen/Dense>
#include <string>
#include <vector>

namespace aegis_core {

using Matrix3d = Eigen::Matrix<double, 3, 3>;

class UKF
{
public:
  struct CovarianceHealth
  {
    std::string stage = "init";
    double min_eigenvalue = 0.0;
    double max_eigenvalue = 0.0;
    double symmetry_error = 0.0;
    double condition_number = 0.0;
    bool finite = true;
    bool positive_semidefinite = true;
  };

  struct UpdateDiagnostics
  {
    std::string measurement_type = "none";
    bool accepted = false;
    Eigen::VectorXd innovation = Eigen::VectorXd();
    Eigen::MatrixXd innovation_covariance = Eigen::MatrixXd();
    Matrix6d state_covariance = Matrix6d::Zero();
    double nis = 0.0;
    bool available = false;
  };

  UKF();
  explicit UKF(const State2D &initial_state);

  void setState(const State2D &initial_state);
  const State2D &state() const noexcept;

  void setProcessNoise(const Matrix6d &Q);
  void setVelocityYawRateNoise(const Matrix3d &R);
  void setPoseNoise(const Matrix3d &R);
  void setSigmaPointParameters(double alpha, double beta, double kappa);
  void setPoseUpdateGate(bool enabled, double nis_threshold);
  const CovarianceHealth &lastCovarianceHealth() const noexcept;
  const UpdateDiagnostics &lastUpdateDiagnostics() const noexcept;

  void predict(double dt);
  bool updateVelocityYawRate(double vx, double vy, double omega);
  bool updatePose(double px, double py, double theta);

private:
  static constexpr int kStateSize = 6;
  static constexpr int kSigmaPointCount = 2 * kStateSize + 1;

  using SigmaPointMatrix = Eigen::Matrix<double, kStateSize, kSigmaPointCount>;
  using WeightVector = Eigen::Matrix<double, kSigmaPointCount, 1>;

  template<int MeasurementSize>
  using MeasurementMatrix = Eigen::Matrix<double, MeasurementSize, kSigmaPointCount>;

  SigmaPointMatrix generateSigmaPoints() const;
  void computeWeights();
  void updateStateFromVector(const Vector6d &mean, const Matrix6d &covariance, const std::string &stage);
  void storeUpdateDiagnostics(
    const std::string &measurement_type,
    bool accepted,
    const Eigen::VectorXd &innovation,
    const Eigen::MatrixXd &innovation_covariance,
    const Matrix6d &state_covariance);
  Matrix6d discretizeProcessNoise(double dt) const;
  Matrix6d validateCovariance(const Matrix6d &covariance, const std::string &stage);
  Eigen::LLT<Matrix6d> computeCovarianceFactor(const Matrix6d &covariance) const;

  template<int MeasurementSize>
  bool update(
    const std::string &measurement_type,
    const Eigen::Matrix<double, MeasurementSize, 1> &measurement,
    const Eigen::Matrix<double, MeasurementSize, MeasurementSize> &noise,
    const MeasurementMatrix<MeasurementSize> &measurement_sigma_points,
    const std::vector<int> &angle_indices);

  template<int VectorSize>
  Eigen::Matrix<double, VectorSize, 1> weightedMean(
    const Eigen::Matrix<double, VectorSize, kSigmaPointCount> &sigma_points,
    const std::vector<int> &angle_indices) const;

  template<int VectorSize>
  Eigen::Matrix<double, VectorSize, 1> residual(
    const Eigen::Matrix<double, VectorSize, 1> &value,
    const Eigen::Matrix<double, VectorSize, 1> &reference,
    const std::vector<int> &angle_indices) const;

  State2D state_;
  Matrix6d process_noise_;
  Matrix3d velocity_yaw_rate_noise_;
  Matrix3d pose_noise_;
  double alpha_;
  double beta_;
  double kappa_;
  double lambda_;
  WeightVector weights_mean_;
  WeightVector weights_covariance_;
  CovarianceHealth last_covariance_health_;
  bool pose_gate_enabled_ = false;
  double pose_gate_threshold_ = 9.324146034653893;
  UpdateDiagnostics last_update_diagnostics_;
};

}  // namespace aegis_core
