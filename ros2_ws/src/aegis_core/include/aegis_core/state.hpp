#pragma once

#include <Eigen/Dense>

namespace aegis_core {

using Vector6d = Eigen::Matrix<double, 6, 1>;
using Matrix6d = Eigen::Matrix<double, 6, 6>;

struct State2D
{
  double px = 0.0;
  double py = 0.0;
  double theta = 0.0;
  double vx = 0.0;
  double vy = 0.0;
  double omega = 0.0;
  Matrix6d covariance = Matrix6d::Identity() * 1e-3;

  Vector6d toVector() const
  {
    Vector6d vector;
    vector << px, py, theta, vx, vy, omega;
    return vector;
  }

  void fromVector(const Vector6d &vector)
  {
    px = vector(0);
    py = vector(1);
    theta = vector(2);
    vx = vector(3);
    vy = vector(4);
    omega = vector(5);
  }
};

}  // namespace aegis_core
