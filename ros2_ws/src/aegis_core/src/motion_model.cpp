#include "aegis_core/motion_model.hpp"

namespace aegis_core {

Vector6d MotionModel::propagate(const Vector6d &x, double dt)
{
  Vector6d result = x;
  result(0) += x(3) * dt;
  result(1) += x(4) * dt;
  result(2) += x(5) * dt;
  return result;
}

Matrix6d MotionModel::stateTransition(double dt)
{
  Matrix6d F = Matrix6d::Identity();
  F(0, 3) = dt;
  F(1, 4) = dt;
  F(2, 5) = dt;
  return F;
}

}  // namespace aegis_core
