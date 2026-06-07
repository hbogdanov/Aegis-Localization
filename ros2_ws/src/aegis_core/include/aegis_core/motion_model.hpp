#pragma once

#include "aegis_core/state.hpp"

namespace aegis_core {

class MotionModel
{
public:
  static Vector6d propagate(const Vector6d &x, double dt);
  static Matrix6d stateTransition(double dt);
};

}  // namespace aegis_core
