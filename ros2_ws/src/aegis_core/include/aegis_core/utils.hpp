#pragma once

#include <cmath>

namespace aegis_core {

inline double normalizeAngle(double angle)
{
  constexpr double PI = 3.14159265358979323846;
  constexpr double TWO_PI = 2.0 * PI;
  double wrapped = std::fmod(angle + PI, TWO_PI);
  if (wrapped < 0.0) {
    wrapped += TWO_PI;
  }
  return wrapped - PI;
}

}  // namespace aegis_core
