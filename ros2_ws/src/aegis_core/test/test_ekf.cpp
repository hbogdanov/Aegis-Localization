#include "aegis_core/ekf.hpp"
#include "aegis_core/motion_model.hpp"
#include "aegis_core/utils.hpp"

#include <gtest/gtest.h>

TEST(UtilsTest, NormalizeAngle)
{
  constexpr double PI = 3.14159265358979323846;
  EXPECT_NEAR(aegis_core::normalizeAngle(3.5 * PI), -0.5 * PI, 1e-9);
  EXPECT_NEAR(aegis_core::normalizeAngle(-4.5 * PI), -0.5 * PI, 1e-9);
}

TEST(MotionModelTest, PropagateConstantVelocity)
{
  aegis_core::Vector6d state;
  state << 0.0, 0.0, 0.0, 1.0, 2.0, 0.1;

  auto predicted = aegis_core::MotionModel::propagate(state, 1.0);
  EXPECT_NEAR(predicted(0), 1.0, 1e-9);
  EXPECT_NEAR(predicted(1), 2.0, 1e-9);
  EXPECT_NEAR(predicted(2), 0.1, 1e-9);
}

TEST(EkfTest, PredictAndUpdate)
{
  aegis_core::EKF filter;
  filter.predict(0.5);

  EXPECT_NEAR(filter.state().px, 0.0, 1e-9);
  EXPECT_NEAR(filter.state().py, 0.0, 1e-9);

  filter.updateVelocityYawRate(0.5, 0.0, 0.2);
  EXPECT_NEAR(filter.state().vx, 0.5, 1e-3);
  EXPECT_NEAR(filter.state().vy, 0.0, 1e-3);
  EXPECT_NEAR(filter.state().omega, 0.2, 1e-3);

  filter.updatePose(0.5, 0.0, 0.2);
  EXPECT_NEAR(filter.state().px, 0.5, 1e-2);
  EXPECT_NEAR(filter.state().py, 0.0, 1e-2);
  EXPECT_NEAR(filter.state().theta, 0.2, 1e-2);
}
