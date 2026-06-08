#include "aegis_core/ukf.hpp"
#include "aegis_core/utils.hpp"

#include <gtest/gtest.h>

TEST(UkfTest, AcceptsSigmaPointParameters)
{
  aegis_core::UKF filter;

  EXPECT_NO_THROW(filter.setSigmaPointParameters(0.2, 2.0, 0.0));
  EXPECT_NO_THROW(filter.setSigmaPointParameters(0.5, 2.0, 1.0));
}

TEST(UkfTest, PredictMovesStateForward)
{
  aegis_core::UKF filter;
  aegis_core::State2D state;
  state.px = 1.0;
  state.py = -2.0;
  state.theta = 0.1;
  state.vx = 0.5;
  state.vy = -0.25;
  state.omega = 0.2;
  filter.setState(state);

  filter.predict(2.0);

  EXPECT_NEAR(filter.state().px, 2.0, 1e-3);
  EXPECT_NEAR(filter.state().py, -2.5, 1e-3);
  EXPECT_NEAR(filter.state().theta, 0.5, 1e-3);
}

TEST(UkfTest, UpdatePosePullsStateTowardMeasurement)
{
  aegis_core::UKF filter;
  filter.setPoseNoise(aegis_core::Matrix3d::Zero());

  aegis_core::State2D state;
  state.px = 0.0;
  state.py = 0.0;
  state.theta = 0.0;
  filter.setState(state);

  filter.updatePose(1.0, -0.5, 0.25);

  EXPECT_NEAR(filter.state().px, 1.0, 1e-2);
  EXPECT_NEAR(filter.state().py, -0.5, 1e-2);
  EXPECT_NEAR(filter.state().theta, 0.25, 1e-2);
}

TEST(UkfTest, UpdateVelocityYawRatePullsStateTowardMeasurement)
{
  aegis_core::UKF filter;
  filter.setVelocityYawRateNoise(aegis_core::Matrix3d::Zero());

  aegis_core::State2D state;
  state.vx = 0.0;
  state.vy = 0.0;
  state.omega = 0.0;
  filter.setState(state);

  filter.updateVelocityYawRate(0.8, -0.3, 0.15);

  EXPECT_NEAR(filter.state().vx, 0.8, 1e-2);
  EXPECT_NEAR(filter.state().vy, -0.3, 1e-2);
  EXPECT_NEAR(filter.state().omega, 0.15, 1e-2);
}

TEST(UkfTest, YawUpdateWrapsCorrectlyNearPi)
{
  constexpr double PI = 3.14159265358979323846;

  aegis_core::UKF filter;
  filter.setPoseNoise(aegis_core::Matrix3d::Zero());

  aegis_core::State2D state;
  state.theta = PI - 0.05;
  filter.setState(state);

  filter.updatePose(0.0, 0.0, -PI + 0.02);

  EXPECT_NEAR(filter.state().theta, -PI + 0.02, 1e-2);
  EXPECT_GE(filter.state().theta, -PI);
  EXPECT_LE(filter.state().theta, PI);
}
