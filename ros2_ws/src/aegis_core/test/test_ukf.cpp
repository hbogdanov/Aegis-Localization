#include "aegis_core/ukf.hpp"
#include "aegis_core/utils.hpp"

#include <gtest/gtest.h>

#include <Eigen/Eigenvalues>

namespace
{

void expectCovarianceIsSymmetricAndPsd(const aegis_core::Matrix6d &covariance)
{
  const aegis_core::Matrix6d symmetrized = 0.5 * (covariance + covariance.transpose());
  EXPECT_LT((covariance - covariance.transpose()).cwiseAbs().maxCoeff(), 1e-9);

  Eigen::SelfAdjointEigenSolver<aegis_core::Matrix6d> solver(symmetrized);
  ASSERT_EQ(solver.info(), Eigen::Success);
  EXPECT_GE(solver.eigenvalues().minCoeff(), -1e-9);
}

}  // namespace

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

TEST(UkfTest, ZeroMotionPredictKeepsStateFixedWithoutProcessNoise)
{
  aegis_core::UKF filter;
  aegis_core::State2D state;
  state.px = -0.7;
  state.py = 0.4;
  state.theta = -0.25;
  state.covariance = aegis_core::Matrix6d::Identity() * 1e-3;
  filter.setState(state);
  filter.setProcessNoise(aegis_core::Matrix6d::Zero());

  for (int i = 0; i < 20; ++i) {
    filter.predict(0.1);
  }

  EXPECT_NEAR(filter.state().px, state.px, 1e-9);
  EXPECT_NEAR(filter.state().py, state.py, 1e-9);
  EXPECT_NEAR(filter.state().theta, state.theta, 1e-9);
}

TEST(UkfTest, RepeatedUpdatesKeepCovarianceFiniteAndPsd)
{
  aegis_core::UKF filter;
  aegis_core::State2D state;
  state.vx = 0.2;
  state.vy = -0.05;
  state.omega = 0.03;
  filter.setState(state);

  for (int i = 0; i < 25; ++i) {
    filter.predict(0.1);
    ASSERT_TRUE(filter.updateVelocityYawRate(0.2, -0.05, 0.03));
    ASSERT_TRUE(filter.updatePose(0.02 * (i + 1), -0.005 * (i + 1), 0.003 * (i + 1)));
  }

  const auto &updated = filter.state();
  EXPECT_TRUE(std::isfinite(updated.px));
  EXPECT_TRUE(std::isfinite(updated.py));
  EXPECT_TRUE(std::isfinite(updated.theta));
  expectCovarianceIsSymmetricAndPsd(updated.covariance);
}

TEST(UkfTest, HighRateUpdatesKeepCovarianceFiniteAndPsd)
{
  aegis_core::UKF filter;
  aegis_core::State2D state;
  state.px = 4.7;
  state.py = -1.8;
  state.theta = 0.45;
  state.vx = 0.1;
  state.vy = 0.42;
  state.omega = 0.12;
  filter.setState(state);

  for (int i = 0; i < 2500; ++i) {
    filter.predict(0.005);
    ASSERT_TRUE(filter.updateVelocityYawRate(0.09, 0.41, 0.11));
  }

  const auto &updated = filter.state();
  EXPECT_TRUE(std::isfinite(updated.px));
  EXPECT_TRUE(std::isfinite(updated.py));
  EXPECT_TRUE(std::isfinite(updated.theta));
  expectCovarianceIsSymmetricAndPsd(updated.covariance);
}
