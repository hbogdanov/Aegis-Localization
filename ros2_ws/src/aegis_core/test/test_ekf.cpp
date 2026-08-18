#include "aegis_core/ekf.hpp"
#include "aegis_core/motion_model.hpp"
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

TEST(EkfTest, ZeroMotionPredictKeepsStateFixedWithoutProcessNoise)
{
  aegis_core::EKF filter;
  aegis_core::State2D initial_state;
  initial_state.px = 1.2;
  initial_state.py = -0.4;
  initial_state.theta = 0.3;
  initial_state.vx = 0.0;
  initial_state.vy = 0.0;
  initial_state.omega = 0.0;
  initial_state.covariance = aegis_core::Matrix6d::Identity() * 1e-3;
  filter.setState(initial_state);
  filter.setProcessNoise(aegis_core::Matrix6d::Zero());

  for (int i = 0; i < 20; ++i) {
    filter.predict(0.1);
  }

  EXPECT_NEAR(filter.state().px, initial_state.px, 1e-9);
  EXPECT_NEAR(filter.state().py, initial_state.py, 1e-9);
  EXPECT_NEAR(filter.state().theta, initial_state.theta, 1e-9);
  EXPECT_NEAR(filter.state().vx, 0.0, 1e-9);
  EXPECT_NEAR(filter.state().vy, 0.0, 1e-9);
  EXPECT_NEAR(filter.state().omega, 0.0, 1e-9);
}

TEST(EkfTest, YawUpdateWrapsCorrectlyAcrossPiBoundary)
{
  constexpr double kPi = 3.14159265358979323846;

  aegis_core::EKF filter;
  filter.setPoseNoise(aegis_core::Matrix3d::Zero());

  aegis_core::State2D initial_state;
  initial_state.theta = kPi - 0.03;
  filter.setState(initial_state);

  filter.updatePose(0.0, 0.0, -kPi + 0.02);

  EXPECT_NEAR(filter.state().theta, -kPi + 0.02, 1e-2);
  EXPECT_GE(filter.state().theta, -kPi);
  EXPECT_LE(filter.state().theta, kPi);
}

TEST(EkfTest, RepeatedDropoutStylePredictsKeepCovarianceFiniteAndPsd)
{
  aegis_core::EKF filter;
  aegis_core::State2D initial_state;
  initial_state.vx = 0.15;
  initial_state.vy = -0.1;
  initial_state.omega = 0.05;
  filter.setState(initial_state);

  for (int i = 0; i < 50; ++i) {
    filter.predict(0.1);
  }

  const auto &state = filter.state();
  EXPECT_TRUE(std::isfinite(state.px));
  EXPECT_TRUE(std::isfinite(state.py));
  EXPECT_TRUE(std::isfinite(state.theta));
  EXPECT_TRUE(std::isfinite(state.vx));
  EXPECT_TRUE(std::isfinite(state.vy));
  EXPECT_TRUE(std::isfinite(state.omega));
  expectCovarianceIsSymmetricAndPsd(state.covariance);
}
