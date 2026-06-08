#include "aegis_core/particle_filter.hpp"

#include <gtest/gtest.h>

#include <cmath>

TEST(ParticleFilterTest, InitializationCreatesRequestedParticleCount)
{
  aegis_core::ParticleFilter filter(250);
  aegis_core::State2D mean;
  filter.initialize(mean, 0.0, 0.0, 0.0);

  EXPECT_EQ(filter.particleCount(), 250u);
}

TEST(ParticleFilterTest, EstimateStateMatchesZeroSpreadInitialization)
{
  aegis_core::ParticleFilter filter(200);
  aegis_core::State2D mean;
  mean.px = 1.0;
  mean.py = -2.0;
  mean.theta = 0.4;
  mean.vx = 0.3;
  mean.vy = -0.1;
  mean.omega = 0.2;

  filter.initialize(mean, 0.0, 0.0, 0.0);
  const aegis_core::State2D estimate = filter.estimateState();

  EXPECT_NEAR(estimate.px, mean.px, 1e-9);
  EXPECT_NEAR(estimate.py, mean.py, 1e-9);
  EXPECT_NEAR(estimate.theta, mean.theta, 1e-9);
}

TEST(ParticleFilterTest, PredictMovesParticlesForward)
{
  aegis_core::ParticleFilter filter(150);
  filter.setNoiseParameters(0.0, 0.0, 0.0, 0.05, 0.05, 0.03, 0.5);

  aegis_core::State2D mean;
  mean.theta = 0.1;
  filter.initialize(mean, 0.0, 0.0, 0.0);
  filter.predict(2.0, 0.5, -0.25, 0.2);

  const aegis_core::State2D estimate = filter.estimateState();
  EXPECT_NEAR(estimate.px, 1.0, 1e-9);
  EXPECT_NEAR(estimate.py, -0.5, 1e-9);
  EXPECT_NEAR(estimate.theta, 0.5, 1e-9);
}

TEST(ParticleFilterTest, UpdatePosePullsEstimateTowardMeasurement)
{
  aegis_core::ParticleFilter filter(1000);
  filter.setNoiseParameters(0.0, 0.0, 0.0, 0.05, 0.05, 0.03, 0.5);

  aegis_core::State2D mean;
  filter.initialize(mean, 0.5, 0.5, 0.2);
  const double error_before = std::abs(filter.estimateState().px - 1.0);

  filter.updatePose(1.0, 0.0, 0.0);
  const double error_after = std::abs(filter.estimateState().px - 1.0);

  EXPECT_LT(error_after, error_before);
}

TEST(ParticleFilterTest, NormalizeWeightsSumsToOne)
{
  aegis_core::ParticleFilter filter(500);
  filter.setNoiseParameters(0.0, 0.0, 0.0, 0.05, 0.05, 0.03, 0.5);

  aegis_core::State2D mean;
  filter.initialize(mean, 0.5, 0.5, 0.2);
  filter.updatePose(0.2, -0.1, 0.05);
  filter.normalizeWeights();

  double sum = 0.0;
  for (const auto &particle : filter.particles()) {
    sum += particle.weight;
  }

  EXPECT_NEAR(sum, 1.0, 1e-9);
}

TEST(ParticleFilterTest, SystematicResamplePreservesParticleCount)
{
  aegis_core::ParticleFilter filter(300);
  filter.setNoiseParameters(0.0, 0.0, 0.0, 0.05, 0.05, 0.03, 1.0);

  aegis_core::State2D mean;
  filter.initialize(mean, 0.5, 0.5, 0.2);
  filter.updatePose(0.0, 0.0, 0.0);
  filter.systematicResample();

  EXPECT_EQ(filter.particleCount(), 300u);
}

TEST(ParticleFilterTest, EffectiveSampleSizeWithinValidRange)
{
  aegis_core::ParticleFilter filter(400);
  filter.setNoiseParameters(0.0, 0.0, 0.0, 0.05, 0.05, 0.03, 0.5);

  aegis_core::State2D mean;
  filter.initialize(mean, 0.4, 0.4, 0.2);
  filter.updatePose(0.1, 0.0, 0.0);

  const double neff = filter.effectiveSampleSize();
  EXPECT_GT(neff, 0.0);
  EXPECT_LE(neff, 400.0);
}
