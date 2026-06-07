# Experiment Plan

## Experiment 1: Dead Reckoning Drift

Goal:
Measure drift using wheel odometry only.

## Experiment 2: EKF Odometry + IMU

Goal:
Measure improvement from yaw-rate fusion.

## Experiment 3: UKF Comparison

Goal:
Compare nonlinear filtering performance against EKF.

## Experiment 4: Particle Filter Robustness

Goal:
Evaluate robustness under injected sensor noise.

## Experiment 5: GTSAM Pose Graph

Goal:
Compare recursive filtering against batch/smoothing-based optimization.

## Metrics

- Absolute Trajectory Error
- Relative Pose Error
- Final drift
- Runtime per update
- Failure recovery behavior
