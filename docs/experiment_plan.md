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

## Experiment 5: Intermittent And Delayed Pose Correction

Goal:
Measure how EKF, UKF, and PF respond to intermittent pose-like corrections under controlled dropout, latency, noise, and outliers.

Current evidence:

- timestamp-aware replay is implemented for delayed corrections
- deterministic EKF/UKF checks cover zero-latency equivalence, arrival-time invariance, reversed arrivals, and history-window rejection
- an initial correction-stress run is preserved, but the main result still requires naive late fusion versus replay and a compact degradation campaign

## Experiment 6: Optional Smoothing Baseline

Goal:
Compare recursive filters with an offline GTSAM/iSAM2 smoothing baseline after the intermittent-correction study is complete.

## Metrics

- Absolute Trajectory Error
- Relative Pose Error
- Final drift
- Runtime per update
- Failure recovery behavior
- NIS consistency and gating statistics where applicable
- online trajectory error and post-fusion current-state error for delayed-correction experiments
