# Aegis Localization: Project Status

## Identity

Aegis is a C++17 ROS2 localization benchmarking and research platform. It compares in-house EKF, UKF, and particle-filter estimators under reproducible synthetic conditions, one bounded public-data proxy, and controlled correction failures. Its strongest current story is not "three filters exist"; it is "how classical estimators behave when measurements are noisy, corrupted, intermittent, or late."

## Confirmed, Working Capabilities

- ROS2 EKF, UKF, and PF pipelines consume odometry and IMU data and publish comparable pose trajectories.
- A documented synthetic benchmark runs all three estimators through one logger and evaluator, producing ATE, final drift, yaw RMSE, update rate, metadata, and plots when the plotting environment is available.
- Synthetic campaigns support fixed seeds, scenario presets, repeated trials, preserved per-run artifacts, and aggregate mean/standard-deviation results.
- EKF and UKF export innovation/covariance diagnostics. The evaluator computes NIS with chi-square bounds and synthetic-only planar NEES where the state/covariance interpretation is clean.
- EKF and UKF support configurable Mahalanobis pose-update gating. The committed sweep records rejection rate, true/false positive behavior, and the tradeoff between rejecting outliers and rejecting useful measurements.
- One EuRoC `MH_01_easy` sequence runs through a shared planar-proxy evaluation path with canonical outputs, metrics, metadata, and a comparison against the synthetic baseline.
- The synthetic correction source supports configurable frequency, dropout, latency, noise, and outliers.
- EKF and UKF perform timestamp-aware replay for delayed pose corrections. Deterministic checks confirm zero-latency equivalence, terminal-state/covariance invariance through one second of delayed arrival, reversed arrival-order handling, and explicit rejection of corrections outside the history window.
- A repeated Phase 5 study compares naive arrival-time fusion with replay and varies correction frequency, dropout, latency, noise, outliers/gating, blackout recovery, and a combined degraded condition.

Primary evidence is preserved in:

- `results/reports/phase2_low_noise_vs_euroc_mh01.md`
- `results/reports/phase3_estimator_consistency.md`
- `results/reports/phase4b_gating_sweep.md`
- `results/reports/phase5_correctness.md`
- `results/reports/phase5_final_report.md`

## Implemented, But Bounded

- EuRoC is a planar `x`, `y`, `yaw` proxy, not native 6-DoF MAV localization. Its yaw conclusions are deliberately limited and it does not validate wheel-odometry realism.
- Particle-filter delayed replay exists, but exact deterministic replay equivalence has only been established for EKF/UKF. PF resampling is stochastic and should be evaluated separately.
- Combined-degraded pose-NIS aggregation is unavailable in the preserved campaign because delayed corrections were not yet emitted to the shared diagnostic log; a focused post-fix run verifies one NIS record per correction. No combined NIS value is claimed.
- Gazebo launches and logs estimator integration, but the ground-truth bridge is not reliable enough for quantitative scoring. It is demo/integration support, not benchmark evidence.

## Not Implemented Or Not Yet Defensible

- No GTSAM/iSAM2 smoothing baseline.
- No second public dataset, recorded ROS bag backend, hardware run, camera/vision frontend, landmark observations, or full 3D state estimator.
- No claim of hardware validation, production readiness, full EuRoC MAV validation, or visual localization.

## Where It Falls Short

The project is stronger on estimator engineering and controlled experimental method than on external validation. Its public-data story has only one reduced sequence and the estimator comparison is planar and classical. Phase 5 now supplies a clear synthetic research result, but it does not substitute for hardware, native 3D, or multi-dataset validation.

## Career And Research Readiness

For robotics software and state-estimation internships, Aegis is already a strong portfolio project: it demonstrates ROS2/C++, estimator implementation, reproducibility, diagnostics, regression-minded thinking, and honest handling of failure modes.

For MS robotics applications and research labs, it now has a focused, bounded research question with measured evidence: timestamp-aware replay improves current-state estimation under delayed corrections, and gating protects EKF/UKF from the specified injected outliers. That is more compelling than adding another generic dataset or polishing Gazebo.

For robotics perception roles, Aegis is supportive rather than sufficient on its own. The correction interface is an intentional bridge to future visual or landmark-based localization, but no perception system is currently implemented.

## Closest Path To The Flagship Goal

1. Freeze Aegis as the localization flagship and use the Phase 5 report in portfolio material.
2. Treat a GTSAM/iSAM2 baseline as optional, not required for the project claim.
3. Move primary effort to the perception project rather than expanding datasets or Gazebo.

See `development_plan.md` for the detailed execution plan and `docs/euroc_backend.md` for the recorded-data limitations.
