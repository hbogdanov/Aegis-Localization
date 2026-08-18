# Phase 4 Gating Report

## Methodology

- Synthetic pose-like outliers were injected into the odometry pose correction stream while velocity and yaw-rate measurements remained on the nominal path.
- EKF and UKF were compared with identical seeds and corruption settings, once with pose gating disabled and once with Mahalanobis-distance pose gating enabled.
- The gating threshold was the same across repeats and was recorded in run metadata and estimator stats.

## Aggregate Metrics

| Metric | EKF No Gate | EKF Gate | UKF No Gate | UKF Gate |
|---|---:|---:|---:|---:|
| ATE RMSE | 0.1088 | 0.3607 | 0.1086 | 0.3607 |
| Final Drift | 0.1183 | 0.3427 | 0.1235 | 0.3428 |
| Yaw RMSE | 0.0264 | 0.2929 | 0.0264 | 0.2930 |

## Gating Behavior

| Metric | EKF No Gate | EKF Gate | UKF No Gate | UKF Gate |
|---|---:|---:|---:|---:|
| Mean rejection rate | 0.0000 | 0.4440 | 0.0000 | 0.4441 |
| Mean rejected pose updates | 0.0000 | 177.6000 | 0.0000 | 177.4000 |

## Interpretation

- EKF gating increased mean ATE from 0.1088 to 0.3607.
- UKF gating increased mean ATE from 0.1086 to 0.3607.
- The current threshold rejected about 0.4440 of EKF pose updates and 0.4441 of UKF pose updates on average.
- This means Phase 4 is functionally complete, but the first gating configuration did not deliver a net robustness win under this corruption regime.
- The most concerning evidence is not that gating rejected bad updates, but that one gated seed became strongly over-rejective and produced much worse pose and yaw error. That points to a threshold or measurement-model problem rather than an implementation failure.
- PF is preserved as a performance baseline but is intentionally excluded from covariance-gating conclusions.

## Artifacts

- Machine-readable summary: `results/phase4_gating/summary.json`
- Run-local stats and diagnostics: `results/phase4_gating/<scenario>/repeats/run_###/`

