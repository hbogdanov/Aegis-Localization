# Phase 3 Estimator Consistency Report

## Methodology

- NIS is evaluated per update as `nu^T S^-1 nu`, using a linear solve against the logged innovation covariance rather than an explicit matrix inverse.
- NIS is reported separately by measurement type so pose updates and velocity/yaw-rate updates are not mixed into one statistical bucket.
- The primary consistency interval uses two-sided 95% chi-square bounds.
- NEES is reported only for synthetic runs, where the planar `x`, `y`, `yaw` state, wrapped yaw error, and the corresponding covariance submatrix can be interpreted consistently.
- PF remains part of Phase 2 performance comparison but is intentionally excluded from covariance-consistency analysis.

## Aggregate Repeated-Trial NIS

### Low Noise - pose

| Metric | EKF | UKF |
|---|---:|---:|
| Mean NIS | 0.1311 | 0.1310 |
| Median NIS | 0.1117 | 0.1114 |
| Expected DOF | 3.0000 | 3.0000 |
| Within 95% bounds | 0.2550 | 0.2541 |
| Below lower bound | 0.7450 | 0.7459 |
| Above upper bound | 0.0000 | 0.0000 |
| Sample count | 400.0000 | 399.0000 |

### Low Noise - velocity_yaw_rate

| Metric | EKF | UKF |
|---|---:|---:|
| Mean NIS | 2.6934 | 2.6938 |
| Median NIS | 2.7132 | 2.7148 |
| Expected DOF | 3.0000 | 3.0000 |
| Within 95% bounds | 0.9890 | 0.9890 |
| Below lower bound | 0.0110 | 0.0110 |
| Above upper bound | 0.0000 | 0.0000 |
| Sample count | 399.8000 | 399.0000 |

### High Noise Dropout - pose

| Metric | EKF | UKF |
|---|---:|---:|
| Mean NIS | 1.1493 | 1.1492 |
| Median NIS | 0.7918 | 0.7913 |
| Expected DOF | 3.0000 | 3.0000 |
| Within 95% bounds | 0.8630 | 0.8629 |
| Below lower bound | 0.1370 | 0.1371 |
| Above upper bound | 0.0000 | 0.0000 |
| Sample count | 317.6000 | 317.4000 |

### High Noise Dropout - velocity_yaw_rate

| Metric | EKF | UKF |
|---|---:|---:|
| Mean NIS | 5.2797 | 5.2753 |
| Median NIS | 4.3737 | 4.3699 |
| Expected DOF | 3.0000 | 3.0000 |
| Within 95% bounds | 0.8477 | 0.8482 |
| Below lower bound | 0.0070 | 0.0070 |
| Above upper bound | 0.1453 | 0.1448 |
| Sample count | 317.4000 | 317.2000 |

### Dead Reckoning - velocity_yaw_rate

| Metric | EKF | UKF |
|---|---:|---:|
| Mean NIS | 2.7285 | 2.7285 |
| Median NIS | 2.7342 | 2.7342 |
| Expected DOF | 3.0000 | 3.0000 |
| Within 95% bounds | 0.9940 | 0.9940 |
| Below lower bound | 0.0060 | 0.0060 |
| Above upper bound | 0.0000 | 0.0000 |
| Sample count | 399.0000 | 399.0000 |

## Synthetic-Only NEES

### Low Noise - Synthetic Planar NEES

| Metric | EKF | UKF |
|---|---:|---:|
| Mean NEES | 2.2570 | 2.2580 |
| Median NEES | 2.3465 | 2.3468 |
| State DOF | 3.0000 | 3.0000 |
| Within 95% bounds | 1.0000 | 1.0000 |
| Below lower bound | 0.0000 | 0.0000 |
| Above upper bound | 0.0000 | 0.0000 |
| Sample count | 399.8000 | 399.0000 |

### High Noise Dropout - Synthetic Planar NEES

| Metric | EKF | UKF |
|---|---:|---:|
| Mean NEES | 7.1463 | 16.0248 |
| Median NEES | 4.3127 | 4.5231 |
| State DOF | 3.0000 | 3.0000 |
| Within 95% bounds | 0.8919 | 0.8523 |
| Below lower bound | 0.0006 | 0.0006 |
| Above upper bound | 0.1074 | 0.1471 |
| Sample count | 317.4000 | 317.2000 |

### Dead Reckoning - Synthetic Planar NEES

| Metric | EKF | UKF |
|---|---:|---:|
| Mean NEES | 0.5260 | 0.5260 |
| Median NEES | 0.3267 | 0.3267 |
| State DOF | 3.0000 | 3.0000 |
| Within 95% bounds | 0.7534 | 0.7534 |
| Below lower bound | 0.2466 | 0.2466 |
| Above upper bound | 0.0000 | 0.0000 |
| Sample count | 399.0000 | 399.0000 |

## Performance and Consistency

- Phase 2 established translational performance and drift behavior across the same synthetic scenarios. This section checks whether EKF and UKF uncertainty reflected those outcomes.

### Low Noise

- EKF: ATE 0.0642, final drift 0.0518, velocity/yaw-rate mean NIS 2.6934, fraction in bounds 0.9890. This suggests consistency is very conservative or well matched; translation error stays in the low-error regime.
- UKF: ATE 0.0642, final drift 0.0667, velocity/yaw-rate mean NIS 2.6938, fraction in bounds 0.9890. This suggests consistency is very conservative or well matched; translation error stays in the low-error regime.

### High Noise Dropout

- EKF: ATE 0.1053, final drift 0.0925, velocity/yaw-rate mean NIS 5.2797, fraction in bounds 0.8477. This suggests consistency is broadly plausible.
- UKF: ATE 0.1066, final drift 0.0917, velocity/yaw-rate mean NIS 5.2753, fraction in bounds 0.8482. This suggests consistency is broadly plausible.

### Dead Reckoning

- EKF: ATE 0.0715, final drift 0.0784, velocity/yaw-rate mean NIS 2.7285, fraction in bounds 0.9940. This suggests consistency is very conservative or well matched; translation error stays in the low-error regime.
- UKF: ATE 0.0715, final drift 0.0810, velocity/yaw-rate mean NIS 2.7285, fraction in bounds 0.9940. This suggests consistency is very conservative or well matched; translation error stays in the low-error regime.

## Limitations

- The chi-square bounds are approximate but consistent across all reported runs and dimensions.
- NEES is intentionally limited to the synthetic planar benchmark and should not be generalized to EuRoC proxy runs.
- Plotting is optional and may be skipped on machines with a broken matplotlib stack; the JSON and CSV artifacts remain the source of record.

## Artifacts

- Machine-readable campaign summary: `results/campaign/summary.json`
- Run-level consistency summaries: `results/campaign/<scenario>/repeats/run_###/consistency_summary.json`
- Run-level diagnostics CSVs: `results/campaign/<scenario>/repeats/run_###/filter_diagnostics.csv`

