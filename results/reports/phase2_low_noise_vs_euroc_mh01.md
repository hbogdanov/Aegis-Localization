# Phase 2 Comparison Report

- Synthetic baseline scenario: `low_noise`
- EuRoC sequence: `MH_01_easy`
- EuRoC replay mode: `faithful`

## Per-Estimator Metrics

| Estimator | Synthetic ATE | Synthetic Drift | Synthetic Yaw | EuRoC ATE | EuRoC Drift | EuRoC Yaw |
|---|---:|---:|---:|---:|---:|---:|
| EKF | 0.064185 | 0.059118 | 0.002135 | 0.001071 | 0.000747 | 1.796856 |
| UKF | 0.064194 | 0.066567 | 0.001960 | 0.001071 | 0.000747 | 1.796856 |
| PF | 0.025589 | 0.012590 | 0.001376 | 0.155208 | 0.178195 | 1.801968 |

## Delta Table

| Estimator | Delta ATE (EuRoC - Synthetic) | Delta Drift | Delta Yaw |
|---|---:|---:|---:|
| EKF | -0.063114 | -0.058371 | 1.794721 |
| UKF | -0.063123 | -0.065820 | 1.794895 |
| PF | 0.129619 | 0.165606 | 1.800592 |

## Notes

- EuRoC is harder because it is a recorded 6-DoF MAV trajectory reduced to a planar proxy rather than a clean synthetic 2D benchmark.
- The EuRoC proxy preserves real recorded timing and motion, but it does not provide a wheel-encoder-faithful or native MAV localization evaluation.
- Translation metrics remain useful evidence for the recorded-data path, while yaw should still be treated cautiously because the current planar proxy does not make heading the strongest validation target.
