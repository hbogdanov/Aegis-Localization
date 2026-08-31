# Phase 5 Correctness Checks

- Run date: 2026-08-19T01:12:20.016513+00:00
- Duration: 8 s
- Interpretation note: arrival-latency runs preserve the same final replayed state, but their online logged trajectories still differ before delayed corrections arrive. Whole-trajectory RMSE therefore reflects online publication history, not terminal replay correctness alone.

## Zero-Latency Equivalence

| Estimator | Final pos diff | Pos RMSE diff | Max pos diff | Final yaw diff | Yaw RMSE diff |
| --- | ---: | ---: | ---: | ---: | ---: |
| EKF | 0.000001 | 0.000015 | 0.000031 | 0.000000 | 0.000000 |
| UKF | 0.000001 | 0.000015 | 0.000031 | 0.000000 | 0.000000 |
| PF | 0.000872 | 0.003695 | 0.007884 | 0.002355 | 0.001917 |

## Arrival-Time Invariance

### arrival_100ms

| Estimator | Final pos diff | Pos RMSE diff | Max pos diff | Final yaw diff | Yaw RMSE diff |
| --- | ---: | ---: | ---: | ---: | ---: |
| EKF | 0.000000 | 0.002064 | 0.005161 | 0.000000 | 0.000000 |
| UKF | 0.000000 | 0.002023 | 0.005161 | 0.000000 | 0.000000 |
| PF | 0.008826 | 0.004397 | 0.017430 | -0.006563 | 0.003532 |

### arrival_500ms

| Estimator | Final pos diff | Pos RMSE diff | Max pos diff | Final yaw diff | Yaw RMSE diff |
| --- | ---: | ---: | ---: | ---: | ---: |
| EKF | 0.000000 | 0.007763 | 0.013392 | 0.000000 | 0.000000 |
| UKF | 0.000000 | 0.007763 | 0.013392 | 0.000000 | 0.000000 |
| PF | 0.008826 | 0.008610 | 0.017659 | -0.006563 | 0.005075 |

### arrival_1000ms

| Estimator | Final pos diff | Pos RMSE diff | Max pos diff | Final yaw diff | Yaw RMSE diff |
| --- | ---: | ---: | ---: | ---: | ---: |
| EKF | 0.000000 | 0.014658 | 0.022083 | 0.000000 | 0.000000 |
| UKF | 0.000000 | 0.014658 | 0.022083 | 0.000000 | 0.000000 |
| PF | 0.008826 | 0.009975 | 0.024343 | -0.006563 | 0.005443 |

## Reversed Arrival Order

| Estimator | Final pos diff | Final yaw diff | Cov diff |
| --- | ---: | ---: | ---: |
| EKF | 0.000000 | 0.000000 | 0.000000 |
| UKF | 0.000000 | 0.000000 | 0.000000 |
| PF | 0.000000 | 0.000000 | n/a |

## History-Window Rejection

| Estimator | Final pos diff vs valid-only | Final yaw diff | Cov diff | Rejection stats |
| --- | ---: | ---: | ---: | --- |
| EKF | 0.000000 | 0.000000 | 0.000000 | received=2, applied=1, history_rejections=1 |
| UKF | 0.000000 | 0.000000 | 0.000000 | n/a |
| PF | 0.000000 | 0.000000 | n/a | n/a |

## Interpretation

- Zero-latency replay and immediate fusion now agree to numerical tolerance.
- EKF and UKF terminal state and covariance are invariant to delayed-correction arrival time in the deterministic replay checks.
- Reversed arrival order also converges to the same terminal state and covariance, showing that replay now preserves prior out-of-sequence corrections rather than erasing them with stale future snapshots.
- The history-window test shows an explicit rejection path for unreconstructable late measurements while leaving the final estimator state unchanged relative to a valid-only reference.
- Nonzero whole-trajectory RMSE in delayed runs should be interpreted as online publication error before the delayed correction arrived, not as a failure of reconstructed terminal replay correctness.
