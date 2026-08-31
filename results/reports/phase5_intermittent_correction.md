# Phase 5 Intermittent Correction

- Run date: 2026-08-19T00:05:34.276207+00:00
- Duration: 12 s

## Metrics

| Estimator | Baseline ATE | Stress ATE | Baseline Drift | Stress Drift | Baseline Yaw RMSE | Stress Yaw RMSE |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| EKF | 0.0601 | 0.1837 | 0.0370 | 0.2206 | 0.0023 | 0.0595 |
| UKF | 0.0601 | 0.1837 | 0.0370 | 0.2206 | 0.0023 | 0.0595 |
| PF | 0.0142 | 0.1190 | 0.0137 | 0.1405 | 0.0076 | 0.0451 |

## Correction Stream

- Frequency: 2.0 Hz
- Dropout probability: 0.35
- Latency: 0.25 s
- Outlier probability: 0.10

## EKF/UKF Consistency

- EKF pose NIS in-bounds: n/a, above upper bound: n/a
- UKF pose NIS in-bounds: n/a, above upper bound: n/a
