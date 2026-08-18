# EuRoC Benchmark Report

- Run root: `C:\Users\Ivan\dev\Aegis-Localization\results\euroc\MH_01_easy\full_mh01_phase1_accounted`
- Sequence: `MH_01_easy`
- Run name: `full_mh01_phase1_accounted`
- Git commit: `76db690ec34c672fede6b22b208034930c391e7f`
- Truth source: `dataset`
- Replay samples published: `36382`
- Replay wall time (s): `209.386`
- Replay mode: `faithful`

## Replay Readiness

- Observed subscribers: `odom=4`, `imu=3`, `truth=1`
- Expected subscribers: `odom=3`, `imu=3`, `truth=1`

## Estimator Coverage

| Estimator | Logged Samples | Coverage vs Replay |
|---|---:|---:|
| ekf | 36264 | 0.997 |
| ukf | 36264 | 0.997 |
| pf | 36259 | 0.997 |

## Metrics

| Estimator | ATE RMSE | Final Drift | Yaw RMSE |
|---|---:|---:|---:|
| ekf | 0.001071 | 0.000915 | 1.797465 |
| ukf | 0.001071 | 0.000915 | 1.797465 |
| pf | 0.155178 | 0.189238 | 1.802607 |

## Yaw Analysis

This run should not be interpreted as a clean yaw-validation benchmark.

| Estimator | Circular Mean Offset | Centered Yaw RMSE | Raw Yaw RMSE |
|---|---:|---:|---:|
| ekf | 1.752021 | 1.690151 | 1.797465 |
| ukf | 1.752021 | 1.690151 | 1.797465 |
| pf | 1.763877 | 1.692073 | 1.802607 |

## Coverage Accounting

| Stage | GT/Odom | EKF | UKF | PF |
|---|---:|---:|---:|---:|
| Replay published | 36382 | 36382 | 36382 | 36382 |
| Estimator odom received | 36200 | 36000 | 36000 | 36000 |
| Estimator pose published | 36200 | 36000 | 36000 | 36000 |
| Logger received | 36200/36200 | 36200 | 36200 | 36200 |
| Final CSV rows | 36285/36285 | 36265 | 36265 | 36260 |

## Reduction Assumptions

- EuRoC is reduced to planar x, y, yaw motion only.
- Ground truth is taken from state_groundtruth_estimate0/data.csv.
- Planar linear velocities are derived from consecutive ground-truth positions.
- Aligned IMU yaw rate is copied into /odom.twist.angular.z because current Aegis ROS wrappers read angular rate from odometry twist rather than /imu directly.
- use_odom_pose_update is disabled so ground-truth pose is used only for initialization and evaluation, not for continuous estimator correction.
- This backend is a real-data motion and timing benchmark, not a wheel-encoder-faithful EuRoC localization benchmark.

## Launch Issues

- No launch issues recorded.

## Conclusions

- This run successfully produced canonical recorded-data outputs from EuRoC proxy replay.
- UKF completed the intended recorded-data evaluation window without the earlier non-positive-definite covariance crash, and its final covariance health remained finite and PSD.
- The UKF crash on earlier MH_01_easy runs was consistent with two implementation-level issues: process noise was being added without dt scaling at high replay rates, and the previous alpha=0.1 sigma-point setting produced an extremely negative central covariance weight.
- Yaw should still be treated cautiously or excluded as a headline proxy metric, because the current planar benchmark compares projected MAV yaw while the motion model propagates x/y directly from world-frame vx/vy without heading-coupled translation.
- Remaining sample loss is now small and accounted for at the artifact level (97 ground-truth rows short of replay publication), but the shutdown/write path is not yet perfect.
- This artifact is suitable as a defensible Phase 1 recorded-data benchmark result, with yaw limitations explicitly documented rather than hidden.
