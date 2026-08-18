# EuRoC Benchmark Report

- Run root: `C:\Users\Ivan\dev\Aegis-Localization\results\euroc\MH_01_easy\full_mh01_ready`
- Sequence: `MH_01_easy`
- Run name: `full_mh01_ready`
- Git commit: `76db690ec34c672fede6b22b208034930c391e7f`
- Truth source: `dataset`
- Replay samples published: `36382`
- Replay wall time (s): `37.3619`

## Replay Readiness

- Observed subscribers: `odom=3`, `imu=3`, `truth=1`
- Expected subscribers: `odom=3`, `imu=3`, `truth=1`

## Estimator Coverage

| Estimator | Logged Samples | Coverage vs Replay |
|---|---:|---:|
| ekf | 32540 | 0.894 |
| ukf | 1019 | 0.028 |
| pf | 32481 | 0.893 |

## Metrics

| Estimator | ATE RMSE | Final Drift | Yaw RMSE |
|---|---:|---:|---:|
| ekf | 0.000467 | 0.000021 | 1.846502 |
| ukf | 0.001348 | 0.000603 | 0.591740 |
| pf | 0.157578 | 0.171659 | 1.858983 |

## Yaw Analysis

This run should not be interpreted as a clean yaw-validation benchmark.

| Estimator | Circular Mean Offset | Centered Yaw RMSE | Raw Yaw RMSE |
|---|---:|---:|---:|
| ekf | 1.977948 | 1.604252 | 1.846502 |
| ukf | 0.503896 | 0.312374 | 0.591740 |
| pf | 2.025679 | 1.601714 | 1.858983 |

## Reduction Assumptions

- EuRoC is reduced to planar x, y, yaw motion only.
- Ground truth is taken from state_groundtruth_estimate0/data.csv.
- Planar linear velocities are derived from consecutive ground-truth positions.
- Aligned IMU yaw rate is copied into /odom.twist.angular.z because current Aegis ROS wrappers read angular rate from odometry twist rather than /imu directly.
- use_odom_pose_update is disabled so ground-truth pose is used only for initialization and evaluation, not for continuous estimator correction.
- This backend is a real-data motion and timing benchmark, not a wheel-encoder-faithful EuRoC localization benchmark.

## Launch Issues

- [ukf_node-2] [ERROR] [1787016446.409270591] [ukf_node]: UKF update failed after 2019 processed odom updates; dt=0.00500019 state=[px=4.78892, py=-1.73995, theta=0.447093, vx=0.0953543, vy=0.422222, omega=0.136285] measurement=[vx=0.0895979, vy=0.41839, omega=0.11589] pose_measurement=[px=4.78886, py=-1.74057, yaw=2.67125] error=UKF covariance is not positive definite after jitter. diag=[2.03265, 2.03265, 4.94395, 0.0148127, 0.0148127, 0.0148127] min_eigenvalue=-0.0738245
- [ukf_node-2] UKF update failed after 2019 processed odom updates; dt=0.00500019 state=[px=4.78892, py=-1.73995, theta=0.447093, vx=0.0953543, vy=0.422222, omega=0.136285] measurement=[vx=0.0895979, vy=0.41839, omega=0.11589] pose_measurement=[px=4.78886, py=-1.74057, yaw=2.67125] error=UKF covariance is not positive definite after jitter. diag=[2.03265, 2.03265, 4.94395, 0.0148127, 0.0148127, 0.0148127] min_eigenvalue=-0.0738245
- [ukf_node-2] ukf_node fatal exception: UKF covariance is not positive definite after jitter. diag=[2.03265, 2.03265, 4.94395, 0.0148127, 0.0148127, 0.0148127] min_eigenvalue=-0.0738245
- [ERROR] [ukf_node-2]: process has died [pid 436, exit code 1, cmd '/mnt/c/Users/Ivan/dev/Aegis-Localization/ros2_ws/install/aegis_ros/lib/aegis_ros/ukf_node --ros-args -r __node:=ukf_node --params-file /mnt/c/Users/Ivan/dev/Aegis-Localization/ros2_ws/install/aegis_ros/share/aegis_ros/config/ukf.yaml --params-file /tmp/launch_params__zv1h88_'].

## Conclusions

- This run successfully produced canonical recorded-data outputs from EuRoC proxy replay.
- UKF coverage is incomplete because the UKF node terminated during the run after a non-positive-definite covariance failure.
- EKF and PF translation metrics are not sufficient to claim strong orientation tracking because the current motion model propagates x/y directly from world-frame vx/vy and does not couple translation to heading.
- Large raw yaw RMSE therefore reflects either true orientation drift, benchmark-definition mismatch, or both, and should be interpreted cautiously.
- This run is suitable as a Phase 1 recorded-data benchmark artifact and failure-analysis artifact, not yet as a final comparative estimator result.
