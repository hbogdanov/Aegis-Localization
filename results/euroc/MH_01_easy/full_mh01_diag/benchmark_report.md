# EuRoC Benchmark Report

- Run root: `C:\Users\Ivan\dev\Aegis-Localization\results\euroc\MH_01_easy\full_mh01_diag`
- Sequence: `MH_01_easy`
- Run name: `full_mh01_diag`
- Git commit: `76db690ec34c672fede6b22b208034930c391e7f`
- Truth source: `dataset`

## Estimator Coverage

| Estimator | Logged Samples | Coverage vs Replay |
|---|---:|---:|
| ekf | 31556 | 0.867 |
| ukf | 581 | 0.016 |
| pf | 31556 | 0.867 |

## Metrics

| Estimator | ATE RMSE | Final Drift | Yaw RMSE |
|---|---:|---:|---:|
| ekf | 0.001096 | 0.000606 | 1.843807 |
| ukf | 0.010822 | 0.012751 | 0.778895 |
| pf | 0.158269 | 0.166625 | 1.847630 |

## Yaw Analysis

This run should not be interpreted as a clean yaw-validation benchmark.

| Estimator | Circular Mean Offset | Centered Yaw RMSE | Raw Yaw RMSE |
|---|---:|---:|---:|
| ekf | 2.265698 | 1.666968 | 1.843807 |
| ukf | 0.685437 | 0.367735 | 0.778895 |
| pf | 2.238627 | 1.665254 | 1.847630 |

## Reduction Assumptions

- EuRoC is reduced to planar x, y, yaw motion only.
- Ground truth is taken from state_groundtruth_estimate0/data.csv.
- Planar linear velocities are derived from consecutive ground-truth positions.
- Aligned IMU yaw rate is copied into /odom.twist.angular.z because current Aegis ROS wrappers read angular rate from odometry twist rather than /imu directly.
- use_odom_pose_update is disabled so ground-truth pose is used only for initialization and evaluation, not for continuous estimator correction.
- This backend is a real-data motion and timing benchmark, not a wheel-encoder-faithful EuRoC localization benchmark.

## Launch Issues

- [ukf_node-2] [ERROR] [1787016219.932152795] [ukf_node]: UKF update failed after 2019 processed odom updates; dt=0.00499994 state=[px=4.56813, py=-1.88183, theta=0.546711, vx=0.316522, vy=-0.488011, omega=0.0481001] measurement=[vx=0.334392, vy=-0.45619, omega=0.0760964] pose_measurement=[px=4.56241, py=-1.87372, yaw=2.80317] error=UKF covariance is not positive definite after jitter. diag=[2.033, 2.033, 4.94115, 0.0148127, 0.0148127, 0.0148127] min_eigenvalue=-0.0738732
- [ukf_node-2] UKF update failed after 2019 processed odom updates; dt=0.00499994 state=[px=4.56813, py=-1.88183, theta=0.546711, vx=0.316522, vy=-0.488011, omega=0.0481001] measurement=[vx=0.334392, vy=-0.45619, omega=0.0760964] pose_measurement=[px=4.56241, py=-1.87372, yaw=2.80317] error=UKF covariance is not positive definite after jitter. diag=[2.033, 2.033, 4.94115, 0.0148127, 0.0148127, 0.0148127] min_eigenvalue=-0.0738732
- [ukf_node-2] ukf_node fatal exception: UKF covariance is not positive definite after jitter. diag=[2.033, 2.033, 4.94115, 0.0148127, 0.0148127, 0.0148127] min_eigenvalue=-0.0738732
- [ERROR] [ukf_node-2]: process has died [pid 447, exit code 1, cmd '/mnt/c/Users/Ivan/dev/Aegis-Localization/ros2_ws/install/aegis_ros/lib/aegis_ros/ukf_node --ros-args -r __node:=ukf_node --params-file /mnt/c/Users/Ivan/dev/Aegis-Localization/ros2_ws/install/aegis_ros/share/aegis_ros/config/ukf.yaml --params-file /tmp/launch_params_qdhn40t0'].

## Conclusions

- This run successfully produced canonical recorded-data outputs from EuRoC proxy replay.
- UKF coverage is incomplete because the UKF node terminated during the run after a non-positive-definite covariance failure.
- EKF and PF translation metrics are not sufficient to claim strong orientation tracking because the current motion model propagates x/y directly from world-frame vx/vy and does not couple translation to heading.
- Large raw yaw RMSE therefore reflects either true orientation drift, benchmark-definition mismatch, or both, and should be interpreted cautiously.
- This run is suitable as a Phase 1 recorded-data benchmark artifact and failure-analysis artifact, not yet as a final comparative estimator result.
