# EuRoC Benchmark Report

- Run root: `C:\Users\Ivan\dev\Aegis-Localization\results\euroc\MH_01_easy\full_mh01`
- Sequence: `MH_01_easy`
- Run name: `full_mh01`
- Git commit: `76db690ec34c672fede6b22b208034930c391e7f`
- Truth source: `dataset`

## Estimator Coverage

| Estimator | Logged Samples | Coverage vs Replay |
|---|---:|---:|
| ekf | 34306 | 0.943 |
| ukf | 1078 | 0.030 |
| pf | 34318 | 0.943 |

## Metrics

| Estimator | ATE RMSE | Final Drift | Yaw RMSE |
|---|---:|---:|---:|
| ekf | 0.000619 | 0.000381 | 1.831803 |
| ukf | 0.004390 | 0.006032 | 0.496487 |
| pf | 0.157417 | 0.167890 | 1.836140 |

## Yaw Analysis

This run should not be interpreted as a clean yaw-validation benchmark.

| Estimator | Circular Mean Offset | Centered Yaw RMSE | Raw Yaw RMSE |
|---|---:|---:|---:|
| ekf | 2.019234 | 1.672697 | 1.831803 |
| ukf | 0.377878 | 0.317374 | 0.496487 |
| pf | 2.010156 | 1.671716 | 1.836140 |

## Reduction Assumptions

- EuRoC is reduced to planar x, y, yaw motion only.
- Ground truth is taken from state_groundtruth_estimate0/data.csv.
- Planar linear velocities are derived from consecutive ground-truth positions.
- Aligned IMU yaw rate is copied into /odom.twist.angular.z because current Aegis ROS wrappers read angular rate from odometry twist rather than /imu directly.
- use_odom_pose_update is disabled so ground-truth pose is used only for initialization and evaluation, not for continuous estimator correction.
- This backend is a real-data motion and timing benchmark, not a wheel-encoder-faithful EuRoC localization benchmark.

## Launch Issues

- [ukf_node-2] terminate called after throwing an instance of 'std::runtime_error'
- [ukf_node-2]   what():  UKF covariance is not positive definite
- [ERROR] [ukf_node-2]: process has died [pid 451, exit code -6, cmd '/mnt/c/Users/Ivan/dev/Aegis-Localization/ros2_ws/install/aegis_ros/lib/aegis_ros/ukf_node --ros-args -r __node:=ukf_node --params-file /mnt/c/Users/Ivan/dev/Aegis-Localization/ros2_ws/install/aegis_ros/share/aegis_ros/config/ukf.yaml --params-file /tmp/launch_params_my8lnrt2'].

## Conclusions

- This run successfully produced canonical recorded-data outputs from EuRoC proxy replay.
- UKF coverage is incomplete because the UKF node terminated during the run after a non-positive-definite covariance failure.
- EKF and PF translation metrics are not sufficient to claim strong orientation tracking because the current motion model propagates x/y directly from world-frame vx/vy and does not couple translation to heading.
- Large raw yaw RMSE therefore reflects either true orientation drift, benchmark-definition mismatch, or both, and should be interpreted cautiously.
- This run is suitable as a Phase 1 recorded-data benchmark artifact and failure-analysis artifact, not yet as a final comparative estimator result.
