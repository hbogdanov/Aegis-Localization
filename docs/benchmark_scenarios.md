# Benchmark Scenarios

Use the fake benchmark to compare EKF, UKF, and PF under repeatable synthetic conditions.

For Phase 3 consistency analysis, the benchmark also logs per-update filter diagnostics under `results/metrics/filter_diagnostics.csv`. The campaign runner uses that artifact to compute EKF and UKF NIS summaries, 95% chi-square bounds, and synthetic-only planar NEES summaries.

## Low Noise

```bash
cd ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch aegis_ros fake_benchmark.launch.py \
  odom_position_noise_std:=0.03 \
  odom_velocity_noise_std:=0.02 \
  imu_yaw_rate_noise_std:=0.01 \
  dropout_probability:=0.0
```

## Medium Noise

```bash
cd ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch aegis_ros fake_benchmark.launch.py \
  odom_position_noise_std:=0.08 \
  odom_velocity_noise_std:=0.05 \
  imu_yaw_rate_noise_std:=0.03 \
  dropout_probability:=0.0
```

## High Noise

```bash
cd ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch aegis_ros fake_benchmark.launch.py \
  odom_position_noise_std:=0.15 \
  odom_velocity_noise_std:=0.10 \
  imu_yaw_rate_noise_std:=0.05 \
  dropout_probability:=0.0
```

## 20 Percent Dropout

```bash
cd ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch aegis_ros fake_benchmark.launch.py \
  odom_position_noise_std:=0.15 \
  odom_velocity_noise_std:=0.10 \
  imu_yaw_rate_noise_std:=0.05 \
  dropout_probability:=0.2
```

## Dead-Reckoning Stress Test

```bash
cd ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch aegis_ros fake_benchmark.launch.py use_odom_pose_update:=false
```

This last scenario disables odometry pose correction for both EKF and UKF so the benchmark reflects drift under velocity and yaw-rate integration alone.

## Repeated-Run Evidence

For the reproducible synthetic evidence path, run:

```bash
python3 scripts/run_fake_benchmark_campaign.py --duration 20 --repeats 5
```

That campaign preserves per-run artifacts and aggregates:

- mean and standard deviation of ATE RMSE
- mean and standard deviation of final drift
- mean and standard deviation of yaw RMSE
- mean and standard deviation of update rate
- mean and standard deviation of NIS fraction-in-bounds for EKF and UKF
