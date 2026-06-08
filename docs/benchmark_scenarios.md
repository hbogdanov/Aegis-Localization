# Benchmark Scenarios

Use the fake benchmark to compare EKF and UKF under repeatable synthetic conditions.

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
