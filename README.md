# Aegis Localization

C++ ROS2 multi-sensor localization and state-estimation framework for autonomous robots.

Aegis Localization implements and benchmarks classical robotics estimation methods for real-time robot pose estimation using wheel odometry, IMU, and optional LiDAR-derived measurements.

## Features

- C++17 ROS2 localization stack
- Extended Kalman Filter (EKF)
- Unscented Kalman Filter (UKF)
- Particle Filter localization
- Optional GTSAM pose graph optimization
- TurtleBot3 Gazebo simulation support
- Trajectory logging and evaluation
- ATE/RPE/drift/runtime benchmarking

## Project Structure

- `ros2_ws/src/aegis_core` — filter math library
- `ros2_ws/src/aegis_ros` — ROS2 nodes, launch files, configs
- `ros2_ws/src/aegis_msgs` — custom diagnostics messages
- `scripts/` — evaluation and plotting tools
- `docs/` — architecture and experiment notes
- `results/` — generated metrics and plots

## Setup

```bash
cd aegis-localization
source /opt/ros/humble/setup.bash
rosdep update
rosdep install --from-paths ros2_ws/src --ignore-src -r -y
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
```

## Run EKF Localization

```bash
cd ros2_ws
source install/setup.bash
ros2 launch aegis_ros ekf_localization.launch.py
```

## Notes

This repository is a starting point for the Aegis Localization project. The first milestone ships a dummy EKF localization node that publishes `/aegis/ekf_pose` and `/aegis/ekf_path` while the full filter math library is built in `aegis_core`.
