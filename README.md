# Aegis Localization

C++ ROS2 multi-sensor localization and state-estimation framework for autonomous robots.

Aegis Localization implements and benchmarks classical robotics estimation methods for real-time robot pose estimation using wheel odometry, IMU, and optional LiDAR-derived measurements.

## Features

- C++17 ROS2 localization stack
- Extended Kalman Filter (EKF)
- Unscented Kalman Filter (UKF)
- Odometry + IMU fusion
- Synthetic sensor benchmark with configurable noise/dropout
- Ground-truth trajectory logging
- ATE/drift/yaw RMSE evaluation
- Trajectory plotting

## Planned

- Particle Filter localization
- TurtleBot3 Gazebo validation
- GTSAM pose graph optimization

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

## Run Benchmark and Trajectory Logging

```bash
cd ros2_ws
source install/setup.bash
ros2 launch aegis_ros benchmark.launch.py
```

The benchmark launch starts the EKF node and the trajectory logger, which writes results to:

- `results/metrics/ekf.csv`
- `results/metrics/odom.csv`

## Run Fake Sensor Benchmark

If you do not want to use Gazebo or TurtleBot3, run a fake sensor benchmark instead:

```bash
cd ros2_ws
source install/setup.bash
ros2 launch aegis_ros fake_benchmark.launch.py
```

This launch starts:

- `fake_sensor_publisher_node` publishing `/odom`, `/imu`, and noiseless `/ground_truth/pose`
- `ekf_node`
- `ukf_node`
- `trajectory_logger_node`

The trajectory logger writes output into the repository-level `results/metrics` folder.

Default fake-sensor noise settings are provided in `ros2_ws/src/aegis_ros/config/fake_sensor_publisher.yaml`.
Use `run_ekf:=false` or `run_ukf:=false` to benchmark just one filter.

## Evaluate and Plot Trajectories

After running the benchmark, evaluate and plot the recorded trajectories from `results/metrics`:

```bash
cd /mnt/c/Users/Ivan/Aegis-Localization
python3 scripts/evaluate_trajectory.py --est results/metrics/ekf.csv --gt results/metrics/ground_truth.csv
python3 scripts/plot_trajectories.py --est results/metrics/ekf.csv --gt results/metrics/ground_truth.csv --out results/plots/ekf_vs_ground_truth.png
```

The plot script also writes:

- `results/plots/ekf_vs_ground_truth.png`
- `results/plots/ekf_position_error.png`

## Benchmark Evidence

After a successful 60-second fake benchmark run, the evaluation artifacts are:

- `results/metrics/ekf.csv`
- `results/metrics/ukf.csv`
- `results/metrics/ground_truth.csv`
- `results/metrics/ekf_metrics.json`
- `results/metrics/ukf_metrics.json`
- `results/plots/ekf_vs_ground_truth.png`
- `results/plots/ekf_position_error.png`

Validated metrics from the current synthetic benchmark:

- EKF ATE RMSE: `0.0206` m
- EKF final drift: `0.0200` m
- EKF yaw RMSE: `0.0019` rad
- UKF ATE RMSE: `0.0206` m
- UKF final drift: `0.0200` m
- UKF yaw RMSE: `0.0019` rad

These files provide direct evidence that the EKF tracks the 1 m circular trajectory closely under the default fake-sensor noise model.

## Notes

The EKF node initializes from the first odometry pose, optionally applies odometry pose corrections through `use_odom_pose_update`, fuses `/odom` and `/imu`, publishes `/aegis/ekf_pose` and `/aegis/ekf_path`, and emits diagnostics on `/aegis/diagnostics`.
