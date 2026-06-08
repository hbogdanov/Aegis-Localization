# Aegis Localization

C++ ROS2 multi-sensor localization and state-estimation framework for autonomous robots.

Aegis Localization implements and benchmarks classical robotics estimation methods for real-time robot pose estimation using wheel odometry, IMU, and optional LiDAR-derived measurements.

## Features

- C++17 ROS2 localization stack
- Extended Kalman Filter (EKF)
- Unscented Kalman Filter (UKF)
- Particle Filter localization
- Odometry + IMU fusion
- Synthetic sensor benchmark with configurable noise/dropout
- Ground-truth trajectory logging
- ATE/drift/yaw RMSE evaluation
- Trajectory plotting

## Planned

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

## Run UKF Localization

```bash
cd ros2_ws
source install/setup.bash
ros2 launch aegis_ros ukf_localization.launch.py
```

## Run Particle Filter Localization

```bash
cd ros2_ws
source install/setup.bash
ros2 launch aegis_ros particle_filter_localization.launch.py
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
- `particle_filter_node`
- `trajectory_logger_node`

The trajectory logger writes output into the repository-level `results/metrics` folder.

Default fake-sensor noise settings are provided in `ros2_ws/src/aegis_ros/config/fake_sensor_publisher.yaml`.
Use `run_ekf:=false`, `run_ukf:=false`, or `run_pf:=false` to benchmark just selected filters.

## Benchmark Scenarios

Scenario reference: `docs/benchmark_scenarios.md`

Low noise:

```bash
cd ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch aegis_ros fake_benchmark.launch.py odom_position_noise_std:=0.03 odom_velocity_noise_std:=0.02 imu_yaw_rate_noise_std:=0.01 dropout_probability:=0.0
```

Medium noise:

```bash
cd ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch aegis_ros fake_benchmark.launch.py odom_position_noise_std:=0.08 odom_velocity_noise_std:=0.05 imu_yaw_rate_noise_std:=0.03 dropout_probability:=0.0
```

High noise:

```bash
cd ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch aegis_ros fake_benchmark.launch.py odom_position_noise_std:=0.15 odom_velocity_noise_std:=0.10 imu_yaw_rate_noise_std:=0.05 dropout_probability:=0.0
```

20 percent dropout:

```bash
cd ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch aegis_ros fake_benchmark.launch.py odom_position_noise_std:=0.15 odom_velocity_noise_std:=0.10 imu_yaw_rate_noise_std:=0.05 dropout_probability:=0.2
```

Dead-reckoning stress test:

```bash
cd ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch aegis_ros fake_benchmark.launch.py use_odom_pose_update:=false
```

## Evaluate and Plot Trajectories

After running the benchmark, evaluate and plot the recorded trajectories from `results/metrics`:

```bash
cd /mnt/c/Users/Ivan/Aegis-Localization
python3 scripts/evaluate_trajectory.py --est results/metrics/ekf.csv --gt results/metrics/ground_truth.csv
python3 scripts/evaluate_trajectory.py --est results/metrics/ukf.csv --gt results/metrics/ground_truth.csv
python3 scripts/evaluate_trajectory.py --est results/metrics/pf.csv --gt results/metrics/ground_truth.csv
python3 scripts/plot_trajectories.py --est results/metrics/ekf.csv --gt results/metrics/ground_truth.csv --out results/plots/ekf_vs_ground_truth.png
python3 scripts/plot_trajectories.py --est results/metrics/pf.csv --gt results/metrics/ground_truth.csv --out results/plots/pf_vs_ground_truth.png
```

The plot script also writes:

- `results/plots/ekf_vs_ground_truth.png`
- `results/plots/ekf_position_error.png`

## Benchmark Evidence

After a successful 60-second fake benchmark run, the evaluation artifacts are:

- `results/metrics/ekf.csv`
- `results/metrics/ukf.csv`
- `results/metrics/pf.csv`
- `results/metrics/ground_truth.csv`
- `results/metrics/ekf_metrics.json`
- `results/metrics/ukf_metrics.json`
- `results/metrics/pf_metrics.json`
- `results/plots/ekf_vs_ground_truth.png`
- `results/plots/pf_vs_ground_truth.png`
- `results/plots/ekf_position_error.png`
- `results/plots/pf_position_error.png`

Validated metrics from the current synthetic benchmark:

| Method | ATE RMSE | Final Drift | Yaw RMSE |
|---|---:|---:|---:|
| EKF | 0.0220 m | 0.0265 m | 0.0019 rad |
| UKF | 0.0220 m | 0.0265 m | 0.0019 rad |
| Particle Filter | 0.0253 m | 0.0071 m | 0.0019 rad |

These files provide direct evidence for comparing EKF, UKF, and particle-filter behavior under the same synthetic trajectory and noise settings.

## Notes

The localization nodes initialize from the first odometry pose, optionally apply odometry pose corrections through `use_odom_pose_update`, fuse `/odom` and `/imu`, and publish filter-specific pose/path topics for benchmarking.
