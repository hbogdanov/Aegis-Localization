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

## Phase 5 Delayed Correction Evidence

The delayed-correction research path uses pose-like corrections with a measurement timestamp and a possibly later arrival timestamp.

```bash
python3 scripts/run_phase5_replay_comparison.py --duration 12 --repeats 3
python3 scripts/run_phase5_degradation_campaign.py --duration 12 --repeats 3
python3 scripts/generate_phase5_final_report.py
```

`run_phase5_replay_comparison.py` compares naive fusion at arrival time with timestamp-aware historical replay. Its ATE is the online published-trajectory result; final drift is the post-fusion current-state result.

`run_phase5_degradation_campaign.py` evaluates replay under low correction frequency, random dropout, latency, noise, outliers with and without gating, a timed correction blackout, and a combined degraded condition. The blackout condition records error growth during the outage and error one second after corrections resume.
