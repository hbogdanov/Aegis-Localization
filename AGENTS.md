# AGENTS.md

## Project purpose
Aegis Localization solves real-time robot pose estimation for autonomous
systems that need fused state estimates from wheel odometry, IMU, and optional
LiDAR-derived measurements. The repository exists to compare classical
localization methods in a reproducible ROS2 workflow and to generate benchmark
evidence for estimator accuracy, drift, and robustness under noise and
dropout.

## Current status
Working now:
- C++17 ROS2 localization stack in `ros2_ws/`
- EKF, UKF, and particle-filter localization pipelines
- synthetic benchmark launch with configurable noise, dropout, and logging
- trajectory evaluation and plotting scripts
- TurtleBot3 Gazebo validation launch with estimator logging in simulation

Incomplete or planned:
- GTSAM pose graph optimization is planned, not implemented
- Gazebo scoring is still limited because the simulator ground-truth bridge is
  not yet producing a populated `ground_truth.csv`

Simulation versus deployment:
- synthetic benchmark runs are simulated
- TurtleBot3 Gazebo validation is simulated ROS2 integration
- do not present this repository as hardware-validated unless real robot runs
  are added and documented

## Setup
```bash
source /opt/ros/humble/setup.bash
rosdep update
cd ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## Build and test commands
- Build: `cd ros2_ws && source /opt/ros/humble/setup.bash && colcon build --symlink-install`
- Unit tests: `cd ros2_ws && source /opt/ros/humble/setup.bash && colcon test`
- Integration tests: `cd ros2_ws && source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch aegis_ros fake_benchmark.launch.py`
- Benchmarks: `python3 scripts/run_fake_benchmark_campaign.py --duration 20`
- Lint/type checking: `cd ros2_ws && source /opt/ros/humble/setup.bash && colcon test --packages-select aegis_core aegis_ros`

## Architecture
- Preserve modular ROS2 node boundaries.
- Do not introduce new dependencies without justification.
- Prefer testable interfaces over tightly coupled scripts.
- Do not claim hardware validation unless hardware was actually used.
- Separate estimator mathematics, sensor adapters, ROS2 integration,
  evaluation, logging, and visualization.
- Keep estimator math, ROS2 wrappers, messages, and evaluation tooling
  separated across `aegis_core`, `aegis_ros`, `aegis_msgs`, and `scripts/`.
- Preserve reproducible benchmark outputs under `results/` and evidence docs
  under `docs/`.

## Benchmark conventions
- Store generated benchmark outputs under `results/`.
- Use fixed random seeds for comparisons unless randomness is the subject of
  the experiment.
- Do not overwrite reference results without explicit approval.
- Record estimator configuration, noise parameters, seed, runtime, and commit
  hash for benchmark runs whenever practical.
- Treat synthetic benchmark results and Gazebo validation results as separate
  evaluation categories.
- Keep committed evidence plots and summary tables aligned with the commands in
  `scripts/` and the scenario documentation in `docs/`.

## Evaluation metrics
Primary metrics:
- absolute trajectory error
- relative pose error when available
- translational RMSE
- rotational or yaw error
- drift over distance or final drift
- runtime and update frequency
- failure rate under dropout and noise

Comparisons must use identical trajectories, seeds, sensor rates, and noise
settings unless the experiment explicitly studies one of those variables.

## Known limitations
- Gazebo scoring is currently limited by the missing populated
  `ground_truth.csv` bridge output.
- Planned pose-graph optimization is not implemented yet.
- Current validation evidence is simulation-only.

## Review priorities
1. Correctness bugs
2. Architectural weaknesses
3. Missing tests
4. Reproducibility
5. Performance bottlenecks
6. Robotics and research relevance
7. Documentation gaps

## Career target
Evaluate this repository for robotics perception, computer vision,
robotics software, and graduate research applications.

Best fit:
- robotics software roles with ROS2 and state-estimation focus
- autonomy and localization engineering internships or research roles
- graduate applications that benefit from reproducible benchmarking and
  estimator comparisons

Less central:
- pure computer-vision portfolios, since the core work here is localization
  rather than image understanding

## Protected areas

Do not modify generated benchmark evidence, reference plots, or archived
results unless the task explicitly concerns regeneration or correction.

Do not replace estimator implementations with library wrappers merely to reduce
code size unless explicitly requested.
