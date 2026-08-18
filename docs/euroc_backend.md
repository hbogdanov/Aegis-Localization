# EuRoC Backend

This backend is intentionally minimal.

It exists to prove that Aegis can run one public dataset sequence through the same estimator and evaluation path used by the synthetic benchmark.

## Current Scope

- one EuRoC sequence at a time
- planar proxy benchmark only
- current EKF, UKF, and PF implementations unchanged
- current ROS estimator wrappers unchanged

## What This Backend Does

1. reads EuRoC ground truth from `mav0/state_groundtruth_estimate0/data.csv`
2. reads EuRoC IMU yaw rate from `mav0/imu0/data.csv`
3. reduces the sequence to planar `x`, `y`, and `yaw`
4. derives planar linear velocity from consecutive ground-truth positions
5. aligns IMU yaw rate to the planar samples
6. replays the resulting proxy stream into `/odom`, `/imu`, and `/ground_truth/pose`
7. runs the existing Aegis estimator nodes and trajectory logger
8. writes canonical normalized outputs, metrics, and plots

## Honest Reduction Assumptions

This is **not** a wheel-encoder-faithful EuRoC localization benchmark.

It is a **planar proxy-odometry benchmark on real motion and timestamps**.

The key assumptions are:

- EuRoC 6-DoF motion is reduced to planar `x`, `y`, and `yaw`
- planar linear velocity is derived from ground-truth position differences
- aligned IMU yaw rate is copied into `/odom.twist.angular.z`

That last point is necessary because the current Aegis ROS wrappers still consume angular rate from odometry twist rather than directly fusing `/imu` in their update path.

## Command

From the repository root:

```bash
python scripts/run_euroc_benchmark.py --sequence-root /path/to/MH_01_easy
```

On the current Windows plus WSL setup, `/path/to/MH_01_easy` should be a Windows path when passed to the script from the repository root.

## Output Layout

Results are written under:

```text
results/euroc/<sequence_name>/proxy_planar/
```

with:

- `normalized/`
- `metrics/`
- `plots/`
- `logs/`
- `manifest.json`
- `metadata.json`

## Preserved Benchmark Artifacts

The `MH_01_easy` development history is intentionally preserved under:

- `results/euroc/MH_01_easy/full_mh01/` - first full recorded-data run artifact
- `results/euroc/MH_01_easy/full_mh01_diag/` - UKF diagnostic rerun with enriched crash logging
- `results/euroc/MH_01_easy/full_mh01_ready/` - replay-readiness-gated benchmark artifact
- `results/euroc/MH_01_easy/full_mh01_phase1_accounted/` - faithful-timing Phase 1 benchmark artifact with coverage accounting and stabilized UKF

Each run includes:

- canonical normalized CSV outputs
- `metrics/summary.json`
- `metrics/diagnostics.json`
- `benchmark_report.md`
- raw launch and replay logs

## Current Findings

On `MH_01_easy`, the backend now demonstrates:

- one documented command that reproduces a recorded-data benchmark run
- shared evaluation outputs in the same canonical layout used by synthetic benchmarks
- graceful degradation when plotting is unavailable on the local Python stack
- faithful replay metadata and coverage accounting
- a stabilized UKF path that no longer reproduces the earlier covariance crash on the Phase 1 accounted run

The earlier UKF failure is not hidden. In the readiness-era runs, the UKF terminated after roughly 2,019 odometry updates with a non-positive-definite covariance. The traced implementation-level causes were:

- process noise was being added each step without `dt` scaling, which inflated covariance under high-rate replay
- the previous `alpha=0.1` sigma-point setting yielded a very large negative central covariance weight
- the covariance update path was using a numerically fragile subtractive form without stage-level covariance validation

The current Phase 1 accounted artifact keeps the UKF finite and PSD through the full intended evaluation window.

The current yaw metric should also be treated carefully. The benchmark reduces a 6-DoF MAV sequence to planar `x`, `y`, `yaw`, and the present motion model propagates translation directly from `vx` and `vy` without coupling `x/y` propagation to heading. That means excellent translational ATE does **not** automatically imply credible yaw tracking.

The remaining recorded-data limitation is small but real sample loss at the artifact boundary. In `full_mh01_phase1_accounted`, replay publishes `36,382` samples while the final logged ground-truth CSV contains `36,285` rows. This is now accounted for and appears consistent with shutdown/write-path behavior rather than the earlier estimator instability, but it is not yet fully eliminated.

## Interpretation

If this backend performs well, the correct claim is:

> Aegis estimators can be evaluated on a planarized proxy stream derived from a public robotics dataset using the same shared evaluation path as the synthetic benchmark.

The correct claim is **not**:

> Aegis is fully validated on EuRoC as a native MAV localization system.

## External-Data Limitations

What the EuRoC proxy path proves:

- Aegis can replay one public recorded sequence through the same evaluation path used for synthetic benchmarks.
- EKF, UKF, and PF can be compared on the same timestamped planar proxy stream.
- The recorded-data path is useful for surfacing implementation weaknesses that synthetic benchmarks may hide.

What it does not prove:

- hardware validation
- wheel-encoder-faithful mobile-robot localization
- native 6-DoF MAV localization performance
- strong yaw-validation evidence under the current planar proxy

Why planarization was chosen:

- the current Aegis estimator stack is 2D
- planarization lets one public dataset exercise the shared benchmark path without rewriting the project into a different problem
- it creates a bridge from synthetic benchmarking to external recorded data with limited engineering overhead

Why this is still useful:

- it strengthens the credibility of the benchmark pipeline beyond toy-only synthetic runs
- it creates a reproducible recorded-data artifact that can be inspected, compared, and extended later
- it gives a concrete foundation for later work on stronger public-dataset support or heading-aware correction methods
