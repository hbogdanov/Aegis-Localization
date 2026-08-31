#!/usr/bin/env python3
"""Run focused Phase 5 correctness checks."""
from __future__ import annotations

import argparse
import json
import math
import shlex
import shutil
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path

import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[1]
PYTHON_ROOT = REPO_ROOT / "python"
if str(PYTHON_ROOT) not in sys.path:
    sys.path.insert(0, str(PYTHON_ROOT))

from aegis_eval import align_by_timestamp, load_filter_diagnostics_csv, load_trajectory_csv


RESULTS_ROOT = REPO_ROOT / "results"
OUTPUT_ROOT = RESULTS_ROOT / "phase5_correctness"
METHODS = ["ekf", "ukf", "pf"]


def run(cmd: list[str], *, cwd: Path | None = None, check: bool = True) -> subprocess.CompletedProcess[str]:
    return subprocess.run(cmd, cwd=cwd, check=check, text=True)


def run_wsl(command: str, *, check: bool = True) -> subprocess.CompletedProcess[str]:
    return run(["wsl.exe", "-d", "Ubuntu-22.04", "-e", "bash", "-lc", command], cwd=REPO_ROOT, check=check)


def to_wsl_path(path: Path) -> str:
    resolved = path.resolve()
    drive = resolved.drive.rstrip(":").lower()
    parts = [part for part in resolved.parts[1:] if part not in ("\\", "/")]
    return "/mnt/" + drive + "/" + "/".join(parts)


def clean_ros_nodes() -> None:
    cleanup_cmd = (
        "for pattern in '/aegis_ros/ekf_node' '/aegis_ros/ukf_node' "
        "'/aegis_ros/particle_filter_node' '/aegis_ros/trajectory_logger_node' "
        "'/aegis_ros/fake_sensor_publisher_node'; do "
        "pgrep -f \"$pattern\" | xargs -r kill; "
        "done; sleep 1"
    )
    run_wsl(cleanup_cmd, check=False)


def trajectory_comparison(a_path: Path, b_path: Path) -> dict[str, float]:
    a = load_trajectory_csv(a_path)
    b = load_trajectory_csv(b_path)
    a = dict(a)
    b = dict(b)
    a["timestamp"] = a["timestamp"] - a["timestamp"][0]
    b["timestamp"] = b["timestamp"] - b["timestamp"][0]
    merged = align_by_timestamp(a, b)
    dx = merged["x_est"] - merged["x_gt"]
    dy = merged["y_est"] - merged["y_gt"]
    dyaw = np.arctan2(np.sin(merged["yaw_est"] - merged["yaw_gt"]), np.cos(merged["yaw_est"] - merged["yaw_gt"]))
    pos = np.sqrt(dx * dx + dy * dy)
    return {
        "num_samples": int(merged["timestamp"].size),
        "final_position_error": float(pos[-1]),
        "max_position_error": float(np.max(pos)),
        "rmse_position_error": float(np.sqrt(np.mean(pos ** 2))),
        "final_yaw_error": float(dyaw[-1]),
        "max_abs_yaw_error": float(np.max(np.abs(dyaw))),
        "rmse_yaw_error": float(np.sqrt(np.mean(dyaw ** 2))),
    }


def final_state(path: Path) -> dict[str, float]:
    traj = load_trajectory_csv(path)
    return {
        "timestamp": float(traj["timestamp"][-1]),
        "x": float(traj["x"][-1]),
        "y": float(traj["y"][-1]),
        "yaw": float(traj["yaw"][-1]),
    }


def final_state_difference(a_path: Path, b_path: Path) -> dict[str, float]:
    a = final_state(a_path)
    b = final_state(b_path)
    dx = a["x"] - b["x"]
    dy = a["y"] - b["y"]
    dyaw = math.atan2(math.sin(a["yaw"] - b["yaw"]), math.cos(a["yaw"] - b["yaw"]))
    return {
        "position_error": float(math.hypot(dx, dy)),
        "yaw_error": float(dyaw),
        "dx": float(dx),
        "dy": float(dy),
        "timestamp_delta": float(a["timestamp"] - b["timestamp"]),
    }


def load_optional_json(path: Path) -> dict[str, object] | None:
    if not path.exists() or path.stat().st_size == 0:
        return None
    return json.loads(path.read_text(encoding="utf-8"))


def last_covariance(path: Path, estimator: str) -> np.ndarray | None:
    if not path.exists():
      return None
    records = load_filter_diagnostics_csv(path)
    filtered = [r for r in records if r.source.lower() == estimator.lower() and r.state_covariance.shape == (6, 6)]
    if not filtered:
      return None
    return filtered[-1].state_covariance


def covariance_comparison(a_path: Path, b_path: Path, estimator: str) -> dict[str, float] | None:
    a_cov = last_covariance(a_path, estimator)
    b_cov = last_covariance(b_path, estimator)
    if a_cov is None or b_cov is None:
        return None
    diff = a_cov - b_cov
    return {
        "frobenius_norm": float(np.linalg.norm(diff)),
        "max_abs_element": float(np.max(np.abs(diff))),
    }


def run_scenario(name: str, launch_args: dict[str, str], duration: int) -> Path:
    scenario_root = OUTPUT_ROOT / name
    if scenario_root.exists():
        shutil.rmtree(scenario_root)
    scenario_root.mkdir(parents=True, exist_ok=True)

    full_args = dict(launch_args)
    full_args["benchmark_duration_seconds"] = f"{float(duration):.1f}"
    full_args["results_dir"] = to_wsl_path(scenario_root)
    full_args["fake_sensor_stats_out"] = to_wsl_path(scenario_root / "fake_sensor_stats.json")
    full_args["correction_log_out"] = to_wsl_path(scenario_root / "correction_log.csv")
    full_args["ekf_stats_out"] = to_wsl_path(scenario_root / "ekf_stats.json")
    full_args["ukf_stats_out"] = to_wsl_path(scenario_root / "ukf_stats.json")
    full_args["pf_stats_out"] = to_wsl_path(scenario_root / "pf_stats.json")
    full_args["logger_stats_out"] = to_wsl_path(scenario_root / "logger_stats.json")

    launch_items = [shlex.quote(f"{key}:={value}") for key, value in full_args.items()]
    ros2_ws_wsl = to_wsl_path(REPO_ROOT / "ros2_ws")
    clean_ros_nodes()
    launch_cmd = (
        "source /opt/ros/humble/setup.bash && "
        f"cd {ros2_ws_wsl} && "
        "source install/setup.bash && "
        f"timeout {duration + 20}s ros2 launch aegis_ros fake_benchmark.launch.py {' '.join(launch_items)}"
    )
    run_wsl(launch_cmd, check=False)
    clean_ros_nodes()
    return scenario_root


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=int, default=8)
    args = parser.parse_args()

    OUTPUT_ROOT.mkdir(parents=True, exist_ok=True)

    zero_noise_common = {
        "fake_sensor_seed": "3100",
        "pf_random_seed": "3101",
        "odom_position_noise_std": "0.0",
        "odom_velocity_noise_std": "0.0",
        "imu_yaw_rate_noise_std": "0.0",
        "dropout_probability": "0.0",
        "pose_outlier_probability": "0.0",
        "correction_dropout_probability": "0.0",
        "correction_position_noise_std": "0.0",
        "correction_yaw_noise_std": "0.0",
        "correction_outlier_probability": "0.0",
        "correction_outlier_position_std": "0.0",
        "correction_outlier_yaw_std": "0.0",
    }

    immediate_root = run_scenario(
        "zero_latency_immediate",
        {
            **zero_noise_common,
            "use_odom_pose_update": "true",
            "correction_enabled": "false",
        },
        args.duration,
    )
    replay_root = run_scenario(
        "zero_latency_replay",
        {
            **zero_noise_common,
            "use_odom_pose_update": "false",
            "correction_enabled": "true",
            "correction_frequency_hz": "20.0",
            "correction_latency_seconds": "0.0",
        },
        args.duration,
    )

    zero_latency_summary: dict[str, object] = {}
    for method in METHODS:
        zero_latency_summary[method] = {
            "trajectory": trajectory_comparison(immediate_root / f"{method}.csv", replay_root / f"{method}.csv"),
        }
        if method in ("ekf", "ukf"):
            zero_latency_summary[method]["covariance"] = covariance_comparison(
                immediate_root / "filter_diagnostics.csv",
                replay_root / "filter_diagnostics.csv",
                method,
            )

    invariance_latencies = [
        ("arrival_0ms", "0.0"),
        ("arrival_100ms", "0.1"),
        ("arrival_500ms", "0.5"),
        ("arrival_1000ms", "1.0"),
    ]
    invariance_roots: dict[str, Path] = {}
    for name, latency in invariance_latencies:
        invariance_roots[name] = run_scenario(
            name,
            {
                **zero_noise_common,
                "use_odom_pose_update": "false",
                "correction_enabled": "true",
                "correction_frequency_hz": "5.0",
                "correction_latency_seconds": latency,
            },
            args.duration,
        )

    invariance_reference = invariance_roots["arrival_0ms"]
    arrival_invariance_summary: dict[str, object] = {}
    for name, _latency in invariance_latencies[1:]:
        comparison = {}
        for method in METHODS:
            comparison[method] = {
                "trajectory": trajectory_comparison(
                    invariance_reference / f"{method}.csv",
                    invariance_roots[name] / f"{method}.csv",
                )
            }
            if method in ("ekf", "ukf"):
                comparison[method]["covariance"] = covariance_comparison(
                    invariance_reference / "filter_diagnostics.csv",
                    invariance_roots[name] / "filter_diagnostics.csv",
                    method,
                )
        arrival_invariance_summary[name] = comparison

    ordering_common = {
        **zero_noise_common,
        "use_odom_pose_update": "false",
        "correction_enabled": "true",
        "correction_frequency_hz": "2.0",
        "correction_max_emissions": "2",
    }
    chronological_root = run_scenario(
        "reversed_order_reference",
        {
            **ordering_common,
            "correction_latency_schedule_seconds": "0.0,0.0",
        },
        args.duration,
    )
    reversed_root = run_scenario(
        "reversed_order_arrival",
        {
            **ordering_common,
            "correction_latency_schedule_seconds": "1.0,0.0",
        },
        args.duration,
    )

    reversed_arrival_summary: dict[str, object] = {}
    for method in METHODS:
        reversed_arrival_summary[method] = {
            "final_state": final_state_difference(
                chronological_root / f"{method}.csv",
                reversed_root / f"{method}.csv",
            )
        }
        if method in ("ekf", "ukf"):
            reversed_arrival_summary[method]["covariance"] = covariance_comparison(
                chronological_root / "filter_diagnostics.csv",
                reversed_root / "filter_diagnostics.csv",
                method,
            )

    stale_root = run_scenario(
        "history_window_rejection",
        {
            **zero_noise_common,
            "use_odom_pose_update": "false",
            "correction_enabled": "true",
            "correction_frequency_hz": "1.0",
            "correction_max_emissions": "2",
            "correction_latency_schedule_seconds": "0.5,0.0",
            "max_history_seconds": "0.2",
        },
        args.duration,
    )
    valid_only_root = run_scenario(
        "history_window_reference",
        {
            **zero_noise_common,
            "use_odom_pose_update": "false",
            "correction_enabled": "true",
            "correction_start_seconds": "0.75",
            "correction_frequency_hz": "1.0",
            "correction_max_emissions": "1",
            "max_history_seconds": "0.2",
        },
        args.duration,
    )

    history_window_summary: dict[str, object] = {}
    stale_stats = {
        "ekf": load_optional_json(stale_root / "ekf_stats.json"),
        "ukf": load_optional_json(stale_root / "ukf_stats.json"),
        "pf": load_optional_json(stale_root / "pf_stats.json"),
    }
    for method in METHODS:
        history_window_summary[method] = {
            "final_state": final_state_difference(
                valid_only_root / f"{method}.csv",
                stale_root / f"{method}.csv",
            ),
            "stale_run_stats": stale_stats[method],
        }
        if method in ("ekf", "ukf"):
            history_window_summary[method]["covariance"] = covariance_comparison(
                valid_only_root / "filter_diagnostics.csv",
                stale_root / "filter_diagnostics.csv",
                method,
            )

    summary = {
        "run_started_at_utc": datetime.now(timezone.utc).isoformat(),
        "duration_seconds": args.duration,
        "zero_latency_equivalence": zero_latency_summary,
        "arrival_time_invariance": arrival_invariance_summary,
        "reversed_arrival_order": reversed_arrival_summary,
        "history_window_rejection": history_window_summary,
    }

    summary_path = OUTPUT_ROOT / "summary.json"
    with summary_path.open("w", encoding="utf-8") as handle:
        json.dump(summary, handle, indent=2)
    print(f"Wrote Phase 5 correctness summary to {summary_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
