#!/usr/bin/env python3
"""Run named fake-benchmark scenarios and summarize EKF/UKF/PF metrics.

Usage:
  python3 scripts/run_fake_benchmark_campaign.py --duration 30
"""
from __future__ import annotations

import argparse
import csv
from datetime import datetime, timezone
import json
import shutil
import subprocess
import sys
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
RESULTS_ROOT = REPO_ROOT / "results"
METRICS_ROOT = RESULTS_ROOT / "metrics"
CAMPAIGN_ROOT = RESULTS_ROOT / "campaign"
SCENARIOS = [
    {
        "name": "low_noise",
        "launch_args": {
            "odom_position_noise_std": "0.03",
            "odom_velocity_noise_std": "0.02",
            "imu_yaw_rate_noise_std": "0.01",
            "dropout_probability": "0.0",
            "use_odom_pose_update": "true",
        },
    },
    {
        "name": "high_noise_dropout",
        "launch_args": {
            "odom_position_noise_std": "0.15",
            "odom_velocity_noise_std": "0.10",
            "imu_yaw_rate_noise_std": "0.05",
            "dropout_probability": "0.2",
            "use_odom_pose_update": "true",
        },
    },
    {
        "name": "dead_reckoning",
        "launch_args": {
            "odom_position_noise_std": "0.03",
            "odom_velocity_noise_std": "0.02",
            "imu_yaw_rate_noise_std": "0.01",
            "dropout_probability": "0.0",
            "use_odom_pose_update": "false",
        },
    },
]
METHODS = ["ekf", "ukf", "pf"]


def run(cmd: list[str], cwd: Path | None = None) -> None:
    print("+", " ".join(cmd))
    subprocess.run(cmd, cwd=cwd, check=True)


def run_wsl_bash(command: str, cwd: Path | None = None, check: bool = True) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        ["wsl.exe", "-d", "Ubuntu-22.04", "-e", "bash", "-lc", command],
        cwd=cwd,
        check=check,
        text=True,
    )


def to_wsl_path(path: Path) -> str:
    resolved = path.resolve()
    drive = resolved.drive.rstrip(":").lower()
    parts = [part for part in resolved.parts[1:] if part not in ("\\", "/")]
    return "/mnt/" + drive + "/" + "/".join(parts)


def compute_update_rate(csv_path: Path) -> float:
    timestamps = []
    with csv_path.open("r", encoding="utf-8", newline="") as handle:
        reader = csv.DictReader(handle)
        if reader.fieldnames is None or "timestamp" not in reader.fieldnames:
            return float("nan")
        for row in reader:
            try:
                timestamps.append(float(row["timestamp"]))
            except (TypeError, ValueError):
                continue
    if len(timestamps) < 2:
        return float("nan")
    timestamps.sort()
    duration = float(timestamps[-1] - timestamps[0])
    if duration <= 0.0:
        return float("nan")
    return float((len(timestamps) - 1) / duration)


def get_git_commit() -> str | None:
    result = subprocess.run(
        ["git", "rev-parse", "HEAD"],
        cwd=REPO_ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    if result.returncode != 0:
        return None
    return result.stdout.strip() or None


def require_samples(csv_path: Path, minimum_rows: int = 2) -> None:
    if not csv_path.exists():
        raise FileNotFoundError(csv_path)

    with csv_path.open("r", encoding="utf-8", newline="") as handle:
        reader = csv.reader(handle)
        row_count = sum(1 for _ in reader)

    if row_count < minimum_rows + 1:
        raise RuntimeError(
            f"CSV did not contain enough trajectory samples: {csv_path} "
            f"(found {max(row_count - 1, 0)}, need at least {minimum_rows})"
        )


def clean_ros_nodes() -> None:
    cleanup_cmd = (
        "for pattern in '/aegis_ros/ekf_node' '/aegis_ros/ukf_node' "
        "'/aegis_ros/particle_filter_node' '/aegis_ros/trajectory_logger_node' "
        "'/aegis_ros/fake_sensor_publisher_node'; do "
        "pgrep -f \"$pattern\" | xargs -r kill; "
        "done; "
        "sleep 1"
    )
    run_wsl_bash(cleanup_cmd, cwd=REPO_ROOT / "ros2_ws", check=False)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=int, default=30, help="Scenario runtime in seconds")
    parser.add_argument("--base-seed", type=int, default=1337, help="Base seed used to derive deterministic scenario seeds")
    args = parser.parse_args()

    CAMPAIGN_ROOT.mkdir(parents=True, exist_ok=True)
    git_commit = get_git_commit()
    run_started_at = datetime.now(timezone.utc).isoformat()
    summary = {
        "duration_seconds": args.duration,
        "base_seed": args.base_seed,
        "git_commit": git_commit,
        "run_started_at_utc": run_started_at,
        "scenarios": {},
    }
    ros2_ws_wsl_path = to_wsl_path(REPO_ROOT / "ros2_ws")

    for index, scenario in enumerate(SCENARIOS):
        name = scenario["name"]
        fake_sensor_seed = args.base_seed + index * 100
        pf_random_seed = fake_sensor_seed + 1
        launch_args = dict(scenario["launch_args"])
        launch_args["benchmark_duration_seconds"] = f"{float(args.duration):.1f}"
        launch_args["fake_sensor_seed"] = str(fake_sensor_seed)
        launch_args["pf_random_seed"] = str(pf_random_seed)
        launch_items = [f"{k}:={v}" for k, v in launch_args.items()]
        scenario_root = CAMPAIGN_ROOT / name
        scenario_root.mkdir(parents=True, exist_ok=True)

        clean_ros_nodes()
        launch_cmd = (
            "source /opt/ros/humble/setup.bash && "
            f"cd {ros2_ws_wsl_path} && "
            "source install/setup.bash && "
            f"timeout {args.duration + 6}s ros2 launch aegis_ros fake_benchmark.launch.py {' '.join(launch_items)}"
        )
        run_wsl_bash(launch_cmd, check=False)
        clean_ros_nodes()

        scenario_metrics = {}
        gt_path = METRICS_ROOT / "ground_truth.csv"
        require_samples(gt_path)
        shutil.copy2(gt_path, scenario_root / "ground_truth.csv")

        for method in METHODS:
            est_path = METRICS_ROOT / f"{method}.csv"
            require_samples(est_path)
            out_json = scenario_root / f"{method}_metrics.json"
            shutil.copy2(est_path, scenario_root / f"{method}.csv")
            run([
                sys.executable,
                str(REPO_ROOT / "scripts" / "evaluate_trajectory.py"),
                "--est",
                str(est_path),
                "--gt",
                str(gt_path),
                "--out-json",
                str(out_json),
            ], cwd=REPO_ROOT)
            with out_json.open("r", encoding="utf-8") as handle:
                metrics = json.load(handle)
            metrics["update_rate_hz"] = compute_update_rate(est_path)
            scenario_metrics[method] = metrics

        scenario_metadata = {
            "name": name,
            "duration_seconds": args.duration,
            "git_commit": git_commit,
            "run_started_at_utc": run_started_at,
            "launch_args": launch_args,
            "seeds": {
                "fake_sensor_seed": fake_sensor_seed,
                "pf_random_seed": pf_random_seed,
            },
        }
        with (scenario_root / "metadata.json").open("w", encoding="utf-8") as handle:
            json.dump(scenario_metadata, handle, indent=2)

        summary["scenarios"][name] = {
            "metadata": scenario_metadata,
            "metrics": scenario_metrics,
        }

    summary_path = CAMPAIGN_ROOT / "summary.json"
    with summary_path.open("w", encoding="utf-8") as handle:
        json.dump(summary, handle, indent=2)
    print(f"Wrote campaign summary to {summary_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
