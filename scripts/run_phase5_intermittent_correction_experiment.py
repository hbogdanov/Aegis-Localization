#!/usr/bin/env python3
"""Run a compact Phase 5 intermittent-correction experiment."""
from __future__ import annotations

import argparse
import json
import shutil
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
PYTHON_ROOT = REPO_ROOT / "python"
if str(PYTHON_ROOT) not in sys.path:
    sys.path.insert(0, str(PYTHON_ROOT))

from aegis_eval import load_filter_diagnostics_csv, summarize_nis


RESULTS_ROOT = REPO_ROOT / "results"
METRICS_ROOT = RESULTS_ROOT / "metrics"
PHASE5_ROOT = RESULTS_ROOT / "phase5_intermittent_correction"
METHODS = ["ekf", "ukf", "pf"]
CONSISTENCY_METHODS = ["ekf", "ukf"]


SCENARIOS = [
    {
        "name": "dead_reckoning_baseline",
        "launch_args": {
            "use_odom_pose_update": "false",
            "correction_enabled": "false",
            "odom_position_noise_std": "0.03",
            "odom_velocity_noise_std": "0.02",
            "imu_yaw_rate_noise_std": "0.01",
            "dropout_probability": "0.0",
        },
    },
    {
        "name": "intermittent_correction_stress",
        "launch_args": {
            "use_odom_pose_update": "false",
            "correction_enabled": "true",
            "correction_frequency_hz": "2.0",
            "correction_dropout_probability": "0.35",
            "correction_latency_seconds": "0.25",
            "correction_position_noise_std": "0.05",
            "correction_yaw_noise_std": "0.08",
            "correction_outlier_probability": "0.10",
            "correction_outlier_position_std": "1.5",
            "correction_outlier_yaw_std": "0.75",
            "odom_position_noise_std": "0.03",
            "odom_velocity_noise_std": "0.02",
            "imu_yaw_rate_noise_std": "0.01",
            "dropout_probability": "0.0",
        },
    },
]


def run(cmd: list[str], cwd: Path | None = None, check: bool = True) -> subprocess.CompletedProcess[str]:
    return subprocess.run(cmd, cwd=cwd, check=check, text=True)


def run_wsl(command: str, check: bool = True) -> subprocess.CompletedProcess[str]:
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


def read_json(path: Path) -> dict[str, object]:
    with path.open("r", encoding="utf-8") as handle:
        return json.load(handle)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=int, default=20)
    parser.add_argument("--base-seed", type=int, default=2600)
    args = parser.parse_args()

    PHASE5_ROOT.mkdir(parents=True, exist_ok=True)
    ros2_ws_wsl = to_wsl_path(REPO_ROOT / "ros2_ws")
    run_started_at = datetime.now(timezone.utc).isoformat()
    summary: dict[str, object] = {
      "duration_seconds": args.duration,
      "base_seed": args.base_seed,
      "run_started_at_utc": run_started_at,
      "scenarios": {},
    }

    for index, scenario in enumerate(SCENARIOS):
        scenario_root = PHASE5_ROOT / scenario["name"]
        if scenario_root.exists():
            shutil.rmtree(scenario_root)
        scenario_root.mkdir(parents=True, exist_ok=True)

        fake_sensor_seed = args.base_seed + index * 20
        pf_random_seed = fake_sensor_seed + 1
        launch_args = dict(scenario["launch_args"])
        launch_args["benchmark_duration_seconds"] = f"{float(args.duration):.1f}"
        launch_args["fake_sensor_seed"] = str(fake_sensor_seed)
        launch_args["pf_random_seed"] = str(pf_random_seed)
        launch_args["results_dir"] = to_wsl_path(scenario_root)
        launch_args["fake_sensor_stats_out"] = to_wsl_path(scenario_root / "fake_sensor_stats.json")
        launch_args["correction_log_out"] = to_wsl_path(scenario_root / "correction_log.csv")
        launch_args["ekf_stats_out"] = to_wsl_path(scenario_root / "ekf_stats.json")
        launch_args["ukf_stats_out"] = to_wsl_path(scenario_root / "ukf_stats.json")
        launch_args["pf_stats_out"] = to_wsl_path(scenario_root / "pf_stats.json")
        launch_args["logger_stats_out"] = to_wsl_path(scenario_root / "logger_stats.json")

        launch_items = [f"{key}:={value}" for key, value in launch_args.items()]
        clean_ros_nodes()
        launch_cmd = (
            "source /opt/ros/humble/setup.bash && "
            f"cd {ros2_ws_wsl} && "
            "source install/setup.bash && "
            f"timeout {args.duration + 8}s ros2 launch aegis_ros fake_benchmark.launch.py {' '.join(launch_items)}"
        )
        run_wsl(launch_cmd, check=False)
        clean_ros_nodes()

        metrics: dict[str, object] = {}
        for method in METHODS:
            est_path = scenario_root / f"{method}.csv"
            gt_path = scenario_root / "ground_truth.csv"
            out_json = scenario_root / f"{method}_metrics.json"
            run(
                [
                    sys.executable,
                    str(REPO_ROOT / "scripts" / "evaluate_trajectory.py"),
                    "--est",
                    str(est_path),
                    "--gt",
                    str(gt_path),
                    "--out-json",
                    str(out_json),
                ]
            )
            metrics[method] = read_json(out_json)

        consistency: dict[str, object] = {}
        diagnostics_path = scenario_root / "filter_diagnostics.csv"
        if diagnostics_path.exists():
            diagnostics = load_filter_diagnostics_csv(diagnostics_path)
            for method in CONSISTENCY_METHODS:
                nis = summarize_nis(diagnostics, estimator=method)
                consistency[method] = {
                    "pose": nis.get("measurements", {}).get("pose", {}),
                    "velocity_yaw_rate": nis.get("measurements", {}).get("velocity_yaw_rate", {}),
                }

        stats = {}
        for name in ["fake_sensor", "ekf", "ukf", "pf", "logger"]:
            path = scenario_root / f"{name}_stats.json"
            if path.exists():
                try:
                    stats[name] = read_json(path)
                except json.JSONDecodeError:
                    continue

        summary["scenarios"][scenario["name"]] = {
            "metadata": {
                "launch_args": launch_args,
                "seeds": {
                    "fake_sensor_seed": fake_sensor_seed,
                    "pf_random_seed": pf_random_seed,
                },
            },
            "metrics": metrics,
            "consistency": consistency,
            "stats": stats,
        }

    summary_path = PHASE5_ROOT / "summary.json"
    with summary_path.open("w", encoding="utf-8") as handle:
        json.dump(summary, handle, indent=2)
    print(f"Wrote Phase 5 summary to {summary_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
