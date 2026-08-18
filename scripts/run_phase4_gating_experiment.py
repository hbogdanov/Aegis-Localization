#!/usr/bin/env python3
"""Run a focused Phase 4 pose-outlier gating experiment."""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
PYTHON_ROOT = REPO_ROOT / "python"
if str(PYTHON_ROOT) not in sys.path:
    sys.path.insert(0, str(PYTHON_ROOT))

from aegis_eval import load_filter_diagnostics_csv, summarize_gating, summarize_nis


RESULTS_ROOT = REPO_ROOT / "results" / "phase4_gating"
METHODS = ["ekf", "ukf", "pf"]
GATING_METHODS = ["ekf", "ukf"]
SCENARIOS = [
    {
        "name": "corrupted_pose_no_gating",
        "launch_args": {
            "odom_position_noise_std": "0.03",
            "odom_velocity_noise_std": "0.02",
            "imu_yaw_rate_noise_std": "0.01",
            "dropout_probability": "0.0",
            "use_odom_pose_update": "true",
            "pose_gating_enabled": "false",
            "pose_gating_threshold": "9.324146034653893",
            "pose_outlier_probability": "0.10",
            "pose_outlier_position_std": "1.5",
            "pose_outlier_yaw_std": "0.75",
        },
    },
    {
        "name": "corrupted_pose_with_gating",
        "launch_args": {
            "odom_position_noise_std": "0.03",
            "odom_velocity_noise_std": "0.02",
            "imu_yaw_rate_noise_std": "0.01",
            "dropout_probability": "0.0",
            "use_odom_pose_update": "true",
            "pose_gating_enabled": "true",
            "pose_gating_threshold": "9.324146034653893",
            "pose_outlier_probability": "0.10",
            "pose_outlier_position_std": "1.5",
            "pose_outlier_yaw_std": "0.75",
        },
    },
]


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


def load_json(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def load_json_when_ready(path: Path, *, retries: int = 20, delay_seconds: float = 0.25) -> dict | None:
    for _ in range(retries):
      if path.exists():
          try:
              text = path.read_text(encoding="utf-8").strip()
              if text:
                  return json.loads(text)
          except json.JSONDecodeError:
              pass
      time.sleep(delay_seconds)
    return None


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
    parser.add_argument("--duration", type=int, default=20)
    parser.add_argument("--repeats", type=int, default=5)
    parser.add_argument("--base-seed", type=int, default=1337)
    args = parser.parse_args()

    RESULTS_ROOT.mkdir(parents=True, exist_ok=True)
    summary = {
        "duration_seconds": args.duration,
        "repeats": args.repeats,
        "base_seed": args.base_seed,
        "run_started_at_utc": datetime.now(timezone.utc).isoformat(),
        "scenarios": {},
    }
    ros2_ws_wsl_path = to_wsl_path(REPO_ROOT / "ros2_ws")

    for scenario_index, scenario in enumerate(SCENARIOS):
        scenario_root = RESULTS_ROOT / scenario["name"]
        scenario_root.mkdir(parents=True, exist_ok=True)
        run_entries: list[dict[str, object]] = []

        for repeat_index in range(args.repeats):
            repeat_name = f"run_{repeat_index + 1:03d}"
            repeat_root = scenario_root / "repeats" / repeat_name
            repeat_root.mkdir(parents=True, exist_ok=True)
            fake_seed = args.base_seed + scenario_index * 1000 + repeat_index * 10
            pf_seed = fake_seed + 1

            results_dir_wsl = to_wsl_path(repeat_root)
            ekf_stats_wsl = to_wsl_path(repeat_root / "ekf_stats.json")
            ukf_stats_wsl = to_wsl_path(repeat_root / "ukf_stats.json")
            pf_stats_wsl = to_wsl_path(repeat_root / "pf_stats.json")
            logger_stats_wsl = to_wsl_path(repeat_root / "logger_stats.json")
            fake_sensor_stats_wsl = to_wsl_path(repeat_root / "fake_sensor_stats.json")

            launch_args = dict(scenario["launch_args"])
            launch_args["benchmark_duration_seconds"] = f"{float(args.duration):.1f}"
            launch_args["fake_sensor_seed"] = str(fake_seed)
            launch_args["pf_random_seed"] = str(pf_seed)
            launch_args["results_dir"] = results_dir_wsl
            launch_args["ekf_stats_out"] = ekf_stats_wsl
            launch_args["ukf_stats_out"] = ukf_stats_wsl
            launch_args["pf_stats_out"] = pf_stats_wsl
            launch_args["logger_stats_out"] = logger_stats_wsl
            launch_args["fake_sensor_stats_out"] = fake_sensor_stats_wsl
            launch_items = [f"{key}:={value}" for key, value in launch_args.items()]

            clean_ros_nodes()
            launch_cmd = (
                "source /opt/ros/humble/setup.bash && "
                f"cd {ros2_ws_wsl_path} && "
                "source install/setup.bash && "
                f"timeout {args.duration + 6}s ros2 launch aegis_ros fake_benchmark.launch.py {' '.join(launch_items)}"
            )
            run_wsl_bash(launch_cmd, check=False)
            clean_ros_nodes()

            metrics: dict[str, dict[str, object]] = {}
            gt_path = repeat_root / "ground_truth.csv"
            for method in METHODS:
                est_path = repeat_root / f"{method}.csv"
                out_json = repeat_root / f"{method}_metrics.json"
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
                metrics[method] = load_json(out_json)

            diagnostics_records = load_filter_diagnostics_csv(repeat_root / "filter_diagnostics.csv")
            gating_summary = {
                method: summarize_gating(diagnostics_records, estimator=method, measurement_type="pose")
                for method in GATING_METHODS
            }
            nis_summary = {
                method: summarize_nis(diagnostics_records, estimator=method)
                for method in GATING_METHODS
            }
            stats_summary = {
                "fake_sensor": load_json_when_ready(repeat_root / "fake_sensor_stats.json"),
                "ekf": load_json_when_ready(repeat_root / "ekf_stats.json"),
                "ukf": load_json_when_ready(repeat_root / "ukf_stats.json"),
                "pf": load_json_when_ready(repeat_root / "pf_stats.json"),
                "logger": load_json_when_ready(repeat_root / "logger_stats.json"),
            }

            repeat_entry = {
                "metadata": {
                    "repeat_name": repeat_name,
                    "duration_seconds": args.duration,
                    "launch_args": launch_args,
                    "seeds": {
                        "fake_sensor_seed": fake_seed,
                        "pf_random_seed": pf_seed,
                    },
                },
                "metrics": metrics,
                "gating": gating_summary,
                "nis": {
                    method: {
                        measurement_type: {
                            key: value
                            for key, value in measurement_summary.items()
                            if key not in {"timestamps", "nis_values", "in_bounds"}
                        }
                        for measurement_type, measurement_summary in summary_payload["measurements"].items()
                    }
                    for method, summary_payload in nis_summary.items()
                },
                "stats": stats_summary,
            }
            with (repeat_root / "phase4_summary.json").open("w", encoding="utf-8") as handle:
                json.dump(repeat_entry, handle, indent=2)
            run_entries.append(repeat_entry)

        aggregate: dict[str, object] = {"metrics": {}, "gating": {}}
        for method in METHODS:
            ate_values = [float(entry["metrics"][method]["ate_rmse"]) for entry in run_entries]
            drift_values = [float(entry["metrics"][method]["final_drift"]) for entry in run_entries]
            yaw_values = [float(entry["metrics"][method]["yaw_rmse"]) for entry in run_entries]
            aggregate["metrics"][method] = {
                "ate_rmse_mean": sum(ate_values) / len(ate_values),
                "final_drift_mean": sum(drift_values) / len(drift_values),
                "yaw_rmse_mean": sum(yaw_values) / len(yaw_values),
            }

        for method in GATING_METHODS:
            rejection_rates = [float(entry["gating"][method]["rejection_rate"]) for entry in run_entries]
            rejected_counts = [float(entry["gating"][method]["rejected_count"]) for entry in run_entries]
            aggregate["gating"][method] = {
                "rejection_rate_mean": sum(rejection_rates) / len(rejection_rates),
                "rejected_count_mean": sum(rejected_counts) / len(rejected_counts),
            }

        summary["scenarios"][scenario["name"]] = {
            "repeats": run_entries,
            "aggregate": aggregate,
        }

    with (RESULTS_ROOT / "summary.json").open("w", encoding="utf-8") as handle:
        json.dump(summary, handle, indent=2)
    print(f"Wrote Phase 4 summary to {RESULTS_ROOT / 'summary.json'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
