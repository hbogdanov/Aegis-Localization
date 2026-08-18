#!/usr/bin/env python3
"""Run a Phase 4b gating sweep with corruption provenance."""

from __future__ import annotations

import argparse
import json
import math
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
PYTHON_ROOT = REPO_ROOT / "python"
if str(PYTHON_ROOT) not in sys.path:
    sys.path.insert(0, str(PYTHON_ROOT))

from aegis_eval import (
    load_corruption_log_csv,
    load_filter_diagnostics_csv,
    summarize_gate_classification,
    summarize_gating,
    summarize_nis,
)


RESULTS_ROOT = REPO_ROOT / "results" / "phase4b_gating_sweep"
METHODS = ["ekf", "ukf", "pf"]
GATING_METHODS = ["ekf", "ukf"]


def aggregate_series(values: list[float]) -> dict[str, float]:
    if not values:
        return {"mean": float("nan"), "std": float("nan")}
    mean = sum(values) / len(values)
    variance = sum((value - mean) ** 2 for value in values) / len(values)
    return {"mean": mean, "std": math.sqrt(variance)}


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


def build_scenarios() -> list[dict[str, object]]:
    scenarios: list[dict[str, object]] = [
        {
            "name": "startup_poison_gate95",
            "group": "diagnosis",
            "gating_enabled": True,
            "pose_gating_threshold": 9.324146034653893,
            "pose_outlier_probability": 0.10,
            "pose_outlier_position_std": 1.5,
            "pose_outlier_yaw_std": 0.75,
            "pose_outlier_start_seconds": 0.0,
        },
        {
            "name": "delayed_corruption_gate95",
            "group": "diagnosis",
            "gating_enabled": True,
            "pose_gating_threshold": 9.324146034653893,
            "pose_outlier_probability": 0.10,
            "pose_outlier_position_std": 1.5,
            "pose_outlier_yaw_std": 0.75,
            "pose_outlier_start_seconds": 1.0,
        },
    ]

    thresholds = [
        ("95", 9.324146034653893),
        ("99", 11.344866730144373),
        ("999", 16.26623619623813),
    ]
    outlier_probabilities = [0.05, 0.10]
    outlier_magnitudes = [
        ("mild", 1.0, 0.5),
        ("strong", 1.5, 0.75),
    ]

    for probability in outlier_probabilities:
        for magnitude_name, position_std, yaw_std in outlier_magnitudes:
            baseline_name = f"baseline_p{int(probability * 100):02d}_{magnitude_name}"
            scenarios.append(
                {
                    "name": baseline_name,
                    "group": "baseline",
                    "gating_enabled": False,
                    "pose_gating_threshold": 9.324146034653893,
                    "pose_outlier_probability": probability,
                    "pose_outlier_position_std": position_std,
                    "pose_outlier_yaw_std": yaw_std,
                    "pose_outlier_start_seconds": 1.0,
                }
            )
            for threshold_name, threshold_value in thresholds:
                scenarios.append(
                    {
                        "name": f"gate_{threshold_name}_p{int(probability * 100):02d}_{magnitude_name}",
                        "group": "sweep",
                        "gating_enabled": True,
                        "pose_gating_threshold": threshold_value,
                        "pose_outlier_probability": probability,
                        "pose_outlier_position_std": position_std,
                        "pose_outlier_yaw_std": yaw_std,
                        "pose_outlier_start_seconds": 1.0,
                    }
                )

    return scenarios


def aggregate_run_entries(run_entries: list[dict[str, object]]) -> dict[str, object]:
    aggregate: dict[str, object] = {"metrics": {}, "gating": {}, "classification": {}, "nis": {}}

    for method in METHODS:
        aggregate["metrics"][method] = {
            "ate_rmse": aggregate_series([float(entry["metrics"][method]["ate_rmse"]) for entry in run_entries]),
            "final_drift": aggregate_series([float(entry["metrics"][method]["final_drift"]) for entry in run_entries]),
            "yaw_rmse": aggregate_series([float(entry["metrics"][method]["yaw_rmse"]) for entry in run_entries]),
        }

    for method in GATING_METHODS:
        aggregate["gating"][method] = {
            "rejection_rate": aggregate_series([float(entry["gating"][method]["rejection_rate"]) for entry in run_entries]),
            "rejected_count": aggregate_series([float(entry["gating"][method]["rejected_count"]) for entry in run_entries]),
        }
        aggregate["classification"][method] = {
            "true_positive_rate": aggregate_series([float(entry["classification"][method]["true_positive_rate"]) for entry in run_entries]),
            "false_positive_rate": aggregate_series([float(entry["classification"][method]["false_positive_rate"]) for entry in run_entries]),
            "precision": aggregate_series([float(entry["classification"][method]["precision"]) for entry in run_entries]),
            "recall": aggregate_series([float(entry["classification"][method]["recall"]) for entry in run_entries]),
        }
        pose_nis_runs = [entry["nis"][method].get("pose", {}) for entry in run_entries if "pose" in entry["nis"][method]]
        aggregate["nis"][method] = {
            "pose_fraction_above_upper_bound": aggregate_series(
                [float(payload["fraction_above_upper_bound"]) for payload in pose_nis_runs]
            ),
            "pose_nis_mean": aggregate_series([float(payload["nis_mean"]) for payload in pose_nis_runs]),
        }

    return aggregate


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=int, default=20)
    parser.add_argument("--repeats", type=int, default=3)
    parser.add_argument("--base-seed", type=int, default=1337)
    args = parser.parse_args()

    scenarios = build_scenarios()
    RESULTS_ROOT.mkdir(parents=True, exist_ok=True)
    ros2_ws_wsl_path = to_wsl_path(REPO_ROOT / "ros2_ws")
    summary = {
        "duration_seconds": args.duration,
        "repeats": args.repeats,
        "base_seed": args.base_seed,
        "run_started_at_utc": datetime.now(timezone.utc).isoformat(),
        "scenarios": {},
    }

    for scenario_index, scenario in enumerate(scenarios):
        scenario_root = RESULTS_ROOT / str(scenario["name"])
        scenario_root.mkdir(parents=True, exist_ok=True)
        run_entries: list[dict[str, object]] = []

        for repeat_index in range(args.repeats):
            repeat_name = f"run_{repeat_index + 1:03d}"
            repeat_root = scenario_root / "repeats" / repeat_name
            repeat_root.mkdir(parents=True, exist_ok=True)
            fake_seed = args.base_seed + scenario_index * 1000 + repeat_index * 10
            pf_seed = fake_seed + 1

            results_dir_wsl = to_wsl_path(repeat_root)
            launch_args = {
                "odom_position_noise_std": "0.03",
                "odom_velocity_noise_std": "0.02",
                "imu_yaw_rate_noise_std": "0.01",
                "dropout_probability": "0.0",
                "use_odom_pose_update": "true",
                "pose_gating_enabled": "true" if scenario["gating_enabled"] else "false",
                "pose_gating_threshold": str(scenario["pose_gating_threshold"]),
                "pose_outlier_probability": str(scenario["pose_outlier_probability"]),
                "pose_outlier_position_std": str(scenario["pose_outlier_position_std"]),
                "pose_outlier_yaw_std": str(scenario["pose_outlier_yaw_std"]),
                "pose_outlier_start_seconds": str(scenario["pose_outlier_start_seconds"]),
                "benchmark_duration_seconds": f"{float(args.duration):.1f}",
                "fake_sensor_seed": str(fake_seed),
                "pf_random_seed": str(pf_seed),
                "results_dir": results_dir_wsl,
                "fake_sensor_stats_out": to_wsl_path(repeat_root / "fake_sensor_stats.json"),
                "corruption_log_out": to_wsl_path(repeat_root / "corruption_log.csv"),
                "ekf_stats_out": to_wsl_path(repeat_root / "ekf_stats.json"),
                "ukf_stats_out": to_wsl_path(repeat_root / "ukf_stats.json"),
                "pf_stats_out": to_wsl_path(repeat_root / "pf_stats.json"),
                "logger_stats_out": to_wsl_path(repeat_root / "logger_stats.json"),
            }
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
            corruption_records = load_corruption_log_csv(repeat_root / "corruption_log.csv")
            gating_summary = {
                method: summarize_gating(diagnostics_records, estimator=method, measurement_type="pose")
                for method in GATING_METHODS
            }
            classification_summary = {
                method: summarize_gate_classification(
                    diagnostics_records,
                    corruption_records,
                    estimator=method,
                    measurement_type="pose",
                )
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
                    "launch_args": launch_args,
                    "seeds": {
                        "fake_sensor_seed": fake_seed,
                        "pf_random_seed": pf_seed,
                    },
                },
                "metrics": metrics,
                "gating": gating_summary,
                "classification": classification_summary,
                "nis": {
                    method: {
                        measurement_type: {
                            key: value
                            for key, value in measurement_summary.items()
                            if key not in {"timestamps", "nis_values", "in_bounds"}
                        }
                        for measurement_type, measurement_summary in payload["measurements"].items()
                    }
                    for method, payload in nis_summary.items()
                },
                "stats": stats_summary,
            }
            with (repeat_root / "phase4b_summary.json").open("w", encoding="utf-8") as handle:
                json.dump(repeat_entry, handle, indent=2)
            run_entries.append(repeat_entry)

        summary["scenarios"][str(scenario["name"])] = {
            "config": scenario,
            "repeats": run_entries,
            "aggregate": aggregate_run_entries(run_entries),
        }

    with (RESULTS_ROOT / "summary.json").open("w", encoding="utf-8") as handle:
        json.dump(summary, handle, indent=2)
    print(f"Wrote Phase 4b summary to {RESULTS_ROOT / 'summary.json'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
