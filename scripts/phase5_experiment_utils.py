"""Small shared helpers for the bounded Phase 5 experiments."""

from __future__ import annotations

import json
import shlex
import shutil
import subprocess
import sys
from pathlib import Path

import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[1]
PYTHON_ROOT = REPO_ROOT / "python"
if str(PYTHON_ROOT) not in sys.path:
    sys.path.insert(0, str(PYTHON_ROOT))

from aegis_eval import (
    align_by_timestamp,
    compute_metrics,
    load_filter_diagnostics_csv,
    load_trajectory_csv,
    summarize_gating,
    summarize_nis,
)


METHODS = ("ekf", "ukf", "pf")


def to_wsl_path(path: Path) -> str:
    resolved = path.resolve()
    drive = resolved.drive.rstrip(":").lower()
    parts = [part for part in resolved.parts[1:] if part not in ("\\", "/")]
    return "/mnt/" + drive + "/" + "/".join(parts)


def run_wsl(command: str, *, quiet: bool = False) -> subprocess.CompletedProcess[str]:
    stdout = subprocess.DEVNULL if quiet else None
    return subprocess.run(
        ["wsl.exe", "-d", "Ubuntu-22.04", "-e", "bash", "-lc", command],
        cwd=REPO_ROOT,
        check=False,
        text=True,
        stdout=stdout,
    )


def clean_ros_nodes() -> None:
    command = (
        "pkill -f '[r]os2 launch aegis_ros fake_benchmark.launch.py' || true; "
        "pkill -f '/aegis_ros/[e]kf_node' || true; "
        "pkill -f '/aegis_ros/[u]kf_node' || true; "
        "pkill -f '/aegis_ros/[p]article_filter_node' || true; "
        "pkill -f '/aegis_ros/[t]rajectory_logger_node' || true; "
        "pkill -f '/aegis_ros/[f]ake_sensor_publisher_node' || true"
    )
    run_wsl(command, quiet=True)


def run_scenario(root: Path, launch_args: dict[str, str], duration: int) -> dict[str, object]:
    if root.exists():
        shutil.rmtree(root)
    root.mkdir(parents=True, exist_ok=True)

    args = dict(launch_args)
    args["benchmark_duration_seconds"] = f"{float(duration):.1f}"
    args["results_dir"] = to_wsl_path(root)
    args["fake_sensor_stats_out"] = to_wsl_path(root / "fake_sensor_stats.json")
    args["correction_log_out"] = to_wsl_path(root / "correction_log.csv")
    args["ekf_stats_out"] = to_wsl_path(root / "ekf_stats.json")
    args["ukf_stats_out"] = to_wsl_path(root / "ukf_stats.json")
    args["pf_stats_out"] = to_wsl_path(root / "pf_stats.json")
    args["logger_stats_out"] = to_wsl_path(root / "logger_stats.json")

    ros2_ws_wsl = to_wsl_path(REPO_ROOT / "ros2_ws")
    launch_items = [shlex.quote(f"{key}:={value}") for key, value in args.items()]
    cleanup = (
        "pkill -f '/aegis_ros/[e]kf_node' || true; "
        "pkill -f '/aegis_ros/[u]kf_node' || true; "
        "pkill -f '/aegis_ros/[p]article_filter_node' || true; "
        "pkill -f '/aegis_ros/[t]rajectory_logger_node' || true; "
        "pkill -f '/aegis_ros/[f]ake_sensor_publisher_node' || true"
    )
    command = (
        "source /opt/ros/humble/setup.bash && "
        f"cd {ros2_ws_wsl} && "
        "source install/setup.bash && "
        f"trap \"{cleanup}\" EXIT INT TERM; "
        f"timeout {duration + 10}s ros2 launch aegis_ros fake_benchmark.launch.py {' '.join(launch_items)}"
    )
    clean_ros_nodes()
    completed = run_wsl(command, quiet=True)
    clean_ros_nodes()
    if completed.returncode not in (0, 124):
        raise RuntimeError(f"ROS launch failed for {root.name}: {completed.returncode}")

    metrics = {}
    for method in METHODS:
        est_path = root / f"{method}.csv"
        gt_path = root / "ground_truth.csv"
        merged = align_by_timestamp(load_trajectory_csv(est_path), load_trajectory_csv(gt_path))
        metrics[method] = compute_metrics(merged)
        (root / f"{method}_metrics.json").write_text(
            json.dumps(metrics[method], indent=2) + "\n", encoding="utf-8"
        )

    stats = {}
    for name in ("fake_sensor", "ekf", "ukf", "pf", "logger"):
        path = root / f"{name}_stats.json"
        if path.exists() and path.stat().st_size:
            stats[name] = json.loads(path.read_text(encoding="utf-8"))
    return {"launch_args": args, "metrics": metrics, "stats": stats}


def aggregate_scalar_records(records: list[dict[str, float]]) -> dict[str, dict[str, float]]:
    if not records:
        return {}
    output: dict[str, dict[str, float]] = {}
    for field in records[0]:
        values = np.array([record[field] for record in records], dtype=float)
        output[field] = {
            "mean": float(np.mean(values)),
            "std": float(np.std(values)),
            "min": float(np.min(values)),
            "max": float(np.max(values)),
        }
    return output


def diagnostic_scalars(root: Path) -> dict[str, dict[str, float]]:
    path = root / "filter_diagnostics.csv"
    if not path.exists():
        return {}
    records = load_filter_diagnostics_csv(path)
    output: dict[str, dict[str, float]] = {}
    for method in ("ekf", "ukf"):
        measurements = summarize_nis(records, estimator=method).get("measurements", {})
        pose_nis = measurements.get("pose", {})
        gate = summarize_gating(records, estimator=method, measurement_type="pose")
        output[method] = {
            "pose_nis_in_bounds_fraction": float(pose_nis.get("fraction_in_bounds", float("nan"))),
            "pose_nis_above_upper_fraction": float(pose_nis.get("fraction_above_upper_bound", float("nan"))),
            "pose_gating_rejection_rate": float(gate.get("rejection_rate", float("nan"))),
            "pose_measurement_count": float(gate.get("num_measurements", 0)),
        }
    return output


def dropout_recovery_metrics(est_path: Path, gt_path: Path, blackout_start: float, blackout_duration: float) -> dict[str, float]:
    est = load_trajectory_csv(est_path)
    gt = load_trajectory_csv(gt_path)
    merged = align_by_timestamp(est, gt)
    elapsed = merged["timestamp"] - merged["timestamp"][0]
    errors = np.hypot(merged["x_est"] - merged["x_gt"], merged["y_est"] - merged["y_gt"])
    during = (elapsed >= blackout_start) & (elapsed <= blackout_start + blackout_duration)
    post_target = blackout_start + blackout_duration + 1.0
    post_index = int(np.argmin(np.abs(elapsed - post_target)))
    pre_index = int(np.argmin(np.abs(elapsed - blackout_start)))
    return {
        "error_at_blackout_start": float(errors[pre_index]),
        "peak_error_during_blackout": float(np.max(errors[during])) if np.any(during) else float("nan"),
        "error_one_second_after_recovery": float(errors[post_index]),
        "recovery_from_peak": float(np.max(errors[during]) - errors[post_index]) if np.any(during) else float("nan"),
    }
