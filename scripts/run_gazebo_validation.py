#!/usr/bin/env python3
"""Run the headless Gazebo validation path and score EKF/UKF/PF against Gazebo ground truth."""
from __future__ import annotations

import argparse
import csv
import json
import shutil
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
RESULTS_ROOT = REPO_ROOT / "results"
GAZEBO_METRICS_ROOT = RESULTS_ROOT / "gazebo_metrics"
GAZEBO_RUN_ROOT = RESULTS_ROOT / "gazebo_validation"
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


def clean_ros_processes() -> None:
    cleanup_cmd = (
        "pkill -9 -f gazebo_validation.launch.py || true; "
        "pkill -9 -f gzserver || true; "
        "pkill -9 -f gzclient || true; "
        "pkill -9 -f gazebo_ground_truth_bridge_node || true; "
        "pkill -9 -f circle_command_publisher_node || true; "
        "pkill -9 -f trajectory_logger_node || true; "
        "pkill -9 -f '/aegis_ros/ekf_node' || true; "
        "pkill -9 -f '/aegis_ros/ukf_node' || true; "
        "pkill -9 -f '/aegis_ros/particle_filter_node' || true; "
        "sleep 4"
    )
    run_wsl_bash(cleanup_cmd, cwd=REPO_ROOT / "ros2_ws", check=False)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=int, default=45, help="Gazebo validation runtime in seconds")
    parser.add_argument("--model", default="burger", help="TurtleBot3 model to spawn")
    parser.add_argument("--max-attempts", type=int, default=3, help="Maximum launch attempts before failing")
    args = parser.parse_args()

    GAZEBO_RUN_ROOT.mkdir(parents=True, exist_ok=True)
    git_commit = get_git_commit()
    run_started_at = datetime.now(timezone.utc).isoformat()
    ros2_ws_wsl_path = to_wsl_path(REPO_ROOT / "ros2_ws")

    gt_path = GAZEBO_METRICS_ROOT / "ground_truth.csv"
    last_error: Exception | None = None
    for attempt in range(1, args.max_attempts + 1):
        print(f"Starting Gazebo validation attempt {attempt}/{args.max_attempts}")
        clean_ros_processes()
        if GAZEBO_METRICS_ROOT.exists():
            shutil.rmtree(GAZEBO_METRICS_ROOT)
        GAZEBO_METRICS_ROOT.mkdir(parents=True, exist_ok=True)

        launch_cmd = (
            "source /opt/ros/humble/setup.bash && "
            f"cd {ros2_ws_wsl_path} && "
            "source install/setup.bash && "
            f"export TURTLEBOT3_MODEL={args.model} && "
            "export GAZEBO_MASTER_URI=http://127.0.0.1:11346 && "
            f"timeout {args.duration + 12}s ros2 launch aegis_ros gazebo_validation.launch.py "
            f"gui:=false model:={args.model}"
        )
        run_wsl_bash(launch_cmd, check=False)
        clean_ros_processes()

        try:
          require_samples(gt_path)
          last_error = None
          break
        except Exception as exc:
          last_error = exc
          print(f"Attempt {attempt} did not produce usable ground truth: {exc}")
    if last_error is not None:
        raise last_error

    summary = {
        "duration_seconds": args.duration,
        "model": args.model,
        "max_attempts": args.max_attempts,
        "git_commit": git_commit,
        "run_started_at_utc": run_started_at,
        "metrics": {},
    }

    shutil.copy2(gt_path, GAZEBO_RUN_ROOT / "ground_truth.csv")

    for method in METHODS:
        est_path = GAZEBO_METRICS_ROOT / f"{method}.csv"
        require_samples(est_path)
        out_json = GAZEBO_RUN_ROOT / f"{method}_metrics.json"
        shutil.copy2(est_path, GAZEBO_RUN_ROOT / f"{method}.csv")
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
        summary["metrics"][method] = metrics

    with (GAZEBO_RUN_ROOT / "summary.json").open("w", encoding="utf-8") as handle:
        json.dump(summary, handle, indent=2)

    print(f"Wrote Gazebo validation summary to {GAZEBO_RUN_ROOT / 'summary.json'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
