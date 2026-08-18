#!/usr/bin/env python3
"""Run one EuRoC sequence through the current Aegis ROS estimators."""

from __future__ import annotations

import argparse
import contextlib
import csv
from datetime import datetime, timezone
import io
import json
import subprocess
import sys
import time
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
PYTHON_ROOT = REPO_ROOT / "python"
if str(PYTHON_ROOT) not in sys.path:
    sys.path.insert(0, str(PYTHON_ROOT))

from aegis_bench import create_run_layout
from aegis_bench.euroc import write_planar_proxy_csv
from aegis_eval import align_by_timestamp, compute_metrics, load_trajectory_csv, write_json


METHODS = ("ekf", "ukf", "pf")


def run(cmd: list[str], cwd: Path | None = None, check: bool = True) -> subprocess.CompletedProcess[str]:
    return subprocess.run(cmd, cwd=cwd, check=check, text=True, capture_output=False)


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


def clean_ros_nodes() -> None:
    cleanup_cmd = (
        "pkill -9 -f dataset_benchmark.launch.py || true; "
        "pkill -9 -f replay_euroc_sequence.py || true; "
        "pkill -9 -f '/aegis_ros/ekf_node' || true; "
        "pkill -9 -f '/aegis_ros/ukf_node' || true; "
        "pkill -9 -f '/aegis_ros/particle_filter_node' || true; "
        "pkill -9 -f '/aegis_ros/trajectory_logger_node' || true; "
        "sleep 2"
    )
    run_wsl_bash(cleanup_cmd, cwd=REPO_ROOT / "ros2_ws", check=False)


def require_samples(csv_path: Path, minimum_rows: int = 2) -> None:
    if not csv_path.exists():
        raise FileNotFoundError(csv_path)

    with csv_path.open("r", encoding="utf-8", newline="") as handle:
        row_count = sum(1 for _ in csv.reader(handle))

    if row_count < minimum_rows + 1:
        raise RuntimeError(
            f"CSV did not contain enough trajectory samples: {csv_path} "
            f"(found {max(row_count - 1, 0)}, need at least {minimum_rows})"
        )


def count_csv_samples(csv_path: Path) -> int:
    with csv_path.open("r", encoding="utf-8", newline="") as handle:
        return max(sum(1 for _ in csv.reader(handle)) - 1, 0)


def build_comparison(summary_metrics: dict[str, dict[str, float | int]]) -> dict[str, object]:
    baseline_root = REPO_ROOT / "results" / "campaign" / "low_noise"
    comparison: dict[str, object] = {
        "baseline_name": "synthetic_low_noise",
        "baseline_path": str(baseline_root),
        "estimators": {},
    }
    for method in METHODS:
        baseline_path = baseline_root / f"{method}_metrics.json"
        if not baseline_path.exists():
            continue
        baseline_metrics = json.loads(baseline_path.read_text(encoding="utf-8"))
        euroc_metrics = summary_metrics[method]
        comparison["estimators"][method] = {
            "synthetic_low_noise": baseline_metrics,
            "euroc_proxy_planar": euroc_metrics,
            "delta_ate_rmse": float(euroc_metrics["ate_rmse"]) - float(baseline_metrics["ate_rmse"]),
            "delta_final_drift": float(euroc_metrics["final_drift"]) - float(baseline_metrics["final_drift"]),
            "delta_yaw_rmse": float(euroc_metrics["yaw_rmse"]) - float(baseline_metrics["yaw_rmse"]),
        }
    return comparison


def read_optional_json(path: Path) -> dict[str, object] | None:
    if not path.exists():
        return None
    return json.loads(path.read_text(encoding="utf-8"))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--sequence-root", required=True, help="Path to one EuRoC sequence root containing mav0/")
    parser.add_argument("--sequence-name", default=None, help="Optional stable name for the run directory")
    parser.add_argument("--run-name", default="proxy_planar", help="Run name within the canonical result layout")
    parser.add_argument("--replay-mode", choices=("faithful", "accelerated", "offline"), default="faithful")
    parser.add_argument("--speedup", type=float, default=1.0, help="Replay speedup factor when replay-mode=accelerated.")
    parser.add_argument("--sleep-scale", type=float, default=None, help="Deprecated compatibility flag for replay speedup.")
    parser.add_argument("--prepare-only", action="store_true", help="Only build the proxy input and metadata without launching ROS.")
    parser.add_argument("--max-samples", type=int, default=None, help="Optional limit for replayed samples during debugging or smoke tests.")
    args = parser.parse_args()

    if args.sleep_scale is not None:
      sleep_scale = args.sleep_scale
      replay_mode = "accelerated" if args.sleep_scale > 0.0 and abs(args.sleep_scale - 1.0) > 1e-9 else "faithful"
    elif args.replay_mode == "faithful":
      sleep_scale = 1.0
      replay_mode = "faithful"
    elif args.replay_mode == "offline":
      sleep_scale = 0.0
      replay_mode = "offline"
    else:
      sleep_scale = args.speedup
      replay_mode = "accelerated"

    sequence_root = Path(args.sequence_root).resolve()
    sequence_name = args.sequence_name or sequence_root.name
    run_layout = create_run_layout(REPO_ROOT / "results", "euroc", sequence_name, args.run_name)
    proxy_csv = run_layout.logs_dir / "proxy_planar.csv"
    write_planar_proxy_csv(sequence_root, proxy_csv)

    manifest = {
        "backend": "euroc",
        "sequence_name": sequence_name,
        "run_name": args.run_name,
        "sequence_root": str(sequence_root),
        "proxy_input_csv": str(proxy_csv),
        "reduction_assumptions": [
            "EuRoC is reduced to planar x, y, yaw motion only.",
            "Ground truth is taken from state_groundtruth_estimate0/data.csv.",
            "Planar linear velocities are derived from consecutive ground-truth positions.",
            "Aligned IMU yaw rate is copied into /odom.twist.angular.z because current Aegis ROS wrappers read angular rate from odometry twist rather than /imu directly.",
            "use_odom_pose_update is disabled so ground-truth pose is used only for initialization and evaluation, not for continuous estimator correction.",
            "This backend is a real-data motion and timing benchmark, not a wheel-encoder-faithful EuRoC localization benchmark.",
        ],
    }
    write_json(run_layout.manifest_path, manifest)

    metadata = {
        "backend": "euroc",
        "sequence_name": sequence_name,
        "run_name": args.run_name,
        "git_commit": get_git_commit(),
        "run_started_at_utc": datetime.now(timezone.utc).isoformat(),
        "truth_source": "dataset",
        "proxy_input_csv": str(proxy_csv),
        "replay_mode": replay_mode,
        "sleep_scale": sleep_scale,
        "max_samples": args.max_samples,
        "expected_subscribers": {"odom": 3, "imu": 3, "truth": 1},
        "result_layout": run_layout.to_dict(),
    }
    write_json(run_layout.metadata_path, metadata)

    if args.prepare_only:
        print(f"Prepared EuRoC proxy inputs at {run_layout.root}")
        return 0

    ros2_ws_wsl_path = to_wsl_path(REPO_ROOT / "ros2_ws")
    proxy_csv_wsl = to_wsl_path(proxy_csv)
    results_dir_wsl = to_wsl_path(run_layout.normalized_dir)
    replay_summary_wsl = to_wsl_path(run_layout.logs_dir / "replay_summary.json")
    launch_log_path = run_layout.logs_dir / "launch.log"
    ekf_stats_wsl = to_wsl_path(run_layout.logs_dir / "ekf_stats.json")
    ukf_stats_wsl = to_wsl_path(run_layout.logs_dir / "ukf_stats.json")
    pf_stats_wsl = to_wsl_path(run_layout.logs_dir / "pf_stats.json")
    logger_stats_wsl = to_wsl_path(run_layout.logs_dir / "logger_stats.json")

    clean_ros_nodes()
    launch_cmd = [
        "wsl.exe",
        "-d",
        "Ubuntu-22.04",
        "-e",
        "bash",
        "-lc",
        (
            "source /opt/ros/humble/setup.bash && "
            f"cd {ros2_ws_wsl_path} && "
            "source install/setup.bash && "
            "ros2 launch aegis_ros dataset_benchmark.launch.py "
            "run_ekf:=true run_ukf:=true run_pf:=true "
            "use_odom_pose_update:=false "
            "log_odom_baseline:=true "
            f"results_dir:={results_dir_wsl} "
            f"ekf_stats_out:={ekf_stats_wsl} "
            f"ukf_stats_out:={ukf_stats_wsl} "
            f"pf_stats_out:={pf_stats_wsl} "
            f"logger_stats_out:={logger_stats_wsl}"
        ),
    ]

    replay_command = (
        "source /opt/ros/humble/setup.bash && "
        f"cd {ros2_ws_wsl_path} && "
        "source install/setup.bash && "
        "ros2 run aegis_ros euroc_replay_node --ros-args "
        f"-p proxy_csv_path:={proxy_csv_wsl} "
        f"-p sleep_scale:={sleep_scale} "
        "-p expected_odom_subscribers:=3 "
        "-p expected_imu_subscribers:=3 "
        "-p expected_truth_subscribers:=1 "
        "-p require_full_subscribers:=true "
        f"-p summary_out:={replay_summary_wsl} "
        f"{('-p max_samples:=' + str(args.max_samples)) if args.max_samples is not None else ''}"
    )

    launch_log_path.parent.mkdir(parents=True, exist_ok=True)
    launch_log_handle = launch_log_path.open("w", encoding="utf-8")
    launch_process = subprocess.Popen(
        launch_cmd,
        cwd=REPO_ROOT / "ros2_ws",
        stdout=launch_log_handle,
        stderr=subprocess.STDOUT,
        text=True,
    )

    try:
        time.sleep(5.0)
        run_wsl_bash(replay_command, cwd=REPO_ROOT / "ros2_ws", check=True)
        time.sleep(5.0)
    finally:
        if launch_process.poll() is None:
            launch_process.terminate()
            try:
                launch_process.wait(timeout=10.0)
            except subprocess.TimeoutExpired:
                launch_process.kill()
                launch_process.wait(timeout=10.0)
        launch_log_handle.close()
        clean_ros_nodes()

    require_samples(run_layout.ground_truth_csv)
    require_samples(run_layout.normalized_dir / "odom.csv")

    summary_metrics: dict[str, dict[str, float | int]] = {}
    plotting_available = True
    plot_error_message = None
    try:
        with contextlib.redirect_stderr(io.StringIO()):
            from aegis_eval.plots import plot_position_error, plot_trajectory_overlay
    except Exception as exc:  # pragma: no cover - depends on local plotting stack
        plotting_available = False
        plot_error_message = str(exc)

    for method in METHODS:
        est_path = run_layout.estimator_csv(method)
        require_samples(est_path)
        metrics = compute_metrics(align_by_timestamp(load_trajectory_csv(est_path), load_trajectory_csv(run_layout.ground_truth_csv)))
        summary_metrics[method] = metrics
        write_json(run_layout.metrics_dir / f"{method}_metrics.json", metrics)
        if plotting_available:
            plot_trajectory_overlay(est_path, run_layout.ground_truth_csv, run_layout.plots_dir / f"{method}_trajectory.png")
            plot_position_error(est_path, run_layout.ground_truth_csv, run_layout.plots_dir / f"{method}_position_error.png")

    comparison = build_comparison(summary_metrics)
    odom_metrics = compute_metrics(
        align_by_timestamp(
            load_trajectory_csv(run_layout.normalized_dir / "odom.csv"),
            load_trajectory_csv(run_layout.ground_truth_csv),
        )
    )
    write_json(run_layout.metrics_dir / "odom_metrics.json", odom_metrics)

    accounting = {
        "replay_summary": json.loads((run_layout.logs_dir / "replay_summary.json").read_text(encoding="utf-8")),
        "ekf_stats": read_optional_json(run_layout.logs_dir / "ekf_stats.json"),
        "ukf_stats": read_optional_json(run_layout.logs_dir / "ukf_stats.json"),
        "pf_stats": read_optional_json(run_layout.logs_dir / "pf_stats.json"),
        "logger_stats": read_optional_json(run_layout.logs_dir / "logger_stats.json"),
        "csv_row_counts": {
            "ground_truth": count_csv_samples(run_layout.ground_truth_csv),
            "odom": count_csv_samples(run_layout.normalized_dir / "odom.csv"),
            "ekf": count_csv_samples(run_layout.estimator_csv("ekf")),
            "ukf": count_csv_samples(run_layout.estimator_csv("ukf")),
            "pf": count_csv_samples(run_layout.estimator_csv("pf")),
        },
    }
    write_json(run_layout.metrics_dir / "coverage_accounting.json", accounting)
    write_json(run_layout.metrics_dir / "comparison_to_synthetic_low_noise.json", comparison)
    write_json(
        run_layout.metrics_dir / "summary.json",
        {
            "metrics": summary_metrics,
            "odom_baseline": odom_metrics,
            "comparison": comparison,
            "coverage_accounting": accounting,
            "plots_generated": plotting_available,
            "plot_error": plot_error_message,
            "replay_mode": replay_mode,
            "sleep_scale": sleep_scale,
        },
    )
    print(f"Wrote EuRoC benchmark outputs to {run_layout.root}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
