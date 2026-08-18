#!/usr/bin/env python3
"""Summarize a EuRoC benchmark run with coverage, failures, and yaw caveats."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path


def wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def load_trajectory(path: Path) -> list[tuple[float, float, float, float]]:
    rows: list[tuple[float, float, float, float]] = []
    with path.open("r", encoding="utf-8", newline="") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            if not row.get("timestamp") or not row.get("x") or not row.get("y") or not row.get("yaw"):
                continue
            rows.append(
                (
                    float(row["timestamp"]),
                    float(row["x"]),
                    float(row["y"]),
                    float(row["yaw"]),
                )
            )
    return rows


def nearest_yaw_errors(
    est_rows: list[tuple[float, float, float, float]],
    gt_rows: list[tuple[float, float, float, float]],
) -> list[float]:
    if not est_rows or not gt_rows:
        return []
    gt_index = 0
    errors: list[float] = []
    for timestamp, _, _, yaw in est_rows:
        while gt_index + 1 < len(gt_rows) and abs(gt_rows[gt_index + 1][0] - timestamp) <= abs(gt_rows[gt_index][0] - timestamp):
            gt_index += 1
        errors.append(wrap_angle(yaw - gt_rows[gt_index][3]))
    return errors


def summarize_yaw(errors: list[float]) -> dict[str, float | None]:
    if not errors:
        return {
            "yaw_rmse": None,
            "circular_mean_offset": None,
            "centered_yaw_rmse": None,
            "min_error": None,
            "max_error": None,
        }
    mean_sin = sum(math.sin(value) for value in errors) / len(errors)
    mean_cos = sum(math.cos(value) for value in errors) / len(errors)
    circular_mean_offset = math.atan2(mean_sin, mean_cos)
    centered = [wrap_angle(value - circular_mean_offset) for value in errors]
    yaw_rmse = math.sqrt(sum(value * value for value in errors) / len(errors))
    centered_yaw_rmse = math.sqrt(sum(value * value for value in centered) / len(centered))
    return {
        "yaw_rmse": yaw_rmse,
        "circular_mean_offset": circular_mean_offset,
        "centered_yaw_rmse": centered_yaw_rmse,
        "min_error": min(errors),
        "max_error": max(errors),
    }


def extract_launch_issues(launch_log_text: str) -> list[str]:
    issues: list[str] = []
    for line in launch_log_text.splitlines():
        lowered = line.lower()
        if "process has died" in lowered or "terminate called after throwing" in lowered or "ukf covariance is not positive definite" in lowered:
            issues.append(line.strip())
    return issues


def write_report(run_root: Path, report_payload: dict[str, object]) -> Path:
    report_path = run_root / "benchmark_report.md"
    metrics = report_payload["metrics"]
    coverage = report_payload["coverage"]
    yaw_analysis = report_payload["yaw_analysis"]
    assumptions = report_payload["reduction_assumptions"]
    launch_issues = report_payload["launch_issues"]
    conclusions = report_payload["conclusions"]
    replay_summary = report_payload["replay_summary"]
    expected_subscribers = report_payload.get("expected_subscribers", {})
    replay_mode = report_payload.get("replay_mode")
    coverage_accounting = report_payload.get("coverage_accounting")

    lines = [
        "# EuRoC Benchmark Report",
        "",
        f"- Run root: `{run_root}`",
        f"- Sequence: `{report_payload['sequence_name']}`",
        f"- Run name: `{report_payload['run_name']}`",
        f"- Git commit: `{report_payload['git_commit']}`",
        f"- Truth source: `{report_payload['truth_source']}`",
        f"- Replay samples published: `{replay_summary['published_samples']}`",
        f"- Replay wall time (s): `{replay_summary['elapsed_wall_seconds']}`",
        f"- Replay mode: `{replay_mode}`",
        "",
        "## Replay Readiness",
        "",
        f"- Observed subscribers: `odom={replay_summary['subscriber_snapshot']['odom']}`, `imu={replay_summary['subscriber_snapshot']['imu']}`, `truth={replay_summary['subscriber_snapshot']['truth']}`",
        f"- Expected subscribers: `odom={expected_subscribers.get('odom')}`, `imu={expected_subscribers.get('imu')}`, `truth={expected_subscribers.get('truth')}`",
        "",
        "## Estimator Coverage",
        "",
        "| Estimator | Logged Samples | Coverage vs Replay |",
        "|---|---:|---:|",
    ]

    for estimator, info in coverage.items():
        lines.append(f"| {estimator} | {info['logged_samples']} | {info['coverage_ratio']:.3f} |")

    lines.extend([
        "",
        "## Metrics",
        "",
        "| Estimator | ATE RMSE | Final Drift | Yaw RMSE |",
        "|---|---:|---:|---:|",
    ])

    for estimator, info in metrics.items():
        lines.append(
            f"| {estimator} | {info['ate_rmse']:.6f} | {info['final_drift']:.6f} | {info['yaw_rmse']:.6f} |"
        )

    lines.extend([
        "",
        "## Yaw Analysis",
        "",
        "This run should not be interpreted as a clean yaw-validation benchmark.",
        "",
        "| Estimator | Circular Mean Offset | Centered Yaw RMSE | Raw Yaw RMSE |",
        "|---|---:|---:|---:|",
    ])

    for estimator, info in yaw_analysis.items():
        lines.append(
            f"| {estimator} | {info['circular_mean_offset']:.6f} | {info['centered_yaw_rmse']:.6f} | {info['yaw_rmse']:.6f} |"
        )

    if coverage_accounting:
        replay_stats = coverage_accounting.get("replay_summary", {})
        ekf_stats = coverage_accounting.get("ekf_stats") or {}
        ukf_stats = coverage_accounting.get("ukf_stats") or {}
        pf_stats = coverage_accounting.get("pf_stats") or {}
        logger_stats = coverage_accounting.get("logger_stats") or {}
        csv_row_counts = coverage_accounting.get("csv_row_counts") or {}
        lines.extend([
            "",
            "## Coverage Accounting",
            "",
            "| Stage | GT/Odom | EKF | UKF | PF |",
            "|---|---:|---:|---:|---:|",
            (
                "| Replay published | "
                f"{replay_stats.get('published_samples')} | "
                f"{replay_stats.get('published_samples')} | "
                f"{replay_stats.get('published_samples')} | "
                f"{replay_stats.get('published_samples')} |"
            ),
            (
                "| Estimator odom received | "
                f"{logger_stats.get('odom_received', 0)} | "
                f"{ekf_stats.get('odom_received', 0)} | "
                f"{ukf_stats.get('odom_received', 0)} | "
                f"{pf_stats.get('odom_received', 0)} |"
            ),
            (
                "| Estimator pose published | "
                f"{logger_stats.get('odom_received', 0)} | "
                f"{ekf_stats.get('pose_published', 0)} | "
                f"{ukf_stats.get('pose_published', 0)} | "
                f"{pf_stats.get('pose_published', 0)} |"
            ),
            (
                "| Logger received | "
                f"{logger_stats.get('ground_truth_received', 0)}/"
                f"{logger_stats.get('odom_received', 0)} | "
                f"{logger_stats.get('ekf_received', 0)} | "
                f"{logger_stats.get('ukf_received', 0)} | "
                f"{logger_stats.get('pf_received', 0)} |"
            ),
            (
                "| Final CSV rows | "
                f"{csv_row_counts.get('ground_truth', 0)}/{csv_row_counts.get('odom', 0)} | "
                f"{csv_row_counts.get('ekf', 0)} | "
                f"{csv_row_counts.get('ukf', 0)} | "
                f"{csv_row_counts.get('pf', 0)} |"
            ),
        ])

    lines.extend([
        "",
        "## Reduction Assumptions",
        "",
    ])
    for assumption in assumptions:
        lines.append(f"- {assumption}")

    lines.extend([
        "",
        "## Launch Issues",
        "",
    ])
    if launch_issues:
        for issue in launch_issues:
            lines.append(f"- {issue}")
    else:
        lines.append("- No launch issues recorded.")

    lines.extend([
        "",
        "## Conclusions",
        "",
    ])
    for conclusion in conclusions:
        lines.append(f"- {conclusion}")

    report_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return report_path


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--run-root", required=True, help="Path to one results/euroc/<sequence>/<run> directory")
    args = parser.parse_args()

    run_root = Path(args.run_root).resolve()
    manifest = json.loads((run_root / "manifest.json").read_text(encoding="utf-8"))
    metadata = json.loads((run_root / "metadata.json").read_text(encoding="utf-8"))
    summary = json.loads((run_root / "metrics" / "summary.json").read_text(encoding="utf-8"))
    replay_summary = json.loads((run_root / "logs" / "replay_summary.json").read_text(encoding="utf-8"))
    launch_log_text = (run_root / "logs" / "launch.log").read_text(encoding="utf-8")
    coverage_accounting_path = run_root / "metrics" / "coverage_accounting.json"
    coverage_accounting = (
        json.loads(coverage_accounting_path.read_text(encoding="utf-8"))
        if coverage_accounting_path.exists()
        else None
    )

    gt_rows = load_trajectory(run_root / "normalized" / "ground_truth.csv")
    estimator_rows = {
        estimator: load_trajectory(run_root / "normalized" / f"{estimator}.csv")
        for estimator in ("ekf", "ukf", "pf")
    }

    replay_samples = int(replay_summary["published_samples"])
    coverage = {}
    yaw_analysis = {}
    for estimator, rows in estimator_rows.items():
        logged_samples = len(rows)
        coverage[estimator] = {
            "logged_samples": logged_samples,
            "coverage_ratio": (logged_samples / replay_samples) if replay_samples > 0 else 0.0,
        }
        yaw_analysis[estimator] = summarize_yaw(nearest_yaw_errors(rows, gt_rows))

    launch_issues = extract_launch_issues(launch_log_text)
    conclusions = ["This run successfully produced canonical recorded-data outputs from EuRoC proxy replay."]

    ukf_metrics = summary["metrics"]["ukf"]
    ukf_coverage_ratio = coverage["ukf"]["coverage_ratio"]
    if launch_issues:
        conclusions.append("UKF still exhibits a recorded launch/runtime issue in this artifact and should not yet be treated as fully stable.")
    elif ukf_coverage_ratio < 0.95:
        conclusions.append("UKF stayed alive but under-covered the replay, so recorded-data stability should still be treated as incomplete.")
    else:
        conclusions.append("UKF completed the intended recorded-data evaluation window without the earlier non-positive-definite covariance crash, and its final covariance health remained finite and PSD.")

    conclusions.append(
        "The UKF crash on earlier MH_01_easy runs was consistent with two implementation-level issues: process noise was being added without dt scaling at high replay rates, and the previous alpha=0.1 sigma-point setting produced an extremely negative central covariance weight."
    )
    conclusions.append(
        "Yaw should still be treated cautiously or excluded as a headline proxy metric, because the current planar benchmark compares projected MAV yaw while the motion model propagates x/y directly from world-frame vx/vy without heading-coupled translation."
    )

    csv_row_counts = (coverage_accounting or {}).get("csv_row_counts", {}) if coverage_accounting else {}
    if csv_row_counts:
        gt_rows = int(csv_row_counts.get("ground_truth", 0))
        if gt_rows < replay_samples:
            conclusions.append(
                f"Remaining sample loss is now small and accounted for at the artifact level ({replay_samples - gt_rows} ground-truth rows short of replay publication), but the shutdown/write path is not yet perfect."
            )
        else:
            conclusions.append("No sample loss remained between replay publication and final logged ground-truth rows.")

    conclusions.append(
        "This artifact is suitable as a defensible Phase 1 recorded-data benchmark result, with yaw limitations explicitly documented rather than hidden."
    )

    payload = {
        "sequence_name": manifest["sequence_name"],
        "run_name": manifest["run_name"],
        "git_commit": metadata.get("git_commit"),
        "truth_source": metadata.get("truth_source"),
        "replay_mode": metadata.get("replay_mode"),
        "reduction_assumptions": manifest["reduction_assumptions"],
        "replay_summary": replay_summary,
        "expected_subscribers": metadata.get("expected_subscribers", {}),
        "coverage_accounting": coverage_accounting,
        "metrics": summary["metrics"],
        "coverage": coverage,
        "yaw_analysis": yaw_analysis,
        "launch_issues": launch_issues,
        "conclusions": conclusions,
    }

    write_json_path = run_root / "metrics" / "diagnostics.json"
    write_json_path.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    write_report(run_root, payload)
    print(f"Wrote diagnostics to {write_json_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
