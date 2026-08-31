#!/usr/bin/env python3
"""Generate a compact Markdown report for Phase 5."""
from __future__ import annotations

import json
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
SUMMARY_PATH = REPO_ROOT / "results" / "phase5_intermittent_correction" / "summary.json"
REPORT_PATH = REPO_ROOT / "results" / "reports" / "phase5_intermittent_correction.md"
METHODS = ["ekf", "ukf", "pf"]


def fmt(value: float | int | None) -> str:
    if value is None:
        return "n/a"
    if isinstance(value, int):
        return str(value)
    return f"{value:.4f}"


def main() -> int:
    summary = json.loads(SUMMARY_PATH.read_text(encoding="utf-8"))
    scenarios = summary["scenarios"]
    baseline = scenarios["dead_reckoning_baseline"]
    stress = scenarios["intermittent_correction_stress"]

    lines = [
        "# Phase 5 Intermittent Correction",
        "",
        f"- Run date: {summary['run_started_at_utc']}",
        f"- Duration: {summary['duration_seconds']} s",
        "",
        "## Metrics",
        "",
        "| Estimator | Baseline ATE | Stress ATE | Baseline Drift | Stress Drift | Baseline Yaw RMSE | Stress Yaw RMSE |",
        "| --- | ---: | ---: | ---: | ---: | ---: | ---: |",
    ]

    for method in METHODS:
        base_metrics = baseline["metrics"][method]
        stress_metrics = stress["metrics"][method]
        lines.append(
            f"| {method.upper()} | "
            f"{fmt(base_metrics['ate_rmse'])} | {fmt(stress_metrics['ate_rmse'])} | "
            f"{fmt(base_metrics['final_drift'])} | {fmt(stress_metrics['final_drift'])} | "
            f"{fmt(base_metrics['yaw_rmse'])} | {fmt(stress_metrics['yaw_rmse'])} |"
        )

    lines.extend([
        "",
        "## Correction Stream",
        "",
        f"- Frequency: {stress['metadata']['launch_args']['correction_frequency_hz']} Hz",
        f"- Dropout probability: {stress['metadata']['launch_args']['correction_dropout_probability']}",
        f"- Latency: {stress['metadata']['launch_args']['correction_latency_seconds']} s",
        f"- Outlier probability: {stress['metadata']['launch_args']['correction_outlier_probability']}",
        "",
        "## EKF/UKF Consistency",
        "",
    ])

    for method in ("ekf", "ukf"):
        pose = stress["consistency"].get(method, {}).get("pose", {})
        lines.append(
            f"- {method.upper()} pose NIS in-bounds: {fmt(pose.get('fraction_in_bounds'))}, "
            f"above upper bound: {fmt(pose.get('fraction_above_upper_bound'))}"
        )

    REPORT_PATH.parent.mkdir(parents=True, exist_ok=True)
    REPORT_PATH.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"Wrote Phase 5 report to {REPORT_PATH}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
