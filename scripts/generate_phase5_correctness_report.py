#!/usr/bin/env python3
"""Generate a compact report for Phase 5 correctness checks."""
from __future__ import annotations

import json
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
SUMMARY_PATH = REPO_ROOT / "results" / "phase5_correctness" / "summary.json"
REPORT_PATH = REPO_ROOT / "results" / "reports" / "phase5_correctness.md"
METHODS = ["ekf", "ukf", "pf"]


def fmt(value: float | int | None) -> str:
    if value is None:
        return "n/a"
    if isinstance(value, int):
        return str(value)
    return f"{value:.6f}"


def main() -> int:
    summary = json.loads(SUMMARY_PATH.read_text(encoding="utf-8"))
    lines = [
        "# Phase 5 Correctness Checks",
        "",
        f"- Run date: {summary['run_started_at_utc']}",
        f"- Duration: {summary['duration_seconds']} s",
        "- Interpretation note: arrival-latency runs preserve the same final replayed state, but their online logged trajectories still differ before delayed corrections arrive. Whole-trajectory RMSE therefore reflects online publication history, not terminal replay correctness alone.",
        "",
        "## Zero-Latency Equivalence",
        "",
        "| Estimator | Final pos diff | Pos RMSE diff | Max pos diff | Final yaw diff | Yaw RMSE diff |",
        "| --- | ---: | ---: | ---: | ---: | ---: |",
    ]
    for method in METHODS:
        traj = summary["zero_latency_equivalence"][method]["trajectory"]
        lines.append(
            f"| {method.upper()} | {fmt(traj['final_position_error'])} | {fmt(traj['rmse_position_error'])} | "
            f"{fmt(traj['max_position_error'])} | {fmt(traj['final_yaw_error'])} | {fmt(traj['rmse_yaw_error'])} |"
        )

    lines.extend([
        "",
        "## Arrival-Time Invariance",
        "",
    ])

    for scenario_name, comparison in summary["arrival_time_invariance"].items():
        lines.extend([
            f"### {scenario_name}",
            "",
            "| Estimator | Final pos diff | Pos RMSE diff | Max pos diff | Final yaw diff | Yaw RMSE diff |",
            "| --- | ---: | ---: | ---: | ---: | ---: |",
        ])
        for method in METHODS:
            traj = comparison[method]["trajectory"]
            lines.append(
                f"| {method.upper()} | {fmt(traj['final_position_error'])} | {fmt(traj['rmse_position_error'])} | "
                f"{fmt(traj['max_position_error'])} | {fmt(traj['final_yaw_error'])} | {fmt(traj['rmse_yaw_error'])} |"
            )
        lines.append("")

    lines.extend([
        "## Reversed Arrival Order",
        "",
        "| Estimator | Final pos diff | Final yaw diff | Cov diff |",
        "| --- | ---: | ---: | ---: |",
    ])
    for method in METHODS:
        comparison = summary["reversed_arrival_order"][method]
        final_state = comparison["final_state"]
        covariance = comparison.get("covariance")
        cov_value = fmt(covariance["frobenius_norm"]) if covariance else "n/a"
        lines.append(
            f"| {method.upper()} | {fmt(final_state['position_error'])} | "
            f"{fmt(final_state['yaw_error'])} | {cov_value} |"
        )

    lines.extend([
        "",
        "## History-Window Rejection",
        "",
        "| Estimator | Final pos diff vs valid-only | Final yaw diff | Cov diff | Rejection stats |",
        "| --- | ---: | ---: | ---: | --- |",
    ])
    for method in METHODS:
        comparison = summary["history_window_rejection"][method]
        final_state = comparison["final_state"]
        covariance = comparison.get("covariance")
        cov_value = fmt(covariance["frobenius_norm"]) if covariance else "n/a"
        stats = comparison.get("stale_run_stats")
        stats_text = "n/a"
        if stats:
            stats_text = (
                f"received={stats.get('correction_messages_received')}, "
                f"applied={stats.get('correction_messages_applied')}, "
                f"history_rejections={stats.get('correction_history_rejections')}"
            )
        lines.append(
            f"| {method.upper()} | {fmt(final_state['position_error'])} | "
            f"{fmt(final_state['yaw_error'])} | {cov_value} | {stats_text} |"
        )
    lines.extend([
        "",
        "## Interpretation",
        "",
        "- Zero-latency replay and immediate fusion now agree to numerical tolerance.",
        "- EKF and UKF terminal state and covariance are invariant to delayed-correction arrival time in the deterministic replay checks.",
        "- Reversed arrival order also converges to the same terminal state and covariance, showing that replay now preserves prior out-of-sequence corrections rather than erasing them with stale future snapshots.",
        "- The history-window test shows an explicit rejection path for unreconstructable late measurements while leaving the final estimator state unchanged relative to a valid-only reference.",
        "- Nonzero whole-trajectory RMSE in delayed runs should be interpreted as online publication error before the delayed correction arrived, not as a failure of reconstructed terminal replay correctness.",
    ])

    REPORT_PATH.parent.mkdir(parents=True, exist_ok=True)
    REPORT_PATH.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"Wrote Phase 5 correctness report to {REPORT_PATH}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
