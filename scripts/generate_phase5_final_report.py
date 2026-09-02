#!/usr/bin/env python3
"""Generate the concise final evidence report for Phase 5B and Phase 5C."""

from __future__ import annotations

import json
import math
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
REPLAY_SUMMARY = REPO_ROOT / "results" / "phase5_replay_comparison" / "summary.json"
DEGRADATION_SUMMARY = REPO_ROOT / "results" / "phase5_degradation_campaign" / "summary.json"
REPORT_PATH = REPO_ROOT / "results" / "reports" / "phase5_final_report.md"


def fmt(value: float | None) -> str:
    return "n/a" if value is None or not math.isfinite(value) else f"{value:.4f}"


def metric(summary: dict[str, object], latency: str, policy: str, method: str, name: str) -> float:
    return float(summary["scenarios"][latency][policy]["aggregate"][method][name]["mean"])


def main() -> int:
    replay = json.loads(REPLAY_SUMMARY.read_text(encoding="utf-8"))
    degradation = json.loads(DEGRADATION_SUMMARY.read_text(encoding="utf-8"))
    lines = [
        "# Phase 5: Delayed And Intermittent Correction",
        "",
        "## Scope",
        "",
        "- Phase 5B compares the same delayed correction stream under naive arrival-time fusion and timestamp-aware replay.",
        "- ATE describes online published-trajectory error. Final drift describes the final current-state error after all received corrections have been incorporated.",
        "- Phase 5C holds the replay policy fixed and varies correction availability and quality with repeated seeds.",
        "- Trajectory metrics use the completed 3-repeat matrices. Pose-NIS logging was subsequently verified in a focused run; it does not alter estimator state or trajectory metrics.",
        "- Combined-degraded pose-NIS aggregation is intentionally shown as unavailable for the pre-instrumentation run, not imputed from estimator counters.",
        "",
        "## Phase 5B: Naive Fusion Versus Replay",
        "",
        "| Latency | Estimator | Naive online ATE | Replay online ATE | Naive final drift | Replay final drift | Drift improvement |",
        "| --- | --- | ---: | ---: | ---: | ---: | ---: |",
    ]
    for latency_name in replay["scenarios"]:
        for method in ("ekf", "ukf", "pf"):
            naive_ate = metric(replay, latency_name, "naive_arrival", method, "ate_rmse")
            replay_ate = metric(replay, latency_name, "timestamp_aware_replay", method, "ate_rmse")
            naive_drift = metric(replay, latency_name, "naive_arrival", method, "final_drift")
            replay_drift = metric(replay, latency_name, "timestamp_aware_replay", method, "final_drift")
            lines.append(
                f"| {latency_name} | {method.upper()} | {fmt(naive_ate)} | {fmt(replay_ate)} | "
                f"{fmt(naive_drift)} | {fmt(replay_drift)} | {fmt(naive_drift - replay_drift)} |"
            )

    lines.extend([
        "",
        "## Phase 5C: Replay Degradation Campaign",
        "",
        "| Scenario | Estimator | ATE mean +/- std | Final drift mean +/- std | Yaw RMSE mean +/- std |",
        "| --- | --- | ---: | ---: | ---: |",
    ])
    for scenario_name, scenario in degradation["scenarios"].items():
        for method in ("ekf", "ukf", "pf"):
            metrics = scenario["aggregate"]["metrics"][method]
            lines.append(
                f"| {scenario_name} | {method.upper()} | "
                f"{fmt(metrics['ate_rmse']['mean'])} +/- {fmt(metrics['ate_rmse']['std'])} | "
                f"{fmt(metrics['final_drift']['mean'])} +/- {fmt(metrics['final_drift']['std'])} | "
                f"{fmt(metrics['yaw_rmse']['mean'])} +/- {fmt(metrics['yaw_rmse']['std'])} |"
            )

    blackout = degradation["scenarios"].get("blackout_recovery")
    if blackout:
        lines.extend([
            "",
            "## Blackout Recovery",
            "",
            "| Estimator | Error at blackout start | Peak during blackout | Error 1 s after recovery | Recovery from peak |",
            "| --- | ---: | ---: | ---: | ---: |",
        ])
        for method, values in blackout["aggregate"].get("recovery", {}).items():
            lines.append(
                f"| {method.upper()} | {fmt(values['error_at_blackout_start']['mean'])} | "
                f"{fmt(values['peak_error_during_blackout']['mean'])} | "
                f"{fmt(values['error_one_second_after_recovery']['mean'])} | "
                f"{fmt(values['recovery_from_peak']['mean'])} |"
            )

    lines.extend([
        "",
        "## Consistency And Gating",
        "",
        "| Scenario | Estimator | Pose NIS in bounds | Pose NIS above upper bound | Pose-gating rejection rate |",
        "| --- | --- | ---: | ---: | ---: |",
    ])
    for scenario_name, scenario in degradation["scenarios"].items():
        for method, values in scenario["aggregate"].get("diagnostics", {}).items():
            lines.append(
                f"| {scenario_name} | {method.upper()} | "
                f"{fmt(values['pose_nis_in_bounds_fraction']['mean'])} | "
                f"{fmt(values['pose_nis_above_upper_fraction']['mean'])} | "
                f"{fmt(values['pose_gating_rejection_rate']['mean'])} |"
            )

    lines.extend([
        "",
        "## Interpretation Rules",
        "",
        "- Do not call lower online ATE proof that replay removes latency: no estimator can improve a belief published before the delayed observation arrived.",
        "- Treat lower replay final drift at the same latency as the direct evidence for timestamp-aware current-state reconstruction.",
        "- Interpret PF separately: its resampling is stochastic, so exact state equivalence is not expected from the EKF/UKF deterministic correctness tests.",
        "- Report negative or mixed gating results directly; the goal is a characterized robustness tradeoff, not a universal claim that gating helps.",
    ])
    REPORT_PATH.parent.mkdir(parents=True, exist_ok=True)
    REPORT_PATH.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"Wrote Phase 5 final report to {REPORT_PATH}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
