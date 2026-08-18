#!/usr/bin/env python3
"""Generate a compact report for the Phase 4 gating experiment."""

from __future__ import annotations

import json
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
SUMMARY_PATH = REPO_ROOT / "results" / "phase4_gating" / "summary.json"
OUT_PATH = REPO_ROOT / "results" / "reports" / "phase4_gating_report.md"


def load_json(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def fmt(value: float) -> str:
    return f"{float(value):.4f}"


def main() -> int:
    summary = load_json(SUMMARY_PATH)
    no_gate = summary["scenarios"]["corrupted_pose_no_gating"]["aggregate"]
    with_gate = summary["scenarios"]["corrupted_pose_with_gating"]["aggregate"]
    ekf_gate_better = with_gate["metrics"]["ekf"]["ate_rmse_mean"] < no_gate["metrics"]["ekf"]["ate_rmse_mean"]
    ukf_gate_better = with_gate["metrics"]["ukf"]["ate_rmse_mean"] < no_gate["metrics"]["ukf"]["ate_rmse_mean"]

    lines = [
        "# Phase 4 Gating Report",
        "",
        "## Methodology",
        "",
        "- Synthetic pose-like outliers were injected into the odometry pose correction stream while velocity and yaw-rate measurements remained on the nominal path.",
        "- EKF and UKF were compared with identical seeds and corruption settings, once with pose gating disabled and once with Mahalanobis-distance pose gating enabled.",
        "- The gating threshold was the same across repeats and was recorded in run metadata and estimator stats.",
        "",
        "## Aggregate Metrics",
        "",
        "| Metric | EKF No Gate | EKF Gate | UKF No Gate | UKF Gate |",
        "|---|---:|---:|---:|---:|",
        f"| ATE RMSE | {fmt(no_gate['metrics']['ekf']['ate_rmse_mean'])} | {fmt(with_gate['metrics']['ekf']['ate_rmse_mean'])} | {fmt(no_gate['metrics']['ukf']['ate_rmse_mean'])} | {fmt(with_gate['metrics']['ukf']['ate_rmse_mean'])} |",
        f"| Final Drift | {fmt(no_gate['metrics']['ekf']['final_drift_mean'])} | {fmt(with_gate['metrics']['ekf']['final_drift_mean'])} | {fmt(no_gate['metrics']['ukf']['final_drift_mean'])} | {fmt(with_gate['metrics']['ukf']['final_drift_mean'])} |",
        f"| Yaw RMSE | {fmt(no_gate['metrics']['ekf']['yaw_rmse_mean'])} | {fmt(with_gate['metrics']['ekf']['yaw_rmse_mean'])} | {fmt(no_gate['metrics']['ukf']['yaw_rmse_mean'])} | {fmt(with_gate['metrics']['ukf']['yaw_rmse_mean'])} |",
        "",
        "## Gating Behavior",
        "",
        "| Metric | EKF No Gate | EKF Gate | UKF No Gate | UKF Gate |",
        "|---|---:|---:|---:|---:|",
        f"| Mean rejection rate | {fmt(no_gate['gating']['ekf']['rejection_rate_mean'])} | {fmt(with_gate['gating']['ekf']['rejection_rate_mean'])} | {fmt(no_gate['gating']['ukf']['rejection_rate_mean'])} | {fmt(with_gate['gating']['ukf']['rejection_rate_mean'])} |",
        f"| Mean rejected pose updates | {fmt(no_gate['gating']['ekf']['rejected_count_mean'])} | {fmt(with_gate['gating']['ekf']['rejected_count_mean'])} | {fmt(no_gate['gating']['ukf']['rejected_count_mean'])} | {fmt(with_gate['gating']['ukf']['rejected_count_mean'])} |",
        "",
        "## Interpretation",
        "",
        f"- EKF gating {'reduced' if ekf_gate_better else 'increased'} mean ATE from {fmt(no_gate['metrics']['ekf']['ate_rmse_mean'])} to {fmt(with_gate['metrics']['ekf']['ate_rmse_mean'])}.",
        f"- UKF gating {'reduced' if ukf_gate_better else 'increased'} mean ATE from {fmt(no_gate['metrics']['ukf']['ate_rmse_mean'])} to {fmt(with_gate['metrics']['ukf']['ate_rmse_mean'])}.",
        f"- The current threshold rejected about {fmt(with_gate['gating']['ekf']['rejection_rate_mean'])} of EKF pose updates and {fmt(with_gate['gating']['ukf']['rejection_rate_mean'])} of UKF pose updates on average.",
        "- This means Phase 4 is functionally complete, but the first gating configuration did not deliver a net robustness win under this corruption regime.",
        "- The most concerning evidence is not that gating rejected bad updates, but that one gated seed became strongly over-rejective and produced much worse pose and yaw error. That points to a threshold or measurement-model problem rather than an implementation failure.",
        "- PF is preserved as a performance baseline but is intentionally excluded from covariance-gating conclusions.",
        "",
        "## Artifacts",
        "",
        "- Machine-readable summary: `results/phase4_gating/summary.json`",
        "- Run-local stats and diagnostics: `results/phase4_gating/<scenario>/repeats/run_###/`",
        "",
    ]

    OUT_PATH.parent.mkdir(parents=True, exist_ok=True)
    OUT_PATH.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"Wrote Phase 4 report to {OUT_PATH}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
