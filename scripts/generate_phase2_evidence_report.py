#!/usr/bin/env python3
"""Generate a compact synthetic-vs-EuRoC comparison report."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys


REPO_ROOT = Path(__file__).resolve().parents[1]
RESULTS_ROOT = REPO_ROOT / "results"


def load_json(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def format_metric(value: float) -> str:
    return f"{value:.6f}"


def extract_summary_triplet(metrics: dict) -> tuple[float, float, float]:
    ate = metrics["ate_rmse"]
    drift = metrics["final_drift"]
    yaw = metrics["yaw_rmse"]
    if isinstance(ate, dict):
        return float(ate["mean"]), float(drift["mean"]), float(yaw["mean"])
    return float(ate), float(drift), float(yaw)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--campaign-summary", default=str(RESULTS_ROOT / "campaign" / "summary.json"))
    parser.add_argument("--euroc-summary", default=str(RESULTS_ROOT / "euroc" / "MH_01_easy" / "full_mh01_phase1_flush" / "metrics" / "summary.json"))
    parser.add_argument("--out", default=str(RESULTS_ROOT / "reports" / "phase2_low_noise_vs_euroc_mh01.md"))
    args = parser.parse_args()

    campaign_summary = load_json(Path(args.campaign_summary))
    euroc_summary = load_json(Path(args.euroc_summary))

    synthetic = campaign_summary["scenarios"]["low_noise"]
    synthetic_metrics = synthetic.get("aggregate_metrics") or synthetic["metrics"]
    euroc_metrics = euroc_summary["metrics"]

    lines = [
        "# Phase 2 Comparison Report",
        "",
        f"- Synthetic baseline scenario: `low_noise`",
        f"- EuRoC sequence: `MH_01_easy`",
        f"- EuRoC replay mode: `{euroc_summary.get('replay_mode', 'unknown')}`",
        "",
        "## Per-Estimator Metrics",
        "",
        "| Estimator | Synthetic ATE | Synthetic Drift | Synthetic Yaw | EuRoC ATE | EuRoC Drift | EuRoC Yaw |",
        "|---|---:|---:|---:|---:|---:|---:|",
    ]

    for method in ("ekf", "ukf", "pf"):
        syn = synthetic_metrics[method]
        syn_ate, syn_drift, syn_yaw = extract_summary_triplet(syn)
        eur = euroc_metrics[method]
        lines.append(
            f"| {method.upper()} | {format_metric(syn_ate)} | {format_metric(syn_drift)} | {format_metric(syn_yaw)} | "
            f"{format_metric(eur['ate_rmse'])} | {format_metric(eur['final_drift'])} | {format_metric(eur['yaw_rmse'])} |"
        )

    lines.extend([
        "",
        "## Delta Table",
        "",
        "| Estimator | Delta ATE (EuRoC - Synthetic) | Delta Drift | Delta Yaw |",
        "|---|---:|---:|---:|",
    ])

    for method in ("ekf", "ukf", "pf"):
        syn = synthetic_metrics[method]
        syn_ate, syn_drift, syn_yaw = extract_summary_triplet(syn)
        eur = euroc_metrics[method]
        lines.append(
            f"| {method.upper()} | {format_metric(eur['ate_rmse'] - syn_ate)} | "
            f"{format_metric(eur['final_drift'] - syn_drift)} | "
            f"{format_metric(eur['yaw_rmse'] - syn_yaw)} |"
        )

    lines.extend([
        "",
        "## Notes",
        "",
        "- EuRoC is harder because it is a recorded 6-DoF MAV trajectory reduced to a planar proxy rather than a clean synthetic 2D benchmark.",
        "- The EuRoC proxy preserves real recorded timing and motion, but it does not provide a wheel-encoder-faithful or native MAV localization evaluation.",
        "- Translation metrics remain useful evidence for the recorded-data path, while yaw should still be treated cautiously because the current planar proxy does not make heading the strongest validation target.",
    ])

    out_path = Path(args.out)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"Wrote Phase 2 evidence report to {out_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
