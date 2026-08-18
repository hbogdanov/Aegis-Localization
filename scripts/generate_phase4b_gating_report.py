#!/usr/bin/env python3
"""Generate the Phase 4b gating-sweep report."""

from __future__ import annotations

import json
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
SUMMARY_PATH = REPO_ROOT / "results" / "phase4b_gating_sweep" / "summary.json"
OUT_PATH = REPO_ROOT / "results" / "reports" / "phase4b_gating_sweep.md"


def load_json(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def fmt(value: float | None) -> str:
    if value is None:
        return "n/a"
    return f"{float(value):.4f}"


def main() -> int:
    summary = load_json(SUMMARY_PATH)
    scenarios = summary["scenarios"]
    startup = scenarios["startup_poison_gate95"]["aggregate"]
    delayed = scenarios["delayed_corruption_gate95"]["aggregate"]

    lines = [
        "# Phase 4b Gating Sweep",
        "",
        "## Diagnosis",
        "",
        "- The Phase 4 catastrophic run pattern is consistent with startup poisoning: a corrupted initialization can make almost every later pose correction look inconsistent and trigger a rejection spiral.",
        "- To test that directly, this sweep compares gating with corruption active from the first update versus the same gate with corruption delayed until after initialization.",
        "",
        "| Diagnosis Metric | Startup Poison Gate95 | Delayed Corruption Gate95 |",
        "|---|---:|---:|",
        f"| EKF ATE RMSE | {fmt(startup['metrics']['ekf']['ate_rmse']['mean'])} | {fmt(delayed['metrics']['ekf']['ate_rmse']['mean'])} |",
        f"| UKF ATE RMSE | {fmt(startup['metrics']['ukf']['ate_rmse']['mean'])} | {fmt(delayed['metrics']['ukf']['ate_rmse']['mean'])} |",
        f"| EKF rejection rate | {fmt(startup['gating']['ekf']['rejection_rate']['mean'])} | {fmt(delayed['gating']['ekf']['rejection_rate']['mean'])} |",
        f"| UKF rejection rate | {fmt(startup['gating']['ukf']['rejection_rate']['mean'])} | {fmt(delayed['gating']['ukf']['rejection_rate']['mean'])} |",
        "",
        "## Delayed-Corruption Sweep",
        "",
        "| Scenario | EKF ATE | UKF ATE | EKF reject | UKF reject | EKF TPR | EKF FPR |",
        "|---|---:|---:|---:|---:|---:|---:|",
    ]

    sweep_names = sorted(name for name in scenarios if name.startswith("baseline_") or name.startswith("gate_"))
    for name in sweep_names:
        aggregate = scenarios[name]["aggregate"]
        lines.append(
            f"| {name} | "
            f"{fmt(aggregate['metrics']['ekf']['ate_rmse']['mean'])} | "
            f"{fmt(aggregate['metrics']['ukf']['ate_rmse']['mean'])} | "
            f"{fmt(aggregate['gating']['ekf']['rejection_rate']['mean'])} | "
            f"{fmt(aggregate['gating']['ukf']['rejection_rate']['mean'])} | "
            f"{fmt(aggregate['classification']['ekf']['true_positive_rate']['mean'])} | "
            f"{fmt(aggregate['classification']['ekf']['false_positive_rate']['mean'])} |"
        )

    lines.extend([
        "",
        "## Interpretation",
        "",
        "- If delayed-corruption gating performs much better than startup-poison gating, the original Phase 4 failure mode was at least partly an initialization problem rather than ordinary update rejection alone.",
        "- The threshold sweep then shows whether a fixed chi-square gate can achieve a useful tradeoff between true outlier rejection and false rejection of valid corrections.",
        "- A good operating region would show lower ATE than the matching no-gate baseline, high true-positive rejection, and meaningfully lower false-positive rejection than the failed Phase 4 configuration.",
        "",
        "## Artifacts",
        "",
        "- Machine-readable summary: `results/phase4b_gating_sweep/summary.json`",
        "- Run-level artifacts: `results/phase4b_gating_sweep/<scenario>/repeats/run_###/`",
        "",
    ])

    OUT_PATH.parent.mkdir(parents=True, exist_ok=True)
    OUT_PATH.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"Wrote Phase 4b report to {OUT_PATH}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
