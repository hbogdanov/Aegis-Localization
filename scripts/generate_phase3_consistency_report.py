#!/usr/bin/env python3
"""Generate the final Phase 3 estimator-consistency report."""

from __future__ import annotations

import argparse
import json
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
RESULTS_ROOT = REPO_ROOT / "results"


def load_json(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def fmt(value: float | int | None) -> str:
    if value is None:
        return "n/a"
    return f"{float(value):.4f}"


def table_for_measurement(scenario_name: str, scenario: dict, measurement_type: str) -> list[str]:
    lines = [
        f"### {scenario_name.replace('_', ' ').title()} - {measurement_type}",
        "",
        "| Metric | EKF | UKF |",
        "|---|---:|---:|",
    ]
    for label, key in [
        ("Mean NIS", "nis_mean"),
        ("Median NIS", "nis_median"),
        ("Expected DOF", "innovation_dim"),
        ("Within 95% bounds", "fraction_in_bounds"),
        ("Below lower bound", "fraction_below_lower_bound"),
        ("Above upper bound", "fraction_above_upper_bound"),
        ("Sample count", "num_updates"),
    ]:
        ekf_value = scenario["aggregate_consistency"]["ekf"]["nis"].get(measurement_type, {}).get(key, {}).get("mean")
        ukf_value = scenario["aggregate_consistency"]["ukf"]["nis"].get(measurement_type, {}).get(key, {}).get("mean")
        lines.append(f"| {label} | {fmt(ekf_value)} | {fmt(ukf_value)} |")
    lines.append("")
    return lines


def nees_table_for_scenario(scenario_name: str, scenario: dict) -> list[str]:
    lines = [
        f"### {scenario_name.replace('_', ' ').title()} - Synthetic Planar NEES",
        "",
        "| Metric | EKF | UKF |",
        "|---|---:|---:|",
    ]
    for label, key in [
        ("Mean NEES", "nees_mean"),
        ("Median NEES", "nees_median"),
        ("State DOF", "state_dim"),
        ("Within 95% bounds", "fraction_in_bounds"),
        ("Below lower bound", "fraction_below_lower_bound"),
        ("Above upper bound", "fraction_above_upper_bound"),
        ("Sample count", "num_updates"),
    ]:
        ekf_value = scenario["aggregate_consistency"]["ekf"]["planar_nees"].get(key, {}).get("mean")
        ukf_value = scenario["aggregate_consistency"]["ukf"]["planar_nees"].get(key, {}).get("mean")
        lines.append(f"| {label} | {fmt(ekf_value)} | {fmt(ukf_value)} |")
    lines.append("")
    return lines


def performance_consistency_lines(scenario_name: str, scenario: dict) -> list[str]:
    lines = [f"### {scenario_name.replace('_', ' ').title()}", ""]
    for method in ("ekf", "ukf"):
        performance = scenario["aggregate_metrics"][method]
        consistency = scenario["aggregate_consistency"][method]["nis"].get("velocity_yaw_rate", {})
        ate = performance["ate_rmse"]["mean"]
        drift = performance["final_drift"]["mean"]
        nis = consistency.get("nis_mean", {}).get("mean")
        in_bounds = consistency.get("fraction_in_bounds", {}).get("mean")
        interpretation = []
        if nis is not None and in_bounds is not None:
            if in_bounds < 0.80:
                interpretation.append("consistency is weak enough to suggest overconfidence or model mismatch")
            elif in_bounds > 0.98:
                interpretation.append("consistency is very conservative or well matched")
            else:
                interpretation.append("consistency is broadly plausible")
        if drift > 0.10:
            interpretation.append("performance degradation is materially visible in final drift")
        elif ate < 0.08:
            interpretation.append("translation error stays in the low-error regime")
        lines.append(
            f"- {method.upper()}: ATE {fmt(ate)}, final drift {fmt(drift)}, velocity/yaw-rate mean NIS {fmt(nis)}, "
            f"fraction in bounds {fmt(in_bounds)}. This suggests {'; '.join(interpretation) if interpretation else 'no interpretation available'}."
        )
    lines.append("")
    return lines


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--campaign-summary", default=str(RESULTS_ROOT / "campaign" / "summary.json"))
    parser.add_argument("--out", default=str(RESULTS_ROOT / "reports" / "phase3_estimator_consistency.md"))
    args = parser.parse_args()

    summary = load_json(Path(args.campaign_summary))
    out_path = Path(args.out)
    out_path.parent.mkdir(parents=True, exist_ok=True)

    lines = [
        "# Phase 3 Estimator Consistency Report",
        "",
        "## Methodology",
        "",
        "- NIS is evaluated per update as `nu^T S^-1 nu`, using a linear solve against the logged innovation covariance rather than an explicit matrix inverse.",
        "- NIS is reported separately by measurement type so pose updates and velocity/yaw-rate updates are not mixed into one statistical bucket.",
        "- The primary consistency interval uses two-sided 95% chi-square bounds.",
        "- NEES is reported only for synthetic runs, where the planar `x`, `y`, `yaw` state, wrapped yaw error, and the corresponding covariance submatrix can be interpreted consistently.",
        "- PF remains part of Phase 2 performance comparison but is intentionally excluded from covariance-consistency analysis.",
        "",
        "## Aggregate Repeated-Trial NIS",
        "",
    ]

    for scenario_name, scenario in summary["scenarios"].items():
        for measurement_type in ("pose", "velocity_yaw_rate"):
            if measurement_type in scenario["aggregate_consistency"]["ekf"]["nis"]:
                lines.extend(table_for_measurement(scenario_name, scenario, measurement_type))

    lines.extend([
        "## Synthetic-Only NEES",
        "",
    ])
    for scenario_name, scenario in summary["scenarios"].items():
        if scenario["aggregate_consistency"]["ekf"]["planar_nees"]:
            lines.extend(nees_table_for_scenario(scenario_name, scenario))

    lines.extend([
        "## Performance and Consistency",
        "",
        "- Phase 2 established translational performance and drift behavior across the same synthetic scenarios. This section checks whether EKF and UKF uncertainty reflected those outcomes.",
        "",
    ])
    for scenario_name, scenario in summary["scenarios"].items():
        lines.extend(performance_consistency_lines(scenario_name, scenario))

    lines.extend([
        "## Limitations",
        "",
        "- The chi-square bounds are approximate but consistent across all reported runs and dimensions.",
        "- NEES is intentionally limited to the synthetic planar benchmark and should not be generalized to EuRoC proxy runs.",
        "- Plotting is optional and may be skipped on machines with a broken matplotlib stack; the JSON and CSV artifacts remain the source of record.",
        "",
        "## Artifacts",
        "",
        "- Machine-readable campaign summary: `results/campaign/summary.json`",
        "- Run-level consistency summaries: `results/campaign/<scenario>/repeats/run_###/consistency_summary.json`",
        "- Run-level diagnostics CSVs: `results/campaign/<scenario>/repeats/run_###/filter_diagnostics.csv`",
        "",
    ])

    out_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"Wrote Phase 3 consistency report to {out_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
