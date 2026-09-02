#!/usr/bin/env python3
"""Run the compact repeated-seed Phase 5 correction-degradation campaign."""

from __future__ import annotations

import argparse
import json
from datetime import datetime, timezone
from pathlib import Path

from phase5_experiment_utils import (
    METHODS,
    REPO_ROOT,
    aggregate_scalar_records,
    diagnostic_scalars,
    dropout_recovery_metrics,
    run_scenario,
)


OUTPUT_ROOT = REPO_ROOT / "results" / "phase5_degradation_campaign"
SCENARIOS = [
    ("reference", {"correction_frequency_hz": "5.0"}),
    ("low_frequency", {"correction_frequency_hz": "1.0"}),
    ("random_dropout", {"correction_dropout_probability": "0.5"}),
    ("delayed", {"correction_latency_seconds": "0.5"}),
    ("noisy", {"correction_position_noise_std": "0.15", "correction_yaw_noise_std": "0.15"}),
    ("corrupted_no_gate", {"correction_outlier_probability": "0.10"}),
    (
        "corrupted_gated",
        {"correction_outlier_probability": "0.10", "pose_gating_enabled": "true"},
    ),
    (
        "blackout_recovery",
        {
            "correction_blackout_start_seconds": "4.0",
            "correction_blackout_duration_seconds": "3.0",
        },
    ),
    (
        "combined_degraded",
        {
            "correction_frequency_hz": "2.0",
            "correction_dropout_probability": "0.35",
            "correction_latency_seconds": "0.5",
            "correction_position_noise_std": "0.10",
            "correction_yaw_noise_std": "0.12",
            "correction_outlier_probability": "0.10",
            "pose_gating_enabled": "true",
        },
    ),
]


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=int, default=12)
    parser.add_argument("--repeats", type=int, default=3)
    parser.add_argument("--base-seed", type=int, default=5600)
    parser.add_argument(
        "--scenarios",
        nargs="+",
        choices=[name for name, _overrides in SCENARIOS],
        default=[name for name, _overrides in SCENARIOS],
        help="Scenario names; use one value for a short smoke check.",
    )
    args = parser.parse_args()

    common = {
        "use_odom_pose_update": "false",
        "correction_enabled": "true",
        "correction_replay_enabled": "true",
        "correction_frequency_hz": "5.0",
        "correction_dropout_probability": "0.0",
        "correction_latency_seconds": "0.0",
        "correction_position_noise_std": "0.05",
        "correction_yaw_noise_std": "0.08",
        "correction_outlier_probability": "0.0",
        "correction_outlier_position_std": "1.5",
        "correction_outlier_yaw_std": "0.75",
        "correction_blackout_start_seconds": "-1.0",
        "correction_blackout_duration_seconds": "0.0",
        "pose_gating_enabled": "false",
        "pose_gating_threshold": "9.324146034653893",
        "odom_position_noise_std": "0.08",
        "odom_velocity_noise_std": "0.05",
        "imu_yaw_rate_noise_std": "0.03",
        "dropout_probability": "0.0",
        "max_history_seconds": "3.0",
    }
    summary: dict[str, object] = {
        "run_started_at_utc": datetime.now(timezone.utc).isoformat(),
        "duration_seconds": args.duration,
        "repeats": args.repeats,
        "scenarios": {},
    }

    for scenario_index, (name, overrides) in enumerate(SCENARIOS):
        if name not in args.scenarios:
            continue
        run_entries = []
        launch_base = {**common, **overrides}
        for repeat in range(args.repeats):
            seed = args.base_seed + scenario_index * 100 + repeat
            root = OUTPUT_ROOT / name / "repeats" / f"run_{repeat + 1:03d}"
            result = run_scenario(
                root,
                {**launch_base, "fake_sensor_seed": str(seed), "pf_random_seed": str(seed + 1)},
                args.duration,
            )
            recovery = {}
            diagnostics = diagnostic_scalars(root)
            if name == "blackout_recovery":
                for method in METHODS:
                    recovery[method] = dropout_recovery_metrics(
                        root / f"{method}.csv",
                        root / "ground_truth.csv",
                        blackout_start=4.0,
                        blackout_duration=3.0,
                    )
            run_entries.append({"seed": seed, **result, "diagnostics": diagnostics, "recovery": recovery})

        aggregate = {
            "metrics": {
                method: aggregate_scalar_records([entry["metrics"][method] for entry in run_entries])
                for method in METHODS
            },
            "diagnostics": {
                method: aggregate_scalar_records(
                    [entry["diagnostics"][method] for entry in run_entries]
                )
                for method in ("ekf", "ukf")
                if all(method in entry["diagnostics"] for entry in run_entries)
            },
        }
        if name == "blackout_recovery":
            aggregate["recovery"] = {
                method: aggregate_scalar_records([entry["recovery"][method] for entry in run_entries])
                for method in METHODS
            }
        summary["scenarios"][name] = {
            "launch_args": launch_base,
            "runs": run_entries,
            "aggregate": aggregate,
        }

    OUTPUT_ROOT.mkdir(parents=True, exist_ok=True)
    (OUTPUT_ROOT / "summary.json").write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    print(f"Wrote degradation campaign to {OUTPUT_ROOT / 'summary.json'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
