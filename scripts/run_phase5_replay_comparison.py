#!/usr/bin/env python3
"""Compare naive late fusion with timestamp-aware correction replay."""

from __future__ import annotations

import argparse
import json
from datetime import datetime, timezone
from pathlib import Path

from phase5_experiment_utils import METHODS, REPO_ROOT, aggregate_scalar_records, run_scenario


OUTPUT_ROOT = REPO_ROOT / "results" / "phase5_replay_comparison"
LATENCIES = (0.0, 0.1, 0.5, 1.0)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=int, default=12)
    parser.add_argument("--repeats", type=int, default=3)
    parser.add_argument("--base-seed", type=int, default=5100)
    parser.add_argument(
        "--latencies",
        nargs="+",
        type=float,
        default=list(LATENCIES),
        help="Latency values in seconds; use one value for a short smoke check.",
    )
    args = parser.parse_args()

    summary: dict[str, object] = {
        "run_started_at_utc": datetime.now(timezone.utc).isoformat(),
        "duration_seconds": args.duration,
        "repeats": args.repeats,
        "latencies_seconds": args.latencies,
        "policies": {"naive_arrival": False, "timestamp_aware_replay": True},
        "scenarios": {},
    }
    common = {
        "use_odom_pose_update": "false",
        "correction_enabled": "true",
        "correction_frequency_hz": "5.0",
        "correction_dropout_probability": "0.0",
        "correction_position_noise_std": "0.05",
        "correction_yaw_noise_std": "0.08",
        "correction_outlier_probability": "0.0",
        "odom_position_noise_std": "0.08",
        "odom_velocity_noise_std": "0.05",
        "imu_yaw_rate_noise_std": "0.03",
        "dropout_probability": "0.0",
        "max_history_seconds": "3.0",
    }

    for latency in args.latencies:
        latency_name = f"latency_{int(latency * 1000):04d}ms"
        summary["scenarios"][latency_name] = {}
        for policy_name, replay_enabled in summary["policies"].items():
            run_entries = []
            for repeat in range(args.repeats):
                seed = args.base_seed + int(latency * 1000) * 10 + repeat
                root = OUTPUT_ROOT / latency_name / policy_name / "repeats" / f"run_{repeat + 1:03d}"
                result = run_scenario(
                    root,
                    {
                        **common,
                        "correction_latency_seconds": str(latency),
                        "correction_replay_enabled": "true" if replay_enabled else "false",
                        "fake_sensor_seed": str(seed),
                        "pf_random_seed": str(seed + 1),
                    },
                    args.duration,
                )
                run_entries.append({"seed": seed, **result})

            aggregates = {}
            for method in METHODS:
                aggregates[method] = aggregate_scalar_records(
                    [entry["metrics"][method] for entry in run_entries]
                )
            summary["scenarios"][latency_name][policy_name] = {
                "runs": run_entries,
                "aggregate": aggregates,
            }

    OUTPUT_ROOT.mkdir(parents=True, exist_ok=True)
    (OUTPUT_ROOT / "summary.json").write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    print(f"Wrote replay comparison to {OUTPUT_ROOT / 'summary.json'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
