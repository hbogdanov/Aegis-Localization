"""Trajectory I/O and alignment helpers."""

from __future__ import annotations

import csv
import json
from pathlib import Path

import numpy as np


REQUIRED_TRAJECTORY_COLUMNS = ("timestamp", "x", "y", "yaw")


def load_trajectory_csv(path: str | Path) -> dict[str, np.ndarray]:
    csv_path = Path(path)
    if not csv_path.exists():
        raise FileNotFoundError(csv_path)

    with csv_path.open("r", encoding="utf-8", newline="") as handle:
        reader = csv.DictReader(handle)
        if reader.fieldnames is None:
            raise ValueError(f"CSV is missing a header row: {csv_path}")

        for column in REQUIRED_TRAJECTORY_COLUMNS:
            if column not in reader.fieldnames:
                raise ValueError(f"CSV missing '{column}' column: {csv_path}")

        rows: list[tuple[float, float, float, float]] = []
        for row in reader:
            try:
                timestamp = float(row["timestamp"])
                x = float(row["x"])
                y = float(row["y"])
                yaw = float(row["yaw"])
            except (TypeError, ValueError):
                continue
            rows.append((timestamp, x, y, yaw))

    if not rows:
        raise ValueError(f"CSV contains no valid trajectory samples: {csv_path}")

    rows.sort(key=lambda row: row[0])
    deduplicated_rows: list[tuple[float, float, float, float]] = []
    for row in rows:
        if deduplicated_rows and row[0] == deduplicated_rows[-1][0]:
            deduplicated_rows[-1] = row
        else:
            deduplicated_rows.append(row)
    return {
        "timestamp": np.array([row[0] for row in deduplicated_rows], dtype=float),
        "x": np.array([row[1] for row in deduplicated_rows], dtype=float),
        "y": np.array([row[2] for row in deduplicated_rows], dtype=float),
        "yaw": np.array([row[3] for row in deduplicated_rows], dtype=float),
    }


def align_by_timestamp(est: dict[str, np.ndarray], gt: dict[str, np.ndarray]) -> dict[str, np.ndarray]:
    est_ts = est["timestamp"]
    gt_ts = gt["timestamp"]
    if est_ts.size == 0 or gt_ts.size == 0:
        raise ValueError("No aligned samples were found between estimate and ground-truth trajectories")

    insertion_indices = np.searchsorted(gt_ts, est_ts, side="left")
    nearest_indices = np.empty(est_ts.size, dtype=int)
    for est_index, insert_index in enumerate(insertion_indices):
        candidates = []
        if insert_index < gt_ts.size:
            candidates.append(insert_index)
        if insert_index > 0:
            candidates.append(insert_index - 1)
        if not candidates:
            raise ValueError("No aligned samples were found between estimate and ground-truth trajectories")
        nearest_indices[est_index] = min(
            candidates,
            key=lambda candidate_index: abs(gt_ts[candidate_index] - est_ts[est_index]),
        )

    return {
        "timestamp": est_ts,
        "x_est": est["x"],
        "y_est": est["y"],
        "yaw_est": est["yaw"],
        "x_gt": gt["x"][nearest_indices],
        "y_gt": gt["y"][nearest_indices],
        "yaw_gt": gt["yaw"][nearest_indices],
    }


def merged_position_error(merged: dict[str, np.ndarray]) -> np.ndarray:
    dx = merged["x_est"] - merged["x_gt"]
    dy = merged["y_est"] - merged["y_gt"]
    return np.sqrt(dx * dx + dy * dy)


def write_json(path: str | Path, payload: object) -> None:
    out_path = Path(path)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    with out_path.open("w", encoding="utf-8") as handle:
        json.dump(payload, handle, indent=2)
