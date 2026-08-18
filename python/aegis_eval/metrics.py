"""Metric helpers for Aegis benchmark runs."""

from __future__ import annotations

import numpy as np

from .io import merged_position_error


def wrap_angle(diff: np.ndarray) -> np.ndarray:
    return np.arctan2(np.sin(diff), np.cos(diff))


def compute_metrics(merged: dict[str, np.ndarray]) -> dict[str, float | int]:
    pos_err = merged_position_error(merged)
    ate_rmse = float(np.sqrt(np.mean(pos_err ** 2))) if pos_err.size > 0 else float("nan")
    final_drift = float(pos_err[-1]) if pos_err.size > 0 else float("nan")

    yaw_diff = wrap_angle(merged["yaw_est"] - merged["yaw_gt"])
    yaw_rmse = float(np.sqrt(np.mean(yaw_diff ** 2))) if yaw_diff.size > 0 else float("nan")

    return {
        "num_samples": int(merged["timestamp"].size),
        "ate_rmse": ate_rmse,
        "final_drift": final_drift,
        "yaw_rmse": yaw_rmse,
    }
