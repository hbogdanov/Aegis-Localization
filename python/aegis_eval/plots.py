"""Plotting helpers for canonical Aegis trajectory outputs."""

from __future__ import annotations

from pathlib import Path

import numpy as np

from .io import align_by_timestamp, load_trajectory_csv, merged_position_error


def _ensure_parent(path: str | Path) -> Path:
    out_path = Path(path)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    return out_path


def _require_matplotlib() -> None:
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt_module
    except Exception as exc:  # pragma: no cover - depends on local plotting stack
        raise RuntimeError(f"matplotlib is unavailable: {exc}") from exc
    return plt_module


def plot_trajectory_overlay(est_path: str | Path, gt_path: str | Path, out_path: str | Path) -> Path:
    plt = _require_matplotlib()
    est = load_trajectory_csv(est_path)
    gt = load_trajectory_csv(gt_path)
    resolved_out = _ensure_parent(out_path)

    plt.figure(figsize=(8, 8))
    plt.plot(gt["x"], gt["y"], label="ground truth", linewidth=2)
    plt.plot(est["x"], est["y"], label="estimate", linewidth=2)
    plt.xlabel("x (m)")
    plt.ylabel("y (m)")
    plt.title("Trajectory: estimate vs ground truth")
    plt.axis("equal")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(resolved_out)
    plt.close()
    return resolved_out


def plot_position_error(est_path: str | Path, gt_path: str | Path, out_path: str | Path) -> Path:
    plt = _require_matplotlib()
    est = load_trajectory_csv(est_path)
    gt = load_trajectory_csv(gt_path)
    merged = align_by_timestamp(est, gt)
    pos_err = merged_position_error(merged)
    resolved_out = _ensure_parent(out_path)

    plt.figure(figsize=(8, 3))
    plt.plot(np.asarray(merged["timestamp"]), pos_err, label="position error (m)")
    plt.xlabel("timestamp")
    plt.ylabel("position error (m)")
    plt.title("Position error over time")
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(resolved_out)
    plt.close()
    return resolved_out


def plot_nis_series(
    timestamps: list[float],
    nis_values: list[float],
    lower_bound: float,
    upper_bound: float,
    out_path: str | Path,
    *,
    title: str,
) -> Path:
    plt = _require_matplotlib()
    resolved_out = _ensure_parent(out_path)
    plt.figure(figsize=(8, 3))
    plt.plot(timestamps, nis_values, label="NIS", linewidth=1.5)
    plt.axhline(lower_bound, color="tab:green", linestyle="--", label="95% bounds")
    plt.axhline(upper_bound, color="tab:green", linestyle="--")
    plt.xlabel("timestamp")
    plt.ylabel("NIS")
    plt.title(title)
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.savefig(resolved_out)
    plt.close()
    return resolved_out


def plot_consistency_fraction(
    fractions: dict[str, float],
    out_path: str | Path,
    *,
    title: str,
) -> Path:
    plt = _require_matplotlib()
    resolved_out = _ensure_parent(out_path)
    labels = list(fractions.keys())
    values = [fractions[label] for label in labels]
    plt.figure(figsize=(6, 3))
    plt.bar(labels, values, color="tab:blue")
    plt.ylim(0.0, 1.0)
    plt.ylabel("fraction in bounds")
    plt.title(title)
    plt.grid(True, axis="y")
    plt.tight_layout()
    plt.savefig(resolved_out)
    plt.close()
    return resolved_out
