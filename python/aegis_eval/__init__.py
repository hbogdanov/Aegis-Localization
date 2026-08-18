"""Reusable evaluation helpers for Aegis benchmarks."""

from .consistency import (
    chi_square_bounds,
    load_corruption_log_csv,
    load_filter_diagnostics_csv,
    summarize_gate_classification,
    summarize_gating,
    summarize_nis,
    summarize_planar_nees,
)
from .io import align_by_timestamp, load_trajectory_csv, merged_position_error, write_json
from .metrics import compute_metrics

__all__ = [
    "align_by_timestamp",
    "chi_square_bounds",
    "compute_metrics",
    "load_corruption_log_csv",
    "load_filter_diagnostics_csv",
    "load_trajectory_csv",
    "merged_position_error",
    "summarize_gate_classification",
    "summarize_gating",
    "summarize_nis",
    "summarize_planar_nees",
    "write_json",
]
