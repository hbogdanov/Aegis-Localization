"""Consistency-analysis helpers for Aegis benchmark runs."""

from __future__ import annotations

import csv
from dataclasses import dataclass
from pathlib import Path
from statistics import NormalDist

import numpy as np

from .io import align_by_timestamp, load_trajectory_csv
from .metrics import wrap_angle


@dataclass(frozen=True)
class DiagnosticRecord:
    source: str
    timestamp: float
    status: str
    measurement_type: str
    innovation_norm: float
    innovation_dim: int
    nis: float
    innovation_vector: np.ndarray
    innovation_covariance: np.ndarray
    state_covariance: np.ndarray


def _parse_vector(raw: str) -> np.ndarray:
    if not raw:
        return np.array([], dtype=float)
    return np.array([float(value) for value in raw.split(";") if value], dtype=float)


def _reshape_square(values: np.ndarray) -> np.ndarray:
    if values.size == 0:
        return np.empty((0, 0), dtype=float)
    side = int(round(np.sqrt(values.size)))
    if side * side != values.size:
        raise ValueError(f"Expected square covariance flattening, got length {values.size}")
    return values.reshape((side, side))


def load_filter_diagnostics_csv(path: str | Path) -> list[DiagnosticRecord]:
    csv_path = Path(path)
    if not csv_path.exists():
        raise FileNotFoundError(csv_path)

    records: list[DiagnosticRecord] = []
    with csv_path.open("r", encoding="utf-8", newline="") as handle:
        reader = csv.DictReader(handle)
        if reader.fieldnames is None:
            raise ValueError(f"CSV is missing a header row: {csv_path}")

        for row in reader:
            try:
                innovation_vector = _parse_vector(row.get("innovation_vector", ""))
                innovation_covariance = _reshape_square(_parse_vector(row.get("innovation_covariance", "")))
                state_covariance = _reshape_square(_parse_vector(row.get("state_covariance", "")))
                records.append(
                    DiagnosticRecord(
                        source=row["source"],
                        timestamp=float(row["timestamp"]),
                        status=row["status"],
                        measurement_type=row.get("measurement_type", ""),
                        innovation_norm=float(row.get("innovation_norm", "nan")),
                        innovation_dim=int(row.get("innovation_dim", 0)),
                        nis=float(row.get("nis", "nan")),
                        innovation_vector=innovation_vector,
                        innovation_covariance=innovation_covariance,
                        state_covariance=state_covariance,
                    )
                )
            except (KeyError, TypeError, ValueError):
                continue

    return records


def chi_square_bounds(dof: int, confidence: float = 0.95) -> tuple[float, float]:
    if dof <= 0:
        raise ValueError("Degrees of freedom must be positive")
    if not 0.0 < confidence < 1.0:
        raise ValueError("confidence must lie strictly between 0 and 1")

    normal = NormalDist()
    alpha = 1.0 - confidence
    lower_probability = alpha / 2.0
    upper_probability = 1.0 - alpha / 2.0

    def approximate_quantile(probability: float) -> float:
        z_score = normal.inv_cdf(probability)
        term = 1.0 - (2.0 / (9.0 * dof)) + z_score * np.sqrt(2.0 / (9.0 * dof))
        return float(dof * term ** 3)

    return approximate_quantile(lower_probability), approximate_quantile(upper_probability)


def summarize_nis(
    records: list[DiagnosticRecord],
    *,
    estimator: str,
    confidence: float = 0.95,
) -> dict[str, object]:
    estimator_records = [
        record
        for record in records
        if record.source.lower() == estimator.lower()
        and record.innovation_dim > 0
        and np.isfinite(record.nis)
    ]

    by_measurement: dict[str, dict[str, object]] = {}
    for measurement_type in sorted({record.measurement_type for record in estimator_records}):
        measurement_records = [record for record in estimator_records if record.measurement_type == measurement_type]
        if not measurement_records:
            continue
        dof = measurement_records[0].innovation_dim
        lower_bound, upper_bound = chi_square_bounds(dof, confidence=confidence)
        nis_values = np.array([record.nis for record in measurement_records], dtype=float)
        timestamps = np.array([record.timestamp for record in measurement_records], dtype=float)
        in_bounds = (nis_values >= lower_bound) & (nis_values <= upper_bound)
        by_measurement[measurement_type] = {
            "num_updates": int(nis_values.size),
            "innovation_dim": int(dof),
            "nis_mean": float(np.mean(nis_values)),
            "nis_median": float(np.median(nis_values)),
            "nis_std": float(np.std(nis_values)),
            "lower_bound": float(lower_bound),
            "upper_bound": float(upper_bound),
            "fraction_below_lower_bound": float(np.mean(nis_values < lower_bound)),
            "fraction_in_bounds": float(np.mean(in_bounds)),
            "fraction_above_upper_bound": float(np.mean(nis_values > upper_bound)),
            "violation_fraction": float(1.0 - np.mean(in_bounds)),
            "timestamps": timestamps.tolist(),
            "nis_values": nis_values.tolist(),
            "in_bounds": in_bounds.astype(int).tolist(),
        }

    return {
        "estimator": estimator.upper(),
        "confidence": confidence,
        "measurements": by_measurement,
    }


def summarize_planar_nees(
    est_path: str | Path,
    gt_path: str | Path,
    records: list[DiagnosticRecord],
    *,
    estimator: str,
    measurement_type: str = "velocity_yaw_rate",
    confidence: float = 0.95,
) -> dict[str, object]:
    estimator_records = [
        record
        for record in records
        if record.source.lower() == estimator.lower()
        and record.measurement_type == measurement_type
        and record.state_covariance.shape == (6, 6)
    ]
    if not estimator_records:
        return {
            "estimator": estimator.upper(),
            "measurement_type": measurement_type,
            "num_updates": 0,
        }

    est = load_trajectory_csv(est_path)
    gt = load_trajectory_csv(gt_path)
    diag_timestamps = np.array([record.timestamp for record in estimator_records], dtype=float)
    est_indices = np.searchsorted(est["timestamp"], diag_timestamps, side="left")
    est_indices = np.clip(est_indices, 0, est["timestamp"].size - 1)
    previous_indices = np.clip(est_indices - 1, 0, est["timestamp"].size - 1)
    choose_previous = np.abs(est["timestamp"][previous_indices] - diag_timestamps) < np.abs(
        est["timestamp"][est_indices] - diag_timestamps
    )
    est_indices = np.where(choose_previous, previous_indices, est_indices)
    diag_traj = {
        "timestamp": diag_timestamps,
        "x": est["x"][est_indices],
        "y": est["y"][est_indices],
        "yaw": est["yaw"][est_indices],
    }
    aligned = align_by_timestamp(diag_traj, gt)
    errors = np.column_stack(
        [
            aligned["x_est"] - aligned["x_gt"],
            aligned["y_est"] - aligned["y_gt"],
            wrap_angle(aligned["yaw_est"] - aligned["yaw_gt"]),
        ]
    )

    nees_values: list[float] = []
    for index, record in enumerate(estimator_records[: errors.shape[0]]):
        covariance = record.state_covariance[np.ix_([0, 1, 2], [0, 1, 2])]
        try:
            solved = np.linalg.solve(covariance, errors[index])
        except np.linalg.LinAlgError:
            continue
        nees_values.append(float(errors[index] @ solved))

    if not nees_values:
        return {
            "estimator": estimator.upper(),
            "measurement_type": measurement_type,
            "num_updates": 0,
        }

    dof = 3
    lower_bound, upper_bound = chi_square_bounds(dof, confidence=confidence)
    nees_array = np.array(nees_values, dtype=float)
    in_bounds = (nees_array >= lower_bound) & (nees_array <= upper_bound)
    return {
        "estimator": estimator.upper(),
        "measurement_type": measurement_type,
        "num_updates": int(nees_array.size),
        "state_dim": dof,
        "nees_mean": float(np.mean(nees_array)),
        "nees_median": float(np.median(nees_array)),
        "nees_std": float(np.std(nees_array)),
        "lower_bound": float(lower_bound),
        "upper_bound": float(upper_bound),
        "fraction_below_lower_bound": float(np.mean(nees_array < lower_bound)),
        "fraction_in_bounds": float(np.mean(in_bounds)),
        "fraction_above_upper_bound": float(np.mean(nees_array > upper_bound)),
    }
