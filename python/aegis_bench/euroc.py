"""Helpers for a minimal EuRoC planar proxy backend."""

from __future__ import annotations

import csv
import math
from dataclasses import dataclass
from pathlib import Path
import zipfile


@dataclass(slots=True)
class PlanarSample:
    timestamp: float
    x: float
    y: float
    yaw: float
    vx: float
    vy: float
    omega: float
    imu_omega_z: float


def _normalize_fieldnames(fieldnames: list[str] | None) -> dict[str, str]:
    if fieldnames is None:
        raise ValueError("CSV is missing a header row")
    mapping: dict[str, str] = {}
    for name in fieldnames:
        canonical = name.strip().lstrip("#")
        mapping[canonical] = name
    return mapping


def _require_field(mapping: dict[str, str], canonical_name: str) -> str:
    if canonical_name not in mapping:
        raise ValueError(f"CSV missing required column '{canonical_name}'")
    return mapping[canonical_name]


def _require_any_field(mapping: dict[str, str], canonical_names: list[str]) -> str:
    for name in canonical_names:
        if name in mapping:
            return mapping[name]
    raise ValueError(f"CSV missing required column variants {canonical_names}")


def _quaternion_to_yaw(w: float, x: float, y: float, z: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def _wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def _resolve_sequence_inputs(sequence_root: str | Path) -> tuple[Path | None, Path | None]:
    root = Path(sequence_root)

    if root.is_file() and root.suffix.lower() == ".zip":
        return None, root

    if (root / "mav0").exists():
        return root, None

    zip_candidates = sorted(root.glob("*.zip"))
    if zip_candidates:
        return None, zip_candidates[0]

    raise FileNotFoundError(f"Could not find EuRoC mav0 directory or zip under: {root}")


def _read_rows_from_zip(zip_path: Path, member_name: str) -> list[dict[str, str]]:
    with zipfile.ZipFile(zip_path) as archive:
        with archive.open(member_name, "r") as handle:
            text_handle = (line.decode("utf-8-sig") for line in handle)
            reader = csv.DictReader(text_handle)
            return list(reader)


def _load_rows_from_csv(path: Path) -> list[dict[str, str]]:
    with path.open("r", encoding="utf-8-sig", newline="") as handle:
        reader = csv.DictReader(handle)
        return list(reader)


def load_euroc_ground_truth(sequence_root: str | Path) -> list[dict[str, float]]:
    directory_root, zip_path = _resolve_sequence_inputs(sequence_root)
    rows_source: list[dict[str, str]]
    source_name: str
    if zip_path is not None:
        source_name = f"{zip_path}!mav0/state_groundtruth_estimate0/data.csv"
        rows_source = _read_rows_from_zip(zip_path, "mav0/state_groundtruth_estimate0/data.csv")
    else:
        gt_path = directory_root / "mav0" / "state_groundtruth_estimate0" / "data.csv"
        if not gt_path.exists():
            raise FileNotFoundError(gt_path)
        source_name = str(gt_path)
        rows_source = _load_rows_from_csv(gt_path)

    rows: list[dict[str, float]] = []
    mapping = _normalize_fieldnames(list(rows_source[0].keys()) if rows_source else None)
    timestamp_key = _require_any_field(mapping, ["timestamp [ns]", "timestamp"])
    px_key = _require_field(mapping, "p_RS_R_x [m]")
    py_key = _require_field(mapping, "p_RS_R_y [m]")
    qw_key = _require_field(mapping, "q_RS_w []")
    qx_key = _require_field(mapping, "q_RS_x []")
    qy_key = _require_field(mapping, "q_RS_y []")
    qz_key = _require_field(mapping, "q_RS_z []")

    for row in rows_source:
        timestamp_seconds = float(row[timestamp_key]) * 1e-9
        qw = float(row[qw_key])
        qx = float(row[qx_key])
        qy = float(row[qy_key])
        qz = float(row[qz_key])
        rows.append(
            {
                "timestamp": timestamp_seconds,
                "x": float(row[px_key]),
                "y": float(row[py_key]),
                "yaw": _quaternion_to_yaw(qw, qx, qy, qz),
            }
        )

    if not rows:
        raise ValueError(f"EuRoC ground-truth file contained no samples: {source_name}")
    return rows


def load_euroc_imu(sequence_root: str | Path) -> list[dict[str, float]]:
    directory_root, zip_path = _resolve_sequence_inputs(sequence_root)
    rows_source: list[dict[str, str]]
    source_name: str
    if zip_path is not None:
        source_name = f"{zip_path}!mav0/imu0/data.csv"
        rows_source = _read_rows_from_zip(zip_path, "mav0/imu0/data.csv")
    else:
        imu_path = directory_root / "mav0" / "imu0" / "data.csv"
        if not imu_path.exists():
            raise FileNotFoundError(imu_path)
        source_name = str(imu_path)
        rows_source = _load_rows_from_csv(imu_path)

    rows: list[dict[str, float]] = []
    mapping = _normalize_fieldnames(list(rows_source[0].keys()) if rows_source else None)
    timestamp_key = _require_any_field(mapping, ["timestamp [ns]", "timestamp"])
    wz_key = _require_field(mapping, "w_RS_S_z [rad s^-1]")

    for row in rows_source:
        rows.append(
            {
                "timestamp": float(row[timestamp_key]) * 1e-9,
                "omega_z": float(row[wz_key]),
            }
        )

    if not rows:
        raise ValueError(f"EuRoC IMU file contained no samples: {source_name}")
    return rows


def _nearest_imu_omega_z(timestamp: float, imu_rows: list[dict[str, float]], start_index: int) -> tuple[float, int]:
    best_index = start_index
    while best_index + 1 < len(imu_rows):
        current_gap = abs(imu_rows[best_index]["timestamp"] - timestamp)
        next_gap = abs(imu_rows[best_index + 1]["timestamp"] - timestamp)
        if next_gap > current_gap:
            break
        best_index += 1
    return imu_rows[best_index]["omega_z"], best_index


def build_planar_proxy_samples(sequence_root: str | Path) -> list[PlanarSample]:
    gt_rows = load_euroc_ground_truth(sequence_root)
    imu_rows = load_euroc_imu(sequence_root)

    samples: list[PlanarSample] = []
    imu_index = 0
    for index, row in enumerate(gt_rows):
        if index + 1 < len(gt_rows):
            next_row = gt_rows[index + 1]
            dt = max(next_row["timestamp"] - row["timestamp"], 1e-9)
            vx = (next_row["x"] - row["x"]) / dt
            vy = (next_row["y"] - row["y"]) / dt
            omega = _wrap_angle(next_row["yaw"] - row["yaw"]) / dt
        elif samples:
            vx = samples[-1].vx
            vy = samples[-1].vy
            omega = samples[-1].omega
        else:
            vx = 0.0
            vy = 0.0
            omega = 0.0

        imu_omega_z, imu_index = _nearest_imu_omega_z(row["timestamp"], imu_rows, imu_index)
        samples.append(
            PlanarSample(
                timestamp=row["timestamp"],
                x=row["x"],
                y=row["y"],
                yaw=row["yaw"],
                vx=vx,
                vy=vy,
                omega=omega,
                imu_omega_z=imu_omega_z,
            )
        )

    return samples


def write_planar_proxy_csv(sequence_root: str | Path, out_path: str | Path) -> Path:
    samples = build_planar_proxy_samples(sequence_root)
    out_csv = Path(out_path)
    out_csv.parent.mkdir(parents=True, exist_ok=True)
    with out_csv.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(["timestamp", "x", "y", "yaw", "vx", "vy", "omega", "imu_omega_z"])
        for sample in samples:
            writer.writerow(
                [
                    f"{sample.timestamp:.9f}",
                    f"{sample.x:.9f}",
                    f"{sample.y:.9f}",
                    f"{sample.yaw:.9f}",
                    f"{sample.vx:.9f}",
                    f"{sample.vy:.9f}",
                    f"{sample.omega:.9f}",
                    f"{sample.imu_omega_z:.9f}",
                ]
            )
    return out_csv
