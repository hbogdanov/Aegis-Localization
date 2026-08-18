"""Canonical run layout for Aegis benchmark outputs."""

from __future__ import annotations

from dataclasses import asdict, dataclass
from pathlib import Path


@dataclass(slots=True)
class CanonicalRunLayout:
    root: Path
    normalized_dir: Path
    metrics_dir: Path
    plots_dir: Path
    logs_dir: Path
    manifest_path: Path
    metadata_path: Path

    def estimator_csv(self, estimator_name: str) -> Path:
        return self.normalized_dir / f"{estimator_name}.csv"

    @property
    def ground_truth_csv(self) -> Path:
        return self.normalized_dir / "ground_truth.csv"

    def to_dict(self) -> dict[str, str]:
        return {key: str(value) for key, value in asdict(self).items()}


def create_run_layout(results_root: str | Path, backend_name: str, benchmark_name: str, run_name: str) -> CanonicalRunLayout:
    root = Path(results_root) / backend_name / benchmark_name / run_name
    normalized_dir = root / "normalized"
    metrics_dir = root / "metrics"
    plots_dir = root / "plots"
    logs_dir = root / "logs"
    for path in (normalized_dir, metrics_dir, plots_dir, logs_dir):
        path.mkdir(parents=True, exist_ok=True)
    return CanonicalRunLayout(
        root=root,
        normalized_dir=normalized_dir,
        metrics_dir=metrics_dir,
        plots_dir=plots_dir,
        logs_dir=logs_dir,
        manifest_path=root / "manifest.json",
        metadata_path=root / "metadata.json",
    )
