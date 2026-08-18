"""Minimal backend interface for synthetic and future dataset runs."""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from pathlib import Path

from aegis_bench.schema import CanonicalRunLayout


@dataclass(slots=True)
class BackendContext:
    backend_name: str
    benchmark_name: str
    run_layout: CanonicalRunLayout
    metadata: dict[str, object] = field(default_factory=dict)


class BenchmarkBackend(ABC):
    """Smallest backend contract needed for shared execution layout."""

    @property
    @abstractmethod
    def name(self) -> str:
        """Stable backend name such as 'synthetic' or 'euroc'."""

    @abstractmethod
    def run(self, context: BackendContext) -> Path:
        """Execute a backend and return the normalized output directory."""
