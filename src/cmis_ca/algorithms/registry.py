"""Registry for selecting algorithms by name."""

from __future__ import annotations

from typing import Any

from cmis_ca.algorithms.orca.algorithm import ORCAAlgorithm


class AlgorithmRegistry:
    """Factory registry used by the CLI and the public API."""

    _builders = {
        "orca": ORCAAlgorithm,
    }

    @classmethod
    def create(cls, name: str, **kwargs: Any):
        try:
            builder = cls._builders[name]
        except KeyError as exc:
            available = ", ".join(sorted(cls._builders))
            raise ValueError(f"unknown algorithm '{name}'. Available: {available}") from exc
        return builder(**kwargs)

    @classmethod
    def list_algorithms(cls) -> list[str]:
        return sorted(cls._builders)


def create_algorithm(name: str, **kwargs: Any):
    return AlgorithmRegistry.create(name, **kwargs)


def list_algorithms() -> list[str]:
    return AlgorithmRegistry.list_algorithms()
