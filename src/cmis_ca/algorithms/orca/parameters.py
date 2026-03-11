"""ORCA-specific parameters."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class ORCAParameters:
    """Initial ORCA parameter set mirrored from the design document."""

    neighbor_dist: float = 5.0
    max_neighbors: int = 10
    time_horizon: float = 5.0
    time_horizon_obst: float = 5.0
