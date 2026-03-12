"""ORCA-specific parameters."""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from cmis_ca.core.agent import AgentProfile


@dataclass(frozen=True)
class ORCAParameters:
    """Optional ORCA overrides layered on top of per-agent profile defaults."""

    neighbor_dist: float | None = None
    max_neighbors: int | None = None
    time_horizon: float | None = None
    time_horizon_obst: float | None = None

    def __post_init__(self) -> None:
        if self.neighbor_dist is not None and self.neighbor_dist < 0.0:
            raise ValueError("neighbor_dist must be non-negative")
        if self.max_neighbors is not None and self.max_neighbors < 0:
            raise ValueError("max_neighbors must be non-negative")
        if self.time_horizon is not None and self.time_horizon <= 0.0:
            raise ValueError("time_horizon must be positive")
        if self.time_horizon_obst is not None and self.time_horizon_obst <= 0.0:
            raise ValueError("time_horizon_obst must be positive")

    def resolve(self, profile: "AgentProfile") -> "ORCAParameters":
        return ORCAParameters(
            neighbor_dist=profile.neighbor_dist if self.neighbor_dist is None else self.neighbor_dist,
            max_neighbors=profile.max_neighbors if self.max_neighbors is None else self.max_neighbors,
            time_horizon=profile.time_horizon if self.time_horizon is None else self.time_horizon,
            time_horizon_obst=(
                profile.time_horizon_obst
                if self.time_horizon_obst is None
                else self.time_horizon_obst
            ),
        )
