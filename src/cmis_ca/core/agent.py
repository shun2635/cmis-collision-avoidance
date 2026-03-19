"""Agent configuration shared across algorithms."""

from __future__ import annotations

from dataclasses import dataclass, field

from cmis_ca.core.geometry import Vector2


@dataclass(frozen=True)
class AgentProfile:
    """Physical properties that remain stable during simulation."""

    radius: float = 0.5
    max_speed: float = 1.0
    neighbor_dist: float = 5.0
    max_neighbors: int = 10
    time_horizon: float = 0.3
    time_horizon_obst: float = 0.3

    def __post_init__(self) -> None:
        if self.radius < 0.0:
            raise ValueError("radius must be non-negative")
        if self.max_speed < 0.0:
            raise ValueError("max_speed must be non-negative")
        if self.neighbor_dist < 0.0:
            raise ValueError("neighbor_dist must be non-negative")
        if self.max_neighbors < 0:
            raise ValueError("max_neighbors must be non-negative")
        if self.time_horizon <= 0.0:
            raise ValueError("time_horizon must be positive")
        if self.time_horizon_obst <= 0.0:
            raise ValueError("time_horizon_obst must be positive")


@dataclass(frozen=True)
class AgentConfig:
    """Static configuration used to create an agent in a scenario."""

    name: str = ""
    profile: AgentProfile = field(default_factory=AgentProfile)
    initial_position: Vector2 = field(default_factory=Vector2)
    initial_velocity: Vector2 = field(default_factory=Vector2)
    preferred_velocity: Vector2 = field(default_factory=Vector2)
    goal_position: Vector2 | None = None
    goal_sequence: tuple[Vector2, ...] = ()
    preferred_speed: float = 1.0
    auto_update_preferred_velocity_from_goal: bool = True
    preferred_velocity_perturbation_scale: float = 0.0
    preferred_velocity_perturbation_phase: float = 0.0

    def __post_init__(self) -> None:
        if self.preferred_speed < 0.0:
            raise ValueError("preferred_speed must be non-negative")
        if self.preferred_velocity_perturbation_scale < 0.0:
            raise ValueError("preferred_velocity_perturbation_scale must be non-negative")


# The draft API refers to `Agent`; the initial skeleton keeps `AgentConfig` as
# the concrete type and exposes `Agent` as a compatibility alias.
Agent = AgentConfig
