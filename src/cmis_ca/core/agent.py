"""Agent configuration shared across algorithms."""

from __future__ import annotations

from dataclasses import dataclass, field

from cmis_ca.core.geometry import Vector2


@dataclass(frozen=True)
class AgentProfile:
    """Physical properties that remain stable during simulation."""

    radius: float = 0.5
    max_speed: float = 1.0


@dataclass(frozen=True)
class AgentConfig:
    """Static configuration used to create an agent in a scenario."""

    name: str = ""
    profile: AgentProfile = field(default_factory=AgentProfile)
    initial_position: Vector2 = field(default_factory=Vector2)
    initial_velocity: Vector2 = field(default_factory=Vector2)
    preferred_velocity: Vector2 = field(default_factory=Vector2)


# The draft API refers to `Agent`; the initial skeleton keeps `AgentConfig` as
# the concrete type and exposes `Agent` as a compatibility alias.
Agent = AgentConfig
