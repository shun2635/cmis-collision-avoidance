"""Scenario and snapshot types shared across algorithms."""

from __future__ import annotations

from dataclasses import dataclass, field

from cmis_ca.core.agent import AgentConfig, AgentProfile
from cmis_ca.core.geometry import Vector2
from cmis_ca.core.state import AgentState


@dataclass(frozen=True)
class ObstacleSegment:
    """Minimal line-segment obstacle used by the initial skeleton."""

    start: Vector2 = field(default_factory=Vector2)
    end: Vector2 = field(default_factory=Vector2)

    def __post_init__(self) -> None:
        if self.start == self.end:
            raise ValueError("obstacle segment must have non-zero length")

    @property
    def direction(self) -> Vector2:
        return (self.end - self.start).normalized()

    @property
    def length(self) -> float:
        return self.start.distance_to(self.end)


@dataclass(frozen=True)
class Scenario:
    """Input scenario shared by all algorithms."""

    agents: tuple[AgentConfig, ...]
    obstacles: tuple[ObstacleSegment, ...] = ()
    time_step: float = 0.1
    steps: int = 1
    name: str = "unnamed"

    def __post_init__(self) -> None:
        if self.time_step <= 0.0:
            raise ValueError("time_step must be positive")
        if self.steps < 0:
            raise ValueError("steps must be non-negative")
        if not self.agents:
            raise ValueError("scenario must contain at least one agent")


@dataclass(frozen=True)
class SnapshotAgent:
    """Read-only view consumed by algorithms during one simulation step."""

    index: int
    name: str
    profile: AgentProfile
    state: AgentState


@dataclass(frozen=True)
class WorldSnapshot:
    """Read-only world state delivered to an algorithm."""

    step_index: int
    time_step: float
    agents: tuple[SnapshotAgent, ...]
    obstacles: tuple[ObstacleSegment, ...] = ()

    def __post_init__(self) -> None:
        if self.time_step <= 0.0:
            raise ValueError("time_step must be positive")
        indices = [agent.index for agent in self.agents]
        if len(indices) != len(set(indices)):
            raise ValueError("snapshot agent indices must be unique")
