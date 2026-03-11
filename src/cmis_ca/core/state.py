"""Dynamic state and command types."""

from __future__ import annotations

from dataclasses import dataclass, field

from cmis_ca.core.geometry import Vector2


@dataclass(frozen=True)
class AgentState:
    """Dynamic state of an agent at a simulation step."""

    position: Vector2 = field(default_factory=Vector2)
    velocity: Vector2 = field(default_factory=Vector2)
    preferred_velocity: Vector2 = field(default_factory=Vector2)


@dataclass(frozen=True)
class AgentCommand:
    """Velocity command returned by an algorithm."""

    velocity: Vector2 = field(default_factory=Vector2)


@dataclass(frozen=True)
class SimulationResult:
    """Container for the output of a simulation run."""

    algorithm: str
    final_states: tuple[AgentState, ...]
    history: tuple[tuple[AgentState, ...], ...]
