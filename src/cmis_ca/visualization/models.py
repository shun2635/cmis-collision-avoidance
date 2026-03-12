"""Data models shared by visualization backends."""

from __future__ import annotations

from dataclasses import dataclass

from cmis_ca.core.geometry import Vector2


@dataclass(frozen=True)
class ObstaclePrimitive:
    """Renderable obstacle polyline or polygon."""

    points: tuple[Vector2, ...]
    closed: bool


@dataclass(frozen=True)
class VisualizationFrame:
    """One animation frame derived from simulation history."""

    step_index: int
    global_time: float
    positions: tuple[Vector2, ...]


@dataclass(frozen=True)
class VisualizationTrace:
    """Visualization-ready snapshot of one simulation run."""

    scenario_name: str
    algorithm: str
    time_step: float
    agent_names: tuple[str, ...]
    agent_radii: tuple[float, ...]
    initial_positions: tuple[Vector2, ...]
    obstacles: tuple[ObstaclePrimitive, ...]
    frames: tuple[VisualizationFrame, ...]

    @property
    def num_agents(self) -> int:
        return len(self.agent_names)

    @property
    def num_frames(self) -> int:
        return len(self.frames)
