"""Scenario and snapshot types shared across algorithms."""

from __future__ import annotations

from dataclasses import dataclass, field

from cmis_ca.core.agent import AgentConfig, AgentProfile
from cmis_ca.core.geometry import Vector2
from cmis_ca.core.state import AgentState


@dataclass(frozen=True)
class ObstaclePath:
    """Scenario-level obstacle path before flattening into linked vertices."""

    vertices: tuple[Vector2, ...]
    closed: bool = True

    def __post_init__(self) -> None:
        if len(self.vertices) < 2:
            raise ValueError("obstacle path must contain at least two vertices")
        for index in range(len(self.vertices) - 1):
            if self.vertices[index] == self.vertices[index + 1]:
                raise ValueError("obstacle path cannot contain duplicated consecutive vertices")
        if self.closed and self.vertices[0] == self.vertices[-1]:
            raise ValueError("closed obstacle path must not repeat the first vertex at the end")


@dataclass(frozen=True)
class ObstacleVertex:
    """Flattened obstacle vertex with upstream-style topology links."""

    point: Vector2
    obstacle_id: int
    vertex_id: int
    previous_index: int | None
    next_index: int | None
    direction: Vector2
    is_convex: bool

    @property
    def has_previous(self) -> bool:
        return self.previous_index is not None

    @property
    def has_next(self) -> bool:
        return self.next_index is not None


def build_obstacle_topology(paths: tuple[ObstaclePath, ...]) -> tuple[ObstacleVertex, ...]:
    """Flatten obstacle paths into linked vertices compatible with upstream semantics."""

    obstacles: list[ObstacleVertex] = []
    offset = 0
    for obstacle_id, path in enumerate(paths):
        count = len(path.vertices)
        for vertex_offset, point in enumerate(path.vertices):
            previous_index = _previous_index(offset, vertex_offset, count, path.closed)
            next_index = _next_index(offset, vertex_offset, count, path.closed)
            direction = _direction_for_vertex(path.vertices, vertex_offset, path.closed)
            obstacles.append(
                ObstacleVertex(
                    point=point,
                    obstacle_id=obstacle_id,
                    vertex_id=offset + vertex_offset,
                    previous_index=previous_index,
                    next_index=next_index,
                    direction=direction,
                    is_convex=_is_convex_vertex(path.vertices, vertex_offset, path.closed),
                )
            )
        offset += count

    return tuple(obstacles)


def build_obstacle_chain(vertices: tuple[Vector2, ...]) -> tuple[ObstacleVertex, ...]:
    """Build an open obstacle chain."""

    return build_obstacle_topology((ObstaclePath(vertices=vertices, closed=False),))


def build_obstacle_polygon(vertices: tuple[Vector2, ...]) -> tuple[ObstacleVertex, ...]:
    """Build a closed obstacle polygon."""

    return build_obstacle_topology((ObstaclePath(vertices=vertices, closed=True),))


def obstacle_segment(obstacles: tuple[ObstacleVertex, ...], obstacle_index: int) -> tuple[Vector2, Vector2]:
    """Return the geometric segment represented by one obstacle vertex."""

    obstacle = obstacles[obstacle_index]
    if obstacle.next_index is None:
        raise ValueError(f"obstacle vertex {obstacle_index} does not define an outgoing edge")
    return obstacle.point, obstacles[obstacle.next_index].point


def obstacle_edges(obstacles: tuple[ObstacleVertex, ...]) -> tuple[int, ...]:
    """Return the indices of vertices that define outgoing obstacle edges."""

    return tuple(index for index, obstacle in enumerate(obstacles) if obstacle.next_index is not None)


def _previous_index(offset: int, index: int, count: int, closed: bool) -> int | None:
    if index > 0:
        return offset + index - 1
    if closed:
        return offset + count - 1
    return None


def _next_index(offset: int, index: int, count: int, closed: bool) -> int | None:
    if index < count - 1:
        return offset + index + 1
    if closed:
        return offset
    return None


def _direction_for_vertex(vertices: tuple[Vector2, ...], index: int, closed: bool) -> Vector2:
    if index < len(vertices) - 1:
        return (vertices[index + 1] - vertices[index]).normalized()
    if closed:
        return (vertices[0] - vertices[index]).normalized()
    return (vertices[index] - vertices[index - 1]).normalized()


def _is_convex_vertex(vertices: tuple[Vector2, ...], index: int, closed: bool) -> bool:
    if not closed or len(vertices) == 2:
        return True

    previous_point = vertices[index - 1 if index > 0 else len(vertices) - 1]
    point = vertices[index]
    next_point = vertices[index + 1 if index < len(vertices) - 1 else 0]
    return (point - previous_point).det(next_point - point) >= 0.0


@dataclass(frozen=True)
class Scenario:
    """Input scenario shared by all algorithms."""

    agents: tuple[AgentConfig, ...]
    obstacles: tuple[ObstacleVertex, ...] = ()
    time_step: float = 0.1
    steps: int = 1
    stop_when_all_agents_reach_goals: bool = False
    name: str = "unnamed"

    def __post_init__(self) -> None:
        if self.time_step <= 0.0:
            raise ValueError("time_step must be positive")
        if self.steps < 0:
            raise ValueError("steps must be non-negative")
        if not self.agents:
            raise ValueError("scenario must contain at least one agent")
        if self.stop_when_all_agents_reach_goals and any(
            agent.goal_position is None for agent in self.agents
        ):
            raise ValueError(
                "scenario with stop_when_all_agents_reach_goals requires goal_position for every agent"
            )


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
    global_time: float
    time_step: float
    agents: tuple[SnapshotAgent, ...]
    obstacles: tuple[ObstacleVertex, ...] = ()

    def __post_init__(self) -> None:
        if self.global_time < 0.0:
            raise ValueError("global_time must be non-negative")
        if self.time_step <= 0.0:
            raise ValueError("time_step must be positive")
        indices = [agent.index for agent in self.agents]
        if len(indices) != len(set(indices)):
            raise ValueError("snapshot agent indices must be unique")
