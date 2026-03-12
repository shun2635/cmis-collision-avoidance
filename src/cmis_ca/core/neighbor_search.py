"""Neighbor search interfaces and a reference implementation."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Protocol

from cmis_ca.core.geometry import Vector2
from cmis_ca.core.world import SnapshotAgent, WorldSnapshot, obstacle_edges, obstacle_segment


@dataclass(frozen=True)
class AgentNeighbor:
    """Distance-ranked neighboring agent."""

    index: int
    distance: float


@dataclass(frozen=True)
class ObstacleNeighbor:
    """Distance-ranked neighboring obstacle segment."""

    index: int
    distance: float


@dataclass(frozen=True)
class NeighborSet:
    """Nearby agents and obstacles for one agent.

    The core layer exposes only distance-ranked search results. Algorithm-
    specific acceptance rules remain outside this type.
    """

    agent_neighbors: tuple[AgentNeighbor, ...] = ()
    obstacle_neighbors: tuple[ObstacleNeighbor, ...] = ()

    @property
    def agent_indices(self) -> tuple[int, ...]:
        return tuple(neighbor.index for neighbor in self.agent_neighbors)

    @property
    def obstacle_indices(self) -> tuple[int, ...]:
        return tuple(neighbor.index for neighbor in self.obstacle_neighbors)


class NeighborSearch(Protocol):
    """Common neighbor-search interface used by algorithms."""

    def find_neighbors(
        self,
        snapshot: WorldSnapshot,
        agent_index: int,
        neighbor_dist: float,
        max_neighbors: int,
        obstacle_range: float | None = None,
    ) -> NeighborSet:
        ...


class NaiveNeighborSearch:
    """O(n^2) reference implementation for the current codebase."""

    def find_neighbors(
        self,
        snapshot: WorldSnapshot,
        agent_index: int,
        neighbor_dist: float,
        max_neighbors: int,
        obstacle_range: float | None = None,
    ) -> NeighborSet:
        if neighbor_dist < 0.0:
            raise ValueError("neighbor_dist must be non-negative")
        if max_neighbors < 0:
            raise ValueError("max_neighbors must be non-negative")
        if obstacle_range is not None and obstacle_range < 0.0:
            raise ValueError("obstacle_range must be non-negative")

        origin_agent = _find_agent(snapshot, agent_index)
        origin = origin_agent.state.position

        agent_neighbors: list[tuple[float, AgentNeighbor]] = []
        agent_range_sq = neighbor_dist * neighbor_dist
        for other in snapshot.agents:
            if other.index == agent_index:
                continue

            distance_sq = (other.state.position - origin).abs_sq()
            if distance_sq >= agent_range_sq:
                continue

            _insert_agent_neighbor(
                agent_neighbors,
                AgentNeighbor(index=other.index, distance=math.sqrt(distance_sq)),
                distance_sq,
                max_neighbors,
            )
            if max_neighbors > 0 and len(agent_neighbors) == max_neighbors:
                agent_range_sq = agent_neighbors[-1][0]

        obstacle_neighbors: list[tuple[float, ObstacleNeighbor]] = []
        obstacle_limit = neighbor_dist if obstacle_range is None else obstacle_range
        obstacle_range_sq = obstacle_limit * obstacle_limit
        for obstacle_index in obstacle_edges(snapshot.obstacles):
            if not _is_on_right_side_of_obstacle(origin, snapshot.obstacles, obstacle_index):
                continue

            distance_sq = _distance_sq_to_obstacle(origin, snapshot.obstacles, obstacle_index)
            if distance_sq >= obstacle_range_sq:
                continue

            _insert_obstacle_neighbor(
                obstacle_neighbors,
                ObstacleNeighbor(index=obstacle_index, distance=math.sqrt(distance_sq)),
                distance_sq,
            )

        return NeighborSet(
            agent_neighbors=tuple(neighbor for _, neighbor in agent_neighbors),
            obstacle_neighbors=tuple(neighbor for _, neighbor in obstacle_neighbors),
        )


def _find_agent(snapshot: WorldSnapshot, agent_index: int) -> SnapshotAgent:
    for agent in snapshot.agents:
        if agent.index == agent_index:
            return agent
    raise ValueError(f"agent index {agent_index} is not present in the snapshot")


def _insert_agent_neighbor(
    neighbors: list[tuple[float, AgentNeighbor]],
    neighbor: AgentNeighbor,
    distance_sq: float,
    max_neighbors: int,
) -> None:
    if max_neighbors == 0:
        return

    if len(neighbors) < max_neighbors:
        neighbors.append((distance_sq, neighbor))
    else:
        neighbors.append(neighbors[-1])

    index = len(neighbors) - 1
    while index != 0 and distance_sq < neighbors[index - 1][0]:
        neighbors[index] = neighbors[index - 1]
        index -= 1
    neighbors[index] = (distance_sq, neighbor)

    if len(neighbors) > max_neighbors:
        neighbors.pop()


def _insert_obstacle_neighbor(
    neighbors: list[tuple[float, ObstacleNeighbor]],
    neighbor: ObstacleNeighbor,
    distance_sq: float,
) -> None:
    neighbors.append((distance_sq, neighbor))

    index = len(neighbors) - 1
    while index != 0 and distance_sq < neighbors[index - 1][0]:
        neighbors[index] = neighbors[index - 1]
        index -= 1
    neighbors[index] = (distance_sq, neighbor)


def _distance_sq_to_obstacle(
    point: Vector2,
    obstacles,
    obstacle_index: int,
) -> float:
    segment_start, segment_end = obstacle_segment(obstacles, obstacle_index)
    segment = segment_end - segment_start
    segment_length_sq = segment.abs_sq()
    if segment_length_sq == 0.0:
        return (point - segment_start).abs_sq()

    projection = (point - segment_start).dot(segment) / segment_length_sq
    clamped_projection = max(0.0, min(1.0, projection))
    closest_point = segment_start + segment * clamped_projection
    return (point - closest_point).abs_sq()


def _is_on_right_side_of_obstacle(
    point: Vector2,
    obstacles,
    obstacle_index: int,
) -> bool:
    segment_start, segment_end = obstacle_segment(obstacles, obstacle_index)
    return (segment_start - point).det(segment_end - segment_start) < 0.0
