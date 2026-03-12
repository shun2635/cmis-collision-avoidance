"""Neighbor search interfaces and a reference implementation."""

from __future__ import annotations

from dataclasses import dataclass
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
    ) -> NeighborSet:
        if neighbor_dist < 0.0:
            raise ValueError("neighbor_dist must be non-negative")
        if max_neighbors < 0:
            raise ValueError("max_neighbors must be non-negative")

        origin_agent = _find_agent(snapshot, agent_index)
        origin = origin_agent.state.position

        agent_candidates = []
        for other in snapshot.agents:
            if other.index == agent_index:
                continue

            distance = other.state.position.distance_to(origin)
            if distance <= neighbor_dist:
                agent_candidates.append(AgentNeighbor(index=other.index, distance=distance))

        obstacle_candidates = []
        for obstacle_index in obstacle_edges(snapshot.obstacles):
            distance = _distance_to_obstacle(origin, snapshot.obstacles, obstacle_index)
            if distance <= neighbor_dist:
                obstacle_candidates.append(ObstacleNeighbor(index=obstacle_index, distance=distance))

        agent_candidates.sort(key=lambda neighbor: (neighbor.distance, neighbor.index))
        obstacle_candidates.sort(key=lambda neighbor: (neighbor.distance, neighbor.index))

        return NeighborSet(
            agent_neighbors=tuple(agent_candidates[:max_neighbors]),
            obstacle_neighbors=tuple(obstacle_candidates),
        )


def _find_agent(snapshot: WorldSnapshot, agent_index: int) -> SnapshotAgent:
    for agent in snapshot.agents:
        if agent.index == agent_index:
            return agent
    raise ValueError(f"agent index {agent_index} is not present in the snapshot")


def _distance_to_obstacle(
    point: Vector2,
    obstacles,
    obstacle_index: int,
) -> float:
    segment_start, segment_end = obstacle_segment(obstacles, obstacle_index)
    segment = segment_end - segment_start
    segment_length_sq = segment.abs_sq()
    if segment_length_sq == 0.0:
        return point.distance_to(segment_start)

    projection = (point - segment_start).dot(segment) / segment_length_sq
    clamped_projection = max(0.0, min(1.0, projection))
    closest_point = segment_start + segment * clamped_projection
    return point.distance_to(closest_point)
