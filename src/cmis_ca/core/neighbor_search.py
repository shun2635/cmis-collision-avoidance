"""Neighbor search interfaces and a minimal reference implementation."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol

from cmis_ca.core.world import WorldSnapshot


@dataclass(frozen=True)
class NeighborSet:
    """Indices of nearby agents and obstacles for one agent."""

    agent_indices: tuple[int, ...] = ()
    obstacle_indices: tuple[int, ...] = ()


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
    """O(n^2) reference implementation good enough for the initial skeleton."""

    def find_neighbors(
        self,
        snapshot: WorldSnapshot,
        agent_index: int,
        neighbor_dist: float,
        max_neighbors: int,
    ) -> NeighborSet:
        origin = snapshot.agents[agent_index].state.position
        candidates = []

        for other in snapshot.agents:
            if other.index == agent_index:
                continue

            distance = (other.state.position - origin).norm()
            if distance <= neighbor_dist:
                candidates.append((distance, other.index))

        candidates.sort(key=lambda item: item[0])
        return NeighborSet(
            agent_indices=tuple(index for _, index in candidates[:max_neighbors]),
            obstacle_indices=tuple(range(len(snapshot.obstacles))),
        )
