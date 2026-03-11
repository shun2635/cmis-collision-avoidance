"""Unit tests for reference neighbor search."""

from __future__ import annotations

import unittest

from cmis_ca.core.agent import AgentProfile
from cmis_ca.core.geometry import Vector2
from cmis_ca.core.neighbor_search import NaiveNeighborSearch
from cmis_ca.core.state import AgentState
from cmis_ca.core.world import ObstacleSegment, SnapshotAgent, WorldSnapshot


def _snapshot_agent(index: int, position: Vector2) -> SnapshotAgent:
    return SnapshotAgent(
        index=index,
        name=f"agent_{index}",
        profile=AgentProfile(radius=0.5, max_speed=1.0),
        state=AgentState(
            position=position,
            velocity=Vector2(),
            preferred_velocity=Vector2(),
        ),
    )


class NaiveNeighborSearchTest(unittest.TestCase):
    def test_agent_neighbors_are_distance_sorted_and_capped(self) -> None:
        snapshot = WorldSnapshot(
            step_index=0,
            time_step=0.1,
            agents=(
                _snapshot_agent(10, Vector2(0.0, 0.0)),
                _snapshot_agent(20, Vector2(3.0, 0.0)),
                _snapshot_agent(30, Vector2(1.0, 0.0)),
                _snapshot_agent(40, Vector2(2.0, 0.0)),
            ),
        )

        neighbors = NaiveNeighborSearch().find_neighbors(
            snapshot=snapshot,
            agent_index=10,
            neighbor_dist=3.0,
            max_neighbors=2,
        )

        self.assertEqual(neighbors.agent_indices, (30, 40))
        self.assertEqual(tuple(neighbor.distance for neighbor in neighbors.agent_neighbors), (1.0, 2.0))

    def test_agent_index_is_resolved_from_snapshot_ids(self) -> None:
        snapshot = WorldSnapshot(
            step_index=0,
            time_step=0.1,
            agents=(
                _snapshot_agent(5, Vector2(0.0, 0.0)),
                _snapshot_agent(9, Vector2(0.0, 1.0)),
            ),
        )

        neighbors = NaiveNeighborSearch().find_neighbors(
            snapshot=snapshot,
            agent_index=9,
            neighbor_dist=2.0,
            max_neighbors=5,
        )

        self.assertEqual(neighbors.agent_indices, (5,))

    def test_obstacle_neighbors_are_filtered_and_sorted_by_segment_distance(self) -> None:
        snapshot = WorldSnapshot(
            step_index=0,
            time_step=0.1,
            agents=(_snapshot_agent(0, Vector2(0.0, 0.0)),),
            obstacles=(
                ObstacleSegment(start=Vector2(5.0, 0.0), end=Vector2(5.0, 2.0)),
                ObstacleSegment(start=Vector2(1.0, -1.0), end=Vector2(1.0, 1.0)),
                ObstacleSegment(start=Vector2(3.0, -1.0), end=Vector2(3.0, 1.0)),
            ),
        )

        neighbors = NaiveNeighborSearch().find_neighbors(
            snapshot=snapshot,
            agent_index=0,
            neighbor_dist=3.0,
            max_neighbors=5,
        )

        self.assertEqual(neighbors.obstacle_indices, (1, 2))
        self.assertEqual(tuple(neighbor.distance for neighbor in neighbors.obstacle_neighbors), (1.0, 3.0))

    def test_zero_max_neighbors_returns_no_agent_neighbors(self) -> None:
        snapshot = WorldSnapshot(
            step_index=0,
            time_step=0.1,
            agents=(
                _snapshot_agent(0, Vector2(0.0, 0.0)),
                _snapshot_agent(1, Vector2(1.0, 0.0)),
            ),
        )

        neighbors = NaiveNeighborSearch().find_neighbors(
            snapshot=snapshot,
            agent_index=0,
            neighbor_dist=2.0,
            max_neighbors=0,
        )

        self.assertEqual(neighbors.agent_neighbors, ())

    def test_invalid_arguments_raise(self) -> None:
        snapshot = WorldSnapshot(
            step_index=0,
            time_step=0.1,
            agents=(_snapshot_agent(0, Vector2(0.0, 0.0)),),
        )
        search = NaiveNeighborSearch()

        with self.assertRaises(ValueError):
            search.find_neighbors(snapshot=snapshot, agent_index=0, neighbor_dist=-1.0, max_neighbors=1)

        with self.assertRaises(ValueError):
            search.find_neighbors(snapshot=snapshot, agent_index=0, neighbor_dist=1.0, max_neighbors=-1)

        with self.assertRaises(ValueError):
            search.find_neighbors(snapshot=snapshot, agent_index=999, neighbor_dist=1.0, max_neighbors=1)


if __name__ == "__main__":
    unittest.main()
