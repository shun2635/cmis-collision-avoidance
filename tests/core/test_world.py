"""Unit tests for shared world and constraint types."""

from __future__ import annotations

import unittest

from cmis_ca.core.agent import AgentConfig, AgentProfile
from cmis_ca.core.constraints import LineConstraint
from cmis_ca.core.geometry import Vector2
from cmis_ca.core.state import AgentState
from cmis_ca.core.world import ObstacleSegment, Scenario, SnapshotAgent, WorldSnapshot


class WorldTypesTest(unittest.TestCase):
    def test_scenario_requires_agents_and_positive_time_step(self) -> None:
        with self.assertRaises(ValueError):
            Scenario(agents=(), time_step=0.1)

        with self.assertRaises(ValueError):
            Scenario(
                agents=(AgentConfig(profile=AgentProfile()),),
                time_step=0.0,
            )

    def test_obstacle_segment_exposes_length_and_direction(self) -> None:
        segment = ObstacleSegment(start=Vector2(0.0, 0.0), end=Vector2(0.0, 2.0))

        self.assertEqual(segment.length, 2.0)
        self.assertEqual(segment.direction, Vector2(0.0, 1.0))

    def test_zero_length_obstacle_segment_raises(self) -> None:
        with self.assertRaises(ValueError):
            ObstacleSegment(start=Vector2(1.0, 1.0), end=Vector2(1.0, 1.0))

    def test_world_snapshot_requires_unique_indices(self) -> None:
        agent_state = AgentState(position=Vector2(), velocity=Vector2(), preferred_velocity=Vector2())
        profile = AgentProfile()
        duplicated_agents = (
            SnapshotAgent(index=0, name="a", profile=profile, state=agent_state),
            SnapshotAgent(index=0, name="b", profile=profile, state=agent_state),
        )

        with self.assertRaises(ValueError):
            WorldSnapshot(step_index=0, time_step=0.1, agents=duplicated_agents)

    def test_line_constraint_normalizes_direction_and_computes_signed_distance(self) -> None:
        constraint = LineConstraint(point=Vector2(0.0, 0.0), direction=Vector2(0.0, 2.0))

        self.assertEqual(constraint.direction, Vector2(0.0, 1.0))
        self.assertEqual(constraint.normal, Vector2(-1.0, 0.0))
        self.assertEqual(constraint.signed_distance(Vector2(-3.0, 0.0)), 3.0)

    def test_line_constraint_rejects_zero_direction(self) -> None:
        with self.assertRaises(ValueError):
            LineConstraint(direction=Vector2())


if __name__ == "__main__":
    unittest.main()
