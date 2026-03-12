"""Tests for shared world and constraint types."""

from __future__ import annotations

import pytest

from cmis_ca.core.agent import AgentConfig, AgentProfile
from cmis_ca.core.constraints import LineConstraint
from cmis_ca.core.geometry import Vector2
from cmis_ca.core.state import AgentState
from cmis_ca.core.world import ObstacleSegment, Scenario, SnapshotAgent, WorldSnapshot


def test_scenario_requires_agents_and_positive_time_step() -> None:
    with pytest.raises(ValueError):
        Scenario(agents=(), time_step=0.1)

    with pytest.raises(ValueError):
        Scenario(
            agents=(AgentConfig(profile=AgentProfile()),),
            time_step=0.0,
        )


def test_obstacle_segment_exposes_length_and_direction() -> None:
    segment = ObstacleSegment(start=Vector2(0.0, 0.0), end=Vector2(0.0, 2.0))

    assert segment.length == 2.0
    assert segment.direction == Vector2(0.0, 1.0)


def test_zero_length_obstacle_segment_raises() -> None:
    with pytest.raises(ValueError):
        ObstacleSegment(start=Vector2(1.0, 1.0), end=Vector2(1.0, 1.0))


def test_world_snapshot_requires_unique_indices() -> None:
    agent_state = AgentState(position=Vector2(), velocity=Vector2(), preferred_velocity=Vector2())
    profile = AgentProfile()
    duplicated_agents = (
        SnapshotAgent(index=0, name="a", profile=profile, state=agent_state),
        SnapshotAgent(index=0, name="b", profile=profile, state=agent_state),
    )

    with pytest.raises(ValueError):
        WorldSnapshot(step_index=0, global_time=0.0, time_step=0.1, agents=duplicated_agents)


def test_line_constraint_normalizes_direction_and_computes_signed_distance() -> None:
    constraint = LineConstraint(point=Vector2(0.0, 0.0), direction=Vector2(0.0, 2.0))

    assert constraint.direction == Vector2(0.0, 1.0)
    assert constraint.normal == Vector2(-1.0, 0.0)
    assert constraint.signed_distance(Vector2(-3.0, 0.0)) == 3.0


def test_line_constraint_rejects_zero_direction() -> None:
    with pytest.raises(ValueError):
        LineConstraint(direction=Vector2())


def test_world_snapshot_rejects_negative_global_time() -> None:
    agent_state = AgentState(position=Vector2(), velocity=Vector2(), preferred_velocity=Vector2())
    agent = SnapshotAgent(index=0, name="a", profile=AgentProfile(), state=agent_state)

    with pytest.raises(ValueError):
        WorldSnapshot(step_index=0, global_time=-0.1, time_step=0.1, agents=(agent,))
