"""Tests for shared world and constraint types."""

from __future__ import annotations

import pytest

from cmis_ca.core.agent import AgentConfig, AgentProfile
from cmis_ca.core.constraints import LineConstraint
from cmis_ca.core.geometry import Vector2
from cmis_ca.core.state import AgentState
from cmis_ca.core.world import (
    NavigationGrid,
    ObstaclePath,
    Scenario,
    SnapshotAgent,
    WorldSnapshot,
    build_obstacle_chain,
    build_obstacle_polygon,
    obstacle_segment,
)


def test_scenario_requires_agents_and_positive_time_step() -> None:
    with pytest.raises(ValueError):
        Scenario(agents=(), time_step=0.1)

    with pytest.raises(ValueError):
        Scenario(
            agents=(AgentConfig(profile=AgentProfile()),),
            time_step=0.0,
        )


def test_scenario_rejects_goal_stop_without_agent_goals() -> None:
    with pytest.raises(ValueError, match="goal_position"):
        Scenario(
            agents=(AgentConfig(profile=AgentProfile()),),
            stop_when_all_agents_reach_goals=True,
        )


def test_obstacle_chain_builds_linked_vertices() -> None:
    obstacles = build_obstacle_chain((Vector2(0.0, 0.0), Vector2(0.0, 2.0)))

    assert len(obstacles) == 2
    assert obstacles[0].next_index == 1
    assert obstacles[1].next_index is None
    assert obstacles[0].direction == Vector2(0.0, 1.0)
    assert obstacle_segment(obstacles, 0) == (Vector2(0.0, 0.0), Vector2(0.0, 2.0))


def test_closed_obstacle_polygon_marks_convex_vertices() -> None:
    obstacles = build_obstacle_polygon(
        (
            Vector2(0.0, 0.0),
            Vector2(1.0, 0.0),
            Vector2(1.0, 1.0),
            Vector2(0.0, 1.0),
        )
    )

    assert len(obstacles) == 4
    assert all(obstacle.is_convex for obstacle in obstacles)
    assert obstacles[0].previous_index == 3
    assert obstacles[3].next_index == 0


def test_invalid_obstacle_path_raises() -> None:
    with pytest.raises(ValueError):
        ObstaclePath(vertices=(Vector2(1.0, 1.0),), closed=False)

    with pytest.raises(ValueError):
        ObstaclePath(vertices=(Vector2(1.0, 1.0), Vector2(1.0, 1.0)), closed=False)


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


def test_navigation_grid_requires_rectangular_binary_passability() -> None:
    grid = NavigationGrid(
        cell_size=1.0,
        passability=((1, 1), (1, 0)),
    )

    assert grid.passability[1][1] == 0

    with pytest.raises(ValueError):
        NavigationGrid(cell_size=1.0, passability=((1, 1), (1,)))
