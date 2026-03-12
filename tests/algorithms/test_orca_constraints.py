"""Tests for ORCA-specific constraint generation."""

from __future__ import annotations

import pytest

from cmis_ca.algorithms.orca.constraints import (
    build_agent_constraints,
    build_obstacle_constraints,
)
from cmis_ca.algorithms.orca.parameters import ORCAParameters
from cmis_ca.core.agent import AgentProfile
from cmis_ca.core.geometry import Vector2
from cmis_ca.core.neighbor_search import AgentNeighbor, NeighborSet, ObstacleNeighbor
from cmis_ca.core.state import AgentState
from cmis_ca.core.world import SnapshotAgent, WorldSnapshot, build_obstacle_chain


def _agent(index: int, position: Vector2, velocity: Vector2 = Vector2()) -> SnapshotAgent:
    return SnapshotAgent(
        index=index,
        name=f"agent_{index}",
        profile=AgentProfile(radius=0.5, max_speed=1.0),
        state=AgentState(
            position=position,
            velocity=velocity,
            preferred_velocity=Vector2(),
        ),
    )


def test_build_agent_constraints_non_collision_case() -> None:
    snapshot = WorldSnapshot(
        step_index=0,
        global_time=0.0,
        time_step=0.1,
        agents=(
            _agent(0, Vector2(0.0, 0.0)),
            _agent(1, Vector2(2.0, 0.0)),
        ),
    )
    neighbors = NeighborSet(agent_neighbors=(AgentNeighbor(index=1, distance=2.0),))

    constraints = build_agent_constraints(snapshot, 0, neighbors, ORCAParameters(time_horizon=5.0))

    assert len(constraints) == 1
    constraint = constraints[0]
    assert constraint.point.x == pytest.approx(0.1)
    assert constraint.point.y == pytest.approx(0.0)
    assert constraint.direction.x == pytest.approx(0.0)
    assert constraint.direction.y == pytest.approx(1.0)
    assert constraint.signed_distance(Vector2(0.0, 0.0)) > 0.0
    assert constraint.signed_distance(Vector2(0.2, 0.0)) < 0.0


def test_build_agent_constraints_collision_case() -> None:
    snapshot = WorldSnapshot(
        step_index=0,
        global_time=0.0,
        time_step=0.1,
        agents=(
            _agent(0, Vector2(0.0, 0.0)),
            _agent(1, Vector2(0.8, 0.0)),
        ),
    )
    neighbors = NeighborSet(agent_neighbors=(AgentNeighbor(index=1, distance=0.8),))

    constraints = build_agent_constraints(snapshot, 0, neighbors, ORCAParameters(time_horizon=5.0))

    assert len(constraints) == 1
    constraint = constraints[0]
    assert constraint.point.x == pytest.approx(-1.0)
    assert constraint.point.y == pytest.approx(0.0)
    assert constraint.direction.x == pytest.approx(0.0)
    assert constraint.direction.y == pytest.approx(1.0)
    assert constraint.signed_distance(Vector2(-1.5, 0.0)) > 0.0


def test_build_obstacle_constraints_non_collision_case() -> None:
    snapshot = WorldSnapshot(
        step_index=0,
        global_time=0.0,
        time_step=0.1,
        agents=(_agent(0, Vector2(0.0, 0.0)),),
        obstacles=build_obstacle_chain((Vector2(2.0, -1.0), Vector2(2.0, 1.0))),
    )
    neighbors = NeighborSet(obstacle_neighbors=(ObstacleNeighbor(index=0, distance=2.0),))

    constraints = build_obstacle_constraints(
        snapshot,
        0,
        neighbors,
        ORCAParameters(time_horizon_obst=5.0),
    )

    assert len(constraints) == 1
    constraint = constraints[0]
    assert constraint.point.x == pytest.approx(0.3)
    assert constraint.point.y == pytest.approx(0.0)
    assert constraint.direction.x == pytest.approx(0.0)
    assert constraint.direction.y == pytest.approx(1.0)
    assert constraint.signed_distance(Vector2(0.0, 0.0)) > 0.0
    assert constraint.signed_distance(Vector2(0.4, 0.0)) < 0.0


def test_build_obstacle_constraints_collision_case() -> None:
    snapshot = WorldSnapshot(
        step_index=0,
        global_time=0.0,
        time_step=0.1,
        agents=(_agent(0, Vector2(0.0, 0.0)),),
        obstacles=build_obstacle_chain((Vector2(0.2, -1.0), Vector2(0.2, 1.0))),
    )
    neighbors = NeighborSet(obstacle_neighbors=(ObstacleNeighbor(index=0, distance=0.2),))

    constraints = build_obstacle_constraints(
        snapshot,
        0,
        neighbors,
        ORCAParameters(time_horizon_obst=5.0),
    )

    assert len(constraints) == 1
    constraint = constraints[0]
    assert constraint.point.x == pytest.approx(-3.0)
    assert constraint.point.y == pytest.approx(0.0)
    assert constraint.direction.x == pytest.approx(0.0)
    assert constraint.direction.y == pytest.approx(1.0)
    assert constraint.signed_distance(Vector2(-3.5, 0.0)) > 0.0
