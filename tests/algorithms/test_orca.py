"""Integration-oriented tests for the ORCA algorithm step."""

from __future__ import annotations

import pytest

from cmis_ca.algorithms.orca.algorithm import ORCAAlgorithm
from cmis_ca.algorithms.orca.agent_solver import compute_orca_velocity
from cmis_ca.algorithms.orca.parameters import ORCAParameters
from cmis_ca.core.neighbor_search import NaiveNeighborSearch
from cmis_ca.core.agent import AgentConfig, AgentProfile
from cmis_ca.core.geometry import Vector2
from cmis_ca.core.simulation import Simulator
from cmis_ca.core.world import Scenario, build_obstacle_polygon


def test_orca_step_keeps_single_agent_smoke_case() -> None:
    scenario = Scenario(
        name="single-agent",
        time_step=0.5,
        steps=1,
        agents=(
            AgentConfig(
                name="agent_0",
                profile=AgentProfile(radius=0.4, max_speed=1.0),
                initial_position=Vector2(0.0, 0.0),
                initial_velocity=Vector2(0.0, 0.0),
                preferred_velocity=Vector2(1.0, 0.0),
            ),
        ),
    )

    simulator = Simulator(scenario=scenario, algorithm=ORCAAlgorithm())
    commands = simulator._algorithm.step(simulator.snapshot())

    assert len(commands) == 1
    assert commands[0].velocity.x == pytest.approx(1.0)
    assert commands[0].velocity.y == pytest.approx(0.0)


def test_orca_step_slows_two_head_on_agents() -> None:
    scenario = Scenario(
        name="head-on",
        time_step=0.5,
        steps=1,
        agents=(
            AgentConfig(
                name="left",
                profile=AgentProfile(radius=0.5, max_speed=1.0),
                initial_position=Vector2(-1.0, 0.0),
                initial_velocity=Vector2(0.0, 0.0),
                preferred_velocity=Vector2(1.0, 0.0),
            ),
            AgentConfig(
                name="right",
                profile=AgentProfile(radius=0.5, max_speed=1.0),
                initial_position=Vector2(1.0, 0.0),
                initial_velocity=Vector2(0.0, 0.0),
                preferred_velocity=Vector2(-1.0, 0.0),
            ),
        ),
    )

    simulator = Simulator(scenario=scenario, algorithm=ORCAAlgorithm())
    result = simulator.run()
    left_state, right_state = result.final_states

    assert left_state.velocity.x == pytest.approx(0.1)
    assert right_state.velocity.x == pytest.approx(-0.1)
    assert left_state.position.x == pytest.approx(-0.95)
    assert right_state.position.x == pytest.approx(0.95)
    assert right_state.position.x - left_state.position.x > 1.0


def test_orca_step_uses_obstacle_constraints() -> None:
    scenario = Scenario(
        name="wall",
        time_step=0.5,
        steps=1,
        agents=(
            AgentConfig(
                name="agent_0",
                profile=AgentProfile(radius=0.5, max_speed=1.0),
                initial_position=Vector2(0.0, 0.0),
                initial_velocity=Vector2(0.0, 0.0),
                preferred_velocity=Vector2(1.0, 0.0),
            ),
        ),
        obstacles=build_obstacle_polygon(
            (
                Vector2(0.2, -1.0),
                Vector2(0.2, 1.0),
                Vector2(0.6, 1.0),
                Vector2(0.6, -1.0),
            )
        ),
    )

    simulator = Simulator(scenario=scenario, algorithm=ORCAAlgorithm())
    commands = simulator._algorithm.step(simulator.snapshot())

    assert len(commands) == 1
    assert commands[0].velocity.x == pytest.approx(0.02)
    assert commands[0].velocity.y == pytest.approx(0.0)


def test_compute_orca_velocity_matches_step_for_preferred_velocity() -> None:
    scenario = Scenario(
        name="head-on",
        time_step=0.5,
        steps=1,
        agents=(
            AgentConfig(
                name="left",
                profile=AgentProfile(radius=0.5, max_speed=1.0),
                initial_position=Vector2(-1.0, 0.0),
                initial_velocity=Vector2(0.0, 0.0),
                preferred_velocity=Vector2(1.0, 0.0),
            ),
            AgentConfig(
                name="right",
                profile=AgentProfile(radius=0.5, max_speed=1.0),
                initial_position=Vector2(1.0, 0.0),
                initial_velocity=Vector2(0.0, 0.0),
                preferred_velocity=Vector2(-1.0, 0.0),
            ),
        ),
    )

    simulator = Simulator(scenario=scenario, algorithm=ORCAAlgorithm())
    snapshot = simulator.snapshot()
    commands = simulator._algorithm.step(snapshot)

    left_velocity = compute_orca_velocity(
        snapshot=snapshot,
        agent_index=0,
        optimization_velocity=Vector2(1.0, 0.0),
        parameters=ORCAParameters(),
        neighbor_search=NaiveNeighborSearch(),
    )
    right_velocity = compute_orca_velocity(
        snapshot=snapshot,
        agent_index=1,
        optimization_velocity=Vector2(-1.0, 0.0),
        parameters=ORCAParameters(),
        neighbor_search=NaiveNeighborSearch(),
    )

    assert commands[0].velocity == left_velocity
    assert commands[1].velocity == right_velocity


def test_compute_orca_velocity_uses_arbitrary_optimization_velocity() -> None:
    scenario = Scenario(
        name="single-agent",
        time_step=0.5,
        steps=1,
        agents=(
            AgentConfig(
                name="agent_0",
                profile=AgentProfile(radius=0.4, max_speed=1.0),
                initial_position=Vector2(0.0, 0.0),
                initial_velocity=Vector2(0.0, 0.0),
                preferred_velocity=Vector2(1.0, 0.0),
            ),
        ),
    )

    simulator = Simulator(scenario=scenario, algorithm=ORCAAlgorithm())
    snapshot = simulator.snapshot()

    velocity = compute_orca_velocity(
        snapshot=snapshot,
        agent_index=0,
        optimization_velocity=Vector2(0.0, 1.0),
        parameters=ORCAParameters(),
        neighbor_search=NaiveNeighborSearch(),
    )

    assert velocity.x == pytest.approx(0.0)
    assert velocity.y == pytest.approx(1.0)
