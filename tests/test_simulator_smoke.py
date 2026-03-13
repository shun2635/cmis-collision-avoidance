"""Smoke test for the Python skeleton."""

from __future__ import annotations

import pytest

from cmis_ca.algorithms.registry import create_algorithm
from cmis_ca.core.agent import AgentConfig, AgentProfile
from cmis_ca.core.geometry import Vector2
from cmis_ca.core.simulation import Simulator
from cmis_ca.core.world import Scenario


def test_orca_skeleton_advances_one_agent() -> None:
    scenario = Scenario(
        name="smoke",
        time_step=0.5,
        steps=1,
        agents=(
            AgentConfig(
                name="agent_0",
                profile=AgentProfile(radius=0.4, max_speed=2.0),
                initial_position=Vector2(0.0, 0.0),
                initial_velocity=Vector2(0.0, 0.0),
                preferred_velocity=Vector2(1.0, 0.0),
            ),
        ),
    )

    simulator = Simulator(scenario=scenario, algorithm=create_algorithm("orca"))
    result = simulator.run()
    state = result.final_states[0]

    assert state.position.x == pytest.approx(0.5)
    assert state.position.y == pytest.approx(0.0)
    assert state.velocity.x == pytest.approx(1.0)
    assert state.velocity.y == pytest.approx(0.0)


def test_cnav_can_be_created_and_advance_goal_directed_smoke_case() -> None:
    scenario = Scenario(
        name="cnav-smoke",
        time_step=0.1,
        steps=1,
        agents=(
            AgentConfig(
                name="agent_0",
                profile=AgentProfile(radius=0.4, max_speed=2.0),
                initial_position=Vector2(0.0, 0.0),
                initial_velocity=Vector2(0.0, 0.0),
                preferred_velocity=Vector2(1.0, 0.0),
                goal_position=Vector2(10.0, 0.0),
            ),
        ),
    )

    simulator = Simulator(scenario=scenario, algorithm=create_algorithm("cnav"))
    result = simulator.run()
    state = result.final_states[0]

    assert result.algorithm == "cnav"
    assert state.position.x > 0.0
    assert state.velocity.x > 0.0
