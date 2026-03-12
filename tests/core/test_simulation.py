"""Tests for simulator goal-driven preferred velocity updates."""

from __future__ import annotations

import pytest

from cmis_ca.algorithms.registry import create_algorithm
from cmis_ca.core.agent import AgentConfig, AgentProfile
from cmis_ca.core.geometry import Vector2
from cmis_ca.core.simulation import Simulator
from cmis_ca.core.world import Scenario


def test_simulator_updates_preferred_velocity_from_goal_before_step() -> None:
    scenario = Scenario(
        agents=(
            AgentConfig(
                profile=AgentProfile(radius=0.4, max_speed=2.0),
                initial_position=Vector2(0.0, 0.0),
                initial_velocity=Vector2(0.0, 0.0),
                preferred_velocity=Vector2(),
                goal_position=Vector2(10.0, 0.0),
                preferred_speed=1.5,
            ),
        ),
        time_step=0.5,
        steps=1,
    )
    simulator = Simulator(scenario=scenario, algorithm=create_algorithm("orca"))

    simulator.step()

    state = simulator.states[0]
    assert state.preferred_velocity.x == pytest.approx(1.5)
    assert state.preferred_velocity.y == pytest.approx(0.0)
    assert state.velocity.x == pytest.approx(1.5)
    assert state.position.x == pytest.approx(0.75)
    assert simulator.global_time == pytest.approx(0.5)


def test_simulator_caps_goal_preferred_velocity_when_close_to_goal() -> None:
    scenario = Scenario(
        agents=(
            AgentConfig(
                profile=AgentProfile(radius=0.4, max_speed=2.0),
                initial_position=Vector2(0.0, 0.0),
                goal_position=Vector2(0.3, 0.4),
                preferred_speed=1.0,
            ),
        ),
        time_step=1.0,
        steps=1,
    )
    simulator = Simulator(scenario=scenario, algorithm=create_algorithm("orca"))

    simulator.refresh_preferred_velocities_from_goals()

    state = simulator.states[0]
    assert state.preferred_velocity.x == pytest.approx(0.3)
    assert state.preferred_velocity.y == pytest.approx(0.4)


def test_agent_config_rejects_negative_preferred_speed() -> None:
    with pytest.raises(ValueError):
        AgentConfig(preferred_speed=-0.1)


def test_simulator_snapshot_exposes_global_time() -> None:
    scenario = Scenario(
        agents=(AgentConfig(profile=AgentProfile()),),
        time_step=0.25,
        steps=2,
    )
    simulator = Simulator(scenario=scenario, algorithm=create_algorithm("orca"))

    assert simulator.snapshot().global_time == pytest.approx(0.0)
    simulator.step()
    assert simulator.snapshot().global_time == pytest.approx(0.25)
