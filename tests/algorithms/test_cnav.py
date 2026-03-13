"""Tests for the initial CNav scaffold."""

from __future__ import annotations

import pytest

from cmis_ca.algorithms.cnav.actions import build_default_action_set
from cmis_ca.algorithms.cnav.algorithm import CNavAlgorithm
from cmis_ca.algorithms.cnav.parameters import CNavParameters
from cmis_ca.core.agent import AgentConfig, AgentProfile
from cmis_ca.core.geometry import Vector2
from cmis_ca.core.simulation import Simulator
from cmis_ca.core.world import Scenario


def test_build_default_action_set_uses_paper_order() -> None:
    actions = build_default_action_set(
        Vector2(1.0, 0.0),
        action_speed=1.5,
        beta_degrees=45.0,
    )

    assert len(actions) == 8
    assert actions[0].x == pytest.approx(1.5)
    assert actions[0].y == pytest.approx(0.0)
    assert actions[1].x == pytest.approx(1.5 / 2**0.5)
    assert actions[1].y == pytest.approx(1.5 / 2**0.5)
    assert actions[2].x == pytest.approx(1.5 / 2**0.5)
    assert actions[2].y == pytest.approx(-(1.5 / 2**0.5))
    assert actions[3].x == pytest.approx(0.0, abs=1e-9)
    assert actions[3].y == pytest.approx(1.5)
    assert actions[5].x == pytest.approx(-1.5)
    assert actions[5].y == pytest.approx(0.0, abs=1e-9)


def test_cnav_parameters_validate_initial_defaults() -> None:
    parameters = CNavParameters()

    assert parameters.coordination_factor == pytest.approx(0.8)
    assert parameters.simulation_horizon_steps == 2
    assert parameters.action_update_interval == pytest.approx(0.2)
    assert parameters.action_speed == pytest.approx(1.5)
    assert parameters.beta_degrees == pytest.approx(45.0)


def test_cnav_algorithm_caches_goal_directed_intended_velocity_until_update_interval() -> None:
    scenario = Scenario(
        name="single-agent",
        time_step=0.1,
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

    algorithm = CNavAlgorithm(parameters=CNavParameters(action_update_interval=0.2))
    simulator = Simulator(scenario=scenario, algorithm=algorithm)

    first_commands = algorithm.step(simulator.snapshot())
    first_intended_velocity = algorithm._intent_cache[0].intended_velocity

    algorithm._intent_cache[0].intended_velocity = Vector2(-1.5, 0.0)
    second_commands = algorithm.step(simulator.snapshot())

    assert first_intended_velocity == Vector2(1.5, 0.0)
    assert algorithm._intent_cache[0].intended_velocity == Vector2(-1.5, 0.0)
    assert first_commands[0].velocity.x == pytest.approx(1.0)
    assert second_commands[0].velocity.x == pytest.approx(-1.0)


def test_cnav_algorithm_refreshes_intended_velocity_after_update_interval() -> None:
    scenario = Scenario(
        name="single-agent",
        time_step=0.1,
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

    algorithm = CNavAlgorithm(parameters=CNavParameters(action_update_interval=0.2))
    simulator = Simulator(scenario=scenario, algorithm=algorithm)

    algorithm.step(simulator.snapshot())
    algorithm._intent_cache[0].intended_velocity = Vector2(-1.5, 0.0)
    simulator.step()
    simulator.step()
    simulator.step()

    assert algorithm._intent_cache[0].intended_velocity == Vector2(1.5, 0.0)
