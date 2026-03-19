"""Tests for built-in demo scenarios."""

from __future__ import annotations

import pytest

from cmis_ca.algorithms.registry import create_algorithm
from cmis_ca.cli.run import build_demo_scenario
from cmis_ca.core.simulation import Simulator


def test_build_demo_scenario_is_circle_setup() -> None:
    scenario = build_demo_scenario(steps=3)

    assert scenario.name == "circle-demo"
    assert scenario.steps == 3
    assert len(scenario.agents) == 8
    assert scenario.agents[0].initial_position.x == pytest.approx(8.0)
    assert scenario.agents[0].goal_position is not None
    assert scenario.agents[0].goal_position.x == pytest.approx(-8.0)
    assert scenario.agents[0].preferred_speed == pytest.approx(1.0)


def test_build_demo_scenario_uses_longer_default_steps() -> None:
    scenario = build_demo_scenario()

    assert scenario.steps == 1000


def test_build_demo_scenario_breaks_symmetry_and_crosses_center() -> None:
    scenario = build_demo_scenario(steps=48)
    simulator = Simulator(scenario=scenario, algorithm=create_algorithm("orca"))

    result = simulator.run()

    initial_average_radius = sum(
        state.position.norm() for state in result.history[0]
    ) / len(result.history[0])
    final_average_radius = sum(
        state.position.norm() for state in result.final_states
    ) / len(result.final_states)

    assert final_average_radius > initial_average_radius - 1.0
