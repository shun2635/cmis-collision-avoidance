"""Regression tests for the upstream-derived Blocks scenario."""

from __future__ import annotations

import pytest

from cmis_ca.core.geometry import Vector2
from cmis_ca.regression import build_upstream_blocks_scenario, run_upstream_blocks_regression


def test_upstream_blocks_scenario_matches_expected_setup() -> None:
    scenario = build_upstream_blocks_scenario()

    assert scenario.name == "upstream-blocks"
    assert scenario.time_step == pytest.approx(0.25)
    assert scenario.steps == 64
    assert len(scenario.agents) == 100
    assert len(scenario.obstacles) == 16
    assert scenario.agents[0].initial_position == Vector2(55.0, 55.0)
    assert scenario.agents[0].goal_position == Vector2(-75.0, -75.0)
    assert scenario.agents[0].auto_update_preferred_velocity_from_goal is False
    assert scenario.agents[0].profile.radius == pytest.approx(2.0)
    assert scenario.agents[0].profile.max_speed == pytest.approx(2.0)
    assert scenario.agents[0].profile.neighbor_dist == pytest.approx(15.0)
    assert scenario.agents[0].profile.max_neighbors == 10
    assert scenario.obstacles[0].point == Vector2(-10.0, 40.0)
    assert scenario.obstacles[0].next_index == 1


def test_upstream_blocks_regression_keeps_progress_and_symmetry() -> None:
    metrics = run_upstream_blocks_regression()

    assert metrics.steps_run == 64
    assert metrics.goal_distance_reduction > 10.0
    assert metrics.minimum_pair_distance > 6.0
    assert metrics.centroid_distance < 1.0e-4
    assert metrics.max_speed <= 2.0
    assert metrics.central_agent_count >= 4
