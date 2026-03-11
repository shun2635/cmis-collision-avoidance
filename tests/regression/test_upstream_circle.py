"""Regression tests for the upstream-derived Circle scenario."""

from __future__ import annotations

import pytest

from cmis_ca.io import load_scenario
from cmis_ca.regression import run_upstream_circle_regression


def test_upstream_circle_scenario_matches_expected_setup() -> None:
    scenario = load_scenario("scenarios/upstream_circle.yaml")

    assert scenario.name == "upstream-circle"
    assert scenario.time_step == pytest.approx(0.25)
    assert scenario.steps == 20
    assert len(scenario.agents) == 250
    assert scenario.agents[0].initial_position.x == pytest.approx(200.0)
    assert scenario.agents[0].preferred_velocity.x == pytest.approx(-1.0)
    assert scenario.agents[125].initial_position.x == pytest.approx(-200.0)


def test_upstream_circle_regression_keeps_symmetry_and_inward_progress() -> None:
    metrics = run_upstream_circle_regression(steps=8)

    assert metrics.steps_run == 8
    assert metrics.average_radius < 198.5
    assert metrics.minimum_pair_distance > 3.0
    assert metrics.centroid_distance < 1.0e-9
    assert metrics.max_speed <= 2.0
    assert metrics.max_antipodal_error < 1.0e-8
