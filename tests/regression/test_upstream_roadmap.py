"""Regression tests for the upstream-derived Roadmap scenario."""

from __future__ import annotations

import math

import pytest

from cmis_ca.core.geometry import Vector2
from cmis_ca.regression import build_upstream_roadmap_setup, run_upstream_roadmap_regression


def test_upstream_roadmap_setup_matches_expected_configuration() -> None:
    setup = build_upstream_roadmap_setup()

    assert setup.scenario.name == "upstream-roadmap"
    assert setup.scenario.time_step == pytest.approx(0.25)
    assert setup.scenario.steps == 64
    assert len(setup.scenario.agents) == 100
    assert len(setup.scenario.obstacles) == 16
    assert len(setup.roadmap) == 20
    assert setup.goals[:4] == (0, 1, 2, 3)
    assert setup.roadmap[0].position == Vector2(-75.0, -75.0)
    assert setup.roadmap[19].position == Vector2(42.0, 42.0)
    assert 5 in setup.roadmap[4].neighbors
    assert 8 in setup.roadmap[4].neighbors
    assert math.isfinite(setup.roadmap[10].dist_to_goal[0])


def test_upstream_roadmap_regression_keeps_progress_and_clearance() -> None:
    metrics = run_upstream_roadmap_regression(steps=32)

    assert metrics.steps_run == 32
    assert metrics.goal_distance_reduction > 7.0
    assert metrics.minimum_pair_distance > 8.0
    assert metrics.centroid_distance < 1.0
    assert metrics.max_speed <= 2.0
    assert not metrics.reached_goal
