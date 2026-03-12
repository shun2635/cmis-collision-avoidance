"""Tests for shared agent configuration types."""

from __future__ import annotations

import pytest

from cmis_ca.algorithms.orca.parameters import ORCAParameters
from cmis_ca.core.agent import AgentConfig, AgentProfile
from cmis_ca.core.geometry import Vector2


def test_agent_profile_exposes_upstream_style_navigation_defaults() -> None:
    profile = AgentProfile(
        radius=1.5,
        max_speed=2.0,
        neighbor_dist=15.0,
        max_neighbors=10,
        time_horizon=10.0,
        time_horizon_obst=10.0,
    )

    assert profile.neighbor_dist == pytest.approx(15.0)
    assert profile.max_neighbors == 10
    assert profile.time_horizon == pytest.approx(10.0)
    assert profile.time_horizon_obst == pytest.approx(10.0)


def test_agent_profile_rejects_invalid_navigation_values() -> None:
    with pytest.raises(ValueError):
        AgentProfile(neighbor_dist=-1.0)

    with pytest.raises(ValueError):
        AgentProfile(max_neighbors=-1)

    with pytest.raises(ValueError):
        AgentProfile(time_horizon=0.0)

    with pytest.raises(ValueError):
        AgentProfile(time_horizon_obst=0.0)


def test_orca_parameters_resolve_against_agent_profile() -> None:
    profile = AgentProfile(
        neighbor_dist=15.0,
        max_neighbors=10,
        time_horizon=10.0,
        time_horizon_obst=12.0,
    )

    resolved = ORCAParameters().resolve(profile)
    overridden = ORCAParameters(max_neighbors=4, time_horizon=3.0).resolve(profile)

    assert resolved.neighbor_dist == pytest.approx(15.0)
    assert resolved.max_neighbors == 10
    assert resolved.time_horizon_obst == pytest.approx(12.0)
    assert overridden.neighbor_dist == pytest.approx(15.0)
    assert overridden.max_neighbors == 4
    assert overridden.time_horizon == pytest.approx(3.0)


def test_agent_config_enables_goal_refresh_by_default() -> None:
    config = AgentConfig(goal_position=Vector2(1.0, 0.0))

    assert config.auto_update_preferred_velocity_from_goal is True


def test_agent_config_rejects_negative_goal_perturbation_scale() -> None:
    with pytest.raises(ValueError):
        AgentConfig(preferred_velocity_perturbation_scale=-1.0e-3)
