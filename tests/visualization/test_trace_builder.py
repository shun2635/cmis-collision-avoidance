"""Tests for visualization trace conversion."""

from __future__ import annotations

import pytest

from cmis_ca.core.agent import AgentConfig, AgentProfile
from cmis_ca.core.geometry import Vector2
from cmis_ca.core.state import AgentState, SimulationResult
from cmis_ca.core.world import ObstaclePath, Scenario, build_obstacle_topology
from cmis_ca.visualization.trace_builder import build_visualization_trace


def test_build_visualization_trace_preserves_history_and_obstacles() -> None:
    scenario = Scenario(
        name="visualization-demo",
        time_step=0.5,
        steps=1,
        agents=(
            AgentConfig(
                name="left",
                profile=AgentProfile(radius=0.5, max_speed=1.0),
                initial_position=Vector2(-1.0, 0.0),
            ),
            AgentConfig(
                name="right",
                profile=AgentProfile(radius=0.6, max_speed=1.0),
                initial_position=Vector2(1.0, 0.0),
            ),
        ),
        obstacles=build_obstacle_topology(
            (
                ObstaclePath((Vector2(0.0, 0.0), Vector2(0.0, 2.0)), closed=False),
                ObstaclePath(
                    (
                        Vector2(2.0, 0.0),
                        Vector2(3.0, 0.0),
                        Vector2(3.0, 1.0),
                    ),
                    closed=True,
                ),
            )
        ),
    )
    result = SimulationResult(
        algorithm="orca",
        final_states=(
            AgentState(position=Vector2(-0.5, 0.0)),
            AgentState(position=Vector2(0.5, 0.0)),
        ),
        history=(
            (
                AgentState(position=Vector2(-1.0, 0.0)),
                AgentState(position=Vector2(1.0, 0.0)),
            ),
            (
                AgentState(position=Vector2(-0.5, 0.0)),
                AgentState(position=Vector2(0.5, 0.0)),
            ),
        ),
    )

    trace = build_visualization_trace(scenario, result)

    assert trace.scenario_name == "visualization-demo"
    assert trace.algorithm == "orca"
    assert trace.agent_names == ("left", "right")
    assert trace.agent_radii == (0.5, 0.6)
    assert trace.num_frames == 2
    assert trace.frames[1].global_time == pytest.approx(0.5)
    assert trace.frames[1].positions[0] == Vector2(-0.5, 0.0)
    assert len(trace.obstacles) == 2
    assert trace.obstacles[0].closed is False
    assert trace.obstacles[1].closed is True
    assert trace.obstacles[1].points[0] == Vector2(2.0, 0.0)


def test_build_visualization_trace_rejects_mismatched_history() -> None:
    scenario = Scenario(
        agents=(AgentConfig(profile=AgentProfile(), initial_position=Vector2()),),
    )
    result = SimulationResult(
        algorithm="orca",
        final_states=(AgentState(position=Vector2()),),
        history=((AgentState(position=Vector2()), AgentState(position=Vector2(1.0, 0.0))),),
    )

    with pytest.raises(ValueError):
        build_visualization_trace(scenario, result)
