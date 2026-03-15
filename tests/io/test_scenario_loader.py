"""Tests for scenario file loading."""

from __future__ import annotations

import json
from pathlib import Path

import pytest

from cmis_ca.io import load_scenario


def test_load_scenario_reads_yaml_with_agents_and_obstacles(tmp_path) -> None:
    scenario_path = tmp_path / "scenario.yaml"
    scenario_path.write_text(
        "\n".join(
            [
                "name: yaml-sample",
                "time_step: 0.25",
                "steps: 3",
                "stop_when_all_agents_reach_goals: true",
                "agents:",
                "  - name: agent_0",
                "    profile:",
                "      radius: 0.4",
                "      max_speed: 1.2",
                "      neighbor_dist: 7.5",
                "      max_neighbors: 12",
                "      time_horizon: 6.0",
                "      time_horizon_obst: 7.0",
                "    initial_position: [0.0, 0.0]",
                "    goal_position: [2.0, 0.0]",
                "    goal_sequence:",
                "      - [2.0, 0.0]",
                "      - [0.0, 2.0]",
                "    auto_update_preferred_velocity_from_goal: false",
                "    preferred_speed: 0.75",
                "    preferred_velocity:",
                "      x: 1.0",
                "      y: 0.0",
                "navigation_grid:",
                "  cell_size: 1.0",
                "  passability:",
                "    - [1, 1]",
                "    - [1, 0]",
                "obstacles:",
                "  - closed: false",
                "    vertices:",
                "      - [1.0, -1.0]",
                "      - [1.0, 1.0]",
            ]
        ),
        encoding="utf-8",
    )

    scenario = load_scenario(scenario_path)

    assert scenario.name == "yaml-sample"
    assert scenario.time_step == pytest.approx(0.25)
    assert scenario.steps == 3
    assert scenario.stop_when_all_agents_reach_goals is True
    assert len(scenario.agents) == 1
    assert scenario.agents[0].profile.max_speed == pytest.approx(1.2)
    assert scenario.agents[0].profile.neighbor_dist == pytest.approx(7.5)
    assert scenario.agents[0].profile.max_neighbors == 12
    assert scenario.agents[0].profile.time_horizon == pytest.approx(6.0)
    assert scenario.agents[0].profile.time_horizon_obst == pytest.approx(7.0)
    assert scenario.agents[0].preferred_velocity.x == pytest.approx(1.0)
    assert scenario.agents[0].goal_position is not None
    assert scenario.agents[0].goal_position.x == pytest.approx(2.0)
    assert len(scenario.agents[0].goal_sequence) == 2
    assert scenario.agents[0].auto_update_preferred_velocity_from_goal is False
    assert scenario.agents[0].preferred_speed == pytest.approx(0.75)
    assert scenario.navigation_grid is not None
    assert scenario.navigation_grid.cell_size == pytest.approx(1.0)
    assert len(scenario.obstacles) == 2
    assert scenario.obstacles[0].point.x == pytest.approx(1.0)
    assert scenario.obstacles[0].next_index == 1
    assert scenario.obstacles[1].next_index is None


def test_load_scenario_reads_json_and_uses_filename_as_default_name(tmp_path) -> None:
    scenario_path = tmp_path / "sample.json"
    scenario_path.write_text(
        json.dumps(
            {
                "time_step": 0.5,
                "steps": 1,
                "agents": [
                    {
                        "profile": {"radius": 0.5, "max_speed": 1.0},
                        "initial_position": [0.0, 0.0],
                        "initial_velocity": [0.0, 0.0],
                        "preferred_velocity": [1.0, 0.0],
                    }
                ],
            }
        ),
        encoding="utf-8",
    )

    scenario = load_scenario(scenario_path)

    assert scenario.name == "sample"
    assert scenario.agents[0].initial_position.x == pytest.approx(0.0)


def test_load_scenario_rejects_missing_agents(tmp_path) -> None:
    scenario_path = tmp_path / "invalid.yaml"
    scenario_path.write_text("name: invalid\nsteps: 1\n", encoding="utf-8")

    with pytest.raises(ValueError, match="agents"):
        load_scenario(scenario_path)


def test_load_scenario_rejects_invalid_vector_shape(tmp_path) -> None:
    scenario_path = tmp_path / "invalid.yaml"
    scenario_path.write_text(
        "\n".join(
            [
                "agents:",
                "  - initial_position: [0.0, 0.0, 0.0]",
                "    preferred_velocity: [1.0, 0.0]",
            ]
        ),
        encoding="utf-8",
    )

    with pytest.raises(ValueError, match="initial_position"):
        load_scenario(scenario_path)


def test_load_scenario_rejects_goal_stop_without_agent_goals(tmp_path) -> None:
    scenario_path = tmp_path / "invalid_goal_stop.yaml"
    scenario_path.write_text(
        "\n".join(
            [
                "stop_when_all_agents_reach_goals: true",
                "agents:",
                "  - initial_position: [0.0, 0.0]",
            ]
        ),
        encoding="utf-8",
    )

    with pytest.raises(ValueError, match="goal_position"):
        load_scenario(scenario_path)


@pytest.mark.parametrize(
    ("relative_path", "expected_agents", "expected_obstacle_vertices", "expected_steps"),
    [
        ("scenarios/cnav_queue_validation.yaml", 2, 0, 12),
        ("scenarios/cnav_head_on_validation.yaml", 2, 0, 16),
        ("scenarios/cnav_crossing_validation.yaml", 4, 0, 18),
        ("scenarios/cnav_obstacle_validation.yaml", 2, 4, 20),
        ("scenarios/cnav_forpaper_direct_port.yaml", 12, 144, 80),
        ("scenarios/cnav_crowd_forpaper_direct_port.yaml", 50, 8, 0),
    ],
)
def test_validation_scenarios_load_from_repo(
    relative_path: str,
    expected_agents: int,
    expected_obstacle_vertices: int,
    expected_steps: int,
) -> None:
    scenario = load_scenario(Path(relative_path))

    assert len(scenario.agents) == expected_agents
    assert all(agent.goal_position is not None for agent in scenario.agents)
    assert scenario.steps == expected_steps
    assert len(scenario.obstacles) == expected_obstacle_vertices


def test_direct_port_scenarios_keep_legacy_time_step_and_goal_stop() -> None:
    forpaper = load_scenario(Path("scenarios/cnav_forpaper_direct_port.yaml"))
    crowd = load_scenario(Path("scenarios/cnav_crowd_forpaper_direct_port.yaml"))

    assert forpaper.time_step == pytest.approx(1.0)
    assert crowd.time_step == pytest.approx(1.0)
    assert forpaper.stop_when_all_agents_reach_goals is False
    assert crowd.stop_when_all_agents_reach_goals is True
    assert forpaper.navigation_grid is not None
    assert len(forpaper.agents[0].goal_sequence) == 2
