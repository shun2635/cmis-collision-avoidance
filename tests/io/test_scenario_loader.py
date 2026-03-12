"""Tests for scenario file loading."""

from __future__ import annotations

import json

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
                "agents:",
                "  - name: agent_0",
                "    profile:",
                "      radius: 0.4",
                "      max_speed: 1.2",
                "    initial_position: [0.0, 0.0]",
                "    goal_position: [2.0, 0.0]",
                "    preferred_speed: 0.75",
                "    preferred_velocity:",
                "      x: 1.0",
                "      y: 0.0",
                "obstacles:",
                "  - start: [1.0, -1.0]",
                "    end: [1.0, 1.0]",
            ]
        ),
        encoding="utf-8",
    )

    scenario = load_scenario(scenario_path)

    assert scenario.name == "yaml-sample"
    assert scenario.time_step == pytest.approx(0.25)
    assert scenario.steps == 3
    assert len(scenario.agents) == 1
    assert scenario.agents[0].profile.max_speed == pytest.approx(1.2)
    assert scenario.agents[0].preferred_velocity.x == pytest.approx(1.0)
    assert scenario.agents[0].goal_position is not None
    assert scenario.agents[0].goal_position.x == pytest.approx(2.0)
    assert scenario.agents[0].preferred_speed == pytest.approx(0.75)
    assert len(scenario.obstacles) == 1
    assert scenario.obstacles[0].start.x == pytest.approx(1.0)


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
