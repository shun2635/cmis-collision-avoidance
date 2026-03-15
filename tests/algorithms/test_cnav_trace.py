"""Tests for CNav trace parity helpers."""

from __future__ import annotations

from pathlib import Path

import pytest

from cmis_ca.algorithms.cnav import (
    CNavAlgorithm,
    create_cnav_parameters,
    run_cnav_trace,
    write_cnav_trace_jsonl,
)
from cmis_ca.io import load_scenario


@pytest.mark.parametrize(
    ("scenario_path", "steps", "expected_agents"),
    [
        ("scenarios/cnav_queue_validation.yaml", 3, 2),
        ("scenarios/cnav_head_on_validation.yaml", 3, 2),
        ("scenarios/cnav_crossing_validation.yaml", 2, 4),
        ("scenarios/cnav_obstacle_validation.yaml", 2, 2),
    ],
)
def test_run_cnav_trace_collects_validation_records(
    scenario_path: str,
    steps: int,
    expected_agents: int,
) -> None:
    scenario = load_scenario(Path(scenario_path))

    result, trace = run_cnav_trace(scenario, CNavAlgorithm(), steps=steps)

    assert result.algorithm == "cnav"
    assert len(result.history) == steps + 1
    assert trace.scenario_name == scenario.name
    assert len(trace.records) == steps * expected_agents
    assert any(record.action_updated for record in trace.records)
    assert any(len(record.candidate_actions) == 8 for record in trace.records if record.action_updated)
    assert all(record.chosen_intended_velocity.norm() >= 0.0 for record in trace.records)


def test_cnav_trace_records_include_chosen_action_and_ranked_neighbors() -> None:
    scenario = load_scenario(Path("scenarios/cnav_queue_validation.yaml"))

    _, trace = run_cnav_trace(scenario, CNavAlgorithm(), steps=3)

    first_record = trace.records[0]
    assert first_record.agent_index == 0
    assert first_record.action_updated is True
    assert first_record.chosen_action_index is not None
    assert len(first_record.candidate_actions) == 8
    assert isinstance(first_record.ranked_neighbors, tuple)

    carried_record = trace.records[2]
    assert carried_record.action_updated is False
    assert carried_record.chosen_action_index is not None
    assert carried_record.candidate_actions == ()


def test_write_cnav_trace_jsonl_emits_one_record_per_line(tmp_path) -> None:
    scenario = load_scenario(Path("scenarios/cnav_head_on_validation.yaml"))
    _, trace = run_cnav_trace(scenario, CNavAlgorithm(), steps=2)

    output_path = tmp_path / "trace.jsonl"
    write_cnav_trace_jsonl(trace, output_path)

    lines = output_path.read_text(encoding="utf-8").strip().splitlines()
    assert len(lines) == len(trace.records)
    assert '"step_index": 0' in lines[0]
    assert '"candidate_actions":' in lines[0]


def test_run_cnav_trace_accepts_legacy_forpaper_comparison_profile() -> None:
    scenario = load_scenario(Path("scenarios/cnav_queue_validation.yaml"))

    _, trace = run_cnav_trace(
        scenario,
        CNavAlgorithm(parameters=create_cnav_parameters("legacy-forpaper-comparison")),
        steps=3,
    )

    assert len(trace.records) == 6
    assert all(record.action_updated for record in trace.records)


def test_run_cnav_trace_accepts_forpaper_direct_port_scenario() -> None:
    scenario = load_scenario(Path("scenarios/cnav_forpaper_direct_port.yaml"))

    result, trace = run_cnav_trace(
        scenario,
        CNavAlgorithm(parameters=create_cnav_parameters("legacy-forpaper-comparison")),
        steps=1,
    )

    assert result.algorithm == "cnav"
    assert trace.scenario_name == "cnav-forpaper-direct-port"
    assert len(trace.records) == 12
    assert any(record.action_updated for record in trace.records)
