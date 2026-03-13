"""CLI tests for scenario loading and step overrides."""

from __future__ import annotations

import pytest

import cmis_ca.cli.main as cli_main

main = cli_main.main


def test_cli_run_accepts_external_scenario(capsys) -> None:
    exit_code = main(
        [
            "run",
            "--algorithm",
            "orca",
            "--scenario",
            "scenarios/head_on.yaml",
        ]
    )

    captured = capsys.readouterr()

    assert exit_code == 0
    assert "agent=0" in captured.out
    assert "agent=1" in captured.out


def test_cli_run_steps_override_external_scenario(capsys) -> None:
    exit_code = main(
        [
            "run",
            "--algorithm",
            "orca",
            "--scenario",
            "scenarios/head_on.yaml",
            "--steps",
            "1",
        ]
    )

    captured = capsys.readouterr()
    lines = [line for line in captured.out.splitlines() if line.startswith("agent=")]

    assert exit_code == 0
    assert "position=(-0.950, 0.000)" in lines[0]
    assert "position=(0.950, 0.000)" in lines[1]


def test_cli_run_accepts_cnav_external_scenario(capsys) -> None:
    exit_code = main(
        [
            "run",
            "--algorithm",
            "cnav",
            "--scenario",
            "scenarios/cnav_queue.yaml",
        ]
    )

    captured = capsys.readouterr()

    assert exit_code == 0
    assert "agent=0" in captured.out
    assert "agent=1" in captured.out


def test_cli_run_rejects_negative_steps() -> None:
    with pytest.raises(SystemExit) as exc_info:
        main(["run", "--algorithm", "orca", "--steps", "-1"])

    assert exc_info.value.code == 2


def test_cli_visualize_accepts_external_scenario(monkeypatch) -> None:
    called = {}

    def fake_run_visualization(algorithm_name, scenario_path, *, steps, fps):
        called["algorithm_name"] = algorithm_name
        called["scenario_path"] = scenario_path
        called["steps"] = steps
        called["fps"] = fps

    monkeypatch.setattr(cli_main, "run_visualization", fake_run_visualization)

    exit_code = main(
        [
            "visualize",
            "--algorithm",
            "orca",
            "--scenario",
            "scenarios/head_on.yaml",
            "--steps",
            "3",
            "--fps",
            "24",
        ]
    )

    assert exit_code == 0
    assert called == {
        "algorithm_name": "orca",
        "scenario_path": "scenarios/head_on.yaml",
        "steps": 3,
        "fps": 24.0,
    }


def test_cli_visualize_rejects_non_positive_fps() -> None:
    with pytest.raises(SystemExit) as exc_info:
        main(["visualize", "--algorithm", "orca", "--fps", "0"])

    assert exc_info.value.code == 2
