"""CLI tests for scenario loading and step overrides."""

from __future__ import annotations

import pytest

from cmis_ca.cli.main import main


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


def test_cli_run_rejects_negative_steps() -> None:
    with pytest.raises(SystemExit) as exc_info:
        main(["run", "--algorithm", "orca", "--steps", "-1"])

    assert exc_info.value.code == 2
