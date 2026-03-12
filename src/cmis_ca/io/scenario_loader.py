"""Scenario file loader shared by the CLI and future batch tools."""

from __future__ import annotations

from pathlib import Path
import json
from typing import Any

import yaml

from cmis_ca.core.agent import AgentConfig, AgentProfile
from cmis_ca.core.geometry import Vector2
from cmis_ca.core.world import ObstacleSegment, Scenario


def load_scenario(path: str | Path) -> Scenario:
    """Load a `Scenario` from YAML or JSON."""
    scenario_path = Path(path)
    document = _load_document(scenario_path)
    return _parse_scenario(document, scenario_path)


def _load_document(path: Path) -> dict[str, Any]:
    if not path.exists():
        raise FileNotFoundError(f"scenario file not found: {path}")
    if not path.is_file():
        raise ValueError(f"scenario path must be a file: {path}")

    suffix = path.suffix.lower()
    with path.open("r", encoding="utf-8") as handle:
        if suffix in {".yaml", ".yml"}:
            loaded = yaml.safe_load(handle)
        elif suffix == ".json":
            loaded = json.load(handle)
        else:
            raise ValueError(f"unsupported scenario extension: {path.suffix}")

    if not isinstance(loaded, dict):
        raise ValueError("scenario document must be a mapping at the top level")
    return loaded


def _parse_scenario(document: dict[str, Any], path: Path) -> Scenario:
    agents_raw = document.get("agents")
    if not isinstance(agents_raw, list) or not agents_raw:
        raise ValueError(f"scenario '{path}' must define a non-empty 'agents' list")

    obstacles_raw = document.get("obstacles", [])
    if not isinstance(obstacles_raw, list):
        raise ValueError(f"scenario '{path}' field 'obstacles' must be a list")

    return Scenario(
        name=_parse_name(document.get("name"), path),
        time_step=_parse_float(document.get("time_step", 0.1), "time_step"),
        steps=_parse_int(document.get("steps", 1), "steps"),
        agents=tuple(_parse_agent(entry, index) for index, entry in enumerate(agents_raw)),
        obstacles=tuple(
            _parse_obstacle(entry, index) for index, entry in enumerate(obstacles_raw)
        ),
    )


def _parse_name(value: Any, path: Path) -> str:
    if value is None:
        return path.stem
    if not isinstance(value, str):
        raise ValueError("scenario field 'name' must be a string")
    return value


def _parse_agent(entry: Any, index: int) -> AgentConfig:
    if not isinstance(entry, dict):
        raise ValueError(f"agent[{index}] must be a mapping")

    profile_raw = entry.get("profile", {})
    if not isinstance(profile_raw, dict):
        raise ValueError(f"agent[{index}].profile must be a mapping")

    return AgentConfig(
        name=_parse_optional_string(entry.get("name"), f"agent[{index}].name"),
        profile=AgentProfile(
            radius=_parse_float(profile_raw.get("radius", 0.5), f"agent[{index}].profile.radius"),
            max_speed=_parse_float(
                profile_raw.get("max_speed", 1.0),
                f"agent[{index}].profile.max_speed",
            ),
        ),
        initial_position=_parse_vector(
            entry.get("initial_position", [0.0, 0.0]),
            f"agent[{index}].initial_position",
        ),
        initial_velocity=_parse_vector(
            entry.get("initial_velocity", [0.0, 0.0]),
            f"agent[{index}].initial_velocity",
        ),
        preferred_velocity=_parse_vector(
            entry.get("preferred_velocity", [0.0, 0.0]),
            f"agent[{index}].preferred_velocity",
        ),
        goal_position=_parse_optional_vector(
            entry.get("goal_position"),
            f"agent[{index}].goal_position",
        ),
        preferred_speed=_parse_float(
            entry.get("preferred_speed", 1.0),
            f"agent[{index}].preferred_speed",
        ),
    )


def _parse_obstacle(entry: Any, index: int) -> ObstacleSegment:
    if not isinstance(entry, dict):
        raise ValueError(f"obstacle[{index}] must be a mapping")
    return ObstacleSegment(
        start=_parse_vector(entry.get("start"), f"obstacle[{index}].start"),
        end=_parse_vector(entry.get("end"), f"obstacle[{index}].end"),
    )


def _parse_vector(value: Any, field_name: str) -> Vector2:
    if isinstance(value, (list, tuple)) and len(value) == 2:
        return Vector2(
            _parse_float(value[0], f"{field_name}[0]"),
            _parse_float(value[1], f"{field_name}[1]"),
        )
    if isinstance(value, dict) and {"x", "y"} <= value.keys():
        return Vector2(
            _parse_float(value["x"], f"{field_name}.x"),
            _parse_float(value["y"], f"{field_name}.y"),
        )
    raise ValueError(
        f"{field_name} must be a 2-element list or a mapping with 'x' and 'y'"
    )


def _parse_optional_string(value: Any, field_name: str) -> str:
    if value is None:
        return ""
    if not isinstance(value, str):
        raise ValueError(f"{field_name} must be a string")
    return value


def _parse_optional_vector(value: Any, field_name: str) -> Vector2 | None:
    if value is None:
        return None
    return _parse_vector(value, field_name)


def _parse_float(value: Any, field_name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{field_name} must be a number")
    return float(value)


def _parse_int(value: Any, field_name: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise ValueError(f"{field_name} must be an integer")
    return value
