"""Trace helpers for CNav parity and validation work."""

from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path
from typing import TYPE_CHECKING

from cmis_ca.core.geometry import Vector2
from cmis_ca.core.simulation import Simulator
from cmis_ca.core.state import SimulationResult
from cmis_ca.core.world import Scenario

if TYPE_CHECKING:
    from cmis_ca.algorithms.cnav.algorithm import CNavAlgorithm


@dataclass(frozen=True)
class CNavTraceAction:
    """Trace payload for one evaluated candidate action."""

    action_index: int
    intended_velocity: Vector2
    goal_progress_reward: float
    constrained_reduction_reward: float
    total_reward: float

    def to_dict(self) -> dict[str, object]:
        return {
            "action_index": self.action_index,
            "intended_velocity": _vector_to_list(self.intended_velocity),
            "goal_progress_reward": self.goal_progress_reward,
            "constrained_reduction_reward": self.constrained_reduction_reward,
            "total_reward": self.total_reward,
        }


@dataclass(frozen=True)
class CNavTraceAgentStep:
    """Trace payload for one agent at one simulation step."""

    step_index: int
    global_time: float
    agent_index: int
    agent_name: str
    action_updated: bool
    ranked_neighbors: tuple[int, ...]
    communicated_intended_velocity: Vector2
    chosen_action_index: int | None
    chosen_intended_velocity: Vector2
    output_velocity: Vector2
    candidate_actions: tuple[CNavTraceAction, ...]

    def to_dict(self) -> dict[str, object]:
        return {
            "step_index": self.step_index,
            "global_time": self.global_time,
            "agent_index": self.agent_index,
            "agent_name": self.agent_name,
            "action_updated": self.action_updated,
            "ranked_neighbors": list(self.ranked_neighbors),
            "communicated_intended_velocity": _vector_to_list(self.communicated_intended_velocity),
            "chosen_action_index": self.chosen_action_index,
            "chosen_intended_velocity": _vector_to_list(self.chosen_intended_velocity),
            "output_velocity": _vector_to_list(self.output_velocity),
            "candidate_actions": [candidate.to_dict() for candidate in self.candidate_actions],
        }


@dataclass(frozen=True)
class CNavTrace:
    """Flat trace for one scenario run."""

    scenario_name: str
    time_step: float
    records: tuple[CNavTraceAgentStep, ...]

    def to_jsonl(self) -> str:
        return "\n".join(json.dumps(record.to_dict(), sort_keys=True) for record in self.records)


def run_cnav_trace(
    scenario: Scenario,
    algorithm: CNavAlgorithm,
    *,
    steps: int | None = None,
) -> tuple[SimulationResult, CNavTrace]:
    """Run a CNav scenario and collect per-step agent traces."""

    simulator = Simulator(scenario=scenario, algorithm=algorithm)
    records: list[CNavTraceAgentStep] = []
    history = [simulator.states]

    if steps is not None:
        for _ in range(steps):
            history.append(simulator.step())
            records.extend(algorithm.latest_step_trace)
    elif scenario.stop_when_all_agents_reach_goals:
        max_steps = None if scenario.steps == 0 else scenario.steps
        steps_run = 0
        while max_steps is None or steps_run < max_steps:
            if simulator.all_agents_reached_goals():
                break
            history.append(simulator.step())
            records.extend(algorithm.latest_step_trace)
            steps_run += 1
    else:
        for _ in range(scenario.steps):
            history.append(simulator.step())
            records.extend(algorithm.latest_step_trace)

    result = SimulationResult(
        algorithm=algorithm.name,
        final_states=simulator.states,
        history=tuple(history),
    )
    trace = CNavTrace(
        scenario_name=scenario.name,
        time_step=scenario.time_step,
        records=tuple(records),
    )
    return result, trace


def write_cnav_trace_jsonl(trace: CNavTrace, path: str | Path) -> None:
    """Write a CNav trace as JSON Lines."""

    output_path = Path(path)
    output_path.write_text(trace.to_jsonl() + ("\n" if trace.records else ""), encoding="utf-8")


def _vector_to_list(vector: Vector2) -> list[float]:
    return [vector.x, vector.y]
