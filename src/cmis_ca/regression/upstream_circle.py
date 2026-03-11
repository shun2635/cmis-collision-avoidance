"""Regression helpers for the upstream Circle scenario.

Derived from: https://github.com/snape/RVO2
Original license: Apache License 2.0
Modified for the CMIS Collision Avoidance project.
Summary of changes: extracted the Circle example conditions into scenario and
regression helpers for the Python codebase.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

from cmis_ca.algorithms.orca.algorithm import ORCAAlgorithm
from cmis_ca.algorithms.orca.parameters import ORCAParameters
from cmis_ca.core.geometry import Vector2
from cmis_ca.core.simulation import Simulator
from cmis_ca.core.state import AgentState
from cmis_ca.io import load_scenario


UPSTREAM_CIRCLE_SCENARIO = Path("scenarios/upstream_circle.yaml")


@dataclass(frozen=True)
class CircleRegressionMetrics:
    """Qualitative metrics used to compare Circle behavior across revisions."""

    steps_run: int
    average_radius: float
    minimum_pair_distance: float
    centroid_distance: float
    average_goal_distance: float
    max_speed: float
    max_antipodal_error: float


def run_upstream_circle_regression(
    steps: int | None = None,
    scenario_path: str | Path = UPSTREAM_CIRCLE_SCENARIO,
) -> CircleRegressionMetrics:
    """Run the upstream Circle scenario with per-step goal updates."""
    scenario = load_scenario(scenario_path)
    simulator = Simulator(
        scenario=scenario,
        algorithm=ORCAAlgorithm(
            parameters=ORCAParameters(
                neighbor_dist=15.0,
                max_neighbors=10,
                time_horizon=10.0,
                time_horizon_obst=10.0,
            )
        ),
    )
    goals = tuple(-agent.initial_position for agent in scenario.agents)
    total_steps = scenario.steps if steps is None else steps

    for _ in range(total_steps):
        for index, goal in enumerate(goals):
            goal_vector = goal - simulator.states[index].position
            preferred_velocity = (
                goal_vector.normalized() if goal_vector.abs_sq() > 1.0 else goal_vector
            )
            simulator.set_preferred_velocity(index, preferred_velocity)
        simulator.step()

    return _collect_metrics(simulator.states, goals, steps_run=total_steps)


def _collect_metrics(
    states: tuple[AgentState, ...],
    goals: tuple[Vector2, ...],
    steps_run: int,
) -> CircleRegressionMetrics:
    count = len(states)
    average_radius = sum(state.position.norm() for state in states) / count
    average_goal_distance = sum(
        state.position.distance_to(goal) for state, goal in zip(states, goals)
    ) / count
    centroid = Vector2(
        sum(state.position.x for state in states) / count,
        sum(state.position.y for state in states) / count,
    )
    max_speed = max(state.velocity.norm() for state in states)
    minimum_pair_distance = min(
        states[left].position.distance_to(states[right].position)
        for left in range(count)
        for right in range(left + 1, count)
    )
    half = count // 2
    max_antipodal_error = max(
        (states[index].position + states[(index + half) % count].position).norm()
        for index in range(half)
    )
    return CircleRegressionMetrics(
        steps_run=steps_run,
        average_radius=average_radius,
        minimum_pair_distance=minimum_pair_distance,
        centroid_distance=centroid.norm(),
        average_goal_distance=average_goal_distance,
        max_speed=max_speed,
        max_antipodal_error=max_antipodal_error,
    )
