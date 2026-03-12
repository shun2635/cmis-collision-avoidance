"""Regression helpers for the upstream Blocks scenario.

Derived from: https://github.com/snape/RVO2
Original license: Apache License 2.0
Modified for the CMIS Collision Avoidance project.
Summary of changes: represented the upstream Blocks example as a programmatic
scenario and regression helper for the Python codebase.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
import random

from cmis_ca.algorithms.orca.algorithm import ORCAAlgorithm
from cmis_ca.algorithms.orca.parameters import ORCAParameters
from cmis_ca.core.agent import AgentConfig, AgentProfile
from cmis_ca.core.geometry import Vector2
from cmis_ca.core.simulation import Simulator
from cmis_ca.core.state import AgentState
from cmis_ca.core.world import ObstaclePath, Scenario, build_obstacle_topology

UPSTREAM_BLOCKS_DEFAULT_STEPS = 64
UPSTREAM_BLOCKS_RANDOM_SEED = 7
_TWO_PI = 2.0 * math.pi


@dataclass(frozen=True)
class BlocksRegressionMetrics:
    """Qualitative metrics used to compare Blocks behavior across revisions."""

    steps_run: int
    average_goal_distance: float
    goal_distance_reduction: float
    minimum_pair_distance: float
    centroid_distance: float
    max_speed: float
    central_agent_count: int


def build_upstream_blocks_scenario(steps: int = UPSTREAM_BLOCKS_DEFAULT_STEPS) -> Scenario:
    """Build the upstream Blocks setup as a Python `Scenario`."""

    profile = AgentProfile(
        radius=2.0,
        max_speed=2.0,
        neighbor_dist=15.0,
        max_neighbors=10,
        time_horizon=5.0,
        time_horizon_obst=5.0,
    )

    agents: list[AgentConfig] = []
    for offset_x in range(5):
        for offset_y in range(5):
            agents.extend(
                (
                    _build_blocks_agent(
                        index=len(agents),
                        profile=profile,
                        position=Vector2(55.0 + offset_x * 10.0, 55.0 + offset_y * 10.0),
                        goal=Vector2(-75.0, -75.0),
                    ),
                    _build_blocks_agent(
                        index=len(agents) + 1,
                        profile=profile,
                        position=Vector2(-55.0 - offset_x * 10.0, 55.0 + offset_y * 10.0),
                        goal=Vector2(75.0, -75.0),
                    ),
                    _build_blocks_agent(
                        index=len(agents) + 2,
                        profile=profile,
                        position=Vector2(55.0 + offset_x * 10.0, -55.0 - offset_y * 10.0),
                        goal=Vector2(-75.0, 75.0),
                    ),
                    _build_blocks_agent(
                        index=len(agents) + 3,
                        profile=profile,
                        position=Vector2(-55.0 - offset_x * 10.0, -55.0 - offset_y * 10.0),
                        goal=Vector2(75.0, 75.0),
                    ),
                )
            )

    obstacles = build_obstacle_topology(
        (
            ObstaclePath(
                (
                    Vector2(-10.0, 40.0),
                    Vector2(-40.0, 40.0),
                    Vector2(-40.0, 10.0),
                    Vector2(-10.0, 10.0),
                )
            ),
            ObstaclePath(
                (
                    Vector2(10.0, 40.0),
                    Vector2(10.0, 10.0),
                    Vector2(40.0, 10.0),
                    Vector2(40.0, 40.0),
                )
            ),
            ObstaclePath(
                (
                    Vector2(10.0, -40.0),
                    Vector2(40.0, -40.0),
                    Vector2(40.0, -10.0),
                    Vector2(10.0, -10.0),
                )
            ),
            ObstaclePath(
                (
                    Vector2(-10.0, -40.0),
                    Vector2(-10.0, -10.0),
                    Vector2(-40.0, -10.0),
                    Vector2(-40.0, -40.0),
                )
            ),
        )
    )

    return Scenario(
        name="upstream-blocks",
        time_step=0.25,
        steps=steps,
        agents=tuple(agents),
        obstacles=obstacles,
    )


def run_upstream_blocks_regression(
    steps: int = UPSTREAM_BLOCKS_DEFAULT_STEPS,
    random_seed: int = UPSTREAM_BLOCKS_RANDOM_SEED,
) -> BlocksRegressionMetrics:
    """Run the upstream Blocks scenario and collect qualitative metrics."""

    scenario = build_upstream_blocks_scenario(steps=steps)
    simulator = Simulator(
        scenario=scenario,
        algorithm=ORCAAlgorithm(parameters=ORCAParameters()),
    )
    initial_average_goal_distance = _average_goal_distance_from_config(scenario.agents)
    rng = random.Random(random_seed)

    for _ in range(steps):
        _set_preferred_velocities(simulator, scenario.agents, rng)
        simulator.step()

    return _collect_metrics(
        states=simulator.states,
        agents=scenario.agents,
        steps_run=steps,
        initial_average_goal_distance=initial_average_goal_distance,
    )


def _build_blocks_agent(
    *,
    index: int,
    profile: AgentProfile,
    position: Vector2,
    goal: Vector2,
) -> AgentConfig:
    preferred_velocity = (goal - position).normalized()
    return AgentConfig(
        name=f"agent_{index:03d}",
        profile=profile,
        initial_position=position,
        initial_velocity=Vector2(),
        preferred_velocity=preferred_velocity,
        goal_position=goal,
        preferred_speed=1.0,
        auto_update_preferred_velocity_from_goal=False,
    )


def _set_preferred_velocities(
    simulator: Simulator,
    agents: tuple[AgentConfig, ...],
    rng: random.Random,
) -> None:
    for index, (config, state) in enumerate(zip(agents, simulator.states)):
        if config.goal_position is None:
            simulator.set_preferred_velocity(index, Vector2())
            continue

        goal_vector = config.goal_position - state.position
        if goal_vector.abs_sq() > 1.0:
            goal_vector = goal_vector.normalized()

        simulator.set_preferred_velocity(index, goal_vector + _tiny_perturbation(rng))


def _tiny_perturbation(rng: random.Random) -> Vector2:
    angle = rng.random() * _TWO_PI
    distance = rng.random() * 1.0e-4
    return Vector2(math.cos(angle), math.sin(angle)) * distance


def _average_goal_distance_from_config(agents: tuple[AgentConfig, ...]) -> float:
    goal_distances = [
        agent.initial_position.distance_to(agent.goal_position)
        for agent in agents
        if agent.goal_position is not None
    ]
    return sum(goal_distances) / len(goal_distances)


def _collect_metrics(
    *,
    states: tuple[AgentState, ...],
    agents: tuple[AgentConfig, ...],
    steps_run: int,
    initial_average_goal_distance: float,
) -> BlocksRegressionMetrics:
    count = len(states)
    average_goal_distance = sum(
        state.position.distance_to(agent.goal_position)
        for state, agent in zip(states, agents)
        if agent.goal_position is not None
    ) / count
    centroid = Vector2(
        sum(state.position.x for state in states) / count,
        sum(state.position.y for state in states) / count,
    )
    minimum_pair_distance = min(
        states[left].position.distance_to(states[right].position)
        for left in range(count)
        for right in range(left + 1, count)
    )
    max_speed = max(state.velocity.norm() for state in states)
    central_agent_count = sum(
        1
        for state in states
        if abs(state.position.x) < 45.0 and abs(state.position.y) < 45.0
    )

    return BlocksRegressionMetrics(
        steps_run=steps_run,
        average_goal_distance=average_goal_distance,
        goal_distance_reduction=initial_average_goal_distance - average_goal_distance,
        minimum_pair_distance=minimum_pair_distance,
        centroid_distance=centroid.norm(),
        max_speed=max_speed,
        central_agent_count=central_agent_count,
    )
