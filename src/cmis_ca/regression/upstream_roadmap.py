"""Regression helpers for the upstream Roadmap scenario.

Derived from: https://github.com/snape/RVO2
Original license: Apache License 2.0
Modified for the CMIS Collision Avoidance project.
Summary of changes: represented the upstream Roadmap example as a programmatic
scenario, visibility graph, and regression helper for the Python codebase.
"""

from __future__ import annotations

from dataclasses import dataclass
from functools import lru_cache
import heapq
import math
import random

from cmis_ca.algorithms.orca.algorithm import ORCAAlgorithm
from cmis_ca.algorithms.orca.parameters import ORCAParameters
from cmis_ca.core.agent import AgentConfig, AgentProfile
from cmis_ca.core.geometry import Vector2
from cmis_ca.core.simulation import Simulator
from cmis_ca.core.state import AgentState
from cmis_ca.core.world import (
    ObstaclePath,
    ObstacleVertex,
    Scenario,
    obstacle_edges,
    obstacle_segment,
    build_obstacle_topology,
)


UPSTREAM_ROADMAP_TEST_STEPS = 32
UPSTREAM_ROADMAP_DEFAULT_STEPS = 64
UPSTREAM_ROADMAP_RANDOM_SEED = 7
UPSTREAM_ROADMAP_GOAL_RADIUS_SQ = 400.0
_TWO_PI = 2.0 * math.pi


@dataclass(frozen=True)
class RoadmapVertex:
    """One visibility-graph vertex used by the Roadmap scenario."""

    position: Vector2
    neighbors: tuple[int, ...]
    dist_to_goal: tuple[float, ...]


@dataclass(frozen=True)
class RoadmapScenarioSetup:
    """Scenario plus roadmap metadata required for preferred-velocity updates."""

    scenario: Scenario
    roadmap: tuple[RoadmapVertex, ...]
    goals: tuple[int, ...]


@dataclass(frozen=True)
class RoadmapRegressionMetrics:
    """Qualitative metrics used to compare Roadmap behavior across revisions."""

    steps_run: int
    reached_goal: bool
    reached_goal_count: int
    average_goal_distance: float
    goal_distance_reduction: float
    minimum_pair_distance: float
    centroid_distance: float
    max_speed: float
    passage_agent_count: int


@lru_cache(maxsize=None)
def build_upstream_roadmap_setup(
    steps: int = UPSTREAM_ROADMAP_DEFAULT_STEPS,
) -> RoadmapScenarioSetup:
    """Build the upstream Roadmap setup as a scenario plus visibility graph."""

    profile = AgentProfile(
        radius=2.0,
        max_speed=2.0,
        neighbor_dist=15.0,
        max_neighbors=10,
        time_horizon=5.0,
        time_horizon_obst=5.0,
    )
    obstacles = _build_roadmap_obstacles()
    roadmap_positions = _build_roadmap_positions()
    roadmap = _build_visibility_roadmap(roadmap_positions, obstacles, profile.radius)

    agents: list[AgentConfig] = []
    goals: list[int] = []
    for offset_x in range(5):
        for offset_y in range(5):
            agents.extend(
                (
                    _build_roadmap_agent(
                        index=len(agents),
                        profile=profile,
                        position=Vector2(55.0 + offset_x * 10.0, 55.0 + offset_y * 10.0),
                    ),
                    _build_roadmap_agent(
                        index=len(agents) + 1,
                        profile=profile,
                        position=Vector2(-55.0 - offset_x * 10.0, 55.0 + offset_y * 10.0),
                    ),
                    _build_roadmap_agent(
                        index=len(agents) + 2,
                        profile=profile,
                        position=Vector2(55.0 + offset_x * 10.0, -55.0 - offset_y * 10.0),
                    ),
                    _build_roadmap_agent(
                        index=len(agents) + 3,
                        profile=profile,
                        position=Vector2(-55.0 - offset_x * 10.0, -55.0 - offset_y * 10.0),
                    ),
                )
            )
            goals.extend((0, 1, 2, 3))

    return RoadmapScenarioSetup(
        scenario=Scenario(
            name="upstream-roadmap",
            time_step=0.25,
            steps=steps,
            agents=tuple(agents),
            obstacles=obstacles,
        ),
        roadmap=roadmap,
        goals=tuple(goals),
    )


def run_upstream_roadmap_regression(
    steps: int = UPSTREAM_ROADMAP_DEFAULT_STEPS,
    random_seed: int = UPSTREAM_ROADMAP_RANDOM_SEED,
) -> RoadmapRegressionMetrics:
    """Run the upstream Roadmap setup with deterministic perturbation."""

    setup = build_upstream_roadmap_setup(steps=steps)
    simulator = Simulator(
        scenario=setup.scenario,
        algorithm=ORCAAlgorithm(parameters=ORCAParameters()),
    )
    rng = random.Random(random_seed)
    initial_average_goal_distance = _average_goal_distance(simulator.states, setup)

    steps_run = 0
    while steps_run < steps and not _reached_goal(simulator.states, setup):
        _set_preferred_velocities(simulator, setup, rng)
        simulator.step()
        steps_run += 1

    return _collect_metrics(
        states=simulator.states,
        setup=setup,
        steps_run=steps_run,
        initial_average_goal_distance=initial_average_goal_distance,
    )


def _build_roadmap_agent(
    *,
    index: int,
    profile: AgentProfile,
    position: Vector2,
) -> AgentConfig:
    return AgentConfig(
        name=f"agent_{index:03d}",
        profile=profile,
        initial_position=position,
        initial_velocity=Vector2(),
        preferred_velocity=Vector2(),
        goal_position=None,
        preferred_speed=1.0,
    )


def _build_roadmap_obstacles() -> tuple[ObstacleVertex, ...]:
    return build_obstacle_topology(
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


def _build_roadmap_positions() -> tuple[Vector2, ...]:
    return (
        Vector2(-75.0, -75.0),
        Vector2(75.0, -75.0),
        Vector2(-75.0, 75.0),
        Vector2(75.0, 75.0),
        Vector2(-42.0, -42.0),
        Vector2(-42.0, -8.0),
        Vector2(-42.0, 8.0),
        Vector2(-42.0, 42.0),
        Vector2(-8.0, -42.0),
        Vector2(-8.0, -8.0),
        Vector2(-8.0, 8.0),
        Vector2(-8.0, 42.0),
        Vector2(8.0, -42.0),
        Vector2(8.0, -8.0),
        Vector2(8.0, 8.0),
        Vector2(8.0, 42.0),
        Vector2(42.0, -42.0),
        Vector2(42.0, -8.0),
        Vector2(42.0, 8.0),
        Vector2(42.0, 42.0),
    )


def _build_visibility_roadmap(
    positions: tuple[Vector2, ...],
    obstacles: tuple[ObstacleVertex, ...],
    radius: float,
) -> tuple[RoadmapVertex, ...]:
    neighbors: list[list[int]] = [[] for _ in positions]
    for left, left_position in enumerate(positions):
        for right, right_position in enumerate(positions):
            if _query_visibility(left_position, right_position, radius, obstacles):
                neighbors[left].append(right)

    dist_to_goal = [
        _compute_goal_distances(goal_index, positions, neighbors) for goal_index in range(4)
    ]

    return tuple(
        RoadmapVertex(
            position=position,
            neighbors=tuple(neighbors[index]),
            dist_to_goal=tuple(distances[index] for distances in dist_to_goal),
        )
        for index, position in enumerate(positions)
    )


def _compute_goal_distances(
    goal_index: int,
    positions: tuple[Vector2, ...],
    neighbors: list[list[int]],
) -> list[float]:
    distances = [math.inf] * len(positions)
    distances[goal_index] = 0.0
    queue: list[tuple[float, int]] = [(0.0, goal_index)]

    while queue:
        current_distance, node = heapq.heappop(queue)
        if current_distance > distances[node]:
            continue

        for neighbor in neighbors[node]:
            edge_length = positions[node].distance_to(positions[neighbor])
            candidate_distance = current_distance + edge_length
            if candidate_distance >= distances[neighbor]:
                continue
            distances[neighbor] = candidate_distance
            heapq.heappush(queue, (candidate_distance, neighbor))

    return distances


def _query_visibility(
    point1: Vector2,
    point2: Vector2,
    radius: float,
    obstacles: tuple[ObstacleVertex, ...],
) -> bool:
    if point1 == point2:
        return True

    for obstacle_index in obstacle_edges(obstacles):
        obstacle1, obstacle2 = obstacle_segment(obstacles, obstacle_index)
        if _segments_intersect(point1, point2, obstacle1, obstacle2):
            return False
        if _segment_distance_sq(point1, point2, obstacle1, obstacle2) < radius * radius:
            return False

    return True


def _segment_distance_sq(point1: Vector2, point2: Vector2, point3: Vector2, point4: Vector2) -> float:
    if _segments_intersect(point1, point2, point3, point4):
        return 0.0
    return min(
        _point_to_segment_distance_sq(point1, point3, point4),
        _point_to_segment_distance_sq(point2, point3, point4),
        _point_to_segment_distance_sq(point3, point1, point2),
        _point_to_segment_distance_sq(point4, point1, point2),
    )


def _point_to_segment_distance_sq(point: Vector2, segment_start: Vector2, segment_end: Vector2) -> float:
    segment = segment_end - segment_start
    segment_length_sq = segment.abs_sq()
    if segment_length_sq == 0.0:
        return (point - segment_start).abs_sq()

    projection = (point - segment_start).dot(segment) / segment_length_sq
    clamped_projection = max(0.0, min(1.0, projection))
    closest_point = segment_start + segment * clamped_projection
    return (point - closest_point).abs_sq()


def _segments_intersect(
    point1: Vector2,
    point2: Vector2,
    point3: Vector2,
    point4: Vector2,
) -> bool:
    orientation1 = _orientation(point1, point2, point3)
    orientation2 = _orientation(point1, point2, point4)
    orientation3 = _orientation(point3, point4, point1)
    orientation4 = _orientation(point3, point4, point2)

    if orientation1 == 0.0 and _on_segment(point1, point3, point2):
        return True
    if orientation2 == 0.0 and _on_segment(point1, point4, point2):
        return True
    if orientation3 == 0.0 and _on_segment(point3, point1, point4):
        return True
    if orientation4 == 0.0 and _on_segment(point3, point2, point4):
        return True

    return (orientation1 > 0.0) != (orientation2 > 0.0) and (orientation3 > 0.0) != (
        orientation4 > 0.0
    )


def _orientation(point1: Vector2, point2: Vector2, point3: Vector2) -> float:
    value = (point2 - point1).det(point3 - point1)
    if abs(value) < 1.0e-9:
        return 0.0
    return value


def _on_segment(segment_start: Vector2, point: Vector2, segment_end: Vector2) -> bool:
    return (
        min(segment_start.x, segment_end.x) - 1.0e-9
        <= point.x
        <= max(segment_start.x, segment_end.x) + 1.0e-9
        and min(segment_start.y, segment_end.y) - 1.0e-9
        <= point.y
        <= max(segment_start.y, segment_end.y) + 1.0e-9
    )


def _set_preferred_velocities(
    simulator: Simulator,
    setup: RoadmapScenarioSetup,
    rng: random.Random,
) -> None:
    for index, state in enumerate(simulator.states):
        goal_index = setup.goals[index]
        min_dist = math.inf
        min_vertex = -1
        radius = setup.scenario.agents[index].profile.radius

        for vertex_index, vertex in enumerate(setup.roadmap):
            candidate_distance = state.position.distance_to(vertex.position) + vertex.dist_to_goal[
                goal_index
            ]
            if candidate_distance >= min_dist:
                continue
            if not _query_visibility(state.position, vertex.position, radius, setup.scenario.obstacles):
                continue
            min_dist = candidate_distance
            min_vertex = vertex_index

        preferred_velocity = _preferred_velocity_for_vertex(state, setup, index, min_vertex)
        simulator.set_preferred_velocity(index, preferred_velocity + _tiny_perturbation(rng))


def _preferred_velocity_for_vertex(
    state: AgentState,
    setup: RoadmapScenarioSetup,
    agent_index: int,
    min_vertex: int,
) -> Vector2:
    if min_vertex == -1:
        return Vector2()

    goal_index = setup.goals[agent_index]
    target_position = setup.roadmap[min_vertex].position
    if (target_position - state.position).abs_sq() == 0.0:
        if min_vertex == goal_index:
            return Vector2()
        return (setup.roadmap[goal_index].position - state.position).normalized()

    return (target_position - state.position).normalized()


def _tiny_perturbation(rng: random.Random) -> Vector2:
    angle = rng.random() * _TWO_PI
    distance = rng.random() * 1.0e-4
    return Vector2(math.cos(angle), math.sin(angle)) * distance


def _reached_goal(states: tuple[AgentState, ...], setup: RoadmapScenarioSetup) -> bool:
    return _reached_goal_count(states, setup) == len(states)


def _reached_goal_count(states: tuple[AgentState, ...], setup: RoadmapScenarioSetup) -> int:
    count = 0
    for index, state in enumerate(states):
        goal_position = setup.roadmap[setup.goals[index]].position
        if (state.position - goal_position).abs_sq() <= UPSTREAM_ROADMAP_GOAL_RADIUS_SQ:
            count += 1
    return count


def _average_goal_distance(states: tuple[AgentState, ...], setup: RoadmapScenarioSetup) -> float:
    return sum(
        state.position.distance_to(setup.roadmap[setup.goals[index]].position)
        for index, state in enumerate(states)
    ) / len(states)


def _collect_metrics(
    *,
    states: tuple[AgentState, ...],
    setup: RoadmapScenarioSetup,
    steps_run: int,
    initial_average_goal_distance: float,
) -> RoadmapRegressionMetrics:
    count = len(states)
    average_goal_distance = _average_goal_distance(states, setup)
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
    passage_agent_count = sum(
        1 for state in states if abs(state.position.x) < 12.0 and abs(state.position.y) < 45.0
    )

    return RoadmapRegressionMetrics(
        steps_run=steps_run,
        reached_goal=_reached_goal(states, setup),
        reached_goal_count=_reached_goal_count(states, setup),
        average_goal_distance=average_goal_distance,
        goal_distance_reduction=initial_average_goal_distance - average_goal_distance,
        minimum_pair_distance=minimum_pair_distance,
        centroid_distance=centroid.norm(),
        max_speed=max_speed,
        passage_agent_count=passage_agent_count,
    )
