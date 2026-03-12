"""Convert simulation data into visualization-ready traces."""

from __future__ import annotations

from cmis_ca.core.state import SimulationResult
from cmis_ca.core.world import ObstacleVertex, Scenario
from cmis_ca.visualization.models import ObstaclePrimitive, VisualizationFrame, VisualizationTrace


def build_visualization_trace(scenario: Scenario, result: SimulationResult) -> VisualizationTrace:
    """Build a visualization trace from a scenario and simulation history."""

    if not result.history:
        raise ValueError("simulation result must contain at least one frame")

    expected_agents = len(scenario.agents)
    for frame in result.history:
        if len(frame) != expected_agents:
            raise ValueError("simulation history frame size must match scenario agent count")

    return VisualizationTrace(
        scenario_name=scenario.name,
        algorithm=result.algorithm,
        time_step=scenario.time_step,
        agent_names=tuple(config.name or f"agent_{index}" for index, config in enumerate(scenario.agents)),
        agent_radii=tuple(config.profile.radius for config in scenario.agents),
        initial_positions=tuple(config.initial_position for config in scenario.agents),
        obstacles=_build_obstacle_primitives(scenario.obstacles),
        frames=tuple(
            VisualizationFrame(
                step_index=step_index,
                global_time=step_index * scenario.time_step,
                positions=tuple(state.position for state in frame),
            )
            for step_index, frame in enumerate(result.history)
        ),
    )


def _build_obstacle_primitives(obstacles: tuple[ObstacleVertex, ...]) -> tuple[ObstaclePrimitive, ...]:
    if not obstacles:
        return ()

    grouped: dict[int, list[ObstacleVertex]] = {}
    for obstacle in obstacles:
        grouped.setdefault(obstacle.obstacle_id, []).append(obstacle)

    primitives = []
    for vertices in grouped.values():
        ordered_vertices = tuple(sorted(vertices, key=lambda obstacle: obstacle.vertex_id))
        first = ordered_vertices[0]
        last = ordered_vertices[-1]
        primitives.append(
            ObstaclePrimitive(
                points=tuple(obstacle.point for obstacle in ordered_vertices),
                closed=last.next_index == first.vertex_id,
            )
        )

    return tuple(primitives)
