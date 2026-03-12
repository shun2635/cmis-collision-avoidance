"""ORCA-specific constraint builders.

Derived from: https://github.com/snape/RVO2
Original license: Apache License 2.0
Modified for the CMIS Collision Avoidance project.
Summary of changes: rewrote agent-agent ORCA constraint generation in Python
and adapted obstacle handling to the current linked obstacle topology using a
closest-point approximation.
"""

from __future__ import annotations

from cmis_ca.algorithms.orca.parameters import ORCAParameters
from cmis_ca.core.constraints import LineConstraint
from cmis_ca.core.geometry import Vector2
from cmis_ca.core.neighbor_search import NeighborSet
from cmis_ca.core.world import SnapshotAgent, WorldSnapshot, obstacle_segment


def build_obstacle_constraints(
    snapshot: WorldSnapshot,
    agent_index: int,
    neighbors: NeighborSet,
    parameters: ORCAParameters,
) -> list[LineConstraint]:
    """Build ORCA constraints against static obstacle segments.

    The current repository now stores upstream-style obstacle topology as
    linked vertices. The actual ORCA obstacle line logic is still approximated
    via the closest point on each outgoing obstacle edge.
    """

    agent = _find_agent(snapshot, agent_index)
    constraints = []

    for obstacle_neighbor in neighbors.obstacle_neighbors:
        closest_point = _closest_point_on_segment(
            agent.state.position,
            snapshot.obstacles,
            obstacle_neighbor.index,
        )
        constraints.append(
            _build_orca_constraint(
                relative_position=closest_point - agent.state.position,
                relative_velocity=agent.state.velocity,
                combined_radius=agent.profile.radius,
                current_velocity=agent.state.velocity,
                time_horizon=parameters.time_horizon_obst,
                time_step=snapshot.time_step,
                responsibility=1.0,
            )
        )

    return constraints


def build_agent_constraints(
    snapshot: WorldSnapshot,
    agent_index: int,
    neighbors: NeighborSet,
    parameters: ORCAParameters,
) -> list[LineConstraint]:
    """Build ORCA constraints against neighboring agents."""

    agent = _find_agent(snapshot, agent_index)
    constraints = []

    for agent_neighbor in neighbors.agent_neighbors:
        other = _find_agent(snapshot, agent_neighbor.index)
        constraints.append(
            _build_orca_constraint(
                relative_position=other.state.position - agent.state.position,
                relative_velocity=agent.state.velocity - other.state.velocity,
                combined_radius=agent.profile.radius + other.profile.radius,
                current_velocity=agent.state.velocity,
                time_horizon=parameters.time_horizon,
                time_step=snapshot.time_step,
                responsibility=0.5,
            )
        )

    return constraints


def _build_orca_constraint(
    *,
    relative_position: Vector2,
    relative_velocity: Vector2,
    combined_radius: float,
    current_velocity: Vector2,
    time_horizon: float,
    time_step: float,
    responsibility: float,
) -> LineConstraint:
    if time_horizon <= 0.0:
        raise ValueError("time_horizon must be positive")
    if time_step <= 0.0:
        raise ValueError("time_step must be positive")
    if combined_radius < 0.0:
        raise ValueError("combined_radius must be non-negative")
    if not 0.0 < responsibility <= 1.0:
        raise ValueError("responsibility must be in the range (0, 1]")

    dist_sq = relative_position.abs_sq()
    combined_radius_sq = combined_radius * combined_radius
    adjustment: Vector2

    if dist_sq > combined_radius_sq:
        inv_time_horizon = 1.0 / time_horizon
        cutoff_offset = inv_time_horizon * relative_position
        w = relative_velocity - cutoff_offset
        w_length_sq = w.abs_sq()
        dot_product = w.dot(relative_position)

        if dot_product < 0.0 and dot_product * dot_product > combined_radius_sq * w_length_sq:
            unit_w = _normalized_or(w, _fallback_normal(relative_position))
            direction = _clockwise_perpendicular(unit_w)
            adjustment = (combined_radius * inv_time_horizon - w.norm()) * unit_w
        else:
            leg = (dist_sq - combined_radius_sq) ** 0.5
            if relative_position.det(w) > 0.0:
                direction = Vector2(
                    relative_position.x * leg - relative_position.y * combined_radius,
                    relative_position.x * combined_radius + relative_position.y * leg,
                ) / dist_sq
            else:
                direction = -Vector2(
                    relative_position.x * leg + relative_position.y * combined_radius,
                    -relative_position.x * combined_radius + relative_position.y * leg,
                ) / dist_sq

            adjustment = relative_velocity.dot(direction) * direction - relative_velocity
    else:
        inv_time_step = 1.0 / time_step
        w = relative_velocity - inv_time_step * relative_position
        unit_w = _normalized_or(w, _fallback_normal(relative_position))
        direction = _clockwise_perpendicular(unit_w)
        adjustment = (combined_radius * inv_time_step - w.norm()) * unit_w

    return LineConstraint(
        point=current_velocity + responsibility * adjustment,
        direction=direction,
    )


def _find_agent(snapshot: WorldSnapshot, agent_index: int) -> SnapshotAgent:
    for agent in snapshot.agents:
        if agent.index == agent_index:
            return agent
    raise ValueError(f"agent index {agent_index} is not present in the snapshot")


def _closest_point_on_segment(point: Vector2, obstacles, obstacle_index: int) -> Vector2:
    segment_start, segment_end = obstacle_segment(obstacles, obstacle_index)
    segment = segment_end - segment_start
    segment_length_sq = segment.abs_sq()
    if segment_length_sq == 0.0:
        return segment_start

    projection = (point - segment_start).dot(segment) / segment_length_sq
    clamped_projection = max(0.0, min(1.0, projection))
    return segment_start + clamped_projection * segment


def _clockwise_perpendicular(vector: Vector2) -> Vector2:
    return Vector2(vector.y, -vector.x)


def _fallback_normal(relative_position: Vector2) -> Vector2:
    if relative_position.norm() == 0.0:
        return Vector2(1.0, 0.0)
    return (-relative_position).normalized()


def _normalized_or(vector: Vector2, fallback: Vector2) -> Vector2:
    normalized = vector.normalized()
    if normalized.norm() == 0.0:
        return fallback
    return normalized
