"""ORCA-specific constraint builders.

Derived from: https://github.com/snape/RVO2
Original license: Apache License 2.0
Modified for the CMIS Collision Avoidance project.
Summary of changes: rewrote agent-agent ORCA constraint generation in Python
and ported obstacle constraint generation to the linked obstacle topology.
"""

from __future__ import annotations

from cmis_ca.algorithms.orca.parameters import ORCAParameters
from cmis_ca.core.constraints import LineConstraint
from cmis_ca.core.geometry import Vector2
from cmis_ca.core.neighbor_search import NeighborSet
from cmis_ca.core.world import ObstacleVertex, SnapshotAgent, WorldSnapshot

EPSILON = 1e-5


def build_obstacle_constraints(
    snapshot: WorldSnapshot,
    agent_index: int,
    neighbors: NeighborSet,
    parameters: ORCAParameters,
) -> list[LineConstraint]:
    """Build ORCA constraints against static obstacles."""

    agent = _find_agent(snapshot, agent_index)
    if parameters.time_horizon_obst <= 0.0:
        raise ValueError("time_horizon_obst must be positive")

    constraints: list[LineConstraint] = []
    inv_time_horizon_obst = 1.0 / parameters.time_horizon_obst

    for obstacle_neighbor in neighbors.obstacle_neighbors:
        line = _build_single_obstacle_constraint(
            obstacles=snapshot.obstacles,
            obstacle_index=obstacle_neighbor.index,
            position=agent.state.position,
            velocity=agent.state.velocity,
            radius=agent.profile.radius,
            inv_time_horizon_obst=inv_time_horizon_obst,
            existing_constraints=constraints,
        )
        if line is not None:
            constraints.append(line)

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


def _build_single_obstacle_constraint(
    *,
    obstacles: tuple[ObstacleVertex, ...],
    obstacle_index: int,
    position: Vector2,
    velocity: Vector2,
    radius: float,
    inv_time_horizon_obst: float,
    existing_constraints: list[LineConstraint],
) -> LineConstraint | None:
    obstacle1_index = obstacle_index
    obstacle1 = obstacles[obstacle1_index]
    if obstacle1.next_index is None:
        return None

    obstacle2_index = obstacle1.next_index
    obstacle2 = obstacles[obstacle2_index]

    relative_position1 = obstacle1.point - position
    relative_position2 = obstacle2.point - position

    if _is_obstacle_already_covered(
        existing_constraints=existing_constraints,
        relative_position1=relative_position1,
        relative_position2=relative_position2,
        inv_time_horizon_obst=inv_time_horizon_obst,
        radius=radius,
    ):
        return None

    dist_sq1 = relative_position1.abs_sq()
    dist_sq2 = relative_position2.abs_sq()
    radius_sq = radius * radius

    obstacle_vector = obstacle2.point - obstacle1.point
    obstacle_length_sq = obstacle_vector.abs_sq()
    if obstacle_length_sq == 0.0:
        return None

    s = (-relative_position1).dot(obstacle_vector) / obstacle_length_sq
    dist_sq_line = (-relative_position1 - s * obstacle_vector).abs_sq()

    if s < 0.0 and dist_sq1 <= radius_sq:
        if obstacle1.is_convex:
            return LineConstraint(
                point=Vector2(),
                direction=_normalized_or(
                    Vector2(-relative_position1.y, relative_position1.x),
                    _clockwise_perpendicular(obstacle1.direction),
                ),
            )
        return None

    if s > 1.0 and dist_sq2 <= radius_sq:
        if obstacle2.is_convex and (
            obstacle2.next_index is None or relative_position2.det(obstacle2.direction) >= 0.0
        ):
            return LineConstraint(
                point=Vector2(),
                direction=_normalized_or(
                    Vector2(-relative_position2.y, relative_position2.x),
                    _clockwise_perpendicular(obstacle1.direction),
                ),
            )
        return None

    if 0.0 <= s <= 1.0 and dist_sq_line <= radius_sq:
        return LineConstraint(
            point=Vector2(),
            direction=-obstacle1.direction,
        )

    left_obstacle = obstacle1
    right_obstacle = obstacle2
    left_obstacle_index = obstacle1_index
    right_obstacle_index = obstacle2_index

    if s < 0.0 and dist_sq_line <= radius_sq:
        if not obstacle1.is_convex:
            return None

        right_obstacle = obstacle1
        right_obstacle_index = obstacle1_index

        leg1 = _sqrt_nonnegative(dist_sq1 - radius_sq)
        left_leg_direction = Vector2(
            relative_position1.x * leg1 - relative_position1.y * radius,
            relative_position1.x * radius + relative_position1.y * leg1,
        ) / dist_sq1
        right_leg_direction = Vector2(
            relative_position1.x * leg1 + relative_position1.y * radius,
            -relative_position1.x * radius + relative_position1.y * leg1,
        ) / dist_sq1
    elif s > 1.0 and dist_sq_line <= radius_sq:
        if not obstacle2.is_convex:
            return None

        left_obstacle = obstacle2
        left_obstacle_index = obstacle2_index

        leg2 = _sqrt_nonnegative(dist_sq2 - radius_sq)
        left_leg_direction = Vector2(
            relative_position2.x * leg2 - relative_position2.y * radius,
            relative_position2.x * radius + relative_position2.y * leg2,
        ) / dist_sq2
        right_leg_direction = Vector2(
            relative_position2.x * leg2 + relative_position2.y * radius,
            -relative_position2.x * radius + relative_position2.y * leg2,
        ) / dist_sq2
    else:
        if left_obstacle.is_convex:
            leg1 = _sqrt_nonnegative(dist_sq1 - radius_sq)
            left_leg_direction = Vector2(
                relative_position1.x * leg1 - relative_position1.y * radius,
                relative_position1.x * radius + relative_position1.y * leg1,
            ) / dist_sq1
        else:
            left_leg_direction = -left_obstacle.direction

        if right_obstacle.is_convex:
            leg2 = _sqrt_nonnegative(dist_sq2 - radius_sq)
            right_leg_direction = Vector2(
                relative_position2.x * leg2 + relative_position2.y * radius,
                -relative_position2.x * radius + relative_position2.y * leg2,
            ) / dist_sq2
        else:
            right_leg_direction = left_obstacle.direction

    is_left_leg_foreign = False
    is_right_leg_foreign = False

    if left_obstacle.previous_index is not None and left_obstacle.is_convex:
        left_neighbor = obstacles[left_obstacle.previous_index]
        if left_leg_direction.det(-left_neighbor.direction) >= 0.0:
            left_leg_direction = -left_neighbor.direction
            is_left_leg_foreign = True

    if right_obstacle.next_index is not None and right_obstacle.is_convex:
        if right_leg_direction.det(right_obstacle.direction) <= 0.0:
            right_leg_direction = right_obstacle.direction
            is_right_leg_foreign = True

    left_cutoff = inv_time_horizon_obst * (left_obstacle.point - position)
    right_cutoff = inv_time_horizon_obst * (right_obstacle.point - position)
    cutoff_vector = right_cutoff - left_cutoff

    if left_obstacle_index == right_obstacle_index:
        t = 0.5
    else:
        t = (velocity - left_cutoff).dot(cutoff_vector) / cutoff_vector.abs_sq()

    t_left = (velocity - left_cutoff).dot(left_leg_direction)
    t_right = (velocity - right_cutoff).dot(right_leg_direction)

    if (t < 0.0 and t_left < 0.0) or (
        left_obstacle_index == right_obstacle_index and t_left < 0.0 and t_right < 0.0
    ):
        unit_w = _normalized_or(velocity - left_cutoff, _fallback_normal(left_cutoff))
        return LineConstraint(
            point=left_cutoff + radius * inv_time_horizon_obst * unit_w,
            direction=_clockwise_perpendicular(unit_w),
        )

    if t > 1.0 and t_right < 0.0:
        unit_w = _normalized_or(velocity - right_cutoff, _fallback_normal(right_cutoff))
        return LineConstraint(
            point=right_cutoff + radius * inv_time_horizon_obst * unit_w,
            direction=_clockwise_perpendicular(unit_w),
        )

    if t < 0.0 or t > 1.0 or left_obstacle_index == right_obstacle_index:
        dist_sq_cutoff = float("inf")
    else:
        dist_sq_cutoff = (velocity - (left_cutoff + t * cutoff_vector)).abs_sq()

    if t_left < 0.0:
        dist_sq_left = float("inf")
    else:
        dist_sq_left = (velocity - (left_cutoff + t_left * left_leg_direction)).abs_sq()

    if t_right < 0.0:
        dist_sq_right = float("inf")
    else:
        dist_sq_right = (velocity - (right_cutoff + t_right * right_leg_direction)).abs_sq()

    if dist_sq_cutoff <= dist_sq_left and dist_sq_cutoff <= dist_sq_right:
        direction = -left_obstacle.direction
        return LineConstraint(
            point=left_cutoff + radius * inv_time_horizon_obst * _left_perpendicular(direction),
            direction=direction,
        )

    if dist_sq_left <= dist_sq_right:
        if is_left_leg_foreign:
            return None
        return LineConstraint(
            point=left_cutoff
            + radius * inv_time_horizon_obst * _left_perpendicular(left_leg_direction),
            direction=left_leg_direction,
        )

    if is_right_leg_foreign:
        return None

    direction = -right_leg_direction
    return LineConstraint(
        point=right_cutoff + radius * inv_time_horizon_obst * _left_perpendicular(direction),
        direction=direction,
    )


def _is_obstacle_already_covered(
    *,
    existing_constraints: list[LineConstraint],
    relative_position1: Vector2,
    relative_position2: Vector2,
    inv_time_horizon_obst: float,
    radius: float,
) -> bool:
    margin = inv_time_horizon_obst * radius
    test_point1 = inv_time_horizon_obst * relative_position1
    test_point2 = inv_time_horizon_obst * relative_position2

    for line in existing_constraints:
        if (
            _offset_side(test_point1, line) - margin >= -EPSILON
            and _offset_side(test_point2, line) - margin >= -EPSILON
        ):
            return True

    return False


def _offset_side(point: Vector2, line: LineConstraint) -> float:
    return (point - line.point).det(line.direction)


def _sqrt_nonnegative(value: float) -> float:
    return max(0.0, value) ** 0.5


def _left_perpendicular(vector: Vector2) -> Vector2:
    return Vector2(-vector.y, vector.x)


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
