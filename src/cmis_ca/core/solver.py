"""Reusable helpers for selecting a velocity under line constraints.

Derived from: https://github.com/snape/RVO2
Original license: Apache License 2.0
Modified for the CMIS Collision Avoidance project.
Summary of changes: rewrote the LP solver routines from `Agent.cc` as
algorithm-neutral Python helpers around `LineConstraint`.
"""

from __future__ import annotations

from typing import Sequence

from cmis_ca.core.constraints import LineConstraint
from cmis_ca.core.geometry import Vector2

EPSILON = 1e-9


def solve_linear_constraints(
    constraints: Sequence[LineConstraint],
    max_speed: float,
    optimization_velocity: Vector2,
    *,
    direction_opt: bool = False,
    protected_constraint_count: int = 0,
) -> Vector2:
    """Solve a 2D line-constrained optimization inside a speed-limit circle.

    The feasible region is defined by the intersection of half-planes from
    `constraints` and the circle centered at the origin with radius
    `max_speed`. Each `LineConstraint` is satisfied when its signed distance is
    non-negative.

    `protected_constraint_count` mirrors the role of obstacle lines in the
    upstream implementation: the first N constraints are kept fixed when the
    fallback projection step is applied.
    """

    if max_speed < 0.0:
        raise ValueError("max_speed must be non-negative")
    if protected_constraint_count < 0:
        raise ValueError("protected_constraint_count must be non-negative")
    if protected_constraint_count > len(constraints):
        raise ValueError("protected_constraint_count cannot exceed the number of constraints")

    result, failing_index = _linear_program_2(
        constraints=constraints,
        radius=max_speed,
        optimization_velocity=optimization_velocity,
        direction_opt=direction_opt,
    )
    if failing_index == len(constraints):
        return result

    return _linear_program_3(
        constraints=constraints,
        protected_constraint_count=protected_constraint_count,
        begin_index=failing_index,
        radius=max_speed,
        result=result,
    )


def choose_preferred_velocity(
    constraints: Sequence[LineConstraint],
    preferred_velocity: Vector2,
    max_speed: float,
) -> Vector2:
    """Choose the feasible velocity closest to the preferred velocity."""

    return solve_linear_constraints(
        constraints=constraints,
        max_speed=max_speed,
        optimization_velocity=preferred_velocity,
        direction_opt=False,
    )


def _linear_program_1(
    constraints: Sequence[LineConstraint],
    line_index: int,
    radius: float,
    optimization_velocity: Vector2,
    direction_opt: bool,
) -> Vector2 | None:
    line = constraints[line_index]
    dot_product = line.point.dot(line.direction)
    discriminant = dot_product * dot_product + radius * radius - line.point.abs_sq()

    if discriminant < 0.0:
        return None

    sqrt_discriminant = discriminant**0.5
    left = -dot_product - sqrt_discriminant
    right = -dot_product + sqrt_discriminant

    for previous_index in range(line_index):
        previous = constraints[previous_index]
        denominator = line.direction.det(previous.direction)
        numerator = previous.direction.det(line.point - previous.point)

        if abs(denominator) <= EPSILON:
            if numerator < 0.0:
                return None
            continue

        offset = numerator / denominator
        if denominator >= 0.0:
            right = min(right, offset)
        else:
            left = max(left, offset)

        if left > right:
            return None

    if direction_opt:
        if optimization_velocity.dot(line.direction) > 0.0:
            return line.point + right * line.direction
        return line.point + left * line.direction

    projection = line.direction.dot(optimization_velocity - line.point)
    if projection < left:
        return line.point + left * line.direction
    if projection > right:
        return line.point + right * line.direction
    return line.point + projection * line.direction


def _linear_program_2(
    constraints: Sequence[LineConstraint],
    radius: float,
    optimization_velocity: Vector2,
    direction_opt: bool,
) -> tuple[Vector2, int]:
    if direction_opt:
        result = optimization_velocity.normalized() * radius
    elif optimization_velocity.abs_sq() > radius * radius:
        result = optimization_velocity.normalized() * radius
    else:
        result = optimization_velocity

    for index, constraint in enumerate(constraints):
        if constraint.signed_distance(result) < -EPSILON:
            candidate = _linear_program_1(
                constraints=constraints,
                line_index=index,
                radius=radius,
                optimization_velocity=optimization_velocity,
                direction_opt=direction_opt,
            )
            if candidate is None:
                return result, index
            result = candidate

    return result, len(constraints)


def _linear_program_3(
    constraints: Sequence[LineConstraint],
    protected_constraint_count: int,
    begin_index: int,
    radius: float,
    result: Vector2,
) -> Vector2:
    max_violation = 0.0

    for index in range(begin_index, len(constraints)):
        constraint = constraints[index]
        violation = -constraint.signed_distance(result)
        if violation <= max_violation + EPSILON:
            continue

        projected_constraints = list(constraints[:protected_constraint_count])

        for previous_index in range(protected_constraint_count, index):
            previous = constraints[previous_index]
            determinant = constraint.direction.det(previous.direction)

            if abs(determinant) <= EPSILON:
                if constraint.direction.dot(previous.direction) > 0.0:
                    continue
                projected_point = 0.5 * (constraint.point + previous.point)
            else:
                projected_point = constraint.point + (
                    previous.direction.det(constraint.point - previous.point) / determinant
                ) * constraint.direction

            projected_direction = (previous.direction - constraint.direction).normalized()
            projected_constraints.append(
                LineConstraint(point=projected_point, direction=projected_direction)
            )

        previous_result = result
        result, failure_index = _linear_program_2(
            constraints=projected_constraints,
            radius=radius,
            optimization_velocity=constraint.normal,
            direction_opt=True,
        )
        if failure_index < len(projected_constraints):
            result = previous_result

        max_violation = -constraint.signed_distance(result)

    return result
