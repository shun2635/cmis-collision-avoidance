"""Action-set generation for the initial CNav implementation."""

from __future__ import annotations

import math

from cmis_ca.core.geometry import Vector2


def build_action_set_from_offsets(
    goal_velocity: Vector2,
    *,
    action_speeds: tuple[float, ...],
    angle_offsets_radians: tuple[float, ...],
) -> tuple[Vector2, ...]:
    """Build an action set by rotating the goal direction with per-action speeds."""

    if len(action_speeds) != len(angle_offsets_radians):
        raise ValueError("action_speeds and angle_offsets_radians must have the same length")
    if any(speed < 0.0 for speed in action_speeds):
        raise ValueError("action_speeds must be non-negative")

    if goal_velocity.abs_sq() == 0.0:
        return tuple(Vector2() for _ in action_speeds)

    goal_direction = goal_velocity.normalized()
    goal_angle = math.atan2(goal_direction.y, goal_direction.x)
    return tuple(
        Vector2(math.cos(goal_angle + angle), math.sin(goal_angle + angle)) * speed
        for speed, angle in zip(action_speeds, angle_offsets_radians, strict=True)
    )


def build_default_action_set(
    goal_velocity: Vector2,
    *,
    action_speed: float,
    beta_degrees: float,
) -> tuple[Vector2, ...]:
    """Build the paper's 8-action set around the goal-oriented direction."""

    if action_speed < 0.0:
        raise ValueError("action_speed must be non-negative")
    if beta_degrees < 0.0:
        raise ValueError("beta_degrees must be non-negative")

    goal_direction = goal_velocity.normalized()
    goal_angle = math.atan2(goal_direction.y, goal_direction.x)
    beta = math.radians(beta_degrees)
    angles = (
        goal_angle,
        goal_angle + beta,
        goal_angle - beta,
        goal_angle + math.pi / 2.0,
        goal_angle - math.pi / 2.0,
        goal_angle + math.pi,
        goal_angle + math.pi + beta,
        goal_angle + math.pi - beta,
    )
    return build_action_set_from_offsets(
        goal_velocity,
        action_speeds=(action_speed,) * 8,
        angle_offsets_radians=tuple(angle - goal_angle for angle in angles),
    )
