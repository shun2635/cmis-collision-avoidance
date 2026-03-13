"""Action-set generation for the initial CNav implementation."""

from __future__ import annotations

import math

from cmis_ca.core.geometry import Vector2


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

    if goal_velocity.abs_sq() == 0.0 or action_speed == 0.0:
        return (Vector2(),) * 8

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
    return tuple(Vector2(math.cos(angle), math.sin(angle)) * action_speed for angle in angles)
