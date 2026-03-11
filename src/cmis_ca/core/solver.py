"""Reusable helpers for selecting a velocity under simple constraints."""

from __future__ import annotations

from typing import Sequence

from cmis_ca.core.constraints import LineConstraint
from cmis_ca.core.geometry import Vector2


def choose_preferred_velocity(
    constraints: Sequence[LineConstraint],
    preferred_velocity: Vector2,
    max_speed: float,
) -> Vector2:
    """Select a command velocity for the initial skeleton.

    Constraint handling is intentionally deferred. The function exists now so
    the ORCA implementation can later replace the placeholder with a proper
    half-plane + circle solver without changing the package structure.
    """

    _ = constraints
    return preferred_velocity.clamp_magnitude(max_speed)
