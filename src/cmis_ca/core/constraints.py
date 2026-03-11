"""Algorithm-neutral constraint types."""

from __future__ import annotations

from dataclasses import dataclass, field

from cmis_ca.core.geometry import Vector2


@dataclass(frozen=True)
class LineConstraint:
    """Half-plane boundary in velocity space.

    The initial skeleton keeps only the geometry carrier. Algorithm-specific
    semantics belong under `algorithms/<name>/`.
    """

    point: Vector2 = field(default_factory=Vector2)
    direction: Vector2 = field(default_factory=lambda: Vector2(1.0, 0.0))
