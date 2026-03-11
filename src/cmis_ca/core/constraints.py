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

    def __post_init__(self) -> None:
        if self.direction.norm() == 0.0:
            raise ValueError("constraint direction must be non-zero")
        object.__setattr__(self, "direction", self.direction.normalized())

    @property
    def normal(self) -> Vector2:
        return Vector2(-self.direction.y, self.direction.x)

    def signed_distance(self, velocity: Vector2) -> float:
        return self.normal.dot(velocity - self.point)
