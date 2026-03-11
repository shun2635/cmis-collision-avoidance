"""Geometry primitives shared across algorithms."""

from __future__ import annotations

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class Vector2:
    """Simple 2D vector used throughout the package."""

    x: float = 0.0
    y: float = 0.0

    def __add__(self, other: "Vector2") -> "Vector2":
        return Vector2(self.x + other.x, self.y + other.y)

    def __sub__(self, other: "Vector2") -> "Vector2":
        return Vector2(self.x - other.x, self.y - other.y)

    def __mul__(self, scalar: float) -> "Vector2":
        return Vector2(self.x * scalar, self.y * scalar)

    def __rmul__(self, scalar: float) -> "Vector2":
        return self * scalar

    def __truediv__(self, scalar: float) -> "Vector2":
        if scalar == 0.0:
            raise ZeroDivisionError("cannot divide a vector by zero")
        return Vector2(self.x / scalar, self.y / scalar)

    def __neg__(self) -> "Vector2":
        return Vector2(-self.x, -self.y)

    def dot(self, other: "Vector2") -> float:
        return self.x * other.x + self.y * other.y

    def det(self, other: "Vector2") -> float:
        return self.x * other.y - self.y * other.x

    def abs_sq(self) -> float:
        return self.x * self.x + self.y * self.y

    def norm(self) -> float:
        return math.hypot(self.x, self.y)

    def normalized(self) -> "Vector2":
        length = self.norm()
        if length == 0.0:
            return Vector2()
        return Vector2(self.x / length, self.y / length)

    def clamp_magnitude(self, max_magnitude: float) -> "Vector2":
        if max_magnitude < 0.0:
            raise ValueError("max_magnitude must be non-negative")

        length = self.norm()
        if length == 0.0 or length <= max_magnitude:
            return self

        scale = max_magnitude / length
        return Vector2(self.x * scale, self.y * scale)

    def distance_to(self, other: "Vector2") -> float:
        return (self - other).norm()

    def as_tuple(self) -> tuple[float, float]:
        return (self.x, self.y)
