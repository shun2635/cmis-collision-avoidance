"""Tests for shared geometry primitives."""

from __future__ import annotations

import pytest

from cmis_ca.core.geometry import Vector2


def test_basic_arithmetic() -> None:
    left = Vector2(1.0, 2.0)
    right = Vector2(3.0, -4.0)

    assert left + right == Vector2(4.0, -2.0)
    assert left - right == Vector2(-2.0, 6.0)
    assert 2.0 * left == Vector2(2.0, 4.0)
    assert -left == Vector2(-1.0, -2.0)


def test_norm_and_normalization() -> None:
    vector = Vector2(3.0, 4.0)

    assert vector.abs_sq() == 25.0
    assert vector.norm() == 5.0
    assert vector.normalized() == Vector2(0.6, 0.8)
    assert Vector2().normalized() == Vector2()


def test_clamp_magnitude() -> None:
    vector = Vector2(3.0, 4.0)

    assert vector.clamp_magnitude(10.0) == vector
    assert vector.clamp_magnitude(5.0) == vector
    assert vector.clamp_magnitude(2.5) == Vector2(1.5, 2.0)


def test_division_and_distance() -> None:
    vector = Vector2(4.0, 6.0)

    assert vector / 2.0 == Vector2(2.0, 3.0)
    assert vector.distance_to(Vector2(1.0, 2.0)) == pytest.approx(5.0)
    assert vector.as_tuple() == (4.0, 6.0)


def test_dot_and_det() -> None:
    left = Vector2(1.0, 2.0)
    right = Vector2(3.0, 4.0)

    assert left.dot(right) == 11.0
    assert left.det(right) == -2.0


def test_division_by_zero_raises() -> None:
    with pytest.raises(ZeroDivisionError):
        _ = Vector2(1.0, 2.0) / 0.0


def test_negative_max_magnitude_raises() -> None:
    with pytest.raises(ValueError):
        Vector2(1.0, 0.0).clamp_magnitude(-1.0)
