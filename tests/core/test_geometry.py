"""Unit tests for shared geometry primitives."""

from __future__ import annotations

import math
import unittest

from cmis_ca.core.geometry import Vector2


class Vector2Test(unittest.TestCase):
    def test_basic_arithmetic(self) -> None:
        left = Vector2(1.0, 2.0)
        right = Vector2(3.0, -4.0)

        self.assertEqual(left + right, Vector2(4.0, -2.0))
        self.assertEqual(left - right, Vector2(-2.0, 6.0))
        self.assertEqual(2.0 * left, Vector2(2.0, 4.0))
        self.assertEqual(-left, Vector2(-1.0, -2.0))

    def test_norm_and_normalization(self) -> None:
        vector = Vector2(3.0, 4.0)

        self.assertEqual(vector.abs_sq(), 25.0)
        self.assertEqual(vector.norm(), 5.0)
        self.assertEqual(vector.normalized(), Vector2(0.6, 0.8))
        self.assertEqual(Vector2().normalized(), Vector2())

    def test_clamp_magnitude(self) -> None:
        vector = Vector2(3.0, 4.0)

        self.assertEqual(vector.clamp_magnitude(10.0), vector)
        self.assertEqual(vector.clamp_magnitude(5.0), vector)
        self.assertEqual(vector.clamp_magnitude(2.5), Vector2(1.5, 2.0))

    def test_division_and_distance(self) -> None:
        vector = Vector2(4.0, 6.0)

        self.assertEqual(vector / 2.0, Vector2(2.0, 3.0))
        self.assertAlmostEqual(vector.distance_to(Vector2(1.0, 2.0)), 5.0)
        self.assertEqual(vector.as_tuple(), (4.0, 6.0))

    def test_dot_and_det(self) -> None:
        left = Vector2(1.0, 2.0)
        right = Vector2(3.0, 4.0)

        self.assertEqual(left.dot(right), 11.0)
        self.assertEqual(left.det(right), -2.0)

    def test_division_by_zero_raises(self) -> None:
        with self.assertRaises(ZeroDivisionError):
            _ = Vector2(1.0, 2.0) / 0.0

    def test_negative_max_magnitude_raises(self) -> None:
        with self.assertRaises(ValueError):
            Vector2(1.0, 0.0).clamp_magnitude(-1.0)


if __name__ == "__main__":
    unittest.main()
