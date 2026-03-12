"""Tests for the algorithm-neutral line solver."""

from __future__ import annotations

import pytest

from cmis_ca.core.constraints import LineConstraint
from cmis_ca.core.geometry import Vector2
from cmis_ca.core.solver import choose_preferred_velocity, solve_linear_constraints


def _assert_satisfies_all(result: Vector2, constraints: list[LineConstraint]) -> None:
    for constraint in constraints:
        assert constraint.signed_distance(result) >= -1e-8


def test_choose_preferred_velocity_without_constraints_inside_circle() -> None:
    result = choose_preferred_velocity(
        constraints=[],
        preferred_velocity=Vector2(0.5, -0.25),
        max_speed=1.0,
    )

    assert result == Vector2(0.5, -0.25)


def test_choose_preferred_velocity_without_constraints_outside_circle() -> None:
    result = choose_preferred_velocity(
        constraints=[],
        preferred_velocity=Vector2(3.0, 4.0),
        max_speed=2.0,
    )

    assert result == Vector2(1.2, 1.6)


def test_choose_preferred_velocity_with_single_constraint_projects_to_boundary() -> None:
    constraints = [LineConstraint(point=Vector2(0.0, 0.0), direction=Vector2(1.0, 0.0))]

    result = choose_preferred_velocity(
        constraints=constraints,
        preferred_velocity=Vector2(1.0, -1.0),
        max_speed=2.0,
    )

    assert result == Vector2(1.0, 0.0)
    _assert_satisfies_all(result, constraints)


def test_choose_preferred_velocity_with_multiple_constraints_finds_intersection() -> None:
    constraints = [
        LineConstraint(point=Vector2(0.0, 0.0), direction=Vector2(0.0, -1.0)),
        LineConstraint(point=Vector2(0.0, 0.0), direction=Vector2(1.0, 0.0)),
    ]

    result = choose_preferred_velocity(
        constraints=constraints,
        preferred_velocity=Vector2(-1.0, -1.0),
        max_speed=3.0,
    )

    assert result == Vector2(0.0, 0.0)
    _assert_satisfies_all(result, constraints)


def test_solve_linear_constraints_can_optimize_direction() -> None:
    result = solve_linear_constraints(
        constraints=[],
        max_speed=2.0,
        optimization_velocity=Vector2(1.0, 1.0),
        direction_opt=True,
    )

    assert result.x == pytest.approx(2.0**0.5)
    assert result.y == pytest.approx(2.0**0.5)


def test_solve_linear_constraints_validates_arguments() -> None:
    with pytest.raises(ValueError):
        solve_linear_constraints([], max_speed=-1.0, optimization_velocity=Vector2())

    with pytest.raises(ValueError):
        solve_linear_constraints([], max_speed=1.0, optimization_velocity=Vector2(), protected_constraint_count=-1)

    with pytest.raises(ValueError):
        solve_linear_constraints(
            [LineConstraint(point=Vector2(), direction=Vector2(1.0, 0.0))],
            max_speed=1.0,
            optimization_velocity=Vector2(),
            protected_constraint_count=2,
        )


def test_solve_linear_constraints_matches_parallel_opposite_line_behavior() -> None:
    constraints = [
        LineConstraint(point=Vector2(0.0, 0.0), direction=Vector2(1.0, 0.0)),
        LineConstraint(point=Vector2(0.0, 0.0), direction=Vector2(-1.0, 0.0)),
    ]

    result = solve_linear_constraints(
        constraints=constraints,
        max_speed=1.0,
        optimization_velocity=Vector2(0.3, 0.4),
    )

    assert result.x == pytest.approx(0.3)
    assert result.y == pytest.approx(0.0)
    _assert_satisfies_all(result, constraints)


def test_solve_linear_constraints_respects_protected_constraint_count() -> None:
    constraints = [
        LineConstraint(point=Vector2(0.0, 0.0), direction=Vector2(-1.0, 0.0)),
        LineConstraint(point=Vector2(0.0, 0.0), direction=Vector2(0.0, 1.0)),
        LineConstraint(point=Vector2(0.3, -0.1), direction=Vector2(1.0, -1.0)),
    ]

    unprotected = solve_linear_constraints(
        constraints=constraints,
        max_speed=1.0,
        optimization_velocity=Vector2(1.0, 1.0),
        protected_constraint_count=0,
    )
    protected = solve_linear_constraints(
        constraints=constraints,
        max_speed=1.0,
        optimization_velocity=Vector2(1.0, 1.0),
        protected_constraint_count=1,
    )

    assert unprotected.x == pytest.approx(0.058578643762690494)
    assert unprotected.y == pytest.approx(0.05857864376269045)
    assert protected.x == pytest.approx(0.082842712474619)
    assert protected.y == pytest.approx(0.0)
    assert constraints[0].signed_distance(protected) >= -1e-8
    assert constraints[1].signed_distance(protected) < 0.0
