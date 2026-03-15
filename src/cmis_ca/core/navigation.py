"""Helpers for grid-based temporary goal guidance."""

from __future__ import annotations

from collections import deque

from cmis_ca.core.geometry import Vector2
from cmis_ca.core.world import NavigationGrid


GridCell = tuple[int, int]


def cell_for_position(position: Vector2, *, cell_size: float) -> GridCell:
    return (int(position.x / cell_size), int(position.y / cell_size))


def cell_center(cell: GridCell, *, cell_size: float) -> Vector2:
    return Vector2((cell[0] + 0.5) * cell_size, (cell[1] + 0.5) * cell_size)


def next_waypoint_toward_goal(
    position: Vector2,
    goal: Vector2,
    *,
    grid: NavigationGrid,
) -> Vector2:
    start = cell_for_position(position, cell_size=grid.cell_size)
    goal_cell = cell_for_position(goal, cell_size=grid.cell_size)
    path = shortest_passable_path(start, goal_cell, grid=grid)
    if not path:
        return goal
    if len(path) == 1:
        return cell_center(path[0], cell_size=grid.cell_size)
    return cell_center(path[1], cell_size=grid.cell_size)


def shortest_passable_path(
    start: GridCell,
    goal: GridCell,
    *,
    grid: NavigationGrid,
) -> tuple[GridCell, ...]:
    if not is_in_bounds(start, grid=grid) or not is_in_bounds(goal, grid=grid):
        return ()
    if not is_passable(start, grid=grid) or not is_passable(goal, grid=grid):
        return ()

    queue: deque[GridCell] = deque([start])
    parents: dict[GridCell, GridCell | None] = {start: None}
    while queue:
        current = queue.popleft()
        if current == goal:
            return _reconstruct_path(current, parents)
        for neighbor in _neighbors(current):
            if neighbor in parents:
                continue
            if not is_in_bounds(neighbor, grid=grid) or not is_passable(neighbor, grid=grid):
                continue
            parents[neighbor] = current
            queue.append(neighbor)
    return ()


def is_passable(cell: GridCell, *, grid: NavigationGrid) -> bool:
    return grid.passability[cell[0]][cell[1]] == 1


def is_in_bounds(cell: GridCell, *, grid: NavigationGrid) -> bool:
    return 0 <= cell[0] < len(grid.passability) and 0 <= cell[1] < len(grid.passability[0])


def _neighbors(cell: GridCell) -> tuple[GridCell, ...]:
    return (
        (cell[0] + 1, cell[1]),
        (cell[0] - 1, cell[1]),
        (cell[0], cell[1] + 1),
        (cell[0], cell[1] - 1),
    )


def _reconstruct_path(goal: GridCell, parents: dict[GridCell, GridCell | None]) -> tuple[GridCell, ...]:
    path: list[GridCell] = []
    current: GridCell | None = goal
    while current is not None:
        path.append(current)
        current = parents[current]
    path.reverse()
    return tuple(path)
