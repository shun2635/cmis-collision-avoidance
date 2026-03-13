"""ORCA per-agent solve helpers."""

from __future__ import annotations

from cmis_ca.algorithms.orca.constraints import (
    build_agent_constraints,
    build_obstacle_constraints,
)
from cmis_ca.algorithms.orca.parameters import ORCAParameters
from cmis_ca.core.geometry import Vector2
from cmis_ca.core.neighbor_search import NeighborSearch
from cmis_ca.core.solver import solve_linear_constraints
from cmis_ca.core.world import SnapshotAgent, WorldSnapshot


def compute_orca_velocity(
    *,
    snapshot: WorldSnapshot,
    agent_index: int,
    optimization_velocity: Vector2,
    parameters: ORCAParameters,
    neighbor_search: NeighborSearch,
) -> Vector2:
    """Compute one agent's ORCA velocity for an arbitrary optimization velocity."""

    agent = _find_agent(snapshot, agent_index)
    resolved_parameters = parameters.resolve(agent.profile)
    neighbors = neighbor_search.find_neighbors(
        snapshot=snapshot,
        agent_index=agent.index,
        neighbor_dist=resolved_parameters.neighbor_dist,
        max_neighbors=resolved_parameters.max_neighbors,
        obstacle_range=(
            resolved_parameters.time_horizon_obst * agent.profile.max_speed + agent.profile.radius
        ),
    )
    obstacle_constraints = build_obstacle_constraints(
        snapshot=snapshot,
        agent_index=agent.index,
        neighbors=neighbors,
        parameters=resolved_parameters,
    )
    agent_constraints = build_agent_constraints(
        snapshot=snapshot,
        agent_index=agent.index,
        neighbors=neighbors,
        parameters=resolved_parameters,
    )
    constraints = [*obstacle_constraints, *agent_constraints]
    return solve_linear_constraints(
        constraints=constraints,
        optimization_velocity=optimization_velocity,
        max_speed=agent.profile.max_speed,
        direction_opt=False,
        protected_constraint_count=len(obstacle_constraints),
    )


def _find_agent(snapshot: WorldSnapshot, agent_index: int) -> SnapshotAgent:
    for agent in snapshot.agents:
        if agent.index == agent_index:
            return agent
    raise ValueError(f"agent index {agent_index} is not present in the snapshot")
