"""ORCA algorithm implementation."""

from __future__ import annotations

from cmis_ca.algorithms.orca.constraints import (
    build_agent_constraints,
    build_obstacle_constraints,
)
from cmis_ca.algorithms.orca.parameters import ORCAParameters
from cmis_ca.core.neighbor_search import NaiveNeighborSearch
from cmis_ca.core.solver import solve_linear_constraints
from cmis_ca.core.state import AgentCommand
from cmis_ca.core.world import WorldSnapshot


class ORCAAlgorithm:
    """Current ORCA implementation built on the staged repository structure."""

    name = "orca"

    def __init__(
        self,
        parameters: ORCAParameters | None = None,
        neighbor_search: NaiveNeighborSearch | None = None,
    ) -> None:
        self.parameters = parameters or ORCAParameters()
        self.neighbor_search = neighbor_search or NaiveNeighborSearch()

    def step(self, snapshot: WorldSnapshot) -> list[AgentCommand]:
        commands = []

        for agent in snapshot.agents:
            resolved_parameters = self.parameters.resolve(agent.profile)
            neighbors = self.neighbor_search.find_neighbors(
                snapshot=snapshot,
                agent_index=agent.index,
                neighbor_dist=resolved_parameters.neighbor_dist,
                max_neighbors=resolved_parameters.max_neighbors,
                obstacle_range=(
                    resolved_parameters.time_horizon_obst * agent.profile.max_speed
                    + agent.profile.radius
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
            commands.append(
                AgentCommand(
                    velocity=solve_linear_constraints(
                        constraints=constraints,
                        optimization_velocity=agent.state.preferred_velocity,
                        max_speed=agent.profile.max_speed,
                        direction_opt=False,
                        protected_constraint_count=len(obstacle_constraints),
                    )
                )
            )

        return commands
