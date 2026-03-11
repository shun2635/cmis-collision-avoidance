"""Initial ORCA algorithm skeleton."""

from __future__ import annotations

from cmis_ca.algorithms.orca.constraints import (
    build_agent_constraints,
    build_obstacle_constraints,
)
from cmis_ca.algorithms.orca.parameters import ORCAParameters
from cmis_ca.core.neighbor_search import NaiveNeighborSearch
from cmis_ca.core.solver import choose_preferred_velocity
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
            neighbors = self.neighbor_search.find_neighbors(
                snapshot=snapshot,
                agent_index=agent.index,
                neighbor_dist=self.parameters.neighbor_dist,
                max_neighbors=self.parameters.max_neighbors,
            )
            constraints = []
            constraints.extend(
                build_obstacle_constraints(
                    snapshot=snapshot,
                    agent_index=agent.index,
                    neighbors=neighbors,
                    parameters=self.parameters,
                )
            )
            constraints.extend(
                build_agent_constraints(
                    snapshot=snapshot,
                    agent_index=agent.index,
                    neighbors=neighbors,
                    parameters=self.parameters,
                )
            )
            commands.append(
                AgentCommand(
                    velocity=choose_preferred_velocity(
                        constraints=constraints,
                        preferred_velocity=agent.state.preferred_velocity,
                        max_speed=agent.profile.max_speed,
                    )
                )
            )

        return commands
