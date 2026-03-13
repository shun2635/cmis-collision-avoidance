"""ORCA algorithm implementation."""

from __future__ import annotations

from cmis_ca.algorithms.orca.agent_solver import compute_orca_velocity
from cmis_ca.algorithms.orca.parameters import ORCAParameters
from cmis_ca.core.neighbor_search import NaiveNeighborSearch
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
            commands.append(
                AgentCommand(
                    velocity=compute_orca_velocity(
                        snapshot=snapshot,
                        agent_index=agent.index,
                        optimization_velocity=agent.state.preferred_velocity,
                        parameters=self.parameters,
                        neighbor_search=self.neighbor_search,
                    )
                )
            )

        return commands
