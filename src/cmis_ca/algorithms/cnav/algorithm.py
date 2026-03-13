"""Initial CNav scaffold built on top of ORCA's per-agent solver."""

from __future__ import annotations

from dataclasses import dataclass

from cmis_ca.algorithms.cnav.actions import build_default_action_set
from cmis_ca.algorithms.cnav.parameters import CNavParameters
from cmis_ca.algorithms.orca.agent_solver import compute_orca_velocity
from cmis_ca.algorithms.orca.parameters import ORCAParameters
from cmis_ca.core.geometry import Vector2
from cmis_ca.core.neighbor_search import NaiveNeighborSearch, NeighborSearch
from cmis_ca.core.state import AgentCommand
from cmis_ca.core.world import SnapshotAgent, WorldSnapshot


@dataclass
class _IntentCacheEntry:
    intended_velocity: Vector2
    last_update_time: float


class CNavAlgorithm:
    """Initial CNav scaffold with action model and intended velocity cache."""

    name = "cnav"

    def __init__(
        self,
        parameters: CNavParameters | None = None,
        orca_parameters: ORCAParameters | None = None,
        neighbor_search: NeighborSearch | None = None,
    ) -> None:
        self.parameters = parameters or CNavParameters()
        self.orca_parameters = orca_parameters or ORCAParameters()
        self.neighbor_search = neighbor_search or NaiveNeighborSearch()
        self._intent_cache: dict[int, _IntentCacheEntry] = {}

    def step(self, snapshot: WorldSnapshot) -> list[AgentCommand]:
        active_indices = {agent.index for agent in snapshot.agents}
        self._intent_cache = {
            agent_index: entry
            for agent_index, entry in self._intent_cache.items()
            if agent_index in active_indices
        }

        commands = []
        for agent in snapshot.agents:
            intended_velocity = self._resolve_intended_velocity(snapshot, agent.index)
            commands.append(
                AgentCommand(
                    velocity=compute_orca_velocity(
                        snapshot=snapshot,
                        agent_index=agent.index,
                        optimization_velocity=intended_velocity,
                        parameters=self.orca_parameters,
                        neighbor_search=self.neighbor_search,
                    )
                )
            )

        return commands

    def _resolve_intended_velocity(self, snapshot: WorldSnapshot, agent_index: int):
        agent = _find_agent(snapshot, agent_index)
        cache_entry = self._intent_cache.get(agent_index)
        if cache_entry is None or self._should_update_action(snapshot.global_time, cache_entry):
            intended_velocity = self._select_intended_velocity(agent.state.preferred_velocity)
            self._intent_cache[agent_index] = _IntentCacheEntry(
                intended_velocity=intended_velocity,
                last_update_time=snapshot.global_time,
            )
            return intended_velocity
        return cache_entry.intended_velocity

    def _should_update_action(self, global_time: float, entry: _IntentCacheEntry) -> bool:
        return global_time - entry.last_update_time >= self.parameters.action_update_interval

    def _select_intended_velocity(self, goal_velocity: Vector2) -> Vector2:
        actions = build_default_action_set(
            goal_velocity,
            action_speed=self.parameters.action_speed,
            beta_degrees=self.parameters.beta_degrees,
        )
        return actions[0]


def _find_agent(snapshot: WorldSnapshot, agent_index: int) -> SnapshotAgent:
    for agent in snapshot.agents:
        if agent.index == agent_index:
            return agent
    raise ValueError(f"agent index {agent_index} is not present in the snapshot")
