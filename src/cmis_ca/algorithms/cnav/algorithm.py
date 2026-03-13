"""Initial CNav scaffold built on top of ORCA's per-agent solver."""

from __future__ import annotations

from dataclasses import dataclass

from cmis_ca.algorithms.cnav.coordination import select_best_action
from cmis_ca.algorithms.cnav.parameters import CNavParameters
from cmis_ca.algorithms.orca.agent_solver import compute_orca_velocity
from cmis_ca.algorithms.orca.parameters import ORCAParameters
from cmis_ca.core.geometry import Vector2
from cmis_ca.core.neighbor_search import NaiveNeighborSearch, NeighborSearch
from cmis_ca.core.state import AgentCommand
from cmis_ca.core.world import WorldSnapshot


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
        communicated_intents = {
            agent.index: self._intent_cache.get(
                agent.index,
                _IntentCacheEntry(
                    intended_velocity=agent.state.preferred_velocity,
                    last_update_time=snapshot.global_time - self.parameters.action_update_interval,
                ),
            ).intended_velocity
            for agent in snapshot.agents
        }

        commands = []
        next_cache: dict[int, _IntentCacheEntry] = {}
        for agent in snapshot.agents:
            intended_velocity, cache_entry = self._resolve_intended_velocity(
                snapshot,
                agent.index,
                communicated_intents,
            )
            next_cache[agent.index] = cache_entry
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

        self._intent_cache = next_cache
        return commands

    def _resolve_intended_velocity(
        self,
        snapshot: WorldSnapshot,
        agent_index: int,
        communicated_intents: dict[int, Vector2],
    ) -> tuple[Vector2, _IntentCacheEntry]:
        cache_entry = self._intent_cache.get(agent_index)
        if cache_entry is None or self._should_update_action(snapshot.global_time, cache_entry):
            evaluation = select_best_action(
                snapshot=snapshot,
                agent_index=agent_index,
                communicated_intents=communicated_intents,
                parameters=self.parameters,
                orca_parameters=self.orca_parameters,
                neighbor_search=self.neighbor_search,
            )
            next_entry = _IntentCacheEntry(
                intended_velocity=evaluation.intended_velocity,
                last_update_time=snapshot.global_time,
            )
            return evaluation.intended_velocity, next_entry
        return cache_entry.intended_velocity, _IntentCacheEntry(
            intended_velocity=cache_entry.intended_velocity,
            last_update_time=cache_entry.last_update_time,
        )

    def _should_update_action(self, global_time: float, entry: _IntentCacheEntry) -> bool:
        return global_time - entry.last_update_time >= self.parameters.action_update_interval
