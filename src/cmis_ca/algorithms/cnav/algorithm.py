"""Initial CNav scaffold built on top of ORCA's per-agent solver."""

from __future__ import annotations

from dataclasses import dataclass

from cmis_ca.algorithms.cnav.coordination import ActionSelection, evaluate_action_set
from cmis_ca.algorithms.cnav.parameters import CNavParameters
from cmis_ca.algorithms.cnav.trace import CNavTraceAction, CNavTraceAgentStep
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
    action_index: int | None


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
        self._latest_step_trace: tuple[CNavTraceAgentStep, ...] = ()

    @property
    def latest_step_trace(self) -> tuple[CNavTraceAgentStep, ...]:
        return self._latest_step_trace

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
                    action_index=None,
                ),
            ).intended_velocity
            for agent in snapshot.agents
        }

        commands = []
        next_cache: dict[int, _IntentCacheEntry] = {}
        step_trace: list[CNavTraceAgentStep] = []
        for agent in snapshot.agents:
            (
                intended_velocity,
                cache_entry,
                action_selection,
                action_updated,
            ) = self._resolve_intended_velocity(
                snapshot,
                agent.index,
                communicated_intents,
            )
            next_cache[agent.index] = cache_entry
            output_velocity = compute_orca_velocity(
                snapshot=snapshot,
                agent_index=agent.index,
                optimization_velocity=intended_velocity,
                parameters=self.orca_parameters,
                neighbor_search=self.neighbor_search,
            )
            commands.append(
                AgentCommand(
                    velocity=output_velocity
                )
            )
            step_trace.append(
                CNavTraceAgentStep(
                    step_index=snapshot.step_index,
                    global_time=snapshot.global_time,
                    agent_index=agent.index,
                    agent_name=agent.name,
                    action_updated=action_updated,
                    ranked_neighbors=(
                        action_selection.ranked_neighbors if action_selection is not None else ()
                    ),
                    communicated_intended_velocity=communicated_intents[agent.index],
                    chosen_action_index=cache_entry.action_index,
                    chosen_intended_velocity=intended_velocity,
                    output_velocity=output_velocity,
                    candidate_actions=(
                        tuple(
                            CNavTraceAction(
                                action_index=evaluation.action_index,
                                intended_velocity=evaluation.intended_velocity,
                                goal_progress_reward=evaluation.goal_progress_reward,
                                constrained_reduction_reward=evaluation.constrained_reduction_reward,
                                total_reward=evaluation.total_reward,
                            )
                            for evaluation in action_selection.evaluations
                        )
                        if action_selection is not None
                        else ()
                    ),
                )
            )

        self._intent_cache = next_cache
        self._latest_step_trace = tuple(step_trace)
        return commands

    def _resolve_intended_velocity(
        self,
        snapshot: WorldSnapshot,
        agent_index: int,
        communicated_intents: dict[int, Vector2],
    ) -> tuple[Vector2, _IntentCacheEntry, ActionSelection | None, bool]:
        cache_entry = self._intent_cache.get(agent_index)
        if cache_entry is None or self._should_update_action(snapshot.global_time, cache_entry):
            action_selection = evaluate_action_set(
                snapshot=snapshot,
                agent_index=agent_index,
                communicated_intents=communicated_intents,
                parameters=self.parameters,
                orca_parameters=self.orca_parameters,
                neighbor_search=self.neighbor_search,
            )
            next_entry = _IntentCacheEntry(
                intended_velocity=action_selection.best_evaluation.intended_velocity,
                last_update_time=snapshot.global_time,
                action_index=action_selection.best_evaluation.action_index,
            )
            return action_selection.best_evaluation.intended_velocity, next_entry, action_selection, True
        return cache_entry.intended_velocity, _IntentCacheEntry(
            intended_velocity=cache_entry.intended_velocity,
            last_update_time=cache_entry.last_update_time,
            action_index=cache_entry.action_index,
        ), None, False

    def _should_update_action(self, global_time: float, entry: _IntentCacheEntry) -> bool:
        if self.parameters.update_every_step:
            return True
        return global_time - entry.last_update_time >= self.parameters.action_update_interval
