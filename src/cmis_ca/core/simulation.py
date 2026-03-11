"""Common simulation loop shared across algorithms."""

from __future__ import annotations

from dataclasses import replace
from typing import TYPE_CHECKING

from cmis_ca.core.state import AgentState, SimulationResult
from cmis_ca.core.world import Scenario, SnapshotAgent, WorldSnapshot

if TYPE_CHECKING:
    from cmis_ca.algorithms.base import CollisionAvoidanceAlgorithm


class Simulator:
    """Minimal simulator that delegates decision making to an algorithm."""

    def __init__(self, scenario: Scenario, algorithm: "CollisionAvoidanceAlgorithm") -> None:
        self._scenario = scenario
        self._algorithm = algorithm
        self._step_index = 0
        self._states = [
            AgentState(
                position=config.initial_position,
                velocity=config.initial_velocity.clamp_magnitude(config.profile.max_speed),
                preferred_velocity=config.preferred_velocity,
            )
            for config in scenario.agents
        ]

    @property
    def states(self) -> tuple[AgentState, ...]:
        return tuple(self._states)

    @property
    def step_index(self) -> int:
        return self._step_index

    def snapshot(self) -> WorldSnapshot:
        agents = tuple(
            SnapshotAgent(
                index=index,
                name=config.name or f"agent_{index}",
                profile=config.profile,
                state=state,
            )
            for index, (config, state) in enumerate(zip(self._scenario.agents, self._states))
        )
        return WorldSnapshot(
            step_index=self._step_index,
            time_step=self._scenario.time_step,
            agents=agents,
            obstacles=self._scenario.obstacles,
        )

    def set_preferred_velocity(self, agent_index: int, preferred_velocity) -> None:
        current = self._states[agent_index]
        self._states[agent_index] = replace(current, preferred_velocity=preferred_velocity)

    def step(self) -> tuple[AgentState, ...]:
        commands = self._algorithm.step(self.snapshot())
        if len(commands) != len(self._states):
            raise ValueError("algorithm returned a command count that does not match the agent count")

        next_states = []
        for config, state, command in zip(self._scenario.agents, self._states, commands):
            next_velocity = command.velocity.clamp_magnitude(config.profile.max_speed)
            next_states.append(
                AgentState(
                    position=state.position + next_velocity * self._scenario.time_step,
                    velocity=next_velocity,
                    preferred_velocity=state.preferred_velocity,
                )
            )

        self._states = next_states
        self._step_index += 1
        return self.states

    def run(self, steps: int | None = None) -> SimulationResult:
        total_steps = self._scenario.steps if steps is None else steps
        history = [self.states]

        for _ in range(total_steps):
            history.append(self.step())

        return SimulationResult(
            algorithm=self._algorithm.name,
            final_states=self.states,
            history=tuple(history),
        )
