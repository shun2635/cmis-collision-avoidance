"""Common simulation loop shared across algorithms."""

from __future__ import annotations

from dataclasses import replace
import math
from typing import TYPE_CHECKING

from cmis_ca.core.agent import AgentConfig
from cmis_ca.core.geometry import Vector2
from cmis_ca.core.navigation import cell_for_position, next_waypoint_toward_goal
from cmis_ca.core.state import AgentState, SimulationResult
from cmis_ca.core.world import Scenario, SnapshotAgent, WorldSnapshot

if TYPE_CHECKING:
    from cmis_ca.algorithms.base import CollisionAvoidanceAlgorithm


PERTURBATION_STEP_PHASE = 0.6180339887498948


class Simulator:
    """Minimal simulator that delegates decision making to an algorithm."""

    def __init__(self, scenario: Scenario, algorithm: "CollisionAvoidanceAlgorithm") -> None:
        self._scenario = scenario
        self._algorithm = algorithm
        self._step_index = 0
        self._global_time = 0.0
        self._goal_sequence_indices = [0 for _ in scenario.agents]
        self._goal_positions = [
            self._initial_goal_position(config) for config in scenario.agents
        ]
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

    @property
    def global_time(self) -> float:
        return self._global_time

    def snapshot(self) -> WorldSnapshot:
        agents = tuple(
            SnapshotAgent(
                index=index,
                name=config.name or f"agent_{index}",
                profile=config.profile,
                state=state,
                goal_position=self._goal_positions[index],
            )
            for index, (config, state) in enumerate(zip(self._scenario.agents, self._states))
        )
        return WorldSnapshot(
            step_index=self._step_index,
            global_time=self._global_time,
            time_step=self._scenario.time_step,
            agents=agents,
            obstacles=self._scenario.obstacles,
        )

    def set_preferred_velocity(self, agent_index: int, preferred_velocity) -> None:
        current = self._states[agent_index]
        self._states[agent_index] = replace(current, preferred_velocity=preferred_velocity)

    def refresh_preferred_velocities_from_goals(self) -> None:
        next_states = []
        for index, (config, state) in enumerate(zip(self._scenario.agents, self._states)):
            goal_position = self._advance_goal_sequence_if_needed(index, config, state)
            if goal_position is None or not config.auto_update_preferred_velocity_from_goal:
                next_states.append(state)
                continue

            target_position = goal_position
            if self._scenario.navigation_grid is not None:
                target_position = next_waypoint_toward_goal(
                    state.position,
                    goal_position,
                    grid=self._scenario.navigation_grid,
                )

            goal_vector = target_position - state.position
            if goal_vector.abs_sq() <= config.preferred_speed * config.preferred_speed:
                preferred_velocity = goal_vector
            else:
                preferred_velocity = goal_vector.normalized() * config.preferred_speed

            if config.preferred_velocity_perturbation_scale > 0.0:
                angle = (
                    config.preferred_velocity_perturbation_phase
                    + self._step_index * PERTURBATION_STEP_PHASE
                )
                preferred_velocity = preferred_velocity + (
                    config.preferred_velocity_perturbation_scale
                    * Vector2(math.cos(angle), math.sin(angle))
                )

            next_states.append(state.with_preferred_velocity(preferred_velocity))

        self._states = next_states

    def all_agents_reached_goals(self) -> bool:
        for index, state in enumerate(self._states):
            goal_position = self._goal_positions[index]
            if goal_position is None:
                return False
            goal_offset = state.position - goal_position
            if goal_offset.abs_sq() > self._scenario.agents[index].profile.radius * self._scenario.agents[index].profile.radius:
                return False
        return True

    def step(self) -> tuple[AgentState, ...]:
        self.refresh_preferred_velocities_from_goals()
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
        self._global_time += self._scenario.time_step
        return self.states

    def _advance_goal_sequence_if_needed(
        self,
        agent_index: int,
        config: AgentConfig,
        state: AgentState,
    ) -> Vector2 | None:
        current_goal = self._goal_positions[agent_index]
        if not config.goal_sequence or current_goal is None:
            return current_goal

        if self._should_advance_goal(state.position, current_goal, radius=config.profile.radius):
            next_index = (self._goal_sequence_indices[agent_index] + 1) % len(config.goal_sequence)
            self._goal_sequence_indices[agent_index] = next_index
            current_goal = config.goal_sequence[next_index]
            self._goal_positions[agent_index] = current_goal
        return current_goal

    def _should_advance_goal(self, position: Vector2, goal: Vector2, *, radius: float) -> bool:
        if self._scenario.navigation_grid is not None:
            return cell_for_position(
                position,
                cell_size=self._scenario.navigation_grid.cell_size,
            ) == cell_for_position(
                goal,
                cell_size=self._scenario.navigation_grid.cell_size,
            )
        return (position - goal).abs_sq() <= radius * radius

    @staticmethod
    def _initial_goal_position(config: AgentConfig) -> Vector2 | None:
        if config.goal_sequence:
            return config.goal_sequence[0]
        return config.goal_position

    def run(self, steps: int | None = None) -> SimulationResult:
        history = [self.states]
        if steps is not None:
            for _ in range(steps):
                history.append(self.step())
            return SimulationResult(
                algorithm=self._algorithm.name,
                final_states=self.states,
                history=tuple(history),
            )

        if self._scenario.stop_when_all_agents_reach_goals:
            max_steps = None if self._scenario.steps == 0 else self._scenario.steps
            steps_run = 0
            while max_steps is None or steps_run < max_steps:
                if self.all_agents_reached_goals():
                    break
                history.append(self.step())
                steps_run += 1
        else:
            for _ in range(self._scenario.steps):
                history.append(self.step())

        return SimulationResult(
            algorithm=self._algorithm.name,
            final_states=self.states,
            history=tuple(history),
        )
