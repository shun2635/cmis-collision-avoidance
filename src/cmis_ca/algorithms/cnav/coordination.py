"""Coordination evaluation helpers for the initial CNav implementation."""

from __future__ import annotations

from dataclasses import dataclass

from cmis_ca.algorithms.cnav.actions import build_default_action_set
from cmis_ca.algorithms.cnav.parameters import CNavParameters
from cmis_ca.algorithms.orca.agent_solver import compute_orca_velocity
from cmis_ca.algorithms.orca.parameters import ORCAParameters
from cmis_ca.core.agent import AgentProfile
from cmis_ca.core.geometry import Vector2
from cmis_ca.core.neighbor_search import NeighborSearch
from cmis_ca.core.state import AgentState
from cmis_ca.core.world import SnapshotAgent, WorldSnapshot


@dataclass(frozen=True)
class ActionEvaluation:
    """Reward breakdown for one candidate CNav action."""

    action_index: int
    intended_velocity: Vector2
    goal_progress_reward: float
    constrained_reduction_reward: float
    total_reward: float


@dataclass(frozen=True)
class ActionSelection:
    """Full action-selection result for one agent at one update point."""

    ranked_neighbors: tuple[int, ...]
    evaluations: tuple[ActionEvaluation, ...]
    best_evaluation: ActionEvaluation


@dataclass
class _SimulatedAgent:
    index: int
    name: str
    profile: AgentProfile
    position: Vector2
    velocity: Vector2
    goal_position: Vector2
    preferred_velocity: Vector2


def rank_constrained_neighbors(
    *,
    snapshot: WorldSnapshot,
    agent_index: int,
    communicated_intents: dict[int, Vector2],
    orca_parameters: ORCAParameters,
    neighbor_search: NeighborSearch,
) -> tuple[int, ...]:
    """Rank neighbors by how constrained their observed motion currently is."""

    agent = _find_agent(snapshot, agent_index)
    if agent.goal_position is None:
        raise ValueError("CNav requires goal_position for every agent")

    resolved_parameters = orca_parameters.resolve(agent.profile)
    neighbors = neighbor_search.find_neighbors(
        snapshot=snapshot,
        agent_index=agent.index,
        neighbor_dist=resolved_parameters.neighbor_dist,
        max_neighbors=resolved_parameters.max_neighbors,
        obstacle_range=(
            resolved_parameters.time_horizon_obst * agent.profile.max_speed + agent.profile.radius
        ),
    )
    self_goal_distance = (agent.goal_position - agent.state.position).norm()
    ranked: list[tuple[float, int]] = []
    for neighbor in neighbors.agent_neighbors:
        other = _find_agent(snapshot, neighbor.index)
        if (agent.goal_position - other.state.position).norm() >= self_goal_distance:
            continue
        intended_velocity = communicated_intents.get(other.index, other.state.preferred_velocity)
        constrained_amount = (intended_velocity - other.state.velocity).norm()
        ranked.append((constrained_amount, other.index))

    ranked.sort(key=lambda item: (-item[0], item[1]))
    return tuple(index for _, index in ranked)


def select_best_action(
    *,
    snapshot: WorldSnapshot,
    agent_index: int,
    communicated_intents: dict[int, Vector2],
    parameters: CNavParameters,
    orca_parameters: ORCAParameters,
    neighbor_search: NeighborSearch,
) -> ActionEvaluation:
    """Evaluate the action set and return the best intended velocity."""

    return evaluate_action_set(
        snapshot=snapshot,
        agent_index=agent_index,
        communicated_intents=communicated_intents,
        parameters=parameters,
        orca_parameters=orca_parameters,
        neighbor_search=neighbor_search,
    ).best_evaluation


def evaluate_action_set(
    *,
    snapshot: WorldSnapshot,
    agent_index: int,
    communicated_intents: dict[int, Vector2],
    parameters: CNavParameters,
    orca_parameters: ORCAParameters,
    neighbor_search: NeighborSearch,
) -> ActionSelection:
    """Evaluate the full action set and return all candidate rewards."""

    agent = _find_agent(snapshot, agent_index)
    goal_velocity = _goal_velocity(agent)
    actions = build_default_action_set(
        goal_velocity,
        action_speed=parameters.action_speed,
        beta_degrees=parameters.beta_degrees,
    )
    ranked_neighbors = rank_constrained_neighbors(
        snapshot=snapshot,
        agent_index=agent_index,
        communicated_intents=communicated_intents,
        orca_parameters=orca_parameters,
        neighbor_search=neighbor_search,
    )

    evaluations: list[ActionEvaluation] = []
    best: ActionEvaluation | None = None
    for action_index, action in enumerate(actions):
        evaluation = evaluate_action(
            snapshot=snapshot,
            agent_index=agent_index,
            action_index=action_index,
            intended_velocity=action,
            ranked_neighbors=ranked_neighbors,
            communicated_intents=communicated_intents,
            parameters=parameters,
            orca_parameters=orca_parameters,
            neighbor_search=neighbor_search,
        )
        evaluations.append(evaluation)
        if best is None or evaluation.total_reward > best.total_reward:
            best = evaluation

    if best is None:
        raise ValueError("CNav action set must contain at least one action")
    return ActionSelection(
        ranked_neighbors=ranked_neighbors,
        evaluations=tuple(evaluations),
        best_evaluation=best,
    )


def evaluate_action(
    *,
    snapshot: WorldSnapshot,
    agent_index: int,
    action_index: int = -1,
    intended_velocity: Vector2,
    ranked_neighbors: tuple[int, ...],
    communicated_intents: dict[int, Vector2],
    parameters: CNavParameters,
    orca_parameters: ORCAParameters,
    neighbor_search: NeighborSearch,
) -> ActionEvaluation:
    """Simulate one candidate action for a short horizon and score it."""

    agent = _find_agent(snapshot, agent_index)
    if agent.goal_position is None:
        raise ValueError("CNav requires goal_position for every agent")

    simulated_agents = _initialize_simulated_agents(
        snapshot=snapshot,
        agent_index=agent_index,
        ranked_neighbors=ranked_neighbors,
        communicated_intents=communicated_intents,
    )
    top_ranked = tuple(ranked_neighbors[: parameters.top_k_constrained_neighbors])
    goal_progress_sum = 0.0
    constrained_reduction_sum = 0.0

    for step_offset in range(parameters.simulation_horizon_steps):
        simulated_snapshot = _build_simulated_snapshot(
            original_snapshot=snapshot,
            simulated_agents=simulated_agents,
            step_offset=step_offset,
        )
        next_velocities: dict[int, Vector2] = {}
        for simulated_agent in simulated_agents.values():
            optimization_velocity = (
                intended_velocity
                if simulated_agent.index == agent_index
                else communicated_intents.get(
                    simulated_agent.index,
                    simulated_agent.preferred_velocity,
                )
            )
            next_velocities[simulated_agent.index] = compute_orca_velocity(
                snapshot=simulated_snapshot,
                agent_index=simulated_agent.index,
                optimization_velocity=optimization_velocity,
                parameters=orca_parameters,
                neighbor_search=neighbor_search,
            )

        self_simulated = simulated_agents[agent_index]
        goal_vector = self_simulated.goal_position - self_simulated.position
        if goal_vector.abs_sq() > 0.0 and self_simulated.profile.max_speed > 0.0:
            goal_progress_sum += next_velocities[agent_index].dot(goal_vector.normalized())

        if step_offset > 0:
            for neighbor_index in top_ranked:
                if neighbor_index not in simulated_agents:
                    continue
                neighbor_simulated = simulated_agents[neighbor_index]
                neighbor_intent = communicated_intents.get(
                    neighbor_index,
                    neighbor_simulated.preferred_velocity,
                )
                constrained_reduction_sum += (
                    neighbor_simulated.profile.max_speed
                    - (neighbor_intent - next_velocities[neighbor_index]).norm()
                )

        for simulated_agent in simulated_agents.values():
            next_velocity = next_velocities[simulated_agent.index]
            simulated_agent.position = (
                simulated_agent.position + next_velocity * snapshot.time_step
            )
            simulated_agent.velocity = next_velocity

    goal_progress_reward = _normalize_goal_progress(
        goal_progress_sum,
        horizon_steps=parameters.simulation_horizon_steps,
        max_speed=agent.profile.max_speed,
    )
    constrained_reduction_reward = _normalize_constrained_reduction(
        constrained_reduction_sum,
        horizon_steps=parameters.simulation_horizon_steps,
        num_ranked=len(top_ranked),
        max_speed=agent.profile.max_speed,
    )
    total_reward = (
        (1.0 - parameters.coordination_factor) * goal_progress_reward
        + parameters.coordination_factor * constrained_reduction_reward
    )
    return ActionEvaluation(
        action_index=action_index,
        intended_velocity=intended_velocity,
        goal_progress_reward=goal_progress_reward,
        constrained_reduction_reward=constrained_reduction_reward,
        total_reward=total_reward,
    )


def _initialize_simulated_agents(
    *,
    snapshot: WorldSnapshot,
    agent_index: int,
    ranked_neighbors: tuple[int, ...],
    communicated_intents: dict[int, Vector2],
) -> dict[int, _SimulatedAgent]:
    agents: dict[int, _SimulatedAgent] = {}
    for index in (agent_index, *ranked_neighbors):
        agent = _find_agent(snapshot, index)
        if agent.goal_position is None:
            raise ValueError("CNav requires goal_position for every agent")
        agents[index] = _SimulatedAgent(
            index=agent.index,
            name=agent.name,
            profile=agent.profile,
            position=agent.state.position,
            velocity=agent.state.velocity,
            goal_position=agent.goal_position,
            preferred_velocity=communicated_intents.get(agent.index, agent.state.preferred_velocity),
        )
    return agents


def _build_simulated_snapshot(
    *,
    original_snapshot: WorldSnapshot,
    simulated_agents: dict[int, _SimulatedAgent],
    step_offset: int,
) -> WorldSnapshot:
    return WorldSnapshot(
        step_index=original_snapshot.step_index + step_offset,
        global_time=original_snapshot.global_time + step_offset * original_snapshot.time_step,
        time_step=original_snapshot.time_step,
        agents=tuple(
            SnapshotAgent(
                index=agent.index,
                name=agent.name,
                profile=agent.profile,
                state=_agent_state(agent),
                goal_position=agent.goal_position,
            )
            for agent in simulated_agents.values()
        ),
        obstacles=original_snapshot.obstacles,
    )


def _agent_state(agent: _SimulatedAgent) -> AgentState:
    return AgentState(
        position=agent.position,
        velocity=agent.velocity,
        preferred_velocity=agent.preferred_velocity,
    )


def _normalize_goal_progress(goal_progress_sum: float, *, horizon_steps: int, max_speed: float) -> float:
    if horizon_steps <= 0 or max_speed <= 0.0:
        return 0.0
    return goal_progress_sum / (horizon_steps * max_speed)


def _normalize_constrained_reduction(
    constrained_reduction_sum: float,
    *,
    horizon_steps: int,
    num_ranked: int,
    max_speed: float,
) -> float:
    if horizon_steps <= 1 or num_ranked == 0 or max_speed <= 0.0:
        return 0.0
    return constrained_reduction_sum / ((horizon_steps - 1) * num_ranked * max_speed)


def _goal_velocity(agent: SnapshotAgent) -> Vector2:
    if agent.goal_position is None:
        raise ValueError("CNav requires goal_position for every agent")
    return agent.goal_position - agent.state.position


def _find_agent(snapshot: WorldSnapshot, agent_index: int) -> SnapshotAgent:
    for agent in snapshot.agents:
        if agent.index == agent_index:
            return agent
    raise ValueError(f"agent index {agent_index} is not present in the snapshot")
