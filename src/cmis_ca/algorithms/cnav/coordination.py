"""Coordination evaluation helpers for the initial CNav implementation."""

from __future__ import annotations

from dataclasses import dataclass

from cmis_ca.algorithms.cnav.actions import build_action_set_from_offsets, build_default_action_set
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
    actions = _build_action_set(goal_velocity, parameters=parameters)
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
        if best is None or _is_better_evaluation(
            evaluation,
            best,
            prefer_last_action_on_tie=parameters.prefer_last_action_on_tie,
        ):
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
    if parameters.reward_model == "legacy":
        return _evaluate_action_legacy(
            snapshot=snapshot,
            agent_index=agent_index,
            action_index=action_index,
            intended_velocity=intended_velocity,
            communicated_intents=communicated_intents,
            parameters=parameters,
            orca_parameters=orca_parameters,
            neighbor_search=neighbor_search,
        )

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


def _evaluate_action_legacy(
    *,
    snapshot: WorldSnapshot,
    agent_index: int,
    action_index: int,
    intended_velocity: Vector2,
    communicated_intents: dict[int, Vector2],
    parameters: CNavParameters,
    orca_parameters: ORCAParameters,
    neighbor_search: NeighborSearch,
) -> ActionEvaluation:
    agent = _find_agent(snapshot, agent_index)
    if agent.goal_position is None:
        raise ValueError("CNav requires goal_position for every agent")

    simulated_neighbor_indices = _collect_simulated_neighbor_indices(
        snapshot=snapshot,
        agent_index=agent_index,
        orca_parameters=orca_parameters,
        neighbor_search=neighbor_search,
        limit=parameters.simulate_neighbor_limit,
    )
    simulated_agents = _initialize_simulated_agents(
        snapshot=snapshot,
        agent_index=agent_index,
        ranked_neighbors=simulated_neighbor_indices,
        communicated_intents=communicated_intents,
    )

    goal_progress_sum = 0.0
    politeness_sum = 0.0
    goal_reach_step: int | None = None

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
        goal_progress = 0.0
        if goal_vector.abs_sq() > 0.0 and self_simulated.profile.max_speed > 0.0:
            goal_progress = next_velocities[agent_index].dot(goal_vector.normalized())
            goal_progress_sum += goal_progress

        if step_offset > 0:
            politeness = _legacy_politeness_reward(
                agent_index=agent_index,
                simulated_agents=simulated_agents,
                simulated_neighbor_indices=simulated_neighbor_indices,
                next_velocities=next_velocities,
                communicated_intents=communicated_intents,
                think_neighbor_limit=parameters.think_neighbor_limit,
                same_direction_neighbor_weight=parameters.same_direction_neighbor_weight,
            )
            politeness_sum += politeness

        for simulated_agent in simulated_agents.values():
            next_velocity = next_velocities[simulated_agent.index]
            simulated_agent.position = (
                simulated_agent.position + next_velocity * snapshot.time_step
            )
            simulated_agent.velocity = next_velocity

        if goal_reach_step is None and _reached_goal_position(
            simulated_agents[agent_index].position,
            simulated_agents[agent_index].goal_position,
        ):
            goal_reach_step = step_offset

    goal_progress_reward = _normalize_goal_progress(
        goal_progress_sum,
        horizon_steps=parameters.simulation_horizon_steps,
        max_speed=agent.profile.max_speed,
    )
    politeness_reward = _normalize_legacy_politeness(
        politeness_sum,
        horizon_steps=parameters.simulation_horizon_steps,
    )
    denominator = _legacy_reward_denominator(
        horizon_steps=parameters.simulation_horizon_steps,
        goal_reach_step=goal_reach_step,
    )
    total_reward = (
        ((1.0 - parameters.coordination_factor) * goal_progress_sum)
        + (parameters.coordination_factor * politeness_sum)
    ) / denominator
    return ActionEvaluation(
        action_index=action_index,
        intended_velocity=intended_velocity,
        goal_progress_reward=goal_progress_reward,
        constrained_reduction_reward=politeness_reward,
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


def _build_action_set(goal_velocity: Vector2, *, parameters: CNavParameters) -> tuple[Vector2, ...]:
    if parameters.action_speed_tiers:
        return build_action_set_from_offsets(
            goal_velocity,
            action_speeds=parameters.action_speed_tiers,
            angle_offsets_radians=parameters.action_angle_offsets_radians,
        )
    return build_default_action_set(
        goal_velocity,
        action_speed=parameters.action_speed,
        beta_degrees=parameters.beta_degrees,
    )


def _is_better_evaluation(
    candidate: ActionEvaluation,
    incumbent: ActionEvaluation,
    *,
    prefer_last_action_on_tie: bool,
) -> bool:
    if prefer_last_action_on_tie:
        return candidate.total_reward >= incumbent.total_reward
    return candidate.total_reward > incumbent.total_reward


def _collect_simulated_neighbor_indices(
    *,
    snapshot: WorldSnapshot,
    agent_index: int,
    orca_parameters: ORCAParameters,
    neighbor_search: NeighborSearch,
    limit: int | None,
) -> tuple[int, ...]:
    agent = _find_agent(snapshot, agent_index)
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
    indices = tuple(neighbor.index for neighbor in neighbors.agent_neighbors)
    if limit is None:
        return indices
    return indices[:limit]


def _legacy_politeness_reward(
    *,
    agent_index: int,
    simulated_agents: dict[int, _SimulatedAgent],
    simulated_neighbor_indices: tuple[int, ...],
    next_velocities: dict[int, Vector2],
    communicated_intents: dict[int, Vector2],
    think_neighbor_limit: int | None,
    same_direction_neighbor_weight: float,
) -> float:
    self_simulated = simulated_agents[agent_index]
    goal_vector = self_simulated.goal_position - self_simulated.position
    self_goal_distance = goal_vector.norm()
    if self_simulated.profile.max_speed <= 0.0:
        return 0.0

    politeness_terms: list[float] = []
    for neighbor_index in simulated_neighbor_indices:
        neighbor_simulated = simulated_agents[neighbor_index]
        if (self_simulated.goal_position - neighbor_simulated.position).norm() >= self_goal_distance:
            continue

        neighbor_intent = communicated_intents.get(
            neighbor_index,
            neighbor_simulated.preferred_velocity,
        )
        change = (
            neighbor_simulated.profile.max_speed
            - (neighbor_intent - next_velocities[neighbor_index]).norm()
        )
        if goal_vector.dot(neighbor_intent) >= 0.5:
            change *= same_direction_neighbor_weight
        politeness_terms.append(change / self_simulated.profile.max_speed)

    if not politeness_terms:
        return 1.0

    politeness_terms.sort()
    limit = len(politeness_terms) if think_neighbor_limit is None else min(
        think_neighbor_limit,
        len(politeness_terms),
    )
    return sum(politeness_terms[:limit]) / limit


def _normalize_legacy_politeness(politeness_sum: float, *, horizon_steps: int) -> float:
    if horizon_steps <= 1:
        return 0.0
    return politeness_sum / (horizon_steps - 1)


def _legacy_reward_denominator(*, horizon_steps: int, goal_reach_step: int | None) -> float:
    if goal_reach_step is not None:
        return float(max(1, goal_reach_step))
    return float(max(1, horizon_steps - 1))


def _reached_goal_position(position: Vector2, goal_position: Vector2) -> bool:
    return (position - goal_position).norm() < 0.15


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
