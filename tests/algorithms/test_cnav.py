"""Tests for the initial CNav scaffold."""

from __future__ import annotations

import pytest

from cmis_ca.algorithms.cnav.actions import build_default_action_set
from cmis_ca.algorithms.cnav.algorithm import CNavAlgorithm
from cmis_ca.algorithms.cnav.coordination import (
    evaluate_action,
    rank_constrained_neighbors,
    select_best_action,
)
from cmis_ca.algorithms.cnav.parameters import CNavParameters, create_cnav_parameters
from cmis_ca.algorithms.orca.parameters import ORCAParameters
from cmis_ca.core.agent import AgentConfig, AgentProfile
from cmis_ca.core.geometry import Vector2
from cmis_ca.core.neighbor_search import NaiveNeighborSearch
from cmis_ca.core.simulation import Simulator
from cmis_ca.core.state import AgentState
from cmis_ca.core.world import Scenario, SnapshotAgent, WorldSnapshot


def _snapshot_agent(
    index: int,
    *,
    position: Vector2,
    velocity: Vector2,
    preferred_velocity: Vector2,
    goal_position: Vector2,
) -> SnapshotAgent:
    return SnapshotAgent(
        index=index,
        name=f"agent_{index}",
        profile=AgentProfile(radius=0.4, max_speed=1.0),
        state=AgentState(
            position=position,
            velocity=velocity,
            preferred_velocity=preferred_velocity,
        ),
        goal_position=goal_position,
    )


def test_build_default_action_set_uses_paper_order() -> None:
    actions = build_default_action_set(
        Vector2(1.0, 0.0),
        action_speed=1.5,
        beta_degrees=45.0,
    )

    assert len(actions) == 8
    assert actions[0].x == pytest.approx(1.5)
    assert actions[0].y == pytest.approx(0.0)
    assert actions[1].x == pytest.approx(1.5 / 2**0.5)
    assert actions[1].y == pytest.approx(1.5 / 2**0.5)
    assert actions[2].x == pytest.approx(1.5 / 2**0.5)
    assert actions[2].y == pytest.approx(-(1.5 / 2**0.5))
    assert actions[3].x == pytest.approx(0.0, abs=1e-9)
    assert actions[3].y == pytest.approx(1.5)
    assert actions[5].x == pytest.approx(-1.5)
    assert actions[5].y == pytest.approx(0.0, abs=1e-9)


def test_cnav_parameters_validate_initial_defaults() -> None:
    parameters = CNavParameters()

    assert parameters.coordination_factor == pytest.approx(0.8)
    assert parameters.simulation_horizon_steps == 2
    assert parameters.action_update_interval == pytest.approx(0.2)
    assert parameters.update_every_step is False
    assert parameters.action_speed == pytest.approx(1.5)
    assert parameters.beta_degrees == pytest.approx(45.0)


def test_create_cnav_parameters_supports_legacy_forpaper_comparison_profile() -> None:
    parameters = create_cnav_parameters("legacy-forpaper-comparison")

    assert parameters.coordination_factor == pytest.approx(0.9)
    assert parameters.simulation_horizon_steps == 3
    assert parameters.action_update_interval == pytest.approx(0.2)
    assert parameters.update_every_step is True


def test_cnav_algorithm_caches_goal_directed_intended_velocity_until_update_interval() -> None:
    scenario = Scenario(
        name="single-agent",
        time_step=0.1,
        steps=1,
        agents=(
            AgentConfig(
                name="agent_0",
                profile=AgentProfile(radius=0.4, max_speed=1.0),
                initial_position=Vector2(0.0, 0.0),
                initial_velocity=Vector2(0.0, 0.0),
                preferred_velocity=Vector2(1.0, 0.0),
                goal_position=Vector2(10.0, 0.0),
            ),
        ),
    )

    algorithm = CNavAlgorithm(parameters=CNavParameters(action_update_interval=0.2))
    simulator = Simulator(scenario=scenario, algorithm=algorithm)

    first_commands = algorithm.step(simulator.snapshot())
    first_intended_velocity = algorithm._intent_cache[0].intended_velocity

    algorithm._intent_cache[0].intended_velocity = Vector2(-1.5, 0.0)
    second_commands = algorithm.step(simulator.snapshot())

    assert first_intended_velocity == Vector2(1.5, 0.0)
    assert algorithm._intent_cache[0].intended_velocity == Vector2(-1.5, 0.0)
    assert first_commands[0].velocity.x == pytest.approx(1.0)
    assert second_commands[0].velocity.x == pytest.approx(-1.0)


def test_cnav_algorithm_refreshes_intended_velocity_after_update_interval() -> None:
    scenario = Scenario(
        name="single-agent",
        time_step=0.1,
        steps=1,
        agents=(
            AgentConfig(
                name="agent_0",
                profile=AgentProfile(radius=0.4, max_speed=1.0),
                initial_position=Vector2(0.0, 0.0),
                initial_velocity=Vector2(0.0, 0.0),
                preferred_velocity=Vector2(1.0, 0.0),
                goal_position=Vector2(10.0, 0.0),
            ),
        ),
    )

    algorithm = CNavAlgorithm(parameters=CNavParameters(action_update_interval=0.2))
    simulator = Simulator(scenario=scenario, algorithm=algorithm)

    algorithm.step(simulator.snapshot())
    algorithm._intent_cache[0].intended_velocity = Vector2(-1.5, 0.0)
    simulator.step()
    simulator.step()
    simulator.step()

    assert algorithm._intent_cache[0].intended_velocity == Vector2(1.5, 0.0)


def test_cnav_algorithm_can_force_action_update_every_step() -> None:
    scenario = Scenario(
        name="single-agent",
        time_step=0.1,
        steps=1,
        agents=(
            AgentConfig(
                name="agent_0",
                profile=AgentProfile(radius=0.4, max_speed=1.0),
                initial_position=Vector2(0.0, 0.0),
                initial_velocity=Vector2(0.0, 0.0),
                preferred_velocity=Vector2(1.0, 0.0),
                goal_position=Vector2(10.0, 0.0),
            ),
        ),
    )

    algorithm = CNavAlgorithm(parameters=CNavParameters(update_every_step=True))
    simulator = Simulator(scenario=scenario, algorithm=algorithm)

    algorithm.step(simulator.snapshot())
    algorithm._intent_cache[0].intended_velocity = Vector2(-1.5, 0.0)
    simulator.step()

    assert algorithm._intent_cache[0].intended_velocity == Vector2(1.5, 0.0)


def test_rank_constrained_neighbors_filters_by_goal_distance_and_orders_by_constraint() -> None:
    snapshot = WorldSnapshot(
        step_index=0,
        global_time=0.0,
        time_step=0.1,
        agents=(
            _snapshot_agent(
                0,
                position=Vector2(0.0, 0.0),
                velocity=Vector2(0.0, 0.0),
                preferred_velocity=Vector2(1.0, 0.0),
                goal_position=Vector2(10.0, 0.0),
            ),
            _snapshot_agent(
                1,
                position=Vector2(4.0, 0.0),
                velocity=Vector2(0.0, 0.0),
                preferred_velocity=Vector2(1.0, 0.0),
                goal_position=Vector2(10.0, 0.0),
            ),
            _snapshot_agent(
                2,
                position=Vector2(4.5, 0.0),
                velocity=Vector2(0.0, 0.0),
                preferred_velocity=Vector2(0.1, 0.0),
                goal_position=Vector2(10.0, 0.0),
            ),
            _snapshot_agent(
                3,
                position=Vector2(-3.0, 0.0),
                velocity=Vector2(0.0, 0.0),
                preferred_velocity=Vector2(3.0, 0.0),
                goal_position=Vector2(10.0, 0.0),
            ),
        ),
    )

    ranked = rank_constrained_neighbors(
        snapshot=snapshot,
        agent_index=0,
        communicated_intents={
            1: Vector2(1.0, 0.0),
            2: Vector2(0.1, 0.0),
            3: Vector2(3.0, 0.0),
        },
        orca_parameters=ORCAParameters(),
        neighbor_search=NaiveNeighborSearch(),
    )

    assert ranked == (1, 2)


def test_evaluate_action_scores_goal_progress_and_constrained_reduction() -> None:
    snapshot = WorldSnapshot(
        step_index=0,
        global_time=0.0,
        time_step=0.1,
        agents=(
            _snapshot_agent(
                0,
                position=Vector2(0.0, 0.0),
                velocity=Vector2(0.0, 0.0),
                preferred_velocity=Vector2(1.0, 0.0),
                goal_position=Vector2(10.0, 0.0),
            ),
            _snapshot_agent(
                1,
                position=Vector2(2.0, 0.0),
                velocity=Vector2(0.0, 0.0),
                preferred_velocity=Vector2(1.0, 0.0),
                goal_position=Vector2(10.0, 0.0),
            ),
        ),
    )

    evaluation = evaluate_action(
        snapshot=snapshot,
        agent_index=0,
        intended_velocity=Vector2(1.5, 0.0),
        ranked_neighbors=(1,),
        communicated_intents={0: Vector2(1.0, 0.0), 1: Vector2(1.0, 0.0)},
        parameters=CNavParameters(coordination_factor=0.8, top_k_constrained_neighbors=1),
        orca_parameters=ORCAParameters(),
        neighbor_search=NaiveNeighborSearch(),
    )

    assert evaluation.goal_progress_reward > 0.0
    assert 0.0 <= evaluation.constrained_reduction_reward <= 1.0
    assert evaluation.total_reward == pytest.approx(
        0.2 * evaluation.goal_progress_reward + 0.8 * evaluation.constrained_reduction_reward
    )


def test_select_best_action_prefers_polite_action_in_same_goal_queue() -> None:
    snapshot = WorldSnapshot(
        step_index=0,
        global_time=0.0,
        time_step=0.1,
        agents=(
            _snapshot_agent(
                0,
                position=Vector2(-1.0, 0.0),
                velocity=Vector2(0.0, 0.0),
                preferred_velocity=Vector2(1.0, 0.0),
                goal_position=Vector2(10.0, 0.0),
            ),
            _snapshot_agent(
                1,
                position=Vector2(0.0, 0.0),
                velocity=Vector2(0.0, 0.0),
                preferred_velocity=Vector2(1.0, 0.0),
                goal_position=Vector2(10.0, 0.0),
            ),
        ),
    )

    evaluation = select_best_action(
        snapshot=snapshot,
        agent_index=0,
        communicated_intents={0: Vector2(1.0, 0.0), 1: Vector2(1.0, 0.0)},
        parameters=CNavParameters(coordination_factor=1.0 - 1e-6, top_k_constrained_neighbors=1),
        orca_parameters=ORCAParameters(),
        neighbor_search=NaiveNeighborSearch(),
    )

    assert evaluation.intended_velocity != Vector2(1.5, 0.0)
