"""Minimal runnable entry point used by the CLI skeleton."""

from __future__ import annotations

import math

from cmis_ca.algorithms.cnav import (
    build_cnav_parameters_from_overrides,
    create_cnav_parameters,
    create_mystyle_driver_settings,
)
from cmis_ca.algorithms.registry import create_algorithm
from cmis_ca.core.agent import AgentConfig, AgentProfile
from cmis_ca.core.geometry import Vector2
from cmis_ca.core.simulation import Simulator
from cmis_ca.core.world import Scenario

DEFAULT_DEMO_STEPS = 1000
DEFAULT_DEMO_PERTURBATION_SCALE = 1.0e-3
GOLDEN_ANGLE = 2.39996322972865332


def build_demo_scenario(steps: int = DEFAULT_DEMO_STEPS) -> Scenario:
    """Build a built-in lightweight circle scenario."""
    profile = AgentProfile(
        radius=0.4,
        max_speed=1.0,
        neighbor_dist=5.0,
        max_neighbors=8,
        time_horizon=0.3,
        time_horizon_obst=0.3,
    )
    agent_count = 8
    spawn_radius = 8.0
    agents = []
    for index in range(agent_count):
        angle = 2.0 * math.pi * index / agent_count
        position = Vector2(spawn_radius * math.cos(angle), spawn_radius * math.sin(angle))
        agents.append(
            AgentConfig(
                name=f"agent_{index}",
                profile=profile,
                initial_position=position,
                initial_velocity=Vector2(),
                preferred_velocity=Vector2(),
                goal_position=-position,
                preferred_speed=1.0,
                preferred_velocity_perturbation_scale=DEFAULT_DEMO_PERTURBATION_SCALE,
                preferred_velocity_perturbation_phase=index * GOLDEN_ANGLE,
            )
        )

    return Scenario(
        name="circle-demo",
        time_step=0.5,
        steps=steps,
        agents=tuple(agents),
    )


def build_algorithm_instance(
    algorithm_name: str,
    *,
    scenario: Scenario | None = None,
    cnav_profile: str | None = None,
    cnav_mystyle_driver: str | None = None,
):
    """Build one algorithm instance with optional CNav-specific profiles."""

    if algorithm_name != "cnav":
        return create_algorithm(algorithm_name)

    if cnav_mystyle_driver is not None:
        settings = create_mystyle_driver_settings(cnav_mystyle_driver)
        return create_algorithm("cnav", parameters=settings.cnav_parameters)
    if cnav_profile is not None:
        return create_algorithm("cnav", parameters=create_cnav_parameters(cnav_profile))
    if scenario is not None and "cnav" in scenario.algorithm_overrides:
        return create_algorithm(
            "cnav",
            parameters=build_cnav_parameters_from_overrides(
                scenario.algorithm_overrides["cnav"]
            ),
        )
    return create_algorithm("cnav")


def apply_cnav_mystyle_driver_to_scenario(scenario: Scenario, driver: str | None) -> Scenario:
    """Override time_step and agent profile to match one legacy MyStyle driver."""

    if driver is None:
        return scenario

    settings = create_mystyle_driver_settings(driver)
    agents = tuple(
        AgentConfig(
            name=agent.name,
            profile=settings.agent_profile,
            initial_position=agent.initial_position,
            initial_velocity=agent.initial_velocity,
            preferred_velocity=agent.preferred_velocity,
            goal_position=agent.goal_position,
            goal_sequence=agent.goal_sequence,
            preferred_speed=settings.agent_profile.max_speed,
            auto_update_preferred_velocity_from_goal=agent.auto_update_preferred_velocity_from_goal,
            preferred_velocity_perturbation_scale=(
                settings.preferred_velocity_perturbation_scale
            ),
            preferred_velocity_perturbation_phase=agent.preferred_velocity_perturbation_phase,
        )
        for agent in scenario.agents
    )
    return Scenario(
        name=scenario.name,
        time_step=settings.time_step,
        steps=scenario.steps,
        stop_when_all_agents_reach_goals=scenario.stop_when_all_agents_reach_goals,
        agents=agents,
        obstacles=scenario.obstacles,
        navigation_grid=scenario.navigation_grid,
        algorithm_overrides=scenario.algorithm_overrides,
    )


def run_demo(
    algorithm_name: str,
    steps: int = DEFAULT_DEMO_STEPS,
    *,
    cnav_profile: str | None = None,
    cnav_mystyle_driver: str | None = None,
):
    """Run the built-in circle scenario."""
    scenario = apply_cnav_mystyle_driver_to_scenario(
        build_demo_scenario(steps=steps),
        cnav_mystyle_driver,
    )
    simulator = Simulator(
        scenario=scenario,
        algorithm=build_algorithm_instance(
            algorithm_name,
            scenario=scenario,
            cnav_profile=cnav_profile,
            cnav_mystyle_driver=cnav_mystyle_driver,
        ),
    )
    return simulator.run()


def run_scenario_file(
    algorithm_name: str,
    scenario_path: str,
    steps: int | None = None,
    *,
    cnav_profile: str | None = None,
    cnav_mystyle_driver: str | None = None,
):
    """Load and run a scenario file shared across algorithms."""
    from cmis_ca.io import load_scenario

    scenario = apply_cnav_mystyle_driver_to_scenario(
        load_scenario(scenario_path),
        cnav_mystyle_driver,
    )
    simulator = Simulator(
        scenario=scenario,
        algorithm=build_algorithm_instance(
            algorithm_name,
            scenario=scenario,
            cnav_profile=cnav_profile,
            cnav_mystyle_driver=cnav_mystyle_driver,
        ),
    )
    return simulator.run(steps=steps)
