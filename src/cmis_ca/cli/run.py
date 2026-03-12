"""Minimal runnable entry point used by the CLI skeleton."""

from __future__ import annotations

import math

from cmis_ca.algorithms.registry import create_algorithm
from cmis_ca.core.agent import AgentConfig, AgentProfile
from cmis_ca.core.geometry import Vector2
from cmis_ca.core.simulation import Simulator
from cmis_ca.core.world import Scenario

DEFAULT_DEMO_STEPS = 100


def build_demo_scenario(steps: int = DEFAULT_DEMO_STEPS) -> Scenario:
    """Build a built-in lightweight circle scenario."""
    profile = AgentProfile(
        radius=0.4,
        max_speed=1.0,
        neighbor_dist=5.0,
        max_neighbors=8,
        time_horizon=5.0,
        time_horizon_obst=5.0,
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
            )
        )

    return Scenario(
        name="circle-demo",
        time_step=0.5,
        steps=steps,
        agents=tuple(agents),
    )


def run_demo(algorithm_name: str, steps: int = DEFAULT_DEMO_STEPS):
    """Run the built-in circle scenario."""
    simulator = Simulator(
        scenario=build_demo_scenario(steps=steps),
        algorithm=create_algorithm(algorithm_name),
    )
    return simulator.run()


def run_scenario_file(algorithm_name: str, scenario_path: str, steps: int | None = None):
    """Load and run a scenario file shared across algorithms."""
    from cmis_ca.io import load_scenario

    scenario = load_scenario(scenario_path)
    simulator = Simulator(
        scenario=Scenario(
            name=scenario.name,
            time_step=scenario.time_step,
            steps=scenario.steps if steps is None else steps,
            agents=scenario.agents,
            obstacles=scenario.obstacles,
        ),
        algorithm=create_algorithm(algorithm_name),
    )
    return simulator.run()
