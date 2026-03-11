"""Minimal runnable entry point used by the CLI skeleton."""

from __future__ import annotations

from cmis_ca.algorithms.registry import create_algorithm
from cmis_ca.core.agent import AgentConfig, AgentProfile
from cmis_ca.core.geometry import Vector2
from cmis_ca.core.simulation import Simulator
from cmis_ca.core.world import Scenario


def build_demo_scenario(steps: int = 1) -> Scenario:
    """Build a built-in one-agent scenario for smoke testing the package layout."""
    return Scenario(
        name="minimal-demo",
        time_step=0.5,
        steps=steps,
        agents=(
            AgentConfig(
                name="agent_0",
                profile=AgentProfile(radius=0.4, max_speed=1.0),
                initial_position=Vector2(0.0, 0.0),
                initial_velocity=Vector2(0.0, 0.0),
                preferred_velocity=Vector2(1.0, 0.0),
            ),
        ),
    )


def run_demo(algorithm_name: str, steps: int = 1):
    """Run the built-in one-agent scenario."""
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
