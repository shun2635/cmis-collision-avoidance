"""Minimal runnable entry point used by the CLI skeleton."""

from __future__ import annotations

from cmis_ca.algorithms.registry import create_algorithm
from cmis_ca.core.agent import AgentConfig, AgentProfile
from cmis_ca.core.geometry import Vector2
from cmis_ca.core.simulation import Simulator
from cmis_ca.core.world import Scenario


def run_demo(algorithm_name: str, steps: int):
    """Run a built-in one-agent scenario for smoke testing the package layout."""
    scenario = Scenario(
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

    simulator = Simulator(scenario=scenario, algorithm=create_algorithm(algorithm_name))
    return simulator.run()
