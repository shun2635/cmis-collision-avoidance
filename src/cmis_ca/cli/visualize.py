"""CLI helpers for launching simulation visualization."""

from __future__ import annotations

from cmis_ca.algorithms.registry import create_algorithm
from cmis_ca.cli.run import build_demo_scenario
from cmis_ca.core.simulation import Simulator
from cmis_ca.core.world import Scenario
from cmis_ca.visualization.pyqtgraph_viewer import launch_pyqtgraph_viewer
from cmis_ca.visualization.trace_builder import build_visualization_trace


def run_visualization(
    algorithm_name: str,
    scenario_path: str | None = None,
    *,
    steps: int | None = None,
    fps: float = 30.0,
):
    """Run a scenario and launch the default visualization backend."""

    scenario = (
        _load_scenario_for_visualization(scenario_path, steps)
        if scenario_path is not None
        else build_demo_scenario() if steps is None else build_demo_scenario(steps=steps)
    )
    simulator = Simulator(
        scenario=scenario,
        algorithm=create_algorithm(algorithm_name),
    )
    result = simulator.run()
    trace = build_visualization_trace(scenario, result)
    launch_pyqtgraph_viewer(trace, playback_fps=fps)
    return trace


def _load_scenario_for_visualization(scenario_path: str, steps: int | None) -> Scenario:
    from cmis_ca.io import load_scenario

    scenario = load_scenario(scenario_path)
    return Scenario(
        name=scenario.name,
        time_step=scenario.time_step,
        steps=scenario.steps if steps is None else steps,
        agents=scenario.agents,
        obstacles=scenario.obstacles,
    )
