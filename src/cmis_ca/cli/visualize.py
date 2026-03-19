"""CLI helpers for launching simulation visualization."""

from __future__ import annotations

from cmis_ca.cli.run import (
    apply_cnav_mystyle_driver_to_scenario,
    build_algorithm_instance,
    build_demo_scenario,
)
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
    cnav_profile: str | None = None,
    cnav_mystyle_driver: str | None = None,
):
    """Run a scenario and launch the default visualization backend."""

    scenario = apply_cnav_mystyle_driver_to_scenario(
        (
            _load_scenario_for_visualization(scenario_path, steps)
            if scenario_path is not None
            else build_demo_scenario() if steps is None else build_demo_scenario(steps=steps)
        ),
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
    result = simulator.run()
    trace = build_visualization_trace(scenario, result)
    launch_pyqtgraph_viewer(trace, playback_fps=fps)
    return trace


def _load_scenario_for_visualization(scenario_path: str, steps: int | None) -> Scenario:
    from cmis_ca.io import load_scenario

    scenario = load_scenario(scenario_path)
    if steps is None:
        return scenario
    return Scenario(
        name=scenario.name,
        time_step=scenario.time_step,
        steps=steps,
        stop_when_all_agents_reach_goals=False,
        agents=scenario.agents,
        obstacles=scenario.obstacles,
        navigation_grid=scenario.navigation_grid,
        algorithm_overrides=scenario.algorithm_overrides,
    )
