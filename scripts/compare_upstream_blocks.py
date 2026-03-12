"""Print qualitative metrics for the upstream Blocks regression scenario."""

from __future__ import annotations

from cmis_ca.regression import run_upstream_blocks_regression


def main() -> None:
    metrics = run_upstream_blocks_regression()
    print(f"steps={metrics.steps_run}")
    print(f"average_goal_distance={metrics.average_goal_distance:.6f}")
    print(f"goal_distance_reduction={metrics.goal_distance_reduction:.6f}")
    print(f"minimum_pair_distance={metrics.minimum_pair_distance:.6f}")
    print(f"centroid_distance={metrics.centroid_distance:.6f}")
    print(f"max_speed={metrics.max_speed:.6f}")
    print(f"central_agent_count={metrics.central_agent_count}")


if __name__ == "__main__":
    main()
