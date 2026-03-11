"""Print qualitative metrics for the upstream Circle regression scenario."""

from __future__ import annotations

from cmis_ca.regression import run_upstream_circle_regression


def main() -> None:
    metrics = run_upstream_circle_regression()
    print(f"steps={metrics.steps_run}")
    print(f"average_radius={metrics.average_radius:.6f}")
    print(f"minimum_pair_distance={metrics.minimum_pair_distance:.6f}")
    print(f"centroid_distance={metrics.centroid_distance:.6f}")
    print(f"average_goal_distance={metrics.average_goal_distance:.6f}")
    print(f"max_speed={metrics.max_speed:.6f}")
    print(f"max_antipodal_error={metrics.max_antipodal_error:.6f}")


if __name__ == "__main__":
    main()
