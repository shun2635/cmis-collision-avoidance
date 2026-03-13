"""CNav-specific parameters."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class CNavParameters:
    """Parameters for the initial CNav action-selection scaffold."""

    coordination_factor: float = 0.8
    simulation_horizon_steps: int = 2
    action_update_interval: float = 0.2
    top_k_constrained_neighbors: int = 3
    action_speed: float = 1.5
    beta_degrees: float = 45.0

    def __post_init__(self) -> None:
        if not 0.0 <= self.coordination_factor < 1.0:
            raise ValueError("coordination_factor must be in the range [0, 1)")
        if self.simulation_horizon_steps <= 0:
            raise ValueError("simulation_horizon_steps must be positive")
        if self.action_update_interval <= 0.0:
            raise ValueError("action_update_interval must be positive")
        if self.top_k_constrained_neighbors < 0:
            raise ValueError("top_k_constrained_neighbors must be non-negative")
        if self.action_speed < 0.0:
            raise ValueError("action_speed must be non-negative")
        if self.beta_degrees < 0.0:
            raise ValueError("beta_degrees must be non-negative")
