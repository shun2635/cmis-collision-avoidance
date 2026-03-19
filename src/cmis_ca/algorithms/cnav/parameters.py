"""CNav-specific parameters."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Literal


CNavRewardModel = Literal["paper", "legacy"]
CNavParameterProfile = Literal[
    "paper",
    "legacy-forpaper-comparison",
    "legacy-forpaper-decision",
    "legacy-crowd-decision",
]

_LEGACY_ACTION_ANGLES = (
    0.0,
    math.pi / 4.0,
    math.pi / 2.0,
    3.0 * math.pi / 4.0,
    -3.0 * math.pi / 4.0,
    -math.pi / 2.0,
    -math.pi / 4.0,
    math.pi,
    0.0,
    math.pi / 4.0,
    math.pi / 2.0,
    3.0 * math.pi / 4.0,
    -3.0 * math.pi / 4.0,
    -math.pi / 2.0,
    -math.pi / 4.0,
    math.pi,
    0.0,
)
_LEGACY_FORPAPER_SPEEDS = (1.5,) * 8 + (0.75,) * 8 + (0.0,)
_LEGACY_CROWD_SPEEDS = (1.5,) * 8 + (0.1,) * 8 + (0.0,)


@dataclass(frozen=True)
class CNavParameters:
    """Parameters for the initial CNav action-selection scaffold."""

    coordination_factor: float = 0.8
    simulation_horizon_steps: int = 2
    action_update_interval: float = 0.2
    update_every_step: bool = False
    top_k_constrained_neighbors: int = 3
    action_speed: float = 1.5
    beta_degrees: float = 45.0
    action_speed_tiers: tuple[float, ...] = ()
    action_angle_offsets_radians: tuple[float, ...] = ()
    reward_model: CNavRewardModel = "paper"
    simulate_neighbor_limit: int | None = None
    think_neighbor_limit: int | None = None
    same_direction_neighbor_weight: float = 1.0
    prefer_last_action_on_tie: bool = False

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
        if any(speed < 0.0 for speed in self.action_speed_tiers):
            raise ValueError("action_speed_tiers must be non-negative")
        if len(self.action_speed_tiers) != len(self.action_angle_offsets_radians):
            raise ValueError(
                "action_speed_tiers and action_angle_offsets_radians must have the same length"
            )
        if self.simulate_neighbor_limit is not None and self.simulate_neighbor_limit < 0:
            raise ValueError("simulate_neighbor_limit must be non-negative")
        if self.think_neighbor_limit is not None and self.think_neighbor_limit < 0:
            raise ValueError("think_neighbor_limit must be non-negative")
        if self.same_direction_neighbor_weight < 0.0:
            raise ValueError("same_direction_neighbor_weight must be non-negative")


def create_cnav_parameters(profile: CNavParameterProfile = "paper") -> CNavParameters:
    """Create a named CNav parameter preset."""

    if profile == "paper":
        return CNavParameters()
    if profile == "legacy-forpaper-comparison":
        return CNavParameters(
            coordination_factor=0.9,
            simulation_horizon_steps=3,
            action_update_interval=0.2,
            update_every_step=True,
        )
    if profile == "legacy-forpaper-decision":
        return CNavParameters(
            coordination_factor=0.9,
            simulation_horizon_steps=3,
            action_update_interval=0.2,
            update_every_step=True,
            action_speed_tiers=_LEGACY_FORPAPER_SPEEDS,
            action_angle_offsets_radians=_LEGACY_ACTION_ANGLES,
            reward_model="legacy",
            simulate_neighbor_limit=3,
            think_neighbor_limit=1,
            same_direction_neighbor_weight=10.0,
            prefer_last_action_on_tie=True,
        )
    if profile == "legacy-crowd-decision":
        return CNavParameters(
            coordination_factor=0.9,
            simulation_horizon_steps=3,
            action_update_interval=0.2,
            update_every_step=True,
            action_speed_tiers=_LEGACY_CROWD_SPEEDS,
            action_angle_offsets_radians=_LEGACY_ACTION_ANGLES,
            reward_model="legacy",
            simulate_neighbor_limit=5,
            think_neighbor_limit=1,
            same_direction_neighbor_weight=10.0,
            prefer_last_action_on_tie=True,
        )
    raise ValueError(f"unknown CNav parameter profile: {profile}")
