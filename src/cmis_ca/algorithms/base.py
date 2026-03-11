"""Common algorithm interface."""

from __future__ import annotations

from typing import Protocol

from cmis_ca.core.state import AgentCommand
from cmis_ca.core.world import WorldSnapshot


class CollisionAvoidanceAlgorithm(Protocol):
    """Shared interface for pluggable collision avoidance algorithms."""

    name: str

    def step(self, snapshot: WorldSnapshot) -> list[AgentCommand]:
        ...
