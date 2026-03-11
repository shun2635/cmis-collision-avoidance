"""ORCA-specific constraint builders."""

from __future__ import annotations

from cmis_ca.algorithms.orca.parameters import ORCAParameters
from cmis_ca.core.constraints import LineConstraint
from cmis_ca.core.neighbor_search import NeighborSet
from cmis_ca.core.world import WorldSnapshot


def build_obstacle_constraints(
    snapshot: WorldSnapshot,
    agent_index: int,
    neighbors: NeighborSet,
    parameters: ORCAParameters,
) -> list[LineConstraint]:
    """Build obstacle-derived ORCA constraints.

    The full ORCA obstacle geometry is intentionally deferred. The function
    exists so the repository layout matches the design documents from day one.
    """

    _ = snapshot, agent_index, neighbors, parameters
    return []


def build_agent_constraints(
    snapshot: WorldSnapshot,
    agent_index: int,
    neighbors: NeighborSet,
    parameters: ORCAParameters,
) -> list[LineConstraint]:
    """Build agent-derived ORCA constraints.

    The reciprocal-avoidance logic will be implemented later. The initial
    skeleton keeps the algorithm boundary stable while returning no constraints.
    """

    _ = snapshot, agent_index, neighbors, parameters
    return []
