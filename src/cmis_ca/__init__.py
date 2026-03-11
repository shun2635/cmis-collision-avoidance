"""CMIS collision avoidance research package."""

from .algorithms.registry import AlgorithmRegistry, create_algorithm, list_algorithms
from .core.simulation import Simulator

__all__ = [
    "AlgorithmRegistry",
    "Simulator",
    "create_algorithm",
    "list_algorithms",
]

__version__ = "0.1.0"
