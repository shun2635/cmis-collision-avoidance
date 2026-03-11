"""Smoke script for the Python skeleton."""

from __future__ import annotations

from pathlib import Path
import sys

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT / "src"))

from cmis_ca.cli.run import run_demo


def main() -> int:
    result = run_demo("orca", steps=1)
    state = result.final_states[0]
    print(
        f"agent=0 position=({state.position.x:.3f}, {state.position.y:.3f}) "
        f"velocity=({state.velocity.x:.3f}, {state.velocity.y:.3f})"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
