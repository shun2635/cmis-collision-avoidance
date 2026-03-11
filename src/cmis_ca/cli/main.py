"""Command line interface for the Python skeleton."""

from __future__ import annotations

import argparse
from typing import Sequence

from cmis_ca.algorithms.registry import list_algorithms
from cmis_ca.cli.run import run_demo, run_scenario_file


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="cmis-ca")
    subparsers = parser.add_subparsers(dest="command")

    run_parser = subparsers.add_parser("run", help="Run a built-in or file-based scenario.")
    run_parser.add_argument(
        "--algorithm",
        default="orca",
        choices=list_algorithms(),
        help="Algorithm to run.",
    )
    run_parser.add_argument(
        "--steps",
        type=int,
        default=None,
        help="Override the number of simulation steps.",
    )
    run_parser.add_argument(
        "--scenario",
        default=None,
        help="Path to a YAML or JSON scenario file.",
    )

    return parser


def main(argv: Sequence[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(list(argv) if argv is not None else None)

    if args.command != "run":
        parser.print_help()
        return 0

    if args.steps is not None and args.steps < 0:
        parser.error("--steps must be non-negative")

    if args.scenario:
        result = run_scenario_file(args.algorithm, args.scenario, steps=args.steps)
    else:
        result = run_demo(args.algorithm, steps=1 if args.steps is None else args.steps)

    for index, state in enumerate(result.final_states):
        print(
            f"agent={index} position=({state.position.x:.3f}, {state.position.y:.3f}) "
            f"velocity=({state.velocity.x:.3f}, {state.velocity.y:.3f})"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
