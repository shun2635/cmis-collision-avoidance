"""Command line interface for the Python skeleton."""

from __future__ import annotations

import argparse
from typing import Sequence

from cmis_ca.algorithms.cnav.parameters import CNavParameterProfile, CNavMyStyleDriver
from cmis_ca.algorithms.registry import list_algorithms
from cmis_ca.cli.run import run_demo, run_scenario_file
from cmis_ca.cli.visualize import run_visualization


CNAV_PROFILE_CHOICES: tuple[CNavParameterProfile, ...] = (
    "paper",
    "legacy-forpaper-comparison",
    "legacy-forpaper-decision",
    "legacy-crowd-decision",
)
CNAV_MYSTYLE_DRIVER_CHOICES: tuple[CNavMyStyleDriver, ...] = (
    "forpaper",
    "crowd-forpaper",
)


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
    run_profile_group = run_parser.add_mutually_exclusive_group()
    run_profile_group.add_argument(
        "--cnav-profile",
        choices=CNAV_PROFILE_CHOICES,
        default=None,
        help="Named CNav parameter profile.",
    )
    run_profile_group.add_argument(
        "--cnav-mystyle-driver",
        choices=CNAV_MYSTYLE_DRIVER_CHOICES,
        default=None,
        help="Apply the full legacy MyStyle driver settings bundle to CNav.",
    )

    visualize_parser = subparsers.add_parser(
        "visualize",
        help="Run a scenario and open the PyQtGraph viewer.",
    )
    visualize_parser.add_argument(
        "--algorithm",
        default="orca",
        choices=list_algorithms(),
        help="Algorithm to run.",
    )
    visualize_parser.add_argument(
        "--steps",
        type=int,
        default=None,
        help="Override the number of simulation steps.",
    )
    visualize_parser.add_argument(
        "--scenario",
        default=None,
        help="Path to a YAML or JSON scenario file.",
    )
    visualize_profile_group = visualize_parser.add_mutually_exclusive_group()
    visualize_profile_group.add_argument(
        "--cnav-profile",
        choices=CNAV_PROFILE_CHOICES,
        default=None,
        help="Named CNav parameter profile.",
    )
    visualize_profile_group.add_argument(
        "--cnav-mystyle-driver",
        choices=CNAV_MYSTYLE_DRIVER_CHOICES,
        default=None,
        help="Apply the full legacy MyStyle driver settings bundle to CNav.",
    )
    visualize_parser.add_argument(
        "--fps",
        type=float,
        default=30.0,
        help="Playback speed used by the viewer.",
    )
    visualize_parser.add_argument(
        "--save-animation",
        default=None,
        help="Save the generated animation to .mp4 or .gif.",
    )

    return parser


def main(argv: Sequence[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(list(argv) if argv is not None else None)

    if args.command not in {"run", "visualize"}:
        parser.print_help()
        return 0

    if args.steps is not None and args.steps < 0:
        parser.error("--steps must be non-negative")
    if getattr(args, "fps", 30.0) <= 0.0:
        parser.error("--fps must be positive")
    if args.algorithm != "cnav" and (
        getattr(args, "cnav_profile", None) is not None
        or getattr(args, "cnav_mystyle_driver", None) is not None
    ):
        parser.error("--cnav-profile and --cnav-mystyle-driver require --algorithm cnav")

    if args.command == "visualize":
        run_visualization(
            args.algorithm,
            args.scenario,
            steps=args.steps,
            fps=args.fps,
            save_animation=args.save_animation,
            cnav_profile=args.cnav_profile,
            cnav_mystyle_driver=args.cnav_mystyle_driver,
        )
        return 0

    if args.scenario:
        result = run_scenario_file(
            args.algorithm,
            args.scenario,
            steps=args.steps,
            cnav_profile=args.cnav_profile,
            cnav_mystyle_driver=args.cnav_mystyle_driver,
        )
    else:
        result = (
            run_demo(
                args.algorithm,
                cnav_profile=args.cnav_profile,
                cnav_mystyle_driver=args.cnav_mystyle_driver,
            )
            if args.steps is None
            else run_demo(
                args.algorithm,
                steps=args.steps,
                cnav_profile=args.cnav_profile,
                cnav_mystyle_driver=args.cnav_mystyle_driver,
            )
        )

    for index, state in enumerate(result.final_states):
        print(
            f"agent={index} position=({state.position.x:.3f}, {state.position.y:.3f}) "
            f"velocity=({state.velocity.x:.3f}, {state.velocity.y:.3f})"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
