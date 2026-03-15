"""Build and run a legacy CNav_MyStyle driver with JSONL trace export enabled."""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import subprocess
import sys
import tempfile

REPO_ROOT = Path(__file__).resolve().parents[1]
LEGACY_ROOT = REPO_ROOT / "external" / "CNav_MyStyle"
LEGACY_SRC_ROOT = LEGACY_ROOT / "src"
LEGACY_SIM_ROOT = LEGACY_ROOT / "simulations"
DEFAULT_BUILD_DIR = LEGACY_ROOT / "build_trace"

DRIVER_SOURCES = {
    "forpaper": LEGACY_SIM_ROOT / "forPaper.cpp",
    "crowd-forpaper": LEGACY_SIM_ROOT / "crowdForPaper.cpp",
}

LIBRARY_SOURCES = (
    LEGACY_SRC_ROOT / "Agent.cpp",
    LEGACY_SRC_ROOT / "KdTree.cpp",
    LEGACY_SRC_ROOT / "Obstacle.cpp",
    LEGACY_SRC_ROOT / "RVOSimulator.cpp",
)


def build_legacy_trace_binary(driver: str, build_dir: Path, cxx: str) -> Path:
    build_dir.mkdir(parents=True, exist_ok=True)
    source = DRIVER_SOURCES[driver]
    output = build_dir / f"{driver.replace('-', '_')}_trace"
    command = [
        cxx,
        "-std=c++17",
        "-O2",
        "-I",
        str(LEGACY_SRC_ROOT),
        str(source),
        *(str(path) for path in LIBRARY_SOURCES),
        "-o",
        str(output),
    ]
    subprocess.run(command, check=True, cwd=REPO_ROOT)
    return output


def run_legacy_trace_binary(
    binary: Path,
    *,
    output_path: Path,
    steps: int | None,
    seed: int,
) -> subprocess.CompletedProcess[str]:
    env = dict(os.environ)
    env["CNAV_TRACE_OUTPUT"] = str(output_path)
    if steps is not None:
        env["CNAV_TRACE_STEPS"] = str(steps)
    env["CNAV_TRACE_SEED"] = str(seed)
    return subprocess.run(
        [str(binary)],
        check=True,
        cwd=REPO_ROOT,
        env=env,
        text=True,
        capture_output=True,
    )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--driver",
        choices=tuple(DRIVER_SOURCES),
        default="forpaper",
        help="Legacy driver to build and run.",
    )
    parser.add_argument(
        "--steps",
        type=int,
        default=None,
        help="Override trace step count via CNAV_TRACE_STEPS.",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=None,
        help="Write JSONL to this path instead of stdout.",
    )
    parser.add_argument(
        "--build-dir",
        type=Path,
        default=DEFAULT_BUILD_DIR,
        help="Directory used for the compiled legacy trace binary.",
    )
    parser.add_argument(
        "--cxx",
        default="g++",
        help="C++ compiler used for the legacy driver build.",
    )
    parser.add_argument(
        "--seed",
        type=int,
        default=1,
        help="Deterministic seed passed to the legacy driver.",
    )
    parser.add_argument(
        "--verbose",
        action="store_true",
        help="Print build / run stdout and stderr to stderr.",
    )
    args = parser.parse_args()

    binary = build_legacy_trace_binary(args.driver, args.build_dir, args.cxx)
    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        result = run_legacy_trace_binary(binary, output_path=args.output, steps=args.steps, seed=args.seed)
        if args.verbose:
            if result.stdout:
                print(result.stdout, file=sys.stderr, end="")
            if result.stderr:
                print(result.stderr, file=sys.stderr, end="")
        return

    with tempfile.TemporaryDirectory() as temp_dir:
        temp_output = Path(temp_dir) / "legacy-trace.jsonl"
        result = run_legacy_trace_binary(
            binary,
            output_path=temp_output,
            steps=args.steps,
            seed=args.seed,
        )
        if args.verbose:
            if result.stdout:
                print(result.stdout, file=sys.stderr, end="")
            if result.stderr:
                print(result.stderr, file=sys.stderr, end="")
        print(temp_output.read_text(encoding="utf-8"), end="")


if __name__ == "__main__":
    main()
