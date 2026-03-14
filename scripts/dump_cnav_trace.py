"""Run a CNav validation scenario and dump per-agent trace records as JSONL."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = REPO_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from cmis_ca.algorithms.cnav import (
    CNavAlgorithm,
    create_cnav_parameters,
    run_cnav_trace,
    write_cnav_trace_jsonl,
)
from cmis_ca.io import load_scenario


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("scenario", help="Path to a YAML/JSON scenario file.")
    parser.add_argument(
        "--steps",
        type=int,
        default=None,
        help="Override the number of steps to run for trace capture.",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=None,
        help="Write JSONL output to this path instead of stdout.",
    )
    parser.add_argument(
        "--profile",
        choices=("paper", "legacy-forpaper-comparison"),
        default="paper",
        help="Named CNav parameter preset used for trace capture.",
    )
    args = parser.parse_args()

    scenario = load_scenario(args.scenario)
    _, trace = run_cnav_trace(
        scenario,
        CNavAlgorithm(parameters=create_cnav_parameters(args.profile)),
        steps=args.steps,
    )
    if args.output is None:
        print(trace.to_jsonl())
        return
    write_cnav_trace_jsonl(trace, args.output)


if __name__ == "__main__":
    main()
