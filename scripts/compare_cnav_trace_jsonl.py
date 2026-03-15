"""Compare Python CNav trace JSONL with legacy CNav_MyStyle trace JSONL."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
from pathlib import Path
import sys


@dataclass(frozen=True)
class TraceComparisonSummary:
    common_records: int
    python_only_records: int
    legacy_only_records: int
    chosen_action_mismatches: int
    max_chosen_intended_velocity_abs_diff: float
    max_output_velocity_abs_diff: float
    max_candidate_total_reward_abs_diff: float
    max_candidate_intended_velocity_abs_diff: float


def load_jsonl_records(path: Path) -> dict[tuple[int, int], dict[str, object]]:
    records: dict[tuple[int, int], dict[str, object]] = {}
    for line in path.read_text(encoding="utf-8").splitlines():
        stripped = line.strip()
        if not stripped:
            continue
        record = json.loads(stripped)
        key = (int(record["step_index"]), int(record["agent_index"]))
        records[key] = record
    return records


def compare_records(
    python_records: dict[tuple[int, int], dict[str, object]],
    legacy_records: dict[tuple[int, int], dict[str, object]],
) -> TraceComparisonSummary:
    python_keys = set(python_records)
    legacy_keys = set(legacy_records)
    common_keys = sorted(python_keys & legacy_keys)

    chosen_action_mismatches = 0
    max_chosen_intended_velocity_abs_diff = 0.0
    max_output_velocity_abs_diff = 0.0
    max_candidate_total_reward_abs_diff = 0.0
    max_candidate_intended_velocity_abs_diff = 0.0

    for key in common_keys:
        python_record = python_records[key]
        legacy_record = legacy_records[key]

        python_action = python_record["chosen_action_index"]
        legacy_action = legacy_record.get("next_action_index")
        if legacy_action is None:
            legacy_action = legacy_record["chosen_action_index"]
        if python_action != legacy_action:
            chosen_action_mismatches += 1

        python_intended = python_record["chosen_intended_velocity"]
        legacy_intended = legacy_record.get("next_chosen_intended_velocity")
        if legacy_intended is None:
            legacy_intended = legacy_record["chosen_intended_velocity"]
        max_chosen_intended_velocity_abs_diff = max(
            max_chosen_intended_velocity_abs_diff,
            _max_vector_abs_diff(python_intended, legacy_intended),
        )

        max_output_velocity_abs_diff = max(
            max_output_velocity_abs_diff,
            _max_vector_abs_diff(
                python_record["output_velocity"],
                legacy_record["output_velocity"],
            ),
        )

        python_candidates = {
            int(candidate["action_index"]): candidate
            for candidate in python_record["candidate_actions"]
        }
        legacy_candidates = {
            int(candidate["action_index"]): candidate
            for candidate in legacy_record["candidate_actions"]
        }
        for action_index in sorted(set(python_candidates) & set(legacy_candidates)):
            python_candidate = python_candidates[action_index]
            legacy_candidate = legacy_candidates[action_index]
            max_candidate_total_reward_abs_diff = max(
                max_candidate_total_reward_abs_diff,
                abs(float(python_candidate["total_reward"]) - float(legacy_candidate["total_reward"])),
            )
            max_candidate_intended_velocity_abs_diff = max(
                max_candidate_intended_velocity_abs_diff,
                _max_vector_abs_diff(
                    python_candidate["intended_velocity"],
                    legacy_candidate["intended_velocity"],
                ),
            )

    return TraceComparisonSummary(
        common_records=len(common_keys),
        python_only_records=len(python_keys - legacy_keys),
        legacy_only_records=len(legacy_keys - python_keys),
        chosen_action_mismatches=chosen_action_mismatches,
        max_chosen_intended_velocity_abs_diff=max_chosen_intended_velocity_abs_diff,
        max_output_velocity_abs_diff=max_output_velocity_abs_diff,
        max_candidate_total_reward_abs_diff=max_candidate_total_reward_abs_diff,
        max_candidate_intended_velocity_abs_diff=max_candidate_intended_velocity_abs_diff,
    )


def _max_vector_abs_diff(left: object, right: object) -> float:
    if left is None or right is None:
        return 0.0
    left_values = [float(value) for value in left]
    right_values = [float(value) for value in right]
    return max(abs(left_values[index] - right_values[index]) for index in range(len(left_values)))


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("python_trace", type=Path, help="Python JSONL trace path.")
    parser.add_argument("legacy_trace", type=Path, help="Legacy JSONL trace path.")
    parser.add_argument(
        "--json",
        action="store_true",
        help="Emit the summary as JSON instead of plain text.",
    )
    args = parser.parse_args()

    summary = compare_records(
        load_jsonl_records(args.python_trace),
        load_jsonl_records(args.legacy_trace),
    )
    if args.json:
        print(json.dumps(summary.__dict__, sort_keys=True))
        return

    print(f"common_records: {summary.common_records}")
    print(f"python_only_records: {summary.python_only_records}")
    print(f"legacy_only_records: {summary.legacy_only_records}")
    print(f"chosen_action_mismatches: {summary.chosen_action_mismatches}")
    print(
        "max_chosen_intended_velocity_abs_diff: "
        f"{summary.max_chosen_intended_velocity_abs_diff:.9f}"
    )
    print(f"max_output_velocity_abs_diff: {summary.max_output_velocity_abs_diff:.9f}")
    print(
        "max_candidate_total_reward_abs_diff: "
        f"{summary.max_candidate_total_reward_abs_diff:.9f}"
    )
    print(
        "max_candidate_intended_velocity_abs_diff: "
        f"{summary.max_candidate_intended_velocity_abs_diff:.9f}"
    )


if __name__ == "__main__":
    main()
