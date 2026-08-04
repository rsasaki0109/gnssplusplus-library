#!/usr/bin/env python3
"""Score disjoint-constellation AR solution separation against offline truth."""

from __future__ import annotations

import argparse
import csv
import json
import math
from bisect import bisect_left
from pathlib import Path
from typing import Any


def tow_key(value: str) -> int:
    return round(float(value) * 1000.0)


def ecef(row: dict[str, str], prefix: str) -> tuple[float, float, float]:
    return tuple(float(row[f"{prefix}_{axis}_ecef_m"]) for axis in "xyz")  # type: ignore[return-value]


def distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return math.sqrt(sum((x - y) ** 2 for x, y in zip(a, b)))


def percentile(values: list[float], fraction: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    index = max(0, min(len(ordered) - 1, math.ceil(fraction * len(ordered)) - 1))
    return ordered[index]


def load_reference(path: Path) -> tuple[list[int], dict[int, tuple[float, float, float]]]:
    rows: dict[int, tuple[float, float, float]] = {}
    with path.open(newline="", encoding="utf-8-sig") as stream:
        for raw in csv.DictReader(stream):
            row = {key.strip(): value.strip() for key, value in raw.items()}
            key = tow_key(row["GPS TOW (s)"])
            rows[key] = (
                float(row["ECEF X (m)"]),
                float(row["ECEF Y (m)"]),
                float(row["ECEF Z (m)"]),
            )
    return sorted(rows), rows


def nearest_reference(
    key: int,
    ordered: list[int],
    rows: dict[int, tuple[float, float, float]],
) -> tuple[float, float, float] | None:
    index = bisect_left(ordered, key)
    choices = ordered[max(0, index - 1) : min(len(ordered), index + 1)]
    if not choices:
        return None
    nearest = min(choices, key=lambda candidate: abs(candidate - key))
    return rows[nearest] if abs(nearest - key) <= 110 else None


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--epoch-csv", type=Path, required=True)
    parser.add_argument("--shadow-csv", type=Path, required=True)
    parser.add_argument("--reference-csv", type=Path, required=True)
    parser.add_argument("--json", type=Path)
    parser.add_argument("--correct-threshold-m", type=float, default=0.5)
    parser.add_argument("--maximum-correct-harm", type=float, default=0.05)
    parser.add_argument("--minimum-wrong-capture", type=float, default=0.50)
    parser.add_argument("--minimum-support", type=int, default=100)
    args = parser.parse_args()

    epoch_rows: dict[int, dict[str, str]] = {}
    with args.epoch_csv.open(newline="", encoding="utf-8-sig") as stream:
        for row in csv.DictReader(stream):
            epoch_rows[tow_key(row["tow"])] = row

    reference_order, reference_rows = load_reference(args.reference_csv)
    scored: list[dict[str, Any]] = []
    unavailable = 0
    with args.shadow_csv.open(newline="", encoding="utf-8-sig") as stream:
        for row in csv.DictReader(stream):
            key = tow_key(row["tow"])
            epoch = epoch_rows.get(key)
            truth = nearest_reference(key, reference_order, reference_rows)
            if (
                epoch is None
                or truth is None
                or row["evaluated"] != "1"
                or row["primary_candidate_available"] != "1"
                or row["partition_a_candidate_available"] != "1"
                or row["partition_b_candidate_available"] != "1"
            ):
                unavailable += 1
                continue
            primary_error = distance(ecef(row, "primary"), truth)
            separations = [
                float(row["partition_separation_m"]),
                float(row["partition_a_primary_separation_m"]),
                float(row["partition_b_primary_separation_m"]),
            ]
            if not math.isfinite(primary_error) or not all(map(math.isfinite, separations)):
                unavailable += 1
                continue
            scored.append(
                {
                    "correct": primary_error <= args.correct_threshold_m,
                    "status": epoch["status"],
                    "score": max(separations),
                    "composition": (
                        f"{row['partition_a_system_mask']}/"
                        f"{row['partition_b_system_mask']}"
                    ),
                }
            )

    correct_scores = [row["score"] for row in scored if row["correct"]]
    wrong_scores = [row["score"] for row in scored if not row["correct"]]
    threshold = percentile(correct_scores, 1.0 - args.maximum_correct_harm)
    correct_rejected = (
        sum(score > threshold for score in correct_scores)
        if threshold is not None
        else 0
    )
    wrong_rejected = (
        sum(score > threshold for score in wrong_scores)
        if threshold is not None
        else 0
    )
    correct_harm = correct_rejected / len(correct_scores) if correct_scores else None
    wrong_capture = wrong_rejected / len(wrong_scores) if wrong_scores else None

    population = {
        f"{'correct' if row['correct'] else 'wrong'}_{row['status'].lower()}": 0
        for row in scored
    }
    composition: dict[str, dict[str, int]] = {}
    for row in scored:
        population[f"{'correct' if row['correct'] else 'wrong'}_{row['status'].lower()}"] += 1
        bucket = composition.setdefault(row["composition"], {"correct": 0, "wrong": 0})
        bucket["correct" if row["correct"] else "wrong"] += 1

    grid = []
    for candidate in (0.01, 0.02, 0.05, 0.1, 0.2, 0.5, 1.0, 2.0, 5.0, 10.0):
        grid.append(
            {
                "threshold_m": candidate,
                "correct_harm": (
                    sum(score > candidate for score in correct_scores) / len(correct_scores)
                    if correct_scores
                    else None
                ),
                "wrong_capture": (
                    sum(score > candidate for score in wrong_scores) / len(wrong_scores)
                    if wrong_scores
                    else None
                ),
            }
        )

    gate_passed = (
        len(correct_scores) >= args.minimum_support
        and len(wrong_scores) >= args.minimum_support
        and correct_harm is not None
        and correct_harm <= args.maximum_correct_harm
        and wrong_capture is not None
        and wrong_capture >= args.minimum_wrong_capture
    )
    summary = {
        "gate_passed": gate_passed,
        "rows": {
            "epoch": len(epoch_rows),
            "scored": len(scored),
            "unavailable": unavailable,
            "correct": len(correct_scores),
            "wrong": len(wrong_scores),
        },
        "population": population,
        "selected_rule": {
            "reject_when_max_separation_exceeds_m": threshold,
            "correct_rejected": correct_rejected,
            "correct_harm": correct_harm,
            "wrong_rejected": wrong_rejected,
            "wrong_capture": wrong_capture,
        },
        "requirements": {
            "minimum_support_each": args.minimum_support,
            "maximum_correct_harm": args.maximum_correct_harm,
            "minimum_wrong_capture": args.minimum_wrong_capture,
        },
        "partition_composition": composition,
        "threshold_grid": grid,
    }
    rendered = json.dumps(summary, indent=2, sort_keys=True, allow_nan=False)
    print(rendered)
    if args.json:
        args.json.write_text(rendered + "\n", encoding="utf-8")
    return 0 if gate_passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
