#!/usr/bin/env python3
"""Score frozen NHC/ZUPT A/B replays without feeding truth into the solver."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path


def load(path: Path) -> dict[float, dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    return {float(row["tow"]): row for row in rows}


def error3d(row: dict[str, str]) -> float:
    return math.sqrt(sum(float(row[key]) ** 2 for key in ("e_err_m", "n_err_m", "u_err_m")))


def percentile(values: list[float], pct: float) -> float:
    ordered = sorted(values)
    if not ordered:
        return math.nan
    rank = (len(ordered) - 1) * pct / 100.0
    lo = math.floor(rank)
    hi = math.ceil(rank)
    return ordered[lo] + (ordered[hi] - ordered[lo]) * (rank - lo)


def metrics(rows: dict[float, dict[str, str]]) -> dict[str, float | int]:
    fixed = [row for row in rows.values() if row["status"] == "FIXED"]
    correct = [row for row in fixed if error3d(row) <= 0.5]
    horizontal = [float(row["horiz_err_m"]) for row in fixed]
    return {
        "rows": len(rows),
        "fixed": len(fixed),
        "correct_fixed": len(correct),
        "wrong_fixed": len(fixed) - len(correct),
        "fixed_horizontal_rms_m": math.sqrt(sum(value * value for value in horizontal) / len(horizontal)),
        "fixed_horizontal_p95_m": percentile(horizontal, 95.0),
        "zupt_candidates": sum(int(row["zupt_candidate"]) for row in rows.values()),
        "zupt_applied": sum(int(row["zupt_applied"]) for row in rows.values()),
        "nhc_candidates": sum(int(row["nhc_candidate"]) for row in rows.values()),
        "nhc_applied": sum(int(row["nhc_applied"]) for row in rows.values()),
    }


def compare(
    baseline: dict[float, dict[str, str]],
    variant: dict[float, dict[str, str]],
    baseline_runtime_s: float,
    variant_runtime_s: float,
) -> dict[str, object]:
    if baseline.keys() != variant.keys():
        raise ValueError("baseline and variant TOW sets differ")
    base_metrics = metrics(baseline)
    variant_metrics = metrics(variant)
    added_correct = lost_correct = added_wrong = 0
    for tow, base in baseline.items():
        candidate = variant[tow]
        base_correct = base["status"] == "FIXED" and error3d(base) <= 0.5
        candidate_correct = candidate["status"] == "FIXED" and error3d(candidate) <= 0.5
        base_wrong = base["status"] == "FIXED" and not base_correct
        candidate_wrong = candidate["status"] == "FIXED" and not candidate_correct
        added_correct += candidate_correct and not base_correct
        lost_correct += base_correct and not candidate_correct
        added_wrong += candidate_wrong and not base_wrong
    rms_ratio = float(variant_metrics["fixed_horizontal_rms_m"]) / float(base_metrics["fixed_horizontal_rms_m"])
    p95_ratio = float(variant_metrics["fixed_horizontal_p95_m"]) / float(base_metrics["fixed_horizontal_p95_m"])
    runtime_ratio = variant_runtime_s / baseline_runtime_s
    gate_checks = {
        "zero_added_wrong_fix": added_wrong == 0,
        "zero_lost_correct_fix": lost_correct == 0,
        "at_least_three_added_correct_fix": added_correct >= 3,
        "same_row_count": variant_metrics["rows"] == base_metrics["rows"],
        "fixed_rms_within_5pct": rms_ratio <= 1.05,
        "fixed_p95_within_5pct": p95_ratio <= 1.05,
        "runtime_within_10pct": runtime_ratio <= 1.10,
    }
    return {
        "metrics": variant_metrics,
        "added_correct_fix": added_correct,
        "lost_correct_fix": lost_correct,
        "added_wrong_fix": added_wrong,
        "fixed_rms_ratio": rms_ratio,
        "fixed_p95_ratio": p95_ratio,
        "runtime_ratio": runtime_ratio,
        "gate_checks": gate_checks,
        "passes": all(gate_checks.values()),
    }


def parse_assignment(value: str) -> tuple[str, str]:
    name, separator, item = value.partition("=")
    if not separator or not name or not item:
        raise argparse.ArgumentTypeError("expected NAME=VALUE")
    return name, item


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--baseline", type=Path, required=True)
    parser.add_argument("--baseline-runtime-s", type=float, required=True)
    parser.add_argument("--variant", action="append", type=parse_assignment, required=True)
    parser.add_argument("--runtime-s", action="append", type=parse_assignment, required=True)
    parser.add_argument("--json", type=Path, required=True)
    args = parser.parse_args()
    runtimes = {name: float(value) for name, value in args.runtime_s}
    baseline = load(args.baseline)
    payload: dict[str, object] = {
        "correct_fix_threshold_3d_m": 0.5,
        "baseline": metrics(baseline),
        "baseline_runtime_s": args.baseline_runtime_s,
        "variants": {},
    }
    variants = payload["variants"]
    assert isinstance(variants, dict)
    for name, value in args.variant:
        if name not in runtimes:
            parser.error(f"missing --runtime-s {name}=SECONDS")
        variants[name] = compare(baseline, load(Path(value)), args.baseline_runtime_s, runtimes[name])
    args.json.parent.mkdir(parents=True, exist_ok=True)
    args.json.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(payload, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
