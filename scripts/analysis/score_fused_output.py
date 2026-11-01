#!/usr/bin/env python3
"""Score `gnss fuse --out` trajectories against a PPC-layout reference: 3D error p50 / p90.

Matching follows the RTK scoring of ppc_fuse_spread.py: the reference as given
(it must be at the antenna), 0.25 s matching, the first 60 matched epochs
skipped. Every matched epoch counts, whatever its status. Percentiles are
linear (numpy's default), as in the imu_fusion.md tables.

With `--control` and `--candidate` the per-run rule of
docs/smartpnt_fuse_output_holdout.md is applied as well:

- a quantile is *lower* if the candidate's is below 0.99 times the control's,
  *higher* if above 1.01 times, *equal* otherwise;
- a run is *unusable* if both p50 are above 10 m, *worse* if a quantile is
  higher or the candidate matches fewer than 99% of the control's epochs,
  *better* if a quantile is lower, *equal* otherwise.

Example:

    python3 scripts/analysis/score_fused_output.py --reference run/reference.csv \\
        --control control_fused.pos --candidate candidate_fused.pos
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys

import numpy as np

ROOT_DIR = Path(__file__).resolve().parents[2]
SCRIPTS_DIR = ROOT_DIR / "scripts"
if str(SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_DIR))

import generate_driving_comparison as comparison  # noqa: E402

MATCH_TOLERANCE_S = 0.25
SKIP_EPOCHS = 60
EQUAL_BAND = 0.01           # quantiles within 1% of the control's are equal
UNUSABLE_P50_M = 10.0       # both p50 above this: the run cannot separate the arms
MIN_EPOCH_SHARE = 0.99      # the candidate must match this share of the control's epochs


def errors_3d(pos: Path, reference_csv: Path) -> list[float]:
    reference = comparison.read_reference_csv(reference_csv)
    solution = comparison.read_libgnss_pos(pos)
    matched = comparison.match_to_reference(solution, reference, MATCH_TOLERANCE_S)[SKIP_EPOCHS:]
    return [math.sqrt(e.east_m**2 + e.north_m**2 + e.up_m**2) for e in matched]


def score(errors: list[float]) -> dict[str, float]:
    if not errors:
        raise ValueError("no epochs matched the reference after the skip")
    values = np.asarray(errors)
    return {"epochs": len(errors), "p50_m": float(np.percentile(values, 50)),
            "p90_m": float(np.percentile(values, 90))}


def score_file(pos: Path, reference_csv: Path) -> dict[str, float]:
    return score(errors_3d(pos, reference_csv))


def compare_quantile(control: float, candidate: float) -> str:
    if candidate < (1.0 - EQUAL_BAND) * control:
        return "lower"
    if candidate > (1.0 + EQUAL_BAND) * control:
        return "higher"
    return "equal"


def run_outcome(control: dict[str, float], candidate: dict[str, float]) -> dict:
    quantiles = {key: compare_quantile(control[key], candidate[key]) for key in ("p50_m", "p90_m")}
    if control["p50_m"] > UNUSABLE_P50_M and candidate["p50_m"] > UNUSABLE_P50_M:
        outcome = "unusable"
    elif "higher" in quantiles.values() or candidate["epochs"] < MIN_EPOCH_SHARE * control["epochs"]:
        outcome = "worse"
    elif "lower" in quantiles.values():
        outcome = "better"
    else:
        outcome = "equal"
    return {"outcome": outcome, "quantiles": quantiles}


def candidate_outcome(run_outcomes: list[str], rtk_identical: list[bool]) -> str:
    """Pass, fail or inconclusive over the runs (docs/smartpnt_fuse_output_holdout.md)."""
    if not all(rtk_identical) or "worse" in run_outcomes:
        return "fail"
    if "better" in run_outcomes:
        return "pass"
    return "inconclusive"


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--reference", type=Path, required=True, help="PPC-layout reference.csv")
    parser.add_argument("--pos", type=Path, nargs="*", default=[], help="trajectories to score")
    parser.add_argument("--control", type=Path, help="control --out of the per-run rule")
    parser.add_argument("--candidate", type=Path, help="candidate --out of the per-run rule")
    args = parser.parse_args(argv)
    if (args.control is None) != (args.candidate is None):
        parser.error("--control and --candidate go together")
    if not args.pos and args.control is None:
        parser.error("nothing to score")
    return args


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    try:
        report = {"files": {str(p): score_file(p, args.reference) for p in args.pos}}
        if args.control is not None:
            control = score_file(args.control, args.reference)
            candidate = score_file(args.candidate, args.reference)
            report.update(control=control, candidate=candidate, rule=run_outcome(control, candidate))
    except (OSError, ValueError) as error:
        print(f"score_fused_output: {error}", file=sys.stderr)
        return 2
    print(json.dumps(report, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
