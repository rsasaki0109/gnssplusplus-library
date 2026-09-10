#!/usr/bin/env python3
"""Compare PPC moving-CLAS candidate solutions against a fixed baseline."""

from __future__ import annotations

from _paths import PPC_DIR

import argparse
import json
import math
import sys
from pathlib import Path
from typing import Any

import numpy as np

if str(PPC_DIR) not in sys.path:
    sys.path.insert(0, str(PPC_DIR))

import generate_driving_comparison as comparison
import generate_ppc_clas_scorecard as scorecard


PPP_FLOAT_STATUS = 5
PPP_SINGLE_STATUS = 1
MIN_COVERAGE = 0.99


def rounded(value: float) -> float:
    return round(float(value), 6)


def percentile(values: np.ndarray, pct: float) -> float | None:
    return rounded(np.percentile(values, pct)) if len(values) else None


def rms(values: np.ndarray) -> float | None:
    return rounded(math.sqrt(float(np.mean(values**2)))) if len(values) else None


def load_solution(
    pos_path: Path,
    reference: list[comparison.ReferenceEpoch],
    observation_epochs: int,
    tow_start: float | None = None,
    tow_end: float | None = None,
) -> dict[str, Any]:
    solutions = scorecard.read_ppp_pos(pos_path)
    if tow_start is not None and tow_end is not None:
        solutions = [
            solution
            for solution in solutions
            if tow_start <= solution.tow <= tow_end
        ]
    matched = comparison.match_to_reference(
        solutions, reference, scorecard.MATCH_TOLERANCE_S
    )
    reference_duration = reference[-1].tow - reference[0].tow
    matched_duration = matched[-1].tow - matched[0].tow if matched else 0.0
    interval_coverage = (
        matched_duration / reference_duration if reference_duration > 0.0 else 0.0
    )
    epoch_coverage = (
        len(matched) / observation_epochs if observation_epochs > 0 else 0.0
    )
    if interval_coverage < MIN_COVERAGE or epoch_coverage < MIN_COVERAGE:
        raise SystemExit(
            f"Incomplete candidate {pos_path}: interval coverage "
            f"{100.0 * interval_coverage:.3f}%, epoch coverage "
            f"{100.0 * epoch_coverage:.3f}%"
        )
    scored = matched[scorecard.ARTICLE_SKIP_EPOCHS :]
    if not scored:
        raise SystemExit(f"No scored epochs for {pos_path}")

    fixed = [item for item in scored if item.status == scorecard.PPP_FIXED_STATUS]
    floating = [item for item in scored if item.status == PPP_FLOAT_STATUS]
    single = [item for item in scored if item.status == PPP_SINGLE_STATUS]
    fixed_error = np.asarray([item.horiz_error_m for item in fixed], dtype=float)
    all_error = np.asarray([item.horiz_error_m for item in scored], dtype=float)
    float_error = np.asarray([item.horiz_error_m for item in floating], dtype=float)
    single_error = np.asarray([item.horiz_error_m for item in single], dtype=float)

    return {
        "path": str(pos_path.resolve()),
        "matched": matched,
        "scored": scored,
        "fixed_by_tow": {rounded(item.tow): item for item in fixed},
        "metrics": {
            "solution_epochs": len(solutions),
            "matched_epochs": len(matched),
            "scored_epochs": len(scored),
            "interval_coverage_pct": rounded(100.0 * interval_coverage),
            "epoch_coverage_pct": rounded(100.0 * epoch_coverage),
            "fixed_epochs": len(fixed),
            "float_epochs": len(floating),
            "single_epochs": len(single),
            "fix_pct": rounded(100.0 * len(fixed) / len(scored)),
            "rms2d_fixed_m": rms(fixed_error),
            "p68_fixed_m": percentile(fixed_error, 68),
            "p95_fixed_m": percentile(fixed_error, 95),
            "max_fixed_m": rounded(np.max(fixed_error)) if len(fixed_error) else None,
            "fixed_over_1m": int(np.count_nonzero(fixed_error > 1.0)),
            "fixed_over_3m": int(np.count_nonzero(fixed_error > 3.0)),
            "rms2d_all_m": rms(all_error),
            "rms2d_float_m": rms(float_error),
            "rms2d_single_m": rms(single_error),
            "ttff_30_s": scorecard.compute_ttff_s(
                scored, scorecard.PPP_FIXED_STATUS
            ),
        },
    }


def parse_candidate(value: str) -> tuple[str, Path]:
    label, separator, path = value.partition("=")
    if not separator or not label or not path:
        raise argparse.ArgumentTypeError("candidate must be LABEL=PATH")
    return label, Path(path)


def summarize_delta(
    baseline: dict[str, Any], candidate: dict[str, Any]
) -> dict[str, Any]:
    baseline_fixed = baseline["fixed_by_tow"]
    candidate_fixed = candidate["fixed_by_tow"]
    gained_tows = sorted(candidate_fixed.keys() - baseline_fixed.keys())
    lost_tows = sorted(baseline_fixed.keys() - candidate_fixed.keys())
    gained_error = np.asarray(
        [candidate_fixed[tow].horiz_error_m for tow in gained_tows], dtype=float
    )
    lost_error = np.asarray(
        [baseline_fixed[tow].horiz_error_m for tow in lost_tows], dtype=float
    )
    return {
        "gained_fixed_epochs": len(gained_tows),
        "lost_fixed_epochs": len(lost_tows),
        "net_fixed_epochs": len(gained_tows) - len(lost_tows),
        "gained_fixed_rms2d_m": rms(gained_error),
        "gained_fixed_p68_m": percentile(gained_error, 68),
        "gained_fixed_max_m": rounded(np.max(gained_error)) if len(gained_error) else None,
        "gained_fixed_over_1m": int(np.count_nonzero(gained_error > 1.0)),
        "gained_fixed_over_3m": int(np.count_nonzero(gained_error > 3.0)),
        "lost_fixed_rms2d_m": rms(lost_error),
        "lost_fixed_max_m": rounded(np.max(lost_error)) if len(lost_error) else None,
        "gained_tows": gained_tows,
        "lost_tows": lost_tows,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset-root", type=Path, required=True)
    parser.add_argument("--run", required=True, help="for example tokyo_run2")
    parser.add_argument("--baseline", type=Path, required=True)
    parser.add_argument("--tow-start", type=float)
    parser.add_argument("--tow-end", type=float)
    parser.add_argument(
        "--candidate",
        type=parse_candidate,
        action="append",
        required=True,
        metavar="LABEL=PATH",
    )
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    if (args.tow_start is None) != (args.tow_end is None):
        parser.error("--tow-start and --tow-end must be provided together")
    if (
        args.tow_start is not None
        and args.tow_end is not None
        and args.tow_end < args.tow_start
    ):
        parser.error("--tow-end must be greater than or equal to --tow-start")

    city, run = args.run.split("_", 1)
    run_dir = args.dataset_root / city / run
    reference = scorecard.read_reference_csv(
        run_dir / "reference.csv", apply_lever_arm=False, city=city
    )
    observation_stamps = scorecard.parse_rinex_observation_epochs(
        run_dir / "rover.obs"
    )
    if args.tow_start is not None and args.tow_end is not None:
        # RINEX headers are UTC while emitted PPC solution TOW follows the
        # dataset's raw epoch convention. Align the observation stamps to the
        # reference origin before applying a solution-time window.
        observation_tow_offset = observation_stamps[0][1] - reference[0].tow
        reference = [
            epoch
            for epoch in reference
            if args.tow_start <= epoch.tow <= args.tow_end
        ]
        observation_stamps = [
            stamp
            for stamp in observation_stamps
            if args.tow_start
            <= stamp[1] - observation_tow_offset
            <= args.tow_end
        ]
    if not reference or not observation_stamps:
        raise SystemExit("The requested time window contains no reference epochs")
    observation_epochs = len(observation_stamps)
    baseline = load_solution(
        args.baseline,
        reference,
        observation_epochs,
        args.tow_start,
        args.tow_end,
    )
    result: dict[str, Any] = {
        "run": args.run,
        "reference_point": "ppc_reference_raw_antenna_position",
        "warmup_skipped_epochs": scorecard.ARTICLE_SKIP_EPOCHS,
        "fixed_status": scorecard.PPP_FIXED_STATUS,
        "ttff_consecutive_fixed_epochs": (
            scorecard.ARTICLE_TTFF_CONSECUTIVE_FIX_EPOCHS
        ),
        "tow_window": (
            {"start": args.tow_start, "end": args.tow_end}
            if args.tow_start is not None
            else None
        ),
        "baseline": {
            "path": baseline["path"],
            "metrics": baseline["metrics"],
        },
        "candidates": {},
    }
    for label, path in args.candidate:
        candidate = load_solution(
            path,
            reference,
            observation_epochs,
            args.tow_start,
            args.tow_end,
        )
        result["candidates"][label] = {
            "path": candidate["path"],
            "metrics": candidate["metrics"],
            "delta_vs_baseline": summarize_delta(baseline, candidate),
        }

    rendered = json.dumps(result, indent=2, sort_keys=True) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered, encoding="utf-8")
        print(f"Wrote {args.output}")
    else:
        print(rendered, end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
