#!/usr/bin/env python3
"""Score RTK-degraded spans bridged by a fused urban-driving trajectory."""

from __future__ import annotations

import argparse
import bisect
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import sys
from typing import Any

import numpy as np

from support.gnss_runtime import application_root, ensure_input_exists


ROOT_DIR = application_root(__file__)
SCRIPTS_DIR = ROOT_DIR / "scripts"
if str(SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_DIR))

import generate_driving_comparison as comparison  # noqa: E402
import gnss_ppc_demo as ppc_demo  # noqa: E402


SCHEMA_VERSION = "libgnsspp.urban_bridge_score.v1"
FIXED_STATUS = 4
COORDINATE_FRAME_CONTRACT = {
    "rtk_pos": "antenna_ecef",
    "fused_pos": "antenna_ecef",
    "reference_csv": "antenna_ecef",
    "internal_eskf_output": "imu_origin_local_enu",
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        prog=os.environ.get("GNSS_CLI_NAME"),
        description=(
            "Align RTK and fused libgnss++ .pos files to PPC reference.csv, "
            "then score RTK-degraded bridge and reacquisition spans."
        ),
    )
    parser.add_argument("--rtk-pos", type=Path, required=True)
    parser.add_argument("--fused-pos", type=Path, required=True)
    parser.add_argument("--reference-csv", type=Path, required=True)
    parser.add_argument("--summary-json", type=Path, required=True)
    parser.add_argument(
        "--segments-csv",
        type=Path,
        default=None,
        help="Per-bridge CSV (default: <summary-json stem>_segments.csv).",
    )
    parser.add_argument("--match-tolerance-s", type=float, default=0.11)
    parser.add_argument("--require-complete-bridges-min", type=int, default=None)
    parser.add_argument("--require-fused-bridge-coverage-min", type=float, default=None)
    parser.add_argument("--require-max-bridge-error-max", type=float, default=None)
    parser.add_argument("--require-max-reacquisition-jump-max", type=float, default=None)
    parser.add_argument("--require-fixed-p95-regression-max", type=float, default=None)
    parser.add_argument(
        "--require-fused-availability-at-least-rtk",
        action="store_true",
        help="Require fused solution availability to be at least the RTK availability.",
    )
    parser.add_argument("--require-no-nonfinite", action="store_true")
    return parser.parse_args()


def rounded(value: float | None) -> float | None:
    return None if value is None else round(float(value), 6)


def percentile(values: list[float], probability: float) -> float | None:
    if not values:
        return None
    return float(np.percentile(np.asarray(values, dtype=float), probability))


def absolute_time(week: int, tow: float) -> float:
    return week * 604800.0 + tow


def file_record(path: Path) -> dict[str, Any]:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return {
        "path": str(path),
        "bytes": path.stat().st_size,
        "sha256": digest.hexdigest(),
    }


def match_solutions(
    reference: list[comparison.ReferenceEpoch],
    solutions: list[comparison.SolutionEpoch],
    tolerance_s: float,
) -> dict[int, dict[str, Any]]:
    """Return the closest solution for each reference epoch."""
    reference_times = [absolute_time(row.week, row.tow) for row in reference]
    matched: dict[int, dict[str, Any]] = {}
    for solution in solutions:
        if not np.isfinite(np.asarray(solution.ecef, dtype=float)).all():
            continue
        solution_time = absolute_time(solution.week, solution.tow)
        index = bisect.bisect_left(reference_times, solution_time)
        candidates = [
            candidate
            for candidate in (index - 1, index, index + 1)
            if 0 <= candidate < len(reference)
        ]
        if not candidates:
            continue
        ref_index = min(
            candidates,
            key=lambda candidate: abs(reference_times[candidate] - solution_time),
        )
        time_gap_s = abs(reference_times[ref_index] - solution_time)
        if time_gap_s > tolerance_s:
            continue
        previous = matched.get(ref_index)
        if previous is not None and float(previous["time_gap_s"]) <= time_gap_s:
            continue
        ref = reference[ref_index]
        enu = comparison.ecef_to_enu(solution.ecef - ref.ecef, ref.lat_deg, ref.lon_deg)
        matched[ref_index] = {
            "solution": solution,
            "time_gap_s": time_gap_s,
            "east_error_m": float(enu[0]),
            "north_error_m": float(enu[1]),
            "up_error_m": float(enu[2]),
            "horizontal_error_m": float(math.hypot(float(enu[0]), float(enu[1]))),
        }
    return matched


def reference_window(
    reference: list[comparison.ReferenceEpoch],
    rtk_solutions: list[comparison.SolutionEpoch],
) -> tuple[int, int]:
    if not reference or not rtk_solutions:
        raise SystemExit("reference and RTK solution must both contain epochs")
    start_time = absolute_time(rtk_solutions[0].week, rtk_solutions[0].tow)
    end_time = absolute_time(rtk_solutions[-1].week, rtk_solutions[-1].tow)
    indices = [
        index
        for index, row in enumerate(reference)
        if start_time <= absolute_time(row.week, row.tow) <= end_time
    ]
    if not indices:
        raise SystemExit("RTK solution time span does not overlap reference.csv")
    return indices[0], indices[-1]


def bridge_ranges(
    first_index: int,
    last_index: int,
    rtk_matches: dict[int, dict[str, Any]],
) -> list[tuple[int, int]]:
    ranges: list[tuple[int, int]] = []
    start: int | None = None
    for index in range(first_index, last_index + 2):
        match = rtk_matches.get(index)
        fixed = match is not None and match["solution"].status == FIXED_STATUS
        degraded = index <= last_index and not fixed
        if degraded and start is None:
            start = index
        if not degraded and start is not None:
            ranges.append((start, index - 1))
            start = None
    return ranges


def score_bridge(
    segment_id: int,
    start_index: int,
    end_index: int,
    first_index: int,
    last_index: int,
    reference: list[comparison.ReferenceEpoch],
    rtk_matches: dict[int, dict[str, Any]],
    fused_matches: dict[int, dict[str, Any]],
) -> dict[str, Any]:
    entry_index = start_index - 1
    reacquisition_index = end_index + 1
    entry_fixed = (
        entry_index >= first_index
        and entry_index in rtk_matches
        and rtk_matches[entry_index]["solution"].status == FIXED_STATUS
    )
    reacquisition_fixed = (
        reacquisition_index <= last_index
        and reacquisition_index in rtk_matches
        and rtk_matches[reacquisition_index]["solution"].status == FIXED_STATUS
    )
    complete = entry_fixed and reacquisition_fixed
    fused_indices = [
        index for index in range(start_index, end_index + 1) if index in fused_matches
    ]
    errors = [float(fused_matches[index]["horizontal_error_m"]) for index in fused_indices]
    start_time = absolute_time(reference[start_index].week, reference[start_index].tow)
    end_time = absolute_time(reference[end_index].week, reference[end_index].tow)
    duration_s = max(0.0, end_time - start_time)

    entry_error = (
        float(fused_matches[entry_index]["horizontal_error_m"])
        if entry_index in fused_matches
        else (errors[0] if errors else None)
    )
    max_error = max(errors) if errors else None
    max_error_growth = (
        max(0.0, max_error - entry_error)
        if max_error is not None and entry_error is not None
        else None
    )
    propagation_age_s = (
        max(
            0.0,
            absolute_time(reference[end_index].week, reference[end_index].tow)
            - absolute_time(reference[entry_index].week, reference[entry_index].tow),
        )
        if entry_fixed
        else duration_s
    )
    drift_rate = (
        max_error_growth / propagation_age_s
        if max_error_growth is not None and propagation_age_s > 0.0
        else None
    )

    reacquisition_jump = None
    reacquisition_separation = None
    if complete and reacquisition_index in fused_matches:
        last_fused_index = fused_indices[-1] if fused_indices else entry_index
        if last_fused_index in fused_matches:
            before = fused_matches[last_fused_index]
            after = fused_matches[reacquisition_index]
            reacquisition_jump = math.hypot(
                float(after["east_error_m"]) - float(before["east_error_m"]),
                float(after["north_error_m"]) - float(before["north_error_m"]),
            )
        fused_solution = fused_matches[reacquisition_index]["solution"]
        rtk_solution = rtk_matches[reacquisition_index]["solution"]
        reacquisition_separation = float(math.dist(fused_solution.ecef, rtk_solution.ecef))

    statuses = sorted(
        {
            int(rtk_matches[index]["solution"].status)
            for index in range(start_index, end_index + 1)
            if index in rtk_matches
        }
    )
    return {
        "segment_id": segment_id,
        "complete_bridge": complete,
        "entry_fixed": entry_fixed,
        "reacquisition_fixed": reacquisition_fixed,
        "start_week": reference[start_index].week,
        "start_tow_s": rounded(reference[start_index].tow),
        "end_week": reference[end_index].week,
        "end_tow_s": rounded(reference[end_index].tow),
        "duration_s": rounded(duration_s),
        "propagation_age_s": rounded(propagation_age_s),
        "reference_epochs": end_index - start_index + 1,
        "rtk_matched_epochs": sum(index in rtk_matches for index in range(start_index, end_index + 1)),
        "rtk_statuses": statuses,
        "fused_matched_epochs": len(fused_indices),
        "fused_coverage_pct": rounded(100.0 * len(fused_indices) / (end_index - start_index + 1)),
        "entry_horizontal_error_m": rounded(entry_error),
        "median_horizontal_error_m": rounded(percentile(errors, 50.0)),
        "p95_horizontal_error_m": rounded(percentile(errors, 95.0)),
        "max_horizontal_error_m": rounded(max_error),
        "max_error_growth_m": rounded(max_error_growth),
        "error_growth_rate_mps": rounded(drift_rate),
        "reacquisition_error_step_m": rounded(reacquisition_jump),
        "fused_rtk_separation_at_reacquisition_m": rounded(reacquisition_separation),
    }


def aggregate_summary(
    reference_count: int,
    rtk_matches: dict[int, dict[str, Any]],
    fused_matches: dict[int, dict[str, Any]],
    segments: list[dict[str, Any]],
    first_index: int,
    last_index: int,
    rtk_nonfinite_epochs: int,
    fused_nonfinite_epochs: int,
) -> dict[str, Any]:
    window_indices = range(first_index, last_index + 1)
    rtk_window = {index: row for index, row in rtk_matches.items() if first_index <= index <= last_index}
    fused_window = {index: row for index, row in fused_matches.items() if first_index <= index <= last_index}
    fixed_indices = [
        index
        for index, row in rtk_window.items()
        if row["solution"].status == FIXED_STATUS
    ]
    common_fixed = [index for index in fixed_indices if index in fused_window]
    rtk_fixed_errors = [float(rtk_window[index]["horizontal_error_m"]) for index in common_fixed]
    fused_fixed_errors = [float(fused_window[index]["horizontal_error_m"]) for index in common_fixed]
    degraded_indices = [
        index
        for index in window_indices
        if index not in rtk_window or rtk_window[index]["solution"].status != FIXED_STATUS
    ]
    fused_bridge_epochs = sum(index in fused_window for index in degraded_indices)
    complete = [segment for segment in segments if segment["complete_bridge"]]
    bridge_errors = [
        float(segment["max_horizontal_error_m"])
        for segment in segments
        if segment["max_horizontal_error_m"] is not None
    ]
    jumps = [
        float(segment["reacquisition_error_step_m"])
        for segment in complete
        if segment["reacquisition_error_step_m"] is not None
    ]
    rtk_p95 = percentile(rtk_fixed_errors, 95.0)
    fused_p95 = percentile(fused_fixed_errors, 95.0)
    return {
        "reference_epochs": reference_count,
        "rtk_nonfinite_epochs": rtk_nonfinite_epochs,
        "fused_nonfinite_epochs": fused_nonfinite_epochs,
        "rtk_matched_epochs": len(rtk_window),
        "fused_matched_epochs": len(fused_window),
        "rtk_solution_availability_pct": rounded(100.0 * len(rtk_window) / reference_count),
        "rtk_fixed_epochs": len(fixed_indices),
        "rtk_fixed_availability_pct": rounded(100.0 * len(fixed_indices) / reference_count),
        "fused_availability_pct": rounded(100.0 * len(fused_window) / reference_count),
        "rtk_degraded_reference_epochs": len(degraded_indices),
        "fused_bridge_matched_epochs": fused_bridge_epochs,
        "fused_bridge_coverage_pct": rounded(
            100.0 * fused_bridge_epochs / len(degraded_indices) if degraded_indices else 100.0
        ),
        "degraded_segments": len(segments),
        "complete_bridges": len(complete),
        "max_bridge_horizontal_error_m": rounded(max(bridge_errors) if bridge_errors else None),
        "max_reacquisition_error_step_m": rounded(max(jumps) if jumps else None),
        "common_fixed_epochs": len(common_fixed),
        "rtk_fixed_p95_horizontal_error_m": rounded(rtk_p95),
        "fused_fixed_p95_horizontal_error_m": rounded(fused_p95),
        "fixed_p95_regression_m": rounded(
            fused_p95 - rtk_p95 if fused_p95 is not None and rtk_p95 is not None else None
        ),
    }


def gate_results(args: argparse.Namespace, aggregate: dict[str, Any]) -> tuple[dict[str, Any], list[str]]:
    thresholds = {
        "complete_bridges_min": args.require_complete_bridges_min,
        "fused_bridge_coverage_pct_min": args.require_fused_bridge_coverage_min,
        "max_bridge_horizontal_error_m_max": args.require_max_bridge_error_max,
        "max_reacquisition_error_step_m_max": args.require_max_reacquisition_jump_max,
        "fixed_p95_regression_m_max": args.require_fixed_p95_regression_max,
        "fused_availability_at_least_rtk": (
            True if args.require_fused_availability_at_least_rtk else None
        ),
        "fused_nonfinite_epochs_max": 0 if args.require_no_nonfinite else None,
    }
    failures: list[str] = []
    comparisons = (
        ("complete_bridges", thresholds["complete_bridges_min"], ">="),
        ("fused_bridge_coverage_pct", thresholds["fused_bridge_coverage_pct_min"], ">="),
        ("max_bridge_horizontal_error_m", thresholds["max_bridge_horizontal_error_m_max"], "<="),
        ("max_reacquisition_error_step_m", thresholds["max_reacquisition_error_step_m_max"], "<="),
        ("fixed_p95_regression_m", thresholds["fixed_p95_regression_m_max"], "<="),
        ("fused_nonfinite_epochs", thresholds["fused_nonfinite_epochs_max"], "<="),
    )
    for metric, threshold, operator in comparisons:
        if threshold is None:
            continue
        value = aggregate.get(metric)
        if value is None:
            failures.append(f"{metric} unavailable (required {operator} {threshold})")
        elif operator == ">=" and float(value) < float(threshold):
            failures.append(f"{metric} {value} < {threshold}")
        elif operator == "<=" and float(value) > float(threshold):
            failures.append(f"{metric} {value} > {threshold}")
    if thresholds["fused_availability_at_least_rtk"] is not None:
        fused_availability = aggregate.get("fused_availability_pct")
        rtk_availability = aggregate.get("rtk_solution_availability_pct")
        if fused_availability is None or rtk_availability is None:
            failures.append(
                "fused_availability_pct/rtk_solution_availability_pct unavailable "
                "(required fused availability >= RTK availability)"
            )
        elif float(fused_availability) < float(rtk_availability):
            failures.append(
                f"fused_availability_pct {fused_availability} < "
                f"rtk_solution_availability_pct {rtk_availability}"
            )
    active = any(value is not None for value in thresholds.values())
    return {
        "status": "failed" if failures else "passed" if active else "measured",
        "thresholds": thresholds,
        "failures": failures,
    }, failures


SEGMENT_FIELDS = (
    "segment_id",
    "complete_bridge",
    "entry_fixed",
    "reacquisition_fixed",
    "start_week",
    "start_tow_s",
    "end_week",
    "end_tow_s",
    "duration_s",
    "propagation_age_s",
    "reference_epochs",
    "rtk_matched_epochs",
    "rtk_statuses",
    "fused_matched_epochs",
    "fused_coverage_pct",
    "entry_horizontal_error_m",
    "median_horizontal_error_m",
    "p95_horizontal_error_m",
    "max_horizontal_error_m",
    "max_error_growth_m",
    "error_growth_rate_mps",
    "reacquisition_error_step_m",
    "fused_rtk_separation_at_reacquisition_m",
)


def write_segments(path: Path, segments: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=SEGMENT_FIELDS)
        writer.writeheader()
        for segment in segments:
            row = dict(segment)
            row["rtk_statuses"] = ";".join(str(value) for value in segment["rtk_statuses"])
            writer.writerow(row)


def main() -> int:
    args = parse_args()
    if args.match_tolerance_s < 0.0:
        raise SystemExit("--match-tolerance-s must be non-negative")
    for path, label in (
        (args.rtk_pos, "RTK solution"),
        (args.fused_pos, "fused solution"),
        (args.reference_csv, "reference CSV"),
    ):
        ensure_input_exists(path, label, ROOT_DIR)

    rtk_solutions = comparison.read_libgnss_pos(args.rtk_pos)
    fused_solutions = comparison.read_libgnss_pos(args.fused_pos)
    reference = ppc_demo.read_flexible_reference_csv(args.reference_csv)
    if not rtk_solutions or not fused_solutions or not reference:
        raise SystemExit("RTK, fused, and reference inputs must each contain epochs")
    rtk_solutions.sort(key=lambda row: absolute_time(row.week, row.tow))
    fused_solutions.sort(key=lambda row: absolute_time(row.week, row.tow))
    reference.sort(key=lambda row: absolute_time(row.week, row.tow))

    first_index, last_index = reference_window(reference, rtk_solutions)
    rtk_matches = match_solutions(reference, rtk_solutions, args.match_tolerance_s)
    fused_matches = match_solutions(reference, fused_solutions, args.match_tolerance_s)
    ranges = bridge_ranges(first_index, last_index, rtk_matches)
    segments = [
        score_bridge(
            segment_id,
            start_index,
            end_index,
            first_index,
            last_index,
            reference,
            rtk_matches,
            fused_matches,
        )
        for segment_id, (start_index, end_index) in enumerate(ranges, start=1)
    ]
    reference_count = last_index - first_index + 1
    aggregate = aggregate_summary(
        reference_count,
        rtk_matches,
        fused_matches,
        segments,
        first_index,
        last_index,
        sum(not np.isfinite(np.asarray(row.ecef, dtype=float)).all() for row in rtk_solutions),
        sum(not np.isfinite(np.asarray(row.ecef, dtype=float)).all() for row in fused_solutions),
    )
    gate, failures = gate_results(args, aggregate)
    segments_path = args.segments_csv or args.summary_json.with_name(
        f"{args.summary_json.stem}_segments.csv"
    )
    write_segments(segments_path, segments)
    payload = {
        "schema_version": SCHEMA_VERSION,
        "coordinate_frame_contract": COORDINATE_FRAME_CONTRACT,
        "evaluation_window": {
            "start_week": reference[first_index].week,
            "start_tow_s": rounded(reference[first_index].tow),
            "end_week": reference[last_index].week,
            "end_tow_s": rounded(reference[last_index].tow),
            "match_tolerance_s": args.match_tolerance_s,
        },
        "inputs": {
            "rtk_pos": file_record(args.rtk_pos),
            "fused_pos": file_record(args.fused_pos),
            "reference_csv": file_record(args.reference_csv),
        },
        "artifacts": {
            "summary_json": str(args.summary_json),
            "segments_csv": str(segments_path),
        },
        "aggregate": aggregate,
        "segments": segments,
        "gate": gate,
        "command": [os.environ.get("GNSS_CLI_NAME", "urban-bridge-score"), *sys.argv[1:]],
    }
    args.summary_json.parent.mkdir(parents=True, exist_ok=True)
    args.summary_json.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")

    print("Urban bridge score:")
    print(f"  window reference epochs: {reference_count}")
    print(f"  RTK fixed epochs: {aggregate['rtk_fixed_epochs']}")
    print(f"  degraded / complete bridges: {len(segments)} / {aggregate['complete_bridges']}")
    print(f"  fused bridge coverage: {aggregate['fused_bridge_coverage_pct']:.3f}%")
    print(f"  max bridge horizontal error: {aggregate['max_bridge_horizontal_error_m']} m")
    print(f"  max reacquisition error step: {aggregate['max_reacquisition_error_step_m']} m")
    print(f"  summary: {args.summary_json}")
    print(f"  segments: {segments_path}")
    if failures:
        for failure in failures:
            print(f"  FAIL: {failure}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
