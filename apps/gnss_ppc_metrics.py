#!/usr/bin/env python3
"""Small PPC comparison metric helpers shared by CLI wrappers."""

from __future__ import annotations

import bisect
import csv
import math
from pathlib import Path
import sys
from typing import Any


ROOT_DIR = Path(__file__).resolve().parent.parent
SCRIPTS_DIR = ROOT_DIR / "scripts"
WGS84_A = 6378137.0
WGS84_E2 = 6.69437999014e-3

if str(SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_DIR))

import generate_driving_comparison as comparison  # noqa: E402


def rounded(value: float) -> float:
    return round(value, 6)


def week_tow_to_seconds(week: int, tow: float) -> float:
    return week * 604800.0 + tow


def solution_span_seconds(epochs: list[comparison.SolutionEpoch]) -> float:
    if len(epochs) < 2:
        return 0.0
    first = week_tow_to_seconds(epochs[0].week, epochs[0].tow)
    last = week_tow_to_seconds(epochs[-1].week, epochs[-1].tow)
    return max(0.0, last - first)


def reference_segment_distances(reference: list[comparison.ReferenceEpoch]) -> list[float]:
    distances = [0.0]
    for index in range(1, len(reference)):
        distances.append(float(math.dist(reference[index - 1].ecef, reference[index].ecef)))
    return distances


def ppc_official_distance_score(
    reference: list[comparison.ReferenceEpoch],
    solution_epochs: list[comparison.SolutionEpoch],
    match_tolerance_s: float,
    threshold_m: float = 0.50,
) -> dict[str, object]:
    """PPC official-style score: driven-distance ratio with 3D error <= threshold."""
    if len(reference) < 2:
        return {
            "ppc_official_score_threshold_m": rounded(threshold_m),
            "ppc_official_total_distance_m": 0.0,
            "ppc_official_matched_distance_m": 0.0,
            "ppc_official_score_distance_m": 0.0,
            "ppc_official_score_pct": 0.0,
        }

    reference_tows = [epoch.tow for epoch in reference]
    best_error_by_reference_index: dict[int, float] = {}
    for solution in solution_epochs:
        index = bisect.bisect_left(reference_tows, solution.tow)
        candidates = [
            candidate
            for candidate in (index - 1, index, index + 1)
            if 0 <= candidate < len(reference)
        ]
        if not candidates:
            continue
        ref_index = min(candidates, key=lambda candidate: abs(reference[candidate].tow - solution.tow))
        ref = reference[ref_index]
        if ref.week != solution.week or abs(ref.tow - solution.tow) > match_tolerance_s:
            continue
        error_3d_m = float(math.dist(solution.ecef, ref.ecef))
        previous = best_error_by_reference_index.get(ref_index)
        if previous is None or error_3d_m < previous:
            best_error_by_reference_index[ref_index] = error_3d_m

    segment_distances = reference_segment_distances(reference)
    total_distance_m = sum(segment_distances)
    matched_distance_m = 0.0
    score_distance_m = 0.0
    for ref_index, distance_m in enumerate(segment_distances):
        if ref_index == 0:
            continue
        error = best_error_by_reference_index.get(ref_index)
        if error is None:
            continue
        matched_distance_m += distance_m
        if error <= threshold_m:
            score_distance_m += distance_m

    score_pct = 100.0 * score_distance_m / total_distance_m if total_distance_m > 0.0 else 0.0
    return {
        "ppc_official_score_threshold_m": rounded(threshold_m),
        "ppc_official_total_distance_m": rounded(total_distance_m),
        "ppc_official_matched_distance_m": rounded(matched_distance_m),
        "ppc_official_score_distance_m": rounded(score_distance_m),
        "ppc_official_score_pct": rounded(score_pct),
    }


def llh_from_ecef(ecef_x_m: float, ecef_y_m: float, ecef_z_m: float) -> tuple[float, float, float]:
    longitude = math.atan2(ecef_y_m, ecef_x_m)
    p = math.hypot(ecef_x_m, ecef_y_m)
    latitude = math.atan2(ecef_z_m, p * (1.0 - WGS84_E2))
    for _ in range(6):
        sin_lat = math.sin(latitude)
        n = WGS84_A / math.sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat)
        height = p / max(math.cos(latitude), 1e-12) - n
        latitude = math.atan2(ecef_z_m, p * (1.0 - WGS84_E2 * n / (n + height)))
    sin_lat = math.sin(latitude)
    n = WGS84_A / math.sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat)
    height = p / max(math.cos(latitude), 1e-12) - n
    return math.degrees(latitude), math.degrees(longitude), height


def summarize_solution_epochs(
    reference: list[comparison.ReferenceEpoch],
    solution_epochs: list[comparison.SolutionEpoch],
    fixed_status: int,
    label: str,
    match_tolerance_s: float,
    solver_wall_time_s: float | None,
) -> dict[str, object]:
    if not solution_epochs:
        raise SystemExit(f"No solution epochs found for {label}")
    matched = comparison.match_to_reference(solution_epochs, reference, match_tolerance_s)
    if not matched:
        raise SystemExit(f"No PPC epochs matched reference for {label}")

    summary = comparison.summarize(matched, fixed_status, label)
    mean_h_m = sum(epoch.horiz_error_m for epoch in matched) / len(matched)
    reference_count = max(len(reference), 1)
    ppc_score_3d_50cm_epochs = sum(
        1
        for epoch in matched
        if math.hypot(epoch.horiz_error_m, epoch.up_m) <= 0.50
    )
    ppc_official_score = ppc_official_distance_score(
        reference,
        solution_epochs,
        match_tolerance_s,
        threshold_m=0.50,
    )
    matched_fixed_epochs = sum(1 for epoch in matched if epoch.status == fixed_status)
    mean_satellites = sum(epoch.num_satellites for epoch in solution_epochs) / len(solution_epochs)
    valid_span_s = solution_span_seconds(solution_epochs)

    payload: dict[str, object] = {
        "valid_epochs": len(solution_epochs),
        "matched_epochs": len(matched),
        "fixed_epochs": matched_fixed_epochs,
        "positioning_rate_pct": rounded(100.0 * len(matched) / reference_count),
        "fix_rate_pct": rounded(float(summary["fix_rate_pct"])),
        "mean_h_m": rounded(mean_h_m),
        "median_h_m": rounded(float(summary["median_h_m"])),
        "p95_h_m": rounded(float(summary["p95_h_m"])),
        "max_h_m": rounded(float(summary["max_h_m"])),
        "ppc_score_3d_50cm_epochs": ppc_score_3d_50cm_epochs,
        "ppc_score_3d_50cm_matched_pct": rounded(
            100.0 * ppc_score_3d_50cm_epochs / len(matched)
        ),
        "ppc_score_3d_50cm_ref_pct": rounded(
            100.0 * ppc_score_3d_50cm_epochs / reference_count
        ),
        **ppc_official_score,
        "median_abs_up_m": rounded(float(summary["median_abs_up_m"])),
        "p95_abs_up_m": rounded(float(summary["p95_abs_up_m"])),
        "mean_up_m": rounded(float(summary["mean_up_m"])),
        "mean_satellites": rounded(mean_satellites),
        "solution_span_s": rounded(valid_span_s),
        "solver_wall_time_s": rounded(solver_wall_time_s) if solver_wall_time_s is not None else None,
        "realtime_factor": None,
        "effective_epoch_rate_hz": None,
    }
    if solver_wall_time_s is not None and solver_wall_time_s > 0.0:
        payload["effective_epoch_rate_hz"] = rounded(len(solution_epochs) / solver_wall_time_s)
        if valid_span_s > 0.0:
            payload["realtime_factor"] = rounded(valid_span_s / solver_wall_time_s)
    return payload


def write_reference_matches_csv(
    path: Path,
    matches: list[comparison.MatchedEpoch],
) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(
            [
                "gps_tow_s",
                "traj_east_m",
                "traj_north_m",
                "traj_up_m",
                "east_error_m",
                "north_error_m",
                "up_error_m",
                "horizontal_error_m",
                "status",
            ]
        )
        for match in matches:
            writer.writerow(
                [
                    f"{match.tow:.3f}",
                    f"{match.traj_east_m:.6f}",
                    f"{match.traj_north_m:.6f}",
                    f"{match.traj_up_m:.6f}",
                    f"{match.east_m:.6f}",
                    f"{match.north_m:.6f}",
                    f"{match.up_m:.6f}",
                    f"{match.horiz_error_m:.6f}",
                    int(match.status),
                ]
            )


def solution_metric_delta(
    left: dict[str, Any],
    right: dict[str, Any],
) -> dict[str, object]:
    def optional_delta(key: str) -> float | None:
        left_value = left.get(key)
        right_value = right.get(key)
        if left_value is None or right_value is None:
            return None
        return rounded(float(left_value) - float(right_value))

    return {
        "valid_epochs": int(left["valid_epochs"]) - int(right["valid_epochs"]),
        "matched_epochs": int(left["matched_epochs"]) - int(right["matched_epochs"]),
        "fixed_epochs": int(left["fixed_epochs"]) - int(right["fixed_epochs"]),
        "positioning_rate_pct": optional_delta("positioning_rate_pct"),
        "fix_rate_pct": optional_delta("fix_rate_pct"),
        "mean_h_m": optional_delta("mean_h_m"),
        "median_h_m": optional_delta("median_h_m"),
        "p95_h_m": optional_delta("p95_h_m"),
        "max_h_m": optional_delta("max_h_m"),
        "ppc_score_3d_50cm_matched_pct": optional_delta("ppc_score_3d_50cm_matched_pct"),
        "ppc_score_3d_50cm_ref_pct": optional_delta("ppc_score_3d_50cm_ref_pct"),
        "ppc_official_score_pct": optional_delta("ppc_official_score_pct"),
        "ppc_official_matched_distance_m": optional_delta("ppc_official_matched_distance_m"),
        "ppc_official_score_distance_m": optional_delta("ppc_official_score_distance_m"),
        "median_abs_up_m": optional_delta("median_abs_up_m"),
        "p95_abs_up_m": optional_delta("p95_abs_up_m"),
        "solver_wall_time_s": optional_delta("solver_wall_time_s"),
        "realtime_factor": optional_delta("realtime_factor"),
    }
