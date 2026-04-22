#!/usr/bin/env python3
"""Analyze PPC coverage-profile quality by solution status."""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
from pathlib import Path
import sys

import numpy as np


ROOT_DIR = Path(__file__).resolve().parents[1]
SCRIPTS_DIR = Path(__file__).resolve().parent
APPS_DIR = ROOT_DIR / "apps"
if str(SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_DIR))
if str(APPS_DIR) not in sys.path:
    sys.path.insert(0, str(APPS_DIR))

import generate_driving_comparison as comparison  # noqa: E402
import gnss_ppc_metrics as ppc_metrics  # noqa: E402


STATUS_NAMES = {
    4: "FIXED",
    3: "FLOAT",
    2: "DGPS",
    1: "SPP",
}
STATUS_ORDER = ("FIXED", "FLOAT", "DGPS", "SPP")
STATUS_COLORS = {
    "FIXED": "#2ecc71",
    "FLOAT": "#f39c12",
    "DGPS": "#3498db",
    "SPP": "#e74c3c",
}
RTKLIB_STATUS_NAMES = {
    1: "FIXED",
    2: "FLOAT",
    4: "DGPS",
    5: "SPP",
}
SCORE_STATE_ORDER = ("scored", "high_error", "no_solution")


def rounded(value: float) -> float:
    return round(float(value), 6)


def percentile(values: list[float], pct: float) -> float:
    if not values:
        return 0.0
    return rounded(float(np.percentile(np.array(values, dtype=float), pct)))


def status_name(status: int) -> str:
    return STATUS_NAMES.get(status, f"status_{status}")


def rtklib_status_name(status: int) -> str:
    return RTKLIB_STATUS_NAMES.get(status, f"status_{status}")


def ordered_status_names(names: set[str]) -> list[str]:
    ordered = [name for name in STATUS_ORDER if name in names]
    ordered.extend(sorted(names.difference(STATUS_ORDER)))
    return ordered


def ordered_segment_status_names(names: set[str]) -> list[str]:
    ordered = [name for name in ("NO_SOLUTION", *STATUS_ORDER) if name in names]
    ordered.extend(sorted(names.difference({"NO_SOLUTION", *STATUS_ORDER})))
    return ordered


def optional_float(value: object) -> float | None:
    return None if value is None else float(value)


def optional_int(value: object) -> int | None:
    return None if value is None else int(value)


def rtk_diagnostic_values(records: list[dict[str, object]], key: str) -> list[float]:
    values: list[float] = []
    for record in records:
        observations = record.get("rtk_update_observations")
        if observations is None or int(observations) <= 0:
            continue
        value = record.get(key)
        if value is None:
            continue
        numeric = float(value)
        if math.isfinite(numeric):
            values.append(numeric)
    return values


def horizontal_track_distance(
    first: comparison.MatchedEpoch,
    second: comparison.MatchedEpoch,
) -> float:
    return math.hypot(
        second.traj_east_m - first.traj_east_m,
        second.traj_north_m - first.traj_north_m,
    )


def score_3d_50cm(matches: list[comparison.MatchedEpoch]) -> int:
    return sum(1 for match in matches if math.hypot(match.horiz_error_m, match.up_m) <= 0.50)


def summarize_matches(
    matches: list[comparison.MatchedEpoch],
    reference_count: int,
) -> dict[str, object]:
    horiz = [match.horiz_error_m for match in matches]
    up_abs = [abs(match.up_m) for match in matches]
    score_epochs = score_3d_50cm(matches)
    matched_count = max(len(matches), 1)
    return {
        "epochs": len(matches),
        "reference_rate_pct": rounded(100.0 * len(matches) / max(reference_count, 1)),
        "median_h_m": percentile(horiz, 50),
        "p90_h_m": percentile(horiz, 90),
        "p95_h_m": percentile(horiz, 95),
        "p99_h_m": percentile(horiz, 99),
        "max_h_m": rounded(max(horiz) if horiz else 0.0),
        "median_abs_up_m": percentile(up_abs, 50),
        "p95_abs_up_m": percentile(up_abs, 95),
        "ppc_score_3d_50cm_epochs": score_epochs,
        "ppc_score_3d_50cm_matched_pct": rounded(100.0 * score_epochs / matched_count),
        "ppc_score_3d_50cm_ref_pct": rounded(100.0 * score_epochs / max(reference_count, 1)),
    }


def summarize_by_status(
    matches: list[comparison.MatchedEpoch],
    reference_count: int,
) -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    for name in ordered_status_names({status_name(match.status) for match in matches}):
        status_matches = [match for match in matches if status_name(match.status) == name]
        row = {"status": name}
        row.update(summarize_matches(status_matches, reference_count))
        rows.append(row)
    return rows


def p95_contribution_by_status(matches: list[comparison.MatchedEpoch]) -> tuple[float, list[dict[str, object]]]:
    global_p95 = percentile([match.horiz_error_m for match in matches], 95)
    bad_matches = [match for match in matches if match.horiz_error_m >= global_p95]
    total_bad = max(len(bad_matches), 1)
    rows: list[dict[str, object]] = []
    for name in STATUS_ORDER:
        status_bad = [match for match in bad_matches if status_name(match.status) == name]
        if not status_bad:
            continue
        horiz = [match.horiz_error_m for match in status_bad]
        rows.append(
            {
                "status": name,
                "epochs": len(status_bad),
                "share_pct": rounded(100.0 * len(status_bad) / total_bad),
                "median_h_m": percentile(horiz, 50),
                "max_h_m": rounded(max(horiz)),
            }
        )
    return global_p95, rows


def paired_degradation_by_status(
    pairs: list[comparison.PairedEpoch],
) -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    for name in ordered_status_names({status_name(pair.lib_epoch.status) for pair in pairs}):
        status_pairs = [pair for pair in pairs if status_name(pair.lib_epoch.status) == name]
        deltas = [
            pair.lib_epoch.horiz_error_m - pair.rtklib_epoch.horiz_error_m
            for pair in status_pairs
        ]
        worse = [delta for delta in deltas if delta > 0.0]
        worse_5m = [delta for delta in deltas if delta > 5.0]
        rows.append(
            {
                "status": name,
                "pairs": len(status_pairs),
                "lib_worse_epochs": len(worse),
                "lib_worse_pct": rounded(100.0 * len(worse) / max(len(status_pairs), 1)),
                "lib_worse_gt_5m_epochs": len(worse_5m),
                "median_delta_h_m": percentile(deltas, 50),
                "p95_delta_h_m": percentile(deltas, 95),
            }
        )
    return rows


def official_loss_by_state(records: list[dict[str, object]]) -> list[dict[str, object]]:
    total_distance_m = sum(float(record["segment_distance_m"]) for record in records)
    rows: list[dict[str, object]] = []
    for state in SCORE_STATE_ORDER:
        state_records = [record for record in records if record["score_state"] == state]
        distance_m = sum(float(record["segment_distance_m"]) for record in state_records)
        errors = [
            float(record["error_3d_m"])
            for record in state_records
            if record["error_3d_m"] is not None
        ]
        rows.append(
            {
                "score_state": state,
                "segments": len(state_records),
                "distance_m": rounded(distance_m),
                "distance_pct": rounded(100.0 * distance_m / total_distance_m)
                if total_distance_m > 0.0
                else 0.0,
                "median_3d_m": percentile(errors, 50),
                "p95_3d_m": percentile(errors, 95),
                "max_3d_m": rounded(max(errors)) if errors else 0.0,
            }
        )
    return rows


def official_loss_by_status(
    records: list[dict[str, object]],
    status_labeler,
    score_states: tuple[str, ...] = ("high_error", "no_solution"),
) -> list[dict[str, object]]:
    total_distance_m = sum(float(record["segment_distance_m"]) for record in records)
    grouped: dict[str, list[dict[str, object]]] = {}
    for record in records:
        if str(record["score_state"]) not in score_states:
            continue
        label = official_status_label(record, status_labeler)
        grouped.setdefault(label, []).append(record)

    rows: list[dict[str, object]] = []
    for name in ordered_segment_status_names(set(grouped)):
        status_records = grouped[name]
        distance_m = sum(float(record["segment_distance_m"]) for record in status_records)
        errors = [
            float(record["error_3d_m"])
            for record in status_records
            if record["error_3d_m"] is not None
        ]
        ratios = [
            float(record["ratio"])
            for record in status_records
            if record.get("ratio") is not None and math.isfinite(float(record["ratio"]))
        ]
        rtk_iterations = rtk_diagnostic_values(status_records, "rtk_iterations")
        rtk_observations = rtk_diagnostic_values(status_records, "rtk_update_observations")
        rtk_phase_observations = rtk_diagnostic_values(
            status_records,
            "rtk_update_phase_observations",
        )
        rtk_code_observations = rtk_diagnostic_values(
            status_records,
            "rtk_update_code_observations",
        )
        rtk_suppressed_outliers = rtk_diagnostic_values(
            status_records,
            "rtk_update_suppressed_outliers",
        )
        rtk_prefit_rms = rtk_diagnostic_values(
            status_records,
            "rtk_update_prefit_residual_rms_m",
        )
        rtk_prefit_max = rtk_diagnostic_values(
            status_records,
            "rtk_update_prefit_residual_max_m",
        )
        rtk_post_suppression_rms = rtk_diagnostic_values(
            status_records,
            "rtk_update_post_suppression_residual_rms_m",
        )
        rtk_post_suppression_max = rtk_diagnostic_values(
            status_records,
            "rtk_update_post_suppression_residual_max_m",
        )
        state_distances: dict[str, float] = {}
        ratio_ge_10_distance_m = 0.0
        for record in status_records:
            state = str(record["score_state"])
            segment_distance_m = float(record["segment_distance_m"])
            state_distances[state] = state_distances.get(state, 0.0) + segment_distance_m
            ratio = record.get("ratio")
            if ratio is not None and math.isfinite(float(ratio)) and float(ratio) >= 10.0:
                ratio_ge_10_distance_m += segment_distance_m
        rows.append(
            {
                "status": name,
                "segments": len(status_records),
                "distance_m": rounded(distance_m),
                "distance_pct": rounded(100.0 * distance_m / total_distance_m)
                if total_distance_m > 0.0
                else 0.0,
                "score_state_distances_m": {
                    state: rounded(state_distances[state])
                    for state in SCORE_STATE_ORDER
                    if state in state_distances
                },
                "median_3d_m": percentile(errors, 50),
                "p95_3d_m": percentile(errors, 95),
                "max_3d_m": rounded(max(errors)) if errors else 0.0,
                "median_ratio": percentile(ratios, 50) if ratios else None,
                "p95_ratio": percentile(ratios, 95) if ratios else None,
                "max_ratio": rounded(max(ratios)) if ratios else None,
                "ratio_ge_10_distance_m": rounded(ratio_ge_10_distance_m),
                "ratio_ge_10_share_pct": rounded(100.0 * ratio_ge_10_distance_m / distance_m)
                if distance_m > 0.0
                else 0.0,
                "median_rtk_iterations": percentile(rtk_iterations, 50)
                if rtk_iterations
                else None,
                "median_rtk_update_observations": percentile(rtk_observations, 50)
                if rtk_observations
                else None,
                "median_rtk_phase_observations": percentile(rtk_phase_observations, 50)
                if rtk_phase_observations
                else None,
                "median_rtk_code_observations": percentile(rtk_code_observations, 50)
                if rtk_code_observations
                else None,
                "median_rtk_suppressed_outliers": percentile(rtk_suppressed_outliers, 50)
                if rtk_suppressed_outliers
                else None,
                "p95_rtk_suppressed_outliers": percentile(rtk_suppressed_outliers, 95)
                if rtk_suppressed_outliers
                else None,
                "median_rtk_prefit_rms_m": percentile(rtk_prefit_rms, 50)
                if rtk_prefit_rms
                else None,
                "p95_rtk_prefit_rms_m": percentile(rtk_prefit_rms, 95)
                if rtk_prefit_rms
                else None,
                "median_rtk_prefit_max_m": percentile(rtk_prefit_max, 50)
                if rtk_prefit_max
                else None,
                "p95_rtk_prefit_max_m": percentile(rtk_prefit_max, 95)
                if rtk_prefit_max
                else None,
                "median_rtk_post_suppression_rms_m": percentile(rtk_post_suppression_rms, 50)
                if rtk_post_suppression_rms
                else None,
                "p95_rtk_post_suppression_rms_m": percentile(rtk_post_suppression_rms, 95)
                if rtk_post_suppression_rms
                else None,
                "median_rtk_post_suppression_max_m": percentile(rtk_post_suppression_max, 50)
                if rtk_post_suppression_max
                else None,
                "p95_rtk_post_suppression_max_m": percentile(rtk_post_suppression_max, 95)
                if rtk_post_suppression_max
                else None,
            }
        )
    rows.sort(key=lambda row: float(row["distance_m"]), reverse=True)
    return rows


def rtk_update_diagnostic_summary(records: list[dict[str, object]]) -> dict[str, object]:
    errors = [
        float(record["error_3d_m"])
        for record in records
        if record.get("error_3d_m") is not None
    ]
    ratios = [
        float(record["ratio"])
        for record in records
        if record.get("ratio") is not None and math.isfinite(float(record["ratio"]))
    ]
    rtk_iterations = rtk_diagnostic_values(records, "rtk_iterations")
    rtk_observations = rtk_diagnostic_values(records, "rtk_update_observations")
    rtk_phase_observations = rtk_diagnostic_values(records, "rtk_update_phase_observations")
    rtk_code_observations = rtk_diagnostic_values(records, "rtk_update_code_observations")
    rtk_suppressed_outliers = rtk_diagnostic_values(records, "rtk_update_suppressed_outliers")
    rtk_prefit_rms = rtk_diagnostic_values(records, "rtk_update_prefit_residual_rms_m")
    rtk_prefit_max = rtk_diagnostic_values(records, "rtk_update_prefit_residual_max_m")
    rtk_post_suppression_rms = rtk_diagnostic_values(
        records,
        "rtk_update_post_suppression_residual_rms_m",
    )
    rtk_post_suppression_max = rtk_diagnostic_values(
        records,
        "rtk_update_post_suppression_residual_max_m",
    )
    return {
        "median_3d_m": percentile(errors, 50),
        "p95_3d_m": percentile(errors, 95),
        "max_3d_m": rounded(max(errors)) if errors else 0.0,
        "median_ratio": percentile(ratios, 50) if ratios else None,
        "p95_ratio": percentile(ratios, 95) if ratios else None,
        "median_rtk_iterations": percentile(rtk_iterations, 50)
        if rtk_iterations
        else None,
        "median_rtk_update_observations": percentile(rtk_observations, 50)
        if rtk_observations
        else None,
        "p95_rtk_update_observations": percentile(rtk_observations, 95)
        if rtk_observations
        else None,
        "median_rtk_phase_observations": percentile(rtk_phase_observations, 50)
        if rtk_phase_observations
        else None,
        "median_rtk_code_observations": percentile(rtk_code_observations, 50)
        if rtk_code_observations
        else None,
        "median_rtk_suppressed_outliers": percentile(rtk_suppressed_outliers, 50)
        if rtk_suppressed_outliers
        else None,
        "p95_rtk_suppressed_outliers": percentile(rtk_suppressed_outliers, 95)
        if rtk_suppressed_outliers
        else None,
        "median_rtk_prefit_rms_m": percentile(rtk_prefit_rms, 50)
        if rtk_prefit_rms
        else None,
        "p95_rtk_prefit_rms_m": percentile(rtk_prefit_rms, 95)
        if rtk_prefit_rms
        else None,
        "median_rtk_prefit_max_m": percentile(rtk_prefit_max, 50)
        if rtk_prefit_max
        else None,
        "p95_rtk_prefit_max_m": percentile(rtk_prefit_max, 95)
        if rtk_prefit_max
        else None,
        "median_rtk_post_suppression_rms_m": percentile(rtk_post_suppression_rms, 50)
        if rtk_post_suppression_rms
        else None,
        "p95_rtk_post_suppression_rms_m": percentile(rtk_post_suppression_rms, 95)
        if rtk_post_suppression_rms
        else None,
        "median_rtk_post_suppression_max_m": percentile(rtk_post_suppression_max, 50)
        if rtk_post_suppression_max
        else None,
        "p95_rtk_post_suppression_max_m": percentile(rtk_post_suppression_max, 95)
        if rtk_post_suppression_max
        else None,
    }


def official_rtk_update_diagnostics_by_state(
    records: list[dict[str, object]],
    status_labeler,
) -> list[dict[str, object]]:
    total_distance_m = sum(float(record["segment_distance_m"]) for record in records)
    grouped: dict[tuple[str, str], list[dict[str, object]]] = {}
    for record in records:
        if record.get("status") is None:
            continue
        observations = record.get("rtk_update_observations")
        if observations is None or int(observations) <= 0:
            continue
        label = status_labeler(int(record["status"]))
        state = str(record["score_state"])
        grouped.setdefault((label, state), []).append(record)

    rows: list[dict[str, object]] = []
    ordered_statuses = ordered_status_names({status for status, _ in grouped})
    for status in ordered_statuses:
        for state in SCORE_STATE_ORDER:
            state_records = grouped.get((status, state), [])
            if not state_records:
                continue
            distance_m = sum(float(record["segment_distance_m"]) for record in state_records)
            row: dict[str, object] = {
                "status": status,
                "score_state": state,
                "segments": len(state_records),
                "distance_m": rounded(distance_m),
                "distance_pct": rounded(100.0 * distance_m / total_distance_m)
                if total_distance_m > 0.0
                else 0.0,
            }
            row.update(rtk_update_diagnostic_summary(state_records))
            rows.append(row)
    rows.sort(
        key=lambda row: (
            str(row["status"]),
            SCORE_STATE_ORDER.index(str(row["score_state"]))
            if str(row["score_state"]) in SCORE_STATE_ORDER
            else len(SCORE_STATE_ORDER),
        )
    )
    return rows


def official_status_label(
    record: dict[str, object],
    status_labeler,
) -> str:
    status = record["status"]
    if status is None:
        return "NO_SOLUTION"
    return status_labeler(int(status))


def dominant_by_distance(distance_by_name: dict[str, float]) -> str:
    if not distance_by_name:
        return ""
    return max(distance_by_name, key=lambda name: distance_by_name[name])


def official_loss_segments(
    records: list[dict[str, object]],
    status_labeler,
    total_distance_m: float | None = None,
) -> list[dict[str, object]]:
    if total_distance_m is None:
        total_distance_m = sum(float(record["segment_distance_m"]) for record in records)

    segments: list[list[dict[str, object]]] = []
    current: list[dict[str, object]] = []
    for record in records:
        if record["score_state"] == "scored":
            if current:
                segments.append(current)
                current = []
            continue
        current.append(record)
    if current:
        segments.append(current)

    rows: list[dict[str, object]] = []
    for segment in segments:
        distance_m = sum(float(record["segment_distance_m"]) for record in segment)
        state_distances: dict[str, float] = {}
        status_counts: dict[str, int] = {}
        for record in segment:
            state = str(record["score_state"])
            state_distances[state] = state_distances.get(state, 0.0) + float(record["segment_distance_m"])
            label = official_status_label(record, status_labeler)
            status_counts[label] = status_counts.get(label, 0) + 1
        errors = [
            float(record["error_3d_m"])
            for record in segment
            if record["error_3d_m"] is not None
        ]
        ordered_statuses = ordered_segment_status_names(set(status_counts))
        rows.append(
            {
                "start_tow_s": rounded(float(segment[0]["start_tow_s"])),
                "end_tow_s": rounded(float(segment[-1]["end_tow_s"])),
                "duration_s": rounded(float(segment[-1]["end_tow_s"]) - float(segment[0]["start_tow_s"])),
                "segments": len(segment),
                "distance_m": rounded(distance_m),
                "distance_pct": rounded(100.0 * distance_m / total_distance_m)
                if total_distance_m > 0.0
                else 0.0,
                "score_state_distances_m": {
                    state: rounded(state_distances[state])
                    for state in SCORE_STATE_ORDER
                    if state in state_distances
                },
                "dominant_score_state": dominant_by_distance(state_distances),
                "statuses": ordered_statuses,
                "status_counts": {name: status_counts[name] for name in ordered_statuses},
                "dominant_status": max(ordered_statuses, key=lambda name: status_counts[name])
                if ordered_statuses
                else "",
                "median_3d_m": percentile(errors, 50),
                "p95_3d_m": percentile(errors, 95),
                "max_3d_m": rounded(max(errors)) if errors else 0.0,
            }
        )
    rows.sort(key=lambda row: float(row["distance_m"]), reverse=True)
    return rows


def official_bucket(lib_record: dict[str, object], rtklib_record: dict[str, object]) -> str:
    lib_scored = bool(lib_record["scored"])
    rtklib_scored = bool(rtklib_record["scored"])
    if lib_scored and rtklib_scored:
        return "both_scored"
    if lib_scored and not rtklib_scored:
        return "gnssplusplus_gain"
    if not lib_scored and rtklib_scored:
        return "rtklib_gain"
    if lib_record["score_state"] == "no_solution" and rtklib_record["score_state"] == "no_solution":
        return "both_no_solution"
    return "both_unscored"


def official_combined_records(
    lib_records: list[dict[str, object]],
    rtklib_records: list[dict[str, object]],
) -> list[dict[str, object]]:
    rtklib_by_index = {int(record["reference_index"]): record for record in rtklib_records}
    rows: list[dict[str, object]] = []
    for lib_record in lib_records:
        reference_index = int(lib_record["reference_index"])
        rtklib_record = rtklib_by_index.get(reference_index)
        if rtklib_record is None:
            continue
        segment_distance_m = float(lib_record["segment_distance_m"])
        lib_score_distance_m = segment_distance_m if bool(lib_record["scored"]) else 0.0
        rtklib_score_distance_m = segment_distance_m if bool(rtklib_record["scored"]) else 0.0
        lib_error = optional_float(lib_record["error_3d_m"])
        rtklib_error = optional_float(rtklib_record["error_3d_m"])
        rows.append(
            {
                "reference_index": reference_index,
                "start_tow_s": lib_record["start_tow_s"],
                "end_tow_s": lib_record["end_tow_s"],
                "segment_distance_m": segment_distance_m,
                "bucket": official_bucket(lib_record, rtklib_record),
                "score_delta_distance_m": lib_score_distance_m - rtklib_score_distance_m,
                "lib_score_state": lib_record["score_state"],
                "lib_scored": lib_record["scored"],
                "lib_status": lib_record["status"],
                "lib_error_3d_m": lib_error,
                "lib_horiz_error_m": optional_float(lib_record["horiz_error_m"]),
                "lib_up_error_m": optional_float(lib_record["up_error_m"]),
                "lib_num_satellites": lib_record["num_satellites"],
                "lib_ratio": optional_float(lib_record["ratio"]),
                "lib_baseline_m": optional_float(lib_record["baseline_m"]),
                "lib_rtk_iterations": optional_int(lib_record.get("rtk_iterations")),
                "lib_rtk_update_observations": optional_int(
                    lib_record.get("rtk_update_observations")
                ),
                "lib_rtk_update_phase_observations": optional_int(
                    lib_record.get("rtk_update_phase_observations")
                ),
                "lib_rtk_update_code_observations": optional_int(
                    lib_record.get("rtk_update_code_observations")
                ),
                "lib_rtk_update_suppressed_outliers": optional_int(
                    lib_record.get("rtk_update_suppressed_outliers")
                ),
                "lib_rtk_update_prefit_residual_rms_m": optional_float(
                    lib_record.get("rtk_update_prefit_residual_rms_m")
                ),
                "lib_rtk_update_prefit_residual_max_m": optional_float(
                    lib_record.get("rtk_update_prefit_residual_max_m")
                ),
                "lib_rtk_update_post_suppression_residual_rms_m": optional_float(
                    lib_record.get("rtk_update_post_suppression_residual_rms_m")
                ),
                "lib_rtk_update_post_suppression_residual_max_m": optional_float(
                    lib_record.get("rtk_update_post_suppression_residual_max_m")
                ),
                "rtklib_score_state": rtklib_record["score_state"],
                "rtklib_scored": rtklib_record["scored"],
                "rtklib_status": rtklib_record["status"],
                "rtklib_error_3d_m": rtklib_error,
                "rtklib_horiz_error_m": optional_float(rtklib_record["horiz_error_m"]),
                "rtklib_up_error_m": optional_float(rtklib_record["up_error_m"]),
                "rtklib_num_satellites": rtklib_record["num_satellites"],
                "rtklib_ratio": optional_float(rtklib_record["ratio"]),
                "rtklib_baseline_m": optional_float(rtklib_record["baseline_m"]),
                "error_delta_3d_m": lib_error - rtklib_error
                if lib_error is not None and rtklib_error is not None
                else None,
            }
        )
    return rows


def official_delta_by_bucket(rows: list[dict[str, object]]) -> list[dict[str, object]]:
    total_distance_m = sum(float(row["segment_distance_m"]) for row in rows)
    bucket_order = (
        "gnssplusplus_gain",
        "rtklib_gain",
        "both_scored",
        "both_unscored",
        "both_no_solution",
    )
    grouped: dict[str, list[dict[str, object]]] = {}
    for row in rows:
        grouped.setdefault(str(row["bucket"]), []).append(row)

    out: list[dict[str, object]] = []
    for bucket in bucket_order:
        bucket_rows = grouped.get(bucket, [])
        if not bucket_rows:
            continue
        distance_m = sum(float(row["segment_distance_m"]) for row in bucket_rows)
        score_delta_distance_m = sum(float(row["score_delta_distance_m"]) for row in bucket_rows)
        out.append(
            {
                "bucket": bucket,
                "segments": len(bucket_rows),
                "distance_m": rounded(distance_m),
                "distance_pct": rounded(100.0 * distance_m / total_distance_m)
                if total_distance_m > 0.0
                else 0.0,
                "score_delta_distance_m": rounded(score_delta_distance_m),
                "score_delta_pct": rounded(100.0 * score_delta_distance_m / total_distance_m)
                if total_distance_m > 0.0
                else 0.0,
            }
        )
    return out


def official_best_of_lib_rtklib_score(
    lib_score: dict[str, object],
    rows: list[dict[str, object]],
) -> dict[str, object]:
    total_distance_m = float(lib_score["ppc_official_total_distance_m"])
    lib_score_distance_m = float(lib_score["ppc_official_score_distance_m"])
    rtklib_gain_distance_m = sum(
        float(row["segment_distance_m"])
        for row in rows
        if str(row["bucket"]) == "rtklib_gain"
    )
    best_score_distance_m = lib_score_distance_m + rtklib_gain_distance_m
    remaining_distance_m = max(0.0, total_distance_m - best_score_distance_m)
    return {
        "score_distance_m": rounded(best_score_distance_m),
        "score_pct": rounded(100.0 * best_score_distance_m / total_distance_m)
        if total_distance_m > 0.0
        else 0.0,
        "rtklib_additional_distance_m": rounded(rtklib_gain_distance_m),
        "rtklib_additional_pct": rounded(100.0 * rtklib_gain_distance_m / total_distance_m)
        if total_distance_m > 0.0
        else 0.0,
        "remaining_unscored_distance_m": rounded(remaining_distance_m),
        "remaining_unscored_pct": rounded(100.0 * remaining_distance_m / total_distance_m)
        if total_distance_m > 0.0
        else 0.0,
    }


def epoch_by_tow(epochs: list[comparison.SolutionEpoch]) -> dict[float, comparison.SolutionEpoch]:
    return {rounded(epoch.tow): epoch for epoch in epochs}


def official_float_spp_divergence_sweep(
    lib_records: list[dict[str, object]],
    spp_records: list[dict[str, object]],
    lib_epochs: list[comparison.SolutionEpoch],
    spp_epochs: list[comparison.SolutionEpoch],
    thresholds_m: list[float],
) -> list[dict[str, object]]:
    total_distance_m = sum(float(record["segment_distance_m"]) for record in lib_records)
    original_score_distance_m = sum(
        float(record["segment_distance_m"]) for record in lib_records if bool(record["scored"])
    )
    spp_by_reference_index = {int(record["reference_index"]): record for record in spp_records}
    lib_by_tow = epoch_by_tow(lib_epochs)
    spp_by_tow = epoch_by_tow(spp_epochs)

    rows: list[dict[str, object]] = []
    for threshold_m in sorted(thresholds_m):
        simulated_score_distance_m = original_score_distance_m
        eligible_float_distance_m = 0.0
        rejected_distance_m = 0.0
        recovered_distance_m = 0.0
        lost_distance_m = 0.0
        rejected_state_distances: dict[str, float] = {}
        replacement_state_distances: dict[str, float] = {}

        for lib_record in lib_records:
            if lib_record["status"] != 3 or lib_record["solution_tow_s"] is None:
                continue
            tow = rounded(float(lib_record["solution_tow_s"]))
            lib_epoch = lib_by_tow.get(tow)
            spp_epoch = spp_by_tow.get(tow)
            if lib_epoch is None or spp_epoch is None:
                continue
            segment_distance_m = float(lib_record["segment_distance_m"])
            eligible_float_distance_m += segment_distance_m
            divergence_m = float(np.linalg.norm(lib_epoch.ecef - spp_epoch.ecef))
            if divergence_m <= threshold_m:
                continue

            spp_record = spp_by_reference_index.get(int(lib_record["reference_index"]))
            before_score_m = segment_distance_m if bool(lib_record["scored"]) else 0.0
            after_score_m = (
                segment_distance_m
                if spp_record is not None and bool(spp_record["scored"])
                else 0.0
            )
            simulated_score_distance_m += after_score_m - before_score_m
            if after_score_m > before_score_m:
                recovered_distance_m += after_score_m - before_score_m
            elif before_score_m > after_score_m:
                lost_distance_m += before_score_m - after_score_m

            rejected_distance_m += segment_distance_m
            rejected_state = str(lib_record["score_state"])
            rejected_state_distances[rejected_state] = (
                rejected_state_distances.get(rejected_state, 0.0) + segment_distance_m
            )
            replacement_state = str(spp_record["score_state"]) if spp_record is not None else "no_solution"
            replacement_state_distances[replacement_state] = (
                replacement_state_distances.get(replacement_state, 0.0) + segment_distance_m
            )

        score_delta_distance_m = simulated_score_distance_m - original_score_distance_m
        rows.append(
            {
                "threshold_m": rounded(threshold_m),
                "eligible_float_distance_m": rounded(eligible_float_distance_m),
                "rejected_distance_m": rounded(rejected_distance_m),
                "rejected_distance_pct": rounded(100.0 * rejected_distance_m / total_distance_m)
                if total_distance_m > 0.0
                else 0.0,
                "score_distance_m": rounded(simulated_score_distance_m),
                "score_pct": rounded(100.0 * simulated_score_distance_m / total_distance_m)
                if total_distance_m > 0.0
                else 0.0,
                "score_delta_distance_m": rounded(score_delta_distance_m),
                "score_delta_pct": rounded(100.0 * score_delta_distance_m / total_distance_m)
                if total_distance_m > 0.0
                else 0.0,
                "recovered_distance_m": rounded(recovered_distance_m),
                "lost_distance_m": rounded(lost_distance_m),
                "rejected_score_state_distances_m": {
                    state: rounded(rejected_state_distances[state])
                    for state in SCORE_STATE_ORDER
                    if state in rejected_state_distances
                },
                "replacement_score_state_distances_m": {
                    state: rounded(replacement_state_distances[state])
                    for state in SCORE_STATE_ORDER
                    if state in replacement_state_distances
                },
            }
        )
    return rows


def fixed_anchor(
    matches: list[comparison.MatchedEpoch],
    start_index: int,
    step: int,
) -> comparison.MatchedEpoch | None:
    index = start_index
    while 0 <= index < len(matches):
        if status_name(matches[index].status) == "FIXED":
            return matches[index]
        index += step
    return None


def bridge_residuals(
    segment: list[comparison.MatchedEpoch],
    anchor_before: comparison.MatchedEpoch | None,
    anchor_after: comparison.MatchedEpoch | None,
) -> list[float]:
    if anchor_before is None or anchor_after is None:
        return []
    anchor_gap_s = anchor_after.tow - anchor_before.tow
    if not math.isfinite(anchor_gap_s) or anchor_gap_s <= 0.0:
        return []

    residuals = []
    for match in segment:
        fraction = (match.tow - anchor_before.tow) / anchor_gap_s
        predicted_east = anchor_before.traj_east_m + (
            anchor_after.traj_east_m - anchor_before.traj_east_m
        ) * fraction
        predicted_north = anchor_before.traj_north_m + (
            anchor_after.traj_north_m - anchor_before.traj_north_m
        ) * fraction
        residuals.append(
            math.hypot(
                match.traj_east_m - predicted_east,
                match.traj_north_m - predicted_north,
            )
        )
    return residuals


def segment_solution_path(segment: list[comparison.MatchedEpoch]) -> float:
    return sum(
        horizontal_track_distance(segment[index - 1], segment[index])
        for index in range(1, len(segment))
    )


def segment_status_counts(segment: list[comparison.MatchedEpoch]) -> dict[str, int]:
    counts: dict[str, int] = {}
    for match in segment:
        name = status_name(match.status)
        counts[name] = counts.get(name, 0) + 1
    return {name: counts[name] for name in ordered_status_names(set(counts))}


def dominant_status(status_counts: dict[str, int]) -> str:
    ordered_names = ordered_status_names(set(status_counts))
    return max(ordered_names, key=lambda name: status_counts[name]) if ordered_names else ""


def segment_anchor_features(
    matches: list[comparison.MatchedEpoch],
    segment_indexes: list[int],
) -> dict[str, object]:
    segment = [matches[index] for index in segment_indexes]
    duration_s = segment[-1].tow - segment[0].tow
    solution_path_m = segment_solution_path(segment)
    status_counts = segment_status_counts(segment)
    anchor_before = fixed_anchor(matches, segment_indexes[0] - 1, -1)
    anchor_after = fixed_anchor(matches, segment_indexes[-1] + 1, 1)
    residuals = bridge_residuals(segment, anchor_before, anchor_after)

    features: dict[str, object] = {
        "status_counts": status_counts,
        "dominant_status": dominant_status(status_counts),
        "solution_path_m": rounded(solution_path_m),
        "solution_chord_m": rounded(
            horizontal_track_distance(segment[0], segment[-1]) if len(segment) > 1 else 0.0
        ),
        "solution_path_speed_mps": rounded(solution_path_m / duration_s)
        if duration_s > 0.0
        else None,
        "previous_fixed_tow_s": rounded(anchor_before.tow) if anchor_before is not None else None,
        "next_fixed_tow_s": rounded(anchor_after.tow) if anchor_after is not None else None,
        "fixed_anchor_gap_s": None,
        "fixed_anchor_distance_m": None,
        "fixed_anchor_speed_mps": None,
        "fixed_anchor_bridge_residual_median_m": None,
        "fixed_anchor_bridge_residual_p95_m": None,
        "fixed_anchor_bridge_residual_max_m": None,
    }
    if anchor_before is not None and anchor_after is not None:
        anchor_gap_s = anchor_after.tow - anchor_before.tow
        anchor_distance_m = horizontal_track_distance(anchor_before, anchor_after)
        features.update(
            {
                "fixed_anchor_gap_s": rounded(anchor_gap_s),
                "fixed_anchor_distance_m": rounded(anchor_distance_m),
                "fixed_anchor_speed_mps": rounded(anchor_distance_m / anchor_gap_s)
                if anchor_gap_s > 0.0
                else None,
                "fixed_anchor_bridge_residual_median_m": percentile(residuals, 50),
                "fixed_anchor_bridge_residual_p95_m": percentile(residuals, 95),
                "fixed_anchor_bridge_residual_max_m": rounded(max(residuals)) if residuals else 0.0,
            }
        )
    return features


def bad_segments(
    matches: list[comparison.MatchedEpoch],
    threshold_m: float,
    max_gap_s: float,
) -> list[dict[str, object]]:
    bad_indexes = [index for index, match in enumerate(matches) if match.horiz_error_m > threshold_m]
    segments: list[list[int]] = []
    current: list[int] = []
    for index in bad_indexes:
        if not current or matches[index].tow - matches[current[-1]].tow <= max_gap_s:
            current.append(index)
            continue
        segments.append(current)
        current = [index]
    if current:
        segments.append(current)

    rows: list[dict[str, object]] = []
    for segment_indexes in segments:
        segment = [matches[index] for index in segment_indexes]
        horiz = [match.horiz_error_m for match in segment]
        statuses = sorted({status_name(match.status) for match in segment})
        row: dict[str, object] = {
            "start_tow_s": rounded(segment[0].tow),
            "end_tow_s": rounded(segment[-1].tow),
            "duration_s": rounded(segment[-1].tow - segment[0].tow),
            "epochs": len(segment),
            "statuses": statuses,
            "median_h_m": percentile(horiz, 50),
            "p95_h_m": percentile(horiz, 95),
            "max_h_m": rounded(max(horiz)),
        }
        row.update(segment_anchor_features(matches, segment_indexes))
        rows.append(row)
    rows.sort(key=lambda row: (int(row["epochs"]), float(row["max_h_m"])), reverse=True)
    return rows


def build_report(
    *,
    lib_pos: Path,
    rtklib_pos: Path,
    reference_csv: Path,
    match_tolerance_s: float,
    bad_h_threshold_m: float,
    bad_gap_s: float,
    spp_pos: Path | None = None,
    float_spp_div_thresholds_m: list[float] | None = None,
) -> dict[str, object]:
    reference = comparison.read_reference_csv(reference_csv)
    lib_epochs = comparison.read_libgnss_pos(lib_pos)
    rtklib_epochs = comparison.read_rtklib_pos(rtklib_pos)
    spp_epochs = comparison.read_libgnss_pos(spp_pos) if spp_pos is not None else []
    lib_matches = comparison.match_to_reference(
        lib_epochs,
        reference,
        match_tolerance_s,
    )
    rtklib_matches = comparison.match_to_reference(
        rtklib_epochs,
        reference,
        match_tolerance_s,
    )
    if not lib_matches:
        raise SystemExit("No gnssplusplus epochs matched the PPC reference.")
    if not rtklib_matches:
        raise SystemExit("No RTKLIB epochs matched the PPC reference.")

    pairs = comparison.pair_epochs(lib_matches, rtklib_matches, match_tolerance_s)
    global_p95_h_m, contribution_rows = p95_contribution_by_status(lib_matches)
    lib_official_records = ppc_metrics.ppc_official_segment_records(
        reference,
        lib_epochs,
        match_tolerance_s,
    )
    rtklib_official_records = ppc_metrics.ppc_official_segment_records(
        reference,
        rtklib_epochs,
        match_tolerance_s,
    )
    official_records = official_combined_records(lib_official_records, rtklib_official_records)
    official_score = ppc_metrics.ppc_official_distance_score(
        reference,
        lib_epochs,
        match_tolerance_s,
    )
    rtklib_official_score = ppc_metrics.ppc_official_distance_score(
        reference,
        rtklib_epochs,
        match_tolerance_s,
    )
    report: dict[str, object] = {
        "reference_epochs": len(reference),
        "lib_matched_epochs": len(lib_matches),
        "rtklib_matched_epochs": len(rtklib_matches),
        "common_epoch_pairs": len(pairs),
        "match_tolerance_s": match_tolerance_s,
        "bad_h_threshold_m": bad_h_threshold_m,
        "global_p95_h_m": global_p95_h_m,
        "lib_by_status": summarize_by_status(lib_matches, len(reference)),
        "p95_contribution_by_status": contribution_rows,
        "paired_delta_by_status": paired_degradation_by_status(pairs),
        "official_score": official_score,
        "rtklib_official_score": rtklib_official_score,
        "official_best_of_lib_rtklib_score": official_best_of_lib_rtklib_score(
            official_score,
            official_records,
        ),
        "official_loss_by_state": official_loss_by_state(lib_official_records),
        "rtklib_official_loss_by_state": official_loss_by_state(rtklib_official_records),
        "official_unscored_by_status": official_loss_by_status(
            lib_official_records,
            status_name,
            ("high_error", "no_solution"),
        ),
        "official_high_error_by_status": official_loss_by_status(
            lib_official_records,
            status_name,
            ("high_error",),
        ),
        "official_rtk_update_diagnostics_by_state": official_rtk_update_diagnostics_by_state(
            lib_official_records,
            status_name,
        ),
        "official_delta_by_bucket": official_delta_by_bucket(official_records),
        "official_loss_segments": official_loss_segments(lib_official_records, status_name)[:12],
        "rtklib_official_loss_segments": official_loss_segments(
            rtklib_official_records,
            rtklib_status_name,
        )[:12],
        "bad_segments": bad_segments(lib_matches, bad_h_threshold_m, bad_gap_s),
        "_official_segment_records": official_records,
    }
    if spp_pos is not None and float_spp_div_thresholds_m:
        spp_official_records = ppc_metrics.ppc_official_segment_records(
            reference,
            spp_epochs,
            match_tolerance_s,
        )
        report["official_float_spp_divergence_sweep"] = official_float_spp_divergence_sweep(
            lib_official_records,
            spp_official_records,
            lib_epochs,
            spp_epochs,
            float_spp_div_thresholds_m,
        )
    return report


def write_segments_csv(path: Path, rows: list[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = [
        "start_tow_s",
        "end_tow_s",
        "duration_s",
        "epochs",
        "statuses",
        "status_counts",
        "dominant_status",
        "median_h_m",
        "p95_h_m",
        "max_h_m",
        "solution_path_m",
        "solution_chord_m",
        "solution_path_speed_mps",
        "previous_fixed_tow_s",
        "next_fixed_tow_s",
        "fixed_anchor_gap_s",
        "fixed_anchor_distance_m",
        "fixed_anchor_speed_mps",
        "fixed_anchor_bridge_residual_median_m",
        "fixed_anchor_bridge_residual_p95_m",
        "fixed_anchor_bridge_residual_max_m",
    ]
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames, lineterminator="\n")
        writer.writeheader()
        for row in rows:
            serialized = dict(row)
            serialized["statuses"] = ";".join(str(item) for item in row["statuses"])
            serialized["status_counts"] = ";".join(
                f"{name}:{count}" for name, count in dict(row["status_counts"]).items()
            )
            for key, value in list(serialized.items()):
                if value is None:
                    serialized[key] = ""
            writer.writerow(serialized)


def write_official_segments_csv(path: Path, rows: list[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = [
        "reference_index",
        "start_tow_s",
        "end_tow_s",
        "segment_distance_m",
        "bucket",
        "score_delta_distance_m",
        "lib_score_state",
        "lib_scored",
        "lib_status",
        "lib_status_name",
        "lib_error_3d_m",
        "lib_horiz_error_m",
        "lib_up_error_m",
        "lib_num_satellites",
        "lib_ratio",
        "lib_baseline_m",
        "lib_rtk_iterations",
        "lib_rtk_update_observations",
        "lib_rtk_update_phase_observations",
        "lib_rtk_update_code_observations",
        "lib_rtk_update_suppressed_outliers",
        "lib_rtk_update_prefit_residual_rms_m",
        "lib_rtk_update_prefit_residual_max_m",
        "lib_rtk_update_post_suppression_residual_rms_m",
        "lib_rtk_update_post_suppression_residual_max_m",
        "rtklib_score_state",
        "rtklib_scored",
        "rtklib_status",
        "rtklib_status_name",
        "rtklib_error_3d_m",
        "rtklib_horiz_error_m",
        "rtklib_up_error_m",
        "rtklib_num_satellites",
        "rtklib_ratio",
        "rtklib_baseline_m",
        "error_delta_3d_m",
    ]
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames, lineterminator="\n")
        writer.writeheader()
        for row in rows:
            serialized = dict(row)
            serialized["lib_status_name"] = (
                status_name(int(row["lib_status"])) if row["lib_status"] is not None else ""
            )
            serialized["rtklib_status_name"] = (
                rtklib_status_name(int(row["rtklib_status"])) if row["rtklib_status"] is not None else ""
            )
            for key, value in list(serialized.items()):
                if value is None:
                    serialized[key] = ""
                elif isinstance(value, float):
                    serialized[key] = rounded(value)
            writer.writerow(serialized)


def public_report(report: dict[str, object]) -> dict[str, object]:
    return {key: value for key, value in report.items() if not key.startswith("_")}


def format_status_table(rows: list[dict[str, object]]) -> str:
    lines = ["status   epochs   p50H   p95H   3D50/ref"]
    for row in rows:
        lines.append(
            f"{row['status']:<7} {int(row['epochs']):>6} "
            f"{float(row['median_h_m']):>6.2f} "
            f"{float(row['p95_h_m']):>6.2f} "
            f"{float(row['ppc_score_3d_50cm_ref_pct']):>8.1f}%"
        )
    return "\n".join(lines)


def format_official_loss_table(rows: list[dict[str, object]]) -> str:
    labels = {
        "scored": "scored",
        "high_error": ">50cm",
        "no_solution": "no-sol",
    }
    lines = ["state      dist(m)  share   p95_3D"]
    for row in rows:
        lines.append(
            f"{labels.get(str(row['score_state']), str(row['score_state'])):<9} "
            f"{float(row['distance_m']):>7.1f} "
            f"{float(row['distance_pct']):>5.1f}% "
            f"{float(row['p95_3d_m']):>7.2f}"
        )
    return "\n".join(lines)


def format_official_delta_table(rows: list[dict[str, object]]) -> str:
    labels = {
        "gnssplusplus_gain": "g++ gain",
        "rtklib_gain": "rtk gain",
        "both_scored": "both ok",
        "both_unscored": "both bad",
        "both_no_solution": "both none",
    }
    lines = ["bucket       dist(m)  delta"]
    for row in rows:
        lines.append(
            f"{labels.get(str(row['bucket']), str(row['bucket'])):<11} "
            f"{float(row['distance_m']):>7.1f} "
            f"{float(row['score_delta_pct']):>+5.1f}pp"
        )
    return "\n".join(lines)


def format_segment_line(row: dict[str, object]) -> str:
    prefix = (
        f"{row['start_tow_s']:.1f}-{row['end_tow_s']:.1f}s "
        f"n={int(row['epochs'])} max={float(row['max_h_m']):.1f} "
        f"{','.join(row['statuses'])}"
    )
    if row["fixed_anchor_speed_mps"] is None:
        return f"{prefix} no-anchor"
    return (
        f"{prefix} a={float(row['fixed_anchor_speed_mps']):.1f}m/s "
        f"b95={float(row['fixed_anchor_bridge_residual_p95_m']):.1f}m"
    )


def format_official_segment_line(row: dict[str, object]) -> str:
    status_text = ",".join(str(status) for status in row["statuses"])
    return (
        f"{float(row['start_tow_s']):.1f}-{float(row['end_tow_s']):.1f}s "
        f"{float(row['distance_m']):.1f}m "
        f"{row['dominant_score_state']} {status_text} "
        f"max3D={float(row['max_3d_m']):.1f}m"
    )


def render_png(report: dict[str, object], output: Path, title: str) -> None:
    import matplotlib.pyplot as plt

    by_status = list(report["lib_by_status"])
    official_loss = list(report["official_loss_by_state"])
    official_delta = list(report["official_delta_by_bucket"])
    official_segments = list(report["official_loss_segments"])[:6]

    fig = plt.figure(figsize=(13.5, 7.2), dpi=100, facecolor="#f4efe6")
    grid = fig.add_gridspec(2, 2, height_ratios=[0.30, 1.0], hspace=0.28, wspace=0.22)

    ax_head = fig.add_subplot(grid[0, :])
    ax_head.set_axis_off()
    official = dict(report["official_score"])
    rtklib_official = dict(report["rtklib_official_score"])
    ax_head.text(0.0, 0.75, title, fontsize=25, weight="bold", color="#14213d")
    ax_head.text(
        0.0,
        0.34,
        (
            f"matched={report['lib_matched_epochs']}/{report['reference_epochs']}  "
            f"official={float(official['ppc_official_score_pct']):.1f}%  "
            f"RTKLIB={float(rtklib_official['ppc_official_score_pct']):.1f}%  "
            f"global p95H={float(report['global_p95_h_m']):.2f} m  "
            f"bad threshold={float(report['bad_h_threshold_m']):.1f} m"
        ),
        fontsize=12.5,
        color="#5f6c7b",
    )

    ax_status = fig.add_subplot(grid[1, 0])
    statuses = [str(row["status"]) for row in by_status]
    p95 = [float(row["p95_h_m"]) for row in by_status]
    ax_status.bar(statuses, p95, color=[STATUS_COLORS.get(name, "#64748b") for name in statuses])
    ax_status.set_title("Horizontal p95 by status", fontsize=14, weight="bold", color="#14213d")
    ax_status.set_ylabel("p95 H error (m)", color="#5f6c7b")
    ax_status.tick_params(colors="#5f6c7b")
    ax_status.grid(axis="y", alpha=0.20)
    for index, value in enumerate(p95):
        ax_status.text(index, value + max(p95) * 0.025, f"{value:.1f}", ha="center", fontsize=10)

    ax_table = fig.add_subplot(grid[1, 1])
    ax_table.set_axis_off()
    ax_table.text(
        0.0,
        1.0,
        "Status summary",
        fontsize=14,
        weight="bold",
        color="#14213d",
        va="top",
    )
    ax_table.text(
        0.0,
        0.88,
        format_status_table(by_status),
        family="monospace",
        fontsize=10.5,
        color="#14213d",
        va="top",
    )

    ax_table.text(
        0.0,
        0.36,
        "Official distance score loss",
        fontsize=12.5,
        weight="bold",
        color="#14213d",
        va="top",
    )
    ax_table.text(
        0.0,
        0.27,
        format_official_loss_table(official_loss),
        family="monospace",
        fontsize=9.7,
        color="#14213d",
        va="top",
    )

    ax_table.text(
        0.48,
        -0.03,
        "Official delta buckets",
        fontsize=12.5,
        weight="bold",
        color="#14213d",
        va="top",
    )
    ax_table.text(
        0.48,
        -0.12,
        format_official_delta_table(official_delta),
        family="monospace",
        fontsize=9.2,
        color="#14213d",
        va="top",
    )

    segment_lines = [format_official_segment_line(row) for row in official_segments]
    ax_table.text(
        0.0,
        -0.42,
        "Largest official loss intervals",
        fontsize=12.5,
        weight="bold",
        color="#14213d",
        va="top",
    )
    ax_table.text(
        0.0,
        -0.51,
        "\n".join(segment_lines),
        family="monospace",
        fontsize=8.8,
        color="#14213d",
        va="top",
    )

    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=100, facecolor=fig.get_facecolor(), bbox_inches="tight")
    print(f"Saved: {output}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(prog=os.environ.get("GNSS_CLI_NAME"))
    parser.add_argument("--lib-pos", type=Path, required=True)
    parser.add_argument("--rtklib-pos", type=Path, required=True)
    parser.add_argument("--reference-csv", type=Path, required=True)
    parser.add_argument("--spp-pos", type=Path, default=None)
    parser.add_argument(
        "--float-spp-div-thresholds",
        default=None,
        help="Comma-separated FLOAT-vs-SPP divergence thresholds in meters.",
    )
    parser.add_argument("--summary-json", type=Path, default=None)
    parser.add_argument("--segments-csv", type=Path, default=None)
    parser.add_argument("--official-segments-csv", type=Path, default=None)
    parser.add_argument("--output-png", type=Path, default=None)
    parser.add_argument("--title", default="PPC coverage quality by status")
    parser.add_argument("--match-tolerance-s", type=float, default=0.25)
    parser.add_argument("--bad-h-threshold-m", type=float, default=30.0)
    parser.add_argument("--bad-gap-s", type=float, default=0.6)
    return parser.parse_args()


def parse_thresholds(text: str | None) -> list[float] | None:
    if text is None:
        return None
    thresholds = [float(part.strip()) for part in text.split(",") if part.strip()]
    if not thresholds:
        raise SystemExit("--float-spp-div-thresholds did not contain any numeric thresholds.")
    if any(threshold < 0.0 for threshold in thresholds):
        raise SystemExit("--float-spp-div-thresholds values must be >= 0.")
    return thresholds


def main() -> int:
    args = parse_args()
    float_spp_div_thresholds = parse_thresholds(args.float_spp_div_thresholds)
    if float_spp_div_thresholds is not None and args.spp_pos is None:
        raise SystemExit("--float-spp-div-thresholds requires --spp-pos.")
    report = build_report(
        lib_pos=args.lib_pos,
        rtklib_pos=args.rtklib_pos,
        reference_csv=args.reference_csv,
        match_tolerance_s=args.match_tolerance_s,
        bad_h_threshold_m=args.bad_h_threshold_m,
        bad_gap_s=args.bad_gap_s,
        spp_pos=args.spp_pos,
        float_spp_div_thresholds_m=float_spp_div_thresholds,
    )
    if args.summary_json is not None:
        args.summary_json.parent.mkdir(parents=True, exist_ok=True)
        args.summary_json.write_text(
            json.dumps(public_report(report), indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        print(f"Saved: {args.summary_json}")
    if args.segments_csv is not None:
        write_segments_csv(args.segments_csv, list(report["bad_segments"]))
        print(f"Saved: {args.segments_csv}")
    if args.official_segments_csv is not None:
        write_official_segments_csv(
            args.official_segments_csv,
            list(report["_official_segment_records"]),
        )
        print(f"Saved: {args.official_segments_csv}")
    if args.output_png is not None:
        render_png(report, args.output_png, args.title)
    if (
        args.summary_json is None
        and args.segments_csv is None
        and args.official_segments_csv is None
        and args.output_png is None
    ):
        print(json.dumps(public_report(report), indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
