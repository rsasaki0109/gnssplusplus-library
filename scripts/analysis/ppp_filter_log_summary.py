#!/usr/bin/env python3
"""Summarize gnss_ppp --ppp-filter-log and --ppp-residual-log CSV files."""

from __future__ import annotations

import argparse
import csv
import json
import math
from collections import Counter
from decimal import Decimal, InvalidOperation, ROUND_HALF_UP
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple


FILTER_REQUIRED_COLUMNS = [
    "week",
    "tow",
    "iteration",
    "rows",
    "code_rows",
    "phase_rows",
    "ionosphere_constraint_rows",
]

RESIDUAL_REQUIRED_COLUMNS = [
    "week",
    "tow",
    "iteration",
    "row_index",
    "sat",
    "row_type",
    "residual_m",
    "iono_state_m",
]

CORRECTION_REQUIRED_COLUMNS = [
    "week",
    "tow",
    "sat",
    "primary_signal",
    "secondary_signal",
    "primary_observation_code",
    "secondary_observation_code",
    "ssr_available",
    "orbit_clock_applied",
    "clock_m",
    "code_bias_m",
    "phase_bias_m",
    "variance_pr",
    "variance_cp",
    "valid_after_corrections",
]

CORRECTION_FLOAT_COLUMNS = [
    "orbit_dx_m",
    "orbit_dy_m",
    "orbit_dz_m",
    "clock_m",
    "ura_sigma_m",
    "code_bias_m",
    "phase_bias_m",
    "ionosphere_coefficient",
    "trop_m",
    "stec_tecu",
    "iono_m",
    "dcb_bias_m",
    "ionex_iono_m",
    "elevation_deg",
    "variance_pr",
    "variance_cp",
]

CORRECTION_INT_COLUMNS = [
    "ssr_available",
    "ssr_orbit_iode",
    "broadcast_iode",
    "orbit_clock_applied",
    "frequency_index",
    "has_carrier_phase",
    "ionosphere_estimation_constraint",
    "dcb_applied",
    "ionex_applied",
    "atmos_token_count",
    "valid_after_corrections",
    "solution_status",
]

CORRECTION_TEXT_COLUMNS = [
    "orbit_clock_skip_reason",
    "preferred_network_id",
]

Row = Dict[str, str]
EpochKey = Tuple[int, int]
CorrectionKey = Tuple[int, int, str, str, str, str, str]
FilterKey = Tuple[int, int, int]


def tow_to_millis(value: str) -> int:
    try:
        tow = Decimal(str(value))
    except InvalidOperation as exc:
        raise ValueError(f"invalid tow value: {value}") from exc
    return int((tow * Decimal("1000")).to_integral_value(rounding=ROUND_HALF_UP))


def epoch_key(row: Row) -> EpochKey:
    return (int(float(row["week"])), tow_to_millis(row["tow"]))


def to_int(value: str) -> int:
    if value == "":
        return 0
    try:
        return int(float(value))
    except ValueError:
        return 0


def to_float(value: str) -> float:
    if value == "":
        return 0.0
    try:
        parsed = float(value)
    except ValueError:
        return 0.0
    return parsed if math.isfinite(parsed) else 0.0


def read_csv(path: Path, required_columns: Sequence[str]) -> List[Row]:
    with path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        if reader.fieldnames is None:
            raise ValueError(f"{path}: missing CSV header")
        missing = [column for column in required_columns if column not in reader.fieldnames]
        if missing:
            raise ValueError(f"{path}: missing required columns: {', '.join(missing)}")
        return [dict(row) for row in reader]


def residual_row_metadata(row: Row, *, prefix: str = "") -> Dict[str, Any]:
    return {
        f"{prefix}primary_signal": row.get("primary_signal", ""),
        f"{prefix}secondary_signal": row.get("secondary_signal", ""),
        f"{prefix}primary_observation_code": row.get("primary_observation_code", ""),
        f"{prefix}secondary_observation_code": row.get("secondary_observation_code", ""),
        f"{prefix}frequency_index": to_int(row.get("frequency_index", "0")),
        f"{prefix}ionosphere_coefficient": to_float(row.get("ionosphere_coefficient", "1")),
        f"{prefix}innovation_variance_m2": to_float(row.get("innovation_variance_m2", "0")),
        f"{prefix}innovation_inverse_diagonal_1_per_m2": to_float(
            row.get("innovation_inverse_diagonal_1_per_m2", "0")
        ),
        f"{prefix}innovation_covariance_code_coupling_abs_m2": to_float(
            row.get("innovation_covariance_code_coupling_abs_m2", "0")
        ),
        f"{prefix}innovation_covariance_phase_coupling_abs_m2": to_float(
            row.get("innovation_covariance_phase_coupling_abs_m2", "0")
        ),
        f"{prefix}innovation_covariance_ionosphere_constraint_coupling_abs_m2": to_float(
            row.get("innovation_covariance_ionosphere_constraint_coupling_abs_m2", "0")
        ),
        f"{prefix}innovation_inverse_code_coupling_abs_1_per_m2": to_float(
            row.get("innovation_inverse_code_coupling_abs_1_per_m2", "0")
        ),
        f"{prefix}innovation_inverse_phase_coupling_abs_1_per_m2": to_float(
            row.get("innovation_inverse_phase_coupling_abs_1_per_m2", "0")
        ),
        f"{prefix}innovation_inverse_ionosphere_constraint_coupling_abs_1_per_m2": to_float(
            row.get("innovation_inverse_ionosphere_constraint_coupling_abs_1_per_m2", "0")
        ),
        f"{prefix}position_x_kalman_gain": to_float(row.get("position_x_kalman_gain", "0")),
        f"{prefix}position_y_kalman_gain": to_float(row.get("position_y_kalman_gain", "0")),
        f"{prefix}position_z_kalman_gain": to_float(row.get("position_z_kalman_gain", "0")),
        f"{prefix}position_update_contribution_x_m": to_float(
            row.get("position_update_contribution_x_m", "0")
        ),
        f"{prefix}position_update_contribution_y_m": to_float(
            row.get("position_update_contribution_y_m", "0")
        ),
        f"{prefix}position_update_contribution_z_m": to_float(
            row.get("position_update_contribution_z_m", "0")
        ),
        f"{prefix}position_update_contribution_3d_m": to_float(
            row.get("position_update_contribution_3d_m", "0")
        ),
        f"{prefix}receiver_clock_state_index": to_int(row.get("receiver_clock_state_index", "-1")),
        f"{prefix}receiver_clock_design_coeff": to_float(row.get("receiver_clock_design_coeff", "0")),
        f"{prefix}receiver_clock_kalman_gain": to_float(row.get("receiver_clock_kalman_gain", "0")),
        f"{prefix}receiver_clock_update_contribution_m": to_float(
            row.get("receiver_clock_update_contribution_m", "0")
        ),
        f"{prefix}ionosphere_state_index": to_int(row.get("ionosphere_state_index", "-1")),
        f"{prefix}ionosphere_design_coeff": to_float(row.get("ionosphere_design_coeff", "0")),
        f"{prefix}ionosphere_kalman_gain": to_float(row.get("ionosphere_kalman_gain", "0")),
        f"{prefix}ionosphere_update_contribution_m": to_float(
            row.get("ionosphere_update_contribution_m", "0")
        ),
        f"{prefix}ambiguity_state_index": to_int(row.get("ambiguity_state_index", "-1")),
        f"{prefix}ambiguity_design_coeff": to_float(row.get("ambiguity_design_coeff", "0")),
        f"{prefix}ambiguity_kalman_gain": to_float(row.get("ambiguity_kalman_gain", "0")),
        f"{prefix}ambiguity_update_contribution_m": to_float(
            row.get("ambiguity_update_contribution_m", "0")
        ),
    }


def correction_key(row: Row) -> CorrectionKey:
    return (
        int(float(row["week"])),
        tow_to_millis(row["tow"]),
        row.get("sat", ""),
        row.get("primary_signal", ""),
        row.get("secondary_signal", ""),
        row.get("primary_observation_code", ""),
        row.get("secondary_observation_code", ""),
    )


def index_correction_rows(rows: Sequence[Row]) -> Dict[CorrectionKey, Row]:
    indexed: Dict[CorrectionKey, Row] = {}
    for row in rows:
        indexed[correction_key(row)] = row
    return indexed


def filter_key(row: Row) -> FilterKey:
    return (
        int(float(row["week"])),
        tow_to_millis(row["tow"]),
        to_int(row.get("iteration", "0")),
    )


def format_filter_key(key: FilterKey) -> Dict[str, Any]:
    return {"week": key[0], "tow": key[1] / 1000.0, "iteration": key[2]}


def index_filter_rows(rows: Sequence[Row]) -> Dict[FilterKey, Row]:
    indexed: Dict[FilterKey, Row] = {}
    for row in rows:
        indexed[filter_key(row)] = row
    return indexed


def numeric_filter_columns(base_rows: Sequence[Row], candidate_rows: Sequence[Row]) -> List[str]:
    excluded = {"week", "tow", "iteration"}
    ordered: List[str] = []
    for rows in (base_rows, candidate_rows):
        if not rows:
            continue
        for column in rows[0].keys():
            if column not in excluded and column not in ordered:
                ordered.append(column)
    return ordered


def numeric_delta(base_row: Row, candidate_row: Row, column: str) -> Optional[Tuple[float, float, float]]:
    try:
        base_value = float(base_row.get(column, ""))
        candidate_value = float(candidate_row.get(column, ""))
    except ValueError:
        return None
    if not math.isfinite(base_value) or not math.isfinite(candidate_value):
        return None
    return candidate_value - base_value, base_value, candidate_value


def summarize_filter_comparison(
    base_rows: Sequence[Row],
    candidate_rows: Sequence[Row],
    *,
    tolerance: float = 1e-9,
    top_rows: int = 10,
    top_columns: int = 12,
) -> Dict[str, Any]:
    base_index = index_filter_rows(base_rows)
    candidate_index = index_filter_rows(candidate_rows)
    common_keys = sorted(set(base_index) & set(candidate_index))
    base_only_keys = sorted(set(base_index) - set(candidate_index))
    candidate_only_keys = sorted(set(candidate_index) - set(base_index))
    columns = numeric_filter_columns(base_rows, candidate_rows)

    changed_rows = 0
    first_changed_rows: List[Dict[str, Any]] = []
    max_by_column: Dict[str, Dict[str, Any]] = {}

    for key in common_keys:
        base_row = base_index[key]
        candidate_row = candidate_index[key]
        row_deltas: List[Dict[str, Any]] = []
        for column in columns:
            delta_tuple = numeric_delta(base_row, candidate_row, column)
            if delta_tuple is None:
                continue
            delta, base_value, candidate_value = delta_tuple
            abs_delta = abs(delta)
            if abs_delta <= tolerance:
                continue
            existing = max_by_column.get(column)
            if existing is None or abs_delta > existing["abs_delta"]:
                max_by_column[column] = {
                    "column": column,
                    **format_filter_key(key),
                    "base": base_value,
                    "candidate": candidate_value,
                    "delta": delta,
                    "abs_delta": abs_delta,
                }
            row_deltas.append(
                {
                    "column": column,
                    "base": base_value,
                    "candidate": candidate_value,
                    "delta": delta,
                    "abs_delta": abs_delta,
                }
            )
        if not row_deltas:
            continue
        changed_rows += 1
        if len(first_changed_rows) < top_rows:
            first_changed_rows.append(
                {
                    **format_filter_key(key),
                    "top_deltas": sorted(
                        row_deltas,
                        key=lambda item: (-item["abs_delta"], item["column"]),
                    )[:top_columns],
                }
            )

    return {
        "base_rows": len(base_rows),
        "candidate_rows": len(candidate_rows),
        "common_rows": len(common_keys),
        "base_only_rows": len(base_only_keys),
        "candidate_only_rows": len(candidate_only_keys),
        "changed_rows": changed_rows,
        "unchanged_rows": len(common_keys) - changed_rows,
        "tolerance": tolerance,
        "first_base_only_rows": [format_filter_key(key) for key in base_only_keys[:top_rows]],
        "first_candidate_only_rows": [format_filter_key(key) for key in candidate_only_keys[:top_rows]],
        "first_changed_rows": first_changed_rows,
        "max_column_deltas": sorted(
            max_by_column.values(),
            key=lambda item: (-item["abs_delta"], item["column"]),
        )[:top_columns],
    }


def correction_context_for_row(
    row: Row,
    correction_index: Optional[Dict[CorrectionKey, Row]],
) -> Optional[Dict[str, Any]]:
    if correction_index is None:
        return None
    correction = correction_index.get(correction_key(row))
    if correction is None:
        return {"matched": False}

    context: Dict[str, Any] = {"matched": True}
    for column in CORRECTION_FLOAT_COLUMNS:
        if column in correction:
            context[column] = to_float(correction.get(column, "0"))
    for column in CORRECTION_INT_COLUMNS:
        if column in correction:
            context[column] = to_int(correction.get(column, "0"))
    for column in CORRECTION_TEXT_COLUMNS:
        if column in correction:
            context[column] = correction.get(column, "")
    return context


def summarize_correction_join(
    rows: Sequence[Row],
    correction_index: Dict[CorrectionKey, Row],
) -> Dict[str, Any]:
    matched_rows = 0
    unmatched_rows = 0
    matched_by_type = Counter()
    unmatched_by_type = Counter()
    for row in rows:
        row_type = row.get("row_type", "")
        if correction_key(row) in correction_index:
            matched_rows += 1
            matched_by_type[row_type] += 1
        else:
            unmatched_rows += 1
            unmatched_by_type[row_type] += 1
    return {
        "matched_rows": matched_rows,
        "unmatched_rows": unmatched_rows,
        "matched_rows_by_type": dict(sorted(matched_by_type.items())),
        "unmatched_rows_by_type": dict(sorted(unmatched_by_type.items())),
    }


def summarize_filter_rows(rows: Sequence[Row], *, top_shapes: int = 10) -> Dict[str, Any]:
    row_shapes = Counter(
        (to_int(row["code_rows"]), to_int(row["phase_rows"]), to_int(row["ionosphere_constraint_rows"]))
        for row in rows
    )
    epochs = {epoch_key(row) for row in rows}
    iterations = Counter(to_int(row["iteration"]) for row in rows)

    def max_column(column: str) -> int:
        return max((to_int(row.get(column, "0")) for row in rows), default=0)

    def max_float_column(column: str) -> float:
        return max((abs(to_float(row.get(column, "0"))) for row in rows), default=0.0)

    return {
        "rows": len(rows),
        "epochs": len(epochs),
        "iterations": dict(sorted(iterations.items())),
        "max_rows": max_column("rows"),
        "max_code_rows": max_column("code_rows"),
        "max_phase_rows": max_column("phase_rows"),
        "max_ionosphere_constraint_rows": max_column("ionosphere_constraint_rows"),
        "row_shapes": [
            {
                "code_rows": shape[0],
                "phase_rows": shape[1],
                "ionosphere_constraint_rows": shape[2],
                "count": count,
            }
            for shape, count in sorted(
                row_shapes.items(),
                key=lambda item: (-item[1], -item[0][0], -item[0][1], -item[0][2]),
            )[:top_shapes]
        ],
        "max_code_residual_rms_m": max_float_column("code_residual_rms_m"),
        "max_code_residual_abs_m": max_float_column("code_residual_max_abs_m"),
        "max_phase_residual_rms_m": max_float_column("phase_residual_rms_m"),
        "max_phase_residual_abs_m": max_float_column("phase_residual_max_abs_m"),
    }


def select_final_iteration_rows(rows: Sequence[Row]) -> List[Row]:
    max_iteration_by_epoch: Dict[EpochKey, int] = {}
    for row in rows:
        key = epoch_key(row)
        max_iteration_by_epoch[key] = max(
            max_iteration_by_epoch.get(key, -1),
            to_int(row.get("iteration", "0")),
        )
    return [
        row
        for row in rows
        if to_int(row.get("iteration", "0")) == max_iteration_by_epoch[epoch_key(row)]
    ]


def residual_rankings(
    rows: Sequence[Row],
    *,
    top_satellites: int = 10,
    top_epochs: int = 10,
    correction_index: Optional[Dict[CorrectionKey, Row]] = None,
) -> Dict[str, Any]:
    by_satellite: Dict[Tuple[str, str], Dict[str, Any]] = {}
    by_epoch_type: Dict[Tuple[int, int, str], Dict[str, Any]] = {}

    def max_context(row: Row) -> Dict[str, Any]:
        context = residual_row_metadata(row, prefix="max_abs_")
        correction_context = correction_context_for_row(row, correction_index)
        if correction_context is not None:
            context["max_abs_correction"] = correction_context
        return context

    def epoch_context(row: Row) -> Dict[str, Any]:
        context = residual_row_metadata(row)
        correction_context = correction_context_for_row(row, correction_index)
        if correction_context is not None:
            context["correction"] = correction_context
        return context

    for row in rows:
        row_type = row.get("row_type", "")
        satellite = row.get("sat", "")
        residual_abs = abs(to_float(row.get("residual_m", "0")))
        iono_nonzero = abs(to_float(row.get("iono_state_m", "0"))) > 0.0
        week, tow_ms = epoch_key(row)
        iteration = to_int(row.get("iteration", "0"))
        row_index = to_int(row.get("row_index", "0"))

        sat_key = (row_type, satellite)
        sat_stats = by_satellite.setdefault(
            sat_key,
            {
                "row_type": row_type,
                "sat": satellite,
                "rows": 0,
                "sum_sq_residual_m2": 0.0,
                "max_abs_residual_m": 0.0,
                "max_abs_week": week,
                "max_abs_tow": tow_ms / 1000.0,
                "max_abs_iteration": iteration,
                "max_abs_row_index": row_index,
                "nonzero_iono_state_rows": 0,
                **max_context(row),
            },
        )
        sat_stats["rows"] += 1
        sat_stats["sum_sq_residual_m2"] += residual_abs * residual_abs
        if iono_nonzero:
            sat_stats["nonzero_iono_state_rows"] += 1
        if residual_abs > sat_stats["max_abs_residual_m"]:
            sat_stats["max_abs_residual_m"] = residual_abs
            sat_stats["max_abs_week"] = week
            sat_stats["max_abs_tow"] = tow_ms / 1000.0
            sat_stats["max_abs_iteration"] = iteration
            sat_stats["max_abs_row_index"] = row_index
            sat_stats.update(max_context(row))

        epoch_type_key = (week, tow_ms, row_type)
        epoch_stats = by_epoch_type.setdefault(
            epoch_type_key,
            {
                "week": week,
                "tow": tow_ms / 1000.0,
                "row_type": row_type,
                "rows": 0,
                "max_abs_residual_m": 0.0,
                "sat": satellite,
                "iteration": iteration,
                "row_index": row_index,
                **epoch_context(row),
            },
        )
        epoch_stats["rows"] += 1
        if residual_abs > epoch_stats["max_abs_residual_m"]:
            epoch_stats["max_abs_residual_m"] = residual_abs
            epoch_stats["sat"] = satellite
            epoch_stats["iteration"] = iteration
            epoch_stats["row_index"] = row_index
            epoch_stats.update(epoch_context(row))

    by_type_satellite: Dict[str, List[Dict[str, Any]]] = {}
    for stats in by_satellite.values():
        row_type = stats["row_type"]
        rows_count = max(int(stats["rows"]), 1)
        item = {
            "sat": stats["sat"],
            "rows": stats["rows"],
            "rms_residual_m": math.sqrt(stats["sum_sq_residual_m2"] / rows_count),
            "max_abs_residual_m": stats["max_abs_residual_m"],
            "max_abs_week": stats["max_abs_week"],
            "max_abs_tow": stats["max_abs_tow"],
            "max_abs_iteration": stats["max_abs_iteration"],
            "max_abs_row_index": stats["max_abs_row_index"],
            "max_abs_primary_signal": stats["max_abs_primary_signal"],
            "max_abs_secondary_signal": stats["max_abs_secondary_signal"],
            "max_abs_primary_observation_code": stats["max_abs_primary_observation_code"],
            "max_abs_secondary_observation_code": stats["max_abs_secondary_observation_code"],
            "max_abs_frequency_index": stats["max_abs_frequency_index"],
            "max_abs_ionosphere_coefficient": stats["max_abs_ionosphere_coefficient"],
            "max_abs_innovation_variance_m2": stats["max_abs_innovation_variance_m2"],
            "max_abs_innovation_inverse_diagonal_1_per_m2": stats[
                "max_abs_innovation_inverse_diagonal_1_per_m2"
            ],
            "max_abs_innovation_covariance_code_coupling_abs_m2": stats[
                "max_abs_innovation_covariance_code_coupling_abs_m2"
            ],
            "max_abs_innovation_covariance_phase_coupling_abs_m2": stats[
                "max_abs_innovation_covariance_phase_coupling_abs_m2"
            ],
            "max_abs_innovation_covariance_ionosphere_constraint_coupling_abs_m2": stats[
                "max_abs_innovation_covariance_ionosphere_constraint_coupling_abs_m2"
            ],
            "max_abs_innovation_inverse_code_coupling_abs_1_per_m2": stats[
                "max_abs_innovation_inverse_code_coupling_abs_1_per_m2"
            ],
            "max_abs_innovation_inverse_phase_coupling_abs_1_per_m2": stats[
                "max_abs_innovation_inverse_phase_coupling_abs_1_per_m2"
            ],
            "max_abs_innovation_inverse_ionosphere_constraint_coupling_abs_1_per_m2": stats[
                "max_abs_innovation_inverse_ionosphere_constraint_coupling_abs_1_per_m2"
            ],
            "max_abs_position_x_kalman_gain": stats["max_abs_position_x_kalman_gain"],
            "max_abs_position_y_kalman_gain": stats["max_abs_position_y_kalman_gain"],
            "max_abs_position_z_kalman_gain": stats["max_abs_position_z_kalman_gain"],
            "max_abs_position_update_contribution_x_m": stats[
                "max_abs_position_update_contribution_x_m"
            ],
            "max_abs_position_update_contribution_y_m": stats[
                "max_abs_position_update_contribution_y_m"
            ],
            "max_abs_position_update_contribution_z_m": stats[
                "max_abs_position_update_contribution_z_m"
            ],
            "max_abs_position_update_contribution_3d_m": stats[
                "max_abs_position_update_contribution_3d_m"
            ],
            "max_abs_receiver_clock_state_index": stats["max_abs_receiver_clock_state_index"],
            "max_abs_receiver_clock_design_coeff": stats["max_abs_receiver_clock_design_coeff"],
            "max_abs_receiver_clock_kalman_gain": stats["max_abs_receiver_clock_kalman_gain"],
            "max_abs_receiver_clock_update_contribution_m": stats[
                "max_abs_receiver_clock_update_contribution_m"
            ],
            "max_abs_ionosphere_state_index": stats["max_abs_ionosphere_state_index"],
            "max_abs_ionosphere_design_coeff": stats["max_abs_ionosphere_design_coeff"],
            "max_abs_ionosphere_kalman_gain": stats["max_abs_ionosphere_kalman_gain"],
            "max_abs_ionosphere_update_contribution_m": stats[
                "max_abs_ionosphere_update_contribution_m"
            ],
            "max_abs_ambiguity_state_index": stats["max_abs_ambiguity_state_index"],
            "max_abs_ambiguity_design_coeff": stats["max_abs_ambiguity_design_coeff"],
            "max_abs_ambiguity_kalman_gain": stats["max_abs_ambiguity_kalman_gain"],
            "max_abs_ambiguity_update_contribution_m": stats[
                "max_abs_ambiguity_update_contribution_m"
            ],
            "nonzero_iono_state_rows": stats["nonzero_iono_state_rows"],
        }
        if "max_abs_correction" in stats:
            item["max_abs_correction"] = stats["max_abs_correction"]
        by_type_satellite.setdefault(row_type, []).append(item)

    by_type_epoch: Dict[str, List[Dict[str, Any]]] = {}
    for stats in by_epoch_type.values():
        by_type_epoch.setdefault(stats["row_type"], []).append(stats)

    return {
        "top_residual_satellites_by_type": {
            row_type: sorted(
                items,
                key=lambda item: (-item["max_abs_residual_m"], -item["rms_residual_m"], item["sat"]),
            )[:top_satellites]
            for row_type, items in sorted(by_type_satellite.items())
        },
        "top_residual_epochs_by_type": {
            row_type: sorted(
                items,
                key=lambda item: (-item["max_abs_residual_m"], item["week"], item["tow"]),
            )[:top_epochs]
            for row_type, items in sorted(by_type_epoch.items())
        },
    }


def summarize_residual_subset(
    rows: Sequence[Row],
    *,
    top_satellites: int = 10,
    top_epochs: int = 10,
    correction_index: Optional[Dict[CorrectionKey, Row]] = None,
) -> Dict[str, Any]:
    row_type_counts = Counter(row.get("row_type", "") for row in rows)
    by_type_satellite: Dict[str, Counter[str]] = {}
    nonzero_iono_by_type = Counter()
    max_abs_residual_by_type: Dict[str, float] = {}

    for row in rows:
        row_type = row.get("row_type", "")
        by_type_satellite.setdefault(row_type, Counter())[row.get("sat", "")] += 1
        if abs(to_float(row.get("iono_state_m", "0"))) > 0.0:
            nonzero_iono_by_type[row_type] += 1
        max_abs_residual_by_type[row_type] = max(
            max_abs_residual_by_type.get(row_type, 0.0),
            abs(to_float(row.get("residual_m", "0"))),
        )

    summary: Dict[str, Any] = {
        "rows": len(rows),
        "epochs": len({epoch_key(row) for row in rows}),
        "row_type_counts": dict(sorted(row_type_counts.items())),
        "nonzero_iono_state_rows_by_type": dict(sorted(nonzero_iono_by_type.items())),
        "max_abs_residual_m_by_type": dict(sorted(max_abs_residual_by_type.items())),
        "top_satellites_by_type": {
            row_type: [
                {"sat": sat, "rows": count}
                for sat, count in counter.most_common(top_satellites)
            ]
            for row_type, counter in sorted(by_type_satellite.items())
        },
    }
    summary.update(
        residual_rankings(
            rows,
            top_satellites=top_satellites,
            top_epochs=top_epochs,
            correction_index=correction_index,
        )
    )
    if correction_index is not None:
        summary["correction_join"] = summarize_correction_join(rows, correction_index)
    return summary


def summarize_residual_rows(
    rows: Sequence[Row],
    *,
    top_satellites: int = 10,
    top_epochs: int = 10,
    correction_index: Optional[Dict[CorrectionKey, Row]] = None,
) -> Dict[str, Any]:
    summary = summarize_residual_subset(
        rows,
        top_satellites=top_satellites,
        top_epochs=top_epochs,
        correction_index=correction_index,
    )
    final_rows = select_final_iteration_rows(rows)
    summary["final_iteration"] = summarize_residual_subset(
        final_rows,
        top_satellites=top_satellites,
        top_epochs=top_epochs,
        correction_index=correction_index,
    )
    return summary


def build_report(args: argparse.Namespace) -> Dict[str, Any]:
    if args.correction_log is not None and args.residual_log is None:
        raise ValueError("--correction-log requires --residual-log")

    filter_rows = read_csv(args.filter_log, FILTER_REQUIRED_COLUMNS)
    report: Dict[str, Any] = {
        "filter_log": str(args.filter_log),
        "filter_summary": summarize_filter_rows(filter_rows, top_shapes=args.top_shapes),
    }
    if args.compare_filter_log is not None:
        candidate_filter_rows = read_csv(args.compare_filter_log, FILTER_REQUIRED_COLUMNS)
        report["candidate_filter_log"] = str(args.compare_filter_log)
        report["filter_comparison"] = summarize_filter_comparison(
            filter_rows,
            candidate_filter_rows,
            tolerance=args.compare_tolerance,
            top_rows=args.top_epochs,
            top_columns=args.top_compare_columns,
        )
    correction_index: Optional[Dict[CorrectionKey, Row]] = None
    if args.correction_log is not None:
        correction_rows = read_csv(args.correction_log, CORRECTION_REQUIRED_COLUMNS)
        correction_index = index_correction_rows(correction_rows)
        report["correction_log"] = str(args.correction_log)
        report["correction_log_rows"] = len(correction_rows)
    if args.residual_log is not None:
        residual_rows = read_csv(args.residual_log, RESIDUAL_REQUIRED_COLUMNS)
        report["residual_log"] = str(args.residual_log)
        report["residual_summary"] = summarize_residual_rows(
            residual_rows,
            top_satellites=args.top_satellites,
            top_epochs=args.top_epochs,
            correction_index=correction_index,
        )
    return report


def print_filter_summary(summary: Dict[str, Any]) -> None:
    print("filter:")
    for key in [
        "rows",
        "epochs",
        "max_rows",
        "max_code_rows",
        "max_phase_rows",
        "max_ionosphere_constraint_rows",
        "max_code_residual_rms_m",
        "max_code_residual_abs_m",
        "max_phase_residual_rms_m",
        "max_phase_residual_abs_m",
    ]:
        print(f"  {key}: {summary[key]}")
    print("  row_shapes:")
    for shape in summary["row_shapes"]:
        print(
            "    "
            f"code={shape['code_rows']} phase={shape['phase_rows']} "
            f"iono={shape['ionosphere_constraint_rows']} count={shape['count']}"
        )


def residual_signal_suffix(item: Dict[str, Any], *, prefix: str = "") -> str:
    primary_code = item.get(f"{prefix}primary_observation_code", "")
    secondary_code = item.get(f"{prefix}secondary_observation_code", "")
    frequency_index = item.get(f"{prefix}frequency_index", 0)
    if primary_code or secondary_code:
        return f" {primary_code or '-'}{('/' + secondary_code) if secondary_code else ''} f={frequency_index}"
    if frequency_index:
        return f" f={frequency_index}"
    return ""


def correction_suffix(context: Optional[Dict[str, Any]]) -> str:
    if context is None:
        return ""
    if not context.get("matched", False):
        return " corr[unmatched]"
    return (
        " corr["
        f"valid={context.get('valid_after_corrections', 0)} "
        f"ssr={context.get('ssr_available', 0)} "
        f"orbclk={context.get('orbit_clock_applied', 0)} "
        f"clk={context.get('clock_m', 0.0):.3f} "
        f"cb={context.get('code_bias_m', 0.0):.3f} "
        f"pb={context.get('phase_bias_m', 0.0):.3f} "
        f"var_pr={context.get('variance_pr', 0.0):.3f} "
        f"var_cp={context.get('variance_cp', 0.0):.6f}"
        "]"
    )


def print_correction_join(label: str, summary: Dict[str, Any]) -> None:
    join = summary.get("correction_join")
    if join is None:
        return
    print(
        f"  {label}_correction_join: matched={join['matched_rows']} "
        f"unmatched={join['unmatched_rows']}"
    )


def print_residual_rankings(label: str, summary: Dict[str, Any], *, limit: int = 3) -> None:
    print(f"  {label}_top_residual_satellites:")
    for row_type, items in summary.get("top_residual_satellites_by_type", {}).items():
        formatted = ", ".join(
            f"{item['sat']}{residual_signal_suffix(item, prefix='max_abs_')} "
            f"max={item['max_abs_residual_m']:.3f} "
            f"rms={item['rms_residual_m']:.3f}"
            f"{correction_suffix(item.get('max_abs_correction'))}"
            for item in items[:limit]
        )
        print(f"    {row_type}: {formatted}")
    print(f"  {label}_top_residual_epochs:")
    for row_type, items in summary.get("top_residual_epochs_by_type", {}).items():
        formatted = ", ".join(
            f"{item['week']}:{item['tow']:.3f} {item['sat']}"
            f"{residual_signal_suffix(item)} max={item['max_abs_residual_m']:.3f}"
            f"{correction_suffix(item.get('correction'))}"
            for item in items[:limit]
        )
        print(f"    {row_type}: {formatted}")


def print_filter_comparison(summary: Dict[str, Any]) -> None:
    print("filter_comparison:")
    for key in [
        "base_rows",
        "candidate_rows",
        "common_rows",
        "base_only_rows",
        "candidate_only_rows",
        "changed_rows",
        "unchanged_rows",
    ]:
        print(f"  {key}: {summary[key]}")
    if summary["first_changed_rows"]:
        print("  first_changed_rows:")
        for row in summary["first_changed_rows"][:3]:
            deltas = ", ".join(
                f"{item['column']}={item['delta']:.6g}"
                for item in row.get("top_deltas", [])[:5]
            )
            print(
                "    "
                f"{row['week']}:{row['tow']:.3f} iter={row['iteration']} {deltas}"
            )
    if summary["max_column_deltas"]:
        print("  max_column_deltas:")
        for item in summary["max_column_deltas"][:8]:
            print(
                "    "
                f"{item['column']} {item['week']}:{item['tow']:.3f} "
                f"iter={item['iteration']} base={item['base']:.6g} "
                f"candidate={item['candidate']:.6g} delta={item['delta']:.6g}"
            )


def print_residual_summary(summary: Dict[str, Any]) -> None:
    print("residual:")
    print(f"  rows: {summary['rows']}")
    print(f"  epochs: {summary['epochs']}")
    print(f"  row_type_counts: {summary['row_type_counts']}")
    print(f"  nonzero_iono_state_rows_by_type: {summary['nonzero_iono_state_rows_by_type']}")
    print(f"  max_abs_residual_m_by_type: {summary['max_abs_residual_m_by_type']}")
    print_correction_join("all", summary)
    print_residual_rankings("all", summary)
    final_summary = summary.get("final_iteration")
    if final_summary is not None:
        print("  final_iteration:")
        print(f"    rows: {final_summary['rows']}")
        print(f"    row_type_counts: {final_summary['row_type_counts']}")
        print(f"    max_abs_residual_m_by_type: {final_summary['max_abs_residual_m_by_type']}")
        print_correction_join("final", final_summary)
        print_residual_rankings("final", final_summary)


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Summarize gnss_ppp --ppp-filter-log and --ppp-residual-log CSV files."
    )
    parser.add_argument("filter_log", type=Path)
    parser.add_argument("--residual-log", type=Path)
    parser.add_argument("--correction-log", type=Path)
    parser.add_argument("--json-out", type=Path, help="write full report JSON")
    parser.add_argument("--compare-filter-log", type=Path, help="candidate filter CSV to compare against the positional base log")
    parser.add_argument("--compare-tolerance", type=float, default=1e-9)
    parser.add_argument("--top-shapes", type=int, default=10)
    parser.add_argument("--top-satellites", type=int, default=10)
    parser.add_argument("--top-epochs", type=int, default=10)
    parser.add_argument("--top-compare-columns", type=int, default=12)
    return parser.parse_args(argv)


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)
    report = build_report(args)
    print_filter_summary(report["filter_summary"])
    if "filter_comparison" in report:
        print_filter_comparison(report["filter_comparison"])
    if "residual_summary" in report:
        print_residual_summary(report["residual_summary"])
    if args.json_out is not None:
        args.json_out.parent.mkdir(parents=True, exist_ok=True)
        args.json_out.write_text(
            json.dumps(report, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except BrokenPipeError:
        raise SystemExit(1)
