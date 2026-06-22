#!/usr/bin/env python3
"""Compare CLAS zero-difference correction component CSV dumps."""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from collections import Counter, defaultdict
from dataclasses import dataclass
from decimal import Decimal, InvalidOperation, ROUND_HALF_UP
from pathlib import Path
from typing import Any, DefaultDict, Iterable, Optional, Sequence


SCHEMA = "clas_zd_component_diff.v1"

KEY_REQUIRED_COLUMNS = ["week", "tow", "sat"]

COMPONENT_ALIASES: dict[str, tuple[str, ...]] = {
    "applied_pr_corr_m": ("applied_pr_corr_m", "pseudorange_correction_m"),
    "carrier_correction_m": ("carrier_correction_m", "carrier_phase_correction_m"),
    "prc_m": ("prc_m", "PRC", "PRC_m"),
    "prc_component_sum_m": ("prc_component_sum_m",),
    "prc_closure_residual_m": ("prc_closure_residual_m",),
    "cpc_m": ("cpc_m", "CPC", "CPC_m"),
    "prc_minus_trop_m": ("prc_minus_trop_m",),
    "cpc_minus_trop_m": ("cpc_minus_trop_m",),
    "trop_correction_m": ("trop_correction_m", "trop_m"),
    "iono_l1_m": ("iono_l1_m", "l1_iono_m"),
    "stec_tecu": ("stec_tecu",),
    "iono_l1_from_stec_m": ("iono_l1_from_stec_m",),
    "iono_l1_stec_closure_residual_m": ("iono_l1_stec_closure_residual_m",),
    "iono_scaled_m": ("iono_scaled_m",),
    "iono_scale": ("iono_scale",),
    "iono_scaled_closure_residual_m": ("iono_scaled_closure_residual_m",),
    "claslib_raw_iono_l1_m": ("claslib_raw_iono_l1_m", "raw_iono_l1_m"),
    "claslib_raw_stec_tecu": ("claslib_raw_stec_tecu", "raw_stec_tecu"),
    "iono_cpc_m": ("iono_cpc_m",),
    "code_bias_m": ("code_bias_m", "code_bias"),
    "phase_bias_m": ("phase_bias_m", "phase_bias"),
    "bias_exact_identity": ("bias_exact_identity",),
    "observation_exact_identity_requested": ("observation_exact_identity_requested",),
    "observation_exact_match": ("observation_exact_match",),
    "observation_family_fallback": ("observation_family_fallback",),
    "code_bias_source_signal_id": ("code_bias_source_signal_id",),
    "phase_bias_source_signal_id": ("phase_bias_source_signal_id",),
    "code_bias_present": ("code_bias_present",),
    "phase_bias_present": ("phase_bias_present",),
    "code_bias_fallback": ("code_bias_fallback",),
    "phase_bias_fallback": ("phase_bias_fallback",),
    "receiver_antenna_m": ("receiver_antenna_m", "receiver_ant_m", "receiver_ant"),
    "relativity_m": ("relativity_m", "relativity_correction_m"),
    "atmos_ref_week": ("atmos_ref_week", "atmos_reference_week"),
    "atmos_ref_tow": ("atmos_ref_tow", "atmos_reference_tow"),
    "clock_ref_week": ("clock_ref_week", "clock_reference_week"),
    "clock_ref_tow": ("clock_ref_tow", "clock_reference_tow"),
    "code_bias_ref_week": ("code_bias_ref_week", "code_bias_reference_week"),
    "code_bias_ref_tow": ("code_bias_ref_tow", "code_bias_reference_tow"),
    "atmos_clock_gap_s": ("atmos_clock_gap_s", "atmos_clock_dt_s"),
    "atmos_network_id": ("atmos_network_id",),
    "atmos_grid_no": ("atmos_grid_no",),
    "atmos_grid_distance_m": ("atmos_grid_distance_m",),
    "atmos_grid_count": ("atmos_grid_count",),
    "atmos_grid1_no": ("atmos_grid1_no",),
    "atmos_grid1_weight": ("atmos_grid1_weight",),
    "atmos_grid2_no": ("atmos_grid2_no",),
    "atmos_grid2_weight": ("atmos_grid2_weight",),
    "atmos_grid3_no": ("atmos_grid3_no",),
    "atmos_grid3_weight": ("atmos_grid3_weight",),
    "atmos_grid4_no": ("atmos_grid4_no",),
    "atmos_grid4_weight": ("atmos_grid4_weight",),
    "atmos_lifecycle": ("atmos_lifecycle",),
    "atmos_lifecycle_tow": ("atmos_lifecycle_tow",),
    "atmos_selected_satellite_count": ("atmos_selected_satellite_count",),
    "atmos_valid_grid_count": ("atmos_valid_grid_count",),
    "atmos_stec_grid_value_count": ("atmos_stec_grid_value_count",),
    "atmos_selected_grid_stec_value_count": ("atmos_selected_grid_stec_value_count",),
    "windup_m": ("windup_m",),
    "phase_compensation_m": ("phase_compensation_m",),
    "orbit_projection_m": ("orbit_projection_m",),
    "clock_correction_m": ("clock_correction_m",),
    "residual_m": ("residual_m",),
    "variance_m2": ("variance_m2",),
}


Row = dict[str, str]
ComponentMap = dict[str, float]
Key = tuple[int, int, str, str, int, str]
DuplicatePolicy = str
GPS_L2W_RINEX_CODES = {"C2W", "L2W"}
IDENTITY_PROVENANCE_COMPONENTS = (
    "bias_exact_identity",
    "observation_exact_identity_requested",
    "observation_exact_match",
    "observation_family_fallback",
    "code_bias_present",
    "phase_bias_present",
    "code_bias_fallback",
    "phase_bias_fallback",
)
PRC_COMPONENT_SUM_INPUTS = (
    "trop_correction_m",
    "relativity_m",
    "receiver_antenna_m",
    "iono_scaled_m",
    "code_bias_m",
)
GPS_IONO_SCALE_BY_BAND = {
    "1": 1.0,
    "2": (1575.42 / 1227.60) ** 2,
    "5": (1575.42 / 1176.45) ** 2,
}
GPS_L1_TECU_TO_METERS = 40.3e16 / ((1575.42 * 1e6) ** 2)


@dataclass(frozen=True)
class ComponentRow:
    key: Key
    stage: str
    row_type: str
    week: int
    tow: float
    sat: str
    freq: int
    signal: str
    source_row: int
    components: ComponentMap


def tow_to_millis(value: str) -> int:
    try:
        tow = Decimal(str(value))
    except InvalidOperation as exc:
        raise ValueError(f"invalid tow value: {value}") from exc
    return int((tow * Decimal("1000")).to_integral_value(rounding=ROUND_HALF_UP))


def to_int(value: str, default: int = 0) -> int:
    if value == "":
        return default
    try:
        return int(float(value))
    except ValueError:
        return default


def to_float(value: str) -> Optional[float]:
    if value == "":
        return None
    try:
        parsed = float(value)
    except ValueError:
        return None
    return parsed if math.isfinite(parsed) else None


def first_present(row: Row, names: Iterable[str], default: str = "") -> str:
    for name in names:
        if name in row and row[name] != "":
            return row[name]
    return default


def row_component_value(row: Row, component: str) -> Optional[float]:
    aliases = COMPONENT_ALIASES.get(component, (component,))
    return to_float(first_present(row, aliases))


def row_iono_scale(row: Row) -> Optional[float]:
    parsed = row_component_value(row, "iono_scale")
    if parsed is not None:
        return parsed
    row_type = detect_row_type(row)
    signal = observation_identity(row, row_type)
    sat = first_present(row, ("sat",))
    if not sat.startswith("G") or len(signal) < 2:
        return None
    return GPS_IONO_SCALE_BY_BAND.get(signal[1])


def derived_component_value(row: Row, component: str) -> Optional[float]:
    if component == "iono_scale":
        return row_iono_scale(row)

    if component == "iono_l1_from_stec_m":
        stec_tecu = row_component_value(row, "stec_tecu")
        if stec_tecu is None:
            return None
        return stec_tecu * GPS_L1_TECU_TO_METERS

    if component == "iono_l1_stec_closure_residual_m":
        iono_l1 = row_component_value(row, "iono_l1_m")
        stec_tecu = row_component_value(row, "stec_tecu")
        if iono_l1 is None or stec_tecu is None:
            return None
        return iono_l1 - stec_tecu * GPS_L1_TECU_TO_METERS

    if component == "iono_scaled_closure_residual_m":
        iono_scaled = row_component_value(row, "iono_scaled_m")
        iono_l1 = row_component_value(row, "iono_l1_m")
        iono_scale = row_iono_scale(row)
        if iono_scaled is None or iono_l1 is None or iono_scale is None:
            return None
        return iono_scaled - iono_scale * iono_l1

    if component not in {"prc_component_sum_m", "prc_closure_residual_m"}:
        return None

    inputs = [row_component_value(row, name) for name in PRC_COMPONENT_SUM_INPUTS]
    if any(value is None for value in inputs):
        return None
    component_sum = sum(float(value) for value in inputs if value is not None)
    if component == "prc_component_sum_m":
        return component_sum

    prc = row_component_value(row, "prc_m")
    if prc is None:
        return None
    return prc - component_sum


def detect_row_type(row: Row) -> str:
    value = first_present(row, ("row_type", "kind", "type", "record"), "")
    normalized = value.strip().lower()
    if normalized in {"code", "pseudorange", "pr"}:
        return "code"
    if normalized in {"phase", "carrier", "carrier_phase", "amb", "ambiguity"}:
        return "phase"
    if normalized.startswith("code"):
        return "code"
    if normalized.startswith("phase") or normalized.startswith("carrier"):
        return "phase"
    return normalized


def detect_frequency(row: Row) -> int:
    return to_int(first_present(row, ("freq", "frequency_index", "frequency", "f"), "0"))


def observation_identity(row: Row, row_type: str) -> str:
    if row_type == "code":
        return first_present(
            row,
            (
                "pseudorange_rinex_code",
                "pseudorange_observation_type",
                "obs_code",
                "rinex_code",
                "code",
                "signal",
            ),
        )
    if row_type == "phase":
        return first_present(
            row,
            (
                "carrier_rinex_code",
                "carrier_phase_observation_type",
                "obs_code",
                "rinex_code",
                "code",
                "signal",
            ),
        )
    return first_present(row, ("obs_code", "rinex_code", "code", "signal"), "")


def extract_components(row: Row, component_names: Sequence[str]) -> ComponentMap:
    components: ComponentMap = {}
    for component in component_names:
        parsed = row_component_value(row, component)
        if parsed is None:
            parsed = derived_component_value(row, component)
        if parsed is not None:
            components[component] = parsed
    return components


def read_csv_rows(path: Path) -> list[Row]:
    with path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        if reader.fieldnames is None:
            raise ValueError(f"{path}: missing CSV header")
        missing = [column for column in KEY_REQUIRED_COLUMNS if column not in reader.fieldnames]
        if missing:
            raise ValueError(f"{path}: missing required columns: {', '.join(missing)}")
        return [dict(row) for row in reader]


def normalize_rows(
    rows: Sequence[Row],
    *,
    component_names: Sequence[str],
    stage_filter: Optional[str],
    row_type_filter: Optional[str],
    sat_filter: Optional[set[str]] = None,
    freq_filter: Optional[set[int]] = None,
    rinex_code_filter: Optional[set[str]] = None,
) -> list[ComponentRow]:
    normalized: list[ComponentRow] = []
    for index, row in enumerate(rows, start=2):
        stage = first_present(row, ("stage",), "")
        if stage_filter is not None and stage != stage_filter:
            continue
        row_type = detect_row_type(row)
        if row_type_filter is not None and row_type != row_type_filter:
            continue
        week = to_int(row["week"])
        tow_millis = tow_to_millis(row["tow"])
        sat = row["sat"]
        freq = detect_frequency(row)
        signal = observation_identity(row, row_type)
        if sat_filter is not None and sat not in sat_filter:
            continue
        if freq_filter is not None and freq not in freq_filter:
            continue
        if rinex_code_filter is not None and signal not in rinex_code_filter:
            continue
        components = extract_components(row, component_names)
        if not components:
            continue
        key = (week, tow_millis, row_type, sat, freq, signal)
        normalized.append(
            ComponentRow(
                key=key,
                stage=stage,
                row_type=row_type,
                week=week,
                tow=tow_millis / 1000.0,
                sat=sat,
                freq=freq,
                signal=signal,
                source_row=index,
                components=components,
            )
        )
    return normalized


def normalize_filter_values(values: Optional[Sequence[str]]) -> Optional[set[str]]:
    if not values:
        return None
    normalized: set[str] = set()
    for value in values:
        for item in value.replace(";", ",").split(","):
            text = item.strip()
            if text:
                normalized.add(text)
    return normalized or None


def normalize_freq_filter(values: Optional[Sequence[str]]) -> Optional[set[int]]:
    text_values = normalize_filter_values(values)
    if text_values is None:
        return None
    return {to_int(value) for value in text_values}


def merge_duplicate_rows(rows: Sequence[ComponentRow]) -> ComponentRow:
    if not rows:
        raise ValueError("cannot merge an empty duplicate group")
    template = rows[0]
    component_values: DefaultDict[str, list[float]] = defaultdict(list)
    for row in rows:
        for component, value in row.components.items():
            component_values[component].append(value)
    return ComponentRow(
        key=template.key,
        stage=template.stage,
        row_type=template.row_type,
        week=template.week,
        tow=template.tow,
        sat=template.sat,
        freq=template.freq,
        signal=template.signal,
        source_row=template.source_row,
        components={
            component: sum(values) / len(values)
            for component, values in component_values.items()
        },
    )


def rows_by_key(
    rows: Sequence[ComponentRow],
    *,
    duplicate_policy: DuplicatePolicy = "last",
) -> tuple[dict[Key, ComponentRow], Counter[Key]]:
    if duplicate_policy not in {"first", "last", "mean", "fail"}:
        raise ValueError(f"unsupported duplicate policy: {duplicate_policy}")
    grouped: dict[Key, ComponentRow] = {}
    duplicate_counts: Counter[Key] = Counter()
    if duplicate_policy == "mean":
        duplicates: DefaultDict[Key, list[ComponentRow]] = defaultdict(list)
        for row in rows:
            duplicates[row.key].append(row)
        for key, group in duplicates.items():
            if len(group) > 1:
                duplicate_counts[key] = len(group) - 1
            grouped[key] = merge_duplicate_rows(group)
        return grouped, duplicate_counts

    for row in rows:
        if row.key in grouped:
            duplicate_counts[row.key] += 1
            if duplicate_policy == "first":
                continue
        grouped[row.key] = row
    if duplicate_policy == "fail" and duplicate_counts:
        first_key, count = duplicate_counts.most_common(1)[0]
        raise ValueError(
            "duplicate row keys are present "
            f"(first duplicate {key_to_dict(first_key)}, extra_rows={count}); "
            "choose --duplicate-policy first, last, or mean to continue"
        )
    return grouped, duplicate_counts


def key_to_dict(key: Key) -> dict[str, Any]:
    week, tow_millis, row_type, sat, freq, signal = key
    return {
        "week": week,
        "tow": tow_millis / 1000.0,
        "row_type": row_type,
        "sat": sat,
        "freq": freq,
        "rinex_code": signal,
    }


def component_stats(values: Sequence[float]) -> dict[str, Any]:
    if not values:
        return {
            "rows": 0,
            "mean_abs_delta_m": 0.0,
            "rms_delta_m": 0.0,
            "max_abs_delta_m": 0.0,
        }
    abs_values = [abs(value) for value in values]
    return {
        "rows": len(values),
        "mean_abs_delta_m": sum(abs_values) / len(abs_values),
        "rms_delta_m": math.sqrt(sum(value * value for value in values) / len(values)),
        "max_abs_delta_m": max(abs_values),
    }


def is_component_true(row: ComponentRow, component: str) -> bool:
    value = row.components.get(component)
    return value is not None and value > 0.5


def is_gps_l2w_row(row: ComponentRow) -> bool:
    return row.sat.startswith("G") and row.signal in GPS_L2W_RINEX_CODES


def identity_provenance_summary(rows: Sequence[ComponentRow]) -> dict[str, Any]:
    gps_l2w_rows = [row for row in rows if is_gps_l2w_row(row)]
    summary: dict[str, Any] = {
        "rows": len(rows),
        "gps_l2w_rows": len(gps_l2w_rows),
    }
    for component in IDENTITY_PROVENANCE_COMPONENTS:
        summary[f"{component}_rows"] = sum(
            1 for row in rows if is_component_true(row, component)
        )
        summary[f"gps_l2w_{component}_rows"] = sum(
            1 for row in gps_l2w_rows if is_component_true(row, component)
        )
    return summary


def row_summary(row: ComponentRow) -> dict[str, Any]:
    return {
        **key_to_dict(row.key),
        "stage": row.stage,
        "signal": row.signal,
        "source_row": row.source_row,
        "components": sorted(row.components),
    }


def row_component_breakdown(
    key: Key,
    base_row: ComponentRow,
    candidate_row: ComponentRow,
) -> dict[str, Any]:
    components: list[dict[str, Any]] = []
    missing_components: list[dict[str, Any]] = []
    for component in sorted(set(base_row.components) | set(candidate_row.components)):
        base_value = base_row.components.get(component)
        candidate_value = candidate_row.components.get(component)
        if base_value is None or candidate_value is None:
            missing_components.append(
                {
                    "component": component,
                    "base_present": base_value is not None,
                    "candidate_present": candidate_value is not None,
                    "base_value": base_value,
                    "candidate_value": candidate_value,
                }
            )
            continue
        delta = candidate_value - base_value
        components.append(
            {
                "component": component,
                "base_value_m": base_value,
                "candidate_value_m": candidate_value,
                "delta_m": delta,
                "abs_delta_m": abs(delta),
            }
        )
    components.sort(key=lambda item: (-item["abs_delta_m"], item["component"]))
    return {
        **key_to_dict(key),
        "base_stage": base_row.stage,
        "candidate_stage": candidate_row.stage,
        "base_signal": base_row.signal,
        "candidate_signal": candidate_row.signal,
        "base_source_row": base_row.source_row,
        "candidate_source_row": candidate_row.source_row,
        "component_count": len(components),
        "missing_component_count": len(missing_components),
        "max_abs_delta_m": max(
            (item["abs_delta_m"] for item in components),
            default=0.0,
        ),
        "sum_abs_delta_m": sum(item["abs_delta_m"] for item in components),
        "components": components,
        "missing_components": missing_components,
    }


def build_report(
    base_rows: Sequence[ComponentRow],
    candidate_rows: Sequence[ComponentRow],
    *,
    base_label: str,
    candidate_label: str,
    threshold_m: Optional[float],
    top_deltas: int,
    top_unmatched: int,
    duplicate_policy: DuplicatePolicy = "last",
) -> dict[str, Any]:
    base_by_key, base_duplicates = rows_by_key(
        base_rows,
        duplicate_policy=duplicate_policy,
    )
    candidate_by_key, candidate_duplicates = rows_by_key(
        candidate_rows,
        duplicate_policy=duplicate_policy,
    )
    base_keys = set(base_by_key)
    candidate_keys = set(candidate_by_key)
    common_keys = sorted(base_keys & candidate_keys)
    base_only_keys = sorted(base_keys - candidate_keys)
    candidate_only_keys = sorted(candidate_keys - base_keys)

    component_deltas: DefaultDict[str, list[float]] = defaultdict(list)
    details: list[dict[str, Any]] = []
    row_breakdowns: list[dict[str, Any]] = []
    missing_components: Counter[str] = Counter()
    threshold_exceedances = 0

    for key in common_keys:
        base_row = base_by_key[key]
        candidate_row = candidate_by_key[key]
        row_breakdown = row_component_breakdown(key, base_row, candidate_row)
        if row_breakdown["component_count"] > 0:
            row_breakdowns.append(row_breakdown)
        components = sorted(set(base_row.components) | set(candidate_row.components))
        for component in components:
            base_value = base_row.components.get(component)
            candidate_value = candidate_row.components.get(component)
            if base_value is None or candidate_value is None:
                missing_components[component] += 1
                continue
            delta = candidate_value - base_value
            component_deltas[component].append(delta)
            abs_delta = abs(delta)
            if threshold_m is not None and abs_delta > threshold_m:
                threshold_exceedances += 1
            details.append(
                {
                    **key_to_dict(key),
                    "component": component,
                    "base_value_m": base_value,
                    "candidate_value_m": candidate_value,
                    "delta_m": delta,
                    "abs_delta_m": abs_delta,
                    "base_signal": base_row.signal,
                    "candidate_signal": candidate_row.signal,
                    "base_source_row": base_row.source_row,
                    "candidate_source_row": candidate_row.source_row,
                }
            )

    details.sort(
        key=lambda item: (
            -item["abs_delta_m"],
            item["week"],
            item["tow"],
            item["row_type"],
            item["sat"],
            item["freq"],
            item["rinex_code"],
            item["component"],
        )
    )
    row_breakdowns.sort(
        key=lambda item: (
            -item["sum_abs_delta_m"],
            -item["max_abs_delta_m"],
            item["week"],
            item["tow"],
            item["row_type"],
            item["sat"],
            item["freq"],
            item["rinex_code"],
        )
    )
    per_component = [
        {"component": component, **component_stats(values)}
        for component, values in sorted(component_deltas.items())
    ]
    per_component.sort(key=lambda item: (-item["max_abs_delta_m"], item["component"]))

    return {
        "schema": SCHEMA,
        "base_label": base_label,
        "candidate_label": candidate_label,
        "base_rows": len(base_rows),
        "candidate_rows": len(candidate_rows),
        "common_rows": len(common_keys),
        "base_only_rows": len(base_only_keys),
        "candidate_only_rows": len(candidate_only_keys),
        "base_duplicate_keys": sum(base_duplicates.values()),
        "candidate_duplicate_keys": sum(candidate_duplicates.values()),
        "duplicate_policy": duplicate_policy,
        "components_compared": sum(item["rows"] for item in per_component),
        "component_threshold_m": threshold_m,
        "threshold_exceedances": threshold_exceedances,
        "missing_component_counts": [
            {"component": component, "rows": rows}
            for component, rows in missing_components.most_common()
        ],
        "identity_provenance": {
            base_label: identity_provenance_summary(base_rows),
            candidate_label: identity_provenance_summary(candidate_rows),
        },
        "per_component": per_component,
        "top_component_deltas": details[:top_deltas],
        "top_row_component_breakdowns": row_breakdowns[:top_deltas],
        "top_base_only": [row_summary(base_by_key[key]) for key in base_only_keys[:top_unmatched]],
        "top_candidate_only": [
            row_summary(candidate_by_key[key]) for key in candidate_only_keys[:top_unmatched]
        ],
    }


def write_details_csv(path: Path, report: dict[str, Any]) -> None:
    fieldnames = [
        "week",
        "tow",
        "row_type",
        "sat",
        "freq",
        "rinex_code",
        "component",
        "base_value_m",
        "candidate_value_m",
        "delta_m",
        "abs_delta_m",
        "base_signal",
        "candidate_signal",
        "base_source_row",
        "candidate_source_row",
    ]
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(report.get("top_component_deltas", []))


def print_summary(report: dict[str, Any]) -> None:
    print("clas_zd_component_diff:")
    for key in [
        "schema",
        "base_label",
        "candidate_label",
        "base_rows",
        "candidate_rows",
        "common_rows",
        "base_only_rows",
        "candidate_only_rows",
        "components_compared",
        "threshold_exceedances",
    ]:
        print(f"  {key}: {report[key]}")
    print("  per_component:")
    for item in report["per_component"]:
        print(
            "    "
            f"{item['component']}: rows={item['rows']} "
            f"mean_abs={item['mean_abs_delta_m']:.6g} "
            f"rms={item['rms_delta_m']:.6g} "
            f"max_abs={item['max_abs_delta_m']:.6g}"
        )
    print("  identity_provenance:")
    for label, summary in report["identity_provenance"].items():
        print(
            f"    {label}: gps_l2w_rows={summary['gps_l2w_rows']} "
            f"gps_l2w_bias_exact_identity={summary['gps_l2w_bias_exact_identity_rows']} "
            f"gps_l2w_observation_exact_match={summary['gps_l2w_observation_exact_match_rows']} "
            f"gps_l2w_observation_family_fallback="
            f"{summary['gps_l2w_observation_family_fallback_rows']} "
            f"gps_l2w_code_bias_fallback={summary['gps_l2w_code_bias_fallback_rows']}"
        )
    if report["top_component_deltas"]:
        top = report["top_component_deltas"][0]
        print(
            "  max_delta: "
            f"{top['week']}:{top['tow']:.3f} {top['row_type']} "
            f"{top['sat']} f{top['freq']} {top['component']} "
            f"delta={top['delta_m']:.6g}"
        )
    if report["top_row_component_breakdowns"]:
        top = report["top_row_component_breakdowns"][0]
        top_components = ", ".join(
            f"{item['component']}={item['delta_m']:.6g}"
            for item in top["components"][:5]
        )
        print(
            "  max_row_delta: "
            f"{top['week']}:{top['tow']:.3f} {top['row_type']} "
            f"{top['sat']} f{top['freq']} {top['rinex_code']} "
            f"sum_abs={top['sum_abs_delta_m']:.6g} "
            f"components=[{top_components}]"
        )


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Compare CLAS zero-difference correction component CSV dumps."
    )
    parser.add_argument("base_csv", type=Path)
    parser.add_argument("candidate_csv", type=Path)
    parser.add_argument("--base-label", default="oracle")
    parser.add_argument("--candidate-label", default="native")
    parser.add_argument("--stage", help="compare only rows with this native/bridge stage")
    parser.add_argument("--row-type", choices=["code", "phase"], help="compare only one row type")
    parser.add_argument(
        "--sat",
        action="append",
        help="compare only one satellite label, such as G14; may be repeated or comma-separated",
    )
    parser.add_argument(
        "--freq",
        action="append",
        help="compare only one frequency index; may be repeated or comma-separated",
    )
    parser.add_argument(
        "--rinex-code",
        action="append",
        help="compare only one row-specific RINEX observation code, such as C2W or L2W",
    )
    parser.add_argument(
        "--duplicate-policy",
        choices=["first", "last", "mean", "fail"],
        default="last",
        help=(
            "how to handle multiple rows with the same comparison key. "
            "Default last preserves the historical behavior."
        ),
    )
    parser.add_argument(
        "--component",
        dest="components",
        action="append",
        help="component name to compare; may be repeated. Defaults to the v1 component set.",
    )
    parser.add_argument("--component-threshold-m", type=float)
    parser.add_argument("--top-deltas", type=int, default=20)
    parser.add_argument("--top-unmatched", type=int, default=20)
    parser.add_argument("--json-out", type=Path)
    parser.add_argument("--details-csv", type=Path)
    parser.add_argument(
        "--fail-on-diff",
        action="store_true",
        help="return non-zero when rows are unmatched or a threshold is exceeded",
    )
    return parser.parse_args(argv)


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)
    component_names = args.components or sorted(COMPONENT_ALIASES)
    sat_filter = normalize_filter_values(args.sat)
    freq_filter = normalize_freq_filter(args.freq)
    rinex_code_filter = normalize_filter_values(args.rinex_code)
    base_rows = normalize_rows(
        read_csv_rows(args.base_csv),
        component_names=component_names,
        stage_filter=args.stage,
        row_type_filter=args.row_type,
        sat_filter=sat_filter,
        freq_filter=freq_filter,
        rinex_code_filter=rinex_code_filter,
    )
    candidate_rows = normalize_rows(
        read_csv_rows(args.candidate_csv),
        component_names=component_names,
        stage_filter=args.stage,
        row_type_filter=args.row_type,
        sat_filter=sat_filter,
        freq_filter=freq_filter,
        rinex_code_filter=rinex_code_filter,
    )
    try:
        report = build_report(
            base_rows,
            candidate_rows,
            base_label=args.base_label,
            candidate_label=args.candidate_label,
            threshold_m=args.component_threshold_m,
            top_deltas=args.top_deltas,
            top_unmatched=args.top_unmatched,
            duplicate_policy=args.duplicate_policy,
        )
    except ValueError as exc:
        print(f"clas_zd_component_diff: {exc}", file=sys.stderr)
        return 2
    report["base_csv"] = str(args.base_csv)
    report["candidate_csv"] = str(args.candidate_csv)
    report["stage_filter"] = args.stage
    report["row_type_filter"] = args.row_type
    report["sat_filter"] = sorted(sat_filter) if sat_filter is not None else None
    report["freq_filter"] = sorted(freq_filter) if freq_filter is not None else None
    report["rinex_code_filter"] = (
        sorted(rinex_code_filter) if rinex_code_filter is not None else None
    )
    report["component_names"] = component_names
    print_summary(report)
    if args.json_out is not None:
        args.json_out.parent.mkdir(parents=True, exist_ok=True)
        args.json_out.write_text(
            json.dumps(report, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
    if args.details_csv is not None:
        write_details_csv(args.details_csv, report)
    if args.fail_on_diff and (
        report["base_only_rows"] > 0
        or report["candidate_only_rows"] > 0
        or report["threshold_exceedances"] > 0
    ):
        return 2
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except BrokenPipeError:
        raise SystemExit(1)
