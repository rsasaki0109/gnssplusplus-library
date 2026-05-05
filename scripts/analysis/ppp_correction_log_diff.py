#!/usr/bin/env python3
"""Summarize and compare gnss_ppp --ppp-correction-log CSV files."""

from __future__ import annotations

import argparse
import csv
import json
import math
from collections import defaultdict
from decimal import Decimal, InvalidOperation, ROUND_HALF_UP
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple


REQUIRED_COLUMNS = [
    "week",
    "tow",
    "sat",
    "ssr_available",
    "orbit_clock_applied",
    "orbit_dx_m",
    "orbit_dy_m",
    "orbit_dz_m",
    "clock_m",
    "ura_sigma_m",
    "code_bias_m",
    "phase_bias_m",
    "trop_m",
    "stec_tecu",
    "iono_m",
    "ionosphere_estimation_constraint",
    "dcb_applied",
    "dcb_bias_m",
    "ionex_applied",
    "ionex_iono_m",
    "atmos_token_count",
    "elevation_deg",
    "variance_pr",
    "variance_cp",
    "valid_after_corrections",
    "solution_status",
]

OPTIONAL_COLUMN_DEFAULTS = {
    "ssr_orbit_iode": "-1",
    "broadcast_iode": "-1",
    "orbit_clock_skip_reason": "",
    "frequency_index": "0",
    "ionosphere_coefficient": "1.0",
    "has_carrier_phase": "0",
    "solid_tide_x_m": "0.0",
    "solid_tide_y_m": "0.0",
    "solid_tide_z_m": "0.0",
    "madocalib_solid_tide_x_m": "0.0",
    "madocalib_solid_tide_y_m": "0.0",
    "madocalib_solid_tide_z_m": "0.0",
    "madocalib_solid_tide_diff_norm_m": "0.0",
    "receiver_antenna_offset_x_m": "0.0",
    "receiver_antenna_offset_y_m": "0.0",
    "receiver_antenna_offset_z_m": "0.0",
    "receiver_antenna_pco_los_m": "0.0",
    "receiver_antenna_pcv_m": "0.0",
    "trop_mapping_wet": "0.0",
    "modeled_trop_delay_m": "0.0",
    "modeled_zenith_trop_delay_m": "0.0",
    "estimated_trop_delay_m": "0.0",
    "madocalib_trop_mapping_dry": "0.0",
    "madocalib_trop_mapping_wet": "0.0",
    "madocalib_trop_mapping_wet_diff": "0.0",
    "madocalib_zenith_hydrostatic_delay_m": "0.0",
    "madocalib_estimated_trop_delay_m": "0.0",
    "madocalib_estimated_trop_delay_diff_m": "0.0",
}

NUMERIC_COLUMNS = [
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
    "solid_tide_x_m",
    "solid_tide_y_m",
    "solid_tide_z_m",
    "madocalib_solid_tide_x_m",
    "madocalib_solid_tide_y_m",
    "madocalib_solid_tide_z_m",
    "madocalib_solid_tide_diff_norm_m",
    "receiver_antenna_offset_x_m",
    "receiver_antenna_offset_y_m",
    "receiver_antenna_offset_z_m",
    "receiver_antenna_pco_los_m",
    "receiver_antenna_pcv_m",
    "trop_mapping_wet",
    "modeled_trop_delay_m",
    "modeled_zenith_trop_delay_m",
    "estimated_trop_delay_m",
    "madocalib_trop_mapping_dry",
    "madocalib_trop_mapping_wet",
    "madocalib_trop_mapping_wet_diff",
    "madocalib_zenith_hydrostatic_delay_m",
    "madocalib_estimated_trop_delay_m",
    "madocalib_estimated_trop_delay_diff_m",
]

BOOL_COLUMNS = [
    "ssr_available",
    "orbit_clock_applied",
    "has_carrier_phase",
    "ionosphere_estimation_constraint",
    "dcb_applied",
    "ionex_applied",
    "valid_after_corrections",
]

COUNT_COLUMNS = ["atmos_token_count", "frequency_index"]

ID_COLUMNS = ["ssr_orbit_iode", "broadcast_iode"]

TEXT_COLUMNS = [
    "primary_signal",
    "secondary_signal",
    "primary_observation_code",
    "secondary_observation_code",
    "orbit_clock_skip_reason",
]

DIFF_NUMERIC_COLUMNS = NUMERIC_COLUMNS + COUNT_COLUMNS + ID_COLUMNS

Row = Dict[str, str]
Key = Tuple[int, int, str, str, str, str, str]


def tow_to_millis(value: str) -> int:
    try:
        tow = Decimal(str(value))
    except InvalidOperation as exc:
        raise ValueError(f"invalid tow value: {value}") from exc
    return int((tow * Decimal("1000")).to_integral_value(rounding=ROUND_HALF_UP))


def key_for_row(row: Row) -> Key:
    return (
        int(row["week"]),
        tow_to_millis(row["tow"]),
        row["sat"],
        row.get("primary_signal", ""),
        row.get("secondary_signal", ""),
        row.get("primary_observation_code", ""),
        row.get("secondary_observation_code", ""),
    )


def format_key(key: Key) -> str:
    week, tow_ms, sat, primary_signal, secondary_signal, primary_code, secondary_code = key
    base = f"{week}:{tow_ms / 1000.0:.3f}:{sat}"
    if any([primary_signal, secondary_signal, primary_code, secondary_code]):
        return f"{base}:{primary_signal}/{secondary_signal}:{primary_code}/{secondary_code}"
    return base


def to_float(value: str) -> float:
    if value == "":
        return 0.0
    try:
        parsed = float(value)
    except ValueError:
        return 0.0
    return parsed if math.isfinite(parsed) else 0.0


def to_int(value: str) -> int:
    if value == "":
        return 0
    try:
        return int(float(value))
    except ValueError:
        return 0


def truthy(value: str) -> bool:
    return str(value).strip().lower() in {"1", "true", "yes", "y"}


def nonzero(row: Row, column: str, tolerance: float = 0.0) -> bool:
    return abs(to_float(row[column])) > tolerance


def read_log(path: Path) -> List[Row]:
    with path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        if reader.fieldnames is None:
            raise ValueError(f"{path}: missing CSV header")
        missing = [column for column in REQUIRED_COLUMNS if column not in reader.fieldnames]
        if missing:
            raise ValueError(f"{path}: missing required columns: {', '.join(missing)}")
        rows = [dict(row) for row in reader]
    for row in rows:
        for column, default in OPTIONAL_COLUMN_DEFAULTS.items():
            row.setdefault(column, default)
        row["_key"] = format_key(key_for_row(row))
    return rows


def index_rows(rows: Sequence[Row]) -> Dict[Key, Row]:
    indexed: Dict[Key, Row] = {}
    duplicate_keys: List[str] = []
    for row in rows:
        key = key_for_row(row)
        if key in indexed:
            duplicate_keys.append(format_key(key))
        indexed[key] = row
    if duplicate_keys:
        sample = ", ".join(duplicate_keys[:5])
        raise ValueError(f"duplicate correction-log rows: {sample}")
    return indexed


def mean(values: Sequence[float]) -> float:
    return sum(values) / len(values) if values else 0.0


def summarize_rows(rows: Sequence[Row], *, top_satellites: int = 10) -> Dict[str, Any]:
    epochs = {(int(row["week"]), tow_to_millis(row["tow"])) for row in rows}
    satellites = {row["sat"] for row in rows}
    by_satellite: Dict[str, Dict[str, int]] = defaultdict(
        lambda: {
            "rows": 0,
            "valid_rows": 0,
            "ssr_available_rows": 0,
            "iono_nonzero_rows": 0,
            "trop_nonzero_rows": 0,
            "phase_bias_nonzero_rows": 0,
            "phase_capable_rows": 0,
            "orbit_clock_skip_rows": 0,
        }
    )

    orbit_norms = []
    orbit_clock_skip_reasons: Dict[str, int] = defaultdict(int)
    absolute_sums = {
        "clock_m": 0.0,
        "code_bias_m": 0.0,
        "phase_bias_m": 0.0,
        "trop_m": 0.0,
        "iono_m": 0.0,
        "dcb_bias_m": 0.0,
        "ionex_iono_m": 0.0,
    }

    counts = {
        "rows": len(rows),
        "epochs": len(epochs),
        "satellites": len(satellites),
        "valid_rows": 0,
        "ssr_available_rows": 0,
        "orbit_clock_applied_rows": 0,
        "atmos_token_rows": 0,
        "trop_nonzero_rows": 0,
        "stec_nonzero_rows": 0,
        "iono_nonzero_rows": 0,
        "code_bias_nonzero_rows": 0,
        "phase_bias_nonzero_rows": 0,
        "phase_capable_rows": 0,
        "ura_nonzero_rows": 0,
        "dcb_applied_rows": 0,
        "ionex_applied_rows": 0,
        "ssr_orbit_iode_rows": 0,
        "orbit_clock_skip_rows": 0,
    }

    for row in rows:
        sat_stats = by_satellite[row["sat"]]
        sat_stats["rows"] += 1
        if truthy(row["valid_after_corrections"]):
            counts["valid_rows"] += 1
            sat_stats["valid_rows"] += 1
        if truthy(row["ssr_available"]):
            counts["ssr_available_rows"] += 1
            sat_stats["ssr_available_rows"] += 1
        if to_int(row.get("ssr_orbit_iode", "-1")) >= 0:
            counts["ssr_orbit_iode_rows"] += 1
        skip_reason = row.get("orbit_clock_skip_reason", "")
        if skip_reason:
            counts["orbit_clock_skip_rows"] += 1
            orbit_clock_skip_reasons[skip_reason] += 1
            sat_stats["orbit_clock_skip_rows"] += 1
        if truthy(row["orbit_clock_applied"]):
            counts["orbit_clock_applied_rows"] += 1
        if to_int(row["atmos_token_count"]) > 0:
            counts["atmos_token_rows"] += 1
        if nonzero(row, "trop_m"):
            counts["trop_nonzero_rows"] += 1
            sat_stats["trop_nonzero_rows"] += 1
        if nonzero(row, "stec_tecu"):
            counts["stec_nonzero_rows"] += 1
        if nonzero(row, "iono_m"):
            counts["iono_nonzero_rows"] += 1
            sat_stats["iono_nonzero_rows"] += 1
        if nonzero(row, "code_bias_m"):
            counts["code_bias_nonzero_rows"] += 1
        if nonzero(row, "phase_bias_m"):
            counts["phase_bias_nonzero_rows"] += 1
            sat_stats["phase_bias_nonzero_rows"] += 1
        if truthy(row.get("has_carrier_phase", "0")):
            counts["phase_capable_rows"] += 1
            sat_stats["phase_capable_rows"] += 1
        if nonzero(row, "ura_sigma_m"):
            counts["ura_nonzero_rows"] += 1
        if truthy(row["dcb_applied"]):
            counts["dcb_applied_rows"] += 1
        if truthy(row["ionex_applied"]):
            counts["ionex_applied_rows"] += 1

        dx = to_float(row["orbit_dx_m"])
        dy = to_float(row["orbit_dy_m"])
        dz = to_float(row["orbit_dz_m"])
        orbit_norms.append(math.sqrt(dx * dx + dy * dy + dz * dz))
        for column in absolute_sums:
            absolute_sums[column] += abs(to_float(row[column]))

    top = sorted(
        by_satellite.items(),
        key=lambda item: (
            item[1]["iono_nonzero_rows"],
            item[1]["ssr_available_rows"],
            item[1]["rows"],
        ),
        reverse=True,
    )[:top_satellites]

    return {
        **counts,
        "mean_orbit_norm_m": mean(orbit_norms),
        "max_orbit_norm_m": max(orbit_norms) if orbit_norms else 0.0,
        "absolute_sums": absolute_sums,
        "orbit_clock_skip_reasons": dict(sorted(orbit_clock_skip_reasons.items())),
        "top_satellites": [{"sat": sat, **stats} for sat, stats in top],
    }


def column_delta_stats(base_rows: Sequence[Row], candidate_rows: Sequence[Row], column: str) -> Dict[str, float]:
    deltas = [
        to_float(candidate[column]) - to_float(base[column])
        for base, candidate in zip(base_rows, candidate_rows)
    ]
    abs_deltas = [abs(value) for value in deltas]
    return {
        "mean_delta": mean(deltas),
        "mean_abs_delta": mean(abs_deltas),
        "max_abs_delta": max(abs_deltas) if abs_deltas else 0.0,
        "rms_delta": math.sqrt(mean([value * value for value in deltas])) if deltas else 0.0,
    }


def first_differences(
    keys: Sequence[Key],
    base_by_key: Dict[Key, Row],
    candidate_by_key: Dict[Key, Row],
    *,
    tolerance: float,
    limit: int,
) -> List[Dict[str, Any]]:
    differences: List[Dict[str, Any]] = []
    for key in keys:
        base = base_by_key[key]
        candidate = candidate_by_key[key]
        changed_columns: Dict[str, Any] = {}
        for column in DIFF_NUMERIC_COLUMNS:
            delta = to_float(candidate[column]) - to_float(base[column])
            if abs(delta) > tolerance:
                changed_columns[column] = {
                    "base": to_float(base[column]),
                    "candidate": to_float(candidate[column]),
                    "delta": delta,
                }
        for column in BOOL_COLUMNS:
            if truthy(base[column]) != truthy(candidate[column]):
                changed_columns[column] = {
                    "base": truthy(base[column]),
                    "candidate": truthy(candidate[column]),
                }
        for column in TEXT_COLUMNS:
            if base.get(column, "") != candidate.get(column, ""):
                changed_columns[column] = {
                    "base": base.get(column, ""),
                    "candidate": candidate.get(column, ""),
                }
        if changed_columns:
            differences.append({"key": format_key(key), "columns": changed_columns})
            if len(differences) >= limit:
                break
    return differences


def compare_logs(
    base_rows: Sequence[Row],
    candidate_rows: Sequence[Row],
    *,
    tolerance: float = 1e-9,
    limit_differences: int = 12,
) -> Dict[str, Any]:
    base_by_key = index_rows(base_rows)
    candidate_by_key = index_rows(candidate_rows)
    base_keys = set(base_by_key)
    candidate_keys = set(candidate_by_key)
    common_keys = sorted(base_keys & candidate_keys)
    common_base = [base_by_key[key] for key in common_keys]
    common_candidate = [candidate_by_key[key] for key in common_keys]

    bool_changes = {}
    for column in BOOL_COLUMNS:
        bool_changes[column] = sum(
            1
            for base, candidate in zip(common_base, common_candidate)
            if truthy(base[column]) != truthy(candidate[column])
        )
    text_changes = {}
    for column in TEXT_COLUMNS:
        text_changes[column] = sum(
            1
            for base, candidate in zip(common_base, common_candidate)
            if base.get(column, "") != candidate.get(column, "")
        )

    return {
        "common_rows": len(common_keys),
        "base_only_rows": len(base_keys - candidate_keys),
        "candidate_only_rows": len(candidate_keys - base_keys),
        "numeric_delta_stats": {
            column: column_delta_stats(common_base, common_candidate, column)
            for column in DIFF_NUMERIC_COLUMNS
        },
        "bool_change_counts": bool_changes,
        "text_change_counts": text_changes,
        "first_differences": first_differences(
            common_keys,
            base_by_key,
            candidate_by_key,
            tolerance=tolerance,
            limit=limit_differences,
        ),
    }


def write_diff_csv(path: Path, base_rows: Sequence[Row], candidate_rows: Sequence[Row]) -> None:
    base_by_key = index_rows(base_rows)
    candidate_by_key = index_rows(candidate_rows)
    common_keys = sorted(set(base_by_key) & set(candidate_by_key))
    fieldnames = [
        "week",
        "tow",
        "sat",
        "primary_signal",
        "secondary_signal",
        "primary_observation_code",
        "secondary_observation_code",
    ]
    fieldnames += [f"{column}_delta" for column in DIFF_NUMERIC_COLUMNS]
    fieldnames += [f"{column}_changed" for column in BOOL_COLUMNS]
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for key in common_keys:
            base = base_by_key[key]
            candidate = candidate_by_key[key]
            (
                week,
                tow_ms,
                sat,
                primary_signal,
                secondary_signal,
                primary_code,
                secondary_code,
            ) = key
            row: Dict[str, Any] = {
                "week": week,
                "tow": f"{tow_ms / 1000.0:.3f}",
                "sat": sat,
                "primary_signal": primary_signal,
                "secondary_signal": secondary_signal,
                "primary_observation_code": primary_code,
                "secondary_observation_code": secondary_code,
            }
            for column in DIFF_NUMERIC_COLUMNS:
                row[f"{column}_delta"] = to_float(candidate[column]) - to_float(base[column])
            for column in BOOL_COLUMNS:
                row[f"{column}_changed"] = int(truthy(base[column]) != truthy(candidate[column]))
            writer.writerow(row)


def print_summary(label: str, summary: Dict[str, Any]) -> None:
    print(f"{label}:")
    for key in [
        "rows",
        "epochs",
        "satellites",
        "valid_rows",
        "ssr_available_rows",
        "orbit_clock_applied_rows",
        "atmos_token_rows",
        "trop_nonzero_rows",
        "iono_nonzero_rows",
        "code_bias_nonzero_rows",
        "phase_bias_nonzero_rows",
        "phase_capable_rows",
        "ura_nonzero_rows",
        "dcb_applied_rows",
        "ionex_applied_rows",
        "ssr_orbit_iode_rows",
        "orbit_clock_skip_rows",
    ]:
        print(f"  {key}: {summary[key]}")
    if summary.get("orbit_clock_skip_reasons"):
        print(f"  orbit_clock_skip_reasons: {summary['orbit_clock_skip_reasons']}")
    print(f"  mean_orbit_norm_m: {summary['mean_orbit_norm_m']:.6f}")
    print(f"  max_orbit_norm_m: {summary['max_orbit_norm_m']:.6f}")


def print_comparison(comparison: Dict[str, Any], *, max_columns: int = 8) -> None:
    print("comparison:")
    print(f"  common_rows: {comparison['common_rows']}")
    print(f"  base_only_rows: {comparison['base_only_rows']}")
    print(f"  candidate_only_rows: {comparison['candidate_only_rows']}")
    ranked = sorted(
        comparison["numeric_delta_stats"].items(),
        key=lambda item: item[1]["max_abs_delta"],
        reverse=True,
    )[:max_columns]
    print("  largest_numeric_deltas:")
    for column, stats in ranked:
        print(
            f"    {column}: max_abs={stats['max_abs_delta']:.6g} "
            f"mean_abs={stats['mean_abs_delta']:.6g}"
        )
    changed_bools = {
        column: count
        for column, count in comparison["bool_change_counts"].items()
        if count > 0
    }
    print(f"  changed_boolean_columns: {changed_bools}")
    changed_text = {
        column: count
        for column, count in comparison.get("text_change_counts", {}).items()
        if count > 0
    }
    print(f"  changed_text_columns: {changed_text}")
    if comparison["first_differences"]:
        print("  first_differences:")
        for item in comparison["first_differences"]:
            print(f"    {item['key']}: {', '.join(item['columns'].keys())}")


def build_report(args: argparse.Namespace) -> Dict[str, Any]:
    base_rows = read_log(args.base_log)
    report: Dict[str, Any] = {
        "base_log": str(args.base_log),
        "base_summary": summarize_rows(base_rows, top_satellites=args.top_satellites),
    }
    if args.candidate_log is not None:
        candidate_rows = read_log(args.candidate_log)
        report["candidate_log"] = str(args.candidate_log)
        report["candidate_summary"] = summarize_rows(
            candidate_rows,
            top_satellites=args.top_satellites,
        )
        report["comparison"] = compare_logs(
            base_rows,
            candidate_rows,
            tolerance=args.tolerance,
            limit_differences=args.limit_differences,
        )
        if args.diff_csv is not None:
            write_diff_csv(args.diff_csv, base_rows, candidate_rows)
            report["diff_csv"] = str(args.diff_csv)
    return report


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Summarize or compare gnss_ppp --ppp-correction-log CSV files."
    )
    parser.add_argument("base_log", type=Path)
    parser.add_argument("candidate_log", type=Path, nargs="?")
    parser.add_argument("--json-out", type=Path, help="write full report JSON")
    parser.add_argument("--diff-csv", type=Path, help="write per-common-row delta CSV when comparing")
    parser.add_argument("--top-satellites", type=int, default=10)
    parser.add_argument("--tolerance", type=float, default=1e-9)
    parser.add_argument("--limit-differences", type=int, default=12)
    return parser.parse_args(argv)


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)
    report = build_report(args)
    print_summary("base", report["base_summary"])
    if "candidate_summary" in report:
        print_summary("candidate", report["candidate_summary"])
        print_comparison(report["comparison"])
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
