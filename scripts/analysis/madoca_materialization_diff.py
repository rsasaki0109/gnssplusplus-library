#!/usr/bin/env python3
"""Compare MADOCA materialization snapshot CSV dumps."""

from __future__ import annotations

import argparse
import csv
import json
import math
from collections import Counter, defaultdict
from dataclasses import dataclass
from decimal import Decimal, InvalidOperation, ROUND_HALF_UP
from pathlib import Path
from typing import Any, DefaultDict, Iterable, Optional, Sequence


SCHEMA = "madoca_materialization_diff.v1"
SNAPSHOT_SCHEMA = "madoca_materialization_snapshot.v1"

REQUIRED_COLUMNS = [
    "schema_version",
    "sat",
    "system",
    "prn",
    "week",
    "tow",
    "orbit_frame",
    "orbit_valid",
    "clock_valid",
    "code_bias_valid",
    "phase_bias_valid",
    "orbit_week",
    "orbit_tow",
    "clock_week",
    "clock_tow",
    "iode",
    "ssr_orbit_iod",
    "ssr_clock_iod",
    "orbit_radial_m",
    "orbit_along_m",
    "orbit_cross_m",
    "clock_m",
    "code_bias_count",
    "code_biases_m",
    "phase_bias_count",
    "phase_biases_m",
    "phase_bias_discnt",
]

DISCRETE_FIELDS = [
    "system",
    "prn",
    "orbit_frame",
    "orbit_valid",
    "clock_valid",
    "code_bias_valid",
    "phase_bias_valid",
    "orbit_week",
    "orbit_tow",
    "clock_week",
    "clock_tow",
    "iode",
    "ssr_orbit_iod",
    "ssr_clock_iod",
    "code_bias_count",
    "phase_bias_count",
    "code_bias_ids",
    "phase_bias_ids",
    "phase_bias_discnt",
]

NUMERIC_FIELDS = [
    "orbit_radial_m",
    "orbit_along_m",
    "orbit_cross_m",
    "clock_m",
]


Row = dict[str, str]
Group = tuple[str, int, int]
MatchKey = tuple[Group, int]


@dataclass(frozen=True)
class MaterializationRow:
    key: MatchKey
    group: Group
    occurrence: int
    sat: str
    week: int
    tow: float
    source_row: int
    discrete: dict[str, str]
    numeric: dict[str, float]


def tow_to_millis(value: str) -> int:
    try:
        tow = Decimal(str(value))
    except InvalidOperation as exc:
        raise ValueError(f"invalid tow value: {value}") from exc
    return int((tow * Decimal("1000")).to_integral_value(rounding=ROUND_HALF_UP))


def to_int_text(value: str) -> str:
    if value == "":
        return "0"
    try:
        return str(int(float(value)))
    except ValueError:
        return value


def normalize_float_text(value: str) -> str:
    if value == "":
        return ""
    try:
        parsed = float(value)
    except ValueError:
        return value
    return f"{parsed:.12g}" if math.isfinite(parsed) else value


def to_float(value: str) -> Optional[float]:
    if value == "":
        return None
    try:
        parsed = float(value)
    except ValueError:
        return None
    return parsed if math.isfinite(parsed) else None


def parse_numeric_map(text: str) -> dict[str, float]:
    values: dict[str, float] = {}
    if text == "":
        return values
    for part in text.split(";"):
        if ":" not in part:
            continue
        key, value = part.split(":", 1)
        parsed = to_float(value)
        if key != "" and parsed is not None:
            values[to_int_text(key)] = parsed
    return values


def parse_discrete_map(text: str) -> dict[str, str]:
    values: dict[str, str] = {}
    if text == "":
        return values
    for part in text.split(";"):
        if ":" not in part:
            continue
        key, value = part.split(":", 1)
        if key != "":
            values[to_int_text(key)] = to_int_text(value)
    return values


def format_ids(values: Iterable[str]) -> str:
    return ";".join(sorted(values, key=lambda item: int(item)))


def format_discrete_map(values: dict[str, str]) -> str:
    return ";".join(
        f"{key}:{values[key]}" for key in sorted(values, key=lambda item: int(item))
    )


def read_csv_rows(path: Path) -> list[Row]:
    with path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        if reader.fieldnames is None:
            raise ValueError(f"{path}: missing CSV header")
        missing = [column for column in REQUIRED_COLUMNS if column not in reader.fieldnames]
        if missing:
            raise ValueError(f"{path}: missing required columns: {', '.join(missing)}")
        rows = [dict(row) for row in reader]
    bad_schema_rows = [
        index
        for index, row in enumerate(rows, start=2)
        if row.get("schema_version") != SNAPSHOT_SCHEMA
    ]
    if bad_schema_rows:
        first = bad_schema_rows[0]
        raise ValueError(
            f"{path}: row {first} has unsupported schema_version "
            f"{rows[first - 2].get('schema_version')!r}"
        )
    return rows


def group_from_row(row: Row) -> Group:
    return (row["sat"], int(float(row["week"])), tow_to_millis(row["tow"]))


def discrete_from_row(row: Row) -> dict[str, str]:
    code_biases = parse_numeric_map(row.get("code_biases_m", ""))
    phase_biases = parse_numeric_map(row.get("phase_biases_m", ""))
    discnt = parse_discrete_map(row.get("phase_bias_discnt", ""))
    return {
        "system": row.get("system", ""),
        "prn": to_int_text(row.get("prn", "")),
        "orbit_frame": row.get("orbit_frame", ""),
        "orbit_valid": to_int_text(row.get("orbit_valid", "")),
        "clock_valid": to_int_text(row.get("clock_valid", "")),
        "code_bias_valid": to_int_text(row.get("code_bias_valid", "")),
        "phase_bias_valid": to_int_text(row.get("phase_bias_valid", "")),
        "orbit_week": to_int_text(row.get("orbit_week", "")),
        "orbit_tow": normalize_float_text(row.get("orbit_tow", "")),
        "clock_week": to_int_text(row.get("clock_week", "")),
        "clock_tow": normalize_float_text(row.get("clock_tow", "")),
        "iode": to_int_text(row.get("iode", "")),
        "ssr_orbit_iod": to_int_text(row.get("ssr_orbit_iod", "")),
        "ssr_clock_iod": to_int_text(row.get("ssr_clock_iod", "")),
        "code_bias_count": to_int_text(row.get("code_bias_count", "")),
        "phase_bias_count": to_int_text(row.get("phase_bias_count", "")),
        "code_bias_ids": format_ids(code_biases),
        "phase_bias_ids": format_ids(phase_biases),
        "phase_bias_discnt": format_discrete_map(discnt),
    }


def numeric_from_row(row: Row) -> dict[str, float]:
    values: dict[str, float] = {}
    for field in NUMERIC_FIELDS:
        parsed = to_float(row.get(field, ""))
        if parsed is not None:
            values[field] = parsed
    for signal_id, value in parse_numeric_map(row.get("code_biases_m", "")).items():
        values[f"code_bias_m[{signal_id}]"] = value
    for signal_id, value in parse_numeric_map(row.get("phase_biases_m", "")).items():
        values[f"phase_bias_m[{signal_id}]"] = value
    return values


def normalize_rows(rows: Sequence[Row]) -> list[MaterializationRow]:
    normalized: list[MaterializationRow] = []
    occurrence_counts: Counter[Group] = Counter()
    for source_row, row in enumerate(rows, start=2):
        group = group_from_row(row)
        occurrence = occurrence_counts[group]
        occurrence_counts[group] += 1
        sat, week, tow_millis = group
        normalized.append(
            MaterializationRow(
                key=(group, occurrence),
                group=group,
                occurrence=occurrence,
                sat=sat,
                week=week,
                tow=tow_millis / 1000.0,
                source_row=source_row,
                discrete=discrete_from_row(row),
                numeric=numeric_from_row(row),
            )
        )
    return normalized


def rows_by_key(
    rows: Sequence[MaterializationRow],
) -> tuple[dict[MatchKey, MaterializationRow], Counter[MatchKey]]:
    grouped: dict[MatchKey, MaterializationRow] = {}
    duplicates: Counter[MatchKey] = Counter()
    for row in rows:
        if row.key in grouped:
            duplicates[row.key] += 1
        grouped[row.key] = row
    return grouped, duplicates


def row_summary(row: MaterializationRow) -> dict[str, Any]:
    return {
        "sat": row.sat,
        "week": row.week,
        "tow": row.tow,
        "occurrence": row.occurrence,
        "source_row": row.source_row,
        "system": row.discrete.get("system", ""),
        "prn": row.discrete.get("prn", ""),
        "numeric_components": sorted(row.numeric),
    }


def component_stats(values: Sequence[float]) -> dict[str, Any]:
    if not values:
        return {
            "rows": 0,
            "mean_abs_delta": 0.0,
            "rms_delta": 0.0,
            "max_abs_delta": 0.0,
        }
    abs_values = [abs(value) for value in values]
    return {
        "rows": len(values),
        "mean_abs_delta": sum(abs_values) / len(abs_values),
        "rms_delta": math.sqrt(sum(value * value for value in values) / len(values)),
        "max_abs_delta": max(abs_values),
    }


def build_report(
    base_rows: Sequence[MaterializationRow],
    candidate_rows: Sequence[MaterializationRow],
    *,
    base_label: str,
    candidate_label: str,
    threshold: Optional[float],
    top_deltas: int,
    top_unmatched: int,
    top_mismatches: int,
) -> dict[str, Any]:
    base_by_key, base_duplicates = rows_by_key(base_rows)
    candidate_by_key, candidate_duplicates = rows_by_key(candidate_rows)
    base_keys = set(base_by_key)
    candidate_keys = set(candidate_by_key)
    common_keys = sorted(base_keys & candidate_keys)
    base_only_keys = sorted(base_keys - candidate_keys)
    candidate_only_keys = sorted(candidate_keys - base_keys)

    numeric_deltas_by_component: DefaultDict[str, list[float]] = defaultdict(list)
    numeric_details: list[dict[str, Any]] = []
    discrete_mismatches: list[dict[str, Any]] = []
    missing_numeric_components: Counter[str] = Counter()
    threshold_exceedances = 0

    for key in common_keys:
        base_row = base_by_key[key]
        candidate_row = candidate_by_key[key]
        for field in DISCRETE_FIELDS:
            base_value = base_row.discrete.get(field, "")
            candidate_value = candidate_row.discrete.get(field, "")
            if base_value == candidate_value:
                continue
            discrete_mismatches.append(
                {
                    "sat": base_row.sat,
                    "week": base_row.week,
                    "tow": base_row.tow,
                    "occurrence": base_row.occurrence,
                    "field": field,
                    "base_value": base_value,
                    "candidate_value": candidate_value,
                    "base_source_row": base_row.source_row,
                    "candidate_source_row": candidate_row.source_row,
                }
            )

        components = sorted(set(base_row.numeric) | set(candidate_row.numeric))
        for component in components:
            base_value = base_row.numeric.get(component)
            candidate_value = candidate_row.numeric.get(component)
            if base_value is None or candidate_value is None:
                missing_numeric_components[component] += 1
                continue
            delta = candidate_value - base_value
            abs_delta = abs(delta)
            numeric_deltas_by_component[component].append(delta)
            if threshold is not None and abs_delta > threshold:
                threshold_exceedances += 1
            numeric_details.append(
                {
                    "sat": base_row.sat,
                    "week": base_row.week,
                    "tow": base_row.tow,
                    "occurrence": base_row.occurrence,
                    "component": component,
                    "base_value": base_value,
                    "candidate_value": candidate_value,
                    "delta": delta,
                    "abs_delta": abs_delta,
                    "base_source_row": base_row.source_row,
                    "candidate_source_row": candidate_row.source_row,
                }
            )

    numeric_details.sort(
        key=lambda item: (
            -item["abs_delta"],
            item["week"],
            item["tow"],
            item["sat"],
            item["occurrence"],
            item["component"],
        )
    )
    discrete_mismatches.sort(
        key=lambda item: (
            item["week"],
            item["tow"],
            item["sat"],
            item["occurrence"],
            item["field"],
        )
    )
    per_component = [
        {"component": component, **component_stats(values)}
        for component, values in sorted(numeric_deltas_by_component.items())
    ]
    per_component.sort(key=lambda item: (-item["max_abs_delta"], item["component"]))

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
        "row_set_complete": len(base_only_keys) == 0 and len(candidate_only_keys) == 0,
        "discrete_fields_compared": len(common_keys) * len(DISCRETE_FIELDS),
        "discrete_mismatches": len(discrete_mismatches),
        "numeric_components_compared": sum(item["rows"] for item in per_component),
        "numeric_threshold": threshold,
        "numeric_threshold_exceedances": threshold_exceedances,
        "missing_numeric_component_counts": [
            {"component": component, "rows": rows}
            for component, rows in missing_numeric_components.most_common()
        ],
        "per_numeric_component": per_component,
        "top_numeric_deltas": numeric_details[:top_deltas],
        "top_discrete_mismatches": discrete_mismatches[:top_mismatches],
        "top_base_only": [row_summary(base_by_key[key]) for key in base_only_keys[:top_unmatched]],
        "top_candidate_only": [
            row_summary(candidate_by_key[key]) for key in candidate_only_keys[:top_unmatched]
        ],
    }


def write_details_csv(path: Path, report: dict[str, Any]) -> None:
    fieldnames = [
        "kind",
        "sat",
        "week",
        "tow",
        "occurrence",
        "name",
        "base_value",
        "candidate_value",
        "delta",
        "abs_delta",
        "base_source_row",
        "candidate_source_row",
    ]
    rows: list[dict[str, Any]] = []
    for item in report.get("top_numeric_deltas", []):
        rows.append(
            {
                "kind": "numeric",
                "sat": item["sat"],
                "week": item["week"],
                "tow": item["tow"],
                "occurrence": item["occurrence"],
                "name": item["component"],
                "base_value": item["base_value"],
                "candidate_value": item["candidate_value"],
                "delta": item["delta"],
                "abs_delta": item["abs_delta"],
                "base_source_row": item["base_source_row"],
                "candidate_source_row": item["candidate_source_row"],
            }
        )
    for item in report.get("top_discrete_mismatches", []):
        rows.append(
            {
                "kind": "discrete",
                "sat": item["sat"],
                "week": item["week"],
                "tow": item["tow"],
                "occurrence": item["occurrence"],
                "name": item["field"],
                "base_value": item["base_value"],
                "candidate_value": item["candidate_value"],
                "delta": "",
                "abs_delta": "",
                "base_source_row": item["base_source_row"],
                "candidate_source_row": item["candidate_source_row"],
            }
        )
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def print_summary(report: dict[str, Any]) -> None:
    print("madoca_materialization_diff:")
    for key in [
        "schema",
        "base_label",
        "candidate_label",
        "base_rows",
        "candidate_rows",
        "common_rows",
        "base_only_rows",
        "candidate_only_rows",
        "row_set_complete",
        "discrete_mismatches",
        "numeric_components_compared",
        "numeric_threshold_exceedances",
    ]:
        print(f"  {key}: {report[key]}")
    print("  per_numeric_component:")
    for item in report["per_numeric_component"]:
        print(
            "    "
            f"{item['component']}: rows={item['rows']} "
            f"mean_abs={item['mean_abs_delta']:.6g} "
            f"rms={item['rms_delta']:.6g} "
            f"max_abs={item['max_abs_delta']:.6g}"
        )
    if report["top_numeric_deltas"]:
        top = report["top_numeric_deltas"][0]
        print(
            "  max_numeric_delta: "
            f"{top['week']}:{top['tow']:.3f} {top['sat']} "
            f"{top['component']} delta={top['delta']:.6g}"
        )
    if report["top_discrete_mismatches"]:
        top = report["top_discrete_mismatches"][0]
        print(
            "  first_discrete_mismatch: "
            f"{top['week']}:{top['tow']:.3f} {top['sat']} "
            f"{top['field']} {top['base_value']!r} != {top['candidate_value']!r}"
        )


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Compare MADOCA materialization snapshot CSV dumps."
    )
    parser.add_argument("base_materialization_csv", type=Path)
    parser.add_argument("candidate_materialization_csv", type=Path)
    parser.add_argument("--base-label", default="oracle")
    parser.add_argument("--candidate-label", default="native")
    parser.add_argument("--numeric-threshold", type=float)
    parser.add_argument("--top-deltas", type=int, default=20)
    parser.add_argument("--top-unmatched", type=int, default=20)
    parser.add_argument("--top-mismatches", type=int, default=20)
    parser.add_argument("--json-out", type=Path)
    parser.add_argument("--details-csv", type=Path)
    parser.add_argument(
        "--fail-on-diff",
        action="store_true",
        help="return non-zero when rows, discrete fields, or numeric thresholds differ",
    )
    return parser.parse_args(argv)


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)
    base_rows = normalize_rows(read_csv_rows(args.base_materialization_csv))
    candidate_rows = normalize_rows(read_csv_rows(args.candidate_materialization_csv))
    report = build_report(
        base_rows,
        candidate_rows,
        base_label=args.base_label,
        candidate_label=args.candidate_label,
        threshold=args.numeric_threshold,
        top_deltas=args.top_deltas,
        top_unmatched=args.top_unmatched,
        top_mismatches=args.top_mismatches,
    )
    report["base_materialization_csv"] = str(args.base_materialization_csv)
    report["candidate_materialization_csv"] = str(args.candidate_materialization_csv)
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
        or report["base_duplicate_keys"] > 0
        or report["candidate_duplicate_keys"] > 0
        or report["discrete_mismatches"] > 0
        or report["numeric_threshold_exceedances"] > 0
    ):
        return 2
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except BrokenPipeError:
        raise SystemExit(1)
