#!/usr/bin/env python3
"""Compare MADOCA/MADOCALIB residual component CSV dumps."""

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


SCHEMA = "madoca_residual_component_diff.v1"

KEY_REQUIRED_COLUMNS = ["week", "tow", "iteration", "sat", "row_type"]

COMPONENT_ALIASES: dict[str, tuple[str, ...]] = {
    "residual_m": ("residual_m", "prefit_residual_m", "postfit_residual_m"),
    "iono_state_m": ("iono_state_m", "ionosphere_state_m", "dion_m"),
    "trop_m": ("trop_m", "troposphere_m", "trop_correction_m"),
    "clock_m": ("clock_m", "receiver_clock_m", "cdtr_m"),
    "sat_clock_m": ("sat_clock_m", "satellite_clock_m", "satclk_m"),
    "orbit_clock_m": ("orbit_clock_m", "orbit_clock_correction_m"),
    "code_bias_m": ("code_bias_m", "code_bias"),
    "phase_bias_m": ("phase_bias_m", "phase_bias"),
    "windup_m": ("windup_m", "phase_windup_m"),
    "antenna_m": ("antenna_m", "receiver_antenna_m", "satellite_antenna_m"),
    "variance_m2": ("variance_m2", "variance", "measurement_variance_m2"),
}

DEFAULT_COMPONENTS = ["residual_m"]


Row = dict[str, str]
Group = tuple[int, int, int, str]
Identity = tuple[str, str, str, str, str, int, str]
MatchKey = tuple[Group, Identity, int]


@dataclass(frozen=True)
class ResidualRow:
    key: MatchKey
    group: Group
    identity: Identity
    occurrence: int
    week: int
    tow: float
    iteration: int
    row_type: str
    sat: str
    source_row: int
    components: dict[str, float]


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


def normalize_float_text(value: str, default: str) -> str:
    if value == "":
        return default
    try:
        parsed = float(value)
    except ValueError:
        return value
    return f"{parsed:.12g}"


def first_present(row: Row, names: Iterable[str], default: str = "") -> str:
    for name in names:
        if name in row and row[name] != "":
            return row[name]
    return default


def read_csv_rows(path: Path) -> list[Row]:
    with path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        if reader.fieldnames is None:
            raise ValueError(f"{path}: missing CSV header")
        missing = [column for column in KEY_REQUIRED_COLUMNS if column not in reader.fieldnames]
        if missing:
            raise ValueError(f"{path}: missing required columns: {', '.join(missing)}")
        return [dict(row) for row in reader]


def group_from_row(row: Row) -> Group:
    return (
        to_int(row["week"]),
        tow_to_millis(row["tow"]),
        to_int(row.get("iteration", "0")),
        row.get("row_type", ""),
    )


def identity_from_row(row: Row) -> Identity:
    return (
        row.get("sat", ""),
        first_present(row, ("primary_signal", "signal_family", "signal", "band"), ""),
        first_present(row, ("secondary_signal",), ""),
        first_present(
            row,
            (
                "primary_observation_code",
                "primary_rinex_code",
                "pseudorange_rinex_code",
                "obs_code",
                "rinex_code",
                "rtklib_code",
                "code",
            ),
        ),
        first_present(
            row,
            (
                "secondary_observation_code",
                "secondary_rinex_code",
                "carrier_rinex_code",
                "secondary_rtklib_code",
            ),
        ),
        to_int(first_present(row, ("frequency_index", "freq", "frequency", "f"), "0")),
        normalize_float_text(row.get("ionosphere_coefficient", "1"), "1"),
    )


def extract_components(row: Row, component_names: Sequence[str]) -> dict[str, float]:
    components: dict[str, float] = {}
    for component in component_names:
        aliases = COMPONENT_ALIASES.get(component, (component,))
        value = to_float(first_present(row, aliases))
        if value is not None:
            components[component] = value
    return components


def normalize_rows(
    rows: Sequence[Row],
    *,
    component_names: Sequence[str],
    row_type_filter: Optional[str],
    iteration_filter: Optional[int],
) -> list[ResidualRow]:
    normalized: list[ResidualRow] = []
    occurrence_counts: Counter[tuple[Group, Identity]] = Counter()
    for source_row, row in enumerate(rows, start=2):
        row_type = row.get("row_type", "")
        if row_type_filter is not None and row_type != row_type_filter:
            continue
        iteration = to_int(row.get("iteration", "0"))
        if iteration_filter is not None and iteration != iteration_filter:
            continue
        components = extract_components(row, component_names)
        if not components:
            continue
        group = group_from_row(row)
        identity = identity_from_row(row)
        occurrence = occurrence_counts[(group, identity)]
        occurrence_counts[(group, identity)] += 1
        week, tow_millis, _, _ = group
        normalized.append(
            ResidualRow(
                key=(group, identity, occurrence),
                group=group,
                identity=identity,
                occurrence=occurrence,
                week=week,
                tow=tow_millis / 1000.0,
                iteration=iteration,
                row_type=row_type,
                sat=identity[0],
                source_row=source_row,
                components=components,
            )
        )
    return normalized


def rows_by_key(rows: Sequence[ResidualRow]) -> tuple[dict[MatchKey, ResidualRow], Counter[MatchKey]]:
    grouped: dict[MatchKey, ResidualRow] = {}
    duplicates: Counter[MatchKey] = Counter()
    for row in rows:
        if row.key in grouped:
            duplicates[row.key] += 1
        grouped[row.key] = row
    return grouped, duplicates


def group_to_dict(group: Group) -> dict[str, Any]:
    return {
        "week": group[0],
        "tow": group[1] / 1000.0,
        "iteration": group[2],
        "row_type": group[3],
    }


def identity_to_dict(identity: Identity) -> dict[str, Any]:
    return {
        "sat": identity[0],
        "primary_signal": identity[1],
        "secondary_signal": identity[2],
        "primary_observation_code": identity[3],
        "secondary_observation_code": identity[4],
        "frequency_index": identity[5],
        "ionosphere_coefficient": identity[6],
    }


def row_summary(row: ResidualRow) -> dict[str, Any]:
    return {
        **group_to_dict(row.group),
        **identity_to_dict(row.identity),
        "occurrence": row.occurrence,
        "source_row": row.source_row,
        "components": sorted(row.components),
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
    base_rows: Sequence[ResidualRow],
    candidate_rows: Sequence[ResidualRow],
    *,
    base_label: str,
    candidate_label: str,
    threshold: Optional[float],
    top_deltas: int,
    top_unmatched: int,
) -> dict[str, Any]:
    base_by_key, base_duplicates = rows_by_key(base_rows)
    candidate_by_key, candidate_duplicates = rows_by_key(candidate_rows)
    base_keys = set(base_by_key)
    candidate_keys = set(candidate_by_key)
    common_keys = sorted(base_keys & candidate_keys)
    base_only_keys = sorted(base_keys - candidate_keys)
    candidate_only_keys = sorted(candidate_keys - base_keys)

    component_deltas: DefaultDict[str, list[float]] = defaultdict(list)
    details: list[dict[str, Any]] = []
    missing_components: Counter[str] = Counter()
    threshold_exceedances = 0

    for key in common_keys:
        base_row = base_by_key[key]
        candidate_row = candidate_by_key[key]
        components = sorted(set(base_row.components) | set(candidate_row.components))
        for component in components:
            base_value = base_row.components.get(component)
            candidate_value = candidate_row.components.get(component)
            if base_value is None or candidate_value is None:
                missing_components[component] += 1
                continue
            delta = candidate_value - base_value
            abs_delta = abs(delta)
            component_deltas[component].append(delta)
            if threshold is not None and abs_delta > threshold:
                threshold_exceedances += 1
            details.append(
                {
                    **group_to_dict(base_row.group),
                    **identity_to_dict(base_row.identity),
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

    details.sort(
        key=lambda item: (
            -item["abs_delta"],
            item["week"],
            item["tow"],
            item["iteration"],
            item["row_type"],
            item["sat"],
            item["frequency_index"],
            item["component"],
        )
    )
    per_component = [
        {"component": component, **component_stats(values)}
        for component, values in sorted(component_deltas.items())
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
        "components_compared": sum(item["rows"] for item in per_component),
        "component_threshold": threshold,
        "threshold_exceedances": threshold_exceedances,
        "row_set_complete": len(base_only_keys) == 0 and len(candidate_only_keys) == 0,
        "missing_component_counts": [
            {"component": component, "rows": rows}
            for component, rows in missing_components.most_common()
        ],
        "per_component": per_component,
        "top_component_deltas": details[:top_deltas],
        "top_base_only": [row_summary(base_by_key[key]) for key in base_only_keys[:top_unmatched]],
        "top_candidate_only": [
            row_summary(candidate_by_key[key]) for key in candidate_only_keys[:top_unmatched]
        ],
    }


def write_details_csv(path: Path, report: dict[str, Any]) -> None:
    fieldnames = [
        "week",
        "tow",
        "iteration",
        "row_type",
        "sat",
        "primary_signal",
        "secondary_signal",
        "primary_observation_code",
        "secondary_observation_code",
        "frequency_index",
        "ionosphere_coefficient",
        "occurrence",
        "component",
        "base_value",
        "candidate_value",
        "delta",
        "abs_delta",
        "base_source_row",
        "candidate_source_row",
    ]
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(report.get("top_component_deltas", []))


def print_summary(report: dict[str, Any]) -> None:
    print("madoca_residual_component_diff:")
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
        "row_set_complete",
    ]:
        print(f"  {key}: {report[key]}")
    print("  per_component:")
    for item in report["per_component"]:
        print(
            "    "
            f"{item['component']}: rows={item['rows']} "
            f"mean_abs={item['mean_abs_delta']:.6g} "
            f"rms={item['rms_delta']:.6g} "
            f"max_abs={item['max_abs_delta']:.6g}"
        )
    if report["top_component_deltas"]:
        top = report["top_component_deltas"][0]
        print(
            "  max_delta: "
            f"{top['week']}:{top['tow']:.3f} iter={top['iteration']} "
            f"{top['row_type']} {top['sat']} f{top['frequency_index']} "
            f"{top['component']} delta={top['delta']:.6g}"
        )


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Compare common-row residual components between MADOCALIB and native PPP CSV dumps."
    )
    parser.add_argument("base_residual_csv", type=Path)
    parser.add_argument("candidate_residual_csv", type=Path)
    parser.add_argument("--base-label", default="madocalib")
    parser.add_argument("--candidate-label", default="native")
    parser.add_argument("--row-type", help="compare only one row_type, for example phase")
    parser.add_argument("--iteration", type=int, help="compare only one filter iteration")
    parser.add_argument(
        "--component",
        dest="components",
        action="append",
        help="component name to compare; may be repeated. Defaults to residual_m.",
    )
    parser.add_argument("--component-threshold", type=float)
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
    component_names = args.components or DEFAULT_COMPONENTS
    base_rows = normalize_rows(
        read_csv_rows(args.base_residual_csv),
        component_names=component_names,
        row_type_filter=args.row_type,
        iteration_filter=args.iteration,
    )
    candidate_rows = normalize_rows(
        read_csv_rows(args.candidate_residual_csv),
        component_names=component_names,
        row_type_filter=args.row_type,
        iteration_filter=args.iteration,
    )
    report = build_report(
        base_rows,
        candidate_rows,
        base_label=args.base_label,
        candidate_label=args.candidate_label,
        threshold=args.component_threshold,
        top_deltas=args.top_deltas,
        top_unmatched=args.top_unmatched,
    )
    report["base_residual_csv"] = str(args.base_residual_csv)
    report["candidate_residual_csv"] = str(args.candidate_residual_csv)
    report["row_type_filter"] = args.row_type
    report["iteration_filter"] = args.iteration
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
