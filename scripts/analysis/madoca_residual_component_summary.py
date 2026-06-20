#!/usr/bin/env python3
"""Summarize and validate a MADOCA residual-component CSV dump."""

from __future__ import annotations

import argparse
from collections import Counter
import json
from pathlib import Path
import sys
from typing import Any, Mapping, Sequence


sys.path.insert(0, str(Path(__file__).resolve().parent))

import madoca_residual_component_diff as residual_diff  # noqa: E402


SCHEMA = "madoca_residual_component_summary.v2"


def _format_counter(counter: Counter[str]) -> dict[str, int]:
    return {key: counter[key] for key in sorted(counter)}


def _format_top(counter: Counter[str], limit: int) -> list[dict[str, object]]:
    return [
        {"key": key, "count": count}
        for key, count in sorted(counter.items(), key=lambda item: (-item[1], item[0]))[:limit]
    ]


def _positive_count(counter: Mapping[str, Any], key: str) -> bool:
    try:
        return int(counter.get(str(key), 0)) > 0
    except (TypeError, ValueError):
        return False


def _sat_system(sat: str) -> str:
    prefix = sat[:1].upper()
    return {
        "G": "GPS",
        "R": "GLONASS",
        "E": "Galileo",
        "J": "QZSS",
        "Q": "QZSS",
        "C": "BeiDou",
        "B": "BeiDou",
        "I": "IRNSS",
        "S": "SBAS",
    }.get(prefix, prefix or "unknown")


def _parse_int_text(value: str) -> str:
    try:
        return str(int(float(value)))
    except (TypeError, ValueError):
        return value


def _row_key(row: Mapping[str, str]) -> tuple[residual_diff.Group, residual_diff.Identity]:
    return (residual_diff.group_from_row(dict(row)), residual_diff.identity_from_row(dict(row)))


def summarize_rows(rows: Sequence[dict[str, str]], *, top_limit: int = 20) -> dict[str, object]:
    systems: Counter[str] = Counter()
    satellites: Counter[str] = Counter()
    row_types: Counter[str] = Counter()
    iterations: Counter[str] = Counter()
    primary_codes: Counter[str] = Counter()
    secondary_codes: Counter[str] = Counter()
    frequency_indices: Counter[str] = Counter()
    ionosphere_coefficients: Counter[str] = Counter()
    component_rows: Counter[str] = Counter()
    key_counts: Counter[tuple[residual_diff.Group, residual_diff.Identity]] = Counter()
    issue_counts: Counter[str] = Counter()
    issue_examples: dict[str, list[dict[str, object]]] = {}
    time_keys: list[tuple[int, int]] = []

    component_names = sorted(residual_diff.COMPONENT_ALIASES)

    def record_issue(kind: str, row_number: int, detail: str) -> None:
        issue_counts[kind] += 1
        examples = issue_examples.setdefault(kind, [])
        if len(examples) < 5:
            examples.append({"row": row_number, "detail": detail})

    for row_number, row in enumerate(rows, start=2):
        sat = row.get("sat", "")
        row_type = row.get("row_type", "")
        if not sat:
            record_issue("missing_satellite", row_number, "")
        if not row_type:
            record_issue("missing_row_type", row_number, "")

        systems[_sat_system(sat)] += 1
        satellites[sat] += 1
        row_types[row_type] += 1
        iterations[_parse_int_text(row.get("iteration", ""))] += 1

        try:
            key_group, key_identity = _row_key(row)
            key_counts[(key_group, key_identity)] += 1
            time_keys.append((key_group[0], key_group[1]))
        except (ValueError, TypeError) as exc:
            record_issue("bad_row_key", row_number, str(exc))

        identity = residual_diff.identity_from_row(dict(row))
        primary_codes[identity[3]] += 1
        secondary_codes[identity[4]] += 1
        frequency_indices[str(identity[5])] += 1
        ionosphere_coefficients[identity[6]] += 1

        components = residual_diff.extract_components(row, component_names)
        if not components:
            record_issue("missing_numeric_component", row_number, "")
        for component in components:
            component_rows[component] += 1

    duplicate_groups = [group for group, count in key_counts.items() if count > 1]
    max_duplicate_occurrences = max(key_counts.values(), default=0)
    if time_keys:
        first_week, first_tow_ms = min(time_keys)
        last_week, last_tow_ms = max(time_keys)
        time_range: dict[str, object] = {
            "start_week": first_week,
            "start_tow": first_tow_ms / 1000.0,
            "end_week": last_week,
            "end_tow": last_tow_ms / 1000.0,
        }
    else:
        time_range = {}

    return {
        "schema": SCHEMA,
        "status": "failed" if issue_counts else "passed",
        "rows": len(rows),
        "unique_satellites": len(satellites),
        "systems": _format_counter(systems),
        "top_satellites": _format_top(satellites, top_limit),
        "time_range": time_range,
        "row_types": _format_counter(row_types),
        "iterations": _format_counter(iterations),
        "observation_identity": {
            "primary_observation_codes": _format_counter(primary_codes),
            "secondary_observation_codes": _format_counter(secondary_codes),
            "frequency_indices": _format_counter(frequency_indices),
            "ionosphere_coefficients": _format_counter(ionosphere_coefficients),
        },
        "component_presence": {
            "rows_with_component": _format_counter(component_rows),
            "known_components": component_names,
        },
        "row_key": {
            "groups": len(key_counts),
            "duplicate_groups": len(duplicate_groups),
            "max_duplicate_occurrences": max_duplicate_occurrences,
        },
        "issue_counts": _format_counter(issue_counts),
        "issue_examples": issue_examples,
    }


def requirement_failures(summary: Mapping[str, Any], args: argparse.Namespace) -> list[str]:
    failures: list[str] = []
    rows = summary.get("rows", 0)
    if not isinstance(rows, int):
        failures.append("summary rows is not an integer")
    elif rows < args.require_rows_min:
        failures.append(f"rows {rows} < required minimum {args.require_rows_min}")

    systems = summary.get("systems", {})
    if not isinstance(systems, dict):
        systems = {}
    for system in args.require_system:
        if not _positive_count(systems, system):
            failures.append(f"required system {system} is absent")

    row_types = summary.get("row_types", {})
    if not isinstance(row_types, dict):
        row_types = {}
    for row_type in args.require_row_type:
        if not _positive_count(row_types, row_type):
            failures.append(f"required row type {row_type} is absent")

    iterations = summary.get("iterations", {})
    if not isinstance(iterations, dict):
        iterations = {}
    for iteration in args.require_iteration:
        if not _positive_count(iterations, iteration):
            failures.append(f"required iteration {iteration} is absent")

    observation_identity = summary.get("observation_identity", {})
    if not isinstance(observation_identity, dict):
        observation_identity = {}
    primary_codes = observation_identity.get("primary_observation_codes", {})
    if not isinstance(primary_codes, dict):
        primary_codes = {}
    secondary_codes = observation_identity.get("secondary_observation_codes", {})
    if not isinstance(secondary_codes, dict):
        secondary_codes = {}
    frequency_indices = observation_identity.get("frequency_indices", {})
    if not isinstance(frequency_indices, dict):
        frequency_indices = {}
    ionosphere_coefficients = observation_identity.get("ionosphere_coefficients", {})
    if not isinstance(ionosphere_coefficients, dict):
        ionosphere_coefficients = {}
    for code in args.require_primary_observation_code:
        if not _positive_count(primary_codes, code):
            failures.append(f"required primary observation code {code} is absent")
    for code in args.require_secondary_observation_code:
        if not _positive_count(secondary_codes, code):
            failures.append(f"required secondary observation code {code} is absent")
    for frequency_index in args.require_frequency_index:
        if not _positive_count(frequency_indices, frequency_index):
            failures.append(f"required frequency index {frequency_index} is absent")
    for coefficient in args.require_ionosphere_coefficient:
        if not _positive_count(ionosphere_coefficients, coefficient):
            failures.append(f"required ionosphere coefficient {coefficient} is absent")

    component_presence = summary.get("component_presence", {})
    if not isinstance(component_presence, dict):
        component_presence = {}
    rows_with_component = component_presence.get("rows_with_component", {})
    if not isinstance(rows_with_component, dict):
        rows_with_component = {}
    for component in args.require_component:
        if not _positive_count(rows_with_component, component):
            failures.append(f"required component {component} is absent")

    row_key = summary.get("row_key", {})
    if not isinstance(row_key, dict):
        row_key = {}
    if args.require_no_duplicate_keys and int(row_key.get("duplicate_groups", 0)) != 0:
        failures.append(f"duplicate row keys {row_key.get('duplicate_groups')} != 0")

    issue_counts = summary.get("issue_counts", {})
    if args.fail_on_issue and isinstance(issue_counts, dict) and issue_counts:
        failures.append(f"snapshot issues present: {issue_counts}")
    return failures


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("snapshot_csv", type=Path)
    parser.add_argument("--json-out", type=Path)
    parser.add_argument("--require-rows-min", type=int, default=0)
    parser.add_argument("--require-system", action="append", default=[])
    parser.add_argument("--require-row-type", action="append", default=[])
    parser.add_argument("--require-iteration", action="append", default=[])
    parser.add_argument("--require-primary-observation-code", action="append", default=[])
    parser.add_argument("--require-secondary-observation-code", action="append", default=[])
    parser.add_argument("--require-frequency-index", action="append", default=[])
    parser.add_argument("--require-ionosphere-coefficient", action="append", default=[])
    parser.add_argument("--require-component", action="append", default=[])
    parser.add_argument("--require-no-duplicate-keys", action="store_true")
    parser.add_argument("--fail-on-issue", action="store_true")
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    try:
        rows = residual_diff.read_csv_rows(args.snapshot_csv)
        summary = summarize_rows(rows)
        failures = requirement_failures(summary, args)
        if failures:
            summary = {**summary, "status": "failed", "failures": failures}
        else:
            summary = {**summary, "failures": []}
    except Exception as exc:
        summary = {
            "schema": SCHEMA,
            "status": "failed",
            "failures": [str(exc)],
        }

    if args.json_out is not None:
        args.json_out.parent.mkdir(parents=True, exist_ok=True)
        args.json_out.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")

    print("madoca_residual_component_summary:")
    print(f"  status: {summary.get('status')}")
    if "rows" in summary:
        print(f"  rows: {summary['rows']}")
    if summary.get("failures"):
        print("  failures:")
        for failure in summary["failures"]:
            print(f"    - {failure}")
    return 0 if summary.get("status") == "passed" else 1


if __name__ == "__main__":
    raise SystemExit(main())
