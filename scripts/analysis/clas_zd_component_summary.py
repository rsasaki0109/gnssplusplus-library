#!/usr/bin/env python3
"""Summarize and validate a CLAS zero-difference component CSV dump."""

from __future__ import annotations

import argparse
from collections import Counter
import json
from pathlib import Path
import sys
from typing import Any, Mapping, Sequence


sys.path.insert(0, str(Path(__file__).resolve().parent))

import clas_zd_component_diff as zd_diff  # noqa: E402


SCHEMA = "clas_zd_component_summary.v1"


def _format_counter(counter: Counter[str]) -> dict[str, int]:
    return {key: counter[key] for key in sorted(counter)}


def _format_top(counter: Counter[str], limit: int) -> list[dict[str, object]]:
    return [
        {"key": key, "count": count}
        for key, count in sorted(counter.items(), key=lambda item: (-item[1], item[0]))[:limit]
    ]


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


def _parse_week(value: str) -> int:
    return int(float(value))


def _row_key(row: Mapping[str, str]) -> zd_diff.Key:
    row_type = zd_diff.detect_row_type(dict(row))
    return (
        _parse_week(row.get("week", "")),
        zd_diff.tow_to_millis(row.get("tow", "")),
        row_type,
        row.get("sat", ""),
        zd_diff.detect_frequency(dict(row)),
        zd_diff.observation_identity(dict(row), row_type),
    )


def _component_true(row: Mapping[str, str], component: str) -> bool:
    components = zd_diff.extract_components(dict(row), [component])
    value = components.get(component)
    return value is not None and value > 0.5


def _has_component(row: Mapping[str, str], component: str) -> bool:
    return component in zd_diff.extract_components(dict(row), [component])


def summarize_rows(rows: Sequence[dict[str, str]], *, top_limit: int = 20) -> dict[str, object]:
    systems: Counter[str] = Counter()
    satellites: Counter[str] = Counter()
    stages: Counter[str] = Counter()
    row_types: Counter[str] = Counter()
    frequencies: Counter[str] = Counter()
    rinex_codes: Counter[str] = Counter()
    component_rows: Counter[str] = Counter()
    key_counts: Counter[zd_diff.Key] = Counter()
    issue_counts: Counter[str] = Counter()
    issue_examples: dict[str, list[dict[str, object]]] = {}
    time_keys: list[tuple[int, int]] = []

    identity_counts: dict[str, Counter[str]] = {
        component: Counter() for component in zd_diff.IDENTITY_PROVENANCE_COMPONENTS
    }
    gps_l2w_identity_counts: dict[str, Counter[str]] = {
        component: Counter() for component in zd_diff.IDENTITY_PROVENANCE_COMPONENTS
    }

    component_names = sorted(zd_diff.COMPONENT_ALIASES)

    def record_issue(kind: str, row_number: int, detail: str) -> None:
        issue_counts[kind] += 1
        examples = issue_examples.setdefault(kind, [])
        if len(examples) < 5:
            examples.append({"row": row_number, "detail": detail})

    for row_number, row in enumerate(rows, start=2):
        sat = row.get("sat", "")
        stage = row.get("stage", "")
        row_type = zd_diff.detect_row_type(row)
        freq = zd_diff.detect_frequency(row)
        signal = zd_diff.observation_identity(row, row_type)

        if not sat:
            record_issue("missing_satellite", row_number, "")
        if not row_type:
            record_issue("missing_row_type", row_number, "")
        if not signal:
            record_issue("missing_observation_identity", row_number, "")

        systems[_sat_system(sat)] += 1
        satellites[sat] += 1
        stages[stage] += 1
        row_types[row_type] += 1
        frequencies[str(freq)] += 1
        rinex_codes[signal] += 1

        try:
            key = _row_key(row)
            key_counts[key] += 1
            time_keys.append((key[0], key[1]))
        except (ValueError, TypeError) as exc:
            record_issue("bad_row_key", row_number, str(exc))

        components = zd_diff.extract_components(row, component_names)
        if not components:
            record_issue("missing_numeric_component", row_number, "")
        for component in components:
            component_rows[component] += 1

        gps_l2w = sat.startswith("G") and signal in zd_diff.GPS_L2W_RINEX_CODES
        for component in zd_diff.IDENTITY_PROVENANCE_COMPONENTS:
            if _component_true(row, component):
                identity_counts[component]["true_rows"] += 1
                if gps_l2w:
                    gps_l2w_identity_counts[component]["true_rows"] += 1

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

    gps_l2w_rows = sum(
        1
        for row in rows
        if row.get("sat", "").startswith("G")
        and zd_diff.observation_identity(row, zd_diff.detect_row_type(row)) in zd_diff.GPS_L2W_RINEX_CODES
    )

    return {
        "schema": SCHEMA,
        "status": "failed" if issue_counts else "passed",
        "rows": len(rows),
        "unique_satellites": len(satellites),
        "systems": _format_counter(systems),
        "top_satellites": _format_top(satellites, top_limit),
        "time_range": time_range,
        "stages": _format_counter(stages),
        "row_types": _format_counter(row_types),
        "observation_identity": {
            "rinex_codes": _format_counter(rinex_codes),
            "frequency_indices": _format_counter(frequencies),
        },
        "component_presence": {
            "rows_with_component": _format_counter(component_rows),
            "known_components": component_names,
        },
        "bias_identity": {
            "rows_with_code_bias": sum(1 for row in rows if _has_component(row, "code_bias_m")),
            "rows_with_phase_bias": sum(1 for row in rows if _has_component(row, "phase_bias_m")),
        },
        "identity_provenance": {
            "gps_l2w_rows": gps_l2w_rows,
            "components": {
                component: {
                    "true_rows": identity_counts[component]["true_rows"],
                    "gps_l2w_true_rows": gps_l2w_identity_counts[component]["true_rows"],
                }
                for component in zd_diff.IDENTITY_PROVENANCE_COMPONENTS
            },
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
        if int(systems.get(str(system), 0)) <= 0:
            failures.append(f"required system {system} is absent")

    stages = summary.get("stages", {})
    if not isinstance(stages, dict):
        stages = {}
    for stage in args.require_stage:
        if int(stages.get(str(stage), 0)) <= 0:
            failures.append(f"required stage {stage} is absent")

    row_types = summary.get("row_types", {})
    if not isinstance(row_types, dict):
        row_types = {}
    for row_type in args.require_row_type:
        if int(row_types.get(str(row_type), 0)) <= 0:
            failures.append(f"required row type {row_type} is absent")

    observation_identity = summary.get("observation_identity", {})
    if not isinstance(observation_identity, dict):
        observation_identity = {}
    rinex_codes = observation_identity.get("rinex_codes", {})
    if not isinstance(rinex_codes, dict):
        rinex_codes = {}
    for rinex_code in args.require_rinex_code:
        if int(rinex_codes.get(str(rinex_code), 0)) <= 0:
            failures.append(f"required RINEX code {rinex_code} is absent")

    component_presence = summary.get("component_presence", {})
    if not isinstance(component_presence, dict):
        component_presence = {}
    rows_with_component = component_presence.get("rows_with_component", {})
    if not isinstance(rows_with_component, dict):
        rows_with_component = {}
    for component in args.require_component:
        if int(rows_with_component.get(str(component), 0)) <= 0:
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
    parser.add_argument("--require-stage", action="append", default=[])
    parser.add_argument("--require-row-type", action="append", default=[])
    parser.add_argument("--require-rinex-code", action="append", default=[])
    parser.add_argument("--require-component", action="append", default=[])
    parser.add_argument("--require-no-duplicate-keys", action="store_true")
    parser.add_argument("--fail-on-issue", action="store_true")
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    try:
        rows = zd_diff.read_csv_rows(args.snapshot_csv)
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

    print("clas_zd_component_summary:")
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
