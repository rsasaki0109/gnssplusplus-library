#!/usr/bin/env python3
"""Summarize and validate a MADOCA materialization snapshot CSV."""

from __future__ import annotations

import argparse
from collections import Counter
import json
from pathlib import Path
import sys
from typing import Any, Mapping, Sequence


sys.path.insert(0, str(Path(__file__).resolve().parent))

import madoca_materialization_diff as materialization_diff  # noqa: E402


SCHEMA = "madoca_materialization_summary.v1"


def _int_value(value: str) -> int | None:
    try:
        return int(float(value))
    except (TypeError, ValueError):
        return None


def _format_counter(counter: Counter[str]) -> dict[str, int]:
    return {key: counter[key] for key in sorted(counter)}


def _format_top(counter: Counter[str], limit: int) -> list[dict[str, object]]:
    return [
        {"key": key, "count": count}
        for key, count in sorted(counter.items(), key=lambda item: (-item[1], item[0]))[:limit]
    ]


def _row_key(row: Mapping[str, str]) -> tuple[str, int, int]:
    return (
        row.get("sat", ""),
        int(float(row.get("week", "0"))),
        materialization_diff.tow_to_millis(row.get("tow", "0")),
    )


def summarize_rows(rows: Sequence[dict[str, str]], *, top_limit: int = 20) -> dict[str, object]:
    systems: Counter[str] = Counter()
    sats: Counter[str] = Counter()
    key_counts: Counter[tuple[str, int, int]] = Counter()
    validity: dict[str, Counter[str]] = {
        "orbit_valid": Counter(),
        "clock_valid": Counter(),
        "code_bias_valid": Counter(),
        "phase_bias_valid": Counter(),
    }
    code_bias_ids: Counter[str] = Counter()
    phase_bias_ids: Counter[str] = Counter()
    phase_discnt_ids: Counter[str] = Counter()
    issue_counts: Counter[str] = Counter()
    issue_examples: dict[str, list[dict[str, object]]] = {}
    time_keys: list[tuple[int, int]] = []

    def record_issue(kind: str, row_number: int, detail: str) -> None:
        issue_counts[kind] += 1
        examples = issue_examples.setdefault(kind, [])
        if len(examples) < 5:
            examples.append({"row": row_number, "detail": detail})

    for row_number, row in enumerate(rows, start=2):
        systems[row.get("system", "")] += 1
        sats[row.get("sat", "")] += 1
        try:
            key = _row_key(row)
            key_counts[key] += 1
            time_keys.append((key[1], key[2]))
        except (ValueError, TypeError) as exc:
            record_issue("bad_time_key", row_number, str(exc))

        for field, counter in validity.items():
            counter[materialization_diff.to_int_text(row.get(field, ""))] += 1

        parsed_code_biases = materialization_diff.parse_numeric_map(row.get("code_biases_m", ""))
        parsed_phase_biases = materialization_diff.parse_numeric_map(row.get("phase_biases_m", ""))
        parsed_discnt = materialization_diff.parse_discrete_map(row.get("phase_bias_discnt", ""))
        code_bias_ids.update(parsed_code_biases.keys())
        phase_bias_ids.update(parsed_phase_biases.keys())
        phase_discnt_ids.update(parsed_discnt.keys())

        declared_code_count = _int_value(row.get("code_bias_count", ""))
        if declared_code_count is None:
            record_issue("bad_code_bias_count", row_number, row.get("code_bias_count", ""))
        elif declared_code_count != len(parsed_code_biases):
            record_issue(
                "code_bias_count_mismatch",
                row_number,
                f"declared={declared_code_count} parsed={len(parsed_code_biases)}",
            )

        declared_phase_count = _int_value(row.get("phase_bias_count", ""))
        if declared_phase_count is None:
            record_issue("bad_phase_bias_count", row_number, row.get("phase_bias_count", ""))
        elif declared_phase_count != len(parsed_phase_biases):
            record_issue(
                "phase_bias_count_mismatch",
                row_number,
                f"declared={declared_phase_count} parsed={len(parsed_phase_biases)}",
            )

        phase_only_discnt_ids = sorted(set(parsed_discnt) - set(parsed_phase_biases), key=int)
        if phase_only_discnt_ids:
            record_issue(
                "phase_discnt_without_phase_bias",
                row_number,
                ";".join(phase_only_discnt_ids),
            )

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
        "snapshot_schema": materialization_diff.SNAPSHOT_SCHEMA,
        "status": "failed" if issue_counts else "passed",
        "rows": len(rows),
        "unique_satellites": len(sats),
        "systems": _format_counter(systems),
        "top_satellites": _format_top(sats, top_limit),
        "time_range": time_range,
        "validity_counts": {
            field: _format_counter(counter) for field, counter in validity.items()
        },
        "bias_identity": {
            "code_bias_ids": _format_counter(code_bias_ids),
            "phase_bias_ids": _format_counter(phase_bias_ids),
            "phase_discontinuity_ids": _format_counter(phase_discnt_ids),
            "rows_with_code_bias": sum(1 for row in rows if row.get("code_biases_m", "") != ""),
            "rows_with_phase_bias": sum(1 for row in rows if row.get("phase_biases_m", "") != ""),
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
        if int(systems.get(system, 0)) <= 0:
            failures.append(f"required system {system} is absent")

    bias_identity = summary.get("bias_identity", {})
    if not isinstance(bias_identity, dict):
        bias_identity = {}
    code_bias_ids = bias_identity.get("code_bias_ids", {})
    if not isinstance(code_bias_ids, dict):
        code_bias_ids = {}
    phase_bias_ids = bias_identity.get("phase_bias_ids", {})
    if not isinstance(phase_bias_ids, dict):
        phase_bias_ids = {}
    for signal_id in args.require_code_bias_id:
        if int(code_bias_ids.get(str(signal_id), 0)) <= 0:
            failures.append(f"required code bias id {signal_id} is absent")
    for signal_id in args.require_phase_bias_id:
        if int(phase_bias_ids.get(str(signal_id), 0)) <= 0:
            failures.append(f"required phase bias id {signal_id} is absent")

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
    parser.add_argument("--require-code-bias-id", action="append", default=[])
    parser.add_argument("--require-phase-bias-id", action="append", default=[])
    parser.add_argument("--require-no-duplicate-keys", action="store_true")
    parser.add_argument("--fail-on-issue", action="store_true")
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    try:
        rows = materialization_diff.read_csv_rows(args.snapshot_csv)
        summary = summarize_rows(rows)
        failures = requirement_failures(summary, args)
        if failures:
            summary = {**summary, "status": "failed", "failures": failures}
        else:
            summary = {**summary, "failures": []}
    except Exception as exc:
        summary = {
            "schema": SCHEMA,
            "snapshot_schema": materialization_diff.SNAPSHOT_SCHEMA,
            "status": "failed",
            "failures": [str(exc)],
        }

    if args.json_out is not None:
        args.json_out.parent.mkdir(parents=True, exist_ok=True)
        args.json_out.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")

    print("madoca_materialization_summary:")
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
