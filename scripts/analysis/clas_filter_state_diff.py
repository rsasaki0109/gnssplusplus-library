#!/usr/bin/env python3
"""Compare versioned CLASLIB/native DD-filter state rows."""

from __future__ import annotations

import argparse
import csv
import json
import math
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Iterable, Sequence


SCHEMA = "clas_dd_filter_state_diff.v1"
ROW_SCHEMA = "clas_dd_filter_state.v1"
VALUE_FIELDS = ("float_value", "fixed_value", "variance", "process_noise")


@dataclass(frozen=True, order=True)
class StateKey:
    week: int
    tow: float
    state_kind: str
    state_key: str


@dataclass(frozen=True)
class StateRow:
    key: StateKey
    solution_state: str
    state_index: int
    values: dict[str, float]


def _number(text: str, path: Path, line: int, field: str) -> float:
    try:
        return float(text)
    except ValueError as exc:
        raise ValueError(f"{path}:{line}: invalid {field}: {text!r}") from exc


def load_rows(path: Path) -> tuple[dict[StateKey, StateRow], list[StateKey]]:
    rows: dict[StateKey, StateRow] = {}
    duplicates: list[StateKey] = []
    with path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        required = {
            "schema", "week", "tow", "solution_state", "state_kind",
            "state_key", "state_index", *VALUE_FIELDS,
        }
        missing_columns = sorted(required - set(reader.fieldnames or ()))
        if missing_columns:
            raise ValueError(f"{path}: missing columns: {', '.join(missing_columns)}")
        for line, raw in enumerate(reader, 2):
            if raw["schema"].strip() != ROW_SCHEMA:
                raise ValueError(f"{path}:{line}: unsupported schema {raw['schema']!r}")
            kind = raw["state_kind"].strip().lower()
            if kind not in {"position", "ionosphere"}:
                raise ValueError(f"{path}:{line}: unsupported state_kind {kind!r}")
            solution_state = raw["solution_state"].strip().lower()
            if solution_state not in {"fix", "float"}:
                raise ValueError(f"{path}:{line}: unsupported solution_state {solution_state!r}")
            key = StateKey(
                int(raw["week"]),
                round(float(raw["tow"]), 9),
                kind,
                raw["state_key"].strip(),
            )
            row = StateRow(
                key,
                solution_state,
                int(raw["state_index"]),
                {field: _number(raw[field], path, line, field) for field in VALUE_FIELDS},
            )
            if key in rows:
                duplicates.append(key)
            else:
                rows[key] = row
    return rows, duplicates


def rms(values: Iterable[float]) -> float | None:
    data = list(values)
    return math.sqrt(sum(value * value for value in data) / len(data)) if data else None


def compare(
    base_path: Path,
    candidate_path: Path,
    *,
    start_week: int | None = None,
    start_tow: float | None = None,
    position_rms_threshold: float = 0.02,
    ionosphere_rms_threshold: float = 0.005,
    require_complete_surface: bool = False,
) -> tuple[dict[str, object], list[dict[str, object]]]:
    base, base_duplicates = load_rows(base_path)
    candidate, candidate_duplicates = load_rows(candidate_path)
    if start_tow is not None:
        def in_window(key: StateKey) -> bool:
            return (
                (key.week, key.tow) >= (start_week, start_tow)
                if start_week is not None
                else key.tow >= start_tow
            )
        base = {key: row for key, row in base.items() if in_window(key)}
        candidate = {key: row for key, row in candidate.items() if in_window(key)}
        base_duplicates = [key for key in base_duplicates if in_window(key)]
        candidate_duplicates = [key for key in candidate_duplicates if in_window(key)]

    common = sorted(base.keys() & candidate.keys())
    base_only = sorted(base.keys() - candidate.keys())
    candidate_only = sorted(candidate.keys() - base.keys())
    grouped: dict[tuple[str, str], list[float]] = {}
    fixed_presence_mismatches = 0
    solution_state_mismatches = 0
    details: list[dict[str, object]] = []
    for key in common:
        base_row = base[key]
        candidate_row = candidate[key]
        if base_row.solution_state != candidate_row.solution_state:
            solution_state_mismatches += 1
        detail: dict[str, object] = {
            **asdict(key),
            "base_solution_state": base_row.solution_state,
            "candidate_solution_state": candidate_row.solution_state,
            "base_state_index": base_row.state_index,
            "candidate_state_index": candidate_row.state_index,
        }
        for field in VALUE_FIELDS:
            base_value = base_row.values[field]
            candidate_value = candidate_row.values[field]
            both_finite = math.isfinite(base_value) and math.isfinite(candidate_value)
            one_finite = math.isfinite(base_value) != math.isfinite(candidate_value)
            if field == "fixed_value" and one_finite:
                fixed_presence_mismatches += 1
            delta = candidate_value - base_value if both_finite else math.nan
            if both_finite:
                grouped.setdefault((key.state_kind, field), []).append(delta)
            detail[f"base_{field}"] = base_value
            detail[f"candidate_{field}"] = candidate_value
            detail[f"delta_{field}"] = delta
        details.append(detail)

    metrics: dict[str, dict[str, object]] = {}
    for (kind, field), values in sorted(grouped.items()):
        metrics[f"{kind}.{field}"] = {
            "rows": len(values),
            "rms_delta": rms(values),
            "mean_abs_delta": sum(abs(value) for value in values) / len(values),
            "max_abs_delta": max(abs(value) for value in values),
        }

    failed_gates: list[str] = []
    position_metric = metrics.get("position.float_value")
    ionosphere_metric = metrics.get("ionosphere.float_value")
    if not position_metric or float(position_metric["rms_delta"]) > position_rms_threshold:
        failed_gates.append("position_float_rms")
    if not ionosphere_metric or float(ionosphere_metric["rms_delta"]) > ionosphere_rms_threshold:
        failed_gates.append("ionosphere_float_rms")
    if solution_state_mismatches:
        failed_gates.append("solution_state")
    if fixed_presence_mismatches:
        failed_gates.append("fixed_presence")
    if base_duplicates or candidate_duplicates:
        failed_gates.append("duplicate_keys")
    if require_complete_surface and (base_only or candidate_only):
        failed_gates.append("row_surface")

    report: dict[str, object] = {
        "schema": SCHEMA,
        "status": "passed" if not failed_gates else "failed",
        "base_path": str(base_path),
        "candidate_path": str(candidate_path),
        "start_week": start_week,
        "start_tow": start_tow,
        "base_rows": len(base),
        "candidate_rows": len(candidate),
        "common_rows": len(common),
        "base_only_rows": len(base_only),
        "candidate_only_rows": len(candidate_only),
        "coverage_pct": 100.0 * len(common) / max(len(base), len(candidate), 1),
        "base_duplicate_keys": len(base_duplicates),
        "candidate_duplicate_keys": len(candidate_duplicates),
        "solution_state_mismatches": solution_state_mismatches,
        "fixed_presence_mismatches": fixed_presence_mismatches,
        "thresholds": {
            "position_float_rms": position_rms_threshold,
            "ionosphere_float_rms": ionosphere_rms_threshold,
            "complete_surface_required": require_complete_surface,
        },
        "metrics": metrics,
        "failed_gates": failed_gates,
        "base_only_examples": [asdict(key) for key in base_only[:20]],
        "candidate_only_examples": [asdict(key) for key in candidate_only[:20]],
    }
    return report, details


def write_details(path: Path, details: list[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fields = list(details[0]) if details else [
        "week", "tow", "state_kind", "state_key",
    ]
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(details)


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("base", type=Path)
    parser.add_argument("candidate", type=Path)
    parser.add_argument("--start-week", type=int)
    parser.add_argument("--start-tow", type=float)
    parser.add_argument("--position-rms-threshold", type=float, default=0.02)
    parser.add_argument("--ionosphere-rms-threshold", type=float, default=0.005)
    parser.add_argument("--require-complete-surface", action="store_true")
    parser.add_argument("--json-out", type=Path)
    parser.add_argument("--details-csv", type=Path)
    parser.add_argument("--fail-on-gate", action="store_true")
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    report, details = compare(
        args.base,
        args.candidate,
        start_week=args.start_week,
        start_tow=args.start_tow,
        position_rms_threshold=args.position_rms_threshold,
        ionosphere_rms_threshold=args.ionosphere_rms_threshold,
        require_complete_surface=args.require_complete_surface,
    )
    if args.json_out:
        args.json_out.parent.mkdir(parents=True, exist_ok=True)
        args.json_out.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    if args.details_csv:
        write_details(args.details_csv, details)
    print(json.dumps(report, indent=2, sort_keys=True))
    return 1 if args.fail_on_gate and report["status"] != "passed" else 0


if __name__ == "__main__":
    raise SystemExit(main())
