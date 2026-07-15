#!/usr/bin/env python3
"""Compare versioned CLAS DD measurement rows with complete key coverage."""

from __future__ import annotations

import argparse
import csv
import json
import math
from dataclasses import astuple, dataclass
from pathlib import Path
from typing import Iterable, Sequence


SCHEMA = "clas_dd_measurement_diff.v1"
ROW_SCHEMA = "clas_dd_measurement.v3"
KEY_FIELDS = (
    "week",
    "tow",
    "stage",
    "system_group",
    "frequency_index",
    "measurement_type",
    "reference_satellite",
    "target_satellite",
)


@dataclass(frozen=True, order=True)
class RowKey:
    week: int
    tow: float
    stage: str
    system_group: int
    frequency_index: int
    measurement_type: str
    reference_satellite: str
    target_satellite: str


@dataclass(frozen=True)
class MeasurementRow:
    key: RowKey
    residual_m: float
    linearization_position_ecef: tuple[float, float, float]
    position_coefficients: tuple[float, float, float]


def parse_row(raw: dict[str, str], path: Path, line: int, value_column: str) -> MeasurementRow:
    vector_fields = (
        "linearization_x_m", "linearization_y_m", "linearization_z_m",
        "position_coeff_x", "position_coeff_y", "position_coeff_z",
    )
    missing = [field for field in (*KEY_FIELDS, "schema", value_column, *vector_fields) if not raw.get(field, "").strip()]
    if missing:
        raise ValueError(f"{path}:{line}: missing fields: {', '.join(missing)}")
    if raw["schema"].strip() != ROW_SCHEMA:
        raise ValueError(f"{path}:{line}: unsupported schema {raw['schema']!r}")
    measurement_type = raw["measurement_type"].strip().lower()
    if measurement_type not in {"phase", "code"}:
        raise ValueError(f"{path}:{line}: measurement_type must be phase or code")
    residual = float(raw[value_column])
    if not math.isfinite(residual):
        raise ValueError(f"{path}:{line}: {value_column} is not finite")
    position = tuple(float(raw[field]) for field in vector_fields[:3])
    coefficients = tuple(float(raw[field]) for field in vector_fields[3:])
    if not all(math.isfinite(value) for value in (*position, *coefficients)):
        raise ValueError(f"{path}:{line}: position normalization fields are not finite")
    return MeasurementRow(
        RowKey(
            int(raw["week"]),
            round(float(raw["tow"]), 9),
            raw["stage"].strip(),
            int(raw["system_group"]),
            int(raw["frequency_index"]),
            measurement_type,
            raw["reference_satellite"].strip(),
            raw["target_satellite"].strip(),
        ),
        residual,
        position,  # type: ignore[arg-type]
        coefficients,  # type: ignore[arg-type]
    )


def load_rows(path: Path, value_column: str) -> tuple[dict[RowKey, MeasurementRow], list[RowKey]]:
    rows: dict[RowKey, MeasurementRow] = {}
    duplicates: list[RowKey] = []
    with path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        for line, raw in enumerate(reader, 2):
            row = parse_row(raw, path, line, value_column)
            if row.key in rows:
                duplicates.append(row.key)
            else:
                rows[row.key] = row
    return rows, duplicates


def rms(values: Iterable[float]) -> float | None:
    values = list(values)
    return math.sqrt(sum(value * value for value in values) / len(values)) if values else None


def key_json(key: RowKey) -> dict[str, object]:
    return dict(zip(KEY_FIELDS, astuple(key), strict=True))


def compare(
    base_path: Path,
    candidate_path: Path,
    phase_threshold_m: float,
    code_threshold_m: float,
    value_column: str = "residual_m",
    normalize_candidate_position_to_base: bool = False,
    start_tow: float | None = None,
) -> tuple[dict[str, object], list[dict[str, object]]]:
    base, base_duplicates = load_rows(base_path, value_column)
    candidate, candidate_duplicates = load_rows(candidate_path, value_column)
    if start_tow is not None:
        base = {key: row for key, row in base.items() if key.tow >= start_tow}
        candidate = {key: row for key, row in candidate.items() if key.tow >= start_tow}
        base_duplicates = [key for key in base_duplicates if key.tow >= start_tow]
        candidate_duplicates = [key for key in candidate_duplicates if key.tow >= start_tow]
    common = sorted(base.keys() & candidate.keys())
    base_only = sorted(base.keys() - candidate.keys())
    candidate_only = sorted(candidate.keys() - base.keys())
    details: list[dict[str, object]] = []
    deltas: dict[str, list[float]] = {"phase": [], "code": []}
    for key in common:
        candidate_value = candidate[key].residual_m
        if normalize_candidate_position_to_base:
            displacement = (
                base[key].linearization_position_ecef[0] - candidate[key].linearization_position_ecef[0],
                base[key].linearization_position_ecef[1] - candidate[key].linearization_position_ecef[1],
                base[key].linearization_position_ecef[2] - candidate[key].linearization_position_ecef[2],
            )
            candidate_value -= sum(
                coefficient * delta_position
                for coefficient, delta_position in zip(
                    candidate[key].position_coefficients, displacement, strict=True
                )
            )
        delta = candidate_value - base[key].residual_m
        deltas[key.measurement_type].append(delta)
        details.append(
            {
                **key_json(key),
                "base_value_m": base[key].residual_m,
                "candidate_value_m": candidate_value,
                "delta_m": delta,
            }
        )
    metrics: dict[str, dict[str, object]] = {}
    grouped_values: dict[tuple[int, int, str], list[float]] = {}
    for detail in details:
        group_key = (
            int(detail["system_group"]),
            int(detail["frequency_index"]),
            str(detail["measurement_type"]),
        )
        grouped_values.setdefault(group_key, []).append(float(detail["delta_m"]))
    gates: list[str] = []
    for kind, threshold in (("phase", phase_threshold_m), ("code", code_threshold_m)):
        values = deltas[kind]
        kind_rms = rms(values)
        metrics[kind] = {
            "common_rows": len(values),
            "rms_delta_m": kind_rms,
            "max_abs_delta_m": max((abs(value) for value in values), default=None),
            "threshold_m": threshold,
        }
        if kind_rms is None or kind_rms > threshold:
            gates.append(f"{kind}_rms")
    if base_only or candidate_only:
        gates.append("row_surface")
    if base_duplicates or candidate_duplicates:
        gates.append("duplicate_keys")
    per_group = [
        {
            "system_group": group,
            "frequency_index": frequency,
            "measurement_type": kind,
            "common_rows": len(values),
            "rms_delta_m": rms(values),
            "max_abs_delta_m": max(abs(value) for value in values),
        }
        for (group, frequency, kind), values in sorted(grouped_values.items())
    ]
    report: dict[str, object] = {
        "schema": SCHEMA,
        "status": "passed" if not gates else "failed",
        "base_path": str(base_path),
        "candidate_path": str(candidate_path),
        "value_column": value_column,
        "candidate_position_normalized_to_base": normalize_candidate_position_to_base,
        "start_tow": start_tow,
        "base_rows": len(base),
        "candidate_rows": len(candidate),
        "common_rows": len(common),
        "base_only_rows": len(base_only),
        "candidate_only_rows": len(candidate_only),
        "base_duplicate_keys": len(base_duplicates),
        "candidate_duplicate_keys": len(candidate_duplicates),
        "coverage_pct": 100.0 * len(common) / max(len(base), len(candidate), 1),
        "metrics": metrics,
        "per_group": per_group,
        "failed_gates": gates,
        "base_only_examples": [key_json(key) for key in base_only[:20]],
        "candidate_only_examples": [key_json(key) for key in candidate_only[:20]],
    }
    return report, details


def write_details(path: Path, details: list[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fields = (*KEY_FIELDS, "base_value_m", "candidate_value_m", "delta_m")
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(details)


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("base", type=Path)
    parser.add_argument("candidate", type=Path)
    parser.add_argument("--phase-rms-threshold-m", type=float, default=0.005)
    parser.add_argument("--code-rms-threshold-m", type=float, default=0.020)
    parser.add_argument("--value-column", choices=("raw_dd_m", "residual_m"), default="residual_m")
    parser.add_argument("--normalize-candidate-position-to-base", action="store_true")
    parser.add_argument("--start-tow", type=float)
    parser.add_argument("--json-out", type=Path)
    parser.add_argument("--details-csv", type=Path)
    parser.add_argument("--fail-on-gate", action="store_true")
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    report, details = compare(
        args.base,
        args.candidate,
        args.phase_rms_threshold_m,
        args.code_rms_threshold_m,
        args.value_column,
        args.normalize_candidate_position_to_base,
        args.start_tow,
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
