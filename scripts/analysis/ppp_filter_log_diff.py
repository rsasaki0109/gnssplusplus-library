#!/usr/bin/env python3
"""Compare PPP filter logs by epoch and summarize state/seed deltas."""

from __future__ import annotations

import argparse
import csv
import json
import math
from decimal import Decimal, InvalidOperation, ROUND_HALF_UP
from pathlib import Path
from statistics import median
from typing import Any, Dict, List, Optional, Sequence, Tuple


REQUIRED_COLUMNS = [
    "week",
    "tow",
    "iteration",
    "position_x_m",
    "position_y_m",
    "position_z_m",
    "seed_position_x_m",
    "seed_position_y_m",
    "seed_position_z_m",
    "seed_clock_m",
    "code_residual_rms_m",
    "phase_residual_rms_m",
]

Row = Dict[str, str]
MatchKey = Tuple[int, int, int]


def tow_to_millis(value: str) -> int:
    try:
        tow = Decimal(str(value))
    except InvalidOperation as exc:
        raise ValueError(f"invalid tow value: {value}") from exc
    return int((tow * Decimal("1000")).to_integral_value(rounding=ROUND_HALF_UP))


def to_float(value: str) -> float:
    if value == "":
        return math.nan
    try:
        parsed = float(value)
    except ValueError:
        return math.nan
    return parsed if math.isfinite(parsed) else math.nan


def read_csv(path: Path) -> List[Row]:
    with path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        if reader.fieldnames is None:
            raise ValueError(f"{path}: missing CSV header")
        missing = [column for column in REQUIRED_COLUMNS if column not in reader.fieldnames]
        if missing:
            raise ValueError(f"{path}: missing required columns: {', '.join(missing)}")
        return [dict(row) for row in reader]


def match_key(row: Row) -> MatchKey:
    return (
        int(float(row["week"])),
        tow_to_millis(row["tow"]),
        int(float(row["iteration"])),
    )


def vector3(row: Row, prefix: str) -> Tuple[float, float, float]:
    return (
        to_float(row[f"{prefix}_x_m"]),
        to_float(row[f"{prefix}_y_m"]),
        to_float(row[f"{prefix}_z_m"]),
    )


def norm_delta(a: Tuple[float, float, float], b: Tuple[float, float, float]) -> float:
    values = [(b[i] - a[i]) for i in range(3)]
    if not all(math.isfinite(value) for value in values):
        return math.nan
    return math.sqrt(sum(value * value for value in values))


def finite(values: Sequence[float]) -> List[float]:
    return [value for value in values if math.isfinite(value)]


def percentile(values: Sequence[float], pct: float) -> float:
    vals = sorted(finite(values))
    if not vals:
        return math.nan
    index = (len(vals) - 1) * pct / 100.0
    lower = math.floor(index)
    upper = math.ceil(index)
    if lower == upper:
        return vals[lower]
    weight = index - lower
    return vals[lower] * (1.0 - weight) + vals[upper] * weight


def stats(values: Sequence[float]) -> Dict[str, float]:
    vals = finite(values)
    if not vals:
        return {"median": math.nan, "p95": math.nan, "max": math.nan, "rms": math.nan}
    return {
        "median": median(vals),
        "p95": percentile(vals, 95.0),
        "max": max(vals),
        "rms": math.sqrt(sum(value * value for value in vals) / len(vals)),
    }


def summarize(
    base_rows: Sequence[Row],
    candidate_rows: Sequence[Row],
    *,
    top: int = 20,
) -> Dict[str, Any]:
    base = {match_key(row): row for row in base_rows}
    candidate = {match_key(row): row for row in candidate_rows}
    common_keys = sorted(set(base).intersection(candidate))

    rows: List[Dict[str, Any]] = []
    for key in common_keys:
        base_row = base[key]
        candidate_row = candidate[key]
        seed_delta = norm_delta(
            vector3(base_row, "seed_position"),
            vector3(candidate_row, "seed_position"),
        )
        position_delta = norm_delta(
            vector3(base_row, "position"),
            vector3(candidate_row, "position"),
        )
        seed_clock_delta = to_float(candidate_row["seed_clock_m"]) - to_float(base_row["seed_clock_m"])
        code_rms_delta = (
            to_float(candidate_row["code_residual_rms_m"]) -
            to_float(base_row["code_residual_rms_m"])
        )
        phase_rms_delta = (
            to_float(candidate_row["phase_residual_rms_m"]) -
            to_float(base_row["phase_residual_rms_m"])
        )
        rows.append(
            {
                "week": key[0],
                "tow": key[1] / 1000.0,
                "iteration": key[2],
                "seed_position_delta_3d_m": seed_delta,
                "position_delta_3d_m": position_delta,
                "seed_clock_delta_m": seed_clock_delta,
                "code_residual_rms_delta_m": code_rms_delta,
                "phase_residual_rms_delta_m": phase_rms_delta,
                "base_code_residual_rms_m": to_float(base_row["code_residual_rms_m"]),
                "candidate_code_residual_rms_m": to_float(candidate_row["code_residual_rms_m"]),
                "base_phase_residual_rms_m": to_float(base_row["phase_residual_rms_m"]),
                "candidate_phase_residual_rms_m": to_float(candidate_row["phase_residual_rms_m"]),
            }
        )

    top_position = sorted(
        rows,
        key=lambda row: row["position_delta_3d_m"] if math.isfinite(row["position_delta_3d_m"]) else -1.0,
        reverse=True,
    )[:top]
    top_seed = sorted(
        rows,
        key=lambda row: row["seed_position_delta_3d_m"] if math.isfinite(row["seed_position_delta_3d_m"]) else -1.0,
        reverse=True,
    )[:top]
    return {
        "base_rows": len(base_rows),
        "candidate_rows": len(candidate_rows),
        "matched_rows": len(rows),
        "base_only_rows": len(set(base).difference(candidate)),
        "candidate_only_rows": len(set(candidate).difference(base)),
        "seed_position_delta_3d_m": stats([row["seed_position_delta_3d_m"] for row in rows]),
        "position_delta_3d_m": stats([row["position_delta_3d_m"] for row in rows]),
        "abs_seed_clock_delta_m": stats([abs(row["seed_clock_delta_m"]) for row in rows]),
        "top_position_deltas": top_position,
        "top_seed_deltas": top_seed,
    }


def write_rows_csv(path: Path, rows: Sequence[Dict[str, Any]]) -> None:
    if not rows:
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("base_csv", type=Path)
    parser.add_argument("candidate_csv", type=Path)
    parser.add_argument("--top", type=int, default=20)
    parser.add_argument("--json-out", type=Path)
    parser.add_argument("--top-position-csv", type=Path)
    return parser.parse_args(argv)


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)
    report = summarize(read_csv(args.base_csv), read_csv(args.candidate_csv), top=args.top)
    if args.json_out is not None:
        args.json_out.parent.mkdir(parents=True, exist_ok=True)
        args.json_out.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    if args.top_position_csv is not None:
        write_rows_csv(args.top_position_csv, report["top_position_deltas"])
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
