#!/usr/bin/env python3
"""Summarize final SPP residual dumps by satellite and signal."""

from __future__ import annotations

import argparse
import csv
import json
import math
from collections import defaultdict
from pathlib import Path
from statistics import median
from typing import Any, DefaultDict, Dict, List, Optional, Sequence, Tuple


REQUIRED_COLUMNS = [
    "week",
    "tow",
    "sat",
    "signal",
    "residual_m",
    "residual_rms_m",
    "corrected_pseudorange_m",
    "sat_clock_correction_m",
    "troposphere_delay_m",
    "ionosphere_delay_m",
    "group_delay_m",
    "ephemeris_variance_m2",
    "variance_m2",
    "elevation_deg",
    "weight",
]

Row = Dict[str, str]
GroupKey = Tuple[str, str]


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


def mean(values: Sequence[float]) -> float:
    vals = finite(values)
    return sum(vals) / len(vals) if vals else math.nan


def rms(values: Sequence[float]) -> float:
    vals = finite(values)
    return math.sqrt(sum(value * value for value in vals) / len(vals)) if vals else math.nan


def group_summary(key: GroupKey, rows: Sequence[Row]) -> Dict[str, Any]:
    residuals = [to_float(row["residual_m"]) for row in rows]
    abs_residuals = [abs(value) for value in residuals if math.isfinite(value)]
    iono = [to_float(row["ionosphere_delay_m"]) for row in rows]
    group_delay = [to_float(row["group_delay_m"]) for row in rows]
    trop = [to_float(row["troposphere_delay_m"]) for row in rows]
    weight = [to_float(row["weight"]) for row in rows]
    variance = [to_float(row["variance_m2"]) for row in rows]
    elevation = [to_float(row["elevation_deg"]) for row in rows]
    rms_values = [to_float(row["residual_rms_m"]) for row in rows]
    return {
        "sat": key[0],
        "signal": key[1],
        "rows": len(rows),
        "mean_residual_m": mean(residuals),
        "median_residual_m": median(finite(residuals)) if finite(residuals) else math.nan,
        "rms_residual_m": rms(residuals),
        "median_abs_residual_m": median(abs_residuals) if abs_residuals else math.nan,
        "p95_abs_residual_m": percentile(abs_residuals, 95.0),
        "max_abs_residual_m": max(abs_residuals) if abs_residuals else math.nan,
        "mean_iono_delay_m": mean(iono),
        "mean_group_delay_m": mean(group_delay),
        "mean_trop_delay_m": mean(trop),
        "mean_weight": mean(weight),
        "mean_variance_m2": mean(variance),
        "min_elevation_deg": min(finite(elevation)) if finite(elevation) else math.nan,
        "mean_residual_rms_m": mean(rms_values),
    }


def summarize(rows: Sequence[Row], *, top: int = 20) -> Dict[str, Any]:
    grouped: DefaultDict[GroupKey, List[Row]] = defaultdict(list)
    top_rows: List[Dict[str, Any]] = []
    for row in rows:
        key = (row.get("sat", ""), row.get("signal", ""))
        grouped[key].append(row)
        residual = to_float(row["residual_m"])
        if math.isfinite(residual):
            top_rows.append(
                {
                    "week": int(float(row["week"])),
                    "tow": to_float(row["tow"]),
                    "sat": key[0],
                    "signal": key[1],
                    "residual_m": residual,
                    "abs_residual_m": abs(residual),
                    "residual_rms_m": to_float(row["residual_rms_m"]),
                    "ionosphere_delay_m": to_float(row["ionosphere_delay_m"]),
                    "group_delay_m": to_float(row["group_delay_m"]),
                    "troposphere_delay_m": to_float(row["troposphere_delay_m"]),
                    "weight": to_float(row["weight"]),
                    "variance_m2": to_float(row["variance_m2"]),
                    "elevation_deg": to_float(row["elevation_deg"]),
                }
            )

    groups = [group_summary(key, group_rows) for key, group_rows in grouped.items()]
    groups.sort(
        key=lambda item: (
            -item["p95_abs_residual_m"],
            -item["max_abs_residual_m"],
            item["sat"],
            item["signal"],
        )
    )
    top_rows.sort(key=lambda item: item["abs_residual_m"], reverse=True)
    all_residuals = [to_float(row["residual_m"]) for row in rows]
    return {
        "rows": len(rows),
        "satellite_signal_groups": len(groups),
        "overall_rms_residual_m": rms(all_residuals),
        "overall_median_abs_residual_m": median([abs(value) for value in finite(all_residuals)])
        if finite(all_residuals) else math.nan,
        "groups": groups[:top],
        "top_residual_rows": top_rows[:top],
    }


def write_groups_csv(path: Path, groups: Sequence[Dict[str, Any]]) -> None:
    fieldnames = [
        "sat",
        "signal",
        "rows",
        "mean_residual_m",
        "median_residual_m",
        "rms_residual_m",
        "median_abs_residual_m",
        "p95_abs_residual_m",
        "max_abs_residual_m",
        "mean_iono_delay_m",
        "mean_group_delay_m",
        "mean_trop_delay_m",
        "mean_weight",
        "mean_variance_m2",
        "min_elevation_deg",
        "mean_residual_rms_m",
    ]
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(groups)


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("dump_csv", type=Path)
    parser.add_argument("--top", type=int, default=20)
    parser.add_argument("--json-out", type=Path)
    parser.add_argument("--groups-csv", type=Path)
    return parser.parse_args(argv)


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)
    report = summarize(read_csv(args.dump_csv), top=args.top)
    if args.json_out is not None:
        args.json_out.parent.mkdir(parents=True, exist_ok=True)
        args.json_out.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    if args.groups_csv is not None:
        write_groups_csv(args.groups_csv, report["groups"])
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
