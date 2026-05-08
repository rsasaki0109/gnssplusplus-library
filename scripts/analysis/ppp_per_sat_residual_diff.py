#!/usr/bin/env python3
"""Compare per-satellite residuals between two PPP residual CSV logs."""

from __future__ import annotations

import argparse
import csv
import json
import math
from collections import Counter, defaultdict
from decimal import Decimal, InvalidOperation, ROUND_HALF_UP
from pathlib import Path
from statistics import median
from typing import Any, DefaultDict, Dict, Iterable, List, Optional, Sequence, Tuple


REQUIRED_COLUMNS = [
    "week",
    "tow",
    "sat",
    "row_type",
    "residual_m",
]

OPTIONAL_DEFAULTS = {
    "iteration": "0",
    "frequency_index": "0",
    "primary_signal": "",
    "secondary_signal": "",
    "primary_observation_code": "",
    "secondary_observation_code": "",
}

Row = Dict[str, str]
MatchKey = Tuple[int, int, int, str, str, int, str, str, str, str]
GroupKey = Tuple[str, int, str]

# Module-level flag toggled by ``--ignore-signals``: when true, the signal /
# observation-code columns are blanked out inside ``match_key`` so logs that
# carry detailed signal metadata (native ``--ppp-residual-log``) match logs
# that don't (CLASLIB bridge ``$SAT`` converter).
_IGNORE_SIGNALS: bool = False


def tow_to_millis(value: str) -> int:
    try:
        tow = Decimal(str(value))
    except InvalidOperation as exc:
        raise ValueError(f"invalid tow value: {value}") from exc
    return int((tow * Decimal("1000")).to_integral_value(rounding=ROUND_HALF_UP))


def to_int(value: str) -> int:
    if value == "":
        return 0
    try:
        return int(float(value))
    except ValueError:
        return 0


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
        rows = [dict(row) for row in reader]
    for row in rows:
        for column, default in OPTIONAL_DEFAULTS.items():
            row.setdefault(column, default)
    return rows


def match_key(row: Row) -> MatchKey:
    if _IGNORE_SIGNALS:
        primary_signal = secondary_signal = ""
        primary_obs_code = secondary_obs_code = ""
    else:
        primary_signal = row.get("primary_signal", "")
        secondary_signal = row.get("secondary_signal", "")
        primary_obs_code = row.get("primary_observation_code", "")
        secondary_obs_code = row.get("secondary_observation_code", "")
    return (
        int(float(row["week"])),
        tow_to_millis(row["tow"]),
        to_int(row.get("iteration", "0")),
        row.get("sat", ""),
        row.get("row_type", ""),
        to_int(row.get("frequency_index", "0")),
        primary_signal,
        secondary_signal,
        primary_obs_code,
        secondary_obs_code,
    )


def group_key(row: Row) -> GroupKey:
    return (
        row.get("sat", ""),
        to_int(row.get("frequency_index", "0")),
        row.get("row_type", ""),
    )


def key_to_dict(key: MatchKey) -> Dict[str, Any]:
    return {
        "week": key[0],
        "tow": key[1] / 1000.0,
        "iteration": key[2],
        "sat": key[3],
        "row_type": key[4],
        "frequency_index": key[5],
        "primary_signal": key[6],
        "secondary_signal": key[7],
        "primary_observation_code": key[8],
        "secondary_observation_code": key[9],
    }


def filter_rows(
    rows: Iterable[Row],
    *,
    row_type: Optional[str],
    iteration: Optional[int],
    tow_min: Optional[float],
    tow_max: Optional[float],
) -> List[Row]:
    filtered: List[Row] = []
    for row in rows:
        if row_type is not None and row.get("row_type", "") != row_type:
            continue
        if iteration is not None and to_int(row.get("iteration", "0")) != iteration:
            continue
        tow = to_float(row["tow"])
        if tow_min is not None and tow < tow_min:
            continue
        if tow_max is not None and tow > tow_max:
            continue
        filtered.append(row)
    return filtered


def index_rows(rows: Sequence[Row]) -> Tuple[Dict[MatchKey, Row], Counter[MatchKey]]:
    indexed: Dict[MatchKey, Row] = {}
    duplicate_counts: Counter[MatchKey] = Counter()
    for row in rows:
        key = match_key(row)
        if key in indexed:
            duplicate_counts[key] += 1
            continue
        indexed[key] = row
    return indexed, duplicate_counts


def finite_stats(values: Sequence[float]) -> Dict[str, float]:
    finite = [value for value in values if math.isfinite(value)]
    if not finite:
        return {
            "mean": math.nan,
            "median": math.nan,
            "stddev": math.nan,
            "rms": math.nan,
            "p95_abs": math.nan,
            "max_abs": math.nan,
        }
    mean = sum(finite) / len(finite)
    variance = sum((value - mean) * (value - mean) for value in finite) / len(finite)
    abs_values = sorted(abs(value) for value in finite)
    p95_index = min(len(abs_values) - 1, math.ceil(0.95 * len(abs_values)) - 1)
    return {
        "mean": mean,
        "median": median(finite),
        "stddev": math.sqrt(variance),
        "rms": math.sqrt(sum(value * value for value in finite) / len(finite)),
        "p95_abs": abs_values[p95_index],
        "max_abs": abs_values[-1],
    }


def summarize_residual_diff(
    base_rows: Sequence[Row],
    candidate_rows: Sequence[Row],
    *,
    row_type: Optional[str] = None,
    iteration: Optional[int] = 0,
    tow_min: Optional[float] = None,
    tow_max: Optional[float] = None,
    top: int = 20,
) -> Dict[str, Any]:
    base_filtered = filter_rows(
        base_rows,
        row_type=row_type,
        iteration=iteration,
        tow_min=tow_min,
        tow_max=tow_max,
    )
    candidate_filtered = filter_rows(
        candidate_rows,
        row_type=row_type,
        iteration=iteration,
        tow_min=tow_min,
        tow_max=tow_max,
    )
    base_index, base_duplicates = index_rows(base_filtered)
    candidate_index, candidate_duplicates = index_rows(candidate_filtered)
    common_keys = sorted(set(base_index).intersection(candidate_index))
    base_only = sorted(set(base_index).difference(candidate_index))
    candidate_only = sorted(set(candidate_index).difference(base_index))

    grouped: DefaultDict[GroupKey, List[Dict[str, Any]]] = defaultdict(list)
    deltas: List[float] = []
    examples: List[Dict[str, Any]] = []
    for key in common_keys:
        base_row = base_index[key]
        candidate_row = candidate_index[key]
        base_residual = to_float(base_row["residual_m"])
        candidate_residual = to_float(candidate_row["residual_m"])
        delta = candidate_residual - base_residual
        if not math.isfinite(delta):
            continue
        item = {
            **key_to_dict(key),
            "base_residual_m": base_residual,
            "candidate_residual_m": candidate_residual,
            "delta_m": delta,
            "abs_delta_m": abs(delta),
        }
        grouped[group_key(base_row)].append(item)
        deltas.append(delta)
        examples.append(item)

    group_summaries: List[Dict[str, Any]] = []
    for key, items in grouped.items():
        group_deltas = [item["delta_m"] for item in items]
        base_values = [item["base_residual_m"] for item in items]
        candidate_values = [item["candidate_residual_m"] for item in items]
        delta_stats = finite_stats(group_deltas)
        base_stats = finite_stats(base_values)
        candidate_stats = finite_stats(candidate_values)
        group_summaries.append(
            {
                "sat": key[0],
                "frequency_index": key[1],
                "row_type": key[2],
                "rows": len(items),
                "base_mean_residual_m": base_stats["mean"],
                "candidate_mean_residual_m": candidate_stats["mean"],
                "mean_delta_m": delta_stats["mean"],
                "median_delta_m": delta_stats["median"],
                "stddev_delta_m": delta_stats["stddev"],
                "rms_delta_m": delta_stats["rms"],
                "p95_abs_delta_m": delta_stats["p95_abs"],
                "max_abs_delta_m": delta_stats["max_abs"],
            }
        )

    group_summaries.sort(
        key=lambda item: (
            -float(item["rms_delta_m"]) if math.isfinite(item["rms_delta_m"]) else 0.0,
            item["sat"],
            item["frequency_index"],
            item["row_type"],
        )
    )
    examples.sort(key=lambda item: item["abs_delta_m"], reverse=True)

    return {
        "base_rows": len(base_filtered),
        "candidate_rows": len(candidate_filtered),
        "matched_rows": len(common_keys),
        "compared_rows": len(deltas),
        "base_only_rows": len(base_only),
        "candidate_only_rows": len(candidate_only),
        "base_duplicate_keys": sum(base_duplicates.values()),
        "candidate_duplicate_keys": sum(candidate_duplicates.values()),
        "row_type": row_type,
        "iteration": iteration,
        "tow_min": tow_min,
        "tow_max": tow_max,
        "overall": finite_stats(deltas),
        "groups": group_summaries,
        "top_deltas": examples[:top],
        "base_only_examples": [key_to_dict(key) for key in base_only[:top]],
        "candidate_only_examples": [key_to_dict(key) for key in candidate_only[:top]],
    }


def write_group_csv(path: Path, groups: Sequence[Dict[str, Any]]) -> None:
    fieldnames = [
        "sat",
        "frequency_index",
        "row_type",
        "rows",
        "base_mean_residual_m",
        "candidate_mean_residual_m",
        "mean_delta_m",
        "median_delta_m",
        "stddev_delta_m",
        "rms_delta_m",
        "p95_abs_delta_m",
        "max_abs_delta_m",
    ]
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(groups)


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("base_residual_log", type=Path)
    parser.add_argument("candidate_residual_log", type=Path)
    parser.add_argument("--row-type", default="phase")
    parser.add_argument("--all-row-types", action="store_true")
    parser.add_argument("--iteration", type=int, default=0)
    parser.add_argument("--all-iterations", action="store_true")
    parser.add_argument("--tow-min", type=float, default=None)
    parser.add_argument("--tow-max", type=float, default=None)
    parser.add_argument("--top", type=int, default=20)
    parser.add_argument(
        "--ignore-signals",
        action="store_true",
        help=(
            "Drop primary/secondary signal and observation-code columns from "
            "the match key. Use when comparing native --ppp-residual-log "
            "against CLASLIB bridge $SAT-derived CSV that lacks signal info."
        ),
    )
    parser.add_argument("--json-out", type=Path)
    parser.add_argument("--groups-csv", type=Path)
    return parser.parse_args(argv)


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)
    global _IGNORE_SIGNALS
    _IGNORE_SIGNALS = bool(args.ignore_signals)
    row_type = None if args.all_row_types else args.row_type
    iteration = None if args.all_iterations else args.iteration
    report = summarize_residual_diff(
        read_csv(args.base_residual_log),
        read_csv(args.candidate_residual_log),
        row_type=row_type,
        iteration=iteration,
        tow_min=args.tow_min,
        tow_max=args.tow_max,
        top=args.top,
    )
    if args.json_out is not None:
        args.json_out.parent.mkdir(parents=True, exist_ok=True)
        args.json_out.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")
    if args.groups_csv is not None:
        args.groups_csv.parent.mkdir(parents=True, exist_ok=True)
        write_group_csv(args.groups_csv, report["groups"])
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
