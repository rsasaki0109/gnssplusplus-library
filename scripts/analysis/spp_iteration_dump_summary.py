#!/usr/bin/env python3
"""Summarize SPP least-squares iteration dumps."""

from __future__ import annotations

import argparse
import csv
import json
import math
from collections import Counter, defaultdict
from decimal import Decimal, InvalidOperation, ROUND_HALF_UP
from pathlib import Path
from typing import Any, DefaultDict, Dict, List, Optional, Sequence, Tuple


REQUIRED_COLUMNS = [
    "week",
    "tow",
    "iteration",
    "sat",
    "residual_m",
    "residual_rms_m",
    "dx_norm_m",
    "rejected_after",
    "abs_residual_gt_threshold",
]

Row = Dict[str, str]
EpochKey = Tuple[int, int]
SatKey = Tuple[str, str]


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


def truthy(value: str) -> bool:
    return str(value).strip().lower() in {"1", "true", "yes", "y"}


def read_csv(path: Path) -> List[Row]:
    with path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        if reader.fieldnames is None:
            raise ValueError(f"{path}: missing CSV header")
        missing = [column for column in REQUIRED_COLUMNS if column not in reader.fieldnames]
        if missing:
            raise ValueError(f"{path}: missing required columns: {', '.join(missing)}")
        return [dict(row) for row in reader]


def epoch_key(row: Row) -> EpochKey:
    return (to_int(row["week"]), tow_to_millis(row["tow"]))


def epoch_to_dict(key: EpochKey) -> Dict[str, Any]:
    return {"week": key[0], "tow": key[1] / 1000.0}


def select_last_iteration_per_epoch(rows: Sequence[Row]) -> List[Row]:
    max_iteration_by_epoch: Dict[EpochKey, int] = {}
    for row in rows:
        key = epoch_key(row)
        max_iteration_by_epoch[key] = max(
            max_iteration_by_epoch.get(key, -1),
            to_int(row["iteration"]),
        )
    return [
        row for row in rows
        if to_int(row["iteration"]) == max_iteration_by_epoch[epoch_key(row)]
    ]


def summarize(
    rows: Sequence[Row],
    *,
    top: int = 20,
    last_iteration_per_epoch: bool = False,
) -> Dict[str, Any]:
    if last_iteration_per_epoch:
        rows = select_last_iteration_per_epoch(rows)

    epoch_iterations: DefaultDict[EpochKey, set[int]] = defaultdict(set)
    epoch_max_rms: DefaultDict[EpochKey, float] = defaultdict(float)
    epoch_max_dx: DefaultDict[EpochKey, float] = defaultdict(float)
    sat_abs_residual_sum: DefaultDict[SatKey, float] = defaultdict(float)
    sat_abs_residual_max: DefaultDict[SatKey, float] = defaultdict(float)
    sat_rows: Counter[SatKey] = Counter()
    rejected: Counter[SatKey] = Counter()
    over_threshold: Counter[SatKey] = Counter()
    top_rows: List[Dict[str, Any]] = []

    for row in rows:
        key = epoch_key(row)
        iteration = to_int(row["iteration"])
        epoch_iterations[key].add(iteration)
        epoch_max_rms[key] = max(epoch_max_rms[key], abs(to_float(row["residual_rms_m"])))
        epoch_max_dx[key] = max(epoch_max_dx[key], abs(to_float(row["dx_norm_m"])))

        sat_key = (row.get("sat", ""), row.get("signal", ""))
        residual = to_float(row["residual_m"])
        abs_residual = abs(residual) if math.isfinite(residual) else math.nan
        if math.isfinite(abs_residual):
            sat_abs_residual_sum[sat_key] += abs_residual
            sat_abs_residual_max[sat_key] = max(sat_abs_residual_max[sat_key], abs_residual)
            top_rows.append(
                {
                    **epoch_to_dict(key),
                    "iteration": iteration,
                    "sat": sat_key[0],
                    "signal": sat_key[1],
                    "residual_m": residual,
                    "abs_residual_m": abs_residual,
                    "residual_rms_m": to_float(row["residual_rms_m"]),
                    "dx_norm_m": to_float(row["dx_norm_m"]),
                    "rejected_after": truthy(row["rejected_after"]),
                    "abs_residual_gt_threshold": truthy(row["abs_residual_gt_threshold"]),
                }
            )
        sat_rows[sat_key] += 1
        if truthy(row["rejected_after"]):
            rejected[sat_key] += 1
        if truthy(row["abs_residual_gt_threshold"]):
            over_threshold[sat_key] += 1

    sat_summaries: List[Dict[str, Any]] = []
    for sat_key, count in sat_rows.items():
        sat_summaries.append(
            {
                "sat": sat_key[0],
                "signal": sat_key[1],
                "rows": count,
                "mean_abs_residual_m": sat_abs_residual_sum[sat_key] / count if count else math.nan,
                "max_abs_residual_m": sat_abs_residual_max[sat_key],
                "rejected_rows": rejected[sat_key],
                "over_threshold_rows": over_threshold[sat_key],
            }
        )
    sat_summaries.sort(
        key=lambda item: (
            -item["rejected_rows"],
            -item["over_threshold_rows"],
            -item["max_abs_residual_m"],
            item["sat"],
            item["signal"],
        )
    )
    top_rows.sort(key=lambda item: item["abs_residual_m"], reverse=True)

    worst_epoch_key: Optional[EpochKey] = None
    if epoch_max_rms:
        worst_epoch_key = max(epoch_max_rms, key=lambda key: epoch_max_rms[key])

    return {
        "rows": len(rows),
        "last_iteration_per_epoch": last_iteration_per_epoch,
        "epochs": len(epoch_iterations),
        "max_iterations_per_epoch": max((len(value) for value in epoch_iterations.values()), default=0),
        "worst_epoch_by_rms": (
            {**epoch_to_dict(worst_epoch_key), "max_residual_rms_m": epoch_max_rms[worst_epoch_key]}
            if worst_epoch_key is not None else None
        ),
        "max_dx_norm_m": max(epoch_max_dx.values(), default=0.0),
        "satellites": sat_summaries[:top],
        "top_residual_rows": top_rows[:top],
    }


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("dump_csv", type=Path)
    parser.add_argument("--top", type=int, default=20)
    parser.add_argument(
        "--last-iteration-per-epoch",
        action="store_true",
        help="Summarize only the highest iteration number observed for each epoch.",
    )
    parser.add_argument("--json-out", type=Path)
    return parser.parse_args(argv)


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)
    report = summarize(
        read_csv(args.dump_csv),
        top=args.top,
        last_iteration_per_epoch=args.last_iteration_per_epoch,
    )
    text = json.dumps(report, indent=2, sort_keys=True) + "\n"
    if args.json_out is not None:
        args.json_out.parent.mkdir(parents=True, exist_ok=True)
        args.json_out.write_text(text, encoding="utf-8")
    print(text, end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
