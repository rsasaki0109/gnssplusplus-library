#!/usr/bin/env python3
"""Summarize per-epoch GNSS solver timing into reproducible interval metrics.

The native SPP and RTK binaries deliberately emit this telemetry only when a
``--timing-csv`` path is supplied.  This keeps the normal solver output and
runtime contract unchanged while making a Release benchmark auditable.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import statistics
from typing import Any


SCHEMA_VERSION = "gnss-performance-report.v1"
SECONDS_PER_WEEK = 604800.0
STAGE_TIMING_FIELDS = (
    "stage_spp_ms",
    "stage_satellite_collection_ms",
    "stage_initialize_filter_ms",
    "stage_reset_position_ms",
    "stage_bias_update_ms",
    "stage_filter_update_ms",
    "stage_ambiguity_ms",
    "stage_velocity_ms",
)

def _number(raw: str, field: str, row_number: int) -> float:
    try:
        value = float(raw)
    except (TypeError, ValueError) as exc:
        raise ValueError(
            f"row {row_number}: {field} must be a finite number, got {raw!r}"
        ) from exc
    if not math.isfinite(value):
        raise ValueError(f"row {row_number}: {field} must be finite")
    return value


def _optional_number(
    row: dict[str, str], field: str, row_number: int
) -> float | None:
    raw = row.get(field, "").strip()
    if not raw:
        return None
    return _number(raw, field, row_number)


def _bool_value(raw: str, field: str, row_number: int) -> bool:
    normalized = raw.strip().lower()
    if normalized in {"1", "true", "yes", "y"}:
        return True
    if normalized in {"0", "false", "no", "n"}:
        return False
    raise ValueError(f"row {row_number}: {field} must be a boolean/0/1 value")


def read_timing_rows(path: Path) -> tuple[list[dict[str, Any]], bool]:
    """Read and validate timing rows.

    The second return value indicates whether GPST week/tow columns were
    present.  Legacy or hand-authored timing CSVs without those columns are
    still summarized by row order, but cannot provide observation-time
    interval or realtime-factor metrics.
    """

    try:
        handle = path.open(newline="", encoding="utf-8")
    except OSError as exc:
        raise ValueError(f"failed to open timing CSV {path}: {exc}") from exc

    with handle:
        reader = csv.DictReader(handle)
        fields = set(reader.fieldnames or [])
        if "elapsed_ms" not in fields:
            raise ValueError("timing CSV must contain an elapsed_ms column")
        has_week = "gps_week" in fields
        has_tow = "gps_tow_s" in fields or "gps_tow" in fields
        if has_week != has_tow:
            raise ValueError(
                "timing CSV must contain both gps_week and gps_tow_s (or gps_tow), "
                "or neither"
            )
        tow_field = "gps_tow_s" if "gps_tow_s" in fields else "gps_tow"
        rows: list[dict[str, Any]] = []
        previous_gps_seconds: float | None = None
        for row_number, raw_row in enumerate(reader, start=2):
            row = {key: (value or "") for key, value in raw_row.items()}
            elapsed_ms = _number(row.get("elapsed_ms", ""), "elapsed_ms", row_number)
            if elapsed_ms < 0.0:
                raise ValueError(f"row {row_number}: elapsed_ms must be non-negative")

            gps_week: int | None = None
            gps_tow_s: float | None = None
            gps_seconds: float | None = None
            if has_week:
                week_value = _number(row.get("gps_week", ""), "gps_week", row_number)
                if week_value != math.trunc(week_value):
                    raise ValueError(f"row {row_number}: gps_week must be integral")
                gps_week = int(week_value)
                gps_tow_s = _number(row.get(tow_field, ""), tow_field, row_number)
                if gps_tow_s < 0.0 or gps_tow_s >= SECONDS_PER_WEEK:
                    raise ValueError(
                        f"row {row_number}: {tow_field} must be in [0, 604800)"
                    )
                gps_seconds = gps_week * SECONDS_PER_WEEK + gps_tow_s
                if (
                    previous_gps_seconds is not None
                    and gps_seconds < previous_gps_seconds
                ):
                    raise ValueError(
                        f"row {row_number}: GPST timestamps must be non-decreasing"
                    )
                previous_gps_seconds = gps_seconds

            valid = None
            if row.get("valid", "").strip():
                valid = _bool_value(row["valid"], "valid", row_number)
            satellites = _optional_number(row, "num_satellites", row_number)
            if satellites is not None and satellites < 0.0:
                raise ValueError(f"row {row_number}: num_satellites must be non-negative")
            iterations = _optional_number(row, "iterations", row_number)
            if iterations is not None and iterations < 0.0:
                raise ValueError(f"row {row_number}: iterations must be non-negative")
            processor_time_ms = _optional_number(row, "processor_time_ms", row_number)
            if processor_time_ms is not None and processor_time_ms < 0.0:
                raise ValueError(
                    f"row {row_number}: processor_time_ms must be non-negative"
                )
            stage_timings: dict[str, float | None] = {}
            for field in STAGE_TIMING_FIELDS:
                value = _optional_number(row, field, row_number)
                if value is not None and value < 0.0:
                    raise ValueError(
                        f"row {row_number}: {field} must be non-negative"
                    )
                stage_timings[field] = value
            rows.append(
                {
                    "row_number": row_number,
                    "elapsed_ms": elapsed_ms,
                    "gps_week": gps_week,
                    "gps_tow_s": gps_tow_s,
                    "gps_seconds": gps_seconds,
                    "valid": valid,
                    "status": row.get("status", ""),
                    "num_satellites": satellites,
                    "iterations": iterations,
                    "processor_time_ms": processor_time_ms,
                    "stage_timings": stage_timings,
                }
            )
    return rows, has_week


def _percentile(values: list[float], fraction: float) -> float:
    if not values:
        return 0.0
    ordered = sorted(values)
    if len(ordered) == 1:
        return ordered[0]
    rank = (len(ordered) - 1) * fraction
    lower = math.floor(rank)
    upper = math.ceil(rank)
    if lower == upper:
        return ordered[lower]
    weight = rank - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def _rounded(value: float) -> float:
    return round(float(value), 6)


def _epoch_count_metric(rows: list[dict[str, Any]], key: str) -> float | None:
    values = [float(row[key]) for row in rows if row[key] is not None]
    return _rounded(statistics.fmean(values)) if values else None


def summarize_rows(
    rows: list[dict[str, Any]],
    timestamp_available: bool,
    interval_index: int | None = None,
) -> dict[str, Any]:
    elapsed_ms = [float(row["elapsed_ms"]) for row in rows]
    processor_time_ms = [
        float(row["processor_time_ms"])
        for row in rows
        if row["processor_time_ms"] is not None
    ]
    wall_time_s = sum(elapsed_ms) / 1000.0
    valid_rows = [row for row in rows if row["valid"] is True]
    starts = [row["gps_seconds"] for row in rows if row["gps_seconds"] is not None]
    start_gps_seconds = min(starts) if starts else None
    end_gps_seconds = max(starts) if starts else None
    observation_span_s = (
        max(0.0, end_gps_seconds - start_gps_seconds)
        if start_gps_seconds is not None and end_gps_seconds is not None
        else None
    )
    status_counts: dict[str, int] = {}
    for row in rows:
        status = str(row["status"])
        status_counts[status] = status_counts.get(status, 0) + 1
    metrics: dict[str, Any] = {
        "epoch_count": len(rows),
        "valid_epoch_count": len(valid_rows),
        "valid_rate": _rounded(len(valid_rows) / len(rows)) if rows else 0.0,
        "solver_wall_time_s": _rounded(wall_time_s),
        "mean_epoch_ms": _rounded(statistics.fmean(elapsed_ms)) if elapsed_ms else 0.0,
        "p50_epoch_ms": _rounded(_percentile(elapsed_ms, 0.50)),
        "p95_epoch_ms": _rounded(_percentile(elapsed_ms, 0.95)),
        "max_epoch_ms": _rounded(max(elapsed_ms) if elapsed_ms else 0.0),
        "processor_time_coverage": _rounded(
            len(processor_time_ms) / len(rows) if rows else 0.0
        ),
        "mean_processor_time_ms": _rounded(statistics.fmean(processor_time_ms))
        if processor_time_ms
        else None,
        "p95_processor_time_ms": _rounded(_percentile(processor_time_ms, 0.95))
        if processor_time_ms
        else None,
        "effective_epoch_rate_hz": _rounded(
            len(rows) / wall_time_s if wall_time_s > 0.0 else 0.0
        ),
        "realtime_factor": _rounded(
            observation_span_s / wall_time_s
            if observation_span_s is not None and wall_time_s > 0.0
            else 0.0
        ),
        "mean_satellites": _epoch_count_metric(rows, "num_satellites"),
        "mean_iterations": _epoch_count_metric(rows, "iterations"),
        "status_counts": status_counts,
        "timestamp_available": timestamp_available,
    }
    stage_metrics: dict[str, Any] = {}
    for field in STAGE_TIMING_FIELDS:
        values = [
            float(row["stage_timings"][field])
            for row in rows
            if row["stage_timings"][field] is not None
        ]
        if values:
            stage_metrics[field] = {
                "coverage": _rounded(len(values) / len(rows)) if rows else 0.0,
                "mean_ms": _rounded(statistics.fmean(values)),
                "p50_ms": _rounded(_percentile(values, 0.50)),
                "p95_ms": _rounded(_percentile(values, 0.95)),
                "max_ms": _rounded(max(values)),
                "total_ms": _rounded(sum(values)),
            }
    metrics["stage_timings"] = stage_metrics
    if interval_index is not None:
        metrics["interval_index"] = interval_index
    if start_gps_seconds is not None:
        metrics["start_gps_week"] = int(math.floor(start_gps_seconds / SECONDS_PER_WEEK))
        metrics["start_gps_tow_s"] = _rounded(
            start_gps_seconds % SECONDS_PER_WEEK
        )
        metrics["end_gps_week"] = int(math.floor(end_gps_seconds / SECONDS_PER_WEEK))
        metrics["end_gps_tow_s"] = _rounded(end_gps_seconds % SECONDS_PER_WEEK)
        metrics["observation_span_s"] = _rounded(observation_span_s or 0.0)
    else:
        metrics["observation_span_s"] = None
    return metrics


def build_report(path: Path, interval_s: float = 60.0) -> dict[str, Any]:
    if not math.isfinite(interval_s) or interval_s <= 0.0:
        raise ValueError("interval_s must be a finite positive number")
    rows, timestamp_available = read_timing_rows(path)
    intervals: list[dict[str, Any]] = []
    if rows:
        if timestamp_available:
            anchor = float(rows[0]["gps_seconds"])
            grouped: dict[int, list[dict[str, Any]]] = {}
            for row in rows:
                index = int(math.floor((float(row["gps_seconds"]) - anchor) / interval_s))
                grouped.setdefault(max(0, index), []).append(row)
            intervals = [
                summarize_rows(grouped[index], timestamp_available, index)
                for index in sorted(grouped)
            ]
        else:
            # Without GPST there is no defensible observation-time interval
            # boundary. Keep one row-order aggregate and report the missing
            # timestamp contract explicitly instead of inventing a sample rate.
            intervals = [summarize_rows(rows, timestamp_available, 0)]
    report: dict[str, Any] = {
        "schema_version": SCHEMA_VERSION,
        "input": {
            "path": str(path),
            "sha256": _sha256(path),
            "row_count": len(rows),
        },
        "interval_s": _rounded(interval_s),
        "timing_scope": "native solver processEpoch/processRTKEpoch calls only",
        "overall": summarize_rows(rows, timestamp_available),
        "intervals": intervals,
    }
    return report


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    try:
        with path.open("rb") as handle:
            for chunk in iter(lambda: handle.read(1024 * 1024), b""):
                digest.update(chunk)
    except OSError as exc:
        raise ValueError(f"failed to hash timing CSV {path}: {exc}") from exc
    return digest.hexdigest()


def write_report(
    report: dict[str, Any], output_json: Path, output_csv: Path | None = None
) -> None:
    output_json.parent.mkdir(parents=True, exist_ok=True)
    output_json.write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    if output_csv is None:
        return
    output_csv.parent.mkdir(parents=True, exist_ok=True)
    columns = [
        "interval_index",
        "start_gps_week",
        "start_gps_tow_s",
        "end_gps_week",
        "end_gps_tow_s",
        "observation_span_s",
        "epoch_count",
        "valid_epoch_count",
        "valid_rate",
        "solver_wall_time_s",
        "mean_epoch_ms",
        "p50_epoch_ms",
        "p95_epoch_ms",
        "max_epoch_ms",
        "processor_time_coverage",
        "mean_processor_time_ms",
        "p95_processor_time_ms",
        "effective_epoch_rate_hz",
        "realtime_factor",
        "mean_satellites",
        "mean_iterations",
        "timestamp_available",
        "status_counts",
    ]
    with output_csv.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=columns, extrasaction="ignore")
        writer.writeheader()
        for interval in report["intervals"]:
            row = dict(interval)
            row["status_counts"] = json.dumps(
                row.get("status_counts", {}), sort_keys=True, separators=(",", ":")
            )
            writer.writerow(row)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Summarize native per-epoch solver timing CSVs by GPST interval."
    )
    parser.add_argument("--timing-csv", type=Path, required=True)
    parser.add_argument("--output-json", type=Path, required=True)
    parser.add_argument("--output-csv", type=Path)
    parser.add_argument("--interval-s", type=float, default=60.0)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        report = build_report(args.timing_csv, args.interval_s)
        write_report(report, args.output_json, args.output_csv)
    except (OSError, ValueError) as exc:
        raise SystemExit(f"performance report failed: {exc}") from exc
    print(f"Performance report: {args.output_json}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
