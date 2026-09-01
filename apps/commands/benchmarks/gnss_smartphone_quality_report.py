#!/usr/bin/env python3
"""Correlate smartphone GNSS quality telemetry with the frozen sign-off lane.

The report is deliberately observational: quality buckets are fixed before
truth errors are attached, and no truth-derived value is fed back into the
device/RINEX preprocessing.  This keeps development-route experiments
auditable and prevents a quality threshold from being trained on the same
errors that it reports.
"""

from __future__ import annotations

import argparse
import bisect
import csv
from collections import Counter, defaultdict
from dataclasses import dataclass
import hashlib
import json
import math
import os
from pathlib import Path
from statistics import median
from typing import Any, Iterable


SCHEMA_VERSION = "smartphone-gnss-quality-report.v1"
DEVICE_REQUIRED = (
    "MessageType",
    "utcTimeMillis",
    "HardwareClockDiscontinuityCount",
    "Svid",
    "AccumulatedDeltaRangeState",
    "SignalType",
    "ReceivedSvTimeUncertaintyNanos",
    "Cn0DbHz",
    "RawPseudorangeMeters",
    "RawPseudorangeUncertaintyMeters",
    "ArrivalTimeNanosSinceGpsEpoch",
)
MATCH_REQUIRED = (
    "solution_unix_time_millis",
    "truth_unix_time_millis",
    "horizontal_error_m",
    "vertical_error_m",
)
DEFAULT_SEGMENT_SECONDS = 60.0
DEFAULT_MATCH_TOLERANCE_MS = 100.0


@dataclass
class EpochFeature:
    index: int
    timestamp: int
    row_count: int
    solver_rows: int
    unsupported_rows: int
    unsupported_signal_counts: dict[str, int]
    cn0_values: list[float]
    raw_pseudorange_uncertainty_values: list[float]
    received_sv_time_uncertainty_values: list[float]
    adr_states: list[int]
    hardware_clock_discontinuity_count: int
    epoch_gap_s: float | None
    match: dict[str, Any] | None = None

    @property
    def median_cn0_dbhz(self) -> float | None:
        return _median(self.cn0_values)

    @property
    def minimum_cn0_dbhz(self) -> float | None:
        return min(self.cn0_values) if self.cn0_values else None

    @property
    def median_raw_pseudorange_uncertainty_m(self) -> float | None:
        return _median(self.raw_pseudorange_uncertainty_values)

    @property
    def median_received_sv_time_uncertainty_ns(self) -> float | None:
        return _median(self.received_sv_time_uncertainty_values)

    @property
    def adr_valid_fraction(self) -> float | None:
        if not self.adr_states:
            return None
        return sum(bool(state & 1) for state in self.adr_states) / len(self.adr_states)

    @property
    def adr_state_bucket(self) -> str:
        if not self.adr_states:
            return "missing"
        if any(state & 0b110 for state in self.adr_states):
            return "reset_or_cycle_slip_present"
        if all(state & 1 for state in self.adr_states):
            return "valid_no_reset_or_slip"
        if any(state & 1 for state in self.adr_states):
            return "mixed_validity"
        return "invalid_no_valid_bit"

    @property
    def unsupported_fraction(self) -> float:
        return self.unsupported_rows / self.row_count if self.row_count else 0.0


def _median(values: list[float]) -> float | None:
    return median(values) if values else None


def _percentile(values: list[float], fraction: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    rank = (len(ordered) - 1) * fraction
    lower = math.floor(rank)
    upper = math.ceil(rank)
    if lower == upper:
        return ordered[lower]
    return ordered[lower] * (upper - rank) + ordered[upper] * (rank - lower)


def _number(raw: str | None, field: str, row_number: int) -> float:
    try:
        value = float(raw)  # type: ignore[arg-type]
    except (TypeError, ValueError) as exc:
        raise ValueError(
            f"row {row_number}: {field} must be a finite number, got {raw!r}"
        ) from exc
    if not math.isfinite(value):
        raise ValueError(f"row {row_number}: {field} must be finite")
    return value


def _integer(raw: str | None, field: str, row_number: int) -> int:
    value = _number(raw, field, row_number)
    if not value.is_integer():
        raise ValueError(f"row {row_number}: {field} must be integral")
    return int(value)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    try:
        with path.open("rb") as handle:
            for chunk in iter(lambda: handle.read(1024 * 1024), b""):
                digest.update(chunk)
    except OSError as exc:
        raise ValueError(f"failed to hash {path}: {exc}") from exc
    return digest.hexdigest()


def _load_json(path: Path, label: str) -> dict[str, Any]:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ValueError(f"invalid {label} {path}: {exc}") from exc
    if not isinstance(payload, dict):
        raise ValueError(f"{label} must contain a JSON object")
    return payload


def _open_device(path: Path) -> tuple[csv.DictReader, Any]:
    try:
        handle = path.open(encoding="utf-8-sig", newline="")
    except OSError as exc:
        raise ValueError(f"failed to open device GNSS CSV {path}: {exc}") from exc
    reader = csv.DictReader(handle)
    fields = list(reader.fieldnames or ())
    if len(fields) != len(set(fields)):
        handle.close()
        raise ValueError(f"duplicate CSV fields in {path}")
    missing = [field for field in DEVICE_REQUIRED if field not in fields]
    if missing:
        handle.close()
        raise ValueError(f"device GNSS CSV missing fields: {', '.join(missing)}")
    return reader, handle


def _iter_device_epochs(path: Path) -> Iterable[tuple[int, int, list[dict[str, str]]]]:
    reader, handle = _open_device(path)
    current_timestamp: int | None = None
    current_rows: list[dict[str, str]] = []
    last_timestamp: int | None = None
    epoch_index = 0
    saw_row = False
    try:
        for row_number, raw_row in enumerate(reader, start=2):
            row = {key: (value or "") for key, value in raw_row.items()}
            saw_row = True
            if row["MessageType"] != "Raw":
                raise ValueError(f"row {row_number}: MessageType must be Raw")
            timestamp = _integer(row["utcTimeMillis"], "utcTimeMillis", row_number)
            if last_timestamp is not None and timestamp < last_timestamp:
                raise ValueError(f"row {row_number}: utcTimeMillis moved backwards")
            last_timestamp = timestamp
            if current_timestamp is None:
                current_timestamp = timestamp
            elif timestamp != current_timestamp:
                yield epoch_index, current_timestamp, current_rows
                epoch_index += 1
                current_timestamp = timestamp
                current_rows = []
            current_rows.append(row)
        if not saw_row:
            raise ValueError(f"no data rows in {path}")
        if current_timestamp is not None:
            yield epoch_index, current_timestamp, current_rows
    finally:
        handle.close()


def _finite_values(rows: list[dict[str, str]], field: str, row_offset: int = 0) -> list[float]:
    values: list[float] = []
    for index, row in enumerate(rows, start=2 + row_offset):
        token = row.get(field, "").strip()
        if not token:
            continue
        values.append(_number(token, field, index))
    return values


def _feature(
    index: int,
    timestamp: int,
    rows: list[dict[str, str]],
    previous_timestamp: int | None,
) -> EpochFeature:
    solver_rows = [
        row
        for row in rows
        if row.get("SignalType", "").strip() == "GPS_L1_CA"
        and row.get("RawPseudorangeMeters", "").strip()
        and row.get("ArrivalTimeNanosSinceGpsEpoch", "").strip()
    ]
    unsupported_signal_counts: Counter[str] = Counter()
    adr_states: list[int] = []
    for row in solver_rows:
        adr_states.append(
            _integer(row.get("AccumulatedDeltaRangeState"), "AccumulatedDeltaRangeState", 0)
        )
    for row in rows:
        signal = row.get("SignalType", "").strip() or "<unmapped>"
        if signal != "GPS_L1_CA":
            unsupported_signal_counts[signal] += 1
    clock_counts = {
        _integer(row.get("HardwareClockDiscontinuityCount"), "HardwareClockDiscontinuityCount", 0)
        for row in rows
    }
    if len(clock_counts) != 1:
        raise ValueError(f"epoch {timestamp}: inconsistent hardware clock discontinuity count")
    return EpochFeature(
        index=index,
        timestamp=timestamp,
        row_count=len(rows),
        solver_rows=len(solver_rows),
        unsupported_rows=sum(unsupported_signal_counts.values()),
        unsupported_signal_counts=dict(sorted(unsupported_signal_counts.items())),
        cn0_values=_finite_values(solver_rows, "Cn0DbHz"),
        raw_pseudorange_uncertainty_values=_finite_values(
            solver_rows, "RawPseudorangeUncertaintyMeters"
        ),
        received_sv_time_uncertainty_values=_finite_values(
            solver_rows, "ReceivedSvTimeUncertaintyNanos"
        ),
        adr_states=adr_states,
        hardware_clock_discontinuity_count=next(iter(clock_counts)),
        epoch_gap_s=(timestamp - previous_timestamp) / 1000.0
        if previous_timestamp is not None
        else None,
    )


def _read_matches(path: Path) -> list[dict[str, Any]]:
    try:
        handle = path.open(encoding="utf-8-sig", newline="")
    except OSError as exc:
        raise ValueError(f"failed to open sign-off matches CSV {path}: {exc}") from exc
    matches: list[dict[str, Any]] = []
    try:
        reader = csv.DictReader(handle)
        fields = set(reader.fieldnames or ())
        missing = [field for field in MATCH_REQUIRED if field not in fields]
        if missing:
            raise ValueError(f"matches CSV missing fields: {', '.join(missing)}")
        previous_timestamp: int | None = None
        for row_number, raw_row in enumerate(reader, start=2):
            row = {key: (value or "") for key, value in raw_row.items()}
            solution_timestamp = _integer(
                row["solution_unix_time_millis"],
                "solution_unix_time_millis",
                row_number,
            )
            if previous_timestamp is not None and solution_timestamp <= previous_timestamp:
                raise ValueError(f"matches row {row_number}: solution timestamps must increase")
            previous_timestamp = solution_timestamp
            horizontal_error = _number(row["horizontal_error_m"], "horizontal_error_m", row_number)
            vertical_error = _number(row["vertical_error_m"], "vertical_error_m", row_number)
            if horizontal_error < 0.0:
                raise ValueError(f"matches row {row_number}: horizontal_error_m must be non-negative")
            item: dict[str, Any] = {
                "solution_unix_time_millis": solution_timestamp,
                "truth_unix_time_millis": _integer(
                    row.get("truth_unix_time_millis"),
                    "truth_unix_time_millis",
                    row_number,
                ),
                "horizontal_error_m": horizontal_error,
                "vertical_error_m": vertical_error,
            }
            for field in ("status", "satellites", "time_delta_ms"):
                if row.get(field, "").strip():
                    item[field] = _number(row[field], field, row_number)
            matches.append(item)
    finally:
        handle.close()
    if not matches:
        raise ValueError("matches CSV contains no rows")
    return matches


def _attach_matches(
    features: list[EpochFeature],
    matches: list[dict[str, Any]],
    tolerance_ms: float,
) -> None:
    timestamps = [feature.timestamp for feature in features]
    used: set[int] = set()
    previous_solution_timestamp: int | None = None
    for match in matches:
        target = int(match["solution_unix_time_millis"])
        insertion = bisect.bisect_left(timestamps, target)
        candidates = [
            index
            for index in (insertion - 1, insertion)
            if 0 <= index < len(features)
        ]
        if not candidates:
            raise ValueError(f"solution timestamp {target} has no device epoch")
        index = min(candidates, key=lambda candidate: (abs(timestamps[candidate] - target), candidate))
        delta_ms = abs(timestamps[index] - target)
        if delta_ms > tolerance_ms:
            raise ValueError(
                f"solution timestamp {target} is {delta_ms} ms from the nearest device epoch"
            )
        if index in used:
            raise ValueError(f"multiple solution rows map to device epoch {timestamps[index]}")
        used.add(index)
        solution_gap_s = (
            (target - previous_solution_timestamp) / 1000.0
            if previous_solution_timestamp is not None
            else None
        )
        previous_solution_timestamp = target
        features[index].match = {
            **match,
            "device_epoch_delta_ms": delta_ms,
            "solution_gap_s": solution_gap_s,
        }


def _distribution(values: list[float]) -> dict[str, float | None]:
    return {
        "count": len(values),
        "min": min(values) if values else None,
        "p10": _percentile(values, 0.10),
        "median": _median(values),
        "p90": _percentile(values, 0.90),
        "p95": _percentile(values, 0.95),
        "max": max(values) if values else None,
    }


def _quality_values(feature: EpochFeature) -> dict[str, float | None]:
    return {
        "median_cn0_dbhz": feature.median_cn0_dbhz,
        "median_raw_pseudorange_uncertainty_m": feature.median_raw_pseudorange_uncertainty_m,
        "median_received_sv_time_uncertainty_ns": feature.median_received_sv_time_uncertainty_ns,
        "adr_valid_fraction": feature.adr_valid_fraction,
        "unsupported_fraction": feature.unsupported_fraction,
        "epoch_gap_s": feature.epoch_gap_s,
        "solution_gap_s": (
            float(feature.match["solution_gap_s"])
            if feature.match is not None and feature.match["solution_gap_s"] is not None
            else None
        ),
    }


def _error_metrics(features: list[EpochFeature]) -> dict[str, Any]:
    matches = [feature.match for feature in features if feature.match is not None]
    horizontal = [float(match["horizontal_error_m"]) for match in matches]
    vertical = [abs(float(match["vertical_error_m"])) for match in matches]
    solution_times = [int(match["solution_unix_time_millis"]) for match in matches]
    solution_gaps = [
        (current - previous) / 1000.0
        for previous, current in zip(solution_times, solution_times[1:])
    ]
    input_gaps = [
        feature.epoch_gap_s
        for feature in features
        if feature.epoch_gap_s is not None
    ]
    return {
        "epoch_count": len(features),
        "solution_epoch_count": len(matches),
        "availability_ratio": len(matches) / len(features) if features else 0.0,
        "horizontal_median_m": _median(horizontal),
        "horizontal_p95_m": _percentile(horizontal, 0.95),
        "vertical_p95_abs_m": _percentile(vertical, 0.95),
        "max_input_epoch_gap_s": max(input_gaps) if input_gaps else 0.0,
        "max_solution_gap_s": max(solution_gaps) if solution_gaps else 0.0,
    }


def _quality_distribution(features: list[EpochFeature], key: str) -> dict[str, Any]:
    values = [
        float(value)
        for feature in features
        if (value := _quality_values(feature).get(key)) is not None
    ]
    return _distribution(values)


def _segment_rows(features: list[EpochFeature], segment_seconds: float) -> list[dict[str, Any]]:
    first_timestamp = features[0].timestamp
    grouped: dict[int, list[EpochFeature]] = defaultdict(list)
    for feature in features:
        segment_index = int(
            math.floor((feature.timestamp - first_timestamp) / 1000.0 / segment_seconds)
        )
        grouped[segment_index].append(feature)
    rows: list[dict[str, Any]] = []
    for segment_index, segment_features in sorted(grouped.items()):
        metrics = _error_metrics(segment_features)
        quality = {
            key: _median(
                [
                    float(value)
                    for feature in segment_features
                    if (value := _quality_values(feature).get(key)) is not None
                ]
            )
            for key in (
                "median_cn0_dbhz",
                "median_raw_pseudorange_uncertainty_m",
                "median_received_sv_time_uncertainty_ns",
                "adr_valid_fraction",
                "unsupported_fraction",
                "epoch_gap_s",
                "solution_gap_s",
            )
        }
        signal_counts: Counter[str] = Counter()
        for feature in segment_features:
            signal_counts.update(feature.unsupported_signal_counts)
        rows.append(
            {
                "segment_index": segment_index,
                "start_utc_time_millis": segment_features[0].timestamp,
                "end_utc_time_millis": segment_features[-1].timestamp,
                **metrics,
                **quality,
                "hardware_clock_discontinuity_counts": sorted(
                    {feature.hardware_clock_discontinuity_count for feature in segment_features}
                ),
                "unsupported_signal_rows": sum(signal_counts.values()),
                "unsupported_signal_counts": dict(sorted(signal_counts.items())),
            }
        )
    return rows


QUALITY_BUCKETS: dict[str, tuple[str, tuple[tuple[float | None, float | None, str], ...]]] = {
    "median_cn0_dbhz": (
        "median GPS_L1_CA C/N0 [dB-Hz]",
        (
            (None, 25.0, "<25"),
            (25.0, 30.0, "25-30"),
            (30.0, 35.0, "30-35"),
            (35.0, 40.0, "35-40"),
            (40.0, 45.0, "40-45"),
            (45.0, None, ">=45"),
        ),
    ),
    "median_raw_pseudorange_uncertainty_m": (
        "median RawPseudorangeUncertaintyMeters [m]",
        (
            (None, 4.0, "<4"),
            (4.0, 6.0, "4-6"),
            (6.0, 8.0, "6-8"),
            (8.0, 12.0, "8-12"),
            (12.0, 20.0, "12-20"),
            (20.0, None, ">=20"),
        ),
    ),
    "median_received_sv_time_uncertainty_ns": (
        "median ReceivedSvTimeUncertaintyNanos [ns]",
        (
            (None, 10.0, "<10"),
            (10.0, 20.0, "10-20"),
            (20.0, 30.0, "20-30"),
            (30.0, 50.0, "30-50"),
            (50.0, 100.0, "50-100"),
            (100.0, None, ">=100"),
        ),
    ),
    "adr_valid_fraction": (
        "fraction of GPS_L1_CA ADR states with valid bit",
        (
            (None, 0.01, "0%"),
            (0.01, 0.50, "1-49%"),
            (0.50, 1.0, "50-99%"),
            (1.0, None, "100%"),
        ),
    ),
    "unsupported_fraction": (
        "fraction of selected rows not mapped to GPS_L1_CA",
        (
            (None, 0.25, "<25%"),
            (0.25, 0.50, "25-50%"),
            (0.50, 0.75, "50-75%"),
            (0.75, 0.90, "75-90%"),
            (0.90, None, ">=90%"),
        ),
    ),
    "epoch_gap_s": (
        "gap from previous selected raw epoch [s]",
        (
            (None, 0.0, "first_epoch"),
            (0.0, 1.5, "<=1.5"),
            (1.5, 2.5, "1.5-2.5"),
            (2.5, 5.0, "2.5-5"),
            (5.0, None, ">=5"),
        ),
    ),
    "solution_gap_s": (
        "gap from previous truth-matched solution epoch [s]",
        (
            (None, 0.0, "first_solution"),
            (0.0, 1.5, "<=1.5"),
            (1.5, 2.5, "1.5-2.5"),
            (2.5, 5.0, "2.5-5"),
            (5.0, None, ">=5"),
        ),
    ),
}


def _bucket(value: float | None, bins: tuple[tuple[float | None, float | None, str], ...]) -> str:
    if value is None:
        return "missing"
    for lower, upper, label in bins:
        if lower is not None and value < lower:
            continue
        if upper is not None and value >= upper:
            continue
        return label
    return "out_of_range"


def _clock_bucket(feature: EpochFeature) -> str:
    return f"count={feature.hardware_clock_discontinuity_count}"


def _bucket_rows(features: list[EpochFeature]) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    dimensions = list(QUALITY_BUCKETS)
    dimensions.extend(("hardware_clock_discontinuity_count", "adr_state"))
    for dimension in dimensions:
        grouped: dict[str, list[EpochFeature]] = defaultdict(list)
        for feature in features:
            if dimension in QUALITY_BUCKETS:
                value = _quality_values(feature)[dimension]
                if dimension == "epoch_gap_s" and value is None:
                    label = "first_epoch"
                elif dimension == "solution_gap_s" and feature.match is None:
                    label = "no_solution"
                elif dimension == "solution_gap_s" and value is None:
                    label = "first_solution"
                else:
                    label = _bucket(value, QUALITY_BUCKETS[dimension][1])
            elif dimension == "hardware_clock_discontinuity_count":
                label = _clock_bucket(feature)
            else:
                label = feature.adr_state_bucket
            grouped[label].append(feature)
        for label, bucket_features in sorted(grouped.items()):
            values = [
                float(value)
                for feature in bucket_features
                if dimension in QUALITY_BUCKETS
                and (value := _quality_values(feature).get(dimension)) is not None
            ]
            metrics = _error_metrics(bucket_features)
            rows.append(
                {
                    "dimension": dimension,
                    "description": QUALITY_BUCKETS[dimension][0]
                    if dimension in QUALITY_BUCKETS
                    else dimension,
                    "bucket": label,
                    "epoch_count": len(bucket_features),
                    "value_median": _median(values),
                    "value_min": min(values) if values else None,
                    "value_max": max(values) if values else None,
                    **metrics,
                }
            )
    return rows


def _summary_metrics(summary: dict[str, Any]) -> dict[str, Any]:
    if summary.get("schema_version") != "smartphone-gnss-signoff.v1":
        raise ValueError("sign-off summary schema is not smartphone-gnss-signoff.v1")
    metrics = summary.get("metrics")
    if not isinstance(metrics, dict):
        raise ValueError("sign-off summary has no metrics object")
    return {
        key: metrics.get(key)
        for key in (
            "selected_epochs",
            "solution_epochs",
            "truth_matched_epochs",
            "solution_availability_ratio",
            "truth_scored_coverage_ratio",
            "horizontal_median_m",
            "horizontal_p95_m",
            "vertical_p95_abs_m",
            "max_solution_gap_s",
        )
    }


def _verify_recomputed_metrics(
    signoff_metrics: dict[str, Any], recomputed: dict[str, Any]
) -> None:
    pairs = {
        "selected_epochs": ("selected_epochs", "epoch_count"),
        "solution_epochs": ("solution_epochs", "solution_epoch_count"),
        "truth_matched_epochs": ("truth_matched_epochs", "solution_epoch_count"),
        "solution_availability_ratio": (
            "solution_availability_ratio",
            "availability_ratio",
        ),
        "truth_scored_coverage_ratio": (
            "truth_scored_coverage_ratio",
            "availability_ratio",
        ),
        "horizontal_median_m": ("horizontal_median_m", "horizontal_median_m"),
        "horizontal_p95_m": ("horizontal_p95_m", "horizontal_p95_m"),
        "vertical_p95_abs_m": ("vertical_p95_abs_m", "vertical_p95_abs_m"),
        "max_solution_gap_s": ("max_solution_gap_s", "max_solution_gap_s"),
    }
    for label, (signoff_key, recomputed_key) in pairs.items():
        expected = signoff_metrics.get(signoff_key)
        actual = recomputed.get(recomputed_key)
        if expected is None or actual is None:
            raise ValueError(f"sign-off metric {label} is missing")
        if isinstance(expected, int) and isinstance(actual, int):
            equal = expected == actual
        else:
            equal = math.isclose(float(expected), float(actual), rel_tol=1e-9, abs_tol=1e-9)
        if not equal:
            raise ValueError(
                f"matches CSV does not reproduce sign-off metric {label}: "
                f"{actual!r} != {expected!r}"
            )


def _candidate_comparison(
    baseline: dict[str, Any], candidate: dict[str, Any]
) -> dict[str, Any]:
    lower_is_better = (
        "horizontal_median_m",
        "horizontal_p95_m",
        "vertical_p95_abs_m",
        "max_solution_gap_s",
    )
    higher_is_better = ("solution_availability_ratio", "truth_scored_coverage_ratio")
    deltas: dict[str, float | None] = {}
    non_regressed = True
    strictly_better = False
    for key in lower_is_better:
        before = baseline.get(key)
        after = candidate.get(key)
        if before is None or after is None:
            deltas[key] = None
            continue
        delta = float(after) - float(before)
        deltas[key] = delta
        if delta > 1e-12:
            non_regressed = False
        if delta < -1e-12:
            strictly_better = True
    for key in higher_is_better:
        before = baseline.get(key)
        after = candidate.get(key)
        if before is None or after is None:
            deltas[key] = None
            continue
        delta = float(after) - float(before)
        deltas[key] = delta
        if delta < -1e-12:
            non_regressed = False
        if delta > 1e-12:
            strictly_better = True
    return {
        "deltas_candidate_minus_baseline": deltas,
        "non_regressed_on_signoff_metrics": non_regressed,
        "strictly_better_on_at_least_one_metric": strictly_better,
        "promotion_decision": "promote"
        if non_regressed and strictly_better
        else "not-promoted",
    }


def build_report(
    device_path: Path,
    adapter_summary_path: Path,
    signoff_summary_path: Path,
    matches_path: Path,
    *,
    segment_seconds: float = DEFAULT_SEGMENT_SECONDS,
    match_tolerance_ms: float = DEFAULT_MATCH_TOLERANCE_MS,
    candidate_signoff_summary_path: Path | None = None,
    candidate_label: str | None = None,
    candidate_snr_reference_dbhz: float | None = None,
) -> tuple[dict[str, Any], list[dict[str, Any]], list[dict[str, Any]]]:
    if not math.isfinite(segment_seconds) or segment_seconds <= 0.0:
        raise ValueError("segment_seconds must be a finite positive number")
    if not math.isfinite(match_tolerance_ms) or match_tolerance_ms <= 0.0:
        raise ValueError("match_tolerance_ms must be a finite positive number")

    adapter = _load_json(adapter_summary_path, "adapter summary")
    if adapter.get("schema_version") != "smartphone-gnss-adapter.v1":
        raise ValueError("adapter summary schema is not smartphone-gnss-adapter.v1")
    adapter_inputs = adapter.get("inputs")
    if not isinstance(adapter_inputs, dict):
        raise ValueError("adapter summary has no inputs object")
    expected_device_hash = dict(adapter_inputs.get("device_gnss", {})).get("sha256")
    if not isinstance(expected_device_hash, str):
        raise ValueError("adapter summary has no device_gnss SHA-256")
    actual_device_hash = _sha256(device_path)
    if actual_device_hash != expected_device_hash:
        raise ValueError("device GNSS hash does not match adapter summary")

    observations = adapter.get("observations")
    if not isinstance(observations, dict):
        raise ValueError("adapter summary has no observations object")
    try:
        skip_epochs = int(observations["explicitly_skipped_epochs"])
        selected_epoch_count = int(observations["epochs"])
        input_epoch_count_expected = int(observations["input_epochs"])
        first_timestamp_expected = int(observations["first_utc_time_millis"])
        last_timestamp_expected = int(observations["last_utc_time_millis"])
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError(f"adapter summary observations are incomplete: {exc}") from exc
    if skip_epochs < 0 or selected_epoch_count <= 0:
        raise ValueError("adapter summary has invalid epoch selection")

    features: list[EpochFeature] = []
    previous_timestamp: int | None = None
    selected_count = 0
    input_epoch_count = 0
    for epoch_index, timestamp, rows in _iter_device_epochs(device_path):
        input_epoch_count += 1
        selected = epoch_index >= skip_epochs and selected_count < selected_epoch_count
        if selected:
            feature = _feature(epoch_index, timestamp, rows, previous_timestamp)
            features.append(feature)
            selected_count += 1
            previous_timestamp = timestamp
    if input_epoch_count != input_epoch_count_expected:
        raise ValueError(
            f"device input epoch count {input_epoch_count} does not match adapter summary "
            f"{input_epoch_count_expected}"
        )
    if selected_count != selected_epoch_count or not features:
        raise ValueError("device rows do not reproduce adapter selected epoch count")
    if features[0].timestamp != first_timestamp_expected or features[-1].timestamp != last_timestamp_expected:
        raise ValueError("device rows do not reproduce adapter selected timestamp range")

    matches = _read_matches(matches_path)
    _attach_matches(features, matches, match_tolerance_ms)
    signoff = _load_json(signoff_summary_path, "sign-off summary")
    baseline_metrics = _summary_metrics(signoff)
    if baseline_metrics.get("selected_epochs") != selected_epoch_count:
        raise ValueError("sign-off selected epoch count does not match adapter summary")
    recomputed_metrics = _error_metrics(features)
    _verify_recomputed_metrics(baseline_metrics, recomputed_metrics)

    segments = _segment_rows(features, segment_seconds)
    buckets = _bucket_rows(features)
    signal_counts: Counter[str] = Counter()
    adr_counts: Counter[str] = Counter()
    clock_counts: Counter[str] = Counter()
    for feature in features:
        signal_counts.update(feature.unsupported_signal_counts)
        adr_counts.update(str(state) for state in feature.adr_states)
        clock_counts[str(feature.hardware_clock_discontinuity_count)] += 1
    quality_distributions = {
        key: _quality_distribution(features, key)
        for key in (
            "median_cn0_dbhz",
            "median_raw_pseudorange_uncertainty_m",
            "median_received_sv_time_uncertainty_ns",
            "adr_valid_fraction",
            "unsupported_fraction",
            "epoch_gap_s",
            "solution_gap_s",
        )
    }
    input_gaps = [
        feature.epoch_gap_s
        for feature in features
        if feature.epoch_gap_s is not None
    ]
    report: dict[str, Any] = {
        "schema_version": SCHEMA_VERSION,
        "analysis_contract": {
            "role": "development-only quality/error audit",
            "raw_quality_features_are_from_device_csv": True,
            "truth_error_is_used_only_for_posthoc_join": True,
            "bucket_boundaries_are_fixed_before_error_join": True,
            "selected_signal_for_quality_values": "GPS_L1_CA rows with raw pseudorange and arrival time",
            "unsupported_signal_policy": "count every selected row whose SignalType is not GPS_L1_CA; preserve exact names",
            "adr_state_bucket_definition": "bit0=valid; any bit1 or bit2=reset_or_cycle_slip_present; otherwise all/any/no bit0 map to valid_no_reset_or_slip/mixed_validity/invalid_no_valid_bit",
            "epoch_gap_definition": "difference between consecutive selected device utcTimeMillis epochs",
            "solution_gap_definition": "difference between consecutive sign-off solution timestamps; no truth error is used",
            "segment_definition": "fixed wall-clock windows from first selected device epoch",
            "position_error_source": "existing smartphone-gnss-signoff matches.csv",
        },
        "inputs": {
            "device_gnss": {"path": str(device_path), "sha256": actual_device_hash},
            "adapter_summary": {
                "path": str(adapter_summary_path),
                "sha256": _sha256(adapter_summary_path),
            },
            "signoff_summary": {
                "path": str(signoff_summary_path),
                "sha256": _sha256(signoff_summary_path),
            },
            "matches": {"path": str(matches_path), "sha256": _sha256(matches_path)},
        },
        "selection": {
            "skip_epochs": skip_epochs,
            "selected_epochs": selected_epoch_count,
            "input_epochs": input_epoch_count,
            "first_utc_time_millis": features[0].timestamp,
            "last_utc_time_millis": features[-1].timestamp,
        },
        "match_join": {
            "matches_rows": len(matches),
            "mapped_device_epochs": sum(feature.match is not None for feature in features),
            "tolerance_ms": match_tolerance_ms,
            "selection_key": "nearest selected device utcTimeMillis to solution_unix_time_millis; ties choose earlier epoch",
        },
        "baseline_signoff": {
            "decision": signoff.get("decision"),
            "metrics": baseline_metrics,
            "recomputed_from_matches": recomputed_metrics,
        },
        "global_quality": {
            "selected_rows": sum(feature.row_count for feature in features),
            "solver_rows": sum(feature.solver_rows for feature in features),
            "unsupported_signal_rows": sum(feature.unsupported_rows for feature in features),
            "unsupported_signal_fraction": (
                sum(feature.unsupported_rows for feature in features)
                / sum(feature.row_count for feature in features)
            ),
            "unsupported_signal_counts": dict(sorted(signal_counts.items())),
            "hardware_clock_discontinuity_count_epochs": dict(sorted(clock_counts.items())),
            "adr_state_rows": dict(sorted(adr_counts.items())),
            "quality_distributions": quality_distributions,
            "input_epoch_gap_s": {
                **_distribution([float(value) for value in input_gaps]),
                "max": max(input_gaps) if input_gaps else 0.0,
            },
            "matched_solution_epochs": sum(feature.match is not None for feature in features),
        },
        "quality_bucket_contract": {
            key: {
                "description": description,
                "bins": [
                    {"lower": lower, "upper": upper, "label": label}
                    for lower, upper, label in bins
                ],
            }
            for key, (description, bins) in QUALITY_BUCKETS.items()
        },
        "segments": {
            "segment_seconds": segment_seconds,
            "rows": segments,
        },
        "quality_buckets": buckets,
    }
    if candidate_signoff_summary_path is not None:
        candidate_summary = _load_json(
            candidate_signoff_summary_path, "candidate sign-off summary"
        )
        candidate_metrics = _summary_metrics(candidate_summary)
        if candidate_metrics.get("selected_epochs") != selected_epoch_count:
            raise ValueError("candidate selected epoch count does not match adapter summary")
        report["candidate_experiment"] = {
            "label": candidate_label or "existing SPP C/N0 variance model",
            "solver_option": (
                f"--snr-reference-dbhz {candidate_snr_reference_dbhz:g}"
                if candidate_snr_reference_dbhz is not None
                else None
            ),
            "candidate_signoff": {
                "path": str(candidate_signoff_summary_path),
                "sha256": _sha256(candidate_signoff_summary_path),
                "decision": candidate_summary.get("decision"),
                "metrics": candidate_metrics,
            },
            "comparison": _candidate_comparison(baseline_metrics, candidate_metrics),
            "data_leakage_guard": "candidate option is fixed; no truth/error field is read by the solver preprocessing",
        }
    return report, segments, buckets


def _write_rows(path: Path, rows: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        path.write_text("\n", encoding="utf-8")
        return
    keys = list(rows[0])
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=keys, lineterminator="\n")
        writer.writeheader()
        for row in rows:
            serialized = {
                key: json.dumps(value, sort_keys=True, separators=(",", ":"))
                if isinstance(value, (dict, list))
                else value
                for key, value in row.items()
            }
            writer.writerow(serialized)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(prog=os.environ.get("GNSS_CLI_NAME"))
    parser.add_argument("--device-gnss", type=Path, required=True)
    parser.add_argument("--adapter-summary", type=Path, required=True)
    parser.add_argument("--signoff-summary", type=Path, required=True)
    parser.add_argument("--matches", type=Path, required=True)
    parser.add_argument("--output-json", type=Path, required=True)
    parser.add_argument("--output-segments-csv", type=Path, required=True)
    parser.add_argument("--output-buckets-csv", type=Path, required=True)
    parser.add_argument("--segment-s", type=float, default=DEFAULT_SEGMENT_SECONDS)
    parser.add_argument("--match-tolerance-ms", type=float, default=DEFAULT_MATCH_TOLERANCE_MS)
    parser.add_argument("--candidate-signoff-summary", type=Path)
    parser.add_argument("--candidate-label")
    parser.add_argument("--candidate-snr-reference-dbhz", type=float)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        report, segments, buckets = build_report(
            args.device_gnss,
            args.adapter_summary,
            args.signoff_summary,
            args.matches,
            segment_seconds=args.segment_s,
            match_tolerance_ms=args.match_tolerance_ms,
            candidate_signoff_summary_path=args.candidate_signoff_summary,
            candidate_label=args.candidate_label,
            candidate_snr_reference_dbhz=args.candidate_snr_reference_dbhz,
        )
        args.output_json.parent.mkdir(parents=True, exist_ok=True)
        args.output_json.write_text(
            json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
        _write_rows(args.output_segments_csv, segments)
        _write_rows(args.output_buckets_csv, buckets)
    except (OSError, ValueError) as exc:
        raise SystemExit(str(exc)) from exc
    print(f"Smartphone GNSS quality report: {args.output_json}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
