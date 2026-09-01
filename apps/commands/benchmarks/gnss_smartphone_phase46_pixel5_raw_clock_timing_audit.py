#!/usr/bin/env python3
"""Audit Android raw receiver-clock timing for the sealed Phase45 Pixel5 cohort.

This is an audit-only command.  It consumes only the four pinned raw
``device_gnss.csv`` files, opens each exactly once in one process, and never
runs a solver or constructs a trajectory.  The exact receiver-clock
intermediate is kept as integer ``TimeNanos - FullBiasNanos`` plus Decimal
``BiasNanos`` so that loss caused by a float64 subtraction of large nanosecond
values is measured rather than hidden.

The command must be run only after the evaluator manifest has been committed
and pushed.  Its output is published atomically as route, event, result, and
hash-manifest tables.  No truth, archive, validation, holdout, MAT, WLS, or
precomputed-coordinate path is accepted by the input guard.
"""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
from decimal import Decimal, InvalidOperation
import hashlib
import io
import json
import math
import os
from pathlib import Path
import statistics
import sys
import tempfile
from typing import Any, Iterable, Sequence


ROOT = Path(__file__).resolve().parents[3]
FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_phase46_pixel5_raw_clock_timing_freeze_v1.json"
FREEZE_SHA256 = "b847826017ae7683d9fa91de7225301af682d778add210acd6f04fcce52e5196"
MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase46_pixel5_raw_clock_timing_evaluator_manifest_v1.json"
MANIFEST_SHA256 = ""
VERIFIED_MANIFEST_SHA256 = ""
DEFAULT_OUTPUT = ROOT / "output/smartphone-r5/phase46-pixel5-raw-clock-timing-v1"

SCHEMA = "smartphone-r5-phase46-pixel5-raw-clock-timing.v1"
MANIFEST_SCHEMA = "smartphone-r5-phase46-pixel5-raw-clock-timing-manifest.v1"
FREEZE_SCHEMA = "smartphone-r5-phase46-pixel5-raw-clock-timing-freeze.v1"

ROUTES = (
    "2021-03-16-18-59-us-ca-mtv-a/pixel5",
    "2021-08-24-20-32-us-ca-mtv-h/pixel5",
    "2022-04-01-18-22-us-ca-lax-t/pixel5",
    "2023-03-08-21-34-us-ca-mtv-u/pixel5",
)

GPS_EPOCH_UNIX_MS = 315_964_800_000
GPS_UTC_LEAP_SECONDS = 18
SPEED_OF_LIGHT_MPS = 299_792_458.0
NS_PER_MS = Decimal("1000000")
NS_PER_SECOND = Decimal("1000000000")
MAX_RESIDUAL_MS = 2.0
MAX_DRIFT_PPM = 1000.0
MAX_NONCOMMON_TOW_M = 10.0
TIME_GAP_NS = 1_000_000_000
STEP_THRESHOLD_MS = 2.0

REQUIRED_COLUMNS = (
    "MessageType",
    "utcTimeMillis",
    "TimeNanos",
    "FullBiasNanos",
    "BiasNanos",
    "HardwareClockDiscontinuityCount",
)
OPTIONAL_NUMERIC_COLUMNS = (
    "DriftNanosPerSecond",
    "BiasUncertaintyNanos",
    "DriftUncertaintyNanosPerSecond",
    "TimeUncertaintyNanos",
    "TimeOffsetNanos",
    "ReceivedSvTimeNanos",
    "CarrierFrequencyHz",
)
OPTIONAL_TEXT_COLUMNS = ("State", "ConstellationType", "Svid", "SignalType")


class Phase46Error(ValueError):
    """Raised when an immutable Phase46 audit contract cannot be proved."""


def _fail(message: str) -> Phase46Error:
    return Phase46Error(message)


def _relative(path: Path) -> str:
    try:
        return str(path.relative_to(ROOT))
    except ValueError:
        return str(path)


def _reject_path(path: Path | str) -> None:
    """Reject paths that could broaden this raw-only audit's scope."""

    lowered = str(path).lower()
    forbidden = (
        ".mat",
        "ground_truth",
        "validation",
        "holdout",
        "precomputed",
        "device_wls",
        "kaggle",
        "token",
        "archive",
    )
    if any(token in lowered for token in forbidden):
        raise _fail(f"forbidden Phase46 input/output path: {path}")


def _sha256_bytes(payload: bytes) -> str:
    return hashlib.sha256(payload).hexdigest()


def _read_bytes_once(path: Path, label: str, expected_sha256: str | None = None) -> tuple[bytes, str]:
    """Read one file into memory exactly once and hash that same byte buffer."""

    _reject_path(path)
    if not path.is_file():
        raise _fail(f"missing {label}: {path}")
    try:
        payload = path.read_bytes()
    except OSError as exc:
        raise _fail(f"failed to read {label}: {path}: {exc}") from exc
    digest = _sha256_bytes(payload)
    if expected_sha256 is not None and digest != expected_sha256:
        raise _fail(f"{label} hash mismatch: {digest} != {expected_sha256}")
    return payload, digest


def _json_bytes(payload: bytes, label: str) -> dict[str, Any]:
    try:
        value = json.loads(payload.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise _fail(f"invalid {label}") from exc
    if not isinstance(value, dict):
        raise _fail(f"{label} must be a JSON object")
    return value


def _load_json_once(path: Path, label: str, expected_sha256: str | None = None) -> tuple[dict[str, Any], str, int]:
    payload, digest = _read_bytes_once(path, label, expected_sha256)
    return _json_bytes(payload, label), digest, len(payload)


def _atomic_write(path: Path, payload: bytes) -> None:
    _reject_path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary = tempfile.mkstemp(prefix=f".{path.name}.", suffix=".tmp", dir=str(path.parent))
    try:
        with os.fdopen(descriptor, "wb") as handle:
            descriptor = -1
            handle.write(payload)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, path)
        temporary = ""
    finally:
        if descriptor >= 0:
            os.close(descriptor)
        if temporary:
            try:
                os.unlink(temporary)
            except FileNotFoundError:
                pass


def _json_bytes_for_output(value: Any) -> bytes:
    return (json.dumps(value, indent=2, sort_keys=True, allow_nan=False) + "\n").encode("utf-8")


def _atomic_json(path: Path, value: Any) -> bytes:
    payload = _json_bytes_for_output(value)
    _atomic_write(path, payload)
    return payload


def _parse_int(value: Any, label: str) -> int:
    text = str(value).strip()
    if not text:
        raise _fail(f"empty integer {label}")
    try:
        decimal = Decimal(text)
    except InvalidOperation as exc:
        raise _fail(f"invalid integer {label}: {text!r}") from exc
    if not decimal.is_finite() or decimal != decimal.to_integral_value():
        raise _fail(f"non-integral integer {label}: {text!r}")
    return int(decimal)


def _parse_decimal(value: Any, label: str, required: bool = True) -> Decimal | None:
    text = str(value).strip()
    if not text:
        if required:
            raise _fail(f"empty numeric {label}")
        return None
    try:
        decimal = Decimal(text)
    except InvalidOperation as exc:
        raise _fail(f"invalid numeric {label}: {text!r}") from exc
    if not decimal.is_finite():
        raise _fail(f"non-finite numeric {label}: {text!r}")
    return decimal


def _decimal_float(value: Decimal) -> float:
    result = float(value)
    if not math.isfinite(result):
        raise _fail("Decimal-to-float conversion became non-finite")
    return result


@dataclass(slots=True)
class RawRow:
    row_number: int
    utc_ms: int
    time_ns: int
    full_bias_ns: int
    bias_ns: Decimal
    hcdc: int
    drift_nsps: Decimal | None
    bias_uncertainty_ns: Decimal | None
    drift_uncertainty_nsps: Decimal | None
    time_uncertainty_ns: Decimal | None
    time_offset_ns: Decimal | None
    received_sv_time_ns: Decimal | None
    state: str
    constellation: str
    svid: str
    signal: str
    carrier_hz: Decimal | None

    @property
    def integer_hardware_ns(self) -> int:
        return self.time_ns - self.full_bias_ns

    @property
    def exact_hardware_gps_ns(self) -> Decimal:
        return Decimal(self.integer_hardware_ns) - self.bias_ns

    @property
    def utc_gps_residual_ms(self) -> Decimal:
        return (
            self.exact_hardware_gps_ns / NS_PER_MS
            + Decimal(GPS_EPOCH_UNIX_MS)
            - Decimal(GPS_UTC_LEAP_SECONDS * 1000)
            - Decimal(self.utc_ms)
        )


@dataclass(slots=True)
class Epoch:
    key_ms: int
    rows: list[RawRow]

    @property
    def first(self) -> RawRow:
        return self.rows[0]

    @property
    def hardware_values_ns(self) -> list[Decimal]:
        return [row.exact_hardware_gps_ns for row in self.rows]

    @property
    def residuals_ms(self) -> list[float]:
        return [_decimal_float(row.utc_gps_residual_ms) for row in self.rows]

    @property
    def median_hardware_ns(self) -> Decimal:
        return _median_decimal(self.hardware_values_ns)

    @property
    def median_residual_ms(self) -> float:
        return _median(self.residuals_ms)

    @property
    def time_nanos_median(self) -> float:
        return _median([float(row.time_ns) for row in self.rows])

    @property
    def time_nanos_median_exact(self) -> Decimal:
        return _median_decimal([Decimal(row.time_ns) for row in self.rows])

    @property
    def full_bias_values(self) -> set[int]:
        return {row.full_bias_ns for row in self.rows}

    @property
    def hcdc_values(self) -> set[int]:
        return {row.hcdc for row in self.rows}


def _median(values: Iterable[float]) -> float:
    values_list = [float(value) for value in values]
    if not values_list:
        raise _fail("median of empty sequence")
    result = float(statistics.median(values_list))
    if not math.isfinite(result):
        raise _fail("non-finite median")
    return result


def _median_decimal(values: Iterable[Decimal]) -> Decimal:
    values_list = sorted(values)
    if not values_list:
        raise _fail("Decimal median of empty sequence")
    middle = len(values_list) // 2
    if len(values_list) % 2:
        return values_list[middle]
    return (values_list[middle - 1] + values_list[middle]) / Decimal(2)


def _mad(values: Iterable[float], center: float | None = None) -> float:
    values_list = [float(value) for value in values]
    if not values_list:
        return 0.0
    if center is None:
        center = _median(values_list)
    return _median(abs(value - center) for value in values_list)


def _percentile(values: Iterable[float], percentile: float) -> float:
    ordered = sorted(float(value) for value in values)
    if not ordered:
        return 0.0
    if not 0.0 <= percentile <= 1.0:
        raise _fail(f"invalid percentile {percentile}")
    if len(ordered) == 1:
        return ordered[0]
    rank = percentile * (len(ordered) - 1)
    lower = int(math.floor(rank))
    upper = int(math.ceil(rank))
    if lower == upper:
        return ordered[lower]
    fraction = rank - lower
    return ordered[lower] + fraction * (ordered[upper] - ordered[lower])


def _distribution(values: Iterable[float], center: float | None = None) -> dict[str, float | int]:
    values_list = [float(value) for value in values]
    if not values_list:
        return {"count": 0, "median": 0.0, "mad": 0.0, "p50_abs": 0.0, "p95_abs": 0.0, "max_abs": 0.0}
    if center is None:
        center = _median(values_list)
    absolute = [abs(value - center) for value in values_list]
    return {
        "count": len(values_list),
        "median": _median(values_list),
        "mad": _mad(values_list, center),
        "p50_abs": _percentile(absolute, 0.50),
        "p95_abs": _percentile(absolute, 0.95),
        "max_abs": max(absolute),
    }


def _fit_affine(times_ms: Sequence[float], values_ms: Sequence[float]) -> tuple[float, float, list[float]]:
    """Fit values = intercept + slope * elapsed_ms and return residuals."""

    if len(times_ms) != len(values_ms) or not times_ms:
        raise _fail("affine fit requires paired non-empty values")
    origin = float(times_ms[0])
    x = [float(value) - origin for value in times_ms]
    y = [float(value) for value in values_ms]
    x_mean = statistics.fmean(x)
    y_mean = statistics.fmean(y)
    denominator = sum((value - x_mean) ** 2 for value in x)
    slope = 0.0 if denominator == 0.0 else sum((a - x_mean) * (b - y_mean) for a, b in zip(x, y)) / denominator
    intercept = y_mean - slope * x_mean
    residuals = [value - (intercept + slope * at) for at, value in zip(x, y)]
    if not all(math.isfinite(value) for value in residuals):
        raise _fail("non-finite affine residual")
    return slope, intercept, residuals


def _parse_raw_payload(payload: bytes) -> tuple[list[Epoch], dict[str, Any]]:
    """Parse a raw CSV byte buffer exactly once into first-seen UTC epochs."""

    try:
        text = payload.decode("utf-8-sig")
    except UnicodeDecodeError as exc:
        raise _fail("device_gnss.csv is not UTF-8") from exc
    reader = csv.DictReader(io.StringIO(text, newline=""))
    if reader.fieldnames is None:
        raise _fail("device_gnss.csv has no header")
    fieldnames = [name.strip() for name in reader.fieldnames]
    missing = [name for name in REQUIRED_COLUMNS if name not in fieldnames]
    if missing:
        raise _fail(f"missing required raw columns: {missing}")

    epochs: list[Epoch] = []
    current: Epoch | None = None
    first_seen_keys: set[int] = set()
    non_raw_rows = 0
    raw_rows = 0
    optional_nonempty_counts = {name: 0 for name in OPTIONAL_NUMERIC_COLUMNS + OPTIONAL_TEXT_COLUMNS}
    optional_numeric_nonfinite = 0
    for row_number, raw in enumerate(reader, start=2):
        message_type = str(raw.get("MessageType", "")).strip()
        if message_type != "Raw":
            non_raw_rows += 1
            continue
        raw_rows += 1
        utc_ms = _parse_int(raw.get("utcTimeMillis", ""), f"utcTimeMillis row {row_number}")
        time_ns = _parse_int(raw.get("TimeNanos", ""), f"TimeNanos row {row_number}")
        full_bias_ns = _parse_int(raw.get("FullBiasNanos", ""), f"FullBiasNanos row {row_number}")
        bias_ns = _parse_decimal(raw.get("BiasNanos", ""), f"BiasNanos row {row_number}")
        assert bias_ns is not None
        hcdc = _parse_int(raw.get("HardwareClockDiscontinuityCount", ""), f"HCDC row {row_number}")

        def optional_decimal(name: str) -> Decimal | None:
            value = raw.get(name, "")
            if str(value).strip():
                optional_nonempty_counts[name] += 1
            return _parse_decimal(value, f"{name} row {row_number}", required=False)

        optional_text: dict[str, str] = {}
        for name in OPTIONAL_TEXT_COLUMNS:
            value = str(raw.get(name, "")).strip()
            if value:
                optional_nonempty_counts[name] += 1
            optional_text[name] = value

        parsed = RawRow(
            row_number=row_number,
            utc_ms=utc_ms,
            time_ns=time_ns,
            full_bias_ns=full_bias_ns,
            bias_ns=bias_ns,
            hcdc=hcdc,
            drift_nsps=optional_decimal("DriftNanosPerSecond"),
            bias_uncertainty_ns=optional_decimal("BiasUncertaintyNanos"),
            drift_uncertainty_nsps=optional_decimal("DriftUncertaintyNanosPerSecond"),
            time_uncertainty_ns=optional_decimal("TimeUncertaintyNanos"),
            time_offset_ns=optional_decimal("TimeOffsetNanos"),
            received_sv_time_ns=optional_decimal("ReceivedSvTimeNanos"),
            state=optional_text["State"],
            constellation=optional_text["ConstellationType"],
            svid=optional_text["Svid"],
            signal=optional_text["SignalType"],
            carrier_hz=optional_decimal("CarrierFrequencyHz"),
        )
        if current is None or utc_ms != current.key_ms:
            if current is not None:
                epochs.append(current)
            current = Epoch(key_ms=utc_ms, rows=[])
            if utc_ms in first_seen_keys:
                # The key reappeared after a later key; retain the epoch so the
                # caller can fail the monotonic/duplicate gate explicitly.
                pass
            first_seen_keys.add(utc_ms)
        assert current is not None
        current.rows.append(parsed)
    if current is not None:
        epochs.append(current)
    if raw_rows == 0:
        raise _fail("no MessageType=Raw rows")

    repeated_key_count = 0
    nonmonotonic_key_count = 0
    previous: int | None = None
    seen: set[int] = set()
    for epoch in epochs:
        if previous is not None:
            if epoch.key_ms < previous:
                nonmonotonic_key_count += 1
            if epoch.key_ms in seen:
                repeated_key_count += 1
        seen.add(epoch.key_ms)
        previous = epoch.key_ms
    positive_utc_gaps = [
        epoch.key_ms - previous_epoch.key_ms
        for previous_epoch, epoch in zip(epochs, epochs[1:])
        if epoch.key_ms > previous_epoch.key_ms
    ]
    nominal_utc_gap = _median(positive_utc_gaps) if positive_utc_gaps else 0.0
    inferred_missing_epoch_count = 0
    if nominal_utc_gap > 0.0:
        for previous_epoch, epoch in zip(epochs, epochs[1:]):
            gap = epoch.key_ms - previous_epoch.key_ms
            if gap > nominal_utc_gap * 1.5:
                inferred_missing_epoch_count += max(0, int(round(gap / nominal_utc_gap)) - 1)
    metadata = {
        "header_columns": fieldnames,
        "raw_rows": raw_rows,
        "non_raw_rows": non_raw_rows,
        "epoch_count": len(epochs),
        "repeated_epoch_key_count": repeated_key_count,
        "nonmonotonic_epoch_key_count": nonmonotonic_key_count,
        "nominal_utc_gap_ms": nominal_utc_gap,
        "inferred_missing_epoch_count": inferred_missing_epoch_count,
        "optional_nonempty_counts": optional_nonempty_counts,
        "optional_numeric_nonfinite_count": optional_numeric_nonfinite,
    }
    return epochs, metadata


def _quantiles(values: Sequence[float]) -> dict[str, float]:
    return {
        "q25": _percentile(values, 0.25),
        "q50": _percentile(values, 0.50),
        "q75": _percentile(values, 0.75),
    }


def _optional_distribution(epochs: Sequence[Epoch], attr: str) -> dict[str, float | int]:
    values: list[float] = []
    for epoch in epochs:
        for row in epoch.rows:
            value = getattr(row, attr)
            if value is not None:
                values.append(_decimal_float(value))
    if not values:
        return {"count": 0, "median": 0.0, "mad": 0.0, "p50_abs": 0.0, "p95_abs": 0.0, "max_abs": 0.0}
    return _distribution(values, 0.0)


def _clock_field_spreads(epochs: Sequence[Epoch]) -> dict[str, Any]:
    fields: dict[str, list[Decimal]] = {
        "TimeNanos": [],
        "FullBiasNanos": [],
        "BiasNanos": [],
        "TimeOffsetNanos": [],
        "DriftNanosPerSecond": [],
    }
    inconsistencies: list[dict[str, Any]] = []
    for epoch in epochs:
        values: dict[str, list[Decimal]] = {
            "TimeNanos": [Decimal(row.time_ns) for row in epoch.rows],
            "FullBiasNanos": [Decimal(row.full_bias_ns) for row in epoch.rows],
            "BiasNanos": [row.bias_ns for row in epoch.rows],
            "TimeOffsetNanos": [row.time_offset_ns for row in epoch.rows if row.time_offset_ns is not None],
            "DriftNanosPerSecond": [row.drift_nsps for row in epoch.rows if row.drift_nsps is not None],
        }
        disagreement: dict[str, float] = {}
        for field, field_values in values.items():
            if not field_values:
                continue
            spread = max(field_values) - min(field_values)
            fields[field].append(spread)
            if spread != 0:
                disagreement[field] = _decimal_float(spread)
        if disagreement:
            inconsistencies.append({"utcTimeMillis": epoch.key_ms, "spreads": disagreement})
    return {
        "per_epoch_spread_ns": {
            name: {
                "count": len(values),
                "median": _decimal_float(_median_decimal(values)) if values else 0.0,
                "p95": _percentile([_decimal_float(value) for value in values], 0.95) if values else 0.0,
                "max": _decimal_float(max(values)) if values else 0.0,
            }
            for name, values in fields.items()
        },
        "same_epoch_inconsistency_count": len(inconsistencies),
        "same_epoch_inconsistency_examples": inconsistencies[:20],
    }


def _segment_and_events(epochs: Sequence[Epoch]) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
    """Assign Phase25 segments and collect discontinuity/jump events."""

    if not epochs:
        raise _fail("cannot segment empty route")
    assignments: list[dict[str, Any]] = []
    events: list[dict[str, Any]] = []
    segment_index = -1
    active_base_full_bias: int | None = None
    previous_epoch: Epoch | None = None
    previous_hcdc: int | None = None
    previous_hardware_ns: Decimal | None = None
    previous_utc_ms: int | None = None
    for epoch_index, epoch in enumerate(epochs):
        hcdc = epoch.first.hcdc
        time_gap_ns: float | None = None
        if previous_epoch is not None:
            time_gap_ns = _decimal_float(epoch.time_nanos_median_exact - previous_epoch.time_nanos_median_exact)
        hcdc_change = previous_hcdc is not None and hcdc != previous_hcdc
        time_gap_boundary = time_gap_ns is not None and time_gap_ns > TIME_GAP_NS
        explicit_boundary = segment_index < 0 or hcdc_change or time_gap_boundary
        if explicit_boundary:
            segment_index += 1
            active_base_full_bias = epoch.first.full_bias_ns
            if hcdc_change:
                events.append({
                    "kind": "hardware_clock_discontinuity",
                    "utcTimeMillis": epoch.key_ms,
                    "epoch_index": epoch_index,
                    "previous_hcdc": previous_hcdc,
                    "hcdc": hcdc,
                    "captured_by_explicit_boundary": True,
                })
            if time_gap_boundary:
                events.append({
                    "kind": "time_nanos_gap_gt_1s",
                    "utcTimeMillis": epoch.key_ms,
                    "epoch_index": epoch_index,
                    "gap_ns": time_gap_ns,
                    "captured_by_explicit_boundary": True,
                })
        assert active_base_full_bias is not None
        current_full_bias_changes = len(epoch.full_bias_values) > 1
        previous_full_bias = previous_epoch.first.full_bias_ns if previous_epoch is not None else epoch.first.full_bias_ns
        transitioned_full_bias = previous_epoch is not None and epoch.first.full_bias_ns != previous_full_bias
        if current_full_bias_changes or transitioned_full_bias:
            change_captured = bool(explicit_boundary and not current_full_bias_changes)
            events.append({
                "kind": "full_bias_change",
                "utcTimeMillis": epoch.key_ms,
                "epoch_index": epoch_index,
                "previous_full_bias_ns": previous_full_bias,
                "full_bias_values_ns": sorted(epoch.full_bias_values),
                "captured_by_explicit_boundary": change_captured,
            })
        if len(epoch.hcdc_values) > 1:
            events.append({
                "kind": "same_epoch_hcdc_disagreement",
                "utcTimeMillis": epoch.key_ms,
                "epoch_index": epoch_index,
                "hcdc_values": sorted(epoch.hcdc_values),
                "captured_by_explicit_boundary": False,
            })

        tow_differences_ns: list[float] = []
        exact_risk_ns: list[float] = []
        for row in epoch.rows:
            contract_tow_ns = Decimal(row.time_ns - active_base_full_bias) - row.bias_ns
            row_tow_ns = row.exact_hardware_gps_ns
            tow_differences_ns.append(_decimal_float(contract_tow_ns - row_tow_ns))
            float_eval = float(row.time_ns) - float(row.full_bias_ns) - float(row.bias_ns)
            exact_eval = _decimal_float(row_tow_ns)
            exact_risk_ns.append(abs(float_eval - exact_eval))
        if tow_differences_ns and max(abs(value) for value in tow_differences_ns) > 0.0:
            events.append({
                "kind": "base_vs_row_full_bias_identity_failure",
                "utcTimeMillis": epoch.key_ms,
                "epoch_index": epoch_index,
                "active_segment": segment_index,
                "max_tow_difference_ns": max(abs(value) for value in tow_differences_ns),
                "captured_by_explicit_boundary": bool(explicit_boundary),
            })
        if tow_differences_ns:
            median_tow_difference_ns = _median(tow_differences_ns)
            tow_spread_ns = max(tow_differences_ns) - min(tow_differences_ns)
        else:
            median_tow_difference_ns = 0.0
            tow_spread_ns = 0.0
        if previous_hardware_ns is not None and previous_utc_ms is not None:
            hardware_gap_ns = epoch.median_hardware_ns - previous_hardware_ns
            utc_gap_ns = Decimal(epoch.key_ms - previous_utc_ms) * NS_PER_MS
            interval_mismatch_ns = hardware_gap_ns - utc_gap_ns
        else:
            hardware_gap_ns = None
            utc_gap_ns = None
            interval_mismatch_ns = None
        assignments.append({
            "epoch_index": epoch_index,
            "utcTimeMillis": epoch.key_ms,
            "segment": segment_index,
            "segment_base_FullBiasNanos": active_base_full_bias,
            "full_bias_values_ns": sorted(epoch.full_bias_values),
            "hcdc_values": sorted(epoch.hcdc_values),
            "median_receiver_tow_base_contract_ns": _decimal_float(
                _median_decimal([Decimal(row.time_ns - active_base_full_bias) - row.bias_ns for row in epoch.rows])
            ),
            "median_receiver_tow_row_contract_ns": _decimal_float(epoch.median_hardware_ns),
            "base_vs_row_tow_difference_ns": median_tow_difference_ns,
            "base_vs_row_tow_spread_ns": tow_spread_ns,
            "base_vs_row_tow_effect_m": abs(median_tow_difference_ns) * SPEED_OF_LIGHT_MPS / 1e9,
            "float64_risk_ns_max": max(exact_risk_ns) if exact_risk_ns else 0.0,
            "time_nanos_gap_ns": time_gap_ns,
            "hardware_clock_gap_ns": _decimal_float(hardware_gap_ns) if hardware_gap_ns is not None else None,
            "utc_key_gap_ns": _decimal_float(utc_gap_ns) if utc_gap_ns is not None else None,
            "hardware_utc_interval_difference_ns": _decimal_float(interval_mismatch_ns) if interval_mismatch_ns is not None else None,
            "receiver_tow_spread_ns": _decimal_float(max(epoch.hardware_values_ns) - min(epoch.hardware_values_ns)),
            "explicit_segment_boundary": bool(explicit_boundary),
        })
        if (
            not explicit_boundary
            and assignments[-1]["receiver_tow_spread_ns"] * SPEED_OF_LIGHT_MPS / 1e9 > MAX_NONCOMMON_TOW_M
        ):
            events.append({
                "kind": "uncaptured_receiver_tow_noncommon_mode_jump",
                "utcTimeMillis": epoch.key_ms,
                "epoch_index": epoch_index,
                "noncommon_effect_m": assignments[-1]["receiver_tow_spread_ns"] * SPEED_OF_LIGHT_MPS / 1e9,
                "captured_by_explicit_boundary": False,
            })
        previous_epoch = epoch
        previous_hcdc = hcdc
        previous_hardware_ns = epoch.median_hardware_ns
        previous_utc_ms = epoch.key_ms
    return assignments, events


def _route_summary(route: str, epochs: Sequence[Epoch], parse_metadata: dict[str, Any], file_size: int, digest: str) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    assignments, events = _segment_and_events(epochs)
    epoch_residuals = [epoch.median_residual_ms for epoch in epochs]
    first_hardware_ns = epochs[0].median_hardware_ns
    epoch_times_ms = [
        _decimal_float((epoch.median_hardware_ns - first_hardware_ns) / NS_PER_MS)
        for epoch in epochs
    ]
    center = _median(epoch_residuals)
    centered = [value - center for value in epoch_residuals]
    slope, intercept, detrended = _fit_affine(epoch_times_ms, centered)
    drift_ppm = slope * 1_000_000.0
    abs_centered = [abs(value) for value in centered]
    abs_detrended = [abs(value) for value in detrended]
    if not all(math.isfinite(value) for value in detrended):
        raise _fail(f"non-finite detrended residual for {route}")

    prefix_count = max(1, len(epoch_residuals) // 4)
    tail_count = prefix_count
    prefix = centered[:prefix_count]
    tail = centered[-tail_count:]
    time_drift = {
        "prefix_median_ms": _median(prefix),
        "tail_median_ms": _median(tail),
        "tail_minus_prefix_median_ms": _median(tail) - _median(prefix),
        "prefix_count": len(prefix),
        "tail_count": len(tail),
    }
    quartiles = {
        "centered_ms": _quantiles(centered),
        "detrended_ms": _quantiles(detrended),
    }

    interval_differences = [
        float(item["hardware_utc_interval_difference_ns"]) / 1e6
        for item in assignments
        if item["hardware_utc_interval_difference_ns"] is not None
    ]
    time_gaps = [
        float(item["time_nanos_gap_ns"]) / 1e6
        for item in assignments
        if item["time_nanos_gap_ns"] is not None
    ]
    tow_spreads_m = [float(item["receiver_tow_spread_ns"]) * SPEED_OF_LIGHT_MPS / 1e9 for item in assignments]
    base_row_effect_m = [float(item["base_vs_row_tow_effect_m"]) for item in assignments]
    float_risks = [float(item["float64_risk_ns_max"]) for item in assignments]
    step_deltas = [abs(b - a) for a, b in zip(centered, centered[1:])]
    noncommon_tow_effect_m = [
        float(event["noncommon_effect_m"])
        for event in events
        if event["kind"] == "uncaptured_receiver_tow_noncommon_mode_jump"
    ]
    uncaptured_events = [
        event for event in events
        if event["kind"] in {"full_bias_change", "base_vs_row_full_bias_identity_failure", "same_epoch_hcdc_disagreement"}
        and not event.get("captured_by_explicit_boundary", False)
    ]

    group_values: dict[str, list[float]] = {}
    group_time_offsets: dict[str, list[float]] = {}
    same_epoch_group_spreads: list[float] = []
    for epoch in epochs:
        by_group: dict[str, list[float]] = {}
        for row in epoch.rows:
            group = f"{row.constellation or 'unknown'}|{row.signal or 'unknown'}"
            group_values.setdefault(group, []).append(_decimal_float(row.utc_gps_residual_ms))
            if row.time_offset_ns is not None:
                group_time_offsets.setdefault(group, []).append(_decimal_float(row.time_offset_ns))
            by_group.setdefault(group, []).append(_decimal_float(row.utc_gps_residual_ms))
        if len(by_group) > 1:
            group_medians = [_median(values) for values in by_group.values()]
            same_epoch_group_spreads.append(max(group_medians) - min(group_medians))
    group_report = {
        group: {
            "row_count": len(values),
            "residual_ms": _distribution(values),
            "time_offset_nanos": _optional_distribution_values(group_time_offsets.get(group, [])),
        }
        for group, values in sorted(group_values.items())
    }
    route = {
        "input": {"path": _relative(Path(parse_metadata["path"])), "file_size": file_size, "sha256": digest},
        "rows": {
            "raw": parse_metadata["raw_rows"],
            "non_raw": parse_metadata["non_raw_rows"],
            "epochs": len(epochs),
            "epoch_domain_coverage": 1.0,
            "repeated_epoch_key_count": parse_metadata["repeated_epoch_key_count"],
            "nonmonotonic_epoch_key_count": parse_metadata["nonmonotonic_epoch_key_count"],
            "inferred_missing_epoch_count": parse_metadata["inferred_missing_epoch_count"],
        },
        "clock_residual": {
            "route_median_ms": center,
            "route_mad_ms": _mad(epoch_residuals, center),
            "median_centered_ms": _distribution(centered, 0.0),
            "affine": {
                "slope_ms_per_ms": slope,
                "intercept_ms": intercept,
                "drift_ppm": drift_ppm,
                "detrended_ms": _distribution(detrended, 0.0),
            },
            "step_jump": {
                "adjacent_epoch_count": len(step_deltas),
                "max_abs_step_ms": max(step_deltas) if step_deltas else 0.0,
                "steps_gt_2ms": sum(1 for value in step_deltas if value > STEP_THRESHOLD_MS),
            },
            "prefix_tail_25_percent": time_drift,
            "quartiles": quartiles,
        },
        "intervals": {
            "utc_key_gap_ms": {
                "median": _median([float(item["utc_key_gap_ns"]) / 1e6 for item in assignments[1:]]) if len(assignments) > 1 else 0.0,
                "p95": _percentile([float(item["utc_key_gap_ns"]) / 1e6 for item in assignments[1:]], 0.95) if len(assignments) > 1 else 0.0,
            },
            "hardware_clock_gap_ms": {
                "median": _median([float(item["hardware_clock_gap_ns"]) / 1e6 for item in assignments[1:]]) if len(assignments) > 1 else 0.0,
                "p95": _percentile([float(item["hardware_clock_gap_ns"]) / 1e6 for item in assignments[1:]], 0.95) if len(assignments) > 1 else 0.0,
            },
            "hardware_minus_utc_gap_ms": _distribution(interval_differences, 0.0),
            "time_nanos_gaps_gt_1s": sum(1 for value in time_gaps if value > 1000.0),
            "time_nanos_gap_ms_max": max(time_gaps) if time_gaps else 0.0,
        },
        "segment_contract": {
            "segment_count": (max(item["segment"] for item in assignments) + 1) if assignments else 0,
            "hcdc_change_events": sum(1 for event in events if event["kind"] == "hardware_clock_discontinuity"),
            "full_bias_change_events": sum(1 for event in events if event["kind"] == "full_bias_change"),
            "uncaptured_change_events": len(uncaptured_events),
            "base_vs_row_identity_failures": sum(1 for event in events if event["kind"] == "base_vs_row_full_bias_identity_failure"),
            "receiver_tow_base_vs_row_ns": _distribution(
                [float(item["base_vs_row_tow_difference_ns"]) for item in assignments], 0.0
            ),
            "receiver_tow_spread_m": _distribution(tow_spreads_m, 0.0),
            "base_vs_row_effect_m": _distribution(base_row_effect_m, 0.0),
            "float64_long_decimal_risk_ns": _distribution(float_risks, 0.0),
            "max_uncaptured_noncommon_tow_effect_m": max(noncommon_tow_effect_m) if noncommon_tow_effect_m else 0.0,
            "per_segment": _segment_contract_summaries(assignments),
        },
        "same_epoch_clock_fields": _clock_field_spreads(epochs),
        "constellation_signal": {
            "group_count": len(group_report),
            "groups": group_report,
            "same_epoch_group_median_spread_ms": _distribution(same_epoch_group_spreads, 0.0),
            "common_mode_classification": "common-mode when same-epoch group median spread is <=2 ms; otherwise non-common-mode",
        },
        "optional_fields": {
            "drift_nanos_per_second": _optional_distribution(epochs, "drift_nsps"),
            "bias_uncertainty_nanos": _optional_distribution(epochs, "bias_uncertainty_ns"),
            "drift_uncertainty_nanos_per_second": _optional_distribution(epochs, "drift_uncertainty_nsps"),
            "time_uncertainty_nanos": _optional_distribution(epochs, "time_uncertainty_ns"),
            "time_offset_nanos": _optional_distribution(epochs, "time_offset_ns"),
        },
        "gate_observations": {
            "all_finite": True,
            "detrended_max_abs_ms": max(abs_detrended),
            "abs_drift_ppm": abs(drift_ppm),
            "uncaptured_hcdc_or_full_bias_changes": sum(
                1 for event in uncaptured_events if event["kind"] in {"full_bias_change", "same_epoch_hcdc_disagreement"}
            ),
            "max_uncaptured_noncommon_tow_effect_m": max(noncommon_tow_effect_m) if noncommon_tow_effect_m else 0.0,
            "base_vs_row_identity_failures": sum(1 for event in events if event["kind"] == "base_vs_row_full_bias_identity_failure"),
        },
        "_epoch_assignments": assignments,
    }
    return route, events


def _optional_distribution_values(values: Sequence[float]) -> dict[str, float | int]:
    if not values:
        return {"count": 0, "median": 0.0, "mad": 0.0, "p50_abs": 0.0, "p95_abs": 0.0, "max_abs": 0.0}
    return _distribution(values, 0.0)


def _segment_contract_summaries(assignments: Sequence[dict[str, Any]]) -> dict[str, Any]:
    by_segment: dict[int, list[dict[str, Any]]] = {}
    for assignment in assignments:
        by_segment.setdefault(int(assignment["segment"]), []).append(assignment)
    return {
        str(segment): {
            "epoch_count": len(items),
            "base_full_bias_nanos": items[0]["segment_base_FullBiasNanos"],
            "base_vs_row_tow_difference_ns": _distribution(
                [float(item["base_vs_row_tow_difference_ns"]) for item in items], 0.0
            ),
            "receiver_tow_spread_ns": _distribution(
                [float(item["receiver_tow_spread_ns"]) for item in items], 0.0
            ),
        }
        for segment, items in sorted(by_segment.items())
    }


def _verify_freeze(path: Path = FREEZE) -> dict[str, Any]:
    freeze, digest, _ = _load_json_once(path, "Phase46 freeze")
    if path == FREEZE and FREEZE_SHA256 and digest != FREEZE_SHA256:
        raise _fail(f"Phase46 freeze hash changed: {digest} != {FREEZE_SHA256}")
    if freeze.get("schema_version") != FREEZE_SCHEMA or freeze.get("status") != "frozen-before-phase46-raw-read":
        raise _fail("Phase46 freeze schema/status mismatch")
    scope = freeze.get("scope")
    if not isinstance(scope, dict) or "ground_truth.csv and any truth-derived file" not in scope.get("forbidden_inputs", []):
        raise _fail("Phase46 forbidden truth input is not frozen")
    reads = freeze.get("read_contract")
    if not isinstance(reads, dict):
        raise _fail("Phase46 read contract missing")
    expected_reads = {
        "single_evaluator_process": True,
        "raw_device_gnss_read_limit_per_route": 1,
        "raw_device_gnss_read_count_required_per_route": 1,
        "truth_read_count_required": 0,
        "archive_reopen_allowed": False,
        "rematerialization_allowed": False,
    }
    if any(reads.get(key) != value for key, value in expected_reads.items()):
        raise _fail("Phase46 read contract is not closed")
    input_map = freeze.get("exact_raw_inputs")
    if not isinstance(input_map, dict) or tuple(input_map) != ROUTES:
        raise _fail("Phase46 route order/input map changed")
    for route in ROUTES:
        item = input_map.get(route)
        if not isinstance(item, dict) or not isinstance(item.get("sha256"), str) or len(item["sha256"]) != 64:
            raise _fail(f"Phase46 raw hash missing for {route}")
        if int(item.get("file_size", 0)) <= 0 or not item.get("path"):
            raise _fail(f"Phase46 raw file metadata missing for {route}")
    equations = freeze.get("fixed_clock_equations")
    if not isinstance(equations, dict) or equations.get("gps_utc_leap_seconds") != GPS_UTC_LEAP_SECONDS:
        raise _fail("Phase46 fixed 18-second UTC/GPS comparison is not frozen")
    gates = freeze.get("numeric_gates")
    if not isinstance(gates, dict):
        raise _fail("Phase46 numeric gates missing")
    if gates.get("route_count", {}).get("required") != 4:
        raise _fail("Phase46 route-count gate changed")
    if gates.get("detrended_utc_gps_residual_ms", {}).get("max_abs_required") != MAX_RESIDUAL_MS:
        raise _fail("Phase46 residual gate changed")
    if gates.get("linear_drift_ppm", {}).get("max_abs_required") != MAX_DRIFT_PPM:
        raise _fail("Phase46 drift gate changed")
    if gates.get("uncaptured_tow_noncommon_mode_m", {}).get("max_required") != MAX_NONCOMMON_TOW_M:
        raise _fail("Phase46 TOW gate changed")
    if gates.get("hcdc_full_bias_boundary_capture", {}).get("uncaptured_hcdc_change_required") != 0:
        raise _fail("Phase46 HCDC boundary gate changed")
    if gates.get("hcdc_full_bias_boundary_capture", {}).get("uncaptured_full_bias_change_required") != 0:
        raise _fail("Phase46 FullBias boundary gate changed")
    if gates.get("base_vs_row_full_bias", {}).get("uncaptured_identity_failures_required") != 0:
        raise _fail("Phase46 base-vs-row gate changed")
    pre = freeze.get("pre_read_assertions")
    if not isinstance(pre, dict) or any(pre.get(key) is not False for key in pre):
        raise _fail("Phase46 pre-read assertions are not closed")
    return freeze


def _verify_manifest(freeze: dict[str, Any], path: Path = MANIFEST) -> dict[str, Any]:
    global VERIFIED_MANIFEST_SHA256
    manifest, digest, _ = _load_json_once(path, "Phase46 evaluator manifest")
    if path == MANIFEST and MANIFEST_SHA256 and digest != MANIFEST_SHA256:
        raise _fail(f"Phase46 evaluator manifest hash changed: {digest} != {MANIFEST_SHA256}")
    if manifest.get("schema_version") != MANIFEST_SCHEMA or manifest.get("status") != "evaluator-frozen-before-phase46-raw-read":
        raise _fail("Phase46 evaluator manifest schema/status mismatch")
    freeze_pin = manifest.get("freeze")
    if not isinstance(freeze_pin, dict) or freeze_pin.get("path") != _relative(FREEZE) or freeze_pin.get("sha256") != FREEZE_SHA256:
        raise _fail("Phase46 freeze pin mismatch in evaluator manifest")
    evaluator = manifest.get("evaluator")
    if not isinstance(evaluator, dict) or evaluator.get("native_solver_invoked") is not False or evaluator.get("operation") != "audit":
        raise _fail("Phase46 evaluator permits solver execution")
    forbidden = manifest.get("forbidden", [])
    if not isinstance(forbidden, list) or "ground_truth.csv" not in forbidden or "solver/native binary rerun" not in forbidden:
        raise _fail("Phase46 manifest forbidden-input policy missing")
    source_pin = evaluator.get("source")
    test_pin = evaluator.get("test")
    if not isinstance(source_pin, dict) or not isinstance(test_pin, dict):
        raise _fail("Phase46 source/test manifest pins missing")
    for pin, label in ((source_pin, "source"), (test_pin, "test")):
        pin_path = ROOT / str(pin.get("path", ""))
        _reject_path(pin_path)
        if not pin_path.is_file():
            raise _fail(f"Phase46 {label} pin missing: {pin_path}")
        actual = _sha256_bytes(pin_path.read_bytes())
        if actual != pin.get("sha256"):
            raise _fail(f"Phase46 {label} hash mismatch: {actual} != {pin.get('sha256')}")
    VERIFIED_MANIFEST_SHA256 = digest
    return manifest


def _gate(name: str, observed: Any, required: Any, passed: bool, detail: str) -> dict[str, Any]:
    return {"name": name, "observed": observed, "required": required, "passed": bool(passed), "detail": detail}


def _aggregate(routes: dict[str, dict[str, Any]]) -> dict[str, Any]:
    route_medians = {route: float(report["clock_residual"]["route_median_ms"]) for route, report in routes.items()}
    values = list(route_medians.values())
    pairwise = []
    route_names = list(routes)
    for index, left in enumerate(route_names):
        for right in route_names[index + 1:]:
            pairwise.append({"route_a": left, "route_b": right, "absolute_median_distance_ms": abs(route_medians[left] - route_medians[right])})
    return {
        "route_count": len(routes),
        "route_median_ms": route_medians,
        "route_median_aggregate_ms": _median(values) if values else 0.0,
        "route_median_mad_ms": _mad(values) if values else 0.0,
        "pairwise_route_median_distances_ms": pairwise,
        "max_pairwise_route_median_distance_ms": max((item["absolute_median_distance_ms"] for item in pairwise), default=0.0),
    }


def _strip_internal(report: dict[str, Any]) -> dict[str, Any]:
    return {key: value for key, value in report.items() if not key.startswith("_")}


def _audit(freeze: dict[str, Any], manifest: dict[str, Any], output_root: Path) -> dict[str, Any]:
    _reject_path(output_root)
    if output_root.exists():
        raise _fail(f"Phase46 one-shot output already exists: {output_root}")
    input_map = freeze["exact_raw_inputs"]
    route_reports: dict[str, dict[str, Any]] = {}
    all_events: list[dict[str, Any]] = []
    read_accounting: dict[str, Any] = {"single_process": True, "raw_device_gnss_reads": {}, "truth_reads": 0, "archive_reopens": 0}
    for route in ROUTES:
        item = input_map[route]
        path = ROOT / item["path"]
        payload, digest = _read_bytes_once(path, f"raw device_gnss {route}", item["sha256"])
        if len(payload) != int(item["file_size"]):
            raise _fail(f"raw device_gnss byte count mismatch for {route}")
        epochs, metadata = _parse_raw_payload(payload)
        del payload
        metadata["path"] = path
        report, events = _route_summary(route, epochs, metadata, int(item["file_size"]), digest)
        report["input"]["expected_sha256"] = item["sha256"]
        report["input"]["expected_file_size"] = int(item["file_size"])
        report["route"] = route
        route_reports[route] = report
        for event in events:
            event["route"] = route
        all_events.extend(events)
        read_accounting["raw_device_gnss_reads"][route] = 1

    aggregate = _aggregate(route_reports)
    gate_rows: list[dict[str, Any]] = []
    gate_rows.append(_gate("route_count", len(route_reports), 4, len(route_reports) == 4, "exact four pinned routes"))
    for route, report in route_reports.items():
        observations = report["gate_observations"]
        gate_rows.append(_gate(f"{route}:all_finite", observations["all_finite"], True, observations["all_finite"], "required raw timing fields"))
        gate_rows.append(_gate(f"{route}:raw_epoch_domain_coverage", report["rows"]["epoch_domain_coverage"], 1.0, report["rows"]["epoch_domain_coverage"] == 1.0, "every eligible Raw row represented exactly once"))
        gate_rows.append(_gate(f"{route}:utc_key_order", report["rows"]["nonmonotonic_epoch_key_count"] + report["rows"]["repeated_epoch_key_count"], 0, report["rows"]["nonmonotonic_epoch_key_count"] == 0 and report["rows"]["repeated_epoch_key_count"] == 0, "first-seen UTC epoch keys"))
        gate_rows.append(_gate(f"{route}:detrended_utc_gps_residual_ms", observations["detrended_max_abs_ms"], MAX_RESIDUAL_MS, observations["detrended_max_abs_ms"] <= MAX_RESIDUAL_MS, "median center plus affine drift"))
        gate_rows.append(_gate(f"{route}:linear_drift_ppm", observations["abs_drift_ppm"], MAX_DRIFT_PPM, observations["abs_drift_ppm"] <= MAX_DRIFT_PPM, "affine drift"))
        gate_rows.append(_gate(f"{route}:hcdc_full_bias_boundary_capture", observations["uncaptured_hcdc_or_full_bias_changes"], 0, observations["uncaptured_hcdc_or_full_bias_changes"] == 0, "explicit Phase25 segment boundary"))
        gate_rows.append(_gate(f"{route}:uncaptured_tow_noncommon_mode_m", observations["max_uncaptured_noncommon_tow_effect_m"], MAX_NONCOMMON_TOW_M, observations["max_uncaptured_noncommon_tow_effect_m"] <= MAX_NONCOMMON_TOW_M, "receiver TOW c-scaled non-common-mode effect"))
        gate_rows.append(_gate(f"{route}:base_vs_row_full_bias", observations["base_vs_row_identity_failures"], 0, observations["base_vs_row_identity_failures"] == 0, "active segment base identity"))
    passed = all(row["passed"] for row in gate_rows)
    status = "go-clock-factor-auditable" if passed else "no-go-clock-factor-not-identifiable-or-gate-failure"
    result = {
        "schema_version": SCHEMA,
        "phase": 46,
        "status": status,
        "mode": "raw-device-gnss-clock-timing-audit-only",
        "freeze": {"path": _relative(FREEZE), "sha256": FREEZE_SHA256 or _sha256_bytes(_json_bytes_for_output(freeze))},
        "evaluator_manifest": {"path": _relative(MANIFEST), "sha256": MANIFEST_SHA256 or VERIFIED_MANIFEST_SHA256 or _sha256_bytes(_json_bytes_for_output(manifest))},
        "read_accounting": read_accounting,
        "routes": {route: _strip_internal(report) for route, report in route_reports.items()},
        "aggregate": aggregate,
        "gates": {"all_passed": passed, "rows": gate_rows},
        "events": {"count": len(all_events), "table": "phase46_pixel5_raw_clock_timing.events.json"},
        "decision": {
            "clock_correction_authorized": False,
            "deployable_mechanism": False,
            "diagnostic_only": True,
            "truth_derived_loo_or_correction": False,
        },
    }
    output_root.mkdir(parents=True, exist_ok=True)
    route_table = {"schema_version": SCHEMA + ".routes", "routes": result["routes"]}
    event_table = {"schema_version": SCHEMA + ".events", "events": all_events}
    result_path = output_root / "phase46_pixel5_raw_clock_timing.json"
    routes_path = output_root / "phase46_pixel5_raw_clock_timing.routes.json"
    events_path = output_root / "phase46_pixel5_raw_clock_timing.events.json"
    result_bytes = _atomic_json(result_path, result)
    routes_bytes = _atomic_json(routes_path, route_table)
    events_bytes = _atomic_json(events_path, event_table)
    artifacts = {
        _relative(result_path): {"bytes": len(result_bytes), "sha256": _sha256_bytes(result_bytes)},
        _relative(routes_path): {"bytes": len(routes_bytes), "sha256": _sha256_bytes(routes_bytes)},
        _relative(events_path): {"bytes": len(events_bytes), "sha256": _sha256_bytes(events_bytes)},
    }
    output_manifest = {
        "schema_version": SCHEMA + ".output-manifest",
        "status": "atomic-publish-complete",
        "phase": 46,
        "freeze_sha256": result["freeze"]["sha256"],
        "evaluator_manifest_sha256": result["evaluator_manifest"]["sha256"],
        "read_accounting": read_accounting,
        "artifacts": artifacts,
    }
    output_manifest_path = output_root / "phase46_pixel5_raw_clock_timing.manifest.json"
    manifest_bytes = _atomic_json(output_manifest_path, output_manifest)
    result["output_artifacts"] = {
        **artifacts,
        _relative(output_manifest_path): {"bytes": len(manifest_bytes), "sha256": _sha256_bytes(manifest_bytes)},
    }
    return result


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args(argv)
    try:
        freeze = _verify_freeze()
        manifest = _verify_manifest(freeze)
        _audit(freeze, manifest, args.output)
    except Phase46Error as exc:
        print(f"phase46: {exc}", file=sys.stderr)
        return 2
    print(f"phase46: completed raw clock/timing audit at {_relative(args.output)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
