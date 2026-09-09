#!/usr/bin/env python3
"""Audit Android smartphone IMU CSVs and build causal motion features.

This module deliberately does not integrate acceleration, estimate attitude, or
feed an absolute device-frame acceleration into the position solution.  It
uses only gyro norm and the change in acceleration norm over a causal window
to decide whether the trajectory smoother should increase its process noise.
Malformed/non-finite samples, clock discontinuities, stale windows, and large
sample gaps all fail safe to the baseline process-noise multiplier.
"""

from __future__ import annotations

import csv
from dataclasses import dataclass
from decimal import Decimal
import hashlib
import math
from array import array
from pathlib import Path
from typing import Any

import numpy as np


SCHEMA_VERSION = "smartphone-imu-motion.v1"
AUDIT_SCHEMA_VERSION = "smartphone-imu-audit.v1"
REQUIRED_FIELDS = (
    "MessageType",
    "utcTimeMillis",
    "elapsedRealtimeNanos",
    "MeasurementX",
    "MeasurementY",
    "MeasurementZ",
    "BiasX",
    "BiasY",
    "BiasZ",
)
SUPPORTED_TYPES = ("UncalGyro", "UncalAccel")
DEFAULT_WINDOW_S = 1.0
DEFAULT_MAX_SAMPLE_GAP_S = 0.25
DEFAULT_MAX_SAMPLE_AGE_S = 0.25
DEFAULT_GYRO_THRESHOLD = 0.15
DEFAULT_ACCEL_DYNAMIC_THRESHOLD = 0.75
DEFAULT_MOTION_Q_MULTIPLIER = 2.0
CLOCK_OFFSET_JUMP_MS = 250.0


class ImuError(ValueError):
    """Raised for an IMU schema or configuration contract violation."""


@dataclass(frozen=True)
class MotionConfig:
    window_s: float = DEFAULT_WINDOW_S
    max_sample_gap_s: float = DEFAULT_MAX_SAMPLE_GAP_S
    max_sample_age_s: float = DEFAULT_MAX_SAMPLE_AGE_S
    gyro_threshold: float = DEFAULT_GYRO_THRESHOLD
    accel_dynamic_threshold: float = DEFAULT_ACCEL_DYNAMIC_THRESHOLD
    motion_q_multiplier: float = DEFAULT_MOTION_Q_MULTIPLIER

    def validate(self) -> None:
        values = {
            "window_s": self.window_s,
            "max_sample_gap_s": self.max_sample_gap_s,
            "max_sample_age_s": self.max_sample_age_s,
            "gyro_threshold": self.gyro_threshold,
            "accel_dynamic_threshold": self.accel_dynamic_threshold,
            "motion_q_multiplier": self.motion_q_multiplier,
        }
        for name, value in values.items():
            if not math.isfinite(value) or value <= 0.0:
                raise ImuError(f"{name} must be finite and positive")


@dataclass(frozen=True)
class ImuStream:
    message_type: str
    timestamps_ms: np.ndarray
    elapsed_ns: np.ndarray
    measurements: np.ndarray
    biases: np.ndarray
    invalid: np.ndarray
    discontinuity: np.ndarray

    @property
    def sample_count(self) -> int:
        return int(self.timestamps_ms.size)


@dataclass(frozen=True)
class ImuDataset:
    path: Path
    streams: dict[str, ImuStream]
    audit: dict[str, Any]


@dataclass(frozen=True)
class MotionEpoch:
    timestamp_ms: int
    process_noise_multiplier: float
    dynamic: bool
    gyro_activity: float | None
    accel_dynamic: float | None
    gyro_samples: int
    accel_samples: int
    window_start_ms: int | None
    latest_sample_age_ms: int | None
    fallback_to_baseline: bool
    fallback_reason: str | None
    gap_detected: bool
    clock_discontinuity_detected: bool
    nonfinite_detected: bool

    def as_dict(self) -> dict[str, Any]:
        return {
            "timestamp_ms": self.timestamp_ms,
            "process_noise_multiplier": self.process_noise_multiplier,
            "dynamic": self.dynamic,
            "gyro_activity": self.gyro_activity,
            "accel_dynamic": self.accel_dynamic,
            "gyro_samples": self.gyro_samples,
            "accel_samples": self.accel_samples,
            "window_start_ms": self.window_start_ms,
            "latest_sample_age_ms": self.latest_sample_age_ms,
            "fallback_to_baseline": self.fallback_to_baseline,
            "fallback_reason": self.fallback_reason,
            "gap_detected": self.gap_detected,
            "clock_discontinuity_detected": self.clock_discontinuity_detected,
            "nonfinite_detected": self.nonfinite_detected,
        }


@dataclass(frozen=True)
class MotionProfile:
    epochs: list[MotionEpoch]
    config: MotionConfig
    summary: dict[str, Any]

    @property
    def multipliers(self) -> list[float]:
        return [epoch.process_noise_multiplier for epoch in self.epochs]


def _sha256(path: Path) -> str:
    if not path.is_file():
        raise ImuError(f"missing device IMU CSV: {path}")
    digest = hashlib.sha256()
    try:
        with path.open("rb") as handle:
            for chunk in iter(lambda: handle.read(1024 * 1024), b""):
                digest.update(chunk)
    except OSError as exc:
        raise ImuError(f"failed to hash device IMU CSV: {path}") from exc
    return digest.hexdigest()


def _finite_float(raw: str | None) -> tuple[float, bool]:
    try:
        value = float(raw or "")
    except (TypeError, ValueError):
        return math.nan, False
    return value, math.isfinite(value)


def _integer(raw: str | None) -> tuple[int, bool]:
    token = (raw or "").strip()
    try:
        # Decimal rejects accidental fractional timestamps while allowing the
        # large integer range used by Android clocks.
        value = Decimal(token)
        if not value.is_finite() or value != value.to_integral_value():
            return 0, False
        integer = int(value)
    except (ArithmeticError, ValueError):
        return 0, False
    return integer, integer >= 0


def _percentile(values: np.ndarray, fraction: float) -> float | None:
    if values.size == 0:
        return None
    return float(np.percentile(values, fraction * 100.0, method="linear"))


def _stream_summary(stream: ImuStream) -> dict[str, Any]:
    timestamps = stream.timestamps_ms
    elapsed = stream.elapsed_ns
    utc_deltas = np.diff(timestamps) if timestamps.size > 1 else np.array([], dtype=np.int64)
    elapsed_deltas = np.diff(elapsed) if elapsed.size > 1 else np.array([], dtype=np.int64)
    offset_ms = elapsed.astype(np.float64) / 1e6 - timestamps.astype(np.float64)
    offset_deltas = np.diff(offset_ms) if offset_ms.size > 1 else np.array([], dtype=float)
    finite_measurements = stream.measurements[np.isfinite(stream.measurements)]
    finite_biases = stream.biases[np.isfinite(stream.biases)]
    nonzero_biases = finite_biases[np.abs(finite_biases) > 0.0]
    return {
        "sample_count": int(timestamps.size),
        "unique_utc_timestamps": int(np.unique(timestamps).size),
        "utc_start_ms": int(timestamps[0]) if timestamps.size else None,
        "utc_end_ms": int(timestamps[-1]) if timestamps.size else None,
        "span_s": float((timestamps[-1] - timestamps[0]) / 1000.0)
        if timestamps.size
        else 0.0,
        "utc_interval_ms": {
            "median": _percentile(utc_deltas.astype(float), 0.5),
            "p95": _percentile(utc_deltas.astype(float), 0.95),
            "min": int(utc_deltas.min()) if utc_deltas.size else None,
            "max": int(utc_deltas.max()) if utc_deltas.size else None,
            "backward_count": int(np.count_nonzero(utc_deltas < 0)),
            "zero_count": int(np.count_nonzero(utc_deltas == 0)),
        },
        "elapsed_interval_ms": {
            "median": _percentile(elapsed_deltas.astype(float) / 1e6, 0.5),
            "p95": _percentile(elapsed_deltas.astype(float) / 1e6, 0.95),
            "min": float(elapsed_deltas.min() / 1e6) if elapsed_deltas.size else None,
            "max": float(elapsed_deltas.max() / 1e6) if elapsed_deltas.size else None,
            "backward_count": int(np.count_nonzero(elapsed_deltas < 0)),
            "zero_count": int(np.count_nonzero(elapsed_deltas == 0)),
        },
        "clock_offset_ms": {
            "median": _percentile(offset_ms, 0.5),
            "p95_abs_step": _percentile(np.abs(offset_deltas), 0.95),
            "maximum_abs_step": float(np.max(np.abs(offset_deltas)))
            if offset_deltas.size
            else 0.0,
        },
        "measurement_finite_count": int(finite_measurements.size),
        "measurement_nonfinite_count": int(stream.measurements.size - finite_measurements.size),
        "bias_finite_count": int(finite_biases.size),
        "bias_nonfinite_count": int(stream.biases.size - finite_biases.size),
        "bias_nonzero_count": int(nonzero_biases.size),
        "bias_min": float(np.min(finite_biases)) if finite_biases.size else None,
        "bias_max": float(np.max(finite_biases)) if finite_biases.size else None,
        "detected_clock_discontinuity_count": int(np.count_nonzero(stream.discontinuity)),
        "invalid_sample_count": int(np.count_nonzero(stream.invalid)),
    }


def _new_stream_buffer() -> dict[str, Any]:
    return {
        "timestamps": array("q"),
        "elapsed": array("q"),
        "measurements": [array("d"), array("d"), array("d")],
        "biases": [array("d"), array("d"), array("d")],
        "invalid": bytearray(),
    }


def _make_stream(message_type: str, buffer: dict[str, Any]) -> ImuStream:
    timestamps = np.asarray(buffer["timestamps"], dtype=np.int64)
    if not timestamps.size:
        return ImuStream(
            message_type,
            np.empty(0, dtype=np.int64),
            np.empty(0, dtype=np.int64),
            np.empty((0, 3), dtype=float),
            np.empty((0, 3), dtype=float),
            np.empty(0, dtype=bool),
            np.empty(0, dtype=bool),
        )
    elapsed = np.asarray(buffer["elapsed"], dtype=np.int64)
    measurements = np.column_stack(
        [np.asarray(values, dtype=float) for values in buffer["measurements"]]
    )
    biases = np.column_stack(
        [np.asarray(values, dtype=float) for values in buffer["biases"]]
    )
    invalid = np.asarray(buffer["invalid"], dtype=bool)
    order = np.lexsort((elapsed, timestamps))
    if not np.array_equal(order, np.arange(timestamps.size)):
        timestamps = timestamps[order]
        elapsed = elapsed[order]
        measurements = measurements[order]
        biases = biases[order]
        invalid = invalid[order]
    utc_deltas = np.diff(timestamps)
    elapsed_deltas = np.diff(elapsed)
    offset_ms = elapsed.astype(np.float64) / 1e6 - timestamps.astype(np.float64)
    offset_deltas = np.diff(offset_ms)
    discontinuity = np.zeros(timestamps.size, dtype=bool)
    if discontinuity.size > 1:
        discontinuity[1:] = (
            (utc_deltas < 0)
            | (elapsed_deltas <= 0)
            | (np.abs(offset_deltas) > CLOCK_OFFSET_JUMP_MS)
            | (utc_deltas > int(DEFAULT_MAX_SAMPLE_GAP_S * 1000.0))
        )
    return ImuStream(message_type, timestamps, elapsed, measurements, biases, invalid, discontinuity)


def read_imu(path: Path) -> ImuDataset:
    """Read and audit the Android IMU CSV without using truth or labels."""

    if not path.is_file():
        raise ImuError(f"missing device IMU CSV: {path}")
    records: dict[str, dict[str, Any]] = {
        message_type: _new_stream_buffer() for message_type in SUPPORTED_TYPES
    }
    type_timestamps: dict[str, list[int]] = {}
    type_counts: dict[str, int] = {}
    type_quality: dict[str, dict[str, Any]] = {}
    parse_error_rows = 0
    extra_nonfinite_rows = 0
    fieldnames: list[str] | None = None
    try:
        with path.open(encoding="utf-8-sig", newline="") as handle:
            reader = csv.DictReader(handle)
            fieldnames = list(reader.fieldnames or ())
            missing = set(REQUIRED_FIELDS) - set(fieldnames)
            if missing:
                raise ImuError(
                    f"device IMU CSV missing fields: {', '.join(sorted(missing))}"
                )
            if len(fieldnames) != len(set(fieldnames)):
                raise ImuError("device IMU CSV has duplicate header fields")
            for row_number, raw_row in enumerate(reader, start=2):
                if None in raw_row:
                    raise ImuError(f"device IMU row {row_number}: more values than header")
                message_type = (raw_row.get("MessageType") or "").strip()
                type_counts[message_type] = type_counts.get(message_type, 0) + 1
                quality = type_quality.setdefault(
                    message_type,
                    {
                        "measurement_finite_components": 0,
                        "measurement_nonfinite_components": 0,
                        "bias_finite_components": 0,
                        "bias_nonfinite_components": 0,
                        "bias_nonzero_components": 0,
                        "bias_min": None,
                        "bias_max": None,
                    },
                )
                timestamp, timestamp_ok = _integer(raw_row.get("utcTimeMillis"))
                elapsed, elapsed_ok = _integer(raw_row.get("elapsedRealtimeNanos"))
                if not timestamp_ok or not elapsed_ok:
                    parse_error_rows += 1
                    continue
                type_timestamps.setdefault(message_type, []).append(timestamp)
                measurement_values: list[float] = []
                bias_values: list[float] = []
                invalid = False
                for field in ("MeasurementX", "MeasurementY", "MeasurementZ"):
                    value, finite = _finite_float(raw_row.get(field))
                    measurement_values.append(value)
                    invalid = invalid or not finite
                    quality[
                        "measurement_finite_components"
                        if finite
                        else "measurement_nonfinite_components"
                    ] += 1
                for field in ("BiasX", "BiasY", "BiasZ"):
                    value, finite = _finite_float(raw_row.get(field))
                    bias_values.append(value)
                    invalid = invalid or not finite
                    quality[
                        "bias_finite_components" if finite else "bias_nonfinite_components"
                    ] += 1
                    if finite:
                        quality["bias_min"] = (
                            value
                            if quality["bias_min"] is None
                            else min(float(quality["bias_min"]), value)
                        )
                        quality["bias_max"] = (
                            value
                            if quality["bias_max"] is None
                            else max(float(quality["bias_max"]), value)
                        )
                        if value != 0.0:
                            quality["bias_nonzero_components"] += 1
                if message_type not in records:
                    continue
                if invalid:
                    extra_nonfinite_rows += 1
                buffer = records[message_type]
                buffer["timestamps"].append(timestamp)
                buffer["elapsed"].append(elapsed)
                for index, value in enumerate(measurement_values):
                    buffer["measurements"][index].append(value)
                for index, value in enumerate(bias_values):
                    buffer["biases"][index].append(value)
                buffer["invalid"].append(1 if invalid else 0)
    except OSError as exc:
        raise ImuError(f"failed to read device IMU CSV: {path}") from exc
    streams = {
        message_type: _make_stream(message_type, stream_buffer)
        for message_type, stream_buffer in records.items()
    }
    # Do not retain Python-side row buffers after conversion to compact NumPy
    # arrays.  This keeps the full-route audit bounded even for large IMU CSVs.
    del records
    stream_summaries = {
        message_type: _stream_summary(stream)
        for message_type, stream in streams.items()
    }
    unsupported_summaries: dict[str, Any] = {}
    for message_type, timestamps in type_timestamps.items():
        if message_type in streams:
            continue
        ordered = np.asarray(sorted(timestamps), dtype=np.int64)
        deltas = np.diff(ordered) if ordered.size > 1 else np.array([], dtype=np.int64)
        unsupported_summaries[message_type] = {
            "sample_count": int(ordered.size),
            "unique_utc_timestamps": int(np.unique(ordered).size),
            "utc_start_ms": int(ordered[0]) if ordered.size else None,
            "utc_end_ms": int(ordered[-1]) if ordered.size else None,
            "span_s": float((ordered[-1] - ordered[0]) / 1000.0) if ordered.size else 0.0,
            "utc_interval_ms": {
                "median": _percentile(deltas.astype(float), 0.5),
                "p95": _percentile(deltas.astype(float), 0.95),
                "min": int(deltas.min()) if deltas.size else None,
                "max": int(deltas.max()) if deltas.size else None,
                "backward_count": int(np.count_nonzero(deltas < 0)),
                "zero_count": int(np.count_nonzero(deltas == 0)),
            },
            "measurement_finite_components": type_quality.get(message_type, {}).get(
                "measurement_finite_components", 0
            ),
            "measurement_nonfinite_components": type_quality.get(message_type, {}).get(
                "measurement_nonfinite_components", 0
            ),
            "bias_finite_components": type_quality.get(message_type, {}).get(
                "bias_finite_components", 0
            ),
            "bias_nonfinite_components": type_quality.get(message_type, {}).get(
                "bias_nonfinite_components", 0
            ),
            "bias_nonzero_components": type_quality.get(message_type, {}).get(
                "bias_nonzero_components", 0
            ),
            "bias_min": type_quality.get(message_type, {}).get("bias_min"),
            "bias_max": type_quality.get(message_type, {}).get("bias_max"),
            "used_for_motion": False,
        }
    audit = {
        "schema_version": AUDIT_SCHEMA_VERSION,
        "path": str(path),
        "sha256": _sha256(path),
        "observed_fields": fieldnames or [],
        "required_fields": list(REQUIRED_FIELDS),
        "hardware_clock_discontinuity_count_field_present":
            "HardwareClockDiscontinuityCount" in (fieldnames or []),
        "supported_message_types": list(SUPPORTED_TYPES),
        "message_type_counts": type_counts,
        "parse_error_rows": parse_error_rows,
        "supported_nonfinite_rows": extra_nonfinite_rows,
        "streams": stream_summaries,
        "unsupported_streams": unsupported_summaries,
        "contract": {
            "utc_key": "utcTimeMillis",
            "elapsed_clock": "elapsedRealtimeNanos",
            "causal_only": True,
            "absolute_acceleration_integration": False,
            "motion_features": "gyro vector norm and robust acceleration-norm dynamics",
            "unsupported_types_are_ignored": True,
            "invalid_or_discontinuous_samples_fail_safe_to_baseline_q": True,
        },
    }
    return ImuDataset(path, streams, audit)


def _window_features(
    stream: ImuStream,
    timestamp_ms: int,
    config: MotionConfig,
    minimum_timestamp_ms: int | None = None,
) -> tuple[np.ndarray, bool, bool, bool, int | None, int | None]:
    if stream.sample_count == 0:
        return np.empty((0, 3)), True, False, False, None, None
    end = int(np.searchsorted(stream.timestamps_ms, timestamp_ms, side="right"))
    window_start_ms = timestamp_ms - int(round(config.window_s * 1000.0))
    if minimum_timestamp_ms is not None:
        window_start_ms = max(window_start_ms, minimum_timestamp_ms)
    start = int(
        np.searchsorted(
            stream.timestamps_ms,
            window_start_ms,
            side="left",
        )
    )
    if end <= start:
        return np.empty((0, 3)), True, False, False, None, None
    timestamps = stream.timestamps_ms[start:end]
    values = stream.measurements[start:end]
    gap = bool(
        timestamps.size > 1
        and np.any(np.diff(timestamps) > int(round(config.max_sample_gap_s * 1000.0)))
    )
    clock = bool(np.any(stream.discontinuity[start:end]))
    invalid = bool(np.any(stream.invalid[start:end]) or not np.isfinite(values).all())
    age = timestamp_ms - int(timestamps[-1])
    stale = age < 0 or age > int(round(config.max_sample_age_s * 1000.0))
    return values, bool(gap or stale), clock, invalid, int(timestamps[0]), int(age)


def build_motion_profile(
    device_epochs: list[int],
    dataset: ImuDataset,
    config: MotionConfig,
    *,
    reset_indices: list[int] | tuple[int, ...] = (),
) -> MotionProfile:
    """Build one causal, truth-free process-noise multiplier per GNSS epoch."""

    config.validate()
    if not device_epochs:
        raise ImuError("cannot build IMU profile for empty device epoch list")
    if any(b <= a for a, b in zip(device_epochs, device_epochs[1:])):
        raise ImuError("device epoch keys must be strictly increasing")
    requested_resets = set(reset_indices)
    if any(index <= 0 or index >= len(device_epochs) for index in requested_resets):
        raise ImuError("motion reset index must be between 1 and epoch count - 1")
    segment_starts = sorted({0, *requested_resets})
    segment_for_index = [0] * len(device_epochs)
    for segment_id, start in enumerate(segment_starts):
        end = segment_starts[segment_id + 1] if segment_id + 1 < len(segment_starts) else len(device_epochs)
        for index in range(start, end):
            segment_for_index[index] = segment_id
    gyro = dataset.streams["UncalGyro"]
    accel = dataset.streams["UncalAccel"]
    epochs: list[MotionEpoch] = []
    for index, timestamp_ms in enumerate(device_epochs):
        minimum_timestamp_ms = device_epochs[segment_starts[segment_for_index[index]]]
        gyro_values, gyro_gap, gyro_clock, gyro_invalid, window_start_g, gyro_age = _window_features(
            gyro, timestamp_ms, config, minimum_timestamp_ms
        )
        accel_values, accel_gap, accel_clock, accel_invalid, window_start_a, accel_age = _window_features(
            accel, timestamp_ms, config, minimum_timestamp_ms
        )
        gyro_norm = (
            np.linalg.norm(gyro_values, axis=1)
            if gyro_values.size and np.isfinite(gyro_values).all()
            else np.empty(0, dtype=float)
        )
        accel_norm = (
            np.linalg.norm(accel_values, axis=1)
            if accel_values.size and np.isfinite(accel_values).all()
            else np.empty(0, dtype=float)
        )
        gyro_activity = _percentile(gyro_norm, 0.90)
        if accel_norm.size:
            accel_dynamic_values = np.abs(accel_norm - float(np.median(accel_norm)))
            accel_dynamic = _percentile(accel_dynamic_values, 0.90)
        else:
            accel_dynamic = None
        clock = gyro_clock or accel_clock
        invalid = gyro_invalid or accel_invalid
        gap = gyro_gap or accel_gap
        no_samples = not gyro_norm.size and not accel_norm.size
        stale = (gyro_age is None or gyro_age < 0 or gyro_age > int(round(config.max_sample_age_s * 1000.0))) and (
            accel_age is None or accel_age < 0 or accel_age > int(round(config.max_sample_age_s * 1000.0))
        )
        reasons: list[str] = []
        if no_samples:
            reasons.append("no_causal_sample")
        if stale:
            reasons.append("stale_causal_window")
        if gap:
            reasons.append("sample_gap")
        if clock:
            reasons.append("clock_discontinuity")
        if invalid:
            reasons.append("nonfinite_sample")
        fallback = bool(reasons)
        dynamic = bool(
            not fallback
            and (
                (gyro_activity is not None and gyro_activity >= config.gyro_threshold)
                or (accel_dynamic is not None and accel_dynamic >= config.accel_dynamic_threshold)
            )
        )
        multiplier = config.motion_q_multiplier if dynamic else 1.0
        window_starts = [value for value in (window_start_g, window_start_a) if value is not None]
        ages = [value for value in (gyro_age, accel_age) if value is not None and value >= 0]
        epochs.append(
            MotionEpoch(
                timestamp_ms=timestamp_ms,
                process_noise_multiplier=multiplier,
                dynamic=dynamic,
                gyro_activity=gyro_activity,
                accel_dynamic=accel_dynamic,
                gyro_samples=int(gyro_norm.size),
                accel_samples=int(accel_norm.size),
                window_start_ms=min(window_starts) if window_starts else None,
                latest_sample_age_ms=min(ages) if ages else None,
                fallback_to_baseline=fallback,
                fallback_reason=",".join(reasons) if reasons else None,
                gap_detected=gap,
                clock_discontinuity_detected=clock,
                nonfinite_detected=invalid,
            )
        )
    dynamic_epochs = sum(epoch.dynamic for epoch in epochs)
    fallback_epochs = sum(epoch.fallback_to_baseline for epoch in epochs)
    summary = {
        "schema_version": SCHEMA_VERSION,
        "algorithm": {
            "name": "causal gyro-norm/acceleration-norm dynamics process-noise adaptation",
            "truth_used": False,
            "absolute_acceleration_integration": False,
            "orientation_estimation": False,
            "future_samples_used": False,
            "window_s": config.window_s,
            "max_sample_gap_s": config.max_sample_gap_s,
            "max_sample_age_s": config.max_sample_age_s,
            "gyro_threshold": config.gyro_threshold,
            "accel_dynamic_threshold": config.accel_dynamic_threshold,
            "motion_q_multiplier": config.motion_q_multiplier,
            "reset_indices": list(segment_starts[1:]),
            "fallback_policy": "any gap, stale window, nonfinite value, or clock discontinuity uses baseline multiplier 1.0",
        },
        "epochs": len(epochs),
        "dynamic_epochs": dynamic_epochs,
        "fallback_epochs": fallback_epochs,
        "dynamic_ratio": dynamic_epochs / len(epochs),
        "fallback_ratio": fallback_epochs / len(epochs),
        "max_latest_sample_age_ms": max(
            (epoch.latest_sample_age_ms or 0 for epoch in epochs), default=0
        ),
        "gap_detected_epochs": sum(epoch.gap_detected for epoch in epochs),
        "clock_discontinuity_epochs": sum(
            epoch.clock_discontinuity_detected for epoch in epochs
        ),
        "nonfinite_epochs": sum(epoch.nonfinite_detected for epoch in epochs),
        "feature_ranges": {
            "gyro_activity": {
                "min": min(
                    (epoch.gyro_activity for epoch in epochs if epoch.gyro_activity is not None),
                    default=None,
                ),
                "max": max(
                    (epoch.gyro_activity for epoch in epochs if epoch.gyro_activity is not None),
                    default=None,
                ),
            },
            "accel_dynamic": {
                "min": min(
                    (epoch.accel_dynamic for epoch in epochs if epoch.accel_dynamic is not None),
                    default=None,
                ),
                "max": max(
                    (epoch.accel_dynamic for epoch in epochs if epoch.accel_dynamic is not None),
                    default=None,
                ),
            },
        },
        "config": {
            "window_s": config.window_s,
            "max_sample_gap_s": config.max_sample_gap_s,
            "max_sample_age_s": config.max_sample_age_s,
            "gyro_threshold": config.gyro_threshold,
            "accel_dynamic_threshold": config.accel_dynamic_threshold,
            "motion_q_multiplier": config.motion_q_multiplier,
        },
        "input_audit": dataset.audit,
    }
    return MotionProfile(epochs, config, summary)


def profile_rows(profile: MotionProfile) -> list[dict[str, Any]]:
    return [epoch.as_dict() for epoch in profile.epochs]
