#!/usr/bin/env python3
"""Truth-free constant-velocity Kalman/RTS smoothing for smartphone POS data.

The solver output and raw device epoch keys are the only positioning inputs to
this command.  Ground truth is intentionally not accepted.  A separate
development-only evaluator may score the emitted artifact and select one of
the pre-declared parameter pairs, but the smoother itself never observes that
score.
"""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
from decimal import Decimal, InvalidOperation, ROUND_FLOOR
import hashlib
import io
import json
import math
import os
from pathlib import Path
import re
import subprocess
import sys
import tempfile
import time
from typing import Any, Iterable

import numpy as np

from support.gnss_runtime import application_root


ROOT = application_root(__file__)
SCHEMA_VERSION = "smartphone-trajectory-smoother.v1"
MANIFEST_SCHEMA_VERSION = "smartphone-trajectory-smoother-manifest.v1"
GPS_EPOCH_UNIX_SECONDS = Decimal("315964800")
SECONDS_PER_WEEK = Decimal("604800")
DEFAULT_GPS_UTC_LEAP_SECONDS = 18
PROPAGATED_STATUS = 7
POSITION_FIELDS = (
    "week",
    "tow",
    "x",
    "y",
    "z",
    "latitude",
    "longitude",
    "height",
    "status",
    "satellites",
    "pdop",
    "ratio",
    "fixed_ambiguities",
    "iterations",
)
_INTEGER_RE = re.compile(r"^[+-]?\d+$")
_FLOAT_RE = re.compile(
    r"^[+-]?(?:(?:\d+(?:\.\d*)?)|(?:\.\d+))(?:[eE][+-]?\d+)?$"
)


class SmootherError(ValueError):
    """Raised for an input or numerical contract violation."""


@dataclass(frozen=True)
class PositionRow:
    week: int
    tow: float
    timestamp_ms: int
    ecef: np.ndarray
    latitude: float
    longitude: float
    height: float
    status: int
    satellites: int
    pdop: float
    ratio: float
    fixed_ambiguities: int
    iterations: int
    source_line: int


@dataclass(frozen=True)
class SmootherConfig:
    process_noise: float
    measurement_floor_m: float
    outlier_gate_sigma: float = 5.0
    segment_gap_s: float = 10.0
    max_consecutive_rejects: int | None = None
    max_prediction_duration_s: float | None = None

    def validate(self) -> None:
        values = {
            "process_noise": self.process_noise,
            "measurement_floor_m": self.measurement_floor_m,
            "outlier_gate_sigma": self.outlier_gate_sigma,
            "segment_gap_s": self.segment_gap_s,
        }
        for name, value in values.items():
            if not math.isfinite(value) or value <= 0.0:
                raise SmootherError(f"{name} must be finite and positive")
        if self.max_consecutive_rejects is not None and (
            isinstance(self.max_consecutive_rejects, bool)
            or not isinstance(self.max_consecutive_rejects, int)
            or self.max_consecutive_rejects <= 0
        ):
            raise SmootherError("max_consecutive_rejects must be a positive integer or None")
        if self.max_prediction_duration_s is not None and (
            not math.isfinite(self.max_prediction_duration_s)
            or self.max_prediction_duration_s <= 0.0
        ):
            raise SmootherError(
                "max_prediction_duration_s must be finite and positive or None"
            )


@dataclass(frozen=True)
class SmoothedRow:
    timestamp_ms: int
    week: int
    tow: float
    ecef: np.ndarray
    latitude: float
    longitude: float
    height: float
    status: int
    satellites: int
    pdop: float
    ratio: float
    fixed_ambiguities: int
    iterations: int
    source: str
    segment_id: int
    measurement_used: bool
    outlier_rejected: bool
    innovation_sigma: float | None
    position_sigma_m: float
    reacquired: bool = False


@dataclass(frozen=True)
class SmoothingResult:
    rows: list[SmoothedRow]
    origin_ecef: np.ndarray
    origin_latitude: float
    origin_longitude: float
    origin_height: float
    selected_device_epochs: int
    measured_epochs: int
    synthesized_epochs: int
    outlier_rejections: int
    segment_count: int
    max_input_gap_s: float
    max_position_gap_s: float
    reset_indices: tuple[int, ...]
    numerical_fallbacks: int
    elapsed_s: float
    reacquisition_indices: tuple[int, ...] = ()
    max_consecutive_rejects: int = 0
    max_prediction_duration_s: float = 0.0


def _require_file(path: Path, label: str) -> None:
    if not path.is_file():
        raise SmootherError(f"missing {label}: {path}")


def _sha256(path: Path) -> str:
    _require_file(path, "input")
    digest = hashlib.sha256()
    try:
        with path.open("rb") as handle:
            for chunk in iter(lambda: handle.read(1024 * 1024), b""):
                digest.update(chunk)
    except OSError as exc:
        raise SmootherError(f"failed to hash {path}: {exc}") from exc
    return digest.hexdigest()


def _atomic_write(path: Path, content: bytes) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary: str | None = None
    descriptor: int | None = None
    try:
        descriptor, temporary = tempfile.mkstemp(
            prefix=f".{path.name}.", suffix=".tmp", dir=str(path.parent)
        )
        with os.fdopen(descriptor, "wb") as handle:
            descriptor = None
            handle.write(content)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, path)
        temporary = None
        if hasattr(os, "O_DIRECTORY"):
            directory_descriptor = os.open(str(path.parent), os.O_DIRECTORY)
            try:
                os.fsync(directory_descriptor)
            finally:
                os.close(directory_descriptor)
    except OSError as exc:
        raise SmootherError(f"atomic publish failed for {path}: {exc}") from exc
    finally:
        if descriptor is not None:
            os.close(descriptor)
        if temporary is not None:
            try:
                os.unlink(temporary)
            except FileNotFoundError:
                pass


def _json_bytes(payload: dict[str, Any]) -> bytes:
    return (json.dumps(payload, indent=2, sort_keys=True) + "\n").encode("utf-8")


def _parse_float(raw: str, field: str, line: int) -> float:
    token = raw.strip()
    if not _FLOAT_RE.fullmatch(token):
        raise SmootherError(f"line {line}: {field} must be finite")
    try:
        value = float(token)
    except ValueError as exc:
        raise SmootherError(f"line {line}: {field} must be finite") from exc
    if not math.isfinite(value):
        raise SmootherError(f"line {line}: {field} must be finite")
    return value


def _parse_int(raw: str, field: str, line: int) -> int:
    token = raw.strip()
    if not _INTEGER_RE.fullmatch(token):
        raise SmootherError(f"line {line}: {field} must be an integer")
    try:
        return int(token)
    except ValueError as exc:
        raise SmootherError(f"line {line}: {field} must be an integer") from exc


def _position_timestamp(
    week_token: str, tow_token: str, leap_seconds: int, line: int
) -> int:
    week = _parse_int(week_token, "GPS week", line)
    if week < 0:
        raise SmootherError(f"line {line}: GPS week must be non-negative")
    tow_token = tow_token.strip()
    if not _FLOAT_RE.fullmatch(tow_token):
        raise SmootherError(f"line {line}: GPS TOW must be finite")
    try:
        tow = Decimal(tow_token)
    except (InvalidOperation, ValueError) as exc:
        raise SmootherError(f"line {line}: GPS TOW must be finite") from exc
    if not tow.is_finite() or tow < 0 or tow >= SECONDS_PER_WEEK:
        raise SmootherError(f"line {line}: GPS TOW must be in [0, 604800)")
    if not isinstance(leap_seconds, int) or leap_seconds < 0:
        raise SmootherError("gps UTC leap seconds must be non-negative")
    return int(
        (
            GPS_EPOCH_UNIX_SECONDS * 1000
            + Decimal(week) * SECONDS_PER_WEEK * 1000
            + tow * 1000
            - Decimal(leap_seconds) * 1000
        ).to_integral_value(rounding=ROUND_FLOOR)
    )


def _read_positions(path: Path, leap_seconds: int) -> list[PositionRow]:
    _require_file(path, "position file")
    rows: list[PositionRow] = []
    seen: set[int] = set()
    try:
        with path.open(encoding="ascii") as handle:
            for line_number, raw_line in enumerate(handle, start=1):
                line = raw_line.strip()
                if not line or line.startswith("%") or line.startswith("#"):
                    continue
                fields = line.split()
                if len(fields) < 11:
                    raise SmootherError(
                        f"position line {line_number}: expected at least 11 fields"
                    )
                week = _parse_int(fields[0], "GPS week", line_number)
                tow = _parse_float(fields[1], "GPS TOW", line_number)
                timestamp = _position_timestamp(
                    fields[0], fields[1], leap_seconds, line_number
                )
                if timestamp in seen:
                    raise SmootherError(
                        f"position line {line_number}: duplicate timestamp {timestamp}"
                    )
                seen.add(timestamp)
                ecef = np.array(
                    [_parse_float(fields[index], POSITION_FIELDS[index], line_number) for index in (2, 3, 4)],
                    dtype=float,
                )
                latitude = _parse_float(fields[5], "latitude", line_number)
                longitude = _parse_float(fields[6], "longitude", line_number)
                height = _parse_float(fields[7], "height", line_number)
                if not -90.0 <= latitude <= 90.0:
                    raise SmootherError(f"position line {line_number}: latitude out of range")
                if not -180.0 <= longitude <= 180.0:
                    raise SmootherError(f"position line {line_number}: longitude out of range")
                status = _parse_int(fields[8], "status", line_number)
                satellites = _parse_int(fields[9], "satellites", line_number)
                if satellites < 0:
                    raise SmootherError(f"position line {line_number}: satellites must be non-negative")
                pdop = _parse_float(fields[10], "PDOP", line_number)
                ratio = _parse_float(fields[11], "ratio", line_number) if len(fields) > 11 else 0.0
                fixed = _parse_int(fields[12], "fixed ambiguities", line_number) if len(fields) > 12 else 0
                iterations = _parse_int(fields[13], "iterations", line_number) if len(fields) > 13 else 0
                rows.append(
                    PositionRow(
                        week,
                        tow,
                        timestamp,
                        ecef,
                        latitude,
                        longitude,
                        height,
                        status,
                        satellites,
                        pdop,
                        ratio,
                        fixed,
                        iterations,
                        line_number,
                    )
                )
    except OSError as exc:
        raise SmootherError(f"failed to read position file {path}: {exc}") from exc
    rows.sort(key=lambda row: row.timestamp_ms)
    if not rows:
        raise SmootherError("position file contains no solution rows")
    return rows


def _read_device_epochs(path: Path, skip_epochs: int) -> list[int]:
    _require_file(path, "device GNSS CSV")
    if skip_epochs < 0:
        raise SmootherError("skip_epochs must be non-negative")
    timestamps: list[int] = []
    current: int | None = None
    previous: int | None = None
    try:
        with path.open(encoding="utf-8-sig", newline="") as handle:
            reader = csv.DictReader(handle)
            fields = list(reader.fieldnames or ())
            if len(fields) != len(set(fields)):
                raise SmootherError(f"device GNSS CSV has duplicate fields: {path}")
            missing = {"MessageType", "utcTimeMillis"} - set(fields)
            if missing:
                raise SmootherError(
                    f"device GNSS CSV missing fields: {', '.join(sorted(missing))}"
                )
            for row_number, raw_row in enumerate(reader, start=2):
                if None in raw_row:
                    raise SmootherError(
                        f"device row {row_number}: more values than header fields"
                    )
                row = {key: (value or "") for key, value in raw_row.items()}
                if row.get("MessageType") != "Raw":
                    raise SmootherError(f"device row {row_number}: MessageType must be Raw")
                timestamp = _parse_int(row.get("utcTimeMillis", ""), "utcTimeMillis", row_number)
                if timestamp < 0:
                    raise SmootherError(f"device row {row_number}: utcTimeMillis must be non-negative")
                if previous is not None and timestamp < previous:
                    raise SmootherError(f"device row {row_number}: utcTimeMillis moved backwards")
                previous = timestamp
                if current is None or timestamp != current:
                    timestamps.append(timestamp)
                    current = timestamp
    except OSError as exc:
        raise SmootherError(f"failed to read device GNSS CSV {path}") from exc
    selected = timestamps[skip_epochs:]
    if not selected:
        raise SmootherError("skip_epochs removes all device GNSS epochs")
    if any(b <= a for a, b in zip(selected, selected[1:])):
        raise SmootherError("selected device epoch keys are not strictly increasing")
    return selected


def _wgs84_ecef_to_geodetic(ecef: np.ndarray) -> tuple[float, float, float]:
    """Convert ECEF to WGS84 geodetic coordinates with polar safeguards."""

    if ecef.shape != (3,) or not np.isfinite(ecef).all():
        raise SmootherError("ECEF coordinate is not finite")
    x, y, z = (float(value) for value in ecef)
    a = 6378137.0
    b = 6356752.314245179
    e2 = 1.0 - (b * b) / (a * a)
    p = math.hypot(x, y)
    if p < 1e-9:
        if abs(z) < 1e-9:
            return 0.0, 0.0, -a
        return math.copysign(math.pi / 2.0, z), 0.0, abs(z) - b
    longitude = math.atan2(y, x)
    latitude = math.atan2(z, p * (1.0 - e2))
    height = 0.0
    for _ in range(12):
        sin_lat = math.sin(latitude)
        cos_lat = math.cos(latitude)
        radius = a / math.sqrt(max(1e-30, 1.0 - e2 * sin_lat * sin_lat))
        if abs(cos_lat) > 1e-12:
            height = p / cos_lat - radius
        else:
            height = abs(z) - b
        denominator = p * (1.0 - e2 * radius / (radius + height))
        next_latitude = math.atan2(z, denominator)
        if abs(next_latitude - latitude) < 1e-14:
            latitude = next_latitude
            break
        latitude = next_latitude
    sin_lat = math.sin(latitude)
    radius = a / math.sqrt(max(1e-30, 1.0 - e2 * sin_lat * sin_lat))
    cos_lat = math.cos(latitude)
    height = p / cos_lat - radius if abs(cos_lat) > 1e-12 else abs(z) - b
    values = (latitude, longitude, height)
    if not all(math.isfinite(value) for value in values):
        raise SmootherError("ECEF-to-geodetic conversion was not finite")
    return values


def _wgs84_geodetic_to_ecef(latitude: float, longitude: float, height: float) -> np.ndarray:
    a = 6378137.0
    e2 = 6.6943799901413165e-3
    sin_lat = math.sin(latitude)
    cos_lat = math.cos(latitude)
    radius = a / math.sqrt(1.0 - e2 * sin_lat * sin_lat)
    return np.array(
        (
            (radius + height) * cos_lat * math.cos(longitude),
            (radius + height) * cos_lat * math.sin(longitude),
            (radius * (1.0 - e2) + height) * sin_lat,
        ),
        dtype=float,
    )


def _enu_rotation(latitude: float, longitude: float) -> np.ndarray:
    sin_lat, cos_lat = math.sin(latitude), math.cos(latitude)
    sin_lon, cos_lon = math.sin(longitude), math.cos(longitude)
    return np.array(
        (
            (-sin_lon, cos_lon, 0.0),
            (-sin_lat * cos_lon, -sin_lat * sin_lon, cos_lat),
            (cos_lat * cos_lon, cos_lat * sin_lon, sin_lat),
        ),
        dtype=float,
    )


def _device_time_to_week_tow(timestamp_ms: int, leap_seconds: int) -> tuple[int, float]:
    gps_seconds = (
        Decimal(timestamp_ms) / 1000
        + Decimal(leap_seconds)
        - GPS_EPOCH_UNIX_SECONDS
    )
    week = int(gps_seconds // SECONDS_PER_WEEK)
    tow = float(gps_seconds - Decimal(week) * SECONDS_PER_WEEK)
    if week < 0 or not math.isfinite(tow) or not 0.0 <= tow < 604800.0:
        raise SmootherError("device epoch cannot be represented as GPS week/TOW")
    return week, tow


def _measurement_sigma(row: PositionRow, floor_m: float) -> float:
    # POS has no per-epoch covariance column.  Use solver quality fields only:
    # PDOP and satellite count scale a declared floor; no truth-derived value
    # enters this covariance proxy.
    pdop = row.pdop if math.isfinite(row.pdop) and row.pdop > 0.0 else 10.0
    satellites = max(row.satellites, 4)
    sigma = 0.75 * pdop * math.sqrt(8.0 / satellites)
    if row.status == 0:
        sigma = max(sigma, 10.0 * floor_m)
    return max(floor_m, sigma)


def _transition(dt: float, process_noise: float) -> tuple[np.ndarray, np.ndarray]:
    if not math.isfinite(dt) or dt <= 0.0:
        raise SmootherError("timestamp interval must be finite and positive")
    identity = np.eye(3, dtype=float)
    transition = np.eye(6, dtype=float)
    transition[:3, 3:] = identity * dt
    q = float(process_noise)
    process = np.zeros((6, 6), dtype=float)
    process[:3, :3] = identity * (q * dt**3 / 3.0)
    process[:3, 3:] = identity * (q * dt**2 / 2.0)
    process[3:, :3] = identity * (q * dt**2 / 2.0)
    process[3:, 3:] = identity * (q * dt)
    return transition, process


def _symmetrize_covariance(covariance: np.ndarray) -> np.ndarray:
    result = (covariance + covariance.T) * 0.5
    if not np.isfinite(result).all():
        raise SmootherError("state covariance became non-finite")
    return result


def smooth_positions(
    positions: list[PositionRow],
    device_epochs: list[int],
    config: SmootherConfig,
    *,
    reset_indices: Iterable[int] = (),
    leap_seconds: int = DEFAULT_GPS_UTC_LEAP_SECONDS,
    process_noise_multipliers: list[float] | None = None,
) -> SmoothingResult:
    """Run a causal CV Kalman filter followed by per-segment RTS smoothing."""

    started = time.perf_counter()
    config.validate()
    if not isinstance(leap_seconds, int) or leap_seconds < 0:
        raise SmootherError("gps UTC leap seconds must be non-negative")
    if not device_epochs:
        raise SmootherError("device epoch list is empty")
    if any(b <= a for a, b in zip(device_epochs, device_epochs[1:])):
        raise SmootherError("device epoch keys must be strictly increasing")
    if process_noise_multipliers is not None:
        if len(process_noise_multipliers) != len(device_epochs):
            raise SmootherError(
                "process noise multiplier count must equal selected device epochs"
            )
        if any(
            not math.isfinite(value) or value <= 0.0
            for value in process_noise_multipliers
        ):
            raise SmootherError("process noise multipliers must be finite and positive")
    position_by_timestamp = {row.timestamp_ms: row for row in positions}
    unknown = sorted(set(position_by_timestamp) - set(device_epochs))
    if unknown:
        raise SmootherError(
            "position timestamp is not an exact selected device epoch: "
            f"{unknown[0]}"
        )
    observations: list[PositionRow | None] = [
        position_by_timestamp.get(timestamp) for timestamp in device_epochs
    ]
    measured_indices = [index for index, row in enumerate(observations) if row is not None]
    if not measured_indices:
        raise SmootherError("no position row matches selected device epochs")
    first_measurement = observations[measured_indices[0]]
    assert first_measurement is not None
    origin_ecef = first_measurement.ecef.astype(float, copy=True)
    origin_latitude, origin_longitude, origin_height = _wgs84_ecef_to_geodetic(origin_ecef)
    rotation = _enu_rotation(origin_latitude, origin_longitude)
    measurements: list[np.ndarray | None] = [
        None
        if row is None
        else rotation @ (row.ecef - origin_ecef)
        for row in observations
    ]
    if any(value is not None and not np.isfinite(value).all() for value in measurements):
        raise SmootherError("local ENU measurement is not finite")

    n = len(device_epochs)
    requested_resets = set(reset_indices)
    if any(index <= 0 or index >= n for index in requested_resets):
        raise SmootherError("reset index must be between 1 and device epoch count - 1")
    automatic_resets = {
        index
        for index in range(1, n)
        if (device_epochs[index] - device_epochs[index - 1]) / 1000.0
        > config.segment_gap_s
    }
    reset_set = requested_resets | automatic_resets
    segment_starts = sorted({0, *reset_set})
    segment_for_index: list[int] = [0] * n
    for segment_id, start in enumerate(segment_starts):
        end = segment_starts[segment_id + 1] if segment_id + 1 < len(segment_starts) else n
        for index in range(start, end):
            segment_for_index[index] = segment_id

    x_filtered = np.full((n, 6), np.nan, dtype=float)
    p_filtered = np.full((n, 6, 6), np.nan, dtype=float)
    x_predicted = np.full((n, 6), np.nan, dtype=float)
    p_predicted = np.full((n, 6, 6), np.nan, dtype=float)
    transitions: list[np.ndarray | None] = [None] * n
    used_measurement = [False] * n
    outlier_rejected = [False] * n
    reacquired = [False] * n
    innovation_sigma: list[float | None] = [None] * n
    numerical_fallbacks = 0
    outlier_count = 0
    reacquisition_indices: set[int] = set()
    maximum_reject_run = 0
    maximum_prediction_duration = 0.0

    for segment_id, start in enumerate(segment_starts):
        end = segment_starts[segment_id + 1] if segment_id + 1 < len(segment_starts) else n
        anchor = next((index for index in range(start, end) if measurements[index] is not None), None)
        if anchor is None:
            continue
        anchor_row = observations[anchor]
        assert anchor_row is not None
        sigma = _measurement_sigma(anchor_row, config.measurement_floor_m)
        x = np.zeros(6, dtype=float)
        x[:3] = measurements[anchor]
        covariance = np.diag(
            [sigma * sigma] * 3 + [100.0 * 100.0] * 3
        ).astype(float)
        x_predicted[anchor] = x
        p_predicted[anchor] = covariance
        x_filtered[anchor] = x
        p_filtered[anchor] = covariance
        used_measurement[anchor] = True
        innovation_sigma[anchor] = 0.0
        consecutive_rejects = 0
        prediction_duration_s = 0.0
        for index in range(anchor + 1, end):
            dt = (device_epochs[index] - device_epochs[index - 1]) / 1000.0
            multiplier = (
                process_noise_multipliers[index]
                if process_noise_multipliers is not None
                else 1.0
            )
            transition, process = _transition(
                dt, config.process_noise * multiplier
            )
            transitions[index] = transition
            x_prior = transition @ x
            p_prior = _symmetrize_covariance(transition @ covariance @ transition.T + process)
            x_predicted[index] = x_prior
            p_predicted[index] = p_prior
            measurement = measurements[index]
            row = observations[index]
            if measurement is None or row is None:
                prediction_duration_s += dt
                maximum_prediction_duration = max(
                    maximum_prediction_duration, prediction_duration_s
                )
                x, covariance = x_prior, p_prior
                x_filtered[index], p_filtered[index] = x, covariance
                continue
            measurement_sigma = _measurement_sigma(row, config.measurement_floor_m)
            measurement_covariance = np.eye(3, dtype=float) * (measurement_sigma**2)
            innovation = measurement - x_prior[:3]
            innovation_covariance = p_prior[:3, :3] + measurement_covariance
            try:
                solved = np.linalg.solve(innovation_covariance, innovation)
                normalized = float(math.sqrt(max(0.0, float(innovation @ solved))))
            except np.linalg.LinAlgError:
                numerical_fallbacks += 1
                solved = np.linalg.pinv(innovation_covariance) @ innovation
                normalized = float(math.sqrt(max(0.0, float(innovation @ solved))))
            innovation_sigma[index] = normalized if math.isfinite(normalized) else None
            if not math.isfinite(normalized) or normalized > config.outlier_gate_sigma:
                outlier_count += 1
                outlier_rejected[index] = True
                consecutive_rejects += 1
                prediction_duration_s += dt
                maximum_reject_run = max(maximum_reject_run, consecutive_rejects)
                maximum_prediction_duration = max(
                    maximum_prediction_duration, prediction_duration_s
                )
                reject_limit_reached = (
                    config.max_consecutive_rejects is not None
                    and consecutive_rejects >= config.max_consecutive_rejects
                )
                duration_limit_reached = (
                    config.max_prediction_duration_s is not None
                    and prediction_duration_s >= config.max_prediction_duration_s
                )
                if (
                    (reject_limit_reached or duration_limit_reached)
                ):
                    # A truth-free reacquisition uses the current observation
                    # as a new position anchor after a bounded prediction
                    # interval.  The gate event remains counted above, but
                    # this row is published as a usable reacquired fix.
                    sigma = _measurement_sigma(row, config.measurement_floor_m)
                    x = np.zeros(6, dtype=float)
                    x[:3] = measurement
                    covariance = np.diag(
                        [sigma * sigma] * 3 + [100.0 * 100.0] * 3
                    ).astype(float)
                    transitions[index] = None
                    x_filtered[index], p_filtered[index] = x, covariance
                    p_predicted[index] = p_prior
                    used_measurement[index] = True
                    outlier_rejected[index] = False
                    reacquired[index] = True
                    reacquisition_indices.add(index)
                    consecutive_rejects = 0
                    prediction_duration_s = 0.0
                    continue
                x, covariance = x_prior, p_prior
                x_filtered[index], p_filtered[index] = x, covariance
                continue
            gain = np.linalg.solve(innovation_covariance, p_prior[:3, :]).T
            x = x_prior + gain @ innovation
            covariance = _symmetrize_covariance(p_prior - gain @ innovation_covariance @ gain.T)
            used_measurement[index] = True
            x_filtered[index], p_filtered[index] = x, covariance
            consecutive_rejects = 0
            prediction_duration_s = 0.0

        # RTS backward pass is isolated to this segment.  This prevents a
        # long device pause or an explicitly requested train/validation
        # boundary from leaking state across the boundary.
        last = end - 1
        if not np.isfinite(x_filtered[last]).all():
            # No anchor after a gap: leave it for the nearest-anchor fallback.
            continue
        x_smoothed = x_filtered.copy()
        p_smoothed = p_filtered.copy()
        for index in range(last - 1, anchor - 1, -1):
            if index in reacquisition_indices:
                # Keep the first observation after a bounded prediction
                # interval as the new anchor; neither past nor future RTS
                # information may overwrite the reacquisition point.
                continue
            next_index = index + 1
            transition = transitions[next_index]
            if transition is None or not np.isfinite(p_predicted[next_index]).all():
                continue
            try:
                smoother_gain = np.linalg.solve(
                    p_predicted[next_index].T,
                    (p_filtered[index] @ transition.T).T,
                ).T
            except np.linalg.LinAlgError:
                numerical_fallbacks += 1
                smoother_gain = p_filtered[index] @ transition.T @ np.linalg.pinv(
                    p_predicted[next_index]
                )
            x_smoothed[index] = x_filtered[index] + smoother_gain @ (
                x_smoothed[next_index] - x_predicted[next_index]
            )
            p_smoothed[index] = _symmetrize_covariance(
                p_filtered[index]
                + smoother_gain
                @ (p_smoothed[next_index] - p_predicted[next_index])
                @ smoother_gain.T
            )
        x_filtered = x_smoothed
        p_filtered = p_smoothed

    # Fill positions before/after an unanchored segment from the nearest
    # available state.  They are explicitly marked predicted, not measured.
    valid_state_indices = [index for index in range(n) if np.isfinite(x_filtered[index]).all()]
    if not valid_state_indices:
        raise SmootherError("filter produced no finite state")
    for index in range(n):
        if np.isfinite(x_filtered[index]).all():
            continue
        nearest = min(valid_state_indices, key=lambda candidate: abs(candidate - index))
        x_filtered[index] = x_filtered[nearest]
        p_filtered[index] = p_filtered[nearest]

    effective_reset_indices = sorted(reset_set | reacquisition_indices)
    effective_segment_starts = [0, *effective_reset_indices]
    segment_for_index: list[int] = [0] * n
    for segment_id, start in enumerate(effective_segment_starts):
        end = (
            effective_segment_starts[segment_id + 1]
            if segment_id + 1 < len(effective_segment_starts)
            else n
        )
        for index in range(start, end):
            segment_for_index[index] = segment_id

    rows: list[SmoothedRow] = []
    for index, timestamp in enumerate(device_epochs):
        enu = x_filtered[index][:3]
        ecef = origin_ecef + rotation.T @ enu
        if not np.isfinite(ecef).all():
            raise SmootherError(f"output epoch {timestamp}: ECEF is non-finite")
        latitude_rad, longitude_rad, height = _wgs84_ecef_to_geodetic(ecef)
        row = observations[index]
        if row is not None and used_measurement[index] and not outlier_rejected[index]:
            week, tow = row.week, row.tow
            status = row.status
            satellites = row.satellites
            pdop = row.pdop
            ratio = row.ratio
            fixed = row.fixed_ambiguities
            iterations = row.iterations
            source = "reacquired" if reacquired[index] else "measured"
            measurement_flag = True
        else:
            week, tow = _device_time_to_week_tow(timestamp, leap_seconds)
            status = PROPAGATED_STATUS
            satellites = 0
            pdop = math.sqrt(max(0.0, float(np.trace(p_filtered[index][:3, :3]))))
            ratio = 0.0
            fixed = 0
            iterations = 0
            source = "outlier_rejected" if outlier_rejected[index] else (
                "interpolated" if row is None else "predicted"
            )
            measurement_flag = False
        position_sigma = math.sqrt(max(0.0, float(np.trace(p_filtered[index][:3, :3]) / 3.0)))
        if not math.isfinite(position_sigma):
            raise SmootherError(f"output epoch {timestamp}: covariance is non-finite")
        rows.append(
            SmoothedRow(
                timestamp,
                week,
                tow,
                ecef,
                math.degrees(latitude_rad),
                math.degrees(longitude_rad),
                height,
                status,
                satellites,
                pdop,
                ratio,
                fixed,
                iterations,
                source,
                segment_for_index[index],
                measurement_flag,
                outlier_rejected[index],
                innovation_sigma[index],
                position_sigma,
                reacquired[index],
            )
        )
    max_input_gap_s = max(
        ((b - a) / 1000.0 for a, b in zip(device_epochs, device_epochs[1:])),
        default=0.0,
    )
    max_position_gap_s = max(
        (
            (b.timestamp_ms - a.timestamp_ms) / 1000.0
            for a, b in zip(positions, positions[1:])
        ),
        default=0.0,
    )
    return SmoothingResult(
        rows,
        origin_ecef,
        math.degrees(origin_latitude),
        math.degrees(origin_longitude),
        origin_height,
        n,
        len(measured_indices),
        n - len(measured_indices),
        outlier_count,
        len(effective_segment_starts),
        max_input_gap_s,
        max_position_gap_s,
        tuple(effective_reset_indices),
        numerical_fallbacks,
        time.perf_counter() - started,
        tuple(sorted(reacquisition_indices)),
        maximum_reject_run,
        maximum_prediction_duration,
    )


def _write_pos(path: Path, rows: list[SmoothedRow]) -> None:
    lines = [
        "% LibGNSS++ truth-free smartphone trajectory smoother",
        "% Columns: GPS_Week GPS_TOW X(m) Y(m) Z(m) Lat(deg) Lon(deg) Height(m) Status Satellites PDOP Ratio FixedAmbiguities Iterations",
        "% Status 7 is PROPAGATED and denotes a synthesized or rejected input epoch.",
    ]
    for row in rows:
        lines.append(
            f"{row.week:d} {row.tow:.6f} "
            f"{row.ecef[0]:.6f} {row.ecef[1]:.6f} {row.ecef[2]:.6f} "
            f"{row.latitude:.9f} {row.longitude:.9f} {row.height:.6f} "
            f"{row.status:d} {row.satellites:d} {row.pdop:.6f} "
            f"{row.ratio:.6f} {row.fixed_ambiguities:d} {row.iterations:d}"
        )
    _atomic_write(path, ("\n".join(lines) + "\n").encode("ascii"))


def _write_trajectory_csv(path: Path, rows: list[SmoothedRow]) -> None:
    output = [
        "UnixTimeMillis,LatitudeDegrees,LongitudeDegrees,AltitudeMeters,"
        "X_ECEF_m,Y_ECEF_m,Z_ECEF_m,source,segment_id,measurement_used,"
        "outlier_rejected,innovation_sigma,position_sigma_m,status,satellites,PDOP"
    ]
    for row in rows:
        innovation = "" if row.innovation_sigma is None else f"{row.innovation_sigma:.9f}"
        output.append(
            f"{row.timestamp_ms},{row.latitude:.9f},{row.longitude:.9f},{row.height:.6f},"
            f"{row.ecef[0]:.6f},{row.ecef[1]:.6f},{row.ecef[2]:.6f},"
            f"{row.source},{row.segment_id},{int(row.measurement_used)},"
            f"{int(row.outlier_rejected)},{innovation},{row.position_sigma_m:.9f},"
            f"{row.status},{row.satellites},{row.pdop:.6f}"
        )
    _atomic_write(path, ("\n".join(output) + "\n").encode("utf-8"))


def _summary(
    result: SmoothingResult,
    config: SmootherConfig,
    motion_summary: dict[str, Any] | None = None,
) -> dict[str, Any]:
    source_counts: dict[str, int] = {}
    for row in result.rows:
        source_counts[row.source] = source_counts.get(row.source, 0) + 1
    output_gaps = [
        (result.rows[index].timestamp_ms - result.rows[index - 1].timestamp_ms) / 1000.0
        for index in range(1, len(result.rows))
    ]
    summary = {
        "schema_version": SCHEMA_VERSION,
        "algorithm": {
            "name": "constant-velocity Kalman filter plus per-segment RTS smoother",
            "state": "ENU position and ENU velocity",
            "measurement": "ECEF solver position transformed to a WGS84 local ENU frame",
            "truth_used": False,
            "process_noise_parameter": config.process_noise,
            "measurement_floor_m": config.measurement_floor_m,
            "outlier_gate_sigma": config.outlier_gate_sigma,
            "segment_gap_s": config.segment_gap_s,
            "outlier_policy": "Mahalanobis position innovation; rejected rows are predicted and marked PROPAGATED",
            "gap_policy": "device epoch keys define output; RTS fills internal missing keys, segment reset prevents cross-gap leakage",
            "covariance_policy": "PDOP/satellite quality proxy with declared floor; no covariance or truth is inferred from labels",
        },
        "frame": {
            "local_frame": "WGS84 ENU",
            "origin_latitude_deg": result.origin_latitude,
            "origin_longitude_deg": result.origin_longitude,
            "origin_height_m": result.origin_height,
            "ecef_reconstruction": "origin_ecef + ENU_rotation.T @ smoothed_enu",
        },
        "populations": {
            "selected_device_epochs": result.selected_device_epochs,
            "input_position_epochs": result.measured_epochs,
            "synthesized_epochs": result.synthesized_epochs,
            "outlier_rejections": result.outlier_rejections,
            "output_epochs": len(result.rows),
            "availability_ratio": len(result.rows) / result.selected_device_epochs,
            "source_counts": source_counts,
            "segment_count": result.segment_count,
            "explicit_reset_indices": list(result.reset_indices),
            "numerical_fallbacks": result.numerical_fallbacks,
        },
        "timing": {"smoothing_wall_s": result.elapsed_s},
        "gaps": {
            "max_device_epoch_gap_s": result.max_input_gap_s,
            "max_position_input_gap_s": result.max_position_gap_s,
            "max_output_gap_s": max(output_gaps, default=0.0),
            "output_keys_strictly_increasing": all(
                b > a
                for a, b in zip(
                    (row.timestamp_ms for row in result.rows),
                    (row.timestamp_ms for row in result.rows[1:]),
                )
            ),
        },
    }
    if (
        config.max_consecutive_rejects is not None
        or config.max_prediction_duration_s is not None
        or result.reacquisition_indices
    ):
        summary["reacquisition"] = {
            "max_consecutive_rejects": config.max_consecutive_rejects,
            "max_prediction_duration_s": config.max_prediction_duration_s,
            "reacquisition_count": len(result.reacquisition_indices),
            "reacquisition_indices": list(result.reacquisition_indices),
            "maximum_observed_reject_run": result.max_consecutive_rejects,
            "maximum_observed_prediction_duration_s": result.max_prediction_duration_s,
            "policy": "reset to current finite observation after either bounded reject run or prediction duration; no truth used",
        }
    if motion_summary is not None:
        summary["motion_adaptive_process_noise"] = motion_summary
    return summary


def _write_motion_profile_csv(
    path: Path, motion_profile_rows: list[dict[str, Any]]
) -> None:
    fields = (
        "timestamp_ms",
        "process_noise_multiplier",
        "dynamic",
        "gyro_activity",
        "accel_dynamic",
        "gyro_samples",
        "accel_samples",
        "window_start_ms",
        "latest_sample_age_ms",
        "fallback_to_baseline",
        "fallback_reason",
        "gap_detected",
        "clock_discontinuity_detected",
        "nonfinite_detected",
    )
    output = io.StringIO(newline="")
    writer = csv.DictWriter(output, fieldnames=fields, lineterminator="\n")
    writer.writeheader()
    for row in motion_profile_rows:
        if not set(fields).issubset(row):
            missing = sorted(set(fields) - set(row))
            raise SmootherError(
                "motion profile row missing fields: " + ", ".join(missing)
            )
        writer.writerow({field: row.get(field) for field in fields})
    _atomic_write(path, output.getvalue().encode("utf-8"))


def _git_revision() -> str | None:
    try:
        result = subprocess.run(
            ["git", "rev-parse", "HEAD"],
            cwd=ROOT,
            text=True,
            capture_output=True,
            check=False,
        )
    except OSError:
        return None
    revision = result.stdout.strip()
    return revision if result.returncode == 0 and revision else None


def _load_profile(path: Path | None, role: str | None) -> dict[str, Any] | None:
    if path is None:
        if role is not None and role != "development":
            raise SmootherError("only development role is permitted")
        return None
    _require_file(path, "profile")
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise SmootherError(f"invalid profile: {path}") from exc
    if not isinstance(payload, dict) or payload.get("schema_version") != "smartphone-r5-profile.v1":
        raise SmootherError("profile schema is not smartphone-r5-profile.v1")
    if role != "development":
        raise SmootherError("trajectory smoothing is development-only; role must be development")
    dataset = payload.get("datasets", {}).get("development") if isinstance(payload.get("datasets"), dict) else None
    if not isinstance(dataset, dict):
        raise SmootherError("profile has no development dataset")
    return {
        "path": path,
        "sha256": _sha256(path),
        "profile_id": payload.get("profile_id"),
        "role": role,
        "dataset": dataset,
    }


def write_outputs(
    result: SmoothingResult,
    config: SmootherConfig,
    output_dir: Path,
    *,
    position_path: Path,
    device_path: Path,
    profile: dict[str, Any] | None = None,
    submission_output: Path | None = None,
    phone: str | None = None,
    dataset_id: str | None = None,
    skip_epochs: int = 0,
    leap_seconds: int = DEFAULT_GPS_UTC_LEAP_SECONDS,
    motion_summary: dict[str, Any] | None = None,
    motion_profile_rows: list[dict[str, Any]] | None = None,
) -> dict[str, Any]:
    """Atomically publish smoother artifacts and optionally a submission."""

    output_dir.mkdir(parents=True, exist_ok=True)
    pos_output = output_dir / "smoothed.pos"
    trajectory_output = output_dir / "trajectory.csv"
    summary_output = output_dir / "smoother_summary.json"
    motion_output: Path | None = None
    if (motion_summary is None) != (motion_profile_rows is None):
        raise SmootherError(
            "motion_summary and motion_profile_rows must be provided together"
        )
    if motion_profile_rows is not None:
        if len(motion_profile_rows) != len(result.rows):
            raise SmootherError(
                "motion profile row count must equal smoother output rows"
            )
        for row, motion_row in zip(result.rows, motion_profile_rows):
            if int(motion_row.get("timestamp_ms", -1)) != row.timestamp_ms:
                raise SmootherError("motion profile timestamp keys do not match output")
        motion_output = output_dir / "motion_profile.csv"
        _write_motion_profile_csv(motion_output, motion_profile_rows)
    _write_pos(pos_output, result.rows)
    _write_trajectory_csv(trajectory_output, result.rows)
    summary = _summary(result, config, motion_summary)
    _atomic_write(summary_output, _json_bytes(summary))

    submission_manifest: dict[str, Any] | None = None
    if submission_output is not None:
        if not phone:
            raise SmootherError("--phone is required with --submission-output")
        sys.path.insert(0, str(ROOT / "apps" / "commands" / "benchmarks"))
        try:
            import gnss_smartphone_kaggle as kaggle

            submission_manifest = kaggle.generate_submission(
                pos_output,
                submission_output,
                phone,
                device_gnss_path=device_path,
                profile_path=profile["path"] if profile else None,
                role=profile["role"] if profile else None,
                dataset_id=dataset_id,
                skip_epochs=skip_epochs,
                gps_utc_leap_seconds=leap_seconds,
            )
        except (ImportError, ValueError) as exc:
            raise SmootherError(f"truth-free submission generation failed: {exc}") from exc

    artifacts: dict[str, Any] = {
        "smoothed_pos": {
            "path": str(pos_output),
            "sha256": _sha256(pos_output),
            "bytes": pos_output.stat().st_size,
        },
        "trajectory_csv": {
            "path": str(trajectory_output),
            "sha256": _sha256(trajectory_output),
            "bytes": trajectory_output.stat().st_size,
        },
        "summary": {
            "path": str(summary_output),
            "sha256": _sha256(summary_output),
            "bytes": summary_output.stat().st_size,
        },
    }
    if submission_output is not None and submission_manifest is not None:
        submission_manifest_path = submission_output.with_name(
            submission_output.name + ".manifest.json"
        )
        artifacts["submission"] = {
            "path": str(submission_output),
            "sha256": _sha256(submission_output),
            "bytes": submission_output.stat().st_size,
            "manifest": {
                "path": str(submission_manifest_path),
                "sha256": _sha256(submission_manifest_path),
            },
        }
    if motion_output is not None:
        artifacts["motion_profile"] = {
            "path": str(motion_output),
            "sha256": _sha256(motion_output),
            "bytes": motion_output.stat().st_size,
        }
    manifest = {
        "schema_version": MANIFEST_SCHEMA_VERSION,
        "truth_used": False,
        "experimental_flag": "--experimental-truth-free-kalman-rts",
        "inputs": {
            "position": {"path": str(position_path), "sha256": _sha256(position_path)},
            "device_gnss": {"path": str(device_path), "sha256": _sha256(device_path)},
            "profile": (
                {
                    "path": str(profile["path"]),
                    "sha256": profile["sha256"],
                    "profile_id": profile["profile_id"],
                    "role": profile["role"],
                }
                if profile
                else None
            ),
        },
        "configuration": {
            "process_noise": config.process_noise,
            "measurement_floor_m": config.measurement_floor_m,
            "outlier_gate_sigma": config.outlier_gate_sigma,
            "segment_gap_s": config.segment_gap_s,
            "gps_utc_leap_seconds": leap_seconds,
            "skip_epochs": skip_epochs,
        },
        "software": {"schema_version": SCHEMA_VERSION, "git_revision": _git_revision()},
        "summary": summary,
        "artifacts": artifacts,
        "submission": {
            "truth_free": True,
            "generator_manifest": submission_manifest,
        }
        if submission_manifest is not None
        else None,
    }
    if motion_summary is not None:
        manifest["motion_adaptive_process_noise"] = motion_summary
        manifest["motion_experimental_flag"] = "--experimental-motion-adaptive-q"
    if config.max_consecutive_rejects is not None:
        manifest["configuration"]["max_consecutive_rejects"] = (
            config.max_consecutive_rejects
        )
    if config.max_prediction_duration_s is not None:
        manifest["configuration"]["max_prediction_duration_s"] = (
            config.max_prediction_duration_s
        )
    if (
        config.max_consecutive_rejects is not None
        or config.max_prediction_duration_s is not None
        or result.reacquisition_indices
    ):
        manifest["reacquisition"] = {
            "reacquisition_count": len(result.reacquisition_indices),
            "reacquisition_indices": list(result.reacquisition_indices),
            "maximum_observed_reject_run": result.max_consecutive_rejects,
            "maximum_observed_prediction_duration_s": result.max_prediction_duration_s,
        }
    manifest_path = output_dir / "smoother_manifest.json"
    _atomic_write(manifest_path, _json_bytes(manifest))
    return manifest


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        prog=os.environ.get("GNSS_CLI_NAME", "gnss smartphone-trajectory-smoother")
    )
    parser.add_argument(
        "--experimental-truth-free-kalman-rts",
        action="store_true",
        help="required opt-in for the development-only truth-free smoother",
    )
    parser.add_argument("--position", type=Path, required=True)
    parser.add_argument("--device-gnss", type=Path, required=True)
    parser.add_argument("--device-imu", type=Path)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--profile", type=Path)
    parser.add_argument("--role", choices=("development", "holdout"), default="development")
    parser.add_argument("--skip-epochs", type=int)
    parser.add_argument("--gps-utc-leap-seconds", type=int, default=DEFAULT_GPS_UTC_LEAP_SECONDS)
    parser.add_argument("--process-noise", type=float, default=0.1)
    parser.add_argument("--measurement-floor-m", type=float, default=1.0)
    parser.add_argument("--outlier-gate-sigma", type=float, default=5.0)
    parser.add_argument("--segment-gap-s", type=float, default=10.0)
    parser.add_argument(
        "--max-consecutive-rejects",
        type=int,
        help="optional truth-free reacquisition bound in consecutive gated observations",
    )
    parser.add_argument(
        "--max-prediction-duration-s",
        type=float,
        help="optional truth-free reacquisition bound in seconds without an accepted observation",
    )
    parser.add_argument(
        "--experimental-motion-adaptive-q",
        action="store_true",
        help="required opt-in when --device-imu enables causal motion-adaptive process noise",
    )
    parser.add_argument("--motion-window-s", type=float, default=1.0)
    parser.add_argument("--motion-max-sample-gap-s", type=float, default=0.25)
    parser.add_argument("--motion-max-sample-age-s", type=float, default=0.25)
    parser.add_argument("--motion-gyro-threshold", type=float, default=0.15)
    parser.add_argument(
        "--motion-accel-dynamic-threshold", type=float, default=0.75
    )
    parser.add_argument("--motion-q-multiplier", type=float, default=2.0)
    parser.add_argument("--reset-at-index", type=int, action="append", default=[])
    parser.add_argument("--phone")
    parser.add_argument("--submission-output", type=Path)
    parser.add_argument("--dataset-id")
    return parser.parse_args(argv)


def run(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    try:
        if not args.experimental_truth_free_kalman_rts:
            raise SmootherError(
                "refusing to run without --experimental-truth-free-kalman-rts"
            )
        if args.role != "development":
            raise SmootherError("trajectory smoothing is development-only; holdout is sealed")
        if args.device_imu is not None and not args.experimental_motion_adaptive_q:
            raise SmootherError(
                "refusing to use --device-imu without --experimental-motion-adaptive-q"
            )
        if args.experimental_motion_adaptive_q and args.device_imu is None:
            raise SmootherError(
                "--experimental-motion-adaptive-q requires --device-imu"
            )
        if args.gps_utc_leap_seconds < 0:
            raise SmootherError("gps UTC leap seconds must be non-negative")
        if args.skip_epochs is not None and args.skip_epochs < 0:
            raise SmootherError("skip_epochs must be non-negative")
        profile = _load_profile(args.profile, args.role)
        profile_skip = (
            int(profile["dataset"].get("skip_epochs", 0)) if profile else 0
        )
        skip_epochs = profile_skip if args.skip_epochs is None else args.skip_epochs
        if profile and args.skip_epochs is not None and args.skip_epochs != profile_skip:
            raise SmootherError("--skip-epochs cannot override the profile value")
        if profile:
            expected_hash = profile["dataset"].get("device_gnss_sha256")
            if expected_hash and _sha256(args.device_gnss) != expected_hash:
                raise SmootherError("device GNSS hash does not match development profile")
        positions = _read_positions(args.position, args.gps_utc_leap_seconds)
        device_epochs = _read_device_epochs(args.device_gnss, skip_epochs)
        motion = None
        motion_rows = None
        if args.device_imu is not None:
            sys.path.insert(0, str(ROOT / "apps" / "commands" / "benchmarks"))
            import gnss_smartphone_imu as imu

            imu_dataset = imu.read_imu(args.device_imu)
            motion_config = imu.MotionConfig(
                window_s=args.motion_window_s,
                max_sample_gap_s=args.motion_max_sample_gap_s,
                max_sample_age_s=args.motion_max_sample_age_s,
                gyro_threshold=args.motion_gyro_threshold,
                accel_dynamic_threshold=args.motion_accel_dynamic_threshold,
                motion_q_multiplier=args.motion_q_multiplier,
            )
            motion = imu.build_motion_profile(
                device_epochs,
                imu_dataset,
                motion_config,
                reset_indices=tuple(args.reset_at_index),
            )
            motion_rows = imu.profile_rows(motion)
        config = SmootherConfig(
            args.process_noise,
            args.measurement_floor_m,
            args.outlier_gate_sigma,
            args.segment_gap_s,
            args.max_consecutive_rejects,
            args.max_prediction_duration_s,
        )
        result = smooth_positions(
            positions,
            device_epochs,
            config,
            reset_indices=args.reset_at_index,
            leap_seconds=args.gps_utc_leap_seconds,
            process_noise_multipliers=motion.multipliers if motion else None,
        )
        manifest = write_outputs(
            result,
            config,
            args.output_dir,
            position_path=args.position,
            device_path=args.device_gnss,
            profile=profile,
            submission_output=args.submission_output,
            phone=args.phone,
            dataset_id=args.dataset_id,
            skip_epochs=skip_epochs,
            leap_seconds=args.gps_utc_leap_seconds,
            motion_summary=motion.summary if motion else None,
            motion_profile_rows=motion_rows,
        )
    except (OSError, SmootherError, ValueError) as exc:
        print(f"Smartphone trajectory smoother failed: {exc}", file=sys.stderr)
        return 1
    print(f"Smartphone trajectory smoother published: {args.output_dir / 'smoother_manifest.json'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(run())
