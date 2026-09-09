#!/usr/bin/env python3
"""Truth-free extraction of Android handset WLS ECEF positions.

The Android WLS fields repeat once per raw measurement row.  This module
collapses them to one position per device epoch only after checking that all
finite rows in the epoch agree, that the ECEF coordinate is earth-plausible,
and that timestamp/clock state contracts are monotonic.  It then publishes a
WGS84 geodetic CSV, a native POS file, and an integrity manifest.  Ground truth
is deliberately not accepted or read by this command.
"""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
import hashlib
import json
import math
import os
from pathlib import Path
import re
import sys
import tempfile
import time
from typing import Any

import numpy as np

from support.gnss_runtime import application_root


ROOT = application_root(__file__)
BENCHMARK_DIR = ROOT / "apps" / "commands" / "benchmarks"
if str(BENCHMARK_DIR) not in sys.path:
    sys.path.insert(0, str(BENCHMARK_DIR))
import gnss_smartphone_kaggle as kaggle  # noqa: E402
import gnss_smartphone_trajectory_smoother as smoother  # noqa: E402


SCHEMA_VERSION = "smartphone-wls-position.v1"
MANIFEST_SCHEMA_VERSION = "smartphone-wls-position-manifest.v1"
DEFAULT_LEAP_SECONDS = 18
DEFAULT_CONSISTENCY_TOLERANCE_M = 1.0e-3
TIMESTAMP_GAP_THRESHOLD_MS = 1_500
ECEF_NORM_MIN_M = 6_000_000.0
ECEF_NORM_MAX_M = 7_500_000.0
ECEF_COMPONENT_MAX_M = 7_500_000.0
EXPECTED_FIELDS = (
    "MessageType",
    "utcTimeMillis",
    "HardwareClockDiscontinuityCount",
    "Svid",
    "WlsPositionXEcefMeters",
    "WlsPositionYEcefMeters",
    "WlsPositionZEcefMeters",
)
MULTI_PHONE_HOLDOUT_FREEZE_SCHEMA = (
    "smartphone-r5-wls-multi-phone-ensemble-holdout-freeze.v1.1"
)
MULTI_PHONE_HOLDOUT_FREEZE_MANIFEST_SCHEMA = (
    "smartphone-r5-wls-multi-phone-ensemble-holdout-freeze-manifest.v1.1"
)
MULTI_PHONE_HOLDOUT_FREEZE_SCHEMA_V1_2 = (
    "smartphone-r5-wls-multi-phone-ensemble-holdout-freeze.v1.2"
)
MULTI_PHONE_HOLDOUT_FREEZE_MANIFEST_SCHEMA_V1_2 = (
    "smartphone-r5-wls-multi-phone-ensemble-holdout-freeze-manifest.v1.2"
)
MULTI_PHONE_HOLDOUT_FREEZE_SCHEMA_V1_3 = (
    "smartphone-r5-wls-multi-phone-ensemble-holdout-freeze.v1.3"
)
MULTI_PHONE_HOLDOUT_FREEZE_MANIFEST_SCHEMA_V1_3 = (
    "smartphone-r5-wls-multi-phone-ensemble-holdout-freeze-manifest.v1.3"
)
MULTI_PHONE_HOLDOUT_FREEZE_SCHEMA_V1_4 = (
    "smartphone-r5-wls-multi-phone-ensemble-holdout-freeze.v1.4"
)
MULTI_PHONE_HOLDOUT_FREEZE_MANIFEST_SCHEMA_V1_4 = (
    "smartphone-r5-wls-multi-phone-ensemble-holdout-freeze-manifest.v1.4"
)
TEST_WLS_AUTHORIZATION_SCHEMA = "smartphone-r5-wls-multi-phone-test-authorization.v1"
TEST_WLS_AUTHORIZATION_MANIFEST_SCHEMA = (
    "smartphone-r5-wls-multi-phone-test-authorization-manifest.v1"
)
TEST_WLS_ALIGNMENT_TOLERANCE_MS = 10
MULTI_PHONE_HOLDOUT_ROUTE = "2022-10-06-21-51-us-ca-mtv-n"
MULTI_PHONE_HOLDOUT_PHONE_ALLOWLIST = (
    "sm-a205u",
    "sm-a217m",
    "sm-a325f",
)
MULTI_PHONE_HOLDOUT_ALGORITHM_PARAMETERS = {
    "selected_method": "coordinate_wise_ecef_median",
    "baseline_method": "single_phone_wls",
    "alignment_tolerance_ms": 1,
    "minimum_aligned_phone_count": 1,
    "fallback_policy": (
        "when fewer than two phones align, use the target phone's finite raw WLS row; "
        "record the one-phone fallback count and never synthesize a missing row"
    ),
    "skip_epochs": 1,
    "max_epochs": -1,
    "consistency_tolerance_m": DEFAULT_CONSISTENCY_TOLERANCE_M,
    "leap_seconds": DEFAULT_LEAP_SECONDS,
    "timestamp_gap_threshold_ms": TIMESTAMP_GAP_THRESHOLD_MS,
}


def _multi_phone_algorithm_parameter_hash() -> str:
    canonical = json.dumps(
        MULTI_PHONE_HOLDOUT_ALGORITHM_PARAMETERS,
        sort_keys=True,
        separators=(",", ":"),
    ).encode("utf-8")
    return hashlib.sha256(canonical).hexdigest()


MULTI_PHONE_HOLDOUT_ALGORITHM_PARAMETER_HASH = _multi_phone_algorithm_parameter_hash()
# Sparse WLS handling is an explicitly authorized input-contract extension;
# the frozen fusion/alignment/numeric algorithm core remains byte-for-byte
# represented by the same parameter projection and hash.
MULTI_PHONE_HOLDOUT_ALGORITHM_CORE_HASH = MULTI_PHONE_HOLDOUT_ALGORITHM_PARAMETER_HASH
_INTEGER_RE = re.compile(r"^[+-]?\d+$")
_FLOAT_RE = re.compile(
    r"^[+-]?(?:(?:\d+(?:\.\d*)?)|(?:\.\d+))(?:[eE][+-]?\d+)?$"
)


class WlsPositionError(ValueError):
    """Raised when the WLS extraction contract cannot be proven."""

    def __init__(self, message: str, classification: str = "contract_error") -> None:
        super().__init__(message)
        self.classification = classification


@dataclass(frozen=True)
class WlsEpoch:
    timestamp_ms: int
    clock_discontinuity_count: int
    ecef: tuple[float, float, float]
    rows: int
    finite_rows: int
    svid_count: int
    first_source_row: int
    last_source_row: int


@dataclass(frozen=True)
class WlsExtraction:
    epochs: tuple[WlsEpoch, ...]
    input_rows: int
    input_epochs: int
    timestamp_gap_count: int
    max_timestamp_gap_s: float
    clock_transition_count: int
    clock_transition_timestamps: tuple[int, ...]
    classification_counts: dict[str, int]
    consistency_tolerance_m: float
    epoch_timestamps: tuple[int, ...]
    sparse_omission_records: tuple[dict[str, Any], ...]
    blank_wls_row_count: int
    allow_missing_wls_epochs: bool
    allow_invalid_wls_epochs: bool


def _sha256(path: Path) -> str:
    if not path.is_file():
        raise WlsPositionError(f"missing file: {path}")
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
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
            directory = os.open(str(path.parent), os.O_DIRECTORY)
            try:
                os.fsync(directory)
            finally:
                os.close(directory)
    except OSError as exc:
        raise WlsPositionError(f"atomic publish failed for {path}: {exc}") from exc
    finally:
        if descriptor is not None:
            os.close(descriptor)
        if temporary is not None:
            try:
                os.unlink(temporary)
            except FileNotFoundError:
                pass


def _parse_integer(raw: str, field: str, row_number: int) -> int:
    token = (raw or "").strip()
    if not _INTEGER_RE.fullmatch(token):
        raise WlsPositionError(
            f"row {row_number}: {field} must be a finite integer",
            f"invalid_{field}",
        )
    try:
        return int(token)
    except ValueError as exc:
        raise WlsPositionError(
            f"row {row_number}: {field} is not representable", f"invalid_{field}"
        ) from exc


def _parse_float(raw: str, field: str, row_number: int) -> float:
    token = (raw or "").strip()
    if not _FLOAT_RE.fullmatch(token):
        raise WlsPositionError(
            f"row {row_number}: {field} must be finite", f"invalid_{field}"
        )
    try:
        value = float(token)
    except ValueError as exc:
        raise WlsPositionError(
            f"row {row_number}: {field} must be finite", f"invalid_{field}"
        ) from exc
    if not math.isfinite(value):
        raise WlsPositionError(
            f"row {row_number}: {field} must be finite", f"invalid_{field}"
        )
    return value


def _validate_ecef(ecef: tuple[float, float, float], row_number: int) -> None:
    if not all(math.isfinite(value) for value in ecef):
        raise WlsPositionError(
            f"row {row_number}: WLS ECEF is non-finite", "nonfinite_wls_ecef"
        )
    if any(abs(value) > ECEF_COMPONENT_MAX_M for value in ecef):
        raise WlsPositionError(
            f"row {row_number}: WLS ECEF component is outside earth range",
            "ecef_out_of_range",
        )
    norm = math.sqrt(sum(value * value for value in ecef))
    if not ECEF_NORM_MIN_M <= norm <= ECEF_NORM_MAX_M:
        raise WlsPositionError(
            f"row {row_number}: WLS ECEF norm is outside earth range",
            "ecef_out_of_range",
        )


def _finish_epoch(
    timestamp_ms: int,
    clock_count: int,
    values: list[tuple[float, float, float]],
    svids: set[int],
    first_row: int,
    last_row: int,
    tolerance_m: float,
) -> WlsEpoch:
    if not values:
        raise WlsPositionError(
            f"epoch {timestamp_ms}: no complete WLS ECEF row",
            "missing_wls_epoch",
        )
    spreads = tuple(
        max(value[index] for value in values)
        - min(value[index] for value in values)
        for index in range(3)
    )
    if any(spread > tolerance_m for spread in spreads):
        raise WlsPositionError(
            f"epoch {timestamp_ms}: WLS ECEF rows disagree by {max(spreads):.6g} m",
            "inconsistent_epoch_wls",
        )
    return WlsEpoch(
        timestamp_ms=timestamp_ms,
        clock_discontinuity_count=clock_count,
        ecef=values[0],
        rows=last_row - first_row + 1,
        finite_rows=len(values),
        svid_count=len(svids),
        first_source_row=first_row,
        last_source_row=last_row,
    )


def extract_epochs(
    device_gnss: Path,
    *,
    consistency_tolerance_m: float = DEFAULT_CONSISTENCY_TOLERANCE_M,
    allow_missing_wls_epochs: bool = False,
    allow_invalid_wls_epochs: bool = False,
) -> WlsExtraction:
    """Extract and validate all WLS epochs without reading truth.

    A normal/single-phone extraction remains fail-closed for any blank WLS
    epoch.  The explicit sparse mode is consumed only by the sealed
    multi-phone orchestration, which authorizes it before calling this helper.
    In sparse mode an epoch with at least one complete finite WLS row is kept
    after checking agreement; an all-blank epoch is omitted and recorded.
    """

    if (
        not math.isfinite(consistency_tolerance_m)
        or consistency_tolerance_m < 0.0
    ):
        raise WlsPositionError("consistency tolerance must be finite and non-negative")
    if not isinstance(allow_missing_wls_epochs, bool):
        raise WlsPositionError("allow_missing_wls_epochs must be boolean")
    if not isinstance(allow_invalid_wls_epochs, bool):
        raise WlsPositionError("allow_invalid_wls_epochs must be boolean")
    if allow_invalid_wls_epochs and not allow_missing_wls_epochs:
        raise WlsPositionError(
            "allow_invalid_wls_epochs requires the sealed sparse mode",
            "invalid_sparse_contract",
        )
    if not device_gnss.is_file():
        raise WlsPositionError(f"missing device GNSS CSV: {device_gnss}")
    epochs: list[WlsEpoch] = []
    classification_counts = {
        "missing_wls_epoch": 0,
        "partial_wls_epoch": 0,
        "inconsistent_epoch_wls": 0,
        "nonfinite_wls_ecef": 0,
        "ecef_out_of_range": 0,
        "timestamp_gap": 0,
        "clock_discontinuity_transition": 0,
        "blank_wls_row": 0,
        "sparse_omitted_epoch": 0,
        "sparse_invalid_wls_epoch": 0,
    }
    input_rows = 0
    current_timestamp: int | None = None
    current_clock: int | None = None
    current_values: list[tuple[float, float, float]] = []
    current_svids: set[int] = set()
    current_first_row = 0
    previous_timestamp: int | None = None
    previous_clock: int | None = None
    timestamp_gap_count = 0
    max_timestamp_gap_s = 0.0
    clock_transitions: list[int] = []
    epoch_timestamps: list[int] = []
    sparse_omission_records: list[dict[str, Any]] = []
    blank_wls_row_count = 0
    current_blank_rows = 0
    current_invalid_rows: list[int] = []

    def finish_current(last_row: int) -> None:
        nonlocal current_timestamp, current_clock, current_values, current_svids
        nonlocal current_first_row, previous_timestamp, previous_clock
        nonlocal timestamp_gap_count, max_timestamp_gap_s
        nonlocal current_blank_rows, current_invalid_rows
        if current_timestamp is None or current_clock is None:
            return
        epoch: WlsEpoch | None = None
        if current_invalid_rows and allow_invalid_wls_epochs:
            classification_counts["sparse_invalid_wls_epoch"] += 1
            sparse_omission_records.append(
                {
                    "timestamp_ms": current_timestamp,
                    "reason": "out-of-earth-range WLS epoch omitted",
                    "invalid_row_count": len(current_invalid_rows),
                    "first_invalid_source_row": min(current_invalid_rows),
                    "last_invalid_source_row": max(current_invalid_rows),
                    "blank_row_count": current_blank_rows,
                }
            )
        elif current_values:
            try:
                epoch = _finish_epoch(
                    current_timestamp,
                    current_clock,
                    current_values,
                    current_svids,
                    current_first_row,
                    last_row,
                    consistency_tolerance_m,
                )
            except WlsPositionError as exc:
                classification_counts[exc.classification] = (
                    classification_counts.get(exc.classification, 0) + 1
                )
                raise
        elif allow_missing_wls_epochs:
            classification_counts["sparse_omitted_epoch"] += 1
            sparse_omission_records.append(
                {
                    "timestamp_ms": current_timestamp,
                    "reason": "all WLS coordinate fields blank",
                    "blank_row_count": current_blank_rows,
                }
            )
        else:
            classification_counts["missing_wls_epoch"] += 1
            raise WlsPositionError(
                f"epoch {current_timestamp}: WLS ECEF is missing_wls_epoch",
                "missing_wls_epoch",
            )
        if previous_timestamp is not None:
            gap_ms = current_timestamp - previous_timestamp
            gap_s = gap_ms / 1000.0
            if gap_ms <= 0 or not math.isfinite(gap_s):
                raise WlsPositionError(
                    f"epoch {current_timestamp}: timestamp is not strictly increasing",
                    "timestamp_not_increasing",
                )
            if gap_ms > TIMESTAMP_GAP_THRESHOLD_MS:
                timestamp_gap_count += 1
                classification_counts["timestamp_gap"] += 1
            max_timestamp_gap_s = max(max_timestamp_gap_s, gap_s)
        if previous_clock is not None and current_clock < previous_clock:
            raise WlsPositionError(
                f"epoch {current_timestamp}: hardware clock count moved backwards",
                "clock_discontinuity_backwards",
            )
        if previous_clock is not None and current_clock != previous_clock:
            clock_transitions.append(current_timestamp)
            classification_counts["clock_discontinuity_transition"] += 1
        if epoch is not None:
            epochs.append(epoch)
        previous_timestamp = current_timestamp
        previous_clock = current_clock
        current_timestamp = None
        current_clock = None
        current_values = []
        current_svids = set()
        current_first_row = 0
        current_blank_rows = 0
        current_invalid_rows = []

    try:
        with device_gnss.open(encoding="utf-8-sig", newline="") as handle:
            reader = csv.DictReader(handle)
            fields = list(reader.fieldnames or ())
            if len(fields) != len(set(fields)):
                raise WlsPositionError("device GNSS CSV has duplicate fields", "duplicate_fields")
            missing = [field for field in EXPECTED_FIELDS if field not in fields]
            if missing:
                raise WlsPositionError(
                    "device GNSS CSV missing fields: " + ", ".join(missing),
                    "missing_required_fields",
                )
            for row_number, raw_row in enumerate(reader, start=2):
                input_rows += 1
                if None in raw_row:
                    raise WlsPositionError(
                        f"row {row_number}: more values than CSV header fields",
                        "extra_fields",
                    )
                if raw_row.get("MessageType") != "Raw":
                    raise WlsPositionError(
                        f"row {row_number}: MessageType must be Raw",
                        "non_raw_message",
                    )
                timestamp = _parse_integer(
                    raw_row.get("utcTimeMillis", ""), "utcTimeMillis", row_number
                )
                if timestamp < 0:
                    raise WlsPositionError(
                        f"row {row_number}: utcTimeMillis must be non-negative",
                        "invalid_timestamp",
                    )
                if previous_timestamp is not None and timestamp < previous_timestamp:
                    raise WlsPositionError(
                        f"row {row_number}: utcTimeMillis moved backwards",
                        "timestamp_moved_backwards",
                    )
                if current_timestamp is None:
                    current_timestamp = timestamp
                    current_first_row = row_number
                    epoch_timestamps.append(timestamp)
                elif timestamp != current_timestamp:
                    finish_current(row_number - 1)
                    current_timestamp = timestamp
                    current_first_row = row_number
                    epoch_timestamps.append(timestamp)
                clock_count = _parse_integer(
                    raw_row.get("HardwareClockDiscontinuityCount", ""),
                    "HardwareClockDiscontinuityCount",
                    row_number,
                )
                if clock_count < 0:
                    raise WlsPositionError(
                        f"row {row_number}: hardware clock count must be non-negative",
                        "invalid_clock_discontinuity",
                    )
                if current_clock is None:
                    current_clock = clock_count
                elif current_clock != clock_count:
                    raise WlsPositionError(
                        f"epoch {timestamp}: inconsistent hardware clock count",
                        "inconsistent_clock_discontinuity",
                    )
                svid = _parse_integer(raw_row.get("Svid", ""), "Svid", row_number)
                if svid <= 0:
                    raise WlsPositionError(
                        f"row {row_number}: Svid must be positive", "invalid_svid"
                    )
                tokens = [
                    (raw_row.get(field) or "").strip()
                    for field in (
                        "WlsPositionXEcefMeters",
                        "WlsPositionYEcefMeters",
                        "WlsPositionZEcefMeters",
                    )
                ]
                present = [bool(token) for token in tokens]
                if not all(present):
                    if any(present):
                        classification = "partial_wls_epoch"
                        classification_counts["partial_wls_epoch"] += 1
                        raise WlsPositionError(
                            f"row {row_number}: WLS ECEF is {classification}",
                            classification,
                        )
                    else:
                        if not allow_missing_wls_epochs:
                            classification_counts["missing_wls_epoch"] += 1
                            raise WlsPositionError(
                                f"row {row_number}: WLS ECEF is missing_wls_epoch",
                                "missing_wls_epoch",
                            )
                        classification_counts["blank_wls_row"] += 1
                        blank_wls_row_count += 1
                        current_blank_rows += 1
                        continue
                ecef = tuple(
                    _parse_float(token, field, row_number)
                    for token, field in zip(
                        tokens,
                        (
                            "WlsPositionXEcefMeters",
                            "WlsPositionYEcefMeters",
                            "WlsPositionZEcefMeters",
                        ),
                    )
                )
                try:
                    _validate_ecef(ecef, row_number)
                except WlsPositionError as exc:
                    classification_counts[exc.classification] = (
                        classification_counts.get(exc.classification, 0) + 1
                    )
                    if (
                        allow_invalid_wls_epochs
                        and exc.classification == "ecef_out_of_range"
                    ):
                        current_invalid_rows.append(row_number)
                        continue
                    raise
                current_values.append(ecef)
                current_svids.add(svid)
            finish_current(input_rows + 1)
    except OSError as exc:
        raise WlsPositionError(f"failed to read device GNSS CSV: {device_gnss}") from exc
    if not epochs:
        raise WlsPositionError("device GNSS CSV contains no usable WLS epochs", "empty_input")
    return WlsExtraction(
        epochs=tuple(epochs),
        input_rows=input_rows,
        input_epochs=len(epoch_timestamps),
        timestamp_gap_count=timestamp_gap_count,
        max_timestamp_gap_s=max_timestamp_gap_s,
        clock_transition_count=len(clock_transitions),
        clock_transition_timestamps=tuple(clock_transitions),
        classification_counts=classification_counts,
        consistency_tolerance_m=consistency_tolerance_m,
        epoch_timestamps=tuple(epoch_timestamps),
        sparse_omission_records=tuple(sparse_omission_records),
        blank_wls_row_count=blank_wls_row_count,
        allow_missing_wls_epochs=allow_missing_wls_epochs,
        allow_invalid_wls_epochs=allow_invalid_wls_epochs,
    )


def select_epochs(
    extraction: WlsExtraction, skip_epochs: int, max_epochs: int = -1
) -> tuple[WlsEpoch, ...]:
    if skip_epochs < 0:
        raise WlsPositionError("skip_epochs must be non-negative")
    if max_epochs == 0 or max_epochs < -1:
        raise WlsPositionError("max_epochs must be -1 or positive")
    selected_timestamps = extraction.epoch_timestamps[skip_epochs:]
    if max_epochs > 0:
        selected_timestamps = selected_timestamps[:max_epochs]
    selected_timestamp_set = set(selected_timestamps)
    selected = tuple(
        epoch for epoch in extraction.epochs if epoch.timestamp_ms in selected_timestamp_set
    )
    if not selected:
        raise WlsPositionError("skip/max epochs remove every WLS epoch")
    return selected


def selected_timestamp_keys(
    extraction: WlsExtraction, skip_epochs: int, max_epochs: int = -1
) -> tuple[int, ...]:
    """Return the selected device epoch keys, including sparse omissions."""

    if skip_epochs < 0:
        raise WlsPositionError("skip_epochs must be non-negative")
    if max_epochs == 0 or max_epochs < -1:
        raise WlsPositionError("max_epochs must be -1 or positive")
    selected = extraction.epoch_timestamps[skip_epochs:]
    if max_epochs > 0:
        selected = selected[:max_epochs]
    if not selected:
        raise WlsPositionError("skip/max epochs remove every device GNSS epoch")
    return tuple(selected)


def epochs_to_positions(
    epochs: tuple[WlsEpoch, ...], leap_seconds: int = DEFAULT_LEAP_SECONDS
) -> list[smoother.PositionRow]:
    if not isinstance(leap_seconds, int) or leap_seconds < 0:
        raise WlsPositionError("leap_seconds must be a non-negative integer")
    positions: list[smoother.PositionRow] = []
    for epoch in epochs:
        ecef_array = np.array(epoch.ecef, dtype=float)
        latitude, longitude, height = smoother._wgs84_ecef_to_geodetic(ecef_array)
        week, tow = smoother._device_time_to_week_tow(epoch.timestamp_ms, leap_seconds)
        positions.append(
            smoother.PositionRow(
                week=week,
                tow=tow,
                timestamp_ms=epoch.timestamp_ms,
                ecef=ecef_array,
                latitude=math.degrees(latitude),
                longitude=math.degrees(longitude),
                height=height,
                status=1,
                satellites=epoch.svid_count,
                pdop=0.0,
                ratio=0.0,
                fixed_ambiguities=0,
                iterations=0,
                source_line=epoch.first_source_row,
            )
        )
    return positions


def _write_pos(path: Path, positions: list[smoother.PositionRow]) -> None:
    lines = [
        "% LibGNSS++ Android handset WLS ECEF position",
        "% Columns: GPS_Week GPS_TOW X(m) Y(m) Z(m) Lat(deg) Lon(deg) Height(m) Status Satellites PDOP Ratio FixedAmbiguities Iterations",
        "% Status 1 denotes a finite, epoch-consistent handset WLS coordinate; PDOP is unknown and set to 0.",
    ]
    for row in positions:
        lines.append(
            f"{row.week:d} {row.tow:.6f} "
            f"{row.ecef[0]:.6f} {row.ecef[1]:.6f} {row.ecef[2]:.6f} "
            f"{row.latitude:.9f} {row.longitude:.9f} {row.height:.6f} "
            f"{row.status:d} {row.satellites:d} {row.pdop:.6f} "
            f"{row.ratio:.6f} {row.fixed_ambiguities:d} {row.iterations:d}"
        )
    _atomic_write(path, ("\n".join(lines) + "\n").encode("ascii"))


def _write_geodetic(path: Path, positions: list[smoother.PositionRow]) -> None:
    output = [
        "UnixTimeMillis,GPSWeek,GPSTow,X_ECEF_m,Y_ECEF_m,Z_ECEF_m,LatitudeDegrees,LongitudeDegrees,AltitudeMeters,Status,Satellites,source"
    ]
    for row in positions:
        output.append(
            f"{row.timestamp_ms},{row.week},{row.tow:.6f},"
            f"{row.ecef[0]:.9f},{row.ecef[1]:.9f},{row.ecef[2]:.9f},"
            f"{row.latitude:.12f},{row.longitude:.12f},{row.height:.6f},"
            f"{row.status},{row.satellites},android_wls_ecef"
        )
    _atomic_write(path, ("\n".join(output) + "\n").encode("utf-8"))


def _summary(
    extraction: WlsExtraction,
    selected: tuple[WlsEpoch, ...],
    device_gnss: Path,
    skip_epochs: int,
    max_epochs: int,
    leap_seconds: int,
    timestamp_gap_diagnostic_authorized: bool = False,
) -> dict[str, Any]:
    selected_clock = [epoch.clock_discontinuity_count for epoch in selected]
    selected_keys = selected_timestamp_keys(extraction, skip_epochs, max_epochs)
    selected_key_set = set(selected_keys)
    selected_sparse_records = [
        record
        for record in extraction.sparse_omission_records
        if record["timestamp_ms"] in selected_key_set
    ]
    return {
        "schema_version": SCHEMA_VERSION,
        "decision": "truth-free-wls-position-lane",
        "truth_free": True,
        "inputs": {
            "device_gnss": {"path": str(device_gnss), "sha256": _sha256(device_gnss)},
            "ground_truth": None,
        },
        "selection": {
            "skip_epochs": skip_epochs,
            "max_epochs": max_epochs,
            "selected_device_epoch_timestamps_ms": list(selected_keys),
            "selected_epochs": len(selected),
            "first_timestamp_ms": selected[0].timestamp_ms,
            "last_timestamp_ms": selected[-1].timestamp_ms,
            "leap_seconds": leap_seconds,
        },
        "validation": {
            "ecef_consistency_tolerance_m": extraction.consistency_tolerance_m,
            "ecef_norm_range_m": [ECEF_NORM_MIN_M, ECEF_NORM_MAX_M],
            "ecef_component_abs_max_m": ECEF_COMPONENT_MAX_M,
            "all_epoch_rows_consistent": True,
            "all_selected_rows_finite": True,
            "classification_counts": dict(extraction.classification_counts),
            "allow_missing_wls_epochs": extraction.allow_missing_wls_epochs,
            "allow_invalid_wls_epochs": extraction.allow_invalid_wls_epochs,
            "timestamp_gap_allowed_as_diagnostic": timestamp_gap_diagnostic_authorized,
            "timestamp_gap_policy": (
                "diagnostic only under sealed v1.3 sparse multi-phone mode"
                if timestamp_gap_diagnostic_authorized
                else "fail-closed for gaps over 1500 ms"
            ),
            "missing_wls_policy": (
                "sparse mode: retain epochs with at least one finite complete WLS row; "
                "omit and record all-blank epochs"
                if extraction.allow_missing_wls_epochs
                else "fail-closed; no artifact is published"
            ),
            "blank_wls_row_policy": (
                "count blank rows and permit only in sealed multi-phone sparse mode"
                if extraction.allow_missing_wls_epochs
                else "fail-closed; blank WLS rows are not permitted"
            ),
            "sparse_omission_policy": (
                "omit all-blank or explicitly out-of-earth-range epochs and record timestamp/reason"
                if extraction.allow_missing_wls_epochs
                else "not permitted"
            ),
            "partial_wls_policy": "fail-closed; no artifact is published",
            "out_of_range_wls_policy": (
                "sealed test sparse mode only: omit the entire invalid epoch and record source rows"
                if extraction.allow_invalid_wls_epochs
                else "fail-closed; no artifact is published"
            ),
            "timestamp_gap_policy": "treat gaps up to and including 1500 ms as continuous; record gaps over 1500 ms as reset boundaries; reject non-increasing keys",
            "timestamp_gap_threshold_ms": TIMESTAMP_GAP_THRESHOLD_MS,
            "clock_discontinuity_policy": "require constant value within epoch and monotonic values across epochs",
        },
        "populations": {
            "input_rows": extraction.input_rows,
            "input_epochs": extraction.input_epochs,
            "selected_epochs": len(selected),
            "blank_wls_row_count": extraction.blank_wls_row_count,
            "sparse_omitted_epoch_count": len(extraction.sparse_omission_records),
            "sparse_omitted_timestamps_ms": [
                record["timestamp_ms"] for record in extraction.sparse_omission_records
            ],
            "sparse_omitted_epoch_records": list(extraction.sparse_omission_records),
            "selected_sparse_omitted_epoch_count": len(selected_sparse_records),
            "selected_sparse_omitted_epoch_records": selected_sparse_records,
            "timestamp_gap_count_gt_1500ms": extraction.timestamp_gap_count,
            "max_timestamp_gap_s": extraction.max_timestamp_gap_s,
            "clock_discontinuity_transition_count": extraction.clock_transition_count,
            "clock_discontinuity_transition_timestamps_ms": list(
                extraction.clock_transition_timestamps
            ),
            "selected_clock_counts": sorted(set(selected_clock)),
            "min_svid_count": min(epoch.svid_count for epoch in selected),
            "max_svid_count": max(epoch.svid_count for epoch in selected),
        },
        "conversion": {
            "ecef": "WGS84 geodetic conversion via shared numerically stable ECEF routine",
            "pos": "GPS week/TOW plus ECEF/geodetic columns consumed by native trajectory smoother",
            "submission": "truth-free phone,UnixTimeMillis,LatitudeDegrees,LongitudeDegrees when requested",
            "quality_fields": "status=1; Satellites=unique Svid count; PDOP/ratio/fix/iterations unknown and zero",
        },
        "artifacts": {},
    }


def _artifact(path: Path) -> dict[str, Any]:
    return {"path": str(path), "sha256": _sha256(path), "bytes": path.stat().st_size}


def _published_artifact(source: Path, target: Path) -> dict[str, Any]:
    artifact = _artifact(source)
    artifact["path"] = str(target)
    return artifact


def _verify_sealed_holdout_eval_freeze(
    path: Path,
    manifest_path: Path | None = None,
    *,
    dataset_id: str | None = None,
    truth_free: bool = False,
    allow_missing_wls_epochs: bool = False,
    allow_invalid_wls_epochs: bool = False,
) -> dict[str, Any]:
    """Authorize a holdout extraction only from an immutable freeze contract.

    The historical stability-selector freezes retain their source-only guard
    for compatibility.  The multi-phone holdout requires the stronger v1.1/v1.2/v1.3
    record+manifest, route/phone allowlist, algorithm hash, and explicit
    truth-free phase checks below.  The sparse v1.2 contract additionally
    authorizes blank-epoch omission; only v1.3 also authorizes timestamp gaps
    as diagnostics.
    """

    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise WlsPositionError(f"invalid sealed holdout freeze record: {exc}") from exc
    if not isinstance(payload, dict):
        raise WlsPositionError("sealed holdout freeze record schema is invalid")
    schema_version = payload.get("schema_version")
    multi_phone_schemas = {
        MULTI_PHONE_HOLDOUT_FREEZE_SCHEMA,
        MULTI_PHONE_HOLDOUT_FREEZE_SCHEMA_V1_2,
        MULTI_PHONE_HOLDOUT_FREEZE_SCHEMA_V1_3,
        MULTI_PHONE_HOLDOUT_FREEZE_SCHEMA_V1_4,
    }
    if schema_version in multi_phone_schemas:
        if allow_invalid_wls_epochs:
            raise WlsPositionError(
                "out-of-range WLS omission is restricted to the sealed test completeness contract"
            )
        if manifest_path is None:
            raise WlsPositionError(
                "multi-phone holdout requires the sealed freeze manifest"
            )
        if payload.get("status") != "frozen-before-holdout-payload-access":
            raise WlsPositionError("multi-phone holdout freeze is not pre-payload")
        contract = payload.get("holdout_execution_contract")
        if not isinstance(contract, dict) or contract.get("authorized") is not True:
            raise WlsPositionError("multi-phone freeze does not authorize this lane")
        if contract.get("truth_free_phase") is not True:
            raise WlsPositionError("multi-phone freeze is not truth-free")
        if contract.get("no_post_holdout_tuning") is not True:
            raise WlsPositionError("multi-phone freeze lacks no-post-tuning guard")
        if contract.get("route") != MULTI_PHONE_HOLDOUT_ROUTE:
            raise WlsPositionError("multi-phone freeze route is not authorized")
        allowlist = contract.get("phone_allowlist")
        if tuple(allowlist or ()) != MULTI_PHONE_HOLDOUT_PHONE_ALLOWLIST:
            raise WlsPositionError("multi-phone freeze phone allowlist is invalid")
        expected_algorithm_hash = MULTI_PHONE_HOLDOUT_ALGORITHM_PARAMETER_HASH
        if (
            payload.get("algorithm_parameter_hash") != expected_algorithm_hash
            or contract.get("algorithm_parameter_hash") != expected_algorithm_hash
        ):
            raise WlsPositionError("multi-phone algorithm parameter hash differs")
        if schema_version in {
            MULTI_PHONE_HOLDOUT_FREEZE_SCHEMA_V1_2,
            MULTI_PHONE_HOLDOUT_FREEZE_SCHEMA_V1_3,
            MULTI_PHONE_HOLDOUT_FREEZE_SCHEMA_V1_4,
        } and payload.get(
            "algorithm_core_hash"
        ) != MULTI_PHONE_HOLDOUT_ALGORITHM_CORE_HASH:
            raise WlsPositionError("multi-phone algorithm core hash differs")
        if not truth_free:
            raise WlsPositionError("multi-phone holdout WLS requires truth-free phase")
        if not isinstance(allow_missing_wls_epochs, bool):
            raise WlsPositionError("allow_missing_wls_epochs must be boolean")
        if allow_missing_wls_epochs:
            if schema_version not in {
                MULTI_PHONE_HOLDOUT_FREEZE_SCHEMA_V1_2,
                MULTI_PHONE_HOLDOUT_FREEZE_SCHEMA_V1_3,
                MULTI_PHONE_HOLDOUT_FREEZE_SCHEMA_V1_4,
            }:
                raise WlsPositionError(
                    "sparse WLS requires the v1.2/v1.3/v1.4 multi-phone holdout freeze"
                )
            sparse_contract = payload.get("sparse_wls_contract")
            if not isinstance(sparse_contract, dict):
                raise WlsPositionError("multi-phone freeze lacks sparse WLS contract")
            if sparse_contract.get("allow_missing_wls_epochs") is not True:
                raise WlsPositionError("sparse WLS is not authorized by the freeze")
            if sparse_contract.get("single_phone_default_fail_closed") is not True:
                raise WlsPositionError("sparse WLS freeze weakens single-phone default")
            for key in (
                "partial_coordinate_triplet_policy",
                "nonfinite_coordinate_policy",
                "inconsistent_coordinate_policy",
            ):
                if sparse_contract.get(key) != "fail-closed":
                    raise WlsPositionError(
                        f"sparse WLS freeze does not fail closed for {key}"
                    )
            if schema_version in {
                MULTI_PHONE_HOLDOUT_FREEZE_SCHEMA_V1_3,
                MULTI_PHONE_HOLDOUT_FREEZE_SCHEMA_V1_4,
            }:
                if sparse_contract.get("allow_timestamp_gaps_as_diagnostic") is not True:
                    raise WlsPositionError(
                        "v1.3 sparse WLS lacks timestamp-gap diagnostic authorization"
                    )
                if sparse_contract.get("extrapolation_policy") != "forbidden":
                    raise WlsPositionError(
                        "v1.3 sparse WLS does not forbid extrapolation"
                    )
        elif schema_version in {
            MULTI_PHONE_HOLDOUT_FREEZE_SCHEMA_V1_2,
            MULTI_PHONE_HOLDOUT_FREEZE_SCHEMA_V1_3,
            MULTI_PHONE_HOLDOUT_FREEZE_SCHEMA_V1_4,
        }:
            sparse_contract = payload.get("sparse_wls_contract")
            if not isinstance(sparse_contract, dict) or sparse_contract.get(
                "single_phone_default_fail_closed"
            ) is not True:
                raise WlsPositionError("v1.2 freeze weakens single-phone default")
        if not isinstance(dataset_id, str) or dataset_id.count("/") != 1:
            raise WlsPositionError("multi-phone holdout requires a route/phone dataset ID")
        route, phone = dataset_id.split("/", 1)
        if route != MULTI_PHONE_HOLDOUT_ROUTE or phone not in allowlist:
            raise WlsPositionError("multi-phone holdout dataset is outside the allowlist")
        if payload.get("holdout_route") != route:
            raise WlsPositionError("multi-phone freeze holdout route mismatch")
        try:
            manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as exc:
            raise WlsPositionError(f"invalid multi-phone freeze manifest: {exc}") from exc
        expected_manifest_schema = {
            MULTI_PHONE_HOLDOUT_FREEZE_SCHEMA_V1_2:
                MULTI_PHONE_HOLDOUT_FREEZE_MANIFEST_SCHEMA_V1_2,
            MULTI_PHONE_HOLDOUT_FREEZE_SCHEMA_V1_3:
                MULTI_PHONE_HOLDOUT_FREEZE_MANIFEST_SCHEMA_V1_3,
            MULTI_PHONE_HOLDOUT_FREEZE_SCHEMA_V1_4:
                MULTI_PHONE_HOLDOUT_FREEZE_MANIFEST_SCHEMA_V1_4,
        }.get(schema_version, MULTI_PHONE_HOLDOUT_FREEZE_MANIFEST_SCHEMA)
        if not isinstance(manifest, dict) or manifest.get("schema_version") != expected_manifest_schema:
            raise WlsPositionError("multi-phone holdout freeze manifest schema is invalid")
        freeze_record = manifest.get("freeze_record")
        if not isinstance(freeze_record, dict):
            raise WlsPositionError("multi-phone freeze manifest lacks record hash")
        if freeze_record.get("sha256") != _sha256(path):
            raise WlsPositionError("multi-phone freeze record hash differs")
        if manifest.get("algorithm_parameter_hash") != expected_algorithm_hash:
            raise WlsPositionError("multi-phone manifest algorithm parameter hash differs")
        if schema_version in {
            MULTI_PHONE_HOLDOUT_FREEZE_SCHEMA_V1_2,
            MULTI_PHONE_HOLDOUT_FREEZE_SCHEMA_V1_3,
            MULTI_PHONE_HOLDOUT_FREEZE_SCHEMA_V1_4,
        }:
            if manifest.get("algorithm_core_hash") != MULTI_PHONE_HOLDOUT_ALGORITHM_CORE_HASH:
                raise WlsPositionError("multi-phone manifest algorithm core hash differs")
            manifest_sparse = manifest.get("sparse_wls_contract")
            if not isinstance(manifest_sparse, dict) or manifest_sparse.get(
                "allow_missing_wls_epochs"
            ) is not True:
                raise WlsPositionError("multi-phone manifest sparse WLS authorization is absent")
            if schema_version in {
                MULTI_PHONE_HOLDOUT_FREEZE_SCHEMA_V1_3,
                MULTI_PHONE_HOLDOUT_FREEZE_SCHEMA_V1_4,
            }:
                if manifest_sparse.get("allow_timestamp_gaps_as_diagnostic") is not True:
                    raise WlsPositionError(
                        "multi-phone manifest timestamp-gap diagnostic authorization is absent"
                    )
                if manifest_sparse.get("extrapolation_policy") != "forbidden":
                    raise WlsPositionError(
                        "multi-phone manifest does not forbid extrapolation"
                    )
        source_files = payload.get("source_hashes")
        if not isinstance(source_files, dict):
            raise WlsPositionError("multi-phone freeze lacks source hashes")
        expected_source = source_files.get("apps/commands/benchmarks/gnss_smartphone_wls.py")
        if isinstance(expected_source, dict):
            expected_source = expected_source.get("sha256")
        if expected_source != _sha256(Path(__file__)):
            raise WlsPositionError("WLS source hash differs from multi-phone freeze")
        return payload
    if schema_version not in {
        "smartphone-r5-wls-stability-selector-holdout-freeze.v1",
        "smartphone-r5-wls-stability-selector-holdout-freeze.v2",
        "smartphone-r5-wls-stability-selector-holdout-freeze.v3",
        "smartphone-r5-wls-stability-selector-holdout-freeze.v4",
    }:
        raise WlsPositionError("sealed holdout freeze record schema is invalid")
    contract = payload.get("holdout_execution_contract")
    if not isinstance(contract, dict) or contract.get("authorized") is not True:
        raise WlsPositionError("sealed holdout freeze record does not authorize this lane")
    source_files = payload.get("source_files")
    # Match the repository-relative source key used by the freeze manifest;
    # accepting a bare basename would make the authorization ambiguous.
    wls_source = (
        source_files.get("apps/commands/benchmarks/gnss_smartphone_wls.py")
        if isinstance(source_files, dict)
        else None
    )
    expected = wls_source.get("sha256") if isinstance(wls_source, dict) else None
    if expected != _sha256(Path(__file__)):
        raise WlsPositionError("WLS source hash differs from the sealed holdout freeze")
    return payload


def _verify_sealed_test_authorization(
    path: Path,
    manifest_path: Path | None = None,
    *,
    dataset_id: str | None = None,
    truth_free: bool = False,
    allow_missing_wls_epochs: bool = False,
    allow_invalid_wls_epochs: bool = False,
) -> dict[str, Any]:
    """Authorize test WLS only from an explicit, truth-free test manifest.

    Test data is not a holdout and therefore cannot reuse the holdout route
    allowlist.  This guard is intentionally separate: a caller must present
    a content-hashed test authorization generated from the central-directory
    inventory, the v1.4 algorithm core, and the current WLS source.  In
    particular, merely passing ``role=test`` never enables sparse extraction.
    """

    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise WlsPositionError(f"invalid sealed test authorization: {exc}") from exc
    if not isinstance(payload, dict) or payload.get("schema_version") != TEST_WLS_AUTHORIZATION_SCHEMA:
        raise WlsPositionError("sealed test authorization schema is invalid")
    if payload.get("status") != "sealed-before-test-payload-access":
        raise WlsPositionError("sealed test authorization is not pre-payload")
    contract = payload.get("test_execution_contract")
    if not isinstance(contract, dict) or contract.get("authorized") is not True:
        raise WlsPositionError("sealed test authorization is not enabled")
    if contract.get("role") != "test":
        raise WlsPositionError("sealed test authorization role is invalid")
    if contract.get("truth_free_phase") is not True or contract.get(
        "truth_access_forbidden"
    ) is not True:
        raise WlsPositionError("sealed test authorization is not truth-free")
    if contract.get("no_post_test_tuning") is not True:
        raise WlsPositionError("sealed test authorization lacks no-tuning guard")
    if contract.get("alignment_tolerance_ms") != TEST_WLS_ALIGNMENT_TOLERANCE_MS:
        raise WlsPositionError("sealed test authorization alignment tolerance differs")
    if contract.get("skip_epochs") != 1:
        raise WlsPositionError("sealed test authorization skip_epochs differs")
    if payload.get("algorithm_core_hash") != MULTI_PHONE_HOLDOUT_ALGORITHM_CORE_HASH:
        raise WlsPositionError("sealed test authorization algorithm core differs")
    if payload.get("algorithm_parameter_hash") != MULTI_PHONE_HOLDOUT_ALGORITHM_PARAMETER_HASH:
        raise WlsPositionError("sealed test authorization algorithm parameters differ")
    if not isinstance(dataset_id, str) or dataset_id.count("/") != 1:
        raise WlsPositionError("test WLS requires a route/phone dataset ID")
    allowlist = contract.get("dataset_allowlist")
    if not isinstance(allowlist, list) or any(
        not isinstance(value, str) or value.count("/") != 1 for value in allowlist
    ):
        raise WlsPositionError("sealed test authorization dataset allowlist is invalid")
    if len(allowlist) != len(set(allowlist)) or dataset_id not in allowlist:
        raise WlsPositionError("test dataset is outside the sealed allowlist")
    if not truth_free:
        raise WlsPositionError("test WLS is authorized only during truth-free phase")
    if not isinstance(allow_missing_wls_epochs, bool):
        raise WlsPositionError("allow_missing_wls_epochs must be boolean")
    if not isinstance(allow_invalid_wls_epochs, bool):
        raise WlsPositionError("allow_invalid_wls_epochs must be boolean")
    if allow_missing_wls_epochs:
        sparse = payload.get("sparse_wls_contract")
        if not isinstance(sparse, dict):
            raise WlsPositionError("sealed test authorization lacks sparse WLS contract")
        if sparse.get("allow_missing_wls_epochs") is not True:
            raise WlsPositionError("sparse WLS is not authorized for test")
        if sparse.get("single_phone_default_fail_closed") is not True:
            raise WlsPositionError("test sparse contract weakens single-phone default")
        if sparse.get("allow_timestamp_gaps_as_diagnostic") is not True:
            raise WlsPositionError("test sparse contract lacks timestamp-gap authorization")
        if sparse.get("extrapolation_policy") != "forbidden":
            raise WlsPositionError("test sparse contract permits extrapolation")
        for key in (
            "partial_coordinate_triplet_policy",
            "nonfinite_coordinate_policy",
            "inconsistent_coordinate_policy",
        ):
            if sparse.get(key) != "fail-closed":
                raise WlsPositionError(f"test sparse contract does not fail closed for {key}")
        if allow_invalid_wls_epochs and sparse.get(
            "allow_out_of_range_wls_epochs_as_omission"
        ) is not True:
            raise WlsPositionError(
                "out-of-range WLS omission is not authorized by the test freeze"
            )
    elif allow_invalid_wls_epochs:
        raise WlsPositionError(
            "out-of-range WLS omission requires the sealed sparse authorization"
        )
    if manifest_path is None:
        raise WlsPositionError("test WLS requires the sealed authorization manifest")
    try:
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise WlsPositionError(f"invalid sealed test authorization manifest: {exc}") from exc
    if not isinstance(manifest, dict) or manifest.get(
        "schema_version"
    ) != TEST_WLS_AUTHORIZATION_MANIFEST_SCHEMA:
        raise WlsPositionError("sealed test authorization manifest schema is invalid")
    record_hash = manifest.get("authorization_record")
    if not isinstance(record_hash, dict) or record_hash.get("sha256") != _sha256(path):
        raise WlsPositionError("sealed test authorization record hash differs")
    if manifest.get("algorithm_core_hash") != MULTI_PHONE_HOLDOUT_ALGORITHM_CORE_HASH:
        raise WlsPositionError("sealed test authorization manifest algorithm core differs")
    if manifest.get("algorithm_parameter_hash") != MULTI_PHONE_HOLDOUT_ALGORITHM_PARAMETER_HASH:
        raise WlsPositionError("sealed test authorization manifest parameters differ")
    if manifest.get("dataset_allowlist") != allowlist:
        raise WlsPositionError("sealed test authorization allowlist differs")
    source_hashes = payload.get("source_hashes")
    if not isinstance(source_hashes, dict):
        raise WlsPositionError("sealed test authorization lacks source hashes")
    expected_source = source_hashes.get("apps/commands/benchmarks/gnss_smartphone_wls.py")
    if isinstance(expected_source, dict):
        expected_source = expected_source.get("sha256")
    if expected_source != _sha256(Path(__file__)):
        raise WlsPositionError("WLS source hash differs from sealed test authorization")
    manifest_wls = manifest.get("wls_source_sha256")
    if manifest_wls != _sha256(Path(__file__)):
        raise WlsPositionError("test authorization manifest WLS source hash differs")
    return payload


def extract_to_directory(
    device_gnss: Path,
    output_dir: Path,
    *,
    skip_epochs: int = 1,
    max_epochs: int = -1,
    consistency_tolerance_m: float = DEFAULT_CONSISTENCY_TOLERANCE_M,
    leap_seconds: int = DEFAULT_LEAP_SECONDS,
    submission_output: Path | None = None,
    phone: str | None = None,
    profile: Path | None = None,
    role: str = "development",
    dataset_id: str | None = None,
    sealed_holdout_eval_freeze: Path | None = None,
    sealed_holdout_eval_freeze_manifest: Path | None = None,
    sealed_test_authorization: Path | None = None,
    sealed_test_authorization_manifest: Path | None = None,
    truth_free: bool = True,
    allow_missing_wls_epochs: bool = False,
    allow_invalid_wls_epochs: bool = False,
) -> dict[str, Any]:
    """Validate, convert, and atomically publish the WLS lane."""

    if not isinstance(allow_missing_wls_epochs, bool):
        raise WlsPositionError("allow_missing_wls_epochs must be boolean")
    if not isinstance(allow_invalid_wls_epochs, bool):
        raise WlsPositionError("allow_invalid_wls_epochs must be boolean")
    if allow_invalid_wls_epochs and not allow_missing_wls_epochs:
        raise WlsPositionError(
            "allow_invalid_wls_epochs is restricted to authorized sparse mode"
        )
    if role not in ("development", "holdout", "test"):
        raise WlsPositionError("role must be development, holdout, or test")
    if allow_missing_wls_epochs and role not in ("holdout", "test"):
        raise WlsPositionError(
            "allow_missing_wls_epochs is restricted to an authorized sparse ensemble"
        )
    if role == "holdout" and sealed_holdout_eval_freeze is None:
        raise WlsPositionError("holdout WLS requires the sealed freeze record")
    if role == "test" and sealed_test_authorization is None:
        raise WlsPositionError("test WLS requires the sealed test authorization")
    if role == "development" and (
        sealed_holdout_eval_freeze is not None
        or sealed_holdout_eval_freeze_manifest is not None
        or sealed_test_authorization is not None
        or sealed_test_authorization_manifest is not None
    ):
        raise WlsPositionError("sealed authorization is invalid for role=development")
    if role == "holdout" and (
        sealed_test_authorization is not None or sealed_test_authorization_manifest is not None
    ):
        raise WlsPositionError("test authorization is invalid for role=holdout")
    if role == "test" and (
        sealed_holdout_eval_freeze is not None or sealed_holdout_eval_freeze_manifest is not None
    ):
        raise WlsPositionError("holdout freeze is invalid for role=test")
    freeze_payload: dict[str, Any] | None = None
    if role == "holdout":
        freeze_payload = _verify_sealed_holdout_eval_freeze(
            sealed_holdout_eval_freeze,
            sealed_holdout_eval_freeze_manifest,
            dataset_id=dataset_id,
            truth_free=truth_free,
            allow_missing_wls_epochs=allow_missing_wls_epochs,
            allow_invalid_wls_epochs=allow_invalid_wls_epochs,
        )
    elif role == "test":
        freeze_payload = _verify_sealed_test_authorization(
            sealed_test_authorization,
            sealed_test_authorization_manifest,
            dataset_id=dataset_id,
            truth_free=truth_free,
            allow_missing_wls_epochs=allow_missing_wls_epochs,
        )
    if role == "holdout" and not truth_free:
        raise WlsPositionError("holdout WLS is authorized only during truth-free phase")
    if role == "test" and not truth_free:
        raise WlsPositionError("test WLS is authorized only during truth-free phase")
    extraction = extract_epochs(
        device_gnss,
        consistency_tolerance_m=consistency_tolerance_m,
        allow_missing_wls_epochs=allow_missing_wls_epochs,
        allow_invalid_wls_epochs=allow_invalid_wls_epochs,
    )
    timestamp_gap_diagnostic_authorized = (
        allow_missing_wls_epochs
        and freeze_payload is not None
        and freeze_payload.get("schema_version") in {
            MULTI_PHONE_HOLDOUT_FREEZE_SCHEMA_V1_3,
            MULTI_PHONE_HOLDOUT_FREEZE_SCHEMA_V1_4,
        }
        and isinstance(freeze_payload.get("sparse_wls_contract"), dict)
        and freeze_payload["sparse_wls_contract"].get(
            "allow_timestamp_gaps_as_diagnostic"
        )
        is True
    )
    timestamp_gap_diagnostic_authorized = timestamp_gap_diagnostic_authorized or (
        role == "test"
        and freeze_payload is not None
        and isinstance(freeze_payload.get("sparse_wls_contract"), dict)
        and freeze_payload["sparse_wls_contract"].get(
            "allow_timestamp_gaps_as_diagnostic"
        )
        is True
    )
    # The default/single-phone contract remains fail-closed for reset-sized
    # gaps.  Only the sealed v1.3 sparse multi-phone lane may retain a gap as
    # a diagnostic for peer coverage; it never synthesizes a WLS row here.
    if extraction.timestamp_gap_count > 0 and not timestamp_gap_diagnostic_authorized:
        raise WlsPositionError(
            "WLS timestamp gap exceeds 1500 ms outside the sealed v1.3/v1.4 sparse lane",
            "timestamp_gap",
        )
    selected = select_epochs(extraction, skip_epochs, max_epochs)
    positions = epochs_to_positions(selected, leap_seconds)
    output_dir.mkdir(parents=True, exist_ok=True)
    started = time.perf_counter()
    final_paths = {
        "position": output_dir / "wls.pos",
        "geodetic": output_dir / "receiver_wls.csv",
        "summary": output_dir / "wls_summary.json",
        "manifest": output_dir / "wls_manifest.json",
    }
    with tempfile.TemporaryDirectory(prefix=".wls-publish-", dir=str(output_dir)) as temp_name:
        temporary_dir = Path(temp_name)
        temp_paths = {key: temporary_dir / path.name for key, path in final_paths.items()}
        _write_pos(temp_paths["position"], positions)
        _write_geodetic(temp_paths["geodetic"], positions)
        summary = _summary(
            extraction,
            selected,
            device_gnss,
            skip_epochs,
            max_epochs,
            leap_seconds,
            timestamp_gap_diagnostic_authorized,
        )
        summary["timing"] = {"extraction_and_conversion_wall_s": time.perf_counter() - started}
        submission_manifest_path: Path | None = None
        if submission_output is not None:
            if not phone:
                raise WlsPositionError("phone is required with submission_output")
            if profile is not None and not profile.is_file():
                raise WlsPositionError(f"missing profile for submission output: {profile}")
            temp_submission = temporary_dir / submission_output.name
            kaggle.generate_submission(
                temp_paths["position"],
                temp_submission,
                phone,
                device_gnss_path=device_gnss,
                profile_path=profile,
                role=role if profile is not None else None,
                dataset_id=dataset_id,
                skip_epochs=skip_epochs,
                gps_utc_leap_seconds=leap_seconds,
            )
            submission_manifest_path = temp_submission.with_name(
                temp_submission.name + ".manifest.json"
            )
        artifacts = {
            "position": _artifact(temp_paths["position"]),
            "geodetic": _artifact(temp_paths["geodetic"]),
        }
        if submission_manifest_path is not None and submission_output is not None:
            artifacts["submission"] = _published_artifact(
                temp_submission, submission_output
            )
            artifacts["submission_manifest"] = _published_artifact(
                submission_manifest_path,
                submission_output.with_name(submission_output.name + ".manifest.json"),
            )
        artifacts["position"]["path"] = str(final_paths["position"])
        artifacts["geodetic"]["path"] = str(final_paths["geodetic"])
        summary["artifacts"] = {
            name: {"path": str(final_paths[name])}
            for name in ("position", "geodetic")
        }
        if submission_output is not None:
            summary["artifacts"]["submission"] = {"path": str(submission_output)}
        _atomic_write(
            temp_paths["summary"],
            (json.dumps(summary, indent=2, sort_keys=True) + "\n").encode("utf-8"),
        )
        artifacts["summary"] = _published_artifact(
            temp_paths["summary"], final_paths["summary"]
        )
        manifest = {
            "schema_version": MANIFEST_SCHEMA_VERSION,
            "truth_used": False,
            "truth_free": True,
            "role": role,
            "inputs": {
                "device_gnss": {"path": str(device_gnss), "sha256": _sha256(device_gnss)},
                "ground_truth": None,
                "profile": (
                    {"path": str(profile), "sha256": _sha256(profile)}
                    if profile is not None
                    else None
                ),
            },
            "contract": {
                "dataset_id": dataset_id,
                "sealed_authorization": (
                    {
                        "record": str(sealed_test_authorization),
                        "manifest": str(sealed_test_authorization_manifest),
                        "record_sha256": _sha256(sealed_test_authorization),
                        "manifest_sha256": _sha256(sealed_test_authorization_manifest),
                    }
                    if role == "test"
                    and sealed_test_authorization is not None
                    and sealed_test_authorization_manifest is not None
                    else None
                ),
                "allow_missing_wls_epochs": extraction.allow_missing_wls_epochs,
                "allow_invalid_wls_epochs": extraction.allow_invalid_wls_epochs,
                "timestamp_gap_allowed_as_diagnostic": timestamp_gap_diagnostic_authorized,
                "timestamp_gap_policy": (
                    "diagnostic only under sealed v1.3 sparse multi-phone mode"
                    if timestamp_gap_diagnostic_authorized
                    else "fail-closed for gaps over 1500 ms"
                ),
                "wls_fields": [
                    "WlsPositionXEcefMeters",
                    "WlsPositionYEcefMeters",
                    "WlsPositionZEcefMeters",
                ],
                "epoch_key": "utcTimeMillis",
                "clock_state": "HardwareClockDiscontinuityCount",
                "consistency_tolerance_m": consistency_tolerance_m,
                "missing_and_partial_policy": "fail-closed",
                "ecef_norm_range_m": [ECEF_NORM_MIN_M, ECEF_NORM_MAX_M],
                "sparse_omission": {
                    "blank_wls_row_count": extraction.blank_wls_row_count,
                    "omitted_epoch_count": len(extraction.sparse_omission_records),
                    "omitted_epoch_records": list(extraction.sparse_omission_records),
                    "policy": (
                        "authorized test completeness-fallback mode only: omit all-blank or out-of-earth-range epochs and record timestamp/reason"
                        if extraction.allow_missing_wls_epochs
                        else "not permitted"
                    ),
                    "allow_out_of_range_wls_epochs_as_omission": extraction.allow_invalid_wls_epochs,
                },
                "source": "Android device_gnss.csv handset WLS ECEF",
            },
            "artifacts": artifacts,
            "summary": {"path": str(final_paths["summary"])},
        }
        _atomic_write(
            temp_paths["manifest"],
            (json.dumps(manifest, indent=2, sort_keys=True) + "\n").encode("utf-8"),
        )
        publish_pairs = [
            (temp_paths["position"], final_paths["position"]),
            (temp_paths["geodetic"], final_paths["geodetic"]),
            (temp_paths["summary"], final_paths["summary"]),
            (temp_paths["manifest"], final_paths["manifest"]),
        ]
        if submission_manifest_path is not None and submission_output is not None:
            publish_pairs.extend(
                (
                    (temp_submission, submission_output),
                    (
                        submission_manifest_path,
                        submission_output.with_name(submission_output.name + ".manifest.json"),
                    ),
                )
            )
        for source, target in publish_pairs:
            target.parent.mkdir(parents=True, exist_ok=True)
            os.replace(source, target)
    return manifest


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        prog=os.environ.get("GNSS_CLI_NAME", "gnss smartphone-wls-position")
    )
    parser.add_argument("--device-gnss", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--skip-epochs", type=int, default=1)
    parser.add_argument("--max-epochs", type=int, default=-1)
    parser.add_argument(
        "--consistency-tolerance-m",
        type=float,
        default=DEFAULT_CONSISTENCY_TOLERANCE_M,
    )
    parser.add_argument("--leap-seconds", type=int, default=DEFAULT_LEAP_SECONDS)
    parser.add_argument("--submission-output", type=Path)
    parser.add_argument("--phone")
    parser.add_argument("--profile", type=Path)
    parser.add_argument("--role", default="development")
    parser.add_argument("--dataset-id")
    parser.add_argument(
        "--sealed-holdout-freeze",
        "--sealed-holdout-eval-freeze",
        dest="sealed_holdout_eval_freeze",
        type=Path,
    )
    parser.add_argument("--sealed-holdout-freeze-manifest", type=Path)
    parser.add_argument("--sealed-test-authorization", type=Path)
    parser.add_argument("--sealed-test-authorization-manifest", type=Path)
    parser.add_argument(
        "--allow-missing-wls-epochs",
        action="store_true",
        help="permit sparse all-blank epochs only under an authorized sparse freeze",
    )
    parser.add_argument(
        "--allow-invalid-wls-epochs",
        action="store_true",
        help=(
            "permit finite out-of-earth-range WLS epochs to be omitted only under "
            "the sealed test completeness-fallback contract"
        ),
    )
    return parser.parse_args(argv)


def run(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    try:
        manifest = extract_to_directory(
            args.device_gnss,
            args.output_dir,
            skip_epochs=args.skip_epochs,
            max_epochs=args.max_epochs,
            consistency_tolerance_m=args.consistency_tolerance_m,
            leap_seconds=args.leap_seconds,
            submission_output=args.submission_output,
            phone=args.phone,
            profile=args.profile,
            role=args.role,
            dataset_id=args.dataset_id,
            sealed_holdout_eval_freeze=args.sealed_holdout_eval_freeze,
            sealed_holdout_eval_freeze_manifest=args.sealed_holdout_freeze_manifest,
            sealed_test_authorization=args.sealed_test_authorization,
            sealed_test_authorization_manifest=args.sealed_test_authorization_manifest,
            allow_missing_wls_epochs=args.allow_missing_wls_epochs,
            allow_invalid_wls_epochs=args.allow_invalid_wls_epochs,
        )
    except (WlsPositionError, ValueError) as exc:
        print(f"Smartphone WLS lane failed: {exc}", file=sys.stderr)
        return 2
    print(f"Smartphone WLS lane complete: {manifest['artifacts']['position']['path']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(run())
