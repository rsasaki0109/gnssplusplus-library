#!/usr/bin/env python3
"""Generate submissions and audit the public GSDC 2023 metric contract.

The official Kaggle page publishes the aggregation (per-phone P50/P95
horizontal-distance errors, then a mean over phones), but does not publish the
distance algorithm/earth model or percentile interpolation.  This module
therefore emits explicit local diagnostic variants for both a WGS84/Vincenty
distance and a spherical/Haversine distance, and never presents either one as
the official score.  It never uses ground truth while generating a submission.
"""

from __future__ import annotations

import argparse
import csv
from decimal import Decimal, InvalidOperation, ROUND_FLOOR
import hashlib
import io
import json
import math
import os
from pathlib import Path
import re
import tempfile
from collections import defaultdict
from dataclasses import dataclass
from typing import Any


SCHEMA_VERSION = "smartphone-kaggle-metric.v2"
MANIFEST_SCHEMA_VERSION = "smartphone-kaggle-submission-manifest.v2"
COMPETITION_SLUG = "smartphone-decimeter-2023"
COMPATIBILITY = "public-spec-compatible-distance-undetermined"
PRIMARY_SCORE_STATUS = "undetermined-from-public-primary-sources"
OFFICIAL_EVALUATION_URL = (
    "https://www.kaggle.com/competitions/smartphone-decimeter-2023"
)
SUBMISSION_FIELDS = (
    "phone",
    "UnixTimeMillis",
    "LatitudeDegrees",
    "LongitudeDegrees",
)
GROUND_TRUTH_FIELDS = ("UnixTimeMillis", "LatitudeDegrees", "LongitudeDegrees")
DEVICE_KEY_FIELDS = ("MessageType", "utcTimeMillis")
GPS_EPOCH_UNIX_SECONDS = Decimal("315964800")
SECONDS_PER_WEEK = Decimal("604800")
DEFAULT_GPS_UTC_LEAP_SECONDS = 18
WGS84_A = 6378137.0
WGS84_F = 1.0 / 298.257223563
WGS84_B = WGS84_A * (1.0 - WGS84_F)
# Explicitly local diagnostic choice.  The official page does not publish an
# earth radius, so this value must not be described as Kaggle's radius.
HAVERSINE_EARTH_RADIUS_M = 6371008.8
DISTANCE_VARIANT_IDS = ("wgs84_vincenty", "haversine_sphere")
PERCENTILE_VARIANT_IDS = ("linear_n_minus_1", "nearest_rank_ceiling")


@dataclass(frozen=True)
class CoordinateRow:
    phone: str
    timestamp: int
    latitude: float
    longitude: float
    source_line: int


@dataclass(frozen=True)
class PositionRow:
    timestamp: int
    latitude: float
    longitude: float
    source_line: int


def _error(message: str) -> ValueError:
    return ValueError(message)


def _require_file(path: Path, label: str) -> None:
    if not path.is_file():
        raise _error(f"missing {label}: {path}")


def _sha256(path: Path) -> str:
    _require_file(path, "input")
    digest = hashlib.sha256()
    try:
        with path.open("rb") as handle:
            for chunk in iter(lambda: handle.read(1024 * 1024), b""):
                digest.update(chunk)
    except OSError as exc:
        raise _error(f"failed to hash {path}: {exc}") from exc
    return digest.hexdigest()


def _load_json(path: Path, label: str) -> dict[str, Any]:
    _require_file(path, label)
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise _error(f"invalid {label} {path}: {exc}") from exc
    if not isinstance(payload, dict):
        raise _error(f"{label} must contain a JSON object")
    return payload


def _parse_phone(raw: str | None, field: str, row_number: int) -> str:
    if raw is None or not raw or raw != raw.strip() or any(
        character in raw for character in ("\r", "\n")
    ):
        raise _error(f"row {row_number}: {field} must be a non-empty exact key")
    return raw


_INTEGER_RE = re.compile(r"^[+-]?\d+$")
_FLOAT_RE = re.compile(
    r"^[+-]?(?:(?:\d+(?:\.\d*)?)|(?:\.\d+))(?:[eE][+-]?\d+)?$"
)


def _parse_integer(raw: str | None, field: str, row_number: int) -> int:
    token = "" if raw is None else raw.strip()
    if not token or not _INTEGER_RE.fullmatch(token):
        raise _error(f"row {row_number}: {field} must be an integer, got {raw!r}")
    try:
        value = int(token)
    except ValueError as exc:
        raise _error(f"row {row_number}: {field} is invalid") from exc
    return value


def _parse_float(raw: str | None, field: str, row_number: int) -> float:
    token = "" if raw is None else raw.strip()
    if not _FLOAT_RE.fullmatch(token):
        raise _error(f"row {row_number}: {field} must be finite, got {raw!r}")
    try:
        value = float(token)
    except (TypeError, ValueError) as exc:
        raise _error(f"row {row_number}: {field} must be finite, got {raw!r}") from exc
    if not math.isfinite(value):
        raise _error(f"row {row_number}: {field} must be finite")
    return value


def _coordinate(raw: str | None, field: str, row_number: int, lower: float, upper: float) -> float:
    value = _parse_float(raw, field, row_number)
    if value < lower or value > upper:
        raise _error(f"row {row_number}: {field} must be in [{lower}, {upper}]")
    return value


def _csv_fields(reader: csv.DictReader, path: Path) -> list[str]:
    fields = list(reader.fieldnames or ())
    if not fields:
        raise _error(f"CSV has no header: {path}")
    if any(field is None or field == "" for field in fields):
        raise _error(f"CSV has an empty header field: {path}")
    if len(fields) != len(set(fields)):
        raise _error(f"CSV has duplicate header fields: {path}")
    return fields


def _read_submission(path: Path) -> list[CoordinateRow]:
    _require_file(path, "submission CSV")
    rows: list[CoordinateRow] = []
    seen: set[tuple[str, int]] = set()
    try:
        with path.open(encoding="utf-8-sig", newline="") as handle:
            reader = csv.DictReader(handle)
            fields = _csv_fields(reader, path)
            if tuple(fields) != SUBMISSION_FIELDS:
                raise _error(
                    "submission header must be exactly "
                    f"{','.join(SUBMISSION_FIELDS)}, got {','.join(fields)}"
                )
            for row_number, raw_row in enumerate(reader, start=2):
                if None in raw_row:
                    raise _error(
                        f"row {row_number}: submission has more columns than its header"
                    )
                row = {key: (value or "") for key, value in raw_row.items()}
                phone = _parse_phone(row.get("phone"), "phone", row_number)
                timestamp = _parse_integer(
                    row.get("UnixTimeMillis"), "UnixTimeMillis", row_number
                )
                if timestamp < 0:
                    raise _error(f"row {row_number}: UnixTimeMillis must be non-negative")
                latitude = _coordinate(
                    row.get("LatitudeDegrees"), "LatitudeDegrees", row_number, -90.0, 90.0
                )
                longitude = _coordinate(
                    row.get("LongitudeDegrees"),
                    "LongitudeDegrees",
                    row_number,
                    -180.0,
                    180.0,
                )
                key = (phone, timestamp)
                if key in seen:
                    raise _error(f"row {row_number}: duplicate submission key {key!r}")
                seen.add(key)
                rows.append(CoordinateRow(phone, timestamp, latitude, longitude, row_number))
    except OSError as exc:
        raise _error(f"failed to read submission CSV {path}: {exc}") from exc
    if not rows:
        raise _error("submission CSV contains no rows")
    return rows


def _read_ground_truth(path: Path, default_phone: str | None) -> list[CoordinateRow]:
    _require_file(path, "ground-truth CSV")
    rows: list[CoordinateRow] = []
    seen: set[tuple[str, int]] = set()
    try:
        with path.open(encoding="utf-8-sig", newline="") as handle:
            reader = csv.DictReader(handle)
            fields = _csv_fields(reader, path)
            missing = [field for field in GROUND_TRUTH_FIELDS if field not in fields]
            if missing:
                raise _error(
                    f"ground-truth CSV missing fields: {', '.join(missing)}"
                )
            has_phone = "phone" in fields
            if not has_phone and default_phone is None:
                raise _error(
                    "ground-truth CSV has no phone field; --phone is required"
                )
            if not has_phone:
                default_phone = _parse_phone(default_phone, "--phone", 0)
            for row_number, raw_row in enumerate(reader, start=2):
                if None in raw_row:
                    raise _error(
                        f"row {row_number}: ground-truth has more columns than its header"
                    )
                row = {key: (value or "") for key, value in raw_row.items()}
                phone = (
                    _parse_phone(row.get("phone"), "phone", row_number)
                    if has_phone
                    else default_phone
                )
                assert phone is not None
                timestamp = _parse_integer(
                    row.get("UnixTimeMillis"), "UnixTimeMillis", row_number
                )
                if timestamp < 0:
                    raise _error(f"row {row_number}: UnixTimeMillis must be non-negative")
                latitude = _coordinate(
                    row.get("LatitudeDegrees"), "LatitudeDegrees", row_number, -90.0, 90.0
                )
                longitude = _coordinate(
                    row.get("LongitudeDegrees"),
                    "LongitudeDegrees",
                    row_number,
                    -180.0,
                    180.0,
                )
                key = (phone, timestamp)
                if key in seen:
                    raise _error(f"row {row_number}: duplicate ground-truth key {key!r}")
                seen.add(key)
                rows.append(CoordinateRow(phone, timestamp, latitude, longitude, row_number))
    except OSError as exc:
        raise _error(f"failed to read ground-truth CSV {path}: {exc}") from exc
    if not rows:
        raise _error("ground-truth CSV contains no rows")
    return rows


def _load_profile(path: Path | None, role: str | None) -> dict[str, Any] | None:
    if path is None:
        if role is not None:
            raise _error("--role requires --profile")
        return None
    if role not in ("development", "holdout"):
        raise _error("--profile requires --role development or holdout")
    payload = _load_json(path, "profile")
    if payload.get("schema_version") != "smartphone-r5-profile.v1":
        raise _error("profile schema is not smartphone-r5-profile.v1")
    datasets = payload.get("datasets")
    if not isinstance(datasets, dict) or not isinstance(datasets.get(role), dict):
        raise _error(f"profile has no {role} dataset")
    dataset = dict(datasets[role])
    dataset_id = dataset.get("id")
    if not isinstance(dataset_id, str) or not dataset_id:
        raise _error(f"profile {role} dataset has no id")
    skip_epochs = dataset.get("skip_epochs", 0)
    if not isinstance(skip_epochs, int) or skip_epochs < 0:
        raise _error(f"profile {role} skip_epochs is invalid")
    return {
        "path": path,
        "sha256": _sha256(path),
        "profile_id": payload.get("profile_id"),
        "role": role,
        "dataset": dataset,
        "dataset_id": dataset_id,
        "skip_epochs": skip_epochs,
    }


def _check_profile_input(
    profile: dict[str, Any] | None,
    dataset_id: str | None,
    path: Path,
    expected_hash_field: str,
) -> None:
    if profile is None:
        return
    if dataset_id is not None and dataset_id != profile["dataset_id"]:
        raise _error(
            f"dataset id {dataset_id!r} does not match profile {profile['dataset_id']!r}"
        )
    expected_hash = profile["dataset"].get(expected_hash_field)
    if not isinstance(expected_hash, str) or not expected_hash:
        raise _error(f"profile has no {expected_hash_field}")
    actual_hash = _sha256(path)
    if actual_hash != expected_hash:
        raise _error(
            f"{expected_hash_field} hash mismatch: {actual_hash} != {expected_hash}"
        )


def _position_timestamp(week_token: str, tow_token: str, leap_seconds: int, line: int) -> int:
    if not _INTEGER_RE.fullmatch(week_token.strip()):
        raise _error(f"position line {line}: GPS week must be an integer")
    week = int(week_token.strip())
    if week < 0:
        raise _error(f"position line {line}: GPS week must be non-negative")
    if not _FLOAT_RE.fullmatch(tow_token.strip()):
        raise _error(f"position line {line}: GPS TOW must be finite")
    try:
        tow = Decimal(tow_token.strip())
    except (InvalidOperation, ValueError) as exc:
        raise _error(f"position line {line}: GPS TOW must be finite") from exc
    if not tow.is_finite() or tow < 0 or tow >= SECONDS_PER_WEEK:
        raise _error(f"position line {line}: GPS TOW must be in [0, 604800)")
    if not isinstance(leap_seconds, int) or leap_seconds < 0:
        raise _error("gps UTC leap seconds must be a non-negative integer")
    unix_ms = (
        GPS_EPOCH_UNIX_SECONDS * 1000
        + Decimal(week) * SECONDS_PER_WEEK * 1000
        + tow * 1000
        - Decimal(leap_seconds) * 1000
    ).to_integral_value(rounding=ROUND_FLOOR)
    return int(unix_ms)


def _read_positions(path: Path, leap_seconds: int) -> list[PositionRow]:
    _require_file(path, "position file")
    rows: list[PositionRow] = []
    previous_timestamp: int | None = None
    try:
        with path.open(encoding="ascii") as handle:
            for line_number, raw_line in enumerate(handle, start=1):
                line = raw_line.strip()
                if not line or line.startswith("%"):
                    continue
                fields = line.split()
                if len(fields) < 7:
                    raise _error(
                        f"position line {line_number}: expected at least 7 columns"
                    )
                timestamp = _position_timestamp(fields[0], fields[1], leap_seconds, line_number)
                latitude = _coordinate(fields[5], "LatitudeDegrees", line_number, -90.0, 90.0)
                longitude = _coordinate(
                    fields[6], "LongitudeDegrees", line_number, -180.0, 180.0
                )
                if previous_timestamp is not None and timestamp <= previous_timestamp:
                    raise _error(
                        f"position line {line_number}: timestamps must increase after floor conversion"
                    )
                previous_timestamp = timestamp
                rows.append(PositionRow(timestamp, latitude, longitude, line_number))
    except OSError as exc:
        raise _error(f"failed to read position file {path}: {exc}") from exc
    if not rows:
        raise _error("position file contains no solution rows")
    return rows


def _read_device_epochs(path: Path, skip_epochs: int) -> list[int]:
    _require_file(path, "device GNSS CSV")
    if skip_epochs < 0:
        raise _error("skip_epochs must be non-negative")
    timestamps: list[int] = []
    current: int | None = None
    previous: int | None = None
    try:
        with path.open(encoding="utf-8-sig", newline="") as handle:
            reader = csv.DictReader(handle)
            fields = _csv_fields(reader, path)
            missing = [field for field in DEVICE_KEY_FIELDS if field not in fields]
            if missing:
                raise _error(f"device GNSS CSV missing fields: {', '.join(missing)}")
            for row_number, raw_row in enumerate(reader, start=2):
                if None in raw_row:
                    raise _error(
                        f"row {row_number}: device GNSS CSV has more columns than its header"
                    )
                row = {key: (value or "") for key, value in raw_row.items()}
                if row.get("MessageType") != "Raw":
                    raise _error(f"device row {row_number}: MessageType must be Raw")
                timestamp = _parse_integer(
                    row.get("utcTimeMillis"), "utcTimeMillis", row_number
                )
                if timestamp < 0:
                    raise _error(f"device row {row_number}: utcTimeMillis must be non-negative")
                if previous is not None and timestamp < previous:
                    raise _error(f"device row {row_number}: utcTimeMillis moved backwards")
                previous = timestamp
                if current is None or timestamp != current:
                    timestamps.append(timestamp)
                    current = timestamp
    except OSError as exc:
        raise _error(f"failed to read device GNSS CSV {path}: {exc}") from exc
    if not timestamps:
        raise _error("device GNSS CSV contains no epochs")
    selected = timestamps[skip_epochs:]
    if not selected:
        raise _error("skip_epochs removes all device GNSS epochs")
    return selected


def _wgs84_horizontal_distance_m(
    latitude_a: float,
    longitude_a: float,
    latitude_b: float,
    longitude_b: float,
) -> float:
    """Return the ellipsoidal WGS84 inverse distance in metres.

    Vincenty's inverse iteration is sufficient for the short driving traces in
    this competition.  Non-convergence is an error instead of silently
    substituting a spherical approximation.
    """

    if latitude_a == latitude_b and longitude_a == longitude_b:
        return 0.0
    phi_a = math.radians(latitude_a)
    phi_b = math.radians(latitude_b)
    reduced_a = math.atan((1.0 - WGS84_F) * math.tan(phi_a))
    reduced_b = math.atan((1.0 - WGS84_F) * math.tan(phi_b))
    sin_a, cos_a = math.sin(reduced_a), math.cos(reduced_a)
    sin_b, cos_b = math.sin(reduced_b), math.cos(reduced_b)
    difference = math.radians(longitude_b - longitude_a)
    difference = (difference + math.pi) % (2.0 * math.pi) - math.pi
    lam = difference
    for _ in range(200):
        sin_lam, cos_lam = math.sin(lam), math.cos(lam)
        term_a = cos_b * sin_lam
        term_b = cos_a * sin_b - sin_a * cos_b * cos_lam
        sin_sigma = math.hypot(term_a, term_b)
        if sin_sigma == 0.0:
            return 0.0
        cos_sigma = sin_a * sin_b + cos_a * cos_b * cos_lam
        sigma = math.atan2(sin_sigma, cos_sigma)
        sin_alpha = cos_a * cos_b * sin_lam / sin_sigma
        cos2_alpha = 1.0 - sin_alpha * sin_alpha
        cos2_sigma_m = (
            0.0
            if cos2_alpha < 1e-18
            else cos_sigma - 2.0 * sin_a * sin_b / cos2_alpha
        )
        coefficient = WGS84_F / 16.0 * cos2_alpha * (
            4.0 + WGS84_F * (4.0 - 3.0 * cos2_alpha)
        )
        next_lam = difference + (1.0 - coefficient) * WGS84_F * sin_alpha * (
            sigma
            + coefficient
            * sin_sigma
            * (
                cos2_sigma_m
                + coefficient
                * cos_sigma
                * (-1.0 + 2.0 * cos2_sigma_m * cos2_sigma_m)
            )
        )
        if abs(next_lam - lam) <= 1e-12:
            lam = next_lam
            break
        lam = next_lam
    else:
        raise _error("WGS84 inverse distance did not converge")

    sin_lam, cos_lam = math.sin(lam), math.cos(lam)
    term_a = cos_b * sin_lam
    term_b = cos_a * sin_b - sin_a * cos_b * cos_lam
    sin_sigma = math.hypot(term_a, term_b)
    cos_sigma = sin_a * sin_b + cos_a * cos_b * cos_lam
    sigma = math.atan2(sin_sigma, cos_sigma)
    sin_alpha = cos_a * cos_b * sin_lam / sin_sigma
    cos2_alpha = 1.0 - sin_alpha * sin_alpha
    cos2_sigma_m = (
        0.0
        if cos2_alpha < 1e-18
        else cos_sigma - 2.0 * sin_a * sin_b / cos2_alpha
    )
    u2 = cos2_alpha * (WGS84_A * WGS84_A - WGS84_B * WGS84_B) / (WGS84_B * WGS84_B)
    coefficient_a = 1.0 + u2 / 16384.0 * (
        4096.0 + u2 * (-768.0 + u2 * (320.0 - 175.0 * u2))
    )
    coefficient_b = u2 / 1024.0 * (
        256.0 + u2 * (-128.0 + u2 * (74.0 - 47.0 * u2))
    )
    delta_sigma = coefficient_b * sin_sigma * (
        cos2_sigma_m
        + coefficient_b
        / 4.0
        * (
            cos_sigma * (-1.0 + 2.0 * cos2_sigma_m * cos2_sigma_m)
            - coefficient_b
            / 6.0
            * cos2_sigma_m
            * (-3.0 + 4.0 * sin_sigma * sin_sigma)
            * (-3.0 + 4.0 * cos2_sigma_m * cos2_sigma_m)
        )
    )
    distance = WGS84_B * coefficient_a * (sigma - delta_sigma)
    if not math.isfinite(distance) or distance < 0.0:
        raise _error("WGS84 inverse distance was not finite")
    return distance


def _haversine_horizontal_distance_m(
    latitude_a: float,
    longitude_a: float,
    latitude_b: float,
    longitude_b: float,
    *,
    radius_m: float = HAVERSINE_EARTH_RADIUS_M,
) -> float:
    """Return a spherical Haversine distance using an explicit local radius.

    ``radius_m`` is intentionally a keyword so every caller and report can
    make the earth-radius assumption visible.  It is not inferred from, or
    claimed to be, the private Kaggle evaluator configuration.
    """

    if not math.isfinite(radius_m) or radius_m <= 0.0:
        raise _error("Haversine earth radius must be finite and positive")
    latitude_a_rad = math.radians(latitude_a)
    latitude_b_rad = math.radians(latitude_b)
    delta_latitude = math.radians(latitude_b - latitude_a)
    delta_longitude = math.radians(longitude_b - longitude_a)
    haversine = (
        math.sin(delta_latitude / 2.0) ** 2
        + math.cos(latitude_a_rad)
        * math.cos(latitude_b_rad)
        * math.sin(delta_longitude / 2.0) ** 2
    )
    # Floating-point roundoff can put a mathematically valid value a few ulps
    # outside [0, 1], which would otherwise make asin fail closed on a valid
    # antipodal pair.
    haversine = min(1.0, max(0.0, haversine))
    distance = 2.0 * radius_m * math.asin(math.sqrt(haversine))
    if not math.isfinite(distance) or distance < 0.0:
        raise _error("Haversine distance was not finite")
    return distance


def _percentile_linear_n_minus_1(values: list[float], fraction: float) -> float:
    if not values:
        raise _error("cannot calculate a percentile of an empty distribution")
    if not math.isfinite(fraction) or not 0.0 <= fraction <= 1.0:
        raise _error("percentile fraction must be finite and in [0, 1]")
    ordered = sorted(values)
    rank = (len(ordered) - 1) * fraction
    lower = math.floor(rank)
    upper = math.ceil(rank)
    if lower == upper:
        return ordered[lower]
    weight = rank - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def _percentile_nearest_rank_ceiling(values: list[float], fraction: float) -> float:
    """Return the explicitly declared nearest-rank percentile.

    This is a sensitivity variant only.  The public Kaggle page describes the
    percentile concept but does not specify a quantile interpolation rule.
    ``ceil(n*q)`` is therefore not treated as the official implementation.
    """

    if not values:
        raise _error("cannot calculate a percentile of an empty distribution")
    if not math.isfinite(fraction) or not 0.0 <= fraction <= 1.0:
        raise _error("percentile fraction must be finite and in [0, 1]")
    ordered = sorted(values)
    rank = max(1, math.ceil(len(ordered) * fraction))
    return ordered[rank - 1]


# Keep the short helper used by earlier local callers as the documented local
# linear variant while making its method explicit in the new report schema.
def _percentile(values: list[float], fraction: float) -> float:
    return _percentile_linear_n_minus_1(values, fraction)


def _score_variant_id(distance_variant: str, percentile_variant: str) -> str:
    return f"{distance_variant}__{percentile_variant}"


def _distance_variant_metadata() -> dict[str, dict[str, Any]]:
    return {
        "wgs84_vincenty": {
            "algorithm": "Vincenty inverse iteration",
            "earth_model": "WGS84 ellipsoid",
            "earth_radius_m": None,
            "formula_status": "local-diagnostic-not-confirmed-by-public-source",
        },
        "haversine_sphere": {
            "algorithm": "Haversine great-circle distance",
            "earth_model": "sphere",
            "earth_radius_m": HAVERSINE_EARTH_RADIUS_M,
            "formula_status": "local-diagnostic-not-confirmed-by-public-source",
        },
    }


def _percentile_variant_metadata() -> dict[str, dict[str, Any]]:
    return {
        "linear_n_minus_1": {
            "method": "linear interpolation at rank (n - 1) * q",
            "implementation": "_percentile_linear_n_minus_1",
            "status": "local-diagnostic-not-confirmed-by-public-source",
        },
        "nearest_rank_ceiling": {
            "method": "nearest rank with rank=max(1, ceil(n * q))",
            "implementation": "_percentile_nearest_rank_ceiling",
            "status": "local-diagnostic-not-confirmed-by-public-source",
        },
    }


def _atomic_write(path: Path, content: bytes) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary: str | None = None
    try:
        descriptor, temporary = tempfile.mkstemp(
            prefix=f".{path.name}.", suffix=".tmp", dir=str(path.parent)
        )
        with os.fdopen(descriptor, "wb") as handle:
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
        raise _error(f"atomic publish failed for {path}: {exc}") from exc
    finally:
        if temporary is not None:
            try:
                os.unlink(temporary)
            except FileNotFoundError:
                pass


def _manifest_path(output: Path, explicit: Path | None) -> Path:
    path = explicit or output.with_name(output.name + ".manifest.json")
    if path.resolve() == output.resolve():
        raise _error("manifest path must differ from submission output path")
    return path


def _reject_output_collisions(
    output_path: Path,
    manifest_path: Path | None,
    inputs: list[tuple[str, Path]],
) -> None:
    output_resolved = output_path.resolve()
    manifest_resolved = manifest_path.resolve() if manifest_path is not None else None
    for label, input_path in inputs:
        input_resolved = input_path.resolve()
        if output_resolved == input_resolved:
            raise _error(f"output path would overwrite {label}: {input_path}")
        if manifest_resolved == input_resolved:
            raise _error(f"manifest path would overwrite {label}: {input_path}")


def _json_bytes(payload: dict[str, Any]) -> bytes:
    return (json.dumps(payload, indent=2, sort_keys=True) + "\n").encode("utf-8")


def _submission_csv(rows: list[CoordinateRow]) -> bytes:
    buffer = io.StringIO(newline="")
    writer = csv.writer(buffer, lineterminator="\n")
    writer.writerow(SUBMISSION_FIELDS)
    for row in rows:
        writer.writerow((row.phone, row.timestamp, row.latitude, row.longitude))
    return buffer.getvalue().encode("utf-8")


def generate_submission(
    position_path: Path,
    output_path: Path,
    phone: str,
    *,
    device_gnss_path: Path | None = None,
    profile_path: Path | None = None,
    role: str | None = None,
    dataset_id: str | None = None,
    skip_epochs: int | None = None,
    gps_utc_leap_seconds: int = DEFAULT_GPS_UTC_LEAP_SECONDS,
    manifest_path: Path | None = None,
) -> dict[str, Any]:
    phone = _parse_phone(phone, "phone", 0)
    profile = _load_profile(profile_path, role)
    if profile is not None:
        profile_skip = profile["skip_epochs"]
        if skip_epochs is not None and skip_epochs != profile_skip:
            raise _error("--skip-epochs cannot override the profile value")
        skip_epochs = profile_skip
    if skip_epochs is None:
        skip_epochs = 0
    if skip_epochs < 0:
        raise _error("--skip-epochs must be non-negative")
    positions = _read_positions(position_path, gps_utc_leap_seconds)
    device_keys: list[int] | None = None
    if device_gnss_path is not None:
        _check_profile_input(profile, dataset_id, device_gnss_path, "device_gnss_sha256")
        device_keys = _read_device_epochs(device_gnss_path, skip_epochs)
        device_key_set = set(device_keys)
        unknown = [row.timestamp for row in positions if row.timestamp not in device_key_set]
        if unknown:
            raise _error(
                "position timestamps are not exact selected device epochs; "
                f"first unknown key is {unknown[0]}"
            )
    elif profile is not None and dataset_id is not None and dataset_id != profile["dataset_id"]:
        raise _error(
            f"dataset id {dataset_id!r} does not match profile {profile['dataset_id']!r}"
        )

    manifest_output = _manifest_path(output_path, manifest_path)
    input_paths: list[tuple[str, Path]] = [("position file", position_path)]
    if device_gnss_path is not None:
        input_paths.append(("device GNSS CSV", device_gnss_path))
    if profile is not None:
        input_paths.append(("profile", profile["path"]))
    _reject_output_collisions(
        output_path,
        manifest_output,
        input_paths,
    )
    output_rows = [
        CoordinateRow(phone, row.timestamp, row.latitude, row.longitude, row.source_line)
        for row in positions
    ]
    output_bytes = _submission_csv(output_rows)
    _atomic_write(output_path, output_bytes)
    manifest: dict[str, Any] = {
        "schema_version": MANIFEST_SCHEMA_VERSION,
        "compatibility": COMPATIBILITY,
        "competition": {"slug": COMPETITION_SLUG, "year": 2023},
        "submission_columns": list(SUBMISSION_FIELDS),
        "phone": phone,
        "dataset_id": dataset_id or (profile["dataset_id"] if profile else None),
        "generator_contract": {
            "truth_used": False,
            "truth_input": None,
            "time_conversion": (
                "GPST GPS_Week/GPS_TOW to UnixTimeMillis using exact Decimal arithmetic, "
                "floor to milliseconds, and the declared GPS-UTC leap-second count"
            ),
            "gps_utc_leap_seconds": gps_utc_leap_seconds,
            "key_source": (
                "selected device_gnss.csv utcTimeMillis exact keys"
                if device_gnss_path is not None
                else "position GPST timestamps after exact floor conversion"
            ),
            "gap_policy": "emit only finite position epochs; never synthesize omitted keys",
        },
        "inputs": {
            "position": {"path": str(position_path), "sha256": _sha256(position_path)},
            "device_gnss": (
                {"path": str(device_gnss_path), "sha256": _sha256(device_gnss_path)}
                if device_gnss_path is not None
                else None
            ),
            "profile": (
                {
                    "path": str(profile["path"]),
                    "sha256": profile["sha256"],
                    "role": profile["role"],
                    "profile_id": profile["profile_id"],
                }
                if profile is not None
                else None
            ),
        },
        "selection": {
            "position_rows": len(output_rows),
            "device_epochs": len(device_keys) if device_keys is not None else None,
            "skip_epochs": skip_epochs,
        },
        "artifacts": {
            "submission": {
                "path": str(output_path),
                "sha256": _sha256(output_path),
                "rows": len(output_rows),
            },
            "manifest": {"path": str(manifest_output)},
        },
    }
    _atomic_write(manifest_output, _json_bytes(manifest))
    return manifest


def _manifest_for_submission(
    submission_path: Path, explicit_path: Path | None
) -> dict[str, Any] | None:
    path = explicit_path
    if path is None:
        candidate = submission_path.with_name(submission_path.name + ".manifest.json")
        path = candidate if candidate.is_file() else None
    if path is None:
        return None
    manifest = _load_json(path, "submission manifest")
    if manifest.get("schema_version") != MANIFEST_SCHEMA_VERSION:
        raise _error("submission manifest schema is invalid")
    if manifest.get("compatibility") != COMPATIBILITY:
        raise _error(
            "submission manifest uses an obsolete or unrecognised metric contract; "
            "regenerate it with the audited truth-free generator"
        )
    contract = manifest.get("generator_contract")
    if not isinstance(contract, dict) or contract.get("truth_used") is not False:
        raise _error("submission manifest does not prove truth-free generation")
    inputs = manifest.get("inputs")
    if not isinstance(inputs, dict) or "ground_truth" in inputs:
        raise _error("submission manifest contains a ground-truth input")
    artifacts = manifest.get("artifacts")
    submission_artifact = dict(artifacts or {}).get("submission")
    if not isinstance(submission_artifact, dict):
        raise _error("submission manifest has no submission artifact")
    expected_hash = submission_artifact.get("sha256")
    actual_hash = _sha256(submission_path)
    if expected_hash != actual_hash:
        raise _error("submission hash does not match its manifest")
    return {
        "path": str(path),
        "sha256": _sha256(path),
        "schema_version": manifest["schema_version"],
    }


def evaluate_submission(
    submission_path: Path,
    ground_truth_path: Path,
    output_json_path: Path,
    *,
    phone: str | None = None,
    profile_path: Path | None = None,
    role: str | None = None,
    dataset_id: str | None = None,
    allow_gaps: bool = False,
    submission_manifest_path: Path | None = None,
) -> dict[str, Any]:
    profile = _load_profile(profile_path, role)
    if profile is not None:
        _check_profile_input(profile, dataset_id, ground_truth_path, "ground_truth_sha256")
    submission_manifest = _manifest_for_submission(submission_path, submission_manifest_path)
    submission = _read_submission(submission_path)
    truth = _read_ground_truth(ground_truth_path, phone)
    manifest_input_path = submission_manifest_path
    if manifest_input_path is None:
        candidate_manifest = submission_path.with_name(submission_path.name + ".manifest.json")
        if candidate_manifest.is_file():
            manifest_input_path = candidate_manifest
    input_paths = [("submission CSV", submission_path), ("ground-truth CSV", ground_truth_path)]
    if manifest_input_path is not None:
        input_paths.append(("submission manifest", manifest_input_path))
    _reject_output_collisions(
        output_json_path,
        None,
        input_paths,
    )
    truth_by_key = {(row.phone, row.timestamp): row for row in truth}
    submission_by_key = {(row.phone, row.timestamp): row for row in submission}
    truth_keys = set(truth_by_key)
    submission_keys = set(submission_by_key)
    extra_keys = sorted(submission_keys - truth_keys)
    if extra_keys:
        raise _error(
            f"submission contains {len(extra_keys)} keys absent from ground truth; "
            f"first extra key is {extra_keys[0]!r}"
        )
    missing_keys = sorted(truth_keys - submission_keys)
    if missing_keys and not allow_gaps:
        raise _error(
            f"submission is missing {len(missing_keys)} ground-truth keys; "
            f"first missing key is {missing_keys[0]!r}; use --allow-gaps only for the "
            "publicly documented sparse-prediction policy"
        )

    distance_functions = {
        "wgs84_vincenty": _wgs84_horizontal_distance_m,
        "haversine_sphere": _haversine_horizontal_distance_m,
    }
    percentile_functions = {
        "linear_n_minus_1": _percentile_linear_n_minus_1,
        "nearest_rank_ceiling": _percentile_nearest_rank_ceiling,
    }
    distances_by_phone: dict[str, dict[str, list[float]]] = {
        distance_variant: defaultdict(list)
        for distance_variant in DISTANCE_VARIANT_IDS
    }
    for key, predicted in submission_by_key.items():
        reference = truth_by_key[key]
        for distance_variant, distance_function in distance_functions.items():
            distances_by_phone[distance_variant][predicted.phone].append(
                distance_function(
                    predicted.latitude,
                    predicted.longitude,
                    reference.latitude,
                    reference.longitude,
                )
            )
    if not any(distances_by_phone.values()):
        raise _error("no exact phone+UnixTimeMillis keys were scored")
    phone_metrics: dict[str, dict[str, Any]] = {}
    phone_names = sorted(distances_by_phone[DISTANCE_VARIANT_IDS[0]])
    score_values: dict[str, list[float]] = {
        _score_variant_id(distance_variant, percentile_variant): []
        for distance_variant in DISTANCE_VARIANT_IDS
        for percentile_variant in PERCENTILE_VARIANT_IDS
    }
    for current_phone in phone_names:
        score_variants: dict[str, dict[str, Any]] = {}
        for distance_variant in DISTANCE_VARIANT_IDS:
            distances = distances_by_phone[distance_variant][current_phone]
            for percentile_variant in PERCENTILE_VARIANT_IDS:
                percentile_function = percentile_functions[percentile_variant]
                p50 = percentile_function(distances, 0.50)
                p95 = percentile_function(distances, 0.95)
                score = (p50 + p95) / 2.0
                variant_id = _score_variant_id(distance_variant, percentile_variant)
                score_values[variant_id].append(score)
                score_variants[variant_id] = {
                    "distance_variant": distance_variant,
                    "percentile_variant": percentile_variant,
                    "p50_m": p50,
                    "p95_m": p95,
                    "phone_score_m": score,
                }
        phone_metrics[current_phone] = {
            "prediction_rows": len(
                distances_by_phone[DISTANCE_VARIANT_IDS[0]][current_phone]
            ),
            "score_variants": score_variants,
            "primary_score_m": None,
            "primary_score_status": PRIMARY_SCORE_STATUS,
        }
    distance_metadata = _distance_variant_metadata()
    percentile_metadata = _percentile_variant_metadata()
    missing_by_phone: dict[str, int] = defaultdict(int)
    for current_phone, _ in missing_keys:
        missing_by_phone[current_phone] += 1
    report: dict[str, Any] = {
        "schema_version": SCHEMA_VERSION,
        "compatibility": COMPATIBILITY,
        "competition": {"slug": COMPETITION_SLUG, "year": 2023},
        "metric_contract": {
            "official_public_source": {
                "url": OFFICIAL_EVALUATION_URL,
                "title": "Google Smartphone Decimeter Challenge 2023-2024",
                "sections": ["Evaluation", "Submission File"],
                "distance_definition": (
                    "horizontal distance in meters between predicted and ground-truth "
                    "latitude/longitude, once per second"
                ),
                "aggregation_definition": (
                    "per-phone P50 and P95 distance errors are averaged, then the "
                    "phone averages are averaged across phones"
                ),
                "distance_formula_published": False,
                "earth_model_published": False,
                "earth_radius_published": False,
                "percentile_interpolation_published": False,
                "unpublished_details": [
                    "distance formula and earth model",
                    "earth radius for a spherical distance, if used",
                    "P50/P95 interpolation or rank convention",
                ],
            },
            "formula": (
                "publicly documented structure only: mean over phones of "
                "((phone P50 horizontal distance + phone P95 horizontal distance) / 2)"
            ),
            "distance": {
                "status": "not-published-by-official-primary-source",
                "primary_variant": None,
                "variants": distance_metadata,
            },
            "percentile": {
                "status": "not-published-by-official-primary-source",
                "primary_variant": None,
                "variants": percentile_metadata,
            },
            "key_validation": (
                "every submission key must exist exactly once in truth; extra keys always fail; "
                "missing keys fail by default"
            ),
            "sparse_prediction_override": (
                "--allow-gaps records missing truth keys and scores submitted keys only, "
                "matching the public page's documented larger-gap allowance"
            ),
            "altitude": "not scored by the public metric",
            "primary_score": {
                "value_m": None,
                "status": PRIMARY_SCORE_STATUS,
                "reason": (
                    "the public primary sources do not identify the distance formula, "
                    "earth model/radius, or percentile interpolation"
                ),
            },
        },
        "inputs": {
            "submission": {"path": str(submission_path), "sha256": _sha256(submission_path)},
            "ground_truth": {
                "path": str(ground_truth_path),
                "sha256": _sha256(ground_truth_path),
            },
            "profile": (
                {
                    "path": str(profile["path"]),
                    "sha256": profile["sha256"],
                    "role": profile["role"],
                    "profile_id": profile["profile_id"],
                }
                if profile is not None
                else None
            ),
            "submission_manifest": submission_manifest,
        },
        "key_validation": {
            "truth_keys": len(truth_keys),
            "submission_keys": len(submission_keys),
            "missing_truth_keys": len(missing_keys),
            "missing_truth_keys_by_phone": dict(sorted(missing_by_phone.items())),
            "extra_submission_keys": len(extra_keys),
            "allow_gaps": allow_gaps,
            "complete_key_match": not missing_keys and not extra_keys,
        },
        "metrics": {
            # Kept as a null compatibility field so consumers cannot mistake a
            # local variant for the hidden leaderboard score.
            "kaggle_metric_m": None,
            "primary_score_m": None,
            "primary_score_status": PRIMARY_SCORE_STATUS,
            "score_variants_m": {
                variant_id: sum(values) / len(values)
                for variant_id, values in sorted(score_values.items())
            },
            "phone_count_scored": len(phone_names),
            "truth_phone_count": len({row.phone for row in truth}),
            "prediction_rows_scored": len(submission),
            "truth_rows": len(truth),
            "coverage_ratio": len(submission) / len(truth),
        },
        "phones": phone_metrics,
    }
    _atomic_write(output_json_path, _json_bytes(report))
    return report


def _profile_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--profile", type=Path)
    parser.add_argument("--role", choices=("development", "holdout"))
    parser.add_argument("--dataset-id")


def build_generate_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog=os.environ.get("GNSS_CLI_NAME"),
        description=(
            "Generate phone,UnixTimeMillis,LatitudeDegrees,LongitudeDegrees "
            "without reading ground truth."
        ),
    )
    parser.add_argument("--position", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--manifest", type=Path)
    parser.add_argument("--phone", required=True)
    parser.add_argument("--device-gnss", type=Path)
    parser.add_argument("--skip-epochs", type=int)
    parser.add_argument(
        "--gps-utc-leap-seconds",
        type=int,
        default=DEFAULT_GPS_UTC_LEAP_SECONDS,
    )
    _profile_args(parser)
    return parser


def build_evaluate_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog=os.environ.get("GNSS_CLI_NAME"),
        description=(
            "Audit a GSDC 2023 submission with explicit WGS84/Vincenty and "
            "spherical/Haversine P50/P95 diagnostic variants."
        ),
    )
    parser.add_argument("--submission", type=Path, required=True)
    parser.add_argument("--ground-truth", type=Path, required=True)
    parser.add_argument("--output-json", type=Path, required=True)
    parser.add_argument("--submission-manifest", type=Path)
    parser.add_argument("--phone")
    parser.add_argument(
        "--allow-gaps",
        action="store_true",
        help="Allow missing truth keys, while still rejecting duplicate/extra keys.",
    )
    _profile_args(parser)
    return parser


def main_generate(argv: list[str] | None = None) -> int:
    args = build_generate_parser().parse_args(argv)
    try:
        manifest = generate_submission(
            args.position,
            args.output,
            args.phone,
            device_gnss_path=args.device_gnss,
            profile_path=args.profile,
            role=args.role,
            dataset_id=args.dataset_id,
            skip_epochs=args.skip_epochs,
            gps_utc_leap_seconds=args.gps_utc_leap_seconds,
            manifest_path=args.manifest,
        )
    except (OSError, ValueError) as exc:
        raise SystemExit(f"smartphone Kaggle submission generation failed: {exc}") from exc
    print(f"Smartphone Kaggle submission: {args.output}")
    print(f"Submission manifest: {manifest['artifacts']['manifest']['path']}")
    return 0


def main_evaluate(argv: list[str] | None = None) -> int:
    args = build_evaluate_parser().parse_args(argv)
    try:
        report = evaluate_submission(
            args.submission,
            args.ground_truth,
            args.output_json,
            phone=args.phone,
            profile_path=args.profile,
            role=args.role,
            dataset_id=args.dataset_id,
            allow_gaps=args.allow_gaps,
            submission_manifest_path=args.submission_manifest,
        )
    except (OSError, ValueError) as exc:
        raise SystemExit(f"smartphone Kaggle evaluation failed: {exc}") from exc
    print(f"Smartphone Kaggle evaluation: {args.output_json}")
    print(f"Primary public score: {report['metrics']['primary_score_status']}")
    for variant_id, score in report["metrics"]["score_variants_m"].items():
        print(f"Diagnostic score ({variant_id}): {score:.9f} m")
    return 0
