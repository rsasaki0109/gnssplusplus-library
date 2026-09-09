#!/usr/bin/env python3
"""Audit systematic phone/model residuals of the sealed native Phase31 lane.

This is a development-only diagnostic.  It reads the three already
materialized Phase29 ``ground_truth.csv`` files and the already sealed Phase31
submissions exactly once in one process.  It never runs the solver, opens the
archive, regenerates a submission, reads a MAT file, or touches validation or
holdout truth.

The candidate CSV contains only latitude/longitude.  Consequently the audit
reports the observable local ENU horizontal residual (east,north); the up
component is explicitly unavailable rather than being filled from a result
file.  Raw IMU samples are used only to label coarse attitude/orientation
groups for a leakage-safe dependence diagnostic.  No correction is fitted or
applied in this phase.
"""

from __future__ import annotations

import argparse
import bisect
import csv
import hashlib
import io
import json
import math
import os
from pathlib import Path
import statistics
import sys
import tempfile
from typing import Any, Iterable


COMMANDS_DIR = Path(__file__).resolve().parents[1]
ROOT = COMMANDS_DIR.parent.parent
FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_phase36_phone_bias_audit_freeze_v1.json"
DEFAULT_OUTPUT = ROOT / "output/smartphone-r5/phase36-phone-bias-audit-v1"
RESULT_NAME = "phone_bias_audit.json"
MANIFEST_NAME = "phone_bias_audit.manifest.json"
FREEZE_SCHEMA = "smartphone-r5-phase36-phone-bias-audit-freeze.v1"
RESULT_SCHEMA = "smartphone-r5-phase36-phone-bias-audit-result.v1"
MANIFEST_SCHEMA = "smartphone-r5-phase36-phone-bias-audit-manifest.v1"
ROUTES = (
    "2021-03-16-18-59-us-ca-mtv-a/pixel5",
    "2021-07-27-19-49-us-ca-mtv-b/pixel4",
    "2021-07-14-20-50-us-ca-mtv-e/sm-g988b",
)
FRESH_VALIDATION = "2023-05-09-21-32-us-ca-mtv-pe1/pixel5"
FUTURE_HOLDOUT = "2023-05-16-19-54-us-ca-mtv-xe1/pixel5"
IMU_MAX_AGE_MS = 250
WGS84_A = 6378137.0
WGS84_F = 1.0 / 298.257223563
WGS84_B = WGS84_A * (1.0 - WGS84_F)
GRAVITY_MIN_MPS2 = 5.0
GRAVITY_MAX_MPS2 = 15.0
EPS = 1e-12


class Phase36Error(ValueError):
    """Raised when the frozen Phase36 audit contract cannot be proven."""


def _fail(message: str) -> Phase36Error:
    return Phase36Error(message)


def _load_json(path: Path, label: str) -> dict[str, Any]:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise _fail(f"invalid {label}: {path}: {exc}") from exc
    if not isinstance(payload, dict):
        raise _fail(f"{label} must be a JSON object: {path}")
    return payload


def _sha256_bytes(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def _read_bytes_once(path: Path, label: str, expected_sha256: str | None = None) -> bytes:
    """Read one file once and verify its digest without a second open."""

    if path.suffix.lower() == ".mat" or ".mat" in str(path).lower():
        raise _fail(f"MAT input is forbidden: {path}")
    try:
        data = path.read_bytes()
    except OSError as exc:
        raise _fail(f"failed to read {label}: {path}: {exc}") from exc
    digest = _sha256_bytes(data)
    if expected_sha256 is not None and digest != expected_sha256:
        raise _fail(f"{label} hash mismatch: {path}: {digest} != {expected_sha256}")
    return data


def _parse_float(value: str | None, field: str, line: int, *, allow_blank: bool = False) -> float | None:
    token = "" if value is None else value.strip()
    if not token:
        if allow_blank:
            return None
        raise _fail(f"line {line}: {field} is blank")
    try:
        number = float(token)
    except ValueError as exc:
        raise _fail(f"line {line}: {field} is not numeric: {value!r}") from exc
    if not math.isfinite(number):
        raise _fail(f"line {line}: {field} is not finite")
    return number


def _parse_int(value: str | None, field: str, line: int) -> int:
    token = "" if value is None else value.strip()
    try:
        number = int(token)
    except ValueError as exc:
        raise _fail(f"line {line}: {field} is not an integer: {value!r}") from exc
    if number < 0:
        raise _fail(f"line {line}: {field} is negative")
    return number


def _fields(reader: csv.DictReader, path: Path) -> list[str]:
    fields = list(reader.fieldnames or ())
    if not fields or any(field is None or field == "" for field in fields):
        raise _fail(f"CSV has no valid header: {path}")
    if len(fields) != len(set(fields)):
        raise _fail(f"CSV has duplicate header fields: {path}")
    return fields


def _reader(data: bytes) -> csv.DictReader:
    # Decoding from the one in-memory byte buffer keeps the source file at one
    # read while retaining csv.DictReader's strict row semantics.
    try:
        text = data.decode("utf-8-sig")
    except UnicodeDecodeError as exc:
        raise _fail(f"CSV is not UTF-8: {exc}") from exc
    return csv.DictReader(io.StringIO(text, newline=""))


def _read_submission(path: Path, expected_sha256: str) -> list[dict[str, Any]]:
    data = _read_bytes_once(path, "sealed candidate submission", expected_sha256)
    reader = _reader(data)
    expected_fields = ["phone", "UnixTimeMillis", "LatitudeDegrees", "LongitudeDegrees"]
    if _fields(reader, path) != expected_fields:
        raise _fail(f"candidate header changed: {path}")
    rows: list[dict[str, Any]] = []
    seen: set[tuple[str, int]] = set()
    for line, raw in enumerate(reader, start=2):
        if None in raw:
            raise _fail(f"line {line}: candidate row has extra columns")
        phone = (raw.get("phone") or "")
        timestamp = _parse_int(raw.get("UnixTimeMillis"), "UnixTimeMillis", line)
        latitude = _parse_float(raw.get("LatitudeDegrees"), "LatitudeDegrees", line)
        longitude = _parse_float(raw.get("LongitudeDegrees"), "LongitudeDegrees", line)
        assert latitude is not None and longitude is not None
        if not -90.0 <= latitude <= 90.0 or not -180.0 <= longitude <= 180.0:
            raise _fail(f"line {line}: candidate coordinate outside bounds")
        key = (phone, timestamp)
        if key in seen:
            raise _fail(f"line {line}: duplicate candidate key {key!r}")
        seen.add(key)
        rows.append({"phone": phone, "timestamp": timestamp, "lat": latitude, "lon": longitude})
    if not rows:
        raise _fail(f"candidate is empty: {path}")
    return rows


def _read_truth(path: Path, expected_sha256: str, default_phone: str) -> list[dict[str, Any]]:
    data = _read_bytes_once(path, "Phase29 materialized development truth", expected_sha256)
    reader = _reader(data)
    fields = _fields(reader, path)
    required = ("UnixTimeMillis", "LatitudeDegrees", "LongitudeDegrees", "AltitudeMeters")
    if any(field not in fields for field in required):
        raise _fail(f"truth header is missing required fields: {path}")
    has_phone = "phone" in fields
    rows: list[dict[str, Any]] = []
    seen: set[tuple[str, int]] = set()
    for line, raw in enumerate(reader, start=2):
        if None in raw:
            raise _fail(f"line {line}: truth row has extra columns")
        phone = (raw.get("phone") or default_phone) if has_phone else default_phone
        timestamp = _parse_int(raw.get("UnixTimeMillis"), "UnixTimeMillis", line)
        latitude = _parse_float(raw.get("LatitudeDegrees"), "LatitudeDegrees", line)
        longitude = _parse_float(raw.get("LongitudeDegrees"), "LongitudeDegrees", line)
        altitude = _parse_float(raw.get("AltitudeMeters"), "AltitudeMeters", line)
        assert latitude is not None and longitude is not None and altitude is not None
        if not -90.0 <= latitude <= 90.0 or not -180.0 <= longitude <= 180.0:
            raise _fail(f"line {line}: truth coordinate outside bounds")
        key = (phone, timestamp)
        if key in seen:
            raise _fail(f"line {line}: duplicate truth key {key!r}")
        seen.add(key)
        rows.append({"phone": phone, "timestamp": timestamp, "lat": latitude, "lon": longitude, "alt": altitude})
    if not rows:
        raise _fail(f"truth is empty: {path}")
    return rows


def _read_imu(
    path: Path,
) -> tuple[dict[str, list[tuple[int, float, float, float]]], str, dict[str, int]]:
    data = _read_bytes_once(path, "raw device_imu.csv")
    reader = _reader(data)
    expected = {
        "MessageType",
        "utcTimeMillis",
        "MeasurementX",
        "MeasurementY",
        "MeasurementZ",
    }
    fields = _fields(reader, path)
    if not expected.issubset(fields):
        raise _fail(f"raw IMU header is missing required fields: {path}")
    samples: dict[str, list[tuple[int, float, float, float]]] = {
        "UncalAccel": [],
        "UncalMag": [],
    }
    raw_counts: dict[str, int] = {}
    for line, raw in enumerate(reader, start=2):
        if None in raw:
            raise _fail(f"line {line}: IMU row has extra columns")
        message_type = (raw.get("MessageType") or "").strip()
        raw_counts[message_type] = raw_counts.get(message_type, 0) + 1
        if message_type not in samples:
            continue
        timestamp = _parse_int(raw.get("utcTimeMillis"), "utcTimeMillis", line)
        x = _parse_float(raw.get("MeasurementX"), "MeasurementX", line)
        y = _parse_float(raw.get("MeasurementY"), "MeasurementY", line)
        z = _parse_float(raw.get("MeasurementZ"), "MeasurementZ", line)
        assert x is not None and y is not None and z is not None
        samples[message_type].append((timestamp, x, y, z))
    for message_type in samples:
        samples[message_type].sort(key=lambda sample: sample[0])
    return samples, _sha256_bytes(data), raw_counts


def _verify_freeze(path: Path) -> dict[str, Any]:
    freeze = _load_json(path, "Phase36 freeze")
    if freeze.get("schema_version") != FREEZE_SCHEMA:
        raise _fail("Phase36 freeze schema mismatch")
    if freeze.get("status") != "frozen-before-phase36-residual-truth-read":
        raise _fail("Phase36 freeze is not pre-truth-read")
    scope = freeze.get("scope")
    if not isinstance(scope, dict) or scope.get("candidate_id") != "phase31_native_fgo_quality_anchor_tdcp_no_bridge":
        raise _fail("Phase36 candidate identity mismatch")
    if scope.get("candidate_artifacts_sealed") is not True or scope.get("development_only") is not True:
        raise _fail("Phase36 candidate/development seal is open")
    if scope.get("validation_and_holdout_remain_sealed") is not True:
        raise _fail("Phase36 validation/holdout seal is open")
    cohort = freeze.get("cohort")
    if not isinstance(cohort, dict) or tuple(cohort.get("routes", ())) != ROUTES:
        raise _fail("Phase36 route cohort mismatch")
    if cohort.get("fresh_validation") != FRESH_VALIDATION or cohort.get("future_holdout") != FUTURE_HOLDOUT:
        raise _fail("Phase36 validation/holdout identity mismatch")
    reads = freeze.get("read_contract")
    if not isinstance(reads, dict):
        raise _fail("Phase36 read contract missing")
    expected_reads = {
        "single_process": True,
        "truth_read_count_per_route": 1,
        "candidate_submission_read_count_per_route": 1,
        "raw_imu_read_count_per_route": 1,
        "archive_access": False,
        "truth_rematerialization": False,
        "solver_rerun": False,
        "candidate_regeneration": False,
        "post_score_tuning": False,
        "mat_read_or_generated": False,
        "token_or_kaggle_access": False,
    }
    if any(reads.get(key) != value for key, value in expected_reads.items()):
        raise _fail("Phase36 read contract is not closed")
    residual = freeze.get("residual_definition")
    if not isinstance(residual, dict):
        raise _fail("Phase36 residual definition missing")
    if residual.get("frame") != "local ENU at the first chronological matched truth coordinate; prediction minus truth in ECEF, then ECEF-to-ENU rotation at that fixed origin":
        raise _fail("Phase36 ENU frame definition changed")
    if residual.get("prefix_tail") != "chronologically sorted matched rows; prefix is first 25 percent and tail is last 25 percent, each with at least one row":
        raise _fail("Phase36 prefix/tail definition changed")
    inputs = freeze.get("sealed_inputs")
    if not isinstance(inputs, dict):
        raise _fail("Phase36 sealed inputs missing")
    candidate = inputs.get("candidate_submissions")
    truth = inputs.get("phase29_materialized_truth")
    raw_root = inputs.get("raw_imu_root")
    if not isinstance(candidate, dict) or not isinstance(truth, dict) or not isinstance(raw_root, str):
        raise _fail("Phase36 sealed input maps are invalid")
    if tuple(candidate) != ROUTES or tuple(truth) != ROUTES:
        raise _fail("Phase36 sealed input route order mismatch")
    for route in ROUTES:
        for mapping, label in ((candidate, "candidate"), (truth, "truth")):
            row = mapping.get(route)
            if not isinstance(row, dict) or not isinstance(row.get("path"), str) or not isinstance(row.get("sha256"), str):
                raise _fail(f"Phase36 {label} pin missing: {route}")
            if ".mat" in row["path"].lower():
                raise _fail(f"Phase36 MAT path is forbidden: {row['path']}")
    if ".mat" in raw_root.lower() or "validation" in raw_root.lower() or "holdout" in raw_root.lower():
        raise _fail("Phase36 raw IMU root is not permitted")
    return freeze


def _ecef(latitude: float, longitude: float, altitude: float) -> tuple[float, float, float]:
    lat = math.radians(latitude)
    lon = math.radians(longitude)
    sin_lat = math.sin(lat)
    cos_lat = math.cos(lat)
    radius = WGS84_A / math.sqrt(1.0 - WGS84_F * (2.0 - WGS84_F) * sin_lat * sin_lat)
    return (
        (radius + altitude) * cos_lat * math.cos(lon),
        (radius + altitude) * cos_lat * math.sin(lon),
        (radius * (1.0 - WGS84_F) ** 2 + altitude) * sin_lat,
    )


def _enu_delta(
    predicted_lat: float,
    predicted_lon: float,
    truth_lat: float,
    truth_lon: float,
    truth_alt: float,
    origin_ecef: tuple[float, float, float],
    origin_lat: float,
    origin_lon: float,
) -> tuple[float, float]:
    # The submission contract has no altitude.  Use the truth altitude for
    # both ECEF calls only to project horizontal latitude/longitude error; this
    # makes the vertical component exactly and explicitly unobservable.
    predicted = _ecef(predicted_lat, predicted_lon, truth_alt)
    actual = _ecef(truth_lat, truth_lon, truth_alt)
    dx, dy, dz = (predicted[i] - actual[i] for i in range(3))
    lat = math.radians(origin_lat)
    lon = math.radians(origin_lon)
    east = -math.sin(lon) * dx + math.cos(lon) * dy
    north = (
        -math.sin(lat) * math.cos(lon) * dx
        - math.sin(lat) * math.sin(lon) * dy
        + math.cos(lat) * dz
    )
    # origin_ecef is intentionally accepted and checked by the caller to keep
    # the fixed-origin contract evident; horizontal differences do not require
    # subtracting it because it cancels between predicted and actual.
    _ = origin_ecef
    return east, north


def _median(values: Iterable[float]) -> float:
    values_list = list(values)
    if not values_list:
        raise _fail("median of empty residual set")
    return float(statistics.median(values_list))


def _vector_median(vectors: list[tuple[float, float]]) -> tuple[float, float]:
    return (_median(vector[0] for vector in vectors), _median(vector[1] for vector in vectors))


def _mad(vectors: list[tuple[float, float]], med: tuple[float, float]) -> tuple[float, float]:
    return (
        _median(abs(vector[0] - med[0]) for vector in vectors),
        _median(abs(vector[1] - med[1]) for vector in vectors),
    )


def _mean(vectors: list[tuple[float, float]]) -> tuple[float, float]:
    return (
        sum(vector[0] for vector in vectors) / len(vectors),
        sum(vector[1] for vector in vectors) / len(vectors),
    )


def _covariance(vectors: list[tuple[float, float]]) -> list[list[float]]:
    if len(vectors) < 2:
        return [[0.0, 0.0], [0.0, 0.0]]
    mean = _mean(vectors)
    divisor = float(len(vectors) - 1)
    return [
        [
            sum((vector[0] - mean[0]) ** 2 for vector in vectors) / divisor,
            sum((vector[0] - mean[0]) * (vector[1] - mean[1]) for vector in vectors) / divisor,
        ],
        [
            sum((vector[1] - mean[1]) * (vector[0] - mean[0]) for vector in vectors) / divisor,
            sum((vector[1] - mean[1]) ** 2 for vector in vectors) / divisor,
        ],
    ]


def _inliers(vectors: list[tuple[float, float]], med: tuple[float, float], mad: tuple[float, float]) -> list[tuple[float, float]]:
    result: list[tuple[float, float]] = []
    for vector in vectors:
        keep = True
        for index in range(2):
            delta = abs(vector[index] - med[index])
            if mad[index] <= EPS:
                keep = keep and delta <= EPS
            else:
                keep = keep and delta <= 3.0 * mad[index]
        if keep:
            result.append(vector)
    return result


def _summary(vectors: list[tuple[float, float]]) -> dict[str, Any]:
    if not vectors:
        return {"count": 0}
    med = _vector_median(vectors)
    mad = _mad(vectors, med)
    inlier = _inliers(vectors, med, mad)
    mean = _mean(vectors)
    radial = [math.hypot(vector[0], vector[1]) for vector in vectors]
    return {
        "count": len(vectors),
        "median_enu_m": {"east": med[0], "north": med[1], "up": None},
        "mad_enu_m": {"east": mad[0], "north": mad[1], "up": None},
        "mean_enu_m": {"east": mean[0], "north": mean[1], "up": None},
        "rms_horizontal_m": math.sqrt(sum(value * value for value in radial) / len(radial)),
        "median_horizontal_m": _median(radial),
        "p95_horizontal_m": _percentile(radial, 0.95),
        "covariance_enu_m2": _covariance(vectors),
        "component_3mad_inlier_count": len(inlier),
        "component_3mad_inlier_fraction": len(inlier) / len(vectors),
        "component_3mad_inlier_covariance_enu_m2": _covariance(inlier),
    }


def _percentile(values: list[float], fraction: float) -> float:
    ordered = sorted(values)
    if not ordered:
        raise _fail("percentile of empty residual set")
    if len(ordered) == 1:
        return ordered[0]
    index = (len(ordered) - 1) * fraction
    lower = math.floor(index)
    upper = math.ceil(index)
    if lower == upper:
        return ordered[lower]
    weight = index - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def _sample_for_time(
    samples: list[tuple[int, float, float, float]], times: list[int], timestamp: int
) -> tuple[float, float, float] | None:
    if not samples:
        return None
    index = bisect.bisect_right(times, timestamp) - 1
    if index < 0 or timestamp - samples[index][0] > IMU_MAX_AGE_MS:
        return None
    _, x, y, z = samples[index]
    return x, y, z


def _orientation_label(
    accel: tuple[float, float, float] | None,
    magnetometer: tuple[float, float, float] | None,
) -> tuple[str, str, str]:
    if accel is None:
        return "unavailable", "unavailable", "unavailable"
    ax, ay, az = accel
    accel_norm = math.sqrt(ax * ax + ay * ay + az * az)
    if not math.isfinite(accel_norm) or accel_norm < EPS:
        return "invalid", "invalid", "unavailable"
    axis_values = (abs(ax), abs(ay), abs(az))
    axis = max(range(3), key=axis_values.__getitem__)
    axis_names = ("x", "y", "z")
    axis_ratio = axis_values[axis] / accel_norm
    gravity_axis = axis_names[axis] + ("+" if (ax, ay, az)[axis] >= 0.0 else "-") if axis_ratio >= 0.75 else "mixed"
    if not (GRAVITY_MIN_MPS2 <= accel_norm <= GRAVITY_MAX_MPS2):
        return gravity_axis, "dynamic", "unavailable"
    if magnetometer is None:
        return gravity_axis, "gravity_valid", "unavailable"
    mx, my, mz = magnetometer
    mag_norm = math.sqrt(mx * mx + my * my + mz * mz)
    if not math.isfinite(mag_norm) or mag_norm < EPS:
        return gravity_axis, "gravity_valid", "unavailable"
    # Android's body-axis convention is not assumed to be a calibrated world
    # frame.  This is only a coarse, raw-only tilt-compensated heading label.
    pitch = math.asin(max(-1.0, min(1.0, -ax / accel_norm)))
    roll = math.atan2(ay, az)
    mx_horizontal = mx * math.cos(pitch) + mz * math.sin(pitch)
    my_horizontal = (
        mx * math.sin(roll) * math.sin(pitch)
        + my * math.cos(roll)
        - mz * math.sin(roll) * math.cos(pitch)
    )
    if abs(mx_horizontal) + abs(my_horizontal) < EPS:
        return gravity_axis, "gravity_valid", "unavailable"
    heading = math.atan2(-my_horizontal, mx_horizontal) % (2.0 * math.pi)
    octant = int(math.floor(heading / (2.0 * math.pi / 8.0)))
    return gravity_axis, "gravity_valid", f"octant_{octant}"


def _group_orientation(
    vectors: list[tuple[float, float]], labels: list[tuple[str, str, str]], index: int
) -> dict[str, Any]:
    groups: dict[str, list[tuple[float, float]]] = {}
    for vector, label in zip(vectors, labels):
        groups.setdefault(label[index], []).append(vector)
    report: dict[str, Any] = {}
    for key in sorted(groups):
        report[key] = _summary(groups[key])
    overall = _summary(vectors)
    comparable = []
    for key, summary in report.items():
        if summary.get("count", 0) >= 20:
            med = summary["median_enu_m"]
            overall_med = overall["median_enu_m"]
            comparable.append(math.hypot(med["east"] - overall_med["east"], med["north"] - overall_med["north"]))
    return {
        "groups": report,
        "groups_with_at_least_20": sum(summary.get("count", 0) >= 20 for summary in report.values()),
        "max_group_median_delta_from_overall_m": max(comparable, default=None),
    }


def _route_audit(route: str, candidate: list[dict[str, Any]], truth: list[dict[str, Any]], imu: dict[str, list[tuple[int, float, float, float]]]) -> dict[str, Any]:
    expected_phone = route.split("/", 1)[1]
    candidate_by_key = {(row["phone"], row["timestamp"]): row for row in candidate}
    truth_by_key = {(row["phone"], row["timestamp"]): row for row in truth}
    candidate_keys = set(candidate_by_key)
    truth_keys = set(truth_by_key)
    matched_keys = candidate_keys & truth_keys
    if not matched_keys:
        raise _fail(f"no matched keys for {route}")
    wrong_candidate_phones = sorted({phone for phone, _ in candidate_keys if phone != route})
    wrong_truth_phones = sorted({phone for phone, _ in truth_keys if phone != expected_phone and phone != route})
    # Phase31 submissions use the dataset route as phone; the truth has no
    # phone column and is assigned the basename phone by this audit.
    if wrong_candidate_phones:
        raise _fail(f"candidate phone mismatch for {route}: {wrong_candidate_phones[:3]}")
    if wrong_truth_phones:
        raise _fail(f"truth phone mismatch for {route}: {wrong_truth_phones[:3]}")
    ordered_keys = sorted(matched_keys, key=lambda key: key[1])
    first_truth = truth_by_key[ordered_keys[0]]
    origin_ecef = _ecef(first_truth["lat"], first_truth["lon"], first_truth["alt"])
    vectors: list[tuple[float, float]] = []
    labels: list[tuple[str, str, str]] = []
    accel_samples = imu.get("UncalAccel", [])
    mag_samples = imu.get("UncalMag", [])
    accel_times = [sample[0] for sample in accel_samples]
    mag_times = [sample[0] for sample in mag_samples]
    orientation_available = 0
    for key in ordered_keys:
        prediction = candidate_by_key[key]
        reference = truth_by_key[key]
        vectors.append(
            _enu_delta(
                prediction["lat"],
                prediction["lon"],
                reference["lat"],
                reference["lon"],
                reference["alt"],
                origin_ecef,
                first_truth["lat"],
                first_truth["lon"],
            )
        )
        accel = _sample_for_time(accel_samples, accel_times, key[1])
        magnetometer = _sample_for_time(mag_samples, mag_times, key[1])
        label = _orientation_label(accel, magnetometer)
        labels.append(label)
        orientation_available += label[0] != "unavailable"
    overall = _summary(vectors)
    prefix_count = max(1, math.ceil(len(vectors) * 0.25))
    prefix = _summary(vectors[:prefix_count])
    tail = _summary(vectors[-prefix_count:])
    prefix_med = prefix.get("median_enu_m", {"east": None, "north": None})
    tail_med = tail.get("median_enu_m", {"east": None, "north": None})
    prefix_tail_delta = math.hypot(tail_med["east"] - prefix_med["east"], tail_med["north"] - prefix_med["north"])
    return {
        "route": route,
        "phone_model": expected_phone,
        "candidate_rows": len(candidate),
        "truth_rows": len(truth),
        "matched_rows": len(matched_keys),
        "missing_prediction_count": len(truth_keys - candidate_keys),
        "extra_prediction_count": len(candidate_keys - truth_keys),
        "same_matched_key_set": True,
        "origin": {
            "definition": "first chronological matched truth coordinate",
            "latitude_degrees": first_truth["lat"],
            "longitude_degrees": first_truth["lon"],
            "altitude_m": first_truth["alt"],
        },
        "overall": overall,
        "prefix_first_25_percent": prefix,
        "tail_last_25_percent": tail,
        "prefix_tail_median_delta_m": prefix_tail_delta,
        "orientation": {
            "imu_match_window_ms": IMU_MAX_AGE_MS,
            "matched_accel_label_count": orientation_available,
            "matched_accel_label_fraction": orientation_available / len(vectors),
            "gravity_axis_sign": _group_orientation(vectors, labels, 0),
            "gravity_quality": _group_orientation(vectors, labels, 1),
            "tilt_compensated_magnetic_heading_octant": _group_orientation(vectors, labels, 2),
        },
    }


def _cross_route(routes: dict[str, dict[str, Any]]) -> dict[str, Any]:
    medians = {
        route: (
            report["overall"]["median_enu_m"]["east"],
            report["overall"]["median_enu_m"]["north"],
        )
        for route, report in routes.items()
    }
    norms = {route: math.hypot(value[0], value[1]) for route, value in medians.items()}
    pairwise: dict[str, float] = {}
    for index, left in enumerate(ROUTES):
        for right in ROUTES[index + 1 :]:
            pairwise[f"{left}__vs__{right}"] = math.hypot(
                medians[left][0] - medians[right][0], medians[left][1] - medians[right][1]
            )
    models = [route.split("/", 1)[1] for route in ROUTES]
    return {
        "route_median_enu_m": {
            route: {"east": medians[route][0], "north": medians[route][1], "up": None}
            for route in ROUTES
        },
        "route_median_horizontal_norm_m": norms,
        "pairwise_route_median_delta_m": pairwise,
        "phone_model_route_counts": {model: models.count(model) for model in sorted(set(models))},
        "same_model_repeated_route_count": max((models.count(model) for model in set(models)), default=0),
        "constant_identifiable": False,
        "constant_identifiability_reason": "Each exact phone model has one route in the declared cohort; ENU route medians are not a repeated-identity estimate and the Pixel5 family has only one route.",
        "body_frame_vector_identifiable": False,
        "body_frame_identifiability_reason": "Raw UncalAccel/UncalMag provide coarse attitude labels but no calibrated, device-to-antenna body-frame pose or repeated same-model route cohort; orientation groups are therefore diagnostic, not a source-supported lever-arm estimate.",
    }


def _atomic_write(path: Path, payload: bytes) -> None:
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


def _atomic_json(path: Path, payload: dict[str, Any]) -> None:
    _atomic_write(path, (json.dumps(payload, indent=2, sort_keys=True) + "\n").encode("utf-8"))


def _record_path(path: Path) -> str:
    try:
        return str(path.relative_to(ROOT))
    except ValueError:
        return str(path)


def audit(freeze_path: Path, output_root: Path) -> dict[str, Any]:
    freeze = _verify_freeze(freeze_path)
    inputs = freeze["sealed_inputs"]
    candidate_pins = inputs["candidate_submissions"]
    truth_pins = inputs["phase29_materialized_truth"]
    raw_root = ROOT / inputs["raw_imu_root"]
    route_reports: dict[str, dict[str, Any]] = {}
    read_counts = {"truth": 0, "candidate_submission": 0, "raw_imu": 0}
    for route in ROUTES:
        candidate_pin = candidate_pins[route]
        truth_pin = truth_pins[route]
        candidate_path = ROOT / candidate_pin["path"]
        truth_path = ROOT / truth_pin["path"]
        route_path, phone = route.split("/", 1)
        imu_path = raw_root / route_path / phone / "device_imu.csv"
        candidate = _read_submission(candidate_path, candidate_pin["sha256"])
        read_counts["candidate_submission"] += 1
        # Truth is opened exactly once in this process, after the freeze has
        # been fully verified and only from the already materialized Phase29
        # development directory.
        truth = _read_truth(truth_path, truth_pin["sha256"], route)
        read_counts["truth"] += 1
        imu, imu_sha256, imu_counts = _read_imu(imu_path)
        read_counts["raw_imu"] += 1
        route_reports[route] = _route_audit(route, candidate, truth, imu)
        route_reports[route]["input_hashes"] = {
            "candidate_submission_sha256": candidate_pin["sha256"],
            "phase29_truth_sha256": truth_pin["sha256"],
            "device_imu_sha256": imu_sha256,
        }
        route_reports[route]["raw_imu_message_type_counts"] = imu_counts
    result = {
        "schema_version": RESULT_SCHEMA,
        "phase": 36,
        "status": "development-only-no-go",
        "decision": "no-go-no-calibration-no-validation-no-kaggle",
        "purpose": "Assess phone/model systematic residual and a body-frame lever-arm-like correction without fitting or applying any correction.",
        "freeze": {
            "path": str(freeze_path.relative_to(ROOT)),
            "sha256": _sha256_bytes(freeze_path.read_bytes()),
        },
        "read_contract": {
            "single_process": True,
            "truth_open_count_total": read_counts["truth"],
            "truth_open_count_per_route": 1,
            "candidate_submission_open_count_total": read_counts["candidate_submission"],
            "candidate_submission_open_count_per_route": 1,
            "raw_imu_open_count_total": read_counts["raw_imu"],
            "raw_imu_open_count_per_route": 1,
            "validation_truth_open_count": 0,
            "future_holdout_truth_open_count": 0,
            "archive_access": False,
            "mat_read_or_generated": False,
            "solver_rerun": False,
            "candidate_regenerated": False,
            "post_score_tuning": False,
            "token_or_kaggle_access": False,
        },
        "residual_definition": freeze["residual_definition"],
        "routes": route_reports,
        "cross_route": _cross_route(route_reports),
        "identifiability": {
            "single_phone_family_or_model_constant": "no-go",
            "body_frame_lever_arm_like_vector": "no-go",
            "reason": "The cohort has one route per exact model, only one Pixel5 identity, and raw IMU lacks a calibrated device-to-antenna pose. Prefix/tail and coarse orientation diagnostics are evidence about stability only; they cannot identify a deployable constant without a repeated route-disjoint same-model cohort.",
            "fit_or_application_performed": False,
        },
        "next_raw_physical_factor": {
            "candidate": "source-supported Android antenna/measurement hardware model and raw timing/clock residual diagnostics, with an explicitly predeclared multi-route train cohort",
            "why": "A phone constant is not identifiable from the current one-route-per-model cohort; the next experiment should target a raw observable with a physical model and repeated identities before any calibration fit.",
        },
        "promotion": {
            "phase31_champion_preserved": True,
            "fresh_validation_remains_sealed": True,
            "future_holdout_remains_sealed": True,
            "native_calibration_added": False,
            "kaggle_submission": False,
        },
    }
    result_bytes = (json.dumps(result, indent=2, sort_keys=True) + "\n").encode("utf-8")
    result_path = output_root / RESULT_NAME
    _atomic_write(result_path, result_bytes)
    manifest = {
        "schema_version": MANIFEST_SCHEMA,
        "result": {"path": _record_path(result_path), "sha256": _sha256_bytes(result_bytes)},
        "freeze": {"path": _record_path(freeze_path), "sha256": _sha256_bytes(freeze_path.read_bytes())},
        "routes": list(ROUTES),
        "truth_open_count_per_route": 1,
        "single_process": True,
        "mat_read_or_generated": False,
        "validation_truth_open_count": 0,
        "future_holdout_truth_open_count": 0,
    }
    _atomic_json(output_root / MANIFEST_NAME, manifest)
    return result


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=("audit",), help="run the frozen audit")
    parser.add_argument("--freeze", type=Path, default=FREEZE)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args(argv)
    try:
        result = audit(args.freeze, args.output)
    except Phase36Error as exc:
        print(f"phase36 phone-bias audit: ERROR: {exc}", file=sys.stderr)
        return 2
    print(json.dumps({"status": result["status"], "decision": result["decision"], "output": str(args.output / RESULT_NAME)}, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
