#!/usr/bin/env python3
"""Diagnose the sealed Phase29 Samsung tail without rerunning the solver.

This command is intentionally a scoring/diagnostic reader, not an inference
path.  It verifies the Phase29 control and candidate artifacts, reads only
raw Android timing/status fields for structural signals, and then reads the
already-materialized Samsung development truth exactly once.  Pixel truth,
validation/holdout truth, MAT files, and Kaggle inputs are rejected.

The native output does not retain the solver timestamp sidecar.  Consequently
the per-key exact/interpolated label is reported as an explicit inference:
the sealed ``raw_utc_key_contract`` counts are reconciled with a small,
rounding-safe ECEF linearity check performed independently in both sealed
lanes.  The authoritative aggregate counts remain in the native summaries.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import io
import json
import math
import os
from pathlib import Path
import re
import sys
import tempfile
from typing import Any

COMMANDS_DIR = Path(__file__).resolve().parents[1]
if str(COMMANDS_DIR) not in sys.path:
    sys.path.insert(0, str(COMMANDS_DIR))
from support.gnss_runtime import application_root  # noqa: E402

ROOT = application_root(__file__)
BENCHMARK_DIR = ROOT / "apps" / "commands" / "benchmarks"
if str(BENCHMARK_DIR) not in sys.path:
    sys.path.insert(0, str(BENCHMARK_DIR))
import gnss_smartphone_kaggle as kaggle  # noqa: E402


SCHEMA = "smartphone-r5-phase30-samsung-chronology.v1"
FREEZE_SCHEMA = "smartphone-r5-phase30-samsung-chronology-freeze.v1"
MANIFEST_SCHEMA = "smartphone-r5-phase30-samsung-chronology-manifest.v1"
FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_phase30_samsung_chronology_freeze_v1.json"
FREEZE_MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase30_samsung_chronology_freeze_v1_manifest.json"
PHASE29_RESULT = ROOT / "docs/use_cases/records/smartphone_r5_phase29_native_fgo_no_bridge_train_eval_result_v1.json"
DEFAULT_OUTPUT = ROOT / "output/smartphone-r5/phase30-samsung-chronology-v1"
ROUTE = "2021-07-14-20-50-us-ca-mtv-e/sm-g988b"
ARCHIVE_SHA256 = "bda30ab456e6fd6f83550c246e8dbd287306d5385f1f1069c99c16298e647408"
PHASE29_RESULT_SHA256 = "88d2cf8b91460e098873872cbcd1d0be8df2a15adf1d3a43749c7301e1533536"
TRUTH_SHA256 = "164a434967f0eeefde2812a27281d39e858024fba6e82015a9dc26bdfb3ad1a8"
TRUTH_BYTES = 113328
MAX_CONTINUITY_SPEED_MPS = 70.0
TAIL_ERROR_THRESHOLD_M = 500.0
LARGE_ERROR_THRESHOLD_M = 100.0
INTERPOLATION_INFERENCE_TOLERANCE_M = 0.01
RAW_CLOCK_GAP_THRESHOLD_MS = 1000.0

RAW_COLUMNS = (
    "MessageType",
    "utcTimeMillis",
    "TimeNanos",
    "HardwareClockDiscontinuityCount",
    "State",
    "Svid",
    "SignalType",
    "PseudorangeRateMetersPerSecond",
    "AccumulatedDeltaRangeState",
    "Cn0DbHz",
)
FORBIDDEN_DERIVED_COLUMNS = {
    "RawPseudorangeMeters",
    "RawPseudorangeUncertaintyMeters",
    "SvPositionXEcefMeters",
    "SvPositionYEcefMeters",
    "SvPositionZEcefMeters",
    "SvElevationDegrees",
    "SvAzimuthDegrees",
    "SvVelocityXEcefMetersPerSecond",
    "SvVelocityYEcefMetersPerSecond",
    "SvVelocityZEcefMetersPerSecond",
    "SvClockBiasMeters",
    "SvClockDriftMetersPerSecond",
    "WlsPositionXEcefMeters",
    "WlsPositionYEcefMeters",
    "WlsPositionZEcefMeters",
}
_INTEGER_RE = re.compile(r"^[+-]?\d+$")


class Phase30Error(ValueError):
    """Raised when the sealed diagnostic contract cannot be proven."""


def _sha256_bytes(payload: bytes) -> str:
    return hashlib.sha256(payload).hexdigest()


def sha256(path: Path) -> str:
    if not path.is_file():
        raise Phase30Error(f"missing file: {path}")
    digest = hashlib.sha256()
    try:
        with path.open("rb") as handle:
            for chunk in iter(lambda: handle.read(1024 * 1024), b""):
                digest.update(chunk)
    except OSError as exc:
        raise Phase30Error(f"failed to hash {path}: {exc}") from exc
    return digest.hexdigest()


def load_json(path: Path, label: str) -> dict[str, Any]:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise Phase30Error(f"invalid {label}: {path}") from exc
    if not isinstance(payload, dict):
        raise Phase30Error(f"{label} must be an object: {path}")
    return payload


def atomic_write(path: Path, payload: bytes) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".tmp", dir=str(path.parent)
    )
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


def atomic_json(path: Path, payload: dict[str, Any]) -> None:
    atomic_write(
        path,
        (json.dumps(payload, indent=2, sort_keys=True) + "\n").encode("utf-8"),
    )


def _parse_int(raw: str | None, field: str, row_number: int) -> int:
    token = "" if raw is None else raw.strip()
    if not token or not _INTEGER_RE.fullmatch(token):
        raise Phase30Error(f"row {row_number}: {field} is not an integer")
    try:
        return int(token)
    except ValueError as exc:
        raise Phase30Error(f"row {row_number}: invalid {field}") from exc


def _parse_float(raw: str | None, field: str, row_number: int) -> float:
    token = "" if raw is None else raw.strip()
    if not token:
        raise Phase30Error(f"row {row_number}: {field} is empty")
    try:
        value = float(token)
    except (TypeError, ValueError) as exc:
        raise Phase30Error(f"row {row_number}: invalid {field}") from exc
    if not math.isfinite(value):
        raise Phase30Error(f"row {row_number}: non-finite {field}")
    return value


def _load_prediction(path: Path, expected_phone: str) -> dict[int, tuple[float, float]]:
    if path.suffix.lower() != ".csv":
        raise Phase30Error(f"prediction must be CSV: {path}")
    rows: dict[int, tuple[float, float]] = {}
    previous: int | None = None
    try:
        with path.open(encoding="utf-8-sig", newline="") as handle:
            reader = csv.DictReader(handle)
            fields = tuple(reader.fieldnames or ())
            if fields != ("phone", "UnixTimeMillis", "LatitudeDegrees", "LongitudeDegrees"):
                raise Phase30Error(f"prediction header mismatch: {path}")
            for row_number, raw in enumerate(reader, start=2):
                phone = raw.get("phone") or ""
                if phone != expected_phone:
                    raise Phase30Error(f"prediction phone mismatch at row {row_number}")
                timestamp = _parse_int(raw.get("UnixTimeMillis"), "UnixTimeMillis", row_number)
                if previous is not None and timestamp <= previous:
                    raise Phase30Error(f"prediction timestamps are not strictly increasing: {path}")
                previous = timestamp
                latitude = _parse_float(raw.get("LatitudeDegrees"), "LatitudeDegrees", row_number)
                longitude = _parse_float(raw.get("LongitudeDegrees"), "LongitudeDegrees", row_number)
                if not -90.0 <= latitude <= 90.0 or not -180.0 <= longitude <= 180.0:
                    raise Phase30Error(f"prediction coordinate out of range at row {row_number}")
                if timestamp in rows:
                    raise Phase30Error(f"duplicate prediction timestamp {timestamp}")
                rows[timestamp] = (latitude, longitude)
    except OSError as exc:
        raise Phase30Error(f"failed to read prediction {path}: {exc}") from exc
    if not rows:
        raise Phase30Error(f"prediction is empty: {path}")
    return rows


def _read_truth_once(
    path: Path, expected_phone: str
) -> tuple[dict[int, tuple[float, float, float]], str, int]:
    """Read one already-materialized truth file once, hashing its same bytes."""

    if path.suffix.lower() != ".csv":
        raise Phase30Error("Phase30 truth path must be CSV")
    try:
        payload = path.read_bytes()
    except OSError as exc:
        raise Phase30Error(f"failed to read Phase30 truth: {path}") from exc
    digest = _sha256_bytes(payload)
    try:
        text = payload.decode("utf-8-sig")
    except UnicodeDecodeError as exc:
        raise Phase30Error("Phase30 truth is not UTF-8 CSV") from exc
    reader = csv.DictReader(io.StringIO(text))
    fields = tuple(reader.fieldnames or ())
    required = {"UnixTimeMillis", "LatitudeDegrees", "LongitudeDegrees"}
    if not required.issubset(fields):
        raise Phase30Error("Phase30 truth is missing coordinate fields")
    has_phone = "phone" in fields
    truth: dict[int, tuple[float, float, float]] = {}
    for row_number, raw in enumerate(reader, start=2):
        phone = (raw.get("phone") or expected_phone) if has_phone else expected_phone
        if phone != expected_phone:
            raise Phase30Error(f"Phase30 truth phone mismatch at row {row_number}")
        timestamp = _parse_int(raw.get("UnixTimeMillis"), "UnixTimeMillis", row_number)
        latitude = _parse_float(raw.get("LatitudeDegrees"), "LatitudeDegrees", row_number)
        longitude = _parse_float(raw.get("LongitudeDegrees"), "LongitudeDegrees", row_number)
        altitude = _parse_float(
            raw.get("AltitudeMeters", "0"), "AltitudeMeters", row_number
        )
        if not -90.0 <= latitude <= 90.0 or not -180.0 <= longitude <= 180.0:
            raise Phase30Error(f"Phase30 truth coordinate out of range at row {row_number}")
        if timestamp in truth:
            raise Phase30Error(f"duplicate Phase30 truth timestamp {timestamp}")
        truth[timestamp] = (latitude, longitude, altitude)
    if not truth:
        raise Phase30Error("Phase30 truth is empty")
    return truth, digest, len(payload)


def _finite_optional(raw: str, field: str, row_number: int) -> float | None:
    token = raw.strip()
    if not token:
        return None
    return _parse_float(token, field, row_number)


def _read_raw_epochs(path: Path) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    """Read only raw timing/status fields; enriched coordinates are never used."""

    epochs: dict[int, dict[str, Any]] = {}
    try:
        with path.open(encoding="utf-8-sig", newline="") as handle:
            reader = csv.DictReader(handle)
            fields = set(reader.fieldnames or ())
            missing = [field for field in RAW_COLUMNS if field not in fields]
            if missing:
                raise Phase30Error(f"raw GNSS missing fields: {', '.join(missing)}")
            ignored_derived = sorted(fields & FORBIDDEN_DERIVED_COLUMNS)
            previous_row_timestamp: int | None = None
            for row_number, raw in enumerate(reader, start=2):
                if raw.get("MessageType") != "Raw":
                    continue
                timestamp = _parse_int(raw.get("utcTimeMillis"), "utcTimeMillis", row_number)
                if previous_row_timestamp is not None and timestamp < previous_row_timestamp:
                    raise Phase30Error("raw GNSS utcTimeMillis moved backwards")
                previous_row_timestamp = timestamp
                time_nanos = _parse_int(raw.get("TimeNanos"), "TimeNanos", row_number)
                hcdc = _parse_int(
                    raw.get("HardwareClockDiscontinuityCount"),
                    "HardwareClockDiscontinuityCount",
                    row_number,
                )
                epoch = epochs.setdefault(
                    timestamp,
                    {
                        "timestamp": timestamp,
                        "time_nanos": time_nanos,
                        "hcdc_values": set(),
                        "raw_rows": 0,
                        "finite_doppler_rows": 0,
                        "finite_carrier_state_rows": 0,
                        "finite_cno_rows": 0,
                        "state_nonzero_rows": 0,
                    },
                )
                epoch["hcdc_values"].add(hcdc)
                epoch["raw_rows"] += 1
                if _finite_optional(
                    raw.get("PseudorangeRateMetersPerSecond", ""),
                    "PseudorangeRateMetersPerSecond",
                    row_number,
                ) is not None:
                    epoch["finite_doppler_rows"] += 1
                if _finite_optional(
                    raw.get("AccumulatedDeltaRangeState", ""),
                    "AccumulatedDeltaRangeState",
                    row_number,
                ) is not None:
                    epoch["finite_carrier_state_rows"] += 1
                if _finite_optional(raw.get("Cn0DbHz", ""), "Cn0DbHz", row_number) is not None:
                    epoch["finite_cno_rows"] += 1
                state = raw.get("State", "").strip()
                if state and state != "0":
                    epoch["state_nonzero_rows"] += 1
    except OSError as exc:
        raise Phase30Error(f"failed to read raw GNSS: {path}") from exc
    if not epochs:
        raise Phase30Error("raw GNSS has no Raw epochs")

    ordered = [epochs[key] for key in sorted(epochs)]
    segment = 0
    previous: dict[str, Any] | None = None
    segment_boundaries: list[int] = []
    for epoch in ordered:
        epoch["hcdc"] = min(epoch["hcdc_values"])
        epoch["hcdc_consistent"] = len(epoch["hcdc_values"]) == 1
        if previous is None:
            epoch["raw_gap_ms"] = None
            epoch["time_nanos_gap_ms"] = None
            epoch["raw_clock_segment"] = segment
        else:
            raw_gap_ms = float(epoch["timestamp"] - previous["timestamp"])
            time_nanos_gap_ms = abs(epoch["time_nanos"] - previous["time_nanos"]) / 1e6
            epoch["raw_gap_ms"] = raw_gap_ms
            epoch["time_nanos_gap_ms"] = time_nanos_gap_ms
            if (
                epoch["hcdc"] != previous["hcdc"]
                or time_nanos_gap_ms > RAW_CLOCK_GAP_THRESHOLD_MS
            ):
                segment += 1
                segment_boundaries.append(epoch["timestamp"])
            epoch["raw_clock_segment"] = segment
        previous = epoch
        epoch["hcdc_values"] = sorted(epoch["hcdc_values"])
    structural = {
        "raw_epoch_count": len(ordered),
        "target_epoch_count_after_warmup": max(0, len(ordered) - 1),
        "hcdc_unique": sorted({value for epoch in ordered for value in epoch["hcdc_values"]}),
        "hcdc_transition_count": sum(
            1
            for index in range(1, len(ordered))
            if ordered[index]["hcdc"] != ordered[index - 1]["hcdc"]
        ),
        "raw_gap_count_over_1000ms": sum(
            1 for epoch in ordered if epoch["raw_gap_ms"] is not None and epoch["raw_gap_ms"] > 1000.0
        ),
        "max_raw_gap_ms": max(
            (epoch["raw_gap_ms"] or 0.0 for epoch in ordered), default=0.0
        ),
        "max_time_nanos_gap_ms": max(
            (epoch["time_nanos_gap_ms"] or 0.0 for epoch in ordered), default=0.0
        ),
        "clock_segment_count": segment + 1,
        "clock_segment_boundaries_utc_ms": segment_boundaries,
        "derived_columns_present_but_ignored": ignored_derived,
        "inference_fields_used": list(RAW_COLUMNS),
        "forbidden_derived_fields_used": [],
    }
    return ordered, structural


def _ecef(latitude: float, longitude: float) -> tuple[float, float, float]:
    a = 6378137.0
    e2 = 6.6943799901413165e-3
    lat = math.radians(latitude)
    lon = math.radians(longitude)
    sin_lat = math.sin(lat)
    radius = a / math.sqrt(1.0 - e2 * sin_lat * sin_lat)
    return (
        radius * math.cos(lat) * math.cos(lon),
        radius * math.cos(lat) * math.sin(lon),
        radius * (1.0 - e2) * sin_lat,
    )


def _ecef_linearity_error(
    rows: dict[int, tuple[float, float]],
    timestamps: list[int],
    index: int,
) -> float | None:
    if index <= 0 or index + 1 >= len(timestamps):
        return None
    left_timestamp = timestamps[index - 1]
    timestamp = timestamps[index]
    right_timestamp = timestamps[index + 1]
    span = right_timestamp - left_timestamp
    if span <= 0 or timestamp <= left_timestamp or timestamp >= right_timestamp:
        return None
    left = _ecef(*rows[left_timestamp])
    middle = _ecef(*rows[timestamp])
    right = _ecef(*rows[right_timestamp])
    fraction = (timestamp - left_timestamp) / span
    predicted = tuple(left[axis] + fraction * (right[axis] - left[axis]) for axis in range(3))
    return math.sqrt(sum((middle[axis] - predicted[axis]) ** 2 for axis in range(3)))


def _source_lane_inference(
    control: dict[int, tuple[float, float]],
    candidate: dict[int, tuple[float, float]],
    control_summary: dict[str, Any],
    candidate_summary: dict[str, Any],
) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    timestamps = sorted(set(control) & set(candidate))
    contract = candidate_summary.get("raw_utc_key_contract", {})
    expected_interpolated = int(contract.get("interpolated_epochs", -1))
    expected_exact = int(contract.get("exact_solution_epochs", -1))
    rows: list[dict[str, Any]] = []
    inferred_count = 0
    for index, timestamp in enumerate(timestamps):
        control_error = _ecef_linearity_error(control, timestamps, index)
        candidate_error = _ecef_linearity_error(candidate, timestamps, index)
        inferred = (
            control_error is not None
            and candidate_error is not None
            and control_error <= INTERPOLATION_INFERENCE_TOLERANCE_M
            and candidate_error <= INTERPOLATION_INFERENCE_TOLERANCE_M
        )
        if inferred:
            inferred_count += 1
        rows.append(
            {
                "timestamp": timestamp,
                "source_lane": (
                    "inferred-interpolated"
                    if inferred
                    else "exact-by-sealed-alignment-count"
                ),
                "source_lane_basis": (
                    "both sealed lanes are within the fixed ECEF linearity tolerance"
                    if inferred
                    else "not classified as interpolation; native summary exact count is authoritative"
                ),
                "control_ecef_linearity_error_m": control_error,
                "candidate_ecef_linearity_error_m": candidate_error,
            }
        )
    report = {
        "method": "sealed-summary-count plus independent two-lane ECEF-linearity inference",
        "authoritative_summary_counts": {
            "control_exact_solution_epochs": control_summary.get("raw_utc_key_contract", {}).get(
                "exact_solution_epochs"
            ),
            "control_interpolated_epochs": control_summary.get("raw_utc_key_contract", {}).get(
                "interpolated_epochs"
            ),
            "candidate_exact_solution_epochs": expected_exact,
            "candidate_interpolated_epochs": expected_interpolated,
        },
        "fixed_linearity_tolerance_m": INTERPOLATION_INFERENCE_TOLERANCE_M,
        "inferred_interpolated_epochs": inferred_count,
        "inference_count_matches_candidate_summary": inferred_count == expected_interpolated,
        "solver_timestamp_sidecar_available": False,
        "warning": "per-key labels are diagnostic inference; aggregate exact/interpolated counts come from sealed native summaries",
    }
    return rows, report


def _local_offset_m(
    predicted: tuple[float, float], truth: tuple[float, float, float]
) -> tuple[float, float]:
    latitude, longitude = predicted
    truth_latitude, truth_longitude, _ = truth
    earth_radius = 6378137.0
    north = math.radians(latitude - truth_latitude) * earth_radius
    east = (
        math.radians(longitude - truth_longitude)
        * earth_radius
        * math.cos(math.radians(truth_latitude))
    )
    return east, north


def _speed_mps(
    previous: tuple[float, float] | None,
    current: tuple[float, float],
    previous_timestamp: int | None,
    timestamp: int,
) -> float | None:
    if previous is None or previous_timestamp is None:
        return None
    delta_ms = timestamp - previous_timestamp
    if delta_ms <= 0:
        raise Phase30Error("non-positive prediction interval")
    distance = kaggle._wgs84_horizontal_distance_m(
        previous[0], previous[1], current[0], current[1]
    )
    speed = distance / (delta_ms / 1000.0)
    if not math.isfinite(speed):
        raise Phase30Error("non-finite transition speed")
    return speed


def _summary_projection(summary: dict[str, Any]) -> dict[str, Any]:
    return {
        "dataset_id": summary.get("dataset_id"),
        "truth_used": summary.get("truth_used"),
        "epochs": summary.get("epochs"),
        "graph": summary.get("graph"),
        "gnss_first": summary.get("gnss_first"),
        "imu_initialization": summary.get("imu_initialization"),
        "android_gnss_diagnostics": summary.get("android_gnss_diagnostics"),
        "raw_utc_key_contract": summary.get("raw_utc_key_contract"),
        "tdcp_contract": summary.get("tdcp_contract"),
        "output_contract": summary.get("output_contract"),
        "production_default_changed": summary.get("production_default_changed"),
    }


def _contiguous_runs(
    rows: list[dict[str, Any]], threshold: float, metric: str
) -> list[dict[str, Any]]:
    runs: list[list[dict[str, Any]]] = []
    current: list[dict[str, Any]] = []
    for row in rows:
        if max(row["control"][metric], row["candidate"][metric]) >= threshold:
            current.append(row)
        elif current:
            runs.append(current)
            current = []
    if current:
        runs.append(current)
    reports: list[dict[str, Any]] = []
    for run in runs:
        all_rows = run
        speeds = [
            value
            for row in all_rows
            for value in (
                row["control"].get("transition_speed_mps"),
                row["candidate"].get("transition_speed_mps"),
            )
            if value is not None
        ]
        raw_gaps = [row["raw"].get("raw_gap_ms") for row in all_rows if row["raw"].get("raw_gap_ms") is not None]
        reports.append(
            {
                "start_utc_ms": all_rows[0]["timestamp"],
                "end_utc_ms": all_rows[-1]["timestamp"],
                "epoch_count": len(all_rows),
                "start_index": all_rows[0]["index"],
                "end_index": all_rows[-1]["index"],
                "control_max_horizontal_error_m": max(row["control"]["horizontal_error_m"] for row in all_rows),
                "candidate_max_horizontal_error_m": max(row["candidate"]["horizontal_error_m"] for row in all_rows),
                "control_mean_horizontal_error_m": sum(row["control"]["horizontal_error_m"] for row in all_rows) / len(all_rows),
                "candidate_mean_horizontal_error_m": sum(row["candidate"]["horizontal_error_m"] for row in all_rows) / len(all_rows),
                "max_transition_speed_mps": max(speeds, default=0.0),
                "over_70_mps_count": sum(1 for value in speeds if value > MAX_CONTINUITY_SPEED_MPS),
                "raw_clock_segments": sorted({row["raw"]["raw_clock_segment"] for row in all_rows}),
                "hcdc_values": sorted({row["raw"]["hcdc"] for row in all_rows}),
                "raw_gap_max_ms": max(raw_gaps, default=0.0),
                "source_lane_counts": {
                    lane: sum(1 for row in all_rows if row["source_lane"] == lane)
                    for lane in ("inferred-interpolated", "exact-by-sealed-alignment-count")
                },
                "candidate_spike_rows": [
                    row["timestamp"]
                    for row in all_rows
                    if (row["candidate"].get("transition_speed_mps") or 0.0) > MAX_CONTINUITY_SPEED_MPS
                ],
                "control_spike_rows": [
                    row["timestamp"]
                    for row in all_rows
                    if (row["control"].get("transition_speed_mps") or 0.0) > MAX_CONTINUITY_SPEED_MPS
                ],
            }
        )
    return reports


def _verify_freeze() -> dict[str, Any]:
    freeze = load_json(FREEZE, "Phase30 freeze")
    if freeze.get("schema_version") != FREEZE_SCHEMA:
        raise Phase30Error("Phase30 freeze schema mismatch")
    if freeze.get("status") != "frozen-before-phase30-samsung-truth-reopen":
        raise Phase30Error("Phase30 freeze is not pre-truth-reopen")
    if freeze.get("dataset_id") != ROUTE:
        raise Phase30Error("Phase30 dataset identity mismatch")
    parent = freeze.get("parent_phase29_result", {})
    if parent.get("path") != str(PHASE29_RESULT.relative_to(ROOT)) or parent.get("sha256") != PHASE29_RESULT_SHA256:
        raise Phase30Error("Phase29 result pin mismatch")
    if sha256(PHASE29_RESULT) != PHASE29_RESULT_SHA256:
        raise Phase30Error("Phase29 result hash changed")
    archive = freeze.get("archive", {})
    if archive.get("sha256") != ARCHIVE_SHA256:
        raise Phase30Error("archive hash mismatch")
    truth = freeze.get("truth_contract", {})
    if truth.get("sha256") != TRUTH_SHA256 or truth.get("bytes") != TRUTH_BYTES:
        raise Phase30Error("Samsung truth hash/size pin mismatch")
    if truth.get("phase29_cumulative_open_count_before_phase30") != 1 or truth.get("phase30_authorized_additional_open_count") != 1:
        raise Phase30Error("Phase30 cumulative truth-read contract mismatch")
    exclusions = freeze.get("exclusions", {})
    for key in (
        "pixel_truth_opened",
        "fresh_validation_truth_opened",
        "future_holdout_truth_opened",
        "test_truth_opened",
        "mat_read_or_generated",
        "token_access",
        "kaggle_or_external_mutation",
    ):
        if exclusions.get(key) is not False:
            raise Phase30Error(f"Phase30 exclusion is not closed: {key}")
    source_hashes = freeze.get("source_hashes", {})
    for relative, expected in source_hashes.items():
        if not isinstance(relative, str) or Path(relative).suffix.lower() == ".mat":
            raise Phase30Error("Phase30 source pin contains forbidden MAT")
        if sha256(ROOT / relative) != expected:
            raise Phase30Error(f"Phase30 source hash mismatch: {relative}")
    artifacts = freeze.get("sealed_artifacts", {})
    if not isinstance(artifacts, dict):
        raise Phase30Error("Phase30 sealed artifacts missing")
    for label, artifact in artifacts.items():
        if not isinstance(artifact, dict):
            raise Phase30Error(f"invalid Phase30 artifact: {label}")
        for field in ("path", "sha256"):
            if not isinstance(artifact.get(field), str):
                raise Phase30Error(f"Phase30 artifact field missing: {label}/{field}")
        path = ROOT / artifact["path"]
        if path.suffix.lower() == ".mat" or sha256(path) != artifact["sha256"]:
            raise Phase30Error(f"Phase30 artifact hash mismatch: {label}")
    raw = freeze.get("raw_inputs", {})
    for label, artifact in raw.items():
        if not isinstance(artifact, dict) or Path(str(artifact.get("path", ""))).suffix.lower() == ".mat":
            raise Phase30Error(f"invalid Phase30 raw input: {label}")
        if artifact.get("read_by_phase30", True) is False:
            continue
        path = ROOT / str(artifact.get("path"))
        if sha256(path) != artifact.get("sha256"):
            raise Phase30Error(f"Phase30 raw input hash mismatch: {label}")
    manifest = load_json(FREEZE_MANIFEST, "Phase30 freeze manifest")
    if manifest.get("schema_version") != "smartphone-r5-phase30-samsung-chronology-freeze-manifest.v1":
        raise Phase30Error("Phase30 freeze manifest schema mismatch")
    if manifest.get("freeze_record", {}).get("path") != str(FREEZE.relative_to(ROOT)):
        raise Phase30Error("Phase30 freeze manifest path mismatch")
    if manifest.get("freeze_record", {}).get("sha256") != sha256(FREEZE):
        raise Phase30Error("Phase30 freeze manifest does not pin freeze hash")
    return freeze


def _score(freeze: dict[str, Any], output_root: Path) -> dict[str, Any]:
    artifacts = freeze["sealed_artifacts"]
    control_submission = ROOT / artifacts["control_submission"]["path"]
    candidate_submission = ROOT / artifacts["candidate_submission"]["path"]
    control_summary = load_json(ROOT / artifacts["control_summary"]["path"], "control summary")
    candidate_summary = load_json(ROOT / artifacts["candidate_summary"]["path"], "candidate summary")
    expected_phone = ROUTE
    control = _load_prediction(control_submission, expected_phone)
    candidate = _load_prediction(candidate_submission, expected_phone)
    if set(control) != set(candidate):
        raise Phase30Error("control/candidate key sets differ")
    raw_path = ROOT / freeze["raw_inputs"]["device_gnss"]["path"]
    raw_epochs, raw_structural = _read_raw_epochs(raw_path)
    raw_by_timestamp = {epoch["timestamp"]: epoch for epoch in raw_epochs}
    source_rows, source_report = _source_lane_inference(
        control, candidate, control_summary, candidate_summary
    )
    source_by_timestamp = {row["timestamp"]: row for row in source_rows}

    # This is the one and only Phase30 truth read.  The hash is computed from
    # the same bytes, so no post-read hash verification reopens the file.
    truth_path = ROOT / freeze["truth_contract"]["path"]
    truth, truth_hash, truth_bytes = _read_truth_once(truth_path, expected_phone)
    if truth_hash != TRUTH_SHA256 or truth_bytes != TRUTH_BYTES:
        raise Phase30Error("Phase30 truth changed since the Phase29 materialization")

    timestamps = sorted(set(control) & set(candidate) & set(truth))
    if not timestamps:
        raise Phase30Error("Phase30 truth/prediction intersection is empty")
    previous_control: tuple[float, float] | None = None
    previous_candidate: tuple[float, float] | None = None
    previous_truth: tuple[float, float, float] | None = None
    previous_timestamp: int | None = None
    chronology: list[dict[str, Any]] = []
    for index, timestamp in enumerate(timestamps):
        if timestamp not in raw_by_timestamp:
            raise Phase30Error(f"raw structural epoch missing for {timestamp}")
        truth_point = truth[timestamp]
        control_point = control[timestamp]
        candidate_point = candidate[timestamp]
        control_horizontal = kaggle._wgs84_horizontal_distance_m(
            control_point[0], control_point[1], truth_point[0], truth_point[1]
        )
        candidate_horizontal = kaggle._wgs84_horizontal_distance_m(
            candidate_point[0], candidate_point[1], truth_point[0], truth_point[1]
        )
        control_east, control_north = _local_offset_m(control_point, truth_point)
        candidate_east, candidate_north = _local_offset_m(candidate_point, truth_point)
        control_speed = _speed_mps(
            previous_control, control_point, previous_timestamp, timestamp
        )
        candidate_speed = _speed_mps(
            previous_candidate, candidate_point, previous_timestamp, timestamp
        )
        truth_speed = (
            _speed_mps(
                (previous_truth[0], previous_truth[1]) if previous_truth else None,
                (truth_point[0], truth_point[1]),
                previous_timestamp,
                timestamp,
            )
            if previous_truth is not None
            else None
        )
        raw = raw_by_timestamp[timestamp]
        source = source_by_timestamp.get(timestamp, {})
        chronology.append(
            {
                "index": index,
                "timestamp": timestamp,
                "source_lane": source.get("source_lane", "not-in-prediction-intersection"),
                "source_lane_basis": source.get("source_lane_basis"),
                "control": {
                    "horizontal_error_m": control_horizontal,
                    "east_error_m": control_east,
                    "north_error_m": control_north,
                    "transition_speed_mps": control_speed,
                    "over_70_mps": control_speed is not None and control_speed > MAX_CONTINUITY_SPEED_MPS,
                },
                "candidate": {
                    "horizontal_error_m": candidate_horizontal,
                    "east_error_m": candidate_east,
                    "north_error_m": candidate_north,
                    "transition_speed_mps": candidate_speed,
                    "over_70_mps": candidate_speed is not None and candidate_speed > MAX_CONTINUITY_SPEED_MPS,
                },
                "truth_reference": {"transition_speed_mps": truth_speed},
                "raw": {
                    "raw_rows": raw["raw_rows"],
                    "finite_doppler_rows": raw["finite_doppler_rows"],
                    "finite_carrier_state_rows": raw["finite_carrier_state_rows"],
                    "finite_cno_rows": raw["finite_cno_rows"],
                    "state_nonzero_rows": raw["state_nonzero_rows"],
                    "hcdc": raw["hcdc"],
                    "hcdc_values": raw["hcdc_values"],
                    "hcdc_consistent": raw["hcdc_consistent"],
                    "raw_clock_segment": raw["raw_clock_segment"],
                    "raw_gap_ms": raw["raw_gap_ms"],
                    "time_nanos_gap_ms": raw["time_nanos_gap_ms"],
                },
                "source_linearity": {
                    "control_ecef_error_m": source.get("control_ecef_linearity_error_m"),
                    "candidate_ecef_error_m": source.get("candidate_ecef_linearity_error_m"),
                },
            }
        )
        previous_control = control_point
        previous_candidate = candidate_point
        previous_truth = truth_point
        previous_timestamp = timestamp

    tail_runs = _contiguous_runs(chronology, TAIL_ERROR_THRESHOLD_M, "horizontal_error_m")
    large_runs = _contiguous_runs(chronology, LARGE_ERROR_THRESHOLD_M, "horizontal_error_m")
    all_speeds = [
        value
        for row in chronology
        for value in (
            row["control"]["transition_speed_mps"],
            row["candidate"]["transition_speed_mps"],
        )
        if value is not None
    ]
    tail_boundaries = set(raw_structural["clock_segment_boundaries_utc_ms"])
    for run in tail_runs:
        run["overlaps_raw_clock_boundary"] = any(
            run["start_utc_ms"] <= boundary <= run["end_utc_ms"]
            for boundary in tail_boundaries
        )
        run["starts_within_first_30_target_epochs"] = run["start_index"] < 30
        run["interpretation"] = (
            "tail overlaps a raw clock segment boundary"
            if run["overlaps_raw_clock_boundary"]
            else "tail has no raw HCDC/clock-segment boundary; inspect graph/source continuity"
        )

    result = {
        "schema_version": SCHEMA,
        "phase": 30,
        "status": "sealed-samsung-development-chronology",
        "dataset_id": ROUTE,
        "diagnostic_purpose": "locate the Phase29 Samsung horizontal tail using truth-linked errors and truth-free structural signals; no candidate selection or solver tuning",
        "freeze": {
            "path": str(FREEZE.relative_to(ROOT)),
            "sha256": sha256(FREEZE),
            "manifest_path": str(FREEZE_MANIFEST.relative_to(ROOT)),
            "manifest_sha256": sha256(FREEZE_MANIFEST),
            "parent_phase29_result_sha256": PHASE29_RESULT_SHA256,
        },
        "truth_access": {
            "phase29_prior_cumulative_open_count": 1,
            "phase30_additional_truth_read_count": 1,
            "cumulative_open_count_after_phase30": 2,
            "truth_path": str(truth_path.relative_to(ROOT)),
            "truth_sha256_from_single_read": truth_hash,
            "truth_bytes_from_single_read": truth_bytes,
            "pixel_truth_opened": False,
            "fresh_validation_truth_opened": False,
            "future_holdout_truth_opened": False,
            "test_truth_opened": False,
            "mat_read_or_generated": False,
            "token_access": False,
            "kaggle_or_external_mutation": False,
        },
        "sealed_lanes": {
            "control": {
                "id": "phase25_raw_clock_no_bridge_control",
                "submission_sha256": artifacts["control_submission"]["sha256"],
                "summary_sha256": artifacts["control_summary"]["sha256"],
            },
            "candidate": {
                "id": "phase28_tdcp_no_bridge",
                "submission_sha256": artifacts["candidate_submission"]["sha256"],
                "summary_sha256": artifacts["candidate_summary"]["sha256"],
            },
            "solver_rerun": False,
            "candidate_or_control_changed": False,
        },
        "structural_signals": {
            "raw_gnss_input_sha256": freeze["raw_inputs"]["device_gnss"]["sha256"],
            **raw_structural,
            "fgo_epoch_coverage": {
                "control": _summary_projection(control_summary),
                "candidate": _summary_projection(candidate_summary),
            },
            "source_lane_inference": source_report,
        },
        "chronology": chronology,
        "tail_thresholds": {
            "tail_error_threshold_m": TAIL_ERROR_THRESHOLD_M,
            "large_error_threshold_m": LARGE_ERROR_THRESHOLD_M,
            "continuity_bound_mps": MAX_CONTINUITY_SPEED_MPS,
            "source_linearity_tolerance_m": INTERPOLATION_INFERENCE_TOLERANCE_M,
        },
        "runs": {
            "tail_over_500m": tail_runs,
            "large_over_100m": large_runs,
        },
        "summary": {
            "truth_rows": len(truth),
            "prediction_rows_control": len(control),
            "prediction_rows_candidate": len(candidate),
            "shared_truth_prediction_rows": len(chronology),
            "candidate_max_horizontal_error_m": max(
                row["candidate"]["horizontal_error_m"] for row in chronology
            ),
            "control_max_horizontal_error_m": max(
                row["control"]["horizontal_error_m"] for row in chronology
            ),
            "candidate_over_70_mps_count": sum(
                1 for row in chronology if row["candidate"]["over_70_mps"]
            ),
            "control_over_70_mps_count": sum(
                1 for row in chronology if row["control"]["over_70_mps"]
            ),
            "candidate_mean_horizontal_error_m": sum(
                row["candidate"]["horizontal_error_m"] for row in chronology
            )
            / len(chronology),
            "control_mean_horizontal_error_m": sum(
                row["control"]["horizontal_error_m"] for row in chronology
            )
            / len(chronology),
            "all_output_keys_have_raw_structural_signal": True,
            "all_truth_reads_in_this_process": 1,
            "overall_interpretation": (
                "This is an evidence record only; it does not authorize a new lane or alter Phase29's No-Go."
            ),
        },
        "policy": {
            "truth_used_for_inference": False,
            "truth_used_for_candidate_selection": False,
            "truth_used_only_for_postfreeze_chronology": True,
            "post_score_tuning": False,
            "validation_or_holdout_opened": False,
        },
    }
    output_root.mkdir(parents=True, exist_ok=True)
    result_path = output_root / "samsung_chronology.json"
    atomic_json(result_path, result)
    manifest = {
        "schema_version": MANIFEST_SCHEMA,
        "result": {
            "path": str(result_path.relative_to(ROOT)),
            "sha256": sha256(result_path),
        },
        "truth_read_count_in_process": 1,
        "truth_hash_from_same_read": truth_hash,
        "candidate_and_control_artifacts_reused": True,
        "candidate_or_control_regenerated": False,
        "mat_read_or_generated": False,
        "validation_or_holdout_opened": False,
        "atomic_publish": True,
    }
    atomic_json(output_root / "samsung_chronology.manifest.json", manifest)
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("operation", choices=("score",))
    parser.add_argument("--output-root", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()
    try:
        freeze = _verify_freeze()
        result = _score(freeze, args.output_root)
    except Phase30Error as exc:
        print(f"phase30: {exc}", file=sys.stderr)
        return 2
    print(
        json.dumps(
            {
                "status": result["status"],
                "dataset_id": ROUTE,
                "truth_read_count": result["truth_access"]["phase30_additional_truth_read_count"],
                "tail_run_count": len(result["runs"]["tail_over_500m"]),
                "candidate_max_horizontal_error_m": result["summary"]["candidate_max_horizontal_error_m"],
                "control_max_horizontal_error_m": result["summary"]["control_max_horizontal_error_m"],
            },
            sort_keys=True,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
