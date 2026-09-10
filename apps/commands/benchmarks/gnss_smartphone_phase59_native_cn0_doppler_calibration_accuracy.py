#!/usr/bin/env python3
"""One-shot Phase59 accuracy score for the sealed Phase58 C/N0 candidate.

Phase58 already sealed the native candidate and its flag-off control.  This
evaluator only reads those immutable run-1 submission/summary artifacts and
the four already-materialized Phase44 development truths.  It never invokes a
native binary, opens raw/navigation data, reopens an archive, or uses a
previous score as an estimator input.

``--verify-freeze`` performs no truth read.  ``score`` verifies every sealed
contract first, then reads each truth file exactly once in this process and
scores candidate/control from in-memory bytes.  The prediction-domain gate is
the exact Phase52-recovered contract: every prediction key must match truth;
truth-row coverage is informational and the two pinned leading raw-UTC
warm-up truth rows remain unmatched.
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
import sys
import tempfile
import time
from typing import Any


ROOT = Path(__file__).resolve().parents[3]
FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_phase59_native_cn0_doppler_calibration_accuracy_freeze_v1.json"
FREEZE_SHA256 = "fbd1a3076da1b5283b08d746d334837fce6eb17cdb45f3d484248e6b6f09bec6"
EVALUATOR_MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase59_native_cn0_doppler_calibration_accuracy_evaluator_manifest_v1.json"
DEFAULT_OUTPUT = ROOT / "output/smartphone-r5/phase59-native-cn0-doppler-calibration-accuracy-v1"
PHASE58_RESULT_RECORD = ROOT / "docs/use_cases/records/smartphone_r5_phase58_native_cn0_doppler_calibration_structural_result_v1.json"
PHASE58_OUTPUT_RESULT = ROOT / "output/smartphone-r5/phase58-native-cn0-doppler-calibration-v1/phase58_native_cn0_doppler_calibration_structural_result.json"
PHASE58_OUTPUT_MANIFEST = ROOT / "output/smartphone-r5/phase58-native-cn0-doppler-calibration-v1/phase58_native_cn0_doppler_calibration_structural_manifest.json"

ROUTES = (
    "2021-03-16-18-59-us-ca-mtv-a/pixel5",
    "2021-08-24-20-32-us-ca-mtv-h/pixel5",
    "2022-04-01-18-22-us-ca-lax-t/pixel5",
    "2023-03-08-21-34-us-ca-mtv-u/pixel5",
)
TARGET = ROUTES[1]
SUBMISSION_FIELDS = ("phone", "UnixTimeMillis", "LatitudeDegrees", "LongitudeDegrees")
TRUTH_FIELDS = ("UnixTimeMillis", "LatitudeDegrees", "LongitudeDegrees")
HAVERSINE_RADIUS_M = 6_371_008.8
MAX_SPEED_MPS = 70.0
STRETCH_TARGET_M = 0.782
TOLERANCE = 1e-12
ALPHA = 0.7586783350728457
REFERENCE_CN0 = 40.0


class Phase59Error(ValueError):
    """Raised when the immutable Phase59 contract cannot be proved."""


def _fail(message: str) -> Phase59Error:
    return Phase59Error(message)


def _reject_path(path: Path | str) -> None:
    token = str(path).lower()
    forbidden = (".mat", "validation", "holdout", "precomputed", "device_wls", "kaggle", "token")
    if any(item in token for item in forbidden):
        raise _fail(f"forbidden input/output path: {path}")


def _read_once(path: Path, label: str) -> tuple[bytes, str]:
    """Read one immutable file once and hash the bytes read from that handle."""

    _reject_path(path)
    if not path.is_file():
        raise _fail(f"missing {label}: {path}")
    try:
        with path.open("rb") as handle:
            payload = handle.read()
    except OSError as exc:
        raise _fail(f"failed to read {label}: {path}: {exc}") from exc
    return payload, hashlib.sha256(payload).hexdigest()


def _hash_path(path: Path, label: str) -> str:
    _reject_path(path)
    if not path.is_file():
        raise _fail(f"missing {label}: {path}")
    digest = hashlib.sha256()
    try:
        with path.open("rb") as handle:
            for chunk in iter(lambda: handle.read(1024 * 1024), b""):
                digest.update(chunk)
    except OSError as exc:
        raise _fail(f"failed to hash {label}: {path}: {exc}") from exc
    return digest.hexdigest()


def _json_bytes(payload: bytes, label: str) -> dict[str, Any]:
    try:
        value = json.loads(payload.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise _fail(f"invalid {label}") from exc
    if not isinstance(value, dict):
        raise _fail(f"{label} must be a JSON object")
    return value


def _load_json_once(path: Path, label: str) -> tuple[dict[str, Any], str, int]:
    payload, digest = _read_once(path, label)
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


def _atomic_json(path: Path, value: dict[str, Any]) -> None:
    _atomic_write(path, (json.dumps(value, indent=2, sort_keys=True) + "\n").encode("utf-8"))


def _relative(path: Path) -> str:
    try:
        return str(path.relative_to(ROOT))
    except ValueError:
        return str(path)


def _route_phone(route: str) -> str:
    if "/" not in route or route.endswith("/"):
        raise _fail(f"invalid route identity: {route}")
    return route


def _finite_float(raw: str | None, field: str, row_number: int) -> float:
    try:
        value = float((raw or "").strip())
    except ValueError as exc:
        raise _fail(f"row {row_number}: invalid {field}") from exc
    if not math.isfinite(value):
        raise _fail(f"row {row_number}: non-finite {field}")
    return value


def _parse_submission(payload: bytes, expected_phone: str, label: str) -> list[tuple[str, int, float, float]]:
    try:
        text = payload.decode("utf-8-sig")
    except UnicodeDecodeError as exc:
        raise _fail(f"{label} is not UTF-8 CSV") from exc
    reader = csv.DictReader(io.StringIO(text, newline=""))
    if tuple(reader.fieldnames or ()) != SUBMISSION_FIELDS:
        raise _fail(f"{label} header must be exactly {','.join(SUBMISSION_FIELDS)}")
    rows: list[tuple[str, int, float, float]] = []
    seen: set[tuple[str, int]] = set()
    previous: int | None = None
    for row_number, raw in enumerate(reader, start=2):
        if None in raw:
            raise _fail(f"{label} row {row_number}: extra columns")
        phone = raw.get("phone") or ""
        if phone != expected_phone or phone != phone.strip():
            raise _fail(f"{label} row {row_number}: phone mismatch")
        try:
            timestamp = int((raw.get("UnixTimeMillis") or "").strip())
        except ValueError as exc:
            raise _fail(f"{label} row {row_number}: invalid UnixTimeMillis") from exc
        if timestamp < 0 or (previous is not None and timestamp <= previous):
            raise _fail(f"{label} row {row_number}: timestamps are not strictly increasing")
        latitude = _finite_float(raw.get("LatitudeDegrees"), "LatitudeDegrees", row_number)
        longitude = _finite_float(raw.get("LongitudeDegrees"), "LongitudeDegrees", row_number)
        if not -90.0 <= latitude <= 90.0 or not -180.0 <= longitude <= 180.0:
            raise _fail(f"{label} row {row_number}: coordinate out of range")
        key = (phone, timestamp)
        if key in seen:
            raise _fail(f"{label} row {row_number}: duplicate prediction key")
        seen.add(key)
        rows.append((phone, timestamp, latitude, longitude))
        previous = timestamp
    if not rows:
        raise _fail(f"{label} is empty")
    return rows


def _parse_truth(payload: bytes, expected_phone: str, label: str) -> dict[tuple[str, int], tuple[float, float]]:
    try:
        text = payload.decode("utf-8-sig")
    except UnicodeDecodeError as exc:
        raise _fail(f"{label} is not UTF-8 CSV") from exc
    reader = csv.DictReader(io.StringIO(text, newline=""))
    fields = list(reader.fieldnames or ())
    missing = [field for field in TRUTH_FIELDS if field not in fields]
    if missing:
        raise _fail(f"{label} missing fields: {missing}")
    has_phone = "phone" in fields
    result: dict[tuple[str, int], tuple[float, float]] = {}
    for row_number, raw in enumerate(reader, start=2):
        if None in raw:
            raise _fail(f"{label} row {row_number}: extra columns")
        phone = (raw.get("phone") or expected_phone) if has_phone else expected_phone
        if phone != expected_phone or phone != phone.strip():
            raise _fail(f"{label} row {row_number}: phone mismatch")
        try:
            timestamp = int((raw.get("UnixTimeMillis") or "").strip())
        except ValueError as exc:
            raise _fail(f"{label} row {row_number}: invalid UnixTimeMillis") from exc
        if timestamp < 0:
            raise _fail(f"{label} row {row_number}: negative UnixTimeMillis")
        latitude = _finite_float(raw.get("LatitudeDegrees"), "LatitudeDegrees", row_number)
        longitude = _finite_float(raw.get("LongitudeDegrees"), "LongitudeDegrees", row_number)
        if not -90.0 <= latitude <= 90.0 or not -180.0 <= longitude <= 180.0:
            raise _fail(f"{label} row {row_number}: coordinate out of range")
        key = (phone, timestamp)
        if key in result:
            raise _fail(f"{label} row {row_number}: duplicate truth key")
        result[key] = (latitude, longitude)
    if not result:
        raise _fail(f"{label} is empty")
    return result


def _haversine(latitude_a: float, longitude_a: float, latitude_b: float, longitude_b: float) -> float:
    latitude_a_rad = math.radians(latitude_a)
    latitude_b_rad = math.radians(latitude_b)
    delta_latitude = math.radians(latitude_b - latitude_a)
    delta_longitude = math.radians(longitude_b - longitude_a)
    value = (
        math.sin(delta_latitude / 2.0) ** 2
        + math.cos(latitude_a_rad) * math.cos(latitude_b_rad) * math.sin(delta_longitude / 2.0) ** 2
    )
    distance = 2.0 * HAVERSINE_RADIUS_M * math.asin(math.sqrt(min(1.0, max(0.0, value))))
    if not math.isfinite(distance) or distance < 0.0:
        raise _fail("Haversine distance is non-finite")
    return distance


def _percentile(values: list[float], fraction: float) -> float:
    if not values:
        raise _fail("cannot calculate percentile for an empty distribution")
    ordered = sorted(values)
    rank = (len(ordered) - 1) * fraction
    lower = math.floor(rank)
    upper = math.ceil(rank)
    if lower == upper:
        return ordered[lower]
    weight = rank - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def _continuity(rows: list[tuple[str, int, float, float]]) -> dict[str, Any]:
    speeds: list[float] = []
    for previous, current in zip(rows, rows[1:]):
        delta_ms = current[1] - previous[1]
        if delta_ms <= 0:
            raise _fail("prediction timestamps are not strictly increasing")
        speed = _haversine(previous[2], previous[3], current[2], current[3]) / (delta_ms / 1000.0)
        if not math.isfinite(speed):
            raise _fail("prediction speed is non-finite")
        speeds.append(speed)
    return {
        "transition_count": len(speeds),
        "max_speed_mps": max(speeds) if speeds else 0.0,
        "over_70_mps_count": sum(speed > MAX_SPEED_MPS for speed in speeds),
    }


def _metrics(
    rows: list[tuple[str, int, float, float]],
    truth: dict[tuple[str, int], tuple[float, float]],
    expected_missing_rows: int,
    expected_missing_key: list[Any] | None,
    label: str,
) -> dict[str, Any]:
    prediction = {(phone, timestamp): (latitude, longitude) for phone, timestamp, latitude, longitude in rows}
    prediction_keys = set(prediction)
    truth_keys = set(truth)
    matched = sorted(prediction_keys & truth_keys, key=lambda key: key[1])
    extra = sorted(prediction_keys - truth_keys, key=lambda key: key[1])
    missing = sorted(truth_keys - prediction_keys, key=lambda key: key[1])
    if len(extra) != 0:
        raise _fail(f"{label}: extra prediction keys are forbidden: {extra[:3]}")
    if len(missing) != expected_missing_rows:
        raise _fail(f"{label}: unexpected missing truth count {len(missing)} != {expected_missing_rows}")
    if expected_missing_key is None:
        if missing:
            raise _fail(f"{label}: unexpected missing truth key {missing[0]}")
    else:
        if len(expected_missing_key) != 2:
            raise _fail(f"{label}: malformed frozen missing key")
        expected_key = (str(expected_missing_key[0]), int(expected_missing_key[1]))
        if missing != [expected_key]:
            raise _fail(f"{label}: missing key does not match frozen warm-up key")
        if expected_key != min(truth_keys, key=lambda key: key[1]):
            raise _fail(f"{label}: frozen missing key is not the leading truth timestamp")
        if expected_key[1] >= min(key[1] for key in prediction_keys):
            raise _fail(f"{label}: warm-up truth key is not before prediction domain")
    if not matched:
        raise _fail(f"{label}: no exact truth-matched prediction keys")
    errors = [_haversine(*prediction[key], *truth[key]) for key in matched]
    p50 = _percentile(errors, 0.50)
    p95 = _percentile(errors, 0.95)
    continuity = _continuity(rows)
    result = {
        "finite": all(math.isfinite(value) for value in errors),
        "prediction_rows": len(rows),
        "truth_rows": len(truth),
        "matched_rows": len(matched),
        "missing_truth_rows": len(missing),
        "extra_prediction_rows": len(extra),
        "prediction_domain_coverage": len(matched) / len(prediction_keys),
        "truth_row_coverage": len(matched) / len(truth_keys),
        "mean_m": sum(errors) / len(errors),
        "p50_m": p50,
        "p95_m": p95,
        "max_m": max(errors),
        "score_m": (p50 + p95) / 2.0,
        "over_70_mps_count": continuity["over_70_mps_count"],
        "max_speed_mps": continuity["max_speed_mps"],
        "continuity": continuity,
        "matched_key_count": len(matched),
        "missing_truth_keys": [[key[0], key[1]] for key in missing],
    }
    if not all(math.isfinite(float(result[key])) for key in ("mean_m", "p50_m", "p95_m", "max_m", "score_m", "prediction_domain_coverage", "truth_row_coverage")):
        raise _fail(f"{label}: non-finite score metric")
    return result


def _verify_freeze() -> dict[str, Any]:
    freeze, digest, _ = _load_json_once(FREEZE, "Phase59 freeze")
    if digest != FREEZE_SHA256:
        raise _fail("Phase59 freeze hash changed")
    if freeze.get("schema_version") != "smartphone-r5-phase59-native-cn0-doppler-calibration-accuracy-freeze.v1":
        raise _fail("Phase59 freeze schema changed")
    if freeze.get("status") != "frozen-before-phase59-truth-read":
        raise _fail("Phase59 freeze is not pre-truth")
    cohort = freeze.get("cohort", {})
    if tuple(cohort.get("route_order", ())) != ROUTES:
        raise _fail("Phase59 route order changed")
    candidate = freeze.get("candidate", {})
    if candidate.get("alpha_mps_at_reference") != ALPHA or candidate.get("reference_cn0_dbhz") != REFERENCE_CN0:
        raise _fail("Phase59 candidate alpha/reference changed")
    if candidate.get("flag") != "--native-cn0-doppler-calibration" or candidate.get("default_enabled") is not False:
        raise _fail("Phase59 candidate flag/default contract changed")
    gates = freeze.get("accuracy_gates", {})
    required_gates = {
        "candidate_improves_each_route_by_at_least_m": 0.05,
        "candidate_macro_improvement_at_least_m": 0.1,
        "candidate_mtv_h_improvement_at_least_m": 0.1,
        "candidate_prediction_domain_coverage": 1.0,
        "candidate_macro_score_max_m": 2.0,
        "candidate_route_score_max_m": 3.0,
        "candidate_mtv_h_p95_max_m": 5.0,
        "candidate_over_70_mps_count": 0,
    }
    for key, expected in required_gates.items():
        if gates.get(key) != expected:
            raise _fail(f"Phase59 accuracy gate changed: {key}")
    if gates.get("candidate_no_route_regression") is not True or gates.get("declared_before_truth") is not True:
        raise _fail("Phase59 route/gate declaration changed")
    policy = freeze.get("read_policy", {})
    if policy.get("truth_reads_before_manifest") != 0 or policy.get("truth_reads_after_manifest") != 4 or policy.get("truth_reads_per_route") != 1:
        raise _fail("Phase59 truth read accounting changed")
    if policy.get("solver_or_native_subprocess") is not False or policy.get("archive_reopen_or_rematerialize") is not False:
        raise _fail("Phase59 forbidden process/archive policy changed")
    if policy.get("validation_holdout_reads") != 0 or policy.get("mat_reads_or_generated") != 0 or policy.get("kaggle_or_token_access") != 0:
        raise _fail("Phase59 forbidden evaluation input policy changed")
    historical = freeze.get("historical_reference_not_scoring_input", {})
    if historical.get("scoring_input") is not False or historical.get("selection_or_tuning_input") is not False:
        raise _fail("Phase51/52 historical reference became scoring input")
    for route in ROUTES:
        row = cohort.get("routes", {}).get(route)
        if not isinstance(row, dict):
            raise _fail(f"Phase59 route metadata missing: {route}")
        truth = row.get("truth", {})
        if int(truth.get("expected_missing_truth_rows", -1)) not in (0, 1):
            raise _fail(f"Phase59 warm-up contract missing: {route}")
        for lane in ("candidate", "control"):
            lane_meta = row.get(lane, {})
            if not isinstance(lane_meta, dict) or not isinstance(lane_meta.get("submission"), dict) or not isinstance(lane_meta.get("summary"), dict):
                raise _fail(f"Phase59 sealed {lane} metadata missing: {route}")
    return freeze


def _verify_manifest() -> dict[str, Any]:
    manifest, digest, _ = _load_json_once(EVALUATOR_MANIFEST, "Phase59 evaluator manifest")
    if manifest.get("schema_version") != "smartphone-r5-phase59-native-cn0-doppler-calibration-accuracy-evaluator-manifest.v1":
        raise _fail("Phase59 evaluator manifest schema changed")
    if manifest.get("status") != "evaluator-sealed-before-phase59-truth":
        raise _fail("Phase59 evaluator manifest is not pre-truth")
    freeze_pin = manifest.get("freeze", {})
    if freeze_pin.get("path") != _relative(FREEZE) or freeze_pin.get("sha256") != FREEZE_SHA256:
        raise _fail("Phase59 evaluator manifest freeze pin changed")
    evaluator = manifest.get("evaluator", {})
    source = evaluator.get("source", {})
    test = evaluator.get("focused_test", {})
    cmake = evaluator.get("cmake", {})
    if source.get("path") != _relative(Path(__file__)):
        raise _fail("Phase59 evaluator source path pin changed")
    if _hash_path(Path(__file__), "Phase59 evaluator source") != source.get("sha256"):
        raise _fail("Phase59 evaluator source hash changed")
    test_path = ROOT / str(test.get("path", ""))
    if _hash_path(test_path, "Phase59 focused test") != test.get("sha256"):
        raise _fail("Phase59 focused test hash changed")
    cmake_path = ROOT / str(cmake.get("path", ""))
    if _hash_path(cmake_path, "Phase59 CMake test list") != cmake.get("sha256"):
        raise _fail("Phase59 CMake test list hash changed")
    if manifest.get("truth_policy", {}).get("truth_reads_before_manifest") != 0:
        raise _fail("Phase59 manifest was not sealed before truth")
    if manifest.get("truth_policy", {}).get("solver_or_native_subprocess") is not False:
        raise _fail("Phase59 manifest permits native process")
    manifest["_sha256"] = digest
    return manifest


def _verify_phase58_seals(freeze: dict[str, Any]) -> None:
    authority = freeze.get("authority", {})
    phase58_record = authority.get("phase58_structural_result_record", {})
    if _hash_path(PHASE58_RESULT_RECORD, "Phase58 structural result record") != phase58_record.get("sha256"):
        raise _fail("Phase58 structural result record hash changed")
    record, _, _ = _load_json_once(PHASE58_RESULT_RECORD, "Phase58 structural result record")
    if record.get("status") != "go-cn0-doppler-calibration-structural" or record.get("truth_free") is not True:
        raise _fail("Phase58 structural result is not sealed truth-free go")
    if record.get("gates", {}).get("all_passed") is not True or record.get("read_accounting", {}).get("truth_reads") != 0:
        raise _fail("Phase58 structural result gate/read contract changed")
    phase58_output = authority.get("phase58_output", {})
    if _hash_path(PHASE58_OUTPUT_RESULT, "Phase58 structural output result") != phase58_output.get("result_sha256"):
        raise _fail("Phase58 structural output result hash changed")
    if _hash_path(PHASE58_OUTPUT_MANIFEST, "Phase58 structural output manifest") != phase58_output.get("manifest_sha256"):
        raise _fail("Phase58 structural output manifest hash changed")
    output_result, _, _ = _load_json_once(PHASE58_OUTPUT_RESULT, "Phase58 structural output result")
    output_manifest, _, _ = _load_json_once(PHASE58_OUTPUT_MANIFEST, "Phase58 structural output manifest")
    if output_result.get("gates", {}).get("all_passed") is not True or output_result.get("read_accounting", {}).get("truth_reads") != 0:
        raise _fail("Phase58 output result gate/read contract changed")
    if output_manifest.get("all_gates_passed") is not True or output_manifest.get("truth_reads") != 0:
        raise _fail("Phase58 output manifest gate/read contract changed")


def _artifact_bytes(spec: dict[str, Any], label: str) -> tuple[bytes, str]:
    path = spec.get("path")
    expected_hash = spec.get("sha256")
    if not isinstance(path, str) or not isinstance(expected_hash, str):
        raise _fail(f"invalid sealed {label} metadata")
    payload, digest = _read_once(ROOT / path, label)
    if digest != expected_hash:
        raise _fail(f"sealed {label} hash mismatch")
    if int(spec.get("bytes", len(payload))) != len(payload):
        raise _fail(f"sealed {label} byte count mismatch")
    return payload, digest


def _validate_summary(summary: dict[str, Any], route: str, candidate: bool) -> None:
    for key, expected in (
        ("dataset_id", route),
        ("truth_used", False),
        ("production_default_changed", False),
        ("native_quality_anchor", True),
        ("native_pdc_imu_tdcp_no_bridge", True),
        ("native_pdc_state_bridge", False),
    ):
        if summary.get(key) != expected:
            raise _fail(f"Phase58 {route} summary contract mismatch: {key}")
    if summary.get("graph", {}).get("converged") is not True:
        raise _fail(f"Phase58 {route} summary is not converged")
    calibration = summary.get("native_cn0_doppler_calibration")
    if candidate:
        if not isinstance(calibration, dict) or calibration.get("enabled") is not True:
            raise _fail(f"Phase58 {route} candidate calibration telemetry missing")
        if calibration.get("alpha_mps_at_reference") != ALPHA or calibration.get("reference_cn0_dbhz") != REFERENCE_CN0:
            raise _fail(f"Phase58 {route} candidate alpha/reference changed")
        if calibration.get("upper_clip") is not False or calibration.get("spp_applied") is not False or calibration.get("tdcp_applied") is not False or calibration.get("single_difference_doppler_applied") is not False:
            raise _fail(f"Phase58 {route} candidate scope changed")
    elif calibration is not None:
        raise _fail(f"Phase58 {route} control unexpectedly exposes calibration telemetry")


def _read_phase58_artifacts(freeze: dict[str, Any], accounting: dict[str, int] | None = None) -> dict[str, Any]:
    records: dict[str, Any] = {}
    for route in ROUTES:
        metadata = freeze["cohort"]["routes"][route]
        phone = _route_phone(route)
        lanes: dict[str, Any] = {}
        for lane_name, candidate in (("candidate", True), ("control", False)):
            lane = metadata[lane_name]
            submission_spec = lane["submission"]
            summary_spec = lane["summary"]
            submission_payload, submission_hash = _artifact_bytes(submission_spec, f"{lane_name} submission {route}")
            summary_payload, summary_hash = _artifact_bytes(summary_spec, f"{lane_name} summary {route}")
            if accounting is not None:
                accounting[f"{lane_name}_submission_reads"] += 1
                accounting[f"{lane_name}_summary_reads"] += 1
            if submission_hash != submission_spec["sha256"] or summary_hash != summary_spec["sha256"]:
                raise _fail(f"Phase58 {route} {lane_name} artifact hash disagreement")
            rows = _parse_submission(submission_payload, phone, f"{lane_name} submission {route}")
            if len(rows) != int(submission_spec["rows"]):
                raise _fail(f"Phase58 {route} {lane_name} prediction row count changed")
            summary = _json_bytes(summary_payload, f"{lane_name} summary {route}")
            _validate_summary(summary, route, candidate)
            lanes[lane_name] = {
                "rows": rows,
                "submission_sha256": submission_hash,
                "summary_sha256": summary_hash,
                "summary": summary,
                "submission_bytes": len(submission_payload),
                "summary_bytes": len(summary_payload),
            }
        candidate_keys = {(row[0], row[1]) for row in lanes["candidate"]["rows"]}
        control_keys = {(row[0], row[1]) for row in lanes["control"]["rows"]}
        if candidate_keys != control_keys:
            raise _fail(f"Phase58 {route} candidate/control prediction domains differ")
        if metadata["control"].get("phase43_champion_reference") is not True:
            raise _fail(f"Phase58 {route} control is not pinned as Phase43 champion reference")
        records[route] = lanes
    return records


def _route_result(
    route: str,
    candidate: dict[str, Any],
    control: dict[str, Any],
) -> dict[str, Any]:
    improvement = float(control["score_m"]) - float(candidate["score_m"])
    failures: list[str] = []
    if candidate["finite"] is not True:
        failures.append("candidate_nonfinite")
    if candidate["prediction_domain_coverage"] != 1.0:
        failures.append("prediction_domain_coverage_not_one")
    if candidate["over_70_mps_count"] != 0:
        failures.append("candidate_over_70_mps")
    if improvement < 0.05 - TOLERANCE:
        failures.append("improvement_below_0.05m")
    if improvement < -TOLERANCE:
        failures.append("route_regression")
    if candidate["score_m"] > 3.0 + TOLERANCE:
        failures.append("candidate_route_score_over_3m")
    if route == TARGET and candidate["p95_m"] > 5.0 + TOLERANCE:
        failures.append("candidate_mtv_h_p95_over_5m")
    return {
        "candidate": candidate,
        "control": control,
        "improvement_m": improvement,
        "gate": {"passed": not failures, "failures": failures},
    }


def _aggregate(metrics: list[dict[str, Any]]) -> dict[str, Any]:
    if not metrics:
        raise _fail("cannot aggregate an empty route set")
    return {
        "route_count": len(metrics),
        "macro_score_m": sum(float(item["score_m"]) for item in metrics) / len(metrics),
        "mean_mean_m": sum(float(item["mean_m"]) for item in metrics) / len(metrics),
        "mean_p50_m": sum(float(item["p50_m"]) for item in metrics) / len(metrics),
        "mean_p95_m": sum(float(item["p95_m"]) for item in metrics) / len(metrics),
        "mean_max_m": sum(float(item["max_m"]) for item in metrics) / len(metrics),
        "mean_prediction_domain_coverage": sum(float(item["prediction_domain_coverage"]) for item in metrics) / len(metrics),
        "mean_truth_row_coverage": sum(float(item["truth_row_coverage"]) for item in metrics) / len(metrics),
        "sum_over_70_mps_count": sum(int(item["over_70_mps_count"]) for item in metrics),
        "all_finite": all(item["finite"] is True for item in metrics),
        "all_prediction_domain_coverage": all(item["prediction_domain_coverage"] == 1.0 for item in metrics),
        "max_route_score_m": max(float(item["score_m"]) for item in metrics),
    }


def _truth_meta(freeze: dict[str, Any], route: str, digest: str, size: int, row_count: int) -> dict[str, Any]:
    source = freeze["cohort"]["routes"][route]["truth"]
    if digest != source["sha256"] or size != int(source["bytes"]) or row_count != int(source["rows"]):
        raise _fail(f"Phase44 truth pin changed: {route}")
    return {
        "path": source["path"],
        "sha256": digest,
        "bytes": size,
        "rows": row_count,
        "expected_missing_truth_rows": source["expected_missing_truth_rows"],
        "expected_missing_key": source["expected_missing_key"],
        "read_count": 1,
    }


def score(output_root: Path = DEFAULT_OUTPUT) -> dict[str, Any]:
    """Read four truths once and score the immutable Phase58 artifacts."""

    output_root = output_root.resolve()
    _reject_path(output_root)
    if output_root.exists() and any(output_root.iterdir()):
        raise _fail(f"refusing to rerun Phase59 evaluation: {output_root}")
    accounting = {"truth_reads": 0, "candidate_submission_reads": 0, "candidate_summary_reads": 0, "control_submission_reads": 0, "control_summary_reads": 0}
    route_results: dict[str, Any] = {}
    try:
        freeze = _verify_freeze()
        _verify_manifest()
        _verify_phase58_seals(freeze)
        artifacts = _read_phase58_artifacts(freeze, accounting)
        output_root.mkdir(parents=True, exist_ok=True)
        candidate_metrics: list[dict[str, Any]] = []
        control_metrics: list[dict[str, Any]] = []
        for route in ROUTES:
            source = freeze["cohort"]["routes"][route]["truth"]
            accounting["truth_reads"] += 1
            truth_payload, truth_digest = _read_once(ROOT / source["path"], f"Phase44 development truth {route}")
            truth = _parse_truth(truth_payload, _route_phone(route), f"Phase44 development truth {route}")
            truth_metadata = _truth_meta(freeze, route, truth_digest, len(truth_payload), len(truth))
            expected_missing_rows = int(source["expected_missing_truth_rows"])
            expected_missing_key = source["expected_missing_key"]
            candidate = _metrics(
                artifacts[route]["candidate"]["rows"],
                truth,
                expected_missing_rows,
                expected_missing_key,
                f"candidate {route}",
            )
            control = _metrics(
                artifacts[route]["control"]["rows"],
                truth,
                expected_missing_rows,
                expected_missing_key,
                f"control {route}",
            )
            candidate_metrics.append(candidate)
            control_metrics.append(control)
            route_results[route] = {
                "truth": truth_metadata,
                "candidate": candidate,
                "control": control,
                "artifact_hashes": {
                    "candidate_submission_sha256": artifacts[route]["candidate"]["submission_sha256"],
                    "candidate_summary_sha256": artifacts[route]["candidate"]["summary_sha256"],
                    "control_submission_sha256": artifacts[route]["control"]["submission_sha256"],
                    "control_summary_sha256": artifacts[route]["control"]["summary_sha256"],
                },
            }
            route_results[route].update(_route_result(route, candidate, control))
        candidate_aggregate = _aggregate(candidate_metrics)
        control_aggregate = _aggregate(control_metrics)
        macro_improvement = control_aggregate["macro_score_m"] - candidate_aggregate["macro_score_m"]
        target_improvement = route_results[TARGET]["improvement_m"]
        macro_failures: list[str] = []
        if macro_improvement < 0.1 - TOLERANCE:
            macro_failures.append("macro_improvement_below_0.10m")
        if candidate_aggregate["macro_score_m"] > 2.0 + TOLERANCE:
            macro_failures.append("candidate_macro_over_2m")
        if target_improvement < 0.1 - TOLERANCE:
            macro_failures.append("mtv_h_improvement_below_0.10m")
        if not candidate_aggregate["all_finite"]:
            macro_failures.append("candidate_nonfinite_route")
        if not candidate_aggregate["all_prediction_domain_coverage"]:
            macro_failures.append("prediction_domain_coverage_not_one")
        if candidate_aggregate["sum_over_70_mps_count"] != 0:
            macro_failures.append("candidate_over_70_mps")
        all_route_gates = all(route_results[route]["gate"]["passed"] for route in ROUTES)
        all_passed = all_route_gates and not macro_failures
        report = {
            "schema_version": "smartphone-r5-phase59-native-cn0-doppler-calibration-accuracy.v1",
            "phase": 59,
            "execution_label": "Luna Max",
            "status": "development-accuracy-pass" if all_passed else "no-go-development-accuracy-gates",
            "decision": "stop-before-fresh-validation" if all_passed else "no-go-no-validation",
            "truth_free": False,
            "freeze": {"path": _relative(FREEZE), "sha256": FREEZE_SHA256},
            "evaluator_manifest": {"path": _relative(EVALUATOR_MANIFEST), "sha256": _hash_path(EVALUATOR_MANIFEST, "Phase59 evaluator manifest")},
            "phase58_structural_result_record": freeze["authority"]["phase58_structural_result_record"],
            "phase58_output": freeze["authority"]["phase58_output"],
            "candidate": freeze["candidate"],
            "metric_contract": freeze["phase44_metric_contract"],
            "routes": route_results,
            "aggregate": {
                "candidate": candidate_aggregate,
                "control": control_aggregate,
                "candidate_macro_score_m": candidate_aggregate["macro_score_m"],
                "control_macro_score_m": control_aggregate["macro_score_m"],
                "macro_improvement_m": macro_improvement,
                "mtv_h_improvement_m": target_improvement,
            },
            "gates": {
                "route_gates": {route: route_results[route]["gate"] for route in ROUTES},
                "macro": {"passed": not macro_failures, "failures": macro_failures},
                "all_applicable": all_passed,
                "all_passed": all_passed,
            },
            "stretch_target": {
                "score_m": STRETCH_TARGET_M,
                "candidate_macro_score_m": candidate_aggregate["macro_score_m"],
                "candidate_mtv_h_score_m": route_results[TARGET]["candidate"]["score_m"],
                "reached": candidate_aggregate["macro_score_m"] <= STRETCH_TARGET_M + TOLERANCE,
                "report_only": True,
            },
            "historical_phase51_reference": freeze["historical_reference_not_scoring_input"],
            "truth_accounting": {
                "truth_open_count": accounting["truth_reads"],
                "truth_read_count_per_route": 1,
                "candidate_submission_read_count_per_route": 1,
                "candidate_summary_read_count_per_route": 1,
                "control_submission_read_count_per_route": 1,
                "control_summary_read_count_per_route": 1,
                "single_process": True,
                "truth_reads_before_manifest": 0,
                "validation_truth_open_count": 0,
                "holdout_truth_open_count": 0,
                "mat_read_or_generated": False,
                "device_wls_or_precomputed_coordinates": 0,
                "raw_gnss_reads": 0,
                "raw_imu_reads": 0,
                "navigation_reads": 0,
                "kaggle_or_token_access": 0,
                "solver_rerun": False,
                "solver_rerun_after_truth": False,
                "post_truth_tuning": False,
            },
            "policy": {
                "phase43_champion_control_scored_exactly": True,
                "phase51_historical_metrics_scoring_input": False,
                "fresh_validation_opened": False,
                "kaggle_action": False,
            },
        }
        result_path = output_root / "phase59_native_cn0_doppler_calibration_accuracy.json"
        _atomic_json(result_path, report)
        result_hash = _hash_path(result_path, "Phase59 result")
        manifest = {
            "schema_version": "smartphone-r5-phase59-native-cn0-doppler-calibration-accuracy-output-manifest.v1",
            "phase": 59,
            "status": "sealed-one-shot-development-score",
            "freeze": {"path": _relative(FREEZE), "sha256": FREEZE_SHA256},
            "evaluator_manifest": {"path": _relative(EVALUATOR_MANIFEST), "sha256": report["evaluator_manifest"]["sha256"]},
            "result": {"path": _relative(result_path), "sha256": result_hash, "bytes": result_path.stat().st_size},
            "truth_reads": accounting["truth_reads"],
            "all_gates_passed": all_passed,
        }
        _atomic_json(output_root / "phase59_native_cn0_doppler_calibration_accuracy.manifest.json", manifest)
        return report
    except Phase59Error as exc:
        failure = {
            "schema_version": "smartphone-r5-phase59-native-cn0-doppler-calibration-accuracy-failure.v1",
            "status": "fail-closed",
            "error": str(exc),
            "truth_reads_attempted": accounting["truth_reads"],
            "partial_routes": route_results,
            "read_accounting": accounting,
            "solver_rerun": False,
            "post_truth_tuning": False,
        }
        if not output_root.exists():
            output_root.mkdir(parents=True, exist_ok=True)
        _atomic_json(output_root / "phase59_native_cn0_doppler_calibration_accuracy.failure.json", failure)
        raise


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("operation", nargs="?", choices=("score",))
    parser.add_argument("--verify-freeze", action="store_true")
    parser.add_argument("--output-root", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args(argv)
    try:
        if args.verify_freeze:
            _verify_freeze()
        if args.operation == "score":
            report = score(args.output_root)
            print(json.dumps({"all_gates_passed": report["gates"]["all_passed"], "truth_reads": report["truth_accounting"]["truth_open_count"], "status": report["status"]}, sort_keys=True))
        elif not args.verify_freeze:
            parser.error("one of --verify-freeze or score is required")
        return 0
    except Phase59Error as exc:
        print(f"phase59 accuracy failure: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
