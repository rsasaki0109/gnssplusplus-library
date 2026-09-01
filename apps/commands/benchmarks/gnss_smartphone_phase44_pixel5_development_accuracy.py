#!/usr/bin/env python3
"""Score the sealed Phase43 Pixel5 development artifacts exactly once.

Phase43 already produced the candidate and the flag-off controls.  This
evaluator never invokes the native binary and never rewrites either lane.  It
verifies those sealed bytes first, then uses one process to read the existing
Pixel5 truth once and to materialize/open/read each of the three added
``ground_truth.csv`` archive members once.  Timestamp matching and the local
Haversine/Kaggle diagnostic are deliberately frozen in the Phase44 record.

The only public operations are ``verify-freeze`` (no truth access) and the
one-shot ``score`` operation.  Validation, holdout, MAT, WLS, precomputed
coordinates, Kaggle services, and solver reruns are rejected by construction.
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
import zipfile


ROOT = Path(__file__).resolve().parents[3]
FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_phase44_pixel5_development_accuracy_freeze_v1.json"
FREEZE_SHA256 = "ec68888d210fb2eb1a43211ebef92fab582bf136ed16b8f105204d22673afa47"
EVALUATOR_MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase44_pixel5_development_accuracy_evaluator_manifest_v1.json"
STRUCTURAL_SEAL = ROOT / "output/smartphone-r5/phase43-native-fallback-seed-quality-anchor-recovery-v1/phase43_structural_seal.json"
STRUCTURAL_SEAL_SHA256 = "fdeaf672b015cae99dfdf8351a5e7a92ca2d37bcdb0872c5c3fc5b937416b64d"
PHASE43_RESULT = ROOT / "docs/use_cases/records/smartphone_r5_phase43_native_fallback_seed_quality_anchor_recovery_result_v1.json"
PHASE43_RESULT_SHA256 = "441c65ed8630ca2c48c329f440e8b54a772c0c062329cc5d89b297180bce2a22"
PHASE43_MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase43_native_fallback_seed_quality_anchor_recovery_evaluator_manifest_v1.json"
PHASE43_MANIFEST_SHA256 = "1433249a9ddd1809a00535b33fc67e3267a8cab29e28f773dd148f0962c44d1a"
PHASE37_FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_phase37_pixel5_repeatability_freeze_v2.json"
PHASE37_FREEZE_SHA256 = "61f46ac734d0c712bc317603570a0630234ee67ab642d12368eb5e3bf3962a3e"
ARCHIVE = ROOT / "data/gsdc2023/cache/dataset_2023.zip"
ARCHIVE_SHA256 = "bda30ab456e6fd6f83550c246e8dbd287306d5385f1f1069c99c16298e647408"
DEFAULT_OUTPUT = ROOT / "output/smartphone-r5/phase44-pixel5-development-accuracy-v1"

SCHEMA = "smartphone-r5-phase44-pixel5-development-accuracy.v1"
MANIFEST_SCHEMA = "smartphone-r5-phase44-pixel5-development-accuracy-manifest.v1"
FREEZE_SCHEMA = "smartphone-r5-phase44-pixel5-development-accuracy-freeze.v1"
TARGET = "2021-08-24-20-32-us-ca-mtv-h/pixel5"
ROUTES = (
    "2021-03-16-18-59-us-ca-mtv-a/pixel5",
    TARGET,
    "2022-04-01-18-22-us-ca-lax-t/pixel5",
    "2023-03-08-21-34-us-ca-mtv-u/pixel5",
)
FRESH_VALIDATION = "2023-05-09-21-32-us-ca-mtv-pe1/pixel5"
FUTURE_HOLDOUT = "2023-05-16-19-54-us-ca-mtv-xe1/pixel5"
HAVERSINE_RADIUS_M = 6_371_008.8
MAX_SPEED_MPS = 70.0
STRETCH_TARGET_M = 0.782
TOLERANCE = 1e-12
SUBMISSION_FIELDS = ("phone", "UnixTimeMillis", "LatitudeDegrees", "LongitudeDegrees")
TRUTH_FIELDS = ("UnixTimeMillis", "LatitudeDegrees", "LongitudeDegrees")
SUMMARY_CANDIDATE_FIELDS = (
    "native_fallback_seed_quality_anchor_recovery",
    "quality_anchor_initialization",
)
SUMMARY_RECOVERY_FIELDS = {
    "normal_quality_anchor_candidates",
    "recovery_anchor_gdop",
    "recovery_anchor_index",
    "recovery_anchor_normalized_residual_rms",
    "recovery_anchor_satellites",
    "recovery_anchor_selected",
    "recovery_enabled",
    "recovery_quality_anchor_candidates",
    "recovery_replay_invalid_epochs",
    "recovery_replay_valid_epochs",
    "recovery_trigger",
    "recovery_triggered",
    "sentinel_factor_bypass",
}


class Phase44Error(ValueError):
    """Raised when the immutable Phase44 contract cannot be proved."""


def _fail(message: str) -> Phase44Error:
    return Phase44Error(message)


def _reject_path(path: Path | str) -> None:
    token = str(path).lower()
    forbidden = (".mat", "validation", "holdout", "precomputed", "device_wls", "kaggle", "token")
    if any(item in token for item in forbidden):
        raise _fail(f"forbidden input/output path: {path}")


def _read_bytes_once(path: Path, label: str) -> tuple[bytes, str]:
    _reject_path(path)
    if not path.is_file():
        raise _fail(f"missing {label}: {path}")
    try:
        payload = path.read_bytes()
    except OSError as exc:
        raise _fail(f"failed to read {label}: {path}: {exc}") from exc
    return payload, hashlib.sha256(payload).hexdigest()


def _hash_path(path: Path, label: str) -> str:
    """Hash a non-truth immutable input (archive only in this evaluator)."""

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
    payload, digest = _read_bytes_once(path, label)
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
    # Native submissions use the complete dataset identity in their ``phone``
    # column (for example ``2021-08-24-.../pixel5``), while the archive member
    # path separately carries the phone model.
    if "/" not in route or route.endswith("/"):
        raise _fail(f"invalid route identity: {route}")
    return route


def _route_member(route: str) -> str:
    trip, phone = route.split("/", 1)
    return f"dataset_2023/train/{trip}/{phone}/ground_truth.csv"


def _finite_float(raw: str | None, field: str, row_number: int) -> float:
    try:
        value = float((raw or "").strip())
    except ValueError as exc:
        raise _fail(f"row {row_number}: invalid {field}") from exc
    if not math.isfinite(value):
        raise _fail(f"row {row_number}: non-finite {field}")
    return value


def _parse_coordinate_rows(payload: bytes, expected_phone: str, label: str) -> list[tuple[str, int, float, float]]:
    try:
        text = payload.decode("utf-8-sig")
    except UnicodeDecodeError as exc:
        raise _fail(f"{label} is not UTF-8 CSV") from exc
    reader = csv.DictReader(io.StringIO(text, newline=""))
    fields = list(reader.fieldnames or ())
    if tuple(fields) != SUBMISSION_FIELDS:
        raise _fail(f"{label} header must be exactly {','.join(SUBMISSION_FIELDS)}")
    rows: list[tuple[str, int, float, float]] = []
    seen: set[tuple[str, int]] = set()
    previous: int | None = None
    for row_number, raw in enumerate(reader, start=2):
        if None in raw:
            raise _fail(f"{label} row {row_number}: extra columns")
        phone = (raw.get("phone") or "")
        if phone != expected_phone or not phone or phone != phone.strip():
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
            raise _fail(f"{label} row {row_number}: duplicate key")
        seen.add(key)
        rows.append((phone, timestamp, latitude, longitude))
        previous = timestamp
    if not rows:
        raise _fail(f"{label} is empty")
    return rows


def _parse_truth_rows(payload: bytes, expected_phone: str, label: str) -> dict[tuple[str, int], tuple[float, float]]:
    try:
        text = payload.decode("utf-8-sig")
    except UnicodeDecodeError as exc:
        raise _fail(f"{label} is not UTF-8 CSV") from exc
    reader = csv.DictReader(io.StringIO(text, newline=""))
    fields = list(reader.fieldnames or ())
    if any(field not in fields for field in TRUTH_FIELDS):
        missing = [field for field in TRUTH_FIELDS if field not in fields]
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
            raise _fail(f"{label} row {row_number}: duplicate key")
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
        raise _fail("cannot calculate a percentile of an empty distribution")
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
    maximum: float | None = None
    for previous, current in zip(rows, rows[1:]):
        delta_ms = current[1] - previous[1]
        if delta_ms <= 0:
            raise _fail("prediction timestamps are not strictly increasing")
        speed = _haversine(previous[2], previous[3], current[2], current[3]) / (delta_ms / 1000.0)
        if not math.isfinite(speed):
            raise _fail("prediction speed is non-finite")
        speeds.append(speed)
        maximum = speed if maximum is None else max(maximum, speed)
    return {
        "transition_count": len(speeds),
        "max_speed_mps": maximum or 0.0,
        "over_70_mps_count": sum(speed > MAX_SPEED_MPS for speed in speeds),
    }


def _lane_metrics(rows: list[tuple[str, int, float, float]], truth: dict[tuple[str, int], tuple[float, float]]) -> dict[str, Any]:
    prediction = {(phone, timestamp): (latitude, longitude) for phone, timestamp, latitude, longitude in rows}
    matched = sorted(set(prediction) & set(truth), key=lambda key: key[1])
    if not matched:
        raise _fail("lane has no exact truth-matched keys")
    errors = [_haversine(*prediction[key], *truth[key]) for key in matched]
    p50 = _percentile(errors, 0.50)
    p95 = _percentile(errors, 0.95)
    continuity = _continuity(rows)
    metrics = {
        "finite": all(math.isfinite(value) for value in errors),
        "prediction_rows": len(rows),
        "truth_rows": len(truth),
        "matched_rows": len(matched),
        "missing_truth_rows": len(set(truth) - set(prediction)),
        "extra_prediction_rows": len(set(prediction) - set(truth)),
        "mean_m": sum(errors) / len(errors),
        "p50_m": p50,
        "p95_m": p95,
        "max_m": max(errors),
        "coverage_ratio": len(matched) / len(truth),
        "kaggle_score_m": (p50 + p95) / 2.0,
        "over_70_mps_count": continuity["over_70_mps_count"],
        "max_speed_mps": continuity["max_speed_mps"],
        "continuity": continuity,
        "matched_key_count": len(matched),
    }
    if not all(math.isfinite(float(metrics[key])) for key in ("mean_m", "p50_m", "p95_m", "max_m", "coverage_ratio", "kaggle_score_m")):
        raise _fail("lane metric is non-finite")
    return metrics


def _project_summary(value: Any, *, root: bool = True) -> Any:
    if isinstance(value, dict):
        result: dict[str, Any] = {}
        for key, item in value.items():
            if root and key in SUMMARY_CANDIDATE_FIELDS:
                continue
            if not root and key in SUMMARY_RECOVERY_FIELDS:
                continue
            result[key] = _project_summary(item, root=False)
        return result
    if isinstance(value, list):
        return [_project_summary(item, root=False) for item in value]
    return value


def _artifact_bytes(artifact: dict[str, Any], label: str) -> tuple[bytes, str]:
    path = artifact.get("path")
    expected = artifact.get("sha256")
    if not isinstance(path, str) or not isinstance(expected, str):
        raise _fail(f"invalid sealed {label} metadata")
    absolute = ROOT / path
    payload, digest = _read_bytes_once(absolute, label)
    if digest != expected:
        raise _fail(f"sealed {label} hash mismatch")
    if "bytes" in artifact and int(artifact["bytes"]) != len(payload):
        raise _fail(f"sealed {label} byte count mismatch")
    return payload, digest


def _verify_manifest() -> dict[str, Any]:
    manifest, digest, _ = _load_json_once(EVALUATOR_MANIFEST, "Phase44 evaluator manifest")
    if manifest.get("schema_version") != MANIFEST_SCHEMA or manifest.get("status") != "evaluator-frozen-before-phase44-truth":
        raise _fail("Phase44 evaluator manifest schema/status mismatch")
    freeze = manifest.get("freeze")
    if not isinstance(freeze, dict) or freeze.get("path") != _relative(FREEZE) or freeze.get("sha256") != FREEZE_SHA256:
        raise _fail("Phase44 evaluator manifest freeze pin mismatch")
    evaluator = manifest.get("evaluator")
    if not isinstance(evaluator, dict):
        raise _fail("Phase44 evaluator source pins missing")
    source = evaluator.get("source")
    test = evaluator.get("test")
    if not isinstance(source, dict) or not isinstance(test, dict):
        raise _fail("Phase44 source/test pins missing")
    if source.get("path") != _relative(Path(__file__)):
        raise _fail("Phase44 source path pin mismatch")
    source_hash = _hash_path(Path(__file__), "Phase44 evaluator source")
    if source_hash != source.get("sha256"):
        raise _fail("Phase44 evaluator source hash mismatch")
    test_path = ROOT / str(test.get("path", ""))
    if _hash_path(test_path, "Phase44 focused test") != test.get("sha256"):
        raise _fail("Phase44 focused test hash mismatch")
    if manifest.get("truth_policy", {}).get("truth_open_count_before_manifest") != 0:
        raise _fail("Phase44 manifest was not pre-truth")
    if manifest.get("truth_policy", {}).get("validation_truth_open_count") != 0 or manifest.get("truth_policy", {}).get("future_holdout_truth_open_count") != 0:
        raise _fail("Phase44 manifest opens validation/holdout")
    if manifest.get("truth_policy", {}).get("solver_rerun_after_truth") is not False:
        raise _fail("Phase44 manifest permits solver rerun")
    manifest["_sha256"] = digest
    return manifest


def _verify_freeze() -> dict[str, Any]:
    freeze, digest, _ = _load_json_once(FREEZE, "Phase44 freeze")
    if digest != FREEZE_SHA256:
        raise _fail("Phase44 freeze hash changed")
    if freeze.get("schema_version") != FREEZE_SCHEMA or freeze.get("status") != "frozen-before-truth-materialization-and-read":
        raise _fail("Phase44 freeze schema/status mismatch")
    cohort = freeze.get("cohort")
    if not isinstance(cohort, dict) or tuple(cohort.get("route_order", ())) != ROUTES:
        raise _fail("Phase44 route order changed")
    if cohort.get("fresh_validation", {}).get("id") != FRESH_VALIDATION or cohort.get("future_holdout", {}).get("id") != FUTURE_HOLDOUT:
        raise _fail("Phase44 validation/holdout identity changed")
    if cohort.get("fresh_validation", {}).get("remains_sealed") is not True or cohort.get("future_holdout", {}).get("remains_sealed") is not True:
        raise _fail("Phase44 validation/holdout is not sealed")
    archive = freeze.get("archive")
    if not isinstance(archive, dict) or archive.get("path") != _relative(ARCHIVE) or archive.get("sha256") != ARCHIVE_SHA256:
        raise _fail("Phase44 archive pin mismatch")
    timestamp = freeze.get("timestamp_contract")
    if not isinstance(timestamp, dict) or timestamp.get("key") != "(phone, UnixTimeMillis)" or timestamp.get("matching") != "exact integer UnixTimeMillis key intersection only":
        raise _fail("Phase44 timestamp matching contract changed")
    interpolation = timestamp.get("interpolation")
    if not isinstance(interpolation, dict) or any(interpolation.get(key) is not False for key in ("enabled", "bracketed_linear", "edge_hold", "extrapolation")):
        raise _fail("Phase44 interpolation contract changed")
    metric = freeze.get("metric_contract")
    if not isinstance(metric, dict) or metric.get("distance", {}).get("name") != "haversine_sphere" or metric.get("distance", {}).get("earth_radius_m") != HAVERSINE_RADIUS_M:
        raise _fail("Phase44 Haversine contract changed")
    if metric.get("kaggle_score") != "(p50_m + p95_m) / 2":
        raise _fail("Phase44 Kaggle score contract changed")
    if freeze.get("truth_budget", {}).get("total_route_truth_reads") != 4 or freeze.get("truth_budget", {}).get("single_process") is not True:
        raise _fail("Phase44 truth budget changed")
    if freeze.get("truth_budget", {}).get("candidate_reads_per_route") != 1 or freeze.get("truth_budget", {}).get("control_reads_per_valid_route") != 1:
        raise _fail("Phase44 artifact read budget changed")
    gates = freeze.get("absolute_gates")
    if not isinstance(gates, dict) or gates.get("all_candidate_rows_finite") is not True or gates.get("all_candidate_routes_full_coverage") is not True:
        raise _fail("Phase44 absolute finite/coverage gate changed")
    target_gate = gates.get("target_route", {})
    if target_gate.get("route") != TARGET or target_gate.get("kaggle_score_max_m") != 3.0 or target_gate.get("p95_max_m") != 5.0 or target_gate.get("over_70_mps_count") != 0:
        raise _fail("Phase44 target gate changed")
    if gates.get("pixel5_macro_score_max_m") != 2.0 or gates.get("individual_route_score_max_m") != 3.0 or gates.get("stretch_target_score_m") != STRETCH_TARGET_M:
        raise _fail("Phase44 macro/individual gate changed")
    return freeze


def _verify_phase43_records() -> dict[str, Any]:
    seal, seal_hash, _ = _load_json_once(STRUCTURAL_SEAL, "Phase43 structural seal")
    if seal_hash != STRUCTURAL_SEAL_SHA256 or seal.get("status") != "sealed-truth-free-structural-go" or seal.get("truth_accounting", {}).get("truth_open_count") != 0:
        raise _fail("Phase43 structural seal is not the sealed truth-free GO")
    if seal.get("truth_accounting", {}).get("mat_read_or_generated") is not False:
        raise _fail("Phase43 structural seal is not MAT-free")
    result, result_hash, _ = _load_json_once(PHASE43_RESULT, "Phase43 result record")
    if result_hash != PHASE43_RESULT_SHA256 or result.get("status") != "sealed-truth-free-structural-go" or result.get("decision", {}).get("truth_open_count") != 0:
        raise _fail("Phase43 result record changed")
    manifest, manifest_hash, _ = _load_json_once(PHASE43_MANIFEST, "Phase43 evaluator manifest")
    if manifest_hash != PHASE43_MANIFEST_SHA256 or manifest.get("truth_accounting", {}).get("allowed_truth_reads") != 0:
        raise _fail("Phase43 evaluator manifest changed")
    phase37, phase37_hash, _ = _load_json_once(PHASE37_FREEZE, "Phase37 freeze")
    if phase37_hash != PHASE37_FREEZE_SHA256 or phase37.get("schema_version") != "smartphone-r5-phase37-pixel5-repeatability-freeze.v2":
        raise _fail("Phase37 freeze pin changed")
    return {"seal": seal, "result": result, "manifest": manifest, "phase37": phase37}


def _verify_artifacts(freeze: dict[str, Any], seal: dict[str, Any]) -> dict[str, dict[str, Any]]:
    records: dict[str, dict[str, Any]] = {}
    sealed_routes = freeze.get("sealed_artifacts", {}).get("routes", {})
    if not isinstance(sealed_routes, dict):
        raise _fail("Phase44 sealed artifact records are missing")
    for route in ROUTES:
        row = sealed_routes.get(route)
        if not isinstance(row, dict):
            raise _fail(f"Phase44 sealed artifact route missing: {route}")
        seal_candidate = seal.get("candidate_runs", {}).get(route)
        seal_control = seal.get("flag_off_runs", {}).get(route)
        if not isinstance(seal_candidate, dict) or not isinstance(seal_control, dict):
            raise _fail(f"Phase43 route seal missing: {route}")
        candidate = row.get("candidate")
        if not isinstance(candidate, dict) or candidate.get("return_code") != 0:
            raise _fail(f"Phase44 candidate is not sealed successful: {route}")
        candidate_submission_payload, candidate_submission_hash = _artifact_bytes(candidate["submission"], f"candidate submission {route}")
        candidate_summary_payload, candidate_summary_hash = _artifact_bytes(candidate["summary"], f"candidate summary {route}")
        candidate_rows = _parse_coordinate_rows(candidate_submission_payload, _route_phone(route), f"candidate submission {route}")
        candidate_summary = _json_bytes(candidate_summary_payload, f"candidate summary {route}")
        if candidate_summary.get("truth_used") is not False or candidate_summary.get("production_default_changed") is not False:
            raise _fail(f"candidate summary truth/default contract failed: {route}")
        if candidate_summary.get("graph", {}).get("converged") is not True:
            raise _fail(f"candidate graph is not converged: {route}")
        expected_candidate_seal = seal_candidate.get("run1", {})
        for lane_name, payload_hash, expected in (("submission", candidate_submission_hash, expected_candidate_seal.get("submission", {}).get("sha256")), ("summary", candidate_summary_hash, expected_candidate_seal.get("summary", {}).get("sha256"))):
            if payload_hash != expected:
                raise _fail(f"Phase43 candidate {lane_name} hash disagreement: {route}")
        if seal_candidate.get("repeat_byte_identical") is not True:
            raise _fail(f"Phase43 candidate repeatability missing: {route}")
        controls: dict[str, Any] | None = None
        control = row.get("control")
        if not isinstance(control, dict):
            raise _fail(f"Phase44 control record missing: {route}")
        if route == TARGET:
            if control.get("return_code") != 1 or seal_control.get("run1", {}).get("return_code") != 1:
                raise _fail("MTV-h control is not the sealed fail-closed run")
            if control.get("submission") is not None or control.get("summary") is not None:
                raise _fail("MTV-h control unexpectedly has output")
        else:
            if control.get("return_code") != 0:
                raise _fail(f"valid control return code changed: {route}")
            control_submission_payload, control_submission_hash = _artifact_bytes(control["submission"], f"control submission {route}")
            control_summary_payload, control_summary_hash = _artifact_bytes(control["summary"], f"control summary {route}")
            control_rows = _parse_coordinate_rows(control_submission_payload, _route_phone(route), f"control submission {route}")
            control_summary = _json_bytes(control_summary_payload, f"control summary {route}")
            if control_summary.get("truth_used") is not False or control_summary.get("production_default_changed") is not False:
                raise _fail(f"control summary truth/default contract failed: {route}")
            if control_summary.get("graph", {}).get("converged") is not True:
                raise _fail(f"control graph is not converged: {route}")
            expected_control_seal = seal_control.get("run1", {})
            if control_submission_hash != expected_control_seal.get("submission", {}).get("sha256") or control_summary_hash != expected_control_seal.get("summary", {}).get("sha256"):
                raise _fail(f"Phase43 control hash disagreement: {route}")
            if seal_control.get("repeat_byte_identical") is not True:
                raise _fail(f"Phase43 control repeatability missing: {route}")
            if candidate_submission_payload != control_submission_payload:
                raise _fail(f"non-trigger candidate/control submission identity failed: {route}")
            if _project_summary(candidate_summary) != _project_summary(control_summary):
                raise _fail(f"non-trigger candidate/control projected summary identity failed: {route}")
            controls = {
                "rows": control_rows,
                "submission_sha256": control_submission_hash,
                "summary_sha256": control_summary_hash,
                "summary": control_summary,
            }
        records[route] = {
            "candidate": {
                "rows": candidate_rows,
                "submission_sha256": candidate_submission_hash,
                "summary_sha256": candidate_summary_hash,
                "summary": candidate_summary,
            },
            "control": controls,
        }
    return records


def _materialize_added_truth(output_root: Path, freeze: dict[str, Any]) -> tuple[dict[str, dict[str, Any]], dict[str, bytes]]:
    if _hash_path(ARCHIVE, "Phase44 archive") != ARCHIVE_SHA256:
        raise _fail("Phase44 archive hash changed")
    metadata = freeze.get("archive", {}).get("added_ground_truth_members", {})
    if not isinstance(metadata, dict):
        raise _fail("Phase44 added truth metadata missing")
    routes: dict[str, dict[str, Any]] = {}
    payloads: dict[str, bytes] = {}
    try:
        with zipfile.ZipFile(ARCHIVE) as archive:
            central: dict[str, list[zipfile.ZipInfo]] = {}
            for info in archive.infolist():
                central.setdefault(info.filename, []).append(info)
            for route in ROUTES[1:]:
                expected = metadata.get(route)
                if not isinstance(expected, dict):
                    raise _fail(f"Phase44 truth metadata missing: {route}")
                member = expected.get("name")
                if not isinstance(member, str) or member != _route_member(route) or Path(member).name != "ground_truth.csv":
                    raise _fail(f"unexpected truth member: {route}")
                infos = central.get(member, [])
                if len(infos) != 1:
                    raise _fail(f"missing or duplicate truth member: {member}")
                info = infos[0]
                if info.is_dir() or info.file_size != expected.get("file_size") or info.compress_size != expected.get("compressed_size") or f"{info.CRC:08x}" != expected.get("crc32_hex"):
                    raise _fail(f"truth central metadata mismatch: {member}")
                # This is the sole archive-member payload open/read for this route.
                with archive.open(info, "r") as source:
                    payload = source.read()
                if len(payload) != info.file_size:
                    raise _fail(f"truth member size changed: {route}")
                destination = output_root / "truth" / route / "ground_truth.csv"
                if destination.exists():
                    raise _fail(f"refusing to overwrite truth materialization: {destination}")
                _atomic_write(destination, payload)
                digest = hashlib.sha256(payload).hexdigest()
                routes[route] = {
                    "path": _relative(destination),
                    "member": member,
                    "bytes": len(payload),
                    "compressed_size": info.compress_size,
                    "crc32_hex": f"{info.CRC:08x}",
                    "sha256": digest,
                    "materialized": True,
                    "read_count": 1,
                }
                payloads[route] = payload
    except (OSError, zipfile.BadZipFile) as exc:
        raise _fail(f"failed to materialize added truth: {exc}") from exc
    if tuple(routes) != ROUTES[1:]:
        raise _fail("added truth materialization order changed")
    return routes, payloads


def _read_existing_truth(freeze: dict[str, Any]) -> tuple[dict[str, Any], bytes]:
    source = freeze.get("existing_truth")
    if not isinstance(source, dict) or source.get("path") is None or source.get("sha256") is None or source.get("phase44_read_count") != 1:
        raise _fail("existing truth source contract missing")
    path = ROOT / str(source["path"])
    payload, digest = _read_bytes_once(path, "existing Pixel5 truth")
    if digest != source["sha256"] or len(payload) != int(source.get("bytes", len(payload))):
        raise _fail("existing truth hash/size changed")
    return {"path": _relative(path), "sha256": digest, "bytes": len(payload), "materialized": False, "read_count": 1}, payload


def _route_gate(route: str, candidate: dict[str, Any]) -> dict[str, Any]:
    failures: list[str] = []
    if candidate["finite"] is not True:
        failures.append("nonfinite")
    if candidate["coverage_ratio"] != 1.0:
        failures.append("incomplete_coverage")
    if candidate["over_70_mps_count"] != 0:
        failures.append("over_70_mps")
    if candidate["kaggle_score_m"] > 3.0 + TOLERANCE:
        failures.append("score_over_3m")
    if route == TARGET and candidate["p95_m"] > 5.0 + TOLERANCE:
        failures.append("target_p95_over_5m")
    if candidate["kaggle_score_m"] > 3.0 + TOLERANCE:
        failures.append("individual_score_over_3m")
    return {"passed": not failures, "failures": failures}


def _aggregate(candidate_metrics: list[dict[str, Any]]) -> dict[str, Any]:
    if not candidate_metrics:
        raise _fail("cannot aggregate no candidate metrics")
    keys = ("mean_m", "p50_m", "p95_m", "max_m", "coverage_ratio", "kaggle_score_m", "max_speed_mps")
    result = {f"mean_{key}": sum(float(item[key]) for item in candidate_metrics) / len(candidate_metrics) for key in keys}
    result.update({
        "route_count": len(candidate_metrics),
        "pixel5_macro_score_m": sum(float(item["kaggle_score_m"]) for item in candidate_metrics) / len(candidate_metrics),
        "max_route_score_m": max(float(item["kaggle_score_m"]) for item in candidate_metrics),
        "sum_over_70_mps_count": sum(int(item["over_70_mps_count"]) for item in candidate_metrics),
        "all_finite": all(item["finite"] is True for item in candidate_metrics),
        "all_full_coverage": all(item["coverage_ratio"] == 1.0 for item in candidate_metrics),
    })
    return result


def score(output_root: Path = DEFAULT_OUTPUT) -> dict[str, Any]:
    """Run the one-shot post-seal score; this function never invokes native."""

    started = time.perf_counter()
    output_root = output_root.resolve()
    _reject_path(output_root)
    if output_root.exists() and any(output_root.iterdir()):
        raise _fail(f"refusing to rerun Phase44 evaluation: {output_root}")
    freeze = _verify_freeze()
    _verify_manifest()
    phase43 = _verify_phase43_records()
    artifacts = _verify_artifacts(freeze, phase43["seal"])
    # No truth is opened before every immutable evaluator and artifact check above.
    existing_truth_meta, existing_truth_payload = _read_existing_truth(freeze)
    added_truth_meta, added_truth_payloads = _materialize_added_truth(output_root, freeze)
    truth_payloads = {ROUTES[0]: existing_truth_payload, **added_truth_payloads}
    truth_meta = {ROUTES[0]: existing_truth_meta, **added_truth_meta}
    route_results: dict[str, Any] = {}
    candidate_rows: list[dict[str, Any]] = []
    control_rows: list[dict[str, Any]] = []
    for route in ROUTES:
        phone = _route_phone(route)
        truth = _parse_truth_rows(truth_payloads[route], phone, f"truth {route}")
        candidate_metrics = _lane_metrics(artifacts[route]["candidate"]["rows"], truth)
        candidate_gate = _route_gate(route, candidate_metrics)
        control_metrics: dict[str, Any] | None = None
        control_gate: dict[str, Any] | None = None
        if artifacts[route]["control"] is not None:
            control_metrics = _lane_metrics(artifacts[route]["control"]["rows"], truth)
            control_gate = {"finite": control_metrics["finite"], "coverage_ratio": control_metrics["coverage_ratio"], "over_70_mps_count": control_metrics["over_70_mps_count"] == 0}
            control_rows.append(control_metrics)
        candidate_rows.append(candidate_metrics)
        row: dict[str, Any] = {
            "role": "existing" if route == ROUTES[0] else "added",
            "truth": truth_meta[route] | {"rows": len(truth), "read_count": 1},
            "candidate": candidate_metrics,
            "candidate_gate": candidate_gate,
            "artifact_hashes": {
                "candidate_submission_sha256": artifacts[route]["candidate"]["submission_sha256"],
                "candidate_summary_sha256": artifacts[route]["candidate"]["summary_sha256"],
            },
        }
        if control_metrics is not None:
            row["control"] = control_metrics
            row["control_gate"] = control_gate
            row["artifact_hashes"].update({
                "control_submission_sha256": artifacts[route]["control"]["submission_sha256"],
                "control_summary_sha256": artifacts[route]["control"]["summary_sha256"],
            })
        else:
            row["control"] = {"status": "phase43-structural-fail-closed", "return_code": 1, "output_written": False}
        route_results[route] = row
    aggregate = _aggregate(candidate_rows)
    macro_failures: list[str] = []
    if aggregate["pixel5_macro_score_m"] > 2.0 + TOLERANCE:
        macro_failures.append("pixel5_macro_score_over_2m")
    if aggregate["all_finite"] is not True:
        macro_failures.append("nonfinite_route")
    if aggregate["all_full_coverage"] is not True:
        macro_failures.append("incomplete_route_coverage")
    if aggregate["sum_over_70_mps_count"] != 0:
        macro_failures.append("over_70_mps")
    gates = {
        "route_gates": {route: route_results[route]["candidate_gate"] for route in ROUTES},
        "target": route_results[TARGET]["candidate_gate"],
        "pixel5_macro": {"passed": not macro_failures, "failures": macro_failures, "score_m": aggregate["pixel5_macro_score_m"], "max_m": 2.0},
        "all_applicable": not macro_failures and all(route_results[route]["candidate_gate"]["passed"] for route in ROUTES),
    }
    passed = bool(gates["all_applicable"])
    report = {
        "schema_version": SCHEMA,
        "phase": 44,
        "status": "development-absolute-gate-pass" if passed else "no-go-development-absolute-gate",
        "decision": "development-only-validation-not-opened" if passed else "no-go-no-validation",
        "freeze": {"path": _relative(FREEZE), "sha256": FREEZE_SHA256},
        "evaluator_manifest": {"path": _relative(EVALUATOR_MANIFEST), "sha256": _hash_path(EVALUATOR_MANIFEST, "Phase44 evaluator manifest")},
        "phase43_structural_seal": {"path": _relative(STRUCTURAL_SEAL), "sha256": STRUCTURAL_SEAL_SHA256},
        "route_order": list(ROUTES),
        "timestamp_contract": freeze["timestamp_contract"],
        "metric_contract": freeze["metric_contract"],
        "routes": route_results,
        "aggregate": {"candidate": aggregate, "valid_controls": _aggregate(control_rows) if control_rows else None},
        "gates": gates,
        "stretch_target": {
            "score_m": STRETCH_TARGET_M,
            "candidate_target_route_score_m": route_results[TARGET]["candidate"]["kaggle_score_m"],
            "met": route_results[TARGET]["candidate"]["kaggle_score_m"] <= STRETCH_TARGET_M + TOLERANCE,
            "policy": "reported honestly; no gate relaxation or post-truth tuning",
        },
        "truth_accounting": {
            "truth_open_count": 4,
            "truth_read_count_per_route": 1,
            "existing_truth_open_count": 1,
            "added_truth_member_materialized_count": 3,
            "added_truth_member_open_count": 3,
            "candidate_read_count_per_route": 1,
            "valid_control_read_count_per_route": 1,
            "validation_truth_open_count": 0,
            "holdout_truth_open_count": 0,
            "mat_read_or_generated": False,
            "precomputed_coordinates_used_for_inference": 0,
            "device_wls_coordinates_used": 0,
            "kaggle_or_token_access": 0,
        },
        "policy": {
            "solver_rerun": False,
            "solver_rerun_after_truth": False,
            "post_truth_tuning": False,
            "validation_opened": False,
            "holdout_opened": False,
            "phase31_champion_mutated": False,
            "truth_used_for_inference_or_selection": False,
        },
        "runtime": {"wall_seconds": time.perf_counter() - started},
    }
    output_root.mkdir(parents=True, exist_ok=True)
    _atomic_json(output_root / "truth_materialization.json", {
        "schema_version": "smartphone-r5-phase44-truth-materialization.v1",
        "status": "truth-materialized-and-read-once",
        "routes": truth_meta,
        "truth_open_count": 4,
        "truth_read_count_per_route": 1,
        "validation_truth_materialized": False,
        "future_holdout_truth_materialized": False,
        "mat_read_or_generated": False,
    })
    result_path = output_root / "phase44_pixel5_development_accuracy.json"
    _atomic_json(result_path, report)
    _atomic_json(output_root / "phase44_pixel5_development_accuracy.manifest.json", {
        "schema_version": MANIFEST_SCHEMA,
        "status": report["status"],
        "result": {"path": _relative(result_path), "sha256": _hash_path(result_path, "Phase44 result")},
        "freeze": report["freeze"],
        "evaluator_manifest": report["evaluator_manifest"],
        "truth_open_count": 4,
        "truth_read_count_per_route": 1,
        "validation_truth_open_count": 0,
        "future_holdout_truth_open_count": 0,
        "solver_rerun_after_truth": False,
        "post_truth_tuning": False,
        "mat_read_or_generated": False,
        "atomic_publish": True,
    })
    return report


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("operation", choices=("verify-freeze", "score"))
    parser.add_argument("--output-root", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args(argv)
    try:
        if args.operation == "verify-freeze":
            freeze = _verify_freeze()
            print(json.dumps({"status": freeze["status"], "truth_open_count": 0}, sort_keys=True))
        else:
            report = score(args.output_root)
            print(json.dumps({"status": report["status"], "truth_open_count": report["truth_accounting"]["truth_open_count"], "pixel5_macro_score_m": report["aggregate"]["candidate"]["pixel5_macro_score_m"]}, sort_keys=True))
    except (OSError, Phase44Error, ValueError, zipfile.BadZipFile) as exc:
        print(f"phase44: {exc}", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
