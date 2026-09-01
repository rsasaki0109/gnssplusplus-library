#!/usr/bin/env python3
"""Recover the Phase51 development score from immutable output artifacts.

This is an integrity-recovery scorer, not a native run.  It contains no
subprocess/native-solver path.  The preflight reads only sealed JSON records;
the score operation then reads each Phase51 candidate run1 and control run1
submission/summary exactly once and each already-materialized Phase44 truth
CSV exactly once in this process.  Phase51 candidate run2 is represented only
by its frozen repeat hash and is not reopened.

The exact-key contract deliberately distinguishes prediction-domain coverage
from truth-row coverage: the first truth timestamp may be the raw-UTC warm-up
row absent from a post-warm-up prediction.  No interpolation, edge hold,
nearest matching, solver rerun, tuning, validation, holdout, MAT, WLS,
precomputed-coordinate, archive, or Kaggle/token operation is present.
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
import tempfile
from typing import Any


ROOT = Path(__file__).resolve().parents[3]
FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_phase52_android_sv_time_uncertainty_integrity_recovery_freeze_v1.json"
FREEZE_SHA256 = "e88de17ba1dff7e915e365b34d9fcdf3da769d8113b38b15b21b8bb7923c41d3"
MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase52_android_sv_time_uncertainty_integrity_recovery_evaluator_manifest_v2.json"
DEFAULT_OUTPUT = ROOT / "output/smartphone-r5/phase52-android-sv-time-uncertainty-integrity-recovery-v1"
RESULT_NAME = "phase52_integrity_recovery_result.json"
ARTIFACT_MANIFEST_NAME = "phase52_integrity_recovery.manifest.json"
FAILURE_NAME = "phase52_integrity_recovery_failure.json"

PHASE51_RESULT = ROOT / "docs/use_cases/records/smartphone_r5_phase51_android_sv_time_uncertainty_sigma_floor_result_v1.json"
PHASE51_OUTPUT = ROOT / "output/smartphone-r5/phase51-android-sv-time-uncertainty-sigma-floor-v2/phase51_structural_and_development.json"
PHASE44_RESULT = ROOT / "docs/use_cases/records/smartphone_r5_phase44_pixel5_development_accuracy_result_v1.json"

SCHEMA = "smartphone-r5-phase52-android-sv-time-uncertainty-integrity-recovery.v1"
MANIFEST_SCHEMA = "smartphone-r5-phase52-android-sv-time-uncertainty-integrity-recovery-manifest.v2"
FREEZE_SCHEMA = "smartphone-r5-phase52-android-sv-time-uncertainty-integrity-recovery-freeze.v1"
ROUTES = (
    "2021-03-16-18-59-us-ca-mtv-a/pixel5",
    "2021-08-24-20-32-us-ca-mtv-h/pixel5",
    "2022-04-01-18-22-us-ca-lax-t/pixel5",
    "2023-03-08-21-34-us-ca-mtv-u/pixel5",
)
TARGET_ROUTE = ROUTES[1]
SUBMISSION_FIELDS = ("phone", "UnixTimeMillis", "LatitudeDegrees", "LongitudeDegrees")
TRUTH_FIELDS = ("UnixTimeMillis", "LatitudeDegrees", "LongitudeDegrees")
HAVERSINE_RADIUS_M = 6_371_008.8
SPEED_RADIUS_M = 6_371_000.0
MAX_SPEED_MPS = 70.0
STRETCH_TARGET_M = 0.782


class Phase52Error(ValueError):
    """Raised when the frozen integrity-recovery contract fails."""


def fail(message: str) -> Phase52Error:
    return Phase52Error(message)


def relative(path: Path) -> str:
    try:
        return str(path.relative_to(ROOT))
    except ValueError:
        return str(path)


def reject_forbidden(path: Path | str, *, allow_truth: bool = False) -> None:
    token = str(path).lower()
    for item in (".mat", "validation", "holdout", "precomputed", "device_wls", "svposition", "kaggle", "token"):
        if item in token:
            raise fail(f"forbidden path: {path}")
    if not allow_truth and ("ground_truth" in token or "/truth/" in token):
        raise fail(f"truth path before score phase: {path}")


def read_bytes_once(path: Path, label: str, *, allow_truth: bool = False) -> tuple[bytes, str]:
    reject_forbidden(path, allow_truth=allow_truth)
    if not path.is_file():
        raise fail(f"missing {label}: {path}")
    digest = hashlib.sha256()
    payload = bytearray()
    try:
        with path.open("rb") as handle:
            for chunk in iter(lambda: handle.read(1024 * 1024), b""):
                payload.extend(chunk)
                digest.update(chunk)
    except OSError as exc:
        raise fail(f"failed to read {label}: {path}") from exc
    return bytes(payload), digest.hexdigest()


def load_json_once(path: Path, label: str, *, allow_truth: bool = False) -> tuple[dict[str, Any], str, int]:
    payload, digest = read_bytes_once(path, label, allow_truth=allow_truth)
    try:
        value = json.loads(payload.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise fail(f"invalid {label}: {path}") from exc
    if not isinstance(value, dict):
        raise fail(f"{label} is not an object")
    return value, digest, len(payload)


def atomic_write(path: Path, payload: bytes) -> None:
    reject_forbidden(path)
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


def atomic_json(path: Path, value: dict[str, Any]) -> None:
    atomic_write(path, (json.dumps(value, indent=2, sort_keys=True) + "\n").encode("utf-8"))


def verify_freeze() -> dict[str, Any]:
    freeze, digest, _ = load_json_once(FREEZE, "Phase52 freeze")
    if digest != FREEZE_SHA256 or freeze.get("schema_version") != FREEZE_SCHEMA or freeze.get("phase") != 52 or freeze.get("status") != "frozen-before-phase52-truth-read":
        raise fail("Phase52 freeze hash/schema/status mismatch")
    authority = freeze.get("authority", {})
    if authority.get("base_commit") != "9a7def1":
        raise fail("Phase52 base commit pin changed")
    phase51 = authority.get("phase51_result", {})
    if phase51.get("path") != relative(PHASE51_RESULT) or phase51.get("sha256") != "b8f7ae93af72e147ef10b839e42a3bf52a51196cfecc2a4596dd76b7c55b78f3":
        raise fail("Phase51 result pin changed")
    if tuple(freeze.get("immutable_phase51_outputs", {}).get("route_order", ())) != ROUTES:
        raise fail("Phase51 route order pin changed")
    outputs = freeze.get("immutable_phase51_outputs", {})
    if outputs.get("candidate_runs_per_route") != 2 or outputs.get("control_runs_per_route") != 1 or outputs.get("candidate_repeat_identity_required") is not True or outputs.get("structural_pass_required_and_pinned") is not True:
        raise fail("Phase51 output read/repeat contract changed")
    routes = outputs.get("routes", {})
    if set(routes) != set(ROUTES):
        raise fail("Phase51 artifact route set changed")
    for route in ROUTES:
        entry = routes[route]
        for lane in ("candidate_run1", "candidate_run2", "control_run1"):
            for artifact_type in ("submission", "summary"):
                artifact = entry.get(lane, {}).get(artifact_type, {})
                if not isinstance(artifact.get("path"), str) or not isinstance(artifact.get("sha256"), str) or len(artifact["sha256"]) != 64 or int(artifact.get("bytes", 0)) <= 0:
                    raise fail(f"malformed Phase51 artifact pin: {route}/{lane}/{artifact_type}")
            if int(entry[lane]["submission"].get("rows", 0)) <= 0:
                raise fail(f"malformed Phase51 row pin: {route}/{lane}")
    domain = freeze.get("domain_contract", {})
    if domain.get("key") != "(phone, UnixTimeMillis)" or domain.get("prediction_domain_coverage_required") != 1.0 or domain.get("truth_row_coverage") != "informational only":
        raise fail("Phase52 domain contract changed")
    warmup = domain.get("warmup_exclusions", {})
    expected_missing = warmup.get("expected_missing_truth_rows", {})
    if set(expected_missing) != set(ROUTES) or expected_missing.get(ROUTES[0]) != 1 or expected_missing.get(TARGET_ROUTE) != 0 or expected_missing.get(ROUTES[2]) != 0 or expected_missing.get(ROUTES[3]) != 1:
        raise fail("Phase52 warm-up count map changed")
    if "first chronological truth timestamp" not in warmup.get("missing_key_rule", "") or warmup.get("arbitrary_missing_keys_forbidden") is not True:
        raise fail("Phase52 warm-up key rule changed")
    metric = domain.get("metric", {})
    if metric.get("earth_radius_m") != HAVERSINE_RADIUS_M or metric.get("percentile") != "linear rank (n - 1) * q" or metric.get("score") != "(P50 + P95) / 2":
        raise fail("Phase52 metric contract changed")
    gates = freeze.get("accuracy_gates", {})
    for key, expected_value in (("candidate_improvement_each_route_min_m", 0.05), ("macro_improvement_min_m", 0.10), ("mtv_h_improvement_min_m", 0.10), ("absolute_candidate_macro_max_m", 2.0), ("absolute_candidate_route_max_m", 3.0), ("absolute_mtv_h_p95_max_m", 5.0), ("over_70_mps_count", 0)):
        if gates.get(key) != expected_value:
            raise fail(f"Phase52 accuracy gate changed: {key}")
    read_policy = freeze.get("read_policy", {})
    for key, expected_value in (("phase52_truth_reads_before_freeze", 0), ("phase52_truth_reads_before_evaluator_manifest_seal", 0), ("truth_read_once_per_route", 1), ("solver_process_invocations", 0), ("validation_reads", 0), ("holdout_reads", 0), ("kaggle_or_token_access", 0)):
        if read_policy.get(key) != expected_value:
            raise fail(f"Phase52 read policy changed: {key}")
    if read_policy.get("single_process") is not True or read_policy.get("truth_derived_inference_or_tuning") is not False:
        raise fail("Phase52 truth policy changed")
    return freeze


def verify_manifest() -> tuple[dict[str, Any], str]:
    manifest, digest, _ = load_json_once(MANIFEST, "Phase52 evaluator manifest")
    if manifest.get("schema_version") != MANIFEST_SCHEMA or manifest.get("status") != "evaluator-frozen-before-phase52-truth-read":
        raise fail("Phase52 evaluator manifest schema/status mismatch")
    freeze_pin = manifest.get("freeze", {})
    if freeze_pin.get("path") != relative(FREEZE) or freeze_pin.get("sha256") != FREEZE_SHA256:
        raise fail("Phase52 manifest freeze pin changed")
    evaluator = manifest.get("evaluator", {})
    source = evaluator.get("source", {})
    if source.get("path") != relative(Path(__file__)) or read_bytes_once(Path(__file__), "Phase52 scorer source")[1] != source.get("sha256"):
        raise fail("Phase52 scorer source hash mismatch")
    for test in evaluator.get("focused_tests", []):
        path = ROOT / str(test.get("path", ""))
        if read_bytes_once(path, "Phase52 focused test")[1] != test.get("sha256"):
            raise fail("Phase52 focused test hash mismatch")
    cmake = evaluator.get("cmake", {})
    cmake_path = ROOT / str(cmake.get("path", ""))
    if read_bytes_once(cmake_path, "Phase52 CMake test list")[1] != cmake.get("sha256"):
        raise fail("Phase52 CMake hash mismatch")
    policy = manifest.get("truth_policy", {})
    if policy.get("truth_reads_before_manifest_seal") != 0 or policy.get("truth_reads_before_score") != 0 or policy.get("solver_process_invocations") != 0 or policy.get("solver_rerun") is not False:
        raise fail("Phase52 manifest truth/solver policy changed")
    if tuple(manifest.get("route_order", ())) != ROUTES or manifest.get("candidate_run1_read_count") != 1 or manifest.get("control_run1_read_count") != 1 or manifest.get("truth_read_count_per_route") != 1:
        raise fail("Phase52 manifest read matrix changed")
    return manifest, digest


def verify_historical_pins(freeze: dict[str, Any]) -> dict[str, Any]:
    result, result_hash, _ = load_json_once(PHASE51_RESULT, "Phase51 result")
    expected_hash = freeze["authority"]["phase51_result"]["sha256"]
    if result_hash != expected_hash or result.get("status") != "evaluator-integrity-no-go" or result.get("evaluator", {}).get("structural_pass") is not True or result.get("truth_accounting", {}).get("truth_reads_after_structural_pass") != 4:
        raise fail("Phase51 result integrity pin failed")
    output, output_hash, _ = load_json_once(PHASE51_OUTPUT, "Phase51 output result")
    if output_hash != freeze["authority"]["phase51_output_result"]["sha256"] or output.get("evaluator", {}).get("structural_pass") is not True:
        raise fail("Phase51 structural output pin failed")
    p44, p44_hash, _ = load_json_once(PHASE44_RESULT, "Phase44 result")
    if p44_hash != freeze["authority"]["phase44_result"]["sha256"] or p44.get("metric_contract", {}).get("earth_radius_m") != HAVERSINE_RADIUS_M or p44.get("metric_contract", {}).get("kaggle_score") != "(P50 + P95) / 2":
        raise fail("Phase44 metric pin failed")
    return {"phase51_result": result, "phase51_output": output, "phase44_result": p44}


def read_artifact_once(path: Path, label: str, expected_sha: str, expected_bytes: int) -> tuple[bytes, str]:
    payload, digest = read_bytes_once(path, label)
    if digest != expected_sha or len(payload) != expected_bytes:
        raise fail(f"sealed {label} hash/size mismatch")
    return payload, digest


def parse_submission_once(payload: bytes, route: str, label: str) -> list[tuple[str, int, float, float]]:
    try:
        text = payload.decode("utf-8-sig")
    except UnicodeDecodeError as exc:
        raise fail(f"{label} is not UTF-8") from exc
    reader = csv.DictReader(io.StringIO(text, newline=""))
    if tuple(reader.fieldnames or ()) != SUBMISSION_FIELDS:
        raise fail(f"{label} header mismatch")
    rows: list[tuple[str, int, float, float]] = []
    seen: set[tuple[str, int]] = set()
    previous: int | None = None
    for row_number, row in enumerate(reader, start=2):
        if None in row or row.get("phone") != route:
            raise fail(f"{label} row {row_number} identity/extra-column mismatch")
        try:
            timestamp = int((row.get("UnixTimeMillis") or "").strip())
            latitude = float((row.get("LatitudeDegrees") or "").strip())
            longitude = float((row.get("LongitudeDegrees") or "").strip())
        except ValueError as exc:
            raise fail(f"{label} row {row_number} is non-numeric") from exc
        if not all(math.isfinite(v) for v in (latitude, longitude)) or not -90.0 <= latitude <= 90.0 or not -180.0 <= longitude <= 180.0:
            raise fail(f"{label} row {row_number} coordinate invalid")
        if previous is not None and timestamp <= previous:
            raise fail(f"{label} timestamps are not strictly increasing")
        key = (route, timestamp)
        if key in seen:
            raise fail(f"{label} duplicate key")
        seen.add(key)
        rows.append((route, timestamp, latitude, longitude))
        previous = timestamp
    if not rows:
        raise fail(f"{label} is empty")
    return rows


def parse_summary_once(payload: bytes, route: str, candidate: bool, label: str) -> dict[str, Any]:
    try:
        summary = json.loads(payload.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise fail(f"{label} is invalid JSON") from exc
    if not isinstance(summary, dict) or summary.get("dataset_id") != route or summary.get("truth_used") is not False or summary.get("production_default_changed") is not False:
        raise fail(f"{label} identity/truth/default gate failed")
    if summary.get("graph", {}).get("converged") is not True:
        raise fail(f"{label} graph is not converged")
    tdcp = summary.get("tdcp_contract", {})
    if int(tdcp.get("factors_built", 0)) <= 0 or int(tdcp.get("factors_inserted", -1)) != int(tdcp.get("factors_built", 0)) or int(tdcp.get("nonfinite_residuals", -1)) != 0:
        raise fail(f"{label} TDCP integrity failed")
    if summary.get("native_quality_anchor") is not True or summary.get("native_pdc_imu_tdcp_no_bridge") is not True or summary.get("native_pdc_state_bridge") is not False:
        raise fail(f"{label} Phase43 base contract failed")
    if candidate:
        section = summary.get("native_android_sv_time_uncertainty_sigma_floor")
        if not isinstance(section, dict) or section.get("enabled") is not True or section.get("source_field") != "ReceivedSvTimeUncertaintyNanos" or section.get("coefficient") != 1.0 or section.get("upper_clip") is not False or section.get("spp_applied") is not False:
            raise fail(f"{label} Phase51 telemetry contract failed")
    elif "native_android_sv_time_uncertainty_sigma_floor" in summary:
        raise fail(f"{label} flag-off telemetry leaked")
    return summary


def read_truth_once(path: Path, route: str, expected_sha: str, expected_bytes: int) -> tuple[dict[tuple[str, int], tuple[float, float]], dict[str, Any]]:
    payload, digest = read_bytes_once(path, f"Phase44 truth {route}", allow_truth=True)
    if digest != expected_sha or len(payload) != expected_bytes:
        raise fail(f"Phase44 truth hash/size mismatch: {route}")
    try:
        text = payload.decode("utf-8-sig")
    except UnicodeDecodeError as exc:
        raise fail(f"Phase44 truth is not UTF-8: {route}") from exc
    reader = csv.DictReader(io.StringIO(text, newline=""))
    if any(field not in set(reader.fieldnames or ()) for field in TRUTH_FIELDS):
        raise fail(f"Phase44 truth fields missing: {route}")
    has_phone = "phone" in set(reader.fieldnames or ())
    values: dict[tuple[str, int], tuple[float, float]] = {}
    for row_number, row in enumerate(reader, start=2):
        phone = (row.get("phone") or route) if has_phone else route
        if phone != route:
            raise fail(f"truth phone mismatch row {row_number}: {route}")
        try:
            timestamp = int((row.get("UnixTimeMillis") or "").strip())
            latitude = float((row.get("LatitudeDegrees") or "").strip())
            longitude = float((row.get("LongitudeDegrees") or "").strip())
        except ValueError as exc:
            raise fail(f"truth numeric error row {row_number}: {route}") from exc
        if not all(math.isfinite(v) for v in (latitude, longitude)) or not -90.0 <= latitude <= 90.0 or not -180.0 <= longitude <= 180.0:
            raise fail(f"truth coordinate error row {row_number}: {route}")
        key = (phone, timestamp)
        if key in values:
            raise fail(f"truth duplicate key row {row_number}: {route}")
        values[key] = (latitude, longitude)
    if not values:
        raise fail(f"empty Phase44 truth: {route}")
    return values, {"path": relative(path), "sha256": digest, "bytes": len(payload), "rows": len(values), "read_count": 1}


def truth_paths() -> dict[str, Path]:
    return {
        ROUTES[0]: ROOT / "output/smartphone-r5/phase29-no-bridge-train-eval-v1/truth/2021-03-16-18-59-us-ca-mtv-a/pixel5/ground_truth.csv",
        ROUTES[1]: ROOT / "output/smartphone-r5/phase44-pixel5-development-accuracy-v1/truth/2021-08-24-20-32-us-ca-mtv-h/pixel5/ground_truth.csv",
        ROUTES[2]: ROOT / "output/smartphone-r5/phase44-pixel5-development-accuracy-v1/truth/2022-04-01-18-22-us-ca-lax-t/pixel5/ground_truth.csv",
        ROUTES[3]: ROOT / "output/smartphone-r5/phase44-pixel5-development-accuracy-v1/truth/2023-03-08-21-34-us-ca-mtv-u/pixel5/ground_truth.csv",
    }


def check_domain(rows: list[tuple[str, int, float, float]], truth: dict[tuple[str, int], tuple[float, float]], route: str, expected_missing: int, label: str) -> dict[str, Any]:
    prediction_keys = {(phone, timestamp) for phone, timestamp, _, _ in rows}
    truth_keys = set(truth)
    matched = prediction_keys & truth_keys
    extras = prediction_keys - truth_keys
    missing = truth_keys - prediction_keys
    if extras or len(prediction_keys) != len(rows):
        raise fail(f"{label} duplicate/extra prediction keys: {route}")
    if len(missing) != expected_missing:
        raise fail(f"{label} warm-up missing count mismatch: {route}")
    if expected_missing:
        first_truth = min(truth_keys, key=lambda key: key[1])
        if missing != {first_truth}:
            raise fail(f"{label} missing key is not exactly first truth timestamp: {route}")
        first_prediction = min(prediction_keys, key=lambda key: key[1])
        if first_prediction[1] <= first_truth[1]:
            raise fail(f"{label} warm-up ordering is not strict: {route}")
    elif missing:
        raise fail(f"{label} unexpected truth rows missing: {route}")
    if matched != prediction_keys:
        raise fail(f"{label} prediction domain is not fully matched: {route}")
    return {"prediction_rows": len(rows), "truth_rows": len(truth), "matched_rows": len(matched), "missing_truth_rows": len(missing), "extra_prediction_rows": len(extras), "prediction_domain_coverage": 1.0, "truth_row_coverage": len(matched) / len(truth), "warmup_exclusion_observed": len(missing), "warmup_exclusion_expected": expected_missing, "warmup_missing_key": next(iter(missing))[1] if missing else None}


def haversine(lat0: float, lon0: float, lat1: float, lon1: float, radius: float = HAVERSINE_RADIUS_M) -> float:
    phi0, phi1 = math.radians(lat0), math.radians(lat1)
    dphi, dlambda = math.radians(lat1 - lat0), math.radians(lon1 - lon0)
    value = math.sin(dphi / 2.0) ** 2 + math.cos(phi0) * math.cos(phi1) * math.sin(dlambda / 2.0) ** 2
    distance = 2.0 * radius * math.asin(math.sqrt(min(1.0, max(0.0, value))))
    if not math.isfinite(distance) or distance < 0.0:
        raise fail("non-finite Haversine distance")
    return distance


def percentile(values: list[float], q: float) -> float:
    ordered = sorted(values)
    if not ordered:
        raise fail("empty error distribution")
    rank = (len(ordered) - 1) * q
    low, high = math.floor(rank), math.ceil(rank)
    if low == high:
        return ordered[low]
    alpha = rank - low
    return ordered[low] * (1.0 - alpha) + ordered[high] * alpha


def speed_report(rows: list[tuple[str, int, float, float]]) -> dict[str, Any]:
    speeds: list[float] = []
    for previous, current in zip(rows, rows[1:]):
        dt = (current[1] - previous[1]) / 1000.0
        if dt <= 0.0:
            raise fail("non-positive prediction interval")
        speed = haversine(previous[2], previous[3], current[2], current[3], SPEED_RADIUS_M) / dt
        if not math.isfinite(speed):
            raise fail("non-finite prediction speed")
        speeds.append(speed)
    return {"transition_count": len(speeds), "max_speed_mps": max(speeds, default=0.0), "over_70_mps_count": sum(v > MAX_SPEED_MPS for v in speeds), "finite": all(math.isfinite(v) for v in speeds)}


def score(rows: list[tuple[str, int, float, float]], truth: dict[tuple[str, int], tuple[float, float]], domain: dict[str, Any], speed: dict[str, Any], route: str) -> dict[str, Any]:
    prediction = {(phone, timestamp): (lat, lon) for phone, timestamp, lat, lon in rows}
    keys = sorted(prediction, key=lambda key: key[1])
    errors = [haversine(*prediction[key], *truth[key]) for key in keys]
    p50, p95 = percentile(errors, 0.50), percentile(errors, 0.95)
    output = dict(domain)
    output.update({"mean_m": sum(errors) / len(errors), "p50_m": p50, "p95_m": p95, "max_m": max(errors), "score_m": (p50 + p95) / 2.0, "finite": all(math.isfinite(value) for value in errors), "over_70_mps_count": speed["over_70_mps_count"], "max_speed_mps": speed["max_speed_mps"], "speed": speed})
    if not output["finite"] or output["over_70_mps_count"] != 0:
        raise fail(f"score finite/speed gate failed: {route}")
    return output


def run_score(output_root: Path = DEFAULT_OUTPUT) -> dict[str, Any]:
    freeze = verify_freeze()
    historical = verify_historical_pins(freeze)
    manifest, manifest_hash = verify_manifest()
    output_root = output_root.resolve()
    reject_forbidden(output_root)
    if output_root.exists() and any(output_root.iterdir()):
        raise fail(f"refusing to overwrite nonempty Phase52 output: {output_root}")
    output_root.mkdir(parents=True, exist_ok=True)
    # Phase51 run1 artifact reads happen exactly once here.  Run2 hashes are
    # checked only by the Phase51 structural seal/result pin and are not read.
    artifact_data: dict[str, Any] = {}
    phase51_routes = freeze["immutable_phase51_outputs"]["routes"]
    for route in ROUTES:
        entry = phase51_routes[route]
        lanes: dict[str, Any] = {}
        for lane_name, lane_key, candidate in (("candidate", "candidate_run1", True), ("control", "control_run1", False)):
            lane = entry[lane_key]
            submission_path = ROOT / lane["submission"]["path"]
            summary_path = ROOT / lane["summary"]["path"]
            submission_payload, submission_hash = read_artifact_once(submission_path, f"Phase51 {lane_name} submission {route}", lane["submission"]["sha256"], int(lane["submission"]["bytes"]))
            summary_payload, summary_hash = read_artifact_once(summary_path, f"Phase51 {lane_name} summary {route}", lane["summary"]["sha256"], int(lane["summary"]["bytes"]))
            rows = parse_submission_once(submission_payload, route, f"Phase51 {lane_name} submission {route}")
            summary = parse_summary_once(summary_payload, route, candidate, f"Phase51 {lane_name} summary {route}")
            if len(rows) != int(lane["submission"]["rows"]):
                raise fail(f"Phase51 row count pin mismatch: {route}/{lane_name}")
            lanes[lane_name] = {"rows": rows, "summary": summary, "submission_sha256": submission_hash, "summary_sha256": summary_hash, "submission_bytes": len(submission_payload), "summary_bytes": len(summary_payload), "read_count": {"submission": 1, "summary": 1}}
        artifact_data[route] = lanes
    truth_reports: dict[str, Any] = {}
    truth_maps: dict[str, dict[tuple[str, int], tuple[float, float]]] = {}
    expected_truth = historical["phase44_result"].get("routes", {})
    expected_missing = freeze["domain_contract"]["warmup_exclusions"]["expected_missing_truth_rows"]
    for route in ROUTES:
        metadata = expected_truth.get(route, {}).get("truth", {})
        if not metadata:
            raise fail(f"Phase44 truth metadata missing: {route}")
        truth_maps[route], truth_reports[route] = read_truth_once(truth_paths()[route], route, str(metadata["sha256"]), int(metadata["bytes"]))
        if len(truth_maps[route]) != int(metadata["rows"]):
            raise fail(f"Phase44 truth row metadata mismatch: {route}")
    route_reports: dict[str, Any] = {}
    for route in ROUTES:
        truth = truth_maps[route]
        route_lanes: dict[str, Any] = {}
        for lane_name in ("candidate", "control"):
            rows = artifact_data[route][lane_name]["rows"]
            domain = check_domain(rows, truth, route, int(expected_missing[route]), f"{lane_name} domain")
            speed = speed_report(rows)
            route_lanes[lane_name] = {"domain": domain, "speed": speed, "metrics": score(rows, truth, domain, speed, route), "submission_sha256": artifact_data[route][lane_name]["submission_sha256"], "summary_sha256": artifact_data[route][lane_name]["summary_sha256"]}
        route_reports[route] = route_lanes
    baseline_scores = freeze.get("phase44_baseline_scores_m", {})
    if set(baseline_scores) != set(ROUTES):
        # The source Phase51 freeze is authoritative for these unchanged
        # scores; Phase52 freeze deliberately pins their origin, not a new
        # threshold or replacement value.
        p51_freeze, _, _ = load_json_once(ROOT / "docs/use_cases/records/smartphone_r5_phase51_android_sv_time_uncertainty_sigma_floor_freeze_v1.json", "Phase51 freeze")
        baseline_scores = p51_freeze.get("baseline_pins", {}).get("phase44_route_baseline_score_m", {})
    candidate_scores = [float(route_reports[route]["candidate"]["metrics"]["score_m"]) for route in ROUTES]
    control_scores = [float(route_reports[route]["control"]["metrics"]["score_m"]) for route in ROUTES]
    candidate_macro, control_macro = sum(candidate_scores) / 4.0, sum(control_scores) / 4.0
    failures: list[str] = []
    gates_by_route: dict[str, Any] = {}
    for route in ROUTES:
        candidate_metric = route_reports[route]["candidate"]["metrics"]
        control_metric = route_reports[route]["control"]["metrics"]
        improvement = float(control_metric["score_m"]) - float(candidate_metric["score_m"])
        route_failures: list[str] = []
        if improvement < 0.05:
            route_failures.append("improvement_below_0.05m")
        if improvement < 0.0:
            route_failures.append("route_regression")
        if candidate_metric["prediction_domain_coverage"] != 1.0:
            route_failures.append("prediction_domain_coverage")
        if candidate_metric["over_70_mps_count"] != 0:
            route_failures.append("over_70_mps")
        if candidate_metric["score_m"] > 3.0:
            route_failures.append("candidate_route_over_3m")
        if route == TARGET_ROUTE and candidate_metric["p95_m"] > 5.0:
            route_failures.append("mtv_h_p95_over_5m")
        gates_by_route[route] = {"candidate_score_m": candidate_metric["score_m"], "control_score_m": control_metric["score_m"], "phase44_baseline_score_m": baseline_scores.get(route), "improvement_m": improvement, "failures": route_failures, "passed": not route_failures}
        failures.extend(f"{route}:{item}" for item in route_failures)
    macro_improvement = control_macro - candidate_macro
    if macro_improvement < 0.10:
        failures.append("macro_improvement_below_0.10m")
    if candidate_macro > 2.0:
        failures.append("candidate_macro_over_2m")
    if max(candidate_scores) > 3.0:
        failures.append("candidate_route_over_3m")
    if route_reports[TARGET_ROUTE]["candidate"]["metrics"]["p95_m"] > 5.0:
        failures.append("mtv_h_p95_over_5m")
    result = {
        "schema_version": SCHEMA,
        "phase": 52,
        "execution_label": "Luna Max",
        "status": "go-eligible-separate-validation" if not failures else "no-go-accuracy-gates",
        "decision": "authorize-separate-fresh-validation-only" if not failures else "preserve-phase43-champion; keep-phase51-experimental",
        "freeze": {"path": relative(FREEZE), "sha256": FREEZE_SHA256},
        "evaluator_manifest": {"path": relative(MANIFEST), "sha256": manifest_hash},
        "phase51_result": {"path": relative(PHASE51_RESULT), "sha256": freeze["authority"]["phase51_result"]["sha256"]},
        "read_accounting": {"phase51_candidate_run1_submission_reads": 4, "phase51_control_run1_submission_reads": 4, "phase51_candidate_run1_summary_reads": 4, "phase51_control_run1_summary_reads": 4, "phase51_candidate_run2_reopened": 0, "phase51_control_other_runs_reopened": 0, "phase44_truth_reads": 4, "truth_read_count_per_route": 1, "single_process": True, "solver_process_invocations": 0, "archive_reopen_or_rematerialize": False, "validation_reads": 0, "holdout_reads": 0, "mat_read_or_generated": False, "wls_or_precomputed_coordinates": False, "kaggle_or_token_access": 0},
        "domain_contract": freeze["domain_contract"],
        "truth": truth_reports,
        "routes": route_reports,
        "aggregate": {"candidate_macro_score_m": candidate_macro, "control_macro_score_m": control_macro, "macro_improvement_m": macro_improvement, "phase44_baseline_macro_score_m": sum(float(baseline_scores[route]) for route in ROUTES) / 4.0, "candidate_scores_m": candidate_scores, "control_scores_m": control_scores, "stretch_target_m": STRETCH_TARGET_M, "stretch_target_reached": candidate_macro <= STRETCH_TARGET_M},
        "accuracy_gates": {"route_improvement_min_m": 0.05, "macro_improvement_min_m": 0.10, "mtv_h_improvement_min_m": 0.10, "absolute_candidate_macro_max_m": 2.0, "absolute_candidate_route_max_m": 3.0, "absolute_mtv_h_p95_max_m": 5.0, "routes": gates_by_route, "all_pass": not failures, "failures": failures, "fresh_validation": "prohibited in Phase52; separate phase only if all absolute gates pass", "post_score_tuning_or_rerun": False},
        "phase51_option": {"status": "experimental-preserved", "formula": "max(existing_sigma_m, 299792458.0 * ReceivedSvTimeUncertaintyNanos * 1e-9)", "spp_applied": False},
        "next_factor_if_no_go": "raw Android per-satellite carrier-phase ADR carrier-frequency/antenna phase-bias residual"
    }
    atomic_json(output_root / RESULT_NAME, result)
    result_hash = read_bytes_once(output_root / RESULT_NAME, "Phase52 result")[1]
    artifact_manifest = {"schema_version": "smartphone-r5-phase52-android-sv-time-uncertainty-integrity-recovery-artifact-manifest.v1", "phase": 52, "result": {"path": relative(output_root / RESULT_NAME), "sha256": result_hash}, "freeze": {"path": relative(FREEZE), "sha256": FREEZE_SHA256}, "evaluator_manifest": {"path": relative(MANIFEST), "sha256": manifest_hash}, "phase51_candidate_control_run1_only": True, "phase51_candidate_run2_reopened": 0, "truth_reads": 4, "solver_process_invocations": 0}
    atomic_json(output_root / ARTIFACT_MANIFEST_NAME, artifact_manifest)
    return result


def write_failure(output_root: Path, message: str) -> None:
    try:
        output_root.mkdir(parents=True, exist_ok=True)
        atomic_json(output_root / FAILURE_NAME, {"schema_version": SCHEMA, "phase": 52, "status": "integrity-recovery-failure", "message": message, "truth_reads": 0, "solver_process_invocations": 0})
    except OSError:
        pass


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--verify-freeze", action="store_true", help="verify records/manifest without Phase51 outputs or truth")
    parser.add_argument("--score", action="store_true", help="read pinned run1 artifacts/truth once and score")
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()
    if args.verify_freeze == args.score:
        parser.error("choose exactly one of --verify-freeze or --score")
    try:
        if args.verify_freeze:
            verify_freeze()
            verify_manifest()
            print("phase52 freeze/evaluator manifest: verified without Phase51 outputs/truth")
            return 0
        result = run_score(args.output)
        print(json.dumps({"status": result["status"], "all_accuracy_gates_pass": result["accuracy_gates"]["all_pass"], "candidate_macro_score_m": result["aggregate"]["candidate_macro_score_m"], "control_macro_score_m": result["aggregate"]["control_macro_score_m"], "output": relative(args.output.resolve())}, indent=2))
        return 0
    except (OSError, Phase52Error) as exc:
        if args.score:
            write_failure(args.output.resolve(), str(exc))
        print(f"phase52: {exc}", file=os.sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
