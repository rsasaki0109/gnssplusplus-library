#!/usr/bin/env python3
"""Diagnose identifiability of the sealed Phase44 Pixel5 residual.

This command is deliberately an evaluation-only, one-shot diagnostic.  It
reads each sealed Phase44 candidate submission, each already materialized
Phase44 truth CSV, and each pinned raw ``device_imu.csv`` exactly once in one
process.  It never opens the dataset archive, reruns native inference,
creates corrected coordinates, or fits a truth-derived correction.

The residual is the observable prediction-minus-truth horizontal vector in a
fixed local ENU frame.  Route medians, robust scales, covariance, temporal
segments, raw-only orientation labels, prediction-only speed bins, and
leave-one-route-out common-median scores are reported as diagnostics.  A
leave-one-route-out or full-cohort median is never a deployable estimator.
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
import time
from typing import Any, Iterable


ROOT = Path(__file__).resolve().parents[3]
FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_phase45_pixel5_residual_diagnostic_freeze_v3.json"
FREEZE_SHA256 = "7dc39fb43f9761ab5bbb12fa2d9992ec02e12d026d18072a0a72b51e8e4665ed"
FREEZE_V1 = ROOT / "docs/use_cases/records/smartphone_r5_phase45_pixel5_residual_diagnostic_freeze_v1.json"
FREEZE_V1_SHA256 = "d99ce86eeb499d4a08746f9f913eeef7ae1b5f7bcfb0e1fe906fdc453b982d6"
FREEZE_V2 = ROOT / "docs/use_cases/records/smartphone_r5_phase45_pixel5_residual_diagnostic_freeze_v2.json"
FREEZE_V2_SHA256 = "2aa1c9e44b052ba2b8915d734cb930bf4fe02c6cbab53dfa6f8adaf764697f0c"
EVALUATOR_MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase45_pixel5_residual_diagnostic_evaluator_manifest_v1.json"
PHASE44_FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_phase44_pixel5_development_accuracy_freeze_v1.json"
PHASE44_FREEZE_SHA256 = "95c0990af0015b7cb5fcf736aefbcff6fc97356093edcf03094b31b4083b28bc"
PHASE44_RESULT_RECORD = ROOT / "docs/use_cases/records/smartphone_r5_phase44_pixel5_development_accuracy_result_v1.json"
PHASE44_RESULT_RECORD_SHA256 = "9e441c78f7c2bf8b3cf2cf9a7c9fe7e447fb2ff8eb6324fa5ef138c3b419e48a"
PHASE44_RESULT_OUTPUT = ROOT / "output/smartphone-r5/phase44-pixel5-development-accuracy-v1/phase44_pixel5_development_accuracy.json"
PHASE44_RESULT_OUTPUT_SHA256 = "a191e5910315337ebd0d752f61eaddcdaa77f61fe5c691f5858981bea39cfa32"
PHASE44_RESULT_MANIFEST = ROOT / "output/smartphone-r5/phase44-pixel5-development-accuracy-v1/phase44_pixel5_development_accuracy.manifest.json"
PHASE44_RESULT_MANIFEST_SHA256 = "5a3cd5d167063e7b1727efc67868fd6e25d26464ccc043b3969e31c9ffccb1e9"
PHASE44_TRUTH_MATERIALIZATION = ROOT / "output/smartphone-r5/phase44-pixel5-development-accuracy-v1/truth_materialization.json"
PHASE44_TRUTH_MATERIALIZATION_SHA256 = "1de76b17f728e239cdf0416aafbdf59e07497c2bc00a1de88d808f7044f7a3bc"
PHASE43_SEAL = ROOT / "output/smartphone-r5/phase43-native-fallback-seed-quality-anchor-recovery-v1/phase43_structural_seal.json"
PHASE43_SEAL_SHA256 = "fdeaf672b015cae99dfdf8351a5e7a92ca2d37bcdb0872c5c3fc5b937416b64d"
PHASE37_FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_phase37_pixel5_repeatability_freeze_v2.json"
PHASE37_FREEZE_SHA256 = "61f46ac734d0c712bc317603570a0630234ee67ab642d12368eb5e3bf3962a3e"
DEFAULT_OUTPUT = ROOT / "output/smartphone-r5/phase45-pixel5-residual-diagnostic-v1"

SCHEMA = "smartphone-r5-phase45-pixel5-residual-diagnostic.v1"
MANIFEST_SCHEMA = "smartphone-r5-phase45-pixel5-residual-diagnostic-manifest.v1"
FREEZE_SCHEMA = "smartphone-r5-phase45-pixel5-residual-diagnostic-freeze.v3"
ROUTES = (
    "2021-03-16-18-59-us-ca-mtv-a/pixel5",
    "2021-08-24-20-32-us-ca-mtv-h/pixel5",
    "2022-04-01-18-22-us-ca-lax-t/pixel5",
    "2023-03-08-21-34-us-ca-mtv-u/pixel5",
)
TARGET = ROUTES[1]
FRESH_VALIDATION = "2023-05-09-21-32-us-ca-mtv-pe1/pixel5"
FUTURE_HOLDOUT = "2023-05-16-19-54-us-ca-mtv-xe1/pixel5"
IMU_MAX_AGE_MS = 250
WGS84_A = 6378137.0
WGS84_F = 1.0 / 298.257223563
WGS84_B = WGS84_A * (1.0 - WGS84_F)
GRAVITY_MIN_MPS2 = 5.0
GRAVITY_MAX_MPS2 = 15.0
EPS = 1e-12
MIN_ORIENTATION_ROWS = 20
MAX_ORIENTATION_DELTA_M = 0.75
MAX_SPEED_MPS = 70.0


class Phase45Error(ValueError):
    """Raised when an immutable Phase45 contract cannot be proved."""


def _fail(message: str) -> Phase45Error:
    return Phase45Error(message)


def _relative(path: Path) -> str:
    try:
        return str(path.relative_to(ROOT))
    except ValueError:
        return str(path)


def _reject_path(path: Path | str) -> None:
    token = str(path).lower()
    forbidden = (".mat", "validation", "holdout", "precomputed", "device_wls", "kaggle", "token")
    if any(part in token for part in forbidden):
        raise _fail(f"forbidden input/output path: {path}")


def _sha256_bytes(payload: bytes) -> str:
    return hashlib.sha256(payload).hexdigest()


def _read_bytes_once(path: Path, label: str, expected_sha256: str | None = None) -> tuple[bytes, str]:
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


def _atomic_json(path: Path, value: dict[str, Any]) -> None:
    _atomic_write(path, (json.dumps(value, indent=2, sort_keys=True) + "\n").encode("utf-8"))


def _verify_freeze(path: Path = FREEZE) -> dict[str, Any]:
    freeze, digest, _ = _load_json_once(path, "Phase45 freeze")
    if path == FREEZE and digest != FREEZE_SHA256:
        raise _fail("Phase45 v3 freeze hash changed")
    if freeze.get("schema_version") != FREEZE_SCHEMA or freeze.get("status") != "frozen-before-phase45-truth-read":
        raise _fail("Phase45 v3 freeze schema/status mismatch")
    supersedes = freeze.get("supersedes")
    if not isinstance(supersedes, dict):
        raise _fail("Phase45 v3 supersession pins missing")
    v1 = supersedes.get("path")
    if v1 != _relative(FREEZE_V1) or supersedes.get("sha256") != FREEZE_V1_SHA256:
        raise _fail("Phase45 v1 supersession pin mismatch")
    v2 = supersedes.get("v2")
    if not isinstance(v2, dict) or v2.get("path") != _relative(FREEZE_V2) or v2.get("sha256") != FREEZE_V2_SHA256:
        raise _fail("Phase45 v2 supersession pin mismatch")
    objective = freeze.get("objective")
    if not isinstance(objective, dict) or objective.get("exact_model") != "pixel5" or objective.get("route_count_required") != 4:
        raise _fail("Phase45 objective changed")
    if any(objective.get(key) is not False for key in ("solver_rerun", "correction_fit_or_application", "native_candidate_mutation")):
        raise _fail("Phase45 objective permits mutation or truth-derived correction")
    cohort = freeze.get("cohort")
    if not isinstance(cohort, dict) or cohort.get("phone_model") != "pixel5" or cohort.get("route_disjoint") is not True:
        raise _fail("Phase45 cohort changed")
    if tuple(cohort.get("route_order", ())) != ROUTES:
        raise _fail("Phase45 route order changed")
    if cohort.get("fresh_validation", {}).get("id") != FRESH_VALIDATION or cohort.get("future_holdout", {}).get("id") != FUTURE_HOLDOUT:
        raise _fail("Phase45 validation/holdout identity changed")
    if cohort.get("fresh_validation", {}).get("remains_sealed") is not True or cohort.get("future_holdout", {}).get("remains_sealed") is not True:
        raise _fail("Phase45 validation/holdout is not sealed")
    reads = freeze.get("read_contract")
    expected_reads = {
        "single_process": True,
        "truth_read_count_per_route": 1,
        "candidate_read_count_per_route": 1,
        "raw_imu_read_count_per_route": 1,
        "phase44_materialized_truth_reused": True,
        "archive_open": False,
        "archive_materialization": False,
        "truth_rematerialization": False,
        "solver_rerun": False,
        "candidate_regeneration": False,
        "truth_derived_correction_fit": False,
        "truth_derived_correction_application": False,
        "post_truth_tuning": False,
        "validation_truth_open_count": 0,
        "holdout_truth_open_count": 0,
        "kaggle_or_token_access": False,
        "mat_read_or_generated": False,
        "device_wls_coordinates_used": False,
        "precomputed_inference_coordinates_used": False,
        "atomic_result_publish": True,
    }
    if not isinstance(reads, dict) or any(reads.get(key) != value for key, value in expected_reads.items()):
        raise _fail("Phase45 read contract is not closed")
    warmup = freeze.get("warmup_contract")
    if not isinstance(warmup, dict) or warmup.get("warmup_epoch_excluded") is not True:
        raise _fail("Phase45 warm-up contract changed")
    gates = freeze.get("identifiability_gates")
    if not isinstance(gates, dict) or gates.get("logic", "").find("AND") < 0:
        raise _fail("Phase45 identifiability gate logic changed")
    exact = gates.get("exact_model_route_count")
    if not isinstance(exact, dict) or exact.get("minimum") != 4 or exact.get("prediction_domain_coverage_required") is not True:
        raise _fail("Phase45 exact-model gate changed")
    if exact.get("truth_row_coverage_informational_only") is not True or exact.get("truth_row_coverage_does_not_gate_prediction_domain") is not True:
        raise _fail("Phase45 truth-row coverage was made a prediction gate")
    if "truth_row_coverage 1.0" in str(exact.get("criterion", "")):
        raise _fail("Phase45 exact-model criterion still gates on truth-row coverage")
    dispersion = gates.get("route_median_center_dispersion")
    if not isinstance(dispersion, dict) or dispersion.get("component_mad_max_m") != {"east": 0.5, "north": 0.5} or dispersion.get("center_radius_max_m") != 1.0:
        raise _fail("Phase45 route-center gate changed")
    prefix = gates.get("prefix_tail_stability")
    if not isinstance(prefix, dict) or prefix.get("absolute_floor_m") != 1.0 or prefix.get("route_mad_factor") != 2.0:
        raise _fail("Phase45 prefix-tail gate changed")
    orientation = gates.get("orientation_independence")
    if not isinstance(orientation, dict) or orientation.get("minimum_rows_per_group") != 20 or orientation.get("max_group_median_delta_m") != 0.75:
        raise _fail("Phase45 orientation gate changed")
    loo = gates.get("loo_all_route_improvement")
    macro = gates.get("loo_macro_improvement")
    worsen = gates.get("individual_worsening_limit")
    stretch = gates.get("stretch_0_782_reachability")
    if not isinstance(loo, dict) or loo.get("minimum_score_improvement_m") != 0.05:
        raise _fail("Phase45 LOO route gate changed")
    if not isinstance(macro, dict) or macro.get("minimum_macro_improvement_m") != 0.05:
        raise _fail("Phase45 LOO macro gate changed")
    if not isinstance(worsen, dict) or worsen.get("maximum_score_worsening_m") != 0.10:
        raise _fail("Phase45 worsening gate changed")
    if not isinstance(stretch, dict) or stretch.get("full_cohort_common_median_macro_max_m") != 0.782 or stretch.get("full_cohort_target_route_max_m") != 0.782:
        raise _fail("Phase45 stretch gate changed")
    return freeze


def _verify_manifest() -> dict[str, Any]:
    manifest, digest, _ = _load_json_once(EVALUATOR_MANIFEST, "Phase45 evaluator manifest")
    if manifest.get("schema_version") != MANIFEST_SCHEMA or manifest.get("status") != "evaluator-frozen-before-phase45-truth":
        raise _fail("Phase45 evaluator manifest schema/status mismatch")
    freeze = manifest.get("freeze")
    if not isinstance(freeze, dict) or freeze.get("path") != _relative(FREEZE) or freeze.get("sha256") != FREEZE_SHA256:
        raise _fail("Phase45 evaluator manifest freeze pin mismatch")
    source = manifest.get("evaluator", {}).get("source")
    test = manifest.get("evaluator", {}).get("test")
    if not isinstance(source, dict) or source.get("path") != _relative(Path(__file__)):
        raise _fail("Phase45 source pin missing")
    if _hash_path(Path(__file__), "Phase45 evaluator source") != source.get("sha256"):
        raise _fail("Phase45 evaluator source hash mismatch")
    if not isinstance(test, dict):
        raise _fail("Phase45 focused test pin missing")
    test_path = ROOT / str(test.get("path", ""))
    if _hash_path(test_path, "Phase45 focused test") != test.get("sha256"):
        raise _fail("Phase45 focused test hash mismatch")
    policy = manifest.get("truth_policy")
    if not isinstance(policy, dict) or policy.get("truth_open_count_before_manifest") != 0 or policy.get("truth_read_count_per_route") != 1:
        raise _fail("Phase45 manifest is not pre-truth or has a repeated truth read")
    if any(policy.get(key) != 0 for key in ("validation_truth_open_count", "holdout_truth_open_count", "mat_read_or_generated", "device_wls_coordinates_used", "precomputed_inference_coordinates_used")):
        raise _fail("Phase45 manifest opens forbidden truth or coordinate inputs")
    if policy.get("solver_rerun") is not False or policy.get("truth_derived_correction_fit") is not False or policy.get("truth_derived_correction_application") is not False:
        raise _fail("Phase45 manifest permits inference or correction")
    manifest["_sha256"] = digest
    return manifest


def _verify_phase44_authority() -> dict[str, Any]:
    phase44_freeze, freeze_hash, _ = _load_json_once(PHASE44_FREEZE, "Phase44 freeze", PHASE44_FREEZE_SHA256)
    result_record, result_hash, _ = _load_json_once(PHASE44_RESULT_RECORD, "Phase44 result record", PHASE44_RESULT_RECORD_SHA256)
    result_output, result_output_hash, _ = _load_json_once(PHASE44_RESULT_OUTPUT, "Phase44 result output", PHASE44_RESULT_OUTPUT_SHA256)
    result_manifest, result_manifest_hash, _ = _load_json_once(PHASE44_RESULT_MANIFEST, "Phase44 result manifest", PHASE44_RESULT_MANIFEST_SHA256)
    materialization, materialization_hash, _ = _load_json_once(PHASE44_TRUTH_MATERIALIZATION, "Phase44 truth materialization", PHASE44_TRUTH_MATERIALIZATION_SHA256)
    seal, seal_hash, _ = _load_json_once(PHASE43_SEAL, "Phase43 structural seal", PHASE43_SEAL_SHA256)
    phase37, phase37_hash, _ = _load_json_once(PHASE37_FREEZE, "Phase37 freeze", PHASE37_FREEZE_SHA256)
    if phase44_freeze.get("phase") != 44 or result_record.get("phase") != 44 or result_output.get("phase") != 44:
        raise _fail("Phase44 authority record changed")
    if result_record.get("status") != "no-go-development-absolute-gate" or result_output.get("status") != "no-go-development-absolute-gate":
        raise _fail("Phase44 result is not the sealed no-go")
    if result_manifest.get("truth_open_count") != 4 or materialization.get("truth_open_count") != 4:
        raise _fail("Phase44 truth accounting changed")
    if result_manifest.get("validation_truth_open_count") != 0 or result_manifest.get("future_holdout_truth_open_count") != 0:
        raise _fail("Phase44 validation/holdout was opened")
    if seal.get("status") != "sealed-truth-free-structural-go" or seal.get("truth_accounting", {}).get("truth_open_count") != 0:
        raise _fail("Phase43 structural seal changed")
    if phase37.get("schema_version") != "smartphone-r5-phase37-pixel5-repeatability-freeze.v2":
        raise _fail("Phase37 authority freeze changed")
    return {
        "phase44_freeze": {"path": _relative(PHASE44_FREEZE), "sha256": freeze_hash},
        "phase44_result_record": {"path": _relative(PHASE44_RESULT_RECORD), "sha256": result_hash},
        "phase44_result_output": {"path": _relative(PHASE44_RESULT_OUTPUT), "sha256": result_output_hash},
        "phase44_result_manifest": {"path": _relative(PHASE44_RESULT_MANIFEST), "sha256": result_manifest_hash},
        "phase44_truth_materialization": {"path": _relative(PHASE44_TRUTH_MATERIALIZATION), "sha256": materialization_hash},
        "phase43_structural_seal": {"path": _relative(PHASE43_SEAL), "sha256": seal_hash},
        "phase37_freeze": {"path": _relative(PHASE37_FREEZE), "sha256": phase37_hash},
    }


def _parse_float(raw: str | None, field: str, row_number: int) -> float:
    try:
        value = float((raw or "").strip())
    except ValueError as exc:
        raise _fail(f"row {row_number}: invalid {field}") from exc
    if not math.isfinite(value):
        raise _fail(f"row {row_number}: non-finite {field}")
    return value


def _parse_int(raw: str | None, field: str, row_number: int) -> int:
    try:
        value = int((raw or "").strip())
    except ValueError as exc:
        raise _fail(f"row {row_number}: invalid {field}") from exc
    if value < 0:
        raise _fail(f"row {row_number}: negative {field}")
    return value


def _csv_reader(payload: bytes, label: str) -> csv.DictReader:
    try:
        text = payload.decode("utf-8-sig")
    except UnicodeDecodeError as exc:
        raise _fail(f"{label} is not UTF-8 CSV") from exc
    return csv.DictReader(io.StringIO(text, newline=""))


def _parse_candidate(payload: bytes, expected_phone: str, label: str) -> list[dict[str, Any]]:
    reader = _csv_reader(payload, label)
    fields = list(reader.fieldnames or ())
    expected = ["phone", "UnixTimeMillis", "LatitudeDegrees", "LongitudeDegrees"]
    if fields != expected:
        raise _fail(f"{label} header must be exactly {','.join(expected)}")
    rows: list[dict[str, Any]] = []
    seen: set[tuple[str, int]] = set()
    previous: int | None = None
    for line, raw in enumerate(reader, start=2):
        if None in raw:
            raise _fail(f"{label} row {line}: extra columns")
        phone = raw.get("phone") or ""
        if phone != expected_phone or phone != phone.strip():
            raise _fail(f"{label} row {line}: phone mismatch")
        timestamp = _parse_int(raw.get("UnixTimeMillis"), "UnixTimeMillis", line)
        if previous is not None and timestamp <= previous:
            raise _fail(f"{label} row {line}: timestamps are not strictly increasing")
        latitude = _parse_float(raw.get("LatitudeDegrees"), "LatitudeDegrees", line)
        longitude = _parse_float(raw.get("LongitudeDegrees"), "LongitudeDegrees", line)
        if not -90.0 <= latitude <= 90.0 or not -180.0 <= longitude <= 180.0:
            raise _fail(f"{label} row {line}: coordinate out of range")
        key = (phone, timestamp)
        if key in seen:
            raise _fail(f"{label} row {line}: duplicate key")
        seen.add(key)
        rows.append({"phone": phone, "timestamp": timestamp, "lat": latitude, "lon": longitude})
        previous = timestamp
    if not rows:
        raise _fail(f"{label} is empty")
    return rows


def _parse_truth(payload: bytes, expected_phone: str, label: str) -> list[dict[str, Any]]:
    reader = _csv_reader(payload, label)
    fields = list(reader.fieldnames or ())
    required = ("UnixTimeMillis", "LatitudeDegrees", "LongitudeDegrees", "AltitudeMeters")
    if any(name not in fields for name in required):
        raise _fail(f"{label} missing required fields")
    has_phone = "phone" in fields
    rows: list[dict[str, Any]] = []
    seen: set[tuple[str, int]] = set()
    previous: int | None = None
    for line, raw in enumerate(reader, start=2):
        if None in raw:
            raise _fail(f"{label} row {line}: extra columns")
        phone = (raw.get("phone") or expected_phone) if has_phone else expected_phone
        if phone != expected_phone or phone != phone.strip():
            raise _fail(f"{label} row {line}: phone mismatch")
        timestamp = _parse_int(raw.get("UnixTimeMillis"), "UnixTimeMillis", line)
        if previous is not None and timestamp <= previous:
            raise _fail(f"{label} row {line}: timestamps are not strictly increasing")
        latitude = _parse_float(raw.get("LatitudeDegrees"), "LatitudeDegrees", line)
        longitude = _parse_float(raw.get("LongitudeDegrees"), "LongitudeDegrees", line)
        altitude = _parse_float(raw.get("AltitudeMeters"), "AltitudeMeters", line)
        if not -90.0 <= latitude <= 90.0 or not -180.0 <= longitude <= 180.0:
            raise _fail(f"{label} row {line}: coordinate out of range")
        key = (phone, timestamp)
        if key in seen:
            raise _fail(f"{label} row {line}: duplicate key")
        seen.add(key)
        rows.append({"phone": phone, "timestamp": timestamp, "lat": latitude, "lon": longitude, "alt": altitude})
        previous = timestamp
    if not rows:
        raise _fail(f"{label} is empty")
    return rows


def _parse_raw_imu(payload: bytes, label: str) -> tuple[dict[str, list[tuple[int, float, float, float]]], dict[str, int]]:
    reader = _csv_reader(payload, label)
    fields = list(reader.fieldnames or ())
    required = {"MessageType", "utcTimeMillis", "MeasurementX", "MeasurementY", "MeasurementZ"}
    if not required.issubset(fields):
        raise _fail(f"{label} missing raw IMU fields")
    samples: dict[str, list[tuple[int, float, float, float]]] = {"UncalAccel": [], "UncalMag": []}
    counts: dict[str, int] = {}
    for line, raw in enumerate(reader, start=2):
        if None in raw:
            raise _fail(f"{label} row {line}: extra columns")
        message_type = (raw.get("MessageType") or "").strip()
        counts[message_type] = counts.get(message_type, 0) + 1
        if message_type not in samples:
            continue
        timestamp = _parse_int(raw.get("utcTimeMillis"), "utcTimeMillis", line)
        x = _parse_float(raw.get("MeasurementX"), "MeasurementX", line)
        y = _parse_float(raw.get("MeasurementY"), "MeasurementY", line)
        z = _parse_float(raw.get("MeasurementZ"), "MeasurementZ", line)
        samples[message_type].append((timestamp, x, y, z))
    for message_type in samples:
        samples[message_type].sort(key=lambda item: item[0])
    return samples, counts


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
    # The submission has no altitude.  Truth altitude is used for both ECEF
    # calls only as a horizontal lat/lon projection; no up residual exists.
    predicted = _ecef(predicted_lat, predicted_lon, truth_alt)
    actual = _ecef(truth_lat, truth_lon, truth_alt)
    dx, dy, dz = (predicted[i] - actual[i] for i in range(3))
    lat = math.radians(origin_lat)
    lon = math.radians(origin_lon)
    east = -math.sin(lon) * dx + math.cos(lon) * dy
    north = -math.sin(lat) * math.cos(lon) * dx - math.sin(lat) * math.sin(lon) * dy + math.cos(lat) * dz
    _ = origin_ecef
    return east, north


def _haversine(latitude_a: float, longitude_a: float, latitude_b: float, longitude_b: float) -> float:
    radius = 6_371_008.8
    lat_a = math.radians(latitude_a)
    lat_b = math.radians(latitude_b)
    delta_lat = math.radians(latitude_b - latitude_a)
    delta_lon = math.radians(longitude_b - longitude_a)
    value = math.sin(delta_lat / 2.0) ** 2 + math.cos(lat_a) * math.cos(lat_b) * math.sin(delta_lon / 2.0) ** 2
    return 2.0 * radius * math.asin(math.sqrt(min(1.0, max(0.0, value))))


def _percentile(values: list[float], fraction: float) -> float:
    if not values:
        raise _fail("percentile of empty set")
    ordered = sorted(values)
    rank = (len(ordered) - 1) * fraction
    low = math.floor(rank)
    high = math.ceil(rank)
    if low == high:
        return ordered[low]
    weight = rank - low
    return ordered[low] * (1.0 - weight) + ordered[high] * weight


def _median(values: Iterable[float]) -> float:
    values_list = list(values)
    if not values_list:
        raise _fail("median of empty set")
    return float(statistics.median(values_list))


def _vector_median(vectors: list[tuple[float, float]]) -> tuple[float, float]:
    return _median(vector[0] for vector in vectors), _median(vector[1] for vector in vectors)


def _covariance(vectors: list[tuple[float, float]]) -> list[list[float]]:
    if len(vectors) < 2:
        return [[0.0, 0.0], [0.0, 0.0]]
    mean_e = sum(vector[0] for vector in vectors) / len(vectors)
    mean_n = sum(vector[1] for vector in vectors) / len(vectors)
    divisor = float(len(vectors) - 1)
    return [
        [sum((e - mean_e) ** 2 for e, _ in vectors) / divisor, sum((e - mean_e) * (n - mean_n) for e, n in vectors) / divisor],
        [sum((n - mean_n) * (e - mean_e) for e, n in vectors) / divisor, sum((n - mean_n) ** 2 for _, n in vectors) / divisor],
    ]


def _inliers(vectors: list[tuple[float, float]], med: tuple[float, float], mad: tuple[float, float]) -> list[tuple[float, float]]:
    kept: list[tuple[float, float]] = []
    for vector in vectors:
        good = True
        for index in range(2):
            delta = abs(vector[index] - med[index])
            good = good and (delta <= EPS if mad[index] <= EPS else delta <= 3.0 * mad[index])
        if good:
            kept.append(vector)
    return kept


def _summary(vectors: list[tuple[float, float]]) -> dict[str, Any]:
    if not vectors:
        return {"count": 0}
    med = _vector_median(vectors)
    mad = (_median(abs(vector[0] - med[0]) for vector in vectors), _median(abs(vector[1] - med[1]) for vector in vectors))
    mean = (sum(vector[0] for vector in vectors) / len(vectors), sum(vector[1] for vector in vectors) / len(vectors))
    inlier = _inliers(vectors, med, mad)
    norms = [math.hypot(*vector) for vector in vectors]
    return {
        "count": len(vectors),
        "median_enu_m": {"east": med[0], "north": med[1], "up": None},
        "mad_enu_m": {"east": mad[0], "north": mad[1], "up": None},
        "mean_enu_m": {"east": mean[0], "north": mean[1], "up": None},
        "rms_horizontal_m": math.sqrt(sum(value * value for value in norms) / len(norms)),
        "median_horizontal_m": _median(norms),
        "p95_horizontal_m": _percentile(norms, 0.95),
        "covariance_enu_m2": _covariance(vectors),
        "component_3mad_inlier_count": len(inlier),
        "component_3mad_inlier_fraction": len(inlier) / len(vectors),
        "component_3mad_inlier_covariance_enu_m2": _covariance(inlier),
    }


def _score_vectors(vectors: list[tuple[float, float]], offset: tuple[float, float] = (0.0, 0.0)) -> dict[str, float]:
    norms = [math.hypot(vector[0] - offset[0], vector[1] - offset[1]) for vector in vectors]
    p50 = _percentile(norms, 0.50)
    p95 = _percentile(norms, 0.95)
    return {"p50_m": p50, "p95_m": p95, "kaggle_score_m": (p50 + p95) / 2.0}


def _sample_for_time(samples: list[tuple[int, float, float, float]], timestamp: int) -> tuple[float, float, float] | None:
    if not samples:
        return None
    times = [sample[0] for sample in samples]
    index = bisect.bisect_right(times, timestamp) - 1
    if index < 0 or timestamp - samples[index][0] > IMU_MAX_AGE_MS:
        return None
    return samples[index][1:]


def _orientation_label(
    accel: tuple[float, float, float] | None,
    magnetometer: tuple[float, float, float] | None,
) -> tuple[str, str, str]:
    if accel is None:
        return "unavailable", "unavailable", "unavailable"
    ax, ay, az = accel
    norm = math.sqrt(ax * ax + ay * ay + az * az)
    if not math.isfinite(norm) or norm < EPS:
        return "invalid", "invalid", "unavailable"
    values = (abs(ax), abs(ay), abs(az))
    axis = max(range(3), key=values.__getitem__)
    axis_names = ("x", "y", "z")
    gravity_axis = axis_names[axis] + ("+" if (ax, ay, az)[axis] >= 0.0 else "-") if values[axis] / norm >= 0.75 else "mixed"
    if not GRAVITY_MIN_MPS2 <= norm <= GRAVITY_MAX_MPS2:
        return gravity_axis, "dynamic", "unavailable"
    if magnetometer is None:
        return gravity_axis, "gravity_valid", "unavailable"
    mx, my, mz = magnetometer
    mag_norm = math.sqrt(mx * mx + my * my + mz * mz)
    if not math.isfinite(mag_norm) or mag_norm < EPS:
        return gravity_axis, "gravity_valid", "unavailable"
    pitch = math.asin(max(-1.0, min(1.0, -ax / norm)))
    roll = math.atan2(ay, az)
    horizontal_x = mx * math.cos(pitch) + mz * math.sin(pitch)
    horizontal_y = mx * math.sin(roll) * math.sin(pitch) + my * math.cos(roll) - mz * math.sin(roll) * math.cos(pitch)
    if abs(horizontal_x) + abs(horizontal_y) < EPS:
        return gravity_axis, "gravity_valid", "unavailable"
    heading = math.atan2(-horizontal_y, horizontal_x) % (2.0 * math.pi)
    return gravity_axis, "gravity_valid", f"octant_{int(math.floor(heading / (2.0 * math.pi / 8.0)))}"


def _group_orientation(vectors: list[tuple[float, float]], labels: list[tuple[str, str, str]], index: int) -> dict[str, Any]:
    groups: dict[str, list[tuple[float, float]]] = {}
    for vector, label in zip(vectors, labels):
        groups.setdefault(label[index], []).append(vector)
    summaries = {name: _summary(values) for name, values in sorted(groups.items())}
    overall = _summary(vectors)
    overall_med = overall["median_enu_m"]
    deltas = {
        name: math.hypot(summary["median_enu_m"]["east"] - overall_med["east"], summary["median_enu_m"]["north"] - overall_med["north"])
        for name, summary in summaries.items()
        if name not in {"unavailable", "invalid"} and summary.get("count", 0) >= MIN_ORIENTATION_ROWS
    }
    comparable = [summary for name, summary in summaries.items() if name not in {"unavailable", "invalid"} and summary.get("count", 0) >= MIN_ORIENTATION_ROWS]
    return {
        "groups": summaries,
        "groups_with_at_least_20": sum(summary.get("count", 0) >= MIN_ORIENTATION_ROWS for summary in summaries.values()),
        "usable_groups_with_at_least_20": len(comparable),
        "max_group_median_delta_from_overall_m": max(deltas.values(), default=None),
        "group_median_delta_from_overall_m": deltas,
    }


def _speed_bin(speed: float | None) -> str:
    if speed is None:
        return "unavailable"
    if speed <= 1.0:
        return "<=1"
    if speed <= 5.0:
        return ">1<=5"
    if speed <= 15.0:
        return ">5<=15"
    return ">15"


def _prediction_speeds(candidate: list[dict[str, Any]]) -> tuple[dict[tuple[str, int], str], dict[str, Any]]:
    bins: dict[str, list[tuple[float, float]]] = {name: [] for name in ("unavailable", "<=1", ">1<=5", ">5<=15", ">15")}
    labels: dict[tuple[str, int], str] = {}
    speeds: list[float] = []
    for index, row in enumerate(candidate):
        key = (row["phone"], row["timestamp"])
        speed: float | None = None
        if index:
            previous = candidate[index - 1]
            delta_ms = row["timestamp"] - previous["timestamp"]
            if delta_ms <= 0:
                raise _fail("candidate time transition is not positive")
            speed = _haversine(previous["lat"], previous["lon"], row["lat"], row["lon"]) / (delta_ms / 1000.0)
            if not math.isfinite(speed):
                raise _fail("candidate speed is non-finite")
            speeds.append(speed)
        labels[key] = _speed_bin(speed)
    return labels, {
        "source": "sealed candidate consecutive-row Haversine speed; truth speed not read or used",
        "bins_mps": ["unavailable", "<=1", ">1<=5", ">5<=15", ">15"],
        "transition_count": len(speeds),
        "max_speed_mps": max(speeds, default=0.0),
        "over_70_mps_count": sum(speed > MAX_SPEED_MPS for speed in speeds),
        "counts": {name: list(labels.values()).count(name) for name in bins},
    }


def _time_drift(records: list[dict[str, Any]]) -> dict[str, Any]:
    if len(records) < 2:
        return {"normalized_time": True, "east_slope_m": 0.0, "north_slope_m": 0.0, "endpoint_drift_enu_m": {"east": 0.0, "north": 0.0, "up": None}}
    times = [record["timestamp"] for record in records]
    start, end = times[0], times[-1]
    span = float(end - start)
    normalized = [0.0 if span <= 0.0 else (timestamp - start) / span for timestamp in times]
    mean_t = sum(normalized) / len(normalized)
    mean_e = sum(record["vector"][0] for record in records) / len(records)
    mean_n = sum(record["vector"][1] for record in records) / len(records)
    denominator = sum((value - mean_t) ** 2 for value in normalized)
    if denominator <= EPS:
        slope_e = slope_n = 0.0
    else:
        slope_e = sum((value - mean_t) * (record["vector"][0] - mean_e) for value, record in zip(normalized, records)) / denominator
        slope_n = sum((value - mean_t) * (record["vector"][1] - mean_n) for value, record in zip(normalized, records)) / denominator
    return {
        "normalized_time": True,
        "time_start_unix_ms": start,
        "time_end_unix_ms": end,
        "east_slope_m_per_normalized_time": slope_e,
        "north_slope_m_per_normalized_time": slope_n,
        "endpoint_drift_enu_m": {"east": slope_e, "north": slope_n, "up": None},
        "endpoint_drift_horizontal_m": math.hypot(slope_e, slope_n),
    }


def _quartiles(records: list[dict[str, Any]]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    count = len(records)
    for index in range(4):
        start = math.floor(index * count / 4)
        end = math.floor((index + 1) * count / 4)
        if end <= start:
            end = min(count, start + 1)
        result[f"q{index + 1}"] = {
            "index_start": start,
            "index_end_exclusive": end,
            "summary": _summary([record["vector"] for record in records[start:end]]),
        }
    return result


def _route_diagnostic(
    route: str,
    candidate: list[dict[str, Any]],
    truth: list[dict[str, Any]],
    imu: dict[str, list[tuple[int, float, float, float]]],
    expected_truth_rows: int,
    expected_warmup_exclusions: int,
) -> dict[str, Any]:
    candidate_by_key = {(row["phone"], row["timestamp"]): row for row in candidate}
    truth_by_key = {(row["phone"], row["timestamp"]): row for row in truth}
    candidate_keys = set(candidate_by_key)
    truth_keys = set(truth_by_key)
    matched_keys = candidate_keys & truth_keys
    if not matched_keys:
        raise _fail(f"no exact matched keys for {route}")
    ordered_keys = sorted(matched_keys, key=lambda key: key[1])
    first_truth = truth_by_key[ordered_keys[0]]
    origin = _ecef(first_truth["lat"], first_truth["lon"], first_truth["alt"])
    speed_labels, speed_report = _prediction_speeds(candidate)
    accel = imu.get("UncalAccel", [])
    magnetometer = imu.get("UncalMag", [])
    records: list[dict[str, Any]] = []
    orientation_labels: list[tuple[str, str, str]] = []
    for key in ordered_keys:
        prediction = candidate_by_key[key]
        reference = truth_by_key[key]
        vector = _enu_delta(prediction["lat"], prediction["lon"], reference["lat"], reference["lon"], reference["alt"], origin, first_truth["lat"], first_truth["lon"])
        label = _orientation_label(_sample_for_time(accel, key[1]), _sample_for_time(magnetometer, key[1]))
        orientation_labels.append(label)
        records.append({"timestamp": key[1], "vector": vector, "speed_bin": speed_labels[key], "orientation_label": label})
    vectors = [record["vector"] for record in records]
    overall = _summary(vectors)
    prefix_count = max(1, math.ceil(len(records) * 0.25))
    prefix = _summary(vectors[:prefix_count])
    tail = _summary(vectors[-prefix_count:])
    prefix_med = prefix["median_enu_m"]
    tail_med = tail["median_enu_m"]
    prefix_tail_delta = math.hypot(tail_med["east"] - prefix_med["east"], tail_med["north"] - prefix_med["north"])
    speed_groups: dict[str, Any] = {}
    for name in ("unavailable", "<=1", ">1<=5", ">5<=15", ">15"):
        speed_groups[name] = _summary([record["vector"] for record in records if record["speed_bin"] == name])
    labels = orientation_labels
    all_imu_labels = {"gravity_axis_sign": _group_orientation(vectors, labels, 0), "gravity_quality": _group_orientation(vectors, labels, 1), "tilt_compensated_magnetic_heading_octant": _group_orientation(vectors, labels, 2)}
    finite = all(math.isfinite(value) for vector in vectors for value in vector)
    matched_count = len(matched_keys)
    extra_count = len(candidate_keys - truth_keys)
    missing_count = len(truth_keys - candidate_keys)
    return {
        "route": route,
        "phone_model": route.split("/", 1)[1],
        "candidate_rows": len(candidate),
        "truth_rows": len(truth),
        "expected_truth_rows": expected_truth_rows,
        "matched_rows": matched_count,
        "missing_truth_rows": missing_count,
        "extra_prediction_rows": extra_count,
        "finite": finite,
        "coverage": {
            "matched_key_count": matched_count,
            "candidate_key_count": len(candidate_keys),
            "truth_row_count": len(truth),
            "matched_key_coverage": matched_count / len(truth),
            "prediction_domain_coverage": matched_count / len(candidate),
            "prediction_domain_complete": extra_count == 0 and matched_count == len(candidate),
            "truth_row_coverage": len(truth) / expected_truth_rows,
            "missing_truth_rows": missing_count,
            "warmup_exclusion_expected": expected_warmup_exclusions,
            "warmup_exclusion_observed": missing_count,
            "warmup_exclusion_matches_expectation": missing_count == expected_warmup_exclusions,
        },
        "origin": {"definition": "first chronological matched truth coordinate", "latitude_degrees": first_truth["lat"], "longitude_degrees": first_truth["lon"], "altitude_m": first_truth["alt"]},
        "overall": overall,
        "baseline_fixed_enu_score": _score_vectors(vectors),
        "prefix_first_25_percent": prefix,
        "tail_last_25_percent": tail,
        "prefix_tail_median_delta_m": prefix_tail_delta,
        "quartiles": _quartiles(records),
        "time_drift": _time_drift(records),
        "speed": speed_report,
        "speed_bins": speed_groups,
        "orientation": {
            "imu_match_window_ms": IMU_MAX_AGE_MS,
            "matched_accel_label_count": sum(label[0] != "unavailable" for label in labels),
            "matched_accel_label_fraction": sum(label[0] != "unavailable" for label in labels) / len(labels),
            **all_imu_labels,
        },
        "_vectors": vectors,
    }


def _cross_route(route_reports: dict[str, dict[str, Any]]) -> dict[str, Any]:
    medians = {
        route: (report["overall"]["median_enu_m"]["east"], report["overall"]["median_enu_m"]["north"])
        for route, report in route_reports.items()
    }
    common = _vector_median(list(medians.values()))
    mad = (_median(abs(value[0] - common[0]) for value in medians.values()), _median(abs(value[1] - common[1]) for value in medians.values()))
    center_distances = {route: math.hypot(vector[0] - common[0], vector[1] - common[1]) for route, vector in medians.items()}
    pairwise: dict[str, float] = {}
    for index, left in enumerate(ROUTES):
        for right in ROUTES[index + 1 :]:
            pairwise[f"{left}__vs__{right}"] = math.hypot(medians[left][0] - medians[right][0], medians[left][1] - medians[right][1])
    return {
        "route_median_enu_m": {route: {"east": vector[0], "north": vector[1], "up": None} for route, vector in medians.items()},
        "common_route_median_enu_m": {"east": common[0], "north": common[1], "up": None},
        "route_median_component_mad_m": {"east": mad[0], "north": mad[1]},
        "route_median_center_distance_m": center_distances,
        "max_center_distance_m": max(center_distances.values()),
        "pairwise_route_median_distance_m": pairwise,
        "route_count": len(route_reports),
    }


def _loo_diagnostics(route_reports: dict[str, dict[str, Any]]) -> dict[str, Any]:
    rows: dict[str, Any] = {}
    for held in ROUTES:
        training_vectors = [
            (route_reports[route]["overall"]["median_enu_m"]["east"], route_reports[route]["overall"]["median_enu_m"]["north"])
            for route in ROUTES if route != held
        ]
        training = _vector_median(training_vectors)
        vectors = route_reports[held]["_vectors"]
        baseline = _score_vectors(vectors)
        loo_score = _score_vectors(vectors, training)
        improvement = baseline["kaggle_score_m"] - loo_score["kaggle_score_m"]
        rows[held] = {
            "held_route": held,
            "training_route_count": 3,
            "training_common_median_enu_m": {"east": training[0], "north": training[1], "up": None},
            "baseline_score_m": baseline["kaggle_score_m"],
            "loo_score_m": loo_score["kaggle_score_m"],
            "baseline_p50_m": baseline["p50_m"],
            "baseline_p95_m": baseline["p95_m"],
            "loo_p50_m": loo_score["p50_m"],
            "loo_p95_m": loo_score["p95_m"],
            "improvement_m": improvement,
            "score_worsening_m": max(0.0, -improvement),
            "diagnostic_only": True,
            "corrected_coordinates_written": False,
            "native_inference_consumed": False,
        }
    baseline_macro = sum(row["baseline_score_m"] for row in rows.values()) / len(rows)
    loo_macro = sum(row["loo_score_m"] for row in rows.values()) / len(rows)
    return {
        "per_held_route": rows,
        "baseline_macro_score_m": baseline_macro,
        "loo_macro_score_m": loo_macro,
        "macro_improvement_m": baseline_macro - loo_macro,
        "max_individual_worsening_m": max((row["score_worsening_m"] for row in rows.values()), default=0.0),
        "diagnostic_only": True,
        "correction_applied_to_inference": False,
    }


def _full_cohort_diagnostic(route_reports: dict[str, dict[str, Any]], cross_route: dict[str, Any]) -> dict[str, Any]:
    common = (cross_route["common_route_median_enu_m"]["east"], cross_route["common_route_median_enu_m"]["north"])
    route_scores: dict[str, Any] = {}
    for route in ROUTES:
        score = _score_vectors(route_reports[route]["_vectors"], common)
        route_scores[route] = {**score, "diagnostic_only": True}
    macro = sum(row["kaggle_score_m"] for row in route_scores.values()) / len(route_scores)
    return {"common_median_enu_m": {"east": common[0], "north": common[1], "up": None}, "route_scores": route_scores, "macro_score_m": macro, "target_route_score_m": route_scores[TARGET]["kaggle_score_m"], "diagnostic_only": True, "corrected_coordinates_written": False, "native_inference_consumed": False}


def _gate(name: str, passed: bool, measurements: dict[str, Any], failures: list[str] | None = None) -> dict[str, Any]:
    return {"name": name, "passed": bool(passed), "measurements": measurements, "failures": list(failures or [])}


def _identifiability_gates(route_reports: dict[str, dict[str, Any]], cross_route: dict[str, Any], loo: dict[str, Any], full: dict[str, Any]) -> dict[str, Any]:
    failures: dict[str, list[str]] = {}
    exact_ok = len(route_reports) == 4 and all(report["finite"] and report["coverage"]["prediction_domain_complete"] and report["coverage"]["truth_row_coverage"] == 1.0 for report in route_reports.values())
    if not exact_ok:
        failures["exact_model_route_count"] = ["route_count_or_prediction_domain_incomplete"]
    exact_gate = _gate("exact_model_route_count", exact_ok, {"route_count": len(route_reports), "required_route_count": 4, "all_prediction_domain_complete": exact_ok, "truth_row_coverage_is_informational": True}, failures.get("exact_model_route_count"))
    mad = cross_route["route_median_component_mad_m"]
    center = cross_route["route_median_center_distance_m"]
    center_ok = mad["east"] <= 0.5 + EPS and mad["north"] <= 0.5 + EPS and all(value <= 1.0 + EPS for value in center.values())
    center_failures = []
    if mad["east"] > 0.5 + EPS or mad["north"] > 0.5 + EPS:
        center_failures.append("component_mad_over_0.5m")
    if any(value > 1.0 + EPS for value in center.values()):
        center_failures.append("route_center_radius_over_1m")
    center_gate = _gate("route_median_center_dispersion", center_ok, {"component_mad_m": mad, "center_distance_m": center, "max_center_distance_m": cross_route["max_center_distance_m"], "component_mad_limits_m": {"east": 0.5, "north": 0.5}, "center_radius_limit_m": 1.0}, center_failures)
    route_mad_radius = math.hypot(mad["east"], mad["north"])
    prefix_limit = max(1.0, 2.0 * route_mad_radius)
    prefix_measurements = {route: {"shift_m": report["prefix_tail_median_delta_m"], "limit_m": prefix_limit} for route, report in route_reports.items()}
    prefix_failures = [route for route, values in prefix_measurements.items() if values["shift_m"] > prefix_limit + EPS]
    prefix_gate = _gate("prefix_tail_stability", not prefix_failures, {"route_limits": prefix_measurements, "absolute_floor_m": 1.0, "route_mad_factor": 2.0}, [f"{route}_prefix_tail_shift_over_limit" for route in prefix_failures])
    orientation_cases: dict[str, int] = {"gravity_axis_sign": 0, "tilt_compensated_magnetic_heading_octant": 0}
    orientation_worst: dict[str, float] = {key: 0.0 for key in orientation_cases}
    orientation_bad: list[str] = []
    for route, report in route_reports.items():
        for dimension in orientation_cases:
            groups = report["orientation"][dimension]
            comparable = groups.get("usable_groups_with_at_least_20", 0)
            if comparable >= 2:
                orientation_cases[dimension] += 1
            maximum = groups.get("max_group_median_delta_from_overall_m")
            if maximum is not None:
                orientation_worst[dimension] = max(orientation_worst[dimension], maximum)
                if maximum > MAX_ORIENTATION_DELTA_M + EPS:
                    orientation_bad.append(f"{route}:{dimension}")
    orientation_ok = all(value >= 2 for value in orientation_cases.values()) and not orientation_bad
    orientation_failures = []
    for dimension, cases in orientation_cases.items():
        if cases < 2:
            orientation_failures.append(f"{dimension}_comparable_cases_below_2")
    orientation_failures.extend(f"{item}_delta_over_0.75m" for item in orientation_bad)
    orientation_gate = _gate("orientation_independence", orientation_ok, {"comparable_route_dimension_cases": orientation_cases, "minimum_cases_per_dimension": 2, "max_group_median_delta_m": orientation_worst, "max_group_median_delta_limit_m": MAX_ORIENTATION_DELTA_M, "raw_only": True}, orientation_failures)
    loo_rows = loo["per_held_route"]
    loo_bad = [route for route, row in loo_rows.items() if row["improvement_m"] < 0.05 - EPS]
    loo_gate = _gate("loo_all_route_improvement", not loo_bad, {"minimum_improvement_m": 0.05, "route_improvements_m": {route: row["improvement_m"] for route, row in loo_rows.items()}}, [f"{route}_improvement_below_0.05m" for route in loo_bad])
    macro_ok = loo["macro_improvement_m"] >= 0.05 - EPS
    macro_gate = _gate("loo_macro_improvement", macro_ok, {"macro_improvement_m": loo["macro_improvement_m"], "minimum_macro_improvement_m": 0.05}, [] if macro_ok else ["macro_improvement_below_0.05m"])
    worsening_ok = loo["max_individual_worsening_m"] <= 0.10 + EPS
    worsening_gate = _gate("individual_worsening_limit", worsening_ok, {"max_individual_worsening_m": loo["max_individual_worsening_m"], "maximum_score_worsening_m": 0.10}, [] if worsening_ok else ["individual_worsening_over_0.10m"])
    stretch_ok = full["macro_score_m"] <= 0.782 + EPS and full["target_route_score_m"] <= 0.782 + EPS
    stretch_gate = _gate("stretch_0_782_reachability", stretch_ok, {"full_cohort_macro_score_m": full["macro_score_m"], "full_cohort_target_route_score_m": full["target_route_score_m"], "maximum_m": 0.782}, [] if stretch_ok else ["full_cohort_0.782_not_reached"])
    gates = {gate["name"]: gate for gate in (exact_gate, center_gate, prefix_gate, orientation_gate, loo_gate, macro_gate, worsening_gate, stretch_gate)}
    return {"logic": "All predeclared gates are ANDed; no threshold was changed after truth read.", "gates": gates, "all_passed": all(gate["passed"] for gate in gates.values())}


def _strongest_structure(route_reports: dict[str, dict[str, Any]], cross_route: dict[str, Any], loo: dict[str, Any], gates: dict[str, Any]) -> dict[str, Any]:
    mad = cross_route["route_median_component_mad_m"]
    scores = {
        "route_median_center_dispersion": max(mad["east"] / 0.5, mad["north"] / 0.5, cross_route["max_center_distance_m"] / 1.0),
        "prefix_tail_temporal_drift": max(report["prefix_tail_median_delta_m"] for report in route_reports.values()),
        "loo_common_median_nontransfer": max(0.0, -min(row["improvement_m"] for row in loo["per_held_route"].values())) / 0.05,
        "stretch_target_gap": max(0.0, gates["gates"]["stretch_0_782_reachability"]["measurements"]["full_cohort_macro_score_m"] - 0.782) / 0.782 if "stretch_0_782_reachability" in gates["gates"] else 0.0,
    }
    name = max(scores, key=scores.get)
    if name == "route_median_center_dispersion":
        evidence = {"component_mad_m": mad, "max_center_distance_m": cross_route["max_center_distance_m"]}
        description = "The route-median ENU centers do not form a tight common residual center under the predeclared component-MAD/radius limits."
    elif name == "prefix_tail_temporal_drift":
        evidence = {route: report["prefix_tail_median_delta_m"] for route, report in route_reports.items()}
        description = "The fixed ENU residual changes materially between the first and last chronological quarters, indicating route-time structure rather than a stationary constant."
    elif name == "loo_common_median_nontransfer":
        evidence = {route: row["improvement_m"] for route, row in loo["per_held_route"].items()}
        description = "A common route median learned on three routes does not transfer to every held route under the predeclared diagnostic score."
    else:
        evidence = {"full_cohort_macro_score_m": gates["gates"]["stretch_0_782_reachability"]["measurements"]["full_cohort_macro_score_m"], "limit_m": 0.782}
        description = "The single full-cohort common-median diagnostic remains materially above the aspirational 0.782 m target."
    return {"name": name, "description": description, "evidence": evidence, "selection": "largest predeclared diagnostic violation score; descriptive only"}


def _next_factor(no_go: bool) -> dict[str, Any]:
    if no_go:
        return {
            "exactly_one": True,
            "name": "raw Android GNSS receiver clock/timing residual",
            "raw_fields": ["TimeNanos", "FullBiasNanos", "BiasNanos", "utcTimeMillis"],
            "source_supported_by": "Phase25 raw-clock contract and the Android raw GNSS timing fields already retained by this repository",
            "mechanism": "truth-free receiver-clock/timestamp residual and segment-discontinuity diagnostic before any calibration fit",
            "implementation_or_deployment": False,
        }
    return {
        "exactly_one": True,
        "name": "raw-observable GNSS/IMU timing residual mechanism",
        "raw_fields": ["TimeNanos", "FullBiasNanos", "BiasNanos", "utcTimeMillis"],
        "source_supported_by": "existing Android raw timing contract",
        "mechanism": "future raw-only deployable timing residual path; no implementation authorized by Phase45",
        "implementation_or_deployment": False,
    }


def score(output_root: Path = DEFAULT_OUTPUT) -> dict[str, Any]:
    """Execute Phase45 exactly once after all pre-truth checks succeed."""
    started = time.perf_counter()
    output_root = output_root.resolve()
    _reject_path(output_root)
    if output_root.exists() and any(output_root.iterdir()):
        raise _fail(f"refusing to rerun Phase45 evaluation: {output_root}")
    freeze = _verify_freeze()
    manifest = _verify_manifest()
    authority = _verify_phase44_authority()
    inputs = freeze.get("sealed_inputs")
    if not isinstance(inputs, dict):
        raise _fail("Phase45 sealed input map missing")
    candidate_pins = inputs.get("candidate_submissions")
    truth_pins = inputs.get("truth")
    imu_pins = inputs.get("raw_imu")
    if not isinstance(candidate_pins, dict) or not isinstance(truth_pins, dict) or not isinstance(imu_pins, dict):
        raise _fail("Phase45 sealed input maps are invalid")
    if tuple(candidate_pins) != ROUTES or tuple(truth_pins) != ROUTES or tuple(imu_pins) != ROUTES:
        raise _fail("Phase45 sealed input order changed")
    warmup_expected = freeze["warmup_contract"].get("expected", {})
    route_reports: dict[str, dict[str, Any]] = {}
    input_hashes: dict[str, Any] = {}
    read_counts = {"candidate": 0, "truth": 0, "raw_imu": 0}
    # This is the only payload-read loop.  All freeze, source, authority and
    # artifact checks above complete before the first truth bytes are opened.
    for route in ROUTES:
        phone = route.split("/", 1)[0] + "/" + route.split("/", 1)[1]
        candidate_pin = candidate_pins[route]
        truth_pin = truth_pins[route]
        imu_pin = imu_pins[route]
        if not all(isinstance(pin, dict) for pin in (candidate_pin, truth_pin, imu_pin)):
            raise _fail(f"Phase45 pin missing: {route}")
        candidate_path = ROOT / str(candidate_pin["path"])
        truth_path = ROOT / str(truth_pin["path"])
        imu_path = ROOT / str(imu_pin["path"])
        candidate_payload, candidate_hash = _read_bytes_once(candidate_path, f"candidate submission {route}", str(candidate_pin["sha256"]))
        read_counts["candidate"] += 1
        candidate = _parse_candidate(candidate_payload, phone, f"candidate submission {route}")
        if len(candidate) != int(candidate_pin.get("rows", len(candidate))):
            raise _fail(f"candidate row count changed: {route}")
        truth_payload, truth_hash = _read_bytes_once(truth_path, f"Phase44 truth {route}", str(truth_pin["sha256"]))
        read_counts["truth"] += 1
        truth = _parse_truth(truth_payload, phone, f"Phase44 truth {route}")
        if len(truth) != int(truth_pin.get("rows", len(truth))):
            raise _fail(f"truth row count changed: {route}")
        imu_payload, imu_hash = _read_bytes_once(imu_path, f"raw IMU {route}", str(imu_pin["sha256"]))
        read_counts["raw_imu"] += 1
        imu, imu_counts = _parse_raw_imu(imu_payload, f"raw IMU {route}")
        expected_warmup = int(warmup_expected.get(route, {}).get("expected_unmatched_truth_rows", 0))
        route_reports[route] = _route_diagnostic(route, candidate, truth, imu, int(truth_pin["rows"]), expected_warmup)
        input_hashes[route] = {"candidate_submission_sha256": candidate_hash, "truth_sha256": truth_hash, "raw_imu_sha256": imu_hash, "raw_imu_message_type_counts": imu_counts}
    cross_route = _cross_route(route_reports)
    loo = _loo_diagnostics(route_reports)
    full = _full_cohort_diagnostic(route_reports, cross_route)
    identifiability = _identifiability_gates(route_reports, cross_route, loo, full)
    strongest = _strongest_structure(route_reports, cross_route, loo, identifiability)
    all_passed = identifiability["all_passed"]
    for report in route_reports.values():
        report.pop("_vectors", None)
    report = {
        "schema_version": SCHEMA,
        "phase": 45,
        "status": "diagnostic-go-raw-only-mechanism-planning" if all_passed else "no-go-residual-not-identifiable",
        "decision": "diagnostic-go-no-native-correction" if all_passed else "no-go-no-correction-no-validation",
        "freeze": {"path": _relative(FREEZE), "sha256": FREEZE_SHA256},
        "evaluator_manifest": {"path": _relative(EVALUATOR_MANIFEST), "sha256": manifest["_sha256"]},
        "authority": authority,
        "route_order": list(ROUTES),
        "residual_contract": freeze["residual_contract"],
        "routes": route_reports,
        "cross_route": cross_route,
        "loo_common_median": loo,
        "full_cohort_common_median_diagnostic": full,
        "identifiability": identifiability,
        "strongest_residual_structure": strongest,
        "next_raw_physical_factor": _next_factor(not all_passed),
        "truth_accounting": {
            "single_process": True,
            "truth_open_count": read_counts["truth"],
            "truth_read_count_per_route": 1,
            "candidate_open_count": read_counts["candidate"],
            "candidate_read_count_per_route": 1,
            "raw_imu_open_count": read_counts["raw_imu"],
            "raw_imu_read_count_per_route": 1,
            "phase44_materialized_truth_reused": True,
            "archive_open_count": 0,
            "archive_materialization_count": 0,
            "truth_rematerialization_count": 0,
            "validation_truth_open_count": 0,
            "holdout_truth_open_count": 0,
            "mat_read_or_generated": False,
            "device_wls_coordinates_used": 0,
            "precomputed_inference_coordinates_used": 0,
            "kaggle_or_token_access": 0,
            "solver_rerun": False,
            "truth_derived_correction_fit": False,
            "truth_derived_correction_application": False,
            "post_truth_tuning": False,
        },
        "input_hashes": input_hashes,
        "policy": {
            "candidate_mutated": False,
            "corrected_coordinates_written": False,
            "native_inference_consumed_loo_or_full_median": False,
            "validation_remains_sealed": True,
            "holdout_remains_sealed": True,
            "exactly_one_next_raw_physical_factor": True,
        },
        "runtime": {"wall_seconds": time.perf_counter() - started},
    }
    output_root.mkdir(parents=True, exist_ok=True)
    result_path = output_root / "phase45_pixel5_residual_diagnostic.json"
    _atomic_json(result_path, report)
    result_hash = _hash_path(result_path, "Phase45 result")
    output_manifest = {
        "schema_version": MANIFEST_SCHEMA,
        "status": report["status"],
        "result": {"path": _relative(result_path), "sha256": result_hash},
        "freeze": report["freeze"],
        "evaluator_manifest": report["evaluator_manifest"],
        "truth_open_count": read_counts["truth"],
        "truth_read_count_per_route": 1,
        "candidate_open_count": read_counts["candidate"],
        "candidate_read_count_per_route": 1,
        "raw_imu_open_count": read_counts["raw_imu"],
        "raw_imu_read_count_per_route": 1,
        "archive_open_count": 0,
        "validation_truth_open_count": 0,
        "holdout_truth_open_count": 0,
        "mat_read_or_generated": False,
        "solver_rerun": False,
        "truth_derived_correction_fit": False,
        "truth_derived_correction_application": False,
        "atomic_publish": True,
    }
    _atomic_json(output_root / "phase45_pixel5_residual_diagnostic.manifest.json", output_manifest)
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
            print(json.dumps({"status": report["status"], "truth_open_count": report["truth_accounting"]["truth_open_count"], "all_gates_passed": report["identifiability"]["all_passed"]}, sort_keys=True))
    except (OSError, Phase45Error, ValueError) as exc:
        print(f"phase45: {exc}", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
