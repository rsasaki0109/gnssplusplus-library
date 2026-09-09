#!/usr/bin/env python3
"""Run the Phase51 native Android SV-time uncertainty sigma-floor matrix.

The evaluator has two deliberately separate phases.  ``verify-freeze`` and
the preflight in ``--run-matrix`` only read sealed records/source files and
perform no raw or truth reads.  Once the evaluator manifest is sealed, the
matrix runs the candidate twice and its flag-off control once for each of the
four frozen Pixel5 routes.  Only after every structural gate passes does one
process read each already-materialized Phase44 development truth file once;
candidate and control metrics are then computed in memory.  No solver is
started after truth is read and no truth-derived value reaches the native
process.

The native option is a continuous measurement-variance model, not a threshold
or bucket exclusion.  This evaluator does not tune it and records a no-go
without a second run when a structural or accuracy gate fails.
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
import subprocess
import tempfile
import time
from typing import Any


ROOT = Path(__file__).resolve().parents[3]
FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_phase51_android_sv_time_uncertainty_sigma_floor_freeze_v1.json"
FREEZE_SHA256 = "829b993d293ea8d55c999b6672ae9b60cc92d3f5bc2afa4942531d20337e10f0"
MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase51_android_sv_time_uncertainty_sigma_floor_evaluator_manifest_v2.json"
BINARY = ROOT / "build/apps/gnss_fgo_imu_no_base"
DEFAULT_OUTPUT = ROOT / "output/smartphone-r5/phase51-android-sv-time-uncertainty-sigma-floor-v2"
RESULT_NAME = "phase51_structural_and_development.json"
MANIFEST_NAME = "phase51_structural_and_development.manifest.json"
FAILURE_NAME = "phase51_structural_failure.json"

PHASE43_RESULT = ROOT / "docs/use_cases/records/smartphone_r5_phase43_native_fallback_seed_quality_anchor_recovery_result_v1.json"
PHASE43_RESULT_SHA256 = "441c65ed8630ca2c48c329f440e8b54a772c0c062329cc5d89b297180bce2a22"
PHASE43_MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase43_native_fallback_seed_quality_anchor_recovery_evaluator_manifest_v1.json"
PHASE43_MANIFEST_SHA256 = "1433249a9ddd1809a00535b33fc67e3267a8cab29e28f773dd148f0962c44d1a"
PHASE43_FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_phase43_native_fallback_seed_quality_anchor_recovery_freeze_v1.json"
PHASE43_FREEZE_SHA256 = "9278a835a21e3a19027aefd3675aba45d8f1f9e19183cbc5590850b776722400"
PHASE44_RESULT = ROOT / "docs/use_cases/records/smartphone_r5_phase44_pixel5_development_accuracy_result_v1.json"
PHASE44_RESULT_SHA256 = "9e441c78f7c2bf8b3cf2cf9a7c9fe7e447fb2ff8eb6324fa5ef138c3b419e48a"
PHASE44_MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase44_pixel5_development_accuracy_evaluator_manifest_v1.json"
PHASE44_MANIFEST_SHA256 = "9b63607a9c8a7581cbe94868aabb21f235698e6f54cd8a879268245e327a6f1e"

SCHEMA = "smartphone-r5-phase51-android-sv-time-uncertainty-sigma-floor.v1"
MANIFEST_SCHEMA = "smartphone-r5-phase51-android-sv-time-uncertainty-sigma-floor-manifest.v2"
FREEZE_SCHEMA = "smartphone-r5-phase51-android-sv-time-uncertainty-sigma-floor-freeze.v1"
ROUTES = (
    "2021-03-16-18-59-us-ca-mtv-a/pixel5",
    "2021-08-24-20-32-us-ca-mtv-h/pixel5",
    "2022-04-01-18-22-us-ca-lax-t/pixel5",
    "2023-03-08-21-34-us-ca-mtv-u/pixel5",
)
TARGET_ROUTE = ROUTES[1]
PHASE25_ROUTES = {ROUTES[0]}
PHASE25_ROOT = ROOT / "output/smartphone-r5/phase25-raw-clock-eval-v1/raw"
PHASE37_ROOT = ROOT / "output/smartphone-r5/phase37-pixel5-repeatability-v1/routes"
RAW_FILENAMES = {"device_gnss": "device_gnss.csv", "device_imu": "device_imu.csv", "brdc_nav": "brdc.nav"}
BASE_FLAGS = (
    "--all-epochs",
    "--android-raw-utc-keys",
    "--android-raw-clock-only",
    "--android-utc-wall-clock-fallback",
    "--native-pdc-imu-tdcp-no-bridge",
    "--native-quality-anchor",
    "--native-fallback-seed-quality-anchor-recovery",
)
CANDIDATE_FLAG = "--native-android-sv-time-uncertainty-sigma-floor"
SUBMISSION_FIELDS = ("phone", "UnixTimeMillis", "LatitudeDegrees", "LongitudeDegrees")
TRUTH_FIELDS = ("UnixTimeMillis", "LatitudeDegrees", "LongitudeDegrees")
HAVERSINE_RADIUS_M = 6_371_008.8
SPEED_RADIUS_M = 6_371_000.0
MAX_SPEED_MPS = 70.0
STRETCH_TARGET_M = 0.782

# The control includes the frozen Phase43 fallback flag.  These are the
# Phase43 candidate bytes, and therefore are the expected Phase51 flag-off
# bytes after the new option is left disabled.  Exact identity is stronger
# than a projected JSON comparison and protects the default-byte contract.
FROZEN_CONTROL = {
    ROUTES[0]: {"submission": "d4d7652e5d12389466e586fe2d8e85d34977a7e11036cee2f591a89293c5426c", "summary": "d4980260e257c92d7e2fb716f900501fd55c3eb9278f9b38a4f72ac2e62303fe"},
    ROUTES[1]: {"submission": "3cfe97750927fa268f71bbe7393754ce7a1eb39d937e085b177c50a71eec10ae", "summary": "23f409e5708dc547bc86b156e07a6fb305732cb9ce8eb3da57f26fd487e4404e"},
    ROUTES[2]: {"submission": "4eb2b566708a87db4903610a41cd648c7ff065d35710d358894a78eaa8e116e3", "summary": "f4fb3bc2b09d44700ee83772d4b23ad93b68d07eff6743b9525626f805264eb8"},
    ROUTES[3]: {"submission": "524769cdf67aa857eefaafdadf943dd76586dda6a53e8b6d1df4a707d7699f71", "summary": "eeb4f60e352ea00277604516b432673317ab035daa601135c6e9f989cb118926"},
}


class Phase51Error(ValueError):
    """Raised when an immutable Phase51 gate cannot be proved."""


def fail(message: str) -> Phase51Error:
    return Phase51Error(message)


def relative(path: Path) -> str:
    try:
        return str(path.relative_to(ROOT))
    except ValueError:
        return str(path)


def reject_forbidden(path: Path | str, *, allow_development_truth: bool = False) -> None:
    token = str(path).lower()
    forbidden = (".mat", "validation", "holdout", "precomputed", "device_wls", "svposition", "kaggle", "token")
    if any(term in token for term in forbidden):
        raise fail(f"forbidden path: {path}")
    if not allow_development_truth and ("ground_truth" in token or "/truth/" in token):
        raise fail(f"truth path before structural pass: {path}")


def read_bytes(path: Path, label: str, *, allow_development_truth: bool = False) -> tuple[bytes, str]:
    reject_forbidden(path, allow_development_truth=allow_development_truth)
    if not path.is_file():
        raise fail(f"missing {label}: {path}")
    digest = hashlib.sha256()
    payload = bytearray()
    try:
        with path.open("rb") as handle:
            for chunk in iter(lambda: handle.read(1024 * 1024), b""):
                digest.update(chunk)
                payload.extend(chunk)
    except OSError as exc:
        raise fail(f"failed to read {label}: {path}: {exc}") from exc
    return bytes(payload), digest.hexdigest()


def hash_file(path: Path, label: str, *, allow_development_truth: bool = False) -> str:
    return read_bytes(path, label, allow_development_truth=allow_development_truth)[1]


def load_json(path: Path, label: str, *, allow_development_truth: bool = False) -> tuple[dict[str, Any], str, int]:
    payload, digest = read_bytes(path, label, allow_development_truth=allow_development_truth)
    try:
        value = json.loads(payload.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise fail(f"invalid {label}: {path}") from exc
    if not isinstance(value, dict):
        raise fail(f"{label} is not a JSON object")
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
    freeze, digest, _ = load_json(FREEZE, "Phase51 freeze")
    if digest != FREEZE_SHA256:
        raise fail("Phase51 freeze hash changed")
    if freeze.get("schema_version") != FREEZE_SCHEMA or freeze.get("phase") != 51 or freeze.get("status") != "frozen-before-code-and-truth":
        raise fail("Phase51 freeze schema/status changed")
    authority = freeze.get("authority", {})
    if authority.get("base_commit") != "7c4bbfa" or authority.get("phase44_commit") != "a323eb6":
        raise fail("Phase51 historical authority pin changed")
    objective = freeze.get("objective", {})
    if objective.get("opt_in_flag") != CANDIDATE_FLAG or objective.get("coefficient") != 1.0 or objective.get("clip") is not False or objective.get("tuning") is not False:
        raise fail("Phase51 source-exact option contract changed")
    if objective.get("default_byte_identical") is not True or objective.get("spp_scope", "").find("FGO-only") < 0:
        raise fail("Phase51 default/SPP contract changed")
    if freeze.get("cohort", {}).get("phone_model") != "pixel5" or tuple(freeze.get("cohort", {}).get("route_order", ())) != ROUTES:
        raise fail("Phase51 route order changed")
    if freeze.get("cohort", {}).get("target_anchor_recovery_required") is not True:
        raise fail("Phase51 target anchor gate changed")
    phase48 = authority.get("phase48_raw_evidence_result", {})
    if phase48.get("metric_input") is not False or phase48.get("policy_input_only") is not True:
        raise fail("Phase48 must remain policy-only")
    gates = freeze.get("structural_gates", {})
    for key, expected in (("candidate_repeat_count", 2), ("control_repeat_count", 1), ("truth_reads_before_structural_seal", 0), ("over_70_mps_count", 0)):
        if gates.get(key) != expected:
            raise fail(f"Phase51 structural gate changed: {key}")
    accuracy = freeze.get("accuracy_gates", {})
    for key, expected in (("route_improvement_min_m", 0.05), ("macro_improvement_min_m", 0.1), ("target_mtv_h_improvement_min_m", 0.1)):
        if accuracy.get(key) != expected:
            raise fail(f"Phase51 accuracy gate changed: {key}")
    if accuracy.get("post_truth_tuning") is not False or accuracy.get("gate_mutation_after_read") is not False:
        raise fail("Phase51 post-truth mutation contract changed")
    raw_routes = freeze.get("raw_inputs", {}).get("routes", {})
    if set(raw_routes) != set(ROUTES):
        raise fail("Phase51 raw route hash set changed")
    for route in ROUTES:
        item = raw_routes[route]
        if not isinstance(item.get("device_gnss_sha256"), str) or len(item["device_gnss_sha256"]) != 64 or int(item.get("device_gnss_bytes", 0)) <= 0:
            raise fail(f"Phase51 GNSS hash pin malformed: {route}")
    return freeze


def verify_pins(freeze: dict[str, Any]) -> dict[str, Any]:
    # These are record-only reads.  They intentionally do not open any truth
    # CSV.  Phase43 is the raw-only champion; Phase44 supplies only the frozen
    # metric/route metadata and already-materialized truth identities.
    phase43_freeze, p43f_hash, _ = load_json(PHASE43_FREEZE, "Phase43 freeze")
    phase43_result, p43r_hash, _ = load_json(PHASE43_RESULT, "Phase43 result")
    phase43_manifest, p43m_hash, _ = load_json(PHASE43_MANIFEST, "Phase43 manifest")
    phase44_result, p44r_hash, _ = load_json(PHASE44_RESULT, "Phase44 result")
    phase44_manifest, p44m_hash, _ = load_json(PHASE44_MANIFEST, "Phase44 manifest")
    if p43f_hash != PHASE43_FREEZE_SHA256 or p43r_hash != PHASE43_RESULT_SHA256 or p43m_hash != PHASE43_MANIFEST_SHA256:
        raise fail("Phase43 pin changed")
    if p44r_hash != PHASE44_RESULT_SHA256 or p44m_hash != PHASE44_MANIFEST_SHA256:
        raise fail("Phase44 pin changed")
    if phase43_result.get("status") != "sealed-truth-free-structural-go" or phase43_result.get("decision", {}).get("truth_open_count") != 0:
        raise fail("Phase43 champion is not sealed truth-free")
    if phase43_manifest.get("truth_accounting", {}).get("allowed_truth_reads") != 0 or phase43_freeze.get("input_contract", {}).get("raw_only") is not True:
        raise fail("Phase43 truth-free/source pin failed")
    if phase44_result.get("timestamp_contract", {}).get("matching") != "exact integer key intersection only" or phase44_result.get("metric_contract", {}).get("earth_radius_m") != HAVERSINE_RADIUS_M or phase44_result.get("metric_contract", {}).get("kaggle_score") != "(P50 + P95) / 2":
        raise fail("Phase44 metric contract changed")
    if tuple(phase44_result.get("route_order", ())) != ROUTES:
        raise fail("Phase44 route order pin changed")
    baseline = freeze.get("baseline_pins", {}).get("phase44_route_baseline_score_m", {})
    if set(baseline) != set(ROUTES) or any(not math.isfinite(float(baseline[r])) for r in ROUTES):
        raise fail("Phase44 score pin missing")
    return {"phase43_freeze": phase43_freeze, "phase43_result": phase43_result, "phase44_result": phase44_result, "phase44_manifest": phase44_manifest}


def verify_manifest() -> tuple[dict[str, Any], str]:
    manifest, digest, _ = load_json(MANIFEST, "Phase51 evaluator manifest")
    if manifest.get("schema_version") != MANIFEST_SCHEMA or manifest.get("status") not in ("evaluator-frozen-before-phase51-structural-and-truth", "evaluator-resealed-before-phase51-structural-resume-and-truth"):
        raise fail("Phase51 evaluator manifest schema/status changed")
    freeze_pin = manifest.get("freeze", {})
    if freeze_pin.get("path") != relative(FREEZE) or freeze_pin.get("sha256") != FREEZE_SHA256:
        raise fail("Phase51 evaluator manifest freeze pin changed")
    source = manifest.get("evaluator", {}).get("source", {})
    if source.get("path") != relative(Path(__file__)) or hash_file(Path(__file__), "Phase51 evaluator source") != source.get("sha256"):
        raise fail("Phase51 evaluator source hash mismatch")
    for item in manifest.get("evaluator", {}).get("focused_tests", []):
        if not isinstance(item, dict) or hash_file(ROOT / str(item.get("path", "")), "Phase51 focused test") != item.get("sha256"):
            raise fail("Phase51 focused-test hash mismatch")
    cmake_pin = manifest.get("evaluator", {}).get("cmake", {})
    if hash_file(ROOT / str(cmake_pin.get("path", "")), "Phase51 CMake test list") != cmake_pin.get("sha256"):
        raise fail("Phase51 CMake hash mismatch")
    binary_pin = manifest.get("binary", {})
    if binary_pin.get("path") != relative(BINARY) or hash_file(BINARY, "Phase51 native binary") != binary_pin.get("sha256"):
        raise fail("Phase51 binary hash mismatch")
    policy = manifest.get("truth_policy", {})
    if policy.get("truth_reads_before_manifest_seal") != 0 or policy.get("truth_reads_before_structural_pass") != 0 or policy.get("solver_rerun_after_truth") is not False:
        raise fail("Phase51 manifest truth policy changed")
    if tuple(manifest.get("route_order", ())) != ROUTES or manifest.get("candidate_repeat_count") != 2 or manifest.get("control_count") != 1:
        raise fail("Phase51 manifest matrix changed")
    return manifest, digest


def input_paths(route: str) -> dict[str, Path]:
    trip, phone = route.split("/", 1)
    root = (PHASE25_ROOT / trip / phone) if route in PHASE25_ROUTES else (PHASE37_ROOT / trip / phone / "inputs")
    return {key: root / filename for key, filename in RAW_FILENAMES.items()}


def invocation_path(path: Path) -> str:
    # Preserve the Phase43 default-byte contract.  The historical Phase43
    # candidate summaries recorded repository-relative input names for every
    # route, including the Phase37 additions.  An absolute-vs-relative input
    # spelling is otherwise only presentation metadata, but it would needlessly
    # change the flag-off summary bytes.
    return relative(path)


def verify_inputs(freeze: dict[str, Any], route: str) -> dict[str, Any]:
    expected_gnss = freeze["raw_inputs"]["routes"][route]["device_gnss_sha256"]
    phase43_hashes = freeze["authority"].get("phase43_raw_input_sha256", {})
    # The Phase51 freeze pins the GNSS stream directly.  For IMU/nav, use the
    # immutable Phase43 source contract (pinned by verify_pins) and record all
    # three exact bytes in the Phase51 result.
    if not phase43_hashes:
        phase43, _, _ = load_json(PHASE43_FREEZE, "Phase43 freeze for input hashes")
        phase43_hashes = phase43.get("input_contract", {}).get("sha256", {})
    expected = phase43_hashes.get(route, {})
    report: dict[str, Any] = {}
    for key, path in input_paths(route).items():
        reject_forbidden(path)
        actual = hash_file(path, f"Phase51 {key} input")
        expected_hash = expected.get(RAW_FILENAMES[key])
        if key == "device_gnss" and actual != expected_gnss:
            raise fail(f"Phase51 GNSS input hash changed: {route}")
        if expected_hash is not None and actual != expected_hash:
            raise fail(f"Phase43 raw input hash changed: {route}/{RAW_FILENAMES[key]}")
        report[key] = {"path": relative(path), "sha256": actual, "bytes": path.stat().st_size, "hash_read_count": 1}
    return report


def read_raw_epoch_keys_once(path: Path) -> list[int]:
    reject_forbidden(path)
    keys: list[int] = []
    previous: int | None = None
    try:
        with path.open(encoding="utf-8-sig", newline="") as handle:
            reader = csv.DictReader(handle)
            if "utcTimeMillis" not in set(reader.fieldnames or ()):
                raise fail(f"raw input lacks utcTimeMillis: {path}")
            for row_number, row in enumerate(reader, start=2):
                try:
                    timestamp = int((row.get("utcTimeMillis") or "").strip())
                except ValueError as exc:
                    raise fail(f"raw row {row_number} has invalid utcTimeMillis") from exc
                if previous is None or timestamp != previous:
                    if previous is not None and timestamp <= previous:
                        raise fail(f"raw UTC keys are not increasing: {path}")
                    keys.append(timestamp)
                    previous = timestamp
    except OSError as exc:
        raise fail(f"failed raw key read: {path}") from exc
    if len(keys) < 2:
        raise fail(f"raw input has fewer than two epochs: {path}")
    return keys


def parse_prediction(path: Path, route: str) -> list[tuple[int, float, float]]:
    reject_forbidden(path)
    rows: list[tuple[int, float, float]] = []
    try:
        with path.open(encoding="utf-8-sig", newline="") as handle:
            reader = csv.DictReader(handle)
            if tuple(reader.fieldnames or ()) != SUBMISSION_FIELDS:
                raise fail(f"submission header mismatch: {path}")
            for row_number, row in enumerate(reader, start=2):
                if row.get("phone") != route:
                    raise fail(f"submission phone mismatch row {row_number}: {path}")
                try:
                    timestamp = int((row.get("UnixTimeMillis") or "").strip())
                    latitude = float((row.get("LatitudeDegrees") or "").strip())
                    longitude = float((row.get("LongitudeDegrees") or "").strip())
                except ValueError as exc:
                    raise fail(f"non-numeric submission row {row_number}: {path}") from exc
                if not all(math.isfinite(v) for v in (latitude, longitude)) or not -90.0 <= latitude <= 90.0 or not -180.0 <= longitude <= 180.0:
                    raise fail(f"invalid submission coordinate row {row_number}: {path}")
                if rows and timestamp <= rows[-1][0]:
                    raise fail(f"submission timestamps are not strictly increasing: {path}")
                rows.append((timestamp, latitude, longitude))
    except OSError as exc:
        raise fail(f"failed submission read: {path}") from exc
    if not rows:
        raise fail(f"empty submission: {path}")
    return rows


def haversine(a_lat: float, a_lon: float, b_lat: float, b_lon: float, radius: float = HAVERSINE_RADIUS_M) -> float:
    lat0, lat1 = math.radians(a_lat), math.radians(b_lat)
    dlat, dlon = math.radians(b_lat - a_lat), math.radians(b_lon - a_lon)
    value = math.sin(dlat / 2.0) ** 2 + math.cos(lat0) * math.cos(lat1) * math.sin(dlon / 2.0) ** 2
    distance = 2.0 * radius * math.asin(math.sqrt(min(1.0, max(0.0, value))))
    if not math.isfinite(distance) or distance < 0.0:
        raise fail("non-finite Haversine distance")
    return distance


def speed_report(rows: list[tuple[int, float, float]]) -> dict[str, Any]:
    speeds: list[float] = []
    for previous, current in zip(rows, rows[1:]):
        dt = (current[0] - previous[0]) / 1000.0
        if dt <= 0.0:
            raise fail("non-positive submission interval")
        speed = haversine(previous[1], previous[2], current[1], current[2], SPEED_RADIUS_M) / dt
        if not math.isfinite(speed):
            raise fail("non-finite submission speed")
        speeds.append(speed)
    return {"transition_count": len(speeds), "max_speed_mps": max(speeds, default=0.0), "over_70_mps_count": sum(v > MAX_SPEED_MPS for v in speeds), "finite": all(math.isfinite(v) for v in speeds)}


def percentile(values: list[float], q: float) -> float:
    if not values:
        raise fail("empty percentile distribution")
    ordered = sorted(values)
    rank = (len(ordered) - 1) * q
    lower, upper = math.floor(rank), math.ceil(rank)
    if lower == upper:
        return ordered[lower]
    fraction = rank - lower
    return ordered[lower] * (1.0 - fraction) + ordered[upper] * fraction


def validate_summary(path: Path, route: str, raw_epoch_count: int, candidate: bool) -> dict[str, Any]:
    summary, digest, size = load_json(path, "Phase51 native summary")
    if summary.get("dataset_id") != route or summary.get("truth_used") is not False or summary.get("production_default_changed") is not False:
        raise fail(f"summary identity/truth/default gate failed: {route}")
    if summary.get("native_quality_anchor") is not True or summary.get("native_pdc_imu_tdcp_no_bridge") is not True or summary.get("native_pdc_state_bridge") is not False:
        raise fail(f"Phase43 base marker gate failed: {route}")
    epochs, graph, tdcp, raw, anchor = (summary.get(k) for k in ("epochs", "graph", "tdcp_contract", "raw_utc_key_contract", "quality_anchor_initialization"))
    if not all(isinstance(item, dict) for item in (epochs, graph, tdcp, raw, anchor)):
        raise fail(f"summary diagnostics missing: {route}")
    if int(epochs.get("problem", -1)) != raw_epoch_count or int(epochs.get("output", -1)) != raw_epoch_count or int(epochs.get("pseudorange_factors", 0)) <= 0:
        raise fail(f"epoch/factor count gate failed: {route}")
    if graph.get("converged") is not True:
        raise fail(f"native graph not converged: {route}")
    if int(tdcp.get("factors_built", 0)) <= 0 or int(tdcp.get("factors_inserted", -1)) != int(tdcp.get("factors_built", 0)) or int(tdcp.get("nonfinite_residuals", -1)) != 0:
        raise fail(f"TDCP build/insert gate failed: {route}")
    if int(raw.get("raw_epoch_keys", -1)) != raw_epoch_count or int(raw.get("target_epochs", -1)) != raw_epoch_count - 1 or int(raw.get("exact_solution_epochs", -1)) != raw_epoch_count - 1 or int(raw.get("interpolated_epochs", -1)) != 0 or int(raw.get("edge_hold_epochs", -1)) != 0 or int(raw.get("unresolved_epochs", -1)) != 0:
        raise fail(f"raw UTC contract failed: {route}")
    # Both lanes carry the same frozen Phase43 recovery recipe.  MTV-h must
    # demonstrate the recovery path before either lane can be scored.
    if route == TARGET_ROUTE:
        if anchor.get("recovery_trigger") is not True or anchor.get("recovery_anchor_selected") is not True or int(anchor.get("recovery_quality_anchor_candidates", 0)) <= 0 or int(anchor.get("recovery_replay_valid_epochs", 0)) <= 0 or int(anchor.get("recovery_replay_invalid_epochs", -1)) != 0:
            raise fail("MTV-h Phase43 anchor recovery gate failed")
    if candidate:
        section = summary.get("native_android_sv_time_uncertainty_sigma_floor")
        if not isinstance(section, dict) or section.get("enabled") is not True or section.get("source_field") != "ReceivedSvTimeUncertaintyNanos" or section.get("coefficient") != 1.0 or section.get("upper_clip") is not False or section.get("spp_applied") is not False:
            raise fail(f"candidate sigma-floor telemetry contract failed: {route}")
        if section.get("sigma_formula") != "max(existing_sigma_m, c*uncertainty_nanos*1e-9)":
            raise fail(f"candidate sigma formula changed: {route}")
        applied, fallback, affected = (int(section.get(key, -1)) for key in ("rows_applied", "rows_fallback_existing_sigma", "factors_affected"))
        if applied < 0 or fallback < 0 or affected < 0 or applied + fallback != int(epochs.get("pseudorange_factors", 0)) or affected > applied:
            raise fail(f"candidate sigma-floor row accounting failed: {route}")
        floors = [section.get(key) for key in ("floor_min_m", "floor_median_m", "floor_p95_m", "floor_max_m")]
        if applied and (not all(isinstance(v, (int, float)) and math.isfinite(float(v)) and float(v) > 0.0 for v in floors) or not floors[0] <= floors[1] <= floors[2] <= floors[3]):
            raise fail(f"candidate sigma-floor distribution failed: {route}")
    elif "native_android_sv_time_uncertainty_sigma_floor" in summary:
        raise fail(f"flag-off sigma telemetry leaked: {route}")
    return {"summary": summary, "summary_sha256": digest, "summary_bytes": size, "epochs": epochs, "graph": graph, "tdcp": tdcp, "raw": raw, "anchor": anchor}


def native_command(paths: dict[str, Path], route: str, run_dir: Path, candidate: bool) -> list[str]:
    flags = list(BASE_FLAGS)
    if candidate:
        flags.append(CANDIDATE_FLAG)
    command = [str(BINARY), "--android-gnss", invocation_path(paths["device_gnss"]), "--android-imu", invocation_path(paths["device_imu"]), "--nav", invocation_path(paths["brdc_nav"]), "--out", str(run_dir / "submission.csv"), "--summary-json", str(run_dir / "summary.json"), "--dataset-id", route, *flags]
    for token in command:
        reject_forbidden(token)
    return command


def run_native(paths: dict[str, Path], route: str, run_dir: Path, raw_keys: list[int], candidate: bool) -> dict[str, Any]:
    if run_dir.exists():
        raise fail(f"refusing to overwrite native run: {run_dir}")
    run_dir.mkdir(parents=True, exist_ok=True)
    command = native_command(paths, route, run_dir, candidate)
    environment = os.environ.copy()
    local_lib = "/home/sasaki/.local/lib"
    environment["LD_LIBRARY_PATH"] = local_lib + ((":" + environment["LD_LIBRARY_PATH"]) if environment.get("LD_LIBRARY_PATH") else "")
    started = time.perf_counter()
    try:
        process = subprocess.run(command, cwd=ROOT, env=environment, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, check=False, timeout=900)
    except subprocess.TimeoutExpired as exc:
        atomic_write(run_dir / "run.log", (str(exc) + "\n").encode())
        raise fail(f"native timeout: {route}") from exc
    wall = time.perf_counter() - started
    atomic_write(run_dir / "run.log", process.stdout.encode())
    log_path = run_dir / "run.log"
    report: dict[str, Any] = {"candidate": candidate, "return_code": process.returncode, "wall_seconds": wall, "command": command, "raw_process_read_count": 1, "log": {"path": relative(log_path), "sha256": hash_file(log_path, "native run log")}}
    if process.returncode != 0:
        raise fail(f"native solver failed ({process.returncode}): {route}\n{process.stdout[-4000:]}")
    submission = run_dir / "submission.csv"
    summary = run_dir / "summary.json"
    rows = parse_prediction(submission, route)
    expected = raw_keys[1:]
    if [row[0] for row in rows] != expected:
        raise fail(f"prediction domain is not exact raw UTC warmup domain: {route}")
    speed = speed_report(rows)
    if not speed["finite"] or speed["over_70_mps_count"] != 0:
        raise fail(f"speed gate failed: {route}")
    summary_report = validate_summary(summary, route, len(raw_keys), candidate)
    submission_hash = hash_file(submission, "native submission")
    summary_hash = hash_file(summary, "native summary")
    report.update({"submission": {"path": relative(submission), "sha256": submission_hash, "bytes": submission.stat().st_size, "rows": len(rows)}, "summary": {"path": relative(summary), "sha256": summary_hash, "bytes": summary.stat().st_size}, "speed": speed, "validation": summary_report})
    return report


def route_structural(freeze: dict[str, Any], output_root: Path, route: str, raw_read_accounting: dict[str, Any]) -> dict[str, Any]:
    paths = input_paths(route)
    inputs = verify_inputs(freeze, route)
    raw_keys = read_raw_epoch_keys_once(paths["device_gnss"])
    raw_read_accounting[route] = {"hash_reads": {key: 1 for key in paths}, "epoch_key_csv_reads": 1, "native_process_reads": {"candidate": 2, "control": 1}}
    trip, phone = route.split("/", 1)
    candidate_root = output_root / "candidate" / trip / phone
    control_root = output_root / "control" / trip / phone
    candidate1 = run_native(paths, route, candidate_root / "run1", raw_keys, True)
    candidate2 = run_native(paths, route, candidate_root / "run2", raw_keys, True)
    control = run_native(paths, route, control_root / "run1", raw_keys, False)
    if candidate1["submission"]["sha256"] != candidate2["submission"]["sha256"] or candidate1["summary"]["sha256"] != candidate2["summary"]["sha256"]:
        raise fail(f"candidate repeat is not byte-identical: {route}")
    expected = FROZEN_CONTROL[route]
    if control["submission"]["sha256"] != expected["submission"] or control["summary"]["sha256"] != expected["summary"]:
        raise fail(f"flag-off default byte identity changed: {route}")
    return {"route": route, "raw_inputs": inputs, "raw_epoch_count": len(raw_keys), "target_epoch_count": len(raw_keys) - 1, "candidate": {"run1": candidate1, "run2": candidate2, "repeat_byte_identical": True}, "control": {"run1": control, "frozen_default_byte_identical": True}, "truth_open_count": 0}


def parse_truth_once(path: Path, route: str) -> tuple[dict[tuple[str, int], tuple[float, float]], dict[str, Any]]:
    # This function is called only after all structural route reports pass.
    reject_forbidden(path, allow_development_truth=True)
    if not path.is_file():
        raise fail(f"missing sealed Phase44 development truth: {path}")
    digest = hashlib.sha256()
    result: dict[tuple[str, int], tuple[float, float]] = {}
    try:
        with path.open("rb") as binary:
            payload = binary.read()
        digest.update(payload)
        text = payload.decode("utf-8-sig")
    except (OSError, UnicodeDecodeError) as exc:
        raise fail(f"failed Phase44 truth read: {path}") from exc
    reader = csv.DictReader(io.StringIO(text, newline=""))
    fields = set(reader.fieldnames or ())
    if not set(TRUTH_FIELDS).issubset(fields):
        raise fail(f"Phase44 truth fields missing: {path}")
    has_phone = "phone" in fields
    for row_number, row in enumerate(reader, start=2):
        phone = (row.get("phone") or route) if has_phone else route
        if phone != route:
            raise fail(f"truth phone mismatch row {row_number}: {path}")
        try:
            timestamp = int((row.get("UnixTimeMillis") or "").strip())
            latitude = float((row.get("LatitudeDegrees") or "").strip())
            longitude = float((row.get("LongitudeDegrees") or "").strip())
        except ValueError as exc:
            raise fail(f"truth numeric error row {row_number}: {path}") from exc
        if not all(math.isfinite(v) for v in (latitude, longitude)) or not -90.0 <= latitude <= 90.0 or not -180.0 <= longitude <= 180.0:
            raise fail(f"truth coordinate error row {row_number}: {path}")
        key = (phone, timestamp)
        if key in result:
            raise fail(f"duplicate truth key row {row_number}: {path}")
        result[key] = (latitude, longitude)
    if not result:
        raise fail(f"empty Phase44 truth: {path}")
    return result, {"path": relative(path), "sha256": digest.hexdigest(), "bytes": len(payload), "rows": len(result), "read_count": 1, "materialized": path.parent.parent.parent.parent.name == "phase44-pixel5-development-accuracy-v1"}


def truth_paths() -> dict[str, Path]:
    # All four paths were already materialized/sealed by Phase44.  No archive
    # is opened or rematerialized in Phase51.
    return {
        ROUTES[0]: ROOT / "output/smartphone-r5/phase29-no-bridge-train-eval-v1/truth/2021-03-16-18-59-us-ca-mtv-a/pixel5/ground_truth.csv",
        ROUTES[1]: ROOT / "output/smartphone-r5/phase44-pixel5-development-accuracy-v1/truth/2021-08-24-20-32-us-ca-mtv-h/pixel5/ground_truth.csv",
        ROUTES[2]: ROOT / "output/smartphone-r5/phase44-pixel5-development-accuracy-v1/truth/2022-04-01-18-22-us-ca-lax-t/pixel5/ground_truth.csv",
        ROUTES[3]: ROOT / "output/smartphone-r5/phase44-pixel5-development-accuracy-v1/truth/2023-03-08-21-34-us-ca-mtv-u/pixel5/ground_truth.csv",
    }


def score_lane(rows: list[tuple[int, float, float]], truth: dict[tuple[str, int], tuple[float, float]], route: str) -> dict[str, Any]:
    prediction = {(route, timestamp): (lat, lon) for timestamp, lat, lon in rows}
    matched = sorted(set(prediction) & set(truth), key=lambda key: key[1])
    if not matched:
        raise fail(f"no exact truth keys matched: {route}")
    errors = [haversine(*prediction[key], *truth[key]) for key in matched]
    p50, p95 = percentile(errors, 0.50), percentile(errors, 0.95)
    metric: dict[str, Any] = {"prediction_rows": len(rows), "truth_rows": len(truth), "matched_rows": len(matched), "missing_truth_rows": len(set(truth) - set(prediction)), "extra_prediction_rows": len(set(prediction) - set(truth)), "prediction_domain_coverage": len(matched) / len(prediction), "truth_row_coverage": len(matched) / len(truth), "mean_m": sum(errors) / len(errors), "p50_m": p50, "p95_m": p95, "max_m": max(errors), "score_m": (p50 + p95) / 2.0, "finite": all(math.isfinite(v) for v in errors)}
    if metric["missing_truth_rows"] != 0 or metric["extra_prediction_rows"] != 0 or not metric["finite"]:
        raise fail(f"Phase51 exact-domain truth gate failed: {route}")
    return metric


def accuracy_gates(route_reports: dict[str, Any], baseline: dict[str, float]) -> dict[str, Any]:
    route_results: dict[str, Any] = {}
    failures: list[str] = []
    candidate_scores: list[float] = []
    control_scores: list[float] = []
    for route in ROUTES:
        candidate = route_reports[route]["candidate_metrics"]
        control = route_reports[route]["control_metrics"]
        improvement = float(control["score_m"]) - float(candidate["score_m"])
        row_failures: list[str] = []
        if candidate["prediction_domain_coverage"] != 1.0 or candidate["truth_row_coverage"] != 1.0:
            row_failures.append("full_prediction_domain_coverage")
        if improvement < 0.05:
            row_failures.append("route_improvement_below_0.05m")
        if improvement < 0.0:
            row_failures.append("route_regression")
        if candidate["score_m"] > 3.0:
            row_failures.append("individual_score_over_3m")
        if candidate["p95_m"] > 5.0 and route == TARGET_ROUTE:
            row_failures.append("target_p95_over_5m")
        if candidate["over_70_mps_count"] != 0:
            row_failures.append("over_70_mps")
        route_results[route] = {"candidate_score_m": candidate["score_m"], "control_score_m": control["score_m"], "improvement_m": improvement, "failures": row_failures, "passed": not row_failures}
        failures.extend(f"{route}:{item}" for item in row_failures)
        candidate_scores.append(float(candidate["score_m"]))
        control_scores.append(float(control["score_m"]))
    candidate_macro, control_macro = sum(candidate_scores) / len(candidate_scores), sum(control_scores) / len(control_scores)
    macro_improvement = control_macro - candidate_macro
    if macro_improvement < 0.10:
        failures.append("macro_improvement_below_0.10m")
    if candidate_macro > 2.0:
        failures.append("absolute_macro_over_2m")
    if any(score > 3.0 for score in candidate_scores):
        failures.append("absolute_individual_over_3m")
    if route_reports[TARGET_ROUTE]["candidate_metrics"]["p95_m"] > 5.0:
        failures.append("absolute_target_p95_over_5m")
    return {"routes": route_results, "candidate_macro_score_m": candidate_macro, "control_macro_score_m": control_macro, "macro_improvement_m": macro_improvement, "phase44_baseline_macro_score_m": sum(float(baseline[r]) for r in ROUTES) / len(ROUTES), "all_routes_pass": not failures, "failures": failures, "stretch_target_m": STRETCH_TARGET_M, "stretch_target_reached": candidate_macro <= STRETCH_TARGET_M}


def run_matrix(output_root: Path = DEFAULT_OUTPUT) -> dict[str, Any]:
    freeze = verify_freeze()
    pins = verify_pins(freeze)
    manifest, manifest_hash = verify_manifest()
    output_root = output_root.resolve()
    reject_forbidden(output_root)
    if output_root.exists() and any(output_root.iterdir()):
        raise fail(f"refusing to overwrite nonempty Phase51 output: {output_root}")
    output_root.mkdir(parents=True, exist_ok=True)
    raw_reads: dict[str, Any] = {}
    routes: dict[str, Any] = {}
    # Structural-only section.  No call to truth_paths or parse_truth_once is
    # reachable until this complete loop has succeeded.
    for route in ROUTES:
        routes[route] = route_structural(freeze, output_root, route, raw_reads)
    truth_sources = truth_paths()
    truth_maps: dict[str, dict[tuple[str, int], tuple[float, float]]] = {}
    truth_reports: dict[str, Any] = {}
    for route in ROUTES:
        truth_maps[route], truth_reports[route] = parse_truth_once(truth_sources[route], route)
        # Pin the expected Phase44 bytes only after opening the path, which is
        # deliberately after structural GO and still counts exactly one read.
        expected = pins["phase44_result"].get("routes", {}).get(route, {}).get("truth", {})
        if expected and (truth_reports[route]["sha256"] != expected.get("sha256") or truth_reports[route]["bytes"] != expected.get("bytes")):
            raise fail(f"sealed Phase44 truth hash/size changed: {route}")
    for route in ROUTES:
        candidate_rows = parse_prediction(ROOT / routes[route]["candidate"]["run1"]["submission"]["path"], route)
        control_rows = parse_prediction(ROOT / routes[route]["control"]["run1"]["submission"]["path"], route)
        routes[route]["candidate_metrics"] = score_lane(candidate_rows, truth_maps[route], route)
        routes[route]["control_metrics"] = score_lane(control_rows, truth_maps[route], route)
        routes[route]["candidate_metrics"]["over_70_mps_count"] = routes[route]["candidate"]["run1"]["speed"]["over_70_mps_count"]
        routes[route]["control_metrics"]["over_70_mps_count"] = routes[route]["control"]["run1"]["speed"]["over_70_mps_count"]
    baseline = freeze["baseline_pins"]["phase44_route_baseline_score_m"]
    gates = accuracy_gates(routes, baseline)
    structural_pass = True
    result = {
        "schema_version": SCHEMA,
        "phase": 51,
        "status": "development-accuracy-go" if gates["all_routes_pass"] else "no-go-development-accuracy-gates",
        "decision": "candidate-eligible-only-if-all-predeclared-gates-pass" if gates["all_routes_pass"] else "no-go-preserve-phase43-champion",
        "freeze": {"path": relative(FREEZE), "sha256": FREEZE_SHA256},
        "evaluator_manifest": {"path": relative(MANIFEST), "sha256": manifest_hash},
        "pins": {"phase43_result": {"path": relative(PHASE43_RESULT), "sha256": PHASE43_RESULT_SHA256}, "phase43_manifest": {"path": relative(PHASE43_MANIFEST), "sha256": PHASE43_MANIFEST_SHA256}, "phase44_result": {"path": relative(PHASE44_RESULT), "sha256": PHASE44_RESULT_SHA256}, "phase44_manifest": {"path": relative(PHASE44_MANIFEST), "sha256": PHASE44_MANIFEST_SHA256}, "phase48_metric_input": False},
        "candidate": {"id": "native_android_sv_time_uncertainty_sigma_floor_v1", "flag": CANDIDATE_FLAG, "formula": "max(existing_sigma_m, 299792458.0*ReceivedSvTimeUncertaintyNanos*1e-9)", "solver_rerun_after_truth": False, "post_truth_tuning": False, "spp_applied": False},
        "route_order": ROUTES,
        "structural": {"passed": structural_pass, "candidate_repeat_count": 2, "control_count": 1, "truth_reads_before_structural_pass": 0, "raw_reads": raw_reads, "routes": routes},
        "truth": {"policy": "Phase44 materialized development truth only after structural GO; one read per route; candidate/control scored in memory", "routes": truth_reports, "total_reads": sum(int(item["read_count"]) for item in truth_reports.values()), "validation_reads": 0, "holdout_reads": 0, "mat_read_or_generated": False, "device_wls_or_precomputed_coordinates": False, "kaggle_or_token_access": 0},
        "accuracy_gates": gates,
        "next_factor_if_no_go": "raw Android per-satellite carrier-phase ADR carrier-frequency/antenna phase-bias residual",
    }
    atomic_json(output_root / RESULT_NAME, result)
    result_hash = hash_file(output_root / RESULT_NAME, "Phase51 result")
    artifact_manifest = {"schema_version": "smartphone-r5-phase51-android-sv-time-uncertainty-sigma-floor-artifact-manifest.v1", "phase": 51, "result": {"path": relative(output_root / RESULT_NAME), "sha256": result_hash}, "freeze": {"path": relative(FREEZE), "sha256": FREEZE_SHA256}, "evaluator_manifest": {"path": relative(MANIFEST), "sha256": manifest_hash}, "truth_read_count": 4, "raw_route_count": 4, "candidate_runs_per_route": 2, "control_runs_per_route": 1, "structural_pass": structural_pass, "truth_free_before_structural_pass": True}
    atomic_json(output_root / MANIFEST_NAME, artifact_manifest)
    # Keep the result hash in the artifact manifest only; putting each file's
    # digest into the other file would create a circular hash dependency.
    return result


def write_failure(output_root: Path, message: str) -> None:
    try:
        output_root.mkdir(parents=True, exist_ok=True)
        atomic_json(output_root / FAILURE_NAME, {"schema_version": SCHEMA, "phase": 51, "status": "structural-no-go-before-truth", "message": message, "truth_reads": 0})
    except OSError:
        pass


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--verify-freeze", action="store_true", help="verify freeze/pins/manifest without raw or truth reads")
    parser.add_argument("--run-matrix", action="store_true", help="run the sealed structural matrix then one-shot development score")
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()
    if args.verify_freeze == args.run_matrix:
        parser.error("choose exactly one of --verify-freeze or --run-matrix")
    try:
        if args.verify_freeze:
            verify_freeze()
            verify_pins(verify_freeze())
            verify_manifest()
            print("phase51 freeze/evaluator manifest: verified without raw/truth reads")
            return 0
        result = run_matrix(args.output)
        print(json.dumps({"status": result["status"], "structural_pass": result["structural"]["passed"], "accuracy_pass": result["accuracy_gates"]["all_routes_pass"], "output": relative(args.output.resolve())}, indent=2))
        return 0
    except (OSError, Phase51Error) as exc:
        if args.run_matrix:
            write_failure(args.output.resolve(), str(exc))
        print(f"phase51: {exc}", file=os.sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
