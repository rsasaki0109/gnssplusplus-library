#!/usr/bin/env python3
"""Fail-closed structural evaluator for the Phase89 C0/D diagnostic matrix.

Phase89 is a four-route, one-run-per-route diagnostic execution of the
source-aligned Phase88 candidate.  This evaluator reads the Phase89 contract,
the pinned repository implementation/binary, and the four native summaries and
submissions produced by that contract.  It never launches a process, hashes or
opens a raw GNSS/IMU/navigation file, reads truth/MAT/Kaggle/precomputed
coordinates, or computes an accuracy score.  Coordinates are parsed only for
finite Earth-domain and speed-continuity gates.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import io
import json
import math
from pathlib import Path
import tempfile
from typing import Any


ROOT = Path(__file__).resolve().parents[3]
FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_phase89_source_clock_c0d_diagnostic_execution_freeze_v1.json"
FREEZE_SHA256 = "b379d508129712107f1e582b68cf0f0072c96d49dd2de181593fbfd89639140b"
MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase89_source_clock_c0d_diagnostic_execution_manifest_v1.json"
EVALUATOR = Path(__file__).resolve()
OUTPUT_ROOT = ROOT / "output/smartphone-r5/phase89-source-clock-c0d-diagnostic-v1"
OUTPUT_SCHEMA = "smartphone-r5-phase89-source-clock-c0d-diagnostic-result.v1"

PHASE88_FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_phase88_source_clock_c0d_active_solve_observability_diagnostic_freeze_v1.json"
PHASE88_FREEZE_SHA256 = "e03cc2327557fe9f5c5189b7a315dd02c50edec9bf4e38284e1a8c6c3271059e"
PHASE88_MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase88_source_clock_c0d_active_solve_diagnostic_execution_manifest_v1.json"
PHASE88_MANIFEST_SHA256 = "609552de0c6f3f7bd70ef4a3a4fc9264aec6af3be42bceb2522376a8756d6ad8"

ROUTES = (
    "2021-03-16-18-59-us-ca-mtv-a/pixel5",
    "2021-08-24-20-32-us-ca-mtv-h/pixel5",
    "2022-04-01-18-22-us-ca-lax-t/pixel5",
    "2023-03-08-21-34-us-ca-mtv-u/pixel5",
)
DOMAIN_ROWS = {
    ROUTES[0]: 2158,
    ROUTES[1]: 3139,
    ROUTES[2]: 1465,
    ROUTES[3]: 1101,
}

CLOCK_FLAG = "--native-source-clock-c0d-factor"
ACTIVE_FLAG = "--native-source-clock-c0d-active-solve-diagnostic"
DIRECT_FLAG = "--native-source-direct-observable-quality"
NO_BRIDGE_FLAG = "--native-pdc-imu-tdcp-no-bridge"
VELOCITY_ONLY_FLAG = "--native-gnss-first-velocity-only-handoff"
RAW_UTC_FLAG = "--android-raw-utc-keys"
SPEED_OF_LIGHT_MPS = 299792458.0
C0D_SIGMA_SECONDS = 0.1 / SPEED_OF_LIGHT_MPS
EARTH_RADIUS_M = 6_371_000.0
MAX_SPEED_MPS = 70.0

REQUIRED_FLAGS = (
    "--android-gnss",
    "--android-imu",
    "--nav",
    "--dataset-id",
    "--all-epochs",
    RAW_UTC_FLAG,
    NO_BRIDGE_FLAG,
    DIRECT_FLAG,
    CLOCK_FLAG,
    VELOCITY_ONLY_FLAG,
    ACTIVE_FLAG,
)
RAW_COMPATIBILITY_FLAGS = (
    "--android-raw-clock-only",
    "--android-utc-wall-clock-fallback",
)
FORBIDDEN_FLAGS = (
    "--obs",
    "--native-base-rinex",
    "--native-base-pseudorange-compensation",
    "--native-base-pseudorange-source-miss-mask",
    "--native-pdc-state-bridge",
    "--native-direct-doppler-wls-handoff",
    "--native-upstream-quality",
    "--native-upstream-position-offset",
    "--native-upstream-stop-constraints",
)
TERMINATION_REASONS = {
    "small_cost_change",
    "maximum_lambda",
    "maximum_outer_iterations",
    "outer_convergence_tolerance",
    "no_inner_iteration",
    "no_progress_unclassified",
    "exception",
}

GATES = (
    "implementation_and_binary_pins",
    "exactly_four_routes_one_run_each",
    "raw_only_command_and_provenance",
    "active_solve_telemetry_complete",
    "exact_terminal_reason_and_counter_consistency",
    "finite_initial_and_final_cost",
    "graph_iterations_at_least_one",
    "final_cost_strictly_less_than_initial_cost",
    "c0d_equation_jacobian_units_sigma_unchanged",
    "c0d_factor_and_skip_invariants",
    "raw_utc_rows_and_domain",
    "finite_coordinates_and_speed_only",
    "no_pdc_bridge_or_coordinate_copy",
    "accuracy_not_scored",
    "all_gates_anded",
)


class Phase89DiagnosticError(ValueError):
    """Raised when the frozen Phase89 contract or output is invalid."""


def fail(message: str) -> Phase89DiagnosticError:
    return Phase89DiagnosticError(message)


def relative(path: Path) -> str:
    try:
        return str(path.relative_to(ROOT))
    except ValueError:
        return str(path)


def _reject_forbidden(path: Path | str) -> None:
    """Reject accuracy, truth, and coordinate-artifact paths."""

    token = str(path).lower()
    if token.endswith(".mat") or ".mat/" in token or ".mat\\" in token:
        raise fail(f"MAT path is forbidden: {path}")
    for term in ("ground_truth", "validation", "holdout", "kaggle", "token", "phase82"):
        if term in token:
            raise fail(f"forbidden artifact path: {path}")


def _read_bytes(path: Path, label: str, expected_sha256: str | None = None) -> bytes:
    _reject_forbidden(path)
    if not path.is_file():
        raise fail(f"missing {label}: {path}")
    try:
        payload = path.read_bytes()
    except OSError as exc:
        raise fail(f"failed to read {label}: {path}: {exc}") from exc
    if expected_sha256 is not None:
        digest = hashlib.sha256(payload).hexdigest()
        if digest != expected_sha256:
            raise fail(f"{label} SHA-256 mismatch: {path}")
    return payload


def _hash_file(path: Path, label: str) -> str:
    return hashlib.sha256(_read_bytes(path, label)).hexdigest()


def _json_payload(payload: bytes, label: str) -> dict[str, Any]:
    try:
        value = json.loads(payload.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise fail(f"invalid {label}: {exc}") from exc
    if not isinstance(value, dict):
        raise fail(f"{label} is not an object")
    return value


def _read_json(path: Path, label: str, expected_sha256: str | None = None) -> dict[str, Any]:
    return _json_payload(_read_bytes(path, label, expected_sha256), label)


def _finite_tree(value: Any, label: str) -> None:
    if isinstance(value, bool) or value is None or isinstance(value, str):
        return
    if isinstance(value, (int, float)):
        if not math.isfinite(float(value)):
            raise fail(f"nonfinite value: {label}")
        return
    if isinstance(value, dict):
        for key, item in value.items():
            _finite_tree(item, f"{label}.{key}")
    elif isinstance(value, list):
        for index, item in enumerate(value):
            _finite_tree(item, f"{label}[{index}]")


def _number(mapping: dict[str, Any], key: str, label: str, *, positive: bool = False) -> float:
    value = mapping.get(key)
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(float(value)):
        raise fail(f"missing/nonfinite number: {label}/{key}")
    result = float(value)
    if positive and result <= 0.0:
        raise fail(f"non-positive number: {label}/{key}")
    return result


def _count(mapping: dict[str, Any], key: str, label: str, minimum: int = 0) -> int:
    value = mapping.get(key)
    if isinstance(value, bool) or not isinstance(value, int) or value < minimum:
        raise fail(f"invalid count: {label}/{key}")
    return value


def verify_phase88_authority() -> tuple[dict[str, Any], dict[str, Any]]:
    """Verify only the sealed Phase88 freeze and pre-raw manifest."""

    freeze = _read_json(PHASE88_FREEZE, "Phase88 freeze", PHASE88_FREEZE_SHA256)
    manifest = _read_json(PHASE88_MANIFEST, "Phase88 execution manifest", PHASE88_MANIFEST_SHA256)
    if freeze.get("phase") != 88 or freeze.get("status") != "frozen-before-implementation" or freeze.get("execution_label") != "Luna Max":
        raise fail("Phase88 freeze identity changed")
    if freeze.get("decision_boundary", {}).get("accuracy_scoring") is not False:
        raise fail("Phase88 accuracy boundary changed")
    if freeze.get("implementation_authorization", {}).get("execution_authorized_at_freeze") is not False:
        raise fail("Phase88 was not a pre-raw freeze")
    if freeze.get("read_accounting", {}).get("truth_reads") != 0:
        raise fail("Phase88 truth read accounting changed")
    if manifest.get("phase") != 88 or manifest.get("status") != "sealed-before-raw-execution" or manifest.get("execution_label") != "Luna Max":
        raise fail("Phase88 manifest identity changed")
    if manifest.get("execution_authorization", {}).get("raw_execution_authorized") is not False:
        raise fail("Phase88 manifest unexpectedly authorizes raw execution")
    phase88_candidate = manifest.get("candidate_contract", {})
    if phase88_candidate.get("raw_only") is not True or phase88_candidate.get("diagnostic_only") is not True or phase88_candidate.get("required_gates", {}).get("accuracy_not_scored") is not True:
        raise fail("Phase88 raw/accuracy contract changed")
    return freeze, manifest


def verify_freeze() -> dict[str, Any]:
    """Verify the Phase89 freeze and its sealed Phase88 authority."""

    if FREEZE_SHA256.startswith("__"):
        raise fail("Phase89 freeze digest has not been sealed in evaluator")
    freeze = _read_json(FREEZE, "Phase89 freeze", FREEZE_SHA256)
    verify_phase88_authority()
    if freeze.get("schema_version") != "smartphone-r5-phase89-source-clock-c0d-diagnostic-execution-freeze.v1" or freeze.get("phase") != 89 or freeze.get("status") != "frozen-before-phase89-raw-execution" or freeze.get("execution_label") != "Luna Max":
        raise fail("Phase89 freeze identity changed")
    authority = freeze.get("authority", {})
    if authority.get("phase88_freeze", {}).get("sha256") != PHASE88_FREEZE_SHA256 or authority.get("phase88_execution_manifest", {}).get("sha256") != PHASE88_MANIFEST_SHA256 or authority.get("phase88_implementation_commit") != "bd14626":
        raise fail("Phase89 Phase88 authority pin changed")
    boundary = freeze.get("decision_boundary", {})
    if boundary.get("candidate_count") != 1 or boundary.get("route_count") != 4 or boundary.get("repetitions_per_route") != 1 or boundary.get("accuracy_scoring") is not False or boundary.get("route_score_selection") is not False or boundary.get("promotion_or_accuracy_authorization") is not False or boundary.get("fail_closed_if_iterations_below_one") is not True or boundary.get("fail_closed_if_final_cost_not_strictly_less") is not True:
        raise fail("Phase89 decision boundary changed")
    candidate = freeze.get("candidate", {})
    if candidate.get("id") != "phase89_source_clock_c0d_active_solve_diagnostic_matrix" or candidate.get("source_aligned") is not True or candidate.get("raw_only") is not True or candidate.get("diagnostic_only") is not True or candidate.get("new_candidate_flags") != []:
        raise fail("Phase89 candidate identity changed")
    if candidate.get("handoff_contract") != {
        "native_pdc_state_bridge": False,
        "native_pdc_imu_tdcp_no_bridge": True,
        "native_gnss_first_velocity_only_handoff": True,
        "gnss_first_position_clock_copy": False,
        "coordinate_bridge_or_copy": False,
        "precomputed_coordinate_input": False,
        "coordinate_output_used_for_selection": False,
    }:
        raise fail("Phase89 handoff contract changed")
    routes = freeze.get("routes_and_domain", {})
    if routes.get("route_order") != list(ROUTES) or routes.get("domain_rows") != DOMAIN_ROWS or routes.get("exactly_one_run_each") is not True or routes.get("controls") != 0 or routes.get("route_selection") is not False:
        raise fail("Phase89 route/domain contract changed")
    auth = freeze.get("execution_authorization", {})
    if auth.get("before_raw_execution") is not True or auth.get("raw_execution_authorized") is not True or auth.get("implementation_and_evaluator_must_be_committed_first") is not True or auth.get("accuracy_or_submission_release") is not False:
        raise fail("Phase89 execution authorization changed")
    accounting = freeze.get("read_accounting_at_freeze", {})
    zero_keys = ("raw_device_gnss_reads", "raw_device_imu_reads", "broadcast_navigation_reads", "base_rinex_reads", "native_solver_invocations", "native_reruns", "truth_reads", "mat_reads_or_generated", "precomputed_coordinate_reads", "phase82_candidate_coordinate_reads", "validation_holdout_reads", "kaggle_or_token_access")
    if any(accounting.get(key) != 0 for key in zero_keys) or accounting.get("accuracy_scored") is not False or accounting.get("route_score_selection") is not False:
        raise fail("Phase89 pre-raw read accounting changed")
    return freeze


def _manifest_required_flags(manifest: dict[str, Any]) -> tuple[list[str], list[str]]:
    contract = manifest.get("candidate_contract")
    if not isinstance(contract, dict):
        raise fail("Phase89 candidate manifest contract missing")
    required = contract.get("required_flags")
    forbidden = contract.get("forbidden_flags")
    if not isinstance(required, list) or not all(isinstance(value, str) for value in required) or not isinstance(forbidden, list) or not all(isinstance(value, str) for value in forbidden):
        raise fail("Phase89 command flag contract malformed")
    return required, forbidden


def _verify_source_pins(manifest: dict[str, Any]) -> dict[str, Any]:
    implementation = manifest.get("implementation")
    if not isinstance(implementation, dict) or implementation.get("commit") != "bd14626":
        raise fail("Phase89 implementation commit pin changed")
    binary = implementation.get("binary")
    if not isinstance(binary, dict) or binary.get("path") != "build/apps/gnss_fgo_imu_no_base" or not isinstance(binary.get("sha256"), str) or len(binary["sha256"]) != 64:
        raise fail("Phase89 binary pin malformed")
    binary_path = ROOT / binary["path"]
    if _hash_file(binary_path, "Phase89 binary") != binary["sha256"]:
        raise fail("Phase89 binary hash changed")
    runtime = implementation.get("gtsam_runtime")
    if not isinstance(runtime, dict) or not isinstance(runtime.get("path"), str) or not isinstance(runtime.get("sha256"), str) or len(runtime["sha256"]) != 64:
        raise fail("Phase89 GTSAM runtime pin malformed")
    if _hash_file(Path(runtime["path"]), "Phase89 GTSAM runtime") != runtime["sha256"]:
        raise fail("Phase89 GTSAM runtime hash changed")
    source_pins = manifest.get("source_pins")
    if not isinstance(source_pins, dict) or not source_pins:
        raise fail("Phase89 source pin map missing")
    for relative_path, expected in source_pins.items():
        if not isinstance(relative_path, str) or not isinstance(expected, str) or len(expected) != 64:
            raise fail(f"Phase89 source pin malformed: {relative_path}")
        path = ROOT / relative_path
        if _hash_file(path, f"Phase89 source {relative_path}") != expected:
            raise fail(f"Phase89 source hash changed: {relative_path}")
    evaluator_pin = manifest.get("evaluator")
    if not isinstance(evaluator_pin, dict) or evaluator_pin.get("path") != relative(EVALUATOR) or evaluator_pin.get("sha256") != _hash_file(EVALUATOR, "Phase89 evaluator") or evaluator_pin.get("accuracy_scoring") is not False:
        raise fail("Phase89 evaluator pin changed")
    freeze_pin = manifest.get("freeze")
    if not isinstance(freeze_pin, dict) or freeze_pin.get("path") != relative(FREEZE) or freeze_pin.get("sha256") != FREEZE_SHA256:
        raise fail("Phase89 manifest freeze pin changed")
    return {"commit": implementation["commit"], "binary": binary, "source_pins": source_pins, "evaluator": evaluator_pin}


def _verify_manifest(manifest: dict[str, Any]) -> dict[str, Any]:
    if manifest.get("schema_version") != "smartphone-r5-phase89-source-clock-c0d-diagnostic-execution-manifest.v1" or manifest.get("phase") != 89 or manifest.get("status") != "sealed-before-phase89-raw-execution" or manifest.get("execution_label") != "Luna Max":
        raise fail("Phase89 manifest identity changed")
    if manifest.get("purpose") != "Pin exactly one diagnostic-only raw execution for each of the four frozen Phase89 routes; no accuracy or submission scoring is authorized.":
        raise fail("Phase89 manifest purpose changed")
    phase88 = manifest.get("authority", {}).get("phase88_execution_manifest", {})
    if phase88.get("sha256") != PHASE88_MANIFEST_SHA256:
        raise fail("Phase89 manifest Phase88 pin changed")
    candidate = manifest.get("candidate_contract", {})
    required, forbidden = _manifest_required_flags(manifest)
    if required != [
        "--android-gnss <frozen raw device_gnss.csv>",
        "--android-imu <frozen raw device_imu.csv>",
        "--nav <frozen broadcast brdc.nav>",
        "--dataset-id <one of the four frozen route IDs>",
        "--all-epochs",
        "--android-raw-utc-keys",
        "--native-pdc-imu-tdcp-no-bridge",
        "--native-source-direct-observable-quality",
        "--native-source-clock-c0d-factor",
        "--native-gnss-first-velocity-only-handoff",
        "--native-source-clock-c0d-active-solve-diagnostic",
    ] or forbidden != list(FORBIDDEN_FLAGS):
        raise fail("Phase89 required/forbidden flags changed")
    if candidate.get("raw_only") is not True or candidate.get("diagnostic_only") is not True or candidate.get("accuracy_scoring") is not False or candidate.get("route_score_selection") is not False:
        raise fail("Phase89 candidate provenance changed")
    if candidate.get("compatibility_flags") != list(RAW_COMPATIBILITY_FLAGS):
        raise fail("Phase89 raw conversion compatibility flags changed")
    if candidate.get("preserved_source_contract") != {
        "c0d_equation": "(c2-c1)-((d1+d2)*dt/(2*C_LIGHT))",
        "c0d_jacobian": "[-1,+1,-dt/(2*C_LIGHT),-dt/(2*C_LIGHT)]",
        "c0d_clock_units": "seconds",
        "c0d_drift_units": "metres_per_second",
        "c0d_residual_units": "seconds",
        "c0d_sigma_m": 0.1,
        "c0d_sigma_seconds": "0.1/C_LIGHT",
        "optimizer_algorithm_changed": False,
        "measurement_factors_changed": False,
        "source_sigma_tuning": False,
        "meter_state_parity": False,
    }:
        raise fail("Phase89 preserved C0/D contract changed")
    matrix = manifest.get("matrix", {})
    expected_matrix = {
        "candidate_count": 1,
        "routes": 4,
        "runs_per_route": 1,
        "native_invocations": 4,
        "raw_device_gnss_reads": 4,
        "raw_device_imu_reads": 4,
        "broadcast_nav_reads": 4,
        "base_rinex_reads": 0,
        "truth_reads": 0,
        "mat_reads_or_generated": 0,
        "precomputed_coordinate_reads": 0,
        "phase82_candidate_coordinate_reads": 0,
        "kaggle_or_token_access": 0,
        "accuracy_scored": False,
        "route_score_selection": False,
    }
    if matrix != expected_matrix:
        raise fail("Phase89 matrix/read contract changed")
    auth = manifest.get("execution_authorization", {})
    if auth.get("before_raw_execution") is not True or auth.get("raw_execution_authorized") is not True or auth.get("native_route_rerun_performed") is not False or auth.get("exactly_one_run_each") is not True or auth.get("stop_before_accuracy_or_submission") is not True:
        raise fail("Phase89 manifest execution authorization changed")
    routes = manifest.get("routes")
    if not isinstance(routes, list) or [item.get("dataset_id") for item in routes if isinstance(item, dict)] != list(ROUTES) or len(routes) != 4:
        raise fail("Phase89 manifest route order changed")
    for item in routes:
        if not isinstance(item, dict) or item.get("dataset_id") not in ROUTES or item.get("runs") != 1 or item.get("domain_rows") != DOMAIN_ROWS[item["dataset_id"]] or item.get("diagnostic_only") is not True:
            raise fail(f"Phase89 route record changed: {item}")
        raw = item.get("raw_inputs")
        if not isinstance(raw, dict) or set(raw) != {"device_gnss.csv", "device_imu.csv", "brdc.nav"}:
            raise fail(f"Phase89 raw input set changed: {item.get('dataset_id')}")
        for name, pin in raw.items():
            if not isinstance(pin, dict) or not isinstance(pin.get("path"), str) or not isinstance(pin.get("sha256"), str) or len(pin["sha256"]) != 64 or ".mat" in pin["path"].lower() or any(term in pin["path"].lower() for term in ("truth", "kaggle", "phase82", "base.obs")):
                raise fail(f"Phase89 raw input pin malformed: {item.get('dataset_id')}/{name}")
        output = item.get("output")
        if not isinstance(output, dict) or set(output) != {"submission", "summary"}:
            raise fail(f"Phase89 output pin malformed: {item.get('dataset_id')}")
        for name, path in output.items():
            if not isinstance(path, str) or Path(path).is_absolute() or not path.startswith("output/smartphone-r5/phase89-source-clock-c0d-diagnostic-v1/"):
                raise fail(f"Phase89 output path leaves output root: {item.get('dataset_id')}/{name}")
    _verify_source_pins(manifest)
    return manifest


def _read_submission(path: Path, route: str) -> tuple[list[tuple[int, float, float]], dict[str, Any]]:
    payload = _read_bytes(path, f"Phase89 submission {route}")
    try:
        text = payload.decode("utf-8-sig")
    except UnicodeDecodeError as exc:
        raise fail(f"submission is not UTF-8: {route}") from exc
    rows = list(csv.reader(io.StringIO(text)))
    header = ["phone", "UnixTimeMillis", "LatitudeDegrees", "LongitudeDegrees"]
    if not rows or rows[0] != header:
        raise fail(f"submission header mismatch: {route}")
    parsed: list[tuple[int, float, float]] = []
    previous: int | None = None
    for line_number, fields in enumerate(rows[1:], start=2):
        if len(fields) != 4 or fields[0] != route:
            raise fail(f"submission row key mismatch: {route}:{line_number}")
        try:
            timestamp = int(fields[1])
            latitude = float(fields[2])
            longitude = float(fields[3])
        except ValueError as exc:
            raise fail(f"non-numeric submission row: {route}:{line_number}") from exc
        if previous is not None and timestamp <= previous:
            raise fail(f"submission timestamps are not increasing: {route}")
        if not all(math.isfinite(value) for value in (latitude, longitude)) or not -90.0 <= latitude <= 90.0 or not -180.0 <= longitude <= 180.0:
            raise fail(f"invalid earth coordinate: {route}:{line_number}")
        parsed.append((timestamp, latitude, longitude))
        previous = timestamp
    if not parsed:
        raise fail(f"empty submission: {route}")
    return parsed, {"bytes": len(payload), "sha256": hashlib.sha256(payload).hexdigest(), "rows": len(parsed)}


def _speed_report(rows: list[tuple[int, float, float]]) -> dict[str, Any]:
    speeds: list[float] = []
    for previous, current in zip(rows, rows[1:]):
        dt = (current[0] - previous[0]) / 1000.0
        if dt <= 0.0:
            raise fail("non-positive submission interval")
        lat1, lon1, lat2, lon2 = map(math.radians, (previous[1], previous[2], current[1], current[2]))
        dlat, dlon = lat2 - lat1, lon2 - lon1
        hav = math.sin(dlat / 2.0) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2.0) ** 2
        distance = EARTH_RADIUS_M * 2.0 * math.asin(math.sqrt(min(1.0, max(0.0, hav))))
        speeds.append(distance / dt)
    return {"finite": all(math.isfinite(speed) for speed in speeds), "max_speed_mps": max(speeds, default=0.0), "over_70_mps_count": sum(speed > MAX_SPEED_MPS for speed in speeds), "transition_count": len(speeds)}


def read_prediction(path: Path, route: str) -> list[tuple[int, float, float]]:
    """Read one output only for finite-coordinate/speed structural gates."""

    rows, _ = _read_submission(path, route)
    return rows


def speed_report(rows: list[tuple[int, float, float]]) -> dict[str, Any]:
    return _speed_report(rows)


def _validate_provenance(summary: dict[str, Any], route: str, command: list[str]) -> dict[str, Any]:
    if summary.get("dataset_id") != route or summary.get("truth_used") is not False or summary.get("production_default_changed") is not False or summary.get("status") != "imu-combined-factor":
        raise fail(f"summary identity/truth/default gate failed: {route}")
    if summary.get("native_source_clock_c0d_factor_enabled") is not True or summary.get("native_source_clock_c0d_active_solve_diagnostic_enabled") is not True or summary.get("native_source_direct_observable_quality_enabled") is not True or summary.get("native_pdc_imu_tdcp_no_bridge") is not True or summary.get("native_pdc_state_bridge") is not False or summary.get("native_upstream_quality") is not False:
        raise fail(f"summary candidate flags changed: {route}")
    inputs = summary.get("inputs")
    if not isinstance(inputs, dict) or inputs.get("observation") is not None or not isinstance(inputs.get("android_gnss"), str) or not isinstance(inputs.get("imu"), str) or not isinstance(inputs.get("navigation"), str):
        raise fail(f"raw input provenance missing: {route}")
    for key in ("android_gnss", "imu", "navigation"):
        token = inputs[key].lower()
        if any(term in token for term in ("base", "truth", "mat", "kaggle", "phase82", "coordinate")):
            raise fail(f"forbidden input provenance: {route}/{key}")
    gnss_diag = summary.get("android_gnss_diagnostics")
    imu_init = summary.get("imu_initialization")
    if not isinstance(gnss_diag, dict) or not isinstance(imu_init, dict) or gnss_diag.get("no_device_wls_seed") is not True:
        raise fail(f"raw GNSS/IMU provenance telemetry missing: {route}")
    if imu_init.get("input_format") != "android-device_imu.csv":
        raise fail(f"non-raw IMU input format: {route}")
    if command.count(ACTIVE_FLAG) != 1 or command.count(CLOCK_FLAG) != 1 or command.count(DIRECT_FLAG) != 1 or command.count(NO_BRIDGE_FLAG) != 1 or command.count(VELOCITY_ONLY_FLAG) != 1 or command.count(RAW_UTC_FLAG) != 1 or "--all-epochs" not in command:
        raise fail(f"candidate command flag multiplicity failed: {route}")
    if any(flag in command for flag in FORBIDDEN_FLAGS) or any("base.obs" in token.lower() or ".mat" in token.lower() or "truth" in token.lower() or "kaggle" in token.lower() or "phase82" in token.lower() for token in command):
        raise fail(f"candidate command contains forbidden input/flag: {route}")
    return {"inputs": inputs, "gnss_diagnostics": gnss_diag, "imu_initialization": imu_init}


def _validate_clock(summary: dict[str, Any], route: str, expected_rows: int) -> tuple[dict[str, Any], dict[str, Any]]:
    telemetry = summary.get("native_source_clock_c0d_factor")
    if not isinstance(telemetry, dict) or telemetry.get("clock_c0d_enabled") is not True or telemetry.get("active_solve_diagnostic_enabled") is not True or telemetry.get("active_solve_attempted") is not True:
        raise fail(f"C0/D active-solve telemetry is not enabled/attempted: {route}")
    factor_count = _count(telemetry, "clock_c0d_factor_count", route, 1)
    skip_keys = ("clock_c0d_clock_jump_skips", "clock_c0d_gap_skips", "clock_c0d_invalid_dt_skips", "clock_c0d_phone_exclusion_skips")
    skips = {key: _count(telemetry, key, route) for key in skip_keys}
    if skips["clock_c0d_phone_exclusion_skips"] != 0 or factor_count + sum(skips.values()) != expected_rows:
        raise fail(f"C0/D factor/skip accounting failed: {route}")
    dt_min = _number(telemetry, "clock_c0d_dt_min_s", f"clock-c0d/{route}", positive=True)
    dt_max = _number(telemetry, "clock_c0d_dt_max_s", f"clock-c0d/{route}", positive=True)
    if not dt_min <= dt_max < 1.5:
        raise fail(f"C0/D dt range invalid: {route}")
    expected = {
        "clock_c0d_equation": "(c2-c1)-((d1+d2)*dt/(2*C_LIGHT))",
        "clock_c0d_jacobian_order": ["c1", "c2", "d1", "d2"],
        "clock_c0d_jacobian": "[-1,+1,-dt/(2*C_LIGHT),-dt/(2*C_LIGHT)]",
        "clock_c0d_units": {"clock": "seconds", "drift": "metres_per_second", "dt": "seconds", "residual": "seconds", "sigma": "seconds"},
        "clock_jump_noise": "Inf (active C0 factor omitted)",
        "parity_scope": "C0/D active-row parity; not full seven-vector",
        "legacy_scalar_clock_between_factor_count": 0,
    }
    if any(telemetry.get(key) != value for key, value in expected.items()):
        raise fail(f"C0/D equation/Jacobian/units contract failed: {route}")
    if not math.isclose(_number(telemetry, "speed_of_light_mps", f"clock-c0d/{route}"), SPEED_OF_LIGHT_MPS, rel_tol=0.0, abs_tol=1e-9) or not math.isclose(_number(telemetry, "clock_c0d_sigma_seconds", f"clock-c0d/{route}"), C0D_SIGMA_SECONDS, rel_tol=1e-12, abs_tol=1e-18):
        raise fail(f"C0/D physical constants mismatch: {route}")
    required_numbers = ("active_solve_initial_cost", "active_solve_final_cost", "initial_lambda", "maximum_lambda", "final_lambda", "max_whitened_clock_column_norm", "max_whitened_drift_column_norm", "conditioning_proxy")
    for key in required_numbers:
        _number(telemetry, key, f"active-solve/{route}")
    for key in ("accepted_outer_iterations", "total_inner_lambda_attempts", "indeterminate_linear_solve_count", "unsuccessful_model_step_count", "small_cost_change_stop_count", "maximum_lambda_stop_count"):
        _count(telemetry, key, f"active-solve/{route}")
    if telemetry.get("active_solve_finite_costs") is not True or telemetry.get("termination_trace_complete") is not True or not isinstance(telemetry.get("termination_branch_reason"), str) or not telemetry["termination_branch_reason"]:
        raise fail(f"C0/D active-solve completeness failed: {route}")
    if telemetry["initial_lambda"] <= 0.0 or telemetry["maximum_lambda"] < telemetry["initial_lambda"] or telemetry["final_lambda"] <= 0.0 or telemetry["max_whitened_clock_column_norm"] <= 0.0 or telemetry["max_whitened_drift_column_norm"] <= 0.0 or telemetry["conditioning_proxy"] <= 0.0:
        raise fail(f"C0/D active-solve conditioning/lambda domain failed: {route}")
    reason = telemetry["termination_branch_reason"]
    if reason not in TERMINATION_REASONS:
        raise fail(f"unknown active-solve terminal reason: {route}/{reason}")
    if reason == "small_cost_change" and (telemetry["small_cost_change_stop_count"] != 1 or telemetry["maximum_lambda_stop_count"] != 0):
        raise fail(f"small-cost-change counters inconsistent: {route}")
    if reason == "maximum_lambda" and (telemetry["maximum_lambda_stop_count"] != 1 or telemetry["small_cost_change_stop_count"] != 0):
        raise fail(f"maximum-lambda counters inconsistent: {route}")
    if reason in {"outer_convergence_tolerance", "maximum_outer_iterations", "no_progress_unclassified", "no_inner_iteration", "exception"} and (telemetry["small_cost_change_stop_count"] != 0 or telemetry["maximum_lambda_stop_count"] != 0):
        raise fail(f"terminal reason counters inconsistent: {route}/{reason}")
    if telemetry["total_inner_lambda_attempts"] < telemetry["accepted_outer_iterations"] + telemetry["indeterminate_linear_solve_count"]:
        raise fail(f"active-solve attempt accounting inconsistent: {route}")
    return telemetry, {"factor_count": factor_count, **skips, "dt_min_s": dt_min, "dt_max_s": dt_max}


def _validate_summary(summary: dict[str, Any], route: str, expected_rows: int, command: list[str]) -> dict[str, Any]:
    _finite_tree(summary, f"summary[{route}]")
    provenance = _validate_provenance(summary, route, command)
    epochs = summary.get("epochs")
    graph = summary.get("graph")
    tdcp = summary.get("tdcp_contract")
    utc = summary.get("raw_utc_key_contract")
    direct = summary.get("native_source_direct_observable_quality")
    gnss_first = summary.get("gnss_first")
    if not all(isinstance(item, dict) for item in (epochs, graph, tdcp, utc, direct, gnss_first)):
        raise fail(f"Phase89 structural telemetry missing: {route}")
    assert isinstance(epochs, dict) and isinstance(graph, dict) and isinstance(tdcp, dict) and isinstance(utc, dict) and isinstance(direct, dict) and isinstance(gnss_first, dict)
    if epochs.get("problem") != expected_rows + 1 or epochs.get("output") != expected_rows + 1 or graph.get("imu_intervals") != expected_rows:
        raise fail(f"rows/IMU interval invariant failed: {route}")
    if graph.get("converged") is not True:
        raise fail(f"native graph did not converge: {route}")
    if direct.get("enabled") is not True or direct.get("direct_no_pdc") is not True or direct.get("pdc_bridge") is not False or direct.get("native_pdc_state_bridge") is not False:
        raise fail(f"direct/no-PDC invariant failed: {route}")
    if tdcp.get("enabled") is not True or _count(tdcp, "factors_built", route, 1) != _count(tdcp, "factors_inserted", route, 1) or tdcp.get("finite_residuals") != tdcp.get("factors_inserted") or tdcp.get("nonfinite_residuals") != 0:
        raise fail(f"TDCP invariant failed: {route}")
    if utc.get("warmup_epoch_excluded") is not True or utc.get("raw_epoch_keys") != expected_rows + 1 or utc.get("target_epochs") != expected_rows or utc.get("exact_solution_epochs") != expected_rows or utc.get("interpolated_epochs") != 0 or utc.get("edge_hold_epochs") != 0 or utc.get("unresolved_epochs") != 0 or utc.get("device_wls_coordinates_used") is not False:
        raise fail(f"raw UTC alignment invariant failed: {route}")
    if gnss_first.get("attempted") is not True or gnss_first.get("handoff_mode") != "velocity-only" or gnss_first.get("positions_clocks_copied") != 0 or gnss_first.get("velocity_handoff_source") != "gnss-first-optimizer-result" or gnss_first.get("velocity_valid_count") != expected_rows + 1 or gnss_first.get("velocity_nonfinite_count") != 0 or gnss_first.get("velocity_over_70_mps_count") != 0:
        raise fail(f"GNSS-first velocity-only handoff invariant failed: {route}")
    telemetry, clock = _validate_clock(summary, route, expected_rows)
    initial = _number(graph, "initial_cost", f"graph/{route}")
    final = _number(graph, "final_cost", f"graph/{route}")
    if not math.isclose(initial, float(telemetry["active_solve_initial_cost"]), rel_tol=0.0, abs_tol=0.0) or not math.isclose(final, float(telemetry["active_solve_final_cost"]), rel_tol=0.0, abs_tol=0.0):
        raise fail(f"graph/active-solve cost identity failed: {route}")
    accepted = _count(telemetry, "accepted_outer_iterations", f"active-solve/{route}")
    iterations = _count(graph, "iterations", f"graph/{route}")
    if accepted != iterations:
        raise fail(f"graph/active-solve iteration identity failed: {route}")
    return {
        "provenance": provenance,
        "population": {"problem_epochs": epochs["problem"], "output_epochs": epochs["output"], "imu_intervals": graph["imu_intervals"], "pseudorange_factors": epochs["pseudorange_factors"], "tdcp_factors_built": tdcp["factors_built"], "tdcp_factors_inserted": tdcp["factors_inserted"]},
        "clock_c0d": {**clock, "active_solve_diagnostic_enabled": telemetry["active_solve_diagnostic_enabled"], "active_solve_attempted": telemetry["active_solve_attempted"], "initial_cost": telemetry["active_solve_initial_cost"], "final_cost": telemetry["active_solve_final_cost"], "accepted_outer_iterations": accepted, "total_inner_lambda_attempts": telemetry["total_inner_lambda_attempts"], "termination_branch_reason": telemetry["termination_branch_reason"], "max_whitened_clock_column_norm": telemetry["max_whitened_clock_column_norm"], "max_whitened_drift_column_norm": telemetry["max_whitened_drift_column_norm"], "conditioning_proxy": telemetry["conditioning_proxy"]},
        "graph": {"initial_cost": initial, "final_cost": final, "iterations": iterations, "converged": graph["converged"]},
        "gnss_first": {key: gnss_first.get(key) for key in ("attempted", "handoff_mode", "positions_clocks_copied", "velocity_valid_count", "velocity_nonfinite_count", "velocity_over_70_mps_count")},
    }


def _route_report(manifest_route: dict[str, Any], output_root: Path) -> dict[str, Any]:
    route = manifest_route["dataset_id"]
    expected_rows = DOMAIN_ROWS[route]
    command = manifest_route.get("command")
    if not isinstance(command, list) or not all(isinstance(token, str) for token in command):
        raise fail(f"Phase89 command missing: {route}")
    if command.count("--dataset-id") != 1 or route not in command:
        raise fail(f"Phase89 route command identity failed: {route}")
    raw = manifest_route.get("raw_inputs")
    if not isinstance(raw, dict):
        raise fail(f"Phase89 raw input pin missing: {route}")
    for flag, name in (("--android-gnss", "device_gnss.csv"), ("--android-imu", "device_imu.csv"), ("--nav", "brdc.nav")):
        try:
            value = command[command.index(flag) + 1]
        except (ValueError, IndexError) as exc:
            raise fail(f"Phase89 raw command argument missing: {route}/{flag}") from exc
        if value != raw[name]["path"]:
            raise fail(f"Phase89 raw command path differs from pin: {route}/{name}")
    for flag in REQUIRED_FLAGS:
        if flag.startswith("--") and flag not in {"--android-gnss", "--android-imu", "--nav", "--dataset-id"} and command.count(flag) != 1:
            raise fail(f"Phase89 required flag missing/multiple: {route}/{flag}")
    if any(flag in command for flag in FORBIDDEN_FLAGS):
        raise fail(f"Phase89 forbidden flag present: {route}")
    if any(command.count(flag) != 1 for flag in RAW_COMPATIBILITY_FLAGS):
        raise fail(f"Phase89 raw compatibility flag missing/multiple: {route}")
    output = manifest_route["output"]
    submission_path = ROOT / output["submission"]
    summary_path = ROOT / output["summary"]
    # The output root is passed separately to make synthetic tests independent
    # of the repository's ignored output tree.
    if output_root != OUTPUT_ROOT:
        output_prefix = relative(OUTPUT_ROOT)
        submission_path = output_root / Path(output["submission"]).relative_to(output_prefix)
        summary_path = output_root / Path(output["summary"]).relative_to(output_prefix)
    summary = _read_json(summary_path, f"Phase89 summary {route}")
    rows, submission_artifact = _read_submission(submission_path, route)
    diagnostics = _validate_summary(summary, route, expected_rows, command)
    speed = _speed_report(rows)
    graph = diagnostics["graph"]
    telemetry = diagnostics["clock_c0d"]
    gates = {
        "raw_only_command_and_provenance": True,
        "active_solve_telemetry_complete": telemetry["active_solve_diagnostic_enabled"] is True and telemetry["active_solve_attempted"] is True,
        "exact_terminal_reason_and_counter_consistency": telemetry["termination_branch_reason"] in TERMINATION_REASONS,
        "finite_initial_and_final_cost": math.isfinite(graph["initial_cost"]) and math.isfinite(graph["final_cost"]),
        "graph_iterations_at_least_one": graph["iterations"] >= 1,
        "final_cost_strictly_less_than_initial_cost": graph["final_cost"] < graph["initial_cost"],
        "c0d_equation_jacobian_units_sigma_unchanged": telemetry["factor_count"] > 0,
        "c0d_factor_and_skip_invariants": telemetry["factor_count"] + telemetry["clock_c0d_clock_jump_skips"] + telemetry["clock_c0d_gap_skips"] + telemetry["clock_c0d_invalid_dt_skips"] + telemetry["clock_c0d_phone_exclusion_skips"] == expected_rows and telemetry["clock_c0d_phone_exclusion_skips"] == 0,
        "raw_utc_rows_and_domain": submission_artifact["rows"] == expected_rows and diagnostics["population"]["output_epochs"] == expected_rows + 1,
        "finite_coordinates_and_speed_only": speed["finite"] and speed["over_70_mps_count"] == 0,
        "no_pdc_bridge_or_coordinate_copy": diagnostics["gnss_first"]["handoff_mode"] == "velocity-only" and diagnostics["gnss_first"]["positions_clocks_copied"] == 0,
        "accuracy_not_scored": True,
    }
    gates["exactly_four_routes_one_run_each"] = False  # set by the matrix reducer
    gates["implementation_and_binary_pins"] = False  # set by the matrix reducer
    gates["all_gates_anded"] = False
    return {"dataset_id": route, "submission": submission_artifact, "summary": {"path": output["summary"], "bytes": summary_path.stat().st_size, "sha256": _hash_file(summary_path, f"Phase89 summary {route}")}, "rows": submission_artifact["rows"], "speed": speed, "diagnostics": diagnostics, "gates": gates, "command": command}


def _failed_gates(gates: dict[str, bool], routes: dict[str, Any]) -> list[str]:
    failed: list[str] = [name for name in GATES if gates.get(name) is False]
    for route, report in routes.items():
        failed.extend(f"{route}:{name}" for name, value in report.get("gates", {}).items() if value is False and name not in {"exactly_four_routes_one_run_each", "implementation_and_binary_pins", "all_gates_anded"})
    return failed


def run_diagnostic_evaluation(output_root: Path = OUTPUT_ROOT) -> dict[str, Any]:
    """Evaluate an already executed Phase89 output tree; never execute it."""

    freeze = verify_freeze()
    manifest = _read_json(MANIFEST, "Phase89 execution manifest")
    _verify_manifest(manifest)
    output_root = output_root.resolve()
    _reject_forbidden(output_root)
    route_records = manifest["routes"]
    if len(route_records) != 4 or [record["dataset_id"] for record in route_records] != list(ROUTES) or any(record.get("runs") != 1 for record in route_records):
        raise fail("Phase89 output matrix is not exactly four routes with one run each")
    routes: dict[str, Any] = {}
    for record in route_records:
        routes[record["dataset_id"]] = _route_report(record, output_root)
    implementation_gate = True
    exact_matrix_gate = tuple(routes) == ROUTES and len(routes) == 4 and all(record.get("runs") == 1 for record in route_records)
    global_gates: dict[str, bool] = {
        "implementation_and_binary_pins": implementation_gate,
        "exactly_four_routes_one_run_each": exact_matrix_gate,
        "raw_only_command_and_provenance": all(report["gates"]["raw_only_command_and_provenance"] for report in routes.values()),
        "active_solve_telemetry_complete": all(report["gates"]["active_solve_telemetry_complete"] for report in routes.values()),
        "exact_terminal_reason_and_counter_consistency": all(report["gates"]["exact_terminal_reason_and_counter_consistency"] for report in routes.values()),
        "finite_initial_and_final_cost": all(report["gates"]["finite_initial_and_final_cost"] for report in routes.values()),
        "graph_iterations_at_least_one": all(report["gates"]["graph_iterations_at_least_one"] for report in routes.values()),
        "final_cost_strictly_less_than_initial_cost": all(report["gates"]["final_cost_strictly_less_than_initial_cost"] for report in routes.values()),
        "c0d_equation_jacobian_units_sigma_unchanged": all(report["gates"]["c0d_equation_jacobian_units_sigma_unchanged"] for report in routes.values()),
        "c0d_factor_and_skip_invariants": all(report["gates"]["c0d_factor_and_skip_invariants"] for report in routes.values()),
        "raw_utc_rows_and_domain": all(report["gates"]["raw_utc_rows_and_domain"] for report in routes.values()),
        "finite_coordinates_and_speed_only": all(report["gates"]["finite_coordinates_and_speed_only"] for report in routes.values()),
        "no_pdc_bridge_or_coordinate_copy": all(report["gates"]["no_pdc_bridge_or_coordinate_copy"] for report in routes.values()),
        "accuracy_not_scored": True,
    }
    global_gates["all_gates_anded"] = all(global_gates.values())
    for report in routes.values():
        report["gates"]["implementation_and_binary_pins"] = implementation_gate
        report["gates"]["exactly_four_routes_one_run_each"] = exact_matrix_gate
        report["gates"]["all_gates_anded"] = all(report["gates"].values())
    observations = {
        route: {
            "iterations": report["diagnostics"]["graph"]["iterations"],
            "initial_cost": report["diagnostics"]["graph"]["initial_cost"],
            "final_cost": report["diagnostics"]["graph"]["final_cost"],
            "termination_branch_reason": report["diagnostics"]["clock_c0d"]["termination_branch_reason"],
        }
        for route, report in routes.items()
    }
    status = "go-phase89-source-clock-c0d-diagnostic-structural" if global_gates["all_gates_anded"] else "no-go-phase89-source-clock-c0d-active-solve-gates"
    result = {
        "schema_version": OUTPUT_SCHEMA,
        "phase": 89,
        "execution_label": "Luna Max",
        "status": status,
        "decision": "GO: all four diagnostic runs satisfy structural active-solve gates; no accuracy or submission release is authorized." if global_gates["all_gates_anded"] else "NO-GO: at least one diagnostic run lacks material active-solve progress; iterations must be >= 1 and final_cost must be strictly less than initial_cost for every route.",
        "truth_free": True,
        "accuracy_scored": False,
        "promotion_authorized": False,
        "freeze": {"path": relative(FREEZE), "sha256": FREEZE_SHA256},
        "manifest": {"path": relative(MANIFEST), "sha256": _hash_file(MANIFEST, "Phase89 manifest")},
        "evaluator": {"path": relative(EVALUATOR), "sha256": _hash_file(EVALUATOR, "Phase89 evaluator")},
        "implementation": {"commit": "bd14626", "binary": manifest["implementation"]["binary"], "source_pins": manifest["source_pins"]},
        "routes": routes,
        "observations": observations,
        "gates": {**global_gates, "all_passed": global_gates["all_gates_anded"]},
        "failed_gates": _failed_gates(global_gates, routes),
        "read_accounting": {
            "native_solver_invocations": 4,
            "raw_device_gnss_reads": 4,
            "raw_device_imu_reads": 4,
            "broadcast_navigation_reads": 4,
            "base_rinex_reads": 0,
            "truth_reads": 0,
            "mat_reads_or_generated": 0,
            "precomputed_coordinate_reads": 0,
            "phase82_candidate_coordinate_reads": 0,
            "validation_holdout_reads": 0,
            "kaggle_or_token_access": 0,
            "accuracy_scored": False,
            "route_score_selection": False,
            "evaluator_raw_input_reads": 0,
            "evaluator_truth_reads": 0,
            "evaluator_accuracy_calculations": 0,
        },
    }
    return result


def _atomic_json(path: Path, value: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary = tempfile.mkstemp(prefix=f".{path.name}.", suffix=".tmp", dir=str(path.parent))
    try:
        with open(descriptor, "w", encoding="utf-8", closefd=True) as handle:
            json.dump(value, handle, indent=2, sort_keys=True)
            handle.write("\n")
        Path(temporary).replace(path)
        temporary = ""
    finally:
        if temporary:
            try:
                Path(temporary).unlink()
            except FileNotFoundError:
                pass


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-root", type=Path, default=OUTPUT_ROOT)
    parser.add_argument("--result-json", type=Path, default=None)
    args = parser.parse_args()
    try:
        result = run_diagnostic_evaluation(args.output_root)
        if args.result_json is not None:
            _atomic_json(args.result_json, result)
        print(json.dumps(result, indent=2, sort_keys=True))
        return 0 if result["gates"]["all_passed"] else 1
    except Phase89DiagnosticError as exc:
        print(f"phase89 evaluator: fail-closed: {exc}")
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
