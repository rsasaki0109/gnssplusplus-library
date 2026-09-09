#!/usr/bin/env python3
"""Fail-closed, raw-free structural evaluator for the Phase91 candidate.

The future Phase91 matrix is exactly one diagnostic-only run for each frozen
Phase80/85 route.  This module verifies the sealed contract and, after an
authorized run, reads only native summaries/submissions.  It never launches a
native process, opens or hashes raw GNSS/IMU/navigation inputs, reads truth,
MAT/Kaggle/Phase82 artifacts, or computes accuracy.
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
INITIAL_FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_phase91_source_clock_c0d_gnss_first_in_memory_raw_drift_d_initializer_freeze_v1.json"
INITIAL_FREEZE_SHA256 = "e218864172b789d67ce5961ebfe67457c267960be8da5f27f5b6c017ebba906b"
PHASE85_MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase85_source_clock_c0d_structural_manifest_v1.json"
PHASE85_MANIFEST_SHA256 = "20adc2e3388b92d045580739d025106c18aa068fcf26f8d117d48d3d2f5f7058"
FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_phase91_source_clock_c0d_gnss_first_raw_drift_d_initializer_execution_freeze_v1.json"
FREEZE_SHA256 = "456634b0a41da74de557185620bcb4d8c7b0572ebb7f3f1e415d8b8c82aa6fa4"
MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase91_source_clock_c0d_gnss_first_raw_drift_d_initializer_execution_manifest_v1.json"
EVALUATOR = Path(__file__).resolve()
FOCUSED_TESTS = ROOT / "tests/test_smartphone_phase91_source_clock_c0d_gnss_first_raw_drift_d_initializer.py"
TESTS_CMAKE = ROOT / "tests/CMakeLists.txt"
BINARY = ROOT / "build/apps/gnss_fgo_imu_no_base"
OUTPUT_ROOT = ROOT / "output/smartphone-r5/phase91-source-clock-c0d-gnss-first-raw-drift-d-initializer-v1"
OUTPUT_SCHEMA = "smartphone-r5-phase91-source-clock-c0d-gnss-first-raw-drift-d-initializer-result.v1"
IMPLEMENTATION_COMMIT = "0b9c4ebc5704428421f0bfc1cad569d834690b6d"

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

SELECTOR = "--native-source-clock-c0d-gnss-first-raw-drift-d-initializer"
CLOCK_FLAG = "--native-source-clock-c0d-factor"
ACTIVE_FLAG = "--native-source-clock-c0d-active-solve-diagnostic"
DIRECT_FLAG = "--native-source-direct-observable-quality"
NO_BRIDGE_FLAG = "--native-pdc-imu-tdcp-no-bridge"
RAW_UTC_FLAG = "--android-raw-utc-keys"
COMPATIBILITY_FLAGS = ("--android-raw-clock-only", "--android-utc-wall-clock-fallback")
FORBIDDEN_FLAGS = (
    "--obs",
    "--native-gnss-first-velocity-only-handoff",
    "--native-direct-doppler-wls-handoff",
    "--native-pdc-state-bridge",
    "--native-upstream-quality",
    "--native-base-rinex",
    "--native-base-pseudorange-compensation",
    "--native-base-pseudorange-source-miss-mask",
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
SPEED_OF_LIGHT_MPS = 299792458.0
C0D_SIGMA_SECONDS = 0.1 / SPEED_OF_LIGHT_MPS
EARTH_RADIUS_M = 6_371_000.0
MAX_SPEED_MPS = 70.0

REQUIRED_FLAGS = (
    "--android-gnss <frozen raw device_gnss.csv>",
    "--android-imu <frozen raw device_imu.csv>",
    "--nav <frozen broadcast brdc.nav>",
    "--dataset-id <one of the four frozen route IDs>",
    "--all-epochs",
    "--android-raw-utc-keys",
    "--android-raw-clock-only",
    "--android-utc-wall-clock-fallback",
    "--native-pdc-imu-tdcp-no-bridge",
    "--native-source-direct-observable-quality",
    "--native-source-clock-c0d-factor",
    "--native-source-clock-c0d-active-solve-diagnostic",
    SELECTOR,
)
SOURCE_PINS = {
    "apps/native/gnss_fgo_imu_no_base.cpp": "825d3234a018cfac33e2e2b4afb1f4f2b00e1e339627f2eb995b7769ee125ad6",
    "include/libgnss++/algorithms/fgo.hpp": "1198431b62b8e356ed1c34474da93ca2a9a99bd80288c2035df77f607305cc69",
    "include/libgnss++/algorithms/fgo_config.hpp": "f1d245b9002f8176d23bdaeca1895d2a8b7dac9726a79d55203ceac0562cecc8",
    "include/libgnss++/algorithms/source_clock_c0d_initializer.hpp": "0431987c902c84909ddf34650f2ab306d3dc2c63f2afd288516ccadafcdbffba",
    "src/algorithms/fgo.cpp": "4776896c345153d94700ad88f8651d9a30e1ee4bbb6c916a5c95df0053506f06",
    "src/algorithms/fgo_gtsam_backend.cpp": "66b575f5c05fb89ae611f3e7c8f01d4028d5d416023e16ea51d62d4819eb7a9a",
    "tests/test_fgo_gtsam_backend.cpp": "5bb057869f2e6951afa10812c06f1bb97d1b1bf857d07dd3250347479d5b0510",
    "tests/test_smartphone_phase91_source_clock_c0d_gnss_first_raw_drift_implementation.py": "e66a27f03696d710c9f0f175252aa555e7194144d1c57c9b96df09a97b69502d",
}

GATES = (
    "implementation_and_binary_pins",
    "exactly_four_routes_one_run_each",
    "raw_only_command_and_provenance",
    "same_run_gnss_first_handoff",
    "raw_drift_d_initializer_coverage",
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


class Phase91DiagnosticError(ValueError):
    """Raised when the sealed Phase91 contract or output is invalid."""


def fail(message: str) -> Phase91DiagnosticError:
    return Phase91DiagnosticError(message)


def relative(path: Path) -> str:
    try:
        return str(path.relative_to(ROOT))
    except ValueError:
        return str(path)


def _reject_forbidden(path: Path | str) -> None:
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
    if expected_sha256 is not None and hashlib.sha256(payload).hexdigest() != expected_sha256:
        raise fail(f"{label} SHA-256 mismatch: {path}")
    return payload


def _hash_file(path: Path, label: str) -> str:
    return hashlib.sha256(_read_bytes(path, label)).hexdigest()


def _read_json(path: Path, label: str, expected_sha256: str | None = None) -> dict[str, Any]:
    try:
        value = json.loads(_read_bytes(path, label, expected_sha256).decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise fail(f"invalid {label}: {exc}") from exc
    if not isinstance(value, dict):
        raise fail(f"{label} is not an object")
    return value


def _finite_tree(value: Any, label: str) -> None:
    if value is None or isinstance(value, bool) or isinstance(value, str):
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


def verify_inherited_authority() -> None:
    initial = _read_json(INITIAL_FREEZE, "Phase91 implementation freeze", INITIAL_FREEZE_SHA256)
    if initial.get("phase") != 91 or initial.get("status") != "frozen-before-implementation" or initial.get("execution_label") != "Luna Max":
        raise fail("Phase91 implementation freeze identity changed")
    if initial.get("decision_boundary", {}).get("execution_authorized") is not False or initial.get("decision_boundary", {}).get("candidate_count") != 1:
        raise fail("Phase91 implementation freeze execution boundary changed")
    phase85 = _read_json(PHASE85_MANIFEST, "Phase85 manifest", PHASE85_MANIFEST_SHA256)
    if phase85.get("phase") != 85 or phase85.get("status") != "frozen-before-phase85-v2-raw-read" or phase85.get("execution_label") != "Luna Max":
        raise fail("Phase85 authority identity changed")
    if phase85.get("routes") != list(ROUTES) or phase85.get("phase80_phase78_domain_rows") != DOMAIN_ROWS:
        raise fail("Phase85 route/domain authority changed")
    if phase85.get("matrix", {}).get("truth_reads") != 0 or phase85.get("matrix", {}).get("mat_reads_or_generated") != 0 or phase85.get("matrix", {}).get("precomputed_coordinate_reads") != 0:
        raise fail("Phase85 authority is not truth/coordinate-artifact free")


def verify_freeze() -> dict[str, Any]:
    if FREEZE_SHA256.startswith("__"):
        raise fail("Phase91 execution freeze digest has not been sealed")
    freeze = _read_json(FREEZE, "Phase91 execution freeze", FREEZE_SHA256)
    verify_inherited_authority()
    if freeze.get("schema_version") != "smartphone-r5-phase91-source-clock-c0d-gnss-first-raw-drift-d-initializer-execution-freeze.v1" or freeze.get("phase") != 91 or freeze.get("status") != "frozen-before-phase91-raw-execution" or freeze.get("execution_label") != "Luna Max":
        raise fail("Phase91 execution freeze identity changed")
    authority = freeze.get("authority", {})
    if authority.get("phase91_implementation_freeze", {}).get("sha256") != INITIAL_FREEZE_SHA256 or authority.get("phase85_manifest", {}).get("sha256") != PHASE85_MANIFEST_SHA256:
        raise fail("Phase91 inherited freeze pins changed")
    boundary = freeze.get("decision_boundary", {})
    expected_boundary = {
        "candidate_count": 1,
        "route_count": 4,
        "repetitions_per_route": 1,
        "controls": 0,
        "implementation_authorized": True,
        "execution_authorized": True,
        "before_raw_execution": True,
        "raw_execution_before_this_freeze": False,
        "accuracy_scoring": False,
        "route_score_selection": False,
        "promotion_or_accuracy_authorization": False,
        "fail_closed_if_missing_active_telemetry": True,
        "fail_closed_if_iterations_below_one": True,
        "fail_closed_if_cost_nonfinite": True,
        "fail_closed_if_final_cost_not_strictly_less": True,
        "stop_before_accuracy_or_submission": True,
    }
    if any(boundary.get(key) != value for key, value in expected_boundary.items()):
        raise fail("Phase91 execution decision boundary changed")
    candidate = freeze.get("candidate", {})
    if candidate.get("id") != "phase91_source_clock_c0d_gnss_first_in_memory_raw_drift_d_initializer" or candidate.get("selector") != SELECTOR or candidate.get("source_aligned") is not True or candidate.get("raw_only") is not True or candidate.get("diagnostic_only") is not True or candidate.get("default_off") is not True:
        raise fail("Phase91 execution candidate identity changed")
    if candidate.get("required_flags") != list(REQUIRED_FLAGS) or candidate.get("forbidden_flags") != [*FORBIDDEN_FLAGS, "any result-file, MAT, truth, device-WLS, or precomputed-coordinate input"]:
        raise fail("Phase91 execution candidate flags changed")
    routes = freeze.get("routes_and_domain", {})
    if routes.get("route_order") != list(ROUTES) or routes.get("domain_rows") != DOMAIN_ROWS or routes.get("exactly_one_run_each") is not True or routes.get("controls") != 0 or routes.get("route_score_selection") is not False:
        raise fail("Phase91 execution route/domain contract changed")
    auth = freeze.get("execution_authorization", {})
    for key, value in (("no_raw_execution_performed_at_freeze", True), ("no_accuracy_or_submission_release", True), ("raw_device_gnss_reads", 4), ("raw_device_imu_reads", 4), ("broadcast_navigation_reads", 4), ("base_rinex_reads", 0), ("truth_reads", 0), ("mat_reads_or_generated", 0), ("precomputed_coordinate_reads", 0), ("phase82_candidate_coordinate_reads", 0), ("validation_holdout_reads", 0), ("kaggle_or_token_access", 0), ("accuracy_scored", False), ("route_score_selection", False)):
        if auth.get(key) != value:
            raise fail(f"Phase91 execution authorization changed: {key}")
    accounting = freeze.get("read_accounting_at_freeze", {})
    zero_keys = ("raw_device_gnss_reads", "raw_device_imu_reads", "broadcast_navigation_reads", "base_rinex_reads", "native_solver_invocations", "native_reruns", "truth_reads", "mat_reads_or_generated", "precomputed_coordinate_reads", "phase82_candidate_coordinate_reads", "validation_holdout_reads", "kaggle_or_token_access")
    if any(accounting.get(key) != 0 for key in zero_keys) or accounting.get("accuracy_scored") is not False or accounting.get("route_score_selection") is not False:
        raise fail("Phase91 pre-raw read accounting changed")
    return freeze


def _verify_source_pins(manifest: dict[str, Any]) -> dict[str, Any]:
    implementation = manifest.get("implementation")
    if not isinstance(implementation, dict) or implementation.get("commit") != IMPLEMENTATION_COMMIT:
        raise fail("Phase91 implementation commit pin changed")
    binary = implementation.get("binary")
    if not isinstance(binary, dict) or binary.get("path") != relative(BINARY) or not isinstance(binary.get("sha256"), str) or len(binary["sha256"]) != 64:
        raise fail("Phase91 binary pin malformed")
    if _hash_file(ROOT / binary["path"], "Phase91 binary") != binary["sha256"]:
        raise fail("Phase91 binary hash changed")
    pins = manifest.get("source_pins")
    if pins != SOURCE_PINS:
        raise fail("Phase91 source pin map changed")
    for path_name, expected in pins.items():
        if _hash_file(ROOT / path_name, f"Phase91 source {path_name}") != expected:
            raise fail(f"Phase91 source hash changed: {path_name}")
    evaluator = manifest.get("evaluator")
    if not isinstance(evaluator, dict) or evaluator.get("path") != relative(EVALUATOR) or evaluator.get("sha256") != _hash_file(EVALUATOR, "Phase91 evaluator") or evaluator.get("accuracy_scoring") is not False:
        raise fail("Phase91 evaluator pin changed")
    focused = manifest.get("focused_tests")
    if not isinstance(focused, dict) or focused.get("path") != relative(FOCUSED_TESTS) or focused.get("sha256") != _hash_file(FOCUSED_TESTS, "Phase91 focused tests"):
        raise fail("Phase91 focused-test pin changed")
    cmake = manifest.get("cmake")
    if not isinstance(cmake, dict) or cmake.get("path") != relative(TESTS_CMAKE) or cmake.get("sha256") != _hash_file(TESTS_CMAKE, "Phase91 CMake"):
        raise fail("Phase91 CMake pin changed")
    freeze = manifest.get("freeze")
    if not isinstance(freeze, dict) or freeze.get("path") != relative(FREEZE) or freeze.get("sha256") != FREEZE_SHA256:
        raise fail("Phase91 manifest freeze pin changed")
    return {"commit": IMPLEMENTATION_COMMIT, "binary": binary, "source_pins": pins, "evaluator": evaluator}


def _verify_manifest(manifest: dict[str, Any]) -> dict[str, Any]:
    if manifest.get("schema_version") != "smartphone-r5-phase91-source-clock-c0d-gnss-first-raw-drift-d-initializer-execution-manifest.v1" or manifest.get("phase") != 91 or manifest.get("status") != "sealed-before-phase91-raw-execution" or manifest.get("execution_label") != "Luna Max":
        raise fail("Phase91 manifest identity changed")
    if manifest.get("purpose") != "Pin exactly one diagnostic-only raw execution for each of the four Phase80/85 routes; no accuracy or submission scoring is authorized.":
        raise fail("Phase91 manifest purpose changed")
    candidate = manifest.get("candidate_contract", {})
    if candidate.get("id") != "phase91_source_clock_c0d_gnss_first_in_memory_raw_drift_d_initializer" or candidate.get("selector") != SELECTOR or candidate.get("raw_only") is not True or candidate.get("diagnostic_only") is not True or candidate.get("accuracy_scoring") is not False or candidate.get("route_score_selection") is not False:
        raise fail("Phase91 manifest candidate provenance changed")
    if candidate.get("required_flags") != list(REQUIRED_FLAGS) or candidate.get("compatibility_flags") != list(COMPATIBILITY_FLAGS) or candidate.get("forbidden_flags") != [*FORBIDDEN_FLAGS, "any result-file, MAT, truth, device-WLS, or precomputed-coordinate input"]:
        raise fail("Phase91 manifest command contract changed")
    preserved = candidate.get("preserved_source_contract")
    expected_preserved = {
        "c0d_equation": "(c2-c1)-((d1+d2)*dt/(2*C_LIGHT))",
        "c0d_jacobian_order": ["c1", "c2", "d1", "d2"],
        "c0d_jacobian": "[-1,+1,-dt/(2*C_LIGHT),-dt/(2*C_LIGHT)]",
        "c0d_clock_units": "seconds",
        "c0d_drift_units": "metres_per_second",
        "c0d_dt_units": "seconds",
        "c0d_residual_units": "seconds",
        "c0d_sigma_m": 0.1,
        "c0d_sigma_seconds": "0.1/C_LIGHT",
        "direct_quality_flag": DIRECT_FLAG,
        "optimizer_algorithm": "unchanged GTSAM batch Levenberg-Marquardt",
        "source_sigma_tuning": False,
        "equation_or_jacobian_rewrite": False,
        "coordinate_bridge": False,
    }
    if preserved != expected_preserved:
        raise fail("Phase91 preserved C0/D contract changed")
    matrix = manifest.get("matrix")
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
        raise fail("Phase91 matrix/read contract changed")
    auth = manifest.get("execution_authorization", {})
    if auth.get("before_raw_execution") is not True or auth.get("raw_execution_authorized") is not True or auth.get("native_route_rerun_performed") is not False or auth.get("exactly_one_run_each") is not True or auth.get("stop_before_accuracy_or_submission") is not True:
        raise fail("Phase91 manifest execution authorization changed")
    routes = manifest.get("routes")
    if not isinstance(routes, list) or len(routes) != 4 or [item.get("dataset_id") for item in routes if isinstance(item, dict)] != list(ROUTES):
        raise fail("Phase91 manifest route order changed")
    for item in routes:
        route = item.get("dataset_id") if isinstance(item, dict) else None
        if not isinstance(item, dict) or route not in ROUTES or item.get("runs") != 1 or item.get("domain_rows") != DOMAIN_ROWS[route] or item.get("diagnostic_only") is not True:
            raise fail(f"Phase91 route record changed: {route}")
        raw = item.get("raw_inputs")
        if not isinstance(raw, dict) or set(raw) != {"device_gnss.csv", "device_imu.csv", "brdc.nav"}:
            raise fail(f"Phase91 raw input set changed: {route}")
        for name, pin in raw.items():
            if not isinstance(pin, dict) or not isinstance(pin.get("path"), str) or Path(pin["path"]).is_absolute() or not isinstance(pin.get("sha256"), str) or len(pin["sha256"]) != 64:
                raise fail(f"Phase91 raw input pin malformed: {route}/{name}")
            _reject_forbidden(pin["path"])
        output = item.get("output")
        if not isinstance(output, dict) or set(output) != {"submission", "summary"}:
            raise fail(f"Phase91 output pin malformed: {route}")
        for path in output.values():
            if not isinstance(path, str) or Path(path).is_absolute() or not path.startswith("output/smartphone-r5/phase91-source-clock-c0d-gnss-first-raw-drift-d-initializer-v1/"):
                raise fail(f"Phase91 output path leaves output root: {route}")
        command = item.get("command")
        if not isinstance(command, list) or not all(isinstance(token, str) for token in command):
            raise fail(f"Phase91 command missing: {route}")
        if command[0] != relative(BINARY) or command.count("--dataset-id") != 1 or route not in command:
            raise fail(f"Phase91 command identity failed: {route}")
        for flag, name in (("--android-gnss", "device_gnss.csv"), ("--android-imu", "device_imu.csv"), ("--nav", "brdc.nav")):
            try:
                value = command[command.index(flag) + 1]
            except (ValueError, IndexError) as exc:
                raise fail(f"Phase91 raw command argument missing: {route}/{flag}") from exc
            if value != raw[name]["path"]:
                raise fail(f"Phase91 raw command path differs from pin: {route}/{name}")
        for flag in ("--all-epochs", RAW_UTC_FLAG, NO_BRIDGE_FLAG, DIRECT_FLAG, CLOCK_FLAG, ACTIVE_FLAG, SELECTOR, *COMPATIBILITY_FLAGS):
            if command.count(flag) != 1:
                raise fail(f"Phase91 required flag missing/multiple: {route}/{flag}")
        if any(flag in command for flag in FORBIDDEN_FLAGS) or any(any(term in token.lower() for term in (".mat", "truth", "kaggle", "phase82", "base.obs", "coordinate")) for token in command):
            raise fail(f"Phase91 forbidden command token: {route}")
        if command.count("--out") != 1 or command.count("--summary-json") != 1:
            raise fail(f"Phase91 output flags missing/multiple: {route}")
    _verify_source_pins(manifest)
    return manifest


def _validate_provenance(summary: dict[str, Any], route: str, command: list[str]) -> dict[str, Any]:
    if summary.get("dataset_id") != route or summary.get("status") != "imu-combined-factor" or summary.get("truth_used") is not False or summary.get("production_default_changed") is not False:
        raise fail(f"Phase91 summary identity/truth/default gate failed: {route}")
    if summary.get("native_source_clock_c0d_gnss_first_raw_drift_d_initializer_enabled") is not True or summary.get("native_source_clock_c0d_factor_enabled") is not True or summary.get("native_source_clock_c0d_active_solve_diagnostic_enabled") is not True or summary.get("native_source_direct_observable_quality_enabled") is not True or summary.get("native_pdc_imu_tdcp_no_bridge") is not True or summary.get("native_pdc_imu_tdcp") is not True or summary.get("native_pdc_state_bridge") is not False or summary.get("native_upstream_quality") is not False:
        raise fail(f"Phase91 candidate flag telemetry failed: {route}")
    if summary.get("native_direct_doppler_wls_handoff") is True or summary.get("native_gnss_first_velocity_only_handoff") is True:
        raise fail(f"Phase91 alternate handoff telemetry failed: {route}")
    inputs = summary.get("inputs")
    if not isinstance(inputs, dict) or inputs.get("observation") is not None or not all(isinstance(inputs.get(key), str) for key in ("android_gnss", "imu", "navigation")):
        raise fail(f"Phase91 raw input provenance missing: {route}")
    for key in ("android_gnss", "imu", "navigation"):
        _reject_forbidden(inputs[key])
        if any(term in inputs[key].lower() for term in ("base", "coordinate")):
            raise fail(f"Phase91 forbidden input provenance: {route}/{key}")
    gnss_diag = summary.get("android_gnss_diagnostics")
    imu_init = summary.get("imu_initialization")
    if not isinstance(gnss_diag, dict) or gnss_diag.get("no_device_wls_seed") is not True or not isinstance(imu_init, dict) or imu_init.get("input_format") != "android-device_imu.csv":
        raise fail(f"Phase91 raw GNSS/IMU provenance telemetry missing: {route}")
    return {"inputs": inputs, "gnss_diagnostics": gnss_diag, "imu_initialization": imu_init}


def _validate_handoff(summary: dict[str, Any], route: str, expected_epochs: int) -> dict[str, Any]:
    handoff = summary.get("gnss_first")
    if not isinstance(handoff, dict) or handoff.get("attempted") is not True or handoff.get("converged") is not True or handoff.get("handoff_mode") != "gnss-first-in-memory-position-clock-velocity" or handoff.get("velocity_handoff_source") != "same-run-gnss-first-optimizer-result" or handoff.get("position_clock_handoff_source") != "same-run-gnss-first-optimizer-result" or handoff.get("raw_drift_d_initializer") != "main-graph-only" or handoff.get("epoch_identity_alignment_valid") is not True:
        raise fail(f"Phase91 GNSS-first handoff provenance failed: {route}")
    exact_counts = ("main_epoch_count", "gnss_first_epoch_count", "solution_epoch_count", "raw_epoch_count", "raw_utc_key_count", "aligned_epoch_count")
    if any(_count(handoff, key, f"handoff/{route}") != expected_epochs for key in exact_counts):
        raise fail(f"Phase91 GNSS-first epoch count failed: {route}")
    zero_counts = ("epoch_count_mismatch_count", "nonfinite_time_count", "gnss_first_time_mismatch_count", "raw_time_mismatch_count", "raw_utc_key_order_mismatch_count", "duplicate_raw_utc_key_count", "raw_utc_key_mismatch_count", "nonfinite_solution_count")
    if any(_count(handoff, key, f"handoff/{route}") != 0 for key in zero_counts):
        raise fail(f"Phase91 exact epoch identity failure: {route}")
    if _count(handoff, "positions_clocks_copied", f"handoff/{route}") != expected_epochs or handoff.get("coordinates_source") != "in-memory GNSS-first result only" or handoff.get("forbidden_coordinate_sources") != ["PDC", "direct-WLS", "external", "precomputed"] or handoff.get("failure") != "":
        raise fail(f"Phase91 position/clock handoff contract failed: {route}")
    return {key: handoff[key] for key in (*exact_counts, *zero_counts, "positions_clocks_copied", "handoff_mode", "velocity_handoff_source", "position_clock_handoff_source")}


def _validate_raw_d(summary: dict[str, Any], route: str, expected_epochs: int) -> dict[str, Any]:
    c0d = summary.get("native_source_clock_c0d_factor")
    if not isinstance(c0d, dict) or c0d.get("clock_c0d_enabled") is not True:
        raise fail(f"Phase91 C0/D telemetry missing: {route}")
    raw = c0d.get("raw_drift_d_initializer")
    if not isinstance(raw, dict) or raw.get("enabled") is not True or raw.get("attempted") is not True or raw.get("coverage_valid") is not True or raw.get("source_field") != "EpochSeed.receiver_clock_drift_mps" or raw.get("units") != "metres_per_second" or raw.get("fallback") != "none" or raw.get("failure") != "":
        raise fail(f"Phase91 raw-D initializer provenance failed: {route}")
    for key in ("epoch_count", "finite_count"):
        if _count(raw, key, f"raw-d/{route}") != expected_epochs:
            raise fail(f"Phase91 raw-D coverage count failed: {route}/{key}")
    if _count(raw, "nonfinite_count", f"raw-d/{route}") != 0:
        raise fail(f"Phase91 raw-D nonfinite coverage: {route}")
    minimum, maximum = _number(raw, "min_mps", f"raw-d/{route}"), _number(raw, "max_mps", f"raw-d/{route}")
    if minimum > maximum:
        raise fail(f"Phase91 raw-D range invalid: {route}")
    return {"epoch_count": raw["epoch_count"], "finite_count": raw["finite_count"], "nonfinite_count": raw["nonfinite_count"], "min_mps": minimum, "max_mps": maximum, "source_field": raw["source_field"]}


def _validate_active_solve(c0d: dict[str, Any], graph: dict[str, Any], route: str, expected_rows: int) -> dict[str, Any]:
    if c0d.get("active_solve_diagnostic_enabled") is not True or c0d.get("active_solve_attempted") is not True or c0d.get("active_solve_finite_costs") is not True or c0d.get("termination_trace_complete") is not True:
        raise fail(f"Phase91 active-solve telemetry incomplete: {route}")
    initial = _number(c0d, "active_solve_initial_cost", f"active-solve/{route}")
    final = _number(c0d, "active_solve_final_cost", f"active-solve/{route}")
    for key in ("initial_lambda", "maximum_lambda", "final_lambda", "max_whitened_clock_column_norm", "max_whitened_drift_column_norm", "conditioning_proxy"):
        _number(c0d, key, f"active-solve/{route}", positive=True)
    accepted = _count(c0d, "accepted_outer_iterations", f"active-solve/{route}")
    attempts = _count(c0d, "total_inner_lambda_attempts", f"active-solve/{route}")
    indeterminate = _count(c0d, "indeterminate_linear_solve_count", f"active-solve/{route}")
    for key in ("unsuccessful_model_step_count", "small_cost_change_stop_count", "maximum_lambda_stop_count"):
        _count(c0d, key, f"active-solve/{route}")
    reason = c0d.get("termination_branch_reason")
    if reason not in TERMINATION_REASONS:
        raise fail(f"Phase91 unknown terminal reason: {route}/{reason}")
    if reason == "small_cost_change" and (c0d["small_cost_change_stop_count"] != 1 or c0d["maximum_lambda_stop_count"] != 0):
        raise fail(f"Phase91 small-cost counters inconsistent: {route}")
    if reason == "maximum_lambda" and (c0d["maximum_lambda_stop_count"] != 1 or c0d["small_cost_change_stop_count"] != 0):
        raise fail(f"Phase91 maximum-lambda counters inconsistent: {route}")
    if reason not in {"small_cost_change", "maximum_lambda"} and (c0d["small_cost_change_stop_count"] != 0 or c0d["maximum_lambda_stop_count"] != 0):
        raise fail(f"Phase91 terminal counters inconsistent: {route}/{reason}")
    if attempts < accepted + indeterminate:
        raise fail(f"Phase91 inner-attempt accounting inconsistent: {route}")
    iterations = _count(graph, "iterations", f"graph/{route}")
    graph_initial = _number(graph, "initial_cost", f"graph/{route}")
    graph_final = _number(graph, "final_cost", f"graph/{route}")
    if graph_initial != initial or graph_final != final or accepted != iterations:
        raise fail(f"Phase91 graph/active-solve identity failed: {route}")
    return {"initial_cost": initial, "final_cost": final, "iterations": iterations, "accepted_outer_iterations": accepted, "total_inner_lambda_attempts": attempts, "termination_branch_reason": reason, "telemetry_complete": True}


def _validate_c0d(summary: dict[str, Any], route: str, expected_rows: int) -> tuple[dict[str, Any], dict[str, Any]]:
    c0d = summary["native_source_clock_c0d_factor"]
    factor_count = _count(c0d, "clock_c0d_factor_count", route)
    skip_keys = ("clock_c0d_clock_jump_skips", "clock_c0d_gap_skips", "clock_c0d_invalid_dt_skips", "clock_c0d_phone_exclusion_skips")
    skips = {key: _count(c0d, key, route) for key in skip_keys}
    if skips["clock_c0d_phone_exclusion_skips"] != 0 or factor_count + sum(skips.values()) != expected_rows:
        raise fail(f"Phase91 C0/D factor/skip accounting failed: {route}")
    dt_min = _number(c0d, "clock_c0d_dt_min_s", f"clock-c0d/{route}", positive=True)
    dt_max = _number(c0d, "clock_c0d_dt_max_s", f"clock-c0d/{route}", positive=True)
    if not dt_min <= dt_max < 1.5:
        raise fail(f"Phase91 C0/D dt range invalid: {route}")
    expected = {
        "clock_c0d_equation": "(c2-c1)-((d1+d2)*dt/(2*C_LIGHT))",
        "clock_c0d_jacobian_order": ["c1", "c2", "d1", "d2"],
        "clock_c0d_jacobian": "[-1,+1,-dt/(2*C_LIGHT),-dt/(2*C_LIGHT)]",
        "clock_c0d_units": {"clock": "seconds", "drift": "metres_per_second", "dt": "seconds", "residual": "seconds", "sigma": "seconds"},
        "clock_jump_noise": "Inf (active C0 factor omitted)",
        "parity_scope": "C0/D active-row parity; not full seven-vector",
        "legacy_scalar_clock_between_factor_count": 0,
    }
    if any(c0d.get(key) != value for key, value in expected.items()) or not math.isclose(_number(c0d, "speed_of_light_mps", route), SPEED_OF_LIGHT_MPS, rel_tol=0.0, abs_tol=1e-9) or not math.isclose(_number(c0d, "clock_c0d_sigma_seconds", route), C0D_SIGMA_SECONDS, rel_tol=1e-12, abs_tol=1e-18):
        raise fail(f"Phase91 C0/D equation/Jacobian/units/sigma contract failed: {route}")
    return c0d, {"factor_count": factor_count, **skips, "dt_min_s": dt_min, "dt_max_s": dt_max}


def _validate_summary(summary: dict[str, Any], route: str, expected_rows: int, command: list[str]) -> dict[str, Any]:
    _finite_tree(summary, f"summary[{route}]")
    provenance = _validate_provenance(summary, route, command)
    epochs, graph, tdcp, utc, direct = (summary.get(key) for key in ("epochs", "graph", "tdcp_contract", "raw_utc_key_contract", "native_source_direct_observable_quality"))
    if not all(isinstance(value, dict) for value in (epochs, graph, tdcp, utc, direct)):
        raise fail(f"Phase91 structural telemetry missing: {route}")
    assert isinstance(epochs, dict) and isinstance(graph, dict) and isinstance(tdcp, dict) and isinstance(utc, dict) and isinstance(direct, dict)
    expected_epochs = expected_rows + 1
    if _count(epochs, "problem", route) != expected_epochs or _count(epochs, "output", route) != expected_epochs or _count(graph, "imu_intervals", route) != expected_rows:
        raise fail(f"Phase91 row/IMU interval invariant failed: {route}")
    if graph.get("converged") is not True:
        raise fail(f"Phase91 native graph did not converge: {route}")
    if direct.get("enabled") is not True or direct.get("direct_no_pdc") is not True or direct.get("pdc_bridge") is not False or direct.get("native_pdc_state_bridge") is not False:
        raise fail(f"Phase91 direct/no-PDC invariant failed: {route}")
    if tdcp.get("enabled") is not True or _count(tdcp, "factors_built", route) != _count(tdcp, "factors_inserted", route) or tdcp.get("finite_residuals") != tdcp.get("factors_inserted") or _count(tdcp, "nonfinite_residuals", route) != 0:
        raise fail(f"Phase91 TDCP invariant failed: {route}")
    if utc.get("warmup_epoch_excluded") is not True or utc.get("raw_epoch_keys") != expected_epochs or utc.get("target_epochs") != expected_rows or utc.get("exact_solution_epochs") != expected_rows or utc.get("interpolated_epochs") != 0 or utc.get("edge_hold_epochs") != 0 or utc.get("unresolved_epochs") != 0 or utc.get("device_wls_coordinates_used") is not False:
        raise fail(f"Phase91 raw UTC alignment invariant failed: {route}")
    handoff = _validate_handoff(summary, route, expected_epochs)
    raw_d = _validate_raw_d(summary, route, expected_epochs)
    c0d, skips = _validate_c0d(summary, route, expected_rows)
    active = _validate_active_solve(c0d, graph, route, expected_rows)
    return {
        "provenance": provenance,
        "population": {"problem_epochs": epochs["problem"], "output_epochs": epochs["output"], "imu_intervals": graph["imu_intervals"], "pseudorange_factors": epochs.get("pseudorange_factors")},
        "gnss_first": handoff,
        "raw_drift_d_initializer": raw_d,
        "clock_c0d": {**skips, **active, "active_solve_diagnostic_enabled": c0d["active_solve_diagnostic_enabled"], "active_solve_attempted": c0d["active_solve_attempted"]},
        "graph": {"initial_cost": active["initial_cost"], "final_cost": active["final_cost"], "iterations": active["iterations"], "converged": graph["converged"]},
    }


def _read_submission(path: Path, route: str) -> tuple[list[tuple[int, float, float]], dict[str, Any]]:
    payload = _read_bytes(path, f"Phase91 submission {route}")
    try:
        rows = list(csv.reader(io.StringIO(payload.decode("utf-8-sig"))))
    except UnicodeDecodeError as exc:
        raise fail(f"submission is not UTF-8: {route}") from exc
    if not rows or rows[0] != ["phone", "UnixTimeMillis", "LatitudeDegrees", "LongitudeDegrees"]:
        raise fail(f"submission header mismatch: {route}")
    parsed: list[tuple[int, float, float]] = []
    previous: int | None = None
    for line, fields in enumerate(rows[1:], 2):
        if len(fields) != 4 or fields[0] != route:
            raise fail(f"submission row key mismatch: {route}:{line}")
        try:
            timestamp, latitude, longitude = int(fields[1]), float(fields[2]), float(fields[3])
        except ValueError as exc:
            raise fail(f"non-numeric submission row: {route}:{line}") from exc
        if previous is not None and timestamp <= previous:
            raise fail(f"submission timestamps are not increasing: {route}")
        if not all(math.isfinite(value) for value in (latitude, longitude)) or not -90.0 <= latitude <= 90.0 or not -180.0 <= longitude <= 180.0:
            raise fail(f"invalid earth coordinate: {route}:{line}")
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
        hav = math.sin((lat2 - lat1) / 2.0) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin((lon2 - lon1) / 2.0) ** 2
        distance = EARTH_RADIUS_M * 2.0 * math.asin(math.sqrt(min(1.0, max(0.0, hav))))
        speeds.append(distance / dt)
    return {"finite": all(math.isfinite(speed) for speed in speeds), "max_speed_mps": max(speeds, default=0.0), "over_70_mps_count": sum(speed > MAX_SPEED_MPS for speed in speeds), "transition_count": len(speeds)}


def _route_report(record: dict[str, Any], output_root: Path) -> dict[str, Any]:
    route = record["dataset_id"]
    command = record["command"]
    raw = record["raw_inputs"]
    for flag, name in (("--android-gnss", "device_gnss.csv"), ("--android-imu", "device_imu.csv"), ("--nav", "brdc.nav")):
        if command[command.index(flag) + 1] != raw[name]["path"]:
            raise fail(f"Phase91 route raw pin mismatch: {route}/{name}")
    if any(flag in command for flag in FORBIDDEN_FLAGS) or command.count(SELECTOR) != 1:
        raise fail(f"Phase91 route command gate failed: {route}")
    output = record["output"]
    prefix = "output/smartphone-r5/phase91-source-clock-c0d-gnss-first-raw-drift-d-initializer-v1/"
    if output_root == OUTPUT_ROOT:
        submission_path, summary_path = ROOT / output["submission"], ROOT / output["summary"]
    else:
        submission_path = output_root / Path(output["submission"]).relative_to(prefix)
        summary_path = output_root / Path(output["summary"]).relative_to(prefix)
    summary = _read_json(summary_path, f"Phase91 summary {route}")
    rows, submission = _read_submission(submission_path, route)
    diagnostics = _validate_summary(summary, route, DOMAIN_ROWS[route], command)
    speed = _speed_report(rows)
    graph, clock = diagnostics["graph"], diagnostics["clock_c0d"]
    gates = {
        "raw_only_command_and_provenance": True,
        "same_run_gnss_first_handoff": diagnostics["gnss_first"]["handoff_mode"] == "gnss-first-in-memory-position-clock-velocity" and diagnostics["gnss_first"]["positions_clocks_copied"] == DOMAIN_ROWS[route] + 1,
        "raw_drift_d_initializer_coverage": diagnostics["raw_drift_d_initializer"]["finite_count"] == DOMAIN_ROWS[route] + 1 and diagnostics["raw_drift_d_initializer"]["nonfinite_count"] == 0,
        "active_solve_telemetry_complete": clock["telemetry_complete"] and clock["active_solve_diagnostic_enabled"] and clock["active_solve_attempted"],
        "exact_terminal_reason_and_counter_consistency": clock["termination_branch_reason"] in TERMINATION_REASONS,
        "finite_initial_and_final_cost": math.isfinite(graph["initial_cost"]) and math.isfinite(graph["final_cost"]),
        "graph_iterations_at_least_one": graph["iterations"] >= 1,
        "final_cost_strictly_less_than_initial_cost": graph["final_cost"] < graph["initial_cost"],
        "c0d_equation_jacobian_units_sigma_unchanged": clock["factor_count"] > 0,
        "c0d_factor_and_skip_invariants": clock["factor_count"] + sum(clock[key] for key in ("clock_c0d_clock_jump_skips", "clock_c0d_gap_skips", "clock_c0d_invalid_dt_skips", "clock_c0d_phone_exclusion_skips")) == DOMAIN_ROWS[route] and clock["clock_c0d_phone_exclusion_skips"] == 0,
        "raw_utc_rows_and_domain": submission["rows"] == DOMAIN_ROWS[route] and diagnostics["population"]["output_epochs"] == DOMAIN_ROWS[route] + 1,
        "finite_coordinates_and_speed_only": speed["finite"] and speed["over_70_mps_count"] == 0,
        "no_pdc_bridge_or_coordinate_copy": diagnostics["gnss_first"]["positions_clocks_copied"] == DOMAIN_ROWS[route] + 1,
        "accuracy_not_scored": True,
    }
    gates["exactly_four_routes_one_run_each"] = False
    gates["implementation_and_binary_pins"] = False
    gates["all_gates_anded"] = False
    return {"dataset_id": route, "submission": submission, "summary": {"path": output["summary"], "bytes": summary_path.stat().st_size, "sha256": _hash_file(summary_path, f"Phase91 summary {route}")}, "rows": submission["rows"], "speed": speed, "diagnostics": diagnostics, "gates": gates, "command": command}


def _failed_gates(global_gates: dict[str, bool], routes: dict[str, Any]) -> list[str]:
    failed = [name for name in GATES if global_gates.get(name) is False]
    for route, report in routes.items():
        failed.extend(f"{route}:{name}" for name, value in report.get("gates", {}).items() if value is False and name not in {"exactly_four_routes_one_run_each", "implementation_and_binary_pins", "all_gates_anded"})
    return failed


def run_diagnostic_evaluation(output_root: Path = OUTPUT_ROOT) -> dict[str, Any]:
    """Evaluate an already executed output tree without launching anything."""

    verify_freeze()
    manifest = _read_json(MANIFEST, "Phase91 execution manifest")
    _verify_manifest(manifest)
    output_root = output_root.resolve()
    _reject_forbidden(output_root)
    records = manifest["routes"]
    routes: dict[str, Any] = {}
    for record in records:
        routes[record["dataset_id"]] = _route_report(record, output_root)
    matrix_gate = len(records) == 4 and [record["dataset_id"] for record in records] == list(ROUTES) and all(record.get("runs") == 1 for record in records)
    implementation_gate = True
    global_gates = {
        "implementation_and_binary_pins": implementation_gate,
        "exactly_four_routes_one_run_each": matrix_gate,
        "raw_only_command_and_provenance": all(report["gates"]["raw_only_command_and_provenance"] for report in routes.values()),
        "same_run_gnss_first_handoff": all(report["gates"]["same_run_gnss_first_handoff"] for report in routes.values()),
        "raw_drift_d_initializer_coverage": all(report["gates"]["raw_drift_d_initializer_coverage"] for report in routes.values()),
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
        report["gates"]["exactly_four_routes_one_run_each"] = matrix_gate
        report["gates"]["all_gates_anded"] = all(report["gates"].values())
    observations = {route: {"iterations": report["diagnostics"]["graph"]["iterations"], "initial_cost": report["diagnostics"]["graph"]["initial_cost"], "final_cost": report["diagnostics"]["graph"]["final_cost"], "termination_branch_reason": report["diagnostics"]["clock_c0d"]["termination_branch_reason"]} for route, report in routes.items()}
    status = "go-phase91-source-clock-c0d-gnss-first-raw-drift-d-initializer-diagnostic" if global_gates["all_gates_anded"] else "no-go-phase91-source-clock-c0d-gnss-first-raw-drift-d-initializer-gates"
    return {
        "schema_version": OUTPUT_SCHEMA,
        "phase": 91,
        "execution_label": "Luna Max",
        "status": status,
        "decision": "GO: all four diagnostic runs satisfy structural active-solve gates; no accuracy or submission release is authorized." if global_gates["all_gates_anded"] else "NO-GO: at least one diagnostic run fails a frozen structural gate; every route requires complete active telemetry, iterations >= 1, and final_cost < initial_cost.",
        "truth_free": True,
        "accuracy_scored": False,
        "promotion_authorized": False,
        "freeze": {"path": relative(FREEZE), "sha256": FREEZE_SHA256},
        "manifest": {"path": relative(MANIFEST), "sha256": _hash_file(MANIFEST, "Phase91 manifest")},
        "evaluator": {"path": relative(EVALUATOR), "sha256": _hash_file(EVALUATOR, "Phase91 evaluator")},
        "implementation": {"commit": IMPLEMENTATION_COMMIT, "binary": manifest["implementation"]["binary"], "source_pins": manifest["source_pins"]},
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
    except Phase91DiagnosticError as exc:
        print(f"phase91 evaluator: fail-closed: {exc}")
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
