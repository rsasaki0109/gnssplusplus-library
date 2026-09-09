#!/usr/bin/env python3
"""Launch-free contract for the Phase99 main-only multifrontal-QR lane.

This module verifies only committed source/record artifacts and sealed raw
path metadata.  It never launches the native program and never opens, hashes,
copies, or transforms a raw GNSS, IMU, or navigation file.  Raw execution is
authorized only by the independent Phase99 authorization record.
"""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import sys
from typing import Any


ROOT = Path(__file__).resolve().parents[3]
FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_phase99_main_multifrontal_qr_solver_candidate_freeze_v1.json"
MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase99_main_multifrontal_qr_solver_execution_manifest_v1.json"
AUTHORIZATION = ROOT / "docs/use_cases/records/smartphone_r5_phase99_main_multifrontal_qr_solver_raw_execution_authorization_v1.json"
PHASE95_RESULT = ROOT / "docs/use_cases/records/smartphone_r5_phase95_raw_input_path_corrected_structural_result_v1.json"
PHASE95_WRAPPER = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase95_raw_input_path_corrected_execute.py"
PHASE91_MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase91_source_clock_c0d_gnss_first_raw_drift_d_initializer_execution_manifest_v1.json"
EVALUATOR = Path(__file__).resolve()
WRAPPER = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase99_main_multifrontal_qr_execute.py"
FOCUSED_TESTS = ROOT / "tests/test_smartphone_phase99_main_multifrontal_qr.py"

APP = ROOT / "apps/native/gnss_fgo_imu_no_base.cpp"
BACKEND = ROOT / "src/algorithms/fgo_gtsam_backend.cpp"
CONFIG = ROOT / "include/libgnss++/algorithms/fgo_config.hpp"
BINARY = ROOT / "build/apps/gnss_fgo_imu_no_base"

FREEZE_SHA = "4e9819d172c4c39842f04d6202c815c9631041320f0b785e2cb2d6a67edf4c27"
FREEZE_COMMIT = "104ab766fffa9e6a5f4f75d54ba102483a81f4b8"
IMPLEMENTATION_COMMIT = "57c21d1028cac659e97f10e49f35a6ccfe39f999"
APP_SHA = "a2f87bae4b39a0d0027cb4df93883214a805ab5fc199611f7241c86ed6720741"
BACKEND_SHA = "38c555b82785aee3d1d5fd8a81ff3bde8f63fd0a9910d5f1786a7c68152d7f3f"
CONFIG_SHA = "e86c777061b558c1721de27a5912948f386ecd47c22c2ea225a50205d7f6cb3d"
BINARY_SHA = "c29a92c745726b3cc6362cffbdac3cf1277d85e1e5de860fafc1f6d70da71a0d"
PHASE95_RESULT_SHA = "beba25d4b0d910064d76bea7aa6b7a884937ca01c3b00cf0f7f9f4b6267ea281"
PHASE95_WRAPPER_SHA = "5ac858cf156555652f6feae60c71a4237269d4b4c2cb70c1adf5a9e08f10e883"
PHASE91_MANIFEST_SHA = "2078a4e2c3b963f07744c435ec303ea848bc372c8efd5003f6cb7b3d4b4ce988"

ROUTES = (
    "2021-03-16-18-59-us-ca-mtv-a/pixel5",
    "2022-04-01-18-22-us-ca-lax-t/pixel5",
)
DOMAIN_ROWS = {ROUTES[0]: 2158, ROUTES[1]: 1465}
RAW_NAMES = ("device_gnss.csv", "device_imu.csv", "brdc.nav")
RAW_FLAGS = (
    ("--android-gnss", "device_gnss.csv", "__PHASE95_RAW_DEVICE_GNSS__"),
    ("--android-imu", "device_imu.csv", "__PHASE95_RAW_DEVICE_IMU__"),
    ("--nav", "brdc.nav", "__PHASE95_RAW_BROADCAST_NAV__"),
)
REQUIRED_FLAGS = (
    "--all-epochs",
    "--android-raw-utc-keys",
    "--android-raw-clock-only",
    "--android-utc-wall-clock-fallback",
    "--native-pdc-imu-tdcp-no-bridge",
    "--native-source-direct-observable-quality",
    "--native-source-clock-c0d-factor",
    "--native-source-clock-c0d-meter-state-parity",
    "--native-source-clock-c0d-active-solve-diagnostic",
    "--native-source-clock-c0d-gnss-first-meter-state-handoff",
    "--native-source-clock-c0d-phase96-main-diagnostics",
    "--native-source-clock-c0d-phase98-solver-rank-diagnostic",
    "--native-source-clock-c0d-phase99-main-multifrontal-qr-solver",
)
FORBIDDEN_FLAGS = (
    "--obs",
    "--native-gnss-first-velocity-only-handoff",
    "--native-direct-doppler-wls-handoff",
    "--native-pdc-state-bridge",
    "--native-upstream-quality",
    "--native-base-rinex",
    "--native-base-pseudorange-compensation",
    "--native-base-pseudorange-source-miss-mask",
    "--native-source-clock-c0d-gnss-first-raw-drift-d-initializer",
    "--native-source-clock-c0d-phase94-stage-diagnostics",
    "--native-source-clock-c0d-phase97-singular-system-diagnostics",
)
FORBIDDEN_PATH_TERMS = (
    ".mat", "ground_truth", "validation", "holdout", "kaggle", "token",
    "base.rinex", "coordinate", "truth",
)
SELECTOR = "--native-source-clock-c0d-phase99-main-multifrontal-qr-solver"
PHASE93_SELECTOR = "--native-source-clock-c0d-gnss-first-meter-state-handoff"
PHASE96_SELECTOR = "--native-source-clock-c0d-phase96-main-diagnostics"
PHASE98_SELECTOR = "--native-source-clock-c0d-phase98-solver-rank-diagnostic"
OUTPUT_ROOT = "output/smartphone-r5/phase99-main-multifrontal-qr-v1/"
SCHEMA = "smartphone-r5-phase99-main-multifrontal-qr-execution-manifest.v1"
AUTH_SCHEMA = "smartphone-r5-phase99-main-multifrontal-qr-raw-execution-authorization.v1"
CANDIDATE_ID = "phase99-main-meter-state-multifrontal-qr-branch-only"


class Phase99PreRawError(ValueError):
    """Raised when the Phase99 contract fails closed."""


def fail(message: str) -> Phase99PreRawError:
    return Phase99PreRawError(message)


def relative(path: Path) -> str:
    try:
        return str(path.relative_to(ROOT))
    except ValueError:
        return str(path)


def sha256_file(path: Path, label: str) -> str:
    if path.name in RAW_NAMES:
        raise fail(f"raw-file hash is forbidden: {label}")
    if not path.is_file():
        raise fail(f"missing {label}: {path}")
    digest = hashlib.sha256()
    try:
        with path.open("rb") as handle:
            for chunk in iter(lambda: handle.read(1024 * 1024), b""):
                digest.update(chunk)
    except OSError as exc:
        raise fail(f"failed to hash {label}: {path}: {exc}") from exc
    return digest.hexdigest()


def read_json(path: Path, label: str) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise fail(f"failed to read {label}: {path}: {exc}") from exc
    if not isinstance(value, dict):
        raise fail(f"{label} is not an object")
    return value


def assert_equal(actual: Any, expected: Any, label: str) -> None:
    if actual != expected:
        raise fail(f"{label}: expected {expected!r}, got {actual!r}")


def _safe_raw_path(path_text: str, route: str, name: str) -> None:
    path = Path(path_text)
    if path.is_absolute() or ".." in path.parts or path.name != name:
        raise fail(f"unsafe Phase95 raw path: {route}/{name}: {path_text}")
    if any(term in path_text.lower() for term in FORBIDDEN_PATH_TERMS):
        raise fail(f"forbidden Phase95 raw path term: {route}/{name}")


def phase95_paths() -> dict[str, dict[str, str]]:
    """Read the sealed Phase95 path map without opening raw input files."""

    assert_equal(sha256_file(PHASE95_RESULT, "Phase95 structural result"), PHASE95_RESULT_SHA, "Phase95 result/sha256")
    result = read_json(PHASE95_RESULT, "Phase95 structural result")
    routes = result.get("routes")
    if not isinstance(routes, dict):
        raise fail("Phase95 result/routes is not an object")
    resolved: dict[str, dict[str, str]] = {}
    for route in ROUTES:
        record = routes.get(route)
        if not isinstance(record, dict):
            raise fail(f"Phase95 result route missing: {route}")
        raw = record.get("raw_inputs")
        if not isinstance(raw, dict) or set(raw) != set(RAW_NAMES):
            raise fail(f"Phase95 raw role set changed: {route}")
        resolved[route] = {}
        for name in RAW_NAMES:
            pin = raw.get(name)
            if not isinstance(pin, dict) or not isinstance(pin.get("path"), str):
                raise fail(f"Phase95 raw path missing: {route}/{name}")
            _safe_raw_path(pin["path"], route, name)
            resolved[route][name] = pin["path"]
    return resolved


def verify_freeze() -> dict[str, Any]:
    assert_equal(sha256_file(FREEZE, "Phase99 freeze"), FREEZE_SHA, "freeze/sha256")
    freeze = read_json(FREEZE, "Phase99 freeze")
    for key, expected in {
        "schema_version": "smartphone-r5-phase99-main-multifrontal-qr-solver-candidate-freeze.v1",
        "phase": 99,
        "execution_label": "Luna Max",
        "status": "frozen-candidate-not-implemented",
    }.items():
        assert_equal(freeze.get(key), expected, f"freeze/{key}")
    decision = freeze.get("decision")
    if not isinstance(decision, dict):
        raise fail("freeze/decision missing")
    for key, expected in {
        "exactly_one_candidate_frozen": True,
        "candidate_count": 1,
        "raw_execution_authorized": False,
        "solver_rerun_authorized": False,
        "accuracy_or_promotion_authorized": False,
    }.items():
        assert_equal(decision.get(key), expected, f"freeze/decision/{key}")
    scope = freeze.get("scope")
    if not isinstance(scope, dict):
        raise fail("freeze/scope missing")
    assert_equal(scope.get("routes_for_structural_gate"), list(ROUTES), "freeze/scope/routes")
    assert_equal(scope.get("gnss_first_solver_unchanged"), True, "freeze/scope/gnss-first")
    assert_equal(scope.get("legacy_default_unchanged"), True, "freeze/scope/legacy")
    candidate = freeze.get("candidate")
    if not isinstance(candidate, dict):
        raise fail("freeze/candidate missing")
    for key, expected in {
        "id": CANDIDATE_ID,
        "implemented": False,
        "opt_in": True,
        "default_off": True,
    }.items():
        assert_equal(candidate.get(key), expected, f"freeze/candidate/{key}")
    solver = candidate.get("solver")
    if not isinstance(solver, dict):
        raise fail("freeze/candidate/solver missing")
    for key, expected in {
        "candidate_solver_type": "MULTIFRONTAL_QR",
        "candidate_elimination_function": "EliminateQR",
        "candidate_branch": "multifrontal",
        "candidate_ordering_type": "COLAMD",
        "explicit_ordering_present": False,
        "ordering_policy_changed": False,
        "sequential_elimination": False,
    }.items():
        assert_equal(solver.get(key), expected, f"freeze/solver/{key}")
    unchanged = candidate.get("unchanged_contract")
    if not isinstance(unchanged, dict):
        raise fail("freeze/candidate/unchanged_contract missing")
    for key in (
        "graph_and_values", "factor_families", "raw_retained_d_initialization",
        "exact_retained_key_alignment", "full_finite_optimized_c_and_d",
        "position_velocity_handoff", "filter_and_observation_retention",
        "initialization", "priors", "lm_iteration_and_acceptance_policy",
        "fail_closed", "no_cholesky_fallback",
    ):
        assert_equal(unchanged.get(key), True, f"freeze/unchanged/{key}")
    assert_equal(unchanged.get("ccdd_sigma"), "0.1 m", "freeze/unchanged/ccdd_sigma")
    execution = freeze.get("execution_policy")
    if not isinstance(execution, dict):
        raise fail("freeze/execution_policy missing")
    assert_equal(execution.get("this_freeze_authorizes_raw_execution"), False, "freeze/raw execution")
    assert_equal(execution.get("this_freeze_authorizes_solver_rerun"), False, "freeze/solver rerun")
    return freeze


def verify_implementation() -> None:
    for path, expected, label in (
        (APP, APP_SHA, "native app"),
        (BACKEND, BACKEND_SHA, "GTSAM backend"),
        (CONFIG, CONFIG_SHA, "FGO config"),
        (BINARY, BINARY_SHA, "native binary"),
        (PHASE95_WRAPPER, PHASE95_WRAPPER_SHA, "Phase95 path wrapper"),
        (PHASE91_MANIFEST, PHASE91_MANIFEST_SHA, "Phase91 manifest"),
    ):
        assert_equal(sha256_file(path, label), expected, f"implementation/{label}")
    source = APP.read_text(encoding="utf-8")
    backend = BACKEND.read_text(encoding="utf-8")
    config = CONFIG.read_text(encoding="utf-8")
    for marker in (
        SELECTOR,
        "use_native_source_clock_c0d_phase99_main_multifrontal_qr_solver",
        "selectPhase99MainSolver",
        "MULTIFRONTAL_QR",
        "EliminateQR",
        "selected_linear_solver_type",
    ):
        if marker not in source and marker not in backend and marker not in config:
            raise fail(f"Phase99 implementation marker missing: {marker}")
    if "use_native_source_clock_c0d_phase99_main_multifrontal_qr_solver =\n            false" not in config:
        raise fail("Phase99 default is not explicitly disabled")
    if "phase99_main_meter_state_graph" not in backend or "!use_imu" not in backend:
        raise fail("Phase99 main-only scope marker missing")


def _output_path(route: str, name: str) -> str:
    return f"{OUTPUT_ROOT}{route.replace('/', '__')}/{name}"


def _validate_command(route: str, command: Any, record: dict[str, Any], paths: dict[str, str]) -> None:
    if not isinstance(command, list) or not command or not all(isinstance(token, str) for token in command):
        raise fail(f"malformed command: {route}")
    assert_equal(command[0], "build/apps/gnss_fgo_imu_no_base", f"command/{route}/binary")
    for token in command:
        if any(term in token.lower() for term in FORBIDDEN_PATH_TERMS):
            raise fail(f"forbidden command path term: {route}/{token}")
        if token in FORBIDDEN_FLAGS:
            raise fail(f"forbidden command flag: {route}/{token}")
    for flag in REQUIRED_FLAGS:
        assert_equal(command.count(flag), 1, f"command/{route}/{flag}")
    assert_equal(command.count("--dataset-id"), 1, f"command/{route}/dataset-id-count")
    assert_equal(command[command.index("--dataset-id") + 1], route, f"command/{route}/dataset-id")
    raw = record.get("raw_inputs")
    if not isinstance(raw, dict) or set(raw) != set(RAW_NAMES):
        raise fail(f"manifest raw-input set changed: {route}")
    for flag, name, placeholder in RAW_FLAGS:
        assert_equal(command.count(flag), 1, f"command/{route}/{flag}")
        assert_equal(command[command.index(flag) + 1], placeholder, f"command/{route}/{name}/placeholder")
        pin = raw.get(name)
        if not isinstance(pin, dict):
            raise fail(f"manifest raw pin malformed: {route}/{name}")
        assert_equal(pin.get("path"), paths[name], f"manifest/raw/{route}/{name}/path")
        assert_equal(pin.get("path_source"), "Phase95 sealed result raw_inputs.path", f"manifest/raw/{route}/{name}/source")
        assert_equal(pin.get("sha256"), None, f"manifest/raw/{route}/{name}/sha256")
        assert_equal(pin.get("read_at_manifest_creation"), False, f"manifest/raw/{route}/{name}/read")
    assert_equal(command.count("--out"), 1, f"command/{route}/out-count")
    assert_equal(command[command.index("--out") + 1], _output_path(route, "withheld_solution_output.csv"), f"command/{route}/out")
    assert_equal(command.count("--summary-json"), 1, f"command/{route}/summary-count")
    assert_equal(command[command.index("--summary-json") + 1], _output_path(route, "summary.json"), f"command/{route}/summary")
    planned = record.get("planned_output")
    if not isinstance(planned, dict):
        raise fail(f"planned output missing: {route}")
    assert_equal(planned.get("summary"), _output_path(route, "summary.json"), f"manifest/output/{route}/summary")
    assert_equal(planned.get("withheld_solution_output"), _output_path(route, "withheld_solution_output.csv"), f"manifest/output/{route}/withheld")


def verify_manifest() -> dict[str, Any]:
    verify_freeze()
    verify_implementation()
    paths = phase95_paths()
    manifest = read_json(MANIFEST, "Phase99 execution manifest")
    for key, expected in {
        "schema_version": SCHEMA,
        "phase": 99,
        "execution_label": "Luna Max",
        "status": "sealed-before-phase99-main-multifrontal-qr-raw-execution",
    }.items():
        assert_equal(manifest.get(key), expected, f"manifest/{key}")
    freeze_ref = manifest.get("freeze")
    if not isinstance(freeze_ref, dict):
        raise fail("manifest/freeze missing")
    for key, expected in {
        "path": relative(FREEZE),
        "sha256": FREEZE_SHA,
        "commit": FREEZE_COMMIT,
        "execution_authorized_before_this_manifest": False,
    }.items():
        assert_equal(freeze_ref.get(key), expected, f"manifest/freeze/{key}")
    implementation = manifest.get("implementation")
    if not isinstance(implementation, dict):
        raise fail("manifest/implementation missing")
    for key, expected in {
        "commit": IMPLEMENTATION_COMMIT,
        "native_algorithm_changed": True,
        "solver_filter_lm_changed": False,
        "equation_units_sigma_changed": False,
        "phase99_change_scope": "opt-in main meter-state multifrontal QR solver branch only",
    }.items():
        assert_equal(implementation.get(key), expected, f"manifest/implementation/{key}")
    assert_equal(implementation.get("source", {}).get("sha256"), APP_SHA, "manifest/implementation/source")
    assert_equal(implementation.get("backend", {}).get("sha256"), BACKEND_SHA, "manifest/implementation/backend")
    assert_equal(implementation.get("config", {}).get("sha256"), CONFIG_SHA, "manifest/implementation/config")
    assert_equal(implementation.get("binary", {}).get("sha256"), BINARY_SHA, "manifest/implementation/binary")
    for key, path in (
        ("pre_raw_evaluator", EVALUATOR),
        ("execution_wrapper", WRAPPER),
        ("focused_tests", FOCUSED_TESTS),
    ):
        pin = manifest.get(key)
        if not isinstance(pin, dict):
            raise fail(f"manifest/{key} missing")
        assert_equal(pin.get("path"), relative(path), f"manifest/{key}/path")
        assert_equal(pin.get("sha256"), sha256_file(path, f"Phase99 {key}"), f"manifest/{key}/sha256")
    inherited = manifest.get("phase95_path_source")
    if not isinstance(inherited, dict):
        raise fail("manifest/phase95_path_source missing")
    for key, expected in {
        "result_path": relative(PHASE95_RESULT),
        "result_sha256": PHASE95_RESULT_SHA,
        "wrapper_path": relative(PHASE95_WRAPPER),
        "wrapper_sha256": PHASE95_WRAPPER_SHA,
        "phase91_manifest_path": relative(PHASE91_MANIFEST),
        "phase91_manifest_sha256": PHASE91_MANIFEST_SHA,
        "materialization": "Phase95 load_inherited_raw_inputs metadata-only stat followed by exact GNSS/IMU/nav path substitution",
        "raw_byte_reads_by_pre_raw_verifier": 0,
        "raw_content_copy_or_transform": False,
    }.items():
        assert_equal(inherited.get(key), expected, f"manifest/phase95_path_source/{key}")
    candidate = manifest.get("candidate")
    if not isinstance(candidate, dict):
        raise fail("manifest/candidate missing")
    for key, expected in {
        "id": CANDIDATE_ID,
        "selector": SELECTOR,
        "phase93_selector": PHASE93_SELECTOR,
        "phase96_selector": PHASE96_SELECTOR,
        "phase98_selector": PHASE98_SELECTOR,
        "candidate_count": 1,
        "opt_in": True,
        "default_off_outside_this_command": True,
        "diagnostic_only": True,
        "raw_only": True,
        "runs_per_route": 1,
        "controls": 0,
        "accuracy_scoring": False,
        "truth_evaluation": False,
        "submission_release": False,
        "no_solver_filter_or_lm_change": True,
        "no_equation_unit_sigma_change": True,
        "no_fallback_or_guard_bypass": True,
        "no_solution_output_publication": True,
        "raw_content_copy_or_transform": False,
        "cholesky_comparison_rerun": False,
    }.items():
        assert_equal(candidate.get(key), expected, f"manifest/candidate/{key}")
    solver = candidate.get("solver_branch")
    if not isinstance(solver, dict):
        raise fail("manifest/candidate/solver_branch missing")
    for key, expected in {
        "selected_main_solver_type": "MULTIFRONTAL_QR",
        "selected_main_elimination_function": "EliminateQR",
        "selected_main_branch": "multifrontal",
        "ordering_type": "COLAMD",
        "explicit_ordering_present": False,
        "gnss_first_solver_type": "MULTIFRONTAL_CHOLESKY",
        "legacy_solver_type": "MULTIFRONTAL_CHOLESKY",
    }.items():
        assert_equal(solver.get(key), expected, f"manifest/solver_branch/{key}")
    raw_contract = candidate.get("raw_input_contract")
    if not isinstance(raw_contract, dict):
        raise fail("manifest/raw_input_contract missing")
    for key, expected in {
        "input_names_exact": list(RAW_NAMES),
        "device_gnss_only": True,
        "device_imu_only": True,
        "broadcast_navigation_only": True,
        "truth_mat_base_precomputed_coordinate_kaggle_accuracy": False,
        "copy_or_transform": False,
    }.items():
        assert_equal(raw_contract.get(key), expected, f"manifest/raw_input_contract/{key}")
    routes = manifest.get("routes")
    if not isinstance(routes, list) or [item.get("dataset_id") for item in routes] != list(ROUTES):
        raise fail("manifest route order/count changed")
    for record in routes:
        route = record.get("dataset_id")
        if route not in DOMAIN_ROWS:
            raise fail(f"unknown route: {route}")
        assert_equal(record.get("runs"), 1, f"manifest/route/{route}/runs")
        assert_equal(record.get("domain_rows"), DOMAIN_ROWS[route], f"manifest/route/{route}/domain_rows")
        assert_equal(record.get("diagnostic_only"), True, f"manifest/route/{route}/diagnostic_only")
        _validate_command(route, record.get("command"), record, paths[route])
    matrix = manifest.get("matrix")
    if not isinstance(matrix, dict):
        raise fail("manifest/matrix missing")
    for key, expected in {
        "candidate_count": 1,
        "route_count": 2,
        "runs_per_route": 1,
        "native_invocations_planned": 2,
        "controls": 0,
        "reruns": 0,
        "fallbacks": 0,
        "raw_device_gnss_route_arguments": 2,
        "raw_device_imu_route_arguments": 2,
        "broadcast_navigation_route_arguments": 2,
        "truth_reads_planned": 0,
        "mat_reads_or_generated_planned": 0,
        "precomputed_coordinate_reads_planned": 0,
        "base_rinex_reads_planned": 0,
        "accuracy_calculations_planned": 0,
        "kaggle_or_token_access_planned": 0,
        "route_score_selection": False,
        "solution_rows_authorized": False,
        "cholesky_comparison_rerun_planned": False,
    }.items():
        assert_equal(matrix.get(key), expected, f"manifest/matrix/{key}")
    gates = manifest.get("structural_gates")
    if not isinstance(gates, dict):
        raise fail("manifest/structural_gates missing")
    for key in (
        "gnss_first_strict_progress",
        "gnss_first_full_finite_d_exact_handoff",
        "main_selected_multifrontal_qr",
        "main_accepted_outer_iterations",
        "main_strict_cost_decrease",
        "main_finite_expected_coverage",
        "no_fallback_or_solution_publication",
    ):
        assert_equal(gates.get(key), True, f"manifest/structural_gates/{key}")
    qualification = manifest.get("qualification_evidence")
    if not isinstance(qualification, dict):
        raise fail("manifest/qualification_evidence missing")
    for key, expected in {
        "tests_run": 1112,
        "passed": 1054,
        "skipped": 58,
        "failed": 0,
        "exit_code": 0,
        "raw_route_execution": False,
    }.items():
        assert_equal(qualification.get("full_cpp_suite", {}).get(key), expected, f"qualification/full_cpp/{key}")
    for key, expected in {"tests_run": 7, "passed": 7, "failed": 0, "exit_code": 0, "raw_route_execution": False}.items():
        assert_equal(qualification.get("focused_python", {}).get(key), expected, f"qualification/focused_python/{key}")
    assert_equal(qualification.get("target_build", {}).get("exit_code"), 0, "qualification/target_build/exit_code")
    assert_equal(qualification.get("diff_check_clean"), True, "qualification/diff_check_clean")
    execution = manifest.get("execution_authorization")
    if not isinstance(execution, dict):
        raise fail("manifest/execution_authorization missing")
    for key, expected in {
        "before_raw_execution": True,
        "raw_execution_authorized": False,
        "native_route_rerun_performed": False,
        "accuracy_or_submission_release": False,
        "stop_after_two_routes": True,
    }.items():
        assert_equal(execution.get(key), expected, f"manifest/execution_authorization/{key}")
    accounting = manifest.get("read_accounting_before_execution")
    if not isinstance(accounting, dict):
        raise fail("manifest/read_accounting_before_execution missing")
    for key in (
        "native_solver_invocations", "raw_device_gnss_reads", "raw_device_imu_reads",
        "broadcast_navigation_reads", "truth_reads", "mat_reads_or_generated",
        "precomputed_coordinate_reads", "base_rinex_reads", "accuracy_calculations",
        "kaggle_or_token_access", "raw_input_hash_reads", "route_reruns", "fallbacks",
    ):
        assert_equal(accounting.get(key), 0, f"manifest/read_accounting/{key}")
    assert_equal(accounting.get("raw_content_copied_or_transformed"), False, "manifest/read_accounting/copy")
    return manifest


def verify_authorization(manifest: dict[str, Any]) -> dict[str, Any]:
    authorization = read_json(AUTHORIZATION, "Phase99 raw execution authorization")
    for key, expected in {
        "schema_version": AUTH_SCHEMA,
        "phase": 99,
        "execution_label": "Luna Max",
        "status": "authorized-for-exact-two-route-phase99-main-multifrontal-qr-structural-execution",
    }.items():
        assert_equal(authorization.get(key), expected, f"authorization/{key}")
    authority = authorization.get("authority")
    if not isinstance(authority, dict):
        raise fail("authorization/authority missing")
    pins = {
        "phase99_freeze": (FREEZE, FREEZE_SHA),
        "phase99_manifest": (MANIFEST, sha256_file(MANIFEST, "Phase99 manifest")),
        "phase99_evaluator": (EVALUATOR, sha256_file(EVALUATOR, "Phase99 evaluator")),
        "phase99_wrapper": (WRAPPER, sha256_file(WRAPPER, "Phase99 wrapper")),
        "phase99_focused_tests": (FOCUSED_TESTS, sha256_file(FOCUSED_TESTS, "Phase99 focused tests")),
        "phase95_result": (PHASE95_RESULT, PHASE95_RESULT_SHA),
        "phase95_path_wrapper": (PHASE95_WRAPPER, PHASE95_WRAPPER_SHA),
        "phase91_manifest": (PHASE91_MANIFEST, PHASE91_MANIFEST_SHA),
    }
    for key, (path, digest) in pins.items():
        pin = authority.get(key)
        if not isinstance(pin, dict):
            raise fail(f"authorization/{key} missing")
        assert_equal(pin.get("path"), relative(path), f"authorization/{key}/path")
        assert_equal(pin.get("sha256"), digest, f"authorization/{key}/sha256")
    for key, expected in {
        "freeze_commit": FREEZE_COMMIT,
        "implementation_commit": IMPLEMENTATION_COMMIT,
        "implementation_source_sha256": APP_SHA,
        "implementation_backend_sha256": BACKEND_SHA,
        "binary_sha256": BINARY_SHA,
        "manifest_sha256": sha256_file(MANIFEST, "Phase99 final manifest"),
    }.items():
        assert_equal(authority.get(key), expected, f"authorization/authority/{key}")
    candidate = authorization.get("candidate")
    if not isinstance(candidate, dict):
        raise fail("authorization/candidate missing")
    for key, expected in {
        "candidate_count": 1,
        "id": CANDIDATE_ID,
        "selector": SELECTOR,
        "phase93_selector": PHASE93_SELECTOR,
        "phase96_selector": PHASE96_SELECTOR,
        "phase98_selector": PHASE98_SELECTOR,
        "diagnostic_only": True,
        "raw_only": True,
        "default_off_outside_this_explicit_command": True,
        "runs_per_route": 1,
        "controls": 0,
        "raw_input_names_exact": list(RAW_NAMES),
        "truth_mat_coordinate_base_kaggle_accuracy": False,
        "raw_content_copy_or_transform": False,
        "solution_output_publication": False,
        "cholesky_comparison_rerun": False,
    }.items():
        assert_equal(candidate.get(key), expected, f"authorization/candidate/{key}")
    assert_equal(authorization.get("routes"), list(ROUTES), "authorization/routes")
    matrix = authorization.get("matrix")
    if not isinstance(matrix, dict):
        raise fail("authorization/matrix missing")
    for key, expected in {
        "candidate_count": 1,
        "route_count": 2,
        "runs_per_route": 1,
        "native_invocations": 2,
        "controls": 0,
        "reruns": 0,
        "fallbacks": 0,
        "raw_device_gnss_reads_max": 2,
        "raw_device_imu_reads_max": 2,
        "broadcast_navigation_reads_max": 2,
        "truth_reads": 0,
        "mat_reads_or_generated": 0,
        "precomputed_coordinate_reads": 0,
        "base_rinex_reads": 0,
        "accuracy_calculations": 0,
        "kaggle_or_token_access": 0,
        "solution_rows_authorized": False,
        "cholesky_comparison_rerun": False,
    }.items():
        assert_equal(matrix.get(key), expected, f"authorization/matrix/{key}")
    policy = authorization.get("execution_policy")
    if not isinstance(policy, dict):
        raise fail("authorization/execution_policy missing")
    for key, expected in {
        "authorized": True,
        "order": "MTV-A then LAX-T, sequentially",
        "one_invocation_per_route": True,
        "no_rerun": True,
        "stop_after_two_routes": True,
        "solver_filter_lm_changes": False,
        "fallback_or_guard_bypass": False,
        "cholesky_comparison_rerun": False,
        "solution_output": "required CLI path is withheld and must not be published or committed",
    }.items():
        assert_equal(policy.get(key), expected, f"authorization/policy/{key}")
    boundary = authorization.get("release_boundary")
    if not isinstance(boundary, dict):
        raise fail("authorization/release_boundary missing")
    for key, expected in {
        "raw_execution_authorized": True,
        "diagnostic_structural_result_authorized": True,
        "truth_or_accuracy_evaluation_authorized": False,
        "submission_release_authorized": False,
        "promotion_authorized": False,
        "solution_rows_authorized": False,
    }.items():
        assert_equal(boundary.get(key), expected, f"authorization/release/{key}")
    assert_equal(manifest.get("execution_authorization", {}).get("raw_execution_authorized"), False, "manifest remains pre-auth")
    return authorization


def verify_pre_raw() -> dict[str, Any]:
    freeze = verify_freeze()
    manifest = verify_manifest()
    source = EVALUATOR.read_text(encoding="utf-8")
    for forbidden in ("sub" + "process", "P" + "open", "os." + "system", "os." + "popen", "exec" + "ve"):
        if forbidden in source:
            raise fail(f"pre-raw evaluator contains launch token: {forbidden}")
    authorized = False
    auth_status = "not-issued-before-separate-raw-authorization"
    if AUTHORIZATION.is_file():
        authorization = verify_authorization(manifest)
        authorized = authorization.get("status") == "authorized-for-exact-two-route-phase99-main-multifrontal-qr-structural-execution"
        auth_status = "authorized" if authorized else "invalid"
    return {
        "status": "pre-raw-verified",
        "phase": 99,
        "execution_label": "Luna Max",
        "candidate_count": 1,
        "routes": len(ROUTES),
        "route_ids": list(ROUTES),
        "runs_per_route": 1,
        "raw_execution_authorized": authorized,
        "raw_reads": 0,
        "native_solver_invocations": 0,
        "truth_reads": 0,
        "mat_reads_or_generated": 0,
        "precomputed_coordinate_reads": 0,
        "accuracy_scored": False,
        "route_score_selection": False,
        "cholesky_comparison_rerun": False,
        "freeze_sha256": FREEZE_SHA,
        "manifest_sha256": sha256_file(MANIFEST, "Phase99 manifest"),
        "phase95_result_sha256": PHASE95_RESULT_SHA,
        "phase95_wrapper_sha256": PHASE95_WRAPPER_SHA,
        "freeze_status": freeze["status"],
        "manifest_status": manifest["status"],
        "authorization_status": auth_status,
    }


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--verify-freeze", action="store_true")
    parser.add_argument("--verify-manifest", action="store_true")
    parser.add_argument("--verify-pre-raw", action="store_true")
    args = parser.parse_args()
    if not any((args.verify_freeze, args.verify_manifest, args.verify_pre_raw)):
        parser.error("one verification mode is required")
    try:
        if args.verify_freeze:
            verify_freeze()
        if args.verify_manifest:
            verify_manifest()
        if args.verify_pre_raw:
            print(json.dumps(verify_pre_raw(), indent=2, sort_keys=True))
    except (Phase99PreRawError, OSError) as exc:
        print(f"phase99 pre-raw verifier: fail-closed: {exc}", file=sys.stderr)
        raise SystemExit(2)
