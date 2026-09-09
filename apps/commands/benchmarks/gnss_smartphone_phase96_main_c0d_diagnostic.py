#!/usr/bin/env python3
"""Launch-free verifier for the Phase96 main C0/D diagnostic lane.

This module verifies only the sealed source, implementation, manifest, and
authorization records.  It never launches a process and never opens or hashes
raw GNSS, IMU, navigation, truth, MAT, coordinate, base, or Kaggle files.
The execution wrapper imports this module only after the separate one-shot
authorization is present.
"""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import sys
from typing import Any


ROOT = Path(__file__).resolve().parents[3]
FREEZE = ROOT / (
    "docs/use_cases/records/"
    "smartphone_r5_phase96_main_c0d_diagnostic_telemetry_freeze_v1.json"
)
MANIFEST = ROOT / (
    "docs/use_cases/records/"
    "smartphone_r5_phase96_main_c0d_diagnostic_execution_manifest_v1.json"
)
AUTHORIZATION = ROOT / (
    "docs/use_cases/records/"
    "smartphone_r5_phase96_main_c0d_diagnostic_raw_execution_authorization_v1.json"
)
EVALUATOR = Path(__file__).resolve()
WRAPPER = ROOT / (
    "apps/commands/benchmarks/"
    "gnss_smartphone_phase96_main_c0d_diagnostic_execute.py"
)
FOCUSED_TESTS = ROOT / "tests/test_smartphone_phase96_main_c0d_diagnostic.py"
PHASE91_MANIFEST = ROOT / (
    "docs/use_cases/records/"
    "smartphone_r5_phase91_source_clock_c0d_gnss_first_raw_drift_d_initializer_execution_manifest_v1.json"
)
APP = ROOT / "apps/native/gnss_fgo_imu_no_base.cpp"
BINARY = ROOT / "build/apps/gnss_fgo_imu_no_base"

FREEZE_SHA = "9605f3ce32ad08ae3f17254834df50a65cd93918cee593b487a02ad7b97ebc65"
FREEZE_COMMIT = "d4c640312568bffc01f21282767d33756a1ef1a9"
IMPLEMENTATION_COMMIT = "bcdcd952943623650cdf77410c17e9ec87f45d7f"
IMPLEMENTATION_SOURCE_SHA = "8e71268904fde04db14db8d9357aea97a9b6eed62b67ffd15fc2acbd7865799f"
BINARY_SHA = "60e244d27fa48bdc962dfeb63047314f16cbed2ce09fc93040bced0bcd790009"
PHASE91_MANIFEST_SHA = "2078a4e2c3b963f07744c435ec303ea848bc372c8efd5003f6cb7b3d4b4ce988"

IMPLEMENTATION_FILES = {
    "apps/native/gnss_fgo_imu_no_base.cpp": "8e71268904fde04db14db8d9357aea97a9b6eed62b67ffd15fc2acbd7865799f",
    "include/libgnss++/algorithms/fgo.hpp": "623dc927266bc82e944b2e9965f3dfa35a2b749c892beeae9ae39fd71d83feef",
    "include/libgnss++/algorithms/fgo_config.hpp": "b4dd5cbae90a3ac8a357dc15fc011b44d923943a7384335688b116731e660ecf",
    "src/algorithms/fgo.cpp": "b297eccfb866d08346cdebbf2e0dff3ac7db001c0a6a539056c4647714d6f746",
    "src/algorithms/fgo_gtsam_backend.cpp": "077afd8df8745fe0c01fc52d35c33ac5908b44185e27d55cdb8c19eac7f6cdc7",
    "src/algorithms/fgo_gtsam_internal.hpp": "7691fa988b6236c14152cd7d9d66aba66fc2879df2861174a5cbe87b3bbef1cb",
    "tests/test_fgo_gtsam_backend.cpp": "8ef84fb719bf77fc2f72a98fb3a9cc1e8481ddc607a5e529b0589d3f911767e2",
}

ROUTES = (
    "2021-03-16-18-59-us-ca-mtv-a/pixel5",
    "2022-04-01-18-22-us-ca-lax-t/pixel5",
)
DOMAIN_ROWS = {
    ROUTES[0]: 2158,
    ROUTES[1]: 1465,
}
RAW_NAMES = ("device_gnss.csv", "device_imu.csv", "brdc.nav")
RAW_FLAGS = (
    ("--android-gnss", "device_gnss.csv"),
    ("--android-imu", "device_imu.csv"),
    ("--nav", "brdc.nav"),
)
BINARY_RELATIVE = "build/apps/gnss_fgo_imu_no_base"
RAW_ROOT = "raw/phase93/"
OUTPUT_ROOT = "output/smartphone-r5/phase96-main-c0d-diagnostic-v1/"
SELECTOR = "--native-source-clock-c0d-phase96-main-diagnostics"
PHASE93_SELECTOR = "--native-source-clock-c0d-gnss-first-meter-state-handoff"
SCHEMA = "smartphone-r5-phase96-main-c0d-diagnostic-execution-manifest.v1"
AUTH_SCHEMA = "smartphone-r5-phase96-main-c0d-diagnostic-raw-execution-authorization.v1"

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
)
FORBIDDEN_PATH_TERMS = (
    ".mat",
    "ground_truth",
    "validation",
    "holdout",
    "kaggle",
    "token",
    "base.rinex",
    "coordinate",
    "truth",
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
    PHASE93_SELECTOR,
    SELECTOR,
)


class Phase96PreRawError(ValueError):
    """Raised when a sealed Phase96 pre-raw contract fails closed."""


def fail(message: str) -> Phase96PreRawError:
    return Phase96PreRawError(message)


def relative(path: Path) -> str:
    try:
        return str(path.relative_to(ROOT))
    except ValueError:
        return str(path)


def sha256_file(path: Path, label: str) -> str:
    """Hash a pinned source/artifact file, never a raw input path."""

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


def _raw_path(route: str, name: str) -> str:
    return f"{RAW_ROOT}{route}/{name}"


def _output_path(route: str, name: str) -> str:
    return f"{OUTPUT_ROOT}{route.replace('/', '__')}/{name}"


def _validate_safe_token(token: str, label: str) -> None:
    if any(term in token.lower() for term in FORBIDDEN_PATH_TERMS):
        raise fail(f"forbidden path term: {label}/{token}")


def verify_freeze() -> dict[str, Any]:
    assert_equal(sha256_file(FREEZE, "Phase96 freeze"), FREEZE_SHA, "freeze/sha256")
    freeze = read_json(FREEZE, "Phase96 freeze")
    for key, expected in {
        "schema_version": "smartphone-r5-phase96-main-c0d-diagnostic-telemetry-freeze.v1",
        "phase": 96,
        "status": "frozen-diagnostic-only-not-implemented",
        "execution_label": "Luna Max",
        "exactly_one_candidate_frozen": True,
    }.items():
        assert_equal(freeze.get(key), expected, f"freeze/{key}")
    scope = freeze.get("scope")
    if not isinstance(scope, dict):
        raise fail("freeze/scope missing")
    assert_equal(scope.get("routes"), list(ROUTES), "freeze/scope/routes")
    assert_equal(scope.get("route_selection_by_score_or_truth"), False, "freeze/scope/route_selection")
    candidate = freeze.get("candidate")
    if not isinstance(candidate, dict):
        raise fail("freeze/candidate missing")
    for key, expected in {
        "id": "phase96_main_c0d_factor_family_lm_trial_diagnostic_telemetry",
        "kind": "source-aligned-observability-only",
        "status": "frozen-not-implemented",
        "implemented": False,
        "default_enabled": False,
        "raw_execution_authorized": False,
        "algorithmic_effect": "None. The existing graph, values, factor equations, optimizer calls, acceptance decisions, and fail-closed gates remain authoritative.",
    }.items():
        assert_equal(candidate.get(key), expected, f"freeze/candidate/{key}")
    boundary = freeze.get("publication_and_execution_policy")
    if not isinstance(boundary, dict):
        raise fail("freeze/publication_and_execution_policy missing")
    for key, expected in {
        "diagnostic_payload_only": True,
        "raw_observations_excluded": True,
        "coordinates_excluded": True,
        "solution_rows_excluded": True,
        "truth_excluded": True,
        "mat_excluded": True,
        "kaggle_or_token_excluded": True,
        "accuracy_excluded": True,
        "native_solver_invocations_at_freeze": 0,
        "raw_execution_authorized": False,
        "separate_future_raw_authorization_required": True,
        "fail_closed_preserved": True,
    }.items():
        assert_equal(boundary.get(key), expected, f"freeze/publication_policy/{key}")
    invariants = freeze.get("source_and_unit_invariants")
    if not isinstance(invariants, dict):
        raise fail("freeze/source_and_unit_invariants missing")
    for key, expected in {
        "official_ccdd_equation": "(C2-C1)-((D1+D2)*dt/2)",
        "official_ccdd_jacobian_order": "[-1,+1,-dt/2,-dt/2] for [C1,C2,D1,D2]",
        "ordinary_ccdd_sigma_m": 0.1,
        "equation_unchanged": True,
        "jacobian_order_unchanged": True,
        "units_unchanged": True,
        "sigma_unchanged": True,
        "filters_unchanged": True,
        "lm_parameters_unchanged": True,
        "legacy_default_unchanged": True,
        "no_pdc_or_external_coordinate_input": True,
        "no_fallback_or_bridge": True,
        "no_mtvu_or_mtuh_fix_included": True,
    }.items():
        assert_equal(invariants.get(key), expected, f"freeze/invariants/{key}")
    accounting = freeze.get("read_accounting_at_freeze")
    if not isinstance(accounting, dict):
        raise fail("freeze/read_accounting_at_freeze missing")
    for key in (
        "native_solver_invocations",
        "raw_gnss_reads",
        "raw_imu_reads",
        "broadcast_navigation_reads",
        "truth_reads",
        "mat_reads_or_generated",
        "kaggle_or_token_access",
        "base_rinex_reads",
        "precomputed_coordinate_reads",
        "accuracy_calculations",
        "route_reruns",
        "fallbacks",
    ):
        assert_equal(accounting.get(key), 0, f"freeze/read_accounting/{key}")
    return freeze


def verify_implementation() -> None:
    assert_equal(sha256_file(APP, "native implementation"), IMPLEMENTATION_SOURCE_SHA, "implementation/source_sha256")
    assert_equal(sha256_file(BINARY, "native binary"), BINARY_SHA, "implementation/binary_sha256")
    for relative_path, expected in IMPLEMENTATION_FILES.items():
        path = ROOT / relative_path
        assert_equal(sha256_file(path, f"implementation file {relative_path}"), expected, f"implementation/{relative_path}")
    try:
        app_text = APP.read_text(encoding="utf-8")
        config_text = (ROOT / "include/libgnss++/algorithms/fgo_config.hpp").read_text(encoding="utf-8")
    except (OSError, UnicodeDecodeError) as exc:
        raise fail(f"failed to read pinned implementation source: {exc}") from exc
    for token in (
        SELECTOR,
        "phase96_main",
        "writePhase96MainDiagnostics",
        "factor_family_cost_sum",
    ):
        if token not in app_text and token not in config_text:
            raise fail(f"Phase96 implementation marker missing: {token}")
    if "use_native_source_clock_c0d_phase96_main_diagnostics = false" not in config_text:
        raise fail("Phase96 default is not explicitly disabled")


def _validate_command(route: str, command: Any, record: dict[str, Any]) -> None:
    if not isinstance(command, list) or not command or not all(isinstance(token, str) for token in command):
        raise fail(f"malformed command: {route}")
    if command[0] != BINARY_RELATIVE:
        raise fail(f"command binary changed: {route}")
    for token in command:
        _validate_safe_token(token, route)
        if token in FORBIDDEN_FLAGS:
            raise fail(f"forbidden command flag: {route}/{token}")
    for flag in REQUIRED_FLAGS:
        assert_equal(command.count(flag), 1, f"command/{route}/{flag}")
    assert_equal(command.count("--dataset-id"), 1, f"command/{route}/dataset-id-count")
    assert_equal(command[command.index("--dataset-id") + 1], route, f"command/{route}/dataset-id")
    raw = record.get("raw_inputs")
    if not isinstance(raw, dict) or tuple(raw) != RAW_NAMES:
        raise fail(f"raw input set changed: {route}")
    for flag, name in RAW_FLAGS:
        assert_equal(command.count(flag), 1, f"command/{route}/{flag}-count")
        assert_equal(command[command.index(flag) + 1], _raw_path(route, name), f"command/{route}/{name}")
        pin = raw[name]
        if not isinstance(pin, dict):
            raise fail(f"raw input pin malformed: {route}/{name}")
        assert_equal(pin.get("path"), _raw_path(route, name), f"manifest/raw/{route}/{name}/path")
        assert_equal(pin.get("sha256"), None, f"manifest/raw/{route}/{name}/sha256")
        assert_equal(pin.get("sha256_available"), False, f"manifest/raw/{route}/{name}/sha256_available")
        assert_equal(pin.get("read_at_manifest_creation"), False, f"manifest/raw/{route}/{name}/read")
    planned = record.get("planned_output")
    if not isinstance(planned, dict):
        raise fail(f"planned output missing: {route}")
    assert_equal(command.count("--out"), 1, f"command/{route}/--out-count")
    assert_equal(command[command.index("--out") + 1], _output_path(route, "withheld_solution_output.csv"), f"command/{route}/withheld-output")
    assert_equal(command.count("--summary-json"), 1, f"command/{route}/--summary-json-count")
    assert_equal(command[command.index("--summary-json") + 1], _output_path(route, "summary.json"), f"command/{route}/summary")
    assert_equal(planned.get("summary"), _output_path(route, "summary.json"), f"manifest/output/{route}/summary")
    assert_equal(planned.get("withheld_solution_output"), _output_path(route, "withheld_solution_output.csv"), f"manifest/output/{route}/withheld")


def verify_manifest() -> dict[str, Any]:
    verify_freeze()
    verify_implementation()
    manifest = read_json(MANIFEST, "Phase96 execution manifest")
    for key, expected in {
        "schema_version": SCHEMA,
        "phase": 96,
        "status": "sealed-before-phase96-main-c0d-diagnostic-raw-execution",
        "execution_label": "Luna Max",
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
        "source_sha256": IMPLEMENTATION_SOURCE_SHA,
        "native_algorithm_changed": False,
        "solver_filter_lm_changed": False,
        "equation_units_sigma_changed": False,
    }.items():
        assert_equal(implementation.get(key), expected, f"manifest/implementation/{key}")
    source = implementation.get("source")
    binary = implementation.get("binary")
    if not isinstance(source, dict) or not isinstance(binary, dict):
        raise fail("manifest/implementation pins malformed")
    assert_equal(source.get("path"), relative(APP), "manifest/source/path")
    assert_equal(source.get("sha256"), IMPLEMENTATION_SOURCE_SHA, "manifest/source/sha256")
    assert_equal(binary.get("path"), BINARY_RELATIVE, "manifest/binary/path")
    assert_equal(binary.get("sha256"), BINARY_SHA, "manifest/binary/sha256")
    changed_files = implementation.get("changed_files")
    if not isinstance(changed_files, list) or len(changed_files) != len(IMPLEMENTATION_FILES):
        raise fail("manifest/implementation/changed_files malformed")
    by_path = {item.get("path"): item for item in changed_files if isinstance(item, dict)}
    if set(by_path) != set(IMPLEMENTATION_FILES):
        raise fail("manifest/implementation/changed_files set changed")
    for path_text, expected in IMPLEMENTATION_FILES.items():
        assert_equal(by_path[path_text].get("sha256"), expected, f"manifest/implementation/changed_files/{path_text}")
        assert_equal(by_path[path_text].get("diagnostic_or_test_only"), True, f"manifest/implementation/changed_files/{path_text}/scope")
    for key, path in (("evaluator", EVALUATOR), ("wrapper", WRAPPER), ("focused_tests", FOCUSED_TESTS)):
        pin = manifest.get(key)
        if not isinstance(pin, dict):
            raise fail(f"manifest/{key} missing")
        assert_equal(pin.get("path"), relative(path), f"manifest/{key}/path")
        assert_equal(pin.get("sha256"), sha256_file(path, f"Phase96 {key}"), f"manifest/{key}/sha256")
    inherited = manifest.get("inherited_raw_manifest")
    if not isinstance(inherited, dict):
        raise fail("manifest/inherited_raw_manifest missing")
    assert_equal(inherited.get("path"), relative(PHASE91_MANIFEST), "manifest/inherited_raw_manifest/path")
    assert_equal(inherited.get("sha256"), PHASE91_MANIFEST_SHA, "manifest/inherited_raw_manifest/sha256")
    assert_equal(inherited.get("raw_byte_reads_by_verifier"), 0, "manifest/inherited_raw_manifest/read_accounting")
    candidate = manifest.get("candidate")
    if not isinstance(candidate, dict):
        raise fail("manifest/candidate missing")
    for key, expected in {
        "id": "phase96_main_c0d_factor_family_lm_trial_diagnostic_telemetry",
        "selector": SELECTOR,
        "phase93_selector": PHASE93_SELECTOR,
        "candidate_count": 1,
        "diagnostic_only": True,
        "default_off": True,
        "raw_only": True,
        "runs_per_route": 1,
        "controls": 0,
        "accuracy_scoring": False,
        "truth_evaluation": False,
        "submission_release": False,
        "no_solver_filter_or_lm_change": True,
        "no_fallback_or_guard_bypass": True,
        "no_solution_output_publication": True,
        "raw_content_copy_or_transform": False,
    }.items():
        assert_equal(candidate.get(key), expected, f"manifest/candidate/{key}")
    raw_contract = candidate.get("raw_input_contract")
    if not isinstance(raw_contract, dict):
        raise fail("manifest/candidate/raw_input_contract missing")
    for key, expected in {
        "input_names_exact": list(RAW_NAMES),
        "device_gnss_only": True,
        "device_imu_only": True,
        "broadcast_navigation_only": True,
        "truth_mat_base_precomputed_coordinate_kaggle_accuracy": False,
        "copy_or_transform": False,
    }.items():
        assert_equal(raw_contract.get(key), expected, f"manifest/raw_contract/{key}")
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
        _validate_command(route, record.get("command"), record)
    matrix = manifest.get("matrix")
    if not isinstance(matrix, dict):
        raise fail("manifest/matrix missing")
    for key, expected in {
        "candidate_count": 1,
        "routes": 2,
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
    }.items():
        assert_equal(matrix.get(key), expected, f"manifest/matrix/{key}")
    gates = manifest.get("structural_gates")
    if not isinstance(gates, dict):
        raise fail("manifest/structural_gates missing")
    for key in (
        "required_before_stop",
        "gnss_first_guard_predicate_and_counts",
        "gnss_first_progress_or_fail_closed_telemetry",
        "main_validation_predicates",
        "main_factor_family_costs",
        "main_gradient_normal_norms_by_variable",
        "first_ten_existing_lm_trials",
        "linear_solver_exception_telemetry",
        "no_solution_or_accuracy_publication",
        "command_policy_no_pdc_external_precomputed",
    ):
        assert_equal(gates.get(key), True, f"manifest/structural_gates/{key}")
    qualification = manifest.get("qualification_evidence")
    if not isinstance(qualification, dict):
        raise fail("manifest/qualification_evidence missing")
    cpp = qualification.get("full_cpp_suite")
    if not isinstance(cpp, dict):
        raise fail("manifest/qualification/full_cpp_suite missing")
    for key, expected in {
        "tests_run": 1104,
        "passed": 1046,
        "skipped": 58,
        "failed": 0,
        "exit_code": 0,
        "raw_route_execution": False,
    }.items():
        assert_equal(cpp.get(key), expected, f"manifest/qualification/full_cpp_suite/{key}")
    focused = qualification.get("focused_cpp_phase96")
    if not isinstance(focused, dict):
        raise fail("manifest/qualification/focused_cpp_phase96 missing")
    for key, expected in {"tests_run": 6, "passed": 6, "failed": 0, "exit_code": 0, "raw_route_execution": False}.items():
        assert_equal(focused.get(key), expected, f"manifest/qualification/focused_cpp_phase96/{key}")
    focused_py = qualification.get("focused_python")
    if not isinstance(focused_py, dict):
        raise fail("manifest/qualification/focused_python missing")
    for key, expected in {"tests_run": 6, "passed": 6, "failed": 0, "exit_code": 0, "raw_route_execution": False}.items():
        assert_equal(focused_py.get(key), expected, f"manifest/qualification/focused_python/{key}")
    assert_equal(qualification.get("diff_check_clean"), True, "manifest/qualification/diff_check_clean")
    archive = manifest.get("workspace_archive_record")
    if not isinstance(archive, dict):
        raise fail("manifest/workspace_archive_record missing")
    assert_equal(archive.get("user_data_deleted"), False, "manifest/archive/user_data_deleted")
    originals = archive.get("original_paths")
    current = archive.get("current_paths")
    if originals != [
        "/tmp/phase38-debug.PhRaoH/libgnss_lib.a",
        "/tmp/phase38-debug.PhRaoH/libgnss_lib_solvers.a",
    ]:
        raise fail("manifest/archive/original_paths changed")
    if current != [
        "/dev/shm/phase96-build-space/libgnss_lib.a",
        "/dev/shm/phase96-build-space/libgnss_lib_solvers.a",
    ]:
        raise fail("manifest/archive/current_paths changed")
    execution = manifest.get("execution_authorization")
    if not isinstance(execution, dict):
        raise fail("manifest/execution_authorization missing")
    authority = manifest.get("authority") or {}
    auth_pin = authority.get("raw_execution_authorization")
    assert_equal(execution.get("raw_execution_authorized"), isinstance(auth_pin, dict), "manifest/execution_authorization/raw_execution_authorized")
    for key, expected in {
        "before_raw_execution": True,
        "native_route_rerun_performed": False,
        "accuracy_or_submission_release": False,
        "stop_after_two_routes": True,
    }.items():
        assert_equal(execution.get(key), expected, f"manifest/execution_authorization/{key}")
    accounting = manifest.get("read_accounting_before_execution")
    if not isinstance(accounting, dict):
        raise fail("manifest/read_accounting_before_execution missing")
    for key in (
        "native_solver_invocations",
        "raw_device_gnss_reads",
        "raw_device_imu_reads",
        "broadcast_navigation_reads",
        "truth_reads",
        "mat_reads_or_generated",
        "precomputed_coordinate_reads",
        "base_rinex_reads",
        "accuracy_calculations",
        "kaggle_or_token_access",
        "raw_input_hash_reads",
        "route_reruns",
        "fallbacks",
    ):
        assert_equal(accounting.get(key), 0, f"manifest/read_accounting/{key}")
    if isinstance(auth_pin, dict):
        assert_equal(auth_pin.get("path"), relative(AUTHORIZATION), "manifest/authority/auth/path")
        assert_equal(auth_pin.get("role"), "separate one-shot authorization; final manifest hash is pinned by the authorization", "manifest/authority/auth/role")
    return manifest


def verify_authorization(manifest: dict[str, Any]) -> dict[str, Any]:
    authorization = read_json(AUTHORIZATION, "Phase96 raw execution authorization")
    for key, expected in {
        "schema_version": AUTH_SCHEMA,
        "phase": 96,
        "status": "authorized-for-exact-two-route-phase96-main-c0d-diagnostic-execution",
        "execution_label": "Luna Max",
    }.items():
        assert_equal(authorization.get(key), expected, f"authorization/{key}")
    authority = authorization.get("authority")
    if not isinstance(authority, dict):
        raise fail("authorization/authority missing")
    pins = {
        "phase96_freeze": (FREEZE, FREEZE_SHA),
        "phase96_manifest": (MANIFEST, sha256_file(MANIFEST, "Phase96 manifest")),
        "phase96_evaluator": (EVALUATOR, sha256_file(EVALUATOR, "Phase96 evaluator")),
        "phase96_wrapper": (WRAPPER, sha256_file(WRAPPER, "Phase96 wrapper")),
        "phase96_focused_tests": (FOCUSED_TESTS, sha256_file(FOCUSED_TESTS, "Phase96 focused tests")),
        "phase91_inherited_raw_manifest": (PHASE91_MANIFEST, PHASE91_MANIFEST_SHA),
    }
    for key, (path, digest) in pins.items():
        pin = authority.get(key)
        if not isinstance(pin, dict):
            raise fail(f"authorization/{key} missing")
        assert_equal(pin.get("path"), relative(path), f"authorization/{key}/path")
        assert_equal(pin.get("sha256"), digest, f"authorization/{key}/sha256")
    assert_equal(authority.get("freeze_commit"), FREEZE_COMMIT, "authorization/freeze/commit")
    assert_equal(authority.get("implementation_commit"), IMPLEMENTATION_COMMIT, "authorization/implementation/commit")
    assert_equal(authority.get("implementation_source_sha256"), IMPLEMENTATION_SOURCE_SHA, "authorization/implementation/source")
    assert_equal(authority.get("binary_sha256"), BINARY_SHA, "authorization/implementation/binary")
    for relative_path, expected in IMPLEMENTATION_FILES.items():
        assert_equal(authority.get("implementation_files", {}).get(relative_path), expected, f"authorization/implementation_files/{relative_path}")
    candidate = authorization.get("candidate")
    if not isinstance(candidate, dict):
        raise fail("authorization/candidate missing")
    for key, expected in {
        "candidate_count": 1,
        "id": "phase96_main_c0d_factor_family_lm_trial_diagnostic_telemetry",
        "selector": SELECTOR,
        "phase93_selector": PHASE93_SELECTOR,
        "diagnostic_only": True,
        "raw_only": True,
        "default_off_outside_this_explicit_command": True,
        "runs_per_route": 1,
        "controls": 0,
        "raw_input_names_exact": list(RAW_NAMES),
        "truth_mat_coordinate_base_kaggle_accuracy": False,
        "raw_content_copy_or_transform": False,
        "solution_output_publication": False,
    }.items():
        assert_equal(candidate.get(key), expected, f"authorization/candidate/{key}")
    routes = authorization.get("routes")
    if routes != list(ROUTES):
        raise fail("authorization route order changed")
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
        "route_score_selection": False,
    }.items():
        assert_equal(matrix.get(key), expected, f"authorization/matrix/{key}")
    policy = authorization.get("execution_policy")
    if not isinstance(policy, dict):
        raise fail("authorization/execution_policy missing")
    for key, expected in {
        "authorized": True,
        "commands_source": "exact two route commands from the pinned Phase96 manifest after inherited Phase91 path materialization",
        "order": "MTV-A then LAX-T, sequentially",
        "one_invocation_per_route": True,
        "no_rerun": True,
        "stop_after_two_routes": True,
        "on_route_failure": "record fail-closed diagnostic and continue the remaining route once; no fallback",
        "raw_input_hashes": "not requested by wrapper before or outside native route reads",
        "solver_filter_lm_changes": False,
        "fallback_or_guard_bypass": False,
        "solution_output": "required CLI path is withheld and must not be published or committed",
    }.items():
        assert_equal(policy.get(key), expected, f"authorization/execution_policy/{key}")
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
        assert_equal(boundary.get(key), expected, f"authorization/release_boundary/{key}")
    read_accounting = authorization.get("read_accounting_before_execution")
    if not isinstance(read_accounting, dict):
        raise fail("authorization/read_accounting_before_execution missing")
    for key, expected in {
        "native_solver_invocations": 0,
        "raw_device_gnss_reads": 0,
        "raw_device_imu_reads": 0,
        "broadcast_navigation_reads": 0,
        "truth_reads": 0,
        "mat_reads_or_generated": 0,
        "precomputed_coordinate_reads": 0,
        "base_rinex_reads": 0,
        "accuracy_calculations": 0,
        "kaggle_or_token_access": 0,
        "raw_input_hash_reads": 0,
        "route_reruns": 0,
        "fallbacks": 0,
    }.items():
        assert_equal(read_accounting.get(key), expected, f"authorization/read_accounting/{key}")
    assert_equal(authority.get("manifest_sha256"), sha256_file(MANIFEST, "final Phase96 manifest"), "authorization/final_manifest_sha256")
    assert_equal(manifest.get("status"), "sealed-before-phase96-main-c0d-diagnostic-raw-execution", "authorization/manifest/status")
    return authorization


def verify_pre_raw() -> dict[str, Any]:
    freeze = verify_freeze()
    manifest = verify_manifest()
    execution = manifest.get("execution_authorization", {})
    authority = manifest.get("authority") or {}
    authorization = None
    if execution.get("raw_execution_authorized") is True:
        authorization = verify_authorization(manifest)
    source = EVALUATOR.read_text(encoding="utf-8")
    # Construct the launch spellings so this self-audit does not match its
    # own list of forbidden tokens.
    for forbidden in (
        "sub" + "process",
        "P" + "open",
        "os." + "system",
        "os." + "popen",
        "os." + "spawn",
        "exec" + "ve",
    ):
        if forbidden in source:
            raise fail(f"pre-raw verifier contains forbidden launch token: {forbidden}")
    return {
        "status": "pre-raw-verified",
        "phase": 96,
        "execution_label": "Luna Max",
        "candidate_count": 1,
        "routes": 2,
        "route_ids": list(ROUTES),
        "runs_per_route": 1,
        "raw_execution_authorized": authorization is not None,
        "raw_reads": 0,
        "native_solver_invocations": 0,
        "truth_reads": 0,
        "mat_reads_or_generated": 0,
        "precomputed_coordinate_reads": 0,
        "accuracy_scored": False,
        "route_score_selection": False,
        "freeze_sha256": FREEZE_SHA,
        "manifest_sha256": sha256_file(MANIFEST, "Phase96 execution manifest"),
        "authorization_sha256": sha256_file(AUTHORIZATION, "Phase96 authorization") if authorization is not None else None,
        "implementation_commit": IMPLEMENTATION_COMMIT,
        "freeze_status": freeze["status"],
        "manifest_status": manifest["status"],
        "authorization_status": authorization["status"] if authorization is not None else "not-issued-before-separate-raw-authorization",
        "manifest_authority_pin_present": isinstance(authority.get("raw_execution_authorization"), dict),
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--verify-freeze", action="store_true")
    parser.add_argument("--verify-manifest", action="store_true")
    parser.add_argument("--verify-pre-raw", action="store_true")
    args = parser.parse_args(argv)
    if not any((args.verify_freeze, args.verify_manifest, args.verify_pre_raw)):
        parser.error("one verification mode is required")
    try:
        if args.verify_freeze:
            verify_freeze()
        if args.verify_manifest:
            verify_manifest()
        if args.verify_pre_raw:
            print(json.dumps(verify_pre_raw(), indent=2, sort_keys=True))
        return 0
    except (Phase96PreRawError, OSError) as exc:
        print(f"phase96 pre-raw verifier: fail-closed: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
