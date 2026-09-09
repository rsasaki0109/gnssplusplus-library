#!/usr/bin/env python3
"""Launch-free verifier for the Phase97 singular-system diagnostic lane.

This module only reads pinned source/JSON records.  It never launches a
process, opens raw GNSS/IMU/navigation bytes, hashes raw files, evaluates
truth, or exposes solution/accuracy output.  The execution wrapper imports it
after the separate one-shot authorization has been committed.
"""

from __future__ import annotations

import hashlib
import json
from pathlib import Path
import sys
from typing import Any


ROOT = Path(__file__).resolve().parents[3]
FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_phase97_singular_system_diagnostic_freeze_v1.json"
MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase97_singular_system_diagnostic_execution_manifest_v1.json"
AUTHORIZATION = ROOT / "docs/use_cases/records/smartphone_r5_phase97_singular_system_diagnostic_raw_execution_authorization_v1.json"
PHASE95_RESULT = ROOT / "docs/use_cases/records/smartphone_r5_phase95_raw_input_path_corrected_structural_result_v1.json"
PHASE95_MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase95_raw_input_path_corrected_execution_manifest_v1.json"
EVALUATOR = Path(__file__).resolve()
WRAPPER = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase97_singular_system_diagnostic_execute.py"
FOCUSED_TESTS = ROOT / "tests/test_smartphone_phase97_singular_system_diagnostic.py"
APP = ROOT / "apps/native/gnss_fgo_imu_no_base.cpp"
INTERNAL = ROOT / "src/algorithms/fgo_gtsam_internal.hpp"
BINARY = ROOT / "build/apps/gnss_fgo_imu_no_base"

FREEZE_SHA = "07723240dc56ae80c3d21e25b783fb086c5e9ea4af47e62cd56fc5b81980d174"
FREEZE_COMMIT = "587ef8db667b47ec539aba9d0a76d43698d8bc53"
IMPLEMENTATION_COMMIT = "e912d4571ba684453a0bc6fa73cdd0b99d9ffc86"
IMPLEMENTATION_FILES = {
    "apps/native/gnss_fgo_imu_no_base.cpp": "c0c49d35df4eb0abf56c2ef64d0beb80c0c6bc16023b987a74bb2d42cd77b55a",
    "include/libgnss++/algorithms/fgo.hpp": "fb984aae9a686f48be9b3e9917d2cfa24cb0e07fc1d6f6f00c717be1294e8700",
    "include/libgnss++/algorithms/fgo_config.hpp": "0bb86f2b9755f7de68c1274f4eebb99bf441f597514c5a421b2c25cc8906f6e9",
    "src/algorithms/fgo.cpp": "2f745bc80ced47d906cafefd1a6e1d3102ecad8643cc666af55d9fd9ab8896f9",
    "src/algorithms/fgo_gtsam_backend.cpp": "26f5822d5ed31b8a1b739aecf94afe17ec71ac8847d02a8bac901f28f9f12379",
    "src/algorithms/fgo_gtsam_internal.hpp": "4ba74c23855472a785f3d2ad57a2c85fd80e233b85a54a2c230f2b5ef6e905f8",
    "tests/test_fgo_gtsam_backend.cpp": "c06e4f31d697a895d0b19147a63880e037bde716640555332b9601c4f106a64b",
}
APP_SHA = IMPLEMENTATION_FILES["apps/native/gnss_fgo_imu_no_base.cpp"]
BINARY_SHA = "131deaf87ec1f368a92f23a68578ec9d31dc90864a1fa4edb1ad52501db20a64"
PHASE95_RESULT_SHA = "beba25d4b0d910064d76bea7aa6b7a884937ca01c3b00cf0f7f9f4b6267ea281"
PHASE95_MANIFEST_SHA = "9cfdcf92fe01b5481ae3b5bd414484abe5d5ac4d33f76b6aea1b3b412d94185f"

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
BINARY_RELATIVE = "build/apps/gnss_fgo_imu_no_base"
OUTPUT_ROOT = "output/smartphone-r5/phase97-singular-system-diagnostic-v1/"
SELECTOR = "--native-source-clock-c0d-phase97-singular-system-diagnostics"
PHASE93_SELECTOR = "--native-source-clock-c0d-gnss-first-meter-state-handoff"
SCHEMA = "smartphone-r5-phase97-singular-system-diagnostic-execution-manifest.v1"
AUTH_SCHEMA = "smartphone-r5-phase97-singular-system-diagnostic-raw-execution-authorization.v1"
CANDIDATE_ID = "phase97_main_c0d_singular_system_key_connectivity_rank_diagnostic"

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
    "--native-source-clock-c0d-phase96-main-diagnostics",
)
FORBIDDEN_PATH_TERMS = (
    ".mat", "ground_truth", "validation", "holdout", "kaggle", "token",
    "base.rinex", "coordinate", "truth",
)


class Phase97PreRawError(ValueError):
    """Raised when the sealed Phase97 pre-raw contract fails closed."""


def fail(message: str) -> Phase97PreRawError:
    return Phase97PreRawError(message)


def relative(path: Path) -> str:
    try:
        return str(path.relative_to(ROOT))
    except ValueError:
        return str(path)


def sha256_file(path: Path, label: str) -> str:
    """Hash only pinned source/record files, never a raw input path."""

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
    """Read only sealed Phase95 path metadata; do not stat or open raw files."""

    assert_equal(sha256_file(PHASE95_RESULT, "Phase95 result"), PHASE95_RESULT_SHA, "Phase95 result/sha256")
    assert_equal(sha256_file(PHASE95_MANIFEST, "Phase95 manifest"), PHASE95_MANIFEST_SHA, "Phase95 manifest/sha256")
    result = read_json(PHASE95_RESULT, "Phase95 result")
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
    assert_equal(sha256_file(FREEZE, "Phase97 freeze"), FREEZE_SHA, "freeze/sha256")
    freeze = read_json(FREEZE, "Phase97 freeze")
    for key, expected in {
        "schema_version": "smartphone-r5-phase97-singular-system-diagnostic-freeze.v1",
        "phase": 97,
        "execution_label": "Luna Max",
        "status": "frozen-diagnostic-only-not-implemented",
    }.items():
        assert_equal(freeze.get(key), expected, f"freeze/{key}")
    scope = freeze.get("scope")
    if not isinstance(scope, dict):
        raise fail("freeze/scope missing")
    assert_equal(scope.get("routes"), list(ROUTES), "freeze/scope/routes")
    assert_equal(scope.get("route_selection_by_score_or_truth"), False, "freeze/scope/selection")
    candidate = freeze.get("candidate")
    if not isinstance(candidate, dict):
        raise fail("freeze/candidate missing")
    for key, expected in {
        "id": CANDIDATE_ID,
        "kind": "source-aligned-observability-only",
        "implemented": False,
        "default_enabled": False,
        "raw_execution_authorized": False,
        "solver_rerun_authorized": False,
        "algorithmic_effect": "None. Existing graph construction, factor equations, values, ordering, filters, priors, noise models, LM parameters, acceptance branches, fallback policy, and fail-closed gates remain authoritative.",
    }.items():
        assert_equal(candidate.get(key), expected, f"freeze/candidate/{key}")
    policy = freeze.get("publication_and_execution_policy")
    if not isinstance(policy, dict):
        raise fail("freeze/publication_and_execution_policy missing")
    for key in (
        "diagnostic_payload_only", "raw_observations_excluded", "coordinates_excluded",
        "solution_rows_excluded", "truth_excluded", "mat_excluded",
        "kaggle_or_token_excluded", "accuracy_excluded", "fail_closed_preserved",
    ):
        assert_equal(policy.get(key), True, f"freeze/policy/{key}")
    assert_equal(policy.get("native_solver_invocations_at_freeze"), 0, "freeze/policy/solver_count")
    assert_equal(policy.get("raw_execution_authorized"), False, "freeze/policy/raw_authorized")
    return freeze


def verify_implementation() -> None:
    assert_equal(sha256_file(APP, "native implementation"), APP_SHA, "implementation/app")
    assert_equal(sha256_file(BINARY, "native binary"), BINARY_SHA, "implementation/binary")
    for path_text, expected in IMPLEMENTATION_FILES.items():
        assert_equal(sha256_file(ROOT / path_text, f"implementation/{path_text}"), expected, f"implementation/{path_text}")
    source = APP.read_text(encoding="utf-8")
    internal = INTERNAL.read_text(encoding="utf-8")
    config = (ROOT / "include/libgnss++/algorithms/fgo_config.hpp").read_text(encoding="utf-8")
    for marker in (SELECTOR, "phase97_main", "collectPhase97InitialGraphDiagnostics", "nearby_variable_unavailable"):
        if marker not in source and marker not in config and marker not in internal:
            raise fail(f"Phase97 implementation marker missing: {marker}")
    if "use_native_source_clock_c0d_phase97_singular_system_diagnostics =\n            false" not in config:
        raise fail("Phase97 default is not explicitly disabled")


def _output_path(route: str, name: str) -> str:
    return f"{OUTPUT_ROOT}{route.replace('/', '__')}/{name}"


def _validate_command(route: str, command: Any, record: dict[str, Any], paths: dict[str, str]) -> None:
    if not isinstance(command, list) or not command or not all(isinstance(token, str) for token in command):
        raise fail(f"malformed command: {route}")
    assert_equal(command[0], BINARY_RELATIVE, f"command/{route}/binary")
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
        pin = raw[name]
        if not isinstance(pin, dict):
            raise fail(f"manifest raw pin malformed: {route}/{name}")
        assert_equal(pin.get("path"), paths[name], f"manifest/raw/{route}/{name}/path")
        assert_equal(pin.get("path_source"), "Phase95 sealed result raw_inputs.path", f"manifest/raw/{route}/{name}/source")
        assert_equal(pin.get("sha256"), None, f"manifest/raw/{route}/{name}/sha256")
        assert_equal(pin.get("read_at_manifest_creation"), False, f"manifest/raw/{route}/{name}/read")
    planned = record.get("planned_output")
    if not isinstance(planned, dict):
        raise fail(f"planned output missing: {route}")
    assert_equal(command.count("--out"), 1, f"command/{route}/out-count")
    assert_equal(command[command.index("--out") + 1], _output_path(route, "withheld_solution_output.csv"), f"command/{route}/out")
    assert_equal(command.count("--summary-json"), 1, f"command/{route}/summary-count")
    assert_equal(command[command.index("--summary-json") + 1], _output_path(route, "summary.json"), f"command/{route}/summary")
    assert_equal(planned.get("summary"), _output_path(route, "summary.json"), f"manifest/output/{route}/summary")
    assert_equal(planned.get("withheld_solution_output"), _output_path(route, "withheld_solution_output.csv"), f"manifest/output/{route}/withheld")


def verify_manifest() -> dict[str, Any]:
    verify_freeze()
    verify_implementation()
    paths = phase95_paths()
    manifest = read_json(MANIFEST, "Phase97 execution manifest")
    for key, expected in {
        "schema_version": SCHEMA,
        "phase": 97,
        "execution_label": "Luna Max",
        "status": "sealed-before-phase97-singular-system-diagnostic-raw-execution",
    }.items():
        assert_equal(manifest.get(key), expected, f"manifest/{key}")
    freeze_ref = manifest.get("freeze")
    if not isinstance(freeze_ref, dict):
        raise fail("manifest/freeze missing")
    for key, expected in {"path": relative(FREEZE), "sha256": FREEZE_SHA, "commit": FREEZE_COMMIT, "execution_authorized_before_this_manifest": False}.items():
        assert_equal(freeze_ref.get(key), expected, f"manifest/freeze/{key}")
    implementation = manifest.get("implementation")
    if not isinstance(implementation, dict):
        raise fail("manifest/implementation missing")
    for key, expected in {"commit": IMPLEMENTATION_COMMIT, "native_algorithm_changed": False, "solver_filter_lm_changed": False, "equation_units_sigma_changed": False}.items():
        assert_equal(implementation.get(key), expected, f"manifest/implementation/{key}")
    assert_equal(implementation.get("source", {}).get("sha256"), APP_SHA, "manifest/implementation/source")
    assert_equal(implementation.get("binary", {}).get("sha256"), BINARY_SHA, "manifest/implementation/binary")
    changed = implementation.get("changed_files")
    if not isinstance(changed, list) or {item.get("path") for item in changed if isinstance(item, dict)} != set(IMPLEMENTATION_FILES):
        raise fail("manifest/implementation/changed_files set changed")
    for item in changed:
        if isinstance(item, dict):
            assert_equal(item.get("sha256"), IMPLEMENTATION_FILES[item["path"]], f"manifest/implementation/{item['path']}/sha256")
            assert_equal(item.get("diagnostic_or_test_only"), True, f"manifest/implementation/{item['path']}/scope")
    for key, path in (("pre_raw_evaluator", EVALUATOR), ("execution_wrapper", WRAPPER), ("focused_tests", FOCUSED_TESTS)):
        pin = manifest.get(key)
        if not isinstance(pin, dict):
            raise fail(f"manifest/{key} missing")
        assert_equal(pin.get("path"), relative(path), f"manifest/{key}/path")
        assert_equal(pin.get("sha256"), sha256_file(path, f"Phase97 {key}"), f"manifest/{key}/sha256")
    inherited = manifest.get("phase95_raw_path_source")
    if not isinstance(inherited, dict):
        raise fail("manifest/phase95_raw_path_source missing")
    assert_equal(inherited.get("path"), relative(PHASE95_RESULT), "manifest/Phase95 result/path")
    assert_equal(inherited.get("sha256"), PHASE95_RESULT_SHA, "manifest/Phase95 result/sha256")
    assert_equal(inherited.get("raw_byte_reads_by_verifier"), 0, "manifest/Phase95 result/read_accounting")
    assert_equal(inherited.get("materialization"), "exact Phase95 result raw_inputs.path substitution for GNSS/IMU/nav only", "manifest/Phase95 result/materialization")
    candidate = manifest.get("candidate")
    if not isinstance(candidate, dict):
        raise fail("manifest/candidate missing")
    for key, expected in {"id": CANDIDATE_ID, "selector": SELECTOR, "phase93_selector": PHASE93_SELECTOR, "candidate_count": 1, "diagnostic_only": True, "raw_only": True, "runs_per_route": 1, "controls": 0, "accuracy_scoring": False, "truth_evaluation": False, "submission_release": False, "no_solver_filter_or_lm_change": True, "no_fallback_or_guard_bypass": True, "no_solution_output_publication": True, "raw_content_copy_or_transform": False}.items():
        assert_equal(candidate.get(key), expected, f"manifest/candidate/{key}")
    raw_contract = candidate.get("raw_input_contract")
    if not isinstance(raw_contract, dict):
        raise fail("manifest/candidate/raw_input_contract missing")
    for key, expected in {"input_names_exact": list(RAW_NAMES), "device_gnss_only": True, "device_imu_only": True, "broadcast_navigation_only": True, "truth_mat_base_precomputed_coordinate_kaggle_accuracy": False, "copy_or_transform": False}.items():
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
        _validate_command(route, record.get("command"), record, paths[route])
    matrix = manifest.get("matrix")
    if not isinstance(matrix, dict):
        raise fail("manifest/matrix missing")
    for key, expected in {"candidate_count": 1, "routes": 2, "runs_per_route": 1, "native_invocations_planned": 2, "controls": 0, "reruns": 0, "fallbacks": 0, "raw_device_gnss_route_arguments": 2, "raw_device_imu_route_arguments": 2, "broadcast_navigation_route_arguments": 2, "truth_reads_planned": 0, "mat_reads_or_generated_planned": 0, "precomputed_coordinate_reads_planned": 0, "base_rinex_reads_planned": 0, "accuracy_calculations_planned": 0, "kaggle_or_token_access_planned": 0, "route_score_selection": False}.items():
        assert_equal(matrix.get(key), expected, f"manifest/matrix/{key}")
    gates = manifest.get("structural_gates")
    if not isinstance(gates, dict):
        raise fail("manifest/structural_gates missing")
    for key in ("required_before_stop", "gnss_first_guard_predicate_and_counts", "gnss_first_progress_or_fail_closed_telemetry", "main_validation_predicates", "variable_family_counts_and_dimensions", "factor_incidence_and_components", "anchor_and_prior_incidence", "zero_and_near_zero_column_buckets", "rank_or_nullspace_attribution", "first_ten_existing_lm_trials", "nearby_variable_status", "linear_solver_exception_telemetry", "no_solution_or_accuracy_publication"):
        assert_equal(gates.get(key), True, f"manifest/structural_gates/{key}")
    qualification = manifest.get("qualification_evidence")
    if not isinstance(qualification, dict):
        raise fail("manifest/qualification_evidence missing")
    for key, expected in {"tests_run": 1107, "passed": 1049, "skipped": 58, "failed": 0, "exit_code": 0, "raw_route_execution": False}.items():
        assert_equal(qualification.get("full_cpp_suite", {}).get(key), expected, f"qualification/full_cpp/{key}")
    for key, expected in {"tests_run": 4, "passed": 4, "failed": 0, "exit_code": 0, "raw_route_execution": False}.items():
        assert_equal(qualification.get("focused_cpp_phase97", {}).get(key), expected, f"qualification/focused_cpp/{key}")
    for key, expected in {"tests_run": 6, "passed": 6, "failed": 0, "exit_code": 0, "raw_route_execution": False}.items():
        assert_equal(qualification.get("focused_python", {}).get(key), expected, f"qualification/focused_python/{key}")
    assert_equal(qualification.get("diff_check_clean"), True, "qualification/diff_check_clean")
    execution = manifest.get("execution_authorization")
    if not isinstance(execution, dict):
        raise fail("manifest/execution_authorization missing")
    for key, expected in {"before_raw_execution": True, "raw_execution_authorized": False, "native_route_rerun_performed": False, "accuracy_or_submission_release": False, "stop_after_two_routes": True}.items():
        assert_equal(execution.get(key), expected, f"manifest/execution_authorization/{key}")
    authority = manifest.get("authority")
    if not isinstance(authority, dict) or not isinstance(authority.get("raw_execution_authorization"), dict):
        raise fail("manifest/authority/raw_execution_authorization missing")
    accounting = manifest.get("read_accounting_before_execution")
    if not isinstance(accounting, dict):
        raise fail("manifest/read_accounting_before_execution missing")
    for key in ("native_solver_invocations", "raw_device_gnss_reads", "raw_device_imu_reads", "broadcast_navigation_reads", "truth_reads", "mat_reads_or_generated", "precomputed_coordinate_reads", "base_rinex_reads", "accuracy_calculations", "kaggle_or_token_access", "raw_input_hash_reads", "route_reruns", "fallbacks"):
        assert_equal(accounting.get(key), 0, f"manifest/read_accounting/{key}")
    assert_equal(accounting.get("raw_content_copied_or_transformed"), False, "manifest/read_accounting/copy")
    return manifest


def verify_authorization(manifest: dict[str, Any]) -> dict[str, Any]:
    authorization = read_json(AUTHORIZATION, "Phase97 raw execution authorization")
    for key, expected in {"schema_version": AUTH_SCHEMA, "phase": 97, "execution_label": "Luna Max", "status": "authorized-for-exact-two-route-phase97-singular-system-diagnostic-execution"}.items():
        assert_equal(authorization.get(key), expected, f"authorization/{key}")
    authority = authorization.get("authority")
    if not isinstance(authority, dict):
        raise fail("authorization/authority missing")
    pins = {"phase97_freeze": (FREEZE, FREEZE_SHA), "phase97_manifest": (MANIFEST, sha256_file(MANIFEST, "Phase97 manifest")), "phase97_evaluator": (EVALUATOR, sha256_file(EVALUATOR, "Phase97 evaluator")), "phase97_wrapper": (WRAPPER, sha256_file(WRAPPER, "Phase97 wrapper")), "phase97_focused_tests": (FOCUSED_TESTS, sha256_file(FOCUSED_TESTS, "Phase97 focused tests")), "phase95_result": (PHASE95_RESULT, PHASE95_RESULT_SHA), "phase95_manifest": (PHASE95_MANIFEST, PHASE95_MANIFEST_SHA)}
    for key, (path, digest) in pins.items():
        pin = authority.get(key)
        if not isinstance(pin, dict):
            raise fail(f"authorization/{key} missing")
        assert_equal(pin.get("path"), relative(path), f"authorization/{key}/path")
        assert_equal(pin.get("sha256"), digest, f"authorization/{key}/sha256")
    assert_equal(authority.get("freeze_commit"), FREEZE_COMMIT, "authorization/freeze_commit")
    assert_equal(authority.get("implementation_commit"), IMPLEMENTATION_COMMIT, "authorization/implementation_commit")
    assert_equal(authority.get("implementation_source_sha256"), APP_SHA, "authorization/implementation_source")
    assert_equal(authority.get("binary_sha256"), BINARY_SHA, "authorization/binary")
    if authority.get("pre_raw_qualification_commit") in (None, ""):
        raise fail("authorization/pre_raw_qualification_commit missing")
    if authority.get("manifest_sha256") != sha256_file(MANIFEST, "final Phase97 manifest"):
        raise fail("authorization/final manifest hash mismatch")
    candidate = authorization.get("candidate")
    if not isinstance(candidate, dict):
        raise fail("authorization/candidate missing")
    for key, expected in {"candidate_count": 1, "id": CANDIDATE_ID, "selector": SELECTOR, "phase93_selector": PHASE93_SELECTOR, "diagnostic_only": True, "raw_only": True, "default_off_outside_this_explicit_command": True, "runs_per_route": 1, "controls": 0, "raw_input_names_exact": list(RAW_NAMES), "truth_mat_coordinate_base_kaggle_accuracy": False, "raw_content_copy_or_transform": False, "solution_output_publication": False}.items():
        assert_equal(candidate.get(key), expected, f"authorization/candidate/{key}")
    assert_equal(authorization.get("routes"), list(ROUTES), "authorization/routes")
    matrix = authorization.get("matrix")
    if not isinstance(matrix, dict):
        raise fail("authorization/matrix missing")
    for key, expected in {"candidate_count": 1, "route_count": 2, "runs_per_route": 1, "native_invocations": 2, "controls": 0, "reruns": 0, "fallbacks": 0, "raw_device_gnss_reads_max": 2, "raw_device_imu_reads_max": 2, "broadcast_navigation_reads_max": 2, "truth_reads": 0, "mat_reads_or_generated": 0, "precomputed_coordinate_reads": 0, "base_rinex_reads": 0, "accuracy_calculations": 0, "kaggle_or_token_access": 0, "route_score_selection": False}.items():
        assert_equal(matrix.get(key), expected, f"authorization/matrix/{key}")
    policy = authorization.get("execution_policy")
    if not isinstance(policy, dict):
        raise fail("authorization/execution_policy missing")
    for key, expected in {"authorized": True, "order": "MTV-A then LAX-T, sequentially", "one_invocation_per_route": True, "no_rerun": True, "stop_after_two_routes": True, "solver_filter_lm_changes": False, "fallback_or_guard_bypass": False, "solution_output": "required CLI path is withheld and must not be published or committed"}.items():
        assert_equal(policy.get(key), expected, f"authorization/policy/{key}")
    boundary = authorization.get("release_boundary")
    if not isinstance(boundary, dict):
        raise fail("authorization/release_boundary missing")
    for key, expected in {"raw_execution_authorized": True, "diagnostic_structural_result_authorized": True, "truth_or_accuracy_evaluation_authorized": False, "submission_release_authorized": False, "promotion_authorized": False, "solution_rows_authorized": False}.items():
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
    if AUTHORIZATION.is_file():
        authorized = verify_authorization(manifest).get("status") == "authorized-for-exact-two-route-phase97-singular-system-diagnostic-execution"
    return {"status": "pre-raw-verified", "phase": 97, "execution_label": "Luna Max", "candidate_count": 1, "routes": 2, "route_ids": list(ROUTES), "runs_per_route": 1, "raw_execution_authorized": authorized, "raw_reads": 0, "native_solver_invocations": 0, "truth_reads": 0, "mat_reads_or_generated": 0, "precomputed_coordinate_reads": 0, "accuracy_scored": False, "route_score_selection": False, "freeze_sha256": FREEZE_SHA, "manifest_sha256": sha256_file(MANIFEST, "Phase97 manifest"), "phase95_result_sha256": PHASE95_RESULT_SHA, "freeze_status": freeze["status"], "manifest_status": manifest["status"], "authorization_status": "authorized" if authorized else "not-issued-before-separate-raw-authorization"}


if __name__ == "__main__":
    import argparse

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
    except (Phase97PreRawError, OSError) as exc:
        print(f"phase97 pre-raw verifier: fail-closed: {exc}", file=sys.stderr)
        raise SystemExit(2)
