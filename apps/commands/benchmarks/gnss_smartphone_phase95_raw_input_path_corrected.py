#!/usr/bin/env python3
"""Launch-free verifier for the Phase95 corrected raw-input path contract.

The Phase95 change is limited to wrapper path resolution.  This verifier reads
only source and sealed JSON records; it never launches a process and never
opens or hashes a raw GNSS, IMU, navigation, truth, MAT, coordinate, base, or
Kaggle artifact.
"""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import re
import sys
from typing import Any


ROOT = Path(__file__).resolve().parents[3]
FREEZE = ROOT / (
    "docs/use_cases/records/"
    "smartphone_r5_phase95_raw_input_path_availability_freeze_v1.json"
)
MANIFEST = ROOT / (
    "docs/use_cases/records/"
    "smartphone_r5_phase95_raw_input_path_corrected_execution_manifest_v1.json"
)
AUTHORIZATION = ROOT / (
    "docs/use_cases/records/"
    "smartphone_r5_phase95_raw_input_path_corrected_raw_execution_authorization_v1.json"
)
EVALUATOR = Path(__file__).resolve()
WRAPPER = ROOT / (
    "apps/commands/benchmarks/"
    "gnss_smartphone_phase95_raw_input_path_corrected_execute.py"
)
FOCUSED_TESTS = ROOT / "tests/test_smartphone_phase95_raw_input_path_corrected.py"
APP = ROOT / "apps/native/gnss_fgo_imu_no_base.cpp"
BINARY = ROOT / "build/apps/gnss_fgo_imu_no_base"
PHASE91_MANIFEST = ROOT / (
    "docs/use_cases/records/"
    "smartphone_r5_phase91_source_clock_c0d_gnss_first_raw_drift_d_initializer_execution_manifest_v1.json"
)
PHASE94_EVALUATOR = ROOT / (
    "apps/commands/benchmarks/"
    "gnss_smartphone_phase94_source_clock_c0d_stage_diagnostics.py"
)

FREEZE_SHA = "983761ee5ddeeab1ff9274d0a5aa062d3c5ed59aa55075f558a0af29e3d3f044"
PHASE91_MANIFEST_SHA = "2078a4e2c3b963f07744c435ec303ea848bc372c8efd5003f6cb7b3d4b4ce988"
PHASE94_SOURCE_SHA = "524a8c0e4703a0ea533a21a4775e82719abaab2d2229a725ccf272cf2ab5430a"
PHASE94_EVALUATOR_SHA = "5648892866e94e3c62a8ec96e2148d7b4e3634d4272702c39ca40c94837f5b77"
BINARY_SHA = "76f6cfe06becadb464bbda3b085ecde93efc803bb74188448ba4abe2caabb621"

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
RAW_NAMES = ("device_gnss.csv", "device_imu.csv", "brdc.nav")
RAW_FLAGS = (
    ("--android-gnss", "device_gnss.csv"),
    ("--android-imu", "device_imu.csv"),
    ("--nav", "brdc.nav"),
)
OUTPUT_ROOT = "output/smartphone-r5/phase95-raw-input-path-corrected-v1/"
BINARY_RELATIVE = "build/apps/gnss_fgo_imu_no_base"
SELECTOR = "--native-source-clock-c0d-phase94-stage-diagnostics"
PHASE93_SELECTOR = "--native-source-clock-c0d-gnss-first-meter-state-handoff"
SCHEMA = "smartphone-r5-phase95-raw-input-path-corrected-execution-manifest.v1"
AUTH_SCHEMA = "smartphone-r5-phase95-raw-input-path-corrected-raw-execution-authorization.v1"
FORBIDDEN_FLAGS = (
    "--obs",
    "--native-gnss-first-velocity-only-handoff",
    "--native-direct-doppler-wls-handoff",
    "--native-pdc-state-bridge",
    "--native-upstream-quality",
    "--native-base-rinex",
    "--native-base-pseudorange-compensation",
    "--native-base-pseudorange-source-miss-mask",
    "--native-base-pseudorange-preserve-additional-frequency-bands",
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


class Phase95PreRawError(ValueError):
    """Raised when the sealed Phase95 pre-raw contract fails."""


def fail(message: str) -> Phase95PreRawError:
    return Phase95PreRawError(message)


def relative(path: Path) -> str:
    try:
        return str(path.relative_to(ROOT))
    except ValueError:
        return str(path)


def sha256_file(path: Path, label: str) -> str:
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
    return f"raw/phase93/{route}/{name}"


def _output_path(route: str, name: str) -> str:
    return f"{OUTPUT_ROOT}{route.replace('/', '__')}/{name}"


def _validate_safe_token(token: str, label: str) -> None:
    if any(term in token.lower() for term in FORBIDDEN_PATH_TERMS):
        raise fail(f"forbidden path term: {label}/{token}")


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
        assert_equal(pin.get("read_at_manifest_creation"), False, f"manifest/raw/{route}/{name}/read")
    planned = record.get("planned_output")
    if not isinstance(planned, dict):
        raise fail(f"planned output missing: {route}")
    for flag, name in (
        ("--out", "withheld_solution_output.csv"),
        ("--summary-json", "summary.json"),
    ):
        assert_equal(command.count(flag), 1, f"command/{route}/{flag}-count")
        assert_equal(command[command.index(flag) + 1], _output_path(route, name), f"command/{route}/{name}")


def verify_freeze() -> dict[str, Any]:
    assert_equal(sha256_file(FREEZE, "Phase95 path-availability freeze"), FREEZE_SHA, "freeze/sha256")
    freeze = read_json(FREEZE, "Phase95 path-availability freeze")
    assert_equal(freeze.get("schema_version"), "smartphone-r5-phase95-raw-input-path-availability-freeze.v1", "freeze/schema_version")
    assert_equal(freeze.get("phase"), 95, "freeze/phase")
    assert_equal(freeze.get("status"), "frozen-after-read-only-raw-input-path-audit", "freeze/status")
    assert_equal(freeze.get("execution_label"), "Luna Max", "freeze/execution_label")
    candidate = freeze.get("candidate")
    if not isinstance(candidate, dict):
        raise fail("freeze/candidate missing")
    assert_equal(candidate.get("exactly_one"), True, "freeze/candidate/exactly_one")
    assert_equal(candidate.get("id"), "phase95_phase94_wrapper_inherit_phase91_raw_paths_and_materialize_exact_route_inputs", "freeze/candidate/id")
    assert_equal(candidate.get("status"), "frozen-not-implemented", "freeze/candidate/status")
    assert_equal(candidate.get("default_off"), True, "freeze/candidate/default_off")
    assert_equal(candidate.get("raw_execution_authorized"), False, "freeze/candidate/raw_execution_authorized")
    assert_equal(candidate.get("native_algorithm_change"), False, "freeze/candidate/native_algorithm_change")
    assert_equal(candidate.get("solver_filter_lm_change"), False, "freeze/candidate/solver_filter_lm_change")
    boundary = freeze.get("execution_boundary")
    if not isinstance(boundary, dict):
        raise fail("freeze/execution_boundary missing")
    for key in ("this_freeze_is_pre_raw", "raw_execution_authorized", "raw_execution_performed", "accuracy_evaluation_authorized", "truth_evaluation_authorized", "submission_release_authorized"):
        assert_equal(boundary.get(key), False if key != "this_freeze_is_pre_raw" else True, f"freeze/execution_boundary/{key}")
    accounting = freeze.get("read_accounting_at_freeze")
    if not isinstance(accounting, dict):
        raise fail("freeze/read_accounting_at_freeze missing")
    for key in ("native_solver_invocations", "raw_device_gnss_reads", "raw_device_imu_reads", "broadcast_navigation_reads", "base_rinex_reads", "truth_reads", "mat_reads_or_generated", "precomputed_coordinate_reads", "validation_holdout_reads", "kaggle_or_token_access", "accuracy_calculations", "raw_input_hash_reads"):
        assert_equal(accounting.get(key), 0, f"freeze/read_accounting/{key}")
    return freeze


def verify_manifest() -> dict[str, Any]:
    verify_freeze()
    manifest = read_json(MANIFEST, "Phase95 execution manifest")
    assert_equal(manifest.get("schema_version"), SCHEMA, "manifest/schema_version")
    assert_equal(manifest.get("phase"), 95, "manifest/phase")
    assert_equal(manifest.get("status"), "sealed-before-phase95-corrected-raw-execution", "manifest/status")
    assert_equal(manifest.get("execution_label"), "Luna Max", "manifest/execution_label")
    freeze_ref = manifest.get("freeze")
    if not isinstance(freeze_ref, dict):
        raise fail("manifest/freeze missing")
    assert_equal(freeze_ref.get("path"), relative(FREEZE), "manifest/freeze/path")
    assert_equal(freeze_ref.get("sha256"), FREEZE_SHA, "manifest/freeze/sha256")
    implementation = manifest.get("implementation")
    if not isinstance(implementation, dict):
        raise fail("manifest/implementation missing")
    source = implementation.get("source")
    binary = implementation.get("binary")
    if not isinstance(source, dict) or not isinstance(binary, dict):
        raise fail("manifest/implementation pins malformed")
    assert_equal(source.get("path"), relative(APP), "manifest/source/path")
    assert_equal(source.get("sha256"), PHASE94_SOURCE_SHA, "manifest/source/sha256")
    assert_equal(sha256_file(APP, "native implementation"), PHASE94_SOURCE_SHA, "native implementation/sha256")
    assert_equal(binary.get("path"), BINARY_RELATIVE, "manifest/binary/path")
    assert_equal(binary.get("sha256"), BINARY_SHA, "manifest/binary/sha256")
    assert_equal(sha256_file(BINARY, "native binary"), BINARY_SHA, "native binary/sha256")
    path_correction = implementation.get("path_correction")
    if not isinstance(path_correction, dict) or not re.fullmatch(r"[0-9a-f]{40}", str(path_correction.get("commit", ""))):
        raise fail("manifest/path_correction commit pin malformed")
    for key, path in (("evaluator", EVALUATOR), ("wrapper", WRAPPER), ("focused_tests", FOCUSED_TESTS)):
        pin = manifest.get(key)
        if not isinstance(pin, dict):
            raise fail(f"manifest/{key} missing")
        assert_equal(pin.get("path"), relative(path), f"manifest/{key}/path")
        assert_equal(pin.get("sha256"), sha256_file(path, f"Phase95 {key}"), f"manifest/{key}/sha256")
    inherited = manifest.get("inherited_raw_manifest")
    if not isinstance(inherited, dict):
        raise fail("manifest/inherited_raw_manifest missing")
    assert_equal(inherited.get("path"), relative(PHASE91_MANIFEST), "manifest/inherited_raw_manifest/path")
    assert_equal(inherited.get("sha256"), PHASE91_MANIFEST_SHA, "manifest/inherited_raw_manifest/sha256")
    assert_equal(manifest.get("candidate", {}).get("id"), "phase95_phase94_wrapper_inherit_phase91_raw_paths_and_materialize_exact_route_inputs", "manifest/candidate/id")
    for key, expected in {
        "candidate_count": 1,
        "diagnostic_only": True,
        "default_off": True,
        "raw_only": True,
        "controls": 0,
        "runs_per_route": 1,
        "accuracy_scoring": False,
        "truth_evaluation": False,
        "submission_release": False,
        "no_solver_filter_or_lm_change": True,
        "no_fallback_or_guard_bypass": True,
    }.items():
        assert_equal(manifest.get("candidate", {}).get(key), expected, f"manifest/candidate/{key}")
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
        for name in RAW_NAMES:
            pin = record["raw_inputs"][name]
            assert_equal(pin.get("sha256_available"), False, f"manifest/raw/{route}/{name}/sha256_available")
    matrix = manifest.get("matrix")
    if not isinstance(matrix, dict):
        raise fail("manifest/matrix missing")
    for key, expected in {
        "candidate_count": 1,
        "routes": 4,
        "runs_per_route": 1,
        "native_invocations_planned": 4,
        "controls": 0,
        "reruns": 0,
        "fallbacks": 0,
        "raw_device_gnss_route_arguments": 4,
        "raw_device_imu_route_arguments": 4,
        "broadcast_navigation_route_arguments": 4,
        "base_rinex_reads_planned": 0,
        "truth_reads_planned": 0,
        "mat_reads_or_generated_planned": 0,
        "precomputed_coordinate_reads_planned": 0,
        "accuracy_calculations_planned": 0,
        "kaggle_or_token_access_planned": 0,
        "route_score_selection": False,
    }.items():
        assert_equal(matrix.get(key), expected, f"manifest/matrix/{key}")
    gates = manifest.get("structural_gates")
    if not isinstance(gates, dict):
        raise fail("manifest/structural_gates missing")
    for key in ("required_before_stop", "gnss_first_guard_predicate_and_counts", "gnss_first_progress_or_fail_closed_telemetry", "main_validation_predicates", "accepted_cost_lambda_terminal_telemetry", "finite_earth_valid_exact_handoff", "official_c0d_units_sigma_and_skip_telemetry", "no_solution_or_accuracy_publication", "command_policy_no_pdc_external_precomputed"):
        assert_equal(gates.get(key), True, f"manifest/structural_gates/{key}")
    qualification = manifest.get("qualification_evidence")
    if not isinstance(qualification, dict):
        raise fail("manifest/qualification_evidence missing")
    assert_equal(qualification.get("full_cpp_suite", {}).get("exit_code"), 0, "manifest/qualification/full_cpp/exit_code")
    assert_equal(qualification.get("focused_python", {}).get("exit_code"), 0, "manifest/qualification/focused_python/exit_code")
    assert_equal(qualification.get("diff_check_clean"), True, "manifest/qualification/diff_check_clean")
    execution = manifest.get("execution_authorization")
    if not isinstance(execution, dict):
        raise fail("manifest/execution_authorization missing")
    authority = manifest.get("authority") or {}
    auth_pin = authority.get("raw_execution_authorization")
    authorized = isinstance(auth_pin, dict)
    assert_equal(execution.get("raw_execution_authorized"), authorized, "manifest/execution_authorization/raw_execution_authorized")
    for key, expected in {
        "before_raw_execution": True,
        "native_route_rerun_performed": False,
        "accuracy_or_submission_release": False,
        "stop_after_four_routes": True,
    }.items():
        assert_equal(execution.get(key), expected, f"manifest/execution_authorization/{key}")
    accounting = manifest.get("read_accounting_before_execution")
    if not isinstance(accounting, dict):
        raise fail("manifest/read_accounting_before_execution missing")
    for key in ("native_solver_invocations", "raw_device_gnss_reads", "raw_device_imu_reads", "broadcast_navigation_reads", "truth_reads", "mat_reads_or_generated", "precomputed_coordinate_reads", "base_rinex_reads", "accuracy_calculations", "kaggle_or_token_access"):
        assert_equal(accounting.get(key), 0, f"manifest/read_accounting/{key}")
    return manifest


def verify_authorization(manifest: dict[str, Any]) -> dict[str, Any]:
    authorization = read_json(AUTHORIZATION, "Phase95 raw execution authorization")
    assert_equal(authorization.get("schema_version"), AUTH_SCHEMA, "authorization/schema_version")
    assert_equal(authorization.get("status"), "authorized-for-exact-four-route-phase95-path-corrected-diagnostic-execution", "authorization/status")
    assert_equal(authorization.get("execution_label"), "Luna Max", "authorization/execution_label")
    authority = authorization.get("authority")
    if not isinstance(authority, dict):
        raise fail("authorization/authority missing")
    for key, path, digest in (
        ("phase95_freeze", FREEZE, FREEZE_SHA),
        ("phase95_manifest", MANIFEST, sha256_file(MANIFEST, "Phase95 manifest")),
        ("phase95_evaluator", EVALUATOR, sha256_file(EVALUATOR, "Phase95 evaluator")),
        ("phase95_wrapper", WRAPPER, sha256_file(WRAPPER, "Phase95 wrapper")),
        ("phase95_focused_tests", FOCUSED_TESTS, sha256_file(FOCUSED_TESTS, "Phase95 focused tests")),
    ):
        pin = authority.get(key)
        if not isinstance(pin, dict):
            raise fail(f"authorization/{key} missing")
        assert_equal(pin.get("path"), relative(path), f"authorization/{key}/path")
        assert_equal(pin.get("sha256"), digest, f"authorization/{key}/sha256")
    assert_equal(authority.get("phase91_inherited_raw_manifest", {}).get("sha256"), PHASE91_MANIFEST_SHA, "authorization/phase91_manifest/sha256")
    assert_equal(authority.get("implementation_source_sha256"), PHASE94_SOURCE_SHA, "authorization/implementation_source_sha256")
    assert_equal(authority.get("binary_sha256"), BINARY_SHA, "authorization/binary_sha256")
    path_correction_commit = manifest.get("implementation", {}).get("path_correction", {}).get("commit")
    assert_equal(authority.get("path_correction_commit"), path_correction_commit, "authorization/path_correction_commit")
    candidate = authorization.get("candidate")
    if not isinstance(candidate, dict):
        raise fail("authorization/candidate missing")
    for key, expected in {
        "candidate_count": 1,
        "id": "phase95_phase94_wrapper_inherit_phase91_raw_paths_and_materialize_exact_route_inputs",
        "diagnostic_only": True,
        "raw_only": True,
        "default_off_outside_this_explicit_command": True,
        "runs_per_route": 1,
        "controls": 0,
        "raw_input_names_exact": list(RAW_NAMES),
        "truth_mat_coordinate_base_kaggle_accuracy": False,
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
        "routes": 4,
        "runs_per_route": 1,
        "native_invocations": 4,
        "controls": 0,
        "reruns": 0,
        "fallbacks": 0,
        "raw_device_gnss_reads_max": 4,
        "raw_device_imu_reads_max": 4,
        "broadcast_navigation_reads_max": 4,
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
        "commands_source": "exact route commands from the pinned Phase95 execution manifest after wrapper-only path materialization",
        "order": "sequential route order in manifest",
        "one_invocation_per_route": True,
        "no_rerun": True,
        "stop_after_four_routes": True,
        "on_route_failure": "record fail-closed diagnostic and continue each remaining route once; no fallback",
        "raw_input_hashes": "not requested by wrapper before or outside native route reads",
        "solver_filter_lm_changes": False,
        "fallback_or_guard_bypass": False,
    }.items():
        assert_equal(policy.get(key), expected, f"authorization/execution_policy/{key}")
    boundary = authorization.get("release_boundary")
    if not isinstance(boundary, dict):
        raise fail("authorization/release_boundary missing")
    for key, expected in {
        "raw_execution_authorized": True,
        "structural_result_authorized": True,
        "truth_or_accuracy_evaluation_authorized": False,
        "submission_release_authorized": False,
        "promotion_authorized": False,
    }.items():
        assert_equal(boundary.get(key), expected, f"authorization/release_boundary/{key}")
    accounting = authorization.get("read_accounting_before_execution")
    if not isinstance(accounting, dict):
        raise fail("authorization/read_accounting_before_execution missing")
    for key in ("native_solver_invocations", "raw_device_gnss_reads", "raw_device_imu_reads", "broadcast_navigation_reads", "base_rinex_reads", "truth_reads", "mat_reads_or_generated", "precomputed_coordinate_reads", "accuracy_calculations", "kaggle_or_token_access"):
        assert_equal(accounting.get(key), 0, f"authorization/read_accounting/{key}")
    assert_equal(manifest.get("execution_authorization", {}).get("raw_execution_authorized"), True, "manifest authorization state")
    return authorization


def verify_pre_raw() -> dict[str, Any]:
    freeze = verify_freeze()
    manifest = verify_manifest()
    execution = manifest.get("execution_authorization", {})
    authorization = None
    if execution.get("raw_execution_authorized") is True:
        authorization = verify_authorization(manifest)
    source = EVALUATOR.read_text(encoding="utf-8")
    for forbidden in ("sub" + "process", "os" + ".system", "P" + "open"):
        if forbidden in source:
            raise fail(f"pre-raw evaluator contains forbidden execution/read token: {forbidden}")
    return {
        "status": "pre-raw-verified",
        "phase": 95,
        "execution_label": "Luna Max",
        "candidate_count": 1,
        "routes": 4,
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
        "manifest_sha256": sha256_file(MANIFEST, "Phase95 execution manifest"),
        "authorization_sha256": sha256_file(AUTHORIZATION, "Phase95 authorization") if authorization is not None else None,
        "freeze_status": freeze.get("status"),
        "manifest_status": manifest.get("status"),
        "authorization_status": authorization.get("status") if authorization is not None else "not-issued-before-separate-raw-authorization",
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
    except (Phase95PreRawError, OSError) as exc:
        print(f"phase95 pre-raw verifier: fail-closed: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
