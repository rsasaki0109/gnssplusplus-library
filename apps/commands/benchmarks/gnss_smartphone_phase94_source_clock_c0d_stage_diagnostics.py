#!/usr/bin/env python3
"""Fail-closed pre-raw verifier for the sealed Phase94 diagnostic lane.

This verifier reads only source files and sealed records.  It has no process
launch path and never opens or hashes raw GNSS, IMU, navigation, truth, MAT,
coordinate, base, or Kaggle artifacts.
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
    "smartphone_r5_phase94_source_clock_c0d_stage_diagnostics_freeze_v1.json"
)
AUTHORIZATION = ROOT / (
    "docs/use_cases/records/"
    "smartphone_r5_phase94_source_clock_c0d_stage_diagnostics_raw_execution_authorization_v1.json"
)
MANIFEST = ROOT / (
    "docs/use_cases/records/"
    "smartphone_r5_phase94_source_clock_c0d_stage_diagnostics_execution_manifest_v1.json"
)
EVALUATOR = Path(__file__).resolve()
WRAPPER = ROOT / (
    "apps/commands/benchmarks/"
    "gnss_smartphone_phase94_source_clock_c0d_stage_diagnostics_execute.py"
)
FOCUSED_TESTS = ROOT / "tests/test_smartphone_phase94_source_clock_c0d_stage_diagnostics.py"
APP = ROOT / "apps/native/gnss_fgo_imu_no_base.cpp"
BINARY = ROOT / "build/apps/gnss_fgo_imu_no_base"

FREEZE_SHA = "9b0b61dee17be33b595ddccbf0edddaf7844b664de3defa8e1e7139e36f0b4b9"
FREEZE_COMMIT = "a6c18530938ff403e9e5b93e093ea98f39990248"
IMPLEMENTATION_COMMIT = "24f3329841f29d18b0b92c5476887464d8bf8be6"
IMPLEMENTATION_SOURCE_SHA = "524a8c0e4703a0ea533a21a4775e82719abaab2d2229a725ccf272cf2ab5430a"

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
RAW_ROOT = "raw/phase93/"
OUTPUT_ROOT = "output/smartphone-r5/phase94-source-clock-c0d-stage-diagnostics-v1/"
BINARY_RELATIVE = "build/apps/gnss_fgo_imu_no_base"
SELECTOR = "--native-source-clock-c0d-phase94-stage-diagnostics"
PHASE93_SELECTOR = "--native-source-clock-c0d-gnss-first-meter-state-handoff"
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


class Phase94PreRawError(ValueError):
    """Raised when the sealed Phase94 pre-raw contract fails."""


def fail(message: str) -> Phase94PreRawError:
    return Phase94PreRawError(message)


def relative(path: Path) -> str:
    try:
        return str(path.relative_to(ROOT))
    except ValueError:
        return str(path)


def sha256_file(path: Path, label: str) -> str:
    try:
        payload = path.read_bytes()
    except OSError as exc:
        raise fail(f"failed to read {label}: {path}: {exc}") from exc
    return hashlib.sha256(payload).hexdigest()


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


def verify_freeze() -> dict[str, Any]:
    if sha256_file(FREEZE, "Phase94 freeze") != FREEZE_SHA:
        raise fail("Phase94 freeze hash changed")
    freeze = read_json(FREEZE, "Phase94 freeze")
    assert_equal(freeze.get("schema_version"), "smartphone-r5-phase94-source-clock-c0d-stage-diagnostics-freeze.v1", "freeze/schema_version")
    assert_equal(freeze.get("phase"), 94, "freeze/phase")
    assert_equal(freeze.get("status"), "frozen-after-read-only-root-cause-audit", "freeze/status")
    assert_equal(freeze.get("execution_label"), "Luna Max", "freeze/execution_label")
    candidate = freeze.get("candidate", {})
    assert_equal(candidate.get("exactly_one"), True, "freeze/candidate/exactly_one")
    assert_equal(candidate.get("id"), "phase94_source_clock_c0d_stage_admission_and_failure_telemetry", "freeze/candidate/id")
    assert_equal(candidate.get("selector"), SELECTOR, "freeze/candidate/selector")
    assert_equal(candidate.get("selector_implemented"), False, "freeze/candidate/selector_implemented")
    assert_equal(candidate.get("default_off"), True, "freeze/candidate/default_off")
    assert_equal(candidate.get("raw_execution_authorized"), False, "freeze/candidate/raw_execution_authorized")
    assert_equal(candidate.get("accuracy_scored"), False, "freeze/candidate/accuracy_scored")
    assert_equal(candidate.get("promotion_authorized"), False, "freeze/candidate/promotion_authorized")
    boundary = freeze.get("execution_boundary", {})
    for key, expected in {
        "this_freeze_is_pre_raw": True,
        "raw_execution_authorized": False,
        "raw_execution_performed": False,
        "accuracy_evaluation_authorized": False,
        "submission_release_authorized": False,
    }.items():
        assert_equal(boundary.get(key), expected, f"freeze/execution_boundary/{key}")
    accounting = freeze.get("read_accounting_at_freeze", {})
    for key in (
        "native_solver_invocations",
        "raw_device_gnss_reads",
        "raw_device_imu_reads",
        "broadcast_navigation_reads",
        "base_rinex_reads",
        "truth_reads",
        "mat_reads_or_generated",
        "precomputed_coordinate_reads",
        "kaggle_or_token_access",
        "accuracy_calculations",
    ):
        assert_equal(accounting.get(key), 0, f"freeze/read_accounting/{key}")
    return freeze


def _validate_command(route: str, command: Any, raw_inputs: dict[str, Any]) -> None:
    if not isinstance(command, list) or not command or command[0] != BINARY_RELATIVE:
        raise fail(f"command binary changed: {route}")
    if any(not isinstance(token, str) for token in command):
        raise fail(f"command token type changed: {route}")
    for token in command:
        if any(term in token.lower() for term in FORBIDDEN_PATH_TERMS):
            raise fail(f"forbidden path term in command: {route}/{token}")
        if token in FORBIDDEN_FLAGS:
            raise fail(f"forbidden flag in command: {route}/{token}")
    for flag in REQUIRED_FLAGS:
        assert_equal(command.count(flag), 1, f"command/{route}/{flag}")
    assert_equal(command.count("--dataset-id"), 1, f"command/{route}/dataset-id-count")
    assert_equal(command[command.index("--dataset-id") + 1], route, f"command/{route}/dataset-id")
    for flag, name in (("--android-gnss", "device_gnss.csv"), ("--android-imu", "device_imu.csv"), ("--nav", "brdc.nav")):
        assert_equal(command.count(flag), 1, f"command/{route}/{flag}-count")
        assert_equal(command[command.index(flag) + 1], raw_inputs[name]["path"], f"command/{route}/{name}")
    for flag, name in (("--out", "withheld_output"), ("--summary-json", "summary")):
        assert_equal(command.count(flag), 1, f"command/{route}/{flag}-count")
        expected = raw_inputs.get("planned_output", {})
        # The output records are kept under the route directory and are not
        # interpreted as accuracy/submission artifacts by this lane.
        if flag == "--out":
            expected_path = _output_path(route, "withheld_solution_output.csv")
        else:
            expected_path = _output_path(route, "summary.json")
        assert_equal(command[command.index(flag) + 1], expected_path, f"command/{route}/{name}")


def verify_manifest() -> dict[str, Any]:
    freeze = verify_freeze()
    manifest = read_json(MANIFEST, "Phase94 execution manifest")
    assert_equal(manifest.get("schema_version"), "smartphone-r5-phase94-source-clock-c0d-stage-diagnostics-execution-manifest.v1", "manifest/schema_version")
    assert_equal(manifest.get("phase"), 94, "manifest/phase")
    assert_equal(manifest.get("status"), "sealed-before-phase94-raw-execution", "manifest/status")
    assert_equal(manifest.get("execution_label"), "Luna Max", "manifest/execution_label")
    freeze_ref = manifest.get("freeze", {})
    assert_equal(freeze_ref.get("path"), relative(FREEZE), "manifest/freeze/path")
    assert_equal(freeze_ref.get("sha256"), FREEZE_SHA, "manifest/freeze/sha256")
    assert_equal(manifest.get("implementation", {}).get("commit"), IMPLEMENTATION_COMMIT, "manifest/implementation/commit")
    assert_equal(manifest.get("implementation", {}).get("source_sha256"), IMPLEMENTATION_SOURCE_SHA, "manifest/implementation/source_sha256")
    binary = manifest.get("implementation", {}).get("binary", {})
    assert_equal(binary.get("path"), BINARY_RELATIVE, "manifest/implementation/binary/path")
    expected_binary = binary.get("sha256")
    if not isinstance(expected_binary, str) or len(expected_binary) != 64:
        raise fail("manifest binary hash missing")
    if sha256_file(BINARY, "Phase94 binary") != expected_binary:
        raise fail("Phase94 binary hash changed")
    for key, path in (("evaluator", EVALUATOR), ("wrapper", WRAPPER), ("focused_tests", FOCUSED_TESTS)):
        pin = manifest.get(key, {})
        assert_equal(pin.get("path"), relative(path), f"manifest/{key}/path")
        assert_equal(pin.get("sha256"), sha256_file(path, f"Phase94 {key}"), f"manifest/{key}/sha256")
    authority = manifest.get("authority", {})
    for key, path, digest in (
        ("phase94_freeze", FREEZE, FREEZE_SHA),
        ("implementation", APP, IMPLEMENTATION_SOURCE_SHA),
    ):
        item = authority.get(key, {})
        assert_equal(item.get("path"), relative(path), f"manifest/authority/{key}/path")
        assert_equal(item.get("sha256"), digest, f"manifest/authority/{key}/sha256")
    authorization_pin = authority.get("raw_execution_authorization")
    if authorization_pin is not None:
        if not isinstance(authorization_pin, dict):
            raise fail("manifest/authority/raw_execution_authorization is malformed")
        assert_equal(
            authorization_pin.get("path"),
            relative(AUTHORIZATION),
            "manifest/authority/raw_execution_authorization/path",
        )
        assert_equal(
            authorization_pin.get("sha256"),
            sha256_file(AUTHORIZATION, "Phase94 raw authorization"),
            "manifest/authority/raw_execution_authorization/sha256",
        )
    candidate = manifest.get("candidate", {})
    for key, expected in {
        "id": "phase94_source_clock_c0d_stage_admission_and_failure_telemetry",
        "selector": SELECTOR,
        "phase93_selector": PHASE93_SELECTOR,
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
        assert_equal(candidate.get(key), expected, f"manifest/candidate/{key}")
    forbidden_lanes = candidate.get("forbidden_inputs_and_lanes", [])
    for term in ("truth", "MAT", "precomputed coordinates", "base", "Kaggle/token", "accuracy"):
        if term not in forbidden_lanes:
            raise fail(f"manifest/candidate forbidden lane missing: {term}")
    routes = manifest.get("routes")
    if not isinstance(routes, list) or [item.get("dataset_id") for item in routes] != list(ROUTES):
        raise fail("manifest route order/count changed")
    for item in routes:
        route = item.get("dataset_id")
        assert_equal(item.get("runs"), 1, f"manifest/route/{route}/runs")
        assert_equal(item.get("domain_rows"), DOMAIN_ROWS[route], f"manifest/route/{route}/domain_rows")
        assert_equal(item.get("diagnostic_only"), True, f"manifest/route/{route}/diagnostic_only")
        raw = item.get("raw_inputs")
        if not isinstance(raw, dict) or tuple(raw) != RAW_NAMES:
            raise fail(f"manifest raw input set changed: {route}")
        for name in RAW_NAMES:
            pin = raw[name]
            assert_equal(pin.get("path"), _raw_path(route, name), f"manifest/raw/{route}/{name}/path")
            assert_equal(pin.get("sha256"), None, f"manifest/raw/{route}/{name}/sha256")
            assert_equal(pin.get("sha256_available"), False, f"manifest/raw/{route}/{name}/sha256_available")
            assert_equal(pin.get("read_at_manifest_creation"), False, f"manifest/raw/{route}/{name}/read")
            if any(term in pin["path"].lower() for term in FORBIDDEN_PATH_TERMS):
                raise fail(f"forbidden raw path term: {route}/{name}")
        planned = item.get("planned_output", {})
        assert_equal(planned.get("summary"), _output_path(route, "summary.json"), f"manifest/output/{route}/summary")
        assert_equal(planned.get("withheld_solution_output"), _output_path(route, "withheld_solution_output.csv"), f"manifest/output/{route}/withheld")
        _validate_command(route, item.get("command"), raw | {"planned_output": planned})
    matrix = manifest.get("matrix", {})
    for key, expected in {
        "candidate_count": 1,
        "routes": 4,
        "runs_per_route": 1,
        "native_invocations_planned": 4,
        "controls": 0,
        "reruns": 0,
        "fallbacks": 0,
        "raw_device_gnss_reads_planned": 4,
        "raw_device_imu_reads_planned": 4,
        "broadcast_navigation_reads_planned": 4,
        "base_rinex_reads_planned": 0,
        "truth_reads_planned": 0,
        "mat_reads_or_generated_planned": 0,
        "precomputed_coordinate_reads_planned": 0,
        "accuracy_calculations_planned": 0,
        "kaggle_or_token_access_planned": 0,
        "route_score_selection": False,
    }.items():
        assert_equal(matrix.get(key), expected, f"manifest/matrix/{key}")
    gates = manifest.get("structural_gates", {})
    assert_equal(gates.get("required_before_stop"), True, "manifest/structural_gates/required_before_stop")
    for key in (
        "gnss_first_guard_predicate_and_counts",
        "gnss_first_progress_or_fail_closed_telemetry",
        "main_validation_predicates",
        "accepted_cost_lambda_terminal_telemetry",
        "finite_earth_valid_and_no_solution_publication",
    ):
        assert_equal(gates.get(key), True, f"manifest/structural_gates/{key}")
    qualification = manifest.get("qualification_evidence", {})
    assert_equal(qualification.get("full_cpp_suite", {}).get("exit_code"), 0, "manifest/qualification/full_cpp/exit_code")
    assert_equal(qualification.get("full_cpp_suite", {}).get("failed"), 0, "manifest/qualification/full_cpp/failed")
    assert_equal(qualification.get("focused_python", {}).get("passed"), 6, "manifest/qualification/focused_python/passed")
    assert_equal(qualification.get("diff_check_clean"), True, "manifest/qualification/diff_check_clean")
    execution_authorization = manifest.get("execution_authorization", {})
    auth_is_pinned = "raw_execution_authorization" in authority
    assert_equal(
        execution_authorization.get("raw_execution_authorized"),
        auth_is_pinned,
        "manifest/execution_authorization/raw_execution_authorized",
    )
    for key, expected in {
        "before_raw_execution": True,
        "native_route_rerun_performed": False,
        "accuracy_or_submission_release": False,
        "stop_after_four_routes": True,
    }.items():
        assert_equal(
            execution_authorization.get(key),
            expected,
            f"manifest/execution_authorization/{key}",
        )
    accounting = manifest.get("read_accounting_before_execution", {})
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
    ):
        assert_equal(accounting.get(key), 0, f"manifest/read_accounting/{key}")
    return manifest


def verify_authorization(manifest: dict[str, Any]) -> dict[str, Any]:
    authorization = read_json(AUTHORIZATION, "Phase94 raw authorization")
    assert_equal(authorization.get("schema_version"), "smartphone-r5-phase94-source-clock-c0d-stage-diagnostics-raw-execution-authorization.v1", "authorization/schema_version")
    assert_equal(authorization.get("status"), "authorized-for-exact-four-route-phase94-diagnostic-execution", "authorization/status")
    assert_equal(authorization.get("execution_label"), "Luna Max", "authorization/execution_label")
    authority = authorization.get("authority", {})
    assert_equal(authority.get("phase94_freeze", {}).get("sha256"), FREEZE_SHA, "authorization/freeze/sha256")
    assert_equal(authority.get("implementation_commit"), IMPLEMENTATION_COMMIT, "authorization/implementation_commit")
    assert_equal(authority.get("implementation_source_sha256"), IMPLEMENTATION_SOURCE_SHA, "authorization/implementation/source_sha256")
    assert_equal(authority.get("evaluator_sha256"), sha256_file(EVALUATOR, "Phase94 evaluator"), "authorization/evaluator/sha256")
    assert_equal(authority.get("wrapper_sha256"), sha256_file(WRAPPER, "Phase94 wrapper"), "authorization/wrapper/sha256")
    assert_equal(authority.get("focused_tests_sha256"), sha256_file(FOCUSED_TESTS, "Phase94 focused tests"), "authorization/focused_tests/sha256")
    candidate = authorization.get("candidate", {})
    for key, expected in {
        "candidate_count": 1,
        "id": "phase94_source_clock_c0d_stage_admission_and_failure_telemetry",
        "selector": SELECTOR,
        "phase93_selector": PHASE93_SELECTOR,
        "diagnostic_only": True,
        "runs_per_route": 1,
        "controls": 0,
        "raw_input_names_exact": list(RAW_NAMES),
        "truth_mat_coordinate_base_kaggle_accuracy": False,
    }.items():
        assert_equal(candidate.get(key), expected, f"authorization/candidate/{key}")
    routes = authorization.get("routes")
    if not isinstance(routes, list) or [item.get("dataset_id") for item in routes] != list(ROUTES):
        raise fail("authorization route order/count changed")
    for item in routes:
        assert_equal(item.get("runs"), 1, f"authorization/route/{item.get('dataset_id')}/runs")
        assert_equal(item.get("domain_rows"), DOMAIN_ROWS[item.get("dataset_id")], f"authorization/route/{item.get('dataset_id')}/rows")
    matrix = authorization.get("matrix", {})
    for key, expected in {
        "candidate_count": 1,
        "route_count": 4,
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
    boundary = authorization.get("release_boundary", {})
    for key, expected in {
        "raw_execution_authorized": True,
        "structural_result_authorized": True,
        "truth_or_accuracy_evaluation_authorized": False,
        "submission_release_authorized": False,
        "promotion_authorized": False,
    }.items():
        assert_equal(boundary.get(key), expected, f"authorization/release_boundary/{key}")
    assert_equal(manifest.get("status"), "sealed-before-phase94-raw-execution", "authorization/manifest/status")
    return authorization


def verify_pre_raw() -> dict[str, Any]:
    freeze = verify_freeze()
    manifest = verify_manifest()
    execution = manifest.get("execution_authorization", {})
    authorization = None
    if execution.get("raw_execution_authorized") is True:
        authorization = verify_authorization(manifest)
    source = EVALUATOR.read_text(encoding="utf-8")
    # Keep this source check launch-free without spelling an importable process
    # module name in the verifier itself.
    forbidden_tokens = ("P" + "open(", "os." + "system", "r" + "un(", "raw_input." + "open")
    for forbidden in forbidden_tokens:
        if forbidden in source:
            raise fail(f"pre-raw verifier contains forbidden launch/read token: {forbidden}")
    return {
        "status": "pre-raw-verified",
        "phase": 94,
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
        "manifest_sha256": sha256_file(MANIFEST, "Phase94 execution manifest"),
        "authorization_sha256": (
            sha256_file(AUTHORIZATION, "Phase94 authorization")
            if authorization is not None
            else None
        ),
        "implementation_commit": IMPLEMENTATION_COMMIT,
        "freeze_status": freeze["status"],
        "manifest_status": manifest["status"],
        "authorization_status": (
            authorization["status"]
            if authorization is not None
            else "not-issued-before-separate-raw-authorization"
        ),
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
    except (Phase94PreRawError, OSError) as exc:
        print(f"phase94 pre-raw verifier: fail-closed: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
