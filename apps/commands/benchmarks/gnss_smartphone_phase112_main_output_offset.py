#!/usr/bin/env python3
"""Launch-free Phase112 raw/base structural contract.

The verifier reads the Phase112 freeze, source pins, and sealed Phase95/65
path metadata.  It deliberately does not stat, hash, or open any raw phone
member or base RINEX member.  A separately committed authorization is needed
before the execution wrapper may materialize those inputs.
"""

from __future__ import annotations

import hashlib
import importlib.util
import json
from pathlib import Path
import sys
from typing import Any


ROOT = Path(__file__).resolve().parents[3]
FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_phase112_main_fgo_source_parity_freeze_v1.json"
MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase112_main_output_offset_manifest_v1.json"
AUTHORIZATION = ROOT / "docs/use_cases/records/smartphone_r5_phase112_main_output_offset_raw_authorization_v1.json"
EVALUATOR = Path(__file__).resolve()
WRAPPER = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase112_main_output_offset_execute.py"
FOCUSED_TESTS = ROOT / "tests/test_smartphone_phase112_main_output_offset_execution.py"
PHASE95_RESULT = ROOT / "docs/use_cases/records/smartphone_r5_phase95_raw_input_path_corrected_structural_result_v1.json"
PHASE65_MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase65_native_base_pseudorange_compensation_manifest_v1.json"
APP = ROOT / "apps/native/gnss_fgo_imu_no_base.cpp"
HELPER = ROOT / "include/libgnss++/algorithms/upstream_position_offset.hpp"
OFFICIAL_OFFSET = ROOT / "output/reproducibility-cache/gsdc2023/functions/add_position_offset.m"
BINARY = ROOT / "build/apps/gnss_fgo_imu_no_base"

FREEZE_SHA256 = "8917b194d46f92943a3fbde7681d961594488fcd3bed0e44ec08a3ba92ae16aa"
FREEZE_COMMIT = "186a001fa97604da6bb9fd09317581baca77d815"
AUDIT_COMMIT = "065f5d83c4deb93df172c5c17566af795a7c86b4"
IMPLEMENTATION_COMMIT = "b9a967d4b88307971d7fe772a3e3439c07fab8bb"
PHASE95_RESULT_SHA256 = "beba25d4b0d910064d76bea7aa6b7a884937ca01c3b00cf0f7f9f4b6267ea281"
PHASE65_MANIFEST_SHA256 = "1306480f6b7fb839e55309e9c2c77b41f2ac0a1baf9f6971949612fd485d2255"
HELPER_SHA256 = "a37fc5d93f2f32cf9e9c6d684b38d9ce855db3acaca743d765d870530ebb2fce"
OFFICIAL_OFFSET_SHA256 = "2308b612b6aab9816ddcf3c1efef621b8ebe15a1bdae5d167df2783e6c831797"
SOURCE_SHA256 = {
    "app": "268bd030496bb677a42d3400e20ec37bc8e36589827fb28fbf57e3e398a1b1be",
    "helper": HELPER_SHA256,
    "binary": "95a93efe37f4f043caf9920f324ea128c1872d0f161e0562da724d47c1483fb4",
}

ROUTES = (
    "2021-03-16-18-59-us-ca-mtv-a/pixel5",
    "2022-04-01-18-22-us-ca-lax-t/pixel5",
)
DOMAIN_ROWS = {ROUTES[0]: 2158, ROUTES[1]: 1465}
PROBLEM_EPOCHS = {route: rows + 1 for route, rows in DOMAIN_ROWS.items()}
RAW_NAMES = ("device_gnss.csv", "device_imu.csv", "brdc.nav")
RAW_FLAGS = (
    ("--android-gnss", "device_gnss.csv"),
    ("--android-imu", "device_imu.csv"),
    ("--nav", "brdc.nav"),
)
PHASE93_SELECTOR = "--native-source-clock-c0d-gnss-first-meter-state-handoff"
VECTOR_SELECTOR = "--native-source-clock-c0d-epoch-vector-parity"
QR_SELECTOR = "--native-source-clock-c0d-phase99-main-multifrontal-qr-solver"
BASE_SELECTORS = (
    "--native-base-pseudorange-compensation",
    "--native-base-pseudorange-source-miss-mask",
    "--native-base-pseudorange-preserve-additional-frequency-bands",
)
OFFSET_SELECTOR = "--native-upstream-position-offset"
BASE_FLAG = ("--native-base-rinex", "__PHASE112_RAW_BASE_RINEX__")
BASE_SHA_FLAG = ("--native-base-rinex-sha256", "__PHASE112_RAW_BASE_SHA256__")
CANDIDATE_ID = "phase112-main-output-upstream-position-offset-v1"
OUTPUT_ROOT = "output/smartphone-r5/phase112-main-output-offset-v1/"
SCHEMA = "smartphone-r5-phase112-main-output-offset-manifest.v1"
AUTH_SCHEMA = "smartphone-r5-phase112-main-output-offset-raw-authorization.v1"

REQUIRED_FLAGS = (
    "--all-epochs", "--android-raw-utc-keys", "--android-raw-clock-only",
    "--android-utc-wall-clock-fallback", "--native-pdc-imu-tdcp-no-bridge",
    "--native-source-direct-observable-quality", "--native-source-clock-c0d-factor",
    "--native-source-clock-c0d-meter-state-parity",
    "--native-source-clock-c0d-active-solve-diagnostic", PHASE93_SELECTOR,
    VECTOR_SELECTOR, QR_SELECTOR, *BASE_SELECTORS, OFFSET_SELECTOR,
)
FORBIDDEN_FLAGS = (
    "--obs", "--native-gnss-first-velocity-only-handoff",
    "--native-direct-doppler-wls-handoff", "--native-pdc-state-bridge",
    "--native-upstream-quality",
    "--native-source-clock-c0d-gnss-first-raw-drift-d-initializer",
    "--native-source-clock-c0d-phase94-stage-diagnostics",
    "--native-source-clock-c0d-phase96-main-diagnostics",
    "--native-source-clock-c0d-phase97-singular-system-diagnostics",
    "--native-source-clock-c0d-phase98-solver-rank-diagnostic",
    "--native-phase104-stage-main-attribution", "--native-quality-anchor",
)
FORBIDDEN_PATH_TERMS = (
    ".mat", "truth", "ground_truth", "validation", "holdout", "precomputed",
    "coordinate", "pdc", "kaggle", "token",
)


class Phase112ContractError(ValueError):
    """Raised when the pinned Phase112 contract fails closed."""


def fail(message: str) -> Phase112ContractError:
    return Phase112ContractError(message)


def relative(path: Path) -> str:
    try:
        return str(path.relative_to(ROOT))
    except ValueError:
        return str(path)


def assert_equal(actual: Any, expected: Any, label: str) -> None:
    if actual != expected:
        raise fail(f"{label}: expected {expected!r}, got {actual!r}")


def sha256_file(path: Path, label: str) -> str:
    # This verifier must never hash an input payload member.
    if path.name in RAW_NAMES or path.name == "base.obs":
        raise fail(f"input-member hash forbidden before authorization: {label}")
    if not path.is_file():
        raise fail(f"missing {label}: {path}")
    digest = hashlib.sha256()
    try:
        with path.open("rb") as handle:
            for chunk in iter(lambda: handle.read(1024 * 1024), b""):
                digest.update(chunk)
    except OSError as exc:
        raise fail(f"failed to hash {label}: {exc}") from exc
    return digest.hexdigest()


def read_json(path: Path, label: str) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise fail(f"failed to read {label}: {path}: {exc}") from exc
    if not isinstance(value, dict):
        raise fail(f"{label} is not an object")
    return value


def _phase109_metadata_module() -> Any:
    path = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase109_raw_base_frequency_parity.py"
    spec = importlib.util.spec_from_file_location("phase109_sealed_metadata", path)
    if spec is None or spec.loader is None:
        raise fail("unable to load sealed Phase109 metadata helpers")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def sealed_input_metadata() -> tuple[dict[str, dict[str, dict[str, Any]]], dict[str, dict[str, Any]]]:
    # These helpers read only sealed JSON metadata.  They do not touch the
    # member paths they return.
    module = _phase109_metadata_module()
    return module.phase95_raw_metadata(), module.phase65_base_metadata()


def safe_relative_member(path_text: Any, basename: str, label: str) -> None:
    if not isinstance(path_text, str):
        raise fail(f"{label} path missing")
    path = Path(path_text)
    if path.is_absolute() or ".." in path.parts or path.name != basename:
        raise fail(f"unsafe {label} path: {path_text}")


def verify_freeze() -> dict[str, Any]:
    assert_equal(sha256_file(FREEZE, "Phase112 freeze"), FREEZE_SHA256, "freeze/sha256")
    freeze = read_json(FREEZE, "Phase112 freeze")
    for key, expected in {
        "schema_version": "smartphone-r5-phase112-main-fgo-source-parity-freeze.v1",
        "phase": 112,
        "status": "frozen-source-backed-default-off-candidate-no-execution",
        "execution_label": "Luna Max",
    }.items():
        assert_equal(freeze.get(key), expected, f"freeze/{key}")
    candidate = freeze.get("candidate")
    if not isinstance(candidate, dict):
        raise fail("freeze/candidate missing")
    for key, expected in {
        "candidate_count": 1,
        "id": CANDIDATE_ID,
        "default_off": True,
        "raw_only": True,
        "selector": OFFSET_SELECTOR,
    }.items():
        assert_equal(candidate.get(key), expected, f"freeze/candidate/{key}")
    offset = candidate.get("official_pixel5_offset")
    if not isinstance(offset, dict):
        raise fail("freeze/official Pixel5 offset missing")
    for key, expected in {
        "offset_rl_m": -0.1,
        "offset_ud_m": -0.3,
        "local_vector_order": "[offsetUD, offsetRL, 0]",
        "rotation_preserving_norm_m": 0.31622776601683794,
    }.items():
        assert_equal(offset.get(key), expected, f"freeze/offset/{key}")
    boundary = freeze.get("authorization_boundary")
    if not isinstance(boundary, dict):
        raise fail("freeze/authorization_boundary missing")
    for key in (
        "implementation_authorized", "structural_raw_execution_authorized",
        "truth_evaluation_authorized", "accuracy_promotion_authorized",
        "solution_publication_authorized",
    ):
        assert_equal(boundary.get(key), False, f"freeze/boundary/{key}")
    pins = freeze.get("source_pins")
    if not isinstance(pins, dict):
        raise fail("freeze/source_pins missing")
    helper_pin = pins.get("native_existing_offset_helper")
    if not isinstance(helper_pin, dict):
        raise fail("freeze/native_existing_offset_helper missing")
    assert_equal(helper_pin.get("sha256"), HELPER_SHA256, "freeze/helper sha256")
    return freeze


def verify_implementation() -> None:
    for key, path in {"app": APP, "helper": HELPER, "binary": BINARY}.items():
        assert_equal(sha256_file(path, f"implementation/{key}"), SOURCE_SHA256[key], f"implementation/{key}/sha256")
    assert_equal(sha256_file(OFFICIAL_OFFSET, "official add_position_offset.m"), OFFICIAL_OFFSET_SHA256, "official offset/sha256")
    source = APP.read_text(encoding="utf-8")
    for marker in (
        "phase112MainOutputPositionOffsetSelectors",
        "native_upstream_position_offset",
        "position offset application attempted more than once",
        "position_offset_application.claim()",
        "offsetFromRpy(",
        'position_offset_report.phone != "pixel5"',
    ):
        if marker not in source:
            raise fail(f"Phase112 implementation marker missing: {marker}")
    if source.count("offsetFromRpy(") != 1 or source.count("position_offset_application.claim()") != 1:
        raise fail("Phase112 offset application is not a single source site")


def output_path(route: str, name: str) -> str:
    return f"{OUTPUT_ROOT}{route.replace('/', '__')}/{name}"


def command_template(route: str) -> list[str]:
    return [
        "build/apps/gnss_fgo_imu_no_base", "--dataset-id", route,
        "--android-gnss", "__PHASE112_RAW_DEVICE_GNSS__",
        "--android-imu", "__PHASE112_RAW_DEVICE_IMU__",
        "--nav", "__PHASE112_RAW_BROADCAST_NAV__", "--all-epochs",
        "--android-raw-utc-keys", "--android-raw-clock-only",
        "--android-utc-wall-clock-fallback", "--native-pdc-imu-tdcp-no-bridge",
        "--native-source-direct-observable-quality", "--native-source-clock-c0d-factor",
        "--native-source-clock-c0d-meter-state-parity",
        "--native-source-clock-c0d-active-solve-diagnostic", PHASE93_SELECTOR,
        VECTOR_SELECTOR, QR_SELECTOR, *BASE_SELECTORS, OFFSET_SELECTOR,
        BASE_FLAG[0], BASE_FLAG[1], BASE_SHA_FLAG[0], BASE_SHA_FLAG[1],
        "--out", output_path(route, "withheld_solution_output.csv"),
        "--summary-json", output_path(route, "summary.json"),
    ]


def validate_command(route: str, command: Any, record: dict[str, Any], raw: dict[str, Any], base: dict[str, Any]) -> None:
    assert_equal(command, command_template(route), f"manifest/{route}/exact command")
    for token in command:
        if token in FORBIDDEN_FLAGS or (
            not token.startswith("--") and any(term in token.lower() for term in FORBIDDEN_PATH_TERMS)
        ):
            raise fail(f"forbidden command token: {route}/{token}")
    for flag in REQUIRED_FLAGS:
        assert_equal(command.count(flag), 1, f"command/{route}/{flag}")
    assert_equal(command.count(BASE_FLAG[0]), 1, f"command/{route}/base path count")
    assert_equal(command[command.index(BASE_FLAG[0]) + 1], BASE_FLAG[1], f"command/{route}/base path placeholder")
    assert_equal(command.count(BASE_SHA_FLAG[0]), 1, f"command/{route}/base sha count")
    assert_equal(command[command.index(BASE_SHA_FLAG[0]) + 1], BASE_SHA_FLAG[1], f"command/{route}/base sha placeholder")
    for flag, name in RAW_FLAGS:
        assert_equal(command.count(flag), 1, f"command/{route}/{flag}")
        placeholder = {
            "device_gnss.csv": "__PHASE112_RAW_DEVICE_GNSS__",
            "device_imu.csv": "__PHASE112_RAW_DEVICE_IMU__",
            "brdc.nav": "__PHASE112_RAW_BROADCAST_NAV__",
        }[name]
        assert_equal(command[command.index(flag) + 1], placeholder, f"command/{route}/{name}/placeholder")
        pin = record.get("raw_inputs", {}).get(name)
        if not isinstance(pin, dict):
            raise fail(f"manifest raw pin missing: {route}/{name}")
        assert_equal(pin.get("path"), raw[name]["path"], f"manifest/raw/{route}/{name}/path")
        assert_equal(pin.get("sha256"), raw[name]["sha256"], f"manifest/raw/{route}/{name}/sha256")
        assert_equal(pin.get("read_at_manifest_creation"), False, f"manifest/raw/{route}/{name}/read")
    pin = record.get("base_input")
    if not isinstance(pin, dict):
        raise fail(f"manifest base pin missing: {route}")
    for key in ("path", "sha256", "bytes", "observed_dt_s", "moving_mean_samples", "approx_position_xyz_m"):
        assert_equal(pin.get(key), base.get(key), f"manifest/base/{route}/{key}")
    for key in ("read_at_manifest_creation", "hash_read_at_manifest_creation"):
        assert_equal(pin.get(key), False, f"manifest/base/{route}/{key}")
    assert_equal(record.get("runs"), 1, f"manifest/{route}/runs")


def verify_manifest() -> dict[str, Any]:
    verify_freeze()
    verify_implementation()
    raw, base = sealed_input_metadata()
    manifest = read_json(MANIFEST, "Phase112 raw structural manifest")
    for key, expected in {
        "schema_version": SCHEMA,
        "phase": 112,
        "execution_label": "Luna Max",
        "status": "sealed-before-phase112-raw-execution",
    }.items():
        assert_equal(manifest.get(key), expected, f"manifest/{key}")
    freeze = manifest.get("freeze")
    if not isinstance(freeze, dict):
        raise fail("manifest/freeze missing")
    for key, expected in {
        "path": relative(FREEZE), "sha256": FREEZE_SHA256,
        "commit": FREEZE_COMMIT, "audit_commit": AUDIT_COMMIT,
        "raw_execution_authorized_before_manifest": False,
    }.items():
        assert_equal(freeze.get(key), expected, f"manifest/freeze/{key}")
    implementation = manifest.get("implementation")
    if not isinstance(implementation, dict):
        raise fail("manifest/implementation missing")
    for key, expected in {
        "commit": IMPLEMENTATION_COMMIT, "candidate_id": CANDIDATE_ID,
        "legacy_default_unchanged": True,
        "solver_filter_lm_equation_unit_sigma_changed": False,
        "offset_application_sites": 1,
    }.items():
        assert_equal(implementation.get(key), expected, f"manifest/implementation/{key}")
    assert_equal(implementation.get("source_sha256"), SOURCE_SHA256, "manifest/source hashes")
    for key, path in (("evaluator", EVALUATOR), ("wrapper", WRAPPER), ("focused_tests", FOCUSED_TESTS)):
        pin = manifest.get(key)
        if not isinstance(pin, dict):
            raise fail(f"manifest/{key} missing")
        assert_equal(pin.get("path"), relative(path), f"manifest/{key}/path")
        expected_hash = pin.get("sha256")
        if not isinstance(expected_hash, str) or len(expected_hash) != 64:
            raise fail(f"manifest/{key}/sha256 missing")
        assert_equal(sha256_file(path, f"manifest/{key}"), expected_hash, f"manifest/{key}/sha256")
    candidate = manifest.get("candidate")
    if not isinstance(candidate, dict):
        raise fail("manifest/candidate missing")
    for key, expected in {
        "id": CANDIDATE_ID, "candidate_count": 1, "opt_in": True,
        "default_off_outside_exact_recipe": True, "selector": OFFSET_SELECTOR,
        "selectors": [PHASE93_SELECTOR, VECTOR_SELECTOR, QR_SELECTOR],
        "base_selectors": list(BASE_SELECTORS), "pixel5_only": True,
        "official_offset_norm_m": 0.31622776601683794, "runs_per_route": 1,
        "controls": 0, "reruns": 0, "fallbacks": 0, "truth_evaluation": False,
        "accuracy_scoring": False, "solution_output_publication": False,
        "base_correction_applied_exactly_once": True,
        "offset_applied_exactly_once_final_output_only": True,
        "equation_unit_sigma_filter_lm_c7_d_qr_changed": False,
    }.items():
        assert_equal(candidate.get(key), expected, f"manifest/candidate/{key}")
    routes = manifest.get("routes")
    if not isinstance(routes, list) or [item.get("dataset_id") for item in routes] != list(ROUTES):
        raise fail("manifest route order/count changed")
    for record in routes:
        route = record.get("dataset_id")
        if route not in DOMAIN_ROWS:
            raise fail(f"unknown route: {route}")
        assert_equal(record.get("domain_rows"), DOMAIN_ROWS[route], f"manifest/{route}/domain rows")
        assert_equal(record.get("expected_problem_epochs"), PROBLEM_EPOCHS[route], f"manifest/{route}/problem epochs")
        assert_equal(record.get("expected_output_epochs"), PROBLEM_EPOCHS[route], f"manifest/{route}/output epochs")
        validate_command(route, record.get("command"), record, raw[route], base[route])
    raw_contract = manifest.get("raw_input_contract")
    if not isinstance(raw_contract, dict):
        raise fail("manifest/raw_input_contract missing")
    for key, expected in {
        "input_names_exact": list(RAW_NAMES), "phone_gnss_only": True,
        "phone_imu_only": True, "broadcast_navigation_only": True,
        "sealed_raw_base_rinex_only": True, "content_copy_or_transform": False,
        "truth_mat_precomputed_coordinate_pdc_kaggle_accuracy": False,
    }.items():
        assert_equal(raw_contract.get(key), expected, f"manifest/raw_input_contract/{key}")
    gates = manifest.get("structural_gates")
    if not isinstance(gates, dict):
        raise fail("manifest/structural_gates missing")
    for key in (
        "base_path_hash_bytes_header_match", "base_factors_active_exactly_once",
        "gnss_first_progress_strict_cost_decrease", "gnss_first_full_finite_c7_d_exact_handoff",
        "main_qr_progress_strict_cost_decrease", "offset_pixel5_exactly_once_final_output",
        "finite_earth_valid_expected_output_coverage", "no_solver_fallback_or_solution_publication",
    ):
        assert_equal(gates.get(key), True, f"manifest/structural_gates/{key}")
    matrix = manifest.get("matrix")
    if not isinstance(matrix, dict):
        raise fail("manifest/matrix missing")
    for key, expected in {
        "candidate_count": 1, "route_count": 2, "runs_per_route": 1,
        "native_solver_invocations_planned": 2, "controls": 0, "reruns": 0,
        "fallbacks": 0, "truth_reads_planned": 0, "mat_reads_or_generated_planned": 0,
        "phone_coordinate_reads_planned": 0, "precomputed_coordinate_reads_planned": 0,
        "pdc_reads_planned": 0, "accuracy_calculations_planned": 0,
        "solution_rows_authorized": False,
    }.items():
        assert_equal(matrix.get(key), expected, f"manifest/matrix/{key}")
    accounting = manifest.get("read_accounting_before_authorization")
    if not isinstance(accounting, dict):
        raise fail("manifest/read_accounting_before_authorization missing")
    for key in (
        "raw_phone_gnss_reads", "raw_phone_imu_reads", "broadcast_navigation_reads",
        "raw_base_rinex_reads", "raw_base_hash_reads", "native_solver_invocations",
        "truth_reads", "mat_reads_or_generated", "phone_coordinate_reads",
        "precomputed_coordinate_reads", "pdc_reads", "accuracy_calculations",
        "kaggle_or_token_access", "route_reruns", "fallbacks",
    ):
        assert_equal(accounting.get(key), 0, f"manifest/read_accounting/{key}")
    assert_equal(accounting.get("raw_content_copied_or_transformed"), False, "manifest/read_accounting/copy")
    return manifest


def verify_authorization(manifest: dict[str, Any] | None = None) -> dict[str, Any]:
    if manifest is None:
        manifest = verify_manifest()
    auth = read_json(AUTHORIZATION, "Phase112 raw authorization")
    for key, expected in {
        "schema_version": AUTH_SCHEMA, "phase": 112, "execution_label": "Luna Max",
        "status": "authorized-for-exact-two-route-phase112-raw-structural-execution",
    }.items():
        assert_equal(auth.get(key), expected, f"authorization/{key}")
    authority = auth.get("authority")
    if not isinstance(authority, dict):
        raise fail("authorization/authority missing")
    for key, path, expected in (
        ("phase112_freeze", FREEZE, FREEZE_SHA256),
        ("phase112_manifest", MANIFEST, sha256_file(MANIFEST, "Phase112 manifest")),
        ("phase112_evaluator", EVALUATOR, sha256_file(EVALUATOR, "Phase112 evaluator")),
        ("phase112_wrapper", WRAPPER, sha256_file(WRAPPER, "Phase112 wrapper")),
        ("phase112_focused_tests", FOCUSED_TESTS, sha256_file(FOCUSED_TESTS, "Phase112 focused tests")),
        ("phase95_result", PHASE95_RESULT, PHASE95_RESULT_SHA256),
        ("phase65_manifest", PHASE65_MANIFEST, PHASE65_MANIFEST_SHA256),
    ):
        pin = authority.get(key)
        if not isinstance(pin, dict):
            raise fail(f"authorization/{key} missing")
        assert_equal(pin.get("path"), relative(path), f"authorization/{key}/path")
        assert_equal(pin.get("sha256"), expected, f"authorization/{key}/sha256")
    for key, expected in {
        "freeze_commit": FREEZE_COMMIT, "audit_commit": AUDIT_COMMIT,
        "implementation_commit": IMPLEMENTATION_COMMIT,
        "manifest_sha256": sha256_file(MANIFEST, "Phase112 manifest"),
        "evaluator_sha256": sha256_file(EVALUATOR, "Phase112 evaluator"),
        "wrapper_sha256": sha256_file(WRAPPER, "Phase112 wrapper"),
        "binary_sha256": SOURCE_SHA256["binary"],
    }.items():
        assert_equal(authority.get(key), expected, f"authorization/authority/{key}")
    assert_equal(auth.get("routes"), list(ROUTES), "authorization/routes")
    candidate = auth.get("candidate")
    if not isinstance(candidate, dict):
        raise fail("authorization/candidate missing")
    for key, expected in {
        "id": CANDIDATE_ID, "candidate_count": 1,
        "selectors": [PHASE93_SELECTOR, VECTOR_SELECTOR, QR_SELECTOR, OFFSET_SELECTOR],
        "base_selectors": list(BASE_SELECTORS), "runs_per_route": 1,
        "controls": 0, "reruns": 0, "fallbacks": 0, "truth_evaluation": False,
        "accuracy_scoring": False, "solution_output_publication": False,
        "offset_final_output_only": True, "base_correction_exactly_once": True,
    }.items():
        assert_equal(candidate.get(key), expected, f"authorization/candidate/{key}")
    matrix = auth.get("matrix")
    if not isinstance(matrix, dict):
        raise fail("authorization/matrix missing")
    for key, expected in {
        "candidate_count": 1, "route_count": 2, "runs_per_route": 1,
        "native_invocations": 2, "raw_phone_gnss_process_reads_max": 2,
        "raw_phone_imu_process_reads_max": 2, "broadcast_navigation_process_reads_max": 2,
        "raw_base_hash_reads_max": 2, "raw_base_native_process_reads_max": 2,
        "truth_reads": 0, "mat_reads_or_generated": 0, "phone_coordinate_reads": 0,
        "precomputed_coordinate_reads": 0, "pdc_reads": 0, "accuracy_calculations": 0,
        "kaggle_or_token_access": 0, "solution_rows_authorized": False,
        "reruns": 0, "fallbacks": 0,
    }.items():
        assert_equal(matrix.get(key), expected, f"authorization/matrix/{key}")
    policy = auth.get("execution_policy")
    if not isinstance(policy, dict):
        raise fail("authorization/execution_policy missing")
    for key, expected in {
        "authorized": True, "order": "MTV-A then LAX-T, sequentially",
        "one_invocation_per_route": True, "no_rerun": True, "no_solver_fallback": True,
        "solver_filter_lm_equation_unit_sigma_unchanged": True,
        "truth_or_accuracy_evaluation": False,
        "solution_output": "isolated withheld path; hash seal only, no coordinate interpretation/publication",
        "base_input": "sealed raw base RINEX; one wrapper hash read plus one native process read per route",
        "offset": "official Pixel5 helper once at final output boundary only",
    }.items():
        assert_equal(policy.get(key), expected, f"authorization/policy/{key}")
    boundary = auth.get("release_boundary")
    if not isinstance(boundary, dict):
        raise fail("authorization/release_boundary missing")
    for key, expected in {
        "raw_execution_authorized": True, "structural_result_authorized": True,
        "truth_or_accuracy_evaluation_authorized": False,
        "solution_publication_authorized": False, "kaggle_submission_authorized": False,
    }.items():
        assert_equal(boundary.get(key), expected, f"authorization/boundary/{key}")
    return auth


def verify_pre_raw() -> dict[str, Any]:
    freeze = verify_freeze()
    manifest = verify_manifest()
    return {
        "status": "pre-raw-verified", "phase": 112, "execution_label": "Luna Max",
        "candidate_count": 1, "route_ids": list(ROUTES), "runs_per_route": 1,
        "raw_execution_authorized": False, "raw_phone_gnss_reads": 0,
        "raw_phone_imu_reads": 0, "broadcast_navigation_reads": 0,
        "raw_base_rinex_reads": 0, "raw_base_hash_reads": 0,
        "native_solver_invocations": 0, "truth_reads": 0,
        "mat_reads_or_generated": 0, "phone_coordinate_reads": 0,
        "precomputed_coordinate_reads": 0, "pdc_reads": 0,
        "accuracy_calculations": 0, "solution_output_published": False,
        "freeze_sha256": FREEZE_SHA256,
        "manifest_sha256": sha256_file(MANIFEST, "Phase112 manifest"),
        "freeze_status": freeze["status"], "manifest_status": manifest["status"],
    }


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
    except (Phase112ContractError, OSError) as exc:
        print(f"phase112 pre-raw verifier: fail-closed: {exc}", file=sys.stderr)
        raise SystemExit(2)
