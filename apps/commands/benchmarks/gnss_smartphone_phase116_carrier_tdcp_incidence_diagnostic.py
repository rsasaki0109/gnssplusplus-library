#!/usr/bin/env python3
"""Launch-free Phase116 ordinary-TDCP diagnostic contract.

The verifier reads only the Phase116 freeze, native source/build pins, and
the already sealed Phase112 path metadata.  It deliberately does not stat,
hash, or open a raw phone member, base RINEX member, truth file, MAT file, or
solution.  A separately committed authorization is required before the
execution wrapper may materialize the two raw/base routes.
"""

from __future__ import annotations

import hashlib
import json
from pathlib import Path
import sys
from typing import Any


ROOT = Path(__file__).resolve().parents[3]
FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_phase116_carrier_tdcp_source_parity_freeze_v1.json"
MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase116_carrier_tdcp_incidence_diagnostic_manifest_v1.json"
AUTHORIZATION = ROOT / "docs/use_cases/records/smartphone_r5_phase116_carrier_tdcp_incidence_diagnostic_authorization_v1.json"
PHASE112_MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase112_main_output_offset_manifest_v1.json"
EVALUATOR = Path(__file__).resolve()
WRAPPER = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase116_carrier_tdcp_incidence_diagnostic_execute.py"
FOCUSED_TESTS = ROOT / "tests/test_smartphone_phase116_carrier_tdcp_incidence_diagnostic.py"
APP = ROOT / "apps/native/gnss_fgo_imu_no_base.cpp"
FGO_HEADER = ROOT / "include/libgnss++/algorithms/fgo.hpp"
FGO_CONFIG = ROOT / "include/libgnss++/algorithms/fgo_config.hpp"
FGO_PROBLEMS = ROOT / "src/algorithms/fgo_problems.cpp"
BINARY = ROOT / "build/apps/gnss_fgo_imu_no_base"

FREEZE_SHA256 = "55c2bc3758dd3c7b15b36b66d89cbbe3089777cdf8c1772a2c289e5245c29d1c"
FREEZE_COMMIT = "631efee766c7b3d1e9006e4e11cc45559f84c1c5"
AUDIT_COMMIT = "03e5360e2258e0611e8e66c958e1803f5f4fc9c2"
IMPLEMENTATION_COMMIT = "d7ffd59c446d87d84753eb3441c365b0fdba8fbb"
PHASE112_MANIFEST_SHA256 = "d8be91f42f07196e07b293558cc6b83dd261226af577c1b26b167eead75902e4"
SOURCE_SHA256 = {
    "app": "1c5002fb9d9c614e139a7928ad8e0cd28149e82456505b71c4cdfbfba5fcad37",
    "fgo_header": "3f7c30ed4448fc59980fd1e606e03518dfbe7e323fce983ab711bed725bb60a0",
    "fgo_config": "a7009ba418601b7b0840c8ce7878aea402c820bb7c95e7114284df54c29db4f0",
    "fgo_problems": "c33f75c94b0fb9145ff4987905e32f4a1955192ee2ad3df10ada7142d45d99b4",
    "binary": "4c4a12bc6b2f13db69cfff46a034ede6f97b7c7c61968871a206fe72074c0190",
}

ROUTES = (
    "2021-03-16-18-59-us-ca-mtv-a/pixel5",
    "2022-04-01-18-22-us-ca-lax-t/pixel5",
)
DOMAIN_ROWS = {ROUTES[0]: 2158, ROUTES[1]: 1465}
PROBLEM_EPOCHS = {route: rows + 1 for route, rows in DOMAIN_ROWS.items()}
RAW_NAMES = ("device_gnss.csv", "device_imu.csv", "brdc.nav")
PHASE93_SELECTOR = "--native-source-clock-c0d-gnss-first-meter-state-handoff"
VECTOR_SELECTOR = "--native-source-clock-c0d-epoch-vector-parity"
QR_SELECTOR = "--native-source-clock-c0d-phase99-main-multifrontal-qr-solver"
DIAGNOSTIC_SELECTOR = "--native-phase116-carrier-tdcp-incidence-diagnostic"
BASE_SELECTORS = (
    "--native-base-pseudorange-compensation",
    "--native-base-pseudorange-source-miss-mask",
    "--native-base-pseudorange-preserve-additional-frequency-bands",
)
OFFSET_SELECTOR = "--native-upstream-position-offset"
BASE_FLAG = ("--native-base-rinex", "__PHASE116_RAW_BASE_RINEX__")
BASE_SHA_FLAG = ("--native-base-rinex-sha256", "__PHASE116_RAW_BASE_SHA256__")
CANDIDATE_ID = "phase116-raw-carrier-tdcp-incidence-diagnostic-v1"
OUTPUT_ROOT = "output/smartphone-r5/phase116-carrier-tdcp-incidence-v1/"
SCHEMA = "smartphone-r5-phase116-carrier-tdcp-incidence-diagnostic-manifest.v1"
AUTH_SCHEMA = "smartphone-r5-phase116-carrier-tdcp-incidence-diagnostic-authorization.v1"

REQUIRED_FLAGS = (
    "--all-epochs", "--android-raw-utc-keys", "--android-raw-clock-only",
    "--android-utc-wall-clock-fallback", "--native-pdc-imu-tdcp-no-bridge",
    "--native-source-direct-observable-quality", "--native-source-clock-c0d-factor",
    "--native-source-clock-c0d-meter-state-parity",
    "--native-source-clock-c0d-active-solve-diagnostic", PHASE93_SELECTOR,
    VECTOR_SELECTOR, QR_SELECTOR, *BASE_SELECTORS, OFFSET_SELECTOR,
    DIAGNOSTIC_SELECTOR,
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
    "--native-direct-wls-ephemeral-c7d-main-seed",
    "--native-phase104-stage-main-attribution", "--native-quality-anchor",
)
FORBIDDEN_PATH_TERMS = (
    ".mat", "truth", "ground_truth", "validation", "holdout", "precomputed",
    "coordinate", "pdc", "kaggle", "token",
)


class Phase116ContractError(ValueError):
    """Raised when the pinned Phase116 contract fails closed."""


def fail(message: str) -> Phase116ContractError:
    return Phase116ContractError(message)


def relative(path: Path) -> str:
    try:
        return str(path.relative_to(ROOT))
    except ValueError:
        return str(path)


def assert_equal(actual: Any, expected: Any, label: str) -> None:
    if actual != expected:
        raise fail(f"{label}: expected {expected!r}, got {actual!r}")


def sha256_file(path: Path, label: str) -> str:
    # Input member bytes are intentionally unread before the auth boundary.
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


def verify_freeze() -> dict[str, Any]:
    assert_equal(sha256_file(FREEZE, "Phase116 freeze"), FREEZE_SHA256, "freeze/sha256")
    freeze = read_json(FREEZE, "Phase116 freeze")
    for key, expected in {
        "schema_version": "smartphone-r5-phase116-carrier-tdcp-source-parity-freeze.v1",
        "phase": 116, "execution_label": "Luna Max",
        "status": "frozen-one-diagnostic-before-execution",
    }.items():
        assert_equal(freeze.get(key), expected, f"freeze/{key}")
    candidate = freeze.get("candidate")
    if not isinstance(candidate, dict):
        raise fail("freeze/candidate missing")
    for key, expected in {
        "id": CANDIDATE_ID, "opt_in": True, "default_off": True,
        "diagnostic_only": True, "raw_only": True,
        "solution_withheld": True, "accuracy_evaluation": False,
    }.items():
        assert_equal(candidate.get(key), expected, f"freeze/candidate/{key}")
    decision = freeze.get("decision")
    if not isinstance(decision, dict):
        raise fail("freeze/decision missing")
    for key in (
        "algorithm_or_binary_change", "graph_factor_value_key_change",
        "c7_d_qr_base_offset_change", "equation_unit_sigma_filter_lm_change",
        "legacy_default_changed", "pdc_enabled", "raw_execution_authorized",
        "solver_execution_authorized", "truth_evaluation_authorized",
        "accuracy_authorized", "solution_publication_authorized",
    ):
        assert_equal(decision.get(key), False, f"freeze/decision/{key}")
    invariants = freeze.get("parity_invariants")
    if not isinstance(invariants, dict):
        raise fail("freeze/parity_invariants missing")
    for key in (
        "ordinary_tdcp_same_satellite_same_signal",
        "standalone_carrier_phase_factors_remain_disabled",
        "double_difference_factors_remain_disabled_in_no_base_app",
        "ambiguity_state_topology_unchanged", "c7_d_exact_keys_units_and_handoff",
        "ccdd_equation_and_sigma", "phase99_multifrontal_qr_and_ordering",
        "raw_base_pseudorange_only_scope", "pixel5_final_offset",
        "carrier_and_tdcp_masks_unchanged", "imu_p_doppler_factors_unchanged",
        "legacy_selector_off", "no_fallback_or_rerun",
    ):
        assert_equal(invariants.get(key), True, f"freeze/parity_invariants/{key}")
    assert_equal(invariants.get("tdcp_sigma_m"), 0.03, "freeze/tdcp_sigma_m")
    return freeze


def verify_implementation() -> dict[str, str]:
    paths = {
        "app": APP, "fgo_header": FGO_HEADER, "fgo_config": FGO_CONFIG,
        "fgo_problems": FGO_PROBLEMS, "binary": BINARY,
    }
    actual = {key: sha256_file(path, f"implementation/{key}") for key, path in paths.items()}
    assert_equal(actual, SOURCE_SHA256, "implementation/source_sha256")
    source = APP.read_text(encoding="utf-8")
    header = FGO_HEADER.read_text(encoding="utf-8")
    config = FGO_CONFIG.read_text(encoding="utf-8")
    problems = FGO_PROBLEMS.read_text(encoding="utf-8")
    for marker, payload in (
        (DIAGNOSTIC_SELECTOR, source),
        ("evaluatePhase116CarrierTdcp", source),
        ("writePhase116CarrierTdcpReport", source),
        ("phase116_carrier_tdcp_incidence", source),
        ("TdcpSignalDiagnostics", header),
        ("use_carrier_tdcp_incidence_diagnostic", config),
        ("signalDiagnostics", problems),
        ("accepted_pairs", problems),
        ("rejected_missing_previous", problems),
    ):
        if marker not in payload:
            raise fail(f"Phase116 implementation marker missing: {marker}")
    return actual


def output_path(route: str, name: str) -> str:
    return f"{OUTPUT_ROOT}{route.replace('/', '__')}/{name}"


def command_template(route: str) -> list[str]:
    return [
        "build/apps/gnss_fgo_imu_no_base", "--dataset-id", route,
        "--android-gnss", "__PHASE116_RAW_DEVICE_GNSS__",
        "--android-imu", "__PHASE116_RAW_DEVICE_IMU__",
        "--nav", "__PHASE116_RAW_BROADCAST_NAV__", "--all-epochs",
        "--android-raw-utc-keys", "--android-raw-clock-only",
        "--android-utc-wall-clock-fallback", "--native-pdc-imu-tdcp-no-bridge",
        "--native-source-direct-observable-quality", "--native-source-clock-c0d-factor",
        "--native-source-clock-c0d-meter-state-parity",
        "--native-source-clock-c0d-active-solve-diagnostic", PHASE93_SELECTOR,
        VECTOR_SELECTOR, QR_SELECTOR, *BASE_SELECTORS, OFFSET_SELECTOR,
        DIAGNOSTIC_SELECTOR, BASE_FLAG[0], BASE_FLAG[1],
        BASE_SHA_FLAG[0], BASE_SHA_FLAG[1], "--out",
        output_path(route, "withheld_solution_output.csv"), "--summary-json",
        output_path(route, "summary.json"),
    ]


def phase112_route_metadata() -> dict[str, dict[str, Any]]:
    phase112 = read_json(PHASE112_MANIFEST, "sealed Phase112 manifest")
    assert_equal(sha256_file(PHASE112_MANIFEST, "sealed Phase112 manifest"),
                 PHASE112_MANIFEST_SHA256, "Phase112 manifest/sha256")
    records = phase112.get("routes")
    if not isinstance(records, list) or [item.get("dataset_id") for item in records] != list(ROUTES):
        raise fail("sealed Phase112 route metadata is not the exact A/LAX order")
    return {item["dataset_id"]: item for item in records}


def validate_command(route: str, command: Any, record: dict[str, Any],
                     source_record: dict[str, Any]) -> None:
    assert_equal(command, command_template(route), f"manifest/{route}/exact command")
    for token in command:
        if token in FORBIDDEN_FLAGS or (
            not token.startswith("--") and
            any(term in token.lower() for term in FORBIDDEN_PATH_TERMS)
        ):
            raise fail(f"forbidden command token: {route}/{token}")
    for flag in REQUIRED_FLAGS:
        assert_equal(command.count(flag), 1, f"command/{route}/{flag}")
    for flag, name in (
        ("--android-gnss", "device_gnss.csv"),
        ("--android-imu", "device_imu.csv"),
        ("--nav", "brdc.nav"),
    ):
        assert_equal(command.count(flag), 1, f"command/{route}/{flag}")
        placeholder = {
            "device_gnss.csv": "__PHASE116_RAW_DEVICE_GNSS__",
            "device_imu.csv": "__PHASE116_RAW_DEVICE_IMU__",
            "brdc.nav": "__PHASE116_RAW_BROADCAST_NAV__",
        }[name]
        assert_equal(command[command.index(flag) + 1], placeholder,
                     f"command/{route}/{name}/placeholder")
        pin = record.get("raw_inputs", {}).get(name)
        source_pin = source_record.get("raw_inputs", {}).get(name)
        if not isinstance(pin, dict) or not isinstance(source_pin, dict):
            raise fail(f"raw pin missing: {route}/{name}")
        for key in ("path", "sha256", "bytes"):
            assert_equal(pin.get(key), source_pin.get(key),
                         f"manifest/raw/{route}/{name}/{key}")
        assert_equal(pin.get("read_at_manifest_creation"), False,
                     f"manifest/raw/{route}/{name}/read")
    pin = record.get("base_input")
    source_pin = source_record.get("base_input")
    if not isinstance(pin, dict) or not isinstance(source_pin, dict):
        raise fail(f"base pin missing: {route}")
    for key in ("path", "sha256", "bytes", "observed_dt_s",
                "moving_mean_samples", "approx_position_xyz_m"):
        assert_equal(pin.get(key), source_pin.get(key),
                     f"manifest/base/{route}/{key}")
    assert_equal(pin.get("read_at_manifest_creation"), False,
                 f"manifest/base/{route}/read")
    assert_equal(pin.get("hash_read_at_manifest_creation"), False,
                 f"manifest/base/{route}/hash_read")
    assert_equal(record.get("runs"), 1, f"manifest/{route}/runs")


def verify_manifest() -> dict[str, Any]:
    verify_freeze()
    implementation = verify_implementation()
    source_routes = phase112_route_metadata()
    manifest = read_json(MANIFEST, "Phase116 pre-raw manifest")
    for key, expected in {
        "schema_version": SCHEMA, "phase": 116,
        "execution_label": "Luna Max",
        "status": "sealed-before-phase116-raw-execution",
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
    impl = manifest.get("implementation")
    if not isinstance(impl, dict):
        raise fail("manifest/implementation missing")
    for key, expected in {
        "commit": IMPLEMENTATION_COMMIT,
        "candidate_id": CANDIDATE_ID,
        "legacy_default_unchanged": True,
        "graph_solver_factors_values_unchanged": True,
        "diagnostic_only": True,
    }.items():
        assert_equal(impl.get(key), expected, f"manifest/implementation/{key}")
    assert_equal(impl.get("source_sha256"), implementation,
                 "manifest/implementation/source_sha256")
    for key, path in (("evaluator", EVALUATOR), ("wrapper", WRAPPER),
                      ("focused_tests", FOCUSED_TESTS)):
        pin = manifest.get(key)
        if not isinstance(pin, dict):
            raise fail(f"manifest/{key} missing")
        assert_equal(pin.get("path"), relative(path), f"manifest/{key}/path")
        expected_hash = pin.get("sha256")
        if not isinstance(expected_hash, str) or len(expected_hash) != 64:
            raise fail(f"manifest/{key}/sha256 missing")
        assert_equal(sha256_file(path, f"manifest/{key}"), expected_hash,
                     f"manifest/{key}/sha256")
    candidate = manifest.get("candidate")
    if not isinstance(candidate, dict):
        raise fail("manifest/candidate missing")
    for key, expected in {
        "id": CANDIDATE_ID, "candidate_count": 1, "opt_in": True,
        "default_off": True, "diagnostic_only": True,
        "ordinary_tdcp_only": True, "runs_per_route": 1, "controls": 0,
        "reruns": 0, "fallbacks": 0, "truth_evaluation": False,
        "accuracy_scoring": False, "solution_output_publication": False,
        "solution_withheld": True,
    }.items():
        assert_equal(candidate.get(key), expected, f"manifest/candidate/{key}")
    assert_equal(candidate.get("selector"), DIAGNOSTIC_SELECTOR,
                 "manifest/candidate/selector")
    assert_equal(candidate.get("selectors"), [PHASE93_SELECTOR, VECTOR_SELECTOR,
                                                QR_SELECTOR, OFFSET_SELECTOR,
                                                DIAGNOSTIC_SELECTOR],
                 "manifest/candidate/selectors")
    assert_equal(candidate.get("base_selectors"), list(BASE_SELECTORS),
                 "manifest/candidate/base_selectors")
    routes = manifest.get("routes")
    if not isinstance(routes, list) or [item.get("dataset_id") for item in routes] != list(ROUTES):
        raise fail("manifest route order/count changed")
    for record in routes:
        route = record.get("dataset_id")
        if route not in DOMAIN_ROWS:
            raise fail(f"unknown route: {route}")
        assert_equal(record.get("domain_rows"), DOMAIN_ROWS[route],
                     f"manifest/{route}/domain_rows")
        assert_equal(record.get("expected_problem_epochs"), PROBLEM_EPOCHS[route],
                     f"manifest/{route}/problem_epochs")
        assert_equal(record.get("expected_output_epochs"), PROBLEM_EPOCHS[route],
                     f"manifest/{route}/output_epochs")
        validate_command(route, record.get("command"), record, source_routes[route])
    raw_contract = manifest.get("raw_input_contract")
    if not isinstance(raw_contract, dict):
        raise fail("manifest/raw_input_contract missing")
    for key, expected in {
        "input_names_exact": list(RAW_NAMES), "phone_gnss_only": True,
        "phone_imu_only": True, "broadcast_navigation_only": True,
        "sealed_raw_base_rinex_only": True, "content_copy_or_transform": False,
        "truth_mat_precomputed_coordinate_pdc_kaggle_accuracy": False,
    }.items():
        assert_equal(raw_contract.get(key), expected,
                     f"manifest/raw_input_contract/{key}")
    telemetry = manifest.get("diagnostic_contract")
    if not isinstance(telemetry, dict):
        raise fail("manifest/diagnostic_contract missing")
    for key, expected in {
        "ordinary_tdcp_same_satellite_same_signal": True,
        "per_signal_frequency_band": True,
        "drop_reasons": ["gap", "clock_discontinuity", "loss_of_lock", "nonfinite", "missing_previous", "code_phase_jump", "missing_wavelength"],
        "costs": ["robust", "unwhitened", "whitened"],
        "initial_and_final_where_accessible": True,
        "connected_keys": True,
        "solution_rows_exported": False,
        "graph_mutation": False,
    }.items():
        assert_equal(telemetry.get(key), expected,
                     f"manifest/diagnostic_contract/{key}")
    matrix = manifest.get("matrix")
    if not isinstance(matrix, dict):
        raise fail("manifest/matrix missing")
    for key, expected in {
        "candidate_count": 1, "route_count": 2, "runs_per_route": 1,
        "native_solver_invocations_planned": 2, "controls": 0,
        "reruns": 0, "fallbacks": 0, "truth_reads_planned": 0,
        "mat_reads_or_generated_planned": 0, "phone_coordinate_reads_planned": 0,
        "precomputed_coordinate_reads_planned": 0, "pdc_reads_planned": 0,
        "accuracy_calculations_planned": 0, "solution_rows_authorized": False,
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
    assert_equal(accounting.get("raw_content_copied_or_transformed"), False,
                 "manifest/read_accounting/copy")
    return manifest


def verify_authorization(manifest: dict[str, Any] | None = None) -> dict[str, Any]:
    if manifest is None:
        manifest = verify_manifest()
    authorization = read_json(AUTHORIZATION, "Phase116 raw authorization")
    for key, expected in {
        "schema_version": AUTH_SCHEMA, "phase": 116,
        "execution_label": "Luna Max",
        "status": "authorized-for-exact-two-route-phase116-raw-base-diagnostic",
    }.items():
        assert_equal(authorization.get(key), expected, f"authorization/{key}")
    authority = authorization.get("authority")
    if not isinstance(authority, dict):
        raise fail("authorization/authority missing")
    for key, path in (
        ("phase116_freeze", FREEZE), ("phase116_manifest", MANIFEST),
        ("phase116_evaluator", EVALUATOR), ("phase116_wrapper", WRAPPER),
        ("phase116_focused_tests", FOCUSED_TESTS),
        ("phase112_manifest", PHASE112_MANIFEST),
    ):
        pin = authority.get(key)
        if not isinstance(pin, dict):
            raise fail(f"authorization/{key} missing")
        assert_equal(pin.get("path"), relative(path), f"authorization/{key}/path")
        expected_hash = pin.get("sha256")
        if not isinstance(expected_hash, str) or len(expected_hash) != 64:
            raise fail(f"authorization/{key}/sha256 missing")
        expected = FREEZE_SHA256 if key == "phase116_freeze" else PHASE112_MANIFEST_SHA256 if key == "phase112_manifest" else sha256_file(path, f"authorization/{key}")
        assert_equal(expected_hash, expected, f"authorization/{key}/sha256")
    for key, expected in {
        "freeze_commit": FREEZE_COMMIT, "audit_commit": AUDIT_COMMIT,
        "implementation_commit": IMPLEMENTATION_COMMIT,
        "manifest_sha256": sha256_file(MANIFEST, "Phase116 manifest"),
        "evaluator_sha256": sha256_file(EVALUATOR, "Phase116 evaluator"),
        "wrapper_sha256": sha256_file(WRAPPER, "Phase116 wrapper"),
        "binary_sha256": SOURCE_SHA256["binary"],
    }.items():
        assert_equal(authority.get(key), expected, f"authorization/authority/{key}")
    assert_equal(authorization.get("routes"), list(ROUTES), "authorization/routes")
    candidate = authorization.get("candidate")
    if not isinstance(candidate, dict):
        raise fail("authorization/candidate missing")
    for key, expected in {
        "id": CANDIDATE_ID, "candidate_count": 1,
        "selector": DIAGNOSTIC_SELECTOR,
        "selectors": [PHASE93_SELECTOR, VECTOR_SELECTOR, QR_SELECTOR,
                      OFFSET_SELECTOR, DIAGNOSTIC_SELECTOR],
        "base_selectors": list(BASE_SELECTORS), "runs_per_route": 1,
        "controls": 0, "reruns": 0, "fallbacks": 0,
        "truth_evaluation": False, "accuracy_scoring": False,
        "solution_output_publication": False, "solution_withheld": True,
        "no_graph_mutation": True, "no_fallback_or_rerun": True,
    }.items():
        assert_equal(candidate.get(key), expected, f"authorization/candidate/{key}")
    matrix = authorization.get("matrix")
    if not isinstance(matrix, dict):
        raise fail("authorization/matrix missing")
    for key, expected in {
        "candidate_count": 1, "route_count": 2, "runs_per_route": 1,
        "native_invocations": 2, "raw_phone_gnss_process_reads_max": 2,
        "raw_phone_imu_process_reads_max": 2,
        "broadcast_navigation_process_reads_max": 2,
        "raw_base_hash_reads_max": 2, "raw_base_native_process_reads_max": 2,
        "truth_reads": 0, "mat_reads_or_generated": 0,
        "phone_coordinate_reads": 0, "precomputed_coordinate_reads": 0,
        "pdc_reads": 0, "accuracy_calculations": 0,
        "kaggle_or_token_access": 0, "solution_rows_authorized": False,
        "reruns": 0, "fallbacks": 0,
    }.items():
        assert_equal(matrix.get(key), expected, f"authorization/matrix/{key}")
    policy = authorization.get("execution_policy")
    if not isinstance(policy, dict):
        raise fail("authorization/execution_policy missing")
    for key, expected in {
        "authorized": True, "order": "MTV-A then LAX-T, sequentially",
        "one_invocation_per_route": True, "no_rerun": True,
        "no_solver_fallback": True,
        "solver_filter_lm_equation_unit_sigma_graph_unchanged": True,
        "truth_or_accuracy_evaluation": False,
        "solution_output": "isolated withheld path; never opened, hashed, published, or committed",
        "base_input": "sealed raw base RINEX; one wrapper hash read plus one native process read per route",
    }.items():
        assert_equal(policy.get(key), expected, f"authorization/policy/{key}")
    boundary = authorization.get("release_boundary")
    if not isinstance(boundary, dict):
        raise fail("authorization/release_boundary missing")
    for key, expected in {
        "raw_base_execution_authorized": True,
        "structural_result_authorized": True,
        "truth_or_accuracy_evaluation_authorized": False,
        "solution_publication_authorized": False,
        "kaggle_submission_authorized": False,
    }.items():
        assert_equal(boundary.get(key), expected, f"authorization/boundary/{key}")
    return authorization


def verify_pre_raw() -> dict[str, Any]:
    freeze = verify_freeze()
    manifest = verify_manifest()
    return {
        "status": "pre-raw-verified", "phase": 116,
        "execution_label": "Luna Max", "candidate_count": 1,
        "route_ids": list(ROUTES), "runs_per_route": 1,
        "raw_execution_authorized": False, "raw_phone_gnss_reads": 0,
        "raw_phone_imu_reads": 0, "broadcast_navigation_reads": 0,
        "raw_base_rinex_reads": 0, "raw_base_hash_reads": 0,
        "native_solver_invocations": 0, "truth_reads": 0,
        "mat_reads_or_generated": 0, "phone_coordinate_reads": 0,
        "precomputed_coordinate_reads": 0, "pdc_reads": 0,
        "accuracy_calculations": 0, "solution_output_published": False,
        "freeze_sha256": FREEZE_SHA256,
        "manifest_sha256": sha256_file(MANIFEST, "Phase116 manifest"),
        "freeze_status": freeze["status"], "manifest_status": manifest["status"],
    }


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--verify-freeze", action="store_true")
    parser.add_argument("--verify-manifest", action="store_true")
    parser.add_argument("--verify-authorization", action="store_true")
    parser.add_argument("--verify-pre-raw", action="store_true")
    args = parser.parse_args()
    if not any((args.verify_freeze, args.verify_manifest,
                args.verify_authorization, args.verify_pre_raw)):
        parser.error("one verification mode is required")
    try:
        if args.verify_freeze:
            verify_freeze()
        if args.verify_manifest:
            verify_manifest()
        if args.verify_authorization:
            verify_authorization()
        if args.verify_pre_raw:
            print(json.dumps(verify_pre_raw(), indent=2, sort_keys=True))
    except (Phase116ContractError, OSError) as exc:
        print(f"phase116 pre-raw verifier: fail-closed: {exc}", file=sys.stderr)
        raise SystemExit(2)
