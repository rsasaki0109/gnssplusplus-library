#!/usr/bin/env python3
"""Launch-free Phase120 structural contract validator.

Only tracked source and sealed metadata are inspected here.  The validator
deliberately has no child-process or payload path operation: raw phone/base
members, solutions, truth, MAT, coordinates, PDC, and Kaggle remain outside
this pre-authorization boundary.
"""

from __future__ import annotations

import hashlib
import json
from pathlib import Path
import sys
from typing import Any


ROOT = Path(__file__).resolve().parents[3]
SOURCE_FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_phase120_tdcp_equation_normalization_freeze_v1.json"
FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_phase120_tdcp_equation_normalization_structural_contract_freeze_v1.json"
AUDIT = ROOT / "docs/use_cases/records/smartphone_r5_phase120_tdcp_equation_normalization_structural_contract_audit_v1.md"
MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase120_tdcp_equation_normalization_structural_manifest_v1.json"
PHASE118_MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase118_tdcp_robust_k_structural_manifest_v1.json"
PHASE118_RESULT = ROOT / "docs/use_cases/records/smartphone_r5_phase118_tdcp_robust_k_structural_result_v1.json"
PHASE112_MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase112_main_output_offset_manifest_v1.json"
PHASE112_RESULT = ROOT / "docs/use_cases/records/smartphone_r5_phase112_main_output_offset_structural_result_v1.json"
PHASE117_RESULT = ROOT / "docs/use_cases/records/smartphone_r5_phase117_tdcp_weighting_structural_result_v1.json"
WRAPPER = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase120_tdcp_equation_normalization_structural_execute.py"
FOCUSED_TESTS = ROOT / "tests/test_smartphone_phase120_tdcp_equation_normalization_structural_execution.py"
BINARY = ROOT / "build/apps/gnss_fgo_imu_no_base"

FREEZE_COMMIT = "fd5dd1f03c08e650909473b9a77bb188b1d992e0"
FREEZE_SHA256 = "930235662b5dfffadb8957e450f20fb135495a8d380a5c692854d688cdeb9d7e"
AUDIT_COMMIT = "cc3cb416d52579c49ca7039cdc4928f52745a2b4"
AUDIT_SHA256 = "c393899dcbfe2052973038c12f1bfa25d8bff49b14d1100a53e1b8e62fa03a82"
SOURCE_FREEZE_COMMIT = "8e5dd41bebef64a93361a25db6275799452720f2"
SOURCE_FREEZE_SHA256 = "5087cbcd052de828b703659d4eed847ca7711481a7b7656af80013b9d0fc6359"
IMPLEMENTATION_COMMIT = "3ea6b2caf5c6f5be78c18e388e49301b961d3036"
PHASE118_MANIFEST_SHA256 = "e0a4a0e879a56d7336fe1e2cbf041e5a72920d8103dd22dfdc9dffddfe62ec1d"
PHASE118_RESULT_SHA256 = "88e8799050fd396eeb14e83d4f5923339279f46d0219a385e303d3cb50731538"
PHASE112_MANIFEST_SHA256 = "d8be91f42f07196e07b293558cc6b83dd261226af577c1b26b167eead75902e4"
PHASE112_RESULT_SHA256 = "087b1bf51b4e584b86a7b558560df4ceb78e19b01e572f2d60aeef4c3d759ba0"
PHASE117_RESULT_SHA256 = "2517dcc805146dc34e790c78a392caf000c79eaba837fe099b6d522c147e9cf2"

ROUTES = (
    "2021-03-16-18-59-us-ca-mtv-a/pixel5",
    "2022-04-01-18-22-us-ca-lax-t/pixel5",
)
DOMAIN_ROWS = {ROUTES[0]: 2158, ROUTES[1]: 1465}
PROBLEM_EPOCHS = {route: rows + 1 for route, rows in DOMAIN_ROWS.items()}
RAW_NAMES = ("device_gnss.csv", "device_imu.csv", "brdc.nav")
SELECTOR = "--native-phase120-official-tdcp-resl-atmosphere-cancellation"
PHASE118_SELECTOR = "--native-phase118-official-tdcp-huber-k"
PHASE117_SELECTOR = "--native-phase117-tdcp-snr-type-sigma"
BASE_SELECTORS = (
    "--native-base-pseudorange-compensation",
    "--native-base-pseudorange-source-miss-mask",
    "--native-base-pseudorange-preserve-additional-frequency-bands",
)
PHASE112_SELECTORS = (
    "--native-source-clock-c0d-gnss-first-meter-state-handoff",
    "--native-source-clock-c0d-epoch-vector-parity",
    "--native-source-clock-c0d-phase99-main-multifrontal-qr-solver",
    "--native-upstream-position-offset",
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
    *PHASE112_SELECTORS,
    *BASE_SELECTORS,
    PHASE118_SELECTOR,
    SELECTOR,
)
FORBIDDEN_FLAGS = (
    "--obs",
    PHASE117_SELECTOR,
    "--native-phase116-carrier-tdcp-incidence-diagnostic",
    "--native-source-clock-c0d-gnss-first-raw-drift-d-initializer",
    "--native-direct-wls-ephemeral-c7d-main-seed",
    "--native-gnss-first-velocity-only-handoff",
    "--native-direct-doppler-wls-handoff",
    "--native-pdc-state-bridge",
    "--native-upstream-quality",
    "--native-source-clock-c0d-phase94-stage-diagnostics",
    "--native-source-clock-c0d-phase96-main-diagnostics",
    "--native-source-clock-c0d-phase97-singular-system-diagnostics",
    "--native-source-clock-c0d-phase98-solver-rank-diagnostic",
    "--native-phase104-stage-main-attribution",
    "--native-quality-anchor",
)
FORBIDDEN_PATH_TERMS = (
    ".mat",
    "truth",
    "ground_truth",
    "validation",
    "holdout",
    "precomputed",
    "coordinate",
    "pdc",
    "kaggle",
    "token",
)
SCHEMA = "smartphone-r5-phase120-tdcp-equation-normalization-structural-manifest.v1"
CANDIDATE_ID = "phase120-official-tdcp-resl-atmosphere-cancellation-v1"

IMPLEMENTATION_SOURCES = {
    "apps/native/gnss_fgo_imu_no_base.cpp": ROOT / "apps/native/gnss_fgo_imu_no_base.cpp",
    "include/libgnss++/algorithms/fgo.hpp": ROOT / "include/libgnss++/algorithms/fgo.hpp",
    "include/libgnss++/algorithms/fgo_config.hpp": ROOT / "include/libgnss++/algorithms/fgo_config.hpp",
    "include/libgnss++/algorithms/tdcp_contract.hpp": ROOT / "include/libgnss++/algorithms/tdcp_contract.hpp",
    "src/algorithms/fgo.cpp": ROOT / "src/algorithms/fgo.cpp",
    "src/algorithms/fgo_gtsam_backend.cpp": ROOT / "src/algorithms/fgo_gtsam_backend.cpp",
    "src/algorithms/fgo_internal.hpp": ROOT / "src/algorithms/fgo_internal.hpp",
    "src/algorithms/fgo_problems.cpp": ROOT / "src/algorithms/fgo_problems.cpp",
    "tests/test_fgo.cpp": ROOT / "tests/test_fgo.cpp",
    "tests/test_smartphone_phase120_tdcp_equation_normalization.py": ROOT / "tests/test_smartphone_phase120_tdcp_equation_normalization.py",
    "tests/CMakeLists.txt": ROOT / "tests/CMakeLists.txt",
}


class Phase120ContractError(ValueError):
    """Raised whenever the Phase120 structural contract fails closed."""


def fail(message: str) -> Phase120ContractError:
    return Phase120ContractError(message)


def relative(path: Path) -> str:
    try:
        return str(path.relative_to(ROOT))
    except ValueError:
        return str(path)


def assert_equal(actual: Any, expected: Any, label: str) -> None:
    if actual != expected:
        raise fail(f"{label}: expected {expected!r}, got {actual!r}")


def sha256_file(path: Path, label: str) -> str:
    # This check precedes is_file/open so an accidental payload path can never
    # be probed by launch-free verification.
    lowered = path.name.lower()
    if (
        lowered in RAW_NAMES
        or lowered == "base.obs"
        or lowered.endswith(".csv")
        or "ground_truth" in lowered
        or "truth" in lowered
    ):
        raise fail(f"payload/solution hash forbidden before authorization: {label}")
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


def verify_source_freeze() -> dict[str, Any]:
    assert_equal(sha256_file(SOURCE_FREEZE, "Phase120 source freeze"), SOURCE_FREEZE_SHA256,
                 "source_freeze/sha256")
    source = read_json(SOURCE_FREEZE, "Phase120 source freeze")
    for key, expected in {
        "schema_version": "smartphone-r5-phase120-tdcp-equation-normalization-freeze.v1",
        "phase": 120,
        "execution_label": "Luna Max",
        "status": "frozen-default-off-source-backed-candidate-no-execution",
    }.items():
        assert_equal(source.get(key), expected, f"source_freeze/{key}")
    candidate = source.get("candidate")
    if not isinstance(candidate, dict):
        raise fail("source_freeze/candidate missing")
    for key, expected in {
        "candidate_count": 1,
        "id": CANDIDATE_ID,
        "default_off": True,
        "opt_in": True,
    }.items():
        assert_equal(candidate.get(key), expected, f"source_freeze/candidate/{key}")
    assert_equal(candidate.get("measurement_contract", {}).get("source_parity_measurement_m"),
                 "carrier_phase_cycles * retained_wavelength_m + satellite_clock_m",
                 "source_freeze/measurement")
    authorization = source.get("authorization")
    if not isinstance(authorization, dict):
        raise fail("source_freeze/authorization missing")
    for key in (
        "implementation_authorized",
        "raw_execution_authorized",
        "truth_evaluation_authorized",
        "accuracy_authorized",
        "solution_publication_authorized",
        "kaggle_authorized",
    ):
        assert_equal(authorization.get(key), False, f"source_freeze/authorization/{key}")
    return source


def verify_freeze() -> dict[str, Any]:
    source = verify_source_freeze()
    assert_equal(sha256_file(FREEZE, "Phase120 structural freeze"), FREEZE_SHA256,
                 "freeze/sha256")
    freeze = read_json(FREEZE, "Phase120 structural freeze")
    for key, expected in {
        "schema_version": "smartphone-r5-phase120-tdcp-equation-normalization-structural-contract-freeze.v1",
        "phase": 120,
        "execution_label": "Luna Max",
        "status": "sealed-before-phase120-raw-execution",
    }.items():
        assert_equal(freeze.get(key), expected, f"freeze/{key}")
    audit = freeze.get("audit")
    if not isinstance(audit, dict):
        raise fail("freeze/audit missing")
    for key, expected in {
        "path": relative(AUDIT),
        "commit": AUDIT_COMMIT,
        "sha256": AUDIT_SHA256,
    }.items():
        assert_equal(audit.get(key), expected, f"freeze/audit/{key}")
    assert_equal(sha256_file(AUDIT, "Phase120 structural audit"), AUDIT_SHA256,
                 "freeze/audit/file_sha256")
    source_pin = freeze.get("source_freeze")
    if not isinstance(source_pin, dict):
        raise fail("freeze/source_freeze missing")
    for key, expected in {
        "path": relative(SOURCE_FREEZE),
        "commit": SOURCE_FREEZE_COMMIT,
        "sha256": SOURCE_FREEZE_SHA256,
        "raw_execution_authorized": False,
        "truth_evaluation_authorized": False,
    }.items():
        assert_equal(source_pin.get(key), expected, f"freeze/source_freeze/{key}")
    candidate = freeze.get("candidate")
    if not isinstance(candidate, dict):
        raise fail("freeze/candidate missing")
    for key, expected in {
        "candidate_count": 1,
        "id": CANDIDATE_ID,
        "selector": SELECTOR,
        "opt_in": True,
        "default_off": True,
        "composes_phase118": True,
        "phase118_selector": PHASE118_SELECTOR,
        "phase117_dynamic_sigma": False,
        "fixed_tdcp_sigma_m": 0.03,
        "official_setting_type": "Highway",
        "official_huber_k": 0.5,
        "unknown_type": "fail-closed",
        "missing_core_measurement": "fail-closed",
        "pair_gate_input": "historical corrected_carrier_m delta",
        "factor_count_and_reject_invariance": True,
        "factor_geometry_unchanged": True,
        "solution_withheld": True,
        "truth_evaluation": False,
        "accuracy_scoring": False,
    }.items():
        assert_equal(candidate.get(key), expected, f"freeze/candidate/{key}")
    assert_equal(candidate.get("source_measurement_m"),
                 "carrier_phase_cycles * retained_wavelength_m + satellite_clock_m",
                 "freeze/candidate/source_measurement")
    assert_equal(candidate.get("factor_delta_order"), "current minus previous",
                 "freeze/candidate/delta_order")
    gates = freeze.get("structural_gates")
    if not isinstance(gates, dict):
        raise fail("freeze/structural_gates missing")
    for key in (
        "default_off_legacy_isolation",
        "phase120_selector_active_exactly_once",
        "phase118_huber_selector_active_exactly_once",
        "phase117_dynamic_sigma_forbidden",
        "fixed_tdcp_sigma_0_03_m",
        "official_type_k_exact",
        "ordinary_tdcp_only",
        "source_measurement_exact",
        "pair_key_and_gate_unchanged",
        "tdcp_factor_and_reject_counts_unchanged",
        "tdcp_factor_equation_units_order_unchanged",
        "gnss_first_accepted_progress",
        "gnss_first_finite_strict_cost_decrease",
        "gnss_first_full_finite_c7_d_exact_handoff",
        "main_qr_selected_accepted_progress",
        "main_finite_strict_cost_decrease",
        "position_clock_finite_expected_coverage",
        "earth_valid_output_coverage",
        "base_correction_exactly_once",
        "pixel5_offset_exactly_once_final_boundary",
        "no_solver_fallback",
        "no_truth_mat_pdc_precomputed_coordinate_accuracy_kaggle",
        "solution_opaque_withheld",
    ):
        assert_equal(gates.get(key), True, f"freeze/structural_gates/{key}")
    policy = freeze.get("execution_policy")
    if not isinstance(policy, dict):
        raise fail("freeze/execution_policy missing")
    for key in (
        "raw_execution_authorized",
        "solver_execution_authorized",
        "structural_result_authorized",
        "truth_evaluation_authorized",
        "accuracy_authorized",
        "solution_publication_authorized",
        "kaggle_submission_authorized",
        "rerun_or_fallback_authorized",
    ):
        assert_equal(policy.get(key), False, f"freeze/execution_policy/{key}")
    return freeze


def verify_implementation(freeze: dict[str, Any]) -> dict[str, str]:
    implementation = freeze.get("implementation")
    if not isinstance(implementation, dict):
        raise fail("freeze/implementation missing")
    assert_equal(implementation.get("commit"), IMPLEMENTATION_COMMIT,
                 "implementation/commit")
    source_pins = implementation.get("source_sha256")
    if not isinstance(source_pins, dict):
        raise fail("freeze/implementation/source_sha256 missing")
    actual: dict[str, str] = {}
    for name, path in IMPLEMENTATION_SOURCES.items():
        expected = source_pins.get(name)
        if not isinstance(expected, str):
            raise fail(f"missing implementation source pin: {name}")
        actual[name] = sha256_file(path, f"implementation/{name}")
        assert_equal(actual[name], expected, f"implementation/{name}/sha256")
    binary = implementation.get("binary")
    if not isinstance(binary, dict):
        raise fail("freeze/implementation/binary missing")
    assert_equal(binary.get("path"), relative(BINARY), "implementation/binary/path")
    assert_equal(sha256_file(BINARY, "implementation/binary"), binary.get("sha256"),
                 "implementation/binary/sha256")
    markers = {
        ROOT / "apps/native/gnss_fgo_imu_no_base.cpp": (
            SELECTOR,
            "native_phase120_official_tdcp_resl_atmosphere_cancellation",
        ),
        ROOT / "include/libgnss++/algorithms/fgo_config.hpp": (
            "use_official_tdcp_resl_atmosphere_cancellation",
        ),
        ROOT / "include/libgnss++/algorithms/tdcp_contract.hpp": (
            "ordinaryTdcpCarrierMeters",
        ),
        ROOT / "src/algorithms/fgo_problems.cpp": (
            "legacy_gate_delta_carrier_m",
            "current.tdcp_carrier_m - previous.tdcp_carrier_m",
        ),
    }
    for path, required in markers.items():
        source = path.read_text(encoding="utf-8")
        for marker in required:
            if marker not in source:
                raise fail(f"implementation marker missing: {relative(path)}:{marker}")
    return actual


def verify_sealed_provenance() -> None:
    for path, expected, label in (
        (PHASE118_MANIFEST, PHASE118_MANIFEST_SHA256, "Phase118 manifest"),
        (PHASE118_RESULT, PHASE118_RESULT_SHA256, "Phase118 result"),
        (PHASE112_MANIFEST, PHASE112_MANIFEST_SHA256, "Phase112 manifest"),
        (PHASE112_RESULT, PHASE112_RESULT_SHA256, "Phase112 result"),
        (PHASE117_RESULT, PHASE117_RESULT_SHA256, "Phase117 result"),
    ):
        assert_equal(sha256_file(path, label), expected, f"{label}/sha256")
    phase118 = read_json(PHASE118_MANIFEST, "sealed Phase118 manifest")
    routes = phase118.get("routes")
    if not isinstance(routes, list) or [item.get("dataset_id") for item in routes] != list(ROUTES):
        raise fail("sealed Phase118 route order changed")
    for item in routes:
        route = item.get("dataset_id")
        for name in RAW_NAMES:
            metadata = item.get("raw_inputs", {}).get(name)
            if not isinstance(metadata, dict) or not isinstance(metadata.get("placeholder"), str):
                raise fail(f"missing sealed raw metadata: {route}/{name}")
            if metadata.get("payload_read_before_authorization") is not False:
                raise fail(f"sealed raw read boundary changed: {route}/{name}")
        base = item.get("base_input")
        if not isinstance(base, dict) or not isinstance(base.get("placeholder"), str):
            raise fail(f"missing sealed base metadata: {route}")
        if base.get("payload_read_before_authorization") is not False:
            raise fail(f"sealed base read boundary changed: {route}")


def command_template(route: str) -> list[str]:
    output_root = "output/smartphone-r5/phase120-tdcp-equation-normalization-v1/"
    output_route = route.replace("/", "__")
    return [
        "build/apps/gnss_fgo_imu_no_base",
        "--dataset-id", route,
        "--android-gnss", "__PHASE120_RAW_DEVICE_GNSS__",
        "--android-imu", "__PHASE120_RAW_DEVICE_IMU__",
        "--nav", "__PHASE120_RAW_BROADCAST_NAV__",
        "--all-epochs",
        "--android-raw-utc-keys", "--android-raw-clock-only",
        "--android-utc-wall-clock-fallback",
        "--native-pdc-imu-tdcp-no-bridge",
        "--native-source-direct-observable-quality",
        "--native-source-clock-c0d-factor",
        "--native-source-clock-c0d-meter-state-parity",
        "--native-source-clock-c0d-active-solve-diagnostic",
        "--native-source-clock-c0d-gnss-first-meter-state-handoff",
        "--native-source-clock-c0d-epoch-vector-parity",
        "--native-source-clock-c0d-phase99-main-multifrontal-qr-solver",
        *BASE_SELECTORS,
        "--native-upstream-position-offset",
        PHASE118_SELECTOR,
        SELECTOR,
        "--native-base-rinex", "__PHASE120_RAW_BASE_RINEX__",
        "--native-base-rinex-sha256", "__PHASE120_RAW_BASE_SHA256__",
        "--out", f"{output_root}{output_route}/withheld_solution_output.csv",
        "--summary-json", f"{output_root}{output_route}/summary.json",
    ]


def validate_command(route: str, command: Any) -> None:
    assert_equal(command, command_template(route), f"manifest/{route}/exact command")
    if not isinstance(command, list):
        raise fail(f"manifest/{route}/command is not argv list")
    for token in command:
        if token in FORBIDDEN_FLAGS:
            raise fail(f"forbidden command flag: {route}/{token}")
        if not token.startswith("--") and any(term in token.lower() for term in FORBIDDEN_PATH_TERMS):
            raise fail(f"forbidden command path token: {route}/{token}")
    for flag in REQUIRED_FLAGS:
        assert_equal(command.count(flag), 1, f"command/{route}/{flag}")
    assert_equal(command.count(PHASE117_SELECTOR), 0, f"command/{route}/Phase117 selector")
    placeholders = {
        "--android-gnss": "__PHASE120_RAW_DEVICE_GNSS__",
        "--android-imu": "__PHASE120_RAW_DEVICE_IMU__",
        "--nav": "__PHASE120_RAW_BROADCAST_NAV__",
        "--native-base-rinex": "__PHASE120_RAW_BASE_RINEX__",
        "--native-base-rinex-sha256": "__PHASE120_RAW_BASE_SHA256__",
    }
    for flag, placeholder in placeholders.items():
        assert_equal(command.count(flag), 1, f"command/{route}/{flag}")
        assert_equal(command[command.index(flag) + 1], placeholder,
                     f"command/{route}/{flag}/placeholder")


def verify_manifest() -> dict[str, Any]:
    freeze = verify_freeze()
    implementation = verify_implementation(freeze)
    verify_sealed_provenance()
    manifest = read_json(MANIFEST, "Phase120 structural manifest")
    for key, expected in {
        "schema_version": SCHEMA,
        "phase": 120,
        "execution_label": "Luna Max",
        "status": "sealed-before-phase120-raw-execution",
    }.items():
        assert_equal(manifest.get(key), expected, f"manifest/{key}")
    for section, expected in (
        ("freeze", {
            "path": relative(FREEZE),
            "sha256": FREEZE_SHA256,
            "commit": FREEZE_COMMIT,
            "raw_execution_authorized_before_manifest": False,
        }),
        ("contract_audit", {
            "path": relative(AUDIT),
            "sha256": AUDIT_SHA256,
            "commit": AUDIT_COMMIT,
        }),
        ("source_freeze", {
            "path": relative(SOURCE_FREEZE),
            "sha256": SOURCE_FREEZE_SHA256,
            "commit": SOURCE_FREEZE_COMMIT,
        }),
    ):
        value = manifest.get(section)
        if not isinstance(value, dict):
            raise fail(f"manifest/{section} missing")
        for key, item in expected.items():
            assert_equal(value.get(key), item, f"manifest/{section}/{key}")
    impl = manifest.get("implementation")
    if not isinstance(impl, dict):
        raise fail("manifest/implementation missing")
    for key, expected in {
        "commit": IMPLEMENTATION_COMMIT,
        "candidate_id": CANDIDATE_ID,
        "legacy_default_unchanged": True,
        "ordinary_tdcp_only": True,
        "pair_gate_input_unchanged": True,
        "graph_factors_values_equations_units_unchanged": True,
        "tdcp_sigma_m_fixed": 0.03,
        "phase118_huber_k_enabled": True,
        "phase117_dynamic_sigma_enabled": False,
        "solver_branch": "MULTIFRONTAL_QR",
        "no_fallback": True,
    }.items():
        assert_equal(impl.get(key), expected, f"manifest/implementation/{key}")
    assert_equal(impl.get("source_sha256"), implementation,
                 "manifest/implementation/source_sha256")
    binary = impl.get("binary")
    if not isinstance(binary, dict):
        raise fail("manifest/implementation/binary missing")
    assert_equal(binary.get("path"), relative(BINARY), "manifest/implementation/binary/path")
    assert_equal(binary.get("sha256"),
                 sha256_file(BINARY, "manifest/implementation/binary"),
                 "manifest/implementation/binary/sha256")
    artifacts = manifest.get("artifacts")
    if not isinstance(artifacts, dict):
        raise fail("manifest/artifacts missing")
    for key, path in (
        ("validator", Path(__file__)),
        ("runner", WRAPPER),
        ("focused_tests", FOCUSED_TESTS),
    ):
        pin = artifacts.get(key)
        if not isinstance(pin, dict):
            raise fail(f"manifest/artifacts/{key} missing")
        assert_equal(pin.get("path"), relative(path), f"manifest/artifacts/{key}/path")
        expected = pin.get("sha256")
        if not isinstance(expected, str) or len(expected) != 64:
            raise fail(f"manifest/artifacts/{key}/sha256 missing")
        assert_equal(sha256_file(path, f"manifest/artifacts/{key}"), expected,
                     f"manifest/artifacts/{key}/sha256")
    candidate = manifest.get("candidate")
    if not isinstance(candidate, dict):
        raise fail("manifest/candidate missing")
    for key, expected in {
        "candidate_count": 1,
        "id": CANDIDATE_ID,
        "selector": SELECTOR,
        "phase118_selector": PHASE118_SELECTOR,
        "opt_in": True,
        "default_off": True,
        "composes_phase118": True,
        "phase117_dynamic_sigma": False,
        "fixed_tdcp_sigma_m": 0.03,
        "official_setting_type": "Highway",
        "official_huber_k": 0.5,
        "factor_count_and_reject_invariance": True,
        "pair_gate_input": "historical corrected_carrier_m delta",
        "solution_withheld": True,
        "truth_evaluation": False,
        "accuracy_scoring": False,
    }.items():
        assert_equal(candidate.get(key), expected, f"manifest/candidate/{key}")
    assert_equal(candidate.get("source_measurement_m"),
                 "carrier_phase_cycles * retained_wavelength_m + satellite_clock_m",
                 "manifest/candidate/source_measurement")
    routes = manifest.get("routes")
    if not isinstance(routes, list) or [item.get("dataset_id") for item in routes] != list(ROUTES):
        raise fail("manifest route order/count changed")
    for record in routes:
        route = record.get("dataset_id")
        if route not in DOMAIN_ROWS:
            raise fail(f"unknown manifest route: {route}")
        for key, expected in {
            "official_setting_type": "Highway",
            "expected_tdcp_huber_k": 0.5,
            "domain_rows": DOMAIN_ROWS[route],
            "expected_problem_epochs": PROBLEM_EPOCHS[route],
            "expected_output_epochs": PROBLEM_EPOCHS[route],
            "runs": 1,
        }.items():
            assert_equal(record.get(key), expected, f"manifest/{route}/{key}")
        validate_command(route, record.get("command"))
        raw = record.get("raw_inputs")
        if not isinstance(raw, dict) or set(raw) != set(RAW_NAMES):
            raise fail(f"manifest/{route}/raw_inputs must name only raw GNSS/IMU/nav")
        for name in RAW_NAMES:
            item = raw[name]
            if not isinstance(item, dict):
                raise fail(f"manifest/{route}/{name} metadata missing")
            for key, expected in {
                "source_manifest": "sealed Phase118 manifest route raw_inputs",
                "payload_read_before_authorization": False,
                "copy_or_transform": False,
            }.items():
                assert_equal(item.get(key), expected, f"manifest/{route}/{name}/{key}")
            if not isinstance(item.get("placeholder"), str):
                raise fail(f"manifest/{route}/{name}/placeholder missing")
        base = record.get("base_input")
        if not isinstance(base, dict):
            raise fail(f"manifest/{route}/base_input missing")
        for key, expected in {
            "source_manifest": "sealed Phase118 manifest route base_input",
            "raw_rinex_only": True,
            "payload_read_before_authorization": False,
            "hash_read_before_authorization": False,
            "copy_or_transform": False,
        }.items():
            assert_equal(base.get(key), expected, f"manifest/{route}/base/{key}")
    raw_contract = manifest.get("raw_input_contract")
    if not isinstance(raw_contract, dict):
        raise fail("manifest/raw_input_contract missing")
    for key, expected in {
        "input_names_exact": list(RAW_NAMES),
        "phone_gnss_only": True,
        "phone_imu_only": True,
        "broadcast_navigation_only": True,
        "sealed_raw_base_rinex_only": True,
        "raw_paths_materialized_exactly_after_authorization": True,
        "raw_content_copied_or_transformed": False,
        "truth_mat_pdc_precomputed_coordinate_phone_coordinate_accuracy_kaggle": False,
    }.items():
        assert_equal(raw_contract.get(key), expected, f"manifest/raw_input_contract/{key}")
    gates = manifest.get("structural_gates")
    if not isinstance(gates, dict):
        raise fail("manifest/structural_gates missing")
    for key in (
        "default_off_legacy_isolation",
        "phase120_selector_active_exactly_once",
        "phase118_huber_selector_active_exactly_once",
        "phase117_dynamic_sigma_forbidden",
        "fixed_tdcp_sigma_0_03_m",
        "official_type_k_exact",
        "ordinary_tdcp_only",
        "source_measurement_exact",
        "pair_key_and_gate_unchanged",
        "tdcp_factor_and_reject_counts_unchanged",
        "tdcp_factor_equation_units_order_unchanged",
        "gnss_first_accepted_progress",
        "gnss_first_finite_strict_cost_decrease",
        "gnss_first_full_finite_c7_d_exact_handoff",
        "main_qr_selected_accepted_progress",
        "main_finite_strict_cost_decrease",
        "position_clock_finite_expected_coverage",
        "earth_valid_output_coverage",
        "base_correction_exactly_once",
        "pixel5_offset_exactly_once_final_boundary",
        "no_solver_fallback",
        "no_truth_mat_pdc_precomputed_coordinate_accuracy_kaggle",
        "solution_opaque_withheld",
    ):
        assert_equal(gates.get(key), True, f"manifest/structural_gates/{key}")
    matrix = manifest.get("matrix")
    if not isinstance(matrix, dict):
        raise fail("manifest/matrix missing")
    for key, expected in {
        "candidate_count": 1,
        "route_count": 2,
        "runs_per_route": 1,
        "native_solver_invocations_planned": 2,
        "controls": 0,
        "reruns": 0,
        "fallbacks": 0,
        "truth_reads_planned": 0,
        "mat_reads_or_generated_planned": 0,
        "phone_coordinate_reads_planned": 0,
        "precomputed_coordinate_reads_planned": 0,
        "pdc_reads_planned": 0,
        "accuracy_calculations_planned": 0,
        "solution_rows_authorized": False,
    }.items():
        assert_equal(matrix.get(key), expected, f"manifest/matrix/{key}")
    accounting = manifest.get("read_accounting_before_authorization")
    if not isinstance(accounting, dict):
        raise fail("manifest/read_accounting_before_authorization missing")
    for key in (
        "raw_phone_gnss_reads",
        "raw_phone_imu_reads",
        "broadcast_navigation_reads",
        "raw_base_rinex_reads",
        "raw_base_hash_reads",
        "native_solver_invocations",
        "solution_rows_opened",
        "solution_coordinate_interpretations",
        "truth_reads",
        "mat_reads_or_generated",
        "phone_coordinate_reads",
        "precomputed_coordinate_reads",
        "pdc_reads",
        "accuracy_calculations",
        "kaggle_or_token_access",
        "route_reruns",
        "fallbacks",
    ):
        assert_equal(accounting.get(key), 0, f"manifest/read_accounting/{key}")
    assert_equal(accounting.get("raw_content_copied_or_transformed"), False,
                 "manifest/read_accounting/raw_copy")
    return manifest


def verify_pre_raw() -> dict[str, Any]:
    freeze = verify_freeze()
    manifest = verify_manifest()
    return {
        "status": "pre-raw-verified-no-payload-activity",
        "phase": 120,
        "execution_label": "Luna Max",
        "candidate_count": 1,
        "route_ids": list(ROUTES),
        "runs_per_route": 1,
        "raw_execution_authorized": False,
        "raw_phone_gnss_reads": 0,
        "raw_phone_imu_reads": 0,
        "broadcast_navigation_reads": 0,
        "raw_base_rinex_reads": 0,
        "raw_base_hash_reads": 0,
        "native_solver_invocations": 0,
        "solution_rows_opened": 0,
        "solution_coordinate_interpretations": 0,
        "truth_reads": 0,
        "mat_reads_or_generated": 0,
        "phone_coordinate_reads": 0,
        "precomputed_coordinate_reads": 0,
        "pdc_reads": 0,
        "accuracy_calculations": 0,
        "kaggle_or_token_access": 0,
        "route_reruns": 0,
        "fallbacks": 0,
        "solution_output_published": False,
        "freeze_commit": FREEZE_COMMIT,
        "freeze_sha256": FREEZE_SHA256,
        "implementation_commit": IMPLEMENTATION_COMMIT,
        "manifest_sha256": sha256_file(MANIFEST, "Phase120 structural manifest"),
        "freeze_status": freeze["status"],
        "manifest_status": manifest["status"],
    }


def main() -> int:
    import argparse

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--verify-freeze", action="store_true")
    parser.add_argument("--verify-manifest", action="store_true")
    parser.add_argument("--verify-pre-raw", action="store_true")
    args = parser.parse_args()
    if not any((args.verify_freeze, args.verify_manifest, args.verify_pre_raw)):
        parser.error("one launch-free verification mode is required")
    try:
        if args.verify_freeze:
            verify_freeze()
        if args.verify_manifest:
            verify_manifest()
        if args.verify_pre_raw:
            print(json.dumps(verify_pre_raw(), indent=2, sort_keys=True))
        return 0
    except (Phase120ContractError, OSError) as exc:
        print(f"phase120 pre-raw validator: fail-closed: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
