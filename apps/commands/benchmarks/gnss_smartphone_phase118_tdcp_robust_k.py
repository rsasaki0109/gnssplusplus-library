#!/usr/bin/env python3
"""Launch-free Phase118 structural contract validator.

This module verifies only tracked source and sealed JSON metadata.  In every
verification mode it refuses to stat, hash, or open raw phone/base members,
solutions, truth, MAT, PDC, or coordinate payloads.  A later independent raw
authorization may pin an execution wrapper separately; this contract itself
has no raw or solver authority.
"""

from __future__ import annotations

import hashlib
import json
from pathlib import Path
import sys
from typing import Any


ROOT = Path(__file__).resolve().parents[3]
SOURCE_PARITY_FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_phase118_tdcp_robust_k_source_parity_freeze_v1.json"
FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_phase118_tdcp_robust_k_structural_contract_freeze_v1.json"
CONTRACT_AUDIT = ROOT / "docs/use_cases/records/smartphone_r5_phase118_tdcp_robust_k_structural_contract_audit_v1.md"
MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase118_tdcp_robust_k_structural_manifest_v1.json"
PHASE112_MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase112_main_output_offset_manifest_v1.json"
PHASE112_RESULT = ROOT / "docs/use_cases/records/smartphone_r5_phase112_main_output_offset_structural_result_v1.json"
PHASE117_RESULT = ROOT / "docs/use_cases/records/smartphone_r5_phase117_tdcp_weighting_structural_result_v1.json"
WRAPPER = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase118_tdcp_robust_k_structural_execute.py"
FOCUSED_TESTS = ROOT / "tests/test_smartphone_phase118_tdcp_robust_k_structural_execution.py"
APP = ROOT / "apps/native/gnss_fgo_imu_no_base.cpp"
FGO_HEADER = ROOT / "include/libgnss++/algorithms/fgo.hpp"
FGO_CONFIG = ROOT / "include/libgnss++/algorithms/fgo_config.hpp"
EIGEN = ROOT / "src/algorithms/fgo.cpp"
GTSAM = ROOT / "src/algorithms/fgo_gtsam_backend.cpp"
FGO_TEST = ROOT / "tests/test_fgo.cpp"
BINARY = ROOT / "build/apps/gnss_fgo_imu_no_base"

FREEZE_SHA256 = "267a2f51dea06f7d29d5b27be1f7ffd84febac1c72a9fbe2676c5f5a2649f81b"
FREEZE_COMMIT = "25f3bada02c4073175054b5d668de02020761208"
SOURCE_PARITY_FREEZE_SHA256 = "c6f4fda2abe170417610d4fd1a4ae8a1a618d07096ff0432669081ab06a1cf77"
SOURCE_PARITY_FREEZE_COMMIT = "5fcc06ab64189dd5dfb8001664bdb8496f85224f"
CONTRACT_AUDIT_COMMIT = "c7da3d9ab80b13dbc5b1715b27c1b03d9509b974"
CONTRACT_AUDIT_SHA256 = "601b3c2dbe3e42e0125d20f70e86a6c9e295e69b46f4c64bff74eb4638bf43bf"
IMPLEMENTATION_COMMIT = "7f339ccc8f0fb58e3dbcb7f3fc24ba2b04acbf40"
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
SELECTOR = "--native-phase118-official-tdcp-huber-k"
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
    "--native-upstream-position-offset",
    SELECTOR,
)
FORBIDDEN_FLAGS = (
    "--obs",
    PHASE117_SELECTOR,
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
    "--native-phase116-carrier-tdcp-incidence-diagnostic",
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
SCHEMA = "smartphone-r5-phase118-tdcp-robust-k-structural-manifest.v1"
CANDIDATE_ID = "phase118-official-tdcp-huber-k-mapping-v1"


class Phase118ContractError(ValueError):
    """Raised when the frozen Phase118 contract fails closed."""


def fail(message: str) -> Phase118ContractError:
    return Phase118ContractError(message)


def relative(path: Path) -> str:
    try:
        return str(path.relative_to(ROOT))
    except ValueError:
        return str(path)


def assert_equal(actual: Any, expected: Any, label: str) -> None:
    if actual != expected:
        raise fail(f"{label}: expected {expected!r}, got {actual!r}")


def sha256_file(path: Path, label: str) -> str:
    # This guard is deliberately before is_file/open/stat.  Pre-raw
    # verification must not touch any payload or solution member.
    if (
        path.name in RAW_NAMES
        or path.name == "base.obs"
        or path.name.endswith("withheld_solution_output.csv")
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


def verify_freeze() -> dict[str, Any]:
    assert_equal(sha256_file(FREEZE, "Phase118 structural contract freeze"), FREEZE_SHA256,
                 "freeze/sha256")
    freeze = read_json(FREEZE, "Phase118 structural contract freeze")
    for key, expected in {
        "schema_version": "smartphone-r5-phase118-tdcp-robust-k-structural-contract-freeze.v1",
        "phase": 118,
        "execution_label": "Luna Max",
        "status": "sealed-before-phase118-raw-execution",
    }.items():
        assert_equal(freeze.get(key), expected, f"freeze/{key}")
    audit = freeze.get("audit")
    if not isinstance(audit, dict):
        raise fail("freeze/audit missing")
    for key, expected in {
        "path": relative(CONTRACT_AUDIT),
        "commit": CONTRACT_AUDIT_COMMIT,
        "sha256": CONTRACT_AUDIT_SHA256,
    }.items():
        assert_equal(audit.get(key), expected, f"freeze/audit/{key}")
    assert_equal(sha256_file(CONTRACT_AUDIT, "Phase118 structural audit"),
                 CONTRACT_AUDIT_SHA256, "freeze/audit/file_sha256")
    source_parity = freeze.get("source_parity_freeze")
    if not isinstance(source_parity, dict):
        raise fail("freeze/source_parity_freeze missing")
    for key, expected in {
        "path": relative(SOURCE_PARITY_FREEZE),
        "commit": SOURCE_PARITY_FREEZE_COMMIT,
        "sha256": SOURCE_PARITY_FREEZE_SHA256,
        "raw_execution_authorized": False,
    }.items():
        assert_equal(source_parity.get(key), expected,
                     f"freeze/source_parity_freeze/{key}")
    assert_equal(sha256_file(SOURCE_PARITY_FREEZE, "Phase118 source-parity freeze"),
                 SOURCE_PARITY_FREEZE_SHA256, "freeze/source_parity/file_sha256")
    candidate = freeze.get("candidate")
    if not isinstance(candidate, dict):
        raise fail("freeze/candidate missing")
    for key, expected in {
        "id": CANDIDATE_ID,
        "selector": SELECTOR,
        "opt_in": True,
        "default_off": True,
        "candidate_count": 1,
        "solution_withheld": True,
        "truth_evaluation": False,
        "accuracy_scoring": False,
    }.items():
        assert_equal(candidate.get(key), expected, f"freeze/candidate/{key}")
    assert_equal(candidate.get("recipe_anchor"),
                 "Phase112 champion recipe plus only the Phase118 ordinary-TDCP Huber-k mapping",
                 "freeze/candidate/recipe_anchor")
    official = candidate.get("official_huber_contract")
    if not isinstance(official, dict):
        raise fail("freeze/official_huber_contract missing")
    for key, expected in {
        "Street": 0.2,
        "Mix": 0.2,
        "Highway": 0.5,
        "other_official_type": 0.5,
        "threshold_units": "whitened residual sigma units",
        "selection_input": "official source setting.Type only",
    }.items():
        assert_equal(official.get(key), expected, f"freeze/official/{key}")
    noise = candidate.get("noise_contract")
    if not isinstance(noise, dict):
        raise fail("freeze/noise_contract missing")
    for key, expected in {
        "tdcp_sigma_m": 0.03,
        "phase117_dynamic_sigma": False,
        "existing_native_threshold_when_selector_off": 4.0,
        "residual_units": "metres",
        "factor_count_and_insertion": "unchanged",
        "pair_key": "(satellite,signal)",
        "reject_predicate": "unchanged gap, clock, loss-of-lock, nonfinite, and code-phase gates",
        "other_robust_kernels": "unchanged",
    }.items():
        assert_equal(noise.get(key), expected, f"freeze/noise/{key}")
    preserved = candidate.get("preserved_paths")
    if not isinstance(preserved, dict):
        raise fail("freeze/preserved_paths missing")
    for key in (
        "c7_d_c0d_handoff",
        "phase99_qr",
        "raw_base_correction",
        "pixel5_final_offset",
        "imu_filter_lm_initialization",
        "equations_units_ordering",
        "legacy_default",
    ):
        assert_equal(preserved.get(key), True, f"freeze/preserved_paths/{key}")
    assert_equal(official.get("unknown_type"), "fail-closed",
                 "freeze/official/unknown_type")
    matrix = freeze.get("matrix")
    if not isinstance(matrix, dict):
        raise fail("freeze/matrix missing")
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
        "solution_rows_authorized": False,
    }.items():
        assert_equal(matrix.get(key), expected, f"freeze/matrix/{key}")
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
        raise fail("implementation/source_sha256 missing")
    paths = {
        "apps/native/gnss_fgo_imu_no_base.cpp": APP,
        "include/libgnss++/algorithms/fgo.hpp": FGO_HEADER,
        "include/libgnss++/algorithms/fgo_config.hpp": FGO_CONFIG,
        "src/algorithms/fgo.cpp": EIGEN,
        "src/algorithms/fgo_gtsam_backend.cpp": GTSAM,
        "tests/test_fgo.cpp": FGO_TEST,
        "tests/test_smartphone_phase118_tdcp_robust_k.py": ROOT / "tests/test_smartphone_phase118_tdcp_robust_k.py",
        "tests/CMakeLists.txt": ROOT / "tests/CMakeLists.txt",
    }
    actual: dict[str, str] = {}
    for name, path in paths.items():
        expected = source_pins.get(name)
        if not isinstance(expected, str):
            raise fail(f"missing implementation source pin: {name}")
        actual[name] = sha256_file(path, f"implementation/{name}")
        assert_equal(actual[name], expected, f"implementation/{name}/sha256")
    binary = implementation.get("binary")
    if not isinstance(binary, dict):
        raise fail("implementation/binary missing")
    assert_equal(binary.get("path"), relative(BINARY), "implementation/binary/path")
    assert_equal(sha256_file(BINARY, "implementation/binary"), binary.get("sha256"),
                 "implementation/binary/sha256")
    markers = {
        APP: (SELECTOR, "native_phase118_official_tdcp_huber_k", "fixed_sigma_m"),
        FGO_CONFIG: ("use_official_tdcp_huber_k", "official_tdcp_setting_type",
                     "resolveOfficialTdcpHuberThresholdSigmaForType"),
        EIGEN: ("ordinary_tdcp_huber_threshold_sigma",),
        GTSAM: ("ordinary_tdcp_huber_threshold_sigma",),
        FGO_TEST: ("OfficialTypeMappingIsOptInAndFailClosed",),
    }
    for path, required in markers.items():
        source = path.read_text(encoding="utf-8")
        for marker in required:
            if marker not in source:
                raise fail(f"implementation marker missing: {relative(path)}:{marker}")
    return actual


def sealed_phase112_routes() -> dict[str, dict[str, Any]]:
    phase112 = read_json(PHASE112_MANIFEST, "sealed Phase112 manifest")
    assert_equal(sha256_file(PHASE112_MANIFEST, "sealed Phase112 manifest"),
                 PHASE112_MANIFEST_SHA256, "Phase112 manifest/sha256")
    routes = phase112.get("routes")
    if not isinstance(routes, list) or [item.get("dataset_id") for item in routes] != list(ROUTES):
        raise fail("sealed Phase112 route order changed")
    result: dict[str, dict[str, Any]] = {}
    for item in routes:
        route = item.get("dataset_id")
        if route not in ROUTES:
            raise fail(f"unexpected Phase112 route: {route}")
        # Only metadata objects are inspected; no member path is touched.
        for name in RAW_NAMES:
            pin = item.get("raw_inputs", {}).get(name)
            if not isinstance(pin, dict) or not isinstance(pin.get("path"), str):
                raise fail(f"missing sealed raw metadata: {route}/{name}")
            if pin.get("read_at_manifest_creation") is not False:
                raise fail(f"raw metadata read boundary changed: {route}/{name}")
        base = item.get("base_input")
        if not isinstance(base, dict) or not isinstance(base.get("path"), str):
            raise fail(f"missing sealed base metadata: {route}")
        if base.get("read_at_manifest_creation") is not False or base.get("hash_read_at_manifest_creation") is not False:
            raise fail(f"base metadata read boundary changed: {route}")
        result[route] = item
    return result


def command_template(route: str) -> list[str]:
    output_root = "output/smartphone-r5/phase118-tdcp-robust-k-v1/"
    output_route = route.replace("/", "__")
    return [
        "build/apps/gnss_fgo_imu_no_base",
        "--dataset-id", route,
        "--android-gnss", "__PHASE118_RAW_DEVICE_GNSS__",
        "--android-imu", "__PHASE118_RAW_DEVICE_IMU__",
        "--nav", "__PHASE118_RAW_BROADCAST_NAV__",
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
        SELECTOR,
        "--native-base-rinex", "__PHASE118_RAW_BASE_RINEX__",
        "--native-base-rinex-sha256", "__PHASE118_RAW_BASE_SHA256__",
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
    assert_equal(command.count(SELECTOR), 1, f"command/{route}/Phase118 selector")
    assert_equal(command.count(PHASE117_SELECTOR), 0, f"command/{route}/Phase117 selector")
    placeholders = {
        "--android-gnss": "__PHASE118_RAW_DEVICE_GNSS__",
        "--android-imu": "__PHASE118_RAW_DEVICE_IMU__",
        "--nav": "__PHASE118_RAW_BROADCAST_NAV__",
        "--native-base-rinex": "__PHASE118_RAW_BASE_RINEX__",
        "--native-base-rinex-sha256": "__PHASE118_RAW_BASE_SHA256__",
    }
    for flag, placeholder in placeholders.items():
        assert_equal(command.count(flag), 1, f"command/{route}/{flag}")
        assert_equal(command[command.index(flag) + 1], placeholder,
                     f"command/{route}/{flag}/placeholder")
    if "--obs" in command:
        raise fail(f"legacy obs path present: {route}")


def verify_manifest() -> dict[str, Any]:
    freeze = verify_freeze()
    implementation = verify_implementation(freeze)
    source_routes = sealed_phase112_routes()
    manifest = read_json(MANIFEST, "Phase118 structural manifest")
    for key, expected in {
        "schema_version": SCHEMA,
        "phase": 118,
        "execution_label": "Luna Max",
        "status": "sealed-before-phase118-raw-execution",
    }.items():
        assert_equal(manifest.get(key), expected, f"manifest/{key}")
    freeze_pin = manifest.get("freeze")
    if not isinstance(freeze_pin, dict):
        raise fail("manifest/freeze missing")
    for key, expected in {
        "path": relative(FREEZE),
        "sha256": FREEZE_SHA256,
        "commit": FREEZE_COMMIT,
        "raw_execution_authorized_before_manifest": False,
    }.items():
        assert_equal(freeze_pin.get(key), expected, f"manifest/freeze/{key}")
    audit_pin = manifest.get("contract_audit")
    if not isinstance(audit_pin, dict):
        raise fail("manifest/contract_audit missing")
    for key, expected in {
        "path": relative(CONTRACT_AUDIT),
        "commit": CONTRACT_AUDIT_COMMIT,
        "sha256": CONTRACT_AUDIT_SHA256,
    }.items():
        assert_equal(audit_pin.get(key), expected, f"manifest/contract_audit/{key}")
    impl = manifest.get("implementation")
    if not isinstance(impl, dict):
        raise fail("manifest/implementation missing")
    for key, expected in {
        "commit": IMPLEMENTATION_COMMIT,
        "candidate_id": CANDIDATE_ID,
        "legacy_default_unchanged": True,
        "graph_factors_values_equations_units_unchanged": True,
        "tdcp_sigma_m_fixed": 0.03,
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
    assert_equal(binary.get("path"), relative(BINARY), "manifest/binary/path")
    assert_equal(binary.get("sha256"),
                 sha256_file(BINARY, "manifest/implementation/binary"),
                 "manifest/binary/sha256")
    artifacts = manifest.get("artifacts")
    if not isinstance(artifacts, dict):
        raise fail("manifest/artifacts missing")
    for key, path in (("validator", Path(__file__)), ("runner", WRAPPER), ("focused_tests", FOCUSED_TESTS)):
        pin = artifacts.get(key)
        if not isinstance(pin, dict):
            raise fail(f"manifest/artifacts/{key} missing")
        assert_equal(pin.get("path"), relative(path), f"manifest/artifacts/{key}/path")
        expected_hash = pin.get("sha256")
        if not isinstance(expected_hash, str) or len(expected_hash) != 64:
            raise fail(f"manifest/artifacts/{key}/sha256 missing")
        assert_equal(sha256_file(path, f"manifest/artifacts/{key}"), expected_hash,
                     f"manifest/artifacts/{key}/sha256")
    candidate = manifest.get("candidate")
    if not isinstance(candidate, dict):
        raise fail("manifest/candidate missing")
    for key, expected in {
        "id": CANDIDATE_ID,
        "selector": SELECTOR,
        "candidate_count": 1,
        "opt_in": True,
        "default_off": True,
        "phase117_dynamic_sigma": False,
        "fixed_tdcp_sigma_m": 0.03,
        "official_highway_k": 0.5,
        "official_street_k": 0.2,
        "official_mix_k": 0.2,
        "factor_count_invariance": True,
        "solution_withheld": True,
        "truth_evaluation": False,
        "accuracy_scoring": False,
    }.items():
        assert_equal(candidate.get(key), expected, f"manifest/candidate/{key}")
    selectors = candidate.get("selectors")
    if not isinstance(selectors, list):
        raise fail("manifest/candidate/selectors missing")
    for flag in (*PHASE112_SELECTORS, *BASE_SELECTORS, SELECTOR):
        assert_equal(selectors.count(flag), 1, f"manifest/candidate/selectors/{flag}")
    assert_equal(selectors.count(PHASE117_SELECTOR), 0,
                 "manifest/candidate/selectors/Phase117")
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
            if not isinstance(item, dict) or item.get("payload_read_before_authorization") is not False or item.get("copy_or_transform") is not False:
                raise fail(f"manifest/{route}/{name} pre-raw boundary changed")
        base = record.get("base_input")
        if not isinstance(base, dict):
            raise fail(f"manifest/{route}/base_input missing")
        for key, expected in {
            "raw_rinex_only": True,
            "payload_read_before_authorization": False,
            "hash_read_before_authorization": False,
            "copy_or_transform": False,
        }.items():
            assert_equal(base.get(key), expected, f"manifest/{route}/base/{key}")
        assert_equal(base.get("source_manifest"), "sealed Phase112 manifest route base_input",
                     f"manifest/{route}/base/source")
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
        "truth_mat_phone_coordinate_precomputed_coordinate_pdc_accuracy_kaggle": False,
    }.items():
        assert_equal(raw_contract.get(key), expected, f"manifest/raw_input_contract/{key}")
    gates = manifest.get("structural_gates")
    if not isinstance(gates, dict):
        raise fail("manifest/structural_gates missing")
    for key in (
        "default_off_legacy_isolation",
        "selector_active_exactly_once",
        "route_type_and_huber_k_exact",
        "fixed_tdcp_sigma_0_03_m",
        "phase117_dynamic_sigma_forbidden",
        "tdcp_factor_equation_units_key_order_unchanged",
        "tdcp_factor_and_reject_counts_unchanged",
        "ordinary_tdcp_only_no_graph_family_change",
        "gnss_first_attempted_and_accepted_progress",
        "gnss_first_finite_strict_cost_decrease",
        "gnss_first_full_finite_c7_d_exact_handoff",
        "main_qr_selected_and_accepted_progress",
        "main_finite_strict_cost_decrease",
        "position_clock_finite_expected_coverage",
        "earth_valid_output_coverage",
        "base_correction_exactly_once",
        "pixel5_offset_exactly_once_final_boundary",
        "no_solver_fallback",
        "no_truth_mat_precomputed_coordinate_pdc_accuracy_kaggle",
        "solution_withheld",
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
                 "manifest/read_accounting/copy")
    return manifest


def verify_pre_raw() -> dict[str, Any]:
    freeze = verify_freeze()
    manifest = verify_manifest()
    # Deliberately do not inspect any route payload path.  These counters are
    # a contract assertion, not a claim that payload members were available.
    return {
        "status": "pre-raw-verified",
        "phase": 118,
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
        "freeze_sha256": FREEZE_SHA256,
        "freeze_commit": FREEZE_COMMIT,
        "implementation_commit": IMPLEMENTATION_COMMIT,
        "manifest_sha256": sha256_file(MANIFEST, "Phase118 manifest"),
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
    except (Phase118ContractError, OSError) as exc:
        print(f"phase118 pre-raw validator: fail-closed: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
