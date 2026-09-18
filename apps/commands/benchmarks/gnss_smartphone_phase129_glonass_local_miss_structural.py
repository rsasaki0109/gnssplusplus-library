#!/usr/bin/env python3
"""Launch-free Phase129 inventory and structural-contract validator.

Only tracked source, sealed contract JSON/Markdown, and the target binary
identity are inspected.  This module deliberately has no input materializer
or native-process launcher.  A later independently authorized runner may
reuse the pure in-memory validators after its one-pass inventory.
"""

from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
import sys
from typing import Any


ROOT = Path(__file__).resolve().parents[3]
AUDIT = ROOT / "docs/use_cases/records/smartphone_r5_phase129_glonass_local_miss_structural_contract_audit_v1.md"
FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_phase129_glonass_local_miss_structural_contract_freeze_v1.json"
MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase129_glonass_local_miss_structural_manifest_v1.json"
RUNNER = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase129_glonass_local_miss_structural_execute.py"
FOCUSED_TESTS = ROOT / "tests/test_smartphone_phase129_glonass_local_miss_structural.py"
BINARY = ROOT / "build/apps/gnss_fgo_imu_no_base"

AUDIT_COMMIT = "4e6634a694da571b15ab7a98d001756cd93d8e75"
AUDIT_SHA256 = "385e9a531881d9ac7fdfe4f7bfba8b9eabcab074a7e4635a14d1d7b2112e881a"
FREEZE_COMMIT = "4b1c32eae1d47f80c32e2ea73a885e179fe5389b"
FREEZE_SHA256 = "3bdc75dedb6a4bfc101bd9907acff6f75059327e1123171d0b4ba34cee507475"
IMPLEMENTATION_COMMIT = "05e57d5008320734de93c763ff84ccd31f755805"
TARGET_BINARY_SHA256 = "20a5b8d96a0c024757f3b8d2948595e857023ecf25e2c3cbcc295d44907f0888"

PHASE126_SELECTOR = "--native-phase126-raw-base-source-complete"
PHASE127_SELECTOR = "--native-phase127-glonass-channel-provenance"
PHASE128_SELECTOR = "--native-phase128-glonass-provenance-parser-admission"
PHASE129_SELECTOR = "--native-phase129-glonass-local-miss-mask"
PHASE118_SELECTOR = "--native-phase118-official-tdcp-huber-k"
PHASE117_SELECTOR = "--native-phase117-tdcp-snr-type-sigma"
PHASE120_SELECTOR = "--native-phase120-official-tdcp-resl-atmosphere-cancellation"
ADDITIONAL_SELECTOR = "--native-base-pseudorange-preserve-additional-frequency-bands"

ROUTES = (
    "2021-03-16-18-59-us-ca-mtv-a/pixel5",
    "2022-04-01-18-22-us-ca-lax-t/pixel5",
)
DOMAIN_ROWS = {ROUTES[0]: 2158, ROUTES[1]: 1465}
PROBLEM_EPOCHS = {route: rows + 1 for route, rows in DOMAIN_ROWS.items()}
RAW_NAMES = ("device_gnss.csv", "device_imu.csv", "brdc.nav")
SCHEMA = "smartphone-r5-phase129-glonass-local-miss-structural-manifest.v1"
CANDIDATE_ID = "phase129-glonass-certified-local-miss-admission-v1"

IMPLEMENTATION_SOURCES = {
    "apps/native/gnss_fgo_imu_no_base.cpp": ROOT / "apps/native/gnss_fgo_imu_no_base.cpp",
    "include/libgnss++/algorithms/fgo_config.hpp": ROOT / "include/libgnss++/algorithms/fgo_config.hpp",
    "include/libgnss++/algorithms/fgo.hpp": ROOT / "include/libgnss++/algorithms/fgo.hpp",
    "include/libgnss++/algorithms/base_pseudorange_compensation.hpp": ROOT / "include/libgnss++/algorithms/base_pseudorange_compensation.hpp",
    "include/libgnss++/algorithms/phase129_glonass_local_miss.hpp": ROOT / "include/libgnss++/algorithms/phase129_glonass_local_miss.hpp",
    "src/algorithms/base_pseudorange_compensation.cpp": ROOT / "src/algorithms/base_pseudorange_compensation.cpp",
    "src/algorithms/fgo_internal.hpp": ROOT / "src/algorithms/fgo_internal.hpp",
    "src/algorithms/fgo_problems.cpp": ROOT / "src/algorithms/fgo_problems.cpp",
    "src/algorithms/source_pseudorange_miss_mask.cpp": ROOT / "src/algorithms/source_pseudorange_miss_mask.cpp",
}
SOURCE_SHA256 = {
    "apps/native/gnss_fgo_imu_no_base.cpp": "76fa5638f56b77ce69c185f384c12dcf7bfa9b59e8cc9bcb41ff3bb096e3f6ec",
    "include/libgnss++/algorithms/fgo_config.hpp": "eed0d551ebfc8eec73ff3abb6cfe70ac3a9f7669476d661c24596b05baf8c9ff",
    "include/libgnss++/algorithms/fgo.hpp": "d2794573f15a1b4ac569ecb97869c9a5bfcbd519ae471472aaaa0a3d6f167ce6",
    "include/libgnss++/algorithms/base_pseudorange_compensation.hpp": "87a7d86a01b4f0c43abda39f7f22f2d73bee502228b2cd05802a3e2820e78566",
    "include/libgnss++/algorithms/phase129_glonass_local_miss.hpp": "c3dc7bc699a0bf95d960b045ec08887f13ba757ddccb32ac5dda4354b72d4abe",
    "src/algorithms/base_pseudorange_compensation.cpp": "716ce1daa694b540b55687255097fe4de2fcdbbae0351c4d183e831ed2753ad2",
    "src/algorithms/fgo_internal.hpp": "175b6cbe699c455793af1eb13ad6fdab933b1d9770d9ee7ef5b44318e29dcaed",
    "src/algorithms/fgo_problems.cpp": "5802eb24c34c3bc954ffd4bdf980bb51a59124b64379795b7867518c577994e5",
    "src/algorithms/source_pseudorange_miss_mask.cpp": "ed44bcf156147d4ff612cbe939c99371a998885ada3f206bb9657f2f72bb0591",
}


class Phase129ContractError(ValueError):
    """A contract violation that must fail closed."""


def fail(message: str) -> Phase129ContractError:
    return Phase129ContractError(message)


def relative(path: Path) -> str:
    return str(path.relative_to(ROOT))


def assert_equal(actual: Any, expected: Any, label: str) -> None:
    if actual != expected:
        raise fail(f"{label}: expected {expected!r}, got {actual!r}")


def assert_true(value: Any, label: str) -> None:
    if value is not True:
        raise fail(f"{label}: expected true")


def nonnegative_int(value: Any, label: str) -> int:
    if not isinstance(value, int) or isinstance(value, bool) or value < 0:
        raise fail(f"{label}: expected nonnegative integer")
    return value


def finite_number(value: Any, label: str) -> float:
    if not isinstance(value, (int, float)) or isinstance(value, bool):
        raise fail(f"{label}: expected finite number")
    result = float(value)
    if not math.isfinite(result):
        raise fail(f"{label}: expected finite number")
    return result


def sha256_static(path: Path, label: str) -> str:
    lowered = path.name.lower()
    if (lowered in RAW_NAMES or lowered in {"base.obs", "truth.csv", "ground_truth.csv"}
            or lowered.endswith((".csv", ".nav", ".obs", ".mat"))
            or "truth" in lowered or "ground_truth" in lowered):
        raise fail(f"payload hash forbidden in launch-free validator: {label}")
    if not path.is_file():
        raise fail(f"missing static artifact: {label}: {path}")
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def read_json(path: Path, label: str) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise fail(f"failed to read {label}: {exc}") from exc
    if not isinstance(value, dict):
        raise fail(f"{label} is not an object")
    return value


def zero_accounting(value: Any, label: str) -> None:
    if not isinstance(value, dict):
        raise fail(f"{label} missing")
    for key, item in value.items():
        if key.endswith("copied_or_transformed"):
            assert_equal(item, False, f"{label}/{key}")
        else:
            assert_equal(item, 0, f"{label}/{key}")


def verify_freeze() -> dict[str, Any]:
    assert_equal(sha256_static(AUDIT, "Phase129 structural audit"), AUDIT_SHA256,
                 "freeze/audit_sha256")
    freeze = read_json(FREEZE, "Phase129 structural freeze")
    for key, expected in {
        "schema_version": "smartphone-r5-phase129-glonass-local-miss-structural-contract-freeze.v1",
        "phase": 129,
        "execution_label": "Luna Max",
        "status": "sealed-before-inventory-raw-execution",
    }.items():
        assert_equal(freeze.get(key), expected, f"freeze/{key}")
    audit = freeze.get("contract_audit")
    if not isinstance(audit, dict):
        raise fail("freeze/contract_audit missing")
    for key, expected in {"path": relative(AUDIT), "commit": AUDIT_COMMIT,
                          "sha256": AUDIT_SHA256}.items():
        assert_equal(audit.get(key), expected, f"freeze/contract_audit/{key}")
    implementation = freeze.get("implementation")
    if not isinstance(implementation, dict):
        raise fail("freeze/implementation missing")
    for key, expected in {
        "commit": IMPLEMENTATION_COMMIT,
        "candidate_id": CANDIDATE_ID,
        "selector": PHASE129_SELECTOR,
        "required_selectors": [PHASE126_SELECTOR, PHASE127_SELECTOR,
                                PHASE128_SELECTOR, PHASE129_SELECTOR,
                                PHASE118_SELECTOR],
        "phase117_dynamic_sigma": False,
        "phase120_selector": False,
        "additional_frequency_selector": False,
        "main_solver": "MULTIFRONTAL_QR",
        "fixed_tdcp_sigma_m": 0.03,
        "official_tdcp_huber_k": 0.5,
        "default_off": True,
        "legacy_default_unchanged": True,
        "partial_selectors_allowed": False,
        "factor_topology_unchanged": True,
        "equations_units_sigma_filter_lm_unchanged": True,
        "no_fallback": True,
        "no_extrapolation": True,
        "no_zero_correction": True,
        "no_raw_uncorrected_retention": True,
    }.items():
        assert_equal(implementation.get(key), expected, f"freeze/implementation/{key}")
    assert_equal(implementation.get("source_sha256"), SOURCE_SHA256,
                 "freeze/implementation/source_sha256")
    target = implementation.get("target_binary")
    if not isinstance(target, dict):
        raise fail("freeze/implementation/target_binary missing")
    assert_equal(target.get("path"), relative(BINARY), "freeze/target/path")
    assert_equal(target.get("sha256"), TARGET_BINARY_SHA256, "freeze/target/sha256")
    candidate = freeze.get("candidate")
    if not isinstance(candidate, dict):
        raise fail("freeze/candidate missing")
    for key, expected in {
        "candidate_count": 1,
        "candidate_id": CANDIDATE_ID,
        "implementation_authorized": True,
        "raw_execution_authorized": False,
        "solver_execution_authorized": False,
        "truth_evaluation_authorized": False,
        "accuracy_authorized": False,
        "solution_publication_authorized": False,
        "kaggle_authorized": False,
        "source_backed": True,
    }.items():
        assert_equal(candidate.get(key), expected, f"freeze/candidate/{key}")
    inventory = freeze.get("inventory_stage")
    if not isinstance(inventory, dict):
        raise fail("freeze/inventory_stage missing")
    for key, expected in {
        "stage": "post-independent-authorization-pre-solver",
        "materialize_after_authorization_only": True,
        "read_each_permitted_input_once": True,
        "raw_content_copied_or_transformed": False,
        "solver_invocations_before_inventory_pass": 0,
        "failure_status": "inventory-fail-closed",
        "glonass_row_policy": "every eligible row is exactly certified or explicit local miss",
        "shared_ledger_policy": "base correction and shared factor ledgers use the same exact key and row decision",
        "conservation": "input = certified + explicit_local_miss",
        "non_glonass_policy": "retain existing source-complete rows",
        "all_miss_policy": "empty/all-miss usable route fails before solver",
    }.items():
        assert_equal(inventory.get(key), expected, f"freeze/inventory_stage/{key}")
    for section in ("inventory_gates", "stage2_structural_gates"):
        value = freeze.get(section)
        if not isinstance(value, dict):
            raise fail(f"freeze/{section} missing")
        for key, item in value.items():
            if key == "solver_invocations_on_inventory_failure":
                assert_equal(item, 0, f"freeze/{section}/{key}")
            else:
                assert_equal(item, True, f"freeze/{section}/{key}")
    zero_accounting(freeze.get("read_accounting_before_authorization"),
                    "freeze/read_accounting_before_authorization")
    authorization = freeze.get("authorization_boundary")
    if not isinstance(authorization, dict):
        raise fail("freeze/authorization_boundary missing")
    for key in ("raw_materialization_authorized", "inventory_reads_authorized",
                "solver_execution_authorized", "truth_evaluation_authorized",
                "accuracy_authorized", "solution_publication_authorized"):
        assert_equal(authorization.get(key), False, f"freeze/authorization/{key}")
    return freeze


def verify_implementation() -> dict[str, str]:
    actual: dict[str, str] = {}
    for name, path in IMPLEMENTATION_SOURCES.items():
        actual[name] = sha256_static(path, f"implementation/{name}")
        assert_equal(actual[name], SOURCE_SHA256[name], f"implementation/{name}/sha256")
    assert_equal(sha256_static(BINARY, "target binary"), TARGET_BINARY_SHA256,
                 "target binary/sha256")
    markers = {
        ROOT / "apps/native/gnss_fgo_imu_no_base.cpp": (
            PHASE129_SELECTOR, "native_phase129_glonass_local_miss_mask = false",
            "requires a GTSAM build", PHASE126_SELECTOR, PHASE127_SELECTOR,
        ),
        ROOT / "include/libgnss++/algorithms/phase129_glonass_local_miss.hpp": (
            "classify", "rowLedgerConsistent", "local_miss",
        ),
        ROOT / "src/algorithms/base_pseudorange_compensation.cpp": (
            "phase129_glonass_local_miss::classify", "local_miss_stream_keys",
            "Phase129 GLONASS row ledger is inconsistent",
        ),
        ROOT / "src/algorithms/fgo_problems.cpp": (
            "phase129_glonass_local_miss::rowLedgerConsistent",
            "phase129_glonass_factor_rows_dropped",
        ),
    }
    for path, required in markers.items():
        source = path.read_text(encoding="utf-8")
        for marker in required:
            if marker not in source:
                raise fail(f"implementation marker missing: {relative(path)}:{marker}")
    return actual


def command_template(route: str) -> list[str]:
    output_route = route.replace("/", "__")
    output_root = "output/smartphone-r5/phase129-glonass-local-miss-structural-v1/"
    return [
        "build/apps/gnss_fgo_imu_no_base", "--dataset-id", route,
        "--android-gnss", "__PHASE129_RAW_DEVICE_GNSS__",
        "--android-imu", "__PHASE129_RAW_DEVICE_IMU__",
        "--nav", "__PHASE129_RAW_BROADCAST_NAV__",
        "--all-epochs", "--android-raw-utc-keys", "--android-raw-clock-only",
        "--android-utc-wall-clock-fallback", "--native-pdc-imu-tdcp-no-bridge",
        "--native-source-direct-observable-quality", "--native-source-clock-c0d-factor",
        "--native-source-clock-c0d-meter-state-parity", "--native-source-clock-c0d-active-solve-diagnostic",
        "--native-source-clock-c0d-gnss-first-meter-state-handoff",
        "--native-source-clock-c0d-epoch-vector-parity",
        "--native-source-clock-c0d-phase99-main-multifrontal-qr-solver",
        "--native-base-pseudorange-compensation", "--native-base-pseudorange-source-miss-mask",
        "--native-upstream-position-offset", PHASE118_SELECTOR,
        PHASE126_SELECTOR, PHASE127_SELECTOR, PHASE128_SELECTOR, PHASE129_SELECTOR,
        "--native-base-rinex", "__PHASE129_RAW_BASE_RINEX__",
        "--native-base-rinex-sha256", "__PHASE129_RAW_BASE_SHA256__",
        "--out", f"{output_root}{output_route}/opaque_solution_output.csv",
        "--summary-json", f"{output_root}{output_route}/structural_summary.json",
    ]


REQUIRED_FLAGS = (
    "--all-epochs", "--android-raw-utc-keys", "--android-raw-clock-only",
    "--android-utc-wall-clock-fallback", "--native-pdc-imu-tdcp-no-bridge",
    "--native-source-direct-observable-quality", "--native-source-clock-c0d-factor",
    "--native-source-clock-c0d-meter-state-parity", "--native-source-clock-c0d-active-solve-diagnostic",
    "--native-source-clock-c0d-gnss-first-meter-state-handoff",
    "--native-source-clock-c0d-epoch-vector-parity",
    "--native-source-clock-c0d-phase99-main-multifrontal-qr-solver",
    "--native-base-pseudorange-compensation", "--native-base-pseudorange-source-miss-mask",
    "--native-upstream-position-offset", PHASE118_SELECTOR,
    PHASE126_SELECTOR, PHASE127_SELECTOR, PHASE128_SELECTOR, PHASE129_SELECTOR,
)
FORBIDDEN_FLAGS = (
    PHASE117_SELECTOR, PHASE120_SELECTOR, ADDITIONAL_SELECTOR,
    "--native-source-clock-c0d-gnss-first-raw-drift-d-initializer",
    "--native-direct-wls-ephemeral-c7d-main-seed", "--native-pdc-state-bridge",
    "--native-phase104-stage-main-attribution", "--native-phase116-carrier-tdcp-incidence-diagnostic",
    "--native-upstream-quality", "--obs",
)
FORBIDDEN_PATH_TERMS = (
    ".mat", "truth", "ground_truth", "precomputed", "coordinate", "pdc",
    "kaggle", "token",
)


def validate_command(route: str, command: Any) -> None:
    assert_equal(command, command_template(route), f"manifest/{route}/command")
    if not isinstance(command, list):
        raise fail(f"manifest/{route}/command is not argv")
    for token in command:
        if token in FORBIDDEN_FLAGS:
            raise fail(f"forbidden flag: {route}/{token}")
        if (not token.startswith("--") and
                any(term in token.lower() for term in FORBIDDEN_PATH_TERMS)):
            raise fail(f"forbidden path token: {route}/{token}")
    for flag in REQUIRED_FLAGS:
        assert_equal(command.count(flag), 1, f"manifest/{route}/{flag}")
    for flag in (PHASE117_SELECTOR, PHASE120_SELECTOR, ADDITIONAL_SELECTOR):
        assert_equal(command.count(flag), 0, f"manifest/{route}/{flag}")
    for flag, placeholder in {
        "--android-gnss": "__PHASE129_RAW_DEVICE_GNSS__",
        "--android-imu": "__PHASE129_RAW_DEVICE_IMU__",
        "--nav": "__PHASE129_RAW_BROADCAST_NAV__",
        "--native-base-rinex": "__PHASE129_RAW_BASE_RINEX__",
        "--native-base-rinex-sha256": "__PHASE129_RAW_BASE_SHA256__",
    }.items():
        assert_equal(command[command.index(flag) + 1], placeholder,
                     f"manifest/{route}/{flag}/placeholder")


def verify_manifest() -> dict[str, Any]:
    freeze = verify_freeze()
    source_pins = verify_implementation()
    manifest = read_json(MANIFEST, "Phase129 structural manifest")
    for key, expected in {"schema_version": SCHEMA, "phase": 129,
                          "execution_label": "Luna Max",
                          "status": "sealed-before-inventory-raw-execution"}.items():
        assert_equal(manifest.get(key), expected, f"manifest/{key}")
    for section, expected in (
        ("freeze", {"path": relative(FREEZE), "commit": FREEZE_COMMIT,
                     "sha256": FREEZE_SHA256}),
        ("contract_audit", {"path": relative(AUDIT), "commit": AUDIT_COMMIT,
                             "sha256": AUDIT_SHA256}),
    ):
        value = manifest.get(section)
        if not isinstance(value, dict):
            raise fail(f"manifest/{section} missing")
        for key, item in expected.items():
            assert_equal(value.get(key), item, f"manifest/{section}/{key}")
    implementation = manifest.get("implementation")
    if not isinstance(implementation, dict):
        raise fail("manifest/implementation missing")
    for key, expected in {
        "commit": IMPLEMENTATION_COMMIT,
        "candidate_id": CANDIDATE_ID,
        "required_selectors": [PHASE126_SELECTOR, PHASE127_SELECTOR,
                                PHASE128_SELECTOR, PHASE129_SELECTOR,
                                PHASE118_SELECTOR],
        "main_solver": "MULTIFRONTAL_QR",
        "fixed_tdcp_sigma_m": 0.03,
        "official_tdcp_huber_k": 0.5,
        "phase117_dynamic_sigma": False,
        "phase120_selector": False,
        "additional_frequency_selector": False,
        "default_off": True,
        "legacy_default_unchanged": True,
        "partial_selectors_allowed": False,
        "no_fallback": True,
        "no_extrapolation": True,
        "no_zero_correction": True,
    }.items():
        assert_equal(implementation.get(key), expected, f"manifest/implementation/{key}")
    assert_equal(implementation.get("source_sha256"), source_pins,
                 "manifest/implementation/source_sha256")
    binary = implementation.get("binary")
    if not isinstance(binary, dict):
        raise fail("manifest/implementation/binary missing")
    assert_equal(binary.get("path"), relative(BINARY), "manifest/binary/path")
    assert_equal(binary.get("sha256"), TARGET_BINARY_SHA256, "manifest/binary/sha256")
    artifacts = manifest.get("artifacts")
    if not isinstance(artifacts, dict):
        raise fail("manifest/artifacts missing")
    for key, path in (("validator", Path(__file__)), ("runner", RUNNER),
                      ("focused_tests", FOCUSED_TESTS)):
        pin = artifacts.get(key)
        if not isinstance(pin, dict):
            raise fail(f"manifest/artifacts/{key} missing")
        assert_equal(pin.get("path"), relative(path), f"manifest/artifacts/{key}/path")
        expected = pin.get("sha256")
        if not isinstance(expected, str) or len(expected) != 64:
            raise fail(f"manifest/artifacts/{key}/sha256 missing")
        assert_equal(sha256_static(path, f"manifest/artifacts/{key}"), expected,
                     f"manifest/artifacts/{key}/sha256")
    raw_contract = manifest.get("raw_input_contract")
    if not isinstance(raw_contract, dict):
        raise fail("manifest/raw_input_contract missing")
    for key, expected in {
        "input_names_exact": list(RAW_NAMES),
        "phone_gnss_only": True,
        "phone_imu_only": True,
        "broadcast_navigation_only": True,
        "sealed_raw_base_rinex_only": True,
        "materialize_after_authorization": True,
        "raw_content_copied_or_transformed": False,
        "solution_content": "opaque hash/row metadata only",
        "truth_mat_pdc_precomputed_coordinate_phone_coordinate_accuracy_kaggle": False,
    }.items():
        assert_equal(raw_contract.get(key), expected, f"manifest/raw_input_contract/{key}")
    inventory_contract = manifest.get("inventory_contract")
    if not isinstance(inventory_contract, dict):
        raise fail("manifest/inventory_contract missing")
    for key, expected in {
        "stage": "post-independent-authorization-pre-solver",
        "glonass_row_partition": "certified + explicit_local_miss = input",
        "shared_base_factor_ledger": True,
        "non_glonass_retained": True,
        "no_raw_or_zero_fallback": True,
        "no_extrapolation": True,
        "all_miss_or_empty_route_fail_closed": True,
        "solver_invocations_on_inventory_failure": 0,
        "per_record_reason_counts": True,
        "base_factor_ledger_equality": True,
    }.items():
        assert_equal(inventory_contract.get(key), expected,
                     f"manifest/inventory_contract/{key}")
    matrix = manifest.get("matrix")
    if not isinstance(matrix, dict):
        raise fail("manifest/matrix missing")
    for key, expected in {
        "candidate_count": 1, "route_count": 2, "runs_per_route": 1,
        "order": "MTV-A then LAX-T", "controls": 0, "reruns": 0,
        "fallbacks": 0, "sweeps": 0, "native_solver_invocations_planned": 2,
        "solution_rows_authorized": False, "truth_reads_planned": 0,
        "accuracy_calculations_planned": 0,
    }.items():
        assert_equal(matrix.get(key), expected, f"manifest/matrix/{key}")
    gates = manifest.get("structural_gates")
    if not isinstance(gates, dict):
        raise fail("manifest/structural_gates missing")
    for key, value in gates.items():
        assert_equal(value, True, f"manifest/structural_gates/{key}")
    routes = manifest.get("routes")
    if not isinstance(routes, list) or [item.get("dataset_id") for item in routes] != list(ROUTES):
        raise fail("manifest route order/count changed")
    for record in routes:
        route = record.get("dataset_id")
        if route not in DOMAIN_ROWS:
            raise fail(f"unknown route: {route}")
        for key, expected in {
            "target": "MTV-A" if route == ROUTES[0] else "LAX-T",
            "official_route_type": "Highway",
            "expected_tdcp_huber_k": 0.5,
            "domain_rows": DOMAIN_ROWS[route],
            "expected_problem_epochs": PROBLEM_EPOCHS[route],
            "expected_output_epochs": PROBLEM_EPOCHS[route],
            "runs": 1,
            "phase126_selector_count": 1,
            "phase127_selector_count": 1,
            "phase128_selector_count": 1,
            "phase129_selector_count": 1,
            "phase118_selector_count": 1,
            "phase117_selector_count": 0,
            "phase120_selector_count": 0,
            "additional_frequency_selector_count": 0,
            "solver_invocations_if_inventory_fails": 0,
        }.items():
            assert_equal(record.get(key), expected, f"manifest/{route}/{key}")
        validate_command(route, record.get("command"))
        raw = record.get("raw_inputs")
        if not isinstance(raw, dict) or set(raw) != set(RAW_NAMES):
            raise fail(f"manifest/{route}/raw_inputs must be GNSS/IMU/nav only")
        for name in RAW_NAMES:
            item = raw[name]
            if not isinstance(item, dict):
                raise fail(f"manifest/{route}/{name} missing")
            kind = ("DEVICE_GNSS" if name == "device_gnss.csv" else
                    "DEVICE_IMU" if name == "device_imu.csv" else "BROADCAST_NAV")
            for key, expected in {
                "placeholder": f"__PHASE129_RAW_{kind}__",
                "payload_read_before_authorization": False,
                "copy_or_transform": False,
            }.items():
                assert_equal(item.get(key), expected, f"manifest/{route}/{name}/{key}")
        base = record.get("base_input")
        if not isinstance(base, dict):
            raise fail(f"manifest/{route}/base_input missing")
        for key, expected in {
            "placeholder": "__PHASE129_RAW_BASE_RINEX__",
            "sha256_placeholder": "__PHASE129_RAW_BASE_SHA256__",
            "raw_rinex_only": True,
            "header_inventory_after_authorization": True,
            "payload_read_before_authorization": False,
            "hash_read_before_authorization": False,
            "copy_or_transform": False,
        }.items():
            assert_equal(base.get(key), expected, f"manifest/{route}/base/{key}")
        gate = record.get("inventory_gate")
        if not isinstance(gate, dict):
            raise fail(f"manifest/{route}/inventory_gate missing")
        for key, expected in {
            "before_solver": True, "fail_closed": True,
            "every_glonass_row_certified_or_local_miss": True,
            "shared_base_factor_ledger_equal": True,
            "non_glonass_retained": True,
            "all_miss_or_empty_fail": True,
            "solver_invocations_on_failure": 0,
        }.items():
            assert_equal(gate.get(key), expected, f"manifest/{route}/inventory_gate/{key}")
    zero_accounting(manifest.get("read_accounting_before_authorization"),
                    "manifest/read_accounting_before_authorization")
    return manifest


def _verify_side(side: dict[str, Any], label: str) -> tuple[int, int, int]:
    if not isinstance(side, dict):
        raise fail(f"{label} missing")
    input_rows = nonnegative_int(side.get("input_rows"), f"{label}/input_rows")
    certified = nonnegative_int(side.get("certified_rows"), f"{label}/certified_rows")
    local_miss = nonnegative_int(side.get("explicit_local_miss_rows"),
                                 f"{label}/explicit_local_miss_rows")
    assert_equal(input_rows, certified + local_miss, f"{label}/partition")
    header = nonnegative_int(side.get("header_primary_certified_rows"),
                             f"{label}/header_primary_certified_rows")
    geph = nonnegative_int(side.get("broadcast_geph_certified_rows"),
                           f"{label}/broadcast_geph_certified_rows")
    assert_equal(certified, header + geph, f"{label}/certification_sources")
    reasons = side.get("reason_counts")
    if not isinstance(reasons, dict):
        raise fail(f"{label}/reason_counts missing")
    reason_total = 0
    for reason, count in reasons.items():
        if not isinstance(reason, str):
            raise fail(f"{label}/reason_counts key malformed")
        reason_total += nonnegative_int(count, f"{label}/reason_counts/{reason}")
    assert_equal(reason_total, local_miss, f"{label}/reason_counts_total")
    for key in ("all_rows_classified", "no_raw_uncorrected", "no_zero_correction",
                "no_fallback", "no_extrapolation", "finite_certified_wavelength"):
        assert_true(side.get(key), f"{label}/{key}")
    non_glo = side.get("non_glonass")
    if not isinstance(non_glo, dict):
        raise fail(f"{label}/non_glonass missing")
    non_glo_input = nonnegative_int(non_glo.get("input_rows"), f"{label}/non_glonass/input_rows")
    non_glo_retained = nonnegative_int(non_glo.get("retained_rows"), f"{label}/non_glonass/retained_rows")
    assert_equal(non_glo_retained, non_glo_input, f"{label}/non_glonass/retained")
    return input_rows, certified, local_miss


def verify_inventory_record(route: str, inventory: dict[str, Any]) -> dict[str, Any]:
    """Validate one already-materialized Stage-1 record without reading a path."""
    if route not in ROUTES:
        raise fail(f"inventory/route unknown: {route}")
    if not isinstance(inventory, dict):
        raise fail("inventory record is not an object")
    assert_equal(inventory.get("route"), route, "inventory/route")
    assert_equal(inventory.get("stage"), "post-authorization-pre-solver", "inventory/stage")
    assert_equal(inventory.get("solver_invocations"), 0, "inventory/solver_invocations")
    assert_true(inventory.get("solver_may_start"), "inventory/solver_may_start")
    rover = _verify_side(inventory.get("rover"), "inventory/rover")
    base = _verify_side(inventory.get("base"), "inventory/base")
    shared = inventory.get("shared_ledger")
    if not isinstance(shared, dict):
        raise fail("inventory/shared_ledger missing")
    for key, expected in {
        "base_factor_ledger_equal": True,
        "exact_key_decision_equal": True,
        "certified_and_miss_input_conservation": True,
        "same_local_miss_reasons": True,
    }.items():
        assert_equal(shared.get(key), expected, f"inventory/shared_ledger/{key}")
    assert_equal(rover, base, "inventory/rover_base_glonass_ledger")
    correction = inventory.get("correction")
    if not isinstance(correction, dict):
        raise fail("inventory/correction missing")
    correction_input = nonnegative_int(correction.get("input_rows"), "inventory/correction/input_rows")
    retained = nonnegative_int(correction.get("retained_corrected_rows"),
                               "inventory/correction/retained_corrected_rows")
    local_miss = nonnegative_int(correction.get("explicit_provenance_miss"),
                                 "inventory/correction/explicit_provenance_miss")
    missing_stream = nonnegative_int(correction.get("missing_stream"),
                                     "inventory/correction/missing_stream")
    out_domain = nonnegative_int(correction.get("out_of_domain"),
                                 "inventory/correction/out_of_domain")
    nonfinite = nonnegative_int(correction.get("nonfinite"), "inventory/correction/nonfinite")
    assert_equal(correction_input, retained + local_miss + missing_stream + out_domain + nonfinite,
                 "inventory/correction/conservation")
    assert_equal(correction.get("application_passes"), 1, "inventory/correction/application_passes")
    for key in ("exactly_once", "no_duplicate_application", "no_raw_fallback",
                "no_zero_fallback", "finite_corrected_rows"):
        assert_true(correction.get(key), f"inventory/correction/{key}")
    usable_streams = nonnegative_int(inventory.get("usable_finite_base_streams"),
                                     "inventory/usable_finite_base_streams")
    corrected_factors = nonnegative_int(inventory.get("retained_corrected_factor_rows"),
                                        "inventory/retained_corrected_factor_rows")
    if usable_streams == 0 or corrected_factors == 0:
        raise fail("inventory empty/all-miss route must fail closed")
    for key in ("all_miss_route", "empty_route"):
        assert_equal(inventory.get(key), False, f"inventory/{key}")
    phase = inventory.get("phase129")
    if not isinstance(phase, dict):
        raise fail("inventory/phase129 missing")
    for key, expected in {
        "enabled": True,
        "all_glonass_rows_certified_or_local_miss": True,
        "shared_base_factor_ledger_equal": True,
        "non_glonass_retained": True,
        "no_raw_uncorrected": True,
        "no_zero_correction": True,
        "no_fallback": True,
        "no_extrapolation": True,
        "source_complete_a_b_c": True,
    }.items():
        assert_equal(phase.get(key), expected, f"inventory/phase129/{key}")
    return {
        "route": route,
        "inventory_passed": True,
        "solver_invocations": 0,
        "solver_may_start": True,
        "glonass_input_rows": rover[0],
        "glonass_certified_rows": rover[1],
        "glonass_local_miss_rows": rover[2],
        "usable_finite_base_streams": usable_streams,
        "retained_corrected_factor_rows": corrected_factors,
    }


def verify_structural_summary(route: str, summary: dict[str, Any]) -> dict[str, Any]:
    """Validate opaque stage/solver telemetry; never inspect solution rows."""
    if not isinstance(summary, dict):
        raise fail("structural summary is not an object")
    assert_equal(summary.get("route"), route, "summary/route")
    assert_equal(summary.get("solution_content_read"), False, "summary/solution_content_read")
    assert_equal(summary.get("fallback_used"), False, "summary/fallback_used")
    assert_equal(summary.get("rerun_count"), 0, "summary/rerun_count")
    for stage_name in ("gnss_first", "main"):
        stage = summary.get(stage_name)
        if not isinstance(stage, dict):
            raise fail(f"summary/{stage_name} missing")
        accepted = nonnegative_int(stage.get("accepted_iterations"),
                                   f"summary/{stage_name}/accepted_iterations")
        if accepted < 1:
            raise fail(f"summary/{stage_name}/accepted_iterations must be positive")
        initial = finite_number(stage.get("initial_cost"), f"summary/{stage_name}/initial_cost")
        final = finite_number(stage.get("final_cost"), f"summary/{stage_name}/final_cost")
        if not final < initial:
            raise fail(f"summary/{stage_name}/cost is not strictly decreasing")
    main = summary["main"]
    assert_equal(main.get("solver_branch"), "MULTIFRONTAL_QR", "summary/main/solver_branch")
    gates = summary.get("gates")
    if not isinstance(gates, dict):
        raise fail("summary/gates missing")
    for key, value in gates.items():
        assert_equal(value, True, f"summary/gates/{key}")
    opaque = summary.get("opaque_solution")
    if not isinstance(opaque, dict):
        raise fail("summary/opaque_solution missing")
    for key in ("sha256", "rows"):
        if key not in opaque:
            raise fail(f"summary/opaque_solution/{key} missing")
    forbidden = {"lat", "lon", "latitude", "longitude", "ecef", "x", "y", "z", "truth", "accuracy"}
    if forbidden.intersection(opaque):
        raise fail("summary exposes solution content")
    return {"route": route, "structural_gate_passed": True,
            "accepted_iterations": {name: summary[name]["accepted_iterations"]
                                     for name in ("gnss_first", "main")}}


def verify_pre_raw_static() -> dict[str, Any]:
    """Return a zero-activity qualification report with no payload access."""
    freeze = verify_freeze()
    manifest = verify_manifest()
    return {
        "status": "pre-raw-verified-no-payload-activity",
        "phase": 129,
        "execution_label": "Luna Max",
        "candidate_id": CANDIDATE_ID,
        "route_ids": list(ROUTES),
        "runs_per_route": 1,
        "route_order": "MTV-A then LAX-T",
        "raw_materialization_authorized": False,
        "raw_execution_authorized": False,
        "raw_phone_gnss_reads": 0,
        "raw_phone_imu_reads": 0,
        "broadcast_navigation_reads": 0,
        "raw_base_rinex_reads": 0,
        "raw_base_header_reads": 0,
        "raw_base_hash_reads": 0,
        "native_solver_invocations": 0,
        "solution_rows_opened": 0,
        "solution_coordinate_interpretations": 0,
        "truth_reads": 0,
        "mat_reads_or_generated": 0,
        "pdc_reads": 0,
        "precomputed_coordinate_reads": 0,
        "accuracy_calculations": 0,
        "kaggle_or_token_access": 0,
        "route_reruns": 0,
        "fallbacks": 0,
        "raw_content_copied_or_transformed": False,
        "freeze_commit": FREEZE_COMMIT,
        "freeze_sha256": FREEZE_SHA256,
        "implementation_commit": IMPLEMENTATION_COMMIT,
        "manifest_sha256": sha256_static(MANIFEST, "Phase129 manifest"),
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
            print(json.dumps(verify_pre_raw_static(), indent=2, sort_keys=True))
        return 0
    except (Phase129ContractError, OSError) as exc:
        print(f"phase129 pre-raw validator: fail-closed: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
