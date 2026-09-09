#!/usr/bin/env python3
"""Launch-free Phase132 typed-canonical preflight contract.

Only static source/manifest pins and synthetic in-memory records are
validated here.  This module never materializes raw inputs, opens a solution
or truth file, or launches the native solver.  A separately authorized
runner may apply the predicates after its one-shot raw inventory.
"""

from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
import sys
from typing import Any


ROOT = Path(__file__).resolve().parents[3]
AUDIT = ROOT / "docs/use_cases/records/smartphone_r5_phase132_typed_canonical_structural_contract_audit_v1.md"
FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_phase132_typed_canonical_structural_contract_freeze_v1.json"
MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase132_typed_canonical_structural_manifest_v1.json"
RUNNER = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase132_typed_canonical_structural_execute.py"
FOCUSED_TESTS = ROOT / "tests/test_smartphone_phase132_typed_canonical_structural.py"
PRE_RAW = ROOT / "docs/use_cases/records/smartphone_r5_phase132_typed_canonical_structural_pre_raw_accounting_v1.json"
BINARY = ROOT / "build/apps/gnss_fgo_imu_no_base"

AUDIT_COMMIT = "f6f15a5bb90a533ae5f06590cd274f005609eaca"
AUDIT_SHA256 = "d4afbca4a84251bd791d796624fe282529bb4c0d05a2076a8ec68fa27f3e0a82"
FREEZE_COMMIT = "75bd859ea29aa8dadc03fc53b131eacc7da6298b"
FREEZE_SHA256 = "2cb249efe3dee6ff22d1fb6ee48f997591ab836bf30f59aeadf85b3110e3aaa9"
IMPLEMENTATION_COMMIT = "4f546734771ac56d8342aadb88d6c36f454540fa"
TARGET_BINARY_SHA256 = "ed24b57b29f679123c3a52dbb04fad9b979092564abc4a35905bdb2abfaa0c8e"

PHASE126_SELECTOR = "--native-phase126-raw-base-source-complete"
PHASE127_SELECTOR = "--native-phase127-glonass-channel-provenance"
PHASE128_SELECTOR = "--native-phase128-glonass-provenance-parser-admission"
PHASE129_SELECTOR = "--native-phase129-glonass-local-miss-mask"
PHASE130_SELECTOR = "--native-phase130-shared-ledger-key-local-support"
PHASE131_SELECTOR = "--native-phase131-canonical-correction-band-key"
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
SCHEMA = "smartphone-r5-phase132-typed-canonical-preflight-structural-manifest.v1"
CANDIDATE_ID = "phase132-runner-canonical-preflight-admission-v1"

SOURCE_SHA256 = {
    "apps/native/gnss_fgo_imu_no_base.cpp": "6fcee581af70535b8af09a674a234352abaaf83a34b722d4e91577abaf352170",
    "include/libgnss++/algorithms/fgo_config.hpp": "75d57e64240bf19ce7be8518b03ba63077ea90ad9d3a6e43ef58290e4376568f",
    "include/libgnss++/algorithms/fgo.hpp": "df00ad9cc968f12525d4e7da16e8b6b9a41f75857d583ccefb244a8d089c2908",
    "include/libgnss++/algorithms/base_pseudorange_compensation.hpp": "67b52a4625a778fd015685aab651d9e07c80c958fa7382ae8d0c145b8969cf45",
    "include/libgnss++/algorithms/source_pseudorange_miss_mask.hpp": "64979f5ef0b6a26342726abe3873023e9a989711ba082547579164a21cb0c880",
    "include/libgnss++/algorithms/phase131_canonical_correction_key.hpp": "b9e464d16551e547f9b1bc298b7c6a90f91f5b0038d97b46625ff56cdf497123",
    "src/algorithms/base_pseudorange_compensation.cpp": "5ec8c119a55a8e974394531a7585a6ce094756c127b048d61fba330991b48ba2",
    "src/algorithms/fgo_problems.cpp": "9f75eda838a6a885657fb78d038733d6e1e027794c869c7a26b2dd610ff96aaa",
    "src/algorithms/source_pseudorange_miss_mask.cpp": "53f08f651e81f7feafc02f3d6a05490781fa4efc45014b72e262961e921ec363",
    "docs/use_cases/records/smartphone_r5_phase131_signal_key_canonicalization_freeze_v1.json": "38c29d1b6cf0a2a93a79760b1aa87c2c9278ae594dfb70495d1edb6c4ee1780f",
}


class Phase132ContractError(ValueError):
    """A structural contract violation which must fail closed."""


def fail(message: str) -> Phase132ContractError:
    return Phase132ContractError(message)


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
        elif isinstance(item, bool):
            assert_equal(item, False, f"{label}/{key}")
        elif isinstance(item, str):
            # Static/read-only labels are allowed in a zero-read record.
            if item not in {"read-only", "not-run", "sealed-metadata-only"}:
                raise fail(f"{label}/{key} is not zero-read metadata: {item!r}")
        else:
            assert_equal(item, 0, f"{label}/{key}")


# These are source-locked physical-band aliases, not a frequency table.
ANDROID_FAMILY = {
    "GPS_L1CA": "L1", "GPS_L1P": "L1", "GPS_L5": "L5",
    "GLO_G1_CA": "L1", "GLO_G1C": "L1", "GLO_L1": "L1",
    "GLO_L1CA": "L1", "GLO_L1P": "L1",
    "GAL_E1": "L1", "GAL_E5A": "L5",
    "BDS_B1I": "L1", "BDS_B1C": "L1", "BDS_B2A": "L5",
    "QZS_L1CA": "L1", "QZS_L5": "L5",
}


def typed_family(system: str, signal: str) -> str | None:
    """Map an existing typed signal to its physical family."""
    del system
    return ANDROID_FAMILY.get(str(signal).strip().upper())


def rinex_family(system: str, observation_type: str) -> str | None:
    """Map a RINEX observation band while dropping literal tracking suffix."""
    if not isinstance(system, str) or not isinstance(observation_type, str):
        return None
    code = observation_type.strip().upper()
    if len(code) < 2 or not code[1].isdigit():
        return None
    band = code[1]
    if band == "1":
        return "L1"
    if band == "5" and system.upper() in {"GPS", "GALILEO", "BEIDOU", "QZSS", "NAVIC"}:
        return "L5"
    return None


def canonicalize_typed(system: str, prn: Any, family: str | None,
                       fcn: Any = None) -> dict[str, Any]:
    """Construct the typed key and reject uncertified GLONASS provenance."""
    result: dict[str, Any] = {
        "accepted": False, "system": system, "prn": prn,
        "family": family, "fcn": fcn, "reason": None,
    }
    if family not in {"L1", "L5"}:
        result["reason"] = "unknown-physical-frequency-family"
        return result
    if not isinstance(prn, int) or isinstance(prn, bool) or prn <= 0:
        result["reason"] = "invalid-prn"
        return result
    if system == "GLONASS":
        if not isinstance(fcn, int) or isinstance(fcn, bool):
            result["reason"] = "glonass-fcn-missing"
            return result
        if fcn < -7 or fcn > 6:
            result["reason"] = "glonass-fcn-out-of-range"
            return result
    elif fcn is not None:
        result["reason"] = "non-glonass-fcn-present"
        return result
    result["accepted"] = True
    result["key"] = (system, prn, family, fcn if system == "GLONASS" else None)
    return result


def verify_freeze() -> dict[str, Any]:
    assert_equal(sha256_static(AUDIT, "Phase132 contract audit"), AUDIT_SHA256,
                 "freeze/contract_audit_sha256")
    freeze = read_json(FREEZE, "Phase132 structural freeze")
    for key, expected in {
        "schema_version": "smartphone-r5-phase132-typed-canonical-preflight-structural-contract-freeze.v1",
        "phase": 132,
        "execution_label": "Luna Max",
        "status": "sealed-launch-free-structural-contract",
    }.items():
        assert_equal(freeze.get(key), expected, f"freeze/{key}")
    starting = freeze.get("starting_state")
    if not isinstance(starting, dict):
        raise fail("freeze/starting_state missing")
    for key, expected in {
        "prior_forensic_freeze_commit": "3b3c785f5a641bc3f2f49ff4e704f80b41a7ea06",
        "contract_audit_commit": AUDIT_COMMIT,
        "contract_audit_sha256": AUDIT_SHA256,
        "implementation_commit": IMPLEMENTATION_COMMIT,
        "worktree_before_freeze": "clean",
        "raw_reads_before_freeze": 0,
        "solver_invocations_before_freeze": 0,
    }.items():
        assert_equal(starting.get(key), expected, f"freeze/starting_state/{key}")
    decision = freeze.get("decision")
    if not isinstance(decision, dict):
        raise fail("freeze/decision missing")
    for key, expected in {
        "candidate_count": 1, "candidate_id": CANDIDATE_ID,
        "candidate_class": "runner-only typed canonical preflight and reachability evidence",
        "selector": PHASE131_SELECTOR, "source_backed": True,
        "implementation_authorized": True, "raw_materialization_authorized": False,
        "solver_execution_authorized": False, "truth_evaluation_authorized": False,
        "accuracy_authorized": False, "solution_publication_authorized": False,
        "kaggle_authorized": False, "default_off": True,
        "old_phase131_authorization_reusable": False,
    }.items():
        assert_equal(decision.get(key), expected, f"freeze/decision/{key}")
    canonical = freeze.get("canonical_key")
    if not isinstance(canonical, dict):
        raise fail("freeze/canonical_key missing")
    for key, expected in {
        "fields": ["GNSSSystem", "SatelliteId.prn", "PhysicalFrequencyFamily",
                   "certified_GLONASS_FCN_if_GLONASS"],
        "physical_frequency_families": ["L1", "L5"],
        "literal_tracking_code_in_join": False,
        "original_signal_retained_as_provenance": True,
        "glonass_fcn_source": "Phase127/128 exact query-time certified header/geph provenance",
        "glonass_fcn_range": [-7, 6], "carrier_frequency_used_as_fcn": False,
        "row_index_pairing": False, "unknown_family": "explicit miss",
        "ambiguous_or_conflicting_mapping": "fail closed",
    }.items():
        assert_equal(canonical.get(key), expected, f"freeze/canonical_key/{key}")
    composition = freeze.get("composition")
    if not isinstance(composition, dict):
        raise fail("freeze/composition missing")
    assert_equal(composition.get("required_selectors_exactly_once"), [
        PHASE126_SELECTOR, PHASE127_SELECTOR, PHASE128_SELECTOR,
        PHASE129_SELECTOR, PHASE130_SELECTOR, PHASE131_SELECTOR,
        PHASE118_SELECTOR,
    ], "freeze/composition/required_selectors_exactly_once")
    assert_equal(composition.get("forbidden_selectors"), [
        PHASE117_SELECTOR, PHASE120_SELECTOR, ADDITIONAL_SELECTOR,
        "--native-direct-wls-ephemeral-c7d-main-seed", "--native-pdc-state-bridge",
    ], "freeze/composition/forbidden_selectors")
    for key, expected in {
        "phase117_dynamic_sigma": False, "phase120_atmosphere_cancellation": False,
        "additional_frequency_bands": False, "fixed_tdcp_sigma_m": 0.03,
        "official_huber_k": 0.5, "main_solver": "MULTIFRONTAL_QR",
        "native_graph_factor_equation_unchanged": True,
        "units_sigma_filter_lm_unchanged": True, "c7_d_c0d_ccdd_unchanged": True,
        "imu_tdcp_unchanged": True, "pixel5_offset_unchanged": True,
        "legacy_default_unchanged": True,
    }.items():
        assert_equal(composition.get(key), expected, f"freeze/composition/{key}")
    inventory = freeze.get("inventory_contract")
    if not isinstance(inventory, dict):
        raise fail("freeze/inventory_contract missing")
    for key, expected in {
        "stage": "post-independent-authorization-pre-solver",
        "python_typed_preflight_required": True,
        "python_preflight_must_not_delegate_phase130_builder": True,
        "python_preflight_evidence_required": True,
        "native_selector_forwarding_evidence_required": True,
        "native_command_construction_evidence_required": True,
        "native_resolver_reach_evidence_required": True,
        "native_resolver_call_count_semantics": "canonical rows plus canonical rejected rows from native summary",
        "old_phase130_counters": "comparison-only; never an admission predicate",
        "same_physical_family_aliases_only": True,
        "literal_tracking_code_excluded_from_join": True,
        "original_signal_provenance_retained": True,
        "certified_glonass_fcn_required": True,
        "ambiguous_or_conflicting_mapping_fail_closed": True,
        "duplicate_or_nonmonotonic_time_fail_closed": True,
        "retained_rover_exact_key_support": True,
        "exact_endpoint_or_adjacent_two_point_bracket": True,
        "unmatched_rover_explicit_factor_miss": True,
        "unused_base_rows_accounted": True,
        "whole_side_count_equality_required": False,
        "equal_cadence_required": False, "row_index_pairing": False,
        "raw_uncorrected_fallback": False, "zero_correction_fallback": False,
        "nearest_hold_or_extrapolation": False, "exactly_once_correction": True,
        "all_miss_or_empty_support": "fail before solver",
        "solver_invocations_on_preflight_failure": 0,
    }.items():
        assert_equal(inventory.get(key), expected, f"freeze/inventory_contract/{key}")
    gates = freeze.get("structural_gates")
    if not isinstance(gates, dict):
        raise fail("freeze/structural_gates missing")
    for key, value in gates.items():
        if key == "solver_invocations_on_preflight_failure":
            assert_equal(value, 0, f"freeze/structural_gates/{key}")
        else:
            assert_equal(value, True, f"freeze/structural_gates/{key}")
    routes = freeze.get("routes")
    if not isinstance(routes, list) or [item.get("target") for item in routes] != ["MTV-A", "LAX-T"]:
        raise fail("freeze route order/count changed")
    for item in routes:
        route = item.get("dataset_id")
        if route not in ROUTES:
            raise fail(f"freeze unknown route: {route}")
        assert_equal(item.get("runs"), 1, f"freeze/{route}/runs")
        assert_equal(item.get("solver_invocations_if_preflight_fails"), 0,
                     f"freeze/{route}/solver_invocations_if_preflight_fails")
        assert_equal(item.get("expected_problem_epochs"), PROBLEM_EPOCHS[route],
                     f"freeze/{route}/expected_problem_epochs")
    accounting = freeze.get("read_accounting")
    if not isinstance(accounting, dict):
        raise fail("freeze/read_accounting missing")
    for key, value in accounting.items():
        if isinstance(value, str):
            assert_equal(value, "read-only" if key == "source_text_reads" else "read-only",
                         f"freeze/read_accounting/{key}")
        elif key.endswith("copied_or_transformed"):
            assert_equal(value, 0, f"freeze/read_accounting/{key}")
        elif isinstance(value, bool):
            assert_equal(value, False, f"freeze/read_accounting/{key}")
        else:
            assert_equal(value, 0, f"freeze/read_accounting/{key}")
    return freeze


def verify_static_sources() -> dict[str, str]:
    actual: dict[str, str] = {}
    for name, expected in SOURCE_SHA256.items():
        digest = sha256_static(ROOT / name, f"source/{name}")
        assert_equal(digest, expected, f"source/{name}/sha256")
        actual[name] = digest
    assert_equal(sha256_static(BINARY, "target binary"), TARGET_BINARY_SHA256,
                 "target binary/sha256")
    return actual


def command_template(route: str) -> list[str]:
    """Build only a placeholder argv for future authorization."""
    if route not in ROUTES:
        raise fail(f"unknown route: {route}")
    output_route = route.replace("/", "__")
    output_root = "output/smartphone-r5/phase132-typed-canonical-structural-v1/"
    return [
        "build/apps/gnss_fgo_imu_no_base", "--dataset-id", route,
        "--android-gnss", "__PHASE132_RAW_DEVICE_GNSS__",
        "--android-imu", "__PHASE132_RAW_DEVICE_IMU__",
        "--nav", "__PHASE132_RAW_BROADCAST_NAV__",
        "--all-epochs", "--android-raw-utc-keys", "--android-raw-clock-only",
        "--android-utc-wall-clock-fallback", "--native-pdc-imu-tdcp-no-bridge",
        "--native-source-direct-observable-quality", "--native-source-clock-c0d-factor",
        "--native-source-clock-c0d-meter-state-parity",
        "--native-source-clock-c0d-active-solve-diagnostic",
        "--native-source-clock-c0d-gnss-first-meter-state-handoff",
        "--native-source-clock-c0d-epoch-vector-parity",
        "--native-source-clock-c0d-phase99-main-multifrontal-qr-solver",
        "--native-base-pseudorange-compensation",
        "--native-base-pseudorange-source-miss-mask", "--native-upstream-position-offset",
        PHASE118_SELECTOR, PHASE126_SELECTOR, PHASE127_SELECTOR,
        PHASE128_SELECTOR, PHASE129_SELECTOR, PHASE130_SELECTOR,
        PHASE131_SELECTOR, "--native-base-rinex", "__PHASE132_RAW_BASE_RINEX__",
        "--native-base-rinex-sha256", "__PHASE132_RAW_BASE_SHA256__",
        "--out", f"{output_root}{output_route}/opaque_solution_output.csv",
        "--summary-json", f"{output_root}{output_route}/structural_summary.json",
    ]


REQUIRED_FLAGS = (
    "--all-epochs", "--android-raw-utc-keys", "--android-raw-clock-only",
    "--android-utc-wall-clock-fallback", "--native-pdc-imu-tdcp-no-bridge",
    "--native-source-direct-observable-quality", "--native-source-clock-c0d-factor",
    "--native-source-clock-c0d-meter-state-parity",
    "--native-source-clock-c0d-active-solve-diagnostic",
    "--native-source-clock-c0d-gnss-first-meter-state-handoff",
    "--native-source-clock-c0d-epoch-vector-parity",
    "--native-source-clock-c0d-phase99-main-multifrontal-qr-solver",
    "--native-base-pseudorange-compensation", "--native-base-pseudorange-source-miss-mask",
    "--native-upstream-position-offset", PHASE118_SELECTOR, PHASE126_SELECTOR,
    PHASE127_SELECTOR, PHASE128_SELECTOR, PHASE129_SELECTOR, PHASE130_SELECTOR,
    PHASE131_SELECTOR,
)
FORBIDDEN_FLAGS = (
    PHASE117_SELECTOR, PHASE120_SELECTOR, ADDITIONAL_SELECTOR,
    "--native-source-clock-c0d-gnss-first-raw-drift-d-initializer",
    "--native-direct-wls-ephemeral-c7d-main-seed", "--native-pdc-state-bridge",
    "--native-phase104-stage-main-attribution", "--native-phase116-carrier-tdcp-incidence-diagnostic",
    "--native-upstream-quality", "--obs",
)
FORBIDDEN_PATH_TERMS = (
    ".mat", "truth", "ground_truth", "precomputed", "coordinate", "pdc", "kaggle", "token",
)


def validate_command(route: str, command: Any) -> None:
    assert_equal(command, command_template(route), f"command/{route}")
    if not isinstance(command, list):
        raise fail(f"command/{route} is not argv")
    for token in command:
        if token in FORBIDDEN_FLAGS:
            raise fail(f"forbidden flag: {route}/{token}")
        if not token.startswith("--") and any(term in token.lower() for term in FORBIDDEN_PATH_TERMS):
            raise fail(f"forbidden path token: {route}/{token}")
    for flag in REQUIRED_FLAGS:
        assert_equal(command.count(flag), 1, f"command/{route}/{flag}")
    for flag in (PHASE117_SELECTOR, PHASE120_SELECTOR, ADDITIONAL_SELECTOR):
        assert_equal(command.count(flag), 0, f"command/{route}/{flag}")
    for flag, placeholder in {
        "--android-gnss": "__PHASE132_RAW_DEVICE_GNSS__",
        "--android-imu": "__PHASE132_RAW_DEVICE_IMU__",
        "--nav": "__PHASE132_RAW_BROADCAST_NAV__",
        "--native-base-rinex": "__PHASE132_RAW_BASE_RINEX__",
        "--native-base-rinex-sha256": "__PHASE132_RAW_BASE_SHA256__",
    }.items():
        assert_equal(command[command.index(flag) + 1], placeholder,
                     f"command/{route}/{flag}/placeholder")


def verify_manifest() -> dict[str, Any]:
    verify_freeze()
    source_pins = verify_static_sources()
    manifest = read_json(MANIFEST, "Phase132 structural manifest")
    for key, expected in {
        "schema_version": SCHEMA, "phase": 132, "execution_label": "Luna Max",
        "status": "sealed-before-independent-authorization",
    }.items():
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
        "commit": IMPLEMENTATION_COMMIT, "candidate_id": CANDIDATE_ID,
        "selector": PHASE131_SELECTOR,
        "required_selectors": [PHASE126_SELECTOR, PHASE127_SELECTOR, PHASE128_SELECTOR,
                                PHASE129_SELECTOR, PHASE130_SELECTOR, PHASE131_SELECTOR,
                                PHASE118_SELECTOR],
        "main_solver": "MULTIFRONTAL_QR", "fixed_tdcp_sigma_m": 0.03,
        "official_huber_k": 0.5, "phase117_dynamic_sigma": False,
        "phase120_selector": False, "additional_frequency_selector": False,
        "default_off": True, "legacy_default_unchanged": True,
        "partial_selectors_allowed": False, "factor_topology_unchanged": True,
        "equations_units_sigma_filter_lm_unchanged": True, "no_fallback": True,
        "no_extrapolation": True, "no_zero_correction": True,
        "no_raw_uncorrected_retention": True,
        "literal_tracking_code_in_join": False,
        "original_signal_provenance_retained": True,
        "phase130_counters_admission_predicate": False,
    }.items():
        assert_equal(implementation.get(key), expected, f"manifest/implementation/{key}")
    assert_equal(implementation.get("source_sha256"), source_pins,
                 "manifest/implementation/source_sha256")
    binary = implementation.get("target_binary")
    if not isinstance(binary, dict):
        raise fail("manifest/implementation/target_binary missing")
    assert_equal(binary.get("path"), relative(BINARY), "manifest/target_binary/path")
    assert_equal(binary.get("sha256"), TARGET_BINARY_SHA256, "manifest/target_binary/sha256")
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
    contract = manifest.get("inventory_contract")
    if not isinstance(contract, dict):
        raise fail("manifest/inventory_contract missing")
    for key, expected in {
        "stage": "post-independent-authorization-pre-solver",
        "python_typed_preflight_required": True,
        "phase130_literal_builder_allowed": False,
        "phase130_counters_admission_predicate": False,
        "python_preflight_evidence_required": True,
        "native_selector_and_command_evidence_required": True,
        "native_resolver_reach_evidence_required": True,
        "side_local_conservation": True, "canonical_key_conservation": True,
        "same_physical_family_aliases_only": True,
        "original_signal_provenance_retained": True,
        "literal_tracking_code_excluded_from_join": True,
        "certified_glonass_fcn_required": True,
        "unknown_family_explicit_miss": True,
        "ambiguous_or_conflicting_mapping_fail_closed": True,
        "duplicate_or_nonmonotonic_fail_closed": True,
        "retained_rover_exact_key_support": True,
        "exact_endpoint_or_adjacent_two_point_bracket": True,
        "unmatched_rover_explicit_factor_miss": True,
        "unused_base_rows_accounted": True,
        "whole_ledger_equality_required": False,
        "equal_cadence_required": False, "equal_row_count_required": False,
        "no_raw_or_zero_fallback": True, "no_extrapolation_or_endpoint_hold": True,
        "all_miss_or_empty_route_fail_closed": True,
        "solver_invocations_on_inventory_failure": 0,
    }.items():
        assert_equal(contract.get(key), expected, f"manifest/inventory_contract/{key}")
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
    routes = manifest.get("routes")
    if not isinstance(routes, list) or [item.get("dataset_id") for item in routes] != list(ROUTES):
        raise fail("manifest route order/count changed")
    for record in routes:
        route = record.get("dataset_id")
        if route not in DOMAIN_ROWS:
            raise fail(f"manifest unknown route: {route}")
        for key, expected in {
            "target": "MTV-A" if route == ROUTES[0] else "LAX-T",
            "official_route_type": "Highway", "expected_tdcp_huber_k": 0.5,
            "domain_rows": DOMAIN_ROWS[route], "expected_problem_epochs": PROBLEM_EPOCHS[route],
            "expected_output_epochs": PROBLEM_EPOCHS[route], "runs": 1,
            "phase126_selector_count": 1, "phase127_selector_count": 1,
            "phase128_selector_count": 1, "phase129_selector_count": 1,
            "phase130_selector_count": 1, "phase131_selector_count": 1,
            "phase118_selector_count": 1, "phase117_selector_count": 0,
            "phase120_selector_count": 0, "additional_frequency_selector_count": 0,
            "solver_invocations_if_preflight_fails": 0,
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
            assert_equal(item.get("payload_read_before_authorization"), False,
                         f"manifest/{route}/{name}/payload_read_before_authorization")
            assert_equal(item.get("copy_or_transform"), False,
                         f"manifest/{route}/{name}/copy_or_transform")
        base = record.get("base_input")
        if not isinstance(base, dict):
            raise fail(f"manifest/{route}/base_input missing")
        for key, expected in {
            "placeholder": "__PHASE132_RAW_BASE_RINEX__",
            "sha256_placeholder": "__PHASE132_RAW_BASE_SHA256__",
            "raw_rinex_only": True, "header_inventory_after_authorization": True,
            "payload_read_before_authorization": False, "hash_read_before_authorization": False,
            "copy_or_transform": False,
        }.items():
            assert_equal(base.get(key), expected, f"manifest/{route}/base/{key}")
    zero_accounting(manifest.get("read_accounting_before_authorization"),
                    "manifest/read_accounting_before_authorization")
    return manifest


def verify_inventory_record(route: str, inventory: dict[str, Any]) -> dict[str, Any]:
    """Validate a post-preflight inventory without reading any payload."""
    if route not in ROUTES or not isinstance(inventory, dict):
        raise fail("inventory route/object invalid")
    assert_equal(inventory.get("route"), route, "inventory/route")
    assert_equal(inventory.get("stage"), "post-independent-authorization-pre-solver",
                 "inventory/stage")
    assert_equal(inventory.get("solver_invocations"), 0, "inventory/solver_invocations")
    assert_true(inventory.get("solver_may_start"), "inventory/solver_may_start")
    for side_name in ("rover", "base"):
        side = inventory.get(side_name)
        if not isinstance(side, dict):
            raise fail(f"inventory/{side_name} missing")
        rows = nonnegative_int(side.get("input_rows"), f"inventory/{side_name}/input_rows")
        certified = nonnegative_int(side.get("certified_rows"), f"inventory/{side_name}/certified_rows")
        missed = nonnegative_int(side.get("explicit_local_miss_rows"), f"inventory/{side_name}/explicit_local_miss_rows")
        assert_equal(rows, certified + missed, f"inventory/{side_name}/conservation")
        for key, expected in {
            "all_rows_classified": True, "no_raw_uncorrected": True,
            "no_zero_correction": True, "no_fallback": True,
            "no_extrapolation": True, "finite_certified_wavelength": True,
            "canonical_key_accounted": True, "literal_provenance_retained": True,
        }.items():
            assert_equal(side.get(key), expected, f"inventory/{side_name}/{key}")
        canonical = side.get("canonical")
        if not isinstance(canonical, dict):
            raise fail(f"inventory/{side_name}/canonical missing")
        for key, expected in {
            "all_rows_accounted": True, "same_physical_family_aliases_only": True,
            "different_family_rejected": True, "unknown_band_explicit_miss": True,
            "literal_tracking_code_in_join": False,
            "original_signal_provenance_retained": True,
            "fcn_certification_fail_closed": True,
        }.items():
            assert_equal(canonical.get(key), expected, f"inventory/{side_name}/canonical/{key}")
    shared = inventory.get("shared_ledger")
    if not isinstance(shared, dict):
        raise fail("inventory/shared_ledger missing")
    for key, expected in {
        "side_local_conservation": True, "canonical_key_conservation": True,
        "same_physical_family_aliases_only": True,
        "different_physical_band_rejected": True,
        "literal_tracking_code_in_join": False,
        "original_signal_provenance_retained": True,
        "certified_glonass_fcn_only": True,
        "ambiguous_multi_code_fail_closed": True,
        "duplicate_conflict_fail_closed": True,
        "retained_rover_exact_key_support": True,
        "unmatched_rover_explicit_factor_miss": True,
        "unused_base_rows_accounted": True,
        "whole_ledger_equality_required": False,
        "cross_side_count_equality_required": False,
        "no_raw_uncorrected": True, "no_zero_correction": True,
        "no_fallback": True, "no_extrapolation": True,
    }.items():
        assert_equal(shared.get(key), expected, f"inventory/shared_ledger/{key}")
    phase = inventory.get("phase132")
    if not isinstance(phase, dict):
        raise fail("inventory/phase132 missing")
    for key, expected in {
        "enabled": True, "python_preflight_started": True,
        "python_preflight_executed": True, "python_preflight_call_count": 1,
        "native_selector_forwarded": False, "native_command_constructed": False,
        "native_binary_invocation_attempted": False, "native_resolver_executed": False,
        "native_resolver_call_count": 0,
        "native_resolver_evidence_source": "not-launched",
        "source_complete_a_b_c": True,
    }.items():
        assert_equal(phase.get(key), expected, f"inventory/phase132/{key}")
    canonical_count = nonnegative_int(phase.get("canonical_key_count"),
                                      "inventory/phase132/canonical_key_count")
    if canonical_count == 0:
        raise fail("inventory/phase132 has no canonical support key")
    correction = inventory.get("correction")
    if not isinstance(correction, dict):
        raise fail("inventory/correction missing")
    factor_input = nonnegative_int(correction.get("factor_input_rows"), "inventory/correction/factor_input_rows")
    retained = nonnegative_int(correction.get("retained_corrected_rows"), "inventory/correction/retained_corrected_rows")
    missing = nonnegative_int(correction.get("missing_exact_stream"), "inventory/correction/missing_exact_stream")
    out_domain = nonnegative_int(correction.get("out_of_domain"), "inventory/correction/out_of_domain")
    nonfinite = nonnegative_int(correction.get("nonfinite"), "inventory/correction/nonfinite")
    assert_equal(factor_input, retained + missing + out_domain + nonfinite,
                 "inventory/correction/factor_conservation")
    support = correction.get("support")
    if not isinstance(support, dict):
        raise fail("inventory/correction/support missing")
    support_count = sum(nonnegative_int(support.get(key), f"inventory/correction/support/{key}")
                        for key in ("exact_endpoint_rows", "adjacent_two_point_bracket_rows",
                                    "exact_interior_sample_rows"))
    assert_equal(support_count, retained, "inventory/correction/support/conservation")
    for key, expected in {
        "retained_factors_have_exact_key_support": True,
        "one_support_result_per_retained_factor": True,
        "unused_base_rows_accounted": True, "application_passes": 1,
        "exactly_once": True, "no_duplicate_application": True,
        "no_raw_fallback": True, "no_zero_fallback": True,
        "no_nearest_fill": True, "no_endpoint_hold": True,
        "no_extrapolation": True, "finite_corrected_rows": True,
        "canonical_key_mode": True, "factor_topology_changed": False,
    }.items():
        assert_equal(correction.get(key), expected, f"inventory/correction/{key}")
    if retained == 0:
        raise fail("inventory has no retained corrected factor")
    return {"route": route, "inventory_passed": True,
            "solver_invocations": 0, "solver_may_start": True,
            "canonical_key_count": canonical_count,
            "retained_corrected_factor_rows": retained}


def verify_structural_summary(route: str, summary: dict[str, Any]) -> dict[str, Any]:
    """Validate opaque stage/native telemetry without inspecting solution rows."""
    if not isinstance(summary, dict):
        raise fail("summary is not an object")
    assert_equal(summary.get("route"), route, "summary/route")
    assert_equal(summary.get("solution_content_read"), False, "summary/solution_content_read")
    assert_equal(summary.get("fallback_used"), False, "summary/fallback_used")
    assert_equal(summary.get("rerun_count"), 0, "summary/rerun_count")
    for stage_name in ("gnss_first", "main"):
        stage = summary.get(stage_name)
        if not isinstance(stage, dict):
            raise fail(f"summary/{stage_name} missing")
        if nonnegative_int(stage.get("accepted_iterations"),
                           f"summary/{stage_name}/accepted_iterations") < 1:
            raise fail(f"summary/{stage_name}/accepted_iterations must be positive")
        initial = finite_number(stage.get("initial_cost"), f"summary/{stage_name}/initial_cost")
        final = finite_number(stage.get("final_cost"), f"summary/{stage_name}/final_cost")
        if not final < initial:
            raise fail(f"summary/{stage_name}/cost is not strictly decreasing")
    assert_equal(summary["main"].get("solver_branch"), "MULTIFRONTAL_QR",
                 "summary/main/solver_branch")
    phase = summary.get("phase132")
    if not isinstance(phase, dict):
        raise fail("summary/phase132 missing")
    for key, expected in {
        "enabled": True, "python_preflight_executed": True,
        "native_selector_forwarded": True, "native_command_constructed": True,
        "native_binary_invocation_attempted": True, "native_resolver_executed": True,
        "literal_tracking_code_in_join": False,
        "original_signal_provenance_retained": True,
        "side_local_conservation": True, "canonical_key_conservation": True,
        "retained_rover_exact_key_support": True,
        "unused_base_rows_accounted": True, "no_fallback": True,
        "factor_topology_changed": False,
    }.items():
        assert_equal(phase.get(key), expected, f"summary/phase132/{key}")
    nonnegative_int(phase.get("native_resolver_call_count"),
                    "summary/phase132/native_resolver_call_count")
    if phase.get("native_resolver_evidence_source") != "native-summary":
        raise fail("summary/phase132 native resolver evidence is not native summary")
    gates = summary.get("gates")
    if not isinstance(gates, dict):
        raise fail("summary/gates missing")
    for key, value in gates.items():
        assert_equal(value, True, f"summary/gates/{key}")
    opaque = summary.get("opaque_solution")
    if not isinstance(opaque, dict) or "sha256" not in opaque or "rows" not in opaque:
        raise fail("summary/opaque_solution missing hash/rows")
    forbidden = {"lat", "lon", "latitude", "longitude", "ecef", "x", "y", "z", "truth", "accuracy"}
    if forbidden.intersection(opaque):
        raise fail("summary exposes solution content")
    return {"route": route, "structural_gate_passed": True,
            "accepted_iterations": {name: summary[name]["accepted_iterations"]
                                     for name in ("gnss_first", "main")}}


def verify_pre_raw_static() -> dict[str, Any]:
    verify_freeze()
    manifest = verify_manifest()
    pre = read_json(PRE_RAW, "Phase132 pre-raw accounting")
    for key, expected in {
        "schema_version": "smartphone-r5-phase132-typed-canonical-preflight-structural-pre-raw-accounting.v1",
        "phase": 132, "execution_label": "Luna Max", "status": "sealed-pre-raw-zero-read",
        "audit_commit": AUDIT_COMMIT, "freeze_commit": FREEZE_COMMIT,
        "implementation_commit": IMPLEMENTATION_COMMIT,
        "contract_code_commit": manifest.get("contract_code_commit"),
    }.items():
        assert_equal(pre.get(key), expected, f"pre-raw/{key}")
    pin = pre.get("manifest_pin")
    if not isinstance(pin, dict):
        raise fail("pre-raw/manifest_pin missing")
    assert_equal(pin.get("path"), relative(MANIFEST), "pre-raw/manifest_pin/path")
    assert_equal(pin.get("sha256"), sha256_static(MANIFEST, "Phase132 manifest"),
                 "pre-raw/manifest_pin/sha256")
    commit = pin.get("commit")
    if not isinstance(commit, str) or len(commit) != 40 or any(c not in "0123456789abcdef" for c in commit):
        raise fail("pre-raw/manifest_pin/commit must be full lowercase SHA")
    zero_accounting(pre.get("read_accounting"), "pre-raw/read_accounting")
    return pre


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
    except (Phase132ContractError, OSError) as exc:
        print(f"phase132 pre-raw validator: fail-closed: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
