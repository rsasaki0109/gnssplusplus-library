#!/usr/bin/env python3
"""Launch-free Phase131 canonical correction structural contract.

This module validates only source pins, the sealed contract, and synthetic
in-memory inventory/summary records.  It deliberately has no raw-input
materializer and no native-process launcher.  A later independently
authorized runner may reuse these pure predicates after its one-pass raw
inventory, but this module never opens a payload path.
"""

from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
import sys
from typing import Any, Iterable


ROOT = Path(__file__).resolve().parents[3]
AUDIT = ROOT / "docs/use_cases/records/smartphone_r5_phase131_canonical_correction_structural_contract_audit_v1.md"
FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_phase131_canonical_correction_structural_contract_freeze_v1.json"
MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase131_canonical_correction_structural_manifest_v1.json"
RUNNER = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase131_canonical_correction_structural_execute.py"
FOCUSED_TESTS = ROOT / "tests/test_smartphone_phase131_canonical_correction_structural.py"
PRE_RAW = ROOT / "docs/use_cases/records/smartphone_r5_phase131_canonical_correction_structural_pre_raw_accounting_v1.json"
BINARY = ROOT / "build/apps/gnss_fgo_imu_no_base"


AUDIT_COMMIT = "c5afacef8416aace5e8f713ad8a4b08654666457"
AUDIT_SHA256 = "403c6434629edf6f51bdb4b6f77fa1800f76d9b367fc8b52a42d60f6802c8227"
FREEZE_COMMIT = "322f1278c9b802d3414b1fd275baeb3e731542a9"
FREEZE_SHA256 = "1ac82fad3ebe27b9a3e0592e9d80f6b73dff5bbee20f7ff3dd276ca8cc763200"
IMPLEMENTATION_COMMIT = "c3af051e46f3685c0d3406537fa8f7c12eea75f2"
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
SCHEMA = "smartphone-r5-phase131-canonical-correction-band-structural-manifest.v1"
CANDIDATE_ID = "phase131-canonical-correction-band-key-v1"

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


class Phase131ContractError(ValueError):
    """A structural-contract violation that must fail closed."""


def fail(message: str) -> Phase131ContractError:
    return Phase131ContractError(message)


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
    """Hash only source/contract artifacts, never an input payload."""
    lowered = path.name.lower()
    if (lowered in RAW_NAMES or
            lowered in {"base.obs", "truth.csv", "ground_truth.csv"} or
            lowered.endswith((".csv", ".nav", ".obs", ".mat")) or
            "truth" in lowered or "ground_truth" in lowered):
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
        else:
            assert_equal(item, 0, f"{label}/{key}")


def verify_freeze() -> dict[str, Any]:
    assert_equal(sha256_static(AUDIT, "Phase131 contract audit"), AUDIT_SHA256,
                 "freeze/contract_audit_sha256")
    freeze = read_json(FREEZE, "Phase131 structural contract freeze")
    for key, expected in {
        "schema_version": "smartphone-r5-phase131-canonical-correction-band-structural-contract-freeze.v1",
        "phase": 131,
        "execution_label": "Luna Max",
        "status": "sealed-launch-free-inventory-structural-contract",
    }.items():
        assert_equal(freeze.get(key), expected, f"freeze/{key}")
    state = freeze.get("starting_state")
    if not isinstance(state, dict):
        raise fail("freeze/starting_state missing")
    for key, expected in {
        "implementation_commit": IMPLEMENTATION_COMMIT,
        "prior_phase131_candidate_freeze_commit": "3b0fb466a07e3738392e098fbf7465209378b183",
        "contract_audit_commit": AUDIT_COMMIT,
        "contract_audit_sha256": AUDIT_SHA256,
        "worktree_before_contract": "clean",
    }.items():
        assert_equal(state.get(key), expected, f"freeze/starting_state/{key}")
    decision = freeze.get("decision")
    if not isinstance(decision, dict):
        raise fail("freeze/decision missing")
    for key, expected in {
        "candidate_count": 1,
        "candidate_id": CANDIDATE_ID,
        "selector": PHASE131_SELECTOR,
        "source_backed": True,
        "implementation_authorized": True,
        "raw_materialization_authorized": False,
        "solver_execution_authorized": False,
        "truth_evaluation_authorized": False,
        "accuracy_authorized": False,
        "solution_publication_authorized": False,
        "kaggle_authorized": False,
        "default_off": True,
        "route_order": ["MTV-A", "LAX-T"],
        "runs_per_route": 1,
    }.items():
        assert_equal(decision.get(key), expected, f"freeze/decision/{key}")
    canonical = freeze.get("canonical_key")
    if not isinstance(canonical, dict):
        raise fail("freeze/canonical_key missing")
    for key, expected in {
        "fields": ["GNSSSystem", "SatelliteId.prn", "PhysicalFrequencyFamily", "certified_GLONASS_FCN_if_GLONASS"],
        "physical_frequency_families": ["L1", "L5"],
        "literal_tracking_code_in_join": False,
        "original_signal_retained_as_provenance": True,
        "glonass_fcn_source": "Phase127/128 exact query-time certified header/geph provenance",
        "glonass_fcn_range": [-7, 6],
        "unknown_band": "explicit local miss",
        "ambiguous_or_conflicting_multi_code": "fail closed",
        "different_physical_band": "not a match",
        "row_index_pairing": False,
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
        "--native-direct-wls-ephemeral-c7d-main-seed",
        "--native-pdc-state-bridge",
    ], "freeze/composition/forbidden_selectors")
    for key, expected in {
        "phase117_dynamic_sigma": False,
        "phase120_atmosphere_cancellation": False,
        "additional_frequency_bands": False,
        "phase118_fixed_tdcp_sigma_m": 0.03,
        "phase118_official_huber_k": 0.5,
        "main_solver": "MULTIFRONTAL_QR",
        "factor_topology_changed": False,
        "equations_units_sigma_filter_lm_changed": False,
        "c7_d_c0d_ccdd_changed": False,
        "imu_tdcp_changed": False,
        "pixel5_offset_changed": False,
    }.items():
        assert_equal(composition.get(key), expected, f"freeze/composition/{key}")
    inventory = freeze.get("inventory_contract")
    if not isinstance(inventory, dict):
        raise fail("freeze/inventory_contract missing")
    for key, expected in {
        "stage": "post-independent-authorization-pre-solver",
        "side_local_conservation": "input_rows = certified_rows + explicit_local_miss_rows",
        "certification_source_conservation": "certified_rows = header_primary + broadcast_geph",
        "canonical_alias_admission": "same physical family only",
        "original_signal_provenance_retained": True,
        "glo_fcn_certification_required": True,
        "ambiguous_multi_code": "global fail or explicit typed miss; never arbitrary merge",
        "duplicate_or_nonmonotonic_time": "global fail",
        "retained_rover_requires_one_finite_support": True,
        "base_unused_rows_allowed": True,
        "base_unused_rows_accounted": True,
        "whole_rover_base_ledger_equality_required": False,
        "equal_epoch_count_required": False,
        "equal_cadence_required": False,
        "row_index_pairing": False,
        "raw_uncorrected_fallback": False,
        "zero_correction_fallback": False,
        "cross_band_fill": False,
        "cross_satellite_fill": False,
        "nearest_hold_or_extrapolation": False,
        "exactly_once_correction": True,
        "solver_invocations_on_inventory_failure": 0,
    }.items():
        assert_equal(inventory.get(key), expected, f"freeze/inventory_contract/{key}")
    routes = freeze.get("routes")
    if not isinstance(routes, list) or [item.get("target") for item in routes] != ["MTV-A", "LAX-T"]:
        raise fail("freeze route order/count changed")
    for item in routes:
        route = item.get("dataset_id")
        if route not in ROUTES:
            raise fail(f"freeze unknown route: {route}")
        assert_equal(item.get("runs"), 1, f"freeze/{route}/runs")
        assert_equal(item.get("solver_invocations_if_inventory_fails"), 0,
                     f"freeze/{route}/solver_invocations_if_inventory_fails")
        assert_equal(item.get("expected_problem_epochs"), PROBLEM_EPOCHS[route],
                     f"freeze/{route}/expected_problem_epochs")
    accounting = freeze.get("read_accounting")
    if not isinstance(accounting, dict):
        raise fail("freeze/read_accounting missing")
    for key, value in accounting.items():
        if key.endswith("copied_or_transformed"):
            assert_equal(value, 0, f"freeze/read_accounting/{key}")
        elif isinstance(value, bool):
            assert_equal(value, False, f"freeze/read_accounting/{key}")
        elif key not in {"source_text_reads", "sealed_metadata_reads"}:
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
    """Return the exact opaque future-run argv, still placeholder-only here."""
    output_route = route.replace("/", "__")
    output_root = "output/smartphone-r5/phase131-canonical-correction-structural-v1/"
    return [
        "build/apps/gnss_fgo_imu_no_base", "--dataset-id", route,
        "--android-gnss", "__PHASE131_RAW_DEVICE_GNSS__",
        "--android-imu", "__PHASE131_RAW_DEVICE_IMU__",
        "--nav", "__PHASE131_RAW_BROADCAST_NAV__",
        "--all-epochs", "--android-raw-utc-keys", "--android-raw-clock-only",
        "--android-utc-wall-clock-fallback", "--native-pdc-imu-tdcp-no-bridge",
        "--native-source-direct-observable-quality", "--native-source-clock-c0d-factor",
        "--native-source-clock-c0d-meter-state-parity",
        "--native-source-clock-c0d-active-solve-diagnostic",
        "--native-source-clock-c0d-gnss-first-meter-state-handoff",
        "--native-source-clock-c0d-epoch-vector-parity",
        "--native-source-clock-c0d-phase99-main-multifrontal-qr-solver",
        "--native-base-pseudorange-compensation",
        "--native-base-pseudorange-source-miss-mask",
        "--native-upstream-position-offset", PHASE118_SELECTOR,
        PHASE126_SELECTOR, PHASE127_SELECTOR, PHASE128_SELECTOR,
        PHASE129_SELECTOR, PHASE130_SELECTOR, PHASE131_SELECTOR,
        "--native-base-rinex", "__PHASE131_RAW_BASE_RINEX__",
        "--native-base-rinex-sha256", "__PHASE131_RAW_BASE_SHA256__",
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
    "--native-base-pseudorange-compensation",
    "--native-base-pseudorange-source-miss-mask",
    "--native-upstream-position-offset", PHASE118_SELECTOR,
    PHASE126_SELECTOR, PHASE127_SELECTOR, PHASE128_SELECTOR,
    PHASE129_SELECTOR, PHASE130_SELECTOR, PHASE131_SELECTOR,
)
FORBIDDEN_FLAGS = (
    PHASE117_SELECTOR, PHASE120_SELECTOR, ADDITIONAL_SELECTOR,
    "--native-source-clock-c0d-gnss-first-raw-drift-d-initializer",
    "--native-direct-wls-ephemeral-c7d-main-seed", "--native-pdc-state-bridge",
    "--native-phase104-stage-main-attribution",
    "--native-phase116-carrier-tdcp-incidence-diagnostic",
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
        "--android-gnss": "__PHASE131_RAW_DEVICE_GNSS__",
        "--android-imu": "__PHASE131_RAW_DEVICE_IMU__",
        "--nav": "__PHASE131_RAW_BROADCAST_NAV__",
        "--native-base-rinex": "__PHASE131_RAW_BASE_RINEX__",
        "--native-base-rinex-sha256": "__PHASE131_RAW_BASE_SHA256__",
    }.items():
        assert_equal(command[command.index(flag) + 1], placeholder,
                     f"manifest/{route}/{flag}/placeholder")


def verify_manifest() -> dict[str, Any]:
    verify_freeze()
    source_pins = verify_static_sources()
    manifest = read_json(MANIFEST, "Phase131 structural manifest")
    for key, expected in {
        "schema_version": SCHEMA,
        "phase": 131,
        "execution_label": "Luna Max",
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
        "commit": IMPLEMENTATION_COMMIT,
        "candidate_id": CANDIDATE_ID,
        "selector": PHASE131_SELECTOR,
        "required_selectors": [PHASE126_SELECTOR, PHASE127_SELECTOR,
                                PHASE128_SELECTOR, PHASE129_SELECTOR,
                                PHASE130_SELECTOR, PHASE131_SELECTOR,
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
        "factor_topology_unchanged": True,
        "equations_units_sigma_filter_lm_unchanged": True,
        "no_fallback": True,
        "no_extrapolation": True,
        "no_zero_correction": True,
        "no_raw_uncorrected_retention": True,
        "literal_tracking_code_in_join": False,
        "original_signal_provenance_retained": True,
        "base_unused_rows_accounted": True,
    }.items():
        assert_equal(implementation.get(key), expected,
                     f"manifest/implementation/{key}")
    assert_equal(implementation.get("source_sha256"), source_pins,
                 "manifest/implementation/source_sha256")
    binary = implementation.get("target_binary")
    if not isinstance(binary, dict):
        raise fail("manifest/implementation/target_binary missing")
    assert_equal(binary.get("path"), relative(BINARY),
                 "manifest/target_binary/path")
    assert_equal(binary.get("sha256"), TARGET_BINARY_SHA256,
                 "manifest/target_binary/sha256")
    artifacts = manifest.get("artifacts")
    if not isinstance(artifacts, dict):
        raise fail("manifest/artifacts missing")
    for key, path in (("validator", Path(__file__)), ("runner", RUNNER),
                      ("focused_tests", FOCUSED_TESTS)):
        pin = artifacts.get(key)
        if not isinstance(pin, dict):
            raise fail(f"manifest/artifacts/{key} missing")
        assert_equal(pin.get("path"), relative(path),
                     f"manifest/artifacts/{key}/path")
        expected = pin.get("sha256")
        if not isinstance(expected, str) or len(expected) != 64:
            raise fail(f"manifest/artifacts/{key}/sha256 missing")
        assert_equal(sha256_static(path, f"manifest/artifacts/{key}"), expected,
                     f"manifest/artifacts/{key}/sha256")
    inventory = manifest.get("inventory_contract")
    if not isinstance(inventory, dict):
        raise fail("manifest/inventory_contract missing")
    for key, expected in {
        "stage": "post-independent-authorization-pre-solver",
        "side_local_conservation": True,
        "canonical_key_conservation": True,
        "same_physical_family_aliases_only": True,
        "original_signal_provenance_retained": True,
        "literal_tracking_code_excluded_from_join": True,
        "certified_glonass_fcn_required": True,
        "unknown_band_explicit_miss": True,
        "ambiguous_multi_code_fail_closed": True,
        "duplicate_or_nonmonotonic_fail_closed": True,
        "retained_rover_exact_key_support": True,
        "exact_endpoint_or_adjacent_two_point_bracket": True,
        "unmatched_rover_explicit_factor_miss": True,
        "unused_base_rows_accounted": True,
        "whole_ledger_equality_required": False,
        "equal_cadence_required": False,
        "equal_row_count_required": False,
        "no_raw_or_zero_fallback": True,
        "no_extrapolation_or_endpoint_hold": True,
        "all_miss_or_empty_route_fail_closed": True,
        "solver_invocations_on_inventory_failure": 0,
    }.items():
        assert_equal(inventory.get(key), expected,
                     f"manifest/inventory_contract/{key}")
    matrix = manifest.get("matrix")
    if not isinstance(matrix, dict):
        raise fail("manifest/matrix missing")
    for key, expected in {
        "candidate_count": 1, "route_count": 2, "runs_per_route": 1,
        "order": "MTV-A then LAX-T", "controls": 0, "reruns": 0,
        "fallbacks": 0, "sweeps": 0,
        "native_solver_invocations_planned": 2,
        "solution_rows_authorized": False, "truth_reads_planned": 0,
        "accuracy_calculations_planned": 0,
    }.items():
        assert_equal(matrix.get(key), expected, f"manifest/matrix/{key}")
    gates = manifest.get("structural_gates")
    if not isinstance(gates, dict):
        raise fail("manifest/structural_gates missing")
    for key, value in gates.items():
        if key == "solver_invocations_on_inventory_failure":
            assert_equal(value, 0, f"manifest/structural_gates/{key}")
        else:
            assert_equal(value, True, f"manifest/structural_gates/{key}")
    routes = manifest.get("routes")
    if not isinstance(routes, list) or [item.get("dataset_id") for item in routes] != list(ROUTES):
        raise fail("manifest route order/count changed")
    for record in routes:
        route = record.get("dataset_id")
        if route not in DOMAIN_ROWS:
            raise fail(f"manifest unknown route: {route}")
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
            "phase130_selector_count": 1,
            "phase131_selector_count": 1,
            "phase118_selector_count": 1,
            "phase117_selector_count": 0,
            "phase120_selector_count": 0,
            "additional_frequency_selector_count": 0,
            "solver_invocations_if_inventory_fails": 0,
        }.items():
            assert_equal(record.get(key), expected,
                         f"manifest/{route}/{key}")
        validate_command(route, record.get("command"))
        raw = record.get("raw_inputs")
        if not isinstance(raw, dict) or set(raw) != set(RAW_NAMES):
            raise fail(f"manifest/{route}/raw_inputs must be GNSS/IMU/nav only")
        for name in RAW_NAMES:
            item = raw[name]
            if not isinstance(item, dict):
                raise fail(f"manifest/{route}/{name} missing")
            for key, expected in {
                "placeholder": f"__PHASE131_RAW_{'DEVICE_GNSS' if name == 'device_gnss.csv' else 'DEVICE_IMU' if name == 'device_imu.csv' else 'BROADCAST_NAV'}__",
                "payload_read_before_authorization": False,
                "copy_or_transform": False,
            }.items():
                assert_equal(item.get(key), expected,
                             f"manifest/{route}/{name}/{key}")
        base = record.get("base_input")
        if not isinstance(base, dict):
            raise fail(f"manifest/{route}/base_input missing")
        for key, expected in {
            "placeholder": "__PHASE131_RAW_BASE_RINEX__",
            "sha256_placeholder": "__PHASE131_RAW_BASE_SHA256__",
            "raw_rinex_only": True,
            "header_inventory_after_authorization": True,
            "payload_read_before_authorization": False,
            "hash_read_before_authorization": False,
            "copy_or_transform": False,
        }.items():
            assert_equal(base.get(key), expected,
                         f"manifest/{route}/base/{key}")
    zero_accounting(manifest.get("read_accounting_before_authorization"),
                    "manifest/read_accounting_before_authorization")
    return manifest


# Source-locked typed aliases used only for synthetic contract tests and
# inventory metadata.  They are not a carrier-frequency table and never read
# a raw payload.
ANDROID_FAMILY = {
    "GPS_L1CA": "L1", "GPS_L1P": "L1", "GPS_L5": "L5",
    "GLO_G1_CA": "L1", "GLO_G1C": "L1", "GLO_L1": "L1",
    "GLO_L1CA": "L1", "GLO_L1P": "L1",
    "GAL_E1": "L1", "GAL_E5A": "L5",
    "BDS_B1I": "L1", "BDS_B1C": "L1", "BDS_B2A": "L5",
    "QZS_L1CA": "L1", "QZS_L5": "L5",
}


def typed_family(system: str, signal: str) -> str | None:
    """Map an existing typed signal to a source-defined L1/L5 family."""
    del system  # system is carried by the key; signal policy owns the family.
    return ANDROID_FAMILY.get(signal)


def rinex_family(system: str, observation_type: str) -> str | None:
    """Map a RINEX system/band code without retaining its literal suffix."""
    if not isinstance(system, str) or not isinstance(observation_type, str):
        return None
    if len(observation_type) < 2 or not observation_type[1].isdigit():
        return None
    band = observation_type[1]
    if band == "1":
        return "L1"
    if band == "5" and system in {"GPS", "GALILEO", "BEIDOU", "QZSS", "NAVIC"}:
        return "L5"
    return None


def canonicalize_typed(system: str, prn: int, family: str | None,
                       fcn: Any = None) -> dict[str, Any]:
    """Construct the canonical key, rejecting uncertified provenance."""
    result: dict[str, Any] = {
        "accepted": False,
        "system": system,
        "prn": prn,
        "family": family,
        "fcn": fcn,
        "reason": None,
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


def canonicalize_row(row: dict[str, Any], label: str = "row") -> dict[str, Any]:
    """Validate one metadata row while retaining literal provenance fields."""
    if not isinstance(row, dict):
        raise fail(f"{label}: row is not an object")
    for key in ("system", "prn", "family", "original_signal", "literal_tracking_code"):
        if key not in row:
            raise fail(f"{label}/{key}: provenance field missing")
    result = canonicalize_typed(row["system"], row["prn"], row["family"],
                                row.get("certified_fcn"))
    result["original_signal"] = row["original_signal"]
    result["literal_tracking_code"] = row["literal_tracking_code"]
    if "value_digest" in row:
        result["value_digest"] = row["value_digest"]
    return result


def verify_canonical_streams(streams: Iterable[dict[str, Any]]) -> dict[str, int]:
    """Validate key collapse and stream uniqueness without choosing a guess."""
    seen: dict[tuple[Any, ...], dict[str, Any]] = {}
    input_rows = certified = misses = alias_rows = 0
    for index, row in enumerate(streams):
        input_rows += 1
        item = canonicalize_row(row, f"canonical_streams/{index}")
        if not item["accepted"]:
            misses += 1
            continue
        certified += 1
        key = item["key"]
        if key in seen:
            previous = seen[key]
            if previous.get("value_digest") != item.get("value_digest"):
                raise fail(f"canonical_streams/{index}: canonical-key-conflict")
            alias_rows += 1
        else:
            seen[key] = item
    return {
        "input_rows": input_rows,
        "certified_rows": certified,
        "explicit_local_miss_rows": misses,
        "canonical_distinct_keys": len(seen),
        "same_family_alias_rows": alias_rows,
    }


def _verify_side(side: dict[str, Any], label: str) -> tuple[int, int, int]:
    if not isinstance(side, dict):
        raise fail(f"{label} missing")
    input_rows = nonnegative_int(side.get("input_rows"), f"{label}/input_rows")
    certified = nonnegative_int(side.get("certified_rows"), f"{label}/certified_rows")
    local_miss = nonnegative_int(side.get("explicit_local_miss_rows"),
                                 f"{label}/explicit_local_miss_rows")
    assert_equal(input_rows, certified + local_miss,
                 f"{label}/side_local_partition")
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
    for key in ("all_rows_classified", "no_raw_uncorrected",
                "no_zero_correction", "no_fallback", "no_extrapolation",
                "finite_certified_wavelength", "canonical_key_accounted",
                "literal_provenance_retained"):
        assert_true(side.get(key), f"{label}/{key}")
    canonical = side.get("canonical")
    if not isinstance(canonical, dict):
        raise fail(f"{label}/canonical missing")
    for key, expected in {
        "input_rows": input_rows,
        "certified_rows": certified,
        "explicit_local_miss_rows": local_miss,
        "all_rows_accounted": True,
        "same_physical_family_aliases_only": True,
        "different_family_rejected": True,
        "unknown_band_explicit_miss": True,
        "literal_tracking_code_in_join": False,
        "original_signal_provenance_retained": True,
        "fcn_certification_fail_closed": True,
    }.items():
        assert_equal(canonical.get(key), expected, f"{label}/canonical/{key}")
    return input_rows, certified, local_miss


def validate_stream_samples(samples: Iterable[dict[str, Any]],
                            label: str = "stream") -> list[tuple[float, float]]:
    """Validate a finite exact-key time stream without deduplication/repair."""
    normalized: list[tuple[float, float]] = []
    for index, sample in enumerate(samples):
        if not isinstance(sample, dict):
            raise fail(f"{label}/{index}: sample is not an object")
        time = finite_number(sample.get("time"), f"{label}/{index}/time")
        value = finite_number(sample.get("value"), f"{label}/{index}/value")
        if normalized and not time > normalized[-1][0]:
            raise fail(f"{label}/{index}: duplicate or non-monotonic time")
        normalized.append((time, value))
    if not normalized:
        raise fail(f"{label}: empty finite stream")
    return normalized


def classify_support(samples: Iterable[dict[str, Any]], query_time: float) -> dict[str, Any]:
    """Classify exact endpoint, interior sample, bracket, or explicit miss."""
    stream = validate_stream_samples(samples)
    query = finite_number(query_time, "query_time")
    if query < stream[0][0] or query > stream[-1][0]:
        return {"supported": False, "reason": "out_of_domain", "support_kind": None}
    for index, (time, value) in enumerate(stream):
        if query == time:
            return {
                "supported": True,
                "reason": None,
                "support_kind": "exact_endpoint" if index in (0, len(stream) - 1) else "exact_sample",
                "value": value,
            }
        if query < time:
            left_time, left_value = stream[index - 1]
            fraction = (query - left_time) / (time - left_time)
            return {
                "supported": True,
                "reason": None,
                "support_kind": "adjacent_two_point_bracket",
                "value": left_value + fraction * (value - left_value),
            }
    raise fail("support classifier reached an impossible state")


def verify_inventory_record(route: str, inventory: dict[str, Any]) -> dict[str, Any]:
    """Validate one synthetic/already-materialized inventory, never a path."""
    if route not in ROUTES:
        raise fail(f"inventory/route unknown: {route}")
    if not isinstance(inventory, dict):
        raise fail("inventory record is not an object")
    assert_equal(inventory.get("route"), route, "inventory/route")
    assert_equal(inventory.get("stage"), "post-independent-authorization-pre-solver",
                 "inventory/stage")
    assert_equal(inventory.get("solver_invocations"), 0,
                 "inventory/solver_invocations")
    assert_true(inventory.get("solver_may_start"), "inventory/solver_may_start")
    rover = _verify_side(inventory.get("rover"), "inventory/rover")
    base = _verify_side(inventory.get("base"), "inventory/base")
    shared = inventory.get("shared_ledger")
    if not isinstance(shared, dict):
        raise fail("inventory/shared_ledger missing")
    for key, expected in {
        "side_local_conservation": True,
        "canonical_key_conservation": True,
        "same_physical_family_aliases_only": True,
        "different_physical_band_rejected": True,
        "literal_tracking_code_in_join": False,
        "original_signal_provenance_retained": True,
        "certified_glonass_fcn_only": True,
        "ambiguous_multi_code_fail_closed": True,
        "duplicate_conflict_fail_closed": True,
        "retained_rover_exact_key_support": True,
        "exact_endpoint_or_adjacent_two_point_bracket": True,
        "unmatched_rover_explicit_factor_miss": True,
        "unused_base_rows_accounted": True,
        "whole_ledger_equality_required": False,
        "cross_side_count_equality_required": False,
        "no_raw_uncorrected": True,
        "no_zero_correction": True,
        "no_fallback": True,
        "no_extrapolation": True,
    }.items():
        assert_equal(shared.get(key), expected, f"inventory/shared_ledger/{key}")
    streams = inventory.get("base_streams")
    if not isinstance(streams, dict):
        raise fail("inventory/base_streams missing")
    finite_samples = nonnegative_int(streams.get("finite_samples"),
                                     "inventory/base_streams/finite_samples")
    unused_rows = nonnegative_int(streams.get("unused_rows"),
                                  "inventory/base_streams/unused_rows")
    unused_streams = nonnegative_int(streams.get("unused_streams"),
                                     "inventory/base_streams/unused_streams")
    used_rows = nonnegative_int(streams.get("used_rows"),
                                "inventory/base_streams/used_rows")
    assert_equal(used_rows + unused_rows, finite_samples,
                 "inventory/base_streams/sample_accounting")
    assert_true(streams.get("all_samples_accounted"),
                "inventory/base_streams/all_samples_accounted")
    assert_true(streams.get("duplicate_nonmonotonic_rejected"),
                "inventory/base_streams/duplicate_nonmonotonic_rejected")
    assert_true(streams.get("canonical_key_streams_accounted"),
                "inventory/base_streams/canonical_key_streams_accounted")
    if finite_samples == 0:
        raise fail("inventory/base_streams has no finite support")
    del unused_streams  # retained for schema validation above
    correction = inventory.get("correction")
    if not isinstance(correction, dict):
        raise fail("inventory/correction missing")
    factor_input = nonnegative_int(correction.get("factor_input_rows"),
                                   "inventory/correction/factor_input_rows")
    retained = nonnegative_int(correction.get("retained_corrected_rows"),
                               "inventory/correction/retained_corrected_rows")
    missing = nonnegative_int(correction.get("missing_exact_stream"),
                              "inventory/correction/missing_exact_stream")
    out_domain = nonnegative_int(correction.get("out_of_domain"),
                                 "inventory/correction/out_of_domain")
    nonfinite = nonnegative_int(correction.get("nonfinite"),
                                "inventory/correction/nonfinite")
    assert_equal(factor_input, retained + missing + out_domain + nonfinite,
                 "inventory/correction/factor_conservation")
    support = correction.get("support")
    if not isinstance(support, dict):
        raise fail("inventory/correction/support missing")
    endpoint = nonnegative_int(support.get("exact_endpoint_rows"),
                               "inventory/correction/support/exact_endpoint_rows")
    bracket = nonnegative_int(support.get("adjacent_two_point_bracket_rows"),
                              "inventory/correction/support/adjacent_two_point_bracket_rows")
    exact_sample = nonnegative_int(support.get("exact_interior_sample_rows"),
                                   "inventory/correction/support/exact_interior_sample_rows")
    assert_equal(endpoint + bracket + exact_sample, retained,
                 "inventory/correction/support/retained")
    for key, expected in {
        "retained_factors_have_exact_key_support": True,
        "one_support_result_per_retained_factor": True,
        "unused_base_rows_accounted": True,
        "application_passes": 1,
        "exactly_once": True,
        "no_duplicate_application": True,
        "no_raw_fallback": True,
        "no_zero_fallback": True,
        "no_nearest_fill": True,
        "no_endpoint_hold": True,
        "no_extrapolation": True,
        "finite_corrected_rows": True,
        "canonical_key_mode": True,
        "factor_topology_changed": False,
    }.items():
        assert_equal(correction.get(key), expected, f"inventory/correction/{key}")
    if retained == 0:
        raise fail("inventory has no retained corrected factor")
    phase = inventory.get("phase131")
    if not isinstance(phase, dict):
        raise fail("inventory/phase131 missing")
    for key, expected in {
        "enabled": True,
        "canonical_key": "GNSSSystem,PRN,physical-frequency-family[,certified-GLO-FCN]",
        "side_local_conservation": True,
        "canonical_key_conservation": True,
        "literal_tracking_code_in_join": False,
        "original_signal_provenance_retained": True,
        "retained_rover_exact_key_support": True,
        "unmatched_rover_explicit_factor_miss": True,
        "unused_base_rows_accounted": True,
        "no_raw_uncorrected": True,
        "no_zero_correction": True,
        "no_fallback": True,
        "no_extrapolation": True,
        "source_complete_a_b_c": True,
    }.items():
        assert_equal(phase.get(key), expected, f"inventory/phase131/{key}")
    return {
        "route": route,
        "inventory_passed": True,
        "solver_invocations": 0,
        "solver_may_start": True,
        "rover_rows": rover[0],
        "base_rows": base[0],
        "rover_certified": rover[1],
        "base_certified": base[1],
        "base_unused_rows": unused_rows,
        "retained_corrected_factor_rows": retained,
    }


def verify_structural_summary(route: str, summary: dict[str, Any]) -> dict[str, Any]:
    """Validate opaque stage/solver telemetry without inspecting solution rows."""
    if not isinstance(summary, dict):
        raise fail("structural summary is not an object")
    assert_equal(summary.get("route"), route, "summary/route")
    assert_equal(summary.get("solution_content_read"), False,
                 "summary/solution_content_read")
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
        initial = finite_number(stage.get("initial_cost"),
                                f"summary/{stage_name}/initial_cost")
        final = finite_number(stage.get("final_cost"),
                              f"summary/{stage_name}/final_cost")
        if not final < initial:
            raise fail(f"summary/{stage_name}/cost is not strictly decreasing")
    assert_equal(summary["main"].get("solver_branch"), "MULTIFRONTAL_QR",
                 "summary/main/solver_branch")
    phase = summary.get("phase131")
    if not isinstance(phase, dict):
        raise fail("summary/phase131 missing")
    for key, expected in {
        "enabled": True,
        "canonical_key": "GNSSSystem,PRN,physical-frequency-family[,certified-GLO-FCN]",
        "side_local_conservation": True,
        "canonical_key_conservation": True,
        "literal_tracking_code_in_join": False,
        "original_signal_provenance_retained": True,
        "retained_rover_exact_key_support": True,
        "unused_base_rows_accounted": True,
        "no_fallback": True,
        "factor_topology_changed": False,
    }.items():
        assert_equal(phase.get(key), expected, f"summary/phase131/{key}")
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
    return {
        "route": route,
        "structural_gate_passed": True,
        "accepted_iterations": {
            name: summary[name]["accepted_iterations"] for name in ("gnss_first", "main")
        },
    }


def verify_pre_raw_static() -> dict[str, Any]:
    """Verify the sealed zero-read artifact and return its accounting."""
    verify_freeze()
    manifest = verify_manifest()
    if not PRE_RAW.is_file():
        raise fail("Phase131 pre-raw accounting artifact is not sealed")
    pre = read_json(PRE_RAW, "Phase131 pre-raw accounting")
    for key, expected in {
        "schema_version": "smartphone-r5-phase131-canonical-correction-structural-pre-raw-accounting.v1",
        "phase": 131,
        "execution_label": "Luna Max",
        "status": "sealed-pre-raw-zero-read",
        "audit_commit": AUDIT_COMMIT,
        "freeze_commit": FREEZE_COMMIT,
        "implementation_commit": IMPLEMENTATION_COMMIT,
        "contract_code_commit": manifest.get("contract_code_commit"),
    }.items():
        assert_equal(pre.get(key), expected, f"pre-raw/{key}")
    manifest_pin = pre.get("manifest_pin")
    if not isinstance(manifest_pin, dict):
        raise fail("pre-raw/manifest_pin missing")
    assert_equal(manifest_pin.get("path"), relative(MANIFEST),
                 "pre-raw/manifest_pin/path")
    assert_equal(manifest_pin.get("sha256"),
                 sha256_static(MANIFEST, "Phase131 manifest"),
                 "pre-raw/manifest_pin/sha256")
    manifest_commit = manifest_pin.get("commit")
    if (not isinstance(manifest_commit, str) or len(manifest_commit) != 40 or
            any(char not in "0123456789abcdef" for char in manifest_commit)):
        raise fail("pre-raw/manifest_pin/commit must be a full lowercase SHA")
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
    except (Phase131ContractError, OSError) as exc:
        print(f"phase131 pre-raw validator: fail-closed: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
