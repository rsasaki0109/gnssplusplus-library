#!/usr/bin/env python3
"""Launch-free Phase130 shared-ledger join-contract validator.

This module validates only sealed source/contract metadata and synthetic
in-memory inventory/summary records.  It contains no raw-input materializer
and no native-process launcher.  A later independently authorized runner may
reuse the pure validators after its own one-pass raw inventory.
"""

from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
import sys
from typing import Any, Iterable


ROOT = Path(__file__).resolve().parents[3]
AUDIT = ROOT / "docs/use_cases/records/smartphone_r5_phase130_shared_ledger_join_semantics_audit_v1.md"
FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_phase130_shared_ledger_join_semantics_freeze_v1.json"
MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase130_shared_ledger_structural_manifest_v1.json"
RUNNER = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase130_shared_ledger_structural_execute.py"
FOCUSED_TESTS = ROOT / "tests/test_smartphone_phase130_shared_ledger_structural.py"
BINARY = ROOT / "build/apps/gnss_fgo_imu_no_base"

AUDIT_COMMIT = "355b6225bcc3bc838dad8a3044bc3aadd34d80f3"
AUDIT_SHA256 = "fcde87b6267daaa2da2dba2210bd0e5baec2658daa8266d53b40bd4bd9256912"
FREEZE_COMMIT = "94085c90ac33ce987c3b9d29a6462c5d77795e88"
FREEZE_SHA256 = "ecc34c4c5daff4a77f6e97da3d35f63710a4dc443d5ff372938ab05c823a6a2f"
TARGET_BINARY_SHA256 = "5c81e9b8f83843e16550553c6add5143fde32ba105cd5f1904b8f4b6cd4b9c4b"

PHASE126_SELECTOR = "--native-phase126-raw-base-source-complete"
PHASE127_SELECTOR = "--native-phase127-glonass-channel-provenance"
PHASE128_SELECTOR = "--native-phase128-glonass-provenance-parser-admission"
PHASE129_SELECTOR = "--native-phase129-glonass-local-miss-mask"
PHASE130_SELECTOR = "--native-phase130-shared-ledger-key-local-support"
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
SCHEMA = "smartphone-r5-phase130-shared-ledger-key-local-support-structural-manifest.v1"
CANDIDATE_ID = "phase130-shared-ledger-key-local-support-v1"

SOURCE_SHA256 = {
    "apps/native/gnss_fgo_imu_no_base.cpp": "3ad1c235721a2ee0b550a01b2446e300d96fa87fd78bfadc6ab7600beb501547",
    "include/libgnss++/algorithms/fgo_config.hpp": "616271e69ec8c1e18ec3da14a710d0716c1c721f44c26b6c46ec77cdc60f1dec",
    "include/libgnss++/algorithms/fgo.hpp": "9d0a966e702b8db3f1498bcf34603173968c685e7ab2260197cc26c44e8bdd36",
    "include/libgnss++/algorithms/base_pseudorange_compensation.hpp": "f6e5e9c1436138e9b0ef943d8b8fa68f3deca51bca650bab71526703d3ad0272",
    "include/libgnss++/algorithms/phase129_glonass_local_miss.hpp": "c3dc7bc699a0bf95d960b045ec08887f13ba757ddccb32ac5dda4354b72d4abe",
    "src/algorithms/base_pseudorange_compensation.cpp": "b6c6a9b6a2f29c66c4f2abecbe8d7e56c4d7cf81e9c145273c3ea1255da6eb9d",
    "src/algorithms/fgo_internal.hpp": "c648f8fd1332a096a771e39e16e2d07aaed38d12345f7917431bb6b32e3fe07d",
    "src/algorithms/fgo_problems.cpp": "b01c9c48f9bdfd470be8aba3bab5247923be4d0ec52d5bc1a3c26f78be139a9c",
    "src/algorithms/source_pseudorange_miss_mask.cpp": "291ff2591dbd62ecba92c24d1d74c686274568f2af6182bc4dc52f237b0b9946",
}


class Phase130ContractError(ValueError):
    """A violation that must fail closed."""


def fail(message: str) -> Phase130ContractError:
    return Phase130ContractError(message)


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
    assert_equal(sha256_static(AUDIT, "Phase130 audit"), AUDIT_SHA256,
                 "freeze/audit_sha256")
    freeze = read_json(FREEZE, "Phase130 freeze")
    for key, expected in {
        "schema_version": "smartphone-r5-phase130-shared-ledger-join-semantics-freeze.v1",
        "phase": 130,
        "execution_label": "Luna Max",
        "status": "sealed-read-only-source-backed-default-off-contract-candidate",
    }.items():
        assert_equal(freeze.get(key), expected, f"freeze/{key}")
    state = freeze.get("starting_state")
    if not isinstance(state, dict):
        raise fail("freeze/starting_state missing")
    for key, expected in {
        "audit_commit": AUDIT_COMMIT,
        "audit_sha256": AUDIT_SHA256,
    }.items():
        assert_equal(state.get(key), expected, f"freeze/starting_state/{key}")
    decision = freeze.get("decision")
    if not isinstance(decision, dict):
        raise fail("freeze/decision missing")
    for key, expected in {
        "candidate_count": 1,
        "candidate_id": CANDIDATE_ID,
        "selector": PHASE130_SELECTOR,
        "source_backed": True,
        "official_requires_rover_base_whole_ledger_equality": False,
        "default_off": True,
        "implementation_authorized": False,
        "raw_materialization_authorized": False,
        "solver_execution_authorized": False,
        "truth_evaluation_authorized": False,
        "accuracy_authorized": False,
        "solution_publication_authorized": False,
        "kaggle_authorized": False,
    }.items():
        assert_equal(decision.get(key), expected, f"freeze/decision/{key}")
    join = freeze.get("join_contract")
    if not isinstance(join, dict):
        raise fail("freeze/join_contract missing")
    for key, expected in {
        "rover_base_row_index_pairing_required": False,
        "rover_base_equal_epoch_count_required": False,
        "rover_base_equal_cadence_required": False,
        "rover_base_equal_key_count_required": False,
        "rover_base_equal_certified_count_required": False,
        "rover_base_equal_miss_reason_map_required": False,
        "base_unused_rows_allowed": True,
        "base_unused_rows_accounted": True,
        "raw_uncorrected_fallback": False,
        "zero_correction_fallback": False,
        "cross_signal_fill": False,
        "cross_satellite_fill": False,
        "nearest_fill": False,
        "endpoint_hold": False,
        "extrapolation": False,
    }.items():
        assert_equal(join.get(key), expected, f"freeze/join_contract/{key}")
    candidate = freeze.get("candidate")
    if not isinstance(candidate, dict):
        raise fail("freeze/candidate missing")
    for key, expected in {
        "id": CANDIDATE_ID,
        "selector": PHASE130_SELECTOR,
        "default_off": True,
        "adds_factor": False,
        "adds_state": False,
        "changes_measurement_equation": False,
        "changes_signal_mapping": False,
        "changes_cadence_or_window": False,
        "changes_interpolation": False,
        "changes_provenance": False,
        "changes_non_glonass_population": False,
        "changes_c7_d_c0d_ccdd": False,
        "changes_solver_or_lm": False,
        "changes_sigma_or_filter": False,
        "implementation_authorized": False,
        "raw_execution_authorized": False,
    }.items():
        assert_equal(candidate.get(key), expected, f"freeze/candidate/{key}")
    accounting = freeze.get("read_accounting")
    if not isinstance(accounting, dict):
        raise fail("freeze/read_accounting missing")
    for key, value in accounting.items():
        if key.startswith("this_freeze_"):
            if key.endswith("reruns_fallbacks_repairs_sweeps"):
                assert_equal(value, 0, f"freeze/read_accounting/{key}")
            elif key.endswith("payload_reads") or key.endswith("reads") or key.endswith("invocations") or key.endswith("calculations") or key.endswith("access") or key.endswith("opened") or key.endswith("generated"):
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
    output_route = route.replace("/", "__")
    output_root = "output/smartphone-r5/phase130-shared-ledger-structural-v1/"
    return [
        "build/apps/gnss_fgo_imu_no_base", "--dataset-id", route,
        "--android-gnss", "__PHASE130_RAW_DEVICE_GNSS__",
        "--android-imu", "__PHASE130_RAW_DEVICE_IMU__",
        "--nav", "__PHASE130_RAW_BROADCAST_NAV__",
        "--all-epochs", "--android-raw-utc-keys", "--android-raw-clock-only",
        "--android-utc-wall-clock-fallback", "--native-pdc-imu-tdcp-no-bridge",
        "--native-source-direct-observable-quality", "--native-source-clock-c0d-factor",
        "--native-source-clock-c0d-meter-state-parity", "--native-source-clock-c0d-active-solve-diagnostic",
        "--native-source-clock-c0d-gnss-first-meter-state-handoff",
        "--native-source-clock-c0d-epoch-vector-parity",
        "--native-source-clock-c0d-phase99-main-multifrontal-qr-solver",
        "--native-base-pseudorange-compensation", "--native-base-pseudorange-source-miss-mask",
        "--native-upstream-position-offset", PHASE118_SELECTOR,
        PHASE126_SELECTOR, PHASE127_SELECTOR, PHASE128_SELECTOR,
        PHASE129_SELECTOR, PHASE130_SELECTOR,
        "--native-base-rinex", "__PHASE130_RAW_BASE_RINEX__",
        "--native-base-rinex-sha256", "__PHASE130_RAW_BASE_SHA256__",
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
    PHASE126_SELECTOR, PHASE127_SELECTOR, PHASE128_SELECTOR,
    PHASE129_SELECTOR, PHASE130_SELECTOR,
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
        "--android-gnss": "__PHASE130_RAW_DEVICE_GNSS__",
        "--android-imu": "__PHASE130_RAW_DEVICE_IMU__",
        "--nav": "__PHASE130_RAW_BROADCAST_NAV__",
        "--native-base-rinex": "__PHASE130_RAW_BASE_RINEX__",
        "--native-base-rinex-sha256": "__PHASE130_RAW_BASE_SHA256__",
    }.items():
        assert_equal(command[command.index(flag) + 1], placeholder,
                     f"manifest/{route}/{flag}/placeholder")


def verify_manifest() -> dict[str, Any]:
    verify_freeze()
    source_pins = verify_static_sources()
    manifest = read_json(MANIFEST, "Phase130 structural manifest")
    for key, expected in {
        "schema_version": SCHEMA,
        "phase": 130,
        "execution_label": "Luna Max",
        "status": "sealed-before-inventory-raw-execution",
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
    commit = implementation.get("commit")
    if not isinstance(commit, str) or len(commit) != 40 or any(c not in "0123456789abcdef" for c in commit):
        raise fail("manifest/implementation/commit must be a full lowercase SHA")
    for key, expected in {
        "candidate_id": CANDIDATE_ID,
        "selector": PHASE130_SELECTOR,
        "required_selectors": [PHASE126_SELECTOR, PHASE127_SELECTOR,
                                PHASE128_SELECTOR, PHASE129_SELECTOR,
                                PHASE130_SELECTOR, PHASE118_SELECTOR],
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
        "whole_ledger_equality_required": False,
        "base_unused_rows_accounted": True,
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
    inventory = manifest.get("inventory_contract")
    if not isinstance(inventory, dict):
        raise fail("manifest/inventory_contract missing")
    for key, expected in {
        "stage": "post-independent-authorization-pre-solver",
        "side_local_conservation": True,
        "retained_rover_exact_key_support": True,
        "exact_endpoint_or_adjacent_two_point_bracket": True,
        "unmatched_rover_explicit_factor_miss": True,
        "unused_base_rows_or_streams_accounted": True,
        "whole_rover_base_ledger_equality": False,
        "equal_cadence_required": False,
        "equal_row_count_required": False,
        "no_raw_or_zero_fallback": True,
        "no_extrapolation_or_endpoint_hold": True,
        "duplicate_nonmonotonic_base_time_global_abort": True,
        "glonass_tie_conflict_global_or_typed_miss": True,
        "all_miss_or_empty_route_fail_closed": True,
        "solver_invocations_on_inventory_failure": 0,
    }.items():
        assert_equal(inventory.get(key), expected, f"manifest/inventory_contract/{key}")
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
            "phase130_selector_count": 1,
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
                "placeholder": f"__PHASE130_RAW_{kind}__",
                "payload_read_before_authorization": False,
                "copy_or_transform": False,
            }.items():
                assert_equal(item.get(key), expected, f"manifest/{route}/{name}/{key}")
        base = record.get("base_input")
        if not isinstance(base, dict):
            raise fail(f"manifest/{route}/base_input missing")
        for key, expected in {
            "placeholder": "__PHASE130_RAW_BASE_RINEX__",
            "sha256_placeholder": "__PHASE130_RAW_BASE_SHA256__",
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
            "before_solver": True,
            "fail_closed": True,
            "every_glonass_row_certified_or_local_miss": True,
            "side_local_conservation": True,
            "retained_rover_exact_key_support": True,
            "unused_base_rows_accounted": True,
            "whole_ledger_equality_required": False,
            "non_glonass_existing_semantics": True,
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
    assert_equal(input_rows, certified + local_miss, f"{label}/side_local_partition")
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
    return input_rows, certified, local_miss


def validate_stream_samples(samples: Iterable[dict[str, Any]], label: str = "stream") -> list[tuple[float, float]]:
    """Validate an exact-key time stream without deduplication or repair."""
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
    """Classify one exact-key query as endpoint, bracket, or explicit miss."""
    stream = validate_stream_samples(samples)
    query = finite_number(query_time, "query_time")
    if query < stream[0][0] or query > stream[-1][0]:
        return {"supported": False, "reason": "out_of_domain", "support_kind": None}
    for index, (time, value) in enumerate(stream):
        if query == time:
            return {"supported": True, "reason": None, "support_kind": "exact_endpoint" if index in (0, len(stream) - 1) else "exact_sample", "value": value}
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
    """Validate synthetic/already-materialized inventory without reading a path."""
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
        "side_local_conservation": True,
        "retained_rover_exact_key_support": True,
        "exact_endpoint_or_adjacent_two_point_bracket": True,
        "unmatched_rover_explicit_factor_miss": True,
        "unused_base_rows_accounted": True,
        "whole_ledger_equality_required": False,
        "cross_side_count_equality_required": False,
        "cross_side_reason_map_equality_required": False,
        "no_raw_uncorrected": True,
        "no_zero_correction": True,
        "no_fallback": True,
        "no_extrapolation": True,
    }.items():
        assert_equal(shared.get(key), expected, f"inventory/shared_ledger/{key}")
    if rover == base:
        # Equal synthetic sides are not invalid, but the contract must still
        # explicitly state that equality is not the admission predicate.
        assert_equal(shared.get("whole_ledger_equality_required"), False,
                     "inventory/shared_ledger/equality-is-not-gate")

    streams = inventory.get("base_streams")
    if not isinstance(streams, dict):
        raise fail("inventory/base_streams missing")
    finite_samples = nonnegative_int(streams.get("finite_samples"), "inventory/base_streams/finite_samples")
    unused_rows = nonnegative_int(streams.get("unused_rows"), "inventory/base_streams/unused_rows")
    unused_streams = nonnegative_int(streams.get("unused_streams"), "inventory/base_streams/unused_streams")
    used_rows = nonnegative_int(streams.get("used_rows"), "inventory/base_streams/used_rows")
    assert_equal(used_rows + unused_rows, finite_samples,
                 "inventory/base_streams/sample_accounting")
    assert_true(streams.get("all_samples_accounted"), "inventory/base_streams/all_samples_accounted")
    assert_true(streams.get("duplicate_nonmonotonic_rejected"),
                "inventory/base_streams/duplicate_nonmonotonic_rejected")
    if finite_samples == 0:
        raise fail("inventory/base_streams has no finite support")

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
    }.items():
        assert_equal(correction.get(key), expected, f"inventory/correction/{key}")
    if retained == 0:
        raise fail("inventory has no retained corrected factor")
    phase = inventory.get("phase130")
    if not isinstance(phase, dict):
        raise fail("inventory/phase130 missing")
    for key, expected in {
        "enabled": True,
        "side_local_conservation": True,
        "whole_ledger_equality_required": False,
        "retained_rover_exact_key_support": True,
        "unmatched_rover_explicit_factor_miss": True,
        "unused_base_rows_accounted": True,
        "no_raw_uncorrected": True,
        "no_zero_correction": True,
        "no_fallback": True,
        "no_extrapolation": True,
        "source_complete_a_b_c": True,
    }.items():
        assert_equal(phase.get(key), expected, f"inventory/phase130/{key}")
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
    assert_equal(summary["main"].get("solver_branch"), "MULTIFRONTAL_QR",
                 "summary/main/solver_branch")
    phase = summary.get("phase130")
    if not isinstance(phase, dict):
        raise fail("summary/phase130 missing")
    for key, expected in {
        "enabled": True,
        "whole_ledger_equality_required": False,
        "side_local_conservation": True,
        "retained_rover_exact_key_support": True,
        "unused_base_rows_accounted": True,
        "no_fallback": True,
    }.items():
        assert_equal(phase.get(key), expected, f"summary/phase130/{key}")
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
    verify_freeze()
    manifest = verify_manifest()
    implementation = manifest["implementation"]
    return {
        "status": "pre-raw-verified-no-payload-activity",
        "phase": 130,
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
        "audit_commit": AUDIT_COMMIT,
        "freeze_commit": FREEZE_COMMIT,
        "freeze_sha256": FREEZE_SHA256,
        "implementation_commit": implementation["commit"],
        "manifest_sha256": sha256_static(MANIFEST, "Phase130 manifest"),
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
    except (Phase130ContractError, OSError) as exc:
        print(f"phase130 pre-raw validator: fail-closed: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
