#!/usr/bin/env python3
"""Launch-free Phase134 native-summary bridge contract.

This module validates only static pins, command snapshots, and synthetic
summary dictionaries.  It never opens a phone GNSS/IMU/navigation/base
payload, a solution or truth row, and it never launches the native binary.
Raw structural execution requires a fresh independent authorization after
this contract is sealed.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
from typing import Any, Mapping, Sequence


ROOT = Path(__file__).resolve().parents[3]
AUDIT = ROOT / "docs/use_cases/records/smartphone_r5_phase134_native_summary_bridge_structural_contract_audit_v1.md"
FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_phase134_native_summary_bridge_structural_contract_freeze_v1.json"
MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase134_native_summary_bridge_structural_manifest_v1.json"
PRE_RAW = ROOT / "docs/use_cases/records/smartphone_r5_phase134_native_summary_bridge_structural_pre_raw_accounting_v1.json"
LAUNCH_FREE_RUNNER = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase134_native_summary_bridge_execute.py"
FOCUSED_TESTS = ROOT / "tests/test_smartphone_phase134_native_summary_bridge_contract.py"
AUTHORIZED_WRAPPER = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase133_runner_native_selector_boundary_authorized_execute.py"
NATIVE_SOURCE = ROOT / "apps/native/gnss_fgo_imu_no_base.cpp"
FGO_HEADER = ROOT / "include/libgnss++/algorithms/fgo.hpp"
BINARY = ROOT / "build/apps/gnss_fgo_imu_no_base"


AUDIT_COMMIT = "a116556d2437f039f0d6947b2b4b445928d64af8"
AUDIT_SHA256 = "d5e62de1e4ad4b2c6219428e5c58a19f0f4b12549a8f0e9b8a27c04626aaa1a3"
FREEZE_COMMIT = "7d00f6e0a43d1b4a8c504db9705a3fa34e67b4cc"
FREEZE_SHA256 = "506c0433556c62f17fdf408e8e34b35238e0b8c579f4dbd796867ffee97e8dbb"
IMPLEMENTATION_COMMIT = "130a7f8f12e191cc12ebd0ff66ad591d732775f7"
NATIVE_SOURCE_SHA256 = "efddfed4104d71db3df13a9b4cd0240bd603e4c78a3be313750dfe96c9273769"
FGO_HEADER_SHA256 = "4f2df010f8b4c73bb1ca83d17811db7672c31d154507f758dc940ee1124b22ae"
AUTHORIZED_WRAPPER_SHA256 = "37b02693ae183edee8435a4247ef474f67e50ecb65773901ebad593900f92fab"
TARGET_BINARY_SHA256 = "3965852271023671cd0e9c6ea3e779ab6f67f4e883d724ccacadf28bd8b2fb79"


PHASE118_SELECTOR = "--native-phase118-official-tdcp-huber-k"
PHASE126_SELECTOR = "--native-phase126-raw-base-source-complete"
PHASE127_SELECTOR = "--native-phase127-glonass-channel-provenance"
PHASE128_SELECTOR = "--native-phase128-glonass-provenance-parser-admission"
PHASE129_SELECTOR = "--native-phase129-glonass-local-miss-mask"
PHASE130_SELECTOR = "--native-phase130-shared-ledger-key-local-support"
PHASE131_SELECTOR = "--native-phase131-canonical-correction-band-key"
PHASE117_SELECTOR = "--native-phase117-tdcp-snr-type-sigma"
PHASE120_SELECTOR = "--native-phase120-official-tdcp-resl-atmosphere-cancellation"
ADDITIONAL_SELECTOR = "--native-base-pseudorange-preserve-additional-frequency-bands"

NATIVE_SELECTORS = (
    PHASE118_SELECTOR,
    PHASE126_SELECTOR,
    PHASE127_SELECTOR,
    PHASE128_SELECTOR,
    PHASE129_SELECTOR,
    PHASE131_SELECTOR,
)
OFF_SELECTORS = (PHASE117_SELECTOR, PHASE120_SELECTOR, ADDITIONAL_SELECTOR)
OTHER_FORBIDDEN_SELECTORS = (
    "--native-direct-wls-ephemeral-c7d-main-seed",
    "--native-pdc-state-bridge",
)

ROUTES = (
    "2021-03-16-18-59-us-ca-mtv-a/pixel5",
    "2022-04-01-18-22-us-ca-lax-t/pixel5",
)
ROUTE_LABELS = {ROUTES[0]: "MTV-A", ROUTES[1]: "LAX-T"}
RAW_NAMES = ("device_gnss.csv", "device_imu.csv", "brdc.nav")
CANDIDATE_ID = "phase134-native-phase131-summary-diagnostics-bridge-v1"
SCHEMA = "smartphone-r5-phase134-native-summary-bridge-structural-manifest.v1"

FORBIDDEN_PATH_TERMS = (
    ".mat",
    "truth",
    "ground_truth",
    "precomputed",
    "coordinate",
    "pdc",
    "kaggle",
    "token",
)

REQUIRED_RECIPE_FLAGS = (
    "--all-epochs",
    "--android-raw-utc-keys",
    "--android-raw-clock-only",
    "--android-utc-wall-clock-fallback",
    "--native-pdc-imu-tdcp-no-bridge",
    "--native-source-direct-observable-quality",
    "--native-source-clock-c0d-factor",
    "--native-source-clock-c0d-meter-state-parity",
    "--native-source-clock-c0d-active-solve-diagnostic",
    "--native-source-clock-c0d-gnss-first-meter-state-handoff",
    "--native-source-clock-c0d-epoch-vector-parity",
    "--native-source-clock-c0d-phase99-main-multifrontal-qr-solver",
    "--native-base-pseudorange-compensation",
    "--native-base-pseudorange-source-miss-mask",
    "--native-upstream-position-offset",
)

CANONICAL_FIELDS = (
    "enabled",
    "configuration_valid",
    "configuration_failure",
    "canonical_rows",
    "canonical_rejected_rows",
    "unknown_band_rows",
    "canonical_key_conflicts",
    "canonical_duplicate_rows",
    "canonical_streams",
    "canonical_selected_streams",
    "canonical_merged_streams",
    "failure_counts",
)

CONSERVATION_FIELDS = (
    "source_miss_mask_enabled",
    "canonical_key_mode",
    "matching_key",
    "original_adopted_pseudorange_rows",
    "retained_finite_pc_pseudorange_rows",
    "dropped_missing_exact_stream_rows",
    "dropped_out_of_domain_rows",
    "dropped_nonfinite_correction_rows",
    "matched_factor_rows",
    "finite_correction_rows_among_matched",
    "source_model_build_count",
    "correction_application_pass_count",
    "corrected_rows",
    "pseudorange_factor_count_consistent",
    "signal_count_consistent",
    "applied",
    "correction_applied_exactly_once",
    "duplicate_correction_rejected",
)

BASE_CANONICAL_MAP = {
    "enabled": "phase131_canonical_correction_band_key",
    "configuration_valid": "phase131_configuration_valid",
    "configuration_failure": "phase131_configuration_failure",
    "canonical_rows": "phase131_canonical_rows",
    "canonical_rejected_rows": "phase131_canonical_rejected_rows",
    "unknown_band_rows": "phase131_unknown_band_rows",
    "canonical_key_conflicts": "phase131_canonical_key_conflicts",
    "canonical_duplicate_rows": "phase131_canonical_duplicate_rows",
    "canonical_streams": "phase131_canonical_streams",
    "canonical_selected_streams": "phase131_canonical_selected_streams",
    "canonical_merged_streams": "phase131_canonical_merged_streams",
    "failure_counts": "phase131_failure_counts",
}


class Phase134ContractError(ValueError):
    """A Phase134 contract violation which must fail closed."""


def fail(message: str) -> Phase134ContractError:
    return Phase134ContractError(message)


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


def sha256_static(path: Path, label: str) -> str:
    """Hash static source/contract artifacts, never payload-like files."""
    lowered = path.name.lower()
    if (
        lowered in set(RAW_NAMES) | {"base.obs", "truth.csv", "ground_truth.csv"}
        or lowered.endswith((".csv", ".nav", ".obs", ".mat"))
        or "truth" in lowered
        or "ground_truth" in lowered
    ):
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


def assert_zero_read_accounting(value: Any, label: str) -> None:
    if not isinstance(value, Mapping):
        raise fail(f"{label} missing")
    for key, item in value.items():
        if key.endswith("copied_or_transformed"):
            assert_equal(item, False, f"{label}/{key}")
        elif isinstance(item, bool):
            assert_equal(item, False, f"{label}/{key}")
        elif isinstance(item, str):
            if item not in {"read-only", "sealed-metadata-only", "not-run"}:
                raise fail(f"{label}/{key}: non-zero read marker {item!r}")
        else:
            assert_equal(item, 0, f"{label}/{key}")


def command_template(route: str) -> list[str]:
    """Return the frozen raw-only argv snapshot without materializing paths."""
    if route not in ROUTES:
        raise fail(f"unknown route: {route}")
    route_dir = route.replace("/", "__")
    output_root = "output/smartphone-r5/phase134-native-summary-bridge-v1/"
    return [
        "build/apps/gnss_fgo_imu_no_base",
        "--dataset-id", route,
        "--android-gnss", "__PHASE134_RAW_DEVICE_GNSS__",
        "--android-imu", "__PHASE134_RAW_DEVICE_IMU__",
        "--nav", "__PHASE134_RAW_BROADCAST_NAV__",
        *REQUIRED_RECIPE_FLAGS,
        *NATIVE_SELECTORS,
        "--native-base-rinex", "__PHASE134_RAW_BASE_RINEX__",
        "--native-base-rinex-sha256", "__PHASE134_RAW_BASE_SHA256__",
        "--out", f"{output_root}{route_dir}/opaque_solution_output.csv",
        "--summary-json", f"{output_root}{route_dir}/native_summary.json",
    ]


def validate_command(route: str, command: Any) -> None:
    if not isinstance(command, list) or any(not isinstance(item, str) for item in command):
        raise fail(f"command/{route} must be a string argv list")
    expected = command_template(route)
    assert_equal(command, expected, f"command/{route}")
    for token in command:
        if token in OFF_SELECTORS or token in OTHER_FORBIDDEN_SELECTORS or token == PHASE130_SELECTOR:
            raise fail(f"forbidden selector forwarded: {route}/{token}")
        if not token.startswith("--") and any(term in token.lower() for term in FORBIDDEN_PATH_TERMS):
            raise fail(f"forbidden path token: {route}/{token}")
    for selector in NATIVE_SELECTORS + REQUIRED_RECIPE_FLAGS:
        assert_equal(command.count(selector), 1, f"command/{route}/{selector}")
    for selector in OFF_SELECTORS + (PHASE130_SELECTOR,) + OTHER_FORBIDDEN_SELECTORS:
        assert_equal(command.count(selector), 0, f"command/{route}/{selector}")
    for flag, placeholder in {
        "--android-gnss": "__PHASE134_RAW_DEVICE_GNSS__",
        "--android-imu": "__PHASE134_RAW_DEVICE_IMU__",
        "--nav": "__PHASE134_RAW_BROADCAST_NAV__",
        "--native-base-rinex": "__PHASE134_RAW_BASE_RINEX__",
        "--native-base-rinex-sha256": "__PHASE134_RAW_BASE_SHA256__",
    }.items():
        assert_equal(command[command.index(flag) + 1], placeholder,
                     f"command/{route}/{flag}/placeholder")
    summary_path = command[command.index("--summary-json") + 1]
    assert_true(summary_path.endswith("/native_summary.json"),
                f"command/{route}/native-summary-path")
    assert_true("structural_summary.json" not in summary_path,
                f"command/{route}/native-summary-not-normalized")


def selector_ownership() -> dict[str, Any]:
    return {
        "native_exactly_once": list(NATIVE_SELECTORS),
        "phase130_runner_only_count": 0,
        "off": list(OFF_SELECTORS),
        "other_forbidden": list(OTHER_FORBIDDEN_SELECTORS),
    }


def _check_artifact_pin(value: Any, path: Path, label: str) -> None:
    if not isinstance(value, Mapping):
        raise fail(f"manifest/artifacts/{label} missing")
    assert_equal(value.get("path"), relative(path), f"manifest/artifacts/{label}/path")
    digest = value.get("sha256")
    if not isinstance(digest, str) or len(digest) != 64:
        raise fail(f"manifest/artifacts/{label}/sha256 missing")
    assert_equal(sha256_static(path, f"manifest/artifacts/{label}"), digest,
                 f"manifest/artifacts/{label}/sha256")


def verify_freeze() -> dict[str, Any]:
    assert_equal(sha256_static(AUDIT, "Phase134 contract audit"), AUDIT_SHA256,
                 "freeze/audit_sha256")
    freeze = read_json(FREEZE, "Phase134 structural freeze")
    for key, expected in {
        "schema_version": "smartphone-r5-phase134-native-summary-bridge-structural-contract-freeze.v1",
        "phase": 134,
        "execution_label": "Luna Max",
        "status": "sealed-launch-free-structural-contract",
    }.items():
        assert_equal(freeze.get(key), expected, f"freeze/{key}")
    audit = freeze.get("audit")
    if not isinstance(audit, Mapping):
        raise fail("freeze/audit missing")
    for key, expected in {
        "path": relative(AUDIT), "commit": AUDIT_COMMIT, "sha256": AUDIT_SHA256,
    }.items():
        assert_equal(audit.get(key), expected, f"freeze/audit/{key}")
    prior = freeze.get("prior_freeze")
    if not isinstance(prior, Mapping):
        raise fail("freeze/prior_freeze missing")
    for key, expected in {
        "commit": "41a9fa8dd52878cd7992313f151014dc9cca0fd8",
        "sha256": "6c9bb8715e465876c7b95096130c3af4c5f9bfaabb038edd3812c71f500e95a0",
        "candidate_id": CANDIDATE_ID,
    }.items():
        assert_equal(prior.get(key), expected, f"freeze/prior_freeze/{key}")
    implementation = freeze.get("implementation")
    if not isinstance(implementation, Mapping):
        raise fail("freeze/implementation missing")
    for key, expected in {
        "commit": IMPLEMENTATION_COMMIT,
        "native_source_sha256": NATIVE_SOURCE_SHA256,
        "diagnostics_header_sha256": FGO_HEADER_SHA256,
        "target_binary_sha256": TARGET_BINARY_SHA256,
        "native_summary_wrapper_sha256": AUTHORIZED_WRAPPER_SHA256,
    }.items():
        assert_equal(implementation.get(key), expected, f"freeze/implementation/{key}")
    candidate = freeze.get("candidate")
    if not isinstance(candidate, Mapping):
        raise fail("freeze/candidate missing")
    for key, expected in {
        "count": 1,
        "id": CANDIDATE_ID,
        "source_backed": True,
        "default_off": True,
        "partial_selector_set_allowed": False,
        "native_algorithm_change": False,
        "factor_topology_changed": False,
        "observation_or_correction_changed": False,
        "equation_or_unit_changed": False,
        "sigma_filter_robust_changed": False,
        "solver_qr_lm_changed": False,
        "legacy_default_changed": False,
        "solution_publication_changed": False,
        "fallback_or_rerun_allowed": False,
    }.items():
        assert_equal(candidate.get(key), expected, f"freeze/candidate/{key}")
    assert_equal(freeze.get("native_selector_ownership"), selector_ownership(),
                 "freeze/native_selector_ownership")
    bridge = freeze.get("bridge_contract")
    if not isinstance(bridge, Mapping):
        raise fail("freeze/bridge_contract missing")
    for key, expected in {
        "source_type": "BasePseudorangeCompensationReport",
        "destination_type": "FGOProblemDiagnostics",
        "sync_count": 1,
        "second_sync": "fail-closed",
        "canonicalization_attempt_formula": "canonical_rows + canonical_rejected_rows",
        "resolver_call_formula": "resolver_call_count = canonicalization_attempt_rows",
        "copy_policy": "exact value copy; no accumulation or recomputation",
        "selector_off_policy": "legacy disabled/zero summary shape remains unchanged",
        "native_summary_path": "native_summary.json",
        "normalized_summary_path": "structural_summary.json",
        "paths_must_be_distinct": True,
        "native_bytes_hash_immutable": True,
        "wrapper_must_not_overwrite_native": True,
    }.items():
        assert_equal(bridge.get(key), expected, f"freeze/bridge_contract/{key}")
    routes = freeze.get("routes")
    if not isinstance(routes, list) or [item.get("target") for item in routes] != ["MTV-A", "LAX-T"]:
        raise fail("freeze route order/count changed")
    for item in routes:
        if item.get("dataset_id") not in ROUTES:
            raise fail(f"freeze unknown route: {item.get('dataset_id')}")
        assert_equal(item.get("runs"), 1, f"freeze/{item.get('target')}/runs")
    matrix = freeze.get("matrix")
    if not isinstance(matrix, Mapping):
        raise fail("freeze/matrix missing")
    for key, expected in {
        "candidate_count": 1,
        "route_order": ["MTV-A", "LAX-T"],
        "runs_per_route": 1,
        "controls": 0,
        "reruns": 0,
        "fallbacks": 0,
        "repairs": 0,
        "sweeps": 0,
        "native_solver_invocations_planned": 2,
        "solution_rows_authorized": False,
        "truth_reads_planned": 0,
        "accuracy_calculations_planned": 0,
    }.items():
        assert_equal(matrix.get(key), expected, f"freeze/matrix/{key}")
    return freeze


def verify_static_sources() -> dict[str, str]:
    sources = {
        relative(NATIVE_SOURCE): (NATIVE_SOURCE, NATIVE_SOURCE_SHA256),
        relative(FGO_HEADER): (FGO_HEADER, FGO_HEADER_SHA256),
        relative(AUTHORIZED_WRAPPER): (AUTHORIZED_WRAPPER, AUTHORIZED_WRAPPER_SHA256),
        relative(BINARY): (BINARY, TARGET_BINARY_SHA256),
    }
    result: dict[str, str] = {}
    for label, (path, expected) in sources.items():
        digest = sha256_static(path, label)
        assert_equal(digest, expected, f"static/{label}/sha256")
        result[label] = digest
    return result


def verify_manifest() -> dict[str, Any]:
    verify_freeze()
    source_pins = verify_static_sources()
    manifest = read_json(MANIFEST, "Phase134 structural manifest")
    for key, expected in {
        "schema_version": SCHEMA,
        "phase": 134,
        "execution_label": "Luna Max",
        "status": "sealed-before-independent-authorization",
        "candidate_id": CANDIDATE_ID,
    }.items():
        assert_equal(manifest.get(key), expected, f"manifest/{key}")
    for section, expected in (
        ("audit", {"path": relative(AUDIT), "commit": AUDIT_COMMIT, "sha256": AUDIT_SHA256}),
        ("freeze", {"path": relative(FREEZE), "commit": FREEZE_COMMIT, "sha256": FREEZE_SHA256}),
    ):
        value = manifest.get(section)
        if not isinstance(value, Mapping):
            raise fail(f"manifest/{section} missing")
        for key, item in expected.items():
            assert_equal(value.get(key), item, f"manifest/{section}/{key}")
    implementation = manifest.get("implementation")
    if not isinstance(implementation, Mapping):
        raise fail("manifest/implementation missing")
    for key, expected in {
        "commit": IMPLEMENTATION_COMMIT,
        "candidate_id": CANDIDATE_ID,
        "default_off": True,
        "native_selector_argv_counts": {flag: 1 for flag in NATIVE_SELECTORS},
        "phase130_argv_count": 0,
        "off_selector_argv_counts": {flag: 0 for flag in OFF_SELECTORS},
        "factor_topology_unchanged": True,
        "native_source_changed": False,
        "native_binary_changed": False,
        "no_fallback": True,
        "no_rerun": True,
        "native_summary_overwrite": False,
        "native_summary_bytes_hash_immutable": True,
    }.items():
        assert_equal(implementation.get(key), expected, f"manifest/implementation/{key}")
    assert_equal(implementation.get("source_sha256"), source_pins,
                 "manifest/implementation/source_sha256")
    artifacts = manifest.get("artifacts")
    if not isinstance(artifacts, Mapping):
        raise fail("manifest/artifacts missing")
    _check_artifact_pin(artifacts.get("validator"), Path(__file__), "validator")
    _check_artifact_pin(artifacts.get("launch_free_runner"), LAUNCH_FREE_RUNNER,
                        "launch_free_runner")
    _check_artifact_pin(artifacts.get("focused_tests"), FOCUSED_TESTS, "focused_tests")
    _check_artifact_pin(artifacts.get("authorized_wrapper"), AUTHORIZED_WRAPPER,
                        "authorized_wrapper")
    assert_equal(manifest.get("selector_ownership"), selector_ownership(),
                 "manifest/selector_ownership")
    bridge = manifest.get("bridge_contract")
    if not isinstance(bridge, Mapping):
        raise fail("manifest/bridge_contract missing")
    assert_equal(bridge.get("native_summary_path"), "native_summary.json",
                 "manifest/bridge_contract/native_summary_path")
    assert_equal(bridge.get("normalized_summary_path"), "structural_summary.json",
                 "manifest/bridge_contract/normalized_summary_path")
    assert_equal(bridge.get("paths_must_be_distinct"), True,
                 "manifest/bridge_contract/paths_must_be_distinct")
    routes = manifest.get("routes")
    if not isinstance(routes, list) or [item.get("dataset_id") for item in routes] != list(ROUTES):
        raise fail("manifest route order/count changed")
    for item in routes:
        route = item.get("dataset_id")
        if route not in ROUTES:
            raise fail(f"manifest unknown route: {route}")
        assert_equal(item.get("target"), ROUTE_LABELS[route], f"manifest/{route}/target")
        assert_equal(item.get("runs"), 1, f"manifest/{route}/runs")
        validate_command(route, item.get("command"))
        raw = item.get("raw_inputs")
        if not isinstance(raw, Mapping) or set(raw) != set(RAW_NAMES):
            raise fail(f"manifest/{route}/raw_inputs must contain GNSS/IMU/nav only")
        for name in RAW_NAMES:
            record = raw[name]
            if not isinstance(record, Mapping):
                raise fail(f"manifest/{route}/{name} missing")
            assert_equal(record.get("payload_read_before_authorization"), False,
                         f"manifest/{route}/{name}/payload_read_before_authorization")
            assert_equal(record.get("copy_or_transform"), False,
                         f"manifest/{route}/{name}/copy_or_transform")
        base = item.get("base_input")
        if not isinstance(base, Mapping):
            raise fail(f"manifest/{route}/base_input missing")
        for key, expected in {
            "raw_rinex_only": True,
            "payload_read_before_authorization": False,
            "hash_read_before_authorization": False,
            "copy_or_transform": False,
        }.items():
            assert_equal(base.get(key), expected, f"manifest/{route}/base/{key}")
    matrix = manifest.get("matrix")
    if not isinstance(matrix, Mapping):
        raise fail("manifest/matrix missing")
    for key, expected in {
        "candidate_count": 1,
        "route_order": ["MTV-A", "LAX-T"],
        "runs_per_route": 1,
        "controls": 0,
        "reruns": 0,
        "fallbacks": 0,
        "native_solver_invocations_planned": 2,
        "solution_rows_authorized": False,
        "truth_reads_planned": 0,
        "accuracy_calculations_planned": 0,
    }.items():
        assert_equal(matrix.get(key), expected, f"manifest/matrix/{key}")
    assert_zero_read_accounting(
        manifest.get("read_accounting_before_authorization"),
        "manifest/read_accounting_before_authorization")
    return manifest


def verify_pre_raw() -> dict[str, Any]:
    verify_manifest()
    pre_raw = read_json(PRE_RAW, "Phase134 pre-raw accounting")
    for key, expected in {
        "schema_version": "smartphone-r5-phase134-native-summary-bridge-structural-pre-raw.v1",
        "phase": 134,
        "status": "sealed-launch-free-zero-read",
        "authorization_commit": None,
    }.items():
        assert_equal(pre_raw.get(key), expected, f"pre_raw/{key}")
    manifest_commit = pre_raw.get("manifest_commit")
    if not isinstance(manifest_commit, str) or len(manifest_commit) != 40 or any(
            char not in "0123456789abcdef" for char in manifest_commit):
        raise fail("pre_raw/manifest_commit must be a full lowercase SHA")
    assert_equal(pre_raw.get("manifest_sha256"),
                 sha256_static(MANIFEST, "Phase134 manifest"),
                 "pre_raw/manifest_sha256")
    pins = pre_raw.get("pins")
    if not isinstance(pins, Mapping):
        raise fail("pre_raw/pins missing")
    for key, expected in {
        "audit_commit": AUDIT_COMMIT,
        "audit_sha256": AUDIT_SHA256,
        "freeze_commit": FREEZE_COMMIT,
        "freeze_sha256": FREEZE_SHA256,
        "implementation_commit": IMPLEMENTATION_COMMIT,
        "target_binary_sha256": TARGET_BINARY_SHA256,
    }.items():
        assert_equal(pins.get(key), expected, f"pre_raw/pins/{key}")
    assert_equal(pins.get("manifest_commit"), manifest_commit,
                 "pre_raw/pins/manifest_commit")
    assert_equal(pins.get("manifest_sha256"), pre_raw.get("manifest_sha256"),
                 "pre_raw/pins/manifest_sha256")
    assert_zero_read_accounting(pre_raw.get("read_accounting"),
                                "pre_raw/read_accounting")
    return pre_raw


def _require_nonnegative_fields(value: Mapping[str, Any], fields: Sequence[str], label: str) -> None:
    for field in fields:
        if field not in value:
            raise fail(f"{label}/{field} missing")
        item = value[field]
        if field.endswith("failure") or field == "matching_key":
            if not isinstance(item, str):
                raise fail(f"{label}/{field} must be a string")
        elif field in {"enabled", "configuration_valid", "source_miss_mask_enabled",
                       "canonical_key_mode", "pseudorange_factor_count_consistent",
                       "signal_count_consistent", "applied",
                       "correction_applied_exactly_once", "duplicate_correction_rejected"}:
            if not isinstance(item, bool):
                raise fail(f"{label}/{field} must be boolean")
        elif field == "failure_counts":
            if not isinstance(item, Mapping):
                raise fail(f"{label}/{field} must be an object")
            for reason, count in item.items():
                nonnegative_int(count, f"{label}/{field}/{reason}")
        else:
            nonnegative_int(item, f"{label}/{field}")


def _base_report_values(native: Mapping[str, Any]) -> tuple[dict[str, Any], dict[str, Any]]:
    base = native.get("native_base_pseudorange_compensation")
    mask = native.get("native_base_pseudorange_source_miss_mask")
    if not isinstance(base, Mapping):
        raise fail("native_base_pseudorange_compensation missing")
    if not isinstance(mask, Mapping):
        raise fail("native_base_pseudorange_source_miss_mask missing")
    canonical: dict[str, Any] = {}
    for destination, source in BASE_CANONICAL_MAP.items():
        if source not in base:
            raise fail(f"native base report/{source} missing")
        canonical[destination] = base[source]
    conservation: dict[str, Any] = {}
    source_locations: dict[str, tuple[Mapping[str, Any], str]] = {
        "source_miss_mask_enabled": (mask, "enabled"),
        "canonical_key_mode": (base, "source_miss_mask_canonical_key_mode"),
        "matching_key": (base, "source_miss_mask_matching_key"),
        "original_adopted_pseudorange_rows": (mask, "original_adopted_pseudorange_rows"),
        "retained_finite_pc_pseudorange_rows": (mask, "retained_finite_pc_pseudorange_rows"),
        "dropped_missing_exact_stream_rows": (mask, "dropped_missing_exact_stream_rows"),
        "dropped_out_of_domain_rows": (mask, "dropped_out_of_domain_rows"),
        "dropped_nonfinite_correction_rows": (mask, "dropped_nonfinite_correction_rows"),
        "matched_factor_rows": (base, "matched_factor_rows"),
        "finite_correction_rows_among_matched": (base, "finite_correction_rows_among_matched"),
        "source_model_build_count": (base, "source_model_build_count"),
        "correction_application_pass_count": (base, "correction_application_pass_count"),
        "corrected_rows": (mask, "corrected_rows"),
        "pseudorange_factor_count_consistent": (mask, "pseudorange_factor_count_consistent"),
        "signal_count_consistent": (mask, "signal_count_consistent"),
        "applied": (base, "applied"),
        "correction_applied_exactly_once": (base, "correction_applied_exactly_once"),
        "duplicate_correction_rejected": (base, "duplicate_correction_rejected"),
    }
    for destination, (source_object, source_field) in source_locations.items():
        if source_field not in source_object:
            raise fail(f"native conservation source/{source_field} missing")
        conservation[destination] = source_object[source_field]
    return canonical, conservation


def validate_bridge_summary(native: Mapping[str, Any], *, require_positive: bool = True,
                            require_nested_base: bool = True,
                            require_applied: bool = True) -> dict[str, Any]:
    """Validate one native summary and return opaque structural evidence."""
    if not isinstance(native, Mapping):
        raise fail("native summary must be an object")
    p131 = native.get("phase131_canonical_correction_band_key")
    if not isinstance(p131, Mapping):
        raise fail("native top-level Phase131 summary missing")
    _require_nonnegative_fields(p131, CANONICAL_FIELDS, "native/phase131")
    bridge = p131.get("diagnostics_bridge")
    if not isinstance(bridge, Mapping):
        raise fail("native/phase131/diagnostics_bridge missing")
    assert_equal(bridge.get("source"), "BasePseudorangeCompensationReport",
                 "native/phase131/diagnostics_bridge/source")
    assert_equal(bridge.get("synchronization_count"), 1,
                 "native/phase131/diagnostics_bridge/synchronization_count")
    assert_equal(bridge.get("exactly_once"), True,
                 "native/phase131/diagnostics_bridge/exactly_once")
    attempts = nonnegative_int(p131.get("canonicalization_attempt_rows"),
                               "native/phase131/canonicalization_attempt_rows")
    resolver = nonnegative_int(p131.get("resolver_call_count"),
                              "native/phase131/resolver_call_count")
    expected_attempts = p131["canonical_rows"] + p131["canonical_rejected_rows"]
    assert_equal(attempts, expected_attempts,
                 "native/phase131/canonicalization_attempt_rows/formula")
    assert_equal(resolver, attempts, "native/phase131/resolver_call_count/formula")
    if require_positive:
        if not p131["enabled"] or not p131["configuration_valid"]:
            raise fail("native Phase131 bridge is not enabled/configuration-valid")
        if p131["canonical_rows"] <= 0 or p131["canonical_selected_streams"] <= 0:
            raise fail("native Phase131 canonical admission is not positive")
        if resolver <= 0:
            raise fail("native Phase131 resolver call count is not positive")
    conservation = p131.get("correction_conservation")
    if not isinstance(conservation, Mapping):
        raise fail("native/phase131/correction_conservation missing")
    _require_nonnegative_fields(conservation, CONSERVATION_FIELDS,
                                "native/phase131/correction_conservation")
    if require_applied:
        if conservation["correction_application_pass_count"] != 1:
            raise fail("native Phase131 correction application pass count is not one")
        if not conservation["correction_applied_exactly_once"]:
            raise fail("native Phase131 correction is not exactly once")
        if conservation["duplicate_correction_rejected"]:
            raise fail("native Phase131 duplicate correction was accepted")
    if conservation["retained_finite_pc_pseudorange_rows"] > conservation["original_adopted_pseudorange_rows"]:
        raise fail("native conservation retained rows exceed original rows")
    if conservation["corrected_rows"] != conservation["retained_finite_pc_pseudorange_rows"]:
        raise fail("native conservation corrected/retained rows differ")

    base_canonical: dict[str, Any] = {}
    base_conservation: dict[str, Any] = {}
    if require_nested_base:
        base_canonical, base_conservation = _base_report_values(native)
        for field in CANONICAL_FIELDS:
            assert_equal(p131[field], base_canonical[field],
                         f"native/base-top-level/{field}")
        for field in CONSERVATION_FIELDS:
            assert_equal(conservation[field], base_conservation[field],
                         f"native/base-top-level/conservation/{field}")
    return {
        "bridge_exactly_once": True,
        "canonicalization_attempt_rows": attempts,
        "resolver_call_count": resolver,
        "resolver_attempt_formula_valid": True,
        "base_top_level_exact_copy": require_nested_base,
        "canonical_conservation_valid": True,
        "reject_reason_conservation_valid": True,
        "solution_content_opened": False,
    }


def validate_summary_views(native_view: Mapping[str, Any], normalized_view: Mapping[str, Any]) -> dict[str, Any]:
    """Validate wrapper metadata without opening either payload."""
    if not isinstance(native_view, Mapping) or not isinstance(normalized_view, Mapping):
        raise fail("summary view metadata must be objects")
    native_path = native_view.get("path")
    normalized_path = normalized_view.get("path")
    if not isinstance(native_path, str) or not isinstance(normalized_path, str):
        raise fail("summary view paths missing")
    if native_path == normalized_path:
        raise fail("native and normalized summary paths must be distinct")
    if not native_path.endswith("native_summary.json"):
        raise fail("native summary path is not native_summary.json")
    if not normalized_path.endswith("structural_summary.json"):
        raise fail("normalized summary path is not structural_summary.json")
    assert_equal(native_view.get("source"), "native-process", "native view/source")
    assert_equal(normalized_view.get("source"), "wrapper-normalized-view",
                 "normalized view/source")
    assert_equal(native_view.get("byte_exact"), True, "native view/byte_exact")
    assert_equal(native_view.get("overwritten"), False, "native view/overwritten")
    assert_equal(normalized_view.get("source_native_path"), native_path,
                 "normalized view/source_native_path")
    native_bytes = nonnegative_int(native_view.get("bytes"), "native view/bytes")
    digest = native_view.get("sha256")
    if native_bytes <= 0 or not isinstance(digest, str) or len(digest) != 64:
        raise fail("native summary byte/hash seal is incomplete")
    return {
        "native_summary_path": native_path,
        "normalized_summary_path": normalized_path,
        "native_summary_bytes": native_bytes,
        "native_summary_sha256": digest,
        "native_summary_overwrite": False,
    }


def verify_synthetic_execution_evidence(evidence: Mapping[str, Any], route: str) -> dict[str, Any]:
    """Validate future-run counters in memory; never launches a process."""
    if route not in ROUTES or not isinstance(evidence, Mapping):
        raise fail("synthetic execution evidence route/object invalid")
    bridge = evidence.get("phase134")
    if not isinstance(bridge, Mapping):
        raise fail("synthetic execution evidence/phase134 missing")
    assert_equal(bridge.get("bridge_sync_count"), 1, "evidence/bridge_sync_count")
    assert_equal(bridge.get("native_summary_overwrite"), False,
                 "evidence/native_summary_overwrite")
    assert_equal(bridge.get("normalized_summary_distinct"), True,
                 "evidence/normalized_summary_distinct")
    assert_true(bridge.get("resolver_call_count", 0) > 0,
                "evidence/resolver_call_count")
    assert_equal(bridge.get("native_solver_invocations"), 0,
                 "evidence/native_solver_invocations/launch-free")
    return {"route": route, "launch_free": True, "bridge_valid": True}


def zero_read_accounting() -> dict[str, Any]:
    return {
        "source_text_reads": "read-only",
        "sealed_metadata_reads": "read-only",
        "raw_phone_gnss_reads": 0,
        "raw_phone_imu_reads": 0,
        "broadcast_navigation_reads": 0,
        "raw_base_rinex_payload_reads": 0,
        "raw_base_header_reads": 0,
        "raw_payload_copies_or_transforms": 0,
        "native_solver_invocations": 0,
        "solution_coordinate_row_reads": 0,
        "truth_reads": 0,
        "accuracy_calculations": 0,
        "mat_pdc_precomputed_coordinate_reads": 0,
        "kaggle_or_token_access": 0,
        "reruns_fallbacks_repairs_sweeps": 0,
    }


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--verify-freeze", action="store_true")
    parser.add_argument("--verify-manifest", action="store_true")
    parser.add_argument("--verify-pre-raw", action="store_true")
    args = parser.parse_args(argv)
    if not any((args.verify_freeze, args.verify_manifest, args.verify_pre_raw)):
        parser.error("launch-free mode requires a verification flag")
    if args.verify_freeze:
        verify_freeze()
    if args.verify_manifest:
        verify_manifest()
    if args.verify_pre_raw:
        verify_pre_raw()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
