#!/usr/bin/env python3
"""Launch-free Phase128 inventory-first structural contract validator.

This module validates only tracked source, sealed contract metadata, and the
already-built target binary.  It has no route-input reader and no launch
authority.  The future authorized runner must perform the Stage-1 inventory
before it may invoke the native structural recipe.
"""

from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
import sys
from typing import Any


ROOT = Path(__file__).resolve().parents[3]
AUDIT = ROOT / "docs/use_cases/records/smartphone_r5_phase128_inventory_first_structural_contract_audit_v1.md"
FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_phase128_inventory_first_structural_contract_freeze_v1.json"
MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase128_inventory_first_structural_manifest_v1.json"
RUNNER = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase128_inventory_first_structural_execute.py"
FOCUSED_TESTS = ROOT / "tests/test_smartphone_phase128_inventory_first_structural.py"
BINARY = ROOT / "build/apps/gnss_fgo_imu_no_base"

AUDIT_COMMIT = "c2557ac7b99f2fd2827233900fbe88a10a9b1695"
AUDIT_SHA256 = "58993acacf45107928b10ebd3f406ca1a494102168e46366edb42777369678f7"
FREEZE_COMMIT = "751dbfff4fe21c1d9642a27c016fcab2cd317afa"
FREEZE_SHA256 = "467b35e14ed3235244a6403eb5ac1fbcad777f9e675fd1a08db20209781ac385"
IMPLEMENTATION_COMMIT = "50357e8f3eabbb2eca672b00a16eb2ce32cc2f16"
TARGET_BINARY_SHA256 = "d823b031d865bde973d3bd6ffcf2957288bccf29d66bf89ade3dea11d108bfe0"
HISTORICAL_PHASE127_APP_SHA256 = "af2d74e23706e354b1b9fd715aaa09df807854787d789b0f46f7ee3efd3b5b73"

PHASE126_DESIGN_COMMIT = "583a6c7a4788f82373a4e71436d2953703bc764c"
PHASE126_IMPLEMENTATION_COMMIT = "9e9972667ee009e1bcd0dd1ea4732ae45415c3ce"
PHASE126_FREEZE_COMMIT = "5d00daf56afb6fe6b6e1f01b4a1213c287f671db"
PHASE126_RESULT_COMMIT = "df27546fafa4fb6fbd96a520dcd3fd91acc3c0ae"
PHASE127_DESIGN_COMMIT = "a6df42a774f1b832796b9fa81a1e75217f7cb038"
PHASE127_IMPLEMENTATION_COMMIT = "f41d082e5170fe5dcbebbb4526c7d513f9512b60"
PHASE127_FREEZE_COMMIT = "5c3e66fc4b62d86b52f8e9ee8aa1f26f4cb2a8a4"
PHASE127_RESULT_COMMIT = "25f144f4943b30d6e74bd6582a5d378f0aad7748"

PHASE126_SELECTOR = "--native-phase126-raw-base-source-complete"
PHASE127_SELECTOR = "--native-phase127-glonass-channel-provenance"
PHASE128_SELECTOR = "--native-phase128-glonass-provenance-parser-admission"
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
SCHEMA = "smartphone-r5-phase128-inventory-first-structural-manifest.v1"
CANDIDATE_ID = "phase128-inventory-first-glonass-parser-admission-structural-v1"

IMPLEMENTATION_SOURCES = {
    "apps/native/gnss_fgo_imu_no_base.cpp": ROOT / "apps/native/gnss_fgo_imu_no_base.cpp",
    "include/libgnss++/algorithms/fgo_config.hpp": ROOT / "include/libgnss++/algorithms/fgo_config.hpp",
    "include/libgnss++/algorithms/fgo.hpp": ROOT / "include/libgnss++/algorithms/fgo.hpp",
    "include/libgnss++/algorithms/base_pseudorange_compensation.hpp": ROOT / "include/libgnss++/algorithms/base_pseudorange_compensation.hpp",
    "include/libgnss++/algorithms/phase126_raw_base_compound.hpp": ROOT / "include/libgnss++/algorithms/phase126_raw_base_compound.hpp",
    "include/libgnss++/algorithms/phase127_glonass_channel_provenance.hpp": ROOT / "include/libgnss++/algorithms/phase127_glonass_channel_provenance.hpp",
    "include/libgnss++/algorithms/phase128_glonass_provenance.hpp": ROOT / "include/libgnss++/algorithms/phase128_glonass_provenance.hpp",
    "include/libgnss++/core/glonass_provenance.hpp": ROOT / "include/libgnss++/core/glonass_provenance.hpp",
    "include/libgnss++/core/navigation.hpp": ROOT / "include/libgnss++/core/navigation.hpp",
    "include/libgnss++/io/rinex.hpp": ROOT / "include/libgnss++/io/rinex.hpp",
    "src/algorithms/base_pseudorange_compensation.cpp": ROOT / "src/algorithms/base_pseudorange_compensation.cpp",
    "src/algorithms/fgo_internal.hpp": ROOT / "src/algorithms/fgo_internal.hpp",
    "src/algorithms/fgo_problems.cpp": ROOT / "src/algorithms/fgo_problems.cpp",
    "src/algorithms/phase126_raw_base_compound.cpp": ROOT / "src/algorithms/phase126_raw_base_compound.cpp",
    "src/algorithms/phase127_glonass_channel_provenance.cpp": ROOT / "src/algorithms/phase127_glonass_channel_provenance.cpp",
    "src/algorithms/phase128_glonass_provenance.cpp": ROOT / "src/algorithms/phase128_glonass_provenance.cpp",
    "src/algorithms/source_pseudorange_miss_mask.cpp": ROOT / "src/algorithms/source_pseudorange_miss_mask.cpp",
    "src/core/glonass_provenance.cpp": ROOT / "src/core/glonass_provenance.cpp",
    "src/io/rinex.cpp": ROOT / "src/io/rinex.cpp",
    "src/io/rtcm_decode_nav.cpp": ROOT / "src/io/rtcm_decode_nav.cpp",
    "src/io/android_raw_gnss.cpp": ROOT / "src/io/android_raw_gnss.cpp",
}
SOURCE_SHA256 = {
    "apps/native/gnss_fgo_imu_no_base.cpp": "898b979c51d8fa4ec98432a3f2aeb8a99ff2603689e8cbe396aecad501985138",
    "include/libgnss++/algorithms/fgo_config.hpp": "4cbb8fa4b00acbb8a30c82299993a45bf3c1a0d2b71b73af2feac04cc4394008",
    "include/libgnss++/algorithms/fgo.hpp": "e7e3e76b3d6a37e270212ba980821e0ac6856cba9285ec60d09166938e473179",
    "include/libgnss++/algorithms/base_pseudorange_compensation.hpp": "ecb6b354de926ccb8d479a27cd35db4cd8624da91d367116e21dd065512f1f3d",
    "include/libgnss++/algorithms/phase126_raw_base_compound.hpp": "8e7c18260905cd8c36149cb55d97cd4aa40c6bae2649202f57a5693f728e6f62",
    "include/libgnss++/algorithms/phase127_glonass_channel_provenance.hpp": "dc1de4ae6de270f306f1c07a54ab1799df26344f8cc6fdd910d7abfaf1914c8b",
    "include/libgnss++/algorithms/phase128_glonass_provenance.hpp": "fe743866c3c4815b75e87104523c53d914e4f4e7485fc6f10f5d04b20feade55",
    "include/libgnss++/core/glonass_provenance.hpp": "f3659dd826f742e381f21f3d6f5ecfbb2cfbbbdf637fa66405bc2073f74b1df6",
    "include/libgnss++/core/navigation.hpp": "04f39050e220c2f51e78ed9f7a9398309ed51580d1947ab261573b5e101ea87c",
    "include/libgnss++/io/rinex.hpp": "b08705073aaf944daa5e6e899eebc8856e974d07f09282605dd14461b8e38669",
    "src/algorithms/base_pseudorange_compensation.cpp": "03459df5505a5e32e4ee356126a30742c693f32fe0d6eccc3bfc243c7fe4cca4",
    "src/algorithms/fgo_internal.hpp": "f3980a5190ef1cc59aa479665c7ccbe6fe4ec0096e0aef1aa3eb451b53241acb",
    "src/algorithms/fgo_problems.cpp": "f3f86db94c8cf3f43e25d402e21b5d3f95d9259435e559fbe76e7564af6c3d55",
    "src/algorithms/phase126_raw_base_compound.cpp": "5bbb276c2c22b2d9d2a27d90746cbb2451b26d8cf87b339a36c50a7fe7dc513d",
    "src/algorithms/phase127_glonass_channel_provenance.cpp": "cc8e3105abc8367786b089e8d0492f8217550f0a69473869ae8a89cc01941bb0",
    "src/algorithms/phase128_glonass_provenance.cpp": "5d930d8be16c70c4956de41822160c0c44c1e2761f893a9a491cb9d1a076bbc7",
    "src/algorithms/source_pseudorange_miss_mask.cpp": "291ff2591dbd62ecba92c24d1d74c686274568f2af6182bc4dc52f237b0b9946",
    "src/core/glonass_provenance.cpp": "ae1db078adcc8db21db6cbc7a660d24be21a681dc8668d6d29e1eda13b34e1cf",
    "src/io/rinex.cpp": "308450b371d80105e9d23ff31523a2a7aef32b0cc0a6e69c8b12744c98d0aada",
    "src/io/rtcm_decode_nav.cpp": "10a45b96830eece70d73feda7d79a8a6fa252c84a5678c3aad22c79d8a4af1a1",
    "src/io/android_raw_gnss.cpp": "a020ce900db6a9d3a994cc3adb3dcebbe84cbf67c1ff9793991b7a75039529c5",
}


class Phase128ContractError(ValueError):
    """A contract violation that must fail closed."""


def fail(message: str) -> Phase128ContractError:
    return Phase128ContractError(message)


def relative(path: Path) -> str:
    return str(path.relative_to(ROOT))


def assert_equal(actual: Any, expected: Any, label: str) -> None:
    if actual != expected:
        raise fail(f"{label}: expected {expected!r}, got {actual!r}")


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
    assert_equal(sha256_static(AUDIT, "Phase128 inventory audit"), AUDIT_SHA256,
                 "audit/sha256")
    freeze = read_json(FREEZE, "Phase128 inventory freeze")
    assert_equal(freeze.get("schema_version"),
                 "smartphone-r5-phase128-inventory-first-structural-contract-freeze.v1",
                 "freeze/schema_version")
    assert_equal(freeze.get("phase"), 128, "freeze/phase")
    assert_equal(freeze.get("execution_label"), "Luna Max", "freeze/execution_label")
    assert_equal(freeze.get("status"), "sealed-before-inventory-raw-execution",
                 "freeze/status")
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
        "selector": PHASE128_SELECTOR,
        "phase126_selector": PHASE126_SELECTOR,
        "phase127_selector": PHASE127_SELECTOR,
        "phase118_selector": PHASE118_SELECTOR,
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
    }.items():
        assert_equal(implementation.get(key), expected, f"freeze/implementation/{key}")
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
    }.items():
        assert_equal(inventory.get(key), expected, f"freeze/inventory_stage/{key}")
    gates = freeze.get("stage2_structural_gates")
    if not isinstance(gates, dict):
        raise fail("freeze/stage2_structural_gates missing")
    for key, value in gates.items():
        assert_equal(value, True, f"freeze/stage2_structural_gates/{key}")
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
            PHASE128_SELECTOR, "native_phase128_glonass_provenance_parser_admission = false",
            PHASE127_SELECTOR, PHASE126_SELECTOR,
        ),
        ROOT / "src/core/glonass_provenance.cpp": (
            "data[10]", "raw_channel - 256.0", "-7.0", "6.0",
        ),
        ROOT / "src/io/rinex.cpp": (
            "GlonassFrequencyChannelHeaderStatus::Absent",
            "glonass_canonical_geph_data_valid",
            "decodeCanonicalGlonassGeph(canonical_data)",
        ),
        ROOT / "src/algorithms/phase128_glonass_provenance.cpp": (
            "phase127_glonass::resolve", "geph-canonical-invalid",
            "source_carrier_frequency_hz",
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
    output_root = "output/smartphone-r5/phase128-inventory-first-structural-v1/"
    return [
        "build/apps/gnss_fgo_imu_no_base", "--dataset-id", route,
        "--android-gnss", "__PHASE128_RAW_DEVICE_GNSS__",
        "--android-imu", "__PHASE128_RAW_DEVICE_IMU__",
        "--nav", "__PHASE128_RAW_BROADCAST_NAV__",
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
        "--native-base-rinex", "__PHASE128_RAW_BASE_RINEX__",
        "--native-base-rinex-sha256", "__PHASE128_RAW_BASE_SHA256__",
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
        "--android-gnss": "__PHASE128_RAW_DEVICE_GNSS__",
        "--android-imu": "__PHASE128_RAW_DEVICE_IMU__",
        "--nav": "__PHASE128_RAW_BROADCAST_NAV__",
        "--native-base-rinex": "__PHASE128_RAW_BASE_RINEX__",
        "--native-base-rinex-sha256": "__PHASE128_RAW_BASE_SHA256__",
    }.items():
        assert_equal(command[command.index(flag) + 1], placeholder,
                     f"manifest/{route}/{flag}/placeholder")


def verify_manifest() -> dict[str, Any]:
    freeze = verify_freeze()
    source_pins = verify_implementation()
    manifest = read_json(MANIFEST, "Phase128 inventory-first manifest")
    for key, expected in {"schema_version": SCHEMA, "phase": 128,
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
        "phase126_selector": PHASE126_SELECTOR,
        "phase127_selector": PHASE127_SELECTOR,
        "phase128_selector": PHASE128_SELECTOR,
        "phase118_selector": PHASE118_SELECTOR,
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
        "solver_invocations_on_inventory_failure": 0,
        "header_statuses": ["absent", "valid-empty", "entries", "malformed"],
        "header_absent_or_valid_empty_geph_fallback": True,
        "header_malformed_fail_closed": True,
        "canonical_fields": "data[0..14]",
        "fcn_field": "data[10]",
        "fcn_encoded_gt_128_subtract_256": True,
        "fcn_min": -7,
        "fcn_max": 6,
        "native_gpst_query_and_toe": True,
        "glonass_validity_seconds": 1800.0,
        "per_record_reject_counts": True,
        "unrelated_malformed_record_does_not_poison_valid_record": True,
        "rover_and_base_full_coverage_required": True,
        "mismatch_duplicate_tie_missing_out_of_time_range_fail_closed": True,
        "phone_carrier_frequency_is_fcn_source": False,
        "fixed_channel_or_external_table": False,
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
                "placeholder": f"__PHASE128_RAW_{kind}__",
                "payload_read_before_authorization": False,
                "copy_or_transform": False,
            }.items():
                assert_equal(item.get(key), expected, f"manifest/{route}/{name}/{key}")
        base = record.get("base_input")
        if not isinstance(base, dict):
            raise fail(f"manifest/{route}/base_input missing")
        for key, expected in {
            "placeholder": "__PHASE128_RAW_BASE_RINEX__",
            "sha256_placeholder": "__PHASE128_RAW_BASE_SHA256__",
            "raw_rinex_only": True,
            "header_inventory_after_authorization": True,
            "payload_read_before_authorization": False,
            "hash_read_before_authorization": False,
            "copy_or_transform": False,
            "canonical_nav_inventory_after_authorization": True,
            "rover_base_full_coverage_after_authorization": True,
        }.items():
            assert_equal(base.get(key), expected, f"manifest/{route}/base/{key}")
        gate = record.get("inventory_gate")
        if not isinstance(gate, dict):
            raise fail(f"manifest/{route}/inventory_gate missing")
        for key, expected in {
            "before_solver": True, "fail_closed": True,
            "rover_full_glonass_coverage": True,
            "base_full_glonass_coverage": True,
            "solver_invocations_on_failure": 0,
        }.items():
            assert_equal(gate.get(key), expected, f"manifest/{route}/inventory_gate/{key}")
    zero_accounting(manifest.get("read_accounting_before_authorization"),
                    "manifest/read_accounting_before_authorization")
    return manifest


def verify_inventory_record(route: str, inventory: dict[str, Any]) -> dict[str, Any]:
    """Validate one in-memory Stage-1 record without accepting a path."""
    if not isinstance(inventory, dict):
        raise fail("inventory record is not an object")
    assert_equal(inventory.get("route"), route, "inventory/route")
    assert_equal(inventory.get("stage"), "post-authorization-pre-solver",
                 "inventory/stage")
    assert_equal(inventory.get("solver_invocations"), 0,
                 "inventory/solver_invocations")
    if inventory.get("solver_may_start") is not True:
        raise fail("inventory/solver_may_start must be true only after every gate passes")
    header = inventory.get("header")
    if not isinstance(header, dict):
        raise fail("inventory/header missing")
    if header.get("status") not in {"absent", "valid-empty", "entries", "malformed"}:
        raise fail("inventory/header/status invalid")
    if header.get("status") == "malformed":
        raise fail("inventory/header malformed")
    for key in ("typed_satellite_keys", "native_gpst_query_times",
                "selected_geph_fcn_matches", "carrier_frequency_used_as_fcn"):
        expected = False if key == "carrier_frequency_used_as_fcn" else True
        assert_equal(header.get(key), expected, f"inventory/header/{key}")
    for key in ("malformed_entries", "conflict_entries", "duplicate_entries",
                "unresolved_entries"):
        assert_equal(header.get(key), 0, f"inventory/header/{key}")
    nav = inventory.get("nav")
    if not isinstance(nav, dict):
        raise fail("inventory/nav missing")
    seen = nav.get("records_seen")
    accepted = nav.get("accepted_records")
    rejected = nav.get("rejected_records")
    if not all(isinstance(value, int) and value >= 0 for value in (seen, accepted, rejected)):
        raise fail("inventory/nav record counts must be nonnegative integers")
    assert_equal(seen, accepted + rejected, "inventory/nav/record conservation")
    if not isinstance(nav.get("reject_counts"), dict):
        raise fail("inventory/nav/reject_counts missing")
    for reason, count in nav["reject_counts"].items():
        if not isinstance(reason, str) or not isinstance(count, int) or count < 0:
            raise fail("inventory/nav/reject_counts malformed")
    assert_equal(nav.get("canonical_field_positions"), True,
                 "inventory/nav/canonical_field_positions")
    assert_equal(nav.get("fcn_field"), "data[10]", "inventory/nav/fcn_field")
    assert_equal(nav.get("fcn_encoded_gt_128_subtract_256"), True,
                 "inventory/nav/fcn_normalization")
    geph = inventory.get("geph")
    if not isinstance(geph, dict):
        raise fail("inventory/geph missing")
    query_rows = geph.get("query_rows")
    if not isinstance(query_rows, int) or query_rows < 0:
        raise fail("inventory/geph/query_rows must be a nonnegative integer")
    for key in ("selected_rows", "valid_rows", "finite_fcns", "in_domain_fcns"):
        assert_equal(geph.get(key), query_rows, f"inventory/geph/{key}")
    assert_equal(geph.get("query_times_exact"), True, "inventory/geph/query_times_exact")
    age = geph.get("max_age_s")
    if (not isinstance(age, (int, float)) or not math.isfinite(float(age)) or
            age < 0.0 or age > 1800.0):
        raise fail("inventory/geph/max_age_s exceeds the frozen validity window")
    for key in ("coverage_gaps", "different_fcn_ties", "header_geph_mismatches",
                "missing_fcn", "out_of_range"):
        assert_equal(geph.get(key), 0, f"inventory/geph/{key}")
    coverage = inventory.get("coverage")
    if not isinstance(coverage, dict):
        raise fail("inventory/coverage missing")
    for side in ("rover", "base"):
        rows = coverage.get(f"{side}_glonass_rows")
        certified = coverage.get(f"{side}_certified_rows")
        if not isinstance(rows, int) or rows < 0 or not isinstance(certified, int):
            raise fail(f"inventory/coverage/{side} counts malformed")
        assert_equal(certified, rows, f"inventory/coverage/{side}_certified_rows")
        assert_equal(coverage.get(f"{side}_unresolved_rows"), 0,
                     f"inventory/coverage/{side}_unresolved_rows")
        assert_equal(coverage.get(f"{side}_full"), True,
                     f"inventory/coverage/{side}_full")
    assert_equal(coverage.get("all_finite_positive_frequency_wavelength"), True,
                 "inventory/coverage/finite_frequency_wavelength")
    phase = inventory.get("phase128")
    if not isinstance(phase, dict):
        raise fail("inventory/phase128 missing")
    for key, expected in {
        "enabled": True, "canonical_records_all_admitted": True,
        "header_status_distinguished": True,
        "phone_carrier_frequency_used_as_fcn": False,
        "source_complete": True, "base_correction_exactly_once": True,
    }.items():
        assert_equal(phase.get(key), expected, f"inventory/phase128/{key}")
    return {"route": route, "inventory_passed": True, "solver_invocations": 0,
            "solver_may_start": True, "rover_full_coverage": True,
            "base_full_coverage": True, "query_rows": query_rows,
            "nav_records_seen": seen, "nav_records_rejected": rejected}


def verify_pre_raw_static() -> dict[str, Any]:
    """Return a zero-activity qualification report with no payload access."""
    freeze = verify_freeze()
    manifest = verify_manifest()
    current_app = sha256_static(ROOT / "apps/native/gnss_fgo_imu_no_base.cpp",
                                "historical comparison app")
    if current_app == HISTORICAL_PHASE127_APP_SHA256:
        raise fail("historical Phase127 app hash unexpectedly equals current source")
    return {
        "status": "pre-raw-verified-no-payload-activity",
        "phase": 128,
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
        "solution_output_published": False,
        "historical_phase127_source_hash_check": "mismatch-retained",
        "historical_phase127_expected_app_sha256": HISTORICAL_PHASE127_APP_SHA256,
        "current_phase128_app_sha256": current_app,
        "freeze_commit": FREEZE_COMMIT,
        "freeze_sha256": FREEZE_SHA256,
        "implementation_commit": IMPLEMENTATION_COMMIT,
        "manifest_sha256": sha256_static(MANIFEST, "Phase128 manifest"),
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
    except (Phase128ContractError, OSError) as exc:
        print(f"phase128 pre-raw validator: fail-closed: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
