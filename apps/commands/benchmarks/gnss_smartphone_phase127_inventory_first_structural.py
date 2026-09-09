#!/usr/bin/env python3
"""Launch-free Phase127 inventory-first structural contract validator.

Only tracked source, the pinned build artifact, and sealed contract metadata
are inspected here.  This module has no route-payload reader and no native
application launch path.  A later, independently authorized runner may bind
the manifest placeholders, but this validator cannot do so.
"""

from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
import sys
from typing import Any


ROOT = Path(__file__).resolve().parents[3]
AUDIT = ROOT / "docs/use_cases/records/smartphone_r5_phase127_inventory_first_structural_contract_audit_v1.md"
FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_phase127_inventory_first_structural_contract_freeze_v1.json"
MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase127_inventory_first_structural_manifest_v1.json"
RUNNER = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase127_inventory_first_structural_execute.py"
FOCUSED_TESTS = ROOT / "tests/test_smartphone_phase127_inventory_first_structural.py"
BINARY = ROOT / "build/apps/gnss_fgo_imu_no_base"

AUDIT_COMMIT = "d4b28a8dbe85ac67bf8957e28de16f072874d072"
AUDIT_SHA256 = "b17964512d9b0026209af3fba22f2c0a270ed8ec0381c94d6ef7003bbeff1ff5"
FREEZE_COMMIT = "5c3e66fc4b62d86b52f8e9ee8aa1f26f4cb2a8a4"
FREEZE_SHA256 = "cc4d1a9262cebb5fb257f16735ab008857e087cebe1e736e1cb5466f0259879c"
PHASE126_DESIGN_COMMIT = "583a6c7a4788f82373a4e71436d2953703bc764c"
PHASE126_IMPLEMENTATION_COMMIT = "9e9972667ee009e1bcd0dd1ea4732ae45415c3ce"
PHASE126_FREEZE_COMMIT = "5d00daf56afb6fe6b6e1f01b4a1213c287f671db"
PHASE126_RESULT_COMMIT = "df27546fafa4fb6fbd96a520dcd3fd91acc3c0ae"
PHASE127_DESIGN_COMMIT = "a6df42a774f1b832796b9fa81a1e75217f7cb038"
IMPLEMENTATION_COMMIT = "f41d082e5170fe5dcbebbb4526c7d513f9512b60"
TARGET_BINARY_SHA256 = "653797970fbe65f199fc98dfe47ddd57ec26da66fadc4df40dff930a6d1c436a"
HISTORICAL_PHASE126_APP_SHA256 = "1c9340a6caf05e16262c947c88d1e286baf3715f93f77a65f0ce34f38fefa95a"

SELECTOR = "--native-phase127-glonass-channel-provenance"
PHASE126_SELECTOR = "--native-phase126-raw-base-source-complete"
PHASE118_SELECTOR = "--native-phase118-official-tdcp-huber-k"
PHASE117_SELECTOR = "--native-phase117-tdcp-snr-type-sigma"
PHASE120_SELECTOR = "--native-phase120-official-tdcp-resl-atmosphere-cancellation"
ADDITIONAL_BAND_SELECTOR = "--native-base-pseudorange-preserve-additional-frequency-bands"
ROUTES = (
    "2021-03-16-18-59-us-ca-mtv-a/pixel5",
    "2022-04-01-18-22-us-ca-lax-t/pixel5",
)
DOMAIN_ROWS = {ROUTES[0]: 2158, ROUTES[1]: 1465}
PROBLEM_EPOCHS = {route: rows + 1 for route, rows in DOMAIN_ROWS.items()}
RAW_NAMES = ("device_gnss.csv", "device_imu.csv", "brdc.nav")
SCHEMA = "smartphone-r5-phase127-inventory-first-structural-manifest.v1"
CANDIDATE_ID = "phase127-inventory-first-raw-base-glonass-structural-v1"

# These are source pins for the already committed Phase127 implementation.
# They are intentionally separate from route payload metadata.
IMPLEMENTATION_SOURCES = {
    "apps/native/gnss_fgo_imu_no_base.cpp": ROOT / "apps/native/gnss_fgo_imu_no_base.cpp",
    "include/libgnss++/algorithms/fgo_config.hpp": ROOT / "include/libgnss++/algorithms/fgo_config.hpp",
    "include/libgnss++/algorithms/fgo.hpp": ROOT / "include/libgnss++/algorithms/fgo.hpp",
    "include/libgnss++/algorithms/base_pseudorange_compensation.hpp": ROOT / "include/libgnss++/algorithms/base_pseudorange_compensation.hpp",
    "include/libgnss++/algorithms/phase126_raw_base_compound.hpp": ROOT / "include/libgnss++/algorithms/phase126_raw_base_compound.hpp",
    "include/libgnss++/algorithms/phase127_glonass_channel_provenance.hpp": ROOT / "include/libgnss++/algorithms/phase127_glonass_channel_provenance.hpp",
    "src/algorithms/base_pseudorange_compensation.cpp": ROOT / "src/algorithms/base_pseudorange_compensation.cpp",
    "src/algorithms/fgo_internal.hpp": ROOT / "src/algorithms/fgo_internal.hpp",
    "src/algorithms/fgo_problems.cpp": ROOT / "src/algorithms/fgo_problems.cpp",
    "src/algorithms/phase126_raw_base_compound.cpp": ROOT / "src/algorithms/phase126_raw_base_compound.cpp",
    "src/algorithms/phase127_glonass_channel_provenance.cpp": ROOT / "src/algorithms/phase127_glonass_channel_provenance.cpp",
    "src/algorithms/source_pseudorange_miss_mask.cpp": ROOT / "src/algorithms/source_pseudorange_miss_mask.cpp",
    "src/io/rinex.cpp": ROOT / "src/io/rinex.cpp",
    "include/libgnss++/io/rinex.hpp": ROOT / "include/libgnss++/io/rinex.hpp",
    "include/libgnss++/core/navigation.hpp": ROOT / "include/libgnss++/core/navigation.hpp",
    "include/libgnss++/core/types.hpp": ROOT / "include/libgnss++/core/types.hpp",
    "src/io/rtcm_decode_nav.cpp": ROOT / "src/io/rtcm_decode_nav.cpp",
}
SOURCE_SHA256 = {
    "apps/native/gnss_fgo_imu_no_base.cpp": "af2d74e23706e354b1b9fd715aaa09df807854787d789b0f46f7ee3efd3b5b73",
    "include/libgnss++/algorithms/fgo_config.hpp": "11970d8bc97d3536e4afec99f997b4e641e9e5f7296a4579ef5b9173188cecff",
    "include/libgnss++/algorithms/fgo.hpp": "fabb24bfa68f745beff94d919a384b20ff66799fca0150aaec9a06aa58b0df7e",
    "include/libgnss++/algorithms/base_pseudorange_compensation.hpp": "575bb46f56a020ba8e6c3cc39f9220dd5a13e8240cbcb8370d140a444869d8da",
    "include/libgnss++/algorithms/phase126_raw_base_compound.hpp": "8e7c18260905cd8c36149cb55d97cd4aa40c6bae2649202f57a5693f728e6f62",
    "include/libgnss++/algorithms/phase127_glonass_channel_provenance.hpp": "dc1de4ae6de270f306f1c07a54ab1799df26344f8cc6fdd910d7abfaf1914c8b",
    "src/algorithms/base_pseudorange_compensation.cpp": "47f897477b3a88aa84c3169fbca85a10cb7d45358f6c23ddc304b067acecee94",
    "src/algorithms/fgo_internal.hpp": "6437c5918d6a89d2495bd89d536a9855986f0ac70d3910c35a59e939b5747620",
    "src/algorithms/fgo_problems.cpp": "f55d547aaef402e559b2f3f6d224cd6c9365758aeddd2e52767a569a36dd9661",
    "src/algorithms/phase126_raw_base_compound.cpp": "5bbb276c2c22b2d9d2a27d90746cbb2451b26d8cf87b339a36c50a7fe7dc513d",
    "src/algorithms/phase127_glonass_channel_provenance.cpp": "cc8e3105abc8367786b089e8d0492f8217550f0a69473869ae8a89cc01941bb0",
    "src/algorithms/source_pseudorange_miss_mask.cpp": "291ff2591dbd62ecba92c24d1d74c686274568f2af6182bc4dc52f237b0b9946",
    "src/io/rinex.cpp": "6fa2778e871587d105382fbd3c20589ccfcf5378cc73f62cb2e33ddbb5cf2771",
    "include/libgnss++/io/rinex.hpp": "2ee2500c0b53b13cc0308e9c94923f6abc9c5c689e10adb31ffb8adb6dc2bff3",
    "include/libgnss++/core/navigation.hpp": "5821a81c149061b544acc5efd761bb63ec3379c55308e2fe26235ee0ab1b81a3",
    "include/libgnss++/core/types.hpp": "475d524280fec85d3e959e318447f917a614d93314207a04f84501b0f7735ee3",
    "src/io/rtcm_decode_nav.cpp": "10a45b96830eece70d73feda7d79a8a6fa252c84a5678c3aad22c79d8a4af1a1",
}


class Phase127ContractError(ValueError):
    """A contract violation; callers must fail closed."""


def fail(message: str) -> Phase127ContractError:
    return Phase127ContractError(message)


def relative(path: Path) -> str:
    return str(path.relative_to(ROOT))


def assert_equal(actual: Any, expected: Any, label: str) -> None:
    if actual != expected:
        raise fail(f"{label}: expected {expected!r}, got {actual!r}")


def sha256_file(path: Path, label: str) -> str:
    """Hash only a known static artifact; reject payload-like paths."""
    lowered = path.name.lower()
    if (lowered in RAW_NAMES or lowered == "base.obs" or lowered.endswith((".csv", ".nav", ".obs", ".mat"))
            or "truth" in lowered or "ground_truth" in lowered):
        raise fail(f"payload hash forbidden before authorization: {label}")
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
        raise fail(f"failed to read {label}: {exc}") from exc
    if not isinstance(value, dict):
        raise fail(f"{label} is not an object")
    return value


def verify_freeze() -> dict[str, Any]:
    assert_equal(sha256_file(AUDIT, "Phase127 inventory audit"), AUDIT_SHA256, "audit/sha256")
    assert_equal(sha256_file(FREEZE, "Phase127 inventory freeze"), FREEZE_SHA256, "freeze/sha256")
    freeze = read_json(FREEZE, "Phase127 inventory freeze")
    for key, expected in {
        "schema_version": "smartphone-r5-phase127-inventory-first-structural-contract-freeze.v1",
        "phase": 127,
        "execution_label": "Luna Max",
        "status": "sealed-before-inventory-raw-execution",
    }.items():
        assert_equal(freeze.get(key), expected, f"freeze/{key}")
    audit = freeze.get("contract_audit")
    if not isinstance(audit, dict):
        raise fail("freeze/contract_audit missing")
    for key, expected in {"path": relative(AUDIT), "commit": AUDIT_COMMIT, "sha256": AUDIT_SHA256}.items():
        assert_equal(audit.get(key), expected, f"freeze/contract_audit/{key}")
    design = freeze.get("authoritative_design")
    if not isinstance(design, dict):
        raise fail("freeze/authoritative_design missing")
    for key, expected in {
        "phase126_design_commit": PHASE126_DESIGN_COMMIT,
        "phase126_implementation_commit": PHASE126_IMPLEMENTATION_COMMIT,
        "phase126_structural_freeze_commit": PHASE126_FREEZE_COMMIT,
        "phase126_structural_result_commit": PHASE126_RESULT_COMMIT,
        "phase127_design_freeze_commit": PHASE127_DESIGN_COMMIT,
        "phase127_implementation_commit": IMPLEMENTATION_COMMIT,
    }.items():
        assert_equal(design.get(key), expected, f"freeze/authoritative_design/{key}")
    candidate = freeze.get("candidate")
    if not isinstance(candidate, dict):
        raise fail("freeze/candidate missing")
    for key, expected in {
        "candidate_count": 1,
        "candidate_id": CANDIDATE_ID,
        "selector": SELECTOR,
        "phase126_selector": PHASE126_SELECTOR,
        "phase118_selector": PHASE118_SELECTOR,
        "phase117_dynamic_sigma": False,
        "phase120_official_tdcp_resl_atmosphere_cancellation": False,
        "phase109_additional_frequency_bands": False,
        "default_off": True,
        "legacy_default_unchanged": True,
        "factor_topology_unchanged": True,
        "admission_unchanged": True,
        "raw_execution_authorized": False,
        "solver_execution_authorized": False,
        "truth_evaluation_authorized": False,
        "accuracy_authorized": False,
        "kaggle_authorized": False,
    }.items():
        assert_equal(candidate.get(key), expected, f"freeze/candidate/{key}")
    target = freeze.get("target")
    if not isinstance(target, dict):
        raise fail("freeze/target missing")
    assert_equal(target.get("implementation_head"), IMPLEMENTATION_COMMIT, "freeze/target/implementation_head")
    assert_equal(target.get("binary_path"), relative(BINARY), "freeze/target/binary_path")
    assert_equal(target.get("binary_sha256"), TARGET_BINARY_SHA256, "freeze/target/binary_sha256")
    inventory = freeze.get("inventory_stage")
    if not isinstance(inventory, dict):
        raise fail("freeze/inventory_stage missing")
    for key, expected in {
        "stage": "post-independent-authorization-pre-solver",
        "solver_forbidden": True,
        "solver_invocations_before_stage2": 0,
        "failure_status": "inventory-fail-closed",
    }.items():
        assert_equal(inventory.get(key), expected, f"freeze/inventory_stage/{key}")
    gates = freeze.get("stage2_structural_gates")
    if not isinstance(gates, dict):
        raise fail("freeze/stage2_structural_gates missing")
    for key, value in gates.items():
        assert_equal(value, True, f"freeze/stage2_structural_gates/{key}")
    accounting = freeze.get("read_accounting_before_authorization")
    if not isinstance(accounting, dict):
        raise fail("freeze/read_accounting_before_authorization missing")
    for key, value in accounting.items():
        if key.endswith("copied_or_transformed"):
            assert_equal(value, False, f"freeze/read_accounting/{key}")
        else:
            assert_equal(value, 0, f"freeze/read_accounting/{key}")
    authorization = freeze.get("authorization_boundary")
    if not isinstance(authorization, dict):
        raise fail("freeze/authorization_boundary missing")
    for key in ("raw_materialization_authorized", "raw_inventory_reads_authorized",
                "solver_execution_authorized", "truth_evaluation_authorized",
                "accuracy_authorized", "solution_publication_authorized"):
        assert_equal(authorization.get(key), False, f"freeze/authorization_boundary/{key}")
    return freeze


def verify_implementation() -> dict[str, str]:
    actual: dict[str, str] = {}
    for name, path in IMPLEMENTATION_SOURCES.items():
        actual[name] = sha256_file(path, f"implementation/{name}")
        assert_equal(actual[name], SOURCE_SHA256[name], f"implementation/{name}/sha256")
    assert_equal(sha256_file(BINARY, "target binary"), TARGET_BINARY_SHA256, "target binary/sha256")
    markers = {
        ROOT / "apps/native/gnss_fgo_imu_no_base.cpp": (
            SELECTOR, "phase127_glonass_channel_provenance", "phase127_query_time_coverage_gaps",
        ),
        ROOT / "src/io/rinex.cpp": ("GLONASS SLOT / FRQ #", "glonass_frequency_channel_present"),
        ROOT / "src/algorithms/phase127_glonass_channel_provenance.cpp": (
            "getEphemeris(satellite, query_time)", "query-time-coverage-gap", "isValidFrequencyChannel",
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
    output_root = "output/smartphone-r5/phase127-inventory-first-structural-v1/"
    return [
        "build/apps/gnss_fgo_imu_no_base", "--dataset-id", route,
        "--android-gnss", "__PHASE127_RAW_DEVICE_GNSS__",
        "--android-imu", "__PHASE127_RAW_DEVICE_IMU__",
        "--nav", "__PHASE127_RAW_BROADCAST_NAV__",
        "--all-epochs", "--android-raw-utc-keys", "--android-raw-clock-only",
        "--android-utc-wall-clock-fallback", "--native-pdc-imu-tdcp-no-bridge",
        "--native-source-direct-observable-quality", "--native-source-clock-c0d-factor",
        "--native-source-clock-c0d-meter-state-parity", "--native-source-clock-c0d-active-solve-diagnostic",
        "--native-source-clock-c0d-gnss-first-meter-state-handoff",
        "--native-source-clock-c0d-epoch-vector-parity",
        "--native-source-clock-c0d-phase99-main-multifrontal-qr-solver",
        "--native-base-pseudorange-compensation", "--native-base-pseudorange-source-miss-mask",
        "--native-upstream-position-offset", PHASE118_SELECTOR,
        PHASE126_SELECTOR, SELECTOR,
        "--native-base-rinex", "__PHASE127_RAW_BASE_RINEX__",
        "--native-base-rinex-sha256", "__PHASE127_RAW_BASE_SHA256__",
        "--out", f"{output_root}{output_route}/opaque_solution_output.csv",
        "--summary-json", f"{output_root}{output_route}/structural_summary.json",
    ]


REQUIRED_FLAGS = (
    "--all-epochs", "--android-raw-utc-keys", "--android-raw-clock-only",
    "--android-utc-wall-clock-fallback", "--native-pdc-imu-tdcp-no-bridge",
    "--native-source-direct-observable-quality", "--native-source-clock-c0d-factor",
    "--native-source-clock-c0d-meter-state-parity", "--native-source-clock-c0d-active-solve-diagnostic",
    "--native-source-clock-c0d-gnss-first-meter-state-handoff", "--native-source-clock-c0d-epoch-vector-parity",
    "--native-source-clock-c0d-phase99-main-multifrontal-qr-solver",
    "--native-base-pseudorange-compensation", "--native-base-pseudorange-source-miss-mask",
    "--native-upstream-position-offset", PHASE118_SELECTOR, PHASE126_SELECTOR, SELECTOR,
)
FORBIDDEN_FLAGS = (
    PHASE117_SELECTOR, PHASE120_SELECTOR, ADDITIONAL_BAND_SELECTOR,
    "--native-source-clock-c0d-gnss-first-raw-drift-d-initializer",
    "--native-direct-wls-ephemeral-c7d-main-seed", "--native-pdc-state-bridge",
    "--native-source-clock-c0d-phase94-stage-diagnostics", "--native-source-clock-c0d-phase96-main-diagnostics",
    "--native-source-clock-c0d-phase97-singular-system-diagnostics", "--native-source-clock-c0d-phase98-solver-rank-diagnostic",
    "--native-phase104-stage-main-attribution", "--native-phase116-carrier-tdcp-incidence-diagnostic",
    "--native-upstream-quality", "--obs",
)
FORBIDDEN_PATH_TERMS = (".mat", "truth", "ground_truth", "precomputed", "coordinate", "pdc", "kaggle", "token")


def validate_command(route: str, command: Any) -> None:
    assert_equal(command, command_template(route), f"manifest/{route}/command")
    if not isinstance(command, list):
        raise fail(f"manifest/{route}/command is not argv")
    for token in command:
        if token in FORBIDDEN_FLAGS:
            raise fail(f"forbidden flag: {route}/{token}")
        if not token.startswith("--") and any(term in token.lower() for term in FORBIDDEN_PATH_TERMS):
            raise fail(f"forbidden path token: {route}/{token}")
    for flag in REQUIRED_FLAGS:
        assert_equal(command.count(flag), 1, f"manifest/{route}/{flag}")
    assert_equal(command.count(PHASE117_SELECTOR), 0, f"manifest/{route}/Phase117")
    assert_equal(command.count(PHASE120_SELECTOR), 0, f"manifest/{route}/Phase120")
    assert_equal(command.count(ADDITIONAL_BAND_SELECTOR), 0, f"manifest/{route}/additional-band")
    for flag, placeholder in {
        "--android-gnss": "__PHASE127_RAW_DEVICE_GNSS__",
        "--android-imu": "__PHASE127_RAW_DEVICE_IMU__",
        "--nav": "__PHASE127_RAW_BROADCAST_NAV__",
        "--native-base-rinex": "__PHASE127_RAW_BASE_RINEX__",
        "--native-base-rinex-sha256": "__PHASE127_RAW_BASE_SHA256__",
    }.items():
        assert_equal(command[command.index(flag) + 1], placeholder, f"manifest/{route}/{flag}/placeholder")


def _zero_accounting(value: Any, label: str) -> None:
    if not isinstance(value, dict):
        raise fail(f"{label} missing")
    for key, item in value.items():
        if key.endswith("copied_or_transformed"):
            assert_equal(item, False, f"{label}/{key}")
        else:
            assert_equal(item, 0, f"{label}/{key}")


def verify_manifest() -> dict[str, Any]:
    freeze = verify_freeze()
    source_pins = verify_implementation()
    manifest = read_json(MANIFEST, "Phase127 inventory-first manifest")
    for key, expected in {"schema_version": SCHEMA, "phase": 127, "execution_label": "Luna Max",
                          "status": "sealed-before-inventory-raw-execution"}.items():
        assert_equal(manifest.get(key), expected, f"manifest/{key}")
    for section, expected in (
        ("freeze", {"path": relative(FREEZE), "commit": FREEZE_COMMIT, "sha256": FREEZE_SHA256}),
        ("contract_audit", {"path": relative(AUDIT), "commit": AUDIT_COMMIT, "sha256": AUDIT_SHA256}),
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
        "phase127_selector": SELECTOR,
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
    assert_equal(implementation.get("source_sha256"), source_pins, "manifest/implementation/source_sha256")
    binary = implementation.get("binary")
    if not isinstance(binary, dict):
        raise fail("manifest/implementation/binary missing")
    assert_equal(binary.get("path"), relative(BINARY), "manifest/implementation/binary/path")
    assert_equal(binary.get("sha256"), TARGET_BINARY_SHA256, "manifest/implementation/binary/sha256")
    artifacts = manifest.get("artifacts")
    if not isinstance(artifacts, dict):
        raise fail("manifest/artifacts missing")
    for key, path in (("validator", Path(__file__)), ("runner", RUNNER), ("focused_tests", FOCUSED_TESTS)):
        pin = artifacts.get(key)
        if not isinstance(pin, dict):
            raise fail(f"manifest/artifacts/{key} missing")
        assert_equal(pin.get("path"), relative(path), f"manifest/artifacts/{key}/path")
        expected = pin.get("sha256")
        if not isinstance(expected, str) or len(expected) != 64:
            raise fail(f"manifest/artifacts/{key}/sha256 missing")
        assert_equal(sha256_file(path, f"manifest/artifacts/{key}"), expected,
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
        "truth_mat_pdc_precomputed_coordinate_phone_coordinate_accuracy_kaggle": False,
    }.items():
        assert_equal(raw_contract.get(key), expected, f"manifest/raw_input_contract/{key}")
    inventory_contract = manifest.get("inventory_contract")
    if not isinstance(inventory_contract, dict):
        raise fail("manifest/inventory_contract missing")
    for key, expected in {
        "stage": "post-independent-authorization-pre-solver",
        "solver_invocations_on_inventory_failure": 0,
        "header_primary": True,
        "selected_geph_exact_query_time": True,
        "glonass_validity_seconds": 1800.0,
        "fcn_min": -7,
        "fcn_max": 6,
        "rover_and_base_full_coverage_required": True,
        "mismatch_duplicate_conflict_missing_fail_closed": True,
        "fixed_channel_or_external_table": False,
    }.items():
        assert_equal(inventory_contract.get(key), expected, f"manifest/inventory_contract/{key}")
    matrix = manifest.get("matrix")
    if not isinstance(matrix, dict):
        raise fail("manifest/matrix missing")
    for key, expected in {"candidate_count": 1, "route_count": 2, "runs_per_route": 1,
                          "order": "MTV-A then LAX-T", "native_solver_invocations_planned": 2,
                          "controls": 0, "reruns": 0, "fallbacks": 0,
                          "solution_rows_authorized": False, "truth_reads_planned": 0,
                          "accuracy_calculations_planned": 0}.items():
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
            kind = "DEVICE_GNSS" if name == "device_gnss.csv" else "DEVICE_IMU" if name == "device_imu.csv" else "BROADCAST_NAV"
            for key, expected in {"placeholder": f"__PHASE127_RAW_{kind}__",
                                  "payload_read_before_authorization": False,
                                  "copy_or_transform": False}.items():
                assert_equal(item.get(key), expected, f"manifest/{route}/{name}/{key}")
        base = record.get("base_input")
        if not isinstance(base, dict):
            raise fail(f"manifest/{route}/base_input missing")
        for key, expected in {
            "placeholder": "__PHASE127_RAW_BASE_RINEX__",
            "sha256_placeholder": "__PHASE127_RAW_BASE_SHA256__",
            "raw_rinex_only": True,
            "header_inventory_after_authorization": True,
            "payload_read_before_authorization": False,
            "hash_read_before_authorization": False,
            "copy_or_transform": False,
            "signal_inventory_complete": True,
            "glonass_channel_frequency_if_present": True,
            "state_atmosphere_finite_in_domain": True,
        }.items():
            assert_equal(base.get(key), expected, f"manifest/{route}/base/{key}")
        inventory = record.get("inventory_gate")
        if not isinstance(inventory, dict):
            raise fail(f"manifest/{route}/inventory_gate missing")
        for key, expected in {"before_solver": True, "fail_closed": True,
                              "rover_full_glonass_coverage": True,
                              "base_full_glonass_coverage": True,
                              "solver_invocations_on_failure": 0}.items():
            assert_equal(inventory.get(key), expected, f"manifest/{route}/inventory_gate/{key}")
    _zero_accounting(manifest.get("read_accounting_before_authorization"),
                     "manifest/read_accounting_before_authorization")
    return manifest


def verify_inventory_record(route: str, inventory: dict[str, Any]) -> dict[str, Any]:
    """Validate one in-memory Stage 1 record; no payload path is accepted."""
    if not isinstance(inventory, dict):
        raise fail("inventory record is not an object")
    assert_equal(inventory.get("route"), route, "inventory/route")
    assert_equal(inventory.get("stage"), "post-authorization-pre-solver", "inventory/stage")
    assert_equal(inventory.get("solver_invocations"), 0, "inventory/solver_invocations")
    if inventory.get("solver_may_start") is not True:
        raise fail("inventory/solver_may_start must be true only for a complete admitted record")
    header = inventory.get("header")
    if not isinstance(header, dict):
        raise fail("inventory/header missing")
    for key in ("primary_or_nav_fallback_accounted", "selected_geph_fcn_matches",
                "exact_satellite_keys", "exact_query_times"):
        assert_equal(header.get(key), True, f"inventory/header/{key}")
    for key in ("malformed_entries", "conflict_entries", "invalid_entries", "unresolved_entries"):
        assert_equal(header.get(key), 0, f"inventory/header/{key}")
    geph = inventory.get("geph")
    if not isinstance(geph, dict):
        raise fail("inventory/geph missing")
    query_rows = geph.get("query_rows")
    if not isinstance(query_rows, int) or query_rows < 0:
        raise fail("inventory/geph/query_rows must be a nonnegative integer")
    for key in ("selected_rows", "valid_rows", "finite_fcns", "in_domain_fcns"):
        assert_equal(geph.get(key), query_rows, f"inventory/geph/{key}")
    if geph.get("query_times_exact") is not True:
        raise fail("inventory/geph/query_times_exact must be true")
    if (not isinstance(geph.get("max_age_s"), (int, float))
            or not math.isfinite(float(geph["max_age_s"]))
            or geph["max_age_s"] < 0.0 or geph["max_age_s"] > 1800.0):
        raise fail("inventory/geph/max_age_s exceeds the frozen validity window")
    if not isinstance(geph.get("duplicate_entries"), int) or geph["duplicate_entries"] < 0:
        raise fail("inventory/geph/duplicate_entries must be a nonnegative integer")
    for key in ("coverage_gaps", "different_fcn_ties", "conflicts", "missing_fcn", "out_of_range"):
        assert_equal(geph.get(key), 0, f"inventory/geph/{key}")
    coverage = inventory.get("coverage")
    if not isinstance(coverage, dict):
        raise fail("inventory/coverage missing")
    for side in ("rover", "base"):
        rows = coverage.get(f"{side}_glonass_rows")
        certified = coverage.get(f"{side}_certified_rows")
        if not isinstance(rows, int) or rows < 0 or not isinstance(certified, int):
            raise fail(f"inventory/coverage/{side} row counts must be nonnegative integers")
        assert_equal(certified, rows, f"inventory/coverage/{side}_certified_rows")
        assert_equal(coverage.get(f"{side}_unresolved_rows"), 0,
                     f"inventory/coverage/{side}_unresolved_rows")
        assert_equal(coverage.get(f"{side}_full"), True, f"inventory/coverage/{side}_full")
    assert_equal(coverage.get("all_finite_positive_frequency_wavelength"), True,
                 "inventory/coverage/all_finite_positive_frequency_wavelength")
    phase126 = inventory.get("phase126")
    if not isinstance(phase126, dict):
        raise fail("inventory/phase126 missing")
    assert_equal(phase126.get("source_complete"), True, "inventory/phase126/source_complete")
    assert_equal(phase126.get("base_correction_exactly_once"), True,
                 "inventory/phase126/base_correction_exactly_once")
    return {
        "route": route,
        "inventory_passed": True,
        "solver_invocations": 0,
        "solver_may_start": True,
        "rover_full_coverage": True,
        "base_full_coverage": True,
        "query_rows": query_rows,
    }


def verify_pre_raw_static() -> dict[str, Any]:
    """Return a zero-activity report without opening a route payload."""
    freeze = verify_freeze()
    manifest = verify_manifest()
    historical = sha256_file(ROOT / "apps/native/gnss_fgo_imu_no_base.cpp", "historical comparison app")
    if historical == HISTORICAL_PHASE126_APP_SHA256:
        raise fail("historical Phase126 app hash unexpectedly equals current Phase127 source")
    return {
        "status": "pre-raw-verified-no-payload-activity",
        "phase": 127,
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
        "historical_phase126_source_hash_check": "mismatch-retained",
        "historical_phase126_expected_app_sha256": HISTORICAL_PHASE126_APP_SHA256,
        "current_phase127_app_sha256": historical,
        "freeze_commit": FREEZE_COMMIT,
        "freeze_sha256": FREEZE_SHA256,
        "implementation_commit": IMPLEMENTATION_COMMIT,
        "manifest_sha256": sha256_file(MANIFEST, "Phase127 manifest"),
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
    except (Phase127ContractError, OSError) as exc:
        print(f"phase127 pre-raw validator: fail-closed: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
