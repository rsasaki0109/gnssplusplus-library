#!/usr/bin/env python3
"""Launch-free Phase126 structural contract validator.

This module reads only tracked source and sealed metadata.  It deliberately
does not materialize, probe, hash, or parse a raw phone/base member and does
not start the native application.  A future authorization may bind the
placeholders in the manifest, but this validator cannot authorize that run.
"""

from __future__ import annotations

import hashlib
import json
from pathlib import Path
import sys
from typing import Any


ROOT = Path(__file__).resolve().parents[3]
DESIGN_FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_phase126_raw_base_compound_port_design_freeze_v1.json"
AUDIT = ROOT / "docs/use_cases/records/smartphone_r5_phase126_raw_base_compound_structural_contract_audit_v1.md"
FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_phase126_raw_base_compound_structural_contract_freeze_v1.json"
MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase126_raw_base_compound_structural_manifest_v1.json"
RUNNER = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase126_raw_base_compound_structural_execute.py"
FOCUSED_TESTS = ROOT / "tests/test_smartphone_phase126_raw_base_compound_structural.py"
BINARY = ROOT / "build/apps/gnss_fgo_imu_no_base"

DESIGN_FREEZE_COMMIT = "583a6c7a4788f82373a4e71436d2953703bc764c"
DESIGN_FREEZE_SHA256 = "96dd275d08057e7848ee988c0cf613c2cfec80b5a1ed65b062f33ccfaf044e88"
IMPLEMENTATION_COMMIT = "9e9972667ee009e1bcd0dd1ea4732ae45415c3ce"
AUDIT_COMMIT = "ea083e3eb8ecf01a1d99ddd6f6ef852483da13ca"
AUDIT_SHA256 = "8ad6fcedc7a3b23c8f13b82968f67bada2fbe039d90f919c104e9a910f71f4ee"
FREEZE_COMMIT = "5d00daf56afb6fe6b6e1f01b4a1213c287f671db"
FREEZE_SHA256 = "196a9ffd1ecc909f8a34bbf8b9107be216a32bbd128fa54c94ed8a588ec7cc7b"

PHASE118_MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase118_tdcp_robust_k_structural_manifest_v1.json"
PHASE118_RESULT = ROOT / "docs/use_cases/records/smartphone_r5_phase118_tdcp_robust_k_structural_result_v1.json"
PHASE112_MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase112_main_output_offset_manifest_v1.json"
PHASE112_RESULT = ROOT / "docs/use_cases/records/smartphone_r5_phase112_main_output_offset_structural_result_v1.json"
PHASE117_RESULT = ROOT / "docs/use_cases/records/smartphone_r5_phase117_tdcp_weighting_structural_result_v1.json"
PHASE118_MANIFEST_SHA256 = "e0a4a0e879a56d7336fe1e2cbf041e5a72920d8103dd22dfdc9dffddfe62ec1d"
PHASE118_RESULT_SHA256 = "88e8799050fd396eeb14e83d4f5923339279f46d0219a385e303d3cb50731538"
PHASE112_MANIFEST_SHA256 = "d8be91f42f07196e07b293558cc6b83dd261226af577c1b26b167eead75902e4"
PHASE112_RESULT_SHA256 = "087b1bf51b4e584b86a7b558560df4ceb78e19b01e572f2d60aeef4c3d759ba0"
PHASE117_RESULT_SHA256 = "2517dcc805146dc34e790c78a392caf000c79eaba837fe099b6d522c147e9cf2"

SELECTOR = "--native-phase126-raw-base-source-complete"
PHASE118_SELECTOR = "--native-phase118-official-tdcp-huber-k"
PHASE117_SELECTOR = "--native-phase117-tdcp-snr-type-sigma"
PHASE120_SELECTOR = "--native-phase120-official-tdcp-resl-atmosphere-cancellation"
ADDITIONAL_BAND_SELECTOR = "--native-base-pseudorange-preserve-additional-frequency-bands"
RAW_NAMES = ("device_gnss.csv", "device_imu.csv", "brdc.nav")
ROUTES = (
    "2021-03-16-18-59-us-ca-mtv-a/pixel5",
    "2022-04-01-18-22-us-ca-lax-t/pixel5",
)
DOMAIN_ROWS = {ROUTES[0]: 2158, ROUTES[1]: 1465}
PROBLEM_EPOCHS = {route: rows + 1 for route, rows in DOMAIN_ROWS.items()}
SCHEMA = "smartphone-r5-phase126-raw-base-compound-structural-manifest.v1"
CANDIDATE_ID = "phase126-raw-base-source-complete-compound-v1"

IMPLEMENTATION_SOURCES = {
    "apps/native/gnss_fgo_imu_no_base.cpp": ROOT / "apps/native/gnss_fgo_imu_no_base.cpp",
    "include/libgnss++/algorithms/fgo_config.hpp": ROOT / "include/libgnss++/algorithms/fgo_config.hpp",
    "include/libgnss++/algorithms/fgo.hpp": ROOT / "include/libgnss++/algorithms/fgo.hpp",
    "include/libgnss++/algorithms/base_pseudorange_compensation.hpp": ROOT / "include/libgnss++/algorithms/base_pseudorange_compensation.hpp",
    "include/libgnss++/algorithms/phase126_raw_base_compound.hpp": ROOT / "include/libgnss++/algorithms/phase126_raw_base_compound.hpp",
    "src/algorithms/base_pseudorange_compensation.cpp": ROOT / "src/algorithms/base_pseudorange_compensation.cpp",
    "src/algorithms/phase126_raw_base_compound.cpp": ROOT / "src/algorithms/phase126_raw_base_compound.cpp",
    "src/algorithms/source_pseudorange_miss_mask.cpp": ROOT / "src/algorithms/source_pseudorange_miss_mask.cpp",
    "src/algorithms/fgo_problems.cpp": ROOT / "src/algorithms/fgo_problems.cpp",
}


class Phase126ContractError(ValueError):
    """A structural contract violation; callers must fail closed."""


def fail(message: str) -> Phase126ContractError:
    return Phase126ContractError(message)


def relative(path: Path) -> str:
    return str(path.relative_to(ROOT))


def assert_equal(actual: Any, expected: Any, label: str) -> None:
    if actual != expected:
        raise fail(f"{label}: expected {expected!r}, got {actual!r}")


def sha256_file(path: Path, label: str) -> str:
    # A future caller cannot accidentally turn this launch-free validator into
    # a payload probe.  Only known static artifacts may reach this function.
    lowered = path.name.lower()
    if (lowered in RAW_NAMES or lowered == "base.obs" or lowered.endswith(".csv")
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


def verify_design_freeze() -> dict[str, Any]:
    assert_equal(sha256_file(DESIGN_FREEZE, "Phase126 design freeze"), DESIGN_FREEZE_SHA256,
                 "design_freeze/sha256")
    design = read_json(DESIGN_FREEZE, "Phase126 design freeze")
    assert_equal(design.get("schema_version"), "smartphone-r5-phase126-raw-base-compound-port-design-freeze.v1",
                 "design_freeze/schema_version")
    assert_equal(design.get("phase"), 126, "design_freeze/phase")
    assert_equal(design.get("status"), "sealed-design-only-default-off", "design_freeze/status")
    decision = design.get("decision")
    if not isinstance(decision, dict):
        raise fail("design_freeze/decision missing")
    for key, expected in {
        "candidate_count": 1,
        "candidate_id": CANDIDATE_ID,
        "selector": SELECTOR,
        "default_off": True,
        "partial_selectors_allowed": False,
        "implementation_authorized": False,
        "raw_execution_authorized": False,
        "truth_evaluation_authorized": False,
    }.items():
        assert_equal(decision.get(key), expected, f"design_freeze/decision/{key}")
    return design


def verify_freeze() -> dict[str, Any]:
    verify_design_freeze()
    assert_equal(sha256_file(AUDIT, "Phase126 structural audit"), AUDIT_SHA256,
                 "audit/sha256")
    assert_equal(sha256_file(FREEZE, "Phase126 structural freeze"), FREEZE_SHA256,
                 "freeze/sha256")
    freeze = read_json(FREEZE, "Phase126 structural freeze")
    for key, expected in {
        "schema_version": "smartphone-r5-phase126-raw-base-compound-structural-contract-freeze.v1",
        "phase": 126,
        "execution_label": "Luna Max",
        "status": "sealed-before-phase126-raw-execution",
    }.items():
        assert_equal(freeze.get(key), expected, f"freeze/{key}")
    audit = freeze.get("contract_audit")
    if not isinstance(audit, dict):
        raise fail("freeze/contract_audit missing")
    for key, expected in {"path": relative(AUDIT), "commit": AUDIT_COMMIT, "sha256": AUDIT_SHA256}.items():
        assert_equal(audit.get(key), expected, f"freeze/contract_audit/{key}")
    design = freeze.get("design_freeze")
    if not isinstance(design, dict):
        raise fail("freeze/design_freeze missing")
    for key, expected in {"path": relative(DESIGN_FREEZE), "commit": DESIGN_FREEZE_COMMIT,
                          "sha256": DESIGN_FREEZE_SHA256}.items():
        assert_equal(design.get(key), expected, f"freeze/design_freeze/{key}")
    implementation = freeze.get("implementation")
    if not isinstance(implementation, dict):
        raise fail("freeze/implementation missing")
    assert_equal(implementation.get("commit"), IMPLEMENTATION_COMMIT, "freeze/implementation/commit")
    assert_equal(implementation.get("selector"), SELECTOR, "freeze/implementation/selector")
    assert_equal(implementation.get("default_off"), True, "freeze/implementation/default_off")
    assert_equal(implementation.get("partial_selectors_allowed"), False,
                 "freeze/implementation/partial_selectors_allowed")
    for key in ("raw_materialization_authorized", "raw_execution_authorized", "solver_execution_authorized",
                "truth_evaluation_authorized", "accuracy_authorized", "solution_publication_authorized",
                "kaggle_submission_authorized", "rerun_or_fallback_authorized"):
        assert_equal(freeze.get("authorization", {}).get(key), False, f"freeze/authorization/{key}")
    gates = freeze.get("structural_gates")
    if not isinstance(gates, dict):
        raise fail("freeze/structural_gates missing")
    for key, value in gates.items():
        assert_equal(value, True, f"freeze/structural_gates/{key}")
    return freeze


def verify_implementation(freeze: dict[str, Any]) -> dict[str, str]:
    implementation = freeze["implementation"]
    source_pins = implementation.get("source_sha256")
    if not isinstance(source_pins, dict):
        raise fail("freeze/implementation/source_sha256 missing")
    actual: dict[str, str] = {}
    for name, path in IMPLEMENTATION_SOURCES.items():
        expected = source_pins.get(name)
        if not isinstance(expected, str):
            raise fail(f"missing source pin: {name}")
        actual[name] = sha256_file(path, f"implementation/{name}")
        assert_equal(actual[name], expected, f"implementation/{name}/sha256")
    binary = implementation.get("binary")
    if not isinstance(binary, dict):
        raise fail("freeze/implementation/binary missing")
    assert_equal(binary.get("path"), relative(BINARY), "freeze/implementation/binary/path")
    assert_equal(sha256_file(BINARY, "implementation/binary"), binary.get("sha256"),
                 "freeze/implementation/binary/sha256")
    markers = {
        ROOT / "apps/native/gnss_fgo_imu_no_base.cpp": (
            SELECTOR, "phase126_atomic_step_a_raw_ingress_verified",
            "phase126_atomic_step_b_source_stream_verified",
            "phase126_atomic_step_c_application_committed",
            "phase126_compound_admitted",
        ),
        ROOT / "src/algorithms/base_pseudorange_compensation.cpp": (
            "officialBaseCodeResidual", "centeredMovingMean", "source-complete",
        ),
        ROOT / "src/algorithms/source_pseudorange_miss_mask.cpp": (
            "Phase126 compound transaction", "factor_count_consistent",
            "factors.swap(retained)",
        ),
    }
    for path, required in markers.items():
        source = path.read_text(encoding="utf-8")
        for marker in required:
            if marker not in source:
                raise fail(f"implementation marker missing: {relative(path)}:{marker}")
    return actual


def verify_sealed_recipe() -> None:
    for path, expected, label in (
        (PHASE118_MANIFEST, PHASE118_MANIFEST_SHA256, "Phase118 manifest"),
        (PHASE118_RESULT, PHASE118_RESULT_SHA256, "Phase118 result"),
        (PHASE112_MANIFEST, PHASE112_MANIFEST_SHA256, "Phase112 manifest"),
        (PHASE112_RESULT, PHASE112_RESULT_SHA256, "Phase112 result"),
        (PHASE117_RESULT, PHASE117_RESULT_SHA256, "Phase117 result"),
    ):
        assert_equal(sha256_file(path, label), expected, f"{label}/sha256")
    phase118 = read_json(PHASE118_MANIFEST, "Phase118 sealed manifest")
    routes = phase118.get("routes")
    if not isinstance(routes, list) or [item.get("dataset_id") for item in routes] != list(ROUTES):
        raise fail("sealed Phase118 route order changed")
    for item in routes:
        route = item.get("dataset_id")
        assert_equal(item.get("official_setting_type"), "Highway", f"Phase118/{route}/Type")
        assert_equal(item.get("expected_tdcp_huber_k"), 0.5, f"Phase118/{route}/k")
        for name in RAW_NAMES:
            metadata = item.get("raw_inputs", {}).get(name)
            if not isinstance(metadata, dict) or not isinstance(metadata.get("placeholder"), str):
                raise fail(f"sealed raw metadata missing: {route}/{name}")
            assert_equal(metadata.get("payload_read_before_authorization"), False,
                         f"Phase118/{route}/{name}/read")
        base = item.get("base_input")
        if not isinstance(base, dict):
            raise fail(f"sealed base metadata missing: {route}")
        assert_equal(base.get("payload_read_before_authorization"), False, f"Phase118/{route}/base/read")


def command_template(route: str) -> list[str]:
    output_root = "output/smartphone-r5/phase126-raw-base-compound-v1/"
    output_route = route.replace("/", "__")
    return [
        "build/apps/gnss_fgo_imu_no_base", "--dataset-id", route,
        "--android-gnss", "__PHASE126_RAW_DEVICE_GNSS__",
        "--android-imu", "__PHASE126_RAW_DEVICE_IMU__",
        "--nav", "__PHASE126_RAW_BROADCAST_NAV__",
        "--all-epochs", "--android-raw-utc-keys", "--android-raw-clock-only",
        "--android-utc-wall-clock-fallback", "--native-pdc-imu-tdcp-no-bridge",
        "--native-source-direct-observable-quality", "--native-source-clock-c0d-factor",
        "--native-source-clock-c0d-meter-state-parity", "--native-source-clock-c0d-active-solve-diagnostic",
        "--native-source-clock-c0d-gnss-first-meter-state-handoff",
        "--native-source-clock-c0d-epoch-vector-parity",
        "--native-source-clock-c0d-phase99-main-multifrontal-qr-solver",
        "--native-base-pseudorange-compensation", "--native-base-pseudorange-source-miss-mask",
        "--native-upstream-position-offset", PHASE118_SELECTOR, SELECTOR,
        "--native-base-rinex", "__PHASE126_RAW_BASE_RINEX__",
        "--native-base-rinex-sha256", "__PHASE126_RAW_BASE_SHA256__",
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
    "--native-upstream-position-offset", PHASE118_SELECTOR, SELECTOR,
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
        "--android-gnss": "__PHASE126_RAW_DEVICE_GNSS__",
        "--android-imu": "__PHASE126_RAW_DEVICE_IMU__",
        "--nav": "__PHASE126_RAW_BROADCAST_NAV__",
        "--native-base-rinex": "__PHASE126_RAW_BASE_RINEX__",
        "--native-base-rinex-sha256": "__PHASE126_RAW_BASE_SHA256__",
    }.items():
        assert_equal(command[command.index(flag) + 1], placeholder, f"manifest/{route}/{flag}/placeholder")


def verify_manifest() -> dict[str, Any]:
    freeze = verify_freeze()
    implementation = verify_implementation(freeze)
    verify_sealed_recipe()
    manifest = read_json(MANIFEST, "Phase126 structural manifest")
    for key, expected in {"schema_version": SCHEMA, "phase": 126, "execution_label": "Luna Max",
                          "status": "sealed-before-phase126-raw-execution"}.items():
        assert_equal(manifest.get(key), expected, f"manifest/{key}")
    for section, expected in (
        ("freeze", {"path": relative(FREEZE), "commit": FREEZE_COMMIT, "sha256": FREEZE_SHA256}),
        ("contract_audit", {"path": relative(AUDIT), "commit": AUDIT_COMMIT, "sha256": AUDIT_SHA256}),
        ("design_freeze", {"path": relative(DESIGN_FREEZE), "commit": DESIGN_FREEZE_COMMIT,
                            "sha256": DESIGN_FREEZE_SHA256}),
    ):
        value = manifest.get(section)
        if not isinstance(value, dict):
            raise fail(f"manifest/{section} missing")
        for key, item in expected.items():
            assert_equal(value.get(key), item, f"manifest/{section}/{key}")
    impl = manifest.get("implementation")
    if not isinstance(impl, dict):
        raise fail("manifest/implementation missing")
    for key, expected in {"commit": IMPLEMENTATION_COMMIT, "candidate_id": CANDIDATE_ID,
                          "legacy_default_unchanged": True, "partial_selectors_allowed": False,
                          "main_solver": "MULTIFRONTAL_QR", "fixed_tdcp_sigma_m": 0.03,
                          "phase118_huber_k": 0.5, "phase117_dynamic_sigma": False,
                          "phase120_selector": False, "additional_frequency_selector": False,
                          "no_fallback": True}.items():
        assert_equal(impl.get(key), expected, f"manifest/implementation/{key}")
    assert_equal(impl.get("source_sha256"), implementation, "manifest/implementation/source_sha256")
    binary = impl.get("binary")
    if not isinstance(binary, dict):
        raise fail("manifest/implementation/binary missing")
    assert_equal(binary.get("path"), relative(BINARY), "manifest/implementation/binary/path")
    assert_equal(binary.get("sha256"), sha256_file(BINARY, "manifest/binary"), "manifest/binary/sha256")
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
    for key, expected in {"input_names_exact": list(RAW_NAMES), "phone_gnss_only": True,
                          "phone_imu_only": True, "broadcast_navigation_only": True,
                          "sealed_raw_base_rinex_only": True, "materialize_after_authorization": True,
                          "raw_content_copied_or_transformed": False,
                          "truth_mat_pdc_precomputed_coordinate_phone_coordinate_accuracy_kaggle": False}.items():
        assert_equal(raw_contract.get(key), expected, f"manifest/raw_input_contract/{key}")
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
        for key, expected in {"target": "MTV-A" if route == ROUTES[0] else "LAX-T",
                              "official_route_type": "Highway", "expected_tdcp_huber_k": 0.5,
                              "domain_rows": DOMAIN_ROWS[route], "expected_problem_epochs": PROBLEM_EPOCHS[route],
                              "expected_output_epochs": PROBLEM_EPOCHS[route], "runs": 1,
                              "phase126_selector_count": 1, "phase118_selector_count": 1,
                              "phase117_selector_count": 0, "phase120_selector_count": 0,
                              "additional_frequency_selector_count": 0}.items():
            assert_equal(record.get(key), expected, f"manifest/{route}/{key}")
        validate_command(route, record.get("command"))
        raw = record.get("raw_inputs")
        if not isinstance(raw, dict) or set(raw) != set(RAW_NAMES):
            raise fail(f"manifest/{route}/raw_inputs must be GNSS/IMU/nav only")
        for name in RAW_NAMES:
            item = raw[name]
            if not isinstance(item, dict):
                raise fail(f"manifest/{route}/{name} missing")
            for key, expected in {"placeholder": f"__PHASE126_RAW_{'DEVICE_GNSS' if name == 'device_gnss.csv' else 'DEVICE_IMU' if name == 'device_imu.csv' else 'BROADCAST_NAV'}__",
                                  "payload_read_before_authorization": False,
                                  "copy_or_transform": False}.items():
                assert_equal(item.get(key), expected, f"manifest/{route}/{name}/{key}")
        base = record.get("base_input")
        if not isinstance(base, dict):
            raise fail(f"manifest/{route}/base_input missing")
        for key, expected in {"placeholder": "__PHASE126_RAW_BASE_RINEX__",
                              "sha256_placeholder": "__PHASE126_RAW_BASE_SHA256__",
                              "raw_rinex_only": True, "header_inventory_after_authorization": True,
                              "payload_read_before_authorization": False, "hash_read_before_authorization": False,
                              "copy_or_transform": False, "signal_inventory_complete": True,
                              "glonass_channel_frequency_if_present": True,
                              "state_atmosphere_finite_in_domain": True}.items():
            assert_equal(base.get(key), expected, f"manifest/{route}/base/{key}")
    accounting = manifest.get("read_accounting_before_authorization")
    if not isinstance(accounting, dict):
        raise fail("manifest/read_accounting_before_authorization missing")
    for key in ("raw_phone_gnss_reads", "raw_phone_imu_reads", "broadcast_navigation_reads", "raw_base_rinex_reads",
                "raw_base_header_reads", "raw_base_hash_reads", "solution_rows_opened", "solution_coordinate_interpretations",
                "native_solver_invocations", "truth_reads", "mat_reads_or_generated", "pdc_reads",
                "precomputed_coordinate_reads", "accuracy_calculations", "kaggle_or_token_access", "route_reruns", "fallbacks"):
        assert_equal(accounting.get(key), 0, f"manifest/read_accounting/{key}")
    assert_equal(accounting.get("raw_content_copied_or_transformed"), False, "manifest/read_accounting/raw_copy")
    return manifest


def verify_pre_raw_static() -> dict[str, Any]:
    """Return zero activity without opening a payload path."""
    freeze = verify_freeze()
    manifest = verify_manifest()
    return {
        "status": "pre-raw-verified-no-payload-activity",
        "phase": 126,
        "execution_label": "Luna Max",
        "candidate_count": 1,
        "route_ids": list(ROUTES),
        "runs_per_route": 1,
        "raw_materialization_authorized": False,
        "raw_execution_authorized": False,
        "raw_phone_gnss_reads": 0,
        "raw_phone_imu_reads": 0,
        "broadcast_navigation_reads": 0,
        "raw_base_rinex_reads": 0,
        "raw_base_header_reads": 0,
        "raw_base_hash_reads": 0,
        "solution_rows_opened": 0,
        "solution_coordinate_interpretations": 0,
        "native_solver_invocations": 0,
        "truth_reads": 0,
        "mat_reads_or_generated": 0,
        "pdc_reads": 0,
        "precomputed_coordinate_reads": 0,
        "accuracy_calculations": 0,
        "kaggle_or_token_access": 0,
        "route_reruns": 0,
        "fallbacks": 0,
        "solution_output_published": False,
        "freeze_commit": FREEZE_COMMIT,
        "freeze_sha256": FREEZE_SHA256,
        "implementation_commit": IMPLEMENTATION_COMMIT,
        "manifest_sha256": sha256_file(MANIFEST, "Phase126 manifest"),
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
    except (Phase126ContractError, OSError) as exc:
        print(f"phase126 pre-raw validator: fail-closed: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
