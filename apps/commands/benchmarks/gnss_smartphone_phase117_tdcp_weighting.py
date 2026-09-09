#!/usr/bin/env python3
"""Launch-free Phase117 raw/base structural contract.

This verifier pins the single Phase117 candidate and the exact two-route raw
recipe.  Before the independent authorization it reads only source and
sealed JSON metadata; it never stats, hashes, or opens phone raw members,
base RINEX, truth, MAT, candidate coordinates, or solution rows.
"""

from __future__ import annotations

import hashlib
import json
from pathlib import Path
import sys
from typing import Any


ROOT = Path(__file__).resolve().parents[3]
FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_phase117_tdcp_weighting_source_parity_freeze_v1.json"
MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase117_tdcp_weighting_manifest_v1.json"
AUTHORIZATION = ROOT / "docs/use_cases/records/smartphone_r5_phase117_tdcp_weighting_raw_authorization_v1.json"
PHASE112_MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase112_main_output_offset_manifest_v1.json"
PHASE116_RESULT = ROOT / "docs/use_cases/records/smartphone_r5_phase116_carrier_tdcp_incidence_structural_result_v1.json"
EVALUATOR = Path(__file__).resolve()
WRAPPER = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase117_tdcp_weighting_execute.py"
FOCUSED_TESTS = ROOT / "tests/test_smartphone_phase117_tdcp_weighting_execution.py"
APP = ROOT / "apps/native/gnss_fgo_imu_no_base.cpp"
FGO_HEADER = ROOT / "include/libgnss++/algorithms/fgo.hpp"
FGO_CONFIG = ROOT / "include/libgnss++/algorithms/fgo_config.hpp"
UPSTREAM_HEADER = ROOT / "include/libgnss++/algorithms/observable_upstream_preprocessing.hpp"
FGO_INTERNAL = ROOT / "src/algorithms/fgo_internal.hpp"
FGO_PROBLEMS = ROOT / "src/algorithms/fgo_problems.cpp"
BINARY = ROOT / "build/apps/gnss_fgo_imu_no_base"

FREEZE_SHA256 = "aba967b2ffab7ded84daa232ab948848d49497a70561262d1077200584b92dea"
FREEZE_COMMIT = "283e129b16ea21f077e041b55c87772d03e74f58"
IMPLEMENTATION_COMMIT = "13c1fe2c7fe9c1970db50018b5bc07cef76901bf"
PHASE112_MANIFEST_SHA256 = "d8be91f42f07196e07b293558cc6b83dd261226af577c1b26b167eead75902e4"
PHASE116_RESULT_SHA256 = "394ce1660ae5f181967b672bffab14d9228ffbe62ec6b748ca3854d30a14876f"
SOURCE_SHA256 = {
    "app": "ae89f38d5672474bcd63a7134f852f1fb63c44c55e70990116875001508fe667",
    "fgo_header": "f17e3e5aaac8ad4bd368d662876637a478977066cf7d0ada734763eca0ef91d4",
    "fgo_config": "b0bba8f8f1c0f5aac1e1dca19705e7291b3134fe1d3155436ff05995da177342",
    "upstream_header": "4e20a30730aeca9c93d7108e4edeb7dbe18d05d2f5842d3a4326e29c61f4b60f",
    "fgo_internal": "8513b74326974042ed97deb50104ef578ce7d0b6b7aab3674432e25fa0a2d131",
    "fgo_problems": "327fdc792afbeb29f150bc8566acaef63156c519d2dcb0a40f626659658edddc",
    "binary": "12c3c12152b443f40f0371cdf51b7d32b02f6bd953ca2bb648d69261a0ed14ee",
}

ROUTES = (
    "2021-03-16-18-59-us-ca-mtv-a/pixel5",
    "2022-04-01-18-22-us-ca-lax-t/pixel5",
)
DOMAIN_ROWS = {ROUTES[0]: 2158, ROUTES[1]: 1465}
PROBLEM_EPOCHS = {route: rows + 1 for route, rows in DOMAIN_ROWS.items()}
RAW_NAMES = ("device_gnss.csv", "device_imu.csv", "brdc.nav")
PHASE93_SELECTOR = "--native-source-clock-c0d-gnss-first-meter-state-handoff"
VECTOR_SELECTOR = "--native-source-clock-c0d-epoch-vector-parity"
QR_SELECTOR = "--native-source-clock-c0d-phase99-main-multifrontal-qr-solver"
PHASE116_SELECTOR = "--native-phase116-carrier-tdcp-incidence-diagnostic"
PHASE117_SELECTOR = "--native-phase117-tdcp-snr-type-sigma"
BASE_SELECTORS = (
    "--native-base-pseudorange-compensation",
    "--native-base-pseudorange-source-miss-mask",
    "--native-base-pseudorange-preserve-additional-frequency-bands",
)
OFFSET_SELECTOR = "--native-upstream-position-offset"
BASE_FLAG = ("--native-base-rinex", "__PHASE117_RAW_BASE_RINEX__")
BASE_SHA_FLAG = ("--native-base-rinex-sha256", "__PHASE117_RAW_BASE_SHA256__")
CANDIDATE_ID = "phase117-official-tdcp-snr-type-weighting-v1"
OUTPUT_ROOT = "output/smartphone-r5/phase117-tdcp-weighting-v1/"
SCHEMA = "smartphone-r5-phase117-tdcp-weighting-manifest.v1"
AUTH_SCHEMA = "smartphone-r5-phase117-tdcp-weighting-raw-authorization.v1"

REQUIRED_FLAGS = (
    "--all-epochs", "--android-raw-utc-keys", "--android-raw-clock-only",
    "--android-utc-wall-clock-fallback", "--native-pdc-imu-tdcp-no-bridge",
    "--native-source-direct-observable-quality", "--native-source-clock-c0d-factor",
    "--native-source-clock-c0d-meter-state-parity",
    "--native-source-clock-c0d-active-solve-diagnostic", PHASE93_SELECTOR,
    VECTOR_SELECTOR, QR_SELECTOR, *BASE_SELECTORS, OFFSET_SELECTOR,
    PHASE116_SELECTOR, PHASE117_SELECTOR,
)
FORBIDDEN_FLAGS = (
    "--obs", "--native-gnss-first-velocity-only-handoff",
    "--native-direct-doppler-wls-handoff", "--native-pdc-state-bridge",
    "--native-upstream-quality",
    "--native-source-clock-c0d-gnss-first-raw-drift-d-initializer",
    "--native-source-clock-c0d-phase94-stage-diagnostics",
    "--native-source-clock-c0d-phase96-main-diagnostics",
    "--native-source-clock-c0d-phase97-singular-system-diagnostics",
    "--native-source-clock-c0d-phase98-solver-rank-diagnostic",
    "--native-direct-wls-ephemeral-c7d-main-seed",
    "--native-phase104-stage-main-attribution", "--native-quality-anchor",
)
FORBIDDEN_PATH_TERMS = (
    ".mat", "truth", "ground_truth", "validation", "holdout", "precomputed",
    "coordinate", "pdc", "kaggle", "token",
)


class Phase117ContractError(ValueError):
    """Raised when the pinned Phase117 contract fails closed."""


def fail(message: str) -> Phase117ContractError:
    return Phase117ContractError(message)


def relative(path: Path) -> str:
    try:
        return str(path.relative_to(ROOT))
    except ValueError:
        return str(path)


def assert_equal(actual: Any, expected: Any, label: str) -> None:
    if actual != expected:
        raise fail(f"{label}: expected {expected!r}, got {actual!r}")


def sha256_file(path: Path, label: str) -> str:
    # No pre-authorization input payload is hashable through this contract.
    if path.name in RAW_NAMES or path.name == "base.obs" or path.name.endswith("withheld_solution_output.csv"):
        raise fail(f"input/solution member hash forbidden before authorization: {label}")
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


def sealed_phase112_routes() -> dict[str, dict[str, Any]]:
    source = read_json(PHASE112_MANIFEST, "sealed Phase112 manifest")
    assert_equal(sha256_file(PHASE112_MANIFEST, "sealed Phase112 manifest"),
                 PHASE112_MANIFEST_SHA256, "Phase112 manifest/sha256")
    routes = source.get("routes")
    if not isinstance(routes, list) or [item.get("dataset_id") for item in routes] != list(ROUTES):
        raise fail("sealed Phase112 route order changed")
    result: dict[str, dict[str, Any]] = {}
    for item in routes:
        route = item.get("dataset_id")
        if route not in ROUTES:
            raise fail(f"unexpected sealed Phase112 route: {route}")
        for name in RAW_NAMES:
            pin = item.get("raw_inputs", {}).get(name)
            if not isinstance(pin, dict) or not isinstance(pin.get("path"), str) or not isinstance(pin.get("sha256"), str):
                raise fail(f"missing sealed raw metadata: {route}/{name}")
            if pin.get("bytes") is None or pin.get("read_at_manifest_creation") is not False:
                raise fail(f"sealed raw metadata boundary changed: {route}/{name}")
        base = item.get("base_input")
        if not isinstance(base, dict) or not isinstance(base.get("path"), str) or not isinstance(base.get("sha256"), str):
            raise fail(f"missing sealed base metadata: {route}")
        if base.get("bytes") is None or base.get("read_at_manifest_creation") is not False or base.get("hash_read_at_manifest_creation") is not False:
            raise fail(f"sealed base metadata boundary changed: {route}")
        result[route] = item
    return result


def verify_freeze() -> dict[str, Any]:
    assert_equal(sha256_file(FREEZE, "Phase117 freeze"), FREEZE_SHA256,
                 "freeze/sha256")
    freeze = read_json(FREEZE, "Phase117 freeze")
    for key, expected in {
        "schema_version": "smartphone-r5-phase117-tdcp-weighting-source-parity-freeze.v1",
        "phase": 117, "execution_label": "Luna Max",
        "status": "frozen-one-source-backed-tdcp-weighting-candidate-before-execution",
    }.items():
        assert_equal(freeze.get(key), expected, f"freeze/{key}")
    decision = freeze.get("decision")
    if not isinstance(decision, dict):
        raise fail("freeze/decision missing")
    for key, expected in {
        "candidate_count": 1, "candidate_id": CANDIDATE_ID,
        "source_backed": True, "selected_dimension": "ordinary TDCP scalar noise/weighting only",
        "implementation_in_this_freeze": False, "graph_factor_value_key_change_in_this_freeze": False,
        "legacy_default_changed": False, "raw_execution_authorized": False,
        "solver_execution_authorized": False, "truth_evaluation_authorized": False,
        "accuracy_authorized": False, "solution_publication_authorized": False,
    }.items():
        assert_equal(decision.get(key), expected, f"freeze/decision/{key}")
    candidate = freeze.get("candidate")
    if not isinstance(candidate, dict):
        raise fail("freeze/candidate missing")
    for key, expected in {
        "id": CANDIDATE_ID, "opt_in": True, "default_off": True,
        "raw_only": True, "factor_family": "existing ordinary same-satellite/same-signal adjacent TDCP only",
        "fail_closed": True, "solution_withheld": True, "accuracy_evaluation": False,
    }.items():
        assert_equal(candidate.get(key), expected, f"freeze/candidate/{key}")
    noise = candidate.get("official_noise_contract")
    if not isinstance(noise, dict):
        raise fail("freeze/official noise contract missing")
    for key, expected in {
        "model": "SNR", "snr_percentile": 85, "snr_denominator": 20,
        "L_sn_ratio": 0.0025,
        "factor_endpoint": "previous endpoint i, exactly as noise_sigmas(obserr.(f).L(i,j)) in the official graph",
        "signal_factor_order": ["L1", "G1", "E1", "B1", "L5", "E5", "B2a"],
        "signal_factors": [0.8, 1.5, 0.8, 0.8, 0.5, 0.5, 0.5],
    }.items():
        assert_equal(noise.get(key), expected, f"freeze/noise/{key}")
    boundary = candidate.get("exact_change_boundary")
    if not isinstance(boundary, dict):
        raise fail("freeze/exact change boundary missing")
    for key, expected in {
        "preserve_residual_equation": True, "preserve_factor_count_and_insertion": True,
        "preserve_exact_pair_keys": True, "preserve_pair_reject_predicate": True,
        "preserve_native_huber_threshold_sigma": 4.0, "preserve_phase99_qr": True,
        "preserve_c7_d_c0d_handoff": True, "preserve_imu_p_doppler_and_base_scope": True,
        "preserve_filter_lm_initialization_and_output": True,
        "missing_or_nonfinite_noise_behavior": "fail closed; no 0.03 fallback, zero fill, average, endpoint substitution, or synthesized wavelength",
    }.items():
        assert_equal(boundary.get(key), expected, f"freeze/boundary/{key}")
    return freeze


def verify_implementation() -> dict[str, str]:
    paths = {
        "app": APP, "fgo_header": FGO_HEADER, "fgo_config": FGO_CONFIG,
        "upstream_header": UPSTREAM_HEADER, "fgo_internal": FGO_INTERNAL,
        "fgo_problems": FGO_PROBLEMS, "binary": BINARY,
    }
    actual = {key: sha256_file(path, f"implementation/{key}") for key, path in paths.items()}
    assert_equal(actual, SOURCE_SHA256, "implementation/source_sha256")
    markers = {
        APP: (PHASE117_SELECTOR, "native_phase117_tdcp_snr_type_sigma"),
        FGO_CONFIG: ("use_official_tdcp_snr_type_sigma",),
        UPSTREAM_HEADER: ("officialTdcpSigmaMeters", "kOfficialSnrPercentile", "kOfficialCarrierPhaseSnrRatio"),
        FGO_INTERNAL: ("snr_dbhz",),
        FGO_PROBLEMS: ("tdcp_rejected_invalid_weight", "official_tdcp_snr_percentiles"),
    }
    for path, required in markers.items():
        source = path.read_text(encoding="utf-8")
        for marker in required:
            if marker not in source:
                raise fail(f"Phase117 implementation marker missing: {marker}")
    return actual


def output_path(route: str, name: str) -> str:
    return f"{OUTPUT_ROOT}{route.replace('/', '__')}/{name}"


def command_template(route: str) -> list[str]:
    return [
        "build/apps/gnss_fgo_imu_no_base", "--dataset-id", route,
        "--android-gnss", "__PHASE117_RAW_DEVICE_GNSS__",
        "--android-imu", "__PHASE117_RAW_DEVICE_IMU__",
        "--nav", "__PHASE117_RAW_BROADCAST_NAV__", "--all-epochs",
        "--android-raw-utc-keys", "--android-raw-clock-only",
        "--android-utc-wall-clock-fallback", "--native-pdc-imu-tdcp-no-bridge",
        "--native-source-direct-observable-quality", "--native-source-clock-c0d-factor",
        "--native-source-clock-c0d-meter-state-parity",
        "--native-source-clock-c0d-active-solve-diagnostic", PHASE93_SELECTOR,
        VECTOR_SELECTOR, QR_SELECTOR, *BASE_SELECTORS, OFFSET_SELECTOR,
        PHASE116_SELECTOR, PHASE117_SELECTOR, BASE_FLAG[0], BASE_FLAG[1],
        BASE_SHA_FLAG[0], BASE_SHA_FLAG[1], "--out",
        output_path(route, "withheld_solution_output.csv"), "--summary-json",
        output_path(route, "summary.json"),
    ]


def validate_command(route: str, command: Any, source: dict[str, Any]) -> None:
    assert_equal(command, command_template(route), f"manifest/{route}/exact command")
    for token in command:
        if token in FORBIDDEN_FLAGS or (not token.startswith("--") and any(term in token.lower() for term in FORBIDDEN_PATH_TERMS)):
            raise fail(f"forbidden command token: {route}/{token}")
    for flag in REQUIRED_FLAGS:
        assert_equal(command.count(flag), 1, f"command/{route}/{flag}")
    assert_equal(command.count(BASE_FLAG[0]), 1, f"command/{route}/base flag")
    assert_equal(command[command.index(BASE_FLAG[0]) + 1], BASE_FLAG[1], f"command/{route}/base placeholder")
    assert_equal(command.count(BASE_SHA_FLAG[0]), 1, f"command/{route}/base sha flag")
    assert_equal(command[command.index(BASE_SHA_FLAG[0]) + 1], BASE_SHA_FLAG[1], f"command/{route}/base sha placeholder")
    placeholders = {
        "--android-gnss": "__PHASE117_RAW_DEVICE_GNSS__",
        "--android-imu": "__PHASE117_RAW_DEVICE_IMU__",
        "--nav": "__PHASE117_RAW_BROADCAST_NAV__",
    }
    for flag, placeholder in placeholders.items():
        assert_equal(command.count(flag), 1, f"command/{route}/{flag}")
        assert_equal(command[command.index(flag) + 1], placeholder, f"command/{route}/{flag}/placeholder")
    for name in RAW_NAMES:
        pin = source.get("raw_inputs", {}).get(name)
        if not isinstance(pin, dict):
            raise fail(f"sealed raw pin missing: {route}/{name}")
        if pin.get("read_at_manifest_creation") is not False:
            raise fail(f"raw pin read boundary changed: {route}/{name}")
    base = source.get("base_input")
    if not isinstance(base, dict) or base.get("read_at_manifest_creation") is not False or base.get("hash_read_at_manifest_creation") is not False:
        raise fail(f"sealed base pin boundary changed: {route}")


def verify_manifest() -> dict[str, Any]:
    verify_freeze()
    implementation = verify_implementation()
    source_routes = sealed_phase112_routes()
    manifest = read_json(MANIFEST, "Phase117 pre-raw manifest")
    for key, expected in {
        "schema_version": SCHEMA, "phase": 117, "execution_label": "Luna Max",
        "status": "sealed-before-phase117-raw-execution",
    }.items():
        assert_equal(manifest.get(key), expected, f"manifest/{key}")
    freeze = manifest.get("freeze")
    if not isinstance(freeze, dict):
        raise fail("manifest/freeze missing")
    for key, expected in {
        "path": relative(FREEZE), "sha256": FREEZE_SHA256, "commit": FREEZE_COMMIT,
        "raw_execution_authorized_before_manifest": False,
    }.items():
        assert_equal(freeze.get(key), expected, f"manifest/freeze/{key}")
    impl = manifest.get("implementation")
    if not isinstance(impl, dict):
        raise fail("manifest/implementation missing")
    for key, expected in {
        "commit": IMPLEMENTATION_COMMIT, "candidate_id": CANDIDATE_ID,
        "legacy_default_unchanged": True, "graph_solver_factors_values_unchanged": True,
        "official_tdcp_sigma_only": True, "no_fallback": True,
    }.items():
        assert_equal(impl.get(key), expected, f"manifest/implementation/{key}")
    assert_equal(impl.get("source_sha256"), implementation, "manifest/implementation/source_sha256")
    for key, path in (("evaluator", EVALUATOR), ("wrapper", WRAPPER), ("focused_tests", FOCUSED_TESTS)):
        pin = manifest.get(key)
        if not isinstance(pin, dict):
            raise fail(f"manifest/{key} missing")
        assert_equal(pin.get("path"), relative(path), f"manifest/{key}/path")
        expected_hash = pin.get("sha256")
        if not isinstance(expected_hash, str) or len(expected_hash) != 64:
            raise fail(f"manifest/{key}/sha256 missing")
        assert_equal(sha256_file(path, f"manifest/{key}"), expected_hash, f"manifest/{key}/sha256")
    provenance = manifest.get("sealed_input_provenance")
    if not isinstance(provenance, dict):
        raise fail("manifest/sealed_input_provenance missing")
    assert_equal(provenance.get("phase112_manifest_path"), relative(PHASE112_MANIFEST), "manifest/provenance/path")
    assert_equal(provenance.get("phase112_manifest_sha256"), PHASE112_MANIFEST_SHA256, "manifest/provenance/sha256")
    assert_equal(provenance.get("raw_paths_materialized_exactly"), True, "manifest/provenance/raw paths")
    assert_equal(provenance.get("base_paths_materialized_exactly"), True, "manifest/provenance/base paths")
    candidate = manifest.get("candidate")
    if not isinstance(candidate, dict):
        raise fail("manifest/candidate missing")
    for key, expected in {
        "id": CANDIDATE_ID, "candidate_count": 1, "opt_in": True, "default_off": True,
        "selector": PHASE117_SELECTOR,
        "selectors": [PHASE93_SELECTOR, VECTOR_SELECTOR, QR_SELECTOR, OFFSET_SELECTOR, PHASE116_SELECTOR, PHASE117_SELECTOR],
        "base_selectors": list(BASE_SELECTORS), "official_snr_percentile": 85,
        "official_snr_denominator": 20, "official_l_sn_ratio": 0.0025,
        "official_signal_factors": [0.8, 1.5, 0.8, 0.8, 0.5, 0.5, 0.5],
        "sigma_units": "source L cycles * retained wavelength_m = native metres",
        "factor_endpoint": "previous",
        "fixed_legacy_tdcp_sigma_m": 0.03, "missing_metadata": "fail-closed-no-fallback",
        "factor_count_invariance_for_valid_metadata": True,
        "runs_per_route": 1, "controls": 0, "reruns": 0, "fallbacks": 0,
        "truth_evaluation": False, "accuracy_scoring": False,
        "solution_output_publication": False, "solution_withheld": True,
    }.items():
        assert_equal(candidate.get(key), expected, f"manifest/candidate/{key}")
    routes = manifest.get("routes")
    if not isinstance(routes, list) or [item.get("dataset_id") for item in routes] != list(ROUTES):
        raise fail("manifest route order/count changed")
    for record in routes:
        route = record.get("dataset_id")
        if route not in DOMAIN_ROWS:
            raise fail(f"unknown route: {route}")
        for key, expected in {"domain_rows": DOMAIN_ROWS[route], "expected_problem_epochs": PROBLEM_EPOCHS[route], "expected_output_epochs": PROBLEM_EPOCHS[route], "runs": 1}.items():
            assert_equal(record.get(key), expected, f"manifest/{route}/{key}")
        validate_command(route, record.get("command"), source_routes[route])
    raw_contract = manifest.get("raw_input_contract")
    if not isinstance(raw_contract, dict):
        raise fail("manifest/raw_input_contract missing")
    for key, expected in {
        "input_names_exact": list(RAW_NAMES), "phone_gnss_only": True,
        "phone_imu_only": True, "broadcast_navigation_only": True,
        "sealed_raw_base_rinex_only": True, "content_copy_or_transform": False,
        "truth_mat_precomputed_coordinate_pdc_kaggle_accuracy": False,
    }.items():
        assert_equal(raw_contract.get(key), expected, f"manifest/raw_input_contract/{key}")
    gates = manifest.get("structural_gates")
    if not isinstance(gates, dict):
        raise fail("manifest/structural_gates missing")
    for key in (
        "official_sigma_all_finite_source_derived", "valid_metadata_factor_count_unchanged",
        "missing_metadata_fail_closed_no_legacy_fallback", "tdcp_residual_keys_reject_predicate_unchanged",
        "gnss_first_progress_strict_cost_decrease", "gnss_first_full_finite_c7_d_exact_handoff",
        "main_qr_progress_strict_cost_decrease", "base_correction_exactly_once",
        "offset_exactly_once_final_boundary", "finite_expected_output_coverage",
        "no_solver_fallback_or_solution_publication",
    ):
        assert_equal(gates.get(key), True, f"manifest/structural_gates/{key}")
    matrix = manifest.get("matrix")
    if not isinstance(matrix, dict):
        raise fail("manifest/matrix missing")
    for key, expected in {
        "candidate_count": 1, "route_count": 2, "runs_per_route": 1,
        "native_solver_invocations_planned": 2, "controls": 0, "reruns": 0,
        "fallbacks": 0, "truth_reads_planned": 0, "mat_reads_or_generated_planned": 0,
        "phone_coordinate_reads_planned": 0, "precomputed_coordinate_reads_planned": 0,
        "pdc_reads_planned": 0, "accuracy_calculations_planned": 0,
        "solution_rows_authorized": False,
    }.items():
        assert_equal(matrix.get(key), expected, f"manifest/matrix/{key}")
    accounting = manifest.get("read_accounting_before_authorization")
    if not isinstance(accounting, dict):
        raise fail("manifest/read_accounting_before_authorization missing")
    for key in (
        "raw_phone_gnss_reads", "raw_phone_imu_reads", "broadcast_navigation_reads",
        "raw_base_rinex_reads", "raw_base_hash_reads", "native_solver_invocations",
        "truth_reads", "mat_reads_or_generated", "phone_coordinate_reads",
        "precomputed_coordinate_reads", "pdc_reads", "accuracy_calculations",
        "kaggle_or_token_access", "route_reruns", "fallbacks",
    ):
        assert_equal(accounting.get(key), 0, f"manifest/read_accounting/{key}")
    assert_equal(accounting.get("raw_content_copied_or_transformed"), False, "manifest/read_accounting/copy")
    return manifest


def verify_authorization(manifest: dict[str, Any] | None = None) -> dict[str, Any]:
    if manifest is None:
        manifest = verify_manifest()
    auth = read_json(AUTHORIZATION, "Phase117 raw authorization")
    for key, expected in {
        "schema_version": AUTH_SCHEMA, "phase": 117, "execution_label": "Luna Max",
        "status": "authorized-for-exact-two-route-phase117-raw-base-structural-execution",
    }.items():
        assert_equal(auth.get(key), expected, f"authorization/{key}")
    authority = auth.get("authority")
    if not isinstance(authority, dict):
        raise fail("authorization/authority missing")
    pins = {
        "phase117_freeze": (FREEZE, FREEZE_SHA256),
        "phase117_manifest": (MANIFEST, sha256_file(MANIFEST, "Phase117 manifest")),
        "phase117_evaluator": (EVALUATOR, sha256_file(EVALUATOR, "Phase117 evaluator")),
        "phase117_wrapper": (WRAPPER, sha256_file(WRAPPER, "Phase117 wrapper")),
        "phase117_focused_tests": (FOCUSED_TESTS, sha256_file(FOCUSED_TESTS, "Phase117 focused tests")),
        "phase112_manifest": (PHASE112_MANIFEST, PHASE112_MANIFEST_SHA256),
        "phase116_result": (PHASE116_RESULT, PHASE116_RESULT_SHA256),
    }
    for key, (path, expected) in pins.items():
        pin = authority.get(key)
        if not isinstance(pin, dict):
            raise fail(f"authorization/{key} missing")
        assert_equal(pin.get("path"), relative(path), f"authorization/{key}/path")
        assert_equal(pin.get("sha256"), expected, f"authorization/{key}/sha256")
    for key, expected in {
        "freeze_commit": FREEZE_COMMIT, "implementation_commit": IMPLEMENTATION_COMMIT,
        "manifest_sha256": sha256_file(MANIFEST, "Phase117 manifest"),
        "evaluator_sha256": sha256_file(EVALUATOR, "Phase117 evaluator"),
        "wrapper_sha256": sha256_file(WRAPPER, "Phase117 wrapper"),
        "binary_sha256": SOURCE_SHA256["binary"],
    }.items():
        assert_equal(authority.get(key), expected, f"authorization/authority/{key}")
    assert_equal(auth.get("routes"), list(ROUTES), "authorization/routes")
    candidate = auth.get("candidate")
    if not isinstance(candidate, dict):
        raise fail("authorization/candidate missing")
    for key, expected in {
        "id": CANDIDATE_ID, "candidate_count": 1, "selector": PHASE117_SELECTOR,
        "selectors": [PHASE93_SELECTOR, VECTOR_SELECTOR, QR_SELECTOR, OFFSET_SELECTOR, PHASE116_SELECTOR, PHASE117_SELECTOR],
        "base_selectors": list(BASE_SELECTORS), "runs_per_route": 1, "controls": 0,
        "reruns": 0, "fallbacks": 0, "truth_evaluation": False, "accuracy_scoring": False,
        "solution_output_publication": False, "solution_withheld": True,
        "sigma_only": True, "no_fallback": True, "no_graph_mutation": True,
    }.items():
        assert_equal(candidate.get(key), expected, f"authorization/candidate/{key}")
    matrix = auth.get("matrix")
    if not isinstance(matrix, dict):
        raise fail("authorization/matrix missing")
    for key, expected in {
        "candidate_count": 1, "route_count": 2, "runs_per_route": 1,
        "native_invocations": 2, "raw_phone_gnss_process_reads_max": 2,
        "raw_phone_imu_process_reads_max": 2, "broadcast_navigation_process_reads_max": 2,
        "raw_base_hash_reads_max": 2, "raw_base_native_process_reads_max": 2,
        "truth_reads": 0, "mat_reads_or_generated": 0, "phone_coordinate_reads": 0,
        "precomputed_coordinate_reads": 0, "pdc_reads": 0, "accuracy_calculations": 0,
        "kaggle_or_token_access": 0, "solution_rows_authorized": False,
        "reruns": 0, "fallbacks": 0,
    }.items():
        assert_equal(matrix.get(key), expected, f"authorization/matrix/{key}")
    policy = auth.get("execution_policy")
    if not isinstance(policy, dict):
        raise fail("authorization/execution_policy missing")
    for key, expected in {
        "authorized": True, "order": "MTV-A then LAX-T, sequentially",
        "one_invocation_per_route": True, "no_rerun": True, "no_solver_fallback": True,
        "solver_filter_lm_equation_unit_sigma_graph_unchanged": True,
        "official_tdcp_sigma_only": True, "truth_or_accuracy_evaluation": False,
        "solution_output": "isolated withheld path; hash/row seal only; no coordinate interpretation or publication",
        "base_input": "sealed raw base RINEX; one wrapper hash read plus one native process read per route",
    }.items():
        assert_equal(policy.get(key), expected, f"authorization/policy/{key}")
    boundary = auth.get("release_boundary")
    if not isinstance(boundary, dict):
        raise fail("authorization/release_boundary missing")
    for key, expected in {
        "raw_base_execution_authorized": True, "structural_result_authorized": True,
        "truth_or_accuracy_evaluation_authorized": False,
        "solution_publication_authorized": False, "kaggle_submission_authorized": False,
    }.items():
        assert_equal(boundary.get(key), expected, f"authorization/boundary/{key}")
    return auth


def verify_pre_raw() -> dict[str, Any]:
    freeze = verify_freeze()
    manifest = verify_manifest()
    return {
        "status": "pre-raw-verified", "phase": 117, "execution_label": "Luna Max",
        "candidate_count": 1, "route_ids": list(ROUTES), "runs_per_route": 1,
        "raw_execution_authorized": False, "raw_phone_gnss_reads": 0,
        "raw_phone_imu_reads": 0, "broadcast_navigation_reads": 0,
        "raw_base_rinex_reads": 0, "raw_base_hash_reads": 0,
        "native_solver_invocations": 0, "truth_reads": 0, "mat_reads_or_generated": 0,
        "phone_coordinate_reads": 0, "precomputed_coordinate_reads": 0,
        "pdc_reads": 0, "accuracy_calculations": 0, "solution_output_published": False,
        "freeze_sha256": FREEZE_SHA256,
        "manifest_sha256": sha256_file(MANIFEST, "Phase117 manifest"),
        "freeze_status": freeze["status"], "manifest_status": manifest["status"],
    }


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--verify-freeze", action="store_true")
    parser.add_argument("--verify-manifest", action="store_true")
    parser.add_argument("--verify-authorization", action="store_true")
    parser.add_argument("--verify-pre-raw", action="store_true")
    args = parser.parse_args()
    if not any((args.verify_freeze, args.verify_manifest, args.verify_authorization, args.verify_pre_raw)):
        parser.error("one verification mode is required")
    try:
        if args.verify_freeze:
            verify_freeze()
        if args.verify_manifest:
            verify_manifest()
        if args.verify_authorization:
            verify_authorization()
        if args.verify_pre_raw:
            print(json.dumps(verify_pre_raw(), indent=2, sort_keys=True))
    except (Phase117ContractError, OSError) as exc:
        print(f"phase117 pre-raw verifier: fail-closed: {exc}", file=sys.stderr)
        raise SystemExit(2)
