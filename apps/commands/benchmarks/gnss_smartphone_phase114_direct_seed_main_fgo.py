#!/usr/bin/env python3
"""Launch-free Phase114 direct-seed structural contract.

The Phase114 candidate deliberately bypasses the GNSS-first optimizer.  This
contract therefore validates the same-run raw WLS/C7/D/velocity adapter and
the existing Phase99 QR + raw-base + final-output-offset recipe separately.
Before the independent authorization this module reads only source files and
sealed metadata; it never stats, hashes, opens, or launches an input member.
"""

from __future__ import annotations

import hashlib
import importlib.util
import json
from pathlib import Path
import sys
from typing import Any


ROOT = Path(__file__).resolve().parents[3]
FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_phase114_direct_seed_main_fgo_freeze_v1.json"
MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase114_direct_seed_main_fgo_manifest_v1.json"
AUTHORIZATION = ROOT / "docs/use_cases/records/smartphone_r5_phase114_direct_seed_main_fgo_authorization_v1.json"
EVALUATOR = Path(__file__).resolve()
WRAPPER = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase114_direct_seed_main_fgo_execute.py"
FOCUSED_TESTS = ROOT / "tests/test_smartphone_phase114_direct_seed_main_fgo.py"
PHASE95_RESULT = ROOT / "docs/use_cases/records/smartphone_r5_phase95_raw_input_path_corrected_structural_result_v1.json"
PHASE65_MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase65_native_base_pseudorange_compensation_manifest_v1.json"

APP = ROOT / "apps/native/gnss_fgo_imu_no_base.cpp"
FGO = ROOT / "include/libgnss++/algorithms/fgo.hpp"
CONFIG = ROOT / "include/libgnss++/algorithms/fgo_config.hpp"
C7_HEADER = ROOT / "include/libgnss++/algorithms/source_clock_c0d_initializer.hpp"
BACKEND = ROOT / "src/algorithms/fgo_gtsam_backend.cpp"
INTERNAL = ROOT / "src/algorithms/fgo_gtsam_internal.hpp"
BINARY = ROOT / "build/apps/gnss_fgo_imu_no_base"

FREEZE_SHA256 = "aa47295bffd61d72c851a06043d5ceb56c96dcde94059442ddfd97ed5a36b838"
FREEZE_COMMIT = "cb365712e32b354ff8b335acb9ad29276b2240f5"
AUDIT_COMMIT = "34d307b7d3068e3610ece037c5b2a8b6d48198aa"
IMPLEMENTATION_COMMIT = "44771ef6b52eddb0df6193b8b9d9f1b52b7fcea5"
PHASE95_RESULT_SHA256 = "beba25d4b0d910064d76bea7aa6b7a884937ca01c3b00cf0f7f9f4b6267ea281"
PHASE65_MANIFEST_SHA256 = "1306480f6b7fb839e55309e9c2c77b41f2ac0a1baf9f6971949612fd485d2255"
SOURCE_SHA256 = {
    "app": "301265974631f20781ff4460e1ede4032165e11c79ef59d0510a273aebb1a59e",
    "fgo": "0b0b3665383ca4b465bcfbf6a9161efe7050e093ad36161f1d9b00bd9029c75c",
    "config": "23d0b67aa33b2186a610c07ba8aa2022476f12d518974bd6d5929fa135a60fa9",
    "c7_header": "cf8562c75307f79a8b6a35262dac0f6e9d04588b23145169d076594a9c7889a4",
    "backend": "78792c9306abe339228a3004e3d23f1486fac77822d0f1785b69dc07af14f578",
    "internal": "7aba732064d2cd6b3858c60226b84aa904423becdf66cb386e628fae16fc5afd",
    "binary": "96374063aa91aea6ea72fd1402b6c4895cb5f004a506e57e10c620234fa0bd69",
}

ROUTES = (
    "2021-03-16-18-59-us-ca-mtv-a/pixel5",
    "2022-04-01-18-22-us-ca-lax-t/pixel5",
)
DOMAIN_ROWS = {ROUTES[0]: 2158, ROUTES[1]: 1465}
PROBLEM_EPOCHS = {route: rows + 1 for route, rows in DOMAIN_ROWS.items()}
RAW_NAMES = ("device_gnss.csv", "device_imu.csv", "brdc.nav")
RAW_FLAGS = (
    ("--android-gnss", "device_gnss.csv"),
    ("--android-imu", "device_imu.csv"),
    ("--nav", "brdc.nav"),
)
DIRECT_SELECTOR = "--native-direct-wls-ephemeral-c7d-main-seed"
VECTOR_SELECTOR = "--native-source-clock-c0d-epoch-vector-parity"
QR_SELECTOR = "--native-source-clock-c0d-phase99-main-multifrontal-qr-solver"
BASE_SELECTORS = (
    "--native-base-pseudorange-compensation",
    "--native-base-pseudorange-source-miss-mask",
    "--native-base-pseudorange-preserve-additional-frequency-bands",
)
OFFSET_SELECTOR = "--native-upstream-position-offset"
BASE_FLAG = ("--native-base-rinex", "__PHASE114_RAW_BASE_RINEX__")
BASE_SHA_FLAG = ("--native-base-rinex-sha256", "__PHASE114_RAW_BASE_SHA256__")
CANDIDATE_ID = "phase114-main-direct-wls-ephemeral-c7d-seed-v1"
OUTPUT_ROOT = "output/smartphone-r5/phase114-main-direct-wls-c7d-v1/"
SCHEMA = "smartphone-r5-phase114-direct-seed-main-fgo-manifest.v1"
AUTH_SCHEMA = "smartphone-r5-phase114-direct-seed-main-fgo-authorization.v1"

REQUIRED_FLAGS = (
    "--all-epochs", "--android-raw-utc-keys", "--android-raw-clock-only",
    "--android-utc-wall-clock-fallback", "--native-pdc-imu-tdcp-no-bridge",
    "--native-source-direct-observable-quality", "--native-source-clock-c0d-factor",
    "--native-source-clock-c0d-meter-state-parity",
    "--native-source-clock-c0d-active-solve-diagnostic", VECTOR_SELECTOR,
    QR_SELECTOR, DIRECT_SELECTOR, *BASE_SELECTORS, OFFSET_SELECTOR,
)
FORBIDDEN_FLAGS = (
    "--obs", "--native-gnss-first-velocity-only-handoff",
    "--native-direct-doppler-wls-handoff", "--native-pdc-state-bridge",
    "--native-upstream-quality",
    "--native-source-clock-c0d-gnss-first-raw-drift-d-initializer",
    "--native-source-clock-c0d-gnss-first-meter-state-handoff",
    "--native-source-clock-c0d-phase94-stage-diagnostics",
    "--native-source-clock-c0d-phase96-main-diagnostics",
    "--native-source-clock-c0d-phase97-singular-system-diagnostics",
    "--native-source-clock-c0d-phase98-solver-rank-diagnostic",
    "--native-phase104-stage-main-attribution", "--native-quality-anchor",
)
FORBIDDEN_PATH_TERMS = (
    ".mat", "truth", "ground_truth", "validation", "holdout", "precomputed",
    "coordinate", "pdc", "kaggle", "token",
)


class Phase114ContractError(ValueError):
    """Raised when the pinned Phase114 contract fails closed."""


def fail(message: str) -> Phase114ContractError:
    return Phase114ContractError(message)


def relative(path: Path) -> str:
    try:
        return str(path.relative_to(ROOT))
    except ValueError:
        return str(path)


def assert_equal(actual: Any, expected: Any, label: str) -> None:
    if actual != expected:
        raise fail(f"{label}: expected {expected!r}, got {actual!r}")


def sha256_file(path: Path, label: str) -> str:
    # Never permit a pre-authorization verifier to touch an input payload.
    if path.name in RAW_NAMES or path.name == "base.obs":
        raise fail(f"input-member hash forbidden before authorization: {label}")
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


def _phase109_metadata_module() -> Any:
    path = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase109_raw_base_frequency_parity.py"
    spec = importlib.util.spec_from_file_location("phase109_sealed_metadata", path)
    if spec is None or spec.loader is None:
        raise fail("unable to load sealed Phase109 metadata helpers")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def sealed_input_metadata() -> tuple[dict[str, dict[str, dict[str, Any]]], dict[str, dict[str, Any]]]:
    # The imported helpers read only sealed JSON records and validate the
    # recorded paths.  They never stat or open the members they describe.
    module = _phase109_metadata_module()
    return module.phase95_raw_metadata(), module.phase65_base_metadata()


def safe_relative_member(path_text: Any, basename: str, label: str) -> None:
    if not isinstance(path_text, str):
        raise fail(f"{label} path missing")
    path = Path(path_text)
    if path.is_absolute() or ".." in path.parts or path.name != basename:
        raise fail(f"unsafe {label} path: {path_text}")


def verify_freeze() -> dict[str, Any]:
    assert_equal(sha256_file(FREEZE, "Phase114 freeze"), FREEZE_SHA256, "freeze/sha256")
    freeze = read_json(FREEZE, "Phase114 freeze")
    for key, expected in {
        "schema_version": "smartphone-r5-phase114-direct-seed-main-fgo-freeze.v1",
        "phase": 114, "execution_label": "Luna Max",
        "status": "frozen-detailed-boundary-not-implemented",
    }.items():
        assert_equal(freeze.get(key), expected, f"freeze/{key}")
    decision = freeze.get("decision")
    if not isinstance(decision, dict):
        raise fail("freeze/decision missing")
    for key, expected in {
        "candidate_count": 1, "exactly_one_default_off_candidate": True,
        "source_backed_hypothesis": True, "implementation_established": False,
        "raw_execution_authorized": False, "solver_execution_authorized": False,
        "truth_evaluation_authorized": False,
        "accuracy_or_promotion_authorized": False,
        "solution_publication_authorized": False,
    }.items():
        assert_equal(decision.get(key), expected, f"freeze/decision/{key}")
    candidate = freeze.get("candidate")
    if not isinstance(candidate, dict):
        raise fail("freeze/candidate missing")
    for key, expected in {
        "id": CANDIDATE_ID, "implemented": False, "opt_in": True,
        "default_off": True, "same_process_ephemeral": True,
        "bypass_gnss_first_for_candidate_only": True,
        "existing_gnss_first_unchanged": True,
        "legacy_default_unchanged": True,
    }.items():
        assert_equal(candidate.get(key), expected, f"freeze/candidate/{key}")
    seed = candidate.get("seed_components")
    if not isinstance(seed, dict):
        raise fail("freeze/seed_components missing")
    if "same-run raw EpochSeed" not in str(seed.get("position", "")):
        raise fail("freeze position seed provenance changed")
    if "EpochSeed.receiver_clock_drift_mps" not in str(seed.get("clock_drift_d", "")):
        raise fail("freeze D seed provenance changed")
    required = candidate.get("required_new_boundary")
    if not isinstance(required, dict):
        raise fail("freeze/required_new_boundary missing")
    assert_equal(required.get("c7_component_order"), [
        "base_gps_l1", "glo_l1", "gal_l1", "bds_l1", "gps_l5", "gal_l5", "bds_l5",
    ], "freeze/C7 component order")
    return freeze


def verify_implementation() -> None:
    paths = {
        "app": APP, "fgo": FGO, "config": CONFIG, "c7_header": C7_HEADER,
        "backend": BACKEND, "internal": INTERNAL, "binary": BINARY,
    }
    for key, path in paths.items():
        assert_equal(sha256_file(path, f"implementation/{key}"), SOURCE_SHA256[key], f"implementation/{key}/sha256")
    source = APP.read_text(encoding="utf-8")
    for marker in (
        "native_direct_wls_ephemeral_c7d_main_seed",
        "validateDirectWlsEphemeralMainSeed",
        "main-direct-wls-ephemeral-c7d-seed",
        "gnss_first_stage_run",
        "epoch_clock_drift_mps",
    ):
        if marker not in source:
            raise fail(f"Phase114 implementation marker missing: {marker}")
    c7 = C7_HEADER.read_text(encoding="utf-8")
    if "validateAndCopyDirectWlsEphemeralC7DSeed" not in c7:
        raise fail("Phase114 exact C7/D adapter marker missing")
    backend = BACKEND.read_text(encoding="utf-8")
    if "use_native_direct_wls_ephemeral_main_seed" not in backend:
        raise fail("Phase114 backend handoff marker missing")


def output_path(route: str, name: str) -> str:
    return f"{OUTPUT_ROOT}{route.replace('/', '__')}/{name}"


def command_template(route: str) -> list[str]:
    return [
        "build/apps/gnss_fgo_imu_no_base", "--dataset-id", route,
        "--android-gnss", "__PHASE114_RAW_DEVICE_GNSS__",
        "--android-imu", "__PHASE114_RAW_DEVICE_IMU__",
        "--nav", "__PHASE114_RAW_BROADCAST_NAV__", "--all-epochs",
        "--android-raw-utc-keys", "--android-raw-clock-only",
        "--android-utc-wall-clock-fallback", "--native-pdc-imu-tdcp-no-bridge",
        "--native-source-direct-observable-quality", "--native-source-clock-c0d-factor",
        "--native-source-clock-c0d-meter-state-parity",
        "--native-source-clock-c0d-active-solve-diagnostic", VECTOR_SELECTOR,
        QR_SELECTOR, DIRECT_SELECTOR, *BASE_SELECTORS, OFFSET_SELECTOR,
        BASE_FLAG[0], BASE_FLAG[1], BASE_SHA_FLAG[0], BASE_SHA_FLAG[1],
        "--out", output_path(route, "withheld_solution_output.csv"),
        "--summary-json", output_path(route, "summary.json"),
    ]


def validate_command(route: str, command: Any, record: dict[str, Any], raw: dict[str, Any], base: dict[str, Any]) -> None:
    assert_equal(command, command_template(route), f"manifest/{route}/exact command")
    for token in command:
        if token in FORBIDDEN_FLAGS or (
            not token.startswith("--") and any(term in token.lower() for term in FORBIDDEN_PATH_TERMS)
        ):
            raise fail(f"forbidden command token: {route}/{token}")
    for flag in REQUIRED_FLAGS:
        assert_equal(command.count(flag), 1, f"command/{route}/{flag}")
    if DIRECT_SELECTOR in command and any(flag in command for flag in ("--native-source-clock-c0d-gnss-first-meter-state-handoff", "--native-direct-doppler-wls-handoff")):
        raise fail(f"direct recipe composes a forbidden staging/raw-D selector: {route}")
    for flag, name in RAW_FLAGS:
        assert_equal(command.count(flag), 1, f"command/{route}/{flag}")
        placeholder = {
            "device_gnss.csv": "__PHASE114_RAW_DEVICE_GNSS__",
            "device_imu.csv": "__PHASE114_RAW_DEVICE_IMU__",
            "brdc.nav": "__PHASE114_RAW_BROADCAST_NAV__",
        }[name]
        assert_equal(command[command.index(flag) + 1], placeholder, f"command/{route}/{name}/placeholder")
        pin = record.get("raw_inputs", {}).get(name)
        if not isinstance(pin, dict):
            raise fail(f"manifest raw pin missing: {route}/{name}")
        assert_equal(pin.get("path"), raw[name]["path"], f"manifest/raw/{route}/{name}/path")
        assert_equal(pin.get("sha256"), raw[name]["sha256"], f"manifest/raw/{route}/{name}/sha256")
        assert_equal(pin.get("read_at_manifest_creation"), False, f"manifest/raw/{route}/{name}/read")
    base_pin = record.get("base_input")
    if not isinstance(base_pin, dict):
        raise fail(f"manifest base pin missing: {route}")
    for key in ("path", "sha256", "bytes", "observed_dt_s", "moving_mean_samples", "approx_position_xyz_m"):
        assert_equal(base_pin.get(key), base.get(key), f"manifest/base/{route}/{key}")
    for key in ("read_at_manifest_creation", "hash_read_at_manifest_creation"):
        assert_equal(base_pin.get(key), False, f"manifest/base/{route}/{key}")
    assert_equal(command[command.index("--out") + 1], output_path(route, "withheld_solution_output.csv"), f"command/{route}/out")
    assert_equal(command[command.index("--summary-json") + 1], output_path(route, "summary.json"), f"command/{route}/summary")
    assert_equal(record.get("runs"), 1, f"manifest/{route}/runs")


def verify_manifest() -> dict[str, Any]:
    verify_freeze()
    verify_implementation()
    raw, base = sealed_input_metadata()
    manifest = read_json(MANIFEST, "Phase114 structural manifest")
    for key, expected in {
        "schema_version": SCHEMA, "phase": 114, "execution_label": "Luna Max",
        "status": "sealed-before-phase114-direct-seed-raw-execution",
    }.items():
        assert_equal(manifest.get(key), expected, f"manifest/{key}")
    freeze = manifest.get("freeze")
    if not isinstance(freeze, dict):
        raise fail("manifest/freeze missing")
    for key, expected in {
        "path": relative(FREEZE), "sha256": FREEZE_SHA256,
        "commit": FREEZE_COMMIT, "audit_commit": AUDIT_COMMIT,
        "raw_execution_authorized_before_manifest": False,
    }.items():
        assert_equal(freeze.get(key), expected, f"manifest/freeze/{key}")
    implementation = manifest.get("implementation")
    if not isinstance(implementation, dict):
        raise fail("manifest/implementation missing")
    for key, expected in {
        "commit": IMPLEMENTATION_COMMIT, "candidate_id": CANDIDATE_ID,
        "legacy_default_unchanged": True,
        "solver_filter_lm_equation_unit_sigma_c7_d_changed": False,
        "gnss_first_bypassed_for_candidate_only": True,
    }.items():
        assert_equal(implementation.get(key), expected, f"manifest/implementation/{key}")
    assert_equal(implementation.get("source_sha256"), SOURCE_SHA256, "manifest/implementation/source_sha256")
    for key, path in (("evaluator", EVALUATOR), ("wrapper", WRAPPER), ("focused_tests", FOCUSED_TESTS)):
        pin = manifest.get(key)
        if not isinstance(pin, dict):
            raise fail(f"manifest/{key} missing")
        assert_equal(pin.get("path"), relative(path), f"manifest/{key}/path")
        expected_hash = pin.get("sha256")
        if not isinstance(expected_hash, str) or len(expected_hash) != 64:
            raise fail(f"manifest/{key}/sha256 missing")
        assert_equal(sha256_file(path, f"manifest/{key}"), expected_hash, f"manifest/{key}/sha256")
    candidate = manifest.get("candidate")
    if not isinstance(candidate, dict):
        raise fail("manifest/candidate missing")
    for key, expected in {
        "id": CANDIDATE_ID, "candidate_count": 1, "opt_in": True,
        "default_off_outside_exact_recipe": True,
        "selector": DIRECT_SELECTOR,
        "selectors": [DIRECT_SELECTOR, VECTOR_SELECTOR, QR_SELECTOR, OFFSET_SELECTOR],
        "base_selectors": list(BASE_SELECTORS), "gnss_first_stage": "bypassed by design",
        "same_run_ephemeral_seed": True, "runs_per_route": 1, "controls": 0,
        "reruns": 0, "fallbacks": 0, "truth_evaluation": False,
        "accuracy_scoring": False, "solution_output_publication": False,
        "no_global_isb_double_state": True, "base_correction_applied_exactly_once": True,
        "final_pixel5_offset_applied_exactly_once": True,
        "velocity_synthesis": False, "zero_d_or_c7_fill": False,
        "equation_unit_sigma_filter_lm_changed": False,
    }.items():
        assert_equal(candidate.get(key), expected, f"manifest/candidate/{key}")
    routes = manifest.get("routes")
    if not isinstance(routes, list) or [item.get("dataset_id") for item in routes] != list(ROUTES):
        raise fail("manifest route order/count changed")
    for record in routes:
        route = record.get("dataset_id")
        if route not in DOMAIN_ROWS:
            raise fail(f"unknown route: {route}")
        assert_equal(record.get("domain_rows"), DOMAIN_ROWS[route], f"manifest/{route}/domain rows")
        assert_equal(record.get("expected_problem_epochs"), PROBLEM_EPOCHS[route], f"manifest/{route}/problem epochs")
        assert_equal(record.get("expected_output_epochs"), PROBLEM_EPOCHS[route], f"manifest/{route}/output epochs")
        validate_command(route, record.get("command"), record, raw[route], base[route])
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
    direct_contract = manifest.get("direct_seed_contract")
    if not isinstance(direct_contract, dict):
        raise fail("manifest/direct_seed_contract missing")
    for key, expected in {
        "gnss_first_attempted": False, "position_source": "same-run raw EpochSeed SPP",
        "velocity_source": "same-run raw Doppler WLS ECEF converted to native ENU",
        "clock_bias_source": "same-run raw EpochSeed receiver_clock_bias_m",
        "clock_drift_source": "exact retained EpochSeed.receiver_clock_drift_mps",
        "c_vector_dimension": 7,
        "c_vector_unit": "metres", "d_unit": "metres_per_second",
        "c_vector_component_order": ["base_gps_l1", "glo_l1", "gal_l1", "bds_l1", "gps_l5", "gal_l5", "bds_l5"],
        "exact_key_alignment": True, "full_finite_coverage_required": True,
        "d_zero_fill": False, "c_zero_fill": False, "velocity_synthesis": False,
        "nearest_interpolation": False, "fallback": False,
    }.items():
        assert_equal(direct_contract.get(key), expected, f"manifest/direct_seed_contract/{key}")
    base_contract = manifest.get("base_contract")
    if not isinstance(base_contract, dict):
        raise fail("manifest/base_contract missing")
    for key, expected in {
        "existing_reader_selector_only": True, "preserve_additional_frequency_bands": True,
        "correction_formula": "P_rover_corrected_m=P_rover_raw_m-pc(t)",
        "stream_key": "exact satellite and SignalType", "in_domain_interpolation_only": True,
        "no_extrapolation_or_endpoint_hold": True, "source_miss_mask": True,
        "base_coordinate_source": "raw base RINEX header APPROX POSITION XYZ",
        "hash_verification_reads_per_route": 1, "native_process_reads_per_route": 1,
    }.items():
        assert_equal(base_contract.get(key), expected, f"manifest/base_contract/{key}")
    gates = manifest.get("structural_gates")
    if not isinstance(gates, dict):
        raise fail("manifest/structural_gates missing")
    for key in (
        "direct_seed_full_finite_exact_key_handoff",
        "main_qr_progress_strict_cost_decrease",
        "main_meter_c0d_units_sigma_factor_gate",
        "base_factors_active_exactly_once",
        "final_pixel5_offset_exactly_once",
        "finite_earth_valid_expected_output_coverage",
        "no_gnss_first_stage_or_solver_fallback",
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
    auth = read_json(AUTHORIZATION, "Phase114 raw authorization")
    for key, expected in {
        "schema_version": AUTH_SCHEMA, "phase": 114, "execution_label": "Luna Max",
        "status": "authorized-for-exact-two-route-phase114-direct-seed-raw-structural-execution",
    }.items():
        assert_equal(auth.get(key), expected, f"authorization/{key}")
    authority = auth.get("authority")
    if not isinstance(authority, dict):
        raise fail("authorization/authority missing")
    for key, path, expected in (
        ("phase114_freeze", FREEZE, FREEZE_SHA256),
        ("phase114_manifest", MANIFEST, sha256_file(MANIFEST, "Phase114 manifest")),
        ("phase114_evaluator", EVALUATOR, sha256_file(EVALUATOR, "Phase114 evaluator")),
        ("phase114_wrapper", WRAPPER, sha256_file(WRAPPER, "Phase114 wrapper")),
        ("phase114_focused_tests", FOCUSED_TESTS, sha256_file(FOCUSED_TESTS, "Phase114 focused tests")),
        ("phase95_result", PHASE95_RESULT, PHASE95_RESULT_SHA256),
        ("phase65_manifest", PHASE65_MANIFEST, PHASE65_MANIFEST_SHA256),
    ):
        pin = authority.get(key)
        if not isinstance(pin, dict):
            raise fail(f"authorization/{key} missing")
        assert_equal(pin.get("path"), relative(path), f"authorization/{key}/path")
        assert_equal(pin.get("sha256"), expected, f"authorization/{key}/sha256")
    for key, expected in {
        "freeze_commit": FREEZE_COMMIT, "audit_commit": AUDIT_COMMIT,
        "implementation_commit": IMPLEMENTATION_COMMIT,
        "manifest_sha256": sha256_file(MANIFEST, "Phase114 manifest"),
        "evaluator_sha256": sha256_file(EVALUATOR, "Phase114 evaluator"),
        "wrapper_sha256": sha256_file(WRAPPER, "Phase114 wrapper"),
        "binary_sha256": SOURCE_SHA256["binary"],
    }.items():
        assert_equal(authority.get(key), expected, f"authorization/authority/{key}")
    assert_equal(auth.get("routes"), list(ROUTES), "authorization/routes")
    candidate = auth.get("candidate")
    if not isinstance(candidate, dict):
        raise fail("authorization/candidate missing")
    for key, expected in {
        "id": CANDIDATE_ID, "candidate_count": 1,
        "selectors": [DIRECT_SELECTOR, VECTOR_SELECTOR, QR_SELECTOR, OFFSET_SELECTOR],
        "base_selectors": list(BASE_SELECTORS), "gnss_first_stage": "bypassed by design",
        "runs_per_route": 1, "controls": 0, "reruns": 0, "fallbacks": 0,
        "truth_evaluation": False, "accuracy_scoring": False,
        "solution_output_publication": False, "no_fallback_or_guard_bypass": True,
        "direct_seed_exact_full_finite_gate": True, "base_correction_exactly_once": True,
        "pixel5_offset_exactly_once": True,
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
        "one_invocation_per_route": True, "no_rerun": True, "no_fallback": True,
        "solver_filter_lm_equation_unit_sigma_unchanged": True,
        "gnss_first_stage": "not attempted by design",
        "truth_or_accuracy_evaluation": False,
        "solution_output": "isolated withheld path; stat metadata only, never opened/published",
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
        "status": "pre-raw-verified", "phase": 114, "execution_label": "Luna Max",
        "candidate_count": 1, "route_ids": list(ROUTES), "runs_per_route": 1,
        "raw_execution_authorized": False, "raw_phone_gnss_reads": 0,
        "raw_phone_imu_reads": 0, "broadcast_navigation_reads": 0,
        "raw_base_rinex_reads": 0, "raw_base_hash_reads": 0,
        "native_solver_invocations": 0, "truth_reads": 0,
        "mat_reads_or_generated": 0, "phone_coordinate_reads": 0,
        "precomputed_coordinate_reads": 0, "pdc_reads": 0,
        "accuracy_calculations": 0, "solution_output_published": False,
        "freeze_sha256": FREEZE_SHA256,
        "manifest_sha256": sha256_file(MANIFEST, "Phase114 manifest"),
        "freeze_status": freeze["status"], "manifest_status": manifest["status"],
    }


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--verify-freeze", action="store_true")
    parser.add_argument("--verify-manifest", action="store_true")
    parser.add_argument("--verify-pre-raw", action="store_true")
    args = parser.parse_args()
    if not any((args.verify_freeze, args.verify_manifest, args.verify_pre_raw)):
        parser.error("one verification mode is required")
    try:
        if args.verify_freeze:
            verify_freeze()
        if args.verify_manifest:
            verify_manifest()
        if args.verify_pre_raw:
            print(json.dumps(verify_pre_raw(), indent=2, sort_keys=True))
    except (Phase114ContractError, OSError) as exc:
        print(f"phase114 pre-raw verifier: fail-closed: {exc}", file=sys.stderr)
        raise SystemExit(2)
