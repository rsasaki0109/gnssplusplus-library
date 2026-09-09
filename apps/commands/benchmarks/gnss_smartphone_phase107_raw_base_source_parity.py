#!/usr/bin/env python3
"""Launch-free Phase107 raw-base structural contract.

This module is deliberately a pre-raw verifier.  It reads only pinned source
files and sealed Phase95/65 metadata.  It never stats, opens, hashes, copies,
or transforms a device GNSS/IMU/navigation file or a base RINEX member, and it
never launches the native solver.  The separate authorization record is the
boundary at which the execution wrapper may materialize those paths.
"""

from __future__ import annotations

import hashlib
import json
from pathlib import Path
import sys
from typing import Any


ROOT = Path(__file__).resolve().parents[3]
EVALUATOR = Path(__file__).resolve()
MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase107_raw_base_source_parity_manifest_v1.json"
AUTHORIZATION = ROOT / "docs/use_cases/records/smartphone_r5_phase107_raw_base_source_parity_authorization_v1.json"
FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_phase107_raw_base_source_parity_freeze_v1.json"
AUDIT = ROOT / "docs/use_cases/records/smartphone_r5_phase107_raw_base_source_parity_audit_v1.md"
PHASE95_RESULT = ROOT / "docs/use_cases/records/smartphone_r5_phase95_raw_input_path_corrected_structural_result_v1.json"
PHASE65_MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase65_native_base_pseudorange_compensation_manifest_v1.json"
WRAPPER = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase107_raw_base_source_parity_execute.py"
FOCUSED_TESTS = ROOT / "tests/test_smartphone_phase107_raw_base_contract.py"

APP = ROOT / "apps/native/gnss_fgo_imu_no_base.cpp"
BACKEND = ROOT / "src/algorithms/fgo_gtsam_backend.cpp"
INTERNAL = ROOT / "src/algorithms/fgo_gtsam_internal.hpp"
FGO_RESULT = ROOT / "include/libgnss++/algorithms/fgo.hpp"
CONFIG = ROOT / "include/libgnss++/algorithms/fgo_config.hpp"
BASE_MODEL = ROOT / "src/algorithms/base_pseudorange_compensation.cpp"
BASE_MODEL_HEADER = ROOT / "include/libgnss++/algorithms/base_pseudorange_compensation.hpp"
RINEX = ROOT / "src/io/rinex.cpp"
BINARY = ROOT / "build/apps/gnss_fgo_imu_no_base"

FREEZE_SHA = "8c05c89dda9cb3356b4c9bdf1c0ec1f2f41145bb37a7b5aee7ff12978aba9d76"
FREEZE_COMMIT = "9598fe7a0c91598a7e36f7dc99e598120f8ed234"
AUDIT_SHA = "79d1c79510138498c05d4096ed7c3f97b34b1ee16690f3fc7a7b57af3b0192ab"
AUDIT_COMMIT = "f047d3341843ab7262c5988264ca6424f49170ae"
IMPLEMENTATION_COMMIT = "50ee574e692752ded7d5041aea24b6178e9a9c76"
APP_SHA = "0a14710768f107d025ef2505450ca85c545e3c0a47e52191f685094838a9c51f"
BACKEND_SHA = "781d65e34826ec08e04d1dfdac0ab10b2ed9409726078465f3020b57afac4baa"
INTERNAL_SHA = "cc6327e36800afa8217e5a834260adc81c0a74a5aefbfc956bb4679c8d86105f"
FGO_RESULT_SHA = "5a26994bda96c4bf50f436b1368bc1df5b192c38ef13a2a882ebd8e15bf5103d"
CONFIG_SHA = "3cc60abe514ef8900012064accb17f27d3927d0b8a8776ad76ce6aefa83ce32c"
BASE_MODEL_SHA = "f3c3c859793c4bed9c7ae93825d32565f20eab0badb09412a4fab2a5930c9817"
BASE_MODEL_HEADER_SHA = "a182091ef47af8692e3f4dfb0d714cf365f5b24bc6927a8204d05648248e026b"
RINEX_SHA = "92f683e34b4052f8d105f69fc4bf4397ca990be3b5a6b220181a56423f465533"
BINARY_SHA = "c284d22f2c998177a8a78c77209f346b7d6b9d56899ded833ffad94d3903cd6e"
PHASE95_RESULT_SHA = "beba25d4b0d910064d76bea7aa6b7a884937ca01c3b00cf0f7f9f4b6267ea281"
PHASE65_MANIFEST_SHA = "1306480f6b7fb839e55309e9c2c77b41f2ac0a1baf9f6971949612fd485d2255"

ROUTES = (
    "2021-03-16-18-59-us-ca-mtv-a/pixel5",
    "2022-04-01-18-22-us-ca-lax-t/pixel5",
)
DOMAIN_ROWS = {ROUTES[0]: 2158, ROUTES[1]: 1465}
RAW_NAMES = ("device_gnss.csv", "device_imu.csv", "brdc.nav")
RAW_FLAGS = (
    ("--android-gnss", "device_gnss.csv", "__PHASE107_RAW_DEVICE_GNSS__"),
    ("--android-imu", "device_imu.csv", "__PHASE107_RAW_DEVICE_IMU__"),
    ("--nav", "brdc.nav", "__PHASE107_RAW_BROADCAST_NAV__"),
)
BASE_FLAG = ("--native-base-rinex", "__PHASE107_RAW_BASE_RINEX__")
BASE_SHA_FLAG = ("--native-base-rinex-sha256", "__PHASE107_RAW_BASE_SHA256__")
REQUIRED_FLAGS = (
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
)
FORBIDDEN_FLAGS = (
    "--obs",
    "--native-gnss-first-velocity-only-handoff",
    "--native-direct-doppler-wls-handoff",
    "--native-pdc-state-bridge",
    "--native-upstream-quality",
    "--native-base-pseudorange-preserve-additional-frequency-bands",
    "--native-source-clock-c0d-gnss-first-raw-drift-d-initializer",
    "--native-source-clock-c0d-phase94-stage-diagnostics",
    "--native-source-clock-c0d-phase96-main-diagnostics",
    "--native-source-clock-c0d-phase97-singular-system-diagnostics",
    "--native-source-clock-c0d-phase98-solver-rank-diagnostic",
    "--native-phase104-stage-main-attribution",
    "--native-upstream-position-offset",
    "--native-quality-anchor",
)
FORBIDDEN_PATH_TERMS = (
    ".mat", "ground_truth", "validation", "holdout", "kaggle", "token",
    "precomputed", "coordinate", "truth", "base.rinex",
)
PHASE93_SELECTOR = "--native-source-clock-c0d-gnss-first-meter-state-handoff"
VECTOR_SELECTOR = "--native-source-clock-c0d-epoch-vector-parity"
QR_SELECTOR = "--native-source-clock-c0d-phase99-main-multifrontal-qr-solver"
CANDIDATE_ID = "phase107-raw-base-rinex-source-exact-pseudorange-phase101-c7d-qr-v1"
SCHEMA = "smartphone-r5-phase107-raw-base-source-parity-manifest.v1"
AUTH_SCHEMA = "smartphone-r5-phase107-raw-base-source-parity-authorization.v1"
OUTPUT_ROOT = "output/smartphone-r5/phase107-raw-base-source-parity-v1/"


class Phase107ContractError(ValueError):
    """Raised when the pinned Phase107 contract fails closed."""


def fail(message: str) -> Phase107ContractError:
    return Phase107ContractError(message)


def relative(path: Path) -> str:
    try:
        return str(path.relative_to(ROOT))
    except ValueError:
        return str(path)


def assert_equal(actual: Any, expected: Any, label: str) -> None:
    if actual != expected:
        raise fail(f"{label}: expected {expected!r}, got {actual!r}")


def _sealed_hash_file(path: Path, label: str) -> str:
    """Hash only a pinned source/sealed artifact, never an input member."""

    lowered = path.name.lower()
    if lowered in RAW_NAMES or lowered.endswith(".obs"):
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


def _safe_sealed_path(path_text: Any, route: str, name: str) -> None:
    if not isinstance(path_text, str):
        raise fail(f"sealed {name} path missing: {route}")
    path = Path(path_text)
    if path.is_absolute() or ".." in path.parts or path.name != name:
        raise fail(f"unsafe sealed {name} path: {route}: {path_text}")
    if any(term in path_text.lower() for term in FORBIDDEN_PATH_TERMS):
        raise fail(f"forbidden sealed {name} path term: {route}: {path_text}")


def phase95_raw_metadata() -> dict[str, dict[str, dict[str, Any]]]:
    """Read Phase95 raw path/hash metadata without touching the raw files."""

    assert_equal(_sealed_hash_file(PHASE95_RESULT, "Phase95 result"), PHASE95_RESULT_SHA, "Phase95 result sha256")
    result = read_json(PHASE95_RESULT, "Phase95 result")
    routes = result.get("routes")
    if not isinstance(routes, dict):
        raise fail("Phase95 routes metadata missing")
    resolved: dict[str, dict[str, dict[str, Any]]] = {}
    for route in ROUTES:
        record = routes.get(route)
        if not isinstance(record, dict):
            raise fail(f"Phase95 route missing: {route}")
        raw = record.get("raw_inputs")
        if not isinstance(raw, dict) or set(raw) != set(RAW_NAMES):
            raise fail(f"Phase95 raw role set changed: {route}")
        resolved[route] = {}
        for name in RAW_NAMES:
            pin = raw.get(name)
            if not isinstance(pin, dict):
                raise fail(f"Phase95 raw pin missing: {route}/{name}")
            _safe_sealed_path(pin.get("path"), route, name)
            assert_equal(pin.get("exists_before_launch"), True, f"Phase95/{route}/{name}/exists")
            assert_equal(pin.get("read_by_runner"), False, f"Phase95/{route}/{name}/runner-read")
            digest = pin.get("sha256")
            if not isinstance(digest, str) or len(digest) != 64:
                raise fail(f"Phase95 raw digest missing: {route}/{name}")
            resolved[route][name] = {
                "path": pin["path"],
                "sha256": digest,
                "bytes": pin.get("bytes"),
                "manifest": pin.get("manifest"),
                "read_at_manifest_creation": False,
                "read_by_pre_raw_verifier": False,
            }
    return resolved


def phase65_base_metadata() -> dict[str, dict[str, Any]]:
    """Read sealed base-member metadata; no base member is opened or stated."""

    assert_equal(_sealed_hash_file(PHASE65_MANIFEST, "Phase65 base manifest"), PHASE65_MANIFEST_SHA, "Phase65 manifest sha256")
    manifest = read_json(PHASE65_MANIFEST, "Phase65 base manifest")
    base_inputs = manifest.get("base_inputs")
    if not isinstance(base_inputs, dict):
        raise fail("Phase65 base_inputs missing")
    resolved: dict[str, dict[str, Any]] = {}
    expected = {
        ROUTES[0]: {
            "path": "output/smartphone-r5/phase63-settings-integrity-recovery-v1/routes/2021-03-16-18-59-us-ca-mtv-a__pixel5/base.obs",
            "sha256": "380b8ff9091344fb756697e27f0983d9a0ba2cf0c201b96849bfc1ecc1af0e52",
            "bytes": 10708536, "observed_dt_s": 1.0, "moving_mean_samples": 151,
            "approx_position_xyz_m": [-2703115.921, -4291767.2078, 3854247.9066],
        },
        ROUTES[1]: {
            "path": "output/smartphone-r5/phase63-settings-integrity-recovery-v1/routes/2022-04-01-18-22-us-ca-lax-t__pixel5/base.obs",
            "sha256": "d731e0e8a7ba4396d62340c85b6238e66c50c6a349621f9dcea0ea6885fd4cfe",
            "bytes": 719969, "observed_dt_s": 15.0, "moving_mean_samples": 11,
            "approx_position_xyz_m": [-2507798.7984, -4676369.6918, 3526890.8008],
        },
    }
    for route in ROUTES:
        pin = base_inputs.get(route)
        if not isinstance(pin, dict):
            raise fail(f"Phase65 base member missing: {route}")
        for key, value in expected[route].items():
            assert_equal(pin.get(key), value, f"Phase65/{route}/{key}")
        _safe_sealed_path(pin.get("path"), route, "base.obs")
        if len(pin["sha256"]) != 64:
            raise fail(f"Phase65 base digest malformed: {route}")
        resolved[route] = dict(expected[route])
        resolved[route].update({
            "path_source": "Phase65 sealed base_inputs metadata",
            "coordinate_source": "RINEX header APPROX POSITION XYZ",
            "read_at_manifest_creation": False,
            "read_by_pre_raw_verifier": False,
            "hash_read_at_manifest_creation": False,
        })
    return resolved


def verify_freeze() -> dict[str, Any]:
    assert_equal(_sealed_hash_file(FREEZE, "Phase107 freeze"), FREEZE_SHA, "freeze sha256")
    freeze = read_json(FREEZE, "Phase107 freeze")
    for key, expected in {
        "schema_version": "smartphone-r5-phase107-raw-base-source-parity-freeze.v1",
        "phase": 107,
        "execution_label": "Luna Max",
        "status": "frozen-read-only-before-raw-base-phase101-execution",
    }.items():
        assert_equal(freeze.get(key), expected, f"freeze/{key}")
    decision = freeze.get("decision")
    if not isinstance(decision, dict):
        raise fail("freeze/decision missing")
    assert_equal(decision.get("candidate_id"), CANDIDATE_ID, "freeze/candidate id")
    assert_equal(decision.get("candidate_count"), 1, "freeze/candidate count")
    assert_equal(decision.get("current_phase101_plus_base_combination_executable"), False, "freeze/pre-implementation state")
    assert_equal(decision.get("only_frozen_candidate"), "guard-only admission of the existing raw-base model alongside Phase101 selectors; no equation, factor, state, solver, or parameter change", "freeze/candidate scope")
    auth = freeze.get("authorization_boundary")
    if not isinstance(auth, dict):
        raise fail("freeze/authorization boundary missing")
    assert_equal(auth.get("raw_base_execution_authorized"), False, "freeze/raw authorization")
    candidate = freeze.get("candidate_contract")
    if not isinstance(candidate, dict):
        raise fail("freeze/candidate contract missing")
    for key, expected in {
        "opt_in": True,
        "default_off": True,
        "base_rinex_header_approx_position_required": True,
        "exact_route_hash_and_byte_preflight_required": True,
        "exact_satellite_signal_in_domain_coverage_required": True,
        "gnss_first_and_main_use_same_corrected_code_factor_vector": True,
        "full_finite_exact_c7_d_handoff_required": True,
        "strict_gnss_first_and_main_progress_required": True,
        "main_qr_selection_required": True,
        "no_fallback": True,
        "no_solution_publication": True,
        "legacy_default_unchanged": True,
    }.items():
        assert_equal(candidate.get(key), expected, f"freeze/candidate/{key}")
    return freeze


def verify_implementation() -> None:
    for path, expected, label in (
        (APP, APP_SHA, "native app"),
        (BACKEND, BACKEND_SHA, "GTSAM backend"),
        (INTERNAL, INTERNAL_SHA, "key helpers"),
        (FGO_RESULT, FGO_RESULT_SHA, "FGO result"),
        (CONFIG, CONFIG_SHA, "FGO config"),
        (BASE_MODEL, BASE_MODEL_SHA, "base model"),
        (BASE_MODEL_HEADER, BASE_MODEL_HEADER_SHA, "base model header"),
        (RINEX, RINEX_SHA, "RINEX reader"),
        (BINARY, BINARY_SHA, "native binary"),
    ):
        assert_equal(_sealed_hash_file(path, label), expected, f"implementation/{label}")
    source = APP.read_text(encoding="utf-8")
    for marker in (
        "phase107_raw_base_source_parity_admission",
        "native_base_pseudorange_source_miss_mask",
        "base_pseudorange_compensation::subtractCorrection",
        "gnss_first_problem = problem",
        "native_source_clock_c0d_gnss_first_meter_state_handoff",
    ):
        if marker not in source:
            raise fail(f"Phase107 implementation marker missing: {marker}")
    if "use_native_source_clock_c0d_gnss_first_meter_state_handoff = false" not in CONFIG.read_text(encoding="utf-8"):
        raise fail("legacy meter-state selector is not explicitly default-off")


def output_path(route: str, name: str) -> str:
    return f"{OUTPUT_ROOT}{route.replace('/', '__')}/{name}"


def validate_command(route: str, command: Any, record: dict[str, Any], raw: dict[str, dict[str, Any]], base: dict[str, Any]) -> None:
    if not isinstance(command, list) or not command or not all(isinstance(item, str) for item in command):
        raise fail(f"malformed command: {route}")
    assert_equal(command[0], "build/apps/gnss_fgo_imu_no_base", f"command/{route}/binary")
    for token in command:
        lowered = token.lower()
        if any(term in lowered for term in FORBIDDEN_PATH_TERMS):
            raise fail(f"forbidden command token: {route}/{token}")
        if token in FORBIDDEN_FLAGS:
            raise fail(f"forbidden command flag: {route}/{token}")
    for flag in REQUIRED_FLAGS:
        assert_equal(command.count(flag), 1, f"command/{route}/{flag}")
    assert_equal(command.count("--dataset-id"), 1, f"command/{route}/dataset-id count")
    assert_equal(command[command.index("--dataset-id") + 1], route, f"command/{route}/dataset-id")
    for flag, name, placeholder in RAW_FLAGS:
        assert_equal(command.count(flag), 1, f"command/{route}/{flag} count")
        assert_equal(command[command.index(flag) + 1], placeholder, f"command/{route}/{name} placeholder")
        pin = record.get("raw_inputs", {}).get(name)
        if not isinstance(pin, dict):
            raise fail(f"manifest raw pin missing: {route}/{name}")
        assert_equal(pin.get("path"), raw[name]["path"], f"manifest/raw/{route}/{name}/path")
        assert_equal(pin.get("sha256"), raw[name]["sha256"], f"manifest/raw/{route}/{name}/sha256")
        assert_equal(pin.get("path_source"), "Phase95 sealed result raw_inputs.path", f"manifest/raw/{route}/{name}/source")
        assert_equal(pin.get("read_at_manifest_creation"), False, f"manifest/raw/{route}/{name}/read")
    assert_equal(command.count(BASE_FLAG[0]), 1, f"command/{route}/base path count")
    assert_equal(command[command.index(BASE_FLAG[0]) + 1], BASE_FLAG[1], f"command/{route}/base path placeholder")
    assert_equal(command.count(BASE_SHA_FLAG[0]), 1, f"command/{route}/base hash count")
    assert_equal(command[command.index(BASE_SHA_FLAG[0]) + 1], BASE_SHA_FLAG[1], f"command/{route}/base hash placeholder")
    base_pin = record.get("base_input")
    if not isinstance(base_pin, dict):
        raise fail(f"manifest base pin missing: {route}")
    for key in ("path", "sha256", "bytes", "observed_dt_s", "moving_mean_samples", "approx_position_xyz_m"):
        assert_equal(base_pin.get(key), base.get(key), f"manifest/base/{route}/{key}")
    for key in ("read_at_manifest_creation", "hash_read_at_manifest_creation"):
        assert_equal(base_pin.get(key), False, f"manifest/base/{route}/{key}")
    assert_equal(command.count("--out"), 1, f"command/{route}/out count")
    assert_equal(command[command.index("--out") + 1], output_path(route, "withheld_solution_output.csv"), f"command/{route}/out")
    assert_equal(command.count("--summary-json"), 1, f"command/{route}/summary count")
    assert_equal(command[command.index("--summary-json") + 1], output_path(route, "summary.json"), f"command/{route}/summary")
    planned = record.get("planned_output")
    if not isinstance(planned, dict):
        raise fail(f"planned output missing: {route}")
    assert_equal(planned.get("summary"), output_path(route, "summary.json"), f"manifest/{route}/summary")
    assert_equal(planned.get("withheld_solution_output"), output_path(route, "withheld_solution_output.csv"), f"manifest/{route}/withheld")
    assert_equal(record.get("runs"), 1, f"manifest/{route}/runs")
    assert_equal(record.get("diagnostic_only"), True, f"manifest/{route}/diagnostic-only")


def verify_manifest() -> dict[str, Any]:
    verify_freeze()
    verify_implementation()
    raw = phase95_raw_metadata()
    base = phase65_base_metadata()
    manifest = read_json(MANIFEST, "Phase107 structural manifest")
    for key, expected in {
        "schema_version": SCHEMA,
        "phase": 107,
        "execution_label": "Luna Max",
        "status": "sealed-before-phase107-raw-base-execution",
    }.items():
        assert_equal(manifest.get(key), expected, f"manifest/{key}")
    freeze = manifest.get("freeze")
    if not isinstance(freeze, dict):
        raise fail("manifest/freeze missing")
    for key, expected in {
        "path": relative(FREEZE), "sha256": FREEZE_SHA, "commit": FREEZE_COMMIT,
        "audit_path": relative(AUDIT), "audit_sha256": AUDIT_SHA, "audit_commit": AUDIT_COMMIT,
        "execution_authorized_before_this_manifest": False,
    }.items():
        assert_equal(freeze.get(key), expected, f"manifest/freeze/{key}")
    implementation = manifest.get("implementation")
    if not isinstance(implementation, dict):
        raise fail("manifest/implementation missing")
    for key, expected in {
        "commit": IMPLEMENTATION_COMMIT,
        "candidate_id": CANDIDATE_ID,
        "implementation_scope": "Phase107 guard-only admission of existing native raw-base correction with Phase101 C7/D and Phase99 QR handoff",
        "solver_filter_lm_changed": False,
        "equation_units_sigma_changed": False,
        "legacy_default_unchanged": True,
        "fallback_or_guard_bypass": False,
    }.items():
        assert_equal(implementation.get(key), expected, f"manifest/implementation/{key}")
    source_pins = {
        "source": (APP, APP_SHA), "backend": (BACKEND, BACKEND_SHA), "key_helpers": (INTERNAL, INTERNAL_SHA),
        "fgo_result": (FGO_RESULT, FGO_RESULT_SHA), "config": (CONFIG, CONFIG_SHA),
        "base_model": (BASE_MODEL, BASE_MODEL_SHA), "base_model_header": (BASE_MODEL_HEADER, BASE_MODEL_HEADER_SHA),
        "rinex_reader": (RINEX, RINEX_SHA), "binary": (BINARY, BINARY_SHA),
    }
    for key, (path, digest) in source_pins.items():
        pin = implementation.get(key)
        if not isinstance(pin, dict):
            raise fail(f"manifest/implementation/{key} missing")
        assert_equal(pin.get("path"), relative(path), f"manifest/implementation/{key}/path")
        assert_equal(pin.get("sha256"), digest, f"manifest/implementation/{key}/sha256")
    for key, path in (("pre_raw_evaluator", EVALUATOR), ("execution_wrapper", WRAPPER), ("focused_tests", FOCUSED_TESTS)):
        pin = manifest.get(key)
        if not isinstance(pin, dict):
            raise fail(f"manifest/{key} missing")
        assert_equal(pin.get("path"), relative(path), f"manifest/{key}/path")
        assert_equal(pin.get("sha256"), _sealed_hash_file(path, f"Phase107 {key}"), f"manifest/{key}/sha256")
    inherited = manifest.get("sealed_input_sources")
    if not isinstance(inherited, dict):
        raise fail("manifest/sealed_input_sources missing")
    for key, expected in {
        "phase95_result_path": relative(PHASE95_RESULT), "phase95_result_sha256": PHASE95_RESULT_SHA,
        "phase65_manifest_path": relative(PHASE65_MANIFEST), "phase65_manifest_sha256": PHASE65_MANIFEST_SHA,
        "raw_materialization": "Phase95 exact path metadata; metadata-only stat after authorization; no copy or transform",
        "base_materialization": "Phase65 exact base member metadata; wrapper hash preflight then one native RINEX process read",
        "pre_raw_raw_payload_reads": 0, "pre_raw_base_payload_reads": 0,
    }.items():
        assert_equal(inherited.get(key), expected, f"manifest/sealed_input_sources/{key}")
    candidate = manifest.get("candidate")
    if not isinstance(candidate, dict):
        raise fail("manifest/candidate missing")
    for key, expected in {
        "id": CANDIDATE_ID, "candidate_count": 1, "opt_in": True, "default_off_outside_this_command": True,
        "raw_base_only": True, "diagnostic_only": True, "runs_per_route": 1, "controls": 0,
        "truth_evaluation": False, "accuracy_scoring": False, "solution_output_publication": False,
        "no_global_isb_double_state": True, "no_solver_filter_lm_change": True,
        "no_equation_unit_sigma_change": True, "no_fallback_or_guard_bypass": True,
        "base_correction_applied_exactly_once": True, "base_correction_scope": "existing adopted raw code factors only",
        "base_coordinate_source": "raw base RINEX header APPROX POSITION XYZ only",
        "raw_d_initializer_source": "retained EpochSeed.receiver_clock_drift_mps",
        "clock_dimension": 7, "ccdd_sigma_m": 0.1, "clock_units": "metres",
        "drift_units": "metres_per_second", "main_solver": "MULTIFRONTAL_QR / EliminateQR",
    }.items():
        assert_equal(candidate.get(key), expected, f"manifest/candidate/{key}")
    assert_equal(candidate.get("selectors"), [PHASE93_SELECTOR, VECTOR_SELECTOR, QR_SELECTOR], "manifest/candidate/selectors")
    routes = manifest.get("routes")
    if not isinstance(routes, list) or [item.get("dataset_id") for item in routes] != list(ROUTES):
        raise fail("manifest route order/count changed")
    for record in routes:
        route = record.get("dataset_id")
        if route not in DOMAIN_ROWS:
            raise fail(f"unknown route: {route}")
        assert_equal(record.get("domain_rows"), DOMAIN_ROWS[route], f"manifest/{route}/domain rows")
        assert_equal(record.get("expected_problem_epochs"), DOMAIN_ROWS[route] + 1, f"manifest/{route}/problem epochs")
        validate_command(route, record.get("command"), record, raw[route], base[route])
    raw_contract = manifest.get("raw_input_contract")
    if not isinstance(raw_contract, dict):
        raise fail("manifest/raw_input_contract missing")
    for key, expected in {
        "input_names_exact": list(RAW_NAMES), "device_gnss_only": True, "device_imu_only": True,
        "broadcast_navigation_only": True, "base_rinex_allowed_only_as_sealed_member": True,
        "truth_mat_phone_coordinates_precomputed_pdc_kaggle_accuracy": False, "copy_or_transform": False,
    }.items():
        assert_equal(raw_contract.get(key), expected, f"manifest/raw_input_contract/{key}")
    base_contract = manifest.get("base_contract")
    if not isinstance(base_contract, dict):
        raise fail("manifest/base_contract missing")
    for key, expected in {
        "enabled": True, "source": "same native C++ process from exact raw base RINEX and broadcast navigation",
        "correction_formula": "P_rover_corrected_m=P_rover_raw_m-pc(t)",
        "stream_key": "exact satellite and signal", "in_domain_interpolation_only": True,
        "no_extrapolation_or_endpoint_hold": True, "source_miss_mask": True,
        "no_new_base_factor_family": True, "spp_applied": False, "doppler_applied": False,
        "tdcp_or_carrier_applied": False, "base_coordinate_input": "RINEX header APPROX POSITION XYZ",
        "wrapper_hash_reads_per_route": 1, "native_process_reads_per_route": 1,
    }.items():
        assert_equal(base_contract.get(key), expected, f"manifest/base_contract/{key}")
    matrix = manifest.get("matrix")
    if not isinstance(matrix, dict):
        raise fail("manifest/matrix missing")
    for key, expected in {
        "candidate_count": 1, "route_count": 2, "runs_per_route": 1, "native_invocations_planned": 2,
        "controls": 0, "reruns": 0, "fallbacks": 0, "base_rinex_process_reads_planned": 2,
        "base_rinex_hash_verification_reads_planned": 2, "truth_reads_planned": 0,
        "mat_reads_or_generated_planned": 0, "phone_coordinate_reads_planned": 0,
        "precomputed_coordinate_reads_planned": 0, "kaggle_or_token_access_planned": 0,
        "accuracy_calculations_planned": 0, "solution_rows_authorized": False,
    }.items():
        assert_equal(matrix.get(key), expected, f"manifest/matrix/{key}")
    gates = manifest.get("structural_gates")
    if not isinstance(gates, dict):
        raise fail("manifest/structural_gates missing")
    for key in (
        "base_factors_active_exactly_once", "base_path_hash_bytes_and_header_match",
        "base_exact_stream_miss_mask_telemetry", "gnss_first_accepted_iterations_and_strict_cost_decrease",
        "gnss_first_full_finite_c7_d_exact_handoff", "no_global_isb_double_state",
        "main_selected_multifrontal_qr", "main_accepted_iterations_and_strict_cost_decrease",
        "main_finite_earth_valid_expected_output_coverage", "no_fallback_or_solution_publication",
    ):
        assert_equal(gates.get(key), True, f"manifest/structural_gates/{key}")
    execution = manifest.get("execution_authorization")
    if not isinstance(execution, dict):
        raise fail("manifest/execution_authorization missing")
    for key, expected in {
        "before_raw_execution": True, "raw_execution_authorized": False,
        "native_route_rerun_performed": False, "accuracy_or_submission_release": False,
        "stop_after_two_routes": True,
    }.items():
        assert_equal(execution.get(key), expected, f"manifest/execution_authorization/{key}")
    accounting = manifest.get("read_accounting_before_execution")
    if not isinstance(accounting, dict):
        raise fail("manifest/read_accounting_before_execution missing")
    for key in (
        "native_solver_invocations", "raw_device_gnss_reads", "raw_device_imu_reads",
        "broadcast_navigation_reads", "truth_reads", "mat_reads_or_generated",
        "phone_coordinate_reads", "precomputed_coordinate_reads", "base_rinex_process_reads",
        "base_rinex_hash_verification_reads", "accuracy_calculations", "kaggle_or_token_access",
        "raw_input_hash_reads", "route_reruns", "fallbacks",
    ):
        assert_equal(accounting.get(key), 0, f"manifest/read_accounting/{key}")
    assert_equal(accounting.get("raw_content_copied_or_transformed"), False, "manifest/read_accounting/copy")
    return manifest


def verify_authorization(manifest: dict[str, Any] | None = None) -> dict[str, Any]:
    if manifest is None:
        manifest = verify_manifest()
    authorization = read_json(AUTHORIZATION, "Phase107 raw authorization")
    for key, expected in {
        "schema_version": AUTH_SCHEMA, "phase": 107, "execution_label": "Luna Max",
        "status": "authorized-for-exact-two-route-phase107-raw-base-structural-execution",
    }.items():
        assert_equal(authorization.get(key), expected, f"authorization/{key}")
    authority = authorization.get("authority")
    if not isinstance(authority, dict):
        raise fail("authorization/authority missing")
    for key, path, digest in (
        ("phase107_freeze", FREEZE, FREEZE_SHA), ("phase107_audit", AUDIT, AUDIT_SHA),
        ("phase107_manifest", MANIFEST, _sealed_hash_file(MANIFEST, "Phase107 manifest")),
        ("phase107_evaluator", EVALUATOR, _sealed_hash_file(EVALUATOR, "Phase107 evaluator")),
        ("phase107_wrapper", WRAPPER, _sealed_hash_file(WRAPPER, "Phase107 wrapper")),
        ("phase107_focused_tests", FOCUSED_TESTS, _sealed_hash_file(FOCUSED_TESTS, "Phase107 focused tests")),
        ("phase95_result", PHASE95_RESULT, PHASE95_RESULT_SHA),
        ("phase65_manifest", PHASE65_MANIFEST, PHASE65_MANIFEST_SHA),
    ):
        pin = authority.get(key)
        if not isinstance(pin, dict):
            raise fail(f"authorization/{key} missing")
        assert_equal(pin.get("path"), relative(path), f"authorization/{key}/path")
        assert_equal(pin.get("sha256"), digest, f"authorization/{key}/sha256")
    for key, expected in {
        "freeze_commit": FREEZE_COMMIT, "audit_commit": AUDIT_COMMIT,
        "implementation_commit": IMPLEMENTATION_COMMIT, "implementation_source_sha256": APP_SHA,
        "implementation_backend_sha256": BACKEND_SHA, "implementation_fgo_result_sha256": FGO_RESULT_SHA,
        "binary_sha256": BINARY_SHA, "manifest_sha256": _sealed_hash_file(MANIFEST, "Phase107 manifest"),
    }.items():
        assert_equal(authority.get(key), expected, f"authorization/authority/{key}")
    assert_equal(authorization.get("routes"), list(ROUTES), "authorization/routes")
    candidate = authorization.get("candidate")
    if not isinstance(candidate, dict):
        raise fail("authorization/candidate missing")
    for key, expected in {
        "candidate_count": 1, "id": CANDIDATE_ID,
        "selectors": [PHASE93_SELECTOR, VECTOR_SELECTOR, QR_SELECTOR],
        "raw_input_names_exact": list(RAW_NAMES), "base_rinex_member_required": True,
        "truth_mat_phone_coordinate_precomputed_pdc_kaggle_accuracy": False,
        "raw_content_copy_or_transform": False, "solution_output_publication": False,
        "runs_per_route": 1, "controls": 0, "reruns": 0, "fallbacks": 0,
        "no_global_isb_double_state": True, "no_fallback_or_guard_bypass": True,
        "base_correction_exactly_once": True,
    }.items():
        assert_equal(candidate.get(key), expected, f"authorization/candidate/{key}")
    matrix = authorization.get("matrix")
    if not isinstance(matrix, dict):
        raise fail("authorization/matrix missing")
    for key, expected in {
        "candidate_count": 1, "route_count": 2, "runs_per_route": 1, "native_invocations": 2,
        "raw_device_gnss_reads_max": 2, "raw_device_imu_reads_max": 2,
        "broadcast_navigation_reads_max": 2, "base_rinex_process_reads_max": 2,
        "base_rinex_hash_verification_reads_max": 2, "truth_reads": 0,
        "mat_reads_or_generated": 0, "phone_coordinate_reads": 0, "precomputed_coordinate_reads": 0,
        "accuracy_calculations": 0, "kaggle_or_token_access": 0, "solution_rows_authorized": False,
        "reruns": 0, "fallbacks": 0,
    }.items():
        assert_equal(matrix.get(key), expected, f"authorization/matrix/{key}")
    policy = authorization.get("execution_policy")
    if not isinstance(policy, dict):
        raise fail("authorization/execution_policy missing")
    for key, expected in {
        "authorized": True, "order": "MTV-A then LAX-T, sequentially",
        "one_invocation_per_route": True, "no_rerun": True, "stop_after_two_routes": True,
        "solver_filter_lm_changes": False, "fallback_or_guard_bypass": False,
        "truth_or_accuracy_evaluation": False,
        "solution_output": "CLI output path is isolated/withheld and must not be opened, published, or committed",
        "base_input": "exact sealed Phase65 base member; wrapper hashes once before native process and native reads once",
    }.items():
        assert_equal(policy.get(key), expected, f"authorization/policy/{key}")
    boundary = authorization.get("release_boundary")
    if not isinstance(boundary, dict):
        raise fail("authorization/release_boundary missing")
    for key, expected in {
        "raw_base_execution_authorized": True, "diagnostic_structural_result_authorized": True,
        "truth_or_accuracy_evaluation_authorized": False, "submission_release_authorized": False,
        "solution_rows_authorized": False,
    }.items():
        assert_equal(boundary.get(key), expected, f"authorization/release/{key}")
    assert_equal(manifest.get("execution_authorization", {}).get("raw_execution_authorized"), False, "manifest remains pre-auth")
    return authorization


def verify_pre_raw() -> dict[str, Any]:
    freeze = verify_freeze()
    manifest = verify_manifest()
    authorized = False
    auth_status = "not-issued-before-separate-raw-authorization"
    if AUTHORIZATION.is_file():
        authorization = verify_authorization(manifest)
        authorized = authorization.get("status") == "authorized-for-exact-two-route-phase107-raw-base-structural-execution"
        auth_status = "authorized" if authorized else "invalid"
    return {
        "status": "pre-raw-verified", "phase": 107, "execution_label": "Luna Max",
        "candidate_count": 1, "routes": len(ROUTES), "route_ids": list(ROUTES), "runs_per_route": 1,
        "raw_execution_authorized": authorized, "raw_reads": 0, "base_payload_reads": 0,
        "base_hash_verification_reads": 0, "native_solver_invocations": 0, "truth_reads": 0,
        "mat_reads_or_generated": 0, "phone_coordinate_reads": 0, "precomputed_coordinate_reads": 0,
        "accuracy_scored": False, "solution_output_published": False, "route_score_selection": False,
        "cholesky_comparison_rerun": False, "freeze_sha256": FREEZE_SHA,
        "manifest_sha256": _sealed_hash_file(MANIFEST, "Phase107 manifest"),
        "phase95_result_sha256": PHASE95_RESULT_SHA, "phase65_manifest_sha256": PHASE65_MANIFEST_SHA,
        "freeze_status": freeze["status"], "manifest_status": manifest["status"],
        "authorization_status": auth_status,
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
    except (Phase107ContractError, OSError) as exc:
        print(f"phase107 pre-raw verifier: fail-closed: {exc}", file=sys.stderr)
        raise SystemExit(2)
