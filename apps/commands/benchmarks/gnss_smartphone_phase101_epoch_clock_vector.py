#!/usr/bin/env python3
"""Launch-free Phase101 raw-only structural contract.

The contract pins the Phase101 source-parity implementation and the already
materialized Phase95 raw-input metadata.  It never opens, hashes, copies, or
transforms a raw GNSS, IMU, or navigation file and it never launches a native
solver.  A separate authorization record is required before the execution
wrapper may substitute raw paths and launch the two native runs.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import sys
from typing import Any


ROOT = Path(__file__).resolve().parents[3]
FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_phase101_source_parity_accuracy_regression_freeze_v1.json"
MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase101_epoch_clock_vector_structural_manifest_v1.json"
AUTHORIZATION = ROOT / "docs/use_cases/records/smartphone_r5_phase101_epoch_clock_vector_raw_execution_authorization_v1.json"
PHASE95_RESULT = ROOT / "docs/use_cases/records/smartphone_r5_phase95_raw_input_path_corrected_structural_result_v1.json"
PHASE95_WRAPPER = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase95_raw_input_path_corrected_execute.py"
PHASE91_MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase91_source_clock_c0d_gnss_first_raw_drift_d_initializer_execution_manifest_v1.json"
EVALUATOR = Path(__file__).resolve()
WRAPPER = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase101_epoch_clock_vector_execute.py"
FOCUSED_TESTS = ROOT / "tests/test_smartphone_phase101_epoch_clock_vector.py"

APP = ROOT / "apps/native/gnss_fgo_imu_no_base.cpp"
BACKEND = ROOT / "src/algorithms/fgo_gtsam_backend.cpp"
INTERNAL = ROOT / "src/algorithms/fgo_gtsam_internal.hpp"
FGO = ROOT / "include/libgnss++/algorithms/fgo.hpp"
CONFIG = ROOT / "include/libgnss++/algorithms/fgo_config.hpp"
BINARY = ROOT / "build/apps/gnss_fgo_imu_no_base"

FREEZE_SHA = "092a12fd6801ab42fcdbd9e8a0df4264ad2f3fbe6ea4ead1f5306dddcd7339ff"
FREEZE_COMMIT = "48aa18fe8311757383e9b7a383dd9b83f737e75c"
AUDIT_COMMIT = "0aea938ce3a8864e00bbc641e1b5135e4e860a43"
IMPLEMENTATION_COMMIT = "d88c9fde97a818c6279d8ae35f1a1a70e06a8ce4"
APP_SHA = "72269b057073c0b0134b069a7436c8c3979504807b94ec7b477b7c4fa6f99339"
BACKEND_SHA = "781d65e34826ec08e04d1dfdac0ab10b2ed9409726078465f3020b57afac4baa"
INTERNAL_SHA = "cc6327e36800afa8217e5a834260adc81c0a74a5aefbfc956bb4679c8d86105f"
FGO_SHA = "5a26994bda96c4bf50f436b1368bc1df5b192c38ef13a2a882ebd8e15bf5103d"
CONFIG_SHA = "3cc60abe514ef8900012064accb17f27d3927d0b8a8776ad76ce6aefa83ce32c"
BINARY_SHA = "45ab71ca416a5c9947a75f2c9e7bc996678b59c2b88ac52c04362d0ccf5dae77"
PHASE95_RESULT_SHA = "beba25d4b0d910064d76bea7aa6b7a884937ca01c3b00cf0f7f9f4b6267ea281"
PHASE95_WRAPPER_SHA = "5ac858cf156555652f6feae60c71a4237269d4b4c2cb70c1adf5a9e08f10e883"
PHASE91_MANIFEST_SHA = "2078a4e2c3b963f07744c435ec303ea848bc372c8efd5003f6cb7b3d4b4ce988"

ROUTES = (
    "2021-03-16-18-59-us-ca-mtv-a/pixel5",
    "2022-04-01-18-22-us-ca-lax-t/pixel5",
)
DOMAIN_ROWS = {ROUTES[0]: 2158, ROUTES[1]: 1465}
RAW_NAMES = ("device_gnss.csv", "device_imu.csv", "brdc.nav")
RAW_FLAGS = (
    ("--android-gnss", "device_gnss.csv", "__PHASE95_RAW_DEVICE_GNSS__"),
    ("--android-imu", "device_imu.csv", "__PHASE95_RAW_DEVICE_IMU__"),
    ("--nav", "brdc.nav", "__PHASE95_RAW_BROADCAST_NAV__"),
)
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
)
FORBIDDEN_FLAGS = (
    "--obs",
    "--native-gnss-first-velocity-only-handoff",
    "--native-direct-doppler-wls-handoff",
    "--native-pdc-state-bridge",
    "--native-upstream-quality",
    "--native-base-rinex",
    "--native-base-pseudorange-compensation",
    "--native-base-pseudorange-source-miss-mask",
    "--native-source-clock-c0d-gnss-first-raw-drift-d-initializer",
    "--native-source-clock-c0d-phase94-stage-diagnostics",
    "--native-source-clock-c0d-phase96-main-diagnostics",
    "--native-source-clock-c0d-phase97-singular-system-diagnostics",
    "--native-source-clock-c0d-phase98-solver-rank-diagnostic",
)
FORBIDDEN_PATH_TERMS = (
    ".mat", "ground_truth", "validation", "holdout", "kaggle", "token",
    "base.rinex", "coordinate", "truth", "precomputed",
)
SELECTOR = "--native-source-clock-c0d-epoch-vector-parity"
PHASE93_SELECTOR = "--native-source-clock-c0d-gnss-first-meter-state-handoff"
PHASE99_SELECTOR = "--native-source-clock-c0d-phase99-main-multifrontal-qr-solver"
OUTPUT_ROOT = "output/smartphone-r5/phase101-epoch-clock-vector-structural-v1/"
SCHEMA = "smartphone-r5-phase101-epoch-clock-vector-structural-manifest.v1"
AUTH_SCHEMA = "smartphone-r5-phase101-epoch-clock-vector-raw-execution-authorization.v1"
CANDIDATE_ID = "phase101-raw-only-native-source-parity-epoch-c-isb-v1"


class Phase101ContractError(ValueError):
    """Raised when the Phase101 contract fails closed."""


def fail(message: str) -> Phase101ContractError:
    return Phase101ContractError(message)


def relative(path: Path) -> str:
    try:
        return str(path.relative_to(ROOT))
    except ValueError:
        return str(path)


def sha256_file(path: Path, label: str) -> str:
    if path.name in RAW_NAMES:
        raise fail(f"raw-file hash is forbidden: {label}")
    if not path.is_file():
        raise fail(f"missing {label}: {path}")
    digest = hashlib.sha256()
    try:
        with path.open("rb") as handle:
            for chunk in iter(lambda: handle.read(1024 * 1024), b""):
                digest.update(chunk)
    except OSError as exc:
        raise fail(f"failed to hash {label}: {path}: {exc}") from exc
    return digest.hexdigest()


def read_json(path: Path, label: str) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise fail(f"failed to read {label}: {path}: {exc}") from exc
    if not isinstance(value, dict):
        raise fail(f"{label} is not an object")
    return value


def assert_equal(actual: Any, expected: Any, label: str) -> None:
    if actual != expected:
        raise fail(f"{label}: expected {expected!r}, got {actual!r}")


def _safe_raw_metadata(path_text: str, route: str, name: str) -> None:
    path = Path(path_text)
    if path.is_absolute() or ".." in path.parts or path.name != name:
        raise fail(f"unsafe sealed raw path: {route}/{name}: {path_text}")
    if any(term in path_text.lower() for term in FORBIDDEN_PATH_TERMS):
        raise fail(f"forbidden sealed raw path term: {route}/{name}")


def phase95_paths() -> dict[str, dict[str, dict[str, Any]]]:
    """Read sealed Phase95 path metadata; never open a raw input file."""

    assert_equal(
        sha256_file(PHASE95_RESULT, "Phase95 structural result"),
        PHASE95_RESULT_SHA,
        "Phase95 result/sha256",
    )
    result = read_json(PHASE95_RESULT, "Phase95 structural result")
    routes = result.get("routes")
    if not isinstance(routes, dict):
        raise fail("Phase95 result/routes is not an object")
    resolved: dict[str, dict[str, dict[str, Any]]] = {}
    for route in ROUTES:
        record = routes.get(route)
        if not isinstance(record, dict):
            raise fail(f"Phase95 result route missing: {route}")
        raw = record.get("raw_inputs")
        if not isinstance(raw, dict) or set(raw) != set(RAW_NAMES):
            raise fail(f"Phase95 raw role set changed: {route}")
        resolved[route] = {}
        for name in RAW_NAMES:
            pin = raw.get(name)
            if not isinstance(pin, dict) or not isinstance(pin.get("path"), str):
                raise fail(f"Phase95 raw path missing: {route}/{name}")
            _safe_raw_metadata(pin["path"], route, name)
            if pin.get("exists_before_launch") is not True:
                raise fail(f"Phase95 raw path was not present: {route}/{name}")
            if pin.get("read_by_runner") is not False:
                raise fail(f"Phase95 runner raw-byte policy changed: {route}/{name}")
            digest = pin.get("sha256")
            if not isinstance(digest, str) or len(digest) != 64:
                raise fail(f"Phase95 sealed SHA metadata missing: {route}/{name}")
            resolved[route][name] = {
                "path": pin["path"],
                "sha256": digest,
                "bytes": pin.get("bytes"),
                "manifest": pin.get("manifest"),
                "read_by_runner": False,
            }
    return resolved


def verify_freeze() -> dict[str, Any]:
    assert_equal(sha256_file(FREEZE, "Phase101 freeze"), FREEZE_SHA, "freeze/sha256")
    freeze = read_json(FREEZE, "Phase101 freeze")
    for key, expected in {
        "schema_version": "smartphone-r5-phase101-source-parity-accuracy-regression-freeze-v1",
        "phase": 101,
        "execution_label": "Luna Max",
        "status": "frozen-for-one-opt-in-implementation-no-execution",
    }.items():
        assert_equal(freeze.get(key), expected, f"freeze/{key}")
    scope = freeze.get("scope")
    if not isinstance(scope, dict):
        raise fail("freeze/scope missing")
    assert_equal(scope.get("source_only_audit"), True, "freeze/scope/source_only_audit")
    assert_equal(scope.get("routes_for_later_structural_qualification"), list(ROUTES), "freeze/scope/routes")
    assert_equal(scope.get("future_input_lane"), ["same-run raw device_gnss.csv", "same-run raw device_imu.csv", "same-run broadcast navigation"], "freeze/scope/raw lane")
    candidate = freeze.get("exactly_one_candidate")
    if not isinstance(candidate, dict):
        raise fail("freeze/exactly_one_candidate missing")
    assert_equal(candidate.get("candidate_id"), CANDIDATE_ID, "freeze/candidate/id")
    assert_equal(candidate.get("selector_off_contract"), "legacy/default behavior is unchanged bit-for-bit at the graph/config boundary", "freeze/candidate/default")
    state = freeze.get("state_contract")
    if not isinstance(state, dict):
        raise fail("freeze/state_contract missing")
    official = state.get("official")
    representation = state.get("candidate_representation")
    if not isinstance(official, dict) or not isinstance(representation, dict):
        raise fail("freeze/state contract incomplete")
    assert_equal(official.get("clock_key"), "c_i: Vector7 per retained epoch", "freeze/official/C")
    assert_equal(official.get("drift_key"), "d_i: Vector1 per retained epoch", "freeze/official/D")
    assert_equal(representation.get("clock_units"), "metres", "freeze/C units")
    assert_equal(representation.get("drift_units"), "metres/second", "freeze/D units")
    assert_equal(representation.get("initial_D"), "exact retained EpochSeed.receiver_clock_drift_mps; no raw/zero/WLS/interpolated fallback", "freeze/D init")
    equations = freeze.get("equation_and_factor_invariants")
    if not isinstance(equations, dict):
        raise fail("freeze/equation_and_factor_invariants missing")
    ccdd = equations.get("clock_ccdd")
    if not isinstance(ccdd, dict):
        raise fail("freeze/clock_ccdd missing")
    assert_equal(ccdd.get("residual"), "(C2-C1) - (D1+D2)*dt/2", "freeze/CCDD residual")
    assert_equal(ccdd.get("sigma"), "official ordinary clock sigma 0.1 m and existing source vector noise construction; no new or tuned sigma", "freeze/CCDD sigma")
    assert_equal(equations.get("solver"), "Keep the already selected Phase100 main MULTIFRONTAL_QR branch and GNSS-first Cholesky branch unchanged; this candidate is not a solver experiment.", "freeze/solver")
    handoff = freeze.get("handoff_contract")
    if not isinstance(handoff, dict):
        raise fail("freeze/handoff_contract missing")
    assert_equal(handoff.get("alignment_key"), "exact retained EpochSeed source index and UTC key; vector index is not accepted as a substitute for key equality", "freeze/handoff/alignment")
    assert_equal(handoff.get("output_policy"), "A later diagnostic or structural run must not publish solution rows; accuracy requires a new isolated evaluator contract.", "freeze/handoff/output")
    return freeze


def verify_implementation() -> None:
    for path, expected, label in (
        (APP, APP_SHA, "native app"),
        (BACKEND, BACKEND_SHA, "GTSAM backend"),
        (INTERNAL, INTERNAL_SHA, "key helpers"),
        (FGO, FGO_SHA, "FGO result"),
        (CONFIG, CONFIG_SHA, "FGO config"),
        (BINARY, BINARY_SHA, "native binary"),
        (PHASE95_WRAPPER, PHASE95_WRAPPER_SHA, "Phase95 path wrapper"),
        (PHASE91_MANIFEST, PHASE91_MANIFEST_SHA, "Phase91 manifest"),
    ):
        assert_equal(sha256_file(path, label), expected, f"implementation/{label}")
    source = APP.read_text(encoding="utf-8")
    backend = BACKEND.read_text(encoding="utf-8")
    config = CONFIG.read_text(encoding="utf-8")
    fgo = FGO.read_text(encoding="utf-8")
    for marker in (
        SELECTOR,
        "epoch_clock_bias_components_m",
        "native_source_clock_c0d_epoch_vector_dimension",
        "native_source_clock_c0d_epoch_vector_handoff_count",
        "MULTIFRONTAL_QR",
        "EliminateQR",
        "PseudorangeFactorSourceClock",
        "ClockFactor_CCDD",
        "global_isb_state_count",
    ):
        if marker not in source and marker not in backend and marker not in fgo:
            raise fail(f"Phase101 implementation marker missing: {marker}")
    if "use_native_source_clock_c0d_epoch_vector_parity = false" not in config:
        raise fail("Phase101 epoch-vector selector is not explicitly default-off")
    if "use_native_source_clock_c0d_phase99_main_multifrontal_qr_solver =" not in config:
        raise fail("Phase99 QR selector is not present in pinned config")
    if "native_source_clock_c0d_global_isb_state_count" not in backend:
        raise fail("global ISB accounting marker missing")


def output_path(route: str, name: str) -> str:
    return f"{OUTPUT_ROOT}{route.replace('/', '__')}/{name}"


def validate_command(route: str, command: Any, record: dict[str, Any], paths: dict[str, dict[str, Any]]) -> None:
    if not isinstance(command, list) or not command or not all(isinstance(token, str) for token in command):
        raise fail(f"malformed command: {route}")
    if command[0] != "build/apps/gnss_fgo_imu_no_base":
        raise fail(f"command/{route}/binary is not the pinned target")
    for token in command:
        lowered = token.lower()
        if any(term in lowered for term in FORBIDDEN_PATH_TERMS):
            raise fail(f"forbidden command path term: {route}/{token}")
        if token in FORBIDDEN_FLAGS:
            raise fail(f"forbidden command flag: {route}/{token}")
    for flag in REQUIRED_FLAGS:
        assert_equal(command.count(flag), 1, f"command/{route}/{flag}")
    assert_equal(command.count("--dataset-id"), 1, f"command/{route}/dataset-id-count")
    assert_equal(command[command.index("--dataset-id") + 1], route, f"command/{route}/dataset-id")
    if not isinstance(record.get("raw_inputs"), dict) or set(record["raw_inputs"]) != set(RAW_NAMES):
        raise fail(f"manifest raw-input set changed: {route}")
    for flag, name, placeholder in RAW_FLAGS:
        assert_equal(command.count(flag), 1, f"command/{route}/{flag}")
        assert_equal(command[command.index(flag) + 1], placeholder, f"command/{route}/{name}/placeholder")
        pin = record["raw_inputs"].get(name)
        if not isinstance(pin, dict):
            raise fail(f"manifest raw pin malformed: {route}/{name}")
        assert_equal(pin.get("path"), paths[name]["path"], f"manifest/raw/{route}/{name}/path")
        assert_equal(pin.get("path_source"), "Phase95 sealed result raw_inputs.path", f"manifest/raw/{route}/{name}/source")
        assert_equal(pin.get("sha256"), paths[name]["sha256"], f"manifest/raw/{route}/{name}/sha256")
        assert_equal(pin.get("read_at_manifest_creation"), False, f"manifest/raw/{route}/{name}/read")
    assert_equal(command.count("--out"), 1, f"command/{route}/out-count")
    assert_equal(command[command.index("--out") + 1], output_path(route, "withheld_solution_output.csv"), f"command/{route}/out")
    assert_equal(command.count("--summary-json"), 1, f"command/{route}/summary-count")
    assert_equal(command[command.index("--summary-json") + 1], output_path(route, "summary.json"), f"command/{route}/summary")
    planned = record.get("planned_output")
    if not isinstance(planned, dict):
        raise fail(f"planned output missing: {route}")
    assert_equal(planned.get("summary"), output_path(route, "summary.json"), f"manifest/output/{route}/summary")
    assert_equal(planned.get("withheld_solution_output"), output_path(route, "withheld_solution_output.csv"), f"manifest/output/{route}/withheld")
    assert_equal(record.get("runs"), 1, f"manifest/route/{route}/runs")
    assert_equal(record.get("diagnostic_only"), True, f"manifest/route/{route}/diagnostic_only")


def verify_manifest() -> dict[str, Any]:
    verify_freeze()
    verify_implementation()
    paths = phase95_paths()
    manifest = read_json(MANIFEST, "Phase101 structural manifest")
    for key, expected in {
        "schema_version": SCHEMA,
        "phase": 101,
        "execution_label": "Luna Max",
        "status": "sealed-before-phase101-epoch-clock-vector-raw-execution",
    }.items():
        assert_equal(manifest.get(key), expected, f"manifest/{key}")
    freeze_ref = manifest.get("freeze")
    if not isinstance(freeze_ref, dict):
        raise fail("manifest/freeze missing")
    for key, expected in {
        "path": relative(FREEZE),
        "sha256": FREEZE_SHA,
        "commit": FREEZE_COMMIT,
        "audit_commit": AUDIT_COMMIT,
        "execution_authorized_before_this_manifest": False,
    }.items():
        assert_equal(freeze_ref.get(key), expected, f"manifest/freeze/{key}")
    implementation = manifest.get("implementation")
    if not isinstance(implementation, dict):
        raise fail("manifest/implementation missing")
    for key, expected in {
        "commit": IMPLEMENTATION_COMMIT,
        "candidate_id": CANDIDATE_ID,
        "implementation_scope": "opt-in official epoch-local seven-vector C/ISB and same-run C/D handoff; Phase99 QR selector already pinned",
        "solver_filter_lm_changed": False,
        "equation_units_sigma_changed": False,
        "legacy_default_unchanged": True,
        "fallback_or_guard_bypass": False,
    }.items():
        assert_equal(implementation.get(key), expected, f"manifest/implementation/{key}")
    for key, path, digest in (
        ("source", APP, APP_SHA),
        ("backend", BACKEND, BACKEND_SHA),
        ("key_helpers", INTERNAL, INTERNAL_SHA),
        ("fgo_result", FGO, FGO_SHA),
        ("config", CONFIG, CONFIG_SHA),
        ("binary", BINARY, BINARY_SHA),
    ):
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
        assert_equal(pin.get("sha256"), sha256_file(path, f"Phase101 {key}"), f"manifest/{key}/sha256")
    inherited = manifest.get("phase95_path_source")
    if not isinstance(inherited, dict):
        raise fail("manifest/phase95_path_source missing")
    for key, expected in {
        "result_path": relative(PHASE95_RESULT),
        "result_sha256": PHASE95_RESULT_SHA,
        "wrapper_path": relative(PHASE95_WRAPPER),
        "wrapper_sha256": PHASE95_WRAPPER_SHA,
        "phase91_manifest_path": relative(PHASE91_MANIFEST),
        "phase91_manifest_sha256": PHASE91_MANIFEST_SHA,
        "materialization": "Phase95 sealed raw_inputs.path metadata followed by metadata-only stat and exact three-role substitution",
        "raw_byte_reads_by_pre_raw_verifier": 0,
        "raw_content_copy_or_transform": False,
    }.items():
        assert_equal(inherited.get(key), expected, f"manifest/phase95_path_source/{key}")
    candidate = manifest.get("candidate")
    if not isinstance(candidate, dict):
        raise fail("manifest/candidate missing")
    for key, expected in {
        "id": CANDIDATE_ID,
        "source_parity_selector": SELECTOR,
        "phase93_handoff_selector": PHASE93_SELECTOR,
        "phase99_qr_selector": PHASE99_SELECTOR,
        "candidate_count": 1,
        "opt_in": True,
        "default_off_outside_this_command": True,
        "raw_only": True,
        "diagnostic_only": True,
        "runs_per_route": 1,
        "controls": 0,
        "truth_evaluation": False,
        "accuracy_scoring": False,
        "solution_output_publication": False,
        "no_global_isb_double_state": True,
        "no_solver_filter_lm_change": True,
        "no_equation_unit_sigma_change": True,
        "no_fallback_or_guard_bypass": True,
        "raw_content_copy_or_transform": False,
        "cholesky_comparison_rerun": False,
    }.items():
        assert_equal(candidate.get(key), expected, f"manifest/candidate/{key}")
    state = candidate.get("state_and_handoff")
    if not isinstance(state, dict):
        raise fail("manifest/candidate/state_and_handoff missing")
    for key, expected in {
        "clock_dimension": 7,
        "clock_component_order": ["base_gps_l1", "glo_l1", "gal_l1", "bds_l1", "gps_l5", "gal_l5", "bds_l5"],
        "clock_units": "metres",
        "drift_units": "metres_per_second",
        "drift_initializer_source": "retained EpochSeed.receiver_clock_drift_mps",
        "drift_initializer_fallback": "none",
        "ccdd_sigma_m": 0.1,
        "ccdd_dt_units": "seconds",
        "exact_retained_key_alignment": True,
        "full_finite_c_and_d_handoff": True,
        "global_isb_state_count": 0,
        "same_run_position_velocity_handoff": True,
    }.items():
        assert_equal(state.get(key), expected, f"manifest/state_and_handoff/{key}")
    raw_contract = manifest.get("raw_input_contract")
    if not isinstance(raw_contract, dict):
        raise fail("manifest/raw_input_contract missing")
    for key, expected in {
        "input_names_exact": list(RAW_NAMES),
        "device_gnss_only": True,
        "device_imu_only": True,
        "broadcast_navigation_only": True,
        "truth_mat_base_precomputed_coordinate_kaggle_accuracy": False,
        "copy_or_transform": False,
    }.items():
        assert_equal(raw_contract.get(key), expected, f"manifest/raw_input_contract/{key}")
    routes = manifest.get("routes")
    if not isinstance(routes, list) or [item.get("dataset_id") for item in routes] != list(ROUTES):
        raise fail("manifest route order/count changed")
    for record in routes:
        route = record.get("dataset_id")
        if route not in DOMAIN_ROWS:
            raise fail(f"unknown route: {route}")
        assert_equal(record.get("domain_rows"), DOMAIN_ROWS[route], f"manifest/route/{route}/domain_rows")
        assert_equal(record.get("expected_problem_epochs"), DOMAIN_ROWS[route] + 1, f"manifest/route/{route}/problem_epochs")
        validate_command(route, record.get("command"), record, paths[route])
    matrix = manifest.get("matrix")
    if not isinstance(matrix, dict):
        raise fail("manifest/matrix missing")
    for key, expected in {
        "candidate_count": 1,
        "route_count": 2,
        "runs_per_route": 1,
        "native_invocations_planned": 2,
        "controls": 0,
        "reruns": 0,
        "fallbacks": 0,
        "raw_device_gnss_route_arguments": 2,
        "raw_device_imu_route_arguments": 2,
        "broadcast_navigation_route_arguments": 2,
        "truth_reads_planned": 0,
        "mat_reads_or_generated_planned": 0,
        "base_rinex_reads_planned": 0,
        "precomputed_coordinate_reads_planned": 0,
        "kaggle_or_token_access_planned": 0,
        "accuracy_calculations_planned": 0,
        "solution_rows_authorized": False,
        "cholesky_comparison_rerun_planned": False,
    }.items():
        assert_equal(matrix.get(key), expected, f"manifest/matrix/{key}")
    gates = manifest.get("structural_gates")
    if not isinstance(gates, dict):
        raise fail("manifest/structural_gates missing")
    for key in (
        "gnss_first_accepted_iterations_and_strict_cost_decrease",
        "gnss_first_full_finite_c7_d_exact_handoff",
        "no_global_isb_double_state",
        "main_selected_multifrontal_qr",
        "main_accepted_iterations_and_strict_cost_decrease",
        "main_finite_earth_valid_expected_output_coverage",
        "no_fallback_or_solution_publication",
    ):
        assert_equal(gates.get(key), True, f"manifest/structural_gates/{key}")
    qualification = manifest.get("qualification_evidence")
    if not isinstance(qualification, dict):
        raise fail("manifest/qualification_evidence missing")
    for key, expected in {"tests_run": 1118, "passed": 1060, "skipped": 58, "failed": 0, "exit_code": 0, "raw_route_execution": False}.items():
        assert_equal(qualification.get("full_cpp_suite", {}).get(key), expected, f"qualification/full_cpp/{key}")
    for key, expected in {"tests_run": 7, "passed": 7, "failed": 0, "exit_code": 0, "raw_route_execution": False}.items():
        assert_equal(qualification.get("focused_python", {}).get(key), expected, f"qualification/focused_python/{key}")
    assert_equal(qualification.get("target_build", {}).get("exit_code"), 0, "qualification/target_build/exit_code")
    assert_equal(qualification.get("diff_check_clean"), True, "qualification/diff_check_clean")
    execution = manifest.get("execution_authorization")
    if not isinstance(execution, dict):
        raise fail("manifest/execution_authorization missing")
    for key, expected in {"before_raw_execution": True, "raw_execution_authorized": False, "native_route_rerun_performed": False, "accuracy_or_submission_release": False, "stop_after_two_routes": True}.items():
        assert_equal(execution.get(key), expected, f"manifest/execution_authorization/{key}")
    accounting = manifest.get("read_accounting_before_execution")
    if not isinstance(accounting, dict):
        raise fail("manifest/read_accounting_before_execution missing")
    for key in ("native_solver_invocations", "raw_device_gnss_reads", "raw_device_imu_reads", "broadcast_navigation_reads", "truth_reads", "mat_reads_or_generated", "precomputed_coordinate_reads", "base_rinex_reads", "accuracy_calculations", "kaggle_or_token_access", "raw_input_hash_reads", "route_reruns", "fallbacks"):
        assert_equal(accounting.get(key), 0, f"manifest/read_accounting/{key}")
    assert_equal(accounting.get("raw_content_copied_or_transformed"), False, "manifest/read_accounting/copy")
    return manifest


def verify_authorization(manifest: dict[str, Any]) -> dict[str, Any]:
    authorization = read_json(AUTHORIZATION, "Phase101 raw execution authorization")
    for key, expected in {
        "schema_version": AUTH_SCHEMA,
        "phase": 101,
        "execution_label": "Luna Max",
        "status": "authorized-for-exact-two-route-phase101-epoch-clock-vector-structural-execution",
    }.items():
        assert_equal(authorization.get(key), expected, f"authorization/{key}")
    authority = authorization.get("authority")
    if not isinstance(authority, dict):
        raise fail("authorization/authority missing")
    for key, path, digest in (
        ("phase101_freeze", FREEZE, FREEZE_SHA),
        ("phase101_manifest", MANIFEST, sha256_file(MANIFEST, "Phase101 manifest")),
        ("phase101_evaluator", EVALUATOR, sha256_file(EVALUATOR, "Phase101 evaluator")),
        ("phase101_wrapper", WRAPPER, sha256_file(WRAPPER, "Phase101 wrapper")),
        ("phase101_focused_tests", FOCUSED_TESTS, sha256_file(FOCUSED_TESTS, "Phase101 focused tests")),
        ("phase95_result", PHASE95_RESULT, PHASE95_RESULT_SHA),
        ("phase95_path_wrapper", PHASE95_WRAPPER, PHASE95_WRAPPER_SHA),
        ("phase91_manifest", PHASE91_MANIFEST, PHASE91_MANIFEST_SHA),
    ):
        pin = authority.get(key)
        if not isinstance(pin, dict):
            raise fail(f"authorization/{key} missing")
        assert_equal(pin.get("path"), relative(path), f"authorization/{key}/path")
        assert_equal(pin.get("sha256"), digest, f"authorization/{key}/sha256")
    for key, expected in {
        "freeze_commit": FREEZE_COMMIT,
        "implementation_commit": IMPLEMENTATION_COMMIT,
        "implementation_source_sha256": APP_SHA,
        "implementation_backend_sha256": BACKEND_SHA,
        "implementation_fgo_result_sha256": FGO_SHA,
        "binary_sha256": BINARY_SHA,
        "manifest_sha256": sha256_file(MANIFEST, "Phase101 final manifest"),
    }.items():
        assert_equal(authority.get(key), expected, f"authorization/authority/{key}")
    assert_equal(authorization.get("routes"), list(ROUTES), "authorization/routes")
    candidate = authorization.get("candidate")
    if not isinstance(candidate, dict):
        raise fail("authorization/candidate missing")
    for key, expected in {
        "candidate_count": 1,
        "id": CANDIDATE_ID,
        "selectors": [PHASE93_SELECTOR, SELECTOR, PHASE99_SELECTOR],
        "raw_input_names_exact": list(RAW_NAMES),
        "truth_mat_base_coordinate_kaggle_accuracy": False,
        "raw_content_copy_or_transform": False,
        "solution_output_publication": False,
        "runs_per_route": 1,
        "controls": 0,
        "no_global_isb_double_state": True,
        "no_fallback_or_guard_bypass": True,
        "cholesky_comparison_rerun": False,
    }.items():
        assert_equal(candidate.get(key), expected, f"authorization/candidate/{key}")
    matrix = authorization.get("matrix")
    if not isinstance(matrix, dict):
        raise fail("authorization/matrix missing")
    for key, expected in {
        "candidate_count": 1,
        "route_count": 2,
        "runs_per_route": 1,
        "native_invocations": 2,
        "raw_device_gnss_reads_max": 2,
        "raw_device_imu_reads_max": 2,
        "broadcast_navigation_reads_max": 2,
        "truth_reads": 0,
        "mat_reads_or_generated": 0,
        "base_rinex_reads": 0,
        "precomputed_coordinate_reads": 0,
        "accuracy_calculations": 0,
        "kaggle_or_token_access": 0,
        "solution_rows_authorized": False,
        "reruns": 0,
        "fallbacks": 0,
    }.items():
        assert_equal(matrix.get(key), expected, f"authorization/matrix/{key}")
    policy = authorization.get("execution_policy")
    if not isinstance(policy, dict):
        raise fail("authorization/execution_policy missing")
    for key, expected in {
        "authorized": True,
        "order": "MTV-A then LAX-T, sequentially",
        "one_invocation_per_route": True,
        "no_rerun": True,
        "stop_after_two_routes": True,
        "solver_filter_lm_changes": False,
        "fallback_or_guard_bypass": False,
        "truth_or_accuracy_evaluation": False,
        "solution_output": "CLI output path is isolated/withheld and must not be opened, published, or committed",
    }.items():
        assert_equal(policy.get(key), expected, f"authorization/policy/{key}")
    boundary = authorization.get("release_boundary")
    if not isinstance(boundary, dict):
        raise fail("authorization/release_boundary missing")
    for key, expected in {"raw_execution_authorized": True, "diagnostic_structural_result_authorized": True, "truth_or_accuracy_evaluation_authorized": False, "submission_release_authorized": False, "solution_rows_authorized": False}.items():
        assert_equal(boundary.get(key), expected, f"authorization/release/{key}")
    assert_equal(manifest.get("execution_authorization", {}).get("raw_execution_authorized"), False, "manifest remains pre-auth")
    return authorization


def verify_pre_raw() -> dict[str, Any]:
    freeze = verify_freeze()
    manifest = verify_manifest()
    source = EVALUATOR.read_text(encoding="utf-8")
    for forbidden in ("sub" + "process", "P" + "open", "os." + "system", "os." + "popen"):
        if forbidden in source:
            raise fail(f"pre-raw evaluator contains launch token: {forbidden}")
    authorized = False
    auth_status = "not-issued-before-separate-raw-authorization"
    if AUTHORIZATION.is_file():
        authorization = verify_authorization(manifest)
        authorized = authorization.get("status") == "authorized-for-exact-two-route-phase101-epoch-clock-vector-structural-execution"
        auth_status = "authorized" if authorized else "invalid"
    return {
        "status": "pre-raw-verified",
        "phase": 101,
        "execution_label": "Luna Max",
        "candidate_count": 1,
        "routes": len(ROUTES),
        "route_ids": list(ROUTES),
        "runs_per_route": 1,
        "raw_execution_authorized": authorized,
        "raw_reads": 0,
        "native_solver_invocations": 0,
        "truth_reads": 0,
        "mat_reads_or_generated": 0,
        "precomputed_coordinate_reads": 0,
        "accuracy_scored": False,
        "solution_output_published": False,
        "route_score_selection": False,
        "cholesky_comparison_rerun": False,
        "freeze_sha256": FREEZE_SHA,
        "manifest_sha256": sha256_file(MANIFEST, "Phase101 manifest"),
        "phase95_result_sha256": PHASE95_RESULT_SHA,
        "phase95_wrapper_sha256": PHASE95_WRAPPER_SHA,
        "freeze_status": freeze["status"],
        "manifest_status": manifest["status"],
        "authorization_status": auth_status,
    }


if __name__ == "__main__":
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
    except (Phase101ContractError, OSError) as exc:
        print(f"phase101 pre-raw verifier: fail-closed: {exc}", file=sys.stderr)
        raise SystemExit(2)
