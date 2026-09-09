#!/usr/bin/env python3
"""Launch-free contract for the Phase113 remaining-route diagnostic lane.

The Phase113 freeze admits one read-only/default-off observation boundary for
MTV-H and MTV-U.  This module verifies the freeze, the already sealed raw
phone/base provenance, and the exact native command recipe.  It does not stat,
open, hash, or otherwise inspect any input payload.  The separate execution
wrapper is the only component allowed to materialize those paths after the
independent authorization record is present.
"""

from __future__ import annotations

import hashlib
import json
from pathlib import Path
import sys
from typing import Any


ROOT = Path(__file__).resolve().parents[3]
FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_phase113_remaining_routes_freeze_v1.json"
MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase113_remaining_routes_diagnostic_manifest_v1.json"
AUTHORIZATION = ROOT / "docs/use_cases/records/smartphone_r5_phase113_remaining_routes_diagnostic_authorization_v1.json"
EVALUATOR = Path(__file__).resolve()
WRAPPER = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase113_remaining_routes_diagnostic_execute.py"
FOCUSED_TESTS = ROOT / "tests/test_smartphone_phase113_remaining_routes_diagnostic.py"

PHASE95_RESULT = ROOT / "docs/use_cases/records/smartphone_r5_phase95_raw_input_path_corrected_structural_result_v1.json"
PHASE107_FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_phase107_raw_base_source_parity_freeze_v1.json"
PHASE65_MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase65_native_base_pseudorange_compensation_manifest_v1.json"

APP = ROOT / "apps/native/gnss_fgo_imu_no_base.cpp"
FGO = ROOT / "include/libgnss++/algorithms/fgo.hpp"
BACKEND = ROOT / "src/algorithms/fgo_gtsam_backend.cpp"
INTERNAL = ROOT / "src/algorithms/fgo_gtsam_internal.hpp"
PROBLEMS = ROOT / "src/algorithms/fgo_problems.cpp"
BASE_MODEL = ROOT / "src/algorithms/base_pseudorange_compensation.cpp"
BINARY = ROOT / "build/apps/gnss_fgo_imu_no_base"

FREEZE_SHA256 = "da9bc9e817ea575ec0775baf375d4df34ed87d3ccdc0c82f2fcf0a32893a0944"
FREEZE_COMMIT = "5da353e00e4eeb7c3508a9d077237469b5891b56"
AUDIT_COMMIT = "93702517c4fc38ffc8df190ecff7d6b63b688700"
PHASE95_RESULT_SHA256 = "beba25d4b0d910064d76bea7aa6b7a884937ca01c3b00cf0f7f9f4b6267ea281"
PHASE107_FREEZE_SHA256 = "8c05c89dda9cb3356b4c9bdf1c0ec1f2f41145bb37a7b5aee7ff12978aba9d76"
PHASE65_MANIFEST_SHA256 = "1306480f6b7fb839e55309e9c2c77b41f2ac0a1baf9f6971949612fd485d2255"
BINARY_SHA256 = "95a93efe37f4f043caf9920f324ea128c1872d0f161e0562da724d47c1483fb4"

ROUTES = (
    "2021-08-24-20-32-us-ca-mtv-h/pixel5",
    "2023-03-08-21-34-us-ca-mtv-u/pixel5",
)
DOMAIN_ROWS = {ROUTES[0]: 3139, ROUTES[1]: 1101}
RAW_NAMES = ("device_gnss.csv", "device_imu.csv", "brdc.nav")
RAW_FLAGS = (
    ("--android-gnss", "device_gnss.csv"),
    ("--android-imu", "device_imu.csv"),
    ("--nav", "brdc.nav"),
)
PHASE101_SELECTOR = "--native-source-clock-c0d-gnss-first-meter-state-handoff"
VECTOR_SELECTOR = "--native-source-clock-c0d-epoch-vector-parity"
QR_SELECTOR = "--native-source-clock-c0d-phase99-main-multifrontal-qr-solver"
BASE_SELECTOR = "--native-base-pseudorange-compensation"
BASE_MISS_SELECTOR = "--native-base-pseudorange-source-miss-mask"
OUTPUT_ROOT = "output/smartphone-r5/phase113-remaining-routes-diagnostic-v1/"
SCHEMA = "smartphone-r5-phase113-remaining-routes-diagnostic-manifest.v1"
AUTH_SCHEMA = "smartphone-r5-phase113-remaining-routes-diagnostic-authorization.v1"
CANDIDATE_ID = "phase113-remaining-routes-structural-admission-incidence-diagnostic-v1"

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
    PHASE101_SELECTOR,
    VECTOR_SELECTOR,
    QR_SELECTOR,
    BASE_SELECTOR,
    BASE_MISS_SELECTOR,
)
FORBIDDEN_FLAGS = (
    "--obs",
    "--native-gnss-first-velocity-only-handoff",
    "--native-direct-doppler-wls-handoff",
    "--native-pdc-state-bridge",
    "--native-upstream-quality",
    "--native-source-clock-c0d-gnss-first-raw-drift-d-initializer",
    "--native-source-clock-c0d-phase94-stage-diagnostics",
    "--native-source-clock-c0d-phase96-main-diagnostics",
    "--native-source-clock-c0d-phase97-singular-system-diagnostics",
    "--native-source-clock-c0d-phase98-solver-rank-diagnostic",
    "--native-phase104-stage-main-attribution",
    "--native-upstream-position-offset",
    "--native-quality-anchor",
    "--native-base-pseudorange-preserve-additional-frequency-bands",
)
FORBIDDEN_PATH_TERMS = (
    ".mat", "truth", "ground_truth", "validation", "holdout", "precomputed",
    "coordinate", "pdc", "kaggle", "token",
)


class Phase113ContractError(ValueError):
    """Raised when the Phase113 contract fails closed."""


def fail(message: str) -> Phase113ContractError:
    return Phase113ContractError(message)


def relative(path: Path) -> str:
    try:
        return str(path.relative_to(ROOT))
    except ValueError:
        return str(path)


def assert_equal(actual: Any, expected: Any, label: str) -> None:
    if actual != expected:
        raise fail(f"{label}: expected {expected!r}, got {actual!r}")


def sha256_file(path: Path, label: str) -> str:
    # This function is used only for source/contract artifacts.  Explicitly
    # reject input-member names so a future pre-raw check cannot accidentally
    # turn into a payload read.
    if path.name in RAW_NAMES or path.name == "base.obs":
        raise fail(f"input payload hash forbidden before authorization: {label}")
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


def safe_member_path(path_text: Any, basename: str, label: str) -> None:
    if not isinstance(path_text, str):
        raise fail(f"{label} path missing")
    path = Path(path_text)
    if path.is_absolute() or ".." in path.parts or path.name != basename:
        raise fail(f"unsafe {label} path: {path_text}")


def verify_freeze() -> dict[str, Any]:
    assert_equal(sha256_file(FREEZE, "Phase113 freeze"), FREEZE_SHA256, "freeze/sha256")
    freeze = read_json(FREEZE, "Phase113 freeze")
    for key, expected in {
        "schema_version": "smartphone-r5-phase113-remaining-routes-freeze.v1",
        "phase": 113,
        "execution_label": "Luna Max",
        "status": "frozen-read-only-diagnostic-no-algorithm-admission",
    }.items():
        assert_equal(freeze.get(key), expected, f"freeze/{key}")
    audit = freeze.get("audit")
    if not isinstance(audit, dict):
        raise fail("freeze/audit missing")
    assert_equal(audit.get("commit"), AUDIT_COMMIT, "freeze/audit/commit")
    assert_equal(audit.get("sha256"), "cdc9db5d3c4004c50a1e6dc9f186266a675f8af05d23c73db8e2d0f1b4fa6b4f", "freeze/audit/sha256")
    scope = freeze.get("scope")
    if not isinstance(scope, dict):
        raise fail("freeze/scope missing")
    assert_equal(scope.get("routes"), list(ROUTES), "freeze/scope/routes")
    assert_equal(scope.get("new_raw_execution"), False, "freeze/scope/new_raw_execution")
    assert_equal(scope.get("new_truth_evaluation"), False, "freeze/scope/new_truth_evaluation")
    candidate = freeze.get("candidate")
    if not isinstance(candidate, dict):
        raise fail("freeze/candidate missing")
    for key, expected in {
        "candidate_count": 1,
        "algorithmic_candidate_count": 0,
        "diagnostic_candidate_count": 1,
        "id": CANDIDATE_ID,
        "type": "opt-in-read-only-structural-diagnostic",
        "status": "frozen-default-off",
    }.items():
        assert_equal(candidate.get(key), expected, f"freeze/candidate/{key}")
    contract = candidate.get("unchanged_contract")
    if not isinstance(contract, dict):
        raise fail("freeze/candidate/unchanged_contract missing")
    for key in (
        "graph_and_factors", "values_and_initialization", "c7_d_ccdd_topology",
        "equations_and_units", "sigma_and_filter", "lm_schedule_ordering_and_damping",
        "legacy_selector_off", "no_guard_removal", "no_velocity_fallback_or_synthesis",
        "no_fallback", "no_solution_coordinate_publication",
    ):
        assert_equal(contract.get(key), True, f"freeze/unchanged_contract/{key}")
    boundary = freeze.get("execution_boundary")
    if not isinstance(boundary, dict):
        raise fail("freeze/execution_boundary missing")
    for key in (
        "implementation_authorized", "raw_execution_authorized", "solver_authorized",
        "base_rinex_read_authorized", "truth_authorized", "accuracy_authorized",
        "solution_publication_authorized", "kaggle_or_token_authorized",
    ):
        assert_equal(boundary.get(key), False, f"freeze/execution_boundary/{key}")
    assert_equal(boundary.get("runs_per_route"), 0, "freeze/execution_boundary/runs_per_route")
    return freeze


def phase95_raw_metadata() -> dict[str, dict[str, dict[str, Any]]]:
    """Return sealed phone path metadata without touching the phone files."""

    assert_equal(sha256_file(PHASE95_RESULT, "Phase95 result"), PHASE95_RESULT_SHA256, "Phase95 result/sha256")
    result = read_json(PHASE95_RESULT, "Phase95 result")
    routes = result.get("routes")
    if not isinstance(routes, dict):
        raise fail("Phase95 result/routes missing")
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
            safe_member_path(pin.get("path"), name, f"Phase95/{route}/{name}")
            assert_equal(pin.get("exists_before_launch"), True, f"Phase95/{route}/{name}/exists")
            assert_equal(pin.get("read_by_runner"), False, f"Phase95/{route}/{name}/runner-read")
            digest = pin.get("sha256")
            if not isinstance(digest, str) or len(digest) != 64:
                raise fail(f"Phase95 raw digest malformed: {route}/{name}")
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
    """Return sealed base metadata without opening/statting base.obs."""

    assert_equal(sha256_file(PHASE65_MANIFEST, "Phase65 manifest"), PHASE65_MANIFEST_SHA256, "Phase65 manifest/sha256")
    manifest = read_json(PHASE65_MANIFEST, "Phase65 manifest")
    base_inputs = manifest.get("base_inputs")
    if not isinstance(base_inputs, dict):
        raise fail("Phase65 base_inputs missing")
    resolved: dict[str, dict[str, Any]] = {}
    for route in ROUTES:
        pin = base_inputs.get(route)
        if not isinstance(pin, dict):
            raise fail(f"Phase65 base member missing: {route}")
        for key in ("path", "sha256", "bytes", "observed_dt_s", "moving_mean_samples", "approx_position_xyz_m"):
            if key not in pin:
                raise fail(f"Phase65 base metadata missing: {route}/{key}")
        safe_member_path(pin["path"], "base.obs", f"Phase65/{route}/base")
        if not isinstance(pin["sha256"], str) or len(pin["sha256"]) != 64:
            raise fail(f"Phase65 base SHA malformed: {route}")
        if not isinstance(pin["approx_position_xyz_m"], list) or len(pin["approx_position_xyz_m"]) != 3:
            raise fail(f"Phase65 base header XYZ malformed: {route}")
        resolved[route] = dict(pin)
        resolved[route].update({
            "path_source": "Phase65 sealed base_inputs metadata",
            "coordinate_source": "raw RINEX header APPROX POSITION XYZ",
            "read_at_manifest_creation": False,
            "hash_read_at_manifest_creation": False,
            "read_by_pre_raw_verifier": False,
        })
    return resolved


def verify_implementation() -> None:
    freeze = read_json(FREEZE, "Phase113 freeze")
    pins: dict[str, Path] = {
        "app": APP, "fgo": FGO, "backend": BACKEND, "internal": INTERNAL,
        "problems": PROBLEMS, "base_model": BASE_MODEL,
    }
    source_pins = freeze.get("source_pins", {})
    for group in ("official", "native"):
        entries = source_pins.get(group)
        if not isinstance(entries, list):
            raise fail(f"freeze/source_pins/{group} missing")
        for entry in entries:
            if not isinstance(entry, dict) or not isinstance(entry.get("path"), str):
                raise fail(f"malformed freeze source pin: {group}")
            path = ROOT / entry["path"]
            expected = entry.get("sha256")
            if not isinstance(expected, str) or len(expected) != 64:
                raise fail(f"malformed freeze source SHA: {entry['path']}")
            assert_equal(sha256_file(path, f"freeze source {entry['path']}"), expected, f"freeze source/{entry['path']}")
    for key, path in pins.items():
        if not path.is_file():
            raise fail(f"missing implementation/{key}: {path}")
    assert_equal(sha256_file(BINARY, "native binary"), BINARY_SHA256, "native binary/sha256")


def output_path(route: str, name: str) -> str:
    return f"{OUTPUT_ROOT}{route.replace('/', '__')}/{name}"


def command_template(route: str) -> list[str]:
    return [
        "build/apps/gnss_fgo_imu_no_base",
        "--dataset-id", route,
        "--android-gnss", "__PHASE113_RAW_DEVICE_GNSS__",
        "--android-imu", "__PHASE113_RAW_DEVICE_IMU__",
        "--nav", "__PHASE113_RAW_BROADCAST_NAV__",
        "--all-epochs",
        "--android-raw-utc-keys",
        "--android-raw-clock-only",
        "--android-utc-wall-clock-fallback",
        "--native-pdc-imu-tdcp-no-bridge",
        "--native-source-direct-observable-quality",
        "--native-source-clock-c0d-factor",
        "--native-source-clock-c0d-meter-state-parity",
        "--native-source-clock-c0d-active-solve-diagnostic",
        PHASE101_SELECTOR,
        VECTOR_SELECTOR,
        QR_SELECTOR,
        BASE_SELECTOR,
        "--native-base-rinex", "__PHASE113_RAW_BASE_RINEX__",
        "--native-base-rinex-sha256", "__PHASE113_RAW_BASE_SHA256__",
        BASE_MISS_SELECTOR,
        "--out", output_path(route, "withheld_solution_output.csv"),
        "--summary-json", output_path(route, "summary.json"),
    ]


def validate_command(route: str, command: Any, record: dict[str, Any], raw: dict[str, Any], base: dict[str, Any]) -> None:
    expected = command_template(route)
    assert_equal(command, expected, f"manifest/{route}/command")
    for token in command:
        if token in FORBIDDEN_FLAGS:
            raise fail(f"forbidden command flag: {route}/{token}")
        if not token.startswith("--") and any(term in token.lower() for term in FORBIDDEN_PATH_TERMS):
            # The only permitted base input token is the explicit placeholder
            # and the only permitted raw tokens are the three exact roles.
            if token not in {
                "__PHASE113_RAW_BASE_RINEX__", "__PHASE113_RAW_DEVICE_GNSS__",
                "__PHASE113_RAW_DEVICE_IMU__", "__PHASE113_RAW_BROADCAST_NAV__",
            }:
                raise fail(f"forbidden command path token: {route}/{token}")
    for flag in REQUIRED_FLAGS:
        assert_equal(command.count(flag), 1, f"manifest/{route}/{flag}")
    assert_equal(command.count("--native-base-rinex"), 1, f"manifest/{route}/base path count")
    assert_equal(command[command.index("--native-base-rinex") + 1], "__PHASE113_RAW_BASE_RINEX__", f"manifest/{route}/base path")
    assert_equal(command.count("--native-base-rinex-sha256"), 1, f"manifest/{route}/base SHA count")
    assert_equal(command[command.index("--native-base-rinex-sha256") + 1], "__PHASE113_RAW_BASE_SHA256__", f"manifest/{route}/base SHA")
    for flag, name in RAW_FLAGS:
        assert_equal(command.count(flag), 1, f"manifest/{route}/{flag}")
        placeholder = {
            "device_gnss.csv": "__PHASE113_RAW_DEVICE_GNSS__",
            "device_imu.csv": "__PHASE113_RAW_DEVICE_IMU__",
            "brdc.nav": "__PHASE113_RAW_BROADCAST_NAV__",
        }[name]
        assert_equal(command[command.index(flag) + 1], placeholder, f"manifest/{route}/{name}/placeholder")
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
    assert_equal(base_pin.get("read_at_manifest_creation"), False, f"manifest/base/{route}/read")
    assert_equal(base_pin.get("hash_read_at_manifest_creation"), False, f"manifest/base/{route}/hash-read")
    assert_equal(record.get("runs"), 1, f"manifest/{route}/runs")


def verify_manifest() -> dict[str, Any]:
    verify_freeze()
    verify_implementation()
    raw = phase95_raw_metadata()
    base = phase65_base_metadata()
    manifest = read_json(MANIFEST, "Phase113 diagnostic manifest")
    for key, expected in {
        "schema_version": SCHEMA,
        "phase": 113,
        "execution_label": "Luna Max",
        "status": "sealed-before-phase113-raw-base-diagnostic-execution",
    }.items():
        assert_equal(manifest.get(key), expected, f"manifest/{key}")
    freeze = manifest.get("freeze")
    if not isinstance(freeze, dict):
        raise fail("manifest/freeze missing")
    for key, expected in {
        "path": relative(FREEZE), "sha256": FREEZE_SHA256, "commit": FREEZE_COMMIT,
        "audit_commit": AUDIT_COMMIT, "raw_execution_authorized_before_manifest": False,
    }.items():
        assert_equal(freeze.get(key), expected, f"manifest/freeze/{key}")
    impl = manifest.get("implementation")
    if not isinstance(impl, dict):
        raise fail("manifest/implementation missing")
    for key in ("commit", "candidate_id", "legacy_default_unchanged", "algorithm_or_solver_source_changed"):
        if key not in impl:
            raise fail(f"manifest/implementation/{key} missing")
    assert_equal(impl.get("candidate_id"), CANDIDATE_ID, "manifest/implementation/candidate_id")
    assert_equal(impl.get("legacy_default_unchanged"), True, "manifest/implementation/default")
    assert_equal(impl.get("algorithm_or_solver_source_changed"), False, "manifest/implementation/source-change")
    for key, path in (("evaluator", EVALUATOR), ("wrapper", WRAPPER), ("focused_tests", FOCUSED_TESTS)):
        pin = manifest.get(key)
        if not isinstance(pin, dict):
            raise fail(f"manifest/{key} missing")
        assert_equal(pin.get("path"), relative(path), f"manifest/{key}/path")
        expected_hash = pin.get("sha256")
        if not isinstance(expected_hash, str) or len(expected_hash) != 64:
            raise fail(f"manifest/{key}/sha256 missing")
        assert_equal(sha256_file(path, f"manifest/{key}"), expected_hash, f"manifest/{key}/sha256")
    for key, path in (("phase95_result", PHASE95_RESULT), ("phase107_freeze", PHASE107_FREEZE), ("phase65_manifest", PHASE65_MANIFEST)):
        pin = manifest.get("sealed_inputs", {}).get(key)
        if not isinstance(pin, dict):
            raise fail(f"manifest/sealed_inputs/{key} missing")
        assert_equal(pin.get("path"), relative(path), f"manifest/sealed_inputs/{key}/path")
        expected = {
            "phase95_result": PHASE95_RESULT_SHA256,
            "phase107_freeze": PHASE107_FREEZE_SHA256,
            "phase65_manifest": PHASE65_MANIFEST_SHA256,
        }[key]
        assert_equal(pin.get("sha256"), expected, f"manifest/sealed_inputs/{key}/sha256")
    candidate = manifest.get("candidate")
    if not isinstance(candidate, dict):
        raise fail("manifest/candidate missing")
    for key, expected in {
        "id": CANDIDATE_ID, "candidate_count": 1, "diagnostic_only": True,
        "opt_in": True, "default_off": True, "raw_base_only": True,
        "runs_per_route": 1, "controls": 0, "reruns": 0, "fallbacks": 0,
        "truth_evaluation": False, "accuracy_scoring": False,
        "solution_output_publication": False, "algorithm_or_solver_change": False,
        "no_guard_removal": True, "no_velocity_fallback_or_synthesis": True,
        "raw_base_correction_selector": "Phase107 existing native path; no additional-band selector",
    }.items():
        assert_equal(candidate.get(key), expected, f"manifest/candidate/{key}")
    routes = manifest.get("routes")
    if not isinstance(routes, list) or [item.get("dataset_id") for item in routes] != list(ROUTES):
        raise fail("manifest route order/count changed")
    for record in routes:
        route = record.get("dataset_id")
        if route not in DOMAIN_ROWS:
            raise fail(f"unknown route: {route}")
        assert_equal(record.get("domain_rows"), DOMAIN_ROWS[route], f"manifest/{route}/domain_rows")
        assert_equal(record.get("expected_problem_epochs"), DOMAIN_ROWS[route] + 1, f"manifest/{route}/problem_epochs")
        validate_command(route, record.get("command"), record, raw[route], base[route])
    raw_contract = manifest.get("raw_input_contract")
    if not isinstance(raw_contract, dict):
        raise fail("manifest/raw_input_contract missing")
    for key, expected in {
        "input_names_exact": list(RAW_NAMES), "phone_gnss_only": True,
        "phone_imu_only": True, "broadcast_navigation_only": True,
        "sealed_raw_base_rinex_only": True, "content_copy_or_transform": False,
        "truth_mat_precomputed_coordinate_pdc_kaggle_accuracy": False,
        "solution_content_read": False,
    }.items():
        assert_equal(raw_contract.get(key), expected, f"manifest/raw_input_contract/{key}")
    telemetry = manifest.get("telemetry_contract")
    if not isinstance(telemetry, dict):
        raise fail("manifest/telemetry_contract missing")
    required_fields = {
        "stage_retained_epoch_count", "per_family_key_incidence_or_unavailable",
        "velocity_d_p_c_observability_admission_predicates",
        "eligible_vs_active_c0d", "raw_base_corrected_miss_taxonomy",
        "main_coverage_earth_valid_solver_progress_or_unavailable",
    }
    if not required_fields.issubset(set(telemetry.get("required_fields", []))):
        raise fail("manifest/telemetry_contract incomplete")
    gates = manifest.get("structural_gates")
    if not isinstance(gates, dict):
        raise fail("manifest/structural_gates missing")
    for key in (
        "stage_retained_and_admission_telemetry", "eligible_vs_active_c0d_telemetry",
        "raw_base_corrected_miss_telemetry", "main_validation_telemetry",
        "finite_earth_valid_handoff_when_reached", "no_fallback_or_solution_publication",
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
    authorization = read_json(AUTHORIZATION, "Phase113 authorization")
    for key, expected in {
        "schema_version": AUTH_SCHEMA, "phase": 113, "execution_label": "Luna Max",
        "status": "authorized-for-exact-two-route-phase113-raw-base-diagnostic-execution",
    }.items():
        assert_equal(authorization.get(key), expected, f"authorization/{key}")
    authority = authorization.get("authority")
    if not isinstance(authority, dict):
        raise fail("authorization/authority missing")
    for key, path in (
        ("phase113_freeze", FREEZE), ("phase113_manifest", MANIFEST),
        ("phase113_evaluator", EVALUATOR), ("phase113_wrapper", WRAPPER),
        ("phase113_focused_tests", FOCUSED_TESTS), ("phase95_result", PHASE95_RESULT),
        ("phase107_freeze", PHASE107_FREEZE), ("phase65_manifest", PHASE65_MANIFEST),
    ):
        pin = authority.get(key)
        if not isinstance(pin, dict):
            raise fail(f"authorization/{key} missing")
        assert_equal(pin.get("path"), relative(path), f"authorization/{key}/path")
        expected_hash = pin.get("sha256")
        if not isinstance(expected_hash, str) or len(expected_hash) != 64:
            raise fail(f"authorization/{key}/sha256 missing")
        expected = {
            "phase113_freeze": FREEZE_SHA256, "phase95_result": PHASE95_RESULT_SHA256,
            "phase107_freeze": PHASE107_FREEZE_SHA256, "phase65_manifest": PHASE65_MANIFEST_SHA256,
        }.get(key, sha256_file(path, f"authorization/{key}"))
        assert_equal(expected_hash, expected, f"authorization/{key}/sha256")
    for key, expected in {
        "freeze_commit": FREEZE_COMMIT, "audit_commit": AUDIT_COMMIT,
        "implementation_commit": manifest["implementation"]["commit"],
        "manifest_sha256": sha256_file(MANIFEST, "Phase113 manifest"),
        "evaluator_sha256": sha256_file(EVALUATOR, "Phase113 evaluator"),
        "wrapper_sha256": sha256_file(WRAPPER, "Phase113 wrapper"),
        "focused_tests_sha256": sha256_file(FOCUSED_TESTS, "Phase113 focused tests"),
        "binary_sha256": BINARY_SHA256,
    }.items():
        assert_equal(authority.get(key), expected, f"authorization/authority/{key}")
    assert_equal(authorization.get("routes"), list(ROUTES), "authorization/routes")
    candidate = authorization.get("candidate")
    if not isinstance(candidate, dict):
        raise fail("authorization/candidate missing")
    for key, expected in {
        "id": CANDIDATE_ID, "candidate_count": 1, "runs_per_route": 1,
        "controls": 0, "reruns": 0, "fallbacks": 0, "truth_evaluation": False,
        "accuracy_scoring": False, "solution_output_publication": False,
        "no_guard_bypass": True, "raw_base_only": True,
    }.items():
        assert_equal(candidate.get(key), expected, f"authorization/candidate/{key}")
    matrix = authorization.get("matrix")
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
    policy = authorization.get("execution_policy")
    if not isinstance(policy, dict):
        raise fail("authorization/execution_policy missing")
    for key, expected in {
        "authorized": True, "order": "MTV-H then MTV-U, sequentially",
        "one_invocation_per_route": True, "no_rerun": True, "no_fallback": True,
        "algorithm_graph_factor_guard_equation_unit_sigma_lm_unchanged": True,
        "truth_or_accuracy_evaluation": False,
        "solution_output": "isolated withheld path, never opened/published/committed",
        "base_input": "sealed raw base RINEX; one wrapper hash read plus one native process read per route",
    }.items():
        assert_equal(policy.get(key), expected, f"authorization/policy/{key}")
    boundary = authorization.get("release_boundary")
    if not isinstance(boundary, dict):
        raise fail("authorization/release_boundary missing")
    for key, expected in {
        "raw_base_execution_authorized": True, "structural_result_authorized": True,
        "truth_or_accuracy_evaluation_authorized": False,
        "solution_publication_authorized": False, "kaggle_submission_authorized": False,
    }.items():
        assert_equal(boundary.get(key), expected, f"authorization/release_boundary/{key}")
    return authorization


def verify_pre_raw() -> dict[str, Any]:
    freeze = verify_freeze()
    manifest = verify_manifest()
    return {
        "status": "pre-raw-verified", "phase": 113, "execution_label": "Luna Max",
        "candidate_count": 1, "route_ids": list(ROUTES), "runs_per_route": 1,
        "raw_execution_authorized": False, "raw_phone_gnss_reads": 0,
        "raw_phone_imu_reads": 0, "broadcast_navigation_reads": 0,
        "raw_base_rinex_reads": 0, "raw_base_hash_reads": 0,
        "native_solver_invocations": 0, "truth_reads": 0,
        "mat_reads_or_generated": 0, "phone_coordinate_reads": 0,
        "precomputed_coordinate_reads": 0, "pdc_reads": 0,
        "accuracy_calculations": 0, "solution_output_published": False,
        "freeze_sha256": FREEZE_SHA256,
        "manifest_sha256": sha256_file(MANIFEST, "Phase113 manifest"),
        "freeze_status": freeze["status"], "manifest_status": manifest["status"],
    }


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--verify-freeze", action="store_true")
    parser.add_argument("--verify-manifest", action="store_true")
    parser.add_argument("--verify-pre-raw", action="store_true")
    parser.add_argument("--verify-authorization", action="store_true")
    args = parser.parse_args()
    if not any((args.verify_freeze, args.verify_manifest, args.verify_pre_raw, args.verify_authorization)):
        parser.error("one verification mode is required")
    try:
        if args.verify_freeze:
            verify_freeze()
        if args.verify_manifest:
            verify_manifest()
        if args.verify_pre_raw:
            print(json.dumps(verify_pre_raw(), indent=2, sort_keys=True))
        if args.verify_authorization:
            verify_authorization()
    except (Phase113ContractError, OSError) as exc:
        print(f"phase113 pre-raw verifier: fail-closed: {exc}", file=sys.stderr)
        raise SystemExit(2)
