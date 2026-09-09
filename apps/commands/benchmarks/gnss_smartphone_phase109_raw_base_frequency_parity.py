#!/usr/bin/env python3
"""Launch-free Phase109 raw-base frequency-parity contract.

This verifier reads only the Phase109 freeze, sealed Phase95/65 metadata, the
pinned implementation sources, and the structural manifest.  It never opens,
stats, or hashes a phone raw member or a base RINEX member.  A separate
authorization record is required before the execution wrapper may materialize
those paths and launch the two native processes.
"""

from __future__ import annotations

import hashlib
import json
from pathlib import Path
import sys
from typing import Any


ROOT = Path(__file__).resolve().parents[3]
FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_phase109_raw_base_miss_frequency_parity_freeze_v1.json"
MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase109_raw_base_frequency_parity_manifest_v1.json"
AUTHORIZATION = ROOT / "docs/use_cases/records/smartphone_r5_phase109_raw_base_frequency_parity_authorization_v1.json"
EVALUATOR = Path(__file__).resolve()
WRAPPER = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase109_raw_base_frequency_parity_execute.py"
FOCUSED_TESTS = ROOT / "tests/test_smartphone_phase109_raw_base_frequency_parity.py"
PHASE95_RESULT = ROOT / "docs/use_cases/records/smartphone_r5_phase95_raw_input_path_corrected_structural_result_v1.json"
PHASE65_MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase65_native_base_pseudorange_compensation_manifest_v1.json"

APP = ROOT / "apps/native/gnss_fgo_imu_no_base.cpp"
FGO = ROOT / "include/libgnss++/algorithms/fgo.hpp"
MISS_MASK_HEADER = ROOT / "include/libgnss++/algorithms/source_pseudorange_miss_mask.hpp"
MISS_MASK = ROOT / "src/algorithms/source_pseudorange_miss_mask.cpp"
BACKEND = ROOT / "src/algorithms/fgo_gtsam_backend.cpp"
INTERNAL = ROOT / "src/algorithms/fgo_gtsam_internal.hpp"
CONFIG = ROOT / "include/libgnss++/algorithms/fgo_config.hpp"
BASE_MODEL = ROOT / "src/algorithms/base_pseudorange_compensation.cpp"
BASE_MODEL_HEADER = ROOT / "include/libgnss++/algorithms/base_pseudorange_compensation.hpp"
RINEX = ROOT / "src/io/rinex.cpp"
RINEX_HEADER = ROOT / "include/libgnss++/io/rinex.hpp"
BINARY = ROOT / "build/apps/gnss_fgo_imu_no_base"

FREEZE_SHA256 = "723fdedd987d200653422df763975c6b3156cbe747769b8690ae10b162d4dce0"
FREEZE_COMMIT = "0f8f82d759820193ef2cd1b9108e2e964e097227"
AUDIT_COMMIT = "ea230fbbe1f533d8701646fdfe28e228975f9927"
IMPLEMENTATION_COMMIT = "e2eb92f0fe3832809d8f3d5b98817ec832b4332f"
PHASE95_RESULT_SHA256 = "beba25d4b0d910064d76bea7aa6b7a884937ca01c3b00cf0f7f9f4b6267ea281"
PHASE65_MANIFEST_SHA256 = "1306480f6b7fb839e55309e9c2c77b41f2ac0a1baf9f6971949612fd485d2255"

SOURCE_SHA256 = {
    "app": "be643811cfaf82304f9434bc969796b2a14cd0f6e5a2c8b46b043af7b326c5c2",
    "fgo": "f13d743225184d5ed4b229f473fb69b290f61aa83398753bdb2d6a95ceeb2e05",
    "miss_mask_header": "6a722da758555555d4fdcfedf146e0031320294fc4194bb6286aad603dc13e82",
    "miss_mask": "ce85e428ac0a7af93ee1ad3eadf082b5361454dff5c920245be7c16c940b6949",
    "backend": "781d65e34826ec08e04d1dfdac0ab10b2ed9409726078465f3020b57afac4baa",
    "internal": "cc6327e36800afa8217e5a834260adc81c0a74a5aefbfc956bb4679c8d86105f",
    "config": "3cc60abe514ef8900012064accb17f27d3927d0b8a8776ad76ce6aefa83ce32c",
    "base_model": "f3c3c859793c4bed9c7ae93825d32565f20eab0badb09412a4fab2a5930c9817",
    "base_model_header": "a182091ef47af8692e3f4dfb0d714cf365f5b24bc6927a8204d05648248e026b",
    "rinex": "92f683e34b4052f8d105f69fc4bf4397ca990be3b5a6b220181a56423f465533",
    "rinex_header": "571ac4e0e3e301e6a8b865ea89caaae3acd40dc8b25f4f1bdb3ede751e9e6e5d",
    "binary": "780a4b89bda8cca75d7c8ac00ae394d1b0da20faf699e55cde0b24d0a5060f9a",
}

ROUTES = (
    "2021-03-16-18-59-us-ca-mtv-a/pixel5",
    "2022-04-01-18-22-us-ca-lax-t/pixel5",
)
DOMAIN_ROWS = {ROUTES[0]: 2158, ROUTES[1]: 1465}
RAW_NAMES = ("device_gnss.csv", "device_imu.csv", "brdc.nav")
RAW_FLAGS = (
    ("--android-gnss", "device_gnss.csv"),
    ("--android-imu", "device_imu.csv"),
    ("--nav", "brdc.nav"),
)
BASE_FLAG = ("--native-base-rinex", "__PHASE109_RAW_BASE_RINEX__")
BASE_SHA_FLAG = ("--native-base-rinex-sha256", "__PHASE109_RAW_BASE_SHA256__")
PHASE93_SELECTOR = "--native-source-clock-c0d-gnss-first-meter-state-handoff"
VECTOR_SELECTOR = "--native-source-clock-c0d-epoch-vector-parity"
QR_SELECTOR = "--native-source-clock-c0d-phase99-main-multifrontal-qr-solver"
PRESERVE_SELECTOR = "--native-base-pseudorange-preserve-additional-frequency-bands"
CANDIDATE_ID = "phase109-phase101-raw-base-existing-additional-frequency-band-guard-v1"
OUTPUT_ROOT = "output/smartphone-r5/phase109-raw-base-frequency-parity-v1/"
SCHEMA = "smartphone-r5-phase109-raw-base-frequency-parity-manifest.v1"
AUTH_SCHEMA = "smartphone-r5-phase109-raw-base-frequency-parity-authorization.v1"

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
    PHASE93_SELECTOR,
    VECTOR_SELECTOR,
    QR_SELECTOR,
    "--native-base-pseudorange-compensation",
    "--native-base-pseudorange-source-miss-mask",
    PRESERVE_SELECTOR,
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
)
FORBIDDEN_PATH_TERMS = (
    ".mat", "truth", "ground_truth", "validation", "holdout", "precomputed",
    "coordinate", "pdc", "kaggle", "token",
)


class Phase109ContractError(ValueError):
    """Raised when the pinned Phase109 contract fails closed."""


def fail(message: str) -> Phase109ContractError:
    return Phase109ContractError(message)


def relative(path: Path) -> str:
    try:
        return str(path.relative_to(ROOT))
    except ValueError:
        return str(path)


def assert_equal(actual: Any, expected: Any, label: str) -> None:
    if actual != expected:
        raise fail(f"{label}: expected {expected!r}, got {actual!r}")


def sha256_file(path: Path, label: str) -> str:
    # The pre-raw verifier must never hash any payload member.  The wrapper
    # has a separate, post-authorization base hash function.
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


def safe_relative_member(path_text: Any, basename: str, label: str) -> None:
    if not isinstance(path_text, str):
        raise fail(f"{label} path missing")
    path = Path(path_text)
    if path.is_absolute() or ".." in path.parts or path.name != basename:
        raise fail(f"unsafe {label} path: {path_text}")


def verify_freeze() -> dict[str, Any]:
    assert_equal(sha256_file(FREEZE, "Phase109 freeze"), FREEZE_SHA256, "freeze/sha256")
    freeze = read_json(FREEZE, "Phase109 freeze")
    for key, expected in {
        "schema_version": "smartphone-r5-phase109-raw-base-miss-frequency-parity-freeze.v1",
        "phase": 109,
        "status": "frozen-read-only-before-phase109-raw-execution",
        "execution_label": "Luna Max",
    }.items():
        assert_equal(freeze.get(key), expected, f"freeze/{key}")
    decision = freeze.get("decision")
    if not isinstance(decision, dict):
        raise fail("freeze/decision missing")
    for key, expected in {
        "candidate_id": CANDIDATE_ID,
        "candidate_count": 1,
        "raw_execution_authorized_at_freeze": False,
        "truth_evaluation_authorized": False,
        "accuracy_authorized": False,
        "solution_publication_authorized": False,
        "kaggle_submission_authorized": False,
    }.items():
        assert_equal(decision.get(key), expected, f"freeze/decision/{key}")
    contract = freeze.get("candidate_contract")
    if not isinstance(contract, dict):
        raise fail("freeze/candidate_contract missing")
    assert_equal(contract.get("opt_in"), True, "freeze/candidate_contract/opt_in")
    assert_equal(contract.get("default_off"), True, "freeze/candidate_contract/default_off")
    assert_equal(contract.get("required_phase101_selectors"), [PHASE93_SELECTOR, VECTOR_SELECTOR, QR_SELECTOR], "freeze/Phase101 selectors")
    assert_equal(contract.get("required_raw_base_selectors"), [
        "--native-base-pseudorange-compensation",
        "--native-base-pseudorange-source-miss-mask",
        PRESERVE_SELECTOR,
        "--native-base-rinex <exact sealed Phase107 route member>",
        "--native-base-rinex-sha256 <exact sealed Phase107 digest>",
    ], "freeze/raw-base selectors")
    accounting = freeze.get("read_accounting_phase109")
    if not isinstance(accounting, dict):
        raise fail("freeze/read_accounting_phase109 missing")
    for key in (
        "raw_device_gnss_payloads",
        "raw_device_imu_payloads",
        "broadcast_navigation_payloads",
        "raw_base_rinex_bytes",
        "raw_base_rinex_headers",
        "raw_base_files_stat_or_hash",
        "truth_payloads",
        "mat_payloads",
        "phone_result_or_precomputed_coordinate_payloads",
        "native_solver_invocations",
        "accuracy_evaluator_invocations",
        "kaggle_or_token_resources",
    ):
        assert_equal(accounting.get(key), 0, f"freeze/read_accounting_phase109/{key}")
    return freeze


def phase95_raw_metadata() -> dict[str, dict[str, dict[str, Any]]]:
    """Read sealed path/hash metadata only; never touch the raw files."""

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
            safe_relative_member(pin.get("path"), name, f"Phase95/{route}/{name}")
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
    """Read sealed base metadata; do not open or stat a base member."""

    assert_equal(sha256_file(PHASE65_MANIFEST, "Phase65 manifest"), PHASE65_MANIFEST_SHA256, "Phase65 manifest/sha256")
    manifest = read_json(PHASE65_MANIFEST, "Phase65 manifest")
    base_inputs = manifest.get("base_inputs")
    if not isinstance(base_inputs, dict):
        raise fail("Phase65 base_inputs missing")
    expected = {
        ROUTES[0]: {
            "path": "output/smartphone-r5/phase63-settings-integrity-recovery-v1/routes/2021-03-16-18-59-us-ca-mtv-a__pixel5/base.obs",
            "sha256": "380b8ff9091344fb756697e27f0983d9a0ba2cf0c201b96849bfc1ecc1af0e52",
            "bytes": 10708536,
            "observed_dt_s": 1.0,
            "moving_mean_samples": 151,
            "approx_position_xyz_m": [-2703115.921, -4291767.2078, 3854247.9066],
        },
        ROUTES[1]: {
            "path": "output/smartphone-r5/phase63-settings-integrity-recovery-v1/routes/2022-04-01-18-22-us-ca-lax-t__pixel5/base.obs",
            "sha256": "d731e0e8a7ba4396d62340c85b6238e66c50c6a349621f9dcea0ea6885fd4cfe",
            "bytes": 719969,
            "observed_dt_s": 15.0,
            "moving_mean_samples": 11,
            "approx_position_xyz_m": [-2507798.7984, -4676369.6918, 3526890.8008],
        },
    }
    resolved: dict[str, dict[str, Any]] = {}
    for route in ROUTES:
        pin = base_inputs.get(route)
        if not isinstance(pin, dict):
            raise fail(f"Phase65 base member missing: {route}")
        for key, value in expected[route].items():
            assert_equal(pin.get(key), value, f"Phase65/{route}/{key}")
        safe_relative_member(pin.get("path"), "base.obs", f"Phase65/{route}/base")
        resolved[route] = dict(value for value in expected[route].items())
        resolved[route].update({
            "path_source": "Phase65 sealed base_inputs metadata",
            "coordinate_source": "RINEX header APPROX POSITION XYZ",
            "read_at_manifest_creation": False,
            "hash_read_at_manifest_creation": False,
            "read_by_pre_raw_verifier": False,
        })
    return resolved


def verify_implementation() -> None:
    paths = {
        "app": APP,
        "fgo": FGO,
        "miss_mask_header": MISS_MASK_HEADER,
        "miss_mask": MISS_MASK,
        "backend": BACKEND,
        "internal": INTERNAL,
        "config": CONFIG,
        "base_model": BASE_MODEL,
        "base_model_header": BASE_MODEL_HEADER,
        "rinex": RINEX,
        "rinex_header": RINEX_HEADER,
        "binary": BINARY,
    }
    for key, path in paths.items():
        assert_equal(sha256_file(path, f"implementation/{key}"), SOURCE_SHA256[key], f"implementation/{key}/sha256")
    source = APP.read_text(encoding="utf-8")
    for marker in (
        "phase101_exact_selector_recipe",
        "phase107_raw_base_source_parity_admission",
        "phase109_raw_base_frequency_parity_admission",
        "native_base_pseudorange_source_miss_mask",
        "native_base_pseudorange_preserve_additional_frequency_bands",
        "source_miss_taxonomy_by_signal",
        "correction_applied_exactly_once",
        "native_base_pseudorange_correction_applied",
    ):
        if marker not in source:
            raise fail(f"implementation marker missing: {marker}")
    mask_header = MISS_MASK_HEADER.read_text(encoding="utf-8")
    if "std::map<SignalType, SignalCounts> signal_counts;" not in mask_header:
        raise fail("signal taxonomy map missing")


def output_path(route: str, name: str) -> str:
    return f"{OUTPUT_ROOT}{route.replace('/', '__')}/{name}"


def command_template(route: str) -> list[str]:
    return [
        "build/apps/gnss_fgo_imu_no_base",
        "--dataset-id", route,
        "--android-gnss", "__PHASE109_RAW_DEVICE_GNSS__",
        "--android-imu", "__PHASE109_RAW_DEVICE_IMU__",
        "--nav", "__PHASE109_RAW_BROADCAST_NAV__",
        "--all-epochs",
        "--android-raw-utc-keys",
        "--android-raw-clock-only",
        "--android-utc-wall-clock-fallback",
        "--native-pdc-imu-tdcp-no-bridge",
        "--native-source-direct-observable-quality",
        "--native-source-clock-c0d-factor",
        "--native-source-clock-c0d-meter-state-parity",
        "--native-source-clock-c0d-active-solve-diagnostic",
        PHASE93_SELECTOR,
        VECTOR_SELECTOR,
        QR_SELECTOR,
        "--native-base-pseudorange-compensation",
        "--native-base-pseudorange-source-miss-mask",
        PRESERVE_SELECTOR,
        BASE_FLAG[0], BASE_FLAG[1],
        BASE_SHA_FLAG[0], BASE_SHA_FLAG[1],
        "--out", output_path(route, "withheld_solution_output.csv"),
        "--summary-json", output_path(route, "summary.json"),
    ]


def validate_command(route: str, command: Any, record: dict[str, Any], raw: dict[str, Any], base: dict[str, Any]) -> None:
    if command != command_template(route):
        raise fail(f"manifest command differs from exact Phase109 template: {route}")
    for token in command:
        # ``--native-pdc-imu-tdcp-no-bridge`` is an explicitly required
        # native mode selector; apply path-content guards only to operands,
        # while forbidden option names remain exact flag checks above.
        if token in FORBIDDEN_FLAGS or (
            not token.startswith("--") and any(term in token.lower() for term in FORBIDDEN_PATH_TERMS)
        ):
            raise fail(f"forbidden command token: {route}/{token}")
    for flag in REQUIRED_FLAGS:
        assert_equal(command.count(flag), 1, f"command/{route}/{flag}")
    assert_equal(command.count(BASE_FLAG[0]), 1, f"command/{route}/base path count")
    assert_equal(command[command.index(BASE_FLAG[0]) + 1], BASE_FLAG[1], f"command/{route}/base path")
    assert_equal(command.count(BASE_SHA_FLAG[0]), 1, f"command/{route}/base SHA count")
    assert_equal(command[command.index(BASE_SHA_FLAG[0]) + 1], BASE_SHA_FLAG[1], f"command/{route}/base SHA")
    for flag, name in RAW_FLAGS:
        assert_equal(command.count(flag), 1, f"command/{route}/{flag}")
        placeholder = "__PHASE109_RAW_" + ("DEVICE_GNSS__" if name == "device_gnss.csv" else "DEVICE_IMU__" if name == "device_imu.csv" else "BROADCAST_NAV__")
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
    raw = phase95_raw_metadata()
    base = phase65_base_metadata()
    manifest = read_json(MANIFEST, "Phase109 structural manifest")
    for key, expected in {
        "schema_version": SCHEMA,
        "phase": 109,
        "execution_label": "Luna Max",
        "status": "sealed-before-phase109-raw-base-frequency-execution",
    }.items():
        assert_equal(manifest.get(key), expected, f"manifest/{key}")
    freeze = manifest.get("freeze")
    if not isinstance(freeze, dict):
        raise fail("manifest/freeze missing")
    for key, expected in {
        "path": relative(FREEZE),
        "sha256": FREEZE_SHA256,
        "commit": FREEZE_COMMIT,
        "audit_commit": AUDIT_COMMIT,
        "raw_execution_authorized_before_manifest": False,
    }.items():
        assert_equal(freeze.get(key), expected, f"manifest/freeze/{key}")
    implementation = manifest.get("implementation")
    if not isinstance(implementation, dict):
        raise fail("manifest/implementation missing")
    assert_equal(implementation.get("commit"), IMPLEMENTATION_COMMIT, "manifest/implementation/commit")
    assert_equal(implementation.get("candidate_id"), CANDIDATE_ID, "manifest/implementation/candidate_id")
    assert_equal(implementation.get("legacy_default_unchanged"), True, "manifest/implementation/default")
    assert_equal(implementation.get("solver_filter_lm_equation_unit_sigma_changed"), False, "manifest/implementation/invariants")
    for key, path in {
        "app": APP, "fgo": FGO, "miss_mask_header": MISS_MASK_HEADER,
        "miss_mask": MISS_MASK, "backend": BACKEND, "internal": INTERNAL,
        "config": CONFIG, "base_model": BASE_MODEL, "base_model_header": BASE_MODEL_HEADER,
        "rinex": RINEX, "rinex_header": RINEX_HEADER, "binary": BINARY,
    }.items():
        pin = implementation.get(key)
        if not isinstance(pin, dict):
            raise fail(f"manifest/implementation/{key} missing")
        assert_equal(pin.get("path"), relative(path), f"manifest/implementation/{key}/path")
        assert_equal(pin.get("sha256"), SOURCE_SHA256[key], f"manifest/implementation/{key}/sha256")
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
        "id": CANDIDATE_ID,
        "candidate_count": 1,
        "opt_in": True,
        "default_off_outside_exact_recipe": True,
        "selectors": [PHASE93_SELECTOR, VECTOR_SELECTOR, QR_SELECTOR],
        "base_selectors": ["--native-base-pseudorange-compensation", "--native-base-pseudorange-source-miss-mask", PRESERVE_SELECTOR],
        "raw_base_only": True,
        "runs_per_route": 1,
        "controls": 0,
        "reruns": 0,
        "fallbacks": 0,
        "truth_evaluation": False,
        "accuracy_scoring": False,
        "solution_output_publication": False,
        "no_global_isb_double_state": True,
        "base_correction_applied_exactly_once": True,
        "equation_unit_sigma_filter_lm_c7_d_qr_changed": False,
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
        assert_equal(record.get("expected_problem_epochs"), DOMAIN_ROWS[route] + 1, f"manifest/{route}/problem epochs")
        validate_command(route, record.get("command"), record, raw[route], base[route])
    raw_contract = manifest.get("raw_input_contract")
    if not isinstance(raw_contract, dict):
        raise fail("manifest/raw_input_contract missing")
    for key, expected in {
        "input_names_exact": list(RAW_NAMES),
        "phone_gnss_only": True,
        "phone_imu_only": True,
        "broadcast_navigation_only": True,
        "sealed_raw_base_rinex_only": True,
        "content_copy_or_transform": False,
        "truth_mat_precomputed_coordinate_pdc_kaggle_accuracy": False,
    }.items():
        assert_equal(raw_contract.get(key), expected, f"manifest/raw_input_contract/{key}")
    base_contract = manifest.get("base_contract")
    if not isinstance(base_contract, dict):
        raise fail("manifest/base_contract missing")
    for key, expected in {
        "existing_reader_selector_only": True,
        "preserve_additional_frequency_bands": True,
        "correction_formula": "P_rover_corrected_m=P_rover_raw_m-pc(t)",
        "stream_key": "exact satellite and SignalType",
        "in_domain_interpolation_only": True,
        "no_extrapolation_or_endpoint_hold": True,
        "source_miss_mask": True,
        "base_coordinate_source": "raw base RINEX header APPROX POSITION XYZ",
        "spp_doppler_tdcp_carrier_unchanged": True,
        "new_factor_family": False,
        "hash_verification_reads_per_route": 1,
        "native_process_reads_per_route": 1,
    }.items():
        assert_equal(base_contract.get(key), expected, f"manifest/base_contract/{key}")
    telemetry = manifest.get("telemetry_contract")
    if not isinstance(telemetry, dict):
        raise fail("manifest/telemetry_contract missing")
    required_fields = {
        "selected_band_observation_rows_by_signal",
        "selected_band_streams_by_signal",
        "signal_taxonomy_by_signal",
        "retained_finite_pc_rows",
        "corrected_rows",
        "dropped_missing_exact_stream_rows",
        "dropped_out_of_domain_rows",
        "correction_application_pass_count",
        "correction_applied_exactly_once",
        "duplicate_correction_rejected",
        "factor_count_consistent",
        "signal_count_consistent",
    }
    if not required_fields.issubset(set(telemetry.get("required_fields", []))):
        raise fail("manifest telemetry contract is incomplete")
    gates = manifest.get("structural_gates")
    if not isinstance(gates, dict):
        raise fail("manifest/structural_gates missing")
    for key in (
        "base_path_hash_bytes_header_match",
        "base_preserve_reader_active",
        "base_factors_active_exactly_once",
        "base_signal_band_taxonomy_conserved",
        "gnss_first_progress_strict_cost_decrease",
        "gnss_first_full_finite_c7_d_exact_handoff",
        "main_qr_progress_strict_cost_decrease",
        "finite_earth_valid_expected_output_coverage",
        "no_fallback_or_solution_publication",
    ):
        assert_equal(gates.get(key), True, f"manifest/structural_gates/{key}")
    matrix = manifest.get("matrix")
    if not isinstance(matrix, dict):
        raise fail("manifest/matrix missing")
    for key, expected in {
        "candidate_count": 1,
        "route_count": 2,
        "runs_per_route": 1,
        "native_solver_invocations_planned": 2,
        "controls": 0,
        "reruns": 0,
        "fallbacks": 0,
        "truth_reads_planned": 0,
        "mat_reads_or_generated_planned": 0,
        "phone_coordinate_reads_planned": 0,
        "precomputed_coordinate_reads_planned": 0,
        "pdc_reads_planned": 0,
        "accuracy_calculations_planned": 0,
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
    authorization = read_json(AUTHORIZATION, "Phase109 authorization")
    for key, expected in {
        "schema_version": AUTH_SCHEMA,
        "phase": 109,
        "execution_label": "Luna Max",
        "status": "authorized-for-exact-two-route-phase109-raw-base-frequency-structural-execution",
    }.items():
        assert_equal(authorization.get(key), expected, f"authorization/{key}")
    authority = authorization.get("authority")
    if not isinstance(authority, dict):
        raise fail("authorization/authority missing")
    for key, path in (
        ("phase109_freeze", FREEZE), ("phase109_manifest", MANIFEST),
        ("phase109_evaluator", EVALUATOR), ("phase109_wrapper", WRAPPER),
        ("phase109_focused_tests", FOCUSED_TESTS),
        ("phase95_result", PHASE95_RESULT), ("phase65_manifest", PHASE65_MANIFEST),
    ):
        pin = authority.get(key)
        if not isinstance(pin, dict):
            raise fail(f"authorization/{key} missing")
        assert_equal(pin.get("path"), relative(path), f"authorization/{key}/path")
        expected_hash = pin.get("sha256")
        if not isinstance(expected_hash, str) or len(expected_hash) != 64:
            raise fail(f"authorization/{key}/sha256 missing")
        expected = FREEZE_SHA256 if key == "phase109_freeze" else PHASE95_RESULT_SHA256 if key == "phase95_result" else PHASE65_MANIFEST_SHA256 if key == "phase65_manifest" else sha256_file(path, f"authorization/{key}")
        assert_equal(expected_hash, expected, f"authorization/{key}/sha256")
    for key, expected in {
        "freeze_commit": FREEZE_COMMIT,
        "audit_commit": AUDIT_COMMIT,
        "implementation_commit": IMPLEMENTATION_COMMIT,
        "manifest_sha256": sha256_file(MANIFEST, "Phase109 manifest"),
        "evaluator_sha256": sha256_file(EVALUATOR, "Phase109 evaluator"),
        "wrapper_sha256": sha256_file(WRAPPER, "Phase109 wrapper"),
        "binary_sha256": SOURCE_SHA256["binary"],
    }.items():
        assert_equal(authority.get(key), expected, f"authorization/authority/{key}")
    assert_equal(authorization.get("routes"), list(ROUTES), "authorization/routes")
    candidate = authorization.get("candidate")
    if not isinstance(candidate, dict):
        raise fail("authorization/candidate missing")
    for key, expected in {
        "id": CANDIDATE_ID,
        "candidate_count": 1,
        "selectors": [PHASE93_SELECTOR, VECTOR_SELECTOR, QR_SELECTOR],
        "base_selectors": ["--native-base-pseudorange-compensation", "--native-base-pseudorange-source-miss-mask", PRESERVE_SELECTOR],
        "runs_per_route": 1,
        "controls": 0,
        "reruns": 0,
        "fallbacks": 0,
        "truth_evaluation": False,
        "accuracy_scoring": False,
        "solution_output_publication": False,
        "no_fallback_or_guard_bypass": True,
        "correction_exactly_once_gate": True,
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
        "raw_phone_gnss_process_reads_max": 2,
        "raw_phone_imu_process_reads_max": 2,
        "broadcast_navigation_process_reads_max": 2,
        "raw_base_hash_reads_max": 2,
        "raw_base_native_process_reads_max": 2,
        "truth_reads": 0,
        "mat_reads_or_generated": 0,
        "phone_coordinate_reads": 0,
        "precomputed_coordinate_reads": 0,
        "pdc_reads": 0,
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
        "no_fallback": True,
        "solver_filter_lm_equation_unit_sigma_unchanged": True,
        "truth_or_accuracy_evaluation": False,
        "solution_output": "isolated withheld path, never opened/published/committed",
        "base_input": "sealed raw base RINEX; one wrapper hash read plus one native process read per route",
    }.items():
        assert_equal(policy.get(key), expected, f"authorization/policy/{key}")
    boundary = authorization.get("release_boundary")
    if not isinstance(boundary, dict):
        raise fail("authorization/release_boundary missing")
    for key, expected in {
        "raw_base_execution_authorized": True,
        "structural_result_authorized": True,
        "truth_or_accuracy_evaluation_authorized": False,
        "solution_publication_authorized": False,
        "kaggle_submission_authorized": False,
    }.items():
        assert_equal(boundary.get(key), expected, f"authorization/release_boundary/{key}")
    return authorization


def verify_pre_raw() -> dict[str, Any]:
    freeze = verify_freeze()
    manifest = verify_manifest()
    return {
        "status": "pre-raw-verified",
        "phase": 109,
        "execution_label": "Luna Max",
        "candidate_count": 1,
        "route_ids": list(ROUTES),
        "runs_per_route": 1,
        "raw_execution_authorized": False,
        "raw_phone_gnss_reads": 0,
        "raw_phone_imu_reads": 0,
        "broadcast_navigation_reads": 0,
        "raw_base_rinex_reads": 0,
        "raw_base_hash_reads": 0,
        "native_solver_invocations": 0,
        "truth_reads": 0,
        "mat_reads_or_generated": 0,
        "phone_coordinate_reads": 0,
        "precomputed_coordinate_reads": 0,
        "pdc_reads": 0,
        "accuracy_calculations": 0,
        "solution_output_published": False,
        "freeze_sha256": FREEZE_SHA256,
        "manifest_sha256": sha256_file(MANIFEST, "Phase109 manifest"),
        "freeze_status": freeze["status"],
        "manifest_status": manifest["status"],
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
    except (Phase109ContractError, OSError) as exc:
        print(f"phase109 pre-raw verifier: fail-closed: {exc}", file=sys.stderr)
        raise SystemExit(2)
