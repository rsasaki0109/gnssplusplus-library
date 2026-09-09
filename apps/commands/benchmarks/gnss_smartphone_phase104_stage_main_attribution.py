#!/usr/bin/env python3
"""Phase104 raw-only stage/main attribution contract and truth evaluator.

The pre-raw verification path reads only pinned source/manifest metadata.  A
separate execution wrapper owns the native Phase101 launch after a separate
authorization record is committed.  The native process receives only raw
Android GNSS, raw Android IMU, and broadcast navigation.  The
GNSS-first ECEF/timestamp sidecar is a copy made after the existing in-memory
handoff and is never fed back into inference.  The evaluator converts that
sidecar to geodetic coordinates in memory and opens each pinned truth file
once, after both candidate artifacts have been sealed.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import io
import json
import math
from pathlib import Path
import tempfile
from typing import Any, Iterable


ROOT = Path(__file__).resolve().parents[3]
EVALUATOR = Path(__file__).resolve()
FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_phase104_stage_main_accuracy_attribution_freeze_v1.json"
MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase104_stage_main_accuracy_attribution_manifest_v1.json"
AUTHORIZATION = ROOT / "docs/use_cases/records/smartphone_r5_phase104_stage_main_attribution_raw_execution_authorization_v1.json"
TRUTH_AUTHORIZATION = ROOT / "docs/use_cases/records/smartphone_r5_phase104_stage_main_attribution_truth_only_authorization_v1.json"
WRAPPER = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase104_stage_main_attribution_execute.py"
FOCUSED_TESTS = ROOT / "tests/test_smartphone_phase104_stage_main_attribution.py"
PHASE95_RESULT = ROOT / "docs/use_cases/records/smartphone_r5_phase95_raw_input_path_corrected_structural_result_v1.json"
PHASE95_WRAPPER = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase95_raw_input_path_corrected_execute.py"
PHASE101_RESULT = ROOT / "docs/use_cases/records/smartphone_r5_phase101_epoch_clock_vector_structural_result_v1.json"
PHASE101_FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_phase101_source_parity_accuracy_regression_freeze_v1.json"
PHASE82_RESULT = ROOT / "docs/use_cases/records/smartphone_r5_phase82_phase80_direct_quality_accuracy_result_v1.json"
PHASE100_RESULT = ROOT / "docs/use_cases/records/smartphone_r5_phase100_phase99_qr_accuracy_result_v1.json"
PHASE103_RESULT = ROOT / "docs/use_cases/records/smartphone_r5_phase103_phase102_truth_only_evaluator_correction_result_v1.json"
APP = ROOT / "apps/native/gnss_fgo_imu_no_base.cpp"
BACKEND = ROOT / "src/algorithms/fgo_gtsam_backend.cpp"
INTERNAL = ROOT / "src/algorithms/fgo_gtsam_internal.hpp"
FGO = ROOT / "include/libgnss++/algorithms/fgo.hpp"
CONFIG = ROOT / "include/libgnss++/algorithms/fgo_config.hpp"
BINARY = ROOT / "build/apps/gnss_fgo_imu_no_base"

FREEZE_SHA = "0d441945897c01899c9ed6cbf0659ee0382c778c35a8e2056315168a27d526dd"
FREEZE_COMMIT = "f201712"
IMPLEMENTATION_COMMIT = "88f726a02d98e6dddd84e561b427ded43368e61d"
APP_SHA = "c81381b0a8fa8cea5c101fc5d54e6364173c0e8cffb83f02eac18c8b6cafde5b"
BACKEND_SHA = "781d65e34826ec08e04d1dfdac0ab10b2ed9409726078465f3020b57afac4baa"
INTERNAL_SHA = "cc6327e36800afa8217e5a834260adc81c0a74a5aefbfc956bb4679c8d86105f"
FGO_SHA = "5a26994bda96c4bf50f436b1368bc1df5b192c38ef13a2a882ebd8e15bf5103d"
CONFIG_SHA = "3cc60abe514ef8900012064accb17f27d3927d0b8a8776ad76ce6aefa83ce32c"
BINARY_SHA = "6c2fbf9ed119e84383beb3b7db7f11cf1bce8db7a7e64919a38378373784c5c4"
PHASE95_WRAPPER_SHA = "5ac858cf156555652f6feae60c71a4237269d4b4c2cb70c1adf5a9e08f10e883"
PHASE95_RESULT_SHA = "beba25d4b0d910064d76bea7aa6b7a884937ca01c3b00cf0f7f9f4b6267ea281"

ROUTES = (
    "2021-03-16-18-59-us-ca-mtv-a/pixel5",
    "2022-04-01-18-22-us-ca-lax-t/pixel5",
)
DOMAIN_ROWS = {ROUTES[0]: 2158, ROUTES[1]: 1465}
PROBLEM_EPOCHS = {route: rows + 1 for route, rows in DOMAIN_ROWS.items()}
RAW_NAMES = ("device_gnss.csv", "device_imu.csv", "brdc.nav")
RAW_FLAGS = (
    ("--android-gnss", "device_gnss.csv", "__PHASE95_RAW_DEVICE_GNSS__"),
    ("--android-imu", "device_imu.csv", "__PHASE95_RAW_DEVICE_IMU__"),
    ("--nav", "brdc.nav", "__PHASE95_RAW_BROADCAST_NAV__"),
)
PHASE93_SELECTOR = "--native-source-clock-c0d-gnss-first-meter-state-handoff"
VECTOR_SELECTOR = "--native-source-clock-c0d-epoch-vector-parity"
QR_SELECTOR = "--native-source-clock-c0d-phase99-main-multifrontal-qr-solver"
PHASE104_SELECTOR = "--native-phase104-stage-main-attribution"
STAGE_FLAG = "--phase104-stage-ecef"
DISPLACEMENT_FLAG = "--phase104-main-displacement-stats"
OUTPUT_RELATIVE_ROOT = "output/smartphone-r5/phase104-stage-main-attribution-v1/"
MANIFEST_SCHEMA = "smartphone-r5-phase104-stage-main-attribution-manifest.v1"
AUTH_SCHEMA = "smartphone-r5-phase104-stage-main-attribution-raw-execution-authorization.v1"
TRUTH_AUTH_SCHEMA = "smartphone-r5-phase104-stage-main-attribution-truth-only-authorization.v1"
RESULT_SCHEMA = "smartphone-r5-phase104-stage-main-attribution-result.v1"
CANDIDATE_ID = "phase104-evaluation-only-gnss-first-stage-export-v1"
STAGE_HEADER = [
    "phone",
    "UnixTimeMillis",
    "PositionEcefX_m",
    "PositionEcefY_m",
    "PositionEcefZ_m",
]
MAIN_HEADER = ["phone", "UnixTimeMillis", "LatitudeDegrees", "LongitudeDegrees"]
MAX_SPEED_MPS = 70.0
EARTH_RADIUS_M = 6371008.8
WGS84_A_M = 6378137.0
WGS84_E2 = 6.6943799901413165e-3


class Phase104Error(ValueError):
    """Raised when a Phase104 contract or gate fails closed."""


def fail(message: str) -> Phase104Error:
    return Phase104Error(message)


def relative(path: Path) -> str:
    try:
        return str(path.relative_to(ROOT))
    except ValueError:
        return str(path)


def _forbidden_path(path: Path | str) -> bool:
    token = str(path).lower()
    return any(term in token for term in (
        ".mat", "truth", "ground_truth", "validation", "holdout", "kaggle",
        "token", "base.rinex", "precomputed", "coordinate", "pdc",
    ))


def _raw_file_name(path: Path | str) -> bool:
    return Path(path).name in RAW_NAMES


def sha256_file(path: Path, label: str, *, allow_truth: bool = False) -> str:
    if _raw_file_name(path) or (_forbidden_path(path) and not allow_truth):
        raise fail(f"forbidden file hash: {label}: {path}")
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


def read_json(path: Path, label: str, *, allow_truth: bool = False) -> dict[str, Any]:
    if _forbidden_path(path) and not allow_truth:
        raise fail(f"forbidden JSON path: {label}: {path}")
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise fail(f"failed to read {label}: {path}: {exc}") from exc
    if not isinstance(value, dict):
        raise fail(f"{label} is not an object")
    return value


def atomic_json(path: Path, value: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".tmp", dir=path.parent
    )
    try:
        with open(descriptor, "w", encoding="utf-8", closefd=True) as handle:
            json.dump(value, handle, indent=2, sort_keys=True, allow_nan=False)
            handle.write("\n")
            handle.flush()
        Path(temporary).replace(path)
    except BaseException:
        try:
            Path(temporary).unlink()
        except OSError:
            pass
        raise


def atomic_text(path: Path, value: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".tmp", dir=path.parent
    )
    try:
        with open(descriptor, "w", encoding="utf-8", closefd=True) as handle:
            handle.write(value)
            handle.flush()
        Path(temporary).replace(path)
    except BaseException:
        try:
            Path(temporary).unlink()
        except OSError:
            pass
        raise


def assert_equal(actual: Any, expected: Any, label: str) -> None:
    if actual != expected:
        raise fail(f"{label}: expected {expected!r}, got {actual!r}")


def _expected_output(route: str, name: str) -> str:
    return f"{OUTPUT_RELATIVE_ROOT}{route.replace('/', '__')}/{name}"


def _finite(value: Any) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(float(value))


def verify_freeze() -> dict[str, Any]:
    assert_equal(sha256_file(FREEZE, "Phase104 freeze"), FREEZE_SHA, "freeze/sha256")
    freeze = read_json(FREEZE, "Phase104 freeze")
    for key, expected in {
        "schema_version": "smartphone-r5-phase104-stage-main-accuracy-attribution-freeze.v1",
        "phase": 104,
        "execution_label": "Luna Max",
        "status": "frozen-before-phase104-implementation-raw-and-truth-execution",
    }.items():
        assert_equal(freeze.get(key), expected, f"freeze/{key}")
    candidate = freeze.get("candidate")
    if not isinstance(candidate, dict):
        raise fail("freeze/candidate missing")
    for key, expected in {
        "candidate_count": 1,
        "id": CANDIDATE_ID,
        "selector_default_off": True,
        "routes": list(ROUTES),
        "route_order_fixed": True,
        "runs_per_route": 1,
        "fresh_phase101_pipeline_required": True,
        "copy_only": True,
        "stage_copy_may_feed_solver": False,
        "stage_copy_may_replace_handoff": False,
        "stage_copy_may_be_used_for_initialization": False,
        "stage_copy_may_be_used_for_tuning": False,
        "native_input_or_intermediate_mutation": False,
        "legacy_default_unchanged": True,
        "no_fallback_or_rerun": True,
    }.items():
        assert_equal(candidate.get(key), expected, f"freeze/candidate/{key}")
    raw = freeze.get("raw_only_boundary")
    if not isinstance(raw, dict):
        raise fail("freeze/raw_only_boundary missing")
    assert_equal(raw.get("future_native_inputs_exact"), list(RAW_NAMES), "freeze/raw inputs")
    assert_equal(raw.get("raw_content_copy_or_transform"), False, "freeze/raw copy")
    sidecar = freeze.get("stage_sidecar_contract")
    if not isinstance(sidecar, dict):
        raise fail("freeze/stage_sidecar_contract missing")
    for key, expected in {
        "private_evaluation_only": True,
        "key": "(phone, UnixTimeMillis)",
        "retained_key_contract": "exact retained EpochSeed source/UTC order; key equality is required and row position is not a substitute",
        "interpolation_allowed": False,
        "edge_hold_allowed": False,
        "unresolved_allowed": False,
        "duplicate_keys_allowed": False,
    }.items():
        assert_equal(sidecar.get(key), expected, f"freeze/stage/{key}")
    stats = freeze.get("main_displacement_diagnostic_contract")
    if not isinstance(stats, dict):
        raise fail("freeze/main_displacement_diagnostic_contract missing")
    assert_equal(stats.get("copy_only"), True, "freeze/displacement/copy")
    assert_equal(stats.get("coordinate_rows_exported"), False, "freeze/displacement/coordinates")
    metric = freeze.get("truth_only_attribution_contract", {}).get("metric")
    if not isinstance(metric, dict):
        raise fail("freeze/truth metric missing")
    for key, expected in {
        "key": "(phone, UnixTimeMillis)",
        "matching": "exact key intersection only",
        "distance": "spherical Haversine per row",
        "earth_radius_m": EARTH_RADIUS_M,
        "route_scalar": "(P50 + P95) / 2 in metres",
        "macro": "unweighted arithmetic mean over exactly MTV-A then LAX-T",
    }.items():
        assert_equal(metric.get(key), expected, f"freeze/metric/{key}")
    accounting = freeze.get("read_accounting_for_this_freeze")
    if not isinstance(accounting, dict):
        raise fail("freeze/read_accounting_for_this_freeze missing")
    for key in (
        "native_solver_invocations", "raw_gnss_reads", "raw_imu_reads",
        "broadcast_navigation_reads", "truth_reads", "accuracy_calculations",
        "MAT_reads_or_generation", "base_reads", "PDC_or_precomputed_coordinate_reads",
        "Kaggle_or_token_access", "reruns_or_tuning",
    ):
        assert_equal(accounting.get(key), 0, f"freeze/accounting/{key}")
    boundary = freeze.get("authorization_boundary")
    if not isinstance(boundary, dict):
        raise fail("freeze/authorization_boundary missing")
    for key in (
        "implementation_authorized_by_this_record", "raw_execution_authorized_by_this_record",
        "truth_evaluation_authorized_by_this_record", "solution_publication_authorized",
        "release_or_validation_authorized", "Kaggle_submission_authorized",
    ):
        assert_equal(boundary.get(key), False, f"freeze/boundary/{key}")
    return freeze


def verify_implementation() -> dict[str, Any]:
    pins = {
        APP: (APP_SHA, "Phase104 native source"),
        BACKEND: (BACKEND_SHA, "Phase101 GTSAM backend"),
        INTERNAL: (INTERNAL_SHA, "Phase101 key helpers"),
        FGO: (FGO_SHA, "Phase101 FGO result header"),
        CONFIG: (CONFIG_SHA, "Phase101 FGO config"),
        BINARY: (BINARY_SHA, "Phase104 native binary"),
    }
    digests: dict[str, Any] = {}
    for path, (expected, label) in pins.items():
        actual = sha256_file(path, label)
        assert_equal(actual, expected, f"implementation/{relative(path)}/sha256")
        digests[relative(path)] = actual
    source = APP.read_text(encoding="utf-8")
    required = (
        "--native-phase104-stage-main-attribution",
        "--phase104-stage-ecef",
        "--phase104-main-displacement-stats",
        "struct Phase104StageExportReport",
        "struct Phase104MainDisplacementReport",
        "exportPhase104StageEcef",
        "writePhase104MainDisplacementStats",
        "PositionEcefX_m",
        "phase104_stage_main_accuracy_attribution",
    )
    for token in required:
        if token not in source:
            raise fail(f"Phase104 implementation marker missing: {token}")
    assert_equal(source.count("exportPhase104StageEcef("), 2, "implementation/stage export call count")
    assert_equal(source.count("writePhase104MainDisplacementStats("), 2, "implementation/displacement call count")
    if "native_phase104_stage_main_accuracy_attribution = false" not in source:
        raise fail("Phase104 selector is not default-off")
    # The Phase104 observer must occur after the existing D/C handoff assignment
    # and before main input construction; this is a source-order guard, not a
    # runtime launch.
    handoff = source.index("problem.native_source_clock_c0d_gnss_first_d_handoff_mps =")
    stage_call = source.index(
        "if (options.native_phase104_stage_main_accuracy_attribution)", handoff
    )
    build_input = source.index("bool use_imu = buildImuInput")
    if not (handoff < stage_call < build_input):
        raise fail("Phase104 stage observer is not after handoff and before main input")
    return {"paths": digests, "implementation_commit": IMPLEMENTATION_COMMIT}


def phase95_paths() -> dict[str, dict[str, dict[str, Any]]]:
    """Read sealed Phase95 path metadata without opening any raw file."""

    assert_equal(sha256_file(PHASE95_RESULT, "Phase95 corrected result"), PHASE95_RESULT_SHA, "Phase95 result/sha256")
    result = read_json(PHASE95_RESULT, "Phase95 corrected result")
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
            raise fail(f"Phase95 raw roles changed: {route}")
        resolved[route] = {}
        for name in RAW_NAMES:
            pin = raw[name]
            if not isinstance(pin, dict) or not isinstance(pin.get("path"), str):
                raise fail(f"Phase95 raw path missing: {route}/{name}")
            path_text = pin["path"]
            path = Path(path_text)
            if path.is_absolute() or ".." in path.parts or path.name != name:
                raise fail(f"unsafe Phase95 raw path: {route}/{name}")
            if pin.get("exists_before_launch") is not True:
                raise fail(f"Phase95 raw path was absent: {route}/{name}")
            if pin.get("read_by_runner") is not False:
                raise fail(f"Phase95 runner raw-read policy changed: {route}/{name}")
            digest = pin.get("sha256")
            if not isinstance(digest, str) or len(digest) != 64:
                raise fail(f"Phase95 raw SHA metadata missing: {route}/{name}")
            resolved[route][name] = {
                "path": path_text,
                "sha256": digest,
                "bytes": pin.get("bytes"),
                "read_by_runner": False,
                "hash_read_by_runner": False,
            }
    return resolved


def _validate_output_token(token: str) -> None:
    # Option names can legitimately describe the raw-only policy (for
    # example --native-pdc-imu-tdcp-no-bridge); forbidden-token checks apply
    # to paths and values, while the explicit forbidden option list below
    # handles disallowed switches.
    if token.startswith("--"):
        return
    if _raw_file_name(token) or _forbidden_path(token):
        raise fail(f"forbidden command/path token: {token}")


def validate_command(route: str, command: Any, record: dict[str, Any], *, raw: dict[str, Any] | None = None) -> None:
    if not isinstance(command, list) or not command or not all(isinstance(token, str) for token in command):
        raise fail(f"malformed Phase104 command: {route}")
    if command[0] != "build/apps/gnss_fgo_imu_no_base":
        raise fail(f"native binary changed: {route}")
    raw_values = {
        pin.get("path")
        for pin in (raw or {}).values()
        if isinstance(pin, dict) and isinstance(pin.get("path"), str)
    }
    for token in command:
        # Raw input paths are the only command tokens whose basename is one
        # of the three permitted raw names.  They are admitted here only
        # after the caller has materialized them from the sealed Phase95 path
        # record; every other command/path token remains fail-closed.
        if token not in raw_values:
            _validate_output_token(token)
    required_flags = (
        "--all-epochs", "--android-raw-utc-keys", "--android-raw-clock-only",
        "--android-utc-wall-clock-fallback", "--native-pdc-imu-tdcp-no-bridge",
        "--native-source-direct-observable-quality", "--native-source-clock-c0d-factor",
        "--native-source-clock-c0d-meter-state-parity",
        "--native-source-clock-c0d-active-solve-diagnostic", PHASE93_SELECTOR,
        VECTOR_SELECTOR, QR_SELECTOR, PHASE104_SELECTOR, STAGE_FLAG, DISPLACEMENT_FLAG,
    )
    for flag in required_flags:
        assert_equal(command.count(flag), 1, f"command/{route}/{flag}")
    for forbidden in (
        "--obs", "--native-base-rinex", "--native-base-pseudorange-compensation",
        "--native-base-pseudorange-source-miss-mask", "--native-pdc-state-bridge",
        "--native-upstream-quality", "--native-gnss-first-velocity-only-handoff",
        "--native-direct-doppler-wls-handoff", "--native-source-clock-c0d-phase94-stage-diagnostics",
        "--native-source-clock-c0d-phase96-main-diagnostics",
        "--native-source-clock-c0d-phase97-singular-system-diagnostics",
        "--native-source-clock-c0d-phase98-solver-rank-diagnostic",
    ):
        assert_equal(command.count(forbidden), 0, f"command/{route}/forbidden/{forbidden}")
    assert_equal(command.count("--dataset-id"), 1, f"command/{route}/dataset-id count")
    assert_equal(command[command.index("--dataset-id") + 1], route, f"command/{route}/dataset-id")
    for flag, name, placeholder in RAW_FLAGS:
        assert_equal(command.count(flag), 1, f"command/{route}/{flag}")
        value = command[command.index(flag) + 1]
        expected = raw[name]["path"] if raw is not None else placeholder
        assert_equal(value, expected, f"command/{route}/{name}/path")
    assert_equal(command.count("--out"), 1, f"command/{route}/out count")
    assert_equal(command[command.index("--out") + 1], record["planned_output"]["solution"], f"command/{route}/out")
    assert_equal(command.count("--summary-json"), 1, f"command/{route}/summary count")
    assert_equal(command[command.index("--summary-json") + 1], record["planned_output"]["summary"], f"command/{route}/summary")
    assert_equal(command[command.index(STAGE_FLAG) + 1], record["planned_output"]["stage_ecef"], f"command/{route}/stage")
    assert_equal(command[command.index(DISPLACEMENT_FLAG) + 1], record["planned_output"]["displacement_stats"], f"command/{route}/stats")


def verify_manifest() -> dict[str, Any]:
    verify_freeze()
    implementation = verify_implementation()
    sealed_raw = phase95_paths()
    manifest = read_json(MANIFEST, "Phase104 manifest")
    for key, expected in {
        "schema_version": MANIFEST_SCHEMA,
        "phase": 104,
        "execution_label": "Luna Max",
        "status": "sealed-before-phase104-raw-execution",
    }.items():
        assert_equal(manifest.get(key), expected, f"manifest/{key}")
    freeze = manifest.get("freeze")
    if not isinstance(freeze, dict):
        raise fail("manifest/freeze missing")
    for key, expected in {
        "path": relative(FREEZE),
        "sha256": FREEZE_SHA,
        "commit": FREEZE_COMMIT,
        "raw_execution_authorized_before_this_manifest": False,
    }.items():
        assert_equal(freeze.get(key), expected, f"manifest/freeze/{key}")
    impl = manifest.get("implementation")
    if not isinstance(impl, dict):
        raise fail("manifest/implementation missing")
    for key, expected in {
        "commit": IMPLEMENTATION_COMMIT,
        "candidate_id": CANDIDATE_ID,
        "solver_filter_lm_changed": False,
        "equation_units_sigma_changed": False,
        "legacy_default_unchanged": True,
        "fallback_or_guard_bypass": False,
    }.items():
        assert_equal(impl.get(key), expected, f"manifest/implementation/{key}")
    for key, path, digest in (
        ("source", APP, APP_SHA), ("backend", BACKEND, BACKEND_SHA),
        ("key_helpers", INTERNAL, INTERNAL_SHA), ("fgo_result", FGO, FGO_SHA),
        ("config", CONFIG, CONFIG_SHA), ("binary", BINARY, BINARY_SHA),
    ):
        pin = impl.get(key)
        if not isinstance(pin, dict):
            raise fail(f"manifest/implementation/{key} missing")
        assert_equal(pin.get("path"), relative(path), f"manifest/implementation/{key}/path")
        assert_equal(pin.get("sha256"), digest, f"manifest/implementation/{key}/sha256")
    phase101 = manifest.get("phase101_authority")
    if not isinstance(phase101, dict):
        raise fail("manifest/phase101_authority missing")
    assert_equal(phase101.get("result_path"), relative(PHASE101_RESULT), "manifest/phase101/result path")
    assert_equal(phase101.get("freeze_path"), relative(PHASE101_FREEZE), "manifest/phase101/freeze path")
    for key, path in (("result_sha256", PHASE101_RESULT), ("freeze_sha256", PHASE101_FREEZE)):
        assert_equal(phase101.get(key), sha256_file(path, f"Phase101 authority {key}"), f"manifest/phase101/{key}")
    sources = manifest.get("source_only_authorities")
    if not isinstance(sources, dict):
        raise fail("manifest/source_only_authorities missing")
    for key, path in (("phase82_result", PHASE82_RESULT), ("phase100_result", PHASE100_RESULT), ("phase103_result", PHASE103_RESULT), ("phase95_wrapper", PHASE95_WRAPPER)):
        pin = sources.get(key)
        if not isinstance(pin, dict):
            raise fail(f"manifest/source authority missing: {key}")
        assert_equal(pin.get("path"), relative(path), f"manifest/source/{key}/path")
        # Phase103 is a sealed aggregate JSON authority whose filename carries
        # the word "truth"; hashing this metadata record is permitted and is
        # not a read of any truth payload.
        assert_equal(
            pin.get("sha256"),
            sha256_file(
                path,
                f"source authority {key}",
                allow_truth=(path == PHASE103_RESULT),
            ),
            f"manifest/source/{key}/sha256",
        )
    for key, path in (("pre_raw_evaluator", EVALUATOR), ("execution_wrapper", WRAPPER), ("focused_tests", FOCUSED_TESTS)):
        pin = manifest.get(key)
        if not isinstance(pin, dict):
            raise fail(f"manifest/{key} missing")
        assert_equal(pin.get("path"), relative(path), f"manifest/{key}/path")
        assert_equal(pin.get("sha256"), sha256_file(path, f"Phase104 {key}"), f"manifest/{key}/sha256")
    candidate = manifest.get("candidate")
    if not isinstance(candidate, dict):
        raise fail("manifest/candidate missing")
    for key, expected in {
        "id": CANDIDATE_ID, "candidate_count": 1, "opt_in": True,
        "default_off_outside_this_command": True, "raw_only": True,
        "evaluation_only": True, "fresh_phase101_pipeline": True,
        "runs_per_route": 1, "controls": 0, "truth_evaluation": False,
        "accuracy_scoring": False, "solution_output_publication": False,
        "stage_sidecar_coordinate_format": "ECEF metres; evaluator-only geodetic conversion",
        "stage_sidecar_reinput": False, "main_displacement_coordinate_rows": False,
        "no_solver_filter_lm_change": True, "no_equation_unit_sigma_change": True,
        "no_fallback_or_guard_bypass": True, "no_raw_content_copy_or_transform": True,
    }.items():
        assert_equal(candidate.get(key), expected, f"manifest/candidate/{key}")
    assert_equal(candidate.get("selectors"), [PHASE93_SELECTOR, VECTOR_SELECTOR, QR_SELECTOR, PHASE104_SELECTOR], "manifest/selectors")
    assert_equal(candidate.get("routes"), list(ROUTES), "manifest/routes")
    raw_contract = manifest.get("raw_input_contract")
    if not isinstance(raw_contract, dict):
        raise fail("manifest/raw_input_contract missing")
    assert_equal(raw_contract.get("input_names_exact"), list(RAW_NAMES), "manifest/raw names")
    assert_equal(raw_contract.get("truth_mat_base_pdc_precomputed_coordinate_kaggle_accuracy"), False, "manifest/raw exclusions")
    assert_equal(raw_contract.get("copy_or_transform"), False, "manifest/raw copy")
    records = manifest.get("routes")
    if not isinstance(records, list) or [item.get("dataset_id") for item in records] != list(ROUTES):
        raise fail("manifest route order/count changed")
    for record in records:
        route = record.get("dataset_id")
        if route not in DOMAIN_ROWS:
            raise fail(f"unknown manifest route: {route}")
        assert_equal(record.get("domain_rows"), DOMAIN_ROWS[route], f"manifest/{route}/domain_rows")
        assert_equal(record.get("expected_problem_epochs"), PROBLEM_EPOCHS[route], f"manifest/{route}/problem_epochs")
        expected_raw = sealed_raw[route]
        for name in RAW_NAMES:
            pin = record.get("raw_inputs", {}).get(name)
            if not isinstance(pin, dict):
                raise fail(f"manifest raw pin missing: {route}/{name}")
            expected_placeholder = {
                "device_gnss.csv": "__PHASE95_RAW_DEVICE_GNSS__",
                "device_imu.csv": "__PHASE95_RAW_DEVICE_IMU__",
                "brdc.nav": "__PHASE95_RAW_BROADCAST_NAV__",
            }[name]
            assert_equal(pin.get("path"), expected_placeholder, f"manifest raw placeholder {route}/{name}")
            assert_equal(pin.get("path_source"), "Phase95 sealed result raw_inputs.path", f"manifest raw source {route}/{name}")
            assert_equal(pin.get("sha256"), None, f"manifest raw digest must remain unread {route}/{name}")
            assert_equal(pin.get("read_at_manifest_creation"), False, f"manifest raw read {route}/{name}")
            if expected_raw[name]["path"] == pin.get("path"):
                raise fail(f"manifest accidentally contains a materialized raw path: {route}/{name}")
        validate_command(route, record.get("command"), record)
    matrix = manifest.get("matrix")
    if not isinstance(matrix, dict):
        raise fail("manifest/matrix missing")
    for key, expected in {
        "candidate_count": 1, "route_count": 2, "runs_per_route": 1,
        "native_invocations_planned": 2, "truth_reads_planned": 0,
        "accuracy_calculations_planned": 0, "reruns": 0, "fallbacks": 0,
        "solution_rows_authorized": True, "stage_sidecar_seals_planned": 2,
        "main_displacement_stats_seals_planned": 2,
    }.items():
        assert_equal(matrix.get(key), expected, f"manifest/matrix/{key}")
    accounting = manifest.get("read_accounting_before_execution")
    if not isinstance(accounting, dict):
        raise fail("manifest/read_accounting_before_execution missing")
    for key in (
        "native_solver_invocations", "raw_device_gnss_reads", "raw_device_imu_reads",
        "broadcast_navigation_reads", "truth_reads", "accuracy_calculations",
        "mat_reads_or_generated", "base_rinex_reads", "precomputed_coordinate_reads",
        "kaggle_or_token_access", "raw_input_hash_reads", "reruns", "fallbacks",
    ):
        assert_equal(accounting.get(key), 0, f"manifest/accounting/{key}")
    assert_equal(accounting.get("raw_content_copied_or_transformed"), False, "manifest/accounting/copy")
    release = manifest.get("release_boundary")
    if not isinstance(release, dict):
        raise fail("manifest/release_boundary missing")
    for key in ("raw_execution_authorized", "truth_evaluation_authorized", "accuracy_promotion_authorized", "Kaggle_submission_authorized"):
        assert_equal(release.get(key), False, f"manifest/release/{key}")
    return manifest


def verify_authorization(manifest: dict[str, Any] | None = None) -> dict[str, Any]:
    manifest = manifest or verify_manifest()
    authorization = read_json(AUTHORIZATION, "Phase104 raw authorization")
    for key, expected in {
        "schema_version": AUTH_SCHEMA,
        "phase": 104,
        "execution_label": "Luna Max",
        "status": "authorized-for-exact-two-route-phase104-stage-main-raw-execution",
    }.items():
        assert_equal(authorization.get(key), expected, f"authorization/{key}")
    authority = authorization.get("authority")
    if not isinstance(authority, dict):
        raise fail("authorization/authority missing")
    for key, path, digest in (
        ("phase104_freeze", FREEZE, FREEZE_SHA),
        ("phase104_manifest", MANIFEST, sha256_file(MANIFEST, "Phase104 manifest")),
        ("phase104_evaluator", EVALUATOR, sha256_file(EVALUATOR, "Phase104 evaluator")),
        ("phase104_wrapper", WRAPPER, sha256_file(WRAPPER, "Phase104 wrapper")),
        ("phase104_focused_tests", FOCUSED_TESTS, sha256_file(FOCUSED_TESTS, "Phase104 focused tests")),
        ("phase95_result", PHASE95_RESULT, PHASE95_RESULT_SHA),
        ("phase95_wrapper", PHASE95_WRAPPER, PHASE95_WRAPPER_SHA),
    ):
        pin = authority.get(key)
        if not isinstance(pin, dict):
            raise fail(f"authorization/{key} missing")
        assert_equal(pin.get("path"), relative(path), f"authorization/{key}/path")
        assert_equal(pin.get("sha256"), digest, f"authorization/{key}/sha256")
    for key, expected in {
        "implementation_commit": IMPLEMENTATION_COMMIT,
        "implementation_source_sha256": APP_SHA,
        "implementation_binary_sha256": BINARY_SHA,
        "manifest_sha256": sha256_file(MANIFEST, "Phase104 manifest"),
    }.items():
        assert_equal(authority.get(key), expected, f"authorization/authority/{key}")
    assert_equal(authorization.get("routes"), list(ROUTES), "authorization/routes")
    candidate = authorization.get("candidate")
    if not isinstance(candidate, dict):
        raise fail("authorization/candidate missing")
    for key, expected in {
        "id": CANDIDATE_ID, "candidate_count": 1,
        "selectors": [PHASE93_SELECTOR, VECTOR_SELECTOR, QR_SELECTOR, PHASE104_SELECTOR],
        "raw_input_names_exact": list(RAW_NAMES), "truth_mat_base_pdc_precomputed_coordinate_kaggle_accuracy": False,
        "stage_sidecar_reinput": False, "runs_per_route": 1, "controls": 0,
        "truth_evaluation": False, "accuracy_calculations": 0,
        "solution_output_private": True, "no_fallback_or_rerun": True,
    }.items():
        assert_equal(candidate.get(key), expected, f"authorization/candidate/{key}")
    policy = authorization.get("execution_policy")
    if not isinstance(policy, dict):
        raise fail("authorization/execution_policy missing")
    for key, expected in {
        "authorized": True, "order": "MTV-A then LAX-T, sequentially",
        "one_invocation_per_route": True, "stop_after_two_routes": True,
        "truth_or_accuracy_evaluation": False, "no_rerun": True,
        "solver_filter_lm_changes": False, "fallback_or_guard_bypass": False,
    }.items():
        assert_equal(policy.get(key), expected, f"authorization/policy/{key}")
    boundary = authorization.get("release_boundary")
    if not isinstance(boundary, dict):
        raise fail("authorization/release_boundary missing")
    for key, expected in {
        "raw_execution_authorized": True,
        "truth_evaluation_authorized": False,
        "solution_publication_authorized": False,
        "Kaggle_submission_authorized": False,
    }.items():
        assert_equal(boundary.get(key), expected, f"authorization/boundary/{key}")
    return authorization


def verify_pre_raw() -> dict[str, Any]:
    verify_freeze()
    manifest = verify_manifest()
    source = EVALUATOR.read_text(encoding="utf-8")
    # This module is launch-free by design.  The execution wrapper owns the
    # only process-launch boundary and is independently pinned in the manifest.
    launch_tokens = (
        "sub" + "process.run",
        "sub" + "process.Popen",
        "os." + "system",
        "os." + "popen",
        "Popen" + "(",
    )
    for token in launch_tokens:
        if token in source:
            raise fail(f"pre-raw evaluator contains launch token: {token}")
    authorized = False
    auth_status = "not-issued-before-separate-raw-authorization"
    if AUTHORIZATION.is_file():
        authorization = verify_authorization(manifest)
        authorized = authorization.get("status") == "authorized-for-exact-two-route-phase104-stage-main-raw-execution"
        auth_status = "authorized" if authorized else "invalid"
    return {
        "status": "pre-raw-verified",
        "phase": 104,
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
        "base_reads": 0,
        "precomputed_coordinate_reads": 0,
        "accuracy_calculations": 0,
        "solution_output_published": False,
        "stage_sidecar_reinput": False,
        "route_score_selection": False,
        "freeze_sha256": FREEZE_SHA,
        "manifest_sha256": sha256_file(MANIFEST, "Phase104 manifest"),
        "implementation_commit": IMPLEMENTATION_COMMIT,
        "authorization_status": auth_status,
    }


def _file_metadata(path: Path, label: str) -> dict[str, Any]:
    if not path.is_file():
        return {"present": False, "path": relative(path)}
    return {
        "present": True,
        "path": relative(path),
        "bytes": path.stat().st_size,
        "sha256": sha256_file(path, label),
    }


def _read_csv_rows(path: Path, label: str) -> tuple[list[str], list[dict[str, str]]]:
    try:
        with path.open("r", encoding="utf-8", newline="") as handle:
            reader = csv.DictReader(handle)
            header = list(reader.fieldnames or [])
            rows = list(reader)
    except (OSError, UnicodeDecodeError, csv.Error) as exc:
        raise fail(f"failed to read {label}: {path}: {exc}") from exc
    return header, rows


def _parse_int(text: Any, label: str) -> int:
    try:
        if isinstance(text, bool):
            raise ValueError
        value = int(str(text))
    except (TypeError, ValueError) as exc:
        raise fail(f"{label} is not an integer: {text!r}") from exc
    return value


def _parse_float(text: Any, label: str) -> float:
    try:
        value = float(str(text))
    except (TypeError, ValueError) as exc:
        raise fail(f"{label} is not numeric: {text!r}") from exc
    if not math.isfinite(value):
        raise fail(f"{label} is nonfinite")
    return value


def _earth_valid_ecef(x: float, y: float, z: float) -> bool:
    norm = math.sqrt(x * x + y * y + z * z)
    return math.isfinite(norm) and 6.0e6 <= norm <= 7.0e6


def _ecef_to_lat_lon(x: float, y: float, z: float) -> tuple[float, float]:
    if not _earth_valid_ecef(x, y, z):
        raise fail("stage ECEF is not finite or Earth-valid")
    lon = math.atan2(y, x)
    p = math.hypot(x, y)
    lat = math.atan2(z, p * (1.0 - WGS84_E2))
    for _ in range(10):
        sin_lat = math.sin(lat)
        n = WGS84_A_M / math.sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat)
        height = p / max(math.cos(lat), 1.0e-15) - n
        next_lat = math.atan2(z, p * (1.0 - WGS84_E2 * n / (n + height)))
        if abs(next_lat - lat) < 1.0e-14:
            lat = next_lat
            break
        lat = next_lat
    return math.degrees(lat), math.degrees(lon)


def _validate_main_output(path: Path, route: str, expected_rows: int) -> dict[str, Any]:
    header, rows = _read_csv_rows(path, f"main solution {route}")
    if header != MAIN_HEADER:
        raise fail(f"main solution header mismatch: {route}: {header}")
    if len(rows) != expected_rows:
        raise fail(f"main solution row count mismatch: {route}: {len(rows)} != {expected_rows}")
    previous: int | None = None
    keys: list[tuple[str, int]] = []
    coordinates: list[tuple[float, float]] = []
    for index, row in enumerate(rows):
        phone = row.get("phone")
        if phone != route:
            raise fail(f"main solution route identity mismatch: {route}/{index}")
        timestamp = _parse_int(row.get("UnixTimeMillis"), f"main timestamp {route}/{index}")
        if previous is not None and timestamp <= previous:
            raise fail(f"main timestamps are not strictly increasing: {route}/{index}")
        previous = timestamp
        lat = _parse_float(row.get("LatitudeDegrees"), f"main latitude {route}/{index}")
        lon = _parse_float(row.get("LongitudeDegrees"), f"main longitude {route}/{index}")
        if not -90.0 <= lat <= 90.0 or not -180.0 <= lon <= 180.0:
            raise fail(f"main coordinate outside Earth domain: {route}/{index}")
        keys.append((route, timestamp))
        coordinates.append((lat, lon))
    if len(set(keys)) != len(keys):
        raise fail(f"duplicate main keys: {route}")
    return {"keys": keys, "coordinates": coordinates, "rows": len(rows), "header": header}


def _validate_stage_output(path: Path, route: str, expected_rows: int) -> dict[str, Any]:
    header, rows = _read_csv_rows(path, f"GNSS-first stage ECEF sidecar {route}")
    if header != STAGE_HEADER:
        raise fail(f"stage sidecar header mismatch: {route}: {header}")
    if len(rows) != expected_rows:
        raise fail(f"stage sidecar row count mismatch: {route}: {len(rows)} != {expected_rows}")
    previous: int | None = None
    keys: list[tuple[str, int]] = []
    coordinates: list[tuple[float, float]] = []
    for index, row in enumerate(rows):
        phone = row.get("phone")
        if phone != route:
            raise fail(f"stage sidecar route identity mismatch: {route}/{index}")
        timestamp = _parse_int(row.get("UnixTimeMillis"), f"stage timestamp {route}/{index}")
        if previous is not None and timestamp <= previous:
            raise fail(f"stage timestamps are not strictly increasing: {route}/{index}")
        previous = timestamp
        x = _parse_float(row.get("PositionEcefX_m"), f"stage X {route}/{index}")
        y = _parse_float(row.get("PositionEcefY_m"), f"stage Y {route}/{index}")
        z = _parse_float(row.get("PositionEcefZ_m"), f"stage Z {route}/{index}")
        coordinates.append(_ecef_to_lat_lon(x, y, z))
        keys.append((route, timestamp))
    if len(set(keys)) != len(keys):
        raise fail(f"duplicate stage keys: {route}")
    return {"keys": keys, "coordinates": coordinates, "rows": len(rows), "header": header}


def _validate_displacement_stats(path: Path, route: str) -> dict[str, Any]:
    value = read_json(path, f"main displacement stats {route}")
    if value.get("schema_version") != "smartphone-r5-phase104-main-displacement-stats.v1":
        raise fail(f"displacement stats schema mismatch: {route}")
    if value.get("diagnostic_only") is not True or value.get("coordinate_rows_exported") is not False:
        raise fail(f"displacement stats is not compact/diagnostic-only: {route}")
    for key in ("solution_epoch_count", "transition_count", "finite_transition_count", "nonfinite_transition_count", "over_70_mps_count"):
        if not isinstance(value.get(key), int):
            raise fail(f"displacement stats integer field missing: {route}/{key}")
    for key in ("p50_displacement_m", "p95_displacement_m", "max_displacement_m", "max_speed_mps"):
        if not _finite(value.get(key)):
            raise fail(f"displacement stats nonfinite field: {route}/{key}")
    if value["nonfinite_transition_count"] != 0 or value["finite_transition_count"] != value["transition_count"]:
        raise fail(f"displacement stats finite transition gate failed: {route}")
    return value


def seal_solution_metadata(path: Path, route: str, expected_rows: int) -> dict[str, Any]:
    parsed = _validate_main_output(path, route, expected_rows)
    return {
        "path": relative(path),
        "sha256": sha256_file(path, f"main solution {route}"),
        "bytes": path.stat().st_size,
        "rows": parsed["rows"],
        "header": parsed["header"],
        "pretruth_schema_valid": True,
        "solution_rows_published": False,
    }


def seal_stage_metadata(path: Path, route: str, expected_rows: int) -> dict[str, Any]:
    parsed = _validate_stage_output(path, route, expected_rows)
    return {
        "path": relative(path),
        "sha256": sha256_file(path, f"stage ECEF sidecar {route}"),
        "bytes": path.stat().st_size,
        "rows": parsed["rows"],
        "header": parsed["header"],
        "exact_retained_timestamp_domain": True,
        "finite_earth_valid": True,
        "stage_sidecar_reinput": False,
    }


def seal_stats_metadata(path: Path, route: str) -> dict[str, Any]:
    value = _validate_displacement_stats(path, route)
    return {
        "path": relative(path),
        "sha256": sha256_file(path, f"main displacement stats {route}"),
        "bytes": path.stat().st_size,
        "schema_version": value["schema_version"],
        "coordinate_rows_exported": False,
    }


def validate_run_artifacts(route: str, run: dict[str, Any]) -> dict[str, Any]:
    planned = run.get("planned_output")
    if not isinstance(planned, dict):
        raise fail(f"run planned output missing: {route}")
    solution = ROOT / planned["solution"]
    stage = ROOT / planned["stage_ecef"]
    stats = ROOT / planned["displacement_stats"]
    summary = ROOT / planned["summary"]
    expected = DOMAIN_ROWS[route]
    artifacts = {
        "solution": seal_solution_metadata(solution, route, expected),
        "stage": seal_stage_metadata(stage, route, expected),
        "displacement_stats": seal_stats_metadata(stats, route),
        "summary": _file_metadata(summary, f"native summary {route}"),
    }
    return artifacts


def _linear_percentile(values: list[float], quantile: float) -> float:
    ordered = sorted(values)
    if not ordered:
        raise fail("percentile input is empty")
    if len(ordered) == 1:
        return ordered[0]
    rank = (len(ordered) - 1) * quantile
    low = int(math.floor(rank))
    high = min(low + 1, len(ordered) - 1)
    return ordered[low] + (rank - low) * (ordered[high] - ordered[low])


def _haversine_m(a: tuple[float, float], b: tuple[float, float]) -> float:
    lat1, lon1 = map(math.radians, a)
    lat2, lon2 = map(math.radians, b)
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    h = math.sin(dlat / 2.0) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2.0) ** 2
    return 2.0 * EARTH_RADIUS_M * math.asin(math.sqrt(min(1.0, h)))


def _read_truth_once(path: Path, route: str, expected_rows: int, expected_missing: list[list[Any]] | None, expected_sha: str) -> tuple[dict[tuple[str, int], tuple[float, float]], dict[str, Any]]:
    """Read one truth file once and parse it from the bytes already read."""

    try:
        payload = path.read_bytes()
    except OSError as exc:
        raise fail(f"failed to read pinned truth {route}: {exc}") from exc
    digest = hashlib.sha256(payload).hexdigest()
    assert_equal(digest, expected_sha, f"truth/{route}/sha256")
    try:
        text = payload.decode("utf-8")
        reader = csv.DictReader(io.StringIO(text, newline=""))
        header = list(reader.fieldnames or [])
        rows = list(reader)
    except (UnicodeDecodeError, csv.Error) as exc:
        raise fail(f"failed to parse pinned truth {route}: {exc}") from exc
    for field in ("UnixTimeMillis", "LatitudeDegrees", "LongitudeDegrees"):
        if field not in header:
            raise fail(f"truth required field missing: {route}/{field}")
    if len(rows) != expected_rows:
        raise fail(f"truth row count mismatch: {route}: {len(rows)} != {expected_rows}")
    truth: dict[tuple[str, int], tuple[float, float]] = {}
    for index, row in enumerate(rows):
        phone = row.get("phone") or route
        if phone != route:
            raise fail(f"truth route identity mismatch: {route}/{index}")
        key = (route, _parse_int(row.get("UnixTimeMillis"), f"truth timestamp {route}/{index}"))
        if key in truth:
            raise fail(f"duplicate truth key: {route}/{index}")
        lat = _parse_float(row.get("LatitudeDegrees"), f"truth latitude {route}/{index}")
        lon = _parse_float(row.get("LongitudeDegrees"), f"truth longitude {route}/{index}")
        if not -90.0 <= lat <= 90.0 or not -180.0 <= lon <= 180.0:
            raise fail(f"truth coordinate outside Earth domain: {route}/{index}")
        truth[key] = (lat, lon)
    return truth, {
        "path": relative(path),
        "sha256": digest,
        "bytes": len(payload),
        "rows": len(rows),
        "read_count": 1,
        "expected_missing_keys": expected_missing or [],
    }


def _score_prediction(prediction: dict[str, Any], truth: dict[tuple[str, int], tuple[float, float]], route: str, allowed_missing: list[list[Any]] | None) -> dict[str, Any]:
    keys = prediction["keys"]
    coordinates = prediction["coordinates"]
    truth_keys = set(truth)
    prediction_keys = set(keys)
    extras = sorted(prediction_keys - truth_keys)
    if extras:
        raise fail(f"prediction has keys absent from truth: {route}")
    missing = sorted(truth_keys - prediction_keys)
    allowed = {tuple(item) for item in (allowed_missing or [])}
    if {tuple(item) for item in missing} != allowed:
        raise fail(f"truth missing-key policy failed: {route}: {missing}")
    if not keys:
        raise fail(f"empty prediction: {route}")
    distances = [_haversine_m(coordinate, truth[key]) for key, coordinate in zip(keys, coordinates)]
    if not all(math.isfinite(value) for value in distances):
        raise fail(f"nonfinite accuracy distance: {route}")
    speeds: list[float] = []
    for index in range(1, len(keys)):
        dt = (keys[index][1] - keys[index - 1][1]) / 1000.0
        if dt <= 0.0:
            raise fail(f"nonpositive prediction time interval: {route}")
        transition = _haversine_m(coordinates[index], coordinates[index - 1])
        speed = transition / dt
        if not math.isfinite(speed):
            raise fail(f"nonfinite prediction speed: {route}")
        speeds.append(speed)
    over_70 = sum(speed > MAX_SPEED_MPS for speed in speeds)
    p50 = _linear_percentile(distances, 0.50)
    p95 = _linear_percentile(distances, 0.95)
    return {
        "prediction_rows": len(keys),
        "matched_rows": len(distances),
        "truth_rows": len(truth),
        "missing_truth_rows": len(missing),
        "missing_truth_keys": [list(item) for item in missing],
        "prediction_domain_coverage": len(distances) / len(keys),
        "finite": True,
        "p50_m": p50,
        "p95_m": p95,
        "score_m": (p50 + p95) / 2.0,
        "max_m": max(distances),
        "max_speed_mps": max(speeds) if speeds else 0.0,
        "over_70_mps_count": over_70,
    }


def _truth_pin(route: str) -> tuple[Path, int, list[list[Any]], str]:
    if route == ROUTES[0]:
        return (
            ROOT / "output/smartphone-r5/phase29-no-bridge-train-eval-v1/truth/2021-03-16-18-59-us-ca-mtv-a/pixel5/ground_truth.csv",
            2159,
            [[route, 1615921153434]],
            "7c84ed6a80b1bbb08c0ffad57493513833b9d5474e22a43c5a44da82824ee22d",
        )
    return (
        ROOT / "output/smartphone-r5/phase44-pixel5-development-accuracy-v1/truth/2022-04-01-18-22-us-ca-lax-t/pixel5/ground_truth.csv",
        1465,
        [],
        "29e0861dd1ecb8865c10adab69396d98ed96618e8877b09d04aa8d671edf79e8",
    )


def verify_truth_authorization() -> dict[str, Any]:
    authorization = read_json(TRUTH_AUTHORIZATION, "Phase104 truth-only authorization", allow_truth=True)
    for key, expected in {
        "schema_version": TRUTH_AUTH_SCHEMA,
        "phase": 104,
        "execution_label": "Luna Max",
        "status": "authorized-for-phase104-truth-only-attribution-evaluation",
    }.items():
        assert_equal(authorization.get(key), expected, f"truth authorization/{key}")
    authority = authorization.get("authority")
    if not isinstance(authority, dict):
        raise fail("truth authorization/authority missing")
    for key, path in (("phase104_freeze", FREEZE), ("phase104_manifest", MANIFEST), ("phase104_raw_authorization", AUTHORIZATION), ("phase104_evaluator", EVALUATOR)):
        pin = authority.get(key)
        if not isinstance(pin, dict):
            raise fail(f"truth authorization authority missing: {key}")
        assert_equal(pin.get("path"), relative(path), f"truth authorization/{key}/path")
        digest = FREEZE_SHA if path == FREEZE else sha256_file(path, f"truth authorization {key}")
        assert_equal(pin.get("sha256"), digest, f"truth authorization/{key}/sha256")
    assert_equal(authorization.get("routes"), list(ROUTES), "truth authorization/routes")
    candidate = authorization.get("candidate")
    if not isinstance(candidate, dict):
        raise fail("truth authorization/candidate missing")
    for key, expected in {
        "id": CANDIDATE_ID, "candidate_count": 1, "runs_per_route": 1,
        "truth_reads_per_route": 1, "native_solver_invocations": 0,
        "truth_only": True, "solution_or_stage_reinput": False,
        "metric_unchanged": True, "kaggle_submission": False,
    }.items():
        assert_equal(candidate.get(key), expected, f"truth authorization/candidate/{key}")
    boundary = authorization.get("release_boundary")
    if not isinstance(boundary, dict):
        raise fail("truth authorization/release boundary missing")
    for key, expected in {
        "truth_evaluation_authorized": True,
        "native_rerun_authorized": False,
        "solution_publication_authorized": False,
        "Kaggle_submission_authorized": False,
    }.items():
        assert_equal(boundary.get(key), expected, f"truth authorization/boundary/{key}")
    return authorization


def _read_sealed_run(route: str, output_root: Path) -> tuple[dict[str, Any], dict[str, Any]]:
    run_path = output_root / route.replace("/", "__") / "run_metadata.json"
    run = read_json(run_path, f"Phase104 run metadata {route}")
    if run.get("dataset_id") != route or run.get("run_number") != 1:
        raise fail(f"invalid sealed run identity: {route}")
    if run.get("raw_byte_reads_by_wrapper") != 0 or run.get("raw_hash_reads_by_wrapper") != 0:
        raise fail(f"wrapper raw-read accounting failed: {route}")
    if run.get("truth_read_by_wrapper") is not False or run.get("solution_output_published") is not False:
        raise fail(f"wrapper truth/publication policy failed: {route}")
    artifacts = run.get("artifacts")
    if not isinstance(artifacts, dict):
        raise fail(f"sealed artifacts missing: {route}")
    for key in ("solution", "stage", "displacement_stats", "summary"):
        pin = artifacts.get(key)
        if not isinstance(pin, dict) or pin.get("present") is not True:
            raise fail(f"sealed artifact missing: {route}/{key}")
        path = ROOT / pin["path"]
        if sha256_file(path, f"sealed {key} {route}") != pin.get("sha256"):
            raise fail(f"sealed artifact changed after raw execution: {route}/{key}")
    return run, artifacts


def evaluate(output_root: Path, result_path: Path) -> dict[str, Any]:
    verify_freeze()
    manifest = verify_manifest()
    verify_truth_authorization()
    execution = read_json(output_root / "execution_metadata.json", "Phase104 execution metadata")
    if execution.get("native_invocations") != 2 or execution.get("truth_reads_by_wrapper") != 0:
        raise fail("Phase104 raw execution seal does not prove exactly two native/no-truth runs")
    route_runs: dict[str, dict[str, Any]] = {}
    route_artifacts: dict[str, dict[str, Any]] = {}
    for route in ROUTES:
        run, artifacts = _read_sealed_run(route, output_root)
        if run.get("return_code") != 0:
            raise fail(f"native route did not complete successfully: {route}")
        route_runs[route] = run
        route_artifacts[route] = artifacts
    # Candidate files are fully preflighted before either truth file is opened.
    parsed: dict[str, dict[str, Any]] = {}
    for route in ROUTES:
        artifacts = route_artifacts[route]
        expected = DOMAIN_ROWS[route]
        main_path = ROOT / artifacts["solution"]["path"]
        stage_path = ROOT / artifacts["stage"]["path"]
        stats_path = ROOT / artifacts["displacement_stats"]["path"]
        main = _validate_main_output(main_path, route, expected)
        stage = _validate_stage_output(stage_path, route, expected)
        stats = _validate_displacement_stats(stats_path, route)
        if main["keys"] != stage["keys"]:
            raise fail(f"stage/main exact retained timestamp domain mismatch: {route}")
        parsed[route] = {"main": main, "stage": stage, "stats": stats}
    route_scores: dict[str, dict[str, Any]] = {}
    truth_reads = 0
    for route in ROUTES:
        truth_path, truth_rows, missing, truth_sha = _truth_pin(route)
        truth, truth_meta = _read_truth_once(truth_path, route, truth_rows, missing, truth_sha)
        truth_reads += truth_meta["read_count"]
        main_score = _score_prediction(parsed[route]["main"], truth, route, missing)
        stage_score = _score_prediction(parsed[route]["stage"], truth, route, missing)
        route_scores[route] = {
            "main": main_score,
            "stage": stage_score,
            "truth": truth_meta,
            "artifact_hashes": {
                "main_solution": route_artifacts[route]["solution"]["sha256"],
                "stage_ecef": route_artifacts[route]["stage"]["sha256"],
                "main_displacement_stats": route_artifacts[route]["displacement_stats"]["sha256"],
                "summary": route_artifacts[route]["summary"].get("sha256"),
            },
        }
        baseline = 1.1139384500152307 if route == ROUTES[0] else 0.9389644134001871
        stage_value = stage_score["score_m"]
        main_value = main_score["score_m"]
        stage_regressed = stage_value > baseline
        main_added = main_value > stage_value
        if stage_regressed and main_added:
            attribution = "stage-regressed-and-main-amplified"
        elif stage_regressed:
            attribution = "stage-regression-present"
        elif main_added:
            attribution = "main-added-or-amplified-regression"
        elif main_value < stage_value:
            attribution = "main-improved-after-stage"
        else:
            attribution = "no-regression-detected"
        route_scores[route]["attribution"] = {
            "phase82_same_route_score_m": baseline,
            "stage_minus_main_m": stage_value - main_value,
            "stage_delta_vs_phase82_m": stage_value - baseline,
            "main_delta_vs_phase82_m": main_value - baseline,
            "stage_already_regressed_vs_phase82": stage_regressed,
            "main_added_or_amplified_regression_vs_stage": main_added,
            "attribution_class": attribution,
        }
    stage_macro = sum(route_scores[route]["stage"]["score_m"] for route in ROUTES) / 2.0
    main_macro = sum(route_scores[route]["main"]["score_m"] for route in ROUTES) / 2.0
    result = {
        "schema_version": RESULT_SCHEMA,
        "phase": 104,
        "execution_label": "Luna Max",
        "candidate": CANDIDATE_ID,
        "status": "complete-truth-only-attribution",
        "decision": "Stage and main were scored with the identical Phase82 metric after raw-only native generation; no promotion or publication is implied.",
        "routes": route_scores,
        "aggregate": {
            "route_order": list(ROUTES),
            "route_count": 2,
            "macro_weighting": "unweighted arithmetic mean over exactly MTV-A then LAX-T",
            "stage_macro_score_m": stage_macro,
            "main_macro_score_m": main_macro,
            "stage_minus_main_macro_m": stage_macro - main_macro,
        },
        "metric_contract": {
            "key": "(phone, UnixTimeMillis)",
            "matching": "exact integer key intersection only",
            "distance": "spherical Haversine per row",
            "earth_radius_m": EARTH_RADIUS_M,
            "percentile": "linear interpolation at rank (n - 1) * q",
            "route_scalar": "(P50 + P95) / 2 in metres",
            "macro": "unweighted arithmetic mean over exactly MTV-A then LAX-T",
            "stage_conversion": "ECEF metres to WGS84 latitude/longitude in evaluator memory only",
        },
        "read_accounting": {
            "native_solver_invocations": 0,
            "raw_device_gnss_reads_by_evaluator": 0,
            "raw_device_imu_reads_by_evaluator": 0,
            "broadcast_navigation_reads_by_evaluator": 0,
            "truth_reads": truth_reads,
            "truth_reads_per_route": 1,
            "accuracy_calculations": 4,
            "stage_or_main_reinput": 0,
            "mat_reads_or_generated": 0,
            "base_reads": 0,
            "pdc_or_precomputed_coordinate_reads": 0,
            "kaggle_or_token_access": 0,
            "reruns": 0,
            "fallbacks": 0,
            "solution_rows_in_result": False,
        },
        "forbidden_lanes": {
            "truth_in_native": False,
            "MAT": False,
            "base": False,
            "PDC": False,
            "precomputed_coordinates": False,
            "Kaggle_or_token": False,
            "solution_rows": False,
        },
        "solution_publication": False,
        "Kaggle_submission": False,
    }
    atomic_json(result_path, result)
    atomic_text(result_path.with_suffix(".md"), result_markdown(result))
    return result


def result_markdown(result: dict[str, Any]) -> str:
    lines = [
        "# Phase104 stage-vs-main attribution result",
        "",
        f"- Status: `{result['status']}`",
        "- Native: fresh Phase101 raw-only pipeline; exactly one MTV-A and one LAX-T run.",
        "- Truth: one isolated evaluator read per route; no native rerun, fallback, or publication.",
        "",
        "## Aggregate",
        "",
        f"- Stage macro: `{result['aggregate']['stage_macro_score_m']}` m",
        f"- Main macro: `{result['aggregate']['main_macro_score_m']}` m",
        f"- Stage minus main: `{result['aggregate']['stage_minus_main_macro_m']}` m",
        "",
        "## Routes",
        "",
        "| Route | Stage | Main | Stage−main | Attribution | Truth reads |",
        "|---|---:|---:|---:|---|---:|",
    ]
    for route in ROUTES:
        item = result["routes"][route]
        lines.append(
            f"| `{route}` | `{item['stage']['score_m']}` | `{item['main']['score_m']}` | "
            f"`{item['attribution']['stage_minus_main_m']}` | "
            f"`{item['attribution']['attribution_class']}` | `1` |"
        )
    lines.extend([
        "",
        "The stage ECEF sidecar and main displacement statistics are private diagnostic artifacts; no coordinate rows are included in this result.",
    ])
    return "\n".join(lines) + "\n"


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--verify-freeze", action="store_true")
    parser.add_argument("--verify-manifest", action="store_true")
    parser.add_argument("--verify-pre-raw", action="store_true")
    parser.add_argument("--evaluate", action="store_true")
    parser.add_argument("--output-root", type=Path)
    parser.add_argument("--result-json", type=Path)
    args = parser.parse_args(argv)
    modes = [args.verify_freeze, args.verify_manifest, args.verify_pre_raw, args.evaluate]
    if sum(bool(value) for value in modes) != 1:
        parser.error("choose exactly one verification/evaluation mode")
    try:
        if args.verify_freeze:
            verify_freeze()
        elif args.verify_manifest:
            verify_manifest()
        elif args.verify_pre_raw:
            print(json.dumps(verify_pre_raw(), indent=2, sort_keys=True))
        else:
            if args.output_root is None or args.result_json is None:
                parser.error("--evaluate requires --output-root and --result-json")
            result = evaluate(args.output_root.resolve(), args.result_json.resolve())
            print(json.dumps(result, indent=2, sort_keys=True))
            return 0
    except (Phase104Error, OSError) as exc:
        print(f"phase104 contract/evaluator: fail-closed: {exc}")
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
