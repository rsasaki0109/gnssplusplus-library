#!/usr/bin/env python3
"""Execute and structurally evaluate the sealed Phase92 raw-only matrix.

The Phase92 pre-raw evaluator remains intentionally launch-free and pinned.
This companion is the post-authorization runner: it imports that verifier,
resolves the three inherited raw files from the sealed Phase91 input manifest,
launches each frozen command exactly once in route order, keeps separate
stdout/stderr files (including partial files), and validates only native
summaries/submissions.  It never reads truth, MAT, Kaggle/token, base, or
precomputed-coordinate artifacts and it never computes accuracy.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import io
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time
from typing import Any


ROOT = Path(__file__).resolve().parents[3]
PRE_RAW_PATH = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase92_source_meter_clock_state_parity.py"
MANIFEST_PATH = ROOT / "docs/use_cases/records/smartphone_r5_phase92_source_meter_clock_state_parity_execution_manifest_v1.json"
PHASE91_INPUT_MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase91_source_clock_c0d_gnss_first_raw_drift_d_initializer_execution_manifest_v1.json"
EXECUTION_AUTHORIZATION = ROOT / "docs/use_cases/records/smartphone_r5_phase92_source_meter_clock_state_parity_raw_execution_authorization_v1.json"
EXECUTION_WRAPPER = Path(__file__).resolve()
BINARY = ROOT / "build/apps/gnss_fgo_imu_no_base"
OUTPUT_ROOT = ROOT / "output/smartphone-r5/phase92-source-meter-clock-state-parity-v1"
STRUCTURAL_RESULT = ROOT / "docs/use_cases/records/smartphone_r5_phase92_source_meter_clock_state_parity_structural_result_v1.json"

ROUTES = (
    "2021-03-16-18-59-us-ca-mtv-a/pixel5",
    "2021-08-24-20-32-us-ca-mtv-h/pixel5",
    "2022-04-01-18-22-us-ca-lax-t/pixel5",
    "2023-03-08-21-34-us-ca-mtv-u/pixel5",
)
DOMAIN_ROWS = {
    ROUTES[0]: 2158,
    ROUTES[1]: 3139,
    ROUTES[2]: 1465,
    ROUTES[3]: 1101,
}
RAW_NAMES = ("device_gnss.csv", "device_imu.csv", "brdc.nav")
RAW_FLAGS = (("--android-gnss", "device_gnss.csv"), ("--android-imu", "device_imu.csv"), ("--nav", "brdc.nav"))
FORBIDDEN_FLAGS = (
    "--obs",
    "--native-gnss-first-velocity-only-handoff",
    "--native-direct-doppler-wls-handoff",
    "--native-pdc-state-bridge",
    "--native-upstream-quality",
    "--native-base-rinex",
    "--native-base-pseudorange-compensation",
    "--native-base-pseudorange-source-miss-mask",
    "--native-base-pseudorange-preserve-additional-frequency-bands",
)
FORBIDDEN_PATH_TERMS = (
    ".mat",
    "ground_truth",
    "validation",
    "holdout",
    "kaggle",
    "token",
    "phase82",
    "base.rinex",
    "coordinate",
    "truth",
)
REQUIRED_FLAGS = (
    "--all-epochs",
    "--android-raw-utc-keys",
    "--android-raw-clock-only",
    "--android-utc-wall-clock-fallback",
    "--native-pdc-imu-tdcp-no-bridge",
    "--native-source-direct-observable-quality",
    "--native-source-clock-c0d-factor",
    "--native-source-clock-c0d-active-solve-diagnostic",
    "--native-source-clock-c0d-gnss-first-raw-drift-d-initializer",
    "--native-source-clock-c0d-meter-state-parity",
)
TERMINATION_REASONS = {
    "small_cost_change",
    "maximum_lambda",
    "maximum_outer_iterations",
    "outer_convergence_tolerance",
    "no_inner_iteration",
    "no_progress_unclassified",
    "exception",
}
SPEED_OF_LIGHT = 299792458.0
EARTH_RADIUS_M = 6_371_000.0
MAX_SPEED_MPS = 70.0
SCHEMA = "smartphone-r5-phase92-source-meter-clock-state-parity-structural-result.v1"


class StructuralError(ValueError):
    """A fail-closed structural result error."""


def fail(message: str) -> StructuralError:
    return StructuralError(message)


def relative(path: Path) -> str:
    try:
        return str(path.relative_to(ROOT))
    except ValueError:
        return str(path)


def _reject_forbidden(path: str | Path) -> None:
    lowered = str(path).lower()
    if any(term in lowered for term in FORBIDDEN_PATH_TERMS):
        raise fail(f"forbidden artifact path: {path}")


def _read_json(path: Path, label: str) -> dict[str, Any]:
    _reject_forbidden(path)
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise fail(f"failed to read {label}: {path}: {exc}") from exc
    if not isinstance(value, dict):
        raise fail(f"{label} is not an object")
    return value


def _hash_file(path: Path, label: str) -> str:
    _reject_forbidden(path)
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


def _verify_execution_authorization(
    manifest: dict[str, Any], evaluator_pin: dict[str, Any]
) -> dict[str, Any]:
    """Verify the separately sealed post-freeze authorization before raw I/O."""
    authorization = _read_json(
        EXECUTION_AUTHORIZATION, "Phase92 raw execution authorization"
    )
    if authorization.get("schema_version") != (
        "smartphone-r5-phase92-source-meter-clock-state-parity-raw-execution-authorization.v1"
    ) or authorization.get("status") != "authorized-before-phase92-raw-execution":
        raise fail("Phase92 raw execution authorization identity changed")
    if authorization.get("execution_label") != "Luna Max":
        raise fail("Phase92 raw execution authorization label changed")

    authority = authorization.get("authority")
    if not isinstance(authority, dict):
        raise fail("Phase92 raw execution authorization authority is missing")
    freeze_pin = authority.get("phase92_execution_freeze")
    manifest_pin = authority.get("phase92_execution_manifest")
    evaluator_pin_record = authority.get("phase92_pre_raw_evaluator")
    if not isinstance(freeze_pin, dict) or not isinstance(manifest_pin, dict) or not isinstance(evaluator_pin_record, dict):
        raise fail("Phase92 raw execution authorization authority is malformed")
    if freeze_pin.get("path") != "docs/use_cases/records/smartphone_r5_phase92_source_meter_clock_state_parity_execution_freeze_v1.json" or freeze_pin.get("sha256") != "c07ac587ebcfde6391de0b8a824687316de854442166a93daadb416d4e9b74d8":
        raise fail("Phase92 execution freeze authorization pin changed")
    if manifest_pin.get("path") != relative(MANIFEST_PATH) or manifest_pin.get("sha256") != _hash_file(MANIFEST_PATH, "Phase92 execution manifest"):
        raise fail("Phase92 execution manifest authorization pin changed")
    if evaluator_pin_record.get("path") != evaluator_pin.get("path") or evaluator_pin_record.get("sha256") != evaluator_pin.get("sha256"):
        raise fail("Phase92 pre-raw evaluator authorization pin changed")

    executor = authorization.get("executor")
    if not isinstance(executor, dict) or executor.get("path") != relative(EXECUTION_WRAPPER):
        raise fail("Phase92 execution wrapper authorization path changed")
    wrapper_sha = executor.get("sha256")
    if not isinstance(wrapper_sha, str) or len(wrapper_sha) != 64 or _hash_file(EXECUTION_WRAPPER, "Phase92 execution wrapper") != wrapper_sha:
        raise fail("Phase92 execution wrapper authorization hash changed")

    candidate = authorization.get("candidate")
    if not isinstance(candidate, dict) or candidate.get("id") != "phase92_source_meter_clock_state_parity_with_retained_raw_d_alignment" or candidate.get("selector") != "--native-source-clock-c0d-meter-state-parity" or candidate.get("raw_only") is not True or candidate.get("diagnostic_only") is not True or candidate.get("accuracy_scored") is not False or candidate.get("route_score_selection") is not False:
        raise fail("Phase92 raw execution candidate authorization changed")
    matrix = authorization.get("matrix")
    if not isinstance(matrix, dict) or matrix.get("candidate_count") != 1 or matrix.get("routes") != 4 or matrix.get("runs_per_route") != 1 or matrix.get("controls") != 0 or matrix.get("candidate_runs_total") != 4 or matrix.get("raw_device_gnss_reads") != 4 or matrix.get("raw_device_imu_reads") != 4 or matrix.get("broadcast_navigation_reads") != 4 or matrix.get("base_rinex_reads") != 0 or matrix.get("truth_reads") != 0 or matrix.get("mat_reads_or_generated") != 0 or matrix.get("precomputed_coordinate_reads") != 0 or matrix.get("accuracy_scored") is not False or matrix.get("route_score_selection") is not False:
        raise fail("Phase92 raw execution matrix authorization changed")
    routes = authorization.get("routes")
    if routes != list(ROUTES):
        raise fail("Phase92 raw execution route order authorization changed")
    execution = authorization.get("execution_authorization")
    if not isinstance(execution, dict) or execution.get("raw_execution_authorized") is not True or execution.get("exactly_one_run_each") is not True or execution.get("native_route_rerun_performed") is not False or execution.get("stop_before_accuracy_or_submission") is not True or execution.get("accuracy_scored") is not False or execution.get("route_score_selection") is not False:
        raise fail("Phase92 raw execution authorization boundary changed")
    accounting = authorization.get("read_accounting_at_authorization")
    if not isinstance(accounting, dict):
        raise fail("Phase92 raw execution authorization accounting is missing")
    for key in (
        "raw_device_gnss_reads",
        "raw_device_imu_reads",
        "broadcast_navigation_reads",
        "base_rinex_reads",
        "native_solver_invocations",
        "truth_reads",
        "mat_reads_or_generated",
        "precomputed_coordinate_reads",
        "validation_holdout_reads",
        "kaggle_or_token_access",
    ):
        if accounting.get(key) != 0:
            raise fail(f"Phase92 authorization pre-run accounting is nonzero: {key}")
    if accounting.get("accuracy_scored") is not False or accounting.get("route_score_selection") is not False:
        raise fail("Phase92 authorization pre-run accuracy accounting changed")
    if manifest.get("status") != "sealed-before-phase92-raw-execution":
        raise fail("Phase92 pre-raw manifest was not sealed at authorization")
    return authorization


def _finite_tree(value: Any, label: str) -> None:
    if value is None or isinstance(value, (bool, str)):
        return
    if isinstance(value, (int, float)):
        if not math.isfinite(float(value)):
            raise fail(f"non-finite value: {label}")
        return
    if isinstance(value, dict):
        for key, item in value.items():
            _finite_tree(item, f"{label}.{key}")
        return
    if isinstance(value, list):
        for index, item in enumerate(value):
            _finite_tree(item, f"{label}[{index}]")


def _number(mapping: dict[str, Any], key: str, label: str, positive: bool = False) -> float:
    value = mapping.get(key)
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(float(value)):
        raise fail(f"missing/non-finite number: {label}/{key}")
    result = float(value)
    if positive and result <= 0.0:
        raise fail(f"non-positive number: {label}/{key}")
    return result


def _count(mapping: dict[str, Any], key: str, label: str, minimum: int = 0) -> int:
    value = mapping.get(key)
    if isinstance(value, bool) or not isinstance(value, int) or value < minimum:
        raise fail(f"invalid count: {label}/{key}")
    return value


def _load_authorized_raw_inputs() -> dict[str, dict[str, dict[str, Any]]]:
    """Resolve only metadata/path pins; raw file bytes are not opened here."""
    inherited = _read_json(PHASE91_INPUT_MANIFEST, "inherited Phase91 raw-input manifest")
    routes = inherited.get("routes")
    if not isinstance(routes, list) or [item.get("dataset_id") for item in routes] != list(ROUTES):
        raise fail("inherited raw-input route order changed")
    resolved: dict[str, dict[str, dict[str, Any]]] = {}
    for item in routes:
        route = item.get("dataset_id")
        raw = item.get("raw_inputs")
        if route not in ROUTES or not isinstance(raw, dict) or tuple(raw) != RAW_NAMES:
            raise fail(f"inherited raw-input set changed: {route}")
        resolved[route] = {}
        for name in RAW_NAMES:
            pin = raw.get(name)
            if not isinstance(pin, dict) or not isinstance(pin.get("path"), str):
                raise fail(f"inherited raw-input pin malformed: {route}/{name}")
            path_text = pin["path"]
            _reject_forbidden(path_text)
            path_obj = Path(path_text)
            if path_obj.is_absolute() or ".." in path_obj.parts:
                raise fail(f"inherited raw-input path is not safely relative: {route}/{name}")
            path = ROOT / path_text
            # stat/is_file is provenance validation; it does not consume file
            # contents and is kept separate from the native process read count.
            if not path.is_file():
                raise fail(f"missing inherited raw input: {path}")
            sha = pin.get("sha256")
            if not isinstance(sha, str) or len(sha) != 64:
                raise fail(f"inherited raw-input hash pin malformed: {route}/{name}")
            if Path(path_text).name != name or Path(path_text).is_absolute():
                raise fail(f"inherited raw-input basename mismatch: {route}/{name}")
            resolved[route][name] = {
                "path": path_text,
                "sha256": sha,
                "size_bytes": path.stat().st_size,
                "inherited_manifest": relative(PHASE91_INPUT_MANIFEST),
            }
    return resolved


def _verify_pre_raw_and_manifest() -> tuple[dict[str, Any], dict[str, Any], dict[str, dict[str, dict[str, Any]]], dict[str, Any]]:
    """Run the pinned static verifier before opening any raw input."""
    sys.path.insert(0, str(PRE_RAW_PATH.parent))
    try:
        import gnss_smartphone_phase92_source_meter_clock_state_parity as pre_raw
    except ImportError as exc:
        raise fail(f"unable to import pinned Phase92 pre-raw verifier: {exc}") from exc
    try:
        pre_raw_report = pre_raw.verify_pre_raw()
        manifest = pre_raw.verify_manifest()
    except Exception as exc:  # verifier has its own fail-closed exception
        raise fail(f"pinned Phase92 pre-raw verifier failed: {exc}") from exc
    if pre_raw_report.get("raw_reads") != 0 or pre_raw_report.get("native_solver_invocations") != 0:
        raise fail("pre-raw verifier reported a raw/native read")
    evaluator_pin = {
        "path": relative(PRE_RAW_PATH),
        "sha256": _hash_file(PRE_RAW_PATH, "Phase92 pre-raw evaluator"),
    }
    authorization = _verify_execution_authorization(manifest, evaluator_pin)
    return pre_raw_report, manifest, _load_authorized_raw_inputs(), {
        **evaluator_pin,
        "execution_authorization": {
            "path": relative(EXECUTION_AUTHORIZATION),
            "sha256": _hash_file(EXECUTION_AUTHORIZATION, "Phase92 raw execution authorization"),
        },
    }


def _materialize_command(record: dict[str, Any], raw: dict[str, dict[str, Any]]) -> list[str]:
    route = record.get("dataset_id")
    command = record.get("command")
    if route not in ROUTES or not isinstance(command, list) or not all(isinstance(token, str) for token in command):
        raise fail(f"invalid sealed command: {route}")
    materialized = list(command)
    for flag, name in RAW_FLAGS:
        if materialized.count(flag) != 1:
            raise fail(f"sealed command flag multiplicity failed: {route}/{flag}")
        index = materialized.index(flag) + 1
        # The Phase92 manifest is intentionally pre-raw and therefore has
        # raw/phase92 placeholders.  Do not permit arbitrary substitutions.
        expected_placeholder = f"raw/phase92/{route}/{name}"
        if materialized[index] != expected_placeholder:
            raise fail(f"sealed command placeholder changed: {route}/{name}")
        materialized[index] = raw[name]["path"]
    if materialized[0] != relative(BINARY):
        raise fail(f"sealed binary changed: {route}")
    if materialized.count("--dataset-id") != 1 or materialized[materialized.index("--dataset-id") + 1] != route:
        raise fail(f"sealed dataset identity changed: {route}")
    for flag in REQUIRED_FLAGS:
        if materialized.count(flag) != 1:
            raise fail(f"required Phase92 flag missing/multiple: {route}/{flag}")
    if any(flag in materialized for flag in FORBIDDEN_FLAGS):
        raise fail(f"forbidden Phase92 flag present: {route}")
    for token in materialized:
        _reject_forbidden(token)
    return materialized


def _expected_paths(record: dict[str, Any]) -> tuple[Path, Path]:
    outputs = record.get("planned_output")
    if not isinstance(outputs, dict):
        raise fail(f"sealed output record missing: {record.get('dataset_id')}")
    submission_text, summary_text = outputs.get("submission"), outputs.get("summary")
    if not isinstance(submission_text, str) or not isinstance(summary_text, str):
        raise fail(f"sealed output paths malformed: {record.get('dataset_id')}")
    prefix = "output/smartphone-r5/phase92-source-meter-clock-state-parity-v1/"
    if not submission_text.startswith(prefix) or not summary_text.startswith(prefix):
        raise fail(f"sealed output escaped Phase92 root: {record.get('dataset_id')}")
    return ROOT / submission_text, ROOT / summary_text


def execute_matrix(manifest: dict[str, Any], raw_inputs: dict[str, dict[str, dict[str, Any]]]) -> list[dict[str, Any]]:
    if OUTPUT_ROOT.exists():
        raise fail(f"refusing to overwrite existing Phase92 output root: {OUTPUT_ROOT}")
    OUTPUT_ROOT.mkdir(parents=True, exist_ok=False)
    records = manifest.get("routes")
    if not isinstance(records, list) or [item.get("dataset_id") for item in records] != list(ROUTES):
        raise fail("sealed Phase92 route order/count changed")
    results: list[dict[str, Any]] = []
    environment = os.environ.copy()
    local_lib = "/home/sasaki/.local/lib"
    environment["LD_LIBRARY_PATH"] = local_lib + (":" + environment["LD_LIBRARY_PATH"] if environment.get("LD_LIBRARY_PATH") else "")
    for record in records:
        route = record["dataset_id"]
        output_path, summary_path = _expected_paths(record)
        route_dir = output_path.parent
        route_dir.mkdir(parents=True, exist_ok=False)
        materialized = _materialize_command(record, raw_inputs[route])
        stdout_path, stderr_path = route_dir / "stdout.log", route_dir / "stderr.log"
        started = time.time()
        return_code: int | None = None
        interrupted = False
        with stdout_path.open("wb") as stdout, stderr_path.open("wb") as stderr:
            try:
                completed = subprocess.run(
                    materialized,
                    cwd=ROOT,
                    env=environment,
                    stdout=stdout,
                    stderr=stderr,
                    check=False,
                )
                return_code = completed.returncode
            except KeyboardInterrupt:
                interrupted = True
                stderr.write(b"\nPhase92 runner interrupted; partial native output preserved.\n")
        ended = time.time()
        metadata = {
            "schema_version": "smartphone-r5-phase92-source-meter-clock-state-parity-route-execution.v1",
            "dataset_id": route,
            "run_number": 1,
            "command": materialized,
            "raw_inputs": raw_inputs[route],
            "planned_output": {
                "submission": relative(output_path),
                "summary": relative(summary_path),
            },
            "stdout": relative(stdout_path),
            "stderr": relative(stderr_path),
            "started_unix_s": started,
            "ended_unix_s": ended,
            "duration_s": ended - started,
            "return_code": return_code,
            "interrupted": interrupted,
        }
        metadata_path = route_dir / "run_metadata.json"
        _atomic_json(metadata_path, metadata)
        results.append(metadata)
    return results


def _atomic_json(path: Path, value: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, temporary = tempfile.mkstemp(prefix=f".{path.name}.", suffix=".tmp", dir=str(path.parent))
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as handle:
            json.dump(value, handle, indent=2, sort_keys=True)
            handle.write("\n")
        Path(temporary).replace(path)
        temporary = ""
    finally:
        if temporary:
            try:
                Path(temporary).unlink()
            except FileNotFoundError:
                pass


def _submission(path: Path, route: str) -> tuple[list[tuple[int, float, float]], dict[str, Any]]:
    _reject_forbidden(path)
    if not path.is_file():
        raise fail(f"missing submission: {route}")
    payload = path.read_bytes()
    try:
        rows = list(csv.reader(io.StringIO(payload.decode("utf-8-sig"))))
    except (UnicodeDecodeError, csv.Error) as exc:
        raise fail(f"invalid submission encoding: {route}: {exc}") from exc
    if not rows or rows[0] != ["phone", "UnixTimeMillis", "LatitudeDegrees", "LongitudeDegrees"]:
        raise fail(f"submission header mismatch: {route}")
    parsed: list[tuple[int, float, float]] = []
    previous: int | None = None
    for line, fields in enumerate(rows[1:], 2):
        if len(fields) != 4 or fields[0] != route:
            raise fail(f"submission row identity mismatch: {route}:{line}")
        try:
            timestamp, latitude, longitude = int(fields[1]), float(fields[2]), float(fields[3])
        except ValueError as exc:
            raise fail(f"non-numeric submission row: {route}:{line}") from exc
        if previous is not None and timestamp <= previous:
            raise fail(f"submission timestamps are not strictly increasing: {route}")
        if not all(math.isfinite(item) for item in (latitude, longitude)) or not -90.0 <= latitude <= 90.0 or not -180.0 <= longitude <= 180.0:
            raise fail(f"invalid earth coordinate: {route}:{line}")
        parsed.append((timestamp, latitude, longitude))
        previous = timestamp
    if not parsed:
        raise fail(f"empty submission: {route}")
    return parsed, {"rows": len(parsed), "bytes": len(payload), "sha256": hashlib.sha256(payload).hexdigest()}


def _speed_report(rows: list[tuple[int, float, float]]) -> dict[str, Any]:
    speeds: list[float] = []
    for previous, current in zip(rows, rows[1:]):
        dt = (current[0] - previous[0]) / 1000.0
        if dt <= 0.0:
            raise fail("non-positive submission interval")
        lat1, lon1, lat2, lon2 = map(math.radians, (previous[1], previous[2], current[1], current[2]))
        hav = math.sin((lat2 - lat1) / 2.0) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin((lon2 - lon1) / 2.0) ** 2
        distance = EARTH_RADIUS_M * 2.0 * math.asin(math.sqrt(min(1.0, max(0.0, hav))))
        speeds.append(distance / dt)
    return {
        "finite": all(math.isfinite(speed) for speed in speeds),
        "max_speed_mps": max(speeds, default=0.0),
        "over_70_mps_count": sum(speed > MAX_SPEED_MPS for speed in speeds),
        "transition_count": len(speeds),
    }


def _validate_provenance(summary: dict[str, Any], route: str, raw: dict[str, dict[str, Any]]) -> None:
    if summary.get("dataset_id") != route or summary.get("status") != "imu-combined-factor" or summary.get("truth_used") is not False or summary.get("base_factors") is not False or summary.get("no_base_contract") is not True or summary.get("production_default_changed") is not False:
        raise fail(f"summary identity/truth/default gate failed: {route}")
    for key in (
        "native_source_clock_c0d_factor_enabled",
        "native_source_clock_c0d_meter_state_parity_enabled",
        "native_source_clock_c0d_active_solve_diagnostic_enabled",
        "native_source_clock_c0d_gnss_first_raw_drift_d_initializer_enabled",
        "native_source_direct_observable_quality_enabled",
        "native_pdc_imu_tdcp_no_bridge",
    ):
        if summary.get(key) is not True:
            raise fail(f"candidate flag telemetry failed: {route}/{key}")
    for key in ("native_pdc_state_bridge", "native_upstream_quality", "native_direct_doppler_wls_handoff", "native_gnss_first_velocity_only_handoff"):
        if summary.get(key) is True:
            raise fail(f"forbidden alternate telemetry enabled: {route}/{key}")
    inputs = summary.get("inputs")
    if not isinstance(inputs, dict) or inputs.get("observation") is not None:
        raise fail(f"raw-only input provenance missing: {route}")
    expected = {"android_gnss": raw["device_gnss.csv"]["path"], "imu": raw["device_imu.csv"]["path"], "navigation": raw["brdc.nav"]["path"]}
    for key, value in expected.items():
        if inputs.get(key) != value:
            raise fail(f"summary input provenance mismatch: {route}/{key}")
        _reject_forbidden(value)
    gnss = summary.get("android_gnss_diagnostics")
    imu = summary.get("imu_initialization")
    if not isinstance(gnss, dict) or gnss.get("no_device_wls_seed") is not True or not isinstance(imu, dict) or imu.get("input_format") != "android-device_imu.csv":
        raise fail(f"raw GNSS/IMU provenance telemetry missing: {route}")


def _validate_handoff(summary: dict[str, Any], route: str, expected_epochs: int) -> dict[str, Any]:
    handoff = summary.get("gnss_first")
    if not isinstance(handoff, dict):
        raise fail(f"GNSS-first telemetry missing: {route}")
    exact = ("main_epoch_count", "gnss_first_epoch_count", "solution_epoch_count", "raw_epoch_count", "raw_utc_key_count", "aligned_epoch_count")
    zero = (
        "epoch_count_mismatch_count",
        "nonfinite_time_count",
        "gnss_first_time_mismatch_count",
        "raw_time_mismatch_count",
        "raw_utc_key_order_mismatch_count",
        "duplicate_raw_utc_key_count",
        "raw_utc_key_mismatch_count",
        "retained_source_index_mismatch_count",
        "retained_source_order_mismatch_count",
        "duplicate_retained_source_index_count",
        "retained_raw_key_mismatch_count",
        "raw_drift_count_mismatch_count",
        "raw_drift_nonfinite_count",
        "nonfinite_solution_count",
    )
    required = {
        "attempted": True,
        "converged": True,
        "handoff_mode": "gnss-first-in-memory-position-clock-velocity",
        "velocity_handoff_source": "same-run-gnss-first-optimizer-result",
        "position_clock_handoff_source": "same-run-gnss-first-optimizer-result",
        "raw_drift_d_initializer": "main-graph-only",
        "epoch_identity_alignment_valid": True,
        "positions_clocks_copied": expected_epochs,
        "coordinates_source": "in-memory GNSS-first result only",
        "forbidden_coordinate_sources": ["PDC", "direct-WLS", "external", "precomputed"],
        "failure": "",
    }
    for key, value in required.items():
        if handoff.get(key) != value:
            raise fail(f"GNSS-first handoff contract failed: {route}/{key}")
    for key in exact:
        if _count(handoff, key, f"gnss-first/{route}") != expected_epochs:
            raise fail(f"GNSS-first retained count failed: {route}/{key}")
    for key in zero:
        if _count(handoff, key, f"gnss-first/{route}") != 0:
            raise fail(f"GNSS-first exact alignment failure: {route}/{key}")
    return {key: handoff[key] for key in (*exact, *zero, "positions_clocks_copied")}


def _validate_raw_d(c0d: dict[str, Any], route: str, expected_epochs: int) -> dict[str, Any]:
    raw = c0d.get("raw_drift_d_initializer")
    if not isinstance(raw, dict):
        raise fail(f"raw D telemetry missing: {route}")
    for key, value in (("enabled", True), ("attempted", True), ("coverage_valid", True), ("source_field", "EpochSeed.receiver_clock_drift_mps"), ("units", "metres_per_second"), ("fallback", "none"), ("failure", "")):
        if raw.get(key) != value:
            raise fail(f"raw D initializer contract failed: {route}/{key}")
    for key in ("epoch_count", "finite_count"):
        if _count(raw, key, f"raw-D/{route}") != expected_epochs:
            raise fail(f"raw D coverage failed: {route}/{key}")
    if _count(raw, "nonfinite_count", f"raw-D/{route}") != 0:
        raise fail(f"raw D finite coverage failed: {route}")
    minimum, maximum = _number(raw, "min_mps", f"raw-D/{route}"), _number(raw, "max_mps", f"raw-D/{route}")
    if minimum > maximum:
        raise fail(f"raw D range invalid: {route}")
    return {key: raw[key] for key in ("epoch_count", "finite_count", "nonfinite_count", "min_mps", "max_mps", "source_field", "units")}


def _validate_c0d(c0d: dict[str, Any], route: str, expected_rows: int) -> dict[str, Any]:
    if c0d.get("clock_c0d_enabled") is not True or c0d.get("meter_state_parity_enabled") is not True or c0d.get("internal_clock_state_unit") != "metres" or c0d.get("internal_isb_state_unit") != "metres" or c0d.get("internal_drift_state_unit") != "metres_per_second":
        raise fail(f"meter-state C0/D telemetry failed: {route}")
    factor_count = _count(c0d, "clock_c0d_factor_count", route)
    skip_keys = ("clock_c0d_clock_jump_skips", "clock_c0d_gap_skips", "clock_c0d_invalid_dt_skips", "clock_c0d_phone_exclusion_skips")
    skips = {key: _count(c0d, key, route) for key in skip_keys}
    if skips["clock_c0d_phone_exclusion_skips"] != 0 or factor_count + sum(skips.values()) != expected_rows:
        raise fail(f"C0/D factor/skip accounting failed: {route}")
    dt_min, dt_max = _number(c0d, "clock_c0d_dt_min_s", route, positive=True), _number(c0d, "clock_c0d_dt_max_s", route, positive=True)
    if not dt_min <= dt_max < 1.5:
        raise fail(f"C0/D dt range invalid: {route}")
    expected = {
        "clock_c0d_equation": "(C2-C1)-((D1+D2)*dt/2)",
        "clock_c0d_jacobian_order": ["c1", "c2", "d1", "d2"],
        "clock_c0d_jacobian": "[-1,+1,-dt/2,-dt/2]",
        "clock_c0d_units": {"clock": "metres", "isb": "metres", "drift": "metres_per_second", "dt": "seconds", "residual": "metres", "sigma": "metres"},
        "clock_jump_noise": "Inf (active C0 factor omitted)",
        "parity_scope": "C0/D active-row parity; not full seven-vector",
        "legacy_scalar_clock_between_factor_count": 0,
        "clock_c0d_sigma_m": 0.1,
        "public_clock_output_unit": "seconds",
        "public_clock_output_conversion": "meter_state ? C_i/C_LIGHT : C_i",
    }
    for key, value in expected.items():
        if c0d.get(key) != value:
            raise fail(f"C0/D equation/unit contract failed: {route}/{key}")
    if not math.isclose(_number(c0d, "speed_of_light_mps", route), SPEED_OF_LIGHT, rel_tol=0.0, abs_tol=1e-9):
        raise fail(f"C0/D speed-of-light constant changed: {route}")
    if not math.isclose(_number(c0d, "clock_c0d_sigma_seconds", route), 0.1 / SPEED_OF_LIGHT, rel_tol=1e-12, abs_tol=1e-18):
        raise fail(f"C0/D public sigma conversion changed: {route}")
    return {"factor_count": factor_count, **skips, "dt_min_s": dt_min, "dt_max_s": dt_max}


def _validate_active(c0d: dict[str, Any], graph: dict[str, Any], route: str) -> dict[str, Any]:
    for key in ("active_solve_diagnostic_enabled", "active_solve_attempted", "active_solve_finite_costs", "termination_trace_complete"):
        if c0d.get(key) is not True:
            raise fail(f"active-solve telemetry incomplete: {route}/{key}")
    initial, final = _number(c0d, "active_solve_initial_cost", route), _number(c0d, "active_solve_final_cost", route)
    for key in ("initial_lambda", "maximum_lambda", "final_lambda", "max_whitened_clock_column_norm", "max_whitened_drift_column_norm", "conditioning_proxy"):
        _number(c0d, key, route, positive=True)
    accepted = _count(c0d, "accepted_outer_iterations", route)
    attempts = _count(c0d, "total_inner_lambda_attempts", route)
    indeterminate = _count(c0d, "indeterminate_linear_solve_count", route)
    for key in ("unsuccessful_model_step_count", "small_cost_change_stop_count", "maximum_lambda_stop_count"):
        _count(c0d, key, route)
    reason = c0d.get("termination_branch_reason")
    if reason not in TERMINATION_REASONS:
        raise fail(f"unknown active-solve terminal reason: {route}/{reason}")
    if reason == "small_cost_change" and (c0d["small_cost_change_stop_count"] != 1 or c0d["maximum_lambda_stop_count"] != 0):
        raise fail(f"small-cost terminal counters inconsistent: {route}")
    if reason == "maximum_lambda" and (c0d["maximum_lambda_stop_count"] != 1 or c0d["small_cost_change_stop_count"] != 0):
        raise fail(f"maximum-lambda terminal counters inconsistent: {route}")
    if reason not in {"small_cost_change", "maximum_lambda"} and (c0d["small_cost_change_stop_count"] != 0 or c0d["maximum_lambda_stop_count"] != 0):
        raise fail(f"terminal counters inconsistent: {route}")
    if attempts < accepted + indeterminate:
        raise fail(f"inner-attempt accounting inconsistent: {route}")
    graph_initial, graph_final = _number(graph, "initial_cost", route), _number(graph, "final_cost", route)
    iterations = _count(graph, "iterations", route, minimum=1)
    if graph.get("converged") is not True or graph_initial != initial or graph_final != final or accepted != iterations or not final < initial:
        raise fail(f"active graph solve gate failed: {route}")
    return {"initial_cost": initial, "final_cost": final, "iterations": iterations, "accepted_outer_iterations": accepted, "total_inner_lambda_attempts": attempts, "indeterminate_linear_solve_count": indeterminate, "termination_branch_reason": reason}


def _validate_summary(summary: dict[str, Any], route: str, raw: dict[str, dict[str, Any]]) -> dict[str, Any]:
    _finite_tree(summary, f"summary[{route}]")
    _validate_provenance(summary, route, raw)
    epochs, graph, utc, direct, c0d = (summary.get(key) for key in ("epochs", "graph", "raw_utc_key_contract", "native_source_direct_observable_quality", "native_source_clock_c0d_factor"))
    if not all(isinstance(value, dict) for value in (epochs, graph, utc, direct, c0d)):
        raise fail(f"structural telemetry missing: {route}")
    assert isinstance(epochs, dict) and isinstance(graph, dict) and isinstance(utc, dict) and isinstance(direct, dict) and isinstance(c0d, dict)
    expected_epochs = DOMAIN_ROWS[route] + 1
    if _count(epochs, "problem", route) != expected_epochs or _count(epochs, "output", route) != expected_epochs or _count(graph, "imu_intervals", route) != DOMAIN_ROWS[route]:
        raise fail(f"epoch/domain invariant failed: {route}")
    if direct.get("enabled") is not True or direct.get("direct_no_pdc") is not True or direct.get("pdc_bridge") is not False or direct.get("native_pdc_state_bridge") is not False:
        raise fail(f"direct/no-PDC invariant failed: {route}")
    if utc.get("warmup_epoch_excluded") is not True or utc.get("raw_epoch_keys") != expected_epochs or utc.get("target_epochs") != DOMAIN_ROWS[route] or utc.get("exact_solution_epochs") != DOMAIN_ROWS[route] or utc.get("interpolated_epochs") != 0 or utc.get("edge_hold_epochs") != 0 or utc.get("unresolved_epochs") != 0 or utc.get("device_wls_coordinates_used") is not False:
        raise fail(f"raw UTC alignment invariant failed: {route}")
    handoff = _validate_handoff(summary, route, expected_epochs)
    raw_d = _validate_raw_d(c0d, route, expected_epochs)
    clock = _validate_c0d(c0d, route, DOMAIN_ROWS[route])
    active = _validate_active(c0d, graph, route)
    return {
        "population": {"problem_epochs": epochs["problem"], "output_epochs": epochs["output"], "imu_intervals": graph["imu_intervals"], "pseudorange_factors": epochs.get("pseudorange_factors")},
        "raw_utc": {key: utc.get(key) for key in ("raw_epoch_keys", "target_epochs", "exact_solution_epochs", "interpolated_epochs", "edge_hold_epochs", "unresolved_epochs", "device_wls_coordinates_used")},
        "gnss_first": handoff,
        "raw_drift_d_initializer": raw_d,
        "clock_c0d": {**clock, **active},
        "graph": {"initial_cost": active["initial_cost"], "final_cost": active["final_cost"], "iterations": active["iterations"], "converged": graph["converged"]},
    }


def _route_report(metadata: dict[str, Any], raw: dict[str, dict[str, Any]]) -> dict[str, Any]:
    route = metadata["dataset_id"]
    summary_path, submission_path = ROOT / metadata["planned_output"]["summary"], ROOT / metadata["planned_output"]["submission"]
    stdout_path, stderr_path = ROOT / metadata["stdout"], ROOT / metadata["stderr"]
    report: dict[str, Any] = {
        "dataset_id": route,
        "run_number": metadata.get("run_number"),
        "return_code": metadata.get("return_code"),
        "interrupted": metadata.get("interrupted"),
        "command": metadata.get("command"),
        "raw_inputs": raw,
        "stdout": {"path": relative(stdout_path), "bytes": stdout_path.stat().st_size, "sha256": _hash_file(stdout_path, f"stdout {route}")},
        "stderr": {"path": relative(stderr_path), "bytes": stderr_path.stat().st_size, "sha256": _hash_file(stderr_path, f"stderr {route}")},
    }
    gate_names = (
        "corrected_retained_key_alignment",
        "full_raw_d_coverage",
        "meter_clock_isb_telemetry",
        "c0d_official_meter_row_sigma",
        "active_solve_iterations_and_strict_cost_decrease",
        "finite_earth_valid_output_and_speed",
        "no_pdc_or_external_coordinates",
    )
    gates = {name: False for name in gate_names}
    try:
        if metadata.get("return_code") != 0 or metadata.get("interrupted"):
            raise fail(f"native route returned {metadata.get('return_code')}")
        summary = _read_json(summary_path, f"summary {route}")
        rows, submission = _submission(submission_path, route)
        diagnostics = _validate_summary(summary, route, raw)
        speed = _speed_report(rows)
        gates["corrected_retained_key_alignment"] = diagnostics["gnss_first"]["epoch_count_mismatch_count"] == 0 and diagnostics["gnss_first"]["retained_source_index_mismatch_count"] == 0 and diagnostics["gnss_first"]["retained_source_order_mismatch_count"] == 0 and diagnostics["gnss_first"]["duplicate_retained_source_index_count"] == 0 and diagnostics["gnss_first"]["retained_raw_key_mismatch_count"] == 0
        gates["full_raw_d_coverage"] = diagnostics["raw_drift_d_initializer"]["epoch_count"] == DOMAIN_ROWS[route] + 1 and diagnostics["raw_drift_d_initializer"]["finite_count"] == DOMAIN_ROWS[route] + 1 and diagnostics["raw_drift_d_initializer"]["nonfinite_count"] == 0
        gates["meter_clock_isb_telemetry"] = diagnostics["clock_c0d"]["factor_count"] > 0
        gates["c0d_official_meter_row_sigma"] = diagnostics["clock_c0d"]["factor_count"] + sum(diagnostics["clock_c0d"][key] for key in ("clock_c0d_clock_jump_skips", "clock_c0d_gap_skips", "clock_c0d_invalid_dt_skips", "clock_c0d_phone_exclusion_skips")) == DOMAIN_ROWS[route]
        gates["active_solve_iterations_and_strict_cost_decrease"] = diagnostics["clock_c0d"]["iterations"] >= 1 and diagnostics["clock_c0d"]["final_cost"] < diagnostics["clock_c0d"]["initial_cost"]
        gates["finite_earth_valid_output_and_speed"] = submission["rows"] == DOMAIN_ROWS[route] and speed["finite"] and speed["over_70_mps_count"] == 0
        gates["no_pdc_or_external_coordinates"] = summary.get("native_pdc_state_bridge") is False and summary.get("native_upstream_quality") is False and summary.get("gnss_first", {}).get("coordinates_source") == "in-memory GNSS-first result only" and summary.get("gnss_first", {}).get("positions_clocks_copied") == DOMAIN_ROWS[route] + 1
        report.update({
            "summary": {"path": relative(summary_path), "bytes": summary_path.stat().st_size, "sha256": _hash_file(summary_path, f"summary {route}")},
            "submission": submission,
            "speed": speed,
            "diagnostics": diagnostics,
        })
    except (StructuralError, OSError, KeyError, TypeError) as exc:
        report["failure"] = str(exc)
    report["gates"] = gates
    report["all_gates_anded"] = all(gates.values())
    return report


def evaluate(metadata: list[dict[str, Any]], manifest: dict[str, Any], raw_inputs: dict[str, dict[str, dict[str, Any]]], pre_raw_report: dict[str, Any], evaluator_pin: dict[str, Any]) -> dict[str, Any]:
    route_reports: dict[str, Any] = {}
    for item in metadata:
        route_reports[item["dataset_id"]] = _route_report(item, raw_inputs[item["dataset_id"]])
    matrix_gate = len(metadata) == 4 and [item.get("dataset_id") for item in metadata] == list(ROUTES) and all(item.get("run_number") == 1 for item in metadata)
    implementation = manifest.get("implementation", {})
    implementation_gate = implementation.get("commit") == "a1852bf45030d681e178fb9da9979c9a0dee737d" and _hash_file(BINARY, "Phase92 binary") == implementation.get("binary", {}).get("sha256")
    global_gates = {
        "implementation_and_binary_pins": implementation_gate,
        "exactly_four_routes_one_run_each": matrix_gate,
        "corrected_retained_key_alignment": all(item["gates"]["corrected_retained_key_alignment"] for item in route_reports.values()),
        "full_raw_d_coverage": all(item["gates"]["full_raw_d_coverage"] for item in route_reports.values()),
        "meter_clock_isb_telemetry": all(item["gates"]["meter_clock_isb_telemetry"] for item in route_reports.values()),
        "c0d_official_meter_row_sigma": all(item["gates"]["c0d_official_meter_row_sigma"] for item in route_reports.values()),
        "active_solve_iterations_and_strict_cost_decrease": all(item["gates"]["active_solve_iterations_and_strict_cost_decrease"] for item in route_reports.values()),
        "finite_earth_valid_output_and_speed": all(item["gates"]["finite_earth_valid_output_and_speed"] for item in route_reports.values()),
        "no_pdc_or_external_coordinates": all(item["gates"]["no_pdc_or_external_coordinates"] for item in route_reports.values()),
        "accuracy_not_scored": True,
    }
    global_gates["all_gates_anded"] = all(global_gates.values())
    failed = [key for key, value in global_gates.items() if value is False]
    for route, item in route_reports.items():
        failed.extend(f"{route}:{key}" for key, value in item["gates"].items() if value is False)
    result = {
        "schema_version": SCHEMA,
        "phase": 92,
        "execution_label": "Luna Max",
        "status": "go-phase92-source-meter-clock-state-parity-structural" if global_gates["all_gates_anded"] else "no-go-phase92-source-meter-clock-state-parity-structural-gates",
        "decision": "GO: all four raw-only diagnostic routes pass structural gates; stop before accuracy." if global_gates["all_gates_anded"] else "NO-GO: at least one raw-only diagnostic route fails a structural gate; stop before accuracy.",
        "truth_free": True,
        "accuracy_scored": False,
        "promotion_authorized": False,
        "stop_before_accuracy": True,
        "pre_raw_verification": pre_raw_report,
        "evaluator": evaluator_pin,
        "freeze": {"path": "docs/use_cases/records/smartphone_r5_phase92_source_meter_clock_state_parity_execution_freeze_v1.json", "sha256": "c07ac587ebcfde6391de0b8a824687316de854442166a93daadb416d4e9b74d8"},
        "manifest": {"path": relative(MANIFEST_PATH), "sha256": _hash_file(MANIFEST_PATH, "Phase92 manifest")},
        "implementation": {"commit": implementation.get("commit"), "binary": implementation.get("binary")},
        "matrix": {"candidate_count": 1, "routes": 4, "runs_per_route": 1, "controls": 0, "candidate_runs_total": 4, "raw_device_gnss_reads": 4, "raw_device_imu_reads": 4, "broadcast_navigation_reads": 4, "no_accuracy_route_selection": True},
        "routes": route_reports,
        "gates": {**global_gates, "all_passed": global_gates["all_gates_anded"]},
        "failed_gates": failed,
        "read_accounting": {
            "native_solver_invocations": 4,
            "raw_device_gnss_reads": 4,
            "raw_device_imu_reads": 4,
            "broadcast_navigation_reads": 4,
            "base_rinex_reads": 0,
            "truth_reads": 0,
            "mat_reads_or_generated": 0,
            "precomputed_coordinate_reads": 0,
            "validation_holdout_reads": 0,
            "kaggle_or_token_access": 0,
            "accuracy_scored": False,
            "route_score_selection": False,
            "evaluator_raw_input_reads": 0,
            "evaluator_truth_reads": 0,
            "evaluator_accuracy_calculations": 0,
            "raw_input_hash_reads": 0,
        },
    }
    return result


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--execute", action="store_true", help="run the sealed four-route matrix exactly once")
    parser.add_argument("--validate", action="store_true", help="validate an already completed output tree")
    parser.add_argument("--result-json", type=Path, default=STRUCTURAL_RESULT)
    args = parser.parse_args(argv)
    if args.execute == args.validate:
        parser.error("choose exactly one of --execute or --validate")
    try:
        pre_raw_report, manifest, raw_inputs, evaluator_pin = _verify_pre_raw_and_manifest()
        if args.execute:
            metadata = execute_matrix(manifest, raw_inputs)
        else:
            metadata = []
            for route in ROUTES:
                route_dir = OUTPUT_ROOT / route.replace("/", "__")
                metadata_path = route_dir / "run_metadata.json"
                metadata.append(_read_json(metadata_path, f"route metadata {route}"))
        result = evaluate(metadata, manifest, raw_inputs, pre_raw_report, evaluator_pin)
        _atomic_json(args.result_json.resolve(), result)
        print(json.dumps(result, indent=2, sort_keys=True))
        return 0 if result["gates"]["all_passed"] else 1
    except (StructuralError, OSError) as exc:
        print(f"phase92 structural runner: fail-closed: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
