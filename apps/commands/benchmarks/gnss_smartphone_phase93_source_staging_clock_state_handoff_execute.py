#!/usr/bin/env python3
"""Run and structurally seal the authorized Phase93 raw-only matrix.

The Phase93 evaluator is deliberately launch-free.  This companion is the
post-authorization runner: it verifies the pinned evaluator and manifest,
resolves only the three inherited raw input files, launches the frozen native
command once for each route in order, preserves stdout/stderr and partial
outputs, and inspects only native summary/submission files.  It never reads or
hashes truth, MAT, Kaggle/token, base, precomputed-coordinate, or accuracy
artifacts.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
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
PRE_RAW = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase93_source_staging_clock_state_handoff.py"
MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase93_source_staging_clock_state_handoff_execution_manifest_v1.json"
AUTHORIZATION = ROOT / "docs/use_cases/records/smartphone_r5_phase93_source_staging_clock_state_handoff_raw_execution_authorization_v1.json"
INHERITED_MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase91_source_clock_c0d_gnss_first_raw_drift_d_initializer_execution_manifest_v1.json"
BINARY = ROOT / "build/apps/gnss_fgo_imu_no_base"
OUTPUT_ROOT = ROOT / "output/smartphone-r5/phase93-source-staging-clock-state-handoff-v1"
RESULT_JSON = ROOT / "docs/use_cases/records/smartphone_r5_phase93_source_staging_clock_state_handoff_structural_result_v1.json"
RESULT_MD = RESULT_JSON.with_suffix(".md")

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
SELECTOR = "--native-source-clock-c0d-gnss-first-meter-state-handoff"
RAW_D_SELECTOR = "--native-source-clock-c0d-gnss-first-raw-drift-d-initializer"
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
    RAW_D_SELECTOR,
)
FORBIDDEN_PATH_TERMS = (
    ".mat", "ground_truth", "validation", "holdout", "kaggle", "token",
    "base.rinex", "coordinate", "truth",
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
    SELECTOR,
)
SPEED_OF_LIGHT = 299792458.0
EARTH_RADIUS_M = 6_371_000.0
MAX_SPEED_MPS = 70.0
SCHEMA = "smartphone-r5-phase93-source-staging-clock-state-handoff-structural-result.v1"
FREEZE_SHA = "6893bf21496b2ecf3afbe43ad94fc4cde81a84a2ac03b47fb08ef30a13047f55"
MANIFEST_SHA = "f5ef7cc946e1ef875a30546f32cee9d04a2aa82471f92767ab4935652cec4277"
EVALUATOR_SHA = "0bf2113fc484a72e7bcea2765381ded93fa8df930c5f95e1585fc03c2185f3ed"
TESTS_SHA = "f394c47aaf01fbdb2e98021f4a3b1aa027943d1ce676a13a1d1e410b796430b8"
SOURCE_FREEZE_SHA = "34ca3a2c6beecdb912aed81c469eca8a45d32e80bad69e9fc5fa12ea1b49ab88"
IMPLEMENTATION_COMMIT = "2613adb12fba80c468f8c6945ba360a126129f58"


class StructuralError(ValueError):
    """Fail-closed structural validation error."""


def fail(message: str) -> StructuralError:
    return StructuralError(message)


def relative(path: Path) -> str:
    try:
        return str(path.relative_to(ROOT))
    except ValueError:
        return str(path)


def reject_path(path: str | Path) -> None:
    lowered = str(path).lower()
    if any(term in lowered for term in FORBIDDEN_PATH_TERMS):
        raise fail(f"forbidden artifact path: {path}")


def read_json(path: Path, label: str) -> dict[str, Any]:
    reject_path(path)
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise fail(f"failed to read {label}: {path}: {exc}") from exc
    if not isinstance(value, dict):
        raise fail(f"{label} is not an object")
    return value


def sha256_file(path: Path, label: str) -> str:
    reject_path(path)
    if not path.is_file():
        raise fail(f"missing {label}: {path}")
    digest = hashlib.sha256()
    try:
        with path.open("rb") as handle:
            for block in iter(lambda: handle.read(1024 * 1024), b""):
                digest.update(block)
    except OSError as exc:
        raise fail(f"failed to hash {label}: {path}: {exc}") from exc
    return digest.hexdigest()


def atomic_json(path: Path, value: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, temporary = tempfile.mkstemp(prefix=f".{path.name}.", suffix=".tmp", dir=path.parent)
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as handle:
            json.dump(value, handle, indent=2, sort_keys=True, allow_nan=False)
            handle.write("\n")
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, path)
    except BaseException:
        try:
            os.unlink(temporary)
        except OSError:
            pass
        raise


def assert_equal(actual: Any, expected: Any, label: str) -> None:
    if actual != expected:
        raise fail(f"{label}: expected {expected!r}, got {actual!r}")


def finite_tree(value: Any, label: str) -> None:
    if value is None or isinstance(value, (bool, str)):
        return
    if isinstance(value, (int, float)):
        if not math.isfinite(float(value)):
            raise fail(f"non-finite value: {label}")
        return
    if isinstance(value, dict):
        for key, child in value.items():
            finite_tree(child, f"{label}.{key}")
        return
    if isinstance(value, list):
        for index, child in enumerate(value):
            finite_tree(child, f"{label}[{index}]")


def number(mapping: dict[str, Any], key: str, label: str, positive: bool = False) -> float:
    value = mapping.get(key)
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(float(value)):
        raise fail(f"missing/non-finite number: {label}/{key}")
    value = float(value)
    if positive and value <= 0.0:
        raise fail(f"non-positive number: {label}/{key}")
    return value


def count(mapping: dict[str, Any], key: str, label: str, minimum: int = 0) -> int:
    value = mapping.get(key)
    if isinstance(value, bool) or not isinstance(value, int) or value < minimum:
        raise fail(f"invalid count: {label}/{key}")
    return value


def verify_authorization(manifest: dict[str, Any]) -> dict[str, Any]:
    auth = read_json(AUTHORIZATION, "Phase93 raw execution authorization")
    assert_equal(auth.get("schema_version"), "smartphone-r5-phase93-source-staging-clock-state-handoff-raw-execution-authorization.v1", "authorization/schema_version")
    assert_equal(auth.get("status"), "authorized-for-exact-four-route-structural-execution", "authorization/status")
    assert_equal(auth.get("execution_label"), "Luna Max", "authorization/execution_label")
    authority = auth.get("authority")
    if not isinstance(authority, dict):
        raise fail("authorization/authority missing")
    pins = {
        "phase93_source_freeze": ("docs/use_cases/records/smartphone_r5_phase93_source_staging_clock_state_handoff_freeze_v1.json", SOURCE_FREEZE_SHA),
        "phase93_execution_freeze": (relative(ROOT / "docs/use_cases/records/smartphone_r5_phase93_source_staging_clock_state_handoff_execution_freeze_v1.json"), FREEZE_SHA),
        "phase93_execution_manifest": (relative(MANIFEST), MANIFEST_SHA),
        "pre_raw_evaluator": (relative(PRE_RAW), EVALUATOR_SHA),
        "pre_raw_tests": ("tests/test_smartphone_phase93_source_staging_clock_state_handoff.py", TESTS_SHA),
    }
    for key, (path_text, expected_sha) in pins.items():
        item = authority.get(key)
        if not isinstance(item, dict):
            raise fail(f"authorization authority pin missing: {key}")
        assert_equal(item.get("path"), path_text, f"authorization/authority/{key}/path")
        assert_equal(item.get("sha256"), expected_sha, f"authorization/authority/{key}/sha256")
    assert_equal(authority.get("implementation_commit"), IMPLEMENTATION_COMMIT, "authorization/authority/implementation_commit")

    candidate = auth.get("candidate")
    if not isinstance(candidate, dict):
        raise fail("authorization/candidate missing")
    for key, expected in {
        "candidate_count": 1,
        "id": "phase93_source_meter_c0d_gnss_first_retained_clock_state_handoff",
        "selector": SELECTOR,
        "default_off_outside_this_explicit_command": True,
        "diagnostic_only": True,
        "runs_per_route": 1,
        "controls": 0,
        "native_binary": relative(BINARY),
        "raw_input_names_exact": list(RAW_NAMES),
    }.items():
        assert_equal(candidate.get(key), expected, f"authorization/candidate/{key}")
    roles = candidate.get("raw_input_roles")
    assert_equal(roles, {"device_gnss.csv": "Android raw GNSS only", "device_imu.csv": "Android raw IMU only", "brdc.nav": "broadcast navigation only"}, "authorization/candidate/raw_input_roles")
    forbidden = candidate.get("forbidden_inputs_and_lanes")
    if not isinstance(forbidden, list) or not all(isinstance(item, str) for item in forbidden):
        raise fail("authorization forbidden lanes malformed")
    for term in ("truth", "MAT", "Kaggle/token", "base RINEX", "precomputed coordinates", "accuracy scoring", "submission release"):
        if term not in forbidden:
            raise fail(f"authorization forbidden lane missing: {term}")

    routes = auth.get("routes")
    if not isinstance(routes, list) or [item.get("dataset_id") for item in routes if isinstance(item, dict)] != list(ROUTES):
        raise fail("authorization/routes order or shape changed")
    if any(not isinstance(item, dict) or item.get("runs") != 1 or item.get("expected_output_rows") != DOMAIN_ROWS[item.get("dataset_id")] for item in routes):
        raise fail("authorization/routes run or expected-output contract changed")
    matrix = auth.get("matrix")
    if not isinstance(matrix, dict):
        raise fail("authorization/matrix missing")
    for key, expected in {
        "candidate_count": 1, "route_count": 4, "runs_per_route": 1,
        "total_native_invocations": 4, "controls": 0, "reruns": 0,
        "fallbacks": 0, "raw_device_gnss_reads_max": 4,
        "raw_device_imu_reads_max": 4, "broadcast_navigation_reads_max": 4,
        "base_rinex_reads": 0, "truth_reads": 0, "mat_reads_or_generated": 0,
        "precomputed_coordinate_reads": 0, "accuracy_calculations": 0,
        "kaggle_or_token_access": 0, "route_score_selection": False,
    }.items():
        assert_equal(matrix.get(key), expected, f"authorization/matrix/{key}")
    policy = auth.get("execution_policy")
    if not isinstance(policy, dict):
        raise fail("authorization/execution_policy missing")
    for key, expected in {
        "authorized": True, "commands_source": "use exact route commands from the pinned Phase93 execution manifest",
        "order": "sequential route order in manifest", "one_invocation_per_route": True,
        "no_rerun": True, "on_route_failure": "record fail-closed result and continue each remaining authorized route once; no fallback",
        "stop_after_four_routes": True, "outputs": "structural diagnostics only; never publish or score accuracy",
        "raw_input_hashes": "not requested before or outside native route reads",
    }.items():
        assert_equal(policy.get(key), expected, f"authorization/execution_policy/{key}")
    accounting = auth.get("read_accounting_before_execution")
    if not isinstance(accounting, dict):
        raise fail("authorization/read_accounting_before_execution missing")
    for key in ("native_solver_invocations", "raw_device_gnss_reads", "raw_device_imu_reads", "broadcast_navigation_reads", "base_rinex_reads", "truth_reads", "mat_reads_or_generated", "precomputed_coordinate_reads", "accuracy_calculations", "kaggle_or_token_access"):
        assert_equal(accounting.get(key), 0, f"authorization/read_accounting_before_execution/{key}")
    assert_equal(accounting.get("route_score_selection"), False, "authorization/read_accounting_before_execution/route_score_selection")
    boundary = auth.get("release_boundary")
    if not isinstance(boundary, dict):
        raise fail("authorization/release_boundary missing")
    for key, expected in {"raw_execution_authorized": True, "structural_result_authorized": True, "truth_or_accuracy_evaluation_authorized": False, "submission_release_authorized": False, "promotion_authorized": False}.items():
        assert_equal(boundary.get(key), expected, f"authorization/release_boundary/{key}")
    assert_equal(manifest.get("status"), "sealed-before-phase93-raw-execution", "manifest/status at authorization")
    return auth


def verify_pre_raw_and_manifest() -> tuple[dict[str, Any], dict[str, Any], dict[str, Any]]:
    sys.path.insert(0, str(PRE_RAW.parent))
    try:
        import gnss_smartphone_phase93_source_staging_clock_state_handoff as evaluator
    except ImportError as exc:
        raise fail(f"unable to import pinned Phase93 evaluator: {exc}") from exc
    try:
        pre_raw_report = evaluator.verify_pre_raw()
        manifest = evaluator.verify_manifest()
    except Exception as exc:
        raise fail(f"pinned Phase93 pre-raw verifier failed: {exc}") from exc
    if pre_raw_report.get("raw_reads") != 0 or pre_raw_report.get("native_solver_invocations") != 0:
        raise fail("pre-raw verifier reported raw/native I/O")
    if pre_raw_report.get("accuracy_scored") is not False:
        raise fail("pre-raw verifier reported accuracy")
    assert_equal(sha256_file(PRE_RAW, "pre-raw evaluator"), EVALUATOR_SHA, "pre-raw evaluator hash")
    assert_equal(sha256_file(MANIFEST, "Phase93 execution manifest"), MANIFEST_SHA, "Phase93 execution manifest hash")
    authorization = verify_authorization(manifest)
    return pre_raw_report, manifest, {"path": relative(PRE_RAW), "sha256": EVALUATOR_SHA, "authorization": {"path": relative(AUTHORIZATION), "sha256": sha256_file(AUTHORIZATION, "raw execution authorization")}}


def load_raw_inputs() -> dict[str, dict[str, dict[str, Any]]]:
    """Read only the inherited manifest; raw bytes stay with the native process."""
    inherited = read_json(INHERITED_MANIFEST, "inherited raw-input manifest")
    routes = inherited.get("routes")
    if not isinstance(routes, list) or [item.get("dataset_id") for item in routes] != list(ROUTES):
        raise fail("inherited raw route order changed")
    resolved: dict[str, dict[str, dict[str, Any]]] = {}
    for item in routes:
        route = item.get("dataset_id")
        raw = item.get("raw_inputs")
        if route not in ROUTES or not isinstance(raw, dict) or tuple(raw) != RAW_NAMES:
            raise fail(f"inherited raw set changed: {route}")
        resolved[route] = {}
        for name in RAW_NAMES:
            pin = raw.get(name)
            if not isinstance(pin, dict) or not isinstance(pin.get("path"), str):
                raise fail(f"malformed inherited raw pin: {route}/{name}")
            path_text = pin["path"]
            reject_path(path_text)
            path_obj = Path(path_text)
            if path_obj.is_absolute() or ".." in path_obj.parts or path_obj.name != name:
                raise fail(f"unsafe inherited raw path: {route}/{name}")
            path = ROOT / path_text
            if not path.is_file():
                raise fail(f"missing inherited raw input: {path}")
            digest = pin.get("sha256")
            if not isinstance(digest, str) or len(digest) != 64:
                raise fail(f"malformed inherited raw SHA pin: {route}/{name}")
            # This stat is only existence/provenance validation.  Deliberately
            # do not hash or open the raw file before native execution.
            resolved[route][name] = {"path": path_text, "sha256": digest, "bytes": path.stat().st_size, "manifest": relative(INHERITED_MANIFEST)}
    return resolved


def materialize_command(record: dict[str, Any], raw: dict[str, dict[str, Any]]) -> list[str]:
    route = record.get("dataset_id")
    command = record.get("command")
    if route not in ROUTES or not isinstance(command, list) or not all(isinstance(token, str) for token in command):
        raise fail(f"invalid sealed command: {route}")
    command = list(command)
    for flag in REQUIRED_FLAGS:
        if command.count(flag) != 1:
            raise fail(f"required flag multiplicity failed: {route}/{flag}")
    for flag in FORBIDDEN_FLAGS:
        if flag in command:
            raise fail(f"forbidden flag present: {route}/{flag}")
    if command[0] != relative(BINARY) or command.count("--dataset-id") != 1 or command[command.index("--dataset-id") + 1] != route:
        raise fail(f"sealed binary/dataset changed: {route}")
    for flag, name in RAW_FLAGS:
        if command.count(flag) != 1:
            raise fail(f"raw flag multiplicity failed: {route}/{flag}")
        index = command.index(flag) + 1
        placeholder = f"raw/phase93/{route}/{name}"
        if command[index] != placeholder:
            raise fail(f"sealed raw placeholder changed: {route}/{name}")
        command[index] = raw[name]["path"]
    planned = record.get("planned_output")
    if not isinstance(planned, dict):
        raise fail(f"planned output missing: {route}")
    for flag, key in (("--out", "structural_output"), ("--summary-json", "summary")):
        if command.count(flag) != 1 or command[command.index(flag) + 1] != planned.get(key):
            raise fail(f"sealed output path changed: {route}/{key}")
    for token in command:
        reject_path(token)
    return command


def execute_matrix(manifest: dict[str, Any], raw_inputs: dict[str, dict[str, dict[str, Any]]]) -> list[dict[str, Any]]:
    if OUTPUT_ROOT.exists():
        raise fail(f"refusing to overwrite existing Phase93 output root: {OUTPUT_ROOT}")
    OUTPUT_ROOT.mkdir(parents=True, exist_ok=False)
    records = manifest.get("routes")
    if not isinstance(records, list) or [item.get("dataset_id") for item in records] != list(ROUTES):
        raise fail("sealed Phase93 route order/count changed")
    env = os.environ.copy()
    local_lib = "/home/sasaki/.local/lib"
    env["LD_LIBRARY_PATH"] = local_lib + ((":" + env["LD_LIBRARY_PATH"]) if env.get("LD_LIBRARY_PATH") else "")
    metadata: list[dict[str, Any]] = []
    for record in records:
        route = record["dataset_id"]
        planned = record["planned_output"]
        output_path = ROOT / planned["structural_output"]
        summary_path = ROOT / planned["summary"]
        route_dir = output_path.parent
        route_dir.mkdir(parents=True, exist_ok=False)
        command = materialize_command(record, raw_inputs[route])
        stdout_path, stderr_path = route_dir / "stdout.log", route_dir / "stderr.log"
        start = time.time()
        return_code: int | None = None
        interrupted = False
        launch_error = ""
        with stdout_path.open("wb") as stdout, stderr_path.open("wb") as stderr:
            try:
                completed = subprocess.run(command, cwd=ROOT, env=env, stdout=stdout, stderr=stderr, check=False)
                return_code = completed.returncode
            except KeyboardInterrupt:
                interrupted = True
                stderr.write(b"\nPhase93 runner interrupted; partial native output preserved.\n")
            except OSError as exc:
                launch_error = str(exc)
                stderr.write(f"\nPhase93 native launch failed: {exc}\n".encode())
        item = {
            "schema_version": "smartphone-r5-phase93-source-staging-clock-state-handoff-route-execution.v1",
            "dataset_id": route,
            "run_number": 1,
            "command": command,
            "raw_inputs": raw_inputs[route],
            "planned_output": {"structural_output": relative(output_path), "summary": relative(summary_path)},
            "stdout": relative(stdout_path),
            "stderr": relative(stderr_path),
            "started_unix_s": start,
            "ended_unix_s": time.time(),
            "return_code": return_code,
            "interrupted": interrupted,
            "launch_error": launch_error,
        }
        atomic_json(route_dir / "run_metadata.json", item)
        metadata.append(item)
    return metadata


def read_submission(path: Path, route: str) -> tuple[list[tuple[int, float, float]], dict[str, Any]]:
    if not path.is_file():
        raise fail(f"missing native structural output: {route}")
    rows: list[tuple[int, float, float]] = []
    expected_header = ["phone", "UnixTimeMillis", "LatitudeDegrees", "LongitudeDegrees"]
    try:
        with path.open("r", encoding="utf-8", newline="") as handle:
            reader = csv.reader(handle)
            header = next(reader, None)
            if header != expected_header:
                raise fail(f"output header changed: {route}")
            for line, values in enumerate(reader, 2):
                if len(values) != 4:
                    raise fail(f"output column count changed: {route}/line{line}")
                if values[0] != route:
                    raise fail(f"output route identity changed: {route}/line{line}")
                try:
                    timestamp = int(values[1])
                    latitude = float(values[2])
                    longitude = float(values[3])
                except ValueError as exc:
                    raise fail(f"non-numeric output row: {route}/line{line}") from exc
                if not math.isfinite(latitude) or not math.isfinite(longitude) or not -90.0 <= latitude <= 90.0 or not -180.0 <= longitude <= 180.0:
                    raise fail(f"non-finite/out-of-earth output row: {route}/line{line}")
                if rows and timestamp <= rows[-1][0]:
                    raise fail(f"output timestamps not strictly increasing: {route}/line{line}")
                rows.append((timestamp, latitude, longitude))
    except OSError as exc:
        raise fail(f"failed to read native output: {route}: {exc}") from exc
    expected = DOMAIN_ROWS[route]
    if len(rows) != expected:
        raise fail(f"output epoch coverage failed: {route}: {len(rows)} != {expected}")
    return rows, {"rows": len(rows), "header": expected_header, "finite_earth_valid": True, "path": relative(path), "bytes": path.stat().st_size, "sha256": sha256_file(path, f"structural output {route}")}


def speed_report(rows: list[tuple[int, float, float]]) -> dict[str, Any]:
    maximum = 0.0
    over_bound = 0
    for previous, current in zip(rows, rows[1:]):
        dt = (current[0] - previous[0]) / 1000.0
        if dt <= 0.0:
            raise fail("non-positive output time interval")
        lat1, lon1, lat2, lon2 = map(math.radians, (previous[1], previous[2], current[1], current[2]))
        a = math.sin((lat2 - lat1) / 2.0) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin((lon2 - lon1) / 2.0) ** 2
        distance = 2.0 * EARTH_RADIUS_M * math.asin(min(1.0, math.sqrt(max(0.0, a))))
        speed = distance / dt
        if not math.isfinite(speed):
            raise fail("non-finite output speed")
        maximum = max(maximum, speed)
        if speed > MAX_SPEED_MPS:
            over_bound += 1
    return {"finite": True, "max_mps": maximum, "over_70_mps_count": over_bound, "threshold_mps": MAX_SPEED_MPS}


def validate_handoff(summary: dict[str, Any], route: str, expected: int) -> dict[str, Any]:
    handoff = summary.get("gnss_first")
    if not isinstance(handoff, dict):
        raise fail(f"GNSS-first handoff telemetry missing: {route}")
    required = {
        "attempted": True,
        "converged": True,
        "handoff_mode": "gnss-first-in-memory-meter-clock-state",
        "velocity_handoff_source": "same-run-gnss-first-optimizer-result",
        "position_clock_handoff_source": "same-run-gnss-first-optimizer-result",
        "clock_drift_handoff_source": "same-run-gnss-first-optimizer-result",
        "clock_drift_unit": "metres_per_second",
        "optimized_d_export_valid": True,
        "epoch_identity_alignment_valid": True,
        "coordinates_source": "in-memory GNSS-first result only",
        "forbidden_coordinate_sources": ["PDC", "direct-WLS", "external", "precomputed"],
        "failure": "",
    }
    for key, value in required.items():
        assert_equal(handoff.get(key), value, f"GNSS-first handoff/{route}/{key}")
    exact = ("epochs", "optimized_d_epoch_count", "optimized_d_finite_count", "main_epoch_count", "gnss_first_epoch_count", "solution_epoch_count", "raw_epoch_count", "raw_utc_key_count", "aligned_epoch_count", "positions_clocks_copied")
    for key in exact:
        assert_equal(count(handoff, key, f"GNSS-first handoff/{route}"), expected, f"GNSS-first handoff/{route}/{key}")
    assert_equal(count(handoff, "optimized_d_nonfinite_count", f"GNSS-first handoff/{route}"), 0, f"GNSS-first handoff/{route}/optimized_d_nonfinite_count")
    iterations = count(handoff, "iterations", f"GNSS-first/{route}", minimum=1)
    accepted = count(handoff, "c0d_accepted_outer_iterations", f"GNSS-first/{route}", minimum=1)
    initial = number(handoff, "initial_cost", f"GNSS-first/{route}")
    final = number(handoff, "final_cost", f"GNSS-first/{route}")
    if not final < initial or handoff.get("c0d_active_solve_finite_costs") is not True:
        raise fail(f"GNSS-first active solve gate failed: {route}")
    return {"epochs": expected, "iterations": iterations, "accepted_outer_iterations": accepted, "initial_cost": initial, "final_cost": final, "strict_cost_decrease": True, "optimized_d_epoch_count": handoff["optimized_d_epoch_count"], "optimized_d_finite_count": handoff["optimized_d_finite_count"], "optimized_d_nonfinite_count": handoff["optimized_d_nonfinite_count"], "exact_key_alignment": True, "positions_clocks_copied": handoff["positions_clocks_copied"]}


def validate_c0d(c0d: dict[str, Any], graph: dict[str, Any], route: str, expected_rows: int) -> dict[str, Any]:
    for key, expected in {
        "clock_c0d_enabled": True, "gnss_first_meter_state_handoff_enabled": True,
        "meter_state_parity_enabled": True, "internal_clock_state_unit": "metres",
        "internal_isb_state_unit": "metres", "internal_drift_state_unit": "metres_per_second",
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
    }.items():
        assert_equal(c0d.get(key), expected, f"C0/D contract/{route}/{key}")
    factor_count = count(c0d, "clock_c0d_factor_count", route, minimum=1)
    skip_keys = ("clock_c0d_clock_jump_skips", "clock_c0d_gap_skips", "clock_c0d_invalid_dt_skips", "clock_c0d_phone_exclusion_skips")
    skips = {key: count(c0d, key, route) for key in skip_keys}
    if skips["clock_c0d_phone_exclusion_skips"] != 0 or factor_count + sum(skips.values()) != expected_rows:
        raise fail(f"C0/D factor/official skip accounting failed: {route}")
    dt_min = number(c0d, "clock_c0d_dt_min_s", route, positive=True)
    dt_max = number(c0d, "clock_c0d_dt_max_s", route, positive=True)
    if not dt_min <= dt_max < 1.5:
        raise fail(f"C0/D dt telemetry failed: {route}")
    if not math.isclose(number(c0d, "speed_of_light_mps", route), SPEED_OF_LIGHT, rel_tol=0.0, abs_tol=1e-9) or not math.isclose(number(c0d, "clock_c0d_sigma_seconds", route), 0.1 / SPEED_OF_LIGHT, rel_tol=1e-12, abs_tol=1e-18):
        raise fail(f"C0/D unit conversion failed: {route}")
    for key in ("active_solve_initial_cost", "active_solve_final_cost", "initial_lambda", "maximum_lambda", "final_lambda", "max_whitened_clock_column_norm", "max_whitened_drift_column_norm", "conditioning_proxy"):
        number(c0d, key, route, positive=(key.endswith("lambda") or key == "conditioning_proxy"))
    for key in ("active_solve_diagnostic_enabled", "active_solve_attempted", "active_solve_finite_costs", "termination_trace_complete"):
        if c0d.get(key) is not True:
            raise fail(f"active C0/D telemetry incomplete: {route}/{key}")
    accepted = count(c0d, "accepted_outer_iterations", route, minimum=1)
    attempts = count(c0d, "total_inner_lambda_attempts", route)
    indeterminate = count(c0d, "indeterminate_linear_solve_count", route)
    for key in ("unsuccessful_model_step_count", "small_cost_change_stop_count", "maximum_lambda_stop_count"):
        count(c0d, key, route)
    if attempts < accepted + indeterminate:
        raise fail(f"C0/D inner-attempt accounting failed: {route}")
    initial = number(c0d, "active_solve_initial_cost", route)
    final = number(c0d, "active_solve_final_cost", route)
    graph_initial = number(graph, "initial_cost", route)
    graph_final = number(graph, "final_cost", route)
    iterations = count(graph, "iterations", route, minimum=1)
    if graph.get("converged") is not True or graph_initial != initial or graph_final != final or iterations != accepted or not final < initial:
        raise fail(f"main active solve gate failed: {route}")
    return {"factor_count": factor_count, **skips, "dt_min_s": dt_min, "dt_max_s": dt_max, "accepted_outer_iterations": accepted, "total_inner_lambda_attempts": attempts, "initial_cost": initial, "final_cost": final, "iterations": iterations, "strict_cost_decrease": True, "conditioning_proxy": c0d["conditioning_proxy"], "initial_lambda": c0d["initial_lambda"], "final_lambda": c0d["final_lambda"], "maximum_lambda": c0d["maximum_lambda"], "official_sigma_m": c0d["clock_c0d_sigma_m"]}


def validate_summary(summary: dict[str, Any], route: str, raw: dict[str, dict[str, Any]]) -> dict[str, Any]:
    finite_tree(summary, f"summary[{route}]")
    expected_epochs = DOMAIN_ROWS[route] + 1
    for key, expected in {"dataset_id": route, "status": "imu-combined-factor", "truth_used": False, "base_factors": False, "no_base_contract": True, "production_default_changed": False, "native_pdc_state_bridge": False, "native_upstream_quality": False, "native_source_clock_c0d_factor_enabled": True, "native_source_clock_c0d_meter_state_parity_enabled": True, "native_source_clock_c0d_active_solve_diagnostic_enabled": True, "native_source_clock_c0d_gnss_first_raw_drift_d_initializer_enabled": False, "native_source_clock_c0d_gnss_first_meter_state_handoff_enabled": True}.items():
        assert_equal(summary.get(key), expected, f"provenance/{route}/{key}")
    inputs = summary.get("inputs")
    if not isinstance(inputs, dict):
        raise fail(f"summary inputs missing: {route}")
    assert_equal(inputs.get("observation"), None, f"summary inputs/{route}/observation")
    for key, name in (("android_gnss", "device_gnss.csv"), ("imu", "device_imu.csv"), ("navigation", "brdc.nav")):
        assert_equal(inputs.get(key), raw[name]["path"], f"summary inputs/{route}/{key}")
    direct = summary.get("native_source_direct_observable_quality")
    if not isinstance(direct, dict):
        raise fail(f"direct quality telemetry missing: {route}")
    for key, expected in {"enabled": True, "direct_no_pdc": True, "pdc_bridge": False, "native_pdc_state_bridge": False}.items():
        assert_equal(direct.get(key), expected, f"direct quality/{route}/{key}")
    android = summary.get("android_gnss_diagnostics")
    if not isinstance(android, dict) or android.get("no_device_wls_seed") is not True:
        raise fail(f"raw GNSS/no-device-WLS contract failed: {route}")
    imu = summary.get("imu_initialization")
    if not isinstance(imu, dict) or imu.get("input_format") != "android-device_imu.csv":
        raise fail(f"raw IMU input contract failed: {route}")
    epochs, graph, utc, c0d = (summary.get(key) for key in ("epochs", "graph", "raw_utc_key_contract", "native_source_clock_c0d_factor"))
    if not all(isinstance(item, dict) for item in (epochs, graph, utc, c0d)):
        raise fail(f"structural telemetry missing: {route}")
    assert isinstance(epochs, dict) and isinstance(graph, dict) and isinstance(utc, dict) and isinstance(c0d, dict)
    assert_equal(count(epochs, "problem", route), expected_epochs, f"epochs/problem/{route}")
    assert_equal(count(epochs, "output", route), expected_epochs, f"epochs/output/{route}")
    assert_equal(count(graph, "imu_intervals", route), DOMAIN_ROWS[route], f"graph/imu_intervals/{route}")
    for key, expected in {"warmup_epoch_excluded": True, "raw_epoch_keys": expected_epochs, "target_epochs": DOMAIN_ROWS[route], "exact_solution_epochs": DOMAIN_ROWS[route], "interpolated_epochs": 0, "edge_hold_epochs": 0, "unresolved_epochs": 0, "device_wls_coordinates_used": False}.items():
        assert_equal(utc.get(key), expected, f"raw UTC contract/{route}/{key}")
    handoff = validate_handoff(summary, route, expected_epochs)
    clock = validate_c0d(c0d, graph, route, DOMAIN_ROWS[route])
    # The native Phase93 summary deliberately reports the main graph's raw-D
    # initializer as disabled: the only D source in this command is the
    # optimized GNSS-first vector.  Full finite D coverage is represented by
    # the explicit handoff export counts above; no raw vector is re-opened by
    # this evaluator.
    raw_d_main = c0d.get("raw_drift_d_initializer")
    if not isinstance(raw_d_main, dict) or raw_d_main.get("enabled") is not False:
        raise fail(f"legacy main raw-D initializer was enabled/leaked: {route}")
    return {"expected_epochs": expected_epochs, "expected_output_rows": DOMAIN_ROWS[route], "gnss_first": handoff, "main": {"graph_iterations": clock["iterations"], "accepted_outer_iterations": clock["accepted_outer_iterations"], "initial_cost": clock["initial_cost"], "final_cost": clock["final_cost"], "strict_cost_decrease": True}, "optimized_C": {"finite_full_coverage": True, "epoch_count": expected_epochs, "source": "same-run GNSS-first position/clock handoff"}, "optimized_D": {"finite_full_coverage": True, "epoch_count": handoff["optimized_d_epoch_count"], "finite_count": handoff["optimized_d_finite_count"], "nonfinite_count": handoff["optimized_d_nonfinite_count"], "source": "FGOResult.epoch_clock_drift_mps; exact retained source-key order"}, "clock_c0d": clock, "raw_utc": {key: utc[key] for key in ("raw_epoch_keys", "target_epochs", "exact_solution_epochs", "interpolated_epochs", "edge_hold_epochs", "unresolved_epochs", "device_wls_coordinates_used")}}


def route_report(metadata: dict[str, Any], raw: dict[str, dict[str, Any]]) -> dict[str, Any]:
    route = metadata["dataset_id"]
    planned = metadata.get("planned_output", {})
    summary_path = ROOT / planned.get("summary", "")
    output_path = ROOT / planned.get("structural_output", "")
    stdout_path = ROOT / metadata.get("stdout", "")
    stderr_path = ROOT / metadata.get("stderr", "")
    report: dict[str, Any] = {"dataset_id": route, "run_number": metadata.get("run_number"), "return_code": metadata.get("return_code"), "interrupted": metadata.get("interrupted"), "launch_error": metadata.get("launch_error", ""), "command": metadata.get("command"), "raw_inputs": raw}
    try:
        report["stdout"] = {"path": relative(stdout_path), "bytes": stdout_path.stat().st_size, "sha256": sha256_file(stdout_path, f"stdout {route}")}
        report["stderr"] = {"path": relative(stderr_path), "bytes": stderr_path.stat().st_size, "sha256": sha256_file(stderr_path, f"stderr {route}")}
    except (OSError, StructuralError) as exc:
        report["failure"] = str(exc)
        report["gates"] = {name: False for name in ("return_code_zero", "expected_output_coverage", "gnss_first_iterations_and_strict_cost_decrease", "main_iterations_and_strict_cost_decrease", "optimized_C_D_finite_full_exact_handoff", "meter_clock_isb_units_and_official_sigma", "c0d_skip_conditioning_lambda_telemetry", "finite_earth_valid_output_and_speed", "no_pdc_external_or_precomputed_coordinates")}
        report["all_gates_anded"] = False
        return report
    gate_names = ("return_code_zero", "expected_output_coverage", "gnss_first_iterations_and_strict_cost_decrease", "main_iterations_and_strict_cost_decrease", "optimized_C_D_finite_full_exact_handoff", "meter_clock_isb_units_and_official_sigma", "c0d_skip_conditioning_lambda_telemetry", "finite_earth_valid_output_and_speed", "no_pdc_external_or_precomputed_coordinates")
    gates = {name: False for name in gate_names}
    try:
        if metadata.get("return_code") != 0 or metadata.get("interrupted") or metadata.get("launch_error"):
            raise fail(f"native route did not return zero: {route}/{metadata.get('return_code')}")
        summary = read_json(summary_path, f"native summary {route}")
        rows, output = read_submission(output_path, route)
        diagnostics = validate_summary(summary, route, raw)
        speed = speed_report(rows)
        gates["return_code_zero"] = True
        gates["expected_output_coverage"] = output["rows"] == DOMAIN_ROWS[route] and diagnostics["expected_epochs"] == DOMAIN_ROWS[route] + 1
        gates["gnss_first_iterations_and_strict_cost_decrease"] = diagnostics["gnss_first"]["accepted_outer_iterations"] > 0 and diagnostics["gnss_first"]["strict_cost_decrease"]
        gates["main_iterations_and_strict_cost_decrease"] = diagnostics["main"]["accepted_outer_iterations"] > 0 and diagnostics["main"]["strict_cost_decrease"]
        gates["optimized_C_D_finite_full_exact_handoff"] = diagnostics["optimized_C"]["finite_full_coverage"] and diagnostics["optimized_D"]["finite_full_coverage"] and diagnostics["optimized_D"]["nonfinite_count"] == 0 and diagnostics["gnss_first"]["exact_key_alignment"]
        gates["meter_clock_isb_units_and_official_sigma"] = diagnostics["clock_c0d"]["factor_count"] > 0 and diagnostics["clock_c0d"]["official_sigma_m"] == 0.1
        gates["c0d_skip_conditioning_lambda_telemetry"] = all(math.isfinite(float(diagnostics["clock_c0d"][key])) for key in ("conditioning_proxy", "initial_lambda", "final_lambda", "maximum_lambda")) and diagnostics["clock_c0d"]["accepted_outer_iterations"] > 0
        gates["finite_earth_valid_output_and_speed"] = output["finite_earth_valid"] and speed["finite"] and speed["over_70_mps_count"] == 0
        gates["no_pdc_external_or_precomputed_coordinates"] = summary.get("native_pdc_state_bridge") is False and summary.get("native_upstream_quality") is False and summary.get("gnss_first", {}).get("coordinates_source") == "in-memory GNSS-first result only"
        report.update({"summary": {"path": relative(summary_path), "bytes": summary_path.stat().st_size, "sha256": sha256_file(summary_path, f"native summary {route}")}, "output": output, "speed": speed, "diagnostics": diagnostics})
    except (StructuralError, OSError, KeyError, TypeError, ValueError) as exc:
        report["failure"] = str(exc)
    report["gates"] = gates
    report["all_gates_anded"] = all(gates.values())
    return report


def build_result(metadata: list[dict[str, Any]], manifest: dict[str, Any], raw: dict[str, dict[str, dict[str, Any]]], pre_raw: dict[str, Any], evaluator_pin: dict[str, Any], authorization: dict[str, Any]) -> dict[str, Any]:
    routes = {item["dataset_id"]: route_report(item, raw[item["dataset_id"]]) for item in metadata}
    matrix_exact = len(metadata) == 4 and [item.get("dataset_id") for item in metadata] == list(ROUTES) and all(item.get("run_number") == 1 for item in metadata)
    route_gate_names = ("return_code_zero", "expected_output_coverage", "gnss_first_iterations_and_strict_cost_decrease", "main_iterations_and_strict_cost_decrease", "optimized_C_D_finite_full_exact_handoff", "meter_clock_isb_units_and_official_sigma", "c0d_skip_conditioning_lambda_telemetry", "finite_earth_valid_output_and_speed", "no_pdc_external_or_precomputed_coordinates")
    global_gates = {"implementation_and_binary_pins": manifest.get("implementation", {}).get("commit") == IMPLEMENTATION_COMMIT and sha256_file(BINARY, "implementation binary") == manifest.get("implementation", {}).get("binary", {}).get("sha256"), "exactly_four_routes_one_run_each": matrix_exact}
    for name in route_gate_names:
        global_gates[name] = bool(routes) and all(item.get("gates", {}).get(name) is True for item in routes.values())
    global_gates["accuracy_not_scored"] = True
    global_gates["truth_free"] = True
    global_gates["all_gates_anded"] = all(global_gates.values())
    failed = [key for key, value in global_gates.items() if value is False]
    for route, item in routes.items():
        failed.extend(f"{route}:{key}" for key, value in item.get("gates", {}).items() if value is False)
    auth_sha = sha256_file(AUTHORIZATION, "raw execution authorization")
    result = {
        "schema_version": SCHEMA,
        "phase": 93,
        "execution_label": "Luna Max",
        "status": "go-phase93-source-staging-clock-state-handoff-structural" if global_gates["all_gates_anded"] else "no-go-phase93-source-staging-clock-state-handoff-structural-gates",
        "decision": "GO: all four authorized raw-only structural routes pass; stop before truth/accuracy." if global_gates["all_gates_anded"] else "NO-GO: one or more authorized raw-only structural gates failed; fail closed before truth/accuracy.",
        "truth_free": True,
        "accuracy_scored": False,
        "promotion_authorized": False,
        "stop_before_truth_accuracy_submission": True,
        "authorization": {"path": relative(AUTHORIZATION), "sha256": auth_sha, "status": authorization.get("status")},
        "pre_raw_verification": pre_raw,
        "evaluator": evaluator_pin,
        "freeze": {"path": "docs/use_cases/records/smartphone_r5_phase93_source_staging_clock_state_handoff_execution_freeze_v1.json", "sha256": FREEZE_SHA},
        "manifest": {"path": relative(MANIFEST), "sha256": MANIFEST_SHA},
        "implementation": {"commit": IMPLEMENTATION_COMMIT, "binary": manifest.get("implementation", {}).get("binary")},
        "execution_wrapper": {"path": relative(Path(__file__).resolve()), "sha256": sha256_file(Path(__file__).resolve(), "Phase93 execution wrapper")},
        "matrix": {"candidate_count": 1, "routes": 4, "runs_per_route": 1, "controls": 0, "candidate_runs_total": 4, "native_solver_invocations": 4, "raw_device_gnss_reads": 4, "raw_device_imu_reads": 4, "broadcast_navigation_reads": 4, "base_rinex_reads": 0, "truth_reads": 0, "mat_reads_or_generated": 0, "precomputed_coordinate_reads": 0, "accuracy_calculations": 0, "kaggle_or_token_access": 0, "route_score_selection": False, "reruns": 0, "fallbacks": 0},
        "routes": routes,
        "gates": {**global_gates, "all_passed": global_gates["all_gates_anded"]},
        "failed_gates": failed,
        "read_accounting": {"native_solver_invocations": 4, "raw_device_gnss_reads": 4, "raw_device_imu_reads": 4, "broadcast_navigation_reads": 4, "base_rinex_reads": 0, "truth_reads": 0, "mat_reads_or_generated": 0, "precomputed_coordinate_reads": 0, "validation_holdout_reads": 0, "kaggle_or_token_access": 0, "accuracy_calculations": 0, "accuracy_scored": False, "route_score_selection": False, "reruns": 0, "fallbacks": 0, "evaluator_raw_input_reads": 0, "evaluator_truth_reads": 0, "evaluator_accuracy_calculations": 0, "raw_input_hash_reads": 0},
    }
    return result


def result_markdown(result: dict[str, Any]) -> str:
    lines = [f"# Phase93 raw structural result", "", f"- Status: `{result['status']}`", f"- Decision: {result['decision']}", "- Truth/accuracy/MAT/Kaggle/base/precomputed-coordinate reads: **0**", "- Matrix: exactly four routes × one native invocation, sequential, no controls/reruns/fallbacks", "", "## Route gates", "", "| Route | Return | GNSS-first accepted | Main accepted | Initial → final (main) | All gates |", "|---|---:|---:|---:|---:|---|"]
    for route in ROUTES:
        item = result["routes"].get(route, {})
        d = item.get("diagnostics", {})
        g = item.get("gates", {})
        gnss = d.get("gnss_first", {})
        main = d.get("main", {})
        lines.append(f"| `{route}` | `{item.get('return_code')}` | `{gnss.get('accepted_outer_iterations', 'n/a')}` | `{main.get('accepted_outer_iterations', 'n/a')}` | `{main.get('initial_cost', 'n/a')} → {main.get('final_cost', 'n/a')}` | `{item.get('all_gates_anded', False)}` |")
        if item.get("failure"):
            lines.append(f"  - Failure: `{item['failure']}`")
        if d:
            c = d.get("clock_c0d", {})
            lines.append(f"  - C0/D factors/skips: `{c.get('factor_count')}` / gap `{c.get('clock_c0d_gap_skips')}`, jump `{c.get('clock_c0d_clock_jump_skips')}`, invalid-dt `{c.get('clock_c0d_invalid_dt_skips')}`, phone `{c.get('clock_c0d_phone_exclusion_skips')}`; conditioning `{c.get('conditioning_proxy')}`; lambda `{c.get('initial_lambda')} → {c.get('final_lambda')}`.")
            lines.append(f"  - Optimized D handoff: `{d.get('optimized_D', {})}`")
            lines.append(f"  - Output/speed: `{item.get('output', {}).get('rows', 'n/a')}` rows, max `{item.get('speed', {}).get('max_mps', 'n/a')}` m/s.")
    lines.extend(["", "## Gate summary", "", "```json", json.dumps(result["gates"], indent=2, sort_keys=True), "```", "", "Structural result only; no accuracy or submission release is authorized.", ""])
    return "\n".join(lines)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--execute", action="store_true", help="execute exact four-route matrix once")
    parser.add_argument("--validate", action="store_true", help="validate preserved completed output tree")
    parser.add_argument("--result-json", type=Path, default=RESULT_JSON)
    args = parser.parse_args(argv)
    if args.execute == args.validate:
        parser.error("choose exactly one of --execute or --validate")
    try:
        pre_raw, manifest, evaluator_pin = verify_pre_raw_and_manifest()
        authorization = read_json(AUTHORIZATION, "Phase93 raw execution authorization")
        raw = load_raw_inputs()
        if args.execute:
            metadata = execute_matrix(manifest, raw)
        else:
            metadata = []
            for route in ROUTES:
                path = OUTPUT_ROOT / route.replace("/", "__") / "run_metadata.json"
                metadata.append(read_json(path, f"route metadata {route}"))
        result = build_result(metadata, manifest, raw, pre_raw, evaluator_pin, authorization)
        target = args.result_json.resolve()
        atomic_json(target, result)
        markdown_target = target.with_suffix(".md")
        markdown_target.write_text(result_markdown(result), encoding="utf-8")
        print(json.dumps(result, indent=2, sort_keys=True))
        return 0 if result["gates"]["all_passed"] else 1
    except (StructuralError, OSError) as exc:
        print(f"phase93 structural runner: fail-closed: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
