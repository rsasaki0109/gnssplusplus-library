#!/usr/bin/env python3
"""Truth-free Phase39 velocity-only GNSS-first handoff matrix.

The candidate keeps the existing raw Android GNSS/IMU graph and asks the
GNSS-first pass for optimized Doppler velocity states only.  This runner
never opens a truth member, a MATLAB file, a result coordinate, or a holdout.
It runs the candidate and the flag-off control on the frozen six-route raw
cohort, validates the predeclared structural gates, and publishes a seal.
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
FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_phase39_gnss_first_velocity_only_handoff_freeze_v1.json"
FREEZE_SHA256 = "17e75c367ae05e10ba5349b0db73cbfa048a27992286ae48b5cdf77346053580"
BINARY = ROOT / "build/apps/gnss_fgo_imu_no_base"
DEFAULT_OUTPUT = ROOT / "output/smartphone-r5/phase39-gnss-first-velocity-only-handoff-v1"
RESULT_NAME = "phase39_structural_seal.json"
MANIFEST_NAME = "phase39_structural_seal.manifest.json"
FAILURE_NAME = "phase39_structural_failure.json"

PHASE25_RAW_ROOT = ROOT / "output/smartphone-r5/phase25-raw-clock-eval-v1/raw"
PHASE37_RAW_ROOT = ROOT / "output/smartphone-r5/phase37-pixel5-repeatability-v1/routes"

BASE_FLAGS = (
    "--all-epochs",
    "--android-raw-utc-keys",
    "--android-raw-clock-only",
    "--android-utc-wall-clock-fallback",
    "--native-pdc-imu-tdcp-no-bridge",
    "--native-quality-anchor",
)
CANDIDATE_FLAG = "--native-gnss-first-velocity-only-handoff"
MAX_SPEED_MPS = 70.0
EARTH_RADIUS_M = 6_371_000.0
RAW_FILES = {
    "device_gnss": "device_gnss.csv",
    "device_imu": "device_imu.csv",
    "broadcast_nav": "brdc.nav",
}
PHASE25_ROUTES = {
    "2021-03-16-18-59-us-ca-mtv-a/pixel5",
    "2021-07-27-19-49-us-ca-mtv-b/pixel4",
    "2021-07-14-20-50-us-ca-mtv-e/sm-g988b",
}
ROUTES = (
    "2021-08-24-20-32-us-ca-mtv-h/pixel5",
    "2021-03-16-18-59-us-ca-mtv-a/pixel5",
    "2021-07-27-19-49-us-ca-mtv-b/pixel4",
    "2021-07-14-20-50-us-ca-mtv-e/sm-g988b",
    "2022-04-01-18-22-us-ca-lax-t/pixel5",
    "2023-03-08-21-34-us-ca-mtv-u/pixel5",
)
TARGET_ROUTE = ROUTES[0]
SAFE_ROUTES = ROUTES[1:]


class Phase39Error(ValueError):
    """Raised when a truth-free Phase39 gate cannot be proven."""


def _fail(message: str) -> Phase39Error:
    return Phase39Error(message)


def _reject_forbidden(path: Path | str) -> None:
    token = str(path).lower()
    if token.endswith(".mat") or ".mat/" in token or ".mat\\" in token:
        raise _fail(f"MAT path is forbidden: {path}")
    if "ground_truth" in token or "validation" in token or "holdout" in token:
        raise _fail(f"truth/validation/holdout path is forbidden: {path}")


def sha256(path: Path) -> str:
    _reject_forbidden(path)
    if not path.is_file():
        raise _fail(f"missing file: {path}")
    digest = hashlib.sha256()
    try:
        with path.open("rb") as handle:
            for chunk in iter(lambda: handle.read(1024 * 1024), b""):
                digest.update(chunk)
    except OSError as exc:
        raise _fail(f"failed to hash {path}: {exc}") from exc
    return digest.hexdigest()


def load_json(path: Path, label: str) -> dict[str, Any]:
    _reject_forbidden(path)
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise _fail(f"invalid {label}: {path}") from exc
    if not isinstance(payload, dict):
        raise _fail(f"{label} is not an object: {path}")
    return payload


def atomic_write(path: Path, payload: bytes) -> None:
    _reject_forbidden(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".tmp", dir=str(path.parent)
    )
    try:
        with os.fdopen(descriptor, "wb") as handle:
            descriptor = -1
            handle.write(payload)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, path)
        temporary = ""
    finally:
        if descriptor >= 0:
            os.close(descriptor)
        if temporary:
            try:
                os.unlink(temporary)
            except FileNotFoundError:
                pass


def atomic_json(path: Path, payload: dict[str, Any]) -> None:
    atomic_write(path, (json.dumps(payload, indent=2, sort_keys=True) + "\n").encode("utf-8"))


def relative(path: Path) -> str:
    try:
        return str(path.relative_to(ROOT))
    except ValueError:
        return str(path)


def verify_freeze() -> dict[str, Any]:
    if sha256(FREEZE) != FREEZE_SHA256:
        raise _fail("Phase39 freeze hash changed")
    freeze = load_json(FREEZE, "Phase39 freeze")
    if freeze.get("schema_version") != "smartphone-r5-phase39-gnss-first-velocity-only-handoff-freeze.v1":
        raise _fail("Phase39 freeze schema mismatch")
    if freeze.get("status") != "frozen-before-implementation-and-solver":
        raise _fail("Phase39 freeze is not pre-implementation")
    objective = freeze.get("objective", {})
    if objective.get("candidate_flag") != CANDIDATE_FLAG or tuple(objective.get("structural_routes", ())) != ROUTES:
        raise _fail("Phase39 candidate or route order changed")
    if objective.get("truth_free") is not True:
        raise _fail("Phase39 truth-free contract changed")
    base_recipe = freeze.get("base_recipe", {})
    if tuple(base_recipe.get("flags", ())) != BASE_FLAGS or base_recipe.get("candidate_addition") != CANDIDATE_FLAG:
        raise _fail("Phase39 base recipe changed")
    if base_recipe.get("flag_off_default_unchanged") is not True or base_recipe.get("phase31_champion_unchanged") is not True:
        raise _fail("Phase39 default/champion contract changed")
    gates = freeze.get("predeclared_numeric_gates", {})
    if gates.get("velocity_count_equals_epoch_count") is not True or gates.get("velocity_all_finite") is not True:
        raise _fail("Phase39 velocity count/finite gate changed")
    if gates.get("velocity_norm_max_mps") != MAX_SPEED_MPS or gates.get("velocity_norm_comparison") != "<=":
        raise _fail("Phase39 velocity norm gate changed")
    output_gate = gates.get("output", {})
    expected_output_gate = {
        "finite": True,
        "converged": True,
        "raw_key_exact": True,
        "unresolved_epochs": 0,
        "edge_hold_epochs": 0,
        "over_70_mps_count": 0,
    }
    if any(output_gate.get(key) != value for key, value in expected_output_gate.items()):
        raise _fail("Phase39 output gate changed")
    tdcp = gates.get("tdcp", {})
    if tdcp.get("factors_built_positive") is not True or tdcp.get("factors_inserted_equals_built") is not True or tdcp.get("inserted_positive") is not True:
        raise _fail("Phase39 TDCP gate changed")
    if gates.get("repeat_runs") != 2 or gates.get("byte_identical_submission_and_summary") is not True:
        raise _fail("Phase39 repeat gate changed")
    forbidden = freeze.get("forbidden_actions", [])
    forbidden_text = "\n".join(str(item).lower() for item in forbidden)
    required_forbidden_terms = (
        "ground_truth",
        "validation",
        "holdout",
        "mat",
        "precomputed",
        "edge hold",
        "alternate trajectory",
        "truth",
        "phase31",
        "kaggle",
        "execute matrix",
    )
    if any(term not in forbidden_text for term in required_forbidden_terms):
        raise _fail("Phase39 forbidden-input contract is incomplete")
    return freeze


def input_paths(route: str) -> dict[str, Path]:
    route_path, phone = route.split("/", 1)
    if route in PHASE25_ROUTES:
        root = PHASE25_RAW_ROOT / route_path / phone
    else:
        root = PHASE37_RAW_ROOT / route_path / phone / "inputs"
    return {key: root / filename for key, filename in RAW_FILES.items()}


def verify_inputs(freeze: dict[str, Any], route: str) -> dict[str, Any]:
    expected = freeze.get("input_contract", {}).get("sha256", {}).get(route)
    if not isinstance(expected, dict):
        raise _fail(f"Phase39 input hash record missing: {route}")
    paths = input_paths(route)
    reports: dict[str, Any] = {}
    for key, path in paths.items():
        if key == "broadcast_nav":
            field = "brdc.nav"
        else:
            field = RAW_FILES[key]
        _reject_forbidden(path)
        actual = sha256(path)
        if actual != expected.get(field):
            raise _fail(f"Phase39 raw input hash changed: {route}/{field}")
        reports[key] = {"path": relative(path), "sha256": actual, "bytes": path.stat().st_size}
    return reports


def _parse_int(raw: str | None, field: str, row: int) -> int:
    token = "" if raw is None else raw.strip()
    try:
        value = int(token)
    except (TypeError, ValueError) as exc:
        raise _fail(f"row {row}: invalid {field}") from exc
    if value < 0:
        raise _fail(f"row {row}: negative {field}")
    return value


def _parse_float(raw: str | None, field: str, row: int) -> float:
    try:
        value = float("" if raw is None else raw.strip())
    except (TypeError, ValueError) as exc:
        raise _fail(f"row {row}: invalid {field}") from exc
    if not math.isfinite(value):
        raise _fail(f"row {row}: non-finite {field}")
    return value


def read_raw_epoch_keys(path: Path) -> list[int]:
    _reject_forbidden(path)
    keys: list[int] = []
    previous: int | None = None
    try:
        with path.open(encoding="utf-8-sig", newline="") as handle:
            reader = csv.DictReader(handle)
            if "utcTimeMillis" not in set(reader.fieldnames or ()):
                raise _fail(f"raw input lacks utcTimeMillis: {path}")
            for row_number, row in enumerate(reader, start=2):
                value = _parse_int(row.get("utcTimeMillis"), "utcTimeMillis", row_number)
                if previous is None or value != previous:
                    if previous is not None and value <= previous:
                        raise _fail(f"raw UTC keys are not increasing: {path}")
                    keys.append(value)
                    previous = value
    except OSError as exc:
        raise _fail(f"failed to read raw input: {path}") from exc
    if len(keys) < 2:
        raise _fail(f"raw input has fewer than two epochs: {path}")
    return keys


def read_prediction(path: Path, dataset_id: str) -> list[tuple[int, float, float]]:
    _reject_forbidden(path)
    rows: list[tuple[int, float, float]] = []
    previous: int | None = None
    try:
        with path.open(encoding="utf-8-sig", newline="") as handle:
            reader = csv.DictReader(handle)
            expected_header = ("phone", "UnixTimeMillis", "LatitudeDegrees", "LongitudeDegrees")
            if tuple(reader.fieldnames or ()) != expected_header:
                raise _fail(f"prediction header mismatch: {path}")
            for row_number, row in enumerate(reader, start=2):
                if row.get("phone") != dataset_id.split("/", 1)[1]:
                    raise _fail(f"prediction phone mismatch: {path}")
                timestamp = _parse_int(row.get("UnixTimeMillis"), "UnixTimeMillis", row_number)
                if previous is not None and timestamp <= previous:
                    raise _fail(f"prediction timestamps are not increasing: {path}")
                previous = timestamp
                latitude = _parse_float(row.get("LatitudeDegrees"), "LatitudeDegrees", row_number)
                longitude = _parse_float(row.get("LongitudeDegrees"), "LongitudeDegrees", row_number)
                if not -90.0 <= latitude <= 90.0 or not -180.0 <= longitude <= 180.0:
                    raise _fail(f"prediction coordinate outside range: {path}")
                rows.append((timestamp, latitude, longitude))
    except OSError as exc:
        raise _fail(f"failed to read prediction: {path}") from exc
    if not rows or len({row[0] for row in rows}) != len(rows):
        raise _fail(f"prediction is empty or has duplicate timestamps: {path}")
    return rows


def speed_report(rows: list[tuple[int, float, float]]) -> dict[str, Any]:
    if len(rows) < 2:
        return {
            "transition_count": 0,
            "max_speed_mps": 0.0,
            "initial_30_transition_max_speed_mps": 0.0,
            "over_70_mps_count": 0,
            "finite": True,
        }
    speeds: list[float] = []
    for previous, current in zip(rows, rows[1:]):
        dt = (current[0] - previous[0]) / 1000.0
        if dt <= 0.0:
            raise _fail("prediction timestamps do not have positive spacing")
        lat0, lat1 = math.radians(previous[1]), math.radians(current[1])
        dlat = lat1 - lat0
        dlon = math.radians(current[2] - previous[2])
        term = math.sin(dlat / 2.0) ** 2 + math.cos(lat0) * math.cos(lat1) * math.sin(dlon / 2.0) ** 2
        distance = 2.0 * EARTH_RADIUS_M * math.asin(min(1.0, math.sqrt(max(0.0, term))))
        speeds.append(distance / dt)
    initial = speeds[: min(30, len(speeds))]
    return {
        "transition_count": len(speeds),
        "max_speed_mps": max(speeds),
        "initial_30_transition_max_speed_mps": max(initial),
        "over_70_mps_count": sum(speed > MAX_SPEED_MPS for speed in speeds),
        "finite": all(math.isfinite(speed) for speed in speeds),
    }


def _finite_number(value: Any, label: str) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise _fail(f"{label} is not numeric") from exc
    if not math.isfinite(number):
        raise _fail(f"{label} is non-finite")
    return number


def validate_candidate_summary(path: Path, dataset_id: str, raw_epoch_count: int) -> dict[str, Any]:
    summary = load_json(path, "Phase39 candidate summary")
    if summary.get("dataset_id") != dataset_id:
        raise _fail(f"candidate summary dataset mismatch: {dataset_id}")
    if summary.get("truth_used") is not False or summary.get("production_default_changed") is not False:
        raise _fail(f"candidate truth/default contract failed: {dataset_id}")
    if summary.get("native_pdc_imu_tdcp_no_bridge") is not True or summary.get("native_quality_anchor") is not True:
        raise _fail(f"candidate base flags missing: {dataset_id}")
    if summary.get("native_pdc_state_bridge") is not False:
        raise _fail(f"candidate PDC bridge unexpectedly enabled: {dataset_id}")
    gnss = summary.get("gnss_first")
    graph = summary.get("graph")
    epochs = summary.get("epochs")
    raw = summary.get("raw_utc_key_contract")
    tdcp = summary.get("tdcp_contract")
    if not all(isinstance(value, dict) for value in (gnss, graph, epochs, raw, tdcp)):
        raise _fail(f"candidate diagnostics missing: {dataset_id}")
    if gnss.get("handoff_mode") != "velocity-only":
        raise _fail(f"candidate velocity-only handoff marker missing: {dataset_id}")
    if gnss.get("converged") is not True or int(gnss.get("epochs", -1)) != raw_epoch_count:
        raise _fail(f"candidate GNSS-first convergence/epoch gate failed: {dataset_id}")
    if int(gnss.get("velocity_states_exported", -1)) != raw_epoch_count:
        raise _fail(f"candidate GNSS-first velocity count failed: {dataset_id}")
    if int(gnss.get("velocity_valid_count", -1)) != raw_epoch_count:
        raise _fail(f"candidate GNSS-first finite/bound velocity count failed: {dataset_id}")
    if int(gnss.get("velocity_nonfinite_count", -1)) != 0 or int(gnss.get("velocity_over_70_mps_count", -1)) != 0:
        raise _fail(f"candidate GNSS-first velocity validity failed: {dataset_id}")
    if _finite_number(gnss.get("max_velocity_norm_mps"), "candidate max velocity") > MAX_SPEED_MPS:
        raise _fail(f"candidate GNSS-first velocity exceeds 70 m/s: {dataset_id}")
    if int(gnss.get("original_raw_seed_position_count", -1)) != raw_epoch_count or int(gnss.get("original_raw_seed_position_invalid_count", -1)) != 0:
        raise _fail(f"candidate original raw SPP position gate failed: {dataset_id}")
    if int(gnss.get("positions_clocks_copied", -1)) != 0:
        raise _fail(f"candidate GNSS-first position/clock copy gate failed: {dataset_id}")
    if int(gnss.get("position_invalid_count", -1)) < 0 or int(gnss.get("clock_invalid_count", -1)) < 0:
        raise _fail(f"candidate GNSS-first invalid-state diagnostics malformed: {dataset_id}")
    if graph.get("converged") is not True:
        raise _fail(f"candidate IMU graph did not converge: {dataset_id}")
    initial = _finite_number(graph.get("initial_cost"), "candidate initial cost")
    final = _finite_number(graph.get("final_cost"), "candidate final cost")
    if final > initial + 1e-9:
        raise _fail(f"candidate graph cost increased: {dataset_id}")
    if int(epochs.get("problem", -1)) != raw_epoch_count or int(epochs.get("output", -1)) != raw_epoch_count:
        raise _fail(f"candidate epoch count mismatch: {dataset_id}")
    if int(epochs.get("pseudorange_factors", 0)) <= 0 or int(graph.get("imu_intervals", 0)) <= 0:
        raise _fail(f"candidate graph factor gate failed: {dataset_id}")
    if int(tdcp.get("factors_built", 0)) <= 0 or int(tdcp.get("factors_inserted", -1)) != int(tdcp.get("factors_built", 0)) or int(tdcp.get("nonfinite_residuals", -1)) != 0:
        raise _fail(f"candidate TDCP build/insert gate failed: {dataset_id}")
    if int(raw.get("raw_epoch_keys", -1)) != raw_epoch_count or int(raw.get("target_epochs", -1)) != raw_epoch_count - 1:
        raise _fail(f"candidate raw epoch-key count failed: {dataset_id}")
    if int(raw.get("exact_solution_epochs", -1)) != raw_epoch_count - 1 or int(raw.get("interpolated_epochs", -1)) != 0 or int(raw.get("edge_hold_epochs", -1)) != 0 or int(raw.get("unresolved_epochs", -1)) != 0:
        raise _fail(f"candidate exact raw-key/edge-hold gate failed: {dataset_id}")
    return {
        "gnss_first": gnss,
        "graph": graph,
        "epochs": epochs,
        "raw_utc_key_contract": raw,
        "tdcp_contract": tdcp,
    }


def artifact_report(submission: Path, summary: Path, dataset_id: str, target_keys: list[int], raw_epoch_count: int) -> dict[str, Any]:
    rows = read_prediction(submission, dataset_id)
    if [row[0] for row in rows] != target_keys:
        raise _fail(f"candidate raw target key mismatch: {dataset_id}")
    speed = speed_report(rows)
    if speed["finite"] is not True or speed["over_70_mps_count"] != 0:
        raise _fail(f"candidate output continuity gate failed: {dataset_id}")
    projection = validate_candidate_summary(summary, dataset_id, raw_epoch_count)
    return {
        "submission": {
            "path": relative(submission),
            "sha256": sha256(submission),
            "bytes": submission.stat().st_size,
            "rows": len(rows),
        },
        "summary": {
            "path": relative(summary),
            "sha256": sha256(summary),
            "bytes": summary.stat().st_size,
        },
        "projection": projection,
        "speed": speed,
    }


def native_command(paths: dict[str, Path], dataset_id: str, run_dir: Path, candidate: bool) -> list[str]:
    flags = list(BASE_FLAGS)
    if candidate:
        flags.append(CANDIDATE_FLAG)
    command = [
        str(BINARY),
        "--android-gnss", str(paths["device_gnss"]),
        "--android-imu", str(paths["device_imu"]),
        "--nav", str(paths["broadcast_nav"]),
        "--out", str(run_dir / "submission.csv"),
        "--summary-json", str(run_dir / "summary.json"),
        "--dataset-id", dataset_id,
        *flags,
    ]
    for token in command:
        _reject_forbidden(token)
    return command


def run_native(paths: dict[str, Path], dataset_id: str, run_dir: Path, candidate: bool) -> dict[str, Any]:
    if run_dir.exists():
        raise _fail(f"refusing to overwrite native run: {run_dir}")
    for path in paths.values():
        _reject_forbidden(path)
        if not path.is_file():
            raise _fail(f"missing raw input: {path}")
    run_dir.mkdir(parents=True, exist_ok=True)
    command = native_command(paths, dataset_id, run_dir, candidate)
    environment = os.environ.copy()
    local_lib = "/home/sasaki/.local/lib"
    environment["LD_LIBRARY_PATH"] = local_lib + ((":" + environment["LD_LIBRARY_PATH"]) if environment.get("LD_LIBRARY_PATH") else "")
    started = time.perf_counter()
    try:
        process = subprocess.run(
            command,
            cwd=ROOT,
            env=environment,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            check=False,
            timeout=900,
        )
    except subprocess.TimeoutExpired as exc:
        atomic_write(run_dir / "run.log", (str(exc) + "\n").encode("utf-8"))
        raise _fail(f"native solver timeout: {dataset_id}") from exc
    wall = time.perf_counter() - started
    atomic_write(run_dir / "run.log", process.stdout.encode("utf-8"))
    if process.returncode != 0:
        raise _fail(f"native solver failed ({process.returncode}): {dataset_id}")
    submission = run_dir / "submission.csv"
    summary = run_dir / "summary.json"
    raw_epoch_count = len(read_raw_epoch_keys(paths["device_gnss"]))
    target_keys = read_raw_epoch_keys(paths["device_gnss"])[1:]
    if not submission.is_file() or not summary.is_file():
        raise _fail(f"native outputs incomplete: {dataset_id}")
    report = artifact_report(submission, summary, dataset_id, target_keys, raw_epoch_count)
    report.update({
        "candidate": candidate,
        "command": command,
        "return_code": process.returncode,
        "wall_seconds": wall,
        "log": {"path": relative(run_dir / "run.log"), "sha256": sha256(run_dir / "run.log")},
    })
    return report


def run_expected_flag_off_failure(paths: dict[str, Path], dataset_id: str, run_dir: Path) -> dict[str, Any]:
    if run_dir.exists():
        raise _fail(f"refusing to overwrite native run: {run_dir}")
    run_dir.mkdir(parents=True, exist_ok=True)
    command = native_command(paths, dataset_id, run_dir, candidate=False)
    environment = os.environ.copy()
    local_lib = "/home/sasaki/.local/lib"
    environment["LD_LIBRARY_PATH"] = local_lib + ((":" + environment["LD_LIBRARY_PATH"]) if environment.get("LD_LIBRARY_PATH") else "")
    started = time.perf_counter()
    process = subprocess.run(
        command,
        cwd=ROOT,
        env=environment,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        check=False,
        timeout=900,
    )
    wall = time.perf_counter() - started
    log = run_dir / "run.log"
    atomic_write(log, process.stdout.encode("utf-8"))
    if process.returncode == 0:
        raise _fail(f"flag-off target unexpectedly succeeded: {dataset_id}")
    if "failed to align output to raw UTC keys" not in process.stdout:
        raise _fail(f"flag-off target failure classification changed: {dataset_id}")
    if (run_dir / "submission.csv").exists() or (run_dir / "summary.json").exists():
        raise _fail(f"flag-off failed run published output: {dataset_id}")
    return {
        "candidate": False,
        "expected_failure": "native_solver_state_failure_before_utc_projection",
        "command": command,
        "return_code": process.returncode,
        "wall_seconds": wall,
        "log": {"path": relative(log), "sha256": sha256(log)},
    }


def _candidate_route_report(freeze: dict[str, Any], output_root: Path, route: str) -> dict[str, Any]:
    paths = input_paths(route)
    raw_inputs = verify_inputs(freeze, route)
    raw_keys = read_raw_epoch_keys(paths["device_gnss"])
    route_path, phone = route.split("/", 1)
    native_root = output_root / "candidate" / route_path / phone
    first = run_native(paths, route, native_root / "run1", candidate=True)
    repeat = run_native(paths, route, native_root / "run2", candidate=True)
    if first["submission"]["sha256"] != repeat["submission"]["sha256"] or first["summary"]["sha256"] != repeat["summary"]["sha256"]:
        raise _fail(f"candidate repeat is not byte-identical: {route}")
    return {
        "dataset_id": route,
        "source": "phase39-native-velocity-only-repeat",
        "raw_inputs": raw_inputs,
        "raw_epoch_count": len(raw_keys),
        "target_epoch_count": len(raw_keys) - 1,
        "run1": first,
        "run2": repeat,
        "repeat_byte_identical": True,
        "truth_open_count": 0,
        "mat_read_or_generated": False,
    }


def _flag_off_report(freeze: dict[str, Any], output_root: Path, route: str) -> dict[str, Any]:
    paths = input_paths(route)
    raw_inputs = verify_inputs(freeze, route)
    route_path, phone = route.split("/", 1)
    control_root = output_root / "flag-off" / route_path / phone
    if route == TARGET_ROUTE:
        run = run_expected_flag_off_failure(paths, route, control_root / "run1")
        return {
            "dataset_id": route,
            "source": "phase38-frozen-flag-off-control",
            "raw_inputs": raw_inputs,
            "run": run,
            "unchanged_failure": True,
            "truth_open_count": 0,
            "mat_read_or_generated": False,
        }
    baseline = freeze["flag_off_identity_contract"]["expected_baselines"][route]
    first = run_native(paths, route, control_root / "run1", candidate=False)
    if first["submission"]["sha256"] != baseline["submission_sha256"] or first["summary"]["sha256"] != baseline["summary_sha256"]:
        raise _fail(f"flag-off frozen baseline changed: {route}")
    # A second control run proves that the candidate build did not make the
    # historical flag-off path nondeterministic; its bytes must match run1.
    repeat = run_native(paths, route, control_root / "run2", candidate=False)
    if first["submission"]["sha256"] != repeat["submission"]["sha256"] or first["summary"]["sha256"] != repeat["summary"]["sha256"]:
        raise _fail(f"flag-off control repeat differs: {route}")
    return {
        "dataset_id": route,
        "source": "phase39-flag-off-frozen-baseline-control",
        "raw_inputs": raw_inputs,
        "expected_baseline": baseline,
        "run1": first,
        "run2": repeat,
        "repeat_byte_identical": True,
        "frozen_baseline_byte_identical": True,
        "truth_open_count": 0,
        "mat_read_or_generated": False,
    }


def seal(output_root: Path = DEFAULT_OUTPUT, run_matrix: bool = False) -> dict[str, Any]:
    freeze = verify_freeze()
    output_root = output_root.resolve()
    _reject_forbidden(output_root)
    if run_matrix:
        if output_root.exists() and any(output_root.iterdir()):
            raise _fail(f"refusing to overwrite nonempty Phase39 output: {output_root}")
        output_root.mkdir(parents=True, exist_ok=True)
        candidate_routes: dict[str, Any] = {}
        flag_off_routes: dict[str, Any] = {}
        failures: list[str] = []
        for route in ROUTES:
            try:
                candidate_routes[route] = _candidate_route_report(freeze, output_root, route)
            except (OSError, Phase39Error) as exc:
                failures.append(f"candidate {route}: {exc}")
                break
        if not failures:
            for route in ROUTES:
                try:
                    flag_off_routes[route] = _flag_off_report(freeze, output_root, route)
                except (OSError, Phase39Error) as exc:
                    failures.append(f"flag-off {route}: {exc}")
                    break
        if failures:
            failure = {
                "schema_version": "smartphone-r5-phase39-structural-failure.v1",
                "phase": 39,
                "status": "fail-closed-no-go",
                "freeze": {"path": relative(FREEZE), "sha256": FREEZE_SHA256},
                "candidate_routes": candidate_routes,
                "flag_off_routes": flag_off_routes,
                "failures": failures,
                "truth_open_count": 0,
                "ground_truth_materialized": False,
                "validation_truth_open_count": 0,
                "holdout_truth_open_count": 0,
                "mat_read_or_generated": False,
                "token_or_kaggle_access": False,
            }
            atomic_json(output_root / FAILURE_NAME, failure)
            raise _fail("Phase39 structural matrix failed; truth remains sealed")
    else:
        candidate_routes = {}
        flag_off_routes = {}
        for route in ROUTES:
            route_path, phone = route.split("/", 1)
            candidate_root = output_root / "candidate" / route_path / phone
            run1 = candidate_root / "run1"
            run2 = candidate_root / "run2"
            paths = input_paths(route)
            raw_inputs = verify_inputs(freeze, route)
            raw_keys = read_raw_epoch_keys(paths["device_gnss"])
            first = artifact_report(run1 / "submission.csv", run1 / "summary.json", route, raw_keys[1:], len(raw_keys))
            repeat = artifact_report(run2 / "submission.csv", run2 / "summary.json", route, raw_keys[1:], len(raw_keys))
            if first["submission"]["sha256"] != repeat["submission"]["sha256"] or first["summary"]["sha256"] != repeat["summary"]["sha256"]:
                raise _fail(f"candidate repeat differs: {route}")
            candidate_routes[route] = {
                "dataset_id": route,
                "source": "phase39-native-velocity-only-repeat",
                "raw_inputs": raw_inputs,
                "raw_epoch_count": len(raw_keys),
                "target_epoch_count": len(raw_keys) - 1,
                "run1": first,
                "run2": repeat,
                "repeat_byte_identical": True,
                "truth_open_count": 0,
                "mat_read_or_generated": False,
            }
        for route in ROUTES:
            route_path, phone = route.split("/", 1)
            control_root = output_root / "flag-off" / route_path / phone
            paths = input_paths(route)
            raw_inputs = verify_inputs(freeze, route)
            if route == TARGET_ROUTE:
                log = control_root / "run1" / "run.log"
                control = load_json(output_root / "flag-off" / "target_failure.json", "Phase39 target flag-off record")
                flag_off_routes[route] = control
                continue
            baseline = freeze["flag_off_identity_contract"]["expected_baselines"][route]
            raw_keys = read_raw_epoch_keys(paths["device_gnss"])
            first = artifact_report(control_root / "run1" / "submission.csv", control_root / "run1" / "summary.json", route, raw_keys[1:], len(raw_keys))
            repeat = artifact_report(control_root / "run2" / "submission.csv", control_root / "run2" / "summary.json", route, raw_keys[1:], len(raw_keys))
            if first["submission"]["sha256"] != baseline["submission_sha256"] or first["summary"]["sha256"] != baseline["summary_sha256"]:
                raise _fail(f"flag-off frozen baseline changed: {route}")
            if first["submission"]["sha256"] != repeat["submission"]["sha256"] or first["summary"]["sha256"] != repeat["summary"]["sha256"]:
                raise _fail(f"flag-off repeat differs: {route}")
            flag_off_routes[route] = {
                "dataset_id": route,
                "source": "phase39-flag-off-frozen-baseline-control",
                "raw_inputs": raw_inputs,
                "expected_baseline": baseline,
                "run1": first,
                "run2": repeat,
                "repeat_byte_identical": True,
                "frozen_baseline_byte_identical": True,
                "truth_open_count": 0,
                "mat_read_or_generated": False,
            }

    result = {
        "schema_version": "smartphone-r5-phase39-structural-seal.v1",
        "phase": 39,
        "status": "sealed-truth-free-structural-pass",
        "decision": "structural-go-development-only; truth remains sealed",
        "freeze": {"path": relative(FREEZE), "sha256": FREEZE_SHA256},
        "native": {
            "binary": {"path": relative(BINARY), "sha256": sha256(BINARY)},
            "base_flags": list(BASE_FLAGS),
            "candidate_flag": CANDIDATE_FLAG,
            "inputs_only": ["raw device_gnss.csv", "raw device_imu.csv", "broadcast brdc.nav"],
            "ground_truth_used": False,
            "precomputed_result_coordinates_used_for_inference": False,
            "edge_hold_or_alternate_trajectory_used": False,
            "production_default_changed": False,
            "phase31_champion_mutated": False,
        },
        "candidate_routes": candidate_routes,
        "flag_off_routes": flag_off_routes,
        "structural_gate": {
            "routes_passed": len(candidate_routes),
            "routes_total": len(ROUTES),
            "velocity_count_equals_epoch_count": all(
                route["run1"]["projection"]["gnss_first"]["velocity_states_exported"] == route["raw_epoch_count"]
                for route in candidate_routes.values()
            ),
            "velocity_all_finite_and_le_70_mps": all(
                route["run1"]["projection"]["gnss_first"]["velocity_valid_count"] == route["raw_epoch_count"] and
                route["run1"]["projection"]["gnss_first"]["velocity_over_70_mps_count"] == 0 and
                route["run1"]["projection"]["gnss_first"]["max_velocity_norm_mps"] <= MAX_SPEED_MPS
                for route in candidate_routes.values()
            ),
            "original_raw_seed_positions_earth_valid": all(
                route["run1"]["projection"]["gnss_first"]["original_raw_seed_position_invalid_count"] == 0
                for route in candidate_routes.values()
            ),
            "gnss_first_positions_clocks_copied_zero": all(
                route["run1"]["projection"]["gnss_first"]["positions_clocks_copied"] == 0
                for route in candidate_routes.values()
            ),
            "tdcp_built_inserted_positive_equal": all(
                route["run1"]["projection"]["tdcp_contract"]["factors_built"] > 0 and
                route["run1"]["projection"]["tdcp_contract"]["factors_inserted"] == route["run1"]["projection"]["tdcp_contract"]["factors_built"]
                for route in candidate_routes.values()
            ),
            "finite_converged_output": all(route["run1"]["speed"]["finite"] and route["run1"]["projection"]["graph"]["converged"] for route in candidate_routes.values()),
            "raw_key_exact_no_edge_hold": all(
                route["run1"]["projection"]["raw_utc_key_contract"]["unresolved_epochs"] == 0 and
                route["run1"]["projection"]["raw_utc_key_contract"]["interpolated_epochs"] == 0 and
                route["run1"]["projection"]["raw_utc_key_contract"]["edge_hold_epochs"] == 0
                for route in candidate_routes.values()
            ),
            "no_over_70_mps_transition": all(route["run1"]["speed"]["over_70_mps_count"] == 0 for route in candidate_routes.values()),
            "repeat_byte_identical": all(route["repeat_byte_identical"] for route in candidate_routes.values()),
            "flag_off_frozen_baseline_byte_identical": all(
                route.get("frozen_baseline_byte_identical", False) for key, route in flag_off_routes.items() if key != TARGET_ROUTE
            ),
            "flag_off_target_failure_unchanged": flag_off_routes[TARGET_ROUTE].get("unchanged_failure") is True,
            "truth_used_false": True,
        },
        "truth_policy": {
            "truth_open_count": 0,
            "ground_truth_materialized": False,
            "validation_truth_open_count": 0,
            "holdout_truth_open_count": 0,
            "mat_read_or_generated": False,
            "token_or_kaggle_access": False,
            "post_truth_tuning": False,
        },
    }
    atomic_json(output_root / RESULT_NAME, result)
    manifest = {
        "schema_version": "smartphone-r5-phase39-structural-seal-manifest.v1",
        "status": result["status"],
        "result": {"path": relative(output_root / RESULT_NAME), "sha256": sha256(output_root / RESULT_NAME)},
        "freeze": result["freeze"],
        "candidate_routes": list(candidate_routes),
        "flag_off_routes": list(flag_off_routes),
        "route_count": len(ROUTES),
        "repeat_runs": 2,
        "truth_open_count": 0,
        "ground_truth_materialized": False,
        "validation_truth_open_count": 0,
        "holdout_truth_open_count": 0,
        "mat_read_or_generated": False,
        "atomic_publish": True,
    }
    atomic_json(output_root / MANIFEST_NAME, manifest)
    return result


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("operation", choices=("verify-freeze", "run-matrix", "seal"))
    parser.add_argument("--output-root", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args(argv)
    try:
        if args.operation == "verify-freeze":
            verify_freeze()
            print(json.dumps({"status": "freeze-verified", "truth_open_count": 0}, sort_keys=True))
        elif args.operation == "run-matrix":
            result = seal(args.output_root, run_matrix=True)
            print(json.dumps({"status": result["status"], "routes": len(result["candidate_routes"]), "truth_open_count": 0}, sort_keys=True))
        else:
            result = seal(args.output_root, run_matrix=False)
            print(json.dumps({"status": result["status"], "routes": len(result["candidate_routes"]), "truth_open_count": 0}, sort_keys=True))
    except (OSError, Phase39Error, ValueError, KeyError, TypeError, json.JSONDecodeError) as exc:
        print(f"phase39: {exc}", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
