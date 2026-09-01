#!/usr/bin/env python3
"""Truth-free Phase43 fallback-seed quality-anchor recovery matrix.

The runner uses only the frozen raw Android GNSS/IMU streams and broadcast
navigation.  It runs the candidate twice per route, checks the recovery
summary and unchanged graph contracts, and compares the five non-target
routes with a flag-off control after removing candidate-only diagnostics.
No truth, validation, holdout, MAT, precomputed coordinate, Kaggle, or token
input is opened.
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
import tempfile
import time
from typing import Any


ROOT = Path(__file__).resolve().parents[3]
FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_phase43_native_fallback_seed_quality_anchor_recovery_freeze_v1.json"
FREEZE_SHA256 = "9278a835a21e3a19027aefd3675aba45d8f1f9e19183cbc5590850b776722400"
BINARY = ROOT / "build/apps/gnss_fgo_imu_no_base"
DEFAULT_OUTPUT = ROOT / "output/smartphone-r5/phase43-native-fallback-seed-quality-anchor-recovery-v1"
RESULT_NAME = "phase43_structural_seal.json"
MANIFEST_NAME = "phase43_structural_seal.manifest.json"
FAILURE_NAME = "phase43_structural_failure.json"

PHASE25_RAW_ROOT = ROOT / "output/smartphone-r5/phase25-raw-clock-eval-v1/raw"
PHASE37_RAW_ROOT = ROOT / "output/smartphone-r5/phase37-pixel5-repeatability-v1/routes"
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
BASE_FLAGS = (
    "--all-epochs",
    "--android-raw-utc-keys",
    "--android-raw-clock-only",
    "--android-utc-wall-clock-fallback",
    "--native-pdc-imu-tdcp-no-bridge",
    "--native-quality-anchor",
)
CANDIDATE_FLAG = "--native-fallback-seed-quality-anchor-recovery"
RAW_FILES = {
    "device_gnss": "device_gnss.csv",
    "device_imu": "device_imu.csv",
    "broadcast_nav": "brdc.nav",
}
MAX_SPEED_MPS = 70.0
EARTH_RADIUS_M = 6_371_000.0

# Historical flag-off controls from the frozen Phase31/39 raw-only lanes.
# The MTV-h lane had no successful flag-off artifact in Phase37, so it is
# recorded as a fail-closed diagnostic and is not used for the safe-route
# projected-identity gate.
FLAG_OFF_BASELINES = {
    "2021-03-16-18-59-us-ca-mtv-a/pixel5": {
        "submission_sha256": "d4d7652e5d12389466e586fe2d8e85d34977a7e11036cee2f591a89293c5426c",
        "summary_sha256": "b0ecd9c0eb931b11908a4e19ea54cb21a7a530492ac6fee245086dff93722460",
    },
    "2021-07-27-19-49-us-ca-mtv-b/pixel4": {
        "submission_sha256": "18d90461883f9b51dc5e235ff2fda2e535d31678bb47b232a6aec12662faacdc",
        "summary_sha256": "915f002eeb1c2f4728741d72e8225f6a36959462d4569f8a04f0ce3eb17da988",
    },
    "2021-07-14-20-50-us-ca-mtv-e/sm-g988b": {
        "submission_sha256": "65b44029a90e75a6b11e96b6ae9344020dee55bb8b5ea9b8bee79f2c765bca47",
        "summary_sha256": "77beb1b59aadefcdac0394c775f5ab45eb58c10ec7af92f21990dbbc694438b8",
    },
    "2022-04-01-18-22-us-ca-lax-t/pixel5": {
        "submission_sha256": "4eb2b566708a87db4903610a41cd648c7ff065d35710d358894a78eaa8e116e3",
        "summary_sha256": "2c85fd545ecf6b1bc815c7c4c42a139424a722787b2b58da6ae48b412188a542",
    },
    "2023-03-08-21-34-us-ca-mtv-u/pixel5": {
        "submission_sha256": "524769cdf67aa857eefaafdadf943dd76586ec3ae73ec8b6d1df4a707d7699f71",
        "summary_sha256": "84b354751b617258db6499cf5ab477f520e3463ae73ec8b73dd3f3b438d45f31",
    },
}


class Phase43Error(ValueError):
    """Raised when a frozen Phase43 structural gate fails."""


def _fail(message: str) -> Phase43Error:
    return Phase43Error(message)


def _reject_forbidden(path: Path | str) -> None:
    token = str(path).lower()
    if token.endswith(".mat") or ".mat/" in token or ".mat\\" in token:
        raise _fail(f"MAT path is forbidden: {path}")
    if any(term in token for term in ("ground_truth", "validation", "holdout")):
        raise _fail(f"truth/validation/holdout path is forbidden: {path}")


def sha256(path: Path) -> str:
    _reject_forbidden(path)
    if not path.is_file():
        raise _fail(f"missing file: {path}")
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def load_json(path: Path, label: str) -> dict[str, Any]:
    _reject_forbidden(path)
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise _fail(f"invalid {label}: {path}") from exc
    if not isinstance(value, dict):
        raise _fail(f"{label} is not an object: {path}")
    return value


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


def atomic_json(path: Path, value: dict[str, Any]) -> None:
    atomic_write(path, (json.dumps(value, indent=2, sort_keys=True) + "\n").encode())


def relative(path: Path) -> str:
    try:
        return str(path.relative_to(ROOT))
    except ValueError:
        return str(path)


def invocation_path(path: Path) -> str:
    """Match historical summary path spelling for each frozen input cohort."""
    if PHASE37_RAW_ROOT in path.parents:
        return str(path)
    return relative(path)


def verify_freeze() -> dict[str, Any]:
    if sha256(FREEZE) != FREEZE_SHA256:
        raise _fail("Phase43 freeze hash changed")
    freeze = load_json(FREEZE, "Phase43 freeze")
    if freeze.get("schema_version") != "smartphone-r5-phase43-native-fallback-seed-quality-anchor-recovery-freeze.v1":
        raise _fail("Phase43 freeze schema mismatch")
    if freeze.get("status") != "frozen-before-implementation-and-matrix":
        raise _fail("Phase43 freeze status changed")
    objective = freeze.get("objective", {})
    if objective.get("candidate_flag") != CANDIDATE_FLAG:
        raise _fail("Phase43 candidate flag changed")
    if tuple(objective.get("structural_routes", ())) != ROUTES:
        raise _fail("Phase43 route order changed")
    if objective.get("route_order_is_fixed") is not True or objective.get("truth_free") is not True:
        raise _fail("Phase43 route/truth contract changed")
    recipe = freeze.get("base_recipe", {})
    if tuple(recipe.get("flags", ())) != BASE_FLAGS or recipe.get("candidate_addition") != CANDIDATE_FLAG:
        raise _fail("Phase43 base recipe changed")
    if recipe.get("flag_off_default_unchanged") is not True or recipe.get("phase31_champion_unchanged") is not True:
        raise _fail("Phase43 default/champion contract changed")
    gates = freeze.get("predeclared_numeric_gates", {})
    target = gates.get("target", {})
    for key in ("recovery_trigger", "anchor_selected", "anchor_position_finite", "anchor_position_earth_valid",
                "replay_valid_epochs_positive", "output_finite", "output_converged", "raw_key_exact",
                "tdcp_factors_built_positive", "tdcp_factors_inserted_equals_built", "tdcp_inserted_positive"):
        if target.get(key) is not True:
            raise _fail(f"Phase43 target gate changed: {key}")
    if target.get("over_70_mps_count") != 0:
        raise _fail("Phase43 speed gate changed")
    other = gates.get("other_routes", {})
    if other.get("recovery_trigger") is not False or other.get("flag_on_submission_and_summary_projected_identity") is not True:
        raise _fail("Phase43 other-route gate changed")
    all_routes = gates.get("all_routes", {})
    if all_routes.get("repeat_runs") != 2 or all_routes.get("byte_identical_submission_and_summary") is not True or all_routes.get("sentinel_factor_bypass") is not False:
        raise _fail("Phase43 repeat/sentinel gate changed")
    forbidden = "\n".join(str(item).lower() for item in freeze.get("forbidden_actions", []))
    for term in ("ground_truth", "validation", "holdout", "mat", "precomputed", "sentinel", "factor-level", "truth", "kaggle"):
        if term not in forbidden:
            raise _fail(f"Phase43 forbidden-input contract is incomplete: {term}")
    expected_hashes = freeze.get("input_contract", {}).get("sha256", {})
    if set(expected_hashes) != set(ROUTES):
        raise _fail("Phase43 raw input hash route set changed")
    return freeze


def input_paths(route: str) -> dict[str, Path]:
    route_path, phone = route.split("/", 1)
    root = PHASE25_RAW_ROOT / route_path / phone if route in PHASE25_ROUTES else PHASE37_RAW_ROOT / route_path / phone / "inputs"
    return {key: root / filename for key, filename in RAW_FILES.items()}


def verify_inputs(freeze: dict[str, Any], route: str) -> dict[str, Any]:
    expected = freeze.get("input_contract", {}).get("sha256", {}).get(route)
    if not isinstance(expected, dict):
        raise _fail(f"Phase43 input hashes missing: {route}")
    report: dict[str, Any] = {}
    for key, path in input_paths(route).items():
        actual = sha256(path)
        filename = RAW_FILES[key]
        if actual != expected.get(filename):
            raise _fail(f"Phase43 raw input hash changed: {route}/{filename}")
        report[key] = {"path": relative(path), "sha256": actual, "bytes": path.stat().st_size}
    return report


def read_raw_epoch_keys(path: Path) -> list[int]:
    _reject_forbidden(path)
    keys: list[int] = []
    previous: int | None = None
    with path.open(encoding="utf-8-sig", newline="") as handle:
        reader = csv.DictReader(handle)
        if "utcTimeMillis" not in set(reader.fieldnames or ()):
            raise _fail(f"raw input lacks utcTimeMillis: {path}")
        for row_number, row in enumerate(reader, start=2):
            try:
                value = int((row.get("utcTimeMillis") or "").strip())
            except ValueError as exc:
                raise _fail(f"row {row_number}: invalid utcTimeMillis") from exc
            if previous is None or value != previous:
                if previous is not None and value <= previous:
                    raise _fail(f"raw UTC keys are not increasing: {path}")
                keys.append(value)
                previous = value
    if len(keys) < 2:
        raise _fail(f"raw input has fewer than two epochs: {path}")
    return keys


def read_prediction(path: Path, dataset_id: str) -> list[tuple[int, float, float]]:
    _reject_forbidden(path)
    rows: list[tuple[int, float, float]] = []
    with path.open(encoding="utf-8-sig", newline="") as handle:
        reader = csv.DictReader(handle)
        expected_header = ("phone", "UnixTimeMillis", "LatitudeDegrees", "LongitudeDegrees")
        if tuple(reader.fieldnames or ()) != expected_header:
            raise _fail(f"submission header mismatch: {path}")
        for row_number, row in enumerate(reader, start=2):
            if row.get("phone") != dataset_id:
                raise _fail(f"submission dataset mismatch: {path}")
            try:
                timestamp = int(row.get("UnixTimeMillis", ""))
                latitude = float(row.get("LatitudeDegrees", ""))
                longitude = float(row.get("LongitudeDegrees", ""))
            except ValueError as exc:
                raise _fail(f"submission row {row_number} is non-numeric") from exc
            if not all(math.isfinite(value) for value in (latitude, longitude)):
                raise _fail(f"submission row {row_number} is non-finite")
            if not -90.0 <= latitude <= 90.0 or not -180.0 <= longitude <= 180.0:
                raise _fail(f"submission row {row_number} is out of range")
            if rows and timestamp <= rows[-1][0]:
                raise _fail(f"submission timestamps are not increasing: {path}")
            rows.append((timestamp, latitude, longitude))
    if not rows:
        raise _fail(f"submission is empty: {path}")
    return rows


def speed_report(rows: list[tuple[int, float, float]]) -> dict[str, Any]:
    speeds: list[float] = []
    for previous, current in zip(rows, rows[1:]):
        dt = (current[0] - previous[0]) / 1000.0
        if dt <= 0.0:
            raise _fail("submission timestamps do not have positive spacing")
        lat0 = math.radians(previous[1])
        lat1 = math.radians(current[1])
        dlat = lat1 - lat0
        dlon = math.radians(current[2] - previous[2])
        haversine = math.sin(dlat / 2.0) ** 2 + math.cos(lat0) * math.cos(lat1) * math.sin(dlon / 2.0) ** 2
        distance = 2.0 * EARTH_RADIUS_M * math.asin(min(1.0, math.sqrt(max(0.0, haversine))))
        speeds.append(distance / dt)
    return {
        "transition_count": len(speeds),
        "max_speed_mps": max(speeds, default=0.0),
        "over_70_mps_count": sum(speed > MAX_SPEED_MPS for speed in speeds),
        "finite": all(math.isfinite(speed) for speed in speeds),
    }


def validate_summary(path: Path, route: str, raw_epoch_count: int, candidate: bool) -> dict[str, Any]:
    summary = load_json(path, "Phase43 summary")
    if summary.get("dataset_id") != route or summary.get("truth_used") is not False or summary.get("production_default_changed") is not False:
        raise _fail(f"summary identity/truth/default contract failed: {route}")
    if summary.get("native_quality_anchor") is not True:
        raise _fail(f"quality-anchor base marker missing: {route}")
    if summary.get("native_pdc_imu_tdcp_no_bridge") is not True or summary.get("native_pdc_state_bridge") is not False:
        raise _fail(f"Phase43 base flags changed: {route}")
    if candidate and summary.get("native_fallback_seed_quality_anchor_recovery") is not True:
        raise _fail(f"candidate marker missing: {route}")
    if not candidate and "native_fallback_seed_quality_anchor_recovery" in summary:
        raise _fail(f"flag-off candidate marker leaked: {route}")
    anchor = summary.get("quality_anchor_initialization")
    epochs = summary.get("epochs")
    graph = summary.get("graph")
    tdcp = summary.get("tdcp_contract")
    raw = summary.get("raw_utc_key_contract")
    if not all(isinstance(value, dict) for value in (anchor, epochs, graph, tdcp, raw)):
        raise _fail(f"summary diagnostics missing: {route}")
    if anchor.get("enabled") is not True or anchor.get("truth_free") is not True or anchor.get("graph_model_changed") is not False:
        raise _fail(f"quality-anchor contract failed: {route}")
    if int(epochs.get("problem", -1)) != raw_epoch_count or int(epochs.get("output", -1)) != raw_epoch_count:
        raise _fail(f"problem/output epoch count mismatch: {route}")
    if int(epochs.get("pseudorange_factors", 0)) <= 0 or graph.get("converged") is not True:
        raise _fail(f"finite/converged graph gate failed: {route}")
    if int(tdcp.get("factors_built", 0)) <= 0 or int(tdcp.get("factors_inserted", -1)) != int(tdcp.get("factors_built", 0)) or int(tdcp.get("nonfinite_residuals", -1)) != 0:
        raise _fail(f"TDCP build/insert gate failed: {route}")
    if int(raw.get("raw_epoch_keys", -1)) != raw_epoch_count or int(raw.get("target_epochs", -1)) != raw_epoch_count - 1 or int(raw.get("exact_solution_epochs", -1)) != raw_epoch_count - 1 or int(raw.get("interpolated_epochs", -1)) != 0 or int(raw.get("edge_hold_epochs", -1)) != 0 or int(raw.get("unresolved_epochs", -1)) != 0:
        raise _fail(f"raw-key contract failed: {route}")
    if not candidate:
        return {"summary": summary, "anchor": anchor, "graph": graph, "epochs": epochs, "tdcp": tdcp, "raw": raw}
    required = ("normal_quality_anchor_candidates", "recovery_quality_anchor_candidates", "recovery_trigger", "recovery_anchor_selected", "recovery_anchor_index", "recovery_anchor_satellites", "recovery_replay_valid_epochs", "recovery_replay_invalid_epochs", "sentinel_factor_bypass")
    if any(key not in anchor for key in required):
        raise _fail(f"Phase43 recovery diagnostics missing: {route}")
    if route == TARGET_ROUTE:
        if anchor.get("recovery_trigger") is not True or anchor.get("normal_quality_anchor_candidates") != 0 or anchor.get("recovery_anchor_selected") is not True or int(anchor.get("recovery_quality_anchor_candidates", 0)) <= 0:
            raise _fail("MTV-h recovery anchor gate failed")
        if not isinstance(anchor.get("recovery_anchor_index"), int) or int(anchor.get("recovery_anchor_satellites", 0)) < 4:
            raise _fail("MTV-h recovery anchor identity gate failed")
        for key in ("recovery_anchor_gdop", "recovery_anchor_normalized_residual_rms"):
            if not isinstance(anchor.get(key), (int, float)) or not math.isfinite(float(anchor[key])):
                raise _fail(f"MTV-h recovery anchor {key} is non-finite")
        if int(anchor.get("recovery_replay_valid_epochs", 0)) <= 0 or int(anchor.get("recovery_replay_invalid_epochs", -1)) != 0:
            raise _fail("MTV-h recovery replay gate failed")
    else:
        if anchor.get("recovery_trigger") is not False or anchor.get("recovery_anchor_selected") is not False:
            raise _fail(f"unexpected recovery trigger on safe route: {route}")
    if anchor.get("sentinel_factor_bypass") is not False:
        raise _fail(f"sentinel factor bypass gate failed: {route}")
    return {"summary": summary, "anchor": anchor, "graph": graph, "epochs": epochs, "tdcp": tdcp, "raw": raw}


def native_command(paths: dict[str, Path], dataset_id: str, run_dir: Path, candidate: bool) -> list[str]:
    flags = list(BASE_FLAGS)
    if candidate:
        flags.append(CANDIDATE_FLAG)
    # Match the path spelling used by each frozen baseline: Phase25 controls
    # were run from repository-relative inputs, while Phase37 additions used
    # absolute inputs in their historical native command.
    command = [str(BINARY), "--android-gnss", invocation_path(paths["device_gnss"]), "--android-imu", invocation_path(paths["device_imu"]), "--nav", invocation_path(paths["broadcast_nav"]), "--out", str(run_dir / "submission.csv"), "--summary-json", str(run_dir / "summary.json"), "--dataset-id", dataset_id, *flags]
    for token in command:
        _reject_forbidden(token)
    return command


def run_native(paths: dict[str, Path], dataset_id: str, run_dir: Path, candidate: bool, allow_failure: bool = False) -> dict[str, Any]:
    if run_dir.exists():
        raise _fail(f"refusing to overwrite native run: {run_dir}")
    for path in paths.values():
        _reject_forbidden(path)
        if not path.is_file():
            raise _fail(f"missing raw input: {path}")
    run_dir.mkdir(parents=True, exist_ok=True)
    environment = os.environ.copy()
    local_lib = "/home/sasaki/.local/lib"
    environment["LD_LIBRARY_PATH"] = local_lib + ((":" + environment["LD_LIBRARY_PATH"]) if environment.get("LD_LIBRARY_PATH") else "")
    started = time.perf_counter()
    try:
        process = subprocess.run(native_command(paths, dataset_id, run_dir, candidate), cwd=ROOT, env=environment, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, check=False, timeout=900)
    except subprocess.TimeoutExpired as exc:
        atomic_write(run_dir / "run.log", (str(exc) + "\n").encode())
        raise _fail(f"native solver timeout: {dataset_id}") from exc
    wall = time.perf_counter() - started
    atomic_write(run_dir / "run.log", process.stdout.encode())
    report: dict[str, Any] = {"candidate": candidate, "return_code": process.returncode, "wall_seconds": wall, "command": native_command(paths, dataset_id, run_dir, candidate), "log": {"path": relative(run_dir / "run.log"), "sha256": sha256(run_dir / "run.log")}}
    if process.returncode != 0:
        if not allow_failure:
            raise _fail(f"native solver failed ({process.returncode}): {dataset_id}")
        report["failure_classification"] = "fail-closed-no-raw-nav-anchor-or-historical-target-control"
        report["log_tail"] = process.stdout[-4000:]
        return report
    raw_keys = read_raw_epoch_keys(paths["device_gnss"])
    submission = run_dir / "submission.csv"
    summary = run_dir / "summary.json"
    rows = read_prediction(submission, dataset_id)
    if [row[0] for row in rows] != raw_keys[1:]:
        raise _fail(f"raw UTC output keys mismatch: {dataset_id}")
    speed = speed_report(rows)
    if not speed["finite"] or speed["over_70_mps_count"] != 0:
        raise _fail(f"output continuity gate failed: {dataset_id}")
    projection = validate_summary(summary, dataset_id, len(raw_keys), candidate)
    report.update({"submission": {"path": relative(submission), "sha256": sha256(submission), "bytes": submission.stat().st_size, "rows": len(rows)}, "summary": {"path": relative(summary), "sha256": sha256(summary), "bytes": summary.stat().st_size}, "speed": speed, "projection": projection})
    return report


def _project(summary: dict[str, Any]) -> dict[str, Any]:
    """Remove only the declared Phase43 candidate-specific summary fields."""
    value = json.loads(json.dumps(summary))
    value.pop("native_fallback_seed_quality_anchor_recovery", None)
    anchor = value.get("quality_anchor_initialization")
    if isinstance(anchor, dict):
        for key in ("recovery_enabled", "normal_quality_anchor_candidates", "recovery_quality_anchor_candidates", "recovery_trigger", "recovery_triggered", "recovery_anchor_selected", "recovery_anchor_index", "recovery_anchor_satellites", "recovery_anchor_gdop", "recovery_anchor_normalized_residual_rms", "recovery_replay_valid_epochs", "recovery_replay_invalid_epochs", "sentinel_factor_bypass"):
            anchor.pop(key, None)
    return value


def route_report(freeze: dict[str, Any], output_root: Path, route: str) -> dict[str, Any]:
    paths = input_paths(route)
    raw_inputs = verify_inputs(freeze, route)
    raw_keys = read_raw_epoch_keys(paths["device_gnss"])
    route_path, phone = route.split("/", 1)
    root = output_root / "candidate" / route_path / phone
    first = run_native(paths, route, root / "run1", candidate=True)
    repeat = run_native(paths, route, root / "run2", candidate=True)
    if first["submission"]["sha256"] != repeat["submission"]["sha256"] or first["summary"]["sha256"] != repeat["summary"]["sha256"]:
        raise _fail(f"candidate repeat differs: {route}")
    return {"dataset_id": route, "raw_inputs": raw_inputs, "raw_epoch_count": len(raw_keys), "target_epoch_count": len(raw_keys) - 1, "run1": first, "run2": repeat, "repeat_byte_identical": True, "truth_open_count": 0, "mat_read_or_generated": False}


def flag_off_report(freeze: dict[str, Any], output_root: Path, route: str) -> dict[str, Any]:
    paths = input_paths(route)
    raw_inputs = verify_inputs(freeze, route)
    raw_keys = read_raw_epoch_keys(paths["device_gnss"])
    route_path, phone = route.split("/", 1)
    root = output_root / "flag-off" / route_path / phone
    first = run_native(paths, route, root / "run1", candidate=False, allow_failure=(route == TARGET_ROUTE))
    if first.get("return_code") != 0:
        return {"dataset_id": route, "raw_inputs": raw_inputs, "raw_epoch_count": len(raw_keys), "run1": first, "historical_fail_closed": True, "truth_open_count": 0, "mat_read_or_generated": False}
    repeat = run_native(paths, route, root / "run2", candidate=False)
    baseline = FLAG_OFF_BASELINES.get(route)
    if baseline:
        if first["submission"]["sha256"] != baseline.get("submission_sha256") or first["summary"]["sha256"] != baseline.get("summary_sha256"):
            raise _fail(f"flag-off frozen baseline changed: {route}")
    if first["submission"]["sha256"] != repeat["submission"]["sha256"] or first["summary"]["sha256"] != repeat["summary"]["sha256"]:
        raise _fail(f"flag-off repeat differs: {route}")
    return {"dataset_id": route, "raw_inputs": raw_inputs, "raw_epoch_count": len(raw_keys), "run1": first, "run2": repeat, "repeat_byte_identical": True, "frozen_baseline_byte_identical": bool(baseline), "truth_open_count": 0, "mat_read_or_generated": False}


def run_matrix(output_root: Path = DEFAULT_OUTPUT) -> dict[str, Any]:
    freeze = verify_freeze()
    output_root = output_root.resolve()
    _reject_forbidden(output_root)
    if output_root.exists() and any(output_root.iterdir()):
        raise _fail(f"refusing to overwrite nonempty Phase43 output: {output_root}")
    output_root.mkdir(parents=True, exist_ok=True)
    candidates: dict[str, Any] = {}
    flag_off: dict[str, Any] = {}
    for route in ROUTES:
        candidates[route] = route_report(freeze, output_root, route)
    for route in ROUTES:
        flag_off[route] = flag_off_report(freeze, output_root, route)
    for route in SAFE_ROUTES:
        candidate_summary = candidates[route]["run1"]["projection"]["summary"]
        off_run = flag_off[route]["run1"]
        if off_run.get("return_code") != 0:
            raise _fail(f"safe-route flag-off control failed: {route}")
        off_summary = off_run["projection"]["summary"]
        if _project(candidate_summary) != _project(off_summary):
            raise _fail(f"safe-route projected summary changed: {route}")
        if candidates[route]["run1"]["submission"]["sha256"] != off_run["submission"]["sha256"]:
            raise _fail(f"safe-route projected submission changed: {route}")
    result = {
        "schema_version": "smartphone-r5-phase43-native-fallback-seed-quality-anchor-recovery-structural-seal.v1",
        "phase": 43,
        "status": "sealed-truth-free-structural-go" if candidates else "sealed-structural-no-go",
        "decision": {"passed": True, "rule": "frozen Phase43 AND gates", "truth_open_count": 0},
        "candidate_flag": CANDIDATE_FLAG,
        "routes": ROUTES,
        "target_route": TARGET_ROUTE,
        "candidate_runs": candidates,
        "flag_off_runs": flag_off,
        "truth_accounting": {"truth_open_count": 0, "ground_truth_materialized": 0, "validation_truth_reads": 0, "holdout_truth_reads": 0, "mat_read_or_generated": False, "precomputed_coordinates_used": 0, "device_wls_coordinates_used": 0, "kaggle_or_token_access": 0},
        "native_policy": {"production_default_changed": False, "sentinel_factor_bypass": False, "factor_builder_elevation_mask_deg": 0.0},
    }
    atomic_json(output_root / RESULT_NAME, result)
    manifest = {"schema_version": "smartphone-r5-phase43-native-fallback-seed-quality-anchor-recovery-manifest.v1", "phase": 43, "freeze": {"path": relative(FREEZE), "sha256": FREEZE_SHA256}, "binary": {"path": relative(BINARY), "sha256": sha256(BINARY)}, "result": {"path": relative(output_root / RESULT_NAME), "sha256": sha256(output_root / RESULT_NAME)}, "candidate_routes": list(ROUTES), "repeat_runs": 2, "truth_open_count": 0, "mat_read_or_generated": False}
    atomic_json(output_root / MANIFEST_NAME, manifest)
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-matrix", action="store_true", help="run all candidate/control routes")
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()
    try:
        if not args.run_matrix:
            verify_freeze()
            print("phase43 freeze: verified")
            return 0
        result = run_matrix(args.output)
        print(json.dumps({"status": result["status"], "passed": result["decision"]["passed"], "output": relative(args.output.resolve())}, indent=2))
        return 0
    except (OSError, Phase43Error) as exc:
        print(f"phase43: {exc}", file=os.sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
