#!/usr/bin/env python3
"""Truth-free structural matrix for the Phase58 native C/N0 candidate.

The native implementation was frozen after the Phase58 raw-only audit.  This
runner verifies the frozen source/input contract, executes the already-built
no-base binary on the four pinned Pixel5 routes, and checks candidate repeat,
flag-off, and projected structural identity.  It never opens a truth file or
uses a previous position/metric payload as an estimator input.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import tempfile
from typing import Any


ROOT = Path(__file__).resolve().parents[3]
FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_phase58_native_cn0_doppler_calibration_freeze_v1.json"
FREEZE_SHA256 = "b4884659a84be8c3afd5daf6f9bbab11fae958b34af942190aa4eb6ef313d526"
BINARY = ROOT / "build/apps/gnss_fgo_imu_no_base"
BINARY_SHA256 = "cd6c713e8e50fbe0dacff4b0968f7e5d20744f0e64336432331347afbd55c7e9"
DEFAULT_OUTPUT = ROOT / "output/smartphone-r5/phase58-native-cn0-doppler-calibration-v1"
PHASE43_ROOT = ROOT / "output/smartphone-r5/phase43-native-fallback-seed-quality-anchor-recovery-v1/candidate"

ROUTES = (
    "2021-03-16-18-59-us-ca-mtv-a/pixel5",
    "2021-08-24-20-32-us-ca-mtv-h/pixel5",
    "2022-04-01-18-22-us-ca-lax-t/pixel5",
    "2023-03-08-21-34-us-ca-mtv-u/pixel5",
)

INPUTS = {
    ROUTES[0]: {
        "device_gnss.csv": "output/smartphone-r5/phase25-raw-clock-eval-v1/raw/2021-03-16-18-59-us-ca-mtv-a/pixel5/device_gnss.csv",
        "device_imu.csv": "output/smartphone-r5/phase25-raw-clock-eval-v1/raw/2021-03-16-18-59-us-ca-mtv-a/pixel5/device_imu.csv",
        "brdc.nav": "output/smartphone-r5/phase25-raw-clock-eval-v1/raw/2021-03-16-18-59-us-ca-mtv-a/pixel5/brdc.nav",
    },
    ROUTES[1]: {
        "device_gnss.csv": "output/smartphone-r5/phase37-pixel5-repeatability-v1/routes/2021-08-24-20-32-us-ca-mtv-h/pixel5/inputs/device_gnss.csv",
        "device_imu.csv": "output/smartphone-r5/phase37-pixel5-repeatability-v1/routes/2021-08-24-20-32-us-ca-mtv-h/pixel5/inputs/device_imu.csv",
        "brdc.nav": "output/smartphone-r5/phase37-pixel5-repeatability-v1/routes/2021-08-24-20-32-us-ca-mtv-h/pixel5/inputs/brdc.nav",
    },
    ROUTES[2]: {
        "device_gnss.csv": "output/smartphone-r5/phase37-pixel5-repeatability-v1/routes/2022-04-01-18-22-us-ca-lax-t/pixel5/inputs/device_gnss.csv",
        "device_imu.csv": "output/smartphone-r5/phase37-pixel5-repeatability-v1/routes/2022-04-01-18-22-us-ca-lax-t/pixel5/inputs/device_imu.csv",
        "brdc.nav": "output/smartphone-r5/phase37-pixel5-repeatability-v1/routes/2022-04-01-18-22-us-ca-lax-t/pixel5/inputs/brdc.nav",
    },
    ROUTES[3]: {
        "device_gnss.csv": "output/smartphone-r5/phase37-pixel5-repeatability-v1/routes/2023-03-08-21-34-us-ca-mtv-u/pixel5/inputs/device_gnss.csv",
        "device_imu.csv": "output/smartphone-r5/phase37-pixel5-repeatability-v1/routes/2023-03-08-21-34-us-ca-mtv-u/pixel5/inputs/device_imu.csv",
        "brdc.nav": "output/smartphone-r5/phase37-pixel5-repeatability-v1/routes/2023-03-08-21-34-us-ca-mtv-u/pixel5/inputs/brdc.nav",
    },
}

BASE_FLAGS = (
    "--all-epochs",
    "--android-raw-utc-keys",
    "--android-raw-clock-only",
    "--android-utc-wall-clock-fallback",
    "--native-pdc-imu-tdcp-no-bridge",
    "--native-quality-anchor",
    "--native-fallback-seed-quality-anchor-recovery",
)
CANDIDATE_FLAG = "--native-cn0-doppler-calibration"
ALPHA = 0.7586783350728457
REFERENCE_CN0 = 40.0
EARTH_RADIUS_M = 6_371_000.0
MAX_SPEED_MPS = 70.0


class Phase58NativeError(ValueError):
    """Raised when the immutable native structural contract fails."""


def fail(message: str) -> Phase58NativeError:
    return Phase58NativeError(message)


def reject_forbidden(path: Path | str) -> None:
    token = str(path).lower()
    if token.endswith(".mat") or ".mat/" in token or ".mat\\" in token:
        raise fail(f"MAT input/output is forbidden: {path}")


def sha256(path: Path) -> str:
    reject_forbidden(path)
    if not path.is_file():
        raise fail(f"missing file: {path}")
    digest = hashlib.sha256()
    try:
        with path.open("rb") as handle:
            for chunk in iter(lambda: handle.read(1024 * 1024), b""):
                digest.update(chunk)
    except OSError as exc:
        raise fail(f"failed to hash {path}: {exc}") from exc
    return digest.hexdigest()


def load_json(path: Path, label: str) -> dict[str, Any]:
    reject_forbidden(path)
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise fail(f"invalid {label}: {path}: {exc}") from exc
    if not isinstance(value, dict):
        raise fail(f"{label} is not an object: {path}")
    return value


def atomic_write(path: Path, payload: bytes) -> None:
    reject_forbidden(path)
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


def verify_freeze() -> dict[str, Any]:
    if sha256(FREEZE) != FREEZE_SHA256:
        raise fail("native implementation freeze hash changed")
    freeze = load_json(FREEZE, "native implementation freeze")
    if freeze.get("status") != "frozen-before-native-code-and-structural-matrix":
        raise fail("native implementation freeze is not pre-matrix")
    if freeze.get("authority", {}).get("base_commit") != "8da76f0":
        raise fail("native implementation freeze base commit changed")
    fixed = freeze.get("fixed_model", {})
    if fixed.get("alpha_mps_at_reference") != ALPHA or fixed.get("reference_cn0_dbhz") != REFERENCE_CN0:
        raise fail("fixed native alpha/reference changed")
    if fixed.get("upper_clip") is not False or fixed.get("spp_applied") is not False:
        raise fail("native clip/SPP contract changed")
    if fixed.get("existing_p85_over_12_path") != "untouched; do not reimplement or mutate observable_upstream_preprocessing":
        raise fail("existing p85/12 contract mutation declared")
    cohort = freeze.get("cohort", {})
    if tuple(cohort.get("route_order", ())) != ROUTES:
        raise fail("native route order changed")
    if set(cohort.get("input_hashes", {})) != set(ROUTES):
        raise fail("native input hash route set changed")
    matrix = freeze.get("structural_matrix", {})
    if matrix.get("candidate_runs_per_route") != 2 or matrix.get("control_runs_per_route") != 1:
        raise fail("native repeat/control counts changed")
    if matrix.get("raw_gnss_reads_expected") != 12 or matrix.get("truth_reads") != 0:
        raise fail("native read accounting changed")
    if not BINARY.is_file() or sha256(BINARY) != BINARY_SHA256:
        raise fail("built native binary hash changed from sealed build")
    return freeze


def verify_inputs(freeze: dict[str, Any]) -> dict[str, dict[str, Any]]:
    expected = freeze["cohort"]["input_hashes"]
    reports: dict[str, dict[str, Any]] = {}
    for route in ROUTES:
        reports[route] = {}
        for filename, relative_path in INPUTS[route].items():
            path = ROOT / relative_path
            digest = sha256(path)
            if digest != expected[route][filename]:
                raise fail(f"pinned input hash mismatch: {route}/{filename}")
            reports[route][filename] = {
                "path": relative(path),
                "bytes": path.stat().st_size,
                "sha256": digest,
            }
    return reports


def _read_prediction(path: Path, dataset_id: str) -> list[tuple[int, float, float]]:
    reject_forbidden(path)
    try:
        lines = path.read_text(encoding="utf-8-sig").splitlines()
    except OSError as exc:
        raise fail(f"failed to read native submission: {path}: {exc}") from exc
    if not lines or lines[0] != "phone,UnixTimeMillis,LatitudeDegrees,LongitudeDegrees":
        raise fail(f"native submission header mismatch: {path}")
    rows: list[tuple[int, float, float]] = []
    previous: int | None = None
    for line_number, line in enumerate(lines[1:], start=2):
        fields = line.split(",")
        if len(fields) != 4 or fields[0] != dataset_id:
            raise fail(f"native submission key mismatch at {path}:{line_number}")
        try:
            timestamp = int(fields[1])
            latitude = float(fields[2])
            longitude = float(fields[3])
        except ValueError as exc:
            raise fail(f"invalid native submission row at {path}:{line_number}") from exc
        if previous is not None and timestamp <= previous:
            raise fail(f"native submission timestamps are not increasing: {path}")
        if not math.isfinite(latitude) or not math.isfinite(longitude):
            raise fail(f"nonfinite native submission coordinate: {path}:{line_number}")
        if not -90.0 <= latitude <= 90.0 or not -180.0 <= longitude <= 180.0:
            raise fail(f"out-of-range native submission coordinate: {path}:{line_number}")
        rows.append((timestamp, latitude, longitude))
        previous = timestamp
    if not rows:
        raise fail(f"native submission is empty: {path}")
    return rows


def _speed_report(rows: list[tuple[int, float, float]]) -> dict[str, Any]:
    speeds: list[float] = []
    for previous, current in zip(rows, rows[1:]):
        dt = (current[0] - previous[0]) / 1000.0
        if dt <= 0.0:
            raise fail("native output has non-positive dt")
        lat0 = math.radians(previous[1])
        lat1 = math.radians(current[1])
        dlat = lat1 - lat0
        dlon = math.radians(current[2] - previous[2])
        haversine = math.sin(dlat / 2.0) ** 2 + math.cos(lat0) * math.cos(lat1) * math.sin(dlon / 2.0) ** 2
        distance = 2.0 * EARTH_RADIUS_M * math.asin(min(1.0, math.sqrt(max(0.0, haversine))))
        speeds.append(distance / dt)
    return {
        "transition_count": len(speeds),
        "max_speed_mps": max(speeds) if speeds else 0.0,
        "over_70_mps_count": sum(speed > MAX_SPEED_MPS for speed in speeds),
        "finite": all(math.isfinite(speed) for speed in speeds),
    }


def _pick(value: Any, keys: tuple[str, ...]) -> dict[str, Any]:
    if not isinstance(value, dict):
        return {}
    return {key: value[key] for key in keys if key in value}


def structural_projection(summary: dict[str, Any]) -> dict[str, Any]:
    """Keep fields that must not depend on the sigma composition."""

    projection: dict[str, Any] = {
        key: summary.get(key)
        for key in (
            "schema_version",
            "dataset_id",
            "status",
            "truth_used",
            "base_factors",
            "no_base_contract",
            "production_default_changed",
            "fgo_imu_sparse_recovery",
            "native_pdc_state_bridge",
            "native_pdc_imu_tdcp",
            "native_pdc_imu_tdcp_no_bridge",
            "native_signal_bias_states",
            "native_residual_ionosphere",
            "native_upstream_quality",
            "native_upstream_absolute_doppler_screen",
            "native_carrier_code_leveling",
            "native_carrier_code_innovation_reset",
            "native_upstream_stop_constraints",
            "native_upstream_position_offset",
            "native_signal_specific_galileo_tgd",
            "native_quality_anchor",
            "native_fallback_seed_quality_anchor_recovery",
            "android_utc_wall_clock_fallback",
            "android_utc_wall_clock_fallback_applied",
            "skip_epochs",
            "all_epochs",
        )
    }
    projection["inputs"] = summary.get("inputs")
    projection["android_gnss_diagnostics"] = _pick(
        summary.get("android_gnss_diagnostics"),
        (
            "input_rows", "raw_rows", "selected_rows", "selected_epochs", "carrier_rows",
            "doppler_rows", "clock_discontinuities", "enriched_pseudorange_checks",
            "enriched_pseudorange_mismatches", "enriched_pseudorange_ignored_rows",
            "enriched_pseudorange_input_ignored", "pseudorange_source", "timing_formula",
            "no_device_wls_seed",
        ),
    )
    projection["epochs"] = summary.get("epochs")
    projection["raw_utc_key_contract"] = summary.get("raw_utc_key_contract")
    projection["output_contract"] = summary.get("output_contract")
    projection["graph"] = _pick(
        summary.get("graph"),
        ("factors", "values", "imu_intervals", "upstream_stop_epochs", "upstream_stop_velocity_factors", "upstream_stop_pose_factors", "upstream_stop_imu_samples", "converged"),
    )
    projection["quality_anchor_initialization"] = _pick(
        summary.get("quality_anchor_initialization"),
        ("enabled", "selected", "truth_free", "graph_model_changed", "eligible_candidates", "fallback_epochs", "anchor_index", "anchor_satellites", "anchor_gdop"),
    )
    projection["tdcp_contract"] = _pick(
        summary.get("tdcp_contract"),
        ("enabled", "factors_built", "factors_inserted", "candidate_pairs", "rejected_gap", "rejected_clock_discontinuity", "rejected_missing_previous", "rejected_loss_of_lock", "rejected_invalid_measurement", "rejected_code_phase_jump", "finite_residuals", "nonfinite_residuals", "arc_count", "min_arc_length_epochs", "median_arc_length_epochs", "max_arc_length_epochs", "sigma_m", "max_gap_s", "code_phase_jump_threshold_m", "pair_key", "adr_state_slip_fail_closed", "standalone_carrier_ambiguity_factors", "base_or_double_difference_factors"),
    )
    projection["native_pdc_state_bridge_report"] = _pick(
        summary.get("native_pdc_state_bridge_report"),
        ("enabled", "ok", "epochs", "pseudorange_rows", "doppler_rows", "motion_intervals", "valid_position_states", "rejected_position_states", "valid_velocity_states", "state_seeds", "first_epoch_doppler_rows", "later_epoch_doppler_rows", "epochs_with_doppler", "finite_raw_clock_drift_epochs", "invalid_or_large_epoch_intervals", "all_state_velocity_over_bound", "all_state_clock_rate_over_bound"),
    )
    return projection


def validate_summary(summary_path: Path, route: str, candidate: bool) -> tuple[dict[str, Any], dict[str, Any] | None]:
    summary = load_json(summary_path, "native summary")
    for key, expected in (("dataset_id", route), ("truth_used", False), ("production_default_changed", False), ("native_quality_anchor", True), ("native_pdc_imu_tdcp_no_bridge", True), ("native_pdc_state_bridge", False)):
        if summary.get(key) != expected:
            raise fail(f"summary contract mismatch {key}: {route}")
    graph = summary.get("graph", {})
    epochs = summary.get("epochs", {})
    raw = summary.get("raw_utc_key_contract", {})
    tdcp = summary.get("tdcp_contract", {})
    if graph.get("converged") is not True or int(epochs.get("problem", 0)) <= 1 or int(epochs.get("output", 0)) <= 1:
        raise fail(f"native graph/epoch contract failed: {route}")
    if int(epochs.get("pseudorange_factors", 0)) <= 0 or int(epochs.get("tdcp_factors_built", 0)) <= 0:
        raise fail(f"required factors missing: {route}")
    if int(raw.get("unresolved_epochs", -1)) != 0 or int(raw.get("target_epochs", 0)) != int(raw.get("exact_solution_epochs", -1)):
        raise fail(f"raw UTC output alignment failed: {route}")
    if int(tdcp.get("factors_built", 0)) != int(tdcp.get("factors_inserted", -1)) or int(tdcp.get("nonfinite_residuals", -1)) != 0:
        raise fail(f"TDCP structural contract failed: {route}")
    quality = summary.get("quality_anchor_initialization", {})
    if quality.get("enabled") is not True or quality.get("selected") is not True or quality.get("truth_free") is not True or quality.get("graph_model_changed") is not False:
        raise fail(f"quality-anchor structural contract failed: {route}")
    calibration = summary.get("native_cn0_doppler_calibration")
    if candidate:
        if not isinstance(calibration, dict) or calibration.get("enabled") is not True:
            raise fail(f"native C/N0 telemetry missing: {route}")
        if calibration.get("alpha_mps_at_reference") != ALPHA or calibration.get("reference_cn0_dbhz") != REFERENCE_CN0:
            raise fail(f"native C/N0 alpha/reference mismatch: {route}")
        if calibration.get("upper_clip") is not False or calibration.get("spp_applied") is not False or calibration.get("tdcp_applied") is not False or calibration.get("single_difference_doppler_applied") is not False:
            raise fail(f"native C/N0 scope contract failed: {route}")
        rows = int(calibration.get("candidate_rows", 0))
        finite_rows = int(calibration.get("finite_cn0_rows", -1))
        affected = int(calibration.get("factors_affected", 0))
        if rows <= 0 or finite_rows < 0 or finite_rows > rows or affected <= 0:
            raise fail(f"native C/N0 telemetry population failed: {route}")
        for key in ("model_sigma_min_mps", "model_sigma_median_mps", "model_sigma_p95_mps", "model_sigma_max_mps"):
            if not math.isfinite(float(calibration.get(key, math.nan))):
                raise fail(f"native C/N0 telemetry nonfinite: {route}/{key}")
    elif calibration is not None:
        raise fail(f"flag-off summary unexpectedly exposes native C/N0 telemetry: {route}")
    return structural_projection(summary), calibration if isinstance(calibration, dict) else None


def artifact_report(submission: Path, summary_path: Path, route: str, candidate: bool) -> dict[str, Any]:
    rows = _read_prediction(submission, route)
    speed = _speed_report(rows)
    if not speed["finite"] or speed["over_70_mps_count"] != 0:
        raise fail(f"output speed contract failed: {route}")
    projection, calibration = validate_summary(summary_path, route, candidate)
    return {
        "submission": {"path": relative(submission), "bytes": submission.stat().st_size, "sha256": sha256(submission), "rows": len(rows)},
        "summary": {"path": relative(summary_path), "bytes": summary_path.stat().st_size, "sha256": sha256(summary_path)},
        "speed": speed,
        "projection": projection,
        "calibration": calibration,
    }


def _run_case(output_root: Path, route: str, variant: str, run_number: int, candidate: bool) -> dict[str, Any]:
    route_dir = output_root / route / variant / f"run{run_number}"
    if route_dir.exists():
        raise fail(f"refusing to overwrite structural output: {route_dir}")
    route_dir.mkdir(parents=True, exist_ok=True)
    paths = INPUTS[route]
    submission = route_dir / "submission.csv"
    summary = route_dir / "summary.json"
    command = [
        str(BINARY.relative_to(ROOT)),
        "--android-gnss", paths["device_gnss.csv"],
        "--android-imu", paths["device_imu.csv"],
        "--nav", paths["brdc.nav"],
        "--out", relative(submission),
        "--summary-json", relative(summary),
        "--dataset-id", route,
        *BASE_FLAGS,
    ]
    if candidate:
        command.append(CANDIDATE_FLAG)
    env = os.environ.copy()
    local_lib = "/home/sasaki/.local/lib"
    env["LD_LIBRARY_PATH"] = local_lib + (":" + env["LD_LIBRARY_PATH"] if env.get("LD_LIBRARY_PATH") else "")
    try:
        completed = subprocess.run(command, cwd=ROOT, env=env, capture_output=True, text=True, timeout=1800)
    except (OSError, subprocess.TimeoutExpired) as exc:
        raise fail(f"native structural process failed: {route}/{variant}/run{run_number}: {exc}") from exc
    log = (completed.stdout or "") + (completed.stderr or "")
    atomic_write(route_dir / "run.log", log.encode())
    if completed.returncode != 0:
        raise fail(f"native structural process returned {completed.returncode}: {route}/{variant}/run{run_number}")
    if not submission.is_file() or not summary.is_file():
        raise fail(f"native structural artifacts missing: {route}/{variant}/run{run_number}")
    report = artifact_report(submission, summary, route, candidate)
    report["command"] = command
    report["variant"] = variant
    report["run"] = run_number
    return report


def _phase43_artifacts(route: str) -> dict[str, Any]:
    base = PHASE43_ROOT / route / "run1"
    submission = base / "submission.csv"
    summary = base / "summary.json"
    if not submission.is_file() or not summary.is_file():
        raise fail(f"missing frozen Phase43 flag-off artifact: {route}")
    # The old artifact is a reference only; it is never used as an estimator
    # input.  Its bytes are included in the flag-off identity report.
    return {
        "submission_sha256": sha256(submission),
        "summary_sha256": sha256(summary),
        "submission_bytes": submission.stat().st_size,
        "summary_bytes": summary.stat().st_size,
    }


def run_matrix(output_root: Path) -> dict[str, Any]:
    freeze = verify_freeze()
    output_root = output_root.resolve()
    reject_forbidden(output_root)
    if output_root.exists() and any(output_root.iterdir()):
        raise fail(f"refusing to overwrite nonempty native matrix output: {output_root}")
    input_reports = verify_inputs(freeze)
    output_root.mkdir(parents=True, exist_ok=True)
    route_reports: dict[str, Any] = {}
    cases_started = 0
    try:
        for route in ROUTES:
            cases_started += 1
            control = _run_case(output_root, route, "control", 1, False)
            cases_started += 1
            candidate_one = _run_case(output_root, route, "candidate", 1, True)
            cases_started += 1
            candidate_two = _run_case(output_root, route, "candidate", 2, True)
            if candidate_one["submission"]["sha256"] != candidate_two["submission"]["sha256"] or candidate_one["summary"]["sha256"] != candidate_two["summary"]["sha256"]:
                raise fail(f"candidate repeat bytes differ: {route}")
            if candidate_one["projection"] != candidate_two["projection"] or candidate_one["calibration"] != candidate_two["calibration"]:
                raise fail(f"candidate repeat structural projection differs: {route}")
            phase43 = _phase43_artifacts(route)
            if control["submission"]["sha256"] != phase43["submission_sha256"] or control["summary"]["sha256"] != phase43["summary_sha256"]:
                raise fail(f"flag-off bytes differ from Phase43 reference: {route}")
            if control["projection"] != candidate_one["projection"]:
                raise fail(f"control/candidate projected structural identity differs: {route}")
            route_reports[route] = {
                "control": control,
                "candidate_run1": candidate_one,
                "candidate_run2": candidate_two,
                "phase43_flag_off_reference": phase43,
                "gates": {
                    "candidate_repeat_identity": True,
                    "flag_off_identity": True,
                    "projected_identity": True,
                    "finite_outputs": True,
                    "raw_utc_key_contract": True,
                    "converged": True,
                    "tdcp_built_equals_inserted": True,
                    "over_70_mps": True,
                },
            }
        result = {
            "schema_version": "smartphone-r5-phase58-native-cn0-doppler-calibration-structural-result.v1",
            "phase": 58,
            "execution_label": "Luna Max",
            "status": "go-cn0-doppler-calibration-structural",
            "truth_free": True,
            "freeze": {"path": relative(FREEZE), "sha256": FREEZE_SHA256, "base_commit": "8da76f0"},
            "source_commit": "c47a67d53b269304a52bf30e95bf1804465288b6",
            "candidate": {"flag": CANDIDATE_FLAG, "alpha_mps_at_reference": ALPHA, "reference_cn0_dbhz": REFERENCE_CN0, "formula": "max(existing_doppler_sigma_mps, alpha*10^(-(Cn0DbHz-reference)/20))", "scope": "FGO undifferenced Doppler only", "spp_applied": False, "upper_clip": False},
            "routes": route_reports,
            "input_hashes": input_reports,
            "gates": {"route_count": True, "raw_input_integrity": True, "candidate_repeat_identity": True, "flag_off_identity": True, "projected_identity": True, "finite_outputs": True, "raw_utc_key_contract": True, "converged": True, "tdcp_built_equals_inserted": True, "over_70_mps": True, "all_passed": True},
            "read_accounting": {"single_process_per_case": True, "routes": 4, "candidate_runs_per_route": 2, "control_runs_per_route": 1, "native_solver_invocations": 12, "raw_device_gnss_process_reads": 12, "raw_device_imu_process_reads": 12, "navigation_process_reads": 12, "hash_verification_reads": {"device_gnss": 4, "device_imu": 4, "brdc.nav": 4}, "truth_reads": 0, "validation_holdout_reads": 0, "mat_reads_or_generated": 0, "coordinate_or_wls_inputs": 0, "kaggle_token_access": 0, "post_truth_tuning": False},
            "phase43_champion_preserved": True,
            "phase51_experimental_preserved": True,
            "zero_point_782": "not evaluated before separately authorized accuracy phase",
        }
        result_path = output_root / "phase58_native_cn0_doppler_calibration_structural_result.json"
        atomic_json(result_path, result)
        result_hash = sha256(result_path)
        manifest = {
            "schema_version": "smartphone-r5-phase58-native-cn0-doppler-calibration-structural-manifest.v1",
            "phase": 58,
            "status": "sealed-truth-free-structural-matrix",
            "freeze": {"path": relative(FREEZE), "sha256": FREEZE_SHA256},
            "source_commit": result["source_commit"],
            "candidate_flag": CANDIDATE_FLAG,
            "route_order": list(ROUTES),
            "runs": {"candidate_per_route": 2, "control_per_route": 1, "solver_invocations": 12},
            "read_accounting": result["read_accounting"],
            "result": {"path": relative(result_path), "sha256": result_hash, "bytes": result_path.stat().st_size},
            "truth_reads": 0,
            "all_gates_passed": True,
        }
        atomic_json(output_root / "phase58_native_cn0_doppler_calibration_structural_manifest.json", manifest)
        return result
    except Phase58NativeError as exc:
        atomic_json(output_root / "phase58_native_cn0_doppler_calibration_structural_failure.json", {"schema_version": "smartphone-r5-phase58-native-cn0-doppler-calibration-structural-failure.v1", "status": "fail-closed", "error": str(exc), "truth_reads": 0, "native_solver_invocations_started": cases_started, "partial_routes": route_reports})
        raise


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--verify-freeze", action="store_true")
    parser.add_argument("--run-matrix", action="store_true")
    parser.add_argument("--output-root", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args(argv)
    try:
        if args.verify_freeze:
            verify_freeze()
        if args.run_matrix:
            result = run_matrix(args.output_root)
            print(json.dumps({"all_gates_passed": result["gates"]["all_passed"], "native_solver_invocations": result["read_accounting"]["native_solver_invocations"], "truth_reads": result["read_accounting"]["truth_reads"], "status": result["status"]}, sort_keys=True))
        elif not args.verify_freeze:
            parser.error("one of --verify-freeze or --run-matrix is required")
        return 0
    except Phase58NativeError as exc:
        print(f"phase58 native structural failure: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
