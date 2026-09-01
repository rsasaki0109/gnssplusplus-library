#!/usr/bin/env python3
"""Truth-free structural matrix for the Phase65 base-pseudorange candidate.

The native candidate is an opt-in subtraction of a source-compatible,
same-satellite/same-signal base RINEX residual stream.  This runner verifies
the immutable source and input pins, hashes every base member before launch,
then executes one flag-off control and two candidate invocations per route.
It never opens truth/MAT/validation data and writes only atomic structural
artifacts.  Accuracy truth is deliberately outside this phase.
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
import time
from typing import Any


ROOT = Path(__file__).resolve().parents[3]
FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_phase65_native_base_pseudorange_compensation_freeze_v1.json"
FREEZE_SHA256 = "1f3cb0bedd9b64d08943ef14f87b6429ade367805ae28511ec56b78dd26033d7"
MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase65_native_base_pseudorange_compensation_manifest_v1.json"
BINARY = ROOT / "build/apps/gnss_fgo_imu_no_base"
BINARY_SHA256 = "f1a3bb43ad68890017cd6e1b273b3df784b002a73bbf606a0f6c5242937403a2"
DEFAULT_OUTPUT = ROOT / "output/smartphone-r5/phase65-native-base-pseudorange-compensation-v1"
PHASE43_ROOT = ROOT / "output/smartphone-r5/phase43-native-fallback-seed-quality-anchor-recovery-v1/candidate"
PHASE58_FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_phase58_native_cn0_doppler_calibration_freeze_v1.json"
PHASE58_FREEZE_SHA256 = "b4884659a84be8c3afd5daf6f9bbab11fae958b34af942190aa4eb6ef313d526"

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

BASE_INPUTS = {
    ROUTES[0]: "output/smartphone-r5/phase63-settings-integrity-recovery-v1/routes/2021-03-16-18-59-us-ca-mtv-a__pixel5/base.obs",
    ROUTES[1]: "output/smartphone-r5/phase63-settings-integrity-recovery-v1/routes/2021-08-24-20-32-us-ca-mtv-h__pixel5/base.obs",
    ROUTES[2]: "output/smartphone-r5/phase63-settings-integrity-recovery-v1/routes/2022-04-01-18-22-us-ca-lax-t__pixel5/base.obs",
    ROUTES[3]: "output/smartphone-r5/phase63-settings-integrity-recovery-v1/routes/2023-03-08-21-34-us-ca-mtv-u__pixel5/base.obs",
}

RAW_INPUT_HASHES = {
    ROUTES[0]: {
        "device_gnss.csv": "c7d50d5127d16586adc6c79d724758e298b385496da22c5e5dfd6ec522cbc863",
        "device_imu.csv": "afc540e7c4ce2ca66b442a1afbcd604e9f6b3d2cc4d3733183739901b5b97bd6",
        "brdc.nav": "6adfaf7fe4452a4faeb94a7b607c15e05f578c46a028a030b94aa6f79de194cd",
    },
    ROUTES[1]: {
        "device_gnss.csv": "46482b82db0992c1f063dbd9cf697268605234d3e38bcbd23525fd4b60bc17a7",
        "device_imu.csv": "fa3f17d07570fdbb8030f307130f33ca3613e8125475c2a907c48f7db3455480",
        "brdc.nav": "147d948f0eba3bf09e295e7f67fbe8db60c25e236bc3c7d958dc933483f10909",
    },
    ROUTES[2]: {
        "device_gnss.csv": "50362c01bff3e0bb7088e54021164591cd750227ed97c2fd7d95d763a08798f1",
        "device_imu.csv": "2e39a3e9f294c64b8ecfd452d0960025d1013b97f2d7497e6e48a2a1997b38c5",
        "brdc.nav": "443d3d5a73f4895b83e576e24a568f4658f869e79a480856de7f0717763dcbe6",
    },
    ROUTES[3]: {
        "device_gnss.csv": "a0fc8e71bdfc03be61b99efcd7d41fbba8ffec126df78b55243f681fd211f204",
        "device_imu.csv": "c7d726e1cc0dacc7a569bd8be9bc2333765bbb3b3010203a4aea2e49778101ab",
        "brdc.nav": "8893ef62fecdd9986f6b2cf1b7b980defa6430da5801aafcab4ecf4c99a03b92",
    },
}

BASE_INPUT_HASHES = {
    ROUTES[0]: {"sha256": "380b8ff9091344fb756697e27f0983d9a0ba2cf0c201b96849bfc1ecc1af0e52", "bytes": 10708536, "dt_s": 1.0, "window": 151, "xyz": [-2703115.921, -4291767.2078, 3854247.9066]},
    ROUTES[1]: {"sha256": "4d3e37cbe0347fa56216db54ede9e0f30731885f337f1653ab5a86afb2bb2150", "bytes": 11854125, "dt_s": 1.0, "window": 151, "xyz": [-2698116.8365, -4301328.0461, 3847285.9307]},
    ROUTES[2]: {"sha256": "d731e0e8a7ba4396d62340c85b6238e66c50c6a349621f9dcea0ea6885fd4cfe", "bytes": 719969, "dt_s": 15.0, "window": 11, "xyz": [-2507798.7984, -4676369.6918, 3526890.8008]},
    ROUTES[3]: {"sha256": "aedb7a39e7b6612ea97964b363c74cd2c10318255b7f2287f4720d18e71803e6", "bytes": 5950275, "dt_s": 1.0, "window": 151, "xyz": [-2698116.8365, -4301328.0461, 3847285.9307]},
}

PHASE43_CONTROL = {
    ROUTES[0]: {"submission": "d4d7652e5d12389466e586fe2d8e85d34977a7e11036cee2f591a89293c5426c", "summary": "d4980260e257c92d7e2fb716f900501fd55c3eb9278f9b38a4f72ac2e62303fe"},
    ROUTES[1]: {"submission": "3cfe97750927fa268f71bbe7393754ce7a1eb39d937e085b177c50a71eec10ae", "summary": "23f409e5708dc547bc86b156e07a6fb305732cb9ce8eb3da57f26fd487e4404e"},
    ROUTES[2]: {"submission": "4eb2b566708a87db4903610a41cd648c7ff065d35710d358894a78eaa8e116e3", "summary": "f4fb3bc2b09d44700ee83772d4b23ad93b68d07eff6743b9525626f805264eb8"},
    ROUTES[3]: {"submission": "524769cdf67aa857eefaafdadf943dd76586dda6a53e8b6d1df4a707d7699f71", "summary": "eeb4f60e352ea00277604516b432673317ab035daa601135c6e9f989cb118926"},
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
CANDIDATE_FLAG = "--native-base-pseudorange-compensation"
BASE_FLAG = "--native-base-rinex"
BASE_SHA_FLAG = "--native-base-rinex-sha256"
EARTH_RADIUS_M = 6_371_000.0
MAX_SPEED_MPS = 70.0


class Phase65Error(ValueError):
    """Raised when an immutable Phase65 structural gate fails."""


def fail(message: str) -> Phase65Error:
    return Phase65Error(message)


def reject_forbidden(path: Path | str) -> None:
    token = str(path).lower()
    if token.endswith(".mat") or ".mat/" in token or ".mat\\" in token:
        raise fail(f"MAT path is forbidden: {path}")
    if any(term in token for term in ("ground_truth", "validation", "holdout", "kaggle", "token")):
        raise fail(f"forbidden truth/validation path: {path}")


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
    descriptor, temporary = tempfile.mkstemp(prefix=f".{path.name}.", suffix=".tmp", dir=str(path.parent))
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
        raise fail("Phase65 freeze hash changed")
    freeze = load_json(FREEZE, "Phase65 freeze")
    if freeze.get("status") != "frozen-before-phase65-source-edit-and-structural-read":
        raise fail("Phase65 freeze status changed")
    if freeze.get("authority", {}).get("base_commit") != "6dc1c9d":
        raise fail("Phase65 authority commit changed")
    source = freeze.get("source_contract", {})
    if source.get("repository_commit") != "29923f9f370f09ebc00f96d8cca375007a18e7d5":
        raise fail("upstream source commit changed")
    if source.get("correct_pseudorange_m_sha256") != "b0536ccff478b0aff253448ffb7a203c715b8064dd8dc85898e38f1f05d0441e":
        raise fail("upstream correction source hash changed")
    moving = source.get("moving_mean", {})
    if moving.get("one_second_samples") != 151 or moving.get("fifteen_second_samples") != 11:
        raise fail("moving-mean contract changed")
    if source.get("coefficient") != 1.0 or source.get("fitted_parameters") is not False:
        raise fail("coefficient/fitting contract changed")
    scope = freeze.get("implementation_scope", {})
    if scope.get("opt_in_flag") != CANDIDATE_FLAG or scope.get("base_path_argument") != BASE_FLAG:
        raise fail("candidate flag/path contract changed")
    if scope.get("default") != "false; Phase43 no-base behavior and bytes remain unchanged when the flag is absent":
        raise fail("flag-off default contract changed")
    if scope.get("no_new_rows") is not True or scope.get("no_new_factors") is not True:
        raise fail("factor/row scope changed")
    if tuple(freeze.get("route_order", ())) != ROUTES:
        raise fail("Phase65 route order changed")
    pinned = freeze.get("sealed_base_inputs_from_phase64", {}).get("routes", {})
    if set(pinned) != set(ROUTES):
        raise fail("sealed base route set changed")
    for route in ROUTES:
        expected = BASE_INPUT_HASHES[route]
        if pinned[route].get("sha256") != expected["sha256"] or pinned[route].get("bytes") != expected["bytes"]:
            raise fail(f"sealed base hash/bytes changed: {route}")
        if pinned[route].get("observed_dt_s") != expected["dt_s"]:
            raise fail(f"sealed base observed dt changed: {route}")
    matrix = freeze.get("structural_matrix", {})
    if matrix.get("control_runs_per_route") != 1 or matrix.get("candidate_runs_per_route") != 2 or matrix.get("expected_solver_invocations") != 12:
        raise fail("structural run count changed")
    if any(matrix.get(key) != 0 for key in ("truth_reads", "mat_reads", "validation_holdout_reads", "kaggle_token_reads")):
        raise fail("forbidden read accounting changed")
    if sha256(PHASE58_FREEZE) != PHASE58_FREEZE_SHA256:
        raise fail("Phase58 input cohort freeze changed")
    if not BINARY.is_file() or sha256(BINARY) != BINARY_SHA256:
        raise fail("built native binary hash changed")
    if not MANIFEST.is_file():
        raise fail("Phase65 structural manifest is missing")
    manifest = load_json(MANIFEST, "Phase65 structural manifest")
    if manifest.get("freeze", {}).get("sha256") != FREEZE_SHA256:
        raise fail("manifest freeze pin changed")
    if manifest.get("binary", {}).get("sha256") != BINARY_SHA256:
        raise fail("manifest binary pin changed")
    if manifest.get("source_commit") != "f8cdf3b":
        raise fail("manifest source commit changed")
    if manifest.get("routes") != list(ROUTES):
        raise fail("manifest route order changed")
    if manifest.get("read_accounting", {}).get("truth_reads") != 0:
        raise fail("manifest truth read contract changed")
    return freeze


def input_paths(route: str) -> dict[str, Path]:
    return {name: ROOT / path for name, path in INPUTS[route].items()} | {"base.obs": ROOT / BASE_INPUTS[route]}


def verify_inputs(route: str) -> dict[str, Any]:
    reports: dict[str, Any] = {}
    for name, path in input_paths(route).items():
        digest = sha256(path)
        expected = BASE_INPUT_HASHES[route]["sha256"] if name == "base.obs" else RAW_INPUT_HASHES[route][name]
        expected_bytes = BASE_INPUT_HASHES[route]["bytes"] if name == "base.obs" else path.stat().st_size
        if digest != expected:
            raise fail(f"input hash mismatch: {route}/{name}")
        if name == "base.obs" and path.stat().st_size != expected_bytes:
            raise fail(f"base byte-size mismatch: {route}")
        reports[name] = {"path": relative(path), "bytes": path.stat().st_size, "sha256": digest}
    return reports


def read_prediction(path: Path, dataset_id: str) -> list[tuple[int, float, float]]:
    reject_forbidden(path)
    try:
        lines = path.read_text(encoding="utf-8-sig").splitlines()
    except OSError as exc:
        raise fail(f"failed to read submission: {path}: {exc}") from exc
    if not lines or lines[0] != "phone,UnixTimeMillis,LatitudeDegrees,LongitudeDegrees":
        raise fail(f"submission header mismatch: {path}")
    rows: list[tuple[int, float, float]] = []
    previous: int | None = None
    for line_number, line in enumerate(lines[1:], start=2):
        fields = line.split(",")
        if len(fields) != 4 or fields[0] != dataset_id:
            raise fail(f"submission key mismatch: {path}:{line_number}")
        try:
            timestamp, latitude, longitude = int(fields[1]), float(fields[2]), float(fields[3])
        except ValueError as exc:
            raise fail(f"non-numeric submission row: {path}:{line_number}") from exc
        if previous is not None and timestamp <= previous:
            raise fail(f"submission timestamps are not increasing: {path}")
        if not all(math.isfinite(value) for value in (latitude, longitude)):
            raise fail(f"nonfinite submission coordinate: {path}:{line_number}")
        if not -90.0 <= latitude <= 90.0 or not -180.0 <= longitude <= 180.0:
            raise fail(f"out-of-range submission coordinate: {path}:{line_number}")
        rows.append((timestamp, latitude, longitude))
        previous = timestamp
    if not rows:
        raise fail(f"empty submission: {path}")
    return rows


def speed_report(rows: list[tuple[int, float, float]]) -> dict[str, Any]:
    speeds: list[float] = []
    for previous, current in zip(rows, rows[1:]):
        dt = (current[0] - previous[0]) / 1000.0
        if dt <= 0.0:
            raise fail("submission has non-positive dt")
        lat0, lat1 = math.radians(previous[1]), math.radians(current[1])
        dlat, dlon = lat1 - lat0, math.radians(current[2] - previous[2])
        h = math.sin(dlat / 2.0) ** 2 + math.cos(lat0) * math.cos(lat1) * math.sin(dlon / 2.0) ** 2
        distance = 2.0 * EARTH_RADIUS_M * math.asin(min(1.0, math.sqrt(max(0.0, h))))
        speeds.append(distance / dt)
    return {"transition_count": len(speeds), "max_speed_mps": max(speeds, default=0.0), "over_70_mps_count": sum(speed > MAX_SPEED_MPS for speed in speeds), "finite": all(math.isfinite(speed) for speed in speeds)}


def validate_summary(path: Path, route: str, candidate: bool) -> dict[str, Any]:
    summary = load_json(path, "native summary")
    for key, expected in (("dataset_id", route), ("truth_used", False), ("production_default_changed", False), ("native_quality_anchor", True), ("native_pdc_imu_tdcp_no_bridge", True), ("native_pdc_state_bridge", False)):
        if summary.get(key) != expected:
            raise fail(f"summary contract mismatch {route}/{key}")
    graph, epochs, raw, tdcp = (summary.get(key) for key in ("graph", "epochs", "raw_utc_key_contract", "tdcp_contract"))
    if not all(isinstance(value, dict) for value in (graph, epochs, raw, tdcp)):
        raise fail(f"summary diagnostics missing: {route}")
    if graph.get("converged") is not True or int(epochs.get("problem", 0)) <= 1 or int(epochs.get("output", 0)) <= 1:
        raise fail(f"graph/epoch gate failed: {route}")
    if int(epochs.get("pseudorange_factors", 0)) <= 0 or int(tdcp.get("factors_built", 0)) <= 0 or int(tdcp.get("factors_inserted", -1)) != int(tdcp.get("factors_built", 0)) or int(tdcp.get("nonfinite_residuals", -1)) != 0:
        raise fail(f"factor gate failed: {route}")
    if int(raw.get("unresolved_epochs", -1)) != 0 or int(raw.get("target_epochs", 0)) != int(raw.get("exact_solution_epochs", -1)):
        raise fail(f"raw UTC alignment gate failed: {route}")
    if candidate:
        telemetry = summary.get("native_base_pseudorange_compensation")
        if not isinstance(telemetry, dict) or telemetry.get("enabled") is not True or telemetry.get("built") is not True or telemetry.get("applied") is not True:
            raise fail(f"base telemetry missing: {route}")
        pin = BASE_INPUT_HASHES[route]
        if telemetry.get("base_member_sha256") != pin["sha256"] or telemetry.get("base_rinex_sha256") != pin["sha256"] or int(telemetry.get("base_rinex_bytes", -1)) != pin["bytes"]:
            raise fail(f"base hash/bytes telemetry mismatch: {route}")
        xyz = telemetry.get("base_coordinate_xyz_m")
        if not isinstance(xyz, list) or len(xyz) != 3 or any(abs(float(a) - float(b)) > 1e-6 for a, b in zip(xyz, pin["xyz"])):
            raise fail(f"base coordinate telemetry mismatch: {route}")
        if abs(float(telemetry.get("observed_interval_s", math.nan)) - pin["dt_s"]) > 1e-6 or int(telemetry.get("moving_mean_samples", -1)) != pin["window"]:
            raise fail(f"base interval/window telemetry mismatch: {route}")
        if telemetry.get("same_satellite_signal_only") is not True or telemetry.get("spp_applied") is not False or telemetry.get("tdcp_applied") is not False or telemetry.get("doppler_applied") is not False or telemetry.get("no_extrapolation_or_endpoint_hold") is not True:
            raise fail(f"base scope contract failed: {route}")
        adopted, corrected = int(telemetry.get("adopted_pseudorange_rows", 0)), int(telemetry.get("adopted_rows_corrected", -1))
        fraction = float(telemetry.get("finite_correction_fraction", math.nan))
        if adopted <= 0 or corrected < 0 or corrected > adopted or not math.isfinite(fraction) or fraction < 0.99 or abs(fraction - corrected / adopted) > 1e-12:
            raise fail(f"base correction coverage failed: {route}")
        for key, minimum in (("correction_abs_p50_m", 0.10), ("correction_abs_p95_m", 0.30)):
            value = float(telemetry.get(key, math.nan))
            if not math.isfinite(value) or value < minimum:
                raise fail(f"base correction materiality failed: {route}/{key}")
        return {"summary": summary, "base": telemetry}
    if "native_base_pseudorange_compensation" in summary:
        raise fail(f"flag-off base telemetry leaked: {route}")
    return {"summary": summary}


def artifact_report(submission: Path, summary: Path, route: str, candidate: bool) -> dict[str, Any]:
    rows = read_prediction(submission, route)
    speed = speed_report(rows)
    if not speed["finite"] or speed["over_70_mps_count"] != 0:
        raise fail(f"speed gate failed: {route}")
    diagnostics = validate_summary(summary, route, candidate)
    return {"submission": {"path": relative(submission), "bytes": submission.stat().st_size, "sha256": sha256(submission), "rows": len(rows)}, "summary": {"path": relative(summary), "bytes": summary.stat().st_size, "sha256": sha256(summary)}, "speed": speed, **diagnostics}


def native_command(route: str, run_dir: Path, candidate: bool) -> list[str]:
    paths = input_paths(route)
    command = [str(BINARY.relative_to(ROOT)), "--android-gnss", relative(paths["device_gnss.csv"]), "--android-imu", relative(paths["device_imu.csv"]), "--nav", relative(paths["brdc.nav"]), "--out", relative(run_dir / "submission.csv"), "--summary-json", relative(run_dir / "summary.json"), "--dataset-id", route, *BASE_FLAGS]
    if candidate:
        command.extend((CANDIDATE_FLAG, BASE_FLAG, relative(paths["base.obs"]), BASE_SHA_FLAG, BASE_INPUT_HASHES[route]["sha256"]))
    for token in command:
        reject_forbidden(token)
    return command


def run_case(output_root: Path, route: str, variant: str, run_number: int, candidate: bool) -> dict[str, Any]:
    run_dir = output_root / route / variant / f"run{run_number}"
    if run_dir.exists():
        raise fail(f"refusing to overwrite structural output: {run_dir}")
    run_dir.mkdir(parents=True, exist_ok=True)
    command = native_command(route, run_dir, candidate)
    environment = os.environ.copy()
    local_lib = "/home/sasaki/.local/lib"
    environment["LD_LIBRARY_PATH"] = local_lib + ((":" + environment["LD_LIBRARY_PATH"]) if environment.get("LD_LIBRARY_PATH") else "")
    started = time.perf_counter()
    try:
        process = subprocess.run(command, cwd=ROOT, env=environment, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, check=False, timeout=1800)
    except (OSError, subprocess.TimeoutExpired) as exc:
        raise fail(f"native process failed: {route}/{variant}/run{run_number}: {exc}") from exc
    log = process.stdout or ""
    atomic_write(run_dir / "run.log", log.encode())
    report: dict[str, Any] = {"variant": variant, "run": run_number, "candidate": candidate, "return_code": process.returncode, "wall_seconds": time.perf_counter() - started, "command": command, "log": {"path": relative(run_dir / "run.log"), "sha256": sha256(run_dir / "run.log")}}
    if process.returncode != 0:
        raise fail(f"native process returned {process.returncode}: {route}/{variant}/run{run_number}")
    submission, summary = run_dir / "submission.csv", run_dir / "summary.json"
    if not submission.is_file() or not summary.is_file():
        raise fail(f"native artifacts missing: {route}/{variant}/run{run_number}")
    report.update(artifact_report(submission, summary, route, candidate))
    return report


def phase43_control(route: str) -> dict[str, Any]:
    base = PHASE43_ROOT / route / "run1"
    submission, summary = base / "submission.csv", base / "summary.json"
    if not submission.is_file() or not summary.is_file():
        raise fail(f"missing Phase43 control artifact: {route}")
    expected = PHASE43_CONTROL[route]
    actual = {"submission": sha256(submission), "summary": sha256(summary)}
    if actual != expected:
        raise fail(f"Phase43 control artifact hash changed: {route}")
    return {"path": relative(base), "submission": {"bytes": submission.stat().st_size, "sha256": actual["submission"]}, "summary": {"bytes": summary.stat().st_size, "sha256": actual["summary"]}}


def run_matrix(output_root: Path = DEFAULT_OUTPUT) -> dict[str, Any]:
    freeze = verify_freeze()
    output_root = output_root.resolve()
    reject_forbidden(output_root)
    if output_root.exists() and any(output_root.iterdir()):
        raise fail(f"refusing to overwrite nonempty output: {output_root}")
    input_reports = {route: verify_inputs(route) for route in ROUTES}
    output_root.mkdir(parents=True, exist_ok=True)
    routes: dict[str, Any] = {}
    started = 0
    try:
        for route in ROUTES:
            control = run_case(output_root, route, "control", 1, False)
            started += 1
            control_reference = phase43_control(route)
            if control["submission"]["sha256"] != control_reference["submission"]["sha256"] or control["summary"]["sha256"] != control_reference["summary"]["sha256"]:
                raise fail(f"Phase43 control identity failed: {route}")
            candidate_one = run_case(output_root, route, "candidate", 1, True)
            started += 1
            candidate_two = run_case(output_root, route, "candidate", 2, True)
            started += 1
            if candidate_one["submission"]["sha256"] != candidate_two["submission"]["sha256"] or candidate_one["summary"]["sha256"] != candidate_two["summary"]["sha256"] or candidate_one["base"] != candidate_two["base"]:
                raise fail(f"candidate repeat identity failed: {route}")
            routes[route] = {"input": input_reports[route], "control": control, "phase43_control_reference": control_reference, "candidate_run1": candidate_one, "candidate_run2": candidate_two, "gates": {"base_hash_and_bytes_exact": True, "base_coordinate_finite_exact": True, "observed_dt_exact": True, "finite_correction_fraction": True, "materiality": True, "candidate_repeat_identity": True, "control_phase43_identity": True, "finite_outputs": True, "converged": True, "tdcp_built_equals_inserted": True, "over_70_mps": True}}
        result = {"schema_version": "smartphone-r5-phase65-native-base-pseudorange-compensation-structural-result.v1", "phase": 65, "execution_label": "Luna Max", "status": "go-native-base-pseudorange-compensation-structural", "truth_free": True, "freeze": {"path": relative(FREEZE), "sha256": FREEZE_SHA256}, "manifest": {"path": relative(MANIFEST), "sha256": sha256(MANIFEST)}, "source_commit": "f8cdf3b", "candidate": {"flag": CANDIDATE_FLAG, "base_path_flag": BASE_FLAG, "base_sha_flag": BASE_SHA_FLAG, "formula": "P_rover_corrected_m=P_rover_raw_m-pc(t); pc=movmean(base P+satellite_clock-ionosphere-troposphere-group_delay-range)", "scope": "adopted undifferenced FGO pseudorange factors only", "coefficient": 1.0, "spp_applied": False, "tdcp_applied": False, "doppler_applied": False, "no_extrapolation_or_endpoint_hold": True}, "routes": routes, "gates": {"all_four_routes": True, "base_hash_and_bytes_exact": True, "finite_correction_fraction_per_route": True, "adopted_pseudorange_coverage": True, "correction_materiality_each_route": True, "candidate_repeat_identity": True, "control_phase43_identity": True, "finite_output_and_earth_valid": True, "converged": True, "tdcp_built_equals_inserted": True, "over_70_mps": True, "truth_free": True, "all_passed": True}, "read_accounting": {"single_process_per_case": True, "routes": 4, "candidate_runs_per_route": 2, "control_runs_per_route": 1, "native_solver_invocations": 12, "raw_device_gnss_process_reads": 12, "raw_device_imu_process_reads": 12, "broadcast_nav_process_reads": 12, "base_rinex_process_reads": 12, "hash_verification_reads": {"device_gnss": 4, "device_imu": 4, "brdc.nav": 4, "base_rinex": 4}, "truth_reads": 0, "mat_reads_or_generated": 0, "validation_holdout_reads": 0, "kaggle_token_reads": 0, "archive_reopens": 0, "post_truth_tuning": False}, "phase43_champion_preserved": True, "phase51_experimental_preserved": True, "phase58_experimental_preserved": True, "zero_point_782_claim": "not evaluated in truth-free structural phase"}
        result_path = output_root / "phase65_native_base_pseudorange_compensation_structural_result.json"
        atomic_json(result_path, result)
        manifest = {"schema_version": "smartphone-r5-phase65-native-base-pseudorange-compensation-structural-manifest-output.v1", "phase": 65, "status": "sealed-truth-free-structural-matrix", "freeze": {"path": relative(FREEZE), "sha256": FREEZE_SHA256}, "evaluator": {"path": relative(Path(__file__)), "sha256": sha256(Path(__file__))}, "result": {"path": relative(result_path), "sha256": sha256(result_path), "bytes": result_path.stat().st_size}, "native_solver_invocations": 12, "truth_reads": 0, "all_gates_passed": True}
        atomic_json(output_root / "phase65_native_base_pseudorange_compensation_structural_manifest.json", manifest)
        return result
    except Phase65Error as exc:
        atomic_json(output_root / "phase65_native_base_pseudorange_compensation_structural_failure.json", {"schema_version": "smartphone-r5-phase65-native-base-pseudorange-compensation-structural-failure.v1", "status": "fail-closed", "error": str(exc), "truth_reads": 0, "native_solver_invocations_started": started, "partial_routes": routes})
        raise


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--verify-freeze", action="store_true")
    parser.add_argument("--run-matrix", action="store_true")
    parser.add_argument("--output-root", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args(argv)
    try:
        if args.verify_freeze:
            verify_freeze()
        if args.run_matrix:
            result = run_matrix(args.output_root)
            print(json.dumps({"status": result["status"], "all_gates_passed": result["gates"]["all_passed"], "native_solver_invocations": result["read_accounting"]["native_solver_invocations"], "truth_reads": result["read_accounting"]["truth_reads"]}, sort_keys=True))
        elif not args.verify_freeze:
            parser.error("one of --verify-freeze or --run-matrix is required")
        return 0
    except Phase65Error as exc:
        print(f"phase65 structural failure: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
