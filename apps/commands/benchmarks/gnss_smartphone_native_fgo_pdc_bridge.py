#!/usr/bin/env python3
"""Run the frozen no-base native P/D/C/TDCP FGO smartphone bridge.

This is intentionally a thin orchestrator around the existing ``gnss_fgo``
problem builder.  It does not read truth and it does not implement a second
trajectory estimator.  The only candidate-specific switch is the explicit
receiver-only Doppler factor; the existing native carrier, ordinary TDCP,
position/clock motion, velocity, and float ambiguity paths are reused.

The command accepts only identities and input paths already frozen in the
bridge record.  Every temporary output is published by rename, and an
existing content-addressed route directory is verified and reused.  A
fallback is never implicit: callers must provide a hash-authorized keyed
baseline CSV when they want the frozen native-FGO v5/v1 lane as a failure
path.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import resource
import subprocess
import sys
import tempfile
import time
from typing import Any


_COMMANDS_DIR = Path(__file__).resolve().parents[1]
if str(_COMMANDS_DIR) not in sys.path:
    sys.path.insert(0, str(_COMMANDS_DIR))
from support.gnss_runtime import application_root  # noqa: E402


ROOT = application_root(__file__)
BENCHMARK_DIR = ROOT / "apps" / "commands" / "benchmarks"
if str(BENCHMARK_DIR) not in sys.path:
    sys.path.insert(0, str(BENCHMARK_DIR))
import gnss_smartphone_kaggle as kaggle  # noqa: E402


FREEZE_SCHEMA = "smartphone-r5-gsdc2023-native-fgo-pdc-bridge-freeze.v1"
RUN_SCHEMA = "smartphone-r5-gsdc2023-native-fgo-pdc-bridge-run.v1"
DEFAULT_FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_gsdc2023_native_fgo_pdc_bridge_freeze_v1.json"
DEFAULT_OUTPUT_ROOT = ROOT / "output/smartphone-r5/native-fgo-pdc-bridge-v1"
DEFAULT_BINARY = ROOT / "build/apps/gnss_fgo"
TRAIN_IDS = (
    "2021-03-16-18-59-us-ca-mtv-a/pixel5",
    "2021-07-27-19-49-us-ca-mtv-b/pixel4",
    "2022-02-24-18-29-us-ca-lax-o/pixel5",
)
LEAP_SECONDS = 18
MAX_RUNTIME_SECONDS = 900
MAX_ADDRESS_SPACE_BYTES = 8 * 1024 * 1024 * 1024
MAX_ARTIFACT_BYTES = 1024 * 1024 * 1024
SUBMISSION_FIELDS = ("phone", "UnixTimeMillis", "LatitudeDegrees", "LongitudeDegrees")


class BridgeError(ValueError):
    """Raised when the frozen bridge contract cannot be satisfied."""


def _resolve(path: str | Path) -> Path:
    candidate = Path(path)
    return candidate if candidate.is_absolute() else ROOT / candidate


def _sha256(path: Path) -> str:
    if not path.is_file():
        raise BridgeError(f"missing file: {path}")
    digest = hashlib.sha256()
    try:
        with path.open("rb") as handle:
            for chunk in iter(lambda: handle.read(1024 * 1024), b""):
                digest.update(chunk)
    except OSError as exc:
        raise BridgeError(f"failed to hash {path}: {exc}") from exc
    return digest.hexdigest()


def _load_json(path: Path, label: str) -> dict[str, Any]:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise BridgeError(f"invalid {label}: {path}") from exc
    if not isinstance(payload, dict):
        raise BridgeError(f"{label} must be a JSON object: {path}")
    return payload


def _atomic_bytes(path: Path, content: bytes) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".tmp", dir=str(path.parent)
    )
    try:
        with os.fdopen(descriptor, "wb") as handle:
            handle.write(content)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def _atomic_json(path: Path, payload: dict[str, Any]) -> None:
    _atomic_bytes(
        path,
        (json.dumps(payload, indent=2, sort_keys=True) + "\n").encode("utf-8"),
    )


def _safe_id(dataset_id: str) -> str:
    return dataset_id.replace("/", "__")


def _load_freeze(
    path: Path,
    truth_use_inventory_hash_override: str | None = None,
) -> dict[str, Any]:
    payload = _load_json(path, "bridge freeze record")
    if payload.get("schema_version") != FREEZE_SCHEMA:
        raise BridgeError("bridge freeze schema mismatch")
    if payload.get("status") != "frozen-before-bridge-implementation":
        raise BridgeError("bridge freeze was not made before implementation")
    policy = payload.get("policy")
    if not isinstance(policy, dict):
        raise BridgeError("bridge freeze policy is missing")
    for key, expected in (
        ("local_only", True),
        ("external_mutation", False),
        ("token_access", False),
        ("leaderboard_used_for_tuning", False),
        ("truth_opened_before_freeze", 0),
        ("validation_or_holdout_materialized", False),
        ("test_data_used", False),
        ("production_default_changed", False),
        ("no_post_holdout_tuning", True),
    ):
        if policy.get(key) is not expected:
            raise BridgeError(f"bridge freeze policy mismatch: {key}")
    split = payload.get("frozen_split")
    if not isinstance(split, dict) or tuple(split.get("train", ())) != TRAIN_IDS:
        raise BridgeError("bridge train split mismatch")
    if split.get("fresh_validation") == split.get("future_holdout"):
        raise BridgeError("bridge validation/holdout identities overlap")
    inputs = payload.get("authoritative_inputs")
    if not isinstance(inputs, dict):
        raise BridgeError("bridge authoritative inputs are missing")
    for key in ("archive", "central_inventory", "profile", "truth_use_inventory"):
        raw_path = inputs.get(key)
        if not isinstance(raw_path, str):
            raise BridgeError(f"bridge input path missing: {key}")
        hash_key = {
            "archive": "archive_sha256",
            "central_inventory": "central_inventory_sha256",
            "profile": "profile_sha256",
            "truth_use_inventory": "truth_use_inventory_sha256",
        }[key]
        expected_hash = (
            truth_use_inventory_hash_override
            if key == "truth_use_inventory" and truth_use_inventory_hash_override is not None
            else inputs.get(hash_key)
        )
        if not isinstance(expected_hash, str) or _sha256(_resolve(raw_path)) != expected_hash:
            raise BridgeError(f"bridge authoritative hash mismatch: {key}")
    artifacts = split.get("train_artifacts")
    if not isinstance(artifacts, dict):
        raise BridgeError("bridge train artifacts are missing")
    for dataset_id in TRAIN_IDS:
        entry = artifacts.get(dataset_id)
        if not isinstance(entry, dict):
            raise BridgeError(f"bridge artifact map missing: {dataset_id}")
        route_dir = _resolve(split["train_artifact_root"]) / str(entry["route_directory"])
        if not route_dir.is_dir():
            raise BridgeError(f"missing materialized train route: {route_dir}")
        if _sha256(route_dir / "route_manifest.json") != entry.get("route_manifest_sha256"):
            raise BridgeError(f"route manifest hash mismatch: {dataset_id}")
        for name, relative in (
            ("rover_obs_sha256", "adapter/rover.obs"),
            ("device_gnss_sha256", "inputs/device_gnss.csv"),
            ("brdc_nav_sha256", "inputs/brdc.nav"),
        ):
            if _sha256(route_dir / relative) != entry.get(name):
                raise BridgeError(f"train input hash mismatch: {dataset_id}/{relative}")
        # A materialized route must not acquire a truth member through this
        # bridge.  Existence is sufficient; the truth file is never opened.
        if (route_dir / "inputs" / "ground_truth.csv").exists() or (route_dir / "ground_truth.csv").exists():
            raise BridgeError(f"truth materialized in bridge route directory: {dataset_id}")
    return payload


CORRECTED_FREEZE_SCHEMA = "smartphone-r5-gsdc2023-native-fgo-pdc-corrected-freeze.v1"


def _load_run_freeze(path: Path) -> dict[str, Any]:
    """Load either the historical bridge freeze or an authorized correction freeze.

    The correction record is intentionally a wrapper: the route/input split is
    still validated by the original freeze, while the wrapper pins the updated
    inventory and corrected source/binary hashes.  A malformed wrapper never
    relaxes the old fail-closed checks.
    """
    payload = _load_json(path, "run freeze record")
    if payload.get("schema_version") != CORRECTED_FREEZE_SCHEMA:
        return _load_freeze(path)
    if payload.get("status") != "frozen-before-corrected-train-artifacts":
        raise BridgeError("corrected freeze status mismatch")
    base = payload.get("base_freeze")
    auth = payload.get("correction_authorization")
    inventory = payload.get("authoritative_inventory")
    if not all(isinstance(value, dict) for value in (base, auth, inventory)):
        raise BridgeError("corrected freeze references are incomplete")
    base_path = _resolve(str(base.get("path")))
    auth_path = _resolve(str(auth.get("path")))
    inventory_path = _resolve(str(inventory.get("path")))
    if _sha256(base_path) != base.get("sha256"):
        raise BridgeError("corrected freeze base hash mismatch")
    if _sha256(auth_path) != auth.get("sha256"):
        raise BridgeError("corrected freeze authorization hash mismatch")
    inventory_hash = inventory.get("sha256")
    if not isinstance(inventory_hash, str) or _sha256(inventory_path) != inventory_hash:
        raise BridgeError("corrected freeze inventory hash mismatch")
    base_payload = _load_freeze(
        base_path, truth_use_inventory_hash_override=inventory_hash
    )
    split = payload.get("fixed_split")
    if not isinstance(split, dict) or tuple(split.get("train", ())) != TRAIN_IDS:
        raise BridgeError("corrected freeze train split mismatch")
    source_hashes = payload.get("source_hashes_at_freeze")
    if not isinstance(source_hashes, dict):
        raise BridgeError("corrected freeze source hashes are missing")
    expected_paths = {
        "apps/native/gnss_fgo.cpp": ROOT / "apps/native/gnss_fgo.cpp",
        "include/libgnss++/algorithms/fgo.hpp": ROOT / "include/libgnss++/algorithms/fgo.hpp",
        "include/libgnss++/algorithms/fgo_config.hpp": ROOT / "include/libgnss++/algorithms/fgo_config.hpp",
        "include/libgnss++/algorithms/doppler_contract.hpp": ROOT / "include/libgnss++/algorithms/doppler_contract.hpp",
        "src/algorithms/fgo.cpp": ROOT / "src/algorithms/fgo.cpp",
        "src/algorithms/fgo_problems.cpp": ROOT / "src/algorithms/fgo_problems.cpp",
        "src/algorithms/fgo_internal.hpp": ROOT / "src/algorithms/fgo_internal.hpp",
        "apps/commands/benchmarks/gnss_smartphone_native_fgo_pdc_bridge.py": Path(__file__).resolve(),
        "tests/test_fgo.cpp": ROOT / "tests/test_fgo.cpp",
        "build/apps/gnss_fgo": ROOT / "build/apps/gnss_fgo",
    }
    for name, source_path in expected_paths.items():
        if _sha256(source_path) != source_hashes.get(name):
            raise BridgeError(f"corrected freeze source hash mismatch: {name}")
    return base_payload


def _source_hashes(binary: Path, freeze_path: Path) -> dict[str, str]:
    paths = {
        "apps/native/gnss_fgo.cpp": ROOT / "apps/native/gnss_fgo.cpp",
        "include/libgnss++/algorithms/fgo.hpp": ROOT / "include/libgnss++/algorithms/fgo.hpp",
        "include/libgnss++/algorithms/fgo_config.hpp": ROOT / "include/libgnss++/algorithms/fgo_config.hpp",
        "src/algorithms/fgo.cpp": ROOT / "src/algorithms/fgo.cpp",
        "src/algorithms/fgo_problems.cpp": ROOT / "src/algorithms/fgo_problems.cpp",
        "bridge": Path(__file__).resolve(),
        "freeze_record": freeze_path,
        "build/apps/gnss_fgo": binary,
    }
    return {name: _sha256(path) for name, path in paths.items()}


def _route_inputs(freeze: dict[str, Any], dataset_id: str) -> dict[str, Any]:
    if dataset_id not in TRAIN_IDS:
        raise BridgeError("only frozen development train identities are accepted")
    entry = freeze["frozen_split"]["train_artifacts"][dataset_id]
    root = _resolve(freeze["frozen_split"]["train_artifact_root"]) / entry["route_directory"]
    return {
        "dataset_id": dataset_id,
        "route_directory": root,
        "obs": root / "adapter/rover.obs",
        "nav": root / "inputs/brdc.nav",
        "device_gnss": root / "inputs/device_gnss.csv",
        "expected_hashes": {
            "obs": entry["rover_obs_sha256"],
            "nav": entry["brdc_nav_sha256"],
            "device_gnss": entry["device_gnss_sha256"],
        },
    }


def _verify_route_inputs(route: dict[str, Any]) -> dict[str, str]:
    observed = {
        "obs": _sha256(route["obs"]),
        "nav": _sha256(route["nav"]),
        "device_gnss": _sha256(route["device_gnss"]),
    }
    if observed != route["expected_hashes"]:
        raise BridgeError(f"frozen route input hash mismatch: {route['dataset_id']}")
    return observed


def _command(
    binary: Path,
    route: dict[str, Any],
    temporary: Path,
    max_epochs: int,
    corrected_doppler: bool = False,
) -> list[str]:
    if max_epochs < 0:
        raise BridgeError("max_epochs must be non-negative")
    command = [
        str(binary),
        "--obs", str(route["obs"]),
        "--nav", str(route["nav"]),
        "--out", str(temporary / "fgo.pos"),
        "--summary-json", str(temporary / "fgo_summary.json"),
        "--epoch-debug-csv", str(temporary / "fgo_epoch_debug.csv"),
        "--factor-debug-csv", str(temporary / "fgo_factor_debug.csv"),
        "--cost-trace-csv", str(temporary / "fgo_cost_trace.csv"),
        "--preset", "default",
        "--backend", "eigen",
        "--skip-epochs", "0",
        "--max-epochs", str(max_epochs),
        "--max-iterations", "8",
        "--relative-cost-threshold", "0",
        "--absolute-cost-threshold", "0",
        "--pseudorange-sigma", "3",
        "--pseudorange-elevation-power", "1",
        "--motion-sigma", "50",
        "--clock-motion-sigma", "300",
        "--velocity-prior-sigma", "100",
        "--velocity-motion-sigma", "0.01",
        "--position-prior-sigma", "0",
        "--clock-prior-sigma", "0",
        "--tdcp-sigma", "0.03",
        "--carrier-phase-sigma", "0.01",
        "--undifferenced-doppler-sigma", "0.2",
        "--pseudorange-huber-threshold", "4",
        "--carrier-phase-huber-threshold", "4",
        "--tdcp-huber-threshold", "4",
        "--max-tdcp-gap", "2",
        "--seed-match-tolerance", "0.01",
        "--seed-interpolation-max-gap", "0",
        "--tdcp-slip-threshold", "10",
        "--min-elevation", "10",
        "--min-snr", "0",
        "--min-satellites-per-epoch", "4",
        "--carrier-phase-factors",
        "--undifferenced-doppler-factors",
        "--velocity-states",
        "--velocity-motion-factors",
        "--ambiguity-between-factors",
        "--no-dd-factors",
        "--ionosphere-model",
        "--troposphere-model",
        "--quiet",
    ]
    if corrected_doppler:
        command.insert(-1, "--corrected-undifferenced-doppler-factors")
    return command


def _child_limits() -> None:
    resource.setrlimit(resource.RLIMIT_AS, (MAX_ADDRESS_SPACE_BYTES, MAX_ADDRESS_SPACE_BYTES))
    resource.setrlimit(resource.RLIMIT_CPU, (MAX_RUNTIME_SECONDS, MAX_RUNTIME_SECONDS + 1))


def _validate_summary(
    summary: dict[str, Any], corrected_doppler: bool = False
) -> dict[str, Any]:
    numeric = (
        "input_epochs", "optimized_epochs", "valid_solutions",
        "pseudorange_factors", "undifferenced_doppler_factors",
        "tdcp_factors", "carrier_phase_factors", "ambiguity_states",
        "motion_factors", "graph_factors", "graph_values", "initial_cost",
        "final_cost",
    )
    for key in numeric:
        value = summary.get(key)
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(float(value)) or float(value) < 0:
            raise BridgeError(f"FGO summary field is invalid: {key}")
    if summary.get("backend") != "eigen" or summary.get("preset") != "default":
        raise BridgeError("FGO summary backend/preset mismatch")
    for key in ("input_epochs", "optimized_epochs", "valid_solutions", "pseudorange_factors", "undifferenced_doppler_factors", "tdcp_factors", "carrier_phase_factors", "ambiguity_states", "motion_factors"):
        if summary[key] <= 0:
            raise BridgeError(f"required factor/epoch count is zero: {key}")
    for key in ("single_difference_doppler_factors", "single_difference_tdcp_factors", "double_difference_pseudorange_factors", "double_difference_carrier_factors"):
        if summary.get(key) != 0:
            raise BridgeError(f"base-dependent factor unexpectedly nonzero: {key}")
    if summary.get("use_undifferenced_doppler_factors") is not True:
        raise BridgeError("summary did not enable undifferenced Doppler")
    if summary.get("use_corrected_undifferenced_doppler_factors") is not corrected_doppler:
        raise BridgeError("summary corrected-Doppler mode mismatch")
    if summary.get("use_single_difference_doppler_factors") is not False or summary.get("use_single_difference_tdcp_factors") is not False:
        raise BridgeError("summary enabled a forbidden single-difference path")
    if summary.get("use_velocity_states") is not True or summary.get("use_velocity_motion_factors") is not True:
        raise BridgeError("summary did not enable velocity graph")
    return summary


def _read_keyed_device_epochs(path: Path) -> list[int]:
    return kaggle._read_device_epochs(path, 0)


def _position_rows(path: Path) -> list[Any]:
    try:
        return kaggle._read_positions(path, LEAP_SECONDS)
    except Exception as exc:  # noqa: BLE001 - normalize parser failures
        raise BridgeError(f"invalid native POS output: {path}") from exc


def _keyed_csv(dataset_id: str, positions: list[Any], device_epochs: list[int]) -> tuple[bytes, dict[str, int]]:
    allowed = set(device_epochs)
    seen: set[int] = set()
    rows: list[tuple[int, float, float]] = []
    for row in positions:
        timestamp = int(row.timestamp)
        if timestamp not in allowed:
            raise BridgeError(f"native output timestamp is not a device key: {timestamp}")
        if timestamp in seen:
            raise BridgeError(f"duplicate native output timestamp: {timestamp}")
        seen.add(timestamp)
        latitude = float(row.latitude)
        longitude = float(row.longitude)
        if not math.isfinite(latitude) or not math.isfinite(longitude) or not -90.0 <= latitude <= 90.0 or not -180.0 <= longitude <= 180.0:
            raise BridgeError("native output coordinate is not finite/in range")
        rows.append((timestamp, latitude, longitude))
    rows.sort()
    output = [",".join(SUBMISSION_FIELDS)]
    for timestamp, latitude, longitude in rows:
        output.append(f"{dataset_id},{timestamp},{latitude:.12f},{longitude:.12f}")
    output.append("")
    return "\n".join(output).encode("ascii"), {
        "device_epochs": len(device_epochs),
        "output_rows": len(rows),
        "missing_device_epochs": len(set(device_epochs) - seen),
        "extra_output_epochs": 0,
        "duplicate_output_epochs": len(rows) - len(seen),
    }


def _artifact(path: Path) -> dict[str, Any]:
    if not path.is_file():
        raise BridgeError(f"missing artifact: {path}")
    size = path.stat().st_size
    if size > MAX_ARTIFACT_BYTES:
        raise BridgeError(f"artifact exceeds size limit: {path.name}")
    return {"path": path.name, "bytes": size, "sha256": _sha256(path)}


def _seal_manifest(directory: Path, payload: dict[str, Any]) -> tuple[Path, str]:
    path = directory / "run_manifest.json"
    _atomic_json(path, payload)
    digest = _sha256(path)
    _atomic_bytes(directory / "run_manifest.sha256", f"{digest}  run_manifest.json\n".encode("ascii"))
    return path, digest


def _verify_existing(target: Path) -> dict[str, Any]:
    manifest_path = target / "run_manifest.json"
    seal_path = target / "run_manifest.sha256"
    manifest = _load_json(manifest_path, "existing bridge manifest")
    if manifest.get("schema_version") != RUN_SCHEMA:
        raise BridgeError("existing bridge manifest schema mismatch")
    expected = seal_path.read_text(encoding="ascii").split()[0]
    if expected != _sha256(manifest_path):
        raise BridgeError("existing bridge manifest seal mismatch")
    for item in manifest.get("artifacts", []):
        path = target / item["path"]
        if _sha256(path) != item.get("sha256"):
            raise BridgeError(f"existing bridge artifact hash mismatch: {item['path']}")
    return manifest


def _publish_failure(temp: Path, target: Path, payload: dict[str, Any]) -> Path:
    _atomic_json(temp / "failure.json", payload)
    failure_target = target.with_name(target.name + ".failure")
    suffix = 1
    while failure_target.exists():
        failure_target = target.with_name(target.name + f".failure-{suffix}")
        suffix += 1
    os.replace(temp, failure_target)
    return failure_target


def _fallback_rows(path: Path, expected_sha256: str | None) -> bytes:
    observed = _sha256(path)
    if expected_sha256 is None or observed != expected_sha256:
        raise BridgeError("fallback is not hash-authorized")
    try:
        with path.open(encoding="utf-8-sig", newline="") as handle:
            reader = csv.DictReader(handle)
            if tuple(reader.fieldnames or ()) != SUBMISSION_FIELDS:
                raise BridgeError("fallback CSV header mismatch")
            seen: set[tuple[str, int]] = set()
            rows: list[str] = [",".join(SUBMISSION_FIELDS)]
            for number, row in enumerate(reader, start=2):
                key = (row.get("phone", ""), int(row.get("UnixTimeMillis", "-1")))
                if key in seen:
                    raise BridgeError(f"fallback duplicate key at row {number}")
                seen.add(key)
                latitude = float(row.get("LatitudeDegrees", "nan"))
                longitude = float(row.get("LongitudeDegrees", "nan"))
                if not math.isfinite(latitude) or not math.isfinite(longitude):
                    raise BridgeError("fallback has non-finite coordinate")
                rows.append(f"{row['phone']},{key[1]},{latitude:.12f},{longitude:.12f}")
            rows.append("")
            return "\n".join(rows).encode("ascii")
    except (OSError, ValueError) as exc:
        if isinstance(exc, BridgeError):
            raise
        raise BridgeError("invalid fallback CSV") from exc


def run_route(
    freeze_path: Path,
    dataset_id: str,
    output_root: Path,
    binary: Path,
    max_epochs: int,
    fallback_csv: Path | None = None,
    fallback_sha256: str | None = None,
    corrected_doppler: bool = False,
    candidate_id: str = "native-fgo-pdc-bridge-v1",
    manifest_freeze_path: Path | None = None,
) -> dict[str, Any]:
    started = time.perf_counter()
    freeze = _load_run_freeze(freeze_path)
    route = _route_inputs(freeze, dataset_id)
    input_hashes = _verify_route_inputs(route)
    binary = binary.resolve()
    if not binary.is_file():
        raise BridgeError(f"missing Release binary: {binary}")
    manifest_freeze_path = manifest_freeze_path or freeze_path
    source_hashes = _source_hashes(binary, manifest_freeze_path)
    tag = "all" if max_epochs == 0 else str(max_epochs)
    target = output_root / _safe_id(dataset_id) / f"epochs-{tag}"
    if target.exists():
        return _verify_existing(target)
    target.parent.mkdir(parents=True, exist_ok=True)
    temporary = Path(tempfile.mkdtemp(prefix=f".{target.name}.", dir=str(target.parent)))
    command = _command(binary, route, temporary, max_epochs, corrected_doppler)
    environment = os.environ.copy()
    local_lib = "/home/sasaki/.local/lib"
    environment["LD_LIBRARY_PATH"] = local_lib + os.pathsep + environment.get("LD_LIBRARY_PATH", "")
    return_code: int | None = None
    timed_out = False
    try:
        with (temporary / "stdout.log").open("wb") as stdout, (temporary / "stderr.log").open("wb") as stderr:
            try:
                completed = subprocess.run(
                    command,
                    cwd=ROOT,
                    env=environment,
                    stdin=subprocess.DEVNULL,
                    stdout=stdout,
                    stderr=stderr,
                    timeout=MAX_RUNTIME_SECONDS + 5,
                    preexec_fn=_child_limits,
                    check=False,
                )
                return_code = completed.returncode
            except subprocess.TimeoutExpired:
                timed_out = True
        runtime = time.perf_counter() - started
        rss_kib = resource.getrusage(resource.RUSAGE_CHILDREN).ru_maxrss
        native_ok = not timed_out and return_code == 0
        if native_ok:
            summary = _validate_summary(
                _load_json(temporary / "fgo_summary.json", "FGO summary"),
                corrected_doppler,
            )
            positions = _position_rows(temporary / "fgo.pos")
            if len(positions) != int(summary["valid_solutions"]):
                raise BridgeError("FGO summary/POS valid solution count mismatch")
            device_epochs = _read_keyed_device_epochs(route["device_gnss"])
            if max_epochs:
                device_epochs = device_epochs[:max_epochs]
            keyed, key_stats = _keyed_csv(dataset_id, positions, device_epochs)
            _atomic_bytes(temporary / "submission.csv", keyed)
            artifacts = [
                _artifact(temporary / name)
                for name in (
                    "submission.csv", "fgo.pos", "fgo_summary.json",
                    "fgo_epoch_debug.csv", "fgo_factor_debug.csv",
                    "fgo_cost_trace.csv", "stdout.log", "stderr.log",
                )
            ]
            manifest = {
                "schema_version": RUN_SCHEMA,
                "status": "truth-free-artifacts-sealed",
                "candidate_id": candidate_id,
                "lane": "native-fgo-pdc",
                "corrected_doppler_contract": corrected_doppler,
                "dataset_id": dataset_id,
                "role": "development-train",
                "truth_opened": False,
                "freeze_record": str(manifest_freeze_path.relative_to(ROOT)),
                "freeze_record_sha256": _sha256(manifest_freeze_path),
                "input_hashes": input_hashes,
                "source_hashes": source_hashes,
                "command": command,
                "factor_coverage": {key: summary.get(key) for key in (
                    "input_epochs", "optimized_epochs", "valid_solutions",
                    "pseudorange_factors", "undifferenced_doppler_factors",
                    "tdcp_factors", "carrier_phase_factors", "ambiguity_states",
                    "ambiguity_between_factors", "motion_factors", "graph_factors",
                    "graph_values", "iterations", "converged", "initial_cost", "final_cost",
                )},
                "key_coverage": key_stats,
                "runtime": {
                    "wall_seconds": runtime,
                    "child_max_rss_kib": rss_kib,
                    "timeout_seconds": MAX_RUNTIME_SECONDS,
                    "address_space_limit_bytes": MAX_ADDRESS_SPACE_BYTES,
                },
                "fallback": {"used": False, "authorized": fallback_csv is None},
                "artifacts": artifacts,
                "atomic_publish": True,
                "truth_free_contract": "No truth, leaderboard, token, or external mutation is used by this bridge.",
            }
            _seal_manifest(temporary, manifest)
            os.replace(temporary, target)
            manifest["published_path"] = str(target.relative_to(ROOT))
            return manifest

        if fallback_csv is None:
            failure = {
                "schema_version": "smartphone-r5-gsdc2023-native-fgo-pdc-bridge-failure.v1",
                "status": "failed-before-publish",
                "dataset_id": dataset_id,
                "truth_opened": False,
                "return_code": return_code,
                "timed_out": timed_out,
                "wall_seconds": runtime,
                "child_max_rss_kib": rss_kib,
                "command": command,
                "input_hashes": input_hashes,
                "source_hashes": source_hashes,
                "freeze_record_sha256": _sha256(manifest_freeze_path),
                "reason": "native FGO failed and no explicit hash-authorized fallback was supplied",
            }
            published = _publish_failure(temporary, target, failure)
            raise BridgeError(f"native FGO failed; preserved {published}")

        fallback = _fallback_rows(fallback_csv, fallback_sha256)
        _atomic_bytes(temporary / "submission.csv", fallback)
        artifacts = [_artifact(temporary / name) for name in ("submission.csv", "stdout.log", "stderr.log")]
        manifest = {
            "schema_version": RUN_SCHEMA,
            "status": "truth-free-artifacts-sealed",
                "candidate_id": candidate_id,
                "lane": "v5-native-fgo-fallback",
                "corrected_doppler_contract": corrected_doppler,
            "dataset_id": dataset_id,
            "role": "development-train",
            "truth_opened": False,
            "freeze_record": str(manifest_freeze_path.relative_to(ROOT)),
            "freeze_record_sha256": _sha256(manifest_freeze_path),
            "input_hashes": input_hashes,
            "source_hashes": source_hashes,
            "command": command,
            "runtime": {"wall_seconds": runtime, "child_max_rss_kib": rss_kib},
            "fallback": {"used": True, "path": str(fallback_csv), "sha256": _sha256(fallback_csv), "reason": "explicit native FGO failure"},
            "artifacts": artifacts,
            "atomic_publish": True,
        }
        _seal_manifest(temporary, manifest)
        os.replace(temporary, target)
        manifest["published_path"] = str(target.relative_to(ROOT))
        return manifest
    except BridgeError:
        if temporary.exists():
            _publish_failure(
                temporary,
                target,
                {
                    "schema_version": "smartphone-r5-gsdc2023-native-fgo-pdc-bridge-failure.v1",
                    "status": "failed-before-publish",
                    "dataset_id": dataset_id,
                    "truth_opened": False,
                    "return_code": return_code,
                    "timed_out": timed_out,
                    "wall_seconds": time.perf_counter() - started,
                    "input_hashes": input_hashes,
                    "source_hashes": source_hashes,
                    "freeze_record_sha256": _sha256(manifest_freeze_path),
                    "reason": "bridge validation failed; no candidate artifact was published",
                },
            )
        raise
    except Exception as exc:  # noqa: BLE001 - retain operational failures
        if temporary.exists():
            _publish_failure(
                temporary,
                target,
                {
                    "schema_version": "smartphone-r5-gsdc2023-native-fgo-pdc-bridge-failure.v1",
                    "status": "failed-before-publish",
                    "dataset_id": dataset_id,
                    "truth_opened": False,
                    "wall_seconds": time.perf_counter() - started,
                    "input_hashes": input_hashes,
                    "source_hashes": source_hashes,
                    "freeze_record_sha256": _sha256(manifest_freeze_path),
                    "reason": f"unexpected operational failure: {type(exc).__name__}",
                },
            )
        raise BridgeError("bridge operational failure; failure artifact preserved") from exc


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--freeze-record", type=Path, default=DEFAULT_FREEZE)
    parser.add_argument("--route", required=True, choices=TRAIN_IDS)
    parser.add_argument("--output-root", type=Path, default=DEFAULT_OUTPUT_ROOT)
    parser.add_argument("--binary", type=Path, default=DEFAULT_BINARY)
    parser.add_argument("--max-epochs", type=int, default=30)
    parser.add_argument("--fallback-csv", type=Path)
    parser.add_argument("--fallback-sha256")
    parser.add_argument(
        "--corrected-undifferenced-doppler",
        action="store_true",
        help="opt into the Android clock-drift/rotated-satellite Doppler contract",
    )
    parser.add_argument(
        "--manifest-freeze-record",
        type=Path,
        help="freeze record to pin in the published manifest (input freeze remains --freeze-record)",
    )
    parser.add_argument("--candidate-id", default="native-fgo-pdc-bridge-v1", help=argparse.SUPPRESS)
    args = parser.parse_args(argv)
    try:
        manifest = run_route(
            _resolve(args.freeze_record),
            args.route,
            _resolve(args.output_root),
            _resolve(args.binary),
            args.max_epochs,
            _resolve(args.fallback_csv) if args.fallback_csv else None,
            args.fallback_sha256,
            args.corrected_undifferenced_doppler,
            args.candidate_id,
            _resolve(args.manifest_freeze_record)
            if args.manifest_freeze_record
            else None,
        )
    except BridgeError as exc:
        print(f"bridge error: {exc}", file=sys.stderr)
        return 2
    print(json.dumps(manifest, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
