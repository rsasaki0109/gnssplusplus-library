#!/usr/bin/env python3
"""Truth-free per-epoch Doppler-WLS initialization candidate.

This is a development-only wrapper around ``gnss_fgo``.  It consumes only the
already materialized, hash-authorized development routes from a new freeze
record and enables the opt-in corrected-Doppler WLS initializer.  It never
opens ground truth, validation, holdout, test data, or Kaggle credentials.
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


FREEZE_SCHEMA = "smartphone-r5-gsdc2023-native-fgo-pdc-wls-freeze.v1"
RUN_SCHEMA = "smartphone-r5-gsdc2023-native-fgo-pdc-wls-run.v1"
DEFAULT_FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_gsdc2023_native_fgo_pdc_wls_freeze_v1.json"
DEFAULT_OUTPUT_ROOT = ROOT / "output/smartphone-r5/native-fgo-pdc-wls-v1"
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


class WlsError(ValueError):
    """Raised when the immutable candidate contract is not satisfied."""


def _resolve(path: str | Path) -> Path:
    candidate = Path(path)
    return candidate if candidate.is_absolute() else ROOT / candidate


def _sha256(path: Path) -> str:
    if not path.is_file():
        raise WlsError(f"missing file: {path}")
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _load_json(path: Path, label: str) -> dict[str, Any]:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise WlsError(f"invalid {label}: {path}") from exc
    if not isinstance(payload, dict):
        raise WlsError(f"{label} must be a JSON object: {path}")
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
    _atomic_bytes(path, (json.dumps(payload, indent=2, sort_keys=True) + "\n").encode("utf-8"))


def _safe_id(dataset_id: str) -> str:
    return dataset_id.replace("/", "__")


def _load_freeze(path: Path) -> dict[str, Any]:
    payload = _load_json(path, "WLS freeze record")
    if payload.get("schema_version") != FREEZE_SCHEMA:
        raise WlsError("WLS freeze schema mismatch")
    if payload.get("status") != "frozen-before-truth-free-wls-smoke":
        raise WlsError("WLS freeze status mismatch")
    policy = payload.get("policy")
    if not isinstance(policy, dict):
        raise WlsError("WLS freeze policy missing")
    for key, expected in (
        ("local_only", True),
        ("external_mutation", False),
        ("token_access", False),
        ("truth_opened_before_freeze", 0),
        ("validation_or_holdout_materialized", False),
        ("test_data_used", False),
        ("production_default_changed", False),
        ("no_post_truth_tuning", True),
    ):
        if policy.get(key) is not expected:
            raise WlsError(f"WLS freeze policy mismatch: {key}")
    split = payload.get("frozen_split")
    if not isinstance(split, dict) or tuple(split.get("train", ())) != TRAIN_IDS:
        raise WlsError("WLS train split mismatch")
    if split.get("fresh_validation") == split.get("future_holdout"):
        raise WlsError("WLS validation/holdout identities overlap")
    inputs = payload.get("authoritative_inputs")
    if not isinstance(inputs, dict):
        raise WlsError("WLS authoritative inputs missing")
    for key in ("archive", "central_inventory", "profile", "truth_use_inventory"):
        raw_path = inputs.get(key)
        hash_key = {
            "archive": "archive_sha256",
            "central_inventory": "central_inventory_sha256",
            "profile": "profile_sha256",
            "truth_use_inventory": "truth_use_inventory_sha256",
        }[key]
        if not isinstance(raw_path, str) or not isinstance(inputs.get(hash_key), str):
            raise WlsError(f"WLS authoritative input missing: {key}")
        if _sha256(_resolve(raw_path)) != inputs[hash_key]:
            raise WlsError(f"WLS authoritative hash mismatch: {key}")
    artifacts = split.get("train_artifacts")
    if not isinstance(artifacts, dict):
        raise WlsError("WLS train artifacts missing")
    for dataset_id in TRAIN_IDS:
        entry = artifacts.get(dataset_id)
        if not isinstance(entry, dict):
            raise WlsError(f"WLS artifact map missing: {dataset_id}")
        route_dir = _resolve(split["train_artifact_root"]) / str(entry["route_directory"])
        if not route_dir.is_dir():
            raise WlsError(f"missing WLS route directory: {route_dir}")
        if _sha256(route_dir / "route_manifest.json") != entry.get("route_manifest_sha256"):
            raise WlsError(f"WLS route manifest hash mismatch: {dataset_id}")
        for name, relative in (
            ("rover_obs_sha256", "adapter/rover.obs"),
            ("device_gnss_sha256", "inputs/device_gnss.csv"),
            ("brdc_nav_sha256", "inputs/brdc.nav"),
        ):
            if _sha256(route_dir / relative) != entry.get(name):
                raise WlsError(f"WLS input hash mismatch: {dataset_id}/{relative}")
        if (route_dir / "inputs" / "ground_truth.csv").exists() or (route_dir / "ground_truth.csv").exists():
            raise WlsError(f"truth materialized in WLS route: {dataset_id}")
    source_hashes = payload.get("source_hashes_at_freeze")
    if not isinstance(source_hashes, dict):
        raise WlsError("WLS source hashes missing")
    for name, path_value in {
        "apps/native/gnss_fgo.cpp": ROOT / "apps/native/gnss_fgo.cpp",
        "include/libgnss++/algorithms/fgo.hpp": ROOT / "include/libgnss++/algorithms/fgo.hpp",
        "include/libgnss++/algorithms/fgo_config.hpp": ROOT / "include/libgnss++/algorithms/fgo_config.hpp",
        "include/libgnss++/algorithms/doppler_contract.hpp": ROOT / "include/libgnss++/algorithms/doppler_contract.hpp",
        "include/libgnss++/algorithms/doppler_velocity_wls.hpp": ROOT / "include/libgnss++/algorithms/doppler_velocity_wls.hpp",
        "src/algorithms/fgo.cpp": ROOT / "src/algorithms/fgo.cpp",
        "src/algorithms/fgo_problems.cpp": ROOT / "src/algorithms/fgo_problems.cpp",
        "apps/commands/benchmarks/gnss_smartphone_native_fgo_pdc_wls.py": Path(__file__).resolve(),
        "build/apps/gnss_fgo": _resolve(payload.get("binary", "build/apps/gnss_fgo")),
    }.items():
        if _sha256(path_value) != source_hashes.get(name):
            raise WlsError(f"WLS source hash mismatch: {name}")
    return payload


def _route_inputs(freeze: dict[str, Any], dataset_id: str) -> dict[str, Any]:
    if dataset_id not in TRAIN_IDS:
        raise WlsError("only frozen WLS train identities are accepted")
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
        raise WlsError(f"WLS route input hash mismatch: {route['dataset_id']}")
    return observed


def _command(binary: Path, route: dict[str, Any], temporary: Path, max_epochs: int) -> list[str]:
    if max_epochs < 0:
        raise WlsError("max_epochs must be non-negative")
    return [
        str(binary), "--obs", str(route["obs"]), "--nav", str(route["nav"]),
        "--out", str(temporary / "fgo.pos"),
        "--summary-json", str(temporary / "fgo_summary.json"),
        "--epoch-debug-csv", str(temporary / "fgo_epoch_debug.csv"),
        "--factor-debug-csv", str(temporary / "fgo_factor_debug.csv"),
        "--cost-trace-csv", str(temporary / "fgo_cost_trace.csv"),
        "--preset", "default", "--backend", "eigen", "--skip-epochs", "0",
        "--max-epochs", str(max_epochs), "--max-iterations", "8",
        "--relative-cost-threshold", "0", "--absolute-cost-threshold", "0",
        "--pseudorange-sigma", "3", "--pseudorange-elevation-power", "1",
        "--motion-sigma", "50", "--clock-motion-sigma", "300",
        "--velocity-prior-sigma", "100", "--velocity-motion-sigma", "0.01",
        "--position-prior-sigma", "0", "--clock-prior-sigma", "0",
        "--tdcp-sigma", "0.03", "--carrier-phase-sigma", "0.01",
        "--undifferenced-doppler-sigma", "0.2",
        "--pseudorange-huber-threshold", "4", "--carrier-phase-huber-threshold", "4",
        "--tdcp-huber-threshold", "4", "--max-tdcp-gap", "2",
        "--seed-match-tolerance", "0.01", "--seed-interpolation-max-gap", "0",
        "--tdcp-slip-threshold", "10", "--min-elevation", "10", "--min-snr", "0",
        "--min-satellites-per-epoch", "4", "--carrier-phase-factors",
        "--undifferenced-doppler-factors", "--corrected-undifferenced-doppler-factors",
        "--doppler-velocity-wls-initialization", "--velocity-states",
        "--velocity-motion-factors", "--ambiguity-between-factors", "--no-dd-factors",
        "--ionosphere-model", "--troposphere-model", "--quiet",
    ]


def _child_limits() -> None:
    resource.setrlimit(resource.RLIMIT_AS, (MAX_ADDRESS_SPACE_BYTES, MAX_ADDRESS_SPACE_BYTES))
    resource.setrlimit(resource.RLIMIT_CPU, (MAX_RUNTIME_SECONDS, MAX_RUNTIME_SECONDS + 1))


def _validate_summary(summary: dict[str, Any]) -> dict[str, Any]:
    numeric = (
        "input_epochs", "optimized_epochs", "valid_solutions", "pseudorange_factors",
        "undifferenced_doppler_factors", "tdcp_factors", "carrier_phase_factors",
        "ambiguity_states", "motion_factors", "graph_factors", "graph_values",
        "initial_cost", "final_cost", "doppler_velocity_wls_valid_epochs",
        "doppler_velocity_wls_propagated_epochs", "doppler_velocity_wls_rejected_epochs",
        "doppler_velocity_wls_max_condition_number", "doppler_velocity_wls_max_normalized_rms",
        "doppler_velocity_wls_max_velocity_norm_mps", "doppler_velocity_wls_max_clock_rate_abs_mps",
    )
    for key in numeric:
        value = summary.get(key)
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(float(value)) or float(value) < 0:
            raise WlsError(f"invalid WLS summary field: {key}")
    if summary.get("backend") != "eigen" or summary.get("preset") != "default":
        raise WlsError("WLS backend/preset mismatch")
    if summary.get("use_undifferenced_doppler_factors") is not True or summary.get("use_corrected_undifferenced_doppler_factors") is not True:
        raise WlsError("WLS corrected Doppler was not enabled")
    if summary.get("use_doppler_velocity_wls_initialization") is not True:
        raise WlsError("WLS initializer was not enabled")
    if summary.get("use_velocity_states") is not True or summary.get("use_velocity_motion_factors") is not True:
        raise WlsError("WLS velocity graph was not enabled")
    if summary.get("single_difference_doppler_factors") != 0 or summary.get("single_difference_tdcp_factors") != 0:
        raise WlsError("forbidden base-dependent factor path enabled")
    if summary["doppler_velocity_wls_rejected_epochs"] != 0 or summary["doppler_velocity_wls_valid_epochs"] != summary["optimized_epochs"]:
        raise WlsError("WLS did not produce a valid estimate for every optimized epoch")
    if summary["doppler_velocity_wls_max_velocity_norm_mps"] > 70.0 or summary["doppler_velocity_wls_max_normalized_rms"] > 4.0:
        raise WlsError("WLS physical or residual gate failed")
    if not math.isfinite(float(summary["initial_cost"])) or not math.isfinite(float(summary["final_cost"])):
        raise WlsError("WLS cost is non-finite")
    return summary


def _read_keyed_device_epochs(path: Path) -> list[int]:
    return kaggle._read_device_epochs(path, 0)


def _position_rows(path: Path) -> list[Any]:
    try:
        return kaggle._read_positions(path, LEAP_SECONDS)
    except Exception as exc:  # noqa: BLE001
        raise WlsError(f"invalid WLS POS output: {path}") from exc


def _keyed_csv(dataset_id: str, positions: list[Any], device_epochs: list[int]) -> tuple[bytes, dict[str, int]]:
    allowed = set(device_epochs)
    seen: set[int] = set()
    rows: list[tuple[int, float, float]] = []
    for row in positions:
        timestamp = int(row.timestamp)
        if timestamp not in allowed:
            raise WlsError(f"WLS output timestamp is not a device key: {timestamp}")
        if timestamp in seen:
            raise WlsError(f"duplicate WLS output timestamp: {timestamp}")
        latitude = float(row.latitude)
        longitude = float(row.longitude)
        if not math.isfinite(latitude) or not math.isfinite(longitude) or not -90.0 <= latitude <= 90.0 or not -180.0 <= longitude <= 180.0:
            raise WlsError("WLS output coordinate is not finite/in range")
        seen.add(timestamp)
        rows.append((timestamp, latitude, longitude))
    rows.sort()
    output = [",".join(SUBMISSION_FIELDS)]
    for timestamp, latitude, longitude in rows:
        output.append(f"{dataset_id},{timestamp},{latitude:.12f},{longitude:.12f}")
    output.append("")
    return "\n".join(output).encode("ascii"), {
        "device_epochs": len(device_epochs), "output_rows": len(rows),
        "missing_device_epochs": len(set(device_epochs) - seen),
        "duplicate_output_epochs": len(rows) - len(seen), "extra_output_epochs": 0,
    }


def _artifact(path: Path) -> dict[str, Any]:
    if not path.is_file():
        raise WlsError(f"missing WLS artifact: {path}")
    size = path.stat().st_size
    if size > MAX_ARTIFACT_BYTES:
        raise WlsError(f"WLS artifact exceeds limit: {path}")
    return {"path": path.name, "bytes": size, "sha256": _sha256(path)}


def _seal_manifest(directory: Path, payload: dict[str, Any]) -> tuple[Path, str]:
    path = directory / "run_manifest.json"
    _atomic_json(path, payload)
    digest = _sha256(path)
    _atomic_bytes(directory / "run_manifest.sha256", f"{digest}  run_manifest.json\n".encode("ascii"))
    return path, digest


def run_route(freeze_path: Path, dataset_id: str, output_root: Path, binary: Path, max_epochs: int) -> dict[str, Any]:
    started = time.perf_counter()
    freeze = _load_freeze(freeze_path)
    route = _route_inputs(freeze, dataset_id)
    input_hashes = _verify_route_inputs(route)
    binary = binary.resolve()
    if not binary.is_file():
        raise WlsError(f"missing WLS Release binary: {binary}")
    source_hashes = dict(freeze["source_hashes_at_freeze"])
    tag = "all" if max_epochs == 0 else str(max_epochs)
    target = output_root / _safe_id(dataset_id) / f"epochs-{tag}"
    if target.exists():
        manifest_path = target / "run_manifest.json"
        if not manifest_path.is_file():
            raise WlsError(f"WLS existing target is unsealed: {target}")
        return _load_json(manifest_path, "WLS run manifest")
    target.parent.mkdir(parents=True, exist_ok=True)
    temporary = Path(tempfile.mkdtemp(prefix=f".{target.name}.", dir=str(target.parent)))
    command = _command(binary, route, temporary, max_epochs)
    environment = os.environ.copy()
    environment["LD_LIBRARY_PATH"] = "/home/sasaki/.local/lib" + os.pathsep + environment.get("LD_LIBRARY_PATH", "")
    return_code: int | None = None
    timed_out = False
    try:
        with (temporary / "stdout.log").open("wb") as stdout, (temporary / "stderr.log").open("wb") as stderr:
            try:
                completed = subprocess.run(command, cwd=ROOT, env=environment, stdin=subprocess.DEVNULL,
                                           stdout=stdout, stderr=stderr, timeout=MAX_RUNTIME_SECONDS + 5,
                                           preexec_fn=_child_limits, check=False)
                return_code = completed.returncode
            except subprocess.TimeoutExpired:
                timed_out = True
        runtime = time.perf_counter() - started
        rss_kib = resource.getrusage(resource.RUSAGE_CHILDREN).ru_maxrss
        if timed_out or return_code != 0:
            raise WlsError(f"WLS native command failed (return_code={return_code}, timed_out={timed_out})")
        summary = _validate_summary(_load_json(temporary / "fgo_summary.json", "WLS FGO summary"))
        positions = _position_rows(temporary / "fgo.pos")
        if len(positions) != int(summary["valid_solutions"]):
            raise WlsError("WLS summary/POS count mismatch")
        device_epochs = _read_keyed_device_epochs(route["device_gnss"])
        if max_epochs:
            device_epochs = device_epochs[:max_epochs]
        keyed, key_stats = _keyed_csv(dataset_id, positions, device_epochs)
        _atomic_bytes(temporary / "submission.csv", keyed)
        artifacts = [_artifact(temporary / name) for name in (
            "submission.csv", "fgo.pos", "fgo_summary.json", "fgo_epoch_debug.csv",
            "fgo_factor_debug.csv", "fgo_cost_trace.csv", "stdout.log", "stderr.log")]
        manifest = {
            "schema_version": RUN_SCHEMA, "status": "truth-free-artifacts-sealed",
            "candidate_id": "native-fgo-pdc-doppler-velocity-wls-v1",
            "lane": "native-fgo-pdc-doppler-velocity-wls", "dataset_id": dataset_id,
            "role": "development-train", "truth_opened": False,
            "freeze_record": str(freeze_path.relative_to(ROOT)),
            "freeze_record_sha256": _sha256(freeze_path), "input_hashes": input_hashes,
            "source_hashes": source_hashes, "command": command,
            "factor_coverage": {key: summary.get(key) for key in (
                "input_epochs", "optimized_epochs", "valid_solutions", "pseudorange_factors",
                "undifferenced_doppler_factors", "tdcp_factors", "carrier_phase_factors",
                "ambiguity_states", "motion_factors", "graph_factors", "graph_values",
                "iterations", "converged", "initial_cost", "final_cost",
                "doppler_velocity_wls_valid_epochs", "doppler_velocity_wls_propagated_epochs",
                "doppler_velocity_wls_rejected_epochs", "doppler_velocity_wls_max_condition_number",
                "doppler_velocity_wls_max_normalized_rms", "doppler_velocity_wls_max_velocity_norm_mps",
                "doppler_velocity_wls_max_clock_rate_abs_mps")},
            "key_coverage": key_stats,
            "runtime": {"wall_seconds": runtime, "child_max_rss_kib": rss_kib,
                         "timeout_seconds": MAX_RUNTIME_SECONDS,
                         "address_space_limit_bytes": MAX_ADDRESS_SPACE_BYTES},
            "artifacts": artifacts, "atomic_publish": True,
            "truth_free_contract": "No truth, validation, holdout, test, leaderboard, token, or external mutation.",
        }
        _seal_manifest(temporary, manifest)
        os.replace(temporary, target)
        manifest["published_path"] = str(target.relative_to(ROOT))
        return manifest
    except Exception:
        if temporary.exists():
            failure = {"schema_version": "smartphone-r5-gsdc2023-native-fgo-pdc-wls-failure.v1",
                       "status": "failed-before-publish", "dataset_id": dataset_id,
                       "truth_opened": False, "return_code": return_code, "timed_out": timed_out,
                       "wall_seconds": time.perf_counter() - started,
                       "input_hashes": input_hashes, "source_hashes": source_hashes,
                       "freeze_record_sha256": _sha256(freeze_path),
                       "reason": "WLS truth-free structural gate failed; no candidate artifact published"}
            _atomic_json(temporary / "failure_manifest.json", failure)
            os.replace(temporary, target.with_name(target.name + ".failure"))
        raise


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--freeze-record", type=Path, default=DEFAULT_FREEZE)
    parser.add_argument("--route", required=True, choices=TRAIN_IDS)
    parser.add_argument("--output-root", type=Path, default=DEFAULT_OUTPUT_ROOT)
    parser.add_argument("--binary", type=Path, default=DEFAULT_BINARY)
    parser.add_argument("--max-epochs", type=int, default=30)
    args = parser.parse_args(argv)
    try:
        manifest = run_route(_resolve(args.freeze_record), args.route, _resolve(args.output_root),
                             _resolve(args.binary), args.max_epochs)
    except WlsError as exc:
        print(f"WLS error: {exc}", file=sys.stderr)
        return 2
    print(json.dumps(manifest, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
