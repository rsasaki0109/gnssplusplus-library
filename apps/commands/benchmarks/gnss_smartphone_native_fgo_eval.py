#!/usr/bin/env python3
"""Run and score the frozen native Android-RINEX FGO candidate.

The ``run`` mode is deliberately truth-free.  It verifies the frozen source
and input hashes, runs the existing ``gnss_fgo`` Eigen executable with
pseudorange, ordinary TDCP, and motion factors, validates the factor summary,
and atomically publishes a content-addressed route artifact.  The ``train-score``
mode verifies every route artifact before opening any of the three train truth
files.  A failed FGO run is retained as a failure artifact and never silently
falls back to WLS or changes a production default.
"""

from __future__ import annotations

import argparse
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
import gnss_smartphone_reacquisition_eval as route_metrics  # noqa: E402
import gnss_smartphone_trajectory_smoother as smoother  # noqa: E402
import gnss_smartphone_trajectory_smoother_eval as smoother_eval  # noqa: E402


SELECTION_SCHEMA = "smartphone-r5-gsdc2023-native-fgo-selection.v1"
RUN_SCHEMA = "smartphone-r5-gsdc2023-native-fgo-run-manifest.v1"
TRAIN_REPORT_SCHEMA = "smartphone-r5-gsdc2023-native-fgo-train-evaluation.v1"
DEFAULT_SELECTION = (
    ROOT
    / "docs"
    / "use_cases"
    / "records"
    / "smartphone_r5_gsdc2023_native_fgo_selection.json"
)
DEFAULT_BINARY = ROOT / "build" / "apps" / "gnss_fgo"
DEFAULT_OUTPUT_ROOT = ROOT / "output" / "smartphone-r5" / "native-fgo-v1"
TRAIN_IDS = (
    "2021-01-04-21-50-us-ca-e1highway280driveroutea/mi8",
    "2022-08-04-20-07-us-ca-sjc-q/pixel5",
    "2023-05-24-20-26-us-ca-sjc-ge2/pixel7pro",
)
FRESH_VALIDATION_ID = "2023-09-07-18-59-us-ca/pixel5"
FUTURE_HOLDOUT_ID = "2023-09-06-00-01-us-ca-routen/pixel6pro"
LEAP_SECONDS = 18
SKIP_EPOCHS = 1
MATCH_TOLERANCE_MS = 100
MAX_RUNTIME_SECONDS = 900
MAX_ADDRESS_SPACE_BYTES = 8 * 1024 * 1024 * 1024
MAX_PUBLISHED_OUTPUT_BYTES = 1024 * 1024 * 1024
TOLERANCE = 1e-12
DIAGNOSTIC_KEYS = tuple(route_metrics._DIAGNOSTIC_KEYS)


class NativeFgoError(ValueError):
    """Raised when the frozen native-FGO contract cannot be satisfied."""


def _sha256(path: Path) -> str:
    if not path.is_file():
        raise NativeFgoError(f"missing file: {path}")
    digest = hashlib.sha256()
    try:
        with path.open("rb") as handle:
            for chunk in iter(lambda: handle.read(1024 * 1024), b""):
                digest.update(chunk)
    except OSError as exc:
        raise NativeFgoError(f"failed to hash {path}: {exc}") from exc
    return digest.hexdigest()


def _resolve(path: str | Path) -> Path:
    candidate = Path(path)
    return candidate if candidate.is_absolute() else ROOT / candidate


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


def _load_json(path: Path, label: str) -> dict[str, Any]:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise NativeFgoError(f"invalid {label}: {path}") from exc
    if not isinstance(payload, dict):
        raise NativeFgoError(f"{label} must be a JSON object: {path}")
    return payload


def _load_selection(path: Path) -> dict[str, Any]:
    payload = _load_json(path, "native FGO selection record")
    if payload.get("schema_version") != SELECTION_SCHEMA:
        raise NativeFgoError("native FGO selection schema mismatch")
    if payload.get("status") != "selection-frozen-before-truth":
        raise NativeFgoError("native FGO selection is not frozen before truth")
    split = payload.get("frozen_split")
    if not isinstance(split, dict):
        raise NativeFgoError("native FGO selection has no frozen split")
    if tuple(split.get("train", ())) != TRAIN_IDS:
        raise NativeFgoError("native FGO train split mismatch")
    if split.get("fresh_validation") != FRESH_VALIDATION_ID:
        raise NativeFgoError("native FGO fresh-validation split mismatch")
    if split.get("future_holdout") != FUTURE_HOLDOUT_ID:
        raise NativeFgoError("native FGO future-holdout split mismatch")
    if split.get("fresh_validation_truth_open_count_before_train_gate") != 0:
        raise NativeFgoError("fresh-validation truth was already opened")
    if split.get("future_holdout_truth_open_count_before_train_gate") != 0:
        raise NativeFgoError("future-holdout truth was already opened")
    policy = payload.get("policy")
    if not isinstance(policy, dict) or any(
        policy.get(key) is not expected
        for key, expected in (
            ("truth_free_generation", True),
            ("leaderboard_scores_used_for_tuning", False),
            ("kaggle_or_external_mutation", False),
            ("token_access", False),
            ("validation_or_holdout_before_train_gate", False),
            ("production_default_changed", False),
            ("old_holdout_results_used_for_selection", False),
        )
    ):
        raise NativeFgoError("native FGO selection policy is not closed")
    recipe = payload.get("recipe")
    if not isinstance(recipe, dict):
        raise NativeFgoError("native FGO selection has no recipe")
    expected = {
        "backend": "eigen",
        "preset": "default",
        "max_epochs": 0,
        "skip_epochs": 0,
        "max_iterations": 8,
        "relative_cost_convergence_threshold": 0.0,
        "absolute_cost_convergence_threshold": 0.0,
        "pseudorange_sigma_m": 3.0,
        "pseudorange_elevation_sigma_power": 1.0,
        "motion_sigma_m": 50.0,
        "clock_motion_sigma_m": 300.0,
        "velocity_prior_sigma_mps": 100.0,
        "velocity_motion_sigma_m": 0.01,
        "position_prior_sigma_m": 0.0,
        "clock_prior_sigma_m": 0.0,
        "tdcp_sigma_m": 0.03,
        "carrier_phase_sigma_m": 0.01,
        "pseudorange_huber_threshold_sigma": 4.0,
        "carrier_phase_huber_threshold_sigma": 4.0,
        "tdcp_huber_threshold_sigma": 4.0,
        "max_tdcp_gap_s": 2.0,
        "seed_match_tolerance_s": 0.01,
        "seed_interpolation_max_gap_s": 0.0,
        "tdcp_slip_threshold_m": 10.0,
        "min_elevation_deg": 10.0,
        "min_snr_dbhz": 0.0,
        "min_satellites_per_epoch": 4,
        "use_spp_seed": True,
        "use_pseudorange_factors": True,
        "use_ordinary_tdcp_factors": True,
        "use_position_motion_factors": True,
        "use_clock_motion_factors": True,
        "use_robust_loss": True,
        "use_ionosphere_model": True,
        "use_troposphere_model": True,
        "use_carrier_phase_factors": False,
        "use_single_difference_doppler_factors": False,
        "use_single_difference_tdcp_factors": False,
        "use_velocity_states": False,
        "use_velocity_motion_factors": False,
        "use_double_difference_factors": False,
        "use_ambiguity_between_factors": False,
        "use_ambiguity_priors": True,
        "no_spp_seed": False,
        "no_motion_factors": False,
        "no_tdcp_factors": False,
        "no_pseudorange_factors": False,
        "no_tdcp_slip_reject": False,
        "reject_rover_carrier_lli": False,
        "exclude_glonass_qzss_sbas": False,
    }
    for key, value in expected.items():
        if recipe.get(key) != value:
            raise NativeFgoError(f"frozen recipe mismatch for {key}")
    source_hashes = payload.get("source_hashes")
    if not isinstance(source_hashes, dict):
        raise NativeFgoError("native FGO selection has no source hashes")
    for raw_path, expected_hash in source_hashes.items():
        path_value = _resolve(raw_path)
        if _sha256(path_value) != expected_hash:
            raise NativeFgoError(f"frozen source hash mismatch: {raw_path}")
    artifacts = payload.get("input_artifacts")
    if not isinstance(artifacts, list) or tuple(item.get("dataset_id") for item in artifacts) != TRAIN_IDS:
        raise NativeFgoError("native FGO input artifact order mismatch")
    return payload


def _selection_record_hash(path: Path) -> str:
    return _sha256(path)


def _route_argument(value: str) -> dict[str, str]:
    fields = value.split("|")
    if len(fields) != 6 or any(not field for field in fields):
        raise NativeFgoError(
            "--route must be dataset_id|obs|nav|seed_pos|device_gnss|truth"
        )
    dataset_id, obs, nav, seed_pos, device_gnss, truth = fields
    if dataset_id.count("/") != 1 or any(character.isspace() for character in dataset_id):
        raise NativeFgoError("route dataset_id must be route/phone without whitespace")
    return {
        "dataset_id": dataset_id,
        "obs": obs,
        "nav": nav,
        "seed_pos": seed_pos,
        "device_gnss": device_gnss,
        "truth": truth,
    }


def _frozen_input(selection: dict[str, Any], route: dict[str, str]) -> dict[str, Any]:
    entries = selection["input_artifacts"]
    expected = next(
        (item for item in entries if item.get("dataset_id") == route["dataset_id"]),
        None,
    )
    if expected is None or route != {
        "dataset_id": expected.get("dataset_id"),
        "obs": expected.get("obs"),
        "nav": expected.get("nav"),
        "seed_pos": expected.get("seed_pos"),
        "device_gnss": expected.get("device_gnss"),
        "truth": expected.get("truth"),
    }:
        raise NativeFgoError("route arguments do not match the frozen input map")
    return expected


def _verify_input_hashes(entry: dict[str, Any]) -> dict[str, str]:
    mapping = {
        "obs": "obs_sha256",
        "nav": "nav_sha256",
        "seed_pos": "seed_pos_sha256",
        "device_gnss": "device_gnss_sha256",
    }
    hashes: dict[str, str] = {}
    for name, hash_key in mapping.items():
        path = _resolve(str(entry[name]))
        observed = _sha256(path)
        if observed != entry.get(hash_key):
            raise NativeFgoError(f"frozen input hash mismatch: {name}")
        hashes[name] = observed
    return hashes


def _child_limits() -> None:
    resource.setrlimit(
        resource.RLIMIT_AS,
        (MAX_ADDRESS_SPACE_BYTES, MAX_ADDRESS_SPACE_BYTES),
    )
    resource.setrlimit(resource.RLIMIT_CPU, (MAX_RUNTIME_SECONDS, MAX_RUNTIME_SECONDS + 1))


def _command(binary: Path, entry: dict[str, Any], temp_dir: Path) -> list[str]:
    return [
        str(binary),
        "--obs",
        str(_resolve(entry["obs"])),
        "--nav",
        str(_resolve(entry["nav"])),
        "--seed-pos",
        str(_resolve(entry["seed_pos"])),
        "--out",
        str(temp_dir / "fgo.pos"),
        "--summary-json",
        str(temp_dir / "fgo_summary.json"),
        "--epoch-debug-csv",
        str(temp_dir / "fgo_epoch_debug.csv"),
        "--factor-debug-csv",
        str(temp_dir / "fgo_factor_debug.csv"),
        "--cost-trace-csv",
        str(temp_dir / "fgo_cost_trace.csv"),
        "--preset",
        "default",
        "--backend",
        "eigen",
        "--skip-epochs",
        "0",
        "--max-epochs",
        "0",
        "--max-iterations",
        "8",
        "--relative-cost-threshold",
        "0",
        "--absolute-cost-threshold",
        "0",
        "--pseudorange-sigma",
        "3",
        "--pseudorange-elevation-power",
        "1",
        "--motion-sigma",
        "50",
        "--clock-motion-sigma",
        "300",
        "--velocity-prior-sigma",
        "100",
        "--velocity-motion-sigma",
        "0.01",
        "--position-prior-sigma",
        "0",
        "--clock-prior-sigma",
        "0",
        "--tdcp-sigma",
        "0.03",
        "--carrier-phase-sigma",
        "0.01",
        "--pseudorange-huber-threshold",
        "4",
        "--carrier-phase-huber-threshold",
        "4",
        "--tdcp-huber-threshold",
        "4",
        "--max-tdcp-gap",
        "2",
        "--seed-match-tolerance",
        "0.01",
        "--seed-interpolation-max-gap",
        "0",
        "--tdcp-slip-threshold",
        "10",
        "--min-elevation",
        "10",
        "--min-snr",
        "0",
        "--min-satellites-per-epoch",
        "4",
        "--no-dd-factors",
        "--ionosphere-model",
        "--troposphere-model",
        "--quiet",
    ]


def _validate_summary(summary: dict[str, Any]) -> dict[str, Any]:
    required = (
        "backend",
        "preset",
        "input_epochs",
        "optimized_epochs",
        "valid_solutions",
        "pseudorange_factors",
        "tdcp_factors",
        "motion_factors",
        "single_difference_doppler_factors",
        "single_difference_tdcp_factors",
        "carrier_phase_factors",
        "double_difference_pseudorange_factors",
        "double_difference_carrier_factors",
    )
    missing = [key for key in required if key not in summary]
    if missing:
        raise NativeFgoError(f"FGO summary missing fields: {', '.join(missing)}")
    if summary["backend"] != "eigen" or summary["preset"] != "default":
        raise NativeFgoError("FGO summary backend/preset differs from frozen recipe")
    for key in required[2:]:
        value = summary[key]
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise NativeFgoError(f"FGO summary field is not numeric: {key}")
        if not math.isfinite(float(value)) or float(value) < 0.0:
            raise NativeFgoError(f"FGO summary field is invalid: {key}")
    if summary["input_epochs"] <= 0 or summary["optimized_epochs"] <= 0:
        raise NativeFgoError("FGO produced no epochs")
    if summary["valid_solutions"] <= 0:
        raise NativeFgoError("FGO produced no valid solutions")
    if summary["pseudorange_factors"] <= 0:
        raise NativeFgoError("FGO inserted no pseudorange factors")
    if summary["tdcp_factors"] <= 0:
        raise NativeFgoError("FGO inserted no ordinary TDCP factors")
    if summary["motion_factors"] <= 0:
        raise NativeFgoError("FGO inserted no motion factors")
    for key in (
        "single_difference_doppler_factors",
        "single_difference_tdcp_factors",
        "carrier_phase_factors",
        "double_difference_pseudorange_factors",
        "double_difference_carrier_factors",
    ):
        if summary[key] != 0:
            raise NativeFgoError(f"forbidden FGO factor class is nonzero: {key}")
    bool_expectations = {
        "use_spp_seed": True,
        "use_pseudorange_factors": True,
        "use_single_difference_doppler_factors": False,
        "use_single_difference_tdcp_factors": False,
        "use_velocity_states": False,
        "use_velocity_motion_factors": False,
        "use_ambiguity_between_factors": False,
        "use_position_motion_factors": True,
        "use_clock_motion_factors": True,
        "use_ionosphere_model": True,
        "use_troposphere_model": True,
    }
    for key, expected in bool_expectations.items():
        if key in summary and summary[key] is not expected:
            raise NativeFgoError(f"FGO summary recipe mismatch: {key}")
    numeric_expectations = {
        "max_iterations": 8,
        "pseudorange_sigma_m": 3.0,
        "pseudorange_elevation_sigma_power": 1.0,
        "motion_sigma_m": 50.0,
        "clock_motion_sigma_m": 300.0,
        "tdcp_sigma_m": 0.03,
        "pseudorange_huber_threshold_sigma": 4.0,
        "tdcp_huber_threshold_sigma": 4.0,
        "max_tdcp_gap_s": 2.0,
        "seed_match_tolerance_s": 0.01,
        "seed_interpolation_max_gap_s": 0.0,
        "tdcp_slip_threshold_m": 10.0,
        "min_elevation_deg": 10.0,
        "min_snr_dbhz": 0.0,
        "min_satellites_per_epoch": 4,
    }
    for key, expected in numeric_expectations.items():
        if key in summary and not math.isclose(float(summary[key]), expected, rel_tol=0.0, abs_tol=1e-12):
            raise NativeFgoError(f"FGO summary numeric recipe mismatch: {key}")
    return summary


def _validate_pos(path: Path, expected_valid: int) -> list[smoother.PositionRow]:
    try:
        rows = smoother._read_positions(path, LEAP_SECONDS)
    except Exception as exc:  # noqa: BLE001 - normalize parser failures
        raise NativeFgoError(f"invalid FGO POS output: {path}") from exc
    if not rows:
        raise NativeFgoError("FGO POS output is empty")
    if expected_valid != len(rows):
        raise NativeFgoError(
            f"FGO summary/output count mismatch: {expected_valid} != {len(rows)}"
        )
    for row in rows:
        if not all(math.isfinite(float(value)) for value in (*row.ecef, row.latitude, row.longitude, row.height)):
            raise NativeFgoError("FGO POS contains non-finite coordinates")
        if not -90.0 <= row.latitude <= 90.0 or not -180.0 <= row.longitude <= 180.0:
            raise NativeFgoError("FGO POS coordinate is out of range")
    return rows


def _artifact(path: Path) -> dict[str, Any]:
    if not path.is_file():
        raise NativeFgoError(f"missing FGO artifact: {path.name}")
    size = path.stat().st_size
    if size > MAX_PUBLISHED_OUTPUT_BYTES:
        raise NativeFgoError(f"FGO artifact exceeds size limit: {path.name}")
    return {"path": path.name, "bytes": size, "sha256": _sha256(path)}


def _write_manifest_seal(directory: Path, manifest: dict[str, Any]) -> tuple[Path, str]:
    manifest_path = directory / "run_manifest.json"
    _atomic_json(manifest_path, manifest)
    digest = _sha256(manifest_path)
    _atomic_bytes(directory / "run_manifest.sha256", f"{digest}  run_manifest.json\n".encode("ascii"))
    return manifest_path, digest


def _failure_target(output_root: Path, key: str) -> Path:
    return output_root / f"{key}.failure"


def _publish_failure(temp_dir: Path, output_root: Path, key: str, payload: dict[str, Any]) -> Path:
    _atomic_json(temp_dir / "failure.json", payload)
    target = _failure_target(output_root, key)
    if target.exists():
        suffix = 1
        while (output_root / f"{key}.failure-{suffix}").exists():
            suffix += 1
        target = output_root / f"{key}.failure-{suffix}"
    os.replace(temp_dir, target)
    return target


def run_route(
    selection_path: Path,
    route: dict[str, str],
    output_root: Path,
    binary: Path,
    *,
    role: str = "train",
    selection_override: dict[str, Any] | None = None,
    entry_override: dict[str, Any] | None = None,
) -> dict[str, Any]:
    started = time.perf_counter()
    selection = selection_override if selection_override is not None else _load_selection(selection_path)
    if role == "train":
        if route["dataset_id"] not in TRAIN_IDS:
            raise NativeFgoError("run mode accepts only the frozen train routes")
        entry = _frozen_input(selection, route)
    elif role == "fresh-validation":
        if route["dataset_id"] != FRESH_VALIDATION_ID or entry_override is None:
            raise NativeFgoError("validation run does not match its frozen route map")
        entry = entry_override
    else:
        raise NativeFgoError(f"unsupported native FGO run role: {role}")
    input_hashes = _verify_input_hashes(entry)
    binary = binary.resolve()
    source_hashes = selection["source_hashes"]
    binary_key = "build/apps/gnss_fgo"
    if source_hashes.get(binary_key) != _sha256(binary):
        raise NativeFgoError("gnss_fgo binary hash differs from frozen record")
    output_root.mkdir(parents=True, exist_ok=True)
    key = _safe_id(route["dataset_id"])
    target = output_root / key
    if target.exists():
        raise NativeFgoError(f"route artifact already exists; refusing overwrite: {target}")
    temp_dir = Path(tempfile.mkdtemp(prefix=f".{key}.", dir=str(output_root)))
    command = _command(binary, entry, temp_dir)
    environment = os.environ.copy()
    local_lib = "/home/sasaki/.local/lib"
    environment["LD_LIBRARY_PATH"] = (
        local_lib + os.pathsep + environment["LD_LIBRARY_PATH"]
        if environment.get("LD_LIBRARY_PATH")
        else local_lib
    )
    return_code: int | None = None
    timed_out = False
    try:
        with (temp_dir / "stdout.log").open("wb") as stdout, (temp_dir / "stderr.log").open("wb") as stderr:
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
        rss_kib = resource.getrusage(resource.RUSAGE_CHILDREN).ru_maxrss
        runtime = time.perf_counter() - started
        if timed_out or return_code != 0:
            failure = {
                "schema_version": "smartphone-r5-gsdc2023-native-fgo-failure.v1",
                "status": "failed-before-publish",
                "dataset_id": route["dataset_id"],
                "truth_opened": False,
                "return_code": return_code,
                "timed_out": timed_out,
                "wall_seconds": runtime,
                "child_max_rss_kib": rss_kib,
                "command": command,
                "input_hashes": input_hashes,
                "selection_record_sha256": _selection_record_hash(selection_path),
                "reason": "native gnss_fgo exited nonzero or exceeded the runtime bound",
            }
            published = _publish_failure(temp_dir, output_root, key, failure)
            raise NativeFgoError(f"native FGO failed; preserved failure artifact: {published}")
        summary_path = temp_dir / "fgo_summary.json"
        summary = _validate_summary(_load_json(summary_path, "FGO summary"))
        pos_path = temp_dir / "fgo.pos"
        _validate_pos(pos_path, int(summary["valid_solutions"]))
        required = (
            pos_path,
            summary_path,
            temp_dir / "fgo_epoch_debug.csv",
            temp_dir / "fgo_factor_debug.csv",
            temp_dir / "fgo_cost_trace.csv",
        )
        artifacts = [_artifact(path) for path in required]
        runtime = time.perf_counter() - started
        manifest = {
            "schema_version": RUN_SCHEMA,
            "status": "truth-free-artifacts-sealed",
            "candidate_id": selection["candidate_id"],
            "dataset_id": route["dataset_id"],
            "role": role,
            "truth_opened": False,
            "selection_record": str(selection_path.relative_to(ROOT)),
            "selection_record_sha256": _selection_record_hash(selection_path),
            "binary": {"path": str(binary.relative_to(ROOT)), "sha256": _sha256(binary)},
            "input_hashes": input_hashes,
            "recipe": selection["recipe"],
            "command": command,
            "runtime": {
                "wall_seconds": runtime,
                "child_max_rss_kib": rss_kib,
                "timeout_seconds": MAX_RUNTIME_SECONDS,
                "address_space_limit_bytes": MAX_ADDRESS_SPACE_BYTES,
            },
            "factor_coverage": {
                key: summary[key]
                for key in (
                    "input_epochs",
                    "optimized_epochs",
                    "valid_solutions",
                    "pseudorange_factors",
                    "tdcp_factors",
                    "tdcp_factors_inserted",
                    "motion_factors",
                    "single_difference_doppler_factors",
                    "single_difference_tdcp_factors",
                    "carrier_phase_factors",
                    "double_difference_pseudorange_factors",
                    "double_difference_carrier_factors",
                )
            },
            "artifacts": artifacts,
            "atomic_publish": True,
            "fallback": "invalid FGO is failure-only; no silent WLS substitution",
        }
        _write_manifest_seal(temp_dir, manifest)
        os.replace(temp_dir, target)
        manifest["published_path"] = str(target.relative_to(ROOT))
        return manifest
    except NativeFgoError:
        if temp_dir.exists():
            # Input validation or a pre-publish parser error has no candidate;
            # retain it so the failure is independently inspectable.
            failure = {
                "schema_version": "smartphone-r5-gsdc2023-native-fgo-failure.v1",
                "status": "failed-before-publish",
                "dataset_id": route["dataset_id"],
                "truth_opened": False,
                "return_code": return_code,
                "timed_out": timed_out,
                "wall_seconds": time.perf_counter() - started,
                "command": command,
                "input_hashes": input_hashes,
                "selection_record_sha256": _selection_record_hash(selection_path),
                "reason": "validation or native FGO failure; candidate not published",
            }
            _publish_failure(temp_dir, output_root, key, failure)
        raise
    except Exception as exc:  # noqa: BLE001 - preserve every operational failure
        if temp_dir.exists():
            _publish_failure(
                temp_dir,
                output_root,
                key,
                {
                    "schema_version": "smartphone-r5-gsdc2023-native-fgo-failure.v1",
                    "status": "failed-before-publish",
                    "dataset_id": route["dataset_id"],
                    "truth_opened": False,
                    "return_code": return_code,
                    "timed_out": timed_out,
                    "wall_seconds": time.perf_counter() - started,
                    "selection_record_sha256": _selection_record_hash(selection_path),
                    "reason": f"unexpected operational failure: {type(exc).__name__}",
                },
            )
        raise NativeFgoError("native FGO operational failure; failure artifact preserved") from exc


def _load_validation_record(
    path: Path, *, allow_truth_materialized: bool = False
) -> tuple[dict[str, Any], dict[str, Any], dict[str, Any]]:
    payload = _load_json(path, "native FGO validation record")
    if payload.get("schema_version") != "smartphone-r5-gsdc2023-native-fgo-validation.v1":
        raise NativeFgoError("native FGO validation schema mismatch")
    if payload.get("status") != "validation-frozen-after-train-pass-before-payload-and-truth":
        raise NativeFgoError("native FGO validation record is not frozen")
    if payload.get("candidate_id") != "native_fgo_pseudorange_tdcp_motion_v1":
        raise NativeFgoError("native FGO validation candidate mismatch")
    base_path = _resolve(str(payload.get("selection_record", "")))
    if payload.get("selection_record_sha256") != _sha256(base_path):
        raise NativeFgoError("native FGO validation base-selection hash mismatch")
    selection = _load_selection(base_path)
    train_report = _resolve(str(payload.get("train_evaluation", "")))
    if payload.get("train_evaluation_sha256") != _sha256(train_report):
        raise NativeFgoError("native FGO validation train-report hash mismatch")
    train_payload = _load_json(train_report, "native FGO train evaluation")
    if train_payload.get("status") != "train-pass":
        raise NativeFgoError("native FGO validation is not authorized by a train pass")
    policy = payload.get("policy")
    if not isinstance(policy, dict) or policy.get("no_holdout_access") is not True:
        raise NativeFgoError("native FGO validation policy is not holdout-closed")
    frozen = payload.get("frozen_validation")
    inputs = payload.get("truth_free_inputs")
    if not isinstance(frozen, dict) or frozen.get("dataset_id") != FRESH_VALIDATION_ID:
        raise NativeFgoError("native FGO validation route mismatch")
    if not isinstance(inputs, dict):
        raise NativeFgoError("native FGO validation lacks truth-free input map")
    entry: dict[str, Any] = {"dataset_id": FRESH_VALIDATION_ID}
    for key in ("obs", "nav", "seed_pos", "device_gnss"):
        item = inputs.get(key)
        if not isinstance(item, dict) or not isinstance(item.get("path"), str):
            raise NativeFgoError(f"native FGO validation input map lacks {key}")
        entry[key] = item["path"]
        entry[f"{key}_sha256"] = item.get("sha256")
    truth_item = inputs.get("truth")
    if not isinstance(truth_item, dict) or not isinstance(truth_item.get("path"), str):
        raise NativeFgoError("native FGO validation truth map is invalid")
    entry["truth"] = truth_item["path"]
    if truth_item.get("opened") is not False:
        raise NativeFgoError("native FGO validation truth is already marked open")
    truth_path = _resolve(str(entry["truth"]))
    if not allow_truth_materialized and truth_path.exists():
        raise NativeFgoError("validation truth was materialized before truth-free FGO run")
    source_hashes = payload.get("source_hashes")
    if not isinstance(source_hashes, dict):
        raise NativeFgoError("native FGO validation source hashes are missing")
    for raw_path, expected_hash in source_hashes.items():
        if _sha256(_resolve(raw_path)) != expected_hash:
            raise NativeFgoError(f"native FGO validation source hash mismatch: {raw_path}")
    _verify_input_hashes(entry)
    return payload, selection, entry


def _verify_sealed_route(
    selection_path: Path,
    selection: dict[str, Any],
    dataset_id: str,
    output_root: Path,
    *,
    entry_override: dict[str, Any] | None = None,
    role: str = "train",
) -> tuple[dict[str, Any], dict[str, Any]]:
    entry = (
        entry_override
        if entry_override is not None
        else next(item for item in selection["input_artifacts"] if item["dataset_id"] == dataset_id)
    )
    _verify_input_hashes(entry)
    route_dir = output_root / _safe_id(dataset_id)
    manifest_path = route_dir / "run_manifest.json"
    seal_path = route_dir / "run_manifest.sha256"
    manifest = _load_json(manifest_path, "FGO run manifest")
    if manifest.get("schema_version") != RUN_SCHEMA or manifest.get("status") != "truth-free-artifacts-sealed":
        raise NativeFgoError(f"route artifact is not sealed: {dataset_id}")
    if (
        manifest.get("dataset_id") != dataset_id
        or manifest.get("truth_opened") is not False
        or manifest.get("role") != role
    ):
        raise NativeFgoError(f"route manifest identity/truth contract mismatch: {dataset_id}")
    try:
        seal_text = seal_path.read_text(encoding="ascii").strip().split()
    except OSError as exc:
        raise NativeFgoError(f"missing run manifest seal: {dataset_id}") from exc
    if len(seal_text) < 1 or seal_text[0] != _sha256(manifest_path):
        raise NativeFgoError(f"run manifest hash mismatch: {dataset_id}")
    if manifest.get("selection_record_sha256") != _selection_record_hash(selection_path):
        raise NativeFgoError(f"selection record hash mismatch: {dataset_id}")
    if manifest.get("input_hashes") != _verify_input_hashes(entry):
        raise NativeFgoError(f"input hash map mismatch: {dataset_id}")
    artifacts = manifest.get("artifacts")
    required_names = {
        "fgo.pos",
        "fgo_summary.json",
        "fgo_epoch_debug.csv",
        "fgo_factor_debug.csv",
        "fgo_cost_trace.csv",
    }
    if not isinstance(artifacts, list) or {
        item.get("path") for item in artifacts if isinstance(item, dict)
    } != required_names:
        raise NativeFgoError(f"FGO artifact set mismatch: {dataset_id}")
    for artifact in artifacts:
        if not isinstance(artifact, dict) or artifact.get("path") in (None, "run_manifest.json", "run_manifest.sha256"):
            raise NativeFgoError(f"invalid FGO artifact entry: {dataset_id}")
        path = route_dir / str(artifact["path"])
        if _sha256(path) != artifact.get("sha256"):
            raise NativeFgoError(f"FGO artifact hash mismatch: {dataset_id}/{artifact.get('path')}")
    return manifest, entry


def _raw_rows(positions: list[smoother.PositionRow]) -> list[smoother.SmoothedRow]:
    return route_metrics._raw_rows(positions)


def _normalize_score_schema(metrics: dict[str, Any]) -> dict[str, Any]:
    """Normalize optional diagnostic values before route aggregation.

    The scorer intentionally returns ``None`` when a sealed route has no
    truth-matched output rows.  Older ad-hoc aggregation code assumed every
    percentile was present and could therefore attempt ``None + None``.  Keep
    the schema explicit here: all four declared variants are present, while
    missing/non-finite values remain null for the existing fail-closed
    ``_metric`` policy in ``route_metrics._aggregate``.
    """

    if not isinstance(metrics, dict):
        raise NativeFgoError("score result must be a JSON object")
    normalized = dict(metrics)
    variants = metrics.get("kaggle_diagnostic_score_variants_m")
    if not isinstance(variants, dict):
        variants = {}
    normalized["kaggle_diagnostic_score_variants_m"] = {
        key: variants.get(key) for key in DIAGNOSTIC_KEYS
    }
    if metrics.get("kaggle_diagnostic_mean_m") is None:
        finite = [
            float(value)
            for value in normalized["kaggle_diagnostic_score_variants_m"].values()
            if isinstance(value, (int, float))
            and not isinstance(value, bool)
            and math.isfinite(float(value))
        ]
        normalized["kaggle_diagnostic_mean_m"] = (
            sum(finite) / len(finite) if len(finite) == len(DIAGNOSTIC_KEYS) else None
        )
    return normalized


def _score_position(
    path: Path,
    device_path: Path,
    truth: dict[int, tuple[float, float, float]],
) -> dict[str, Any]:
    positions = smoother._read_positions(path, LEAP_SECONDS)
    epochs = smoother._read_device_epochs(device_path, SKIP_EPOCHS)
    unknown = set(row.timestamp_ms for row in positions) - set(epochs)
    if unknown:
        raise NativeFgoError(f"position output has non-device timestamps: {path}")
    metrics = smoother_eval._score_rows(
        _raw_rows(positions),
        {row.timestamp_ms: row for row in positions},
        truth,
        0,
        len(epochs),
        match_tolerance_ms=MATCH_TOLERANCE_MS,
    )
    return _normalize_score_schema(metrics)


def _metric(metrics: dict[str, Any], path: tuple[str, ...]) -> float:
    value: Any = metrics
    for key in path:
        value = value[key]
    if value is None or not math.isfinite(float(value)):
        return math.inf
    return float(value)


def _non_regression(candidate: dict[str, Any], baseline: dict[str, Any]) -> tuple[bool, list[str]]:
    failures: list[str] = []
    if _metric(candidate, ("availability_ratio",)) < _metric(baseline, ("availability_ratio",)) - TOLERANCE:
        failures.append("availability_regression")
    if _metric(candidate, ("truth_coverage_ratio",)) < _metric(baseline, ("truth_coverage_ratio",)) - TOLERANCE:
        failures.append("truth_coverage_regression")
    for path, label in (
        (("horizontal_wgs84_m", "p50_m"), "h_p50_regression"),
        (("horizontal_wgs84_m", "p95_m"), "h_p95_regression"),
        (("vertical_p95_abs_m",), "v_p95_regression"),
    ):
        if _metric(candidate, path) > _metric(baseline, path) + TOLERANCE:
            failures.append(label)
    for key in DIAGNOSTIC_KEYS:
        if _metric(candidate, ("kaggle_diagnostic_score_variants_m", key)) > _metric(
            baseline, ("kaggle_diagnostic_score_variants_m", key)
        ) + TOLERANCE:
            failures.append(f"{key}_regression")
    return not failures, failures


def _aggregate(metrics: list[dict[str, Any]]) -> dict[str, Any]:
    return route_metrics._aggregate(metrics)


def _aggregate_gate(candidate: dict[str, Any], baseline: dict[str, Any]) -> dict[str, Any]:
    passed, failures = _non_regression(
        {
            "availability_ratio": candidate["mean_availability_ratio"],
            "truth_coverage_ratio": candidate["mean_truth_coverage_ratio"],
            "horizontal_wgs84_m": {"p50_m": candidate["mean_horizontal_wgs84_p50_m"], "p95_m": candidate["mean_horizontal_wgs84_p95_m"]},
            "vertical_p95_abs_m": candidate["mean_vertical_p95_abs_m"],
            "kaggle_diagnostic_score_variants_m": candidate["mean_kaggle_diagnostic_score_variants_m"],
        },
        {
            "availability_ratio": baseline["mean_availability_ratio"],
            "truth_coverage_ratio": baseline["mean_truth_coverage_ratio"],
            "horizontal_wgs84_m": {"p50_m": baseline["mean_horizontal_wgs84_p50_m"], "p95_m": baseline["mean_horizontal_wgs84_p95_m"]},
            "vertical_p95_abs_m": baseline["mean_vertical_p95_abs_m"],
            "kaggle_diagnostic_score_variants_m": baseline["mean_kaggle_diagnostic_score_variants_m"],
        },
    )
    strict = {
        "aggregate_h_p95_improvement": candidate["mean_horizontal_wgs84_p95_m"] < baseline["mean_horizontal_wgs84_p95_m"] - TOLERANCE,
        "aggregate_diagnostic_mean_improvement": candidate["mean_kaggle_diagnostic_m"] < baseline["mean_kaggle_diagnostic_m"] - TOLERANCE,
    }
    strict_failures = [name for name, value in strict.items() if not value]
    return {
        "non_regression_passed": passed,
        "non_regression_failures": failures,
        "strict": strict,
        "strict_failures": strict_failures,
        "passed": passed and not strict_failures,
    }


def train_score(selection_path: Path, output_root: Path) -> dict[str, Any]:
    started = time.perf_counter()
    selection = _load_selection(selection_path)
    sealed: dict[str, tuple[dict[str, Any], dict[str, Any]]] = {}
    # This loop performs no truth path stat/read.  Every route is sealed before
    # the first call to _read_truth below.
    for dataset_id in TRAIN_IDS:
        sealed[dataset_id] = _verify_sealed_route(selection_path, selection, dataset_id, output_root)
    route_reports: dict[str, Any] = {}
    baseline_metrics: list[dict[str, Any]] = []
    candidate_metrics: list[dict[str, Any]] = []
    truth_open_count = 0
    for dataset_id in TRAIN_IDS:
        manifest, entry = sealed[dataset_id]
        device = _resolve(entry["device_gnss"])
        # Open each train truth file exactly once, after every route manifest
        # and every truth-free input hash has been verified above.
        truth_path = _resolve(entry["truth"])
        truth = smoother_eval._read_truth(truth_path)
        truth_open_count += 1
        baseline = _score_position(_resolve(entry["seed_pos"]), device, truth)
        candidate = _score_position(
            output_root / _safe_id(dataset_id) / "fgo.pos", device, truth
        )
        baseline_metrics.append(baseline)
        candidate_metrics.append(candidate)
        route_passed, route_failures = _non_regression(candidate, baseline)
        route_reports[dataset_id] = {
            "baseline_wls": baseline,
            "candidate_native_fgo": candidate,
            "gate": {"non_regression_passed": route_passed, "failures": route_failures},
            "run_manifest_sha256": _sha256(output_root / _safe_id(dataset_id) / "run_manifest.json"),
            "truth_opened": True,
        }
    baseline_aggregate = _aggregate(baseline_metrics)
    candidate_aggregate = _aggregate(candidate_metrics)
    aggregate_gate = _aggregate_gate(candidate_aggregate, baseline_aggregate)
    route_gate_passed = all(item["gate"]["non_regression_passed"] for item in route_reports.values())
    train_passed = route_gate_passed and aggregate_gate["passed"]
    report = {
        "schema_version": TRAIN_REPORT_SCHEMA,
        "status": "train-pass" if train_passed else "no-go-train-gate",
        "candidate_id": selection["candidate_id"],
        "truth_free_artifacts_sealed_before_truth": True,
        "truth_open_count": truth_open_count,
        "fresh_validation_truth_open_count": 0,
        "future_holdout_truth_open_count": 0,
        "routes": route_reports,
        "aggregate": {
            "baseline_wls": baseline_aggregate,
            "candidate_native_fgo": candidate_aggregate,
            "gate": aggregate_gate,
        },
        "train_gate": {
            "route_level_non_regression_passed": route_gate_passed,
            "strict_aggregate_passed": aggregate_gate["passed"],
            "passed": train_passed,
        },
        "validation_policy": {
            "opened": False,
            "reason": "fresh validation remains sealed in this train-only command; it is permitted only when every train gate passes",
            "fresh_validation_id": FRESH_VALIDATION_ID,
            "future_holdout_id": FUTURE_HOLDOUT_ID,
        },
        "runtime": {"wall_seconds": time.perf_counter() - started},
        "external_mutation": False,
        "leaderboard_scores_used_for_tuning": False,
    }
    output_root.mkdir(parents=True, exist_ok=True)
    report_path = output_root / "train_evaluation.json"
    _atomic_json(report_path, report)
    manifest = {
        "schema_version": "smartphone-r5-gsdc2023-native-fgo-train-evaluation-manifest.v1",
        "status": report["status"],
        "report": {"path": report_path.name, "sha256": _sha256(report_path), "bytes": report_path.stat().st_size},
        "selection_record": {"path": str(selection_path.relative_to(ROOT)), "sha256": _sha256(selection_path)},
        "route_manifest_sha256": {dataset_id: route_reports[dataset_id]["run_manifest_sha256"] for dataset_id in TRAIN_IDS},
        "truth_open_count": truth_open_count,
        "fresh_validation_truth_open_count": 0,
        "future_holdout_truth_open_count": 0,
    }
    _atomic_json(output_root / "train_evaluation.manifest.json", manifest)
    report["report_path"] = str(report_path.relative_to(ROOT))
    report["report_sha256"] = _sha256(report_path)
    return report


def validation_score(validation_path: Path, output_root: Path) -> dict[str, Any]:
    """Score the one authorized fresh-validation route after its seal."""

    started = time.perf_counter()
    _payload, selection, entry = _load_validation_record(
        validation_path, allow_truth_materialized=True
    )
    manifest, _ = _verify_sealed_route(
        validation_path,
        selection,
        FRESH_VALIDATION_ID,
        output_root,
        entry_override=entry,
        role="fresh-validation",
    )
    truth_path = _resolve(entry["truth"])
    if not truth_path.is_file():
        raise NativeFgoError("fresh-validation truth was not materialized")
    # This is the only truth read in this mode and it happens after the
    # validation FGO manifest has been hash-verified.
    truth = smoother_eval._read_truth(truth_path)
    baseline = _score_position(_resolve(entry["seed_pos"]), _resolve(entry["device_gnss"]), truth)
    candidate = _score_position(
        output_root / _safe_id(FRESH_VALIDATION_ID) / "fgo.pos",
        _resolve(entry["device_gnss"]),
        truth,
    )
    non_regression, failures = _non_regression(candidate, baseline)
    strict = candidate["horizontal_wgs84_m"]["p95_m"] < baseline["horizontal_wgs84_m"]["p95_m"] - TOLERANCE
    report = {
        "schema_version": "smartphone-r5-gsdc2023-native-fgo-validation-evaluation.v1",
        "status": "validation-pass" if non_regression and strict else "no-go-validation-gate",
        "candidate_id": selection["candidate_id"],
        "dataset_id": FRESH_VALIDATION_ID,
        "truth_free_artifacts_sealed_before_truth": True,
        "truth_open_count": 1,
        "baseline_wls": baseline,
        "candidate_native_fgo": candidate,
        "gate": {
            "route_non_regression_passed": non_regression,
            "route_non_regression_failures": failures,
            "strict_h_p95_improvement": strict,
            "passed": non_regression and strict,
        },
        "truth": {"path": str(truth_path), "sha256": _sha256(truth_path)},
        "run_manifest_sha256": _sha256(
            output_root / _safe_id(FRESH_VALIDATION_ID) / "run_manifest.json"
        ),
        "future_holdout_truth_open_count": 0,
        "future_holdout_materialized": False,
        "runtime": {"wall_seconds": time.perf_counter() - started},
        "external_mutation": False,
        "leaderboard_scores_used_for_tuning": False,
    }
    output_root.mkdir(parents=True, exist_ok=True)
    report_path = output_root / "validation_evaluation.json"
    _atomic_json(report_path, report)
    manifest_payload = {
        "schema_version": "smartphone-r5-gsdc2023-native-fgo-validation-evaluation-manifest.v1",
        "status": report["status"],
        "report": {"path": report_path.name, "sha256": _sha256(report_path), "bytes": report_path.stat().st_size},
        "validation_record": {"path": str(validation_path.relative_to(ROOT)), "sha256": _sha256(validation_path)},
        "run_manifest_sha256": report["run_manifest_sha256"],
        "truth_open_count": 1,
        "future_holdout_truth_open_count": 0,
        "future_holdout_materialized": False,
    }
    _atomic_json(output_root / "validation_evaluation.manifest.json", manifest_payload)
    report["report_path"] = str(report_path.relative_to(ROOT))
    report["report_sha256"] = _sha256(report_path)
    return report


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(prog="gnss_smartphone_native_fgo_eval")
    subparsers = parser.add_subparsers(dest="mode", required=True)
    run_parser = subparsers.add_parser("run", help="truth-free run for one frozen train route")
    run_parser.add_argument("--selection-record", type=Path, default=DEFAULT_SELECTION)
    run_parser.add_argument("--route", required=True)
    run_parser.add_argument("--output-root", type=Path, default=DEFAULT_OUTPUT_ROOT)
    run_parser.add_argument("--binary", type=Path, default=DEFAULT_BINARY)
    score_parser = subparsers.add_parser("train-score", help="score sealed train artifacts once")
    score_parser.add_argument("--selection-record", type=Path, default=DEFAULT_SELECTION)
    score_parser.add_argument("--output-root", type=Path, default=DEFAULT_OUTPUT_ROOT)
    validation_parser = subparsers.add_parser(
        "validation-run", help="truth-free run for the authorized fresh-validation route"
    )
    validation_parser.add_argument(
        "--validation-record",
        type=Path,
        default=ROOT / "docs" / "use_cases" / "records" / "smartphone_r5_gsdc2023_native_fgo_validation.json",
    )
    validation_parser.add_argument(
        "--output-root",
        type=Path,
        default=ROOT / "output" / "smartphone-r5" / "native-fgo-v1" / "validation",
    )
    validation_parser.add_argument("--binary", type=Path, default=DEFAULT_BINARY)
    validation_score_parser = subparsers.add_parser(
        "validation-score", help="score the sealed fresh-validation route once"
    )
    validation_score_parser.add_argument(
        "--validation-record",
        type=Path,
        default=ROOT / "docs" / "use_cases" / "records" / "smartphone_r5_gsdc2023_native_fgo_validation.json",
    )
    validation_score_parser.add_argument(
        "--output-root",
        type=Path,
        default=ROOT / "output" / "smartphone-r5" / "native-fgo-v1" / "validation",
    )
    return parser.parse_args(argv)


def run(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    try:
        if args.mode == "run":
            selection_path = _resolve(args.selection_record)
            output_root = _resolve(args.output_root)
            manifest = run_route(
                selection_path,
                _route_argument(args.route),
                output_root,
                _resolve(args.binary),
            )
            print(json.dumps({"status": manifest["status"], "dataset_id": manifest["dataset_id"], "factor_coverage": manifest["factor_coverage"]}, sort_keys=True))
            return 0
        if args.mode == "train-score":
            selection_path = _resolve(args.selection_record)
            output_root = _resolve(args.output_root)
            report = train_score(selection_path, output_root)
            print(json.dumps({"status": report["status"], "train_gate": report["train_gate"], "truth_open_count": report["truth_open_count"]}, sort_keys=True))
            return 0
        if args.mode == "validation-run":
            validation_path = _resolve(args.validation_record)
            _validation_payload, selection, entry = _load_validation_record(validation_path)
            route = {
                "dataset_id": FRESH_VALIDATION_ID,
                "obs": entry["obs"],
                "nav": entry["nav"],
                "seed_pos": entry["seed_pos"],
                "device_gnss": entry["device_gnss"],
                "truth": entry["truth"],
            }
            manifest = run_route(
                validation_path,
                route,
                _resolve(args.output_root),
                _resolve(args.binary),
                role="fresh-validation",
                selection_override=selection,
                entry_override=entry,
            )
            print(json.dumps({"status": manifest["status"], "dataset_id": manifest["dataset_id"], "factor_coverage": manifest["factor_coverage"]}, sort_keys=True))
            return 0
        if args.mode == "validation-score":
            report = validation_score(_resolve(args.validation_record), _resolve(args.output_root))
            print(json.dumps({"status": report["status"], "gate": report["gate"], "truth_open_count": report["truth_open_count"]}, sort_keys=True))
            return 0
    except NativeFgoError as exc:
        print(f"native FGO evaluation failed: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(run())
