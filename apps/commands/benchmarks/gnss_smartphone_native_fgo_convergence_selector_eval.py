#!/usr/bin/env python3
"""Truth-free convergence selector for the frozen native smartphone FGO.

This research-only lane runs the exact baseline8 and candidate50 native-FGO
recipes from the optimizer-stop experiment.  It selects candidate50 only when
the solver reports a genuine early convergence, its finite weighted cost trace
is monotonically non-increasing, its output covers the device epochs exactly,
all required/forbidden factor classes satisfy the frozen contract, and no
transition exceeds the predeclared 70 m/s physical bound.  Otherwise it
publishes a byte-exact copy of baseline8.  No truth, device model, residual to
truth, or leaderboard value is read by the selector.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import sys
import tempfile
import time
from typing import Any


_COMMANDS_DIR = Path(__file__).resolve().parents[1]
if str(_COMMANDS_DIR) not in sys.path:
    sys.path.insert(0, str(_COMMANDS_DIR))
_BENCHMARK_DIR = Path(__file__).resolve().parent
if str(_BENCHMARK_DIR) not in sys.path:
    sys.path.insert(0, str(_BENCHMARK_DIR))

from support.gnss_runtime import application_root  # noqa: E402
import gnss_smartphone_native_fgo_optimizer_stop_eval as optimizer  # noqa: E402
import gnss_smartphone_trajectory_smoother as smoother  # noqa: E402
import gnss_smartphone_trajectory_smoother_eval as smoother_eval  # noqa: E402


ROOT = application_root(__file__)
FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_gsdc2023_native_fgo_convergence_selector_freeze_v1.json"
FREEZE_MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_gsdc2023_native_fgo_convergence_selector_freeze_v1_manifest.json"
ARCHIVE = ROOT / "data/gsdc2023/cache/dataset_2023.zip"
PROFILE = ROOT / "configs/benchmarks/smartphone_r5_gsdc2023.json"
FGO_BINARY = ROOT / "build/apps/gnss_fgo"
SPP_BINARY = ROOT / "build/apps/gnss_spp"
OUTPUT_ROOT = ROOT / "output/smartphone-r5/native-fgo-convergence-selector-v1"
TRAIN_IDS = (
    "2021-01-04-21-50-us-ca-e1highway280driveroutea/pixel5",
    "2021-07-14-20-50-us-ca-mtv-e/sm-g988b",
)
FRESH_VALIDATION_ID = "2022-08-04-20-07-us-ca-sjc-q/sm-a325f"
FUTURE_HOLDOUT_ID = "2023-03-08-21-34-us-ca-mtv-u/pixel5"
ARCHIVE_SHA256 = "bda30ab456e6fd6f83550c246e8dbd287306d5385f1f1069c99c16298e647408"
CENTRAL_INVENTORY_SHA256 = "f4e68109885eecfc14b2bd5e8fab87e18d73473c04bb7f53da31b7040a8e90a7"
CENTRAL_METADATA_SHA256 = "5f619be94a00c33c3067f13b1cd3351d96f2804f07fa04eda8fb027739fb0992"
PROFILE_SHA256 = "273dfcc4e4636940d5216cca793a55773d9a04f668d1bfa5fcdc0013f4768776"
FGO_BINARY_SHA256 = "9190455cb34a9e818b79ed2ed6b023af5711fbef545215d4588bfed87f5c03e4"
SPP_BINARY_SHA256 = "4d272940437c2ab2dcffef31b7541c8c01212e97d2d2320edcf7bd8f80ea3c12"
OPTIMIZER_SOURCE_SHA256 = "86e4f7db8a0b0e5bcf8b30eb02b5f1bce7bc9b86528065803b5c68f0f83b3729"
FREEZE_SHA256 = "dd94186ee2f1509d1c45bcae60de4c40d8cd51a8c5298d2dbb51b36e6cc7801a"
FREEZE_MANIFEST_SHA256 = "4211c038911be86d7711b828e4689e8089cccf7bce494161b170d6207f2feb1e"
FREEZE_SCHEMA = "smartphone-r5-gsdc2023-native-fgo-convergence-selector-freeze.v1"
FREEZE_MANIFEST_SCHEMA = "smartphone-r5-gsdc2023-native-fgo-convergence-selector-freeze-manifest.v1"
RUN_SCHEMA = "smartphone-r5-gsdc2023-native-fgo-convergence-selector-run.v1"
ROUTE_SCHEMA = optimizer.ROUTE_SCHEMA
TRAIN_SCHEMA = "smartphone-r5-gsdc2023-native-fgo-convergence-selector-train-evaluation.v1"
LEAP_SECONDS = optimizer.LEAP_SECONDS
SPEED_BOUND_MPS = 70.0
VERTICAL_MARGIN_M = 0.25
RUNTIME_MULTIPLIER_CEILING = 3.0

class SelectorError(ValueError):
    """Raised when the frozen selector contract cannot be satisfied."""


def sha256(path: Path) -> str:
    if not path.is_file():
        raise SelectorError(f"missing file: {path}")
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def relative(path: Path) -> str:
    try:
        return str(path.resolve().relative_to(ROOT.resolve()))
    except ValueError:
        return str(path)


def load_json(path: Path, label: str) -> dict[str, Any]:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise SelectorError(f"invalid {label}: {path}") from exc
    if not isinstance(payload, dict):
        raise SelectorError(f"{label} must be an object: {path}")
    return payload


def atomic_bytes(path: Path, content: bytes) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".tmp", dir=str(path.parent)
    )
    temporary = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "wb") as handle:
            handle.write(content)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, path)
    finally:
        temporary.unlink(missing_ok=True)


def atomic_json(path: Path, payload: dict[str, Any]) -> None:
    atomic_bytes(path, (json.dumps(payload, indent=2, sort_keys=True) + "\n").encode("utf-8"))


def safe_id(dataset_id: str) -> str:
    if dataset_id.count("/") != 1 or any(ch.isspace() for ch in dataset_id):
        raise SelectorError(f"invalid dataset id: {dataset_id}")
    return dataset_id.replace("/", "__")


def verify_freeze() -> dict[str, Any]:
    freeze = load_json(FREEZE, "convergence selector freeze")
    manifest = load_json(FREEZE_MANIFEST, "convergence selector freeze manifest")
    if freeze.get("schema_version") != FREEZE_SCHEMA or freeze.get("status") != "frozen-before-selector-truth-free-materialization":
        raise SelectorError("selector freeze status/schema mismatch")
    if manifest.get("schema_version") != FREEZE_MANIFEST_SCHEMA or manifest.get("freeze_record") != relative(FREEZE):
        raise SelectorError("selector freeze manifest mismatch")
    if manifest.get("freeze_record_sha256") != FREEZE_SHA256 or sha256(FREEZE) != FREEZE_SHA256:
        raise SelectorError("selector freeze record hash mismatch")
    if sha256(FREEZE_MANIFEST) != FREEZE_MANIFEST_SHA256:
        raise SelectorError("selector freeze manifest hash mismatch")
    expected_train = tuple(freeze["split"]["train"])
    if expected_train != TRAIN_IDS or freeze["split"]["fresh_validation"] != FRESH_VALIDATION_ID or freeze["split"]["future_holdout"] != FUTURE_HOLDOUT_ID:
        raise SelectorError("selector split mismatch")
    if freeze["authorization"]["train_truth_open_count_before_freeze"] != 0 or freeze["authorization"]["fresh_validation_truth_open_count_before_freeze"] != 0 or freeze["authorization"]["future_holdout_truth_open_count_before_freeze"] != 0:
        raise SelectorError("selected identity truth was opened before freeze")
    if freeze["split"].get("future_holdout_materialized_before_freeze") is not False:
        raise SelectorError("future holdout was materialized before freeze")
    archive = freeze["archive"]
    if archive["sha256"] != ARCHIVE_SHA256 or archive["central_inventory_sha256"] != CENTRAL_INVENTORY_SHA256 or archive["central_metadata_canonical_sha256"] != CENTRAL_METADATA_SHA256:
        raise SelectorError("archive hash contract mismatch")
    if sha256(ARCHIVE) != ARCHIVE_SHA256:
        raise SelectorError("archive bytes changed")
    inventory_path = ROOT / "output/smartphone-r5/generalization-v6/archive_inventory.json"
    if sha256(inventory_path) != CENTRAL_INVENTORY_SHA256:
        raise SelectorError("central inventory bytes changed")
    for path_string, expected in {
        "apps/native/gnss_fgo.cpp": "2d9e53485c427d1e0d43b07dd611aaf3644e2cd51d5e175c2977038ca05e3e32",
        "src/algorithms/fgo.cpp": "4805a30741e3bcc53a2a29d832429924a3cc46c0fac5aa530446cedd224af3b2",
        "include/libgnss++/algorithms/fgo.hpp": "67845f6365eb1f621afed19c52b82ad48d00ac1623e99b3bf7b96cd944263307",
        "include/libgnss++/algorithms/fgo_config.hpp": "8a9390e6709f4c4a55ca4f9d4d43fd451465234d1aa55cd4dd662b06f0872d80",
        "apps/commands/benchmarks/gnss_smartphone_gnss_adapter.py": "4a26dbbef0d5eff4a4840c43b600d790d573accc9977a9993754386ad086466b",
        "configs/benchmarks/smartphone_r5_gsdc2023.json": PROFILE_SHA256,
        "apps/commands/benchmarks/gnss_smartphone_native_fgo_optimizer_stop_eval.py": OPTIMIZER_SOURCE_SHA256,
    }.items():
        if sha256(ROOT / path_string) != expected:
            raise SelectorError(f"frozen source changed: {path_string}")
    if sha256(FGO_BINARY) != FGO_BINARY_SHA256 or sha256(SPP_BINARY) != SPP_BINARY_SHA256 or sha256(PROFILE) != PROFILE_SHA256:
        raise SelectorError("frozen binary/profile changed")
    selector = freeze["selector_contract"]
    if selector.get("truth_free_features_only") is not True or selector.get("truth_or_leaderboard_features") is not False:
        raise SelectorError("selector is not truth-free")
    if manifest.get("continuity_speed_bound_mps") != SPEED_BOUND_MPS:
        raise SelectorError("continuity speed bound changed")
    algorithm = freeze["algorithm_contract"]
    if algorithm.get("baseline8") != {
        "max_iterations": 8,
        "relative_cost_convergence_threshold": 0.0,
        "absolute_cost_convergence_threshold": 0.0,
    } or algorithm.get("candidate50") != {
        "max_iterations": 50,
        "relative_cost_convergence_threshold": 0.000001,
        "absolute_cost_convergence_threshold": 0.0,
    }:
        raise SelectorError("optimizer stopping contract changed")
    if freeze["promotion_gate"]["vertical_p95_safety_margin_m"] != VERTICAL_MARGIN_M:
        raise SelectorError("vertical safety margin changed")
    if freeze["promotion_gate"]["future_holdout_remains_sealed"] is not True or freeze["promotion_gate"]["no_post_truth_tuning"] is not True:
        raise SelectorError("selector promotion policy is not closed")
    return freeze


def artifact(path: Path) -> dict[str, Any]:
    if not path.is_file():
        raise SelectorError(f"missing artifact: {path}")
    return {"path": relative(path), "bytes": path.stat().st_size, "sha256": sha256(path)}


def atomic_copy(source: Path, target: Path) -> dict[str, Any]:
    if not source.is_file():
        raise SelectorError(f"missing source output: {source}")
    target.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(prefix=f".{target.name}.", suffix=".tmp", dir=str(target.parent))
    temporary = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "wb") as handle, source.open("rb") as source_handle:
            shutil.copyfileobj(source_handle, handle, length=1024 * 1024)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, target)
    finally:
        temporary.unlink(missing_ok=True)
    return artifact(target)


def cost_trace_stats(path: Path, max_iterations: int) -> dict[str, Any]:
    rows: list[dict[str, Any]] = []
    try:
        with path.open(encoding="utf-8", newline="") as handle:
            reader = csv.DictReader(handle)
            required = {"global_iteration", "cost", "converged"}
            if not required.issubset(set(reader.fieldnames or ())):
                raise SelectorError("cost trace schema is incomplete")
            for raw in reader:
                try:
                    iteration = int(raw["global_iteration"])
                    cost = float(raw["cost"])
                except (TypeError, ValueError) as exc:
                    raise SelectorError("cost trace numeric field is invalid") from exc
                if iteration < 0 or iteration > max_iterations or not math.isfinite(cost):
                    raise SelectorError("cost trace is non-finite or exceeds max bound")
                rows.append({"iteration": iteration, "cost": cost, "converged": raw["converged"].strip().lower() in {"1", "true", "yes"}})
    except OSError as exc:
        raise SelectorError("cannot read cost trace") from exc
    if not rows:
        raise SelectorError("cost trace is empty")
    iterations = [row["iteration"] for row in rows]
    if iterations != sorted(iterations) or len(iterations) != len(set(iterations)):
        raise SelectorError("cost trace iterations are not strictly ordered")
    increases = [current["cost"] - previous["cost"] for previous, current in zip(rows, rows[1:])]
    monotonic = all(delta <= 1e-9 for delta in increases)
    return {
        "rows": len(rows),
        "initial_cost": rows[0]["cost"],
        "final_cost": rows[-1]["cost"],
        "max_cost_increase": max(increases, default=0.0),
        "monotonic_non_increasing": monotonic,
        "converged_rows": sum(1 for row in rows if row["converged"]),
    }


def candidate_rejection_reasons(
    summary: dict[str, Any],
    trace: dict[str, Any],
    keys: dict[str, Any],
    continuity: dict[str, Any],
) -> list[str]:
    """Return deterministic truth-free reasons for rejecting candidate50."""

    reasons: list[str] = []
    if summary.get("converged") is not True:
        reasons.append("candidate_summary_not_converged")
    try:
        iterations = int(summary.get("iterations", -1))
    except (TypeError, ValueError):
        iterations = -1
    if not 0 < iterations < 50:
        reasons.append("candidate_did_not_terminate_before_max_iterations")
    if trace.get("converged_rows", 0) <= 0:
        reasons.append("candidate_trace_has_no_convergence_row")
    if not trace.get("monotonic_non_increasing", False):
        reasons.append("candidate_cost_not_monotonic_non_increasing")
    if not keys.get("exact_key_coverage") or not keys.get("finite_coordinates"):
        reasons.append("candidate_key_or_finite_output_contract_failed")
    for key in ("pseudorange_factors", "tdcp_factors", "motion_factors"):
        try:
            valid = int(summary.get(key, 0)) > 0
        except (TypeError, ValueError):
            valid = False
        if not valid:
            reasons.append(f"candidate_missing_{key}")
    for key in (
        "single_difference_doppler_factors",
        "single_difference_tdcp_factors",
        "carrier_phase_factors",
        "double_difference_pseudorange_factors",
        "double_difference_carrier_factors",
    ):
        try:
            forbidden = int(summary.get(key, 0)) != 0
        except (TypeError, ValueError):
            forbidden = True
        if forbidden:
            reasons.append(f"candidate_forbidden_factor_{key}")
    if not continuity.get("physical_continuity_passed", False):
        reasons.append("candidate_physical_continuity_failed")
    return reasons


def choose_lane(reasons: list[str]) -> str:
    """Choose the candidate only with an empty rejection list."""

    return "candidate50" if not reasons else "baseline8"


def continuity_stats(path: Path) -> dict[str, Any]:
    positions = smoother._read_positions(path, LEAP_SECONDS)
    speeds: list[float] = []
    for previous, current in zip(positions, positions[1:]):
        delta_seconds = (current.timestamp_ms - previous.timestamp_ms) / 1000.0
        if delta_seconds <= 0.0:
            raise SelectorError(f"non-increasing output timestamp: {path}")
        distance = float(((current.ecef - previous.ecef) ** 2).sum() ** 0.5)
        speed = distance / delta_seconds
        if not math.isfinite(speed):
            raise SelectorError(f"non-finite output speed: {path}")
        speeds.append(speed)
    return {
        "epochs": len(positions),
        "max_speed_mps": max(speeds, default=0.0),
        "above_speed_bound_count": sum(speed > SPEED_BOUND_MPS for speed in speeds),
        "physical_continuity_passed": all(speed <= SPEED_BOUND_MPS for speed in speeds),
    }


def epoch_key_check(path: Path, device_path: Path) -> dict[str, Any]:
    positions = smoother._read_positions(path, LEAP_SECONDS)
    epochs = smoother._read_device_epochs(device_path, 0)
    position_keys = [row.timestamp_ms for row in positions]
    missing = sorted(set(epochs) - set(position_keys))
    extra = sorted(set(position_keys) - set(epochs))
    return {
        "device_epochs": len(epochs),
        "output_epochs": len(position_keys),
        "missing_count": len(missing),
        "extra_count": len(extra),
        "duplicate_count": len(position_keys) - len(set(position_keys)),
        "finite_coordinates": all(math.isfinite(float(value)) for row in positions for value in (*row.ecef, row.latitude, row.longitude, row.height)),
        "exact_key_coverage": not missing and not extra and len(position_keys) == len(epochs),
    }


def selector_decision(route_root: Path, device_path: Path) -> dict[str, Any]:
    baseline = load_json(route_root / "baseline8/fgo_summary.json", "baseline summary")
    candidate = load_json(route_root / "candidate50/fgo_summary.json", "candidate summary")
    baseline_trace = cost_trace_stats(route_root / "baseline8/fgo_cost_trace.csv", 8)
    candidate_trace = cost_trace_stats(route_root / "candidate50/fgo_cost_trace.csv", 50)
    baseline_keys = epoch_key_check(route_root / "baseline8/fgo.pos", device_path)
    candidate_keys = epoch_key_check(route_root / "candidate50/fgo.pos", device_path)
    baseline_continuity = continuity_stats(route_root / "baseline8/fgo.pos")
    candidate_continuity = continuity_stats(route_root / "candidate50/fgo.pos")
    if not baseline_keys["exact_key_coverage"] or not baseline_keys["finite_coordinates"]:
        raise SelectorError("baseline8 key or finite-output contract failed")
    if not baseline_continuity["physical_continuity_passed"]:
        raise SelectorError("baseline8 physical continuity contract failed")
    reasons = candidate_rejection_reasons(candidate, candidate_trace, candidate_keys, candidate_continuity)
    selected_lane = choose_lane(reasons)
    selected_source = route_root / selected_lane / "fgo.pos"
    selected_dir = route_root / "selected"
    selected = atomic_copy(selected_source, selected_dir / "fgo.pos")
    selected_summary = atomic_copy(route_root / selected_lane / "fgo_summary.json", selected_dir / "fgo_summary.json")
    selected_trace = atomic_copy(route_root / selected_lane / "fgo_cost_trace.csv", selected_dir / "fgo_cost_trace.csv")
    selected_keys = epoch_key_check(selected_dir / "fgo.pos", device_path)
    selected_continuity = continuity_stats(selected_dir / "fgo.pos")
    if selected_lane == "baseline8" and selected["sha256"] != sha256(route_root / "baseline8/fgo.pos"):
        raise SelectorError("baseline fallback is not byte-exact")
    return {
        "selected_lane": selected_lane,
        "decision": "candidate50" if selected_lane == "candidate50" else "baseline8-fallback",
        "candidate_rejection_reasons": reasons,
        "baseline": {"summary": baseline, "cost_trace": baseline_trace, "keys": baseline_keys, "continuity": baseline_continuity},
        "candidate": {"summary": candidate, "cost_trace": candidate_trace, "keys": candidate_keys, "continuity": candidate_continuity},
        "selected": {"artifacts": {"fgo.pos": selected, "fgo_summary.json": selected_summary, "fgo_cost_trace.csv": selected_trace}, "keys": selected_keys, "continuity": selected_continuity, "byte_exact_to_lane": selected_lane},
    }


def patch_route_selector_manifest(route_root: Path, dataset_id: str, freeze: dict[str, Any], decision: dict[str, Any]) -> dict[str, Any]:
    manifest_path = route_root / "route_manifest.json"
    manifest = load_json(manifest_path, "route manifest")
    manifest["source_hashes"]["selector_freeze_record"] = sha256(FREEZE)
    manifest["source_hashes"]["selector_freeze_manifest"] = sha256(FREEZE_MANIFEST)
    manifest["source_hashes"]["selector_cli"] = sha256(Path(__file__).resolve())
    manifest["selector"] = decision
    manifest["selector_contract"] = freeze["selector_contract"]
    manifest["status"] = "truth-free-selector-artifacts-sealed"
    manifest["truth_opened"] = False
    atomic_json(manifest_path, manifest)
    route_hash = sha256(manifest_path)
    atomic_bytes(route_root / "route_manifest.sha256", f"{route_hash}  route_manifest.json\n".encode("ascii"))
    manifest["route_manifest_sha256"] = route_hash
    return manifest


def run_truth_free(freeze: dict[str, Any], output_root: Path) -> dict[str, Any]:
    if output_root.exists():
        raise SelectorError(f"selector output already exists; refusing rerun: {output_root}")
    output_root.mkdir(parents=True, exist_ok=True)
    (output_root / "train").mkdir(parents=True, exist_ok=True)
    index = optimizer.central_index()
    route_manifests: dict[str, Any] = {}
    for dataset_id in TRAIN_IDS:
        optimizer.run_route(freeze, index, dataset_id, output_root)
        route_root = output_root / "train" / safe_id(dataset_id)
        decision = selector_decision(route_root, route_root / "inputs/device_gnss.csv")
        route_manifests[dataset_id] = patch_route_selector_manifest(route_root, dataset_id, freeze, decision)
    top = {
        "schema_version": RUN_SCHEMA,
        "status": "truth-free-selector-artifacts-sealed",
        "candidate_id": freeze["candidate_id"],
        "train": list(TRAIN_IDS),
        "fresh_validation": FRESH_VALIDATION_ID,
        "future_holdout": FUTURE_HOLDOUT_ID,
        "truth_open_count": 0,
        "fresh_validation_truth_open_count": 0,
        "future_holdout_truth_open_count": 0,
        "truth_free_dual_outputs_before_selector": True,
        "selector_truth_free": True,
        "route_manifest_sha256": {dataset_id: sha256(output_root / "train" / safe_id(dataset_id) / "route_manifest.json") for dataset_id in TRAIN_IDS},
        "selector_output_sha256": {dataset_id: sha256(output_root / "train" / safe_id(dataset_id) / "selected/fgo.pos") for dataset_id in TRAIN_IDS},
        "freeze_record_sha256": sha256(FREEZE),
        "freeze_manifest_sha256": sha256(FREEZE_MANIFEST),
        "archive_sha256": ARCHIVE_SHA256,
        "candidate_source_hashes": {"optimizer_stop_cli": OPTIMIZER_SOURCE_SHA256, "selector_cli": sha256(Path(__file__).resolve())},
        "fgo_binary_sha256": FGO_BINARY_SHA256,
        "spp_binary_sha256": SPP_BINARY_SHA256,
        "prior_v5_unchanged": True,
        "external_mutation": False,
        "validation_or_holdout_materialized": False,
        "atomic_publish": True,
    }
    atomic_json(output_root / "truth_free_manifest.json", top)
    return top


def verify_route(route_root: Path, dataset_id: str) -> dict[str, Any]:
    manifest = load_json(route_root / "route_manifest.json", "selector route manifest")
    seal = (route_root / "route_manifest.sha256").read_text(encoding="ascii").split()[0]
    if seal != sha256(route_root / "route_manifest.json") or manifest.get("dataset_id") != dataset_id or manifest.get("status") != "truth-free-selector-artifacts-sealed" or manifest.get("truth_opened") is not False:
        raise SelectorError(f"selector route manifest contract mismatch: {dataset_id}")
    for lane in ("baseline8", "candidate50"):
        lane_artifacts = manifest.get("lanes", {}).get(lane, {}).get("artifacts", {})
        if not lane_artifacts:
            raise SelectorError(f"missing selector lane artifacts: {dataset_id}/{lane}")
        for item in lane_artifacts.values():
            if not isinstance(item, dict) or not isinstance(item.get("path"), str) or not isinstance(item.get("sha256"), str):
                raise SelectorError(f"invalid selector lane artifact record: {dataset_id}/{lane}")
            path = Path(item["path"])
            if not path.is_absolute():
                path = ROOT / path
            if sha256(path) != item["sha256"]:
                raise SelectorError(f"selector lane artifact hash mismatch: {dataset_id}/{lane}")
    selected = manifest.get("selector", {}).get("selected", {}).get("artifacts", {})
    for item in selected.values():
        path = Path(item["path"])
        if not path.is_absolute():
            path = ROOT / path
        if sha256(path) != item["sha256"]:
            raise SelectorError(f"selected artifact hash mismatch: {dataset_id}")
    selected_lane = manifest.get("selector", {}).get("selected_lane")
    if selected_lane not in {"baseline8", "candidate50"}:
        raise SelectorError(f"invalid selector lane: {dataset_id}")
    selected_pos = manifest["selector"]["selected"]["artifacts"]["fgo.pos"]
    lane_pos = manifest["lanes"][selected_lane]["artifacts"]["fgo.pos"]
    if selected_pos["sha256"] != lane_pos["sha256"]:
        raise SelectorError(f"selected fgo.pos is not byte-exact to lane: {dataset_id}")
    return manifest


def score_route(route_root: Path, dataset_id: str, index: dict[str, dict[str, Any]], freeze: dict[str, Any]) -> dict[str, Any]:
    selected = freeze["central_directory_selection"][dataset_id]
    truth_path = route_root.parent.parent / "train_truth" / safe_id(dataset_id) / "ground_truth.csv"
    if truth_path.exists():
        raise SelectorError(f"truth already materialized; refusing repeat: {dataset_id}")
    truth_path.parent.mkdir(parents=True, exist_ok=True)
    truth_artifact = optimizer.materialize_member(index, selected["ground_truth"], truth_path)
    truth = smoother_eval._read_truth(truth_path)
    device = route_root / "inputs/device_gnss.csv"
    paths = {
        "baseline8": route_root / "baseline8/fgo.pos",
        "candidate50": route_root / "candidate50/fgo.pos",
        "selector": route_root / "selected/fgo.pos",
    }
    metrics = {lane: optimizer._score_optimizer_position(path, device, truth) for lane, path in paths.items()}
    baseline = metrics["baseline8"]
    selected_metrics = metrics["selector"]
    variants = optimizer.native_eval.DIAGNOSTIC_KEYS
    strict_route = all(
        selected_metrics["kaggle_diagnostic_score_variants_m"].get(key) is not None
        and baseline["kaggle_diagnostic_score_variants_m"].get(key) is not None
        and selected_metrics["kaggle_diagnostic_score_variants_m"][key] < baseline["kaggle_diagnostic_score_variants_m"][key]
        for key in variants
    )
    selected_vertical = float(selected_metrics["vertical_p95_abs_m"])
    baseline_vertical = float(baseline["vertical_p95_abs_m"])
    route_report = load_json(route_root / "route_manifest.json", "selector route manifest")
    selected_lane = route_report["selector"]["selected_lane"]
    return {
        "dataset_id": dataset_id,
        "truth": {"artifact": truth_artifact, "read_count": 1},
        "selected_lane": selected_lane,
        "metrics": metrics,
        "gate": {
            "all_four_diagnostics_strictly_improve": strict_route,
            "diagnostic_mean_strictly_improves": selected_metrics["kaggle_diagnostic_mean_m"] < baseline["kaggle_diagnostic_mean_m"],
            "availability_non_regression": selected_metrics["availability_ratio"] >= baseline["availability_ratio"],
            "truth_coverage_non_regression": selected_metrics["truth_coverage_ratio"] >= baseline["truth_coverage_ratio"],
            "vertical_within_margin": selected_vertical <= baseline_vertical + VERTICAL_MARGIN_M,
            "passed": strict_route and selected_metrics["kaggle_diagnostic_mean_m"] < baseline["kaggle_diagnostic_mean_m"] and selected_metrics["availability_ratio"] >= baseline["availability_ratio"] and selected_metrics["truth_coverage_ratio"] >= baseline["truth_coverage_ratio"] and selected_vertical <= baseline_vertical + VERTICAL_MARGIN_M,
        },
        "route_manifest_sha256": sha256(route_root / "route_manifest.json"),
    }


def train_score(freeze: dict[str, Any], output_root: Path) -> dict[str, Any]:
    top = load_json(output_root / "truth_free_manifest.json", "selector truth-free manifest")
    if top.get("truth_open_count") != 0 or top.get("future_holdout_truth_open_count") != 0:
        raise SelectorError("truth-free selector manifest is not sealed")
    for dataset_id in TRAIN_IDS:
        verify_route(output_root / "train" / safe_id(dataset_id), dataset_id)
    reports = [score_route(output_root / "train" / safe_id(dataset_id), dataset_id, optimizer.central_index(), freeze) for dataset_id in TRAIN_IDS]
    baseline = [report["metrics"]["baseline8"] for report in reports]
    selected = [report["metrics"]["selector"] for report in reports]
    baseline_aggregate = optimizer.native_eval._aggregate(baseline)
    selected_aggregate = optimizer.native_eval._aggregate(selected)
    strict_aggregate = all(
        selected_aggregate["mean_kaggle_diagnostic_score_variants_m"].get(key) is not None
        and selected_aggregate["mean_kaggle_diagnostic_score_variants_m"][key] < baseline_aggregate["mean_kaggle_diagnostic_score_variants_m"][key]
        for key in optimizer.native_eval.DIAGNOSTIC_KEYS
    )
    aggregate_gate = {
        "all_four_diagnostics_strictly_improve": strict_aggregate,
        "diagnostic_mean_strictly_improves": selected_aggregate["mean_kaggle_diagnostic_m"] < baseline_aggregate["mean_kaggle_diagnostic_m"],
        "availability_non_regression": selected_aggregate["mean_availability_ratio"] >= baseline_aggregate["mean_availability_ratio"],
        "truth_coverage_non_regression": selected_aggregate["mean_truth_coverage_ratio"] >= baseline_aggregate["mean_truth_coverage_ratio"],
        "vertical_within_margin": selected_aggregate["mean_vertical_p95_abs_m"] <= baseline_aggregate["mean_vertical_p95_abs_m"] + VERTICAL_MARGIN_M,
    }
    route_gate = all(report["gate"]["passed"] for report in reports)
    train_passed = route_gate and all(aggregate_gate.values())
    report = {
        "schema_version": TRAIN_SCHEMA,
        "status": "train-pass" if train_passed else "No-Go",
        "candidate_id": freeze["candidate_id"],
        "truth_free_selector_artifacts_sealed_before_truth": True,
        "truth_open_count": len(reports),
        "fresh_validation_truth_open_count": 0,
        "future_holdout_truth_open_count": 0,
        "routes": reports,
        "aggregate": {"baseline8": baseline_aggregate, "selector": selected_aggregate, "gate": aggregate_gate},
        "train_gate": {"route_gates": route_gate, "aggregate_gate": all(aggregate_gate.values()), "passed": train_passed},
        "validation_policy": {"opened": False, "fresh_validation": FRESH_VALIDATION_ID, "reason": "selector train gate did not authorize validation" if not train_passed else "validation may be opened once unchanged"},
        "policy": {"truth_features": False, "leaderboard_used_for_tuning": False, "old_holdout_used": False, "post_truth_tuning": False, "future_holdout_sealed": True, "external_mutation": False, "token_access": False},
        "freeze": {"record": relative(FREEZE), "record_sha256": sha256(FREEZE), "manifest": relative(FREEZE_MANIFEST), "manifest_sha256": sha256(FREEZE_MANIFEST)},
    }
    atomic_json(output_root / "train_evaluation.json", report)
    atomic_json(output_root / "train_evaluation.manifest.json", {"schema_version": "smartphone-r5-gsdc2023-native-fgo-convergence-selector-train-manifest.v1", "status": report["status"], "report_sha256": sha256(output_root / "train_evaluation.json"), "truth_open_count": len(reports), "fresh_validation_truth_open_count": 0, "future_holdout_truth_open_count": 0, "no_post_truth_tuning": True})
    return report


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(prog="gnss_smartphone_native_fgo_convergence_selector_eval")
    subparsers = parser.add_subparsers(dest="mode", required=True)
    for mode in ("truth-free-run", "train-score"):
        sub = subparsers.add_parser(mode)
        sub.add_argument("--output-root", type=Path, default=OUTPUT_ROOT)
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    try:
        freeze = verify_freeze()
        output_root = args.output_root if args.output_root.is_absolute() else ROOT / args.output_root
        if args.mode == "truth-free-run":
            result = run_truth_free(freeze, output_root)
            print(json.dumps({"status": result["status"], "train": result["train"], "truth_open_count": 0}, sort_keys=True))
            return 0
        result = train_score(freeze, output_root)
        print(json.dumps({"status": result["status"], "train_gate": result["train_gate"], "truth_open_count": result["truth_open_count"]}, sort_keys=True))
        return 0 if result["status"] == "train-pass" else 2
    except (SelectorError, OSError, ValueError, KeyError, TypeError) as exc:
        print(f"convergence-selector error: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
