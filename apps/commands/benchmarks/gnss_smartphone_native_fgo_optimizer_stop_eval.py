#!/usr/bin/env python3
"""Evaluate a frozen native-FGO optimizer stopping policy.

This is a research-only orchestration layer.  It keeps the native-FGO v1
factor graph, initialization, measurement selection, robust loss, and output
format fixed.  The only candidate difference is the stopping policy:

* baseline: eight fixed Eigen iterations, with cost stopping disabled;
* candidate: a 50-iteration safety bound and a relative weighted-cost
  decrease threshold of 1e-6, matching the installed GTSAM Ceres-default
  stopping values.

The command is intentionally truth-free until ``train-score``.  It extracts
only the selected train device GNSS and broadcast-nav members, builds both
lanes from the same adapter/SPP seed, seals all route artifacts atomically,
then opens each train truth member exactly once for scoring.  Validation and
future holdout are never materialized by this command.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import resource
import shutil
import subprocess
import sys
import tempfile
import time
from typing import Any
import zipfile

# The benchmark is both imported by Python tests and executed directly.  In
# the latter case the commands directory is not on sys.path yet, while the
# shared runtime helper intentionally lives there.
_COMMANDS_DIR = Path(__file__).resolve().parents[1]
if str(_COMMANDS_DIR) not in sys.path:
    sys.path.insert(0, str(_COMMANDS_DIR))
from support.gnss_runtime import application_root  # noqa: E402


ROOT = application_root(__file__)
BENCHMARK_DIR = ROOT / "apps" / "commands" / "benchmarks"
if str(BENCHMARK_DIR) not in sys.path:
    sys.path.insert(0, str(BENCHMARK_DIR))

import gnss_smartphone_native_fgo_eval as native_eval  # noqa: E402
import gnss_smartphone_trajectory_smoother as smoother  # noqa: E402
import gnss_smartphone_trajectory_smoother_eval as smoother_eval  # noqa: E402


FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_gsdc2023_native_fgo_optimizer_stop_freeze_v1.json"
FREEZE_MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_gsdc2023_native_fgo_optimizer_stop_freeze_v1_manifest.json"
ARCHIVE = ROOT / "data/gsdc2023/cache/dataset_2023.zip"
PROFILE = ROOT / "configs/benchmarks/smartphone_r5_gsdc2023.json"
FGO_BINARY = ROOT / "build/apps/gnss_fgo"
SPP_BINARY = ROOT / "build/apps/gnss_spp"
DEFAULT_OUTPUT = ROOT / "output/smartphone-r5/native-fgo-optimizer-stop-v1"
TRAIN_IDS = (
    "2021-03-10-23-13-us-ca-mtv-h/pixel5",
    "2021-08-24-20-32-us-ca-mtv-h/pixel5",
    "2022-04-01-18-22-us-ca-lax-t/pixel5",
)
FRESH_VALIDATION_ID = "2023-05-09-21-32-us-ca-mtv-pe1/pixel5"
FUTURE_HOLDOUT_ID = "2023-05-16-19-54-us-ca-mtv-xe1/pixel5"
ARCHIVE_SHA256 = "bda30ab456e6fd6f83550c246e8dbd287306d5385f1f1069c99c16298e647408"
CENTRAL_INVENTORY_SHA256 = "f4e68109885eecfc14b2bd5e8fab87e18d73473c04bb7f53da31b7040a8e90a7"
CENTRAL_METADATA_SHA256 = "5f619be94a00c33c3067f13b1cd3351d96f2804f07fa04eda8fb027739fb0992"
PROFILE_SHA256 = "273dfcc4e4636940d5216cca793a55773d9a04f668d1bfa5fcdc0013f4768776"
FGO_SOURCE_HASHES = {
    "apps/native/gnss_fgo.cpp": "2d9e53485c427d1e0d43b07dd611aaf3644e2cd51d5e175c2977038ca05e3e32",
    "src/algorithms/fgo.cpp": "4805a30741e3bcc53a2a29d832429924a3cc46c0fac5aa530446cedd224af3b2",
    "include/libgnss++/algorithms/fgo.hpp": "67845f6365eb1f621afed19c52b82ad48d00ac1623e99b3bf7b96cd944263307",
    "include/libgnss++/algorithms/fgo_config.hpp": "8a9390e6709f4c4a55ca4f9d4d43fd451465234d1aa55cd4dd662b06f0872d80",
    "apps/commands/benchmarks/gnss_smartphone_gnss_adapter.py": "4a26dbbef0d5eff4a4840c43b600d790d573accc9977a9993754386ad086466b",
    "configs/benchmarks/smartphone_r5_gsdc2023.json": PROFILE_SHA256,
}
FGO_BINARY_SHA256 = "9190455cb34a9e818b79ed2ed6b023af5711fbef545215d4588bfed87f5c03e4"
SPP_BINARY_SHA256 = "4d272940437c2ab2dcffef31b7541c8c01212e97d2d2320edcf7bd8f80ea3c12"
FREEZE_RECORD_SHA256 = "30285097322549612555d2dfc99d9c1a5e14613bda1b882dfa94e1e753263860"
FREEZE_MANIFEST_SCHEMA = "smartphone-r5-gsdc2023-native-fgo-optimizer-stop-freeze-manifest.v1"
FREEZE_SCHEMA = "smartphone-r5-gsdc2023-native-fgo-optimizer-stop-freeze.v1"
ROUTE_SCHEMA = "smartphone-r5-gsdc2023-native-fgo-optimizer-stop-route.v1"
RUN_SCHEMA = "smartphone-r5-gsdc2023-native-fgo-optimizer-stop-run.v1"
TRAIN_SCHEMA = "smartphone-r5-gsdc2023-native-fgo-optimizer-stop-train-evaluation.v1"
LEAP_SECONDS = 18
SKIP_EPOCHS = 1
MATCH_TOLERANCE_MS = 100
MAX_RUNTIME_SECONDS = 900
MAX_ADDRESS_SPACE_BYTES = 8 * 1024 * 1024 * 1024
MAX_OUTPUT_BYTES = 1024 * 1024 * 1024

# Complete immutable candidate surface: the graph recipe is shared and only
# optimizer stopping values differ between these two lanes.
STOP_POLICIES = (
    ("baseline8", 8, 0.0, 0.0),
    ("candidate50", 50, 1e-6, 0.0),
)


class OptimizerStopError(ValueError):
    """Raised when the frozen optimizer-stop contract is violated."""


def sha256(path: Path) -> str:
    if not path.is_file():
        raise OptimizerStopError(f"missing file: {path}")
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
        raise OptimizerStopError(f"invalid {label}: {path}") from exc
    if not isinstance(payload, dict):
        raise OptimizerStopError(f"{label} must be an object: {path}")
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
        raise OptimizerStopError(f"invalid dataset id: {dataset_id}")
    return dataset_id.replace("/", "__")


def verify_freeze() -> dict[str, Any]:
    freeze = load_json(FREEZE, "optimizer-stop freeze")
    if freeze.get("schema_version") != FREEZE_SCHEMA or freeze.get("status") != "frozen-before-optimizer-candidate-materialization":
        raise OptimizerStopError("optimizer-stop freeze is not pre-materialization")
    manifest = load_json(FREEZE_MANIFEST, "optimizer-stop freeze manifest")
    if manifest.get("schema_version") != FREEZE_MANIFEST_SCHEMA:
        raise OptimizerStopError("optimizer-stop freeze manifest schema mismatch")
    if manifest.get("freeze_record") != relative(FREEZE) or manifest.get("freeze_record_sha256") != FREEZE_RECORD_SHA256:
        raise OptimizerStopError("optimizer-stop freeze record hash mismatch")
    if sha256(FREEZE) != FREEZE_RECORD_SHA256:
        raise OptimizerStopError("optimizer-stop freeze record changed")
    if manifest.get("truth_open_count_before_freeze") != 0 or manifest.get("future_holdout_truth_open_count") != 0:
        raise OptimizerStopError("truth was opened before optimizer-stop freeze")
    if manifest.get("future_holdout_materialized") is not False or manifest.get("candidate_materialization_root_absent_before_freeze") is not True:
        raise OptimizerStopError("pre-materialization contract is not closed")
    archive = freeze.get("archive")
    if not isinstance(archive, dict) or archive.get("sha256") != ARCHIVE_SHA256 or archive.get("central_inventory_sha256") != CENTRAL_INVENTORY_SHA256 or archive.get("central_metadata_canonical_sha256") != CENTRAL_METADATA_SHA256:
        raise OptimizerStopError("archive contract mismatch")
    if sha256(ARCHIVE) != ARCHIVE_SHA256:
        raise OptimizerStopError("archive hash changed")
    inventory_path = ROOT / str(archive["central_inventory_path"])
    if sha256(inventory_path) != CENTRAL_INVENTORY_SHA256:
        raise OptimizerStopError("central inventory hash changed")
    split = freeze.get("split")
    if not isinstance(split, dict) or tuple(split.get("train", ())) != TRAIN_IDS or split.get("fresh_validation") != FRESH_VALIDATION_ID or split.get("future_holdout") != FUTURE_HOLDOUT_ID:
        raise OptimizerStopError("optimizer-stop split mismatch")
    if split.get("future_holdout_remains_sealed") is not True:
        raise OptimizerStopError("future holdout is not sealed")
    for path_string, expected in FGO_SOURCE_HASHES.items():
        path = ROOT / path_string
        if sha256(path) != expected:
            raise OptimizerStopError(f"source changed: {path_string}")
    if sha256(FGO_BINARY) != FGO_BINARY_SHA256 or sha256(SPP_BINARY) != SPP_BINARY_SHA256:
        raise OptimizerStopError("Release binary hash changed")
    if sha256(PROFILE) != PROFILE_SHA256:
        raise OptimizerStopError("profile hash changed")
    recipe = freeze.get("candidate_contract", {})
    baseline = recipe.get("baseline", {})
    candidate = recipe.get("candidate", {})
    if baseline != {
        "max_iterations": 8,
        "relative_cost_convergence_threshold": 0.0,
        "absolute_cost_convergence_threshold": 0.0,
        "stopping": "fixed max bound",
    }:
        raise OptimizerStopError("baseline stopping contract changed")
    if candidate.get("max_iterations") != 50 or candidate.get("relative_cost_convergence_threshold") != 1e-06 or candidate.get("absolute_cost_convergence_threshold") != 0.0:
        raise OptimizerStopError("candidate stopping contract changed")
    return freeze


def central_index() -> dict[str, dict[str, Any]]:
    result: dict[str, dict[str, Any]] = {}
    try:
        with zipfile.ZipFile(ARCHIVE) as archive:
            for info in archive.infolist():
                result[info.filename] = {
                    "name": info.filename,
                    "file_size": info.file_size,
                    "compressed_size": info.compress_size,
                    "crc32_hex": f"{info.CRC:08x}",
                    "is_dir": info.is_dir(),
                }
    except (OSError, zipfile.BadZipFile) as exc:
        raise OptimizerStopError("failed to inspect archive central directory") from exc
    return result


def member_metadata(freeze: dict[str, Any], dataset_id: str, key: str) -> dict[str, Any]:
    selected = freeze.get("central_directory_selection", {}).get(dataset_id)
    if not isinstance(selected, dict) or not isinstance(selected.get(key), dict):
        raise OptimizerStopError(f"missing frozen member metadata: {dataset_id}/{key}")
    value = dict(selected[key])
    value.pop("is_dir", None)
    return value


def verify_members(index: dict[str, dict[str, Any]], freeze: dict[str, Any], dataset_id: str) -> dict[str, dict[str, Any]]:
    verified: dict[str, dict[str, Any]] = {}
    for key in ("device_gnss", "device_imu", "ground_truth", "broadcast_nav", "reference_obs"):
        expected = member_metadata(freeze, dataset_id, key)
        actual = index.get(str(expected["name"]))
        if actual is None or actual.get("is_dir") or {k: actual.get(k) for k in ("name", "file_size", "compressed_size", "crc32_hex")} != expected:
            raise OptimizerStopError(f"central metadata changed: {dataset_id}/{key}")
        verified[key] = expected
    return verified


def materialize_member(index: dict[str, dict[str, Any]], expected: dict[str, Any], output: Path) -> dict[str, Any]:
    if output.exists():
        raise OptimizerStopError(f"refusing to overwrite materialized input: {output}")
    output.parent.mkdir(parents=True, exist_ok=True)
    info = index.get(str(expected["name"]))
    if info is None or info.get("is_dir"):
        raise OptimizerStopError(f"archive member is unavailable: {expected['name']}")
    descriptor, temporary_name = tempfile.mkstemp(prefix=f".{output.name}.", suffix=".tmp", dir=str(output.parent))
    temporary = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "wb") as target:
            with zipfile.ZipFile(ARCHIVE) as archive, archive.open(str(expected["name"]), "r") as source:
                shutil.copyfileobj(source, target, length=1024 * 1024)
            target.flush()
            os.fsync(target.fileno())
        if temporary.stat().st_size != int(expected["file_size"]):
            raise OptimizerStopError(f"materialized size mismatch: {expected['name']}")
        os.replace(temporary, output)
    except (OSError, KeyError, RuntimeError, zipfile.BadZipFile, OptimizerStopError):
        temporary.unlink(missing_ok=True)
        raise
    return {"path": relative(output), "member": expected["name"], "metadata": expected, "bytes": output.stat().st_size, "sha256": sha256(output)}


def child_limits() -> None:
    resource.setrlimit(resource.RLIMIT_AS, (MAX_ADDRESS_SPACE_BYTES, MAX_ADDRESS_SPACE_BYTES))
    resource.setrlimit(resource.RLIMIT_CPU, (MAX_RUNTIME_SECONDS, MAX_RUNTIME_SECONDS + 1))


def run_child(command: list[str], stage_dir: Path, label: str) -> dict[str, Any]:
    stage_dir.mkdir(parents=True, exist_ok=True)
    started = time.perf_counter()
    stdout_path = stage_dir / f"{label}.stdout.log"
    stderr_path = stage_dir / f"{label}.stderr.log"
    environment = os.environ.copy()
    environment["LD_LIBRARY_PATH"] = "/home/sasaki/.local/lib:/opt/ros/jazzy/lib" + (os.pathsep + environment["LD_LIBRARY_PATH"] if environment.get("LD_LIBRARY_PATH") else "")
    timed_out = False
    return_code: int | None = None
    with stdout_path.open("wb") as stdout, stderr_path.open("wb") as stderr:
        try:
            completed = subprocess.run(command, cwd=ROOT, env=environment, stdin=subprocess.DEVNULL, stdout=stdout, stderr=stderr, timeout=MAX_RUNTIME_SECONDS + 5, preexec_fn=child_limits, check=False)
            return_code = completed.returncode
        except subprocess.TimeoutExpired:
            timed_out = True
    return {
        "command": command,
        "return_code": return_code,
        "timed_out": timed_out,
        "wall_seconds": time.perf_counter() - started,
        "child_max_rss_kib": resource.getrusage(resource.RUSAGE_CHILDREN).ru_maxrss,
        "stdout": relative(stdout_path),
        "stderr": relative(stderr_path),
    }


def adapter_command(device: Path, nav: Path, output: Path, dataset_id: str) -> list[str]:
    phone = dataset_id.split("/", 1)[1]
    return [
        sys.executable, str(ROOT / "apps/gnss.py"), "smartphone-gnss-adapter",
        "--device-gnss", str(device), "--output-dir", str(output),
        "--dataset-id", dataset_id, "--device-model", phone,
        "--source-url", "local-frozen-archive", "--source-terms", "GSDC2023 archive hash frozen",
        "--role", "development", "--truth-free", "--skip-epochs", "0",
        "--experimental-galileo-e1", "--experimental-galileo-e1-hatch-window-s", "30",
        "--broadcast-nav", str(nav),
    ]


def spp_command(obs: Path, nav: Path, output: Path) -> list[str]:
    return [
        str(SPP_BINARY), "--obs", str(obs), "--nav", str(nav),
        "--out", str(output / "libgnsspp_spp.pos"),
        "--summary-json", str(output / "libgnsspp_spp_summary.json"),
        "--clock-csv", str(output / "libgnsspp_spp_clock.csv"),
        "--timing-csv", str(output / "libgnsspp_spp_timing.csv"), "--quiet",
    ]


def fgo_command(obs: Path, nav: Path, seed: Path, output: Path, max_iterations: int, relative_threshold: float, absolute_threshold: float) -> list[str]:
    return [
        str(FGO_BINARY), "--obs", str(obs), "--nav", str(nav), "--seed-pos", str(seed),
        "--out", str(output / "fgo.pos"), "--summary-json", str(output / "fgo_summary.json"),
        "--epoch-debug-csv", str(output / "fgo_epoch_debug.csv"),
        "--factor-debug-csv", str(output / "fgo_factor_debug.csv"),
        "--cost-trace-csv", str(output / "fgo_cost_trace.csv"),
        "--preset", "default", "--backend", "eigen", "--skip-epochs", "0", "--max-epochs", "0",
        "--max-iterations", str(max_iterations), "--relative-cost-threshold", format(relative_threshold, ".17g"),
        "--absolute-cost-threshold", format(absolute_threshold, ".17g"),
        "--pseudorange-sigma", "3", "--pseudorange-elevation-power", "1", "--motion-sigma", "50",
        "--clock-motion-sigma", "300", "--velocity-prior-sigma", "100", "--velocity-motion-sigma", "0.01",
        "--position-prior-sigma", "0", "--clock-prior-sigma", "0", "--tdcp-sigma", "0.03",
        "--carrier-phase-sigma", "0.01", "--pseudorange-huber-threshold", "4",
        "--carrier-phase-huber-threshold", "4", "--tdcp-huber-threshold", "4", "--max-tdcp-gap", "2",
        "--seed-match-tolerance", "0.01", "--seed-interpolation-max-gap", "0", "--tdcp-slip-threshold", "10",
        "--min-elevation", "10", "--min-snr", "0", "--min-satellites-per-epoch", "4",
        "--no-dd-factors", "--ionosphere-model", "--troposphere-model", "--quiet",
    ]


def deterministic_converged_artifact_check(
    first: Path, second: Path, summary: dict[str, Any]
) -> dict[str, Any]:
    """Require byte identity for a lane converged by the baseline bound.

    The candidate may legitimately run beyond eight iterations, so the
    repeatability check is conditional.  If it converges at or before the
    frozen baseline bound, a repeat with the same inputs must be byte-identical
    and any mismatch is a fail-closed reproducibility error.
    """

    converged = summary.get("converged") is True
    iterations = int(summary.get("iterations", -1))
    checked = converged and 0 <= iterations <= 8
    if checked:
        if not first.is_file() or not second.is_file():
            raise OptimizerStopError("determinism check input is missing")
        first_hash = sha256(first)
        second_hash = sha256(second)
        if first_hash != second_hash:
            raise OptimizerStopError(
                "converged-by-eight lane is not byte-identical across repeats"
            )
        return {
            "checked": True,
            "iterations": iterations,
            "first_sha256": first_hash,
            "second_sha256": second_hash,
            "byte_identical": True,
        }
    return {
        "checked": False,
        "iterations": iterations,
        "byte_identical": None,
        "reason": "lane did not report convergence at or before eight iterations",
    }


def artifact(path: Path) -> dict[str, Any]:
    if not path.is_file() or path.stat().st_size > MAX_OUTPUT_BYTES:
        raise OptimizerStopError(f"missing or oversized artifact: {path}")
    return {"path": relative(path), "bytes": path.stat().st_size, "sha256": sha256(path)}


def _rewrite_published_paths(
    value: Any, temporary_root: Path, published_root: Path
) -> Any:
    """Rewrite only orchestration path strings across an atomic publish.

    Child commands and artifact records are assembled before the temporary
    route directory is renamed.  They must refer to the final route directory
    after that rename; bytes produced by the child processes are never
    rewritten.  The replacement is exact to the temporary root, preventing a
    path-like payload value from being changed accidentally.
    """

    temporary_abs = str(temporary_root.resolve())
    published_abs = str(published_root.resolve())
    temporary_rel = relative(temporary_root)
    published_rel = relative(published_root)
    if isinstance(value, str):
        if value == temporary_abs or value.startswith(temporary_abs + os.sep):
            return published_abs + value[len(temporary_abs) :]
        if value == temporary_rel or value.startswith(temporary_rel + os.sep):
            return published_rel + value[len(temporary_rel) :]
        return value
    if isinstance(value, list):
        return [_rewrite_published_paths(item, temporary_root, published_root) for item in value]
    if isinstance(value, dict):
        return {
            key: _rewrite_published_paths(item, temporary_root, published_root)
            for key, item in value.items()
        }
    return value


def _refresh_artifact_records(value: Any) -> None:
    """Refresh hashes for nested ``{path, bytes, sha256}`` records."""

    if isinstance(value, dict):
        if isinstance(value.get("path"), str) and {"bytes", "sha256"}.issubset(value):
            path = Path(value["path"])
            if not path.is_absolute():
                path = ROOT / path
            value.update(artifact(path))
        for item in value.values():
            _refresh_artifact_records(item)
    elif isinstance(value, list):
        for item in value:
            _refresh_artifact_records(item)


def repair_published_route(route_root: Path, dataset_id: str) -> dict[str, Any]:
    """Repair stale temporary-root references without rerunning a child.

    This recovery is limited to manifests/seals.  It requires all published
    lane artifacts to already exist and therefore cannot manufacture missing
    solver output.
    """

    route_manifest_path = route_root / "route_manifest.json"
    manifest = load_json(route_manifest_path, "optimizer-stop route manifest")
    if manifest.get("dataset_id") != dataset_id:
        raise OptimizerStopError(f"route manifest dataset mismatch: {dataset_id}")
    stale_value = manifest["lanes"]["baseline8"]["artifacts"]["fgo.pos"]["path"]
    stale_path = Path(stale_value)
    if not stale_path.is_absolute():
        stale_path = ROOT / stale_path
    if stale_path.is_file():
        raise OptimizerStopError(
            "refusing manifest recovery while temporary artifact still exists"
        )
    # .../<train>/.<safe-id>.<random>/baseline8/fgo.pos
    temporary_root = stale_path.parent.parent
    if not temporary_root.name.startswith("."):
        raise OptimizerStopError("route manifest does not contain a temporary root")
    repaired = _rewrite_published_paths(manifest, temporary_root, route_root)

    materialization_path = route_root / "materialization_manifest.json"
    materialization = load_json(materialization_path, "materialization manifest")
    materialization = _rewrite_published_paths(
        materialization, temporary_root, route_root
    )
    _refresh_artifact_records(materialization)
    atomic_json(materialization_path, materialization)
    _refresh_artifact_records(repaired)
    atomic_json(route_manifest_path, repaired)
    route_hash = sha256(route_manifest_path)
    atomic_bytes(
        route_root / "route_manifest.sha256",
        f"{route_hash}  route_manifest.json\n".encode("ascii"),
    )
    return {
        "dataset_id": dataset_id,
        "route_manifest_sha256": route_hash,
        "manifest_recovery": "temporary-root-paths-normalized; solver artifacts untouched",
    }


def validate_spp(path: Path) -> dict[str, Any]:
    rows = smoother._read_positions(path, LEAP_SECONDS)
    if not rows or not all(math.isfinite(float(value)) for row in rows for value in (*row.ecef, row.latitude, row.longitude, row.height)):
        raise OptimizerStopError("SPP seed is empty or non-finite")
    return {"rows": len(rows), "artifact": artifact(path)}


def validate_cost_trace(path: Path, expected_max: int) -> dict[str, Any]:
    try:
        lines = path.read_text(encoding="utf-8").splitlines()
    except OSError as exc:
        raise OptimizerStopError("cannot read FGO cost trace") from exc
    if len(lines) < 2:
        raise OptimizerStopError("FGO cost trace has no iteration telemetry")
    header = [field.strip() for field in lines[0].split(",")]
    required = {"global_iteration", "cost", "absolute_decrease", "relative_decrease", "update_norm", "converged"}
    if not required.issubset(header):
        raise OptimizerStopError("FGO cost trace schema is incomplete")
    index = {name: header.index(name) for name in required}
    costs: list[float] = []
    converged_rows = 0
    for line in lines[1:]:
        fields = line.split(",")
        if len(fields) != len(header):
            raise OptimizerStopError("FGO cost trace row width mismatch")
        try:
            cost = float(fields[index["cost"]])
            iteration = int(fields[index["global_iteration"]])
        except (TypeError, ValueError) as exc:
            raise OptimizerStopError("FGO cost trace contains invalid numeric data") from exc
        if not math.isfinite(cost) or iteration < 0 or iteration > expected_max:
            raise OptimizerStopError(
                "FGO cost trace contains non-finite data or exceeds iteration bound"
            )
        costs.append(cost)
        if fields[index["converged"]].strip().lower() in {"1", "true", "yes"}:
            converged_rows += 1
    if len(costs) > expected_max:
        raise OptimizerStopError("FGO cost trace exceeds frozen iteration bound")
    return {"rows": len(costs), "initial_cost": costs[0], "final_cost": costs[-1], "converged_rows": converged_rows, "artifact": artifact(path)}


def validate_fgo(output: Path, max_iterations: int, relative_threshold: float, absolute_threshold: float) -> dict[str, Any]:
    summary = load_json(output / "fgo_summary.json", "FGO summary")
    required = (
        "backend", "preset", "input_epochs", "optimized_epochs", "valid_solutions",
        "pseudorange_factors", "tdcp_factors", "motion_factors", "tdcp_factors_inserted",
        "single_difference_doppler_factors", "single_difference_tdcp_factors", "carrier_phase_factors",
        "double_difference_pseudorange_factors", "double_difference_carrier_factors",
        "max_iterations", "relative_cost_convergence_threshold", "absolute_cost_convergence_threshold",
        "iterations", "converged", "initial_cost", "final_cost", "last_update_norm_m",
    )
    missing = [key for key in required if key not in summary]
    if missing:
        raise OptimizerStopError(f"FGO summary missing fields: {', '.join(missing)}")
    if summary["backend"] != "eigen" or summary["preset"] != "default":
        raise OptimizerStopError("FGO backend/preset changed")
    if int(summary["max_iterations"]) != max_iterations or not math.isclose(float(summary["relative_cost_convergence_threshold"]), relative_threshold, rel_tol=0.0, abs_tol=1e-15) or not math.isclose(float(summary["absolute_cost_convergence_threshold"]), absolute_threshold, rel_tol=0.0, abs_tol=1e-15):
        raise OptimizerStopError("FGO stopping values differ from requested lane")
    for key in required[2:]:
        if key in {"converged"}:
            if not isinstance(summary[key], bool):
                raise OptimizerStopError("FGO convergence flag is not boolean")
        elif key not in {"backend", "preset"}:
            try:
                if not math.isfinite(float(summary[key])) or float(summary[key]) < 0.0:
                    raise OptimizerStopError(f"FGO summary field invalid: {key}")
            except (TypeError, ValueError) as exc:
                raise OptimizerStopError(f"FGO summary field invalid: {key}") from exc
    if summary["input_epochs"] <= 0 or summary["optimized_epochs"] <= 0 or summary["valid_solutions"] <= 0:
        raise OptimizerStopError("FGO produced no usable epochs")
    if summary["pseudorange_factors"] <= 0 or summary["tdcp_factors"] <= 0 or summary["motion_factors"] <= 0:
        raise OptimizerStopError("required v1 factor class is missing")
    if summary["tdcp_factors_inserted"] != summary["tdcp_factors"]:
        raise OptimizerStopError("TDCP factor insertion count differs")
    for key in ("single_difference_doppler_factors", "single_difference_tdcp_factors", "carrier_phase_factors", "double_difference_pseudorange_factors", "double_difference_carrier_factors"):
        if summary[key] != 0:
            raise OptimizerStopError(f"forbidden factor class is nonzero: {key}")
    positions = smoother._read_positions(output / "fgo.pos", LEAP_SECONDS)
    if len(positions) != int(summary["valid_solutions"]):
        raise OptimizerStopError("FGO summary/output row count mismatch")
    if not all(math.isfinite(float(value)) for row in positions for value in (*row.ecef, row.latitude, row.longitude, row.height)):
        raise OptimizerStopError("FGO output contains non-finite coordinates")
    return {
        "summary": summary,
        "positions": len(positions),
        "cost_trace": validate_cost_trace(output / "fgo_cost_trace.csv", max_iterations),
        "artifacts": {name: artifact(output / name) for name in ("fgo.pos", "fgo_summary.json", "fgo_epoch_debug.csv", "fgo_factor_debug.csv", "fgo_cost_trace.csv")},
    }


def run_route(freeze: dict[str, Any], index: dict[str, dict[str, Any]], dataset_id: str, output_root: Path) -> dict[str, Any]:
    route_started = time.perf_counter()
    route_root = output_root / "train" / safe_id(dataset_id)
    if route_root.exists():
        raise OptimizerStopError(f"refusing to overwrite route output: {route_root}")
    temporary = Path(tempfile.mkdtemp(prefix=f".{safe_id(dataset_id)}.", dir=str(output_root / "train")))
    try:
        verified = verify_members(index, freeze, dataset_id)
        inputs = temporary / "inputs"
        device = materialize_member(index, verified["device_gnss"], inputs / "device_gnss.csv")
        nav = materialize_member(index, verified["broadcast_nav"], inputs / "brdc.nav")
        materialization_manifest = {
            "schema_version": "smartphone-r5-native-fgo-optimizer-stop-materialization.v1",
            "status": "truth-free-inputs-sealed",
            "dataset_id": dataset_id,
            "truth_opened": False,
            "truth_materialized": False,
            "ground_truth_member_declared_only": verified["ground_truth"],
            "archive_sha256": ARCHIVE_SHA256,
            "central_metadata": verified,
            "inputs": {"device_gnss": device, "broadcast_nav": nav},
        }
        # The child stages run below the temporary root, but the manifest is
        # published below route_root.  Normalize those references before the
        # materialization manifest itself is sealed.
        atomic_json(
            temporary / "materialization_manifest.json",
            _rewrite_published_paths(materialization_manifest, temporary, route_root),
        )
        adapter_dir = temporary / "adapter"
        adapter_run = run_child(adapter_command(inputs / "device_gnss.csv", inputs / "brdc.nav", adapter_dir, dataset_id), adapter_dir, "adapter")
        if adapter_run["return_code"] != 0 or adapter_run["timed_out"]:
            raise OptimizerStopError("truth-free adapter failed")
        adapter_summary = load_json(adapter_dir / "summary.json", "adapter summary")
        if adapter_summary.get("truth_free") is not True or adapter_summary.get("truth", {}).get("used") is not False or adapter_summary.get("inputs", {}).get("ground_truth") is not None:
            raise OptimizerStopError("adapter was not truth-free")
        spp_dir = temporary / "spp"
        spp_run = run_child(spp_command(adapter_dir / "rover.obs", inputs / "brdc.nav", spp_dir), spp_dir, "spp")
        if spp_run["return_code"] != 0 or spp_run["timed_out"]:
            raise OptimizerStopError("truth-free SPP failed")
        spp_report = validate_spp(spp_dir / "libgnsspp_spp.pos")
        lanes: dict[str, dict[str, Any]] = {}
        for lane, max_iterations, relative_threshold, absolute_threshold in STOP_POLICIES:
            lane_dir = temporary / lane
            fgo_run = run_child(fgo_command(adapter_dir / "rover.obs", inputs / "brdc.nav", spp_dir / "libgnsspp_spp.pos", lane_dir, max_iterations, relative_threshold, absolute_threshold), lane_dir, "fgo")
            if fgo_run["return_code"] != 0 or fgo_run["timed_out"]:
                raise OptimizerStopError(f"{lane} FGO failed")
            lanes[lane] = {"run": fgo_run, "validation": validate_fgo(lane_dir, max_iterations, relative_threshold, absolute_threshold)}
        route_manifest = {
            "schema_version": ROUTE_SCHEMA,
            "status": "truth-free-artifacts-sealed",
            "candidate_id": freeze["candidate_id"],
            "dataset_id": dataset_id,
            "role": "train",
            "truth_opened": False,
            "truth_materialized": False,
            "archive_sha256": ARCHIVE_SHA256,
            "central_metadata": verified,
            "materialization_manifest": artifact(temporary / "materialization_manifest.json"),
            "adapter": {"run": adapter_run, "summary": adapter_summary, "artifacts": {name: artifact(adapter_dir / name) for name in ("rover.obs", "observations.csv", "receiver_wls.csv", "reference.csv", "summary.json")}},
            "spp": {"run": spp_run, "validation": spp_report, "artifacts": {name: artifact(spp_dir / name) for name in ("libgnsspp_spp.pos", "libgnsspp_spp_summary.json", "libgnsspp_spp_clock.csv", "libgnsspp_spp_timing.csv")}},
            "lanes": {
                lane: {"run": value["run"], "summary": value["validation"]["summary"], "cost_trace": value["validation"]["cost_trace"], "artifacts": value["validation"]["artifacts"]}
                for lane, value in lanes.items()
            },
            "algorithm_contract": "native-FGO v1 graph and all numeric parameters unchanged; only optimizer stopping differs",
            "truth_free": True,
            "runtime": {"wall_seconds": time.perf_counter() - route_started, "child_max_rss_kib": max(adapter_run["child_max_rss_kib"], spp_run["child_max_rss_kib"], *(value["run"]["child_max_rss_kib"] for value in lanes.values()))},
            "source_hashes": {**FGO_SOURCE_HASHES, "fgo_binary": FGO_BINARY_SHA256, "spp_binary": SPP_BINARY_SHA256, "profile": PROFILE_SHA256, "freeze_record": sha256(FREEZE), "freeze_manifest": sha256(FREEZE_MANIFEST)},
            "atomic_publish": True,
        }
        route_manifest = _rewrite_published_paths(
            route_manifest, temporary, route_root
        )
        atomic_json(temporary / "route_manifest.json", route_manifest)
        route_hash = sha256(temporary / "route_manifest.json")
        atomic_bytes(temporary / "route_manifest.sha256", f"{route_hash}  route_manifest.json\n".encode("ascii"))
        os.replace(temporary, route_root)
        route_manifest["route_manifest_sha256"] = route_hash
        return route_manifest
    except Exception:
        if temporary.exists():
            failure_path = output_root / "failures" / f"{safe_id(dataset_id)}.json"
            failure_path.parent.mkdir(parents=True, exist_ok=True)
            atomic_json(failure_path, {"schema_version": "smartphone-r5-native-fgo-optimizer-stop-failure.v1", "status": "failed-before-publish", "dataset_id": dataset_id, "truth_opened": False, "reason": "truth-free stage failed; no candidate route was published"})
            shutil.rmtree(temporary, ignore_errors=True)
        raise


def verify_route_manifest(path: Path, dataset_id: str) -> dict[str, Any]:
    manifest = load_json(path, "optimizer-stop route manifest")
    if manifest.get("schema_version") != ROUTE_SCHEMA or manifest.get("status") != "truth-free-artifacts-sealed" or manifest.get("dataset_id") != dataset_id or manifest.get("truth_opened") is not False or manifest.get("truth_materialized") is not False:
        raise OptimizerStopError(f"route manifest contract mismatch: {dataset_id}")
    seal_path = path.with_name("route_manifest.sha256")
    seal = seal_path.read_text(encoding="ascii").split()[0]
    if seal != sha256(path):
        raise OptimizerStopError(f"route manifest seal mismatch: {dataset_id}")
    for lane in ("baseline8", "candidate50"):
        for item in manifest["lanes"][lane]["artifacts"].values():
            if sha256(ROOT / item["path"]) != item["sha256"]:
                raise OptimizerStopError(f"route artifact hash mismatch: {dataset_id}/{lane}")
    return manifest


def publish_truth_free(freeze: dict[str, Any], index: dict[str, dict[str, Any]], output_root: Path) -> dict[str, Any]:
    output_root.mkdir(parents=True, exist_ok=True)
    if (output_root / "truth_free_manifest.json").exists():
        raise OptimizerStopError("truth-free run already exists; refusing rerun")
    (output_root / "train").mkdir(parents=True, exist_ok=True)
    route_reports = [run_route(freeze, index, dataset_id, output_root) for dataset_id in TRAIN_IDS]
    manifest = {
        "schema_version": RUN_SCHEMA,
        "status": "truth-free-train-artifacts-sealed",
        "candidate_id": freeze["candidate_id"],
        "train": list(TRAIN_IDS),
        "fresh_validation": FRESH_VALIDATION_ID,
        "future_holdout": FUTURE_HOLDOUT_ID,
        "truth_open_count": 0,
        "fresh_validation_truth_open_count": 0,
        "future_holdout_truth_open_count": 0,
        "route_manifest_sha256": {row["dataset_id"]: sha256(output_root / "train" / safe_id(row["dataset_id"]) / "route_manifest.json") for row in route_reports},
        "truth_free": True,
        "archive_sha256": ARCHIVE_SHA256,
        "freeze_record_sha256": sha256(FREEZE),
        "freeze_manifest_sha256": sha256(FREEZE_MANIFEST),
        "candidate_source_hashes": FGO_SOURCE_HASHES,
        "fgo_binary_sha256": FGO_BINARY_SHA256,
        "spp_binary_sha256": SPP_BINARY_SHA256,
        "atomic_publish": True,
        "validation_or_holdout_materialized": False,
        "external_mutation": False,
    }
    atomic_json(output_root / "truth_free_manifest.json", manifest)
    manifest["truth_free_manifest_sha256"] = sha256(output_root / "truth_free_manifest.json")
    return manifest


def repair_truth_free_manifests(output_root: Path) -> dict[str, Any]:
    """Normalize a previously published run after the atomic-path bug.

    Only route/top-level JSON and the external route seals are rewritten.  No
    archive member, truth file, solver, or child process is accessed.
    """

    top_path = output_root / "truth_free_manifest.json"
    top = load_json(top_path, "truth-free run manifest")
    if top.get("status") != "truth-free-train-artifacts-sealed" or top.get("truth_open_count") != 0:
        raise OptimizerStopError("run is not an unopened truth-free optimizer run")
    repaired_routes = {
        dataset_id: repair_published_route(
            output_root / "train" / safe_id(dataset_id), dataset_id
        )
        for dataset_id in TRAIN_IDS
    }
    top["route_manifest_sha256"] = {
        dataset_id: item["route_manifest_sha256"]
        for dataset_id, item in repaired_routes.items()
    }
    top["manifest_recovery"] = {
        "status": "completed",
        "scope": "temporary-root-paths-only",
        "solver_artifacts_rewritten": False,
        "truth_open_count": 0,
    }
    atomic_json(top_path, top)
    return {
        "status": "truth-free-manifests-repaired",
        "truth_open_count": 0,
        "routes": repaired_routes,
        "truth_free_manifest_sha256": sha256(top_path),
    }


def _score_optimizer_position(
    path: Path,
    device_path: Path,
    truth: dict[int, tuple[float, float, float]],
) -> dict[str, Any]:
    """Score all epochs emitted by the frozen ``gnss_fgo`` command.

    The native FGO command is frozen at ``--skip-epochs 0``.  The older
    evaluator's compatibility constant skips the first device epoch because
    several historical routes did not emit a valid first solution.  New
    optimizer routes can legitimately emit that epoch, so scoring accepts the
    exact device key set and fails closed on any other timestamp.
    """

    positions = smoother._read_positions(path, LEAP_SECONDS)
    epochs = smoother._read_device_epochs(device_path, 0)
    unknown = set(row.timestamp_ms for row in positions) - set(epochs)
    if unknown:
        raise OptimizerStopError(f"position output has non-device timestamps: {path}")
    metrics = smoother_eval._score_rows(
        native_eval._raw_rows(positions),
        {row.timestamp_ms: row for row in positions},
        truth,
        0,
        len(epochs),
        match_tolerance_ms=MATCH_TOLERANCE_MS,
    )
    return native_eval._normalize_score_schema(metrics)


def score_route(
    manifest: dict[str, Any],
    output_root: Path,
    index: dict[str, dict[str, Any]],
    freeze: dict[str, Any],
    dataset_id: str,
    *,
    allow_existing_truth: bool = False,
) -> dict[str, Any]:
    route_root = output_root / "train" / safe_id(dataset_id)
    selected = freeze["central_directory_selection"][dataset_id]
    truth_dir = output_root / "train_truth" / safe_id(dataset_id)
    truth_dir.mkdir(parents=True, exist_ok=True)
    truth_path = truth_dir / "ground_truth.csv"
    truth_recovery_read = False
    if truth_path.exists():
        if not allow_existing_truth:
            raise OptimizerStopError(f"train truth already materialized; refusing repeat score: {dataset_id}")
        truth_artifact = artifact(truth_path)
        truth_recovery_read = True
    else:
        truth_artifact = materialize_member(index, selected["ground_truth"], truth_path)
    truth = smoother_eval._read_truth(truth_path)
    device_path = route_root / "inputs" / "device_gnss.csv"
    baseline_path = route_root / "baseline8" / "fgo.pos"
    candidate_path = route_root / "candidate50" / "fgo.pos"
    baseline = _score_optimizer_position(baseline_path, device_path, truth)
    candidate = _score_optimizer_position(candidate_path, device_path, truth)
    baseline = native_eval._normalize_score_schema(baseline)
    candidate = native_eval._normalize_score_schema(candidate)
    route_passed, failures = native_eval._non_regression(candidate, baseline)
    baseline_summary = manifest["lanes"]["baseline8"]["summary"]
    candidate_summary = manifest["lanes"]["candidate50"]["summary"]
    convergence_improved = (
        (candidate_summary.get("converged") is True and int(candidate_summary.get("iterations", 50)) < 50)
        or float(candidate_summary.get("final_cost", math.inf)) < float(baseline_summary.get("final_cost", math.inf))
    )
    return {
        "dataset_id": dataset_id,
        "truth": {
            "artifact": truth_artifact,
            "materialized_this_run": not truth_recovery_read,
            "read_count_this_evaluation": 1,
            "recovery_read_of_previously_materialized_truth": truth_recovery_read,
        },
        "baseline8": {"metrics": baseline, "summary": baseline_summary},
        "candidate50": {"metrics": candidate, "summary": candidate_summary},
        "gate": {"route_non_regression": route_passed, "failures": failures, "convergence_improved": convergence_improved, "passed": route_passed and convergence_improved},
        "route_manifest_sha256": sha256(route_root / "route_manifest.json"),
    }


def train_score(
    freeze: dict[str, Any], output_root: Path, *, recovery: bool = False
) -> dict[str, Any]:
    truth_free_manifest = load_json(output_root / "truth_free_manifest.json", "truth-free run manifest")
    if truth_free_manifest.get("truth_open_count") != 0 or truth_free_manifest.get("future_holdout_truth_open_count") != 0:
        raise OptimizerStopError("truth-free manifest is not sealed")
    sealed = {dataset_id: verify_route_manifest(output_root / "train" / safe_id(dataset_id) / "route_manifest.json", dataset_id) for dataset_id in TRAIN_IDS}
    index = central_index()
    reports = [
        score_route(
            sealed[dataset_id],
            output_root,
            index,
            freeze,
            dataset_id,
            allow_existing_truth=recovery and position == 0,
        )
        for position, dataset_id in enumerate(TRAIN_IDS)
    ]
    baseline_metrics = [report["baseline8"]["metrics"] for report in reports]
    candidate_metrics = [report["candidate50"]["metrics"] for report in reports]
    baseline_aggregate = native_eval._aggregate(baseline_metrics)
    candidate_aggregate = native_eval._aggregate(candidate_metrics)
    aggregate_gate = native_eval._aggregate_gate(candidate_aggregate, baseline_aggregate)
    route_gate = all(report["gate"]["passed"] for report in reports)
    convergence_gate = all(report["gate"]["convergence_improved"] for report in reports)
    train_passed = route_gate and convergence_gate and aggregate_gate["passed"]
    report = {
        "schema_version": TRAIN_SCHEMA,
        "status": "train-pass" if train_passed else "no-go-train-gate",
        "candidate_id": freeze["candidate_id"],
        "truth_free_artifacts_sealed_before_truth": True,
        "truth_open_count": len(reports),
        "truth_read_count_this_evaluation": sum(
            int(report["truth"]["read_count_this_evaluation"]) for report in reports
        ),
        "truth_recovery": {
            "enabled": recovery,
            "prior_evaluator_failure_truth_open_count": 1 if recovery else 0,
            "prior_truth_file_reused_without_rematerialization": recovery,
        },
        "fresh_validation_truth_open_count": 0,
        "future_holdout_truth_open_count": 0,
        "routes": reports,
        "aggregate": {"baseline8": baseline_aggregate, "candidate50": candidate_aggregate, "gate": aggregate_gate},
        "train_gate": {"route_level_non_regression_and_convergence": route_gate, "all_routes_convergence_improved": convergence_gate, "aggregate_gate": aggregate_gate["passed"], "passed": train_passed},
        "runtime_policy": {"candidate_max_iterations": 50, "candidate_relative_cost_convergence_threshold": 1e-6, "runtime_multiplier_reported_per_route": True},
        "validation_policy": {"opened": False, "reason": "fresh validation is authorized only after every unchanged train gate passes", "fresh_validation_id": FRESH_VALIDATION_ID, "future_holdout_id": FUTURE_HOLDOUT_ID},
        "selection_policy": {"truth_free_selection": True, "leaderboard_scores_used_for_tuning": False, "old_holdout_used": False, "post_truth_tuning": False, "external_mutation": False},
        "freeze": {"record": relative(FREEZE), "record_sha256": sha256(FREEZE), "manifest": relative(FREEZE_MANIFEST), "manifest_sha256": sha256(FREEZE_MANIFEST)},
    }
    atomic_json(output_root / "train_evaluation.json", report)
    evaluation_manifest = {
        "schema_version": "smartphone-r5-gsdc2023-native-fgo-optimizer-stop-train-manifest.v1",
        "status": report["status"],
        "report": {"path": relative(output_root / "train_evaluation.json"), "sha256": sha256(output_root / "train_evaluation.json")},
        "truth_free_manifest_sha256": sha256(output_root / "truth_free_manifest.json"),
        "route_manifest_sha256": {dataset_id: report["routes"][index]["route_manifest_sha256"] for index, dataset_id in enumerate(TRAIN_IDS)},
        "truth_open_count": len(reports),
        "fresh_validation_truth_open_count": 0,
        "future_holdout_truth_open_count": 0,
        "candidate_source_hashes": FGO_SOURCE_HASHES,
        "fgo_binary_sha256": FGO_BINARY_SHA256,
        "spp_binary_sha256": SPP_BINARY_SHA256,
        "no_post_truth_tuning": True,
        "truth_recovery": {
            "enabled": recovery,
            "prior_evaluator_failure_truth_open_count": 1 if recovery else 0,
            "prior_truth_file_reused_without_rematerialization": recovery,
        },
    }
    atomic_json(output_root / "train_evaluation.manifest.json", evaluation_manifest)
    return report


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(prog="gnss_smartphone_native_fgo_optimizer_stop_eval")
    subparsers = parser.add_subparsers(dest="mode", required=True)
    run = subparsers.add_parser("truth-free-run")
    run.add_argument("--output-root", type=Path, default=DEFAULT_OUTPUT)
    score = subparsers.add_parser("train-score")
    score.add_argument("--output-root", type=Path, default=DEFAULT_OUTPUT)
    recovery = subparsers.add_parser("train-score-recovery")
    recovery.add_argument("--output-root", type=Path, default=DEFAULT_OUTPUT)
    repair = subparsers.add_parser("repair-truth-free-manifests")
    repair.add_argument("--output-root", type=Path, default=DEFAULT_OUTPUT)
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    try:
        freeze = verify_freeze()
        output_root = args.output_root if args.output_root.is_absolute() else ROOT / args.output_root
        if args.mode == "truth-free-run":
            result = publish_truth_free(freeze, central_index(), output_root)
            print(json.dumps({"status": result["status"], "train": result["train"], "truth_open_count": 0}, sort_keys=True))
            return 0
        if args.mode == "repair-truth-free-manifests":
            result = repair_truth_free_manifests(output_root)
            print(json.dumps(result, sort_keys=True))
            return 0
        result = train_score(
            freeze,
            output_root,
            recovery=args.mode == "train-score-recovery",
        )
        print(json.dumps({"status": result["status"], "train_gate": result["train_gate"], "truth_open_count": result["truth_open_count"]}, sort_keys=True))
        return 0 if result["status"] == "train-pass" else 2
    except (OptimizerStopError, OSError, ValueError, KeyError, TypeError, zipfile.BadZipFile) as exc:
        print(f"optimizer-stop error: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
