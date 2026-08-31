#!/usr/bin/env python3
"""Run and score the frozen Phase33 fresh-validation quality-anchor recipe.

The materialization and run operations open only the three declared raw
inference members.  They never read validation truth.  The score operation is
the sole truth operation: it verifies the immutable truth-free seal, extracts
the one declared ground-truth member, reads that CSV once, and scores the
already sealed control and candidate together.  No solver rerun or tuning is
possible after that read.
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
import zipfile


COMMANDS_DIR = Path(__file__).resolve().parents[1]
if str(COMMANDS_DIR) not in sys.path:
    sys.path.insert(0, str(COMMANDS_DIR))
from support.gnss_runtime import application_root  # noqa: E402

ROOT = application_root(__file__)
BENCHMARK_DIR = ROOT / "apps" / "commands" / "benchmarks"
if str(BENCHMARK_DIR) not in sys.path:
    sys.path.insert(0, str(BENCHMARK_DIR))
import gnss_smartphone_phase29_train_eval as phase29  # noqa: E402
import gnss_smartphone_phase31_quality_anchor_structural as phase31  # noqa: E402


SCHEMA = "smartphone-r5-phase33-quality-anchor-validation.v1"
MATERIALIZATION_SCHEMA = "smartphone-r5-phase33-quality-anchor-validation-raw-materialization.v1"
RUN_SCHEMA = "smartphone-r5-phase33-quality-anchor-validation-truth-free-run.v1"
SEAL_SCHEMA = "smartphone-r5-phase33-quality-anchor-validation-truth-free-seal.v1"
RESULT_SCHEMA = "smartphone-r5-phase33-quality-anchor-validation-result.v1"
RESULT_MANIFEST_SCHEMA = "smartphone-r5-phase33-quality-anchor-validation-result-manifest.v1"
FREEZE_SCHEMA = "smartphone-r5-phase33-quality-anchor-validation-freeze.v1"
FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_phase33_quality_anchor_validation_freeze_v1.json"
PHASE24_FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_phase24_route_group_split_freeze_v1.json"
ARCHIVE = ROOT / "data/gsdc2023/cache/dataset_2023.zip"
BINARY = ROOT / "build/apps/gnss_fgo_imu_no_base"
DEFAULT_OUTPUT = ROOT / "output/smartphone-r5/phase33-quality-anchor-validation-v1"
PHASE24_FREEZE_SHA256 = "a20fb65524648fab76c5494d2f28b1846e48fae821296bbdfc212cf6519e4a62"
ARCHIVE_SHA256 = "bda30ab456e6fd6f83550c246e8dbd287306d5385f1f1069c99c16298e647408"
ROUTE = "2023-05-09-21-32-us-ca-mtv-pe1/pixel5"
ROUTE_NAME, PHONE = ROUTE.split("/", 1)
RAW_KEYS = ("device_gnss", "device_imu", "broadcast_nav")
RAW_NAMES = {"device_gnss": "device_gnss.csv", "device_imu": "device_imu.csv", "broadcast_nav": "brdc.nav"}
CONTROL_FLAGS = (
    "--native-pdc-imu-tdcp-no-bridge",
    "--android-raw-clock-only",
    "--android-utc-wall-clock-fallback",
    "--android-raw-utc-keys",
)
CANDIDATE_FLAGS = CONTROL_FLAGS + ("--native-quality-anchor",)
DIAGNOSTIC_KEYS = tuple(
    f"{distance}__{percentile}"
    for distance in phase29.kaggle.DISTANCE_VARIANT_IDS
    for percentile in phase29.kaggle.PERCENTILE_VARIANT_IDS
)
MAX_SPEED_MPS = 70.0
TOLERANCE = 1e-12


class Phase33Error(ValueError):
    """Raised when the Phase33 validation contract cannot be proven."""


def sha256(path: Path) -> str:
    if not path.is_file():
        raise Phase33Error(f"missing file: {path}")
    digest = hashlib.sha256()
    try:
        with path.open("rb") as handle:
            for chunk in iter(lambda: handle.read(1024 * 1024), b""):
                digest.update(chunk)
    except OSError as exc:
        raise Phase33Error(f"failed to hash {path}: {exc}") from exc
    return digest.hexdigest()


def load_json(path: Path, label: str) -> dict[str, Any]:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise Phase33Error(f"invalid {label}: {path}") from exc
    if not isinstance(payload, dict):
        raise Phase33Error(f"{label} must be an object: {path}")
    return payload


def atomic_write(path: Path, payload: bytes) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, temporary = tempfile.mkstemp(prefix=f".{path.name}.", suffix=".tmp", dir=str(path.parent))
    try:
        with os.fdopen(fd, "wb") as handle:
            fd = -1
            handle.write(payload)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, path)
        temporary = ""
        try:
            directory = os.open(path.parent, getattr(os, "O_DIRECTORY", 0))
        except OSError:
            directory = -1
        if directory >= 0:
            try:
                os.fsync(directory)
            finally:
                os.close(directory)
    finally:
        if fd >= 0:
            os.close(fd)
        if temporary:
            try:
                os.unlink(temporary)
            except FileNotFoundError:
                pass


def atomic_json(path: Path, payload: dict[str, Any]) -> None:
    atomic_write(path, (json.dumps(payload, indent=2, sort_keys=True) + "\n").encode("utf-8"))


def _verify_phase24_metadata() -> dict[str, Any]:
    if sha256(PHASE24_FREEZE) != PHASE24_FREEZE_SHA256:
        raise Phase33Error("Phase24 split freeze hash changed")
    split = load_json(PHASE24_FREEZE, "Phase24 split freeze")
    if split.get("schema_version") != "smartphone-r5-phase24-route-group-split-freeze.v1":
        raise Phase33Error("Phase24 split freeze schema mismatch")
    metadata = split.get("selected_central_directory_metadata")
    if not isinstance(metadata, dict):
        raise Phase33Error("Phase24 central-directory metadata missing")
    route = metadata.get(ROUTE)
    if not isinstance(route, dict):
        raise Phase33Error(f"fresh validation metadata missing: {ROUTE}")
    for key in RAW_KEYS + ("ground_truth",):
        member = route.get(key)
        if not isinstance(member, dict) or not isinstance(member.get("name"), str):
            raise Phase33Error(f"fresh validation member metadata missing: {key}")
        if Path(member["name"]).suffix.lower() == ".mat":
            raise Phase33Error("MAT member rejected before archive access")
    return route


def _verify_freeze() -> dict[str, Any]:
    freeze = load_json(FREEZE, "Phase33 validation freeze")
    if freeze.get("schema_version") != FREEZE_SCHEMA or freeze.get("status") != "frozen-before-validation-raw-materialization":
        raise Phase33Error("Phase33 validation freeze schema/status mismatch")
    split = freeze.get("split")
    if not isinstance(split, dict) or split.get("fresh_validation") != ROUTE or split.get("route") != ROUTE_NAME or split.get("phone") != PHONE:
        raise Phase33Error("Phase33 validation identity mismatch")
    if split.get("future_holdout_remains_sealed") is not True:
        raise Phase33Error("future holdout is not sealed")
    train = freeze.get("train_authorization")
    if not isinstance(train, dict) or train.get("status") != "train-pass" or train.get("candidate_control_artifacts_changed") is not False or train.get("solver_rerun_for_train") is not False:
        raise Phase33Error("Phase32 train authorization is not immutable")
    if freeze.get("archive", {}).get("sha256") != ARCHIVE_SHA256:
        raise Phase33Error("archive hash pin mismatch")
    phase24 = freeze.get("archive", {}).get("phase24_split_freeze", {})
    if phase24.get("path") != str(PHASE24_FREEZE.relative_to(ROOT)) or phase24.get("sha256") != PHASE24_FREEZE_SHA256:
        raise Phase33Error("Phase24 split pin mismatch")
    recipe = freeze.get("recipe")
    if not isinstance(recipe, dict) or recipe.get("binary") != str(BINARY.relative_to(ROOT)):
        raise Phase33Error("Phase33 recipe binary mismatch")
    if recipe.get("control_flags") != list(CONTROL_FLAGS) or recipe.get("candidate_flags") != list(CANDIDATE_FLAGS):
        raise Phase33Error("Phase33 recipe flags changed")
    if recipe.get("model_and_numeric_parameters_changed") is not False or recipe.get("production_default_changed") is not False:
        raise Phase33Error("Phase33 recipe permits model/default changes")
    access = freeze.get("truth_access")
    if not isinstance(access, dict):
        raise Phase33Error("Phase33 truth access policy missing")
    for key, expected in (
        ("validation_truth_open_count_before_freeze", 0),
        ("validation_truth_materialized_before_freeze", False),
        ("phase32_train_truth_reads", 3),
        ("future_holdout_truth_open_count", 0),
        ("mat_read_or_generated", False),
        ("token_or_kaggle_access", False),
    ):
        if access.get(key) != expected:
            raise Phase33Error(f"Phase33 truth policy mismatch: {key}")
    integrity = freeze.get("integrity")
    if not isinstance(integrity, dict) or integrity.get("candidate_and_control_immutable") is not True or integrity.get("no_post_validation_tuning") is not True:
        raise Phase33Error("Phase33 integrity policy mismatch")
    source_hashes = freeze.get("source_and_binary_hashes_before_validation")
    if not isinstance(source_hashes, dict):
        raise Phase33Error("Phase33 source pins missing")
    for relative_path, expected in source_hashes.items():
        if sha256(ROOT / relative_path) != expected:
            raise Phase33Error(f"Phase33 source pin mismatch: {relative_path}")
    _verify_phase24_metadata()
    return freeze


def _copy_member(archive: zipfile.ZipFile, info: zipfile.ZipInfo, destination: Path) -> dict[str, Any]:
    if Path(info.filename).suffix.lower() == ".mat":
        raise Phase33Error("MAT archive member rejected before content open")
    destination.parent.mkdir(parents=True, exist_ok=True)
    temporary = destination.with_name(f".{destination.name}.{os.getpid()}.tmp")
    if temporary.exists():
        raise Phase33Error(f"stale temporary member: {temporary}")
    digest = hashlib.sha256()
    size = 0
    try:
        with archive.open(info, "r") as source, temporary.open("xb") as target:
            for chunk in iter(lambda: source.read(1024 * 1024), b""):
                if not chunk:
                    break
                target.write(chunk)
                digest.update(chunk)
                size += len(chunk)
            target.flush()
            os.fsync(target.fileno())
        if size != info.file_size:
            raise Phase33Error(f"member size changed: {info.filename}")
        os.replace(temporary, destination)
    finally:
        try:
            temporary.unlink()
        except FileNotFoundError:
            pass
    return {
        "member": info.filename,
        "sha256": digest.hexdigest(),
        "file_size": size,
        "compressed_size": info.compress_size,
        "crc32_hex": f"{info.CRC:08x}",
    }


def _archive_info(archive: zipfile.ZipFile, expected: dict[str, Any]) -> zipfile.ZipInfo:
    name = expected.get("name")
    if not isinstance(name, str) or Path(name).suffix.lower() == ".mat":
        raise Phase33Error("invalid or forbidden archive member")
    try:
        info = archive.getinfo(name)
    except KeyError as exc:
        raise Phase33Error(f"archive member missing: {name}") from exc
    if info.is_dir() or info.file_size != expected.get("file_size") or f"{info.CRC:08x}" != expected.get("crc32_hex"):
        raise Phase33Error(f"central metadata mismatch: {name}")
    return info


def materialize_raw(output_root: Path) -> dict[str, Any]:
    freeze = _verify_freeze()
    if sha256(ARCHIVE) != ARCHIVE_SHA256:
        raise Phase33Error("archive SHA256 mismatch")
    if output_root.exists() and any(output_root.iterdir()):
        raise Phase33Error(f"refusing to overwrite validation output: {output_root}")
    route_metadata = _verify_phase24_metadata()
    input_root = output_root / "inputs" / ROUTE_NAME / PHONE
    if input_root.exists():
        raise Phase33Error(f"refusing to overwrite validation inputs: {input_root}")
    staged = input_root.with_name(f".{input_root.name}.{os.getpid()}.staging")
    if staged.exists():
        raise Phase33Error(f"stale input staging directory: {staged}")
    staged.mkdir(parents=True)
    members: dict[str, Any] = {}
    try:
        with zipfile.ZipFile(ARCHIVE) as archive:
            for key in RAW_KEYS:
                info = _archive_info(archive, route_metadata[key])
                members[key] = _copy_member(archive, info, staged / RAW_NAMES[key])
        input_root.parent.mkdir(parents=True, exist_ok=True)
        os.replace(staged, input_root)
    finally:
        if staged.exists():
            for child in staged.iterdir():
                child.unlink()
            staged.rmdir()
    manifest = {
        "schema_version": MATERIALIZATION_SCHEMA,
        "status": "raw-materialized-truth-unopened",
        "freeze": {"path": str(FREEZE.relative_to(ROOT)), "sha256": sha256(FREEZE)},
        "archive": {"path": str(ARCHIVE.relative_to(ROOT)), "sha256": ARCHIVE_SHA256},
        "dataset_id": ROUTE,
        "role": "fresh_validation",
        "inputs": {
            key: {"path": str((input_root / RAW_NAMES[key]).relative_to(ROOT)), **value}
            for key, value in members.items()
        },
        "members_opened": list(RAW_KEYS),
        "truth_open_count": 0,
        "truth_materialized": False,
        "mat_read_or_generated": False,
        "validation_truth_remains_sealed": True,
        "future_holdout_truth_remains_sealed": True,
    }
    atomic_json(input_root / "materialization_manifest.json", manifest)
    report = {
        "schema_version": SCHEMA,
        "operation": "materialize-validation-raw-only",
        "status": "raw-materialized-truth-unopened",
        "freeze": {"path": str(FREEZE.relative_to(ROOT)), "sha256": sha256(FREEZE)},
        "dataset_id": ROUTE,
        "input_root": str(input_root.relative_to(ROOT)),
        "materialization_manifest": str((input_root / "materialization_manifest.json").relative_to(ROOT)),
        "materialization_manifest_sha256": sha256(input_root / "materialization_manifest.json"),
        "members": members,
        "truth_open_count": 0,
        "mat_read_or_generated": False,
    }
    atomic_json(output_root / "raw_materialization.json", report)
    return report


def _raw_input_paths(output_root: Path) -> dict[str, Path]:
    root = output_root / "inputs" / ROUTE_NAME / PHONE
    paths = {key: root / RAW_NAMES[key] for key in RAW_KEYS}
    for key, path in paths.items():
        if not path.is_file() or path.suffix.lower() == ".mat":
            raise Phase33Error(f"missing/forbidden raw input {key}: {path}")
    return paths


def _raw_target_keys(path: Path) -> list[int]:
    keys = phase31.read_raw_epoch_keys(path)
    if len(keys) < 2:
        raise Phase33Error("fresh validation raw input has no post-warmup target")
    return keys[1:]


def _summary_contract(summary: dict[str, Any], *, candidate: bool) -> dict[str, Any]:
    if summary.get("dataset_id") != ROUTE or summary.get("truth_used") is not False or summary.get("production_default_changed") is not False:
        raise Phase33Error("validation summary identity/truth/default contract failed")
    if summary.get("native_pdc_state_bridge") is not False:
        raise Phase33Error("PDC state bridge is enabled")
    if candidate:
        return phase31.validate_summary(summary, ROUTE)
    graph = summary.get("graph")
    epochs = summary.get("epochs")
    contract = summary.get("raw_utc_key_contract")
    if not isinstance(graph, dict) or not isinstance(epochs, dict) or not isinstance(contract, dict):
        raise Phase33Error("control summary diagnostics missing")
    if graph.get("converged") is not True or not math.isfinite(float(graph.get("initial_cost", math.nan))) or not math.isfinite(float(graph.get("final_cost", math.nan))):
        raise Phase33Error("control graph did not converge finitely")
    if float(graph["final_cost"]) > float(graph["initial_cost"]) + 1e-9:
        raise Phase33Error("control graph cost increased")
    if int(epochs.get("pseudorange_factors", 0)) <= 0 or int(epochs.get("tdcp_factors_built", 0)) <= 0 or int(graph.get("imu_intervals", 0)) <= 0:
        raise Phase33Error("control required factors absent")
    if int(contract.get("unresolved_epochs", -1)) != 0:
        raise Phase33Error("control has unresolved output epochs")
    return {
        "graph": {key: graph.get(key) for key in ("factors", "values", "imu_intervals", "iterations", "converged", "initial_cost", "final_cost")},
        "epochs": {key: epochs.get(key) for key in ("problem", "output", "pseudorange_factors", "tdcp_factors_built")},
        "raw_utc_key_contract": contract,
    }


def _validate_run_artifact(run_dir: Path, target_keys: list[int], *, candidate: bool) -> dict[str, Any]:
    submission = run_dir / "submission.csv"
    summary_path = run_dir / "summary.json"
    if not submission.is_file() or not summary_path.is_file():
        raise Phase33Error(f"run did not publish both outputs: {run_dir}")
    rows = phase31.read_prediction(submission, ROUTE)
    if [row[0] for row in rows] != target_keys:
        raise Phase33Error(f"validation key contract mismatch: {run_dir}")
    summary = load_json(summary_path, "native validation summary")
    projection = _summary_contract(summary, candidate=candidate)
    return {
        "submission": {"path": str(submission.relative_to(ROOT)), "sha256": sha256(submission), "bytes": submission.stat().st_size, "rows": len(rows)},
        "summary": {"path": str(summary_path.relative_to(ROOT)), "sha256": sha256(summary_path), "bytes": summary_path.stat().st_size},
        "projection": projection,
        "speed": phase31.speed_report(rows),
    }


def _run_one(output_root: Path, lane: str, flags: tuple[str, ...], run_number: int, target_keys: list[int], *, candidate: bool) -> dict[str, Any]:
    paths = _raw_input_paths(output_root)
    run_dir = output_root / lane / f"run{run_number}"
    if run_dir.exists():
        raise Phase33Error(f"refusing to overwrite validation run: {run_dir}")
    run_dir.mkdir(parents=True)
    submission = run_dir / "submission.csv"
    summary = run_dir / "summary.json"
    log = run_dir / "run.log"
    command = [
        str(BINARY),
        "--android-gnss", str(paths["device_gnss"]),
        "--android-imu", str(paths["device_imu"]),
        "--nav", str(paths["broadcast_nav"]),
        "--out", str(submission),
        "--summary-json", str(summary),
        "--dataset-id", ROUTE,
        "--all-epochs",
        *flags,
    ]
    env = os.environ.copy()
    local_lib = "/home/sasaki/.local/lib"
    existing_lib = env.get("LD_LIBRARY_PATH", "")
    env["LD_LIBRARY_PATH"] = local_lib + ((":" + existing_lib) if existing_lib else "")
    started = time.perf_counter()
    result = subprocess.run(command, cwd=ROOT, env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, check=False)
    wall = time.perf_counter() - started
    atomic_write(log, result.stdout.encode("utf-8"))
    if result.returncode != 0:
        raise Phase33Error(f"{lane} run{run_number} failed with exit {result.returncode}")
    artifact = _validate_run_artifact(run_dir, target_keys, candidate=candidate)
    return {
        "status": "truth-free-complete",
        "lane": lane,
        "run_number": run_number,
        "flags": list(flags),
        "command": command,
        "return_code": result.returncode,
        "wall_seconds": wall,
        "max_rss_kb_process": resource.getrusage(resource.RUSAGE_CHILDREN).ru_maxrss,
        "log": {"path": str(log.relative_to(ROOT)), "sha256": sha256(log), "bytes": log.stat().st_size},
        **artifact,
        "truth_open_count": 0,
        "mat_read_or_generated": False,
    }


def run_truth_free(output_root: Path) -> dict[str, Any]:
    freeze = _verify_freeze()
    materialization = load_json(output_root / "raw_materialization.json", "raw materialization")
    if materialization.get("schema_version") != SCHEMA or materialization.get("truth_open_count") != 0 or materialization.get("mat_read_or_generated") is not False:
        raise Phase33Error("raw materialization is not truth-free")
    target_keys = _raw_target_keys(_raw_input_paths(output_root)["device_gnss"])
    report: dict[str, Any] = {
        "schema_version": RUN_SCHEMA,
        "status": "truth-free-complete",
        "freeze": {"path": str(FREEZE.relative_to(ROOT)), "sha256": sha256(FREEZE)},
        "dataset_id": ROUTE,
        "target_epoch_count": len(target_keys),
        "candidate_control_mutation": False,
        "solver_rerun_after_truth": False,
        "truth_open_count": 0,
        "mat_read_or_generated": False,
        "lanes": {},
    }
    for lane, flags, candidate in (("control", CONTROL_FLAGS, False), ("candidate", CANDIDATE_FLAGS, True)):
        first = _run_one(output_root, lane, flags, 1, target_keys, candidate=candidate)
        repeat = _run_one(output_root, lane, flags, 2, target_keys, candidate=candidate)
        if first["submission"]["sha256"] != repeat["submission"]["sha256"] or first["summary"]["sha256"] != repeat["summary"]["sha256"]:
            raise Phase33Error(f"{lane} repeat is not byte-identical")
        report["lanes"][lane] = {
            "first": first,
            "repeat": repeat,
            "repeat_byte_identical": True,
        }
    atomic_json(output_root / "truth_free_run.json", report)
    return report


def seal_truth_free(output_root: Path) -> dict[str, Any]:
    freeze = _verify_freeze()
    report = load_json(output_root / "truth_free_run.json", "truth-free run report")
    if report.get("schema_version") != RUN_SCHEMA or report.get("truth_open_count") != 0 or report.get("mat_read_or_generated") is not False:
        raise Phase33Error("truth-free run report contract failed")
    target_keys = _raw_target_keys(_raw_input_paths(output_root)["device_gnss"])
    lanes = report.get("lanes")
    if not isinstance(lanes, dict) or set(lanes) != {"control", "candidate"}:
        raise Phase33Error("truth-free lanes are incomplete")
    sealed: dict[str, Any] = {}
    for lane, candidate in (("control", False), ("candidate", True)):
        row = lanes[lane]
        if row.get("repeat_byte_identical") is not True:
            raise Phase33Error(f"{lane} repeat was not sealed")
        first = row.get("first")
        if not isinstance(first, dict):
            raise Phase33Error(f"{lane} first run missing")
        # Re-read only the sealed raw-only prediction/summary for structural
        # verification.  This is not a truth operation.
        artifact = _validate_run_artifact(ROOT / first["submission"]["path"].rsplit("/", 1)[0], target_keys, candidate=candidate)
        speed = artifact["speed"]
        if speed["over_70_mps_count"] != 0 or not speed["finite"]:
            raise Phase33Error(f"{lane} continuity gate failed")
        sealed[lane] = {
            "submission": artifact["submission"],
            "summary": artifact["summary"],
            "speed": speed,
            "projection": artifact["projection"],
            "repeat_byte_identical": True,
        }
    seal = {
        "schema_version": SEAL_SCHEMA,
        "status": "sealed-truth-free-structural-pass",
        "decision": "validation-truth-read-authorized",
        "freeze": {"path": str(FREEZE.relative_to(ROOT)), "sha256": sha256(FREEZE)},
        "raw_materialization": {"path": str((output_root / "raw_materialization.json").relative_to(ROOT)), "sha256": sha256(output_root / "raw_materialization.json")},
        "truth_free_run": {"path": str((output_root / "truth_free_run.json").relative_to(ROOT)), "sha256": sha256(output_root / "truth_free_run.json")},
        "dataset_id": ROUTE,
        "target_epoch_count": len(target_keys),
        "lanes": sealed,
        "structural_gate": {
            "control_finite_converged": True,
            "candidate_finite_converged": True,
            "exact_raw_target_keys": True,
            "repeat_byte_identical": True,
            "continuity_over_70_mps": 0,
            "truth_open_count": 0,
            "mat_read_or_generated": False,
            "validation_truth_read_authorized_next": True,
        },
        "candidate_control_unchanged": True,
        "solver_rerun_after_truth": False,
    }
    atomic_json(output_root / "truth_free_seal.json", seal)
    atomic_json(output_root / "truth_free_seal.manifest.json", {
        "schema_version": SEAL_SCHEMA + "-manifest",
        "seal": {"path": str((output_root / "truth_free_seal.json").relative_to(ROOT)), "sha256": sha256(output_root / "truth_free_seal.json")},
        "truth_open_count": 0,
        "mat_read_or_generated": False,
        "atomic_publish": True,
    })
    return seal


def _horizontal_compare(candidate: dict[str, Any], control: dict[str, Any]) -> dict[str, Any]:
    failures: list[str] = []
    for key in DIAGNOSTIC_KEYS:
        if candidate["kaggle_diagnostic_score_variants_m"][key] > control["kaggle_diagnostic_score_variants_m"][key] + TOLERANCE:
            failures.append(f"{key}_regression")
    for key in ("p50_m", "p95_m"):
        if candidate["horizontal_wgs84_m"][key] > control["horizontal_wgs84_m"][key] + TOLERANCE:
            failures.append(f"h_{key}_regression")
    for key in ("availability_ratio", "truth_coverage_ratio"):
        if candidate[key] + TOLERANCE < control[key]:
            failures.append(f"{key}_regression")
    if candidate["continuity"]["over_70_mps_count"] > control["continuity"]["over_70_mps_count"]:
        failures.append("continuity_regression")
    return {
        "passed": not failures,
        "failures": failures,
        "vertical": "informational-unavailable",
    }


def _aggregate(rows: list[dict[str, Any]]) -> dict[str, Any]:
    if not rows:
        raise Phase33Error("no lane metrics")
    return {
        "route_count": len(rows),
        "mean_availability_ratio": sum(row["availability_ratio"] for row in rows) / len(rows),
        "mean_truth_coverage_ratio": sum(row["truth_coverage_ratio"] for row in rows) / len(rows),
        "mean_horizontal_wgs84_p50_m": sum(row["horizontal_wgs84_m"]["p50_m"] for row in rows) / len(rows),
        "mean_horizontal_wgs84_p95_m": sum(row["horizontal_wgs84_m"]["p95_m"] for row in rows) / len(rows),
        "mean_kaggle_diagnostic_score_variants_m": {
            key: sum(row["kaggle_diagnostic_score_variants_m"][key] for row in rows) / len(rows)
            for key in DIAGNOSTIC_KEYS
        },
        "sum_over_70_mps_count": sum(row["continuity"]["over_70_mps_count"] for row in rows),
    }


def _aggregate_compare(candidate: dict[str, Any], control: dict[str, Any]) -> dict[str, Any]:
    failures: list[str] = []
    for key in DIAGNOSTIC_KEYS:
        if candidate["mean_kaggle_diagnostic_score_variants_m"][key] >= control["mean_kaggle_diagnostic_score_variants_m"][key] - TOLERANCE:
            failures.append(f"{key}_not_strictly_improved")
    for key in ("mean_horizontal_wgs84_p50_m", "mean_horizontal_wgs84_p95_m"):
        if candidate[key] > control[key] + TOLERANCE:
            failures.append(f"{key}_regression")
    for key in ("mean_availability_ratio", "mean_truth_coverage_ratio"):
        if candidate[key] + TOLERANCE < control[key]:
            failures.append(f"{key}_regression")
    if candidate["sum_over_70_mps_count"] > control["sum_over_70_mps_count"]:
        failures.append("continuity_regression")
    return {"passed": not failures, "failures": failures}


def _materialize_truth_after_seal(output_root: Path, route_metadata: dict[str, Any]) -> dict[str, Any]:
    truth_root = output_root / "truth" / ROUTE_NAME / PHONE
    destination = truth_root / "ground_truth.csv"
    if destination.exists():
        raise Phase33Error("validation truth was already materialized")
    truth_root.mkdir(parents=True)
    with zipfile.ZipFile(ARCHIVE) as archive:
        info = _archive_info(archive, route_metadata["ground_truth"])
        member = _copy_member(archive, info, destination)
    return {
        "path": str(destination.relative_to(ROOT)),
        **member,
        "parsed": False,
        "read_count": 0,
        "mat_read_or_generated": False,
    }


def score_once(output_root: Path) -> dict[str, Any]:
    started = time.perf_counter()
    _verify_freeze()
    seal = load_json(output_root / "truth_free_seal.json", "truth-free seal")
    if seal.get("schema_version") != SEAL_SCHEMA or seal.get("status") != "sealed-truth-free-structural-pass" or seal.get("structural_gate", {}).get("truth_open_count") != 0:
        raise Phase33Error("truth-free structural seal is not valid")
    result_path = output_root / "validation_evaluation.json"
    if result_path.exists():
        raise Phase33Error("validation evaluation already exists; one-shot rule forbids rerun")
    route_metadata = _verify_phase24_metadata()
    truth_info = _materialize_truth_after_seal(output_root, route_metadata)
    truth_path = ROOT / truth_info["path"]
    truth, truth_hash, truth_bytes = phase29._read_truth_once(truth_path, ROUTE)
    truth_info["sha256"] = truth_hash
    truth_info["file_size"] = truth_bytes
    truth_info["parsed"] = True
    truth_info["read_count"] = 1
    truth_info["expected_sha256"] = "recorded-from-materialized-member"
    truth_keys = set(truth)
    lanes: dict[str, dict[str, Any]] = {}
    for lane in ("control", "candidate"):
        first = seal["lanes"][lane]["submission"]["path"]
        rows = phase29._prediction_rows(ROOT / first, ROUTE)
        prediction_keys = {(row.phone, row.timestamp) for row in rows}
        matched = prediction_keys & truth_keys
        lanes[lane] = {
            "rows": rows,
            "prediction_keys": prediction_keys,
            "matched_keys": matched,
        }
    shared = lanes["control"]["matched_keys"] & lanes["candidate"]["matched_keys"]
    if shared != lanes["control"]["matched_keys"] or shared != lanes["candidate"]["matched_keys"]:
        raise Phase33Error("control/candidate truth intersection differs")
    metrics: dict[str, Any] = {}
    for lane in ("control", "candidate"):
        metrics[lane] = phase29._lane_metrics(lanes[lane]["rows"], truth, shared)
    route_gate = _horizontal_compare(metrics["candidate"], metrics["control"])
    aggregate_control = _aggregate([metrics["control"]])
    aggregate_candidate = _aggregate([metrics["candidate"]])
    aggregate_gate = _aggregate_compare(aggregate_candidate, aggregate_control)
    passed = route_gate["passed"] and aggregate_gate["passed"]
    report = {
        "schema_version": RESULT_SCHEMA,
        "phase": 33,
        "status": "validation-pass" if passed else "no-go-validation-gate",
        "decision": "development-only-validation-go" if passed else "no-go-no-holdout",
        "dataset_id": ROUTE,
        "freeze": {"path": str(FREEZE.relative_to(ROOT)), "sha256": sha256(FREEZE)},
        "truth_free_seal": {"path": str((output_root / "truth_free_seal.json").relative_to(ROOT)), "sha256": sha256(output_root / "truth_free_seal.json")},
        "truth": {
            "path": truth_info["path"],
            "sha256": truth_hash,
            "bytes": truth_bytes,
            "archive_member": truth_info["member"],
            "materialized_after_truth_free_seal": True,
            "read_count": 1,
        },
        "truth_open_count": 1,
        "control": metrics["control"],
        "candidate": metrics["candidate"],
        "key_set": {
            "truth_keys": len(truth_keys),
            "control_prediction_keys": len(lanes["control"]["prediction_keys"]),
            "candidate_prediction_keys": len(lanes["candidate"]["prediction_keys"]),
            "control_matched_keys": len(lanes["control"]["matched_keys"]),
            "candidate_matched_keys": len(lanes["candidate"]["matched_keys"]),
            "shared_scored_keys": len(shared),
            "same_matched_key_set": True,
        },
        "route_gate": route_gate,
        "aggregate": {"control": aggregate_control, "candidate": aggregate_candidate, "gate": aggregate_gate},
        "vertical": "informational-unavailable-four-column-prediction",
        "fresh_validation": True,
        "future_holdout_truth_open_count": 0,
        "mat_read_or_generated": False,
        "token_or_kaggle_access": False,
        "solver_rerun_after_truth": False,
        "post_score_tuning": False,
        "runtime": {"wall_seconds": time.perf_counter() - started},
    }
    atomic_json(result_path, report)
    manifest = {
        "schema_version": RESULT_MANIFEST_SCHEMA,
        "status": report["status"],
        "result": {"path": str(result_path.relative_to(ROOT)), "sha256": sha256(result_path)},
        "freeze": {"path": str(FREEZE.relative_to(ROOT)), "sha256": sha256(FREEZE)},
        "truth_free_seal": {"path": str((output_root / "truth_free_seal.json").relative_to(ROOT)), "sha256": sha256(output_root / "truth_free_seal.json")},
        "truth_open_count": 1,
        "fresh_validation_truth_open_count": 1,
        "future_holdout_truth_open_count": 0,
        "mat_read_or_generated": False,
        "token_or_kaggle_access": False,
        "post_score_tuning": False,
        "atomic_publish": True,
    }
    atomic_json(output_root / "validation_evaluation.manifest.json", manifest)
    return report


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("operation", choices=("materialize-raw", "run", "seal", "score"))
    parser.add_argument("--output-root", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args(argv)
    try:
        if args.operation == "materialize-raw":
            result = materialize_raw(args.output_root)
        elif args.operation == "run":
            result = run_truth_free(args.output_root)
        elif args.operation == "seal":
            result = seal_truth_free(args.output_root)
        else:
            result = score_once(args.output_root)
    except (OSError, Phase33Error, ValueError, zipfile.BadZipFile) as exc:
        print(f"phase33: {exc}", file=sys.stderr)
        return 2
    print(json.dumps({"status": result.get("status"), "truth_open_count": result.get("truth_open_count"), "operation": args.operation}, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
