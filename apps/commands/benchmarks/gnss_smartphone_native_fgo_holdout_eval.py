#!/usr/bin/env python3
"""Execute the single sealed native-FGO future holdout evaluation.

The command has two phases.  ``truth-free-run`` verifies the immutable
pre-payload freeze, inspects only ZIP central-directory metadata, materializes
device GNSS/navigation, and atomically publishes adapter/SPP/FGO artifacts.
``score`` verifies that seal, materializes the declared truth member, reads it
once, and writes one sealed Go/No-Go report.  There is deliberately no tuning,
rerun, fallback, or test-batch path in this module.
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

from support.gnss_runtime import application_root


ROOT = application_root(__file__)
BENCHMARK_DIR = ROOT / "apps" / "commands" / "benchmarks"
if str(BENCHMARK_DIR) not in sys.path:
    sys.path.insert(0, str(BENCHMARK_DIR))

import gnss_smartphone_kaggle as kaggle  # noqa: E402
import gnss_smartphone_native_fgo_eval as native  # noqa: E402
import gnss_smartphone_wls_multi_phone_ensemble_eval as archive  # noqa: E402
import gnss_smartphone_trajectory_smoother as smoother  # noqa: E402
import gnss_smartphone_trajectory_smoother_eval as smoother_eval  # noqa: E402


FREEZE_SCHEMA = "smartphone-r5-gsdc2023-native-fgo-holdout-freeze.v1"
FREEZE_MANIFEST_SCHEMA = (
    "smartphone-r5-gsdc2023-native-fgo-holdout-freeze-manifest.v1"
)
TRUTH_FREE_SCHEMA = "smartphone-r5-gsdc2023-native-fgo-holdout-truth-free.v1"
EVALUATION_SCHEMA = "smartphone-r5-gsdc2023-native-fgo-holdout-evaluation.v1"
EVALUATION_MANIFEST_SCHEMA = (
    "smartphone-r5-gsdc2023-native-fgo-holdout-evaluation-manifest.v1"
)
FAILURE_SCHEMA = "smartphone-r5-gsdc2023-native-fgo-holdout-failure.v1"

HOLDOUT_ID = "2023-09-06-00-01-us-ca-routen/pixel6pro"
HOLDOUT_ROUTE = "2023-09-06-00-01-us-ca-routen"
HOLDOUT_PHONE = "pixel6pro"
LEAP_SECONDS = 18
SKIP_EPOCHS = 1
MATCH_TOLERANCE_MS = 100
MAX_RUNTIME_SECONDS = 900
MAX_ADDRESS_SPACE_BYTES = 8 * 1024 * 1024 * 1024
TOLERANCE = 1e-12
DIAGNOSTIC_KEYS = tuple(native.DIAGNOSTIC_KEYS)

DEFAULT_ARCHIVE = ROOT / "data" / "gsdc2023" / "cache" / "dataset_2023.zip"
DEFAULT_INVENTORY = (
    ROOT / "output" / "smartphone-r5" / "generalization-v6" / "archive_inventory.json"
)
DEFAULT_SELECTION = (
    ROOT
    / "docs"
    / "use_cases"
    / "records"
    / "smartphone_r5_gsdc2023_native_fgo_selection.json"
)
DEFAULT_FREEZE = (
    ROOT
    / "docs"
    / "use_cases"
    / "records"
    / "smartphone_r5_gsdc2023_native_fgo_holdout_freeze.json"
)
DEFAULT_FREEZE_MANIFEST = (
    ROOT
    / "docs"
    / "use_cases"
    / "records"
    / "smartphone_r5_gsdc2023_native_fgo_holdout_freeze_manifest.json"
)
DEFAULT_OUTPUT = ROOT / "output" / "smartphone-r5" / "native-fgo-v1" / "holdout"


class HoldoutFgoError(ValueError):
    """Raised when the one-shot native-FGO holdout contract is not provable."""


def _sha256(path: Path) -> str:
    if not path.is_file():
        raise HoldoutFgoError(f"missing file: {path}")
    digest = hashlib.sha256()
    try:
        with path.open("rb") as handle:
            for chunk in iter(lambda: handle.read(1024 * 1024), b""):
                digest.update(chunk)
    except OSError as exc:
        raise HoldoutFgoError(f"failed to hash {path}: {exc}") from exc
    return digest.hexdigest()


def _resolve(value: str | Path) -> Path:
    path = Path(value)
    return path if path.is_absolute() else ROOT / path


def _relative(value: str | Path) -> Path:
    path = Path(value)
    if path.is_absolute() or ".." in path.parts:
        raise HoldoutFgoError(f"freeze path must be repository-relative: {value}")
    return ROOT / path


def _load_json(path: Path, label: str) -> dict[str, Any]:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise HoldoutFgoError(f"invalid {label}: {path}") from exc
    if not isinstance(payload, dict):
        raise HoldoutFgoError(f"{label} must be a JSON object: {path}")
    return payload


def _atomic_bytes(path: Path, content: bytes) -> None:
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
        try:
            directory_fd = os.open(path.parent, os.O_DIRECTORY)
            try:
                os.fsync(directory_fd)
            finally:
                os.close(directory_fd)
        except OSError:
            # The file replacement is still atomic on the same filesystem;
            # some test filesystems do not expose a directory fd.
            pass
    finally:
        temporary.unlink(missing_ok=True)


def _atomic_json(path: Path, payload: dict[str, Any]) -> None:
    _atomic_bytes(
        path,
        (json.dumps(payload, indent=2, sort_keys=True) + "\n").encode("utf-8"),
    )


def _artifact(path: Path, *, name: str | None = None) -> dict[str, Any]:
    if not path.is_file():
        raise HoldoutFgoError(f"missing artifact: {path}")
    return {
        "path": name if name is not None else str(path),
        "bytes": path.stat().st_size,
        "sha256": _sha256(path),
    }


def _canonical_sha256(payload: Any) -> str:
    encoded = json.dumps(
        payload, ensure_ascii=True, sort_keys=True, separators=(",", ":")
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def algorithm_parameter_hash(recipe: dict[str, Any]) -> str:
    """Hash the exact frozen FGO recipe without any truth-derived values."""

    return _canonical_sha256(recipe)


def _member_names() -> dict[str, str]:
    return archive._member_names(HOLDOUT_ID)


def _validate_selection(selection_path: Path) -> dict[str, Any]:
    """Validate the historical selection without requiring its old adapter hash."""

    selection = _load_json(selection_path, "native-FGO selection")
    if selection.get("schema_version") != (
        "smartphone-r5-gsdc2023-native-fgo-selection.v1"
    ):
        raise HoldoutFgoError("native-FGO selection schema differs")
    if selection.get("status") != "selection-frozen-before-truth":
        raise HoldoutFgoError("native-FGO selection is not frozen before truth")
    if selection.get("candidate_id") != "native_fgo_pseudorange_tdcp_motion_v1":
        raise HoldoutFgoError("native-FGO candidate differs")
    split = selection.get("frozen_split")
    if not isinstance(split, dict) or split.get("future_holdout") != HOLDOUT_ID:
        raise HoldoutFgoError("native-FGO future holdout differs")
    if split.get("fresh_validation_truth_open_count_before_train_gate") != 0:
        raise HoldoutFgoError("selection does not prove sealed validation")
    if split.get("future_holdout_truth_open_count_before_train_gate") != 0:
        raise HoldoutFgoError("selection does not prove sealed holdout")
    policy = selection.get("policy")
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
        raise HoldoutFgoError("selection policy is not closed")
    recipe = selection.get("recipe")
    if not isinstance(recipe, dict):
        raise HoldoutFgoError("selection lacks frozen recipe")
    required = {
        "backend": "eigen",
        "preset": "default",
        "max_iterations": 8,
        "pseudorange_sigma_m": 3.0,
        "tdcp_sigma_m": 0.03,
        "motion_sigma_m": 50.0,
        "clock_motion_sigma_m": 300.0,
        "pseudorange_huber_threshold_sigma": 4.0,
        "tdcp_huber_threshold_sigma": 4.0,
        "max_tdcp_gap_s": 2.0,
        "use_pseudorange_factors": True,
        "use_ordinary_tdcp_factors": True,
        "use_position_motion_factors": True,
        "use_clock_motion_factors": True,
        "use_carrier_phase_factors": False,
        "use_single_difference_doppler_factors": False,
        "use_single_difference_tdcp_factors": False,
        "use_double_difference_factors": False,
        "use_velocity_states": False,
    }
    for key, expected in required.items():
        if recipe.get(key) != expected:
            raise HoldoutFgoError(f"selection recipe differs for {key}")
    artifacts = selection.get("input_artifacts")
    if not isinstance(artifacts, list) or len(artifacts) != 3:
        raise HoldoutFgoError("selection train artifact map differs")
    return selection


def _verify_hash_map(source_hashes: Any, label: str) -> None:
    if not isinstance(source_hashes, dict) or not source_hashes:
        raise HoldoutFgoError(f"freeze lacks {label}")
    for relative, expected in source_hashes.items():
        if isinstance(expected, dict):
            expected = expected.get("sha256")
        if not isinstance(relative, str) or not isinstance(expected, str):
            raise HoldoutFgoError(f"freeze {label} shape is invalid")
        if _sha256(_relative(relative)) != expected:
            raise HoldoutFgoError(f"frozen {label} hash differs: {relative}")


def _verify_evidence(evidence: Any, label: str) -> None:
    if not isinstance(evidence, dict):
        raise HoldoutFgoError(f"freeze lacks {label} evidence")
    for key in ("path", "sha256"):
        if not isinstance(evidence.get(key), str):
            raise HoldoutFgoError(f"freeze {label} evidence is malformed")
    path = _relative(evidence["path"])
    if _sha256(path) != evidence["sha256"]:
        raise HoldoutFgoError(f"freeze {label} evidence hash differs")


def _verify_holdout_contract(freeze: dict[str, Any]) -> None:
    if freeze.get("schema_version") != FREEZE_SCHEMA:
        raise HoldoutFgoError("holdout freeze schema differs")
    if freeze.get("status") not in {
        "frozen-before-holdout-payload-access",
        "recovery-freeze-before-truth-access",
    }:
        raise HoldoutFgoError("holdout freeze is not pre-payload")
    if freeze.get("candidate_id") != "native_fgo_pseudorange_tdcp_motion_v1":
        raise HoldoutFgoError("holdout freeze candidate differs")
    holdout = freeze.get("holdout")
    if not isinstance(holdout, dict):
        raise HoldoutFgoError("holdout freeze lacks holdout identity")
    if (
        holdout.get("dataset_id") != HOLDOUT_ID
        or holdout.get("route") != HOLDOUT_ROUTE
        or holdout.get("phone") != HOLDOUT_PHONE
        or holdout.get("phone_allowlist") != [HOLDOUT_PHONE]
        or holdout.get("materialization_forbidden_before_freeze") is not True
        or holdout.get("truth_open_forbidden_before_freeze") is not True
    ):
        raise HoldoutFgoError("holdout identity or pre-freeze contract differs")
    contract = freeze.get("holdout_execution_contract")
    if not isinstance(contract, dict):
        raise HoldoutFgoError("holdout execution contract is missing")
    for key, expected in (
        ("authorized", True),
        ("truth_free_phase", True),
        ("one_shot_authorization", True),
        ("no_post_holdout_tuning", True),
        ("route", HOLDOUT_ROUTE),
        ("phone_allowlist", [HOLDOUT_PHONE]),
        ("truth_evaluation_pass_count", 1),
    ):
        if contract.get(key) != expected:
            raise HoldoutFgoError(f"holdout execution contract differs for {key}")
    if not isinstance(contract.get("algorithm_parameter_hash"), str):
        raise HoldoutFgoError("holdout algorithm parameter hash is missing")
    policy = freeze.get("policy")
    if not isinstance(policy, dict) or any(
        policy.get(key) is not expected
        for key, expected in (
            ("truth_free_before_truth", True),
            ("future_holdout_payload_materialized_before_freeze", False),
            ("future_holdout_truth_open_count_before_freeze", 0),
            ("leaderboard_scores_used_for_tuning", False),
            ("external_mutation", False),
            ("token_access", False),
            ("no_post_holdout_tuning", True),
        )
    ):
        raise HoldoutFgoError("holdout freeze policy is not closed")


def _verify_freeze(
    freeze_path: Path,
    freeze_manifest_path: Path,
    selection_path: Path,
    archive_path: Path,
    inventory_path: Path,
) -> tuple[dict[str, Any], dict[str, Any], str, str, str, str]:
    """Verify source/binary/evidence hashes and ZIP central metadata only."""

    freeze = _load_json(freeze_path, "native-FGO holdout freeze")
    _verify_holdout_contract(freeze)
    selection = _validate_selection(selection_path)
    selection_hash = _sha256(selection_path)
    selection_contract = freeze.get("selection_record")
    if (
        not isinstance(selection_contract, dict)
        or selection_contract.get("path") != str(selection_path.relative_to(ROOT))
        or selection_contract.get("sha256") != selection_hash
    ):
        raise HoldoutFgoError("holdout selection hash differs")
    recipe = selection["recipe"]
    recipe_hash = algorithm_parameter_hash(recipe)
    if freeze.get("algorithm_parameter_hash") != recipe_hash:
        raise HoldoutFgoError("holdout algorithm parameter hash differs")
    contract = freeze["holdout_execution_contract"]
    if contract.get("algorithm_parameter_hash") != recipe_hash:
        raise HoldoutFgoError("holdout execution algorithm hash differs")

    archive_contract = freeze.get("archive")
    inventory_contract = freeze.get("central_directory_inventory")
    if not isinstance(archive_contract, dict) or not isinstance(inventory_contract, dict):
        raise HoldoutFgoError("holdout freeze archive/inventory contract is missing")
    archive_hash = _sha256(archive_path)
    inventory_hash = _sha256(inventory_path)
    if archive_contract.get("path") != str(archive_path.relative_to(ROOT)):
        raise HoldoutFgoError("holdout archive path differs")
    if inventory_contract.get("path") != str(inventory_path.relative_to(ROOT)):
        raise HoldoutFgoError("holdout inventory path differs")
    if archive_contract.get("sha256") != archive_hash:
        raise HoldoutFgoError("holdout archive hash differs")
    if inventory_contract.get("sha256") != inventory_hash:
        raise HoldoutFgoError("holdout inventory hash differs")
    inventory = _load_json(inventory_path, "archive inventory")
    if inventory.get("archive", {}).get("central_directory_only") is not True:
        raise HoldoutFgoError("archive inventory is not central-directory-only")
    if inventory.get("archive", {}).get("member_content_read") is not False:
        raise HoldoutFgoError("archive inventory records payload access")

    members = freeze["holdout"].get("members")
    if not isinstance(members, dict):
        raise HoldoutFgoError("holdout freeze member metadata is missing")
    names = _member_names()
    for key in ("device_gnss", "broadcast_nav", "ground_truth"):
        expected = members.get(key)
        if not isinstance(expected, dict) or expected.get("name") != names[
            "device_gnss" if key == "device_gnss" else "ground_truth" if key == "ground_truth" else "broadcast_nav"
        ]:
            raise HoldoutFgoError(f"holdout member metadata is malformed: {key}")
        if archive._central_metadata(archive_path, expected["name"]) != expected:
            raise HoldoutFgoError(f"holdout central metadata changed: {key}")

    _verify_hash_map(freeze.get("source_files"), "source")
    release_binaries = freeze.get("release_binaries")
    if not isinstance(release_binaries, dict):
        raise HoldoutFgoError("holdout freeze release binaries are missing")
    for label, row in release_binaries.items():
        if not isinstance(row, dict) or not isinstance(row.get("path"), str) or not isinstance(row.get("sha256"), str):
            raise HoldoutFgoError(f"holdout binary hash shape is invalid: {label}")
        if _sha256(_relative(row["path"])) != row["sha256"]:
            raise HoldoutFgoError(f"holdout binary hash differs: {label}")
    evidence = freeze.get("evidence")
    if not isinstance(evidence, dict):
        raise HoldoutFgoError("holdout train/validation evidence is missing")
    for label in ("train_report", "train_manifest", "validation_report", "validation_manifest", "validation_authorization", "evaluation_record", "evaluation_manifest"):
        _verify_evidence(evidence.get(label), label)
    if freeze.get("truth_access_before_freeze") != {
        "train_truth_open_count": 3,
        "fresh_validation_truth_open_count": 1,
        "future_holdout_truth_open_count": 0,
        "future_holdout_materialized": False,
    }:
        raise HoldoutFgoError("holdout pre-freeze truth-access evidence differs")

    freeze_hash = _sha256(freeze_path)
    freeze_manifest = _load_json(freeze_manifest_path, "native-FGO holdout freeze manifest")
    if freeze_manifest.get("schema_version") != FREEZE_MANIFEST_SCHEMA:
        raise HoldoutFgoError("holdout freeze manifest schema differs")
    record_ref = freeze_manifest.get("freeze_record")
    if (
        not isinstance(record_ref, dict)
        or record_ref.get("path") != str(freeze_path.relative_to(ROOT))
        or record_ref.get("sha256") != freeze_hash
    ):
        raise HoldoutFgoError("holdout freeze manifest does not pin record hash")
    if freeze_manifest.get("archive_sha256") != archive_hash or freeze_manifest.get("inventory_sha256") != inventory_hash:
        raise HoldoutFgoError("holdout freeze manifest archive/inventory hash differs")
    if freeze_manifest.get("algorithm_parameter_hash") != recipe_hash:
        raise HoldoutFgoError("holdout freeze manifest algorithm hash differs")
    if freeze_manifest.get("truth_open_count_before_freeze") != 0 or freeze_manifest.get("payload_materialized_before_freeze") is not False:
        raise HoldoutFgoError("holdout freeze manifest is not pre-payload")
    freeze_manifest_hash = _sha256(freeze_manifest_path)
    return freeze, selection, archive_hash, inventory_hash, freeze_hash, freeze_manifest_hash


def _child_limits() -> None:
    resource.setrlimit(resource.RLIMIT_AS, (MAX_ADDRESS_SPACE_BYTES, MAX_ADDRESS_SPACE_BYTES))
    resource.setrlimit(resource.RLIMIT_CPU, (MAX_RUNTIME_SECONDS, MAX_RUNTIME_SECONDS + 1))


def _run_child(command: list[str], work_dir: Path, label: str) -> dict[str, Any]:
    started = time.perf_counter()
    stdout_path = work_dir / f"{label}.stdout.log"
    stderr_path = work_dir / f"{label}.stderr.log"
    environment = os.environ.copy()
    local_lib = "/home/sasaki/.local/lib"
    environment["LD_LIBRARY_PATH"] = (
        local_lib + os.pathsep + environment["LD_LIBRARY_PATH"]
        if environment.get("LD_LIBRARY_PATH")
        else local_lib
    )
    timed_out = False
    return_code: int | None = None
    with stdout_path.open("wb") as stdout, stderr_path.open("wb") as stderr:
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
    rss = resource.getrusage(resource.RUSAGE_CHILDREN).ru_maxrss
    result = {
        "command": command,
        "return_code": return_code,
        "timed_out": timed_out,
        "wall_seconds": runtime,
        "child_max_rss_kib": rss,
        "stdout": stdout_path.name,
        "stderr": stderr_path.name,
    }
    if timed_out or return_code != 0:
        raise HoldoutFgoError(f"{label} failed; inspect {stderr_path}")
    return result


def _materialize_truth_without_hash(
    archive_path: Path,
    member: str,
    output: Path,
    expected: dict[str, Any],
) -> dict[str, Any]:
    """Extract truth after the seal without a second content/hash read."""

    output.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{output.name}.", suffix=".tmp", dir=output.parent
    )
    os.close(descriptor)
    temporary = Path(temporary_name)
    try:
        with zipfile.ZipFile(archive_path) as source:
            info = source.getinfo(member)
            actual = {
                "name": info.filename,
                "file_size": info.file_size,
                "compressed_size": info.compress_size,
                "crc32_hex": f"{info.CRC:08x}",
            }
            if actual != expected:
                raise HoldoutFgoError("truth central metadata changed while materializing")
            with source.open(member, "r") as input_stream, temporary.open("wb") as target:
                shutil.copyfileobj(input_stream, target)
                target.flush()
                os.fsync(target.fileno())
        if temporary.stat().st_size != int(expected["file_size"]):
            raise HoldoutFgoError("materialized truth size differs from central metadata")
        os.replace(temporary, output)
    finally:
        temporary.unlink(missing_ok=True)
    return {"path": str(output), "bytes": int(expected["file_size"]), "metadata": expected}


def _publish_failure(parent: Path, payload: dict[str, Any]) -> Path:
    parent.mkdir(parents=True, exist_ok=True)
    target = parent / "holdout_failure.json"
    if target.exists():
        raise HoldoutFgoError(f"refusing to overwrite failure artifact: {target}")
    _atomic_json(target, payload)
    return target


def _truth_free_manifest_path(output_dir: Path) -> Path:
    return output_dir / "truth_free_manifest.json"


def _verify_truth_free_manifest(
    output_dir: Path,
    freeze_hash: str,
    freeze_manifest_hash: str,
) -> dict[str, Any]:
    manifest_path = _truth_free_manifest_path(output_dir)
    manifest = _load_json(manifest_path, "native-FGO holdout truth-free manifest")
    if manifest.get("schema_version") != TRUTH_FREE_SCHEMA:
        raise HoldoutFgoError("truth-free manifest schema differs")
    if manifest.get("dataset_id") != HOLDOUT_ID or manifest.get("status") != "truth-free-artifacts-sealed":
        raise HoldoutFgoError("truth-free manifest identity/status differs")
    if manifest.get("truth_opened") is not False or manifest.get("truth_materialized") is not False or manifest.get("truth_open_count") != 0:
        raise HoldoutFgoError("truth-free manifest has opened/materialized truth")
    if manifest.get("freeze_record_sha256") != freeze_hash or manifest.get("freeze_manifest_sha256") != freeze_manifest_hash:
        raise HoldoutFgoError("truth-free manifest freeze hash differs")
    if (output_dir / "inputs" / "ground_truth.csv").exists():
        raise HoldoutFgoError("truth is present in the truth-free artifact directory")
    artifacts = manifest.get("artifacts")
    if not isinstance(artifacts, dict) or not artifacts:
        raise HoldoutFgoError("truth-free manifest has no artifacts")
    for relative, expected in artifacts.items():
        if not isinstance(relative, str) or not isinstance(expected, dict) or not isinstance(expected.get("sha256"), str):
            raise HoldoutFgoError("truth-free artifact entry is malformed")
        path = output_dir / relative
        # Older pre-truth runs placed the FGO child logs below ``fgo/`` while
        # recording their basename.  Accept that already-sealed spelling for
        # recovery verification; newly published manifests use the full path.
        if not path.is_file() and relative in {"fgo.stdout.log", "fgo.stderr.log"}:
            path = output_dir / "fgo" / relative
        if not path.is_file() or _sha256(path) != expected["sha256"]:
            raise HoldoutFgoError(f"truth-free artifact hash differs: {relative}")
    return manifest


def _adapter_command(device: Path, nav: Path, adapter_dir: Path, freeze: Path) -> list[str]:
    return [
        sys.executable,
        str(ROOT / "apps" / "gnss.py"),
        "smartphone-gnss-adapter",
        "--device-gnss",
        str(device),
        "--output-dir",
        str(adapter_dir),
        "--dataset-id",
        HOLDOUT_ID,
        "--device-model",
        HOLDOUT_PHONE,
        "--source-url",
        "https://taroz.net/data/dataset_2023.zip",
        "--source-terms",
        "Google Smartphone Decimeter Challenge dataset; archive hash frozen",
        "--role",
        "holdout",
        "--skip-epochs",
        str(SKIP_EPOCHS),
        "--truth-free",
        "--experimental-galileo-e1",
        "--broadcast-nav",
        str(nav),
        "--sealed-holdout-eval-freeze",
        str(freeze),
    ]


def _spp_command(obs: Path, nav: Path, spp_dir: Path, binary: Path) -> list[str]:
    return [
        str(binary),
        "--obs",
        str(obs),
        "--nav",
        str(nav),
        "--out",
        str(spp_dir / "libgnsspp_spp.pos"),
        "--summary-json",
        str(spp_dir / "libgnsspp_spp_summary.json"),
        "--clock-csv",
        str(spp_dir / "libgnsspp_spp_clock.csv"),
        "--timing-csv",
        str(spp_dir / "libgnsspp_spp_timing.csv"),
        "--quiet",
    ]


def _validate_adapter(adapter_dir: Path) -> dict[str, Any]:
    summary = _load_json(adapter_dir / "summary.json", "holdout adapter summary")
    if summary.get("truth_free") is not True or summary.get("decision") != "truth-free-pipeline":
        raise HoldoutFgoError("holdout adapter did not remain truth-free")
    if summary.get("dataset", {}).get("id") != HOLDOUT_ID or summary.get("dataset", {}).get("role") != "holdout":
        raise HoldoutFgoError("holdout adapter identity differs")
    if summary.get("truth", {}).get("used") is not False or summary.get("inputs", {}).get("ground_truth") is not None:
        raise HoldoutFgoError("holdout adapter touched truth")
    if not isinstance(summary.get("navigation"), dict):
        raise HoldoutFgoError("holdout adapter did not validate broadcast navigation")
    required = ("observations.csv", "rover.obs", "receiver_wls.csv", "reference.csv", "summary.json")
    return {
        name: _artifact(adapter_dir / name, name=f"adapter/{name}") for name in required
    }


def _validate_spp(spp_dir: Path) -> dict[str, Any]:
    position = spp_dir / "libgnsspp_spp.pos"
    rows = smoother._read_positions(position, LEAP_SECONDS)
    if not rows or not all(
        math.isfinite(float(value))
        for row in rows
        for value in (*row.ecef, row.latitude, row.longitude, row.height)
    ):
        raise HoldoutFgoError("holdout SPP produced invalid seed positions")
    required = (
        "libgnsspp_spp.pos",
        "libgnsspp_spp_summary.json",
        "libgnsspp_spp_clock.csv",
        "libgnsspp_spp_timing.csv",
    )
    return {
        name: _artifact(spp_dir / name, name=f"spp/{name}") for name in required
    }


def _run_truth_free(
    freeze_path: Path,
    freeze_manifest_path: Path,
    selection_path: Path,
    archive_path: Path,
    inventory_path: Path,
    output_dir: Path,
    fgo_binary: Path,
    spp_binary: Path,
    materialized_input_dir: Path | None = None,
) -> dict[str, Any]:
    if output_dir.exists() and any(output_dir.iterdir()):
        raise HoldoutFgoError(f"refusing to reuse non-empty holdout output: {output_dir}")
    freeze, selection, archive_hash, inventory_hash, freeze_hash, freeze_manifest_hash = _verify_freeze(
        freeze_path, freeze_manifest_path, selection_path, archive_path, inventory_path
    )
    parent = output_dir.parent
    parent.mkdir(parents=True, exist_ok=True)
    work_dir = Path(tempfile.mkdtemp(prefix=f".{output_dir.name}.", dir=str(parent)))
    started = time.perf_counter()
    try:
        members = freeze["holdout"]["members"]
        names = _member_names()
        inputs_dir = work_dir / "inputs"
        inputs_dir.mkdir(parents=True, exist_ok=True)
        if materialized_input_dir is None:
            archive._materialize_member(
                archive_path,
                names["device_gnss"],
                inputs_dir / "device_gnss.csv",
                members["device_gnss"],
            )
            archive._materialize_member(
                archive_path,
                names["broadcast_nav"],
                inputs_dir / "brdc.nav",
                members["broadcast_nav"],
            )
        else:
            source_dir = materialized_input_dir / "inputs"
            source_device = source_dir / "device_gnss.csv"
            source_nav = source_dir / "brdc.nav"
            recovery_inputs = freeze.get("recovery_materialized_inputs")
            if not isinstance(recovery_inputs, dict):
                raise HoldoutFgoError("recovery input hashes are missing")
            for source, target, key in (
                (source_device, inputs_dir / "device_gnss.csv", "device_gnss"),
                (source_nav, inputs_dir / "brdc.nav", "broadcast_nav"),
            ):
                expected = recovery_inputs.get(key)
                if (
                    not isinstance(expected, dict)
                    or not isinstance(expected.get("sha256"), str)
                    or not source.is_file()
                    or _sha256(source) != expected["sha256"]
                    or source.stat().st_size != int(members[key]["file_size"])
                ):
                    raise HoldoutFgoError(f"reused recovery input is invalid: {key}")
                shutil.copyfile(source, target)
        if (inputs_dir / "ground_truth.csv").exists():
            raise HoldoutFgoError("truth was materialized during truth-free phase")

        adapter_dir = work_dir / "adapter"
        adapter_run = _run_child(
            _adapter_command(inputs_dir / "device_gnss.csv", inputs_dir / "brdc.nav", adapter_dir, freeze_path.resolve()),
            work_dir,
            "adapter",
        )
        adapter_artifacts = _validate_adapter(adapter_dir)
        spp_dir = work_dir / "spp"
        spp_dir.mkdir(parents=True, exist_ok=True)
        spp_run = _run_child(
            _spp_command(adapter_dir / "rover.obs", inputs_dir / "brdc.nav", spp_dir, spp_binary.resolve()),
            work_dir,
            "spp",
        )
        spp_artifacts = _validate_spp(spp_dir)

        entry = {
            "obs": str((adapter_dir / "rover.obs").resolve()),
            "nav": str((inputs_dir / "brdc.nav").resolve()),
            "seed_pos": str((spp_dir / "libgnsspp_spp.pos").resolve()),
            "device_gnss": str((inputs_dir / "device_gnss.csv").resolve()),
        }
        fgo_dir = work_dir / "fgo"
        fgo_dir.mkdir(parents=True, exist_ok=True)
        fgo_command = native._command(fgo_binary.resolve(), entry, fgo_dir)
        fgo_run = _run_child(fgo_command, fgo_dir, "fgo")
        summary = native._validate_summary(native._load_json(fgo_dir / "fgo_summary.json", "holdout FGO summary"))
        native._validate_pos(fgo_dir / "fgo.pos", int(summary["valid_solutions"]))
        fgo_names = (
            "fgo.pos",
            "fgo_summary.json",
            "fgo_epoch_debug.csv",
            "fgo_factor_debug.csv",
            "fgo_cost_trace.csv",
        )
        fgo_artifacts = {
            name: _artifact(fgo_dir / name, name=f"fgo/{name}") for name in fgo_names
        }
        artifacts: dict[str, Any] = {
            "inputs/device_gnss.csv": {
                **_artifact(inputs_dir / "device_gnss.csv", name="inputs/device_gnss.csv"),
                "member": names["device_gnss"],
            },
            "inputs/brdc.nav": {
                **_artifact(inputs_dir / "brdc.nav", name="inputs/brdc.nav"),
                "member": names["broadcast_nav"],
            },
        }
        for group in (adapter_artifacts, spp_artifacts, fgo_artifacts):
            for row in group.values():
                artifacts[row["path"]] = row
        artifacts.update(
            {
                "adapter.stdout.log": _artifact(work_dir / "adapter.stdout.log", name="adapter.stdout.log"),
                "adapter.stderr.log": _artifact(work_dir / "adapter.stderr.log", name="adapter.stderr.log"),
                "spp.stdout.log": _artifact(work_dir / "spp.stdout.log", name="spp.stdout.log"),
                "spp.stderr.log": _artifact(work_dir / "spp.stderr.log", name="spp.stderr.log"),
                "fgo/fgo.stdout.log": _artifact(fgo_dir / "fgo.stdout.log", name="fgo/fgo.stdout.log"),
                "fgo/fgo.stderr.log": _artifact(fgo_dir / "fgo.stderr.log", name="fgo/fgo.stderr.log"),
            }
        )
        route_manifest = {
            "schema_version": "smartphone-r5-gsdc2023-native-fgo-holdout-route-manifest.v1",
            "dataset_id": HOLDOUT_ID,
            "truth_free": True,
            "truth_opened": False,
            "truth_materialized": False,
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
            "adapter_run": adapter_run,
            "spp_run": spp_run,
            "fgo_run": fgo_run,
            "artifacts": artifacts,
        }
        _atomic_json(work_dir / "route_truth_free_manifest.json", route_manifest)
        artifacts["route_truth_free_manifest.json"] = _artifact(
            work_dir / "route_truth_free_manifest.json", name="route_truth_free_manifest.json"
        )
        manifest = {
            "schema_version": TRUTH_FREE_SCHEMA,
            "status": "truth-free-artifacts-sealed",
            "candidate_id": selection["candidate_id"],
            "dataset_id": HOLDOUT_ID,
            "truth_opened": False,
            "truth_materialized": False,
            "truth_open_count": 0,
            "freeze_record": str(freeze_path.relative_to(ROOT)),
            "freeze_record_sha256": freeze_hash,
            "freeze_manifest": str(freeze_manifest_path.relative_to(ROOT)),
            "freeze_manifest_sha256": freeze_manifest_hash,
            "archive_sha256": archive_hash,
            "inventory_sha256": inventory_hash,
            "selection_record_sha256": _sha256(selection_path),
            "recipe": selection["recipe"],
            "algorithm_parameter_hash": algorithm_parameter_hash(selection["recipe"]),
            "truth_member_declared_but_not_materialized": members["ground_truth"],
            "factor_coverage": route_manifest["factor_coverage"],
            "artifacts": artifacts,
            "atomic_publish": True,
            "fallback": "no FGO fallback during candidate evaluation; failure is sealed and no test batch is permitted",
            "runtime": {
                "wall_seconds": time.perf_counter() - started,
                "child_max_rss_kib": resource.getrusage(resource.RUSAGE_CHILDREN).ru_maxrss,
                "timeout_seconds": MAX_RUNTIME_SECONDS,
                "address_space_limit_bytes": MAX_ADDRESS_SPACE_BYTES,
            },
        }
        _atomic_json(_truth_free_manifest_path(work_dir), manifest)
        manifest_hash = _sha256(_truth_free_manifest_path(work_dir))
        _atomic_bytes(work_dir / "truth_free_manifest.sha256", f"{manifest_hash}  truth_free_manifest.json\n".encode("ascii"))
        if output_dir.exists():
            raise HoldoutFgoError(f"holdout output appeared during run: {output_dir}")
        os.replace(work_dir, output_dir)
        manifest["published_path"] = str(output_dir.relative_to(ROOT))
        manifest["manifest_sha256"] = _sha256(_truth_free_manifest_path(output_dir))
        return manifest
    except Exception as exc:  # noqa: BLE001 - preserve a sealed failure artifact
        if work_dir.exists():
            failure = {
                "schema_version": FAILURE_SCHEMA,
                "status": "truth-free-failed-before-truth",
                "dataset_id": HOLDOUT_ID,
                "truth_opened": False,
                "truth_materialized": False,
                "error": str(exc),
                "error_type": type(exc).__name__,
                "wall_seconds": time.perf_counter() - started,
            }
            _atomic_json(work_dir / "failure.json", failure)
            failure_target = parent / f"{output_dir.name}.failure"
            if not failure_target.exists():
                os.replace(work_dir, failure_target)
            else:
                shutil.rmtree(work_dir)
        raise


def _score_once(
    freeze_path: Path,
    freeze_manifest_path: Path,
    selection_path: Path,
    archive_path: Path,
    inventory_path: Path,
    output_dir: Path,
) -> dict[str, Any]:
    started = time.perf_counter()
    freeze, selection, archive_hash, inventory_hash, freeze_hash, freeze_manifest_hash = _verify_freeze(
        freeze_path, freeze_manifest_path, selection_path, archive_path, inventory_path
    )
    truth_free = _verify_truth_free_manifest(output_dir, freeze_hash, freeze_manifest_hash)
    evaluation_path = output_dir / "holdout_evaluation.json"
    failure_path = output_dir / "holdout_evaluation_failure.json"
    if evaluation_path.exists() or failure_path.exists():
        raise HoldoutFgoError("holdout evaluation is already sealed; rerun is forbidden")
    truth_path = output_dir / "inputs" / "ground_truth.csv"
    if truth_path.exists():
        raise HoldoutFgoError("truth was materialized before the sealed scoring phase")
    members = freeze["holdout"]["members"]
    names = _member_names()
    # This is the only truth extraction.  It is intentionally after all
    # truth-free artifacts and their hashes have been checked.
    truth_materialization = _materialize_truth_without_hash(
        archive_path, names["ground_truth"], truth_path, members["ground_truth"]
    )
    truth = smoother_eval._read_truth(truth_path)
    truth_open_count = 1
    device_path = output_dir / "inputs" / "device_gnss.csv"
    baseline = native._score_position(
        output_dir / "spp" / "libgnsspp_spp.pos", device_path, truth
    )
    candidate = native._score_position(
        output_dir / "fgo" / "fgo.pos", device_path, truth
    )
    non_regression, failures = native._non_regression(candidate, baseline)
    strict_h_p95 = candidate["horizontal_wgs84_m"]["p95_m"] < baseline["horizontal_wgs84_m"]["p95_m"] - TOLERANCE
    strict_diagnostic = candidate["kaggle_diagnostic_mean_m"] < baseline["kaggle_diagnostic_mean_m"] - TOLERANCE
    passed = non_regression and strict_h_p95 and strict_diagnostic
    report = {
        "schema_version": EVALUATION_SCHEMA,
        "status": "holdout-pass-development-only" if passed else "no-go-holdout",
        "candidate_id": selection["candidate_id"],
        "dataset_id": HOLDOUT_ID,
        "truth_free_artifacts_sealed_before_truth": True,
        "truth_open_count": truth_open_count,
        "truth_materialized": True,
        "baseline_wls": baseline,
        "candidate_native_fgo": candidate,
        "gate": {
            "route_non_regression_passed": non_regression,
            "route_non_regression_failures": failures,
            "strict_h_p95_improvement": strict_h_p95,
            "strict_four_diagnostic_mean_improvement": strict_diagnostic,
            "passed": passed,
        },
        "selected_lane": "native_fgo" if passed else "none",
        "truth": {
            "path": str(truth_path.relative_to(ROOT)),
            "member": names["ground_truth"],
            "central_directory_metadata": members["ground_truth"],
            "content_hash_intentionally_omitted_after_single_read": True,
        },
        "truth_materialization": truth_materialization,
        "truth_free_manifest_sha256": _sha256(_truth_free_manifest_path(output_dir)),
        "freeze_record_sha256": freeze_hash,
        "freeze_manifest_sha256": freeze_manifest_hash,
        "archive_sha256": archive_hash,
        "inventory_sha256": inventory_hash,
        "runtime": {
            "wall_seconds": time.perf_counter() - started,
            "child_max_rss_kib": truth_free.get("runtime", {}).get("child_max_rss_kib"),
            "scoring_process_max_rss_kib": resource.getrusage(resource.RUSAGE_SELF).ru_maxrss,
        },
        "no_post_holdout_tuning": True,
        "external_mutation": False,
        "test_batch_authorized": passed,
    }
    _atomic_json(evaluation_path, report)
    evaluation_hash = _sha256(evaluation_path)
    manifest = {
        "schema_version": EVALUATION_MANIFEST_SCHEMA,
        "status": report["status"],
        "evaluation": {"path": str(evaluation_path.relative_to(ROOT)), "sha256": evaluation_hash},
        "truth_free_manifest": {"path": str(_truth_free_manifest_path(output_dir).relative_to(ROOT)), "sha256": report["truth_free_manifest_sha256"]},
        "freeze_record_sha256": freeze_hash,
        "freeze_manifest_sha256": freeze_manifest_hash,
        "truth_open_count": truth_open_count,
        "future_holdout_truth_open_count": truth_open_count,
        "no_post_holdout_tuning": True,
        "external_mutation": False,
    }
    _atomic_json(output_dir / "holdout_evaluation.manifest.json", manifest)
    report["evaluation_sha256"] = evaluation_hash
    report["evaluation_manifest_sha256"] = _sha256(output_dir / "holdout_evaluation.manifest.json")
    return report


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(prog="gnss_smartphone_native_fgo_holdout_eval")
    subparsers = parser.add_subparsers(dest="mode", required=True)
    for name, help_text in (
        ("freeze-verify", "verify freeze and central metadata without payload access"),
        ("truth-free-run", "materialize device/nav and seal native-FGO artifacts"),
        ("score", "materialize and read truth once after the truth-free seal"),
    ):
        subparser = subparsers.add_parser(name, help=help_text)
        subparser.add_argument("--freeze-record", type=Path, default=DEFAULT_FREEZE)
        subparser.add_argument("--freeze-manifest", type=Path, default=DEFAULT_FREEZE_MANIFEST)
        subparser.add_argument("--selection-record", type=Path, default=DEFAULT_SELECTION)
        subparser.add_argument("--archive", type=Path, default=DEFAULT_ARCHIVE)
        subparser.add_argument("--inventory", type=Path, default=DEFAULT_INVENTORY)
        subparser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT)
        if name == "truth-free-run":
            subparser.add_argument("--fgo-binary", type=Path, default=ROOT / "build" / "apps" / "gnss_fgo")
            subparser.add_argument("--spp-binary", type=Path, default=ROOT / "build" / "apps" / "gnss_spp")
            subparser.add_argument(
                "--materialized-input-dir",
                type=Path,
                help="Reuse a previously sealed device/nav materialization after a pre-truth recovery failure.",
            )
    return parser.parse_args(argv)


def run(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    try:
        paths = (
            _resolve(args.freeze_record),
            _resolve(args.freeze_manifest),
            _resolve(args.selection_record),
            _resolve(args.archive),
            _resolve(args.inventory),
        )
        if args.mode == "freeze-verify":
            _verify_freeze(*paths)
            print(json.dumps({"status": "freeze-verified", "dataset_id": HOLDOUT_ID}, sort_keys=True))
            return 0
        if args.mode == "truth-free-run":
            materialized = (
                _resolve(args.materialized_input_dir)
                if args.materialized_input_dir is not None
                else None
            )
            report = _run_truth_free(
                *paths,
                _resolve(args.output_dir),
                _resolve(args.fgo_binary),
                _resolve(args.spp_binary),
                materialized,
            )
            print(json.dumps({"status": report["status"], "dataset_id": HOLDOUT_ID, "truth_open_count": 0, "factor_coverage": report["factor_coverage"]}, sort_keys=True))
            return 0
        if args.mode == "score":
            report = _score_once(*paths, _resolve(args.output_dir))
            print(json.dumps({"status": report["status"], "dataset_id": HOLDOUT_ID, "truth_open_count": report["truth_open_count"], "gate": report["gate"]}, sort_keys=True))
            return 0
    except HoldoutFgoError as exc:
        print(f"native FGO holdout evaluation failed: {exc}", file=sys.stderr)
        return 2
    return 2


if __name__ == "__main__":
    raise SystemExit(run())
