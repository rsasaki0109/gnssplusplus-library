#!/usr/bin/env python3
"""Run the one-shot, post-freeze smartphone selector holdout evaluation.

The freeze record is verified before the designated holdout is mentioned to a
ZIP member reader.  Device GNSS and broadcast navigation are then materialized
for the two truth-free lanes, the selector output is hashed, and only then is
the holdout truth member opened exactly once.  This command refuses to reuse a
non-empty output directory, and it never tunes a parameter from holdout data.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import resource
import shutil
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
import gnss_smartphone_generalization as generalization  # noqa: E402
import gnss_smartphone_kaggle as kaggle  # noqa: E402
import gnss_smartphone_reacquisition_eval as previous  # noqa: E402
import gnss_smartphone_trajectory_smoother as smoother  # noqa: E402
import gnss_smartphone_trajectory_smoother_eval as smoother_eval  # noqa: E402
import gnss_smartphone_wls as wls  # noqa: E402
import gnss_smartphone_wls_residual as residual  # noqa: E402
import gnss_smartphone_wls_residual_v2_eval as residual_v2  # noqa: E402
import gnss_smartphone_wls_stability_selector as selector  # noqa: E402
import gnss_smartphone_wls_stability_selector_eval as selector_eval  # noqa: E402


SCHEMA_VERSION = "smartphone-r5-wls-stability-selector-holdout-evaluation.v1"
V4_SCHEMA_VERSION = "smartphone-r5-wls-stability-selector-holdout-evaluation.v4"
V4_MANIFEST_SCHEMA_VERSION = "smartphone-r5-wls-stability-selector-holdout-run-manifest.v4"
V3_ALGORITHM_PARAMETER_HASH = (
    "c82fba6130c7d16c207213ab1e2ac542e02280db194949a300c68bebd412ca7e"
)
FREEZE_SCHEMA_VERSIONS = {
    "smartphone-r5-wls-stability-selector-holdout-freeze.v1",
    "smartphone-r5-wls-stability-selector-holdout-freeze.v2",
    "smartphone-r5-wls-stability-selector-holdout-freeze.v3",
    "smartphone-r5-wls-stability-selector-holdout-freeze.v4",
}
FREEZE_MANIFEST_SCHEMA_VERSIONS = {
    "smartphone-r5-wls-stability-selector-holdout-freeze-manifest.v1",
    "smartphone-r5-wls-stability-selector-holdout-freeze-manifest.v2",
    "smartphone-r5-wls-stability-selector-holdout-freeze-manifest.v3",
    "smartphone-r5-wls-stability-selector-holdout-freeze-manifest.v4",
}
HOLDOUT_ID = "2023-09-06-22-49-us-ca-routebb1/pixel7pro"
V4_HOLDOUT_ID = "2023-05-25-19-10-us-ca-sjc-be2/sm-s908b"
DEFAULT_ARCHIVE = ROOT / "data" / "gsdc2023" / "cache" / "dataset_2023.zip"
DEFAULT_PROFILE = ROOT / "configs" / "benchmarks" / "smartphone_r5_gsdc2023.json"
DEFAULT_FREEZE = (
    ROOT
    / "docs"
    / "use_cases"
    / "records"
    / "smartphone_r5_wls_stability_selector_holdout_freeze.json"
)
DEFAULT_FREEZE_MANIFEST = (
    ROOT
    / "docs"
    / "use_cases"
    / "records"
    / "smartphone_r5_wls_stability_selector_holdout_freeze_manifest.json"
)
DEFAULT_PREVIOUS_RECORD = ROOT / "docs" / "use_cases" / "records" / "smartphone_r5_holdout_run1.json"
DEFAULT_OUTPUT_DIR = ROOT / "output" / "smartphone-r5" / "wls-stability-selector-holdout-run2-v1"
SKIP_EPOCHS = 1
HATCH_WINDOW_S = 30
LEAP_SECONDS = 18
V4_RESIDUAL_CANDIDATE_ID = "median5_shift_0"
BASELINE_CONFIG = {
    "process_noise": 1.0,
    "measurement_floor_m": 1.0,
    "outlier_gate_sigma": 5.0,
    "segment_gap_s": 10.0,
}
SEGMENT_CONFIG = {
    "max_consecutive_rejects": 15,
    "max_prediction_duration_s": 15.0,
    "reject_fraction_max": None,
}
WLS_CONFIG = {
    "consistency_tolerance_m": wls.DEFAULT_CONSISTENCY_TOLERANCE_M,
    "ecef_norm_range_m": [wls.ECEF_NORM_MIN_M, wls.ECEF_NORM_MAX_M],
    "ecef_component_abs_max_m": wls.ECEF_COMPONENT_MAX_M,
}
DIAGNOSTIC_KEYS = tuple(previous._DIAGNOSTIC_KEYS)
HOLDOUT_FREEZE_HASH_KEYS = {
    "device_gnss": (
        "device_gnss_sha256_from_frozen_profile",
        "device_gnss_sha256_from_profile",
    ),
    "broadcast_nav": (
        "broadcast_nav_sha256_from_frozen_profile",
        "broadcast_nav_sha256_from_profile",
    ),
    "ground_truth": (
        "ground_truth_sha256_from_frozen_profile",
        "ground_truth_sha256_from_profile",
    ),
}


class HoldoutEvaluationError(ValueError):
    """Raised when the immutable holdout contract cannot be proven."""


def _is_v4_freeze(freeze: dict[str, Any]) -> bool:
    return freeze.get("schema_version") == (
        "smartphone-r5-wls-stability-selector-holdout-freeze.v4"
    )


def _run_holdout_id(freeze: dict[str, Any]) -> str:
    """Return the holdout fixed by the freeze, without consulting payload data."""

    if _is_v4_freeze(freeze):
        holdout = freeze.get("holdout")
        if not isinstance(holdout, dict) or holdout.get("id") != V4_HOLDOUT_ID:
            raise HoldoutEvaluationError("v4 freeze does not designate the reserved next holdout")
        return V4_HOLDOUT_ID
    return HOLDOUT_ID


def _central_contract(freeze: dict[str, Any]) -> dict[str, dict[str, Any]] | None:
    """Return the v4 central-directory contract, if one is sealed."""

    if not _is_v4_freeze(freeze):
        return None
    holdout = freeze.get("holdout")
    members = holdout.get("central_directory_members") if isinstance(holdout, dict) else None
    if not isinstance(members, dict):
        raise HoldoutEvaluationError("v4 freeze lacks the central-directory member contract")
    required = {"device_gnss", "broadcast_nav", "ground_truth"}
    if set(members) != required:
        raise HoldoutEvaluationError("v4 central-directory contract has an unexpected member set")
    normalized: dict[str, dict[str, Any]] = {}
    for key in sorted(required):
        metadata = members.get(key)
        if not isinstance(metadata, dict):
            raise HoldoutEvaluationError(f"v4 central metadata is invalid for {key}")
        for field in ("name", "file_size", "compressed_size", "crc32_hex"):
            if field not in metadata:
                raise HoldoutEvaluationError(f"v4 central metadata lacks {key}.{field}")
        if not isinstance(metadata["name"], str) or not metadata["name"]:
            raise HoldoutEvaluationError(f"v4 central member name is invalid for {key}")
        if any(int(metadata[field]) < 0 for field in ("file_size", "compressed_size")):
            raise HoldoutEvaluationError(f"v4 central member size is invalid for {key}")
        if not isinstance(metadata["crc32_hex"], str) or len(metadata["crc32_hex"]) != 8:
            raise HoldoutEvaluationError(f"v4 central CRC is invalid for {key}")
        try:
            int(metadata["crc32_hex"], 16)
        except ValueError as exc:
            raise HoldoutEvaluationError(f"v4 central CRC is invalid for {key}") from exc
        normalized[key] = dict(metadata)
    return normalized


def _verify_central_contract(
    archive_path: Path,
    member_names: dict[str, str],
    freeze: dict[str, Any],
) -> dict[str, dict[str, Any]]:
    """Verify archive central metadata before any member payload is opened."""

    required = ("device_gnss", "broadcast_nav", "ground_truth")
    actual = {key: _central_metadata(archive_path, member_names[key]) for key in required}
    expected = _central_contract(freeze)
    if expected is not None:
        if actual != expected:
            raise HoldoutEvaluationError("archive central metadata differs from v4 freeze")
    return actual


def _sha256(path: Path) -> str:
    if not path.is_file():
        raise HoldoutEvaluationError(f"missing artifact: {path}")
    digest = hashlib.sha256()
    try:
        with path.open("rb") as handle:
            for chunk in iter(lambda: handle.read(1024 * 1024), b""):
                digest.update(chunk)
    except OSError as exc:
        raise HoldoutEvaluationError(f"failed to hash {path}: {exc}") from exc
    return digest.hexdigest()


def _artifact(path: Path) -> dict[str, Any]:
    return {"path": str(path), "sha256": _sha256(path), "bytes": path.stat().st_size}


def _load_json(path: Path, label: str) -> dict[str, Any]:
    if not path.is_file():
        raise HoldoutEvaluationError(f"missing {label}: {path}")
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise HoldoutEvaluationError(f"invalid {label}: {path}") from exc
    if not isinstance(payload, dict):
        raise HoldoutEvaluationError(f"{label} must be a JSON object")
    return payload


def _atomic_json(path: Path, payload: dict[str, Any]) -> None:
    smoother._atomic_write(
        path,
        (json.dumps(payload, indent=2, sort_keys=True) + "\n").encode("utf-8"),
    )


def _safe_id(dataset_id: str) -> str:
    return dataset_id.replace("/", "__")


def _source_path(relative: str) -> Path:
    path = ROOT / relative
    if not path.is_file():
        raise HoldoutEvaluationError(f"frozen source file is missing: {path}")
    return path


def _verify_freeze(
    freeze_path: Path,
    freeze_manifest_path: Path,
    archive_path: Path,
    profile_path: Path,
) -> tuple[dict[str, Any], dict[str, Any], str, str, str]:
    """Verify only the freeze/profile/archive bytes before member access."""

    freeze = _load_json(freeze_path, "holdout freeze record")
    if freeze.get("schema_version") not in FREEZE_SCHEMA_VERSIONS:
        raise HoldoutEvaluationError("holdout freeze schema is invalid")
    if freeze.get("status") not in {
        "frozen-before-holdout-evaluation",
        "frozen-before-holdout-evaluation-v2",
        "frozen-before-holdout-evaluation-v3",
        "frozen-before-holdout-evaluation-v4",
    }:
        raise HoldoutEvaluationError("holdout freeze is not in the immutable pre-run state")
    contract = freeze.get("holdout_execution_contract")
    if not isinstance(contract, dict) or contract.get("authorized") is not True:
        raise HoldoutEvaluationError("holdout freeze does not authorize a run")
    if contract.get("no_post_holdout_tuning") is not True:
        raise HoldoutEvaluationError("holdout freeze lacks no-post-holdout-tuning declaration")
    if _is_v4_freeze(freeze):
        postprocess = freeze.get("wls_residual_postprocess")
        if not isinstance(postprocess, dict):
            raise HoldoutEvaluationError("v4 freeze lacks WLS residual postprocess contract")
        if postprocess.get("candidate_id") != V4_RESIDUAL_CANDIDATE_ID:
            raise HoldoutEvaluationError("v4 WLS residual candidate is not median5 shift0")
        if postprocess.get("candidate_search_after_freeze") is not False:
            raise HoldoutEvaluationError("v4 candidate search is not sealed")
        if postprocess.get("along_track_shift_m") != 0.0 or postprocess.get("robust_window_epochs") != 5:
            raise HoldoutEvaluationError("v4 WLS residual numeric parameters differ")
        if postprocess.get("timestamp_gap_threshold_ms") != wls.TIMESTAMP_GAP_THRESHOLD_MS:
            raise HoldoutEvaluationError("v4 WLS timestamp-gap threshold differs from source")
        parameters = freeze.get("parameters")
        comparison = freeze.get("algorithm_parameter_hash_comparison")
        if not isinstance(parameters, dict) or not isinstance(comparison, dict):
            raise HoldoutEvaluationError("v4 freeze lacks the v3 parameter contract")
        canonical_parameter_hash = hashlib.sha256(
            json.dumps(parameters, sort_keys=True, separators=(",", ":")).encode("utf-8")
        ).hexdigest()
        if canonical_parameter_hash != V3_ALGORITHM_PARAMETER_HASH:
            raise HoldoutEvaluationError("v4 v3-pipeline parameter hash differs")
        if comparison.get("v3_parameters_sha256") != V3_ALGORITHM_PARAMETER_HASH:
            raise HoldoutEvaluationError("v4 parameter comparison does not preserve v3 hash")
        gate = freeze.get("v2_1_promotion_gate_frozen_before_truth")
        if not isinstance(gate, dict):
            raise HoldoutEvaluationError("v4 freeze lacks the pre-truth v2.1 gate")
        if gate.get("selection_and_ranking_truth_free") is not True:
            raise HoldoutEvaluationError("v4 promotion gate is not truth-free")
        for section in ("diagnostics", "horizontal_p50", "vertical_p95"):
            if not isinstance(gate.get(section), dict):
                raise HoldoutEvaluationError(f"v4 promotion gate lacks {section}")
    manifest = _load_json(freeze_manifest_path, "holdout freeze manifest")
    if manifest.get("schema_version") not in FREEZE_MANIFEST_SCHEMA_VERSIONS:
        raise HoldoutEvaluationError("holdout freeze manifest schema is invalid")
    recorded_freeze_hash = dict(manifest.get("freeze_record", {})).get("sha256")
    actual_freeze_hash = _sha256(freeze_path)
    if recorded_freeze_hash != actual_freeze_hash:
        raise HoldoutEvaluationError("holdout freeze record hash is not fixed in its manifest")
    source_files = freeze.get("source_files")
    if not isinstance(source_files, dict) or not source_files:
        raise HoldoutEvaluationError("holdout freeze has no source-file hash set")
    for relative, entry in source_files.items():
        if not isinstance(entry, dict) or entry.get("sha256") != _sha256(_source_path(relative)):
            raise HoldoutEvaluationError(f"frozen source hash changed: {relative}")
    binary = freeze.get("release_build")
    if not isinstance(binary, dict):
        raise HoldoutEvaluationError("holdout freeze has no release binary")
    binary_path = ROOT / str(binary.get("binary", binary.get("path", "")))
    if binary.get("sha256") != _sha256(binary_path):
        raise HoldoutEvaluationError("Release binary hash changed since freeze")
    archive = freeze.get("archive")
    profile = freeze.get("profile")
    if not isinstance(archive, dict) or not isinstance(profile, dict):
        raise HoldoutEvaluationError("holdout freeze lacks archive/profile hashes")
    archive_hash = _sha256(archive_path)
    profile_hash = _sha256(profile_path)
    if archive.get("path") != str(Path("data/gsdc2023/cache/dataset_2023.zip")):
        raise HoldoutEvaluationError("archive path differs from frozen path")
    if archive.get("sha256") != archive_hash:
        raise HoldoutEvaluationError("archive hash differs from the holdout freeze")
    if profile.get("path") != str(Path("configs/benchmarks/smartphone_r5_gsdc2023.json")):
        raise HoldoutEvaluationError("profile path differs from frozen path")
    profile_expected_hash = profile.get("sha256", profile.get("sha256_before_holdout"))
    if profile_expected_hash != profile_hash:
        raise HoldoutEvaluationError("profile hash differs from the holdout freeze")
    profile_payload = _load_json(profile_path, "smartphone profile")
    holdout = dict(dict(profile_payload.get("datasets", {})).get("holdout", {}))
    if _is_v4_freeze(freeze):
        # The reserved next holdout has no payload hashes before this run by
        # design.  Its central-directory contract is checked immediately
        # before materialization; the profile itself remains hash-pinned.
        if _run_holdout_id(freeze) != V4_HOLDOUT_ID:
            raise HoldoutEvaluationError("v4 designated holdout differs from the contract")
        _central_contract(freeze)
    else:
        if holdout.get("id") != HOLDOUT_ID:
            raise HoldoutEvaluationError("profile holdout ID differs from the designated holdout")
        frozen_nav_hash = _frozen_holdout_hash(freeze, "broadcast_nav")
        if holdout.get("broadcast_nav_sha256") != frozen_nav_hash:
            raise HoldoutEvaluationError("profile holdout broadcast-nav hash differs from freeze")
        if freeze.get("holdout", {}).get("id") != HOLDOUT_ID:
            raise HoldoutEvaluationError("freeze designated holdout differs from the contract")
    return freeze, profile_payload, actual_freeze_hash, archive_hash, profile_hash


def _central_metadata(archive_path: Path, member: str) -> dict[str, Any]:
    """Read ZIP central-directory metadata without opening a member payload."""

    try:
        with zipfile.ZipFile(archive_path) as archive:
            entries = [info for info in archive.infolist() if info.filename == member]
    except (OSError, zipfile.BadZipFile) as exc:
        raise HoldoutEvaluationError(f"failed to inspect archive directory: {exc}") from exc
    if len(entries) != 1 or entries[0].is_dir():
        raise HoldoutEvaluationError(f"holdout member is not a unique file: {member}")
    info = entries[0]
    return {
        "name": info.filename,
        "file_size": info.file_size,
        "compressed_size": info.compress_size,
        "crc32_hex": f"{info.CRC:08x}",
    }


def _frozen_holdout_hash(freeze: dict[str, Any], key: str) -> str:
    """Map a logical member name to the v1 freeze's explicit hash key."""

    freeze_keys = HOLDOUT_FREEZE_HASH_KEYS.get(key)
    if freeze_keys is None:
        raise HoldoutEvaluationError(f"unknown frozen holdout member: {key}")
    holdout = dict(freeze.get("holdout", {}))
    for freeze_key in freeze_keys:
        value = holdout.get(freeze_key)
        if isinstance(value, str) and value:
            return value
    raise HoldoutEvaluationError(f"freeze lacks {freeze_keys[0]}")


def _materialize_member(
    archive_path: Path,
    member: str,
    output: Path,
    metadata: dict[str, Any],
    expected_hash: str | None,
) -> dict[str, Any]:
    """Atomically materialize one non-truth member after archive verification."""

    output.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(
        mode="wb", prefix=f".{output.name}.", suffix=".tmp", dir=output.parent, delete=False
    ) as temporary:
        temporary_path = Path(temporary.name)
        try:
            with zipfile.ZipFile(archive_path) as archive:
                info = archive.getinfo(member)
                actual_metadata = {
                    "name": info.filename,
                    "file_size": info.file_size,
                    "compressed_size": info.compress_size,
                    "crc32_hex": f"{info.CRC:08x}",
                }
                if actual_metadata != metadata:
                    raise HoldoutEvaluationError(
                        f"archive central metadata changed while materializing {member}"
                    )
                with archive.open(member, "r") as source:
                    shutil.copyfileobj(source, temporary)
            temporary.flush()
            os.fsync(temporary.fileno())
        except (OSError, KeyError, zipfile.BadZipFile, HoldoutEvaluationError) as exc:
            temporary_path.unlink(missing_ok=True)
            raise HoldoutEvaluationError(f"failed to materialize {member}: {exc}") from exc
    actual_size = temporary_path.stat().st_size
    actual_hash = _sha256(temporary_path)
    if actual_size != int(metadata["file_size"]) or (
        expected_hash is not None and actual_hash != expected_hash
    ):
        temporary_path.unlink(missing_ok=True)
        raise HoldoutEvaluationError(f"materialized member failed hash/size verification: {member}")
    os.replace(temporary_path, output)
    return {"member": member, "metadata": metadata, **_artifact(output)}


def _run_stage(stage: str, command: list[str], log_path: Path) -> dict[str, Any]:
    return generalization._run_stage(stage, command, log_path)


def _truth_free_pipeline(
    archive_path: Path,
    profile: dict[str, Any],
    freeze_path: Path,
    output_dir: Path,
    *,
    archive_hash: str,
    holdout_id: str = HOLDOUT_ID,
    holdout_spec: dict[str, Any] | None = None,
    residual_candidate_id: str | None = None,
) -> dict[str, Any]:
    dataset = dict(
        holdout_spec
        if holdout_spec is not None
        else dict(dict(profile.get("datasets", {})).get("holdout", {}))
    )
    route, phone = holdout_id.split("/", 1)
    members = generalization._member_names(route, phone)
    input_dir = output_dir / "materialized" / "inputs"
    input_dir.mkdir(parents=True, exist_ok=True)
    expected = {
        "device_gnss": dataset.get("device_gnss_sha256"),
        "broadcast_nav": dataset.get("broadcast_nav_sha256"),
    }
    if any(value is not None and (not isinstance(value, str) or not value) for value in expected.values()):
        raise HoldoutEvaluationError("holdout input hash contract is invalid")
    metadata = {key: _central_metadata(archive_path, members[key]) for key in expected}
    materialized = {
        key: _materialize_member(
            archive_path,
            members[key],
            input_dir / ("device_gnss.csv" if key == "device_gnss" else "brdc.nav"),
            metadata[key],
            expected[key],
        )
        for key in expected
    }
    device_path = input_dir / "device_gnss.csv"
    nav_path = input_dir / "brdc.nav"
    materialization_manifest = {
        "schema_version": "smartphone-r5-wls-stability-selector-holdout-materialization.v1",
        "truth_free": True,
        "truth_opened": False,
        "archive": {"path": str(archive_path), "sha256": archive_hash},
        "dataset_id": holdout_id,
        "members": materialized,
        "ground_truth_member": members["ground_truth"],
        "ground_truth_materialized": False,
    }
    materialization_path = output_dir / "materialized" / "materialization_manifest.json"
    _atomic_json(materialization_path, materialization_manifest)

    route_root = output_dir / "route" / _safe_id(holdout_id)
    adapter_dir = route_root / "native" / "adapter"
    spp_dir = route_root / "native" / "spp"
    adapter_dir.mkdir(parents=True, exist_ok=True)
    spp_dir.mkdir(parents=True, exist_ok=True)
    log_path = route_root / "truth-free-pipeline.log"
    cli = [sys.executable, str(ROOT / "apps" / "gnss.py")]
    source_url = str(dict(profile.get("archive", {})).get("url", "local-profile"))
    source_terms = str(dict(profile.get("archive", {})).get("source_terms", "local evaluation only"))
    adapter_command = [
        *cli,
        "smartphone-gnss-adapter",
        "--device-gnss",
        str(device_path),
        "--truth-free",
        "--output-dir",
        str(adapter_dir),
        "--dataset-id",
        holdout_id,
        "--device-model",
        phone,
        "--source-url",
        source_url,
        "--source-terms",
        source_terms,
        "--role",
        "holdout",
        "--skip-epochs",
        str(SKIP_EPOCHS),
        "--experimental-galileo-e1",
        "--experimental-galileo-e1-hatch-window-s",
        str(HATCH_WINDOW_S),
        "--broadcast-nav",
        str(nav_path),
        "--sealed-holdout-eval-freeze",
        str(freeze_path),
    ]
    stages = [_run_stage("adapter_truth_free_galileo_e1_hatch30", adapter_command, log_path)]
    obs_path = adapter_dir / "rover.obs"
    raw_native_path = spp_dir / "libgnsspp_spp.pos"
    native_summary_path = spp_dir / "libgnsspp_spp_summary.json"
    spp_command = generalization._spp_command(obs_path, nav_path, raw_native_path, native_summary_path)
    stages.append(_run_stage("spp", spp_command, log_path))
    canonical_native_path = spp_dir / "canonical.pos"
    canonicalization = generalization._canonicalize_position_timestamps(
        raw_native_path, device_path, canonical_native_path, SKIP_EPOCHS
    )
    stages.append(
        {
            "stage": "position_timestamp_canonicalization",
            "return_code": 0,
            "wall_time_s": canonicalization.get("wall_time_s"),
            "peak_rss_kb": None,
            "details": canonicalization,
        }
    )
    adapter_summary = _load_json(adapter_dir / "summary.json", "truth-free adapter summary")
    if adapter_summary.get("truth_free") is not True or adapter_summary.get("inputs", {}).get("ground_truth") is not None:
        raise HoldoutEvaluationError("adapter truth-free contract was not honored")

    wls_dir = route_root / "wls"
    wls_started = time.perf_counter()
    wls_payload = wls.extract_to_directory(
        device_path,
        wls_dir,
        skip_epochs=SKIP_EPOCHS,
        role="holdout",
        dataset_id=holdout_id,
        sealed_holdout_eval_freeze=freeze_path,
    )
    wls_wall = time.perf_counter() - wls_started

    native_dir = route_root / "native" / "stability"
    native_started = time.perf_counter()
    native_stable_path, native_report_path, native_stability, native_artifacts = (
        selector_eval._write_native_stability_outputs(
            canonical_native_path, device_path, native_dir
        )
    )
    native_stability_wall = time.perf_counter() - native_started

    selector_dir = route_root / "selector"
    selector_started = time.perf_counter()
    selector_manifest = selector.select_and_publish(
        native_stable_path,
        native_report_path,
        wls_dir / "wls.pos",
        wls_dir / "wls_manifest.json",
        device_path,
        selector_dir,
        phone=phone,
        dataset_id=holdout_id,
        skip_epochs=SKIP_EPOCHS,
    )
    selector_wall = time.perf_counter() - selector_started
    selector.load_selector_manifest(selector_dir / "selector_manifest.json")

    selector_position_path = selector_dir / "selected.pos"
    selector_submission_path = selector_dir / "submission.csv"
    selected_position_path = selector_position_path
    selected_submission_path = selector_submission_path
    residual_postprocess: dict[str, Any] = {
        "requested": residual_candidate_id is not None,
        "candidate_id": residual_candidate_id,
        "applied": False,
        "reason": "not-requested",
    }
    residual_artifacts: dict[str, Any] = {}
    if residual_candidate_id is not None:
        if selector_manifest["decision"] == "wls_raw":
            residual_root = route_root / "wls-residual-postprocess" / residual_candidate_id
            residual_position_path = residual_root / f"{residual_candidate_id}.pos"
            residual_manifest_path = residual_root / f"{residual_candidate_id}.manifest.json"
            residual_payload = residual.write_candidate(
                wls_dir / "wls.pos",
                residual_position_path,
                residual_candidate_id,
                manifest_path=residual_manifest_path,
                leap_seconds=LEAP_SECONDS,
            )
            residual_submission_path = residual_root / "submission.csv"
            residual_submission_manifest_path = residual_root / "submission.manifest.json"
            kaggle_manifest = kaggle.generate_submission(
                residual_position_path,
                residual_submission_path,
                phone,
                device_gnss_path=device_path,
                dataset_id=holdout_id,
                skip_epochs=SKIP_EPOCHS,
                manifest_path=residual_submission_manifest_path,
            )
            selected_position_path = residual_position_path
            selected_submission_path = residual_submission_path
            residual_postprocess = {
                "requested": True,
                "candidate_id": residual_candidate_id,
                "applied": True,
                "reason": "selector_selected_wls_raw",
                "candidate": residual_payload["candidate"],
                "position": _artifact(residual_position_path),
                "manifest": _artifact(residual_manifest_path),
                "submission": _artifact(residual_submission_path),
                "submission_manifest": _artifact(residual_submission_manifest_path),
                "submission_generator_manifest_schema": kaggle_manifest.get("schema_version"),
            }
            residual_artifacts = {
                "position": residual_postprocess["position"],
                "manifest": residual_postprocess["manifest"],
                "submission": residual_postprocess["submission"],
                "submission_manifest": residual_postprocess["submission_manifest"],
            }
        else:
            residual_postprocess["reason"] = "selector_selected_native_stable"

    truth_free_artifacts = {
        "materialization_manifest": _artifact(materialization_path),
        "adapter_summary": _artifact(adapter_dir / "summary.json"),
        "adapter_observations": _artifact(adapter_dir / "observations.csv"),
        "adapter_rinex": _artifact(obs_path),
        "native_raw_position": _artifact(canonical_native_path),
        "native_spp_summary": _artifact(native_summary_path),
        "native_stable_position": _artifact(native_stable_path),
        "native_stability_report": _artifact(native_report_path),
        "native_smoother_manifest": _artifact(native_dir / "smoother_manifest.json"),
        "wls_position": _artifact(wls_dir / "wls.pos"),
        "wls_manifest": _artifact(wls_dir / "wls_manifest.json"),
        "wls_summary": _artifact(wls_dir / "wls_summary.json"),
        "selector_manifest": _artifact(selector_dir / "selector_manifest.json"),
        "selector_selected_position": _artifact(selector_position_path),
        "selector_selected_submission": _artifact(selector_submission_path),
        "selector_selected_submission_manifest": _artifact(selector_dir / "submission.csv.manifest.json"),
        "selected_position": _artifact(selected_position_path),
        "selected_submission": _artifact(selected_submission_path),
        "selected_submission_manifest": (
            residual_postprocess["submission_manifest"]
            if residual_postprocess["applied"]
            else _artifact(selector_dir / "submission.csv.manifest.json")
        ),
        "pipeline_log": _artifact(log_path),
    }
    if residual_artifacts:
        truth_free_artifacts["wls_residual_postprocess"] = residual_artifacts
    truth_free_manifest = {
        "schema_version": "smartphone-r5-wls-stability-selector-holdout-truth-free-manifest.v1",
        "dataset_id": holdout_id,
        "truth_free": True,
        "truth_opened": False,
        "archive_sha256": archive_hash,
        "device_gnss": _artifact(device_path),
        "broadcast_nav": _artifact(nav_path),
        "selector_decision": selector_manifest["decision"],
        "selector_reason": selector_manifest["reason"],
        "wls_residual_postprocess": residual_postprocess,
        "artifacts": truth_free_artifacts,
        "commands": stages,
        "timing": {
            "wls_wall_s": wls_wall,
            "native_stability_wall_s": native_stability_wall,
            "selector_wall_s": selector_wall,
        },
    }
    truth_free_manifest_path = output_dir / "truth_free_manifest.json"
    _atomic_json(truth_free_manifest_path, truth_free_manifest)
    truth_free_manifest_artifact = _artifact(truth_free_manifest_path)

    return {
        "dataset_id": holdout_id,
        "phone": phone,
        "route_root": route_root,
        "device_path": device_path,
        "nav_path": nav_path,
        "truth_member": members["ground_truth"],
        "truth_free_manifest_path": truth_free_manifest_path,
        "truth_free_manifest_artifact": truth_free_manifest_artifact,
        "materialization_manifest_path": materialization_path,
        "stages": stages,
        "canonicalization": canonicalization,
        "native_raw_path": canonical_native_path,
        "native_stable_path": native_stable_path,
        "native_report_path": native_report_path,
        "native_stability": native_stability,
        "native_artifacts": native_artifacts,
        "wls_path": wls_dir / "wls.pos",
        "wls_manifest_path": wls_dir / "wls_manifest.json",
        "wls_summary_path": wls_dir / "wls_summary.json",
        "wls_payload": wls_payload,
        "selector_dir": selector_dir,
        "selector_manifest_path": selector_dir / "selector_manifest.json",
        "selector_manifest": selector_manifest,
        "selector_position_path": selector_position_path,
        "selector_submission_path": selector_submission_path,
        "selected_position_path": selected_position_path,
        "selected_submission_path": selected_submission_path,
        "wls_residual_postprocess": residual_postprocess,
        "truth_free_artifacts": truth_free_artifacts,
        "timing": {
            "wls_wall_s": wls_wall,
            "native_stability_wall_s": native_stability_wall,
            "selector_wall_s": selector_wall,
        },
    }


def _materialize_truth_once(
    archive_path: Path,
    member: str,
    output: Path,
    expected_size: int,
) -> dict[str, Any]:
    """Open the designated truth member exactly once, after selector hashing."""

    output.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(
        mode="wb", prefix=f".{output.name}.", suffix=".tmp", dir=output.parent, delete=False
    ) as temporary:
        temporary_path = Path(temporary.name)
        try:
            with zipfile.ZipFile(archive_path) as archive:
                with archive.open(member, "r") as source:
                    shutil.copyfileobj(source, temporary)
            temporary.flush()
            os.fsync(temporary.fileno())
        except (OSError, KeyError, zipfile.BadZipFile) as exc:
            temporary_path.unlink(missing_ok=True)
            raise HoldoutEvaluationError(f"failed to materialize holdout truth: {exc}") from exc
    if temporary_path.stat().st_size != expected_size:
        temporary_path.unlink(missing_ok=True)
        raise HoldoutEvaluationError("holdout truth size differs from central metadata")
    os.replace(temporary_path, output)
    return _artifact(output)


def _raw_rows(positions: list[smoother.PositionRow]) -> list[smoother.SmoothedRow]:
    return previous._raw_rows(positions)


def _score_lane(
    position_path: Path,
    epochs: list[int],
    truth: dict[int, tuple[float, float, float]],
) -> dict[str, Any]:
    positions = smoother._read_positions(position_path, LEAP_SECONDS)
    return previous._score(_raw_rows(positions), positions, epochs, truth)


def _load_previous_baseline(path: Path) -> dict[str, Any]:
    payload = _load_json(path, "previous holdout run1 record")
    if payload.get("schema_version") != "smartphone-r5-holdout-record.v1":
        raise HoldoutEvaluationError("previous holdout baseline schema is invalid")
    if payload.get("dataset_id") != HOLDOUT_ID or payload.get("run") != 1:
        raise HoldoutEvaluationError("previous holdout baseline is not the designated run1")
    return payload


def _comparison(selected: dict[str, Any], baseline: dict[str, Any]) -> dict[str, Any]:
    metrics = dict(baseline.get("metrics", {}))
    baseline_values = {
        "availability_ratio": metrics.get("solution_availability_ratio"),
        "horizontal_median_m": metrics.get("horizontal_median_m"),
        "horizontal_p95_m": metrics.get("horizontal_p95_m"),
        "vertical_p95_abs_m": metrics.get("vertical_p95_abs_m"),
    }
    current = {
        "availability_ratio": selected.get("availability_ratio"),
        "horizontal_median_m": selected.get("horizontal_wgs84_m", {}).get("p50_m"),
        "horizontal_p95_m": selected.get("horizontal_wgs84_m", {}).get("p95_m"),
        "vertical_p95_abs_m": selected.get("vertical_p95_abs_m"),
    }
    delta = {
        key: float(current[key]) - float(baseline_values[key])
        for key in current
        if current[key] is not None and baseline_values[key] is not None
    }
    return {
        "baseline_record": {"path": str(DEFAULT_PREVIOUS_RECORD), "sha256": _sha256(DEFAULT_PREVIOUS_RECORD)},
        "baseline_metrics": baseline_values,
        "selected_metrics": current,
        "selected_minus_baseline": delta,
        "horizontal_non_regression_vs_run1": (
            current["availability_ratio"] >= baseline_values["availability_ratio"]
            and current["horizontal_median_m"] <= baseline_values["horizontal_median_m"]
            and current["horizontal_p95_m"] <= baseline_values["horizontal_p95_m"]
        ),
    }


def _score_summary(route: dict[str, Any], truth: dict[int, tuple[float, float, float]]) -> dict[str, Any]:
    device_epochs = smoother._read_device_epochs(route["device_path"], SKIP_EPOCHS)
    native_raw = _score_lane(route["native_raw_path"], device_epochs, truth)
    native_stable = _score_lane(route["native_stable_path"], device_epochs, truth)
    wls_raw = _score_lane(route["wls_path"], device_epochs, truth)
    selector_raw = _score_lane(route["selector_position_path"], device_epochs, truth)
    selected = _score_lane(route["selected_position_path"], device_epochs, truth)
    residual_candidate = None
    if route["wls_residual_postprocess"]["applied"]:
        residual_candidate = selected
    return {
        "native_raw": native_raw,
        "native_stable": native_stable,
        "wls_raw": wls_raw,
        "selector_raw": selector_raw,
        "residual_candidate": residual_candidate,
        "selector": selected,
        "selected_lane": route["selector_manifest"]["decision"],
        "device_epochs": len(device_epochs),
    }


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        prog=os.environ.get("GNSS_CLI_NAME", "gnss smartphone-wls-stability-selector-holdout-eval")
    )
    parser.add_argument("--archive", type=Path, default=DEFAULT_ARCHIVE)
    parser.add_argument("--profile", type=Path, default=DEFAULT_PROFILE)
    parser.add_argument("--freeze", type=Path, default=DEFAULT_FREEZE)
    parser.add_argument("--freeze-manifest", type=Path, default=DEFAULT_FREEZE_MANIFEST)
    parser.add_argument("--previous-record", type=Path, default=DEFAULT_PREVIOUS_RECORD)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    parser.add_argument(
        "--preflight-only",
        action="store_true",
        help="verify the frozen bytes and ZIP central metadata without materializing a member",
    )
    return parser.parse_args(argv)


def run(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    started = time.perf_counter()
    rss_before = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss
    report_path: Path | None = None
    route: dict[str, Any] | None = None
    freeze_hash = None
    report_schema = SCHEMA_VERSION
    try:
        # This is deliberately the first operation involving the archive.  It
        # verifies the freeze and archive bytes without opening any member.
        freeze, profile, freeze_hash, archive_hash, profile_hash = _verify_freeze(
            args.freeze, args.freeze_manifest, args.archive, args.profile
        )
        v4 = _is_v4_freeze(freeze)
        if v4:
            report_schema = V4_SCHEMA_VERSION
        holdout_id = _run_holdout_id(freeze)
        holdout_spec = (
            dict(freeze.get("holdout", {}))
            if v4
            else dict(dict(profile.get("datasets", {})).get("holdout", {}))
        )
        if not args.preflight_only and args.output_dir.exists() and any(args.output_dir.iterdir()):
            raise HoldoutEvaluationError(
                f"refusing to reuse non-empty one-shot output: {args.output_dir}"
            )
        # Central directory metadata is the only holdout-specific operation
        # before the two truth-free input members are materialized.
        route, phone = holdout_id.split("/", 1)
        member_names = generalization._member_names(route, phone)
        holdout_meta = _verify_central_contract(args.archive, member_names, freeze)
        if not v4:
            profile_holdout = dict(dict(profile.get("datasets", {}))["holdout"])
            for key in ("device_gnss", "broadcast_nav", "ground_truth"):
                expected_size = holdout_meta[key]["file_size"]
                expected_hash = profile_holdout.get(
                    {
                        "device_gnss": "device_gnss_sha256",
                        "broadcast_nav": "broadcast_nav_sha256",
                        "ground_truth": "ground_truth_sha256",
                    }[key]
                )
                frozen = _frozen_holdout_hash(freeze, key)
                if expected_hash != frozen:
                    raise HoldoutEvaluationError(f"profile/freeze {key} hash mismatch")
                if not isinstance(expected_size, int) or expected_size <= 0:
                    raise HoldoutEvaluationError(f"invalid central size for {key}")
        else:
            if holdout_spec.get("device_model") != phone:
                raise HoldoutEvaluationError("v4 device model metadata differs from dataset ID")
            if int(holdout_spec.get("skip_epochs", SKIP_EPOCHS)) != SKIP_EPOCHS:
                raise HoldoutEvaluationError("v4 skip-epochs parameter differs from the freeze")
        if args.preflight_only:
            print(
                "Smartphone WLS stability selector v4 preflight passed: "
                f"{holdout_id} (central metadata only)"
            )
            return 0
        args.output_dir.mkdir(parents=True, exist_ok=True)

        residual_candidate_id = (
            V4_RESIDUAL_CANDIDATE_ID
            if v4
            else None
        )
        route = _truth_free_pipeline(
            args.archive,
            profile,
            args.freeze,
            args.output_dir,
            archive_hash=archive_hash,
            holdout_id=holdout_id,
            holdout_spec=holdout_spec,
            residual_candidate_id=residual_candidate_id,
        )
        # The truth-free manifest and every selector artifact are complete and
        # hash-fixed before any truth path is opened.
        truth_path = args.output_dir / "materialized" / "ground_truth.csv"
        truth_artifact = _materialize_truth_once(
            args.archive,
            member_names["ground_truth"],
            truth_path,
            holdout_meta["ground_truth"]["file_size"],
        )
        truth = smoother_eval._read_truth(truth_path)
        if not truth:
            raise HoldoutEvaluationError("holdout truth contains no rows")
        scores = _score_summary(route, truth)
        baseline_comparison = None
        if not v4:
            previous_record = _load_previous_baseline(args.previous_record)
            baseline_comparison = _comparison(scores["selector"], previous_record)
        v4_gate: dict[str, Any] | None = None
        if v4:
            v4_gate = residual_v2._v2_gate(
                scores["selector"],
                scores["selector_raw"],
                profile,
                dict(freeze["v2_1_promotion_gate_frozen_before_truth"]),
            )
            if route["selector_manifest"]["decision"] != "wls_raw":
                v4_gate["failures"].append("selector_did_not_choose_wls_branch")
                v4_gate["passed"] = False
            if not route["wls_residual_postprocess"]["applied"]:
                v4_gate["failures"].append("median5_postprocess_not_applied_to_wls_branch")
                v4_gate["passed"] = False
            v4_promotion = (
                "promote-development-only-wls-branch-postprocess"
                if v4_gate["passed"]
                else "no-go-v4-promotion-gate"
            )
        else:
            v4_promotion = "not-applicable"
        materialization_manifest_path = route["materialization_manifest_path"]
        materialization = _load_json(materialization_manifest_path, "materialization manifest")
        materialization["truth_free"] = False
        materialization["truth_opened"] = True
        materialization["ground_truth_materialized"] = True
        materialization["ground_truth"] = {
            "member": member_names["ground_truth"],
            **truth_artifact,
        }
        _atomic_json(materialization_manifest_path, materialization)
        rss_after = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss
        report_path = args.output_dir / "wls_stability_selector_holdout_run2.json"
        report = {
            "schema_version": report_schema,
            "status": "sealed-completed-one-shot",
            "run": 2,
            "dataset_id": holdout_id,
            "freeze_record": {"path": str(args.freeze), "sha256": freeze_hash},
            "freeze_manifest": _artifact(args.freeze_manifest),
            "archive": {"path": str(args.archive), "sha256": archive_hash},
            "profile": {"path": str(args.profile), "sha256_before_holdout": profile_hash},
            "holdout_central_metadata": holdout_meta,
            "truth_free_contract": {
                "native_lane": "Galileo E1 + Hatch C1C 30 s + segment stability r15/d15",
                "wls_lane": "handset WLS ECEF with fail-closed integrity",
                "selector": "all native segments stable => native stable; otherwise raw WLS",
                "device_model_used_for_decision": False,
                "truth_used_for_decision": False,
                "truth_free_artifacts_completed_before_truth_open": True,
                "selector_outputs_atomic": True,
                "truth_open_count": 1,
                "post_holdout_tuning": False,
            },
            "truth_free": {
                "manifest": route["truth_free_manifest_artifact"],
                "artifacts": route["truth_free_artifacts"],
                "selector_decision": route["selector_manifest"]["decision"],
                "selector_reason": route["selector_manifest"]["reason"],
                "native_stability": route["native_stability"],
            },
            "truth": {
                "path": str(truth_path),
                "artifact": truth_artifact,
                "parsed_once_after_selector": True,
                "rows": len(truth),
            },
            "scores": scores,
            "selected_lane": (
                f"{route['selector_manifest']['decision']}+{V4_RESIDUAL_CANDIDATE_ID}"
                if route["wls_residual_postprocess"]["applied"]
                else route["selector_manifest"]["decision"]
            ),
            "comparison_to_existing_holdout_run1": baseline_comparison,
            "v3_counterfactual": (
                {
                    "offline_same_sealed_artifacts": True,
                    "lane": "selector_raw",
                    "scores": scores["selector_raw"],
                    "truth_free_artifacts_reused": True,
                    "new_materialization": False,
                }
                if v4
                else None
            ),
            "v2_1_promotion_gate": (
                {
                    "frozen_before_truth": freeze["v2_1_promotion_gate_frozen_before_truth"],
                    "result": v4_gate,
                }
                if v4
                else None
            ),
            "wls_residual_postprocess": route["wls_residual_postprocess"],
            "timing": {
                "stage_timings": route["stages"],
                **route["timing"],
                "total_wall_s": time.perf_counter() - started,
                "process_peak_rss_kb": max(int(rss_before), int(rss_after)),
                "process_rss_unit": "kilobytes from getrusage ru_maxrss",
            },
            "promotion": {
                "decision": v4_promotion if v4 else "sealed-holdout-result-only; no post-holdout tuning permitted",
                "production_rtk_spp_default_changed": False,
                "recommended_lane_remains_development_only": True,
                "post_holdout_tuning": False,
            },
        }
        _atomic_json(report_path, report)
        manifest_path = args.output_dir / "wls_stability_selector_holdout_run2_manifest.json"
        manifest = {
            "schema_version": (
                V4_MANIFEST_SCHEMA_VERSION
                if v4
                else "smartphone-r5-wls-stability-selector-holdout-run-manifest.v1"
            ),
            "report": _artifact(report_path),
            "freeze_record": {"path": str(args.freeze), "sha256": freeze_hash},
            "truth_free_manifest": route["truth_free_manifest_artifact"],
            "truth": truth_artifact,
            "selected_lane": (
                f"{route['selector_manifest']['decision']}+{V4_RESIDUAL_CANDIDATE_ID}"
                if route["wls_residual_postprocess"]["applied"]
                else route["selector_manifest"]["decision"]
            ),
            "selected_position": _artifact(route["selected_position_path"]),
            "selected_submission": _artifact(route["selected_submission_path"]),
            "truth_open_count": 1,
            "post_holdout_tuning": False,
            "sealed": True,
        }
        _atomic_json(manifest_path, manifest)
        print(f"Smartphone WLS stability selector holdout complete: {report_path}")
        print(f"Selected lane: {route['selector_manifest']['decision']}")
        return 0
    except (
        OSError,
        ValueError,
        KeyError,
        TypeError,
        json.JSONDecodeError,
        zipfile.BadZipFile,
        generalization.GeneralizationError,
        smoother.SmootherError,
        selector.StabilitySelectorError,
        HoldoutEvaluationError,
    ) as exc:
        print(f"Smartphone WLS stability selector holdout failed: {exc}", file=sys.stderr)
        if args.output_dir.exists():
            failure_path = args.output_dir / "wls_stability_selector_holdout_run2_failure.json"
            try:
                _atomic_json(
                    failure_path,
                    {
                        "schema_version": (
                            V4_SCHEMA_VERSION if report_schema == V4_SCHEMA_VERSION else SCHEMA_VERSION
                        ),
                        "status": "sealed-failed-one-shot",
                        "dataset_id": (
                            V4_HOLDOUT_ID if report_schema == V4_SCHEMA_VERSION else HOLDOUT_ID
                        ),
                        "freeze_record": {"path": str(args.freeze), "sha256": freeze_hash},
                        "error": str(exc),
                        "post_holdout_tuning": False,
                        "truth_open_count": 0,
                        "sealed": True,
                        "timing": {
                            "total_wall_s": time.perf_counter() - started,
                            "process_peak_rss_kb": int(
                                max(rss_before, resource.getrusage(resource.RUSAGE_SELF).ru_maxrss)
                            ),
                        },
                    },
                )
            except OSError:
                pass
        return 1


if __name__ == "__main__":
    raise SystemExit(run())
