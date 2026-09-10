#!/usr/bin/env python3
"""Inventory and evaluate fixed, train-only smartphone generalization routes.

The archive inventory uses only ZIP central-directory metadata.  Once the
selection is frozen, the command verifies the profile archive digest, extracts
only the selected route/phone members through the R5 materializer, and runs a
fixed Galileo-E1/Hatch30 SPP lane followed by the already-promoted trajectory
smoother and its causal IMU-adaptive candidate.  Truth is never passed to an
adapter, solver, or smoother; it is read only by the final scoring function.

The command intentionally has no holdout mode.  The designated holdout ID is
read from profile metadata solely to exclude it, and no holdout member is
opened or materialized.
"""

from __future__ import annotations

import argparse
from bisect import bisect_left
from collections import Counter, defaultdict
import copy
from decimal import Decimal
import hashlib
import json
import math
import os
from pathlib import Path
import re
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
sys.path.insert(0, str(BENCHMARK_DIR))
import gnss_smartphone_gnss_workflow as workflow  # noqa: E402
import gnss_smartphone_kaggle as kaggle  # noqa: E402
import gnss_smartphone_trajectory_smoother as smoother  # noqa: E402
import gnss_smartphone_trajectory_smoother_eval as smoother_eval  # noqa: E402


SCHEMA_VERSION = "smartphone-r5-generalization.v1"
DEFAULT_PROFILE = ROOT / "configs" / "benchmarks" / "smartphone_r5_gsdc2023.json"
REQUIRED_PHONE_MEMBERS = ("device_gnss.csv", "device_imu.csv", "ground_truth.csv")
REQUIRED_ROUTE_MEMBER = "brdc.nav"
FIXED_CANDIDATE_IDS = (
    "2021-01-04-21-50-us-ca-e1highway280driveroutea/mi8",
    "2022-08-04-20-07-us-ca-sjc-q/pixel5",
    "2023-03-08-21-34-us-ca-mtv-u/pixel6pro",
)
IMU_CONFIG = {
    "window_s": 1.0,
    "max_sample_gap_s": 0.25,
    "max_sample_age_s": 0.25,
    "gyro_threshold": 0.25,
    "accel_dynamic_threshold": 1.0,
    "motion_q_multiplier": 2.0,
}
_RSS_MARKER = "__GNSS_GENERALIZATION_RSS_KB__"
_RSS_RE = re.compile(r"__GNSS_GENERALIZATION_RSS_KB__\s+(\d+)")
_GPS_EPOCH_UNIX_SECONDS = Decimal("315964800")
_SECONDS_PER_WEEK = Decimal("604800")
_LEAP_SECONDS = 18


class GeneralizationError(ValueError):
    """Raised when the fixed generalization contract cannot be satisfied."""


def _sha256(path: Path) -> str:
    if not path.is_file():
        raise GeneralizationError(f"missing file: {path}")
    digest = hashlib.sha256()
    try:
        with path.open("rb") as handle:
            for chunk in iter(lambda: handle.read(1024 * 1024), b""):
                digest.update(chunk)
    except OSError as exc:
        raise GeneralizationError(f"failed to hash {path}: {exc}") from exc
    return digest.hexdigest()


def _atomic_json(path: Path, payload: dict[str, Any]) -> None:
    content = (json.dumps(payload, indent=2, sort_keys=True) + "\n").encode("utf-8")
    smoother._atomic_write(path, content)


def _load_profile(path: Path) -> dict[str, Any]:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise GeneralizationError(f"invalid R5 profile: {path}") from exc
    if not isinstance(payload, dict) or payload.get("schema_version") != "smartphone-r5-profile.v1":
        raise GeneralizationError("profile schema is not smartphone-r5-profile.v1")
    datasets = payload.get("datasets")
    if not isinstance(datasets, dict):
        raise GeneralizationError("profile has no datasets")
    for role in ("development", "holdout"):
        if not isinstance(datasets.get(role), dict):
            raise GeneralizationError(f"profile has no {role} dataset metadata")
    return payload


def _dataset_id(dataset: dict[str, Any]) -> str:
    value = dataset.get("id")
    if not isinstance(value, str) or value.count("/") != 1:
        raise GeneralizationError("dataset id must have exactly one route/phone separator")
    route, phone = value.split("/", 1)
    if not route or not phone:
        raise GeneralizationError("dataset id has an empty route or phone")
    return value


def _route_region(route: str) -> str | None:
    tokens = route.split("-")
    try:
        index = tokens.index("ca")
    except ValueError:
        return None
    return tokens[index + 1] if index + 1 < len(tokens) else None


def _central_metadata(info: zipfile.ZipInfo) -> dict[str, Any]:
    return {
        "name": info.filename,
        "file_size": info.file_size,
        "compressed_size": info.compress_size,
        "crc32_hex": f"{info.CRC:08x}",
    }


def inventory_archive(archive_path: Path) -> dict[str, Any]:
    """Return a train inventory without opening a member payload.

    ``ZipFile.infolist`` reads the central directory only.  This function does
    not call ``open``/``read`` for any member, so it is safe to use before the
    candidate selection is frozen.
    """

    if not archive_path.is_file():
        raise GeneralizationError(f"missing GSDC archive: {archive_path}")
    records: dict[tuple[str, str], dict[str, list[dict[str, Any]]]] = defaultdict(
        lambda: defaultdict(list)
    )
    route_nav: dict[str, list[dict[str, Any]]] = defaultdict(list)
    duplicate_names: Counter[str] = Counter()
    train_entry_count = 0
    ignored_train_files = 0
    try:
        with zipfile.ZipFile(archive_path) as archive:
            infos = archive.infolist()
            for info in infos:
                duplicate_names[info.filename] += 1
                name = info.filename
                if not name.startswith("dataset_2023/train/"):
                    continue
                train_entry_count += 1
                if info.is_dir():
                    continue
                parts = name.split("/")
                if len(parts) == 4 and parts[3] == REQUIRED_ROUTE_MEMBER:
                    route_nav[parts[2]].append(_central_metadata(info))
                    continue
                if len(parts) == 5 and parts[4] in REQUIRED_PHONE_MEMBERS:
                    records[(parts[2], parts[3])][parts[4]].append(
                        _central_metadata(info)
                    )
                    continue
                ignored_train_files += 1
    except (OSError, zipfile.BadZipFile) as exc:
        raise GeneralizationError(f"failed to inspect archive central directory: {archive_path}") from exc

    rows: list[dict[str, Any]] = []
    for route, phone in sorted(records):
        files = records[(route, phone)]
        file_presence = {
            name: len(files.get(name, [])) for name in REQUIRED_PHONE_MEMBERS
        }
        complete = all(count == 1 for count in file_presence.values())
        nav_entries = route_nav.get(route, [])
        rows.append(
            {
                "dataset_id": f"{route}/{phone}",
                "route": route,
                "phone": phone,
                "calendar_year": int(route[:4]) if route[:4].isdigit() else None,
                "route_region": _route_region(route),
                "required_files": file_presence,
                "required_files_complete": complete,
                "broadcast_nav_present": len(nav_entries) == 1,
                "broadcast_nav_duplicate_count": max(0, len(nav_entries) - 1),
                "central_directory_files": {
                    name: values[0] if len(values) == 1 else values
                    for name, values in sorted(files.items())
                },
                "central_directory_broadcast_nav": nav_entries[0] if len(nav_entries) == 1 else nav_entries,
            }
        )

    duplicate_list = sorted(name for name, count in duplicate_names.items() if count > 1)
    return {
        "schema_version": "smartphone-r5-gsdc2023-central-inventory.v1",
        "archive": {
            "path": str(archive_path),
            "size_bytes": archive_path.stat().st_size,
            "central_directory_only": True,
            "member_content_read": False,
            "member_sha256_computed": False,
            "central_entry_count": len(infos),
            "train_entry_count": train_entry_count,
        },
        "train": {
            "route_count": len({row["route"] for row in rows}),
            "route_phone_count": len(rows),
            "ignored_train_file_count": ignored_train_files,
            "duplicate_central_names": duplicate_list,
            "records": rows,
        },
    }


def select_candidates(
    inventory: dict[str, Any], profile: dict[str, Any], max_candidates: int = 3
) -> list[dict[str, Any]]:
    """Select the predeclared candidates and enforce all exclusion rules."""

    if max_candidates <= 0 or max_candidates > len(FIXED_CANDIDATE_IDS):
        raise GeneralizationError("max-candidates must be between 1 and 3")
    development_id = _dataset_id(dict(profile["datasets"]["development"]))
    holdout_id = _dataset_id(dict(profile["datasets"]["holdout"]))
    if development_id in FIXED_CANDIDATE_IDS or holdout_id in FIXED_CANDIDATE_IDS:
        raise GeneralizationError("profile development/holdout ID collides with fixed candidates")
    by_id = {row["dataset_id"]: row for row in inventory["train"]["records"]}
    selected: list[dict[str, Any]] = []
    for dataset_id in FIXED_CANDIDATE_IDS[:max_candidates]:
        if dataset_id in (development_id, holdout_id):
            raise GeneralizationError(f"sealed dataset selected accidentally: {dataset_id}")
        row = by_id.get(dataset_id)
        if row is None:
            raise GeneralizationError(f"fixed candidate is not present in archive inventory: {dataset_id}")
        if not row["required_files_complete"]:
            raise GeneralizationError(f"fixed candidate lacks one of the three phone CSVs: {dataset_id}")
        if not row["broadcast_nav_present"] or row["broadcast_nav_duplicate_count"]:
            raise GeneralizationError(f"fixed candidate lacks a unique route broadcast nav: {dataset_id}")
        selected.append(row)
    if len({row["route"] for row in selected}) != len(selected):
        raise GeneralizationError("candidate selection reused a route")
    if len({row["phone"] for row in selected}) != len(selected):
        raise GeneralizationError("candidate selection reused a phone model")
    return selected


def _member_names(route: str, phone: str) -> dict[str, str]:
    return {
        "device_gnss": f"dataset_2023/train/{route}/{phone}/device_gnss.csv",
        "device_imu": f"dataset_2023/train/{route}/{phone}/device_imu.csv",
        "ground_truth": f"dataset_2023/train/{route}/{phone}/ground_truth.csv",
        "broadcast_nav": f"dataset_2023/train/{route}/brdc.nav",
    }


def _discover_member_hashes(
    archive_path: Path, member_names: dict[str, str]
) -> dict[str, dict[str, Any]]:
    """Hash only selected payloads and reject missing/duplicate entries."""

    try:
        with zipfile.ZipFile(archive_path) as archive:
            by_name: dict[str, list[zipfile.ZipInfo]] = defaultdict(list)
            for info in archive.infolist():
                by_name[info.filename].append(info)
            result: dict[str, dict[str, Any]] = {}
            for key, member in member_names.items():
                entries = by_name.get(member, [])
                if len(entries) != 1 or entries[0].is_dir():
                    raise GeneralizationError(
                        f"selected member must exist exactly once as a file: {member}"
                    )
                info = entries[0]
                digest = hashlib.sha256()
                size = 0
                with archive.open(info, "r") as source:
                    for chunk in iter(lambda: source.read(1024 * 1024), b""):
                        digest.update(chunk)
                        size += len(chunk)
                if size != info.file_size:
                    raise GeneralizationError(
                        f"selected member size differs from central directory: {member}"
                    )
                result[key] = {
                    "member": member,
                    "sha256": digest.hexdigest(),
                    "file_size": size,
                    "compressed_size": info.compress_size,
                    "crc32_hex": f"{info.CRC:08x}",
                }
            return result
    except (OSError, zipfile.BadZipFile, zipfile.LargeZipFile) as exc:
        raise GeneralizationError(f"failed to hash selected archive members") from exc


def _candidate_profile(
    base_profile: dict[str, Any],
    candidate: dict[str, Any],
    archive_sha256: str,
    member_hashes: dict[str, dict[str, Any]],
) -> dict[str, Any]:
    profile = copy.deepcopy(base_profile)
    profile["profile_id"] = f"gsdc2023-generalization-{candidate['dataset_id'].replace('/', '-')}-v1"
    profile["archive"]["sha256"] = archive_sha256
    development = dict(profile["datasets"]["development"])
    profile["datasets"]["development"] = {
        "id": candidate["dataset_id"],
        "device_model": candidate["phone"],
        "device_gnss_sha256": member_hashes["device_gnss"]["sha256"],
        "device_imu_sha256": member_hashes["device_imu"]["sha256"],
        "ground_truth_sha256": member_hashes["ground_truth"]["sha256"],
        "broadcast_nav_sha256": member_hashes["broadcast_nav"]["sha256"],
        "skip_epochs": int(development.get("skip_epochs", 1)),
    }
    profile["generalization_candidate"] = {
        "truth_free_selection": True,
        "holdout_untouched": True,
        "member_hashes": member_hashes,
    }
    return profile


def materialize_candidate(
    archive_path: Path,
    profile: dict[str, Any],
    candidate: dict[str, Any],
    archive_sha256: str,
    output_root: Path,
    member_hashes: dict[str, dict[str, Any]],
) -> dict[str, Any]:
    """Use the frozen workflow materializer for one explicit candidate."""

    route, phone = candidate["dataset_id"].split("/", 1)
    candidate_root = output_root / "routes" / route / phone
    candidate_root.mkdir(parents=True, exist_ok=True)
    inputs = candidate_root / "inputs"
    if inputs.exists() and any(inputs.iterdir()):
        raise GeneralizationError(f"refusing to overwrite existing candidate inputs: {inputs}")
    staging = candidate_root / f".inputs-staging-{os.getpid()}"
    if staging.exists():
        raise GeneralizationError(f"staging path already exists: {staging}")
    staged_paths: dict[str, Path] = {}
    try:
        staged_paths = workflow.materialize_inputs(
            archive_path,
            profile,
            "development",
            staging,
            verified_archive_sha256=archive_sha256,
        )
        inputs.mkdir(parents=True, exist_ok=True)
        for key, staged in staged_paths.items():
            expected = member_hashes[key]["sha256"]
            actual = _sha256(staged)
            if actual != expected:
                raise GeneralizationError(f"materialized {key} hash mismatch")
            os.replace(staged, inputs / staged.name)
    finally:
        if staging.exists():
            shutil.rmtree(staging)
    manifest = {
        "schema_version": "smartphone-r5-generalization-materialization.v1",
        "dataset_id": candidate["dataset_id"],
        "role": "development",
        "archive": {"path": str(archive_path), "sha256": archive_sha256},
        "central_directory_contract": {
            "selected_members_only": True,
            "missing_duplicate_or_crc_error": "fail-closed",
        },
        "inputs": {
            key: {
                "path": str(inputs / Path(staged_paths[key]).name),
                "sha256": member_hashes[key]["sha256"],
                "file_size": member_hashes[key]["file_size"],
            }
            for key in member_hashes
        },
        "holdout_content_opened": False,
        "truth_used_for_materialization": False,
    }
    _atomic_json(candidate_root / "materialization_manifest.json", manifest)
    return {
        "root": candidate_root,
        "inputs": inputs,
        "manifest": candidate_root / "materialization_manifest.json",
        "manifest_sha256": _sha256(candidate_root / "materialization_manifest.json"),
    }


def _run_stage(
    stage: str, command: list[str], log_path: Path
) -> dict[str, Any]:
    started = time.perf_counter()
    timed_command = command
    if Path("/usr/bin/time").is_file():
        timed_command = ["/usr/bin/time", "-f", f"{_RSS_MARKER} %M", *command]
    result = subprocess.run(
        timed_command,
        cwd=ROOT,
        capture_output=True,
        text=True,
        check=False,
    )
    elapsed = time.perf_counter() - started
    output = result.stdout + result.stderr
    log_path.parent.mkdir(parents=True, exist_ok=True)
    with log_path.open("a", encoding="utf-8") as handle:
        handle.write(f"$ {' '.join(command)}\n")
        handle.write(output)
    rss_matches = _RSS_RE.findall(output)
    rss = int(rss_matches[-1]) if rss_matches else None
    record = {
        "stage": stage,
        "command": command,
        "return_code": result.returncode,
        "wall_time_s": elapsed,
        "peak_rss_kb": rss,
    }
    if result.returncode != 0:
        tail = output[-2000:].replace("\n", " ")
        raise GeneralizationError(f"{stage} failed ({result.returncode}): {tail}")
    return record


def _truth_match(
    timestamp: int,
    truth: dict[int, tuple[float, float, float]],
    tolerance_ms: int,
) -> tuple[float, float, float] | None:
    if timestamp in truth:
        return truth[timestamp]
    nearest = min(truth, key=lambda candidate: abs(candidate - timestamp))
    return truth[nearest] if abs(nearest - timestamp) <= tolerance_ms else None


def _score_position(
    position_path: Path,
    device_gnss_path: Path,
    ground_truth_path: Path,
    skip_epochs: int,
    *,
    tolerance_ms: int = 100,
) -> dict[str, Any]:
    """Score one published POS against truth after the truth-free run."""

    device_epochs = smoother._read_device_epochs(device_gnss_path, skip_epochs)
    position_rows = smoother._read_positions(
        position_path, smoother.DEFAULT_GPS_UTC_LEAP_SECONDS
    )
    device_keys = set(device_epochs)
    if any(row.timestamp_ms not in device_keys for row in position_rows):
        raise GeneralizationError("position output contains a key absent from device GNSS")
    truth = smoother_eval._read_truth(ground_truth_path)
    h_wgs84: list[float] = []
    h_haversine: list[float] = []
    vertical: list[float] = []
    for row in position_rows:
        reference = _truth_match(row.timestamp_ms, truth, tolerance_ms)
        if reference is None:
            continue
        truth_lat, truth_lon, truth_height = reference
        h_wgs84.append(
            kaggle._wgs84_horizontal_distance_m(
                row.latitude, row.longitude, truth_lat, truth_lon
            )
        )
        h_haversine.append(
            kaggle._haversine_horizontal_distance_m(
                row.latitude, row.longitude, truth_lat, truth_lon
            )
        )
        vertical.append(abs(row.height - truth_height))

    def pair(values: list[float]) -> dict[str, float | None]:
        return {
            "p50_m": kaggle._percentile_linear_n_minus_1(values, 0.50)
            if values
            else None,
            "p95_m": kaggle._percentile_linear_n_minus_1(values, 0.95)
            if values
            else None,
        }

    wgs84 = pair(h_wgs84)
    haversine = pair(h_haversine)
    score_variants: dict[str, float | None] = {}
    for distance_name, metrics in (
        ("wgs84_vincenty", wgs84),
        ("haversine_sphere", haversine),
    ):
        score_variants[f"{distance_name}__linear_n_minus_1"] = (
            (float(metrics["p50_m"]) + float(metrics["p95_m"])) / 2.0
            if metrics["p50_m"] is not None and metrics["p95_m"] is not None
            else None
        )
    for distance_name, values in (
        ("wgs84_vincenty", h_wgs84),
        ("haversine_sphere", h_haversine),
    ):
        score_variants[f"{distance_name}__nearest_rank_ceiling"] = (
            (
                kaggle._percentile_nearest_rank_ceiling(values, 0.50)
                + kaggle._percentile_nearest_rank_ceiling(values, 0.95)
            )
            / 2.0
            if values
            else None
        )
    phone_scores = {
        key: value for key, value in score_variants.items()
    }
    return {
        "device_epochs": len(device_epochs),
        "position_epochs": len(position_rows),
        "truth_matched_epochs": len(h_wgs84),
        "availability_ratio": len(position_rows) / len(device_epochs)
        if device_epochs
        else 0.0,
        "truth_coverage_ratio": len(h_wgs84) / len(device_epochs)
        if device_epochs
        else 0.0,
        "horizontal_wgs84_m": wgs84,
        "horizontal_haversine_m": haversine,
        "vertical_p95_abs_m": kaggle._percentile_linear_n_minus_1(vertical, 0.95)
        if vertical
        else None,
        "kaggle_diagnostic_score_variants_m": score_variants,
        "phone_score_variants_m": phone_scores,
        "official_primary_score_m": None,
        "official_primary_status": "undetermined-from-public-primary-sources",
        "position_sha256": _sha256(position_path),
    }


def _spp_command(
    obs_path: Path, nav_path: Path, out_path: Path, summary_path: Path
) -> list[str]:
    return [
        sys.executable,
        str(ROOT / "apps" / "gnss.py"),
        "spp",
        "--obs",
        str(obs_path),
        "--nav",
        str(nav_path),
        "--out",
        str(out_path),
        "--summary-json",
        str(summary_path),
    ]


def _canonicalize_position_timestamps(
    raw_position_path: Path,
    device_gnss_path: Path,
    output_path: Path,
    skip_epochs: int,
    tolerance_ms: int = 1,
) -> dict[str, Any]:
    """Align sub-millisecond POS formatting to the raw device epoch keys.

    Android logs commonly represent a one-second epoch as ``...999`` while a
    native POS writer rounds its GPS TOW to ``...000``.  The coordinates and
    quality tokens are copied unchanged; only week/TOW tokens are replaced by
    exact Decimal values derived from the selected device key.  A mapping
    outside the explicit one-millisecond tolerance, a duplicate mapping, or a
    non-data line parse failure is rejected.
    """

    device_epochs = smoother._read_device_epochs(device_gnss_path, skip_epochs)
    selected = set(device_epochs)
    mapped: set[int] = set()
    output_lines: list[str] = []
    changed = 0
    data_rows = 0
    try:
        with raw_position_path.open(encoding="ascii") as handle:
            for line_number, raw_line in enumerate(handle, start=1):
                stripped = raw_line.strip()
                if not stripped or stripped.startswith("%") or stripped.startswith("#"):
                    output_lines.append(raw_line)
                    continue
                fields = stripped.split()
                if len(fields) < 2:
                    raise GeneralizationError(
                        f"position line {line_number}: missing week/TOW fields"
                    )
                try:
                    timestamp = smoother._position_timestamp(
                        fields[0], fields[1], _LEAP_SECONDS, line_number
                    )
                except (smoother.SmootherError, ValueError) as exc:
                    raise GeneralizationError(
                        f"position line {line_number}: invalid timestamp"
                    ) from exc
                index = bisect_left(device_epochs, timestamp)
                nearest_candidates = []
                if index < len(device_epochs):
                    nearest_candidates.append(device_epochs[index])
                if index:
                    nearest_candidates.append(device_epochs[index - 1])
                if not nearest_candidates:
                    raise GeneralizationError("position output has no selected device epoch")
                nearest = min(nearest_candidates, key=lambda value: abs(value - timestamp))
                if abs(nearest - timestamp) > tolerance_ms or nearest not in selected:
                    raise GeneralizationError(
                        "position timestamp is not within the selected device epoch tolerance: "
                        f"{timestamp} (nearest {nearest})"
                    )
                if nearest in mapped:
                    raise GeneralizationError(
                        f"multiple position rows map to device epoch {nearest}"
                    )
                mapped.add(nearest)
                gps_seconds = (
                    Decimal(nearest) / Decimal(1000)
                    + Decimal(_LEAP_SECONDS)
                    - _GPS_EPOCH_UNIX_SECONDS
                )
                week = int(gps_seconds // _SECONDS_PER_WEEK)
                tow = gps_seconds - Decimal(week) * _SECONDS_PER_WEEK
                original_week, original_tow = fields[0], fields[1]
                fields[0] = str(week)
                fields[1] = format(tow, "f")
                normalized = " ".join(fields) + "\n"
                if fields[0] != original_week or fields[1] != original_tow:
                    changed += 1
                output_lines.append(normalized)
                data_rows += 1
    except OSError as exc:
        raise GeneralizationError(f"failed to canonicalize position file: {raw_position_path}") from exc
    if not data_rows:
        raise GeneralizationError("position output contains no data rows")
    smoother._atomic_write(output_path, "".join(output_lines).encode("ascii"))
    return {
        "path": str(output_path),
        "sha256": _sha256(output_path),
        "data_rows": data_rows,
        "mapped_device_epochs": len(mapped),
        "changed_timestamp_rows": changed,
        "tolerance_ms": tolerance_ms,
    }


def _smoother_command(
    position_path: Path,
    device_path: Path,
    output_dir: Path,
    profile_path: Path,
    skip_epochs: int,
    *,
    device_imu_path: Path | None = None,
) -> list[str]:
    command = [
        sys.executable,
        str(ROOT / "apps" / "gnss.py"),
        "smartphone-trajectory-smoother",
        "--experimental-truth-free-kalman-rts",
        "--position",
        str(position_path),
        "--device-gnss",
        str(device_path),
        "--output-dir",
        str(output_dir),
        "--profile",
        str(profile_path),
        "--role",
        "development",
        "--skip-epochs",
        str(skip_epochs),
        "--process-noise",
        "1.0",
        "--measurement-floor-m",
        "1.0",
        "--outlier-gate-sigma",
        "5.0",
        "--segment-gap-s",
        "10.0",
    ]
    if device_imu_path is not None:
        command.extend(
            [
                "--device-imu",
                str(device_imu_path),
                "--experimental-motion-adaptive-q",
                "--motion-window-s",
                str(IMU_CONFIG["window_s"]),
                "--motion-max-sample-gap-s",
                str(IMU_CONFIG["max_sample_gap_s"]),
                "--motion-max-sample-age-s",
                str(IMU_CONFIG["max_sample_age_s"]),
                "--motion-gyro-threshold",
                str(IMU_CONFIG["gyro_threshold"]),
                "--motion-accel-dynamic-threshold",
                str(IMU_CONFIG["accel_dynamic_threshold"]),
                "--motion-q-multiplier",
                str(IMU_CONFIG["motion_q_multiplier"]),
            ]
        )
    return command


def _route_report(
    archive_path: Path,
    archive_sha256: str,
    profile: dict[str, Any],
    candidate: dict[str, Any],
    selected_profile: dict[str, Any],
    materialized: dict[str, Any],
    output_root: Path,
    max_epochs: int,
) -> dict[str, Any]:
    route_started = time.perf_counter()
    route_root: Path = materialized["root"]
    inputs: Path = materialized["inputs"]
    profile_path = route_root / "candidate_profile.json"
    _atomic_json(profile_path, selected_profile)
    device_path = inputs / "device_gnss.csv"
    imu_path = inputs / "device_imu.csv"
    truth_path = inputs / "ground_truth.csv"
    nav_path = inputs / "brdc.nav"
    adapter_dir = route_root / "adapter"
    spp_dir = route_root / "spp"
    baseline_dir = route_root / "baseline"
    imu_dir = route_root / "imu-adaptive"
    log_path = route_root / "generalization.log"
    for stage_dir in (adapter_dir, spp_dir, baseline_dir, imu_dir):
        stage_dir.mkdir(parents=True, exist_ok=True)
    common = [
        sys.executable,
        str(ROOT / "apps" / "gnss.py"),
        "smartphone-gnss-adapter",
        "--device-gnss",
        str(device_path),
        "--ground-truth",
        str(truth_path),
        "--output-dir",
        str(adapter_dir),
        "--dataset-id",
        candidate["dataset_id"],
        "--device-model",
        candidate["phone"],
        "--source-url",
        str(dict(profile["archive"]).get("url", "local-profile")),
        "--source-terms",
        str(dict(profile["archive"]).get("source_terms", "local evaluation only")),
        "--role",
        "development",
        "--skip-epochs",
        str(int(selected_profile["datasets"]["development"].get("skip_epochs", 1))),
        "--allow-missing-truth",
        "--experimental-galileo-e1",
        "--experimental-galileo-e1-hatch-window-s",
        "30",
        "--broadcast-nav",
        str(nav_path),
    ]
    if max_epochs > 0:
        common.extend(["--max-epochs", str(max_epochs)])
    stages = [
        _run_stage("adapter_galileo_e1_hatch30", common, log_path),
    ]
    adapter_rinex = adapter_dir / "rover.obs"
    raw_pos = spp_dir / "libgnsspp_spp.pos"
    raw_summary = spp_dir / "libgnsspp_spp_summary.json"
    stages.append(
        _run_stage(
            "spp",
            _spp_command(adapter_rinex, nav_path, raw_pos, raw_summary),
            log_path,
        )
    )
    if not raw_pos.is_file():
        raise GeneralizationError(f"SPP did not publish a position file: {raw_pos}")
    canonical_pos = spp_dir / "canonical.pos"
    canonical_position = _canonicalize_position_timestamps(
        raw_pos,
        device_path,
        canonical_pos,
        int(selected_profile["datasets"]["development"].get("skip_epochs", 1)),
    )
    stages.append(
        _run_stage(
            "trajectory_baseline",
            _smoother_command(
                canonical_pos,
                device_path,
                baseline_dir,
                profile_path,
                int(selected_profile["datasets"]["development"].get("skip_epochs", 1)),
            ),
            log_path,
        )
    )
    stages.append(
        _run_stage(
            "trajectory_imu_adaptive",
            _smoother_command(
                canonical_pos,
                device_path,
                imu_dir,
                profile_path,
                int(selected_profile["datasets"]["development"].get("skip_epochs", 1)),
                device_imu_path=imu_path,
            ),
            log_path,
        )
    )
    skip_epochs = int(selected_profile["datasets"]["development"].get("skip_epochs", 1))
    baseline_metrics = _score_position(
        baseline_dir / "smoothed.pos", device_path, truth_path, skip_epochs
    )
    imu_metrics = _score_position(
        imu_dir / "smoothed.pos", device_path, truth_path, skip_epochs
    )
    adapter_summary_path = adapter_dir / "summary.json"
    if not adapter_summary_path.is_file():
        raise GeneralizationError(f"adapter summary is missing: {adapter_summary_path}")
    adapter_summary = json.loads(adapter_summary_path.read_text(encoding="utf-8"))
    observation_summary = dict(adapter_summary.get("observations", {}))
    navigation_summary = adapter_summary.get("navigation")
    return {
        "dataset_id": candidate["dataset_id"],
        "route": candidate["route"],
        "phone": candidate["phone"],
        "calendar_year": candidate["calendar_year"],
        "route_region": candidate["route_region"],
        "inputs": {
            "archive": {"path": str(archive_path), "sha256": archive_sha256},
            "profile": {"path": str(profile_path), "sha256": _sha256(profile_path)},
            "materialization_manifest": {
                "path": str(materialized["manifest"]),
                "sha256": materialized["manifest_sha256"],
            },
        },
        "observation_counts": {
            "source_rows": observation_summary.get("rows"),
            "source_epochs": observation_summary.get("epochs"),
            "signal_rows": observation_summary.get("signal_rows"),
            "galileo_e1_rows": (
                dict(observation_summary.get("signal_rows", {})).get("GAL_E1_C_P", 0)
            ),
            "rinex_source_rows": dict(
                adapter_summary.get("native_observation_adapter", {})
            ).get("source_rows"),
            "navigation": navigation_summary,
        },
        "stages": stages,
        "position_timestamp_canonicalization": canonical_position,
        "lanes": {
            "galileo_e1_hatch30_fixed_q": {
                "configuration": {
                    "process_noise": 1.0,
                    "measurement_floor_m": 1.0,
                    "outlier_gate_sigma": 5.0,
                    "segment_gap_s": 10.0,
                },
                "metrics": baseline_metrics,
                "artifacts": {
                    "position": str(baseline_dir / "smoothed.pos"),
                    "manifest": str(baseline_dir / "smoother_manifest.json"),
                },
            },
            "galileo_e1_hatch30_imu_adaptive": {
                "configuration": IMU_CONFIG,
                "metrics": imu_metrics,
                "artifacts": {
                    "position": str(imu_dir / "smoothed.pos"),
                    "manifest": str(imu_dir / "smoother_manifest.json"),
                    "motion_profile": str(imu_dir / "motion_profile.csv"),
                },
            },
        },
        "route_wall_time_s": time.perf_counter() - route_started,
    }


def _mean(values: list[float]) -> float | None:
    return sum(values) / len(values) if values else None


def _aggregate(route_reports: list[dict[str, Any]], lane: str) -> dict[str, Any]:
    metric_rows = [report["lanes"][lane]["metrics"] for report in route_reports]
    variant_keys = sorted(
        {
            key
            for row in metric_rows
            for key in row["phone_score_variants_m"]
        }
    )
    scores = {
        key: _mean(
            [
                float(row["phone_score_variants_m"][key])
                for row in metric_rows
                if row["phone_score_variants_m"].get(key) is not None
            ]
        )
        for key in variant_keys
    }
    h_p50 = [
        float(row["horizontal_wgs84_m"]["p50_m"])
        for row in metric_rows
        if row["horizontal_wgs84_m"]["p50_m"] is not None
    ]
    h_p95 = [
        float(row["horizontal_wgs84_m"]["p95_m"])
        for row in metric_rows
        if row["horizontal_wgs84_m"]["p95_m"] is not None
    ]
    v_p95 = [
        float(row["vertical_p95_abs_m"])
        for row in metric_rows
        if row["vertical_p95_abs_m"] is not None
    ]
    return {
        "phone_count": len(metric_rows),
        "phone_score_definition": "mean over phones of (phone P50 + phone P95) / 2; four distance/percentile diagnostics are reported separately",
        "aggregate_phone_score_variants_m": scores,
        "mean_route_horizontal_wgs84_p50_m": _mean(h_p50),
        "mean_route_horizontal_wgs84_p95_m": _mean(h_p95),
        "mean_route_vertical_p95_abs_m": _mean(v_p95),
        "mean_availability_ratio": _mean(
            [float(row["availability_ratio"]) for row in metric_rows]
        ),
        "mean_truth_coverage_ratio": _mean(
            [float(row["truth_coverage_ratio"]) for row in metric_rows]
        ),
    }


def _aggregate_performance(route_reports: list[dict[str, Any]]) -> dict[str, Any]:
    """Summarize wall/RSS for the two comparable trajectory stages."""

    result: dict[str, Any] = {}
    for lane, stage_name in (
        ("galileo_e1_hatch30_fixed_q", "trajectory_baseline"),
        ("galileo_e1_hatch30_imu_adaptive", "trajectory_imu_adaptive"),
    ):
        stages = [
            next(stage for stage in report["stages"] if stage["stage"] == stage_name)
            for report in route_reports
        ]
        walls = [float(stage["wall_time_s"]) for stage in stages]
        rss = [
            int(stage["peak_rss_kb"])
            for stage in stages
            if stage.get("peak_rss_kb") is not None
        ]
        result[lane] = {
            "stage": stage_name,
            "mean_wall_time_s": _mean(walls),
            "p95_wall_time_s": kaggle._percentile_linear_n_minus_1(walls, 0.95)
            if walls
            else None,
            "mean_peak_rss_kb": _mean([float(value) for value in rss]),
            "max_peak_rss_kb": max(rss) if rss else None,
        }
    return result


def _comparison(
    baseline: dict[str, Any], candidate: dict[str, Any]
) -> dict[str, Any]:
    """Apply the existing non-regression gate without promoting anything."""

    failures: list[str] = []
    if float(candidate["mean_availability_ratio"]) < float(
        baseline["mean_availability_ratio"]
    ):
        failures.append("availability_regression")
    for field in (
        "mean_route_horizontal_wgs84_p50_m",
        "mean_route_horizontal_wgs84_p95_m",
        "mean_route_vertical_p95_abs_m",
    ):
        if float(candidate[field]) > float(baseline[field]) and not math.isclose(
            float(candidate[field]), float(baseline[field]), rel_tol=0.0, abs_tol=1e-12
        ):
            failures.append(f"{field}_regression")
    baseline_scores = baseline["aggregate_phone_score_variants_m"]
    candidate_scores = candidate["aggregate_phone_score_variants_m"]
    for key in sorted(baseline_scores):
        if (
            candidate_scores.get(key) is not None
            and baseline_scores.get(key) is not None
            and float(candidate_scores[key]) > float(baseline_scores[key])
            and not math.isclose(
                float(candidate_scores[key]),
                float(baseline_scores[key]),
                rel_tol=0.0,
                abs_tol=1e-12,
            )
        ):
            failures.append(f"{key}_regression")
    strict = (
        float(candidate["mean_route_horizontal_wgs84_p50_m"])
        < float(baseline["mean_route_horizontal_wgs84_p50_m"]) - 1e-12
        or float(candidate["mean_route_horizontal_wgs84_p95_m"])
        < float(baseline["mean_route_horizontal_wgs84_p95_m"]) - 1e-12
        or float(candidate["mean_route_vertical_p95_abs_m"])
        < float(baseline["mean_route_vertical_p95_abs_m"]) - 1e-12
        or any(
            candidate_scores.get(key) is not None
            and baseline_scores.get(key) is not None
            and float(candidate_scores[key]) < float(baseline_scores[key]) - 1e-12
            for key in baseline_scores
        )
    )
    if not strict:
        failures.append("no_strict_accuracy_improvement")
    return {
        "non_regression_passed": not any(
            failure != "no_strict_accuracy_improvement" for failure in failures
        ),
        "strict_accuracy_improvement": strict,
        "promotion_decision": "no-go-no-strict-improvement" if failures else "eligible-for-review",
        "failures": failures,
    }


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        prog=os.environ.get("GNSS_CLI_NAME", "gnss smartphone-generalization")
    )
    parser.add_argument("--archive", type=Path, required=True)
    parser.add_argument("--profile", type=Path, default=DEFAULT_PROFILE)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--max-candidates", type=int, default=3)
    parser.add_argument("--max-epochs", type=int, default=-1)
    parser.add_argument(
        "--inventory-only",
        action="store_true",
        help="central-directory inventory and fixed selection only; do not read members",
    )
    return parser.parse_args(argv)


def run(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    try:
        if args.max_epochs == 0 or args.max_epochs < -1:
            raise GeneralizationError("max-epochs must be -1 or a positive integer")
        profile = _load_profile(args.profile)
        inventory = inventory_archive(args.archive)
        selected = select_candidates(inventory, profile, args.max_candidates)
        archive_expected = str(dict(profile["archive"]).get("sha256", ""))
        if not archive_expected:
            raise GeneralizationError("profile archive SHA-256 is empty")
        # Hash the immutable archive exactly once before opening any selected
        # payload.  The central inventory above remains content-free.
        archive_sha = _sha256(args.archive)
        if archive_sha != archive_expected:
            raise GeneralizationError("GSDC archive hash does not match the frozen profile")
        inventory["archive"].update(
            {
                "sha256": archive_sha,
                "member_sha256_computed": False,
                "archive_hash_verified_before_member_extraction": True,
            }
        )
        inventory["selection"] = {
            "fixed_candidate_ids": list(FIXED_CANDIDATE_IDS),
            "selected_candidate_ids": [row["dataset_id"] for row in selected],
            "development_excluded": _dataset_id(dict(profile["datasets"]["development"])),
            "holdout_excluded": _dataset_id(dict(profile["datasets"]["holdout"])),
            "holdout_content_opened": False,
            "holdout_truth_opened": False,
            "selection_content_read": False,
        }
        args.output_dir.mkdir(parents=True, exist_ok=True)
        _atomic_json(args.output_dir / "archive_inventory.json", inventory)
        if args.inventory_only:
            report = {
                "schema_version": SCHEMA_VERSION,
                "decision": "inventory-complete",
                "archive_inventory": str(args.output_dir / "archive_inventory.json"),
                "selected_candidate_ids": [row["dataset_id"] for row in selected],
                "holdout_content_opened": False,
            }
            _atomic_json(args.output_dir / "generalization_report.json", report)
            print(f"Smartphone generalization inventory complete: {args.output_dir}")
            return 0

        output_root = args.output_dir
        route_reports: list[dict[str, Any]] = []
        materialization_records: list[dict[str, Any]] = []
        for candidate in selected:
            member_hashes = _discover_member_hashes(
                args.archive,
                _member_names(candidate["route"], candidate["phone"]),
            )
            candidate_profile = _candidate_profile(
                profile, candidate, archive_sha, member_hashes
            )
            materialized = materialize_candidate(
                args.archive,
                candidate_profile,
                candidate,
                archive_sha,
                output_root,
                member_hashes,
            )
            materialization_records.append(
                {
                    "dataset_id": candidate["dataset_id"],
                    "manifest": str(materialized["manifest"]),
                    "manifest_sha256": materialized["manifest_sha256"],
                    "member_hashes": member_hashes,
                }
            )
            route_reports.append(
                _route_report(
                    args.archive,
                    archive_sha,
                    profile,
                    candidate,
                    candidate_profile,
                    materialized,
                    output_root,
                    args.max_epochs,
                )
            )
        aggregate = {
            "galileo_e1_hatch30_fixed_q": _aggregate(
                route_reports, "galileo_e1_hatch30_fixed_q"
            ),
            "galileo_e1_hatch30_imu_adaptive": _aggregate(
                route_reports, "galileo_e1_hatch30_imu_adaptive"
            ),
        }
        aggregate["comparison_imu_vs_fixed_q"] = _comparison(
            aggregate["galileo_e1_hatch30_fixed_q"],
            aggregate["galileo_e1_hatch30_imu_adaptive"],
        )
        report = {
            "schema_version": SCHEMA_VERSION,
            "decision": "evaluated-train-generalization-no-holdout",
            "archive": {
                "path": str(args.archive),
                "sha256": archive_sha,
                "profile_sha256": _sha256(args.profile),
            },
            "selection": inventory["selection"],
            "evaluation_contract": {
                "signal_lane": "Galileo E1 with Hatch C1C 30 s",
                "fixed_q_lane": {
                    "process_noise": 1.0,
                    "measurement_floor_m": 1.0,
                    "outlier_gate_sigma": 5.0,
                    "segment_gap_s": 10.0,
                },
                "imu_lane": IMU_CONFIG,
                "truth_free_before_scoring": True,
                "official_metric_asserted": False,
                "official_metric_status": "undetermined-from-public-primary-sources",
                "phone_score": "per-phone (P50+P95)/2, then arithmetic mean across selected phones",
            },
            "inventory": {
                "path": str(args.output_dir / "archive_inventory.json"),
                "sha256": _sha256(args.output_dir / "archive_inventory.json"),
            },
            "materialization": materialization_records,
            "routes": route_reports,
            "aggregate": aggregate,
            "performance": _aggregate_performance(route_reports),
            "holdout_policy": {
                "opened": False,
                "materialized": False,
                "scored": False,
            },
        }
        _atomic_json(args.output_dir / "generalization_report.json", report)
        print(f"Smartphone generalization complete: {args.output_dir / 'generalization_report.json'}")
        return 0
    except (GeneralizationError, OSError, ValueError, KeyError, TypeError, json.JSONDecodeError) as exc:
        print(f"Smartphone generalization failed: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(run())
