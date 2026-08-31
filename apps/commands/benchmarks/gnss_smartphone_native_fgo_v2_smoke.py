#!/usr/bin/env python3
"""Run the frozen, truth-free native-FGO v2 IMU-factor smoke.

This command is deliberately a bounded research harness.  It materializes
only the three newly frozen train payloads (GNSS, IMU, broadcast navigation,
and the route reference RINEX), converts the phone GNSS stream to RINEX, and
invokes the existing ``gnss_fgo_parity`` GTSAM path for two epochs.  That path
constructs real Pose3/velocity/bias states and GTSAM CombinedImuFactors.

The harness also reports a constant GNSS/IMU timestamp offset estimated from
raw observable timing and a fixed, truth-free speed/activity correlation grid.
The current parity binary has no offset argument and therefore never applies
the estimate silently: a non-finite/low-coverage estimate is fail-closed and
the frozen native-FGO-v1 fallback is recorded.  No truth member is opened or
materialized by this module, and no smartphone submission is produced.
"""

from __future__ import annotations

import argparse
import bisect
import csv
from collections import defaultdict
import hashlib
import json
import math
import os
from pathlib import Path
import resource
import re
import shutil
import statistics
import subprocess
import sys
import tempfile
import time
from typing import Any, Iterable
import zipfile

_COMMANDS_DIR = Path(__file__).resolve().parents[1]
if str(_COMMANDS_DIR) not in sys.path:
    sys.path.insert(0, str(_COMMANDS_DIR))
from support.gnss_runtime import application_root


ROOT = application_root(__file__)
BENCHMARK_DIR = ROOT / "apps" / "commands" / "benchmarks"
if str(BENCHMARK_DIR) not in sys.path:
    sys.path.insert(0, str(BENCHMARK_DIR))


SCHEMA_VERSION = "smartphone-r5-gsdc2023-native-fgo-v2-smoke.v1"
ROUTE_SCHEMA_VERSION = "smartphone-r5-gsdc2023-native-fgo-v2-route-smoke.v1"
FREEZE_SCHEMA = "smartphone-r5-gsdc2023-native-fgo-v2-candidate-freeze.v1"
FREEZE_MANIFEST_SCHEMA = (
    "smartphone-r5-gsdc2023-native-fgo-v2-candidate-freeze-manifest.v1"
)
DEFAULT_FREEZE = (
    ROOT
    / "docs"
    / "use_cases"
    / "records"
    / "smartphone_r5_gsdc2023_native_fgo_v2_candidate_freeze.json"
)
DEFAULT_FREEZE_MANIFEST = DEFAULT_FREEZE.with_name(
    "smartphone_r5_gsdc2023_native_fgo_v2_candidate_freeze_manifest.json"
)
DEFAULT_ARCHIVE = ROOT / "data" / "gsdc2023" / "cache" / "dataset_2023.zip"
DEFAULT_OUTPUT = ROOT / "output" / "smartphone-r5" / "native-fgo-v2"
DEFAULT_PROFILE = ROOT / "configs" / "benchmarks" / "smartphone_r5_gsdc2023.json"
DEFAULT_ADAPTER = ROOT / "apps" / "commands" / "benchmarks" / "gnss_smartphone_gnss_adapter.py"
DEFAULT_PARITY = ROOT / "build" / "apps" / "gnss_fgo_parity"

TRAIN_IDS = (
    "2021-03-16-18-59-us-ca-mtv-a/pixel5",
    "2021-07-27-19-49-us-ca-mtv-b/pixel4",
    "2022-02-24-18-29-us-ca-lax-o/pixel5",
)
FRESH_VALIDATION_ID = "2023-05-09-21-32-us-ca-mtv-pe1/pixel5"
FUTURE_HOLDOUT_ID = "2023-05-16-19-54-us-ca-mtv-xe1/pixel5"
ARCHIVE_SHA256 = "bda30ab456e6fd6f83550c246e8dbd287306d5385f1f1069c99c16298e647408"
CENTRAL_METADATA_SHA256 = "5f619be94a00c33c3067f13b1cd3351d96f2804f07fa04eda8fb027739fb0992"
PROFILE_SHA256 = "273dfcc4e4636940d5216cca793a55773d9a04f668d1bfa5fcdc0013f4768776"
ADAPTER_SHA256 = "4a26dbbef0d5eff4a4840c43b600d790d573accc9977a9993754386ad086466b"
PARITY_SOURCE_SHA256 = "24e1a42d91b4e6e7a961e9dfb3c77bf08e54cc21ec9d17764a7f7bb171e624a2"
PARITY_BINARY_SHA256 = "3d5505509c78cf632328d93f6f941c9a55790b4b04dd272920cbf2e3e8f550ba"

GPS_EPOCH_UNIX_SECONDS = 315964800.0
LEAP_SECONDS = 18.0
SECONDS_PER_WEEK = 604800.0
MAX_OFFSET_SECONDS = 0.1
OFFSET_STEP_SECONDS = 0.01
MIN_PAIRED_SAMPLES = 20
PAIR_TOLERANCE_SECONDS = 0.20
SMOKE_EPOCHS = 2
MAX_STAGE_SECONDS = 900
MAX_ADDRESS_SPACE_BYTES = 8 * 1024 * 1024 * 1024
RSS_MARKER = "__GNSS_NATIVE_FGO_V2_RSS_KB__"
RSS_RE = re.compile(r"__GNSS_NATIVE_FGO_V2_RSS_KB__\s+(\d+)")


class V2SmokeError(ValueError):
    """Raised when the frozen v2 smoke contract cannot be proven."""


def _sha256(path: Path) -> str:
    if not path.is_file():
        raise V2SmokeError(f"missing file: {path}")
    digest = hashlib.sha256()
    try:
        with path.open("rb") as handle:
            for chunk in iter(lambda: handle.read(1024 * 1024), b""):
                digest.update(chunk)
    except OSError as exc:
        raise V2SmokeError(f"failed to hash {path}: {exc}") from exc
    return digest.hexdigest()


def _canonical_sha256(payload: Any) -> str:
    encoded = json.dumps(
        payload, ensure_ascii=True, sort_keys=True, separators=(",", ":")
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


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
            pass
    finally:
        temporary.unlink(missing_ok=True)


def _atomic_json(path: Path, payload: dict[str, Any]) -> None:
    _atomic_bytes(
        path,
        (json.dumps(payload, indent=2, sort_keys=True) + "\n").encode("utf-8"),
    )


def _load_json(path: Path, label: str) -> dict[str, Any]:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise V2SmokeError(f"invalid {label}: {path}") from exc
    if not isinstance(payload, dict):
        raise V2SmokeError(f"{label} must be a JSON object: {path}")
    return payload


def _relative(path: Path) -> str:
    try:
        return str(path.resolve().relative_to(ROOT.resolve()))
    except ValueError:
        return str(path)


def _safe_id(dataset_id: str) -> str:
    if dataset_id.count("/") != 1 or any(ch.isspace() for ch in dataset_id):
        raise V2SmokeError(f"invalid dataset ID: {dataset_id}")
    return dataset_id.replace("/", "__")


def _load_and_verify_freeze(
    freeze_path: Path, manifest_path: Path, archive_path: Path
) -> dict[str, Any]:
    freeze = _load_json(freeze_path, "v2 candidate freeze")
    if freeze.get("schema_version") != FREEZE_SCHEMA:
        raise V2SmokeError("v2 candidate freeze schema mismatch")
    if freeze.get("status") != "frozen-before-new-train-payload-materialization":
        raise V2SmokeError("v2 candidate freeze is not pre-materialization")
    manifest = _load_json(manifest_path, "v2 candidate freeze manifest")
    if manifest.get("schema_version") != FREEZE_MANIFEST_SCHEMA:
        raise V2SmokeError("v2 candidate freeze manifest schema mismatch")
    if manifest.get("candidate_freeze_record") != _relative(freeze_path):
        raise V2SmokeError("candidate freeze manifest record path mismatch")
    if manifest.get("candidate_freeze_record_sha256") != _sha256(freeze_path):
        raise V2SmokeError("candidate freeze record hash mismatch")
    if manifest.get("split_freeze_record_sha256") != "7e7c8d29bc385cefef947e56a653c8a19e76ac4df36eb3999bf154899187a2a9":
        raise V2SmokeError("split freeze hash mismatch")
    if manifest.get("truth_open_count_before_freeze") != 0:
        raise V2SmokeError("truth was opened before candidate freeze")
    if manifest.get("new_train_payload_materialized_before_freeze") is not False:
        raise V2SmokeError("new train payload was materialized before candidate freeze")
    if manifest.get("fresh_validation_materialized") is not False:
        raise V2SmokeError("fresh validation was materialized")
    if manifest.get("future_holdout_materialized") is not False:
        raise V2SmokeError("future holdout was materialized")
    if manifest.get("future_holdout_truth_open_count") != 0:
        raise V2SmokeError("future holdout truth was opened")
    archive_contract = freeze.get("archive")
    if not isinstance(archive_contract, dict):
        raise V2SmokeError("candidate freeze lacks archive contract")
    if archive_contract.get("path") != _relative(archive_path):
        raise V2SmokeError("archive path differs from candidate freeze")
    archive_hash = _sha256(archive_path)
    if archive_hash != ARCHIVE_SHA256 or archive_hash != archive_contract.get("sha256"):
        raise V2SmokeError("archive hash differs from candidate freeze")
    if archive_contract.get("central_metadata_sha256") != CENTRAL_METADATA_SHA256:
        raise V2SmokeError("central metadata digest differs from candidate freeze")
    source_files = freeze.get("source_files")
    if not isinstance(source_files, dict):
        raise V2SmokeError("candidate freeze lacks source hashes")
    expected_sources = {
        "apps/native/gnss_fgo_parity.cpp": PARITY_SOURCE_SHA256,
        "apps/commands/benchmarks/gnss_smartphone_gnss_adapter.py": ADAPTER_SHA256,
        "configs/benchmarks/smartphone_r5_gsdc2023.json": PROFILE_SHA256,
    }
    for name, expected in expected_sources.items():
        entry = source_files.get(name)
        if not isinstance(entry, dict) or entry.get("sha256") != expected:
            raise V2SmokeError(f"frozen source entry mismatch: {name}")
        if _sha256(ROOT / name) != expected:
            raise V2SmokeError(f"source hash changed: {name}")
    binary = freeze.get("binary")
    if not isinstance(binary, dict) or binary.get("path") != _relative(DEFAULT_PARITY):
        raise V2SmokeError("candidate parity binary path mismatch")
    if binary.get("sha256") != PARITY_BINARY_SHA256 or _sha256(DEFAULT_PARITY) != PARITY_BINARY_SHA256:
        raise V2SmokeError("candidate parity binary hash changed")
    split = freeze.get("split_freeze")
    if not isinstance(split, dict):
        raise V2SmokeError("candidate freeze lacks split")
    split_record_value = split.get("record")
    if not isinstance(split_record_value, str) or Path(split_record_value).is_absolute() or ".." in Path(split_record_value).parts:
        raise V2SmokeError("candidate split-freeze path is not repository-relative")
    split_record_path = ROOT / split_record_value
    if _sha256(split_record_path) != split.get("sha256"):
        raise V2SmokeError("split freeze record hash changed")
    split_record = _load_json(split_record_path, "v2 split inventory freeze")
    if split_record.get("schema_version") != "smartphone-r5-gsdc2023-native-fgo-v2-split-inventory-freeze.v1":
        raise V2SmokeError("split freeze schema mismatch")
    if tuple(split.get("train", ())) != TRAIN_IDS:
        raise V2SmokeError("candidate train split differs")
    if split.get("fresh_validation") != FRESH_VALIDATION_ID:
        raise V2SmokeError("candidate fresh validation differs")
    if split.get("future_holdout") != FUTURE_HOLDOUT_ID:
        raise V2SmokeError("candidate future holdout differs")
    if split.get("future_holdout_payload_or_truth_read") is not False:
        raise V2SmokeError("candidate freeze does not seal future holdout")
    truth_policy = freeze.get("truth_policy")
    if not isinstance(truth_policy, dict) or truth_policy.get("future_holdout_materialization_or_truth_open") is not False:
        raise V2SmokeError("truth policy is not fail-closed")
    recipe = freeze.get("recipe")
    if not isinstance(recipe, dict):
        raise V2SmokeError("candidate recipe is missing")
    if recipe.get("backend") != "gtsam" or recipe.get("pose3_state") is not True or recipe.get("combined_imu_factor") is not True:
        raise V2SmokeError("candidate recipe is not the frozen GTSAM IMU path")
    imu_contract = recipe.get("imu_input_contract")
    if not isinstance(imu_contract, dict) or imu_contract.get("nearest_accel_max_abs_offset_ms") != 25:
        raise V2SmokeError("IMU input pairing contract changed")
    smoke_window = recipe.get("smoke_window")
    if not isinstance(smoke_window, dict) or smoke_window.get("max_epochs") != SMOKE_EPOCHS:
        raise V2SmokeError("smoke epoch bound changed")
    # Central-directory member metadata lives in the identity freeze record;
    # keep it separate from this recipe record so the two hashes remain
    # independently auditable.
    freeze["_split_record"] = split_record
    return freeze


def _metadata(info: zipfile.ZipInfo) -> dict[str, Any]:
    return {
        "name": info.filename,
        "file_size": info.file_size,
        "compressed_size": info.compress_size,
        "crc32_hex": f"{info.CRC:08x}",
    }


def _member_names(dataset_id: str) -> dict[str, str]:
    route, phone = dataset_id.split("/", 1)
    prefix = f"dataset_2023/train/{route}"
    return {
        "device_gnss": f"{prefix}/{phone}/device_gnss.csv",
        "device_imu": f"{prefix}/{phone}/device_imu.csv",
        "ground_truth": f"{prefix}/{phone}/ground_truth.csv",
        "broadcast_nav": f"{prefix}/brdc.nav",
    }


def _central_index(archive_path: Path) -> dict[str, zipfile.ZipInfo]:
    index: dict[str, zipfile.ZipInfo] = {}
    duplicates: set[str] = set()
    try:
        with zipfile.ZipFile(archive_path) as archive:
            for info in archive.infolist():
                if info.filename in index:
                    duplicates.add(info.filename)
                index[info.filename] = info
    except (OSError, zipfile.BadZipFile) as exc:
        raise V2SmokeError("failed to inspect archive central directory") from exc
    if duplicates:
        raise V2SmokeError("archive contains duplicate central names")
    return index


def _expected_members(freeze: dict[str, Any], dataset_id: str) -> dict[str, dict[str, Any]]:
    split_record = freeze.get("_split_record", freeze)
    central = split_record.get("central_directory_metadata")
    if not isinstance(central, dict) or not isinstance(central.get(dataset_id), dict):
        raise V2SmokeError(f"freeze lacks central metadata for {dataset_id}")
    row = central[dataset_id]
    names = _member_names(dataset_id)
    expected: dict[str, dict[str, Any]] = {}
    for key, basename in (
        ("device_gnss", "device_gnss.csv"),
        ("device_imu", "device_imu.csv"),
        ("ground_truth", "ground_truth.csv"),
        ("broadcast_nav", "brdc.nav"),
    ):
        value = row.get(basename)
        if not isinstance(value, dict):
            raise V2SmokeError(f"freeze lacks {basename} metadata for {dataset_id}")
        expected[key] = {
            "name": names[key],
            "file_size": value.get("file_size"),
            "compressed_size": value.get("compressed_size"),
            "crc32_hex": value.get("crc32_hex"),
        }
    reference = row.get("reference_obs")
    if not isinstance(reference, dict) or not isinstance(reference.get("member"), str):
        raise V2SmokeError(f"freeze lacks reference RINEX metadata for {dataset_id}")
    expected["reference_obs"] = {
        "name": reference["member"],
        "file_size": reference.get("file_size"),
        "compressed_size": reference.get("compressed_size"),
        "crc32_hex": reference.get("crc32_hex"),
    }
    return expected


def _verify_central_members(
    index: dict[str, zipfile.ZipInfo], freeze: dict[str, Any], dataset_id: str
) -> dict[str, dict[str, Any]]:
    expected = _expected_members(freeze, dataset_id)
    verified: dict[str, dict[str, Any]] = {}
    for key, wanted in expected.items():
        info = index.get(wanted["name"])
        if info is None or info.is_dir():
            raise V2SmokeError(f"archive missing frozen {key}: {wanted['name']}")
        actual = _metadata(info)
        if actual != wanted:
            raise V2SmokeError(f"central metadata changed for {dataset_id} {key}")
        verified[key] = actual
    return verified


def _materialize_member(
    archive_path: Path,
    index: dict[str, zipfile.ZipInfo],
    expected: dict[str, Any],
    output: Path,
) -> dict[str, Any]:
    """Materialize one post-freeze payload member through atomic rename."""

    if output.exists():
        raise V2SmokeError(f"refusing to overwrite existing v2 input: {output}")
    output.parent.mkdir(parents=True, exist_ok=True)
    info = index.get(expected["name"])
    if info is None or _metadata(info) != expected:
        raise V2SmokeError(f"member metadata changed before materialization: {expected['name']}")
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{output.name}.", suffix=".tmp", dir=str(output.parent)
    )
    temporary = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "wb") as target:
            with zipfile.ZipFile(archive_path) as archive, archive.open(info, "r") as source:
                shutil.copyfileobj(source, target, length=1024 * 1024)
            target.flush()
            os.fsync(target.fileno())
        if temporary.stat().st_size != int(expected["file_size"]):
            raise V2SmokeError(f"materialized member size mismatch: {expected['name']}")
        os.replace(temporary, output)
    except (OSError, KeyError, zipfile.BadZipFile, V2SmokeError) as exc:
        temporary.unlink(missing_ok=True)
        if isinstance(exc, V2SmokeError):
            raise
        raise V2SmokeError(f"failed to materialize {expected['name']}") from exc
    return {
        "member": expected["name"],
        "path": _relative(output),
        "metadata": expected,
        "sha256": _sha256(output),
        "bytes": output.stat().st_size,
    }


def materialize_train_route(
    archive_path: Path,
    index: dict[str, zipfile.ZipInfo],
    freeze: dict[str, Any],
    dataset_id: str,
    destination_root: Path,
) -> dict[str, Any]:
    """Extract only truth-free payload files for one frozen train phone."""

    route_root = destination_root / "routes" / _safe_id(dataset_id)
    if route_root.exists():
        raise V2SmokeError(f"refusing to reuse an unsealed route directory: {route_root}")
    inputs = route_root / "inputs"
    expected = _verify_central_members(index, freeze, dataset_id)
    artifacts: dict[str, dict[str, Any]] = {}
    output_names = {
        "device_gnss": "device_gnss.csv",
        "device_imu": "device_imu.csv",
        "broadcast_nav": "brdc.nav",
        "reference_obs": "reference.obs",
    }
    for key, output_name in output_names.items():
        artifacts[key] = _materialize_member(
            archive_path, index, expected[key], inputs / output_name
        )
    manifest = {
        "schema_version": ROUTE_SCHEMA_VERSION,
        "status": "payload-materialized-truth-free",
        "dataset_id": dataset_id,
        "role": "new-v2-train",
        "truth_opened": False,
        "truth_materialized": False,
        "ground_truth_member_declared_but_not_materialized": expected["ground_truth"],
        "archive_sha256": ARCHIVE_SHA256,
        "central_metadata": expected,
        "inputs": artifacts,
    }
    route_root.mkdir(parents=True, exist_ok=True)
    _atomic_json(route_root / "materialization_manifest.json", manifest)
    manifest_hash = _sha256(route_root / "materialization_manifest.json")
    _atomic_bytes(route_root / "materialization_manifest.sha256", f"{manifest_hash}  materialization_manifest.json\n".encode("ascii"))
    return {
        "dataset_id": dataset_id,
        "root": route_root,
        "inputs": inputs,
        "manifest": route_root / "materialization_manifest.json",
        "manifest_sha256": manifest_hash,
        "artifacts": artifacts,
    }


def _normalise_header(name: str) -> str:
    return "".join(ch.lower() for ch in name.strip() if ch.isalnum())


def _find_column(headers: list[str], candidates: Iterable[str]) -> int | None:
    lookup = {_normalise_header(value): index for index, value in enumerate(headers)}
    for candidate in candidates:
        index = lookup.get(_normalise_header(candidate))
        if index is not None:
            return index
    return None


def _parse_float(value: str) -> float | None:
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return None
    return parsed if math.isfinite(parsed) else None


def _read_gnss_observable(path: Path) -> dict[int, tuple[float, float, float]]:
    required = ("utcTimeMillis", "WlsPositionXEcefMeters", "WlsPositionYEcefMeters", "WlsPositionZEcefMeters")
    epochs: dict[int, tuple[float, float, float]] = {}
    with path.open(encoding="utf-8-sig", newline="") as handle:
        reader = csv.DictReader(handle)
        fields = list(reader.fieldnames or ())
        missing = [name for name in required if name not in fields]
        if missing:
            raise V2SmokeError(f"GNSS observable missing fields: {', '.join(missing)}")
        for row_number, row in enumerate(reader, start=2):
            try:
                timestamp = int(float(row["utcTimeMillis"]))
            except (TypeError, ValueError):
                raise V2SmokeError(f"GNSS observable invalid timestamp at row {row_number}")
            coordinates = tuple(
                _parse_float(row[name])
                for name in required[1:]
            )
            if any(value is None for value in coordinates):
                continue
            point = tuple(float(value) for value in coordinates)
            previous = epochs.get(timestamp)
            if previous is not None and max(abs(a - b) for a, b in zip(previous, point)) > 1e-3:
                raise V2SmokeError(f"GNSS observable WLS position changes within epoch {timestamp}")
            epochs[timestamp] = point
    if len(epochs) < 2:
        raise V2SmokeError("GNSS observable has fewer than two finite WLS epochs")
    return dict(sorted(epochs.items()))


def _read_imu_activity(path: Path) -> list[tuple[float, float]]:
    """Read raw Android IMU activity keyed by UTC, without reading truth."""

    streams = _read_android_imu_streams(path)
    activity: list[tuple[float, float]] = []
    for timestamp_ms, accel in streams["UncalAccel"].items():
        gyro = streams["UncalGyro"].get(timestamp_ms)
        accel_norm = math.sqrt(sum(value * value for value in accel))
        gyro_norm = math.sqrt(sum(value * value for value in (gyro or (0.0, 0.0, 0.0))))
        value = abs(accel_norm - 9.80665) + 0.25 * gyro_norm
        timestamp = timestamp_ms / 1000.0 - GPS_EPOCH_UNIX_SECONDS + LEAP_SECONDS
        if math.isfinite(timestamp) and math.isfinite(value):
            activity.append((timestamp, value))
    activity.sort()
    if len(activity) < MIN_PAIRED_SAMPLES:
        raise V2SmokeError("IMU observable has too few finite samples")
    return activity


def _read_android_imu_streams(
    path: Path,
) -> dict[str, dict[int, tuple[float, float, float]]]:
    """Read only the two Android streams needed by the frozen adapter.

    UncalAccel is m/s^2 and UncalGyro is rad/s according to the Android
    sensor contract.  The raw rows remain untouched on disk; this function
    returns finite values grouped by their UTC millisecond key.
    """

    required = (
        "MessageType",
        "utcTimeMillis",
        "MeasurementX",
        "MeasurementY",
        "MeasurementZ",
    )
    streams: dict[str, dict[int, tuple[float, float, float]]] = {
        "UncalAccel": {},
        "UncalGyro": {},
    }
    with path.open(encoding="utf-8-sig", newline="") as handle:
        reader = csv.DictReader(handle)
        fields = list(reader.fieldnames or ())
        missing = [name for name in required if name not in fields]
        if missing:
            raise V2SmokeError(f"Android IMU missing fields: {', '.join(missing)}")
        for row_number, row in enumerate(reader, start=2):
            message_type = (row.get("MessageType") or "").strip()
            if message_type not in streams:
                continue
            try:
                timestamp = int(float(row["utcTimeMillis"]))
            except (TypeError, ValueError):
                continue
            values = tuple(
                _parse_float(row[name])
                for name in ("MeasurementX", "MeasurementY", "MeasurementZ")
            )
            if any(value is None for value in values):
                continue
            point = tuple(float(value) for value in values)
            previous = streams[message_type].get(timestamp)
            if previous is not None and max(abs(a - b) for a, b in zip(previous, point)) > 1e-9:
                raise V2SmokeError(
                    f"Android IMU has inconsistent duplicate {message_type} timestamp {timestamp}"
                )
            streams[message_type][timestamp] = point
    if not streams["UncalAccel"] or not streams["UncalGyro"]:
        raise V2SmokeError("Android IMU lacks a finite UncalAccel or UncalGyro stream")
    return streams


def _convert_android_imu_to_loadable(
    source: Path,
    output: Path,
    *,
    nearest_accel_max_abs_offset_ms: int = 25,
) -> dict[str, Any]:
    """Create the explicit GPST/metric CSV consumed by ``loadImuCsv``.

    The conversion is a schema adapter, not an estimator: gyro timestamps are
    the anchors, acceleration is selected by deterministic nearest-neighbour
    matching, and rows outside the frozen 25 ms bound are omitted.  No truth,
    position labels, interpolation, or learned threshold is used.
    """

    if nearest_accel_max_abs_offset_ms != 25:
        raise V2SmokeError("the v2 IMU schema contract fixes a 25 ms pairing bound")
    streams = _read_android_imu_streams(source)
    accel_times = sorted(streams["UncalAccel"])
    gyro_times = sorted(streams["UncalGyro"])
    paired: list[tuple[int, tuple[float, float, float], tuple[float, float, float], int]] = []
    omitted = 0
    for gyro_timestamp in gyro_times:
        index = bisect.bisect_left(accel_times, gyro_timestamp)
        choices = []
        if index < len(accel_times):
            choices.append(accel_times[index])
        if index:
            choices.append(accel_times[index - 1])
        if not choices:
            omitted += 1
            continue
        nearest = min(choices, key=lambda timestamp: (abs(timestamp - gyro_timestamp), timestamp))
        delta_ms = nearest - gyro_timestamp
        if abs(delta_ms) > nearest_accel_max_abs_offset_ms:
            omitted += 1
            continue
        paired.append(
            (
                gyro_timestamp,
                streams["UncalAccel"][nearest],
                streams["UncalGyro"][gyro_timestamp],
                delta_ms,
            )
        )
    if not paired:
        raise V2SmokeError("Android IMU conversion produced no paired samples")
    lines = [
        "GPS TOW (s),GPS Week,Acc X (m/s^2),Acc Y (m/s^2),Acc Z (m/s^2),"
        "Ang Rate X (deg/s),Ang Rate Y (deg/s),Ang Rate Z (deg/s)\n"
    ]
    for timestamp_ms, accel, gyro_radps, _delta_ms in paired:
        gpst = timestamp_ms / 1000.0 - GPS_EPOCH_UNIX_SECONDS + LEAP_SECONDS
        week = math.floor(gpst / SECONDS_PER_WEEK)
        tow = gpst - week * SECONDS_PER_WEEK
        gyro_degps = tuple(math.degrees(value) for value in gyro_radps)
        values = (tow, week, *accel, *gyro_degps)
        if not all(math.isfinite(float(value)) for value in values):
            raise V2SmokeError("Android IMU conversion produced a non-finite value")
        lines.append(
            f"{tow:.9f},{week},{accel[0]:.9f},{accel[1]:.9f},{accel[2]:.9f},"
            f"{gyro_degps[0]:.9f},{gyro_degps[1]:.9f},{gyro_degps[2]:.9f}\n"
        )
    _atomic_bytes(output, "".join(lines).encode("ascii"))
    offsets = [abs(delta) for *_, delta in paired]
    return {
        "path": _relative(output),
        "sha256": _sha256(output),
        "source_sha256": _sha256(source),
        "source_streams": {name: len(values) for name, values in streams.items()},
        "gyro_anchor_rows": len(gyro_times),
        "paired_rows": len(paired),
        "omitted_no_nearest_or_outside_bound": omitted,
        "nearest_accel_max_abs_offset_ms": nearest_accel_max_abs_offset_ms,
        "nearest_accel_abs_offset_ms": {
            "max": max(offsets),
            "median": statistics.median(offsets),
        },
        "units": "accel m/s^2; raw gyro rad/s converted once to output deg/s",
        "truth_used": False,
    }


def _pearson(first: list[float], second: list[float]) -> float | None:
    if len(first) < 2 or len(first) != len(second):
        return None
    mean_first = statistics.fmean(first)
    mean_second = statistics.fmean(second)
    centered_first = [value - mean_first for value in first]
    centered_second = [value - mean_second for value in second]
    denominator = math.sqrt(
        sum(value * value for value in centered_first)
        * sum(value * value for value in centered_second)
    )
    if denominator <= 0.0 or not math.isfinite(denominator):
        return None
    value = sum(a * b for a, b in zip(centered_first, centered_second)) / denominator
    return value if math.isfinite(value) else None


def _nearest_value(
    samples: list[tuple[float, float]],
    target: float,
    sample_times: list[float] | None = None,
) -> tuple[float, float] | None:
    """Return a bounded nearest sample with a deterministic time tie-break."""

    if not samples:
        return None
    times = sample_times if sample_times is not None else [item[0] for item in samples]
    index = bisect.bisect_left(times, target)
    candidates: list[tuple[float, float]] = []
    if index < len(samples):
        candidates.append(samples[index])
    if index:
        candidates.append(samples[index - 1])
    candidate = min(candidates, key=lambda item: (abs(item[0] - target), item[0]))
    return candidate if abs(candidate[0] - target) <= PAIR_TOLERANCE_SECONDS else None


def estimate_time_offset(
    gnss_epochs: dict[int, tuple[float, float, float]],
    imu_activity: list[tuple[float, float]],
) -> dict[str, Any]:
    """Estimate fixed IMU clock offset using an immutable observable grid.

    GNSS activity is the absolute change in finite-difference WLS speed; IMU
    activity is acceleration magnitude's gravity deviation plus a fixed 0.25
    gyro magnitude contribution.  The estimate is diagnostic only because the
    existing parity executable has no safe offset option.
    """

    gnss_times = sorted(gnss_epochs)
    gnss_seconds = [
        timestamp / 1000.0 - GPS_EPOCH_UNIX_SECONDS + LEAP_SECONDS
        for timestamp in gnss_times
    ]
    speeds: list[float] = [0.0]
    for before, after, dt in zip(gnss_times, gnss_times[1:], zip(gnss_seconds, gnss_seconds[1:])):
        dt_seconds = dt[1] - dt[0]
        if dt_seconds <= 0.0:
            speeds.append(float("nan"))
            continue
        first = gnss_epochs[before]
        second = gnss_epochs[after]
        distance = math.sqrt(sum((b - a) ** 2 for a, b in zip(first, second)))
        speeds.append(distance / dt_seconds)
    activity_change: list[float] = [0.0]
    for before, after in zip(speeds, speeds[1:]):
        activity_change.append(abs(after - before) if math.isfinite(after) and math.isfinite(before) else float("nan"))

    candidates: list[dict[str, Any]] = []
    imu_times = [sample[0] for sample in imu_activity]
    offset = -MAX_OFFSET_SECONDS
    while offset <= MAX_OFFSET_SECONDS + 1e-12:
        offset = round(offset, 6)
        observed: list[float] = []
        inertial: list[float] = []
        for gps_time, change in zip(gnss_seconds, activity_change):
            if not math.isfinite(change):
                continue
            # Positive offset means the IMU clock is moved later; query its
            # raw sample earlier by the same amount.
            nearest = _nearest_value(imu_activity, gps_time - offset, imu_times)
            if nearest is None:
                continue
            observed.append(change)
            inertial.append(nearest[1])
        correlation = _pearson(observed, inertial)
        candidates.append(
            {
                "offset_s": offset,
                "paired_samples": len(observed),
                "pearson_abs": abs(correlation) if correlation is not None else None,
                "pearson": correlation,
            }
        )
        offset += OFFSET_STEP_SECONDS
    usable = [
        row for row in candidates
        if row["paired_samples"] >= MIN_PAIRED_SAMPLES
        and row["pearson_abs"] is not None
        and math.isfinite(float(row["pearson_abs"]))
    ]
    if not usable:
        return {
            "status": "fallback-insufficient-observable-correlation",
            "estimated_offset_s": None,
            "physical_bound_abs_s": MAX_OFFSET_SECONDS,
            "minimum_paired_samples": MIN_PAIRED_SAMPLES,
            "pair_tolerance_s": PAIR_TOLERANCE_SECONDS,
            "candidate_count": len(candidates),
            "candidates": candidates,
            "solver_application": "not-applied; frozen native-fgo-v1 fallback",
        }
    best = min(
        usable,
        key=lambda row: (-float(row["pearson_abs"]), abs(float(row["offset_s"])), float(row["offset_s"])),
    )
    estimated = float(best["offset_s"])
    within_bound = abs(estimated) <= MAX_OFFSET_SECONDS + 1e-12
    return {
        "status": "observable-estimate-within-bound" if within_bound else "fallback-offset-out-of-bound",
        "estimated_offset_s": estimated if within_bound else None,
        "physical_bound_abs_s": MAX_OFFSET_SECONDS,
        "minimum_paired_samples": MIN_PAIRED_SAMPLES,
        "pair_tolerance_s": PAIR_TOLERANCE_SECONDS,
        "candidate_count": len(candidates),
        "best_paired_samples": best["paired_samples"],
        "best_pearson": best["pearson"],
        "best_pearson_abs": best["pearson_abs"],
        "candidates": candidates,
        "solver_application": (
            "not-applied; parity binary has no imu offset option; frozen native-fgo-v1 fallback"
        ),
    }


def _child_limits() -> None:
    resource.setrlimit(resource.RLIMIT_AS, (MAX_ADDRESS_SPACE_BYTES, MAX_ADDRESS_SPACE_BYTES))
    resource.setrlimit(resource.RLIMIT_CPU, (MAX_STAGE_SECONDS, MAX_STAGE_SECONDS + 1))


def _run_stage(
    stage: str,
    command: list[str],
    output_dir: Path,
    *,
    timeout_s: int = MAX_STAGE_SECONDS,
) -> dict[str, Any]:
    output_dir.mkdir(parents=True, exist_ok=True)
    stdout_path = output_dir / f"{stage}.stdout.log"
    stderr_path = output_dir / f"{stage}.stderr.log"
    started = time.perf_counter()
    before = resource.getrusage(resource.RUSAGE_CHILDREN).ru_maxrss
    environment = os.environ.copy()
    local_lib = "/home/sasaki/.local/lib"
    environment["LD_LIBRARY_PATH"] = (
        local_lib + os.pathsep + environment["LD_LIBRARY_PATH"]
        if environment.get("LD_LIBRARY_PATH")
        else local_lib
    )
    try:
        with stdout_path.open("w", encoding="utf-8") as stdout, stderr_path.open("w", encoding="utf-8") as stderr:
            completed = subprocess.run(
                command,
                cwd=ROOT,
                env=environment,
                stdin=subprocess.DEVNULL,
                stdout=stdout,
                stderr=stderr,
                timeout=timeout_s,
                preexec_fn=_child_limits,
                check=False,
            )
    except subprocess.TimeoutExpired:
        completed = None
    elapsed = time.perf_counter() - started
    after = resource.getrusage(resource.RUSAGE_CHILDREN).ru_maxrss
    stdout_text = stdout_path.read_text(encoding="utf-8", errors="replace") if stdout_path.exists() else ""
    stderr_text = stderr_path.read_text(encoding="utf-8", errors="replace") if stderr_path.exists() else ""
    rss_matches = RSS_RE.findall(stdout_text + "\n" + stderr_text)
    record = {
        "stage": stage,
        "command": command,
        "return_code": None if completed is None else completed.returncode,
        "timed_out": completed is None,
        "wall_time_s": elapsed,
        "peak_rss_kb": max(after, before),
        "stdout": {"path": _relative(stdout_path), "sha256": _sha256(stdout_path)},
        "stderr": {"path": _relative(stderr_path), "sha256": _sha256(stderr_path)},
    }
    if rss_matches:
        record["reported_peak_rss_kb"] = int(rss_matches[-1])
    return record


def _adapter_command(inputs: Path, output_dir: Path, dataset_id: str, phone: str, profile: dict[str, Any]) -> list[str]:
    archive = profile.get("archive")
    if not isinstance(archive, dict):
        raise V2SmokeError("profile archive contract missing")
    return [
        sys.executable,
        str(ROOT / "apps" / "gnss.py"),
        "smartphone-gnss-adapter",
        "--device-gnss", str(inputs / "device_gnss.csv"),
        "--output-dir", str(output_dir),
        "--dataset-id", dataset_id,
        "--device-model", phone,
        "--source-url", str(archive.get("url", "")),
        "--source-terms", str(archive.get("source_terms", "")),
        "--role", "development",
        "--truth-free",
        "--skip-epochs", "0",
        "--experimental-galileo-e1",
        "--experimental-galileo-e1-hatch-window-s", "30",
        "--broadcast-nav", str(inputs / "brdc.nav"),
    ]


def _parity_command(inputs: Path, adapter_dir: Path, parity_binary: Path) -> list[str]:
    return [
        str(parity_binary),
        "--rover", str(adapter_dir / "rover.obs"),
        "--base", str(inputs / "reference.obs"),
        "--nav", str(inputs / "brdc.nav"),
        "--imu", str(inputs / "imu_processed.csv"),
        "--max-epochs", str(SMOKE_EPOCHS),
        "--max-iters", "12",
        "--float-only",
    ]


def _parse_adapter_summary(path: Path) -> dict[str, Any]:
    summary = _load_json(path, "truth-free adapter summary")
    if summary.get("truth_free") is not True or summary.get("truth", {}).get("used") is not False:
        raise V2SmokeError("adapter summary records truth use")
    if summary.get("inputs", {}).get("ground_truth") is not None:
        raise V2SmokeError("truth-free adapter summary contains a truth input")
    return summary


def parse_parity_output(stdout: str) -> dict[str, Any]:
    """Parse only the factor/finite/convergence lines from parity stdout."""

    problem = re.search(
        r"FGOProblem:\s+epochs=(\d+).*dd_pr_factors=(\d+).*dd_cp_factors=(\d+).*sd_doppler_factors=(\d+).*ambiguity_states=(\d+).*pr_factors=(\d+)",
        stdout,
    )
    imu = re.search(
        r"imu:\s+([0-9.eE+-]+)\s+s,\s+iters=(\d+),\s+imu_intervals=(\d+),\s+initial_cost=([0-9.eE+\-]+),\s+final_cost=([0-9.eE+\-]+),\s+graph_values=(\d+),\s+graph_factors=(\d+)",
        stdout,
    )
    loaded = re.search(r"IMU:\s+loaded\s+(\d+)\s+samples", stdout)
    solve = re.search(r"solve_ok=(yes|no),\s+nonfinite_epochs=(\d+)", stdout)
    result = re.search(r"RESULT \(milestone 2b\): IMU-coupled solve (GO|NO-GO)", stdout)
    parsed: dict[str, Any] = {
        "problem_epochs": int(problem.group(1)) if problem else None,
        "dd_pr_factors": int(problem.group(2)) if problem else None,
        "dd_cp_factors": int(problem.group(3)) if problem else None,
        "sd_doppler_factors": int(problem.group(4)) if problem else None,
        "ambiguity_states": int(problem.group(5)) if problem else None,
        "pr_factors": int(problem.group(6)) if problem else None,
        "imu_samples": int(loaded.group(1)) if loaded else None,
        "imu_wall_s": float(imu.group(1)) if imu else None,
        "iterations": int(imu.group(2)) if imu else None,
        "imu_intervals": int(imu.group(3)) if imu else None,
        "initial_cost": float(imu.group(4)) if imu else None,
        "final_cost": float(imu.group(5)) if imu else None,
        "graph_values": int(imu.group(6)) if imu else None,
        "graph_factors": int(imu.group(7)) if imu else None,
        "solve_ok": solve.group(1) == "yes" if solve else None,
        "nonfinite_epochs": int(solve.group(2)) if solve else None,
        "milestone_2b": result.group(1) if result else None,
    }
    return parsed


def smoke_gate(stage: dict[str, Any], parsed: dict[str, Any]) -> dict[str, Any]:
    checks = {
        "return_code_zero": stage.get("return_code") == 0 and stage.get("timed_out") is False,
        "imu_samples_positive": isinstance(parsed.get("imu_samples"), int) and parsed["imu_samples"] > 0,
        "imu_intervals_positive": isinstance(parsed.get("imu_intervals"), int) and parsed["imu_intervals"] > 0,
        "graph_values_positive": isinstance(parsed.get("graph_values"), int) and parsed["graph_values"] > 0,
        "graph_factors_positive": isinstance(parsed.get("graph_factors"), int) and parsed["graph_factors"] > 0,
        "converged_yes": parsed.get("solve_ok") is True,
        "nonfinite_epochs_zero": parsed.get("nonfinite_epochs") == 0,
        "imu_factor_milestone_go": parsed.get("milestone_2b") == "GO",
    }
    return {"passed": all(checks.values()), "checks": checks}


def _profile(path: Path) -> dict[str, Any]:
    profile = _load_json(path, "smartphone profile")
    if profile.get("schema_version") != "smartphone-r5-profile.v1":
        raise V2SmokeError("smartphone profile schema mismatch")
    if _sha256(path) != PROFILE_SHA256:
        raise V2SmokeError("smartphone profile hash changed")
    return profile


def run_smoke(
    freeze_path: Path,
    manifest_path: Path,
    archive_path: Path,
    output_root: Path,
    *,
    parity_binary: Path = DEFAULT_PARITY,
    resume: bool = False,
) -> dict[str, Any]:
    freeze = _load_and_verify_freeze(freeze_path, manifest_path, archive_path)
    profile = _profile(DEFAULT_PROFILE)
    if _sha256(parity_binary) != PARITY_BINARY_SHA256:
        raise V2SmokeError("parity binary hash differs from frozen candidate")
    if output_root.exists() and not resume:
        raise V2SmokeError(f"output exists; use --resume only for sealed route artifacts: {output_root}")
    output_root.mkdir(parents=True, exist_ok=True)
    index = _central_index(archive_path)
    # Validate all selected central metadata before opening any selected member.
    for dataset_id in TRAIN_IDS:
        _verify_central_members(index, freeze, dataset_id)
    route_reports: list[dict[str, Any]] = []
    for dataset_id in TRAIN_IDS:
        route_started = time.perf_counter()
        route_key = _safe_id(dataset_id)
        route_root = output_root / "routes" / route_key
        sealed_route = route_root / "route_manifest.json"
        if resume and sealed_route.is_file():
            route_reports.append(_load_json(sealed_route, "sealed v2 route manifest"))
            continue
        if route_root.exists():
            raise V2SmokeError(f"partial route output requires --resume or cleanup: {route_root}")
        materialized = materialize_train_route(
            archive_path, index, freeze, dataset_id, output_root
        )
        inputs = materialized["inputs"]
        imu_conversion = _convert_android_imu_to_loadable(
            inputs / "device_imu.csv", inputs / "imu_processed.csv"
        )
        adapter_dir = materialized["root"] / "adapter"
        adapter_dir.mkdir(parents=True, exist_ok=True)
        phone = dataset_id.split("/", 1)[1]
        adapter_stage = _run_stage(
            "adapter",
            _adapter_command(inputs, adapter_dir, dataset_id, phone, profile),
            adapter_dir,
        )
        adapter_summary: dict[str, Any] | None = None
        adapter_error: str | None = None
        try:
            adapter_summary = _parse_adapter_summary(adapter_dir / "summary.json")
        except V2SmokeError as exc:
            adapter_error = str(exc)
        gnss_epochs: dict[int, tuple[float, float, float]] | None = None
        imu_activity: list[tuple[float, float]] | None = None
        offset_report: dict[str, Any] | None = None
        parity_stage: dict[str, Any] | None = None
        parity_report: dict[str, Any] = {}
        gate: dict[str, Any] = {"passed": False, "checks": {"adapter_truth_free": adapter_error is None}}
        if adapter_error is None and adapter_stage["return_code"] == 0 and not adapter_stage["timed_out"]:
            try:
                gnss_epochs = _read_gnss_observable(inputs / "device_gnss.csv")
                imu_activity = _read_imu_activity(inputs / "device_imu.csv")
                offset_report = estimate_time_offset(gnss_epochs, imu_activity)
                parity_stage = _run_stage(
                    "parity",
                    _parity_command(inputs, adapter_dir, parity_binary),
                    adapter_dir,
                )
                parity_stdout = (adapter_dir / "parity.stdout.log").read_text(
                    encoding="utf-8", errors="replace"
                )
                parity_report = parse_parity_output(parity_stdout)
                gate = smoke_gate(parity_stage, parity_report)
                gate["checks"]["adapter_truth_free"] = True
            except (OSError, V2SmokeError, ValueError) as exc:
                adapter_error = str(exc)
                gate = {"passed": False, "checks": {"adapter_truth_free": True}, "error": adapter_error}
        route_manifest = {
            "schema_version": ROUTE_SCHEMA_VERSION,
            "status": "truth-free-smoke-sealed",
            "candidate_id": freeze["candidate_id"],
            "dataset_id": dataset_id,
            "role": "new-v2-train",
            "truth_opened": False,
            "truth_materialized": False,
            "future_validation_or_holdout_touched": False,
            "materialization": {
                "manifest": _relative(materialized["manifest"]),
                "manifest_sha256": materialized["manifest_sha256"],
                "inputs": materialized["artifacts"],
                "imu_conversion": imu_conversion,
            },
            "stages": {"adapter": adapter_stage, "parity": parity_stage},
            "adapter_summary": adapter_summary,
            "adapter_error": adapter_error,
            "timestamp_alignment": offset_report,
            "parity": parity_report,
            "smoke_gate": gate,
            "fallback": (
                "frozen-native-fgo-v1"
                if not gate.get("passed")
                or (offset_report is not None and offset_report.get("estimated_offset_s") is None)
                else "not-selected; v2 offset remains diagnostic-only"
            ),
            "wall_time_s": time.perf_counter() - route_started,
            "source_hashes": {
                "freeze_record": _sha256(freeze_path),
                "freeze_manifest": _sha256(manifest_path),
                "profile": PROFILE_SHA256,
                "parity_binary": PARITY_BINARY_SHA256,
            },
        }
        _atomic_json(sealed_route, route_manifest)
        route_manifest_hash = _sha256(sealed_route)
        _atomic_bytes(route_root / "route_manifest.sha256", f"{route_manifest_hash}  route_manifest.json\n".encode("ascii"))
        route_manifest["route_manifest_sha256"] = route_manifest_hash
        route_reports.append(route_manifest)

    all_passed = bool(route_reports) and all(report.get("smoke_gate", {}).get("passed") is True for report in route_reports)
    final = {
        "schema_version": SCHEMA_VERSION,
        "status": "truth-free-train-smoke-sealed",
        "candidate_id": freeze["candidate_id"],
        "split": {
            "train": list(TRAIN_IDS),
            "fresh_validation": FRESH_VALIDATION_ID,
            "future_holdout": FUTURE_HOLDOUT_ID,
        },
        "truth_access": {
            "train_truth_opened": False,
            "fresh_validation_truth_opened": False,
            "future_holdout_truth_opened": False,
            "future_holdout_materialized": False,
            "truth_free_first": True,
        },
        "archive": {
            "path": _relative(archive_path),
            "sha256": ARCHIVE_SHA256,
            "central_metadata_sha256": CENTRAL_METADATA_SHA256,
            "selection_before_member_read": True,
        },
        "candidate": {
            "freeze_record": _relative(freeze_path),
            "freeze_record_sha256": _sha256(freeze_path),
            "freeze_manifest": _relative(manifest_path),
            "freeze_manifest_sha256": _sha256(manifest_path),
            "profile_sha256": PROFILE_SHA256,
            "parity_binary_sha256": PARITY_BINARY_SHA256,
            "parity_source_sha256": PARITY_SOURCE_SHA256,
            "adapter_source_sha256": ADAPTER_SHA256,
        },
        "route_reports": route_reports,
        "train_smoke_gate": {
            "passed": all_passed,
            "definition": "all three unchanged two-epoch truth-free runs must insert finite CombinedImuFactor intervals and converge",
            "no_truth_score_performed": True,
        },
        "promotion": {
            "decision": "blocked-pending-safe-full-route-integration" if all_passed else "no-go",
            "production_default_changed": False,
            "fresh_validation_opened": False,
            "reason": (
                "Existing parity CLI is base-assisted and has no reusable smartphone output or IMU offset application; bounded smoke proves factor wiring only."
                if all_passed
                else "At least one frozen train smoke failed; retain failure artifacts and do not open validation."
            ),
        },
    }
    _atomic_json(output_root / "train_smoke_manifest.json", final)
    final_hash = _sha256(output_root / "train_smoke_manifest.json")
    _atomic_bytes(output_root / "train_smoke_manifest.sha256", f"{final_hash}  train_smoke_manifest.json\n".encode("ascii"))
    final["train_smoke_manifest_sha256"] = final_hash
    return final


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(prog="gnss_smartphone_native_fgo_v2_smoke")
    parser.add_argument("--freeze", type=Path, default=DEFAULT_FREEZE)
    parser.add_argument("--freeze-manifest", type=Path, default=DEFAULT_FREEZE_MANIFEST)
    parser.add_argument("--archive", type=Path, default=DEFAULT_ARCHIVE)
    parser.add_argument("--output-root", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--parity-binary", type=Path, default=DEFAULT_PARITY)
    parser.add_argument("--resume", action="store_true", help="reuse only already sealed route manifests")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        result = run_smoke(
            args.freeze.resolve(),
            args.freeze_manifest.resolve(),
            args.archive.resolve(),
            args.output_root.resolve(),
            parity_binary=args.parity_binary.resolve(),
            resume=args.resume,
        )
    except (OSError, V2SmokeError, ValueError) as exc:
        print(f"native FGO v2 smoke failed closed: {exc}", file=sys.stderr)
        return 2
    print(json.dumps({
        "status": result["status"],
        "train_smoke_passed": result["train_smoke_gate"]["passed"],
        "route_count": len(result["route_reports"]),
        "truth_opened": result["truth_access"]["train_truth_opened"],
        "output": str(args.output_root / "train_smoke_manifest.json"),
    }, sort_keys=True))
    return 0 if result["train_smoke_gate"]["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
