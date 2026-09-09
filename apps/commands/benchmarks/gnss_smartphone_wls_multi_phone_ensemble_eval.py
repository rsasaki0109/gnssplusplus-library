#!/usr/bin/env python3
"""Research a truth-free multi-phone handset-WLS trajectory ensemble.

The route/phone roles and the maximum-three candidate methods are fixed in a
central-directory-only selection record.  Only the two train routes are
materialized before training truth is audited.  After the train-only method is
fixed, one validation route is generated truth-free and its artifacts are
hashed; validation truth is then opened once as an evaluation phase.  The
reserved next holdout is never materialized by this command.
"""

from __future__ import annotations

import argparse
import bisect
import csv
import hashlib
import io
import json
import math
import os
from pathlib import Path
import resource
import shutil
import sys
import tempfile
import time
from typing import Any
import zipfile

import numpy as np

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


SCHEMA_VERSION = "smartphone-r5-wls-multi-phone-ensemble-evaluation.v1"
MANIFEST_SCHEMA_VERSION = "smartphone-r5-wls-multi-phone-ensemble-manifest.v1"
TRUTH_FREE_SCHEMA_VERSION = "smartphone-r5-wls-multi-phone-ensemble-truth-free.v1"
DEFAULT_ARCHIVE = ROOT / "data" / "gsdc2023" / "cache" / "dataset_2023.zip"
DEFAULT_INVENTORY = (
    ROOT / "output" / "smartphone-r5" / "generalization-v6" / "archive_inventory.json"
)
DEFAULT_SELECTION = (
    ROOT
    / "docs"
    / "use_cases"
    / "records"
    / "smartphone_r5_wls_multi_phone_ensemble_selection.json"
)
DEFAULT_OUTPUT = ROOT / "output" / "smartphone-r5" / "wls-multi-phone-ensemble-v1"
SKIP_EPOCHS = 1
DEFAULT_ALIGNMENT_TOLERANCE_MS = 1
ALIGNMENT_TOLERANCE_MS = DEFAULT_ALIGNMENT_TOLERANCE_MS
V1_4_ALIGNMENT_TOLERANCE_MS = 10
TRUTH_MATCH_TOLERANCE_MS = 100
LEAP_SECONDS = 18
METHOD_IDS = (
    "coordinate_wise_ecef_median",
    "geometric_median_ecef",
    "trimmed_mean_ecef",
)
BASELINE_METHOD_ID = "single_phone_wls"
DIAGNOSTIC_KEYS = tuple(previous._DIAGNOSTIC_KEYS)
MAX_GEOMETRIC_ITERATIONS = 64
GEOMETRIC_TOLERANCE_M = 1.0e-6


class EnsembleError(ValueError):
    """Raised when the fixed multi-phone ensemble contract cannot be proven."""


def _sha256(path: Path) -> str:
    if not path.is_file():
        raise EnsembleError(f"missing artifact: {path}")
    digest = hashlib.sha256()
    try:
        with path.open("rb") as handle:
            for chunk in iter(lambda: handle.read(1024 * 1024), b""):
                digest.update(chunk)
    except OSError as exc:
        raise EnsembleError(f"failed to hash {path}: {exc}") from exc
    return digest.hexdigest()


def _artifact(path: Path) -> dict[str, Any]:
    return {"path": str(path), "sha256": _sha256(path), "bytes": path.stat().st_size}


def _load_json(path: Path, label: str) -> dict[str, Any]:
    if not path.is_file():
        raise EnsembleError(f"missing {label}: {path}")
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise EnsembleError(f"invalid {label}: {path}") from exc
    if not isinstance(payload, dict):
        raise EnsembleError(f"{label} must be a JSON object")
    return payload


def _atomic_json(path: Path, payload: dict[str, Any]) -> None:
    smoother._atomic_write(
        path,
        (json.dumps(payload, indent=2, sort_keys=True) + "\n").encode("utf-8"),
    )


def _safe_id(value: str) -> str:
    return value.replace("/", "__")


def _central_metadata(archive_path: Path, member: str) -> dict[str, Any]:
    """Inspect one ZIP central-directory entry without reading its payload."""

    try:
        with zipfile.ZipFile(archive_path) as archive:
            entries = [info for info in archive.infolist() if info.filename == member]
    except (OSError, zipfile.BadZipFile) as exc:
        raise EnsembleError(f"failed to inspect archive central directory: {exc}") from exc
    if len(entries) != 1 or entries[0].is_dir():
        raise EnsembleError(f"member is not a unique file: {member}")
    info = entries[0]
    return {
        "name": info.filename,
        "file_size": info.file_size,
        "compressed_size": info.compress_size,
        "crc32_hex": f"{info.CRC:08x}",
    }


def _member_names(dataset_id: str) -> dict[str, str]:
    route, phone = dataset_id.split("/", 1)
    return generalization._member_names(route, phone)


def _verify_selected_record(
    archive_path: Path,
    inventory_path: Path,
    selection_path: Path,
) -> tuple[dict[str, Any], dict[str, Any], str, str]:
    """Verify role selection and all selected central metadata only."""

    selection = _load_json(selection_path, "multi-phone ensemble selection")
    if selection.get("schema_version") != (
        "smartphone-r5-wls-multi-phone-ensemble-selection.v1"
    ):
        raise EnsembleError("multi-phone selection schema is invalid")
    if selection.get("status") != "selection-frozen-before-member-content-read":
        raise EnsembleError("multi-phone selection was not frozen before payload access")
    archive_contract = selection.get("archive")
    if not isinstance(archive_contract, dict):
        raise EnsembleError("selection lacks archive contract")
    inventory_hash = _sha256(inventory_path)
    if inventory_hash != archive_contract.get("central_directory_inventory_sha256"):
        raise EnsembleError("central-directory inventory hash changed")
    inventory = _load_json(inventory_path, "central-directory inventory")
    inventory_archive = inventory.get("archive")
    if not isinstance(inventory_archive, dict):
        raise EnsembleError("inventory lacks archive metadata")
    if inventory_archive.get("central_directory_only") is not True:
        raise EnsembleError("inventory is not central-directory-only")
    if inventory_archive.get("member_content_read") is not False:
        raise EnsembleError("inventory records payload access")
    archive_hash = _sha256(archive_path)
    if archive_hash != archive_contract.get("sha256"):
        raise EnsembleError("archive hash differs from selection")
    rows = {
        str(row.get("dataset_id")): row
        for row in inventory.get("train", {}).get("records", [])
        if isinstance(row, dict)
    }
    selected_routes: set[str] = set()
    selected_ids: set[str] = set()

    def verify_group(group: dict[str, Any], role: str) -> None:
        route = group.get("route")
        phones = group.get("phones")
        records = group.get("records")
        if not isinstance(route, str) or not isinstance(phones, list) or not isinstance(records, list):
            raise EnsembleError(f"{role} selection shape is invalid")
        if len(phones) < 2 or len(records) != len(phones):
            raise EnsembleError(f"{role} does not contain at least two phones")
        if route in selected_routes:
            raise EnsembleError(f"route is assigned to multiple roles: {route}")
        selected_routes.add(route)
        nav = group.get("central_directory_broadcast_nav")
        if not isinstance(nav, dict):
            raise EnsembleError(f"{role} lacks central broadcast navigation metadata")
        nav_name = nav.get("name")
        if not isinstance(nav_name, str):
            raise EnsembleError(f"{role} navigation member name is invalid")
        if _central_metadata(archive_path, nav_name) != nav:
            raise EnsembleError(f"{role} navigation central metadata changed")
        actual_phones: list[str] = []
        for entry in records:
            if not isinstance(entry, dict):
                raise EnsembleError(f"{role} record is invalid")
            dataset_id = entry.get("dataset_id")
            if not isinstance(dataset_id, str) or dataset_id in selected_ids:
                raise EnsembleError(f"{role} dataset ID is invalid or duplicated")
            expected_row = rows.get(dataset_id)
            if expected_row is None or expected_row.get("route") != route:
                raise EnsembleError(f"{role} dataset is absent from inventory: {dataset_id}")
            if not expected_row.get("required_files_complete"):
                raise EnsembleError(f"{role} dataset lacks required phone files: {dataset_id}")
            if not expected_row.get("broadcast_nav_present") or expected_row.get("broadcast_nav_duplicate_count"):
                raise EnsembleError(f"{role} navigation is missing or duplicated: {dataset_id}")
            phone = expected_row.get("phone")
            if phone not in phones or phone in actual_phones:
                raise EnsembleError(f"{role} phone list does not match inventory: {dataset_id}")
            names = _member_names(dataset_id)
            expected_device = entry.get("device_gnss")
            expected_truth = entry.get("ground_truth")
            if expected_device != expected_row.get("central_directory_files", {}).get("device_gnss.csv"):
                raise EnsembleError(f"{role} device central metadata changed: {dataset_id}")
            if expected_truth != expected_row.get("central_directory_files", {}).get("ground_truth.csv"):
                raise EnsembleError(f"{role} truth central metadata changed: {dataset_id}")
            if _central_metadata(archive_path, names["device_gnss"]) != expected_device:
                raise EnsembleError(f"{role} device archive metadata changed: {dataset_id}")
            if _central_metadata(archive_path, names["ground_truth"]) != expected_truth:
                raise EnsembleError(f"{role} truth archive metadata changed: {dataset_id}")
            actual_phones.append(phone)
            selected_ids.add(dataset_id)
        if sorted(actual_phones) != sorted(phones):
            raise EnsembleError(f"{role} phone ordering differs from inventory")

    train_routes = selection.get("train_routes")
    if not isinstance(train_routes, list) or len(train_routes) != 2:
        raise EnsembleError("selection must contain exactly two train routes")
    for group in train_routes:
        verify_group(group, "train")
    validation = selection.get("new_validation")
    holdout = selection.get("next_holdout")
    if not isinstance(validation, dict) or not isinstance(holdout, dict):
        raise EnsembleError("selection lacks validation or holdout")
    verify_group(validation, "validation")
    verify_group(holdout, "holdout")
    excluded = selection.get("excluded_route_ids")
    if not isinstance(excluded, list):
        raise EnsembleError("selection lacks the prior-route exclusion list")
    if any(route in excluded for route in selected_routes):
        raise EnsembleError("selected route collides with a prior-route exclusion")
    if holdout.get("materialization_forbidden") is not True or holdout.get("truth_open_forbidden") is not True:
        raise EnsembleError("next holdout is not sealed in the selection record")
    if selection.get("exclusion_contract", {}).get("all_previously_used_routes_excluded") is not True:
        raise EnsembleError("selection does not prove prior-route exclusion")
    return selection, inventory, archive_hash, inventory_hash


def _materialize_member(
    archive_path: Path,
    member: str,
    output: Path,
    expected_metadata: dict[str, Any],
) -> dict[str, Any]:
    """Materialize a selected train/validation member after central verification."""

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
                if actual_metadata != expected_metadata:
                    raise EnsembleError(f"central metadata changed while materializing {member}")
                with archive.open(member, "r") as source:
                    shutil.copyfileobj(source, temporary)
            temporary.flush()
            os.fsync(temporary.fileno())
        except (OSError, KeyError, zipfile.BadZipFile, EnsembleError) as exc:
            temporary_path.unlink(missing_ok=True)
            raise EnsembleError(f"failed to materialize {member}: {exc}") from exc
    if temporary_path.stat().st_size != int(expected_metadata["file_size"]):
        temporary_path.unlink(missing_ok=True)
        raise EnsembleError(f"materialized member size differs from central metadata: {member}")
    os.replace(temporary_path, output)
    return {"member": member, "metadata": expected_metadata, **_artifact(output)}


def _truth_read(path: Path) -> dict[int, tuple[float, float, float]]:
    return smoother_eval._read_truth(path)


def _p95(values: list[float]) -> float:
    if not values:
        return math.inf
    return float(kaggle._percentile_linear_n_minus_1(values, 0.95))


def _truth_vehicle_audit(
    truths: dict[str, dict[int, tuple[float, float, float]]],
    thresholds: dict[str, Any],
) -> dict[str, Any]:
    """Audit that train phone truth files describe the same vehicle path."""

    def nearest_timestamp(timestamps: list[int], target: int) -> int | None:
        index = bisect.bisect_left(timestamps, target)
        candidates: list[int] = []
        if index < len(timestamps):
            candidates.append(timestamps[index])
        if index > 0:
            candidates.append(timestamps[index - 1])
        return min(candidates, key=lambda value: (abs(value - target), value)) if candidates else None

    phones = sorted(truths)
    pairs: list[dict[str, Any]] = []
    for index, first_phone in enumerate(phones):
        for second_phone in phones[index + 1 :]:
            first = truths[first_phone]
            second = truths[second_phone]
            second_timestamps = sorted(second)
            # Different Android logging paths can carry a stable clock offset
            # (for example, 433 ms) while still describing the same vehicle.
            # Estimate that fixed offset from timestamps only, then enforce the
            # frozen tolerance on the residual alignment.  This keeps the
            # audit strict without treating a stable logger offset as a path
            # disagreement.
            offset_samples = [
                nearest_timestamp(second_timestamps, timestamp) - timestamp
                for timestamp in first
                if nearest_timestamp(second_timestamps, timestamp) is not None
            ]
            if not offset_samples:
                raise EnsembleError("train truth files have no timestamp overlap")
            estimated_offset = int(round(float(np.median(offset_samples))))
            horizontal: list[float] = []
            vertical: list[float] = []
            residual_offsets: list[float] = []
            for timestamp, first_value in first.items():
                nearest = nearest_timestamp(second_timestamps, timestamp + estimated_offset)
                if nearest is None:
                    continue
                residual = abs(nearest - (timestamp + estimated_offset))
                if residual > TRUTH_MATCH_TOLERANCE_MS:
                    continue
                residual_offsets.append(float(nearest - (timestamp + estimated_offset)))
                second_value = second[nearest]
                horizontal.append(
                    kaggle._wgs84_horizontal_distance_m(
                        first_value[0], first_value[1], second_value[0], second_value[1]
                    )
                )
                vertical.append(abs(first_value[2] - second_value[2]))
            coverage = len(horizontal) / max(len(first), len(second))
            row = {
                "phones": [first_phone, second_phone],
                "first_epochs": len(first),
                "second_epochs": len(second),
                "matched_epochs": len(horizontal),
                "coverage_ratio": coverage,
                "estimated_constant_time_offset_ms": estimated_offset,
                "offset_sample_count": len(offset_samples),
                "residual_offset_p50_ms": (
                    float(kaggle._percentile_linear_n_minus_1(residual_offsets, 0.50))
                    if residual_offsets
                    else None
                ),
                "residual_offset_p95_ms": _p95(residual_offsets) if residual_offsets else None,
                "horizontal_p50_m": float(kaggle._percentile_linear_n_minus_1(horizontal, 0.50)) if horizontal else None,
                "horizontal_p95_m": _p95(horizontal),
                "vertical_p95_m": _p95(vertical),
            }
            row["passed"] = (
                coverage >= float(thresholds["minimum_pairwise_coverage_ratio"])
                and row["horizontal_p95_m"] <= float(thresholds["maximum_pairwise_horizontal_p95_m"])
                and row["vertical_p95_m"] <= float(thresholds["maximum_pairwise_vertical_p95_m"])
            )
            pairs.append(row)
    if not pairs or not all(pair["passed"] for pair in pairs):
        raise EnsembleError("train truth files do not support the same-vehicle assumption")
    return {
        "truth_used_only_for_train_assumption_audit": True,
        "match_tolerance_ms": TRUTH_MATCH_TOLERANCE_MS,
        "thresholds": thresholds,
        "pairs": pairs,
        "passed": True,
    }


def _nearest_position(
    positions: list[smoother.PositionRow],
    timestamps: list[int],
    timestamp: int,
    tolerance_ms: int = ALIGNMENT_TOLERANCE_MS,
) -> tuple[smoother.PositionRow, int] | None:
    if (
        not isinstance(tolerance_ms, int)
        or isinstance(tolerance_ms, bool)
        or tolerance_ms < 0
    ):
        raise EnsembleError("alignment tolerance must be a non-negative integer")
    index = bisect.bisect_left(timestamps, timestamp)
    choices: list[tuple[int, smoother.PositionRow]] = []
    if index < len(positions):
        choices.append((abs(positions[index].timestamp_ms - timestamp), positions[index]))
    if index > 0:
        choices.append((abs(positions[index - 1].timestamp_ms - timestamp), positions[index - 1]))
    if not choices:
        return None
    delta, row = min(choices, key=lambda pair: (pair[0], pair[1].timestamp_ms))
    # Ties are deterministic: the earlier source timestamp wins because the
    # key orders by (absolute delta, source timestamp).  No interpolation or
    # extrapolation is ever performed.
    return (row, delta) if delta <= tolerance_ms else None


def _geometric_median(values: np.ndarray) -> np.ndarray:
    if values.ndim != 2 or values.shape[0] == 0 or not np.isfinite(values).all():
        raise EnsembleError("geometric median input is invalid")
    estimate = np.mean(values, axis=0)
    for _ in range(MAX_GEOMETRIC_ITERATIONS):
        distances = np.linalg.norm(values - estimate, axis=1)
        if not np.isfinite(distances).all():
            raise EnsembleError("geometric median distance is non-finite")
        zero = distances <= GEOMETRIC_TOLERANCE_M
        if np.any(zero):
            return values[np.flatnonzero(zero)[0]].copy()
        weights = 1.0 / distances
        next_estimate = np.sum(values * weights[:, None], axis=0) / np.sum(weights)
        if not np.isfinite(next_estimate).all():
            raise EnsembleError("geometric median result is non-finite")
        if float(np.linalg.norm(next_estimate - estimate)) <= GEOMETRIC_TOLERANCE_M:
            return next_estimate
        estimate = next_estimate
    return estimate


def _fuse(values: list[np.ndarray], method_id: str) -> np.ndarray:
    matrix = np.asarray(values, dtype=float)
    if matrix.ndim != 2 or matrix.shape[0] == 0 or matrix.shape[1] != 3:
        raise EnsembleError(f"{method_id}: no usable phone coordinates")
    if not np.isfinite(matrix).all():
        raise EnsembleError(f"{method_id}: non-finite phone coordinate")
    if method_id == "coordinate_wise_ecef_median":
        result = np.median(matrix, axis=0)
    elif method_id == "geometric_median_ecef":
        result = _geometric_median(matrix)
    elif method_id == "trimmed_mean_ecef":
        if matrix.shape[0] >= 5:
            trim = int(math.floor(matrix.shape[0] * 0.2))
            ordered = np.sort(matrix, axis=0)
            result = np.mean(ordered[trim : matrix.shape[0] - trim], axis=0)
        else:
            result = np.mean(matrix, axis=0)
    else:
        raise EnsembleError(f"unknown fixed ensemble method: {method_id}")
    result = np.asarray(result, dtype=float)
    if result.shape != (3,) or not np.isfinite(result).all():
        raise EnsembleError(f"{method_id}: fused ECEF is non-finite")
    return result


def _position_from_ecef(
    timestamp_ms: int, ecef: np.ndarray, source_line: int, satellites: int
) -> smoother.PositionRow:
    latitude, longitude, height = smoother._wgs84_ecef_to_geodetic(ecef)
    week, tow = smoother._device_time_to_week_tow(timestamp_ms, LEAP_SECONDS)
    return smoother.PositionRow(
        week=week,
        tow=tow,
        timestamp_ms=timestamp_ms,
        ecef=ecef,
        latitude=math.degrees(latitude),
        longitude=math.degrees(longitude),
        height=height,
        status=1,
        satellites=satellites,
        pdop=0.0,
        ratio=0.0,
        fixed_ambiguities=0,
        iterations=0,
        source_line=source_line,
    )


def _fused_positions(
    positions_by_phone: dict[str, list[smoother.PositionRow]],
    method_id: str,
    timestamp_keys_by_phone: dict[str, list[int]] | None = None,
    alignment_tolerance_ms: int = ALIGNMENT_TOLERANCE_MS,
) -> tuple[dict[str, list[smoother.PositionRow]], dict[str, Any]]:
    if (
        not isinstance(alignment_tolerance_ms, int)
        or isinstance(alignment_tolerance_ms, bool)
        or alignment_tolerance_ms < 0
    ):
        raise EnsembleError("alignment tolerance must be a non-negative integer")
    phone_names = sorted(positions_by_phone)
    timestamps_by_phone = {
        phone: [row.timestamp_ms for row in positions_by_phone[phone]]
        for phone in phone_names
    }
    if timestamp_keys_by_phone is None:
        target_timestamps_by_phone = timestamps_by_phone
    else:
        if set(timestamp_keys_by_phone) != set(phone_names):
            raise EnsembleError("alignment timestamp phone set differs from WLS phone set")
        target_timestamps_by_phone = {}
        for phone in phone_names:
            keys = [int(timestamp) for timestamp in timestamp_keys_by_phone[phone]]
            if keys != sorted(set(keys)):
                raise EnsembleError(f"alignment timestamp keys are not increasing for {phone}")
            if not keys:
                raise EnsembleError(f"alignment timestamp keys are empty for {phone}")
            target_timestamps_by_phone[phone] = keys
    outputs: dict[str, list[smoother.PositionRow]] = {}
    count_histogram: dict[str, int] = {}
    offset_values: list[float] = []
    spread_values: list[float] = []
    missing_total = 0
    target_total = 0
    epoch_manifest: list[dict[str, Any]] = []
    for target_phone in phone_names:
        output_rows: list[smoother.PositionRow] = []
        for target_timestamp in target_timestamps_by_phone[target_phone]:
            target_total += 1
            matches: list[tuple[str, smoother.PositionRow, int]] = []
            for phone in phone_names:
                nearest = _nearest_position(
                    positions_by_phone[phone],
                    timestamps_by_phone[phone],
                    target_timestamp,
                    alignment_tolerance_ms,
                )
                if nearest is None:
                    continue
                row, delta = nearest
                matches.append((phone, row, delta))
                if phone != target_phone:
                    offset_values.append(float(delta))
            if not matches:
                raise EnsembleError(
                    "alignment unresolved: all phones are missing WLS at "
                    f"{target_phone}/{target_timestamp}"
                )
            missing_total += len(phone_names) - len(matches)
            count_histogram[str(len(matches))] = count_histogram.get(str(len(matches)), 0) + 1
            values = [row.ecef for _, row, _ in matches]
            fused = _fuse(values, method_id)
            spread = max(
                (float(np.linalg.norm(first - second)) for first in values for second in values),
                default=0.0,
            )
            spread_values.append(spread)
            output_rows.append(
                _position_from_ecef(
                    target_timestamp,
                    fused,
                    min(row.source_line for _, row, _ in matches),
                    min(row.satellites for _, row, _ in matches),
                )
            )
            target_wls_available = any(phone == target_phone for phone, _, _ in matches)
            epoch_manifest.append(
                {
                    "target_phone": target_phone,
                    "timestamp_ms": target_timestamp,
                    "used_phones": [phone for phone, _, _ in matches],
                    "phone_count": len(matches),
                    "target_wls_available": target_wls_available,
                    "fallback": len(matches) == 1,
                    "source": "target_wls_and_peers"
                    if target_wls_available and len(matches) > 1
                    else "target_wls_only"
                    if target_wls_available
                    else "peer_wls_sparse_recovery",
                    "spread_ecef_m": spread,
                }
            )
        outputs[target_phone] = output_rows
    return outputs, {
        "alignment_tolerance_ms": alignment_tolerance_ms,
        "target_epoch_count": target_total,
        "covered_target_epoch_count": target_total,
        "unresolved_target_epoch_count": 0,
        "target_coverage_ratio": 1.0,
        "all_target_timestamps_covered_by_source": True,
        "extrapolation_used": False,
        "alignment_source_policy": (
            "nearest finite WLS source at or within tolerance; peer recovery is allowed; "
            "no interpolation or extrapolation"
        ),
        "phone_count_histogram": count_histogram,
        "missing_phone_observations": missing_total,
        "max_abs_alignment_offset_ms": max(offset_values, default=0.0),
        "spread_ecef_m": {
            "p50_m": float(kaggle._percentile_linear_n_minus_1(spread_values, 0.50)) if spread_values else None,
            "p95_m": _p95(spread_values) if spread_values else None,
            "max_m": max(spread_values, default=0.0),
        },
        "epoch_manifest": epoch_manifest,
    }


def _combined_submission_bytes(paths: list[Path]) -> bytes:
    rows: list[tuple[str, int, str, str]] = []
    for path in paths:
        try:
            with path.open(encoding="utf-8-sig", newline="") as handle:
                reader = csv.DictReader(handle)
                if tuple(reader.fieldnames or ()) != kaggle.SUBMISSION_FIELDS:
                    raise EnsembleError(f"individual submission header is invalid: {path}")
                for raw in reader:
                    rows.append(
                        (
                            str(raw["phone"]),
                            int(raw["UnixTimeMillis"]),
                            str(raw["LatitudeDegrees"]),
                            str(raw["LongitudeDegrees"]),
                        )
                    )
        except (OSError, KeyError, TypeError, ValueError) as exc:
            raise EnsembleError(f"failed to combine submissions: {path}") from exc
    rows.sort(key=lambda row: (row[0], row[1]))
    buffer = io.StringIO(newline="")
    writer = csv.writer(buffer, lineterminator="\n")
    writer.writerow(kaggle.SUBMISSION_FIELDS)
    writer.writerows(rows)
    return buffer.getvalue().encode("utf-8")


def _write_method_outputs(
    route_dir: Path,
    dataset_records: list[dict[str, Any]],
    device_paths: dict[str, Path],
    positions_by_phone: dict[str, list[smoother.PositionRow]],
    method_id: str,
    alignment: dict[str, Any],
    timestamp_keys_by_phone: dict[str, list[int]] | None = None,
    alignment_tolerance_ms: int = ALIGNMENT_TOLERANCE_MS,
) -> tuple[dict[str, list[smoother.PositionRow]], dict[str, Any]]:
    if method_id == BASELINE_METHOD_ID:
        output_positions = {phone: list(rows) for phone, rows in positions_by_phone.items()}
        alignment = {
            "alignment_tolerance_ms": 0,
            "target_epoch_count": sum(len(rows) for rows in output_positions.values()),
            "covered_target_epoch_count": sum(len(rows) for rows in output_positions.values()),
            "unresolved_target_epoch_count": 0,
            "target_coverage_ratio": 1.0,
            "all_target_timestamps_covered_by_source": True,
            "extrapolation_used": False,
            "alignment_source_policy": "raw single-phone WLS; no extrapolation",
            "phone_count_histogram": {"1": sum(len(rows) for rows in output_positions.values())},
            "missing_phone_observations": 0,
            "max_abs_alignment_offset_ms": 0.0,
            "spread_ecef_m": {"p50_m": 0.0, "p95_m": 0.0, "max_m": 0.0},
            "baseline": True,
        }
    else:
        output_positions, alignment = _fused_positions(
            positions_by_phone,
            method_id,
            timestamp_keys_by_phone=timestamp_keys_by_phone,
            alignment_tolerance_ms=alignment_tolerance_ms,
        )
    method_dir = route_dir / "methods" / method_id
    phone_outputs: list[dict[str, Any]] = []
    individual_submissions: list[Path] = []
    for record in dataset_records:
        dataset_id = str(record["dataset_id"])
        phone = dataset_id.split("/", 1)[1]
        phone_dir = method_dir / phone
        position_path = phone_dir / "fused.pos"
        submission_path = phone_dir / "submission.csv"
        submission_manifest_path = phone_dir / "submission.manifest.json"
        wls._write_pos(position_path, output_positions[phone])
        submission_manifest = kaggle.generate_submission(
            position_path,
            submission_path,
            phone,
            device_gnss_path=device_paths[phone],
            dataset_id=dataset_id,
            skip_epochs=SKIP_EPOCHS,
            gps_utc_leap_seconds=LEAP_SECONDS,
            manifest_path=submission_manifest_path,
        )
        individual_submissions.append(submission_path)
        phone_outputs.append(
            {
                "phone": phone,
                "dataset_id": dataset_id,
                "position": _artifact(position_path),
                "submission": _artifact(submission_path),
                "submission_manifest": _artifact(submission_manifest_path),
                "submission_schema": submission_manifest.get("schema_version"),
                "output_epochs": len(output_positions[phone]),
            }
        )
    combined_path = method_dir / "submission.csv"
    combined_manifest_path = method_dir / "ensemble_manifest.json"
    smoother._atomic_write(combined_path, _combined_submission_bytes(individual_submissions))
    combined_manifest = {
        "schema_version": MANIFEST_SCHEMA_VERSION,
        "truth_free": True,
        "truth_used": False,
        "method": method_id,
        "phones": sorted(positions_by_phone),
        "alignment": alignment,
        "phone_outputs": phone_outputs,
        "combined_submission": _artifact(combined_path),
        "inputs": {
            "device_gnss": {
                phone: _artifact(device_paths[phone]) for phone in sorted(device_paths)
            }
        },
    }
    _atomic_json(combined_manifest_path, combined_manifest)
    return output_positions, {
        "method": method_id,
        "alignment": alignment,
        "phone_outputs": phone_outputs,
        "combined_submission": _artifact(combined_path),
        "manifest": _artifact(combined_manifest_path),
    }


def _score_positions(
    positions: list[smoother.PositionRow], truth: dict[int, tuple[float, float, float]]
) -> dict[str, Any]:
    epochs = [row.timestamp_ms for row in positions]
    rows = previous._raw_rows(positions)
    return previous._score(rows, positions, epochs, truth)


def _metric(metrics: dict[str, Any], path: tuple[str, ...]) -> float:
    value: Any = metrics
    try:
        for key in path:
            value = value[key]
        number = float(value)
    except (KeyError, TypeError, ValueError):
        return math.inf
    return number if math.isfinite(number) else math.inf


def _aggregate(phone_metrics: dict[str, dict[str, Any]]) -> dict[str, Any]:
    if not phone_metrics:
        raise EnsembleError("cannot aggregate empty phone metrics")
    values = list(phone_metrics.values())
    output: dict[str, Any] = {
        "phone_count": len(values),
        "mean_availability_ratio": sum(_metric(value, ("availability_ratio",)) for value in values) / len(values),
        "mean_truth_coverage_ratio": sum(_metric(value, ("truth_coverage_ratio",)) for value in values) / len(values),
        "mean_horizontal_wgs84_p50_m": sum(_metric(value, ("horizontal_wgs84_m", "p50_m")) for value in values) / len(values),
        "mean_horizontal_wgs84_p95_m": sum(_metric(value, ("horizontal_wgs84_m", "p95_m")) for value in values) / len(values),
        "mean_vertical_p95_abs_m": sum(_metric(value, ("vertical_p95_abs_m",)) for value in values) / len(values),
        "mean_kaggle_diagnostic_score_variants_m": {
            key: sum(_metric(value, ("kaggle_diagnostic_score_variants_m", key)) for value in values) / len(values)
            for key in DIAGNOSTIC_KEYS
        },
    }
    output["mean_kaggle_diagnostic_m"] = sum(
        output["mean_kaggle_diagnostic_score_variants_m"].values()
    ) / len(DIAGNOSTIC_KEYS)
    return output


def _score_methods(
    route_result: dict[str, Any], truths: dict[str, dict[int, tuple[float, float, float]]]
) -> dict[str, Any]:
    methods: dict[str, Any] = {}
    for method_id, positions_by_phone in route_result["method_positions"].items():
        phone_metrics = {
            phone: _score_positions(positions_by_phone[phone], truths[phone])
            for phone in sorted(positions_by_phone)
        }
        methods[method_id] = {
            "phone_metrics": phone_metrics,
            "aggregate": _aggregate(phone_metrics),
        }
    return methods


def _write_route(
    archive_path: Path,
    route_spec: dict[str, Any],
    phase_root: Path,
    *,
    truth_materialized: bool,
    alignment_tolerance_ms: int = ALIGNMENT_TOLERANCE_MS,
) -> dict[str, Any]:
    if (
        not isinstance(alignment_tolerance_ms, int)
        or isinstance(alignment_tolerance_ms, bool)
        or alignment_tolerance_ms < 0
    ):
        raise EnsembleError("alignment tolerance must be a non-negative integer")
    route = str(route_spec["route"])
    route_root = phase_root / _safe_id(route)
    input_root = route_root / "inputs"
    input_root.mkdir(parents=True, exist_ok=True)
    device_paths: dict[str, Path] = {}
    truth_paths: dict[str, Path] = {}
    materialized: dict[str, Any] = {}
    for record in route_spec["records"]:
        dataset_id = str(record["dataset_id"])
        phone = dataset_id.split("/", 1)[1]
        names = _member_names(dataset_id)
        device_path = input_root / phone / "device_gnss.csv"
        device_paths[phone] = device_path
        materialized[f"{phone}/device_gnss"] = _materialize_member(
            archive_path, names["device_gnss"], device_path, record["device_gnss"]
        )
        if truth_materialized:
            truth_path = input_root / phone / "ground_truth.csv"
            truth_paths[phone] = truth_path
            materialized[f"{phone}/ground_truth"] = _materialize_member(
                archive_path, names["ground_truth"], truth_path, record["ground_truth"]
            )
    positions_by_phone: dict[str, list[smoother.PositionRow]] = {}
    wls_artifacts: dict[str, Any] = {}
    for record in route_spec["records"]:
        dataset_id = str(record["dataset_id"])
        phone = dataset_id.split("/", 1)[1]
        wls_root = route_root / "wls" / phone
        wls_payload = wls.extract_to_directory(
            device_paths[phone],
            wls_root,
            skip_epochs=SKIP_EPOCHS,
            role="development",
            dataset_id=dataset_id,
        )
        positions = smoother._read_positions(wls_root / "wls.pos", LEAP_SECONDS)
        if not positions:
            raise EnsembleError(f"WLS produced no positions for {dataset_id}")
        positions_by_phone[phone] = positions
        wls_artifacts[phone] = {
            "payload": wls_payload,
            "position": _artifact(wls_root / "wls.pos"),
            "manifest": _artifact(wls_root / "wls_manifest.json"),
            "summary": _artifact(wls_root / "wls_summary.json"),
        }
    method_positions: dict[str, dict[str, list[smoother.PositionRow]]] = {}
    method_artifacts: dict[str, Any] = {}
    for method_id in (BASELINE_METHOD_ID, *METHOD_IDS):
        alignment = {}
        output_positions, artifacts = _write_method_outputs(
            route_root,
            route_spec["records"],
            device_paths,
            positions_by_phone,
            method_id,
            alignment,
            alignment_tolerance_ms=alignment_tolerance_ms,
        )
        method_positions[method_id] = output_positions
        method_artifacts[method_id] = artifacts
    route_manifest = {
        "schema_version": MANIFEST_SCHEMA_VERSION,
        "route": route,
        "phones": sorted(device_paths),
        "truth_free": not truth_materialized,
        "truth_used": False,
        "materialized": materialized,
        "wls": wls_artifacts,
        "methods": method_artifacts,
        "alignment_contract": {
            "unix_time_millis": "strict selected device keys; cross-phone nearest match within the frozen tolerance",
            "tolerance_ms": alignment_tolerance_ms,
            "tie_rule": "equal-distance source timestamps choose the earlier timestamp",
            "extrapolation": "forbidden",
            "missing_phone_policy": "record count/spread and use all finite available phone coordinates",
        },
    }
    route_manifest_path = route_root / "route_manifest.json"
    _atomic_json(route_manifest_path, route_manifest)
    return {
        "route": route,
        "route_root": route_root,
        "device_paths": device_paths,
        "truth_paths": truth_paths,
        "materialized": materialized,
        "positions_by_phone": positions_by_phone,
        "method_positions": method_positions,
        "method_artifacts": method_artifacts,
        "wls_artifacts": wls_artifacts,
        "route_manifest_path": route_manifest_path,
    }


def _gate(
    candidate: dict[str, Any],
    baseline: dict[str, Any],
    thresholds: dict[str, Any],
) -> dict[str, Any]:
    failures: list[str] = []
    candidate_aggregate = candidate["aggregate"]
    baseline_aggregate = baseline["aggregate"]
    if candidate_aggregate["mean_availability_ratio"] < float(thresholds["availability_min"]):
        failures.append("aggregate_availability_below_signoff")
    if candidate_aggregate["mean_truth_coverage_ratio"] < float(thresholds["truth_coverage_min"]):
        failures.append("aggregate_truth_coverage_below_signoff")
    if candidate_aggregate["mean_horizontal_wgs84_p50_m"] > float(thresholds["horizontal_median_max_m"]):
        failures.append("aggregate_horizontal_median_signoff")
    if candidate_aggregate["mean_horizontal_wgs84_p95_m"] > float(thresholds["horizontal_p95_max_m"]):
        failures.append("aggregate_horizontal_p95_signoff")
    if candidate_aggregate["mean_vertical_p95_abs_m"] > float(thresholds["vertical_p95_max_m"]):
        failures.append("aggregate_vertical_p95_signoff")
    if candidate_aggregate["mean_availability_ratio"] < baseline_aggregate["mean_availability_ratio"] - 1e-12:
        failures.append("availability_regression_vs_single_phone_wls")
    if candidate_aggregate["mean_truth_coverage_ratio"] < baseline_aggregate["mean_truth_coverage_ratio"] - 1e-12:
        failures.append("coverage_regression_vs_single_phone_wls")
    for key, candidate_key, baseline_key in (
        ("horizontal_p50", "mean_horizontal_wgs84_p50_m", "mean_horizontal_wgs84_p50_m"),
        ("horizontal_p95", "mean_horizontal_wgs84_p95_m", "mean_horizontal_wgs84_p95_m"),
        ("vertical_p95", "mean_vertical_p95_abs_m", "mean_vertical_p95_abs_m"),
    ):
        if candidate_aggregate[candidate_key] > baseline_aggregate[baseline_key] + 1e-12:
            failures.append(f"{key}_regression_vs_single_phone_wls")
    diagnostic_deltas: dict[str, float] = {}
    for key in DIAGNOSTIC_KEYS:
        delta = (
            candidate_aggregate["mean_kaggle_diagnostic_score_variants_m"][key]
            - baseline_aggregate["mean_kaggle_diagnostic_score_variants_m"][key]
        )
        diagnostic_deltas[key] = delta
        if delta > 1e-12:
            failures.append(f"{key}_regression_vs_single_phone_wls")
    strict_improvement = any(
        candidate_aggregate[key] < baseline_aggregate[key] - 1e-12
        for key in (
            "mean_horizontal_wgs84_p50_m",
            "mean_horizontal_wgs84_p95_m",
            "mean_vertical_p95_abs_m",
        )
    ) or any(value < -1e-12 for value in diagnostic_deltas.values())
    if not strict_improvement:
        failures.append("no_strict_improvement_vs_single_phone_wls")
    return {
        "passed": not failures,
        "failures": failures,
        "diagnostic_deltas_m": diagnostic_deltas,
        "strict_improvement": strict_improvement,
        "reference": BASELINE_METHOD_ID,
    }


def _rank_train(method_id: str, score: dict[str, Any]) -> tuple[float, ...]:
    aggregate = score["aggregate"]
    return (
        float(aggregate["mean_kaggle_diagnostic_m"]),
        float(aggregate["mean_horizontal_wgs84_p95_m"]),
        float(aggregate["mean_horizontal_wgs84_p50_m"]),
        float(aggregate["mean_vertical_p95_abs_m"]),
        METHOD_IDS.index(method_id) if method_id in METHOD_IDS else -1,
    )


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        prog=os.environ.get("GNSS_CLI_NAME", "gnss smartphone-wls-multi-phone-ensemble-eval")
    )
    parser.add_argument("--archive", type=Path, default=DEFAULT_ARCHIVE)
    parser.add_argument("--inventory", type=Path, default=DEFAULT_INVENTORY)
    parser.add_argument("--selection-record", type=Path, default=DEFAULT_SELECTION)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument(
        "--alignment-tolerance-ms",
        type=int,
        default=ALIGNMENT_TOLERANCE_MS,
        help="frozen non-negative nearest-source tolerance in milliseconds",
    )
    return parser.parse_args(argv)


def run(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    started = time.perf_counter()
    rss_before = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss
    try:
        if (
            not isinstance(args.alignment_tolerance_ms, int)
            or isinstance(args.alignment_tolerance_ms, bool)
            or args.alignment_tolerance_ms < 0
        ):
            raise EnsembleError("alignment tolerance must be a non-negative integer")
        if args.output_dir.exists() and any(args.output_dir.iterdir()):
            raise EnsembleError(f"refusing to reuse non-empty output: {args.output_dir}")
        selection, inventory, archive_hash, inventory_hash = _verify_selected_record(
            args.archive, args.inventory, args.selection_record
        )
        args.output_dir.mkdir(parents=True, exist_ok=True)
        selection_hash = _sha256(args.selection_record)
        use_manifest = {
            "schema_version": "smartphone-r5-wls-multi-phone-ensemble-selection-use.v1",
            "selection_record": {"path": str(args.selection_record), "sha256": selection_hash},
            "archive": {"path": str(args.archive), "sha256": archive_hash},
            "inventory": {"path": str(args.inventory), "sha256": inventory_hash},
            "train_routes": [group["route"] for group in selection["train_routes"]],
            "new_validation": selection["new_validation"]["route"],
            "next_holdout": {
                "route": selection["next_holdout"]["route"],
                "materialized": False,
                "truth_opened": False,
            },
            "holdout_content_access": False,
        }
        _atomic_json(args.output_dir / "selection_use_manifest.json", use_manifest)

        thresholds = dict(selection["selection_contract"]["same_vehicle_audit_thresholds"])
        train_results: list[dict[str, Any]] = []
        train_truths: dict[str, dict[str, dict[int, tuple[float, float, float]]]] = {}
        train_audits: dict[str, Any] = {}
        train_score_inputs: dict[str, dict[str, Any]] = {}
        for group in selection["train_routes"]:
            result = _write_route(
                args.archive,
                group,
                args.output_dir / "train",
                truth_materialized=True,
                alignment_tolerance_ms=args.alignment_tolerance_ms,
            )
            train_results.append(result)
            truths = {
                record["dataset_id"].split("/", 1)[1]: _truth_read(
                    result["truth_paths"][record["dataset_id"].split("/", 1)[1]]
                )
                for record in group["records"]
            }
            train_truths[group["route"]] = truths
            train_audits[group["route"]] = _truth_vehicle_audit(truths, thresholds)
            train_score_inputs[group["route"]] = _score_methods(result, truths)
        train_scores: dict[str, Any] = {}
        for method_id in (BASELINE_METHOD_ID, *METHOD_IDS):
            per_route = [train_score_inputs[group["route"]][method_id] for group in selection["train_routes"]]
            route_aggregates = [value["aggregate"] for value in per_route]
            # Aggregate phone metrics across routes, preserving per-phone
            # evidence in each route result and making the train rank explicit.
            merged_phone_metrics: dict[str, dict[str, Any]] = {}
            for route_score in per_route:
                merged_phone_metrics.update(route_score["phone_metrics"])
            train_scores[method_id] = {
                "route_scores": per_route,
                "aggregate": _aggregate(merged_phone_metrics),
                "route_aggregate_count": len(route_aggregates),
            }
        selected_method = min(
            METHOD_IDS, key=lambda method_id: _rank_train(method_id, train_scores[method_id])
        )
        train_report = {
            "schema_version": "smartphone-r5-wls-multi-phone-ensemble-train.v1",
            "truth_free_wls_generation": True,
            "truth_used_for_method_ranking": True,
            "truth_used_for_same_vehicle_audit": True,
            "selection_record": {"path": str(args.selection_record), "sha256": selection_hash},
            "routes": [result["route"] for result in train_results],
            "truth_vehicle_audit": train_audits,
            "scores": train_scores,
            "selected_method": selected_method,
            "candidate_methods": list(METHOD_IDS),
            "candidate_search_after_selection_record": False,
        }
        train_report_path = args.output_dir / "train_report.json"
        _atomic_json(train_report_path, train_report)

        validation_spec = selection["new_validation"]
        validation = _write_route(
            args.archive,
            validation_spec,
            args.output_dir / "validation",
            truth_materialized=False,
            alignment_tolerance_ms=args.alignment_tolerance_ms,
        )
        validation_truth_free = {
            "schema_version": TRUTH_FREE_SCHEMA_VERSION,
            "truth_free": True,
            "truth_used": False,
            "route": validation["route"],
            "selected_method_from_train": selected_method,
            "candidate_methods_generated": list(METHOD_IDS),
            "alignment_tolerance_ms": args.alignment_tolerance_ms,
            "train_report": _artifact(train_report_path),
            "route_manifest": _artifact(validation["route_manifest_path"]),
            "methods": validation["method_artifacts"],
            "wls": validation["wls_artifacts"],
            "materialized_device_count": len(validation["device_paths"]),
            "materialized_truth_count": 0,
            "next_holdout_materialized": False,
            "next_holdout_truth_opened": False,
        }
        validation_truth_free_path = args.output_dir / "validation_truth_free_manifest.json"
        _atomic_json(validation_truth_free_path, validation_truth_free)
        validation_truth_free_artifact = _artifact(validation_truth_free_path)

        validation_truths: dict[str, dict[int, tuple[float, float, float]]] = {}
        validation_truth_artifacts: dict[str, Any] = {}
        for record in validation_spec["records"]:
            dataset_id = str(record["dataset_id"])
            phone = dataset_id.split("/", 1)[1]
            truth_path = validation["route_root"] / "inputs" / phone / "ground_truth.csv"
            materialized = _materialize_member(
                args.archive,
                _member_names(dataset_id)["ground_truth"],
                truth_path,
                record["ground_truth"],
            )
            validation_truth_artifacts[phone] = materialized
            validation_truths[phone] = _truth_read(truth_path)
        validation_scores = _score_methods(validation, validation_truths)
        baseline_score = validation_scores[BASELINE_METHOD_ID]
        selected_score = validation_scores[selected_method]
        signoff_failures: list[str] = []
        for phone, score in selected_score["phone_metrics"].items():
            if score["availability_ratio"] < 0.98:
                signoff_failures.append(f"{phone}:availability")
            if score["truth_coverage_ratio"] < 0.98:
                signoff_failures.append(f"{phone}:coverage")
        gate = _gate(selected_score, baseline_score, {
            "availability_min": 0.98,
            "truth_coverage_min": 0.98,
            "horizontal_median_max_m": 7.0,
            "horizontal_p95_max_m": 25.0,
            "vertical_p95_max_m": 45.0,
        })
        gate["per_phone_signoff_failures"] = signoff_failures
        if signoff_failures:
            gate["failures"].extend(signoff_failures)
            gate["passed"] = False
        promotion = "promote-development-only" if gate["passed"] else "no-go"
        rss_after = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss
        report = {
            "schema_version": SCHEMA_VERSION,
            "status": "sealed-completed-validation-one-truth-evaluation",
            "selection_record": {"path": str(args.selection_record), "sha256": selection_hash},
            "archive": {"path": str(args.archive), "sha256": archive_hash},
            "inventory": {"path": str(args.inventory), "sha256": inventory_hash},
            "roles": {
                "train_routes": [group["route"] for group in selection["train_routes"]],
                "new_validation": validation["route"],
                "next_holdout": selection["next_holdout"]["route"],
            },
            "excluded_route_ids": selection["excluded_route_ids"],
            "candidate_methods": list(METHOD_IDS),
            "alignment_tolerance_ms": args.alignment_tolerance_ms,
            "selected_method": selected_method,
            "train": {
                "report": _artifact(train_report_path),
                "truth_vehicle_audit": train_audits,
                "scores": train_scores,
                "truth_materialized_routes": [result["route"] for result in train_results],
            },
            "validation_truth_free": {
                "manifest": validation_truth_free_artifact,
                "device_materialized_count": len(validation["device_paths"]),
                "truth_materialized_before_manifest": 0,
                "all_candidate_artifacts_hash_fixed_before_truth": True,
                "next_holdout_materialized": False,
            },
            "validation_truth": {
                "truth_member_open_count": len(validation_truths),
                "evaluation_pass_count": 1,
                "artifacts": validation_truth_artifacts,
                "rows_by_phone": {phone: len(truth) for phone, truth in validation_truths.items()},
            },
            "validation_scores": validation_scores,
            "promotion_gate": {
                "result": gate,
                "decision": promotion,
                "reference": BASELINE_METHOD_ID,
            },
            "holdout_contract": {
                "next_holdout": selection["next_holdout"]["route"],
                "materialized": False,
                "truth_opened": False,
                "used_for_ranking": False,
            },
            "timing": {
                "total_wall_s": time.perf_counter() - started,
                "process_peak_rss_kb": max(int(rss_before), int(rss_after)),
                "process_rss_unit": "kilobytes from getrusage ru_maxrss",
            },
            "production_policy": {
                "production_rtk_spp_default_changed": False,
                "development_only_recommended_lane": gate["passed"],
                "post_validation_tuning": False,
            },
        }
        report_path = args.output_dir / "wls_multi_phone_ensemble_report.json"
        _atomic_json(report_path, report)
        manifest = {
            "schema_version": MANIFEST_SCHEMA_VERSION,
            "report": _artifact(report_path),
            "train_report": _artifact(train_report_path),
            "validation_truth_free_manifest": validation_truth_free_artifact,
            "selected_method": selected_method,
            "promotion_decision": promotion,
            "next_holdout_materialized": False,
            "next_holdout_truth_opened": False,
            "validation_truth_member_open_count": len(validation_truths),
            "truth_free_before_validation_truth": True,
            "sealed": True,
        }
        _atomic_json(args.output_dir / "wls_multi_phone_ensemble_manifest.json", manifest)
        print(f"Smartphone WLS multi-phone ensemble evaluation complete: {report_path}")
        print(f"Selected method: {selected_method}; promotion: {promotion}")
        return 0
    except (
        OSError,
        ValueError,
        KeyError,
        TypeError,
        json.JSONDecodeError,
        zipfile.BadZipFile,
        EnsembleError,
        wls.WlsPositionError,
        smoother.SmootherError,
    ) as exc:
        print(f"Smartphone WLS multi-phone ensemble failed: {exc}", file=sys.stderr)
        if args.output_dir.exists():
            try:
                _atomic_json(
                    args.output_dir / "wls_multi_phone_ensemble_failure.json",
                    {
                        "schema_version": SCHEMA_VERSION,
                        "status": "sealed-failed",
                        "error": str(exc),
                        "next_holdout_materialized": False,
                        "next_holdout_truth_opened": False,
                        "truth_evaluation_pass_count": 0,
                        "post_validation_tuning": False,
                    },
                )
            except OSError:
                pass
        return 1


if __name__ == "__main__":
    raise SystemExit(run())
