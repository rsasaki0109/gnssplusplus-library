#!/usr/bin/env python3
"""Generate a truth-free full-test GSDC submission from the frozen v1.4 lane.

The test split has no usable truth contract.  This command therefore reads
only ZIP central-directory metadata before authorization, materializes the
test ``device_gnss.csv``/``brdc.nav`` members after a content-hashed test
authorization, and never opens a test truth member.  The public sample
submission is the sole authority for final key order by default; a missing or
malformed sample is a hard preflight failure rather than an inferred key list.
An explicit ``--allow-derived-unverified-key-order`` opt-in can produce a
provisional, non-submittable key order from validated ``device_gnss.csv``
epochs.  That opt-in never changes the frozen positioning algorithm and is not
an official-sample verification.

For a route with at least two phones the frozen v1.4 coordinate-wise ECEF
median is applied at each target phone epoch with a 10 ms nearest-source
tolerance, earlier-source tie rule, and no interpolation/extrapolation.  A
single-phone route emits raw WLS.  An exact-key native Galileo E1/Hatch30
segment-stability artifact may fill an all-phone-unresolved key; otherwise
the route fails closed.  The production RTK/SPP default is not changed.
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

import gnss_smartphone_kaggle as kaggle  # noqa: E402
import gnss_smartphone_trajectory_smoother as smoother  # noqa: E402
import gnss_smartphone_wls as wls  # noqa: E402
# The module is named ``..._eval`` in the existing research implementation;
# keep the import local and explicit so the test command cannot accidentally
# select a different fusion implementation.
import gnss_smartphone_wls_multi_phone_ensemble_eval as ensemble  # noqa: E402
import gnss_smartphone_wls_stability_selector as stability_selector  # noqa: E402


SCHEMA_VERSION = "smartphone-r5-wls-test-batch.v1"
INVENTORY_SCHEMA_VERSION = "smartphone-r5-gsdc2023-test-central-inventory.v1"
AUTHORIZATION_SCHEMA = wls.TEST_WLS_AUTHORIZATION_SCHEMA
AUTHORIZATION_MANIFEST_SCHEMA = wls.TEST_WLS_AUTHORIZATION_MANIFEST_SCHEMA
RUN_MANIFEST_SCHEMA = "smartphone-r5-wls-test-batch-run-manifest.v1"
ROUTE_MANIFEST_SCHEMA = "smartphone-r5-wls-test-batch-route-manifest.v1"
SAMPLE_SCHEMA = "smartphone-r5-gsdc2023-sample-submission.v1"
DERIVED_KEY_MANIFEST_SCHEMA = "smartphone-r5-gsdc2023-derived-key-manifest.v1"
DERIVED_SUBMISSION_MANIFEST_SCHEMA = "smartphone-r5-gsdc2023-derived-submission-manifest.v1"
DERIVED_RUN_MANIFEST_SCHEMA = "smartphone-r5-wls-test-batch-derived-run-manifest.v1"
OFFICIAL_SAMPLE_FIELDS = (
    "tripId",
    "UnixTimeMillis",
    "LatitudeDegrees",
    "LongitudeDegrees",
)
OFFICIAL_SAMPLE_SCHEMA = "smartphone-r5-gsdc2023-official-sample-submission.v2"
RECONCILIATION_MANIFEST_SCHEMA = "smartphone-r5-gsdc2023-sample-reconciliation-manifest.v2"
COMPLETENESS_FALLBACK_FREEZE_SCHEMA = (
    "smartphone-r5-wls-test-completeness-fallback-freeze.v1"
)
COMPLETENESS_FALLBACK_MANIFEST_SCHEMA = (
    "smartphone-r5-wls-test-completeness-fallback-freeze-manifest.v1"
)
EDGE_COMPLETENESS_FALLBACK_FREEZE_SCHEMA = (
    "smartphone-r5-wls-test-edge-completeness-fallback-freeze.v2"
)
EDGE_COMPLETENESS_FALLBACK_MANIFEST_SCHEMA = (
    "smartphone-r5-wls-test-edge-completeness-fallback-freeze-manifest.v2"
)
EDGE_COMPLETENESS_FALLBACK_V2_1_FREEZE_SCHEMA = (
    "smartphone-r5-wls-test-edge-completeness-fallback-freeze.v2.1"
)
EDGE_COMPLETENESS_FALLBACK_V2_1_MANIFEST_SCHEMA = (
    "smartphone-r5-wls-test-edge-completeness-fallback-freeze-manifest.v2.1"
)
DEFAULT_ARCHIVE = ROOT / "data" / "gsdc2023" / "cache" / "dataset_2023.zip"
DEFAULT_INVENTORY = ROOT / "output" / "smartphone-r5" / "wls-test-batch-v1" / "test_inventory.json"
DEFAULT_SAMPLE = ROOT / "data" / "gsdc2023" / "sample_submission.csv"
DEFAULT_PROFILE = ROOT / "configs" / "benchmarks" / "smartphone_r5_gsdc2023.json"
DEFAULT_FREEZE = (
    ROOT
    / "docs"
    / "use_cases"
    / "records"
    / "smartphone_r5_wls_multi_phone_ensemble_holdout_freeze_v1_4.json"
)
DEFAULT_FREEZE_MANIFEST = (
    ROOT
    / "docs"
    / "use_cases"
    / "records"
    / "smartphone_r5_wls_multi_phone_ensemble_holdout_freeze_v1_4_manifest.json"
)
DEFAULT_OUTPUT = ROOT / "output" / "smartphone-r5" / "wls-test-batch-v1"
DEFAULT_CACHE = DEFAULT_OUTPUT / "cache"
DEFAULT_ALIGNMENT_TOLERANCE_MS = wls.TEST_WLS_ALIGNMENT_TOLERANCE_MS
FALLBACK_INTERPOLATION_MAX_GAP_MS = 10_000
FALLBACK_NATIVE_TOLERANCE_MS = DEFAULT_ALIGNMENT_TOLERANCE_MS
COMPLETENESS_FALLBACK_CONTRACT = {
    "allow_out_of_range_wls_epochs_as_omission": True,
    "out_of_range_policy": "finite out-of-earth-range epoch omission only",
    "partial_coordinate_triplet_policy": "fail-closed",
    "nonfinite_coordinate_policy": "fail-closed",
    "inconsistent_coordinate_policy": "fail-closed",
    "native_fallback": "nearest native Galileo E1/Hatch30 stable position",
    "native_tolerance_ms": FALLBACK_NATIVE_TOLERANCE_MS,
    "tie_rule": "equal-distance native timestamps choose earlier timestamp",
    "interpolation": "ECEF linear interpolation only between valid selected-source positions",
    "interpolation_max_gap_ms": FALLBACK_INTERPOLATION_MAX_GAP_MS,
    "edge_extrapolation": "forbidden",
    "long_gap_policy": "fail-closed",
}
EDGE_COMPLETENESS_FALLBACK_CONTRACT = {
    "base_contract": dict(COMPLETENESS_FALLBACK_CONTRACT),
    "edge_hold": "constant nearest finite selected-source ECEF; no velocity extrapolation",
    "edge_scope": "leading or trailing unresolved target keys only",
    "edge_max_gap_ms": FALLBACK_INTERPOLATION_MAX_GAP_MS,
    "edge_gap_reference": "nearest valid selected-source timestamp",
    "edge_continuity": (
        "target-to-source epochs must share HardwareClockDiscontinuityCount segment; "
        "clock transitions fail-closed"
    ),
    "edge_segment_identity": "HardwareClockDiscontinuityCount constant across the edge span",
    "edge_source": "finite valid selected-source WLS ECEF",
    "edge_no_valid_source": "fail-closed",
    "edge_long_gap": "fail-closed when nearest valid source is more than 10 s away",
    "velocity_extrapolation": "forbidden",
}
EDGE_COMPLETENESS_FALLBACK_V2_1_MAX_GAP_MS = 1_000
EDGE_COMPLETENESS_FALLBACK_V2_1_CONTRACT = {
    "base_contract": dict(COMPLETENESS_FALLBACK_CONTRACT),
    "edge_hold": "constant nearest finite selected-source ECEF; no velocity extrapolation",
    "edge_scope": "leading or trailing unresolved target keys only",
    "edge_max_gap_ms": EDGE_COMPLETENESS_FALLBACK_V2_1_MAX_GAP_MS,
    "edge_gap_reference": "nearest valid selected-source timestamp",
    "edge_continuity": (
        "target-to-source epochs must share HardwareClockDiscontinuityCount segment; "
        "clock transitions fail-closed"
    ),
    "edge_segment_identity": "HardwareClockDiscontinuityCount constant across the edge span",
    "edge_source": "finite valid selected-source WLS ECEF",
    "edge_no_valid_source": "fail-closed",
    "edge_long_gap": "fail-closed when nearest valid source is more than 1 s away",
    "velocity_extrapolation": "forbidden",
}
SKIP_EPOCHS = 1
LEAP_SECONDS = 18
SAMPLE_FIELDS = tuple(kaggle.SUBMISSION_FIELDS)
REQUIRED_PHONE_MEMBERS = ("device_gnss.csv",)
REQUIRED_ROUTE_MEMBERS = ("brdc.nav",)
# The native-only contract has no MATLAB inputs.  Keep only the raw CSV truth
# member name for metadata classification; any other extension (including
# ``.mat``) is rejected by the native/test authorization boundary before open.
TRUTH_BASENAMES = frozenset({"ground_truth.csv"})


class TestBatchError(ValueError):
    """Raised when the sealed full-test contract cannot be proven."""


def _sha256(path: Path) -> str:
    if not path.is_file():
        raise TestBatchError(f"missing artifact: {path}")
    digest = hashlib.sha256()
    try:
        with path.open("rb") as handle:
            for chunk in iter(lambda: handle.read(1024 * 1024), b""):
                digest.update(chunk)
    except OSError as exc:
        raise TestBatchError(f"failed to hash {path}: {exc}") from exc
    return digest.hexdigest()


def _artifact(path: Path) -> dict[str, Any]:
    return {"path": str(path), "sha256": _sha256(path), "bytes": path.stat().st_size}


def _atomic_write(path: Path, content: bytes) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary: str | None = None
    descriptor: int | None = None
    try:
        descriptor, temporary = tempfile.mkstemp(
            prefix=f".{path.name}.", suffix=".tmp", dir=str(path.parent)
        )
        with os.fdopen(descriptor, "wb") as handle:
            descriptor = None
            handle.write(content)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, path)
        temporary = None
        if hasattr(os, "O_DIRECTORY"):
            directory = os.open(str(path.parent), os.O_DIRECTORY)
            try:
                os.fsync(directory)
            finally:
                os.close(directory)
    except OSError as exc:
        raise TestBatchError(f"atomic publish failed for {path}: {exc}") from exc
    finally:
        if descriptor is not None:
            os.close(descriptor)
        if temporary is not None:
            try:
                os.unlink(temporary)
            except FileNotFoundError:
                pass


def _atomic_json(path: Path, payload: dict[str, Any]) -> None:
    _atomic_write(path, (json.dumps(payload, indent=2, sort_keys=True) + "\n").encode("utf-8"))


def _load_json(path: Path, label: str) -> dict[str, Any]:
    if not path.is_file():
        raise TestBatchError(f"missing {label}: {path}")
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise TestBatchError(f"invalid {label}: {path}") from exc
    if not isinstance(payload, dict):
        raise TestBatchError(f"{label} must be a JSON object")
    return payload


def _canonical_source_key(path: Path) -> str:
    try:
        return str(path.resolve().relative_to(ROOT.resolve()))
    except ValueError:
        return str(path)


def _safe_id(value: str) -> str:
    return value.replace("/", "__")


def _central_metadata(info: zipfile.ZipInfo) -> dict[str, Any]:
    return {
        "name": info.filename,
        "file_size": info.file_size,
        "compressed_size": info.compress_size,
        "crc32_hex": f"{info.CRC:08x}",
    }


def _test_member(route: str, phone: str, basename: str) -> str:
    return f"dataset_2023/test/{route}/{phone}/{basename}"


def _test_nav_member(route: str) -> str:
    return f"dataset_2023/test/{route}/brdc.nav"


def inventory_archive(archive_path: Path) -> dict[str, Any]:
    """Build test inventory using ZIP central-directory metadata only."""

    if not archive_path.is_file():
        raise TestBatchError(f"missing GSDC archive: {archive_path}")
    records: dict[tuple[str, str], dict[str, list[dict[str, Any]]]] = {}
    route_nav: dict[str, list[dict[str, Any]]] = {}
    duplicate_names: dict[str, int] = {}
    test_entry_count = 0
    sample_candidates: list[dict[str, Any]] = []
    try:
        with zipfile.ZipFile(archive_path) as archive:
            infos = archive.infolist()
            for info in infos:
                duplicate_names[info.filename] = duplicate_names.get(info.filename, 0) + 1
                name = info.filename
                if name.endswith("/sample_submission.csv") or name == "sample_submission.csv":
                    if not info.is_dir():
                        sample_candidates.append(_central_metadata(info))
                if not name.startswith("dataset_2023/test/"):
                    continue
                test_entry_count += 1
                if info.is_dir():
                    continue
                parts = name.split("/")
                if len(parts) == 4 and parts[3] == "brdc.nav":
                    route_nav.setdefault(parts[2], []).append(_central_metadata(info))
                    continue
                if len(parts) == 5:
                    route, phone, basename = parts[2], parts[3], parts[4]
                    files = records.setdefault((route, phone), {})
                    files.setdefault(basename, []).append(_central_metadata(info))
    except (OSError, zipfile.BadZipFile) as exc:
        raise TestBatchError(f"failed to inspect test archive central directory: {exc}") from exc

    rows: list[dict[str, Any]] = []
    for route, phone in sorted(records):
        files = records[(route, phone)]
        device_count = len(files.get("device_gnss.csv", []))
        nav_entries = route_nav.get(route, [])
        truth_members = sorted(
            basename
            for basename in files
            if basename in TRUTH_BASENAMES or "truth" in basename.lower()
        )
        rows.append(
            {
                "dataset_id": f"{route}/{phone}",
                "route": route,
                "phone": phone,
                "sample_phone_key": f"{route}_{phone}",
                "calendar_year": int(route[:4]) if route[:4].isdigit() else None,
                "required_files": {"device_gnss.csv": device_count, "brdc.nav": len(nav_entries)},
                "required_files_complete": device_count == 1 and len(nav_entries) == 1,
                "broadcast_nav_present": len(nav_entries) == 1,
                "broadcast_nav_duplicate_count": max(0, len(nav_entries) - 1),
                "truth_present": bool(truth_members),
                "truth_members_not_materialized": truth_members,
                "central_directory_files": {
                    basename: values[0] if len(values) == 1 else values
                    for basename, values in sorted(files.items())
                },
                "central_directory_broadcast_nav": nav_entries[0]
                if len(nav_entries) == 1
                else nav_entries,
            }
        )
    duplicate_list = sorted(name for name, count in duplicate_names.items() if count > 1)
    archive_hash = _sha256(archive_path)
    return {
        "schema_version": INVENTORY_SCHEMA_VERSION,
        "archive": {
            "path": str(archive_path),
            "sha256": archive_hash,
            "size_bytes": archive_path.stat().st_size,
            "central_directory_only": True,
            "member_content_read": False,
            "member_sha256_computed": False,
            "central_entry_count": len(infos),
            "test_entry_count": test_entry_count,
        },
        "test": {
            "route_count": len({row["route"] for row in rows}),
            "route_phone_count": len(rows),
            "records": rows,
            "duplicate_central_names": duplicate_list,
        },
        "sample_submission": {
            "archive_candidates": sample_candidates,
            "archive_candidate_count": len(sample_candidates),
            "payload_opened": False,
            "authoritative_key_order_required": True,
        },
        "truth_policy": {
            "test_truth_payload_opened": False,
            "truth_members_are_metadata_only": True,
            "truth_materialization_forbidden": True,
        },
    }


def _write_or_verify_inventory(path: Path, archive_path: Path) -> dict[str, Any]:
    if path.is_file():
        inventory = _load_json(path, "test central inventory")
        if inventory.get("schema_version") != INVENTORY_SCHEMA_VERSION:
            raise TestBatchError("test inventory schema is invalid")
        if inventory.get("archive", {}).get("sha256") != _sha256(archive_path):
            raise TestBatchError("test inventory archive hash differs")
        if inventory.get("archive", {}).get("central_directory_only") is not True:
            raise TestBatchError("test inventory is not central-directory-only")
        if inventory.get("archive", {}).get("member_content_read") is not False:
            raise TestBatchError("test inventory records member payload access")
        if inventory.get("truth_policy", {}).get("test_truth_payload_opened") is not False:
            raise TestBatchError("test inventory records test truth access")
        return inventory
    inventory = inventory_archive(archive_path)
    _atomic_json(path, inventory)
    return inventory


def _read_sample_submission(path: Path) -> dict[str, Any]:
    """Read only sample key/order and validate its exact public schema."""

    if not path.is_file():
        raise TestBatchError(f"sample submission is missing: {path}")
    rows: list[dict[str, Any]] = []
    seen: set[tuple[str, int]] = set()
    try:
        with path.open(encoding="utf-8-sig", newline="") as handle:
            reader = csv.DictReader(handle)
            if tuple(reader.fieldnames or ()) != SAMPLE_FIELDS:
                raise TestBatchError(
                    "sample submission header must be exactly "
                    + ",".join(SAMPLE_FIELDS)
                )
            for line, raw in enumerate(reader, start=2):
                if None in raw:
                    raise TestBatchError(f"sample row {line} has extra columns")
                phone = raw.get("phone")
                if not isinstance(phone, str) or not phone or phone != phone.strip():
                    raise TestBatchError(f"sample row {line} has an invalid phone key")
                timestamp_token = (raw.get("UnixTimeMillis") or "").strip()
                try:
                    timestamp = int(timestamp_token)
                except ValueError as exc:
                    raise TestBatchError(f"sample row {line} has an invalid timestamp") from exc
                if timestamp < 0:
                    raise TestBatchError(f"sample row {line} has a negative timestamp")
                key = (phone, timestamp)
                if key in seen:
                    raise TestBatchError(f"sample row {line} duplicates key {key!r}")
                seen.add(key)
                # Competition samples conventionally leave coordinates blank.
                # If they are populated, validate them without using their values.
                for field, lower, upper in (
                    ("LatitudeDegrees", -90.0, 90.0),
                    ("LongitudeDegrees", -180.0, 180.0),
                ):
                    token = (raw.get(field) or "").strip()
                    if token:
                        try:
                            value = float(token)
                        except ValueError as exc:
                            raise TestBatchError(f"sample row {line} has invalid {field}") from exc
                        if not math.isfinite(value) or not lower <= value <= upper:
                            raise TestBatchError(f"sample row {line} has invalid {field}")
                rows.append({"phone": phone, "timestamp": timestamp, "source_line": line})
    except OSError as exc:
        raise TestBatchError(f"failed to read sample submission: {path}") from exc
    if not rows:
        raise TestBatchError("sample submission contains no rows")
    return {
        "schema_version": SAMPLE_SCHEMA,
        "fields": list(SAMPLE_FIELDS),
        "rows": rows,
        "key_count": len(rows),
        "artifact": _artifact(path),
    }


def _read_device_epoch_keys(path: Path, *, skip_epochs: int = SKIP_EPOCHS) -> dict[str, Any]:
    """Derive a provisional key sequence from one validated device CSV.

    This is deliberately narrower than the WLS extractor: it reads only the
    raw epoch identity fields and applies the already-frozen ``skip_epochs``
    contract.  Repeated raw rows in one epoch collapse to one key; epoch keys
    must be strictly increasing and finite.  No truth-bearing file is accepted
    by this helper.
    """

    if skip_epochs < 0:
        raise TestBatchError("derived key skip_epochs must be non-negative")
    if not path.is_file():
        raise TestBatchError(f"missing device GNSS CSV for derived keys: {path}")
    epoch_timestamps: list[int] = []
    previous_timestamp: int | None = None
    input_rows = 0
    repeated_rows = 0
    try:
        with path.open(encoding="utf-8-sig", newline="") as handle:
            reader = csv.DictReader(handle)
            fields = list(reader.fieldnames or ())
            if len(fields) != len(set(fields)):
                raise TestBatchError("device GNSS CSV has duplicate fields for derived keys")
            missing = [field for field in ("MessageType", "utcTimeMillis") if field not in fields]
            if missing:
                raise TestBatchError(
                    "device GNSS CSV missing derived-key fields: " + ", ".join(missing)
                )
            for row_number, raw_row in enumerate(reader, start=2):
                input_rows += 1
                if None in raw_row:
                    raise TestBatchError(
                        f"device row {row_number}: extra fields in derived-key source"
                    )
                if raw_row.get("MessageType") != "Raw":
                    raise TestBatchError(
                        f"device row {row_number}: MessageType must be Raw for derived keys"
                    )
                try:
                    timestamp = wls._parse_integer(
                        raw_row.get("utcTimeMillis", ""), "utcTimeMillis", row_number
                    )
                except wls.WlsPositionError as exc:
                    raise TestBatchError(str(exc)) from exc
                if timestamp < 0:
                    raise TestBatchError(
                        f"device row {row_number}: utcTimeMillis must be non-negative"
                    )
                try:
                    timestamp_finite = math.isfinite(float(timestamp))
                except (OverflowError, ValueError):
                    timestamp_finite = False
                if not timestamp_finite:
                    raise TestBatchError(
                        f"device row {row_number}: utcTimeMillis is not finite"
                    )
                if previous_timestamp is not None:
                    if timestamp < previous_timestamp:
                        raise TestBatchError(
                            f"device row {row_number}: utcTimeMillis moved backwards"
                        )
                    if timestamp == previous_timestamp:
                        repeated_rows += 1
                        continue
                epoch_timestamps.append(timestamp)
                previous_timestamp = timestamp
    except OSError as exc:
        raise TestBatchError(f"failed to read derived-key source: {path}") from exc
    if not epoch_timestamps:
        raise TestBatchError(f"device GNSS CSV contains no epochs for derived keys: {path}")
    selected = epoch_timestamps[skip_epochs:]
    if not selected:
        raise TestBatchError(f"skip_epochs removes every derived key: {path}")
    if selected != sorted(set(selected)):
        raise TestBatchError(f"derived device epoch keys are not strictly increasing: {path}")
    return {
        "source": _artifact(path),
        "source_field": "utcTimeMillis",
        "message_type": "Raw",
        "skip_epochs": skip_epochs,
        "input_rows": input_rows,
        "input_epoch_count": len(epoch_timestamps),
        "repeated_raw_timestamp_rows": repeated_rows,
        "selected_epoch_count": len(selected),
        "timestamps_ms": selected,
    }


def _read_epoch_clock_segments(
    path: Path, *, skip_epochs: int = SKIP_EPOCHS
) -> dict[str, Any]:
    """Read truth-free epoch clock/segment metadata for edge continuity.

    A segment is defined by one constant ``HardwareClockDiscontinuityCount``.
    Timestamp gaps remain observable diagnostics; they do not silently permit
    a source outside the frozen edge distance.  Every target epoch in an edge
    hold must have metadata so malformed or ambiguous inputs fail closed.
    """

    if skip_epochs < 0:
        raise TestBatchError("clock segment skip_epochs must be non-negative")
    if not path.is_file():
        raise TestBatchError(f"missing device GNSS CSV for clock segments: {path}")
    epochs: list[tuple[int, int, int]] = []
    previous_timestamp: int | None = None
    current_timestamp: int | None = None
    current_clock: int | None = None
    segment_id = 0
    input_rows = 0
    gap_diagnostics: list[dict[str, int]] = []

    def finish_epoch() -> None:
        nonlocal current_timestamp, current_clock
        if current_timestamp is not None and current_clock is not None:
            epochs.append((current_timestamp, current_clock, segment_id))
        current_timestamp = None
        current_clock = None

    try:
        with path.open(encoding="utf-8-sig", newline="") as handle:
            reader = csv.DictReader(handle)
            fields = list(reader.fieldnames or ())
            required = ("MessageType", "utcTimeMillis", "HardwareClockDiscontinuityCount")
            missing = [field for field in required if field not in fields]
            if len(fields) != len(set(fields)) or missing:
                raise TestBatchError("device GNSS CSV lacks unique clock segment fields")
            for row_number, raw_row in enumerate(reader, start=2):
                input_rows += 1
                if None in raw_row:
                    raise TestBatchError(f"device row {row_number}: extra fields in clock metadata")
                if raw_row.get("MessageType") != "Raw":
                    raise TestBatchError(f"device row {row_number}: MessageType must be Raw")
                try:
                    timestamp = wls._parse_integer(
                        raw_row.get("utcTimeMillis", ""), "utcTimeMillis", row_number
                    )
                    clock = wls._parse_integer(
                        raw_row.get("HardwareClockDiscontinuityCount", ""),
                        "HardwareClockDiscontinuityCount",
                        row_number,
                    )
                except wls.WlsPositionError as exc:
                    raise TestBatchError(str(exc)) from exc
                if timestamp < 0 or clock < 0:
                    raise TestBatchError(f"device row {row_number}: clock metadata is negative")
                if previous_timestamp is not None and timestamp < previous_timestamp:
                    raise TestBatchError(f"device row {row_number}: epoch timestamp moved backwards")
                if current_timestamp is None:
                    current_timestamp = timestamp
                    current_clock = clock
                elif timestamp != current_timestamp:
                    if current_clock is None:
                        raise TestBatchError("clock metadata lost within epoch")
                    if previous_timestamp is not None:
                        gap_ms = timestamp - previous_timestamp
                        if gap_ms <= 0:
                            raise TestBatchError("clock metadata timestamps are not increasing")
                        if gap_ms > wls.TIMESTAMP_GAP_THRESHOLD_MS:
                            gap_diagnostics.append(
                                {"from_timestamp_ms": previous_timestamp, "to_timestamp_ms": timestamp, "gap_ms": gap_ms}
                            )
                    finish_epoch()
                    if epochs and clock != epochs[-1][1]:
                        segment_id += 1
                    current_timestamp = timestamp
                    current_clock = clock
                elif current_clock != clock:
                    raise TestBatchError(f"device row {row_number}: clock changes within epoch")
                previous_timestamp = timestamp
            finish_epoch()
    except OSError as exc:
        raise TestBatchError(f"failed to read clock metadata: {path}") from exc
    if not epochs:
        raise TestBatchError(f"device GNSS CSV contains no clock epochs: {path}")
    selected = epochs[skip_epochs:]
    if not selected:
        raise TestBatchError(f"clock segment skip_epochs removes every epoch: {path}")
    clock_by_timestamp = {timestamp: clock for timestamp, clock, _ in selected}
    segment_by_timestamp = {timestamp: segment for timestamp, _, segment in selected}
    if len(clock_by_timestamp) != len(selected):
        raise TestBatchError("clock segment metadata has duplicate epoch timestamps")
    return {
        "source": _artifact(path),
        "skip_epochs": skip_epochs,
        "timestamps_ms": [timestamp for timestamp, _, _ in selected],
        "clock_by_timestamp": clock_by_timestamp,
        "segment_by_timestamp": segment_by_timestamp,
        "segment_count": len({segment for _, _, segment in selected}),
        "timestamp_gap_diagnostics": gap_diagnostics,
    }


def _verify_upstream_freeze(path: Path, manifest_path: Path) -> dict[str, Any]:
    """Verify the already-frozen v1.4 source/algorithm contract."""

    payload = _load_json(path, "v1.4 freeze record")
    if payload.get("schema_version") != wls.MULTI_PHONE_HOLDOUT_FREEZE_SCHEMA_V1_4:
        raise TestBatchError("upstream freeze is not multi-phone v1.4")
    if payload.get("status") != "frozen-before-holdout-payload-access":
        raise TestBatchError("upstream v1.4 freeze is not pre-payload")
    if payload.get("algorithm_core_hash") != wls.MULTI_PHONE_HOLDOUT_ALGORITHM_CORE_HASH:
        raise TestBatchError("upstream v1.4 algorithm core hash differs")
    contract = payload.get("holdout_execution_contract")
    if not isinstance(contract, dict) or contract.get("truth_free_phase") is not True:
        raise TestBatchError("upstream v1.4 freeze is not truth-free")
    if contract.get("no_post_holdout_tuning") is not True:
        raise TestBatchError("upstream v1.4 freeze lacks no-tuning guard")
    if payload.get("alignment_tolerance_ms") not in (None, DEFAULT_ALIGNMENT_TOLERANCE_MS):
        raise TestBatchError("upstream v1.4 alignment tolerance differs")
    manifest = _load_json(manifest_path, "v1.4 freeze manifest")
    if manifest.get("schema_version") != wls.MULTI_PHONE_HOLDOUT_FREEZE_MANIFEST_SCHEMA_V1_4:
        raise TestBatchError("upstream v1.4 freeze manifest schema is invalid")
    record = manifest.get("freeze_record")
    if not isinstance(record, dict) or record.get("sha256") != _sha256(path):
        raise TestBatchError("upstream v1.4 freeze record hash differs")
    if manifest.get("algorithm_core_hash") != wls.MULTI_PHONE_HOLDOUT_ALGORITHM_CORE_HASH:
        raise TestBatchError("upstream v1.4 manifest algorithm core differs")
    return {
        "record": _artifact(path),
        "manifest": _artifact(manifest_path),
        "algorithm_core_hash": wls.MULTI_PHONE_HOLDOUT_ALGORITHM_CORE_HASH,
    }


def _verify_completeness_fallback_freeze(
    path: Path,
    manifest_path: Path,
    *,
    archive_hash: str,
    inventory_hash: str,
    profile_path: Path,
) -> dict[str, Any]:
    """Verify the separately frozen, truth-free test completeness extension."""

    payload = _load_json(path, "test completeness fallback freeze")
    if payload.get("schema_version") != COMPLETENESS_FALLBACK_FREEZE_SCHEMA:
        raise TestBatchError("test completeness fallback freeze schema is invalid")
    if payload.get("status") != "frozen-before-test-payload-access":
        raise TestBatchError("test completeness fallback freeze is not pre-payload")
    contract = payload.get("test_completeness_fallback_contract")
    if not isinstance(contract, dict) or contract.get("authorized") is not True:
        raise TestBatchError("test completeness fallback is not authorized")
    if contract.get("truth_free_phase") is not True:
        raise TestBatchError("test completeness fallback is not truth-free")
    if contract.get("no_post_test_tuning") is not True:
        raise TestBatchError("test completeness fallback lacks no-tuning guard")
    frozen_contract = contract.get("fallback_policy")
    if frozen_contract != COMPLETENESS_FALLBACK_CONTRACT:
        raise TestBatchError("test completeness fallback policy differs from frozen candidate")
    if payload.get("algorithm_core_hash") != wls.MULTI_PHONE_HOLDOUT_ALGORITHM_CORE_HASH:
        raise TestBatchError("test completeness fallback algorithm core differs")
    if payload.get("algorithm_parameter_hash") != wls.MULTI_PHONE_HOLDOUT_ALGORITHM_PARAMETER_HASH:
        raise TestBatchError("test completeness fallback parameters differ")
    archive = payload.get("archive")
    inventory = payload.get("inventory")
    if not isinstance(archive, dict) or archive.get("sha256") != archive_hash:
        raise TestBatchError("test completeness fallback archive hash differs")
    if not isinstance(inventory, dict) or inventory.get("sha256") != inventory_hash:
        raise TestBatchError("test completeness fallback inventory hash differs")
    profile = payload.get("profile")
    if not isinstance(profile, dict) or profile.get("sha256") != _sha256(profile_path):
        raise TestBatchError("test completeness fallback profile hash differs")
    source_hashes = payload.get("source_hashes")
    if not isinstance(source_hashes, dict):
        raise TestBatchError("test completeness fallback lacks source hashes")
    expected_wls = source_hashes.get("apps/commands/benchmarks/gnss_smartphone_wls.py")
    if isinstance(expected_wls, dict):
        expected_wls = expected_wls.get("sha256")
    if expected_wls != _sha256(Path(wls.__file__)):
        raise TestBatchError("test completeness fallback WLS source hash differs")
    expected_batch = source_hashes.get("apps/commands/benchmarks/gnss_smartphone_wls_test_batch.py")
    if isinstance(expected_batch, dict):
        expected_batch = expected_batch.get("sha256")
    if expected_batch != _sha256(Path(__file__)):
        raise TestBatchError("test completeness fallback batch source hash differs")
    manifest = _load_json(manifest_path, "test completeness fallback manifest")
    if manifest.get("schema_version") != COMPLETENESS_FALLBACK_MANIFEST_SCHEMA:
        raise TestBatchError("test completeness fallback manifest schema is invalid")
    freeze_record = manifest.get("freeze_record")
    if not isinstance(freeze_record, dict) or freeze_record.get("sha256") != _sha256(path):
        raise TestBatchError("test completeness fallback freeze record hash differs")
    if manifest.get("algorithm_core_hash") != wls.MULTI_PHONE_HOLDOUT_ALGORITHM_CORE_HASH:
        raise TestBatchError("test completeness fallback manifest core differs")
    if manifest.get("algorithm_parameter_hash") != wls.MULTI_PHONE_HOLDOUT_ALGORITHM_PARAMETER_HASH:
        raise TestBatchError("test completeness fallback manifest parameters differ")
    if manifest.get("archive_sha256") != archive_hash or manifest.get("inventory_sha256") != inventory_hash:
        raise TestBatchError("test completeness fallback manifest input hash differs")
    if manifest.get("fallback_policy") != COMPLETENESS_FALLBACK_CONTRACT:
        raise TestBatchError("test completeness fallback manifest policy differs")
    return {
        "record": _artifact(path),
        "manifest": _artifact(manifest_path),
        "algorithm_core_hash": wls.MULTI_PHONE_HOLDOUT_ALGORITHM_CORE_HASH,
        "algorithm_parameter_hash": wls.MULTI_PHONE_HOLDOUT_ALGORITHM_PARAMETER_HASH,
        "fallback_policy": dict(COMPLETENESS_FALLBACK_CONTRACT),
    }


def _verify_edge_completeness_fallback_freeze(
    path: Path,
    manifest_path: Path,
    *,
    archive_hash: str,
    inventory_hash: str,
    profile_path: Path,
    expected_schema: str = EDGE_COMPLETENESS_FALLBACK_FREEZE_SCHEMA,
    expected_manifest_schema: str = EDGE_COMPLETENESS_FALLBACK_MANIFEST_SCHEMA,
    expected_contract: dict[str, Any] = EDGE_COMPLETENESS_FALLBACK_CONTRACT,
) -> dict[str, Any]:
    """Verify the separately frozen v2 edge constant-hold test extension."""

    payload = _load_json(path, "test edge completeness fallback freeze")
    if payload.get("schema_version") != expected_schema:
        raise TestBatchError("test edge completeness fallback freeze schema is invalid")
    if payload.get("status") != "frozen-before-test-payload-access":
        raise TestBatchError("test edge completeness fallback freeze is not pre-payload")
    contract = payload.get("test_edge_completeness_fallback_contract")
    if not isinstance(contract, dict) or contract.get("authorized") is not True:
        raise TestBatchError("test edge completeness fallback is not authorized")
    if contract.get("truth_free_phase") is not True:
        raise TestBatchError("test edge completeness fallback is not truth-free")
    if contract.get("no_post_test_tuning") is not True:
        raise TestBatchError("test edge completeness fallback lacks no-tuning guard")
    if contract.get("fallback_policy") != expected_contract:
        raise TestBatchError("test edge completeness fallback policy differs from frozen candidate")
    if payload.get("algorithm_core_hash") != wls.MULTI_PHONE_HOLDOUT_ALGORITHM_CORE_HASH:
        raise TestBatchError("test edge completeness fallback algorithm core differs")
    if payload.get("algorithm_parameter_hash") != wls.MULTI_PHONE_HOLDOUT_ALGORITHM_PARAMETER_HASH:
        raise TestBatchError("test edge completeness fallback parameters differ")
    archive = payload.get("archive")
    inventory = payload.get("inventory")
    if not isinstance(archive, dict) or archive.get("sha256") != archive_hash:
        raise TestBatchError("test edge completeness fallback archive hash differs")
    if not isinstance(inventory, dict) or inventory.get("sha256") != inventory_hash:
        raise TestBatchError("test edge completeness fallback inventory hash differs")
    profile = payload.get("profile")
    if not isinstance(profile, dict) or profile.get("sha256") != _sha256(profile_path):
        raise TestBatchError("test edge completeness fallback profile hash differs")
    source_hashes = payload.get("source_hashes")
    if not isinstance(source_hashes, dict):
        raise TestBatchError("test edge completeness fallback lacks source hashes")
    expected_wls = source_hashes.get("apps/commands/benchmarks/gnss_smartphone_wls.py")
    if isinstance(expected_wls, dict):
        expected_wls = expected_wls.get("sha256")
    if expected_wls != _sha256(Path(wls.__file__)):
        raise TestBatchError("test edge completeness fallback WLS source hash differs")
    expected_batch = source_hashes.get("apps/commands/benchmarks/gnss_smartphone_wls_test_batch.py")
    if isinstance(expected_batch, dict):
        expected_batch = expected_batch.get("sha256")
    if expected_batch != _sha256(Path(__file__)):
        raise TestBatchError("test edge completeness fallback batch source hash differs")
    parent = payload.get("parent_v1_freeze")
    if not isinstance(parent, dict):
        raise TestBatchError("test edge completeness fallback lacks v1 parent freeze")
    parent_record = _resolve_repo_path(str(parent.get("record", "")))
    parent_manifest = _resolve_repo_path(str(parent.get("manifest", "")))
    if parent.get("record_sha256") != _sha256(parent_record):
        raise TestBatchError("test edge completeness fallback parent record hash differs")
    if parent.get("manifest_sha256") != _sha256(parent_manifest):
        raise TestBatchError("test edge completeness fallback parent manifest hash differs")
    evaluation = payload.get("truth_free_evaluation")
    if not isinstance(evaluation, dict):
        raise TestBatchError("test edge completeness fallback lacks truth-free evaluation")
    evaluation_path = _resolve_repo_path(str(evaluation.get("path", "")))
    if evaluation.get("sha256") != _sha256(evaluation_path):
        raise TestBatchError("test edge completeness fallback evaluation hash differs")
    manifest = _load_json(manifest_path, "test edge completeness fallback manifest")
    if manifest.get("schema_version") != expected_manifest_schema:
        raise TestBatchError("test edge completeness fallback manifest schema is invalid")
    freeze_record = manifest.get("freeze_record")
    if not isinstance(freeze_record, dict) or freeze_record.get("sha256") != _sha256(path):
        raise TestBatchError("test edge completeness fallback freeze record hash differs")
    if manifest.get("algorithm_core_hash") != wls.MULTI_PHONE_HOLDOUT_ALGORITHM_CORE_HASH:
        raise TestBatchError("test edge completeness fallback manifest core differs")
    if manifest.get("algorithm_parameter_hash") != wls.MULTI_PHONE_HOLDOUT_ALGORITHM_PARAMETER_HASH:
        raise TestBatchError("test edge completeness fallback manifest parameters differ")
    if manifest.get("archive_sha256") != archive_hash or manifest.get("inventory_sha256") != inventory_hash:
        raise TestBatchError("test edge completeness fallback manifest input hash differs")
    if manifest.get("fallback_policy") != expected_contract:
        raise TestBatchError("test edge completeness fallback manifest policy differs")
    return {
        "record": _artifact(path),
        "manifest": _artifact(manifest_path),
        "algorithm_core_hash": wls.MULTI_PHONE_HOLDOUT_ALGORITHM_CORE_HASH,
        "algorithm_parameter_hash": wls.MULTI_PHONE_HOLDOUT_ALGORITHM_PARAMETER_HASH,
        "fallback_policy": dict(expected_contract),
        "parent_v1_freeze": dict(parent),
        "truth_free_evaluation": dict(evaluation),
    }


def _verify_edge_completeness_fallback_v2_1_freeze(
    path: Path,
    manifest_path: Path,
    *,
    archive_hash: str,
    inventory_hash: str,
    profile_path: Path,
) -> dict[str, Any]:
    """Verify the v2.1 one-second edge-hold freeze."""

    return _verify_edge_completeness_fallback_freeze(
        path,
        manifest_path,
        archive_hash=archive_hash,
        inventory_hash=inventory_hash,
        profile_path=profile_path,
        expected_schema=EDGE_COMPLETENESS_FALLBACK_V2_1_FREEZE_SCHEMA,
        expected_manifest_schema=EDGE_COMPLETENESS_FALLBACK_V2_1_MANIFEST_SCHEMA,
        expected_contract=EDGE_COMPLETENESS_FALLBACK_V2_1_CONTRACT,
    )


def _profile_artifact(path: Path) -> dict[str, Any]:
    payload = _load_json(path, "smartphone profile")
    if payload.get("schema_version") != "smartphone-r5-profile.v1":
        raise TestBatchError("smartphone profile schema is invalid")
    lane = payload.get("development_only_submission_lane", {}).get("multi_phone_ensemble_v1_4")
    if not isinstance(lane, dict) or lane.get("production_default") is not False:
        raise TestBatchError("profile lacks the frozen development-only v1.4 lane")
    if lane.get("alignment_tolerance_ms") != DEFAULT_ALIGNMENT_TOLERANCE_MS:
        raise TestBatchError("profile v1.4 alignment tolerance differs")
    return _artifact(path)


def _binary_artifacts() -> dict[str, Any]:
    result: dict[str, Any] = {}
    for name in ("gnss_spp", "gnss_solve"):
        path = ROOT / "build" / "apps" / name
        result[name] = _artifact(path) if path.is_file() else {"path": str(path), "missing": True}
    return result


def _source_artifacts(profile: Path, freeze: Path, freeze_manifest: Path) -> dict[str, Any]:
    paths = {
        "apps/commands/benchmarks/gnss_smartphone_wls.py": Path(wls.__file__),
        "apps/commands/benchmarks/gnss_smartphone_wls_multi_phone_ensemble_eval.py": Path(ensemble.__file__),
        "apps/commands/benchmarks/gnss_smartphone_wls_test_batch.py": Path(__file__),
        "configs/benchmarks/smartphone_r5_gsdc2023.json": profile,
        "freeze_record": freeze,
        "freeze_manifest": freeze_manifest,
    }
    return {key: _artifact(path) for key, path in paths.items()}


def _create_test_authorization(
    output_dir: Path,
    archive_path: Path,
    inventory_path: Path,
    inventory: dict[str, Any],
    profile_path: Path,
    freeze_path: Path,
    freeze_manifest_path: Path,
    *,
    sample_member: dict[str, Any] | None,
    derived_key_order: bool = False,
    completeness_fallback_freeze_path: Path | None = None,
    completeness_fallback_manifest_path: Path | None = None,
    edge_completeness_fallback_freeze_path: Path | None = None,
    edge_completeness_fallback_manifest_path: Path | None = None,
    edge_completeness_fallback_v2_1_freeze_path: Path | None = None,
    edge_completeness_fallback_v2_1_manifest_path: Path | None = None,
) -> tuple[Path, Path, dict[str, Any]]:
    """Seal the test allowlist and source hashes before any test payload read."""

    records = inventory.get("test", {}).get("records")
    if not isinstance(records, list) or not records:
        raise TestBatchError("test inventory has no route/device records")
    dataset_ids: list[str] = []
    for record in records:
        if not isinstance(record, dict):
            raise TestBatchError("test inventory record is invalid")
        dataset_id = record.get("dataset_id")
        if not isinstance(dataset_id, str) or dataset_id.count("/") != 1:
            raise TestBatchError("test inventory dataset ID is invalid")
        if not record.get("required_files_complete"):
            raise TestBatchError(f"test route lacks device GNSS or nav: {dataset_id}")
        dataset_ids.append(dataset_id)
    if len(dataset_ids) != len(set(dataset_ids)):
        raise TestBatchError("test inventory has duplicate dataset IDs")
    upstream = _verify_upstream_freeze(freeze_path, freeze_manifest_path)
    profile_artifact = _profile_artifact(profile_path)
    source_hashes = _source_artifacts(profile_path, freeze_path, freeze_manifest_path)
    binary_hashes = _binary_artifacts()
    archive_hash = _sha256(archive_path)
    inventory_hash = _sha256(inventory_path)
    completeness_fallback: dict[str, Any] | None = None
    if (completeness_fallback_freeze_path is None) != (
        completeness_fallback_manifest_path is None
    ):
        raise TestBatchError(
            "test completeness fallback requires both freeze record and manifest"
        )
    if completeness_fallback_freeze_path is not None:
        completeness_fallback = _verify_completeness_fallback_freeze(
            completeness_fallback_freeze_path,
            completeness_fallback_manifest_path,
            archive_hash=archive_hash,
            inventory_hash=inventory_hash,
            profile_path=profile_path,
        )
    edge_completeness_fallback: dict[str, Any] | None = None
    if (edge_completeness_fallback_freeze_path is None) != (
        edge_completeness_fallback_manifest_path is None
    ):
        raise TestBatchError(
            "edge completeness fallback requires both freeze record and manifest"
        )
    if edge_completeness_fallback_freeze_path is not None:
        edge_completeness_fallback = _verify_edge_completeness_fallback_freeze(
            edge_completeness_fallback_freeze_path,
            edge_completeness_fallback_manifest_path,
            archive_hash=archive_hash,
            inventory_hash=inventory_hash,
            profile_path=profile_path,
        )
    edge_completeness_fallback_v2_1: dict[str, Any] | None = None
    if (edge_completeness_fallback_v2_1_freeze_path is None) != (
        edge_completeness_fallback_v2_1_manifest_path is None
    ):
        raise TestBatchError(
            "edge completeness fallback v2.1 requires both freeze record and manifest"
        )
    if edge_completeness_fallback_v2_1_freeze_path is not None:
        edge_completeness_fallback_v2_1 = _verify_edge_completeness_fallback_v2_1_freeze(
            edge_completeness_fallback_v2_1_freeze_path,
            edge_completeness_fallback_v2_1_manifest_path,
            archive_hash=archive_hash,
            inventory_hash=inventory_hash,
            profile_path=profile_path,
        )
    if edge_completeness_fallback and edge_completeness_fallback_v2_1:
        raise TestBatchError("edge completeness fallback v2 and v2.1 are mutually exclusive")
    effective_completeness_fallback = (
        edge_completeness_fallback_v2_1
        or edge_completeness_fallback
        or completeness_fallback
    )
    sample_contract = {
        "authoritative_key_order_required": not derived_key_order,
        "derived_key_order_opt_in": derived_key_order,
        "derived_key_order_verified": False,
        "archive_central_candidate": sample_member,
        "truth_free": True,
        "coordinates_ignored": True,
    }
    record = {
        "schema_version": AUTHORIZATION_SCHEMA,
        "status": "sealed-before-test-payload-access",
        "created_for": "full-test-submission-integration",
        "archive": {"path": str(archive_path), "sha256": archive_hash},
        "inventory": {"path": str(inventory_path), "sha256": inventory_hash},
        "upstream_v1_4_freeze": upstream,
        "profile": profile_artifact,
        "algorithm_parameter_hash": wls.MULTI_PHONE_HOLDOUT_ALGORITHM_PARAMETER_HASH,
        "algorithm_core_hash": wls.MULTI_PHONE_HOLDOUT_ALGORITHM_CORE_HASH,
        "test_execution_contract": {
            "authorized": True,
            "role": "test",
            "truth_free_phase": True,
            "truth_access_forbidden": True,
            "no_post_test_tuning": True,
            "dataset_allowlist": dataset_ids,
            "skip_epochs": SKIP_EPOCHS,
            "alignment_tolerance_ms": DEFAULT_ALIGNMENT_TOLERANCE_MS,
            "tie_rule": "equal-distance source timestamps choose earlier timestamp",
            "interpolation": "forbidden",
            "extrapolation": "forbidden",
            "single_phone_policy": "finite raw WLS; exact native key fallback only when raw WLS is unresolved",
            "multi_phone_policy": "coordinate-wise ECEF median over all finite nearest WLS sources within tolerance",
        },
        "sparse_wls_contract": {
            "allow_missing_wls_epochs": True,
            "single_phone_default_fail_closed": True,
            "allow_timestamp_gaps_as_diagnostic": True,
            "extrapolation_policy": "forbidden",
            "partial_coordinate_triplet_policy": "fail-closed",
            "nonfinite_coordinate_policy": "fail-closed",
            "inconsistent_coordinate_policy": "fail-closed",
            "allow_out_of_range_wls_epochs_as_omission": effective_completeness_fallback is not None,
        },
        "completeness_fallback": completeness_fallback,
        "edge_completeness_fallback": edge_completeness_fallback,
        "edge_completeness_fallback_v2_1": edge_completeness_fallback_v2_1,
        "sample_submission": sample_contract,
        "source_hashes": source_hashes,
        "release_binaries": binary_hashes,
        "truth_policy": {
            "truth_member_names_may_exist": True,
            "truth_materialized": False,
            "truth_open_count": 0,
            "test_truth_payload_forbidden": True,
        },
        "production_policy": {"production_rtk_spp_default_changed": False},
    }
    record_path = output_dir / "test_authorization.json"
    manifest_path = output_dir / "test_authorization_manifest.json"
    _atomic_json(record_path, record)
    authorization_manifest = {
        "schema_version": AUTHORIZATION_MANIFEST_SCHEMA,
        "authorization_record": {"path": str(record_path), "sha256": _sha256(record_path)},
        "algorithm_parameter_hash": wls.MULTI_PHONE_HOLDOUT_ALGORITHM_PARAMETER_HASH,
        "algorithm_core_hash": wls.MULTI_PHONE_HOLDOUT_ALGORITHM_CORE_HASH,
        "dataset_allowlist": dataset_ids,
        "wls_source_sha256": _sha256(Path(wls.__file__)),
        "archive_sha256": archive_hash,
        "inventory_sha256": inventory_hash,
        "completeness_fallback": completeness_fallback,
        "edge_completeness_fallback": edge_completeness_fallback,
        "edge_completeness_fallback_v2_1": edge_completeness_fallback_v2_1,
        "allow_out_of_range_wls_epochs_as_omission": effective_completeness_fallback is not None,
        "truth_open_count": 0,
        "sealed_before_test_payload_access": True,
    }
    _atomic_json(manifest_path, authorization_manifest)
    return record_path, manifest_path, record


def _materialize_member(
    archive_path: Path,
    member: str,
    output: Path,
    expected_metadata: dict[str, Any],
) -> dict[str, Any]:
    """Materialize one authorized non-truth member and verify central metadata."""

    output.parent.mkdir(parents=True, exist_ok=True)
    temporary_path: Path | None = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="wb", prefix=f".{output.name}.", suffix=".tmp", dir=output.parent, delete=False
        ) as temporary:
            temporary_path = Path(temporary.name)
            with zipfile.ZipFile(archive_path) as archive:
                info = archive.getinfo(member)
                actual = _central_metadata(info)
                if actual != expected_metadata:
                    raise TestBatchError(f"central metadata changed for {member}")
                with archive.open(info, "r") as source:
                    shutil.copyfileobj(source, temporary)
            temporary.flush()
            os.fsync(temporary.fileno())
        if temporary_path.stat().st_size != int(expected_metadata["file_size"]):
            raise TestBatchError(f"materialized size differs for {member}")
        os.replace(temporary_path, output)
        temporary_path = None
    except (OSError, KeyError, zipfile.BadZipFile) as exc:
        raise TestBatchError(f"failed to materialize {member}: {exc}") from exc
    finally:
        if temporary_path is not None:
            temporary_path.unlink(missing_ok=True)
    return {"member": member, "central_metadata": expected_metadata, **_artifact(output)}


def _verify_cached_wls(
    cache_dir: Path,
    device_path: Path,
    authorization_path: Path,
    authorization_manifest_path: Path,
    dataset_id: str,
) -> tuple[Path, dict[str, Any]] | None:
    required = [cache_dir / name for name in ("wls.pos", "receiver_wls.csv", "wls_summary.json", "wls_manifest.json")]
    if not all(path.is_file() for path in required):
        if cache_dir.exists() and any(cache_dir.iterdir()):
            raise TestBatchError(f"incomplete content-addressed WLS cache: {cache_dir}")
        return None
    manifest = _load_json(cache_dir / "wls_manifest.json", "cached WLS manifest")
    if manifest.get("role") != "test" or manifest.get("truth_free") is not True or manifest.get("truth_used") is not False:
        raise TestBatchError(f"cached WLS manifest is not sealed truth-free test output: {dataset_id}")
    inputs = manifest.get("inputs", {})
    device_artifact = inputs.get("device_gnss") if isinstance(inputs, dict) else None
    if not isinstance(device_artifact, dict) or device_artifact.get("sha256") != _sha256(device_path):
        raise TestBatchError(f"cached WLS device input hash differs: {dataset_id}")
    contract = manifest.get("contract", {})
    if contract.get("dataset_id") != dataset_id:
        raise TestBatchError(f"cached WLS dataset ID differs: {dataset_id}")
    auth = contract.get("sealed_authorization")
    if not isinstance(auth, dict) or auth.get("record_sha256") != _sha256(authorization_path) or auth.get("manifest_sha256") != _sha256(authorization_manifest_path):
        raise TestBatchError(f"cached WLS authorization hash differs: {dataset_id}")
    artifacts = manifest.get("artifacts", {})
    for key in ("position", "geodetic", "summary"):
        entry = artifacts.get(key)
        if not isinstance(entry, dict):
            raise TestBatchError(f"cached WLS lacks {key} artifact: {dataset_id}")
        path = Path(str(entry.get("path", "")))
        if path != cache_dir / ("receiver_wls.csv" if key == "geodetic" else "wls_summary.json" if key == "summary" else "wls.pos"):
            raise TestBatchError(f"cached WLS {key} path differs: {dataset_id}")
        if entry.get("sha256") != _sha256(path):
            raise TestBatchError(f"cached WLS {key} hash differs: {dataset_id}")
    return cache_dir / "wls.pos", manifest


def _resolve_repo_path(raw_path: str, base: Path | None = None) -> Path:
    path = Path(raw_path)
    if path.is_absolute():
        return path
    if base is not None and (base / path).is_file():
        return (base / path).resolve()
    return (ROOT / path).resolve()


def _load_resumed_success_routes(
    resume_path: Path,
    *,
    archive_hash: str,
    inventory_hash: str,
    records: list[dict[str, Any]],
    expected_completed_count: int = 37,
    expected_failed_routes: tuple[str, ...] = (),
) -> tuple[dict[str, dict[str, Any]], dict[str, Any]]:
    """Reuse only sealed, successful artifacts from a prior truth-free run."""

    manifest_path = resume_path
    if manifest_path.is_dir():
        manifest_path = manifest_path / "test_batch_derived_run_manifest.json"
    run = _load_json(manifest_path, "resumed derived test batch manifest")
    if run.get("schema_version") != DERIVED_RUN_MANIFEST_SCHEMA:
        raise TestBatchError("resumed derived run schema is invalid")
    if run.get("status") != "failed-truth-free-derived-submission":
        raise TestBatchError("resumed derived run must be the sealed 37-success failure run")
    if run.get("official_sample_verified") is not False:
        raise TestBatchError("resumed derived run has invalid sample state")
    if run.get("truth_policy", {}).get("truth_open_count") != 0:
        raise TestBatchError("resumed derived run records truth access")
    if run.get("archive", {}).get("sha256") != archive_hash:
        raise TestBatchError("resumed derived run archive hash differs")
    if run.get("inventory", {}).get("sha256") != inventory_hash:
        raise TestBatchError("resumed derived run inventory hash differs")
    route_contract = run.get("routes")
    if not isinstance(route_contract, dict) or route_contract.get("completed_count") != expected_completed_count:
        raise TestBatchError(
            f"resumed derived run does not contain exactly {expected_completed_count} successes"
        )
    route_entries = route_contract.get("route_manifests")
    if not isinstance(route_entries, list) or len(route_entries) != expected_completed_count:
        raise TestBatchError("resumed derived run route manifest count is invalid")
    if expected_failed_routes:
        failures = route_contract.get("failures")
        actual_failed_routes = tuple(
            sorted(
                str(item.get("route"))
                for item in (failures if isinstance(failures, list) else ())
                if isinstance(item, dict)
            )
        )
        if actual_failed_routes != tuple(sorted(expected_failed_routes)):
            raise TestBatchError("resumed derived run failure scope differs")
    expected_by_route: dict[str, list[dict[str, Any]]] = {}
    for record in records:
        expected_by_route.setdefault(str(record["route"]), []).append(record)
    results: dict[str, dict[str, Any]] = {}
    for raw_entry in route_entries:
        if not isinstance(raw_entry, dict):
            raise TestBatchError("resumed route artifact entry is invalid")
        route_manifest_path = _resolve_repo_path(str(raw_entry.get("path", "")), manifest_path.parent)
        if raw_entry.get("sha256") != _sha256(route_manifest_path):
            raise TestBatchError(f"resumed route manifest hash differs: {route_manifest_path}")
        payload = _load_json(route_manifest_path, "resumed route manifest")
        route = payload.get("route")
        if not isinstance(route, str) or route not in expected_by_route:
            raise TestBatchError("resumed route is outside the current inventory")
        expected_ids = sorted(str(item["dataset_id"]) for item in expected_by_route[route])
        dataset_ids = payload.get("dataset_ids")
        if sorted(dataset_ids or ()) != expected_ids:
            raise TestBatchError(f"resumed route dataset set differs: {route}")
        if payload.get("truth_free") is not True or payload.get("truth_used") is not False:
            raise TestBatchError(f"resumed route is not truth-free: {route}")
        provenance = payload.get("provenance", {})
        if provenance.get("archive_sha256") != archive_hash or provenance.get("inventory_sha256") != inventory_hash:
            raise TestBatchError(f"resumed route provenance differs: {route}")
        materialized = payload.get("materialized")
        outputs = payload.get("outputs")
        phones = payload.get("phones")
        if not isinstance(materialized, dict) or not isinstance(outputs, dict) or not isinstance(phones, list):
            raise TestBatchError(f"resumed route payload is incomplete: {route}")
        states: dict[str, dict[str, Any]] = {}
        selected_positions: dict[str, list[smoother.PositionRow]] = {}
        row_map: dict[tuple[str, int], tuple[float, float]] = {}
        for phone in sorted(str(value) for value in phones):
            dataset_id = f"{route}/{phone}"
            material = materialized.get(phone)
            output = outputs.get(phone)
            if not isinstance(material, dict) or not isinstance(output, dict):
                raise TestBatchError(f"resumed phone payload is incomplete: {dataset_id}")
            device_entry = material.get("device_gnss")
            nav_entry = material.get("broadcast_nav")
            if not isinstance(device_entry, dict) or not isinstance(nav_entry, dict):
                raise TestBatchError(f"resumed materialization is incomplete: {dataset_id}")
            device_path = _resolve_repo_path(str(device_entry.get("path", "")), manifest_path.parent)
            nav_path = _resolve_repo_path(str(nav_entry.get("path", "")), manifest_path.parent)
            if device_entry.get("sha256") != _sha256(device_path) or nav_entry.get("sha256") != _sha256(nav_path):
                raise TestBatchError(f"resumed input hash differs: {dataset_id}")
            wls_info = output.get("wls")
            selected_info = output.get("selected_position")
            if not isinstance(wls_info, dict) or not isinstance(selected_info, dict):
                raise TestBatchError(f"resumed WLS/output entry is incomplete: {dataset_id}")
            wls_position_path = _resolve_repo_path(str(wls_info.get("position", {}).get("path", "")), manifest_path.parent)
            wls_summary_path = _resolve_repo_path(str(wls_info.get("summary", {}).get("path", "")), manifest_path.parent)
            wls_manifest_path = _resolve_repo_path(str(wls_info.get("manifest", {}).get("path", "")), manifest_path.parent)
            for entry, path in (
                (wls_info.get("position"), wls_position_path),
                (wls_info.get("summary"), wls_summary_path),
                (wls_info.get("manifest"), wls_manifest_path),
            ):
                if not isinstance(entry, dict) or entry.get("sha256") != _sha256(path):
                    raise TestBatchError(f"resumed WLS artifact hash differs: {dataset_id}")
            wls_manifest = _load_json(wls_manifest_path, "resumed WLS manifest")
            if wls_manifest.get("role") != "test" or wls_manifest.get("truth_free") is not True or wls_manifest.get("truth_used") is not False:
                raise TestBatchError(f"resumed WLS output is not truth-free test data: {dataset_id}")
            summary = _load_json(wls_summary_path, "resumed WLS summary")
            selected_keys = summary.get("selection", {}).get("selected_device_epoch_timestamps_ms")
            if not isinstance(selected_keys, list) or selected_keys != sorted(set(selected_keys)):
                raise TestBatchError(f"resumed WLS selected keys are invalid: {dataset_id}")
            positions = smoother._read_positions(wls_position_path, LEAP_SECONDS)
            position_by_timestamp = {row.timestamp_ms: row for row in positions}
            if len(position_by_timestamp) != len(positions):
                raise TestBatchError(f"resumed WLS keys are duplicated: {dataset_id}")
            selected_output_path = _resolve_repo_path(str(selected_info.get("path", "")), manifest_path.parent)
            if selected_info.get("sha256") != _sha256(selected_output_path):
                raise TestBatchError(f"resumed selected output hash differs: {dataset_id}")
            selected_rows = smoother._read_positions(selected_output_path, LEAP_SECONDS)
            if [row.timestamp_ms for row in selected_rows] != list(selected_keys):
                raise TestBatchError(f"resumed selected output keys differ: {dataset_id}")
            states[phone] = {
                "dataset_id": dataset_id,
                "route": route,
                "phone": phone,
                "device_path": device_path,
                "nav_path": nav_path,
                "positions": positions,
                "position_timestamps": [row.timestamp_ms for row in positions],
                "position_by_timestamp": position_by_timestamp,
                "selected_keys": selected_keys,
                "wls_manifest": wls_manifest,
                "wls_position": {"path": str(wls_position_path), "sha256": _sha256(wls_position_path), "bytes": wls_position_path.stat().st_size},
                "wls_summary": {"path": str(wls_summary_path), "sha256": _sha256(wls_summary_path), "bytes": wls_summary_path.stat().st_size},
                "wls_manifest_artifact": {"path": str(wls_manifest_path), "sha256": _sha256(wls_manifest_path), "bytes": wls_manifest_path.stat().st_size},
                "cache_id": str(wls_info.get("cache_id", "resumed")),
                "cache_dir": wls_position_path.parent,
                "materialized": {"device_gnss": device_entry, "broadcast_nav": nav_entry},
                "native_fallback": None,
            }
            selected_positions[phone] = selected_rows
            sample_phone = f"{route}_{phone}"
            row_map.update({(sample_phone, row.timestamp_ms): (row.latitude, row.longitude) for row in selected_rows})
        if route in results:
            raise TestBatchError(f"resumed route is duplicated: {route}")
        results[route] = {
            "route": route,
            "states": states,
            "selected_positions": selected_positions,
            "alignment": payload.get("alignment", {}),
            "row_map": row_map,
            "route_manifest": dict(raw_entry),
            "route_manifest_path": route_manifest_path,
            "resumed": True,
        }
    return results, {
        "source_run_manifest": _artifact(manifest_path),
        "route_count": len(results),
        "expected_completed_count": expected_completed_count,
        "selected_output_hashes_reused": sum(
            len(result["selected_positions"]) for result in results.values()
        ),
        "truth_open_count": 0,
        "byte_identity_proven": True,
    }


def _load_native_fallback(
    fallback_root: Path | None,
    fallback_manifest_path: Path | None,
    dataset_id: str,
) -> dict[str, Any] | None:
    """Load an exact-key native stable artifact, if explicitly supplied."""

    entry: dict[str, Any] | None = None
    base: Path | None = None
    if fallback_manifest_path is not None:
        index = _load_json(fallback_manifest_path, "native fallback manifest")
        entries = index.get("entries", index.get("datasets"))
        if isinstance(entries, dict):
            raw = entries.get(dataset_id)
            entry = raw if isinstance(raw, dict) else None
        elif isinstance(entries, list):
            for raw in entries:
                if isinstance(raw, dict) and raw.get("dataset_id") == dataset_id:
                    entry = raw
                    break
        if entry is not None:
            base = fallback_manifest_path.parent
    elif fallback_root is not None:
        safe = _safe_id(dataset_id)
        candidates = [fallback_root / safe, fallback_root / dataset_id.split("/", 1)[0] / dataset_id.split("/", 1)[1]]
        for candidate in candidates:
            if not candidate.is_dir():
                continue
            position = next((candidate / name for name in ("selected.pos", "smoothed.pos", "native_stable.pos", "trajectory.pos") if (candidate / name).is_file()), None)
            report = next((candidate / name for name in ("segment_stability.json", "native_segment_stability.json") if (candidate / name).is_file()), None)
            manifest = candidate / "smoother_manifest.json"
            if position is not None and report is not None:
                entry = {"position": str(position), "stability_report": str(report), "manifest": str(manifest) if manifest.is_file() else None}
                base = ROOT
                break
    if entry is None:
        return None
    if not isinstance(entry.get("position"), str) or not isinstance(entry.get("stability_report"), str):
        raise TestBatchError(f"native fallback entry is incomplete: {dataset_id}")
    position_path = Path(entry["position"])
    report_path = Path(entry["stability_report"])
    if base is not None:
        if not position_path.is_absolute():
            position_path = (base / position_path).resolve()
        if not report_path.is_absolute():
            report_path = (base / report_path).resolve()
    try:
        report_summary = stability_selector._validate_segment_report(report_path)
    except (OSError, ValueError, KeyError, TypeError) as exc:
        raise TestBatchError(f"native fallback stability report is invalid: {dataset_id}") from exc
    raw_report = _load_json(report_path, "native fallback stability report")
    if raw_report.get("position_lane") != "native-galileo-e1-hatch30-segment-stability":
        raise TestBatchError(f"native fallback lane is not Galileo E1/Hatch30 stability: {dataset_id}")
    if not report_summary["all_segments_stable"]:
        raise TestBatchError(f"native fallback is not a stable native output: {dataset_id}")
    positions = smoother._read_positions(position_path, LEAP_SECONDS)
    if not positions:
        raise TestBatchError(f"native fallback has no positions: {dataset_id}")
    timestamps = [row.timestamp_ms for row in positions]
    if timestamps != sorted(set(timestamps)):
        raise TestBatchError(f"native fallback timestamps are not increasing: {dataset_id}")
    for row in positions:
        if not all(math.isfinite(float(value)) for value in (*row.ecef, row.latitude, row.longitude, row.height)):
            raise TestBatchError(f"native fallback contains non-finite position: {dataset_id}")
    manifest_artifact = None
    raw_manifest = entry.get("manifest")
    if isinstance(raw_manifest, str):
        manifest_path = Path(raw_manifest)
        if base is not None and not manifest_path.is_absolute():
            manifest_path = (base / manifest_path).resolve()
        manifest_payload = _load_json(manifest_path, "native fallback manifest")
        if manifest_payload.get("truth_used") is not False:
            raise TestBatchError(f"native fallback manifest is not truth-free: {dataset_id}")
        manifest_artifact = _artifact(manifest_path)
    for field, path in (("position_sha256", position_path), ("stability_report_sha256", report_path)):
        expected = entry.get(field)
        if expected is not None and expected != _sha256(path):
            raise TestBatchError(f"native fallback {field} differs: {dataset_id}")
    return {
        "position_path": position_path,
        "report_path": report_path,
        "positions": positions,
        "report": report_summary,
        "position": _artifact(position_path),
        "stability_report": _artifact(report_path),
        "manifest": manifest_artifact,
    }


def _nearest_position(
    positions: list[smoother.PositionRow], timestamps: list[int], timestamp: int, tolerance_ms: int
) -> tuple[smoother.PositionRow, int] | None:
    index = bisect.bisect_left(timestamps, timestamp)
    choices: list[tuple[int, smoother.PositionRow]] = []
    if index < len(positions):
        choices.append((abs(positions[index].timestamp_ms - timestamp), positions[index]))
    if index > 0:
        choices.append((abs(positions[index - 1].timestamp_ms - timestamp), positions[index - 1]))
    if not choices:
        return None
    delta, row = min(choices, key=lambda pair: (pair[0], pair[1].timestamp_ms))
    return (row, delta) if delta <= tolerance_ms else None


def _interpolate_position(
    positions: list[smoother.PositionRow],
    timestamps: list[int],
    timestamp: int,
    max_gap_ms: int = FALLBACK_INTERPOLATION_MAX_GAP_MS,
) -> tuple[smoother.PositionRow, int] | None:
    """Linearly interpolate only a bracketed, finite selected-source gap."""

    index = bisect.bisect_left(timestamps, timestamp)
    if index <= 0 or index >= len(positions):
        return None
    before = positions[index - 1]
    after = positions[index]
    gap_ms = after.timestamp_ms - before.timestamp_ms
    if gap_ms <= 0 or gap_ms > max_gap_ms:
        return None
    if timestamp <= before.timestamp_ms or timestamp >= after.timestamp_ms:
        return None
    fraction = (timestamp - before.timestamp_ms) / gap_ms
    if not math.isfinite(fraction) or not 0.0 < fraction < 1.0:
        return None
    first = np.asarray(before.ecef, dtype=float)
    second = np.asarray(after.ecef, dtype=float)
    ecef = first + fraction * (second - first)
    if ecef.shape != (3,) or not np.isfinite(ecef).all():
        raise TestBatchError(f"non-finite ECEF interpolation at {timestamp}")
    return (
        ensemble._position_from_ecef(
            timestamp,
            ecef,
            min(before.source_line, after.source_line),
            min(before.satellites, after.satellites),
        ),
        gap_ms,
    )


def _edge_hold_position(
    state: dict[str, Any],
    timestamp: int,
    max_gap_ms: int = FALLBACK_INTERPOLATION_MAX_GAP_MS,
) -> tuple[smoother.PositionRow, dict[str, Any]] | None:
    """Hold the nearest selected-source ECEF only at a legal edge.

    The helper deliberately requires the caller to provide the raw epoch
    clock/segment map.  Without that metadata, an edge source cannot be
    proven continuous and the operation fails closed rather than guessing.
    """

    positions = state.get("positions")
    timestamps = state.get("position_timestamps")
    if not isinstance(positions, list) or not isinstance(timestamps, list) or not positions:
        return None
    if len(positions) != len(timestamps) or timestamps != sorted(set(timestamps)):
        raise TestBatchError("edge hold selected-source timestamps are invalid")
    if timestamp >= timestamps[0] and timestamp <= timestamps[-1]:
        return None
    if timestamp < timestamps[0]:
        source = positions[0]
        direction = "leading"
    else:
        source = positions[-1]
        direction = "trailing"
    gap_ms = abs(source.timestamp_ms - timestamp)
    if gap_ms <= 0 or gap_ms > max_gap_ms:
        raise TestBatchError(
            f"{direction} edge hold exceeds {max_gap_ms / 1000.0:g} s at {timestamp}"
        )
    metadata = state.get("epoch_clock_segments")
    if not isinstance(metadata, dict):
        raise TestBatchError("edge hold lacks epoch clock/segment continuity metadata")
    clock_by_timestamp = metadata.get("clock_by_timestamp")
    segment_by_timestamp = metadata.get("segment_by_timestamp")
    target_keys = state.get("selected_keys")
    if (
        not isinstance(clock_by_timestamp, dict)
        or not isinstance(segment_by_timestamp, dict)
        or not isinstance(target_keys, list)
    ):
        raise TestBatchError("edge hold clock/segment metadata is incomplete")
    source_clock = clock_by_timestamp.get(source.timestamp_ms)
    source_segment = segment_by_timestamp.get(source.timestamp_ms)
    if not isinstance(source_clock, int) or not isinstance(source_segment, int):
        raise TestBatchError("edge hold source lacks clock/segment identity")
    span_keys = [
        value
        for value in target_keys
        if (value <= source.timestamp_ms if direction == "leading" else value >= source.timestamp_ms)
    ]
    if timestamp not in span_keys:
        raise TestBatchError("edge hold target is absent from selected epoch metadata")
    for value in span_keys:
        clock = clock_by_timestamp.get(value)
        segment = segment_by_timestamp.get(value)
        if clock != source_clock or segment != source_segment:
            raise TestBatchError(
                f"edge hold crosses a clock/segment boundary at {value}"
            )
    ecef = np.asarray(source.ecef, dtype=float)
    if ecef.shape != (3,) or not np.isfinite(ecef).all():
        raise TestBatchError(f"edge hold source has non-finite ECEF at {source.timestamp_ms}")
    row = ensemble._position_from_ecef(
        timestamp,
        ecef.copy(),
        source.source_line,
        source.satellites,
    )
    return row, {
        "direction": direction,
        "gap_ms": gap_ms,
        "source_timestamp_ms": source.timestamp_ms,
        "source_clock_discontinuity_count": source_clock,
        "source_segment_id": source_segment,
        "continuity_verified": True,
        "velocity_extrapolation": False,
    }


def _test_fuse_positions(
    states: dict[str, dict[str, Any]],
    *,
    alignment_tolerance_ms: int = DEFAULT_ALIGNMENT_TOLERANCE_MS,
    allow_completeness_fallback: bool = False,
    allow_edge_completeness_fallback: bool = False,
    edge_hold_max_gap_ms: int = FALLBACK_INTERPOLATION_MAX_GAP_MS,
) -> tuple[dict[str, list[smoother.PositionRow]], dict[str, Any]]:
    """Fuse a route and resolve sparse keys under the frozen fallback contract."""

    if alignment_tolerance_ms != DEFAULT_ALIGNMENT_TOLERANCE_MS:
        raise TestBatchError("test batch alignment tolerance is frozen at 10 ms")
    if allow_edge_completeness_fallback and not allow_completeness_fallback:
        raise TestBatchError("edge completeness fallback requires the base sparse fallback")
    if (
        not isinstance(edge_hold_max_gap_ms, int)
        or isinstance(edge_hold_max_gap_ms, bool)
        or edge_hold_max_gap_ms <= 0
    ):
        raise TestBatchError("edge hold max gap must be a positive integer in milliseconds")
    phones = sorted(states)
    if not phones:
        raise TestBatchError("route has no phone states")
    output: dict[str, list[smoother.PositionRow]] = {}
    epoch_records: list[dict[str, Any]] = []
    count_histogram: dict[str, int] = {}
    spread_values: list[float] = []
    offset_values: list[float] = []
    fallback_count = 0
    native_nearest_count = 0
    interpolation_count = 0
    edge_hold_count = 0
    edge_hold_gap_values: list[float] = []
    edge_hold_directions: dict[str, int] = {"leading": 0, "trailing": 0}
    unresolved_count = 0
    for target_phone in phones:
        state = states[target_phone]
        target_keys = list(state["selected_keys"])
        target_rows: list[smoother.PositionRow] = []
        for target_timestamp in target_keys:
            matches: list[tuple[str, smoother.PositionRow, int]] = []
            if len(phones) >= 2:
                for source_phone in phones:
                    source_positions = states[source_phone]["positions"]
                    nearest = _nearest_position(
                        source_positions,
                        states[source_phone]["position_timestamps"],
                        target_timestamp,
                        alignment_tolerance_ms,
                    )
                    if nearest is not None:
                        row, delta = nearest
                        matches.append((source_phone, row, delta))
                        if source_phone != target_phone:
                            offset_values.append(float(delta))
            else:
                exact = state["position_by_timestamp"].get(target_timestamp)
                if exact is not None:
                    matches.append((target_phone, exact, 0))
            if matches:
                values = [row.ecef for _, row, _ in matches]
                fused = (
                    np.asarray(values[0], dtype=float)
                    if len(phones) == 1
                    else ensemble._fuse(values, "coordinate_wise_ecef_median")
                )
                if not np.isfinite(fused).all():
                    raise TestBatchError(f"non-finite fused test position at {target_phone}/{target_timestamp}")
                spread = max(
                    (float(np.linalg.norm(first - second)) for first in values for second in values),
                    default=0.0,
                )
                spread_values.append(spread)
                row = (
                    matches[0][1]
                    if len(phones) == 1
                    else ensemble._position_from_ecef(
                        target_timestamp,
                        fused,
                        min(value[1].source_line for value in matches),
                        min(value[1].satellites for value in matches),
                    )
                )
                source = "wls_raw" if len(phones) == 1 else "coordinate_wise_ecef_median"
                native_fallback = False
            else:
                unresolved_count += 1
                fallback = state.get("native_fallback")
                native_match = None
                if fallback is not None:
                    if allow_completeness_fallback:
                        native_match = _nearest_position(
                            fallback["positions"],
                            [value.timestamp_ms for value in fallback["positions"]],
                            target_timestamp,
                            FALLBACK_NATIVE_TOLERANCE_MS,
                        )
                    else:
                        exact_native = fallback.get("position_by_timestamp", {}).get(
                            target_timestamp
                        )
                        if exact_native is not None:
                            native_match = (exact_native, 0)
                if native_match is not None:
                    native_source_row, native_delta = native_match
                    native_ecef = np.asarray(native_source_row.ecef, dtype=float)
                    if native_ecef.shape != (3,) or not np.isfinite(native_ecef).all():
                        raise TestBatchError(
                            f"native fallback contains non-finite ECEF at {target_timestamp}"
                        )
                    row = ensemble._position_from_ecef(
                        target_timestamp,
                        native_ecef.copy(),
                        native_source_row.source_line,
                        native_source_row.satellites,
                    )
                    spread = 0.0
                    source = "native_stability_nearest_fallback"
                    native_fallback = True
                    fallback_count += 1
                    native_nearest_count += 1
                    native_offset = native_delta
                elif allow_edge_completeness_fallback:
                    held = _edge_hold_position(
                        state, target_timestamp, edge_hold_max_gap_ms
                    )
                    if held is not None:
                        row, edge_meta = held
                        spread = 0.0
                        source = "wls_ecef_constant_edge_hold"
                        native_fallback = False
                        fallback_count += 1
                        edge_hold_count += 1
                        edge_hold_gap_values.append(float(edge_meta["gap_ms"]))
                        edge_hold_directions[edge_meta["direction"]] += 1
                        native_delta = None
                    else:
                        edge_meta = None
                else:
                    edge_meta = None
                if (
                    native_match is None
                    and (not allow_edge_completeness_fallback or edge_meta is None)
                    and allow_completeness_fallback
                ):
                    interpolated = _interpolate_position(
                        state["positions"],
                        state["position_timestamps"],
                        target_timestamp,
                    )
                    if interpolated is None:
                        raise TestBatchError(
                            "unresolved test epoch has no native <=10 ms match and no "
                            "bracketed WLS interpolation within 10 s at "
                            f"{target_phone}/{target_timestamp}"
                        )
                    row, interpolation_gap = interpolated
                    spread = 0.0
                    source = "wls_ecef_linear_interpolation"
                    native_fallback = False
                    fallback_count += 1
                    interpolation_count += 1
                    native_delta = None
                elif (
                    native_match is None
                    and (not allow_edge_completeness_fallback or edge_meta is None)
                    and not allow_completeness_fallback
                ):
                    raise TestBatchError(
                        "all-phone WLS unresolved and exact native fallback is unavailable at "
                        f"{target_phone}/{target_timestamp}"
                    )
            count_histogram[str(len(matches))] = count_histogram.get(str(len(matches)), 0) + 1
            target_rows.append(row)
            epoch_record = {
                "target_phone": target_phone,
                "timestamp_ms": target_timestamp,
                "used_phones": [phone for phone, _, _ in matches],
                "phone_count": len(matches),
                "source": source,
                "native_fallback": native_fallback,
                "spread_ecef_m": spread,
                "alignment_offsets_ms": [delta for _, _, delta in matches],
            }
            if source == "native_stability_nearest_fallback":
                epoch_record["native_offset_ms"] = native_delta
                epoch_record["native_tie_rule"] = "earlier timestamp on equal distance"
            if source == "wls_ecef_linear_interpolation":
                epoch_record["interpolation_source_gap_ms"] = interpolation_gap
                epoch_record["interpolation_policy"] = "bracketed selected-source ECEF linear interpolation"
            if source == "wls_ecef_constant_edge_hold":
                epoch_record.update(
                    {
                        "edge_hold_direction": edge_meta["direction"],
                        "edge_hold_gap_ms": edge_meta["gap_ms"],
                        "edge_hold_source_timestamp_ms": edge_meta["source_timestamp_ms"],
                        "edge_hold_source_clock_discontinuity_count": edge_meta[
                            "source_clock_discontinuity_count"
                        ],
                        "edge_hold_source_segment_id": edge_meta["source_segment_id"],
                        "edge_hold_continuity_verified": edge_meta["continuity_verified"],
                        "edge_hold_velocity_extrapolation": edge_meta[
                            "velocity_extrapolation"
                        ],
                        "edge_hold_policy": "constant nearest selected-source ECEF; no velocity extrapolation",
                    }
                )
            epoch_records.append(epoch_record)
        output[target_phone] = target_rows
    total = sum(len(state["selected_keys"]) for state in states.values())
    if unresolved_count and fallback_count != unresolved_count:
        raise TestBatchError("unresolved test epochs were not fully accounted for")
    return output, {
        "alignment_tolerance_ms": alignment_tolerance_ms,
        "completeness_fallback_enabled": allow_completeness_fallback,
        "target_epoch_count": total,
        "covered_target_epoch_count": total,
        "unresolved_wls_epoch_count": unresolved_count,
        "native_fallback_epoch_count": fallback_count,
        "native_nearest_fallback_epoch_count": native_nearest_count,
        "wls_interpolation_epoch_count": interpolation_count,
        "edge_constant_hold_epoch_count": edge_hold_count,
        "edge_constant_hold_max_gap_ms": edge_hold_max_gap_ms,
        "edge_constant_hold_gap_ms": {
            "p50_ms": float(kaggle._percentile_linear_n_minus_1(edge_hold_gap_values, 0.50))
            if edge_hold_gap_values
            else None,
            "p95_ms": float(kaggle._percentile_linear_n_minus_1(edge_hold_gap_values, 0.95))
            if edge_hold_gap_values
            else None,
            "max_ms": max(edge_hold_gap_values, default=0.0),
        },
        "edge_constant_hold_direction_counts": edge_hold_directions,
        "fallback_contract": dict(COMPLETENESS_FALLBACK_CONTRACT),
        "edge_fallback_contract": (
            dict(
                EDGE_COMPLETENESS_FALLBACK_V2_1_CONTRACT
                if edge_hold_max_gap_ms == EDGE_COMPLETENESS_FALLBACK_V2_1_MAX_GAP_MS
                else EDGE_COMPLETENESS_FALLBACK_CONTRACT
            )
            if allow_edge_completeness_fallback
            else None
        ),
        "all_target_timestamps_covered": True,
        "extrapolation_used": False,
        "tie_rule": "equal-distance source timestamps choose earlier timestamp",
        "phone_count_histogram": count_histogram,
        "max_abs_alignment_offset_ms": max(offset_values, default=0.0),
        "spread_ecef_m": {
            "p50_m": float(kaggle._percentile_linear_n_minus_1(spread_values, 0.50)) if spread_values else None,
            "p95_m": float(kaggle._percentile_linear_n_minus_1(spread_values, 0.95)) if spread_values else None,
            "max_m": max(spread_values, default=0.0),
        },
        "epoch_manifest": epoch_records,
    }


def _cache_key(
    archive_hash: str,
    inventory_hash: str,
    authorization_hash: str,
    record: dict[str, Any],
    device_artifact: dict[str, Any],
    nav_artifact: dict[str, Any],
) -> str:
    payload = {
        "archive_sha256": archive_hash,
        "inventory_sha256": inventory_hash,
        "authorization_sha256": authorization_hash,
        "dataset_id": record["dataset_id"],
        "device_gnss": device_artifact,
        "broadcast_nav": nav_artifact,
        "algorithm_core_hash": wls.MULTI_PHONE_HOLDOUT_ALGORITHM_CORE_HASH,
        "alignment_tolerance_ms": DEFAULT_ALIGNMENT_TOLERANCE_MS,
        "skip_epochs": SKIP_EPOCHS,
    }
    return hashlib.sha256(json.dumps(payload, sort_keys=True, separators=(",", ":")).encode("utf-8")).hexdigest()


def _write_route(
    archive_path: Path,
    record: dict[str, Any],
    output_dir: Path,
    cache_root: Path,
    authorization_path: Path,
    authorization_manifest_path: Path,
    archive_hash: str,
    inventory_hash: str,
    fallback_root: Path | None,
    fallback_manifest_path: Path | None,
    allow_completeness_fallback: bool = False,
    allow_edge_completeness_fallback: bool = False,
    edge_hold_max_gap_ms: int = FALLBACK_INTERPOLATION_MAX_GAP_MS,
) -> dict[str, Any]:
    route = str(record["route"])
    phone = str(record["phone"])
    dataset_id = str(record["dataset_id"])
    route_root = output_dir / "routes" / _safe_id(route)
    input_root = route_root / "inputs" / phone
    input_root.mkdir(parents=True, exist_ok=True)
    device_metadata = record["central_directory_files"].get("device_gnss.csv")
    nav_metadata = record["central_directory_broadcast_nav"]
    if not isinstance(device_metadata, dict) or not isinstance(nav_metadata, dict):
        raise TestBatchError(f"route central metadata is malformed: {dataset_id}")
    names = {"device_gnss": _test_member(route, phone, "device_gnss.csv"), "broadcast_nav": _test_nav_member(route)}
    materialized = {
        "device_gnss": _materialize_member(archive_path, names["device_gnss"], input_root / "device_gnss.csv", device_metadata),
        "broadcast_nav": _materialize_member(archive_path, names["broadcast_nav"], input_root / "brdc.nav", nav_metadata),
    }
    device_path = input_root / "device_gnss.csv"
    nav_path = input_root / "brdc.nav"
    cache_id = _cache_key(
        archive_hash,
        inventory_hash,
        _sha256(authorization_path),
        record,
        materialized["device_gnss"],
        materialized["broadcast_nav"],
    )
    cache_dir = cache_root / cache_id
    cached = _verify_cached_wls(cache_dir, device_path, authorization_path, authorization_manifest_path, dataset_id)
    if cached is None:
        cache_dir.mkdir(parents=True, exist_ok=True)
        wls.extract_to_directory(
            device_path,
            cache_dir,
            skip_epochs=SKIP_EPOCHS,
            role="test",
            dataset_id=dataset_id,
            sealed_test_authorization=authorization_path,
            sealed_test_authorization_manifest=authorization_manifest_path,
            truth_free=True,
            allow_missing_wls_epochs=True,
            allow_invalid_wls_epochs=allow_completeness_fallback,
        )
        position_path = cache_dir / "wls.pos"
        wls_manifest = _load_json(cache_dir / "wls_manifest.json", "WLS integrity manifest")
    else:
        position_path, wls_manifest = cached
    positions = smoother._read_positions(position_path, LEAP_SECONDS)
    summary = _load_json(cache_dir / "wls_summary.json", "WLS summary")
    selected_keys = summary.get("selection", {}).get("selected_device_epoch_timestamps_ms")
    if not isinstance(selected_keys, list) or not selected_keys or any(not isinstance(value, int) for value in selected_keys):
        raise TestBatchError(f"WLS summary has invalid selected test keys: {dataset_id}")
    if selected_keys != sorted(set(selected_keys)):
        raise TestBatchError(f"WLS selected test keys are not increasing: {dataset_id}")
    position_by_timestamp = {row.timestamp_ms: row for row in positions}
    if len(position_by_timestamp) != len(positions):
        raise TestBatchError(f"WLS position has duplicate test keys: {dataset_id}")
    epoch_clock_segments = _read_epoch_clock_segments(device_path, skip_epochs=SKIP_EPOCHS)
    fallback = _load_native_fallback(fallback_root, fallback_manifest_path, dataset_id)
    if fallback is not None:
        fallback["position_by_timestamp"] = {row.timestamp_ms: row for row in fallback["positions"]}
    state = {
        "dataset_id": dataset_id,
        "route": route,
        "phone": phone,
        "device_path": device_path,
        "nav_path": nav_path,
        "positions": positions,
        "position_timestamps": [row.timestamp_ms for row in positions],
        "position_by_timestamp": position_by_timestamp,
        "selected_keys": selected_keys,
        "epoch_clock_segments": epoch_clock_segments,
        "wls_manifest": wls_manifest,
        "wls_position": _artifact(position_path),
        "wls_summary": _artifact(cache_dir / "wls_summary.json"),
        "wls_manifest_artifact": _artifact(cache_dir / "wls_manifest.json"),
        "cache_id": cache_id,
        "cache_dir": cache_dir,
        "materialized": materialized,
        "native_fallback": fallback,
    }
    states = {phone: state}
    selected_positions, alignment = _test_fuse_positions(
        states,
        allow_completeness_fallback=allow_completeness_fallback,
        allow_edge_completeness_fallback=allow_edge_completeness_fallback,
        edge_hold_max_gap_ms=edge_hold_max_gap_ms,
    )
    selected_rows = selected_positions[phone]
    position_output = route_root / "outputs" / phone / "selected.pos"
    wls._write_pos(position_output, selected_rows)
    row_map = {
        (f"{route}_{phone}", row.timestamp_ms): (row.latitude, row.longitude)
        for row in selected_rows
    }
    route_manifest = {
        "schema_version": ROUTE_MANIFEST_SCHEMA,
        "route": route,
        "dataset_ids": [dataset_id],
        "phones": [phone],
        "lane": "wls_raw",
        "truth_free": True,
        "truth_used": False,
        "materialized": materialized,
        "wls": {phone: {
            "cache_id": cache_id,
            "position": state["wls_position"],
            "summary": state["wls_summary"],
            "manifest": state["wls_manifest_artifact"],
        }},
        "native_fallback": ({phone: {
            "position": fallback["position"],
            "stability_report": fallback["stability_report"],
            "manifest": fallback["manifest"],
        }} if fallback is not None else {}),
        "selected_output": {"phone": phone, "position": _artifact(position_output)},
        "alignment": alignment,
        "contract": {
            "alignment_tolerance_ms": DEFAULT_ALIGNMENT_TOLERANCE_MS,
            "selected_method": "raw WLS for single-phone route",
            "interpolation": "forbidden",
            "extrapolation": "forbidden",
            "completeness_fallback": (
                dict(
                    EDGE_COMPLETENESS_FALLBACK_V2_1_CONTRACT
                    if allow_edge_completeness_fallback
                    and edge_hold_max_gap_ms == EDGE_COMPLETENESS_FALLBACK_V2_1_MAX_GAP_MS
                    else EDGE_COMPLETENESS_FALLBACK_CONTRACT
                    if allow_edge_completeness_fallback
                    else COMPLETENESS_FALLBACK_CONTRACT
                )
                if allow_completeness_fallback
                else None
            ),
            "edge_constant_hold": allow_edge_completeness_fallback,
            "edge_constant_hold_max_gap_ms": edge_hold_max_gap_ms,
            "truth_access": "forbidden; ground_truth members never materialized",
        },
    }
    route_manifest_path = route_root / "route_manifest.json"
    _atomic_json(route_manifest_path, route_manifest)
    return {
        "route": route,
        "states": states,
        "selected_positions": selected_positions,
        "alignment": alignment,
        "row_map": row_map,
        "route_manifest": _artifact(route_manifest_path),
        "route_manifest_path": route_manifest_path,
        "position_output": _artifact(position_output),
    }


def _write_multi_phone_route(
    archive_path: Path,
    records: list[dict[str, Any]],
    output_dir: Path,
    cache_root: Path,
    authorization_path: Path,
    authorization_manifest_path: Path,
    archive_hash: str,
    inventory_hash: str,
    fallback_root: Path | None,
    fallback_manifest_path: Path | None,
    allow_completeness_fallback: bool = False,
    allow_edge_completeness_fallback: bool = False,
    edge_hold_max_gap_ms: int = FALLBACK_INTERPOLATION_MAX_GAP_MS,
) -> dict[str, Any]:
    """Process a route with one or more phones using one shared fuse pass."""

    route_started = time.perf_counter()
    rss_before = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss
    route = str(records[0]["route"])
    if any(record.get("route") != route for record in records):
        raise TestBatchError("route group contains multiple routes")
    route_root = output_dir / "routes" / _safe_id(route)
    route_root.mkdir(parents=True, exist_ok=True)
    states: dict[str, dict[str, Any]] = {}
    materialized: dict[str, Any] = {}
    for record in sorted(records, key=lambda value: str(value["phone"])):
        phone = str(record["phone"])
        dataset_id = str(record["dataset_id"])
        input_root = route_root / "inputs" / phone
        input_root.mkdir(parents=True, exist_ok=True)
        device_metadata = record["central_directory_files"].get("device_gnss.csv")
        nav_metadata = record["central_directory_broadcast_nav"]
        if not isinstance(device_metadata, dict) or not isinstance(nav_metadata, dict):
            raise TestBatchError(f"route central metadata is malformed: {dataset_id}")
        device_materialized = _materialize_member(
            archive_path,
            _test_member(route, phone, "device_gnss.csv"),
            input_root / "device_gnss.csv",
            device_metadata,
        )
        nav_materialized = _materialize_member(
            archive_path,
            _test_nav_member(route),
            input_root / "brdc.nav",
            nav_metadata,
        )
        materialized[phone] = {"device_gnss": device_materialized, "broadcast_nav": nav_materialized}
        device_path = input_root / "device_gnss.csv"
        cache_id = _cache_key(
            archive_hash,
            inventory_hash,
            _sha256(authorization_path),
            record,
            device_materialized,
            nav_materialized,
        )
        cache_dir = cache_root / cache_id
        cached = _verify_cached_wls(cache_dir, device_path, authorization_path, authorization_manifest_path, dataset_id)
        if cached is None:
            cache_dir.mkdir(parents=True, exist_ok=True)
            wls.extract_to_directory(
                device_path,
                cache_dir,
                skip_epochs=SKIP_EPOCHS,
                role="test",
                dataset_id=dataset_id,
                sealed_test_authorization=authorization_path,
                sealed_test_authorization_manifest=authorization_manifest_path,
                truth_free=True,
                allow_missing_wls_epochs=True,
                allow_invalid_wls_epochs=allow_completeness_fallback,
            )
            position_path = cache_dir / "wls.pos"
            wls_manifest = _load_json(cache_dir / "wls_manifest.json", "WLS integrity manifest")
        else:
            position_path, wls_manifest = cached
        positions = smoother._read_positions(position_path, LEAP_SECONDS)
        summary = _load_json(cache_dir / "wls_summary.json", "WLS summary")
        selected_keys = summary.get("selection", {}).get("selected_device_epoch_timestamps_ms")
        if not isinstance(selected_keys, list) or not selected_keys or any(not isinstance(value, int) for value in selected_keys):
            raise TestBatchError(f"WLS summary has invalid selected test keys: {dataset_id}")
        if selected_keys != sorted(set(selected_keys)):
            raise TestBatchError(f"WLS selected test keys are not increasing: {dataset_id}")
        derived_key_source = _read_device_epoch_keys(device_path, skip_epochs=SKIP_EPOCHS)
        if derived_key_source["timestamps_ms"] != selected_keys:
            raise TestBatchError(f"WLS selected keys differ from device epochs: {dataset_id}")
        epoch_clock_segments = _read_epoch_clock_segments(device_path, skip_epochs=SKIP_EPOCHS)
        fallback = _load_native_fallback(fallback_root, fallback_manifest_path, dataset_id)
        if fallback is not None:
            fallback["position_by_timestamp"] = {row.timestamp_ms: row for row in fallback["positions"]}
        states[phone] = {
            "dataset_id": dataset_id,
            "route": route,
            "phone": phone,
            "device_path": device_path,
            "nav_path": input_root / "brdc.nav",
            "positions": positions,
            "position_timestamps": [row.timestamp_ms for row in positions],
            "position_by_timestamp": {row.timestamp_ms: row for row in positions},
            "selected_keys": selected_keys,
            "epoch_clock_segments": epoch_clock_segments,
            "derived_key_source": derived_key_source,
            "wls_manifest": wls_manifest,
            "wls_position": _artifact(position_path),
            "wls_summary": _artifact(cache_dir / "wls_summary.json"),
            "wls_manifest_artifact": _artifact(cache_dir / "wls_manifest.json"),
            "cache_id": cache_id,
            "cache_dir": cache_dir,
            "materialized": {"device_gnss": device_materialized, "broadcast_nav": nav_materialized},
            "native_fallback": fallback,
        }
    selected_positions, alignment = _test_fuse_positions(
        states,
        allow_completeness_fallback=allow_completeness_fallback,
        allow_edge_completeness_fallback=allow_edge_completeness_fallback,
        edge_hold_max_gap_ms=edge_hold_max_gap_ms,
    )
    route_outputs: dict[str, Any] = {}
    row_map: dict[tuple[str, int], tuple[float, float]] = {}
    for phone, state in states.items():
        position_output = route_root / "outputs" / phone / "selected.pos"
        wls._write_pos(position_output, selected_positions[phone])
        sample_phone = f"{route}_{phone}"
        row_map.update({(sample_phone, row.timestamp_ms): (row.latitude, row.longitude) for row in selected_positions[phone]})
        route_outputs[phone] = {
            "selected_position": _artifact(position_output),
            "wls": {
                "cache_id": state["cache_id"],
                "position": state["wls_position"],
                "summary": state["wls_summary"],
                "manifest": state["wls_manifest_artifact"],
            },
            "native_fallback": ({
                "position": state["native_fallback"]["position"],
                "stability_report": state["native_fallback"]["stability_report"],
                "manifest": state["native_fallback"]["manifest"],
            } if state["native_fallback"] is not None else None),
        }
    route_manifest_path = route_root / "route_manifest.json"
    _atomic_json(
        route_manifest_path,
        {
            "schema_version": ROUTE_MANIFEST_SCHEMA,
            "route": route,
            "dataset_ids": [state["dataset_id"] for state in states.values()],
            "phones": sorted(states),
            "lane": "coordinate_wise_ecef_median" if len(states) >= 2 else "wls_raw",
            "truth_free": True,
            "truth_used": False,
            "materialized": materialized,
            "outputs": route_outputs,
            "alignment": alignment,
            "source_count": alignment["phone_count_histogram"],
            "spread_ecef_m": alignment["spread_ecef_m"],
            "timing": {
                "route_wall_s": time.perf_counter() - route_started,
                "route_peak_rss_kb": max(
                    int(rss_before),
                    int(resource.getrusage(resource.RUSAGE_SELF).ru_maxrss),
                ),
                "process_rss_unit": "kilobytes from getrusage ru_maxrss",
            },
            "contract": {
                "alignment_tolerance_ms": DEFAULT_ALIGNMENT_TOLERANCE_MS,
                "selected_method": "coordinate-wise ECEF median for >=2 phones; raw WLS for one phone",
                "provisional_key_order": "device_gnss.csv utcTimeMillis after frozen skip_epochs; official sample not used",
                "interpolation": (
                    "ECEF linear only for bracketed selected-source positions with gap <=10 s"
                    if allow_completeness_fallback
                    else "forbidden"
                ),
                "edge_constant_hold": (
                    f"constant nearest selected-source ECEF for leading/trailing edge <={edge_hold_max_gap_ms / 1000.0:g} s with clock continuity"
                    if allow_edge_completeness_fallback
                    else "forbidden"
                ),
                "extrapolation": "forbidden",
                "completeness_fallback": (
                    dict(
                        EDGE_COMPLETENESS_FALLBACK_V2_1_CONTRACT
                        if allow_edge_completeness_fallback
                        and edge_hold_max_gap_ms == EDGE_COMPLETENESS_FALLBACK_V2_1_MAX_GAP_MS
                        else EDGE_COMPLETENESS_FALLBACK_CONTRACT
                        if allow_edge_completeness_fallback
                        else COMPLETENESS_FALLBACK_CONTRACT
                    )
                    if allow_completeness_fallback
                    else None
                ),
                "truth_access": "forbidden; ground_truth members never materialized",
            },
        },
    )
    return {
        "route": route,
        "states": states,
        "selected_positions": selected_positions,
        "alignment": alignment,
        "row_map": row_map,
        "route_manifest": _artifact(route_manifest_path),
        "route_manifest_path": route_manifest_path,
    }


def _resolve_sample_dataset(
    sample_phone: str,
    records: list[dict[str, Any]],
) -> dict[str, Any]:
    exact = [record for record in records if sample_phone in (record["sample_phone_key"], record["dataset_id"])]
    if exact:
        if len(exact) != 1:
            raise TestBatchError(f"sample phone key is ambiguous: {sample_phone}")
        return exact[0]
    model_matches = [record for record in records if sample_phone == record.get("phone")]
    if len(model_matches) == 1:
        return model_matches[0]
    if len(model_matches) > 1:
        raise TestBatchError(
            f"sample phone key {sample_phone!r} is a non-unique model alias; route-qualified key required"
        )
    raise TestBatchError(f"sample phone key is not in the test inventory: {sample_phone}")


def _ordered_submission_bytes(
    sample: dict[str, Any],
    records: list[dict[str, Any]],
    row_map: dict[tuple[str, int], tuple[float, float]],
) -> tuple[bytes, dict[str, Any]]:
    output_rows: list[tuple[str, int, str, str]] = []
    sample_to_canonical: dict[str, str] = {}
    for raw in sample["rows"]:
        record = _resolve_sample_dataset(raw["phone"], records)
        canonical = str(record["sample_phone_key"])
        sample_to_canonical[raw["phone"]] = canonical
        coordinate = row_map.get((canonical, raw["timestamp"]))
        if coordinate is None:
            raise TestBatchError(
                f"sample key is missing from truth-free output: {raw['phone']}/{raw['timestamp']}"
            )
        latitude, longitude = coordinate
        if not all(math.isfinite(float(value)) for value in (latitude, longitude)):
            raise TestBatchError(f"non-finite output coordinate: {raw['phone']}/{raw['timestamp']}")
        if not -90.0 <= latitude <= 90.0 or not -180.0 <= longitude <= 180.0:
            raise TestBatchError(f"output coordinate is outside range: {raw['phone']}/{raw['timestamp']}")
        output_rows.append((raw["phone"], raw["timestamp"], f"{latitude:.12f}", f"{longitude:.12f}"))
    expected = set(row_map)
    seen_canonical = {(sample_to_canonical[phone], timestamp) for phone, timestamp, _, _ in output_rows}
    if seen_canonical != expected:
        missing = sorted(expected - seen_canonical)[:3]
        extra = sorted(seen_canonical - expected)[:3]
        raise TestBatchError(f"sample/output key-set mismatch; missing={missing}, extra={extra}")
    buffer = io.StringIO(newline="")
    writer = csv.writer(buffer, lineterminator="\n")
    writer.writerow(SAMPLE_FIELDS)
    writer.writerows(output_rows)
    return buffer.getvalue().encode("utf-8"), {
        "authoritative_sample_key_count": len(sample["rows"]),
        "output_key_count": len(output_rows),
        "duplicate_keys": 0,
        "missing_keys": 0,
        "extra_keys": 0,
        "nonfinite_coordinates": 0,
        "exact_sample_order": True,
    }


def _canonical_provisional_phone(dataset_id: str) -> str:
    """Validate the route/phone key used by the provisional lane."""

    if not isinstance(dataset_id, str) or dataset_id.count("/") != 1:
        raise TestBatchError("derived phone key must be a route/phone dataset ID")
    try:
        dataset_id.encode("ascii")
    except UnicodeEncodeError as exc:
        raise TestBatchError("derived phone key must contain ASCII only") from exc
    if not dataset_id.isprintable() or any(character.isspace() for character in dataset_id):
        raise TestBatchError("derived phone key must contain printable non-whitespace ASCII")
    return dataset_id


def _derived_key_manifest(
    route_results: list[dict[str, Any]],
    records: list[dict[str, Any]],
    output_dir: Path,
) -> tuple[Path, dict[tuple[str, int], tuple[float, float]], dict[str, Any]]:
    """Publish the canonical device-epoch key set for the explicit opt-in lane."""

    expected_dataset_ids = {
        _canonical_provisional_phone(str(record["dataset_id"])) for record in records
    }
    key_entries: list[dict[str, Any]] = []
    row_map: dict[tuple[str, int], tuple[float, float]] = {}
    phone_summaries: list[dict[str, Any]] = []
    seen_dataset_ids: set[str] = set()
    for result in sorted(route_results, key=lambda value: str(value["route"])):
        states = result.get("states")
        selected_positions = result.get("selected_positions")
        if not isinstance(states, dict) or not isinstance(selected_positions, dict):
            raise TestBatchError("route result lacks states for derived key manifest")
        for phone, state in sorted(states.items()):
            dataset_id = _canonical_provisional_phone(str(state.get("dataset_id", "")))
            if dataset_id in seen_dataset_ids:
                raise TestBatchError(f"duplicate derived dataset key: {dataset_id}")
            seen_dataset_ids.add(dataset_id)
            device_path = state.get("device_path")
            if not isinstance(device_path, Path):
                raise TestBatchError(f"derived key source path is invalid: {dataset_id}")
            key_source = _read_device_epoch_keys(device_path, skip_epochs=SKIP_EPOCHS)
            selected_keys = state.get("selected_keys")
            if selected_keys != key_source["timestamps_ms"]:
                raise TestBatchError(
                    f"WLS selected keys differ from device epoch keys: {dataset_id}"
                )
            positions = selected_positions.get(phone)
            if not isinstance(positions, list):
                raise TestBatchError(f"derived route output is missing phone: {dataset_id}")
            position_timestamps = [int(row.timestamp_ms) for row in positions]
            if position_timestamps != key_source["timestamps_ms"]:
                raise TestBatchError(
                    f"selected output keys differ from device epoch keys: {dataset_id}"
                )
            if position_timestamps != sorted(set(position_timestamps)):
                raise TestBatchError(f"derived output timestamps are not strictly increasing: {dataset_id}")
            for timestamp, row in zip(key_source["timestamps_ms"], positions):
                if not isinstance(timestamp, int):
                    raise TestBatchError(f"derived timestamp is not an integer: {dataset_id}")
                try:
                    timestamp_finite = math.isfinite(float(timestamp))
                except (OverflowError, ValueError):
                    timestamp_finite = False
                if not timestamp_finite:
                    raise TestBatchError(f"derived timestamp is not finite: {dataset_id}")
                latitude = float(row.latitude)
                longitude = float(row.longitude)
                if not all(math.isfinite(value) for value in (latitude, longitude)):
                    raise TestBatchError(
                        f"derived output coordinate is non-finite: {dataset_id}/{timestamp}"
                    )
                if not -90.0 <= latitude <= 90.0 or not -180.0 <= longitude <= 180.0:
                    raise TestBatchError(
                        f"derived output coordinate is outside range: {dataset_id}/{timestamp}"
                    )
                key = (dataset_id, timestamp)
                if key in row_map:
                    raise TestBatchError(f"duplicate derived output key: {dataset_id}/{timestamp}")
                row_map[key] = (latitude, longitude)
                key_entries.append(
                    {
                        "phone": dataset_id,
                        "UnixTimeMillis": timestamp,
                        "position_source": (
                            "wls_raw"
                            if len(states) == 1
                            else "coordinate_wise_ecef_median"
                        ),
                    }
                )
            phone_summaries.append(
                {
                    "dataset_id": dataset_id,
                    "phone": dataset_id,
                    "route": str(state["route"]),
                    "lane": "wls_raw" if len(states) == 1 else "coordinate_wise_ecef_median",
                    "device_gnss": key_source["source"],
                    "source_field": key_source["source_field"],
                    "input_rows": key_source["input_rows"],
                    "input_epoch_count": key_source["input_epoch_count"],
                    "repeated_raw_timestamp_rows": key_source["repeated_raw_timestamp_rows"],
                    "skip_epochs": key_source["skip_epochs"],
                    "selected_epoch_count": key_source["selected_epoch_count"],
                    "duplicate_keys": 0,
                    "nonfinite_timestamps": 0,
                }
            )
    if seen_dataset_ids != expected_dataset_ids:
        missing = sorted(expected_dataset_ids - seen_dataset_ids)
        extra = sorted(seen_dataset_ids - expected_dataset_ids)
        raise TestBatchError(f"derived dataset key-set mismatch; missing={missing}, extra={extra}")
    key_entries.sort(key=lambda entry: (str(entry["phone"]), int(entry["UnixTimeMillis"])))
    expected_keys = [(entry["phone"], entry["UnixTimeMillis"]) for entry in key_entries]
    if expected_keys != sorted(set(expected_keys)):
        raise TestBatchError("derived provisional key manifest contains duplicate or unsorted keys")
    manifest = {
        "schema_version": DERIVED_KEY_MANIFEST_SCHEMA,
        "status": "provisional-unverified-key-order",
        "truth_free": True,
        "truth_used": False,
        "official_sample_verified": False,
        "cannot_submit_to_kaggle": True,
        "derived_key_order_opt_in": True,
        "canonical_phone_format": "ASCII route/phone dataset ID",
        "order": "bytewise ASCII phone ascending, then numeric UnixTimeMillis ascending",
        "timestamp_source": "materialized device_gnss.csv utcTimeMillis Raw epochs",
        "skip_epochs": SKIP_EPOCHS,
        "algorithm_core_hash": wls.MULTI_PHONE_HOLDOUT_ALGORITHM_CORE_HASH,
        "algorithm_parameter_hash": wls.MULTI_PHONE_HOLDOUT_ALGORITHM_PARAMETER_HASH,
        "phones": phone_summaries,
        "keys": key_entries,
        "key_count": len(key_entries),
        "validation": {
            "duplicate_keys": 0,
            "missing_keys": 0,
            "extra_keys": 0,
            "nonfinite_timestamps": 0,
            "nonfinite_coordinates": 0,
            "monotonic_per_phone": True,
        },
        "truth_policy": {
            "truth_open_count": 0,
            "truth_materialized_count": 0,
            "ground_truth_members_read": False,
        },
    }
    path = output_dir / "provisional_key_manifest.json"
    _atomic_json(path, manifest)
    return path, row_map, manifest


def _derived_submission_bytes(
    key_manifest: dict[str, Any],
    row_map: dict[tuple[str, int], tuple[float, float]],
) -> tuple[bytes, dict[str, Any]]:
    """Serialize the provisional self-keyed submission in canonical order."""

    keys = key_manifest.get("keys")
    if not isinstance(keys, list) or not keys:
        raise TestBatchError("derived key manifest contains no keys")
    output_rows: list[tuple[str, int, str, str]] = []
    seen: set[tuple[str, int]] = set()
    for entry in keys:
        if not isinstance(entry, dict):
            raise TestBatchError("derived key manifest key entry is invalid")
        phone = _canonical_provisional_phone(str(entry.get("phone", "")))
        timestamp = entry.get("UnixTimeMillis")
        if isinstance(timestamp, bool) or not isinstance(timestamp, int) or timestamp < 0:
            raise TestBatchError(f"derived key timestamp is invalid: {phone}")
        key = (phone, timestamp)
        if key in seen:
            raise TestBatchError(f"derived submission contains duplicate key: {phone}/{timestamp}")
        seen.add(key)
        coordinate = row_map.get(key)
        if coordinate is None:
            raise TestBatchError(f"derived submission key is unresolved: {phone}/{timestamp}")
        latitude, longitude = (float(coordinate[0]), float(coordinate[1]))
        if not all(math.isfinite(value) for value in (latitude, longitude)):
            raise TestBatchError(f"derived submission coordinate is non-finite: {phone}/{timestamp}")
        if not -90.0 <= latitude <= 90.0 or not -180.0 <= longitude <= 180.0:
            raise TestBatchError(f"derived submission coordinate is outside range: {phone}/{timestamp}")
        output_rows.append((phone, timestamp, f"{latitude:.12f}", f"{longitude:.12f}"))
    if seen != set(row_map):
        missing = sorted(set(row_map) - seen)[:3]
        extra = sorted(seen - set(row_map))[:3]
        raise TestBatchError(f"derived submission key-set mismatch; missing={missing}, extra={extra}")
    if [(phone, timestamp) for phone, timestamp, _, _ in output_rows] != sorted(seen):
        raise TestBatchError("derived submission is not in canonical phone/timestamp order")
    buffer = io.StringIO(newline="")
    writer = csv.writer(buffer, lineterminator="\n")
    writer.writerow(SAMPLE_FIELDS)
    writer.writerows(output_rows)
    return buffer.getvalue().encode("utf-8"), {
        "authoritative_sample_verified": False,
        "derived_key_order": True,
        "cannot_submit_to_kaggle": True,
        "key_count": len(output_rows),
        "duplicate_keys": 0,
        "missing_keys": 0,
        "extra_keys": 0,
        "nonfinite_coordinates": 0,
        "exact_canonical_order": True,
    }


def _read_generated_submission(path: Path) -> dict[tuple[str, int], tuple[float, float]]:
    """Read a generated CSV for strict sample-only promotion."""

    if not path.is_file():
        raise TestBatchError(f"missing provisional submission: {path}")
    rows: dict[tuple[str, int], tuple[float, float]] = {}
    try:
        with path.open(encoding="utf-8-sig", newline="") as handle:
            reader = csv.DictReader(handle)
            if tuple(reader.fieldnames or ()) != SAMPLE_FIELDS:
                raise TestBatchError("generated submission header is invalid")
            for line, raw in enumerate(reader, start=2):
                if None in raw:
                    raise TestBatchError(f"generated submission row {line} has extra columns")
                phone = raw.get("phone")
                if not isinstance(phone, str) or not phone:
                    raise TestBatchError(f"generated submission row {line} has invalid phone")
                timestamp_token = (raw.get("UnixTimeMillis") or "").strip()
                try:
                    timestamp = int(timestamp_token)
                except ValueError as exc:
                    raise TestBatchError(f"generated submission row {line} has invalid timestamp") from exc
                if timestamp < 0:
                    raise TestBatchError(f"generated submission row {line} has negative timestamp")
                try:
                    latitude = float((raw.get("LatitudeDegrees") or "").strip())
                    longitude = float((raw.get("LongitudeDegrees") or "").strip())
                except ValueError as exc:
                    raise TestBatchError(f"generated submission row {line} has invalid coordinate") from exc
                if not all(math.isfinite(value) for value in (latitude, longitude)):
                    raise TestBatchError(f"generated submission row {line} has non-finite coordinate")
                if not -90.0 <= latitude <= 90.0 or not -180.0 <= longitude <= 180.0:
                    raise TestBatchError(f"generated submission row {line} has out-of-range coordinate")
                key = (phone, timestamp)
                if key in rows:
                    raise TestBatchError(f"generated submission duplicates key {key!r}")
                rows[key] = (latitude, longitude)
    except OSError as exc:
        raise TestBatchError(f"failed to read generated submission: {path}") from exc
    if not rows:
        raise TestBatchError("generated submission contains no rows")
    return rows


def _read_official_sample_submission(path: Path) -> dict[str, Any]:
    """Read the authenticated official sample without changing its key values."""

    if not path.is_file():
        raise TestBatchError(f"official sample submission is missing: {path}")
    rows: list[dict[str, Any]] = []
    seen: set[tuple[str, int]] = set()
    try:
        with path.open(encoding="utf-8-sig", newline="") as handle:
            reader = csv.DictReader(handle)
            if tuple(reader.fieldnames or ()) != OFFICIAL_SAMPLE_FIELDS:
                raise TestBatchError(
                    "official sample header must be exactly "
                    + ",".join(OFFICIAL_SAMPLE_FIELDS)
                )
            for line, raw in enumerate(reader, start=2):
                if None in raw:
                    raise TestBatchError(f"official sample row {line} has extra columns")
                trip_id = raw.get("tripId")
                if (
                    not isinstance(trip_id, str)
                    or not trip_id
                    or trip_id != trip_id.strip()
                    or not trip_id.isprintable()
                    or any(character.isspace() for character in trip_id)
                ):
                    raise TestBatchError(f"official sample row {line} has an invalid tripId")
                timestamp_token = raw.get("UnixTimeMillis") or ""
                if timestamp_token != timestamp_token.strip() or not timestamp_token:
                    raise TestBatchError(
                        f"official sample row {line} has an invalid UnixTimeMillis"
                    )
                try:
                    timestamp = int(timestamp_token)
                except ValueError as exc:
                    raise TestBatchError(
                        f"official sample row {line} has an invalid UnixTimeMillis"
                    ) from exc
                if timestamp < 0:
                    raise TestBatchError(f"official sample row {line} has a negative timestamp")
                coordinates: dict[str, float] = {}
                for field, lower, upper in (
                    ("LatitudeDegrees", -90.0, 90.0),
                    ("LongitudeDegrees", -180.0, 180.0),
                ):
                    token = raw.get(field) or ""
                    if token != token.strip() or not token:
                        raise TestBatchError(
                            f"official sample row {line} has a missing {field} fallback coordinate"
                        )
                    try:
                        value = float(token)
                    except ValueError as exc:
                        raise TestBatchError(
                            f"official sample row {line} has invalid {field}"
                        ) from exc
                    if not math.isfinite(value) or not lower <= value <= upper:
                        raise TestBatchError(
                            f"official sample row {line} has invalid {field} fallback coordinate"
                        )
                    coordinates[field] = value
                key = (trip_id, timestamp)
                if key in seen:
                    raise TestBatchError(f"official sample row {line} duplicates key {key!r}")
                seen.add(key)
                rows.append(
                    {
                        "trip_id": trip_id,
                        "timestamp": timestamp,
                        "timestamp_text": timestamp_token,
                        "latitude": coordinates["LatitudeDegrees"],
                        "longitude": coordinates["LongitudeDegrees"],
                        "source_line": line,
                    }
                )
    except OSError as exc:
        raise TestBatchError(f"failed to read official sample submission: {path}") from exc
    if not rows:
        raise TestBatchError("official sample submission contains no rows")
    return {
        "schema_version": OFFICIAL_SAMPLE_SCHEMA,
        "fields": list(OFFICIAL_SAMPLE_FIELDS),
        "rows": rows,
        "key_count": len(rows),
        "duplicate_keys": 0,
        "coordinate_validation": {
            "rows_with_two_finite_in_range_coordinates": len(rows),
            "blank_coordinate_fields": 0,
            "nonfinite_or_invalid_coordinate_fields": 0,
            "out_of_range_coordinate_fields": 0,
        },
        "artifact": _artifact(path),
    }


def _reconciliation_coordinate(
    coordinate: tuple[float, float],
    label: str,
) -> tuple[float, float]:
    try:
        latitude, longitude = float(coordinate[0]), float(coordinate[1])
    except (IndexError, TypeError, ValueError) as exc:
        raise TestBatchError(f"{label} coordinate is invalid") from exc
    if not all(math.isfinite(value) for value in (latitude, longitude)):
        raise TestBatchError(f"{label} coordinate is non-finite")
    if not -90.0 <= latitude <= 90.0 or not -180.0 <= longitude <= 180.0:
        raise TestBatchError(f"{label} coordinate is outside range")
    return latitude, longitude


def _reconcile_official_sample_rows(
    sample_rows: list[dict[str, Any]],
    provisional: dict[tuple[str, int], tuple[float, float]],
) -> tuple[bytes, dict[str, Any]]:
    """Reconcile exact official keys without nearest matching or key synthesis."""

    output_rows: list[tuple[str, str, str, str]] = []
    seen: set[tuple[str, int]] = set()
    route_analysis: dict[str, dict[str, Any]] = {}
    exact_count = 0
    sample_fallback_count = 0
    exact_offsets_ms: list[int] = []
    for row in sample_rows:
        trip_id = row.get("trip_id")
        timestamp = row.get("timestamp")
        if not isinstance(trip_id, str) or isinstance(timestamp, bool) or not isinstance(timestamp, int):
            raise TestBatchError("official sample row has an invalid reconciliation key")
        key = (trip_id, timestamp)
        if key in seen:
            raise TestBatchError(f"official sample contains duplicate reconciliation key: {key!r}")
        seen.add(key)
        route = trip_id.split("/", 1)[0]
        route_stats = route_analysis.setdefault(
            route,
            {
                "sample_rows": 0,
                "exact": 0,
                "sample_fallback": 0,
                "provisional_rows": 0,
                "drop_extra": 0,
                "sample_first_timestamp": timestamp,
                "sample_last_timestamp": timestamp,
            },
        )
        route_stats["sample_rows"] += 1
        route_stats["sample_first_timestamp"] = min(route_stats["sample_first_timestamp"], timestamp)
        route_stats["sample_last_timestamp"] = max(route_stats["sample_last_timestamp"], timestamp)
        if key in provisional:
            latitude, longitude = _reconciliation_coordinate(
                provisional[key], f"exact provisional {trip_id}/{timestamp}"
            )
            exact_count += 1
            route_stats["exact"] += 1
            exact_offsets_ms.append(0)
            source = "exact_provisional"
        else:
            try:
                latitude, longitude = _reconciliation_coordinate(
                    (float(row["latitude"]), float(row["longitude"])),
                    f"official sample fallback {trip_id}/{timestamp}",
                )
            except (KeyError, TypeError, ValueError) as exc:
                raise TestBatchError(
                    f"official sample fallback coordinate is invalid: {trip_id}/{timestamp}"
                ) from exc
            sample_fallback_count += 1
            route_stats["sample_fallback"] += 1
            source = "official_sample_baseline_fallback"
        timestamp_text = row.get("timestamp_text", str(timestamp))
        if not isinstance(timestamp_text, str) or timestamp_text != timestamp_text.strip():
            raise TestBatchError(f"official sample timestamp text is invalid: {trip_id}/{timestamp}")
        output_rows.append(
            (trip_id, timestamp_text, f"{latitude:.12f}", f"{longitude:.12f}")
        )
        route_stats.setdefault("source_values", {"exact_provisional": 0, "official_sample_baseline_fallback": 0})
        route_stats["source_values"][source] += 1

    provisional_by_route: dict[str, int] = {}
    for phone, _timestamp in provisional:
        route = phone.split("/", 1)[0]
        provisional_by_route[route] = provisional_by_route.get(route, 0) + 1
    for route, count in provisional_by_route.items():
        route_stats = route_analysis.setdefault(
            route,
            {
                "sample_rows": 0,
                "exact": 0,
                "sample_fallback": 0,
                "provisional_rows": 0,
                "drop_extra": 0,
                "sample_first_timestamp": None,
                "sample_last_timestamp": None,
                "source_values": {"exact_provisional": 0, "official_sample_baseline_fallback": 0},
            },
        )
        route_stats["provisional_rows"] = count
    provisional_extra = set(provisional) - seen
    extra_by_route: dict[str, int] = {}
    for phone, _timestamp in provisional_extra:
        route = phone.split("/", 1)[0]
        extra_by_route[route] = extra_by_route.get(route, 0) + 1
    for route, count in extra_by_route.items():
        route_analysis[route]["drop_extra"] = count
    for route_stats in route_analysis.values():
        route_stats.pop("source_values", None)

    buffer = io.StringIO(newline="")
    writer = csv.writer(buffer, lineterminator="\n")
    writer.writerow(OFFICIAL_SAMPLE_FIELDS)
    writer.writerows(output_rows)
    return buffer.getvalue().encode("utf-8"), {
        "source_counts": {
            "exact": exact_count,
            "sample_fallback": sample_fallback_count,
            "drop_extra": len(provisional_extra),
        },
        "sample_key_count": len(seen),
        "provisional_key_count": len(provisional),
        "final_key_count": len(output_rows),
        "sample_duplicate_keys": 0,
        "provisional_duplicate_keys": 0,
        "provisional_extra_keys_dropped": len(provisional_extra),
        "route_analysis": {
            route: route_analysis[route] for route in sorted(route_analysis)
        },
        "offset_analysis": {
            "exact_timestamp_offset_ms": {
                "count": len(exact_offsets_ms),
                "min": min(exact_offsets_ms) if exact_offsets_ms else None,
                "max": max(exact_offsets_ms) if exact_offsets_ms else None,
                "unique_values": sorted(set(exact_offsets_ms)),
            },
            "sample_fallback_timestamp_offset_ms": None,
            "dropped_extra_timestamp_offset_ms": None,
            "nearest_matching": False,
            "remap": False,
            "interpolation": False,
        },
        "contract": {
            "official_sample_is_sole_authority": True,
            "output_header": list(OFFICIAL_SAMPLE_FIELDS),
            "sample_order_preserved": True,
            "exact_provisional_coordinates_preferred": True,
            "sample_coordinates_used_only_for_exact_missing_keys": True,
            "extra_provisional_keys_dropped": True,
            "key_synthesis": False,
            "nearest_or_remap": False,
            "interpolation": False,
            "algorithm_rerun": False,
        },
    }


def _verify_reconciled_output(
    path: Path,
    sample_rows: list[dict[str, Any]],
) -> dict[str, Any]:
    """Independently re-read the published reconciliation CSV and verify keys."""

    if not path.is_file():
        raise TestBatchError(f"missing reconciled submission: {path}")
    seen: set[tuple[str, str]] = set()
    output_rows = 0
    nonfinite_coordinates = 0
    key_order_byte_equivalent = True
    try:
        with path.open(encoding="utf-8-sig", newline="") as handle:
            reader = csv.DictReader(handle)
            if tuple(reader.fieldnames or ()) != OFFICIAL_SAMPLE_FIELDS:
                raise TestBatchError("reconciled submission header is not official sample exact")
            for expected, raw in zip(sample_rows, reader):
                output_rows += 1
                if None in raw:
                    raise TestBatchError(f"reconciled submission row {output_rows + 1} has extra columns")
                trip_id = raw.get("tripId") or ""
                timestamp_text = raw.get("UnixTimeMillis") or ""
                expected_key = (str(expected["trip_id"]), str(expected["timestamp_text"]))
                if (trip_id, timestamp_text) != expected_key:
                    key_order_byte_equivalent = False
                    raise TestBatchError(
                        f"reconciled submission key/order differs at row {output_rows + 1}"
                    )
                if (trip_id, timestamp_text) in seen:
                    raise TestBatchError(f"reconciled submission duplicates key: {trip_id}/{timestamp_text}")
                seen.add((trip_id, timestamp_text))
                try:
                    latitude = float((raw.get("LatitudeDegrees") or "").strip())
                    longitude = float((raw.get("LongitudeDegrees") or "").strip())
                except ValueError as exc:
                    raise TestBatchError(
                        f"reconciled submission row {output_rows + 1} has invalid coordinate"
                    ) from exc
                if not all(math.isfinite(value) for value in (latitude, longitude)):
                    nonfinite_coordinates += 1
                    raise TestBatchError(
                        f"reconciled submission row {output_rows + 1} has non-finite coordinate"
                    )
                if not -90.0 <= latitude <= 90.0 or not -180.0 <= longitude <= 180.0:
                    raise TestBatchError(
                        f"reconciled submission row {output_rows + 1} has out-of-range coordinate"
                    )
            if next(reader, None) is not None:
                raise TestBatchError("reconciled submission has extra rows")
    except OSError as exc:
        raise TestBatchError(f"failed to read reconciled submission: {path}") from exc
    if output_rows != len(sample_rows):
        raise TestBatchError(
            f"reconciled submission row count differs: {output_rows} != {len(sample_rows)}"
        )
    expected_keys = {
        (str(row["trip_id"]), str(row["timestamp_text"])) for row in sample_rows
    }
    missing = expected_keys - seen
    extra = seen - expected_keys
    if missing or extra:
        raise TestBatchError(
            f"reconciled submission key-set differs: missing={len(missing)}, extra={len(extra)}"
        )
    return {
        "header_exact": True,
        "row_count": output_rows,
        "duplicate_keys": 0,
        "missing_keys": len(missing),
        "extra_keys": len(extra),
        "nonfinite_coordinates": nonfinite_coordinates,
        "key_order_byte_equivalent": key_order_byte_equivalent,
    }


def _load_sealed_derived_provisional(
    output_dir: Path,
    archive_path: Path,
    inventory_path: Path,
) -> dict[str, Any]:
    """Load and hash-check a completed truth-free derived run for reconciliation."""

    run_manifest_path = output_dir / "test_batch_derived_run_manifest.json"
    run_manifest = _load_json(run_manifest_path, "derived test batch manifest")
    if run_manifest.get("schema_version") != DERIVED_RUN_MANIFEST_SCHEMA:
        raise TestBatchError("derived test batch manifest schema is invalid")
    if run_manifest.get("status") != "completed-truth-free-derived-submission":
        raise TestBatchError("derived test batch is not a completed truth-free run")
    if run_manifest.get("official_sample_verified") is not False:
        raise TestBatchError("derived run manifest has an invalid sample verification state")
    truth_policy = run_manifest.get("truth_policy", {})
    if truth_policy.get("truth_open_count") != 0 or truth_policy.get("truth_materialized_count") != 0:
        raise TestBatchError("derived run manifest records truth access")
    if run_manifest.get("archive", {}).get("sha256") != _sha256(archive_path):
        raise TestBatchError("derived run archive hash differs")
    if run_manifest.get("inventory", {}).get("sha256") != _sha256(inventory_path):
        raise TestBatchError("derived run inventory hash differs")
    key_entry = run_manifest.get("provisional_key_manifest")
    submission_entry = run_manifest.get("provisional_submission")
    if not isinstance(key_entry, dict) or not isinstance(submission_entry, dict):
        raise TestBatchError("derived run manifest lacks provisional artifacts")
    key_path = Path(str(key_entry.get("path", "")))
    submission_path = Path(str(submission_entry.get("path", "")))
    if key_entry.get("sha256") != _sha256(key_path):
        raise TestBatchError("derived provisional key manifest hash differs")
    if submission_entry.get("sha256") != _sha256(submission_path):
        raise TestBatchError("derived provisional submission hash differs")
    key_manifest = _load_json(key_path, "provisional key manifest")
    if key_manifest.get("official_sample_verified") is not False:
        raise TestBatchError("provisional key manifest is not unverified")
    if key_manifest.get("cannot_submit_to_kaggle") is not True:
        raise TestBatchError("provisional key manifest has an invalid submission state")
    return {
        "run_manifest_path": run_manifest_path,
        "run_manifest": run_manifest,
        "key_manifest_path": key_path,
        "key_manifest": key_manifest,
        "submission_path": submission_path,
        "submission_entry": submission_entry,
        "key_entry": key_entry,
    }


def _reconcile_derived_with_sample_v2(
    output_dir: Path,
    sample_path: Path,
    inventory_path: Path,
    archive_path: Path,
    inventory: dict[str, Any],
    profile_path: Path,
) -> int:
    """Publish an official-schema CSV by exact sample-key reconciliation only."""

    output_path = output_dir / "submission.csv"
    submission_manifest_path = output_dir / "submission.manifest.json"
    run_manifest_path = output_dir / "test_batch_reconciliation_v2_manifest.json"
    if output_path.exists() or submission_manifest_path.exists() or run_manifest_path.exists():
        raise TestBatchError("reconciliation output already exists; refusing overwrite")
    sample = _read_official_sample_submission(sample_path)
    provisional_state = _load_sealed_derived_provisional(
        output_dir, archive_path, inventory_path
    )
    provisional = _read_generated_submission(provisional_state["submission_path"])
    records = inventory.get("test", {}).get("records")
    if not isinstance(records, list) or not records:
        raise TestBatchError("test inventory has no records for reconciliation")
    dataset_ids = {
        str(record.get("dataset_id"))
        for record in records
        if isinstance(record, dict) and record.get("dataset_id")
    }
    for row in sample["rows"]:
        if row["trip_id"] not in dataset_ids:
            raise TestBatchError(
                f"official sample tripId is not an exact inventory dataset id: {row['trip_id']}"
            )
    output_bytes, reconciliation = _reconcile_official_sample_rows(
        sample["rows"], provisional
    )
    if reconciliation["source_counts"] != {
        "exact": 71912,
        "sample_fallback": 24,
        "drop_extra": 98,
    }:
        raise TestBatchError(
            "reconciliation source counts differ from the sealed expected counts"
        )
    _atomic_write(output_path, output_bytes)
    verification = _verify_reconciled_output(output_path, sample["rows"])
    if verification["missing_keys"] or verification["extra_keys"] or verification["duplicate_keys"]:
        raise TestBatchError("reconciled submission failed independent key validation")
    if verification["nonfinite_coordinates"] or not verification["key_order_byte_equivalent"]:
        raise TestBatchError("reconciled submission failed independent coordinate/order validation")
    sample_manifest_path = sample_path.with_name("sample_submission.manifest.json")
    sample_provenance = (
        _artifact(sample_manifest_path) if sample_manifest_path.is_file() else None
    )
    submission_manifest = {
        "schema_version": RECONCILIATION_MANIFEST_SCHEMA,
        "status": "official-sample-reconciled-truth-free",
        "truth_free": True,
        "truth_used": False,
        "official_sample_verified": True,
        "algorithm_recomputed": False,
        "candidate_or_numeric_parameter_changed": False,
        "token_value_recorded": False,
        "archive": {"path": str(archive_path), "sha256": _sha256(archive_path)},
        "inventory": {"path": str(inventory_path), "sha256": _sha256(inventory_path)},
        "profile": _artifact(profile_path),
        "algorithm_core_hash": provisional_state["run_manifest"].get("algorithm_core_hash"),
        "algorithm_parameter_hash": provisional_state["run_manifest"].get("algorithm_parameter_hash"),
        "release_binaries": provisional_state["run_manifest"].get("release_binaries"),
        "official_sample": sample["artifact"],
        "official_sample_provenance_manifest": sample_provenance,
        "provisional_run_manifest": _artifact(provisional_state["run_manifest_path"]),
        "provisional_key_manifest": _artifact(provisional_state["key_manifest_path"]),
        "provisional_submission": {
            **dict(provisional_state["submission_entry"]),
            "path": str(provisional_state["submission_path"]),
        },
        "reconciliation": reconciliation,
        "final_verification": verification,
        "submission": _artifact(output_path),
        "atomic_publish": True,
        "truth_policy": {
            "truth_open_count": 0,
            "truth_materialized_count": 0,
            "ground_truth_members_read": False,
        },
        "external_kaggle_submission": False,
    }
    _atomic_json(submission_manifest_path, submission_manifest)
    run_manifest = {
        "schema_version": "smartphone-r5-wls-test-batch-reconciliation-run-manifest.v2",
        "status": "completed-truth-free-official-sample-reconciliation",
        "official_sample": sample["artifact"],
        "official_sample_provenance_manifest": sample_provenance,
        "provisional_run_manifest": _artifact(provisional_state["run_manifest_path"]),
        "provisional_submission": _artifact(provisional_state["submission_path"]),
        "submission": _artifact(output_path),
        "submission_manifest": _artifact(submission_manifest_path),
        "source_counts": reconciliation["source_counts"],
        "route_analysis": reconciliation["route_analysis"],
        "offset_analysis": reconciliation["offset_analysis"],
        "final_verification": verification,
        "algorithm_rerun": False,
        "truth_policy": {
            "truth_open_count": 0,
            "truth_materialized_count": 0,
            "ground_truth_members_read": False,
        },
        "external_kaggle_submission": False,
    }
    _atomic_json(run_manifest_path, run_manifest)
    print(f"Smartphone official sample reconciliation v2 complete: {output_path}")
    return 0


def _promote_derived_with_sample(
    output_dir: Path,
    sample_path: Path,
    inventory_path: Path,
    archive_path: Path,
    inventory: dict[str, Any],
) -> int:
    """Reorder a sealed provisional CSV using a later official sample only."""

    run_manifest_path = output_dir / "test_batch_derived_run_manifest.json"
    run_manifest = _load_json(run_manifest_path, "derived test batch manifest")
    if run_manifest.get("schema_version") != DERIVED_RUN_MANIFEST_SCHEMA:
        raise TestBatchError("derived test batch manifest schema is invalid")
    if run_manifest.get("status") != "completed-truth-free-derived-submission":
        raise TestBatchError("derived test batch is not a completed truth-free run")
    if run_manifest.get("official_sample_verified") is not False:
        raise TestBatchError("derived run manifest has an invalid sample verification state")
    if run_manifest.get("truth_policy", {}).get("truth_open_count") != 0:
        raise TestBatchError("derived run manifest records truth access")
    if run_manifest.get("archive", {}).get("sha256") != _sha256(archive_path):
        raise TestBatchError("derived run archive hash differs")
    if run_manifest.get("inventory", {}).get("sha256") != _sha256(inventory_path):
        raise TestBatchError("derived run inventory hash differs")
    key_entry = run_manifest.get("provisional_key_manifest")
    submission_entry = run_manifest.get("provisional_submission")
    if not isinstance(key_entry, dict) or not isinstance(submission_entry, dict):
        raise TestBatchError("derived run manifest lacks provisional artifacts")
    key_path = Path(str(key_entry.get("path", "")))
    submission_path = Path(str(submission_entry.get("path", "")))
    if key_entry.get("sha256") != _sha256(key_path) or submission_entry.get("sha256") != _sha256(submission_path):
        raise TestBatchError("derived provisional artifact hash differs")
    key_manifest = _load_json(key_path, "provisional key manifest")
    if key_manifest.get("official_sample_verified") is not False or key_manifest.get("cannot_submit_to_kaggle") is not True:
        raise TestBatchError("provisional key manifest is not unverified")
    sample = _read_sample_submission(sample_path)
    provisional = _read_generated_submission(submission_path)
    records = inventory.get("test", {}).get("records")
    if not isinstance(records, list) or not records:
        raise TestBatchError("test inventory has no records for promotion")
    sample_rows: list[tuple[str, int, tuple[float, float]]] = []
    sample_keys: set[tuple[str, int]] = set()
    for raw in sample["rows"]:
        record = _resolve_sample_dataset(raw["phone"], records)
        canonical = _canonical_provisional_phone(str(record["dataset_id"]))
        key = (canonical, raw["timestamp"])
        if key in sample_keys:
            raise TestBatchError(f"official sample aliases duplicate key: {key!r}")
        coordinate = provisional.get(key)
        if coordinate is None:
            raise TestBatchError(f"official sample key is absent from provisional output: {key!r}")
        sample_keys.add(key)
        sample_rows.append((raw["phone"], raw["timestamp"], coordinate))
    if sample_keys != set(provisional):
        missing = sorted(set(provisional) - sample_keys)[:3]
        extra = sorted(sample_keys - set(provisional))[:3]
        raise TestBatchError(f"official sample/provisional key-set mismatch; missing={missing}, extra={extra}")
    buffer = io.StringIO(newline="")
    writer = csv.writer(buffer, lineterminator="\n")
    writer.writerow(SAMPLE_FIELDS)
    for phone, timestamp, (latitude, longitude) in sample_rows:
        writer.writerow((phone, timestamp, f"{latitude:.12f}", f"{longitude:.12f}"))
    output_path = output_dir / "submission.csv"
    _atomic_write(output_path, buffer.getvalue().encode("utf-8"))
    submission_manifest_path = output_dir / "submission.manifest.json"
    submission_manifest = {
        "schema_version": "smartphone-r5-wls-test-batch-promotion-manifest.v1",
        "status": "promoted-by-official-sample-reorder-only",
        "truth_free": True,
        "truth_used": False,
        "official_sample_verified": True,
        "algorithm_recomputed": False,
        "derived_source_submission": dict(submission_entry),
        "provisional_submission": {
            "path": str(submission_path),
            "sha256_before_overwrite": submission_entry["sha256"],
        },
        "official_sample": sample["artifact"],
        "submission_columns": list(SAMPLE_FIELDS),
        "key_count": len(sample_rows),
        "exact_sample_order": True,
        "duplicate_keys": 0,
        "missing_keys": 0,
        "extra_keys": 0,
        "coordinate_source": "unchanged provisional coordinates; sample used only for strict key-set/order",
        "truth_open_count": 0,
    }
    # Replace the self-referential source artifact with the final output hash.
    submission_manifest["submission"] = _artifact(output_path)
    _atomic_json(submission_manifest_path, submission_manifest)
    promotion_manifest = {
        "schema_version": "smartphone-r5-wls-test-batch-derived-promotion-run-manifest.v1",
        "status": "promoted-by-official-sample-reorder-only",
        "derived_run_manifest": _artifact(run_manifest_path),
        "official_sample": sample["artifact"],
        "submission": _artifact(output_path),
        "submission_manifest": _artifact(submission_manifest_path),
        "key_count": len(sample_rows),
        "truth_policy": {"truth_open_count": 0, "truth_materialized_count": 0, "ground_truth_members_read": False},
        "algorithm_recomputed": False,
        "test_payload_materialized": False,
    }
    _atomic_json(output_dir / "test_batch_derived_promotion_manifest.json", promotion_manifest)
    print(f"Smartphone derived test batch promoted by sample reorder: {output_path}")
    return 0


def _find_archive_sample_member(inventory: dict[str, Any]) -> dict[str, Any] | None:
    candidates = inventory.get("sample_submission", {}).get("archive_candidates", [])
    if not isinstance(candidates, list) or len(candidates) != 1:
        return None
    return candidates[0] if isinstance(candidates[0], dict) else None


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        prog=os.environ.get("GNSS_CLI_NAME", "gnss smartphone-wls-test-batch")
    )
    parser.add_argument("--archive", type=Path, default=DEFAULT_ARCHIVE)
    parser.add_argument("--inventory", type=Path, default=DEFAULT_INVENTORY)
    parser.add_argument("--sample-submission", type=Path, default=DEFAULT_SAMPLE)
    parser.add_argument("--profile", type=Path, default=DEFAULT_PROFILE)
    parser.add_argument("--freeze-record", type=Path, default=DEFAULT_FREEZE)
    parser.add_argument("--freeze-manifest", type=Path, default=DEFAULT_FREEZE_MANIFEST)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--cache-dir", type=Path)
    parser.add_argument("--native-fallback-root", type=Path)
    parser.add_argument("--native-fallback-manifest", type=Path)
    parser.add_argument(
        "--completeness-fallback-freeze",
        type=Path,
        help="explicit truth-free freeze authorizing test completeness fallback v1",
    )
    parser.add_argument(
        "--completeness-fallback-manifest",
        type=Path,
        help="manifest paired with --completeness-fallback-freeze",
    )
    parser.add_argument(
        "--enable-completeness-fallback-v1",
        action="store_true",
        help="opt in to the separately frozen completeness fallback contract",
    )
    parser.add_argument(
        "--edge-completeness-fallback-freeze",
        type=Path,
        help="explicit truth-free freeze authorizing edge constant-hold fallback v2",
    )
    parser.add_argument(
        "--edge-completeness-fallback-manifest",
        type=Path,
        help="manifest paired with --edge-completeness-fallback-freeze",
    )
    parser.add_argument(
        "--enable-edge-completeness-fallback-v2",
        action="store_true",
        help="opt in to the separately frozen edge completeness fallback contract",
    )
    parser.add_argument(
        "--edge-completeness-fallback-v2-1-freeze",
        type=Path,
        help="explicit truth-free freeze authorizing one-second edge constant-hold fallback v2.1",
    )
    parser.add_argument(
        "--edge-completeness-fallback-v2-1-manifest",
        type=Path,
        help="manifest paired with --edge-completeness-fallback-v2-1-freeze",
    )
    parser.add_argument(
        "--enable-edge-completeness-fallback-v2-1",
        action="store_true",
        help="opt in to the separately frozen one-second edge completeness fallback",
    )
    parser.add_argument(
        "--resume-derived-run",
        type=Path,
        help="reuse a sealed truth-free derived run and process only its failures",
    )
    parser.add_argument(
        "--allow-derived-unverified-key-order",
        action="store_true",
        help=(
            "explicitly permit a provisional non-submittable key order derived from "
            "validated device_gnss epochs when the official sample is unavailable"
        ),
    )
    parser.add_argument(
        "--promote-derived-with-sample",
        action="store_true",
        help=(
            "reorder a completed provisional CSV with a later official sample; "
            "no test payload or algorithm is rerun"
        ),
    )
    parser.add_argument(
        "--reconcile-derived-with-sample-v2",
        action="store_true",
        help=(
            "reconcile a sealed provisional CSV to the official tripId sample; "
            "exact keys win, missing sample keys use finite sample coordinates, "
            "and provisional extras are dropped without rerunning positioning"
        ),
    )
    parser.add_argument("--inventory-only", action="store_true")
    return parser.parse_args(argv)


def run(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    started = time.perf_counter()
    rss_before = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss
    output_dir = args.output_dir
    try:
        inventory = _write_or_verify_inventory(args.inventory, args.archive)
        if args.inventory_only:
            print(f"Smartphone test central inventory complete: {args.inventory}")
            return 0
        output_dir.mkdir(parents=True, exist_ok=True)
        archive_hash = _sha256(args.archive)
        if args.promote_derived_with_sample and args.reconcile_derived_with_sample_v2:
            raise TestBatchError(
                "--promote-derived-with-sample and --reconcile-derived-with-sample-v2 are mutually exclusive"
            )
        if args.reconcile_derived_with_sample_v2:
            return _reconcile_derived_with_sample_v2(
                output_dir,
                args.sample_submission,
                args.inventory,
                args.archive,
                inventory,
                args.profile,
            )
        if args.promote_derived_with_sample:
            if not args.sample_submission.is_file():
                raise TestBatchError(
                    "promotion requires a local official sample_submission.csv; "
                    "no archive payload or test truth is materialized"
                )
            return _promote_derived_with_sample(
                output_dir,
                args.sample_submission,
                args.inventory,
                args.archive,
                inventory,
            )
        enabled_fallbacks = sum(
            int(value)
            for value in (
                args.enable_completeness_fallback_v1,
                args.enable_edge_completeness_fallback_v2,
                args.enable_edge_completeness_fallback_v2_1,
            )
        )
        if enabled_fallbacks > 1:
            raise TestBatchError(
                "completeness fallback v1, edge fallback v2, and edge fallback v2.1 are mutually exclusive"
            )
        if args.enable_completeness_fallback_v1:
            if args.completeness_fallback_freeze is None or args.completeness_fallback_manifest is None:
                raise TestBatchError(
                    "completeness fallback v1 requires both freeze record and manifest"
                )
        elif args.completeness_fallback_freeze is not None or args.completeness_fallback_manifest is not None:
            raise TestBatchError(
                "completeness fallback freeze paths require --enable-completeness-fallback-v1"
            )
        if args.enable_edge_completeness_fallback_v2:
            if (
                args.edge_completeness_fallback_freeze is None
                or args.edge_completeness_fallback_manifest is None
            ):
                raise TestBatchError(
                    "edge completeness fallback v2 requires both freeze record and manifest"
                )
        elif (
            args.edge_completeness_fallback_freeze is not None
            or args.edge_completeness_fallback_manifest is not None
        ):
            raise TestBatchError(
                "edge completeness fallback paths require --enable-edge-completeness-fallback-v2"
            )
        if args.enable_edge_completeness_fallback_v2_1:
            if (
                args.edge_completeness_fallback_v2_1_freeze is None
                or args.edge_completeness_fallback_v2_1_manifest is None
            ):
                raise TestBatchError(
                    "edge completeness fallback v2.1 requires both freeze record and manifest"
                )
        elif (
            args.edge_completeness_fallback_v2_1_freeze is not None
            or args.edge_completeness_fallback_v2_1_manifest is not None
        ):
            raise TestBatchError(
                "edge completeness fallback v2.1 paths require --enable-edge-completeness-fallback-v2-1"
            )
        sample_member = _find_archive_sample_member(inventory)
        # A local sample is not a test route member and may be inspected during
        # preflight.  Persist its schema/key count in the central inventory so
        # the authorization pins the exact authoritative order source.
        sample_preview: dict[str, Any] | None = None
        if args.sample_submission.is_file():
            sample_preview = _read_sample_submission(args.sample_submission)
            inventory["sample_submission"].update(
                {
                    "local_path": str(args.sample_submission),
                    "local_artifact": sample_preview["artifact"],
                    "schema_version": sample_preview["schema_version"],
                    "fields": sample_preview["fields"],
                    "key_count": sample_preview["key_count"],
                }
            )
            _atomic_json(args.inventory, inventory)
        derived_mode = sample_preview is None and sample_member is None
        if derived_mode and not args.allow_derived_unverified_key_order:
            raise TestBatchError(
                "sample submission is missing; pass --allow-derived-unverified-key-order "
                "for an explicitly provisional, non-submittable key order"
            )
        inventory_hash = _sha256(args.inventory)
        authorization_path, authorization_manifest_path, authorization = _create_test_authorization(
            output_dir,
            args.archive,
            args.inventory,
            inventory,
            args.profile,
            args.freeze_record,
            args.freeze_manifest,
            sample_member=sample_member,
            derived_key_order=derived_mode,
            completeness_fallback_freeze_path=(
                args.completeness_fallback_freeze
                if args.enable_completeness_fallback_v1
                else None
            ),
            completeness_fallback_manifest_path=(
                args.completeness_fallback_manifest
                if args.enable_completeness_fallback_v1
                else None
            ),
            edge_completeness_fallback_freeze_path=(
                args.edge_completeness_fallback_freeze
                if args.enable_edge_completeness_fallback_v2
                else None
            ),
            edge_completeness_fallback_manifest_path=(
                args.edge_completeness_fallback_manifest
                if args.enable_edge_completeness_fallback_v2
                else None
            ),
            edge_completeness_fallback_v2_1_freeze_path=(
                args.edge_completeness_fallback_v2_1_freeze
                if args.enable_edge_completeness_fallback_v2_1
                else None
            ),
            edge_completeness_fallback_v2_1_manifest_path=(
                args.edge_completeness_fallback_v2_1_manifest
                if args.enable_edge_completeness_fallback_v2_1
                else None
            ),
        )
        sample_path = args.sample_submission
        sample_materialized: dict[str, Any] | None = None
        if not derived_mode and not sample_path.is_file() and sample_member is not None:
            sample_path = output_dir / "inputs" / "sample_submission.csv"
            sample_materialized = _materialize_member(
                args.archive,
                sample_member["name"],
                sample_path,
                sample_member,
            )
        sample = (
            None
            if derived_mode
            else sample_preview
            if sample_preview is not None and sample_path == args.sample_submission
            else _read_sample_submission(sample_path)
        )
        records = inventory.get("test", {}).get("records")
        if not isinstance(records, list) or not records:
            raise TestBatchError("test inventory has no records")
        grouped: dict[str, list[dict[str, Any]]] = {}
        for record in records:
            if not isinstance(record, dict):
                raise TestBatchError("test inventory has an invalid record")
            grouped.setdefault(str(record["route"]), []).append(record)
        cache_root = args.cache_dir or output_dir / "cache"
        cache_root.mkdir(parents=True, exist_ok=True)
        route_results: list[dict[str, Any]] = []
        route_failures: list[dict[str, Any]] = []
        resumed_results: dict[str, dict[str, Any]] = {}
        resume_audit: dict[str, Any] | None = None
        if args.resume_derived_run is not None:
            if not (
                args.enable_completeness_fallback_v1
                or args.enable_edge_completeness_fallback_v2
                or args.enable_edge_completeness_fallback_v2_1
            ):
                raise TestBatchError(
                    "resuming the prior derived run requires an explicit completeness fallback freeze"
                )
            resumed_results, resume_audit = _load_resumed_success_routes(
                args.resume_derived_run,
                archive_hash=archive_hash,
                inventory_hash=inventory_hash,
                records=records,
                expected_completed_count=(
                    39
                    if (
                        args.enable_edge_completeness_fallback_v2
                        or args.enable_edge_completeness_fallback_v2_1
                    )
                    else 37
                ),
                expected_failed_routes=(
                    ("2022-10-06-20-46-us-ca-sjc-r",)
                    if (
                        args.enable_edge_completeness_fallback_v2
                        or args.enable_edge_completeness_fallback_v2_1
                    )
                    else ()
                ),
            )
        for route in sorted(grouped):
            if route in resumed_results:
                route_results.append(resumed_results[route])
                continue
            try:
                route_result = _write_multi_phone_route(
                    args.archive,
                    grouped[route],
                    output_dir,
                    cache_root,
                    authorization_path,
                    authorization_manifest_path,
                    archive_hash,
                    inventory_hash,
                    args.native_fallback_root,
                    args.native_fallback_manifest,
                    (
                        args.enable_completeness_fallback_v1
                        or args.enable_edge_completeness_fallback_v2
                        or args.enable_edge_completeness_fallback_v2_1
                    ),
                    (
                        args.enable_edge_completeness_fallback_v2
                        or args.enable_edge_completeness_fallback_v2_1
                    ),
                    (
                        EDGE_COMPLETENESS_FALLBACK_V2_1_MAX_GAP_MS
                        if args.enable_edge_completeness_fallback_v2_1
                        else FALLBACK_INTERPOLATION_MAX_GAP_MS
                    ),
                )
                route_manifest_payload = _load_json(
                    route_result["route_manifest_path"], "test route manifest"
                )
                route_manifest_payload["provenance"] = {
                    "archive_sha256": archive_hash,
                    "inventory_sha256": inventory_hash,
                    "authorization_sha256": _sha256(authorization_path),
                    "authorization_manifest_sha256": _sha256(authorization_manifest_path),
                    "freeze_record_sha256": authorization["upstream_v1_4_freeze"]["record"]["sha256"],
                    "freeze_manifest_sha256": authorization["upstream_v1_4_freeze"]["manifest"]["sha256"],
                    "algorithm_core_hash": authorization["algorithm_core_hash"],
                }
                if authorization.get("edge_completeness_fallback"):
                    route_manifest_payload["provenance"][
                        "edge_completeness_fallback_freeze_record_sha256"
                    ] = authorization["edge_completeness_fallback"]["record"]["sha256"]
                    route_manifest_payload["provenance"][
                        "edge_completeness_fallback_manifest_sha256"
                    ] = authorization["edge_completeness_fallback"]["manifest"]["sha256"]
                if authorization.get("edge_completeness_fallback_v2_1"):
                    route_manifest_payload["provenance"][
                        "edge_completeness_fallback_v2_1_freeze_record_sha256"
                    ] = authorization["edge_completeness_fallback_v2_1"]["record"]["sha256"]
                    route_manifest_payload["provenance"][
                        "edge_completeness_fallback_v2_1_manifest_sha256"
                    ] = authorization["edge_completeness_fallback_v2_1"]["manifest"]["sha256"]
                _atomic_json(route_result["route_manifest_path"], route_manifest_payload)
                route_result["route_manifest"] = _artifact(route_result["route_manifest_path"])
                route_results.append(route_result)
            except (TestBatchError, OSError, ValueError, KeyError, TypeError, wls.WlsPositionError, smoother.SmootherError) as exc:
                route_failures.append({"route": route, "error": str(exc), "truth_open_count": 0})
        if derived_mode:
            if route_failures:
                run_manifest = {
                    "schema_version": DERIVED_RUN_MANIFEST_SCHEMA,
                    "status": "failed-truth-free-derived-submission",
                    "official_sample_verified": False,
                    "derived_key_order_opt_in": True,
                    "archive": {"path": str(args.archive), "sha256": archive_hash},
                    "inventory": {"path": str(args.inventory), "sha256": inventory_hash},
                    "profile": _artifact(args.profile),
                    "authorization": {
                        "record": _artifact(authorization_path),
                        "manifest": _artifact(authorization_manifest_path),
                    },
                    "release_binaries": authorization.get("release_binaries"),
                    "freeze": authorization.get("upstream_v1_4_freeze"),
                    "completeness_fallback": authorization.get("completeness_fallback"),
                    "edge_completeness_fallback": authorization.get("edge_completeness_fallback"),
                    "edge_completeness_fallback_v2_1": authorization.get("edge_completeness_fallback_v2_1"),
                    "resume": resume_audit,
                    "sample_submission": {
                        "present": False,
                        "authoritative_key_order": False,
                        "derived_key_order": True,
                    },
                    "routes": {
                        "inventory_count": len(records),
                        "completed_count": len(route_results),
                        "failed_count": len(route_failures),
                        "resumed_count": sum(1 for result in route_results if result.get("resumed")),
                        "failures": route_failures,
                        "route_manifests": [result["route_manifest"] for result in route_results],
                    },
                    "truth_policy": {
                        "truth_open_count": 0,
                        "truth_materialized_count": 0,
                        "ground_truth_members_read": False,
                    },
                    "timing": {
                        "total_wall_s": time.perf_counter() - started,
                        "process_peak_rss_kb": max(
                            int(rss_before),
                            int(resource.getrusage(resource.RUSAGE_SELF).ru_maxrss),
                        ),
                        "process_rss_unit": "kilobytes from getrusage ru_maxrss",
                    },
                    "production_policy": {
                        "production_rtk_spp_default_changed": False,
                        "kaggle_external_submission": False,
                    },
                }
                _atomic_json(output_dir / "test_batch_derived_run_manifest.json", run_manifest)
                raise TestBatchError(
                    f"{len(route_failures)} test route(s) failed; provisional submission was not promoted"
                )
            key_manifest_path, derived_row_map, key_manifest = _derived_key_manifest(
                route_results,
                records,
                output_dir,
            )
            output_bytes, submission_contract = _derived_submission_bytes(
                key_manifest,
                derived_row_map,
            )
            provisional_submission_path = output_dir / "submission_derived_unverified.csv"
            _atomic_write(provisional_submission_path, output_bytes)
            provisional_submission_manifest_path = (
                output_dir / "submission_derived_unverified.manifest.json"
            )
            provisional_submission_manifest = {
                "schema_version": DERIVED_SUBMISSION_MANIFEST_SCHEMA,
                "status": "provisional-unverified-key-order",
                "truth_free": True,
                "truth_used": False,
                "official_sample_verified": False,
                "cannot_submit_to_kaggle": True,
                "key_manifest": _artifact(key_manifest_path),
                "submission": _artifact(provisional_submission_path),
                "submission_columns": list(SAMPLE_FIELDS),
                "contract": submission_contract,
                "truth_open_count": 0,
            }
            _atomic_json(
                provisional_submission_manifest_path,
                provisional_submission_manifest,
            )
            rss_after = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss
            run_manifest = {
                "schema_version": DERIVED_RUN_MANIFEST_SCHEMA,
                "status": "completed-truth-free-derived-submission",
                "official_sample_verified": False,
                "derived_key_order_opt_in": True,
                "cannot_submit_to_kaggle": True,
                "archive": {"path": str(args.archive), "sha256": archive_hash},
                "inventory": {"path": str(args.inventory), "sha256": inventory_hash},
                "profile": _artifact(args.profile),
                "authorization": {
                    "record": _artifact(authorization_path),
                    "manifest": _artifact(authorization_manifest_path),
                },
                "release_binaries": authorization.get("release_binaries"),
                "freeze": authorization.get("upstream_v1_4_freeze"),
                "completeness_fallback": authorization.get("completeness_fallback"),
                "edge_completeness_fallback": authorization.get("edge_completeness_fallback"),
                "edge_completeness_fallback_v2_1": authorization.get("edge_completeness_fallback_v2_1"),
                "resume": resume_audit,
                "sample_submission": {
                    "present": False,
                    "authoritative_key_order": False,
                    "derived_key_order": True,
                    "official_sample_required_before_submission": True,
                },
                "routes": {
                    "inventory_count": len(records),
                    "completed_count": len(route_results),
                    "failed_count": 0,
                    "resumed_count": sum(1 for result in route_results if result.get("resumed")),
                    "failures": [],
                    "route_manifests": [result["route_manifest"] for result in route_results],
                },
                "provisional_key_manifest": _artifact(key_manifest_path),
                "provisional_submission": _artifact(provisional_submission_path),
                "provisional_submission_manifest": _artifact(provisional_submission_manifest_path),
                "algorithm_core_hash": authorization["algorithm_core_hash"],
                "algorithm_parameter_hash": authorization["algorithm_parameter_hash"],
                "truth_policy": {
                    "truth_open_count": 0,
                    "truth_materialized_count": 0,
                    "ground_truth_members_read": False,
                },
                "timing": {
                    "total_wall_s": time.perf_counter() - started,
                    "process_peak_rss_kb": max(int(rss_before), int(rss_after)),
                    "process_rss_unit": "kilobytes from getrusage ru_maxrss",
                },
                "production_policy": {
                    "production_rtk_spp_default_changed": False,
                    "kaggle_external_submission": False,
                },
            }
            _atomic_json(output_dir / "test_batch_derived_run_manifest.json", run_manifest)
            print(
                "Smartphone WLS test batch provisional submission complete: "
                f"{provisional_submission_path}"
            )
            return 0
        row_map: dict[tuple[str, int], tuple[float, float]] = {}
        for result in route_results:
            row_map.update(result["row_map"])
        output_bytes, submission_contract = _ordered_submission_bytes(sample, records, row_map)
        submission_path = output_dir / "submission.csv"
        submission_manifest_path = output_dir / "submission.manifest.json"
        _atomic_write(submission_path, output_bytes)
        submission_manifest = {
            "schema_version": "smartphone-r5-wls-test-batch-submission-manifest.v1",
            "truth_free": True,
            "truth_used": False,
            "submission_columns": list(SAMPLE_FIELDS),
            "authoritative_sample": sample["artifact"],
            "submission": _artifact(submission_path),
            "contract": submission_contract,
            "truth_open_count": 0,
        }
        _atomic_json(submission_manifest_path, submission_manifest)
        rss_after = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss
        run_manifest = {
            "schema_version": RUN_MANIFEST_SCHEMA,
            "status": "completed-truth-free-test-submission",
            "archive": {"path": str(args.archive), "sha256": archive_hash},
            "inventory": {"path": str(args.inventory), "sha256": inventory_hash},
            "profile": _artifact(args.profile),
            "authorization": {"record": _artifact(authorization_path), "manifest": _artifact(authorization_manifest_path)},
            "release_binaries": authorization.get("release_binaries"),
            "freeze": authorization.get("upstream_v1_4_freeze"),
            "completeness_fallback": authorization.get("completeness_fallback"),
            "resume": resume_audit,
            "sample_submission": {
                "source": _artifact(sample_path),
                "materialized_from_archive": sample_materialized,
                "authoritative_key_order": True,
                "key_count": sample["key_count"],
            },
            "routes": {
                "inventory_count": len(records),
                "completed_count": len(route_results),
                "failed_count": len(route_failures),
                "failures": route_failures,
                "route_manifests": [result["route_manifest"] for result in route_results],
            },
            "submission": {
                "artifact": _artifact(submission_path),
                "manifest": _artifact(submission_manifest_path),
                "contract": submission_contract,
            },
            "truth_policy": {"truth_open_count": 0, "truth_materialized_count": 0, "ground_truth_members_read": False},
            "timing": {
                "total_wall_s": time.perf_counter() - started,
                "process_peak_rss_kb": max(int(rss_before), int(rss_after)),
                "process_rss_unit": "kilobytes from getrusage ru_maxrss",
            },
            "production_policy": {"production_rtk_spp_default_changed": False, "kaggle_external_submission": False},
        }
        run_manifest_path = output_dir / "test_batch_manifest.json"
        _atomic_json(run_manifest_path, run_manifest)
        if route_failures:
            raise TestBatchError(f"{len(route_failures)} test route(s) failed; submission was not promoted")
        print(f"Smartphone WLS test batch complete: {submission_path}")
        return 0
    except (TestBatchError, OSError, ValueError, KeyError, TypeError, json.JSONDecodeError, zipfile.BadZipFile, wls.WlsPositionError, smoother.SmootherError) as exc:
        output_dir.mkdir(parents=True, exist_ok=True)
        failure = {
            "schema_version": SCHEMA_VERSION,
            "status": "sealed-failed-truth-free-test-batch",
            "error": str(exc),
            "archive": {"path": str(args.archive), "sha256": _sha256(args.archive) if args.archive.is_file() else None},
            "inventory": {"path": str(args.inventory), "sha256": _sha256(args.inventory) if args.inventory.is_file() else None},
            "sample_submission": {"path": str(args.sample_submission), "present": args.sample_submission.is_file()},
            "truth_policy": {"truth_open_count": 0, "truth_materialized_count": 0, "ground_truth_members_read": False},
            "post_failure_tuning": False,
            "kaggle_external_submission": False,
            "timing": {"total_wall_s": time.perf_counter() - started, "process_peak_rss_kb": int(rss_before)},
        }
        try:
            failure_path = output_dir / "test_batch_failure.json"
            _atomic_json(failure_path, failure)
            failure["artifact"] = _artifact(failure_path)
            _atomic_json(output_dir / "test_batch_failure_manifest.json", failure)
        except OSError:
            pass
        print(f"Smartphone WLS test batch failed: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(run())
