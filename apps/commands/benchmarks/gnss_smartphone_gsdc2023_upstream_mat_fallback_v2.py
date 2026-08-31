#!/usr/bin/env python3
"""Build a truth-free complete GSDC test candidate from published MAT lanes.

The first upstream-MAT batch is intentionally immutable and fails closed when
the published ``result_gnss_imu.mat`` lane has no position for a sample key.
This separate research command resolves those keys using a predeclared,
source-only hierarchy:

1. the exact/interpolated/short-edge projection of ``result_gnss_imu.mat``;
2. the same projection of the public ``result_gnss.mat`` lane;
3. the exact coordinate from the already frozen truth-free v5 output; and
4. the same-trip bounded projection of an already approved truth-free WLS
   source (only if the earlier lanes remain unresolved).

The official sample is read for its header, trip/timestamp keys, and order
only.  Its coordinate cells are never parsed.  No test truth is accepted by
this command, and a missing key, invalid coordinate, cross-trip bracket, long
gap, dummy coordinate, or physical discontinuity fails closed.

This is an isolated local promotion lane.  It does not alter the v1 converter,
the frozen v5 bytes, production RTK/SPP defaults, or any Kaggle state.
"""

from __future__ import annotations

import argparse
import bisect
import csv
from collections import Counter
from dataclasses import dataclass
import hashlib
import json
import math
import os
from pathlib import Path
import resource
import sys
import tempfile
import time
from typing import Any, Iterable, Sequence
from zipfile import ZipFile


COMMANDS_DIR = Path(__file__).resolve().parents[1]
if str(COMMANDS_DIR) not in sys.path:
    sys.path.insert(0, str(COMMANDS_DIR))
from support.gnss_runtime import application_root  # noqa: E402


BENCHMARK_DIR = Path(__file__).resolve().parent
if str(BENCHMARK_DIR) not in sys.path:
    sys.path.insert(0, str(BENCHMARK_DIR))
import gnss_smartphone_gsdc2023_upstream_mat as upstream  # noqa: E402


ROOT = application_root(__file__)
DEFAULT_ARCHIVE = ROOT / "data/gsdc2023/cache/dataset_2023.zip"
DEFAULT_SAMPLE = ROOT / "data/gsdc2023/sample_submission.csv"
DEFAULT_INVENTORY = ROOT / "output/smartphone-r5/gsdc2023-upstream-mat-v1/mat_inventory.json"
DEFAULT_V1_BATCH = ROOT / "output/smartphone-r5/gsdc2023-upstream-mat-v1/test-batch"
DEFAULT_V5 = ROOT / "output/smartphone-r5/native-fgo-test-v5-source-seam-bridge/submission.csv"
DEFAULT_V5_RECORD = ROOT / (
    "docs/use_cases/records/"
    "smartphone_r5_gsdc2023_native_fgo_test_submission_source_seam_bridge_v1.json"
)
DEFAULT_DIAGNOSTIC = ROOT / (
    "output/smartphone-r5/gsdc2023-upstream-mat-v1/test-batch/fallback_diagnostic_v2.json"
)
DEFAULT_OUTPUT = ROOT / "output/smartphone-r5/gsdc2023-upstream-mat-v2/test-batch"
DEFAULT_WLS_ROOTS = (
    ROOT / "output/smartphone-r5/wls-test-batch-derived-unverified-v1",
    ROOT / "output/smartphone-r5/wls-test-batch-edge-completeness-fallback-v2-1",
)
SCHEMA = "smartphone-r5-gsdc2023-upstream-mat-fallback.v2"
SAMPLE_FIELDS = ("tripId", "UnixTimeMillis", "LatitudeDegrees", "LongitudeDegrees")
EXPECTED_ARCHIVE_SHA256 = "bda30ab456e6fd6f83550c246e8dbd287306d5385f1f1069c99c16298e647408"
EXPECTED_SAMPLE_SHA256 = "b0c4853076f715d6bdca46e5c8c99e575f7982d8ee4fc9c0fe417507badcb780"
EXPECTED_V5_SHA256 = "89de0e03ff8eb687c9228ac19a1e62d160400b317352652cf8665b22591eef26"
EXPECTED_V1_BATCH_SHA256 = "3c29d25fee11125fc6e644b52ea8e52b8b10c8d3308d73f84c74835e894885e7"
EXPECTED_FAILURE_SHA256 = "64b787f3b9e110e6fc889c7df16819eac480401c4bb3d1183999f7122c413c84"
EXPECTED_DIAGNOSTIC_SHA256 = "1a5a91d2c87a384af59bedf5d0a1611151eb577dd13482ad039d2e4ccbac5093"
INTERPOLATION_MAX_GAP_MS = 10_000
EDGE_HOLD_MAX_GAP_MS = 1_000
MAX_TRANSITION_SPEED_MPS = 70.0
MIN_ECEF_NORM_M = 6_000_000.0
MAX_ECEF_NORM_M = 7_500_000.0
KNOWN_SAMPLE_DUMMY = upstream.KNOWN_SAMPLE_DUMMY
MAX_MEMBER_BYTES = 256 * 1024 * 1024


class FallbackError(ValueError):
    """Raised when a truth-free source or publication contract fails."""


@dataclass(frozen=True)
class Candidate:
    """A finite position plus a source provenance label."""

    trip_id: str
    timestamp_ms: int
    latitude: float
    longitude: float
    height_m: float
    source: str

    def as_upstream(self) -> upstream.Position:
        return upstream.Position(
            self.trip_id,
            self.timestamp_ms,
            self.latitude,
            self.longitude,
            self.height_m,
            self.source,
        )


def sha256_file(path: Path) -> str:
    if not path.is_file():
        raise FallbackError(f"missing file: {path}")
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def sha256_bytes(payload: bytes) -> str:
    return hashlib.sha256(payload).hexdigest()


def atomic_bytes(path: Path, payload: bytes) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".tmp", dir=str(path.parent)
    )
    temporary = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "wb") as handle:
            handle.write(payload)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, path)
        if hasattr(os, "O_DIRECTORY"):
            directory_descriptor = os.open(str(path.parent), os.O_DIRECTORY)
            try:
                os.fsync(directory_descriptor)
            finally:
                os.close(directory_descriptor)
    finally:
        temporary.unlink(missing_ok=True)


def atomic_json(path: Path, payload: dict[str, Any]) -> None:
    atomic_bytes(path, (json.dumps(payload, indent=2, sort_keys=True) + "\n").encode("utf-8"))


def _require_hash(path: Path, expected: str, label: str) -> str:
    actual = sha256_file(path)
    if actual != expected:
        raise FallbackError(f"{label} SHA256 mismatch: {actual} != {expected}")
    return actual


def _load_json(path: Path, label: str) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise FallbackError(f"invalid {label}: {path}: {exc}") from exc
    if not isinstance(value, dict):
        raise FallbackError(f"{label} is not a JSON object: {path}")
    return value


def _verify_freeze_record(path: Path) -> dict[str, Any]:
    """Fail closed unless this exact source was sealed before generation."""

    freeze = _load_json(path, "fallback freeze record")
    if freeze.get("status") != "frozen-before-test-candidate-generation":
        raise FallbackError("fallback freeze record is not sealed before generation")
    source = freeze.get("frozen_inputs", {}).get("fallback_source", {})
    source_path = Path(__file__).resolve()
    if source.get("path") != str(source_path.relative_to(ROOT.resolve())):
        raise FallbackError("fallback freeze source path does not match this command")
    if source.get("sha256") != sha256_file(source_path):
        raise FallbackError("fallback source changed after freeze")
    inputs = freeze.get("frozen_inputs", {})
    expected_inputs = {
        "archive": EXPECTED_ARCHIVE_SHA256,
        "sample_key_order": EXPECTED_SAMPLE_SHA256,
        "v1_batch_manifest": EXPECTED_V1_BATCH_SHA256,
        "v1_failure_artifact": EXPECTED_FAILURE_SHA256,
        "v1_coverage_diagnostic": EXPECTED_DIAGNOSTIC_SHA256,
        "frozen_v5": EXPECTED_V5_SHA256,
    }
    for key, expected in expected_inputs.items():
        value = inputs.get(key, {})
        actual = value.get("sha256") or value.get("submission_sha256")
        if actual != expected:
            raise FallbackError(f"freeze input hash is not pinned for {key}")
    contract = freeze.get("fallback_contract", {})
    if contract.get("same_trip_only") is not True or contract.get("cross_trip_interpolation") is not False:
        raise FallbackError("freeze does not enforce same-trip-only projection")
    if contract.get("interpolation_max_gap_ms") != INTERPOLATION_MAX_GAP_MS:
        raise FallbackError("freeze interpolation bound differs")
    if contract.get("edge_hold_max_gap_ms") != EDGE_HOLD_MAX_GAP_MS:
        raise FallbackError("freeze edge bound differs")
    if contract.get("continuity") != "all final same-trip transitions and source seams must be <=70 m/s ECEF; no isolated repair or numeric tuning is permitted in this phase":
        raise FallbackError("freeze continuity contract differs")
    policy = freeze.get("policy", {})
    if policy.get("test_truth_open_count") != 0 or policy.get("official_sample_coordinates_read") is not False:
        raise FallbackError("freeze truth/sample policy is not fail-closed")
    return freeze


def _parse_float(value: str, label: str, line: int) -> float:
    try:
        number = float(value.strip())
    except (TypeError, ValueError) as exc:
        raise FallbackError(f"{label}:{line} is not finite") from exc
    if not math.isfinite(number):
        raise FallbackError(f"{label}:{line} is not finite")
    return number


def _ecef(latitude: float, longitude: float, height_m: float = 0.0) -> tuple[float, float, float]:
    if not all(math.isfinite(value) for value in (latitude, longitude, height_m)):
        raise FallbackError("ECEF conversion received a non-finite coordinate")
    latitude_rad = math.radians(latitude)
    longitude_rad = math.radians(longitude)
    semi_major = 6378137.0
    eccentricity_sq = 6.6943799901413165e-3
    sin_latitude = math.sin(latitude_rad)
    radius = semi_major / math.sqrt(1.0 - eccentricity_sq * sin_latitude * sin_latitude)
    return (
        (radius + height_m) * math.cos(latitude_rad) * math.cos(longitude_rad),
        (radius + height_m) * math.cos(latitude_rad) * math.sin(longitude_rad),
        (radius * (1.0 - eccentricity_sq) + height_m) * sin_latitude,
    )


def validate_candidate(candidate: Candidate) -> None:
    if not candidate.trip_id or "/" not in candidate.trip_id:
        raise FallbackError("candidate has an invalid trip id")
    if candidate.timestamp_ms < 0:
        raise FallbackError("candidate has a negative timestamp")
    if not all(
        math.isfinite(value)
        for value in (candidate.latitude, candidate.longitude, candidate.height_m)
    ):
        raise FallbackError("candidate coordinate is non-finite")
    if not -90.0 <= candidate.latitude <= 90.0:
        raise FallbackError("candidate latitude is out of range")
    if not -180.0 <= candidate.longitude <= 180.0:
        raise FallbackError("candidate longitude is out of range")
    if (
        abs(candidate.latitude - KNOWN_SAMPLE_DUMMY[0]) < 1e-9
        and abs(candidate.longitude - KNOWN_SAMPLE_DUMMY[1]) < 1e-9
    ):
        raise FallbackError("known official-sample dummy coordinate is forbidden")
    ecef = _ecef(candidate.latitude, candidate.longitude, candidate.height_m)
    norm = math.sqrt(sum(value * value for value in ecef))
    if not MIN_ECEF_NORM_M <= norm <= MAX_ECEF_NORM_M:
        raise FallbackError(f"candidate ECEF norm is outside Earth bounds: {norm}")


def _candidate_from_position(position: upstream.Position, source: str) -> Candidate:
    candidate = Candidate(
        position.trip_id,
        position.timestamp_ms,
        position.latitude,
        position.longitude,
        position.height_m,
        source,
    )
    validate_candidate(candidate)
    return candidate


def project_from_source(
    rows: Sequence[Candidate],
    trip_id: str,
    timestamp_ms: int,
    source_name: str,
    *,
    interpolation_max_gap_ms: int = INTERPOLATION_MAX_GAP_MS,
    edge_hold_max_gap_ms: int = EDGE_HOLD_MAX_GAP_MS,
) -> Candidate | None:
    """Project one key from one same-trip source under the frozen bounds."""

    source_rows = sorted((row for row in rows if row.trip_id == trip_id), key=lambda row: row.timestamp_ms)
    if not source_rows:
        return None
    timestamps = [row.timestamp_ms for row in source_rows]
    index = bisect.bisect_left(timestamps, timestamp_ms)
    if index < len(source_rows) and source_rows[index].timestamp_ms == timestamp_ms:
        result = Candidate(
            trip_id,
            timestamp_ms,
            source_rows[index].latitude,
            source_rows[index].longitude,
            source_rows[index].height_m,
            f"{source_name}_exact",
        )
        validate_candidate(result)
        return result
    if 0 < index < len(source_rows):
        left = source_rows[index - 1]
        right = source_rows[index]
        gap = right.timestamp_ms - left.timestamp_ms
        if gap <= interpolation_max_gap_ms:
            alpha = (timestamp_ms - left.timestamp_ms) / gap
            result = Candidate(
                trip_id,
                timestamp_ms,
                left.latitude + alpha * (right.latitude - left.latitude),
                left.longitude + alpha * (right.longitude - left.longitude),
                left.height_m + alpha * (right.height_m - left.height_m),
                f"{source_name}_interpolated",
            )
            validate_candidate(result)
            return result
        return None
    if index == 0:
        nearest = source_rows[0]
    else:
        nearest = source_rows[-1]
    if abs(timestamp_ms - nearest.timestamp_ms) <= edge_hold_max_gap_ms:
        result = Candidate(
            trip_id,
            timestamp_ms,
            nearest.latitude,
            nearest.longitude,
            nearest.height_m,
            f"{source_name}_edge_hold",
        )
        validate_candidate(result)
        return result
    return None


def resolve_one(
    trip_id: str,
    timestamp_ms: int,
    sources: Sequence[tuple[str, Sequence[Candidate]]],
) -> Candidate | None:
    """Apply source precedence; each source remains same-trip-only."""

    for source_name, rows in sources:
        candidate = project_from_source(rows, trip_id, timestamp_ms, source_name)
        if candidate is not None:
            return candidate
    return None


def _sample_keys(path: Path) -> list[tuple[str, int]]:
    # Delegate to the frozen v1 helper.  It reads only tripId/UnixTimeMillis;
    # in particular, it does not parse either coordinate column.
    return upstream._sample_keys(path)


def read_keyed_csv(path: Path, *, expected_header: Sequence[str] = SAMPLE_FIELDS) -> list[Candidate]:
    if not path.is_file():
        raise FallbackError(f"missing keyed position CSV: {path}")
    rows: list[Candidate] = []
    seen: set[tuple[str, int]] = set()
    try:
        with path.open(encoding="utf-8-sig", newline="") as handle:
            reader = csv.DictReader(handle)
            if tuple(reader.fieldnames or ()) != tuple(expected_header):
                raise FallbackError(
                    f"{path} header must be {','.join(expected_header)}"
                )
            for line, raw in enumerate(reader, start=2):
                if None in raw:
                    raise FallbackError(f"{path}:{line} has extra columns")
                trip_id = raw.get("tripId") or ""
                if not trip_id or trip_id != trip_id.strip():
                    raise FallbackError(f"{path}:{line} has an invalid tripId")
                try:
                    timestamp_ms = int((raw.get("UnixTimeMillis") or "").strip())
                except ValueError as exc:
                    raise FallbackError(f"{path}:{line} has an invalid timestamp") from exc
                candidate = Candidate(
                    trip_id,
                    timestamp_ms,
                    _parse_float(raw.get("LatitudeDegrees") or "", str(path), line),
                    _parse_float(raw.get("LongitudeDegrees") or "", str(path), line),
                    0.0,
                    "input",
                )
                validate_candidate(candidate)
                key = (trip_id, timestamp_ms)
                if key in seen:
                    raise FallbackError(f"{path}:{line} duplicates key {key!r}")
                seen.add(key)
                rows.append(candidate)
    except OSError as exc:
        raise FallbackError(f"failed to read {path}: {exc}") from exc
    return rows


def read_v1_route(path: Path) -> list[Candidate]:
    """Read a v1 result_gnss_imu route artifact (truth-free)."""

    return [
        Candidate(row.trip_id, row.timestamp_ms, row.latitude, row.longitude, row.height_m, "result_gnss_imu")
        for row in read_keyed_csv(path)
    ]


def read_wls_positions(path: Path, trip_id: str) -> list[Candidate]:
    """Read an already-produced WLS ``.pos`` as an approved fallback only."""

    try:
        import gnss_smartphone_trajectory_smoother as smoother
        parsed = smoother._read_positions(path, smoother.DEFAULT_GPS_UTC_LEAP_SECONDS)
    except Exception as exc:
        raise FallbackError(f"failed to read approved WLS source {path}: {exc}") from exc
    rows = [
        Candidate(trip_id, row.timestamp_ms, row.latitude, row.longitude, row.height, "raw_wls")
        for row in parsed
    ]
    for row in rows:
        validate_candidate(row)
    return rows


def _read_zip_member(zf: ZipFile, name: str) -> bytes:
    try:
        info = zf.getinfo(name)
    except KeyError as exc:
        raise FallbackError(f"archive member is missing: {name}") from exc
    if info.file_size > MAX_MEMBER_BYTES:
        raise FallbackError(f"archive member exceeds safety limit: {name}")
    with zf.open(info, "r") as handle:
        payload = handle.read(MAX_MEMBER_BYTES + 1)
    if len(payload) != info.file_size:
        raise FallbackError(f"archive member size mismatch: {name}")
    return payload


def _inventory_member(inventory: dict[str, Any], name: str) -> dict[str, Any]:
    for member in inventory.get("members", []):
        if member.get("path") == name:
            return member
    raise FallbackError(f"archive member is absent from pinned inventory: {name}")


def _verify_member_payload(inventory: dict[str, Any], name: str, payload: bytes) -> str:
    member = _inventory_member(inventory, name)
    actual = sha256_bytes(payload)
    if actual != member.get("sha256"):
        raise FallbackError(f"archive member hash mismatch: {name}")
    if len(payload) != member.get("size_bytes"):
        raise FallbackError(f"archive member size differs from inventory: {name}")
    return actual


def _archive_route_sources(
    zf: ZipFile,
    inventory: dict[str, Any],
    trip_id: str,
) -> dict[str, Any]:
    route, phone = trip_id.split("/", 1)
    prefix = f"dataset_2023/test/{route}/{phone}"
    result_imu_name = f"{prefix}/result_gnss_imu.mat"
    result_gnss_name = f"{prefix}/result_gnss.mat"
    phone_name = f"{prefix}/phone_data.mat"
    phone_payload = _read_zip_member(zf, phone_name)
    _verify_member_payload(inventory, phone_name, phone_payload)
    timestamps = upstream.timestamps_from_phone_data_bytes(phone_payload)
    result_imu_payload = _read_zip_member(zf, result_imu_name)
    result_gnss_payload = _read_zip_member(zf, result_gnss_name)
    imu_hash = _verify_member_payload(inventory, result_imu_name, result_imu_payload)
    gnss_hash = _verify_member_payload(inventory, result_gnss_name, result_gnss_payload)
    imu_rows = [
        Candidate(row.trip_id, row.timestamp_ms, row.latitude, row.longitude, row.height_m, "result_gnss_imu")
        for row in upstream.positions_from_result(
            result_imu_payload,
            timestamps,
            trip_id,
            "result_gnss_imu",
            allow_missing_rows=True,
        )
    ]
    gnss_rows = [
        Candidate(row.trip_id, row.timestamp_ms, row.latitude, row.longitude, row.height_m, "result_gnss")
        for row in upstream.positions_from_result(
            result_gnss_payload,
            timestamps,
            trip_id,
            "result_gnss",
            allow_missing_rows=True,
        )
    ]
    for row in (*imu_rows, *gnss_rows):
        validate_candidate(row)
    return {
        "result_gnss_imu": imu_rows,
        "result_gnss": gnss_rows,
        "members": {
            "result_gnss_imu": {"path": result_imu_name, "sha256": imu_hash},
            "result_gnss": {"path": result_gnss_name, "sha256": gnss_hash},
            "phone_data": {"path": phone_name, "sha256": sha256_bytes(phone_payload)},
        },
        "phone_data_timestamps": {
            "count": len(timestamps),
            "first_ms": timestamps[0],
            "last_ms": timestamps[-1],
        },
        "finite_rows": {
            "result_gnss_imu": len(imu_rows),
            "result_gnss": len(gnss_rows),
        },
    }


def _write_rows(path: Path, rows: Sequence[Candidate]) -> None:
    output: list[str] = [",".join(SAMPLE_FIELDS)]
    for row in rows:
        output.append(
            ",".join(
                (
                    row.trip_id,
                    str(row.timestamp_ms),
                    f"{row.latitude:.12f}",
                    f"{row.longitude:.12f}",
                )
            )
        )
    atomic_bytes(path, ("\n".join(output) + "\n").encode("ascii"))


def _verify_submission(path: Path, keys: Sequence[tuple[str, int]]) -> dict[str, Any]:
    rows = read_keyed_csv(path)
    actual_keys = [(row.trip_id, row.timestamp_ms) for row in rows]
    if actual_keys != list(keys):
        raise FallbackError("published submission key order differs from sample key authority")
    continuity = upstream.continuity_statistics([row.as_upstream() for row in rows])
    if continuity["over_bound_count"]:
        raise FallbackError(
            f"published submission has {continuity['over_bound_count']} transitions over "
            f"{MAX_TRANSITION_SPEED_MPS:g} m/s"
        )
    return {
        "header_exact": True,
        "row_count": len(rows),
        "duplicate_keys": 0,
        "missing_keys": 0,
        "extra_keys": 0,
        "nonfinite_coordinates": 0,
        "known_dummy_rows": 0,
        "out_of_earth_rows": 0,
        "continuity": continuity,
    }


def _source_seam_statistics(rows: Sequence[Candidate]) -> dict[str, Any]:
    by_trip: dict[str, list[Candidate]] = {}
    for row in rows:
        by_trip.setdefault(row.trip_id, []).append(row)
    changed: list[dict[str, Any]] = []
    for trip_id, trip_rows in by_trip.items():
        ordered = sorted(trip_rows, key=lambda row: row.timestamp_ms)
        for left, right in zip(ordered, ordered[1:]):
            if left.source == right.source:
                continue
            delta_ms = right.timestamp_ms - left.timestamp_ms
            if delta_ms <= 0:
                raise FallbackError("source seam timestamps are not increasing")
            left_ecef = _ecef(left.latitude, left.longitude, left.height_m)
            right_ecef = _ecef(right.latitude, right.longitude, right.height_m)
            speed = math.sqrt(sum((a - b) ** 2 for a, b in zip(left_ecef, right_ecef))) / (delta_ms / 1000.0)
            changed.append(
                {
                    "trip_id": trip_id,
                    "from_timestamp_ms": left.timestamp_ms,
                    "to_timestamp_ms": right.timestamp_ms,
                    "from_source": left.source,
                    "to_source": right.source,
                    "gap_ms": delta_ms,
                    "speed_mps": speed,
                }
            )
    return {
        "source_change_count": len(changed),
        "over_bound_source_changes": sum(item["speed_mps"] > MAX_TRANSITION_SPEED_MPS for item in changed),
        "changes": changed,
    }


def _compact_diagnostic(diagnostic: dict[str, Any], trip_id: str) -> dict[str, Any]:
    route = diagnostic.get("routes", {}).get(trip_id, {})
    coverage = route.get("coverage", {})
    selected: dict[str, Any] = {}
    for label in (
        "batch_result_gnss_imu_route_csv",
        "result_gnss_imu",
        "result_gnss",
        "phone_data_obs_utcms",
        "raw_wls_candidates",
        "native_fgo_v2",
        "frozen_v5_submission",
    ):
        if label in coverage:
            value = coverage[label]
            if label == "raw_wls_candidates":
                selected[label] = [
                    {
                        key: item.get(key)
                        for key in ("path", "sha256", "decoded_rows", "sample_key_coverage")
                        if key in item
                    }
                    for item in value
                ]
            else:
                selected[label] = value
    return {
        "trip_id": trip_id,
        "sample_key_count": route.get("sample_key_count"),
        "coverage": selected,
        "unresolved_under_v1": coverage.get("unresolved_under_v1"),
    }


def _wls_path_for_trip(trip_id: str, roots: Sequence[Path]) -> Path | None:
    route, phone = trip_id.split("/", 1)
    candidates = [root / "routes" / route / "outputs" / phone / "selected.pos" for root in roots]
    for path in candidates:
        if path.is_file():
            return path
    return None


def build_test_submission(
    archive_path: Path,
    sample_path: Path,
    inventory_path: Path,
    v1_batch_dir: Path,
    v5_path: Path,
    v5_record_path: Path,
    diagnostic_path: Path,
    output_dir: Path,
    wls_roots: Sequence[Path],
    freeze_record_path: Path | None = None,
) -> dict[str, Any]:
    started = time.monotonic()
    freeze = _verify_freeze_record(freeze_record_path) if freeze_record_path is not None else None
    archive_sha = _require_hash(archive_path, EXPECTED_ARCHIVE_SHA256, "archive")
    sample_sha = _require_hash(sample_path, EXPECTED_SAMPLE_SHA256, "sample key authority")
    v5_sha = _require_hash(v5_path, EXPECTED_V5_SHA256, "frozen v5 submission")
    v5_record_sha = sha256_file(v5_record_path)
    inventory = _load_json(inventory_path, "MAT inventory")
    if inventory.get("archive", {}).get("sha256") != archive_sha:
        raise FallbackError("pinned MAT inventory archive hash differs")
    v1_manifest_path = v1_batch_dir / "test_batch.manifest.json"
    v1_manifest_sha = _require_hash(v1_manifest_path, EXPECTED_V1_BATCH_SHA256, "v1 batch manifest")
    failure_path = v1_batch_dir / "batch_failure.json"
    failure_sha = _require_hash(failure_path, EXPECTED_FAILURE_SHA256, "v1 failure artifact")
    failure = _load_json(failure_path, "v1 failure artifact")
    if failure.get("test_truth_materialized") is not False or failure.get("truth_used") is not False:
        raise FallbackError("v1 failure artifact does not prove truth-free operation")
    diagnostic_sha = _require_hash(diagnostic_path, EXPECTED_DIAGNOSTIC_SHA256, "coverage diagnostic")
    diagnostic = _load_json(diagnostic_path, "coverage diagnostic")
    if diagnostic.get("test_truth_used") is not False or diagnostic.get("sample_coordinates_read") is not False:
        raise FallbackError("coverage diagnostic does not prove test-truth/sample-coordinate isolation")
    v5_record = _load_json(v5_record_path, "frozen v5 record")
    if v5_record.get("publication_policy", {}).get("kaggle_external_submission") is not False:
        raise FallbackError("frozen v5 record does not prove local-only provenance")
    v5_verification = v5_record.get("verification", {})
    if (
        v5_verification.get("v5_sample_coordinate_fallback_rows") != 0
        or v5_verification.get("v5_known_dummy_rows") != 0
        or v5_verification.get("v5_out_of_earth_rows") != 0
    ):
        raise FallbackError("frozen v5 record does not prove non-dummy finite coordinates")
    keys = _sample_keys(sample_path)
    if len(keys) != 71_936:
        raise FallbackError(f"expected 71936 sample keys, found {len(keys)}")
    by_trip: dict[str, list[tuple[str, int]]] = {}
    for key in keys:
        by_trip.setdefault(key[0], []).append(key)
    if len(by_trip) != 40:
        raise FallbackError(f"expected 40 test trips, found {len(by_trip)}")

    v5_rows = read_keyed_csv(v5_path)
    v5_by_key = {(row.trip_id, row.timestamp_ms): Candidate(
        row.trip_id, row.timestamp_ms, row.latitude, row.longitude, row.height_m, "v5"
    ) for row in v5_rows}
    if set(v5_by_key) != set(keys):
        raise FallbackError("frozen v5 does not cover exactly the official key set")

    route_rows: list[Candidate] = []
    route_manifests: list[dict[str, Any]] = []
    source_counts: Counter[str] = Counter()
    archive_routes_loaded: list[str] = []
    wls_loaded: list[dict[str, Any]] = []
    with ZipFile(archive_path) as zf:
        for trip_id in sorted(by_trip):
            trip_keys = by_trip[trip_id]
            route_csv = v1_batch_dir / "routes" / (trip_id.replace("/", "__") + ".csv")
            primary_rows = read_v1_route(route_csv)
            primary_map = {(row.trip_id, row.timestamp_ms): row for row in primary_rows}
            if len(primary_map) != len(primary_rows):
                raise FallbackError(f"duplicate v1 route keys: {trip_id}")
            route_result: list[Candidate] = []
            missing_before_fallback: list[int] = []
            archive_sources: dict[str, Any] | None = None
            wls_rows: list[Candidate] = []
            wls_path = _wls_path_for_trip(trip_id, wls_roots)
            for route_trip, timestamp_ms in trip_keys:
                direct = primary_map.get((route_trip, timestamp_ms))
                if direct is not None:
                    selected = Candidate(
                        direct.trip_id,
                        direct.timestamp_ms,
                        direct.latitude,
                        direct.longitude,
                        direct.height_m,
                        "result_gnss_imu_exact",
                    )
                else:
                    selected = None
                    missing_before_fallback.append(timestamp_ms)
                    if archive_sources is None:
                        archive_sources = _archive_route_sources(zf, inventory, trip_id)
                        archive_routes_loaded.append(trip_id)
                    selected = resolve_one(
                        route_trip,
                        timestamp_ms,
                        (
                            ("result_gnss_imu", archive_sources["result_gnss_imu"]),
                            ("result_gnss", archive_sources["result_gnss"]),
                            ("v5", [v5_by_key[(route_trip, timestamp_ms)]] if (route_trip, timestamp_ms) in v5_by_key else []),
                        ),
                    )
                    if selected is None and wls_path is not None:
                        if not wls_rows:
                            wls_rows = read_wls_positions(wls_path, trip_id)
                            wls_loaded.append(
                                {
                                    "trip_id": trip_id,
                                    "path": str(wls_path),
                                    "sha256": sha256_file(wls_path),
                                    "rows": len(wls_rows),
                                }
                            )
                        selected = resolve_one(route_trip, timestamp_ms, (("raw_wls", wls_rows),))
                    if selected is None:
                        raise FallbackError(f"unresolved official key after hierarchy: {trip_id}/{timestamp_ms}")
                validate_candidate(selected)
                route_result.append(selected)
                source_counts[selected.source] += 1
            if len(route_result) != len(trip_keys):
                raise FallbackError(f"route key count mismatch: {trip_id}")
            route_rows.extend(route_result)
            route_path = output_dir / "routes" / (trip_id.replace("/", "__") + ".csv")
            _write_rows(route_path, route_result)
            route_manifests.append(
                {
                    "trip_id": trip_id,
                    "sample_key_count": len(trip_keys),
                    "v1_input": {
                        "path": str(route_csv),
                        "sha256": sha256_file(route_csv),
                        "rows": len(primary_rows),
                        "missing_before_fallback": len(missing_before_fallback),
                        "missing_timestamps_ms": missing_before_fallback,
                    },
                    "archive_sources": (
                        {
                            "members": archive_sources["members"],
                            "phone_data_timestamps": archive_sources["phone_data_timestamps"],
                            "finite_rows": archive_sources["finite_rows"],
                        }
                        if archive_sources is not None
                        else None
                    ),
                    "selected_source_counts": dict(sorted(Counter(row.source for row in route_result).items())),
                    "selected_keys": len(route_result),
                    "route_artifact": {
                        "path": str(route_path),
                        "sha256": sha256_file(route_path),
                        "rows": len(route_result),
                    },
                }
            )

    if len(route_rows) != len(keys):
        raise FallbackError(f"candidate row count mismatch: {len(route_rows)} != {len(keys)}")
    actual_keys = [(row.trip_id, row.timestamp_ms) for row in route_rows]
    if actual_keys != keys:
        raise FallbackError("candidate route concatenation changed sample order")
    continuity = upstream.continuity_statistics([row.as_upstream() for row in route_rows])
    if continuity["over_bound_count"]:
        raise FallbackError("candidate continuity gate failed")
    seam_statistics = _source_seam_statistics(route_rows)
    if seam_statistics["over_bound_source_changes"]:
        raise FallbackError("candidate source-seam continuity gate failed")

    output_dir.mkdir(parents=True, exist_ok=True)
    submission_path = output_dir / "submission.csv"
    _write_rows(submission_path, route_rows)
    verification = _verify_submission(submission_path, keys)
    submission_sha = sha256_file(submission_path)
    submission_manifest_path = output_dir / "submission.manifest.json"
    submission_manifest = {
        "schema_version": SCHEMA + "-submission-manifest",
        "status": "completed-truth-free-upstream-mat-fallback",
        "truth_used": False,
        "test_truth_materialized": False,
        "test_truth_open_count": 0,
        "official_sample_coordinates_used": False,
        "sample_key_authority": {
            "path": str(sample_path),
            "sha256": sample_sha,
            "rows": len(keys),
            "header": list(SAMPLE_FIELDS),
            "coordinates_read": False,
        },
        "source_precedence": [
            "result_gnss_imu exact/interpolated/edge_hold",
            "result_gnss exact/interpolated/edge_hold",
            "frozen v5 exact/interpolated/edge_hold",
            "approved truth-free raw WLS exact/interpolated/edge_hold",
        ],
        "bounded_contract": {
            "same_trip_only": True,
            "cross_trip_interpolation": False,
            "interpolation_max_gap_ms": INTERPOLATION_MAX_GAP_MS,
            "edge_hold_max_gap_ms": EDGE_HOLD_MAX_GAP_MS,
            "edge_velocity_extrapolation": False,
            "long_gap_policy": "use a finite higher-priority source if available; otherwise fail-closed",
            "max_transition_speed_mps": MAX_TRANSITION_SPEED_MPS,
            "ecef_norm_m": [MIN_ECEF_NORM_M, MAX_ECEF_NORM_M],
            "known_sample_dummy": list(KNOWN_SAMPLE_DUMMY),
            "sample_coordinate_fallback": "forbidden",
            "truth_fallback": "forbidden",
        },
        "artifacts": {
            "submission": {
                "path": str(submission_path),
                "sha256": submission_sha,
                "bytes": submission_path.stat().st_size,
                "rows": len(route_rows),
            }
        },
        "source_counts": dict(sorted(source_counts.items())),
        "verification": verification,
    }
    atomic_json(submission_manifest_path, submission_manifest)

    diagnostic_evidence = {
        "path": str(diagnostic_path),
        "sha256": diagnostic_sha,
        "routes": {
            trip_id: _compact_diagnostic(diagnostic, trip_id)
            for trip_id in sorted(("2022-04-25-21-04-us-ca-ebf-x/mi8", "2022-10-06-20-46-us-ca-sjc-r/sm-a205u"))
        },
    }
    run_manifest = {
        "schema_version": SCHEMA + "-run-manifest",
        "status": "completed-truth-free-upstream-mat-fallback",
        "truth_policy": {
            "test_truth_materialized": False,
            "test_truth_open_count": 0,
            "validation_truth_open_count": 0,
            "holdout_truth_open_count": 0,
            "official_sample_coordinates_used": False,
            "token_access": False,
            "external_mutation": False,
        },
        "freeze_record": (
            {"path": str(freeze_record_path), "sha256": sha256_file(freeze_record_path)}
            if freeze_record_path is not None
            else None
        ),
        "inputs": {
            "archive": {"path": str(archive_path), "sha256": archive_sha},
            "inventory": {"path": str(inventory_path), "sha256": sha256_file(inventory_path)},
            "v1_batch_manifest": {"path": str(v1_manifest_path), "sha256": v1_manifest_sha},
            "v1_failure_artifact": {"path": str(failure_path), "sha256": failure_sha},
            "official_sample_key_order": {"path": str(sample_path), "sha256": sample_sha, "coordinates_read": False},
            "frozen_v5_submission": {"path": str(v5_path), "sha256": v5_sha},
            "frozen_v5_record": {"path": str(v5_record_path), "sha256": v5_record_sha},
            "v5_sample_coordinate_fallback": v5_record.get("verification", {}).get("v5_sample_coordinate_fallback_rows") == 0,
        },
        "published_upstream_semantics": {
            "source": "output/reproducibility-cache/gsdc2023/functions/submission.m",
            "sha256": sha256_file(ROOT / "output/reproducibility-cache/gsdc2023/functions/submission.m"),
            "lines": "21-38",
            "semantics": "intersect obs.utcms with sample UTC keys; nearest fillmissing; nearest interp1 extrapolation",
            "local_contract": "same-trip only, <=10000ms bracket, <=1000ms edge hold, no unbounded extrapolation",
        },
        "coverage_diagnostic": diagnostic_evidence,
        "archive_routes_decoded_for_fallback": sorted(archive_routes_loaded),
        "wls_fallback_sources_loaded": wls_loaded,
        "source_counts": dict(sorted(source_counts.items())),
        "route_count": len(route_manifests),
        "sample_key_count": len(keys),
        "route_manifests": route_manifests,
        "quality": {
            "continuity": continuity,
            "source_seams": seam_statistics,
            "dummy_rows": 0,
            "out_of_earth_rows": 0,
            "unresolved_rows": 0,
        },
        "artifacts": {
            "submission": {"path": str(submission_path), "sha256": submission_sha, "bytes": submission_path.stat().st_size},
            "submission_manifest": {"path": str(submission_manifest_path), "sha256": sha256_file(submission_manifest_path)},
        },
        "runtime": {
            "wall_seconds": time.monotonic() - started,
            "max_rss_bytes": resource.getrusage(resource.RUSAGE_SELF).ru_maxrss * 1024,
        },
        "reproduction": {
            "truth_free": True,
            "algorithm_rerun": False,
            "sample_coordinates_read": False,
            "command": "PYTHONHASHSEED=0 python3 apps/commands/benchmarks/gnss_smartphone_gsdc2023_upstream_mat_fallback_v2.py test-batch ...",
            "repeat_policy": "same frozen inputs may be regenerated in a separate directory; submission bytes must match",
        },
    }
    run_manifest_path = output_dir / "fallback_v2_run_manifest.json"
    atomic_json(run_manifest_path, run_manifest)
    run_manifest["artifacts"]["run_manifest"] = {
        "path": str(run_manifest_path),
        "sha256": sha256_file(run_manifest_path),
    }
    return run_manifest


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    batch = sub.add_parser("test-batch", help="build the complete truth-free test candidate")
    batch.add_argument("--archive", type=Path, default=DEFAULT_ARCHIVE)
    batch.add_argument("--sample", type=Path, default=DEFAULT_SAMPLE)
    batch.add_argument("--inventory", type=Path, default=DEFAULT_INVENTORY)
    batch.add_argument("--v1-batch-dir", type=Path, default=DEFAULT_V1_BATCH)
    batch.add_argument("--v5", type=Path, default=DEFAULT_V5)
    batch.add_argument("--v5-record", type=Path, default=DEFAULT_V5_RECORD)
    batch.add_argument("--diagnostic", type=Path, default=DEFAULT_DIAGNOSTIC)
    batch.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT)
    batch.add_argument("--wls-root", type=Path, action="append", dest="wls_roots")
    batch.add_argument("--freeze-record", type=Path)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = _build_parser().parse_args(argv)
    try:
        if args.command != "test-batch":
            raise FallbackError(f"unsupported command: {args.command}")
        result = build_test_submission(
            args.archive,
            args.sample,
            args.inventory,
            args.v1_batch_dir,
            args.v5,
            args.v5_record,
            args.diagnostic,
            args.output_dir,
            tuple(args.wls_roots or DEFAULT_WLS_ROOTS),
            args.freeze_record,
        )
    except (FallbackError, OSError, ValueError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2
    print(json.dumps(result, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
