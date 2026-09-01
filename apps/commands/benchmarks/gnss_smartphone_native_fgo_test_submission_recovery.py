#!/usr/bin/env python3
"""Recover the frozen truth-free test submission without sample coordinates.

The first native-FGO test batch used the official sample's dummy coordinates
for unresolved keys.  This recovery lane is deliberately separate from that
batch.  It reads the sample only as a key/order/schema authority and rebuilds
every coordinate from already sealed native-FGO/WLS position artifacts.  A
small, predeclared same-trip ECEF interpolation/edge-hold contract handles
the sparse epochs; an unresolved key is a hard failure.

No archive payload is materialized here, no truth member is opened, and no
FGO/WLS process is started.  The v2 route manifests and the prior frozen WLS
route outputs are content-addressed inputs.  The resulting CSV is a local,
development-only artifact and is not a Kaggle submission.
"""

from __future__ import annotations

import argparse
from collections import Counter
import csv
from dataclasses import dataclass
import hashlib
import io
import json
import math
import os
from pathlib import Path
import sys
import tempfile
import time
from typing import Any, Iterable, Mapping, Sequence

from support.gnss_runtime import application_root


ROOT = application_root(__file__)
BENCHMARK_DIR = ROOT / "apps" / "commands" / "benchmarks"
if str(BENCHMARK_DIR) not in sys.path:
    sys.path.insert(0, str(BENCHMARK_DIR))

import gnss_smartphone_trajectory_smoother as smoother  # noqa: E402


SAMPLE_FIELDS = (
    "tripId",
    "UnixTimeMillis",
    "LatitudeDegrees",
    "LongitudeDegrees",
)
RECOVERY_SCHEMA = "smartphone-r5-gsdc2023-native-fgo-test-submission-recovery.v3"
FREEZE_SCHEMA = "smartphone-r5-gsdc2023-native-fgo-test-submission-recovery-freeze.v3"
FREEZE_MANIFEST_SCHEMA = (
    "smartphone-r5-gsdc2023-native-fgo-test-submission-recovery-freeze-manifest.v3"
)
RUN_MANIFEST_SCHEMA = (
    "smartphone-r5-gsdc2023-native-fgo-test-submission-recovery-run-manifest.v3"
)
POSITION_MANIFEST_SCHEMA = (
    "smartphone-r5-gsdc2023-native-fgo-test-submission-recovery-position-manifest.v1"
)
LEAP_SECONDS = 18
EXPECTED_SAMPLE_ROWS = 71936
EXPECTED_ROUTE_COUNT = 40
EXPECTED_PRIOR_SAMPLE_FALLBACK_KEYS = 24

# The largest already-sealed missing-key deficit is 2,004 ms (two leading
# sparse epochs in the old sm-a205u WLS output).  2.5 s is a fixed recovery
# contract with a 496 ms margin, not a truth-derived or model-tuned value.
MAX_INTERPOLATION_GAP_MS = 2500
MAX_EDGE_HOLD_GAP_MS = 2500
EARTH_ECEF_NORM_MIN_M = 6_000_000.0
EARTH_ECEF_NORM_MAX_M = 7_000_000.0
KNOWN_DUMMY_LATITUDE = 34.640195
KNOWN_DUMMY_LONGITUDE = -120.589642
KNOWN_DUMMY_TOLERANCE_DEG = 1e-9
KNOWN_DUMMY_MIN_ROUTE_DISTANCE_M = 10_000.0

FGO_RECIPE_HASH = "4633bfd3a86cf34ebd86820ed59ee7192b3cbf23fc75ce9e72fc1f2c88fb39f6"
FGO_CORE_HASH = "e27eb31f4fcb597a1c5b392d8b558f429b25cd5c5a4b39b6474491209649f7b1"
ARCHIVE_SHA256 = "bda30ab456e6fd6f83550c246e8dbd287306d5385f1f1069c99c16298e647408"
INVENTORY_SHA256 = "dbf89222e6860d4fb31f12c9c46402033ed31e905f8fe3f6615e5131cb9e9495"
SAMPLE_SHA256 = "b0c4853076f715d6bdca46e5c8c99e575f7982d8ee4fc9c0fe417507badcb780"
V2_RUN_SHA256 = "0808082ed8c81d24e5b5403812cca50651d493f39ea2c70ff6a91b6df556518c"
OLD_WLS_RUN_SHA256 = "a89c5c3ac816289b6c6e5958d77fbc034f601869c3e64bd5191d87808fe74822"
OLD_RECONCILIATION_SHA256 = "9b611f1920bb48597eac518cf9f4a8e26f54f3719598ff9aaebea4839caa703a"

DEFAULT_ARCHIVE = ROOT / "data" / "gsdc2023" / "cache" / "dataset_2023.zip"
DEFAULT_INVENTORY = ROOT / "output" / "smartphone-r5" / "wls-test-batch-v1" / "test_inventory.json"
DEFAULT_SAMPLE = ROOT / "data" / "gsdc2023" / "sample_submission.csv"
DEFAULT_V2_ROOT = ROOT / "output" / "smartphone-r5" / "native-fgo-test-v2"
DEFAULT_OLD_WLS_ROOT = ROOT / "output" / "smartphone-r5" / "wls-test-batch-derived-unverified-v1"
DEFAULT_RECONCILIATION = (
    ROOT
    / "output"
    / "smartphone-r5"
    / "wls-test-batch-edge-completeness-fallback-v2-1"
    / "test_batch_reconciliation_v2_manifest.json"
)
DEFAULT_FREEZE = (
    ROOT
    / "docs"
    / "use_cases"
    / "records"
    / "smartphone_r5_gsdc2023_native_fgo_test_submission_recovery_freeze_v3.json"
)
DEFAULT_FREEZE_MANIFEST = DEFAULT_FREEZE.with_name(
    "smartphone_r5_gsdc2023_native_fgo_test_submission_recovery_freeze_v3_manifest.json"
)
DEFAULT_OUTPUT = ROOT / "output" / "smartphone-r5" / "native-fgo-test-v3-recovered"


class RecoveryError(ValueError):
    """Raised when the sealed truth-free recovery contract cannot be proven."""


@dataclass(frozen=True)
class PositionSample:
    """A finite position that came from an already sealed artifact."""

    timestamp_ms: int
    ecef: tuple[float, float, float]
    latitude: float
    longitude: float
    height: float
    source: str


def _sha256(path: Path) -> str:
    if not path.is_file():
        raise RecoveryError(f"missing artifact: {path}")
    digest = hashlib.sha256()
    try:
        with path.open("rb") as handle:
            for chunk in iter(lambda: handle.read(1024 * 1024), b""):
                digest.update(chunk)
    except OSError as exc:
        raise RecoveryError(f"failed to hash {path}: {exc}") from exc
    return digest.hexdigest()


def _load_json(path: Path, label: str) -> dict[str, Any]:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise RecoveryError(f"invalid {label}: {path}") from exc
    if not isinstance(payload, dict):
        raise RecoveryError(f"{label} must be a JSON object: {path}")
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
    finally:
        temporary.unlink(missing_ok=True)


def _atomic_json(path: Path, payload: dict[str, Any]) -> None:
    _atomic_bytes(
        path,
        (json.dumps(payload, indent=2, sort_keys=True) + "\n").encode("utf-8"),
    )


def _artifact(path: Path, *, name: str | None = None) -> dict[str, Any]:
    if not path.is_file():
        raise RecoveryError(f"missing artifact: {path}")
    return {
        "path": name if name is not None else str(path.relative_to(ROOT)),
        "bytes": path.stat().st_size,
        "sha256": _sha256(path),
    }


def _repo_path(value: str | Path) -> Path:
    candidate = Path(value)
    if candidate.is_absolute() or ".." in candidate.parts:
        raise RecoveryError(f"artifact path must be repository-relative: {value}")
    return ROOT / candidate


def _verify_artifact(path: Path, expected: Mapping[str, Any], label: str) -> dict[str, Any]:
    if not isinstance(expected, Mapping):
        raise RecoveryError(f"{label} artifact metadata is malformed")
    actual = _artifact(path)
    if expected.get("sha256") != actual["sha256"] or int(expected.get("bytes", -1)) != actual["bytes"]:
        raise RecoveryError(f"{label} hash/size differs: {path}")
    return actual


def _read_sample_keys(path: Path) -> dict[str, Any]:
    """Read only key/order fields; official coordinate cells are never used."""

    rows: list[dict[str, Any]] = []
    seen: set[tuple[str, int]] = set()
    try:
        with path.open(encoding="utf-8-sig", newline="") as handle:
            reader = csv.DictReader(handle)
            if tuple(reader.fieldnames or ()) != SAMPLE_FIELDS:
                raise RecoveryError("official sample header is not exact")
            for line, raw in enumerate(reader, start=2):
                if None in raw:
                    raise RecoveryError(f"official sample row {line} has extra fields")
                trip_id = raw.get("tripId") or ""
                timestamp_text = raw.get("UnixTimeMillis") or ""
                if (
                    not trip_id
                    or trip_id != trip_id.strip()
                    or not trip_id.isprintable()
                    or any(char.isspace() for char in trip_id)
                    or trip_id.count("/") != 1
                    or timestamp_text != timestamp_text.strip()
                ):
                    raise RecoveryError(f"official sample row {line} key is invalid")
                try:
                    timestamp = int(timestamp_text)
                except ValueError as exc:
                    raise RecoveryError(f"official sample row {line} timestamp is invalid") from exc
                if timestamp < 0:
                    raise RecoveryError(f"official sample row {line} timestamp is negative")
                key = (trip_id, timestamp)
                if key in seen:
                    raise RecoveryError(f"official sample duplicate key: {key!r}")
                seen.add(key)
                # Do not read, parse, or retain LatitudeDegrees/LongitudeDegrees.
                rows.append(
                    {
                        "trip_id": trip_id,
                        "timestamp": timestamp,
                        "timestamp_text": timestamp_text,
                    }
                )
    except OSError as exc:
        raise RecoveryError(f"failed to read official sample keys: {path}") from exc
    if len(rows) != EXPECTED_SAMPLE_ROWS:
        raise RecoveryError(f"official sample row count differs: {len(rows)}")
    return {
        "header": list(SAMPLE_FIELDS),
        "rows": rows,
        "key_count": len(rows),
        "artifact": _artifact(path),
        "coordinates_read": False,
    }


def _is_known_dummy(latitude: float, longitude: float) -> bool:
    return (
        abs(latitude - KNOWN_DUMMY_LATITUDE) <= KNOWN_DUMMY_TOLERANCE_DEG
        and abs(longitude - KNOWN_DUMMY_LONGITUDE) <= KNOWN_DUMMY_TOLERANCE_DEG
    )


def _great_circle_distance_m(first: tuple[float, float], second: tuple[float, float]) -> float:
    radius = 6_378_137.0
    lat1, lon1 = (math.radians(value) for value in first)
    lat2, lon2 = (math.radians(value) for value in second)
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    term = math.sin(dlat / 2.0) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2.0) ** 2
    return 2.0 * radius * math.asin(min(1.0, math.sqrt(max(0.0, term))))


def _guard_coordinate(
    latitude: float,
    longitude: float,
    finite_route_estimates: Iterable[tuple[float, float]] = (),
) -> None:
    """Reject impossible coordinates and the known 300-km sample dummy."""

    if not all(math.isfinite(value) for value in (latitude, longitude)):
        raise RecoveryError("recovered coordinate is non-finite")
    if not -90.0 <= latitude <= 90.0 or not -180.0 <= longitude <= 180.0:
        raise RecoveryError("recovered coordinate is out of range")
    if not _is_known_dummy(latitude, longitude):
        return
    estimates = tuple(finite_route_estimates)
    if not estimates:
        raise RecoveryError("known sample dummy cannot be emitted without route estimates")
    nearest = min(
        _great_circle_distance_m((latitude, longitude), estimate) for estimate in estimates
    )
    if nearest > KNOWN_DUMMY_MIN_ROUTE_DISTANCE_M:
        raise RecoveryError(
            f"known sample dummy is implausibly far from route estimates: {nearest:.1f} m"
        )


def _position_samples(path: Path, source: str) -> dict[int, PositionSample]:
    try:
        rows = smoother._read_positions(path, LEAP_SECONDS)
    except (OSError, ValueError, KeyError, TypeError) as exc:
        raise RecoveryError(f"invalid sealed position artifact: {path}") from exc
    result: dict[int, PositionSample] = {}
    for row in rows:
        ecef = tuple(float(value) for value in row.ecef)
        latitude = float(row.latitude)
        longitude = float(row.longitude)
        height = float(row.height)
        values = (*ecef, latitude, longitude, height)
        if not all(math.isfinite(value) for value in values):
            raise RecoveryError(f"non-finite sealed position: {path}:{row.source_line}")
        norm = math.sqrt(sum(value * value for value in ecef))
        if not EARTH_ECEF_NORM_MIN_M <= norm <= EARTH_ECEF_NORM_MAX_M:
            raise RecoveryError(f"sealed position ECEF norm is outside earth range: {path}:{row.source_line}")
        _guard_coordinate(latitude, longitude, ())
        if row.timestamp_ms in result:
            raise RecoveryError(f"duplicate sealed position timestamp: {path}:{row.timestamp_ms}")
        result[row.timestamp_ms] = PositionSample(
            timestamp_ms=row.timestamp_ms,
            ecef=ecef,
            latitude=latitude,
            longitude=longitude,
            height=height,
            source=source,
        )
    if not result:
        raise RecoveryError(f"sealed position artifact is empty: {path}")
    return result


def _load_v2_sources(v2_root: Path) -> tuple[dict[str, dict[str, dict[int, PositionSample]]], list[dict[str, Any]]]:
    run_path = v2_root / "test_batch_run_manifest.json"
    if _sha256(run_path) != V2_RUN_SHA256:
        raise RecoveryError("native-FGO v2 run manifest hash differs")
    run = _load_json(run_path, "native-FGO v2 run manifest")
    if run.get("status") != "completed-truth-free-test-batch":
        raise RecoveryError("native-FGO v2 run is not a completed truth-free batch")
    if run.get("algorithm_parameter_hash") != FGO_RECIPE_HASH or run.get("algorithm_core_hash") != FGO_CORE_HASH:
        raise RecoveryError("native-FGO v2 algorithm hash differs")
    truth = run.get("truth_policy")
    if truth != {"truth_open_count": 0, "truth_materialized_count": 0, "ground_truth_members_read": False}:
        raise RecoveryError("native-FGO v2 truth policy is not closed")
    route_contract = run.get("routes")
    entries = route_contract.get("route_manifests") if isinstance(route_contract, dict) else None
    if not isinstance(entries, list) or len(entries) != EXPECTED_ROUTE_COUNT:
        raise RecoveryError("native-FGO v2 route manifest count differs")
    by_trip: dict[str, dict[str, dict[int, PositionSample]]] = {}
    route_artifacts: list[dict[str, Any]] = []
    for entry in entries:
        if not isinstance(entry, dict) or not isinstance(entry.get("path"), str):
            raise RecoveryError("native-FGO v2 route manifest entry is malformed")
        manifest_path = _repo_path(entry["path"])
        if entry.get("sha256") != _sha256(manifest_path):
            raise RecoveryError(f"native-FGO v2 route manifest hash differs: {manifest_path}")
        manifest = _load_json(manifest_path, "native-FGO v2 route manifest")
        dataset_id = manifest.get("dataset_id")
        lane = manifest.get("lane")
        if not isinstance(dataset_id, str) or dataset_id.count("/") != 1:
            raise RecoveryError("native-FGO v2 dataset identity is invalid")
        if manifest.get("truth_free") is not True or manifest.get("truth_used") is not False:
            raise RecoveryError(f"native-FGO v2 route touched truth: {dataset_id}")
        if manifest.get("algorithm_parameter_hash") != FGO_RECIPE_HASH or manifest.get("algorithm_core_hash") != FGO_CORE_HASH:
            raise RecoveryError(f"native-FGO v2 route algorithm hash differs: {dataset_id}")
        cache_key = manifest.get("cache_key")
        selected = manifest.get("selected_position")
        if not isinstance(cache_key, str) or not isinstance(selected, dict):
            raise RecoveryError(f"native-FGO v2 selected artifact is incomplete: {dataset_id}")
        relative = "fgo/fgo.pos" if lane == "native_fgo" else "wls_fallback/wls.pos" if lane == "wls_fallback" else None
        if relative is None or selected.get("path") != relative:
            raise RecoveryError(f"native-FGO v2 lane/position contract differs: {dataset_id}")
        cache_path = v2_root / "cache" / cache_key / relative
        _verify_artifact(cache_path, selected, f"native-FGO v2 selected position {dataset_id}")
        source_name = "native_fgo" if lane == "native_fgo" else "wls"
        by_trip[dataset_id] = {source_name: _position_samples(cache_path, source_name)}
        route_artifacts.append(
            {
                "dataset_id": dataset_id,
                "lane": lane,
                "route_manifest": _artifact(manifest_path),
                "selected_position": _artifact(cache_path, name=f"cache/{cache_key}/{relative}"),
            }
        )
    if len(by_trip) != EXPECTED_ROUTE_COUNT:
        raise RecoveryError("native-FGO v2 dataset count differs")
    return by_trip, route_artifacts


def _load_prior_wls_sources(old_root: Path) -> tuple[dict[str, dict[int, PositionSample]], list[dict[str, Any]]]:
    run_path = old_root / "test_batch_derived_run_manifest.json"
    if _sha256(run_path) != OLD_WLS_RUN_SHA256:
        raise RecoveryError("prior WLS run manifest hash differs")
    run = _load_json(run_path, "prior WLS run manifest")
    if run.get("truth_policy", {}).get("truth_open_count") != 0 or run.get("truth_policy", {}).get("ground_truth_members_read") is not False:
        raise RecoveryError("prior WLS run is not truth-free")
    route_contract = run.get("routes")
    entries = route_contract.get("route_manifests") if isinstance(route_contract, dict) else None
    if not isinstance(entries, list) or len(entries) != 37:
        raise RecoveryError("prior WLS successful route count differs")
    by_trip: dict[str, dict[int, PositionSample]] = {}
    artifacts: list[dict[str, Any]] = []
    for entry in entries:
        if not isinstance(entry, dict) or not isinstance(entry.get("path"), str):
            raise RecoveryError("prior WLS route manifest entry is malformed")
        manifest_path = _repo_path(entry["path"])
        if entry.get("sha256") != _sha256(manifest_path):
            raise RecoveryError(f"prior WLS route manifest hash differs: {manifest_path}")
        manifest = _load_json(manifest_path, "prior WLS route manifest")
        route = manifest.get("route")
        outputs = manifest.get("outputs")
        if not isinstance(route, str) or not isinstance(outputs, dict) or len(outputs) != 1:
            raise RecoveryError(f"prior WLS route manifest is incomplete: {manifest_path}")
        phone, output = next(iter(outputs.items()))
        if not isinstance(phone, str) or not isinstance(output, dict):
            raise RecoveryError(f"prior WLS phone output is malformed: {manifest_path}")
        selected = output.get("selected_position")
        if not isinstance(selected, dict) or not isinstance(selected.get("path"), str):
            raise RecoveryError(f"prior WLS selected output is missing: {route}/{phone}")
        position_path = _repo_path(selected["path"])
        _verify_artifact(position_path, selected, f"prior WLS selected position {route}/{phone}")
        dataset_id = f"{route}/{phone}"
        if dataset_id in by_trip:
            raise RecoveryError(f"prior WLS dataset is duplicated: {dataset_id}")
        by_trip[dataset_id] = _position_samples(position_path, "prior_wls")
        artifacts.append(
            {
                "dataset_id": dataset_id,
                "route_manifest": _artifact(manifest_path),
                "selected_position": _artifact(position_path),
            }
        )
    return by_trip, artifacts


def _merge_sources(
    source_maps: Mapping[str, Mapping[int, PositionSample]],
) -> dict[int, PositionSample]:
    merged: dict[int, PositionSample] = {}
    for name in ("native_fgo", "wls", "prior_wls"):
        for timestamp, sample in source_maps.get(name, {}).items():
            merged.setdefault(timestamp, sample)
    return merged


def _interpolate_same_trip(
    trip_id: str,
    timestamp_ms: int,
    sources_by_trip: Mapping[str, Mapping[str, Mapping[int, PositionSample]]],
) -> PositionSample | None:
    """Interpolate only within one trip and only over finite sealed positions."""

    source_maps = sources_by_trip.get(trip_id)
    if source_maps is None:
        return None
    merged = _merge_sources(source_maps)
    ordered = sorted(merged)
    before = max((value for value in ordered if value < timestamp_ms), default=None)
    after = min((value for value in ordered if value > timestamp_ms), default=None)
    if before is None or after is None or after - before > MAX_INTERPOLATION_GAP_MS:
        return None
    first = merged[before]
    second = merged[after]
    fraction = (timestamp_ms - before) / float(after - before)
    ecef = tuple((1.0 - fraction) * first.ecef[i] + fraction * second.ecef[i] for i in range(3))
    try:
        latitude_rad, longitude_rad, height = smoother._wgs84_ecef_to_geodetic(
            smoother.np.asarray(ecef, dtype=float)
        )
    except (OSError, ValueError, TypeError) as exc:
        raise RecoveryError(f"same-trip interpolation failed: {trip_id}/{timestamp_ms}") from exc
    latitude = math.degrees(latitude_rad)
    longitude = math.degrees(longitude_rad)
    return PositionSample(
        timestamp_ms=timestamp_ms,
        ecef=tuple(float(value) for value in ecef),
        latitude=float(latitude),
        longitude=float(longitude),
        height=float(height),
        source="same_trip_ecef_interpolation",
    )


def _edge_hold_same_trip(
    trip_id: str,
    timestamp_ms: int,
    sources_by_trip: Mapping[str, Mapping[str, Mapping[int, PositionSample]]],
) -> PositionSample | None:
    source_maps = sources_by_trip.get(trip_id)
    if source_maps is None:
        return None
    merged = _merge_sources(source_maps)
    if not merged:
        return None
    ordered = sorted(merged)
    if timestamp_ms < ordered[0]:
        nearest = merged[ordered[0]]
        gap = ordered[0] - timestamp_ms
    elif timestamp_ms > ordered[-1]:
        nearest = merged[ordered[-1]]
        gap = timestamp_ms - ordered[-1]
    else:
        return None
    if gap > MAX_EDGE_HOLD_GAP_MS:
        return None
    return PositionSample(
        timestamp_ms=timestamp_ms,
        ecef=nearest.ecef,
        latitude=nearest.latitude,
        longitude=nearest.longitude,
        height=nearest.height,
        source="same_trip_edge_hold",
    )


def _select_position(
    trip_id: str,
    timestamp_ms: int,
    sources_by_trip: Mapping[str, Mapping[str, Mapping[int, PositionSample]]],
) -> PositionSample:
    """Select exact native/WLS first, then same-trip bounded recovery."""

    source_maps = sources_by_trip.get(trip_id)
    if source_maps is None:
        raise RecoveryError(f"no sealed source for trip: {trip_id}")
    for name, label in (
        ("native_fgo", "native_fgo_exact"),
        ("wls", "wls_exact"),
        ("prior_wls", "prior_wls_exact"),
    ):
        candidate = source_maps.get(name, {}).get(timestamp_ms)
        if candidate is not None:
            return PositionSample(
                timestamp_ms=timestamp_ms,
                ecef=candidate.ecef,
                latitude=candidate.latitude,
                longitude=candidate.longitude,
                height=candidate.height,
                source=label,
            )
    candidate = _interpolate_same_trip(trip_id, timestamp_ms, sources_by_trip)
    if candidate is not None:
        return candidate
    candidate = _edge_hold_same_trip(trip_id, timestamp_ms, sources_by_trip)
    if candidate is not None:
        return candidate
    raise RecoveryError(
        f"unresolved key has no exact, bounded same-trip interpolation, or edge hold: {trip_id}/{timestamp_ms}"
    )


def _load_prior_fallback_keys(sample: dict[str, Any], reconciliation_path: Path) -> set[tuple[str, int]]:
    if _sha256(reconciliation_path) != OLD_RECONCILIATION_SHA256:
        raise RecoveryError("prior reconciliation manifest hash differs")
    reconciliation = _load_json(reconciliation_path, "prior sample reconciliation manifest")
    analysis = reconciliation.get("route_analysis")
    if not isinstance(analysis, dict):
        raise RecoveryError("prior reconciliation route analysis is missing")
    routes = {str(route) for route, values in analysis.items() if isinstance(values, dict) and values.get("sample_fallback", 0) > 0}
    if len(routes) != EXPECTED_PRIOR_SAMPLE_FALLBACK_KEYS:
        raise RecoveryError(f"prior sample fallback route count differs: {len(routes)}")
    result: set[tuple[str, int]] = set()
    for route in sorted(routes):
        first = next(
            (row for row in sample["rows"] if row["trip_id"].split("/", 1)[0] == route),
            None,
        )
        if first is None:
            raise RecoveryError(f"prior sample fallback route is absent from key order: {route}")
        result.add((first["trip_id"], first["timestamp"]))
    if len(result) != EXPECTED_PRIOR_SAMPLE_FALLBACK_KEYS:
        raise RecoveryError("prior sample fallback keys cannot be reconstructed from key order")
    return result


def _submission_bytes(rows: Sequence[dict[str, Any]], selected: Mapping[tuple[str, int], PositionSample]) -> tuple[bytes, dict[str, Any]]:
    buffer = io.StringIO(newline="")
    writer = csv.writer(buffer, lineterminator="\n")
    writer.writerow(SAMPLE_FIELDS)
    counts: Counter[str] = Counter()
    route_counts: dict[str, Counter[str]] = {}
    route_estimates: dict[str, list[tuple[float, float]]] = {}
    for row in rows:
        key = (row["trip_id"], row["timestamp"])
        candidate = selected.get(key)
        if candidate is None:
            raise RecoveryError(f"submission key has no selected coordinate: {key!r}")
        route = row["trip_id"].split("/", 1)[0]
        route_estimates.setdefault(route, []).append((candidate.latitude, candidate.longitude))
    for row in rows:
        key = (row["trip_id"], row["timestamp"])
        candidate = selected[key]
        route = row["trip_id"].split("/", 1)[0]
        _guard_coordinate(candidate.latitude, candidate.longitude, route_estimates[route])
        counts[candidate.source] += 1
        route_counts.setdefault(route, Counter())[candidate.source] += 1
        writer.writerow(
            (
                row["trip_id"],
                row["timestamp_text"],
                f"{candidate.latitude:.12f}",
                f"{candidate.longitude:.12f}",
            )
        )
    return buffer.getvalue().encode("utf-8"), {
        "row_count": len(rows),
        "source_counts": dict(sorted(counts.items())),
        "route_counts": {route: dict(sorted(counter.items())) for route, counter in sorted(route_counts.items())},
        "official_sample_coordinates_used": False,
        "same_trip_only": True,
        "cross_trip_interpolation": False,
        "max_interpolation_gap_ms": MAX_INTERPOLATION_GAP_MS,
        "max_edge_hold_gap_ms": MAX_EDGE_HOLD_GAP_MS,
    }


def _verify_submission(path: Path, sample: dict[str, Any]) -> dict[str, Any]:
    expected_rows = sample["rows"]
    seen: set[tuple[str, str]] = set()
    count = 0
    route_coordinates: dict[str, list[tuple[float, float]]] = {}
    parsed: list[tuple[str, str, float, float]] = []
    try:
        with path.open(encoding="utf-8", newline="") as handle:
            reader = csv.DictReader(handle)
            if tuple(reader.fieldnames or ()) != SAMPLE_FIELDS:
                raise RecoveryError("recovered submission header differs")
            for expected, raw in zip(expected_rows, reader):
                count += 1
                if None in raw:
                    raise RecoveryError("recovered submission has extra fields")
                key = (raw.get("tripId") or "", raw.get("UnixTimeMillis") or "")
                expected_key = (expected["trip_id"], expected["timestamp_text"])
                if key != expected_key:
                    raise RecoveryError(f"recovered submission key/order differs at row {count}")
                if key in seen:
                    raise RecoveryError("recovered submission contains duplicate keys")
                seen.add(key)
                try:
                    latitude = float(raw.get("LatitudeDegrees") or "")
                    longitude = float(raw.get("LongitudeDegrees") or "")
                except ValueError as exc:
                    raise RecoveryError("recovered submission coordinate is invalid") from exc
                route = expected["trip_id"].split("/", 1)[0]
                parsed.append((expected["trip_id"], expected["timestamp_text"], latitude, longitude))
                route_coordinates.setdefault(route, []).append((latitude, longitude))
            if next(reader, None) is not None:
                raise RecoveryError("recovered submission contains extra rows")
    except OSError as exc:
        raise RecoveryError(f"failed to read recovered submission: {path}") from exc
    for trip_id, _, latitude, longitude in parsed:
        if not all(math.isfinite(value) for value in (latitude, longitude)):
            raise RecoveryError("recovered submission contains non-finite coordinates")
        route = trip_id.split("/", 1)[0]
        finite_estimates = tuple(
            estimate
            for estimate in route_coordinates[route]
            if not _is_known_dummy(estimate[0], estimate[1])
        )
        _guard_coordinate(latitude, longitude, finite_estimates)
    if count != EXPECTED_SAMPLE_ROWS or len(seen) != EXPECTED_SAMPLE_ROWS:
        raise RecoveryError("recovered submission row/key count differs")
    # The final independent check deliberately has no access to sample cells.
    return {
        "header_exact": True,
        "key_order_exact": True,
        "row_count": count,
        "duplicate_keys": 0,
        "missing_keys": 0,
        "extra_keys": 0,
        "nonfinite_coordinates": 0,
        "official_sample_coordinates_compared": False,
        "known_dummy_rows": 0,
    }


def _verify_freeze(record_path: Path, manifest_path: Path) -> tuple[dict[str, Any], dict[str, Any]]:
    record = _load_json(record_path, "recovery freeze record")
    manifest = _load_json(manifest_path, "recovery freeze manifest")
    if record.get("schema_version") != FREEZE_SCHEMA or record.get("status") != "sealed-before-recovery-generation":
        raise RecoveryError("recovery freeze record is not sealed")
    if manifest.get("schema_version") != FREEZE_MANIFEST_SCHEMA or manifest.get("status") != "sealed-before-recovery-generation":
        raise RecoveryError("recovery freeze manifest is not sealed")
    record_artifact = manifest.get("freeze_record")
    if not isinstance(record_artifact, dict) or record_artifact.get("sha256") != _sha256(record_path):
        raise RecoveryError("recovery freeze record hash is not pinned by manifest")
    contract = record.get("recovery_contract")
    expected_contract = {
        "sample_coordinates_forbidden": True,
        "priority": ["native_fgo_exact", "wls_exact", "prior_wls_exact", "same_trip_ecef_interpolation", "same_trip_edge_hold"],
        "max_interpolation_gap_ms": MAX_INTERPOLATION_GAP_MS,
        "max_edge_hold_gap_ms": MAX_EDGE_HOLD_GAP_MS,
        "gap_bound_rationale": "The sealed source audit has a maximum missing-key deficit of 2004 ms; 2500 ms is a fixed 496 ms safety margin and is not truth-derived tuning.",
        "same_trip_only": True,
        "cross_trip_interpolation": False,
        "edge_velocity_extrapolation": False,
        "unresolved_policy": "fail-closed",
        "known_dummy_coordinate_rejection": {
            "enabled": True,
            "latitude_degrees": KNOWN_DUMMY_LATITUDE,
            "longitude_degrees": KNOWN_DUMMY_LONGITUDE,
            "minimum_route_distance_m": KNOWN_DUMMY_MIN_ROUTE_DISTANCE_M,
        },
        "out_of_earth_source_policy": "fail-closed",
        "fgo_wls_rerun": False,
        "numeric_algorithm_change": False,
        "atomic_publish": True,
    }
    if contract != expected_contract:
        raise RecoveryError("recovery freeze contract differs")
    truth_policy = record.get("truth_policy")
    if truth_policy != {
        "truth_open_count": 0,
        "truth_materialized_count": 0,
        "ground_truth_members_read": False,
        "holdout_or_test_truth_used": False,
    }:
        raise RecoveryError("recovery freeze truth policy is not closed")
    inputs = record.get("inputs")
    if not isinstance(inputs, dict) or inputs.get("v2_native_fgo_run", {}).get("sha256") != V2_RUN_SHA256 or inputs.get("prior_truth_free_wls_run", {}).get("sha256") != OLD_WLS_RUN_SHA256:
        raise RecoveryError("recovery freeze input hashes are incomplete")
    source_files = record.get("source_files")
    if not isinstance(source_files, dict):
        raise RecoveryError("recovery freeze source hashes are missing")
    for relative, expected in source_files.items():
        if not isinstance(relative, str) or not isinstance(expected, dict):
            raise RecoveryError("recovery freeze source hash entry is malformed")
        path = _repo_path(relative)
        if expected.get("sha256") != _sha256(path):
            raise RecoveryError(f"recovery freeze source hash differs: {relative}")
    return record, manifest


def run_recovery(
    archive_path: Path,
    inventory_path: Path,
    sample_path: Path,
    v2_root: Path,
    old_wls_root: Path,
    reconciliation_path: Path,
    freeze_path: Path,
    freeze_manifest_path: Path,
    output_dir: Path,
) -> dict[str, Any]:
    started = time.perf_counter()
    if (output_dir / "submission.csv").exists():
        raise RecoveryError(f"recovery output already exists; refusing rerun: {output_dir / 'submission.csv'}")
    freeze_record, freeze_manifest = _verify_freeze(freeze_path, freeze_manifest_path)
    if _sha256(archive_path) != ARCHIVE_SHA256 or _sha256(inventory_path) != INVENTORY_SHA256:
        raise RecoveryError("archive or test inventory hash differs")
    sample = _read_sample_keys(sample_path)
    if sample["artifact"]["sha256"] != SAMPLE_SHA256:
        raise RecoveryError("official sample hash differs")
    v2_sources, v2_artifacts = _load_v2_sources(v2_root)
    prior_wls, prior_artifacts = _load_prior_wls_sources(old_wls_root)
    for dataset_id, positions in prior_wls.items():
        v2_sources.setdefault(dataset_id, {})["prior_wls"] = positions
    sample_keys = {(row["trip_id"], row["timestamp"]) for row in sample["rows"]}
    if set(v2_sources) != {row["trip_id"] for row in sample["rows"]}:
        raise RecoveryError("sealed source dataset set differs from official sample keys")
    prior_fallback_keys = _load_prior_fallback_keys(sample, reconciliation_path)
    selected: dict[tuple[str, int], PositionSample] = {}
    route_estimates: dict[str, list[tuple[float, float]]] = {}
    source_counts: Counter[str] = Counter()
    route_counts: dict[str, Counter[str]] = {}
    for row in sample["rows"]:
        key = (row["trip_id"], row["timestamp"])
        candidate = _select_position(row["trip_id"], row["timestamp"], v2_sources)
        selected[key] = candidate
        source_counts[candidate.source] += 1
        route = row["trip_id"].split("/", 1)[0]
        route_counts.setdefault(route, Counter())[candidate.source] += 1
        route_estimates.setdefault(route, []).append((candidate.latitude, candidate.longitude))
    if set(selected) != sample_keys:
        raise RecoveryError("selected key set differs from official sample key set")
    for row in sample["rows"]:
        candidate = selected[(row["trip_id"], row["timestamp"])]
        route = row["trip_id"].split("/", 1)[0]
        _guard_coordinate(candidate.latitude, candidate.longitude, route_estimates[route])
    prior_reconstructed = {
        "expected_count": EXPECTED_PRIOR_SAMPLE_FALLBACK_KEYS,
        "keys": [
            {"trip_id": trip_id, "timestamp": timestamp, "source": selected[(trip_id, timestamp)].source}
            for trip_id, timestamp in sorted(prior_fallback_keys)
        ],
        "all_recovered_without_sample_coordinates": all(
            selected[key].source != "official_sample_coordinate_fallback" for key in prior_fallback_keys
        ),
    }
    if len(prior_reconstructed["keys"]) != EXPECTED_PRIOR_SAMPLE_FALLBACK_KEYS or not prior_reconstructed["all_recovered_without_sample_coordinates"]:
        raise RecoveryError("old WLS sample-fallback keys were not reconstructed from sealed positions")
    output_bytes, contract = _submission_bytes(sample["rows"], selected)
    output_dir.mkdir(parents=True, exist_ok=True)
    submission_path = output_dir / "submission.csv"
    _atomic_bytes(submission_path, output_bytes)
    verification = _verify_submission(submission_path, sample)
    submission_manifest = {
        "schema_version": RECOVERY_SCHEMA,
        "status": "completed-truth-free-coordinate-recovery",
        "truth_free": True,
        "truth_used": False,
        "official_sample_coordinates_used": False,
        "official_sample_key_order_authority": True,
        "submission": _artifact(submission_path),
        "verification": verification,
        "contract": contract,
        "source_counts": dict(sorted(source_counts.items())),
        "prior_wls_sample_fallback_reconstruction": prior_reconstructed,
        "truth_open_count": 0,
        "external_kaggle_submission": False,
    }
    _atomic_json(output_dir / "submission.manifest.json", submission_manifest)
    route_manifest = {
        "schema_version": POSITION_MANIFEST_SCHEMA,
        "status": "completed-truth-free-coordinate-recovery",
        "sample_coordinates_used": False,
        "v2_route_artifacts": v2_artifacts,
        "prior_wls_route_artifacts": prior_artifacts,
        "route_counts": {route: dict(sorted(counts.items())) for route, counts in sorted(route_counts.items())},
        "source_counts": dict(sorted(source_counts.items())),
        "selected_key_count": len(selected),
        "known_dummy_rows": 0,
        "out_of_earth_source_rows": 0,
        "truth_open_count": 0,
    }
    _atomic_json(output_dir / "position_recovery_manifest.json", route_manifest)
    run_manifest = {
        "schema_version": RUN_MANIFEST_SCHEMA,
        "status": "completed-truth-free-coordinate-recovery",
        "freeze": {
            "record": _artifact(freeze_path),
            "manifest": _artifact(freeze_manifest_path),
        },
        "inputs": {
            "archive": _artifact(archive_path),
            "inventory": _artifact(inventory_path),
            "official_sample_keys": _artifact(sample_path),
            "v2_run_manifest": _artifact(v2_root / "test_batch_run_manifest.json"),
            "prior_wls_run_manifest": _artifact(old_wls_root / "test_batch_derived_run_manifest.json"),
            "prior_reconciliation_manifest": _artifact(reconciliation_path),
        },
        "algorithm": {
            "parameter_hash": FGO_RECIPE_HASH,
            "core_hash": FGO_CORE_HASH,
            "rerun": False,
            "numeric_changes": False,
        },
        "sample_policy": {
            "header": list(SAMPLE_FIELDS),
            "row_count": EXPECTED_SAMPLE_ROWS,
            "coordinates_read": False,
            "coordinates_used": False,
            "key_order_authority_only": True,
        },
        "recovery_contract": freeze_record["recovery_contract"],
        "source_counts": dict(sorted(source_counts.items())),
        "route_counts": {route: dict(sorted(counts.items())) for route, counts in sorted(route_counts.items())},
        "prior_wls_sample_fallback_reconstruction": prior_reconstructed,
        "artifacts": {
            "submission": _artifact(submission_path),
            "submission_manifest": _artifact(output_dir / "submission.manifest.json"),
            "position_manifest": _artifact(output_dir / "position_recovery_manifest.json"),
        },
        "truth_policy": {
            "truth_open_count": 0,
            "truth_materialized_count": 0,
            "ground_truth_members_read": False,
        },
        "production_policy": {
            "production_rtk_spp_default_changed": False,
            "development_only": True,
            "kaggle_external_submission": False,
        },
        "timing": {"wall_seconds": time.perf_counter() - started},
        "no_post_recovery_tuning": True,
    }
    _atomic_json(output_dir / "recovery_run_manifest.json", run_manifest)
    return run_manifest


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(prog="gnss_smartphone_native_fgo_test_submission_recovery")
    parser.add_argument("--archive", type=Path, default=DEFAULT_ARCHIVE)
    parser.add_argument("--inventory", type=Path, default=DEFAULT_INVENTORY)
    parser.add_argument("--sample-submission", type=Path, default=DEFAULT_SAMPLE)
    parser.add_argument("--v2-root", type=Path, default=DEFAULT_V2_ROOT)
    parser.add_argument("--old-wls-root", type=Path, default=DEFAULT_OLD_WLS_ROOT)
    parser.add_argument("--reconciliation", type=Path, default=DEFAULT_RECONCILIATION)
    parser.add_argument("--freeze", type=Path, default=DEFAULT_FREEZE)
    parser.add_argument("--freeze-manifest", type=Path, default=DEFAULT_FREEZE_MANIFEST)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT)
    return parser.parse_args(argv)


def _resolve(path: Path) -> Path:
    return path if path.is_absolute() else ROOT / path


def run(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    try:
        report = run_recovery(
            _resolve(args.archive),
            _resolve(args.inventory),
            _resolve(args.sample_submission),
            _resolve(args.v2_root),
            _resolve(args.old_wls_root),
            _resolve(args.reconciliation),
            _resolve(args.freeze),
            _resolve(args.freeze_manifest),
            _resolve(args.output_dir),
        )
    except (RecoveryError, OSError, ValueError, KeyError, TypeError) as exc:
        print(f"native FGO submission recovery failed: {exc}", file=sys.stderr)
        return 2
    print(
        json.dumps(
            {
                "status": report["status"],
                "source_counts": report["source_counts"],
                "submission": report["artifacts"]["submission"],
            },
            sort_keys=True,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(run())
