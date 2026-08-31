#!/usr/bin/env python3
"""Truth-free physical-continuity audit for the sealed v3 test artifact.

This is a bounded post-process over the already sealed v3 native-FGO/WLS
positions.  It never opens a truth member and never starts FGO or WLS.  A
point is replaceable only when it is an interior, finite, same-trip isolated
two-sided spike: both incident ECEF speeds are strictly above the frozen
70 m/s receiver bound while the direct neighbor-to-neighbor speed is at most
that bound.  Adjacent candidate spikes are left untouched and reported.

The v3 submission is an input and is never overwritten.  A separate output is
published atomically, and an existing output submission refuses rerun.
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
from typing import Any, Mapping, Sequence

from support.gnss_runtime import application_root


ROOT = application_root(__file__)
BENCHMARK_DIR = ROOT / "apps" / "commands" / "benchmarks"
if str(BENCHMARK_DIR) not in sys.path:
    sys.path.insert(0, str(BENCHMARK_DIR))

import gnss_smartphone_native_fgo_test_submission_recovery as recovery  # noqa: E402
import gnss_smartphone_trajectory_smoother as smoother  # noqa: E402


SAMPLE_FIELDS = (
    "tripId",
    "UnixTimeMillis",
    "LatitudeDegrees",
    "LongitudeDegrees",
)
SCHEMA = "smartphone-r5-gsdc2023-native-fgo-test-submission-continuity.v1"
FREEZE_SCHEMA = "smartphone-r5-gsdc2023-native-fgo-test-submission-continuity-freeze.v1"
FREEZE_MANIFEST_SCHEMA = (
    "smartphone-r5-gsdc2023-native-fgo-test-submission-continuity-freeze-manifest.v1"
)
RUN_MANIFEST_SCHEMA = (
    "smartphone-r5-gsdc2023-native-fgo-test-submission-continuity-run-manifest.v1"
)
MAX_SPEED_MPS = 70.0
MAX_GAP_MS = recovery.MAX_INTERPOLATION_GAP_MS
EXPECTED_ROWS = recovery.EXPECTED_SAMPLE_ROWS
EARTH_ECEF_NORM_MIN_M = recovery.EARTH_ECEF_NORM_MIN_M
EARTH_ECEF_NORM_MAX_M = recovery.EARTH_ECEF_NORM_MAX_M

DEFAULT_V3_ROOT = ROOT / "output" / "smartphone-r5" / "native-fgo-test-v3-recovered"
DEFAULT_FREEZE = (
    ROOT
    / "docs"
    / "use_cases"
    / "records"
    / "smartphone_r5_gsdc2023_native_fgo_test_submission_continuity_freeze_v1.json"
)
DEFAULT_FREEZE_MANIFEST = DEFAULT_FREEZE.with_name(
    "smartphone_r5_gsdc2023_native_fgo_test_submission_continuity_freeze_v1_manifest.json"
)
DEFAULT_OUTPUT = (
    ROOT / "output" / "smartphone-r5" / "native-fgo-test-v4-continuity-recovered"
)


class ContinuityError(ValueError):
    """Raised when the sealed continuity contract cannot be proven."""


@dataclass(frozen=True)
class BaselineRow:
    trip_id: str
    timestamp_ms: int
    timestamp_text: str
    latitude_text: str
    longitude_text: str


def _sha256(path: Path) -> str:
    if not path.is_file():
        raise ContinuityError(f"missing artifact: {path}")
    digest = hashlib.sha256()
    try:
        with path.open("rb") as handle:
            for chunk in iter(lambda: handle.read(1024 * 1024), b""):
                digest.update(chunk)
    except OSError as exc:
        raise ContinuityError(f"failed to hash {path}: {exc}") from exc
    return digest.hexdigest()


def _load_json(path: Path, label: str) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ContinuityError(f"invalid {label}: {path}") from exc
    if not isinstance(value, dict):
        raise ContinuityError(f"{label} must be a JSON object: {path}")
    return value


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
    _atomic_bytes(path, (json.dumps(payload, indent=2, sort_keys=True) + "\n").encode("utf-8"))


def _artifact(path: Path) -> dict[str, Any]:
    return {
        "path": str(path.relative_to(ROOT)),
        "bytes": path.stat().st_size,
        "sha256": _sha256(path),
    }


def _verify_hash(path: Path, expected: str, label: str) -> None:
    actual = _sha256(path)
    if actual != expected:
        raise ContinuityError(f"{label} hash differs: {actual} != {expected}")


def _verify_freeze(freeze_path: Path, manifest_path: Path) -> tuple[dict[str, Any], dict[str, Any]]:
    record = _load_json(freeze_path, "continuity freeze record")
    manifest = _load_json(manifest_path, "continuity freeze manifest")
    if record.get("schema_version") != FREEZE_SCHEMA or record.get("status") != "sealed-before-continuity-generation":
        raise ContinuityError("continuity freeze record is not sealed")
    if manifest.get("schema_version") != FREEZE_MANIFEST_SCHEMA or manifest.get("status") != "sealed-before-continuity-generation":
        raise ContinuityError("continuity freeze manifest is not sealed")
    pinned = manifest.get("freeze_record")
    if not isinstance(pinned, dict) or pinned.get("sha256") != _sha256(freeze_path):
        raise ContinuityError("continuity freeze record hash is not pinned")
    contract = record.get("continuity_contract")
    if not isinstance(contract, dict):
        raise ContinuityError("continuity contract is missing")
    if contract.get("maximum_speed_mps") != MAX_SPEED_MPS:
        raise ContinuityError("continuity speed bound differs")
    if contract.get("interpolation_window_ms") != MAX_GAP_MS:
        raise ContinuityError("continuity interpolation window differs")
    if contract.get("same_trip_only") is not True or contract.get("cross_trip_interpolation") is not False:
        raise ContinuityError("continuity trip boundary contract differs")
    replacement = contract.get("replacement_condition")
    if not isinstance(replacement, dict) or replacement != {
        "both_adjacent_gaps_within_window": True,
        "both_incident_speeds_strictly_above_bound": True,
        "direct_neighbor_to_neighbor_speed_at_or_below_bound": True,
        "adjacent_candidate_spikes_are_not_repaired": True,
    }:
        raise ContinuityError("continuity replacement condition differs")
    replacement_method = contract.get("replacement")
    if not isinstance(replacement_method, dict) or replacement_method.get("method") != "timestamp-weighted linear interpolation in ECEF between the two neighbors":
        raise ContinuityError("continuity replacement method differs")
    if contract.get("sample_coordinates_forbidden") is not True or contract.get("unresolved_or_invalid_policy") != "fail-closed":
        raise ContinuityError("continuity sample/failure policy differs")
    if contract.get("algorithm_or_numeric_parameter_change") is not False or contract.get("fgo_wls_rerun") is not False:
        raise ContinuityError("continuity algorithm-change policy differs")
    if contract.get("atomic_publish") is not True:
        raise ContinuityError("continuity atomic contract differs")
    source_hashes = manifest.get("source_hashes")
    if not isinstance(source_hashes, dict):
        raise ContinuityError("continuity source hashes are missing")
    inputs = record.get("inputs")
    if not isinstance(inputs, dict):
        raise ContinuityError("continuity inputs are missing")
    for key, entry in inputs.items():
        if not isinstance(entry, dict) or not isinstance(entry.get("path"), str) or not isinstance(entry.get("sha256"), str):
            raise ContinuityError(f"continuity input hash entry is malformed: {key}")
        path = recovery.ROOT / entry["path"]
        _verify_hash(path, entry["sha256"], f"continuity input {key}")
    if record.get("truth_policy") != {
        "ground_truth_members_read": False,
        "truth_materialized_count": 0,
        "truth_open_count": 0,
        "holdout_or_test_truth_used": False,
    }:
        raise ContinuityError("continuity truth policy is not closed")
    return record, manifest


def _read_baseline(path: Path, sample: dict[str, Any]) -> list[BaselineRow]:
    expected = sample["rows"]
    rows: list[BaselineRow] = []
    seen: set[tuple[str, str]] = set()
    try:
        with path.open(encoding="utf-8", newline="") as handle:
            reader = csv.DictReader(handle)
            if tuple(reader.fieldnames or ()) != SAMPLE_FIELDS:
                raise ContinuityError("v3 baseline header differs")
            for expected_row, raw in zip(expected, reader):
                if None in raw:
                    raise ContinuityError("v3 baseline row has extra fields")
                key = (raw.get("tripId") or "", raw.get("UnixTimeMillis") or "")
                expected_key = (expected_row["trip_id"], expected_row["timestamp_text"])
                if key != expected_key:
                    raise ContinuityError(f"v3 baseline key order differs at {len(rows) + 1}")
                if key in seen:
                    raise ContinuityError("v3 baseline contains duplicate key")
                seen.add(key)
                latitude_text = raw.get("LatitudeDegrees") or ""
                longitude_text = raw.get("LongitudeDegrees") or ""
                try:
                    latitude = float(latitude_text)
                    longitude = float(longitude_text)
                except ValueError as exc:
                    raise ContinuityError("v3 baseline coordinate is invalid") from exc
                if not all(math.isfinite(value) for value in (latitude, longitude)):
                    raise ContinuityError("v3 baseline coordinate is non-finite")
                rows.append(
                    BaselineRow(
                        trip_id=expected_row["trip_id"],
                        timestamp_ms=expected_row["timestamp"],
                        timestamp_text=expected_row["timestamp_text"],
                        latitude_text=latitude_text,
                        longitude_text=longitude_text,
                    )
                )
            if next(reader, None) is not None:
                raise ContinuityError("v3 baseline has extra rows")
    except OSError as exc:
        raise ContinuityError(f"failed to read v3 baseline: {path}") from exc
    if len(rows) != EXPECTED_ROWS:
        raise ContinuityError(f"v3 baseline row count differs: {len(rows)}")
    return rows


def _load_sealed_source_maps() -> dict[str, dict[str, dict[int, recovery.PositionSample]]]:
    sources, _ = recovery._load_v2_sources(recovery.DEFAULT_V2_ROOT)
    prior, _ = recovery._load_prior_wls_sources(recovery.DEFAULT_OLD_WLS_ROOT)
    for dataset_id, positions in prior.items():
        sources.setdefault(dataset_id, {})["prior_wls"] = positions
    return sources


def _load_v3_points(
    baseline_rows: Sequence[BaselineRow],
    baseline_path: Path,
    v3_manifest_path: Path,
) -> list[recovery.PositionSample]:
    sources = _load_sealed_source_maps()
    v3_manifest = _load_json(v3_manifest_path, "v3 submission manifest")
    expected_counts = v3_manifest.get("source_counts")
    if not isinstance(expected_counts, dict):
        raise ContinuityError("v3 source counts are missing")
    result: list[recovery.PositionSample] = []
    actual_counts: Counter[str] = Counter()
    for row in baseline_rows:
        selected = recovery._select_position(row.trip_id, row.timestamp_ms, sources)
        expected_latitude = f"{selected.latitude:.12f}"
        expected_longitude = f"{selected.longitude:.12f}"
        if row.latitude_text != expected_latitude or row.longitude_text != expected_longitude:
            raise ContinuityError(
                f"v3 baseline differs from sealed source at {row.trip_id}/{row.timestamp_ms}"
            )
        actual_counts[selected.source] += 1
        result.append(selected)
    if dict(sorted(actual_counts.items())) != dict(sorted(expected_counts.items())):
        raise ContinuityError("v3 baseline source counts differ from sealed sources")
    if len(result) != EXPECTED_ROWS:
        raise ContinuityError("v3 source point count differs")
    return result


def _ecef_norm(point: recovery.PositionSample) -> float:
    return math.sqrt(sum(value * value for value in point.ecef))


def _edge_speed_mps(first: recovery.PositionSample, second: recovery.PositionSample) -> float:
    gap_ms = second.timestamp_ms - first.timestamp_ms
    if gap_ms <= 0:
        raise ContinuityError("position timestamps are not strictly increasing")
    distance = math.sqrt(sum((second.ecef[i] - first.ecef[i]) ** 2 for i in range(3)))
    speed = distance / (gap_ms / 1000.0)
    if not math.isfinite(speed):
        raise ContinuityError("edge speed is non-finite")
    return speed


def _candidate_details(points: Sequence[recovery.PositionSample], index: int) -> dict[str, Any] | None:
    if index <= 0 or index >= len(points) - 1:
        return None
    previous, candidate, following = points[index - 1], points[index], points[index + 1]
    previous_gap = candidate.timestamp_ms - previous.timestamp_ms
    following_gap = following.timestamp_ms - candidate.timestamp_ms
    if previous_gap <= 0 or following_gap <= 0 or previous_gap > MAX_GAP_MS or following_gap > MAX_GAP_MS:
        return None
    previous_speed = _edge_speed_mps(previous, candidate)
    following_speed = _edge_speed_mps(candidate, following)
    direct_speed = _edge_speed_mps(previous, following)
    if previous_speed <= MAX_SPEED_MPS or following_speed <= MAX_SPEED_MPS or direct_speed > MAX_SPEED_MPS:
        return None
    return {
        "index": index,
        "timestamp_ms": candidate.timestamp_ms,
        "source": candidate.source,
        "previous_source": previous.source,
        "following_source": following.source,
        "previous_gap_ms": previous_gap,
        "following_gap_ms": following_gap,
        "previous_speed_mps": previous_speed,
        "following_speed_mps": following_speed,
        "direct_neighbor_speed_mps": direct_speed,
    }


def _interpolate_point(
    previous: recovery.PositionSample,
    candidate: recovery.PositionSample,
    following: recovery.PositionSample,
) -> recovery.PositionSample:
    total = following.timestamp_ms - previous.timestamp_ms
    if total <= 0:
        raise ContinuityError("cannot interpolate non-increasing timestamps")
    fraction = (candidate.timestamp_ms - previous.timestamp_ms) / float(total)
    if not 0.0 < fraction < 1.0:
        raise ContinuityError("continuity replacement is not interior")
    ecef = tuple(
        (1.0 - fraction) * previous.ecef[i] + fraction * following.ecef[i]
        for i in range(3)
    )
    norm = math.sqrt(sum(value * value for value in ecef))
    if not all(math.isfinite(value) for value in ecef) or not EARTH_ECEF_NORM_MIN_M <= norm <= EARTH_ECEF_NORM_MAX_M:
        raise ContinuityError("continuity interpolation produced invalid ECEF")
    try:
        latitude_rad, longitude_rad, height = smoother._wgs84_ecef_to_geodetic(
            smoother.np.asarray(ecef, dtype=float)
        )
    except (OSError, TypeError, ValueError) as exc:
        raise ContinuityError("continuity ECEF-to-geodetic conversion failed") from exc
    latitude = math.degrees(latitude_rad)
    longitude = math.degrees(longitude_rad)
    recovery._guard_coordinate(
        latitude,
        longitude,
        ((previous.latitude, previous.longitude), (following.latitude, following.longitude)),
    )
    return recovery.PositionSample(
        timestamp_ms=candidate.timestamp_ms,
        ecef=tuple(float(value) for value in ecef),
        latitude=float(latitude),
        longitude=float(longitude),
        height=float(height),
        source="continuity_ecef_interpolation",
    )


def _edge_diagnostics(points: Sequence[recovery.PositionSample]) -> dict[str, Any]:
    speeds: list[float] = []
    above_edges = 0
    for first, second in zip(points, points[1:]):
        speed = _edge_speed_mps(first, second)
        speeds.append(speed)
        if speed > MAX_SPEED_MPS:
            above_edges += 1
    one_sided_points = 0
    for index in range(len(points)):
        incident: list[float] = []
        if index > 0:
            incident.append(speeds[index - 1])
        if index + 1 < len(points):
            incident.append(speeds[index])
        if sum(speed > MAX_SPEED_MPS for speed in incident) == 1:
            one_sided_points += 1
    return {
        "edge_count": len(speeds),
        "speed_above_bound_edges": above_edges,
        "one_sided_violation_points": one_sided_points,
        "max_speed_mps": max(speeds, default=0.0),
    }


def _process_trip(
    trip_id: str,
    points: Sequence[recovery.PositionSample],
) -> tuple[list[recovery.PositionSample], dict[str, Any]]:
    if not points:
        raise ContinuityError(f"empty trip: {trip_id}")
    if any(point.timestamp_ms >= following.timestamp_ms for point, following in zip(points, points[1:])):
        raise ContinuityError(f"trip timestamps are not increasing: {trip_id}")
    before = _edge_diagnostics(points)
    candidate_details = [
        detail
        for index in range(1, len(points) - 1)
        if (detail := _candidate_details(points, index)) is not None
    ]
    candidate_indices = {int(detail["index"]) for detail in candidate_details}
    blocked_indices = {
        index
        for index in candidate_indices
        if index - 1 in candidate_indices or index + 1 in candidate_indices
    }
    repair_indices = sorted(candidate_indices - blocked_indices)
    processed = list(points)
    repairs: list[dict[str, Any]] = []
    for detail in candidate_details:
        index = int(detail["index"])
        if index not in repair_indices:
            continue
        replacement = _interpolate_point(points[index - 1], points[index], points[index + 1])
        processed[index] = replacement
        repairs.append(
            {
                "timestamp_ms": replacement.timestamp_ms,
                "source_before": points[index].source,
                "source_after": replacement.source,
                "previous_speed_mps": detail["previous_speed_mps"],
                "following_speed_mps": detail["following_speed_mps"],
                "direct_neighbor_speed_mps": detail["direct_neighbor_speed_mps"],
            }
        )
    after = _edge_diagnostics(processed)
    post_candidates = [
        detail
        for index in range(1, len(processed) - 1)
        if (detail := _candidate_details(processed, index)) is not None
    ]
    post_candidate_indices = {int(detail["index"]) for detail in post_candidates}
    post_blocked_indices = {
        index
        for index in post_candidate_indices
        if index - 1 in post_candidate_indices or index + 1 in post_candidate_indices
    }
    post_isolated_candidates = [
        detail
        for detail in post_candidates
        if int(detail["index"]) not in post_blocked_indices
    ]
    if post_isolated_candidates:
        raise ContinuityError(f"isolated two-sided spikes remain after continuity pass: {trip_id}")
    return processed, {
        "trip_id": trip_id,
        "row_count": len(points),
        "candidate_spike_points": len(candidate_details),
        "repaired_points": len(repairs),
        "blocked_adjacent_candidate_points": len(blocked_indices),
        "adjacent_candidate_pairs": sum(
            1 for index in candidate_indices if index + 1 in candidate_indices
        ),
        "before": before,
        "after": after,
        "post_isolated_two_sided_spike_points": len(post_isolated_candidates),
        "post_sequential_candidate_points": len(post_blocked_indices),
        "repairs": repairs,
    }


def _submission_bytes(
    rows: Sequence[BaselineRow],
    points: Sequence[recovery.PositionSample],
) -> bytes:
    if len(rows) != len(points):
        raise ContinuityError("submission row/position count differs")
    buffer = io.StringIO(newline="")
    writer = csv.writer(buffer, lineterminator="\n")
    writer.writerow(SAMPLE_FIELDS)
    for row, point in zip(rows, points):
        writer.writerow(
            (
                row.trip_id,
                row.timestamp_text,
                f"{point.latitude:.12f}",
                f"{point.longitude:.12f}",
            )
        )
    return buffer.getvalue().encode("utf-8")


def _source_counts(points: Sequence[recovery.PositionSample]) -> dict[str, int]:
    return dict(sorted(Counter(point.source for point in points).items()))


def run_continuity(
    freeze_path: Path,
    freeze_manifest_path: Path,
    v3_root: Path,
    output_dir: Path,
) -> dict[str, Any]:
    started = time.perf_counter()
    output_submission = output_dir / "submission.csv"
    if output_submission.exists():
        raise ContinuityError(f"continuity output already exists; refusing rerun: {output_submission}")
    freeze_record, freeze_manifest = _verify_freeze(freeze_path, freeze_manifest_path)
    v3_submission = v3_root / "submission.csv"
    v3_submission_manifest = v3_root / "submission.manifest.json"
    v3_position_manifest = v3_root / "position_recovery_manifest.json"
    v3_run_manifest = v3_root / "recovery_run_manifest.json"
    input_paths = {
        "v3_submission": v3_submission,
        "v3_submission_manifest": v3_submission_manifest,
        "v3_position_manifest": v3_position_manifest,
        "v3_run_manifest": v3_run_manifest,
    }
    for name, path in input_paths.items():
        entry = freeze_record["inputs"].get(name)
        if not isinstance(entry, dict) or entry.get("path") != str(path.relative_to(ROOT)):
            raise ContinuityError(f"continuity freeze input path differs: {name}")
        _verify_hash(path, entry["sha256"], f"continuity {name}")
    sample = recovery._read_sample_keys(recovery.DEFAULT_SAMPLE)
    baseline_rows = _read_baseline(v3_submission, sample)
    baseline_points = _load_v3_points(baseline_rows, v3_submission, v3_submission_manifest)
    grouped: dict[str, list[recovery.PositionSample]] = {}
    grouped_rows: dict[str, list[BaselineRow]] = {}
    for row, point in zip(baseline_rows, baseline_points):
        grouped.setdefault(row.trip_id, []).append(point)
        grouped_rows.setdefault(row.trip_id, []).append(row)
    processed_by_trip: dict[str, list[recovery.PositionSample]] = {}
    trip_diagnostics: list[dict[str, Any]] = []
    for trip_id in sorted(grouped):
        processed, diagnosis = _process_trip(trip_id, grouped[trip_id])
        processed_by_trip[trip_id] = processed
        trip_diagnostics.append(diagnosis)
    processed_points: list[recovery.PositionSample] = []
    # Reassemble in authoritative v3 row order without relying on dictionary
    # iteration or a route-specific timestamp assumption.
    offsets: dict[str, int] = Counter()
    for row in baseline_rows:
        index = offsets[row.trip_id]
        processed_points.append(processed_by_trip[row.trip_id][index])
        offsets[row.trip_id] += 1
    if len(processed_points) != EXPECTED_ROWS:
        raise ContinuityError("processed point count differs")
    for point in processed_points:
        values = (*point.ecef, point.latitude, point.longitude, point.height)
        if not all(math.isfinite(value) for value in values):
            raise ContinuityError("processed point is non-finite")
        norm = _ecef_norm(point)
        if not EARTH_ECEF_NORM_MIN_M <= norm <= EARTH_ECEF_NORM_MAX_M:
            raise ContinuityError("processed point is outside Earth ECEF norm")
        recovery._guard_coordinate(point.latitude, point.longitude)
    output_bytes = _submission_bytes(baseline_rows, processed_points)
    output_dir.mkdir(parents=True, exist_ok=True)
    _atomic_bytes(output_submission, output_bytes)
    verification = recovery._verify_submission(output_submission, sample)
    if verification["row_count"] != EXPECTED_ROWS or verification["known_dummy_rows"] != 0:
        raise ContinuityError("continuity submission verification failed")
    post_isolated = sum(
        int(diagnosis["post_isolated_two_sided_spike_points"])
        for diagnosis in trip_diagnostics
    )
    if post_isolated != 0:
        raise ContinuityError("continuity output still has isolated two-sided spikes")
    baseline_counts = _source_counts(baseline_points)
    output_counts = _source_counts(processed_points)
    changed_rows = sum(
        int(diagnosis["repaired_points"]) for diagnosis in trip_diagnostics
    )
    manifest = {
        "schema_version": SCHEMA,
        "status": "completed-truth-free-continuity-recovery",
        "truth_free": True,
        "truth_used": False,
        "official_sample_coordinates_used": False,
        "official_sample_key_order_authority": True,
        "freeze": {
            "record": _artifact(freeze_path),
            "manifest": _artifact(freeze_manifest_path),
        },
        "inputs": {
            "v3_submission": _artifact(v3_submission),
            "v3_submission_manifest": _artifact(v3_submission_manifest),
            "v3_position_manifest": _artifact(v3_position_manifest),
            "v3_run_manifest": _artifact(v3_run_manifest),
            "official_sample_keys": _artifact(recovery.DEFAULT_SAMPLE),
        },
        "algorithm": {
            "maximum_speed_mps": MAX_SPEED_MPS,
            "interpolation_window_ms": MAX_GAP_MS,
            "same_trip_only": True,
            "cross_trip_interpolation": False,
            "algorithm_or_numeric_parameter_change": False,
            "fgo_wls_rerun": False,
            "continuity_script_sha256": _sha256(Path(__file__).resolve()),
        },
        "source_counts": {
            "v3_baseline": baseline_counts,
            "continuity_output": output_counts,
        },
        "changed_rows": changed_rows,
        "byte_identical_to_v3": output_bytes == v3_submission.read_bytes(),
        "verification": verification,
        "sample_coordinate_fallback_rows": 0,
        "known_dummy_rows": 0,
        "out_of_earth_rows": 0,
        "physical_continuity": {
            "isolated_two_sided_spike_points_after": post_isolated,
            "trip_count": len(trip_diagnostics),
            "trip_diagnostics": trip_diagnostics,
        },
        "truth_policy": {
            "ground_truth_members_read": False,
            "truth_materialized_count": 0,
            "truth_open_count": 0,
            "holdout_or_test_truth_used": False,
        },
        "publication_policy": {
            "development_only": True,
            "kaggle_external_submission": False,
            "production_rtk_spp_default_changed": False,
            "no_post_generation_tuning": True,
        },
    }
    _atomic_json(output_dir / "continuity_manifest.json", manifest)
    run_manifest = {
        "schema_version": RUN_MANIFEST_SCHEMA,
        "status": "completed-truth-free-continuity-recovery",
        "freeze": {
            "record": _artifact(freeze_path),
            "manifest": _artifact(freeze_manifest_path),
        },
        "source_artifacts": {
            "v3_submission": _artifact(v3_submission),
            "continuity_manifest": _artifact(output_dir / "continuity_manifest.json"),
            "submission": _artifact(output_submission),
        },
        "source_counts": {
            "v3_baseline": baseline_counts,
            "continuity_output": output_counts,
        },
        "changed_rows": changed_rows,
        "byte_identical_to_v3": output_bytes == v3_submission.read_bytes(),
        "truth_policy": manifest["truth_policy"],
        "physical_continuity": {
            "maximum_speed_mps": MAX_SPEED_MPS,
            "interpolation_window_ms": MAX_GAP_MS,
            "isolated_two_sided_spike_points_after": post_isolated,
            "one_sided_and_sequential_violations_reported": True,
        },
        "timing": {"wall_seconds": time.perf_counter() - started},
        "no_post_generation_tuning": True,
    }
    _atomic_json(output_dir / "continuity_run_manifest.json", run_manifest)
    return run_manifest


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(prog="gnss_smartphone_native_fgo_test_submission_continuity")
    parser.add_argument("--freeze", type=Path, default=DEFAULT_FREEZE)
    parser.add_argument("--freeze-manifest", type=Path, default=DEFAULT_FREEZE_MANIFEST)
    parser.add_argument("--v3-root", type=Path, default=DEFAULT_V3_ROOT)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT)
    return parser.parse_args(argv)


def _resolve(path: Path) -> Path:
    return path if path.is_absolute() else ROOT / path


def run(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    try:
        report = run_continuity(
            _resolve(args.freeze),
            _resolve(args.freeze_manifest),
            _resolve(args.v3_root),
            _resolve(args.output_dir),
        )
    except (ContinuityError, OSError, ValueError, KeyError, TypeError) as exc:
        print(f"native FGO continuity recovery failed: {exc}", file=sys.stderr)
        return 2
    print(
        json.dumps(
            {
                "status": report["status"],
                "changed_rows": report["changed_rows"],
                "byte_identical_to_v3": report["byte_identical_to_v3"],
                "source_counts": report["source_counts"],
            },
            sort_keys=True,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(run())
