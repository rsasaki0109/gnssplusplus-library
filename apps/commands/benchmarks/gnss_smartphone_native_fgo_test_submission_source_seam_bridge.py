#!/usr/bin/env python3
"""Truth-free source-seam bridge for the sealed v4 smartphone artifact.

This post-process never starts FGO/WLS and never reads official sample
coordinates or test truth.  It can replace only a contiguous exact-source
run whose two neighboring points are from the same trusted lane, whose
endpoint bracket is bounded, whose endpoint speed is physically bounded, and
whose seam has at least one high-speed boundary edge.  Every interior point
of an accepted run is reconstructed by timestamp-weighted ECEF interpolation.
All other seams are preserved and reported.
"""

from __future__ import annotations

import argparse
from collections import Counter, defaultdict
from dataclasses import dataclass
import hashlib
import math
import os
from pathlib import Path
import resource
import sys
import tempfile
import time
from typing import Any, Sequence


from support.gnss_runtime import application_root


ROOT = application_root(__file__)
BENCHMARK_DIR = ROOT / "apps" / "commands" / "benchmarks"
if str(BENCHMARK_DIR) not in sys.path:
    sys.path.insert(0, str(BENCHMARK_DIR))

import gnss_smartphone_native_fgo_test_submission_continuity as continuity  # noqa: E402
import gnss_smartphone_native_fgo_test_submission_recovery as recovery  # noqa: E402
import gnss_smartphone_trajectory_smoother as smoother  # noqa: E402


SAMPLE_FIELDS = ("tripId", "UnixTimeMillis", "LatitudeDegrees", "LongitudeDegrees")
SCHEMA = "smartphone-r5-gsdc2023-native-fgo-test-submission-source-seam-bridge.v1"
FREEZE_SCHEMA = "smartphone-r5-gsdc2023-native-fgo-test-submission-source-seam-bridge-freeze.v1"
FREEZE_MANIFEST_SCHEMA = (
    "smartphone-r5-gsdc2023-native-fgo-test-submission-source-seam-bridge-freeze-manifest.v1"
)
RUN_MANIFEST_SCHEMA = (
    "smartphone-r5-gsdc2023-native-fgo-test-submission-source-seam-bridge-run-manifest.v1"
)
MAX_SPEED_MPS = 70.0
MAX_BRACKET_GAP_MS = 10000
TRUSTED_LANES = ("native_fgo", "wls")
SOURCE_TO_LANE = {
    "native_fgo_exact": "native_fgo",
    "wls_exact": "wls",
    "prior_wls_exact": "prior_wls",
}
DEFAULT_V4_ROOT = ROOT / "output" / "smartphone-r5" / "native-fgo-test-v4-continuity-recovered"
DEFAULT_FREEZE = (
    ROOT
    / "docs"
    / "use_cases"
    / "records"
    / "smartphone_r5_gsdc2023_native_fgo_test_submission_source_seam_bridge_freeze_v1.json"
)
DEFAULT_FREEZE_MANIFEST = DEFAULT_FREEZE.with_name(
    "smartphone_r5_gsdc2023_native_fgo_test_submission_source_seam_bridge_freeze_v1_manifest.json"
)
DEFAULT_OUTPUT = ROOT / "output" / "smartphone-r5" / "native-fgo-test-v5-source-seam-bridge"
V3_ROOT = ROOT / "output" / "smartphone-r5" / "native-fgo-test-v3-recovered"


class SourceSeamBridgeError(ValueError):
    """Raised when a sealed source-seam contract cannot be proven."""


def _sha256(path: Path) -> str:
    if not path.is_file():
        raise SourceSeamBridgeError(f"missing artifact: {path}")
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _load_json(path: Path, label: str) -> dict[str, Any]:
    try:
        import json

        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError) as exc:
        raise SourceSeamBridgeError(f"invalid {label}: {path}") from exc
    if not isinstance(value, dict):
        raise SourceSeamBridgeError(f"{label} must be a JSON object: {path}")
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


def _atomic_json(path: Path, value: dict[str, Any]) -> None:
    import json

    _atomic_bytes(path, (json.dumps(value, indent=2, sort_keys=True) + "\n").encode("utf-8"))


def _artifact(path: Path) -> dict[str, Any]:
    return {
        "path": str(path.relative_to(ROOT)),
        "bytes": path.stat().st_size,
        "sha256": _sha256(path),
    }


def _verify_hash(path: Path, expected: str, label: str) -> None:
    actual = _sha256(path)
    if actual != expected:
        raise SourceSeamBridgeError(f"{label} hash differs: {actual} != {expected}")


def _verify_freeze(freeze_path: Path, freeze_manifest_path: Path) -> dict[str, Any]:
    record = _load_json(freeze_path, "source-seam bridge freeze record")
    manifest = _load_json(freeze_manifest_path, "source-seam bridge freeze manifest")
    if record.get("schema_version") != FREEZE_SCHEMA or record.get("status") != "sealed-before-source-seam-bridge-decision":
        raise SourceSeamBridgeError("source-seam bridge freeze record is not sealed")
    if manifest.get("schema_version") != FREEZE_MANIFEST_SCHEMA or manifest.get("status") != "sealed-before-source-seam-bridge-decision":
        raise SourceSeamBridgeError("source-seam bridge freeze manifest is not sealed")
    pinned = manifest.get("freeze_record")
    if not isinstance(pinned, dict) or pinned.get("sha256") != _sha256(freeze_path):
        raise SourceSeamBridgeError("freeze record hash is not pinned")
    contract = record.get("bridge_contract")
    if not isinstance(contract, dict):
        raise SourceSeamBridgeError("bridge contract is missing")
    expected = {
        "same_trip_only": True,
        "cross_trip_interpolation": False,
        "source_seam_or_gap_only": True,
        "high_speed_boundary_trigger": "At least one endpoint-to-adjacent-seam transition must exceed the frozen 70 m/s bound; otherwise the source seam is preserved byte-for-byte.",
        "trusted_lanes": ["native_fgo", "wls"],
        "both_endpoints_same_trusted_lane": True,
        "maximum_bracket_gap_ms": MAX_BRACKET_GAP_MS,
        "endpoint_speed_metric": "finite ECEF Euclidean displacement divided by elapsed seconds",
        "maximum_endpoint_speed_mps": MAX_SPEED_MPS,
        "finite_ecef_required": True,
        "earth_ecef_norm_range_m": [6000000.0, 7500000.0],
        "known_sample_coordinates_forbidden": True,
        "dummy_or_sample_coordinate_rows_must_equal": 0,
        "replace_entire_interior_seam": True,
        "interior_only": True,
        "replacement": "timestamp-weighted ECEF linear interpolation between same-lane bracketing endpoints, followed by WGS84 conversion",
        "no_velocity_extrapolation": True,
        "no_truth_selection": True,
        "fgo_wls_rerun": False,
        "numeric_parameter_change": False,
        "invalid_nonfinite_or_unresolved_policy": "fail-closed",
        "atomic_publish_if_applicable": True,
        "no_v5_if_no_eligible_bridge": True,
    }
    if contract != expected:
        raise SourceSeamBridgeError("source-seam bridge contract differs from freeze")
    if record.get("truth_policy") != {
        "ground_truth_members_read": False,
        "truth_materialized_count": 0,
        "truth_open_count": 0,
        "holdout_or_test_truth_used": False,
        "leaderboard_used": False,
    }:
        raise SourceSeamBridgeError("freeze truth policy is not closed")
    inputs = record.get("inputs")
    if not isinstance(inputs, dict):
        raise SourceSeamBridgeError("freeze inputs are missing")
    for name, entry in inputs.items():
        if not isinstance(entry, dict) or not isinstance(entry.get("path"), str) or not isinstance(entry.get("sha256"), str):
            raise SourceSeamBridgeError(f"freeze input is malformed: {name}")
        path = ROOT / entry["path"]
        _verify_hash(path, entry["sha256"], f"freeze input {name}")
    return record


def _source_lane(source: str) -> str | None:
    return SOURCE_TO_LANE.get(source)


def _interpolate(
    before: recovery.PositionSample,
    point: recovery.PositionSample,
    after: recovery.PositionSample,
) -> recovery.PositionSample:
    total = after.timestamp_ms - before.timestamp_ms
    if total <= 0 or not before.timestamp_ms < point.timestamp_ms < after.timestamp_ms:
        raise SourceSeamBridgeError("source-seam interpolation is not interior")
    fraction = (point.timestamp_ms - before.timestamp_ms) / float(total)
    ecef = tuple(
        (1.0 - fraction) * before.ecef[index] + fraction * after.ecef[index]
        for index in range(3)
    )
    if not all(math.isfinite(value) for value in ecef):
        raise SourceSeamBridgeError("source-seam interpolation is non-finite")
    norm = math.sqrt(sum(value * value for value in ecef))
    if not recovery.EARTH_ECEF_NORM_MIN_M <= norm <= recovery.EARTH_ECEF_NORM_MAX_M:
        raise SourceSeamBridgeError("source-seam interpolation is outside Earth ECEF norm")
    try:
        latitude_rad, longitude_rad, height = smoother._wgs84_ecef_to_geodetic(
            smoother.np.asarray(ecef, dtype=float)
        )
    except (OSError, TypeError, ValueError) as exc:
        raise SourceSeamBridgeError("source-seam geodetic conversion failed") from exc
    latitude = math.degrees(latitude_rad)
    longitude = math.degrees(longitude_rad)
    recovery._guard_coordinate(
        latitude,
        longitude,
        ((before.latitude, before.longitude), (after.latitude, after.longitude)),
    )
    if recovery._is_known_dummy(latitude, longitude):
        raise SourceSeamBridgeError("source-seam interpolation emitted known dummy")
    return recovery.PositionSample(
        timestamp_ms=point.timestamp_ms,
        ecef=tuple(float(value) for value in ecef),
        latitude=float(latitude),
        longitude=float(longitude),
        height=float(height),
        source="source_seam_ecef_interpolation",
    )


@dataclass(frozen=True)
class SeamCandidate:
    trip_id: str
    start_index: int
    end_index: int
    run_lane: str
    trusted_lane: str
    run_start_timestamp_ms: int
    run_end_timestamp_ms: int
    before: recovery.PositionSample
    after: recovery.PositionSample
    left_speed_mps: float
    right_speed_mps: float
    bracket_gap_ms: int
    endpoint_speed_mps: float


def _candidate_run(
    trip_id: str,
    points: Sequence[recovery.PositionSample],
    start: int,
    end: int,
) -> SeamCandidate | None:
    if start <= 0 or end >= len(points):
        return None
    run_lane = _source_lane(points[start].source)
    before_lane = _source_lane(points[start - 1].source)
    after_lane = _source_lane(points[end].source)
    if (
        run_lane is None
        or before_lane not in TRUSTED_LANES
        or before_lane != after_lane
        or run_lane == before_lane
    ):
        return None
    before = points[start - 1]
    after = points[end]
    left_speed = continuity._edge_speed_mps(before, points[start])
    right_speed = continuity._edge_speed_mps(points[end - 1], after)
    bracket_gap = after.timestamp_ms - before.timestamp_ms
    endpoint_speed = continuity._edge_speed_mps(before, after)
    return SeamCandidate(
        trip_id=trip_id,
        start_index=start,
        end_index=end,
        run_lane=run_lane,
        trusted_lane=before_lane,
        run_start_timestamp_ms=points[start].timestamp_ms,
        run_end_timestamp_ms=points[end - 1].timestamp_ms,
        before=before,
        after=after,
        left_speed_mps=left_speed,
        right_speed_mps=right_speed,
        bracket_gap_ms=bracket_gap,
        endpoint_speed_mps=endpoint_speed,
    )


def _candidate_dict(candidate: SeamCandidate, *, eligible: bool, reason: str) -> dict[str, Any]:
    return {
        "trip_id": candidate.trip_id,
        "run_lane": candidate.run_lane,
        "trusted_lane": candidate.trusted_lane,
        "run_count": candidate.end_index - candidate.start_index,
        "run_start_timestamp_ms": candidate.run_start_timestamp_ms,
        "run_end_timestamp_ms": candidate.run_end_timestamp_ms,
        "before_timestamp_ms": candidate.before.timestamp_ms,
        "after_timestamp_ms": candidate.after.timestamp_ms,
        "left_boundary_speed_mps": candidate.left_speed_mps,
        "right_boundary_speed_mps": candidate.right_speed_mps,
        "bracket_gap_ms": candidate.bracket_gap_ms,
        "endpoint_speed_mps": candidate.endpoint_speed_mps,
        "high_speed_boundary_trigger": max(candidate.left_speed_mps, candidate.right_speed_mps) > MAX_SPEED_MPS,
        "eligible": eligible,
        "reason": reason,
    }


def _bridge_trips(
    points_by_trip: dict[str, list[recovery.PositionSample]],
) -> tuple[dict[str, list[recovery.PositionSample]], dict[str, Any]]:
    output = {trip: list(points) for trip, points in points_by_trip.items()}
    audits: list[dict[str, Any]] = []
    eligible: list[SeamCandidate] = []
    seam_run_count = 0
    high_speed_seam_run_count = 0
    skipped_nontrigger_count = 0
    for trip_id, points in sorted(points_by_trip.items()):
        lanes = [_source_lane(point.source) for point in points]
        start = 0
        while start < len(points):
            end = start + 1
            while end < len(points) and lanes[end] == lanes[start]:
                end += 1
            candidate = _candidate_run(trip_id, points, start, end)
            if candidate is not None:
                seam_run_count += 1
                high_trigger = max(candidate.left_speed_mps, candidate.right_speed_mps) > MAX_SPEED_MPS
                if not high_trigger:
                    skipped_nontrigger_count += 1
                else:
                    high_speed_seam_run_count += 1
                    eligible_now = (
                        candidate.bracket_gap_ms > 0
                        and candidate.bracket_gap_ms <= MAX_BRACKET_GAP_MS
                        and candidate.endpoint_speed_mps <= MAX_SPEED_MPS
                    )
                    if eligible_now:
                        eligible.append(candidate)
                    else:
                        reason = (
                            "bracket gap exceeds 10 seconds"
                            if candidate.bracket_gap_ms > MAX_BRACKET_GAP_MS
                            else "same-lane endpoint speed exceeds 70 m/s"
                        )
                        audits.append(_candidate_dict(candidate, eligible=False, reason=reason))
            start = end
    for candidate in eligible:
        points = points_by_trip[candidate.trip_id]
        replacements: list[int] = []
        for index in range(candidate.start_index, candidate.end_index):
            output[candidate.trip_id][index] = _interpolate(
                candidate.before, points[index], candidate.after
            )
            replacements.append(points[index].timestamp_ms)
        entry = _candidate_dict(candidate, eligible=True, reason="all frozen bridge conditions passed")
        entry["replaced_timestamps_ms"] = replacements
        audits.append(entry)
    audits.sort(key=lambda item: (item["trip_id"], item["before_timestamp_ms"], item["run_lane"]))
    return output, {
        "seam_run_count": seam_run_count,
        "high_speed_seam_run_count": high_speed_seam_run_count,
        "high_speed_seam_audits": audits,
        "eligible_run_count": len(eligible),
        "replaced_row_count": sum(candidate.end_index - candidate.start_index for candidate in eligible),
        "nontrigger_seams_preserved": skipped_nontrigger_count,
    }


def _isolated_candidate_count(points_by_trip: dict[str, Sequence[recovery.PositionSample]]) -> int:
    count = 0
    for points in points_by_trip.values():
        details = [
            continuity._candidate_details(points, index)
            for index in range(1, len(points) - 1)
        ]
        details = [detail for detail in details if detail is not None]
        indices = {int(detail["index"]) for detail in details}
        count += sum(
            int(detail["index"]) - 1 not in indices
            and int(detail["index"]) + 1 not in indices
            for detail in details
        )
    return count


def _aggregate_edge_diagnostics(
    points_by_trip: dict[str, Sequence[recovery.PositionSample]],
) -> dict[str, Any]:
    values = [continuity._edge_diagnostics(points) for points in points_by_trip.values()]
    return {
        "trip_count": len(values),
        "edge_count": sum(int(value["edge_count"]) for value in values),
        "speed_above_bound_edges": sum(int(value["speed_above_bound_edges"]) for value in values),
        "one_sided_violation_points": sum(int(value["one_sided_violation_points"]) for value in values),
        "max_speed_mps": max((float(value["max_speed_mps"]) for value in values), default=0.0),
    }


def _load_v4_points(v4_root: Path, sample: dict[str, Any]) -> tuple[list[dict[str, str]], dict[str, list[recovery.PositionSample]], dict[str, Any]]:
    v3_submission = V3_ROOT / "submission.csv"
    v3_manifest = V3_ROOT / "submission.manifest.json"
    rows = continuity._read_baseline(v3_submission, sample)
    v3_points = continuity._load_v3_points(rows, v3_submission, v3_manifest)
    grouped: dict[str, list[recovery.PositionSample]] = defaultdict(list)
    for row, point in zip(rows, v3_points):
        grouped[row.trip_id].append(point)
    processed_by_trip = {
        trip_id: continuity._process_trip(trip_id, points)[0]
        for trip_id, points in grouped.items()
    }
    v4_path = v4_root / "submission.csv"
    offsets: Counter[str] = Counter()
    v4_points: list[recovery.PositionSample] = []
    for row in rows:
        v4_points.append(processed_by_trip[row.trip_id][offsets[row.trip_id]])
        offsets[row.trip_id] += 1
    expected_bytes = continuity._submission_bytes(
        rows,
        v4_points,
    )
    if v4_path.read_bytes() != expected_bytes:
        raise SourceSeamBridgeError("v4 submission differs from sealed continuity reconstruction")
    verification = recovery._verify_submission(v4_path, sample)
    if verification["row_count"] != recovery.EXPECTED_SAMPLE_ROWS:
        raise SourceSeamBridgeError("v4 row count differs")
    return rows, processed_by_trip, verification


def run_bridge(
    freeze_path: Path = DEFAULT_FREEZE,
    freeze_manifest_path: Path = DEFAULT_FREEZE_MANIFEST,
    v4_root: Path = DEFAULT_V4_ROOT,
    output_dir: Path = DEFAULT_OUTPUT,
) -> dict[str, Any]:
    started = time.perf_counter()
    output_submission = output_dir / "submission.csv"
    if output_submission.exists():
        raise SourceSeamBridgeError(f"source-seam output already exists; refusing rerun: {output_submission}")
    freeze = _verify_freeze(freeze_path, freeze_manifest_path)
    sample = recovery._read_sample_keys(recovery.DEFAULT_SAMPLE)
    rows, points_by_trip, v4_verification = _load_v4_points(v4_root, sample)
    # Verify the exact frozen v4 hash after independently reconstructing it.
    v4_entry = freeze["inputs"]["v4_submission"]
    _verify_hash(v4_root / "submission.csv", v4_entry["sha256"], "v4 submission")
    before = _aggregate_edge_diagnostics(points_by_trip)
    output_points_by_trip, seam_report = _bridge_trips(points_by_trip)
    after = _aggregate_edge_diagnostics(output_points_by_trip)
    isolated_after = _isolated_candidate_count(output_points_by_trip)
    if isolated_after != 0:
        raise SourceSeamBridgeError(f"isolated continuity spikes remain after bridge: {isolated_after}")
    flat_points: list[recovery.PositionSample] = []
    offsets: Counter[str] = Counter()
    for row in rows:
        flat_points.append(output_points_by_trip[row.trip_id][offsets[row.trip_id]])
        offsets[row.trip_id] += 1
    output_bytes = continuity._submission_bytes(rows, flat_points)
    _atomic_bytes(output_submission, output_bytes)
    verification = recovery._verify_submission(output_submission, sample)
    if verification["row_count"] != recovery.EXPECTED_SAMPLE_ROWS or verification["known_dummy_rows"] != 0:
        raise SourceSeamBridgeError("source-seam output key/coordinate verification failed")
    for point in flat_points:
        values = (*point.ecef, point.latitude, point.longitude, point.height)
        if not all(math.isfinite(value) for value in values):
            raise SourceSeamBridgeError("source-seam output contains non-finite values")
        norm = math.sqrt(sum(value * value for value in point.ecef))
        if not recovery.EARTH_ECEF_NORM_MIN_M <= norm <= recovery.EARTH_ECEF_NORM_MAX_M:
            raise SourceSeamBridgeError("source-seam output is outside Earth ECEF norm")
    output_counts = dict(sorted(Counter(point.source for point in flat_points).items()))
    before_counts = dict(sorted(Counter(point.source for points in points_by_trip.values() for point in points).items()))
    manifest = {
        "schema_version": SCHEMA,
        "status": "completed-truth-free-source-seam-bridge",
        "truth_free": True,
        "truth_used": False,
        "official_sample_coordinates_used": False,
        "official_sample_key_order_authority": True,
        "freeze": {
            "record": _artifact(freeze_path),
            "manifest": _artifact(freeze_manifest_path),
        },
        "inputs": {
            "v4_submission": _artifact(v4_root / "submission.csv"),
            "v4_continuity_manifest": _artifact(v4_root / "continuity_manifest.json"),
            "v4_run_manifest": _artifact(v4_root / "continuity_run_manifest.json"),
            "official_sample_keys": _artifact(recovery.DEFAULT_SAMPLE),
        },
        "algorithm": {
            "script_sha256": _sha256(Path(__file__).resolve()),
            "maximum_endpoint_speed_mps": MAX_SPEED_MPS,
            "maximum_bracket_gap_ms": MAX_BRACKET_GAP_MS,
            "trusted_lanes": ["native_fgo", "wls"],
            "same_trip_only": True,
            "cross_trip_interpolation": False,
            "high_speed_boundary_trigger": True,
            "replace_entire_interior_seam": True,
            "timestamp_weighted_ecef_interpolation": True,
            "fgo_wls_rerun": False,
            "numeric_parameter_change": False,
        },
        "source_counts": {
            "v4_baseline": before_counts,
            "v5_output": output_counts,
        },
        "source_seam_bridge": seam_report,
        "physical_continuity": {
            "before": before,
            "after": after,
            "isolated_two_sided_spike_points_before": _isolated_candidate_count(points_by_trip),
            "isolated_two_sided_spike_points_after": isolated_after,
            "remaining_speed_above_bound_edges_reported": True,
        },
        "verification": {
            "v4": v4_verification,
            "v5": verification,
            "v4_submission_sha256": _sha256(v4_root / "submission.csv"),
            "v5_submission": _artifact(output_submission),
            "row_count": recovery.EXPECTED_SAMPLE_ROWS,
            "duplicate_keys": 0,
            "missing_keys": 0,
            "extra_keys": 0,
            "nonfinite_coordinates": 0,
            "known_dummy_rows": 0,
            "sample_coordinate_fallback_rows": 0,
            "out_of_earth_rows": 0,
            "byte_identical_to_v4": output_bytes == (v4_root / "submission.csv").read_bytes(),
            "atomic_publish": True,
        },
        "truth_policy": {
            "ground_truth_members_read": False,
            "truth_materialized_count": 0,
            "truth_open_count": 0,
            "holdout_or_test_truth_used": False,
            "leaderboard_used": False,
        },
        "publication_policy": {
            "development_only": True,
            "kaggle_external_submission": False,
            "token_access": False,
            "production_rtk_spp_default_changed": False,
            "no_post_generation_tuning": True,
        },
    }
    _atomic_json(output_dir / "source_seam_bridge_manifest.json", manifest)
    run_manifest = {
        "schema_version": RUN_MANIFEST_SCHEMA,
        "status": "completed-truth-free-source-seam-bridge",
        "freeze": {
            "record": _artifact(freeze_path),
            "manifest": _artifact(freeze_manifest_path),
        },
        "source_artifacts": {
            "v4_submission": _artifact(v4_root / "submission.csv"),
            "source_seam_bridge_manifest": _artifact(output_dir / "source_seam_bridge_manifest.json"),
            "submission": _artifact(output_submission),
        },
        "source_seam_bridge": seam_report,
        "physical_continuity": {
            "before": before,
            "after": after,
            "isolated_two_sided_spike_points_after": isolated_after,
        },
        "truth_policy": manifest["truth_policy"],
        "timing": {
            "wall_seconds": time.perf_counter() - started,
            "peak_rss_bytes": resource.getrusage(resource.RUSAGE_SELF).ru_maxrss * 1024,
        },
        "no_post_generation_tuning": True,
    }
    _atomic_json(output_dir / "source_seam_bridge_run_manifest.json", run_manifest)
    return run_manifest


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(prog="gnss_smartphone_native_fgo_test_submission_source_seam_bridge")
    parser.add_argument("--freeze", type=Path, default=DEFAULT_FREEZE)
    parser.add_argument("--freeze-manifest", type=Path, default=DEFAULT_FREEZE_MANIFEST)
    parser.add_argument("--v4-root", type=Path, default=DEFAULT_V4_ROOT)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT)
    return parser.parse_args(argv)


def _resolve(path: Path) -> Path:
    return path if path.is_absolute() else ROOT / path


def run(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    try:
        report = run_bridge(
            _resolve(args.freeze),
            _resolve(args.freeze_manifest),
            _resolve(args.v4_root),
            _resolve(args.output_dir),
        )
    except (SourceSeamBridgeError, OSError, ValueError, KeyError, TypeError) as exc:
        print(f"native FGO source-seam bridge failed: {exc}", file=sys.stderr)
        return 2
    import json

    print(
        json.dumps(
            {
                "status": report["status"],
                "eligible_run_count": report["source_seam_bridge"]["eligible_run_count"],
                "replaced_row_count": report["source_seam_bridge"]["replaced_row_count"],
                "submission": report["source_artifacts"]["submission"],
            },
            sort_keys=True,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(run())
