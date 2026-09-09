#!/usr/bin/env python3
"""Truth-free triangular ECEF stitch for sealed windowed PDC artifacts.

This is a post-processing recovery for the v1 independent-window experiment.
It never invokes ``gnss_fgo``.  Structurally valid v1 windows contribute finite
ECEF rows with a predeclared triangular weight (low at edges, maximal at the
window centre); invalid windows and disagreement pairs are excluded wholesale.
Every missing or physically unsafe row falls back to the exact sealed native
FGO route position, never to a sample or truth coordinate.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import hashlib
import json
import math
import os
from pathlib import Path
import sys
import tempfile
from typing import Any, Iterable, Sequence


_COMMANDS_DIR = Path(__file__).resolve().parents[1]
if str(_COMMANDS_DIR) not in sys.path:
    sys.path.insert(0, str(_COMMANDS_DIR))
from support.gnss_runtime import application_root  # noqa: E402
import gnss_smartphone_native_fgo_pdc_windowed as windowed  # noqa: E402


ROOT = application_root(__file__)
DEFAULT_V1_ROOT = ROOT / "output/smartphone-r5/native-fgo-pdc-windowed-v1"
DEFAULT_OUTPUT = ROOT / "output/smartphone-r5/native-fgo-pdc-windowed-v2-stitch"
DEFAULT_FALLBACK = windowed.DEFAULT_FALLBACK
DEFAULT_FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_gsdc2023_native_fgo_pdc_windowed_stitch_freeze_v1.json"
SCHEMA = "smartphone-r5-gsdc2023-native-fgo-pdc-windowed-stitch.v1"
MAX_TRANSITION_SPEED_MPS = windowed.MAX_TRANSITION_SPEED_MPS
MIN_ECEF_NORM_M = windowed.MIN_ECEF_NORM_M
MAX_ECEF_NORM_M = windowed.MAX_ECEF_NORM_M
V5_MOTION_SIGMA_M = 50.0
V5_UNCERTAINTY_SIGMA_MULTIPLIER = 3.0
OVERLAP_DISAGREEMENT_BOUND_M = V5_UNCERTAINTY_SIGMA_MULTIPLIER * V5_MOTION_SIGMA_M
MAX_ARTIFACT_BYTES = windowed.MAX_ARTIFACT_BYTES


class StitchError(ValueError):
    """Raised when the sealed post-processing contract is invalid."""


@dataclass(frozen=True)
class CandidateWindow:
    index: int
    segment_index: int
    start_epoch: int
    count: int
    rows: tuple[windowed.PosRow, ...]
    summary: dict[str, Any]
    source_path: Path
    contribution: str = "eligible"
    rejection_reason: str | None = None


def sha256(path: Path) -> str:
    if not path.is_file():
        raise StitchError(f"missing file: {path}")
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def relative(path: Path) -> str:
    try:
        return str(path.resolve().relative_to(ROOT.resolve()))
    except ValueError:
        return str(path.resolve())


def atomic_bytes(path: Path, content: bytes) -> None:
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
        if temporary.exists():
            temporary.unlink()


def atomic_json(path: Path, value: dict[str, Any]) -> None:
    atomic_bytes(path, (json.dumps(value, indent=2, sort_keys=True) + "\n").encode("utf-8"))


def verify_freeze(path: Path = DEFAULT_FREEZE) -> dict[str, Any]:
    try:
        freeze = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise StitchError(f"invalid stitch freeze: {path}") from exc
    if not isinstance(freeze, dict) or freeze.get("schema_version") != SCHEMA + "-freeze":
        raise StitchError("stitch freeze schema mismatch")
    expected_source = freeze.get("source_hashes", {}).get(relative(Path(__file__)))
    if expected_source != sha256(Path(__file__)):
        raise StitchError("stitch source hash does not match freeze")
    if freeze.get("window_contract", {}).get("v1_manifest_sha256") != sha256(DEFAULT_V1_ROOT / "windowed_manifest.json"):
        raise StitchError("v1 window manifest changed after stitch freeze")
    if freeze.get("fallback_sha256") != sha256(DEFAULT_FALLBACK):
        raise StitchError("sealed fallback changed after stitch freeze")
    if freeze.get("window_contract", {}).get("window_epochs") != windowed.WINDOW_EPOCHS:
        raise StitchError("window length changed after stitch freeze")
    if freeze.get("window_contract", {}).get("overlap_epochs") != windowed.WINDOW_OVERLAP:
        raise StitchError("window overlap changed after stitch freeze")
    return freeze


def triangular_weight(offset: int, count: int) -> float:
    """Discrete triangular weight with a low, nonzero edge floor."""
    if count <= 0 or offset < 0 or offset >= count:
        raise StitchError("invalid triangular weight coordinate")
    return float(max(1, min(offset + 1, count - offset)))


def _geodetic_from_ecef(ecef: Sequence[float]) -> tuple[float, float, float]:
    x, y, z = (float(value) for value in ecef)
    if not all(math.isfinite(value) for value in (x, y, z)):
        raise StitchError("nonfinite ECEF blend")
    longitude = math.atan2(y, x)
    semi_major = 6378137.0
    eccentricity_sq = 6.6943799901413165e-3
    p = math.hypot(x, y)
    if p == 0.0:
        raise StitchError("ECEF blend at polar singularity")
    latitude = math.atan2(z, p * (1.0 - eccentricity_sq))
    for _ in range(12):
        sin_lat = math.sin(latitude)
        radius = semi_major / math.sqrt(1.0 - eccentricity_sq * sin_lat * sin_lat)
        height = p / math.cos(latitude) - radius
        updated = math.atan2(z, p * (1.0 - eccentricity_sq * radius / (radius + height)))
        if abs(updated - latitude) < 1e-13:
            latitude = updated
            break
        latitude = updated
    sin_lat = math.sin(latitude)
    radius = semi_major / math.sqrt(1.0 - eccentricity_sq * sin_lat * sin_lat)
    height = p / math.cos(latitude) - radius
    return math.degrees(latitude), math.degrees(longitude), height


def _blended_row(canonical: windowed.PosRow, values: Sequence[tuple[windowed.PosRow, float]]) -> windowed.PosRow:
    if not values:
        return canonical
    if len(values) == 1:
        return values[0][0]
    total = sum(weight for _, weight in values)
    if not math.isfinite(total) or total <= 0.0:
        raise StitchError("invalid normalized triangular weights")
    ecef = tuple(
        sum(row.ecef[index] * weight for row, weight in values) / total
        for index in range(3)
    )
    norm = math.sqrt(sum(value * value for value in ecef))
    if not MIN_ECEF_NORM_M <= norm <= MAX_ECEF_NORM_M:
        raise StitchError(f"blended ECEF outside physical range: {norm}")
    latitude, longitude, height = _geodetic_from_ecef(ecef)
    fields = canonical.text.split()
    fields[2:5] = [f"{value:.6f}" for value in ecef]
    fields[5:8] = [f"{latitude:.9f}", f"{longitude:.9f}", f"{height:.6f}"]
    return windowed.PosRow(canonical.key, canonical.week, canonical.tow, ecef, " ".join(fields))


def _load_v1_manifest(v1_root: Path) -> dict[str, Any]:
    path = v1_root / "windowed_manifest.json"
    try:
        manifest = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise StitchError(f"invalid v1 window manifest: {path}") from exc
    if not isinstance(manifest, dict) or manifest.get("schema_version") != windowed.SCHEMA:
        raise StitchError("v1 window manifest schema mismatch")
    if not manifest.get("manifest_paths_repaired", False):
        raise StitchError("v1 nested paths are not repaired/sealed")
    return manifest


def _load_candidate_windows(v1_root: Path, v1_manifest: dict[str, Any], canonical: Sequence[windowed.PosRow]) -> tuple[list[CandidateWindow], list[dict[str, Any]]]:
    windows: list[CandidateWindow] = []
    diagnostics: list[dict[str, Any]] = []
    entries = v1_manifest.get("window_manifests")
    if not isinstance(entries, list):
        raise StitchError("v1 window manifest list missing")
    for entry in entries:
        if not isinstance(entry, dict) or not isinstance(entry.get("window"), dict):
            raise StitchError("malformed v1 window entry")
        metadata = entry["window"]
        index = int(metadata["index"])
        start = int(metadata["start_epoch"])
        count = int(metadata["count"])
        segment = int(metadata["segment_index"])
        path = v1_root / "windows" / f"window-{index:04d}" / "fgo.pos"
        reason: str | None = None
        rows: tuple[windowed.PosRow, ...] = ()
        summary = entry.get("summary")
        if entry.get("source") != "windowed_native_fgo":
            reason = entry.get("reason") or "v1 window was already fail-closed"
        elif not isinstance(summary, dict):
            reason = "v1 valid window summary missing"
        else:
            try:
                windowed._required_summary(summary, count)
                parsed = windowed.read_pos(path)
                expected = tuple(row.key for row in canonical[start:start + count])
                if tuple(row.key for row in parsed) != expected:
                    raise StitchError("v1 window key set/order mismatch")
                speeds = [windowed.ecef_speed(left, right) for left, right in zip(parsed, parsed[1:])]
                if any(speed > MAX_TRANSITION_SPEED_MPS for speed in speeds):
                    raise StitchError("v1 window internal continuity exceeds 70 m/s")
                rows = tuple(parsed)
            except (OSError, KeyError, TypeError, ValueError, StitchError) as exc:
                reason = str(exc)
        if reason is None:
            windows.append(CandidateWindow(index, segment, start, count, rows, summary, path))
            diagnostics.append({"index": index, "status": "eligible", "path": relative(path)})
        else:
            diagnostics.append({"index": index, "status": "rejected", "reason": reason, "path": relative(path)})
    return windows, diagnostics


def _mark_overlap_rejections(windows: Sequence[CandidateWindow], canonical: Sequence[windowed.PosRow]) -> tuple[list[CandidateWindow], list[dict[str, Any]]]:
    by_index = {item.index: item for item in windows}
    rejected: dict[int, str] = {}
    disagreements: list[dict[str, Any]] = []
    ordered = sorted(windows, key=lambda item: item.index)
    for left, right in zip(ordered, ordered[1:]):
        if left.segment_index != right.segment_index:
            continue
        left_rows = {row.key: row for row in left.rows}
        right_rows = {row.key: row for row in right.rows}
        common = sorted(set(left_rows) & set(right_rows))
        if not common:
            continue
        distances = [
            math.sqrt(sum((a - b) ** 2 for a, b in zip(left_rows[key].ecef, right_rows[key].ecef)))
            for key in common
        ]
        maximum = max(distances)
        # Exact keyed overlap has zero timestamp span.  If a future artifact
        # carries a nonzero alignment span, the physical 70 m/s term expands
        # the same frozen 3-sigma v5 uncertainty bound deterministically.
        timestamp_span_s = 0.0
        bound = OVERLAP_DISAGREEMENT_BOUND_M + MAX_TRANSITION_SPEED_MPS * timestamp_span_s
        item = {
            "left_window": left.index,
            "right_window": right.index,
            "common_epochs": len(common),
            "max_disagreement_m": maximum,
            "median_disagreement_m": sorted(distances)[len(distances) // 2],
            "timestamp_span_s": timestamp_span_s,
            "bound_m": bound,
            "within_bound": maximum <= bound,
        }
        if maximum > bound:
            rejected[left.index] = f"overlap disagreement with window {right.index}: {maximum:.9f} m > {bound:.9f} m"
            rejected[right.index] = f"overlap disagreement with window {left.index}: {maximum:.9f} m > {bound:.9f} m"
            item["decision"] = "reject_both_window_contributions"
        else:
            item["decision"] = "retain_both_window_contributions"
        disagreements.append(item)
    output: list[CandidateWindow] = []
    for item in windows:
        if item.index in rejected:
            output.append(CandidateWindow(
                item.index, item.segment_index, item.start_epoch, item.count,
                item.rows, item.summary, item.source_path,
                contribution="rejected_overlap", rejection_reason=rejected[item.index],
            ))
        else:
            output.append(item)
    return output, disagreements


def _candidate_values(canonical: Sequence[windowed.PosRow], windows: Sequence[CandidateWindow]) -> tuple[list[windowed.PosRow], list[str], list[dict[str, Any]]]:
    # A key is normally unique across a clock-reset boundary, but do not let
    # a malformed/re-stamped artifact blend rows merely because its numeric
    # key happens to collide.  The canonical segment map makes the reset rule
    # explicit and keeps the stitch fail-closed for adversarial fixtures too.
    canonical_segment_by_key: dict[tuple[int, int], int] = {}
    for segment_index, (start, end) in enumerate(windowed.segment_ranges(canonical)):
        for row in canonical[start:end]:
            canonical_segment_by_key[row.key] = segment_index
    values: dict[tuple[int, int], list[tuple[windowed.PosRow, float, int, int]]] = {}
    for item in windows:
        if item.contribution != "eligible":
            continue
        for offset, row in enumerate(item.rows):
            values.setdefault(row.key, []).append((row, triangular_weight(offset, item.count), item.index, item.segment_index))
    selected: list[windowed.PosRow] = []
    sources: list[str] = []
    weight_stats: list[dict[str, Any]] = []
    for baseline in canonical:
        expected_segment = canonical_segment_by_key[baseline.key]
        entries = [
            entry for entry in values.get(baseline.key, [])
            if entry[3] == expected_segment
        ]
        if not entries:
            selected.append(baseline)
            sources.append("sealed_v5_fallback_missing_or_rejected")
            weight_stats.append({"key": list(baseline.key), "contributors": 0, "normalized_weights": []})
            continue
        if len(entries) == 1:
            selected.append(entries[0][0])
            sources.append("single_window_unchanged")
            weight_stats.append({"key": list(baseline.key), "contributors": 1, "normalized_weights": [[entries[0][2], 1.0]]})
            continue
        total = sum(weight for _, weight, _, _ in entries)
        selected.append(_blended_row(baseline, [(row, weight) for row, weight, _, _ in entries]))
        sources.append("triangular_ecef_blend")
        weight_stats.append({
            "key": list(baseline.key),
            "contributors": len(entries),
            "normalized_weights": [[index, weight / total] for _, weight, index, _ in entries],
        })
    return selected, sources, weight_stats


def _continuity_fallback(canonical: Sequence[windowed.PosRow], selected: list[windowed.PosRow], sources: list[str]) -> tuple[list[windowed.PosRow], list[str], dict[str, Any]]:
    fallback_count = 0
    offending: list[dict[str, Any]] = []
    # Fail closed at affected epochs only.  The sealed fallback itself is
    # validated first, so replacing a candidate endpoint cannot introduce a
    # new unsafe transition.  Iterate to a fixed point, without translating or
    # smoothing any coordinate.
    for _ in range(len(selected) + 1):
        changed = False
        for index, (left, right) in enumerate(zip(selected, selected[1:])):
            speed = windowed.ecef_speed(left, right)
            if speed <= MAX_TRANSITION_SPEED_MPS:
                continue
            offending.append({
                "left_key": list(left.key), "right_key": list(right.key),
                "speed_mps": speed,
            })
            for position in (index, index + 1):
                if sources[position] != "sealed_v5_fallback_continuity":
                    selected[position] = canonical[position]
                    sources[position] = "sealed_v5_fallback_continuity"
                    fallback_count += 1
                    changed = True
        if not changed:
            break
    speeds = [windowed.ecef_speed(left, right) for left, right in zip(selected, selected[1:])]
    return selected, sources, {
        "fallback_count": fallback_count,
        "initial_offending_edges": offending,
        "max_transition_speed_mps": max(speeds, default=0.0),
        "above_bound_edges_after": sum(speed > MAX_TRANSITION_SPEED_MPS for speed in speeds),
    }


def _write_pos(path: Path, rows: Sequence[windowed.PosRow]) -> None:
    payload = (
        "% LibGNSS++ Position Solution\n"
        "% Format: pos\n"
        "% Columns: GPS_Week GPS_TOW X(m) Y(m) Z(m) Lat(deg) Lon(deg) Height(m) Status Satellites PDOP Ratio FixedAmbiguities Iterations\n"
        + "\n".join(row.text for row in rows) + "\n"
    ).encode("utf-8")
    atomic_bytes(path, payload)


def _validate_existing(output_root: Path) -> dict[str, Any]:
    manifest_path = output_root / "stitch_manifest.json"
    output_path = output_root / "windowed_v2.pos"
    if not manifest_path.is_file() or not output_path.is_file():
        raise StitchError(f"existing stitch output is not sealed: {output_root}")
    value = json.loads(manifest_path.read_text(encoding="utf-8"))
    if not isinstance(value, dict) or value.get("schema_version") != SCHEMA:
        raise StitchError("existing stitch manifest schema mismatch")
    if value.get("output_sha256") != sha256(output_path):
        raise StitchError("existing stitch output hash mismatch")
    return value


def run(
    *,
    v1_root: Path = DEFAULT_V1_ROOT,
    fallback_pos: Path = DEFAULT_FALLBACK,
    output_root: Path = DEFAULT_OUTPUT,
    freeze_record: Path = DEFAULT_FREEZE,
) -> dict[str, Any]:
    if output_root.exists():
        return _validate_existing(output_root)
    freeze = verify_freeze(freeze_record)
    v1_manifest = _load_v1_manifest(v1_root)
    canonical = windowed.read_pos(fallback_pos)
    if not canonical:
        raise StitchError("sealed fallback has no route rows")
    windows, validity = _load_candidate_windows(v1_root, v1_manifest, canonical)
    windows, disagreements = _mark_overlap_rejections(windows, canonical)
    selected, sources, weight_stats = _candidate_values(canonical, windows)
    selected, sources, continuity = _continuity_fallback(canonical, selected, sources)
    if len(selected) != len(canonical) or [row.key for row in selected] != [row.key for row in canonical]:
        raise StitchError("stitch key coverage/order mismatch")
    # Re-read the generated in-memory rows through the same strict parser after
    # writing to staging, catching duplicate/nonfinite/out-of-Earth regressions.
    output_root.parent.mkdir(parents=True, exist_ok=True)
    temporary_root = Path(tempfile.mkdtemp(prefix=f".{output_root.name}.", dir=str(output_root.parent)))
    try:
        staged_output = temporary_root / "windowed_v2.pos"
        _write_pos(staged_output, selected)
        reread = windowed.read_pos(staged_output)
        if [row.key for row in reread] != [row.key for row in canonical]:
            raise StitchError("independent output reread key mismatch")
        reread_speeds = [windowed.ecef_speed(left, right) for left, right in zip(reread, reread[1:])]
        if any(speed > MAX_TRANSITION_SPEED_MPS for speed in reread_speeds):
            raise StitchError("post-write output retains unsafe transition")
        source_counts = {
            source: sources.count(source)
            for source in sorted(set(sources))
        }
        rejected = [item for item in windows if item.contribution != "eligible"]
        structural_passed = (
            len(reread) == len(canonical)
            and not any(speed > MAX_TRANSITION_SPEED_MPS for speed in reread_speeds)
            and all(MIN_ECEF_NORM_M <= math.sqrt(sum(value * value for value in row.ecef)) <= MAX_ECEF_NORM_M for row in reread)
        )
        manifest: dict[str, Any] = {
            "schema_version": SCHEMA,
            "status": "truth-free-windowed-pdc-stitch-sealed",
            "candidate": "native_fgo_pdc_triangular_ecef_overlap_stitch_v1",
            "truth_policy": {
                "truth_free": True,
                "truth_opened": False,
                "truth_materialized_count": 0,
                "validation_opened": False,
                "holdout_opened": False,
                "test_data_used": False,
                "leaderboard_used": False,
            },
            "route": "2021-03-16-18-59-us-ca-mtv-a/pixel5",
            "freeze": {
                "path": relative(freeze_record),
                "sha256": sha256(freeze_record),
            },
            "inputs": {
                "v1_manifest": {"path": relative(v1_root / "windowed_manifest.json"), "sha256": sha256(v1_root / "windowed_manifest.json")},
                "v1_position_root": {"path": relative(v1_root), "window_count": len(v1_manifest.get("window_manifests", []))},
                "sealed_fallback": {"path": relative(fallback_pos), "sha256": sha256(fallback_pos)},
            },
            "contract": {
                "window_epochs": windowed.WINDOW_EPOCHS,
                "overlap_epochs": windowed.WINDOW_OVERLAP,
                "stride_epochs": windowed.WINDOW_STRIDE,
                "weight_family": "discrete triangular",
                "edge_weight_floor": 1.0,
                "center_weight": "maximal",
                "coordinate_space": "ECEF; WGS84 conversion only after normalized blend",
                "one_contributor": "byte/text unchanged from its valid window row",
                "invalid_window": "whole contribution rejected",
                "overlap_disagreement": {
                    "v5_motion_sigma_m": V5_MOTION_SIGMA_M,
                    "v5_uncertainty_sigma_multiplier": V5_UNCERTAINTY_SIGMA_MULTIPLIER,
                    "same_timestamp_span_s": 0.0,
                    "speed_bound_mps": MAX_TRANSITION_SPEED_MPS,
                    "bound_m": OVERLAP_DISAGREEMENT_BOUND_M,
                    "formula": "3 * frozen v5 motion sigma + 70 m/s * exact-key timestamp span",
                    "exceedance_decision": "reject both neighboring window contributions; fallback at affected keys",
                },
                "segment_boundary": "never blend across v1 segment_index or clock/time reset",
                "dummy_or_sample_coordinates": "forbidden",
                "truth_selection": "forbidden",
            },
            "validity": validity,
            "overlap_disagreements": disagreements,
            "window_contributions": [
                {
                    "index": item.index,
                    "segment_index": item.segment_index,
                    "start_epoch": item.start_epoch,
                    "count": item.count,
                    "contribution": item.contribution,
                    "rejection_reason": item.rejection_reason,
                    "source_path": relative(item.source_path),
                    "source_sha256": sha256(item.source_path) if item.source_path.is_file() else None,
                }
                for item in windows
            ],
            "weight_statistics": {
                "epoch_count": len(weight_stats),
                "max_contributors": max((item["contributors"] for item in weight_stats), default=0),
                "multi_contributor_epochs": sum(item["contributors"] > 1 for item in weight_stats),
                "preview": weight_stats[:3] + weight_stats[-3:],
            },
            "source_counts": source_counts,
            "continuity": continuity,
            "verification": {
                "exact_key_order": True,
                "row_count": len(reread),
                "duplicate_keys": 0,
                "missing_keys": 0,
                "extra_keys": 0,
                "nonfinite_coordinates": 0,
                "out_of_earth_rows": 0,
                "sample_coordinate_rows": 0,
                "max_transition_speed_mps": max(reread_speeds, default=0.0),
                "above_transition_speed_edges": sum(speed > MAX_TRANSITION_SPEED_MPS for speed in reread_speeds),
                "atomic_publish": True,
            },
            "structural_gate": {
                "all_output_rows_finite_and_physical": True,
                "exact_keys_and_coverage": True,
                "no_unsafe_transition": True,
                "passed": structural_passed,
            },
            "output_sha256": sha256(staged_output),
            "output_rows": len(reread),
            "promotion": "not scored; structural pass permits one development train CV score, with validation/holdout still sealed",
        }
        atomic_json(temporary_root / "stitch_manifest.json", manifest)
        atomic_bytes(temporary_root / "stitch_manifest.sha256", f"{sha256(temporary_root / 'stitch_manifest.json')}  stitch_manifest.json\n".encode("ascii"))
        os.replace(temporary_root, output_root)
        return manifest
    except Exception:
        import shutil
        shutil.rmtree(temporary_root, ignore_errors=True)
        raise


def main(argv: Iterable[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--v1-root", type=Path, default=DEFAULT_V1_ROOT)
    parser.add_argument("--fallback-pos", type=Path, default=DEFAULT_FALLBACK)
    parser.add_argument("--output-root", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--freeze-record", type=Path, default=DEFAULT_FREEZE)
    parser.add_argument("--no-truth", action="store_true", help="assert no truth file may be opened")
    args = parser.parse_args(list(argv) if argv is not None else None)
    if not args.no_truth:
        print("windowed stitch requires explicit --no-truth", file=sys.stderr)
        return 2
    try:
        manifest = run(
            v1_root=args.v1_root, fallback_pos=args.fallback_pos,
            output_root=args.output_root, freeze_record=args.freeze_record,
        )
    except (OSError, ValueError, StitchError) as exc:
        print(f"windowed stitch failed closed: {exc}", file=sys.stderr)
        return 2
    print(json.dumps({
        "output_root": relative(args.output_root),
        "status": manifest.get("status"),
        "structural_passed": manifest.get("structural_gate", {}).get("passed"),
        "output_sha256": manifest.get("output_sha256"),
        "source_counts": manifest.get("source_counts"),
    }, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
