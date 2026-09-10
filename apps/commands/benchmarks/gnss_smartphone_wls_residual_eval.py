#!/usr/bin/env python3
"""Development-only WLS residual candidate research for smartphone lanes.

The candidate set is fixed before the new validation payload is opened.  The
already truth-opened seven development routes are used to describe local
along/cross/up residual bias and to rank the nine small, truth-free WLS
post-processors.  A metadata-frozen validation route is then materialized
without truth, run through Galileo-E1/Hatch30, WLS, native segment stability,
and the selector, and scored once after all truth-free artifacts are sealed.

The next holdout is represented only by central-directory metadata in the
selection record.  This command never materializes or opens that holdout.
The experimental result is development-only; production RTK/SPP defaults are
not changed here.
"""

from __future__ import annotations

import argparse
from bisect import bisect_left
from dataclasses import dataclass
import hashlib
import json
import math
import os
from pathlib import Path
import resource
import sys
import time
from typing import Any

from support.gnss_runtime import application_root


ROOT = application_root(__file__)
BENCHMARK_DIR = ROOT / "apps" / "commands" / "benchmarks"
if str(BENCHMARK_DIR) not in sys.path:
    sys.path.insert(0, str(BENCHMARK_DIR))
import gnss_smartphone_generalization as generalization  # noqa: E402
import gnss_smartphone_kaggle as kaggle  # noqa: E402
import gnss_smartphone_reacquisition_eval as reacquisition  # noqa: E402
import gnss_smartphone_reacquisition_conservative_eval as conservative  # noqa: E402
import gnss_smartphone_trajectory_smoother as smoother  # noqa: E402
import gnss_smartphone_trajectory_smoother_eval as smoother_eval  # noqa: E402
import gnss_smartphone_wls as wls  # noqa: E402
import gnss_smartphone_wls_eval as wls_eval  # noqa: E402
import gnss_smartphone_wls_residual as residual  # noqa: E402
import gnss_smartphone_wls_stability_selector as selector  # noqa: E402
import gnss_smartphone_wls_stability_selector_eval as stability_eval  # noqa: E402


SCHEMA_VERSION = "smartphone-r5-wls-residual-release-evaluation.v1"
MANIFEST_SCHEMA_VERSION = "smartphone-r5-wls-residual-release-manifest.v1"
SELECTION_SCHEMA_VERSION = "smartphone-r5-wls-residual-release-candidate-selection.v1"
DEFAULT_ARCHIVE = ROOT / "data" / "gsdc2023" / "cache" / "dataset_2023.zip"
DEFAULT_PROFILE = ROOT / "configs" / "benchmarks" / "smartphone_r5_gsdc2023.json"
DEFAULT_INVENTORY = (
    ROOT / "output" / "smartphone-r5" / "generalization-v6" / "archive_inventory.json"
)
DEFAULT_SELECTION_RECORD = (
    ROOT
    / "docs"
    / "use_cases"
    / "records"
    / "smartphone_r5_wls_residual_release_candidate_selection.json"
)
DEFAULT_EXISTING_REPORT = (
    ROOT / "output" / "smartphone-r5" / "wls-position-v1" / "wls_position_report.json"
)
DEFAULT_DEVICE_FAMILY_REPORT = (
    ROOT
    / "output"
    / "smartphone-r5"
    / "wls-device-family-v1"
    / "wls_device_family_report.json"
)
DEFAULT_OUTPUT_DIR = ROOT / "output" / "smartphone-r5" / "wls-residual-v1"

HOLDOUT_ID = "2023-09-06-22-49-us-ca-routebb1/pixel7pro"
NEW_VALIDATION_ID = "2023-05-25-20-11-us-ca-sjc-he2/pixel7pro"
NEXT_HOLDOUT_ID = "2023-05-25-19-10-us-ca-sjc-be2/sm-s908b"
OLD_HOLDOUT_ID = HOLDOUT_ID
KNOWN_SEVEN_IDS = tuple(stability_eval.KNOWN_SEVEN_IDS)
DIAGNOSTIC_KEYS = tuple(wls_eval.DIAGNOSTIC_KEYS)
SPEED_BINS = ("slow_lt_1mps", "mid_1_to_5mps", "fast_ge_5mps")
TURN_BINS = ("straight_lt_5degps", "turning_ge_5degps")
MATCH_TOLERANCE_MS = 100
SIGNOFF_EPSILON = 1.0e-12


class ResidualEvaluationError(ValueError):
    """Raised when the frozen residual research contract is violated."""


@dataclass(frozen=True)
class TruthRecord:
    timestamp_ms: int
    latitude_deg: float
    longitude_deg: float
    height_m: float
    speed_mps: float | None
    bearing_deg: float | None
    ecef: Any


def _sha256(path: Path) -> str:
    if not path.is_file():
        raise ResidualEvaluationError(f"missing artifact: {path}")
    digest = hashlib.sha256()
    try:
        with path.open("rb") as handle:
            for chunk in iter(lambda: handle.read(1024 * 1024), b""):
                digest.update(chunk)
    except OSError as exc:
        raise ResidualEvaluationError(f"failed to hash {path}") from exc
    return digest.hexdigest()


def _artifact(path: Path) -> dict[str, Any]:
    return {"path": str(path), "sha256": _sha256(path), "bytes": path.stat().st_size}


def _atomic_json(path: Path, payload: dict[str, Any]) -> None:
    smoother._atomic_write(
        path,
        (json.dumps(payload, indent=2, sort_keys=True) + "\n").encode("utf-8"),
    )


def _load_json(path: Path, label: str) -> dict[str, Any]:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ResidualEvaluationError(f"invalid {label}: {path}") from exc
    if not isinstance(payload, dict):
        raise ResidualEvaluationError(f"{label} must be a JSON object: {path}")
    return payload


def _absolute(path: Path) -> Path:
    return path if path.is_absolute() else ROOT / path


def _metric(metrics: dict[str, Any], path: tuple[str, ...]) -> float:
    value: Any = metrics
    try:
        for key in path:
            value = value[key]
        number = float(value)
    except (KeyError, TypeError, ValueError):
        return math.inf
    return number if math.isfinite(number) else math.inf


def _comparison_metric(metrics: dict[str, Any], path: tuple[str, ...]) -> float:
    """Read either a route score or its seven-route aggregate equivalent."""

    aggregate_paths = {
        ("availability_ratio",): ("mean_availability_ratio",),
        ("horizontal_wgs84_m", "p50_m"): ("mean_horizontal_wgs84_p50_m",),
        ("horizontal_wgs84_m", "p95_m"): ("mean_horizontal_wgs84_p95_m",),
        ("vertical_p95_abs_m",): ("mean_vertical_p95_abs_m",),
    }
    if path[0] == "kaggle_diagnostic_score_variants_m":
        aggregate_path = ("mean_kaggle_diagnostic_score_variants_m", *path[1:])
        if aggregate_path[0] in metrics:
            return _metric(metrics, aggregate_path)
    if path in aggregate_paths and aggregate_paths[path][0] in metrics:
        return _metric(metrics, aggregate_paths[path])
    return _metric(metrics, path)


def _score(position_path: Path, device_path: Path, truth: dict[int, tuple[float, float, float]]) -> dict[str, Any]:
    positions = smoother._read_positions(position_path, 18)
    epochs = smoother._read_device_epochs(device_path, 1)
    return reacquisition._score(reacquisition._raw_rows(positions), positions, epochs, truth)


def _aggregate(metrics: list[dict[str, Any]]) -> dict[str, Any]:
    if not metrics:
        raise ResidualEvaluationError("cannot aggregate an empty lane")
    return wls_eval._aggregate(metrics)


def _non_regression(candidate: dict[str, Any], reference: dict[str, Any]) -> tuple[bool, list[str]]:
    failures: list[str] = []
    if _comparison_metric(candidate, ("availability_ratio",)) < _comparison_metric(reference, ("availability_ratio",)) - SIGNOFF_EPSILON:
        failures.append("availability_regression")
    for path, name in (
        (("horizontal_wgs84_m", "p50_m"), "h_p50_regression"),
        (("horizontal_wgs84_m", "p95_m"), "h_p95_regression"),
        (("vertical_p95_abs_m",), "v_p95_regression"),
    ):
        if _comparison_metric(candidate, path) > _comparison_metric(reference, path) + SIGNOFF_EPSILON:
            failures.append(name)
    for key in DIAGNOSTIC_KEYS:
        if _comparison_metric(candidate, ("kaggle_diagnostic_score_variants_m", key)) > _comparison_metric(
            reference, ("kaggle_diagnostic_score_variants_m", key)
        ) + SIGNOFF_EPSILON:
            failures.append(f"{key}_regression")
    return not failures, failures


def _strict_horizontal_better(candidate: dict[str, Any], reference: dict[str, Any]) -> bool:
    # Keep the diagnostic comparison explicit: each of the four official-like
    # variants must improve, not just their average.
    for path in (
        ("horizontal_wgs84_m", "p50_m"),
        ("horizontal_wgs84_m", "p95_m"),
    ):
        if _comparison_metric(candidate, path) >= _comparison_metric(reference, path) - SIGNOFF_EPSILON:
            return False
    return all(
        _comparison_metric(candidate, ("kaggle_diagnostic_score_variants_m", key))
        < _comparison_metric(reference, ("kaggle_diagnostic_score_variants_m", key)) - SIGNOFF_EPSILON
        for key in DIAGNOSTIC_KEYS
    )


def _load_selection_record(path: Path) -> dict[str, Any]:
    record = _load_json(path, "residual candidate selection record")
    if record.get("schema_version") != SELECTION_SCHEMA_VERSION:
        raise ResidualEvaluationError("residual candidate selection schema is invalid")
    if record.get("status") != "selection-frozen-before-member-content-read":
        raise ResidualEvaluationError("residual candidate selection is not pre-payload frozen")
    archive = record.get("archive")
    if not isinstance(archive, dict):
        raise ResidualEvaluationError("selection record lacks archive contract")
    if archive.get("central_directory_only") is not True:
        raise ResidualEvaluationError("selection record was not central-directory-only")
    if archive.get("member_content_read_at_selection") is not False:
        raise ResidualEvaluationError("selection record claims payload access")
    selected = record.get("new_validation")
    next_holdout = record.get("next_holdout")
    if not isinstance(selected, dict) or selected.get("dataset_id") != NEW_VALIDATION_ID:
        raise ResidualEvaluationError("selection record does not freeze the new validation")
    if not isinstance(next_holdout, dict) or next_holdout.get("dataset_id") != NEXT_HOLDOUT_ID:
        raise ResidualEvaluationError("selection record does not freeze the next holdout")
    if next_holdout.get("materialization_forbidden") is not True or next_holdout.get("truth_open_forbidden") is not True:
        raise ResidualEvaluationError("next holdout is not sealed in selection record")
    policy = record.get("candidate_source_and_selection_policy")
    if not isinstance(policy, dict) or policy.get("old_holdout_run2_metrics_used_for_ranking") is not False:
        raise ResidualEvaluationError("old holdout exclusion is not proven")
    if policy.get("known_development_seven_only_for_residual_analysis") is not True:
        raise ResidualEvaluationError("residual analysis role is not frozen to seven routes")
    excluded = record.get("excluded_ids", {}).get("all_previously_used_or_sealed", ())
    if not isinstance(excluded, list) or OLD_HOLDOUT_ID not in excluded:
        raise ResidualEvaluationError("old holdout is absent from the exclusion record")
    if OLD_HOLDOUT_ID in KNOWN_SEVEN_IDS or NEXT_HOLDOUT_ID in KNOWN_SEVEN_IDS:
        raise ResidualEvaluationError("sealed holdout leaked into known development routes")
    return record


def _verify_central_inventory(
    archive_path: Path, inventory_path: Path, record: dict[str, Any]
) -> tuple[dict[str, Any], dict[str, Any]]:
    expected_inventory = str(record["archive"]["central_directory_inventory_sha256"])
    if _sha256(inventory_path) != expected_inventory:
        raise ResidualEvaluationError("central inventory hash differs from frozen selection")
    inventory = _load_json(inventory_path, "central-directory inventory")
    archive = inventory.get("archive")
    if not isinstance(archive, dict) or archive.get("central_directory_only") is not True or archive.get("member_content_read") is not False:
        raise ResidualEvaluationError("inventory is not a payload-free central-directory inventory")
    expected_archive = str(record["archive"]["sha256"])
    actual_archive = _sha256(archive_path)
    if actual_archive != expected_archive:
        raise ResidualEvaluationError("archive hash differs from frozen selection")
    rows = {
        str(row.get("dataset_id")): row
        for row in inventory.get("train", {}).get("records", [])
        if isinstance(row, dict)
    }
    candidate = rows.get(NEW_VALIDATION_ID)
    next_holdout = rows.get(NEXT_HOLDOUT_ID)
    if candidate is None or next_holdout is None:
        raise ResidualEvaluationError("frozen validation/next holdout is absent from inventory")
    frozen_candidate = record["new_validation"]
    for key in ("route", "phone", "calendar_year", "environment"):
        if key == "environment":
            continue
        if key in frozen_candidate and candidate.get(key) != frozen_candidate[key]:
            raise ResidualEvaluationError(f"validation metadata changed: {key}")
    if not candidate.get("required_files_complete") or not candidate.get("broadcast_nav_present"):
        raise ResidualEvaluationError("validation lacks required central-directory members")
    if int(candidate.get("broadcast_nav_duplicate_count", 0)) != 0:
        raise ResidualEvaluationError("validation broadcast nav is duplicated")
    frozen_pair = next(
        (
            item
            for item in record.get("metadata_selected_pair_candidates", [])
            if isinstance(item, dict) and item.get("dataset_id") == NEW_VALIDATION_ID
        ),
        None,
    )
    if not isinstance(frozen_pair, dict):
        raise ResidualEvaluationError("selection record lacks validation central metadata")
    if frozen_pair.get("central_directory_members") != candidate.get("central_directory_files"):
        raise ResidualEvaluationError("validation central-directory member metadata changed")
    if next_holdout.get("dataset_id") != NEXT_HOLDOUT_ID:
        raise ResidualEvaluationError("next holdout inventory key mismatch")
    return inventory, candidate


def _known_route_records(
    existing_report: dict[str, Any], prior_report: dict[str, Any]
) -> dict[str, dict[str, Any]]:
    records: dict[str, dict[str, Any]] = {}
    for dataset_id in KNOWN_SEVEN_IDS:
        if dataset_id == OLD_HOLDOUT_ID or dataset_id == NEXT_HOLDOUT_ID:
            raise ResidualEvaluationError("holdout appeared in known route list")
        inputs = stability_eval._known_route_inputs(existing_report, prior_report, dataset_id)
        device = _absolute(Path(inputs["device"]))
        native = _absolute(Path(inputs["native"]))
        wls_path = _absolute(Path(inputs["wls"]))
        wls_manifest = _absolute(Path(inputs["wls_manifest"]))
        if dataset_id == stability_eval.PRIOR_NEW_ROUTE_ID:
            truth = _absolute(Path(prior_report["route"]["inputs"]["ground_truth"]["path"]))
        else:
            artifact = existing_report.get("route_artifacts", {}).get(dataset_id)
            if not isinstance(artifact, dict):
                raise ResidualEvaluationError(f"existing report lacks truth path: {dataset_id}")
            truth = _absolute(Path(str(artifact["truth"])))
        for path, label in ((device, "device"), (native, "native"), (wls_path, "WLS"), (wls_manifest, "WLS manifest"), (truth, "truth")):
            if not path.is_file():
                raise ResidualEvaluationError(f"known {label} artifact is missing: {path}")
        selector_path = (
            ROOT
            / "output"
            / "smartphone-r5"
            / "wls-stability-selector-v1"
            / "known-routes"
            / stability_eval._safe_id(dataset_id)
            / "selector"
            / "selector_manifest.json"
        )
        selector_manifest = _load_json(selector_path, "known selector manifest")
        if selector_manifest.get("truth_free") is not True or selector_manifest.get("truth_used") is not False:
            raise ResidualEvaluationError(f"known selector is not truth-free: {dataset_id}")
        decision = selector_manifest.get("decision")
        if decision not in ("native_stable", "wls_raw"):
            raise ResidualEvaluationError(f"unknown known selector decision: {dataset_id}")
        # Re-validate the raw WLS integrity contract without opening any new
        # dataset.  This also proves the candidate input lane is finite.
        selector._validate_wls_manifest(wls_manifest, wls_path, device)
        records[dataset_id] = {
            "dataset_id": dataset_id,
            "phone": dataset_id.rsplit("/", 1)[1],
            "device": device,
            "native": native,
            "wls": wls_path,
            "wls_manifest": wls_manifest,
            "truth": truth,
            "selector_manifest": selector_path,
            "decision": decision,
            "native_artifact": _artifact(native),
            "wls_artifact": _artifact(wls_path),
            "device_artifact": _artifact(device),
            "truth_artifact": _artifact(truth),
        }
    if tuple(records) != KNOWN_SEVEN_IDS:
        raise ResidualEvaluationError("known route order differs from frozen seven-route contract")
    return records


def _optional_float(raw: Any) -> float | None:
    try:
        value = float(raw)
    except (TypeError, ValueError):
        return None
    return value if math.isfinite(value) else None


def _truth_records(path: Path) -> list[TruthRecord]:
    rows: list[TruthRecord] = []
    try:
        import csv

        with path.open(encoding="utf-8-sig", newline="") as handle:
            reader = csv.DictReader(handle)
            required = {"UnixTimeMillis", "LatitudeDegrees", "LongitudeDegrees", "AltitudeMeters"}
            if not required.issubset(set(reader.fieldnames or ())):
                raise ResidualEvaluationError(f"truth lacks residual-analysis fields: {path}")
            seen: set[int] = set()
            for line_number, raw in enumerate(reader, start=2):
                try:
                    timestamp = int(raw["UnixTimeMillis"])
                    latitude = float(raw["LatitudeDegrees"])
                    longitude = float(raw["LongitudeDegrees"])
                    height = float(raw["AltitudeMeters"])
                except (KeyError, TypeError, ValueError) as exc:
                    raise ResidualEvaluationError(f"invalid truth row {line_number}: {path}") from exc
                values = (latitude, longitude, height)
                if timestamp < 0 or not all(math.isfinite(value) for value in values):
                    raise ResidualEvaluationError(f"non-finite truth row {line_number}: {path}")
                if timestamp in seen:
                    raise ResidualEvaluationError(f"duplicate truth timestamp {timestamp}: {path}")
                seen.add(timestamp)
                ecef = smoother._wgs84_geodetic_to_ecef(
                    math.radians(latitude), math.radians(longitude), height
                )
                rows.append(
                    TruthRecord(
                        timestamp,
                        latitude,
                        longitude,
                        height,
                        _optional_float(raw.get("SpeedMps")),
                        _optional_float(raw.get("BearingDegrees")),
                        ecef,
                    )
                )
    except OSError as exc:
        raise ResidualEvaluationError(f"failed to read truth for residual analysis: {path}") from exc
    if not rows:
        raise ResidualEvaluationError(f"truth is empty: {path}")
    rows.sort(key=lambda row: row.timestamp_ms)
    return rows


def _nearest_truth(records: list[TruthRecord], timestamps: list[int], timestamp: int) -> TruthRecord | None:
    index = bisect_left(timestamps, timestamp)
    candidates: list[TruthRecord] = []
    if index < len(records):
        candidates.append(records[index])
    if index:
        candidates.append(records[index - 1])
    if not candidates:
        return None
    nearest = min(candidates, key=lambda item: abs(item.timestamp_ms - timestamp))
    return nearest if abs(nearest.timestamp_ms - timestamp) <= MATCH_TOLERANCE_MS else None


def _bearing(records: list[TruthRecord], index: int) -> float | None:
    direct = records[index].bearing_deg
    if direct is not None:
        return direct % 360.0
    if len(records) < 2:
        return None
    if index == 0:
        before, after = records[0], records[1]
    elif index == len(records) - 1:
        before, after = records[-2], records[-1]
    else:
        before, after = records[index - 1], records[index + 1]
    delta = after.ecef - before.ecef
    east, north, up = residual._enu_basis(records[index].ecef)
    east_component = float(delta @ east)
    north_component = float(delta @ north)
    if math.hypot(east_component, north_component) <= 1.0e-9:
        return None
    return math.degrees(math.atan2(east_component, north_component)) % 360.0


def _turn_rate(records: list[TruthRecord], index: int) -> float | None:
    if len(records) < 2:
        return None
    if index == 0:
        left, right = 0, 1
    elif index == len(records) - 1:
        left, right = len(records) - 2, len(records) - 1
    else:
        left, right = index - 1, index + 1
    first = _bearing(records, left)
    second = _bearing(records, right)
    dt = (records[right].timestamp_ms - records[left].timestamp_ms) / 1000.0
    if first is None or second is None or dt <= 0.0:
        return None
    delta = abs((second - first + 180.0) % 360.0 - 180.0)
    return delta / dt


def _speed(records: list[TruthRecord], index: int) -> float | None:
    direct = records[index].speed_mps
    if direct is not None and direct >= 0.0:
        return direct
    if len(records) < 2:
        return None
    if index == 0:
        left, right = 0, 1
    elif index == len(records) - 1:
        left, right = len(records) - 2, len(records) - 1
    else:
        left, right = index - 1, index + 1
    dt = (records[right].timestamp_ms - records[left].timestamp_ms) / 1000.0
    if dt <= 0.0:
        return None
    delta = records[right].ecef - records[left].ecef
    value = float(math.sqrt(float(delta @ delta)) / dt)
    return value if math.isfinite(value) and value >= 0.0 else None


def _summary(values: list[float]) -> dict[str, Any]:
    if not values:
        return {"count": 0, "median_m": None, "mean_m": None, "mad_m": None, "p95_abs_m": None}
    median = float(kaggle._percentile_linear_n_minus_1(values, 0.50))
    deviations = [abs(value - median) for value in values]
    return {
        "count": len(values),
        "median_m": median,
        "mean_m": sum(values) / len(values),
        "mad_m": float(kaggle._percentile_linear_n_minus_1(deviations, 0.50)),
        "p95_abs_m": float(kaggle._percentile_linear_n_minus_1([abs(value) for value in values], 0.95)),
    }


def _residual_analysis(records: dict[str, dict[str, Any]]) -> dict[str, Any]:
    per_route: dict[str, Any] = {}
    pooled: dict[str, list[float]] = {"along": [], "cross": [], "up": []}
    pooled_groups: dict[str, dict[str, list[float]]] = {}
    for dataset_id, route in records.items():
        truth_rows = _truth_records(Path(route["truth"]))
        truth_timestamps = [row.timestamp_ms for row in truth_rows]
        wls_rows = smoother._read_positions(Path(route["wls"]), 18)
        buckets: dict[str, dict[str, list[float]]] = {
            "overall": {key: [] for key in ("along", "cross", "up")},
            **{name: {key: [] for key in ("along", "cross", "up")} for name in SPEED_BINS + TURN_BINS},
        }
        matched = 0
        for position in wls_rows:
            truth = _nearest_truth(truth_rows, truth_timestamps, position.timestamp_ms)
            if truth is None:
                continue
            index = min(range(len(truth_rows)), key=lambda value: abs(truth_rows[value].timestamp_ms - truth.timestamp_ms))
            difference = position.ecef - truth.ecef
            east, north, up = residual._enu_basis(truth.ecef)
            east_error = float(difference @ east)
            north_error = float(difference @ north)
            up_error = float(difference @ up)
            heading = _bearing(truth_rows, index)
            if heading is None:
                continue
            heading_rad = math.radians(heading)
            values = {
                "along": east_error * math.sin(heading_rad) + north_error * math.cos(heading_rad),
                "cross": east_error * math.cos(heading_rad) - north_error * math.sin(heading_rad),
                "up": up_error,
            }
            if not all(math.isfinite(value) for value in values.values()):
                raise ResidualEvaluationError(f"non-finite local residual: {dataset_id}")
            matched += 1
            for key, value in values.items():
                buckets["overall"][key].append(value)
                pooled[key].append(value)
            speed = _speed(truth_rows, index)
            if speed is not None:
                speed_bin = "slow_lt_1mps" if speed < 1.0 else "mid_1_to_5mps" if speed < 5.0 else "fast_ge_5mps"
                for key, value in values.items():
                    buckets[speed_bin][key].append(value)
                    pooled_groups.setdefault(speed_bin, {name: [] for name in ("along", "cross", "up")})[key].append(value)
            turn_rate = _turn_rate(truth_rows, index)
            if turn_rate is not None:
                turn_bin = "straight_lt_5degps" if turn_rate < 5.0 else "turning_ge_5degps"
                for key, value in values.items():
                    buckets[turn_bin][key].append(value)
                    pooled_groups.setdefault(turn_bin, {name: [] for name in ("along", "cross", "up")})[key].append(value)
        per_route[dataset_id] = {
            "phone": route["phone"],
            "truth": route["truth_artifact"],
            "wls": route["wls_artifact"],
            "matched_epochs": matched,
            "groups": {
                group: {axis: _summary(values) for axis, values in axes.items()}
                for group, axes in buckets.items()
            },
        }
    return {
        "truth_free": False,
        "truth_role": "already-truth-opened-development-seven-only",
        "old_holdout_excluded": OLD_HOLDOUT_ID not in records,
        "local_frame": {
            "origin": "truth WGS84 ECEF",
            "along": "truth bearing, east*sin(bearing)+north*cos(bearing)",
            "cross": "right-of-heading, east*cos(bearing)-north*sin(bearing)",
            "up": "WGS84 ellipsoidal up",
        },
        "speed_bins": {"slow_lt_1mps": "speed < 1", "mid_1_to_5mps": "1 <= speed < 5", "fast_ge_5mps": "speed >= 5"},
        "turn_bins": {"straight_lt_5degps": "turn rate < 5 deg/s", "turning_ge_5degps": "turn rate >= 5 deg/s"},
        "per_route": per_route,
        "pooled": {axis: _summary(values) for axis, values in pooled.items()},
        "pooled_groups": {
            group: {axis: _summary(values) for axis, values in axes.items()}
            for group, axes in pooled_groups.items()
        },
        "bias_interpretation": "route/device vertical and horizontal signs vary; no fixed global shift is frozen",
        "chosen_estimator": "component-wise centered median, window 3, zero along-track shift",
    }


def _candidate_position(
    input_path: Path, output_dir: Path, candidate_id: str
) -> tuple[Path, dict[str, Any]]:
    output_path = output_dir / f"{candidate_id}.pos"
    manifest_path = output_dir / f"{candidate_id}.manifest.json"
    payload = residual.write_candidate(input_path, output_path, candidate_id, manifest_path=manifest_path)
    return output_path, payload


def _known_candidate_evaluation(
    records: dict[str, dict[str, Any]], output_dir: Path
) -> tuple[dict[str, Any], dict[str, Any], dict[str, Any]]:
    truth_cache = {
        dataset_id: smoother_eval._read_truth(Path(route["truth"]))
        for dataset_id, route in records.items()
    }
    native_scores: dict[str, dict[str, Any]] = {}
    wls_scores: dict[str, dict[str, Any]] = {}
    selector_scores: dict[str, dict[str, Any]] = {}
    for dataset_id, route in records.items():
        native_scores[dataset_id] = _score(Path(route["native"]), Path(route["device"]), truth_cache[dataset_id])
        wls_scores[dataset_id] = _score(Path(route["wls"]), Path(route["device"]), truth_cache[dataset_id])
        selected = route["native"] if route["decision"] == "native_stable" else route["wls"]
        selector_scores[dataset_id] = _score(Path(selected), Path(route["device"]), truth_cache[dataset_id])

    native_aggregate = _aggregate(list(native_scores.values()))
    wls_aggregate = _aggregate(list(wls_scores.values()))
    selector_aggregate = _aggregate(list(selector_scores.values()))
    candidates: dict[str, Any] = {}
    candidate_paths: dict[str, dict[str, Any]] = {}
    for spec in residual.PREDECLARED_CANDIDATES:
        candidate_id = spec.candidate_id
        per_route: dict[str, dict[str, Any]] = {}
        candidate_paths[candidate_id] = {}
        for dataset_id, route in records.items():
            route_dir = output_dir / "known" / stability_eval._safe_id(dataset_id) / "candidates"
            position_path, manifest = _candidate_position(Path(route["wls"]), route_dir, candidate_id)
            candidate_paths[candidate_id][dataset_id] = {
                "position": _artifact(position_path),
                "manifest": _artifact(Path(str(manifest["manifest"]["path"]))) if "manifest" in manifest else _artifact(route_dir / f"{candidate_id}.manifest.json"),
                "candidate": spec.__dict__,
            }
            selected_path = Path(route["native"]) if route["decision"] == "native_stable" else position_path
            per_route[dataset_id] = _score(selected_path, Path(route["device"]), truth_cache[dataset_id])
        aggregate = _aggregate(list(per_route.values()))
        eligible, failures = _non_regression(aggregate, selector_aggregate)
        candidates[candidate_id] = {
            "candidate": spec.__dict__,
            "aggregate": aggregate,
            "non_regression_vs_v3_selector": eligible,
            "non_regression_failures": failures,
            "scores": per_route,
        }
    eligible = [
        value for value in candidates.values() if value["non_regression_vs_v3_selector"]
    ]
    if not eligible:
        raise ResidualEvaluationError("no candidate is non-regressing against the v3 selector")
    selected = min(
        eligible,
        key=lambda value: (
            _metric(value["aggregate"], ("mean_horizontal_wgs84_p95_m",)),
            _metric(value["aggregate"], ("mean_horizontal_wgs84_p50_m",)),
            _metric(value["aggregate"], ("mean_vertical_p95_abs_m",)),
            _metric(value["aggregate"], ("mean_kaggle_diagnostic_m",)),
            str(value["candidate"]["candidate_id"]),
        ),
    )
    selected_id = str(selected["candidate"]["candidate_id"])
    selected_aggregate = selected["aggregate"]
    known_gate = {
        "passed": (
            _strict_horizontal_better(selected_aggregate, native_aggregate)
            and _strict_horizontal_better(selected_aggregate, wls_aggregate)
            and _metric(selected_aggregate, ("mean_availability_ratio",)) >= _metric(native_aggregate, ("mean_availability_ratio",)) - SIGNOFF_EPSILON
            and _metric(selected_aggregate, ("mean_availability_ratio",)) >= _metric(wls_aggregate, ("mean_availability_ratio",)) - SIGNOFF_EPSILON
            and _metric(selected_aggregate, ("mean_vertical_p95_abs_m",)) <= 45.0 + SIGNOFF_EPSILON
        ),
        "failures": [],
        "selected_candidate_id": selected_id,
        "requirements": {
            "strict_horizontal_vs_native_only": True,
            "strict_horizontal_vs_wls_only": True,
            "vertical_p95_profile_threshold_m": 45.0,
            "availability_non_regression": True,
        },
        "native_only": native_aggregate,
        "wls_only": wls_aggregate,
        "v3_selector": selector_aggregate,
        "selected_candidate": selected_aggregate,
    }
    if not known_gate["passed"]:
        if not _strict_horizontal_better(selected_aggregate, native_aggregate):
            known_gate["failures"].append("not_strictly_better_than_native_only")
        if not _strict_horizontal_better(selected_aggregate, wls_aggregate):
            known_gate["failures"].append("not_strictly_better_than_wls_only")
        if _metric(selected_aggregate, ("mean_vertical_p95_abs_m",)) > 45.0 + SIGNOFF_EPSILON:
            known_gate["failures"].append("vertical_p95_profile_threshold")
    return (
        {
            "native_only": native_aggregate,
            "wls_only": wls_aggregate,
            "v3_selector": selector_aggregate,
            "candidates": candidates,
            "selected_candidate_id": selected_id,
            "selected_candidate_paths": candidate_paths[selected_id],
            "candidate_set": [spec.__dict__ for spec in residual.PREDECLARED_CANDIDATES],
        },
        known_gate,
        {"truth_cache": truth_cache, "candidate_paths": candidate_paths},
    )


def _validation_truth_free(
    archive_path: Path,
    profile: dict[str, Any],
    candidate: dict[str, Any],
    archive_sha256: str,
    output_dir: Path,
    *,
    validation_id: str = NEW_VALIDATION_ID,
    selected_candidate_id: str = "median3_shift_0",
    next_holdout_id: str = NEXT_HOLDOUT_ID,
) -> dict[str, Any]:
    route_dir = output_dir / "validation" / "routes" / stability_eval._safe_id(validation_id)
    if route_dir.exists() and any(route_dir.iterdir()):
        raise ResidualEvaluationError(f"refusing to overwrite validation artifacts: {route_dir}")
    member_hashes = generalization._discover_member_hashes(
        archive_path,
        {key: generalization._member_names(candidate["route"], candidate["phone"])[key] for key in ("device_gnss", "device_imu", "broadcast_nav")},
    )
    candidate_profile = stability_eval._candidate_profile_without_truth(
        profile, candidate, archive_sha256, member_hashes
    )
    materialized = stability_eval._materialize_truth_free_inputs(
        archive_path,
        candidate,
        output_dir / "validation",
        archive_sha256,
        member_hashes,
    )
    profile_path = materialized["root"] / "candidate_profile.json"
    _atomic_json(profile_path, candidate_profile)
    screen = stability_eval._screen_device(materialized["inputs"] / "device_gnss.csv", validation_id)
    generated = conservative._run_adapter_and_spp(
        candidate,
        candidate_profile,
        materialized,
        profile_path,
        output_dir / "validation",
        max_epochs=-1,
    )
    device_path = materialized["inputs"] / "device_gnss.csv"
    route_root = output_dir / "validation" / "routes" / stability_eval._safe_id(validation_id)
    wls_dir = route_root / "wls"
    before_rss = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss
    started = time.perf_counter()
    wls_manifest_payload = wls.extract_to_directory(
        device_path,
        wls_dir,
        skip_epochs=1,
        max_epochs=-1,
        role="development",
        dataset_id=validation_id,
    )
    wls_wall = time.perf_counter() - started
    after_rss = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss
    wls_path = wls_dir / "wls.pos"
    native_dir = route_root / "native-segment-stability"
    native_path, native_report, stability, native_artifacts = stability_eval._write_native_stability_outputs(
        generated["position"], device_path, native_dir
    )
    selector_dir = route_root / "selector"
    selector_result = selector.select_and_publish(
        native_path,
        native_report,
        wls_path,
        wls_dir / "wls_manifest.json",
        device_path,
        selector_dir,
        phone=candidate["phone"],
        dataset_id=validation_id,
        skip_epochs=1,
    )
    selected_id = residual.candidate_spec(selected_candidate_id).candidate_id
    candidate_dir = route_root / "residual-candidate" / selected_id
    candidate_path, candidate_manifest = _candidate_position(wls_path, candidate_dir, selected_id)
    selected_path = native_path if selector_result["decision"] == "native_stable" else candidate_path
    submission_path = route_root / "submission.csv"
    submission_manifest_path = route_root / "submission.manifest.json"
    submission_manifest = kaggle.generate_submission(
        selected_path,
        submission_path,
        candidate["phone"],
        device_gnss_path=device_path,
        dataset_id=validation_id,
        skip_epochs=1,
        manifest_path=submission_manifest_path,
    )
    truth_path = materialized["inputs"] / "ground_truth.csv"
    if truth_path.exists():
        raise ResidualEvaluationError("validation truth appeared before truth-free sealing")
    truth_free_manifest = {
        "schema_version": "smartphone-r5-wls-residual-validation-truth-free.v1",
        "dataset_id": validation_id,
        "truth_used": False,
        "truth_opened": False,
        "next_holdout_id": next_holdout_id,
        "next_holdout_materialized": False,
        "next_holdout_truth_opened": False,
        "e1_hatch_contract": {"signal": "GAL_E1_C_P", "hatch_window_s": 30},
        "wls_integrity": {"manifest": _artifact(wls_dir / "wls_manifest.json"), "summary": _artifact(wls_dir / "wls_summary.json")},
        "native_stability": {
            "position": _artifact(native_path),
            "report": _artifact(native_report),
            "manifest": _artifact(native_dir / "smoother_manifest.json"),
            "population": stability["population"],
        },
        "selector": {
            "decision": selector_result["decision"],
            "reason": selector_result["reason"],
            "manifest": _artifact(selector_dir / "selector_manifest.json"),
            "selected_position": _artifact(selector_dir / "selected.pos"),
            "selected_submission": _artifact(selector_dir / "submission.csv"),
        },
        "residual_candidate": {
            "candidate": residual.candidate_spec(selected_id).__dict__,
            "manifest": _artifact(candidate_dir / f"{selected_id}.manifest.json"),
            "position": _artifact(candidate_path),
        },
        "submission": {
            "manifest": _artifact(submission_manifest_path),
            "submission": _artifact(submission_path),
        },
        "adapter_summary": _artifact(Path(generated["adapter_summary"])),
        "spp_summary": _artifact(Path(generated["spp_summary"])),
        "materialization": _artifact(Path(materialized["manifest"])),
        "screen": screen,
        "timing": {
            "adapter_spp_stages": generated["stages"],
            "wls_wall_s": wls_wall,
            "peak_rss_kb": max(int(before_rss), int(after_rss)),
        },
    }
    truth_free_path = output_dir / "validation" / "truth_free_manifest.json"
    _atomic_json(truth_free_path, truth_free_manifest)
    # This is the sole validation truth materialization/read in this run.  It
    # occurs only after all output hashes above are fixed.
    truth_artifact = stability_eval._extract_member_atomic(
        archive_path,
        generalization._member_names(candidate["route"], candidate["phone"])["ground_truth"],
        truth_path,
        expected_size=int(candidate["central_directory_files"]["ground_truth.csv"]["file_size"]),
    )
    truth = smoother_eval._read_truth(truth_path)
    baseline_path = native_path if selector_result["decision"] == "native_stable" else wls_path
    baseline_score = _score(baseline_path, device_path, truth)
    candidate_score = _score(selected_path, device_path, truth)
    native_score = _score(native_path, device_path, truth)
    wls_score = _score(wls_path, device_path, truth)
    profile_thresholds = dict(profile.get("thresholds", {}))
    failures: list[str] = []
    nonregression, nonregression_failures = _non_regression(candidate_score, baseline_score)
    failures.extend(nonregression_failures)
    checks = (
        ("availability_min", _metric(candidate_score, ("availability_ratio",)) >= float(profile_thresholds.get("availability_min", 0.0)) - SIGNOFF_EPSILON),
        ("truth_coverage_min", _metric(candidate_score, ("truth_coverage_ratio",)) >= float(profile_thresholds.get("truth_coverage_min", 0.0)) - SIGNOFF_EPSILON),
        ("horizontal_median_max", _metric(candidate_score, ("horizontal_wgs84_m", "p50_m")) <= float(profile_thresholds.get("horizontal_median_max", math.inf)) + SIGNOFF_EPSILON),
        ("horizontal_p95_max", _metric(candidate_score, ("horizontal_wgs84_m", "p95_m")) <= float(profile_thresholds.get("horizontal_p95_max", math.inf)) + SIGNOFF_EPSILON),
        ("vertical_p95_max", _metric(candidate_score, ("vertical_p95_abs_m",)) <= float(profile_thresholds.get("vertical_p95_max", math.inf)) + SIGNOFF_EPSILON),
    )
    signoff = {name: passed for name, passed in checks}
    failures.extend(name for name, passed in checks if not passed)
    return {
        "dataset_id": validation_id,
        "truth_free_manifest": {"path": str(truth_free_path), "sha256": _sha256(truth_free_path)},
        "truth_free_artifacts_sealed_before_truth": True,
        "truth_open_count": 1,
        "truth_artifact": truth_artifact,
        "selector_decision": selector_result["decision"],
        "selector_reason": selector_result["reason"],
        "selected_candidate_id": selected_id,
        "selected_position": _artifact(selected_path),
        "selected_submission": _artifact(submission_path),
        "scores": {
            "native_segment_stability": native_score,
            "wls_raw": wls_score,
            "v3_selector_baseline": baseline_score,
            "residual_candidate_selector": candidate_score,
        },
        "gate": {
            "non_regression_vs_v3_selector": nonregression,
            "non_regression_failures": nonregression_failures,
            "signoff": signoff,
            "passed": not failures,
            "failures": failures,
            "official_like_horizontal_metric": "wgs84_vincenty__linear_n_minus_1",
        },
        "materialization": {
            "validation_inputs": _artifact(Path(materialized["manifest"])),
            "truth": truth_artifact,
        },
        "timing": {
            "adapter_spp_stages": generated["stages"],
            "wls_wall_s": wls_wall,
            "peak_rss_kb": max(int(before_rss), int(after_rss)),
        },
    }


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        prog=os.environ.get("GNSS_CLI_NAME", "gnss smartphone-wls-residual-eval")
    )
    parser.add_argument("--archive", type=Path, default=DEFAULT_ARCHIVE)
    parser.add_argument("--profile", type=Path, default=DEFAULT_PROFILE)
    parser.add_argument("--inventory", type=Path, default=DEFAULT_INVENTORY)
    parser.add_argument("--selection-record", type=Path, default=DEFAULT_SELECTION_RECORD)
    parser.add_argument("--existing-report", type=Path, default=DEFAULT_EXISTING_REPORT)
    parser.add_argument("--device-family-report", type=Path, default=DEFAULT_DEVICE_FAMILY_REPORT)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    return parser.parse_args(argv)


def run(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    try:
        archive = _absolute(args.archive)
        profile_path = _absolute(args.profile)
        inventory_path = _absolute(args.inventory)
        selection_path = _absolute(args.selection_record)
        existing_path = _absolute(args.existing_report)
        prior_path = _absolute(args.device_family_report)
        output_dir = _absolute(args.output_dir)
        if output_dir.exists() and any(output_dir.iterdir()):
            raise ResidualEvaluationError(f"refusing to overwrite non-empty output: {output_dir}")
        selection_record = _load_selection_record(selection_path)
        inventory, validation_metadata = _verify_central_inventory(archive, inventory_path, selection_record)
        profile = stability_eval.previous._load_profile(profile_path)
        if profile.get("datasets", {}).get("holdout", {}).get("id") != HOLDOUT_ID:
            raise ResidualEvaluationError("profile holdout differs from sealed contract")
        existing_report = _load_json(existing_path, "existing WLS report")
        prior_report = _load_json(prior_path, "prior device-family WLS report")
        records = _known_route_records(existing_report, prior_report)
        started = time.perf_counter()
        analysis = _residual_analysis(records)
        train_dir = output_dir / "train"
        candidate_evaluation, known_gate, train_context = _known_candidate_evaluation(records, train_dir)
        # The new validation route is the only post-selection payload access.
        archive_hash = _sha256(archive)
        validation = _validation_truth_free(archive, profile, validation_metadata, archive_hash, output_dir)
        promotion = (
            "promote-development-only"
            if known_gate["passed"] and validation["gate"]["passed"]
            else "no-go-validation-or-known-aggregate-gate"
        )
        inventory_path_out = output_dir / "archive_inventory.json"
        inventory_payload = {
            "schema_version": "smartphone-r5-wls-residual-central-inventory-use.v1",
            "archive": {
                "path": str(archive),
                "sha256": archive_hash,
                "central_directory_inventory_sha256": _sha256(inventory_path),
                "central_directory_only_at_selection": True,
                "member_content_read_for_next_holdout": False,
                "next_holdout_materialized": False,
                "next_holdout_truth_opened": False,
            },
            "selection_record": {"path": str(selection_path), "sha256": _sha256(selection_path)},
            "new_validation": validation_metadata,
            "next_holdout": {"dataset_id": NEXT_HOLDOUT_ID, "materialization_forbidden": True, "truth_open_forbidden": True},
            "inventory_record_count": len(inventory.get("train", {}).get("records", [])),
            "inventory_payload_not_reused_for_nonselected_members": True,
        }
        _atomic_json(inventory_path_out, inventory_payload)
        report = {
            "schema_version": SCHEMA_VERSION,
            "decision": "development-only-wls-residual-release-research",
            "promotion_decision": promotion,
            "archive": {"path": str(archive), "sha256": archive_hash},
            "selection_record": {"path": str(selection_path), "sha256": _sha256(selection_path)},
            "inventory": {"path": str(inventory_path), "sha256": _sha256(inventory_path), "payload_free": True},
            "roles": {
                "residual_analysis_and_candidate_train": list(KNOWN_SEVEN_IDS),
                "new_validation": NEW_VALIDATION_ID,
                "next_holdout": NEXT_HOLDOUT_ID,
                "old_holdout_excluded": OLD_HOLDOUT_ID,
            },
            "old_holdout_metrics_used_for_selection": False,
            "old_holdout_used_as_train_or_candidate": False,
            "next_holdout": {
                "dataset_id": NEXT_HOLDOUT_ID,
                "central_metadata_only": True,
                "materialized": False,
                "truth_opened": False,
            },
            "residual_analysis": analysis,
            "candidate_evaluation": candidate_evaluation,
            "known_seven_route_gate": known_gate,
            "new_validation": validation,
            "candidate_contract": {
                "candidate_count": len(residual.PREDECLARED_CANDIDATES),
                "candidate_ids": [spec.candidate_id for spec in residual.PREDECLARED_CANDIDATES],
                "estimator": "component-wise centered median; robust against isolated WLS residual outliers",
                "windows_epochs": [1, 3, 5],
                "along_track_shift_m": [-1.0, 0.0, 1.0],
                "heading_source": "WLS ECEF trajectory only",
                "truth_free_runtime": True,
                "device_model_free": True,
            },
            "promotion_scope": "development-only recommended Kaggle submission lane; production RTK/SPP default unchanged",
            "timing": {
                "total_wall_s": time.perf_counter() - started,
                "known_route_count": len(KNOWN_SEVEN_IDS),
                "validation_truth_open_count": validation["truth_open_count"],
            },
        }
        report_path = output_dir / "wls_residual_release_report.json"
        _atomic_json(report_path, report)
        manifest = {
            "schema_version": MANIFEST_SCHEMA_VERSION,
            "truth_free_candidate_generation": True,
            "truth_free_validation_generation_sealed_before_truth": True,
            "truth_open_count_new_validation": validation["truth_open_count"],
            "holdout_content_opened": False,
            "holdout_truth_opened": False,
            "next_holdout_materialized": False,
            "selection_record": {"path": str(selection_path), "sha256": _sha256(selection_path)},
            "inventory": {"path": str(inventory_path_out), "sha256": _sha256(inventory_path_out)},
            "report": {"path": str(report_path), "sha256": _sha256(report_path)},
            "truth_free_validation_manifest": validation["truth_free_manifest"],
        }
        manifest_path = output_dir / "wls_residual_release_manifest.json"
        _atomic_json(manifest_path, manifest)
        print(f"Smartphone WLS residual release evaluation complete: {report_path}")
        return 0
    except (ResidualEvaluationError, OSError, ValueError, wls.WlsPositionError, smoother.SmootherError) as exc:
        print(f"Smartphone WLS residual release evaluation failed: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(run())
