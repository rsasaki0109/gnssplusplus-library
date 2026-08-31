#!/usr/bin/env python3
"""Evaluate WLS/native lane choice on one previously unused Pixel7Pro route.

The route is frozen from ZIP central-directory metadata before any selected
member is opened.  The selected route is then materialized and both the
truth-free native Galileo-E1/Hatch30 lane and the truth-free WLS lane are
published.  Ground truth is parsed only after every candidate artifact has
been written.  A device-model-only selector is considered only when the new
Pixel7Pro route reproduces the existing main-route native advantage and the
combined seven-route sign-off remains non-regressive.

This command is development-only.  It never includes the sealed holdout and
does not change the production/default smartphone lane automatically.
"""

from __future__ import annotations

import argparse
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
import gnss_smartphone_reacquisition_conservative_eval as conservative  # noqa: E402
import gnss_smartphone_reacquisition_eval as previous  # noqa: E402
import gnss_smartphone_segment_stability as segment_stability  # noqa: E402
import gnss_smartphone_trajectory_smoother as smoother  # noqa: E402
import gnss_smartphone_trajectory_smoother_eval as smoother_eval  # noqa: E402
import gnss_smartphone_wls as wls  # noqa: E402
import gnss_smartphone_wls_eval as wls_eval  # noqa: E402


SCHEMA_VERSION = "smartphone-r5-wls-device-family-evaluation.v1"
SELECTION_SCHEMA = "smartphone-r5-wls-device-family-selection.v1"
DEFAULT_ARCHIVE = ROOT / "data" / "gsdc2023" / "cache" / "dataset_2023.zip"
DEFAULT_PROFILE = ROOT / "configs" / "benchmarks" / "smartphone_r5_gsdc2023.json"
DEFAULT_INVENTORY = (
    ROOT / "output" / "smartphone-r5" / "generalization-v6" / "archive_inventory.json"
)
DEFAULT_OUTPUT_DIR = ROOT / "output" / "smartphone-r5" / "wls-device-family-v1"
DEFAULT_SELECTION_RECORD = (
    ROOT
    / "docs"
    / "use_cases"
    / "records"
    / "smartphone_r5_wls_device_family_selection.json"
)
DEFAULT_EXISTING_REPORT = (
    ROOT / "output" / "smartphone-r5" / "wls-position-v1" / "wls_position_report.json"
)
HOLDOUT_ID = previous.HOLDOUT_ID
MAIN_ID = previous.MAIN_ID
EXISTING_IDS = (
    "2021-01-04-21-50-us-ca-e1highway280driveroutea/mi8",
    "2022-08-04-20-07-us-ca-sjc-q/pixel5",
    "2023-03-08-21-34-us-ca-mtv-u/pixel6pro",
    "2021-03-16-20-40-us-ca-mtv-b/pixel4xl",
    "2021-07-14-20-50-us-ca-mtv-e/pixel4",
    MAIN_ID,
)
CANDIDATE_ID = "2022-11-15-00-53-us-ca-mtv-a/pixel7pro"
SELECTOR_PIXEL7PRO = "native_segment_stability"
SELECTOR_OTHER_VERIFIED = "wls_raw"
SELECTOR_UNKNOWN = "wls_raw"
SEGMENT_MAX_REJECTS = 15
SEGMENT_MAX_PREDICTION_DURATION_S = 15.0
BASELINE_CONFIG = {
    "process_noise": 1.0,
    "measurement_floor_m": 1.0,
    "outlier_gate_sigma": 5.0,
    "segment_gap_s": 10.0,
}
DIAGNOSTIC_KEYS = wls_eval.DIAGNOSTIC_KEYS


class DeviceFamilyEvaluationError(ValueError):
    """Raised when the frozen route or evaluation contract is violated."""


def _sha256(path: Path) -> str:
    if not path.is_file():
        raise DeviceFamilyEvaluationError(f"missing file: {path}")
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _atomic_json(path: Path, payload: dict[str, Any]) -> None:
    smoother._atomic_write(
        path,
        (json.dumps(payload, indent=2, sort_keys=True) + "\n").encode("utf-8"),
    )


def _artifact(path: Path) -> dict[str, Any]:
    return {"path": str(path), "sha256": _sha256(path), "bytes": path.stat().st_size}


def _safe_id(dataset_id: str) -> str:
    return dataset_id.replace("/", "__")


def _load_selection_record(path: Path) -> dict[str, Any]:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise DeviceFamilyEvaluationError(f"invalid selection record: {path}") from exc
    if not isinstance(payload, dict) or payload.get("schema_version") != SELECTION_SCHEMA:
        raise DeviceFamilyEvaluationError("selection record schema is not the frozen device-family v1")
    if payload.get("status") != "selection-frozen-before-evaluation":
        raise DeviceFamilyEvaluationError("selection record is not frozen before evaluation")
    selected = payload.get("selected_candidate")
    if not isinstance(selected, dict) or selected.get("dataset_id") != CANDIDATE_ID:
        raise DeviceFamilyEvaluationError("selection record does not freeze the expected Pixel7Pro route")
    sealed = payload.get("sealed_data_policy")
    if not isinstance(sealed, dict) or sealed.get("designated_holdout_id") != HOLDOUT_ID:
        raise DeviceFamilyEvaluationError("selection record holdout differs from the sealed profile")
    if sealed.get("holdout_content_opened") or sealed.get("holdout_truth_opened"):
        raise DeviceFamilyEvaluationError("selection record claims holdout access")
    excluded = tuple(payload.get("previously_used_ids_excluded", ()))
    if CANDIDATE_ID in excluded or MAIN_ID in excluded and CANDIDATE_ID == MAIN_ID:
        raise DeviceFamilyEvaluationError("selected route appears in the exclusion set")
    post_evaluation = payload.get("post_evaluation")
    if post_evaluation is not None and not isinstance(post_evaluation, dict):
        raise DeviceFamilyEvaluationError("selection record post-evaluation block is malformed")
    return payload


def _load_frozen_inventory(path: Path, selection_record: dict[str, Any]) -> dict[str, Any]:
    """Verify and load the metadata-only inventory that froze the route choice."""

    archive_record = selection_record.get("archive")
    if not isinstance(archive_record, dict):
        raise DeviceFamilyEvaluationError("selection record has no frozen archive metadata")
    expected_sha = str(archive_record.get("central_directory_inventory_sha256", ""))
    if not expected_sha:
        raise DeviceFamilyEvaluationError("selection record has no frozen inventory hash")
    actual_sha = _sha256(path)
    if actual_sha != expected_sha:
        raise DeviceFamilyEvaluationError("frozen central-directory inventory hash changed")
    try:
        inventory = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise DeviceFamilyEvaluationError(f"invalid frozen inventory: {path}") from exc
    if not isinstance(inventory, dict):
        raise DeviceFamilyEvaluationError("frozen inventory is not an object")
    archive = inventory.get("archive")
    if not isinstance(archive, dict):
        raise DeviceFamilyEvaluationError("frozen inventory has no archive metadata")
    if archive.get("central_directory_only") is not True:
        raise DeviceFamilyEvaluationError("frozen inventory was not central-directory-only")
    if archive.get("member_content_read") is not False:
        raise DeviceFamilyEvaluationError("frozen inventory claims member content access")
    return inventory


def _verify_central_candidate(
    inventory: dict[str, Any], selection_record: dict[str, Any]
) -> dict[str, Any]:
    rows = {
        str(row.get("dataset_id")): row
        for row in inventory.get("train", {}).get("records", [])
        if isinstance(row, dict)
    }
    candidate = rows.get(CANDIDATE_ID)
    if candidate is None:
        raise DeviceFamilyEvaluationError("frozen Pixel7Pro candidate is absent from central inventory")
    if candidate.get("dataset_id") in (*EXISTING_IDS, HOLDOUT_ID):
        raise DeviceFamilyEvaluationError("candidate collides with a used or sealed route")
    if candidate.get("phone") != "pixel7pro":
        raise DeviceFamilyEvaluationError("candidate is not an exact Pixel7Pro route")
    if not candidate.get("required_files_complete") or not candidate.get("broadcast_nav_present"):
        raise DeviceFamilyEvaluationError("candidate lacks complete phone files or unique broadcast nav")
    if int(candidate.get("broadcast_nav_duplicate_count", 0)) != 0:
        raise DeviceFamilyEvaluationError("candidate broadcast nav is duplicated")
    frozen = selection_record.get("selected_candidate")
    frozen_members = dict(frozen.get("central_directory_members", {}))
    actual_members = dict(candidate.get("central_directory_files", {}))
    actual_nav = candidate.get("central_directory_broadcast_nav")
    frozen_phone_members = {
        name: metadata
        for name, metadata in frozen_members.items()
        if name in generalization.REQUIRED_PHONE_MEMBERS
    }
    if frozen_phone_members != actual_members:
        raise DeviceFamilyEvaluationError("candidate central-directory phone metadata changed after freeze")
    frozen_nav = frozen_members.get("brdc.nav")
    if frozen_nav is None:
        frozen_nav = frozen.get("central_directory_broadcast_nav")
    if frozen_nav != actual_nav:
        raise DeviceFamilyEvaluationError("candidate central-directory nav metadata changed after freeze")
    return candidate


def _write_lane_outputs(
    result: smoother.SmoothingResult,
    stability: dict[str, Any],
    output_dir: Path,
    raw_position: Path,
    device_path: Path,
    lane: str,
) -> dict[str, Any]:
    output_dir.mkdir(parents=True, exist_ok=True)
    config = smoother.SmootherConfig(**BASELINE_CONFIG)
    smoother.write_outputs(
        result,
        config,
        output_dir,
        position_path=raw_position,
        device_path=device_path,
        skip_epochs=1,
        leap_seconds=18,
    )
    stability_path = output_dir / "segment_stability.json"
    _atomic_json(
        stability_path,
        {
            **stability,
            "truth_used": False,
            "position_lane": lane,
            "raw_position": _artifact(raw_position),
        },
    )
    manifest_path = output_dir / "smoother_manifest.json"
    try:
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise DeviceFamilyEvaluationError(f"invalid smoother manifest: {manifest_path}") from exc
    manifest["position_lane"] = lane
    manifest["segment_stability"] = {
        "path": str(stability_path),
        "sha256": _sha256(stability_path),
        "thresholds": stability["thresholds"],
        "population": stability["population"],
    }
    _atomic_json(manifest_path, manifest)
    return {
        "position": _artifact(output_dir / "smoothed.pos"),
        "stability": _artifact(stability_path),
        "manifest": _artifact(manifest_path),
    }


def _run_truth_free_route(
    archive_path: Path,
    profile: dict[str, Any],
    candidate: dict[str, Any],
    archive_sha256: str,
    output_dir: Path,
    *,
    max_epochs: int,
) -> dict[str, Any]:
    member_hashes = generalization._discover_member_hashes(
        archive_path,
        generalization._member_names(candidate["route"], candidate["phone"]),
    )
    candidate_profile = generalization._candidate_profile(
        profile, candidate, archive_sha256, member_hashes
    )
    materialized = generalization.materialize_candidate(
        archive_path,
        candidate_profile,
        candidate,
        archive_sha256,
        output_dir / "materialized",
        member_hashes,
    )
    route_root = output_dir / "routes" / _safe_id(CANDIDATE_ID)
    route_root.mkdir(parents=True, exist_ok=True)
    profile_path = route_root / "candidate_profile.json"
    _atomic_json(profile_path, candidate_profile)
    # Reuse the already-audited truth-free adapter/SPP runner.  Keeping this
    # call in one implementation prevents the device-family lane from
    # silently diverging from the conservative generalization contract.
    generated = conservative._run_adapter_and_spp(
        candidate,
        candidate_profile,
        materialized,
        profile_path,
        output_dir,
        max_epochs=max_epochs,
    )
    device_path = Path(generated["device"])
    native_position_path = Path(generated["position"])
    wls_dir = route_root / "wls"
    rss_before = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss
    wls_started = time.perf_counter()
    wls_manifest_payload = wls.extract_to_directory(
        device_path,
        wls_dir,
        skip_epochs=1,
        max_epochs=max_epochs,
        role="development",
        dataset_id=CANDIDATE_ID,
    )
    wls_wall = time.perf_counter() - wls_started
    rss_after = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss
    wls_position_path = wls_dir / "wls.pos"
    wls_summary_path = wls_dir / "wls_summary.json"
    native_positions = smoother._read_positions(native_position_path, 18)
    wls_positions = smoother._read_positions(wls_position_path, 18)
    device_epochs = smoother._read_device_epochs(device_path, 1)
    if max_epochs > 0:
        device_epochs = device_epochs[:max_epochs]
    if [row.timestamp_ms for row in native_positions] != device_epochs:
        raise DeviceFamilyEvaluationError("native POS keys do not equal candidate device keys")
    if [row.timestamp_ms for row in wls_positions] != device_epochs:
        raise DeviceFamilyEvaluationError("WLS POS keys do not equal candidate device keys")

    native_result = smoother.smooth_positions(
        native_positions, device_epochs, smoother.SmootherConfig(**BASELINE_CONFIG)
    )
    wls_result = smoother.smooth_positions(
        wls_positions, device_epochs, smoother.SmootherConfig(**BASELINE_CONFIG)
    )
    native_stable, native_stability = segment_stability.apply_segment_stability(
        native_result,
        native_positions,
        device_epochs,
        max_consecutive_rejects=SEGMENT_MAX_REJECTS,
        max_prediction_duration_s=SEGMENT_MAX_PREDICTION_DURATION_S,
        reject_fraction_max=None,
        measurement_floor_m=BASELINE_CONFIG["measurement_floor_m"],
        leap_seconds=18,
    )
    wls_stable, wls_stability = segment_stability.apply_segment_stability(
        wls_result,
        wls_positions,
        device_epochs,
        max_consecutive_rejects=SEGMENT_MAX_REJECTS,
        max_prediction_duration_s=SEGMENT_MAX_PREDICTION_DURATION_S,
        reject_fraction_max=None,
        measurement_floor_m=BASELINE_CONFIG["measurement_floor_m"],
        leap_seconds=18,
    )
    native_raw_path = route_root / "native_raw.pos"
    wls_raw_path = route_root / "wls_raw.pos"
    smoother._atomic_write(native_raw_path, native_position_path.read_bytes())
    smoother._atomic_write(wls_raw_path, wls_position_path.read_bytes())
    native_artifacts = _write_lane_outputs(
        native_stable,
        native_stability,
        route_root / "native-segment-stability",
        native_raw_path,
        device_path,
        "native-galileo-e1-hatch30-segment-stability",
    )
    wls_artifacts = _write_lane_outputs(
        wls_stable,
        wls_stability,
        route_root / "wls-segment-stability",
        wls_raw_path,
        device_path,
        "android-handset-wls-ecef-segment-stability",
    )
    # These two files are copied only after the truth-free lane is complete;
    # they are the raw rows scored later and do not contain truth.
    native_raw_copy = route_root / "native-raw.pos"
    wls_raw_copy = route_root / "wls-raw.pos"
    smoother._atomic_write(native_raw_copy, native_position_path.read_bytes())
    smoother._atomic_write(wls_raw_copy, wls_position_path.read_bytes())
    return {
        "dataset_id": CANDIDATE_ID,
        "candidate": candidate,
        "profile": {"path": str(profile_path), "sha256": _sha256(profile_path)},
        "materialization": {
            "manifest": str(materialized["manifest"]),
            "sha256": materialized["manifest_sha256"],
            "truth_used_for_materialization": False,
        },
        "inputs": {
            "device_gnss": _artifact(device_path),
            "device_imu": _artifact(device_path.with_name("device_imu.csv")),
            "ground_truth": _artifact(device_path.with_name("ground_truth.csv")),
            "broadcast_nav": _artifact(device_path.parent / "brdc.nav"),
        },
        "truth": device_path.with_name("ground_truth.csv"),
        "device": device_path,
        "epochs": device_epochs,
        "native_positions": native_positions,
        "wls_positions": wls_positions,
        "native_stable": native_stable,
        "wls_stable": wls_stable,
        "native_raw_path": native_raw_copy,
        "wls_raw_path": wls_raw_copy,
        "native_stability": native_stability,
        "wls_stability": wls_stability,
        "wls_manifest_payload": wls_manifest_payload,
        "wls_manifest": _artifact(wls_dir / "wls_manifest.json"),
        "wls_summary": _artifact(wls_summary_path),
        "native_artifacts": native_artifacts,
        "wls_artifacts": wls_artifacts,
        "truth_free_generation_complete": True,
        "timing": {
            "adapter_spp_stages": generated["stages"],
            "wls_extraction_wall_s": wls_wall,
            "native_smoother_wall_s": native_result.elapsed_s,
            "wls_smoother_wall_s": wls_result.elapsed_s,
            "peak_rss_kb_process_max": max(int(rss_before), int(rss_after)),
        },
    }


def _raw_rows(positions: list[smoother.PositionRow]) -> list[smoother.SmoothedRow]:
    return previous._raw_rows(positions)


def _score(rows: list[smoother.SmoothedRow], positions: list[smoother.PositionRow], epochs: list[int], truth: dict[int, tuple[float, float, float]]) -> dict[str, Any]:
    return wls_eval._score(rows, positions, epochs, truth)


def _score_candidate(route: dict[str, Any]) -> dict[str, Any]:
    truth_path = Path(route["truth"])
    truth = smoother_eval._read_truth(truth_path)
    positions = route["wls_positions"]
    epochs = route["epochs"]
    return {
        "wls_raw": _score(_raw_rows(route["wls_positions"]), positions, epochs, truth),
        "native_raw": _score(_raw_rows(route["native_positions"]), route["native_positions"], epochs, truth),
        "wls_segment_stability": _score(route["wls_stable"].rows, positions, epochs, truth),
        "native_segment_stability": _score(route["native_stable"].rows, route["native_positions"], epochs, truth),
    }


def _metric(metrics: dict[str, Any], path: tuple[str, ...]) -> float:
    value: Any = metrics
    for key in path:
        value = value[key]
    if value is None or not math.isfinite(float(value)):
        return math.inf
    return float(value)


def _non_regression(candidate: dict[str, Any], reference: dict[str, Any]) -> dict[str, Any]:
    failures: list[str] = []
    if _metric(candidate, ("availability_ratio",)) < _metric(reference, ("availability_ratio",)) - 1e-12:
        failures.append("availability_regression")
    for path, name in (
        (("horizontal_wgs84_m", "p50_m"), "h_median_regression"),
        (("horizontal_wgs84_m", "p95_m"), "h_p95_regression"),
        (("vertical_p95_abs_m",), "v_p95_regression"),
    ):
        if _metric(candidate, path) > _metric(reference, path) + 1e-12:
            failures.append(name)
    for key in DIAGNOSTIC_KEYS:
        if _metric(candidate, ("kaggle_diagnostic_score_variants_m", key)) > _metric(
            reference, ("kaggle_diagnostic_score_variants_m", key)
        ) + 1e-12:
            failures.append(f"{key}_regression")
    return {"non_regression_passed": not failures, "failures": failures}


def _aggregate_non_regression(
    candidate: dict[str, Any], reference: dict[str, Any]
) -> dict[str, Any]:
    """Compare aggregate metrics using the aggregate schema's mean fields."""

    failures: list[str] = []
    if _metric(candidate, ("mean_availability_ratio",)) < _metric(
        reference, ("mean_availability_ratio",)
    ) - 1e-12:
        failures.append("availability_regression")
    for path, name in (
        (("mean_horizontal_wgs84_p50_m",), "h_median_regression"),
        (("mean_horizontal_wgs84_p95_m",), "h_p95_regression"),
        (("mean_vertical_p95_abs_m",), "v_p95_regression"),
    ):
        if _metric(candidate, path) > _metric(reference, path) + 1e-12:
            failures.append(name)
    for key in DIAGNOSTIC_KEYS:
        if _metric(
            candidate, ("mean_kaggle_diagnostic_score_variants_m", key)
        ) > _metric(
            reference, ("mean_kaggle_diagnostic_score_variants_m", key)
        ) + 1e-12:
            failures.append(f"{key}_regression")
    return {"non_regression_passed": not failures, "failures": failures}


def _aggregate(metrics: list[dict[str, Any]]) -> dict[str, Any]:
    return wls_eval._aggregate(metrics)


def _selector_lane(device_model: str | None, profile: dict[str, Any]) -> str:
    """Select a lane using only the device metadata and a frozen profile."""

    mapping = dict(profile.get("device_lane_map", {}))
    if device_model in mapping:
        return str(mapping[device_model])
    return str(profile.get("unknown_device_lane", SELECTOR_UNKNOWN))


def _selector_profile() -> dict[str, Any]:
    return {
        "schema_version": "smartphone-r5-wls-device-lane-selector.v1",
        "scope": "development-only candidate; truth-free device metadata selection",
        "device_lane_map": {
            "pixel7pro": SELECTOR_PIXEL7PRO,
            "mi8": SELECTOR_OTHER_VERIFIED,
            "pixel5": SELECTOR_OTHER_VERIFIED,
            "pixel6pro": SELECTOR_OTHER_VERIFIED,
            "pixel4xl": SELECTOR_OTHER_VERIFIED,
            "pixel4": SELECTOR_OTHER_VERIFIED,
        },
        "unknown_device_lane": SELECTOR_UNKNOWN,
        "truth_used_at_runtime": False,
        "integrity_rejection": "malformed WLS is rejected by the WLS manifest contract before lane selection",
        "promotion_scope": "development-only until a separately frozen release profile is approved",
    }


def _existing_score_maps(report: dict[str, Any]) -> dict[str, dict[str, Any]]:
    scores = report.get("scores")
    if not isinstance(scores, dict):
        raise DeviceFamilyEvaluationError("existing WLS report has no scores")
    required = ("wls_raw", "native_galileo_e1_hatch_raw", "native_segment_stability_recommended")
    if any(not isinstance(scores.get(key), dict) for key in required):
        raise DeviceFamilyEvaluationError("existing WLS report lacks required lane scores")
    return {key: dict(scores[key]) for key in required}


def _route_lane_metric(
    maps: dict[str, dict[str, Any]], dataset_id: str, lane: str
) -> dict[str, Any]:
    if lane == "wls_raw":
        return maps["wls_raw"][dataset_id]
    if lane == "native_raw":
        return maps["native_galileo_e1_hatch_raw"][dataset_id]
    if lane == "native_segment_stability":
        return maps["native_segment_stability_recommended"][dataset_id]
    raise DeviceFamilyEvaluationError(f"unknown existing lane: {lane}")


def _combined_lane_scores(
    existing_report: dict[str, Any],
    candidate_scores: dict[str, dict[str, Any]],
    selector_profile: dict[str, Any],
) -> dict[str, dict[str, Any]]:
    maps = _existing_score_maps(existing_report)
    all_ids = (*EXISTING_IDS, CANDIDATE_ID)
    result: dict[str, dict[str, Any]] = {}
    for lane in ("wls_raw", "native_segment_stability"):
        rows: list[dict[str, Any]] = []
        for dataset_id in all_ids:
            if dataset_id == CANDIDATE_ID:
                rows.append(candidate_scores[lane])
            else:
                rows.append(_route_lane_metric(maps, dataset_id, lane))
        result[lane] = _aggregate(rows)
    selector_rows: list[dict[str, Any]] = []
    for dataset_id in all_ids:
        device_model = dataset_id.rsplit("/", 1)[1]
        lane = _selector_lane(device_model, selector_profile)
        if dataset_id == CANDIDATE_ID:
            selector_rows.append(candidate_scores[lane])
        else:
            selector_rows.append(_route_lane_metric(maps, dataset_id, lane))
    result["selector"] = _aggregate(selector_rows)
    result["existing_six_wls_raw"] = _aggregate([maps["wls_raw"][dataset_id] for dataset_id in EXISTING_IDS])
    result["existing_six_selector"] = _aggregate(
        [
            _route_lane_metric(
                maps,
                dataset_id,
                _selector_lane(dataset_id.rsplit("/", 1)[1], selector_profile),
            )
            for dataset_id in EXISTING_IDS
        ]
    )
    return result


def _selector_gate(
    combined: dict[str, dict[str, Any]],
    new_scores: dict[str, dict[str, Any]],
) -> dict[str, Any]:
    reproduction = _non_regression(
        new_scores["native_segment_stability"], new_scores["wls_raw"]
    )
    reproduction["strict_h_p95_improvement"] = _metric(
        new_scores["native_segment_stability"], ("horizontal_wgs84_m", "p95_m")
    ) < _metric(new_scores["wls_raw"], ("horizontal_wgs84_m", "p95_m")) - 1e-12
    combined_gate = _aggregate_non_regression(combined["selector"], combined["wls_raw"])
    existing_gate = _aggregate_non_regression(
        combined["existing_six_selector"], combined["existing_six_wls_raw"]
    )
    strict_combined = (
        _metric(combined["selector"], ("mean_horizontal_wgs84_p95_m",))
        < _metric(combined["wls_raw"], ("mean_horizontal_wgs84_p95_m",)) - 1e-12
        or any(
            _metric(
                combined["selector"], ("mean_kaggle_diagnostic_score_variants_m", key)
            )
            < _metric(
                combined["wls_raw"], ("mean_kaggle_diagnostic_score_variants_m", key)
            )
            - 1e-12
            for key in DIAGNOSTIC_KEYS
        )
    )
    promoted = bool(
        reproduction["non_regression_passed"]
        and reproduction["strict_h_p95_improvement"]
        and combined_gate["non_regression_passed"]
        and existing_gate["non_regression_passed"]
        and strict_combined
    )
    return {
        "new_pixel7pro_reproduction": reproduction,
        "combined_seven_route_gate": combined_gate,
        "existing_six_route_gate": existing_gate,
        "combined_strict_improvement": strict_combined,
        "promotion_decision": "promote-development-only-selector" if promoted else "no-go-selector",
    }


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        prog=os.environ.get("GNSS_CLI_NAME", "gnss smartphone-wls-device-family-eval")
    )
    parser.add_argument("--archive", type=Path, default=DEFAULT_ARCHIVE)
    parser.add_argument("--profile", type=Path, default=DEFAULT_PROFILE)
    parser.add_argument("--inventory", type=Path, default=DEFAULT_INVENTORY)
    parser.add_argument("--selection-record", type=Path, default=DEFAULT_SELECTION_RECORD)
    parser.add_argument("--existing-report", type=Path, default=DEFAULT_EXISTING_REPORT)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    parser.add_argument("--max-epochs", type=int, default=-1)
    return parser.parse_args(argv)


def run(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    try:
        if args.max_epochs == 0 or args.max_epochs < -1:
            raise DeviceFamilyEvaluationError("max-epochs must be -1 or a positive integer")
        selection_record = _load_selection_record(args.selection_record)
        selection_record_sha256 = _sha256(args.selection_record)
        profile = previous._load_profile(args.profile)
        if str(dict(profile["datasets"]["holdout"])["id"]) != HOLDOUT_ID:
            raise DeviceFamilyEvaluationError("profile holdout differs from sealed contract")
        frozen_inventory = _load_frozen_inventory(args.inventory, selection_record)
        _verify_central_candidate(frozen_inventory, selection_record)
        frozen_archive_sha = str(
            dict(selection_record["archive"]).get("sha256_from_frozen_profile", "")
        )
        if frozen_archive_sha and frozen_archive_sha != str(dict(profile["archive"]).get("sha256", "")):
            raise DeviceFamilyEvaluationError("selection record archive hash differs from frozen profile")
        # Read the central directory for a fresh metadata-only check.  The
        # selected member payload is not opened until after this validation.
        inventory = generalization.inventory_archive(args.archive)
        candidate = _verify_central_candidate(inventory, selection_record)
        archive_expected = str(dict(profile["archive"]).get("sha256", ""))
        if not archive_expected:
            raise DeviceFamilyEvaluationError("profile archive SHA-256 is empty")
        archive_sha = generalization._sha256(args.archive)
        if archive_sha != archive_expected:
            raise DeviceFamilyEvaluationError("archive SHA-256 does not match frozen profile")
        if args.output_dir.exists() and any(args.output_dir.iterdir()):
            raise DeviceFamilyEvaluationError(f"refusing to overwrite non-empty output: {args.output_dir}")
        args.output_dir.mkdir(parents=True, exist_ok=True)
        inventory["selection"] = {
            "selected_candidate_id": CANDIDATE_ID,
            "development_main_excluded": MAIN_ID,
            "holdout_excluded": HOLDOUT_ID,
            "previously_used_ids_excluded": list(EXISTING_IDS) + [HOLDOUT_ID],
            "selection_content_read": False,
            "selection_truth_opened": False,
        }
        inventory["archive"].update(
            {
                "sha256": archive_sha,
                "archive_hash_verified_before_member_extraction": True,
                "member_content_read": False,
                "member_sha256_computed": False,
            }
        )
        inventory_path = args.output_dir / "archive_inventory.json"
        _atomic_json(inventory_path, inventory)

        started = time.perf_counter()
        route = _run_truth_free_route(
            args.archive,
            profile,
            candidate,
            archive_sha,
            args.output_dir,
            max_epochs=args.max_epochs,
        )
        # This is the first and only truth parse for the newly selected route;
        # it occurs after adapter, native SPP, WLS, and stability artifacts.
        candidate_scores = _score_candidate(route)
        existing_report = json.loads(args.existing_report.read_text(encoding="utf-8"))
        selector_profile = _selector_profile()
        combined = _combined_lane_scores(existing_report, candidate_scores, selector_profile)
        gate = _selector_gate(combined, candidate_scores)
        selector_path: Path | None = None
        if gate["promotion_decision"] == "promote-development-only-selector":
            selector_path = args.output_dir / "device_lane_selector_profile.json"
            _atomic_json(selector_path, selector_profile)
        report_path = args.output_dir / "wls_device_family_report.json"
        report = {
            "schema_version": SCHEMA_VERSION,
            "decision": "development-only-wls-device-family-evaluation",
            "selection_record": str(args.selection_record),
            "selection_record_sha256": selection_record_sha256,
            "archive": {
                "path": str(args.archive),
                "sha256": archive_sha,
                "profile_sha256": _sha256(args.profile),
                "inventory": {"path": str(inventory_path), "sha256": _sha256(inventory_path)},
            },
            "roles": {
                "new_route": CANDIDATE_ID,
                "existing_six_routes": list(EXISTING_IDS),
                "development_main": MAIN_ID,
                "holdout": HOLDOUT_ID,
            },
            "truth_free_contract": {
                "central_directory_selection": True,
                "truth_free_generation_completed_before_new_route_truth_parse": True,
                "new_route_truth_parsed_once_after_generation": True,
                "truth_dependent_runtime_selection": False,
                "holdout_content_opened": False,
                "holdout_truth_opened": False,
                "holdout_materialized": False,
            },
            "candidate_metadata": candidate,
            "route": {
                "inputs": route["inputs"],
                "profile": route["profile"],
                "materialization": route["materialization"],
                "truth_free_generation_complete": route["truth_free_generation_complete"],
                "wls_manifest": route["wls_manifest"],
                "wls_summary": route["wls_summary"],
                "native_segment_stability": route["native_artifacts"],
                "wls_segment_stability": route["wls_artifacts"],
                "segment_reports": {
                    "native": route["native_stability"],
                    "wls": route["wls_stability"],
                },
                "timing": route["timing"],
            },
            "new_route_scores": candidate_scores,
            "existing_report": {
                "path": str(args.existing_report),
                "sha256": _sha256(args.existing_report),
                "routes_reused_without_reprocessing": list(EXISTING_IDS),
            },
            "selector_candidate": {
                "profile": selector_profile,
                "profile_artifact": _artifact(selector_path) if selector_path is not None else None,
                "lane_rule": {
                    "pixel7pro": SELECTOR_PIXEL7PRO,
                    "other_verified_devices": SELECTOR_OTHER_VERIFIED,
                    "unknown_device": SELECTOR_UNKNOWN,
                    "runtime_inputs": ["device_model", "WLS_integrity_manifest"],
                    "truth_used": False,
                },
                "combined_metrics_existing_six_plus_new": combined,
                "gate": gate,
            },
            "promotion_decision": gate["promotion_decision"],
            "timing": {
                "total_wall_s": time.perf_counter() - started,
                "new_route_truth_free_generation_before_scoring": True,
            },
        }
        _atomic_json(report_path, report)
        manifest_path = args.output_dir / "wls_device_family_manifest.json"
        manifest = {
            "schema_version": "smartphone-r5-wls-device-family-manifest.v1",
            "report": _artifact(report_path),
            "selection_record": {"path": str(args.selection_record), "sha256": selection_record_sha256},
            "inventory": _artifact(inventory_path),
            "new_route": {
                "dataset_id": CANDIDATE_ID,
                "truth_free": True,
                "wls_manifest": route["wls_manifest"],
                "native_segment_stability": route["native_artifacts"],
                "wls_segment_stability": route["wls_artifacts"],
            },
            "selector_profile": _artifact(selector_path) if selector_path is not None else None,
            "truth_free_generation": True,
            "new_route_truth_parsed_once_after_generation": True,
            "holdout_content_opened": False,
            "holdout_truth_opened": False,
            "holdout_materialized": False,
        }
        _atomic_json(manifest_path, manifest)
        # The selection record is intentionally updated only after this
        # report/manifest exists; its pre-evaluation status protected the
        # route choice and remains auditable in the post-evaluation block.
        print(f"Smartphone WLS device-family evaluation complete: {report_path}")
        return 0
    except (
        OSError,
        ValueError,
        KeyError,
        TypeError,
        json.JSONDecodeError,
        generalization.GeneralizationError,
        conservative.ConservativeEvaluationError,
        previous.ReacquisitionError,
        smoother.SmootherError,
        segment_stability.SegmentStabilityError,
        wls.WlsPositionError,
        DeviceFamilyEvaluationError,
    ) as exc:
        print(f"Smartphone WLS device-family evaluation failed: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(run())
