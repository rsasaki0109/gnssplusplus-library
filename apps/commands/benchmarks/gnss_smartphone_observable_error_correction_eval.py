#!/usr/bin/env python3
"""Frozen train/validation evaluator for observable handset-WLS correction.

The evaluator has an intentionally visible phase boundary:

1. all fixed train features are extracted and sealed without opening truth;
2. train truth is opened only to fit leave-one-route-out models and score them;
3. fresh validation payload/truth is materialized once only after the train gate;
4. the future holdout is touched only after a passed validation gate.

The command is development-only.  It never changes the production smartphone
pipeline or uses Kaggle scores for fitting/ranking.
"""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
import hashlib
import json
import math
import os
from pathlib import Path
import resource
import sys
import time
import zipfile
from typing import Any, Iterable

import numpy as np

from support.gnss_runtime import application_root


ROOT = application_root(__file__)
BENCHMARK_DIR = ROOT / "apps" / "commands" / "benchmarks"
if str(BENCHMARK_DIR) not in sys.path:
    sys.path.insert(0, str(BENCHMARK_DIR))
import gnss_smartphone_kaggle as kaggle  # noqa: E402
import gnss_smartphone_observable_error_correction as correction  # noqa: E402
import gnss_smartphone_trajectory_smoother as smoother  # noqa: E402
import gnss_smartphone_trajectory_smoother_eval as smoother_eval  # noqa: E402
import gnss_smartphone_wls as wls  # noqa: E402


SCHEMA_VERSION = "smartphone-r5-observable-error-correction-evaluation.v1"
SELECTION_SCHEMA_VERSION = "smartphone-r5-observable-error-correction-selection.v1"
DEFAULT_SELECTION = (
    ROOT
    / "docs"
    / "use_cases"
    / "records"
    / "smartphone_r5_observable_error_correction_selection.json"
)
DEFAULT_INVENTORY = ROOT / "output" / "smartphone-r5" / "generalization-v6" / "archive_inventory.json"
DEFAULT_ARCHIVE = ROOT / "data" / "gsdc2023" / "cache" / "dataset_2023.zip"
DEFAULT_OUTPUT = ROOT / "output" / "smartphone-r5" / "observable-error-correction-v1"
MATCH_TOLERANCE_MS = 100
GATE_TOLERANCE_M = 1.0e-6
H_P50_VALIDATION_TOLERANCE_M = 0.01

TRAIN_IDS = (
    "2021-01-04-21-50-us-ca-e1highway280driveroutea/mi8",
    "2021-03-16-20-40-us-ca-mtv-b/pixel4xl",
    "2021-07-14-20-50-us-ca-mtv-e/pixel4",
    "2022-08-04-20-07-us-ca-sjc-q/pixel5",
    "2023-03-08-21-34-us-ca-mtv-u/pixel6pro",
    "2022-11-15-00-53-us-ca-mtv-a/pixel7pro",
    "2023-05-24-20-26-us-ca-sjc-ge2/pixel7pro",
)
FRESH_VALIDATION_ID = "2023-09-07-18-59-us-ca/pixel5"
FUTURE_HOLDOUT_ID = "2023-09-06-00-01-us-ca-routen/pixel6pro"
DIAGNOSTIC_KEYS = tuple(
    f"{distance}__{percentile}"
    for distance in ("wgs84_vincenty", "haversine_sphere")
    for percentile in ("linear_n_minus_1", "nearest_rank_ceiling")
)


class ObservableCorrectionEvaluationError(ValueError):
    """Raised when the frozen evaluation contract cannot be proven."""


@dataclass(frozen=True)
class RouteSpec:
    dataset_id: str
    device_gnss: Path | None
    ground_truth: Path | None
    role: str


def _sha256(path: Path) -> str:
    if not path.is_file():
        raise ObservableCorrectionEvaluationError(f"missing file: {path}")
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _atomic_write(path: Path, content: bytes) -> None:
    correction._atomic_write(path, content)


def _atomic_json(path: Path, payload: dict[str, Any]) -> None:
    correction._atomic_json(path, payload)


def _safe_id(dataset_id: str) -> str:
    return dataset_id.replace("/", "__")


def _load_selection(path: Path) -> dict[str, Any]:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ObservableCorrectionEvaluationError(f"invalid selection record: {path}") from exc
    if not isinstance(payload, dict) or payload.get("schema_version") != SELECTION_SCHEMA_VERSION:
        raise ObservableCorrectionEvaluationError("observable correction selection schema differs")
    if payload.get("status") != "selection-frozen-before-candidate-implementation":
        raise ObservableCorrectionEvaluationError("selection was not frozen before implementation")
    split = payload.get("fixed_split")
    if not isinstance(split, dict):
        raise ObservableCorrectionEvaluationError("selection has no fixed split")
    if tuple(split.get("candidate_train", ())) != TRAIN_IDS:
        raise ObservableCorrectionEvaluationError("train route split differs from selection")
    if split.get("fresh_validation") != FRESH_VALIDATION_ID:
        raise ObservableCorrectionEvaluationError("fresh validation differs from selection")
    if split.get("future_holdout") != FUTURE_HOLDOUT_ID:
        raise ObservableCorrectionEvaluationError("future holdout differs from selection")
    candidate = payload.get("candidate")
    if not isinstance(candidate, dict) or candidate.get("id") != correction.MODEL_ID:
        raise ObservableCorrectionEvaluationError("candidate id differs from selection")
    if float(candidate.get("alpha", 0.0)) != correction.MODEL_ALPHA:
        raise ObservableCorrectionEvaluationError("candidate alpha differs from selection")
    if candidate.get("features_in_order") != list(correction.FEATURE_NAMES):
        raise ObservableCorrectionEvaluationError("candidate feature order differs from selection")
    if candidate.get("skip_epochs") != correction.DEFAULT_SKIP_EPOCHS:
        raise ObservableCorrectionEvaluationError("candidate skip_epochs differs from selection")
    leakage = payload.get("data_leakage_policy")
    if not isinstance(leakage, dict) or leakage.get("public_private_kaggle_scores_used_for_tuning") is not False:
        raise ObservableCorrectionEvaluationError("selection does not prohibit leaderboard tuning")
    return payload


def _route_specs() -> dict[str, RouteSpec]:
    g6 = ROOT / "output" / "smartphone-r5" / "generalization-v6" / "routes"
    conservative = ROOT / "output" / "smartphone-r5" / "reacquisition-conservative-v3" / "new-validation-materialized" / "routes"
    segment = ROOT / "output" / "smartphone-r5" / "segment-stability-v1" / "new-validation-materialized" / "routes"
    specs = {
        TRAIN_IDS[0]: RouteSpec(
            TRAIN_IDS[0],
            g6 / "2021-01-04-21-50-us-ca-e1highway280driveroutea" / "mi8" / "inputs" / "device_gnss.csv",
            g6 / "2021-01-04-21-50-us-ca-e1highway280driveroutea" / "mi8" / "inputs" / "ground_truth.csv",
            "candidate-train",
        ),
        TRAIN_IDS[1]: RouteSpec(
            TRAIN_IDS[1],
            conservative / "2021-03-16-20-40-us-ca-mtv-b" / "pixel4xl" / "inputs" / "device_gnss.csv",
            conservative / "2021-03-16-20-40-us-ca-mtv-b" / "pixel4xl" / "inputs" / "ground_truth.csv",
            "candidate-train",
        ),
        TRAIN_IDS[2]: RouteSpec(
            TRAIN_IDS[2],
            segment / "2021-07-14-20-50-us-ca-mtv-e" / "pixel4" / "inputs" / "device_gnss.csv",
            segment / "2021-07-14-20-50-us-ca-mtv-e" / "pixel4" / "inputs" / "ground_truth.csv",
            "candidate-train",
        ),
        TRAIN_IDS[3]: RouteSpec(
            TRAIN_IDS[3],
            g6 / "2022-08-04-20-07-us-ca-sjc-q" / "pixel5" / "inputs" / "device_gnss.csv",
            g6 / "2022-08-04-20-07-us-ca-sjc-q" / "pixel5" / "inputs" / "ground_truth.csv",
            "candidate-train",
        ),
        TRAIN_IDS[4]: RouteSpec(
            TRAIN_IDS[4],
            g6 / "2023-03-08-21-34-us-ca-mtv-u" / "pixel6pro" / "inputs" / "device_gnss.csv",
            g6 / "2023-03-08-21-34-us-ca-mtv-u" / "pixel6pro" / "inputs" / "ground_truth.csv",
            "candidate-train",
        ),
        TRAIN_IDS[5]: RouteSpec(
            TRAIN_IDS[5],
            ROOT / "output" / "smartphone-r5" / "wls-device-family-v1" / "materialized" / "routes" / "2022-11-15-00-53-us-ca-mtv-a" / "pixel7pro" / "inputs" / "device_gnss.csv",
            ROOT / "output" / "smartphone-r5" / "wls-device-family-v1" / "materialized" / "routes" / "2022-11-15-00-53-us-ca-mtv-a" / "pixel7pro" / "inputs" / "ground_truth.csv",
            "candidate-train",
        ),
        TRAIN_IDS[6]: RouteSpec(
            TRAIN_IDS[6],
            ROOT / "data" / "gsdc2023" / "materialized" / "dataset_2023" / "train" / "2023-05-24-20-26-us-ca-sjc-ge2" / "pixel7pro" / "device_gnss.csv",
            ROOT / "data" / "gsdc2023" / "materialized" / "dataset_2023" / "train" / "2023-05-24-20-26-us-ca-sjc-ge2" / "pixel7pro" / "ground_truth.csv",
            "candidate-train",
        ),
    }
    return specs


def _truth_to_ecef(truth: dict[int, tuple[float, float, float]], timestamp: int) -> np.ndarray | None:
    reference = smoother_eval._match_truth(timestamp, truth, MATCH_TOLERANCE_MS)
    if reference is None:
        return None
    latitude, longitude, height = reference
    return smoother._wgs84_geodetic_to_ecef(
        math.radians(latitude), math.radians(longitude), height
    )


def _training_arrays(
    extraction: correction.FeatureExtraction,
    truth: dict[int, tuple[float, float, float]],
) -> tuple[np.ndarray, np.ndarray, int]:
    features: list[np.ndarray] = []
    residuals: list[np.ndarray] = []
    for row in extraction.rows:
        target = _truth_to_ecef(truth, row.timestamp_ms)
        if target is None:
            continue
        residual = target - row.ecef
        if not np.isfinite(residual).all():
            raise ObservableCorrectionEvaluationError("truth residual is non-finite")
        features.append(row.features)
        residuals.append(residual)
    if not features:
        raise ObservableCorrectionEvaluationError("no feature/truth timestamp matches")
    return np.asarray(features, dtype=float), np.asarray(residuals, dtype=float), len(features)


def _smoothed_rows(
    positions: list[smoother.PositionRow], *, source: str
) -> list[smoother.SmoothedRow]:
    rows: list[smoother.SmoothedRow] = []
    for position in positions:
        rows.append(
            smoother.SmoothedRow(
                timestamp_ms=position.timestamp_ms,
                week=position.week,
                tow=position.tow,
                ecef=position.ecef,
                latitude=position.latitude,
                longitude=position.longitude,
                height=position.height,
                status=position.status,
                satellites=position.satellites,
                pdop=position.pdop,
                ratio=position.ratio,
                fixed_ambiguities=position.fixed_ambiguities,
                iterations=position.iterations,
                source=source,
                segment_id=0,
                measurement_used=True,
                outlier_rejected=False,
                innovation_sigma=None,
                position_sigma_m=smoother._measurement_sigma(position, 1.0),
            )
        )
    return rows


def _positions_from_ecef(
    extraction: correction.FeatureExtraction, ecef_values: Iterable[np.ndarray]
) -> list[smoother.PositionRow]:
    values = list(ecef_values)
    if len(values) != len(extraction.rows):
        raise ObservableCorrectionEvaluationError("candidate ECEF row count differs")
    epochs = tuple(
        wls.WlsEpoch(
            timestamp_ms=row.timestamp_ms,
            clock_discontinuity_count=0,
            ecef=tuple(float(value) for value in values[index]),
            rows=row.raw_row_count,
            finite_rows=row.raw_row_count,
            svid_count=row.satellite_count,
            first_source_row=row.source_line,
            last_source_row=row.source_line + row.raw_row_count - 1,
        )
        for index, row in enumerate(extraction.rows)
    )
    return wls.epochs_to_positions(epochs, correction.DEFAULT_LEAP_SECONDS)


def _score(
    positions: list[smoother.PositionRow], truth: dict[int, tuple[float, float, float]]
) -> dict[str, Any]:
    rows = _smoothed_rows(positions, source="measured")
    position_map = {position.timestamp_ms: position for position in positions}
    return smoother_eval._score_rows(
        rows,
        position_map,
        truth,
        0,
        len(rows),
        match_tolerance_ms=MATCH_TOLERANCE_MS,
    )


def _metric(metrics: dict[str, Any], path: tuple[str, ...]) -> float:
    value: Any = metrics
    for key in path:
        value = value[key]
    if value is None or not math.isfinite(float(value)):
        return math.inf
    return float(value)


def _aggregate(metrics: list[dict[str, Any]]) -> dict[str, Any]:
    if not metrics:
        raise ObservableCorrectionEvaluationError("cannot aggregate empty metrics")
    result: dict[str, Any] = {
        "route_count": len(metrics),
        "mean_availability_ratio": sum(_metric(m, ("availability_ratio",)) for m in metrics) / len(metrics),
        "mean_truth_coverage_ratio": sum(_metric(m, ("truth_coverage_ratio",)) for m in metrics) / len(metrics),
        "mean_horizontal_wgs84_p50_m": sum(_metric(m, ("horizontal_wgs84_m", "p50_m")) for m in metrics) / len(metrics),
        "mean_horizontal_wgs84_p95_m": sum(_metric(m, ("horizontal_wgs84_m", "p95_m")) for m in metrics) / len(metrics),
        "mean_vertical_p95_abs_m": sum(_metric(m, ("vertical_p95_abs_m",)) for m in metrics) / len(metrics),
    }
    result["mean_kaggle_diagnostic_score_variants_m"] = {
        key: sum(_metric(m, ("kaggle_diagnostic_score_variants_m", key)) for m in metrics) / len(metrics)
        for key in DIAGNOSTIC_KEYS
    }
    result["mean_kaggle_diagnostic_m"] = sum(result["mean_kaggle_diagnostic_score_variants_m"].values()) / len(DIAGNOSTIC_KEYS)
    return result


def _non_regression(candidate: dict[str, Any], baseline: dict[str, Any], *, validation: bool = False) -> list[str]:
    failures: list[str] = []
    tolerance = GATE_TOLERANCE_M
    if _metric(candidate, ("availability_ratio",)) < _metric(baseline, ("availability_ratio",)) - tolerance:
        failures.append("availability_regression")
    if _metric(candidate, ("truth_coverage_ratio",)) < _metric(baseline, ("truth_coverage_ratio",)) - tolerance:
        failures.append("truth_coverage_regression")
    h_p50_tolerance = H_P50_VALIDATION_TOLERANCE_M if validation else tolerance
    if _metric(candidate, ("horizontal_wgs84_m", "p50_m")) > _metric(baseline, ("horizontal_wgs84_m", "p50_m")) + h_p50_tolerance:
        failures.append("h_p50_regression")
    for path, label in (
        (("horizontal_wgs84_m", "p95_m"), "h_p95_regression"),
        (("vertical_p95_abs_m",), "v_p95_regression"),
    ):
        if _metric(candidate, path) > _metric(baseline, path) + tolerance:
            failures.append(label)
    for key in DIAGNOSTIC_KEYS:
        if _metric(candidate, ("kaggle_diagnostic_score_variants_m", key)) > _metric(
            baseline, ("kaggle_diagnostic_score_variants_m", key)
        ) + tolerance:
            failures.append(f"{key}_regression")
    return failures


def _gate(
    route_results: dict[str, dict[str, Any]], *, validation: bool = False
) -> dict[str, Any]:
    failures_by_route = {
        dataset_id: _non_regression(value["candidate"], value["baseline"], validation=validation)
        for dataset_id, value in route_results.items()
    }
    baseline_aggregate = _aggregate([value["baseline"] for value in route_results.values()])
    candidate_aggregate = _aggregate([value["candidate"] for value in route_results.values()])
    strict_failures: list[str] = []
    if candidate_aggregate["mean_horizontal_wgs84_p95_m"] >= baseline_aggregate["mean_horizontal_wgs84_p95_m"] - GATE_TOLERANCE_M:
        strict_failures.append("aggregate_h_p95_not_strictly_improved")
    if candidate_aggregate["mean_kaggle_diagnostic_m"] >= baseline_aggregate["mean_kaggle_diagnostic_m"] - GATE_TOLERANCE_M:
        strict_failures.append("aggregate_diagnostic_mean_not_strictly_improved")
    if validation and candidate_aggregate["mean_vertical_p95_abs_m"] > baseline_aggregate["mean_vertical_p95_abs_m"] + GATE_TOLERANCE_M:
        strict_failures.append("validation_aggregate_v_p95_regression")
    return {
        "passed": not any(failures_by_route.values()) and not strict_failures,
        "failures_by_route": failures_by_route,
        "strict_failures": strict_failures,
        "baseline_aggregate": baseline_aggregate,
        "candidate_aggregate": candidate_aggregate,
    }


def _load_truth(path: Path) -> dict[int, tuple[float, float, float]]:
    return smoother_eval._read_truth(path)


def _write_model(path: Path, model: dict[str, Any]) -> dict[str, Any]:
    _atomic_json(path, model)
    return {"path": str(path), "sha256": _sha256(path), "bytes": path.stat().st_size}


def _write_truth_free_manifest(
    output_dir: Path,
    feature_artifacts: dict[str, dict[str, Any]],
    selection_path: Path,
    *,
    phase_wall_s: float,
) -> dict[str, Any]:
    manifest = {
        "schema_version": SCHEMA_VERSION + "-truth-free-manifest",
        "truth_free": True,
        "truth_used": False,
        "selection_record": {"path": str(selection_path), "sha256": _sha256(selection_path)},
        "candidate_source": {"path": str(Path(__file__).with_name("gnss_smartphone_observable_error_correction.py")), "sha256": _sha256(Path(__file__).with_name("gnss_smartphone_observable_error_correction.py"))},
        "feature_artifacts": feature_artifacts,
        "feature_artifact_count": len(feature_artifacts),
        "phase_wall_s": phase_wall_s,
        "future_holdout_payload_opened": False,
        "future_holdout_truth_opened": False,
    }
    path = output_dir / "truth_free_manifest.json"
    _atomic_json(path, manifest)
    manifest["path"] = str(path)
    manifest["sha256"] = _sha256(path)
    return manifest


def _inventory_record(inventory: dict[str, Any], dataset_id: str) -> dict[str, Any]:
    for record in inventory.get("train", {}).get("records", []):
        if isinstance(record, dict) and record.get("dataset_id") == dataset_id:
            return record
    raise ObservableCorrectionEvaluationError(f"dataset missing from central inventory: {dataset_id}")


def _load_inventory(path: Path) -> dict[str, Any]:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ObservableCorrectionEvaluationError(f"invalid inventory: {path}") from exc
    if not isinstance(payload, dict) or payload.get("schema_version") != "smartphone-r5-gsdc2023-central-inventory.v1":
        raise ObservableCorrectionEvaluationError("central inventory schema differs")
    archive = payload.get("archive")
    if not isinstance(archive, dict) or archive.get("central_directory_only") is not True:
        raise ObservableCorrectionEvaluationError("central inventory is not metadata-only")
    return payload


def _archive_sha256(archive: Path) -> str:
    return _sha256(archive)


def _safe_extract_member(
    archive: Path,
    member_name: str,
    target: Path,
    *,
    expected_size: int,
    expected_crc32_hex: str,
) -> dict[str, Any]:
    if not member_name.startswith("dataset_2023/") or ".." in Path(member_name).parts:
        raise ObservableCorrectionEvaluationError("archive member path is unsafe")
    try:
        with zipfile.ZipFile(archive) as handle:
            info = handle.getinfo(member_name)
            if info.file_size != expected_size or f"{info.CRC:08x}" != expected_crc32_hex.lower():
                raise ObservableCorrectionEvaluationError(f"archive central metadata differs for {member_name}")
            with handle.open(info, "r") as source:
                content = source.read()
    except (OSError, KeyError, zipfile.BadZipFile, RuntimeError) as exc:
        raise ObservableCorrectionEvaluationError(f"failed to materialize archive member {member_name}") from exc
    if len(content) != expected_size:
        raise ObservableCorrectionEvaluationError(f"materialized member size differs for {member_name}")
    _atomic_write(target, content)
    return {"path": str(target), "sha256": _sha256(target), "bytes": target.stat().st_size, "member": member_name}


def _materialize_route_member(
    archive: Path,
    inventory_record: dict[str, Any],
    dataset_id: str,
    target_root: Path,
    basename: str,
) -> dict[str, Any]:
    metadata = inventory_record.get("central_directory_files", {}).get(basename)
    if not isinstance(metadata, dict):
        raise ObservableCorrectionEvaluationError(f"inventory lacks {basename} for {dataset_id}")
    target = target_root / basename
    return _safe_extract_member(
        archive,
        str(metadata["name"]),
        target,
        expected_size=int(metadata["file_size"]),
        expected_crc32_hex=str(metadata["crc32_hex"]),
    )


def _truth_free_route(
    dataset_id: str,
    device_path: Path,
    output_root: Path,
) -> tuple[correction.FeatureExtraction, dict[str, Any]]:
    extraction = correction.extract_features(device_path, skip_epochs=correction.DEFAULT_SKIP_EPOCHS)
    route_dir = output_root / "routes" / _safe_id(dataset_id)
    feature_path = route_dir / "observable_features.csv"
    feature_manifest = correction.write_feature_artifact(
        extraction,
        feature_path,
        dataset_id=dataset_id,
        device_gnss=device_path,
    )
    feature_manifest["output_root"] = str(route_dir)
    _atomic_json(route_dir / "observable_features_manifest.json", feature_manifest)
    return extraction, {
        "features": {"path": str(feature_path), "sha256": _sha256(feature_path), "bytes": feature_path.stat().st_size},
        "manifest": {"path": str(route_dir / "observable_features_manifest.json"), "sha256": _sha256(route_dir / "observable_features_manifest.json")},
        "input": {"path": str(device_path), "sha256": extraction.device_sha256},
        "selected_epochs": extraction.selected_epochs,
    }


def _evaluate_model_on_route(
    dataset_id: str,
    extraction: correction.FeatureExtraction,
    truth: dict[int, tuple[float, float, float]],
    model: dict[str, Any],
    output_root: Path,
    device_gnss: Path,
) -> dict[str, Any]:
    base_positions = _positions_from_ecef(extraction, [row.ecef for row in extraction.rows])
    corrected, fallback_counts = correction.apply_model(extraction.rows, model)
    candidate_positions = _positions_from_ecef(extraction, corrected)
    baseline_metrics = _score(base_positions, truth)
    candidate_metrics = _score(candidate_positions, truth)
    route_dir = output_root / "routes" / _safe_id(dataset_id) / "candidate"
    candidate_manifest = correction.write_correction_outputs(
        extraction,
        corrected,
        route_dir,
        device_gnss=device_gnss,
        dataset_id=dataset_id,
        model=model,
        fallback_counts=fallback_counts,
    )
    return {
        "baseline": baseline_metrics,
        "candidate": candidate_metrics,
        "fallback_counts": fallback_counts,
        "candidate_manifest": candidate_manifest,
    }


def _fit_all_train_model(
    extractions: dict[str, correction.FeatureExtraction],
    truths: dict[str, dict[int, tuple[float, float, float]]],
) -> tuple[dict[str, Any], int]:
    features: list[np.ndarray] = []
    targets: list[np.ndarray] = []
    count = 0
    for dataset_id in TRAIN_IDS:
        x, y, matched = _training_arrays(extractions[dataset_id], truths[dataset_id])
        features.append(x)
        targets.append(y)
        count += matched
    model = correction.fit_ridge_model(np.vstack(features), np.vstack(targets), alpha=correction.MODEL_ALPHA)
    return model, count


def run(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    started = time.perf_counter()
    selection_path = args.frozen_selection
    try:
        selection = _load_selection(selection_path)
        inventory = _load_inventory(args.inventory)
        archive_meta = inventory["archive"]
        if archive_meta.get("path") != str(args.archive.relative_to(ROOT)):
            raise ObservableCorrectionEvaluationError("archive path differs from frozen inventory")
        specs = _route_specs()
        if any(dataset_id not in specs for dataset_id in TRAIN_IDS):
            raise ObservableCorrectionEvaluationError("route spec table is incomplete")
        for dataset_id in TRAIN_IDS:
            spec = specs[dataset_id]
            if spec.device_gnss is None or spec.ground_truth is None or not spec.device_gnss.is_file() or not spec.ground_truth.is_file():
                raise ObservableCorrectionEvaluationError(f"train input missing: {dataset_id}")
        # These checks intentionally inspect only filesystem presence.  They
        # prevent accidentally reusing a prior materialization as a fresh
        # validation or future holdout.
        for forbidden_id in (FRESH_VALIDATION_ID, FUTURE_HOLDOUT_ID):
            record = _inventory_record(inventory, forbidden_id)
            if not record.get("required_files_complete") or not record.get("broadcast_nav_present"):
                raise ObservableCorrectionEvaluationError(f"frozen split lacks complete metadata: {forbidden_id}")
        if args.output_dir.exists() and (args.output_dir / "evaluation.json").is_file() and not args.resume:
            raise ObservableCorrectionEvaluationError("evaluation output exists; use --resume only for the same frozen run")
        args.output_dir.mkdir(parents=True, exist_ok=True)
        feature_started = time.perf_counter()
        extractions: dict[str, correction.FeatureExtraction] = {}
        feature_artifacts: dict[str, dict[str, Any]] = {}
        for dataset_id in TRAIN_IDS:
            extraction, artifact = _truth_free_route(dataset_id, specs[dataset_id].device_gnss, args.output_dir)
            extractions[dataset_id] = extraction
            feature_artifacts[dataset_id] = artifact
        truth_free_manifest = _write_truth_free_manifest(
            args.output_dir,
            feature_artifacts,
            selection_path,
            phase_wall_s=time.perf_counter() - feature_started,
        )

        # Truth is opened only after every train feature file and the
        # truth-free manifest are sealed.
        truths: dict[str, dict[int, tuple[float, float, float]]] = {}
        train_truth_open_count = 0
        for dataset_id in TRAIN_IDS:
            truths[dataset_id] = _load_truth(specs[dataset_id].ground_truth)
            train_truth_open_count += 1

        loo_results: dict[str, dict[str, Any]] = {}
        loo_models: dict[str, dict[str, Any]] = {}
        loo_truth_matches: dict[str, int] = {}
        for held_out in TRAIN_IDS:
            train_features: list[np.ndarray] = []
            train_targets: list[np.ndarray] = []
            matched_count = 0
            for dataset_id in TRAIN_IDS:
                if dataset_id == held_out:
                    continue
                x, y, matched = _training_arrays(extractions[dataset_id], truths[dataset_id])
                train_features.append(x)
                train_targets.append(y)
                matched_count += matched
            model = correction.fit_ridge_model(
                np.vstack(train_features), np.vstack(train_targets), alpha=correction.MODEL_ALPHA
            )
            model_path = args.output_dir / "models" / f"loo_{_safe_id(held_out)}.json"
            model_artifact = _write_model(model_path, model)
            loo_models[held_out] = {"model": model, "artifact": model_artifact}
            loo_truth_matches[held_out] = matched_count
            loo_results[held_out] = _evaluate_model_on_route(
                held_out,
                extractions[held_out],
                truths[held_out],
                model,
                args.output_dir,
                specs[held_out].device_gnss,
            )
            loo_results[held_out]["train_rows_excluding_held_out"] = matched_count

        train_gate = _gate(loo_results)
        source_path = Path(__file__).with_name("gnss_smartphone_observable_error_correction.py")
        report: dict[str, Any] = {
            "schema_version": SCHEMA_VERSION,
            "decision": "pending-fresh-validation" if train_gate["passed"] else "no-go-train-loo",
            "selection": {"path": str(selection_path), "sha256": _sha256(selection_path)},
            "inventory": {"path": str(args.inventory), "sha256": _sha256(args.inventory)},
            "archive": {"path": str(args.archive), "sha256": _archive_sha256(args.archive)},
            "candidate": {
                "id": correction.MODEL_ID,
                "alpha": correction.MODEL_ALPHA,
                "source": {"path": str(source_path), "sha256": _sha256(source_path)},
                "feature_names": list(correction.FEATURE_NAMES),
            },
            "split": {
                "candidate_train": list(TRAIN_IDS),
                "fresh_validation": FRESH_VALIDATION_ID,
                "future_holdout": FUTURE_HOLDOUT_ID,
                "future_holdout_payload_opened": False,
                "future_holdout_truth_opened": False,
            },
            "truth_access": {
                "truth_free_train_feature_phase": True,
                "train_truth_open_count": train_truth_open_count,
                "fresh_validation_truth_open_count": 0,
                "future_holdout_truth_open_count": 0,
            },
            "truth_free_manifest": truth_free_manifest,
            "train_loo": {"routes": loo_results, "gate": train_gate},
            "timing": {"wall_s_through_train_gate": time.perf_counter() - started, "max_rss_kb": resource.getrusage(resource.RUSAGE_SELF).ru_maxrss},
            "no_post_holdout_tuning": True,
        }
        if not train_gate["passed"]:
            report["validation"] = {"not_run": "train LOO gate failed"}
            report["holdout"] = {"not_run": "train LOO gate failed"}
            _atomic_json(args.output_dir / "evaluation.json", report)
            return 0

        # The first gate passed.  Materialize only the fresh validation
        # device member, produce and seal its truth-free artifact, then open
        # validation truth exactly once.
        archive_hash = _archive_sha256(args.archive)
        if archive_hash != archive_meta.get("sha256"):
            raise ObservableCorrectionEvaluationError("archive SHA differs from central inventory")
        fresh_root = args.output_dir / "fresh-validation" / _safe_id(FRESH_VALIDATION_ID) / "inputs"
        fresh_record = _inventory_record(inventory, FRESH_VALIDATION_ID)
        fresh_device_artifact = _materialize_route_member(
            args.archive, fresh_record, FRESH_VALIDATION_ID, fresh_root, "device_gnss.csv"
        )
        fresh_extraction, fresh_feature_artifact = _truth_free_route(
            FRESH_VALIDATION_ID, fresh_root / "device_gnss.csv", args.output_dir / "fresh-validation"
        )
        fresh_truth_artifact = _materialize_route_member(
            args.archive, fresh_record, FRESH_VALIDATION_ID, fresh_root, "ground_truth.csv"
        )
        fresh_truth = _load_truth(fresh_root / "ground_truth.csv")
        report["truth_access"]["fresh_validation_truth_open_count"] = 1
        final_model, final_train_rows = _fit_all_train_model(extractions, truths)
        final_model_artifact = _write_model(args.output_dir / "models" / "all_train.json", final_model)
        fresh_result = _evaluate_model_on_route(
            FRESH_VALIDATION_ID,
            fresh_extraction,
            fresh_truth,
            final_model,
            args.output_dir / "fresh-validation",
            fresh_root / "device_gnss.csv",
        )
        fresh_gate = _gate({FRESH_VALIDATION_ID: fresh_result}, validation=True)
        report["decision"] = "pending-holdout" if fresh_gate["passed"] else "no-go-fresh-validation"
        report["validation"] = {
            "route": fresh_result,
            "gate": fresh_gate,
            "materialization": {"device_gnss": fresh_device_artifact, "ground_truth": fresh_truth_artifact},
            "features": fresh_feature_artifact,
            "final_model": final_model_artifact,
            "train_rows": final_train_rows,
        }
        if not fresh_gate["passed"]:
            report["holdout"] = {"not_run": "fresh validation gate failed"}
            report["timing"]["wall_s_final"] = time.perf_counter() - started
            report["timing"]["max_rss_kb"] = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss
            _atomic_json(args.output_dir / "evaluation.json", report)
            return 0

        # Holdout is now authorized by the frozen train+validation gate.  No
        # ranking or parameter adjustment is performed after this point.
        holdout_root = args.output_dir / "future-holdout" / _safe_id(FUTURE_HOLDOUT_ID) / "inputs"
        holdout_record = _inventory_record(inventory, FUTURE_HOLDOUT_ID)
        holdout_device_artifact = _materialize_route_member(
            args.archive, holdout_record, FUTURE_HOLDOUT_ID, holdout_root, "device_gnss.csv"
        )
        holdout_extraction, holdout_feature_artifact = _truth_free_route(
            FUTURE_HOLDOUT_ID, holdout_root / "device_gnss.csv", args.output_dir / "future-holdout"
        )
        holdout_truth_artifact = _materialize_route_member(
            args.archive, holdout_record, FUTURE_HOLDOUT_ID, holdout_root, "ground_truth.csv"
        )
        holdout_truth = _load_truth(holdout_root / "ground_truth.csv")
        report["truth_access"]["future_holdout_truth_open_count"] = 1
        holdout_result = _evaluate_model_on_route(
            FUTURE_HOLDOUT_ID,
            holdout_extraction,
            holdout_truth,
            final_model,
            args.output_dir / "future-holdout",
            holdout_root / "device_gnss.csv",
        )
        holdout_gate = _gate({FUTURE_HOLDOUT_ID: holdout_result}, validation=True)
        report["decision"] = "promote-development-only" if holdout_gate["passed"] else "no-go-holdout"
        report["holdout"] = {
            "route": holdout_result,
            "gate": holdout_gate,
            "materialization": {"device_gnss": holdout_device_artifact, "ground_truth": holdout_truth_artifact},
            "features": holdout_feature_artifact,
        }
        report["timing"]["wall_s_final"] = time.perf_counter() - started
        report["timing"]["max_rss_kb"] = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss
        _atomic_json(args.output_dir / "evaluation.json", report)
        return 0
    except (ObservableCorrectionEvaluationError, correction.ObservableCorrectionError, smoother.SmootherError, ValueError, OSError, zipfile.BadZipFile) as exc:
        failure = {
            "schema_version": SCHEMA_VERSION + "-failure",
            "decision": "blocked-before-truth-or-incomplete",
            "error_type": type(exc).__name__,
            "error": str(exc),
            "truth_access": {"train": 0, "fresh_validation": 0, "future_holdout": 0},
            "selection": {"path": str(selection_path), "exists": selection_path.is_file()},
            "timing": {"wall_s": time.perf_counter() - started},
        }
        args.output_dir.mkdir(parents=True, exist_ok=True)
        _atomic_json(args.output_dir / "evaluation_failure.json", failure)
        print(f"Observable correction evaluation failed: {exc}", file=sys.stderr)
        return 2


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        prog=os.environ.get("GNSS_CLI_NAME", "gnss smartphone-observable-error-correction-eval")
    )
    parser.add_argument("--frozen-selection", type=Path, default=DEFAULT_SELECTION)
    parser.add_argument("--inventory", type=Path, default=DEFAULT_INVENTORY)
    parser.add_argument("--archive", type=Path, default=DEFAULT_ARCHIVE)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--resume", action="store_true", help="resume only the same local frozen evaluation output")
    return parser.parse_args(argv)


if __name__ == "__main__":
    raise SystemExit(run())
