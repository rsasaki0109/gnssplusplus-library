#!/usr/bin/env python3
"""Evaluate the truth-free Android WLS position lane.

All WLS extraction, conversion, smoothing, and segment decisions are made
before any truth file is opened.  Truth is used only in the ordered
train-selection, fixed-validation/main gates, and post-hoc route audit.  The
sealed holdout is not a route input to this command.
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
import time
from typing import Any

from support.gnss_runtime import application_root


ROOT = application_root(__file__)
BENCHMARK_DIR = ROOT / "apps" / "commands" / "benchmarks"
if str(BENCHMARK_DIR) not in sys.path:
    sys.path.insert(0, str(BENCHMARK_DIR))
import gnss_smartphone_reacquisition_eval as previous  # noqa: E402
import gnss_smartphone_segment_stability as segment_stability  # noqa: E402
import gnss_smartphone_trajectory_smoother as smoother  # noqa: E402
import gnss_smartphone_trajectory_smoother_eval as smoother_eval  # noqa: E402
import gnss_smartphone_wls as wls  # noqa: E402


SCHEMA_VERSION = "smartphone-r5-wls-position-evaluation.v1"
DEFAULT_OUTPUT_DIR = ROOT / "output" / "smartphone-r5" / "wls-position-v1"
DEFAULT_SELECTION_RECORD = (
    ROOT
    / "docs"
    / "use_cases"
    / "records"
    / "smartphone_r5_wls_position_candidates.json"
)
HOLDOUT_ID = previous.HOLDOUT_ID
TRAIN_IDS = previous.TRAIN_IDS
VALIDATION_IDS = previous.VALIDATION_IDS
MAIN_ID = previous.MAIN_ID
AUDIT_IDS = (
    "2021-03-16-20-40-us-ca-mtv-b/pixel4xl",
    "2021-07-14-20-50-us-ca-mtv-e/pixel4",
)
ALL_IDS = (*TRAIN_IDS, *VALIDATION_IDS, *AUDIT_IDS, MAIN_ID)
CANDIDATES = (
    {
        "id": "wls_segment_r15_d15",
        "max_consecutive_rejects": 15,
        "max_prediction_duration_s": 15.0,
        "reject_fraction_max": None,
    },
    {
        "id": "wls_segment_r20_d20",
        "max_consecutive_rejects": 20,
        "max_prediction_duration_s": 20.0,
        "reject_fraction_max": None,
    },
    {
        "id": "wls_segment_r30_d30",
        "max_consecutive_rejects": 30,
        "max_prediction_duration_s": 30.0,
        "reject_fraction_max": None,
    },
)
BASELINE_CONFIG = {
    "process_noise": 1.0,
    "measurement_floor_m": 1.0,
    "outlier_gate_sigma": 5.0,
    "segment_gap_s": 10.0,
}
DIAGNOSTIC_KEYS = previous._DIAGNOSTIC_KEYS


class WlsEvaluationError(ValueError):
    """Raised when the fixed WLS evaluation contract is violated."""


@dataclass(frozen=True)
class RouteSpec:
    dataset_id: str
    role: str
    device_gnss: Path
    truth: Path
    native_position: Path
    native_existing_smoother: Path


def _sha256(path: Path) -> str:
    if not path.is_file():
        raise WlsEvaluationError(f"missing file: {path}")
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


def _safe_id(dataset_id: str) -> str:
    return dataset_id.replace("/", "__")


def _artifact(path: Path) -> dict[str, Any]:
    return {"path": str(path), "sha256": _sha256(path), "bytes": path.stat().st_size}


def _route_specs() -> dict[str, RouteSpec]:
    g6 = ROOT / "output" / "smartphone-r5" / "generalization-v6" / "routes"
    conservative = (
        ROOT
        / "output"
        / "smartphone-r5"
        / "reacquisition-conservative-v3"
        / "new-validation-materialized"
        / "routes"
    )
    conservative_outputs = (
        ROOT / "output" / "smartphone-r5" / "reacquisition-conservative-v3" / "routes"
    )
    segment = (
        ROOT
        / "output"
        / "smartphone-r5"
        / "segment-stability-v1"
        / "new-validation-materialized"
        / "routes"
    )
    segment_outputs = ROOT / "output" / "smartphone-r5" / "segment-stability-v1" / "routes"
    main_device = previous.DEFAULT_MAIN_DEVICE
    main_truth = previous.DEFAULT_MAIN_TRUTH
    main_position = ROOT / "output" / "smartphone-r5" / "hatch-full-w30" / "libgnsspp_spp.pos"
    main_existing = segment_outputs / "2023-05-24-20-26-us-ca-sjc-ge2__pixel7pro" / "existing-smoother" / "smoothed.pos"
    specs = {
        "2021-01-04-21-50-us-ca-e1highway280driveroutea/mi8": RouteSpec(
            "2021-01-04-21-50-us-ca-e1highway280driveroutea/mi8",
            "candidate-train",
            g6 / "2021-01-04-21-50-us-ca-e1highway280driveroutea" / "mi8" / "inputs" / "device_gnss.csv",
            g6 / "2021-01-04-21-50-us-ca-e1highway280driveroutea" / "mi8" / "inputs" / "ground_truth.csv",
            g6 / "2021-01-04-21-50-us-ca-e1highway280driveroutea" / "mi8" / "spp" / "canonical.pos",
            g6 / "2021-01-04-21-50-us-ca-e1highway280driveroutea" / "mi8" / "baseline" / "smoothed.pos",
        ),
        "2022-08-04-20-07-us-ca-sjc-q/pixel5": RouteSpec(
            "2022-08-04-20-07-us-ca-sjc-q/pixel5",
            "candidate-train",
            g6 / "2022-08-04-20-07-us-ca-sjc-q" / "pixel5" / "inputs" / "device_gnss.csv",
            g6 / "2022-08-04-20-07-us-ca-sjc-q" / "pixel5" / "inputs" / "ground_truth.csv",
            g6 / "2022-08-04-20-07-us-ca-sjc-q" / "pixel5" / "spp" / "canonical.pos",
            g6 / "2022-08-04-20-07-us-ca-sjc-q" / "pixel5" / "baseline" / "smoothed.pos",
        ),
        "2023-03-08-21-34-us-ca-mtv-u/pixel6pro": RouteSpec(
            "2023-03-08-21-34-us-ca-mtv-u/pixel6pro",
            "fixed-validation",
            g6 / "2023-03-08-21-34-us-ca-mtv-u" / "pixel6pro" / "inputs" / "device_gnss.csv",
            g6 / "2023-03-08-21-34-us-ca-mtv-u" / "pixel6pro" / "inputs" / "ground_truth.csv",
            g6 / "2023-03-08-21-34-us-ca-mtv-u" / "pixel6pro" / "spp" / "canonical.pos",
            g6 / "2023-03-08-21-34-us-ca-mtv-u" / "pixel6pro" / "baseline" / "smoothed.pos",
        ),
        "2021-03-16-20-40-us-ca-mtv-b/pixel4xl": RouteSpec(
            "2021-03-16-20-40-us-ca-mtv-b/pixel4xl",
            "additional-truth-opened-audit",
            conservative / "2021-03-16-20-40-us-ca-mtv-b" / "pixel4xl" / "inputs" / "device_gnss.csv",
            conservative / "2021-03-16-20-40-us-ca-mtv-b" / "pixel4xl" / "inputs" / "ground_truth.csv",
            conservative_outputs / "2021-03-16-20-40-us-ca-mtv-b__pixel4xl" / "truth-free-pipeline" / "spp" / "canonical.pos",
            conservative_outputs / "2021-03-16-20-40-us-ca-mtv-b__pixel4xl" / "existing-smoother" / "smoothed.pos",
        ),
        "2021-07-14-20-50-us-ca-mtv-e/pixel4": RouteSpec(
            "2021-07-14-20-50-us-ca-mtv-e/pixel4",
            "additional-truth-opened-audit",
            segment / "2021-07-14-20-50-us-ca-mtv-e" / "pixel4" / "inputs" / "device_gnss.csv",
            segment / "2021-07-14-20-50-us-ca-mtv-e" / "pixel4" / "inputs" / "ground_truth.csv",
            segment_outputs / "2021-07-14-20-50-us-ca-mtv-e__pixel4" / "truth-free-pipeline" / "spp" / "canonical.pos",
            segment_outputs / "2021-07-14-20-50-us-ca-mtv-e__pixel4" / "existing-smoother" / "smoothed.pos",
        ),
        MAIN_ID: RouteSpec(
            MAIN_ID,
            "development-main-regression",
            main_device,
            main_truth,
            main_position,
            main_existing,
        ),
    }
    if tuple(specs) != ALL_IDS:
        raise WlsEvaluationError("route spec order differs from frozen role contract")
    return specs


def _load_selection_record(path: Path) -> dict[str, Any]:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise WlsEvaluationError(f"invalid WLS selection record: {path}") from exc
    if not isinstance(payload, dict) or payload.get("schema_version") != "smartphone-r5-wls-position-selection.v1":
        raise WlsEvaluationError("WLS selection record schema is not v1")
    if payload.get("status") != "selection-frozen-before-evaluation":
        raise WlsEvaluationError("WLS selection record is not frozen before evaluation")
    roles = payload.get("fixed_roles")
    if not isinstance(roles, dict):
        raise WlsEvaluationError("WLS selection record has no fixed roles")
    if tuple(roles.get("candidate_train", ())) != TRAIN_IDS:
        raise WlsEvaluationError("WLS train role differs from frozen contract")
    if tuple(roles.get("validation", ())) != VALIDATION_IDS:
        raise WlsEvaluationError("WLS validation role differs from frozen contract")
    if tuple(roles.get("development_main_regression", ())) != (MAIN_ID,):
        raise WlsEvaluationError("WLS main role differs from frozen contract")
    if tuple(roles.get("additional_truth_opened_audit", ())) != AUDIT_IDS:
        raise WlsEvaluationError("WLS audit role differs from frozen contract")
    if payload.get("candidate_set") != list(CANDIDATES):
        raise WlsEvaluationError("WLS candidate set differs from frozen contract")
    sealed = payload.get("sealed_data_policy")
    if not isinstance(sealed, dict) or sealed.get("designated_holdout_id") != HOLDOUT_ID:
        raise WlsEvaluationError("WLS holdout differs from frozen contract")
    return payload


def _raw_rows(positions: list[smoother.PositionRow]) -> list[smoother.SmoothedRow]:
    return previous._raw_rows(positions)


def _score(
    rows: list[smoother.SmoothedRow],
    positions: list[smoother.PositionRow],
    epochs: list[int],
    truth: dict[int, tuple[float, float, float]],
) -> dict[str, Any]:
    return previous._score(rows, positions, epochs, truth)


def _aggregate(metrics: list[dict[str, Any]]) -> dict[str, Any]:
    if not metrics:
        raise WlsEvaluationError("cannot aggregate empty metrics")
    result: dict[str, Any] = {
        "route_count": len(metrics),
        "mean_availability_ratio": sum(float(m["availability_ratio"]) for m in metrics) / len(metrics),
        "mean_truth_coverage_ratio": sum(float(m["truth_coverage_ratio"]) for m in metrics) / len(metrics),
        "mean_horizontal_wgs84_p50_m": sum(float(m["horizontal_wgs84_m"]["p50_m"]) for m in metrics) / len(metrics),
        "mean_horizontal_wgs84_p95_m": sum(float(m["horizontal_wgs84_m"]["p95_m"]) for m in metrics) / len(metrics),
        "mean_vertical_p95_abs_m": sum(float(m["vertical_p95_abs_m"]) for m in metrics) / len(metrics),
    }
    result["mean_kaggle_diagnostic_score_variants_m"] = {
        key: sum(float(m["kaggle_diagnostic_score_variants_m"][key]) for m in metrics) / len(metrics)
        for key in DIAGNOSTIC_KEYS
    }
    result["mean_kaggle_diagnostic_m"] = sum(
        result["mean_kaggle_diagnostic_score_variants_m"].values()
    ) / len(DIAGNOSTIC_KEYS)
    return result


def _metric(metrics: dict[str, Any], path: tuple[str, ...]) -> float:
    value: Any = metrics
    for key in path:
        value = value[key]
    if value is None or not math.isfinite(float(value)):
        return math.inf
    return float(value)


def _compare(candidate: dict[str, Any], references: dict[str, dict[str, Any]]) -> dict[str, Any]:
    failures: dict[str, list[str]] = {}
    for name, reference in references.items():
        current: list[str] = []
        if _metric(candidate, ("availability_ratio",)) < _metric(reference, ("availability_ratio",)) - 1e-12:
            current.append("availability_regression")
        for path, label in (
            (("horizontal_wgs84_m", "p50_m"), "h_median_regression"),
            (("horizontal_wgs84_m", "p95_m"), "h_p95_regression"),
            (("vertical_p95_abs_m",), "v_p95_regression"),
        ):
            if _metric(candidate, path) > _metric(reference, path) + 1e-12:
                current.append(label)
        for key in DIAGNOSTIC_KEYS:
            if _metric(candidate, ("kaggle_diagnostic_score_variants_m", key)) > _metric(
                reference, ("kaggle_diagnostic_score_variants_m", key)
            ) + 1e-12:
                current.append(f"{key}_regression")
        failures[name] = current
    return {
        "non_regression_passed": all(not current for current in failures.values()),
        "failures_by_reference": failures,
    }


def _compare_aggregate(candidate: dict[str, Any], reference: dict[str, Any]) -> tuple[bool, list[str]]:
    return previous._aggregate_non_regression(candidate, reference)


def _candidate_rank(value: dict[str, Any]) -> tuple[float, ...]:
    aggregate = value["train_aggregate"]
    return (
        float(aggregate["mean_horizontal_wgs84_p95_m"]),
        float(aggregate["mean_horizontal_wgs84_p50_m"]),
        float(aggregate["mean_vertical_p95_abs_m"]),
        float(aggregate["mean_kaggle_diagnostic_m"]),
        str(value["candidate"]["id"]),
    )


def _write_candidate_outputs(
    result: smoother.SmoothingResult,
    stability_report: dict[str, Any],
    output_dir: Path,
    wls_position: Path,
    device_gnss: Path,
) -> dict[str, Any]:
    output_dir.mkdir(parents=True, exist_ok=True)
    smoother.write_outputs(
        result,
        smoother.SmootherConfig(**BASELINE_CONFIG),
        output_dir,
        position_path=wls_position,
        device_path=device_gnss,
        skip_epochs=1,
        leap_seconds=18,
    )
    report_path = output_dir / "segment_stability.json"
    report = {
        **stability_report,
        "raw_position": {"path": str(wls_position), "sha256": _sha256(wls_position)},
        "truth_used": False,
        "position_lane": "android-handset-wls-ecef",
    }
    _atomic_json(report_path, report)
    manifest_path = output_dir / "smoother_manifest.json"
    try:
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise WlsEvaluationError(f"candidate manifest is invalid: {manifest_path}") from exc
    manifest["position_lane"] = "android-handset-wls-ecef"
    manifest["segment_stability"] = {
        "path": str(report_path),
        "sha256": _sha256(report_path),
        "raw_position": report["raw_position"],
        "thresholds": report["thresholds"],
        "population": report["population"],
    }
    manifest.setdefault("artifacts", {})["segment_stability"] = _artifact(report_path)
    _atomic_json(manifest_path, manifest)
    return {
        "manifest": {"path": str(manifest_path), "sha256": _sha256(manifest_path)},
        "report": {"path": str(report_path), "sha256": _sha256(report_path)},
        "position": {"path": str(output_dir / "smoothed.pos"), "sha256": _sha256(output_dir / "smoothed.pos")},
    }


def _run_truth_free_route(spec: RouteSpec, output_root: Path) -> dict[str, Any]:
    for path in (spec.device_gnss, spec.native_position, spec.native_existing_smoother):
        if not path.is_file():
            raise WlsEvaluationError(f"missing fixed route input for {spec.dataset_id}: {path}")
    route_root = output_root / "routes" / _safe_id(spec.dataset_id)
    wls_dir = route_root / "wls"
    started = time.perf_counter()
    wls_manifest = wls.extract_to_directory(
        spec.device_gnss,
        wls_dir,
        skip_epochs=1,
        role="development",
        dataset_id=spec.dataset_id,
    )
    extraction_wall = time.perf_counter() - started
    wls_position_path = wls_dir / "wls.pos"
    positions = smoother._read_positions(wls_position_path, 18)
    epochs = smoother._read_device_epochs(spec.device_gnss, 1)
    if [row.timestamp_ms for row in positions] != epochs:
        raise WlsEvaluationError(f"WLS POS keys do not equal device keys: {spec.dataset_id}")
    native_positions = smoother._read_positions(spec.native_position, 18)
    if [row.timestamp_ms for row in native_positions] != epochs:
        raise WlsEvaluationError(f"native POS keys do not equal device keys: {spec.dataset_id}")
    config = smoother.SmootherConfig(**BASELINE_CONFIG)
    wls_baseline = smoother.smooth_positions(positions, epochs, config)
    native_baseline = smoother.smooth_positions(native_positions, epochs, config)
    candidate_results: dict[str, smoother.SmoothingResult] = {}
    candidate_reports: dict[str, dict[str, Any]] = {}
    candidate_artifacts: dict[str, dict[str, Any]] = {}
    for candidate in CANDIDATES:
        candidate_id = str(candidate["id"])
        result, stability_report = segment_stability.apply_segment_stability(
            wls_baseline,
            positions,
            epochs,
            max_consecutive_rejects=int(candidate["max_consecutive_rejects"]),
            max_prediction_duration_s=float(candidate["max_prediction_duration_s"]),
            reject_fraction_max=candidate["reject_fraction_max"],
            measurement_floor_m=float(BASELINE_CONFIG["measurement_floor_m"]),
            leap_seconds=18,
        )
        candidate_results[candidate_id] = result
        candidate_reports[candidate_id] = stability_report
        candidate_artifacts[candidate_id] = _write_candidate_outputs(
            result,
            stability_report,
            route_root / candidate_id,
            wls_position_path,
            spec.device_gnss,
        )
    native_recommended, native_stability = segment_stability.apply_segment_stability(
        native_baseline,
        native_positions,
        epochs,
        max_consecutive_rejects=15,
        max_prediction_duration_s=15.0,
        reject_fraction_max=None,
        measurement_floor_m=float(BASELINE_CONFIG["measurement_floor_m"]),
        leap_seconds=18,
    )
    return {
        "spec": spec,
        "positions": positions,
        "epochs": epochs,
        "native_positions": native_positions,
        "wls_baseline": wls_baseline,
        "native_baseline": native_baseline,
        "native_recommended": native_recommended,
        "native_stability_report": native_stability,
        "candidates": candidate_results,
        "candidate_reports": candidate_reports,
        "candidate_artifacts": candidate_artifacts,
        "route_root": route_root,
        "wls_manifest": {
            "path": str(wls_dir / "wls_manifest.json"),
            "sha256": _sha256(wls_dir / "wls_manifest.json"),
        },
        "wls_summary": _artifact(wls_dir / "wls_summary.json"),
        "wls_position": _artifact(wls_position_path),
        "native_position": _artifact(spec.native_position),
        "native_existing_smoother": _artifact(spec.native_existing_smoother),
        "input_hashes": {
            "device_gnss": _sha256(spec.device_gnss),
            "native_position": _sha256(spec.native_position),
            "native_existing_smoother": _sha256(spec.native_existing_smoother),
        },
        "timing": {
            "wls_extraction_and_conversion_wall_s": extraction_wall,
            "wls_smoother_wall_s": wls_baseline.elapsed_s,
            "native_smoother_wall_s": native_baseline.elapsed_s,
        },
        "wls_manifest_payload": wls_manifest,
    }


def _score_routes(
    routes: dict[str, dict[str, Any]],
    route_ids: tuple[str, ...],
    lane: str,
) -> dict[str, dict[str, Any]]:
    scored: dict[str, dict[str, Any]] = {}
    for dataset_id in route_ids:
        route = routes[dataset_id]
        truth_path = route["spec"].truth
        truth = smoother_eval._read_truth(truth_path)
        route.setdefault("truth_hash", _sha256(truth_path))
        if lane == "wls_raw":
            rows = _raw_rows(route["positions"])
        elif lane == "native_raw":
            rows = _raw_rows(route["native_positions"])
        elif lane == "native_existing_smoother":
            rows = smoother._read_positions(route["spec"].native_existing_smoother, 18)
            rows = _raw_rows(rows)
        elif lane == "native_segment_stability":
            rows = route["native_recommended"].rows
        elif lane == "wls_existing_smoother":
            rows = route["wls_baseline"].rows
        elif lane.startswith("candidate:"):
            rows = route["candidates"][lane.split(":", 1)[1]].rows
        else:
            raise WlsEvaluationError(f"unknown score lane: {lane}")
        score_positions = route["positions"] if lane.startswith("wls") or lane.startswith("candidate:") else route["native_positions"]
        scored[dataset_id] = _score(rows, score_positions, route["epochs"], truth)
    return scored


def _select_candidate(
    routes: dict[str, dict[str, Any]],
    train_wls_raw: dict[str, dict[str, Any]],
) -> tuple[dict[str, Any], str | None]:
    train_raw_aggregate = _aggregate([train_wls_raw[dataset_id] for dataset_id in TRAIN_IDS])
    values: dict[str, Any] = {}
    for candidate in CANDIDATES:
        candidate_id = str(candidate["id"])
        scores = _score_routes(routes, TRAIN_IDS, f"candidate:{candidate_id}")
        aggregate = _aggregate([scores[dataset_id] for dataset_id in TRAIN_IDS])
        passed, failures = _compare_aggregate(aggregate, train_raw_aggregate)
        values[candidate_id] = {
            "candidate": candidate,
            "train_aggregate": aggregate,
            "train_non_regression_vs_wls_raw": passed,
            "train_non_regression_failures": failures,
            "scores": scores,
        }
    eligible = [value for value in values.values() if value["train_non_regression_vs_wls_raw"]]
    selected = min(eligible, key=_candidate_rank) if eligible else None
    selected_id = str(selected["candidate"]["id"]) if selected else None
    return {
        "wls_raw_train_aggregate": train_raw_aggregate,
        "candidates": values,
        "selected_candidate_id": selected_id,
        "selection_eligible": bool(eligible),
    }, selected_id


def _mixed_route_analysis(
    raw_wls: dict[str, dict[str, Any]],
    native_raw: dict[str, dict[str, Any]],
    routes: dict[str, dict[str, Any]],
) -> dict[str, Any]:
    wls_better: list[str] = []
    wls_worse: list[str] = []
    ties: list[str] = []
    integrity: dict[str, Any] = {}
    for dataset_id in ALL_IDS:
        wls_p95 = _metric(raw_wls[dataset_id], ("horizontal_wgs84_m", "p95_m"))
        native_p95 = _metric(native_raw[dataset_id], ("horizontal_wgs84_m", "p95_m"))
        if wls_p95 < native_p95 - 1e-12:
            wls_better.append(dataset_id)
        elif wls_p95 > native_p95 + 1e-12:
            wls_worse.append(dataset_id)
        else:
            ties.append(dataset_id)
        summary = json.loads(
            (routes[dataset_id]["route_root"] / "wls" / "wls_summary.json").read_text(encoding="utf-8")
        )
        validation = summary["validation"]
        populations = summary["populations"]
        integrity[dataset_id] = {
            "selected_epoch_ratio": populations["selected_epochs"] / populations["input_epochs"],
            "max_timestamp_gap_s": populations["max_timestamp_gap_s"],
            "clock_discontinuity_transition_count": populations["clock_discontinuity_transition_count"],
            "ecef_consistency_tolerance_m": validation["ecef_consistency_tolerance_m"],
            "all_epoch_rows_consistent": validation["all_epoch_rows_consistent"],
            "all_selected_rows_finite": validation["all_selected_rows_finite"],
            "min_svid_count": populations["min_svid_count"],
            "max_svid_count": populations["max_svid_count"],
        }
    mixed = bool(wls_better and wls_worse)
    return {
        "horizontal_p95_route_outcomes": {
            "wls_better": wls_better,
            "wls_worse": wls_worse,
            "tie": ties,
            "mixed": mixed,
        },
        "truth_free_integrity_features": integrity,
        "runtime_selector": {
            "implemented": False,
            "decision": "not-implemented-truth-free-accuracy-selector",
            "reason": "integrity fields can fail closed but are not calibrated to predict route accuracy; truth-dependent route labels are audit-only",
            "safe_future_use": "use integrity features only to reject malformed WLS, then require an externally frozen lane choice",
        },
    }


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        prog=os.environ.get("GNSS_CLI_NAME", "gnss smartphone-wls-position-eval")
    )
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    parser.add_argument("--selection-record", type=Path, default=DEFAULT_SELECTION_RECORD)
    return parser.parse_args(argv)


def run(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    try:
        selection_record = _load_selection_record(args.selection_record)
        specs = _route_specs()
        args.output_dir.mkdir(parents=True, exist_ok=True)
        started = time.perf_counter()
        # This loop is deliberately truth-free.  ``RouteSpec.truth`` is not
        # opened until the first ordered scoring call below.
        routes = {
            dataset_id: _run_truth_free_route(spec, args.output_dir)
            for dataset_id, spec in specs.items()
        }
        train_wls_raw = _score_routes(routes, TRAIN_IDS, "wls_raw")
        selection, selected_id = _select_candidate(routes, train_wls_raw)
        native_raw = _score_routes(routes, ALL_IDS, "native_raw")
        native_segment = _score_routes(routes, ALL_IDS, "native_segment_stability")
        wls_raw_all = _score_routes(routes, ALL_IDS, "wls_raw")
        candidate_scores_all = {
            str(candidate["id"]): _score_routes(
                routes, ALL_IDS, f"candidate:{candidate['id']}"
            )
            for candidate in CANDIDATES
        }
        validation_gate: dict[str, Any] | None = None
        main_gate: dict[str, Any] | None = None
        selected_all: dict[str, dict[str, Any]] = {}
        if selected_id is not None:
            selected_all = candidate_scores_all[selected_id]
            validation_gate = {
                dataset_id: _compare(
                    selected_all[dataset_id],
                    {
                        "wls_raw": wls_raw_all[dataset_id],
                        "native_segment_stability": native_segment[dataset_id],
                    },
                )
                for dataset_id in VALIDATION_IDS
            }
            main_gate = _compare(
                selected_all[MAIN_ID],
                {
                    "wls_raw": wls_raw_all[MAIN_ID],
                    "native_segment_stability": native_segment[MAIN_ID],
                },
            )
            validation_passed = all(
                value["non_regression_passed"] for value in validation_gate.values()
            )
            main_passed = main_gate["non_regression_passed"]
            promotion = (
                "promote-development-only"
                if validation_passed and main_passed
                else "no-go-validation-or-main-regression"
            )
        else:
            promotion = "no-go-no-eligible-train-candidate"
        audit_selected = {
            dataset_id: selected_all[dataset_id]
            for dataset_id in AUDIT_IDS
        } if selected_id is not None else {}
        mixed_analysis = _mixed_route_analysis(wls_raw_all, native_raw, routes)
        route_artifacts: dict[str, Any] = {}
        for dataset_id, route in routes.items():
            route_artifacts[dataset_id] = {
                "role": route["spec"].role,
                "device_gnss": str(route["spec"].device_gnss),
                "truth": str(route["spec"].truth),
                "truth_sha256": route.get("truth_hash"),
                "wls_manifest": route["wls_manifest"],
                "wls_summary": route["wls_summary"],
                "wls_position": route["wls_position"],
                "native_position": route["native_position"],
                "native_existing_smoother": route["native_existing_smoother"],
                "native_segment_stability": {
                    "population": route["native_stability_report"]["population"],
                    "segments": route["native_stability_report"]["segments"],
                },
                "wls_candidates": route["candidate_artifacts"],
                "timing": route["timing"],
            }
        report = {
            "schema_version": SCHEMA_VERSION,
            "decision": "development-only-wls-position-evaluation",
            "selection_record": str(args.selection_record),
            "truth_free_generation": True,
            "holdout_content_opened": False,
            "holdout_truth_opened": False,
            "holdout_materialized": False,
            "archive_or_new_route_access": "none; all six route inputs were already truth-opened/materialized",
            "roles": {
                "candidate_train": list(TRAIN_IDS),
                "validation": list(VALIDATION_IDS),
                "development_main_regression": [MAIN_ID],
                "additional_truth_opened_audit": list(AUDIT_IDS),
                "holdout": HOLDOUT_ID,
            },
            "evaluation_contract": {
                "wls_fields": list(wls.EXPECTED_FIELDS[-3:]),
                "consistency_tolerance_m": wls.DEFAULT_CONSISTENCY_TOLERANCE_M,
                "ecef_norm_range_m": [wls.ECEF_NORM_MIN_M, wls.ECEF_NORM_MAX_M],
                "timestamp_and_clock": "strict timestamp order; epoch-consistent and monotonic HardwareClockDiscontinuityCount",
                "missing_policy": "fail-closed with explicit classification",
                "quality_fields": "WLS has no PDOP/fix covariance; POS uses status=1, unique Svid count, and zero unknown quality fields",
                "segment_gate": "shared trajectory smoother plus fixed 15/20/30 reject and prediction bounds; reject fraction recorded only",
                "truth_free_runtime_selector": "not implemented; integrity-only selector analysis is reported",
            },
            "selection": selection,
            "promotion_decision": promotion,
            "selected_candidate_id": selected_id,
            "validation_gate": validation_gate,
            "development_main_gate": main_gate,
            "audit_selected": audit_selected,
            "scores": {
                "wls_raw": wls_raw_all,
                "native_galileo_e1_hatch_raw": native_raw,
                "native_segment_stability_recommended": native_segment,
                "wls_segment_stability_candidates": candidate_scores_all,
                "selected_wls_segment_stability": selected_all if selected_id is not None else None,
            },
            "mixed_route_analysis": mixed_analysis,
            "route_artifacts": route_artifacts,
            "timing": {
                "total_wall_s": time.perf_counter() - started,
                "route_count": len(routes),
                "truth_free_generation_completed_before_scoring": True,
            },
        }
        report_path = args.output_dir / "wls_position_report.json"
        _atomic_json(report_path, report)
        manifest_path = args.output_dir / "wls_position_manifest.json"
        manifest = {
            "schema_version": "smartphone-r5-wls-position-evaluation-manifest.v1",
            "truth_used_for_generation": False,
            "holdout_content_opened": False,
            "holdout_truth_opened": False,
            "selection_record": {"path": str(args.selection_record), "sha256": _sha256(args.selection_record)},
            "report": {"path": str(report_path), "sha256": _sha256(report_path), "bytes": report_path.stat().st_size},
            "routes": route_artifacts,
        }
        _atomic_json(manifest_path, manifest)
        print(f"Smartphone WLS evaluation complete: {report_path}")
        return 0
    except (WlsEvaluationError, wls.WlsPositionError, smoother.SmootherError, ValueError) as exc:
        print(f"Smartphone WLS evaluation failed: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(run())
