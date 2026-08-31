#!/usr/bin/env python3
"""Train-only evaluation for the frozen raw-quality robust SPP candidate.

The route inputs are fixed by ``smartphone_r5_raw_quality_control_selection``.
All truth-free quality reports, native solver outputs, fallback POS files, and
hash manifests are produced first.  Only after that seal does this evaluator
open the three development truth files.  A failed train gate stops before the
fresh validation and future holdout roles; this command has no holdout mode.
"""

from __future__ import annotations

import argparse
from collections import defaultdict
import hashlib
import json
import math
import os
from pathlib import Path
import sys
import time
from typing import Any

# Keep direct invocation equivalent to dispatcher invocation.
_COMMANDS_DIR = Path(__file__).resolve().parents[1]
if str(_COMMANDS_DIR) not in sys.path:
    sys.path.insert(0, str(_COMMANDS_DIR))
from support.gnss_runtime import application_root


ROOT = application_root(__file__)
BENCHMARK_DIR = ROOT / "apps" / "commands" / "benchmarks"
if str(BENCHMARK_DIR) not in sys.path:
    sys.path.insert(0, str(BENCHMARK_DIR))
import gnss_smartphone_raw_quality_control as quality  # noqa: E402
import gnss_smartphone_kaggle as kaggle  # noqa: E402
import gnss_smartphone_trajectory_smoother as smoother  # noqa: E402
import gnss_smartphone_trajectory_smoother_eval as smoother_eval  # noqa: E402
import gnss_smartphone_wls_eval as wls_eval  # noqa: E402


SCHEMA_VERSION = "smartphone-r5-raw-quality-control-evaluation.v1"
MANIFEST_SCHEMA_VERSION = "smartphone-r5-raw-quality-control-evaluation-manifest.v1"
SELECTION_RECORD = ROOT / "docs" / "use_cases" / "records" / "smartphone_r5_raw_quality_control_selection.json"
DEFAULT_OUTPUT_DIR = ROOT / "output" / "smartphone-r5" / "raw-quality-control-v1"
TRAIN_IDS = (
    "2021-01-04-21-50-us-ca-e1highway280driveroutea/mi8",
    "2022-08-04-20-07-us-ca-sjc-q/pixel5",
    "2023-05-24-20-26-us-ca-sjc-ge2/pixel7pro",
)
FRESH_VALIDATION_ID = "2023-09-07-18-59-us-ca/pixel5"
FUTURE_HOLDOUT_ID = "2023-09-06-00-01-us-ca-routen/pixel6pro"
DIAGNOSTIC_KEYS = tuple(wls_eval.DIAGNOSTIC_KEYS)
HOLDOUT_IDS = {FUTURE_HOLDOUT_ID, "2023-05-25-19-10-us-ca-sjc-be2/sm-s908b"}


class EvaluationError(ValueError):
    """Raised when the frozen evaluation contract cannot be satisfied."""


def _sha256(path: Path) -> str:
    return quality._sha256(path)


def _atomic_json(path: Path, payload: dict[str, Any]) -> None:
    quality._atomic_json(path, payload)


def _safe_id(dataset_id: str) -> str:
    return dataset_id.replace("/", "__")


def _artifact(path: Path) -> dict[str, Any]:
    return {"path": str(path), "sha256": _sha256(path), "bytes": path.stat().st_size}


def _load_selection(path: Path) -> dict[str, Any]:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise EvaluationError(f"invalid selection record: {path}") from exc
    if not isinstance(payload, dict) or payload.get("schema_version") != "smartphone-r5-gsdc2023-raw-quality-control-selection.v1":
        raise EvaluationError("raw quality selection record schema is invalid")
    if payload.get("status") != "selection-frozen-before-new-implementation":
        raise EvaluationError("raw quality selection record was not frozen before implementation")
    split = payload.get("frozen_split")
    if not isinstance(split, dict) or tuple(split.get("train", ())) != TRAIN_IDS:
        raise EvaluationError("raw quality train split differs from frozen record")
    if split.get("fresh_validation") != FRESH_VALIDATION_ID or split.get("future_holdout") != FUTURE_HOLDOUT_ID:
        raise EvaluationError("raw quality validation/holdout split differs from frozen record")
    if split.get("fresh_validation_truth_opened_before_freeze") != 0 or split.get("future_holdout_truth_opened_before_freeze") != 0:
        raise EvaluationError("selection record does not prove unopened validation/holdout truth")
    candidate = payload.get("candidate")
    if not isinstance(candidate, dict) or candidate.get("id") != "innovation_huber_native_spp_v1":
        raise EvaluationError("candidate id differs from frozen record")
    parameters = candidate.get("parameters")
    if parameters != quality.ROBUST_PARAMETERS:
        raise EvaluationError("robust SPP parameters differ from frozen record")
    if split.get("old_holdouts_or_leaderboard_used") is not False or payload.get("public_private_scores_used_for_tuning") is not False:
        raise EvaluationError("selection record permits forbidden score leakage")
    return payload


def _route_specs() -> dict[str, dict[str, Path]]:
    g6 = ROOT / "output" / "smartphone-r5" / "generalization-v6" / "routes"
    wls_root = ROOT / "output" / "smartphone-r5" / "wls-position-v1" / "routes"
    specs = {
        TRAIN_IDS[0]: {
            "device_gnss": g6 / "2021-01-04-21-50-us-ca-e1highway280driveroutea" / "mi8" / "inputs" / "device_gnss.csv",
            "truth": g6 / "2021-01-04-21-50-us-ca-e1highway280driveroutea" / "mi8" / "inputs" / "ground_truth.csv",
            "obs": g6 / "2021-01-04-21-50-us-ca-e1highway280driveroutea" / "mi8" / "adapter" / "rover.obs",
            "nav": g6 / "2021-01-04-21-50-us-ca-e1highway280driveroutea" / "mi8" / "inputs" / "brdc.nav",
            "fallback_position": wls_root / "2021-01-04-21-50-us-ca-e1highway280driveroutea__mi8" / "wls" / "wls.pos",
        },
        TRAIN_IDS[1]: {
            "device_gnss": g6 / "2022-08-04-20-07-us-ca-sjc-q" / "pixel5" / "inputs" / "device_gnss.csv",
            "truth": g6 / "2022-08-04-20-07-us-ca-sjc-q" / "pixel5" / "inputs" / "ground_truth.csv",
            "obs": g6 / "2022-08-04-20-07-us-ca-sjc-q" / "pixel5" / "adapter" / "rover.obs",
            "nav": g6 / "2022-08-04-20-07-us-ca-sjc-q" / "pixel5" / "inputs" / "brdc.nav",
            "fallback_position": wls_root / "2022-08-04-20-07-us-ca-sjc-q__pixel5" / "wls" / "wls.pos",
        },
        TRAIN_IDS[2]: {
            "device_gnss": ROOT / "data" / "gsdc2023" / "materialized" / "dataset_2023" / "train" / "2023-05-24-20-26-us-ca-sjc-ge2" / "pixel7pro" / "device_gnss.csv",
            "truth": ROOT / "data" / "gsdc2023" / "materialized" / "dataset_2023" / "train" / "2023-05-24-20-26-us-ca-sjc-ge2" / "pixel7pro" / "ground_truth.csv",
            "obs": ROOT / "output" / "smartphone-r5" / "hatch-full-w30" / "rover.obs",
            "nav": ROOT / "data" / "gsdc2023" / "materialized" / "dataset_2023" / "train" / "2023-05-24-20-26-us-ca-sjc-ge2" / "brdc.nav",
            "fallback_position": wls_root / "2023-05-24-20-26-us-ca-sjc-ge2__pixel7pro" / "wls" / "wls.pos",
        },
    }
    if tuple(specs) != TRAIN_IDS:
        raise EvaluationError("route order differs from frozen train contract")
    for dataset_id, spec in specs.items():
        for name, path in spec.items():
            if not path.is_file():
                raise EvaluationError(f"missing {name} for {dataset_id}: {path}")
    return specs


def _metric(metrics: dict[str, Any], path: tuple[str, ...]) -> float:
    value: Any = metrics
    for key in path:
        value = value[key]
    if value is None or not math.isfinite(float(value)):
        return math.inf
    return float(value)


def _compare_route(candidate: dict[str, Any], baseline: dict[str, Any]) -> dict[str, Any]:
    failures: list[str] = []
    if _metric(candidate, ("availability_ratio",)) < _metric(baseline, ("availability_ratio",)) - 1e-12:
        failures.append("availability_regression")
    if _metric(candidate, ("truth_coverage_ratio",)) < _metric(baseline, ("truth_coverage_ratio",)) - 1e-12:
        failures.append("truth_coverage_regression")
    for path, label in (
        (("horizontal_wgs84_m", "p50_m"), "h_p50_regression"),
        (("horizontal_wgs84_m", "p95_m"), "h_p95_regression"),
        (("vertical_p95_abs_m",), "v_p95_regression"),
    ):
        if _metric(candidate, path) > _metric(baseline, path) + 1e-12:
            failures.append(label)
    for key in DIAGNOSTIC_KEYS:
        if _metric(candidate, ("kaggle_diagnostic_score_variants_m", key)) > _metric(baseline, ("kaggle_diagnostic_score_variants_m", key)) + 1e-12:
            failures.append(f"{key}_regression")
    return {"passed": not failures, "failures": failures}


def _aggregate_gate(candidate: dict[str, Any], baseline: dict[str, Any]) -> dict[str, Any]:
    failures: list[str] = []
    for key in ("mean_availability_ratio", "mean_truth_coverage_ratio"):
        if float(candidate[key]) < float(baseline[key]) - 1e-12:
            failures.append(f"{key}_regression")
    if float(candidate["mean_horizontal_wgs84_p95_m"]) > float(baseline["mean_horizontal_wgs84_p95_m"]) + 1e-12:
        failures.append("aggregate_h_p95_not_improved")
    if float(candidate["mean_horizontal_wgs84_p50_m"]) > float(baseline["mean_horizontal_wgs84_p50_m"]) + 0.01:
        failures.append("aggregate_h_p50_regression_over_0p01m")
    if float(candidate["mean_vertical_p95_abs_m"]) > float(baseline["mean_vertical_p95_abs_m"]) + 5.0:
        failures.append("aggregate_v_p95_major_regression")
    for key in DIAGNOSTIC_KEYS:
        if float(candidate["mean_kaggle_diagnostic_score_variants_m"][key]) > float(baseline["mean_kaggle_diagnostic_score_variants_m"][key]) + 1e-6:
            failures.append(f"{key}_not_improved")
    return {
        "passed": not failures,
        "failures": failures,
        "strict_h_p95_improvement": float(candidate["mean_horizontal_wgs84_p95_m"]) < float(baseline["mean_horizontal_wgs84_p95_m"]) - 1e-12,
        "strict_diagnostic_mean_improvement": float(candidate["mean_kaggle_diagnostic_m"]) < float(baseline["mean_kaggle_diagnostic_m"]) - 1e-12,
    }


def _quality_bucket(value: float | None, kind: str) -> str:
    if value is None or not math.isfinite(value):
        return "unavailable"
    if kind == "sigma":
        return "le_3" if value <= 3.0 else "le_5" if value <= 5.0 else "gt_5"
    if kind == "uncertainty":
        return "le_5m" if value <= 5.0 else "le_10m" if value <= 10.0 else "gt_10m"
    if kind == "cn0":
        return "lt_30" if value < 30.0 else "lt_35" if value < 35.0 else "ge_35"
    if kind == "gdop":
        return "le_2" if value <= 2.0 else "le_5" if value <= 5.0 else "gt_5"
    if kind == "adr":
        return "lt_0p5" if value < 0.5 else "lt_0p8" if value < 0.8 else "ge_0p8"
    return str(value)


def _quality_error_join(
    quality_report_path: Path,
    positions: list[smoother.PositionRow],
    truth: dict[int, tuple[float, float, float]],
) -> dict[str, Any]:
    payload = json.loads(quality_report_path.read_text(encoding="utf-8"))
    features = {int(item["timestamp_ms"]): item for item in payload.get("epoch_features", [])}
    buckets: defaultdict[str, dict[str, list[float]]] = defaultdict(
        lambda: {"horizontal": [], "vertical": []}
    )
    values: list[float] = []
    for position in positions:
        feature = features.get(position.timestamp_ms)
        if feature is None:
            continue
        reference = smoother_eval._match_truth(position.timestamp_ms, truth, 100)
        if reference is None:
            continue
        horizontal = kaggle._wgs84_horizontal_distance_m(
            position.latitude, position.longitude, reference[0], reference[1]
        )
        vertical = abs(position.height - reference[2])
        values.append(horizontal)
        keys = (
            ("standardized_abs_residual_p95_sigma", "sigma"),
            ("raw_pseudorange_uncertainty_p95_m", "uncertainty"),
            ("cn0_median_dbhz", "cn0"),
            ("gdop_proxy", "gdop"),
            ("adr_valid_fraction", "adr"),
        )
        for feature_key, bucket_kind in keys:
            bucket = buckets[f"{feature_key}={_quality_bucket(feature.get(feature_key), bucket_kind)}"]
            bucket["horizontal"].append(horizontal)
            bucket["vertical"].append(vertical)
        signal_set = "+".join(sorted(feature.get("signal_counts", {}))) or "none"
        frequency_set = "+".join(sorted(feature.get("frequencies_hz", {}))) or "none"
        gap_value = feature.get("epoch_gap_s")
        if gap_value is None:
            gap_bucket = "first_or_unavailable"
        elif gap_value <= 1.5:
            gap_bucket = "le_1p5s"
        elif gap_value <= 10.0:
            gap_bucket = "le_10s"
        else:
            gap_bucket = "gt_10s"
        for key in (
            f"signals={signal_set}",
            f"frequencies_hz={frequency_set}",
            f"clock_discontinuity_count={feature.get('clock_discontinuity_count', 'unavailable')}",
            f"epoch_gap={gap_bucket}",
        ):
            bucket = buckets[key]
            bucket["horizontal"].append(horizontal)
            bucket["vertical"].append(vertical)
    result: dict[str, Any] = {}
    for key, errors in sorted(buckets.items()):
        horizontal_errors = errors["horizontal"]
        vertical_errors = errors["vertical"]
        result[key] = {
            "count": len(horizontal_errors),
            "horizontal_p50_m": kaggle._percentile_linear_n_minus_1(horizontal_errors, 0.50) if horizontal_errors else None,
            "horizontal_p95_m": kaggle._percentile_linear_n_minus_1(horizontal_errors, 0.95) if horizontal_errors else None,
            "vertical_p95_m": kaggle._percentile_linear_n_minus_1(vertical_errors, 0.95) if vertical_errors else None,
        }
    return {
        "schema_version": "smartphone-raw-quality-error-tail-join.v1",
        "truth_free_features_sealed_first": True,
        "truth_labels_used_only_posthoc": True,
        "matched_position_epochs": len(values),
        "feature_buckets": result,
    }


def _bounded_fgo_feasibility(selection_path: Path, output_dir: Path) -> dict[str, Any]:
    binary = ROOT / "build" / "apps" / "gnss_fgo"
    source_files = [
        ROOT / "apps" / "native" / "gnss_fgo.cpp",
        ROOT / "include" / "libgnss++" / "algorithms" / "fgo.hpp",
        ROOT / "src" / "algorithms" / "fgo.cpp",
        ROOT / "apps" / "commands" / "benchmarks" / "gnss_smartphone_gnss_adapter.py",
    ]
    present = [path for path in source_files if path.is_file()]
    record = {
        "schema_version": "smartphone-r5-gsdc2023-fgo-wiring-feasibility.v1",
        "status": "blocked-bounded-wiring-no-go",
        "truth_free": True,
        "external_mutation": False,
        "candidate_trigger": "robust raw-quality train gate failed; bounded alternative inspected after sealed No-Go",
        "evidence": {
            "fgo_binary": {"path": str(binary), "present": binary.is_file(), "sha256": _sha256(binary) if binary.is_file() else None},
            "source_files": [{"path": str(path), "sha256": _sha256(path)} for path in present],
            "adapter_output": "Android adapter produces RINEX rover observations and already validates GPS L1/Galileo E1; no direct Android-to-factor graph CLI exists",
            "fgo_input_contract": "existing FGO executable is RTK/factor-graph oriented and expects its own rover/base or problem inputs; it is not wired to handset raw CSV plus single-phone broadcast SPP",
        },
        "raw_data_constraint": "frozen smartphone routes provide a single handset observation stream, ADR/Doppler, satellite states, and broadcast nav; no independent base/reference stream is available for the existing DD-oriented FGO path",
        "decision": "do-not-rewrite-estimator-in-this-phase",
        "next_bounded_step": "define an Android pseudorange/clock factor adapter and explicit observability tests before any coupled FGO implementation; not a production change",
        "selection_record": {"path": str(selection_path), "sha256": _sha256(selection_path)},
    }
    path = output_dir / "fgo_wiring_feasibility.json"
    _atomic_json(path, record)
    return {"path": str(path), "sha256": _sha256(path)}


def run_train(output_dir: Path, selection_path: Path) -> int:
    selection = _load_selection(selection_path)
    specs = _route_specs()
    output_dir.mkdir(parents=True, exist_ok=True)
    started = time.perf_counter()
    routes: dict[str, dict[str, Any]] = {}
    # This loop is strictly truth-free.  The ``truth`` paths are only stored
    # as frozen metadata and are not opened until the seal below exists.
    for dataset_id in TRAIN_IDS:
        spec = specs[dataset_id]
        route_output = output_dir / "train" / _safe_id(dataset_id)
        result = quality.run_robust_candidate(
            spec["device_gnss"],
            spec["obs"],
            spec["nav"],
            spec["fallback_position"],
            route_output,
            dataset_id=dataset_id,
        )
        routes[dataset_id] = {
            "spec": spec,
            "result": result,
            "position_path": result["position_path"],
            "baseline_position": spec["fallback_position"],
            "quality_report_path": result["quality_report_path"],
            "truth_opened": False,
        }
    truth_free_seal = {
        "schema_version": "smartphone-r5-raw-quality-control-truth-free-seal.v1",
        "truth_free": True,
        "holdout_content_opened": False,
        "holdout_truth_opened": False,
        "validation_truth_opened": False,
        "train_truth_opened": False,
        "selection_record": {"path": str(selection_path), "sha256": _sha256(selection_path)},
        "routes": {
            dataset_id: {
                "candidate_manifest": _artifact(route["result"]["manifest_path"]),
                "candidate_position": _artifact(route["position_path"]),
                "quality_report": _artifact(route["quality_report_path"]),
                "baseline_position": _artifact(route["baseline_position"]),
                "truth_path": str(route["spec"]["truth"]),
                "truth_open_count": 0,
            }
            for dataset_id, route in routes.items()
        },
    }
    seal_path = output_dir / "truth_free_seal.json"
    _atomic_json(seal_path, truth_free_seal)
    seal_hash = _sha256(seal_path)
    # Truth is opened only after all candidate artifacts and the seal hash are
    # durable.  There is no validation/holdout branch in this train command.
    baseline_scores: dict[str, dict[str, Any]] = {}
    candidate_scores: dict[str, dict[str, Any]] = {}
    quality_joins: dict[str, Any] = {}
    for dataset_id in TRAIN_IDS:
        route = routes[dataset_id]
        truth = smoother_eval._read_truth(route["spec"]["truth"])
        route["truth_opened"] = True
        route["truth_hash"] = _sha256(route["spec"]["truth"])
        epochs = smoother._read_device_epochs(route["spec"]["device_gnss"], 1)
        baseline_positions = smoother._read_positions(route["baseline_position"], 18)
        candidate_positions = smoother._read_positions(route["position_path"], 18)
        if [row.timestamp_ms for row in baseline_positions] != epochs or [row.timestamp_ms for row in candidate_positions] != epochs:
            raise EvaluationError(f"candidate or baseline keys differ from device epochs: {dataset_id}")
        baseline_scores[dataset_id] = wls_eval._score(
            wls_eval._raw_rows(baseline_positions), baseline_positions, epochs, truth
        )
        candidate_scores[dataset_id] = wls_eval._score(
            wls_eval._raw_rows(candidate_positions), candidate_positions, epochs, truth
        )
        quality_joins[dataset_id] = {
            "baseline": _quality_error_join(route["quality_report_path"], baseline_positions, truth),
            "candidate": _quality_error_join(route["quality_report_path"], candidate_positions, truth),
        }
    baseline_aggregate = wls_eval._aggregate([baseline_scores[dataset_id] for dataset_id in TRAIN_IDS])
    candidate_aggregate = wls_eval._aggregate([candidate_scores[dataset_id] for dataset_id in TRAIN_IDS])
    route_gates = {
        dataset_id: _compare_route(candidate_scores[dataset_id], baseline_scores[dataset_id])
        for dataset_id in TRAIN_IDS
    }
    aggregate_gate = _aggregate_gate(candidate_aggregate, baseline_aggregate)
    train_passed = all(gate["passed"] for gate in route_gates.values()) and aggregate_gate["passed"]
    promotion = "promote-development-only-robust-spp-fallback" if train_passed else "no-go-train-gate"
    fgo = _bounded_fgo_feasibility(selection_path, output_dir) if not train_passed else None
    report = {
        "schema_version": SCHEMA_VERSION,
        "decision": "train-only-raw-quality-control-evaluation",
        "selection_record": {"path": str(selection_path), "sha256": _sha256(selection_path)},
        "selection_candidate": selection.get("candidate", {}),
        "truth_free_generation_completed_before_scoring": True,
        "truth_free_seal": {"path": str(seal_path), "sha256": seal_hash},
        "truth_access": {
            "train_truth_open_count": len(TRAIN_IDS),
            "fresh_validation_truth_open_count": 0,
            "future_holdout_truth_open_count": 0,
            "fresh_validation_materialized": False,
            "future_holdout_materialized": False,
            "holdout_content_opened": False,
            "holdout_ids_excluded": sorted(HOLDOUT_IDS),
            "public_private_scores_used_for_tuning": False,
        },
        "roles": {"train": list(TRAIN_IDS), "fresh_validation": FRESH_VALIDATION_ID, "future_holdout": FUTURE_HOLDOUT_ID},
        "algorithm": {
            "id": "innovation_huber_native_spp_v1",
            "parameters": quality.ROBUST_PARAMETERS,
            "fallback": "exact supplied handset WLS POS when robust output fails validation; no extrapolation",
        },
        "scores": {"baseline_wls_raw": baseline_scores, "candidate_robust_or_wls": candidate_scores},
        "aggregates": {"baseline_wls_raw": baseline_aggregate, "candidate_robust_or_wls": candidate_aggregate},
        "route_gates": route_gates,
        "aggregate_gate": aggregate_gate,
        "promotion_decision": promotion,
        "quality_error_tail_joins": quality_joins,
        "route_artifacts": {
            dataset_id: {
                "role": "candidate-train",
                "device_gnss": str(route["spec"]["device_gnss"]),
                "truth": str(route["spec"]["truth"]),
                "truth_sha256": route.get("truth_hash"),
                "candidate_manifest": _artifact(route["result"]["manifest_path"]),
                "candidate_position": _artifact(route["position_path"]),
                "quality_report": _artifact(route["quality_report_path"]),
                "baseline_position": _artifact(route["baseline_position"]),
                "source_counts": route["result"]["candidate"]["source_counts"],
                "performance": route["result"]["candidate"]["performance"],
            }
            for dataset_id, route in routes.items()
        },
        "bounded_fgo_alternative": fgo,
        "timing": {"total_wall_s": time.perf_counter() - started, "route_count": len(TRAIN_IDS)},
        "limitations": [
            "raw pseudorange-minus-WLS range is a clock-centered observable proxy, not a full receiver-clock innovation",
            "native robust weighting may be inactive when pre-QC rejection resolves all tails; that behavior is recorded, not tuned",
            "no fresh validation or holdout is opened when the train gate fails",
        ],
    }
    report_path = output_dir / "raw_quality_control_evaluation.json"
    _atomic_json(report_path, report)
    manifest = {
        "schema_version": MANIFEST_SCHEMA_VERSION,
        "selection_record": {"path": str(selection_path), "sha256": _sha256(selection_path)},
        "truth_free_seal": {"path": str(seal_path), "sha256": seal_hash},
        "report": _artifact(report_path),
        "truth_open_counts": {"train": len(TRAIN_IDS), "fresh_validation": 0, "future_holdout": 0},
        "holdout_content_opened": False,
        "routes": report["route_artifacts"],
    }
    manifest_path = output_dir / "raw_quality_control_evaluation_manifest.json"
    _atomic_json(manifest_path, manifest)
    print(f"Raw quality control evaluation complete: {report_path}")
    print(f"Promotion: {promotion}")
    return 0


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        prog=os.environ.get("GNSS_CLI_NAME", "gnss smartphone-raw-quality-control-eval")
    )
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    parser.add_argument("--selection-record", type=Path, default=SELECTION_RECORD)
    parser.add_argument("--role", choices=("train",), default="train")
    return parser.parse_args(argv)


def run(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    try:
        return run_train(args.output_dir, args.selection_record)
    except (EvaluationError, quality.QualityControlError, smoother.SmootherError, ValueError) as exc:
        print(f"Raw quality control evaluation failed: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(run())
