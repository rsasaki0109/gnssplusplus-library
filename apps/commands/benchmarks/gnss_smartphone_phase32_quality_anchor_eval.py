#!/usr/bin/env python3
"""Score the sealed Phase31 anchor lane against the sealed Phase28 control.

This is a development-only recovery evaluator.  It never runs the solver and
does not materialize data.  Prediction artifacts are verified first, then one
process reads each already-materialized development truth file exactly once
and scores control and candidate on their common truth keys.  The four
horizontal diagnostic variants are authoritative for this phase; the native
four-column output has no height, so vertical P95 is recorded as informational
and never used as a gate.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import sys
import tempfile
import time
from typing import Any


COMMANDS_DIR = Path(__file__).resolve().parents[1]
if str(COMMANDS_DIR) not in sys.path:
    sys.path.insert(0, str(COMMANDS_DIR))
from support.gnss_runtime import application_root  # noqa: E402

ROOT = application_root(__file__)
BENCHMARK_DIR = ROOT / "apps" / "commands" / "benchmarks"
if str(BENCHMARK_DIR) not in sys.path:
    sys.path.insert(0, str(BENCHMARK_DIR))
import gnss_smartphone_phase29_train_eval as phase29  # noqa: E402


SCHEMA = "smartphone-r5-phase32-quality-anchor-train-evaluation.v1"
MANIFEST_SCHEMA = "smartphone-r5-phase32-quality-anchor-train-evaluation-manifest.v1"
FREEZE_SCHEMA = "smartphone-r5-phase32-quality-anchor-train-evaluation-freeze.v1"
FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_phase32_quality_anchor_train_eval_freeze_v1.json"
DEFAULT_OUTPUT = ROOT / "output/smartphone-r5/phase32-quality-anchor-train-eval-v1"
PHASE29_MATERIALIZATION = ROOT / "output/smartphone-r5/phase29-no-bridge-train-eval-v1/truth_materialization.json"
PHASE29_MATERIALIZATION_SHA256 = "dcc474f928f9295daa36913f458157f00fe5f088cfcc2397387a0f06555346e5"
TOLERANCE = 1e-12

ROUTES = (
    "2021-03-16-18-59-us-ca-mtv-a/pixel5",
    "2021-07-27-19-49-us-ca-mtv-b/pixel4",
    "2021-07-14-20-50-us-ca-mtv-e/sm-g988b",
)
DIAGNOSTIC_KEYS = tuple(
    f"{distance}__{percentile}"
    for distance in phase29.kaggle.DISTANCE_VARIANT_IDS
    for percentile in phase29.kaggle.PERCENTILE_VARIANT_IDS
)


class Phase32Error(ValueError):
    """Raised when the sealed Phase32 evaluation contract is not provable."""


def sha256(path: Path) -> str:
    if not path.is_file():
        raise Phase32Error(f"missing file: {path}")
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def load_json(path: Path, label: str) -> dict[str, Any]:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise Phase32Error(f"invalid {label}: {path}") from exc
    if not isinstance(payload, dict):
        raise Phase32Error(f"{label} must be an object: {path}")
    return payload


def atomic_json(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".tmp", dir=str(path.parent)
    )
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8", newline="") as handle:
            descriptor = -1
            json.dump(payload, handle, indent=2, sort_keys=True)
            handle.write("\n")
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, path)
        temporary = ""
    finally:
        if descriptor >= 0:
            os.close(descriptor)
        if temporary:
            try:
                os.unlink(temporary)
            except FileNotFoundError:
                pass


def _verify_freeze() -> dict[str, Any]:
    freeze = load_json(FREEZE, "Phase32 freeze")
    if freeze.get("schema_version") != FREEZE_SCHEMA:
        raise Phase32Error("Phase32 freeze schema mismatch")
    if freeze.get("status") != "frozen-before-development-truth-reread":
        raise Phase32Error("Phase32 freeze is not pre-truth")
    policy = freeze.get("truth_access")
    if not isinstance(policy, dict):
        raise Phase32Error("Phase32 truth policy missing")
    for key, expected in (
        ("phase32_truth_reads_before_freeze", 0),
        ("validation_truth_open_count_before_freeze", 0),
        ("holdout_truth_open_count_before_freeze", 0),
        ("mat_read_or_generated", False),
        ("token_access", False),
        ("post_score_tuning", False),
    ):
        if policy.get(key) != expected:
            raise Phase32Error(f"Phase32 truth policy mismatch: {key}")
    if freeze.get("candidate_control_mutation") is not False or freeze.get("solver_rerun") is not False:
        raise Phase32Error("Phase32 permits candidate/control mutation")
    evaluator = freeze.get("evaluator_contract")
    if not isinstance(evaluator, dict) or evaluator.get("vertical_gate") != "informational-unavailable":
        raise Phase32Error("Phase32 evaluator contract mismatch")
    routes = freeze.get("routes")
    if not isinstance(routes, dict) or tuple(routes) != ROUTES:
        raise Phase32Error("Phase32 route order mismatch")
    for dataset_id in ROUTES:
        row = routes[dataset_id]
        if not isinstance(row, dict):
            raise Phase32Error(f"Phase32 route row missing: {dataset_id}")
        for lane in ("control", "candidate"):
            artifact = row.get(lane)
            if not isinstance(artifact, dict):
                raise Phase32Error(f"Phase32 lane missing: {dataset_id}/{lane}")
            for name in ("submission", "summary"):
                path = ROOT / str(artifact.get(name, ""))
                if path.suffix.lower() == ".mat":
                    raise Phase32Error("MAT prediction is forbidden")
                if sha256(path) != artifact.get(f"{name}_sha256"):
                    raise Phase32Error(f"sealed {lane} {name} hash mismatch: {dataset_id}")
    source_hashes = freeze.get("source_and_binary_hashes")
    if not isinstance(source_hashes, dict):
        raise Phase32Error("Phase32 source pins missing")
    for relative_path, expected in source_hashes.items():
        if sha256(ROOT / relative_path) != expected:
            raise Phase32Error(f"Phase32 source pin mismatch: {relative_path}")
    materialization = freeze.get("truth_materialization")
    if not isinstance(materialization, dict):
        raise Phase32Error("Phase32 truth metadata missing")
    if materialization.get("path") != str(PHASE29_MATERIALIZATION.relative_to(ROOT)):
        raise Phase32Error("Phase32 truth materialization path mismatch")
    if sha256(PHASE29_MATERIALIZATION) != PHASE29_MATERIALIZATION_SHA256:
        raise Phase32Error("Phase29 truth materialization metadata changed")
    return freeze


def _verify_predictions(freeze: dict[str, Any]) -> dict[str, dict[str, list[phase29.kaggle.CoordinateRow]]]:
    verified: dict[str, dict[str, list[phase29.kaggle.CoordinateRow]]] = {}
    for dataset_id in ROUTES:
        row = freeze["routes"][dataset_id]
        verified[dataset_id] = {}
        for lane in ("control", "candidate"):
            artifact = row[lane]
            prediction_path = ROOT / artifact["submission"]
            prediction_rows = phase29._prediction_rows(prediction_path, dataset_id)
            if len(prediction_rows) != int(artifact["prediction_rows"]):
                raise Phase32Error(f"prediction row count changed: {dataset_id}/{lane}")
            verified[dataset_id][lane] = prediction_rows
    return verified


def _horizontal_compare(candidate: dict[str, Any], control: dict[str, Any]) -> dict[str, Any]:
    failures: list[str] = []
    for key in DIAGNOSTIC_KEYS:
        if candidate["kaggle_diagnostic_score_variants_m"][key] > control["kaggle_diagnostic_score_variants_m"][key] + TOLERANCE:
            failures.append(f"{key}_regression")
    for path, label in (
        (("horizontal_wgs84_m", "p50_m"), "h_p50_regression"),
        (("horizontal_wgs84_m", "p95_m"), "h_p95_regression"),
    ):
        current: Any = candidate
        reference: Any = control
        for key in path:
            current = current[key]
            reference = reference[key]
        if current > reference + TOLERANCE:
            failures.append(label)
    for key in ("availability_ratio", "truth_coverage_ratio"):
        if candidate[key] + TOLERANCE < control[key]:
            failures.append(f"{key}_regression")
    if candidate["continuity"]["over_70_mps_count"] > control["continuity"]["over_70_mps_count"]:
        failures.append("continuity_regression")
    return {
        "passed": not failures,
        "failures": failures,
        "vertical": {
            "status": "informational-unavailable-four-column-prediction",
            "candidate_p95_abs_m": None,
            "control_p95_abs_m": None,
        },
    }


def _aggregate(route_metrics: list[dict[str, Any]]) -> dict[str, Any]:
    if not route_metrics:
        raise Phase32Error("no route metrics")
    return {
        "route_count": len(route_metrics),
        "mean_availability_ratio": sum(row["availability_ratio"] for row in route_metrics) / len(route_metrics),
        "mean_truth_coverage_ratio": sum(row["truth_coverage_ratio"] for row in route_metrics) / len(route_metrics),
        "mean_horizontal_wgs84_p50_m": sum(row["horizontal_wgs84_m"]["p50_m"] for row in route_metrics) / len(route_metrics),
        "mean_horizontal_wgs84_p95_m": sum(row["horizontal_wgs84_m"]["p95_m"] for row in route_metrics) / len(route_metrics),
        "mean_kaggle_diagnostic_score_variants_m": {
            key: sum(row["kaggle_diagnostic_score_variants_m"][key] for row in route_metrics) / len(route_metrics)
            for key in DIAGNOSTIC_KEYS
        },
        "sum_over_70_mps_count": sum(row["continuity"]["over_70_mps_count"] for row in route_metrics),
    }


def _aggregate_compare(candidate: dict[str, Any], control: dict[str, Any]) -> dict[str, Any]:
    failures: list[str] = []
    for key in DIAGNOSTIC_KEYS:
        if candidate["mean_kaggle_diagnostic_score_variants_m"][key] >= control["mean_kaggle_diagnostic_score_variants_m"][key] - TOLERANCE:
            failures.append(f"{key}_not_strictly_improved")
    for key in ("mean_availability_ratio", "mean_truth_coverage_ratio"):
        if candidate[key] + TOLERANCE < control[key]:
            failures.append(f"{key}_regression")
    for key, label in (
        ("mean_horizontal_wgs84_p50_m", "h_p50_regression"),
        ("mean_horizontal_wgs84_p95_m", "h_p95_regression"),
    ):
        if candidate[key] > control[key] + TOLERANCE:
            failures.append(label)
    if candidate["sum_over_70_mps_count"] > control["sum_over_70_mps_count"]:
        failures.append("continuity_regression")
    return {"passed": not failures, "failures": failures}


def score(output_root: Path) -> dict[str, Any]:
    started = time.perf_counter()
    freeze = _verify_freeze()
    predictions = _verify_predictions(freeze)
    materialization = load_json(PHASE29_MATERIALIZATION, "Phase29 truth materialization")
    if materialization.get("truth_parse_count") != 0 or materialization.get("validation_truth_materialized") is not False or materialization.get("future_holdout_truth_materialized") is not False:
        raise Phase32Error("truth materialization is not sealed for this recovery")

    route_results: dict[str, Any] = {}
    control_rows: list[dict[str, Any]] = []
    candidate_rows: list[dict[str, Any]] = []
    truth_reads = 0
    for dataset_id in ROUTES:
        truth_info = freeze["truth_materialization"]["routes"][dataset_id]
        truth_path = ROOT / truth_info["path"]
        truth, truth_hash, truth_bytes = phase29._read_truth_once(truth_path, dataset_id)
        truth_reads += 1
        if truth_hash != truth_info["sha256"] or truth_bytes != int(truth_info["bytes"]):
            raise Phase32Error(f"truth changed since sealed materialization: {dataset_id}")
        truth_keys = set(truth)
        control = predictions[dataset_id]["control"]
        candidate = predictions[dataset_id]["candidate"]
        control_keys = {(row.phone, row.timestamp) for row in control}
        candidate_keys = {(row.phone, row.timestamp) for row in candidate}
        control_matched = control_keys & truth_keys
        candidate_matched = candidate_keys & truth_keys
        shared = control_matched & candidate_matched
        if shared != control_matched or shared != candidate_matched:
            raise Phase32Error(f"control/candidate truth-matched key sets differ: {dataset_id}")
        control_metrics = phase29._lane_metrics(control, truth, shared)
        candidate_metrics = phase29._lane_metrics(candidate, truth, shared)
        route_gate = _horizontal_compare(candidate_metrics, control_metrics)
        route_results[dataset_id] = {
            "truth": {
                "path": truth_info["path"],
                "sha256": truth_hash,
                "bytes": truth_bytes,
                "read_count": 1,
            },
            "key_set": {
                "truth_keys": len(truth_keys),
                "control_prediction_keys": len(control_keys),
                "candidate_prediction_keys": len(candidate_keys),
                "control_truth_matched_keys": len(control_matched),
                "candidate_truth_matched_keys": len(candidate_matched),
                "shared_scored_keys": len(shared),
                "same_matched_key_set": True,
            },
            "control": control_metrics,
            "candidate": candidate_metrics,
            "gate": route_gate,
            "artifact_hashes": {
                "control_submission_sha256": sha256(ROOT / freeze["routes"][dataset_id]["control"]["submission"]),
                "control_summary_sha256": sha256(ROOT / freeze["routes"][dataset_id]["control"]["summary"]),
                "candidate_submission_sha256": sha256(ROOT / freeze["routes"][dataset_id]["candidate"]["submission"]),
                "candidate_summary_sha256": sha256(ROOT / freeze["routes"][dataset_id]["candidate"]["summary"]),
            },
        }
        control_rows.append(control_metrics)
        candidate_rows.append(candidate_metrics)
    control_aggregate = _aggregate(control_rows)
    candidate_aggregate = _aggregate(candidate_rows)
    aggregate_gate = _aggregate_compare(candidate_aggregate, control_aggregate)
    route_gate = all(row["gate"]["passed"] for row in route_results.values())
    train_passed = route_gate and aggregate_gate["passed"]
    report = {
        "schema_version": SCHEMA,
        "phase": 32,
        "status": "train-pass" if train_passed else "no-go-train-gate",
        "decision": "development-go-fresh-validation-eligible" if train_passed else "no-go-no-validation",
        "candidate_id": "native_fgo_raw_quality_anchor_spp_replay_v1",
        "control_id": "phase28_tdcp_no_bridge_sealed_control",
        "truth_free_artifacts_verified_before_truth": True,
        "truth_open_count": truth_reads,
        "truth_read_count_per_route": 1,
        "cumulative_truth_reads_after_phase32": {
            "pixel5": 2,
            "pixel4": 2,
            "sm-g988b": 3,
        },
        "fresh_validation_truth_open_count": 0,
        "future_holdout_truth_open_count": 0,
        "vertical_policy": "informational-unavailable-four-column-prediction; not a gate",
        "routes": route_results,
        "aggregate": {"control": control_aggregate, "candidate": candidate_aggregate, "gate": aggregate_gate},
        "train_gate": {
            "routewise_horizontal_nonregression": route_gate,
            "aggregate_all_four_strict_improvement": aggregate_gate["passed"],
            "passed": train_passed,
        },
        "validation_policy": {
            "opened": False,
            "fresh_validation": "2023-05-09-21-32-us-ca-mtv-pe1/pixel5",
            "future_holdout": "2023-05-16-19-54-us-ca-mtv-xe1/pixel5",
        },
        "policy": {
            "post_score_tuning": False,
            "kaggle_or_token_access": False,
            "mat_read_or_generated": False,
            "test_truth_read": False,
            "solver_rerun": False,
        },
        "runtime": {"wall_seconds": time.perf_counter() - started},
    }
    report_path = output_root / "train_evaluation.json"
    atomic_json(report_path, report)
    manifest = {
        "schema_version": MANIFEST_SCHEMA,
        "status": report["status"],
        "report": {"path": str(report_path.relative_to(ROOT)), "sha256": sha256(report_path)},
        "freeze_record": {"path": str(FREEZE.relative_to(ROOT)), "sha256": sha256(FREEZE)},
        "truth_materialization": {"path": str(PHASE29_MATERIALIZATION.relative_to(ROOT)), "sha256": sha256(PHASE29_MATERIALIZATION)},
        "truth_open_count": truth_reads,
        "truth_read_count_per_route": 1,
        "fresh_validation_truth_open_count": 0,
        "future_holdout_truth_open_count": 0,
        "vertical_policy": "informational-unavailable-four-column-prediction; not a gate",
        "post_score_tuning": False,
        "mat_read_or_generated": False,
        "atomic_publish": True,
    }
    atomic_json(output_root / "train_evaluation.manifest.json", manifest)
    return report


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("operation", choices=("score",))
    parser.add_argument("--output-root", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args(argv)
    try:
        report = score(args.output_root)
    except (OSError, Phase32Error, ValueError) as exc:
        print(f"phase32: {exc}", file=sys.stderr)
        return 2
    print(json.dumps({"status": report["status"], "truth_open_count": report["truth_open_count"], "routes": len(report["routes"])}, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
