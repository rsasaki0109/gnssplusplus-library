#!/usr/bin/env python3
"""Evaluate the sealed Phase35 raw-only feature matrix in one process.

The native solver has already run and been structurally sealed by
``gnss_smartphone_phase35_matrix.py``.  This command never runs the solver and
never changes a candidate.  After verifying that seal, it reads each of the
three already-materialized development truth CSVs exactly once, scores the
control and all three frozen lanes in memory, and selects at most one lane.
Only when the unchanged train route/macro gate passes does the same process
read the already-materialized Phase34 validation truth once.  The future
holdout and Kaggle are never opened here.
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
import gnss_smartphone_phase31_quality_anchor_structural as phase31  # noqa: E402
import gnss_smartphone_phase35_matrix as phase35  # noqa: E402


SCHEMA = "smartphone-r5-phase35-matrix-evaluation.v1"
MANIFEST_SCHEMA = "smartphone-r5-phase35-matrix-evaluation-manifest.v1"
FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_phase35_matrix_freeze_v1.json"
FREEZE_SHA256 = "9be22fd18dd3f818a8c902dafde2aa7b5828ded13df98b138682863e8f14a92a"
EVALUATOR_MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase35_matrix_evaluator_manifest_v2.json"
EVALUATOR_MANIFEST_SHA256 = "1c72af0f616231241978c447a296c3bb67e57e19b220de680cd72ace0a91bc86"
RECOVERY_RECORD = ROOT / "docs/use_cases/records/smartphone_r5_phase35_interruption_recovery_v1.json"
RECOVERY_RECORD_SHA256 = "9fbb44ed3142073d2640a76dbab55d53943293cb03ddf25fc7d5571db1d0c925"
STRUCTURAL_ROOT = ROOT / "output/smartphone-r5/phase35-matrix-v1"
STRUCTURAL_SEAL = STRUCTURAL_ROOT / "truth_free_seal.json"
STRUCTURAL_MATRIX = STRUCTURAL_ROOT / "truth_free_matrix.json"
TRAIN_MATERIALIZATION = ROOT / "output/smartphone-r5/phase29-no-bridge-train-eval-v1/truth_materialization.json"
TRAIN_MATERIALIZATION_SHA256 = "dcc474f928f9295daa36913f458157f00fe5f088cfcc2397387a0f06555346e5"
PHASE34_FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_phase34_quality_anchor_validation_freeze_v1.json"
PHASE34_FREEZE_SHA256 = "18fd075f30699d0213a1cca3a9eac25d24d3765d75df8d770380dfa7fbeeb43b"
PHASE34_RESULT = ROOT / "output/smartphone-r5/phase34-quality-anchor-validation-v1/validation_evaluation.json"
PHASE34_RESULT_MANIFEST = ROOT / "output/smartphone-r5/phase34-quality-anchor-validation-v1/validation_evaluation.manifest.json"
PHASE34_TRUTH = ROOT / "output/smartphone-r5/phase34-quality-anchor-validation-v1-io-failure/truth/2023-05-09-21-32-us-ca-mtv-pe1/pixel5/ground_truth.csv"
PHASE34_TRUTH_SHA256 = "ef29d4712204bdf51b543c8e3fba28c639d759c4b21f8438e407ba555f920967"
DEFAULT_OUTPUT = ROOT / "output/smartphone-r5/phase35-matrix-eval-v1"
TRAIN_ROUTES = tuple(spec["dataset_id"] for spec in phase35.ROUTES[:3])
VALIDATION_ROUTE = phase35.ROUTES[3]["dataset_id"]
ALL_ROUTES = tuple(spec["dataset_id"] for spec in phase35.ROUTES)
LANES = tuple(phase35.LANE_NAMES)
CANDIDATES = LANES[1:]
DIAGNOSTIC_KEYS = tuple(phase29.DIAGNOSTIC_KEYS)
TOLERANCE = 1e-12


class Phase35EvaluationError(ValueError):
    """Raised when the sealed Phase35 evaluation contract is not provable."""


def sha256(path: Path) -> str:
    if not path.is_file():
        raise Phase35EvaluationError(f"missing file: {path}")
    digest = hashlib.sha256()
    try:
        with path.open("rb") as handle:
            for chunk in iter(lambda: handle.read(1024 * 1024), b""):
                digest.update(chunk)
    except OSError as exc:
        raise Phase35EvaluationError(f"failed to hash {path}: {exc}") from exc
    return digest.hexdigest()


def load_json(path: Path, label: str) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise Phase35EvaluationError(f"invalid {label}: {path}") from exc
    if not isinstance(value, dict):
        raise Phase35EvaluationError(f"{label} must be an object: {path}")
    return value


def atomic_json(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary = tempfile.mkstemp(prefix=f".{path.name}.", suffix=".tmp", dir=str(path.parent))
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


def _key_digest(keys: set[tuple[str, int]]) -> str:
    canonical = json.dumps(
        [[phone, timestamp] for phone, timestamp in sorted(keys)],
        ensure_ascii=True,
        separators=(",", ":"),
    ).encode("utf-8")
    return hashlib.sha256(canonical).hexdigest()


def _verify_evaluator_manifest() -> dict[str, Any]:
    if sha256(EVALUATOR_MANIFEST) != EVALUATOR_MANIFEST_SHA256:
        raise Phase35EvaluationError("Phase35 evaluator manifest hash changed")
    manifest = load_json(EVALUATOR_MANIFEST, "Phase35 evaluator manifest")
    if manifest.get("schema_version") != "smartphone-r5-phase35-matrix-evaluator-manifest.v2":
        raise Phase35EvaluationError("Phase35 evaluator manifest schema mismatch")
    if manifest.get("status") != "evaluator-frozen-after-truth-free-resume-before-truth-evaluation":
        raise Phase35EvaluationError("Phase35 evaluator manifest is not pre-truth")
    freeze = manifest.get("freeze")
    if not isinstance(freeze, dict) or freeze.get("path") != str(FREEZE.relative_to(ROOT)) or freeze.get("sha256") != FREEZE_SHA256:
        raise Phase35EvaluationError("Phase35 evaluator freeze pin mismatch")
    runner = manifest.get("runner")
    if not isinstance(runner, dict):
        raise Phase35EvaluationError("Phase35 runner pin missing")
    runner_path = ROOT / str(runner.get("source", ""))
    test_path = ROOT / str(runner.get("test", ""))
    if sha256(runner_path) != runner.get("sha256") or sha256(test_path) != runner.get("test_sha256"):
        raise Phase35EvaluationError("Phase35 runner/test pin mismatch")
    if runner.get("resume_operation") != "resume":
        raise Phase35EvaluationError("Phase35 resume operation is not pinned")
    recovery = manifest.get("recovery")
    if not isinstance(recovery, dict) or recovery.get("record") != str(RECOVERY_RECORD.relative_to(ROOT)) or recovery.get("record_sha256") != RECOVERY_RECORD_SHA256:
        raise Phase35EvaluationError("Phase35 recovery pin mismatch")
    if sha256(RECOVERY_RECORD) != RECOVERY_RECORD_SHA256:
        raise Phase35EvaluationError("Phase35 recovery record changed")
    if recovery.get("truth_open_count_before_structural_seal") != 0 or recovery.get("mat_read_or_generated_before_structural_seal") is not False:
        raise Phase35EvaluationError("Phase35 recovery truth policy mismatch")
    if manifest.get("truth_policy", {}).get("mat_read_or_generated") is not False:
        raise Phase35EvaluationError("Phase35 evaluator permits MAT")
    return manifest


def _verify_phase35_freeze() -> dict[str, Any]:
    if sha256(FREEZE) != FREEZE_SHA256:
        raise Phase35EvaluationError("Phase35 freeze hash changed")
    try:
        freeze = phase35.verify_freeze()
    except (OSError, ValueError) as exc:
        raise Phase35EvaluationError(f"Phase35 freeze verification failed: {exc}") from exc
    return freeze


def _artifact_path(item: Any, field: str) -> tuple[Path, str]:
    if not isinstance(item, dict):
        raise Phase35EvaluationError(f"missing structural artifact metadata: {field}")
    path = item.get("path")
    expected = item.get("sha256")
    if not isinstance(path, str) or not isinstance(expected, str):
        raise Phase35EvaluationError(f"invalid structural artifact metadata: {field}")
    absolute = ROOT / path
    if absolute.suffix.lower() == ".mat":
        raise Phase35EvaluationError(f"MAT prediction is forbidden: {absolute}")
    if sha256(absolute) != expected:
        raise Phase35EvaluationError(f"structural artifact hash mismatch: {field}")
    return absolute, expected


def _verify_structural_seal() -> dict[str, Any]:
    _verify_phase35_freeze()
    matrix = load_json(STRUCTURAL_MATRIX, "Phase35 truth-free matrix")
    if matrix.get("schema_version") != phase35.RUN_SCHEMA or matrix.get("status") != "truth-free-complete" or matrix.get("truth_open_count") != 0 or matrix.get("mat_read_or_generated") is not False:
        raise Phase35EvaluationError("Phase35 truth-free matrix is not complete and truth-free")
    seal = load_json(STRUCTURAL_SEAL, "Phase35 truth-free seal")
    if seal.get("schema_version") != phase35.SEAL_SCHEMA or seal.get("status") != "sealed-truth-free-structural-pass":
        raise Phase35EvaluationError("Phase35 structural seal is not a pass")
    gate = seal.get("structural_gate")
    if not isinstance(gate, dict) or gate.get("passed") is not True or gate.get("truth_open_count") != 0 or gate.get("mat_read_or_generated") is not False:
        raise Phase35EvaluationError("Phase35 structural gate failed")
    if seal.get("solver_rerun_after_truth") is not False:
        raise Phase35EvaluationError("Phase35 seal permits post-truth solver rerun")
    verified: dict[str, Any] = {"seal": seal, "routes": {}}
    for spec in phase35.ROUTES:
        dataset_id = str(spec["dataset_id"])
        route_report = seal.get("routes", {}).get(dataset_id)
        if not isinstance(route_report, dict):
            raise Phase35EvaluationError(f"missing sealed route: {dataset_id}")
        target_keys = phase35._target_keys(phase35._input_paths(spec))
        lanes: dict[str, Any] = {}
        for lane in LANES:
            item = route_report.get(lane)
            if not isinstance(item, dict) or item.get("status") != "sealed" or item.get("repeat_byte_identical") is not True:
                raise Phase35EvaluationError(f"unsealed Phase35 lane: {dataset_id}/{lane}")
            submission, submission_hash = _artifact_path(item.get("submission"), f"{dataset_id}/{lane}/submission")
            summary, summary_hash = _artifact_path(item.get("summary"), f"{dataset_id}/{lane}/summary")
            repeat_submission, repeat_submission_hash = _artifact_path(
                {"path": str(submission.parent.parent / "run2" / "submission.csv"), "sha256": item.get("submission", {}).get("sha256")},
                f"{dataset_id}/{lane}/repeat-submission",
            )
            repeat_summary, repeat_summary_hash = _artifact_path(
                {"path": str(summary.parent.parent / "run2" / "summary.json"), "sha256": item.get("summary", {}).get("sha256")},
                f"{dataset_id}/{lane}/repeat-summary",
            )
            if repeat_submission_hash != submission_hash or repeat_summary_hash != summary_hash:
                raise Phase35EvaluationError(f"repeat artifact differs: {dataset_id}/{lane}")
            rows = phase29._prediction_rows(submission, dataset_id)
            repeat_rows = phase29._prediction_rows(repeat_submission, dataset_id)
            if [row.timestamp for row in rows] != target_keys or [row.timestamp for row in repeat_rows] != target_keys:
                raise Phase35EvaluationError(f"exact raw target keys failed: {dataset_id}/{lane}")
            speed = phase31.speed_report(rows)
            if not speed.get("finite") or speed.get("over_70_mps_count") != 0:
                raise Phase35EvaluationError(f"structural continuity failed: {dataset_id}/{lane}")
            summary_payload = load_json(summary, f"{dataset_id}/{lane} summary")
            if summary_payload.get("truth_used") is not False or summary_payload.get("production_default_changed") is not False:
                raise Phase35EvaluationError(f"truth/default contract failed: {dataset_id}/{lane}")
            graph = summary_payload.get("graph")
            if not isinstance(graph, dict) or graph.get("converged") is not True:
                raise Phase35EvaluationError(f"graph is not converged: {dataset_id}/{lane}")
            lanes[lane] = {
                "submission": {"path": str(submission.relative_to(ROOT)), "sha256": submission_hash, "rows": len(rows)},
                "summary": {"path": str(summary.relative_to(ROOT)), "sha256": summary_hash},
                "repeat_submission_sha256": repeat_submission_hash,
                "repeat_summary_sha256": repeat_summary_hash,
                "speed": speed,
                "target_epoch_count": len(target_keys),
                "rows": rows,
            }
        verified["routes"][dataset_id] = lanes
    return verified


def _load_train_truth_sources() -> dict[str, dict[str, Any]]:
    if sha256(TRAIN_MATERIALIZATION) != TRAIN_MATERIALIZATION_SHA256:
        raise Phase35EvaluationError("train truth materialization metadata changed")
    materialization = load_json(TRAIN_MATERIALIZATION, "train truth materialization")
    if materialization.get("truth_parse_count") != 0 or materialization.get("mat_member_opened") is not False:
        raise Phase35EvaluationError("train truth was parsed before Phase35 evaluation")
    sources: dict[str, dict[str, Any]] = {}
    for dataset_id in TRAIN_ROUTES:
        metadata = materialization.get("routes", {}).get(dataset_id)
        if not isinstance(metadata, dict):
            raise Phase35EvaluationError(f"missing train truth metadata: {dataset_id}")
        path = ROOT / str(metadata.get("path", ""))
        if path.suffix.lower() != ".csv":
            raise Phase35EvaluationError(f"train truth is not CSV: {dataset_id}")
        sources[dataset_id] = {
            "path": path,
            "expected_sha256": metadata.get("sha256"),
            "expected_bytes": metadata.get("bytes", metadata.get("file_size")),
        }
    return sources


def _load_validation_truth_source() -> dict[str, Any]:
    if sha256(PHASE34_FREEZE) != PHASE34_FREEZE_SHA256:
        raise Phase35EvaluationError("Phase34 validation freeze changed")
    freeze = load_json(PHASE34_FREEZE, "Phase34 validation freeze")
    if freeze.get("truth_access", {}).get("future_holdout_truth_open_count") != 0 or freeze.get("truth_access", {}).get("mat_read_or_generated") is not False:
        raise Phase35EvaluationError("Phase34 validation truth policy changed")
    result_manifest = load_json(PHASE34_RESULT_MANIFEST, "Phase34 validation result manifest")
    result_info = result_manifest.get("result")
    if not isinstance(result_info, dict) or result_info.get("path") != str(PHASE34_RESULT.relative_to(ROOT)) or sha256(PHASE34_RESULT) != result_info.get("sha256"):
        raise Phase35EvaluationError("Phase34 validation result pin mismatch")
    prior = load_json(PHASE34_RESULT, "Phase34 validation result")
    truth = prior.get("truth")
    if not isinstance(truth, dict) or truth.get("path") != str(PHASE34_TRUTH.relative_to(ROOT)) or truth.get("sha256") != PHASE34_TRUTH_SHA256 or truth.get("read_count") != 1:
        raise Phase35EvaluationError("Phase34 validation truth pin mismatch")
    return {"path": PHASE34_TRUTH, "expected_sha256": PHASE34_TRUTH_SHA256, "expected_bytes": truth.get("bytes")}


def _route_metrics(rows_by_lane: dict[str, list[Any]], truth: dict[tuple[str, int], tuple[float, float, float]], dataset_id: str) -> tuple[dict[str, Any], dict[str, Any]]:
    truth_keys = set(truth)
    prediction_keys = {lane: {(row.phone, row.timestamp) for row in rows} for lane, rows in rows_by_lane.items()}
    matched = {lane: keys & truth_keys for lane, keys in prediction_keys.items()}
    common = set.intersection(*matched.values()) if matched else set()
    key_sets_equal = bool(matched) and all(keys == matched[LANES[0]] for keys in matched.values())
    metrics: dict[str, Any] = {}
    for lane, rows in rows_by_lane.items():
        metrics[lane] = phase29._lane_metrics(rows, truth, common)
        metrics[lane]["key_set"] = {
            "prediction_keys": len(prediction_keys[lane]),
            "truth_matched_keys": len(matched[lane]),
            "extra_prediction_keys": len(prediction_keys[lane] - truth_keys),
            "missing_truth_keys": len(truth_keys - prediction_keys[lane]),
            "matched_key_set_sha256": _key_digest(matched[lane]),
        }
    key_report = {
        "truth_keys": len(truth_keys),
        "common_scored_keys": len(common),
        "common_key_set_sha256": _key_digest(common),
        "same_matched_key_set_all_lanes": key_sets_equal,
        "coverage": len(common) / len(truth_keys) if truth_keys else 0.0,
        "dataset_id": dataset_id,
    }
    return metrics, key_report


def _route_gate(candidate: dict[str, Any], control: dict[str, Any], key_report: dict[str, Any]) -> dict[str, Any]:
    failures: list[str] = []
    if not key_report["same_matched_key_set_all_lanes"]:
        failures.append("matched_key_set_mismatch")
    for key in DIAGNOSTIC_KEYS:
        if candidate["kaggle_diagnostic_score_variants_m"][key] > control["kaggle_diagnostic_score_variants_m"][key] + TOLERANCE:
            failures.append(f"{key}_regression")
    for distance in ("horizontal_wgs84_m", "horizontal_haversine_m"):
        for percentile in ("p50_m", "p95_m"):
            if candidate[distance][percentile] > control[distance][percentile] + TOLERANCE:
                failures.append(f"{distance}_{percentile}_regression")
    for key in ("availability_ratio", "truth_coverage_ratio"):
        if candidate[key] + TOLERANCE < control[key]:
            failures.append(f"{key}_regression")
    if candidate["continuity"]["over_70_mps_count"] > control["continuity"]["over_70_mps_count"]:
        failures.append("continuity_regression")
    return {"passed": not failures, "failures": failures}


def _aggregate(route_metrics: list[dict[str, Any]]) -> dict[str, Any]:
    if not route_metrics:
        raise Phase35EvaluationError("cannot aggregate no routes")
    return {
        "route_count": len(route_metrics),
        "mean_availability_ratio": sum(row["availability_ratio"] for row in route_metrics) / len(route_metrics),
        "mean_truth_coverage_ratio": sum(row["truth_coverage_ratio"] for row in route_metrics) / len(route_metrics),
        "mean_horizontal_wgs84_p50_m": sum(row["horizontal_wgs84_m"]["p50_m"] for row in route_metrics) / len(route_metrics),
        "mean_horizontal_wgs84_p95_m": sum(row["horizontal_wgs84_m"]["p95_m"] for row in route_metrics) / len(route_metrics),
        "mean_horizontal_haversine_p50_m": sum(row["horizontal_haversine_m"]["p50_m"] for row in route_metrics) / len(route_metrics),
        "mean_horizontal_haversine_p95_m": sum(row["horizontal_haversine_m"]["p95_m"] for row in route_metrics) / len(route_metrics),
        "mean_kaggle_diagnostic_score_variants_m": {
            key: sum(row["kaggle_diagnostic_score_variants_m"][key] for row in route_metrics) / len(route_metrics)
            for key in DIAGNOSTIC_KEYS
        },
        "mean_kaggle_diagnostic_mean_m": sum(row["kaggle_diagnostic_mean_m"] for row in route_metrics) / len(route_metrics),
        "sum_over_70_mps_count": sum(row["continuity"]["over_70_mps_count"] for row in route_metrics),
    }


def _macro_gate(candidate: dict[str, Any], control: dict[str, Any]) -> dict[str, Any]:
    failures: list[str] = []
    strict: list[str] = []
    for key in DIAGNOSTIC_KEYS:
        current = candidate["mean_kaggle_diagnostic_score_variants_m"][key]
        reference = control["mean_kaggle_diagnostic_score_variants_m"][key]
        if current >= reference - TOLERANCE:
            failures.append(f"{key}_not_strictly_improved")
        else:
            strict.append(key)
    for key in (
        "mean_horizontal_wgs84_p50_m",
        "mean_horizontal_wgs84_p95_m",
        "mean_horizontal_haversine_p50_m",
        "mean_horizontal_haversine_p95_m",
    ):
        if candidate[key] > control[key] + TOLERANCE:
            failures.append(f"{key}_regression")
    for key in ("mean_availability_ratio", "mean_truth_coverage_ratio"):
        if candidate[key] + TOLERANCE < control[key]:
            failures.append(f"{key}_regression")
    if candidate["sum_over_70_mps_count"] > control["sum_over_70_mps_count"]:
        failures.append("continuity_regression")
    return {"passed": not failures, "failures": failures, "strictly_improved_variants": strict}


def _validation_gate(candidate: dict[str, Any], control: dict[str, Any], key_report: dict[str, Any]) -> dict[str, Any]:
    gate = _route_gate(candidate, control, key_report)
    strict = []
    for key in DIAGNOSTIC_KEYS:
        if candidate["kaggle_diagnostic_score_variants_m"][key] < control["kaggle_diagnostic_score_variants_m"][key] - TOLERANCE:
            strict.append(key)
        else:
            gate["failures"].append(f"{key}_not_strictly_improved")
    gate["strictly_improved_variants"] = strict
    gate["passed"] = not gate["failures"]
    return gate


def _score_once(output_root: Path) -> dict[str, Any]:
    started = time.perf_counter()
    output_root = output_root.resolve()
    if output_root.exists() and any(output_root.iterdir()):
        raise Phase35EvaluationError(f"refusing to rerun Phase35 evaluation: {output_root}")
    manifest = _verify_evaluator_manifest()
    structural = _verify_structural_seal()
    train_sources = _load_train_truth_sources()
    validation_source = _load_validation_truth_source()
    output_root.mkdir(parents=True, exist_ok=True)
    # The marker prevents a second truth read if this one-shot process is
    # interrupted after opening truth but before publishing its result.
    atomic_json(output_root / "evaluation_in_progress.json", {
        "schema_version": SCHEMA,
        "status": "truth-evaluation-in-progress",
        "truth_open_count": 0,
        "mat_read_or_generated": False,
        "structural_seal_verified": True,
    })

    train_routes: dict[str, Any] = {}
    aggregate_by_lane: dict[str, list[dict[str, Any]]] = {lane: [] for lane in LANES}
    train_truth_read_count = 0
    for dataset_id in TRAIN_ROUTES:
        source = train_sources[dataset_id]
        # Sole read of this route's development truth in this process.
        truth, truth_hash, truth_bytes = phase29._read_truth_once(source["path"], dataset_id)
        train_truth_read_count += 1
        if truth_hash != source["expected_sha256"] or truth_bytes != source["expected_bytes"]:
            raise Phase35EvaluationError(f"train truth hash/size mismatch: {dataset_id}")
        rows_by_lane = {lane: structural["routes"][dataset_id][lane]["rows"] for lane in LANES}
        metrics, key_report = _route_metrics(rows_by_lane, truth, dataset_id)
        gates = {lane: _route_gate(metrics[lane], metrics[LANES[0]], key_report) for lane in CANDIDATES}
        for lane in LANES:
            aggregate_by_lane[lane].append(metrics[lane])
        train_routes[dataset_id] = {
            "truth": {"path": str(source["path"].relative_to(ROOT)), "sha256": truth_hash, "bytes": truth_bytes, "read_count": 1},
            "key_set": key_report,
            "lanes": {lane: {key: value for key, value in metrics[lane].items() if key != "key_set"} | {"key_set": metrics[lane]["key_set"]} for lane in LANES},
            "candidate_route_gates": gates,
        }

    train_aggregate = {lane: _aggregate(aggregate_by_lane[lane]) for lane in LANES}
    macro_gates = {lane: _macro_gate(train_aggregate[lane], train_aggregate[LANES[0]]) for lane in CANDIDATES}
    selected = [lane for lane in CANDIDATES if all(train_routes[route]["candidate_route_gates"][lane]["passed"] for route in TRAIN_ROUTES) and macro_gates[lane]["passed"]]
    selected_lane = min(selected, key=lambda lane: (train_aggregate[lane]["mean_kaggle_diagnostic_mean_m"], CANDIDATES.index(lane))) if selected else None
    train_gate_passed = selected_lane is not None

    validation: dict[str, Any] | None = None
    validation_truth_read_count = 0
    if selected_lane is not None:
        # Sole read of the already-opened Phase34 validation truth in this process.
        truth, truth_hash, truth_bytes = phase29._read_truth_once(validation_source["path"], VALIDATION_ROUTE)
        validation_truth_read_count = 1
        if truth_hash != validation_source["expected_sha256"] or truth_bytes != validation_source["expected_bytes"]:
            raise Phase35EvaluationError("Phase34 validation truth hash/size mismatch")
        rows_by_lane = {
            LANES[0]: structural["routes"][VALIDATION_ROUTE][LANES[0]]["rows"],
            selected_lane: structural["routes"][VALIDATION_ROUTE][selected_lane]["rows"],
        }
        metrics, key_report = _route_metrics(rows_by_lane, truth, VALIDATION_ROUTE)
        gate = _validation_gate(metrics[selected_lane], metrics[LANES[0]], key_report)
        validation = {
            "dataset_id": VALIDATION_ROUTE,
            "truth": {"path": str(validation_source["path"].relative_to(ROOT)), "sha256": truth_hash, "bytes": truth_bytes, "read_count": 1, "reused_phase34_materialization": True},
            "key_set": key_report,
            "control": metrics[LANES[0]],
            "selected_lane": selected_lane,
            "candidate": metrics[selected_lane],
            "gate": gate,
            "future_holdout_truth_open_count": 0,
        }
    passed = train_gate_passed and validation is not None and validation["gate"]["passed"]
    report: dict[str, Any] = {
        "schema_version": SCHEMA,
        "phase": 35,
        "status": "validation-pass" if passed else "no-go-train-or-validation-gate",
        "decision": "development-only-validation-go" if passed else "no-go-no-kaggle",
        "freeze": {"path": str(FREEZE.relative_to(ROOT)), "sha256": FREEZE_SHA256},
        "evaluator_manifest": {"path": str(EVALUATOR_MANIFEST.relative_to(ROOT)), "sha256": EVALUATOR_MANIFEST_SHA256},
        "structural_seal": {"path": str(STRUCTURAL_SEAL.relative_to(ROOT)), "sha256": sha256(STRUCTURAL_SEAL), "status": "sealed-truth-free-structural-pass"},
        "routes": train_routes,
        "train": {
            "routes": list(TRAIN_ROUTES),
            "truth_read_count": train_truth_read_count,
            "truth_read_count_per_route": 1,
            "aggregate": train_aggregate,
            "macro_gates": macro_gates,
            "selected_lane": selected_lane,
            "candidate_selection": "one unchanged A/B/C lane only; macro mean diagnostics then fixed lane-order tie-break",
            "gate_passed": train_gate_passed,
        },
        "validation": validation,
        "truth_open_count": train_truth_read_count + validation_truth_read_count,
        "train_truth_open_count": train_truth_read_count,
        "validation_truth_open_count": validation_truth_read_count,
        "future_holdout_truth_open_count": 0,
        "mat_read_or_generated": False,
        "token_or_kaggle_access": False,
        "solver_rerun_after_truth": False,
        "post_score_tuning": False,
        "vertical": "informational-unavailable-four-column-prediction",
        "runtime": {"wall_seconds": time.perf_counter() - started, "max_rss_kb_process": resource.getrusage(resource.RUSAGE_SELF).ru_maxrss},
    }
    result_path = output_root / "phase35_matrix_evaluation.json"
    atomic_json(result_path, report)
    atomic_json(output_root / "phase35_matrix_evaluation.manifest.json", {
        "schema_version": MANIFEST_SCHEMA,
        "status": report["status"],
        "result": {"path": str(result_path.relative_to(ROOT)), "sha256": sha256(result_path)},
        "freeze": {"path": str(FREEZE.relative_to(ROOT)), "sha256": FREEZE_SHA256},
        "structural_seal": {"path": str(STRUCTURAL_SEAL.relative_to(ROOT)), "sha256": sha256(STRUCTURAL_SEAL)},
        "evaluator_manifest": {"path": str(EVALUATOR_MANIFEST.relative_to(ROOT)), "sha256": EVALUATOR_MANIFEST_SHA256},
        "train_truth_open_count": train_truth_read_count,
        "validation_truth_open_count": validation_truth_read_count,
        "future_holdout_truth_open_count": 0,
        "truth_open_count": report["truth_open_count"],
        "mat_read_or_generated": False,
        "token_or_kaggle_access": False,
        "post_score_tuning": False,
        "atomic_publish": True,
    })
    try:
        (output_root / "evaluation_in_progress.json").unlink()
    except FileNotFoundError:
        pass
    return report


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("operation", choices=("score",))
    parser.add_argument("--output-root", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args(argv)
    try:
        report = _score_once(args.output_root)
    except (OSError, Phase35EvaluationError, ValueError) as exc:
        print(f"phase35-eval: {exc}", file=sys.stderr)
        return 2
    print(json.dumps({"status": report["status"], "selected_lane": report["train"]["selected_lane"], "truth_open_count": report["truth_open_count"]}, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
