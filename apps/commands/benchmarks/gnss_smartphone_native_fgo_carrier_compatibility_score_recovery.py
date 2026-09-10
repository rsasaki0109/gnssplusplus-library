#!/usr/bin/env python3
"""Scoring-only recovery for the carrier compatibility development run.

The first evaluator process read all eight already-authorized development
truth files and then stopped on a schema bug: route metrics use
``kaggle_diagnostic_score_variants_m`` while the aggregate helper uses
``mean_kaggle_diagnostic_score_variants_m``.  This command does not rerun any
solver, does not materialize truth, and does not alter the frozen selector. It
hashes each existing truth file before and after one recovery read and writes a
separate sealed result.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys
from typing import Any

_COMMANDS_DIR = Path(__file__).resolve().parents[1]
if str(_COMMANDS_DIR) not in sys.path:
    sys.path.insert(0, str(_COMMANDS_DIR))
from support.gnss_runtime import application_root  # noqa: E402

ROOT = application_root(__file__)
BENCHMARK_DIR = ROOT / "apps/commands/benchmarks"
if str(BENCHMARK_DIR) not in sys.path:
    sys.path.insert(0, str(BENCHMARK_DIR))

import gnss_smartphone_native_fgo_carrier_compatibility_eval as carrier  # noqa: E402


DEFAULT_OUTPUT = ROOT / "output/smartphone-r5/native-fgo-carrier-compatibility-v1"
RECOVERY_OUTPUT = DEFAULT_OUTPUT / "train_score_recovery.json"
RECOVERY_MANIFEST = DEFAULT_OUTPUT / "train_score_recovery.manifest.json"
DIAGNOSTIC_KEYS = tuple(carrier.DIAGNOSTIC_KEYS)


class RecoveryError(ValueError):
    """Raised when the scoring-only recovery contract is violated."""


def _metric_map(metrics: dict[str, Any]) -> dict[str, float]:
    route_values = metrics.get("kaggle_diagnostic_score_variants_m")
    if isinstance(route_values, dict):
        values = route_values
    else:
        values = metrics.get("mean_kaggle_diagnostic_score_variants_m")
    if not isinstance(values, dict):
        raise RecoveryError("diagnostic aggregate schema is missing")
    result: dict[str, float] = {}
    for key in DIAGNOSTIC_KEYS:
        try:
            value = float(values[key])
        except (KeyError, TypeError, ValueError) as exc:
            raise RecoveryError(f"diagnostic value is missing/non-numeric: {key}") from exc
        if not math.isfinite(value):
            raise RecoveryError(f"diagnostic value is non-finite: {key}")
        result[key] = value
    return result


def _diagnostic_mean(metrics: dict[str, Any]) -> float:
    if "kaggle_diagnostic_mean_m" in metrics and metrics["kaggle_diagnostic_mean_m"] is not None:
        value = float(metrics["kaggle_diagnostic_mean_m"])
        if math.isfinite(value):
            return value
    if "mean_kaggle_diagnostic_m" in metrics and metrics["mean_kaggle_diagnostic_m"] is not None:
        value = float(metrics["mean_kaggle_diagnostic_m"])
        if math.isfinite(value):
            return value
    values = _metric_map(metrics)
    return sum(values.values()) / len(values)


def strict_comparison(candidate: dict[str, Any], baseline: dict[str, Any]) -> dict[str, Any]:
    candidate_values = _metric_map(candidate)
    baseline_values = _metric_map(baseline)
    failures: list[str] = []
    for key in DIAGNOSTIC_KEYS:
        if not candidate_values[key] < baseline_values[key] - 1e-9:
            failures.append(f"{key}_not_strictly_improved")
    if not _diagnostic_mean(candidate) < _diagnostic_mean(baseline) - 1e-9:
        failures.append("diagnostic_mean_not_strictly_improved")
    def scalar(metrics: dict[str, Any], route_key: str, aggregate_key: str) -> float:
        value = metrics.get(route_key, metrics.get(aggregate_key))
        if value is None or not math.isfinite(float(value)):
            raise RecoveryError(f"aggregate scalar schema is missing/non-finite: {route_key}/{aggregate_key}")
        return float(value)

    for name, route_key, aggregate_key in (
        ("availability_regression", "availability_ratio", "mean_availability_ratio"),
        ("truth_coverage_regression", "truth_coverage_ratio", "mean_truth_coverage_ratio"),
    ):
        if scalar(candidate, route_key, aggregate_key) < scalar(baseline, route_key, aggregate_key) - 1e-12:
            failures.append(name)
    candidate_vertical = candidate.get("vertical_p95_abs_m", candidate.get("mean_vertical_p95_abs_m"))
    baseline_vertical = baseline.get("vertical_p95_abs_m", baseline.get("mean_vertical_p95_abs_m"))
    if candidate_vertical is not None and baseline_vertical is not None and float(candidate_vertical) > float(baseline_vertical) + 0.25:
        failures.append("vertical_safety_margin_regression")
    return {"passed": not failures, "failures": failures}


def _aggregate(metrics: list[dict[str, Any]]) -> dict[str, Any]:
    return carrier._aggregate(metrics)


def run_recovery(output_root: Path) -> dict[str, Any]:
    carrier.verify_freeze()
    sealed = carrier._verify_truth_free(output_root)
    reports: dict[str, Any] = {}
    truth_hash_before: dict[str, str] = {}
    truth_hash_after: dict[str, str] = {}
    recovery_read_count = 0
    for dataset_id in carrier.COHORT:
        truth_path = output_root / "train_truth" / carrier.safe_id(dataset_id) / "ground_truth.csv"
        if not truth_path.is_file():
            raise RecoveryError(f"failed run did not leave a truth file for recovery: {dataset_id}")
        truth_hash_before[dataset_id] = carrier.sha256(truth_path)
        reports[dataset_id] = carrier._score_route(sealed["routes"][dataset_id], truth_path, output_root, dataset_id)
        recovery_read_count += 1
        truth_hash_after[dataset_id] = carrier.sha256(truth_path)
        if truth_hash_before[dataset_id] != truth_hash_after[dataset_id]:
            raise RecoveryError(f"truth file changed during recovery: {dataset_id}")
    baseline_metrics = [item["baseline8"] for item in reports.values()]
    candidate_metrics = [item["carrier_float50"] for item in reports.values()]
    selected_metrics = [item["selected"] for item in reports.values()]
    aggregate = {
        "baseline8": _aggregate(baseline_metrics),
        "carrier_float50": _aggregate(candidate_metrics),
        "selected": _aggregate(selected_metrics),
    }
    aggregate["gate_selected_vs_baseline"] = strict_comparison(aggregate["selected"], aggregate["baseline8"])
    route_gates = {dataset_id: strict_comparison(item["selected"], item["baseline8"]) for dataset_id, item in reports.items()}
    role = carrier.load_json(carrier.ROLE_INVENTORY, "role inventory")
    role_rows = {row["dataset_id"]: row for row in role["development_cv_cohort"]}
    folds: dict[str, Any] = {"leave_one_route_group_out": {}, "leave_one_phone_family_out": {}}
    for name, grouping in (
        ("leave_one_route_group_out", {dataset_id: role_rows[dataset_id]["route_group"] for dataset_id in carrier.COHORT}),
        ("leave_one_phone_family_out", {dataset_id: dataset_id.split("/", 1)[1] for dataset_id in carrier.COHORT}),
    ):
        for group in sorted(set(grouping.values())):
            ids = [dataset_id for dataset_id in carrier.COHORT if grouping[dataset_id] == group]
            baseline = _aggregate([reports[dataset_id]["baseline8"] for dataset_id in ids])
            selected = _aggregate([reports[dataset_id]["selected"] for dataset_id in ids])
            folds[name][group] = {"identities": ids, "baseline": baseline, "selected": selected, "gate": strict_comparison(selected, baseline)}
    route_pass = all(value["passed"] for value in route_gates.values())
    fold_pass = all(value["gate"]["passed"] for groups in folds.values() for value in groups.values())
    passed = route_pass and fold_pass and aggregate["gate_selected_vs_baseline"]["passed"]
    result = {
        "schema_version": "smartphone-r5-native-fgo-carrier-compatibility-train-score-recovery.v1",
        "status": "promote-development-only" if passed else "no-go-train-gate",
        "candidate_id": "native-fgo-v1-carrier-compatibility-selector-v1",
        "recovery_reason": "aggregate diagnostic schema mismatch after first scoring process; scoring-only null/schema correction",
        "truth_access": {
            "truth_free_artifacts_reused": True,
            "prior_failed_evaluation_truth_open_count": len(carrier.COHORT),
            "recovery_truth_read_count": recovery_read_count,
            "truth_read_operations_total": len(carrier.COHORT) + recovery_read_count,
            "truth_materialized_in_recovery": False,
            "truth_hash_before": truth_hash_before,
            "truth_hash_after": truth_hash_after,
            "truth_hash_unchanged": truth_hash_before == truth_hash_after,
            "validation_truth_open_count": 0,
            "holdout_truth_open_count": 0,
            "test_truth_open_count": 0,
        },
        "routes": reports,
        "aggregate": aggregate,
        "route_gates": route_gates,
        "cv_folds": folds,
        "gate": {
            "all_four_diagnostics_and_mean_strictly_improve_routewise": route_pass,
            "all_leave_out_folds_strictly_improve": fold_pass,
            "aggregate_strict_improvement": aggregate["gate_selected_vs_baseline"],
            "passed": passed,
        },
        "policy": {
            "candidate_or_parameter_changed": False,
            "solver_rerun": False,
            "leaderboard_used_for_tuning": False,
            "validation_truth_open_count": 0,
            "holdout_truth_open_count": 0,
            "post_truth_tuning": False,
            "kaggle_submission": False,
            "production_default_changed": False,
            "next_action": "require a genuinely new validation asset" if passed else "seal final No-Go; do not open validation or holdout",
        },
        "freeze": {
            "record": carrier.relative(carrier.FREEZE),
            "record_sha256": carrier.sha256(carrier.FREEZE),
            "manifest": carrier.relative(carrier.FREEZE_MANIFEST),
            "manifest_sha256": carrier.sha256(carrier.FREEZE_MANIFEST),
            "truth_free_manifest_sha256": sealed["manifest_sha256"],
        },
    }
    carrier.atomic_json(RECOVERY_OUTPUT, result)
    carrier.atomic_json(RECOVERY_MANIFEST, {
        "schema_version": "smartphone-r5-native-fgo-carrier-compatibility-train-score-recovery-manifest.v1",
        "report": {"path": carrier.relative(RECOVERY_OUTPUT), "sha256": carrier.sha256(RECOVERY_OUTPUT)},
        "truth_free_manifest_sha256": sealed["manifest_sha256"],
        "prior_failed_truth_open_count": len(carrier.COHORT),
        "recovery_truth_read_count": recovery_read_count,
        "truth_hash_unchanged": truth_hash_before == truth_hash_after,
        "validation_truth_open_count": 0,
        "holdout_truth_open_count": 0,
        "no_post_truth_tuning": True,
        "candidate_or_parameter_changed": False,
    })
    return result


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="gnss_smartphone_native_fgo_carrier_compatibility_score_recovery")
    parser.add_argument("--output-root", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args(argv)
    try:
        report = run_recovery(args.output_root.resolve())
        print(json.dumps({"status": report["status"], "prior_failed_truth_open_count": report["truth_access"]["prior_failed_evaluation_truth_open_count"], "recovery_truth_read_count": report["truth_access"]["recovery_truth_read_count"], "validation_truth_open_count": 0, "holdout_truth_open_count": 0}, sort_keys=True))
        return 0 if report["status"] == "promote-development-only" else 1
    except (carrier.CarrierCompatibilityError, RecoveryError) as exc:
        print(f"carrier compatibility scoring recovery error: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
