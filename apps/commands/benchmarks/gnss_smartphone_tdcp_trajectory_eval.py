#!/usr/bin/env python3
"""Evaluate the frozen TDCP trajectory candidate on the fixed train split.

This evaluator is intentionally train-only.  It emits a sealed No-Go record
when the frozen candidate regresses any required per-route metric, and it will
not materialize or read the metadata-selected fresh validation/holdout route
unless the train gate passes.  The candidate itself is generated without a
truth path; truth is opened only after the candidate manifest has been sealed.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys
from typing import Any

from support.gnss_runtime import application_root


ROOT = application_root(__file__)
BENCHMARK_DIR = ROOT / "apps" / "commands" / "benchmarks"
if str(BENCHMARK_DIR) not in sys.path:
    sys.path.insert(0, str(BENCHMARK_DIR))
import gnss_smartphone_kaggle as kaggle  # noqa: E402
import gnss_smartphone_tdcp_trajectory as tdcp  # noqa: E402
import gnss_smartphone_trajectory_smoother as smoother  # noqa: E402
import gnss_smartphone_trajectory_smoother_eval as smoother_eval  # noqa: E402


SCHEMA_VERSION = "smartphone-r5-tdcp-trajectory-evaluation.v1"
DEFAULT_SELECTION_RECORD = ROOT / "docs/use_cases/records/smartphone_r5_tdcp_trajectory_selection.json"
DEFAULT_OUTPUT_DIR = ROOT / "output/smartphone-r5/tdcp-trajectory-evaluation"
TRAIN_CASES = (
    {
        "dataset_id": "2021-01-04-21-50-us-ca-e1highway280driveroutea/mi8",
        "name": "mi8",
        "base_position": ROOT / "output/smartphone-r5/generalization-v6/routes/2021-01-04-21-50-us-ca-e1highway280driveroutea/mi8/spp/libgnsspp_spp.pos",
        "device_gnss": ROOT / "output/smartphone-r5/generalization-v6/routes/2021-01-04-21-50-us-ca-e1highway280driveroutea/mi8/inputs/device_gnss.csv",
        "ground_truth": ROOT / "output/smartphone-r5/generalization-v6/routes/2021-01-04-21-50-us-ca-e1highway280driveroutea/mi8/inputs/ground_truth.csv",
    },
    {
        "dataset_id": "2022-08-04-20-07-us-ca-sjc-q/pixel5",
        "name": "pixel5",
        "base_position": ROOT / "output/smartphone-r5/generalization-v6/routes/2022-08-04-20-07-us-ca-sjc-q/pixel5/spp/libgnsspp_spp.pos",
        "device_gnss": ROOT / "output/smartphone-r5/generalization-v6/routes/2022-08-04-20-07-us-ca-sjc-q/pixel5/inputs/device_gnss.csv",
        "ground_truth": ROOT / "output/smartphone-r5/generalization-v6/routes/2022-08-04-20-07-us-ca-sjc-q/pixel5/inputs/ground_truth.csv",
    },
    {
        "dataset_id": "2023-05-24-20-26-us-ca-sjc-ge2/pixel7pro",
        "name": "pixel7pro",
        "base_position": ROOT / "output/smartphone-r5/hatch-full-w30/libgnsspp_spp.pos",
        "device_gnss": ROOT / "data/gsdc2023/materialized/dataset_2023/train/2023-05-24-20-26-us-ca-sjc-ge2/pixel7pro/device_gnss.csv",
        "ground_truth": ROOT / "data/gsdc2023/materialized/dataset_2023/train/2023-05-24-20-26-us-ca-sjc-ge2/pixel7pro/ground_truth.csv",
    },
)
REQUIRED_METRICS = (
    ("availability_ratio",),
    ("truth_coverage_ratio",),
    ("horizontal_wgs84_m", "p50_m"),
    ("horizontal_wgs84_m", "p95_m"),
    ("vertical_p95_abs_m",),
    ("kaggle_diagnostic_score_variants_m", "haversine_sphere__linear_n_minus_1"),
    ("kaggle_diagnostic_score_variants_m", "haversine_sphere__nearest_rank_ceiling"),
    ("kaggle_diagnostic_score_variants_m", "wgs84_vincenty__linear_n_minus_1"),
    ("kaggle_diagnostic_score_variants_m", "wgs84_vincenty__nearest_rank_ceiling"),
)


def _load_json(path: Path, label: str) -> dict[str, Any]:
    if not path.is_file():
        raise ValueError(f"missing {label}: {path}")
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ValueError(f"invalid {label}: {path}") from exc
    if not isinstance(payload, dict):
        raise ValueError(f"{label} must be a JSON object")
    return payload


def _metric_value(metrics: dict[str, Any], path: tuple[str, ...]) -> float:
    value: Any = metrics
    for key in path:
        if not isinstance(value, dict) or key not in value:
            return math.inf
        value = value[key]
    try:
        result = float(value)
    except (TypeError, ValueError):
        return math.inf
    return result if math.isfinite(result) else math.inf


def _row_from_position(row: smoother.PositionRow, source: str) -> smoother.SmoothedRow:
    return smoother.SmoothedRow(
        timestamp_ms=row.timestamp_ms,
        week=row.week,
        tow=row.tow,
        ecef=row.ecef,
        latitude=row.latitude,
        longitude=row.longitude,
        height=row.height,
        status=row.status,
        satellites=row.satellites,
        pdop=row.pdop,
        ratio=row.ratio,
        fixed_ambiguities=row.fixed_ambiguities,
        iterations=row.iterations,
        source=source,
        segment_id=0,
        measurement_used=True,
        outlier_rejected=False,
        innovation_sigma=None,
        position_sigma_m=0.0,
        reacquired=False,
    )


def _score(position_path: Path, device_path: Path, truth_path: Path, source: str) -> dict[str, Any]:
    positions = smoother._read_positions(position_path, tdcp.LEAP_SECONDS)
    device_epochs = smoother._read_device_epochs(device_path, 0)
    truth = smoother_eval._read_truth(truth_path)
    by_timestamp = {row.timestamp_ms: row for row in positions}
    rows = [_row_from_position(row, source) for row in positions]
    if not rows:
        raise ValueError(f"empty position artifact: {position_path}")
    return smoother_eval._score_rows(
        rows,
        by_timestamp,
        truth,
        0,
        len(device_epochs),
        match_tolerance_ms=100,
    )


def _aggregate(route_metrics: list[dict[str, Any]]) -> dict[str, Any]:
    aggregate: dict[str, Any] = {}
    for path in REQUIRED_METRICS:
        values = [_metric_value(metrics, path) for metrics in route_metrics]
        if any(not math.isfinite(value) for value in values):
            value: float | None = None
        else:
            value = sum(values) / len(values) if values else None
        cursor: dict[str, Any] = aggregate
        for key in path[:-1]:
            cursor = cursor.setdefault(key, {})
        cursor[path[-1]] = value
    diagnostic_values = [metrics.get("kaggle_diagnostic_mean_m") for metrics in route_metrics]
    aggregate["kaggle_diagnostic_mean_m"] = (
        sum(float(value) for value in diagnostic_values) / len(diagnostic_values)
        if diagnostic_values and all(value is not None and math.isfinite(float(value)) for value in diagnostic_values)
        else None
    )
    return aggregate


def _compare(
    baseline: dict[str, Any], candidate: dict[str, Any], *, aggregate: bool = False
) -> dict[str, Any]:
    failures: list[str] = []
    deltas: dict[str, float | None] = {}
    for path in REQUIRED_METRICS:
        base = _metric_value(baseline, path)
        current = _metric_value(candidate, path)
        label = ".".join(path)
        delta = current - base if math.isfinite(current) and math.isfinite(base) else None
        deltas[label] = delta
        if not math.isfinite(current) or not math.isfinite(base):
            failures.append(f"{label}_nonfinite")
        elif current > base + 1e-12:
            failures.append(f"{label}_regression")
    strict_h_p95 = (
        _metric_value(candidate, ("horizontal_wgs84_m", "p95_m"))
        < _metric_value(baseline, ("horizontal_wgs84_m", "p95_m")) - 1e-12
    )
    strict_diagnostic = (
        _metric_value(candidate, ("kaggle_diagnostic_mean_m",))
        < _metric_value(baseline, ("kaggle_diagnostic_mean_m",)) - 1e-12
    )
    if aggregate and not strict_h_p95:
        failures.append("aggregate_horizontal_p95_not_strictly_improved")
    if aggregate and not strict_diagnostic:
        failures.append("aggregate_diagnostic_mean_not_strictly_improved")
    return {
        "passed": not failures,
        "failures": failures,
        "deltas_candidate_minus_baseline": deltas,
        "strict_aggregate_horizontal_p95_improvement": strict_h_p95 if aggregate else None,
        "strict_aggregate_diagnostic_mean_improvement": strict_diagnostic if aggregate else None,
    }


def _ensure_candidate(case: dict[str, Any], output_dir: Path) -> tuple[Path, dict[str, Any]]:
    case_dir = output_dir / str(case["name"])
    position = case_dir / "tdcp_blended.pos"
    manifest_path = case_dir / "tdcp_manifest.json"
    if manifest_path.is_file() and position.is_file():
        manifest = _load_json(manifest_path, f"TDCP manifest for {case['dataset_id']}")
        inputs = dict(manifest.get("inputs", {}))
        if (
            dict(inputs.get("base_position", {})).get("sha256") == tdcp._sha256(case["base_position"])
            and dict(inputs.get("device_gnss", {})).get("sha256") == tdcp._sha256(case["device_gnss"])
            and dict(manifest.get("truth_free_contract", {})).get("truth_path") is None
        ):
            return position, manifest
    result = tdcp.build_trajectory(case["base_position"], case["device_gnss"])
    manifest = tdcp.write_outputs(
        result,
        base_position_path=case["base_position"],
        device_gnss_path=case["device_gnss"],
        output_dir=case_dir,
        phone=str(case["name"]),
        dataset_id=str(case["dataset_id"]),
        submission_output=case_dir / "submission.csv",
    )
    return position, manifest


def evaluate(selection_record: Path, output_dir: Path) -> dict[str, Any]:
    selection = _load_json(selection_record, "TDCP selection record")
    if selection.get("schema_version") != "smartphone-r5-tdcp-trajectory-selection.v1":
        raise ValueError("TDCP selection record schema is invalid")
    if selection.get("status") != "selection-frozen-before-evaluation":
        raise ValueError("TDCP selection record is not frozen")
    candidate = dict(selection.get("frozen_candidate", {}))
    if candidate.get("candidate_search_after_freeze") is not False or candidate.get("numeric_parameters_frozen") is not True:
        raise ValueError("TDCP candidate is not frozen")
    sealed_policy = dict(selection.get("sealed_data_policy", {}))
    if sealed_policy.get("new_holdout_materialized") is not False or sealed_policy.get("new_holdout_truth_opened") is not False:
        raise ValueError("selection record does not prove an unopened holdout")
    output_dir.mkdir(parents=True, exist_ok=True)
    route_reports: list[dict[str, Any]] = []
    truth_open_count = 0
    for case in TRAIN_CASES:
        for key in ("base_position", "device_gnss", "ground_truth"):
            if not Path(case[key]).is_file():
                raise ValueError(f"missing fixed train {key}: {case[key]}")
        candidate_position, candidate_manifest = _ensure_candidate(case, output_dir)
        # The candidate manifest and its position hash are sealed before this
        # call opens the corresponding development truth file.
        candidate_hash = tdcp._sha256(candidate_position)
        baseline_metrics = _score(case["base_position"], case["device_gnss"], case["ground_truth"], "baseline")
        truth_open_count += 1
        candidate_metrics = _score(candidate_position, case["device_gnss"], case["ground_truth"], "tdcp_blend")
        route_gate = _compare(baseline_metrics, candidate_metrics)
        route_reports.append(
            {
                "dataset_id": case["dataset_id"],
                "baseline": baseline_metrics,
                "candidate": candidate_metrics,
                "comparison": route_gate,
                "truth_free_artifact": {
                    "manifest_path": str(output_dir / str(case["name"]) / "tdcp_manifest.json"),
                    "manifest_sha256": tdcp._sha256(output_dir / str(case["name"]) / "tdcp_manifest.json"),
                    "position_path": str(candidate_position),
                    "position_sha256": candidate_hash,
                    "truth_path": None,
                    "candidate_manifest_truth_free": dict(candidate_manifest.get("truth_free_contract", {})).get("truth_path") is None,
                },
            }
        )
    baseline_aggregate = _aggregate([report["baseline"] for report in route_reports])
    candidate_aggregate = _aggregate([report["candidate"] for report in route_reports])
    aggregate_gate = _compare(baseline_aggregate, candidate_aggregate, aggregate=True)
    per_route_pass = all(bool(report["comparison"]["passed"]) for report in route_reports)
    train_pass = per_route_pass and bool(aggregate_gate["passed"])
    decision = "train-gate-pass-fresh-validation-authorized" if train_pass else "no-go-train-gate"
    record = {
        "schema_version": SCHEMA_VERSION,
        "status": "sealed-train-evaluation",
        "decision": decision,
        "selection_record": {"path": str(selection_record), "sha256": tdcp._sha256(selection_record)},
        "candidate": {
            "id": candidate.get("candidate_id"),
            "parameters": tdcp._candidate_parameters(),
            "source": {"path": str(Path(tdcp.__file__)), "sha256": tdcp._sha256(Path(tdcp.__file__))},
        },
        "evaluation_contract": {
            "train_routes": [case["dataset_id"] for case in TRAIN_CASES],
            "train_gate": "no per-route or aggregate regression on availability, coverage, H P50/P95, V P95, or four diagnostics; strict aggregate H P95 and diagnostic mean improvement",
            "truth_read_order": "truth-free candidate position/manifest hash before each development truth read",
            "public_private_kaggle_scores_used": False,
            "holdout_opened": False,
            "fresh_validation_opened": False,
            "unused_holdout": selection.get("fixed_roles", {}).get("unused_holdout"),
            "post_holdout_tuning": False,
        },
        "truth_open_count": truth_open_count,
        "baseline_aggregate": baseline_aggregate,
        "candidate_aggregate": candidate_aggregate,
        "aggregate_comparison": aggregate_gate,
        "route_reports": route_reports,
        "next_step": (
            "A fresh validation may be materialized only after this train gate passes."
            if train_pass
            else "Fresh validation and the metadata-selected unused holdout remain sealed because the fixed train gate failed."
        ),
        "integrity": {
            "truth_used_for_candidate_generation": False,
            "truth_used_for_scoring_only": True,
            "unused_holdout_truth_opened": False,
            "external_mutation": False,
            "production_default_changed": False,
        },
    }
    record_path = output_dir / "tdcp_train_evaluation.json"
    tdcp._atomic_write(record_path, tdcp._json_bytes(record))
    manifest = {
        "schema_version": "smartphone-r5-tdcp-trajectory-evaluation-manifest.v1",
        "record": {"path": str(record_path), "sha256": tdcp._sha256(record_path)},
        "selection_record": record["selection_record"],
        "candidate_source": record["candidate"]["source"],
        "truth_free_artifacts": [report["truth_free_artifact"] for report in route_reports],
        "holdout_materialized": False,
        "holdout_truth_opened": False,
        "public_private_kaggle_scores_used": False,
    }
    manifest_path = output_dir / "tdcp_train_evaluation_manifest.json"
    tdcp._atomic_write(manifest_path, tdcp._json_bytes(manifest))
    record["artifacts"] = {
        "record": {"path": str(record_path), "sha256": tdcp._sha256(record_path)},
        "manifest": {"path": str(manifest_path), "sha256": tdcp._sha256(manifest_path)},
    }
    # Keep the record immutable after publication; the manifest is the
    # authoritative link for the record hash and is written after the record.
    print(f"Smartphone TDCP train evaluation {decision}: {record_path}")
    return record


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(prog="smartphone-tdcp-trajectory-eval")
    parser.add_argument("--selection-record", type=Path, default=DEFAULT_SELECTION_RECORD)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    return parser.parse_args(argv)


def run(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    try:
        evaluate(args.selection_record, args.output_dir)
    except (OSError, ValueError, smoother.SmootherError, tdcp.TdcpError) as exc:
        print(f"smartphone TDCP trajectory evaluation failed: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(run())
