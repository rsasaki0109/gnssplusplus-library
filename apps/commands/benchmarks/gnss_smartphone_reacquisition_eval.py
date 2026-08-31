#!/usr/bin/env python3
"""Evaluate bounded truth-free smartphone trajectory reacquisition.

The filter is run without labels.  Ground truth is opened only after all
raw/existing-smoother/candidate artifacts for a route have been published.
The fixed train/validation roles and the candidate set are deliberately kept
in this module and in a pre-evaluation record so that the validation route
cannot influence parameter selection.
"""

from __future__ import annotations

import argparse
import csv
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
sys.path.insert(0, str(BENCHMARK_DIR))
import gnss_smartphone_kaggle as kaggle  # noqa: E402
import gnss_smartphone_trajectory_smoother as smoother  # noqa: E402
import gnss_smartphone_trajectory_smoother_eval as smoother_eval  # noqa: E402


SCHEMA_VERSION = "smartphone-r5-reacquisition-evaluation.v1"
DEFAULT_GENERALIZATION_ROOT = ROOT / "output" / "smartphone-r5" / "generalization-v6"
DEFAULT_PROFILE = ROOT / "configs" / "benchmarks" / "smartphone_r5_gsdc2023.json"
DEFAULT_MAIN_POSITION = ROOT / "output" / "smartphone-r5" / "hatch-full-w30" / "libgnsspp_spp.pos"
DEFAULT_MAIN_DEVICE = (
    ROOT
    / "data"
    / "gsdc2023"
    / "materialized"
    / "dataset_2023"
    / "train"
    / "2023-05-24-20-26-us-ca-sjc-ge2"
    / "pixel7pro"
    / "device_gnss.csv"
)
DEFAULT_MAIN_TRUTH = DEFAULT_MAIN_DEVICE.with_name("ground_truth.csv")
HOLDOUT_ID = "2023-09-06-22-49-us-ca-routebb1/pixel7pro"
TRAIN_IDS = (
    "2021-01-04-21-50-us-ca-e1highway280driveroutea/mi8",
    "2022-08-04-20-07-us-ca-sjc-q/pixel5",
)
VALIDATION_IDS = ("2023-03-08-21-34-us-ca-mtv-u/pixel6pro",)
MAIN_ID = "2023-05-24-20-26-us-ca-sjc-ge2/pixel7pro"
CANDIDATES = (
    {"id": "reacq_r2_d2", "max_consecutive_rejects": 2, "max_prediction_duration_s": 2.0},
    {"id": "reacq_r3_d3", "max_consecutive_rejects": 3, "max_prediction_duration_s": 3.0},
    {"id": "reacq_r5_d5", "max_consecutive_rejects": 5, "max_prediction_duration_s": 5.0},
)
BASELINE_CONFIG = {
    "process_noise": 1.0,
    "measurement_floor_m": 1.0,
    "outlier_gate_sigma": 5.0,
    "segment_gap_s": 10.0,
}
METRIC_PATHS = (
    ("horizontal_wgs84_m", "p50_m"),
    ("horizontal_wgs84_m", "p95_m"),
    ("vertical_p95_abs_m",),
)
_DIAGNOSTIC_KEYS = tuple(
    f"{distance}__{percentile}"
    for distance in ("wgs84_vincenty", "haversine_sphere")
    for percentile in ("linear_n_minus_1", "nearest_rank_ceiling")
)


class ReacquisitionError(ValueError):
    """Raised when the fixed development-only contract cannot be satisfied."""


def _sha256(path: Path) -> str:
    if not path.is_file():
        raise ReacquisitionError(f"missing file: {path}")
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


def _atomic_csv(path: Path, fields: tuple[str, ...], rows: list[dict[str, Any]]) -> None:
    import io

    output = io.StringIO(newline="")
    writer = csv.DictWriter(output, fieldnames=fields, lineterminator="\n")
    writer.writeheader()
    for row in rows:
        writer.writerow({field: row.get(field) for field in fields})
    smoother._atomic_write(path, output.getvalue().encode("utf-8"))


def _load_profile(path: Path) -> dict[str, Any]:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ReacquisitionError(f"invalid profile: {path}") from exc
    if not isinstance(payload, dict) or payload.get("schema_version") != "smartphone-r5-profile.v1":
        raise ReacquisitionError("profile schema is not smartphone-r5-profile.v1")
    datasets = payload.get("datasets")
    if not isinstance(datasets, dict) or not isinstance(datasets.get("holdout"), dict):
        raise ReacquisitionError("profile has no holdout metadata")
    if datasets["holdout"].get("id") != HOLDOUT_ID:
        raise ReacquisitionError("profile holdout ID differs from the sealed contract")
    return payload


def _safe_id(dataset_id: str) -> str:
    return dataset_id.replace("/", "__")


def _route_inputs(
    dataset_id: str,
    generalization_root: Path,
    *,
    main_position: Path,
    main_device: Path,
    main_truth: Path,
) -> dict[str, Path | str]:
    if dataset_id == HOLDOUT_ID:
        raise ReacquisitionError("holdout route is sealed")
    if dataset_id == MAIN_ID:
        return {
            "dataset_id": dataset_id,
            "position": main_position,
            "device": main_device,
            "truth": main_truth,
        }
    if dataset_id not in (*TRAIN_IDS, *VALIDATION_IDS):
        raise ReacquisitionError(f"unexpected fixed route ID: {dataset_id}")
    route, phone = dataset_id.split("/", 1)
    root = generalization_root / "routes" / route / phone
    return {
        "dataset_id": dataset_id,
        "position": root / "spp" / "canonical.pos",
        "device": root / "inputs" / "device_gnss.csv",
        "truth": root / "inputs" / "ground_truth.csv",
    }


def _raw_rows(positions: list[smoother.PositionRow]) -> list[smoother.SmoothedRow]:
    return [
        smoother.SmoothedRow(
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
            source="measured",
            segment_id=0,
            measurement_used=True,
            outlier_rejected=False,
            innovation_sigma=None,
            position_sigma_m=smoother._measurement_sigma(row, 1.0),
        )
        for row in positions
    ]


def _metric(metrics: dict[str, Any], path: tuple[str, ...]) -> float:
    value: Any = metrics
    for key in path:
        value = value[key]
    if value is None or not math.isfinite(float(value)):
        return math.inf
    return float(value)


def _score(
    rows: list[smoother.SmoothedRow],
    positions: list[smoother.PositionRow],
    epochs: list[int],
    truth: dict[int, tuple[float, float, float]],
) -> dict[str, Any]:
    return smoother_eval._score_rows(
        rows,
        {row.timestamp_ms: row for row in positions},
        truth,
        0,
        len(epochs),
        match_tolerance_ms=100,
    )


def _aggregate(metrics: list[dict[str, Any]]) -> dict[str, Any]:
    if not metrics:
        raise ReacquisitionError("cannot aggregate an empty route set")
    values: dict[str, Any] = {
        "route_count": len(metrics),
        "mean_availability_ratio": sum(float(row["availability_ratio"]) for row in metrics) / len(metrics),
        "mean_truth_coverage_ratio": sum(float(row["truth_coverage_ratio"]) for row in metrics) / len(metrics),
        "mean_horizontal_wgs84_p50_m": sum(_metric(row, ("horizontal_wgs84_m", "p50_m")) for row in metrics) / len(metrics),
        "mean_horizontal_wgs84_p95_m": sum(_metric(row, ("horizontal_wgs84_m", "p95_m")) for row in metrics) / len(metrics),
        "mean_vertical_p95_abs_m": sum(_metric(row, ("vertical_p95_abs_m",)) for row in metrics) / len(metrics),
        "mean_kaggle_diagnostic_score_variants_m": {
            key: sum(
                _metric(row, ("kaggle_diagnostic_score_variants_m", key))
                for row in metrics
            )
            / len(metrics)
            for key in _DIAGNOSTIC_KEYS
        },
    }
    values["mean_kaggle_diagnostic_m"] = sum(
        values["mean_kaggle_diagnostic_score_variants_m"].values()
    ) / len(_DIAGNOSTIC_KEYS)
    return values


def _aggregate_non_regression(
    candidate: dict[str, Any], reference: dict[str, Any]
) -> tuple[bool, list[str]]:
    failures: list[str] = []
    if float(candidate["mean_availability_ratio"]) < float(reference["mean_availability_ratio"]) - 1e-12:
        failures.append("availability_regression")
    for key, path in (
        ("h_median", ("mean_horizontal_wgs84_p50_m",)),
        ("h_p95", ("mean_horizontal_wgs84_p95_m",)),
        ("v_p95", ("mean_vertical_p95_abs_m",)),
    ):
        if float(candidate[path[0]]) > float(reference[path[0]]) + 1e-12:
            failures.append(f"{key}_regression")
    for key in _DIAGNOSTIC_KEYS:
        if (
            float(candidate["mean_kaggle_diagnostic_score_variants_m"][key])
            > float(reference["mean_kaggle_diagnostic_score_variants_m"][key]) + 1e-12
        ):
            failures.append(f"{key}_regression")
    return not failures, failures


def _candidate_rank(value: dict[str, Any]) -> tuple[float, ...]:
    aggregate = value["train_aggregate"]
    return (
        float(aggregate["mean_horizontal_wgs84_p95_m"]),
        float(aggregate["mean_horizontal_wgs84_p50_m"]),
        float(aggregate["mean_vertical_p95_abs_m"]),
        float(aggregate["mean_kaggle_diagnostic_m"]),
        str(value["candidate"]["id"]),
    )


def _compare_metrics(
    candidate: dict[str, Any], references: dict[str, dict[str, Any]]
) -> dict[str, Any]:
    failures: dict[str, list[str]] = {}
    for name, reference in references.items():
        current_failures: list[str] = []
        if float(candidate["availability_ratio"]) < float(reference["availability_ratio"]) - 1e-12:
            current_failures.append("availability_regression")
        for path, label in (
            (("horizontal_wgs84_m", "p50_m"), "h_median_regression"),
            (("horizontal_wgs84_m", "p95_m"), "h_p95_regression"),
            (("vertical_p95_abs_m",), "v_p95_regression"),
        ):
            if _metric(candidate, path) > _metric(reference, path) + 1e-12:
                current_failures.append(label)
        for key in _DIAGNOSTIC_KEYS:
            if _metric(candidate, ("kaggle_diagnostic_score_variants_m", key)) > _metric(
                reference, ("kaggle_diagnostic_score_variants_m", key)
            ) + 1e-12:
                current_failures.append(f"{key}_regression")
        failures[name] = current_failures
    return {
        "non_regression_passed": all(not values for values in failures.values()),
        "failures_by_reference": failures,
    }


def _speed(previous: smoother.SmoothedRow | None, current: smoother.SmoothedRow) -> float | None:
    if previous is None:
        return None
    dt = (current.timestamp_ms - previous.timestamp_ms) / 1000.0
    if dt <= 0.0:
        return None
    displacement = current.ecef - previous.ecef
    value = float(math.sqrt(float(displacement @ displacement)) / dt)
    return value if math.isfinite(value) else None


def _diagnostics(
    output_dir: Path,
    dataset_id: str,
    positions: list[smoother.PositionRow],
    epochs: list[int],
    baseline_rows: list[smoother.SmoothedRow],
    truth: dict[int, tuple[float, float, float]],
) -> dict[str, Any]:
    """Emit full post-hoc error/reject/innovation/speed data for one route."""

    raw_by_timestamp = {row.timestamp_ms: row for row in positions}
    rows: list[dict[str, Any]] = []
    reject_runs: list[dict[str, Any]] = []
    current_run: list[int] = []
    consecutive_rejects = 0
    prediction_duration_s = 0.0
    speeds: list[float] = []
    innovations: list[float] = []
    previous: smoother.SmoothedRow | None = None

    def finish_run() -> None:
        nonlocal current_run
        if not current_run:
            return
        run_rows = [rows[index] for index in current_run]
        reject_runs.append(
            {
                "start_index": current_run[0],
                "end_index": current_run[-1],
                "length_epochs": len(current_run),
                "duration_s": (baseline_rows[current_run[-1]].timestamp_ms - baseline_rows[current_run[0]].timestamp_ms) / 1000.0,
                "max_innovation_sigma": max(
                    (float(row["innovation_sigma"]) for row in run_rows if row["innovation_sigma"] is not None),
                    default=None,
                ),
                "max_speed_mps": max(
                    (float(row["baseline_speed_mps"]) for row in run_rows if row["baseline_speed_mps"] is not None),
                    default=None,
                ),
                "max_baseline_horizontal_error_m": max(
                    (float(row["baseline_horizontal_error_m"]) for row in run_rows if row["baseline_horizontal_error_m"] is not None),
                    default=None,
                ),
                "max_raw_horizontal_error_m": max(
                    (float(row["raw_horizontal_error_m"]) for row in run_rows if row["raw_horizontal_error_m"] is not None),
                    default=None,
                ),
                "max_baseline_minus_raw_horizontal_m": max(
                    (float(row["baseline_minus_raw_horizontal_m"]) for row in run_rows if row["baseline_minus_raw_horizontal_m"] is not None),
                    default=None,
                ),
            }
        )
        current_run = []

    fields = (
        "index",
        "timestamp_ms",
        "raw_horizontal_error_m",
        "raw_vertical_error_m",
        "baseline_horizontal_error_m",
        "baseline_vertical_error_m",
        "baseline_minus_raw_horizontal_m",
        "baseline_speed_mps",
        "innovation_sigma",
        "outlier_rejected",
        "source",
        "consecutive_rejects",
        "prediction_duration_s",
    )
    for index, (timestamp, baseline_row) in enumerate(zip(epochs, baseline_rows)):
        reference = smoother_eval._match_truth(timestamp, truth, 100)
        raw_row = raw_by_timestamp.get(timestamp)
        raw_h = raw_v = None
        baseline_h = baseline_v = None
        if reference is not None:
            truth_lat, truth_lon, truth_height = reference
            if raw_row is not None:
                raw_h = kaggle._wgs84_horizontal_distance_m(
                    raw_row.latitude, raw_row.longitude, truth_lat, truth_lon
                )
                raw_v = abs(raw_row.height - truth_height)
            baseline_h = kaggle._wgs84_horizontal_distance_m(
                baseline_row.latitude,
                baseline_row.longitude,
                truth_lat,
                truth_lon,
            )
            baseline_v = abs(baseline_row.height - truth_height)
        current_speed = _speed(previous, baseline_row)
        if current_speed is not None:
            speeds.append(current_speed)
        if baseline_row.innovation_sigma is not None:
            innovations.append(float(baseline_row.innovation_sigma))
        if baseline_row.outlier_rejected:
            consecutive_rejects += 1
            if index:
                prediction_duration_s += (timestamp - epochs[index - 1]) / 1000.0
            current_run.append(index)
        else:
            finish_run()
            consecutive_rejects = 0
            prediction_duration_s = 0.0
        row = {
            "index": index,
            "timestamp_ms": timestamp,
            "raw_horizontal_error_m": raw_h,
            "raw_vertical_error_m": raw_v,
            "baseline_horizontal_error_m": baseline_h,
            "baseline_vertical_error_m": baseline_v,
            "baseline_minus_raw_horizontal_m": (
                kaggle._wgs84_horizontal_distance_m(
                    baseline_row.latitude,
                    baseline_row.longitude,
                    raw_row.latitude,
                    raw_row.longitude,
                )
                if raw_row is not None
                else None
            ),
            "baseline_speed_mps": current_speed,
            "innovation_sigma": baseline_row.innovation_sigma,
            "outlier_rejected": int(baseline_row.outlier_rejected),
            "source": baseline_row.source,
            "consecutive_rejects": consecutive_rejects,
            "prediction_duration_s": prediction_duration_s,
        }
        rows.append(row)
        previous = baseline_row
    finish_run()

    output_route = output_dir / "routes" / _safe_id(dataset_id)
    output_route.mkdir(parents=True, exist_ok=True)
    csv_path = output_route / "trajectory_diagnostics.csv"
    _atomic_csv(csv_path, fields, rows)
    raw_h = [float(row["raw_horizontal_error_m"]) for row in rows if row["raw_horizontal_error_m"] is not None]
    baseline_h = [float(row["baseline_horizontal_error_m"]) for row in rows if row["baseline_horizontal_error_m"] is not None]
    rejected_h = [float(row["baseline_horizontal_error_m"]) for row in rows if row["outlier_rejected"] and row["baseline_horizontal_error_m"] is not None]
    accepted_h = [float(row["baseline_horizontal_error_m"]) for row in rows if not row["outlier_rejected"] and row["baseline_horizontal_error_m"] is not None]
    top_rows = sorted(
        (
            row
            for row in rows
            if row["baseline_horizontal_error_m"] is not None
        ),
        key=lambda row: float(row["baseline_horizontal_error_m"]),
        reverse=True,
    )[:10]
    top_runs = sorted(
        reject_runs,
        key=lambda run: (
            float(run["max_baseline_horizontal_error_m"] or -1.0),
            int(run["length_epochs"]),
        ),
        reverse=True,
    )[:10]
    percentile = kaggle._percentile_linear_n_minus_1
    report = {
        "dataset_id": dataset_id,
        "artifacts": {
            "trajectory_diagnostics_csv": {
                "path": str(csv_path),
                "sha256": _sha256(csv_path),
                "rows": len(rows),
            }
        },
        "population": {
            "device_epochs": len(epochs),
            "raw_position_epochs": len(positions),
            "truth_matched_epochs": len(baseline_h),
            "rejected_epochs": sum(int(row["outlier_rejected"]) for row in rows),
            "reject_run_count": len(reject_runs),
            "maximum_reject_run_epochs": max(
                (int(run["length_epochs"]) for run in reject_runs), default=0
            ),
            "maximum_reject_run_duration_s": max(
                (float(run["duration_s"]) for run in reject_runs), default=0.0
            ),
        },
        "raw_error": {
            "horizontal_p50_m": percentile(raw_h, 0.50) if raw_h else None,
            "horizontal_p95_m": percentile(raw_h, 0.95) if raw_h else None,
            "horizontal_max_m": max(raw_h) if raw_h else None,
        },
        "baseline_smoother_error": {
            "horizontal_p50_m": percentile(baseline_h, 0.50) if baseline_h else None,
            "horizontal_p95_m": percentile(baseline_h, 0.95) if baseline_h else None,
            "horizontal_max_m": max(baseline_h) if baseline_h else None,
            "rejected_horizontal_p95_m": percentile(rejected_h, 0.95) if rejected_h else None,
            "rejected_horizontal_max_m": max(rejected_h) if rejected_h else None,
            "accepted_horizontal_p95_m": percentile(accepted_h, 0.95) if accepted_h else None,
            "accepted_horizontal_max_m": max(accepted_h) if accepted_h else None,
        },
        "speed": {
            "p50_mps": percentile(speeds, 0.50) if speeds else None,
            "p95_mps": percentile(speeds, 0.95) if speeds else None,
            "max_mps": max(speeds) if speeds else None,
        },
        "innovation": {
            "p50_sigma": percentile(innovations, 0.50) if innovations else None,
            "p95_sigma": percentile(innovations, 0.95) if innovations else None,
            "max_sigma": max(innovations) if innovations else None,
            "gate_sigma": BASELINE_CONFIG["outlier_gate_sigma"],
        },
        "top_reject_runs": top_runs,
        "top_error_epochs": top_rows,
        "drift_evidence": {
            "prediction_drift_supported": bool(
                rejected_h
                and raw_h
                and max(rejected_h) > max(raw_h)
                and (percentile(baseline_h, 0.95) or 0.0)
                > (percentile(raw_h, 0.95) or 0.0)
                and any(
                    row["outlier_rejected"]
                    and row["baseline_horizontal_error_m"] == max(baseline_h)
                    for row in rows
                    if row["baseline_horizontal_error_m"] is not None
                )
            ),
            "interpretation": "The raw observation remains near the reference while the existing smoother's largest errors are rejected/predicted rows inside long gate-reject runs; this is consistent with prediction drift after the outlier gate.",
        },
    }
    _atomic_json(output_route / "trajectory_diagnostics.json", report)
    report["artifacts"]["trajectory_diagnostics_json"] = {
        "path": str(output_route / "trajectory_diagnostics.json"),
        "sha256": _sha256(output_route / "trajectory_diagnostics.json"),
    }
    return report


def _run_filter_routes(
    route_specs: list[dict[str, Any]],
    generalization_root: Path,
    output_dir: Path,
    *,
    main_position: Path,
    main_device: Path,
    main_truth: Path,
) -> dict[str, dict[str, Any]]:
    """Run all truth-free lanes before opening any ground truth."""

    results: dict[str, dict[str, Any]] = {}
    for spec in route_specs:
        dataset_id = str(spec["dataset_id"])
        inputs = _route_inputs(
            dataset_id,
            generalization_root,
            main_position=main_position,
            main_device=main_device,
            main_truth=main_truth,
        )
        position_path = Path(inputs["position"])
        device_path = Path(inputs["device"])
        for path, label in (
            (position_path, "position"),
            (device_path, "device GNSS"),
        ):
            if not path.is_file():
                raise ReacquisitionError(f"missing {label} for {dataset_id}: {path}")
        positions = smoother._read_positions(position_path, 18)
        epochs = smoother._read_device_epochs(device_path, 1)
        route_output = output_dir / "routes" / _safe_id(dataset_id)
        baseline_result = smoother.smooth_positions(
            positions,
            epochs,
            smoother.SmootherConfig(**BASELINE_CONFIG),
        )
        baseline_dir = route_output / "existing-smoother"
        smoother.write_outputs(
            baseline_result,
            smoother.SmootherConfig(**BASELINE_CONFIG),
            baseline_dir,
            position_path=position_path,
            device_path=device_path,
            skip_epochs=1,
            leap_seconds=18,
        )
        candidate_results: dict[str, smoother.SmoothingResult] = {}
        for candidate in CANDIDATES:
            config = smoother.SmootherConfig(
                BASELINE_CONFIG["process_noise"],
                BASELINE_CONFIG["measurement_floor_m"],
                BASELINE_CONFIG["outlier_gate_sigma"],
                BASELINE_CONFIG["segment_gap_s"],
                int(candidate["max_consecutive_rejects"]),
                float(candidate["max_prediction_duration_s"]),
            )
            result = smoother.smooth_positions(positions, epochs, config)
            candidate_results[str(candidate["id"])] = result
            smoother.write_outputs(
                result,
                config,
                route_output / str(candidate["id"]),
                position_path=position_path,
                device_path=device_path,
                skip_epochs=1,
                leap_seconds=18,
            )
        results[dataset_id] = {
            "dataset_id": dataset_id,
            "position": position_path,
            "device": device_path,
            "truth": Path(inputs["truth"]),
            "positions": positions,
            "epochs": epochs,
            "baseline": baseline_result,
            "candidates": candidate_results,
            "input_hashes": {
                "position": _sha256(position_path),
                "device": _sha256(device_path),
            },
        }
    return results


def _score_group(
    route_results: dict[str, dict[str, Any]],
    route_ids: tuple[str, ...],
    *,
    candidate_id: str | None = None,
) -> dict[str, dict[str, Any]]:
    scored: dict[str, dict[str, Any]] = {}
    for dataset_id in route_ids:
        route = route_results[dataset_id]
        truth = smoother_eval._read_truth(Path(route["truth"]))
        route["input_hashes"]["truth"] = _sha256(Path(route["truth"]))
        if candidate_id is None:
            rows = _raw_rows(route["positions"])
        elif candidate_id == "existing-smoother":
            rows = route["baseline"].rows
        else:
            rows = route["candidates"][candidate_id].rows
        scored[dataset_id] = _score(rows, route["positions"], route["epochs"], truth)
    return scored


def _candidate_report(
    route_results: dict[str, dict[str, Any]],
    train_scores: dict[str, dict[str, dict[str, Any]]],
    validation_scores: dict[str, dict[str, Any]],
    validation_raw: dict[str, dict[str, Any]],
    validation_existing: dict[str, dict[str, Any]],
    main_scores: dict[str, dict[str, Any]],
    main_existing: dict[str, dict[str, Any]],
) -> tuple[dict[str, Any], str | None]:
    existing_train = _aggregate(
        [train_scores["existing-smoother"][dataset_id] for dataset_id in TRAIN_IDS]
    )
    candidate_values: dict[str, Any] = {}
    for candidate in CANDIDATES:
        candidate_id = str(candidate["id"])
        train_aggregate = _aggregate(
            [train_scores[candidate_id][dataset_id] for dataset_id in TRAIN_IDS]
        )
        eligible, failures = _aggregate_non_regression(train_aggregate, existing_train)
        candidate_values[candidate_id] = {
            "candidate": candidate,
            "train_aggregate": train_aggregate,
            "train_non_regression_vs_existing": eligible,
            "train_non_regression_failures": failures,
            "validation": {
                dataset_id: validation_scores[candidate_id][dataset_id]
                for dataset_id in VALIDATION_IDS
            },
            "main_route": {
                dataset_id: main_scores[candidate_id][dataset_id]
                for dataset_id in (MAIN_ID,)
            },
        }
    eligible_values = [value for value in candidate_values.values() if value["train_non_regression_vs_existing"]]
    selected = min(eligible_values, key=_candidate_rank) if eligible_values else None
    selected_id = str(selected["candidate"]["id"]) if selected else None
    validation_gate = None
    main_gate = None
    if selected_id is not None:
        validation_reference = {
            "raw_spp_hatch": validation_raw[VALIDATION_IDS[0]],
            "existing_smoother": validation_existing[VALIDATION_IDS[0]],
        }
        validation_gate = _compare_metrics(
            validation_scores[selected_id][VALIDATION_IDS[0]], validation_reference
        )
        validation_gate["strict_h_p95_improvement_vs_existing"] = (
            _metric(validation_scores[selected_id][VALIDATION_IDS[0]], ("horizontal_wgs84_m", "p95_m"))
            < _metric(validation_existing[VALIDATION_IDS[0]], ("horizontal_wgs84_m", "p95_m")) - 1e-12
        )
        main_gate = _compare_metrics(
            main_scores[selected_id][MAIN_ID],
            {"existing_smoother": main_existing[MAIN_ID]},
        )
        main_gate["strict_h_p95_improvement_vs_existing"] = (
            _metric(main_scores[selected_id][MAIN_ID], ("horizontal_wgs84_m", "p95_m"))
            < _metric(main_existing[MAIN_ID], ("horizontal_wgs84_m", "p95_m")) - 1e-12
        )
    promotion = "no-go-no-eligible-train-candidate"
    if selected_id is not None:
        if not validation_gate["non_regression_passed"]:
            promotion = "no-go-validation-regression"
        elif not validation_gate["strict_h_p95_improvement_vs_existing"]:
            promotion = "no-go-validation-no-p95-improvement"
        elif not main_gate["non_regression_passed"]:
            promotion = "no-go-development-main-regression"
        else:
            promotion = "promote-development-only"
    return (
        {
            "existing_train_aggregate": existing_train,
            "candidates": candidate_values,
            "selected_candidate_id": selected_id,
            "validation_gate": validation_gate,
            "development_main_gate": main_gate,
            "promotion_decision": promotion,
            "existing_smoother_retained": promotion != "promote-development-only",
        },
        selected_id,
    )


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        prog=os.environ.get("GNSS_CLI_NAME", "gnss smartphone-reacquisition-eval")
    )
    parser.add_argument("--generalization-root", type=Path, default=DEFAULT_GENERALIZATION_ROOT)
    parser.add_argument("--profile", type=Path, default=DEFAULT_PROFILE)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--main-position", type=Path, default=DEFAULT_MAIN_POSITION)
    parser.add_argument("--main-device-gnss", type=Path, default=DEFAULT_MAIN_DEVICE)
    parser.add_argument("--main-ground-truth", type=Path, default=DEFAULT_MAIN_TRUTH)
    return parser.parse_args(argv)


def run(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    try:
        _load_profile(args.profile)
        all_specs = [
            {"dataset_id": dataset_id, "role": "candidate-train"}
            for dataset_id in TRAIN_IDS
        ] + [
            {"dataset_id": dataset_id, "role": "validation"}
            for dataset_id in VALIDATION_IDS
        ] + [{"dataset_id": MAIN_ID, "role": "development-main-regression"}]
        started = time.perf_counter()
        route_results = _run_filter_routes(
            all_specs,
            args.generalization_root,
            args.output_dir,
            main_position=args.main_position,
            main_device=args.main_device_gnss,
            main_truth=args.main_ground_truth,
        )
        # Training labels are opened only after all route filter artifacts have
        # been written.  Validation/main labels are opened in the same
        # post-filter phase and cannot alter the fixed candidate set.
        train_scores = {
            "raw_spp_hatch": _score_group(route_results, TRAIN_IDS),
            "existing-smoother": _score_group(
                route_results, TRAIN_IDS, candidate_id="existing-smoother"
            ),
        }
        for candidate in CANDIDATES:
            candidate_id = str(candidate["id"])
            train_scores[candidate_id] = _score_group(
                route_results, TRAIN_IDS, candidate_id=candidate_id
            )
        validation_raw = _score_group(route_results, VALIDATION_IDS)
        validation_existing = _score_group(
            route_results, VALIDATION_IDS, candidate_id="existing-smoother"
        )
        validation_scores = {
            str(candidate["id"]): _score_group(
                route_results, VALIDATION_IDS, candidate_id=str(candidate["id"])
            )
            for candidate in CANDIDATES
        }
        main_existing = _score_group(
            route_results, (MAIN_ID,), candidate_id="existing-smoother"
        )
        main_scores = {
            str(candidate["id"]): _score_group(
                route_results, (MAIN_ID,), candidate_id=str(candidate["id"])
            )
            for candidate in CANDIDATES
        }
        route_diagnostics: dict[str, Any] = {}
        for dataset_id in (*VALIDATION_IDS, *TRAIN_IDS, MAIN_ID):
            route = route_results[dataset_id]
            truth = smoother_eval._read_truth(Path(route["truth"]))
            route_diagnostics[dataset_id] = _diagnostics(
                args.output_dir,
                dataset_id,
                route["positions"],
                route["epochs"],
                route["baseline"].rows,
                truth,
            )
        candidate_report, selected_id = _candidate_report(
            route_results,
            train_scores,
            validation_scores,
            validation_raw,
            validation_existing,
            main_scores,
            main_existing,
        )
        report_path = args.output_dir / "reacquisition_report.json"
        manifest_path = args.output_dir / "reacquisition_manifest.json"
        report = {
            "schema_version": SCHEMA_VERSION,
            "decision": "development-only-reacquisition-evaluation",
            "selection_record": "docs/use_cases/records/smartphone_r5_reacquisition_candidates.json",
            "source_artifacts": {
                "profile": {"path": str(args.profile), "sha256": _sha256(args.profile)},
                "generalization_root": str(args.generalization_root),
            },
            "roles": {
                "candidate_train": list(TRAIN_IDS),
                "validation": list(VALIDATION_IDS),
                "development_main_regression": [MAIN_ID],
                "holdout_id": HOLDOUT_ID,
                "holdout_content_opened": False,
                "holdout_truth_opened": False,
                "holdout_materialized": False,
            },
            "evaluation_contract": {
                "truth_free_filter": True,
                "raw_lane": "canonical Galileo E1/Hatch30 SPP POS",
                "existing_smoother": BASELINE_CONFIG,
                "candidate_set": list(CANDIDATES),
                "imu_adaptive_status": "No-Go retained; not evaluated or changed by this experiment",
            },
            "routes": {
                dataset_id: {
                    "role": next(spec["role"] for spec in all_specs if spec["dataset_id"] == dataset_id),
                    "inputs": {
                        "position": str(route_results[dataset_id]["position"]),
                        "device_gnss": str(route_results[dataset_id]["device"]),
                        "ground_truth": str(route_results[dataset_id]["truth"]),
                        "sha256": route_results[dataset_id]["input_hashes"],
                    },
                    "diagnostics": route_diagnostics[dataset_id],
                    "baseline_reacquisition": {
                        "reacquisition_count": len(route_results[dataset_id]["baseline"].reacquisition_indices),
                        "maximum_observed_reject_run": route_results[dataset_id]["baseline"].max_consecutive_rejects,
                        "maximum_observed_prediction_duration_s": route_results[dataset_id]["baseline"].max_prediction_duration_s,
                    },
                }
                for dataset_id in (*TRAIN_IDS, *VALIDATION_IDS, MAIN_ID)
            },
            "scores": {
                "train": train_scores,
                "validation_raw_spp_hatch": validation_raw,
                "validation_existing_smoother": validation_existing,
                "validation_candidates": validation_scores,
                "development_main_existing_smoother": main_existing,
                "development_main_candidates": main_scores,
            },
            "selection": candidate_report,
            "timing": {"total_filter_and_report_wall_s": time.perf_counter() - started},
            "artifact_manifest": str(manifest_path),
        }
        args.output_dir.mkdir(parents=True, exist_ok=True)
        _atomic_json(report_path, report)
        artifact_manifest = {
            "schema_version": "smartphone-r5-reacquisition-manifest.v1",
            "report": {"path": str(report_path), "sha256": _sha256(report_path)},
            "route_diagnostics": {
                dataset_id: route_diagnostics[dataset_id]["artifacts"]
                for dataset_id in (*TRAIN_IDS, *VALIDATION_IDS, MAIN_ID)
            },
            "candidate_smoother_manifests": {
                dataset_id: {
                    candidate_id: {
                        "path": str(
                            args.output_dir
                            / "routes"
                            / _safe_id(dataset_id)
                            / candidate_id
                            / "smoother_manifest.json"
                        ),
                        "sha256": _sha256(
                            args.output_dir
                            / "routes"
                            / _safe_id(dataset_id)
                            / candidate_id
                            / "smoother_manifest.json"
                        ),
                    }
                    for candidate_id in (str(candidate["id"]) for candidate in CANDIDATES)
                }
                for dataset_id in (*TRAIN_IDS, *VALIDATION_IDS, MAIN_ID)
            },
            "holdout_content_opened": False,
            "holdout_truth_opened": False,
        }
        _atomic_json(manifest_path, artifact_manifest)
        print(
            f"Smartphone reacquisition evaluation complete: {report_path}"
        )
        return 0
    except (OSError, ReacquisitionError, smoother.SmootherError, ValueError, KeyError, TypeError, json.JSONDecodeError) as exc:
        print(f"Smartphone reacquisition evaluation failed: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(run())
