#!/usr/bin/env python3
"""Truth-scoring gate for the frozen causal IMU motion-Q alternative.

The smoother and IMU feature builder never receive truth.  This evaluator
opens truth only after each truth-free route artifact and its manifest have
been verified.  It intentionally supports only the route-disjoint train or
fresh-validation roles from the frozen selection record; holdout is sealed.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import sys
import time
from typing import Any

from support.gnss_runtime import application_root


ROOT = application_root(__file__)
BENCHMARK_DIR = ROOT / "apps" / "commands" / "benchmarks"
if str(BENCHMARK_DIR) not in sys.path:
    sys.path.insert(0, str(BENCHMARK_DIR))
import gnss_smartphone_kaggle as kaggle  # noqa: E402
import gnss_smartphone_trajectory_smoother as smoother  # noqa: E402
import gnss_smartphone_trajectory_smoother_eval as smoother_eval  # noqa: E402


SCHEMA_VERSION = "smartphone-r5-gsdc2023-imu-motion-q-evaluation.v1"
SELECTION_SCHEMA = "smartphone-r5-gsdc2023-imu-motion-q-selection.v1"
SMOOTHER_MANIFEST_SCHEMA = "smartphone-trajectory-smoother-manifest.v1"
MATCH_TOLERANCE_MS = 100
TRAIN_IDS = (
    "2021-01-04-21-50-us-ca-e1highway280driveroutea/mi8",
    "2022-08-04-20-07-us-ca-sjc-q/pixel5",
    "2023-05-24-20-26-us-ca-sjc-ge2/pixel7pro",
)
FRESH_VALIDATION_ID = "2023-09-07-18-59-us-ca/pixel5"
FUTURE_HOLDOUT_ID = "2023-09-06-00-01-us-ca-routen/pixel6pro"
FORBIDDEN_IDS = {
    "2023-09-06-22-49-us-ca-routebb1/pixel7pro",
    FUTURE_HOLDOUT_ID,
    "2022-10-06-21-51-us-ca-mtv-n/sm-a205u",
    "2022-10-06-21-51-us-ca-mtv-n/sm-a217m",
    "2022-10-06-21-51-us-ca-mtv-n/sm-a325f",
}
DIAGNOSTIC_KEYS = (
    "haversine_sphere__linear_n_minus_1",
    "haversine_sphere__nearest_rank_ceiling",
    "wgs84_vincenty__linear_n_minus_1",
    "wgs84_vincenty__nearest_rank_ceiling",
)
EXPECTED_PARAMETERS = {
    "smoother_process_noise": 1.0,
    "measurement_floor_m": 1.0,
    "outlier_gate_sigma": 5.0,
    "segment_gap_s": 10.0,
    "motion_window_s": 1.0,
    "motion_max_sample_gap_s": 0.25,
    "motion_max_sample_age_s": 0.25,
    "motion_gyro_threshold": 0.25,
    "motion_accel_dynamic_threshold": 1.0,
    "motion_q_multiplier": 2.0,
    "skip_epochs": 1,
}


class ImuMotionEvaluationError(ValueError):
    """Raised when the frozen evaluation or artifact contract is invalid."""


def _sha256(path: Path) -> str:
    if not path.is_file():
        raise ImuMotionEvaluationError(f"missing artifact: {path}")
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _atomic_json(path: Path, payload: dict[str, Any]) -> None:
    smoother._atomic_write(
        path, (json.dumps(payload, indent=2, sort_keys=True) + "\n").encode("utf-8")
    )


def _load_json(path: Path, label: str) -> dict[str, Any]:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ImuMotionEvaluationError(f"invalid {label}: {path}") from exc
    if not isinstance(payload, dict):
        raise ImuMotionEvaluationError(f"{label} must be a JSON object")
    return payload


def _load_selection(path: Path) -> dict[str, Any]:
    selection = _load_json(path, "selection record")
    if selection.get("schema_version") != SELECTION_SCHEMA:
        raise ImuMotionEvaluationError("selection record schema is invalid")
    if selection.get("status") != "selection-frozen-before-new-truth-evaluation":
        raise ImuMotionEvaluationError("selection record is not frozen")
    candidate = selection.get("candidate")
    if not isinstance(candidate, dict) or candidate.get("id") != "causal_imu_motion_adaptive_q_v1":
        raise ImuMotionEvaluationError("selection candidate differs from frozen IMU candidate")
    parameters = candidate.get("parameters")
    if not isinstance(parameters, dict):
        raise ImuMotionEvaluationError("selection record lacks candidate parameters")
    for name, expected in EXPECTED_PARAMETERS.items():
        value = parameters.get(name)
        if isinstance(expected, float):
            if not isinstance(value, (int, float)) or isinstance(value, bool) or float(value) != expected:
                raise ImuMotionEvaluationError(f"candidate parameter differs: {name}")
        elif value != expected:
            raise ImuMotionEvaluationError(f"candidate parameter differs: {name}")
    split = selection.get("frozen_split")
    if not isinstance(split, dict) or split.get("future_holdout") != FUTURE_HOLDOUT_ID:
        raise ImuMotionEvaluationError("selection future holdout differs from frozen contract")
    if split.get("fresh_validation") != FRESH_VALIDATION_ID:
        raise ImuMotionEvaluationError("selection fresh validation differs from frozen contract")
    return selection


def _parse_route_spec(raw: str) -> tuple[str, Path, Path, Path, Path, Path]:
    parts = raw.split("|", 5)
    if len(parts) != 6 or not all(parts):
        raise ImuMotionEvaluationError(
            "route spec must be dataset_id|baseline.pos|candidate.smoothed.pos|device_gnss.csv|device_imu.csv|ground_truth.csv"
        )
    dataset_id, baseline, candidate, device, imu, truth = parts
    if dataset_id in FORBIDDEN_IDS or dataset_id == FUTURE_HOLDOUT_ID:
        raise ImuMotionEvaluationError(f"sealed holdout route is forbidden: {dataset_id}")
    return dataset_id, Path(baseline), Path(candidate), Path(device), Path(imu), Path(truth)


def _position_rows(
    positions: list[smoother.PositionRow], source: str
) -> list[smoother.SmoothedRow]:
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
            source=source,
            segment_id=0,
            measurement_used=True,
            outlier_rejected=False,
            innovation_sigma=None,
            position_sigma_m=smoother._measurement_sigma(row, 1.0),
        )
        for row in positions
    ]


def _baseline_population(
    positions: list[smoother.PositionRow], device_epochs: list[int]
) -> list[smoother.SmoothedRow]:
    by_timestamp = {row.timestamp_ms: row for row in _position_rows(positions, "measured")}
    rows: list[smoother.SmoothedRow] = []
    for timestamp in device_epochs:
        row = by_timestamp.get(timestamp)
        if row is not None:
            rows.append(row)
            continue
        week, tow = smoother._device_time_to_week_tow(
            timestamp, smoother.DEFAULT_GPS_UTC_LEAP_SECONDS
        )
        rows.append(
            smoother.SmoothedRow(
                timestamp_ms=timestamp,
                week=week,
                tow=tow,
                ecef=smoother.np.zeros(3),
                latitude=0.0,
                longitude=0.0,
                height=0.0,
                status=smoother.PROPAGATED_STATUS,
                satellites=0,
                pdop=0.0,
                ratio=0.0,
                fixed_ambiguities=0,
                iterations=0,
                source="missing",
                segment_id=0,
                measurement_used=False,
                outlier_rejected=False,
                innovation_sigma=None,
                position_sigma_m=0.0,
            )
        )
    return rows


def _distance_arrays(
    rows: list[smoother.SmoothedRow],
    device_epochs: list[int],
    truth: dict[int, tuple[float, float, float]],
) -> tuple[list[float], list[float], list[float]]:
    by_timestamp = {row.timestamp_ms: row for row in rows}
    wgs84: list[float] = []
    haversine: list[float] = []
    vertical: list[float] = []
    for timestamp in device_epochs:
        row = by_timestamp.get(timestamp)
        if row is None or row.source == "missing":
            continue
        reference = smoother_eval._match_truth(timestamp, truth, MATCH_TOLERANCE_MS)
        if reference is None:
            continue
        latitude, longitude, height = reference
        wgs84.append(kaggle._wgs84_horizontal_distance_m(row.latitude, row.longitude, latitude, longitude))
        haversine.append(kaggle._haversine_horizontal_distance_m(row.latitude, row.longitude, latitude, longitude))
        vertical.append(abs(row.height - height))
    return wgs84, haversine, vertical


def _percentile(values: list[float], fraction: float) -> float | None:
    return kaggle._percentile_linear_n_minus_1(values, fraction) if values else None


def _metrics(
    rows: list[smoother.SmoothedRow],
    device_epochs: list[int],
    truth: dict[int, tuple[float, float, float]],
) -> tuple[dict[str, Any], tuple[list[float], list[float], list[float]]]:
    arrays = _distance_arrays(rows, device_epochs, truth)
    wgs84, haversine, vertical = arrays
    if not wgs84:
        raise ImuMotionEvaluationError("no candidate epochs matched ground truth")
    wgs_p50, wgs_p95 = _percentile(wgs84, 0.50), _percentile(wgs84, 0.95)
    hav_p50, hav_p95 = _percentile(haversine, 0.50), _percentile(haversine, 0.95)
    diagnostics = {
        "wgs84_vincenty__linear_n_minus_1": (wgs_p50 + wgs_p95) / 2.0,
        "wgs84_vincenty__nearest_rank_ceiling": (
            kaggle._percentile_nearest_rank_ceiling(wgs84, 0.50)
            + kaggle._percentile_nearest_rank_ceiling(wgs84, 0.95)
        )
        / 2.0,
        "haversine_sphere__linear_n_minus_1": (hav_p50 + hav_p95) / 2.0,
        "haversine_sphere__nearest_rank_ceiling": (
            kaggle._percentile_nearest_rank_ceiling(haversine, 0.50)
            + kaggle._percentile_nearest_rank_ceiling(haversine, 0.95)
        )
        / 2.0,
    }
    output = {
        "device_epochs": len(device_epochs),
        "position_epochs": len(rows),
        "matched_epochs": len(wgs84),
        "availability_ratio": len(rows) / len(device_epochs),
        "truth_coverage_ratio": len(wgs84) / len(device_epochs),
        "horizontal_wgs84_m": {"p50_m": wgs_p50, "p95_m": wgs_p95},
        "horizontal_haversine_m": {"p50_m": hav_p50, "p95_m": hav_p95},
        "vertical_p95_abs_m": _percentile(vertical, 0.95),
        "kaggle_diagnostic_score_variants_m": diagnostics,
        "kaggle_diagnostic_mean_m": sum(diagnostics.values()) / len(diagnostics),
    }
    return output, arrays


def _non_regression(candidate: dict[str, Any], baseline: dict[str, Any]) -> list[str]:
    failures: list[str] = []
    comparisons = (
        (("availability_ratio",), "availability_regression"),
        (("truth_coverage_ratio",), "coverage_regression"),
        (("horizontal_wgs84_m", "p50_m"), "h_p50_regression"),
        (("horizontal_wgs84_m", "p95_m"), "h_p95_regression"),
        (("vertical_p95_abs_m",), "v_p95_regression"),
    )
    for path, label in comparisons:
        left: Any = candidate
        right: Any = baseline
        for key in path:
            left, right = left[key], right[key]
        if float(left) > float(right) and not math.isclose(float(left), float(right), rel_tol=0.0, abs_tol=1.0e-9):
            failures.append(label)
    for key in DIAGNOSTIC_KEYS:
        left = candidate["kaggle_diagnostic_score_variants_m"][key]
        right = baseline["kaggle_diagnostic_score_variants_m"][key]
        if left > right and not math.isclose(left, right, rel_tol=0.0, abs_tol=1.0e-9):
            failures.append(key + "_regression")
    return sorted(set(failures))


def _strict_train_gate(candidate: dict[str, Any], baseline: dict[str, Any]) -> list[str]:
    failures = _non_regression(candidate, baseline)
    if not candidate["horizontal_wgs84_m"]["p95_m"] < baseline["horizontal_wgs84_m"]["p95_m"] - 1.0e-9:
        failures.append("aggregate_h_p95_not_strictly_improved")
    if not candidate["kaggle_diagnostic_mean_m"] < baseline["kaggle_diagnostic_mean_m"] - 1.0e-9:
        failures.append("aggregate_diagnostic_mean_not_strictly_improved")
    return sorted(set(failures))


def _verify_candidate_manifest(
    candidate_path: Path,
    device_path: Path,
    imu_path: Path,
) -> dict[str, Any]:
    manifest_path = candidate_path.with_name("smoother_manifest.json")
    manifest = _load_json(manifest_path, "candidate smoother manifest")
    if manifest.get("schema_version") != SMOOTHER_MANIFEST_SCHEMA:
        raise ImuMotionEvaluationError("candidate smoother manifest schema is invalid")
    if manifest.get("truth_used") is not False:
        raise ImuMotionEvaluationError("candidate smoother manifest is not truth-free")
    if manifest.get("motion_experimental_flag") != "--experimental-motion-adaptive-q":
        raise ImuMotionEvaluationError("candidate manifest lacks motion-adaptive opt-in")
    inputs = manifest.get("inputs")
    if not isinstance(inputs, dict):
        raise ImuMotionEvaluationError("candidate manifest lacks inputs")
    if inputs.get("device_gnss", {}).get("sha256") != _sha256(device_path):
        raise ImuMotionEvaluationError("candidate device GNSS hash mismatch")
    artifact = manifest.get("artifacts", {}).get("smoothed_pos")
    if not isinstance(artifact, dict) or artifact.get("sha256") != _sha256(candidate_path):
        raise ImuMotionEvaluationError("candidate smoothed position hash mismatch")
    motion = manifest.get("motion_adaptive_process_noise")
    if not isinstance(motion, dict) or motion.get("input_audit", {}).get("sha256") != _sha256(imu_path):
        raise ImuMotionEvaluationError("candidate IMU hash mismatch")
    algorithm = motion.get("algorithm", {})
    expected = {
        "window_s": 1.0,
        "max_sample_gap_s": 0.25,
        "max_sample_age_s": 0.25,
        "gyro_threshold": 0.25,
        "accel_dynamic_threshold": 1.0,
        "motion_q_multiplier": 2.0,
        "future_samples_used": False,
        "orientation_estimation": False,
        "absolute_acceleration_integration": False,
    }
    for key, value in expected.items():
        if algorithm.get(key) != value:
            raise ImuMotionEvaluationError(f"candidate motion parameter mismatch: {key}")
    return {
        "path": str(manifest_path),
        "sha256": _sha256(manifest_path),
        "position": {"path": str(candidate_path), "sha256": _sha256(candidate_path)},
        "device_gnss": {"path": str(device_path), "sha256": _sha256(device_path)},
        "device_imu": {"path": str(imu_path), "sha256": _sha256(imu_path)},
    }


def evaluate_route(
    dataset_id: str,
    baseline_path: Path,
    candidate_path: Path,
    device_path: Path,
    imu_path: Path,
    truth_path: Path,
    truth_cache: dict[str, dict[int, tuple[float, float, float]]],
) -> dict[str, Any]:
    candidate_manifest = _verify_candidate_manifest(candidate_path, device_path, imu_path)
    positions = smoother._read_positions(baseline_path, smoother.DEFAULT_GPS_UTC_LEAP_SECONDS)
    candidate_positions = smoother._read_positions(candidate_path, smoother.DEFAULT_GPS_UTC_LEAP_SECONDS)
    device_epochs = smoother._read_device_epochs(device_path, EXPECTED_PARAMETERS["skip_epochs"])
    if not positions or not candidate_positions or not device_epochs:
        raise ImuMotionEvaluationError(f"empty route input: {dataset_id}")
    truth_key = str(truth_path)
    truth = truth_cache.setdefault(truth_key, smoother_eval._read_truth(truth_path))
    baseline_metrics, baseline_arrays = _metrics(_baseline_population(positions, device_epochs), device_epochs, truth)
    candidate_metrics, candidate_arrays = _metrics(_position_rows(candidate_positions, "imu_motion_q"), device_epochs, truth)
    return {
        "dataset_id": dataset_id,
        "baseline": baseline_metrics,
        "candidate": candidate_metrics,
        "gate_failures": _non_regression(candidate_metrics, baseline_metrics),
        "inputs": {
            "baseline": {"path": str(baseline_path), "sha256": _sha256(baseline_path)},
            "candidate": candidate_manifest,
            "ground_truth": {"path": str(truth_path), "sha256": _sha256(truth_path)},
        },
        "_baseline_arrays": baseline_arrays,
        "_candidate_arrays": candidate_arrays,
    }


def _aggregate(routes: list[dict[str, Any]], key: str) -> dict[str, Any]:
    arrays = [[], [], []]
    for route in routes:
        for index, values in enumerate(route[key]):
            arrays[index].extend(values)
    wgs84, haversine, vertical = arrays
    wgs_p50, wgs_p95 = _percentile(wgs84, 0.50), _percentile(wgs84, 0.95)
    hav_p50, hav_p95 = _percentile(haversine, 0.50), _percentile(haversine, 0.95)
    diagnostics = {
        "wgs84_vincenty__linear_n_minus_1": (wgs_p50 + wgs_p95) / 2.0,
        "wgs84_vincenty__nearest_rank_ceiling": (
            kaggle._percentile_nearest_rank_ceiling(wgs84, 0.50)
            + kaggle._percentile_nearest_rank_ceiling(wgs84, 0.95)
        )
        / 2.0,
        "haversine_sphere__linear_n_minus_1": (hav_p50 + hav_p95) / 2.0,
        "haversine_sphere__nearest_rank_ceiling": (
            kaggle._percentile_nearest_rank_ceiling(haversine, 0.50)
            + kaggle._percentile_nearest_rank_ceiling(haversine, 0.95)
        )
        / 2.0,
    }
    total_epochs = sum(route["candidate"]["device_epochs"] for route in routes)
    metric_key = "baseline" if key == "_baseline_arrays" else "candidate"
    position_epochs = sum(route[metric_key]["position_epochs"] for route in routes)
    return {
        "device_epochs": total_epochs,
        "position_epochs": position_epochs,
        "matched_epochs": len(wgs84),
        "availability_ratio": position_epochs / total_epochs,
        "truth_coverage_ratio": len(wgs84) / total_epochs,
        "horizontal_wgs84_m": {"p50_m": wgs_p50, "p95_m": wgs_p95},
        "horizontal_haversine_m": {"p50_m": hav_p50, "p95_m": hav_p95},
        "vertical_p95_abs_m": _percentile(vertical, 0.95),
        "kaggle_diagnostic_score_variants_m": diagnostics,
        "kaggle_diagnostic_mean_m": sum(diagnostics.values()) / len(diagnostics),
    }


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(prog="gnss smartphone-imu-motion-q-eval")
    parser.add_argument("--selection-record", type=Path, required=True)
    parser.add_argument("--route", action="append", required=True)
    parser.add_argument("--role", choices=("train", "fresh-validation"), required=True)
    parser.add_argument("--output-json", type=Path, required=True)
    return parser.parse_args(argv)


def run(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    started = time.perf_counter()
    try:
        selection = _load_selection(args.selection_record)
        specs = [_parse_route_spec(raw) for raw in args.route]
        identifiers = tuple(spec[0] for spec in specs)
        expected = TRAIN_IDS if args.role == "train" else (FRESH_VALIDATION_ID,)
        if identifiers != expected:
            raise ImuMotionEvaluationError(
                f"route IDs must exactly match frozen {args.role} role: {expected}"
            )
        truth_cache: dict[str, dict[int, tuple[float, float, float]]] = {}
        routes = [evaluate_route(*spec, truth_cache) for spec in specs]
        baseline_aggregate = _aggregate(routes, "_baseline_arrays")
        candidate_aggregate = _aggregate(routes, "_candidate_arrays")
        aggregate_failures = (
            _strict_train_gate(candidate_aggregate, baseline_aggregate)
            if args.role == "train"
            else _non_regression(candidate_aggregate, baseline_aggregate)
        )
        if args.role == "fresh-validation" and not candidate_aggregate["horizontal_wgs84_m"]["p95_m"] < baseline_aggregate["horizontal_wgs84_m"]["p95_m"] - 1.0e-9:
            aggregate_failures.append("aggregate_h_p95_not_strictly_improved")
        for route in routes:
            route["gate_failures"] = sorted(set(route["gate_failures"]))
            route.pop("_baseline_arrays", None)
            route.pop("_candidate_arrays", None)
        payload = {
            "schema_version": SCHEMA_VERSION,
            "role": args.role,
            "selection_record": {"path": str(args.selection_record), "sha256": _sha256(args.selection_record)},
            "candidate_id": selection["candidate"]["id"],
            "truth_open_count": len(truth_cache),
            "truth_free_artifacts_preceded_truth": True,
            "public_private_scores_used_for_tuning": False,
            "routes": routes,
            "aggregate": {
                "baseline": baseline_aggregate,
                "candidate": candidate_aggregate,
                "gate_failures": sorted(set(aggregate_failures)),
            },
            "passed": not aggregate_failures and all(not route["gate_failures"] for route in routes),
            "timing": {"wall_time_s": time.perf_counter() - started},
            "fresh_validation_opened": args.role == "fresh-validation",
            "future_holdout_opened": False,
            "production_default_changed": False,
        }
        _atomic_json(args.output_json, payload)
        manifest_path = args.output_json.with_name(args.output_json.stem + "_manifest.json")
        _atomic_json(
            manifest_path,
            {
                "schema_version": SCHEMA_VERSION + ".manifest",
                "record": {"path": str(args.output_json), "sha256": _sha256(args.output_json)},
                "selection_record_sha256": _sha256(args.selection_record),
                "truth_open_count": len(truth_cache),
                "passed": payload["passed"],
                "future_holdout_opened": False,
            },
        )
    except (OSError, ValueError, KeyError, TypeError) as exc:
        print(f"Smartphone IMU motion-Q evaluation failed: {exc}", file=sys.stderr)
        return 1
    print(f"Smartphone IMU motion-Q evaluation: {args.output_json}")
    print(f"Smartphone IMU motion-Q evaluation manifest: {manifest_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(run())
