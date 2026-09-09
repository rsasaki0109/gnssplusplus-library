#!/usr/bin/env python3
"""Development-only selector/evaluator for the truth-free trajectory smoother.

This command is intentionally separate from the smoother.  It may read a
development ground-truth file to score frozen candidates, but it never passes
truth to the filter and never accepts a holdout role.  The selected candidate
is reset at the fixed 830/553 device-epoch boundary before validation.
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
sys.path.insert(0, str(ROOT / "apps" / "commands" / "benchmarks"))
import gnss_smartphone_kaggle as kaggle  # noqa: E402
import gnss_smartphone_trajectory_smoother as smoother  # noqa: E402


SCHEMA_VERSION = "smartphone-trajectory-smoother-evaluation.v1"
DEFAULT_SPLIT_INDEX = 830
DEFAULT_MATCH_TOLERANCE_MS = 100
PREDECLARED_CANDIDATES = (
    ("q0p01_floor0p5", 0.01, 0.5),
    ("q0p01_floor1p0", 0.01, 1.0),
    ("q0p10_floor0p5", 0.10, 0.5),
    ("q0p10_floor1p0", 0.10, 1.0),
    ("q1p00_floor1p0", 1.00, 1.0),
)


def _atomic_json(path: Path, payload: dict[str, Any]) -> None:
    content = (json.dumps(payload, indent=2, sort_keys=True) + "\n").encode("utf-8")
    smoother._atomic_write(path, content)


def _read_truth(path: Path) -> dict[int, tuple[float, float, float]]:
    smoother._require_file(path, "development ground truth")
    truth: dict[int, tuple[float, float, float]] = {}
    try:
        with path.open(encoding="utf-8-sig", newline="") as handle:
            reader = csv.DictReader(handle)
            required = {"UnixTimeMillis", "LatitudeDegrees", "LongitudeDegrees", "AltitudeMeters"}
            missing = required - set(reader.fieldnames or ())
            if missing:
                raise smoother.SmootherError(
                    f"ground truth missing fields: {', '.join(sorted(missing))}"
                )
            for line_number, raw_row in enumerate(reader, start=2):
                try:
                    timestamp = int(raw_row.get("UnixTimeMillis", ""))
                    latitude = float(raw_row.get("LatitudeDegrees", ""))
                    longitude = float(raw_row.get("LongitudeDegrees", ""))
                    height = float(raw_row.get("AltitudeMeters", ""))
                except (TypeError, ValueError) as exc:
                    raise smoother.SmootherError(
                        f"truth row {line_number}: invalid numeric value"
                    ) from exc
                values = (latitude, longitude, height)
                if timestamp < 0 or not all(math.isfinite(value) for value in values):
                    raise smoother.SmootherError(
                        f"truth row {line_number}: non-finite or negative timestamp"
                    )
                if timestamp in truth:
                    raise smoother.SmootherError(
                        f"truth row {line_number}: duplicate timestamp {timestamp}"
                    )
                truth[timestamp] = values
    except OSError as exc:
        raise smoother.SmootherError(f"failed to read development ground truth: {path}") from exc
    if not truth:
        raise smoother.SmootherError("development ground truth is empty")
    return truth


def _match_truth(
    timestamp: int,
    truth: dict[int, tuple[float, float, float]],
    tolerance_ms: int,
) -> tuple[float, float, float] | None:
    if timestamp in truth:
        return truth[timestamp]
    nearest = min(truth, key=lambda candidate: abs(candidate - timestamp))
    return truth[nearest] if abs(nearest - timestamp) <= tolerance_ms else None


def _percentile(values: list[float], fraction: float) -> float | None:
    return kaggle._percentile_linear_n_minus_1(values, fraction) if values else None


def _score_rows(
    rows: list[smoother.SmoothedRow],
    positions_by_timestamp: dict[int, smoother.PositionRow],
    truth: dict[int, tuple[float, float, float]],
    start: int,
    end: int,
    *,
    match_tolerance_ms: int,
) -> dict[str, Any]:
    selected_rows = rows[start:end]
    scored_output_rows = [row for row in selected_rows if row.source != "missing"]
    selected_timestamps = {row.timestamp_ms for row in selected_rows}
    raw_rows = [
        row
        for timestamp, row in positions_by_timestamp.items()
        if timestamp in selected_timestamps
    ]
    matched: list[tuple[smoother.SmoothedRow, tuple[float, float, float]]] = []
    for row in scored_output_rows:
        if row.source == "missing":
            continue
        reference = _match_truth(row.timestamp_ms, truth, match_tolerance_ms)
        if reference is not None:
            matched.append((row, reference))
    horizontal_wgs84: list[float] = []
    horizontal_haversine: list[float] = []
    vertical: list[float] = []
    for row, reference in matched:
        truth_lat, truth_lon, truth_height = reference
        horizontal_wgs84.append(
            kaggle._wgs84_horizontal_distance_m(
                row.latitude, row.longitude, truth_lat, truth_lon
            )
        )
        horizontal_haversine.append(
            kaggle._haversine_horizontal_distance_m(
                row.latitude, row.longitude, truth_lat, truth_lon
            )
        )
        vertical.append(abs(row.height - truth_height))

    def metric_pair(values: list[float]) -> dict[str, float | None]:
        return {
            "p50_m": _percentile(values, 0.50),
            "p95_m": _percentile(values, 0.95),
        }

    wgs84 = metric_pair(horizontal_wgs84)
    haversine = metric_pair(horizontal_haversine)

    def diagnostic_score(metric_values: list[float]) -> float | None:
        """Return a diagnostic pair score, preserving an empty match as null.

        A route with no truth matches is a valid diagnostic result for schema
        purposes, but it is not a zero-error route.  Returning ``None`` for
        every percentile variant lets the caller apply its fail-closed
        aggregation policy instead of accidentally mixing zero and null (or
        adding two null values).
        """

        if not metric_values:
            return None
        p50 = _percentile(metric_values, 0.50)
        p95 = _percentile(metric_values, 0.95)
        if p50 is None or p95 is None:
            return None
        return (p50 + p95) / 2.0

    score_variants = {
        "wgs84_vincenty__linear_n_minus_1": diagnostic_score(horizontal_wgs84),
        "wgs84_vincenty__nearest_rank_ceiling": (
            kaggle._percentile_nearest_rank_ceiling(horizontal_wgs84, 0.50)
            + kaggle._percentile_nearest_rank_ceiling(horizontal_wgs84, 0.95)
        )
        / 2.0
        if horizontal_wgs84
        else None,
        "haversine_sphere__linear_n_minus_1": diagnostic_score(horizontal_haversine),
        "haversine_sphere__nearest_rank_ceiling": (
            kaggle._percentile_nearest_rank_ceiling(horizontal_haversine, 0.50)
            + kaggle._percentile_nearest_rank_ceiling(horizontal_haversine, 0.95)
        )
        / 2.0
        if horizontal_haversine
        else None,
    }
    score_values = [value for value in score_variants.values() if value is not None]
    return {
        "device_epochs": end - start,
        "input_position_epochs": len(raw_rows),
        "output_epochs": len(scored_output_rows),
        "truth_matched_epochs": len(matched),
        "availability_ratio": len(scored_output_rows) / (end - start),
        "truth_coverage_ratio": len(matched) / (end - start),
        "horizontal_wgs84_m": wgs84,
        "horizontal_haversine_m": haversine,
        "vertical_p95_abs_m": _percentile(vertical, 0.95),
        "kaggle_diagnostic_score_variants_m": score_variants,
        "kaggle_diagnostic_mean_m": sum(score_values) / len(score_values) if score_values else None,
        "source_counts": {
            source: sum(row.source == source for row in scored_output_rows)
            for source in sorted({row.source for row in scored_output_rows})
        },
    }


def _metric_value(metrics: dict[str, Any], path: tuple[str, ...]) -> float:
    value: Any = metrics
    for key in path:
        value = value[key]
    if value is None or not math.isfinite(float(value)):
        return math.inf
    return float(value)


def _train_rank(metrics: dict[str, Any]) -> tuple[float, ...]:
    return (
        _metric_value(metrics, ("horizontal_wgs84_m", "p50_m")),
        _metric_value(metrics, ("horizontal_wgs84_m", "p95_m")),
        _metric_value(metrics, ("vertical_p95_abs_m",)),
        _metric_value(metrics, ("kaggle_diagnostic_mean_m",)),
    )


def _validation_pass(
    candidate: dict[str, Any], baseline: dict[str, Any]
) -> tuple[bool, list[str]]:
    failures: list[str] = []
    comparisons = (
        (("availability_ratio",), "availability_regression", True),
        (("horizontal_wgs84_m", "p50_m"), "h_median_regression", False),
        (("horizontal_wgs84_m", "p95_m"), "h_p95_regression", False),
        (("vertical_p95_abs_m",), "v_p95_regression", False),
    )
    for path, label, minimum in comparisons:
        current = _metric_value(candidate, path)
        reference = _metric_value(baseline, path)
        if (current < reference if minimum else current > reference) and not math.isclose(
            current, reference, rel_tol=0.0, abs_tol=1e-12
        ):
            failures.append(label)
    for variant in kaggle.DISTANCE_VARIANT_IDS:
        for percentile_variant in kaggle.PERCENTILE_VARIANT_IDS:
            key = f"{variant}__{percentile_variant}"
            current = _metric_value(candidate, ("kaggle_diagnostic_score_variants_m", key))
            reference = _metric_value(baseline, ("kaggle_diagnostic_score_variants_m", key))
            if current > reference and not math.isclose(current, reference, rel_tol=0.0, abs_tol=1e-12):
                failures.append(f"{key}_regression")
    strict_improvement = any(
        _metric_value(candidate, path) < _metric_value(baseline, path) - 1e-12
        for path in (
            ("horizontal_wgs84_m", "p50_m"),
            ("horizontal_wgs84_m", "p95_m"),
            ("vertical_p95_abs_m",),
        )
    )
    strict_improvement = strict_improvement or any(
        _metric_value(candidate, ("kaggle_diagnostic_score_variants_m", key))
        < _metric_value(baseline, ("kaggle_diagnostic_score_variants_m", key)) - 1e-12
        for key in candidate["kaggle_diagnostic_score_variants_m"]
    )
    if not strict_improvement:
        failures.append("no_strict_improvement")
    return not failures, failures


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        prog=os.environ.get("GNSS_CLI_NAME", "gnss smartphone-trajectory-smoother-eval")
    )
    parser.add_argument(
        "--experimental-trajectory-selection",
        action="store_true",
        help="required development-only opt-in for truth-scored candidate selection",
    )
    parser.add_argument("--position", type=Path, required=True)
    parser.add_argument("--device-gnss", type=Path, required=True)
    parser.add_argument("--ground-truth", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--profile", type=Path)
    parser.add_argument("--role", choices=("development", "holdout"), default="development")
    parser.add_argument("--skip-epochs", type=int)
    parser.add_argument("--split-index", type=int, default=DEFAULT_SPLIT_INDEX)
    parser.add_argument("--match-tolerance-ms", type=int, default=DEFAULT_MATCH_TOLERANCE_MS)
    parser.add_argument("--submission-output", type=Path)
    parser.add_argument("--phone")
    parser.add_argument("--dataset-id")
    return parser.parse_args(argv)


def run(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    try:
        started = time.perf_counter()
        if not args.experimental_trajectory_selection:
            raise smoother.SmootherError(
                "refusing to run without --experimental-trajectory-selection"
            )
        if args.role != "development":
            raise smoother.SmootherError("holdout evaluation is sealed")
        if args.split_index <= 0:
            raise smoother.SmootherError("split-index must be positive")
        if args.match_tolerance_ms < 0:
            raise smoother.SmootherError("match-tolerance-ms must be non-negative")
        profile = smoother._load_profile(args.profile, args.role)
        profile_skip = int(profile["dataset"].get("skip_epochs", 0)) if profile else 0
        skip_epochs = profile_skip if args.skip_epochs is None else args.skip_epochs
        if skip_epochs < 0:
            raise smoother.SmootherError("skip_epochs must be non-negative")
        if profile and args.skip_epochs is not None and args.skip_epochs != profile_skip:
            raise smoother.SmootherError("--skip-epochs cannot override profile value")
        if profile:
            expected_hash = profile["dataset"].get("device_gnss_sha256")
            if expected_hash and smoother._sha256(args.device_gnss) != expected_hash:
                raise smoother.SmootherError("device GNSS hash does not match profile")
        positions = smoother._read_positions(
            args.position, smoother.DEFAULT_GPS_UTC_LEAP_SECONDS
        )
        device_epochs = smoother._read_device_epochs(args.device_gnss, skip_epochs)
        if args.split_index >= len(device_epochs):
            raise smoother.SmootherError(
                f"split-index {args.split_index} must be below {len(device_epochs)} epochs"
            )
        truth = _read_truth(args.ground_truth)
        position_by_timestamp = {row.timestamp_ms: row for row in positions}
        baseline_rows = [
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
        # Baseline metrics use the raw solver epochs and retain the fixed
        # device-key population; no baseline state is carried across the split.
        baseline_by_timestamp = {row.timestamp_ms: row for row in baseline_rows}
        baseline_all = [
            baseline_by_timestamp.get(timestamp)
            for timestamp in device_epochs
        ]
        baseline_full = [
            row
            if row is not None
            else smoother.SmoothedRow(
                timestamp_ms=device_epochs[index],
                week=smoother._device_time_to_week_tow(
                    device_epochs[index], smoother.DEFAULT_GPS_UTC_LEAP_SECONDS
                )[0],
                tow=smoother._device_time_to_week_tow(
                    device_epochs[index], smoother.DEFAULT_GPS_UTC_LEAP_SECONDS
                )[1],
                ecef=np.zeros(3),
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
            for index, row in enumerate(baseline_all)
        ]
        # Missing placeholders are excluded from raw baseline error scoring;
        # candidate output is expected to synthesize them.
        baseline_train = _score_rows(
            [row for row in baseline_full],
            position_by_timestamp,
            truth,
            0,
            args.split_index,
            match_tolerance_ms=args.match_tolerance_ms,
        )
        baseline_validation = _score_rows(
            [row for row in baseline_full],
            position_by_timestamp,
            truth,
            args.split_index,
            len(device_epochs),
            match_tolerance_ms=args.match_tolerance_ms,
        )
        baseline_full_development = _score_rows(
            [row for row in baseline_full],
            position_by_timestamp,
            truth,
            0,
            len(device_epochs),
            match_tolerance_ms=args.match_tolerance_ms,
        )

        candidates: dict[str, Any] = {}
        for candidate_id, process_noise, measurement_floor in PREDECLARED_CANDIDATES:
            config = smoother.SmootherConfig(process_noise, measurement_floor)
            result = smoother.smooth_positions(
                positions,
                device_epochs,
                config,
                reset_indices=(args.split_index,),
            )
            candidate_dir = args.output_dir / "candidates" / candidate_id
            smoother.write_outputs(
                result,
                config,
                candidate_dir,
                position_path=args.position,
                device_path=args.device_gnss,
                profile=profile,
                skip_epochs=skip_epochs,
                leap_seconds=smoother.DEFAULT_GPS_UTC_LEAP_SECONDS,
            )
            train_metrics = _score_rows(
                result.rows,
                position_by_timestamp,
                truth,
                0,
                args.split_index,
                match_tolerance_ms=args.match_tolerance_ms,
            )
            validation_metrics = _score_rows(
                result.rows,
                position_by_timestamp,
                truth,
                args.split_index,
                len(device_epochs),
                match_tolerance_ms=args.match_tolerance_ms,
            )
            full_metrics = _score_rows(
                result.rows,
                position_by_timestamp,
                truth,
                0,
                len(device_epochs),
                match_tolerance_ms=args.match_tolerance_ms,
            )
            candidate_passed, candidate_failures = _validation_pass(
                validation_metrics, baseline_validation
            )
            candidates[candidate_id] = {
                "config": {
                    "process_noise": process_noise,
                    "measurement_floor_m": measurement_floor,
                    "outlier_gate_sigma": config.outlier_gate_sigma,
                    "segment_gap_s": config.segment_gap_s,
                },
                "train": train_metrics,
                "validation": validation_metrics,
                "full_development": full_metrics,
                "validation_gate": {
                    "passed": candidate_passed,
                    "failures": candidate_failures,
                },
                "artifacts": {
                    "directory": str(candidate_dir),
                    "manifest": str(candidate_dir / "smoother_manifest.json"),
                },
            }
        ranked = sorted(
            candidates,
            key=lambda candidate_id: _train_rank(candidates[candidate_id]["train"]),
        )
        selected_id = ranked[0]
        selected = candidates[selected_id]
        validation_passed, validation_failures = _validation_pass(
            selected["validation"], baseline_validation
        )
        selected["validation_gate"] = {
            "passed": validation_passed,
            "failures": validation_failures,
            "baseline": baseline_validation,
        }
        selected_result_dir = args.output_dir / "selected"
        selected_result_manifest: dict[str, Any] | None = None
        if validation_passed:
            config = smoother.SmootherConfig(
                float(selected["config"]["process_noise"]),
                float(selected["config"]["measurement_floor_m"]),
            )
            selected_result = smoother.smooth_positions(
                positions,
                device_epochs,
                config,
                reset_indices=(args.split_index,),
            )
            selected_result_manifest = smoother.write_outputs(
                selected_result,
                config,
                selected_result_dir,
                position_path=args.position,
                device_path=args.device_gnss,
                profile=profile,
                submission_output=args.submission_output,
                phone=args.phone,
                dataset_id=args.dataset_id,
                skip_epochs=skip_epochs,
                leap_seconds=smoother.DEFAULT_GPS_UTC_LEAP_SECONDS,
            )
        report = {
            "schema_version": SCHEMA_VERSION,
            "decision": "promote-development-only" if validation_passed else "no-go",
            "truth_free_smoother_contract": {
                "truth_passed_to_filter": False,
                "holdout_used": False,
                "selection_truth_role": "development_evaluation_only",
            },
            "split": {
                "selected_device_epochs": len(device_epochs),
                "train_epochs": args.split_index,
                "validation_epochs": len(device_epochs) - args.split_index,
                "boundary_policy": "filter/covariance/RTS state reset exactly at split index",
                "reset_indices": [args.split_index],
            },
            "candidate_policy": {
                "predeclared": [
                    {
                        "id": candidate_id,
                        "process_noise": process_noise,
                        "measurement_floor_m": measurement_floor,
                    }
                    for candidate_id, process_noise, measurement_floor in PREDECLARED_CANDIDATES
                ],
                "train_rank": "WGS84 H median, WGS84 H P95, V P95, then mean of four local public-spec diagnostics",
                "validation_gate": "availability and H median/H P95/V P95/four diagnostics non-inferior, with at least one strict improvement",
            },
            "baseline": {
                "train": baseline_train,
                "validation": baseline_validation,
                "full_development": baseline_full_development,
            },
            "ranked_candidate_ids": ranked,
            "selected_candidate_id": selected_id,
            "selected_candidate": selected,
            "candidates": candidates,
            "selected_artifacts": selected_result_manifest,
            "inputs": {
                "position": {"path": str(args.position), "sha256": smoother._sha256(args.position)},
                "device_gnss": {"path": str(args.device_gnss), "sha256": smoother._sha256(args.device_gnss)},
                "ground_truth": {"path": str(args.ground_truth), "sha256": smoother._sha256(args.ground_truth)},
                "profile": (
                    {"path": str(profile["path"]), "sha256": profile["sha256"], "role": profile["role"]}
                    if profile
                    else None
                ),
            },
            "timing": {"evaluation_wall_s": time.perf_counter() - started},
        }
        args.output_dir.mkdir(parents=True, exist_ok=True)
        _atomic_json(args.output_dir / "evaluation_report.json", report)
    except (OSError, smoother.SmootherError, ValueError, KeyError, TypeError) as exc:
        print(f"Smartphone trajectory smoother evaluation failed: {exc}", file=sys.stderr)
        return 1
    print(f"Smartphone trajectory smoother evaluation: {report['decision']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(run())
