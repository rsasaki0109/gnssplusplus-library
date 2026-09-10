#!/usr/bin/env python3
"""Development-only evaluation of causal IMU-adaptive trajectory smoothing.

The IMU feature builder is truth-free and causal.  This command is the only
place that reads development ground truth, and it uses it solely to select a
predeclared motion-adaptive process-noise candidate.  Holdout roles are
rejected by construction.
"""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import sys
import time
from typing import Any

from support.gnss_runtime import application_root


ROOT = application_root(__file__)
sys.path.insert(0, str(ROOT / "apps" / "commands" / "benchmarks"))
import gnss_smartphone_imu as imu  # noqa: E402
import gnss_smartphone_trajectory_smoother as smoother  # noqa: E402
import gnss_smartphone_trajectory_smoother_eval as base_eval  # noqa: E402


SCHEMA_VERSION = "smartphone-trajectory-imu-evaluation.v1"
DEFAULT_SPLIT_INDEX = base_eval.DEFAULT_SPLIT_INDEX
PREDECLARED_MOTION_CANDIDATES = (
    ("gyro0p25_accel1p0_gain2", 0.25, 1.0, 2.0),
    ("gyro0p15_accel0p75_gain2", 0.15, 0.75, 2.0),
    ("gyro0p10_accel0p50_gain2", 0.10, 0.50, 2.0),
    ("gyro0p15_accel0p75_gain3", 0.15, 0.75, 3.0),
    ("gyro0p25_accel1p0_gain3", 0.25, 1.00, 3.0),
)


def _atomic_json(path: Path, payload: dict[str, Any]) -> None:
    content = (json.dumps(payload, indent=2, sort_keys=True) + "\n").encode("utf-8")
    smoother._atomic_write(path, content)


def _baseline_rows(
    positions: list[smoother.PositionRow],
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
            source="measured",
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
    by_timestamp = {row.timestamp_ms: row for row in _baseline_rows(positions)}
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


def _metrics(
    rows: list[smoother.SmoothedRow],
    positions_by_timestamp: dict[int, smoother.PositionRow],
    truth: dict[int, tuple[float, float, float]],
    start: int,
    end: int,
    tolerance_ms: int,
) -> dict[str, Any]:
    return base_eval._score_rows(
        rows,
        positions_by_timestamp,
        truth,
        start,
        end,
        match_tolerance_ms=tolerance_ms,
    )


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        prog=os.environ.get("GNSS_CLI_NAME", "gnss smartphone-trajectory-imu-eval")
    )
    parser.add_argument(
        "--experimental-motion-adaptive-q-selection",
        action="store_true",
        help="required development-only opt-in for IMU-adaptive q selection",
    )
    parser.add_argument("--position", type=Path, required=True)
    parser.add_argument("--device-gnss", type=Path, required=True)
    parser.add_argument("--device-imu", type=Path, required=True)
    parser.add_argument("--ground-truth", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--profile", type=Path)
    parser.add_argument("--role", choices=("development", "holdout"), default="development")
    parser.add_argument("--skip-epochs", type=int)
    parser.add_argument("--split-index", type=int, default=DEFAULT_SPLIT_INDEX)
    parser.add_argument("--match-tolerance-ms", type=int, default=base_eval.DEFAULT_MATCH_TOLERANCE_MS)
    parser.add_argument("--submission-output", type=Path)
    parser.add_argument("--phone")
    parser.add_argument("--dataset-id")
    return parser.parse_args(argv)


def run(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    started = time.perf_counter()
    try:
        if not args.experimental_motion_adaptive_q_selection:
            raise smoother.SmootherError(
                "refusing to run without --experimental-motion-adaptive-q-selection"
            )
        if args.role != "development":
            raise smoother.SmootherError("IMU-adaptive selection is development-only")
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
        truth = base_eval._read_truth(args.ground_truth)
        imu_dataset = imu.read_imu(args.device_imu)
        motion_config_base = {
            "window_s": imu.DEFAULT_WINDOW_S,
            "max_sample_gap_s": imu.DEFAULT_MAX_SAMPLE_GAP_S,
            "max_sample_age_s": imu.DEFAULT_MAX_SAMPLE_AGE_S,
        }
        positions_by_timestamp = {row.timestamp_ms: row for row in positions}
        baseline = _baseline_population(positions, device_epochs)
        baseline_train = _metrics(
            baseline,
            positions_by_timestamp,
            truth,
            0,
            args.split_index,
            args.match_tolerance_ms,
        )
        baseline_validation = _metrics(
            baseline,
            positions_by_timestamp,
            truth,
            args.split_index,
            len(device_epochs),
            args.match_tolerance_ms,
        )
        baseline_full = _metrics(
            baseline,
            positions_by_timestamp,
            truth,
            0,
            len(device_epochs),
            args.match_tolerance_ms,
        )
        candidates: dict[str, Any] = {}
        candidate_results: dict[str, tuple[smoother.SmoothingResult, imu.MotionProfile]] = {}
        config = smoother.SmootherConfig(1.0, 1.0, 5.0, 10.0)
        for candidate_id, gyro_threshold, accel_threshold, q_multiplier in PREDECLARED_MOTION_CANDIDATES:
            motion_config = imu.MotionConfig(
                **motion_config_base,
                gyro_threshold=gyro_threshold,
                accel_dynamic_threshold=accel_threshold,
                motion_q_multiplier=q_multiplier,
            )
            motion_profile = imu.build_motion_profile(
                device_epochs,
                imu_dataset,
                motion_config,
                reset_indices=(args.split_index,),
            )
            result = smoother.smooth_positions(
                positions,
                device_epochs,
                config,
                reset_indices=(args.split_index,),
                process_noise_multipliers=motion_profile.multipliers,
            )
            candidate_results[candidate_id] = (result, motion_profile)
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
                motion_summary=motion_profile.summary,
                motion_profile_rows=imu.profile_rows(motion_profile),
            )
            train = _metrics(
                result.rows,
                positions_by_timestamp,
                truth,
                0,
                args.split_index,
                args.match_tolerance_ms,
            )
            validation = _metrics(
                result.rows,
                positions_by_timestamp,
                truth,
                args.split_index,
                len(device_epochs),
                args.match_tolerance_ms,
            )
            full = _metrics(
                result.rows,
                positions_by_timestamp,
                truth,
                0,
                len(device_epochs),
                args.match_tolerance_ms,
            )
            passed, failures = base_eval._validation_pass(validation, baseline_validation)
            candidates[candidate_id] = {
                "config": {
                    "process_noise": config.process_noise,
                    "measurement_floor_m": config.measurement_floor_m,
                    "outlier_gate_sigma": config.outlier_gate_sigma,
                    "segment_gap_s": config.segment_gap_s,
                    "motion": {
                        "window_s": motion_config.window_s,
                        "max_sample_gap_s": motion_config.max_sample_gap_s,
                        "max_sample_age_s": motion_config.max_sample_age_s,
                        "gyro_threshold": motion_config.gyro_threshold,
                        "accel_dynamic_threshold": motion_config.accel_dynamic_threshold,
                        "motion_q_multiplier": motion_config.motion_q_multiplier,
                    },
                },
                "train": train,
                "validation": validation,
                "full_development": full,
                "motion_summary": motion_profile.summary,
                "validation_gate": {"passed": passed, "failures": failures},
                "artifacts": {
                    "directory": str(candidate_dir),
                    "manifest": str(candidate_dir / "smoother_manifest.json"),
                },
            }
        ranked = sorted(
            candidates,
            key=lambda candidate_id: base_eval._train_rank(candidates[candidate_id]["train"]),
        )
        selected_id = ranked[0]
        selected = candidates[selected_id]
        validation_passed, validation_failures = base_eval._validation_pass(
            selected["validation"], baseline_validation
        )
        selected["validation_gate"] = {
            "passed": validation_passed,
            "failures": validation_failures,
            "baseline": baseline_validation,
        }
        selected_manifest: dict[str, Any] | None = None
        if validation_passed:
            result, motion_profile = candidate_results[selected_id]
            selected_manifest = smoother.write_outputs(
                result,
                config,
                args.output_dir / "selected",
                position_path=args.position,
                device_path=args.device_gnss,
                profile=profile,
                submission_output=args.submission_output,
                phone=args.phone,
                dataset_id=args.dataset_id,
                skip_epochs=skip_epochs,
                leap_seconds=smoother.DEFAULT_GPS_UTC_LEAP_SECONDS,
                motion_summary=motion_profile.summary,
                motion_profile_rows=imu.profile_rows(motion_profile),
            )
        report = {
            "schema_version": SCHEMA_VERSION,
            "decision": "promote-development-only" if validation_passed else "no-go",
            "truth_free_motion_contract": {
                "truth_passed_to_motion_builder": False,
                "truth_passed_to_smoother": False,
                "absolute_acceleration_integration": False,
                "future_imu_samples_used": False,
                "holdout_used": False,
                "selection_truth_role": "development_evaluation_only",
            },
            "split": {
                "selected_device_epochs": len(device_epochs),
                "train_epochs": args.split_index,
                "validation_epochs": len(device_epochs) - args.split_index,
                "boundary_policy": "filter/covariance/RTS and motion profile state reset at index",
                "reset_indices": [args.split_index],
            },
            "candidate_policy": {
                "predeclared": [
                    {
                        "id": candidate_id,
                        "gyro_threshold": gyro_threshold,
                        "accel_dynamic_threshold": accel_threshold,
                        "motion_q_multiplier": q_multiplier,
                        **motion_config_base,
                    }
                    for candidate_id, gyro_threshold, accel_threshold, q_multiplier in PREDECLARED_MOTION_CANDIDATES
                ],
                "train_rank": "WGS84 H median, WGS84 H P95, V P95, then mean of four local diagnostics",
                "validation_gate": "availability and H median/H P95/V P95/four diagnostics non-inferior, with at least one strict improvement",
            },
            "imu_audit": imu_dataset.audit,
            "baseline": {
                "train": baseline_train,
                "validation": baseline_validation,
                "full_development": baseline_full,
            },
            "ranked_candidate_ids": ranked,
            "selected_candidate_id": selected_id,
            "selected_candidate": selected,
            "candidates": candidates,
            "selected_artifacts": selected_manifest,
            "inputs": {
                "position": {"path": str(args.position), "sha256": smoother._sha256(args.position)},
                "device_gnss": {"path": str(args.device_gnss), "sha256": smoother._sha256(args.device_gnss)},
                "device_imu": {"path": str(args.device_imu), "sha256": imu_dataset.audit["sha256"]},
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
    except (OSError, imu.ImuError, smoother.SmootherError, ValueError, KeyError, TypeError) as exc:
        print(f"Smartphone trajectory IMU evaluation failed: {exc}", file=sys.stderr)
        return 1
    print(f"Smartphone trajectory IMU evaluation: {report['decision']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(run())
