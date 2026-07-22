#!/usr/bin/env python3
"""Bridge FLOAT horizontal motion and missing epochs with velocity anchors.

The bridge is reference-free.  It trapezoidally integrates an ENU velocity
stream inside each FLOAT span, distributes the endpoint closure error across
the span, and blends the resulting horizontal trajectory with the input POS.
FIX epochs, status labels, telemetry, and the input Up component are retained.
Optional missing-epoch filling emits FLOAT solutions on the inferred regular
time grid between existing position anchors; it never promotes a status.
"""

from __future__ import annotations

import argparse
import csv
import dataclasses
import json
import math
from pathlib import Path
import sys

import numpy as np


ROOT_DIR = Path(__file__).resolve().parents[1]
SCRIPTS_DIR = Path(__file__).resolve().parent
COMMANDS_DIR = ROOT_DIR / "apps" / "commands"
for command_path in (
    SCRIPTS_DIR,
    COMMANDS_DIR,
    *(COMMANDS_DIR / group for group in ("benchmarks", "positioning", "products", "receivers")),
):
    if str(command_path) not in sys.path:
        sys.path.insert(0, str(command_path))

import apply_ppc_dual_profile_selector as pos_writer  # noqa: E402
import generate_driving_comparison as comparison  # noqa: E402
import gnss_ppc_metrics as ppc_metrics  # noqa: E402


def ecef_to_enu_rotation(epoch: comparison.SolutionEpoch) -> np.ndarray:
    lat = math.radians(epoch.lat_deg)
    lon = math.radians(epoch.lon_deg)
    sin_lat, cos_lat = math.sin(lat), math.cos(lat)
    sin_lon, cos_lon = math.sin(lon), math.cos(lon)
    return np.asarray(
        [
            [-sin_lon, cos_lon, 0.0],
            [-sin_lat * cos_lon, -sin_lat * sin_lon, cos_lat],
            [cos_lat * cos_lon, cos_lat * sin_lon, sin_lat],
        ]
    )


def read_velocity_csv(path: Path) -> tuple[np.ndarray, np.ndarray, str]:
    times: list[float] = []
    velocities: list[tuple[float, ...]] = []
    with path.open(encoding="utf-8", newline="") as handle:
        reader = csv.DictReader(handle)
        fields = set(reader.fieldnames or ())
        if {"tow", "vel_e_mps", "vel_n_mps"}.issubset(fields):
            time_column = "tow"
            velocity_columns = ("vel_e_mps", "vel_n_mps")
            frame = "enu"
        elif {"gps_tow", "fgo_vx_mps", "fgo_vy_mps", "fgo_vz_mps"}.issubset(fields):
            time_column = "gps_tow"
            velocity_columns = ("fgo_vx_mps", "fgo_vy_mps", "fgo_vz_mps")
            frame = "ecef"
        else:
            raise ValueError(
                "velocity CSV must contain FGO ENU or gnss_vel_d ECEF velocity columns"
            )
        for row in reader:
            tow = float(row[time_column])
            velocity = tuple(float(row[column]) for column in velocity_columns)
            if math.isfinite(tow) and all(math.isfinite(value) for value in velocity):
                times.append(tow)
                velocities.append(velocity)  # type: ignore[arg-type]
    if len(times) < 2:
        raise ValueError("velocity CSV contains fewer than two finite rows")
    order = np.argsort(np.asarray(times), kind="stable")
    ordered_times = np.asarray(times, dtype=float)[order]
    ordered_velocities = np.asarray(velocities, dtype=float)[order]
    if np.any(np.diff(ordered_times) <= 0.0):
        raise ValueError("velocity CSV times must be unique")
    return ordered_times, ordered_velocities, frame


def _velocity_interval_is_usable(
    velocity_times_s: np.ndarray,
    start_s: float,
    end_s: float,
    max_velocity_sample_gap_s: float,
) -> bool:
    if start_s < velocity_times_s[0] or end_s > velocity_times_s[-1]:
        return False
    first = max(0, int(np.searchsorted(velocity_times_s, start_s) - 1))
    last = min(
        len(velocity_times_s) - 1,
        int(np.searchsorted(velocity_times_s, end_s, side="right")),
    )
    return bool(
        last > first
        and np.max(np.diff(velocity_times_s[first : last + 1]))
        <= max_velocity_sample_gap_s
    )


def _integrated_horizontal(
    sample_times_s: np.ndarray,
    velocity_times_s: np.ndarray,
    velocity_enu_mps: np.ndarray,
) -> np.ndarray:
    velocities = np.column_stack(
        [
            np.interp(sample_times_s, velocity_times_s, velocity_enu_mps[:, 0]),
            np.interp(sample_times_s, velocity_times_s, velocity_enu_mps[:, 1]),
        ]
    )
    integrated = np.zeros((len(sample_times_s), 2), dtype=float)
    for index in range(1, len(sample_times_s)):
        dt = sample_times_s[index] - sample_times_s[index - 1]
        integrated[index] = integrated[index - 1] + 0.5 * (
            velocities[index - 1] + velocities[index]
        ) * dt
    return integrated


def fill_missing_velocity_epochs(
    epochs: list[comparison.SolutionEpoch],
    velocity_times_s: np.ndarray,
    velocity_enu_mps: np.ndarray,
    *,
    epoch_interval_s: float,
    max_fill_span_s: float,
    max_candidate_correction_m: float,
    max_velocity_sample_gap_s: float,
    max_closure_rate_mps: float = 0.0,
) -> tuple[list[comparison.SolutionEpoch], dict[str, int]]:
    """Fill regular-grid gaps between existing positions as FLOAT epochs."""
    if max_fill_span_s <= 0.0:
        return sorted(epochs, key=lambda epoch: (epoch.week, epoch.tow)), {
            "filled_missing_epochs": 0,
            "accepted_fill_spans": 0,
            "fill_duration_rejected_spans": 0,
            "fill_correction_rejected_spans": 0,
            "fill_velocity_rejected_spans": 0,
            "fill_closure_rate_rejected_spans": 0,
        }
    if epoch_interval_s <= 0.0:
        raise ValueError("epoch interval must be positive when filling gaps")

    ordered = sorted(epochs, key=lambda epoch: (epoch.week, epoch.tow))
    origin = ordered[0]
    rotation = ecef_to_enu_rotation(origin)
    origin_ecef = np.asarray(origin.ecef, dtype=float)
    output: list[comparison.SolutionEpoch] = []
    counters = {
        "filled_missing_epochs": 0,
        "accepted_fill_spans": 0,
        "fill_duration_rejected_spans": 0,
        "fill_correction_rejected_spans": 0,
        "fill_velocity_rejected_spans": 0,
        "fill_closure_rate_rejected_spans": 0,
    }

    optional_fields = (
        "rtk_iterations",
        "rtk_update_observations",
        "rtk_update_phase_observations",
        "rtk_update_code_observations",
        "rtk_update_suppressed_outliers",
        "rtk_update_prefit_residual_rms_m",
        "rtk_update_prefit_residual_max_m",
        "rtk_update_post_suppression_residual_rms_m",
        "rtk_update_post_suppression_residual_max_m",
        "rtk_update_normalized_innovation_squared",
        "rtk_update_normalized_innovation_squared_per_observation",
        "rtk_update_rejected_by_innovation_gate",
    )
    clear_telemetry = {field: None for field in optional_fields}

    for left, right in zip(ordered, ordered[1:]):
        output.append(left)
        duration_s = right.tow - left.tow
        missing_count = int(round(duration_s / epoch_interval_s)) - 1
        if (
            left.week != right.week
            or missing_count <= 0
            or abs(duration_s - (missing_count + 1) * epoch_interval_s)
            > 0.1 * epoch_interval_s
        ):
            continue
        if duration_s > max_fill_span_s:
            counters["fill_duration_rejected_spans"] += 1
            continue
        if not _velocity_interval_is_usable(
            velocity_times_s, left.tow, right.tow, max_velocity_sample_gap_s
        ):
            counters["fill_velocity_rejected_spans"] += 1
            continue

        sample_times = np.linspace(left.tow, right.tow, missing_count + 2)
        integrated = _integrated_horizontal(
            sample_times, velocity_times_s, velocity_enu_mps
        )
        left_enu = rotation @ (np.asarray(left.ecef) - origin_ecef)
        right_enu = rotation @ (np.asarray(right.ecef) - origin_ecef)
        target_delta = right_enu[:2] - left_enu[:2]
        closure = target_delta - integrated[-1]
        if (
            max_closure_rate_mps > 0.0
            and float(np.linalg.norm(closure)) / duration_s > max_closure_rate_mps
        ):
            counters["fill_closure_rate_rejected_spans"] += 1
            continue
        alpha = (sample_times - sample_times[0]) / duration_s
        candidate_horizontal = left_enu[:2] + integrated + alpha[:, None] * closure
        linear_horizontal = left_enu[:2] + alpha[:, None] * target_delta
        if (
            float(np.max(np.linalg.norm(candidate_horizontal - linear_horizontal, axis=1)))
            > max_candidate_correction_m
        ):
            counters["fill_correction_rejected_spans"] += 1
            continue

        for index in range(1, missing_count + 1):
            enu = np.asarray(
                [
                    candidate_horizontal[index, 0],
                    candidate_horizontal[index, 1],
                    left_enu[2] + alpha[index] * (right_enu[2] - left_enu[2]),
                ]
            )
            ecef = origin_ecef + rotation.transpose() @ enu
            lat_deg, lon_deg, height_m = ppc_metrics.llh_from_ecef(*ecef)
            output.append(
                dataclasses.replace(
                    left,
                    tow=float(sample_times[index]),
                    ecef=ecef,
                    lat_deg=lat_deg,
                    lon_deg=lon_deg,
                    height_m=height_m,
                    status=3,
                    num_satellites=min(left.num_satellites, right.num_satellites),
                    ratio=0.0,
                    baseline_m=(
                        None
                        if left.baseline_m is None or right.baseline_m is None
                        else left.baseline_m
                        + alpha[index] * (right.baseline_m - left.baseline_m)
                    ),
                    **clear_telemetry,
                )
            )
        counters["filled_missing_epochs"] += missing_count
        counters["accepted_fill_spans"] += 1
    output.append(ordered[-1])
    return output, counters


def bridge_velocity_spans(
    epochs: list[comparison.SolutionEpoch],
    velocity_times_s: np.ndarray,
    velocity_enu_mps: np.ndarray,
    *,
    blend: float,
    max_span_s: float,
    max_candidate_correction_m: float,
    max_velocity_sample_gap_s: float,
    max_closure_rate_mps: float = 0.0,
    min_span_s: float = 0.0,
    min_candidate_correction_m: float = 0.0,
) -> tuple[list[comparison.SolutionEpoch], dict[str, object]]:
    if not epochs:
        raise ValueError("input trajectory is empty")
    if not 0.0 < blend <= 1.0:
        raise ValueError("blend must be in (0, 1]")
    if max_span_s <= 0.0 or max_candidate_correction_m <= 0.0:
        raise ValueError("span and correction gates must be positive")
    if (
        max_closure_rate_mps < 0.0
        or min_span_s < 0.0
        or min_candidate_correction_m < 0.0
    ):
        raise ValueError("closure rate and minimum gates must be non-negative")
    if min_candidate_correction_m > max_candidate_correction_m:
        raise ValueError("minimum correction cannot exceed maximum correction")
    if max_velocity_sample_gap_s <= 0.0:
        raise ValueError("velocity sample gap must be positive")

    ordered = sorted(epochs, key=lambda epoch: (epoch.week, epoch.tow))
    origin = ordered[0]
    rotation = ecef_to_enu_rotation(origin)
    origin_ecef = np.asarray(origin.ecef, dtype=float)
    positions_enu = np.asarray(
        [rotation @ (np.asarray(epoch.ecef) - origin_ecef) for epoch in ordered]
    )
    times = np.asarray([epoch.tow for epoch in ordered], dtype=float)
    output = list(ordered)
    accepted_spans = 0
    duration_rejected_spans = 0
    correction_rejected_spans = 0
    velocity_rejected_spans = 0
    closure_rate_rejected_spans = 0
    minimum_duration_rejected_spans = 0
    minimum_correction_rejected_spans = 0
    replaced_float_epochs = 0

    index = 0
    while index < len(ordered):
        if ordered[index].status == 4:
            index += 1
            continue
        left = index - 1
        right = index
        while right < len(ordered) and ordered[right].status != 4:
            right += 1
        index = right
        if left < 0 or right >= len(ordered):
            continue
        if ordered[left].week != ordered[right].week:
            continue
        duration_s = times[right] - times[left]
        if duration_s <= 0.0 or duration_s > max_span_s:
            duration_rejected_spans += 1
            continue
        if duration_s < min_span_s:
            minimum_duration_rejected_spans += 1
            continue
        if not _velocity_interval_is_usable(
            velocity_times_s, times[left], times[right], max_velocity_sample_gap_s
        ):
            velocity_rejected_spans += 1
            continue

        span_indices = np.arange(left, right + 1)
        span_times = times[span_indices]
        integrated = _integrated_horizontal(
            span_times, velocity_times_s, velocity_enu_mps
        )
        target_delta = positions_enu[right, :2] - positions_enu[left, :2]
        closure = target_delta - integrated[-1]
        closure_rate_mps = float(np.linalg.norm(closure)) / duration_s
        if max_closure_rate_mps > 0.0 and closure_rate_mps > max_closure_rate_mps:
            closure_rate_rejected_spans += 1
            continue
        alpha = (span_times - span_times[0]) / duration_s
        candidate_horizontal = (
            positions_enu[left, :2]
            + integrated
            + alpha[:, None] * closure
        )
        candidate_correction = (
            candidate_horizontal - positions_enu[span_indices, :2]
        )
        span_max_correction_m = float(
            np.max(np.linalg.norm(candidate_correction, axis=1))
        )
        if span_max_correction_m > max_candidate_correction_m:
            correction_rejected_spans += 1
            continue
        if span_max_correction_m < min_candidate_correction_m:
            minimum_correction_rejected_spans += 1
            continue

        for local_index, epoch_index in enumerate(span_indices[1:-1], start=1):
            correction_enu = np.asarray(
                [
                    blend * candidate_correction[local_index, 0],
                    blend * candidate_correction[local_index, 1],
                    0.0,
                ]
            )
            correction_ecef = rotation.transpose() @ correction_enu
            epoch_lat = math.radians(ordered[epoch_index].lat_deg)
            epoch_lon = math.radians(ordered[epoch_index].lon_deg)
            local_up_ecef = np.asarray(
                [
                    math.cos(epoch_lat) * math.cos(epoch_lon),
                    math.cos(epoch_lat) * math.sin(epoch_lon),
                    math.sin(epoch_lat),
                ]
            )
            correction_ecef -= local_up_ecef * float(
                np.dot(correction_ecef, local_up_ecef)
            )
            ecef = np.asarray(ordered[epoch_index].ecef) + correction_ecef
            lat_deg, lon_deg, height_m = ppc_metrics.llh_from_ecef(*ecef)
            output[epoch_index] = dataclasses.replace(
                ordered[epoch_index],
                ecef=ecef,
                lat_deg=lat_deg,
                lon_deg=lon_deg,
                height_m=height_m,
            )
            replaced_float_epochs += 1
        accepted_spans += 1

    summary: dict[str, object] = {
        "reference_truth_used": False,
        "input_epochs": len(ordered),
        "output_epochs": len(output),
        "preserved_fixed_epochs": sum(epoch.status == 4 for epoch in ordered),
        "replaced_float_epochs": replaced_float_epochs,
        "accepted_spans": accepted_spans,
        "duration_rejected_spans": duration_rejected_spans,
        "correction_rejected_spans": correction_rejected_spans,
        "velocity_rejected_spans": velocity_rejected_spans,
        "closure_rate_rejected_spans": closure_rate_rejected_spans,
        "minimum_duration_rejected_spans": minimum_duration_rejected_spans,
        "minimum_correction_rejected_spans": minimum_correction_rejected_spans,
        "blend": blend,
        "max_span_s": max_span_s,
        "max_candidate_correction_m": max_candidate_correction_m,
        "max_velocity_sample_gap_s": max_velocity_sample_gap_s,
        "max_closure_rate_mps": max_closure_rate_mps,
        "min_span_s": min_span_s,
        "min_candidate_correction_m": min_candidate_correction_m,
        "preserved_status": True,
        "preserved_up": True,
        "velocity_columns_used": ["tow", "vel_e_mps", "vel_n_mps"],
    }
    return output, summary


def overlay_changed_horizontal(
    baseline: list[comparison.SolutionEpoch],
    candidate: list[comparison.SolutionEpoch],
    overlay: list[comparison.SolutionEpoch],
    *,
    change_threshold_m: float = 1e-5,
) -> tuple[list[comparison.SolutionEpoch], int]:
    """Apply candidate horizontal deltas to an existing staged solution."""
    if not (len(baseline) == len(candidate) == len(overlay)):
        raise ValueError("baseline, candidate, and overlay epoch counts must match")
    output: list[comparison.SolutionEpoch] = []
    replaced = 0
    for base_epoch, candidate_epoch, overlay_epoch in zip(
        baseline, candidate, overlay
    ):
        keys = {
            (base_epoch.week, round(base_epoch.tow * 1000.0)),
            (candidate_epoch.week, round(candidate_epoch.tow * 1000.0)),
            (overlay_epoch.week, round(overlay_epoch.tow * 1000.0)),
        }
        if len(keys) != 1:
            raise ValueError("baseline, candidate, and overlay epoch grids must match")
        lat = math.radians(overlay_epoch.lat_deg)
        lon = math.radians(overlay_epoch.lon_deg)
        local_up_ecef = np.asarray(
            [
                math.cos(lat) * math.cos(lon),
                math.cos(lat) * math.sin(lon),
                math.sin(lat),
            ]
        )
        delta_ecef = np.asarray(candidate_epoch.ecef) - np.asarray(base_epoch.ecef)
        horizontal_delta_ecef = delta_ecef - local_up_ecef * float(
            np.dot(delta_ecef, local_up_ecef)
        )
        if float(np.linalg.norm(horizontal_delta_ecef)) <= change_threshold_m:
            output.append(overlay_epoch)
            continue
        ecef = np.asarray(overlay_epoch.ecef) + horizontal_delta_ecef
        lat_deg, lon_deg, height_m = ppc_metrics.llh_from_ecef(*ecef)
        output.append(
            dataclasses.replace(
                overlay_epoch,
                ecef=ecef,
                lat_deg=lat_deg,
                lon_deg=lon_deg,
                height_m=height_m,
            )
        )
        replaced += 1
    return output, replaced


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--baseline-pos", type=Path, required=True)
    parser.add_argument("--velocity-csv", type=Path, required=True)
    parser.add_argument("--output-pos", type=Path, required=True)
    parser.add_argument(
        "--overlay-pos",
        type=Path,
        help="Apply accepted horizontal deltas to this staged POS solution.",
    )
    parser.add_argument("--summary-json", type=Path)
    parser.add_argument("--blend", type=float, default=0.75)
    parser.add_argument("--max-span-s", type=float, default=30.0)
    parser.add_argument("--max-candidate-correction-m", type=float, default=5.0)
    parser.add_argument("--max-velocity-sample-gap-s", type=float, default=0.25)
    parser.add_argument(
        "--max-closure-rate-mps",
        type=float,
        default=0.0,
        help="Reject spans whose FIX-endpoint velocity closure exceeds this rate; 0 disables.",
    )
    parser.add_argument("--min-span-s", type=float, default=0.0)
    parser.add_argument("--min-candidate-correction-m", type=float, default=0.0)
    parser.add_argument(
        "--fill-missing-epochs",
        action="store_true",
        help="Emit FLOAT epochs on regular-grid gaps bounded by existing positions.",
    )
    parser.add_argument(
        "--fill-only",
        action="store_true",
        help="Fill missing epochs without re-bridging existing FLOAT positions.",
    )
    parser.add_argument(
        "--epoch-interval-s",
        type=float,
        default=0.2,
        help="Expected regular output interval for missing-epoch filling.",
    )
    parser.add_argument("--max-fill-span-s", type=float, default=30.0)
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    if args.fill_only and not args.fill_missing_epochs:
        raise SystemExit("--fill-only requires --fill-missing-epochs")
    epochs = comparison.read_libgnss_pos(args.baseline_pos)
    input_epoch_count = len(epochs)
    velocity_times, velocities, velocity_frame = read_velocity_csv(args.velocity_csv)
    if velocity_frame == "ecef":
        rotation = ecef_to_enu_rotation(epochs[0])
        velocities = (rotation @ velocities.transpose()).transpose()[:, :2]
    fill_summary: dict[str, int] = {}
    if args.fill_missing_epochs:
        epochs, fill_summary = fill_missing_velocity_epochs(
            epochs,
            velocity_times,
            velocities,
            epoch_interval_s=args.epoch_interval_s,
            max_fill_span_s=args.max_fill_span_s,
            max_candidate_correction_m=args.max_candidate_correction_m,
            max_velocity_sample_gap_s=args.max_velocity_sample_gap_s,
            max_closure_rate_mps=args.max_closure_rate_mps,
        )
    if args.fill_only:
        bridged = epochs
        summary: dict[str, object] = {
            "reference_truth_used": False,
            "input_epochs": input_epoch_count,
            "output_epochs": len(bridged),
            "preserved_fixed_epochs": sum(epoch.status == 4 for epoch in bridged),
            "replaced_float_epochs": 0,
            "accepted_spans": 0,
            "duration_rejected_spans": 0,
            "correction_rejected_spans": 0,
            "velocity_rejected_spans": 0,
            "closure_rate_rejected_spans": 0,
            "minimum_duration_rejected_spans": 0,
            "blend": args.blend,
            "max_span_s": args.max_span_s,
            "max_candidate_correction_m": args.max_candidate_correction_m,
            "max_velocity_sample_gap_s": args.max_velocity_sample_gap_s,
            "max_closure_rate_mps": args.max_closure_rate_mps,
            "min_span_s": args.min_span_s,
            "preserved_status": True,
            "preserved_up": True,
            "velocity_columns_used": ["tow", "vel_e_mps", "vel_n_mps"],
        }
    else:
        bridged, summary = bridge_velocity_spans(
            epochs,
            velocity_times,
            velocities,
            blend=args.blend,
            max_span_s=args.max_span_s,
            max_candidate_correction_m=args.max_candidate_correction_m,
            max_velocity_sample_gap_s=args.max_velocity_sample_gap_s,
            max_closure_rate_mps=args.max_closure_rate_mps,
            min_span_s=args.min_span_s,
            min_candidate_correction_m=args.min_candidate_correction_m,
        )
        summary["input_epochs"] = input_epoch_count
        summary["output_epochs"] = len(bridged)
    if args.overlay_pos is not None:
        overlay = comparison.read_libgnss_pos(args.overlay_pos)
        bridged, overlay_replaced_epochs = overlay_changed_horizontal(
            comparison.read_libgnss_pos(args.baseline_pos), bridged, overlay
        )
        summary["overlay_pos"] = str(args.overlay_pos)
        summary["overlay_replaced_epochs"] = overlay_replaced_epochs
        summary["output_epochs"] = len(bridged)
    summary["velocity_csv"] = str(args.velocity_csv)
    summary["fill_missing_epochs"] = args.fill_missing_epochs
    summary["fill_only"] = args.fill_only
    summary["epoch_interval_s"] = args.epoch_interval_s
    summary["max_fill_span_s"] = args.max_fill_span_s
    summary.update(fill_summary)
    summary["velocity_input_frame"] = velocity_frame
    summary["velocity_columns_used"] = (
        ["tow", "vel_e_mps", "vel_n_mps"]
        if velocity_frame == "enu"
        else ["gps_tow", "fgo_vx_mps", "fgo_vy_mps", "fgo_vz_mps"]
    )
    pos_writer.write_pos(args.output_pos, bridged)
    payload = json.dumps(summary, indent=2, sort_keys=True) + "\n"
    if args.summary_json is None:
        print(payload, end="")
    else:
        args.summary_json.parent.mkdir(parents=True, exist_ok=True)
        args.summary_json.write_text(payload, encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
