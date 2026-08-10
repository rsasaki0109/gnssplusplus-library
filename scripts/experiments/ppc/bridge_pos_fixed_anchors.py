#!/usr/bin/env python3
"""Bridge short FLOAT/dropout spans between trusted FIX position anchors.

The bridge is an offline, reference-free smoother.  It preserves every FIX
epoch and interpolates ECEF positions only between consecutive trusted FIX
anchors, using either a straight chord or neighboring-anchor Hermite velocity.
Generated/replaced epochs are conservatively labelled FLOAT.
"""

from __future__ import annotations

from _paths import ANALYSIS_DIR, APPS_DIR, COMMANDS_DIR, PPC_DIR, ROOT_DIR, SCRIPTS_DIR

import argparse
import dataclasses
import json
import math
from pathlib import Path
import statistics
import sys

import numpy as np


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


def epoch_key(epoch: comparison.SolutionEpoch) -> tuple[int, int]:
    return epoch.week, round(epoch.tow * 1000.0)


def trusted_anchor(epoch: comparison.SolutionEpoch, args: argparse.Namespace) -> bool:
    if epoch.status != 4:
        return False
    if epoch.ratio is None or epoch.ratio < args.anchor_min_ratio:
        return False
    if epoch.num_satellites < args.anchor_min_satellites:
        return False
    post_rms = epoch.rtk_update_post_suppression_residual_rms_m
    if (
        args.anchor_max_post_rms_m > 0.0
        and (post_rms is None or post_rms > args.anchor_max_post_rms_m)
    ):
        return False
    nis_per_observation = epoch.rtk_update_normalized_innovation_squared_per_observation
    if (
        args.anchor_max_nis_per_observation > 0.0
        and (
            nis_per_observation is None
            or nis_per_observation > args.anchor_max_nis_per_observation
        )
    ):
        return False
    return True


def infer_nominal_interval_s(epochs: list[comparison.SolutionEpoch]) -> float:
    deltas = [
        current.tow - previous.tow
        for previous, current in zip(epochs, epochs[1:])
        if current.week == previous.week and 1e-6 < current.tow - previous.tow <= 1.0
    ]
    if not deltas:
        raise SystemExit("could not infer nominal interval; pass --nominal-interval-s")
    return float(statistics.median(deltas))


def preserve_up_component(
    existing: comparison.SolutionEpoch,
    candidate: comparison.SolutionEpoch,
) -> comparison.SolutionEpoch:
    """Apply only the candidate's local-horizontal ECEF correction."""
    lat = math.radians(existing.lat_deg)
    lon = math.radians(existing.lon_deg)
    up = np.asarray(
        [math.cos(lat) * math.cos(lon), math.cos(lat) * math.sin(lon), math.sin(lat)]
    )
    delta = np.asarray(candidate.ecef) - np.asarray(existing.ecef)
    horizontal_ecef = np.asarray(existing.ecef) + delta - up * float(np.dot(delta, up))
    lat_deg, lon_deg, height_m = ppc_metrics.llh_from_ecef(*horizontal_ecef)
    return dataclasses.replace(
        candidate,
        ecef=horizontal_ecef,
        lat_deg=lat_deg,
        lon_deg=lon_deg,
        height_m=height_m,
    )


def preserve_horizontal_component(
    existing: comparison.SolutionEpoch,
    candidate: comparison.SolutionEpoch,
) -> comparison.SolutionEpoch:
    """Apply only the candidate's local-Up ECEF correction."""
    lat = math.radians(existing.lat_deg)
    lon = math.radians(existing.lon_deg)
    up = np.asarray(
        [math.cos(lat) * math.cos(lon), math.cos(lat) * math.sin(lon), math.sin(lat)]
    )
    delta = np.asarray(candidate.ecef) - np.asarray(existing.ecef)
    vertical_ecef = np.asarray(existing.ecef) + up * float(np.dot(delta, up))
    lat_deg, lon_deg, height_m = ppc_metrics.llh_from_ecef(*vertical_ecef)
    return dataclasses.replace(
        candidate,
        ecef=vertical_ecef,
        lat_deg=lat_deg,
        lon_deg=lon_deg,
        height_m=height_m,
    )


def interpolated_epoch(
    left: comparison.SolutionEpoch,
    right: comparison.SolutionEpoch,
    tow: float,
    left_velocity_ecef_mps: np.ndarray | None = None,
    right_velocity_ecef_mps: np.ndarray | None = None,
    hermite_horizontal_only: bool = False,
) -> comparison.SolutionEpoch:
    alpha = (tow - left.tow) / (right.tow - left.tow)
    linear_ecef = (1.0 - alpha) * np.asarray(left.ecef) + alpha * np.asarray(right.ecef)
    if left_velocity_ecef_mps is None or right_velocity_ecef_mps is None:
        ecef = linear_ecef
    else:
        duration_s = right.tow - left.tow
        h00 = 2.0 * alpha**3 - 3.0 * alpha**2 + 1.0
        h10 = alpha**3 - 2.0 * alpha**2 + alpha
        h01 = -2.0 * alpha**3 + 3.0 * alpha**2
        h11 = alpha**3 - alpha**2
        ecef = (
            h00 * np.asarray(left.ecef)
            + h10 * duration_s * left_velocity_ecef_mps
            + h01 * np.asarray(right.ecef)
            + h11 * duration_s * right_velocity_ecef_mps
        )
        if hermite_horizontal_only:
            lat_deg, lon_deg, _ = ppc_metrics.llh_from_ecef(*linear_ecef)
            lat = math.radians(lat_deg)
            lon = math.radians(lon_deg)
            sin_lat, cos_lat = math.sin(lat), math.cos(lat)
            sin_lon, cos_lon = math.sin(lon), math.cos(lon)
            ecef_to_enu = np.asarray(
                [
                    [-sin_lon, cos_lon, 0.0],
                    [-sin_lat * cos_lon, -sin_lat * sin_lon, cos_lat],
                    [cos_lat * cos_lon, cos_lat * sin_lon, sin_lat],
                ]
            )
            hermite_delta_enu = ecef_to_enu @ (ecef - linear_ecef)
            hermite_delta_enu[2] = 0.0
            ecef = linear_ecef + ecef_to_enu.transpose() @ hermite_delta_enu
    lat_deg, lon_deg, height_m = ppc_metrics.llh_from_ecef(*ecef)
    return comparison.SolutionEpoch(
        week=left.week,
        tow=tow,
        lat_deg=lat_deg,
        lon_deg=lon_deg,
        height_m=height_m,
        ecef=ecef,
        status=3,
        num_satellites=0,
    )


def bridge_epochs(
    epochs: list[comparison.SolutionEpoch],
    args: argparse.Namespace,
) -> tuple[list[comparison.SolutionEpoch], dict[str, object]]:
    if not epochs:
        raise SystemExit("baseline POS contains no solution epochs")
    ordered = sorted(epochs, key=epoch_key)
    interval_s = (
        args.nominal_interval_s
        if args.nominal_interval_s > 0.0
        else infer_nominal_interval_s(ordered)
    )
    selected = {epoch_key(epoch): epoch for epoch in ordered}
    fill_between_all_positions = bool(
        getattr(args, "fill_between_all_positions", False)
    )
    anchors = (
        ordered
        if fill_between_all_positions
        else [epoch for epoch in ordered if trusted_anchor(epoch, args)]
    )
    bridge_spans = 0
    generated_epochs = 0
    replaced_nonfixed_epochs = 0
    filled_missing_epochs = 0
    displacement_rejected_epochs = 0

    interpolation = getattr(args, "interpolation", "linear")
    for anchor_index, (left, right) in enumerate(zip(anchors, anchors[1:])):
        if left.week != right.week:
            continue
        gap_s = right.tow - left.tow
        if gap_s <= interval_s * 1.01 or gap_s > args.max_anchor_gap_s:
            continue
        step_count = round(gap_s / interval_s)
        if step_count < 2 or abs(step_count * interval_s - gap_s) > interval_s * 0.1:
            continue
        left_velocity: np.ndarray | None = None
        right_velocity: np.ndarray | None = None
        if interpolation in {"hermite-outer", "hermite-horizontal"}:
            if anchor_index == 0 or anchor_index + 2 >= len(anchors):
                continue
            previous = anchors[anchor_index - 1]
            following = anchors[anchor_index + 2]
            if previous.week != left.week or following.week != right.week:
                continue
            left_dt_s = left.tow - previous.tow
            right_dt_s = following.tow - right.tow
            if left_dt_s <= 0.0 or right_dt_s <= 0.0:
                continue
            left_velocity = (np.asarray(left.ecef) - np.asarray(previous.ecef)) / left_dt_s
            right_velocity = (np.asarray(following.ecef) - np.asarray(right.ecef)) / right_dt_s
        span_generated = 0
        for step in range(1, step_count):
            tow = left.tow + step * interval_s
            key = (left.week, round(tow * 1000.0))
            existing = selected.get(key)
            if existing is not None and existing.status == 4:
                continue
            if existing is None and getattr(args, "no_fill_missing", False):
                continue
            if existing is not None and not args.replace_nonfixed:
                continue
            candidate = interpolated_epoch(
                left,
                right,
                tow,
                left_velocity,
                right_velocity,
                interpolation == "hermite-horizontal",
            )
            if existing is not None and getattr(args, "preserve_existing_up", False):
                candidate = preserve_up_component(existing, candidate)
            if existing is not None and getattr(
                args, "preserve_existing_horizontal", False
            ):
                candidate = preserve_horizontal_component(existing, candidate)
            max_displacement_m = getattr(args, "max_replacement_displacement_m", 0.0)
            if (
                existing is not None
                and max_displacement_m > 0.0
                and float(np.linalg.norm(candidate.ecef - existing.ecef)) > max_displacement_m
            ):
                displacement_rejected_epochs += 1
                continue
            selected[key] = candidate
            generated_epochs += 1
            span_generated += 1
            if existing is None:
                filled_missing_epochs += 1
            else:
                replaced_nonfixed_epochs += 1
        if span_generated:
            bridge_spans += 1

    payload: dict[str, object] = {
        "reference_truth_used": False,
        "nominal_interval_s": interval_s,
        "input_epochs": len(ordered),
        "output_epochs": len(selected),
        "trusted_anchor_epochs": len(anchors),
        "bridge_spans": bridge_spans,
        "generated_epochs": generated_epochs,
        "filled_missing_epochs": filled_missing_epochs,
        "displacement_rejected_epochs": displacement_rejected_epochs,
        "replaced_nonfixed_epochs": replaced_nonfixed_epochs,
        "preserved_fixed_epochs": sum(epoch.status == 4 for epoch in ordered),
        "max_anchor_gap_s": args.max_anchor_gap_s,
        "replace_nonfixed": args.replace_nonfixed,
        "fill_missing": not getattr(args, "no_fill_missing", False),
        "fill_between_all_positions": fill_between_all_positions,
        "preserve_existing_up": getattr(args, "preserve_existing_up", False),
        "preserve_existing_horizontal": getattr(
            args, "preserve_existing_horizontal", False
        ),
        "max_replacement_displacement_m": getattr(
            args, "max_replacement_displacement_m", 0.0
        ),
        "anchor_min_ratio": args.anchor_min_ratio,
        "anchor_min_satellites": args.anchor_min_satellites,
        "anchor_max_post_rms_m": args.anchor_max_post_rms_m,
        "anchor_max_nis_per_observation": args.anchor_max_nis_per_observation,
        "bridge_status": "FLOAT",
        "interpolation": interpolation,
    }
    return [selected[key] for key in sorted(selected)], payload


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--baseline-pos", type=Path, required=True)
    parser.add_argument("--output-pos", type=Path, required=True)
    parser.add_argument("--summary-json", type=Path)
    parser.add_argument("--max-anchor-gap-s", type=float, default=3.0)
    parser.add_argument("--nominal-interval-s", type=float, default=0.0)
    parser.add_argument("--anchor-min-ratio", type=float, default=2.0)
    parser.add_argument("--anchor-min-satellites", type=int, default=0)
    parser.add_argument("--anchor-max-post-rms-m", type=float, default=0.8)
    parser.add_argument("--anchor-max-nis-per-observation", type=float, default=5.0)
    parser.add_argument("--replace-nonfixed", action="store_true")
    parser.add_argument(
        "--fill-between-all-positions",
        action="store_true",
        help=(
            "Use every emitted solution as a position anchor when filling missing "
            "epochs. This is intended for short dropout interpolation and cannot "
            "be combined with --replace-nonfixed."
        ),
    )
    parser.add_argument(
        "--no-fill-missing",
        action="store_true",
        help="Preserve the input epoch grid; only replace existing non-FIX epochs.",
    )
    parser.add_argument(
        "--preserve-existing-up",
        action="store_true",
        help="For replaced epochs, apply only the interpolation's local-horizontal correction.",
    )
    parser.add_argument(
        "--preserve-existing-horizontal",
        action="store_true",
        help="For replaced epochs, apply only the interpolation's local-Up correction.",
    )
    parser.add_argument(
        "--max-replacement-displacement-m",
        type=float,
        default=0.0,
        help="Reject a replacement farther than this from the existing epoch; 0 disables.",
    )
    parser.add_argument(
        "--interpolation",
        choices=("linear", "hermite-outer", "hermite-horizontal"),
        default="linear",
        help="ECEF interpolation model between trusted FIX anchors.",
    )
    args = parser.parse_args(argv)
    for name in (
        "max_anchor_gap_s",
        "nominal_interval_s",
        "anchor_min_ratio",
        "anchor_max_post_rms_m",
        "anchor_max_nis_per_observation",
        "max_replacement_displacement_m",
    ):
        if getattr(args, name) < 0.0:
            raise SystemExit(f"--{name.replace('_', '-')} must be non-negative")
    if args.anchor_min_satellites < 0:
        raise SystemExit("--anchor-min-satellites must be non-negative")
    if args.preserve_existing_up and args.preserve_existing_horizontal:
        raise SystemExit(
            "--preserve-existing-up and --preserve-existing-horizontal are mutually exclusive"
        )
    if args.fill_between_all_positions and args.replace_nonfixed:
        raise SystemExit(
            "--fill-between-all-positions cannot be combined with --replace-nonfixed"
        )
    return args


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    epochs = comparison.read_libgnss_pos(args.baseline_pos)
    bridged, summary = bridge_epochs(epochs, args)
    pos_writer.write_pos(args.output_pos, bridged)
    if args.summary_json is not None:
        args.summary_json.parent.mkdir(parents=True, exist_ok=True)
        args.summary_json.write_text(
            json.dumps(summary, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
    else:
        print(json.dumps(summary, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
