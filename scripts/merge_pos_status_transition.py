#!/usr/bin/env python3
"""Merge candidate positions on an exact status transition without reference truth."""

from __future__ import annotations

import argparse
import dataclasses
import json
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
import bridge_pos_fixed_anchors as fixed_anchor_bridge  # noqa: E402
import generate_driving_comparison as comparison  # noqa: E402


def epoch_key(epoch: comparison.SolutionEpoch) -> tuple[int, int]:
    return epoch.week, round(epoch.tow * 1000.0)


def merge_positions(
    baseline: list[comparison.SolutionEpoch],
    candidate: list[comparison.SolutionEpoch],
    baseline_status: int | None,
    candidate_status: int,
    max_displacement_m: float = 0.0,
    max_baseline_satellites: int | None = None,
    preserve_existing_up: bool = False,
    min_candidate_ratio: float = 0.0,
    promote_candidate_status: bool = False,
) -> tuple[list[comparison.SolutionEpoch], dict[str, object]]:
    candidate_by_key = {epoch_key(epoch): epoch for epoch in candidate}
    merged: list[comparison.SolutionEpoch] = []
    replaced = 0
    displacement_rejected = 0
    telemetry_rejected = 0
    ratio_rejected = 0
    for epoch in baseline:
        other = candidate_by_key.get(epoch_key(epoch))
        if (
            (baseline_status is None or epoch.status == baseline_status)
            and other is not None
            and other.status == candidate_status
        ):
            if (
                max_baseline_satellites is not None
                and epoch.num_satellites > max_baseline_satellites
            ):
                telemetry_rejected += 1
                merged.append(epoch)
                continue
            candidate_ratio = float(other.ratio or 0.0)
            if candidate_ratio < min_candidate_ratio:
                ratio_rejected += 1
                merged.append(epoch)
                continue
            displacement_m = float(np.linalg.norm(other.ecef - epoch.ecef))
            if max_displacement_m > 0.0 and displacement_m > max_displacement_m:
                displacement_rejected += 1
                merged.append(epoch)
                continue
            position_source = (
                fixed_anchor_bridge.preserve_up_component(epoch, other)
                if preserve_existing_up
                else other
            )
            epoch = dataclasses.replace(
                epoch,
                ecef=position_source.ecef,
                lat_deg=position_source.lat_deg,
                lon_deg=position_source.lon_deg,
                height_m=position_source.height_m,
                status=other.status if promote_candidate_status else epoch.status,
                num_satellites=(
                    other.num_satellites if promote_candidate_status else epoch.num_satellites
                ),
                ratio=other.ratio if promote_candidate_status else epoch.ratio,
            )
            replaced += 1
        merged.append(epoch)
    return merged, {
        "reference_truth_used": False,
        "input_epochs": len(baseline),
        "candidate_epochs": len(candidate),
        "output_epochs": len(merged),
        "baseline_status": baseline_status,
        "candidate_status": candidate_status,
        "replaced_positions": replaced,
        "displacement_rejected_positions": displacement_rejected,
        "telemetry_rejected_positions": telemetry_rejected,
        "ratio_rejected_positions": ratio_rejected,
        "max_displacement_m": max_displacement_m,
        "max_baseline_satellites": max_baseline_satellites,
        "preserve_existing_up": preserve_existing_up,
        "min_candidate_ratio": min_candidate_ratio,
        "promote_candidate_status": promote_candidate_status,
        "preserved_status_labels": not promote_candidate_status,
        "preserved_epoch_grid": True,
        "preserved_baseline_telemetry": not promote_candidate_status,
    }


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--baseline-pos", type=Path, required=True)
    parser.add_argument("--candidate-pos", type=Path, required=True)
    parser.add_argument("--output-pos", type=Path, required=True)
    parser.add_argument("--summary-json", type=Path)
    parser.add_argument(
        "--baseline-status",
        type=int,
        help="Baseline status to replace; omit to allow every baseline status.",
    )
    parser.add_argument("--candidate-status", type=int, required=True)
    parser.add_argument("--max-displacement-m", type=float, default=0.0)
    parser.add_argument("--max-baseline-satellites", type=int)
    parser.add_argument("--min-candidate-ratio", type=float, default=0.0)
    parser.add_argument(
        "--promote-candidate-status",
        action="store_true",
        help="Also adopt the candidate status, satellite count, and ratio.",
    )
    parser.add_argument(
        "--preserve-existing-up",
        action="store_true",
        help="Apply only the candidate's local-horizontal correction.",
    )
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    baseline = comparison.read_libgnss_pos(args.baseline_pos)
    candidate = comparison.read_libgnss_pos(args.candidate_pos)
    if args.max_displacement_m < 0.0:
        raise SystemExit("--max-displacement-m must be non-negative")
    if args.max_baseline_satellites is not None and args.max_baseline_satellites < 0:
        raise SystemExit("--max-baseline-satellites must be non-negative")
    if args.min_candidate_ratio < 0.0:
        raise SystemExit("--min-candidate-ratio must be non-negative")
    merged, summary = merge_positions(
        baseline,
        candidate,
        args.baseline_status,
        args.candidate_status,
        args.max_displacement_m,
        args.max_baseline_satellites,
        args.preserve_existing_up,
        args.min_candidate_ratio,
        args.promote_candidate_status,
    )
    pos_writer.write_pos(args.output_pos, merged)
    if args.summary_json is not None:
        args.summary_json.parent.mkdir(parents=True, exist_ok=True)
        args.summary_json.write_text(
            json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
    else:
        print(json.dumps(summary, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
