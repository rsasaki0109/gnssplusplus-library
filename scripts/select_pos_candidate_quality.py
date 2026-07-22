#!/usr/bin/env python3
"""Select alternate POS positions using baseline/candidate quality telemetry.

The selector is reference-free and keeps the baseline epoch grid and telemetry.
A matched candidate contributes its position when all configured
baseline/candidate quality and separation gates pass. Candidate-status or
explicit-status replacement is opt-in for recovery candidates that
intentionally emit FLOAT.
"""

from __future__ import annotations

import argparse
import bisect
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
import generate_driving_comparison as comparison  # noqa: E402


def candidate_passes(
    baseline: comparison.SolutionEpoch,
    candidate: comparison.SolutionEpoch,
    args: argparse.Namespace,
) -> bool:
    baseline_prefit_rms = baseline.rtk_update_prefit_residual_rms_m
    min_baseline_prefit_rms = getattr(args, "baseline_min_prefit_rms_m", 0.0)
    if min_baseline_prefit_rms > 0.0 and (
        baseline_prefit_rms is None or baseline_prefit_rms <= min_baseline_prefit_rms
    ):
        return False
    baseline_outliers = baseline.rtk_update_suppressed_outliers
    min_baseline_outliers = getattr(args, "baseline_min_outliers", 0)
    if min_baseline_outliers > 0 and (
        baseline_outliers is None or baseline_outliers < min_baseline_outliers
    ):
        return False
    min_baseline_satellites = getattr(args, "baseline_min_satellites", 0)
    if min_baseline_satellites > 0 and baseline.num_satellites < min_baseline_satellites:
        return False
    if candidate.status != args.candidate_status:
        return False
    if candidate.num_satellites < args.candidate_min_satellites:
        return False
    if candidate.ratio is None or candidate.ratio < args.candidate_min_ratio:
        return False
    candidate_max_ratio = getattr(args, "candidate_max_ratio", 0.0)
    if candidate_max_ratio > 0.0 and candidate.ratio > candidate_max_ratio:
        return False
    post_rms = candidate.rtk_update_post_suppression_residual_rms_m
    if (
        args.candidate_max_post_rms_m > 0.0
        and (post_rms is None or post_rms > args.candidate_max_post_rms_m)
    ):
        return False
    nis_per_observation = candidate.rtk_update_normalized_innovation_squared_per_observation
    if (
        args.candidate_max_nis_per_observation > 0.0
        and (
            nis_per_observation is None
            or nis_per_observation > args.candidate_max_nis_per_observation
        )
    ):
        return False
    separation_m = float(
        np.linalg.norm(np.asarray(candidate.ecef) - np.asarray(baseline.ecef))
    )
    if separation_m < args.min_position_separation_m:
        return False
    if args.max_position_separation_m > 0.0 and separation_m > args.max_position_separation_m:
        return False
    return True


def select_candidate_positions(
    baseline_epochs: list[comparison.SolutionEpoch],
    candidate_epochs: list[comparison.SolutionEpoch],
    args: argparse.Namespace,
) -> tuple[list[comparison.SolutionEpoch], dict[str, object]]:
    if not baseline_epochs or not candidate_epochs:
        raise ValueError("baseline and candidate trajectories must be non-empty")
    candidates = sorted(candidate_epochs, key=lambda epoch: (epoch.week, epoch.tow))
    candidate_keys = [(epoch.week, epoch.tow) for epoch in candidates]
    output: list[comparison.SolutionEpoch] = []
    matched_epochs = 0
    selected_epochs = 0
    for baseline in sorted(baseline_epochs, key=lambda epoch: (epoch.week, epoch.tow)):
        key = (baseline.week, baseline.tow)
        index = bisect.bisect_left(candidate_keys, key)
        nearby = [
            candidates[i]
            for i in (index - 1, index)
            if 0 <= i < len(candidates)
            and candidates[i].week == baseline.week
            and abs(candidates[i].tow - baseline.tow) <= args.match_tolerance_s
        ]
        if not nearby:
            output.append(baseline)
            continue
        candidate = min(nearby, key=lambda epoch: abs(epoch.tow - baseline.tow))
        matched_epochs += 1
        if not candidate_passes(baseline, candidate, args):
            output.append(baseline)
            continue
        replacements: dict[str, object] = {
            "ecef": np.asarray(candidate.ecef).copy(),
            "lat_deg": candidate.lat_deg,
            "lon_deg": candidate.lon_deg,
            "height_m": candidate.height_m,
        }
        replacement_status = getattr(args, "replacement_status", None)
        if replacement_status is not None:
            replacements["status"] = replacement_status
        elif getattr(args, "replace_status", False):
            replacements["status"] = candidate.status
        output.append(dataclasses.replace(baseline, **replacements))
        selected_epochs += 1
    summary: dict[str, object] = {
        "reference_truth_used": False,
        "input_epochs": len(baseline_epochs),
        "output_epochs": len(output),
        "candidate_epochs": len(candidate_epochs),
        "matched_candidate_epochs": matched_epochs,
        "selected_candidate_positions": selected_epochs,
        "preserved_baseline_epoch_grid": True,
        "preserved_baseline_status": not (
            getattr(args, "replace_status", False)
            or getattr(args, "replacement_status", None) is not None
        ),
        "preserved_baseline_telemetry": True,
        "candidate_status": args.candidate_status,
        "candidate_min_ratio": args.candidate_min_ratio,
        "candidate_max_ratio": getattr(args, "candidate_max_ratio", 0.0),
        "candidate_min_satellites": args.candidate_min_satellites,
        "candidate_max_post_rms_m": args.candidate_max_post_rms_m,
        "candidate_max_nis_per_observation": args.candidate_max_nis_per_observation,
        "baseline_min_prefit_rms_m": getattr(args, "baseline_min_prefit_rms_m", 0.0),
        "baseline_min_outliers": getattr(args, "baseline_min_outliers", 0),
        "baseline_min_satellites": getattr(args, "baseline_min_satellites", 0),
        "replace_status": getattr(args, "replace_status", False),
        "replacement_status": getattr(args, "replacement_status", None),
        "min_position_separation_m": args.min_position_separation_m,
        "max_position_separation_m": args.max_position_separation_m,
        "match_tolerance_s": args.match_tolerance_s,
    }
    return output, summary


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--baseline-pos", type=Path, required=True)
    parser.add_argument("--candidate-pos", type=Path, required=True)
    parser.add_argument("--output-pos", type=Path, required=True)
    parser.add_argument("--summary-json", type=Path)
    parser.add_argument("--match-tolerance-s", type=float, default=0.11)
    parser.add_argument("--candidate-status", type=int, default=4)
    parser.add_argument("--candidate-min-ratio", type=float, default=2.0)
    parser.add_argument("--candidate-max-ratio", type=float, default=0.0)
    parser.add_argument("--candidate-min-satellites", type=int, default=12)
    parser.add_argument("--candidate-max-post-rms-m", type=float, default=1.0)
    parser.add_argument("--candidate-max-nis-per-observation", type=float, default=5.0)
    parser.add_argument("--baseline-min-prefit-rms-m", type=float, default=0.0)
    parser.add_argument("--baseline-min-outliers", type=int, default=0)
    parser.add_argument("--baseline-min-satellites", type=int, default=0)
    status_group = parser.add_mutually_exclusive_group()
    status_group.add_argument("--replace-status", action="store_true")
    status_group.add_argument(
        "--replacement-status",
        type=int,
        help="Emit selected epochs with this explicit status instead of the candidate status.",
    )
    parser.add_argument("--min-position-separation-m", type=float, default=0.5)
    parser.add_argument("--max-position-separation-m", type=float, default=0.0)
    args = parser.parse_args(argv)
    for name in (
        "match_tolerance_s",
        "candidate_min_ratio",
        "candidate_max_ratio",
        "candidate_max_post_rms_m",
        "candidate_max_nis_per_observation",
        "baseline_min_prefit_rms_m",
        "min_position_separation_m",
        "max_position_separation_m",
    ):
        if getattr(args, name) < 0.0:
            raise SystemExit(f"--{name.replace('_', '-')} must be non-negative")
    if args.candidate_min_satellites < 0:
        raise SystemExit("--candidate-min-satellites must be non-negative")
    if args.baseline_min_satellites < 0:
        raise SystemExit("--baseline-min-satellites must be non-negative")
    if args.baseline_min_outliers < 0:
        raise SystemExit("--baseline-min-outliers must be non-negative")
    if args.candidate_max_ratio > 0.0 and args.candidate_min_ratio > args.candidate_max_ratio:
        raise SystemExit("candidate minimum ratio cannot exceed maximum")
    if (
        args.max_position_separation_m > 0.0
        and args.min_position_separation_m > args.max_position_separation_m
    ):
        raise SystemExit("minimum position separation cannot exceed maximum")
    return args


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    baseline = comparison.read_libgnss_pos(args.baseline_pos)
    candidate = comparison.read_libgnss_pos(args.candidate_pos)
    output, summary = select_candidate_positions(baseline, candidate, args)
    summary["baseline_pos"] = str(args.baseline_pos)
    summary["candidate_pos"] = str(args.candidate_pos)
    pos_writer.write_pos(args.output_pos, output)
    payload = json.dumps(summary, indent=2, sort_keys=True) + "\n"
    if args.summary_json is None:
        print(payload, end="")
    else:
        args.summary_json.parent.mkdir(parents=True, exist_ok=True)
        args.summary_json.write_text(payload, encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
