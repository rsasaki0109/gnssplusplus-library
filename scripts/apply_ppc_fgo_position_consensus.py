#!/usr/bin/env python3
"""Replace a primary FIX position only when independent FGO FIX shadows agree.

The combiner is causal and accepts no reference trajectory.  It preserves the
primary epoch grid, status, and RTK telemetry.  A replacement needs a unique
cluster of independently generated FGO FIX positions plus a minimum separation
from the primary, so one long-lived shadow cannot become recovery authority.
"""

from __future__ import annotations

import argparse
import csv
import dataclasses
import itertools
import json
import math
from pathlib import Path
import sys
from typing import Any

import numpy as np


ROOT_DIR = Path(__file__).resolve().parents[1]
SCRIPTS_DIR = Path(__file__).resolve().parent
COMMANDS_DIR = ROOT_DIR / "apps" / "commands"
for command_path in (SCRIPTS_DIR, COMMANDS_DIR, COMMANDS_DIR / "benchmarks"):
    if str(command_path) not in sys.path:
        sys.path.insert(0, str(command_path))

import apply_ppc_dual_profile_selector as pos_writer  # noqa: E402
import apply_ppc_integrity_consensus as integrity  # noqa: E402
import generate_driving_comparison as comparison  # noqa: E402
import gnss_ppc_metrics as metrics  # noqa: E402


@dataclasses.dataclass(frozen=True)
class TrackEpoch:
    sample: integrity.ShadowEpoch
    age_epochs: int


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--primary-pos", type=Path, required=True)
    parser.add_argument("--shadow-csv", type=Path, action="append", required=True)
    parser.add_argument("--output-pos", type=Path, required=True)
    parser.add_argument("--summary-json", type=Path, required=True)
    parser.add_argument("--ledger-csv", type=Path)
    parser.add_argument("--min-independent-shadows", type=int, default=2)
    parser.add_argument("--shadow-agreement-aperture-m", type=float, default=0.25)
    parser.add_argument("--primary-separation-min-m", type=float, default=0.5)
    parser.add_argument(
        "--fresh-shadow-max-age-epochs",
        type=int,
        default=0,
        help=(
            "Require every authority cluster to include a shadow younger than "
            "this many output epochs; 0 disables the freshness requirement."
        ),
    )
    parser.add_argument(
        "--candidate-max-prediction-error-m",
        type=float,
        default=0.0,
        help=(
            "Reject a consensus position farther than this from the causal "
            "two-epoch constant-velocity prediction; 0 disables the gate."
        ),
    )
    parser.add_argument(
        "--shadow-max-gdop",
        type=float,
        default=0.0,
        help="Optional per-shadow GDOP ceiling; 0 disables it.",
    )
    parser.add_argument(
        "--shadow-max-ddpr-rms-m",
        type=float,
        default=0.0,
        help="Optional per-shadow DDPR RMS ceiling; 0 disables it.",
    )
    parser.add_argument(
        "--shadow-min-satellites",
        type=int,
        default=0,
        help="Optional per-shadow satellite floor; 0 disables it.",
    )
    args = parser.parse_args(argv)
    resolved = [path.resolve() for path in args.shadow_csv]
    if len(set(resolved)) != len(resolved):
        raise SystemExit("--shadow-csv paths must be distinct independent runs")
    if args.min_independent_shadows < 2:
        raise SystemExit("--min-independent-shadows must be >= 2")
    if args.min_independent_shadows > len(args.shadow_csv):
        raise SystemExit("minimum independent shadows exceeds supplied shadow files")
    if args.shadow_agreement_aperture_m <= 0.0:
        raise SystemExit("--shadow-agreement-aperture-m must be positive")
    if args.primary_separation_min_m < 0.0:
        raise SystemExit("--primary-separation-min-m must be non-negative")
    if args.fresh_shadow_max_age_epochs < 0:
        raise SystemExit("--fresh-shadow-max-age-epochs must be non-negative")
    if args.candidate_max_prediction_error_m < 0.0:
        raise SystemExit("--candidate-max-prediction-error-m must be non-negative")
    if args.shadow_max_gdop < 0.0 or args.shadow_max_ddpr_rms_m < 0.0:
        raise SystemExit("optional shadow health ceilings must be non-negative")
    if args.shadow_min_satellites < 0:
        raise SystemExit("--shadow-min-satellites must be non-negative")
    return args


def shadow_passes_health(track_epoch: TrackEpoch, args: argparse.Namespace) -> bool:
    sample = track_epoch.sample
    return (
        sample.status == "FIXED"
        and (
            args.shadow_max_gdop <= 0.0
            or (sample.gdop is not None and sample.gdop <= args.shadow_max_gdop)
        )
        and (
            args.shadow_max_ddpr_rms_m <= 0.0
            or (
                sample.ddpr_rms_m is not None
                and sample.ddpr_rms_m <= args.shadow_max_ddpr_rms_m
            )
        )
        and (
            args.shadow_min_satellites <= 0
            or (
                sample.nsat is not None
                and sample.nsat >= args.shadow_min_satellites
            )
        )
    )


def cluster_diameter(samples: tuple[tuple[int, TrackEpoch], ...]) -> float:
    diameter = 0.0
    for left, right in itertools.combinations(samples, 2):
        diameter = max(
            diameter,
            float(np.linalg.norm(left[1].sample.ecef - right[1].sample.ecef)),
        )
    return diameter


def select_unique_consensus(
    samples: list[tuple[int, TrackEpoch]],
    minimum: int,
    aperture_m: float,
    fresh_max_age_epochs: int = 0,
) -> tuple[tuple[tuple[int, TrackEpoch], ...] | None, float, bool]:
    """Return the largest tight cluster, rejecting equal-size competing clusters."""
    valid: list[tuple[float, tuple[tuple[int, TrackEpoch], ...]]] = []
    for size in range(len(samples), minimum - 1, -1):
        for candidate in itertools.combinations(samples, size):
            diameter = cluster_diameter(candidate)
            has_fresh_shadow = fresh_max_age_epochs <= 0 or any(
                sample.age_epochs < fresh_max_age_epochs for _, sample in candidate
            )
            if diameter <= aperture_m and has_fresh_shadow:
                valid.append((diameter, candidate))
        if valid:
            break
    if not valid:
        return None, math.inf, False
    valid.sort(key=lambda item: (item[0], tuple(sample[0] for sample in item[1])))
    best_diameter, best = valid[0]
    best_ids = {sample[0] for sample in best}
    ambiguous = any(
        {sample[0] for sample in candidate} != best_ids
        for _, candidate in valid[1:]
    )
    return (None, best_diameter, True) if ambiguous else (best, best_diameter, False)


def apply_position_consensus(
    primary: list[comparison.SolutionEpoch],
    tracks: list[dict[float, TrackEpoch]],
    args: argparse.Namespace,
) -> tuple[list[comparison.SolutionEpoch], dict[str, Any], list[dict[str, Any]]]:
    output: list[comparison.SolutionEpoch] = []
    ledger: list[dict[str, Any]] = []
    replacements = 0
    fixed_epochs_with_two_or_more = 0
    no_cluster = 0
    ambiguous_clusters = 0
    below_primary_separation = 0
    prediction_rejections = 0
    prediction_gate_unavailable = 0

    for epoch in primary:
        emitted = epoch
        if epoch.status == 4:
            samples = []
            for track_index, track in enumerate(tracks):
                sample = track.get(round(epoch.tow, 3))
                if sample is not None and shadow_passes_health(sample, args):
                    samples.append((track_index, sample))
            if len(samples) >= args.min_independent_shadows:
                fixed_epochs_with_two_or_more += 1
                cluster, diameter, ambiguous = select_unique_consensus(
                    samples,
                    args.min_independent_shadows,
                    args.shadow_agreement_aperture_m,
                    args.fresh_shadow_max_age_epochs,
                )
                if ambiguous:
                    ambiguous_clusters += 1
                elif cluster is None:
                    no_cluster += 1
                else:
                    consensus_ecef = np.median(
                        np.asarray([sample.sample.ecef for _, sample in cluster]),
                        axis=0,
                    )
                    separation = float(np.linalg.norm(consensus_ecef - epoch.ecef))
                    if separation < args.primary_separation_min_m:
                        below_primary_separation += 1
                    else:
                        prediction_error: float | None = None
                        prediction_limit = float(
                            getattr(args, "candidate_max_prediction_error_m", 0.0)
                        )
                        if prediction_limit > 0.0:
                            if len(output) < 2:
                                prediction_gate_unavailable += 1
                                output.append(emitted)
                                continue
                            older, previous = output[-2], output[-1]
                            history_dt = previous.tow - older.tow
                            current_dt = epoch.tow - previous.tow
                            if history_dt <= 0.0 or current_dt <= 0.0:
                                prediction_gate_unavailable += 1
                                output.append(emitted)
                                continue
                            predicted_ecef = previous.ecef + (
                                previous.ecef - older.ecef
                            ) * (current_dt / history_dt)
                            prediction_error = float(
                                np.linalg.norm(consensus_ecef - predicted_ecef)
                            )
                            if prediction_error > prediction_limit:
                                prediction_rejections += 1
                                output.append(emitted)
                                continue
                        lat, lon, height = metrics.llh_from_ecef(*consensus_ecef)
                        emitted = dataclasses.replace(
                            epoch,
                            ecef=consensus_ecef,
                            lat_deg=lat,
                            lon_deg=lon,
                            height_m=height,
                        )
                        replacements += 1
                        ledger.append(
                            {
                                "week": epoch.week,
                                "tow_s": epoch.tow,
                                "shadow_tracks": ";".join(
                                    str(index) for index, _ in cluster
                                ),
                                "shadow_count": len(cluster),
                                "shadow_diameter_m": diameter,
                                "primary_separation_m": separation,
                                "candidate_prediction_error_m": prediction_error,
                                "x_ecef_m": float(consensus_ecef[0]),
                                "y_ecef_m": float(consensus_ecef[1]),
                                "z_ecef_m": float(consensus_ecef[2]),
                            }
                        )
        output.append(emitted)

    summary = {
        "schema_version": 1,
        "reference_truth_used": False,
        "runtime_truth_used": False,
        "positions_replaced": replacements,
        "statuses_changed": 0,
        "primary_epochs": len(primary),
        "fixed_epochs_with_minimum_shadows": fixed_epochs_with_two_or_more,
        "no_consensus_cluster": no_cluster,
        "ambiguous_consensus_clusters": ambiguous_clusters,
        "below_primary_separation": below_primary_separation,
        "candidate_prediction_rejections": prediction_rejections,
        "prediction_gate_unavailable": prediction_gate_unavailable,
        "min_independent_shadows": args.min_independent_shadows,
        "shadow_agreement_aperture_m": args.shadow_agreement_aperture_m,
        "primary_separation_min_m": args.primary_separation_min_m,
        "fresh_shadow_max_age_epochs": args.fresh_shadow_max_age_epochs,
        "candidate_max_prediction_error_m": float(
            getattr(args, "candidate_max_prediction_error_m", 0.0)
        ),
        "shadow_health_gate": {
            "max_gdop": args.shadow_max_gdop,
            "max_ddpr_rms_m": args.shadow_max_ddpr_rms_m,
            "min_satellites": args.shadow_min_satellites,
        },
    }
    return output, summary, ledger


def write_ledger(path: Path, rows: list[dict[str, Any]]) -> None:
    fields = [
        "week",
        "tow_s",
        "shadow_tracks",
        "shadow_count",
        "shadow_diameter_m",
        "primary_separation_m",
        "candidate_prediction_error_m",
        "x_ecef_m",
        "y_ecef_m",
        "z_ecef_m",
    ]
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields, lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)


def load_track(path: Path) -> dict[float, TrackEpoch]:
    samples = integrity.load_shadow([path])
    return {
        tow: TrackEpoch(sample=sample, age_epochs=age)
        for age, (tow, sample) in enumerate(sorted(samples.items()))
    }


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    primary = comparison.read_libgnss_pos(args.primary_pos)
    tracks = [load_track(path) for path in args.shadow_csv]
    output, summary, ledger = apply_position_consensus(primary, tracks, args)
    summary.update(
        {
            "primary_pos": str(args.primary_pos),
            "shadow_csv": [str(path) for path in args.shadow_csv],
        }
    )
    pos_writer.write_pos(args.output_pos, output)
    args.summary_json.parent.mkdir(parents=True, exist_ok=True)
    args.summary_json.write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    if args.ledger_csv is not None:
        write_ledger(args.ledger_csv, ledger)
    print(json.dumps(summary, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
