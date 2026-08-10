#!/usr/bin/env python3
"""Offline audit of whether an FGO shadow may safely correct KF FIX positions.

Reference truth is required and is used only by this evaluator.  Every proposed
runtime gate is based on solver outputs: shadow status/health and KF--FGO
separation.  The report separates useful recoveries from unsafe replacements.
"""

from __future__ import annotations

import argparse
import bisect
import json
from pathlib import Path
import sys
from typing import Any

import numpy as np


ROOT_DIR = Path(__file__).resolve().parents[2]
SCRIPTS_DIR = ROOT_DIR / "scripts"
ANALYSIS_DIR = SCRIPTS_DIR / "analysis"
COMMANDS_DIR = ROOT_DIR / "apps" / "commands"
for command_path in (SCRIPTS_DIR, ANALYSIS_DIR, COMMANDS_DIR, COMMANDS_DIR / "benchmarks"):
    if str(command_path) not in sys.path:
        sys.path.insert(0, str(command_path))

import apply_ppc_integrity_consensus as consensus  # noqa: E402
import generate_driving_comparison as comparison  # noqa: E402


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--primary-pos", type=Path, required=True)
    parser.add_argument("--shadow-csv", type=Path, action="append", required=True)
    parser.add_argument("--reference-csv", type=Path, required=True)
    parser.add_argument("--output-json", type=Path, required=True)
    parser.add_argument("--match-tolerance-s", type=float, default=0.11)
    parser.add_argument("--wrong-fix-threshold-m", type=float, default=0.5)
    parser.add_argument("--shadow-max-gdop", type=float, default=4.0)
    parser.add_argument("--shadow-max-ddpr-rms-m", type=float, default=40.0)
    parser.add_argument("--shadow-min-satellites", type=int, default=8)
    parser.add_argument(
        "--aperture-m",
        type=float,
        action="append",
        help="KF--FGO 3D separation aperture; repeat as needed (default: 0.5,1,2,5).",
    )
    args = parser.parse_args(argv)
    if args.match_tolerance_s <= 0.0 or args.wrong_fix_threshold_m <= 0.0:
        raise SystemExit("match tolerance and wrong-FIX threshold must be positive")
    if args.shadow_max_gdop <= 0.0 or args.shadow_max_ddpr_rms_m <= 0.0:
        raise SystemExit("shadow GDOP and DDPR limits must be positive")
    if args.shadow_min_satellites < 1:
        raise SystemExit("shadow minimum satellites must be >= 1")
    args.aperture_m = args.aperture_m or [0.5, 1.0, 2.0, 5.0]
    if any(value <= 0.0 for value in args.aperture_m):
        raise SystemExit("all apertures must be positive")
    return args


def nearest_reference(
    references: list[comparison.ReferenceEpoch],
    tows: list[float],
    week: int,
    tow: float,
    tolerance_s: float,
) -> comparison.ReferenceEpoch | None:
    index = bisect.bisect_left(tows, tow)
    candidates = [candidate for candidate in (index - 1, index) if 0 <= candidate < len(tows)]
    if not candidates:
        return None
    reference = min(candidates, key=lambda candidate: abs(tows[candidate] - tow))
    row = references[reference]
    if row.week != week or abs(row.tow - tow) > tolerance_s:
        return None
    return row


def empty_counts() -> dict[str, int]:
    return {
        "selected_epochs": 0,
        "recoveries": 0,
        "unsafe_replacements": 0,
        "both_correct": 0,
        "both_wrong": 0,
    }


def classify(counts: dict[str, int], primary_wrong: bool, shadow_wrong: bool) -> None:
    counts["selected_epochs"] += 1
    if primary_wrong and not shadow_wrong:
        counts["recoveries"] += 1
    elif not primary_wrong and shadow_wrong:
        counts["unsafe_replacements"] += 1
    elif primary_wrong:
        counts["both_wrong"] += 1
    else:
        counts["both_correct"] += 1


def build_report(args: argparse.Namespace) -> dict[str, Any]:
    primary = comparison.read_libgnss_pos(args.primary_pos)
    shadows = consensus.load_shadow(args.shadow_csv)
    references = sorted(
        comparison.read_reference_csv(args.reference_csv), key=lambda row: row.tow
    )
    reference_tows = [row.tow for row in references]
    overall = empty_counts()
    healthy = empty_counts()
    status = {"FIXED": empty_counts(), "FLOAT": empty_counts()}
    apertures = {
        f"{value:g}": empty_counts() for value in sorted(set(args.aperture_m))
    }
    matched_primary_fixed = 0
    healthy_shadow_epochs = 0

    for epoch in primary:
        if epoch.status != 4:
            continue
        shadow = shadows.get(round(epoch.tow, 3))
        reference = nearest_reference(
            references,
            reference_tows,
            epoch.week,
            epoch.tow,
            args.match_tolerance_s,
        )
        if shadow is None or reference is None:
            continue
        matched_primary_fixed += 1
        primary_error = float(
            np.linalg.norm(
                comparison.ecef_to_enu(
                    epoch.ecef - reference.ecef,
                    reference.lat_deg,
                    reference.lon_deg,
                )
            )
        )
        shadow_error = float(
            np.linalg.norm(
                comparison.ecef_to_enu(
                    shadow.ecef - reference.ecef,
                    reference.lat_deg,
                    reference.lon_deg,
                )
            )
        )
        primary_wrong = primary_error > args.wrong_fix_threshold_m
        shadow_wrong = shadow_error > args.wrong_fix_threshold_m
        separation = float(np.linalg.norm(epoch.ecef - shadow.ecef))
        classify(overall, primary_wrong, shadow_wrong)
        if shadow.status in status:
            classify(status[shadow.status], primary_wrong, shadow_wrong)
        is_healthy = (
            shadow.gdop is not None
            and shadow.gdop <= args.shadow_max_gdop
            and shadow.ddpr_rms_m is not None
            and shadow.ddpr_rms_m <= args.shadow_max_ddpr_rms_m
            and shadow.nsat is not None
            and shadow.nsat >= args.shadow_min_satellites
        )
        if is_healthy:
            healthy_shadow_epochs += 1
            classify(healthy, primary_wrong, shadow_wrong)
            for aperture_text, counts in apertures.items():
                if separation <= float(aperture_text):
                    classify(counts, primary_wrong, shadow_wrong)

    return {
        "schema_version": 1,
        "reference_truth_used": True,
        "runtime_gate_inputs": [
            "shadow_status",
            "shadow_gdop",
            "shadow_ddpr_rms_m",
            "shadow_nsat",
            "kf_fgo_separation_m",
        ],
        "primary_pos": str(args.primary_pos),
        "shadow_csv": [str(path) for path in args.shadow_csv],
        "reference_csv": str(args.reference_csv),
        "wrong_fix_threshold_m": args.wrong_fix_threshold_m,
        "matched_primary_fixed_epochs": matched_primary_fixed,
        "healthy_shadow_epochs": healthy_shadow_epochs,
        "health_gate": {
            "max_gdop": args.shadow_max_gdop,
            "max_ddpr_rms_m": args.shadow_max_ddpr_rms_m,
            "min_satellites": args.shadow_min_satellites,
        },
        "overall": overall,
        "healthy": healthy,
        "by_shadow_status": status,
        "healthy_by_separation_aperture_m": apertures,
    }


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    report = build_report(args)
    args.output_json.parent.mkdir(parents=True, exist_ok=True)
    args.output_json.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report["healthy"], indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
