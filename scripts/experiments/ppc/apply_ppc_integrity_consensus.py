#!/usr/bin/env python3
"""Apply a causal, truth-free primary/shadow integrity state machine to POS.

The shadow trajectory is used only to demote primary FIX status while estimators
disagree and to authorize recovery after consecutive agreement. It never
replaces the primary position. Reference truth is neither accepted nor read.
"""

from __future__ import annotations

from _paths import ANALYSIS_DIR, APPS_DIR, COMMANDS_DIR, PPC_DIR, ROOT_DIR, SCRIPTS_DIR

import argparse
import csv
import dataclasses
import json
import math
from pathlib import Path
import sys
from typing import Any

import numpy as np


for command_path in (SCRIPTS_DIR, COMMANDS_DIR, COMMANDS_DIR / "benchmarks"):
    if str(command_path) not in sys.path:
        sys.path.insert(0, str(command_path))

import apply_ppc_dual_profile_selector as pos_writer  # noqa: E402
import generate_driving_comparison as comparison  # noqa: E402


NORMAL = "NORMAL"
SUSPECT = "SUSPECT"
QUARANTINE = "QUARANTINE"
RECOVERY = "RECOVERY"


@dataclasses.dataclass(frozen=True)
class ShadowEpoch:
    tow: float
    ecef: np.ndarray
    status: str
    gdop: float | None
    ddpr_rms_m: float | None
    nsat: int | None


def optional_float(value: object) -> float | None:
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return None
    return parsed if math.isfinite(parsed) else None


def optional_int(value: object) -> int | None:
    parsed = optional_float(value)
    return int(parsed) if parsed is not None else None


def load_shadow(paths: list[Path]) -> dict[float, ShadowEpoch]:
    epochs: dict[float, ShadowEpoch] = {}
    for path in paths:
        with path.open(newline="", encoding="utf-8") as handle:
            for row in csv.DictReader(handle):
                try:
                    tow = round(float(row["tow"]), 3)
                    ecef = np.asarray(
                        [float(row[name]) for name in ("x_ecef_m", "y_ecef_m", "z_ecef_m")]
                    )
                except (KeyError, TypeError, ValueError):
                    continue
                if np.all(np.isfinite(ecef)):
                    epochs[tow] = ShadowEpoch(
                        tow=tow,
                        ecef=ecef,
                        status=row.get("status", ""),
                        gdop=optional_float(row.get("gdop")),
                        ddpr_rms_m=optional_float(row.get("ddpr_rms_m")),
                        nsat=optional_int(row.get("nsat")),
                    )
    return epochs


def load_debug(path: Path | None) -> dict[tuple[int, float], dict[str, str]]:
    if path is None:
        return {}
    rows: dict[tuple[int, float], dict[str, str]] = {}
    with path.open(newline="", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            week = optional_int(row.get("gps_week"))
            tow = optional_float(row.get("tow"))
            if week is not None and tow is not None:
                rows[(week, round(tow, 3))] = row
    return rows


def shadow_healthy(shadow: ShadowEpoch, args: argparse.Namespace) -> bool:
    return (
        shadow.status in {"FLOAT", "FIXED"}
        and shadow.gdop is not None
        and shadow.gdop <= args.shadow_max_gdop
        and shadow.ddpr_rms_m is not None
        and shadow.ddpr_rms_m <= args.shadow_max_ddpr_rms_m
        and shadow.nsat is not None
        and shadow.nsat >= args.shadow_min_satellites
    )


def primary_suspect(
    epoch: comparison.SolutionEpoch,
    debug: dict[str, str] | None,
    args: argparse.Namespace,
) -> tuple[bool, bool]:
    prefit = epoch.rtk_update_prefit_residual_rms_m
    outliers = epoch.rtk_update_suppressed_outliers
    residual_suspect = (
        prefit is not None
        and prefit > args.primary_max_prefit_rms_m
        and outliers is not None
        and outliers >= args.primary_min_outliers
    )
    covariance_trace = optional_float(
        debug.get("float_position_covariance_trace_m2") if debug else None
    )
    hard = (
        residual_suspect
        and covariance_trace is not None
        and covariance_trace <= args.primary_max_covariance_trace_m2
    )
    return residual_suspect, hard


def recovery_candidate_healthy(
    epoch: comparison.SolutionEpoch,
    separation_m: float | None,
    args: argparse.Namespace,
) -> bool:
    """Gate provisional FIX output without authorizing anchor promotion."""
    if not getattr(args, "allow_recovery_fixed_output", False):
        return False
    prefit = epoch.rtk_update_prefit_residual_rms_m
    outliers = epoch.rtk_update_suppressed_outliers
    return (
        separation_m is not None
        and separation_m <= args.recovery_max_separation_m
        and epoch.ratio is not None
        and epoch.ratio >= args.recovery_min_ratio
        and prefit is not None
        and prefit <= args.recovery_max_prefit_rms_m
        and outliers is not None
        and outliers <= args.recovery_max_outliers
    )


def apply_consensus(
    primary: list[comparison.SolutionEpoch],
    shadows: dict[float, ShadowEpoch],
    debug: dict[tuple[int, float], dict[str, str]],
    args: argparse.Namespace,
) -> tuple[list[comparison.SolutionEpoch], dict[str, Any], list[dict[str, Any]]]:
    state = NORMAL
    suspect_count = 0
    disagreement_count = 0
    recovery_count = 0
    output: list[comparison.SolutionEpoch] = []
    ledger: list[dict[str, Any]] = []
    demoted = 0
    transitions: dict[str, int] = {}
    shadow_healthy_epochs = 0

    for epoch in sorted(primary, key=lambda item: (item.week, item.tow)):
        previous_state = state
        shadow = shadows.get(round(epoch.tow, 3))
        healthy = shadow is not None and shadow_healthy(shadow, args)
        if healthy:
            shadow_healthy_epochs += 1
        separation = (
            float(np.linalg.norm(np.asarray(epoch.ecef) - shadow.ecef))
            if healthy and shadow is not None
            else None
        )
        agree = separation is not None and separation <= args.agreement_aperture_m
        fixed_candidate = epoch.status == 4
        suspect, hard_suspect = primary_suspect(
            epoch, debug.get((epoch.week, round(epoch.tow, 3))), args
        )
        disagreement = (
            fixed_candidate
            and separation is not None
            and not agree
            and (
                not getattr(args, "require_primary_suspect_for_disagreement", False)
                or suspect
            )
        )
        effective_suspect = suspect and (
            (healthy and not agree)
            or getattr(args, "allow_soft_suspect_without_shadow", False)
        )
        suspect_count = suspect_count + 1 if effective_suspect else 0
        disagreement_count = disagreement_count + 1 if disagreement else 0

        request_reset = False
        promote_anchor = False
        if state == NORMAL:
            if hard_suspect:
                state = QUARANTINE
                recovery_count = 0
                request_reset = True
            elif (
                suspect_count >= args.suspect_streak
                or disagreement_count >= args.suspect_streak
            ):
                state = SUSPECT
        elif state == SUSPECT:
            if (
                hard_suspect
                or suspect_count >= args.quarantine_streak
                or disagreement_count >= args.quarantine_streak
            ):
                state = QUARANTINE
                recovery_count = 0
                request_reset = True
            elif not effective_suspect and not disagreement:
                state = NORMAL
        elif state == QUARANTINE:
            if fixed_candidate and agree and not effective_suspect:
                state = RECOVERY
                recovery_count = 1
        elif state == RECOVERY:
            if effective_suspect or (fixed_candidate and not agree):
                state = QUARANTINE
                recovery_count = 0
            elif fixed_candidate and agree:
                recovery_count += 1
                if recovery_count >= args.recovery_streak:
                    state = NORMAL
                    recovery_count = 0
                    suspect_count = 0
                    disagreement_count = 0
                    promote_anchor = True

        healthy_recovery_candidate = (
            state == RECOVERY
            and fixed_candidate
            and agree
            and recovery_candidate_healthy(epoch, separation, args)
        )
        emitted = epoch
        if epoch.status == 4 and state != NORMAL and not healthy_recovery_candidate:
            emitted = dataclasses.replace(epoch, status=3)
            demoted += 1
        output.append(emitted)
        if state != previous_state:
            key = f"{previous_state}->{state}"
            transitions[key] = transitions.get(key, 0) + 1
        ledger.append(
            {
                "week": epoch.week,
                "tow_s": epoch.tow,
                "state": state,
                "primary_status": epoch.status,
                "emitted_status": emitted.status,
                "shadow_healthy": healthy,
                "separation_m": round(separation, 6) if separation is not None else None,
                "agree": agree,
                "primary_suspect": suspect,
                "effective_primary_suspect": effective_suspect,
                "hard_primary_suspect": hard_suspect,
                "request_primary_reset": request_reset,
                "promote_joint_anchor": promote_anchor,
                "provisional_recovery_fixed": healthy_recovery_candidate,
                "recovery_streak": recovery_count,
            }
        )

    summary: dict[str, Any] = {
        "reference_truth_used": False,
        "runtime_truth_used": False,
        "positions_replaced": 0,
        "input_epochs": len(primary),
        "output_epochs": len(output),
        "shadow_epochs": len(shadows),
        "shadow_healthy_epochs": shadow_healthy_epochs,
        "fixed_status_demotions": demoted,
        "state_transitions": transitions,
        "final_state": state,
        "agreement_aperture_m": args.agreement_aperture_m,
        "suspect_streak": args.suspect_streak,
        "quarantine_streak": args.quarantine_streak,
        "recovery_streak": args.recovery_streak,
        "shadow_max_gdop": args.shadow_max_gdop,
        "shadow_max_ddpr_rms_m": args.shadow_max_ddpr_rms_m,
        "shadow_min_satellites": args.shadow_min_satellites,
        "primary_max_prefit_rms_m": args.primary_max_prefit_rms_m,
        "primary_min_outliers": args.primary_min_outliers,
        "primary_max_covariance_trace_m2": args.primary_max_covariance_trace_m2,
        "soft_suspect_requires_independent": not getattr(
            args, "allow_soft_suspect_without_shadow", False
        ),
        "disagreement_requires_primary_suspect": getattr(
            args, "require_primary_suspect_for_disagreement", False
        ),
        "allow_recovery_fixed_output": getattr(
            args, "allow_recovery_fixed_output", False
        ),
        "recovery_min_ratio": getattr(args, "recovery_min_ratio", None),
        "recovery_max_separation_m": getattr(
            args, "recovery_max_separation_m", None
        ),
        "recovery_max_prefit_rms_m": getattr(
            args, "recovery_max_prefit_rms_m", None
        ),
        "recovery_max_outliers": getattr(args, "recovery_max_outliers", None),
    }
    return output, summary, ledger


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--primary-pos", type=Path, required=True)
    parser.add_argument("--shadow-csv", type=Path, action="append", required=True)
    parser.add_argument("--primary-debug-log", type=Path)
    parser.add_argument("--output-pos", type=Path, required=True)
    parser.add_argument("--summary-json", type=Path, required=True)
    parser.add_argument("--ledger-csv", type=Path)
    parser.add_argument("--agreement-aperture-m", type=float, default=10.0)
    parser.add_argument("--suspect-streak", type=int, default=2)
    parser.add_argument("--quarantine-streak", type=int, default=3)
    parser.add_argument("--recovery-streak", type=int, default=5)
    parser.add_argument("--shadow-max-gdop", type=float, default=3.0)
    parser.add_argument("--shadow-max-ddpr-rms-m", type=float, default=40.0)
    parser.add_argument("--shadow-min-satellites", type=int, default=8)
    parser.add_argument("--primary-max-prefit-rms-m", type=float, default=10.0)
    parser.add_argument("--primary-min-outliers", type=int, default=35)
    parser.add_argument("--primary-max-covariance-trace-m2", type=float, default=0.01)
    parser.add_argument(
        "--allow-soft-suspect-without-shadow",
        action="store_true",
        help=(
            "legacy diagnostic behavior: allow soft primary telemetry alone to "
            "start quarantine even though independent recovery is unavailable"
        ),
    )
    parser.add_argument(
        "--require-primary-suspect-for-disagreement",
        action="store_true",
        help="advance the disagreement streak only when primary telemetry is suspect",
    )
    parser.add_argument(
        "--allow-recovery-fixed-output",
        action="store_true",
        help="emit a provisional FIX during RECOVERY when all runtime health gates pass",
    )
    parser.add_argument("--recovery-min-ratio", type=float, default=3.0)
    parser.add_argument("--recovery-max-separation-m", type=float, default=6.0)
    parser.add_argument("--recovery-max-prefit-rms-m", type=float, default=5.0)
    parser.add_argument("--recovery-max-outliers", type=int, default=20)
    args = parser.parse_args(argv)
    for name in (
        "agreement_aperture_m",
        "shadow_max_gdop",
        "shadow_max_ddpr_rms_m",
        "primary_max_prefit_rms_m",
        "primary_max_covariance_trace_m2",
        "recovery_min_ratio",
        "recovery_max_separation_m",
        "recovery_max_prefit_rms_m",
    ):
        if getattr(args, name) <= 0.0:
            raise SystemExit(f"--{name.replace('_', '-')} must be positive")
    for name in (
        "suspect_streak",
        "quarantine_streak",
        "recovery_streak",
        "shadow_min_satellites",
        "primary_min_outliers",
        "recovery_max_outliers",
    ):
        if getattr(args, name) < 1:
            raise SystemExit(f"--{name.replace('_', '-')} must be >= 1")
    return args


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    primary = comparison.read_libgnss_pos(args.primary_pos)
    shadows = load_shadow(args.shadow_csv)
    debug = load_debug(args.primary_debug_log)
    output, summary, ledger = apply_consensus(primary, shadows, debug, args)
    summary.update(
        {
            "primary_pos": str(args.primary_pos),
            "shadow_csv": [str(path) for path in args.shadow_csv],
            "primary_debug_log": (
                str(args.primary_debug_log) if args.primary_debug_log else None
            ),
        }
    )
    pos_writer.write_pos(args.output_pos, output)
    args.summary_json.parent.mkdir(parents=True, exist_ok=True)
    args.summary_json.write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    if args.ledger_csv is not None:
        args.ledger_csv.parent.mkdir(parents=True, exist_ok=True)
        with args.ledger_csv.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=list(ledger[0]) if ledger else [])
            if ledger:
                writer.writeheader()
                writer.writerows(ledger)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
