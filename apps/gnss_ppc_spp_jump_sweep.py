#!/usr/bin/env python3
"""Post-hoc PPC SPP position-jump gate threshold sweep."""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
import json
import math
from pathlib import Path
import sys


ROOT_DIR = Path(__file__).resolve().parent.parent
SCRIPTS_DIR = ROOT_DIR / "scripts"

if str(SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_DIR))

import generate_driving_comparison as comparison  # noqa: E402
import gnss_ppc_metrics as ppc_metrics  # noqa: E402
from gnss_ppc_demo import read_flexible_reference_csv  # noqa: E402


@dataclass(frozen=True)
class JumpGateResult:
    epochs: list[comparison.SolutionEpoch]
    rejected_epochs: int
    rejected_groups: int
    bridged_epochs: int
    max_rejected_jump_m: float
    max_rejected_jump_rate_mps: float


@dataclass(frozen=True)
class CandidateResult:
    row: dict[str, object]
    epochs: list[comparison.SolutionEpoch]


def parse_float_list(text: str) -> list[float]:
    values: list[float] = []
    for token in text.split(","):
        stripped = token.strip()
        if not stripped:
            continue
        values.append(float(stripped))
    if not values:
        raise argparse.ArgumentTypeError("expected at least one numeric value")
    return values


def llh_from_ecef(ecef: object) -> tuple[float, float, float]:
    return ppc_metrics.llh_from_ecef(
        float(ecef[0]),
        float(ecef[1]),
        float(ecef[2]),
    )


def apply_position_jump_gate(
    epochs: list[comparison.SolutionEpoch],
    max_rate_mps: float,
    min_jump_m: float,
    bridge_max_gap_s: float = 0.0,
    bridge_max_anchor_speed_mps: float = 0.0,
) -> JumpGateResult:
    if max_rate_mps <= 0.0:
        return JumpGateResult(list(epochs), 0, 0, 0, 0.0, 0.0)

    kept: list[comparison.SolutionEpoch] = []
    kept_by_index: dict[int, comparison.SolutionEpoch] = {}
    rejected_indices: list[int] = []
    max_rejected_jump_m = 0.0
    max_rejected_jump_rate_mps = 0.0
    last_kept: comparison.SolutionEpoch | None = None
    for index, epoch in enumerate(epochs):
        if last_kept is None:
            kept.append(epoch)
            kept_by_index[index] = epoch
            last_kept = epoch
            continue

        dt = (epoch.week - last_kept.week) * 604800.0 + (epoch.tow - last_kept.tow)
        if not math.isfinite(dt) or dt <= 0.0:
            kept.append(epoch)
            last_kept = epoch
            continue

        jump_m = float(math.dist(epoch.ecef, last_kept.ecef))
        jump_rate_mps = jump_m / dt
        allowed_jump_m = max(min_jump_m, max_rate_mps * dt)
        if math.isfinite(jump_m) and jump_m > allowed_jump_m:
            rejected_indices.append(index)
            max_rejected_jump_m = max(max_rejected_jump_m, jump_m)
            max_rejected_jump_rate_mps = max(max_rejected_jump_rate_mps, jump_rate_mps)
            continue

        kept.append(epoch)
        kept_by_index[index] = epoch
        last_kept = epoch

    rejected_groups: list[list[int]] = []
    current_group: list[int] = []
    for index in rejected_indices:
        if not current_group or index == current_group[-1] + 1:
            current_group.append(index)
        else:
            rejected_groups.append(current_group)
            current_group = [index]
    if current_group:
        rejected_groups.append(current_group)

    bridged_epochs: list[comparison.SolutionEpoch] = []
    if bridge_max_gap_s > 0.0 and bridge_max_anchor_speed_mps > 0.0:
        for group in rejected_groups:
            before_index = group[0] - 1
            while before_index >= 0 and before_index not in kept_by_index:
                before_index -= 1
            after_index = group[-1] + 1
            while after_index < len(epochs) and after_index not in kept_by_index:
                after_index += 1
            if before_index < 0 or after_index >= len(epochs):
                continue

            before = kept_by_index[before_index]
            after = kept_by_index[after_index]
            total_dt = (after.week - before.week) * 604800.0 + (after.tow - before.tow)
            if not math.isfinite(total_dt) or total_dt <= 0.0 or total_dt > bridge_max_gap_s:
                continue

            anchor_speed_mps = float(math.dist(before.ecef, after.ecef)) / total_dt
            if anchor_speed_mps > bridge_max_anchor_speed_mps:
                continue

            for index in group:
                epoch = epochs[index]
                interp_dt = (epoch.week - before.week) * 604800.0 + (epoch.tow - before.tow)
                fraction = interp_dt / total_dt
                ecef = before.ecef * (1.0 - fraction) + after.ecef * fraction
                lat_deg, lon_deg, height_m = llh_from_ecef(ecef)
                bridged_epochs.append(
                    comparison.SolutionEpoch(
                        week=epoch.week,
                        tow=epoch.tow,
                        lat_deg=lat_deg,
                        lon_deg=lon_deg,
                        height_m=height_m,
                        ecef=ecef,
                        status=1,
                        num_satellites=min(before.num_satellites, after.num_satellites),
                    )
                )

    output_epochs = sorted(kept + bridged_epochs, key=lambda epoch: (epoch.week, epoch.tow))
    return JumpGateResult(
        epochs=output_epochs,
        rejected_epochs=len(rejected_indices),
        rejected_groups=len(rejected_groups),
        bridged_epochs=len(bridged_epochs),
        max_rejected_jump_m=max_rejected_jump_m,
        max_rejected_jump_rate_mps=max_rejected_jump_rate_mps,
    )


def summarize_candidate(
    reference: list[comparison.ReferenceEpoch],
    epochs: list[comparison.SolutionEpoch],
    match_tolerance_s: float,
    max_rate_mps: float | None,
    min_jump_m: float | None,
    rejected_epochs: int,
    rejected_groups: int,
    bridged_epochs: int,
    max_rejected_jump_m: float,
    max_rejected_jump_rate_mps: float,
    bridge_max_gap_s: float,
    bridge_max_anchor_speed_mps: float,
) -> dict[str, object]:
    metrics = ppc_metrics.summarize_solution_epochs(
        reference,
        epochs,
        1,
        "SPP",
        match_tolerance_s,
        None,
    )
    metrics.update(
        {
            "max_position_jump_rate_mps": max_rate_mps,
            "max_position_jump_min_m": min_jump_m,
            "jump_gate_rejected_epochs": rejected_epochs,
            "jump_gate_rejected_groups": rejected_groups,
            "bridge_inserted_epochs": bridged_epochs,
            "bridge_max_gap_s": bridge_max_gap_s,
            "bridge_max_anchor_speed_mps": bridge_max_anchor_speed_mps,
            "jump_gate_max_rejected_jump_m": ppc_metrics.rounded(max_rejected_jump_m),
            "jump_gate_max_rejected_jump_rate_mps": ppc_metrics.rounded(
                max_rejected_jump_rate_mps
            ),
        }
    )
    return metrics


def write_csv(path: Path, rows: list[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = [
        "max_position_jump_rate_mps",
        "max_position_jump_min_m",
        "valid_epochs",
        "matched_epochs",
        "positioning_rate_pct",
        "positioning_drop_pct",
        "median_h_m",
        "p95_h_m",
        "max_h_m",
        "median_abs_up_m",
        "p95_abs_up_m",
        "mean_up_m",
        "mean_satellites",
        "jump_gate_rejected_epochs",
        "jump_gate_rejected_groups",
        "bridge_inserted_epochs",
        "bridge_max_gap_s",
        "bridge_max_anchor_speed_mps",
        "jump_gate_max_rejected_jump_m",
        "jump_gate_max_rejected_jump_rate_mps",
    ]
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow({name: row.get(name) for name in fieldnames})


def write_libgnss_pos(path: Path, epochs: list[comparison.SolutionEpoch]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="ascii") as handle:
        handle.write("% LibGNSS++ Position Solution\n")
        handle.write(
            "% GPS_Week GPS_TOW X(m) Y(m) Z(m) Lat(deg) Lon(deg) Height(m) "
            "Status NumSat PDOP Ratio\n"
        )
        for epoch in epochs:
            ratio = epoch.ratio if epoch.ratio is not None else 0.0
            handle.write(
                f"{epoch.week:d} {epoch.tow:.3f} "
                f"{float(epoch.ecef[0]):.4f} {float(epoch.ecef[1]):.4f} "
                f"{float(epoch.ecef[2]):.4f} "
                f"{epoch.lat_deg:.9f} {epoch.lon_deg:.9f} {epoch.height_m:.4f} "
                f"{epoch.status:d} {epoch.num_satellites:d} 0.00 {ratio:.1f}\n"
            )


def candidate_matches_threshold(
    candidate: CandidateResult,
    rate_mps: float,
    min_jump_m: float,
) -> bool:
    row = candidate.row
    row_rate = row["max_position_jump_rate_mps"]
    row_min_jump = row["max_position_jump_min_m"]
    if row_rate is None or row_min_jump is None:
        return False
    return math.isclose(float(row_rate), rate_mps) and math.isclose(float(row_min_jump), min_jump_m)


def candidate_label(candidate: CandidateResult) -> str:
    rate = candidate.row["max_position_jump_rate_mps"]
    min_jump = candidate.row["max_position_jump_min_m"]
    return "baseline" if rate is None else f"rate={float(rate):g} min={float(min_jump):g}"


def annotate_positioning_drop(candidates: list[CandidateResult]) -> float:
    baseline_positioning_rate = float(candidates[0].row["positioning_rate_pct"])
    for candidate in candidates:
        positioning_rate = float(candidate.row["positioning_rate_pct"])
        candidate.row["positioning_drop_pct"] = ppc_metrics.rounded(
            baseline_positioning_rate - positioning_rate
        )
    return baseline_positioning_rate


def rank_candidates(candidates: list[CandidateResult]) -> list[CandidateResult]:
    return sorted(
        candidates,
        key=lambda candidate: (
            float(candidate.row["p95_h_m"]),
            -float(candidate.row["positioning_rate_pct"]),
        ),
    )


def filter_policy_candidates(
    candidates: list[CandidateResult],
    baseline_positioning_rate_pct: float,
    max_positioning_drop_pct: float | None,
    min_positioning_rate_pct: float | None,
) -> list[CandidateResult]:
    min_allowed_positioning = -math.inf
    if max_positioning_drop_pct is not None:
        min_allowed_positioning = max(
            min_allowed_positioning,
            baseline_positioning_rate_pct - max_positioning_drop_pct,
        )
    if min_positioning_rate_pct is not None:
        min_allowed_positioning = max(min_allowed_positioning, min_positioning_rate_pct)

    return [
        candidate for candidate in candidates
        if float(candidate.row["positioning_rate_pct"]) + 1e-9 >= min_allowed_positioning
    ]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--reference-csv", type=Path, required=True)
    parser.add_argument("--pos", type=Path, required=True, help="Input libgnss++ SPP .pos file")
    parser.add_argument(
        "--rates-mps",
        type=parse_float_list,
        default=parse_float_list("25,50,75,100,150,200"),
        help="Comma-separated max jump rates to sweep in m/s",
    )
    parser.add_argument(
        "--min-jumps-m",
        type=parse_float_list,
        default=parse_float_list("10,20,30,40,50"),
        help="Comma-separated minimum allowed jumps to sweep in meters",
    )
    parser.add_argument("--match-tolerance-s", type=float, default=0.25)
    parser.add_argument(
        "--bridge-max-gap-s",
        type=float,
        default=0.0,
        help="Fill rejected epochs between accepted anchors when the total gap is within this value",
    )
    parser.add_argument(
        "--bridge-max-anchor-speed-mps",
        type=float,
        default=0.0,
        help="Fill rejected epochs only when the accepted anchor-to-anchor speed is below this value",
    )
    parser.add_argument("--summary-json", type=Path, required=True)
    parser.add_argument("--csv", type=Path, required=True)
    parser.add_argument(
        "--max-positioning-drop-pct",
        type=float,
        help="Policy guard: only choose best/filtered candidates whose positioning rate drops by at most this many percentage points",
    )
    parser.add_argument(
        "--min-positioning-rate-pct",
        type=float,
        help="Policy guard: only choose best/filtered candidates whose positioning rate is at least this percentage",
    )
    parser.add_argument(
        "--filtered-pos-out",
        type=Path,
        help=(
            "Write a libgnss++ .pos for the policy/best p95 candidate, or for "
            "the candidate selected by --filtered-rate-mps/--filtered-min-jump-m"
        ),
    )
    parser.add_argument(
        "--filtered-rate-mps",
        type=float,
        help="Select the written .pos candidate by max jump rate",
    )
    parser.add_argument(
        "--filtered-min-jump-m",
        type=float,
        help="Select the written .pos candidate by minimum jump threshold",
    )
    parser.add_argument("--top", type=int, default=8, help="Rows to print after sorting by p95")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if (args.filtered_rate_mps is None) != (args.filtered_min_jump_m is None):
        raise SystemExit("--filtered-rate-mps and --filtered-min-jump-m must be used together")
    if args.max_positioning_drop_pct is not None and args.max_positioning_drop_pct < 0.0:
        raise SystemExit("--max-positioning-drop-pct must be >= 0")
    if args.min_positioning_rate_pct is not None and not (0.0 <= args.min_positioning_rate_pct <= 100.0):
        raise SystemExit("--min-positioning-rate-pct must be in [0, 100]")

    reference = read_flexible_reference_csv(args.reference_csv)
    epochs = comparison.read_libgnss_pos(args.pos)
    if not epochs:
        raise SystemExit(f"No solution epochs found in {args.pos}")

    candidates: list[CandidateResult] = [
        CandidateResult(
            row=summarize_candidate(
                reference,
                epochs,
                args.match_tolerance_s,
                None,
                None,
                0,
                0,
                0,
                0.0,
                0.0,
                args.bridge_max_gap_s,
                args.bridge_max_anchor_speed_mps,
            ),
            epochs=list(epochs),
        )
    ]
    for rate in args.rates_mps:
        for min_jump in args.min_jumps_m:
            gate = apply_position_jump_gate(
                epochs,
                rate,
                min_jump,
                args.bridge_max_gap_s,
                args.bridge_max_anchor_speed_mps,
            )
            candidates.append(
                CandidateResult(
                    row=summarize_candidate(
                        reference,
                        gate.epochs,
                        args.match_tolerance_s,
                        rate,
                        min_jump,
                        gate.rejected_epochs,
                        gate.rejected_groups,
                        gate.bridged_epochs,
                        gate.max_rejected_jump_m,
                        gate.max_rejected_jump_rate_mps,
                        args.bridge_max_gap_s,
                        args.bridge_max_anchor_speed_mps,
                    ),
                    epochs=gate.epochs,
                )
            )

    baseline_positioning_rate_pct = annotate_positioning_drop(candidates)
    rows = [candidate.row for candidate in candidates]
    ranked_candidates = rank_candidates(candidates)
    ranked = [candidate.row for candidate in ranked_candidates]

    policy_enabled = (
        args.max_positioning_drop_pct is not None or
        args.min_positioning_rate_pct is not None
    )
    policy_candidates = filter_policy_candidates(
        candidates,
        baseline_positioning_rate_pct,
        args.max_positioning_drop_pct,
        args.min_positioning_rate_pct,
    )
    if not policy_candidates:
        raise SystemExit("No jump-gate candidates satisfied the positioning-rate policy")
    policy_ranked_candidates = rank_candidates(policy_candidates)
    policy_best_candidate = policy_ranked_candidates[0]

    filtered_candidate: CandidateResult | None = None
    if args.filtered_pos_out is not None:
        if args.filtered_rate_mps is None:
            filtered_candidate = policy_best_candidate if policy_enabled else ranked_candidates[0]
        else:
            filtered_candidate = next(
                (
                    candidate
                    for candidate in candidates
                    if candidate_matches_threshold(
                        candidate,
                        args.filtered_rate_mps,
                        args.filtered_min_jump_m,
                    )
                ),
                None,
            )
            if filtered_candidate is None:
                raise SystemExit(
                    "No candidate matched "
                    f"--filtered-rate-mps {args.filtered_rate_mps:g} "
                    f"--filtered-min-jump-m {args.filtered_min_jump_m:g}"
                )
        write_libgnss_pos(args.filtered_pos_out, filtered_candidate.epochs)

    payload = {
        "reference_csv": str(args.reference_csv),
        "pos": str(args.pos),
        "match_tolerance_s": args.match_tolerance_s,
        "rates_mps": args.rates_mps,
        "min_jumps_m": args.min_jumps_m,
        "bridge_max_gap_s": args.bridge_max_gap_s,
        "bridge_max_anchor_speed_mps": args.bridge_max_anchor_speed_mps,
        "baseline_positioning_rate_pct": baseline_positioning_rate_pct,
        "candidate_count": len(rows),
        "results": rows,
        "best_by_p95_h_m": ranked[0],
        "policy": {
            "enabled": policy_enabled,
            "max_positioning_drop_pct": args.max_positioning_drop_pct,
            "min_positioning_rate_pct": args.min_positioning_rate_pct,
            "candidate_count": len(policy_candidates),
            "best_by_p95_h_m": policy_best_candidate.row,
        },
    }
    if filtered_candidate is not None and args.filtered_pos_out is not None:
        payload["filtered_pos"] = {
            "path": str(args.filtered_pos_out),
            "candidate": filtered_candidate.row,
        }

    args.summary_json.parent.mkdir(parents=True, exist_ok=True)
    args.summary_json.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    write_csv(args.csv, rows)

    print("PPC SPP jump-gate sweep")
    print(f"  candidates: {len(rows)}")
    print(f"  summary: {args.summary_json}")
    print(f"  csv: {args.csv}")
    if args.filtered_pos_out is not None:
        print(f"  filtered_pos: {args.filtered_pos_out}")
    if policy_enabled:
        print(
            "  policy: "
            f"max_drop={args.max_positioning_drop_pct} "
            f"min_positioning={args.min_positioning_rate_pct} "
            f"candidates={len(policy_candidates)}"
        )
        print(
            f"  policy_best: {candidate_label(policy_best_candidate)} "
            f"p95_h={float(policy_best_candidate.row['p95_h_m']):.3f} "
            f"positioning={float(policy_best_candidate.row['positioning_rate_pct']):.3f}% "
            f"drop={float(policy_best_candidate.row['positioning_drop_pct']):.3f}%"
        )
    print("  best rows by p95_h_m:")
    for row in ranked[: max(0, args.top)]:
        rate = row["max_position_jump_rate_mps"]
        min_jump = row["max_position_jump_min_m"]
        label = "baseline" if rate is None else f"rate={rate:g} min={min_jump:g}"
        print(
            f"    {label}: p95_h={float(row['p95_h_m']):.3f} "
            f"max_h={float(row['max_h_m']):.3f} "
            f"positioning={float(row['positioning_rate_pct']):.3f}% "
            f"drop={float(row['positioning_drop_pct']):.3f}% "
            f"rejected={row['jump_gate_rejected_epochs']} "
            f"bridged={row['bridge_inserted_epochs']}"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
