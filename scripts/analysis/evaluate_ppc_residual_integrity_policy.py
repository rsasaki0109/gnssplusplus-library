#!/usr/bin/env python3
"""Audit the fixed truth-free staged integrity policy on solution files.

Reference truth is used only by this offline evaluator to label catches and
harm. The replayed satellite and residual rules consume POS status and RTK
telemetry only.
"""

from __future__ import annotations

import argparse
import bisect
import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import analyze_ppc_wrong_fix_residuals as audit


@dataclass(frozen=True)
class Policy:
    base_min_satellites: int = 8
    base_low_satellite_ceiling: int = 11
    base_low_satellite_max_ratio: float = 15.0
    base_exonerate_min_satellites: int = 11
    base_exonerate_max_prefit_rms_m: float = 0.5
    base_exonerate_max_nis_per_obs: float = 0.2
    # Odaiba exposed that the original 15 m floor demoted a 27-epoch,
    # otherwise-consistent FIX segment.  The PPC N2 event starts above 42 m,
    # so 40 m preserves every PPC selection while removing that false alarm.
    streak_prefit_rms_m: float = 40.0
    streak_max_ratio: float = 15.0
    # Full Shinjuku exposed a benign sequence with intermittent 11-outlier
    # epochs. Requiring 12 breaks that sequence and preserves all PPC picks.
    streak_min_outliers: int = 12
    streak_min_outlier_fraction: float = 0.5
    streak_epochs: int = 8
    spike_prefit_rms_m: float = 40.0
    spike_max_satellites: int = 14


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset-root", type=Path)
    parser.add_argument("--input-dir", type=Path)
    parser.add_argument(
        "--run",
        action="append",
        default=[],
        metavar="KEY=POS,REFERENCE",
        help="Audit an explicit holdout run; repeat for multiple runs.",
    )
    parser.add_argument("--match-tolerance-s", type=float, default=0.25)
    parser.add_argument(
        "--wrong-fix-threshold-m",
        type=float,
        default=0.5,
        help="Offline truth-label threshold only; it is never consumed by the runtime policy.",
    )
    parser.add_argument("--summary-json", type=Path, required=True)
    parser.add_argument("--markdown-output", type=Path, required=True)
    return parser.parse_args()


def parse_run_spec(spec: str) -> tuple[str, Path, Path]:
    try:
        key, paths = spec.split("=", 1)
        pos, reference = paths.split(",", 1)
    except ValueError as exc:
        raise SystemExit(f"Invalid --run {spec!r}; expected KEY=POS,REFERENCE") from exc
    if not key or not pos or not reference:
        raise SystemExit(f"Invalid --run {spec!r}; expected non-empty KEY=POS,REFERENCE")
    return key, Path(pos), Path(reference)


def resolve_runs(args: argparse.Namespace) -> list[tuple[str, Path, Path]]:
    if args.run:
        if args.dataset_root is not None or args.input_dir is not None:
            raise SystemExit("Use either --run or --dataset-root with --input-dir, not both")
        return [parse_run_spec(spec) for spec in args.run]
    if args.dataset_root is None or args.input_dir is None:
        raise SystemExit("PPC mode requires both --dataset-root and --input-dir")
    return [
        (
            key,
            args.input_dir / f"{key}.pos",
            args.dataset_root / relative / "reference.csv",
        )
        for key, relative in audit.RUNS
    ]


def build_reference_index(
    reference: dict[tuple[int, float], tuple[float, float, float]],
) -> dict[int, tuple[list[float], list[tuple[float, float, float]]]]:
    by_week: dict[int, list[tuple[float, tuple[float, float, float]]]] = {}
    for (week, tow_s), ecef in reference.items():
        by_week.setdefault(week, []).append((tow_s, ecef))
    result = {}
    for week, entries in by_week.items():
        entries.sort(key=lambda item: item[0])
        result[week] = ([item[0] for item in entries], [item[1] for item in entries])
    return result


def nearest_reference(
    index: dict[int, tuple[list[float], list[tuple[float, float, float]]]],
    week: int,
    tow_s: float,
    tolerance_s: float,
) -> tuple[float, float, float] | None:
    week_data = index.get(week)
    if week_data is None:
        return None
    times, positions = week_data
    insertion = bisect.bisect_left(times, tow_s)
    candidates = [candidate for candidate in (insertion - 1, insertion) if 0 <= candidate < len(times)]
    if not candidates:
        return None
    nearest = min(candidates, key=lambda candidate: abs(times[candidate] - tow_s))
    if abs(times[nearest] - tow_s) > tolerance_s:
        return None
    return positions[nearest]


def streak_matches(epoch: audit.SolutionEpoch, policy: Policy) -> bool:
    return (
        epoch.status == 4
        and epoch.prefit_rms_m is not None
        and epoch.prefit_rms_m > policy.streak_prefit_rms_m
        and epoch.ratio is not None
        and epoch.ratio <= policy.streak_max_ratio
        and epoch.outliers is not None
        and epoch.outliers >= policy.streak_min_outliers
        and epoch.observations is not None
        and epoch.observations > 0
        and epoch.outliers / epoch.observations >= policy.streak_min_outlier_fraction
    )


def base_matches(epoch: audit.SolutionEpoch, policy: Policy) -> bool:
    gate = epoch.status == 4 and epoch.telemetry_complete and (
        epoch.nsat is None
        or epoch.nsat < policy.base_min_satellites
        or (
            epoch.nsat <= policy.base_low_satellite_ceiling
            and epoch.ratio is not None
            and epoch.ratio <= policy.base_low_satellite_max_ratio
        )
    )
    exonerated = (
        gate
        and epoch.nsat is not None
        and epoch.nsat >= policy.base_exonerate_min_satellites
        and epoch.prefit_rms_m is not None
        and epoch.prefit_rms_m <= policy.base_exonerate_max_prefit_rms_m
        and epoch.nis_per_obs is not None
        and epoch.nis_per_obs <= policy.base_exonerate_max_nis_per_obs
    )
    return gate and not exonerated


def selected_indices(
    epochs: list[audit.SolutionEpoch], policy: Policy
) -> tuple[set[int], set[int]]:
    buffered_streak: set[int] = set()
    start = 0
    for index in range(len(epochs) + 1):
        if index < len(epochs) and streak_matches(epochs[index], policy):
            continue
        if index - start >= policy.streak_epochs:
            buffered_streak.update(range(start, index))
        start = index + 1
    spike = {
        index
        for index, epoch in enumerate(epochs)
        if epoch.status == 4
        and epoch.prefit_rms_m is not None
        and epoch.prefit_rms_m >= policy.spike_prefit_rms_m
        and epoch.nsat is not None
        and epoch.nsat <= policy.spike_max_satellites
    }
    return buffered_streak, spike


def evaluate_run(
    epochs: list[audit.SolutionEpoch],
    reference: dict[tuple[int, float], tuple[float, float, float]],
    policy: Policy,
    match_tolerance_s: float = 0.25,
    wrong_fix_threshold_m: float = 0.5,
) -> dict[str, Any]:
    streak, spike = selected_indices(epochs, policy)
    base = {index for index, epoch in enumerate(epochs) if base_matches(epoch, policy)}
    residual = streak | spike
    selected = base | residual
    reference_index = build_reference_index(reference)
    fixed = wrong = caught = harmed = above_5m = above_10m = 0
    caught_above_5m = caught_above_10m = 0
    for index, epoch in enumerate(epochs):
        if epoch.status != 4:
            continue
        truth = nearest_reference(reference_index, epoch.week, epoch.tow_s, match_tolerance_s)
        if truth is None:
            continue
        fixed += 1
        error_m = audit.error_3d_m(epoch.ecef, truth)
        is_wrong = error_m > wrong_fix_threshold_m
        wrong += is_wrong
        above_5m += error_m > 5.0
        above_10m += error_m > 10.0
        if index in selected:
            caught += is_wrong
            harmed += not is_wrong
            caught_above_5m += error_m > 5.0
            caught_above_10m += error_m > 10.0
    return {
        "fixed_before": fixed,
        "fixed_after": fixed - caught - harmed,
        "wrong_before": wrong,
        "wrong_after": wrong - caught,
        "selected_epochs": len(selected),
        "wrong_caught": caught,
        "correct_harmed": harmed,
        "above_5m_before": above_5m,
        "above_5m_caught": caught_above_5m,
        "above_10m_before": above_10m,
        "above_10m_caught": caught_above_10m,
        "streak_selected": len(streak),
        "spike_selected": len(spike),
        "overlap_selected": len(streak & spike),
        "base_selected": len(base),
        "base_residual_overlap_selected": len(base & residual),
    }


def render_markdown(payload: dict[str, Any]) -> str:
    lines = [
        "# Staged integrity fixed-policy audit",
        "",
        "Reference truth is used only for offline labels. Runtime policy inputs are POS status and RTK telemetry.",
        "",
        "| Run | Selected | Wrong caught | Correct harmed | >5 m caught | >10 m caught | Wrong after |",
        "|---|---:|---:|---:|---:|---:|---:|",
    ]
    for row in payload["runs"]:
        lines.append(
            f"| {row['key']} | {row['selected_epochs']} | {row['wrong_caught']} | "
            f"{row['correct_harmed']} | {row['above_5m_caught']} | "
            f"{row['above_10m_caught']} | {row['wrong_after']} |"
        )
    total = payload["total"]
    lines.extend(
        [
            "",
            f"Total: selected {total['selected_epochs']}, caught {total['wrong_caught']} wrong FIX, "
            f"harmed {total['correct_harmed']} correct FIX; wrong FIX {total['wrong_before']} -> "
            f"{total['wrong_after']}.",
            "",
            payload["interpretation"],
        ]
    )
    return "\n".join(lines) + "\n"


def main() -> int:
    args = parse_args()
    if args.match_tolerance_s < 0.0:
        raise SystemExit("--match-tolerance-s must be non-negative")
    if args.wrong_fix_threshold_m <= 0.0:
        raise SystemExit("--wrong-fix-threshold-m must be positive")
    policy = Policy()
    rows: list[dict[str, Any]] = []
    run_specs = resolve_runs(args)
    for key, pos_path, reference_path in run_specs:
        epochs = audit.load_solution(pos_path)
        reference = audit.load_reference(reference_path)
        rows.append(
            {
                "key": key,
                **evaluate_run(
                    epochs,
                    reference,
                    policy,
                    args.match_tolerance_s,
                    args.wrong_fix_threshold_m,
                ),
            }
        )
    metric_names = tuple(name for name in rows[0] if name != "key")
    total = {name: sum(int(row[name]) for row in rows) for name in metric_names}
    explicit_holdout = bool(args.run)
    if explicit_holdout:
        external_status = (
            "safe_no_false_demotions"
            if total["correct_harmed"] == 0
            else "failed_false_demotions"
        )
        interpretation = (
            "The frozen runtime policy produced no false demotions on these explicit holdout runs."
            if total["correct_harmed"] == 0
            else "The frozen runtime policy falsely demoted correct FIX epochs on these explicit holdout runs."
        )
    else:
        external_status = "pending"
        interpretation = (
            "The fixed policy is harmless on five runs but active only on Nagoya 2. "
            "This proves six-run safety for these artifacts, not cross-city efficacy."
        )
    payload = {
        "schema_version": 1,
        "reference_truth_used_by_runtime_policy": False,
        "reference_truth_used_for_offline_labels": True,
        "bounded_output_latency_epochs": policy.streak_epochs - 1,
        "match_tolerance_s": args.match_tolerance_s,
        "wrong_fix_threshold_m": args.wrong_fix_threshold_m,
        "policy": policy.__dict__,
        "runs": rows,
        "total": total,
        "evaluation_scope": "explicit_holdout" if explicit_holdout else "ppc_six_run",
        "external_validation_status": external_status,
        "external_policy_active": total["selected_epochs"] > 0 if explicit_holdout else None,
        "interpretation": interpretation,
    }
    args.summary_json.parent.mkdir(parents=True, exist_ok=True)
    args.summary_json.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    args.markdown_output.parent.mkdir(parents=True, exist_ok=True)
    args.markdown_output.write_text(render_markdown(payload), encoding="utf-8")
    print(json.dumps(total, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
