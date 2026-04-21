#!/usr/bin/env python3
"""Analyze PPC coverage-profile quality by solution status."""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
from pathlib import Path
import sys
from typing import Any

import numpy as np


SCRIPTS_DIR = Path(__file__).resolve().parent
if str(SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_DIR))

import generate_driving_comparison as comparison  # noqa: E402


STATUS_NAMES = {
    4: "FIXED",
    3: "FLOAT",
    2: "DGPS",
    1: "SPP",
}
STATUS_ORDER = ("FIXED", "FLOAT", "DGPS", "SPP")
STATUS_COLORS = {
    "FIXED": "#2ecc71",
    "FLOAT": "#f39c12",
    "DGPS": "#3498db",
    "SPP": "#e74c3c",
}


def rounded(value: float) -> float:
    return round(float(value), 6)


def percentile(values: list[float], pct: float) -> float:
    if not values:
        return 0.0
    return rounded(float(np.percentile(np.array(values, dtype=float), pct)))


def status_name(status: int) -> str:
    return STATUS_NAMES.get(status, f"status_{status}")


def ordered_status_names(names: set[str]) -> list[str]:
    ordered = [name for name in STATUS_ORDER if name in names]
    ordered.extend(sorted(names.difference(STATUS_ORDER)))
    return ordered


def horizontal_track_distance(
    first: comparison.MatchedEpoch,
    second: comparison.MatchedEpoch,
) -> float:
    return math.hypot(
        second.traj_east_m - first.traj_east_m,
        second.traj_north_m - first.traj_north_m,
    )


def score_3d_50cm(matches: list[comparison.MatchedEpoch]) -> int:
    return sum(1 for match in matches if math.hypot(match.horiz_error_m, match.up_m) <= 0.50)


def summarize_matches(
    matches: list[comparison.MatchedEpoch],
    reference_count: int,
) -> dict[str, object]:
    horiz = [match.horiz_error_m for match in matches]
    up_abs = [abs(match.up_m) for match in matches]
    score_epochs = score_3d_50cm(matches)
    matched_count = max(len(matches), 1)
    return {
        "epochs": len(matches),
        "reference_rate_pct": rounded(100.0 * len(matches) / max(reference_count, 1)),
        "median_h_m": percentile(horiz, 50),
        "p90_h_m": percentile(horiz, 90),
        "p95_h_m": percentile(horiz, 95),
        "p99_h_m": percentile(horiz, 99),
        "max_h_m": rounded(max(horiz) if horiz else 0.0),
        "median_abs_up_m": percentile(up_abs, 50),
        "p95_abs_up_m": percentile(up_abs, 95),
        "ppc_score_3d_50cm_epochs": score_epochs,
        "ppc_score_3d_50cm_matched_pct": rounded(100.0 * score_epochs / matched_count),
        "ppc_score_3d_50cm_ref_pct": rounded(100.0 * score_epochs / max(reference_count, 1)),
    }


def summarize_by_status(
    matches: list[comparison.MatchedEpoch],
    reference_count: int,
) -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    for name in ordered_status_names({status_name(match.status) for match in matches}):
        status_matches = [match for match in matches if status_name(match.status) == name]
        row = {"status": name}
        row.update(summarize_matches(status_matches, reference_count))
        rows.append(row)
    return rows


def p95_contribution_by_status(matches: list[comparison.MatchedEpoch]) -> tuple[float, list[dict[str, object]]]:
    global_p95 = percentile([match.horiz_error_m for match in matches], 95)
    bad_matches = [match for match in matches if match.horiz_error_m >= global_p95]
    total_bad = max(len(bad_matches), 1)
    rows: list[dict[str, object]] = []
    for name in STATUS_ORDER:
        status_bad = [match for match in bad_matches if status_name(match.status) == name]
        if not status_bad:
            continue
        horiz = [match.horiz_error_m for match in status_bad]
        rows.append(
            {
                "status": name,
                "epochs": len(status_bad),
                "share_pct": rounded(100.0 * len(status_bad) / total_bad),
                "median_h_m": percentile(horiz, 50),
                "max_h_m": rounded(max(horiz)),
            }
        )
    return global_p95, rows


def paired_degradation_by_status(
    pairs: list[comparison.PairedEpoch],
) -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    for name in ordered_status_names({status_name(pair.lib_epoch.status) for pair in pairs}):
        status_pairs = [pair for pair in pairs if status_name(pair.lib_epoch.status) == name]
        deltas = [
            pair.lib_epoch.horiz_error_m - pair.rtklib_epoch.horiz_error_m
            for pair in status_pairs
        ]
        worse = [delta for delta in deltas if delta > 0.0]
        worse_5m = [delta for delta in deltas if delta > 5.0]
        rows.append(
            {
                "status": name,
                "pairs": len(status_pairs),
                "lib_worse_epochs": len(worse),
                "lib_worse_pct": rounded(100.0 * len(worse) / max(len(status_pairs), 1)),
                "lib_worse_gt_5m_epochs": len(worse_5m),
                "median_delta_h_m": percentile(deltas, 50),
                "p95_delta_h_m": percentile(deltas, 95),
            }
        )
    return rows


def fixed_anchor(
    matches: list[comparison.MatchedEpoch],
    start_index: int,
    step: int,
) -> comparison.MatchedEpoch | None:
    index = start_index
    while 0 <= index < len(matches):
        if status_name(matches[index].status) == "FIXED":
            return matches[index]
        index += step
    return None


def bridge_residuals(
    segment: list[comparison.MatchedEpoch],
    anchor_before: comparison.MatchedEpoch | None,
    anchor_after: comparison.MatchedEpoch | None,
) -> list[float]:
    if anchor_before is None or anchor_after is None:
        return []
    anchor_gap_s = anchor_after.tow - anchor_before.tow
    if not math.isfinite(anchor_gap_s) or anchor_gap_s <= 0.0:
        return []

    residuals = []
    for match in segment:
        fraction = (match.tow - anchor_before.tow) / anchor_gap_s
        predicted_east = anchor_before.traj_east_m + (
            anchor_after.traj_east_m - anchor_before.traj_east_m
        ) * fraction
        predicted_north = anchor_before.traj_north_m + (
            anchor_after.traj_north_m - anchor_before.traj_north_m
        ) * fraction
        residuals.append(
            math.hypot(
                match.traj_east_m - predicted_east,
                match.traj_north_m - predicted_north,
            )
        )
    return residuals


def segment_solution_path(segment: list[comparison.MatchedEpoch]) -> float:
    return sum(
        horizontal_track_distance(segment[index - 1], segment[index])
        for index in range(1, len(segment))
    )


def segment_status_counts(segment: list[comparison.MatchedEpoch]) -> dict[str, int]:
    counts: dict[str, int] = {}
    for match in segment:
        name = status_name(match.status)
        counts[name] = counts.get(name, 0) + 1
    return {name: counts[name] for name in ordered_status_names(set(counts))}


def dominant_status(status_counts: dict[str, int]) -> str:
    ordered_names = ordered_status_names(set(status_counts))
    return max(ordered_names, key=lambda name: status_counts[name]) if ordered_names else ""


def segment_anchor_features(
    matches: list[comparison.MatchedEpoch],
    segment_indexes: list[int],
) -> dict[str, object]:
    segment = [matches[index] for index in segment_indexes]
    duration_s = segment[-1].tow - segment[0].tow
    solution_path_m = segment_solution_path(segment)
    status_counts = segment_status_counts(segment)
    anchor_before = fixed_anchor(matches, segment_indexes[0] - 1, -1)
    anchor_after = fixed_anchor(matches, segment_indexes[-1] + 1, 1)
    residuals = bridge_residuals(segment, anchor_before, anchor_after)

    features: dict[str, object] = {
        "status_counts": status_counts,
        "dominant_status": dominant_status(status_counts),
        "solution_path_m": rounded(solution_path_m),
        "solution_chord_m": rounded(
            horizontal_track_distance(segment[0], segment[-1]) if len(segment) > 1 else 0.0
        ),
        "solution_path_speed_mps": rounded(solution_path_m / duration_s)
        if duration_s > 0.0
        else None,
        "previous_fixed_tow_s": rounded(anchor_before.tow) if anchor_before is not None else None,
        "next_fixed_tow_s": rounded(anchor_after.tow) if anchor_after is not None else None,
        "fixed_anchor_gap_s": None,
        "fixed_anchor_distance_m": None,
        "fixed_anchor_speed_mps": None,
        "fixed_anchor_bridge_residual_median_m": None,
        "fixed_anchor_bridge_residual_p95_m": None,
        "fixed_anchor_bridge_residual_max_m": None,
    }
    if anchor_before is not None and anchor_after is not None:
        anchor_gap_s = anchor_after.tow - anchor_before.tow
        anchor_distance_m = horizontal_track_distance(anchor_before, anchor_after)
        features.update(
            {
                "fixed_anchor_gap_s": rounded(anchor_gap_s),
                "fixed_anchor_distance_m": rounded(anchor_distance_m),
                "fixed_anchor_speed_mps": rounded(anchor_distance_m / anchor_gap_s)
                if anchor_gap_s > 0.0
                else None,
                "fixed_anchor_bridge_residual_median_m": percentile(residuals, 50),
                "fixed_anchor_bridge_residual_p95_m": percentile(residuals, 95),
                "fixed_anchor_bridge_residual_max_m": rounded(max(residuals)) if residuals else 0.0,
            }
        )
    return features


def bad_segments(
    matches: list[comparison.MatchedEpoch],
    threshold_m: float,
    max_gap_s: float,
) -> list[dict[str, object]]:
    bad_indexes = [index for index, match in enumerate(matches) if match.horiz_error_m > threshold_m]
    segments: list[list[int]] = []
    current: list[int] = []
    for index in bad_indexes:
        if not current or matches[index].tow - matches[current[-1]].tow <= max_gap_s:
            current.append(index)
            continue
        segments.append(current)
        current = [index]
    if current:
        segments.append(current)

    rows: list[dict[str, object]] = []
    for segment_indexes in segments:
        segment = [matches[index] for index in segment_indexes]
        horiz = [match.horiz_error_m for match in segment]
        statuses = sorted({status_name(match.status) for match in segment})
        row: dict[str, object] = {
            "start_tow_s": rounded(segment[0].tow),
            "end_tow_s": rounded(segment[-1].tow),
            "duration_s": rounded(segment[-1].tow - segment[0].tow),
            "epochs": len(segment),
            "statuses": statuses,
            "median_h_m": percentile(horiz, 50),
            "p95_h_m": percentile(horiz, 95),
            "max_h_m": rounded(max(horiz)),
        }
        row.update(segment_anchor_features(matches, segment_indexes))
        rows.append(row)
    rows.sort(key=lambda row: (int(row["epochs"]), float(row["max_h_m"])), reverse=True)
    return rows


def build_report(
    *,
    lib_pos: Path,
    rtklib_pos: Path,
    reference_csv: Path,
    match_tolerance_s: float,
    bad_h_threshold_m: float,
    bad_gap_s: float,
) -> dict[str, object]:
    reference = comparison.read_reference_csv(reference_csv)
    lib_matches = comparison.match_to_reference(
        comparison.read_libgnss_pos(lib_pos),
        reference,
        match_tolerance_s,
    )
    rtklib_matches = comparison.match_to_reference(
        comparison.read_rtklib_pos(rtklib_pos),
        reference,
        match_tolerance_s,
    )
    if not lib_matches:
        raise SystemExit("No gnssplusplus epochs matched the PPC reference.")
    if not rtklib_matches:
        raise SystemExit("No RTKLIB epochs matched the PPC reference.")

    pairs = comparison.pair_epochs(lib_matches, rtklib_matches, match_tolerance_s)
    global_p95_h_m, contribution_rows = p95_contribution_by_status(lib_matches)
    return {
        "reference_epochs": len(reference),
        "lib_matched_epochs": len(lib_matches),
        "rtklib_matched_epochs": len(rtklib_matches),
        "common_epoch_pairs": len(pairs),
        "match_tolerance_s": match_tolerance_s,
        "bad_h_threshold_m": bad_h_threshold_m,
        "global_p95_h_m": global_p95_h_m,
        "lib_by_status": summarize_by_status(lib_matches, len(reference)),
        "p95_contribution_by_status": contribution_rows,
        "paired_delta_by_status": paired_degradation_by_status(pairs),
        "bad_segments": bad_segments(lib_matches, bad_h_threshold_m, bad_gap_s),
    }


def write_segments_csv(path: Path, rows: list[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = [
        "start_tow_s",
        "end_tow_s",
        "duration_s",
        "epochs",
        "statuses",
        "status_counts",
        "dominant_status",
        "median_h_m",
        "p95_h_m",
        "max_h_m",
        "solution_path_m",
        "solution_chord_m",
        "solution_path_speed_mps",
        "previous_fixed_tow_s",
        "next_fixed_tow_s",
        "fixed_anchor_gap_s",
        "fixed_anchor_distance_m",
        "fixed_anchor_speed_mps",
        "fixed_anchor_bridge_residual_median_m",
        "fixed_anchor_bridge_residual_p95_m",
        "fixed_anchor_bridge_residual_max_m",
    ]
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            serialized = dict(row)
            serialized["statuses"] = ";".join(str(item) for item in row["statuses"])
            serialized["status_counts"] = ";".join(
                f"{name}:{count}" for name, count in dict(row["status_counts"]).items()
            )
            for key, value in list(serialized.items()):
                if value is None:
                    serialized[key] = ""
            writer.writerow(serialized)


def format_status_table(rows: list[dict[str, object]]) -> str:
    lines = ["status   epochs   p50H   p95H   3D50/ref"]
    for row in rows:
        lines.append(
            f"{row['status']:<7} {int(row['epochs']):>6} "
            f"{float(row['median_h_m']):>6.2f} "
            f"{float(row['p95_h_m']):>6.2f} "
            f"{float(row['ppc_score_3d_50cm_ref_pct']):>8.1f}%"
        )
    return "\n".join(lines)


def format_segment_line(row: dict[str, object]) -> str:
    prefix = (
        f"{row['start_tow_s']:.1f}-{row['end_tow_s']:.1f}s "
        f"n={int(row['epochs'])} max={float(row['max_h_m']):.1f} "
        f"{','.join(row['statuses'])}"
    )
    if row["fixed_anchor_speed_mps"] is None:
        return f"{prefix} no-anchor"
    return (
        f"{prefix} a={float(row['fixed_anchor_speed_mps']):.1f}m/s "
        f"b95={float(row['fixed_anchor_bridge_residual_p95_m']):.1f}m"
    )


def render_png(report: dict[str, object], output: Path, title: str) -> None:
    import matplotlib.pyplot as plt

    by_status = list(report["lib_by_status"])
    contribution = list(report["p95_contribution_by_status"])
    segments = list(report["bad_segments"])[:6]

    fig = plt.figure(figsize=(13.5, 7.2), dpi=100, facecolor="#f4efe6")
    grid = fig.add_gridspec(2, 2, height_ratios=[0.30, 1.0], hspace=0.28, wspace=0.22)

    ax_head = fig.add_subplot(grid[0, :])
    ax_head.set_axis_off()
    ax_head.text(0.0, 0.75, title, fontsize=25, weight="bold", color="#14213d")
    ax_head.text(
        0.0,
        0.34,
        (
            f"matched={report['lib_matched_epochs']}/{report['reference_epochs']}  "
            f"global p95H={float(report['global_p95_h_m']):.2f} m  "
            f"bad threshold={float(report['bad_h_threshold_m']):.1f} m"
        ),
        fontsize=12.5,
        color="#5f6c7b",
    )

    ax_status = fig.add_subplot(grid[1, 0])
    statuses = [str(row["status"]) for row in by_status]
    p95 = [float(row["p95_h_m"]) for row in by_status]
    ax_status.bar(statuses, p95, color=[STATUS_COLORS.get(name, "#64748b") for name in statuses])
    ax_status.set_title("Horizontal p95 by status", fontsize=14, weight="bold", color="#14213d")
    ax_status.set_ylabel("p95 H error (m)", color="#5f6c7b")
    ax_status.tick_params(colors="#5f6c7b")
    ax_status.grid(axis="y", alpha=0.20)
    for index, value in enumerate(p95):
        ax_status.text(index, value + max(p95) * 0.025, f"{value:.1f}", ha="center", fontsize=10)

    ax_table = fig.add_subplot(grid[1, 1])
    ax_table.set_axis_off()
    ax_table.text(
        0.0,
        1.0,
        "Status summary",
        fontsize=14,
        weight="bold",
        color="#14213d",
        va="top",
    )
    ax_table.text(
        0.0,
        0.88,
        format_status_table(by_status),
        family="monospace",
        fontsize=10.5,
        color="#14213d",
        va="top",
    )

    contrib_text = "\n".join(
        f"{row['status']:<7} {int(row['epochs']):>4} epochs  {float(row['share_pct']):>5.1f}%"
        for row in contribution
    )
    ax_table.text(
        0.0,
        0.36,
        "Global p95H exceedance contribution",
        fontsize=12.5,
        weight="bold",
        color="#14213d",
        va="top",
    )
    ax_table.text(
        0.0,
        0.27,
        contrib_text,
        family="monospace",
        fontsize=10.5,
        color="#14213d",
        va="top",
    )

    segment_lines = [format_segment_line(row) for row in segments]
    ax_table.text(
        0.0,
        -0.03,
        "Largest >threshold segments",
        fontsize=12.5,
        weight="bold",
        color="#14213d",
        va="top",
    )
    ax_table.text(
        0.0,
        -0.12,
        "\n".join(segment_lines),
        family="monospace",
        fontsize=9.5,
        color="#14213d",
        va="top",
    )

    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=100, facecolor=fig.get_facecolor(), bbox_inches="tight")
    print(f"Saved: {output}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(prog=os.environ.get("GNSS_CLI_NAME"))
    parser.add_argument("--lib-pos", type=Path, required=True)
    parser.add_argument("--rtklib-pos", type=Path, required=True)
    parser.add_argument("--reference-csv", type=Path, required=True)
    parser.add_argument("--summary-json", type=Path, default=None)
    parser.add_argument("--segments-csv", type=Path, default=None)
    parser.add_argument("--output-png", type=Path, default=None)
    parser.add_argument("--title", default="PPC coverage quality by status")
    parser.add_argument("--match-tolerance-s", type=float, default=0.25)
    parser.add_argument("--bad-h-threshold-m", type=float, default=30.0)
    parser.add_argument("--bad-gap-s", type=float, default=0.6)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    report = build_report(
        lib_pos=args.lib_pos,
        rtklib_pos=args.rtklib_pos,
        reference_csv=args.reference_csv,
        match_tolerance_s=args.match_tolerance_s,
        bad_h_threshold_m=args.bad_h_threshold_m,
        bad_gap_s=args.bad_gap_s,
    )
    if args.summary_json is not None:
        args.summary_json.parent.mkdir(parents=True, exist_ok=True)
        args.summary_json.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        print(f"Saved: {args.summary_json}")
    if args.segments_csv is not None:
        write_segments_csv(args.segments_csv, list(report["bad_segments"]))
        print(f"Saved: {args.segments_csv}")
    if args.output_png is not None:
        render_png(report, args.output_png, args.title)
    if args.summary_json is None and args.segments_csv is None and args.output_png is None:
        print(json.dumps(report, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
