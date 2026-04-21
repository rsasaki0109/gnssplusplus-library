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
    seen = {status_name(match.status) for match in matches}
    ordered_names = [name for name in STATUS_ORDER if name in seen]
    ordered_names.extend(sorted(seen.difference(STATUS_ORDER)))
    for name in ordered_names:
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
    seen = {status_name(pair.lib_epoch.status) for pair in pairs}
    ordered_names = [name for name in STATUS_ORDER if name in seen]
    ordered_names.extend(sorted(seen.difference(STATUS_ORDER)))
    for name in ordered_names:
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


def bad_segments(
    matches: list[comparison.MatchedEpoch],
    threshold_m: float,
    max_gap_s: float,
) -> list[dict[str, object]]:
    bad = [match for match in matches if match.horiz_error_m > threshold_m]
    segments: list[list[comparison.MatchedEpoch]] = []
    current: list[comparison.MatchedEpoch] = []
    for match in bad:
        if not current or match.tow - current[-1].tow <= max_gap_s:
            current.append(match)
            continue
        segments.append(current)
        current = [match]
    if current:
        segments.append(current)

    rows: list[dict[str, object]] = []
    for segment in segments:
        horiz = [match.horiz_error_m for match in segment]
        statuses = sorted({status_name(match.status) for match in segment})
        rows.append(
            {
                "start_tow_s": rounded(segment[0].tow),
                "end_tow_s": rounded(segment[-1].tow),
                "duration_s": rounded(segment[-1].tow - segment[0].tow),
                "epochs": len(segment),
                "statuses": statuses,
                "median_h_m": percentile(horiz, 50),
                "p95_h_m": percentile(horiz, 95),
                "max_h_m": rounded(max(horiz)),
            }
        )
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
        "median_h_m",
        "p95_h_m",
        "max_h_m",
    ]
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            serialized = dict(row)
            serialized["statuses"] = ";".join(str(item) for item in row["statuses"])
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

    segment_lines = [
        f"{row['start_tow_s']:.1f}-{row['end_tow_s']:.1f}s "
        f"n={int(row['epochs'])} max={float(row['max_h_m']):.1f} "
        f"{','.join(row['statuses'])}"
        for row in segments
    ]
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
