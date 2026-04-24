#!/usr/bin/env python3
"""Generate a PPC RTK 2D trajectory figure colored by solution status."""

from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import sys


ROOT_DIR = Path(__file__).resolve().parents[1]
APPS_DIR = ROOT_DIR / "apps"
if str(APPS_DIR) not in sys.path:
    sys.path.insert(0, str(APPS_DIR))

from generate_driving_comparison import (
    build_status_legend_handles,
    match_to_reference,
    matched_xy,
    padded_xy_limits,
    plot_solver_trajectory,
    read_libgnss_pos,
    read_reference_csv,
    read_rtklib_pos,
    status_style,
    trajectory_enu,
)
import gnss_ppc_metrics as ppc_metrics  # noqa: E402


BG = "#f4efe6"
PANEL = "#fffaf2"
TEXT = "#14213d"
MUTED = "#5f6c7b"
EDGE = "#d8c9b1"
OFFICIAL_SCORE_COLORS = {
    "scored": "#168a46",
    "high_error": "#d1495b",
    "no_solution": "#8d99ae",
}
OFFICIAL_SCORE_LABELS = {
    "scored": "official scored",
    "high_error": ">50 cm loss",
    "no_solution": "no-solution loss",
}
BAD_SEGMENT_COLOR = "#d1495b"


def ppc_3d_score(matched, reference_count: int, threshold_m: float) -> tuple[int, float, float]:
    if not matched:
        return 0, 0.0, 0.0
    scored = sum(
        1
        for epoch in matched
        if math.hypot(epoch.horiz_error_m, epoch.up_m) <= threshold_m
    )
    matched_pct = 100.0 * scored / len(matched)
    reference_pct = 100.0 * scored / max(reference_count, 1)
    return scored, matched_pct, reference_pct


def status_counts(matched, solver: str) -> dict[str, int]:
    counts: dict[str, int] = {}
    for epoch in matched:
        status_name, _ = status_style(solver, epoch.status)
        counts[status_name] = counts.get(status_name, 0) + 1
    return counts


def official_state_distances(records: list[dict[str, object]]) -> dict[str, float]:
    distances: dict[str, float] = {}
    for record in records:
        state = str(record["score_state"])
        distances[state] = distances.get(state, 0.0) + float(record["segment_distance_m"])
    return distances


def official_score_legend_handles():
    from matplotlib.lines import Line2D

    return [
        Line2D([0], [0], color=OFFICIAL_SCORE_COLORS[state], lw=4, label=OFFICIAL_SCORE_LABELS[state])
        for state in ("scored", "high_error", "no_solution")
    ]


def bad_segment_legend_handle():
    from matplotlib.lines import Line2D

    return Line2D(
        [0],
        [0],
        color=BAD_SEGMENT_COLOR,
        lw=2.5,
        marker="o",
        markerfacecolor="none",
        markeredgecolor=BAD_SEGMENT_COLOR,
        markeredgewidth=1.2,
        label="bad P95 segment",
    )


def load_bad_segments(path: Path | None, top_n: int) -> list[dict[str, object]]:
    if path is None:
        return []
    payload = json.loads(path.read_text(encoding="utf-8"))
    segments = list(payload.get("bad_segments", []))
    segments.sort(
        key=lambda row: (
            int(row.get("epochs", 0) or 0),
            float(row.get("max_h_m", 0.0) or 0.0),
        ),
        reverse=True,
    )
    return segments[:top_n] if top_n > 0 else segments


def plot_bad_segment_overlays(ax, matched, bad_segments: list[dict[str, object]]) -> None:
    for index, segment in enumerate(bad_segments, start=1):
        start_tow = float(segment["start_tow_s"])
        end_tow = float(segment["end_tow_s"])
        points = [
            (epoch.traj_east_m, epoch.traj_north_m)
            for epoch in matched
            if start_tow <= epoch.tow <= end_tow
        ]
        if not points:
            continue
        xs = [point[0] for point in points]
        ys = [point[1] for point in points]
        if len(points) > 1:
            ax.plot(
                xs,
                ys,
                color=BAD_SEGMENT_COLOR,
                linewidth=2.0,
                alpha=0.78,
                zorder=10,
            )
        ax.scatter(
            xs,
            ys,
            s=42,
            marker="o",
            facecolor="none",
            edgecolor=BAD_SEGMENT_COLOR,
            linewidth=1.15,
            alpha=0.90,
            zorder=11,
        )
        label_index = len(points) // 2
        max_h_m = float(segment.get("max_h_m", 0.0) or 0.0)
        ax.text(
            xs[label_index],
            ys[label_index],
            f"{index}: {max_h_m:.0f}m",
            fontsize=7.8,
            color=BAD_SEGMENT_COLOR,
            weight="bold",
            bbox=dict(boxstyle="round,pad=0.20", facecolor="white", edgecolor=BAD_SEGMENT_COLOR, alpha=0.88),
            zorder=12,
        )


def plot_official_score_segments(ax, reference_enu, official_records: list[dict[str, object]]) -> None:
    from matplotlib.collections import LineCollection

    segments = []
    colors = []
    for record in official_records:
        ref_index = int(record["reference_index"])
        if ref_index <= 0 or ref_index >= len(reference_enu):
            continue
        segments.append(
            [
                (reference_enu[ref_index - 1, 0], reference_enu[ref_index - 1, 1]),
                (reference_enu[ref_index, 0], reference_enu[ref_index, 1]),
            ]
        )
        colors.append(OFFICIAL_SCORE_COLORS.get(str(record["score_state"]), "#64748b"))
    ax.add_collection(LineCollection(segments, colors=colors, linewidths=2.8, alpha=0.92, zorder=4))


def draw_solver_panel(
    ax,
    *,
    title: str,
    solver: str,
    matched,
    official_records: list[dict[str, object]],
    reference_count: int,
    reference_enu,
    limits: tuple[float, float, float, float],
    official_score_pct: float,
    score_threshold_m: float,
    max_gap_s: float,
    color_mode: str,
    bad_segments: list[dict[str, object]] | None = None,
) -> None:
    ax.set_facecolor(PANEL)
    ax.plot(reference_enu[:, 0], reference_enu[:, 1], color="black", linewidth=1.4, alpha=0.46, zorder=1)
    if color_mode == "official":
        plot_official_score_segments(ax, reference_enu, official_records)
    else:
        plot_solver_trajectory(ax, matched, solver=solver, max_gap_s=max_gap_s, point_size=9.5)
    if bad_segments:
        plot_bad_segment_overlays(ax, matched, bad_segments)
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlim(limits[0], limits[1])
    ax.set_ylim(limits[2], limits[3])
    ax.set_title(title, fontsize=14, color=TEXT, weight="bold", pad=10)
    ax.set_xlabel("East (m)", fontsize=9.5, color=MUTED)
    ax.set_ylabel("North (m)", fontsize=9.5, color=MUTED)
    ax.tick_params(labelsize=8, colors=MUTED)
    ax.grid(alpha=0.18)
    for spine in ax.spines.values():
        spine.set_color(EDGE)

    scored, matched_score_pct, _reference_score_pct = ppc_3d_score(
        matched,
        reference_count,
        score_threshold_m,
    )
    positioning_rate_pct = 100.0 * len(matched) / max(reference_count, 1)
    counts = status_counts(matched, solver)
    if color_mode == "official":
        total_distance_m = sum(float(record["segment_distance_m"]) for record in official_records)
        state_distances = official_state_distances(official_records)

        def dist_pct(state: str) -> float:
            return 100.0 * state_distances.get(state, 0.0) / total_distance_m if total_distance_m > 0.0 else 0.0

        detail = "\n".join(
            [
                f"pos {len(matched)}/{reference_count}",
                f"pos rate {positioning_rate_pct:.1f}%",
                f"official {official_score_pct:.1f}%",
                f"scored {dist_pct('scored'):.1f}%",
                f">50cm {dist_pct('high_error'):.1f}%",
                f"no-sol {dist_pct('no_solution'):.1f}%",
            ]
        )
        corner_label = f"{state_distances.get('scored', 0.0):.0f}/{total_distance_m:.0f} m scored"
    else:
        detail = "\n".join(
            [
                f"pos {len(matched)}/{reference_count}",
                f"pos rate {positioning_rate_pct:.1f}%",
                f"official {official_score_pct:.1f}%",
                f"3D50/pos {matched_score_pct:.1f}%",
                f"FIX {counts.get('FIXED', 0)}",
                f"FLOAT {counts.get('FLOAT', 0)}",
                f"DGPS  {counts.get('DGPS', 0)}",
                f"SPP   {counts.get('SPP', 0)}",
            ]
        )
        corner_label = f"{scored}/{reference_count} ref epochs"
    ax.text(
        0.025,
        0.975,
        detail,
        transform=ax.transAxes,
        va="top",
        ha="left",
        family="monospace",
        fontsize=8.7,
        color=TEXT,
        bbox=dict(boxstyle="round,pad=0.35", facecolor="white", edgecolor=EDGE, alpha=0.90),
        zorder=12,
    )
    ax.text(
        0.975,
        0.035,
        corner_label,
        transform=ax.transAxes,
        va="bottom",
        ha="right",
        fontsize=8.5,
        color=MUTED,
        bbox=dict(boxstyle="round,pad=0.25", facecolor="white", edgecolor=EDGE, alpha=0.80),
        zorder=12,
    )


def main() -> int:
    parser = argparse.ArgumentParser(prog=os.environ.get("GNSS_CLI_NAME"))
    parser.add_argument("--lib-pos", type=Path, required=True)
    parser.add_argument("--rtklib-pos", type=Path, required=True)
    parser.add_argument("--reference-csv", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--title", default="PPC RTK trajectory")
    parser.add_argument(
        "--subtitle",
        default=None,
    )
    parser.add_argument("--color-mode", choices=("status", "official"), default="status")
    parser.add_argument("--match-tolerance-s", type=float, default=0.25)
    parser.add_argument("--score-threshold-m", type=float, default=0.50)
    parser.add_argument("--max-gap-s", type=float, default=2.0)
    parser.add_argument(
        "--bad-segments-json",
        type=Path,
        default=None,
        help="Optional coverage-quality JSON whose bad_segments should be highlighted on the gnssplusplus panel.",
    )
    parser.add_argument(
        "--bad-segments-top-n",
        type=int,
        default=8,
        help="Number of largest bad segments to overlay when --bad-segments-json is set (default: 8; <=0 for all).",
    )
    args = parser.parse_args()
    if args.subtitle is None:
        args.subtitle = (
            "2D reference track colored by PPC official score outcome; no-solution segments count as lost distance."
            if args.color_mode == "official"
            else "2D track colored by solution status; positioning rate is matched/reference epochs."
        )

    import matplotlib.pyplot as plt

    reference = read_reference_csv(args.reference_csv)
    lib_epochs = read_libgnss_pos(args.lib_pos)
    rtklib_epochs = read_rtklib_pos(args.rtklib_pos)
    lib_matched = match_to_reference(
        lib_epochs,
        reference,
        args.match_tolerance_s,
    )
    rtklib_matched = match_to_reference(
        rtklib_epochs,
        reference,
        args.match_tolerance_s,
    )
    if not lib_matched:
        raise SystemExit("No gnssplusplus epochs matched the PPC reference.")
    if not rtklib_matched:
        raise SystemExit("No RTKLIB epochs matched the PPC reference.")

    reference_enu = trajectory_enu(reference, reference[0])
    lib_official = ppc_metrics.ppc_official_distance_score(
        reference,
        lib_epochs,
        args.match_tolerance_s,
        args.score_threshold_m,
    )
    rtklib_official = ppc_metrics.ppc_official_distance_score(
        reference,
        rtklib_epochs,
        args.match_tolerance_s,
        args.score_threshold_m,
    )
    lib_official_records = ppc_metrics.ppc_official_segment_records(
        reference,
        lib_epochs,
        args.match_tolerance_s,
        args.score_threshold_m,
    )
    rtklib_official_records = ppc_metrics.ppc_official_segment_records(
        reference,
        rtklib_epochs,
        args.match_tolerance_s,
        args.score_threshold_m,
    )
    bad_segments = load_bad_segments(args.bad_segments_json, args.bad_segments_top_n)
    limits = padded_xy_limits(
        reference_enu[:, :2],
        matched_xy(lib_matched),
        matched_xy(rtklib_matched),
        min_pad=20.0,
    )

    fig = plt.figure(figsize=(14, 7.2), dpi=100, facecolor=BG)
    grid = fig.add_gridspec(2, 2, height_ratios=[0.22, 1.0], hspace=0.12, wspace=0.08)
    ax_head = fig.add_subplot(grid[0, :])
    ax_head.set_axis_off()
    ax_head.set_xlim(0, 1)
    ax_head.set_ylim(0, 1)
    ax_head.text(0.0, 0.78, args.title, fontsize=28, color=TEXT, weight="bold")
    ax_head.text(0.0, 0.28, args.subtitle, fontsize=12.8, color=MUTED)
    legend_handles = (
        official_score_legend_handles() if args.color_mode == "official" else build_status_legend_handles()
    )
    if bad_segments:
        legend_handles = [*legend_handles, bad_segment_legend_handle()]
    ax_head.legend(
        handles=legend_handles,
        loc="lower right",
        bbox_to_anchor=(1.0, 0.00),
        ncol=5 if bad_segments else 4,
        frameon=False,
        fontsize=9.5,
    )

    ax_rtklib = fig.add_subplot(grid[1, 0])
    draw_solver_panel(
        ax_rtklib,
        title="RTKLIB demo5",
        solver="RTKLIB",
        matched=rtklib_matched,
        official_records=rtklib_official_records,
        reference_count=len(reference),
        reference_enu=reference_enu,
        limits=limits,
        official_score_pct=float(rtklib_official["ppc_official_score_pct"]),
        score_threshold_m=args.score_threshold_m,
        max_gap_s=args.max_gap_s,
        color_mode=args.color_mode,
    )
    ax_lib = fig.add_subplot(grid[1, 1])
    draw_solver_panel(
        ax_lib,
        title="gnssplusplus",
        solver="libgnss++",
        matched=lib_matched,
        official_records=lib_official_records,
        reference_count=len(reference),
        reference_enu=reference_enu,
        limits=limits,
        official_score_pct=float(lib_official["ppc_official_score_pct"]),
        score_threshold_m=args.score_threshold_m,
        max_gap_s=args.max_gap_s,
        color_mode=args.color_mode,
        bad_segments=bad_segments,
    )

    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output, dpi=100, facecolor=fig.get_facecolor(), bbox_inches="tight")
    print(f"Saved: {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
