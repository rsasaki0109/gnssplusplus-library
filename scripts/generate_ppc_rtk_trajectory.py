#!/usr/bin/env python3
"""Generate a PPC RTK 2D trajectory figure colored by solution status."""

from __future__ import annotations

import argparse
import math
import os
from pathlib import Path

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


BG = "#f4efe6"
PANEL = "#fffaf2"
TEXT = "#14213d"
MUTED = "#5f6c7b"
EDGE = "#d8c9b1"


def ppc_3d_score(matched, threshold_m: float) -> tuple[int, float]:
    if not matched:
        return 0, 0.0
    scored = sum(
        1
        for epoch in matched
        if math.hypot(epoch.horiz_error_m, epoch.up_m) <= threshold_m
    )
    return scored, 100.0 * scored / len(matched)


def status_counts(matched, solver: str) -> dict[str, int]:
    counts: dict[str, int] = {}
    for epoch in matched:
        status_name, _ = status_style(solver, epoch.status)
        counts[status_name] = counts.get(status_name, 0) + 1
    return counts


def draw_solver_panel(
    ax,
    *,
    title: str,
    solver: str,
    matched,
    reference_enu,
    limits: tuple[float, float, float, float],
    score_threshold_m: float,
    max_gap_s: float,
) -> None:
    ax.set_facecolor(PANEL)
    ax.plot(reference_enu[:, 0], reference_enu[:, 1], color="black", linewidth=1.5, alpha=0.78)
    plot_solver_trajectory(ax, matched, solver=solver, max_gap_s=max_gap_s, point_size=9.5)
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

    scored, score_pct = ppc_3d_score(matched, score_threshold_m)
    counts = status_counts(matched, solver)
    detail = "\n".join(
        [
            f"matched {len(matched)}",
            f"3D<={score_threshold_m:.2f}m {score_pct:.1f}%",
            f"FIXED {counts.get('FIXED', 0)}",
            f"FLOAT {counts.get('FLOAT', 0)}",
            f"DGPS  {counts.get('DGPS', 0)}",
            f"SPP   {counts.get('SPP', 0)}",
        ]
    )
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
    )
    ax.text(
        0.975,
        0.035,
        f"{scored}/{len(matched)} epochs",
        transform=ax.transAxes,
        va="bottom",
        ha="right",
        fontsize=8.5,
        color=MUTED,
        bbox=dict(boxstyle="round,pad=0.25", facecolor="white", edgecolor=EDGE, alpha=0.80),
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
        default="2D track colored by solution status; PPC-style score uses 3D error <= 0.50 m.",
    )
    parser.add_argument("--match-tolerance-s", type=float, default=0.25)
    parser.add_argument("--score-threshold-m", type=float, default=0.50)
    parser.add_argument("--max-gap-s", type=float, default=2.0)
    args = parser.parse_args()

    import matplotlib.pyplot as plt

    reference = read_reference_csv(args.reference_csv)
    lib_matched = match_to_reference(
        read_libgnss_pos(args.lib_pos),
        reference,
        args.match_tolerance_s,
    )
    rtklib_matched = match_to_reference(
        read_rtklib_pos(args.rtklib_pos),
        reference,
        args.match_tolerance_s,
    )
    if not lib_matched:
        raise SystemExit("No gnssplusplus epochs matched the PPC reference.")
    if not rtklib_matched:
        raise SystemExit("No RTKLIB epochs matched the PPC reference.")

    reference_enu = trajectory_enu(reference, reference[0])
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
    ax_head.legend(
        handles=build_status_legend_handles(),
        loc="center right",
        ncol=4,
        frameon=False,
        fontsize=10,
    )

    ax_rtklib = fig.add_subplot(grid[1, 0])
    draw_solver_panel(
        ax_rtklib,
        title="RTKLIB demo5",
        solver="RTKLIB",
        matched=rtklib_matched,
        reference_enu=reference_enu,
        limits=limits,
        score_threshold_m=args.score_threshold_m,
        max_gap_s=args.max_gap_s,
    )
    ax_lib = fig.add_subplot(grid[1, 1])
    draw_solver_panel(
        ax_lib,
        title="gnssplusplus",
        solver="libgnss++",
        matched=lib_matched,
        reference_enu=reference_enu,
        limits=limits,
        score_threshold_m=args.score_threshold_m,
        max_gap_s=args.max_gap_s,
    )

    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output, dpi=100, facecolor=fig.get_facecolor(), bbox_inches="tight")
    print(f"Saved: {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
