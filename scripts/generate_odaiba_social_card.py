#!/usr/bin/env python3
"""Generate a clean, shareable UrbanNav Tokyo Odaiba social card."""

from __future__ import annotations

import argparse
import os
from pathlib import Path

from generate_driving_comparison import (
    match_to_reference,
    matched_xy,
    padded_xy_limits,
    plot_solver_trajectory,
    read_libgnss_pos,
    read_reference_csv,
    read_rtklib_pos,
    STATUS_ORDER,
    STATUS_STYLES,
    trajectory_enu,
)


BG = "#f4efe6"
TEXT = "#14213d"
MUTED = "#5f6c7b"
LIB = "#c56a1a"
RTKLIB = "#2563eb"
PANEL = "#fffaf2"


def draw_status_legend(ax) -> None:
    from matplotlib.patches import FancyBboxPatch

    box_x = 0.62
    box_y = 0.12
    box_w = 0.33
    box_h = 0.58
    panel = FancyBboxPatch(
        (box_x, box_y),
        box_w,
        box_h,
        boxstyle="round,pad=0.012,rounding_size=0.03",
        linewidth=1.0,
        edgecolor="#d8c9b1",
        facecolor=PANEL,
        zorder=1,
    )
    ax.add_patch(panel)

    ax.text(
        box_x + 0.035,
        box_y + box_h - 0.08,
        "Status legend",
        fontsize=12.0,
        color=MUTED,
        weight="bold",
        va="top",
    )

    for index, status_name in enumerate(STATUS_ORDER):
        x = box_x + 0.045 + index * 0.076
        y = box_y + 0.15
        ax.scatter(
            [x],
            [y],
            s=70,
            color=STATUS_STYLES[status_name],
            edgecolors="white",
            linewidths=0.9,
            zorder=3,
        )
        ax.text(x + 0.018, y, status_name, fontsize=12.0, color=TEXT, va="center")


def draw_trajectory_panel(ax, title: str, reference_xy, matched, solver: str, edge_color: str, limits) -> None:
    from matplotlib.patches import FancyBboxPatch

    ax.set_axis_off()
    panel = FancyBboxPatch(
        (0, 0),
        1,
        1,
        boxstyle="round,pad=0.008,rounding_size=0.03",
        linewidth=1.2,
        edgecolor=edge_color,
        facecolor=PANEL,
    )
    ax.add_patch(panel)
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)
    ax.text(0.05, 0.96, title, fontsize=14, color=TEXT, weight="bold", va="top")
    img_ax = ax.inset_axes([0.045, 0.06, 0.91, 0.84])
    img_ax.plot(reference_xy[:, 0], reference_xy[:, 1], color="black", linewidth=1.3, alpha=0.85, zorder=1)
    plot_solver_trajectory(img_ax, matched, solver=solver, max_gap_s=2.0, point_size=9.5)
    img_ax.set_xlim(limits[0], limits[1])
    img_ax.set_ylim(limits[2], limits[3])
    img_ax.set_aspect("equal", adjustable="box")
    img_ax.set_xlabel("East (m)", fontsize=8.5, color=MUTED)
    img_ax.set_ylabel("North (m)", fontsize=8.5, color=MUTED)
    img_ax.tick_params(labelsize=7.0, colors=MUTED)
    img_ax.grid(True, alpha=0.18)
    for spine in img_ax.spines.values():
        spine.set_alpha(0.10)


def main() -> None:
    parser = argparse.ArgumentParser(prog=os.environ.get("GNSS_CLI_NAME"))
    parser.add_argument("--lib-pos", type=Path, required=True)
    parser.add_argument("--rtklib-pos", type=Path, required=True)
    parser.add_argument("--reference-csv", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--rtklib-2d", type=Path)
    parser.add_argument("--lib-2d", type=Path)
    parser.add_argument("--title", default="UrbanNav Tokyo Odaiba")
    parser.add_argument("--match-tolerance", type=float, default=0.15)
    args = parser.parse_args()

    global plt
    import matplotlib.pyplot as plt

    reference = read_reference_csv(args.reference_csv)
    lib_matched = match_to_reference(read_libgnss_pos(args.lib_pos), reference, args.match_tolerance)
    rtklib_matched = match_to_reference(read_rtklib_pos(args.rtklib_pos), reference, args.match_tolerance)
    reference_enu = trajectory_enu(reference, reference[0])
    ref_xy = reference_enu[:, :2]
    limits = padded_xy_limits(ref_xy, matched_xy(rtklib_matched), matched_xy(lib_matched), min_pad=24.0)

    fig = plt.figure(figsize=(12, 6.3), dpi=100, facecolor=BG)
    grid = fig.add_gridspec(2, 2, height_ratios=[0.25, 1.19], hspace=0.08, wspace=0.10)

    ax_head = fig.add_subplot(grid[0, :])
    ax_head.set_axis_off()
    ax_head.set_xlim(0, 1)
    ax_head.set_ylim(0, 1)
    ax_head.text(0.0, 0.86, args.title, fontsize=15, color=MUTED, weight="bold")
    ax_head.text(0.0, 0.34, "libgnss++ vs RTKLIB", fontsize=29, color=TEXT, weight="bold")
    draw_status_legend(ax_head)

    ax_rtk = fig.add_subplot(grid[1, 0])
    draw_trajectory_panel(ax_rtk, "RTKLIB 2D trajectory", ref_xy, rtklib_matched, "RTKLIB", RTKLIB, limits)

    ax_lib = fig.add_subplot(grid[1, 1])
    draw_trajectory_panel(ax_lib, "libgnss++ 2D trajectory", ref_xy, lib_matched, "libgnss++", LIB, limits)

    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output, dpi=100, facecolor=fig.get_facecolor())
    print(f"Saved: {args.output}")


if __name__ == "__main__":
    main()
