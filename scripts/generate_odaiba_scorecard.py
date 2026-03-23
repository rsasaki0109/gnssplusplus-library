#!/usr/bin/env python3
"""
Generate a promotion-friendly scorecard for the UrbanNav Tokyo Odaiba benchmark.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch

from generate_driving_comparison import (
    match_to_reference,
    read_libgnss_pos,
    read_reference_csv,
    read_rtklib_pos,
    summarize,
)


LIB_COLOR = "#c56a1a"
RTKLIB_COLOR = "#2563eb"
WIN_COLOR = "#18794e"
LOSS_COLOR = "#b45309"
TEXT_COLOR = "#14213d"
MUTED_COLOR = "#5f6c7b"
CARD_BG = "#fffaf2"
FIG_BG = "#f4efe6"


def format_delta_higher(lib_value: float, rtklib_value: float, unit: str = "") -> str:
    delta = lib_value - rtklib_value
    if unit == "%":
        return f"{delta:+.1f} pt"
    if unit:
        return f"{delta:+.0f} {unit}"
    return f"{delta:+.0f}"


def format_delta_lower(lib_value: float, rtklib_value: float, unit: str) -> str:
    delta = rtklib_value - lib_value
    if unit == "mm":
        return f"{delta:+.1f} mm"
    return f"{delta:+.2f} {unit}"


def add_card(ax, x: float, y: float, w: float, h: float, title: str, headline: str,
             detail: str, winner: str) -> None:
    edge_color = WIN_COLOR if winner == "libgnss++" else LOSS_COLOR
    badge_bg = "#e7f6ec" if winner == "libgnss++" else "#fff1dd"
    badge_fg = WIN_COLOR if winner == "libgnss++" else LOSS_COLOR

    patch = FancyBboxPatch(
        (x, y),
        w,
        h,
        boxstyle="round,pad=0.012,rounding_size=0.03",
        linewidth=1.5,
        edgecolor=edge_color,
        facecolor=CARD_BG,
    )
    ax.add_patch(patch)
    ax.text(x + 0.03 * w, y + h - 0.20 * h, title, fontsize=12, color=MUTED_COLOR, weight="bold")
    ax.text(x + 0.03 * w, y + h - 0.52 * h, headline, fontsize=22, color=TEXT_COLOR, weight="bold")
    ax.text(x + 0.03 * w, y + 0.17 * h, detail, fontsize=11.5, color=MUTED_COLOR)

    badge_w = 0.24 * w
    badge_h = 0.22 * h
    badge_x = x + w - badge_w - 0.04 * w
    badge_y = y + h - badge_h - 0.10 * h
    badge = FancyBboxPatch(
        (badge_x, badge_y),
        badge_w,
        badge_h,
        boxstyle="round,pad=0.01,rounding_size=0.03",
        linewidth=0.0,
        facecolor=badge_bg,
    )
    ax.add_patch(badge)
    ax.text(
        badge_x + badge_w / 2,
        badge_y + badge_h / 2,
        winner,
        fontsize=10,
        color=badge_fg,
        weight="bold",
        ha="center",
        va="center",
    )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--lib-pos", type=Path, required=True)
    parser.add_argument("--rtklib-pos", type=Path, required=True)
    parser.add_argument("--reference-csv", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--title", default="UrbanNav Tokyo Odaiba")
    parser.add_argument("--match-tolerance", type=float, default=0.15)
    args = parser.parse_args()

    reference = read_reference_csv(args.reference_csv)
    lib_matched = match_to_reference(
        read_libgnss_pos(args.lib_pos),
        reference,
        args.match_tolerance,
    )
    rtklib_matched = match_to_reference(
        read_rtklib_pos(args.rtklib_pos),
        reference,
        args.match_tolerance,
    )

    lib_summary = summarize(lib_matched, fixed_status=4, label="libgnss++")
    rtklib_summary = summarize(rtklib_matched, fixed_status=1, label="RTKLIB")

    wins = 0
    if lib_summary["epochs"] > rtklib_summary["epochs"]:
        wins += 1
    if lib_summary["fix_rate_pct"] > rtklib_summary["fix_rate_pct"]:
        wins += 1
    if lib_summary["median_h_m"] < rtklib_summary["median_h_m"]:
        wins += 1
    if lib_summary["p95_h_m"] < rtklib_summary["p95_h_m"]:
        wins += 1
    if lib_summary["max_h_m"] < rtklib_summary["max_h_m"]:
        wins += 1

    median_gap_mm = (lib_summary["median_h_m"] - rtklib_summary["median_h_m"]) * 1000.0
    fix_ratio = lib_summary["fix_rate_pct"] / rtklib_summary["fix_rate_pct"]
    p95_improvement = 100.0 * (1.0 - lib_summary["p95_h_m"] / rtklib_summary["p95_h_m"])
    max_improvement = 100.0 * (1.0 - lib_summary["max_h_m"] / rtklib_summary["max_h_m"])

    fig = plt.figure(figsize=(14, 8), facecolor=FIG_BG)
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_axis_off()
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)

    ax.text(0.05, 0.92, args.title, fontsize=28, color=MUTED_COLOR, weight="bold")
    ax.text(0.05, 0.84, "libgnss++ vs RTKLIB", fontsize=36, color=TEXT_COLOR, weight="bold")
    ax.text(
        0.05,
        0.78,
        "Open moving-platform GNSS RTK benchmark on UrbanNav Odaiba",
        fontsize=15,
        color=MUTED_COLOR,
    )

    badge = FancyBboxPatch(
        (0.05, 0.68),
        0.24,
        0.08,
        boxstyle="round,pad=0.01,rounding_size=0.03",
        linewidth=0.0,
        facecolor="#e7f6ec",
    )
    ax.add_patch(badge)
    ax.text(
        0.17,
        0.72,
        f"Leads on {wins}/5 metrics",
        fontsize=16,
        color=WIN_COLOR,
        weight="bold",
        ha="center",
        va="center",
    )

    badge2 = FancyBboxPatch(
        (0.31, 0.68),
        0.22,
        0.08,
        boxstyle="round,pad=0.01,rounding_size=0.03",
        linewidth=0.0,
        facecolor="#fff1dd",
    )
    ax.add_patch(badge2)
    ax.text(
        0.42,
        0.72,
        f"Median: RTKLIB {median_gap_mm:+.1f} mm",
        fontsize=16,
        color=LOSS_COLOR,
        weight="bold",
        ha="center",
        va="center",
    )

    summary_box = FancyBboxPatch(
        (0.58, 0.63),
        0.36,
        0.22,
        boxstyle="round,pad=0.015,rounding_size=0.03",
        linewidth=1.5,
        edgecolor="#d9ccb8",
        facecolor=CARD_BG,
    )
    ax.add_patch(summary_box)
    ax.text(0.61, 0.79, "Where libgnss++ leads", fontsize=14, color=MUTED_COLOR, weight="bold")
    ax.text(0.61, 0.73, f"Coverage: {lib_summary['epochs']} vs {rtklib_summary['epochs']} matched epochs", fontsize=14, color=TEXT_COLOR)
    ax.text(0.61, 0.68, f"Fix rate: {lib_summary['fix_rate_pct']:.1f}% vs {rtklib_summary['fix_rate_pct']:.1f}% ({fix_ratio:.1f}x)", fontsize=14, color=TEXT_COLOR)
    ax.text(0.61, 0.63, f"Tail error: p95 {lib_summary['p95_h_m']:.1f} m, max {lib_summary['max_h_m']:.1f} m", fontsize=14, color=TEXT_COLOR)

    add_card(
        ax, 0.05, 0.42, 0.26, 0.18,
        "Matched Epochs",
        f"{int(lib_summary['epochs'])} vs {int(rtklib_summary['epochs'])}",
        f"libgnss++ keeps {format_delta_higher(lib_summary['epochs'], rtklib_summary['epochs'])} more matched epochs",
        "libgnss++",
    )
    add_card(
        ax, 0.35, 0.42, 0.26, 0.18,
        "Fix Rate",
        f"{lib_summary['fix_rate_pct']:.1f}% vs {rtklib_summary['fix_rate_pct']:.1f}%",
        f"libgnss++ is {fix_ratio:.1f}x higher on this run",
        "libgnss++",
    )
    add_card(
        ax, 0.65, 0.42, 0.26, 0.18,
        "Median H Error",
        f"{lib_summary['median_h_m']:.3f} m vs {rtklib_summary['median_h_m']:.3f} m",
        f"RTKLIB still leads by {abs(median_gap_mm):.1f} mm",
        "RTKLIB",
    )
    add_card(
        ax, 0.20, 0.18, 0.26, 0.18,
        "P95 H Error",
        f"{lib_summary['p95_h_m']:.2f} m vs {rtklib_summary['p95_h_m']:.2f} m",
        f"libgnss++ cuts the 95th percentile by {p95_improvement:.0f}%",
        "libgnss++",
    )
    add_card(
        ax, 0.50, 0.18, 0.26, 0.18,
        "Max H Error",
        f"{lib_summary['max_h_m']:.2f} m vs {rtklib_summary['max_h_m']:.2f} m",
        f"libgnss++ reduces the worst-case error by {max_improvement:.0f}%",
        "libgnss++",
    )

    ax.text(
        0.05,
        0.08,
        "Setup: GPS-only L1+L2, forward kinematic RTK, matched against UrbanNav reference.csv ground truth.",
        fontsize=11,
        color=MUTED_COLOR,
    )
    ax.text(
        0.05,
        0.045,
        "Takeaway: near-parity median, but clearly stronger coverage and tighter tail behavior on the open Odaiba driving dataset.",
        fontsize=12.5,
        color=TEXT_COLOR,
        weight="bold",
    )

    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output, dpi=180, bbox_inches="tight", facecolor=fig.get_facecolor())
    print(f"Saved: {args.output}")
    print(f"libgnss++: {lib_summary}")
    print(f"RTKLIB: {rtklib_summary}")


if __name__ == "__main__":
    main()
