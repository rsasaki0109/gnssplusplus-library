#!/usr/bin/env python3
"""Generate a Twitter-ready UrbanNav Tokyo Odaiba social card."""

from __future__ import annotations

import argparse
import os
from pathlib import Path

from generate_driving_comparison import (
    match_to_reference,
    pair_epochs,
    read_libgnss_pos,
    read_reference_csv,
    read_rtklib_pos,
    summarize,
    summarize_common_epochs,
)


BG_COLOR = "#0b1220"
PANEL_COLOR = "#101827"
PANEL_ALT_COLOR = "#172235"
LIB_COLOR = "#d97706"
RTKLIB_COLOR = "#3b82f6"
WIN_COLOR = "#22c55e"
TEXT_COLOR = "#f8fafc"
MUTED_COLOR = "#94a3b8"
HAZE_ORANGE = "#f59e0b"
HAZE_BLUE = "#60a5fa"


def mm_text(delta_m: float) -> str:
    return f"{abs(delta_m) * 1000.0:.1f} mm"


def improvement_text(lib_value: float, rtklib_value: float) -> str:
    if rtklib_value <= 0.0:
        return "n/a"
    return f"{100.0 * (1.0 - lib_value / rtklib_value):.0f}% lower"


def add_badge(ax, x: float, y: float, w: float, h: float, text: str, face: str, text_color: str) -> None:
    badge = FancyBboxPatch(
        (x, y),
        w,
        h,
        boxstyle="round,pad=0.01,rounding_size=0.02",
        linewidth=0.0,
        facecolor=face,
        alpha=0.95,
    )
    ax.add_patch(badge)
    ax.text(
        x + w / 2,
        y + h / 2,
        text,
        fontsize=12.5,
        color=text_color,
        ha="center",
        va="center",
        weight="bold",
    )


def add_metric_card(
    ax,
    x: float,
    y: float,
    w: float,
    h: float,
    label: str,
    headline: str,
    detail: str,
    accent: str,
) -> None:
    patch = FancyBboxPatch(
        (x, y),
        w,
        h,
        boxstyle="round,pad=0.012,rounding_size=0.03",
        linewidth=1.3,
        edgecolor=accent,
        facecolor=PANEL_ALT_COLOR,
    )
    ax.add_patch(patch)
    ax.text(x + 0.05 * w, y + 0.73 * h, label, fontsize=11.5, color=MUTED_COLOR, weight="bold")
    ax.text(x + 0.05 * w, y + 0.43 * h, headline, fontsize=20, color=TEXT_COLOR, weight="bold")
    ax.text(x + 0.05 * w, y + 0.16 * h, detail, fontsize=11.5, color=MUTED_COLOR)


def metric_row(
    ax,
    y: float,
    label: str,
    lib_value: str,
    rtklib_value: str,
    lib_better: bool | None,
) -> None:
    row = FancyBboxPatch(
        (0.60, y),
        0.33,
        0.068,
        boxstyle="round,pad=0.008,rounding_size=0.018",
        linewidth=0.0,
        facecolor=PANEL_ALT_COLOR if int(y * 1000) % 2 == 0 else PANEL_COLOR,
    )
    ax.add_patch(row)
    ax.text(0.62, y + 0.034, label, fontsize=11.5, color=MUTED_COLOR, va="center")
    lib_color = WIN_COLOR if lib_better is True else TEXT_COLOR
    rtklib_color = WIN_COLOR if lib_better is False else TEXT_COLOR
    ax.text(0.80, y + 0.034, lib_value, fontsize=12.5, color=lib_color, va="center", ha="right", weight="bold")
    ax.text(0.91, y + 0.034, rtklib_value, fontsize=12.5, color=rtklib_color, va="center", ha="right", weight="bold")


def main() -> None:
    parser = argparse.ArgumentParser(prog=os.environ.get("GNSS_CLI_NAME"))
    parser.add_argument("--lib-pos", type=Path, required=True)
    parser.add_argument("--rtklib-pos", type=Path, required=True)
    parser.add_argument("--reference-csv", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--title", default="UrbanNav Tokyo Odaiba")
    parser.add_argument("--match-tolerance", type=float, default=0.15)
    args = parser.parse_args()

    global plt, Circle, FancyBboxPatch
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle, FancyBboxPatch

    reference = read_reference_csv(args.reference_csv)
    lib_matched = match_to_reference(read_libgnss_pos(args.lib_pos), reference, args.match_tolerance)
    rtklib_matched = match_to_reference(read_rtklib_pos(args.rtklib_pos), reference, args.match_tolerance)
    lib_summary = summarize(lib_matched, fixed_status=4, label="libgnss++")
    rtklib_summary = summarize(rtklib_matched, fixed_status=1, label="RTKLIB")
    common_pairs = pair_epochs(lib_matched, rtklib_matched, args.match_tolerance)
    lib_common_summary, rtklib_common_summary = summarize_common_epochs(
        common_pairs,
        lib_fixed_status=4,
        rtklib_fixed_status=1,
    )

    matched_gain = int(lib_summary["epochs"] - rtklib_summary["epochs"])
    common_median_gap = lib_common_summary["median_h_m"] - rtklib_common_summary["median_h_m"]
    vertical_gap_mm = (
        rtklib_common_summary["median_abs_up_m"] - lib_common_summary["median_abs_up_m"]
    ) * 1000.0

    fig = plt.figure(figsize=(12, 6.3), dpi=100, facecolor=BG_COLOR)
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_axis_off()
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)

    ax.add_patch(Circle((0.16, 0.84), 0.18, color=HAZE_ORANGE, alpha=0.15))
    ax.add_patch(Circle((0.88, 0.18), 0.22, color=HAZE_BLUE, alpha=0.14))

    ax.text(0.06, 0.90, args.title, fontsize=15, color=MUTED_COLOR, weight="bold")
    ax.text(0.06, 0.83, "libgnss++ vs RTKLIB", fontsize=30, color=TEXT_COLOR, weight="bold")
    ax.text(0.06, 0.74, "Broader coverage. Much tighter tails.", fontsize=27, color=TEXT_COLOR, weight="bold")
    ax.text(
        0.06,
        0.68,
        "Open moving-platform GNSS RTK benchmark on UrbanNav Tokyo Odaiba",
        fontsize=13.5,
        color=MUTED_COLOR,
    )
    ax.text(
        0.06,
        0.63,
        "RTKLIB-free C++17 GNSS/RTK/PPP/CLAS stack with checked-in benchmark artifacts.",
        fontsize=13.5,
        color=MUTED_COLOR,
    )

    add_badge(ax, 0.06, 0.54, 0.17, 0.06, f"+{matched_gain} matched epochs", "#2a1c09", LIB_COLOR)
    add_badge(
        ax,
        0.245,
        0.54,
        0.20,
        0.06,
        f"Common median H within {mm_text(common_median_gap)}",
        "#1f2937",
        TEXT_COLOR,
    )

    add_metric_card(
        ax,
        0.06,
        0.29,
        0.16,
        0.18,
        "Matched epochs",
        f"{int(lib_summary['epochs']):,} vs {int(rtklib_summary['epochs']):,}",
        f"libgnss++ keeps {matched_gain:,} more matched epochs",
        LIB_COLOR,
    )
    add_metric_card(
        ax,
        0.24,
        0.29,
        0.16,
        0.18,
        "All-epoch fix rate",
        f"{lib_summary['fix_rate_pct']:.1f}% vs {rtklib_summary['fix_rate_pct']:.1f}%",
        "libgnss++ stays ahead on moving epochs",
        WIN_COLOR,
    )
    add_metric_card(
        ax,
        0.42,
        0.29,
        0.16,
        0.18,
        "Common p95 H",
        f"{lib_common_summary['p95_h_m']:.2f} m vs {rtklib_common_summary['p95_h_m']:.2f} m",
        f"{improvement_text(lib_common_summary['p95_h_m'], rtklib_common_summary['p95_h_m'])} tail error",
        WIN_COLOR,
    )

    panel = FancyBboxPatch(
        (0.60, 0.16),
        0.33,
        0.66,
        boxstyle="round,pad=0.015,rounding_size=0.03",
        linewidth=0.0,
        facecolor=PANEL_COLOR,
    )
    ax.add_patch(panel)
    ax.text(0.63, 0.77, "Benchmark snapshot", fontsize=14, color=MUTED_COLOR, weight="bold")
    ax.text(0.80, 0.73, "libgnss++", fontsize=13, color=LIB_COLOR, weight="bold", ha="right")
    ax.text(0.91, 0.73, "RTKLIB", fontsize=13, color=RTKLIB_COLOR, weight="bold", ha="right")

    metric_row(ax, 0.64, "All matched epochs", f"{int(lib_summary['epochs']):,}", f"{int(rtklib_summary['epochs']):,}", True)
    metric_row(ax, 0.56, "All fix rate", f"{lib_summary['fix_rate_pct']:.1f}%", f"{rtklib_summary['fix_rate_pct']:.1f}%", True)
    metric_row(
        ax,
        0.48,
        "Common median H",
        f"{lib_common_summary['median_h_m']:.3f} m",
        f"{rtklib_common_summary['median_h_m']:.3f} m",
        lib_common_summary["median_h_m"] < rtklib_common_summary["median_h_m"],
    )
    metric_row(
        ax,
        0.40,
        "Common median |Up|",
        f"{lib_common_summary['median_abs_up_m']:.3f} m",
        f"{rtklib_common_summary['median_abs_up_m']:.3f} m",
        lib_common_summary["median_abs_up_m"] < rtklib_common_summary["median_abs_up_m"],
    )
    metric_row(ax, 0.32, "Common p95 H", f"{lib_common_summary['p95_h_m']:.2f} m", f"{rtklib_common_summary['p95_h_m']:.2f} m", True)

    ax.text(0.63, 0.23, f"Vertical median edge: libgnss++ by {vertical_gap_mm:.1f} mm", fontsize=12.5, color=MUTED_COLOR)
    ax.text(
        0.06,
        0.14,
        "Source: UrbanNav Tokyo Odaiba open dataset. Full comparison, 2D plots, and machine-readable summaries ship in-tree.",
        fontsize=11.5,
        color=MUTED_COLOR,
    )
    ax.text(0.06, 0.09, "Share image: docs/driving_odaiba_social_card.png", fontsize=11.5, color=TEXT_COLOR, weight="bold")

    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output, dpi=100, facecolor=fig.get_facecolor())
    print(f"Saved: {args.output}")


if __name__ == "__main__":
    main()
