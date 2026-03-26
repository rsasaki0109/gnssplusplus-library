#!/usr/bin/env python3
"""Generate a clean, shareable UrbanNav Tokyo Odaiba social card."""

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


BG = "#f4efe6"
TEXT = "#14213d"
MUTED = "#5f6c7b"
LIB = "#c56a1a"
RTKLIB = "#2563eb"
PANEL = "#fffaf2"


def improvement_text(lib_value: float, rtklib_value: float) -> str:
    if rtklib_value <= 0.0:
        return "n/a"
    return f"{100.0 * (1.0 - lib_value / rtklib_value):.0f}% lower"


def load_image(path: Path) -> object | None:
    if not path.exists():
        return None
    from PIL import Image

    return Image.open(path).convert("RGB")


def draw_image_panel(ax, title: str, image: object | None, edge_color: str) -> None:
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

    if image is None:
        ax.text(0.5, 0.48, "2D figure not found", fontsize=14, color=MUTED, ha="center", va="center")
        return

    img_ax = ax.inset_axes([0.045, 0.06, 0.91, 0.84])
    img_ax.imshow(image)
    img_ax.axis("off")


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
    lib_summary = summarize(lib_matched, fixed_status=4, label="libgnss++")
    rtklib_summary = summarize(rtklib_matched, fixed_status=1, label="RTKLIB")
    common_pairs = pair_epochs(lib_matched, rtklib_matched, args.match_tolerance)
    lib_common_summary, rtklib_common_summary = summarize_common_epochs(
        common_pairs,
        lib_fixed_status=4,
        rtklib_fixed_status=1,
    )

    output_dir = args.output.parent
    rtklib_2d_path = args.rtklib_2d or output_dir / "driving_odaiba_comparison_rtklib_2d.png"
    lib_2d_path = args.lib_2d or output_dir / "driving_odaiba_comparison_libgnss_2d.png"
    rtklib_img = load_image(rtklib_2d_path)
    lib_img = load_image(lib_2d_path)

    fig = plt.figure(figsize=(12, 6.3), dpi=100, facecolor=BG)
    grid = fig.add_gridspec(3, 2, height_ratios=[0.30, 1.16, 0.16], hspace=0.12, wspace=0.10)

    ax_head = fig.add_subplot(grid[0, :])
    ax_head.set_axis_off()
    ax_head.text(0.0, 0.80, args.title, fontsize=15, color=MUTED, weight="bold")
    ax_head.text(0.0, 0.42, "libgnss++ vs RTKLIB", fontsize=29, color=TEXT, weight="bold")
    ax_head.text(
        0.0,
        0.08,
        f"Odaiba moving benchmark: +{int(lib_summary['epochs'] - rtklib_summary['epochs'])} matched epochs and "
        f"{improvement_text(lib_common_summary['p95_h_m'], rtklib_common_summary['p95_h_m'])} common-epoch p95 tail.",
        fontsize=13.5,
        color=MUTED,
    )

    ax_rtk = fig.add_subplot(grid[1, 0])
    draw_image_panel(ax_rtk, "RTKLIB 2D trajectory", rtklib_img, RTKLIB)

    ax_lib = fig.add_subplot(grid[1, 1])
    draw_image_panel(ax_lib, "libgnss++ 2D trajectory", lib_img, LIB)

    ax_footer = fig.add_subplot(grid[2, :])
    ax_footer.set_axis_off()
    ax_footer.text(
        0.00,
        0.60,
        (
            f"Matched epochs {int(lib_summary['epochs']):,} vs {int(rtklib_summary['epochs']):,}   "
            f"|   Fix rate {lib_summary['fix_rate_pct']:.1f}% vs {rtklib_summary['fix_rate_pct']:.1f}%   "
            f"|   Common median H {lib_common_summary['median_h_m']:.3f} m vs {rtklib_common_summary['median_h_m']:.3f} m   "
            f"|   Common p95 H {lib_common_summary['p95_h_m']:.2f} m vs {rtklib_common_summary['p95_h_m']:.2f} m"
        ),
        fontsize=11.2,
        color=TEXT,
        weight="bold",
        va="center",
    )
    ax_footer.text(
        0.00,
        0.10,
        "Open dataset: UrbanNav Tokyo Odaiba. Full comparison and summary JSON are checked into this repo.",
        fontsize=10.5,
        color=MUTED,
        va="center",
    )

    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output, dpi=100, facecolor=fig.get_facecolor())
    print(f"Saved: {args.output}")


if __name__ == "__main__":
    main()
