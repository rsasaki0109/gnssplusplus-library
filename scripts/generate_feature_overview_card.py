#!/usr/bin/env python3
"""Generate a README-friendly feature overview card for libgnss++."""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import textwrap


BG = "#f4efe6"
PANEL = "#fffaf2"
TEXT = "#14213d"
MUTED = "#5f6c7b"
ACCENT = "#c56a1a"
ACCENT_2 = "#2563eb"
ACCENT_3 = "#18794e"


def wrap_lines(lines: list[str], width: int) -> list[str]:
    wrapped: list[str] = []
    for line in lines:
        parts = textwrap.wrap(line, width=width)
        if not parts:
            wrapped.append("•")
            continue
        wrapped.append(f"• {parts[0]}")
        for part in parts[1:]:
            wrapped.append(f"  {part}")
    return wrapped


def add_panel(ax, x: float, y: float, w: float, h: float, title: str, lines: list[str], edge: str) -> None:
    from matplotlib.patches import FancyBboxPatch

    patch = FancyBboxPatch(
        (x, y),
        w,
        h,
        boxstyle="round,pad=0.012,rounding_size=0.03",
        linewidth=1.3,
        edgecolor=edge,
        facecolor=PANEL,
    )
    ax.add_patch(patch)
    ax.text(x + 0.04 * w, y + h - 0.14 * h, title, fontsize=15, color=TEXT, weight="bold", va="center")
    wrapped = wrap_lines(lines, 34 if w >= 0.4 else 26)
    line_step = 0.10 * h
    start_y = y + h - 0.34 * h
    for index, line in enumerate(wrapped):
        ax.text(
            x + 0.05 * w,
            start_y - index * line_step,
            line,
            fontsize=10.6,
            color=MUTED,
            va="center",
        )


def main() -> None:
    import matplotlib.pyplot as plt

    parser = argparse.ArgumentParser(prog=os.environ.get("GNSS_CLI_NAME"))
    parser.add_argument(
        "--output",
        type=Path,
        default=Path(__file__).resolve().parents[1] / "docs" / "libgnsspp_feature_overview.png",
        help="Output path for the feature overview PNG.",
    )
    args = parser.parse_args()

    output = args.output

    fig = plt.figure(figsize=(14, 8.6), dpi=160, facecolor=BG)
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_axis_off()
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)

    ax.text(0.05, 0.92, "What Ships In libgnss++", fontsize=28, color=TEXT, weight="bold")
    ax.text(
        0.05,
        0.86,
        "A native non-GUI GNSS stack for positioning, protocols, automation, and deployment.",
        fontsize=14,
        color=MUTED,
    )

    ax.text(0.05, 0.78, "No RTKLIB runtime dependency", fontsize=14, color=ACCENT, weight="bold")
    ax.text(0.34, 0.78, "C++17 core + CLI + Python + ROS2", fontsize=14, color=ACCENT_2, weight="bold")
    ax.text(0.64, 0.78, "Benchmarks and sign-off scripts included", fontsize=14, color=ACCENT_3, weight="bold")

    add_panel(
        ax,
        0.05,
        0.43,
        0.27,
        0.29,
        "Positioning Engines",
        [
            "SPP, RTK, PPP, CLAS-style PPP",
            "Dual-frequency workflows",
            "Sign-off scripts and open-data regressions",
        ],
        ACCENT,
    )
    add_panel(
        ax,
        0.365,
        0.43,
        0.27,
        0.29,
        "Protocols and Raw Ingest",
        [
            "RINEX, RTCM, UBX, direct QZSS L6",
            "NMEA, NovAtel, SBP, SBF, Trimble, BINEX",
            "File, NTRIP, TCP, and serial paths",
        ],
        ACCENT_2,
    )
    add_panel(
        ax,
        0.68,
        0.43,
        0.27,
        0.29,
        "Tooling",
        [
            "gnss spp, solve, ppp, stream",
            "convert, replay, live, rcv",
            "Benchmark and README asset generators",
        ],
        ACCENT_3,
    )
    add_panel(
        ax,
        0.05,
        0.10,
        0.42,
        0.26,
        "Developer Interfaces",
        [
            "Native C++ API for integration",
            "Python bindings for solve and inspection",
            "ROS2 playback for POS telemetry",
        ],
        ACCENT_2,
    )
    add_panel(
        ax,
        0.53,
        0.10,
        0.42,
        0.26,
        "Packaging and Ops",
        [
            "CMake install/export, pkg-config, cpack",
            "Installed-prefix dogfooding and CLI regressions",
            "Odaiba cards, 2D plots, scorecards, summary JSON",
        ],
        ACCENT,
    )

    ax.text(
        0.05,
        0.035,
        "See README for Odaiba 2D comparison, benchmark snapshots, and exact command examples.",
        fontsize=11.8,
        color=MUTED,
    )

    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=160, facecolor=fig.get_facecolor())
    print(f"Saved: {output}")


if __name__ == "__main__":
    main()
