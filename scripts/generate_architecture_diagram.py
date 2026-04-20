#!/usr/bin/env python3
"""Generate an architecture diagram for libgnss++ docs."""

from __future__ import annotations

import argparse
import os
from pathlib import Path


BG = "#f4f1e8"
PANEL = "#fffdf8"
TEXT = "#162033"
MUTED = "#5f6c7b"
LINE = "#d8d0c3"
ACCENT = "#c56a1a"
ACCENT_2 = "#2563eb"
ACCENT_3 = "#18794e"
ACCENT_4 = "#a855f7"


def add_box(ax, x: float, y: float, w: float, h: float, title: str, lines: list[str], edge: str) -> None:
    from matplotlib.patches import FancyBboxPatch

    patch = FancyBboxPatch(
        (x, y),
        w,
        h,
        boxstyle="round,pad=0.010,rounding_size=0.025",
        linewidth=1.4,
        edgecolor=edge,
        facecolor=PANEL,
    )
    ax.add_patch(patch)
    ax.text(x + 0.04 * w, y + h - 0.16 * h, title, fontsize=16, color=TEXT, weight="bold", va="center")
    start_y = y + h - 0.34 * h
    step = 0.16 * h
    for index, line in enumerate(lines):
        ax.text(x + 0.05 * w, start_y - index * step, f"• {line}", fontsize=10.8, color=MUTED, va="center")


def add_arrow(ax, x0: float, y0: float, x1: float, y1: float, color: str = "#7b8794") -> None:
    ax.annotate(
        "",
        xy=(x1, y1),
        xytext=(x0, y0),
        arrowprops={"arrowstyle": "-|>", "lw": 1.8, "color": color, "shrinkA": 8, "shrinkB": 8},
    )


def main() -> None:
    import matplotlib.pyplot as plt

    parser = argparse.ArgumentParser(prog=os.environ.get("GNSS_CLI_NAME"))
    parser.add_argument(
        "--output",
        type=Path,
        default=Path(__file__).resolve().parents[1] / "docs" / "libgnsspp_architecture.png",
        help="Output path for the architecture diagram PNG.",
    )
    args = parser.parse_args()

    output = args.output

    fig = plt.figure(figsize=(14, 8.8), dpi=160, facecolor=BG)
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_axis_off()
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)

    ax.text(0.05, 0.94, "How libgnss++ is organized", fontsize=28, color=TEXT, weight="bold")
    ax.text(
        0.05,
        0.89,
        "Explicit state, staged solvers, shared ingest policy, and reproducible validation.",
        fontsize=14,
        color=MUTED,
    )

    add_box(
        ax,
        0.05,
        0.60,
        0.22,
        0.20,
        "Protocols and Ingest",
        ["RINEX / RTCM / UBX", "direct QZSS L6", "raw/log formats"],
        ACCENT,
    )
    add_box(
        ax,
        0.05,
        0.32,
        0.22,
        0.20,
        "Shared Core",
        ["signal policy", "navigation products", "time / coords / solutions"],
        ACCENT_2,
    )
    add_box(
        ax,
        0.34,
        0.64,
        0.26,
        0.14,
        "SPP / RTK / PPP",
        ["native solver processors", "explicit filter state"],
        ACCENT_3,
    )
    add_box(
        ax,
        0.34,
        0.42,
        0.26,
        0.14,
        "RTK Stages",
        ["selection", "measurement", "update", "AR selection / evaluation", "validation"],
        ACCENT_4,
    )
    add_box(
        ax,
        0.34,
        0.18,
        0.26,
        0.14,
        "PPP / CLAS Stages",
        ["precise products", "ppp_atmosphere", "ambiguity / correction path"],
        ACCENT_2,
    )
    add_box(
        ax,
        0.67,
        0.60,
        0.28,
        0.20,
        "Interfaces",
        ["CLI and sign-off commands", "Python bindings", "ROS2 playback", "local web UI"],
        ACCENT_3,
    )
    add_box(
        ax,
        0.67,
        0.32,
        0.28,
        0.20,
        "Validation and Ops",
        ["benchmark JSON and cards", "installed-prefix dogfooding", "CI / browser / packaging tests"],
        ACCENT,
    )

    add_arrow(ax, 0.27, 0.70, 0.34, 0.71)
    add_arrow(ax, 0.27, 0.42, 0.34, 0.49)
    add_arrow(ax, 0.47, 0.64, 0.47, 0.56)
    add_arrow(ax, 0.47, 0.42, 0.47, 0.32)
    add_arrow(ax, 0.60, 0.71, 0.67, 0.71)
    add_arrow(ax, 0.60, 0.49, 0.67, 0.43)
    add_arrow(ax, 0.81, 0.60, 0.81, 0.52)

    ax.text(
        0.05,
        0.06,
        "Design intent: one native GNSS stack, multiple interfaces, shared validation, no separate hidden solver path.",
        fontsize=11.5,
        color=MUTED,
    )

    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=160, facecolor=fig.get_facecolor())
    print(f"Saved: {output}")


if __name__ == "__main__":
    main()
