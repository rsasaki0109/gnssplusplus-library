#!/usr/bin/env python3
"""Generate a README-friendly PPC RTK benchmark scorecard."""

from __future__ import annotations

import argparse
import os
from pathlib import Path


BG = "#f4efe6"
PANEL = "#fffaf2"
TEXT = "#14213d"
MUTED = "#5f6c7b"
LIB = "#c56a1a"
RTKLIB = "#2563eb"
WIN = "#18794e"
EDGE = "#d8c9b1"

TOKYO_RUNS = [
    {
        "run": "run1",
        "lib_fix": 3572,
        "lib_rate": 81.26,
        "rtklib_fix": 2418,
        "rtklib_rate": 30.52,
        "hmed_ratio": 42,
        "vp95_ratio": 29,
    },
    {
        "run": "run2",
        "lib_fix": 4674,
        "lib_rate": 80.12,
        "rtklib_fix": 2127,
        "rtklib_rate": 27.58,
        "hmed_ratio": 52,
        "vp95_ratio": 136,
    },
    {
        "run": "run3",
        "lib_fix": 7516,
        "lib_rate": 86.84,
        "rtklib_fix": 5778,
        "rtklib_rate": 40.55,
        "hmed_ratio": 56,
        "vp95_ratio": 179,
    },
]

NAGOYA_RUNS = [
    {"run": "run1", "fix_delta": 1743, "rate_delta": 58.03, "hmed_ratio": 9},
    {"run": "run2", "fix_delta": 1735, "rate_delta": 64.00, "hmed_ratio": 10},
    {"run": "run3", "fix_delta": 154, "rate_delta": 50.16, "hmed_ratio": 44},
]


def average(values: list[float]) -> float:
    if not values:
        return 0.0
    return sum(values) / float(len(values))


def tokyo_average_rate_delta() -> float:
    return average([row["lib_rate"] - row["rtklib_rate"] for row in TOKYO_RUNS])


def nagoya_average_rate_delta() -> float:
    return average([row["rate_delta"] for row in NAGOYA_RUNS])


def add_panel(ax, x: float, y: float, w: float, h: float, radius: float = 0.025) -> None:
    from matplotlib.patches import FancyBboxPatch

    ax.add_patch(
        FancyBboxPatch(
            (x, y),
            w,
            h,
            boxstyle=f"round,pad=0.010,rounding_size={radius}",
            linewidth=1.0,
            edgecolor=EDGE,
            facecolor=PANEL,
        )
    )


def draw_summary_card(ax, x: float, y: float, w: float, h: float, title: str, value: str, detail: str) -> None:
    add_panel(ax, x, y, w, h, radius=0.020)
    ax.text(x + 0.05 * w, y + h - 0.22 * h, title, fontsize=12, color=MUTED, weight="bold")
    ax.text(x + 0.05 * w, y + 0.40 * h, value, fontsize=27, color=TEXT, weight="bold")
    ax.text(x + 0.05 * w, y + 0.17 * h, detail, fontsize=11.5, color=MUTED)


def draw_tokyo_bars(ax) -> None:
    import numpy as np

    labels = [row["run"] for row in TOKYO_RUNS]
    lib_rates = [row["lib_rate"] for row in TOKYO_RUNS]
    rtklib_rates = [row["rtklib_rate"] for row in TOKYO_RUNS]

    positions = np.arange(len(labels))
    width = 0.34
    ax.bar(positions - width / 2, lib_rates, width, color=LIB, label="gnssplusplus")
    ax.bar(positions + width / 2, rtklib_rates, width, color=RTKLIB, label="RTKLIB demo5")
    ax.set_ylim(0, 100)
    ax.set_xticks(positions)
    ax.set_xticklabels(labels, color=TEXT, fontsize=11)
    ax.set_ylabel("Fix rate (%)", color=MUTED, fontsize=10)
    ax.tick_params(axis="y", labelsize=9, colors=MUTED)
    ax.grid(axis="y", alpha=0.18)
    ax.legend(loc="upper left", frameon=False, fontsize=10)
    ax.set_title("PPC Tokyo: fix rate", color=TEXT, fontsize=15, weight="bold", pad=10)
    for spine in ax.spines.values():
        spine.set_alpha(0.12)

    for index, row in enumerate(TOKYO_RUNS):
        ax.text(
            positions[index] - width / 2,
            row["lib_rate"] + 2.1,
            f"{row['lib_rate']:.1f}%",
            color=TEXT,
            fontsize=9,
            ha="center",
            weight="bold",
        )
        ax.text(
            positions[index] + width / 2,
            row["rtklib_rate"] + 2.1,
            f"{row['rtklib_rate']:.1f}%",
            color=TEXT,
            fontsize=9,
            ha="center",
            weight="bold",
        )
        ax.text(
            positions[index],
            5,
            f"Hmed {row['hmed_ratio']}x\nVp95 {row['vp95_ratio']}x",
            color=MUTED,
            fontsize=8.4,
            ha="center",
            va="bottom",
        )


def draw_nagoya_table(ax) -> None:
    ax.set_axis_off()
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)
    ax.text(0.0, 0.95, "PPC Nagoya: same preset", fontsize=15, color=TEXT, weight="bold", va="top")
    ax.text(
        0.0,
        0.875,
        "gnssplusplus delta vs RTKLIB demo5",
        fontsize=10.5,
        color=MUTED,
        va="top",
    )

    header_y = 0.75
    xs = [0.02, 0.25, 0.50, 0.84]
    headers = ["run", "Fix delta", "Rate delta", "Hmed"]
    for x, header in zip(xs, headers):
        ax.text(x, header_y, header, fontsize=10.5, color=MUTED, weight="bold", va="center")
    ax.plot([0.0, 1.0], [0.69, 0.69], color=EDGE, linewidth=1.0)

    y = 0.57
    for row in NAGOYA_RUNS:
        ax.text(xs[0], y, row["run"], fontsize=12, color=TEXT, weight="bold", va="center")
        ax.text(xs[1], y, f"+{row['fix_delta']}", fontsize=11.5, color=WIN, weight="bold", va="center")
        ax.text(xs[2], y, f"+{row['rate_delta']:.2f} pp", fontsize=11.5, color=WIN, weight="bold", va="center")
        ax.text(xs[3], y, f"{row['hmed_ratio']}x", fontsize=11.5, color=WIN, weight="bold", va="center")
        y -= 0.18


def main() -> int:
    parser = argparse.ArgumentParser(prog=os.environ.get("GNSS_CLI_NAME"))
    parser.add_argument("--output", type=Path, default=Path("docs/ppc_rtk_demo5_scorecard.png"))
    args = parser.parse_args()

    import matplotlib.pyplot as plt

    fig = plt.figure(figsize=(14, 7.5), dpi=100, facecolor=BG)
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_axis_off()
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)

    ax.text(0.05, 0.925, "PPC RTK benchmark", fontsize=22, color=MUTED, weight="bold")
    ax.text(0.05, 0.845, "gnssplusplus vs RTKLIB demo5", fontsize=38, color=TEXT, weight="bold")
    ax.text(
        0.05,
        0.785,
        "Same public rover/base/nav observations. Survey-grade receiver logs plus reference.csv truth, not receiver-engine output.",
        fontsize=14,
        color=MUTED,
    )

    draw_summary_card(
        ax,
        0.05,
        0.625,
        0.27,
        0.105,
        "Tokyo average fix-rate lead",
        f"+{tokyo_average_rate_delta():.1f} pp",
        "3 urban vehicle runs",
    )
    draw_summary_card(
        ax,
        0.365,
        0.625,
        0.27,
        0.105,
        "Nagoya average fix-rate lead",
        f"+{nagoya_average_rate_delta():.1f} pp",
        "same low-cost preset",
    )
    draw_summary_card(
        ax,
        0.68,
        0.625,
        0.27,
        0.105,
        "Phase 2 flags required",
        "0",
        "PPC Tokyo and Nagoya pass on defaults",
    )

    left = fig.add_axes([0.065, 0.11, 0.53, 0.42], facecolor=PANEL)
    draw_tokyo_bars(left)

    add_panel(ax, 0.625, 0.11, 0.325, 0.42, radius=0.020)
    table_ax = fig.add_axes([0.650, 0.145, 0.280, 0.35], facecolor=PANEL)
    draw_nagoya_table(table_ax)

    ax.text(
        0.05,
        0.04,
        "PPC-Dataset: Septentrio mosaic-X5 rover, Trimble Alloy/NetR9 base, broadcast nav, reference trajectory truth.",
        fontsize=10.5,
        color=MUTED,
    )

    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output, dpi=100, facecolor=fig.get_facecolor())
    print(f"Saved: {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
