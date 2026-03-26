#!/usr/bin/env python3
"""Generate a README-friendly feature overview card for libgnss++."""

from __future__ import annotations

from pathlib import Path


BG = "#f4efe6"
PANEL = "#fffaf2"
TEXT = "#14213d"
MUTED = "#5f6c7b"
ACCENT = "#c56a1a"
ACCENT_2 = "#2563eb"
ACCENT_3 = "#18794e"


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
    ax.text(x + 0.04 * w, y + h - 0.15 * h, title, fontsize=17, color=TEXT, weight="bold", va="center")
    for index, line in enumerate(lines):
        ax.text(
            x + 0.05 * w,
            y + h - 0.34 * h - index * 0.14 * h,
            f"• {line}",
            fontsize=12.5,
            color=MUTED,
            va="center",
        )


def main() -> None:
    import matplotlib.pyplot as plt

    output = Path(__file__).resolve().parents[1] / "docs" / "libgnsspp_feature_overview.png"

    fig = plt.figure(figsize=(14, 8), dpi=160, facecolor=BG)
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_axis_off()
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)

    ax.text(0.05, 0.91, "What Ships In libgnss++", fontsize=30, color=TEXT, weight="bold")
    ax.text(
        0.05,
        0.85,
        "A native non-GUI GNSS stack for positioning, protocols, automation, and deployment.",
        fontsize=15,
        color=MUTED,
    )

    ax.text(0.05, 0.76, "No RTKLIB runtime dependency", fontsize=16, color=ACCENT, weight="bold")
    ax.text(0.34, 0.76, "C++17 core + CLI + Python + ROS2", fontsize=16, color=ACCENT_2, weight="bold")
    ax.text(0.66, 0.76, "Benchmarks and sign-off scripts included", fontsize=16, color=ACCENT_3, weight="bold")

    add_panel(
        ax,
        0.05,
        0.42,
        0.27,
        0.26,
        "Positioning Engines",
        [
            "SPP, RTK, PPP, and CLAS/MADOCA-style PPP",
            "Dual-frequency workflows with benchmarked sign-off",
            "Open-data regression assets checked into the repo",
        ],
        ACCENT,
    )
    add_panel(
        ax,
        0.365,
        0.42,
        0.27,
        0.26,
        "Protocols and Raw Ingest",
        [
            "RINEX, RTCM, UBX, direct QZSS L6",
            "NMEA, NovAtel, SBP, SBF, Trimble, SkyTraq, BINEX",
            "File, NTRIP, TCP, and serial transport paths",
        ],
        ACCENT_2,
    )
    add_panel(
        ax,
        0.68,
        0.42,
        0.27,
        0.26,
        "Tooling",
        [
            "`gnss spp`, `solve`, `ppp`, `stream`, `convert`",
            "`replay`, `live`, `rcv`, benchmark and sign-off commands",
            "README assets regenerated from in-tree scripts",
        ],
        ACCENT_3,
    )
    add_panel(
        ax,
        0.05,
        0.11,
        0.42,
        0.22,
        "Developer Interfaces",
        [
            "C++ library API for native integration",
            "Python bindings for file-based solve and inspection",
            "ROS2 playback node for `.pos` telemetry publishing",
        ],
        ACCENT_2,
    )
    add_panel(
        ax,
        0.53,
        0.11,
        0.42,
        0.22,
        "Packaging and Ops",
        [
            "CMake install/export, pkg-config, and cpack",
            "Installed-prefix dogfooding and CLI regression coverage",
            "Benchmark artifacts: Odaiba social card, 2D plots, scorecards, summary JSON",
        ],
        ACCENT,
    )

    ax.text(
        0.05,
        0.04,
        "See README for Odaiba 2D comparison, benchmark snapshots, and exact command examples.",
        fontsize=12.5,
        color=MUTED,
    )

    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=160, facecolor=fig.get_facecolor())
    print(f"Saved: {output}")


if __name__ == "__main__":
    main()
