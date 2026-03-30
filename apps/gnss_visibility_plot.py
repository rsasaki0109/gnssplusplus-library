#!/usr/bin/env python3
"""Render a visibility CSV into a quick-look PNG."""

from __future__ import annotations

import argparse
import csv
import math
import os
from pathlib import Path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(prog=os.environ.get("GNSS_CLI_NAME"))
    parser.add_argument("csv", type=Path, help="Visibility CSV written by gnss visibility.")
    parser.add_argument(
        "output",
        type=Path,
        nargs="?",
        default=None,
        help="Output PNG path. Defaults to <csv stem>.png.",
    )
    parser.add_argument(
        "--title",
        default="Satellite visibility",
        help="Plot title.",
    )
    parser.add_argument(
        "--max-points",
        type=int,
        default=4000,
        help="Maximum number of plotted points after downsampling (default: 4000).",
    )
    return parser.parse_args()


def load_rows(csv_path: Path) -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    with csv_path.open("r", encoding="utf-8", newline="") as stream:
        reader = csv.DictReader(stream)
        for row in reader:
            rows.append(
                {
                    "epoch_index": int(row["epoch_index"]),
                    "satellite": row["satellite"],
                    "system": row["system"],
                    "azimuth_deg": float(row["azimuth_deg"]),
                    "elevation_deg": float(row["elevation_deg"]),
                    "snr_dbhz": float(row["snr_dbhz"]) if row["snr_dbhz"] else None,
                }
            )
    return rows


def downsample_rows(rows: list[dict[str, object]], limit: int) -> list[dict[str, object]]:
    if limit <= 0 or len(rows) <= limit:
        return rows
    step = max(1, len(rows) // limit)
    sampled = rows[::step]
    if sampled[-1] is not rows[-1]:
        sampled.append(rows[-1])
    return sampled[:limit]


def render_plot(rows: list[dict[str, object]], output: Path, title: str) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    if not rows:
        raise SystemExit("Visibility CSV contains no rows.")

    azimuth_rad = [math.radians(float(row["azimuth_deg"])) for row in rows]
    radius = [90.0 - float(row["elevation_deg"]) for row in rows]
    epoch_index = [int(row["epoch_index"]) for row in rows]
    elevation = [float(row["elevation_deg"]) for row in rows]
    snr = [float(row["snr_dbhz"]) if row["snr_dbhz"] is not None else 0.0 for row in rows]

    fig = plt.figure(figsize=(11, 5.5))
    fig.suptitle(title)

    ax_polar = fig.add_subplot(1, 2, 1, projection="polar")
    scatter = ax_polar.scatter(
        azimuth_rad,
        radius,
        c=snr,
        s=12,
        cmap="viridis",
        alpha=0.85,
        linewidths=0.0,
    )
    ax_polar.set_theta_zero_location("N")
    ax_polar.set_theta_direction(-1)
    ax_polar.set_rlim(90.0, 0.0)
    ax_polar.set_rticks([0.0, 30.0, 60.0, 90.0])
    ax_polar.set_yticklabels(["90°", "60°", "30°", "0°"])
    ax_polar.set_title("Polar visibility")

    ax_time = fig.add_subplot(1, 2, 2)
    ax_time.scatter(epoch_index, elevation, c=snr, s=12, cmap="viridis", alpha=0.85, linewidths=0.0)
    ax_time.set_xlabel("Epoch index")
    ax_time.set_ylabel("Elevation (deg)")
    ax_time.set_title("Elevation vs epoch")
    ax_time.grid(True, alpha=0.3)

    cbar = fig.colorbar(scatter, ax=[ax_polar, ax_time], shrink=0.88, pad=0.08)
    cbar.set_label("SNR (dB-Hz)")

    output.parent.mkdir(parents=True, exist_ok=True)
    fig.tight_layout()
    fig.savefig(output, dpi=160, bbox_inches="tight")
    plt.close(fig)


def main() -> int:
    args = parse_args()
    rows = downsample_rows(load_rows(args.csv), args.max_points)
    output = args.output if args.output is not None else args.csv.with_suffix(".png")
    render_plot(rows, output, args.title)
    print(f"Saved: {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
