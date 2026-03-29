#!/usr/bin/env python3
"""Render moving-base sign-off matches into a quick-look PNG."""

from __future__ import annotations

import argparse
import os
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
from matplotlib import pyplot as plt  # noqa: E402

from gnss_moving_base_signoff import (  # noqa: E402
    match_solution_to_reference,
    read_pos_records,
    read_reference_rows,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        prog=os.environ.get("GNSS_CLI_NAME"),
        description="Render moving-base baseline and heading errors into a quick-look PNG.",
    )
    parser.add_argument("solution_pos", type=Path)
    parser.add_argument("reference_csv", type=Path)
    parser.add_argument("output_png", type=Path)
    parser.add_argument("--match-tolerance-s", type=float, default=0.25)
    parser.add_argument("--title", default="Moving-base sign-off")
    return parser.parse_args()


def render_matches_plot(matches: list[dict[str, float]], output_png: Path, title: str) -> None:
    if not matches:
        raise SystemExit("No matched moving-base epochs available for plotting")

    indices = list(range(1, len(matches) + 1))
    baseline_errors = [float(match["baseline_error_m"]) for match in matches]
    baseline_lengths = [float(match["baseline_length_m"]) for match in matches]
    heading_indices = [
        index for index, match in zip(indices, matches) if match["heading_error_deg"] is not None
    ]
    heading_errors = [
        float(match["heading_error_deg"]) for match in matches if match["heading_error_deg"] is not None
    ]

    figure, axes = plt.subplots(2, 1, figsize=(10.5, 6.4), sharex=True)
    figure.patch.set_facecolor("#f8f5ec")
    for axis in axes:
        axis.set_facecolor("#fffdf8")
        axis.grid(True, alpha=0.25)

    axes[0].plot(indices, baseline_errors, color="#0f766e", linewidth=1.8, label="baseline error")
    axes[0].plot(indices, baseline_lengths, color="#9ca3af", linewidth=1.0, alpha=0.65, label="baseline length")
    axes[0].set_ylabel("meters")
    axes[0].set_title(title)
    axes[0].legend(loc="upper right")

    if heading_errors:
        axes[1].plot(heading_indices, heading_errors, color="#b45309", linewidth=1.8, label="heading error")
        axes[1].legend(loc="upper right")
    else:
        axes[1].text(
            0.5,
            0.5,
            "No heading samples above threshold",
            ha="center",
            va="center",
            transform=axes[1].transAxes,
            color="#6b7280",
        )
    axes[1].set_ylabel("degrees")
    axes[1].set_xlabel("matched epoch")

    output_png.parent.mkdir(parents=True, exist_ok=True)
    figure.tight_layout()
    figure.savefig(output_png, dpi=170)
    plt.close(figure)


def main() -> int:
    args = parse_args()
    solution_records = read_pos_records(args.solution_pos)
    reference_rows = read_reference_rows(args.reference_csv)
    matches = match_solution_to_reference(solution_records, reference_rows, args.match_tolerance_s)
    render_matches_plot(matches, args.output_png, args.title)
    print(f"Saved: {args.output_png}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
