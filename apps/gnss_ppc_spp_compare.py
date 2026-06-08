#!/usr/bin/env python3
"""Compare multiple SPP .pos files against a PPC reference trajectory."""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
import json
from pathlib import Path
import sys
from typing import Any
import warnings


ROOT_DIR = Path(__file__).resolve().parent.parent
SCRIPTS_DIR = ROOT_DIR / "scripts"

if str(SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_DIR))

import generate_driving_comparison as comparison  # noqa: E402
import gnss_ppc_metrics as ppc_metrics  # noqa: E402
from gnss_ppc_demo import read_flexible_reference_csv  # noqa: E402


SUMMARY_FIELDS = [
    "label",
    "path",
    "valid_epochs",
    "matched_epochs",
    "positioning_rate_pct",
    "mean_h_m",
    "median_h_m",
    "p95_h_m",
    "max_h_m",
    "median_abs_up_m",
    "p95_abs_up_m",
    "mean_up_m",
    "mean_satellites",
    "ppc_score_3d_50cm_ref_pct",
    "ppc_official_score_pct",
    "delta_valid_epochs",
    "delta_matched_epochs",
    "delta_positioning_rate_pct",
    "delta_median_h_m",
    "delta_p95_h_m",
    "delta_max_h_m",
    "delta_p95_abs_up_m",
]


COLORS = [
    "#2563eb",
    "#d97706",
    "#059669",
    "#dc2626",
    "#7c3aed",
    "#0891b2",
]


@dataclass(frozen=True)
class SolutionSpec:
    label: str
    path: Path


@dataclass(frozen=True)
class SolutionComparison:
    spec: SolutionSpec
    epochs: list[comparison.SolutionEpoch]
    matched: list[comparison.MatchedEpoch]
    summary: dict[str, Any]


def parse_solution_spec(text: str) -> SolutionSpec:
    if "=" in text:
        label, path_text = text.split("=", 1)
        label = label.strip()
        path_text = path_text.strip()
        if not label or not path_text:
            raise argparse.ArgumentTypeError("expected --solution label=path")
        return SolutionSpec(label, Path(path_text))
    path = Path(text)
    return SolutionSpec(path.stem, path)


def build_solution_comparison(
    spec: SolutionSpec,
    reference: list[comparison.ReferenceEpoch],
    match_tolerance_s: float,
) -> SolutionComparison:
    if not spec.path.exists():
        raise SystemExit(f"Solution file not found: {spec.path}")
    epochs = comparison.read_libgnss_pos(spec.path)
    if not epochs:
        raise SystemExit(f"No solution epochs found in {spec.path}")
    matched = comparison.match_to_reference(epochs, reference, match_tolerance_s)
    if not matched:
        raise SystemExit(f"No PPC epochs matched reference for {spec.label}: {spec.path}")
    summary = ppc_metrics.summarize_solution_epochs(
        reference,
        epochs,
        1,
        spec.label,
        match_tolerance_s,
        None,
    )
    summary.update({"label": spec.label, "path": str(spec.path)})
    return SolutionComparison(spec, epochs, matched, summary)


def write_summary_csv(
    path: Path,
    comparisons: list[SolutionComparison],
    baseline: SolutionComparison,
) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=SUMMARY_FIELDS)
        writer.writeheader()
        for item in comparisons:
            delta = ppc_metrics.solution_metric_delta(item.summary, baseline.summary)
            row = dict(item.summary)
            row.update(
                {
                    "delta_valid_epochs": delta["valid_epochs"],
                    "delta_matched_epochs": delta["matched_epochs"],
                    "delta_positioning_rate_pct": delta["positioning_rate_pct"],
                    "delta_median_h_m": delta["median_h_m"],
                    "delta_p95_h_m": delta["p95_h_m"],
                    "delta_max_h_m": delta["max_h_m"],
                    "delta_p95_abs_up_m": delta["p95_abs_up_m"],
                }
            )
            writer.writerow({field: row.get(field) for field in SUMMARY_FIELDS})


def write_matched_csv(path: Path, comparisons: list[SolutionComparison]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(
            [
                "solution_label",
                "gps_tow_s",
                "traj_east_m",
                "traj_north_m",
                "traj_up_m",
                "east_error_m",
                "north_error_m",
                "up_error_m",
                "horizontal_error_m",
                "status",
            ]
        )
        for item in comparisons:
            for match in item.matched:
                writer.writerow(
                    [
                        item.spec.label,
                        f"{match.tow:.3f}",
                        f"{match.traj_east_m:.6f}",
                        f"{match.traj_north_m:.6f}",
                        f"{match.traj_up_m:.6f}",
                        f"{match.east_m:.6f}",
                        f"{match.north_m:.6f}",
                        f"{match.up_m:.6f}",
                        f"{match.horiz_error_m:.6f}",
                        int(match.status),
                    ]
                )


def write_summary_json(
    path: Path,
    reference_csv: Path,
    match_tolerance_s: float,
    comparisons: list[SolutionComparison],
    baseline: SolutionComparison,
) -> None:
    payload: dict[str, Any] = {
        "reference_csv": str(reference_csv),
        "match_tolerance_s": match_tolerance_s,
        "baseline_label": baseline.spec.label,
        "solutions": [],
    }
    for item in comparisons:
        delta = ppc_metrics.solution_metric_delta(item.summary, baseline.summary)
        payload["solutions"].append(
            {
                **item.summary,
                "delta_vs_baseline": delta,
            }
        )
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")


def seconds_from_first(item: SolutionComparison) -> list[float]:
    first = ppc_metrics.week_tow_to_seconds(item.epochs[0].week, item.matched[0].tow)
    return [ppc_metrics.week_tow_to_seconds(item.epochs[0].week, match.tow) - first for match in item.matched]


def plot_comparison(
    path: Path,
    reference: list[comparison.ReferenceEpoch],
    comparisons: list[SolutionComparison],
    title: str,
) -> None:
    warnings.filterwarnings("ignore", message="Unable to import Axes3D.*", category=UserWarning)
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    origin = reference[0]
    ref_enu = comparison.trajectory_enu(reference, origin)

    fig, axes = plt.subplots(2, 2, figsize=(15.2, 10.0))
    ax_track, ax_time, ax_cdf, ax_bar = axes.flatten()

    ax_track.plot(ref_enu[:, 0], ref_enu[:, 1], color="#111827", linewidth=1.6, label="reference")
    for index, item in enumerate(comparisons):
        color = COLORS[index % len(COLORS)]
        points = comparison.matched_xy(item.matched)
        ax_track.plot(points[:, 0], points[:, 1], color=color, linewidth=0.85, alpha=0.78, label=item.spec.label)
    ax_track.set_title("Trajectory")
    ax_track.set_xlabel("East (m)")
    ax_track.set_ylabel("North (m)")
    ax_track.set_aspect("equal", adjustable="box")
    ax_track.grid(alpha=0.25)
    ax_track.legend(fontsize=8)

    for index, item in enumerate(comparisons):
        color = COLORS[index % len(COLORS)]
        elapsed_s = seconds_from_first(item)
        horiz = [match.horiz_error_m for match in item.matched]
        ax_time.plot(elapsed_s, horiz, color=color, linewidth=0.9, alpha=0.84, label=item.spec.label)
    ax_time.set_title("Horizontal Error")
    ax_time.set_xlabel("Elapsed time (s)")
    ax_time.set_ylabel("Horizontal error (m)")
    ax_time.grid(alpha=0.25)
    ax_time.legend(fontsize=8)

    for index, item in enumerate(comparisons):
        color = COLORS[index % len(COLORS)]
        horiz, pct = comparison.cdf_xy(item.matched)
        ax_cdf.plot(horiz, pct, color=color, linewidth=1.5, label=item.spec.label)
    ax_cdf.set_title("Horizontal Error CDF")
    ax_cdf.set_xlabel("Horizontal error (m)")
    ax_cdf.set_ylabel("Percentile (%)")
    ax_cdf.set_xlim(left=0.0)
    ax_cdf.set_ylim(0.0, 100.0)
    ax_cdf.grid(alpha=0.25)
    ax_cdf.legend(fontsize=8)

    labels = [item.spec.label for item in comparisons]
    p95_values = [float(item.summary["p95_h_m"]) for item in comparisons]
    bars = ax_bar.bar(labels, p95_values, color=[COLORS[i % len(COLORS)] for i in range(len(labels))])
    ax_bar.set_title("P95 Horizontal Error")
    ax_bar.set_ylabel("P95 H (m)")
    ax_bar.grid(axis="y", alpha=0.25)
    ax_bar.tick_params(axis="x", rotation=18)
    for bar, item in zip(bars, comparisons):
        positioning = float(item.summary["positioning_rate_pct"])
        ax_bar.text(
            bar.get_x() + bar.get_width() / 2.0,
            bar.get_height(),
            f"{bar.get_height():.2f} m\n{positioning:.2f}%",
            ha="center",
            va="bottom",
            fontsize=8.0,
        )

    fig.suptitle(title)
    fig.tight_layout(rect=(0.0, 0.0, 1.0, 0.965))
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=170, bbox_inches="tight")
    plt.close(fig)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--reference-csv", type=Path, required=True)
    parser.add_argument(
        "--solution",
        type=parse_solution_spec,
        action="append",
        required=True,
        help="Solution as label=path. Repeat for each SPP candidate.",
    )
    parser.add_argument("--baseline-label", help="Label to use for delta columns; default is the first solution")
    parser.add_argument("--match-tolerance-s", type=float, default=0.25)
    parser.add_argument("--summary-json", type=Path, required=True)
    parser.add_argument("--csv", type=Path, required=True)
    parser.add_argument("--matched-csv", type=Path, required=True)
    parser.add_argument("--png", type=Path, required=True)
    parser.add_argument("--title", default="PPC SPP candidate comparison")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    reference = read_flexible_reference_csv(args.reference_csv)
    if not reference:
        raise SystemExit(f"No reference epochs found in {args.reference_csv}")

    comparisons = [
        build_solution_comparison(spec, reference, args.match_tolerance_s)
        for spec in args.solution
    ]
    labels = [item.spec.label for item in comparisons]
    if len(set(labels)) != len(labels):
        raise SystemExit("--solution labels must be unique")

    if args.baseline_label is None:
        baseline = comparisons[0]
    else:
        baseline = next((item for item in comparisons if item.spec.label == args.baseline_label), None)
        if baseline is None:
            raise SystemExit(f"--baseline-label not found: {args.baseline_label}")

    write_summary_json(args.summary_json, args.reference_csv, args.match_tolerance_s, comparisons, baseline)
    write_summary_csv(args.csv, comparisons, baseline)
    write_matched_csv(args.matched_csv, comparisons)
    plot_comparison(args.png, reference, comparisons, args.title)

    print("PPC SPP comparison")
    print(f"  solutions: {len(comparisons)}")
    print(f"  baseline: {baseline.spec.label}")
    print(f"  summary: {args.summary_json}")
    print(f"  csv: {args.csv}")
    print(f"  matched_csv: {args.matched_csv}")
    print(f"  png: {args.png}")
    for item in comparisons:
        delta = ppc_metrics.solution_metric_delta(item.summary, baseline.summary)
        print(
            f"    {item.spec.label}: "
            f"matched={item.summary['matched_epochs']} "
            f"pos={float(item.summary['positioning_rate_pct']):.3f}% "
            f"median_h={float(item.summary['median_h_m']):.3f} "
            f"p95_h={float(item.summary['p95_h_m']):.3f} "
            f"delta_p95={float(delta['p95_h_m']):.3f}"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
