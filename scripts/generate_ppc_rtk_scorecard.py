#!/usr/bin/env python3
"""Generate a README-friendly PPC RTK coverage scorecard."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
import os
from pathlib import Path


BG = "#f7f8fb"
PANEL = "#ffffff"
TEXT = "#172033"
MUTED = "#667085"
GRID = "#d0d5dd"
LIB = "#d97706"
RTKLIB = "#2563eb"
WIN = "#18794e"
EDGE = "#d0d5dd"


@dataclass(frozen=True)
class CoverageRun:
    key: str
    label: str
    lib_positioning_pct: float
    rtklib_positioning_pct: float
    positioning_delta_pct: float
    lib_fix_pct: float
    rtklib_fix_pct: float
    lib_official_score_pct: float
    rtklib_official_score_pct: float
    official_score_delta_pct: float
    p95_h_delta_m: float


DEFAULT_RUNS = [
    CoverageRun("tokyo_run1", "Tokyo r1", 86.201991, 66.287340, 19.914651, 48.582799, 30.522595, 29.329930, 0.028882, 29.301048, -6.973535),
    CoverageRun("tokyo_run2", "Tokyo r2", 95.311988, 84.274943, 11.037045, 60.811740, 27.580394, 68.375721, 16.873271, 51.502450, -18.893530),
    CoverageRun("tokyo_run3", "Tokyo r3", 95.987190, 93.131168, 2.856022, 60.257370, 40.547368, 59.411151, 35.630677, 23.780474, -0.691234),
    CoverageRun("nagoya_run1", "Nagoya r1", 87.883937, 65.821461, 22.062476, 60.306365, 33.756950, 43.032180, 22.446739, 20.585441, -22.632527),
    CoverageRun("nagoya_run2", "Nagoya r2", 86.244842, 69.791556, 16.453286, 40.326340, 18.829594, 20.815568, 11.002465, 9.813103, -27.159796),
    CoverageRun("nagoya_run3", "Nagoya r3", 94.635647, 67.698520, 26.937127, 19.687119, 13.888100, 26.857978, 7.649536, 19.208442, -5.539148),
]


def average(values: list[float]) -> float:
    return sum(values) / len(values) if values else 0.0


def run_label(key: str) -> str:
    city, _, run_name = key.partition("_")
    return f"{city.capitalize()} {run_name.replace('run', 'r')}" if run_name else key


def required_number(mapping: dict[str, object], name: str, context: str) -> float:
    value = mapping.get(name)
    if not isinstance(value, (int, float)):
        raise SystemExit(f"Missing numeric `{name}` in {context}")
    return float(value)


def runs_from_summary(path: Path) -> list[CoverageRun]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    runs = payload.get("runs")
    if not isinstance(runs, list):
        raise SystemExit(f"{path} does not look like a ppc-coverage-matrix summary")

    parsed: list[CoverageRun] = []
    for item in runs:
        if not isinstance(item, dict):
            continue
        key = str(item.get("key", ""))
        metrics = item.get("metrics")
        rtklib = item.get("rtklib")
        delta = item.get("delta_vs_rtklib")
        if not isinstance(metrics, dict) or not isinstance(rtklib, dict) or not isinstance(delta, dict):
            raise SystemExit(f"{path} run `{key}` is missing metrics/rtklib/delta blocks")
        parsed.append(
            CoverageRun(
                key=key,
                label=run_label(key),
                lib_positioning_pct=required_number(metrics, "positioning_rate_pct", key),
                rtklib_positioning_pct=required_number(rtklib, "positioning_rate_pct", key),
                positioning_delta_pct=required_number(delta, "positioning_rate_pct", key),
                lib_fix_pct=required_number(metrics, "fix_rate_pct", key),
                rtklib_fix_pct=required_number(rtklib, "fix_rate_pct", key),
                lib_official_score_pct=required_number(metrics, "ppc_official_score_pct", key),
                rtklib_official_score_pct=required_number(rtklib, "ppc_official_score_pct", key),
                official_score_delta_pct=required_number(delta, "ppc_official_score_pct", key),
                p95_h_delta_m=required_number(delta, "p95_h_m", key),
            )
        )
    if not parsed:
        raise SystemExit(f"{path} did not contain any coverage runs")
    return parsed


def load_runs(summary_json: Path | None) -> list[CoverageRun]:
    if summary_json is None:
        return list(DEFAULT_RUNS)
    return runs_from_summary(summary_json)


def add_panel(ax, x: float, y: float, w: float, h: float, radius: float = 0.018) -> None:
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


def draw_summary_card(
    ax,
    x: float,
    y: float,
    w: float,
    h: float,
    title: str,
    value: str,
    detail: str,
    *,
    value_color: str = TEXT,
) -> None:
    add_panel(ax, x, y, w, h)
    ax.text(x + 0.052 * w, y + h - 0.25 * h, title, fontsize=11.0, color=MUTED, weight="bold")
    ax.text(x + 0.052 * w, y + 0.40 * h, value, fontsize=24.0, color=value_color, weight="bold")
    ax.text(x + 0.052 * w, y + 0.17 * h, detail, fontsize=10.3, color=MUTED)


def draw_positioning_bars(ax, runs: list[CoverageRun]) -> None:
    import numpy as np

    labels = [run.label for run in runs]
    lib_rates = [run.lib_positioning_pct for run in runs]
    rtklib_rates = [run.rtklib_positioning_pct for run in runs]

    positions = np.arange(len(labels))
    width = 0.34
    ax.bar(positions - width / 2, lib_rates, width, color=LIB, label="gnssplusplus")
    ax.bar(positions + width / 2, rtklib_rates, width, color=RTKLIB, label="RTKLIB demo5")
    ax.set_ylim(0, 105)
    ax.set_xticks(positions)
    ax.set_xticklabels(labels, color=TEXT, fontsize=9.2)
    ax.set_ylabel("Positioning rate (%)", color=MUTED, fontsize=9.5)
    ax.tick_params(axis="y", labelsize=8.5, colors=MUTED)
    ax.grid(axis="y", alpha=0.22, color=GRID)
    ax.legend(loc="upper left", frameon=False, fontsize=9.5, ncol=2)
    ax.set_title("PPC coverage profile: positioned epochs", color=TEXT, fontsize=14.5, weight="bold", pad=10)
    for spine in ax.spines.values():
        spine.set_alpha(0.15)

    for index, run in enumerate(runs):
        y = max(run.lib_positioning_pct, run.rtklib_positioning_pct) + 2.0
        ax.text(
            positions[index],
            min(y, 101.0),
            f"+{run.positioning_delta_pct:.1f} pp",
            color=WIN,
            fontsize=8.4,
            ha="center",
            weight="bold",
        )


def draw_delta_table(ax, runs: list[CoverageRun]) -> None:
    ax.set_axis_off()
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)
    ax.text(0.0, 0.965, "Run deltas vs RTKLIB demo5", fontsize=14.5, color=TEXT, weight="bold", va="top")
    ax.text(0.0, 0.895, "Positive pp is better; negative P95 H delta is better.", fontsize=9.4, color=MUTED, va="top")

    headers = ["Run", "Pos", "Fix", "Official", "P95 H"]
    xs = [0.02, 0.40, 0.56, 0.74, 0.98]
    aligns = ["left", "right", "right", "right", "right"]
    header_y = 0.78
    for x, header, align in zip(xs, headers, aligns):
        ax.text(x, header_y, header, fontsize=9.2, color=MUTED, weight="bold", ha=align, va="center")
    ax.plot([0.0, 1.0], [0.735, 0.735], color=EDGE, linewidth=1.0)

    y = 0.655
    for run in runs:
        ax.text(xs[0], y, run.label, fontsize=9.8, color=TEXT, weight="bold", ha="left", va="center")
        ax.text(xs[1], y, f"+{run.positioning_delta_pct:.1f}", fontsize=9.6, color=WIN, weight="bold", ha="right", va="center")
        ax.text(xs[2], y, f"+{run.lib_fix_pct - run.rtklib_fix_pct:.1f}", fontsize=9.6, color=WIN, weight="bold", ha="right", va="center")
        ax.text(xs[3], y, f"+{run.official_score_delta_pct:.1f}", fontsize=9.6, color=WIN, weight="bold", ha="right", va="center")
        ax.text(xs[4], y, f"{run.p95_h_delta_m:.2f} m", fontsize=9.6, color=WIN, weight="bold", ha="right", va="center")
        y -= 0.095


def main() -> int:
    parser = argparse.ArgumentParser(prog=os.environ.get("GNSS_CLI_NAME"))
    parser.add_argument("--output", type=Path, default=Path("docs/ppc_rtk_demo5_scorecard.png"))
    parser.add_argument(
        "--summary-json",
        type=Path,
        default=None,
        help="Optional ppc-coverage-matrix summary JSON to render instead of the embedded current metrics.",
    )
    args = parser.parse_args()

    import matplotlib.pyplot as plt

    runs = load_runs(args.summary_json)
    positioning_wins = sum(1 for run in runs if run.positioning_delta_pct > 0.0)
    avg_positioning_delta = average([run.positioning_delta_pct for run in runs])
    avg_official_delta = average([run.official_score_delta_pct for run in runs])
    avg_p95_delta = average([run.p95_h_delta_m for run in runs])
    min_positioning_delta = min(run.positioning_delta_pct for run in runs)

    fig = plt.figure(figsize=(14, 7.5), dpi=100, facecolor=BG)
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_axis_off()
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)

    ax.text(0.05, 0.925, "PPC RTK coverage profile", fontsize=21, color=MUTED, weight="bold")
    ax.text(0.05, 0.845, "gnssplusplus vs RTKLIB demo5", fontsize=37, color=TEXT, weight="bold")
    ax.text(
        0.05,
        0.785,
        "Same public rover/base/nav observations. Positioning rate is reported separately from Fix rate.",
        fontsize=13.5,
        color=MUTED,
    )

    draw_summary_card(
        ax,
        0.05,
        0.625,
        0.205,
        0.105,
        "Positioning wins",
        f"{positioning_wins}/{len(runs)}",
        f"minimum lead +{min_positioning_delta:.1f} pp",
        value_color=WIN,
    )
    draw_summary_card(
        ax,
        0.282,
        0.625,
        0.205,
        0.105,
        "Average positioning delta",
        f"+{avg_positioning_delta:.1f} pp",
        "valid fallback epochs retained",
        value_color=WIN,
    )
    draw_summary_card(
        ax,
        0.514,
        0.625,
        0.205,
        0.105,
        "Average official-score delta",
        f"+{avg_official_delta:.1f} pp",
        "PPC distance-ratio score",
        value_color=WIN,
    )
    draw_summary_card(
        ax,
        0.746,
        0.625,
        0.205,
        0.105,
        "Average P95 H delta",
        f"{avg_p95_delta:.2f} m",
        "negative means tighter error tail",
        value_color=WIN,
    )

    bars_ax = fig.add_axes([0.065, 0.115, 0.555, 0.405], facecolor=PANEL)
    draw_positioning_bars(bars_ax, runs)

    add_panel(ax, 0.655, 0.105, 0.295, 0.435)
    table_ax = fig.add_axes([0.675, 0.135, 0.255, 0.375], facecolor=PANEL)
    draw_delta_table(table_ax, runs)

    ax.text(
        0.05,
        0.044,
        "Profile: low-cost RTK, no AR/post kinematic output filters, default drift/SPP/bridge-tail guards, match tolerance 0.25 s.",
        fontsize=10.3,
        color=MUTED,
    )

    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output, dpi=100, facecolor=fig.get_facecolor())
    print(f"Saved: {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
