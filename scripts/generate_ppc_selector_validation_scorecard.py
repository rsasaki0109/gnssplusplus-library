#!/usr/bin/env python3
"""Generate a PPC segment-selector validation scorecard."""

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
BASELINE = "#64748b"
ROBUST = "#0f766e"
CONSTRAINED = "#2563eb"
WIN = "#18794e"
LOSS = "#b42318"
EDGE = "#d0d5dd"


@dataclass(frozen=True)
class SummarySpec:
    label: str
    path: Path


@dataclass(frozen=True)
class ObjectiveSummary:
    label: str
    holdout_net_m: float
    selector_vs_candidate_m: float
    precision_pct: float
    nonnegative_runs: int
    fold_count: int
    min_holdout_m: float
    holdout_loss_m: float
    folds: tuple[dict[str, object], ...]


def parse_summary_spec(value: str) -> SummarySpec:
    label, separator, path_text = value.partition("=")
    if not separator or not label.strip() or not path_text.strip():
        raise SystemExit("--summary must use LABEL=JSON")
    return SummarySpec(label.strip(), Path(path_text.strip()))


def required_number(mapping: dict[str, object], name: str, context: str) -> float:
    value = mapping.get(name)
    if not isinstance(value, (int, float)):
        raise SystemExit(f"{context}: missing numeric `{name}`")
    return float(value)


def load_summary(spec: SummarySpec) -> ObjectiveSummary:
    if not spec.path.exists():
        raise SystemExit(f"{spec.label}: missing summary JSON: {spec.path}")
    payload = json.loads(spec.path.read_text(encoding="utf-8"))
    aggregates = payload.get("aggregates")
    folds = payload.get("folds")
    if not isinstance(aggregates, dict) or not isinstance(folds, list):
        raise SystemExit(f"{spec.path}: expected a selector LOO summary")
    fold_count = int(required_number(aggregates, "fold_count", spec.label))
    return ObjectiveSummary(
        label=spec.label,
        holdout_net_m=required_number(
            aggregates,
            "holdout_selected_score_delta_distance_m",
            spec.label,
        ),
        selector_vs_candidate_m=required_number(
            aggregates,
            "holdout_selector_vs_candidate_all_delta_m",
            spec.label,
        ),
        precision_pct=required_number(aggregates, "holdout_distance_precision_pct", spec.label),
        nonnegative_runs=int(required_number(aggregates, "nonnegative_holdout_runs", spec.label)),
        fold_count=fold_count,
        min_holdout_m=required_number(aggregates, "min_holdout_delta_m", spec.label),
        holdout_loss_m=required_number(
            aggregates,
            "holdout_selected_loss_distance_m",
            spec.label,
        ),
        folds=tuple(dict(fold) for fold in folds if isinstance(fold, dict)),
    )


def fmt_m(value: float) -> str:
    return f"{value:+.1f} m"


def fmt_pct(value: float) -> str:
    return f"{value:.1f}%"


def run_label(key: object) -> str:
    text = str(key)
    city, _, run_name = text.partition("_")
    return f"{city.capitalize()} {run_name.replace('run', 'r')}" if run_name else text


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
    value_color: str,
) -> None:
    add_panel(ax, x, y, w, h)
    ax.text(x + 0.052 * w, y + h - 0.26 * h, title, fontsize=10.8, color=MUTED, weight="bold")
    ax.text(x + 0.052 * w, y + 0.40 * h, value, fontsize=23.5, color=value_color, weight="bold")
    ax.text(x + 0.052 * w, y + 0.17 * h, detail, fontsize=9.8, color=MUTED)


def draw_objective_bars(ax, summaries: list[ObjectiveSummary]) -> None:
    import numpy as np

    labels = [summary.label for summary in summaries]
    values = [summary.holdout_net_m for summary in summaries]
    positions = np.arange(len(summaries))
    colors = [
        BASELINE if index == 0 else ROBUST if index == 1 else CONSTRAINED
        for index in range(len(summaries))
    ]

    ax.bar(positions, values, color=colors, width=0.62)
    ax.axhline(0.0, color=GRID, linewidth=1.2)
    ax.set_xticks(positions)
    ax.set_xticklabels(labels, color=TEXT, fontsize=9.1, rotation=8, ha="right")
    ax.set_ylabel("LOO holdout net official distance (m)", color=MUTED, fontsize=9.5)
    ax.tick_params(axis="y", labelsize=8.8, colors=MUTED)
    ax.grid(axis="y", alpha=0.22, color=GRID)
    ax.set_title("Objective and constraint sweep", color=TEXT, fontsize=14.5, weight="bold", pad=10)
    for spine in ax.spines.values():
        spine.set_alpha(0.15)
    ymax = max(max(values), 1.0)
    ymin = min(min(values), 0.0)
    ax.set_ylim(ymin - 0.12 * ymax, ymax * 1.20)
    for position, value in zip(positions, values):
        ax.text(
            position,
            value + ymax * 0.035,
            fmt_m(value),
            ha="center",
            va="bottom",
            fontsize=8.8,
            color=WIN if value >= 0.0 else LOSS,
            weight="bold",
        )


def draw_fold_table(ax, summary: ObjectiveSummary) -> None:
    ax.set_axis_off()
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)
    ax.text(
        0.0,
        0.965,
        "Best constrained LOO folds",
        fontsize=14.5,
        color=TEXT,
        weight="bold",
        va="top",
    )
    ax.text(
        0.0,
        0.895,
        "Holdout deltas use the PPC official distance metric.",
        fontsize=9.4,
        color=MUTED,
        va="top",
    )

    headers = ["Holdout", "Net", "Loss", "Selected"]
    xs = [0.02, 0.55, 0.76, 0.98]
    aligns = ["left", "right", "right", "right"]
    for x, header, align in zip(xs, headers, aligns):
        ax.text(
            x,
            0.78,
            header,
            fontsize=9.2,
            color=MUTED,
            weight="bold",
            ha=align,
            va="center",
        )
    ax.plot([0.0, 1.0], [0.735, 0.735], color=EDGE, linewidth=1.0)

    y = 0.655
    for fold in summary.folds:
        net = required_number(fold, "holdout_selected_score_delta_distance_m", summary.label)
        loss = required_number(fold, "holdout_selected_loss_distance_m", summary.label)
        selected = int(required_number(fold, "holdout_selected_segments", summary.label))
        color = WIN if net >= 0.0 else LOSS
        ax.text(
            xs[0],
            y,
            run_label(fold.get("holdout_run")),
            fontsize=9.8,
            color=TEXT,
            weight="bold",
            ha="left",
            va="center",
        )
        ax.text(
            xs[1],
            y,
            fmt_m(net),
            fontsize=9.6,
            color=color,
            weight="bold",
            ha="right",
            va="center",
        )
        ax.text(
            xs[2],
            y,
            fmt_m(loss),
            fontsize=9.6,
            color=LOSS if loss < 0.0 else TEXT,
            weight="bold",
            ha="right",
            va="center",
        )
        ax.text(
            xs[3],
            y,
            str(selected),
            fontsize=9.6,
            color=TEXT,
            weight="bold",
            ha="right",
            va="center",
        )
        y -= 0.095


def main() -> int:
    parser = argparse.ArgumentParser(prog=os.environ.get("GNSS_CLI_NAME"))
    parser.add_argument(
        "--summary",
        action="append",
        default=[],
        metavar="LABEL=JSON",
        help="Selector LOO summary JSON. Repeat for each objective to compare.",
    )
    parser.add_argument(
        "--best-label",
        default=None,
        help="Label from --summary to use for the fold table. Defaults to the last summary.",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("docs/ppc_jump0p5_selector_validation_scorecard.png"),
    )
    args = parser.parse_args()

    if not args.summary:
        raise SystemExit("At least one --summary LABEL=JSON is required")
    summaries = [load_summary(parse_summary_spec(value)) for value in args.summary]
    best = summaries[-1]
    if args.best_label is not None:
        matches = [summary for summary in summaries if summary.label == args.best_label]
        if not matches:
            raise SystemExit(f"--best-label did not match any summary: {args.best_label}")
        best = matches[0]

    import matplotlib.pyplot as plt

    fig = plt.figure(figsize=(14, 7.6), dpi=100, facecolor=BG)
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_axis_off()
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)

    ax.text(0.05, 0.925, "PPC jump0.5 segment selector", fontsize=21, color=MUTED, weight="bold")
    ax.text(0.05, 0.845, "Run-level robustness validation", fontsize=36, color=TEXT, weight="bold")
    ax.text(
        0.05,
        0.785,
        "Public Tokyo/Nagoya segment deltas; no IMU; leave-one-run-out uses only training-run rules.",
        fontsize=13.3,
        color=MUTED,
    )

    draw_summary_card(
        ax,
        0.05,
        0.635,
        0.205,
        0.115,
        "Best holdout net",
        fmt_m(best.holdout_net_m),
        f"{fmt_m(best.selector_vs_candidate_m)} vs candidate-all",
        value_color=WIN if best.holdout_net_m >= 0.0 else LOSS,
    )
    draw_summary_card(
        ax,
        0.285,
        0.635,
        0.205,
        0.115,
        "Non-negative runs",
        f"{best.nonnegative_runs}/{best.fold_count}",
        f"min holdout {fmt_m(best.min_holdout_m)}",
        value_color=WIN if best.nonnegative_runs == best.fold_count else LOSS,
    )
    draw_summary_card(
        ax,
        0.52,
        0.635,
        0.205,
        0.115,
        "Holdout precision",
        fmt_pct(best.precision_pct),
        f"loss exposure {fmt_m(best.holdout_loss_m)}",
        value_color=WIN,
    )
    draw_summary_card(
        ax,
        0.755,
        0.635,
        0.195,
        0.115,
        "Best constraint",
        "F->F",
        "baseline + residual RMS",
        value_color=CONSTRAINED,
    )

    bars_ax = fig.add_axes([0.065, 0.135, 0.46, 0.40], facecolor=PANEL)
    table_ax = fig.add_axes([0.575, 0.125, 0.36, 0.42], facecolor=BG)
    draw_objective_bars(bars_ax, summaries)
    draw_fold_table(table_ax, best)

    ax.text(
        0.05,
        0.055,
        "Score is selected candidate official-score distance minus reset10 baseline. Positive is better.",
        fontsize=10.2,
        color=MUTED,
    )

    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output, dpi=100, facecolor=fig.get_facecolor())
    plt.close(fig)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
