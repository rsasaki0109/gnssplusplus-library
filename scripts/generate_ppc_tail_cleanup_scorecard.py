#!/usr/bin/env python3
"""Generate a PPC RTK tail-cleanup diagnostic scorecard."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
import os
from pathlib import Path


BG = "#f6f7f2"
PANEL = "#ffffff"
TEXT = "#172033"
MUTED = "#667085"
GRID = "#d0d5dd"
BASELINE = "#d97706"
CLEANUP = "#0f766e"
WIN = "#18794e"
LOSS = "#b42318"
EDGE = "#d0d5dd"


@dataclass(frozen=True)
class ProfileRun:
    key: str
    label: str
    positioning_pct: float
    fix_pct: float
    official_score_pct: float
    p95_h_m: float
    max_h_m: float
    p95_h_delta_vs_rtklib_m: float
    nonfix_rejected_epochs: int
    fixed_burst_rejected_epochs: int


@dataclass(frozen=True)
class CleanupRun:
    key: str
    label: str
    baseline: ProfileRun
    cleanup: ProfileRun

    @property
    def positioning_delta_pp(self) -> float:
        return self.cleanup.positioning_pct - self.baseline.positioning_pct

    @property
    def fix_delta_pp(self) -> float:
        return self.cleanup.fix_pct - self.baseline.fix_pct

    @property
    def official_delta_pp(self) -> float:
        return self.cleanup.official_score_pct - self.baseline.official_score_pct

    @property
    def p95_improvement_m(self) -> float:
        return self.baseline.p95_h_m - self.cleanup.p95_h_m

    @property
    def max_h_improvement_m(self) -> float:
        return self.baseline.max_h_m - self.cleanup.max_h_m


def average(values: list[float]) -> float:
    return sum(values) / len(values) if values else 0.0


def neutralized(value: float, epsilon: float) -> float:
    return 0.0 if abs(value) < epsilon else value


def tradeoff_color(value: float, *, positive_is_good: bool, epsilon: float) -> str:
    value = neutralized(value, epsilon)
    if value == 0.0:
        return TEXT
    good = value > 0.0 if positive_is_good else value < 0.0
    return WIN if good else LOSS


def fmt_pp(value: float) -> str:
    value = neutralized(value, 0.005)
    return "0.00 pp" if value == 0.0 else f"{value:+.2f} pp"


def fmt_pp_short(value: float) -> str:
    value = neutralized(value, 0.05)
    return "0.0 pp" if value == 0.0 else f"{value:+.1f} pp"


def fmt_m(value: float) -> str:
    value = neutralized(value, 0.005)
    return "0.00 m" if value == 0.0 else f"{value:+.2f} m"


def fmt_m_short(value: float) -> str:
    value = neutralized(value, 0.05)
    return "0.0 m" if value == 0.0 else f"{value:+.1f} m"


def run_label(key: str) -> str:
    city, _, run_name = key.partition("_")
    return f"{city.capitalize()} {run_name.replace('run', 'r')}" if run_name else key


def required_number(mapping: dict[str, object], name: str, context: str) -> float:
    value = mapping.get(name)
    if not isinstance(value, (int, float)):
        raise SystemExit(f"Missing numeric `{name}` in {context}")
    return float(value)


def rejected_epochs(guards: object, name: str) -> int:
    if not isinstance(guards, dict):
        return 0
    guard = guards.get(name)
    if not isinstance(guard, dict):
        return 0
    value = guard.get("rejected_epochs")
    return int(value) if isinstance(value, int) else 0


def profile_runs_from_summary(path: Path) -> dict[str, ProfileRun]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    raw_runs = payload.get("runs")
    if not isinstance(raw_runs, list):
        raise SystemExit(f"{path} does not look like a ppc-coverage-matrix summary")

    runs: dict[str, ProfileRun] = {}
    for item in raw_runs:
        if not isinstance(item, dict):
            continue
        key = str(item.get("key", ""))
        metrics = item.get("metrics")
        delta = item.get("delta_vs_rtklib")
        if not key or not isinstance(metrics, dict) or not isinstance(delta, dict):
            raise SystemExit(f"{path} contains a run without key, metrics, or delta_vs_rtklib")
        runs[key] = ProfileRun(
            key=key,
            label=run_label(key),
            positioning_pct=required_number(metrics, "positioning_rate_pct", key),
            fix_pct=required_number(metrics, "fix_rate_pct", key),
            official_score_pct=required_number(metrics, "ppc_official_score_pct", key),
            p95_h_m=required_number(metrics, "p95_h_m", key),
            max_h_m=required_number(metrics, "max_h_m", key),
            p95_h_delta_vs_rtklib_m=required_number(delta, "p95_h_m", key),
            nonfix_rejected_epochs=rejected_epochs(item.get("guards"), "nonfix_drift_guard"),
            fixed_burst_rejected_epochs=rejected_epochs(item.get("guards"), "fixed_bridge_burst_guard"),
        )
    if not runs:
        raise SystemExit(f"{path} did not contain any runs")
    return runs


def load_comparison_runs(baseline_summary: Path, cleanup_summary: Path) -> list[CleanupRun]:
    baseline = profile_runs_from_summary(baseline_summary)
    cleanup = profile_runs_from_summary(cleanup_summary)
    missing = sorted(set(baseline) ^ set(cleanup))
    if missing:
        raise SystemExit("Baseline and cleanup summaries have different run keys: " + ", ".join(missing))
    return [
        CleanupRun(key=key, label=baseline[key].label, baseline=baseline[key], cleanup=cleanup[key])
        for key in baseline
    ]


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
    ax.text(x + 0.052 * w, y + h - 0.25 * h, title, fontsize=10.5, color=MUTED, weight="bold")
    ax.text(x + 0.052 * w, y + 0.39 * h, value, fontsize=22.5, color=value_color, weight="bold")
    ax.text(x + 0.052 * w, y + 0.16 * h, detail, fontsize=9.8, color=MUTED)


def draw_p95_bars(ax, runs: list[CleanupRun]) -> None:
    import numpy as np

    labels = [run.label for run in runs]
    baseline = [run.baseline.p95_h_m for run in runs]
    cleanup = [run.cleanup.p95_h_m for run in runs]
    positions = np.arange(len(runs))
    width = 0.34

    ax.bar(positions - width / 2, baseline, width, color=BASELINE, label="coverage profile")
    ax.bar(positions + width / 2, cleanup, width, color=CLEANUP, label="tail cleanup")
    ax.set_xticks(positions)
    ax.set_xticklabels(labels, color=TEXT, fontsize=9.2)
    ax.set_ylabel("P95 horizontal error (m)", color=MUTED, fontsize=9.5)
    ax.tick_params(axis="y", labelsize=8.5, colors=MUTED)
    ax.grid(axis="y", alpha=0.22, color=GRID)
    ax.legend(loc="upper left", frameon=False, fontsize=9.5, ncol=2)
    ax.set_title("P95 H tail cleanup effect", color=TEXT, fontsize=14.5, weight="bold", pad=10)
    for spine in ax.spines.values():
        spine.set_alpha(0.15)

    ymax = max(max(baseline), max(cleanup), 1.0)
    ax.set_ylim(0, ymax * 1.22)
    for index, run in enumerate(runs):
        p95_gain = neutralized(run.p95_improvement_m, 0.05)
        color = tradeoff_color(p95_gain, positive_is_good=True, epsilon=0.05)
        ax.text(
            positions[index],
            max(run.baseline.p95_h_m, run.cleanup.p95_h_m) + ymax * 0.035,
            fmt_m_short(p95_gain),
            color=color,
            fontsize=8.4,
            ha="center",
            weight="bold",
        )


def draw_tradeoff_table(ax, runs: list[CleanupRun]) -> None:
    ax.set_axis_off()
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)
    ax.text(0.0, 0.965, "Cleanup tradeoff by run", fontsize=14.5, color=TEXT, weight="bold", va="top")
    ax.text(0.0, 0.895, "Delta is cleanup minus coverage; positive P95 improvement is better.", fontsize=9.2, color=MUTED, va="top")

    headers = ["Run", "Pos", "Official", "P95 gain", "Rejected"]
    xs = [0.02, 0.38, 0.58, 0.78, 0.99]
    aligns = ["left", "right", "right", "right", "right"]
    header_y = 0.78
    for x, header, align in zip(xs, headers, aligns):
        ax.text(x, header_y, header, fontsize=9.0, color=MUTED, weight="bold", ha=align, va="center")
    ax.plot([0.0, 1.0], [0.735, 0.735], color=EDGE, linewidth=1.0)

    y = 0.655
    for run in runs:
        rejected = run.cleanup.nonfix_rejected_epochs + run.cleanup.fixed_burst_rejected_epochs
        pos_color = tradeoff_color(run.positioning_delta_pp, positive_is_good=True, epsilon=0.05)
        official_color = tradeoff_color(run.official_delta_pp, positive_is_good=True, epsilon=0.05)
        p95_color = tradeoff_color(run.p95_improvement_m, positive_is_good=True, epsilon=0.05)
        ax.text(xs[0], y, run.label, fontsize=9.8, color=TEXT, weight="bold", ha="left", va="center")
        ax.text(xs[1], y, fmt_pp_short(run.positioning_delta_pp), fontsize=9.3, color=pos_color, weight="bold", ha="right", va="center")
        ax.text(xs[2], y, fmt_pp_short(run.official_delta_pp), fontsize=9.3, color=official_color, weight="bold", ha="right", va="center")
        ax.text(xs[3], y, fmt_m_short(run.p95_improvement_m), fontsize=9.3, color=p95_color, weight="bold", ha="right", va="center")
        ax.text(xs[4], y, str(rejected), fontsize=9.3, color=TEXT, weight="bold", ha="right", va="center")
        y -= 0.095


def footer_text(cleanup_summary: Path) -> str:
    payload = json.loads(cleanup_summary.read_text(encoding="utf-8"))
    ratio = payload.get("ratio")
    residual = payload.get("nonfix_drift_max_residual")
    fixed_residual = payload.get("fixed_bridge_burst_max_residual")
    parts = ["coverage profile plus tail-cleanup diagnostics"]
    if isinstance(ratio, (int, float)):
        parts.append(f"ratio {ratio:g}")
    if isinstance(residual, (int, float)):
        parts.append(f"non-FIX residual {residual:g} m")
    if isinstance(fixed_residual, (int, float)):
        parts.append(f"fixed-burst residual {fixed_residual:g} m")
    parts.append("not the Positioning-rate sign-off profile")
    return "Profile: " + ", ".join(parts) + "."


def main() -> int:
    parser = argparse.ArgumentParser(prog=os.environ.get("GNSS_CLI_NAME"))
    parser.add_argument("--baseline-summary-json", type=Path, required=True)
    parser.add_argument("--cleanup-summary-json", type=Path, required=True)
    parser.add_argument("--output", type=Path, default=Path("docs/ppc_tail_cleanup_scorecard.png"))
    args = parser.parse_args()

    import matplotlib.pyplot as plt

    runs = load_comparison_runs(args.baseline_summary_json, args.cleanup_summary_json)
    improved_runs = sum(1 for run in runs if run.p95_improvement_m > 0.05)
    avg_p95_gain = average([run.p95_improvement_m for run in runs])
    avg_pos_delta = average([run.positioning_delta_pp for run in runs])
    avg_official_delta = average([run.official_delta_pp for run in runs])
    rejected_epochs = sum(
        run.cleanup.nonfix_rejected_epochs + run.cleanup.fixed_burst_rejected_epochs for run in runs
    )

    fig = plt.figure(figsize=(14, 7.6), dpi=100, facecolor=BG)
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_axis_off()
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)

    ax.text(0.05, 0.925, "PPC RTK tail-cleanup diagnostic", fontsize=21, color=MUTED, weight="bold")
    ax.text(0.05, 0.845, "P95 tail vs positioning rate", fontsize=36, color=TEXT, weight="bold")
    ax.text(
        0.05,
        0.785,
        "Same public Tokyo/Nagoya runs; this view measures the cost of pruning suspect fallback/fixed bursts.",
        fontsize=13.2,
        color=MUTED,
    )

    draw_summary_card(
        ax,
        0.05,
        0.625,
        0.205,
        0.105,
        "P95 improved runs",
        f"{improved_runs}/{len(runs)}",
        "cleanup P95H below coverage",
        value_color=WIN if improved_runs == len(runs) else TEXT,
    )
    draw_summary_card(
        ax,
        0.282,
        0.625,
        0.205,
        0.105,
        "Average P95H gain",
        fmt_m(avg_p95_gain),
        "positive means tighter tail",
        value_color=tradeoff_color(avg_p95_gain, positive_is_good=True, epsilon=0.005),
    )
    draw_summary_card(
        ax,
        0.514,
        0.625,
        0.205,
        0.105,
        "Average positioning cost",
        fmt_pp(avg_pos_delta),
        "cleanup minus coverage",
        value_color=tradeoff_color(avg_pos_delta, positive_is_good=True, epsilon=0.005),
    )
    draw_summary_card(
        ax,
        0.746,
        0.625,
        0.205,
        0.105,
        "Average official-score cost",
        fmt_pp(avg_official_delta),
        f"{rejected_epochs} total rejected epochs",
        value_color=tradeoff_color(avg_official_delta, positive_is_good=True, epsilon=0.005),
    )

    bars_ax = fig.add_axes([0.065, 0.115, 0.555, 0.405], facecolor=PANEL)
    draw_p95_bars(bars_ax, runs)

    add_panel(ax, 0.655, 0.105, 0.295, 0.435)
    table_ax = fig.add_axes([0.675, 0.135, 0.255, 0.375], facecolor=PANEL)
    draw_tradeoff_table(table_ax, runs)

    ax.text(0.05, 0.044, footer_text(args.cleanup_summary_json), fontsize=10.1, color=MUTED)

    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output, dpi=100, facecolor=fig.get_facecolor())
    print(f"Saved: {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
