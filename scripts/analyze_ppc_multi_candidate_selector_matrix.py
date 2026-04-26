#!/usr/bin/env python3
"""Aggregate PPC multi-candidate selector matrix JSON into markdown and scorecard PNG.

Reads the ``--summary-json`` output produced by
``run_ppc_multi_candidate_selector_matrix.py`` (PR B) and emits:

- A per-run + total markdown table (``--markdown-output``, stdout if omitted).
- A scorecard PNG (``--scorecard``) using the same dark-card style as
  ``analyze_ppc_dual_profile_selector_matrix.py``.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
from pathlib import Path


# ---------------------------------------------------------------------------
# Color palette (mirror of analyze_ppc_dual_profile_selector_matrix.py)
# ---------------------------------------------------------------------------

BG = "#f7f8fb"
PANEL = "#ffffff"
TEXT = "#172033"
MUTED = "#667085"
GRID = "#d0d5dd"
BASELINE = "#d97706"
SELECTOR = "#0f766e"
WIN = "#18794e"
LOSS = "#b42318"
EDGE = "#d0d5dd"
CAND_COLORS = [
    "#64748b",
    "#3b82f6",
    "#8b5cf6",
    "#ec4899",
    "#f59e0b",
    "#10b981",
    "#ef4444",
    "#06b6d4",
]

# Maximum number of candidates to show in the bar chart
MAX_BAR_CANDIDATES = 8


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        prog=os.environ.get("GNSS_CLI_NAME"),
        description=(
            "Analyze PPC multi-candidate selector matrix JSON and produce "
            "markdown table + scorecard PNG."
        ),
    )
    parser.add_argument(
        "--summary-json",
        type=Path,
        default=Path("output/ppc_multi_selector_matrix.json"),
        help="Matrix-level summary JSON from run_ppc_multi_candidate_selector_matrix.py.",
    )
    parser.add_argument(
        "--markdown-output",
        type=Path,
        default=None,
        help="Path for markdown table output (stdout if omitted).",
    )
    parser.add_argument(
        "--scorecard",
        type=Path,
        default=None,
        help="Path for scorecard PNG output.",
    )
    parser.add_argument(
        "--title",
        default=None,
        help="Override title (defaults to value from JSON).",
    )
    return parser.parse_args()


# ---------------------------------------------------------------------------
# JSON loading
# ---------------------------------------------------------------------------


def load_payload(path: Path) -> dict[str, object]:
    if not path.exists():
        raise SystemExit(f"summary JSON not found: {path}")
    raw = path.read_text(encoding="utf-8")
    payload = json.loads(raw)
    if not isinstance(payload, dict):
        raise SystemExit(f"{path}: expected a JSON object")
    return payload


# ---------------------------------------------------------------------------
# Helper functions (copied locally; no shared module dependency)
# ---------------------------------------------------------------------------


def required_dict(
    payload: dict[str, object], name: str, context: str
) -> dict[str, object]:
    value = payload.get(name)
    if not isinstance(value, dict):
        raise SystemExit(f"{context}: missing `{name}` object")
    return value


def required_number(
    mapping: dict[str, object], name: str, context: str
) -> float:
    value = mapping.get(name)
    if not isinstance(value, (int, float)):
        raise SystemExit(f"{context}: missing numeric `{name}`")
    return float(value)


def optional_number(mapping: dict[str, object], name: str) -> float:
    value = mapping.get(name)
    return float(value) if isinstance(value, (int, float)) else 0.0


def fmt_pp(value: float) -> str:
    return f"{value:+.2f} pp"


def fmt_m(value: float) -> str:
    return f"{value:+.3f} m"


def run_label(key: str) -> str:
    city, _, run_name = key.partition("_")
    return (
        f"{city.capitalize()} {run_name.replace('run', 'r')}" if run_name else key
    )


# ---------------------------------------------------------------------------
# Candidate ordering
# ---------------------------------------------------------------------------


def ordered_candidates(
    payload: dict[str, object],
    run_entries: list[dict[str, object]],
) -> list[str]:
    """Return stable candidate list: priority_order from first run, then lexicographic."""
    # Use priority_order from the first run entry if available
    for run_entry in run_entries:
        if isinstance(run_entry, dict):
            priority = run_entry.get("priority_order")
            if isinstance(priority, list) and priority:
                seen: list[str] = list(priority)
                # Append any extras (lexicographic) not in priority_order
                all_labels: list[str] = list(payload.get("candidates", []))
                for label in sorted(all_labels):
                    if label not in seen:
                        seen.append(label)
                return seen
    # Fall back: all candidates from payload sorted lexicographically
    all_cands = payload.get("candidates", [])
    return sorted(all_cands) if isinstance(all_cands, list) else []


# ---------------------------------------------------------------------------
# Markdown rendering
# ---------------------------------------------------------------------------


def render_markdown(payload: dict[str, object], title_override: str | None) -> str:
    title = title_override or str(
        payload.get("title", "PPC multi-candidate selector")
    )
    aggregates = required_dict(payload, "aggregates", "payload")
    runs = payload.get("runs", [])
    if not isinstance(runs, list):
        raise SystemExit("payload: missing runs list")

    baseline_weighted = required_number(
        aggregates, "weighted_baseline_official_score_pct", "aggregates"
    )
    selector_weighted = required_number(
        aggregates, "weighted_selector_official_score_pct", "aggregates"
    )
    delta_m = optional_number(aggregates, "selector_official_score_delta_m")
    delta_pp = optional_number(aggregates, "selector_official_score_delta_pct")
    min_delta_m = optional_number(aggregates, "min_official_score_delta_m")
    max_delta_m = optional_number(aggregates, "max_official_score_delta_m")
    total_cand_segs = int(
        optional_number(aggregates, "total_candidate_selected_segments")
    )
    dropped = aggregates.get("dropped_candidates_any_run", [])

    lines = [
        f"# {title}",
        "",
        "## Aggregate",
        "",
        "| metric | value |",
        "|---|---:|",
        f"| baseline weighted official | {baseline_weighted:.6f}% |",
        f"| multi-candidate selector weighted official | {selector_weighted:.6f}% |",
        f"| selector vs baseline | {fmt_m(delta_m)} / {fmt_pp(delta_pp)} |",
        f"| per-run delta min / max | {fmt_m(min_delta_m)} / {fmt_m(max_delta_m)} |",
        f"| total candidate-selected segments | {total_cand_segs} |",
    ]
    if isinstance(dropped, list) and dropped:
        lines.append(
            f"| dropped candidates (any run) | {', '.join(str(d) for d in dropped)} |"
        )
    lines += [
        "",
        "## Runs",
        "",
        "| run | baseline % | selector % | delta m | delta pp |",
        "|---|---:|---:|---:|---:|",
    ]

    for run_entry in sorted(runs, key=lambda r: str(r.get("key", "")) if isinstance(r, dict) else ""):
        if not isinstance(run_entry, dict):
            continue
        key = str(run_entry.get("key", "?"))
        bl = run_entry.get("baseline", {})
        sel = run_entry.get("selector", {})
        dlt = run_entry.get("delta_vs_baseline", {})
        if not isinstance(bl, dict):
            bl = {}
        if not isinstance(sel, dict):
            sel = {}
        if not isinstance(dlt, dict):
            dlt = {}
        bl_pct = optional_number(bl, "ppc_official_score_pct")
        sel_pct = optional_number(sel, "ppc_official_score_pct")
        dlt_m = optional_number(dlt, "ppc_official_score_distance_m")
        dlt_pp = optional_number(dlt, "ppc_official_score_pct")
        lines.append(
            f"| {run_label(key)} | {bl_pct:.6f}% | {sel_pct:.6f}% |"
            f" {dlt_m:+.3f} | {dlt_pp:+.3f} pp |"
        )

    lines.append("")
    return "\n".join(lines)


# ---------------------------------------------------------------------------
# Matplotlib helpers (copied from analyze_ppc_dual_profile_selector_matrix.py)
# ---------------------------------------------------------------------------


def add_panel(
    ax, x: float, y: float, w: float, h: float, radius: float = 0.018
) -> None:
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
    ax.text(
        x + 0.052 * w, y + h - 0.25 * h, title, fontsize=10.5, color=MUTED, weight="bold"
    )
    ax.text(
        x + 0.052 * w, y + 0.39 * h, value, fontsize=22.5, color=value_color, weight="bold"
    )
    ax.text(x + 0.052 * w, y + 0.16 * h, detail, fontsize=9.7, color=MUTED)


# ---------------------------------------------------------------------------
# Scorecard sub-plots
# ---------------------------------------------------------------------------


def draw_official_bars(
    ax,
    runs: list[dict[str, object]],
) -> None:
    import numpy as np

    labels = [run_label(str(r.get("key", ""))) for r in runs]
    baseline = [
        optional_number(r.get("baseline", {}), "ppc_official_score_pct")  # type: ignore[arg-type]
        for r in runs
    ]
    selector = [
        optional_number(r.get("selector", {}), "ppc_official_score_pct")  # type: ignore[arg-type]
        for r in runs
    ]
    positions = np.arange(len(labels))
    width = 0.30

    ax.bar(positions - width / 2, baseline, width, color=BASELINE, label="baseline")
    ax.bar(positions + width / 2, selector, width, color=SELECTOR, label="selector")
    ymax = max(max(baseline, default=1.0), max(selector, default=1.0), 1.0) * 1.22
    ax.set_ylim(0, ymax)
    ax.set_xticks(positions)
    ax.set_xticklabels(labels, color=TEXT, fontsize=9.2)
    ax.set_ylabel("PPC official score (%)", color=MUTED, fontsize=9.5)
    ax.tick_params(axis="y", labelsize=8.5, colors=MUTED)
    ax.grid(axis="y", alpha=0.22, color=GRID)
    ax.legend(loc="upper left", frameon=False, fontsize=9.4, ncol=2)
    ax.set_title(
        "Official score by run", color=TEXT, fontsize=14.5, weight="bold", pad=10
    )
    for spine in ax.spines.values():
        spine.set_alpha(0.15)

    for index, run_entry in enumerate(runs):
        delta_dict = run_entry.get("delta_vs_baseline", {})
        if not isinstance(delta_dict, dict):
            delta_dict = {}
        delta = optional_number(delta_dict, "ppc_official_score_distance_m")
        color = WIN if delta >= 0.0 else LOSS
        ax.text(
            positions[index] + width / 2,
            selector[index] + ymax * 0.028,
            f"{delta:+.0f} m",
            color=color,
            fontsize=8.2,
            ha="center",
            weight="bold",
        )


def draw_candidate_distribution(
    ax,
    runs: list[dict[str, object]],
    candidate_order: list[str],
) -> None:
    """Stacked bar showing candidate-selected segments per run."""
    import numpy as np

    top_candidates = candidate_order[:MAX_BAR_CANDIDATES]
    labels = [run_label(str(r.get("key", ""))) for r in runs]
    positions = np.arange(len(labels))
    bar_width = 0.55

    bottoms = np.zeros(len(runs), dtype=float)
    for ci, label in enumerate(top_candidates):
        counts = np.array(
            [
                float(
                    (
                        r.get("per_candidate", {}).get(label, {}).get(  # type: ignore[union-attr]
                            "selected_segments", 0
                        )
                        if isinstance(r.get("per_candidate"), dict)
                        else 0
                    )
                )
                for r in runs
            ]
        )
        color = CAND_COLORS[ci % len(CAND_COLORS)]
        ax.bar(
            positions,
            counts,
            bar_width,
            bottom=bottoms,
            color=color,
            label=label,
        )
        bottoms += counts

    # Baseline-selected bar on top with hatching
    bl_segs = np.array(
        [
            float(
                r.get("selection", {}).get("baseline_selected_segments", 0)  # type: ignore[union-attr]
                if isinstance(r.get("selection"), dict)
                else 0
            )
            for r in runs
        ]
    )
    ax.bar(
        positions,
        bl_segs,
        bar_width,
        bottom=bottoms,
        color=BASELINE,
        alpha=0.55,
        hatch="//",
        label="baseline",
    )

    ax.set_xticks(positions)
    ax.set_xticklabels(labels, color=TEXT, fontsize=9.0)
    ax.set_ylabel("Segments selected", color=MUTED, fontsize=9.5)
    ax.tick_params(axis="y", labelsize=8.5, colors=MUTED)
    ax.grid(axis="y", alpha=0.22, color=GRID)
    ax.legend(
        loc="upper right",
        frameon=False,
        fontsize=8.2,
        ncol=min(3, len(top_candidates) + 1),
    )
    ax.set_title(
        "Candidate distribution by run",
        color=TEXT,
        fontsize=14.5,
        weight="bold",
        pad=10,
    )
    for spine in ax.spines.values():
        spine.set_alpha(0.15)


def draw_run_table(
    ax,
    runs: list[dict[str, object]],
    candidate_order: list[str],
) -> None:
    ax.set_axis_off()
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)
    ax.text(
        0.0,
        0.965,
        "Selector deltas vs baseline",
        fontsize=14.5,
        color=TEXT,
        weight="bold",
        va="top",
    )
    ax.text(
        0.0,
        0.895,
        "Segment-local multi-candidate selection; positive deltas are better.",
        fontsize=9.2,
        color=MUTED,
        va="top",
    )

    headers = ["Run", "Score", "Cand segs"]
    xs = [0.02, 0.52, 0.99]
    aligns = ["left", "right", "right"]
    header_y = 0.78
    for x, header, align in zip(xs, headers, aligns):
        ax.text(
            x,
            header_y,
            header,
            fontsize=9.0,
            color=MUTED,
            weight="bold",
            ha=align,
            va="center",
        )
    ax.plot([0.0, 1.0], [0.735, 0.735], color=EDGE, linewidth=1.0)

    y = 0.655
    for run_entry in runs:
        if not isinstance(run_entry, dict):
            continue
        key = str(run_entry.get("key", ""))
        delta_dict = run_entry.get("delta_vs_baseline", {})
        if not isinstance(delta_dict, dict):
            delta_dict = {}
        selection = run_entry.get("selection", {})
        if not isinstance(selection, dict):
            selection = {}
        score_delta = optional_number(delta_dict, "ppc_official_score_distance_m")
        cand_segs = int(optional_number(selection, "candidate_selected_segments"))
        score_color = WIN if score_delta >= 0.0 else LOSS
        ax.text(
            xs[0],
            y,
            run_label(key),
            fontsize=9.8,
            color=TEXT,
            weight="bold",
            ha="left",
            va="center",
        )
        ax.text(
            xs[1],
            y,
            f"{score_delta:+.1f} m",
            fontsize=9.6,
            color=score_color,
            weight="bold",
            ha="right",
            va="center",
        )
        ax.text(
            xs[2],
            y,
            str(cand_segs),
            fontsize=9.6,
            color=TEXT,
            weight="bold",
            ha="right",
            va="center",
        )
        y -= 0.095

    # Footer: dropped candidates
    dropped_key = "dropped_candidates"
    dropped: list[str] = []
    for run_entry in runs:
        if not isinstance(run_entry, dict):
            continue
        for label in run_entry.get(dropped_key, []):
            if label not in dropped:
                dropped.append(str(label))
    if dropped:
        footer = "Dropped: " + ", ".join(sorted(dropped))
        ax.text(
            0.02,
            0.04,
            footer,
            fontsize=8.4,
            color=MUTED,
            ha="left",
            va="bottom",
        )
    if len(candidate_order) > MAX_BAR_CANDIDATES:
        extra = len(candidate_order) - MAX_BAR_CANDIDATES
        ax.text(
            0.02,
            0.12,
            f"(+{extra} more candidates not shown in bars)",
            fontsize=8.4,
            color=MUTED,
            ha="left",
            va="bottom",
        )


# ---------------------------------------------------------------------------
# PNG renderer
# ---------------------------------------------------------------------------


def render_png(
    payload: dict[str, object],
    output: Path,
    title_override: str | None = None,
) -> None:
    import matplotlib.pyplot as plt

    title = title_override or str(
        payload.get("title", "PPC multi-candidate selector")
    )
    aggregates = required_dict(payload, "aggregates", "payload")
    raw_runs = payload.get("runs")
    if not isinstance(raw_runs, list):
        raise SystemExit("payload: missing runs")
    runs = [r for r in raw_runs if isinstance(r, dict)]
    runs_sorted = sorted(runs, key=lambda r: str(r.get("key", "")))

    candidate_order = ordered_candidates(payload, runs_sorted)

    fig = plt.figure(figsize=(14, 7.8), dpi=100, facecolor=BG)
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_axis_off()
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)

    # Header
    ax.text(0.05, 0.925, "PPC multi-candidate selector", fontsize=21, color=MUTED, weight="bold")
    ax.text(0.05, 0.845, title, fontsize=28, color=TEXT, weight="bold")
    ax.text(
        0.05,
        0.789,
        "Same GNSS-only PPC Tokyo/Nagoya public observations. "
        "Selection is segment-local across all candidates.",
        fontsize=12.8,
        color=MUTED,
    )

    # Summary cards
    baseline_w = optional_number(aggregates, "weighted_baseline_official_score_pct")
    selector_w = optional_number(aggregates, "weighted_selector_official_score_pct")
    delta_m = optional_number(aggregates, "selector_official_score_delta_m")
    delta_pp = optional_number(aggregates, "selector_official_score_delta_pct")
    cand_segs = int(optional_number(aggregates, "total_candidate_selected_segments"))
    n_cands = len(candidate_order)

    draw_summary_card(
        ax,
        0.05,
        0.625,
        0.205,
        0.105,
        "Weighted official score",
        f"{selector_w:.2f}%",
        f"baseline {baseline_w:.2f}%",
        value_color=WIN if selector_w >= baseline_w else LOSS,
    )
    draw_summary_card(
        ax,
        0.282,
        0.625,
        0.205,
        0.105,
        "Official gain",
        f"{delta_m:+.1f} m",
        f"{delta_pp:+.2f} pp vs baseline",
        value_color=WIN if delta_m >= 0.0 else LOSS,
    )
    draw_summary_card(
        ax,
        0.514,
        0.625,
        0.205,
        0.105,
        "Cand-selected segments",
        str(cand_segs),
        f"across {len(runs_sorted)} runs",
        value_color=TEXT,
    )
    draw_summary_card(
        ax,
        0.746,
        0.625,
        0.205,
        0.105,
        "Active candidates",
        str(n_cands),
        f"top {min(n_cands, MAX_BAR_CANDIDATES)} shown in chart",
        value_color=TEXT,
    )

    # Official score bar chart (bottom-left)
    bar_ax = fig.add_axes([0.065, 0.105, 0.38, 0.43], facecolor=PANEL)
    draw_official_bars(bar_ax, runs_sorted)

    # Candidate distribution stacked bar (bottom-center)
    dist_ax = fig.add_axes([0.49, 0.105, 0.20, 0.43], facecolor=PANEL)
    draw_candidate_distribution(dist_ax, runs_sorted, candidate_order)

    # Run table (bottom-right)
    table_ax = fig.add_axes([0.73, 0.105, 0.225, 0.43], facecolor=BG)
    draw_run_table(table_ax, runs_sorted, candidate_order)

    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=100, facecolor=BG)
    plt.close(fig)


# ---------------------------------------------------------------------------
# main
# ---------------------------------------------------------------------------


def main() -> int:
    args = parse_args()
    payload = load_payload(args.summary_json)

    title_override: str | None = args.title

    md = render_markdown(payload, title_override)

    if args.markdown_output is not None:
        args.markdown_output.parent.mkdir(parents=True, exist_ok=True)
        args.markdown_output.write_text(md, encoding="utf-8")
    else:
        sys.stdout.write(md)
        if not md.endswith("\n"):
            sys.stdout.write("\n")

    if args.scorecard is not None:
        render_png(payload, args.scorecard, title_override)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
