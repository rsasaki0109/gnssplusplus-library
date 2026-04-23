#!/usr/bin/env python3
"""Aggregate PPC dual-profile selector summaries across multiple runs."""

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
BASELINE = "#d97706"
CANDIDATE = "#64748b"
SELECTOR = "#0f766e"
WIN = "#18794e"
LOSS = "#b42318"
EDGE = "#d0d5dd"


@dataclass(frozen=True)
class RunSpec:
    key: str
    path: Path


@dataclass(frozen=True)
class SelectorRun:
    key: str
    label: str
    rule: str
    baseline: dict[str, float]
    candidate: dict[str, float]
    selected: dict[str, float]
    delta_vs_baseline: dict[str, float]
    delta_vs_candidate: dict[str, float]
    selection: dict[str, float]


METRIC_KEYS = (
    "positioning_rate_pct",
    "fix_rate_pct",
    "ppc_official_score_pct",
    "ppc_official_score_distance_m",
    "ppc_official_total_distance_m",
    "p95_h_m",
    "max_h_m",
)
DELTA_KEYS = (
    "positioning_rate_pct",
    "fix_rate_pct",
    "ppc_official_score_pct",
    "ppc_official_score_distance_m",
    "p95_h_m",
)
SELECTION_KEYS = (
    "baseline_selected_segments",
    "candidate_selected_segments",
    "candidate_selected_gain_distance_m",
    "candidate_selected_loss_distance_m",
    "candidate_selected_score_delta_distance_m",
    "segments",
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(prog=os.environ.get("GNSS_CLI_NAME"))
    parser.add_argument(
        "--run",
        action="append",
        default=[],
        metavar="KEY=SUMMARY_JSON",
        help="Dual-profile selector summary JSON. Repeat once per PPC run.",
    )
    parser.add_argument("--summary-json", type=Path, default=None)
    parser.add_argument("--markdown-output", type=Path, default=None)
    parser.add_argument("--output-png", type=Path, default=None)
    parser.add_argument(
        "--title",
        default="PPC dual-profile selector",
        help="Title used in Markdown and PNG outputs.",
    )
    return parser.parse_args()


def parse_run(value: str) -> RunSpec:
    key, sep, path_text = value.partition("=")
    if not sep or not key.strip() or not path_text.strip():
        raise SystemExit("--run must use KEY=SUMMARY_JSON")
    return RunSpec(key=key.strip(), path=Path(path_text.strip()))


def run_label(key: str) -> str:
    city, _, run_name = key.partition("_")
    return f"{city.capitalize()} {run_name.replace('run', 'r')}" if run_name else key


def required_dict(payload: dict[str, object], name: str, context: str) -> dict[str, object]:
    value = payload.get(name)
    if not isinstance(value, dict):
        raise SystemExit(f"{context}: missing `{name}` object")
    return value


def required_number(mapping: dict[str, object], name: str, context: str) -> float:
    value = mapping.get(name)
    if not isinstance(value, (int, float)):
        raise SystemExit(f"{context}: missing numeric `{name}`")
    return float(value)


def optional_number(mapping: dict[str, object], name: str) -> float | None:
    value = mapping.get(name)
    return float(value) if isinstance(value, (int, float)) else None


def metric_subset(mapping: dict[str, object], keys: tuple[str, ...], context: str) -> dict[str, float]:
    return {key: required_number(mapping, key, context) for key in keys}


def load_run(spec: RunSpec) -> SelectorRun:
    if not spec.path.exists():
        raise SystemExit(f"{spec.key}: missing summary JSON: {spec.path}")
    payload = json.loads(spec.path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise SystemExit(f"{spec.path}: expected a JSON object")
    baseline = required_dict(payload, "baseline", spec.key)
    candidate = required_dict(payload, "candidate", spec.key)
    selected = required_dict(payload, "metrics", spec.key)
    delta_vs_baseline = required_dict(payload, "delta_vs_baseline", spec.key)
    delta_vs_candidate = required_dict(payload, "delta_vs_candidate", spec.key)
    selection = required_dict(payload, "selection", spec.key)
    rule = payload.get("rule")
    if not isinstance(rule, str) or not rule.strip():
        raise SystemExit(f"{spec.key}: missing selector rule")
    return SelectorRun(
        key=spec.key,
        label=run_label(spec.key),
        rule=rule,
        baseline=metric_subset(baseline, METRIC_KEYS, f"{spec.key}.baseline"),
        candidate=metric_subset(candidate, METRIC_KEYS, f"{spec.key}.candidate"),
        selected=metric_subset(selected, METRIC_KEYS, f"{spec.key}.metrics"),
        delta_vs_baseline=metric_subset(delta_vs_baseline, DELTA_KEYS, f"{spec.key}.delta_vs_baseline"),
        delta_vs_candidate=metric_subset(delta_vs_candidate, DELTA_KEYS, f"{spec.key}.delta_vs_candidate"),
        selection={
            key: required_number(selection, key, f"{spec.key}.selection")
            for key in SELECTION_KEYS
        },
    )


def load_runs(specs: list[RunSpec]) -> list[SelectorRun]:
    if not specs:
        raise SystemExit("At least one --run is required")
    runs = [load_run(spec) for spec in specs]
    keys = [run.key for run in runs]
    if len(set(keys)) != len(keys):
        raise SystemExit("Duplicate run keys are not allowed")
    return runs


def weighted_official_score(runs: list[SelectorRun], profile: str) -> float:
    score_distance_m = 0.0
    total_distance_m = 0.0
    for run in runs:
        metrics = getattr(run, profile)
        score_distance_m += metrics["ppc_official_score_distance_m"]
        total_distance_m += metrics["ppc_official_total_distance_m"]
    if total_distance_m <= 0.0:
        raise SystemExit("Cannot compute weighted official score without positive total distance")
    return 100.0 * score_distance_m / total_distance_m


def sum_metric(runs: list[SelectorRun], profile: str, key: str) -> float:
    return sum(getattr(run, profile)[key] for run in runs)


def average(values: list[float]) -> float:
    return sum(values) / len(values) if values else 0.0


def rounded(value: float | None) -> float | None:
    return None if value is None else round(float(value), 6)


def aggregate_runs(runs: list[SelectorRun]) -> dict[str, object]:
    baseline_weighted = weighted_official_score(runs, "baseline")
    candidate_weighted = weighted_official_score(runs, "candidate")
    selected_weighted = weighted_official_score(runs, "selected")
    baseline_score_m = sum_metric(runs, "baseline", "ppc_official_score_distance_m")
    candidate_score_m = sum_metric(runs, "candidate", "ppc_official_score_distance_m")
    selected_score_m = sum_metric(runs, "selected", "ppc_official_score_distance_m")
    total_distance_m = sum_metric(runs, "baseline", "ppc_official_total_distance_m")
    return {
        "run_count": len(runs),
        "weighted_baseline_official_score_pct": rounded(baseline_weighted),
        "weighted_candidate_all_official_score_pct": rounded(candidate_weighted),
        "weighted_selector_official_score_pct": rounded(selected_weighted),
        "official_total_distance_m": rounded(total_distance_m),
        "selector_official_score_delta_m": rounded(selected_score_m - baseline_score_m),
        "selector_official_score_delta_pct": rounded(selected_weighted - baseline_weighted),
        "candidate_all_official_score_delta_m": rounded(candidate_score_m - baseline_score_m),
        "candidate_all_official_score_delta_pct": rounded(candidate_weighted - baseline_weighted),
        "selector_vs_candidate_all_official_score_delta_m": rounded(selected_score_m - candidate_score_m),
        "selector_vs_candidate_all_official_score_delta_pct": rounded(selected_weighted - candidate_weighted),
        "avg_positioning_delta_pct": rounded(
            average([run.delta_vs_baseline["positioning_rate_pct"] for run in runs])
        ),
        "avg_fix_delta_pct": rounded(average([run.delta_vs_baseline["fix_rate_pct"] for run in runs])),
        "min_official_score_delta_m": rounded(
            min(run.delta_vs_baseline["ppc_official_score_distance_m"] for run in runs)
        ),
        "max_official_score_delta_m": rounded(
            max(run.delta_vs_baseline["ppc_official_score_distance_m"] for run in runs)
        ),
        "candidate_selected_segments": int(
            sum(optional_number(run.selection, "candidate_selected_segments") or 0.0 for run in runs)
        ),
        "baseline_selected_segments": int(
            sum(optional_number(run.selection, "baseline_selected_segments") or 0.0 for run in runs)
        ),
        "candidate_selected_gain_distance_m": rounded(
            sum(optional_number(run.selection, "candidate_selected_gain_distance_m") or 0.0 for run in runs)
        ),
        "candidate_selected_loss_distance_m": rounded(
            sum(optional_number(run.selection, "candidate_selected_loss_distance_m") or 0.0 for run in runs)
        ),
    }


def build_payload(runs: list[SelectorRun], title: str) -> dict[str, object]:
    rules = sorted({run.rule for run in runs})
    if len(rules) != 1:
        raise SystemExit("Selector summaries use different rules: " + "; ".join(rules))
    return {
        "title": title,
        "rule": rules[0],
        "aggregates": aggregate_runs(runs),
        "runs": [
            {
                "key": run.key,
                "label": run.label,
                "baseline": {key: rounded(value) for key, value in run.baseline.items()},
                "candidate_all": {key: rounded(value) for key, value in run.candidate.items()},
                "selector": {key: rounded(value) for key, value in run.selected.items()},
                "delta_vs_baseline": {
                    key: rounded(value) for key, value in run.delta_vs_baseline.items()
                },
                "delta_vs_candidate_all": {
                    key: rounded(value) for key, value in run.delta_vs_candidate.items()
                },
                "selection": {key: rounded(value) for key, value in run.selection.items()},
            }
            for run in runs
        ],
    }


def fmt_pp(value: float | None) -> str:
    if value is None:
        return "n/a"
    return f"{value:+.2f} pp"


def fmt_m(value: float | None) -> str:
    if value is None:
        return "n/a"
    return f"{value:+.3f} m"


def render_markdown(payload: dict[str, object]) -> str:
    aggregates = required_dict(payload, "aggregates", "payload")
    runs = payload.get("runs")
    if not isinstance(runs, list):
        raise SystemExit("payload: missing runs")
    lines = [
        f"# {payload.get('title', 'PPC dual-profile selector')}",
        "",
        f"Rule: `{payload['rule']}`",
        "",
        "## Aggregate",
        "",
        "| metric | value |",
        "|---|---:|",
        f"| reset10 weighted official | {aggregates['weighted_baseline_official_score_pct']:.6f}% |",
        f"| jump0.5 candidate-all weighted official | {aggregates['weighted_candidate_all_official_score_pct']:.6f}% |",
        f"| dual selector weighted official | {aggregates['weighted_selector_official_score_pct']:.6f}% |",
        f"| dual selector vs reset10 | {fmt_m(aggregates['selector_official_score_delta_m'])} / {fmt_pp(aggregates['selector_official_score_delta_pct'])} |",
        f"| candidate-all vs reset10 | {fmt_m(aggregates['candidate_all_official_score_delta_m'])} / {fmt_pp(aggregates['candidate_all_official_score_delta_pct'])} |",
        f"| selector vs candidate-all | {fmt_m(aggregates['selector_vs_candidate_all_official_score_delta_m'])} / {fmt_pp(aggregates['selector_vs_candidate_all_official_score_delta_pct'])} |",
        f"| average Positioning delta vs reset10 | {fmt_pp(aggregates['avg_positioning_delta_pct'])} |",
        f"| average Fix delta vs reset10 | {fmt_pp(aggregates['avg_fix_delta_pct'])} |",
        "",
        "## Runs",
        "",
        "| run | reset10 | candidate-all | selector | selector delta m | Positioning delta | Fix delta |",
        "|---|---:|---:|---:|---:|---:|---:|",
    ]
    for run in runs:
        if not isinstance(run, dict):
            continue
        baseline = required_dict(run, "baseline", str(run.get("key")))
        candidate = required_dict(run, "candidate_all", str(run.get("key")))
        selector = required_dict(run, "selector", str(run.get("key")))
        delta = required_dict(run, "delta_vs_baseline", str(run.get("key")))
        lines.append(
            "| {label} | {baseline:.6f}% | {candidate:.6f}% | {selector:.6f}% | "
            "{delta_m:+.3f} | {pos:+.3f} pp | {fix:+.3f} pp |".format(
                label=run.get("label", run.get("key")),
                baseline=baseline["ppc_official_score_pct"],
                candidate=candidate["ppc_official_score_pct"],
                selector=selector["ppc_official_score_pct"],
                delta_m=delta["ppc_official_score_distance_m"],
                pos=delta["positioning_rate_pct"],
                fix=delta["fix_rate_pct"],
            )
        )
    lines.append("")
    return "\n".join(lines)


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
    ax.text(x + 0.052 * w, y + 0.16 * h, detail, fontsize=9.7, color=MUTED)


def draw_official_bars(ax, runs: list[dict[str, object]]) -> None:
    import numpy as np

    labels = [str(run["label"]) for run in runs]
    baseline = [required_dict(run, "baseline", str(run["key"]))["ppc_official_score_pct"] for run in runs]
    candidate = [
        required_dict(run, "candidate_all", str(run["key"]))["ppc_official_score_pct"] for run in runs
    ]
    selector = [required_dict(run, "selector", str(run["key"]))["ppc_official_score_pct"] for run in runs]
    positions = np.arange(len(labels))
    width = 0.24

    ax.bar(positions - width, baseline, width, color=BASELINE, label="reset10")
    ax.bar(positions, candidate, width, color=CANDIDATE, label="jump0.5 all")
    ax.bar(positions + width, selector, width, color=SELECTOR, label="dual selector")
    ax.set_ylim(0, max(max(baseline), max(candidate), max(selector), 1.0) * 1.22)
    ax.set_xticks(positions)
    ax.set_xticklabels(labels, color=TEXT, fontsize=9.2)
    ax.set_ylabel("PPC official score (%)", color=MUTED, fontsize=9.5)
    ax.tick_params(axis="y", labelsize=8.5, colors=MUTED)
    ax.grid(axis="y", alpha=0.22, color=GRID)
    ax.legend(loc="upper left", frameon=False, fontsize=9.4, ncol=3)
    ax.set_title("Official score by run", color=TEXT, fontsize=14.5, weight="bold", pad=10)
    for spine in ax.spines.values():
        spine.set_alpha(0.15)

    ymax = ax.get_ylim()[1]
    for index, run in enumerate(runs):
        delta = required_dict(run, "delta_vs_baseline", str(run["key"]))["ppc_official_score_distance_m"]
        color = WIN if delta >= 0.0 else LOSS
        ax.text(
            positions[index] + width,
            selector[index] + ymax * 0.028,
            f"{delta:+.0f} m",
            color=color,
            fontsize=8.2,
            ha="center",
            weight="bold",
        )


def draw_run_table(ax, runs: list[dict[str, object]]) -> None:
    ax.set_axis_off()
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)
    ax.text(0.0, 0.965, "Selector deltas vs reset10", fontsize=14.5, color=TEXT, weight="bold", va="top")
    ax.text(0.0, 0.895, "Segment-local FIX + residual evidence; positive deltas are better.", fontsize=9.2, color=MUTED, va="top")

    headers = ["Run", "Score", "Pos", "Fix", "Selected"]
    xs = [0.02, 0.44, 0.61, 0.77, 0.99]
    aligns = ["left", "right", "right", "right", "right"]
    header_y = 0.78
    for x, header, align in zip(xs, headers, aligns):
        ax.text(x, header_y, header, fontsize=9.0, color=MUTED, weight="bold", ha=align, va="center")
    ax.plot([0.0, 1.0], [0.735, 0.735], color=EDGE, linewidth=1.0)

    y = 0.655
    for run in runs:
        delta = required_dict(run, "delta_vs_baseline", str(run["key"]))
        selection = required_dict(run, "selection", str(run["key"]))
        score_delta = delta["ppc_official_score_distance_m"]
        score_color = WIN if score_delta >= 0.0 else LOSS
        ax.text(xs[0], y, str(run["label"]), fontsize=9.8, color=TEXT, weight="bold", ha="left", va="center")
        ax.text(xs[1], y, f"{score_delta:+.1f} m", fontsize=9.6, color=score_color, weight="bold", ha="right", va="center")
        ax.text(xs[2], y, f"{delta['positioning_rate_pct']:+.2f}", fontsize=9.6, color=WIN, weight="bold", ha="right", va="center")
        ax.text(xs[3], y, f"{delta['fix_rate_pct']:+.2f}", fontsize=9.6, color=WIN, weight="bold", ha="right", va="center")
        ax.text(xs[4], y, f"{int(selection['candidate_selected_segments'])}", fontsize=9.6, color=TEXT, weight="bold", ha="right", va="center")
        y -= 0.095


def render_png(payload: dict[str, object], output: Path) -> None:
    import matplotlib.pyplot as plt

    aggregates = required_dict(payload, "aggregates", "payload")
    raw_runs = payload.get("runs")
    if not isinstance(raw_runs, list):
        raise SystemExit("payload: missing runs")
    runs = [run for run in raw_runs if isinstance(run, dict)]

    fig = plt.figure(figsize=(14, 7.8), dpi=100, facecolor=BG)
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_axis_off()
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)

    ax.text(0.05, 0.925, "PPC dual-profile selector", fontsize=21, color=MUTED, weight="bold")
    ax.text(0.05, 0.845, "reset10 baseline plus jump0.5 recovery", fontsize=34, color=TEXT, weight="bold")
    ax.text(
        0.05,
        0.785,
        "Same GNSS-only PPC Tokyo/Nagoya public observations. Selection is segment-local, not city-wide.",
        fontsize=13.3,
        color=MUTED,
    )

    draw_summary_card(
        ax,
        0.05,
        0.625,
        0.205,
        0.105,
        "Weighted official score",
        f"{aggregates['weighted_selector_official_score_pct']:.2f}%",
        f"reset10 {aggregates['weighted_baseline_official_score_pct']:.2f}%",
        value_color=WIN,
    )
    draw_summary_card(
        ax,
        0.282,
        0.625,
        0.205,
        0.105,
        "Official gain",
        f"+{aggregates['selector_official_score_delta_m']:.1f} m",
        f"{aggregates['selector_official_score_delta_pct']:+.2f} pp vs reset10",
        value_color=WIN,
    )
    draw_summary_card(
        ax,
        0.514,
        0.625,
        0.205,
        0.105,
        "Candidate-all avoided",
        f"+{aggregates['selector_vs_candidate_all_official_score_delta_m']:.1f} m",
        "selector vs jump0.5 everywhere",
        value_color=WIN,
    )
    draw_summary_card(
        ax,
        0.746,
        0.625,
        0.205,
        0.105,
        "Coverage retained",
        f"{aggregates['avg_positioning_delta_pct']:+.2f} pp",
        f"Fix {aggregates['avg_fix_delta_pct']:+.2f} pp average",
        value_color=WIN,
    )

    bar_ax = fig.add_axes([0.065, 0.105, 0.56, 0.43], facecolor=PANEL)
    draw_official_bars(bar_ax, runs)

    table_ax = fig.add_axes([0.675, 0.105, 0.275, 0.43], facecolor=BG)
    draw_run_table(table_ax, runs)

    rule_text = str(payload["rule"])
    rule_text = rule_text.replace("candidate_status_name == FIXED", "candidate FIXED")
    rule_text = rule_text.replace(
        "candidate_rtk_update_post_suppression_residual_rms_m",
        "post RMS",
    )
    rule_text = rule_text.replace(
        "candidate_rtk_update_normalized_innovation_squared_per_observation",
        "NIS/obs",
    )
    rule_text = rule_text.replace("candidate_num_satellites", "candidate sats")
    rule_text = rule_text.replace("candidate_baseline_m", "candidate baseline")
    ax.text(0.05, 0.035, f"Rule: {rule_text}.", fontsize=10.2, color=MUTED)

    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=100, facecolor=BG)
    plt.close(fig)


def main() -> int:
    args = parse_args()
    runs = load_runs([parse_run(value) for value in args.run])
    payload = build_payload(runs, args.title)
    if args.summary_json is not None:
        args.summary_json.parent.mkdir(parents=True, exist_ok=True)
        args.summary_json.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    if args.markdown_output is not None:
        args.markdown_output.parent.mkdir(parents=True, exist_ok=True)
        args.markdown_output.write_text(render_markdown(payload), encoding="utf-8")
    if args.output_png is not None:
        render_png(payload, args.output_png)
    if args.summary_json is None and args.markdown_output is None and args.output_png is None:
        print(render_markdown(payload))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
