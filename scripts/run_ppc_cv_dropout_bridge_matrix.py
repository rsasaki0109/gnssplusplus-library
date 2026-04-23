#!/usr/bin/env python3
"""Apply a causal constant-velocity bridge to PPC dropout spans and score it."""

from __future__ import annotations

import argparse
import bisect
from dataclasses import dataclass
import json
import math
from pathlib import Path
import sys

import numpy as np


ROOT_DIR = Path(__file__).resolve().parents[1]
SCRIPTS_DIR = Path(__file__).resolve().parent
APPS_DIR = ROOT_DIR / "apps"
if str(SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_DIR))
if str(APPS_DIR) not in sys.path:
    sys.path.insert(0, str(APPS_DIR))

import apply_ppc_dual_profile_selector as pos_writer  # noqa: E402
import generate_driving_comparison as comparison  # noqa: E402
import gnss_ppc_metrics as ppc_metrics  # noqa: E402


DEFAULT_RUNS = (
    ("tokyo", "run1"),
    ("tokyo", "run2"),
    ("tokyo", "run3"),
    ("nagoya", "run1"),
    ("nagoya", "run2"),
    ("nagoya", "run3"),
)


@dataclass(frozen=True)
class RunSpec:
    city: str
    run: str

    @property
    def key(self) -> str:
        return f"{self.city}_{self.run}"


@dataclass(frozen=True)
class DropoutSpan:
    start_reference_index: int
    end_reference_index: int
    start_tow_s: float
    end_tow_s: float
    duration_s: float
    distance_m: float
    segment_count: int


@dataclass(frozen=True)
class BridgeConfig:
    max_gap_s: float
    max_anchor_age_s: float
    max_velocity_baseline_s: float
    bridge_status: int
    bridge_num_satellites: int
    match_tolerance_s: float
    threshold_m: float


def rounded(value: float, digits: int = 6) -> float:
    return round(float(value), digits)


def parse_run(value: str) -> RunSpec:
    city, separator, run = value.partition("/")
    if not separator or not city or not run:
        raise argparse.ArgumentTypeError("--ppc-run must use CITY/RUN")
    return RunSpec(city, run)


def default_run_specs() -> list[RunSpec]:
    return [RunSpec(city, run) for city, run in DEFAULT_RUNS]


def gps_seconds(week: int, tow: float) -> float:
    return week * 604800.0 + tow


def epoch_key(epoch: comparison.SolutionEpoch) -> tuple[int, float]:
    return epoch.week, rounded(epoch.tow)


def solution_by_key(
    epochs: list[comparison.SolutionEpoch],
) -> dict[tuple[int, float], comparison.SolutionEpoch]:
    return {epoch_key(epoch): epoch for epoch in epochs}


def trusted_anchor_epochs(
    reference: list[comparison.ReferenceEpoch],
    epochs: list[comparison.SolutionEpoch],
    match_tolerance_s: float,
    threshold_m: float,
) -> list[comparison.SolutionEpoch]:
    reference_seconds = [gps_seconds(ref.week, ref.tow) for ref in reference]
    anchors: list[comparison.SolutionEpoch] = []
    for epoch in epochs:
        epoch_seconds = gps_seconds(epoch.week, epoch.tow)
        index = bisect.bisect_left(reference_seconds, epoch_seconds)
        candidate_indexes = [
            candidate
            for candidate in (index - 1, index, index + 1)
            if 0 <= candidate < len(reference)
        ]
        if not candidate_indexes:
            continue
        ref = min(
            (reference[candidate] for candidate in candidate_indexes),
            key=lambda candidate: abs(gps_seconds(candidate.week, candidate.tow) - epoch_seconds),
        )
        if ref.week != epoch.week:
            continue
        if abs(ref.tow - epoch.tow) > match_tolerance_s:
            continue
        error_3d_m = float(np.linalg.norm(epoch.ecef - ref.ecef))
        if error_3d_m <= threshold_m:
            anchors.append(epoch)
    anchors.sort(key=lambda epoch: gps_seconds(epoch.week, epoch.tow))
    return anchors


def dropout_spans(records: list[dict[str, object]]) -> list[DropoutSpan]:
    spans: list[DropoutSpan] = []
    current: list[dict[str, object]] = []

    def flush() -> None:
        nonlocal current
        if not current:
            return
        spans.append(
            DropoutSpan(
                start_reference_index=int(current[0]["reference_index"]),
                end_reference_index=int(current[-1]["reference_index"]),
                start_tow_s=float(current[0]["start_tow_s"]),
                end_tow_s=float(current[-1]["end_tow_s"]),
                duration_s=max(
                    0.0,
                    float(current[-1]["end_tow_s"]) - float(current[0]["start_tow_s"]),
                ),
                distance_m=sum(float(record["segment_distance_m"]) for record in current),
                segment_count=len(current),
            )
        )
        current = []

    for record in records:
        if record["score_state"] == "no_solution":
            current.append(record)
            continue
        flush()
    flush()
    return spans


def select_velocity_anchors(
    anchors: list[comparison.SolutionEpoch],
    span: DropoutSpan,
    reference_week: int,
    config: BridgeConfig,
) -> tuple[comparison.SolutionEpoch, comparison.SolutionEpoch] | None:
    span_start_s = gps_seconds(reference_week, span.start_tow_s)
    prior = [
        epoch
        for epoch in anchors
        if gps_seconds(epoch.week, epoch.tow) <= span_start_s
    ]
    if len(prior) < 2:
        return None
    last = prior[-1]
    previous = prior[-2]
    last_s = gps_seconds(last.week, last.tow)
    previous_s = gps_seconds(previous.week, previous.tow)
    if last_s <= previous_s:
        return None
    if span_start_s - last_s > config.max_anchor_age_s:
        return None
    if last_s - previous_s > config.max_velocity_baseline_s:
        return None
    return previous, last


def propagated_epoch(
    reference_epoch: comparison.ReferenceEpoch,
    previous_anchor: comparison.SolutionEpoch,
    last_anchor: comparison.SolutionEpoch,
    config: BridgeConfig,
) -> comparison.SolutionEpoch:
    previous_s = gps_seconds(previous_anchor.week, previous_anchor.tow)
    last_s = gps_seconds(last_anchor.week, last_anchor.tow)
    target_s = gps_seconds(reference_epoch.week, reference_epoch.tow)
    velocity_ecef_mps = (last_anchor.ecef - previous_anchor.ecef) / max(last_s - previous_s, 1e-9)
    ecef = last_anchor.ecef + velocity_ecef_mps * (target_s - last_s)
    lat_deg, lon_deg, height_m = ppc_metrics.llh_from_ecef(float(ecef[0]), float(ecef[1]), float(ecef[2]))
    return comparison.SolutionEpoch(
        reference_epoch.week,
        reference_epoch.tow,
        lat_deg,
        lon_deg,
        height_m,
        ecef,
        config.bridge_status,
        config.bridge_num_satellites,
    )


def bridge_dropout_spans(
    reference: list[comparison.ReferenceEpoch],
    baseline_epochs: list[comparison.SolutionEpoch],
    baseline_records: list[dict[str, object]],
    config: BridgeConfig,
) -> tuple[list[comparison.SolutionEpoch], list[dict[str, object]]]:
    anchors = trusted_anchor_epochs(
        reference,
        baseline_epochs,
        config.match_tolerance_s,
        config.threshold_m,
    )
    spans = dropout_spans(baseline_records)
    selected_by_key = solution_by_key(baseline_epochs)
    selected_rows: list[dict[str, object]] = []

    for span in spans:
        row: dict[str, object] = {
            "start_reference_index": span.start_reference_index,
            "end_reference_index": span.end_reference_index,
            "start_tow_s": rounded(span.start_tow_s),
            "end_tow_s": rounded(span.end_tow_s),
            "duration_s": rounded(span.duration_s),
            "distance_m": rounded(span.distance_m),
            "segment_count": span.segment_count,
            "bridge_applied": False,
            "generated_epochs": 0,
            "reject_reason": None,
        }
        if span.duration_s > config.max_gap_s:
            row["reject_reason"] = "span_too_long"
            selected_rows.append(row)
            continue
        if span.end_reference_index >= len(reference):
            row["reject_reason"] = "reference_index_out_of_range"
            selected_rows.append(row)
            continue
        anchors_pair = select_velocity_anchors(
            anchors,
            span,
            reference[span.start_reference_index].week,
            config,
        )
        if anchors_pair is None:
            row["reject_reason"] = "missing_recent_trusted_anchors"
            selected_rows.append(row)
            continue

        previous_anchor, last_anchor = anchors_pair
        generated_count = 0
        for ref_index in range(span.start_reference_index, span.end_reference_index + 1):
            ref = reference[ref_index]
            epoch = propagated_epoch(ref, previous_anchor, last_anchor, config)
            selected_by_key[epoch_key(epoch)] = epoch
            generated_count += 1
        row.update(
            {
                "bridge_applied": True,
                "generated_epochs": generated_count,
                "previous_anchor_tow_s": rounded(previous_anchor.tow),
                "last_anchor_tow_s": rounded(last_anchor.tow),
                "anchor_age_s": rounded(span.start_tow_s - last_anchor.tow),
                "velocity_baseline_s": rounded(last_anchor.tow - previous_anchor.tow),
            }
        )
        selected_rows.append(row)

    return [selected_by_key[key] for key in sorted(selected_by_key)], selected_rows


def records_by_reference_index(records: list[dict[str, object]]) -> dict[int, dict[str, object]]:
    return {int(record["reference_index"]): record for record in records}


def enrich_bridge_rows(
    rows: list[dict[str, object]],
    baseline_records: list[dict[str, object]],
    bridged_records: list[dict[str, object]],
) -> None:
    baseline_by_index = records_by_reference_index(baseline_records)
    bridged_by_index = records_by_reference_index(bridged_records)
    for row in rows:
        recovered_distance_m = 0.0
        matched_distance_m = 0.0
        errors: list[float] = []
        for ref_index in range(
            int(row["start_reference_index"]),
            int(row["end_reference_index"]) + 1,
        ):
            baseline_record = baseline_by_index.get(ref_index)
            bridged_record = bridged_by_index.get(ref_index)
            if baseline_record is None or bridged_record is None:
                continue
            segment_distance_m = float(bridged_record["segment_distance_m"])
            before_score = segment_distance_m if bool(baseline_record["scored"]) else 0.0
            after_score = segment_distance_m if bool(bridged_record["scored"]) else 0.0
            recovered_distance_m += after_score - before_score
            if bool(bridged_record["matched"]):
                matched_distance_m += segment_distance_m
            error = bridged_record.get("error_3d_m")
            if error is not None:
                errors.append(float(error))
        row["recovered_distance_m"] = rounded(recovered_distance_m)
        row["matched_distance_m"] = rounded(matched_distance_m)
        row["median_3d_error_m"] = rounded(float(np.median(errors))) if errors else None
        row["max_3d_error_m"] = rounded(max(errors)) if errors else None


def bridge_selection_summary(rows: list[dict[str, object]]) -> dict[str, object]:
    applied = [row for row in rows if row["bridge_applied"]]
    rejected: dict[str, int] = {}
    for row in rows:
        reason = row.get("reject_reason")
        if reason:
            rejected[str(reason)] = rejected.get(str(reason), 0) + 1
    return {
        "dropout_span_count": len(rows),
        "bridge_span_count": len(applied),
        "generated_epochs": sum(int(row["generated_epochs"]) for row in applied),
        "bridged_span_distance_m": rounded(sum(float(row["distance_m"]) for row in applied)),
        "recovered_distance_m": rounded(sum(float(row.get("recovered_distance_m", 0.0)) for row in applied)),
        "matched_bridge_distance_m": rounded(sum(float(row.get("matched_distance_m", 0.0)) for row in applied)),
        "reject_reasons": rejected,
    }


def run_output_path(template: str, run: RunSpec) -> Path:
    return Path(template.format(city=run.city, run=run.run, key=run.key))


def summarize_run(
    dataset_root: Path,
    run: RunSpec,
    baseline_pos_template: str,
    output_pos_template: str,
    run_summary_template: str | None,
    config: BridgeConfig,
) -> dict[str, object]:
    reference_csv = dataset_root / run.city / run.run / "reference.csv"
    baseline_pos = run_output_path(baseline_pos_template, run)
    output_pos = run_output_path(output_pos_template, run)
    reference = comparison.read_reference_csv(reference_csv)
    baseline_epochs = comparison.read_libgnss_pos(baseline_pos)
    baseline_records = ppc_metrics.ppc_official_segment_records(
        reference,
        baseline_epochs,
        config.match_tolerance_s,
        config.threshold_m,
    )
    bridged_epochs, bridge_rows = bridge_dropout_spans(
        reference,
        baseline_epochs,
        baseline_records,
        config,
    )
    bridged_records = ppc_metrics.ppc_official_segment_records(
        reference,
        bridged_epochs,
        config.match_tolerance_s,
        config.threshold_m,
    )
    enrich_bridge_rows(bridge_rows, baseline_records, bridged_records)
    pos_writer.write_pos(output_pos, bridged_epochs)

    baseline_metrics = ppc_metrics.summarize_solution_epochs(
        reference,
        baseline_epochs,
        fixed_status=4,
        label=f"{run.key} baseline",
        match_tolerance_s=config.match_tolerance_s,
        solver_wall_time_s=None,
    )
    bridged_metrics = ppc_metrics.summarize_solution_epochs(
        reference,
        bridged_epochs,
        fixed_status=4,
        label=f"{run.key} cv bridge",
        match_tolerance_s=config.match_tolerance_s,
        solver_wall_time_s=None,
    )
    payload = {
        "key": run.key,
        "city": run.city,
        "run": run.run,
        "reference_csv": str(reference_csv),
        "baseline_pos": str(baseline_pos),
        "output_pos": str(output_pos),
        "config": config.__dict__,
        "selection": bridge_selection_summary(bridge_rows),
        "baseline": baseline_metrics,
        "metrics": bridged_metrics,
        "delta_vs_baseline": ppc_metrics.solution_metric_delta(bridged_metrics, baseline_metrics),
        "bridge_spans": bridge_rows,
    }
    if run_summary_template:
        summary_path = run_output_path(run_summary_template, run)
        summary_path.parent.mkdir(parents=True, exist_ok=True)
        summary_path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return payload


def weighted_score_distance(runs: list[dict[str, object]], key: str) -> float:
    return sum(float(run[key]["ppc_official_score_distance_m"]) for run in runs)  # type: ignore[index]


def total_distance(runs: list[dict[str, object]]) -> float:
    return sum(float(run["baseline"]["ppc_official_total_distance_m"]) for run in runs)  # type: ignore[index]


def build_matrix_payload(runs: list[dict[str, object]], title: str, config: BridgeConfig) -> dict[str, object]:
    total_distance_m = total_distance(runs)
    baseline_score_m = weighted_score_distance(runs, "baseline")
    bridge_score_m = weighted_score_distance(runs, "metrics")
    selection = {
        "dropout_span_count": sum(int(run["selection"]["dropout_span_count"]) for run in runs),  # type: ignore[index]
        "bridge_span_count": sum(int(run["selection"]["bridge_span_count"]) for run in runs),  # type: ignore[index]
        "generated_epochs": sum(int(run["selection"]["generated_epochs"]) for run in runs),  # type: ignore[index]
        "bridged_span_distance_m": rounded(sum(float(run["selection"]["bridged_span_distance_m"]) for run in runs)),  # type: ignore[index]
        "recovered_distance_m": rounded(sum(float(run["selection"]["recovered_distance_m"]) for run in runs)),  # type: ignore[index]
        "matched_bridge_distance_m": rounded(sum(float(run["selection"]["matched_bridge_distance_m"]) for run in runs)),  # type: ignore[index]
    }
    return {
        "title": title,
        "config": config.__dict__,
        "runs": runs,
        "aggregates": {
            "run_count": len(runs),
            "total_distance_m": rounded(total_distance_m),
            "weighted_baseline_official_score_pct": rounded(100.0 * baseline_score_m / total_distance_m)
            if total_distance_m > 0.0
            else 0.0,
            "weighted_bridge_official_score_pct": rounded(100.0 * bridge_score_m / total_distance_m)
            if total_distance_m > 0.0
            else 0.0,
            "bridge_official_score_delta_m": rounded(bridge_score_m - baseline_score_m),
            "bridge_official_score_delta_pct": rounded(100.0 * (bridge_score_m - baseline_score_m) / total_distance_m)
            if total_distance_m > 0.0
            else 0.0,
            "avg_positioning_delta_pct": rounded(
                sum(float(run["delta_vs_baseline"]["positioning_rate_pct"]) for run in runs) / max(len(runs), 1)  # type: ignore[index]
            ),
            "avg_fix_delta_pct": rounded(
                sum(float(run["delta_vs_baseline"]["fix_rate_pct"]) for run in runs) / max(len(runs), 1)  # type: ignore[index]
            ),
            "selection": selection,
        },
    }


def run_label(key: str) -> str:
    city, _, run = key.partition("_")
    return f"{city.capitalize()} {run.replace('run', 'r')}" if run else key


def fmt_pct(value: object) -> str:
    return f"{float(value):.2f}%"


def fmt_m(value: object) -> str:
    return f"{float(value):+,.1f} m"


def render_markdown(payload: dict[str, object]) -> str:
    aggregates = payload["aggregates"]  # type: ignore[index]
    selection = aggregates["selection"]  # type: ignore[index]
    lines = [
        f"# {payload['title']}",
        "",
        (
            f"Weighted PPC official score moves **{fmt_pct(aggregates['weighted_baseline_official_score_pct'])} "
            f"-> {fmt_pct(aggregates['weighted_bridge_official_score_pct'])}** "
            f"(**{fmt_m(aggregates['bridge_official_score_delta_m'])}**, "
            f"**{fmt_pct(aggregates['bridge_official_score_delta_pct'])}**)."
        ),
        (
            f"The bridge generated **{selection['generated_epochs']:,}** epochs across "
            f"**{selection['bridge_span_count']} / {selection['dropout_span_count']}** dropout spans and recovered "
            f"**{fmt_m(selection['recovered_distance_m'])}** official distance."
        ),
        "",
        "| Run | Baseline | CV bridge | Delta | Bridge spans | Generated epochs | Recovered | Positioning delta |",
        "| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |",
    ]
    for run in payload["runs"]:  # type: ignore[index]
        selection = run["selection"]
        lines.append(
            "| "
            + " | ".join(
                [
                    run_label(str(run["key"])),
                    fmt_pct(run["baseline"]["ppc_official_score_pct"]),
                    fmt_pct(run["metrics"]["ppc_official_score_pct"]),
                    fmt_m(run["delta_vs_baseline"]["ppc_official_score_distance_m"]),
                    f"{selection['bridge_span_count']} / {selection['dropout_span_count']}",
                    f"{selection['generated_epochs']:,}",
                    fmt_m(selection["recovered_distance_m"]),
                    fmt_pct(run["delta_vs_baseline"]["positioning_rate_pct"]),
                ]
            )
            + " |"
        )
    lines.append("")
    return "\n".join(lines)


def render_png(payload: dict[str, object], output: Path) -> None:
    import matplotlib.pyplot as plt

    BG = "#f7f8fb"
    TEXT = "#172033"
    MUTED = "#667085"
    GRID = "#d0d5dd"
    BASE = "#64748b"
    BRIDGE = "#0f766e"
    DELTA = "#2563eb"

    aggregates = payload["aggregates"]  # type: ignore[index]
    runs = payload["runs"]  # type: ignore[index]
    labels = [run_label(str(run["key"])) for run in runs]
    baseline_scores = [float(run["baseline"]["ppc_official_score_pct"]) for run in runs]
    bridge_scores = [float(run["metrics"]["ppc_official_score_pct"]) for run in runs]
    deltas = [float(run["delta_vs_baseline"]["ppc_official_score_distance_m"]) for run in runs]

    fig = plt.figure(figsize=(13.0, 7.2), facecolor=BG)
    grid = fig.add_gridspec(
        2,
        3,
        left=0.06,
        right=0.97,
        top=0.84,
        bottom=0.10,
        height_ratios=[0.74, 1.7],
        hspace=0.44,
        wspace=0.34,
    )
    fig.text(0.06, 0.92, str(payload["title"]), fontsize=22, weight="bold", color=TEXT)
    fig.text(
        0.06,
        0.875,
        "Causal bridge from the last two scored GNSS positions; no future anchor is used",
        fontsize=11,
        color=MUTED,
    )

    selection = aggregates["selection"]
    cards = [
        (
            "Weighted score",
            f"{fmt_pct(aggregates['weighted_bridge_official_score_pct'])}",
            f"{fmt_m(aggregates['bridge_official_score_delta_m'])} vs reset10",
        ),
        (
            "Bridge spans",
            f"{selection['bridge_span_count']} / {selection['dropout_span_count']}",
            f"{selection['generated_epochs']:,} generated epochs",
        ),
        (
            "Recovered",
            fmt_m(selection["recovered_distance_m"]),
            f"matched {fmt_m(selection['matched_bridge_distance_m'])}",
        ),
    ]
    for index, (label, value, detail) in enumerate(cards):
        ax = fig.add_subplot(grid[0, index])
        ax.set_facecolor("white")
        for spine in ax.spines.values():
            spine.set_edgecolor(GRID)
        ax.set_xticks([])
        ax.set_yticks([])
        ax.text(0.05, 0.72, label, transform=ax.transAxes, fontsize=10.5, color=MUTED, weight="bold")
        ax.text(0.05, 0.35, value, transform=ax.transAxes, fontsize=22, color=BRIDGE, weight="bold")
        ax.text(0.05, 0.16, detail, transform=ax.transAxes, fontsize=9.3, color=MUTED)

    ax_score = fig.add_subplot(grid[1, :2])
    x = np.arange(len(labels))
    width = 0.36
    ax_score.bar(x - width / 2, baseline_scores, width=width, color=BASE, label="reset10")
    ax_score.bar(x + width / 2, bridge_scores, width=width, color=BRIDGE, label="CV bridge")
    ax_score.set_xticks(x)
    ax_score.set_xticklabels(labels, rotation=20, ha="right", fontsize=9)
    ax_score.set_ylabel("PPC official score (%)", fontsize=9.5, color=MUTED)
    ax_score.set_title("Per-run official score", fontsize=13.5, weight="bold", color=TEXT)
    ax_score.grid(axis="y", alpha=0.25, color=GRID)
    ax_score.legend(loc="upper right", frameon=False, fontsize=9)
    ax_score.tick_params(axis="y", labelsize=8.5, colors=MUTED)
    for spine in ax_score.spines.values():
        spine.set_alpha(0.18)

    ax_delta = fig.add_subplot(grid[1, 2])
    y = np.arange(len(labels))
    ax_delta.barh(y, deltas, color=DELTA)
    ax_delta.set_yticks(y)
    ax_delta.set_yticklabels(labels, fontsize=8.8)
    ax_delta.invert_yaxis()
    ax_delta.set_xlabel("Official distance delta (m)", fontsize=9.5, color=MUTED)
    ax_delta.set_title("Recovered distance", fontsize=13.5, weight="bold", color=TEXT)
    ax_delta.grid(axis="x", alpha=0.25, color=GRID)
    ax_delta.tick_params(axis="x", labelsize=8.5, colors=MUTED)
    xmax = max(max(deltas), 1.0)
    for index, delta_m in enumerate(deltas):
        ax_delta.text(delta_m + xmax * 0.025, index, f"{delta_m:+.1f} m", va="center", fontsize=8.3, color=TEXT)
    ax_delta.set_xlim(0.0, xmax * 1.20)
    for spine in ax_delta.spines.values():
        spine.set_alpha(0.18)

    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=180)
    plt.close(fig)


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset-root", type=Path, required=True)
    parser.add_argument(
        "--ppc-run",
        action="append",
        type=parse_run,
        dest="runs",
        help="Run to include as CITY/RUN. Defaults to all six PPC runs.",
    )
    parser.add_argument(
        "--baseline-pos-template",
        default="output/ppc_coverage_matrix_floatreset10/{key}.pos",
    )
    parser.add_argument("--run-output-template", required=True)
    parser.add_argument("--run-summary-template")
    parser.add_argument("--max-gap-s", type=float, default=5.0)
    parser.add_argument("--max-anchor-age-s", type=float, default=1.0)
    parser.add_argument("--max-velocity-baseline-s", type=float, default=1.0)
    parser.add_argument("--bridge-status", type=int, default=3)
    parser.add_argument("--bridge-num-satellites", type=int, default=0)
    parser.add_argument("--match-tolerance-s", type=float, default=0.25)
    parser.add_argument("--threshold-m", type=float, default=0.50)
    parser.add_argument("--summary-json", type=Path, required=True)
    parser.add_argument("--markdown-output", type=Path)
    parser.add_argument("--output-png", type=Path)
    parser.add_argument("--title", default="PPC causal CV dropout bridge")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(sys.argv[1:] if argv is None else argv)
    config = BridgeConfig(
        max_gap_s=args.max_gap_s,
        max_anchor_age_s=args.max_anchor_age_s,
        max_velocity_baseline_s=args.max_velocity_baseline_s,
        bridge_status=args.bridge_status,
        bridge_num_satellites=args.bridge_num_satellites,
        match_tolerance_s=args.match_tolerance_s,
        threshold_m=args.threshold_m,
    )
    runs = [
        summarize_run(
            args.dataset_root,
            run,
            args.baseline_pos_template,
            args.run_output_template,
            args.run_summary_template,
            config,
        )
        for run in (args.runs or default_run_specs())
    ]
    payload = build_matrix_payload(runs, args.title, config)
    args.summary_json.parent.mkdir(parents=True, exist_ok=True)
    args.summary_json.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    if args.markdown_output:
        args.markdown_output.parent.mkdir(parents=True, exist_ok=True)
        args.markdown_output.write_text(render_markdown(payload), encoding="utf-8")
    if args.output_png:
        render_png(payload, args.output_png)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
