#!/usr/bin/env python3
"""Estimate PPC official-score upside from short IMU dropout bridges."""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
import json
import math
from pathlib import Path
import sys


DEFAULT_RUNS = (
    ("tokyo", "run1"),
    ("tokyo", "run2"),
    ("tokyo", "run3"),
    ("nagoya", "run1"),
    ("nagoya", "run2"),
    ("nagoya", "run3"),
)
DEFAULT_GAP_THRESHOLDS_S = (0.5, 1.0, 2.0, 3.0, 5.0, 10.0, 20.0, 30.0, 60.0, 120.0)
SCORE_STATES = ("scored", "high_error", "no_solution")


@dataclass(frozen=True)
class RunSpec:
    city: str
    run: str

    @property
    def key(self) -> str:
        return f"{self.city}_{self.run}"


@dataclass(frozen=True)
class SegmentRecord:
    run_key: str
    reference_index: int
    start_tow_s: float
    end_tow_s: float
    distance_m: float
    score_state: str
    status_name: str
    error_3d_m: float | None

    @property
    def has_solution(self) -> bool:
        return self.score_state != "no_solution" and bool(self.status_name)


@dataclass(frozen=True)
class LossSpan:
    run_key: str
    score_state: str
    start_tow_s: float
    end_tow_s: float
    duration_s: float
    distance_m: float
    segment_count: int
    before_state: str | None
    after_state: str | None
    before_status_name: str | None
    after_status_name: str | None
    bracketed_by_solution: bool


def rounded(value: float, digits: int = 6) -> float:
    return round(float(value), digits)


def percentile(values: list[float], pct: float) -> float:
    if not values:
        return 0.0
    ordered = sorted(values)
    if len(ordered) == 1:
        return rounded(ordered[0])
    position = (len(ordered) - 1) * pct / 100.0
    low = math.floor(position)
    high = math.ceil(position)
    if low == high:
        return rounded(ordered[int(position)])
    fraction = position - low
    return rounded(ordered[low] * (1.0 - fraction) + ordered[high] * fraction)


def parse_optional_float(value: str | None) -> float | None:
    if value is None or not value.strip():
        return None
    return float(value)


def load_segments(path: Path, run_key: str) -> list[SegmentRecord]:
    if not path.exists():
        raise SystemExit(f"missing official-segments CSV: {path}")
    records: list[SegmentRecord] = []
    with path.open("r", encoding="utf-8", newline="") as handle:
        reader = csv.DictReader(handle)
        required = {
            "reference_index",
            "start_tow_s",
            "end_tow_s",
            "segment_distance_m",
            "lib_score_state",
        }
        missing = required.difference(reader.fieldnames or [])
        if missing:
            raise SystemExit(f"{path}: missing columns: {', '.join(sorted(missing))}")
        for row in reader:
            records.append(
                SegmentRecord(
                    run_key=run_key,
                    reference_index=int(float(row["reference_index"])),
                    start_tow_s=float(row["start_tow_s"]),
                    end_tow_s=float(row["end_tow_s"]),
                    distance_m=float(row["segment_distance_m"]),
                    score_state=str(row["lib_score_state"]),
                    status_name=str(row.get("lib_status_name", "") or ""),
                    error_3d_m=parse_optional_float(row.get("lib_error_3d_m")),
                )
            )
    records.sort(key=lambda record: record.reference_index)
    return records


def group_loss_spans(records: list[SegmentRecord], score_state: str) -> list[LossSpan]:
    spans: list[LossSpan] = []
    current: list[SegmentRecord] = []

    def flush(next_index: int) -> None:
        nonlocal current
        if not current:
            return
        first = current[0]
        last = current[-1]
        previous_record = records[next_index - len(current) - 1] if next_index - len(current) - 1 >= 0 else None
        next_record = records[next_index] if next_index < len(records) else None
        spans.append(
            LossSpan(
                run_key=first.run_key,
                score_state=score_state,
                start_tow_s=first.start_tow_s,
                end_tow_s=last.end_tow_s,
                duration_s=max(0.0, last.end_tow_s - first.start_tow_s),
                distance_m=sum(record.distance_m for record in current),
                segment_count=len(current),
                before_state=previous_record.score_state if previous_record else None,
                after_state=next_record.score_state if next_record else None,
                before_status_name=previous_record.status_name if previous_record else None,
                after_status_name=next_record.status_name if next_record else None,
                bracketed_by_solution=bool(
                    previous_record is not None
                    and next_record is not None
                    and previous_record.has_solution
                    and next_record.has_solution
                ),
            )
        )
        current = []

    for index, record in enumerate(records):
        if record.score_state == score_state:
            current.append(record)
            continue
        flush(index)
    flush(len(records))
    return spans


def score_state_distances(records: list[SegmentRecord]) -> dict[str, float]:
    distances = {state: 0.0 for state in SCORE_STATES}
    for record in records:
        if record.score_state in distances:
            distances[record.score_state] += record.distance_m
    return {state: rounded(distance) for state, distance in distances.items()}


def high_error_by_status(records: list[SegmentRecord]) -> list[dict[str, object]]:
    grouped: dict[str, list[SegmentRecord]] = {}
    for record in records:
        if record.score_state != "high_error":
            continue
        grouped.setdefault(record.status_name or "UNKNOWN", []).append(record)
    total_high_error = sum(record.distance_m for rows in grouped.values() for record in rows)
    rows: list[dict[str, object]] = []
    for status_name, status_records in grouped.items():
        distance_m = sum(record.distance_m for record in status_records)
        errors = [
            record.error_3d_m
            for record in status_records
            if record.error_3d_m is not None and math.isfinite(record.error_3d_m)
        ]
        rows.append(
            {
                "status_name": status_name,
                "segments": len(status_records),
                "distance_m": rounded(distance_m),
                "share_of_high_error_pct": rounded(100.0 * distance_m / total_high_error)
                if total_high_error > 0.0
                else 0.0,
                "median_3d_error_m": percentile(errors, 50) if errors else None,
                "p95_3d_error_m": percentile(errors, 95) if errors else None,
            }
        )
    rows.sort(key=lambda row: float(row["distance_m"]), reverse=True)
    return rows


def span_payload(span: LossSpan) -> dict[str, object]:
    return {
        "run_key": span.run_key,
        "score_state": span.score_state,
        "start_tow_s": rounded(span.start_tow_s),
        "end_tow_s": rounded(span.end_tow_s),
        "duration_s": rounded(span.duration_s),
        "distance_m": rounded(span.distance_m),
        "segment_count": span.segment_count,
        "before_state": span.before_state,
        "after_state": span.after_state,
        "before_status_name": span.before_status_name,
        "after_status_name": span.after_status_name,
        "bracketed_by_solution": span.bracketed_by_solution,
    }


def recoverable_spans(
    spans: list[LossSpan],
    max_gap_s: float,
    require_bracketed_solution: bool,
) -> list[LossSpan]:
    return [
        span
        for span in spans
        if span.duration_s <= max_gap_s
        and (span.bracketed_by_solution or not require_bracketed_solution)
    ]


def bridge_threshold_rows(
    records: list[SegmentRecord],
    no_solution_spans: list[LossSpan],
    thresholds_s: list[float],
    *,
    require_bracketed_solution: bool,
) -> list[dict[str, object]]:
    distances = score_state_distances(records)
    total_distance_m = sum(distances.values())
    scored_distance_m = distances["scored"]
    no_solution_distance_m = distances["no_solution"]
    rows: list[dict[str, object]] = []
    for threshold_s in sorted(thresholds_s):
        recovered = recoverable_spans(
            no_solution_spans,
            threshold_s,
            require_bracketed_solution,
        )
        recovered_distance_m = sum(span.distance_m for span in recovered)
        score_distance_m = scored_distance_m + recovered_distance_m
        rows.append(
            {
                "max_gap_s": rounded(threshold_s, 3),
                "bridge_span_count": len(recovered),
                "recovered_no_solution_distance_m": rounded(recovered_distance_m),
                "remaining_no_solution_distance_m": rounded(
                    max(0.0, no_solution_distance_m - recovered_distance_m)
                ),
                "score_distance_m": rounded(score_distance_m),
                "score_pct": rounded(100.0 * score_distance_m / total_distance_m)
                if total_distance_m > 0.0
                else 0.0,
                "score_delta_distance_m": rounded(recovered_distance_m),
                "score_delta_pct": rounded(100.0 * recovered_distance_m / total_distance_m)
                if total_distance_m > 0.0
                else 0.0,
            }
        )
    return rows


def run_summary(
    run_key: str,
    records: list[SegmentRecord],
    thresholds_s: list[float],
    *,
    require_bracketed_solution: bool,
) -> dict[str, object]:
    distances = score_state_distances(records)
    total_distance_m = sum(distances.values())
    no_solution_spans = group_loss_spans(records, "no_solution")
    high_error_spans = group_loss_spans(records, "high_error")
    return {
        "key": run_key,
        "segments": len(records),
        "distance_m": rounded(total_distance_m),
        "score_state_distances_m": distances,
        "baseline_score_pct": rounded(100.0 * distances["scored"] / total_distance_m)
        if total_distance_m > 0.0
        else 0.0,
        "no_solution_span_count": len(no_solution_spans),
        "no_solution_bracketed_span_count": sum(
            1 for span in no_solution_spans if span.bracketed_by_solution
        ),
        "high_error_span_count": len(high_error_spans),
        "largest_no_solution_spans": [
            span_payload(span)
            for span in sorted(no_solution_spans, key=lambda span: span.distance_m, reverse=True)[:8]
        ],
        "high_error_by_status": high_error_by_status(records),
        "bridge_thresholds": bridge_threshold_rows(
            records,
            no_solution_spans,
            thresholds_s,
            require_bracketed_solution=require_bracketed_solution,
        ),
    }


def aggregate_payload(
    records_by_run: dict[str, list[SegmentRecord]],
    thresholds_s: list[float],
    *,
    require_bracketed_solution: bool,
) -> dict[str, object]:
    all_records = [record for records in records_by_run.values() for record in records]
    distances = score_state_distances(all_records)
    total_distance_m = sum(distances.values())
    no_solution_spans = [
        span
        for records in records_by_run.values()
        for span in group_loss_spans(records, "no_solution")
    ]
    high_error_spans = [
        span
        for records in records_by_run.values()
        for span in group_loss_spans(records, "high_error")
    ]
    return {
        "run_count": len(records_by_run),
        "segments": len(all_records),
        "distance_m": rounded(total_distance_m),
        "score_state_distances_m": distances,
        "baseline_score_pct": rounded(100.0 * distances["scored"] / total_distance_m)
        if total_distance_m > 0.0
        else 0.0,
        "no_solution_span_count": len(no_solution_spans),
        "no_solution_bracketed_span_count": sum(
            1 for span in no_solution_spans if span.bracketed_by_solution
        ),
        "high_error_span_count": len(high_error_spans),
        "bridge_thresholds": bridge_threshold_rows(
            all_records,
            no_solution_spans,
            thresholds_s,
            require_bracketed_solution=require_bracketed_solution,
        ),
        "high_error_by_status": high_error_by_status(all_records),
        "largest_no_solution_spans": [
            span_payload(span)
            for span in sorted(no_solution_spans, key=lambda span: span.distance_m, reverse=True)[:12]
        ],
    }


def parse_run(value: str) -> RunSpec:
    city, separator, run = value.partition("/")
    if not separator or not city or not run:
        raise argparse.ArgumentTypeError("--run must use CITY/RUN")
    return RunSpec(city, run)


def default_run_specs() -> list[RunSpec]:
    return [RunSpec(city, run) for city, run in DEFAULT_RUNS]


def build_payload(
    segment_csv_template: str,
    runs: list[RunSpec],
    thresholds_s: list[float],
    *,
    require_bracketed_solution: bool = True,
) -> dict[str, object]:
    records_by_run = {
        run.key: load_segments(
            Path(segment_csv_template.format(city=run.city, run=run.run, key=run.key)),
            run.key,
        )
        for run in runs
    }
    run_payloads = [
        run_summary(
            run.key,
            records_by_run[run.key],
            thresholds_s,
            require_bracketed_solution=require_bracketed_solution,
        )
        for run in runs
    ]
    return {
        "segment_csv_template": segment_csv_template,
        "require_bracketed_solution": require_bracketed_solution,
        "thresholds_s": [rounded(value, 3) for value in sorted(thresholds_s)],
        "runs": run_payloads,
        "aggregates": aggregate_payload(
            records_by_run,
            thresholds_s,
            require_bracketed_solution=require_bracketed_solution,
        ),
    }


def run_label(key: str) -> str:
    city, _, run = key.partition("_")
    return f"{city.capitalize()} {run.replace('run', 'r')}" if run else key


def fmt_m(value: object) -> str:
    return f"{float(value):,.1f} m"


def fmt_pct(value: object) -> str:
    return f"{float(value):.2f}%"


def threshold_row(rows: list[dict[str, object]], max_gap_s: float) -> dict[str, object]:
    if not rows:
        raise SystemExit("no threshold rows")
    return min(rows, key=lambda row: abs(float(row["max_gap_s"]) - max_gap_s))


def render_markdown(payload: dict[str, object]) -> str:
    aggregates = payload["aggregates"]  # type: ignore[index]
    distances = aggregates["score_state_distances_m"]  # type: ignore[index]
    threshold_rows = aggregates["bridge_thresholds"]  # type: ignore[index]
    best_row = max(threshold_rows, key=lambda row: float(row["score_pct"]))
    lines = [
        "# PPC IMU Bridge Targets",
        "",
        (
            f"Baseline reset10 score is **{fmt_pct(aggregates['baseline_score_pct'])}** "
            f"over **{fmt_m(aggregates['distance_m'])}**. The no-solution pool is "
            f"**{fmt_m(distances['no_solution'])}** across "
            f"**{aggregates['no_solution_span_count']}** spans "
            f"(**{aggregates['no_solution_bracketed_span_count']}** bracketed by solutions)."
        ),
        (
            f"If every bracketed no-solution span up to **{best_row['max_gap_s']:.1f} s** "
            f"were bridged within the 0.5 m official threshold, the upper-bound score becomes "
            f"**{fmt_pct(best_row['score_pct'])}** "
            f"(**+{fmt_m(best_row['score_delta_distance_m'])}**)."
        ),
        "",
        "| Max bridge gap | Bridge spans | Recovered no-solution | Score | Delta | Remaining no-solution |",
        "| ---: | ---: | ---: | ---: | ---: | ---: |",
    ]
    for row in threshold_rows:
        lines.append(
            "| "
            + " | ".join(
                [
                    f"{float(row['max_gap_s']):.1f} s",
                    f"{int(row['bridge_span_count']):,}",
                    fmt_m(row["recovered_no_solution_distance_m"]),
                    fmt_pct(row["score_pct"]),
                    fmt_m(row["score_delta_distance_m"]),
                    fmt_m(row["remaining_no_solution_distance_m"]),
                ]
            )
            + " |"
        )

    lines.extend(
        [
            "",
            "| Run | Baseline | No solution | <= 5 s upper bound | <= 30 s upper bound | High-error |",
            "| --- | ---: | ---: | ---: | ---: | ---: |",
        ]
    )
    for run in payload["runs"]:  # type: ignore[index]
        run_distances = run["score_state_distances_m"]
        row_5 = threshold_row(run["bridge_thresholds"], 5.0)
        row_30 = threshold_row(run["bridge_thresholds"], 30.0)
        lines.append(
            "| "
            + " | ".join(
                [
                    run_label(str(run["key"])),
                    fmt_pct(run["baseline_score_pct"]),
                    fmt_m(run_distances["no_solution"]),
                    fmt_pct(row_5["score_pct"]),
                    fmt_pct(row_30["score_pct"]),
                    fmt_m(run_distances["high_error"]),
                ]
            )
            + " |"
        )

    lines.extend(
        [
            "",
            "| High-error status | Distance | Share | Median 3D | P95 3D |",
            "| --- | ---: | ---: | ---: | ---: |",
        ]
    )
    for row in aggregates["high_error_by_status"]:  # type: ignore[index]
        median = row["median_3d_error_m"]
        p95 = row["p95_3d_error_m"]
        lines.append(
            "| "
            + " | ".join(
                [
                    str(row["status_name"]),
                    fmt_m(row["distance_m"]),
                    fmt_pct(row["share_of_high_error_pct"]),
                    fmt_m(median) if median is not None else "n/a",
                    fmt_m(p95) if p95 is not None else "n/a",
                ]
            )
            + " |"
        )

    lines.extend(
        [
            "",
            "| Largest no-solution span | Duration | Distance | Bracketed | Before -> After |",
            "| --- | ---: | ---: | :---: | --- |",
        ]
    )
    for span in aggregates["largest_no_solution_spans"]:  # type: ignore[index]
        lines.append(
            "| "
            + " | ".join(
                [
                    f"{run_label(str(span['run_key']))} {float(span['start_tow_s']):.1f}-{float(span['end_tow_s']):.1f}",
                    f"{float(span['duration_s']):.1f} s",
                    fmt_m(span["distance_m"]),
                    "yes" if span["bracketed_by_solution"] else "no",
                    f"{span['before_state'] or 'none'} -> {span['after_state'] or 'none'}",
                ]
            )
            + " |"
        )
    lines.append("")
    return "\n".join(lines)


def render_png(payload: dict[str, object], output: Path, title: str) -> None:
    import matplotlib.pyplot as plt

    BG = "#f7f8fb"
    TEXT = "#172033"
    MUTED = "#667085"
    GRID = "#d0d5dd"
    LINE = "#0f766e"
    NO_SOLUTION = "#2563eb"
    HIGH_ERROR = "#d97706"

    aggregates = payload["aggregates"]  # type: ignore[index]
    distances = aggregates["score_state_distances_m"]  # type: ignore[index]
    rows = aggregates["bridge_thresholds"]  # type: ignore[index]
    high_error_rows = aggregates["high_error_by_status"]  # type: ignore[index]

    x = [float(row["max_gap_s"]) for row in rows]
    score_pct = [float(row["score_pct"]) for row in rows]
    recovered = [float(row["recovered_no_solution_distance_m"]) for row in rows]
    labels = [str(row["status_name"]) for row in high_error_rows]
    high_error_distances = [float(row["distance_m"]) for row in high_error_rows]

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
    fig.text(0.06, 0.92, title, fontsize=22, weight="bold", color=TEXT)
    fig.text(
        0.06,
        0.875,
        "Upper bound: bracketed no-solution spans bridged within the 0.5 m PPC threshold",
        fontsize=11,
        color=MUTED,
    )

    best_5 = threshold_row(rows, 5.0)
    best_30 = threshold_row(rows, 30.0)
    cards = [
        ("Baseline", fmt_pct(aggregates["baseline_score_pct"]), f"scored {fmt_m(distances['scored'])}"),
        ("<= 5 s bridge", fmt_pct(best_5["score_pct"]), f"+{fmt_m(best_5['score_delta_distance_m'])}"),
        ("<= 30 s bridge", fmt_pct(best_30["score_pct"]), f"+{fmt_m(best_30['score_delta_distance_m'])}"),
    ]
    for index, (label, value, detail) in enumerate(cards):
        ax = fig.add_subplot(grid[0, index])
        ax.set_facecolor("white")
        for spine in ax.spines.values():
            spine.set_edgecolor(GRID)
        ax.set_xticks([])
        ax.set_yticks([])
        ax.text(0.05, 0.72, label, transform=ax.transAxes, fontsize=10.5, color=MUTED, weight="bold")
        ax.text(0.05, 0.35, value, transform=ax.transAxes, fontsize=22, color=LINE, weight="bold")
        ax.text(0.05, 0.16, detail, transform=ax.transAxes, fontsize=9.3, color=MUTED)

    ax_score = fig.add_subplot(grid[1, :2])
    ax_score.plot(x, score_pct, marker="o", linewidth=2.4, color=LINE)
    ax_score.fill_between(x, [float(aggregates["baseline_score_pct"])] * len(x), score_pct, color=LINE, alpha=0.13)
    ax_score.set_xscale("log")
    ax_score.set_xlabel("Maximum bridged no-solution span (s)", fontsize=9.5, color=MUTED)
    ax_score.set_ylabel("PPC official score upper bound (%)", fontsize=9.5, color=MUTED)
    ax_score.set_title("Loose IMU bridge upper bound", fontsize=13.5, weight="bold", color=TEXT)
    ax_score.grid(alpha=0.25, color=GRID)
    ax_score.tick_params(labelsize=8.5, colors=MUTED)
    for gap_s, pct, recovered_m in zip(x, score_pct, recovered):
        if gap_s in {1.0, 5.0, 30.0, 120.0}:
            ax_score.text(
                gap_s,
                pct + 0.16,
                f"+{recovered_m / 1000.0:.1f} km",
                ha="center",
                fontsize=8.2,
                color=TEXT,
            )
    for spine in ax_score.spines.values():
        spine.set_alpha(0.18)

    ax_high = fig.add_subplot(grid[1, 2])
    y = list(range(len(labels)))
    ax_high.barh(y, high_error_distances, color=HIGH_ERROR)
    ax_high.set_yticks(y)
    ax_high.set_yticklabels(labels, fontsize=8.8)
    ax_high.invert_yaxis()
    ax_high.set_xlabel("High-error distance (m)", fontsize=9.5, color=MUTED)
    ax_high.set_title("Robust update target", fontsize=13.5, weight="bold", color=TEXT)
    ax_high.grid(axis="x", alpha=0.25, color=GRID)
    ax_high.tick_params(axis="x", labelsize=8.5, colors=MUTED)
    xmax = max(high_error_distances) if high_error_distances else 1.0
    for index, distance_m in enumerate(high_error_distances):
        ax_high.text(
            distance_m + xmax * 0.025,
            index,
            f"{distance_m / 1000.0:.1f} km",
            va="center",
            fontsize=8.3,
            color=TEXT,
        )
    ax_high.set_xlim(0.0, xmax * 1.18)
    for spine in ax_high.spines.values():
        spine.set_alpha(0.18)

    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=180)
    plt.close(fig)


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--segment-csv-template",
        required=True,
        help="Template for official segment CSVs, e.g. output/quality/{key}_official_segments.csv",
    )
    parser.add_argument(
        "--run",
        action="append",
        type=parse_run,
        dest="runs",
        help="Run to include as CITY/RUN. Defaults to all six PPC runs.",
    )
    parser.add_argument(
        "--max-gap-s",
        action="append",
        type=float,
        dest="thresholds_s",
        help="Bridge threshold to report. Can be repeated.",
    )
    parser.add_argument(
        "--allow-unbracketed",
        action="store_true",
        help="Also count no-solution spans without solution anchors on both sides.",
    )
    parser.add_argument("--summary-json", type=Path)
    parser.add_argument("--markdown-output", type=Path)
    parser.add_argument("--output-png", type=Path)
    parser.add_argument("--title", default="PPC IMU bridge target upper bound")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(sys.argv[1:] if argv is None else argv)
    payload = build_payload(
        args.segment_csv_template,
        args.runs or default_run_specs(),
        args.thresholds_s or list(DEFAULT_GAP_THRESHOLDS_S),
        require_bracketed_solution=not args.allow_unbracketed,
    )
    if args.summary_json:
        args.summary_json.parent.mkdir(parents=True, exist_ok=True)
        args.summary_json.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    if args.markdown_output:
        args.markdown_output.parent.mkdir(parents=True, exist_ok=True)
        args.markdown_output.write_text(render_markdown(payload), encoding="utf-8")
    if args.output_png:
        render_png(payload, args.output_png, args.title)
    if not args.summary_json and not args.markdown_output and not args.output_png:
        print(render_markdown(payload))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
