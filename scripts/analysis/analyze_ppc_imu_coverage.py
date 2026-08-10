#!/usr/bin/env python3
"""Summarize PPC IMU timing coverage and current official-score loss pools."""

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

TIME_CANDIDATES = (
    "gpstows",
    "gpstow",
    "gpssecondsofweek",
    "gpssecondsweek",
    "tow",
    "tows",
    "time",
    "times",
    "timestamp",
)
WEEK_CANDIDATES = ("gpsweek", "week")
ACCEL_CANDIDATES = {
    "x": ("accxms2", "accx", "accelx", "accelerationx", "ax"),
    "y": ("accyms2", "accy", "accely", "accelerationy", "ay"),
    "z": ("acczms2", "accz", "accelz", "accelerationz", "az"),
}
GYRO_CANDIDATES = {
    "x": (
        "angratexdegs",
        "angratex",
        "angularratexdegs",
        "angularratex",
        "gyroxdegs",
        "gyrox",
        "gyrx",
        "wx",
    ),
    "y": (
        "angrateydegs",
        "angratey",
        "angularrateydegs",
        "angularratey",
        "gyroydegs",
        "gyroy",
        "gyry",
        "wy",
    ),
    "z": (
        "angratezdegs",
        "angratez",
        "angularratezdegs",
        "angularratez",
        "gyrozdegs",
        "gyroz",
        "gyrz",
        "wz",
    ),
}


@dataclass(frozen=True)
class RunSpec:
    city: str
    run: str

    @property
    def key(self) -> str:
        return f"{self.city}_{self.run}"

    @property
    def relpath(self) -> str:
        return f"{self.city}/{self.run}"


@dataclass
class RunningStats:
    count: int = 0
    mean: float = 0.0
    m2: float = 0.0
    minimum: float = math.inf
    maximum: float = -math.inf

    def add(self, value: float) -> None:
        if not math.isfinite(value):
            return
        self.count += 1
        delta = value - self.mean
        self.mean += delta / self.count
        self.m2 += delta * (value - self.mean)
        self.minimum = min(self.minimum, value)
        self.maximum = max(self.maximum, value)

    def as_dict(self) -> dict[str, float | int]:
        if self.count == 0:
            return {
                "count": 0,
                "mean": 0.0,
                "std": 0.0,
                "min": 0.0,
                "max": 0.0,
            }
        variance = self.m2 / max(self.count - 1, 1)
        return {
            "count": self.count,
            "mean": rounded(self.mean),
            "std": rounded(math.sqrt(max(variance, 0.0))),
            "min": rounded(self.minimum),
            "max": rounded(self.maximum),
        }


def rounded(value: float, digits: int = 6) -> float:
    return round(float(value), digits)


def normalize_header(name: str) -> str:
    return "".join(ch for ch in name.strip().lower() if ch.isalnum())


def field_lookup(fieldnames: list[str] | None) -> dict[str, str]:
    if not fieldnames:
        return {}
    return {normalize_header(name): name for name in fieldnames}


def find_column(lookup: dict[str, str], candidates: tuple[str, ...], context: str) -> str:
    for candidate in candidates:
        if candidate in lookup:
            return lookup[candidate]
    raise SystemExit(f"{context}: could not find any of {', '.join(candidates)}")


def maybe_column(lookup: dict[str, str], candidates: tuple[str, ...]) -> str | None:
    for candidate in candidates:
        if candidate in lookup:
            return lookup[candidate]
    return None


def parse_float(value: str | None, context: str) -> float:
    if value is None:
        raise SystemExit(f"{context}: missing numeric value")
    try:
        return float(value.strip())
    except ValueError as exc:
        raise SystemExit(f"{context}: invalid numeric value {value!r}") from exc


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


def summarize_time_series(path: Path, *, gap_multiple: float) -> dict[str, object]:
    if not path.exists():
        raise SystemExit(f"missing CSV: {path}")
    with path.open("r", encoding="utf-8", newline="") as handle:
        reader = csv.DictReader(handle)
        lookup = field_lookup(reader.fieldnames)
        time_column = find_column(lookup, TIME_CANDIDATES, str(path))
        week_column = maybe_column(lookup, WEEK_CANDIDATES)

        times: list[float] = []
        weeks: set[int] = set()
        for row_index, row in enumerate(reader, start=2):
            times.append(parse_float(row.get(time_column), f"{path}:{row_index}:{time_column}"))
            if week_column is not None:
                week_text = row.get(week_column)
                if week_text is not None and week_text.strip():
                    weeks.add(int(float(week_text.strip())))

    if not times:
        raise SystemExit(f"{path}: no data rows")

    deltas = [next_time - time for time, next_time in zip(times, times[1:])]
    positive_deltas = [delta for delta in deltas if delta > 0.0]
    duplicate_or_reverse_count = len(deltas) - len(positive_deltas)
    median_dt_s = percentile(positive_deltas, 50)
    mean_dt_s = (
        rounded(sum(positive_deltas) / len(positive_deltas))
        if positive_deltas
        else 0.0
    )
    gap_threshold_s = gap_multiple * median_dt_s if median_dt_s > 0.0 else math.inf
    gap_deltas = [delta for delta in positive_deltas if delta > gap_threshold_s]

    return {
        "path": str(path),
        "row_count": len(times),
        "time_column": time_column.strip(),
        "week_column": week_column.strip() if week_column else None,
        "gps_weeks": sorted(weeks),
        "start_tow_s": rounded(min(times)),
        "end_tow_s": rounded(max(times)),
        "duration_s": rounded(max(times) - min(times)),
        "median_dt_s": rounded(median_dt_s),
        "mean_dt_s": mean_dt_s,
        "median_rate_hz": rounded(1.0 / median_dt_s, 3) if median_dt_s > 0.0 else 0.0,
        "max_gap_s": rounded(max(positive_deltas) if positive_deltas else 0.0),
        "gap_threshold_s": rounded(gap_threshold_s) if math.isfinite(gap_threshold_s) else 0.0,
        "gap_count": len(gap_deltas),
        "duplicate_or_reverse_time_count": duplicate_or_reverse_count,
    }


def summarize_imu(path: Path, *, gap_multiple: float) -> dict[str, object]:
    timing = summarize_time_series(path, gap_multiple=gap_multiple)
    with path.open("r", encoding="utf-8", newline="") as handle:
        reader = csv.DictReader(handle)
        lookup = field_lookup(reader.fieldnames)
        accel_columns = {
            axis: find_column(lookup, candidates, f"{path}: accel {axis}")
            for axis, candidates in ACCEL_CANDIDATES.items()
        }
        gyro_columns = {
            axis: find_column(lookup, candidates, f"{path}: gyro {axis}")
            for axis, candidates in GYRO_CANDIDATES.items()
        }

        accel_stats = {axis: RunningStats() for axis in accel_columns}
        gyro_stats = {axis: RunningStats() for axis in gyro_columns}
        accel_norm = RunningStats()
        gyro_norm = RunningStats()

        for row_index, row in enumerate(reader, start=2):
            accel_values: dict[str, float] = {}
            gyro_values: dict[str, float] = {}
            for axis, column in accel_columns.items():
                value = parse_float(row.get(column), f"{path}:{row_index}:{column}")
                accel_values[axis] = value
                accel_stats[axis].add(value)
            for axis, column in gyro_columns.items():
                value = parse_float(row.get(column), f"{path}:{row_index}:{column}")
                gyro_values[axis] = value
                gyro_stats[axis].add(value)
            accel_norm.add(
                math.sqrt(sum(value * value for value in accel_values.values()))
            )
            gyro_norm.add(math.sqrt(sum(value * value for value in gyro_values.values())))

    timing.update(
        {
            "accel_columns": {axis: column.strip() for axis, column in accel_columns.items()},
            "gyro_columns": {axis: column.strip() for axis, column in gyro_columns.items()},
            "accel_stats": {
                axis: stats.as_dict() for axis, stats in accel_stats.items()
            },
            "gyro_stats": {axis: stats.as_dict() for axis, stats in gyro_stats.items()},
            "accel_norm_mps2": accel_norm.as_dict(),
            "gyro_norm_degps": gyro_norm.as_dict(),
        }
    )
    return timing


def reference_overlap(imu: dict[str, object], reference: dict[str, object]) -> dict[str, object]:
    imu_start = float(imu["start_tow_s"])
    imu_end = float(imu["end_tow_s"])
    ref_start = float(reference["start_tow_s"])
    ref_end = float(reference["end_tow_s"])
    overlap_start = max(imu_start, ref_start)
    overlap_end = min(imu_end, ref_end)
    overlap_duration = max(0.0, overlap_end - overlap_start)
    ref_duration = max(ref_end - ref_start, 0.0)
    imu_duration = max(imu_end - imu_start, 0.0)
    return {
        "start_offset_s": rounded(imu_start - ref_start),
        "end_offset_s": rounded(imu_end - ref_end),
        "overlap_start_tow_s": rounded(overlap_start),
        "overlap_end_tow_s": rounded(overlap_end),
        "overlap_duration_s": rounded(overlap_duration),
        "reference_overlap_pct": rounded(100.0 * overlap_duration / ref_duration)
        if ref_duration > 0.0
        else 0.0,
        "imu_overlap_pct": rounded(100.0 * overlap_duration / imu_duration)
        if imu_duration > 0.0
        else 0.0,
    }


def load_quality_summary(path: Path | None) -> dict[str, object] | None:
    if path is None or not path.exists():
        return None
    payload = json.loads(path.read_text(encoding="utf-8"))
    rows = payload.get("official_loss_by_state")
    if not isinstance(rows, list):
        raise SystemExit(f"{path}: missing official_loss_by_state")
    distances = {"scored": 0.0, "high_error": 0.0, "no_solution": 0.0}
    for row in rows:
        if not isinstance(row, dict):
            continue
        state = str(row.get("score_state", ""))
        if state in distances:
            distances[state] = float(row.get("distance_m", 0.0))
    total = sum(distances.values())
    return {
        "path": str(path),
        "scored_distance_m": rounded(distances["scored"]),
        "high_error_distance_m": rounded(distances["high_error"]),
        "no_solution_distance_m": rounded(distances["no_solution"]),
        "loss_distance_m": rounded(distances["high_error"] + distances["no_solution"]),
        "total_distance_m": rounded(total),
        "official_score_pct": rounded(100.0 * distances["scored"] / total)
        if total > 0.0
        else 0.0,
    }


def parse_run(value: str) -> RunSpec:
    city, separator, run = value.partition("/")
    if not separator or not city or not run:
        raise argparse.ArgumentTypeError("--run must use CITY/RUN")
    return RunSpec(city=city, run=run)


def default_run_specs() -> list[RunSpec]:
    return [RunSpec(city, run) for city, run in DEFAULT_RUNS]


def format_template(template: str | None, run: RunSpec) -> Path | None:
    if template is None:
        return None
    return Path(template.format(city=run.city, run=run.run, key=run.key))


def summarize_run(
    dataset_root: Path,
    run: RunSpec,
    *,
    quality_json_template: str | None,
    gap_multiple: float,
    readiness_min_overlap_pct: float,
    readiness_min_rate_hz: float,
    readiness_max_gap_s: float,
) -> dict[str, object]:
    run_root = dataset_root / run.city / run.run
    imu = summarize_imu(run_root / "imu.csv", gap_multiple=gap_multiple)
    reference = summarize_time_series(run_root / "reference.csv", gap_multiple=gap_multiple)
    overlap = reference_overlap(imu, reference)
    quality = load_quality_summary(format_template(quality_json_template, run))
    ready = (
        float(overlap["reference_overlap_pct"]) >= readiness_min_overlap_pct
        and float(imu["median_rate_hz"]) >= readiness_min_rate_hz
        and float(imu["max_gap_s"]) <= readiness_max_gap_s
        and int(imu["gap_count"]) == 0
    )

    return {
        "key": run.key,
        "city": run.city,
        "run": run.run,
        "imu": imu,
        "reference": reference,
        "reference_overlap": overlap,
        "quality": quality,
        "ready_for_loose_kalman_bridge": ready,
    }


def aggregate_runs(runs: list[dict[str, object]], target_score_pct: float | None) -> dict[str, object]:
    imu_rates = [float(run["imu"]["median_rate_hz"]) for run in runs]  # type: ignore[index]
    overlaps = [
        float(run["reference_overlap"]["reference_overlap_pct"]) for run in runs  # type: ignore[index]
    ]
    max_gaps = [float(run["imu"]["max_gap_s"]) for run in runs]  # type: ignore[index]
    quality_rows = [run.get("quality") for run in runs if run.get("quality")]
    scored = sum(float(row["scored_distance_m"]) for row in quality_rows)  # type: ignore[index]
    high_error = sum(float(row["high_error_distance_m"]) for row in quality_rows)  # type: ignore[index]
    no_solution = sum(float(row["no_solution_distance_m"]) for row in quality_rows)  # type: ignore[index]
    total = scored + high_error + no_solution
    aggregates: dict[str, object] = {
        "run_count": len(runs),
        "ready_run_count": sum(1 for run in runs if run["ready_for_loose_kalman_bridge"]),
        "total_imu_samples": sum(int(run["imu"]["row_count"]) for run in runs),  # type: ignore[index]
        "min_imu_rate_hz": rounded(min(imu_rates), 3) if imu_rates else 0.0,
        "median_imu_rate_hz": rounded(percentile(imu_rates, 50), 3) if imu_rates else 0.0,
        "max_imu_gap_s": rounded(max(max_gaps)) if max_gaps else 0.0,
        "min_reference_overlap_pct": rounded(min(overlaps)) if overlaps else 0.0,
        "quality_run_count": len(quality_rows),
    }
    if quality_rows:
        aggregates.update(
            {
                "scored_distance_m": rounded(scored),
                "high_error_distance_m": rounded(high_error),
                "no_solution_distance_m": rounded(no_solution),
                "loss_distance_m": rounded(high_error + no_solution),
                "official_score_pct": rounded(100.0 * scored / total) if total > 0.0 else 0.0,
                "total_distance_m": rounded(total),
            }
        )
        if target_score_pct is not None:
            target_distance = total * target_score_pct / 100.0
            target_gap = max(0.0, target_distance - scored)
            aggregates.update(
                {
                    "target_score_pct": rounded(target_score_pct),
                    "target_scored_distance_m": rounded(target_distance),
                    "target_gap_distance_m": rounded(target_gap),
                    "no_solution_share_of_target_gap_pct": rounded(
                        100.0 * no_solution / target_gap
                    )
                    if target_gap > 0.0
                    else 0.0,
                    "loss_share_of_target_gap_pct": rounded(
                        100.0 * (high_error + no_solution) / target_gap
                    )
                    if target_gap > 0.0
                    else 0.0,
                }
            )
    return aggregates


def build_payload(
    dataset_root: Path,
    runs: list[RunSpec],
    *,
    quality_json_template: str | None = None,
    target_score_pct: float | None = None,
    gap_multiple: float = 2.5,
    readiness_min_overlap_pct: float = 99.9,
    readiness_min_rate_hz: float = 50.0,
    readiness_max_gap_s: float = 0.05,
) -> dict[str, object]:
    run_payloads = [
        summarize_run(
            dataset_root,
            run,
            quality_json_template=quality_json_template,
            gap_multiple=gap_multiple,
            readiness_min_overlap_pct=readiness_min_overlap_pct,
            readiness_min_rate_hz=readiness_min_rate_hz,
            readiness_max_gap_s=readiness_max_gap_s,
        )
        for run in runs
    ]
    return {
        "dataset_root": str(dataset_root),
        "target_score_pct": target_score_pct,
        "runs": run_payloads,
        "aggregates": aggregate_runs(run_payloads, target_score_pct),
    }


def run_label(key: str) -> str:
    city, _, run = key.partition("_")
    return f"{city.capitalize()} {run.replace('run', 'r')}" if run else key


def fmt_m(value: object) -> str:
    return f"{float(value):,.1f} m"


def fmt_pct(value: object) -> str:
    return f"{float(value):.2f}%"


def render_markdown(payload: dict[str, object]) -> str:
    aggregates = payload["aggregates"]  # type: ignore[index]
    lines = [
        "# PPC IMU Coverage",
        "",
        (
            f"Ready runs: **{aggregates['ready_run_count']} / {aggregates['run_count']}**; "
            f"median IMU rate **{aggregates['median_imu_rate_hz']:.3f} Hz**, "
            f"minimum reference overlap **{aggregates['min_reference_overlap_pct']:.3f}%**, "
            f"maximum IMU gap **{aggregates['max_imu_gap_s']:.3f} s**."
        ),
    ]
    if int(aggregates.get("quality_run_count", 0)) > 0:
        lines.extend(
            [
                "",
                (
                    f"Current scored distance is **{fmt_m(aggregates['scored_distance_m'])}** "
                    f"(**{fmt_pct(aggregates['official_score_pct'])}**). "
                    f"The remaining loss pool is **{fmt_m(aggregates['loss_distance_m'])}**: "
                    f"high-error **{fmt_m(aggregates['high_error_distance_m'])}** plus "
                    f"no-solution **{fmt_m(aggregates['no_solution_distance_m'])}**."
                ),
            ]
        )
        if "target_gap_distance_m" in aggregates:
            lines.append(
                f"Gap to **{fmt_pct(aggregates['target_score_pct'])}** is "
                f"**{fmt_m(aggregates['target_gap_distance_m'])}**; no-solution distance alone "
                f"covers **{fmt_pct(aggregates['no_solution_share_of_target_gap_pct'])}** of that gap."
            )

    lines.extend(
        [
            "",
            "| Run | IMU Hz | IMU rows | Ref overlap | Max gap | No solution | High error | Ready |",
            "| --- | ---: | ---: | ---: | ---: | ---: | ---: | :---: |",
        ]
    )
    for run in payload["runs"]:  # type: ignore[index]
        imu = run["imu"]
        overlap = run["reference_overlap"]
        quality = run.get("quality")
        lines.append(
            "| "
            + " | ".join(
                [
                    run_label(str(run["key"])),
                    f"{float(imu['median_rate_hz']):.3f}",
                    f"{int(imu['row_count']):,}",
                    fmt_pct(overlap["reference_overlap_pct"]),
                    f"{float(imu['max_gap_s']):.3f} s",
                    fmt_m(quality["no_solution_distance_m"]) if quality else "n/a",
                    fmt_m(quality["high_error_distance_m"]) if quality else "n/a",
                    "yes" if run["ready_for_loose_kalman_bridge"] else "no",
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
    READY = "#0f766e"
    HIGH_ERROR = "#d97706"
    NO_SOLUTION = "#2563eb"

    aggregates = payload["aggregates"]  # type: ignore[index]
    runs = payload["runs"]  # type: ignore[index]
    labels = [run_label(str(run["key"])) for run in runs]
    no_solution = [
        float(run["quality"]["no_solution_distance_m"]) if run.get("quality") else 0.0
        for run in runs
    ]
    high_error = [
        float(run["quality"]["high_error_distance_m"]) if run.get("quality") else 0.0
        for run in runs
    ]
    sample_counts = [int(run["imu"]["row_count"]) for run in runs]

    fig = plt.figure(figsize=(13.0, 7.5), facecolor=BG)
    grid = fig.add_gridspec(
        2,
        3,
        left=0.06,
        right=0.97,
        top=0.84,
        bottom=0.10,
        height_ratios=[0.75, 1.65],
        width_ratios=[1.0, 1.0, 1.0],
        hspace=0.45,
        wspace=0.28,
    )
    fig.text(0.06, 0.92, title, fontsize=22, weight="bold", color=TEXT)
    fig.text(
        0.06,
        0.875,
        "PPC public IMU timing coverage and reset10 official-score loss pools",
        fontsize=11,
        color=MUTED,
    )

    card_values = [
        (
            "Ready runs",
            f"{aggregates['ready_run_count']} / {aggregates['run_count']}",
            "100 Hz IMU aligned with reference",
        ),
        (
            "Min reference overlap",
            fmt_pct(aggregates["min_reference_overlap_pct"]),
            f"max IMU gap {float(aggregates['max_imu_gap_s']):.3f} s",
        ),
        (
            "Gap to target",
            fmt_m(aggregates.get("target_gap_distance_m", 0.0)),
            f"target {fmt_pct(aggregates.get('target_score_pct', 0.0))}",
        ),
    ]
    for index, (label, value, detail) in enumerate(card_values):
        ax = fig.add_subplot(grid[0, index])
        ax.set_facecolor("white")
        for spine in ax.spines.values():
            spine.set_edgecolor(GRID)
        ax.set_xticks([])
        ax.set_yticks([])
        ax.text(0.05, 0.72, label, transform=ax.transAxes, fontsize=10.5, color=MUTED, weight="bold")
        ax.text(0.05, 0.35, value, transform=ax.transAxes, fontsize=22, color=READY, weight="bold")
        ax.text(0.05, 0.16, detail, transform=ax.transAxes, fontsize=9.3, color=MUTED)

    ax_loss = fig.add_subplot(grid[1, :2])
    x = list(range(len(labels)))
    ax_loss.bar(x, no_solution, color=NO_SOLUTION, label="No solution")
    ax_loss.bar(x, high_error, bottom=no_solution, color=HIGH_ERROR, label="High error")
    ax_loss.set_xticks(x)
    ax_loss.set_xticklabels(labels, rotation=20, ha="right", fontsize=9)
    ax_loss.set_ylabel("Unscored official distance (m)", fontsize=9.5, color=MUTED)
    ax_loss.set_title("Loss pool that IMU fusion must attack", fontsize=13.5, weight="bold", color=TEXT)
    ax_loss.grid(axis="y", alpha=0.25, color=GRID)
    ax_loss.legend(loc="upper right", frameon=False, fontsize=9)
    ax_loss.tick_params(axis="y", labelsize=8.5, colors=MUTED)
    for spine in ax_loss.spines.values():
        spine.set_alpha(0.18)

    ax_timing = fig.add_subplot(grid[1, 2])
    y = list(range(len(labels)))
    ax_timing.barh(y, sample_counts, color=READY)
    ax_timing.set_yticks(y)
    ax_timing.set_yticklabels(labels, fontsize=8.6)
    ax_timing.invert_yaxis()
    ax_timing.set_xlabel("IMU samples", fontsize=9.5, color=MUTED)
    ax_timing.set_title("100 Hz input coverage", fontsize=13.5, weight="bold", color=TEXT)
    ax_timing.grid(axis="x", alpha=0.25, color=GRID)
    ax_timing.tick_params(axis="x", labelsize=8.5, colors=MUTED)
    for index, count in enumerate(sample_counts):
        ax_timing.text(
            count + max(sample_counts) * 0.015,
            index,
            f"{count / 1000.0:.0f}k",
            va="center",
            fontsize=8.3,
            color=TEXT,
        )
    ax_timing.set_xlim(0.0, max(sample_counts) * 1.18)
    for spine in ax_timing.spines.values():
        spine.set_alpha(0.18)

    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=180)
    plt.close(fig)


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset-root", type=Path, required=True)
    parser.add_argument(
        "--run",
        action="append",
        type=parse_run,
        dest="runs",
        help="Run to include as CITY/RUN. Defaults to all six PPC runs.",
    )
    parser.add_argument(
        "--quality-json-template",
        help="Optional current-quality JSON template, e.g. output/quality/{key}.json",
    )
    parser.add_argument("--target-score-pct", type=float, default=None)
    parser.add_argument("--gap-multiple", type=float, default=2.5)
    parser.add_argument("--readiness-min-overlap-pct", type=float, default=99.9)
    parser.add_argument("--readiness-min-rate-hz", type=float, default=50.0)
    parser.add_argument("--readiness-max-gap-s", type=float, default=0.05)
    parser.add_argument("--summary-json", type=Path)
    parser.add_argument("--markdown-output", type=Path)
    parser.add_argument("--output-png", type=Path)
    parser.add_argument("--title", default="PPC IMU fusion readiness")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(sys.argv[1:] if argv is None else argv)
    payload = build_payload(
        args.dataset_root,
        args.runs or default_run_specs(),
        quality_json_template=args.quality_json_template,
        target_score_pct=args.target_score_pct,
        gap_multiple=args.gap_multiple,
        readiness_min_overlap_pct=args.readiness_min_overlap_pct,
        readiness_min_rate_hz=args.readiness_min_rate_hz,
        readiness_max_gap_s=args.readiness_max_gap_s,
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
