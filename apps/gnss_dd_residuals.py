#!/usr/bin/env python3
"""Summarize per-DD RTK residual diagnostics emitted by gnss solve."""

from __future__ import annotations

import argparse
import csv
import html
import json
import math
import os
from pathlib import Path
from typing import Any


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(prog=os.environ.get("GNSS_CLI_NAME"))
    parser.add_argument("input_csv", type=Path, help="Input --dd-residuals-csv file.")
    parser.add_argument("--summary-json", type=Path, default=None, help="Optional summary JSON path.")
    parser.add_argument("--top-pairs-csv", type=Path, default=None, help="Optional worst-pair CSV path.")
    parser.add_argument("--html-report", type=Path, default=None, help="Optional self-contained HTML report path.")
    parser.add_argument("--top-n", type=int, default=10, help="Number of worst pairs to keep.")
    parser.add_argument(
        "--kind",
        choices=("all", "phase", "code"),
        default="all",
        help="Filter measurements before summarizing.",
    )
    parser.add_argument(
        "--frequency-index",
        type=int,
        default=None,
        help="Filter one frequency index before summarizing.",
    )
    parser.add_argument(
        "--require-phase-p95-max",
        type=float,
        default=None,
        help="Fail if phase p95 absolute residual exceeds this value in meters.",
    )
    parser.add_argument(
        "--require-code-p95-max",
        type=float,
        default=None,
        help="Fail if code p95 absolute residual exceeds this value in meters.",
    )
    parser.add_argument(
        "--require-max-abs-residual-max",
        type=float,
        default=None,
        help="Fail if max absolute residual exceeds this value in meters.",
    )
    parser.add_argument(
        "--require-suppressed-rows-max",
        type=int,
        default=None,
        help="Fail if outlier-threshold suppressed row count exceeds this value.",
    )
    return parser.parse_args()


def rounded(value: float | None) -> float | None:
    if value is None:
        return None
    return round(value, 6)


def percentile(values: list[float], q: float) -> float | None:
    if not values:
        return None
    sorted_values = sorted(values)
    if len(sorted_values) == 1:
        return sorted_values[0]
    rank = q * (len(sorted_values) - 1)
    lower = math.floor(rank)
    upper = math.ceil(rank)
    if lower == upper:
        return sorted_values[lower]
    fraction = rank - lower
    return sorted_values[lower] * (1.0 - fraction) + sorted_values[upper] * fraction


def frequency_label(frequency_index: int) -> str:
    labels = {
        0: "freq0 primary (L1/E1/B1)",
        1: "freq1 secondary (L2/E5/B2)",
        2: "freq2 L5-class (L5/E5a/B2a)",
    }
    return labels.get(frequency_index, f"freq{frequency_index}")


def pair_label(reference_satellite: Any, satellite: Any) -> str:
    return f"{reference_satellite}-{satellite}"


def residual_sigma_m(record: dict[str, Any]) -> float | None:
    reference_variance = record.get("reference_variance_m2")
    satellite_variance = record.get("satellite_variance_m2")
    if reference_variance is None or satellite_variance is None:
        return None
    variance = float(reference_variance) + float(satellite_variance)
    if not math.isfinite(variance) or variance <= 0.0:
        return None
    return math.sqrt(variance)


def normalized_residual(record: dict[str, Any]) -> float | None:
    sigma = residual_sigma_m(record)
    if sigma is None:
        return None
    return float(record["residual_m"]) / sigma


def abs_normalized_residual(record: dict[str, Any]) -> float | None:
    value = normalized_residual(record)
    return abs(value) if value is not None else None


def read_records(path: Path) -> list[dict[str, Any]]:
    records: list[dict[str, Any]] = []
    with path.open(encoding="ascii", newline="") as handle:
        reader = csv.DictReader(handle)
        required = {
            "epoch_index",
            "gps_week",
            "tow",
            "status",
            "kind",
            "frequency_index",
            "reference_satellite",
            "satellite",
            "residual_m",
            "abs_residual_m",
            "suppressed_by_outlier_threshold",
        }
        missing = required.difference(reader.fieldnames or [])
        if missing:
            raise SystemExit(f"Missing DD residual columns in {path}: {', '.join(sorted(missing))}")
        for row in reader:
            residual_m = float(row["residual_m"])
            abs_residual_m = float(row["abs_residual_m"])
            if not math.isfinite(residual_m) or not math.isfinite(abs_residual_m):
                continue
            records.append(
                {
                    "epoch_index": int(float(row["epoch_index"])),
                    "gps_week": int(float(row["gps_week"])),
                    "tow": float(row["tow"]),
                    "status": int(float(row["status"])),
                    "kind": row["kind"],
                    "frequency_index": int(float(row["frequency_index"])),
                    "reference_satellite": row["reference_satellite"],
                    "satellite": row["satellite"],
                    "residual_m": residual_m,
                    "abs_residual_m": abs_residual_m,
                    "reference_variance_m2": _optional_float(row.get("reference_variance_m2")),
                    "satellite_variance_m2": _optional_float(row.get("satellite_variance_m2")),
                    "suppressed_by_outlier_threshold": int(
                        float(row["suppressed_by_outlier_threshold"])
                    ),
                }
            )
    return records


def _optional_float(value: str | None) -> float | None:
    if value is None or value == "":
        return None
    parsed = float(value)
    return parsed if math.isfinite(parsed) else None


def filter_records(
    records: list[dict[str, Any]],
    kind: str = "all",
    frequency_index: int | None = None,
) -> list[dict[str, Any]]:
    filtered = records
    if kind != "all":
        filtered = [record for record in filtered if record["kind"] == kind]
    if frequency_index is not None:
        filtered = [
            record for record in filtered if int(record["frequency_index"]) == frequency_index
        ]
    return filtered


def residual_stats(records: list[dict[str, Any]]) -> dict[str, Any]:
    residuals = [float(record["residual_m"]) for record in records]
    abs_residuals = [float(record["abs_residual_m"]) for record in records]
    normalized_residuals = [
        value
        for value in (normalized_residual(record) for record in records)
        if value is not None
    ]
    abs_normalized_residuals = [abs(value) for value in normalized_residuals]
    suppressed_rows = sum(int(record["suppressed_by_outlier_threshold"]) for record in records)
    worst = max(records, key=lambda record: float(record["abs_residual_m"])) if records else None
    worst_normalized = max(
        (record for record in records if abs_normalized_residual(record) is not None),
        key=lambda record: float(abs_normalized_residual(record) or 0.0),
        default=None,
    )
    return {
        "rows": len(records),
        "epochs": len({(int(record["gps_week"]), float(record["tow"])) for record in records}),
        "suppressed_rows": suppressed_rows,
        "mean_abs_residual_m": rounded(
            sum(abs_residuals) / len(abs_residuals) if abs_residuals else None
        ),
        "rms_residual_m": rounded(
            math.sqrt(sum(value * value for value in residuals) / len(residuals))
            if residuals
            else None
        ),
        "rms_normalized_residual": rounded(
            math.sqrt(
                sum(value * value for value in normalized_residuals)
                / len(normalized_residuals)
            )
            if normalized_residuals
            else None
        ),
        "p95_abs_residual_m": rounded(percentile(abs_residuals, 0.95)),
        "p95_abs_normalized_residual": rounded(percentile(abs_normalized_residuals, 0.95)),
        "max_abs_residual_m": rounded(max(abs_residuals) if abs_residuals else None),
        "max_abs_normalized_residual": rounded(
            max(abs_normalized_residuals) if abs_normalized_residuals else None
        ),
        "worst_row": compact_row(worst) if worst is not None else None,
        "worst_normalized_row": (
            compact_row(worst_normalized) if worst_normalized is not None else None
        ),
    }


def coverage_stats(records: list[dict[str, Any]]) -> dict[str, Any]:
    epochs = sorted({(int(record["epoch_index"]), int(record["gps_week"]), float(record["tow"])) for record in records})
    pairs = {
        pair_label(record["reference_satellite"], record["satellite"])
        for record in records
    }
    satellites = {
        str(record[name])
        for record in records
        for name in ("reference_satellite", "satellite")
    }
    tracks = {
        (
            str(record["kind"]),
            int(record["frequency_index"]),
            str(record["reference_satellite"]),
            str(record["satellite"]),
        )
        for record in records
    }
    frequencies = sorted({int(record["frequency_index"]) for record in records})
    return {
        "rows": len(records),
        "epochs": len(epochs),
        "first_epoch_index": epochs[0][0] if epochs else None,
        "last_epoch_index": epochs[-1][0] if epochs else None,
        "first_tow": rounded(epochs[0][2]) if epochs else None,
        "last_tow": rounded(epochs[-1][2]) if epochs else None,
        "satellites": len(satellites),
        "satellite_pairs": len(pairs),
        "pair_frequency_kind_tracks": len(tracks),
        "frequency_labels": [frequency_label(frequency) for frequency in frequencies],
    }


def compact_row(record: dict[str, Any]) -> dict[str, Any]:
    freq_index = int(record["frequency_index"])
    pair = pair_label(record["reference_satellite"], record["satellite"])
    sigma = residual_sigma_m(record)
    normalized = normalized_residual(record)
    return {
        "epoch_index": int(record["epoch_index"]),
        "gps_week": int(record["gps_week"]),
        "tow": rounded(float(record["tow"])),
        "status": int(record["status"]),
        "kind": record["kind"],
        "frequency_index": freq_index,
        "frequency_label": frequency_label(freq_index),
        "reference_satellite": record["reference_satellite"],
        "satellite": record["satellite"],
        "pair": pair,
        "residual_m": rounded(float(record["residual_m"])),
        "abs_residual_m": rounded(float(record["abs_residual_m"])),
        "residual_sigma_m": rounded(sigma),
        "normalized_residual": rounded(normalized),
        "abs_normalized_residual": rounded(abs(normalized)) if normalized is not None else None,
        "suppressed_by_outlier_threshold": int(record["suppressed_by_outlier_threshold"]),
    }


def group_records(records: list[dict[str, Any]], keys: tuple[str, ...]) -> dict[tuple[Any, ...], list[dict[str, Any]]]:
    groups: dict[tuple[Any, ...], list[dict[str, Any]]] = {}
    for record in records:
        key = tuple(record[name] for name in keys)
        groups.setdefault(key, []).append(record)
    return groups


def summarize_groups(
    records: list[dict[str, Any]],
    keys: tuple[str, ...],
    top_n: int | None = None,
) -> list[dict[str, Any]]:
    summaries: list[dict[str, Any]] = []
    for key, group in group_records(records, keys).items():
        entry = {keys[index]: key[index] for index in range(len(keys))}
        if "frequency_index" in entry:
            entry["frequency_label"] = frequency_label(int(entry["frequency_index"]))
        if "reference_satellite" in entry and "satellite" in entry:
            entry["pair"] = pair_label(entry["reference_satellite"], entry["satellite"])
        entry.update(residual_stats(group))
        summaries.append(entry)
    summaries.sort(
        key=lambda item: (
            float(item["p95_abs_residual_m"] or -1.0),
            float(item["max_abs_residual_m"] or -1.0),
            int(item["rows"]),
        ),
        reverse=True,
    )
    return summaries[:top_n] if top_n is not None else summaries


def summarize_time_series(records: list[dict[str, Any]]) -> list[dict[str, Any]]:
    series: list[dict[str, Any]] = []
    for key, group in group_records(records, ("epoch_index", "gps_week", "tow", "status")).items():
        phase_records = [record for record in group if record["kind"] == "phase"]
        code_records = [record for record in group if record["kind"] == "code"]
        group_stats = residual_stats(group)
        phase_stats = residual_stats(phase_records)
        code_stats = residual_stats(code_records)
        series.append(
            {
                "epoch_index": int(key[0]),
                "gps_week": int(key[1]),
                "tow": rounded(float(key[2])),
                "status": int(key[3]),
                "rows": len(group),
                "suppressed_rows": group_stats["suppressed_rows"],
                "phase_p95_abs_residual_m": phase_stats["p95_abs_residual_m"],
                "phase_max_abs_residual_m": phase_stats["max_abs_residual_m"],
                "phase_p95_abs_normalized_residual": phase_stats[
                    "p95_abs_normalized_residual"
                ],
                "phase_max_abs_normalized_residual": phase_stats[
                    "max_abs_normalized_residual"
                ],
                "code_p95_abs_residual_m": code_stats["p95_abs_residual_m"],
                "code_max_abs_residual_m": code_stats["max_abs_residual_m"],
                "code_p95_abs_normalized_residual": code_stats[
                    "p95_abs_normalized_residual"
                ],
                "code_max_abs_normalized_residual": code_stats[
                    "max_abs_normalized_residual"
                ],
                "max_abs_residual_m": group_stats["max_abs_residual_m"],
                "max_abs_normalized_residual": group_stats[
                    "max_abs_normalized_residual"
                ],
            }
        )
    series.sort(key=lambda item: (int(item["epoch_index"]), float(item["tow"] or 0.0)))
    return series


def compact_residual_points(records: list[dict[str, Any]]) -> list[dict[str, Any]]:
    points = [compact_row(record) for record in records]
    points.sort(
        key=lambda item: (
            int(item["epoch_index"]),
            float(item["tow"] or 0.0),
            str(item["kind"]),
            int(item["frequency_index"]),
            str(item["reference_satellite"]),
            str(item["satellite"]),
        )
    )
    return points


def summarize_records(records: list[dict[str, Any]], top_n: int = 10) -> dict[str, Any]:
    pair_tracks = summarize_groups(
        records,
        ("kind", "frequency_index", "reference_satellite", "satellite"),
        top_n=None,
    )
    top_pairs = summarize_groups(
        records,
        ("kind", "frequency_index", "reference_satellite", "satellite"),
        top_n=max(0, top_n),
    )
    top_normalized_pairs = sorted(
        pair_tracks,
        key=lambda item: (
            float(item["p95_abs_normalized_residual"] or -1.0),
            float(item["max_abs_normalized_residual"] or -1.0),
            int(item["rows"]),
        ),
        reverse=True,
    )[:max(0, top_n)]
    return {
        "rows": len(records),
        "coverage": coverage_stats(records),
        "overall": residual_stats(records),
        "by_kind": {
            kind: residual_stats([record for record in records if record["kind"] == kind])
            for kind in ("phase", "code")
        },
        "by_frequency": summarize_groups(records, ("frequency_index",), top_n=None),
        "pair_tracks": pair_tracks,
        "top_pairs": top_pairs,
        "top_normalized_pairs": top_normalized_pairs,
        "time_series": summarize_time_series(records),
        "residual_points": compact_residual_points(records),
    }


def write_top_pairs_csv(path: Path, top_pairs: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = [
        "rank",
        "kind",
        "frequency_index",
        "frequency_label",
        "reference_satellite",
        "satellite",
        "pair",
        "rows",
        "epochs",
        "suppressed_rows",
        "rms_residual_m",
        "p95_abs_residual_m",
        "max_abs_residual_m",
        "rms_normalized_residual",
        "p95_abs_normalized_residual",
        "max_abs_normalized_residual",
    ]
    with path.open("w", encoding="ascii", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for index, pair in enumerate(top_pairs, start=1):
            row = {field: pair.get(field) for field in fieldnames if field != "rank"}
            row["rank"] = index
            writer.writerow(row)


def _html_value(value: Any) -> str:
    if value is None:
        return "n/a"
    return html.escape(str(value))


def _metric_cell(label: str, value: Any) -> str:
    return (
        "<div class=\"metric\">"
        f"<div class=\"metric-label\">{html.escape(label)}</div>"
        f"<div class=\"metric-value\">{_html_value(value)}</div>"
        "</div>"
    )


def _stats_table(title: str, rows: list[dict[str, Any]], label_keys: list[str]) -> str:
    if not rows:
        return f"<section><h2>{html.escape(title)}</h2><p>No rows.</p></section>"
    columns = [
        *label_keys,
        "rows",
        "epochs",
        "suppressed_rows",
        "rms_residual_m",
        "p95_abs_residual_m",
        "max_abs_residual_m",
        "rms_normalized_residual",
        "p95_abs_normalized_residual",
        "max_abs_normalized_residual",
    ]
    header = "".join(f"<th>{html.escape(column)}</th>" for column in columns)
    body_rows = []
    for row in rows:
        cells = "".join(f"<td>{_html_value(row.get(column))}</td>" for column in columns)
        body_rows.append(f"<tr>{cells}</tr>")
    return (
        f"<section><h2>{html.escape(title)}</h2>"
        f"<table><thead><tr>{header}</tr></thead><tbody>"
        + "".join(body_rows)
        + "</tbody></table></section>"
    )


def _nice_max(values: list[float]) -> float:
    if not values:
        return 1.0
    maximum = max(values)
    if maximum <= 0.0:
        return 1.0
    exponent = math.floor(math.log10(maximum))
    base = 10.0 ** exponent
    for multiplier in (1.0, 2.0, 5.0, 10.0):
        candidate = multiplier * base
        if candidate >= maximum:
            return candidate
    return maximum


def _svg_line_chart(
    title: str,
    series: list[dict[str, Any]],
    y_fields: list[tuple[str, str, str]],
    height: int = 260,
) -> str:
    width = 1080
    left = 54
    right = 18
    top = 28
    bottom = 38
    plot_width = width - left - right
    plot_height = height - top - bottom
    points_by_field: dict[str, list[tuple[float, float, dict[str, Any]]]] = {}
    all_values: list[float] = []
    for field, _, _ in y_fields:
        points: list[tuple[float, float, dict[str, Any]]] = []
        for index, row in enumerate(series):
            value = row.get(field)
            if value is None:
                continue
            parsed = float(value)
            if not math.isfinite(parsed):
                continue
            points.append((float(index), parsed, row))
            all_values.append(parsed)
        points_by_field[field] = points
    if not all_values:
        return f"<section><h2>{html.escape(title)}</h2><p>No chartable rows.</p></section>"

    y_max = _nice_max(all_values)
    x_max = max(1, len(series) - 1)

    def sx(index: float) -> float:
        return left + (index / x_max) * plot_width

    def sy(value: float) -> float:
        return top + plot_height - (value / y_max) * plot_height

    grid = []
    for tick in range(5):
        value = y_max * tick / 4.0
        y = sy(value)
        grid.append(
            f"<line x1=\"{left}\" y1=\"{y:.2f}\" x2=\"{width - right}\" y2=\"{y:.2f}\" />"
            f"<text x=\"8\" y=\"{y + 4:.2f}\">{value:.3g}</text>"
        )
    paths = []
    legend = []
    for field, label, color in y_fields:
        points = points_by_field.get(field, [])
        if not points:
            continue
        d = " ".join(f"{sx(index):.2f},{sy(value):.2f}" for index, value, _ in points)
        paths.append(f"<polyline points=\"{d}\" stroke=\"{color}\" />")
        if len(points) <= 240:
            circles = []
            for index, value, row in points:
                circles.append(
                    f"<circle cx=\"{sx(index):.2f}\" cy=\"{sy(value):.2f}\" r=\"2.8\" "
                    f"fill=\"{color}\"><title>epoch {row['epoch_index']} tow {row['tow']} "
                    f"{label}: {value:.6f} m</title></circle>"
                )
            paths.append("".join(circles))
        legend.append(
            f"<span><i style=\"background:{color}\"></i>{html.escape(label)}</span>"
        )

    x_labels = ""
    if series:
        first = series[0]
        last = series[-1]
        x_labels = (
            f"<text class=\"x-label\" x=\"{left}\" y=\"{height - 8}\">"
            f"epoch {first['epoch_index']}</text>"
            f"<text class=\"x-label\" x=\"{width - right}\" y=\"{height - 8}\" text-anchor=\"end\">"
            f"epoch {last['epoch_index']}</text>"
        )
    return (
        f"<section><h2>{html.escape(title)}</h2>"
        f"<div class=\"legend\">{''.join(legend)}</div>"
        f"<svg class=\"chart\" viewBox=\"0 0 {width} {height}\" role=\"img\" "
        f"aria-label=\"{html.escape(title)}\">"
        f"<g class=\"grid\">{''.join(grid)}</g>"
        f"<line class=\"axis\" x1=\"{left}\" y1=\"{top}\" x2=\"{left}\" y2=\"{height - bottom}\" />"
        f"<line class=\"axis\" x1=\"{left}\" y1=\"{height - bottom}\" "
        f"x2=\"{width - right}\" y2=\"{height - bottom}\" />"
        f"{''.join(paths)}{x_labels}</svg></section>"
    )


def _svg_bar_chart(title: str, rows: list[dict[str, Any]], value_key: str, height: int = 320) -> str:
    if not rows:
        return f"<section><h2>{html.escape(title)}</h2><p>No rows.</p></section>"
    width = 1080
    left = 240
    right = 64
    top = 18
    row_height = 28
    chart_height = max(height, top + row_height * len(rows) + 24)
    max_value = max(float(row.get(value_key) or 0.0) for row in rows) or 1.0
    max_value = _nice_max([max_value])
    bars = []
    for index, row in enumerate(rows):
        y = top + index * row_height
        value = float(row.get(value_key) or 0.0)
        bar_width = (value / max_value) * (width - left - right)
        label = (
            f"{row.get('kind')} {row.get('frequency_label')} "
            f"{row.get('pair')}"
        )
        color = "#0f766e" if row.get("kind") == "phase" else "#2563eb"
        bars.append(
            f"<text x=\"8\" y=\"{y + 17}\">{html.escape(label)}</text>"
            f"<rect x=\"{left}\" y=\"{y + 5}\" width=\"{bar_width:.2f}\" height=\"16\" "
            f"fill=\"{color}\"><title>{html.escape(label)} {value:.6f} m</title></rect>"
            f"<text x=\"{left + bar_width + 6:.2f}\" y=\"{y + 17}\">{value:.6f}</text>"
        )
    return (
        f"<section><h2>{html.escape(title)}</h2>"
        f"<svg class=\"bar-chart\" viewBox=\"0 0 {width} {chart_height}\" role=\"img\" "
        f"aria-label=\"{html.escape(title)}\">{''.join(bars)}</svg></section>"
    )


def _svg_residual_scatter(
    title: str,
    points: list[dict[str, Any]],
    kind: str,
    height: int = 340,
) -> str:
    kind_points = [point for point in points if point.get("kind") == kind]
    if not kind_points:
        return f"<section><h2>{html.escape(title)}</h2><p>No {html.escape(kind)} rows.</p></section>"
    width = 1080
    left = 60
    right = 20
    top = 28
    bottom = 42
    plot_width = width - left - right
    plot_height = height - top - bottom
    epochs = [int(point["epoch_index"]) for point in kind_points]
    min_epoch = min(epochs)
    max_epoch = max(epochs)
    x_span = max(1, max_epoch - min_epoch)
    max_abs = max(abs(float(point["residual_m"])) for point in kind_points) or 1.0
    y_limit = _nice_max([max_abs])

    def sx(epoch: int) -> float:
        return left + ((epoch - min_epoch) / x_span) * plot_width

    def sy(value: float) -> float:
        return top + (y_limit - value) / (2.0 * y_limit) * plot_height

    colors = {
        0: "#0f766e" if kind == "phase" else "#2563eb",
        1: "#b45309",
        2: "#7c3aed",
    }
    grid = []
    for tick in (-1.0, -0.5, 0.0, 0.5, 1.0):
        value = y_limit * tick
        y = sy(value)
        klass = "zero-line" if abs(value) < 1e-12 else "grid-line"
        grid.append(
            f"<line class=\"{klass}\" x1=\"{left}\" y1=\"{y:.2f}\" "
            f"x2=\"{width - right}\" y2=\"{y:.2f}\" />"
            f"<text x=\"8\" y=\"{y + 4:.2f}\">{value:.3g}</text>"
        )
    rendered_points = []
    for point in kind_points:
        residual = float(point["residual_m"])
        frequency = int(point["frequency_index"])
        color = colors.get(frequency, "#64748b")
        radius = 3.2 if int(point["suppressed_by_outlier_threshold"]) else 2.2
        pair = str(point["pair"])
        rendered_points.append(
            f"<circle cx=\"{sx(int(point['epoch_index'])):.2f}\" cy=\"{sy(residual):.2f}\" "
            f"r=\"{radius}\" fill=\"{color}\" fill-opacity=\"0.72\">"
            f"<title>epoch {point['epoch_index']} tow {point['tow']} {pair} "
            f"{point['frequency_label']} residual {residual:.6f} m</title></circle>"
        )
    legend_items = []
    for frequency in sorted({int(point["frequency_index"]) for point in kind_points}):
        color = colors.get(frequency, "#64748b")
        legend_items.append(
            f"<span><i style=\"background:{color}\"></i>{html.escape(frequency_label(frequency))}</span>"
        )
    return (
        f"<section><h2>{html.escape(title)}</h2>"
        f"<div class=\"legend\">{''.join(legend_items)}"
        "<span><b class=\"suppressed-dot\"></b>threshold-suppressed rows use larger dots</span></div>"
        f"<svg class=\"chart\" viewBox=\"0 0 {width} {height}\" role=\"img\" "
        f"aria-label=\"{html.escape(title)}\">"
        f"<g class=\"grid\">{''.join(grid)}</g>"
        f"<line class=\"axis\" x1=\"{left}\" y1=\"{top}\" x2=\"{left}\" y2=\"{height - bottom}\" />"
        f"<line class=\"axis\" x1=\"{left}\" y1=\"{height - bottom}\" "
        f"x2=\"{width - right}\" y2=\"{height - bottom}\" />"
        f"{''.join(rendered_points)}"
        f"<text class=\"x-label\" x=\"{left}\" y=\"{height - 9}\">epoch {min_epoch}</text>"
        f"<text class=\"x-label\" x=\"{width - right}\" y=\"{height - 9}\" text-anchor=\"end\">"
        f"epoch {max_epoch}</text>"
        "</svg></section>"
    )


def _svg_pair_heatmap(
    title: str,
    points: list[dict[str, Any]],
    pair_tracks: list[dict[str, Any]],
    kind: str,
    max_rows: int = 24,
) -> str:
    pair_rows = [
        row for row in pair_tracks
        if row.get("kind") == kind
    ][:max_rows]
    if not pair_rows:
        return f"<section><h2>{html.escape(title)}</h2><p>No {html.escape(kind)} rows.</p></section>"
    kind_points = [point for point in points if point.get("kind") == kind]
    if not kind_points:
        return f"<section><h2>{html.escape(title)}</h2><p>No {html.escape(kind)} points.</p></section>"

    epochs = sorted({int(point["epoch_index"]) for point in kind_points})
    min_epoch = epochs[0]
    max_epoch = epochs[-1]
    x_span = max(1, max_epoch - min_epoch)
    width = 1080
    left = 250
    right = 24
    top = 18
    cell_height = 20
    row_gap = 4
    height = top + len(pair_rows) * (cell_height + row_gap) + 36
    plot_width = width - left - right
    max_abs = max(float(point["abs_residual_m"]) for point in kind_points) or 1.0

    def sx(epoch: int) -> float:
        return left + ((epoch - min_epoch) / x_span) * plot_width

    def color(value: float, suppressed: int) -> str:
        ratio = min(1.0, value / max_abs)
        if suppressed:
            return "#7f1d1d"
        if kind == "phase":
            green = int(229 - 119 * ratio)
            blue = int(224 - 150 * ratio)
            return f"rgb(15,{green},{blue})"
        red = int(219 - 10 * ratio)
        green = int(234 - 150 * ratio)
        blue = int(254 - 190 * ratio)
        return f"rgb({red},{green},{blue})"

    grouped: dict[tuple[str, int, int], dict[str, Any]] = {}
    for point in kind_points:
        key = (str(point["pair"]), int(point["frequency_index"]), int(point["epoch_index"]))
        current = grouped.get(key)
        if current is None or float(point["abs_residual_m"]) > float(current["abs_residual_m"]):
            grouped[key] = point

    rows_html = []
    for row_index, pair_row in enumerate(pair_rows):
        y = top + row_index * (cell_height + row_gap)
        pair = str(pair_row["pair"])
        freq_index = int(pair_row["frequency_index"])
        label = f"{pair} {pair_row['frequency_label']}"
        rows_html.append(f"<text x=\"8\" y=\"{y + 14}\">{html.escape(label)}</text>")
        for epoch in epochs:
            point = grouped.get((pair, freq_index, epoch))
            if point is None:
                continue
            x = sx(epoch)
            residual = float(point["residual_m"])
            abs_residual = float(point["abs_residual_m"])
            suppressed = int(point["suppressed_by_outlier_threshold"])
            rows_html.append(
                f"<rect x=\"{x:.2f}\" y=\"{y}\" width=\"{max(2.0, plot_width / max(1, len(epochs))):.2f}\" "
                f"height=\"{cell_height}\" fill=\"{color(abs_residual, suppressed)}\" "
                f"fill-opacity=\"0.82\"><title>epoch {epoch} {label} residual "
                f"{residual:.6f} m</title></rect>"
            )
    hidden_rows = max(0, len([row for row in pair_tracks if row.get("kind") == kind]) - len(pair_rows))
    note = (
        f"<p class=\"chart-note\">showing worst {len(pair_rows)} tracks"
        + (f"; {hidden_rows} lower-residual tracks are listed in the coverage table" if hidden_rows else "")
        + "</p>"
    )
    return (
        f"<section><h2>{html.escape(title)}</h2>"
        f"{note}"
        f"<svg class=\"heatmap\" viewBox=\"0 0 {width} {height}\" role=\"img\" "
        f"aria-label=\"{html.escape(title)}\">"
        f"{''.join(rows_html)}"
        f"<text class=\"x-label\" x=\"{left}\" y=\"{height - 9}\">epoch {min_epoch}</text>"
        f"<text class=\"x-label\" x=\"{width - right}\" y=\"{height - 9}\" text-anchor=\"end\">"
        f"epoch {max_epoch}</text></svg></section>"
    )


def write_html_report(path: Path, summary: dict[str, Any], title: str = "DD Residual Report") -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    by_kind = summary.get("by_kind", {})
    phase = by_kind.get("phase", {})
    code = by_kind.get("code", {})
    coverage = summary.get("coverage", {})
    cards = "".join(
        [
            _metric_cell("rows", summary.get("overall", {}).get("rows")),
            _metric_cell("epochs", coverage.get("epochs")),
            _metric_cell("satellite pairs", coverage.get("satellite_pairs")),
            _metric_cell("pair/freq/kind tracks", coverage.get("pair_frequency_kind_tracks")),
            _metric_cell("phase p95 abs m", phase.get("p95_abs_residual_m")),
            _metric_cell("code p95 abs m", code.get("p95_abs_residual_m")),
            _metric_cell("phase p95 sigma", phase.get("p95_abs_normalized_residual")),
            _metric_cell("code p95 sigma", code.get("p95_abs_normalized_residual")),
            _metric_cell("max abs m", summary.get("overall", {}).get("max_abs_residual_m")),
            _metric_cell("suppressed rows", summary.get("overall", {}).get("suppressed_rows")),
        ]
    )
    kind_rows = [
        {"kind": kind, **stats}
        for kind, stats in by_kind.items()
        if isinstance(stats, dict)
    ]
    top_pairs = list(summary.get("top_pairs", []))
    top_normalized_pairs = list(summary.get("top_normalized_pairs", []))
    pair_tracks = list(summary.get("pair_tracks", []))
    frequency_rows = list(summary.get("by_frequency", []))
    time_series = list(summary.get("time_series", []))
    residual_points = list(summary.get("residual_points", []))
    residual_chart = _svg_line_chart(
        "Raw Prefit Residual Time Series",
        time_series,
        [
            ("phase_p95_abs_residual_m", "phase p95", "#0f766e"),
            ("code_p95_abs_residual_m", "code p95", "#2563eb"),
            ("max_abs_residual_m", "max abs", "#b91c1c"),
        ],
    )
    normalized_residual_chart = _svg_line_chart(
        "Normalized Prefit Residual Time Series",
        time_series,
        [
            ("phase_p95_abs_normalized_residual", "phase p95 sigma", "#0f766e"),
            ("code_p95_abs_normalized_residual", "code p95 sigma", "#2563eb"),
            ("max_abs_normalized_residual", "max sigma", "#b91c1c"),
        ],
    )
    suppressed_chart = _svg_line_chart(
        "Suppressed Rows Over Time",
        time_series,
        [("suppressed_rows", "suppressed rows", "#9333ea")],
        height=210,
    )
    top_pairs_chart = _svg_bar_chart(
        "Worst Pair Raw P95",
        top_pairs,
        "p95_abs_residual_m",
    )
    top_normalized_pairs_chart = _svg_bar_chart(
        "Worst Pair Normalized P95",
        top_normalized_pairs,
        "p95_abs_normalized_residual",
    )
    phase_scatter = _svg_residual_scatter(
        "All Carrier Phase DD Residuals",
        residual_points,
        "phase",
    )
    code_scatter = _svg_residual_scatter(
        "All Pseudorange DD Residuals",
        residual_points,
        "code",
    )
    phase_heatmap = _svg_pair_heatmap(
        "Carrier Phase Residual Heatmap By Satellite Pair",
        residual_points,
        pair_tracks,
        "phase",
    )
    code_heatmap = _svg_pair_heatmap(
        "Pseudorange Residual Heatmap By Satellite Pair",
        residual_points,
        pair_tracks,
        "code",
    )
    worst_row = summary.get("overall", {}).get("worst_row")
    worst_html = ""
    if isinstance(worst_row, dict):
        worst_html = (
            "<section><h2>Worst Row</h2>"
            "<div class=\"kv\">"
            + "".join(
                f"<div><span>{html.escape(str(key))}</span><strong>{_html_value(value)}</strong></div>"
                for key, value in worst_row.items()
            )
            + "</div></section>"
        )
    document = f"""<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>{html.escape(title)}</title>
  <style>
    body {{ font-family: system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif; margin: 0; color: #1f2933; background: #f7f8fa; }}
    main {{ max-width: 1180px; margin: 0 auto; padding: 28px; }}
    h1 {{ font-size: 28px; margin: 0 0 18px; }}
    h2 {{ font-size: 18px; margin: 26px 0 10px; }}
    .metrics {{ display: grid; grid-template-columns: repeat(auto-fit, minmax(150px, 1fr)); gap: 10px; }}
    .metric {{ background: #fff; border: 1px solid #d9dee5; border-radius: 6px; padding: 12px; }}
    .metric-label {{ color: #52616f; font-size: 12px; }}
    .metric-value {{ font-size: 22px; font-weight: 650; margin-top: 4px; }}
    table {{ width: 100%; border-collapse: collapse; background: #fff; border: 1px solid #d9dee5; }}
    th, td {{ padding: 8px 10px; border-bottom: 1px solid #e5e9ef; text-align: right; font-size: 13px; }}
    th {{ background: #eef2f6; color: #344150; }}
    th:first-child, td:first-child {{ text-align: left; }}
    .kv {{ display: grid; grid-template-columns: repeat(auto-fit, minmax(220px, 1fr)); gap: 8px; }}
    .kv div {{ background: #fff; border: 1px solid #d9dee5; border-radius: 6px; padding: 10px; }}
    .kv span {{ display: block; color: #52616f; font-size: 12px; }}
    .kv strong {{ display: block; margin-top: 4px; font-size: 15px; }}
    svg.chart, svg.bar-chart, svg.heatmap {{ width: 100%; height: auto; background: #fff; border: 1px solid #d9dee5; }}
    .grid line {{ stroke: #e5e9ef; stroke-width: 1; }}
    .grid .zero-line {{ stroke: #7b8794; stroke-width: 1.4; }}
    .grid .grid-line {{ stroke: #e5e9ef; stroke-width: 1; }}
    .grid text, .x-label {{ fill: #52616f; font-size: 12px; }}
    .axis {{ stroke: #8b98a8; stroke-width: 1.2; }}
    polyline {{ fill: none; stroke-width: 2.2; }}
    .legend {{ display: flex; gap: 14px; flex-wrap: wrap; margin: 0 0 8px; color: #344150; font-size: 13px; }}
    .legend i {{ display: inline-block; width: 18px; height: 3px; margin-right: 6px; vertical-align: middle; }}
    .suppressed-dot {{ display: inline-block; width: 9px; height: 9px; border-radius: 50%; background: #1f2933; margin-right: 6px; vertical-align: middle; }}
    .chart-note {{ color: #52616f; font-size: 13px; margin: -4px 0 8px; }}
  </style>
</head>
<body>
<main>
  <h1>{html.escape(title)}</h1>
  <section class="metrics">{cards}</section>
  {phase_scatter}
  {code_scatter}
  {phase_heatmap}
  {code_heatmap}
  {residual_chart}
  {normalized_residual_chart}
  {suppressed_chart}
  {top_pairs_chart}
  {top_normalized_pairs_chart}
  {_stats_table("By Kind", kind_rows, ["kind"])}
  {_stats_table("By Frequency", frequency_rows, ["frequency_label"])}
  {_stats_table("Worst Satellite Pairs By Raw Residual", top_pairs, ["kind", "frequency_label", "pair"])}
  {_stats_table("Worst Satellite Pairs By Normalized Residual", top_normalized_pairs, ["kind", "frequency_label", "pair"])}
  {_stats_table("All Satellite Pair Tracks", pair_tracks, ["kind", "frequency_label", "pair"])}
  {worst_html}
</main>
</body>
</html>
"""
    path.write_text(document, encoding="utf-8")


def enforce_requirements(summary: dict[str, Any], args: argparse.Namespace) -> None:
    failures: list[str] = []

    phase_p95 = summary["by_kind"]["phase"]["p95_abs_residual_m"]
    if args.require_phase_p95_max is not None:
        if phase_p95 is None or float(phase_p95) > args.require_phase_p95_max:
            observed = "n/a" if phase_p95 is None else f"{float(phase_p95):.6f}"
            failures.append(
                f"phase p95 abs residual {observed} m > {args.require_phase_p95_max:.6f} m"
            )

    code_p95 = summary["by_kind"]["code"]["p95_abs_residual_m"]
    if args.require_code_p95_max is not None:
        if code_p95 is None or float(code_p95) > args.require_code_p95_max:
            observed = "n/a" if code_p95 is None else f"{float(code_p95):.6f}"
            failures.append(
                f"code p95 abs residual {observed} m > {args.require_code_p95_max:.6f} m"
            )

    max_abs = summary["overall"]["max_abs_residual_m"]
    if args.require_max_abs_residual_max is not None:
        if max_abs is None or float(max_abs) > args.require_max_abs_residual_max:
            observed = "n/a" if max_abs is None else f"{float(max_abs):.6f}"
            failures.append(
                f"max abs residual {observed} m > {args.require_max_abs_residual_max:.6f} m"
            )

    suppressed = int(summary["overall"]["suppressed_rows"])
    if args.require_suppressed_rows_max is not None and suppressed > args.require_suppressed_rows_max:
        failures.append(
            f"suppressed rows {suppressed} > {args.require_suppressed_rows_max}"
        )

    if failures:
        raise SystemExit("DD residual checks failed:\n" + "\n".join(f"  - {item}" for item in failures))


def main() -> int:
    args = parse_args()
    if args.top_n < 0:
        raise SystemExit("--top-n must be non-negative")
    records = filter_records(read_records(args.input_csv), args.kind, args.frequency_index)
    if not records:
        raise SystemExit(f"No DD residual rows matched {args.input_csv}")

    summary = summarize_records(records, top_n=args.top_n)
    if args.summary_json is not None:
        args.summary_json.parent.mkdir(parents=True, exist_ok=True)
        args.summary_json.write_text(
            json.dumps(summary, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
    if args.top_pairs_csv is not None:
        write_top_pairs_csv(args.top_pairs_csv, list(summary["top_pairs"]))
    if args.html_report is not None:
        write_html_report(args.html_report, summary)

    enforce_requirements(summary, args)

    print("DD residual summary")
    print(f"  rows: {summary['overall']['rows']}")
    print(f"  epochs: {summary['coverage']['epochs']}")
    print(f"  satellite_pairs: {summary['coverage']['satellite_pairs']}")
    print(f"  pair_frequency_kind_tracks: {summary['coverage']['pair_frequency_kind_tracks']}")
    print(f"  phase_p95_abs_m: {summary['by_kind']['phase']['p95_abs_residual_m']}")
    print(f"  code_p95_abs_m: {summary['by_kind']['code']['p95_abs_residual_m']}")
    print(f"  phase_p95_sigma: {summary['by_kind']['phase']['p95_abs_normalized_residual']}")
    print(f"  code_p95_sigma: {summary['by_kind']['code']['p95_abs_normalized_residual']}")
    print(f"  max_abs_m: {summary['overall']['max_abs_residual_m']}")
    if args.summary_json is not None:
        print(f"  summary: {args.summary_json}")
    if args.top_pairs_csv is not None:
        print(f"  top_pairs: {args.top_pairs_csv}")
    if args.html_report is not None:
        print(f"  html_report: {args.html_report}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
