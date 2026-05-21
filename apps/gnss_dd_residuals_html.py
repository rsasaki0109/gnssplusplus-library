"""HTML / SVG rendering for DD residual diagnostics.

Consumes the summary dict produced by ``gnss_dd_residuals_summary`` and
emits a self-contained HTML report (no external assets).  Pure rendering —
no statistics or I/O beyond the final file write.
"""

from __future__ import annotations

import html
import math
from pathlib import Path
from typing import Any

from gnss_dd_residuals_records import frequency_label


def html_value(value: Any) -> str:
    """Escape ``value`` for HTML output, mapping ``None`` to the literal ``n/a``."""

    if value is None:
        return "n/a"
    return html.escape(str(value))


def metric_cell(label: str, value: Any) -> str:
    """Top-of-report KPI card."""

    return (
        "<div class=\"metric\">"
        f"<div class=\"metric-label\">{html.escape(label)}</div>"
        f"<div class=\"metric-value\">{html_value(value)}</div>"
        "</div>"
    )


def stats_table(title: str, rows: list[dict[str, Any]], label_keys: list[str]) -> str:
    """Render a labeled stats table; emits an empty placeholder for no-row inputs."""

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
        cells = "".join(f"<td>{html_value(row.get(column))}</td>" for column in columns)
        body_rows.append(f"<tr>{cells}</tr>")
    return (
        f"<section><h2>{html.escape(title)}</h2>"
        f"<table><thead><tr>{header}</tr></thead><tbody>"
        + "".join(body_rows)
        + "</tbody></table></section>"
    )


def nice_max(values: list[float]) -> float:
    """Round-up a chart's Y-axis upper bound to a ``{1, 2, 5} × 10^n`` step."""

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


def svg_line_chart(
    title: str,
    series: list[dict[str, Any]],
    y_fields: list[tuple[str, str, str]],
    height: int = 260,
) -> str:
    """Multi-line time-series chart used for the per-epoch residual trend.

    ``y_fields`` is a list of ``(field_name, legend_label, stroke_color)``
    tuples.  Rows missing the field are silently skipped so a partial input
    still produces a chart.
    """

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

    y_max = nice_max(all_values)
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


def svg_bar_chart(
    title: str, rows: list[dict[str, Any]], value_key: str, height: int = 320
) -> str:
    """Horizontal bar chart for the worst-pair summaries."""

    if not rows:
        return f"<section><h2>{html.escape(title)}</h2><p>No rows.</p></section>"
    width = 1080
    left = 240
    right = 64
    top = 18
    row_height = 28
    chart_height = max(height, top + row_height * len(rows) + 24)
    max_value = max(float(row.get(value_key) or 0.0) for row in rows) or 1.0
    max_value = nice_max([max_value])
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


def svg_residual_scatter(
    title: str,
    points: list[dict[str, Any]],
    kind: str,
    height: int = 340,
) -> str:
    """Per-epoch scatter of raw residuals coloured by frequency band."""

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
    y_limit = nice_max([max_abs])

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


def svg_pair_heatmap(
    title: str,
    points: list[dict[str, Any]],
    pair_tracks: list[dict[str, Any]],
    kind: str,
    max_rows: int = 24,
) -> str:
    """Per-pair heatmap of worst residual magnitude across epochs."""

    pair_rows = [row for row in pair_tracks if row.get("kind") == kind][:max_rows]
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
    hidden_rows = max(
        0,
        len([row for row in pair_tracks if row.get("kind") == kind]) - len(pair_rows),
    )
    note = (
        f"<p class=\"chart-note\">showing worst {len(pair_rows)} tracks"
        + (
            f"; {hidden_rows} lower-residual tracks are listed in the coverage table"
            if hidden_rows
            else ""
        )
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


_DOCUMENT_CSS = """\
body { font-family: system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif; margin: 0; color: #1f2933; background: #f7f8fa; }
main { max-width: 1180px; margin: 0 auto; padding: 28px; }
h1 { font-size: 28px; margin: 0 0 18px; }
h2 { font-size: 18px; margin: 26px 0 10px; }
.metrics { display: grid; grid-template-columns: repeat(auto-fit, minmax(150px, 1fr)); gap: 10px; }
.metric { background: #fff; border: 1px solid #d9dee5; border-radius: 6px; padding: 12px; }
.metric-label { color: #52616f; font-size: 12px; }
.metric-value { font-size: 22px; font-weight: 650; margin-top: 4px; }
table { width: 100%; border-collapse: collapse; background: #fff; border: 1px solid #d9dee5; }
th, td { padding: 8px 10px; border-bottom: 1px solid #e5e9ef; text-align: right; font-size: 13px; }
th { background: #eef2f6; color: #344150; }
th:first-child, td:first-child { text-align: left; }
.kv { display: grid; grid-template-columns: repeat(auto-fit, minmax(220px, 1fr)); gap: 8px; }
.kv div { background: #fff; border: 1px solid #d9dee5; border-radius: 6px; padding: 10px; }
.kv span { display: block; color: #52616f; font-size: 12px; }
.kv strong { display: block; margin-top: 4px; font-size: 15px; }
svg.chart, svg.bar-chart, svg.heatmap { width: 100%; height: auto; background: #fff; border: 1px solid #d9dee5; }
.grid line { stroke: #e5e9ef; stroke-width: 1; }
.grid .zero-line { stroke: #7b8794; stroke-width: 1.4; }
.grid .grid-line { stroke: #e5e9ef; stroke-width: 1; }
.grid text, .x-label { fill: #52616f; font-size: 12px; }
.axis { stroke: #8b98a8; stroke-width: 1.2; }
polyline { fill: none; stroke-width: 2.2; }
.legend { display: flex; gap: 14px; flex-wrap: wrap; margin: 0 0 8px; color: #344150; font-size: 13px; }
.legend i { display: inline-block; width: 18px; height: 3px; margin-right: 6px; vertical-align: middle; }
.suppressed-dot { display: inline-block; width: 9px; height: 9px; border-radius: 50%; background: #1f2933; margin-right: 6px; vertical-align: middle; }
.chart-note { color: #52616f; font-size: 13px; margin: -4px 0 8px; }
"""


def _worst_row_section(worst_row: Any) -> str:
    if not isinstance(worst_row, dict):
        return ""
    return (
        "<section><h2>Worst Row</h2>"
        "<div class=\"kv\">"
        + "".join(
            f"<div><span>{html.escape(str(key))}</span><strong>{html_value(value)}</strong></div>"
            for key, value in worst_row.items()
        )
        + "</div></section>"
    )


def write_html_report(
    path: Path, summary: dict[str, Any], title: str = "DD Residual Report"
) -> None:
    """Render the full HTML document for ``summary`` to ``path``."""

    path.parent.mkdir(parents=True, exist_ok=True)
    by_kind = summary.get("by_kind", {})
    phase = by_kind.get("phase", {})
    code = by_kind.get("code", {})
    coverage = summary.get("coverage", {})
    cards = "".join(
        [
            metric_cell("rows", summary.get("overall", {}).get("rows")),
            metric_cell("epochs", coverage.get("epochs")),
            metric_cell("satellite pairs", coverage.get("satellite_pairs")),
            metric_cell("pair/freq/kind tracks", coverage.get("pair_frequency_kind_tracks")),
            metric_cell("phase p95 abs m", phase.get("p95_abs_residual_m")),
            metric_cell("code p95 abs m", code.get("p95_abs_residual_m")),
            metric_cell("phase p95 sigma", phase.get("p95_abs_normalized_residual")),
            metric_cell("code p95 sigma", code.get("p95_abs_normalized_residual")),
            metric_cell("max abs m", summary.get("overall", {}).get("max_abs_residual_m")),
            metric_cell("suppressed rows", summary.get("overall", {}).get("suppressed_rows")),
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
    residual_chart = svg_line_chart(
        "Raw Prefit Residual Time Series",
        time_series,
        [
            ("phase_p95_abs_residual_m", "phase p95", "#0f766e"),
            ("code_p95_abs_residual_m", "code p95", "#2563eb"),
            ("max_abs_residual_m", "max abs", "#b91c1c"),
        ],
    )
    normalized_residual_chart = svg_line_chart(
        "Normalized Prefit Residual Time Series",
        time_series,
        [
            ("phase_p95_abs_normalized_residual", "phase p95 sigma", "#0f766e"),
            ("code_p95_abs_normalized_residual", "code p95 sigma", "#2563eb"),
            ("max_abs_normalized_residual", "max sigma", "#b91c1c"),
        ],
    )
    suppressed_chart = svg_line_chart(
        "Suppressed Rows Over Time",
        time_series,
        [("suppressed_rows", "suppressed rows", "#9333ea")],
        height=210,
    )
    top_pairs_chart = svg_bar_chart("Worst Pair Raw P95", top_pairs, "p95_abs_residual_m")
    top_normalized_pairs_chart = svg_bar_chart(
        "Worst Pair Normalized P95", top_normalized_pairs, "p95_abs_normalized_residual"
    )
    phase_scatter = svg_residual_scatter(
        "All Carrier Phase DD Residuals", residual_points, "phase"
    )
    code_scatter = svg_residual_scatter(
        "All Pseudorange DD Residuals", residual_points, "code"
    )
    phase_heatmap = svg_pair_heatmap(
        "Carrier Phase Residual Heatmap By Satellite Pair",
        residual_points,
        pair_tracks,
        "phase",
    )
    code_heatmap = svg_pair_heatmap(
        "Pseudorange Residual Heatmap By Satellite Pair",
        residual_points,
        pair_tracks,
        "code",
    )
    worst_html = _worst_row_section(summary.get("overall", {}).get("worst_row"))
    document = f"""<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>{html.escape(title)}</title>
  <style>
{_DOCUMENT_CSS}  </style>
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
  {stats_table("By Kind", kind_rows, ["kind"])}
  {stats_table("By Frequency", frequency_rows, ["frequency_label"])}
  {stats_table("Worst Satellite Pairs By Raw Residual", top_pairs, ["kind", "frequency_label", "pair"])}
  {stats_table("Worst Satellite Pairs By Normalized Residual", top_normalized_pairs, ["kind", "frequency_label", "pair"])}
  {stats_table("All Satellite Pair Tracks", pair_tracks, ["kind", "frequency_label", "pair"])}
  {worst_html}
</main>
</body>
</html>
"""
    path.write_text(document, encoding="utf-8")


__all__ = [
    "html_value",
    "metric_cell",
    "nice_max",
    "stats_table",
    "svg_bar_chart",
    "svg_line_chart",
    "svg_pair_heatmap",
    "svg_residual_scatter",
    "write_html_report",
]
