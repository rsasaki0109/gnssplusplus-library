"""Top-level summary assembly for DD residual diagnostics.

Composes the per-row, per-group, and per-epoch statistics into the
JSON-serialisable ``summary`` dict consumed by the CLI, JSON dump, and HTML
renderer.  No I/O or rendering here — those live in the io / html modules.
"""

from __future__ import annotations

from typing import Any

from gnss_dd_residuals_records import (
    abs_normalized_residual,
    compact_row,
    frequency_label,
    normalized_residual,
    pair_label,
    rounded,
)
from gnss_dd_residuals_statistics import (
    coverage_stats,
    group_records,
    residual_stats,
)


def summarize_groups(
    records: list[dict[str, Any]],
    keys: tuple[str, ...],
    top_n: int | None = None,
) -> list[dict[str, Any]]:
    """Group records by ``keys`` and return per-group summaries.

    Adds the ``frequency_label`` / ``pair`` annotations expected by the
    renderer when those keys are present in the grouping tuple.  Sorted by
    descending ``p95_abs_residual_m`` so the first ``top_n`` rows are the
    worst tracks for that grouping.
    """

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
    """Per-epoch summary row used by the line-chart renderer."""

    series: list[dict[str, Any]] = []
    for key, group in group_records(
        records, ("epoch_index", "gps_week", "tow", "status")
    ).items():
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
                "max_abs_normalized_residual": group_stats["max_abs_normalized_residual"],
            }
        )
    series.sort(key=lambda item: (int(item["epoch_index"]), float(item["tow"] or 0.0)))
    return series


def compact_residual_points(records: list[dict[str, Any]]) -> list[dict[str, Any]]:
    """Renderer-shaped point list for the scatter / heatmap charts."""

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
    """Assemble the full summary dict (overall + per-kind + per-track)."""

    pair_keys = ("kind", "frequency_index", "reference_satellite", "satellite")
    pair_tracks = summarize_groups(records, pair_keys, top_n=None)
    top_pairs = summarize_groups(records, pair_keys, top_n=max(0, top_n))
    top_normalized_pairs = sorted(
        pair_tracks,
        key=lambda item: (
            float(item["p95_abs_normalized_residual"] or -1.0),
            float(item["max_abs_normalized_residual"] or -1.0),
            int(item["rows"]),
        ),
        reverse=True,
    )[: max(0, top_n)]
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


__all__ = [
    "compact_residual_points",
    "summarize_groups",
    "summarize_records",
    "summarize_time_series",
]
