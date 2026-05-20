"""Pure statistical aggregations over DD residual records.

Owns the percentile / RMS / worst-row computations and the
``(epoch, pair, frequency)`` coverage stats.  All functions accept lists of
dict-shaped records and return JSON-serialisable dicts so the summary and
HTML layers can compose them without circular imports.
"""

from __future__ import annotations

import math
from typing import Any

from gnss_dd_residuals_records import (
    abs_normalized_residual,
    compact_row,
    frequency_label,
    normalized_residual,
    pair_label,
    rounded,
)


def percentile(values: list[float], q: float) -> float | None:
    """Linear-interpolation percentile for ``q`` in ``[0, 1]``.

    Matches numpy ``percentile(q*100)`` for finite inputs but avoids the
    numpy dependency so this module stays portable across stripped-down
    benchmark sandboxes.
    """

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


def group_records(
    records: list[dict[str, Any]],
    keys: tuple[str, ...],
) -> dict[tuple[Any, ...], list[dict[str, Any]]]:
    """Group records by the requested record-dict ``keys`` tuple."""

    groups: dict[tuple[Any, ...], list[dict[str, Any]]] = {}
    for record in records:
        key = tuple(record[name] for name in keys)
        groups.setdefault(key, []).append(record)
    return groups


def residual_stats(records: list[dict[str, Any]]) -> dict[str, Any]:
    """Per-group summary statistics (rms / p95 / max / worst row)."""

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
    """Counts and band labels describing the records' DD coverage envelope."""

    epochs = sorted(
        {
            (int(record["epoch_index"]), int(record["gps_week"]), float(record["tow"]))
            for record in records
        }
    )
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


__all__ = [
    "coverage_stats",
    "group_records",
    "percentile",
    "residual_stats",
]
