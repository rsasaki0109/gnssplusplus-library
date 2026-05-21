"""DD residual record schema helpers.

This module owns the per-row transforms that mediate between the raw CSV
dict and the higher-level statistics / rendering layers.  All functions are
pure; there is no I/O or global state, so each predicate can be unit-tested
with a hand-built dict literal.
"""

from __future__ import annotations

import math
from typing import Any


def frequency_label(frequency_index: int) -> str:
    """Human-readable band label for a 0/1/2-indexed RINEX frequency slot."""

    labels = {
        0: "freq0 primary (L1/E1/B1)",
        1: "freq1 secondary (L2/E5/B2)",
        2: "freq2 L5-class (L5/E5a/B2a)",
    }
    return labels.get(frequency_index, f"freq{frequency_index}")


def pair_label(reference_satellite: Any, satellite: Any) -> str:
    """Canonical ``REF-SAT`` label used everywhere we group by DD pair."""

    return f"{reference_satellite}-{satellite}"


def residual_sigma_m(record: dict[str, Any]) -> float | None:
    """Combined-variance σ for a DD record, or ``None`` if missing/non-finite.

    The DD residual variance is the sum of the reference and satellite
    measurement variances (independence assumption); when either is absent
    or the sum is non-positive the σ is undefined and we return ``None``
    so downstream summaries can skip the row.
    """

    reference_variance = record.get("reference_variance_m2")
    satellite_variance = record.get("satellite_variance_m2")
    if reference_variance is None or satellite_variance is None:
        return None
    variance = float(reference_variance) + float(satellite_variance)
    if not math.isfinite(variance) or variance <= 0.0:
        return None
    return math.sqrt(variance)


def normalized_residual(record: dict[str, Any]) -> float | None:
    """Residual divided by its σ, or ``None`` if σ is undefined."""

    sigma = residual_sigma_m(record)
    if sigma is None:
        return None
    return float(record["residual_m"]) / sigma


def abs_normalized_residual(record: dict[str, Any]) -> float | None:
    """Magnitude of the normalized residual, ``None`` when σ is undefined."""

    value = normalized_residual(record)
    return abs(value) if value is not None else None


def rounded(value: float | None) -> float | None:
    """Round to 6 decimal places, propagating ``None``.

    Centralised here so the choice of precision is one edit away if we
    ever switch to a fixed-point output format.
    """

    if value is None:
        return None
    return round(value, 6)


def compact_row(record: dict[str, Any]) -> dict[str, Any]:
    """Lossy serialisation of a DD record into the renderer's row shape.

    Drops the raw ``*_variance_m2`` fields (already folded into σ /
    normalized residual) and adds the per-record annotations
    (``frequency_label``, ``pair``, ``residual_sigma_m`` …) that the HTML
    and CSV consumers reuse.
    """

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


__all__ = [
    "abs_normalized_residual",
    "compact_row",
    "frequency_label",
    "normalized_residual",
    "pair_label",
    "residual_sigma_m",
    "rounded",
]
