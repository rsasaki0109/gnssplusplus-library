"""CSV I/O for DD residual diagnostics.

Reads ``--dd-residuals-csv`` files emitted by ``gnss solve`` into the
dict-of-Python-primitives shape consumed by the rest of the pipeline, and
writes the worst-pair CSV used in CI dashboards.  Pure I/O — no statistics
or rendering happen here.
"""

from __future__ import annotations

import csv
import math
from pathlib import Path
from typing import Any


_REQUIRED_COLUMNS: frozenset[str] = frozenset(
    {
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
)


_TOP_PAIRS_FIELDNAMES: tuple[str, ...] = (
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
)


def optional_float(value: str | None) -> float | None:
    """Parse a CSV cell into a float, returning ``None`` for empty/NaN cells.

    Used for the ``*_variance_m2`` columns which may be omitted when the
    solver did not emit per-measurement variance estimates.
    """

    if value is None or value == "":
        return None
    parsed = float(value)
    return parsed if math.isfinite(parsed) else None


def read_records(path: Path) -> list[dict[str, Any]]:
    """Read a DD residual CSV into per-row dicts.

    Raises ``SystemExit`` if any of the canonical columns are missing; this
    matches the original script's contract so callers can rely on the
    CLI-level error message.  Non-finite residual rows are silently
    dropped because they would otherwise propagate NaNs into the summary
    statistics.
    """

    records: list[dict[str, Any]] = []
    with path.open(encoding="ascii", newline="") as handle:
        reader = csv.DictReader(handle)
        missing = _REQUIRED_COLUMNS.difference(reader.fieldnames or [])
        if missing:
            raise SystemExit(
                f"Missing DD residual columns in {path}: {', '.join(sorted(missing))}"
            )
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
                    "reference_variance_m2": optional_float(row.get("reference_variance_m2")),
                    "satellite_variance_m2": optional_float(row.get("satellite_variance_m2")),
                    "suppressed_by_outlier_threshold": int(
                        float(row["suppressed_by_outlier_threshold"])
                    ),
                }
            )
    return records


def write_top_pairs_csv(path: Path, top_pairs: list[dict[str, Any]]) -> None:
    """Persist the ranked worst-pair table to disk."""

    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="ascii", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(_TOP_PAIRS_FIELDNAMES))
        writer.writeheader()
        for index, pair in enumerate(top_pairs, start=1):
            row = {field: pair.get(field) for field in _TOP_PAIRS_FIELDNAMES if field != "rank"}
            row["rank"] = index
            writer.writerow(row)


__all__ = [
    "optional_float",
    "read_records",
    "write_top_pairs_csv",
]
