"""Per-row filters for DD residual records.

Kept as a separate module because filtering is the one piece a downstream
caller (e.g., an interactive notebook session) is most likely to want to
import in isolation, without dragging in the statistics or HTML layers.
"""

from __future__ import annotations

from typing import Any


def filter_records(
    records: list[dict[str, Any]],
    kind: str = "all",
    frequency_index: int | None = None,
) -> list[dict[str, Any]]:
    """Keep records matching the requested ``kind`` and ``frequency_index``.

    ``kind`` of ``"all"`` is the documented no-op default; passing
    ``"phase"`` / ``"code"`` filters to that DD family.  ``frequency_index``
    of ``None`` keeps every band.
    """

    filtered = records
    if kind != "all":
        filtered = [record for record in filtered if record["kind"] == kind]
    if frequency_index is not None:
        filtered = [
            record for record in filtered if int(record["frequency_index"]) == frequency_index
        ]
    return filtered


__all__ = ["filter_records"]
