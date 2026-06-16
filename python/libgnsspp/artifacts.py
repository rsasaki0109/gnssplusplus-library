"""Read-only helpers for libgnss++ summary and solution artifacts."""

from __future__ import annotations

from dataclasses import dataclass
import datetime as dt
import json
import math
from pathlib import Path
from typing import Any, Iterable, Literal


GPS_EPOCH = dt.datetime(1980, 1, 6)
WGS84_A = 6378137.0
WGS84_F = 1.0 / 298.257223563
WGS84_E2 = WGS84_F * (2.0 - WGS84_F)

PosFormat = Literal["auto", "libgnss", "rtklib"]


@dataclass(frozen=True)
class PosRecord:
    week: int
    tow: float
    lat_deg: float
    lon_deg: float
    height_m: float
    ecef_m: tuple[float, float, float]
    status: int
    satellites: int
    ratio: float | None = None
    source_format: str = "libgnss"
    line_number: int = 0


def load_summary(path: str | Path) -> dict[str, Any]:
    """Load a summary JSON artifact and require a top-level object."""

    summary_path = Path(path)
    with summary_path.open(encoding="utf-8") as handle:
        payload = json.load(handle)
    if not isinstance(payload, dict):
        raise ValueError(f"{summary_path}: summary JSON must contain an object")
    return payload


def summary_schema(payload: dict[str, Any]) -> str | None:
    """Return the declared schema field used by current summary artifacts."""

    value = (
        payload.get("summary_schema")
        or payload.get("schema")
        or payload.get("schema_version")
    )
    return str(value) if value is not None else None


def geodetic_deg_to_ecef(lat_deg: float, lon_deg: float, height_m: float) -> tuple[float, float, float]:
    lat = math.radians(lat_deg)
    lon = math.radians(lon_deg)
    sin_lat = math.sin(lat)
    cos_lat = math.cos(lat)
    sin_lon = math.sin(lon)
    cos_lon = math.cos(lon)
    n = WGS84_A / math.sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat)
    return (
        (n + height_m) * cos_lat * cos_lon,
        (n + height_m) * cos_lat * sin_lon,
        (n * (1.0 - WGS84_E2) + height_m) * sin_lat,
    )


def parse_rtklib_timestamp(token_a: str, token_b: str) -> tuple[int, float]:
    """Parse RTKLIB date/time or week/TOW timestamp tokens."""

    if "/" not in token_a:
        return int(float(token_a)), float(token_b)

    year, month, day = map(int, token_a.split("/"))
    hour, minute, second = token_b.split(":")
    second_float = float(second)
    second_int = int(second_float)
    microsecond = int(round((second_float - second_int) * 1_000_000))
    stamp = dt.datetime(year, month, day, int(hour), int(minute), second_int, microsecond)
    total_seconds = (stamp - GPS_EPOCH).total_seconds()
    week = int(total_seconds // 604800)
    return week, total_seconds - week * 604800


def _data_lines(path: Path) -> Iterable[tuple[int, list[str]]]:
    with path.open(encoding="utf-8") as handle:
        for line_number, line in enumerate(handle, start=1):
            stripped = line.strip()
            if not stripped or stripped.startswith("%") or stripped.startswith("#"):
                continue
            yield line_number, stripped.split()


def _detect_pos_format(parts: list[str]) -> Literal["libgnss", "rtklib"]:
    if "/" in parts[0]:
        return "rtklib"
    if len(parts) >= 11:
        return "libgnss"
    return "rtklib"


def _parse_libgnss_pos_row(parts: list[str], line_number: int) -> PosRecord:
    if len(parts) < 10:
        raise ValueError(f"line {line_number}: expected at least 10 libgnss++ .pos columns")
    ecef = (float(parts[2]), float(parts[3]), float(parts[4]))
    return PosRecord(
        week=int(float(parts[0])),
        tow=float(parts[1]),
        ecef_m=ecef,
        lat_deg=float(parts[5]),
        lon_deg=float(parts[6]),
        height_m=float(parts[7]),
        status=int(float(parts[8])),
        satellites=int(float(parts[9])),
        ratio=float(parts[11]) if len(parts) > 11 else None,
        source_format="libgnss",
        line_number=line_number,
    )


def _parse_rtklib_pos_row(parts: list[str], line_number: int) -> PosRecord:
    if len(parts) < 7:
        raise ValueError(f"line {line_number}: expected at least 7 RTKLIB .pos columns")
    week, tow = parse_rtklib_timestamp(parts[0], parts[1])
    lat = float(parts[2])
    lon = float(parts[3])
    height = float(parts[4])
    return PosRecord(
        week=week,
        tow=tow,
        lat_deg=lat,
        lon_deg=lon,
        height_m=height,
        ecef_m=geodetic_deg_to_ecef(lat, lon, height),
        status=int(float(parts[5])),
        satellites=int(float(parts[6])),
        ratio=float(parts[14]) if len(parts) > 14 else None,
        source_format="rtklib",
        line_number=line_number,
    )


def load_pos(path: str | Path, source_format: PosFormat = "auto") -> list[PosRecord]:
    """Load libgnss++ or RTKLIB .pos rows without running any solver."""

    pos_path = Path(path)
    rows: list[PosRecord] = []
    detected_format: Literal["libgnss", "rtklib"] | None = None
    for line_number, parts in _data_lines(pos_path):
        if source_format == "auto":
            if detected_format is None:
                detected_format = _detect_pos_format(parts)
            row_format = detected_format
        elif source_format in {"libgnss", "rtklib"}:
            row_format = source_format
        else:
            raise ValueError("source_format must be one of: auto, libgnss, rtklib")

        if row_format == "libgnss":
            rows.append(_parse_libgnss_pos_row(parts, line_number))
        else:
            rows.append(_parse_rtklib_pos_row(parts, line_number))
    return rows


def pos_status_name(record: PosRecord) -> str:
    if record.source_format == "rtklib":
        return {
            1: "FIX",
            2: "FLOAT",
            4: "DGPS",
            5: "SINGLE",
            6: "PPP",
        }.get(record.status, f"RTKLIB_{record.status}")
    return {
        0: "NONE",
        1: "SPP",
        2: "DGPS",
        3: "FLOAT",
        4: "FIXED",
        5: "PPP_FLOAT",
        6: "PPP_FIXED",
    }.get(record.status, f"STATUS_{record.status}")


def pos_stats(records: Iterable[PosRecord]) -> dict[str, Any]:
    """Return compact counts for loaded .pos records."""

    rows = list(records)
    status_counts: dict[str, int] = {}
    for row in rows:
        name = pos_status_name(row)
        status_counts[name] = status_counts.get(name, 0) + 1

    valid_rows = [row for row in rows if row.status != 0]
    fixed_rows = [
        row
        for row in rows
        if pos_status_name(row) in {"FIX", "FIXED", "PPP_FIXED"}
    ]
    float_rows = [
        row
        for row in rows
        if pos_status_name(row) in {"FLOAT", "PPP_FLOAT"}
    ]
    if rows:
        first = min(rows, key=lambda row: (row.week, row.tow))
        last = max(rows, key=lambda row: (row.week, row.tow))
        solution_span_s = (last.week - first.week) * 604800.0 + (last.tow - first.tow)
    else:
        first = last = None
        solution_span_s = 0.0

    return {
        "total_epochs": len(rows),
        "valid_epochs": len(valid_rows),
        "fixed_epochs": len(fixed_rows),
        "float_epochs": len(float_rows),
        "status_counts": status_counts,
        "source_formats": sorted({row.source_format for row in rows}),
        "mean_satellites": (
            sum(row.satellites for row in rows) / len(rows)
            if rows else 0.0
        ),
        "first_week": first.week if first is not None else None,
        "first_tow": first.tow if first is not None else None,
        "last_week": last.week if last is not None else None,
        "last_tow": last.tow if last is not None else None,
        "solution_span_s": solution_span_s,
    }


__all__ = [
    "PosRecord",
    "geodetic_deg_to_ecef",
    "load_pos",
    "load_summary",
    "parse_rtklib_timestamp",
    "pos_stats",
    "pos_status_name",
    "summary_schema",
]
