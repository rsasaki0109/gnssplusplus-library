"""Small fail-closed SINEX station-coordinate reader.

This module intentionally supports only the pieces needed by application
sign-off workflows: ``SOLUTION/EPOCHS`` validity records and station XYZ rows
from ``SOLUTION/ESTIMATE``.  It never falls back to ``SOLUTION/APRIORI``.
"""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timedelta, timezone
from pathlib import Path
import math


@dataclass(frozen=True)
class SinexStationEstimate:
    station: str
    point: str
    solution_id: str
    validity_start: datetime
    validity_end: datetime | None
    mean_epoch: datetime
    estimate_epoch: datetime
    ecef_m: tuple[float, float, float]
    sigma_m: tuple[float, float, float]


def parse_sinex_epoch(value: str, *, allow_open: bool = False) -> datetime | None:
    """Parse ``YY:DOY:SSSSS`` with the SINEX 00=open convention."""
    fields = value.split(":")
    if len(fields) != 3:
        raise ValueError(f"invalid SINEX epoch: {value!r}")
    year2, doy, seconds = (int(field) for field in fields)
    if allow_open and year2 == 0 and doy == 0 and seconds == 0:
        return None
    year = 1900 + year2 if year2 >= 80 else 2000 + year2
    if doy < 1 or doy > 366 or seconds < 0 or seconds >= 86400:
        raise ValueError(f"invalid SINEX epoch: {value!r}")
    return datetime(year, 1, 1, tzinfo=timezone.utc) + timedelta(
        days=doy - 1, seconds=seconds
    )


def _blocks(path: Path) -> dict[str, list[str]]:
    blocks: dict[str, list[str]] = {}
    current: str | None = None
    with path.open(encoding="ascii", errors="strict") as handle:
        for raw in handle:
            line = raw.rstrip("\r\n")
            if line.startswith("+"):
                current = line[1:].strip()
                blocks.setdefault(current, [])
                continue
            if line.startswith("-"):
                current = None
                continue
            if current is not None and line and not line.startswith("*"):
                blocks[current].append(line)
    return blocks


def read_station_estimate(
    path: Path,
    station: str,
    observation_epoch: datetime,
) -> SinexStationEstimate:
    """Return the unique valid estimated XYZ for ``station``.

    Raises ``ValueError`` when metadata is absent, ambiguous, outside the
    solution validity interval, non-finite, or split across solution IDs.
    """
    station = station.upper()
    if observation_epoch.tzinfo is None:
        observation_epoch = observation_epoch.replace(tzinfo=timezone.utc)
    else:
        observation_epoch = observation_epoch.astimezone(timezone.utc)
    blocks = _blocks(path)
    epoch_rows = blocks.get("SOLUTION/EPOCHS")
    estimate_rows = blocks.get("SOLUTION/ESTIMATE")
    if not epoch_rows or not estimate_rows:
        raise ValueError("SINEX must contain SOLUTION/EPOCHS and SOLUTION/ESTIMATE")

    candidates: list[tuple[str, str, datetime, datetime | None, datetime]] = []
    for line in epoch_rows:
        fields = line.split()
        if len(fields) < 7 or fields[0].upper() != station:
            continue
        point, solution_id = fields[1], fields[2]
        start = parse_sinex_epoch(fields[4])
        end = parse_sinex_epoch(fields[5], allow_open=True)
        mean = parse_sinex_epoch(fields[6])
        assert start is not None and mean is not None
        if observation_epoch >= start and (end is None or observation_epoch < end):
            candidates.append((point, solution_id, start, end, mean))
    if len(candidates) != 1:
        raise ValueError(
            f"expected one valid SOLUTION/EPOCHS row for {station}, found {len(candidates)}"
        )
    point, solution_id, start, end, mean = candidates[0]

    components: dict[str, tuple[float, float, datetime]] = {}
    for line in estimate_rows:
        fields = line.split()
        if len(fields) < 10:
            continue
        parameter, site, row_point, row_solution = (
            fields[1], fields[2].upper(), fields[3], fields[4]
        )
        if (
            site != station
            or row_point != point
            or row_solution != solution_id
            or parameter not in {"STAX", "STAY", "STAZ"}
        ):
            continue
        epoch = parse_sinex_epoch(fields[5])
        assert epoch is not None
        value, sigma = float(fields[8]), float(fields[9])
        if not math.isfinite(value) or not math.isfinite(sigma) or sigma < 0.0:
            raise ValueError(f"invalid {parameter} estimate for {station}")
        if parameter in components:
            raise ValueError(f"duplicate {parameter} estimate for {station}")
        components[parameter] = (value, sigma, epoch)
    if set(components) != {"STAX", "STAY", "STAZ"}:
        raise ValueError(f"incomplete XYZ estimate for {station} solution {solution_id}")
    estimate_epochs = {row[2] for row in components.values()}
    if len(estimate_epochs) != 1:
        raise ValueError(f"XYZ estimate epochs disagree for {station}")
    return SinexStationEstimate(
        station=station,
        point=point,
        solution_id=solution_id,
        validity_start=start,
        validity_end=end,
        mean_epoch=mean,
        estimate_epoch=next(iter(estimate_epochs)),
        ecef_m=tuple(components[name][0] for name in ("STAX", "STAY", "STAZ")),
        sigma_m=tuple(components[name][1] for name in ("STAX", "STAY", "STAZ")),
    )
