#!/usr/bin/env python3
"""Launch-free Phase128 GLONASS parser/admission primitives.

The functions in this module operate only on caller-supplied synthetic text or
metadata.  They deliberately have no route-file, solver, truth, MAT, or
Kaggle access.  A future authorized runner may use the same value-level
contract after its independent inventory boundary, but this module itself is
safe to import in pre-raw tests.
"""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timedelta, timezone
import math
import re
from typing import Any, Iterable, Sequence


SELECTOR = "--native-phase128-glonass-provenance-parser-admission"
MIN_FCN = -7
MAX_FCN = 6
MAX_AGE_S = 1800.0
TIE_TOLERANCE_S = 1.0e-9
GPS_EPOCH = datetime(1980, 1, 6, tzinfo=timezone.utc)
GPS_UTC_LEAP_SECONDS = 18.0


class Phase128ContractError(ValueError):
    """A parser/admission contract violation; callers must fail closed."""


@dataclass(frozen=True, order=True)
class SatelliteId:
    """Typed system/PRN key; a bare integer is never a valid key here."""

    system: str
    prn: int

    def __post_init__(self) -> None:
        if self.system != "R" or not isinstance(self.prn, int) or not 1 <= self.prn <= 27:
            raise Phase128ContractError(f"invalid GLONASS SatelliteId: {self.system!r}/{self.prn!r}")


@dataclass(frozen=True)
class CanonicalGephRecord:
    satellite: SatelliteId
    toe_gpst: tuple[int, float]
    fields: tuple[float, ...]
    fcn: int


@dataclass(frozen=True)
class ParseLedger:
    records_seen: int
    accepted_records: tuple[CanonicalGephRecord, ...]
    rejected_records: int
    reject_counts: dict[str, int]


@dataclass(frozen=True)
class HeaderLedger:
    status: str
    entries: tuple[tuple[SatelliteId, int], ...]
    label_lines: int
    malformed_entries: int
    reject_counts: dict[str, int]


def _reject(reason: str, detail: str) -> Phase128ContractError:
    return Phase128ContractError(f"{reason}: {detail}")


def canonical_fcn(raw: float) -> int:
    """Normalize canonical data[10] and apply the signed source domain."""

    if not math.isfinite(raw):
        raise _reject("fcn-nonfinite", "data[10] is not finite")
    normalized = raw - 256.0 if raw > 128.0 else raw
    if not math.isfinite(normalized) or normalized != math.floor(normalized):
        raise _reject("fcn-nonintegral", "data[10] is not integral")
    channel = int(normalized)
    if not MIN_FCN <= channel <= MAX_FCN:
        raise _reject("fcn-out-of-range", f"{channel} outside [{MIN_FCN},{MAX_FCN}]")
    return channel


def decode_canonical_geph(fields: Sequence[float]) -> tuple[tuple[float, ...], int]:
    """Validate exactly 15 positional fields, retaining blanks as rejection."""

    if len(fields) != 15:
        raise _reject("field-count", f"expected 15, got {len(fields)}")
    values = tuple(float(value) for value in fields)
    if not math.isfinite(values[10]):
        raise _reject("fcn-nonfinite", "data[10] is not finite")
    for index, value in enumerate(values):
        if not math.isfinite(value):
            raise _reject("nonfinite-field", f"data[{index}] is not finite")
    return values, canonical_fcn(values[10])


def _parse_fixed(line: str, start: int, width: int = 19) -> float:
    if start + width > len(line):
        raise _reject("field-missing", f"column {start}:{start + width}")
    text = line[start : start + width].strip().replace("D", "E").replace("d", "E")
    if not text:
        raise _reject("field-missing", f"column {start}:{start + width}")
    try:
        value = float(text)
    except ValueError as exc:
        raise _reject("field-malformed", text) from exc
    if not math.isfinite(value):
        raise _reject("field-nonfinite", text)
    return value


def _calendar_to_gpst(time_text: str, rinex_version: float) -> tuple[int, float]:
    """Convert the native RINEX calendar field to a GPST week/tow key."""

    tokens = time_text.replace("D", "E").split()
    if len(tokens) < 6:
        raise _reject("time-malformed", time_text)
    year = int(tokens[0])
    if rinex_version < 3.0:
        year += 1900 if year >= 80 else 2000
    month, day, hour, minute = (int(token) for token in tokens[1:5])
    second = float(tokens[5])
    whole_second = int(math.floor(second))
    micros = int(round((second - whole_second) * 1_000_000.0))
    utc = datetime(year, month, day, hour, minute, whole_second,
                   micros, tzinfo=timezone.utc)
    total_utc = (utc - GPS_EPOCH).total_seconds()
    week = int(math.floor(total_utc / 604800.0))
    tow = total_utc - week * 604800.0
    # Native GLONASS records use the nearest 900-second UTC frame before the
    # UTC->GPST conversion; retain the same source boundary here.
    rounded_tow = math.floor((tow + 450.0) / 900.0) * 900.0
    while rounded_tow >= 604800.0:
        rounded_tow -= 604800.0
        week += 1
    while rounded_tow < 0.0:
        rounded_tow += 604800.0
        week -= 1
    leap_seconds = 0
    for year_cutover, month_cutover, day_cutover, seconds in (
        (1981, 7, 1, 1), (1982, 7, 1, 2), (1983, 7, 1, 3),
        (1985, 7, 1, 4), (1988, 1, 1, 5), (1990, 1, 1, 6),
        (1991, 1, 1, 7), (1992, 7, 1, 8), (1993, 7, 1, 9),
        (1994, 7, 1, 10), (1996, 1, 1, 11), (1997, 7, 1, 12),
        (1999, 1, 1, 13), (2006, 1, 1, 14), (2009, 1, 1, 15),
        (2012, 7, 1, 16), (2015, 7, 1, 17), (2017, 1, 1, 18),
    ):
        if (year, month, day) >= (year_cutover, month_cutover, day_cutover):
            leap_seconds = seconds
    total_gpst = week * 604800.0 + rounded_tow + leap_seconds
    gpst_week = int(math.floor(total_gpst / 604800.0))
    return gpst_week, total_gpst - gpst_week * 604800.0


def _start_record(line: str, rinex_version: float) -> tuple[SatelliteId, str, int, int, int] | None:
    if rinex_version >= 3.0:
        match = re.match(r"^R(\d{1,2})(.*)$", line)
        if not match:
            return None
        prn = int(match.group(1))
        # RINEX 3: [sat(3), calendar(20), fields at 23/42/61].
        return SatelliteId("R", prn), line[3:23], 23, 42, 61
    # RINEX 2 has no system character.  This GLONASS-only parser accepts the
    # explicit Rnn spelling as well as a two-column numeric PRN fixture.
    explicit = re.match(r"^\s*R(\d{1,2})(.*)$", line)
    if explicit:
        return SatelliteId("R", int(explicit.group(1))), line[3:22], 22, 41, 60
    numeric = re.match(r"^\s*(\d{1,2})(.*)$", line)
    if numeric:
        return SatelliteId("R", int(numeric.group(1))), line[2:21], 22, 41, 60
    return None


def parse_nav_geph_records(lines: Iterable[str], rinex_version: float) -> ParseLedger:
    """Parse GLONASS records independently; one bad record never poisons others."""

    records = list(lines)
    accepted: list[CanonicalGephRecord] = []
    reject_counts: dict[str, int] = {}
    seen = 0
    cursor = 0

    def reject_record(reason: str) -> None:
        reject_counts[reason] = reject_counts.get(reason, 0) + 1

    while cursor < len(records):
        start = _start_record(records[cursor], rinex_version)
        if start is None:
            cursor += 1
            continue
        seen += 1
        satellite, time_text, c0, c1, c2 = start
        continuation = (4, 23, 42, 61) if rinex_version >= 3.0 else (3, 22, 41, 60)
        block = records[cursor : cursor + 4]
        if len(block) < 4:
            reject_record("record-lines-missing")
            cursor += 1
            continue
        try:
            toe = _calendar_to_gpst(time_text, rinex_version)
            fields = [
                _parse_fixed(block[0], c0), _parse_fixed(block[0], c1), _parse_fixed(block[0], c2),
                _parse_fixed(block[1], continuation[0]),
                _parse_fixed(block[1], continuation[1]),
                _parse_fixed(block[1], continuation[2]),
                _parse_fixed(block[1], continuation[3]),
                _parse_fixed(block[2], continuation[0]),
                _parse_fixed(block[2], continuation[1]),
                _parse_fixed(block[2], continuation[2]),
                _parse_fixed(block[2], continuation[3]),
                _parse_fixed(block[3], continuation[0]),
                _parse_fixed(block[3], continuation[1]),
                _parse_fixed(block[3], continuation[2]),
                _parse_fixed(block[3], continuation[3]),
            ]
            canonical, fcn = decode_canonical_geph(fields)
        except Phase128ContractError as exc:
            reason = str(exc).split(":", 1)[0]
            reject_record(reason)
            cursor += 4
            continue
        accepted.append(CanonicalGephRecord(satellite, toe, canonical, fcn))
        cursor += 4
    return ParseLedger(seen, tuple(accepted), seen - len(accepted), reject_counts)


def parse_glonass_header(lines: Iterable[str]) -> HeaderLedger:
    """Parse fixed 7-column RINEX header slots with explicit status states."""

    entries: list[tuple[SatelliteId, int]] = []
    reject_counts: dict[str, int] = {}
    labels = 0

    def reject(reason: str) -> None:
        reject_counts[reason] = reject_counts.get(reason, 0) + 1

    for line in lines:
        if len(line) < 60 or "GLONASS SLOT / FRQ #" not in line[60:]:
            continue
        labels += 1
        for index in range(8):
            start = 4 + index * 7
            slot = line[start : start + 7]
            if not slot.strip():
                continue
            if len(slot) < 7 or slot[0] != "R":
                reject("malformed-slot")
                continue
            try:
                prn_text = slot[1:3].strip()
                fcn_text = slot[4:7].strip()
                if not prn_text or not fcn_text:
                    raise ValueError("empty")
                sat = SatelliteId("R", int(prn_text))
                raw = float(fcn_text.replace("D", "E").replace("d", "E"))
                fcn = canonical_fcn(raw)
            except (ValueError, Phase128ContractError) as exc:
                reason = "malformed-slot" if isinstance(exc, ValueError) else str(exc).split(":", 1)[0]
                reject(reason)
                continue
            entries.append((sat, fcn))
    if labels == 0:
        status = "absent"
    elif reject_counts:
        status = "malformed"
    elif not entries:
        status = "valid-empty"
    else:
        status = "entries"
    return HeaderLedger(status, tuple(entries), labels, sum(reject_counts.values()), reject_counts)


def _query_seconds(query: tuple[int, float], toe: tuple[int, float]) -> float:
    return (query[0] - toe[0]) * 604800.0 + query[1] - toe[1]


def admit_records(
    records: Sequence[CanonicalGephRecord],
    observations: Sequence[dict[str, Any]],
    header: HeaderLedger | None = None,
) -> dict[str, Any]:
    """Admit each typed observation against exact GPST geph/header provenance."""

    if header is None:
        header = HeaderLedger("absent", (), 0, 0, {})
    if header.status == "malformed":
        raise _reject("header-malformed", "GLONASS header ledger")
    header_map: dict[SatelliteId, list[int]] = {}
    for satellite, channel in header.entries:
        header_map.setdefault(satellite, []).append(channel)
    certified = 0
    failures: dict[str, int] = {}

    def fail(reason: str) -> None:
        failures[reason] = failures.get(reason, 0) + 1

    for observation in observations:
        try:
            satellite = observation["satellite"]
            if not isinstance(satellite, SatelliteId):
                raise _reject("typed-key-required", "observation satellite")
            query = (int(observation["week"]), float(observation["tow"]))
            if not math.isfinite(query[1]):
                raise _reject("query-time-invalid", "nonfinite GPST")
            candidates = [
                record for record in records
                if record.satellite == satellite and
                abs(_query_seconds(query, record.toe_gpst)) <= MAX_AGE_S
            ]
            if not candidates:
                fail("query-time-coverage-gap")
                continue
            ages = [abs(_query_seconds(query, record.toe_gpst)) for record in candidates]
            minimum = min(ages)
            tied = [record for record, age in zip(candidates, ages)
                    if abs(age - minimum) <= TIE_TOLERANCE_S]
            channels = {record.fcn for record in tied}
            if len(channels) > 1:
                fail("different-fcn-tie")
                continue
            selected = tied[0]
            header_channels = header_map.get(satellite, [])
            if header_channels and len(set(header_channels)) > 1:
                fail("header-fcn-conflict")
                continue
            if header_channels and header_channels[0] != selected.fcn:
                fail("header-geph-fcn-mismatch")
                continue
            # carrierFrequencyHz is intentionally ignored.  It may be present
            # in observation metadata, but cannot certify GLONASS FCN.
            certified += 1
        except (KeyError, TypeError, ValueError, Phase128ContractError) as exc:
            reason = str(exc).split(":", 1)[0]
            fail(reason)
    return {
        "query_rows": len(observations),
        "certified_rows": certified,
        "unresolved_rows": len(observations) - certified,
        "full_coverage": certified == len(observations),
        "failure_counts": failures,
        "header_status": header.status,
        "phone_carrier_frequency_used_as_fcn": False,
    }


def zero_pre_raw_accounting() -> dict[str, Any]:
    """Static accounting object used by launch-free contract tests."""

    return {
        "raw_phone_gnss_reads": 0,
        "raw_phone_imu_reads": 0,
        "broadcast_navigation_reads": 0,
        "raw_base_rinex_reads": 0,
        "solver_invocations": 0,
        "truth_reads": 0,
        "mat_reads_or_generated": 0,
        "kaggle_or_token_access": 0,
        "raw_content_copied_or_transformed": False,
    }


if __name__ == "__main__":
    # No input path is accepted.  This command is intentionally a launch-free
    # smoke check over an in-memory empty header only.
    print({"selector": SELECTOR, "header": parse_glonass_header([]).__dict__,
           "read_accounting": zero_pre_raw_accounting()})
