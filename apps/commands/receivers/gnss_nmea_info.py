#!/usr/bin/env python3
"""Inspect NMEA logs or serial streams and print decoded GGA/RMC summaries."""

from __future__ import annotations

import argparse
import os
from dataclasses import dataclass

from support.gnss_input_source import InputSource, NMEA_SERIAL_BAUDS


@dataclass
class GGARecord:
    time_utc: str
    latitude_deg: float
    longitude_deg: float
    quality: int
    satellites: int
    hdop: float
    altitude_m: float


@dataclass
class RMCRecord:
    time_utc: str
    status: str
    latitude_deg: float
    longitude_deg: float
    speed_knots: float
    course_deg: float
    date_ddmmyy: str


@dataclass
class Stats:
    total_sentences: int = 0
    checksum_errors: int = 0
    valid_sentences: int = 0
    gga_sentences: int = 0
    rmc_sentences: int = 0
    valid_positions: int = 0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(prog=os.environ.get("GNSS_CLI_NAME"))
    parser.add_argument(
        "--input",
        required=True,
        help="Input path (.nmea, .log, serial://..., or /dev/tty...).",
    )
    parser.add_argument(
        "--decode-gga",
        action="store_true",
        help="Print decoded GGA records.",
    )
    parser.add_argument(
        "--decode-rmc",
        action="store_true",
        help="Print decoded RMC records.",
    )
    parser.add_argument(
        "--limit",
        type=int,
        default=-1,
        help="Stop after decoding NMEA sentences (default: no limit).",
    )
    parser.add_argument(
        "--quiet",
        action="store_true",
        help="Suppress per-sentence printing and show only the summary.",
    )
    return parser.parse_args()


def nmea_checksum(payload: str) -> int:
    value = 0
    for character in payload:
        value ^= ord(character)
    return value


def parse_lat_lon(value: str, hemisphere: str) -> float:
    if not value or not hemisphere:
        raise ValueError("missing latitude/longitude")
    if hemisphere in ("N", "S"):
        degrees = int(value[:2])
        minutes = float(value[2:])
    else:
        degrees = int(value[:3])
        minutes = float(value[3:])
    decimal = degrees + minutes / 60.0
    if hemisphere in ("S", "W"):
        decimal *= -1.0
    return decimal


def parse_sentence(line: str, stats: Stats) -> tuple[str, object] | None:
    text = line.strip()
    if not text.startswith("$") or "*" not in text:
        return None
    stats.total_sentences += 1
    payload, checksum_text = text[1:].split("*", 1)
    try:
        expected = int(checksum_text[:2], 16)
    except ValueError:
        stats.checksum_errors += 1
        return None
    if nmea_checksum(payload) != expected:
        stats.checksum_errors += 1
        return None
    fields = payload.split(",")
    sentence_type = fields[0]
    stats.valid_sentences += 1
    if sentence_type.endswith("GGA"):
        stats.gga_sentences += 1
        record = GGARecord(
            time_utc=fields[1],
            latitude_deg=parse_lat_lon(fields[2], fields[3]),
            longitude_deg=parse_lat_lon(fields[4], fields[5]),
            quality=int(fields[6] or 0),
            satellites=int(fields[7] or 0),
            hdop=float(fields[8] or 0.0),
            altitude_m=float(fields[9] or 0.0),
        )
        if record.quality > 0:
            stats.valid_positions += 1
        return "GGA", record
    if sentence_type.endswith("RMC"):
        stats.rmc_sentences += 1
        record = RMCRecord(
            time_utc=fields[1],
            status=fields[2],
            latitude_deg=parse_lat_lon(fields[3], fields[4]),
            longitude_deg=parse_lat_lon(fields[5], fields[6]),
            speed_knots=float(fields[7] or 0.0),
            course_deg=float(fields[8] or 0.0),
            date_ddmmyy=fields[9],
        )
        if record.status == "A":
            stats.valid_positions += 1
        return "RMC", record
    return None


def format_record(kind: str, record: object) -> str:
    if kind == "GGA":
        assert isinstance(record, GGARecord)
        return (
            f"gga: time={record.time_utc} lat={record.latitude_deg:.7f} "
            f"lon={record.longitude_deg:.7f} quality={record.quality} "
            f"sats={record.satellites} hdop={record.hdop:.1f} alt={record.altitude_m:.2f}m"
        )
    assert isinstance(record, RMCRecord)
    return (
        f"rmc: time={record.time_utc} status={record.status} "
        f"lat={record.latitude_deg:.7f} lon={record.longitude_deg:.7f} "
        f"speed_knots={record.speed_knots:.2f} course_deg={record.course_deg:.2f} "
        f"date={record.date_ddmmyy}"
    )


def main() -> int:
    args = parse_args()
    if args.limit == 0:
        raise SystemExit("--limit must be positive or -1")

    source = InputSource(
        args.input,
        default_baud=9600,
        allowed_bauds=NMEA_SERIAL_BAUDS,
        eio_as_eof=False,
    )
    if not source.open():
        raise SystemExit(f"failed to open input: {args.input}")

    stats = Stats()
    decoded = 0
    buffer = b""
    try:
        while args.limit < 0 or decoded < args.limit:
            chunk = source.read()
            if not chunk:
                break
            buffer += chunk
            while b"\n" in buffer and (args.limit < 0 or decoded < args.limit):
                raw_line, buffer = buffer.split(b"\n", 1)
                line = raw_line.decode("ascii", errors="ignore").strip()
                parsed = parse_sentence(line, stats)
                if parsed is None:
                    continue
                kind, record = parsed
                decoded += 1
                should_print = not args.quiet and (
                    (kind == "GGA" and (args.decode_gga or (not args.decode_gga and not args.decode_rmc)))
                    or (kind == "RMC" and (args.decode_rmc or (not args.decode_gga and not args.decode_rmc)))
                )
                if should_print:
                    print(format_record(kind, record))
    finally:
        source.close()

    print(
        "summary: "
        f"sentences={stats.total_sentences} valid={stats.valid_sentences} "
        f"gga={stats.gga_sentences} rmc={stats.rmc_sentences} "
        f"valid_positions={stats.valid_positions} checksum_errors={stats.checksum_errors}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
