#!/usr/bin/env python3
"""Convert GICI NMEA output to the libgnss++ PPC ``.pos`` schema."""

from __future__ import annotations

import argparse
import datetime as dt
import math
from pathlib import Path


GPS_EPOCH = dt.datetime(1980, 1, 6)
WGS84_A = 6378137.0
WGS84_E2 = 6.69437999014e-3

# GGA quality -> RTKLIB solution status. In particular, GGA 4 is integer-fixed
# RTK and RTKLIB represents FIX with status 1.
GGA_TO_RTKLIB_STATUS = {1: 5, 2: 4, 3: 5, 4: 1, 5: 2, 6: 7}
GGA_TO_LIBGNSS_STATUS = {1: 1, 2: 2, 3: 1, 4: 4, 5: 3, 6: 1}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("nmea", type=Path, help="GICI NMEA solution file")
    parser.add_argument("output", type=Path, help="output libgnss++ .pos file")
    parser.add_argument(
        "--gps-utc-leap-seconds",
        type=int,
        default=18,
        help="GPST-UTC offset during the dataset (default: 18)",
    )
    parser.add_argument(
        "--libgnss-status",
        action="store_true",
        help="write libgnss++ status codes (FIX=4, FLOAT=3) instead of RTKLIB codes",
    )
    return parser.parse_args()


def degrees_minutes(value: str, hemisphere: str) -> float:
    raw = float(value)
    degrees = int(raw // 100)
    result = degrees + (raw - degrees * 100) / 60.0
    return -result if hemisphere in {"S", "W"} else result


def llh_to_ecef(lat_deg: float, lon_deg: float, height_m: float) -> tuple[float, float, float]:
    lat = math.radians(lat_deg)
    lon = math.radians(lon_deg)
    sin_lat = math.sin(lat)
    cos_lat = math.cos(lat)
    prime_vertical = WGS84_A / math.sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat)
    return (
        (prime_vertical + height_m) * cos_lat * math.cos(lon),
        (prime_vertical + height_m) * cos_lat * math.sin(lon),
        (prime_vertical * (1.0 - WGS84_E2) + height_m) * sin_lat,
    )


def utc_stamp(date_token: str, time_token: str) -> dt.datetime:
    day = int(date_token[0:2])
    month = int(date_token[2:4])
    year_2d = int(date_token[4:6])
    year = 2000 + year_2d if year_2d < 80 else 1900 + year_2d
    hour = int(time_token[0:2])
    minute = int(time_token[2:4])
    second_float = float(time_token[4:])
    second = int(second_float)
    microsecond = round((second_float - second) * 1_000_000)
    if microsecond == 1_000_000:
        second += 1
        microsecond = 0
    return dt.datetime(year, month, day, hour, minute, second, microsecond)


def convert(
    nmea_path: Path,
    output_path: Path,
    leap_seconds: int,
    *,
    libgnss_status: bool = False,
) -> int:
    current_date: str | None = None
    output_path.parent.mkdir(parents=True, exist_ok=True)
    count = 0
    with nmea_path.open(encoding="ascii", errors="strict") as source, output_path.open(
        "w", encoding="ascii", newline="\n"
    ) as output:
        output.write("% week tow x-ecef(m) y-ecef(m) z-ecef(m) lat(deg) lon(deg) h(m) Q ns sdn(m)\n")
        for line_number, line in enumerate(source, 1):
            payload = line.strip().split("*", 1)[0]
            fields = payload.split(",")
            sentence = fields[0][-3:] if fields else ""
            if sentence == "RMC" and len(fields) > 9 and fields[9]:
                current_date = fields[9]
                continue
            if sentence != "GGA" or len(fields) < 15 or not fields[1]:
                continue
            if current_date is None:
                raise ValueError(f"GGA before dated RMC at {nmea_path}:{line_number}")

            quality = int(fields[6])
            lat = degrees_minutes(fields[2], fields[3])
            lon = degrees_minutes(fields[4], fields[5])
            # GGA altitude is orthometric; field 11 is geoid separation.
            height = float(fields[9]) + float(fields[11])
            stamp = utc_stamp(current_date, fields[1])
            gps_seconds = (stamp - GPS_EPOCH).total_seconds() + leap_seconds
            week = int(gps_seconds // 604800)
            tow = gps_seconds - week * 604800
            x, y, z = llh_to_ecef(lat, lon, height)
            status_map = GGA_TO_LIBGNSS_STATUS if libgnss_status else GGA_TO_RTKLIB_STATUS
            status = status_map.get(quality, 0)
            satellites = int(fields[7])
            output.write(
                f"{week:d} {tow:.3f} {x:.4f} {y:.4f} {z:.4f} "
                f"{lat:.10f} {lon:.10f} {height:.4f} {status:d} {satellites:d} 0.0\n"
            )
            count += 1
    return count


def main() -> int:
    args = parse_args()
    count = convert(
        args.nmea,
        args.output,
        args.gps_utc_leap_seconds,
        libgnss_status=args.libgnss_status,
    )
    print(f"Converted {count} GGA epochs to {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
