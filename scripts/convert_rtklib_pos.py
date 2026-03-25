#!/usr/bin/env python3
"""Convert raw RTKLIB POS output into libgnss++ POS format."""

from __future__ import annotations

import argparse
from datetime import datetime
import math
import os
from pathlib import Path


WGS84_A = 6378137.0
WGS84_F = 1.0 / 298.257223563
WGS84_E2 = 2 * WGS84_F - WGS84_F * WGS84_F
GPS_EPOCH = datetime(1980, 1, 6)
STATUS_MAP = {1: 4, 2: 3, 4: 2, 5: 1}


def llh_to_ecef(lat_deg: float, lon_deg: float, height_m: float) -> tuple[float, float, float]:
    lat = math.radians(lat_deg)
    lon = math.radians(lon_deg)
    sin_lat = math.sin(lat)
    cos_lat = math.cos(lat)
    n = WGS84_A / math.sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat)
    x = (n + height_m) * cos_lat * math.cos(lon)
    y = (n + height_m) * cos_lat * math.sin(lon)
    z = (n * (1.0 - WGS84_E2) + height_m) * sin_lat
    return x, y, z


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(prog=os.environ.get("GNSS_CLI_NAME"))
    parser.add_argument("input_rtklib_pos", type=Path, help="Input RTKLIB .pos file")
    parser.add_argument("output_pos", type=Path, help="Output libgnss++ .pos file")
    return parser.parse_args()


def parse_gpst(date_token: str, time_token: str) -> tuple[int, float]:
    stamp = datetime.strptime(f"{date_token} {time_token}", "%Y/%m/%d %H:%M:%S.%f")
    delta = stamp - GPS_EPOCH
    week = delta.days // 7
    tow = (delta.days % 7) * 86400 + delta.seconds + delta.microseconds / 1e6
    return week, tow


def convert_rtklib_pos(input_file: Path, output_file: Path) -> int:
    line_count = 0
    output_file.parent.mkdir(parents=True, exist_ok=True)
    with input_file.open("r", encoding="utf-8") as infile, output_file.open("w", encoding="utf-8") as outfile:
        outfile.write("% LibGNSS++ Position Solution\n")
        outfile.write("% Converted from RTKLIB POS format\n")
        outfile.write("% Columns: GPS_Week GPS_TOW X(m) Y(m) Z(m) Lat(deg) Lon(deg) Height(m) Status Satellites PDOP\n")

        for line in infile:
            line = line.strip()
            if not line or line.startswith("%"):
                continue

            parts = line.split()
            if len(parts) < 7:
                continue

            try:
                week, tow = parse_gpst(parts[0], parts[1])
                lat = float(parts[2])
                lon = float(parts[3])
                height = float(parts[4])
                rtklib_q = int(parts[5])
                satellites = int(parts[6])
                x, y, z = llh_to_ecef(lat, lon, height)
                status = STATUS_MAP.get(rtklib_q, 1)
                outfile.write(
                    f"{week} {tow:.3f} "
                    f"{x:.4f} {y:.4f} {z:.4f} "
                    f"{lat:.9f} {lon:.9f} {height:.4f} "
                    f"{status} {satellites} 0.0\n"
                )
                line_count += 1
            except (ValueError, IndexError):
                print(f"Warning: Skipping invalid line: {line}")

    return line_count


def main() -> int:
    args = parse_args()
    if not args.input_rtklib_pos.exists():
        print(f"Error: Input file not found: {args.input_rtklib_pos}")
        return 1

    print(f"Converting RTKLIB POS file: {args.input_rtklib_pos}")
    print(f"Output file: {args.output_pos}")
    converted = convert_rtklib_pos(args.input_rtklib_pos, args.output_pos)
    print(f"Converted {converted} position solutions")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
