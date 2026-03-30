#!/usr/bin/env python3
"""Inspect IONEX ionosphere map files."""

from __future__ import annotations

import argparse
from datetime import datetime
import json
import os
from pathlib import Path
import sys


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(prog=os.environ.get("GNSS_CLI_NAME"))
    parser.add_argument("--input", type=Path, required=True, help="IONEX input file.")
    parser.add_argument(
        "--summary-json",
        type=Path,
        default=None,
        help="Optional JSON summary output path.",
    )
    return parser.parse_args()


def parse_epoch_fields(text: str) -> str | None:
    fields = text.split()
    if len(fields) < 6:
        return None
    try:
        stamp = datetime(
            int(float(fields[0])),
            int(float(fields[1])),
            int(float(fields[2])),
            int(float(fields[3])),
            int(float(fields[4])),
            int(float(fields[5])),
        )
    except ValueError:
        return None
    return stamp.isoformat(sep=" ")


def parse_ionex(path: Path) -> dict[str, object]:
    payload: dict[str, object] = {
        "input": str(path),
        "format": "IONEX",
        "version": None,
        "system": None,
        "map_count": 0,
        "rms_map_count": 0,
        "aux_dcb_count": 0,
        "first_map_epoch": None,
        "last_map_epoch": None,
        "interval_s": None,
        "map_dimension": None,
        "base_radius_km": None,
        "elevation_cutoff_deg": None,
        "mapping_function": None,
        "exponent": None,
        "lat_grid": None,
        "lon_grid": None,
        "hgt_grid": None,
    }

    in_header = True
    map_epochs: list[str] = []
    header_map_count: int | None = None
    actual_map_count = 0
    with path.open(encoding="ascii", errors="ignore") as handle:
        for line in handle:
            if in_header:
                if "IONEX VERSION / TYPE" in line:
                    fields = line[:20].split()
                    head_fields = line[:60].split()
                    if fields:
                        payload["version"] = fields[0]
                    if len(head_fields) >= 3:
                        payload["system"] = head_fields[2]
                elif "EPOCH OF FIRST MAP" in line:
                    payload["first_map_epoch"] = parse_epoch_fields(line[:43])
                elif "EPOCH OF LAST MAP" in line:
                    payload["last_map_epoch"] = parse_epoch_fields(line[:43])
                elif "INTERVAL" in line:
                    fields = line[:20].split()
                    if fields:
                        payload["interval_s"] = int(float(fields[0]))
                elif "# OF MAPS IN FILE" in line:
                    fields = line[:20].split()
                    if fields:
                        header_map_count = int(float(fields[0]))
                        payload["map_count"] = header_map_count
                elif "MAP DIMENSION" in line:
                    fields = line[:20].split()
                    if fields:
                        payload["map_dimension"] = int(float(fields[0]))
                elif "BASE RADIUS" in line:
                    fields = line[:20].split()
                    if fields:
                        payload["base_radius_km"] = float(fields[0])
                elif "ELEVATION CUTOFF" in line:
                    fields = line[:20].split()
                    if fields:
                        payload["elevation_cutoff_deg"] = float(fields[0])
                elif "MAPPING FUNCTION" in line:
                    payload["mapping_function"] = line[:20].strip() or None
                elif "EXPONENT" in line:
                    fields = line[:20].split()
                    if fields:
                        payload["exponent"] = int(float(fields[0]))
                elif "LAT1 / LAT2 / DLAT" in line:
                    fields = line[:30].split()
                    if len(fields) >= 3:
                        payload["lat_grid"] = [float(fields[0]), float(fields[1]), float(fields[2])]
                elif "LON1 / LON2 / DLON" in line:
                    fields = line[:30].split()
                    if len(fields) >= 3:
                        payload["lon_grid"] = [float(fields[0]), float(fields[1]), float(fields[2])]
                elif "HGT1 / HGT2 / DHGT" in line:
                    fields = line[:30].split()
                    if len(fields) >= 3:
                        payload["hgt_grid"] = [float(fields[0]), float(fields[1]), float(fields[2])]
                elif "PRN / BIAS / RMS" in line:
                    payload["aux_dcb_count"] = int(payload["aux_dcb_count"]) + 1
                elif "END OF HEADER" in line:
                    in_header = False
            else:
                if "START OF TEC MAP" in line:
                    actual_map_count += 1
                elif "START OF RMS MAP" in line:
                    payload["rms_map_count"] = int(payload["rms_map_count"]) + 1
                elif "EPOCH OF CURRENT MAP" in line:
                    epoch = parse_epoch_fields(line[:43])
                    if epoch is not None:
                        map_epochs.append(epoch)

    if map_epochs:
        payload["first_map_epoch"] = payload["first_map_epoch"] or map_epochs[0]
        payload["last_map_epoch"] = payload["last_map_epoch"] or map_epochs[-1]
    if header_map_count is None:
        payload["map_count"] = actual_map_count
    return payload


def main() -> int:
    args = parse_args()
    if not args.input.exists():
        raise SystemExit(f"Missing IONEX input: {args.input}")
    payload = parse_ionex(args.input)
    if args.summary_json is not None:
        args.summary_json.parent.mkdir(parents=True, exist_ok=True)
        args.summary_json.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")

    print(f"input: {payload['input']}")
    print(f"format: {payload['format']}")
    print(f"version: {payload['version']}")
    print(f"system: {payload['system']}")
    print(f"maps: {payload['map_count']}")
    print(f"rms maps: {payload['rms_map_count']}")
    print(f"aux dcb entries: {payload['aux_dcb_count']}")
    print(f"first map epoch: {payload['first_map_epoch']}")
    print(f"last map epoch: {payload['last_map_epoch']}")
    print(f"interval_s: {payload['interval_s']}")
    print(f"map dimension: {payload['map_dimension']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
