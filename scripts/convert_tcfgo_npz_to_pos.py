#!/usr/bin/env python3
"""Convert the external tightly-coupled FGO NPZ into PPC ``.pos`` format.

Only estimator outputs and GNSS timestamps are consumed.  Diagnostic truth
arrays that may be present in the NPZ are deliberately ignored.
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path

import numpy as np


WGS84_A = 6378137.0
WGS84_E2 = 6.69437999014e-3


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input_npz", type=Path)
    parser.add_argument("output_pos", type=Path)
    return parser.parse_args()


def ecef_to_llh(ecef: np.ndarray) -> tuple[float, float, float]:
    x_m, y_m, z_m = (float(value) for value in ecef)
    lon = math.atan2(y_m, x_m)
    p = math.hypot(x_m, y_m)
    lat = math.atan2(z_m, p * (1.0 - WGS84_E2))
    for _ in range(8):
        sin_lat = math.sin(lat)
        radius = WGS84_A / math.sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat)
        height = p / max(abs(math.cos(lat)), 1e-12) - radius
        lat = math.atan2(
            z_m, p * (1.0 - WGS84_E2 * radius / (radius + height))
        )
    sin_lat = math.sin(lat)
    radius = WGS84_A / math.sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat)
    height = p / max(abs(math.cos(lat)), 1e-12) - radius
    return math.degrees(lat), math.degrees(lon), height


def _required(data: np.lib.npyio.NpzFile, key: str) -> np.ndarray:
    if key not in data:
        raise ValueError(f"NPZ is missing required estimator field: {key}")
    return np.asarray(data[key])


def convert(input_npz: Path, output_pos: Path) -> int:
    with np.load(input_npz, allow_pickle=False) as data:
        week = _required(data, "week")
        tow = _required(data, "tow")
        sol_xyz = _required(data, "sol_xyz")
        smode = _required(data, "smode")
        nb = np.asarray(data["nb"]) if "nb" in data else np.zeros(len(week))
        nsat = np.asarray(data["nsat"]) if "nsat" in data else np.zeros(len(week))

    count = len(week)
    if tow.shape != (count,) or smode.shape != (count,):
        raise ValueError("week, tow, and smode arrays must have equal length")
    if sol_xyz.shape != (count, 3):
        raise ValueError("sol_xyz must have shape (n, 3)")

    output_pos.parent.mkdir(parents=True, exist_ok=True)
    written = 0
    with output_pos.open("w", encoding="ascii", newline="\n") as output:
        output.write("% External TC-FGO oracle converted without truth inputs\n")
        output.write(
            "% GPS_Week GPS_TOW X(m) Y(m) Z(m) Lat(deg) Lon(deg) "
            "Height(m) Status NumSat PDOP Ratio\n"
        )
        for i in range(count):
            ecef = np.asarray(sol_xyz[i], dtype=float)
            if not np.all(np.isfinite(ecef)):
                continue
            lat, lon, height = ecef_to_llh(ecef)
            # The oracle NPZ encodes FIX as 4 and FLOAT as 5.  libgnss++ PPC
            # solution files use status 4 for FIXED and 3 for FLOAT.
            status = 4 if int(smode[i]) == 4 else 3 if int(smode[i]) == 5 else 0
            sat_value = nsat[i] if i < len(nsat) and np.isfinite(nsat[i]) else nb[i]
            num_satellites = max(0, int(sat_value))
            output.write(
                f"{int(week[i])} {float(tow[i]):.3f} "
                f"{ecef[0]:.4f} {ecef[1]:.4f} {ecef[2]:.4f} "
                f"{lat:.9f} {lon:.9f} {height:.4f} "
                f"{status} {num_satellites} 0.0 0.0\n"
            )
            written += 1
    return written


def main() -> int:
    args = parse_args()
    count = convert(args.input_npz, args.output_pos)
    print(f"Converted {count} TC-FGO epochs to {args.output_pos}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
