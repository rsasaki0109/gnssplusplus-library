#!/usr/bin/env python3
"""Compare a LibGNSS++ .pos solution against a fixed ECEF truth."""

from __future__ import annotations

import argparse
import math
from dataclasses import dataclass
from pathlib import Path


WGS84_A = 6378137.0
WGS84_F = 1.0 / 298.257223563
WGS84_E2 = 2.0 * WGS84_F - WGS84_F * WGS84_F


@dataclass(frozen=True)
class Epoch:
    week: int
    tow: float
    x: float
    y: float
    z: float
    status: int
    satellites: int


def ecef_to_llh(x: float, y: float, z: float) -> tuple[float, float, float]:
    lon = math.atan2(y, x)
    p = math.hypot(x, y)
    lat = math.atan2(z, p * (1.0 - WGS84_E2))
    height = 0.0
    for _ in range(10):
        sin_lat = math.sin(lat)
        n = WGS84_A / math.sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat)
        height = p / max(math.cos(lat), 1e-12) - n
        lat = math.atan2(z, p * (1.0 - WGS84_E2 * n / (n + height)))
    sin_lat = math.sin(lat)
    n = WGS84_A / math.sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat)
    height = p / max(math.cos(lat), 1e-12) - n
    return lat, lon, height


def ecef_delta_to_enu(
    dx: float,
    dy: float,
    dz: float,
    lat_rad: float,
    lon_rad: float,
) -> tuple[float, float, float]:
    sin_lat = math.sin(lat_rad)
    cos_lat = math.cos(lat_rad)
    sin_lon = math.sin(lon_rad)
    cos_lon = math.cos(lon_rad)
    east = -sin_lon * dx + cos_lon * dy
    north = -sin_lat * cos_lon * dx - sin_lat * sin_lon * dy + cos_lat * dz
    up = cos_lat * cos_lon * dx + cos_lat * sin_lon * dy + sin_lat * dz
    return east, north, up


def read_pos(path: Path) -> list[Epoch]:
    epochs: list[Epoch] = []
    with path.open(encoding="utf-8") as handle:
        for line in handle:
            stripped = line.strip()
            if not stripped or stripped.startswith("%") or stripped.startswith("#"):
                continue
            parts = stripped.split()
            if len(parts) < 10:
                continue
            try:
                epochs.append(
                    Epoch(
                        week=int(float(parts[0])),
                        tow=float(parts[1]),
                        x=float(parts[2]),
                        y=float(parts[3]),
                        z=float(parts[4]),
                        status=int(float(parts[8])),
                        satellites=int(float(parts[9])),
                    )
                )
            except ValueError:
                continue
    return epochs


def rms(values: list[float]) -> float:
    return math.sqrt(sum(value * value for value in values) / len(values))


def percentile(values: list[float], pct: float) -> float:
    ordered = sorted(values)
    if not ordered:
        return float("nan")
    index = (len(ordered) - 1) * pct / 100.0
    lower = math.floor(index)
    upper = math.ceil(index)
    if lower == upper:
        return ordered[lower]
    weight = index - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--candidate", type=Path, required=True, help="LibGNSS++ .pos file")
    parser.add_argument("--ecef-x", type=float, required=True, help="Truth ECEF X in meters")
    parser.add_argument("--ecef-y", type=float, required=True, help="Truth ECEF Y in meters")
    parser.add_argument("--ecef-z", type=float, required=True, help="Truth ECEF Z in meters")
    parser.add_argument("--last-n", type=int, default=None, help="Only evaluate the last N epochs")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    epochs = read_pos(args.candidate)
    if args.last_n is not None:
        if args.last_n <= 0:
            raise SystemExit("--last-n must be positive")
        epochs = epochs[-args.last_n :]
    if not epochs:
        raise SystemExit(f"No solution epochs read from {args.candidate}")

    ref_lat, ref_lon, _ = ecef_to_llh(args.ecef_x, args.ecef_y, args.ecef_z)
    horizontal: list[float] = []
    vertical: list[float] = []
    three_d: list[float] = []
    for epoch in epochs:
        dx = epoch.x - args.ecef_x
        dy = epoch.y - args.ecef_y
        dz = epoch.z - args.ecef_z
        east, north, up = ecef_delta_to_enu(dx, dy, dz, ref_lat, ref_lon)
        horizontal.append(math.hypot(east, north))
        vertical.append(up)
        three_d.append(math.sqrt(dx * dx + dy * dy + dz * dz))

    fixed_epochs = sum(1 for epoch in epochs if epoch.status in (4, 6))
    float_epochs = sum(1 for epoch in epochs if epoch.status in (3, 5))
    print(f"epochs: {len(epochs)}")
    print(f"fixed_epochs: {fixed_epochs}")
    print(f"float_epochs: {float_epochs}")
    print(f"mean_satellites: {sum(epoch.satellites for epoch in epochs) / len(epochs):.6f}")
    print(f"rms_h_m: {rms(horizontal):.6f}")
    print(f"rms_v_m: {rms(vertical):.6f}")
    print(f"rms_3d_m: {rms(three_d):.6f}")
    print(f"median_h_m: {percentile(horizontal, 50.0):.6f}")
    print(f"median_abs_v_m: {percentile([abs(value) for value in vertical], 50.0):.6f}")
    print(f"median_3d_m: {percentile(three_d, 50.0):.6f}")
    print(f"p95_3d_m: {percentile(three_d, 95.0):.6f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
