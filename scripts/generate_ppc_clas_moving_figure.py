#!/usr/bin/env python3
"""Generate the README PPC moving-CLAS comparison figure."""

from __future__ import annotations

import argparse
import bisect
import csv
from dataclasses import dataclass
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

import generate_driving_comparison as comparison


GPS_UTC_LEAP_SECONDS = 18.0
WGS84_A = 6378137.0
WGS84_E2 = 6.69437999014e-3


@dataclass(frozen=True)
class TimedPosition:
    utc_sod: float
    ecef: np.ndarray
    fixed: bool


def parse_reference(path: Path) -> list[TimedPosition]:
    rows: list[TimedPosition] = []
    with path.open(newline="") as handle:
        reader = csv.reader(handle)
        next(reader)
        for row in reader:
            tow = float(row[0])
            rows.append(
                TimedPosition(
                    utc_sod=(tow - GPS_UTC_LEAP_SECONDS) % 86400.0,
                    ecef=np.array([float(row[5]), float(row[6]), float(row[7])]),
                    fixed=True,
                )
            )
    return sorted(rows, key=lambda item: item.utc_sod)


def parse_libgnss_pos(path: Path) -> list[TimedPosition]:
    rows: list[TimedPosition] = []
    for epoch in comparison.read_libgnss_pos(path):
        rows.append(
            TimedPosition(
                utc_sod=(epoch.tow - GPS_UTC_LEAP_SECONDS) % 86400.0,
                ecef=epoch.ecef,
                fixed=epoch.status == 6,
            )
        )
    return sorted(rows, key=lambda item: item.utc_sod)


def nmea_degrees(value: str, hemisphere: str) -> float:
    raw = float(value)
    degrees = int(raw // 100)
    result = degrees + (raw - degrees * 100) / 60.0
    return -result if hemisphere in {"S", "W"} else result


def parse_nmea(path: Path) -> list[TimedPosition]:
    rows: list[TimedPosition] = []
    with path.open() as handle:
        for raw in handle:
            line = raw.strip().split("*", 1)[0]
            fields = line.split(",")
            if len(fields) < 12 or fields[0] not in {"$GPGGA", "$GNGGA"}:
                continue
            try:
                quality = int(fields[6])
                if quality == 0:
                    continue
                stamp = fields[1]
                utc_sod = (
                    int(stamp[0:2]) * 3600.0
                    + int(stamp[2:4]) * 60.0
                    + float(stamp[4:])
                )
                lat = nmea_degrees(fields[2], fields[3])
                lon = nmea_degrees(fields[4], fields[5])
                height = float(fields[9]) + float(fields[11] or 0.0)
            except (ValueError, IndexError):
                continue
            rows.append(
                TimedPosition(
                    utc_sod=utc_sod,
                    ecef=comparison.llh_to_ecef(lat, lon, height),
                    fixed=quality == 4,
                )
            )
    return sorted(rows, key=lambda item: item.utc_sod)


def trim_window(rows: list[TimedPosition], start: float, duration: float) -> list[TimedPosition]:
    return [row for row in rows if start <= row.utc_sod < start + duration]


def match_reference(
    positions: list[TimedPosition],
    reference: list[TimedPosition],
    tolerance_s: float = 0.15,
) -> list[tuple[TimedPosition, TimedPosition]]:
    ref_times = [row.utc_sod for row in reference]
    matched: list[tuple[TimedPosition, TimedPosition]] = []
    for position in positions:
        index = bisect.bisect_left(ref_times, position.utc_sod)
        candidates = [
            reference[i]
            for i in (index - 1, index)
            if 0 <= i < len(reference)
        ]
        if not candidates:
            continue
        truth = min(candidates, key=lambda item: abs(item.utc_sod - position.utc_sod))
        if abs(truth.utc_sod - position.utc_sod) <= tolerance_s:
            matched.append((position, truth))
    return matched


def enu_delta(ecef: np.ndarray, origin_ecef: np.ndarray, lat: float, lon: float) -> np.ndarray:
    return comparison.ecef_to_enu(ecef - origin_ecef, lat, lon)


def ecef_lat_lon(ecef: np.ndarray) -> tuple[float, float]:
    x, y, z = ecef
    lon = np.arctan2(y, x)
    radius = np.hypot(x, y)
    lat = np.arctan2(z, radius * (1.0 - WGS84_E2))
    for _ in range(8):
        sin_lat = np.sin(lat)
        prime_vertical = WGS84_A / np.sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat)
        lat = np.arctan2(z + WGS84_E2 * prime_vertical * sin_lat, radius)
    return float(np.degrees(lat)), float(np.degrees(lon))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--reference", type=Path, required=True)
    parser.add_argument("--libgnss-pos", type=Path, required=True)
    parser.add_argument("--mrtklib-nmea", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--duration", type=float, default=480.0)
    args = parser.parse_args()

    reference = parse_reference(args.reference)
    libgnss = parse_libgnss_pos(args.libgnss_pos)
    mrtklib = parse_nmea(args.mrtklib_nmea)
    start = min(libgnss[0].utc_sod, mrtklib[0].utc_sod)
    reference = trim_window(reference, start, args.duration)
    lib_pairs = match_reference(trim_window(libgnss, start, args.duration), reference)
    mrtk_pairs = match_reference(trim_window(mrtklib, start, args.duration), reference)

    if not reference or not lib_pairs or not mrtk_pairs:
        raise SystemExit("No matched PPC epochs available for plotting")

    origin = reference[0]
    lat0, lon0 = ecef_lat_lon(origin.ecef)
    truth_xy = np.array(
        [enu_delta(row.ecef, origin.ecef, lat0, lon0)[:2] for row in reference]
    )

    def series(pairs: list[tuple[TimedPosition, TimedPosition]]):
        xy = np.array(
            [enu_delta(position.ecef, origin.ecef, lat0, lon0)[:2] for position, _ in pairs]
        )
        times = np.array([position.utc_sod - start for position, _ in pairs])
        errors = np.array(
            [np.linalg.norm(enu_delta(position.ecef, truth.ecef, lat0, lon0)[:2])
             for position, truth in pairs]
        )
        fixed = np.array([position.fixed for position, _ in pairs], dtype=bool)
        return xy, times, errors, fixed

    lib_xy, lib_t, lib_error, lib_fixed = series(lib_pairs)
    mrtk_xy, mrtk_t, mrtk_error, mrtk_fixed = series(mrtk_pairs)

    plt.style.use("seaborn-v0_8-whitegrid")
    figure, (ax_route, ax_error) = plt.subplots(1, 2, figsize=(13.2, 5.6))
    ax_route.plot(truth_xy[:, 0], truth_xy[:, 1], color="#111827", linewidth=2.4,
                  label="PPC truth", zorder=3)
    ax_route.plot(mrtk_xy[:, 0], mrtk_xy[:, 1], color="#2563eb", linewidth=1.2,
                  alpha=0.85, label="MRTKLIB v0.5.1")
    ax_route.plot(lib_xy[:, 0], lib_xy[:, 1], color="#d97706", linewidth=1.2,
                  alpha=0.9, label="libgnss++")
    ax_route.scatter(lib_xy[lib_fixed, 0], lib_xy[lib_fixed, 1], s=9,
                     color="#16a34a", label="libgnss++ FIX", zorder=4)
    ax_route.set_aspect("equal", adjustable="datalim")
    ax_route.set_xlabel("East (m)")
    ax_route.set_ylabel("North (m)")
    ax_route.set_title("Tokyo run2 moving trajectory — first 480 s")
    ax_route.legend(loc="best", fontsize=8)

    ax_error.plot(mrtk_t, mrtk_error, color="#2563eb", linewidth=0.9,
                  label="MRTKLIB v0.5.1")
    ax_error.plot(lib_t, lib_error, color="#d97706", linewidth=0.9,
                  label="libgnss++")
    ax_error.scatter(mrtk_t[mrtk_fixed], mrtk_error[mrtk_fixed], s=8,
                     color="#60a5fa", label="MRTKLIB FIX", zorder=3)
    ax_error.scatter(lib_t[lib_fixed], lib_error[lib_fixed], s=10,
                     color="#16a34a", label="libgnss++ FIX", zorder=4)
    ax_error.axhline(1.0, color="#6b7280", linestyle="--", linewidth=0.9)
    ax_error.set_yscale("log")
    ax_error.set_ylim(0.03, max(200.0, float(np.nanmax([lib_error.max(), mrtk_error.max()]))))
    ax_error.set_xlabel("Elapsed time (s)")
    ax_error.set_ylabel("Horizontal error (m, log scale)")
    ax_error.set_title("Horizontal error and FIX epochs")
    ax_error.legend(loc="upper right", fontsize=8, ncol=2)

    figure.suptitle("PPC moving CLAS: libgnss++ vs MRTKLIB", fontsize=14, fontweight="bold")
    figure.tight_layout()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(args.output, dpi=180, bbox_inches="tight")
    plt.close(figure)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
