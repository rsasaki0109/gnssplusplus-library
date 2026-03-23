#!/usr/bin/env python3
"""
Generate a README-ready comparison figure for the UrbanNav Tokyo Odaiba dataset.
"""

from __future__ import annotations

import argparse
import bisect
import csv
import datetime as dt
import math
from dataclasses import dataclass
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


GPS_EPOCH = dt.datetime(1980, 1, 6)
WGS84_A = 6378137.0
WGS84_E2 = 6.69437999014e-3


@dataclass(frozen=True)
class ReferenceEpoch:
    week: int
    tow: float
    lat_deg: float
    lon_deg: float
    height_m: float
    ecef: np.ndarray


@dataclass(frozen=True)
class SolutionEpoch:
    week: int
    tow: float
    lat_deg: float
    lon_deg: float
    height_m: float
    ecef: np.ndarray
    status: int
    num_satellites: int


@dataclass(frozen=True)
class MatchedEpoch:
    tow: float
    east_m: float
    north_m: float
    up_m: float
    horiz_error_m: float
    status: int


def llh_to_ecef(lat_deg: float, lon_deg: float, height_m: float) -> np.ndarray:
    lat = math.radians(lat_deg)
    lon = math.radians(lon_deg)
    sin_lat = math.sin(lat)
    cos_lat = math.cos(lat)
    sin_lon = math.sin(lon)
    cos_lon = math.cos(lon)
    N = WGS84_A / math.sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat)
    return np.array(
        [
            (N + height_m) * cos_lat * cos_lon,
            (N + height_m) * cos_lat * sin_lon,
            (N * (1.0 - WGS84_E2) + height_m) * sin_lat,
        ]
    )


def ecef_to_enu(delta_ecef: np.ndarray, ref_lat_deg: float, ref_lon_deg: float) -> np.ndarray:
    lat = math.radians(ref_lat_deg)
    lon = math.radians(ref_lon_deg)
    sin_lat = math.sin(lat)
    cos_lat = math.cos(lat)
    sin_lon = math.sin(lon)
    cos_lon = math.cos(lon)
    rot = np.array(
        [
            [-sin_lon, cos_lon, 0.0],
            [-sin_lat * cos_lon, -sin_lat * sin_lon, cos_lat],
            [cos_lat * cos_lon, cos_lat * sin_lon, sin_lat],
        ]
    )
    return rot @ delta_ecef


def read_reference_csv(path: Path) -> list[ReferenceEpoch]:
    rows: list[ReferenceEpoch] = []
    with path.open(newline="") as handle:
        reader = csv.reader(handle)
        next(reader)  # header
        for row in reader:
            tow = float(row[0])
            week = int(row[1])
            lat = float(row[2])
            lon = float(row[3])
            height = float(row[4])
            ecef = np.array([float(row[5]), float(row[6]), float(row[7])])
            rows.append(ReferenceEpoch(week, tow, lat, lon, height, ecef))
    return rows


def parse_rtklib_timestamp(token_a: str, token_b: str) -> tuple[int, float]:
    if "/" not in token_a:
        return int(float(token_a)), float(token_b)

    year, month, day = map(int, token_a.split("/"))
    hour, minute, second = token_b.split(":")
    sec_float = float(second)
    second_int = int(sec_float)
    microsecond = int(round((sec_float - second_int) * 1_000_000))
    stamp = dt.datetime(year, month, day, int(hour), int(minute), second_int, microsecond)
    total_seconds = (stamp - GPS_EPOCH).total_seconds()
    week = int(total_seconds // 604800)
    tow = total_seconds - week * 604800
    return week, tow


def read_libgnss_pos(path: Path) -> list[SolutionEpoch]:
    rows: list[SolutionEpoch] = []
    with path.open() as handle:
        for line in handle:
            if not line.strip() or line.startswith("%"):
                continue
            parts = line.split()
            if len(parts) < 11:
                continue
            week = int(float(parts[0]))
            tow = float(parts[1])
            ecef = np.array([float(parts[2]), float(parts[3]), float(parts[4])])
            lat = float(parts[5])
            lon = float(parts[6])
            height = float(parts[7])
            status = int(parts[8])
            num_satellites = int(parts[9])
            rows.append(SolutionEpoch(week, tow, lat, lon, height, ecef, status, num_satellites))
    return rows


def read_rtklib_pos(path: Path) -> list[SolutionEpoch]:
    rows: list[SolutionEpoch] = []
    with path.open() as handle:
        for line in handle:
            if not line.strip() or line.startswith("%"):
                continue
            parts = line.split()
            if len(parts) < 7:
                continue
            week, tow = parse_rtklib_timestamp(parts[0], parts[1])
            lat = float(parts[2])
            lon = float(parts[3])
            height = float(parts[4])
            status = int(parts[5])
            num_satellites = int(parts[6])
            ecef = llh_to_ecef(lat, lon, height)
            rows.append(SolutionEpoch(week, tow, lat, lon, height, ecef, status, num_satellites))
    return rows


def match_to_reference(
    solutions: list[SolutionEpoch],
    reference: list[ReferenceEpoch],
    tolerance_s: float,
) -> list[MatchedEpoch]:
    ref_tows = [epoch.tow for epoch in reference]
    matched: list[MatchedEpoch] = []
    for epoch in solutions:
        idx = bisect.bisect_left(ref_tows, epoch.tow)
        candidates = [
            reference[j]
            for j in (idx - 1, idx, idx + 1)
            if 0 <= j < len(reference)
        ]
        if not candidates:
            continue
        ref = min(candidates, key=lambda item: abs(item.tow - epoch.tow))
        if ref.week != epoch.week or abs(ref.tow - epoch.tow) > tolerance_s:
            continue
        enu = ecef_to_enu(epoch.ecef - ref.ecef, ref.lat_deg, ref.lon_deg)
        matched.append(
            MatchedEpoch(
                tow=epoch.tow,
                east_m=float(enu[0]),
                north_m=float(enu[1]),
                up_m=float(enu[2]),
                horiz_error_m=float(np.linalg.norm(enu[:2])),
                status=epoch.status,
            )
        )
    matched.sort(key=lambda epoch: epoch.tow)
    return matched


def trajectory_enu(
    epochs: list[ReferenceEpoch] | list[SolutionEpoch],
    origin: ReferenceEpoch,
) -> np.ndarray:
    enu_points = []
    for epoch in epochs:
        delta = epoch.ecef - origin.ecef
        enu = ecef_to_enu(delta, origin.lat_deg, origin.lon_deg)
        enu_points.append(enu)
    return np.vstack(enu_points) if enu_points else np.empty((0, 3))


def summarize(matched: list[MatchedEpoch], fixed_status: int, label: str) -> dict[str, float]:
    if not matched:
        raise SystemExit(f"No epochs matched reference.csv for {label}")
    horiz = np.array([epoch.horiz_error_m for epoch in matched])
    fix_count = sum(epoch.status == fixed_status for epoch in matched)
    return {
        "epochs": len(matched),
        "fix_rate_pct": 100.0 * fix_count / len(matched),
        "median_h_m": float(np.median(horiz)),
        "p95_h_m": float(np.percentile(horiz, 95)),
        "max_h_m": float(np.max(horiz)),
    }


def cdf_xy(matched: list[MatchedEpoch]) -> tuple[np.ndarray, np.ndarray]:
    horiz = np.sort(np.array([epoch.horiz_error_m for epoch in matched]))
    pct = np.linspace(0.0, 100.0, len(horiz), endpoint=False)
    return horiz, pct


def matched_segments(
    matched: list[MatchedEpoch],
    max_gap_s: float,
) -> list[np.ndarray]:
    if not matched:
        return []

    segments: list[np.ndarray] = []
    start = 0
    for idx in range(1, len(matched) + 1):
        split = idx == len(matched)
        if not split:
            split = (matched[idx].tow - matched[idx - 1].tow) > max_gap_s
        if split:
            segment = matched[start:idx]
            if len(segment) >= 2:
                segments.append(
                    np.array([[epoch.east_m, epoch.north_m] for epoch in segment])
                )
            start = idx
    return segments


def plot_solver_trajectory(
    ax: plt.Axes,
    matched: list[MatchedEpoch],
    color: str,
    label: str,
    fixed_status: int,
    max_gap_s: float,
) -> None:
    segments = matched_segments(matched, max_gap_s)
    line_labeled = False
    for segment in segments:
        ax.plot(
            segment[:, 0],
            segment[:, 1],
            color=color,
            linewidth=1.2,
            alpha=0.8,
            label=label if not line_labeled else None,
        )
        line_labeled = True

    non_fixed = np.array(
        [
            [epoch.east_m, epoch.north_m]
            for epoch in matched
            if epoch.status != fixed_status
        ]
    )
    fixed = np.array(
        [
            [epoch.east_m, epoch.north_m]
            for epoch in matched
            if epoch.status == fixed_status
        ]
    )

    if len(non_fixed):
        ax.scatter(
            non_fixed[:, 0],
            non_fixed[:, 1],
            s=8,
            color=color,
            alpha=0.20,
            linewidths=0.0,
        )
    if len(fixed):
        ax.scatter(
            fixed[:, 0],
            fixed[:, 1],
            s=10,
            color=color,
            alpha=0.85,
            label=f"{label} fixed",
        )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--lib-pos", type=Path, required=True)
    parser.add_argument("--rtklib-pos", type=Path, required=True)
    parser.add_argument("--reference-csv", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--title", default="Urban Driving RTK Comparison")
    parser.add_argument("--match-tolerance", type=float, default=0.15)
    args = parser.parse_args()

    reference = read_reference_csv(args.reference_csv)
    lib_epochs = read_libgnss_pos(args.lib_pos)
    rtklib_epochs = read_rtklib_pos(args.rtklib_pos)
    origin = reference[0]
    max_gap_s = 2.0

    ref_enu = trajectory_enu(reference, origin)
    lib_matched = match_to_reference(lib_epochs, reference, args.match_tolerance)
    rtklib_matched = match_to_reference(rtklib_epochs, reference, args.match_tolerance)

    lib_summary = summarize(lib_matched, fixed_status=4, label="libgnss++")
    rtklib_summary = summarize(rtklib_matched, fixed_status=1, label="RTKLIB")

    fig, axes = plt.subplots(2, 2, figsize=(14, 10))

    ax = axes[0, 0]
    ax.plot(ref_enu[:, 0], ref_enu[:, 1], color="black", linewidth=2.0, label="Ground truth")
    plot_solver_trajectory(
        ax,
        rtklib_matched,
        color="#1f77b4",
        label="RTKLIB matched",
        fixed_status=1,
        max_gap_s=max_gap_s,
    )
    plot_solver_trajectory(
        ax,
        lib_matched,
        color="#d97706",
        label="libgnss++ matched",
        fixed_status=4,
        max_gap_s=max_gap_s,
    )
    ax.set_title("Trajectory Overlay (matched epochs only)")
    ax.set_xlabel("East (m)")
    ax.set_ylabel("North (m)")
    ax.grid(alpha=0.3)
    ax.axis("equal")
    ax.legend(loc="best", fontsize=9)

    ax = axes[0, 1]
    lib_t = np.array([epoch.tow for epoch in lib_matched])
    rtklib_t = np.array([epoch.tow for epoch in rtklib_matched])
    lib_h = np.array([epoch.horiz_error_m for epoch in lib_matched])
    rtklib_h = np.array([epoch.horiz_error_m for epoch in rtklib_matched])
    ax.semilogy((rtklib_t - rtklib_t[0]) / 60.0, rtklib_h, color="#1f77b4", linewidth=1.1, label="RTKLIB")
    ax.semilogy((lib_t - lib_t[0]) / 60.0, lib_h, color="#d97706", linewidth=1.1, label="libgnss++")
    ax.set_title("Horizontal Error vs Ground Truth")
    ax.set_xlabel("Time from start (min)")
    ax.set_ylabel("Horizontal error (m, log scale)")
    ax.grid(alpha=0.3)
    ax.legend(loc="upper right")

    ax = axes[1, 0]
    rtklib_x, rtklib_pct = cdf_xy(rtklib_matched)
    lib_x, lib_pct = cdf_xy(lib_matched)
    ax.semilogx(rtklib_x, rtklib_pct, color="#1f77b4", linewidth=1.4, label="RTKLIB")
    ax.semilogx(lib_x, lib_pct, color="#d97706", linewidth=1.4, label="libgnss++")
    ax.set_title("Horizontal Error CDF")
    ax.set_xlabel("Horizontal error (m, log scale)")
    ax.set_ylabel("CDF (%)")
    ax.grid(alpha=0.3)
    ax.legend(loc="lower right")

    ax = axes[1, 1]
    ax.axis("off")
    text = "\n".join(
        [
            "Dataset: UrbanNav Tokyo Odaiba (open)",
            "Ground truth: reference.csv (Applanix POS LV620)",
            "",
            f"libgnss++  matched={lib_summary['epochs']}",
            f"  fix rate={lib_summary['fix_rate_pct']:.1f}%",
            f"  median h={lib_summary['median_h_m']:.2f} m",
            f"  p95 h={lib_summary['p95_h_m']:.2f} m",
            f"  max h={lib_summary['max_h_m']:.1f} m",
            "",
            f"RTKLIB     matched={rtklib_summary['epochs']}",
            f"  fix rate={rtklib_summary['fix_rate_pct']:.1f}%",
            f"  median h={rtklib_summary['median_h_m']:.2f} m",
            f"  p95 h={rtklib_summary['p95_h_m']:.2f} m",
            f"  max h={rtklib_summary['max_h_m']:.1f} m",
            "",
            "RTKLIB config:",
            "  GPS-only, L1+L2, kinematic, continuous AR",
            "",
            f"Trajectory panel: matched epochs only, gaps > {max_gap_s:.0f}s are not connected",
        ]
    )
    ax.text(0.0, 1.0, text, va="top", ha="left", family="monospace", fontsize=10.5)

    fig.suptitle(args.title, fontsize=16)
    fig.tight_layout()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output, dpi=180, bbox_inches="tight")
    print(f"Saved: {args.output}")
    print(f"libgnss++: {lib_summary}")
    print(f"RTKLIB: {rtklib_summary}")


if __name__ == "__main__":
    main()
