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
import os
from dataclasses import dataclass
from pathlib import Path
import numpy as np


GPS_EPOCH = dt.datetime(1980, 1, 6)
WGS84_A = 6378137.0
WGS84_E2 = 6.69437999014e-3

SOLVER_LINE_COLORS = {
    "RTKLIB": "#2563eb",
    "libgnss++": "#d97706",
}

SOLVER_MARKERS = {
    "RTKLIB": "s",
    "libgnss++": "o",
}

STATUS_ORDER = ("FIXED", "FLOAT", "DGPS", "SPP")

STATUS_STYLES = {
    "FIXED": "#2ecc71",
    "FLOAT": "#f39c12",
    "DGPS": "#3498db",
    "SPP": "#e74c3c",
}

STATUS_MAPS = {
    "RTKLIB": {
        1: "FIXED",
        2: "FLOAT",
        4: "DGPS",
        5: "SPP",
    },
    "libgnss++": {
        4: "FIXED",
        3: "FLOAT",
        2: "DGPS",
        1: "SPP",
    },
}


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
    traj_east_m: float
    traj_north_m: float
    traj_up_m: float
    east_m: float
    north_m: float
    up_m: float
    horiz_error_m: float
    status: int


@dataclass(frozen=True)
class PairedEpoch:
    tow: float
    lib_epoch: MatchedEpoch
    rtklib_epoch: MatchedEpoch
    gap_m: float


def llh_to_ecef(lat_deg: float, lon_deg: float, height_m: float) -> np.ndarray:
    lat = math.radians(lat_deg)
    lon = math.radians(lon_deg)
    sin_lat = math.sin(lat)
    cos_lat = math.cos(lat)
    sin_lon = math.sin(lon)
    cos_lon = math.cos(lon)
    n = WGS84_A / math.sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat)
    return np.array(
        [
            (n + height_m) * cos_lat * cos_lon,
            (n + height_m) * cos_lat * sin_lon,
            (n * (1.0 - WGS84_E2) + height_m) * sin_lat,
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
        next(reader)
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
        traj_enu = ecef_to_enu(epoch.ecef - reference[0].ecef, reference[0].lat_deg, reference[0].lon_deg)
        enu = ecef_to_enu(epoch.ecef - ref.ecef, ref.lat_deg, ref.lon_deg)
        matched.append(
            MatchedEpoch(
                tow=epoch.tow,
                traj_east_m=float(traj_enu[0]),
                traj_north_m=float(traj_enu[1]),
                traj_up_m=float(traj_enu[2]),
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
    up_signed = np.array([epoch.up_m for epoch in matched])
    up_abs = np.abs(up_signed)
    fix_count = sum(epoch.status == fixed_status for epoch in matched)
    return {
        "epochs": len(matched),
        "fix_rate_pct": 100.0 * fix_count / len(matched),
        "median_h_m": float(np.median(horiz)),
        "p95_h_m": float(np.percentile(horiz, 95)),
        "max_h_m": float(np.max(horiz)),
        "median_abs_up_m": float(np.median(up_abs)),
        "p95_abs_up_m": float(np.percentile(up_abs, 95)),
        "mean_up_m": float(np.mean(up_signed)),
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
                    np.array([[epoch.traj_east_m, epoch.traj_north_m] for epoch in segment])
                )
            start = idx
    return segments


def status_style(solver: str, status: int) -> tuple[str, str]:
    status_name = STATUS_MAPS.get(solver, {}).get(status, f"status {status}")
    return status_name, STATUS_STYLES.get(status_name, "#64748b")


def pair_epochs(
    lib_matched: list[MatchedEpoch],
    rtklib_matched: list[MatchedEpoch],
    tolerance_s: float,
) -> list[PairedEpoch]:
    pairs: list[PairedEpoch] = []
    lib_idx = 0
    rt_idx = 0
    while lib_idx < len(lib_matched) and rt_idx < len(rtklib_matched):
        lib_epoch = lib_matched[lib_idx]
        rt_epoch = rtklib_matched[rt_idx]
        dt = lib_epoch.tow - rt_epoch.tow
        if abs(dt) <= tolerance_s:
            pairs.append(
                PairedEpoch(
                    tow=lib_epoch.tow,
                    lib_epoch=lib_epoch,
                    rtklib_epoch=rt_epoch,
                    gap_m=rt_epoch.horiz_error_m - lib_epoch.horiz_error_m,
                )
            )
            lib_idx += 1
            rt_idx += 1
            continue
        if dt < 0.0:
            lib_idx += 1
        else:
            rt_idx += 1
    return pairs


def summarize_common_epochs(
    pairs: list[PairedEpoch],
    lib_fixed_status: int,
    rtklib_fixed_status: int,
) -> tuple[dict[str, float], dict[str, float]]:
    if not pairs:
        raise SystemExit("No common libgnss++ / RTKLIB epochs for apples-to-apples summary")
    lib_common = [pair.lib_epoch for pair in pairs]
    rtklib_common = [pair.rtklib_epoch for pair in pairs]
    return (
        summarize(lib_common, fixed_status=lib_fixed_status, label="libgnss++ common epochs"),
        summarize(rtklib_common, fixed_status=rtklib_fixed_status, label="RTKLIB common epochs"),
    )


def select_zoom_window(
    lib_matched: list[MatchedEpoch],
    rtklib_matched: list[MatchedEpoch],
    tolerance_s: float,
) -> tuple[float, float, PairedEpoch]:
    pairs = pair_epochs(lib_matched, rtklib_matched, tolerance_s)
    if not pairs:
        raise SystemExit("No overlapping libgnss++ / RTKLIB epochs for zoom selection")

    def score(pair: PairedEpoch) -> float:
        bonus = 0.0
        if pair.lib_epoch.status == 4 and pair.rtklib_epoch.status != 1:
            bonus += 12.0
        elif pair.lib_epoch.status in (3, 4) and pair.rtklib_epoch.status == 5:
            bonus += 6.0
        return pair.gap_m + bonus

    selected = max(pairs, key=score)
    half_window_s = 3.0
    return selected.tow - half_window_s, selected.tow + half_window_s, selected


def filter_window(epochs: list[MatchedEpoch], start_tow: float, end_tow: float) -> list[MatchedEpoch]:
    return [epoch for epoch in epochs if start_tow <= epoch.tow <= end_tow]


def reference_window(reference: list[ReferenceEpoch], start_tow: float, end_tow: float) -> list[ReferenceEpoch]:
    return [epoch for epoch in reference if start_tow <= epoch.tow <= end_tow]


def plot_solver_trajectory(
    ax: plt.Axes,
    matched: list[MatchedEpoch],
    solver: str,
    max_gap_s: float,
    point_size: float,
) -> None:
    line_color = SOLVER_LINE_COLORS[solver]
    marker = SOLVER_MARKERS[solver]
    segments = matched_segments(matched, max_gap_s)
    for segment in segments:
        ax.plot(
            segment[:, 0],
            segment[:, 1],
            color=line_color,
            linewidth=0.9 if point_size < 20 else 1.15,
            alpha=0.15 if point_size < 20 else 0.20,
            zorder=1,
        )

    status_rank = {name: idx for idx, name in enumerate(STATUS_ORDER)}
    ordered_statuses = sorted(
        {epoch.status for epoch in matched},
        key=lambda value: status_rank.get(status_style(solver, value)[0], len(STATUS_ORDER)),
    )
    for status in ordered_statuses:
        points = np.array(
            [
                [epoch.traj_east_m, epoch.traj_north_m]
                for epoch in matched
                if epoch.status == status
            ]
        )
        if not len(points):
            continue
        status_name, status_color = status_style(solver, status)
        ax.scatter(
            points[:, 0],
            points[:, 1],
            s=point_size if status_name != "FIXED" else point_size * 1.15,
            marker=marker,
            facecolor=status_color,
            edgecolor="white",
            linewidth=0.25 if point_size < 20 else 0.45,
            alpha=0.95 if status_name == "FIXED" else 0.86,
            zorder=5 if status_name == "FIXED" else 4,
        )


def solver_proxy_label(solver: str) -> str:
    return f"{solver} track"


def build_status_legend_handles() -> list[object]:
    from matplotlib.lines import Line2D

    handles: list[object] = []
    for status_name in STATUS_ORDER:
        handles.append(
            Line2D(
                [0],
                [0],
                marker="o",
                linestyle="None",
                markersize=8,
                markerfacecolor=STATUS_STYLES[status_name],
                markeredgecolor="white",
                markeredgewidth=0.5,
                label=status_name,
            )
        )
    return handles


def build_solver_legend_handles() -> list[object]:
    from matplotlib.lines import Line2D

    return [
        Line2D([0], [0], color="black", linewidth=2.0, label="Ground truth"),
        Line2D(
            [0],
            [0],
            color=SOLVER_LINE_COLORS["RTKLIB"],
            marker=SOLVER_MARKERS["RTKLIB"],
            markersize=6,
            linewidth=1.2,
            alpha=0.7,
            label="RTKLIB",
        ),
        Line2D(
            [0],
            [0],
            color=SOLVER_LINE_COLORS["libgnss++"],
            marker=SOLVER_MARKERS["libgnss++"],
            markersize=6,
            linewidth=1.2,
            alpha=0.7,
            label="libgnss++",
        ),
    ]


def matched_xy(matched: list[MatchedEpoch]) -> np.ndarray:
    if not matched:
        return np.empty((0, 2))
    return np.array([[epoch.traj_east_m, epoch.traj_north_m] for epoch in matched])


def padded_xy_limits(*point_sets: np.ndarray, min_pad: float = 8.0) -> tuple[float, float, float, float]:
    valid_sets = [points for points in point_sets if len(points)]
    if not valid_sets:
        return -1.0, 1.0, -1.0, 1.0
    all_points = np.vstack(valid_sets)
    xmin, ymin = np.min(all_points, axis=0)
    xmax, ymax = np.max(all_points, axis=0)
    span = max(float(xmax - xmin), float(ymax - ymin), 1.0)
    pad = max(min_pad, span * 0.04)
    return float(xmin - pad), float(xmax + pad), float(ymin - pad), float(ymax + pad)


def status_breakdown_text(matched: list[MatchedEpoch], solver: str, fixed_status: int) -> str:
    counts = {name: 0 for name in STATUS_ORDER}
    for epoch in matched:
        status_name, _ = status_style(solver, epoch.status)
        counts[status_name] = counts.get(status_name, 0) + 1
    return "\n".join(
        [
            f"matched={len(matched)}",
            f"fix={100.0 * sum(epoch.status == fixed_status for epoch in matched) / max(len(matched), 1):.1f}%",
            f"FIXED {counts.get('FIXED', 0)}",
            f"FLOAT {counts.get('FLOAT', 0)}",
            f"DGPS  {counts.get('DGPS', 0)}",
            f"SPP   {counts.get('SPP', 0)}",
        ]
    )


def main() -> None:
    parser = argparse.ArgumentParser(prog=os.environ.get("GNSS_CLI_NAME"))
    parser.add_argument("--lib-pos", type=Path, required=True)
    parser.add_argument("--rtklib-pos", type=Path, required=True)
    parser.add_argument("--reference-csv", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--title", default="Urban Driving RTK Comparison")
    parser.add_argument("--match-tolerance", type=float, default=0.15)
    args = parser.parse_args()

    global plt
    import matplotlib.pyplot as plt

    reference = read_reference_csv(args.reference_csv)
    lib_epochs = read_libgnss_pos(args.lib_pos)
    rtklib_epochs = read_rtklib_pos(args.rtklib_pos)
    origin = reference[0]
    max_gap_s = 2.0

    ref_enu = trajectory_enu(reference, origin)
    lib_matched = match_to_reference(lib_epochs, reference, args.match_tolerance)
    rtklib_matched = match_to_reference(rtklib_epochs, reference, args.match_tolerance)
    common_pairs = pair_epochs(lib_matched, rtklib_matched, tolerance_s=args.match_tolerance)
    zoom_start, zoom_end, zoom_pair = select_zoom_window(
        lib_matched,
        rtklib_matched,
        tolerance_s=args.match_tolerance,
    )

    ref_zoom = reference_window(reference, zoom_start, zoom_end)
    ref_zoom_enu = trajectory_enu(ref_zoom, origin)
    lib_zoom = filter_window(lib_matched, zoom_start, zoom_end)
    rtklib_zoom = filter_window(rtklib_matched, zoom_start, zoom_end)

    lib_summary = summarize(lib_matched, fixed_status=4, label="libgnss++")
    rtklib_summary = summarize(rtklib_matched, fixed_status=1, label="RTKLIB")
    lib_common_summary, rtklib_common_summary = summarize_common_epochs(
        common_pairs,
        lib_fixed_status=4,
        rtklib_fixed_status=1,
    )

    fig = plt.figure(figsize=(19, 12.6))
    grid = fig.add_gridspec(3, 4, height_ratios=[1.0, 0.95, 0.82], hspace=0.34, wspace=0.28)

    ax_rtk = fig.add_subplot(grid[0, 0:2])
    ax_lib = fig.add_subplot(grid[0, 2:4])
    ax_zoom = fig.add_subplot(grid[1, 0:2])
    ax_text = fig.add_subplot(grid[1, 2:4])
    ax_h = fig.add_subplot(grid[2, 0])
    ax_up = fig.add_subplot(grid[2, 1])
    ax_cdf = fig.add_subplot(grid[2, 2:4])

    full_xmin, full_xmax, full_ymin, full_ymax = padded_xy_limits(
        ref_enu[:, :2],
        matched_xy(rtklib_matched),
        matched_xy(lib_matched),
        min_pad=12.0,
    )

    ax = ax_rtk
    ax.plot(ref_enu[:, 0], ref_enu[:, 1], color="black", linewidth=2.0)
    plot_solver_trajectory(ax, rtklib_matched, "RTKLIB", max_gap_s=max_gap_s, point_size=14)
    ax.set_title("RTKLIB 2D Trajectory")
    ax.set_xlabel("East (m)")
    ax.set_ylabel("North (m)")
    ax.grid(alpha=0.3)
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlim(full_xmin, full_xmax)
    ax.set_ylim(full_ymin, full_ymax)
    ax.text(
        0.02,
        0.98,
        status_breakdown_text(rtklib_matched, "RTKLIB", fixed_status=1),
        transform=ax.transAxes,
        va="top",
        ha="left",
        family="monospace",
        fontsize=8.8,
        bbox=dict(boxstyle="round,pad=0.3", facecolor="white", alpha=0.82),
    )

    ax = ax_lib
    ax.plot(ref_enu[:, 0], ref_enu[:, 1], color="black", linewidth=2.0)
    plot_solver_trajectory(ax, lib_matched, "libgnss++", max_gap_s=max_gap_s, point_size=14)
    ax.set_title("libgnss++ 2D Trajectory")
    ax.set_xlabel("East (m)")
    ax.set_ylabel("North (m)")
    ax.grid(alpha=0.3)
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlim(full_xmin, full_xmax)
    ax.set_ylim(full_ymin, full_ymax)
    ax.text(
        0.02,
        0.98,
        status_breakdown_text(lib_matched, "libgnss++", fixed_status=4),
        transform=ax.transAxes,
        va="top",
        ha="left",
        family="monospace",
        fontsize=8.8,
        bbox=dict(boxstyle="round,pad=0.3", facecolor="white", alpha=0.82),
    )

    ax = ax_zoom
    if len(ref_zoom_enu):
        ax.plot(ref_zoom_enu[:, 0], ref_zoom_enu[:, 1], color="black", linewidth=2.0, label="Ground truth")
    plot_solver_trajectory(
        ax,
        rtklib_zoom,
        "RTKLIB",
        max_gap_s=max_gap_s,
        point_size=40,
    )
    plot_solver_trajectory(
        ax,
        lib_zoom,
        "libgnss++",
        max_gap_s=max_gap_s,
        point_size=40,
    )
    all_zoom_points = np.vstack(
        [
            ref_zoom_enu[:, :2] if len(ref_zoom_enu) else np.empty((0, 2)),
            np.array([[epoch.traj_east_m, epoch.traj_north_m] for epoch in lib_zoom]) if lib_zoom else np.empty((0, 2)),
            np.array([[epoch.traj_east_m, epoch.traj_north_m] for epoch in rtklib_zoom]) if rtklib_zoom else np.empty((0, 2)),
        ]
    )
    xmin, ymin = np.min(all_zoom_points, axis=0)
    xmax, ymax = np.max(all_zoom_points, axis=0)
    pad = 10.0
    ax.set_xlim(float(xmin - pad), float(xmax + pad))
    ax.set_ylim(float(ymin - pad), float(ymax + pad))
    ax.set_title(
        "Zoom: RTKLIB weak / libgnss++ stable\n"
        f"tow {zoom_start:.1f}-{zoom_end:.1f}s, lib={zoom_pair.lib_epoch.horiz_error_m:.1f}m, RTKLIB={zoom_pair.rtklib_epoch.horiz_error_m:.1f}m"
    )
    ax.set_xlabel("East (m)")
    ax.set_ylabel("North (m)")
    ax.grid(alpha=0.3)
    ax.set_aspect("equal", adjustable="box")
    solver_legend = ax.legend(handles=build_solver_legend_handles(), loc="upper right", fontsize=7.8)
    ax.add_artist(solver_legend)
    ax.legend(handles=build_status_legend_handles(), loc="lower right", fontsize=7.8, ncol=2)

    ax = ax_text
    ax.axis("off")
    text = "\n".join(
        [
            "Dataset: UrbanNav Tokyo Odaiba (open)",
            "Ground truth: reference.csv (Applanix POS LV620)",
            "",
            "All matched epochs:",
            f"  libgnss++ matched={lib_summary['epochs']}",
            f"  fix rate={lib_summary['fix_rate_pct']:.1f}%",
            f"  median h={lib_summary['median_h_m']:.3f} m",
            f"  p95 h={lib_summary['p95_h_m']:.2f} m",
            f"  p95 |up|={lib_summary['p95_abs_up_m']:.2f} m",
            f"  mean up={lib_summary['mean_up_m']:+.2f} m",
            "",
            f"  RTKLIB    matched={rtklib_summary['epochs']}",
            f"  fix rate={rtklib_summary['fix_rate_pct']:.1f}%",
            f"  median h={rtklib_summary['median_h_m']:.3f} m",
            f"  p95 h={rtklib_summary['p95_h_m']:.2f} m",
            f"  p95 |up|={rtklib_summary['p95_abs_up_m']:.2f} m",
            f"  mean up={rtklib_summary['mean_up_m']:+.2f} m",
            "",
            f"Common epochs only: {int(lib_common_summary['epochs'])}",
            f"  libgnss++ median h={lib_common_summary['median_h_m']:.3f} m",
            f"  RTKLIB    median h={rtklib_common_summary['median_h_m']:.3f} m",
            f"  libgnss++ median |up|={lib_common_summary['median_abs_up_m']:.3f} m",
            f"  RTKLIB    median |up|={rtklib_common_summary['median_abs_up_m']:.3f} m",
            "",
            "Status colors:",
            "  green=FIXED, orange=FLOAT, blue=DGPS, red=SPP/SINGLE",
            "Markers: circle=libgnss++, square=RTKLIB",
            "",
            "Comparison scope:",
            "  checked-in Odaiba libgnss++ vs RTKLIB .pos artifacts",
        ]
    )
    ax.text(0.0, 1.0, text, va="top", ha="left", family="monospace", fontsize=10.2)

    ax = ax_h
    lib_t = np.array([epoch.tow for epoch in lib_matched])
    rtklib_t = np.array([epoch.tow for epoch in rtklib_matched])
    lib_h = np.array([epoch.horiz_error_m for epoch in lib_matched])
    rtklib_h = np.array([epoch.horiz_error_m for epoch in rtklib_matched])
    ax.semilogy((rtklib_t - rtklib_t[0]) / 60.0, rtklib_h, color=SOLVER_LINE_COLORS["RTKLIB"], linewidth=1.1, label="RTKLIB")
    ax.semilogy((lib_t - lib_t[0]) / 60.0, lib_h, color=SOLVER_LINE_COLORS["libgnss++"], linewidth=1.1, label="libgnss++")
    ax.set_title("Horizontal Error vs Ground Truth")
    ax.set_xlabel("Time from start (min)")
    ax.set_ylabel("Horizontal error (m, log scale)")
    ax.grid(alpha=0.3)
    ax.legend(loc="upper right")

    ax = ax_up
    lib_up = np.array([epoch.up_m for epoch in lib_matched])
    rtklib_up = np.array([epoch.up_m for epoch in rtklib_matched])
    ax.plot((rtklib_t - rtklib_t[0]) / 60.0, rtklib_up, color=SOLVER_LINE_COLORS["RTKLIB"], linewidth=1.0, label="RTKLIB")
    ax.plot((lib_t - lib_t[0]) / 60.0, lib_up, color=SOLVER_LINE_COLORS["libgnss++"], linewidth=1.0, label="libgnss++")
    ax.axhline(0.0, color="#475569", linewidth=0.9, linestyle="--", alpha=0.8)
    ax.set_title("Signed Vertical Error vs Ground Truth")
    ax.set_xlabel("Time from start (min)")
    ax.set_ylabel("Up error (m)")
    ax.grid(alpha=0.3)
    ax.legend(loc="upper right")

    ax = ax_cdf
    rtklib_x, rtklib_pct = cdf_xy(rtklib_matched)
    lib_x, lib_pct = cdf_xy(lib_matched)
    ax.semilogx(rtklib_x, rtklib_pct, color=SOLVER_LINE_COLORS["RTKLIB"], linewidth=1.4, label="RTKLIB")
    ax.semilogx(lib_x, lib_pct, color=SOLVER_LINE_COLORS["libgnss++"], linewidth=1.4, label="libgnss++")
    ax.set_title("Horizontal Error CDF")
    ax.set_xlabel("Horizontal error (m, log scale)")
    ax.set_ylabel("CDF (%)")
    ax.grid(alpha=0.3)
    ax.legend(loc="lower right")

    fig.suptitle(args.title, fontsize=16)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output, dpi=180, bbox_inches="tight")
    print(f"Saved: {args.output}")
    print(f"libgnss++: {lib_summary}")
    print(f"RTKLIB: {rtklib_summary}")


if __name__ == "__main__":
    main()
