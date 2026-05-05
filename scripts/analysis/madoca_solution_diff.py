#!/usr/bin/env python3
"""Compare MADOCA/MADOCALIB and native PPP solution .pos files."""

from __future__ import annotations

import argparse
import csv
import datetime as dt
import json
import math
from collections import Counter
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Sequence


GPS_EPOCH = dt.datetime(1980, 1, 6)
GPS_WEEK_SECONDS = 604800.0
WGS84_A = 6378137.0
WGS84_E2 = 6.69437999014e-3
DEFAULT_EVENT_THRESHOLDS_M = (1.0, 10.0, 100.0, 1000.0)


@dataclass(frozen=True)
class SolutionEpoch:
    week: int
    tow: float
    x_m: float
    y_m: float
    z_m: float
    lat_deg: float
    lon_deg: float
    height_m: float
    status: int
    satellites: int


@dataclass(frozen=True)
class ReferenceFrame:
    x_m: float
    y_m: float
    z_m: float
    lat_deg: float
    lon_deg: float
    height_m: float


@dataclass(frozen=True)
class MatchedPair:
    base: SolutionEpoch
    candidate: SolutionEpoch
    dt_s: float
    base_error_enu_m: tuple[float, float, float]
    candidate_error_enu_m: tuple[float, float, float]
    delta_enu_m: tuple[float, float, float]


def is_finite_triplet(values: Sequence[float]) -> bool:
    return all(math.isfinite(value) for value in values)


def llh_to_ecef(lat_deg: float, lon_deg: float, height_m: float) -> tuple[float, float, float]:
    lat = math.radians(lat_deg)
    lon = math.radians(lon_deg)
    sin_lat = math.sin(lat)
    cos_lat = math.cos(lat)
    sin_lon = math.sin(lon)
    cos_lon = math.cos(lon)
    n = WGS84_A / math.sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat)
    return (
        (n + height_m) * cos_lat * cos_lon,
        (n + height_m) * cos_lat * sin_lon,
        (n * (1.0 - WGS84_E2) + height_m) * sin_lat,
    )


def ecef_to_llh(x_m: float, y_m: float, z_m: float) -> tuple[float, float, float]:
    lon = math.atan2(y_m, x_m)
    p = math.hypot(x_m, y_m)
    lat = math.atan2(z_m, p * (1.0 - WGS84_E2))
    height = 0.0
    for _ in range(10):
        sin_lat = math.sin(lat)
        n = WGS84_A / math.sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat)
        height = p / max(math.cos(lat), 1e-15) - n
        next_lat = math.atan2(z_m, p * (1.0 - WGS84_E2 * n / (n + height)))
        if abs(next_lat - lat) < 1e-14:
            lat = next_lat
            break
        lat = next_lat
    sin_lat = math.sin(lat)
    n = WGS84_A / math.sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat)
    height = p / max(math.cos(lat), 1e-15) - n
    return (math.degrees(lat), math.degrees(lon), height)


def ecef_delta_to_enu(
    delta_x_m: float,
    delta_y_m: float,
    delta_z_m: float,
    ref_lat_deg: float,
    ref_lon_deg: float,
) -> tuple[float, float, float]:
    lat = math.radians(ref_lat_deg)
    lon = math.radians(ref_lon_deg)
    sin_lat = math.sin(lat)
    cos_lat = math.cos(lat)
    sin_lon = math.sin(lon)
    cos_lon = math.cos(lon)
    east = -sin_lon * delta_x_m + cos_lon * delta_y_m
    north = -sin_lat * cos_lon * delta_x_m - sin_lat * sin_lon * delta_y_m + cos_lat * delta_z_m
    up = cos_lat * cos_lon * delta_x_m + cos_lat * sin_lon * delta_y_m + sin_lat * delta_z_m
    return (east, north, up)


def gps_time_from_calendar(date_token: str, time_token: str) -> tuple[int, float]:
    year, month, day = (int(part) for part in date_token.split("/"))
    hour_token, minute_token, second_token = time_token.split(":")
    second_float = float(second_token)
    second_int = int(math.floor(second_float))
    microsecond = int(round((second_float - second_int) * 1_000_000))
    stamp = dt.datetime(year, month, day, int(hour_token), int(minute_token), second_int, microsecond)
    total_seconds = (stamp - GPS_EPOCH).total_seconds()
    week = int(total_seconds // GPS_WEEK_SECONDS)
    tow = total_seconds - week * GPS_WEEK_SECONDS
    return week, tow


def parse_solution_line(line: str) -> SolutionEpoch | None:
    stripped = line.strip()
    if not stripped or stripped.startswith("%") or stripped.startswith("#"):
        return None
    parts = stripped.split()
    if len(parts) >= 11 and "/" not in parts[0]:
        try:
            week = int(float(parts[0]))
            tow = float(parts[1])
            x_m = float(parts[2])
            y_m = float(parts[3])
            z_m = float(parts[4])
            lat_deg = float(parts[5])
            lon_deg = float(parts[6])
            height_m = float(parts[7])
            status = int(float(parts[8]))
            satellites = int(float(parts[9]))
        except ValueError:
            return None
        if not is_finite_triplet((x_m, y_m, z_m)):
            return None
        return SolutionEpoch(week, tow, x_m, y_m, z_m, lat_deg, lon_deg, height_m, status, satellites)

    if len(parts) >= 7 and "/" in parts[0]:
        try:
            week, tow = gps_time_from_calendar(parts[0], parts[1])
            lat_deg = float(parts[2])
            lon_deg = float(parts[3])
            height_m = float(parts[4])
            status = int(float(parts[5]))
            satellites = int(float(parts[6]))
        except ValueError:
            return None
        x_m, y_m, z_m = llh_to_ecef(lat_deg, lon_deg, height_m)
        return SolutionEpoch(week, tow, x_m, y_m, z_m, lat_deg, lon_deg, height_m, status, satellites)
    return None


def read_solution(path: Path) -> list[SolutionEpoch]:
    rows: list[SolutionEpoch] = []
    with path.open(encoding="utf-8", errors="replace") as handle:
        for line in handle:
            epoch = parse_solution_line(line)
            if epoch is not None:
                rows.append(epoch)
    rows.sort(key=lambda epoch: (epoch.week, epoch.tow))
    return rows


def make_reference_frame(values: Sequence[float]) -> ReferenceFrame:
    if len(values) != 3:
        raise ValueError("reference ECEF requires exactly 3 values")
    x_m, y_m, z_m = values
    lat_deg, lon_deg, height_m = ecef_to_llh(x_m, y_m, z_m)
    return ReferenceFrame(x_m, y_m, z_m, lat_deg, lon_deg, height_m)


def epoch_key(epoch: SolutionEpoch, tolerance_s: float) -> tuple[int, int]:
    scale = 1.0 / tolerance_s if tolerance_s > 0.0 else 1000.0
    return (epoch.week, int(round(epoch.tow * scale)))


def match_solutions(
    base: Sequence[SolutionEpoch],
    candidate: Sequence[SolutionEpoch],
    reference: ReferenceFrame,
    tolerance_s: float,
) -> tuple[list[MatchedPair], list[SolutionEpoch], list[SolutionEpoch]]:
    candidate_used: set[int] = set()
    matches: list[MatchedPair] = []
    base_only: list[SolutionEpoch] = []

    by_week: dict[int, list[tuple[int, SolutionEpoch]]] = {}
    for index, epoch in enumerate(candidate):
        by_week.setdefault(epoch.week, []).append((index, epoch))

    for epoch in base:
        candidates = by_week.get(epoch.week, [])
        best_index: int | None = None
        best_epoch: SolutionEpoch | None = None
        best_gap = math.inf
        for index, candidate_epoch in candidates:
            if index in candidate_used:
                continue
            gap = abs(candidate_epoch.tow - epoch.tow)
            if gap <= tolerance_s and gap < best_gap:
                best_index = index
                best_epoch = candidate_epoch
                best_gap = gap
        if best_epoch is None or best_index is None:
            base_only.append(epoch)
            continue
        candidate_used.add(best_index)
        base_error = error_enu(epoch, reference)
        candidate_error = error_enu(best_epoch, reference)
        delta = (
            candidate_error[0] - base_error[0],
            candidate_error[1] - base_error[1],
            candidate_error[2] - base_error[2],
        )
        matches.append(MatchedPair(epoch, best_epoch, best_epoch.tow - epoch.tow, base_error, candidate_error, delta))

    candidate_only = [epoch for index, epoch in enumerate(candidate) if index not in candidate_used]
    matches.sort(key=lambda pair: (pair.base.week, pair.base.tow))
    return matches, base_only, candidate_only


def error_enu(epoch: SolutionEpoch, reference: ReferenceFrame) -> tuple[float, float, float]:
    return ecef_delta_to_enu(
        epoch.x_m - reference.x_m,
        epoch.y_m - reference.y_m,
        epoch.z_m - reference.z_m,
        reference.lat_deg,
        reference.lon_deg,
    )


def norm_2d(east_m: float, north_m: float) -> float:
    return math.hypot(east_m, north_m)


def norm_3d(east_m: float, north_m: float, up_m: float) -> float:
    return math.sqrt(east_m * east_m + north_m * north_m + up_m * up_m)


def rms(values: Sequence[float]) -> float:
    return math.sqrt(sum(value * value for value in values) / len(values)) if values else 0.0


def mean(values: Sequence[float]) -> float:
    return sum(values) / len(values) if values else 0.0


def percentile(values: Sequence[float], pct: float) -> float:
    if not values:
        return 0.0
    ordered = sorted(values)
    if len(ordered) == 1:
        return ordered[0]
    rank = (len(ordered) - 1) * pct / 100.0
    lower = int(math.floor(rank))
    upper = int(math.ceil(rank))
    if lower == upper:
        return ordered[lower]
    fraction = rank - lower
    return ordered[lower] * (1.0 - fraction) + ordered[upper] * fraction


def status_counts(epochs: Sequence[SolutionEpoch]) -> dict[str, int]:
    counts = Counter(epoch.status for epoch in epochs)
    return {str(status): counts[status] for status in sorted(counts)}


def normalize_thresholds(thresholds: Sequence[float] | None) -> list[float]:
    if thresholds is None:
        thresholds = DEFAULT_EVENT_THRESHOLDS_M
    cleaned = sorted({float(value) for value in thresholds if math.isfinite(value) and value > 0.0})
    return cleaned


def pair_event_summary(pair: MatchedPair) -> dict[str, Any]:
    base_h = norm_2d(pair.base_error_enu_m[0], pair.base_error_enu_m[1])
    candidate_h = norm_2d(pair.candidate_error_enu_m[0], pair.candidate_error_enu_m[1])
    delta_h = norm_2d(pair.delta_enu_m[0], pair.delta_enu_m[1])
    return {
        "week": pair.base.week,
        "tow": pair.base.tow,
        "candidate_tow": pair.candidate.tow,
        "dt_s": pair.dt_s,
        "base_status": pair.base.status,
        "candidate_status": pair.candidate.status,
        "base_satellites": pair.base.satellites,
        "candidate_satellites": pair.candidate.satellites,
        "base_east_m": pair.base_error_enu_m[0],
        "base_north_m": pair.base_error_enu_m[1],
        "base_up_m": pair.base_error_enu_m[2],
        "base_horizontal_m": base_h,
        "base_3d_m": norm_3d(*pair.base_error_enu_m),
        "candidate_east_m": pair.candidate_error_enu_m[0],
        "candidate_north_m": pair.candidate_error_enu_m[1],
        "candidate_up_m": pair.candidate_error_enu_m[2],
        "candidate_horizontal_m": candidate_h,
        "candidate_3d_m": norm_3d(*pair.candidate_error_enu_m),
        "delta_east_m": pair.delta_enu_m[0],
        "delta_north_m": pair.delta_enu_m[1],
        "delta_up_m": pair.delta_enu_m[2],
        "delta_horizontal_m": delta_h,
        "delta_3d_m": norm_3d(*pair.delta_enu_m),
    }


def summarize_events(matches: Sequence[MatchedPair], thresholds_m: Sequence[float] | None) -> dict[str, Any]:
    thresholds = normalize_thresholds(thresholds_m)
    if not matches:
        return {
            "max_delta_3d": None,
            "max_delta_horizontal": None,
            "first_delta_3d_over_thresholds": [
                {"threshold_m": threshold, "event": None} for threshold in thresholds
            ],
        }

    max_3d = max(matches, key=lambda pair: norm_3d(*pair.delta_enu_m))
    max_horizontal = max(
        matches,
        key=lambda pair: norm_2d(pair.delta_enu_m[0], pair.delta_enu_m[1]),
    )
    crossings: list[dict[str, Any]] = []
    for threshold in thresholds:
        event = None
        for pair in matches:
            if norm_3d(*pair.delta_enu_m) >= threshold:
                event = pair_event_summary(pair)
                break
        crossings.append({"threshold_m": threshold, "event": event})

    return {
        "max_delta_3d": pair_event_summary(max_3d),
        "max_delta_horizontal": pair_event_summary(max_horizontal),
        "first_delta_3d_over_thresholds": crossings,
    }


def summarize_solution(epochs: Sequence[SolutionEpoch], reference: ReferenceFrame) -> dict[str, Any]:
    errors = [error_enu(epoch, reference) for epoch in epochs]
    horizontal = [norm_2d(east, north) for east, north, _ in errors]
    up_abs = [abs(up) for _, _, up in errors]
    three_d = [norm_3d(east, north, up) for east, north, up in errors]
    return {
        "rows": len(epochs),
        "first_week": epochs[0].week if epochs else None,
        "first_tow": epochs[0].tow if epochs else None,
        "last_week": epochs[-1].week if epochs else None,
        "last_tow": epochs[-1].tow if epochs else None,
        "status_counts": status_counts(epochs),
        "satellites_mean": mean([epoch.satellites for epoch in epochs]),
        "reference_rms_horizontal_m": rms(horizontal),
        "reference_rms_up_m": rms([up for _, _, up in errors]),
        "reference_rms_3d_m": rms(three_d),
        "reference_p95_horizontal_m": percentile(horizontal, 95.0),
        "reference_p95_abs_up_m": percentile(up_abs, 95.0),
        "reference_last_east_m": errors[-1][0] if errors else None,
        "reference_last_north_m": errors[-1][1] if errors else None,
        "reference_last_up_m": errors[-1][2] if errors else None,
        "reference_last_3d_m": three_d[-1] if three_d else None,
    }


def subset_tail(matches: Sequence[MatchedPair], tail_seconds: float) -> list[MatchedPair]:
    if not matches or tail_seconds <= 0.0:
        return list(matches)
    end_tow = matches[-1].base.tow
    start_tow = end_tow - tail_seconds
    return [pair for pair in matches if pair.base.tow > start_tow]


def summarize_matches(matches: Sequence[MatchedPair]) -> dict[str, Any]:
    delta_e = [pair.delta_enu_m[0] for pair in matches]
    delta_n = [pair.delta_enu_m[1] for pair in matches]
    delta_u = [pair.delta_enu_m[2] for pair in matches]
    delta_h = [norm_2d(pair.delta_enu_m[0], pair.delta_enu_m[1]) for pair in matches]
    delta_3d = [norm_3d(*pair.delta_enu_m) for pair in matches]
    base_h = [norm_2d(pair.base_error_enu_m[0], pair.base_error_enu_m[1]) for pair in matches]
    candidate_h = [norm_2d(pair.candidate_error_enu_m[0], pair.candidate_error_enu_m[1]) for pair in matches]
    base_3d = [norm_3d(*pair.base_error_enu_m) for pair in matches]
    candidate_3d = [norm_3d(*pair.candidate_error_enu_m) for pair in matches]
    return {
        "matched_epochs": len(matches),
        "first_week": matches[0].base.week if matches else None,
        "first_tow": matches[0].base.tow if matches else None,
        "last_week": matches[-1].base.week if matches else None,
        "last_tow": matches[-1].base.tow if matches else None,
        "base_reference_rms_horizontal_m": rms(base_h),
        "candidate_reference_rms_horizontal_m": rms(candidate_h),
        "base_reference_rms_3d_m": rms(base_3d),
        "candidate_reference_rms_3d_m": rms(candidate_3d),
        "delta_rms_east_m": rms(delta_e),
        "delta_rms_north_m": rms(delta_n),
        "delta_rms_up_m": rms(delta_u),
        "delta_rms_horizontal_m": rms(delta_h),
        "delta_rms_3d_m": rms(delta_3d),
        "delta_max_horizontal_m": max(delta_h) if delta_h else 0.0,
        "delta_max_3d_m": max(delta_3d) if delta_3d else 0.0,
        "delta_last_east_m": delta_e[-1] if delta_e else None,
        "delta_last_north_m": delta_n[-1] if delta_n else None,
        "delta_last_up_m": delta_u[-1] if delta_u else None,
        "delta_last_horizontal_m": delta_h[-1] if delta_h else None,
        "delta_last_3d_m": delta_3d[-1] if delta_3d else None,
        "mean_dt_s": mean([pair.dt_s for pair in matches]),
        "max_abs_dt_s": max([abs(pair.dt_s) for pair in matches], default=0.0),
    }


def build_report(
    base_path: Path,
    candidate_path: Path,
    reference: ReferenceFrame,
    tolerance_s: float,
    tail_seconds: float,
    base_label: str,
    candidate_label: str,
    event_thresholds_m: Sequence[float] | None = None,
) -> tuple[dict[str, Any], list[MatchedPair]]:
    base = read_solution(base_path)
    candidate = read_solution(candidate_path)
    if not base:
        raise ValueError(f"{base_path}: no solution rows parsed")
    if not candidate:
        raise ValueError(f"{candidate_path}: no solution rows parsed")
    matches, base_only, candidate_only = match_solutions(base, candidate, reference, tolerance_s)
    if not matches:
        raise ValueError("no common solution epochs matched")
    tail_matches = subset_tail(matches, tail_seconds)
    event_thresholds = normalize_thresholds(event_thresholds_m)
    report = {
        "base_label": base_label,
        "candidate_label": candidate_label,
        "base_path": str(base_path),
        "candidate_path": str(candidate_path),
        "match_tolerance_s": tolerance_s,
        "reference": {
            "ecef_m": [reference.x_m, reference.y_m, reference.z_m],
            "lat_deg": reference.lat_deg,
            "lon_deg": reference.lon_deg,
            "height_m": reference.height_m,
        },
        "base": summarize_solution(base, reference),
        "candidate": summarize_solution(candidate, reference),
        "comparison": summarize_matches(matches),
        "tail_comparison": summarize_matches(tail_matches),
        "events": summarize_events(matches, event_thresholds),
        "tail_events": summarize_events(tail_matches, event_thresholds),
        "event_thresholds_3d_m": event_thresholds,
        "base_only_epochs": len(base_only),
        "candidate_only_epochs": len(candidate_only),
        "base_only_sample": [format_epoch_key(epoch) for epoch in base_only[:10]],
        "candidate_only_sample": [format_epoch_key(epoch) for epoch in candidate_only[:10]],
        "tail_seconds": tail_seconds,
    }
    return report, matches


def format_epoch_key(epoch: SolutionEpoch) -> str:
    return f"{epoch.week}:{epoch.tow:.3f}"


def write_matches_csv(path: Path, matches: Sequence[MatchedPair]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = [
        "week",
        "tow",
        "candidate_tow",
        "dt_s",
        "base_status",
        "candidate_status",
        "base_satellites",
        "candidate_satellites",
        "base_east_m",
        "base_north_m",
        "base_up_m",
        "base_horizontal_m",
        "base_3d_m",
        "candidate_east_m",
        "candidate_north_m",
        "candidate_up_m",
        "candidate_horizontal_m",
        "candidate_3d_m",
        "delta_east_m",
        "delta_north_m",
        "delta_up_m",
        "delta_horizontal_m",
        "delta_3d_m",
    ]
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for pair in matches:
            base_h = norm_2d(pair.base_error_enu_m[0], pair.base_error_enu_m[1])
            candidate_h = norm_2d(pair.candidate_error_enu_m[0], pair.candidate_error_enu_m[1])
            delta_h = norm_2d(pair.delta_enu_m[0], pair.delta_enu_m[1])
            writer.writerow(
                {
                    "week": pair.base.week,
                    "tow": f"{pair.base.tow:.3f}",
                    "candidate_tow": f"{pair.candidate.tow:.3f}",
                    "dt_s": f"{pair.dt_s:.6f}",
                    "base_status": pair.base.status,
                    "candidate_status": pair.candidate.status,
                    "base_satellites": pair.base.satellites,
                    "candidate_satellites": pair.candidate.satellites,
                    "base_east_m": f"{pair.base_error_enu_m[0]:.6f}",
                    "base_north_m": f"{pair.base_error_enu_m[1]:.6f}",
                    "base_up_m": f"{pair.base_error_enu_m[2]:.6f}",
                    "base_horizontal_m": f"{base_h:.6f}",
                    "base_3d_m": f"{norm_3d(*pair.base_error_enu_m):.6f}",
                    "candidate_east_m": f"{pair.candidate_error_enu_m[0]:.6f}",
                    "candidate_north_m": f"{pair.candidate_error_enu_m[1]:.6f}",
                    "candidate_up_m": f"{pair.candidate_error_enu_m[2]:.6f}",
                    "candidate_horizontal_m": f"{candidate_h:.6f}",
                    "candidate_3d_m": f"{norm_3d(*pair.candidate_error_enu_m):.6f}",
                    "delta_east_m": f"{pair.delta_enu_m[0]:.6f}",
                    "delta_north_m": f"{pair.delta_enu_m[1]:.6f}",
                    "delta_up_m": f"{pair.delta_enu_m[2]:.6f}",
                    "delta_horizontal_m": f"{delta_h:.6f}",
                    "delta_3d_m": f"{norm_3d(*pair.delta_enu_m):.6f}",
                }
            )


def print_event(label: str, event: dict[str, Any] | None) -> None:
    if event is None:
        print(f"  {label}: none")
        return
    print(
        f"  {label}: week={event['week']} tow={event['tow']:.3f} "
        f"delta_3d_m={event['delta_3d_m']:.6g} "
        f"delta_horizontal_m={event['delta_horizontal_m']:.6g} "
        f"delta_up_m={event['delta_up_m']:.6g}"
    )


def print_report(report: dict[str, Any]) -> None:
    comparison = report["comparison"]
    tail = report["tail_comparison"]
    events = report["events"]
    tail_events = report["tail_events"]
    print(f"base ({report['base_label']}):")
    print(f"  rows: {report['base']['rows']}")
    print(f"  reference_rms_3d_m: {report['base']['reference_rms_3d_m']:.6g}")
    print(f"candidate ({report['candidate_label']}):")
    print(f"  rows: {report['candidate']['rows']}")
    print(f"  reference_rms_3d_m: {report['candidate']['reference_rms_3d_m']:.6g}")
    print("comparison:")
    print(f"  matched_epochs: {comparison['matched_epochs']}")
    print(f"  base_only_epochs: {report['base_only_epochs']}")
    print(f"  candidate_only_epochs: {report['candidate_only_epochs']}")
    print(f"  delta_rms_horizontal_m: {comparison['delta_rms_horizontal_m']:.6g}")
    print(f"  delta_rms_up_m: {comparison['delta_rms_up_m']:.6g}")
    print(f"  delta_rms_3d_m: {comparison['delta_rms_3d_m']:.6g}")
    print(f"  delta_last_3d_m: {comparison['delta_last_3d_m']:.6g}")
    print("events:")
    print_event("max_delta_3d", events["max_delta_3d"])
    print_event("max_delta_horizontal", events["max_delta_horizontal"])
    for crossing in events["first_delta_3d_over_thresholds"]:
        threshold = crossing["threshold_m"]
        print_event(f"first_delta_3d_ge_{threshold:g}m", crossing["event"])
    print(f"tail_{int(report['tail_seconds'])}s:")
    print(f"  matched_epochs: {tail['matched_epochs']}")
    print(f"  delta_rms_horizontal_m: {tail['delta_rms_horizontal_m']:.6g}")
    print(f"  delta_rms_up_m: {tail['delta_rms_up_m']:.6g}")
    print(f"  delta_rms_3d_m: {tail['delta_rms_3d_m']:.6g}")
    print_event("tail_max_delta_3d", tail_events["max_delta_3d"])


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Compare MADOCALIB/RTKLIB and native LibGNSS++ .pos solution files."
    )
    parser.add_argument("base_pos", type=Path, help="Base/reference solution .pos file, e.g. MADOCALIB bridge")
    parser.add_argument("candidate_pos", type=Path, help="Candidate solution .pos file, e.g. native LibGNSS++")
    parser.add_argument(
        "--reference-ecef",
        type=float,
        nargs=3,
        metavar=("X", "Y", "Z"),
        required=True,
        help="Reference ECEF coordinate in meters for ENU error summaries.",
    )
    parser.add_argument("--base-label", default="base", help="Label used in printed and JSON output")
    parser.add_argument("--candidate-label", default="candidate", help="Label used in printed and JSON output")
    parser.add_argument("--match-tolerance", type=float, default=0.11, help="Epoch match tolerance in seconds")
    parser.add_argument("--tail-seconds", type=float, default=1800.0, help="Trailing window length for extra summary")
    parser.add_argument(
        "--threshold-3d",
        type=float,
        action="append",
        dest="thresholds_3d",
        help=(
            "3D delta threshold in meters for first-crossing event output. "
            "Repeat to set multiple thresholds; defaults to 1, 10, 100, and 1000 m."
        ),
    )
    parser.add_argument("--json-out", type=Path, help="Optional JSON report path")
    parser.add_argument("--match-csv", type=Path, help="Optional per-matched-epoch CSV path")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    reference = make_reference_frame(args.reference_ecef)
    report, matches = build_report(
        args.base_pos,
        args.candidate_pos,
        reference,
        args.match_tolerance,
        args.tail_seconds,
        args.base_label,
        args.candidate_label,
        args.thresholds_3d,
    )
    print_report(report)
    if args.json_out:
        args.json_out.parent.mkdir(parents=True, exist_ok=True)
        args.json_out.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    if args.match_csv:
        write_matches_csv(args.match_csv, matches)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
