#!/usr/bin/env python3
"""Run PPC kinematic CLAS PPP-RTK baseline and emit a scorecard vs MRTKLIB targets."""

from __future__ import annotations

import argparse
import bisect
import csv
import datetime as dt
import json
import math
import os
import subprocess
import sys
import time
import urllib.parse
import urllib.request
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping, Sequence

import numpy as np


ROOT_DIR = Path(__file__).resolve().parents[1]
APPS_DIR = ROOT_DIR / "apps"
SCRIPTS_DIR = ROOT_DIR / "scripts"
if str(APPS_DIR) not in sys.path:
    sys.path.insert(0, str(APPS_DIR))
if str(SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_DIR))

import generate_driving_comparison as comparison  # noqa: E402
from gnss_clas_ppp import expand_qzss_l6_source  # noqa: E402


GPS_EPOCH = dt.datetime(1980, 1, 6)
GPS_UTC_LEAP_SECONDS = 18  # valid for PPC 2024 collection dates
QZSS_ARCHIVE_BASE = "https://sys.qzss.go.jp"
QZSS_ARCHIVE_AJAX = f"{QZSS_ARCHIVE_BASE}/ajax.do"
QZSS_L6_DOWNLOAD = f"{QZSS_ARCHIVE_BASE}/archives/l6"
PPP_FIXED_STATUS = 6
MATCH_TOLERANCE_S = 0.25
LEAD_IN_MINUTES = 30
LEVER_ARM_BODY_M = np.array([0.593, -0.670, -1.216])  # IMU -> antenna, x fwd / y right / z down
LEVER_ARM_BY_CITY: dict[str, np.ndarray] = {
    # PPC README: Tokyo AT1675 mount; Nagoya Zephyr 3 Rover mount.
    "tokyo": np.array([0.31, 0.0, -0.55]),
    "nagoya": LEVER_ARM_BODY_M,
}

MRTKLIB_TARGETS: dict[str, dict[str, float]] = {
    "nagoya_run1": {"fix_pct": 17.0, "rms2d_m": 1.105, "sigma2d_m": 0.402},
    "nagoya_run2": {"fix_pct": 23.4, "rms2d_m": 1.119, "sigma2d_m": 0.461},
    "nagoya_run3": {"fix_pct": 6.3, "rms2d_m": 0.318, "sigma2d_m": 0.339},
    "tokyo_run1": {"fix_pct": 4.9, "rms2d_m": 0.747, "sigma2d_m": 0.244},
    "tokyo_run2": {"fix_pct": 21.7, "rms2d_m": 0.514, "sigma2d_m": 0.120},
    "tokyo_run3": {"fix_pct": 7.4, "rms2d_m": 0.801, "sigma2d_m": 0.075},
}

PARITY_ENV = {
    "GNSS_PPP_CLAS_BASE_CLOCK_PARITY": "1",
    "GNSS_PPP_CLAS_SIS_BOUNDARY": "1",
    "GNSS_PPP_CLAS_TROP_GRID_PARITY": "1",
    "GNSS_PPP_CLAS_QZSS_S_PRN_FIX": "1",
}

CONFIG_LABELS = {
    "default": "default (no env gates)",
    "parity": "parity stack",
}


@dataclass(frozen=True)
class RunWindow:
    key: str
    city: str
    run: str
    gps_week: int
    gps_tow_start: float
    gps_tow_end: float
    doy: int
    year: int
    utc_start: dt.datetime
    utc_end: dt.datetime


@dataclass(frozen=True)
class RunPaths:
    dataset_root: Path
    rover_obs: Path
    base_nav: Path
    reference_csv: Path
    l6_concat: Path
    ssr_csv: Path
    pos_default: Path
    pos_parity: Path
    summary_default: Path
    summary_parity: Path


def rounded(value: float, digits: int = 3) -> float:
    return round(value, digits)


def gps_to_utc(stamp: dt.datetime) -> dt.datetime:
    return stamp - dt.timedelta(seconds=GPS_UTC_LEAP_SECONDS)


def utc_to_gps(stamp: dt.datetime) -> dt.datetime:
    return stamp + dt.timedelta(seconds=GPS_UTC_LEAP_SECONDS)


def gps_week_tow(stamp: dt.datetime) -> tuple[int, float]:
    total_seconds = (stamp - GPS_EPOCH).total_seconds()
    week = int(total_seconds // 604800)
    tow = total_seconds - week * 604800
    return week, tow


def day_of_year(stamp: dt.datetime) -> int:
    return int(stamp.strftime("%j"))


def parse_rinex_first_last_obs(path: Path) -> tuple[dt.datetime, dt.datetime, float]:
    first_obs: dt.datetime | None = None
    last_obs: dt.datetime | None = None
    interval = 1.0
    with path.open(encoding="ascii", errors="replace") as handle:
        for line in handle:
            if "INTERVAL" in line and line.strip().split()[0].replace(".", "", 1).isdigit():
                interval = float(line.split()[0])
            if "TIME OF FIRST OBS" in line:
                parts = line.split()
                first_obs = dt.datetime(
                    int(parts[0]),
                    int(parts[1]),
                    int(parts[2]),
                    int(parts[3]),
                    int(parts[4]),
                    int(float(parts[5])),
                )
            if "TIME OF LAST OBS" in line:
                parts = line.split()
                last_obs = dt.datetime(
                    int(parts[0]),
                    int(parts[1]),
                    int(parts[2]),
                    int(parts[3]),
                    int(parts[4]),
                    int(float(parts[5])),
                )
                break
    if first_obs is None or last_obs is None:
        raise SystemExit(f"Could not parse RINEX observation times from {path}")
    return first_obs, last_obs, interval


def discover_run_window(dataset_root: Path, city: str, run: str) -> RunWindow:
    rover_obs = dataset_root / city / run / "rover.obs"
    first_gps, last_gps, _interval = parse_rinex_first_last_obs(rover_obs)
    week_start, tow_start = gps_week_tow(first_gps)
    week_end, tow_end = gps_week_tow(last_gps)
    if week_start != week_end:
        raise SystemExit(f"{city}/{run} spans multiple GPS weeks; not supported")
    utc_start = gps_to_utc(first_gps)
    utc_end = gps_to_utc(last_gps)
    return RunWindow(
        key=f"{city}_{run}",
        city=city,
        run=run,
        gps_week=week_start,
        gps_tow_start=tow_start,
        gps_tow_end=tow_end,
        doy=day_of_year(first_gps),
        year=first_gps.year,
        utc_start=utc_start,
        utc_end=utc_end,
    )


def slot_letter_for_utc_hour(hour: int) -> str:
    if not 0 <= hour <= 23:
        raise ValueError(f"invalid UTC hour: {hour}")
    return chr(ord("A") + hour)


def utc_hour_slots(start_utc: dt.datetime, end_utc: dt.datetime) -> list[str]:
    slots: list[str] = []
    cursor = start_utc.replace(minute=0, second=0, microsecond=0)
    end_hour = end_utc.replace(minute=0, second=0, microsecond=0)
    while cursor <= end_hour:
        slots.append(slot_letter_for_utc_hour(cursor.hour))
        cursor += dt.timedelta(hours=1)
    return slots


def l6_slots_for_window(window: RunWindow, lead_in_minutes: int) -> list[str]:
    start_utc = window.utc_start - dt.timedelta(minutes=lead_in_minutes)
    end_utc = window.utc_end + dt.timedelta(minutes=5)
    return utc_hour_slots(start_utc, end_utc)


def l6_file_name(year: int, doy: int, slot: str) -> str:
    return f"{year}{doy:03d}{slot}.l6"


def http_get(url: str, timeout_s: float = 120.0) -> bytes:
    request = urllib.request.Request(url, headers={"User-Agent": "Mozilla/5.0 gnssplusplus-ppc-clas"})
    with urllib.request.urlopen(request, timeout=timeout_s) as response:
        return response.read()


def archive_list_l6_files(date_utc: dt.datetime) -> list[dict[str, str]]:
    payload = urllib.parse.urlencode(
        {
            "CURRENT_PAGE": "1",
            "MAX_PAGE": "5",
            "DATA_TYPE": "",
            "START_YEAR": str(date_utc.year),
            "START_MONTH": str(date_utc.month),
            "START_DAY": str(date_utc.day),
            "END_YEAR": str(date_utc.year),
            "END_MONTH": str(date_utc.month),
            "END_DAY": str(date_utc.day),
            "ARC_TARGET": "l6",
            "coreAjaxField": "DirectConductor",
            "coreAjaxFixNo": "DOD_USR_CMN001@1",
        }
    ).encode("ascii")
    request = urllib.request.Request(
        QZSS_ARCHIVE_AJAX,
        data=payload,
        headers={"User-Agent": "Mozilla/5.0 gnssplusplus-ppc-clas"},
        method="POST",
    )
    with urllib.request.urlopen(request, timeout=60.0) as response:
        data = json.loads(response.read().decode("utf-8"))
    return list(data.get("JSON_LIST", []))


def fetch_l6_slots(
    window: RunWindow,
    cache_dir: Path,
    *,
    lead_in_minutes: int,
    force: bool,
) -> tuple[Path, list[str], str]:
    cache_dir.mkdir(parents=True, exist_ok=True)
    slots = l6_slots_for_window(window, lead_in_minutes)
    downloaded: list[str] = []
    for slot in slots:
        name = l6_file_name(window.year, window.doy, slot)
        target = cache_dir / name
        if target.exists() and not force and target.stat().st_size >= 100_000:
            downloaded.append(name)
            continue
        url = f"{QZSS_L6_DOWNLOAD}/{window.year}/{name}"
        body = http_get(url)
        if len(body) < 100_000 or body.startswith(b"<!DOCTYPE"):
            raise SystemExit(
                f"Failed to download CLAS L6 archive file {name} from {url}; "
                "verify network access to sys.qzss.go.jp and the archive date."
            )
        target.write_bytes(body)
        downloaded.append(name)
    concat_path = cache_dir / f"{window.key}.l6"
    with concat_path.open("wb") as handle:
        for slot in slots:
            handle.write((cache_dir / l6_file_name(window.year, window.doy, slot)).read_bytes())
    source = (
        f"QZSS public archive {QZSS_ARCHIVE_BASE}/dod/archives/clas.html "
        f"(L6 hourly files under {QZSS_L6_DOWNLOAD}/{{year}}/{{YYYYDOY}}{{slot}}.l6)"
    )
    return concat_path, downloaded, source


def expand_ssr_csv(
    l6_path: Path,
    window: RunWindow,
    output_csv: Path,
    *,
    lead_in_minutes: int,
    force: bool,
) -> dict[str, Any]:
    if output_csv.exists() and not force:
        return {"expanded_csv": str(output_csv), "cached": True}
    raw_csv = output_csv.with_suffix(".full.csv")
    summary = expand_qzss_l6_source(str(l6_path), window.gps_week, raw_csv)
    tow_min = window.gps_tow_start - lead_in_minutes * 60.0 - 60.0
    tow_max = window.gps_tow_end + 120.0
    rows_kept = 0
    output_csv.parent.mkdir(parents=True, exist_ok=True)
    with raw_csv.open(encoding="ascii") as source, output_csv.open("w", encoding="ascii") as handle:
        for line in source:
            if not line.strip() or line.startswith("#"):
                handle.write(line)
                continue
            tow = float(line.split(",", 2)[1])
            if tow < tow_min or tow > tow_max:
                continue
            handle.write(line)
            rows_kept += 1
    if raw_csv.exists():
        raw_csv.unlink()
    summary["expanded_csv"] = str(output_csv)
    summary["rows_written"] = rows_kept
    summary["tow_window"] = [rounded(tow_min), rounded(tow_max)]
    summary["cached"] = False
    return summary


def ssr_csv_tow_range(path: Path) -> tuple[float, float] | None:
    tow_min: float | None = None
    tow_max: float | None = None
    with path.open(encoding="ascii", errors="replace") as handle:
        for line in handle:
            if not line.strip() or line.startswith("#"):
                continue
            tow = float(line.split(",", 2)[1])
            tow_min = tow if tow_min is None else min(tow_min, tow)
            tow_max = tow if tow_max is None else max(tow_max, tow)
    if tow_min is None or tow_max is None:
        return None
    return tow_min, tow_max


def ssr_csv_covers_window(
    path: Path,
    window: RunWindow,
    *,
    lead_in_minutes: int,
) -> bool:
    tow_range = ssr_csv_tow_range(path)
    if tow_range is None:
        return False
    tow_min, tow_max = tow_range
    required_min = window.gps_tow_start - lead_in_minutes * 60.0 - 60.0
    required_max = window.gps_tow_end + 60.0
    return tow_min <= required_min + 30.0 and tow_max >= required_max - 30.0


def build_gnss_ppp_command(
    *,
    gnss_ppp_bin: Path,
    rover_obs: Path,
    base_nav: Path,
    ssr_csv: Path,
    out_pos: Path,
) -> list[str]:
    return [
        str(gnss_ppp_bin),
        "--obs",
        str(rover_obs),
        "--nav",
        str(base_nav),
        "--ssr",
        str(ssr_csv),
        "--kinematic",
        "--estimate-troposphere",
        "--no-ionosphere-free",
        "--estimate-ionosphere",
        "--enable-ar",
        "--ar-ratio-threshold",
        "2.0",
        "--ar-method",
        "wlnl",
        "--clas-osr",
        "--emit-epoch-time",
        "--out",
        str(out_pos),
    ]


def run_logged(
    command: Sequence[str],
    *,
    env: Mapping[str, str] | None,
    log_path: Path,
) -> subprocess.CompletedProcess[str]:
    log_path.parent.mkdir(parents=True, exist_ok=True)
    started = time.monotonic()
    with log_path.open("a", encoding="utf-8") as handle:
        handle.write(f"$ {' '.join(command)}\n")
        if env:
            parity_keys = [key for key in PARITY_ENV if env.get(key) == "1"]
            if parity_keys:
                handle.write(f"# env parity keys: {', '.join(parity_keys)}\n")
    completed = subprocess.run(
        list(command),
        cwd=ROOT_DIR,
        env=dict(env) if env is not None else None,
        text=True,
        capture_output=True,
        check=False,
    )
    elapsed = time.monotonic() - started
    combined = completed.stdout
    if completed.stderr:
        if combined and not combined.endswith("\n"):
            combined += "\n"
        combined += completed.stderr
    with log_path.open("a", encoding="utf-8") as handle:
        handle.write(combined)
        if combined and not combined.endswith("\n"):
            handle.write("\n")
        handle.write(f"# exit={completed.returncode} elapsed_s={elapsed:.3f}\n\n")
    return completed


def enu_to_ecef_delta(enu: np.ndarray, ref_lat_deg: float, ref_lon_deg: float) -> np.ndarray:
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
    return rot.T @ enu


def body_to_enu_rotation_matrix(roll_deg: float, pitch_deg: float, heading_deg: float) -> np.ndarray:
    """Rotate vehicle-frame vectors (x fwd, y right, z down) into local ENU."""
    roll = math.radians(roll_deg)
    pitch = math.radians(pitch_deg)
    yaw = math.radians(heading_deg)
    psi = math.pi / 2.0 - yaw
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(psi), math.sin(psi)
    return np.array(
        [
            [cy * cp, sy * cp, -sp],
            [-sy * cr + cy * sp * sr, cy * cr + sy * sp * sr, cp * sr],
            [sy * sr + cy * sp * cr, -cy * sr + sy * sp * cr, cp * cr],
        ]
    )


def parse_rinex_observation_epochs(path: Path) -> list[tuple[int, float]]:
    epochs: list[tuple[int, float]] = []
    with path.open(encoding="ascii", errors="replace") as handle:
        for line in handle:
            if not line.startswith(">"):
                continue
            parts = line[1:].split()
            if len(parts) < 6:
                continue
            stamp = dt.datetime(
                int(parts[0]),
                int(parts[1]),
                int(parts[2]),
                int(parts[3]),
                int(parts[4]),
                int(float(parts[5])),
            )
            week, tow = gps_week_tow(utc_to_gps(stamp))
            epochs.append((week, tow))
    if not epochs:
        raise SystemExit(f"Could not parse RINEX epoch headers from {path}")
    return epochs


def lever_arm_for_city(city: str) -> np.ndarray:
    return LEVER_ARM_BY_CITY.get(city, LEVER_ARM_BODY_M)


def read_reference_csv(path: Path, *, apply_lever_arm: bool = True, city: str | None = None) -> list[comparison.ReferenceEpoch]:
    rows: list[comparison.ReferenceEpoch] = []
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
            if apply_lever_arm and len(row) >= 11:
                roll = float(row[8])
                pitch = float(row[9])
                heading = float(row[10])
                lever_body = lever_arm_for_city(city) if city is not None else LEVER_ARM_BODY_M
                lever_enu = body_to_enu_rotation_matrix(roll, pitch, heading) @ lever_body
                ecef = ecef + enu_to_ecef_delta(lever_enu, lat, lon)
            rows.append(comparison.ReferenceEpoch(week, tow, lat, lon, height, ecef))
    return rows


def read_ppp_pos(path: Path) -> list[comparison.SolutionEpoch]:
    return comparison.read_libgnss_pos(path)


def stamp_solution_times(
    solutions: list[comparison.SolutionEpoch],
    obs_epochs: list[tuple[int, float]],
) -> list[comparison.SolutionEpoch]:
    """Recover GPS week/TOW when .pos rows omitted timing metadata."""
    if not solutions or not obs_epochs:
        return solutions
    if not all(epoch.week == 0 and abs(epoch.tow) < 1e-6 for epoch in solutions[: min(5, len(solutions))]):
        return solutions
    if len(solutions) == len(obs_epochs):
        return [
            comparison.SolutionEpoch(
                week=week,
                tow=tow,
                lat_deg=epoch.lat_deg,
                lon_deg=epoch.lon_deg,
                height_m=epoch.height_m,
                ecef=epoch.ecef,
                status=epoch.status,
                num_satellites=epoch.num_satellites,
                ratio=epoch.ratio,
                baseline_m=epoch.baseline_m,
                rtk_iterations=epoch.rtk_iterations,
                rtk_update_observations=epoch.rtk_update_observations,
                rtk_update_phase_observations=epoch.rtk_update_phase_observations,
                rtk_update_code_observations=epoch.rtk_update_code_observations,
                rtk_update_suppressed_outliers=epoch.rtk_update_suppressed_outliers,
                rtk_update_prefit_residual_rms_m=epoch.rtk_update_prefit_residual_rms_m,
                rtk_update_prefit_residual_max_m=epoch.rtk_update_prefit_residual_max_m,
                rtk_update_post_suppression_residual_rms_m=epoch.rtk_update_post_suppression_residual_rms_m,
                rtk_update_post_suppression_residual_max_m=epoch.rtk_update_post_suppression_residual_max_m,
                rtk_update_normalized_innovation_squared=epoch.rtk_update_normalized_innovation_squared,
                rtk_update_normalized_innovation_squared_per_observation=epoch.rtk_update_normalized_innovation_squared_per_observation,
                rtk_update_rejected_by_innovation_gate=epoch.rtk_update_rejected_by_innovation_gate,
            )
            for epoch, (week, tow) in zip(solutions, obs_epochs, strict=True)
        ]
    raise SystemExit(
        f"PPP produced {len(solutions)} epochs but rover.obs has {len(obs_epochs)}; "
        "re-run gnss_ppp so .pos rows carry GPS week/TOW for skipped-epoch alignment"
    )


def trajectory_sanity(
    solutions: list[comparison.SolutionEpoch],
    reference: list[comparison.ReferenceEpoch],
    matched: list[comparison.MatchedEpoch] | None = None,
) -> dict[str, float]:
    if not solutions or not reference:
        return {}
    if matched is None:
        matched = comparison.match_to_reference(solutions, reference, MATCH_TOLERANCE_S)
    if not matched:
        return {}
    ref_by_tow = {epoch.tow: epoch for epoch in reference}
    ref_aligned: list[comparison.ReferenceEpoch] = []
    sol_aligned: list[comparison.SolutionEpoch] = []
    sol_by_tow = {epoch.tow: epoch for epoch in solutions}
    for item in matched:
        ref = min(
            reference,
            key=lambda epoch: abs(epoch.tow - item.tow),
        )
        sol = sol_by_tow.get(item.tow)
        if sol is None:
            closest = min(solutions, key=lambda epoch: abs(epoch.tow - item.tow))
            if abs(closest.tow - item.tow) > MATCH_TOLERANCE_S:
                continue
            sol = closest
        ref_aligned.append(ref)
        sol_aligned.append(sol)
    if not ref_aligned:
        return {}
    origin = ref_aligned[0]
    truth_traj = comparison.trajectory_enu(ref_aligned, origin)
    sol_traj = comparison.trajectory_enu(sol_aligned, origin)
    truth_path_m = float(np.sum(np.linalg.norm(np.diff(truth_traj[:, :2], axis=0), axis=1)))
    sol_path_m = float(np.sum(np.linalg.norm(np.diff(sol_traj[:, :2], axis=0), axis=1)))
    truth_closure_m = float(np.linalg.norm(truth_traj[-1, :2] - truth_traj[0, :2]))
    sol_closure_m = float(np.linalg.norm(sol_traj[-1, :2] - sol_traj[0, :2]))
    max_lag_m = 0.0
    truth_tows = [epoch.tow for epoch in reference]
    for epoch in solutions:
        idx = bisect.bisect_left(truth_tows, epoch.tow)
        candidates = [reference[j] for j in (idx - 1, idx, idx + 1) if 0 <= j < len(reference)]
        if not candidates:
            continue
        ref = min(candidates, key=lambda item: abs(item.tow - epoch.tow))
        lag = float(np.linalg.norm(comparison.ecef_to_enu(epoch.ecef - ref.ecef, ref.lat_deg, ref.lon_deg)[:2]))
        max_lag_m = max(max_lag_m, lag)
    return {
        "truth_path_m": rounded(truth_path_m),
        "solution_path_m": rounded(sol_path_m),
        "truth_end_to_start_m": rounded(truth_closure_m),
        "solution_end_to_start_m": rounded(sol_closure_m),
        "max_lag_vs_truth_m": rounded(max_lag_m),
    }


def compute_ttff_s(matched: list[comparison.MatchedEpoch], fixed_status: int) -> float | None:
    for epoch in sorted(matched, key=lambda item: item.tow):
        if epoch.status == fixed_status:
            first_tow = epoch.tow
            return rounded(first_tow - matched[0].tow)
    return None


def score_run(
    pos_path: Path,
    reference_csv: Path,
    rover_obs: Path,
    *,
    fixed_status: int = PPP_FIXED_STATUS,
    match_tolerance_s: float = MATCH_TOLERANCE_S,
) -> dict[str, Any]:
    reference = read_reference_csv(reference_csv, city=reference_csv.parent.parent.name)
    solutions = read_ppp_pos(pos_path)
    if solutions and all(
        epoch.week == 0 and abs(epoch.tow) < 1e-6 for epoch in solutions[: min(5, len(solutions))]
    ):
        solutions = stamp_solution_times(solutions, parse_rinex_observation_epochs(rover_obs))
    matched = comparison.match_to_reference(solutions, reference, match_tolerance_s)
    if not matched:
        raise SystemExit(f"No epochs matched reference for {pos_path}")

    all_horiz = np.array([epoch.horiz_error_m for epoch in matched], dtype=float)
    fixed = [epoch for epoch in matched if epoch.status == fixed_status]
    fixed_horiz = np.array([epoch.horiz_error_m for epoch in fixed], dtype=float)

    sat_counts = [solution.num_satellites for solution in solutions]
    status_counts: dict[int, int] = {}
    for solution in solutions:
        status_counts[solution.status] = status_counts.get(solution.status, 0) + 1

    return {
        "matched_epochs": len(matched),
        "reference_epochs": len(reference),
        "solution_epochs": len(solutions),
        "fix_pct": rounded(100.0 * len(fixed) / len(matched)),
        "fixed_epochs": len(fixed),
        "rms2d_fixed_m": rounded(float(np.sqrt(np.mean(fixed_horiz**2))) if len(fixed_horiz) else float("nan")),
        "sigma2d_fixed_m": rounded(float(np.std(fixed_horiz)) if len(fixed_horiz) > 1 else float("nan")),
        "rms2d_all_m": rounded(float(np.sqrt(np.mean(all_horiz**2)))),
        "median_h_all_m": rounded(float(np.median(all_horiz))),
        "p95_h_all_m": rounded(float(np.percentile(all_horiz, 95))),
        "ttff_s": compute_ttff_s(matched, fixed_status),
        "mean_satellites": rounded(float(np.mean(sat_counts))),
        "status_counts": status_counts,
        **trajectory_sanity(solutions, reference, matched),
    }


def run_paths(dataset_root: Path, work_dir: Path, window: RunWindow) -> RunPaths:
    run_dir = work_dir / window.key
    return RunPaths(
        dataset_root=dataset_root,
        rover_obs=dataset_root / window.city / window.run / "rover.obs",
        base_nav=dataset_root / window.city / window.run / "base.nav",
        reference_csv=dataset_root / window.city / window.run / "reference.csv",
        l6_concat=run_dir / "l6" / f"{window.key}.l6",
        ssr_csv=run_dir / "ssr" / f"{window.key}_expanded.csv",
        pos_default=run_dir / "pos" / f"{window.key}_default.pos",
        pos_parity=run_dir / "pos" / f"{window.key}_parity.pos",
        summary_default=run_dir / "pos" / f"{window.key}_default_summary.json",
        summary_parity=run_dir / "pos" / f"{window.key}_parity_summary.json",
    )


def render_markdown_table(headers: list[str], rows: list[list[str]]) -> str:
    lines = [
        "| " + " | ".join(headers) + " |",
        "| " + " | ".join("---" for _ in headers) + " |",
    ]
    for row in rows:
        lines.append("| " + " | ".join(row) + " |")
    return "\n".join(lines)


def gap_notes(run_key: str, config: str, metrics: dict[str, Any], target: dict[str, float]) -> list[str]:
    notes: list[str] = []
    fix_gap = metrics["fix_pct"] - target["fix_pct"]
    rms_gap = metrics["rms2d_fixed_m"] - target["rms2d_m"] if not math.isnan(metrics["rms2d_fixed_m"]) else float("inf")
    if metrics["fixed_epochs"] == 0:
        notes.append("no fixed epochs")
    if fix_gap < -5.0:
        notes.append(f"fix rate {fix_gap:+.1f} pp vs MRTKLIB")
    if not math.isnan(metrics["rms2d_fixed_m"]) and rms_gap > 0.2:
        notes.append(f"fixed RMS2D +{rms_gap:.2f} m vs target")
    if metrics.get("ttff_s") is None:
        notes.append("no TTFF (never fixed)")
    elif metrics["ttff_s"] > 300:
        notes.append(f"slow TTFF {metrics['ttff_s']:.0f} s")
    if metrics["mean_satellites"] < 8:
        notes.append(f"low mean sat count {metrics['mean_satellites']:.1f}")
    return notes


def rank_gaps(results: list[dict[str, Any]]) -> list[str]:
    ranked: list[tuple[float, str]] = []
    for item in results:
        key = item["run_key"]
        target = MRTKLIB_TARGETS[key]
        for config in ("default", "parity"):
            metrics = item["configs"][config]
            fix_gap = target["fix_pct"] - metrics["fix_pct"]
            rms_gap = metrics["rms2d_fixed_m"] - target["rms2d_m"] if not math.isnan(metrics["rms2d_fixed_m"]) else 5.0
            score = max(fix_gap, 0.0) * 2.0 + max(rms_gap, 0.0)
            if metrics["fixed_epochs"] == 0:
                score += 50.0
            ranked.append((score, f"{key}/{config}: close fix gap {fix_gap:+.1f} pp, RMS2D gap {rms_gap:+.2f} m"))
    ranked.sort(key=lambda pair: pair[0], reverse=True)
    return [text for _score, text in ranked[:12]]


def write_report(
    path: Path,
    *,
    l6_source: str,
    csv_recipe: str,
    command_template: str,
    run_results: list[dict[str, Any]],
    work_dir: Path,
    l6_cache: Path,
) -> None:
    lines: list[str] = []
    lines.append("# PPC kinematic CLAS PPP-RTK baseline")
    lines.append("")
    lines.append("## Harness")
    lines.append("")
    lines.append(f"- L6 source: {l6_source}")
    lines.append(f"- L6 cache: `{l6_cache}` (per-run concatenations under `{work_dir}`)")
    lines.append(f"- SSR CSV recipe: {csv_recipe}")
    lines.append("- Observation rate: 5 Hz (0.2 s); SSR expanded rows are time-stamped (1 s class sampling in compact expansion). MW/AR windows in PPP are time-based on receiver epochs, but any epoch-count heuristics in the CLAS path should be checked against 5 Hz density.")
    lines.append("- Ground truth: `reference.csv` Applanix POS LVX ECEF with city-specific lever arm (Tokyo 0.31/0/−0.55 m, Nagoya 0.593/−0.670/−1.216 m body → antenna); horizontal error in local ENU at each matched epoch.")
    lines.append("- PPP `.pos` rows carry GPS week/TOW from the receiver epoch; legacy rows with `GPS_Week=0` fall back to rover.obs `>` header alignment when counts match.")
    lines.append("- Match tolerance: 0.25 s; PPP fixed status = 6.")
    lines.append("")
    lines.append("### gnss_ppp kinematic CLAS invocation")
    lines.append("")
    lines.append("```bash")
    lines.append(command_template)
    lines.append("```")
    lines.append("")
    lines.append("Configs:")
    lines.append("- (a) default: no `GNSS_PPP_CLAS_*` gates")
    lines.append("- (b) parity: `GNSS_PPP_CLAS_BASE_CLOCK_PARITY=1 GNSS_PPP_CLAS_SIS_BOUNDARY=1 GNSS_PPP_CLAS_TROP_GRID_PARITY=1 GNSS_PPP_CLAS_QZSS_S_PRN_FIX=1` (never `GNSS_PPP_CLAS_ATMOS_LIFECYCLE`)")
    lines.append("")

    for config_key, config_label in CONFIG_LABELS.items():
        lines.append(f"## Scorecard — {config_label}")
        lines.append("")
        lines.append(
            render_markdown_table(
                ["case", "Fix%", "RMS2D (m)", "1sigma (m)", "TTFF (s)", "all RMS2D", "notes"],
                [
                    [
                        item["run_key"],
                        f"{item['configs'][config_key]['fix_pct']:.1f}",
                        f"{item['configs'][config_key]['rms2d_fixed_m']:.3f}"
                        if not math.isnan(item["configs"][config_key]["rms2d_fixed_m"])
                        else "n/a",
                        f"{item['configs'][config_key]['sigma2d_fixed_m']:.3f}"
                        if not math.isnan(item["configs"][config_key]["sigma2d_fixed_m"])
                        else "n/a",
                        f"{item['configs'][config_key]['ttff_s']:.1f}"
                        if item["configs"][config_key]["ttff_s"] is not None
                        else "n/a",
                        f"{item['configs'][config_key]['rms2d_all_m']:.3f}",
                        "; ".join(item["configs"][config_key]["notes"]) or "—",
                    ]
                    for item in run_results
                ],
            )
        )
        lines.append("")
        lines.append("### MRTKLIB v0.4.2 targets")
        lines.append("")
        lines.append(
            render_markdown_table(
                ["case", "Fix%", "RMS2D (m)", "1sigma (m)"],
                [
                    [
                        key,
                        f"{MRTKLIB_TARGETS[key]['fix_pct']:.1f}",
                        f"{MRTKLIB_TARGETS[key]['rms2d_m']:.3f}",
                        f"{MRTKLIB_TARGETS[key]['sigma2d_m']:.3f}",
                    ]
                    for key in sorted(MRTKLIB_TARGETS)
                ],
            )
        )
        lines.append("")

    lines.append("## Per-run notes")
    lines.append("")
    for item in run_results:
        lines.append(f"### {item['run_key']}")
        lines.append("")
        lines.append(
            f"- Window: GPS week {item['gps_week']}, TOW {item['gps_tow_start']:.1f}-{item['gps_tow_end']:.1f}; "
            f"UTC {item['utc_start']} – {item['utc_end']}; DOY {item['doy']} ({item['year']})"
        )
        lines.append(f"- L6 slots: {', '.join(item['l6_slots'])}")
        lines.append(f"- SSR rows: {item['ssr_rows']}")
        for config_key, config_label in CONFIG_LABELS.items():
            metrics = item["configs"][config_key]
            lines.append(
                f"- {config_label}: fix {metrics['fix_pct']:.1f}% ({metrics['fixed_epochs']}/{metrics['matched_epochs']}), "
                f"mean sats {metrics['mean_satellites']:.1f}, status counts {metrics['status_counts']}"
            )
        lines.append("")

    lines.append("## Top gaps to close")
    lines.append("")
    for index, gap in enumerate(rank_gaps(run_results), start=1):
        lines.append(f"{index}. {gap}")
    lines.append("")

    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(prog=os.environ.get("GNSS_CLI_NAME"))
    parser.add_argument(
        "--dataset-root",
        type=Path,
        default=ROOT_DIR / "data" / "PPC-Dataset",
        help="PPC dataset root containing tokyo/ and nagoya/ runs.",
    )
    parser.add_argument(
        "--work-dir",
        type=Path,
        default=Path("/tmp/ppc_clas_baseline"),
        help="Working directory for L6/SSR/pos artifacts.",
    )
    parser.add_argument(
        "--l6-cache",
        type=Path,
        default=Path("/tmp/ppc_clas_baseline/l6_cache"),
        help="Shared cache for downloaded hourly QZSS L6 files.",
    )
    parser.add_argument(
        "--gnss-ppp",
        type=Path,
        default=ROOT_DIR / "build" / "apps" / "gnss_ppp",
        help="Path to gnss_ppp binary.",
    )
    parser.add_argument(
        "--report",
        type=Path,
        default=Path("/tmp/gnss_ppc_clas_baseline_report.md"),
        help="Markdown report output path.",
    )
    parser.add_argument(
        "--runs",
        nargs="*",
        default=["tokyo_run1", "tokyo_run2", "tokyo_run3", "nagoya_run1", "nagoya_run2", "nagoya_run3"],
        help="Run keys to process (city_runN).",
    )
    parser.add_argument("--skip-ppp", action="store_true", help="Reuse existing .pos files.")
    parser.add_argument("--skip-l6", action="store_true", help="Reuse cached L6/SSR files.")
    parser.add_argument("--force-fetch", action="store_true", help="Re-download L6 even if cached.")
    parser.add_argument("--force-ssr", action="store_true", help="Re-expand SSR CSV.")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if not args.gnss_ppp.exists():
        raise SystemExit(f"gnss_ppp binary not found: {args.gnss_ppp}")

    args.work_dir.mkdir(parents=True, exist_ok=True)
    args.l6_cache.mkdir(parents=True, exist_ok=True)

    run_results: list[dict[str, Any]] = []
    l6_source = ""
    for run_key in args.runs:
        city, run = run_key.split("_", 1)
        window = discover_run_window(args.dataset_root, city, run)
        paths = run_paths(args.dataset_root, args.work_dir, window)
        paths.l6_concat.parent.mkdir(parents=True, exist_ok=True)
        paths.ssr_csv.parent.mkdir(parents=True, exist_ok=True)
        paths.pos_default.parent.mkdir(parents=True, exist_ok=True)

        print(f"=== {run_key}: fetching L6 / expanding SSR ===", flush=True)
        if not args.skip_l6:
            l6_path, _files, l6_source = fetch_l6_slots(
                window,
                args.l6_cache,
                lead_in_minutes=LEAD_IN_MINUTES,
                force=args.force_fetch,
            )
            if l6_path != paths.l6_concat:
                paths.l6_concat.write_bytes(l6_path.read_bytes())
        else:
            l6_path = paths.l6_concat
            if not l6_path.exists():
                raise SystemExit(f"Missing cached L6 for {run_key}: {l6_path}")

        slots = l6_slots_for_window(window, LEAD_IN_MINUTES)
        force_ssr = args.force_ssr and not args.skip_l6
        if not args.skip_l6 or not paths.ssr_csv.exists():
            ssr_summary = expand_ssr_csv(
                paths.l6_concat,
                window,
                paths.ssr_csv,
                lead_in_minutes=LEAD_IN_MINUTES,
                force=force_ssr,
            )
        elif not ssr_csv_covers_window(paths.ssr_csv, window, lead_in_minutes=LEAD_IN_MINUTES):
            print(
                f"=== {run_key}: cached SSR TOW window does not cover run; re-expanding ===",
                flush=True,
            )
            ssr_summary = expand_ssr_csv(
                paths.l6_concat,
                window,
                paths.ssr_csv,
                lead_in_minutes=LEAD_IN_MINUTES,
                force=True,
            )
        else:
            ssr_summary = {"rows_written": "cached", "cached": True}

        command_template = " ".join(
            build_gnss_ppp_command(
                gnss_ppp_bin=args.gnss_ppp,
                rover_obs=paths.rover_obs,
                base_nav=paths.base_nav,
                ssr_csv=paths.ssr_csv,
                out_pos=paths.pos_default,
            )
        )

        config_metrics: dict[str, Any] = {}
        for config_key, pos_path, summary_path in (
            ("default", paths.pos_default, paths.summary_default),
            ("parity", paths.pos_parity, paths.summary_parity),
        ):
            log_path = paths.pos_default.parent / f"{window.key}_{config_key}.log"
            run_ppp = not args.skip_ppp or not pos_path.exists()
            if run_ppp:
                print(f"=== {run_key}/{config_key}: running gnss_ppp ===", flush=True)
                env = os.environ.copy()
                if config_key == "parity":
                    env.update(PARITY_ENV)
                else:
                    for key in PARITY_ENV:
                        env.pop(key, None)
                completed = run_logged(
                    build_gnss_ppp_command(
                        gnss_ppp_bin=args.gnss_ppp,
                        rover_obs=paths.rover_obs,
                        base_nav=paths.base_nav,
                        ssr_csv=paths.ssr_csv,
                        out_pos=pos_path,
                    ),
                    env=env,
                    log_path=log_path,
                )
                if completed.returncode != 0:
                    raise SystemExit(f"gnss_ppp failed for {run_key}/{config_key}; see {log_path}")
            else:
                print(
                    f"=== {run_key}/{config_key}: skipping gnss_ppp (reusing {pos_path.name}) ===",
                    flush=True,
                )

            metrics = score_run(pos_path, paths.reference_csv, paths.rover_obs)
            metrics["notes"] = gap_notes(run_key, config_key, metrics, MRTKLIB_TARGETS[run_key])
            config_metrics[config_key] = metrics
            print(
                f"=== {run_key}/{config_key}: fix={metrics['fix_pct']}% "
                f"rms2d={metrics['rms2d_fixed_m']} ttff={metrics['ttff_s']} ===",
                flush=True,
            )
            summary_path.write_text(json.dumps(metrics, indent=2) + "\n", encoding="utf-8")

        run_results.append(
            {
                "run_key": run_key,
                "gps_week": window.gps_week,
                "gps_tow_start": window.gps_tow_start,
                "gps_tow_end": window.gps_tow_end,
                "utc_start": window.utc_start.isoformat(sep=" "),
                "utc_end": window.utc_end.isoformat(sep=" "),
                "doy": window.doy,
                "year": window.year,
                "l6_slots": slots,
                "ssr_rows": ssr_summary.get("rows_written", "unknown"),
                "configs": config_metrics,
            }
        )

    csv_recipe = (
        "apps/gnss_clas_ppp.expand_qzss_l6_source(): decode QZSS L6 via gnss_qzss_l6_info "
        "(default compact policies: lag-tolerant flush, stec-coeff-carry atmos merge, union subtype merge, "
        "latest-union phase bias, direct code/phase bias composition, pending-epoch banks, overlap-only "
        "bias row materialization, independent row construction) then expand_compact_ssr_text() to gnss_ppp --ssr CSV."
    )
    write_report(
        args.report,
        l6_source=l6_source,
        csv_recipe=csv_recipe,
        command_template=command_template,
        run_results=run_results,
        work_dir=args.work_dir,
        l6_cache=args.l6_cache,
    )
    print(f"Wrote report: {args.report}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
