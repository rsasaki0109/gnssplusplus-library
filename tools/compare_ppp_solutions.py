#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""
Compare LibGNSS++ .pos output to a reference trajectory (.pos) or a single ECEF truth.

.pos format (from Solution::writeToFile):
  Lines starting with '%' are comments.
  Data columns: GPS_Week GPS_TOW X(m) Y(m) Z(m) Lat(deg) Lon(deg) Height(m) Status Satellites PDOP

SolutionStatus (libgnss): NONE=0 SPP=1 DGPS=2 FLOAT=3 FIXED=4 PPP_FLOAT=5 PPP_FIXED=6

Usage:
  python3 compare_ppp_solutions.py --candidate run.pos --reference claslib.pos --last-n 100
  python3 compare_ppp_solutions.py --candidate ours.pos --reference rnx2rtkp.pos --reference-format rtklib-ecef
  # Negative ECEF: use equals so the value is not parsed as another flag:
  python3 compare_ppp_solutions.py --candidate run.pos --ref-ecef=-3957237.16,3310369.08,3737530.92
  python3 compare_ppp_solutions.py --candidate run.pos --ecef-x -3957237.16 --ecef-y 3310369.08 --ecef-z 3737530.92
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from dataclasses import dataclass
from datetime import datetime, timezone
from typing import Any, Iterable, Iterator, Optional

# Unix timestamp (UTC) at 1980-01-06 00:00:00 — start of GPS time (GPST scale).
_GPS_UNIX_EPOCH = datetime(1980, 1, 6, 0, 0, 0, tzinfo=timezone.utc).timestamp()


PPP_FIXED = 6
PPP_FLOAT = 5


@dataclass(frozen=True)
class Epoch:
    week: int
    tow: float
    x: float
    y: float
    z: float
    status: int
    satellites: int
    pdop: float


def _parse_floats_line(line: str) -> Optional[Epoch]:
    parts = line.strip().split()
    if len(parts) < 11:
        return None
    try:
        week = int(parts[0])
        tow = float(parts[1])
        x, y, z = float(parts[2]), float(parts[3]), float(parts[4])
        lat = float(parts[5])
        lon = float(parts[6])
        h = float(parts[7])
        status = int(parts[8])
        sats = int(parts[9])
        pdop = float(parts[10])
    except ValueError:
        return None
    _ = (lat, lon, h)  # present in file; optional for future checks
    return Epoch(week, tow, x, y, z, status, sats, pdop)


def read_libgnss_pos(path: str) -> list[Epoch]:
    out: list[Epoch] = []
    with open(path, "r", encoding="utf-8", errors="replace") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("%"):
                continue
            e = _parse_floats_line(line)
            if e:
                out.append(e)
    return out


def _unix_to_gps_week_tow(unix_t: float) -> tuple[int, float]:
    """Map UTC Unix instant to GPS week and TOW (seconds), GPST scale."""
    gps_s = unix_t - _GPS_UNIX_EPOCH
    if not math.isfinite(gps_s):
        return 0, 0.0
    week = int(gps_s // 604800)
    tow = gps_s - week * 604800.0
    return week, tow


def _rtklib_solq_to_status(solq: int) -> int:
    """Map RTKLIB solution quality (see RTKLIB solstat) to libgnss status codes."""
    # SOLQ_FIX=1, SOLQ_FLOAT=2, SOLQ_PPP=6 in RTKLIB — treat FIX/PPP-AR as FIXED.
    if solq == 1:
        return PPP_FIXED
    if solq == 2:
        return PPP_FLOAT
    if solq == 6:
        return PPP_FLOAT
    return 0


def read_rtklib_gpst_xyz_pos(path: str) -> list[Epoch]:
    """
    RTKLIB/CLASLIB rnx2rtkp text .pos when out-solformat includes ECEF XYZ.

    Supported rows (white-space separated):
      YYYY/MM/DD HH:MM:SS.sss  X(m) Y(m) Z(m)  Q  ns  ...
      week tow  X(m) Y(m) Z(m)  Q  ns  ...
    Lines starting with % or # are skipped.
    """
    out: list[Epoch] = []
    with open(path, "r", encoding="utf-8", errors="replace") as f:
        for raw in f:
            line = raw.strip()
            if not line or line.startswith("%") or line.startswith("#"):
                continue
            parts = line.split()
            if len(parts) < 7:
                continue

            week = 0
            tow = 0.0
            x = y = z = 0.0
            solq_idx = 5
            nsats_idx = 6

            if "/" in parts[0] and len(parts[0]) >= 8:
                date_s = parts[0]
                time_s = parts[1]
                try:
                    x, y, z = float(parts[2]), float(parts[3]), float(parts[4])
                except ValueError:
                    continue
                if not (abs(x) > 1e5 and abs(y) > 1e5 and abs(z) > 1e5):
                    continue
                try:
                    dt = datetime.strptime(
                        f"{date_s} {time_s}", "%Y/%m/%d %H:%M:%S.%f"
                    ).replace(tzinfo=timezone.utc)
                except ValueError:
                    try:
                        dt = datetime.strptime(
                            f"{date_s} {time_s}", "%Y/%m/%d %H:%M:%S"
                        ).replace(tzinfo=timezone.utc)
                    except ValueError:
                        continue
                week, tow = _unix_to_gps_week_tow(dt.timestamp())
            else:
                try:
                    week = int(parts[0])
                    tow = float(parts[1])
                    x, y, z = float(parts[2]), float(parts[3]), float(parts[4])
                except ValueError:
                    continue
                if week <= 0 or tow < 0.0:
                    continue
                if not (abs(x) > 1e5 and abs(y) > 1e5 and abs(z) > 1e5):
                    continue

            solq = int(float(parts[solq_idx])) if len(parts) > solq_idx else 0
            nsats = int(float(parts[nsats_idx])) if len(parts) > nsats_idx else 0
            pdop = 0.0
            status = _rtklib_solq_to_status(solq)
            out.append(
                Epoch(
                    week=week,
                    tow=tow,
                    x=x,
                    y=y,
                    z=z,
                    status=status,
                    satellites=nsats,
                    pdop=pdop,
                )
            )
    return out


def read_pos_autodetect(path: str) -> tuple[list[Epoch], str]:
    """Try libgnss format first, then RTKLIB GPST+XYZ."""
    lg = read_libgnss_pos(path)
    if lg:
        return lg, "libgnss"
    rt = read_rtklib_gpst_xyz_pos(path)
    if rt:
        return rt, "rtklib-ecef"
    return [], "none"


def _time_key(e: Epoch) -> tuple[int, float]:
    """Align LibGNSS and RTKLIB/CLASLIB rows (GPST from date vs stored TOW)."""
    return (e.week, round(e.tow, 4))


def index_by_time(epochs: Iterable[Epoch]) -> dict[tuple[int, float], Epoch]:
    idx: dict[tuple[int, float], Epoch] = {}
    for e in epochs:
        idx[_time_key(e)] = e
    return idx


def ecef_delta_to_enu(dx: float, dy: float, dz: float, lat0_rad: float, lon0_rad: float) -> tuple[float, float, float]:
    """Rotate ECEF delta into ENU at reference (lat0, lon0) radians."""
    sl = math.sin(lat0_rad)
    cl = math.cos(lat0_rad)
    so = math.sin(lon0_rad)
    co = math.cos(lon0_rad)
    east = -so * dx + co * dy
    north = -sl * co * dx - sl * so * dy + cl * dz
    up = cl * co * dx + cl * so * dy + sl * dz
    return east, north, up


def ecef_to_llh(x: float, y: float, z: float) -> tuple[float, float, float]:
    """WGS84 approximate: returns lat, lon, h in radians / meters."""
    a = 6378137.0
    f = 1.0 / 298.257223563
    e2 = f * (2.0 - f)
    lon = math.atan2(y, x)
    p = math.hypot(x, y)
    lat = math.atan2(z, p * (1.0 - e2))
    for _ in range(5):
        sl = math.sin(lat)
        n = a / math.sqrt(1.0 - e2 * sl * sl)
        lat = math.atan2(z + e2 * n * sl, p)
    sl = math.sin(lat)
    cl = math.cos(lat)
    n = a / math.sqrt(1.0 - e2 * sl * sl)
    h = p / cl - n
    return lat, lon, h


def rms(values: list[float]) -> float:
    if not values:
        return float("nan")
    return math.sqrt(sum(v * v for v in values) / len(values))


def statistics_for_errors(
    errs_e: list[float],
    errs_n: list[float],
    errs_u: list[float],
    errs_3d: list[float],
) -> dict[str, float]:
    return {
        "rms_e_m": rms(errs_e),
        "rms_n_m": rms(errs_n),
        "rms_h_m": rms([math.hypot(e, n) for e, n in zip(errs_e, errs_n)]),
        "rms_v_m": rms(errs_u),
        "rms_3d_m": rms(errs_3d),
        "max_3d_m": max(errs_3d) if errs_3d else float("nan"),
    }


def pick_last_n(epochs: list[Epoch], valid_only: bool, n: int) -> list[Epoch]:
    if n <= 0:
        return epochs
    seq = [e for e in epochs if e.status != 0 and e.satellites >= 4] if valid_only else list(epochs)
    return seq[-n:] if len(seq) >= n else seq


def time_to_first_fix(epochs: list[Epoch], fixed_status: int = PPP_FIXED) -> Optional[float]:
    if not epochs:
        return None
    t0 = epochs[0].tow + epochs[0].week * 604800.0
    for e in epochs:
        if e.status == fixed_status:
            t = e.tow + e.week * 604800.0
            return t - t0
    return None


def compare_to_point(
    epochs: list[Epoch],
    xref: float,
    yref: float,
    zref: float,
    last_n: int,
) -> dict[str, Any]:
    lat0, lon0, _ = ecef_to_llh(xref, yref, zref)
    use = pick_last_n(epochs, valid_only=True, n=last_n)
    errs_e: list[float] = []
    errs_n: list[float] = []
    errs_u: list[float] = []
    errs_3d: list[float] = []
    for e in use:
        dx = e.x - xref
        dy = e.y - yref
        dz = e.z - zref
        east, north, up = ecef_delta_to_enu(dx, dy, dz, lat0, lon0)
        errs_e.append(east)
        errs_n.append(north)
        errs_u.append(up)
        errs_3d.append(math.sqrt(dx * dx + dy * dy + dz * dz))
    stats = statistics_for_errors(errs_e, errs_n, errs_u, errs_3d)
    stats["epochs_used"] = len(use)
    stats["reference_mode"] = "ecef_point"
    return stats


def compare_to_trajectory(
    candidate: list[Epoch],
    reference: list[Epoch],
    last_n: int,
) -> dict[str, Any]:
    ref_idx = index_by_time(reference)
    aligned_c: list[Epoch] = []
    aligned_r: list[Epoch] = []
    for e in candidate:
        k = _time_key(e)
        if k in ref_idx:
            aligned_c.append(e)
            aligned_r.append(ref_idx[k])
    if last_n > 0 and len(aligned_c) > last_n:
        aligned_c = aligned_c[-last_n:]
        aligned_r = aligned_r[-last_n:]
    if not aligned_c:
        return {
            "reference_mode": "trajectory",
            "epochs_used": 0,
            "error": "no_matching_epochs",
        }

    # ENU at each reference point (per-epoch rotation)
    errs_e: list[float] = []
    errs_n: list[float] = []
    errs_u: list[float] = []
    errs_3d: list[float] = []
    for c, r in zip(aligned_c, aligned_r):
        dx = c.x - r.x
        dy = c.y - r.y
        dz = c.z - r.z
        lat0, lon0, _ = ecef_to_llh(r.x, r.y, r.z)
        east, north, up = ecef_delta_to_enu(dx, dy, dz, lat0, lon0)
        errs_e.append(east)
        errs_n.append(north)
        errs_u.append(up)
        errs_3d.append(math.sqrt(dx * dx + dy * dy + dz * dz))
    stats = statistics_for_errors(errs_e, errs_n, errs_u, errs_3d)
    stats["epochs_used"] = len(aligned_c)
    stats["reference_mode"] = "trajectory"
    return stats


def summarize_candidate(epochs: list[Epoch], last_n: int) -> dict[str, Any]:
    total = len(epochs)
    valid = [e for e in epochs if e.status != 0 and e.satellites >= 4]
    fixed = [e for e in valid if e.status == PPP_FIXED]
    window = pick_last_n(epochs, valid_only=True, n=last_n if last_n > 0 else total)
    fix_in_window = sum(1 for e in window if e.status == PPP_FIXED)
    return {
        "total_lines": total,
        "valid_epochs": len(valid),
        "fixed_epochs": len(fixed),
        "fix_rate_all_valid": (len(fixed) / len(valid)) if valid else 0.0,
        "window_epochs": len(window),
        "fixed_in_window": fix_in_window,
        "fix_rate_in_window": (fix_in_window / len(window)) if window else 0.0,
        "time_to_first_fix_s": time_to_first_fix(epochs),
    }


def main() -> int:
    ap = argparse.ArgumentParser(description="Compare LibGNSS++ .pos to reference point or .pos trajectory.")
    ap.add_argument("--candidate", required=True, help=".pos from gnss_ppp (libgnss) or rtklib-ecef")
    ap.add_argument(
        "--candidate-format",
        choices=("auto", "libgnss", "rtklib-ecef"),
        default="auto",
        help="Solution file format (default: auto-detect)",
    )
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--reference", help="Reference .pos (libgnss, rtklib-ecef per --reference-format)")
    g.add_argument(
        "--ref-ecef",
        metavar="X,Y,Z",
        help="Single truth ECEF meters comma-separated. For negative X use --ref-ecef=-x,y,z or --ecef-x/-y/-z.",
    )
    g.add_argument(
        "--ecef-x",
        type=float,
        default=None,
        help="With --ecef-y and --ecef-z, single reference ECEF (avoids argparse issues with negative coords).",
    )
    ap.add_argument("--ecef-y", type=float, default=None)
    ap.add_argument("--ecef-z", type=float, default=None)
    ap.add_argument("--last-n", type=int, default=100, help="Use only last N valid epochs for error stats (0=all matched)")
    ap.add_argument("--json-out", help="Write full report as JSON")
    ap.add_argument(
        "--reference-format",
        choices=("auto", "libgnss", "rtklib-ecef"),
        default="auto",
        help="Reference file format when using --reference (default: auto-detect)",
    )
    args = ap.parse_args()

    if args.candidate_format == "libgnss":
        cand = read_libgnss_pos(args.candidate)
        cand_fmt = "libgnss"
    elif args.candidate_format == "rtklib-ecef":
        cand = read_rtklib_gpst_xyz_pos(args.candidate)
        cand_fmt = "rtklib-ecef"
    else:
        cand, cand_fmt = read_pos_autodetect(args.candidate)
    if not cand:
        print("No epochs parsed from candidate.", file=sys.stderr)
        return 2

    summary = summarize_candidate(cand, args.last_n)

    report: dict[str, Any] = {
        "candidate_file": args.candidate,
        "candidate_format": cand_fmt,
        "candidate_summary": summary,
    }

    if args.reference:
        if args.reference_format == "libgnss":
            ref = read_libgnss_pos(args.reference)
            ref_fmt = "libgnss"
        elif args.reference_format == "rtklib-ecef":
            ref = read_rtklib_gpst_xyz_pos(args.reference)
            ref_fmt = "rtklib-ecef"
        else:
            ref, ref_fmt = read_pos_autodetect(args.reference)
        if not ref:
            print("No epochs parsed from reference.", file=sys.stderr)
            return 2
        report["reference_file"] = args.reference
        report["reference_format"] = ref_fmt
        report["error_vs_reference"] = compare_to_trajectory(cand, ref, args.last_n)
    elif args.ecef_x is not None:
        if args.ecef_y is None or args.ecef_z is None:
            print("--ecef-x requires --ecef-y and --ecef-z", file=sys.stderr)
            return 2
        xref, yref, zref = args.ecef_x, args.ecef_y, args.ecef_z
        report["ref_ecef_m"] = [xref, yref, zref]
        report["error_vs_reference"] = compare_to_point(cand, xref, yref, zref, args.last_n)
    else:
        assert args.ref_ecef is not None
        parts = args.ref_ecef.split(",")
        if len(parts) != 3:
            print("--ref-ecef must be X,Y,Z", file=sys.stderr)
            return 2
        xref, yref, zref = float(parts[0]), float(parts[1]), float(parts[2])
        report["ref_ecef_m"] = [xref, yref, zref]
        report["error_vs_reference"] = compare_to_point(cand, xref, yref, zref, args.last_n)

    if args.json_out:
        with open(args.json_out, "w", encoding="utf-8") as jf:
            json.dump(report, jf, indent=2)

    # Human-readable
    ev = report["error_vs_reference"]
    print("=== Candidate ===")
    print(f"  format: {report.get('candidate_format', 'libgnss')}")
    print(f"  valid epochs: {summary['valid_epochs']} / {summary['total_lines']}")
    print(f"  fix rate (all valid): {100.0 * summary['fix_rate_all_valid']:.2f}%")
    if summary["window_epochs"]:
        print(
            f"  fix rate (last {summary['window_epochs']} valid): "
            f"{100.0 * summary['fix_rate_in_window']:.2f}%"
        )
    ttf = summary["time_to_first_fix_s"]
    print(f"  time to first PPP_FIXED: {ttf if ttf is not None else 'n/a'} s")

    print("=== Error vs reference ===")
    if ev.get("error"):
        print(f"  {ev['error']}")
    else:
        print(f"  epochs in stats: {ev.get('epochs_used', 0)}")
        print(f"  RMS H: {ev.get('rms_h_m', float('nan')):.4f} m")
        print(f"  RMS V (ENU up): {ev.get('rms_v_m', float('nan')):.4f} m")
        print(f"  RMS 3D: {ev.get('rms_3d_m', float('nan')):.4f} m")
        print(f"  max 3D: {ev.get('max_3d_m', float('nan')):.4f} m")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
