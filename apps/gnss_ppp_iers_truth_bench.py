#!/usr/bin/env python3
"""End-to-end PPP truth bench for the IERS Conventions 2010 stack.

The Phase A-D PR cascade (#52-#76) lifted the libgnss++ PPP path
through the full IERS Conventions 2010 modeling: SOFA-derived
earth rotation, Dehant Step-1+Step-2 solid tide (default true,
PR #59), §7.1.4 pole tide (default true, PR #70), §5.5.1.1 +
§8.2 sub-daily EOP (default true, PR #71), §7.1.2 HARDISP ocean
loading (opt-in, PR #60), §7.1.5 atmospheric tidal loading
(opt-in, PR #66). The per-effect bench harnesses (#57 / #63 /
#65 / #66 / #68 / #75 / #76) report per-epoch DELTA between
toggled paths — they verify the model is doing something, but
not that the resulting absolute position is correct.

This bench closes that loop: it runs ``gnss_ppp`` end-to-end
with all IERS defaults ON, takes the converged static-mode
average from the .pos solution, and compares it to the station's
RINEX-header reference position. Optionally it ALSO runs a
matching all-IERS-OFF arc (``--ab``) so the residual delta from
turning IERS modeling ON can be quantified directly.

The harness is the natural cap on the Phase A-D rollout: the
per-effect benches show each model is faithful, this bench shows
the integrated stack delivers cm-class absolute position against
known IGS station coordinates.

Typical use::

    python apps/gnss_ppp_iers_truth_bench.py \\
        --obs path/to/rover.obs \\
        --nav path/to/nav.rnx \\
        --sp3 path/to/orbit.sp3 \\
        --clk path/to/clock.clk \\
        --eop-c04 data/iers/finals2000A.daily \\
        --converged-tail-epochs 600 \\
        --ab \\
        --output-dir output/iers_truth_bench

The script writes:
    <output-dir>/iers_on.pos    — gnss_ppp output with all IERS defaults
    <output-dir>/iers_off.pos   — gnss_ppp output with all IERS off (--ab)
    <output-dir>/comparison.json — residual statistics

The reference position is the RINEX OBS header's
``APPROX POSITION XYZ`` field, which IGS stations keep at the
published ITRF coordinate (typically mm-accurate). For non-IGS
benches users can override with ``--reference-xyz``.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import statistics
import subprocess
import sys
from pathlib import Path

from gnss_runtime import ensure_input_exists, resolve_gnss_command


ROOT_DIR = Path(__file__).resolve().parent.parent


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(prog=os.environ.get("GNSS_CLI_NAME"))
    parser.add_argument("--obs", type=Path, required=True,
                        help="Rover RINEX observation file.")
    parser.add_argument("--nav", type=Path, default=None,
                        help="Optional broadcast navigation file.")
    parser.add_argument("--sp3", type=Path, default=None,
                        help="Optional precise orbit file (SP3).")
    parser.add_argument("--clk", type=Path, default=None,
                        help="Optional precise clock file (CLK).")
    parser.add_argument("--ionex", type=Path, default=None,
                        help="Optional IONEX TEC map.")
    parser.add_argument("--dcb", type=Path, default=None,
                        help="Optional DCB / Bias-SINEX product.")
    parser.add_argument("--antex", type=Path, default=None,
                        help="Optional ANTEX antenna file.")
    parser.add_argument("--blq", type=Path, default=None,
                        help="Optional BLQ ocean-loading coefficient file.")
    parser.add_argument("--ocean-loading-station", default=None,
                        help="Station name to select from the BLQ file.")
    parser.add_argument(
        "--eop-c04", type=Path, default=None,
        help="IERS 20 C04 or Bulletin A EOP file. Recommended: without "
             "it the pole-tide and sub-daily-EOP paths are no-ops "
             "(silently degraded, not an error).")
    parser.add_argument(
        "--reference-xyz", type=float, nargs=3, default=None,
        metavar=("X", "Y", "Z"),
        help="ECEF reference coordinates (m). If not given, the bench "
             "reads APPROX POSITION XYZ from the RINEX OBS header.")
    parser.add_argument(
        "--converged-tail-epochs", type=int, default=600,
        help="Average over the last N epochs only (default 600). "
             "PPP needs ~20-30 minutes to converge from cold start; "
             "averaging the tail gives the static converged solution.")
    parser.add_argument(
        "--mode", choices=("static", "kinematic"), default="static",
        help="PPP motion model. (default: static)")
    parser.add_argument(
        "--max-epochs", type=int, default=0,
        help="Limit processed epochs in the PPP run (default: 0 == all).")
    parser.add_argument(
        "--ab", action="store_true",
        help="Also run a matching all-IERS-OFF arc and report the "
             "delta in residual-to-truth between IERS-on and "
             "IERS-off. Doubles the runtime.")
    parser.add_argument(
        "--output-dir", type=Path,
        default=ROOT_DIR / "output" / "iers_truth_bench",
        help="Directory for paired .pos outputs and the residual summary.")
    return parser.parse_args()


def read_approx_position(path: Path) -> tuple[float, float, float]:
    """Read APPROX POSITION XYZ from a RINEX 2/3/4 OBS header."""
    with path.open(encoding="ascii", errors="ignore") as handle:
        for line in handle:
            if "APPROX POSITION XYZ" in line:
                fields = line[:60].split()
                if len(fields) < 3:
                    break
                return (float(fields[0]), float(fields[1]),
                        float(fields[2]))
            if "END OF HEADER" in line:
                break
    raise SystemExit(
        f"Failed to read APPROX POSITION XYZ from {path}; "
        "pass --reference-xyz instead.")


def read_pos_records(path: Path) -> list[dict[str, float | int]]:
    """Parse a libgnss++ .pos solution file."""
    records: list[dict[str, float | int]] = []
    with path.open(encoding="ascii") as handle:
        for line in handle:
            if not line.strip() or line.startswith("%"):
                continue
            parts = line.split()
            if len(parts) < 10:
                continue
            records.append({
                "week": int(float(parts[0])),
                "tow": float(parts[1]),
                "x": float(parts[2]),
                "y": float(parts[3]),
                "z": float(parts[4]),
                "status": int(parts[8]),
                "satellites": int(parts[9]),
            })
    return records


def ecef_to_enu(d_xyz: tuple[float, float, float],
                origin_xyz: tuple[float, float, float]
                ) -> tuple[float, float, float]:
    """Rotate an ECEF delta into a local East-North-Up frame at the
    origin geocentric latitude / longitude. Geodetic latitude would
    add an WGS84 flattening rotation; geocentric is exact for the
    rotation matrix and accurate enough for a few-cm residual."""
    x, y, z = origin_xyz
    r_xy = math.sqrt(x * x + y * y)
    phi = math.atan2(z, r_xy)
    lam = math.atan2(y, x)
    s_phi, c_phi = math.sin(phi), math.cos(phi)
    s_lam, c_lam = math.sin(lam), math.cos(lam)
    dx, dy, dz = d_xyz
    e = -s_lam * dx + c_lam * dy
    n = -s_phi * c_lam * dx - s_phi * s_lam * dy + c_phi * dz
    u =  c_phi * c_lam * dx + c_phi * s_lam * dy + s_phi * dz
    return (e, n, u)


def run_ppp(
    base_command: list[str],
    args: argparse.Namespace,
    out_pos: Path,
    iers_on: bool,
) -> None:
    """Invoke gnss ppp once. With ``iers_on=True`` the command line
    relies on the post-#59/#70/#71 defaults and adds --eop-c04 when
    available; with ``iers_on=False`` every IERS toggle is forced
    OFF for the all-IERS-off A/B arm."""
    cmd = [
        *base_command, "ppp",
        "--obs", str(args.obs),
        "--out", str(out_pos),
        "--quiet",
    ]
    if args.nav is not None: cmd += ["--nav", str(args.nav)]
    if args.sp3 is not None: cmd += ["--sp3", str(args.sp3)]
    if args.clk is not None: cmd += ["--clk", str(args.clk)]
    if args.ionex is not None: cmd += ["--ionex", str(args.ionex)]
    if args.dcb is not None: cmd += ["--dcb", str(args.dcb)]
    if args.antex is not None: cmd += ["--antex", str(args.antex)]
    if args.blq is not None:
        cmd += ["--blq", str(args.blq)]
        if args.ocean_loading_station is not None:
            cmd += ["--ocean-loading-station", args.ocean_loading_station]
    if args.mode == "kinematic":
        cmd += ["--kinematic"]
    if args.max_epochs > 0:
        cmd += ["--max-epochs", str(args.max_epochs)]
    if iers_on:
        if args.eop_c04 is not None:
            cmd += ["--eop-c04", str(args.eop_c04)]
        # Solid-tide / pole-tide / sub-daily-EOP defaults are already
        # ON post-#59/#70/#71, so no explicit flag needed. Be
        # explicit anyway for forward-compat with future default
        # changes.
        cmd += ["--use-iers-solid-tide",
                "--use-iers-pole-tide",
                "--use-iers-sub-daily-eop"]
    else:
        cmd += ["--no-iers-solid-tide",
                "--no-iers-pole-tide",
                "--no-iers-sub-daily-eop"]

    label = "iers_on" if iers_on else "iers_off"
    print(f"[truth] running {label} PPP -> {out_pos}", file=sys.stderr)
    subprocess.run(cmd, check=True)


def residual_summary(records: list[dict],
                     reference_xyz: tuple[float, float, float],
                     tail: int) -> dict:
    """Average the converged tail of a PPP solution and report the
    residual to the reference station coordinates in ECEF and ENU."""
    if not records:
        return {"n_records": 0, "n_tail": 0}
    tail = min(tail, len(records))
    tail_records = records[-tail:]
    n_status = sum(1 for r in tail_records if r["status"] not in (0, 5))
    xs = [r["x"] for r in tail_records]
    ys = [r["y"] for r in tail_records]
    zs = [r["z"] for r in tail_records]
    avg_x = statistics.fmean(xs)
    avg_y = statistics.fmean(ys)
    avg_z = statistics.fmean(zs)
    dx = avg_x - reference_xyz[0]
    dy = avg_y - reference_xyz[1]
    dz = avg_z - reference_xyz[2]
    horiz = math.sqrt(dx * dx + dy * dy)
    full = math.sqrt(dx * dx + dy * dy + dz * dz)
    e, n, u = ecef_to_enu((dx, dy, dz), reference_xyz)
    horiz_enu = math.sqrt(e * e + n * n)
    summary = {
        "n_records": len(records),
        "n_tail":    tail,
        "n_tail_converged_status": n_status,
        "reference_x_m":     reference_xyz[0],
        "reference_y_m":     reference_xyz[1],
        "reference_z_m":     reference_xyz[2],
        "tail_avg_x_m":      avg_x,
        "tail_avg_y_m":      avg_y,
        "tail_avg_z_m":      avg_z,
        "residual_dx_m":     dx,
        "residual_dy_m":     dy,
        "residual_dz_m":     dz,
        "residual_de_m":     e,
        "residual_dn_m":     n,
        "residual_du_m":     u,
        "horizontal_ecef_m": horiz,
        "horizontal_enu_m":  horiz_enu,
        "vertical_enu_m":    abs(u),
        "distance_3d_m":     full,
    }
    return summary


def main() -> int:
    args = parse_args()

    ensure_input_exists(args.obs, "observation file", ROOT_DIR)
    if args.nav is not None:
        ensure_input_exists(args.nav, "navigation file", ROOT_DIR)
    if args.sp3 is not None:
        ensure_input_exists(args.sp3, "SP3 orbit file", ROOT_DIR)
    if args.clk is not None:
        ensure_input_exists(args.clk, "clock file", ROOT_DIR)
    if args.eop_c04 is not None:
        ensure_input_exists(args.eop_c04, "IERS EOP file", ROOT_DIR)

    if args.reference_xyz is None:
        ref = read_approx_position(args.obs)
    else:
        ref = (args.reference_xyz[0],
               args.reference_xyz[1],
               args.reference_xyz[2])

    args.output_dir.mkdir(parents=True, exist_ok=True)
    iers_on_pos = args.output_dir / "iers_on.pos"
    iers_off_pos = args.output_dir / "iers_off.pos"
    comparison_json = args.output_dir / "comparison.json"

    base_command = resolve_gnss_command(ROOT_DIR)
    run_ppp(base_command, args, iers_on_pos, iers_on=True)
    iers_on = read_pos_records(iers_on_pos)
    on_summary = residual_summary(iers_on, ref, args.converged_tail_epochs)

    summary: dict = {
        "reference_source":
            "rinex_approx_position_xyz" if args.reference_xyz is None
            else "user_supplied",
        "reference_x_m": ref[0],
        "reference_y_m": ref[1],
        "reference_z_m": ref[2],
        "iers_on": on_summary,
        "iers_on_pos": str(iers_on_pos),
    }

    if args.ab:
        run_ppp(base_command, args, iers_off_pos, iers_on=False)
        iers_off = read_pos_records(iers_off_pos)
        off_summary = residual_summary(
            iers_off, ref, args.converged_tail_epochs)
        summary["iers_off"] = off_summary
        summary["iers_off_pos"] = str(iers_off_pos)
        # Improvement = how much closer to truth IERS-on lands vs
        # IERS-off, per component. Positive value = IERS-on is closer.
        on_dist = on_summary.get("distance_3d_m")
        off_dist = off_summary.get("distance_3d_m")
        on_h = on_summary.get("horizontal_enu_m")
        off_h = off_summary.get("horizontal_enu_m")
        on_v = on_summary.get("vertical_enu_m")
        off_v = off_summary.get("vertical_enu_m")
        if (on_dist is not None and off_dist is not None
                and on_h is not None and off_h is not None
                and on_v is not None and off_v is not None):
            summary["delta"] = {
                "horizontal_enu_m_off_minus_on": off_h - on_h,
                "vertical_enu_m_off_minus_on":   off_v - on_v,
                "distance_3d_m_off_minus_on":    off_dist - on_dist,
                "iers_on_closer_to_truth":       on_dist < off_dist,
            }

    comparison_json.write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(summary, indent=2, sort_keys=True))

    return 0


if __name__ == "__main__":
    sys.exit(main())
