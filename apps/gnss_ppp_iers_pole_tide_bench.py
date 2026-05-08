#!/usr/bin/env python3
"""Paired PPP run with the IERS Conventions 2010 pole-tide opt-in
flag toggled, summarizing the per-epoch displacement between the two
solutions.

This is the truth-bench harness for Phase D-1 (PR #62). It runs
``gnss_ppp`` twice on the same input observations / navigation /
products — once with ``--no-iers-pole-tide`` (today's default; the
pole tide is omitted entirely) and once with ``--use-iers-pole-tide``
(the IERS Conventions 2010 §7.1.4 pole-tide model from the
libgnss::iers wrapper). It reports the displacement statistics
between the two .pos outputs.

Unlike the solid-tide bench (#57), the pole-tide path requires an
EOP series — the displacement is driven by instantaneous polar
motion (xp, yp). The bench therefore takes ``--eop-c04`` as a
mandatory argument; running without it would silently produce
identical outputs (the pole-tide path is a no-op when the EOP table
is unavailable, by design).

Typical use:

    python apps/gnss_ppp_iers_pole_tide_bench.py \\
        --obs path/to/rover.obs \\
        --nav path/to/nav.rnx \\
        --sp3 path/to/orbit.sp3 \\
        --clk path/to/clock.clk \\
        --eop-c04 data/iers/eopc04.1962-now.txt \\
        --output-dir output/iers_pole_tide_bench \\
        --max-epochs 600

The script writes:
    <output-dir>/legacy.pos       — gnss_ppp output without pole tide
    <output-dir>/iers.pos         — gnss_ppp output with pole tide
    <output-dir>/comparison.json  — paired-epoch displacement stats

See docs/iers-integration-plan.md for the rollout plan this harness
participates in.
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
        "--eop-c04", type=Path, required=True,
        help="IERS 20 C04 EOP file (mandatory). Without it the pole-tide "
             "path is a no-op and the bench would report zero displacement. "
             "Source: https://hpiers.obspm.fr/iers/eop/eopc04/eopc04.1962-now")
    parser.add_argument(
        "--mode", choices=("static", "kinematic"), default="static",
        help="PPP motion model. (default: static)")
    parser.add_argument(
        "--max-epochs", type=int, default=0,
        help="Limit processed epochs in each PPP run (default: 0 == all).")
    parser.add_argument(
        "--output-dir", type=Path,
        default=ROOT_DIR / "output" / "iers_pole_tide_bench",
        help="Directory for paired .pos outputs and the comparison summary.")
    parser.add_argument(
        "--require-max-displacement-m", type=float, default=None,
        help="Optional acceptance gate: fail if the maximum per-epoch "
             "ECEF displacement between the two paths exceeds this many "
             "meters. The pole tide is sub-cm horizontal and "
             "≤ ~25 mm radial during large polar-motion excursions, so "
             "values in the 0.005-0.05 m range are reasonable upper "
             "bounds for sanity checking.")
    return parser.parse_args()


def read_pos_records(path: Path) -> list[dict[str, float | int]]:
    """Parse a libgnss++ ``.pos`` solution file into a list of
    per-epoch dicts. Mirrors the parser pattern used by the
    solid-tide bench (#57) so the field semantics are consistent."""
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


def run_ppp(
    base_command: list[str],
    args: argparse.Namespace,
    out_pos: Path,
    use_iers: bool,
) -> None:
    """Invoke ``gnss ppp`` for one of the two paired runs."""
    cmd = [
        *base_command, "ppp",
        "--obs", str(args.obs),
        "--out", str(out_pos),
        "--quiet",
        "--eop-c04", str(args.eop_c04),
    ]
    if args.nav is not None:
        cmd += ["--nav", str(args.nav)]
    if args.sp3 is not None:
        cmd += ["--sp3", str(args.sp3)]
    if args.clk is not None:
        cmd += ["--clk", str(args.clk)]
    if args.ionex is not None:
        cmd += ["--ionex", str(args.ionex)]
    if args.dcb is not None:
        cmd += ["--dcb", str(args.dcb)]
    if args.antex is not None:
        cmd += ["--antex", str(args.antex)]
    if args.blq is not None:
        cmd += ["--blq", str(args.blq)]
        if args.ocean_loading_station is not None:
            cmd += ["--ocean-loading-station", args.ocean_loading_station]
    if args.mode == "kinematic":
        cmd += ["--kinematic"]
    if args.max_epochs > 0:
        cmd += ["--max-epochs", str(args.max_epochs)]
    cmd += ["--use-iers-pole-tide" if use_iers else "--no-iers-pole-tide"]

    label = "iers" if use_iers else "legacy"
    print(f"[bench] running {label} PPP -> {out_pos}", file=sys.stderr)
    subprocess.run(cmd, check=True)


def epoch_key(record: dict) -> tuple[int, int]:
    """Match epochs across the two paired runs by (week, tow_microseconds).
    Microsecond-rounding the TOW absorbs the ``.pos`` writer's printf
    formatting so we don't double-match on float comparison."""
    return (int(record["week"]), int(round(float(record["tow"]) * 1e6)))


def summarize(legacy_pos: Path, iers_pos: Path) -> dict:
    legacy = {epoch_key(r): r for r in read_pos_records(legacy_pos)}
    iers = {epoch_key(r): r for r in read_pos_records(iers_pos)}
    matched_keys = sorted(set(legacy) & set(iers))

    # Per-epoch ECEF displacement magnitude (legacy -> iers).
    displacements_m: list[float] = []
    # Per-component signed deltas. The pole tide produces a slowly
    # varying signal driven by polar-motion excursions, which shows
    # up most clearly as the *median* per-component displacement
    # (insensitive to per-epoch Kalman noise).
    dx_m: list[float] = []
    dy_m: list[float] = []
    dz_m: list[float] = []
    for key in matched_keys:
        l = legacy[key]
        i = iers[key]
        dx = float(i["x"]) - float(l["x"])
        dy = float(i["y"]) - float(l["y"])
        dz = float(i["z"]) - float(l["z"])
        dx_m.append(dx)
        dy_m.append(dy)
        dz_m.append(dz)
        displacements_m.append(math.sqrt(dx * dx + dy * dy + dz * dz))

    summary = {
        "matched_epochs": len(matched_keys),
        "legacy_total_epochs": len(legacy),
        "iers_total_epochs": len(iers),
    }
    if displacements_m:
        sorted_d = sorted(displacements_m)
        summary.update({
            "max_displacement_m": sorted_d[-1],
            "min_displacement_m": sorted_d[0],
            "mean_displacement_m": statistics.fmean(displacements_m),
            "median_displacement_m": statistics.median(displacements_m),
            "p95_displacement_m": sorted_d[int(0.95 * (len(sorted_d) - 1))],
            # First matched epoch is the best estimate of the
            # pole-tide-only contribution: the Kalman filter has not
            # yet had time to integrate any divergence between the
            # two paths, so the displacement here reflects the
            # immediate effect of switching pole-tide on/off at the
            # receiver position.
            "first_epoch_displacement_m": displacements_m[0],
            # Per-component median deltas. The pole tide signal scale
            # at typical mid-latitudes during normal polar motion is
            # ~mm radial and sub-mm horizontal. If the |median|
            # component values are at this scale and the aggregate
            # magnitudes (max / p95 above) are much larger, the run
            # is being dominated by Kalman trajectory noise rather
            # than the tide model — use first_epoch_displacement_m
            # and the medians as the trustworthy signals in that
            # case.
            "median_dx_m": statistics.median(dx_m),
            "median_dy_m": statistics.median(dy_m),
            "median_dz_m": statistics.median(dz_m),
        })
        # Health flag: an order-of-magnitude gap between
        # first-epoch and aggregate signals usually means
        # the trajectory has diverged faster than the tide
        # signal can be measured.
        if displacements_m[0] > 1e-9:
            summary["aggregate_to_first_epoch_ratio"] = (
                sorted_d[-1] / displacements_m[0])
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
    ensure_input_exists(args.eop_c04, "IERS C04 EOP file", ROOT_DIR)

    args.output_dir.mkdir(parents=True, exist_ok=True)
    legacy_pos = args.output_dir / "legacy.pos"
    iers_pos = args.output_dir / "iers.pos"
    comparison_json = args.output_dir / "comparison.json"

    base_command = resolve_gnss_command(ROOT_DIR)
    run_ppp(base_command, args, legacy_pos, use_iers=False)
    run_ppp(base_command, args, iers_pos, use_iers=True)

    summary = summarize(legacy_pos, iers_pos)
    summary["legacy_pos"] = str(legacy_pos)
    summary["iers_pos"] = str(iers_pos)

    comparison_json.write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(summary, indent=2, sort_keys=True))

    if (args.require_max_displacement_m is not None
            and "max_displacement_m" in summary
            and summary["max_displacement_m"] >
                args.require_max_displacement_m):
        print(
            f"FAIL: max displacement {summary['max_displacement_m']:.6f} m "
            f"exceeds gate {args.require_max_displacement_m} m",
            file=sys.stderr,
        )
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
