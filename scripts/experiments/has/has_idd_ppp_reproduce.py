#!/usr/bin/env python3
"""Score float-PPP runs against a static reference coordinate.

Used by the `gnss reproduce has-idd-ppp` lane (Galileo HAS corrections from
the HAS Internet Data Distribution, station OBE4). For each libgnss++ `.pos`
file it reports the horizontal / vertical error at fixed elapsed times and the
convergence time, i.e. the first epoch after which the error stays below the
threshold until the end of the run.

    has_idd_ppp_reproduce.py score \
        --reference-ecef 4186704.2262 834903.7677 4723664.9337 \
        --run static=out/static.pos --run kinematic=out/kinematic.pos \
        --summary-json out/summary.json
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Sequence

CHECKPOINT_MINUTES = (10, 20, 30, 60)
HORIZONTAL_THRESHOLD_M = 0.20
VERTICAL_THRESHOLD_M = 0.40
WGS84_A = 6378137.0
WGS84_F = 1.0 / 298.257223563


def ecef_to_geodetic_rad(x: float, y: float, z: float) -> tuple[float, float]:
    e2 = WGS84_F * (2.0 - WGS84_F)
    lon = math.atan2(y, x)
    p = math.hypot(x, y)
    lat = math.atan2(z, p * (1.0 - e2))
    for _ in range(10):
        n = WGS84_A / math.sqrt(1.0 - e2 * math.sin(lat) ** 2)
        h = p / math.cos(lat) - n
        lat = math.atan2(z, p * (1.0 - e2 * n / (n + h)))
    return lat, lon


def ecef_delta_to_enu(delta: Sequence[float], lat: float, lon: float) -> tuple[float, float, float]:
    dx, dy, dz = delta
    sl, cl = math.sin(lat), math.cos(lat)
    so, co = math.sin(lon), math.cos(lon)
    east = -so * dx + co * dy
    north = -sl * co * dx - sl * so * dy + cl * dz
    up = cl * co * dx + cl * so * dy + sl * dz
    return east, north, up


def read_pos(path: Path) -> list[tuple[float, float, float, float]]:
    """Return (seconds_since_first_epoch, x, y, z) rows of a libgnss++ .pos file."""
    rows: list[tuple[float, float, float, float]] = []
    for line in path.read_text(encoding="utf-8").splitlines():
        if not line.strip() or line.startswith(("%", "#")):
            continue
        fields = line.split()
        try:
            week = int(fields[0])
            tow = float(fields[1])
            x, y, z = (float(value) for value in fields[2:5])
        except (ValueError, IndexError):
            continue
        rows.append((week * 604800.0 + tow, x, y, z))
    if not rows:
        raise SystemExit(f"no solution epochs in {path}")
    start = rows[0][0]
    return [(t - start, x, y, z) for t, x, y, z in rows]


def convergence_seconds(times: Sequence[float], ok: Sequence[bool]) -> float | None:
    """First time after which every remaining epoch satisfies the threshold."""
    result: float | None = None
    for t, good in zip(reversed(times), reversed(ok)):
        if not good:
            break
        result = t
    return result


def score_run(path: Path, reference: Sequence[float]) -> dict:
    lat, lon = ecef_to_geodetic_rad(*reference)
    rows = read_pos(path)
    times = [row[0] for row in rows]
    enu = [ecef_delta_to_enu((x - reference[0], y - reference[1], z - reference[2]), lat, lon)
           for _, x, y, z in rows]
    horizontal = [math.hypot(e, n) for e, n, _ in enu]
    vertical = [u for _, _, u in enu]
    checkpoints = {}
    for minutes in CHECKPOINT_MINUTES:
        target = minutes * 60.0
        index = min(range(len(times)), key=lambda i: abs(times[i] - target))
        if abs(times[index] - target) > 5.0:
            continue
        checkpoints[f"{minutes}min"] = {
            "elapsed_s": times[index],
            "h_m": round(horizontal[index], 4),
            "up_m": round(vertical[index], 4),
            "abs_up_m": round(abs(vertical[index]), 4),
            "east_m": round(enu[index][0], 4),
            "north_m": round(enu[index][1], 4),
        }
    conv_h = convergence_seconds(times, [h < HORIZONTAL_THRESHOLD_M for h in horizontal])
    conv_v = convergence_seconds(times, [abs(u) < VERTICAL_THRESHOLD_M for u in vertical])
    return {
        "pos": str(path),
        "epochs": len(rows),
        "span_s": times[-1],
        "checkpoints": checkpoints,
        "final": {"h_m": round(horizontal[-1], 4), "up_m": round(vertical[-1], 4)},
        "convergence_h_below_0p20_min": None if conv_h is None else round(conv_h / 60.0, 2),
        "convergence_abs_up_below_0p40_min": None if conv_v is None else round(conv_v / 60.0, 2),
    }


def command_score(args: argparse.Namespace) -> int:
    reference = [float(value) for value in args.reference_ecef]
    runs = {}
    for item in args.run:
        label, _, path = item.partition("=")
        if not label or not path:
            raise SystemExit(f"--run expects LABEL=PATH, got {item!r}")
        runs[label] = score_run(Path(path), reference)
    summary = {
        "reference_ecef_m": reference,
        "thresholds": {"horizontal_m": HORIZONTAL_THRESHOLD_M, "vertical_m": VERTICAL_THRESHOLD_M},
        "runs": runs,
    }
    output = Path(args.summary_json)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    for label, run in runs.items():
        cps = "  ".join(f"{key}: H {cp['h_m']:.3f} U {cp['up_m']:+.3f}" for key, cp in run["checkpoints"].items())
        print(f"{label}: {cps}  | conv H<0.20 {run['convergence_h_below_0p20_min']} min, "
              f"|U|<0.40 {run['convergence_abs_up_below_0p40_min']} min")
    return 0


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="command", required=True)
    score = sub.add_parser("score", help="Score .pos files against a static reference coordinate.")
    score.add_argument("--reference-ecef", nargs=3, required=True, metavar=("X", "Y", "Z"))
    score.add_argument("--run", action="append", required=True, metavar="LABEL=POS")
    score.add_argument("--summary-json", required=True)
    score.set_defaults(func=command_score)
    args = parser.parse_args(argv)
    return args.func(args)


if __name__ == "__main__":
    raise SystemExit(main())
