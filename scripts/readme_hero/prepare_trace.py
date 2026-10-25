"""Prepare a measured three-track replay, with an explicit showcase selector."""

import argparse
import csv
import hashlib
import json
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from generate_driving_comparison import (
    read_reference_csv,
    read_libgnss_pos,
    read_rtklib_pos,
    STATUS_MAPS,
    ecef_to_enu,
)

p = argparse.ArgumentParser(description=__doc__)
p.add_argument("--reference-csv", type=Path, required=True)
p.add_argument("--lib-pos", type=Path, required=True)
p.add_argument("--rtklib-pos", type=Path, required=True)
choice = p.add_mutually_exclusive_group(required=True)
choice.add_argument("--start-tow", type=float)
choice.add_argument(
    "--select-showcase",
    action="store_true",
    help="Select a favorable illustrative window; not an unbiased benchmark",
)
p.add_argument("--duration", type=float, default=20)
p.add_argument(
    "--lib-recipe",
    default="",
    help="Non-default gnss_solve flags used for --lib-pos; shown in the legend",
)
template = json.loads(
    (Path(__file__).resolve().parents[2] / "docs/odaiba_hero_trace.json").read_text()
)
p.add_argument("--tileset-url", default=template["tilesetUrl"])
p.add_argument("--output", type=Path, required=True)
a = p.parse_args()
if not math.isfinite(a.duration) or a.duration <= 0:
    p.error("duration must be finite and positive")
reference = read_reference_csv(a.reference_csv)
lib = read_libgnss_pos(a.lib_pos)
rtk = read_rtklib_pos(a.rtklib_pos)
if not all([reference, lib, rtk]):
    p.error("all inputs must contain epochs")
weeks = (
    set(r.week for r in reference) & set(r.week for r in lib) & set(r.week for r in rtk)
)
if len(weeks) != 1:
    p.error("inputs must share exactly one GPS week")
week = weeks.pop()


def key(r):
    return (r.week, round(r.tow, 3))


refs = {key(r): r for r in reference}
libs = {key(r): r for r in lib}
rtks = {key(r): r for r in rtk}
with a.reference_csv.open(newline="") as handle:
    reader = csv.reader(handle)
    next(reader)
    headings = {(int(float(r[1])), round(float(r[0]), 3)): float(r[10]) for r in reader}


def horizontal_error(r):
    truth = refs.get(key(r))
    if truth is None:
        return None
    return float(
        np.linalg.norm(
            ecef_to_enu(r.ecef - truth.ecef, truth.lat_deg, truth.lon_deg)[:2]
        )
    )


common = []
for k in sorted(refs.keys() & libs.keys() & rtks.keys()):
    if k[0] == week:
        common.append([k[1], horizontal_error(libs[k]), horizontal_error(rtks[k])])
common = np.asarray(common)
if not len(common):
    p.error("no exact common reference/solver epochs")
selection = {
    "mode": "manual",
    "note": "Illustrative interval; not a full-run performance claim",
}
if a.select_showcase:
    candidates = []
    expected = round(a.duration * 10) + 1  # published 10 Hz Odaiba reference
    for grid_tow in np.arange(
        math.ceil(common[0, 0]), common[-1, 0] - a.duration - 2, 2
    ):
        start_index = np.searchsorted(common[:, 0], grid_tow)
        if start_index == len(common) or common[start_index, 0] - grid_tow > 1.0:
            continue
        start_tow = float(common[start_index, 0])
        pairs = common[
            (common[:, 0] >= start_tow) & (common[:, 0] <= start_tow + a.duration)
        ]
        if (
            len(pairs) < math.ceil(expected * 0.8)
            or np.max(np.diff(pairs[:, 0])) > 2.501
        ):
            continue
        rr = [
            r
            for r in reference
            if r.week == week and start_tow <= r.tow <= start_tow + a.duration + 2
        ]
        if len(rr) < round((a.duration + 2) * 10) - 5:
            continue
        xyz = np.array([r.ecef for r in rr])
        speed = np.linalg.norm(xyz[15:] - xyz[:-15], axis=1) / 1.5
        if speed.min() < 1 or np.percentile(pairs[:, 1], 95) > 1:
            continue
        candidates.append((float(np.median(pairs[:, 2])), float(start_tow)))
    if not candidates:
        p.error("no interval satisfies the documented showcase criteria")
    _, a.start_tow = sorted(candidates, key=lambda r: (-r[0], r[1]))[0]
    selection = {
        "mode": "favorable_showcase",
        "grid_s": 2,
        "start_alignment": "first exact common epoch within 1 s after each grid point",
        "minimum_exact_common_coverage": 0.8,
        "maximum_common_gap_s": 2.501,
        "minimum_reference_forward_1p5s_speed_mps": 1,
        "maximum_libgnss_common_p95_h_m": 1,
        "ranking": "largest RTKLIB common-epoch median H; earliest window breaks ties",
        "eligible_windows": len(candidates),
        "note": "Deliberately favorable selection to illustrate visible solver differences; not an unbiased benchmark",
    }
start = week * 604800 + a.start_tow
end = start + a.duration
pairs = common[
    (common[:, 0] >= a.start_tow) & (common[:, 0] <= a.start_tow + a.duration)
]
if not len(pairs):
    p.error("selected window has no exact common epochs")
metrics = {"common_epochs": len(pairs)}
for col, name in [(1, "libgnss"), (2, "rtklib")]:
    metrics[name] = {
        "median_h_m": float(np.median(pairs[:, col])),
        "p95_h_m": float(np.percentile(pairs[:, col], 95)),
    }


def pack(rows, solver):
    result = []
    for r in rows:
        t = r.week * 604800 + r.tow
        if start - 12 <= t <= end + 2:
            values = [t, r.lon_deg, r.lat_deg, r.height_m]
            if not all(math.isfinite(v) for v in values):
                p.error("nonfinite position/time")
            if not (-180 <= r.lon_deg <= 180 and -90 <= r.lat_deg <= 90):
                p.error("invalid WGS84 degrees")
            status = (
                "REFERENCE"
                if solver is None
                else STATUS_MAPS[solver].get(r.status, f"Q{r.status}")
            )
            extra = headings[key(r)] if solver is None else horizontal_error(r)
            result.append(values + [status, extra])
    result.sort(key=lambda r: r[0])
    if len(result) < 2 or any(x[0] >= y[0] for x, y in zip(result, result[1:])):
        p.error("too few samples or duplicate timestamps")
    return result


ref = pack(reference, None)
if ref[0][0] > start or ref[-1][0] < end:
    p.error("reference does not cover selected interval")
result = {
    "schema": "odaiba_three_track_replay.v2",
    "groundHeight": template["groundHeight"],
    "basemapBounds": template["basemapBounds"],
    "basemapTiles": template["basemapTiles"],
    "reference": ref,
    "rtklib": pack(rtk, "RTKLIB"),
    "libgnss": pack(lib, "libgnss++"),
    "start": start,
    "end": end,
    "tilesetUrl": a.tileset_url,
    "selection": selection,
    "metrics": metrics,
    "provenance": {
        "note": "Unmodified horizontal positions. One scene and one camera driven only by measured Ground Truth position/heading. Flat 42m ellipsoidal ground; all paths at 42.45m. Recorded future paths visible; horizontal visualization only.",
        "row_schema": {
            "reference": [
                "absolute_gps_s",
                "longitude_deg",
                "latitude_deg",
                "height_m",
                "REFERENCE",
                "heading_deg",
            ],
            "solvers": [
                "absolute_gps_s",
                "longitude_deg",
                "latitude_deg",
                "height_m",
                "status",
                "reference_horizontal_error_m_or_null",
            ],
        },
        "inputs": {
            str(path): hashlib.sha256(path.read_bytes()).hexdigest()
            for path in [a.reference_csv, a.lib_pos, a.rtklib_pos]
        },
    },
}
if a.lib_recipe:
    result["libgnssRecipe"] = a.lib_recipe
a.output.parent.mkdir(parents=True, exist_ok=True)
a.output.write_text(json.dumps(result, separators=(",", ":")) + "\n")
print(f"Saved {a.output}: TOW {a.start_tow}–{a.start_tow+a.duration}; {metrics}")
