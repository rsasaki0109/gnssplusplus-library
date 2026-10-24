"""Prepare synchronized measured trajectories for the PLATEAU/OSM renderer."""

import argparse
import hashlib
import json
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from generate_driving_comparison import (
    read_reference_csv,
    read_libgnss_pos,
    read_rtklib_pos,
    STATUS_MAPS,
)

p = argparse.ArgumentParser(description=__doc__)
p.add_argument("--reference-csv", type=Path, required=True)
p.add_argument("--lib-pos", type=Path, required=True)
p.add_argument("--rtklib-pos", type=Path, required=True)
p.add_argument("--start-tow", type=float, required=True)
p.add_argument("--duration", type=float, default=30)
template = json.loads(
    (Path(__file__).resolve().parents[2] / "docs/odaiba_hero_trace.json").read_text()
)
p.add_argument("--tileset-url", default=template["tilesetUrl"])
p.add_argument("--output", type=Path, required=True)
a = p.parse_args()
if not a.duration > 0:
    p.error("duration must be positive")
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
start = week * 604800 + a.start_tow
end = start + a.duration


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
            result.append(values + [status])
    result.sort(key=lambda r: r[0])
    if len(result) < 2:
        p.error("selected interval has too few samples")
    if any(x[0] >= y[0] for x, y in zip(result, result[1:])):
        p.error("duplicate or unsorted timestamps")
    return result


ref = pack(reference, None)
if ref[0][0] > start or ref[-1][0] < end + 1.5:
    p.error("reference must cover camera look-ahead")
result = {
    "groundHeight": template["groundHeight"],
    "basemapBounds": template["basemapBounds"],
    "basemapTiles": template["basemapTiles"],
    "reference": ref,
    "rtklib": pack(rtk, "RTKLIB"),
    "libgnss": pack(lib, "libgnss++"),
    "start": start,
    "end": end,
    "tilesetUrl": a.tileset_url,
    "provenance": {
        "note": "Raw measured horizontal coordinates; shared reference camera and flat schematic 42 m ellipsoidal ground plane. Paths drawn at 42.4–42.45 m, independent of estimated height. Recorded forward paths are shown during offline replay. No vertical accuracy claim.",
        "inputs": {
            str(path): hashlib.sha256(path.read_bytes()).hexdigest()
            for path in [a.reference_csv, a.lib_pos, a.rtklib_pos]
        },
    },
}
a.output.write_text(json.dumps(result, separators=(",", ":")))
print(
    f"Saved {a.output}: "
    + str({k: len(result[k]) for k in ["reference", "rtklib", "libgnss"]})
)
