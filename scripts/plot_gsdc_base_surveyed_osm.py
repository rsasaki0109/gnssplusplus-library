#!/usr/bin/env python3
"""GSDC base-surveyed dev routes on OpenStreetMap.

2x2 OpenStreetMap basemaps, one per dev route, overlaying the ground truth
and the base-surveyed native FGO trajectory.

Predictions/truth are submission CSVs with UnixTimeMillis, LatitudeDegrees,
LongitudeDegrees. Tile usage respects the OSM tile policy (descriptive
User-Agent, one zoom per route). Attribution: (c) OpenStreetMap contributors.
"""
from __future__ import annotations

import argparse
import io
import math
import urllib.request
from pathlib import Path

import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

TILE_URL = "https://tile.openstreetmap.org/{z}/{x}/{y}.png"
USER_AGENT = "gnssplusplus-library/1.0 (GSDC research figure)"
TRUTH_COLOR = "#555555"
FIX_COLOR = "#1a7f37"


def lonlat_to_world(lat, lon, zoom):
    n = 2.0 ** zoom
    x = (lon + 180.0) / 360.0 * n * 256.0
    y = (1.0 - math.asinh(math.tan(math.radians(lat))) / math.pi) / 2.0 * n * 256.0
    return x, y


def choose_zoom(lat_min, lat_max, lon_min, lon_max, max_px=2200):
    for zoom in range(17, 8, -1):
        x0, y0 = lonlat_to_world(lat_max, lon_min, zoom)
        x1, y1 = lonlat_to_world(lat_min, lon_max, zoom)
        if (x1 - x0) <= max_px and (y1 - y0) <= max_px:
            return zoom
    return 10


def build_basemap(lat_min, lat_max, lon_min, lon_max, zoom):
    from PIL import Image

    x0f, y0f = lonlat_to_world(lat_max, lon_min, zoom)
    x1f, y1f = lonlat_to_world(lat_min, lon_max, zoom)
    tx0, ty0 = int(math.floor(x0f / 256.0)), int(math.floor(y0f / 256.0))
    tx1, ty1 = int(math.floor(x1f / 256.0)), int(math.floor(y1f / 256.0))
    canvas = Image.new("RGB", ((tx1 - tx0 + 1) * 256, (ty1 - ty0 + 1) * 256), (235, 235, 235))
    for tx in range(tx0, tx1 + 1):
        for ty in range(ty0, ty1 + 1):
            req = urllib.request.Request(
                TILE_URL.format(z=zoom, x=tx, y=ty), headers={"User-Agent": USER_AGENT})
            with urllib.request.urlopen(req, timeout=25) as resp:
                tile = Image.open(io.BytesIO(resp.read())).convert("RGB")
            canvas.paste(tile, ((tx - tx0) * 256, (ty - ty0) * 256))
    return canvas, zoom, tx0 * 256.0, ty0 * 256.0


def read_latlon(path):
    import csv
    lat, lon = [], []
    with open(path, newline="") as fh:
        for row in csv.DictReader(fh):
            if not row.get("LatitudeDegrees"):
                continue
            lat.append(float(row["LatitudeDegrees"]))
            lon.append(float(row["LongitudeDegrees"]))
    return np.array(lat), np.array(lon)


def to_px(lat, lon, zoom):
    xs, ys = [], []
    for la, lo in zip(lat, lon):
        x, y = lonlat_to_world(la, lo, zoom)
        xs.append(x)
        ys.append(y)
    return np.asarray(xs), np.asarray(ys)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--routes", nargs="+", required=True, help="name:pred.csv:truth.csv")
    ap.add_argument("--output", type=Path, required=True)
    ap.add_argument("--dpi", type=int, default=150)
    args = ap.parse_args()

    fig, axes = plt.subplots(2, 2, figsize=(16, 15))
    for ax, spec in zip(axes.ravel(), args.routes):
        name, pred_path, truth_path = spec.split(":")
        tlat, tlon = read_latlon(truth_path)
        plat, plon = read_latlon(pred_path)
        all_lat = np.concatenate([tlat, plat])
        all_lon = np.concatenate([tlon, plon])
        pad = 0.05
        dlat = max((all_lat.max() - all_lat.min()) * pad, 1e-4)
        dlon = max((all_lon.max() - all_lon.min()) * pad, 1e-4)
        zoom = choose_zoom(all_lat.min() - dlat, all_lat.max() + dlat,
                           all_lon.min() - dlon, all_lon.max() + dlon)
        base, _, ox, oy = build_basemap(all_lat.min() - dlat, all_lat.max() + dlat,
                                        all_lon.min() - dlon, all_lon.max() + dlon, zoom)
        w, h = base.size
        ax.imshow(np.asarray(base), extent=[ox, ox + w, oy + h, oy])
        for lat, lon, color, lw, label in (
            (tlat, tlon, TRUTH_COLOR, 4.0, "Ground truth"),
            (plat, plon, FIX_COLOR, 1.8, "base-surveyed"),
        ):
            xs, ys = to_px(lat, lon, zoom)
            ax.plot(xs, ys, color=color, lw=lw, alpha=0.9, label=label, zorder=3)
        ax.set_xlim(ox, ox + w)
        ax.set_ylim(oy + h, oy)
        ax.set_aspect("equal")
        ax.set_xticks([])
        ax.set_yticks([])
        ax.set_title(name, fontsize=16, fontweight="bold")
        ax.legend(fontsize=11, loc="upper right", framealpha=0.9)
    fig.text(0.995, 0.01, "(c) OpenStreetMap contributors", ha="right", va="bottom",
             fontsize=10, bbox=dict(boxstyle="round", fc="white", ec="#999999", alpha=0.85))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.tight_layout(rect=(0, 0.015, 1, 1))
    fig.savefig(args.output, dpi=args.dpi)
    print(f"wrote {args.output}")


if __name__ == "__main__":
    raise SystemExit(main())
