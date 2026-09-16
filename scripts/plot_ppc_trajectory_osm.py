#!/usr/bin/env python3
"""OSM basemap overlay for PPC RTK-only vs tightly-coupled GNSS/IMU.

Downloads OpenStreetMap raster tiles for the trajectory bounding box, then
overlays the reference, RTK-only and GNSS/IMU-fusion latitude/longitude
tracks. Produces a single tweet-friendly map figure. Tile usage respects the
OSM tile policy (a descriptive User-Agent and a small area at one zoom).

Attribution: (c) OpenStreetMap contributors.
"""

from __future__ import annotations

import argparse
import io
import math
import sys
import urllib.request
from pathlib import Path

import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

SCRIPTS_DIR = Path(__file__).resolve().parent
ROOT_DIR = SCRIPTS_DIR.parent
for path in (SCRIPTS_DIR, ROOT_DIR / "apps" / "commands",
             ROOT_DIR / "apps" / "commands" / "benchmarks"):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

import generate_driving_comparison as comparison  # noqa: E402

RTK_COLOR = "#1f77b4"
FUSION_COLOR = "#1a7f37"
REF_COLOR = "#555555"
TILE_URL = "https://tile.openstreetmap.org/{z}/{x}/{y}.png"
USER_AGENT = "gnssplusplus-library/1.0 (PPC research figure)"


def lonlat_to_world(lat: float, lon: float, zoom: int) -> tuple[float, float]:
    n = 2.0**zoom
    x = (lon + 180.0) / 360.0 * n * 256.0
    y = (1.0 - math.asinh(math.tan(math.radians(lat))) / math.pi) / 2.0 * n * 256.0
    return x, y


def choose_zoom(lat_min, lat_max, lon_min, lon_max, max_px=2800) -> int:
    for zoom in range(18, 8, -1):
        x0, y0 = lonlat_to_world(lat_max, lon_min, zoom)
        x1, y1 = lonlat_to_world(lat_min, lon_max, zoom)
        if (x1 - x0) <= max_px and (y1 - y0) <= max_px:
            return zoom
    return 9


def build_basemap(lat_min, lat_max, lon_min, lon_max, zoom):
    from PIL import Image

    x0f, y0f = lonlat_to_world(lat_max, lon_min, zoom)
    x1f, y1f = lonlat_to_world(lat_min, lon_max, zoom)
    tx0, ty0 = int(math.floor(x0f / 256.0)), int(math.floor(y0f / 256.0))
    tx1, ty1 = int(math.floor(x1f / 256.0)), int(math.floor(y1f / 256.0))
    width, height = (tx1 - tx0 + 1) * 256, (ty1 - ty0 + 1) * 256
    canvas = Image.new("RGB", (width, height), (235, 235, 235))
    for tx in range(tx0, tx1 + 1):
        for ty in range(ty0, ty1 + 1):
            req = urllib.request.Request(
                TILE_URL.format(z=zoom, x=tx, y=ty), headers={"User-Agent": USER_AGENT}
            )
            with urllib.request.urlopen(req, timeout=25) as resp:
                tile = Image.open(io.BytesIO(resp.read())).convert("RGB")
            canvas.paste(tile, ((tx - tx0) * 256, (ty - ty0) * 256))
    return canvas, zoom, tx0 * 256.0, ty0 * 256.0


def to_px(lat, lon, zoom):
    xs, ys = [], []
    for la, lo in zip(lat, lon):
        x, y = lonlat_to_world(la, lo, zoom)
        xs.append(x)
        ys.append(y)
    return np.asarray(xs), np.asarray(ys)


def parse_args():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--reference", type=Path, required=True)
    p.add_argument("--rtk-pos", type=Path, required=True)
    p.add_argument("--fusion-pos", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--title", default="PPC Tokyo run1 on OpenStreetMap")
    p.add_argument("--rtk-label", default="RTK only")
    p.add_argument("--fusion-label", default="RTK + GNSS/IMU (tight, robust)")
    p.add_argument("--zoom", type=int, default=0)
    p.add_argument("--inset-zoom", type=int, default=18)
    p.add_argument("--no-inset", action="store_true",
                   help="Draw the overview only")
    p.add_argument("--dpi", type=int, default=170)
    return p.parse_args()


def find_worst_rtk_epoch(reference, rtk):
    matched = comparison.match_to_reference(rtk, reference, 0.25)
    if not matched:
        return None
    worst = max(matched, key=lambda epoch: epoch.horiz_error_m)
    by_tow = {epoch.tow: epoch for epoch in rtk}
    return by_tow.get(worst.tow), worst.horiz_error_m


def plot_tracks(ax, epochs_list, zoom):
    for epochs, color, label, lw in epochs_list:
        xs, ys = to_px([e.lat_deg for e in epochs],
                       [e.lon_deg for e in epochs], zoom)
        ax.plot(xs, ys, color=color, lw=lw, alpha=0.9, label=label, zorder=3)


def main() -> int:
    args = parse_args()
    reference = comparison.read_reference_csv(args.reference)
    rtk = comparison.read_libgnss_pos(args.rtk_pos)
    fusion = comparison.read_libgnss_pos(args.fusion_pos)

    worst = None if args.no_inset else find_worst_rtk_epoch(reference, rtk)

    lat = np.array([e.lat_deg for e in reference] + [e.lat_deg for e in rtk]
                   + [e.lat_deg for e in fusion])
    lon = np.array([e.lon_deg for e in reference] + [e.lon_deg for e in rtk]
                   + [e.lon_deg for e in fusion])
    lat_min, lat_max = lat.min(), lat.max()
    lon_min, lon_max = lon.min(), lon.max()
    pad_lat = max((lat_max - lat_min) * 0.06, 1e-4)
    pad_lon = max((lon_max - lon_min) * 0.06, 1e-4)
    lat_min, lat_max = lat_min - pad_lat, lat_max + pad_lat
    lon_min, lon_max = lon_min - pad_lon, lon_max + pad_lon

    zoom = args.zoom or choose_zoom(lat_min, lat_max, lon_min, lon_max)
    overview, _, origin_x, origin_y = build_basemap(
        lat_min, lat_max, lon_min, lon_max, zoom)
    width, height = overview.size

    tracks = (
        (reference, REF_COLOR, "Reference", 4.0),
        (rtk, RTK_COLOR, args.rtk_label, 1.8),
        (fusion, FUSION_COLOR, args.fusion_label, 1.8),
    )

    if worst is None:
        fig, ax_over = plt.subplots(figsize=(14, 11))
        ax_zoom = None
    else:
        fig, (ax_over, ax_zoom) = plt.subplots(
            1, 2, figsize=(20, 10), gridspec_kw={"width_ratios": [3, 2]})

    ax_over.imshow(np.asarray(overview),
                   extent=[origin_x, origin_x + width, origin_y + height, origin_y])
    plot_tracks(ax_over, tracks, zoom)
    ax_over.set_xlim(origin_x, origin_x + width)
    ax_over.set_ylim(origin_y + height, origin_y)
    ax_over.set_aspect("equal")
    ax_over.set_xticks([])
    ax_over.set_yticks([])
    ax_over.set_title(args.title, fontsize=18, fontweight="bold")
    ax_over.legend(fontsize=13, loc="upper right", framealpha=0.9)

    if ax_zoom is not None:
        worst_epoch, worst_err = worst
        span_lat = 0.0018 * 2.0 / (2 ** (args.inset_zoom - 16))
        span_lon = span_lat / math.cos(math.radians(worst_epoch.lat_deg))
        z_lat_min = worst_epoch.lat_deg - span_lat
        z_lat_max = worst_epoch.lat_deg + span_lat
        z_lon_min = worst_epoch.lon_deg - span_lon
        z_lon_max = worst_epoch.lon_deg + span_lon
        inset, izoom, ox, oy = build_basemap(
            z_lat_min, z_lat_max, z_lon_min, z_lon_max, args.inset_zoom)
        iw, ih = inset.size
        ax_zoom.imshow(np.asarray(inset),
                       extent=[ox, ox + iw, oy + ih, oy])
        plot_tracks(ax_zoom, tracks, izoom)
        wsk, wsy = to_px([worst_epoch.lat_deg], [worst_epoch.lon_deg], izoom)
        ax_zoom.scatter(wsk, wsy, s=130, facecolors="none", edgecolors="black",
                        linewidths=2.0, zorder=4)
        ax_zoom.set_xlim(ox, ox + iw)
        ax_zoom.set_ylim(oy + ih, oy)
        ax_zoom.set_aspect("equal")
        ax_zoom.set_xticks([])
        ax_zoom.set_yticks([])
        ax_zoom.set_title(f"Zoom at worst {args.rtk_label} epoch"
                          f" ({worst_err:.0f} m)", fontsize=15, fontweight="bold")

    fig.text(0.995, 0.01, "(c) OpenStreetMap contributors",
             ha="right", va="bottom", fontsize=10,
             bbox=dict(boxstyle="round", fc="white", ec="#999999", alpha=0.85))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.tight_layout(rect=(0, 0.02, 1, 1))
    fig.savefig(args.output, dpi=args.dpi)
    print(f"wrote {args.output} (overview zoom {zoom})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
