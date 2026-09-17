#!/usr/bin/env python3
"""PPC figure: GNSS-only FGO vs GNSS/IMU FGO.

Compares two factor-graph-optimization solutions against the PPC reference:
  * GNSS-only FGO  -> ``gnss_fgo --obs rover.obs --nav base.nav --out x.pos``
  * GNSS/IMU  FGO  -> ``gnss_fgo_imu_no_base --obs ... --imu imu.csv ...``
    (a submission CSV ``phone,UnixTimeMillis,LatitudeDegrees,LongitudeDegrees``)

Renders an OpenStreetMap overlay plus a horizontal-error panel, and prints
P50 / P95 / max / RMS. Tile usage respects the OSM tile policy (descriptive
User-Agent, one zoom). Attribution: (c) OpenStreetMap contributors.
"""

from __future__ import annotations

import argparse
import io
import json
import math
import sys
import urllib.request
from pathlib import Path

import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.lines import Line2D  # noqa: E402

SCRIPTS_DIR = Path(__file__).resolve().parent
ROOT_DIR = SCRIPTS_DIR.parent
if str(SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_DIR))

import generate_driving_comparison as comparison  # noqa: E402

GNSS_COLOR = "#1f77b4"
FUSION_COLOR = "#1a7f37"
EXTRA_COLOR = "#d62728"
REF_COLOR = "#555555"
ARM_LINESTYLES = ["-", "--", ":"]

STATUS_COLORS = {4: "#1a7f37", 3: "#ff7f0e", 2: "#9467bd", 1: "#d62728",
                 0: "#999999"}
STATUS_NAMES = {4: "FIX", 3: "FLOAT", 2: "DGPS", 1: "SPP", 0: "status n/a"}


def plot_status_track(ax, xy, statuses, linestyle="-", lw=1.5, alpha=0.9,
                      zorder=3):
    """Draw a trajectory with line segments coloured by per-epoch status."""
    n = len(xy)
    if n == 0:
        return
    i = 0
    while i < n:
        j = i
        while j + 1 < n and statuses[j + 1] == statuses[i]:
            j += 1
        end = min(j + 1, n - 1)
        seg = xy[i:end + 1]
        if seg.shape[0] >= 2:
            ax.plot(seg[:, 0], seg[:, 1],
                    color=STATUS_COLORS.get(statuses[i], "#999999"),
                    ls=linestyle, lw=lw, alpha=alpha, zorder=zorder)
        i = j + 1


def status_legend_handles(epochs_list, arm_handles):
    present = {e.status for epochs in epochs_list for e in epochs}
    handles = list(arm_handles)
    for code in (4, 3, 2, 1, 0):
        if code in present:
            handles.append(Line2D([0], [0], color=STATUS_COLORS[code], lw=2.4,
                                  label=STATUS_NAMES[code]))
    return handles
TILE_URL = "https://tile.openstreetmap.org/{z}/{x}/{y}.png"
USER_AGENT = "gnssplusplus-library/1.0 (PPC research figure)"
GPS_TO_UNIX_OFFSET_S = 315964800.0  # 1980-01-06 - 1970-01-01
LEAP_SECONDS = 18  # GPST - UTC since 2017-01-01


def unix_millis_to_week_tow(unix_millis: float) -> tuple[int, float]:
    gps_s = unix_millis / 1000.0 + LEAP_SECONDS - GPS_TO_UNIX_OFFSET_S
    week = int(gps_s // 604800.0)
    return week, gps_s - week * 604800.0


def read_submission_lat_lon(path: Path) -> list[comparison.SolutionEpoch]:
    rows: list[comparison.SolutionEpoch] = []
    with path.open() as handle:
        header = handle.readline()
        if "UnixTimeMillis" not in header:
            raise ValueError(f"{path}: unexpected header {header!r}")
        for line in handle:
            parts = line.strip().split(",")
            if len(parts) < 4:
                continue
            unix_millis = float(parts[1])
            lat = float(parts[2])
            lon = float(parts[3])
            week, tow = unix_millis_to_week_tow(unix_millis)
            ecef = comparison.llh_to_ecef(lat, lon, 0.0)
            rows.append(comparison.SolutionEpoch(
                week, tow, lat, lon, 0.0, ecef, 0, 0))
    return rows


def horizontal_stats(solution, reference, tolerance_s: float) -> dict:
    matched = comparison.match_to_reference(solution, reference, tolerance_s)
    if not matched:
        return {"matched": 0}
    h = np.array([epoch.horiz_error_m for epoch in matched])
    acc_mean, acc_p95 = smoothness_stats(solution)
    return {
        "matched": len(matched),
        "p50_m": float(np.percentile(h, 50)),
        "p95_m": float(np.percentile(h, 95)),
        "max_m": float(h.max()),
        "rms_m": float(np.sqrt(np.mean(h * h))),
        "official_pct": float((np.percentile(h, 50) + np.percentile(h, 95)) / 2.0),
        "acc_mean_mps2": acc_mean,
        "acc_p95_mps2": acc_p95,
    }


def smoothness_stats(solution) -> tuple:
    """Mean/95th horizontal acceleration magnitude from position second
    differences at the native epoch rate (in-estimator smoothness)."""
    acc = []
    for i in range(1, len(solution) - 1):
        dt1 = solution[i].tow - solution[i - 1].tow
        dt2 = solution[i + 1].tow - solution[i].tow
        if not (0.15 <= dt1 <= 0.25 and 0.15 <= dt2 <= 0.25):
            continue
        a = solution[i - 1].ecef
        b = solution[i].ecef
        d = solution[i + 1].ecef
        second = (d - 2.0 * b + a) / (dt1 * dt2)
        acc.append(math.hypot(second[0], second[1]))
    if not acc:
        return float("nan"), float("nan")
    arr = np.array(acc)
    return float(arr.mean()), float(np.percentile(arr, 95))


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
    p.add_argument("--gnss-pos", type=Path, required=True,
                   help="GNSS-only FGO .pos (gnss_fgo)")
    p.add_argument("--imu-csv", type=Path, default=None,
                   help="GNSS/IMU FGO submission CSV (gnss_fgo_imu_no_base)")
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--summary-json", type=Path, default=None)
    p.add_argument("--title", default="PPC Tokyo run1: GNSS-only vs GNSS/IMU FGO")
    p.add_argument("--gnss-label", default="GNSS-only FGO")
    p.add_argument("--imu-label", default="GNSS/IMU FGO")
    p.add_argument("--extra-pos", type=Path, default=None,
                   help="Optional third arm .pos (e.g. carrier-phase + IMU FGO)")
    p.add_argument("--extra-label", default="Carrier + IMU FGO")
    p.add_argument("--tolerance-s", type=float, default=0.25)
    p.add_argument("--no-osm", action="store_true",
                   help="Skip the OSM basemap (offline plot)")
    p.add_argument("--no-zoom", action="store_true",
                   help="Disable the zoomed inset at the worst combined epoch")
    p.add_argument("--zoom-span-m", type=float, default=120.0)
    p.add_argument("--summary-only", action="store_true",
                   help="Print stats only, draw nothing")
    p.add_argument("--dpi", type=int, default=170)
    return p.parse_args()


def main() -> int:
    args = parse_args()
    reference = comparison.read_reference_csv(args.reference)
    gnss = comparison.read_libgnss_pos(args.gnss_pos)
    imu = read_submission_lat_lon(args.imu_csv) if args.imu_csv else None
    extra = comparison.read_libgnss_pos(args.extra_pos) if args.extra_pos else None

    arms = [
        {"epochs": gnss, "color": GNSS_COLOR, "label": args.gnss_label},
    ]
    if imu is not None:
        arms.append({"epochs": imu, "color": FUSION_COLOR, "label": args.imu_label})
    if extra is not None:
        arms.append({"epochs": extra, "color": EXTRA_COLOR, "label": args.extra_label})
    for arm in arms:
        arm["stats"] = horizontal_stats(arm["epochs"], reference, args.tolerance_s)
        arm["matched"] = comparison.match_to_reference(
            arm["epochs"], reference, args.tolerance_s)

    summary = {arm["label"]: arm["stats"] for arm in arms}
    print(json.dumps(summary, indent=2))
    if args.summary_json:
        args.summary_json.parent.mkdir(parents=True, exist_ok=True)
        args.summary_json.write_text(json.dumps(summary, indent=2))
    if args.summary_only:
        return 0

    fig = plt.figure(figsize=(20, 11))
    grid = fig.add_gridspec(2, 2, width_ratios=[3, 2], hspace=0.28, wspace=0.12)
    ax_map = fig.add_subplot(grid[:, 0])
    ax_err = fig.add_subplot(grid[0, 1])
    ax_cdf = fig.add_subplot(grid[1, 1])

    all_epochs = [reference] + [arm["epochs"] for arm in arms]
    if args.no_osm:
        ax_map.plot([e.lon_deg for e in reference], [e.lat_deg for e in reference],
                    color=REF_COLOR, lw=2.0, alpha=0.9, label="Reference")
        for idx, arm in enumerate(arms):
            plot_status_track(
                ax_map,
                np.column_stack([[e.lon_deg for e in arm["epochs"]],
                                 [e.lat_deg for e in arm["epochs"]]]),
                [e.status for e in arm["epochs"]],
                linestyle=ARM_LINESTYLES[idx % len(ARM_LINESTYLES)], lw=1.5)
    else:
        lat = np.array([e.lat_deg for epochs in all_epochs for e in epochs])
        lon = np.array([e.lon_deg for epochs in all_epochs for e in epochs])
        pad = 0.06
        lat_min, lat_max = lat.min(), lat.max()
        lon_min, lon_max = lon.min(), lon.max()
        dlat = max((lat_max - lat_min) * pad, 1e-4)
        dlon = max((lon_max - lon_min) * pad, 1e-4)
        zoom = choose_zoom(lat_min - dlat, lat_max + dlat, lon_min - dlon, lon_max + dlon)
        basemap, _, ox, oy = build_basemap(
            lat_min - dlat, lat_max + dlat, lon_min - dlon, lon_max + dlon, zoom)
        w, h = basemap.size
        ax_map.imshow(np.asarray(basemap), extent=[ox, ox + w, oy + h, oy])
        xs, ys = to_px([e.lat_deg for e in reference], [e.lon_deg for e in reference], zoom)
        ax_map.plot(xs, ys, color=REF_COLOR, lw=4.0, alpha=0.9, label="Reference", zorder=3)
        for idx, arm in enumerate(arms):
            xs, ys = to_px([e.lat_deg for e in arm["epochs"]],
                           [e.lon_deg for e in arm["epochs"]], zoom)
            plot_status_track(ax_map, np.column_stack([xs, ys]),
                              [e.status for e in arm["epochs"]],
                              linestyle=ARM_LINESTYLES[idx % len(ARM_LINESTYLES)],
                              lw=1.8)
        ax_map.set_xlim(ox, ox + w)
        ax_map.set_ylim(oy + h, oy)
        fig.text(0.995, 0.01, "(c) OpenStreetMap contributors", ha="right", va="bottom",
                 fontsize=10, bbox=dict(boxstyle="round", fc="white", ec="#999999", alpha=0.85))

        if not args.no_zoom:
            worst_err = -1.0
            worst_tow = None
            for arm in arms:
                for e in arm["matched"]:
                    if e.horiz_error_m > worst_err:
                        worst_err = e.horiz_error_m
                        worst_tow = e.tow
            center = None
            if worst_tow is not None:
                for arm in arms:
                    for e in arm["epochs"]:
                        if abs(e.tow - worst_tow) < 1e-6:
                            center = (e.lat_deg, e.lon_deg)
                            break
                    if center is not None:
                        break
            if center is not None:
                clat, clon = center
                span_m = args.zoom_span_m
                dlat = span_m / 111320.0
                dlon = span_m / (111320.0 * math.cos(math.radians(clat)))
                inset, izoom, ix, iy = build_basemap(
                    clat - dlat, clat + dlat, clon - dlon, clon + dlon, 18)
                iw, ih = inset.size
                axi = ax_map.inset_axes([0.61, 0.03, 0.36, 0.36])
                axi.imshow(np.asarray(inset), extent=[ix, ix + iw, iy + ih, iy])
                rxs, rys = to_px([e.lat_deg for e in reference],
                                 [e.lon_deg for e in reference], izoom)
                axi.plot(rxs, rys, color=REF_COLOR, lw=2.5, alpha=0.9, zorder=3)
                for idx, arm in enumerate(arms):
                    xs, ys = to_px([e.lat_deg for e in arm["epochs"]],
                                   [e.lon_deg for e in arm["epochs"]], izoom)
                    plot_status_track(
                        axi, np.column_stack([xs, ys]),
                        [e.status for e in arm["epochs"]],
                        linestyle=ARM_LINESTYLES[idx % len(ARM_LINESTYLES)],
                        lw=1.6)
                axi.set_xlim(ix, ix + iw)
                axi.set_ylim(iy + ih, iy)
                axi.set_aspect("equal")
                axi.set_xticks([])
                axi.set_yticks([])
                axi.set_title(f"zoom {span_m:.0f} m at worst epoch ({worst_err:.1f} m)",
                              fontsize=9)

    ax_map.set_aspect("equal")
    ax_map.set_xticks([])
    ax_map.set_yticks([])
    ax_map.set_title(args.title, fontsize=18, fontweight="bold")
    arm_handles = [Line2D([0], [0], color=REF_COLOR, lw=3.5, label="Reference")]
    for idx, arm in enumerate(arms):
        arm_handles.append(Line2D(
            [0], [0], color="#333333", lw=1.8,
            ls=ARM_LINESTYLES[idx % len(ARM_LINESTYLES)], label=arm["label"]))
    ax_map.legend(handles=status_legend_handles([a["epochs"] for a in arms],
                                                arm_handles),
                  fontsize=10, loc="upper right", framealpha=0.9)
    table = "\n".join(
        f"{arm['label'][:18]:18s} P50 {arm['stats']['p50_m']:.2f}"
        f"  P95 {arm['stats']['p95_m']:.2f}  max {arm['stats']['max_m']:.1f} m"
        for arm in arms)
    ax_map.text(0.01, 0.01, table, transform=ax_map.transAxes, ha="left", va="bottom",
                fontsize=12, family="monospace",
                bbox=dict(boxstyle="round", fc="white", ec="#999999", alpha=0.9))

    p95_max = 0.0
    for arm in arms:
        matched = arm["matched"]
        err = np.array([e.horiz_error_m for e in matched])
        ax_err.plot([e.tow for e in matched], err, color=arm["color"], lw=0.9,
                    alpha=0.75, label=arm["label"])
        hs = np.sort(err)
        ax_cdf.plot(hs, np.arange(1, hs.size + 1) / hs.size,
                    color=arm["color"], lw=2.0, label=arm["label"])
        p95_max = max(p95_max, float(np.percentile(err, 95)))
    ax_err.set_yscale("log")
    ax_err.set_ylabel("horizontal error (m)")
    ax_err.set_xlabel("GPS TOW (s)")
    ax_err.grid(True, which="both", alpha=0.3)
    ax_err.legend(fontsize=12, loc="upper left", framealpha=0.9)
    ax_err.set_title("Horizontal error vs time", fontsize=14, fontweight="bold")
    ax_cdf.set_xlabel("horizontal error (m)")
    ax_cdf.set_ylabel("fraction of epochs")
    ax_cdf.set_xlim(0.0, p95_max * 1.35)
    ax_cdf.grid(True, alpha=0.3)
    ax_cdf.legend(fontsize=12, loc="lower right", framealpha=0.9)
    ax_cdf.set_title("Horizontal-error CDF", fontsize=14, fontweight="bold")

    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.tight_layout(rect=(0, 0.02, 1, 1))
    fig.savefig(args.output, dpi=args.dpi)
    print(f"wrote {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
