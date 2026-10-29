#!/usr/bin/env python3
"""Truth-free road-height maps for gnss_solve --road-height-map (docs/rtk_road_height_prior.md).

A map is a ``lat_deg,lon_deg,height_m`` CSV of ellipsoidal road (or ground) heights on a 1 m
grid within 30 m of a run's own solution track (any status). The JSON next to it gives the
antenna offset and sigma for --road-height-offset-m / --road-height-sigma-m: the median and
1.4826 * MAD of (FIX height - map height) over the run's FIX epochs on the map. No reference
trajectory is read.

Two sources:

* PLATEAU LOD3 tran 3D Tiles (Tokyo wards). Road surfaces (車道部, 車道交差部) are rasterised;
  a grid node whose surfaces disagree by more than 0.5 m (an overpass) is dropped. Decoding
  the Draco-compressed tiles needs ``pip install DracoPy``.
* GSI DEM5A (z15 ``dem5a_png`` tiles, anywhere in Japan): orthometric height plus a geoid plane
  fitted to GSI geoid-calculation points. A DEM is ground height, so overpasses cannot be
  removed.

    road_height_maps.py plateau-fetch --tileset-url URL [URL ...] --out tiles/
    road_height_maps.py plateau-cache --tiles tiles/ --out road_triangles.npy
    road_height_maps.py plateau-map --triangles road_triangles.npy --solution run.pos --out map.csv
    road_height_maps.py dem-fetch --solution run.pos --out dem/
    road_height_maps.py dem-geoid --solution run.pos --out geoid.txt
    road_height_maps.py dem-map --tiles dem/ --geoid geoid.txt --solution run.pos --out map.csv
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import struct
import sys
import time
import urllib.parse
import urllib.request

import numpy as np

ROOT_DIR = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT_DIR / "scripts"))
from generate_driving_comparison import read_libgnss_pos  # noqa: E402

WGS84_A, WGS84_F = 6378137.0, 1 / 298.257223563
WGS84_E2 = WGS84_F * (2 - WGS84_F)
ROAD_FUNCTIONS = ("車道部", "車道交差部")
CORRIDOR_M = 30.0
MULTILEVEL_SPREAD_M = 0.5
FIX_RADIUS_M = 1.5
DEM_URL = "https://cyberjapandata.gsi.go.jp/xyz/dem5a_png/15/{x}/{y}.png"
GEOID_URL = "https://vldb.gsi.go.jp/sokuchi/surveycalc/geoid/calcgh/cgi/geoidcalc.pl"


def llh_to_ecef(lat_deg, lon_deg, height_m):
    lat, lon = np.radians(lat_deg), np.radians(lon_deg)
    n = WGS84_A / np.sqrt(1 - WGS84_E2 * np.sin(lat) ** 2)
    return np.c_[(n + height_m) * np.cos(lat) * np.cos(lon), (n + height_m) * np.cos(lat) * np.sin(lon),
                 (n * (1 - WGS84_E2) + height_m) * np.sin(lat)]


def ecef_to_llh(ecef):
    p = np.hypot(ecef[:, 0], ecef[:, 1])
    lat = np.arctan2(ecef[:, 2], p * (1 - WGS84_E2))
    height = p * 0.0
    for _ in range(6):
        n = WGS84_A / np.sqrt(1 - WGS84_E2 * np.sin(lat) ** 2)
        height = p / np.cos(lat) - n
        lat = np.arctan2(ecef[:, 2], p * (1 - WGS84_E2 * n / (n + height)))
    return np.degrees(lat), np.degrees(np.arctan2(ecef[:, 1], ecef[:, 0])), height


class Enu:
    """Tangent plane at (lat0, lon0, h0)."""

    def __init__(self, lat0, lon0, h0=0.0):
        self.o = llh_to_ecef(lat0, lon0, h0)[0]
        la, lo = math.radians(lat0), math.radians(lon0)
        self.R = np.array([[-math.sin(lo), math.cos(lo), 0],
                           [-math.sin(la) * math.cos(lo), -math.sin(la) * math.sin(lo), math.cos(la)],
                           [math.cos(la) * math.cos(lo), math.cos(la) * math.sin(lo), math.sin(la)]])

    def __call__(self, xyz):
        return (np.asarray(xyz) - self.o) @ self.R.T

    def to_ecef(self, enu):
        return self.o + np.asarray(enu) @ self.R


def fetch(url: str, path: Path, timeout_s: float = 60.0) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with urllib.request.urlopen(url, timeout=timeout_s) as response:
        path.write_bytes(response.read())


def solution_track(pos: Path):
    """Solution epochs (any status) and their local frame centred on the median position."""
    sols = [s for s in read_libgnss_pos(pos) if np.all(np.isfinite(s.ecef)) and np.linalg.norm(s.ecef) > 6e6]
    lat0, lon0 = np.median([s.lat_deg for s in sols]), np.median([s.lon_deg for s in sols])
    # Drop gross SPP outliers (more than 0.2 deg from the median).
    sols = [s for s in sols if abs(s.lat_deg - lat0) < 0.2 and abs(s.lon_deg - lon0) < 0.2]
    enu = Enu(np.median([s.lat_deg for s in sols]), np.median([s.lon_deg for s in sols]), 0.0)
    return sols, enu, enu(np.array([s.ecef for s in sols]))


def offset_and_sigma(sols, track, map_xy, map_height):
    """Median and 1.4826 * MAD of (FIX height - map height) over FIX epochs within 1.5 m of a node."""
    from scipy.spatial import cKDTree  # noqa: PLC0415

    tree = cKDTree(map_xy)
    residuals = []
    for s, t in zip(sols, track):
        if s.status != 4:
            continue
        distance, index = tree.query(t[:2])
        if distance <= FIX_RADIUS_M:
            residuals.append(s.height_m - map_height[index])
    residuals = np.array(residuals)
    if not len(residuals):
        raise SystemExit("no FIX epoch on the map: offset and sigma are undefined")
    offset = float(np.median(residuals))
    return offset, float(1.4826 * np.median(np.abs(residuals - offset))), len(residuals)


def write_map(out_csv: Path, enu: Enu, local_xyz, info: dict) -> None:
    """local_xyz: east, north, height above the tangent plane at h0 = 0."""
    lat, lon, height = ecef_to_llh(enu.to_ecef(local_xyz))
    write_llh(out_csv, lat, lon, height, info)


def write_llh(out_csv: Path, lat, lon, height, info: dict) -> None:
    with open(out_csv, "w", encoding="utf-8") as handle:
        handle.write("lat_deg,lon_deg,height_m\n")
        for a, b, c in zip(lat, lon, height):
            handle.write(f"{a:.9f},{b:.9f},{c:.4f}\n")
    out_json = out_csv.with_suffix(".json")
    out_json.write_text(json.dumps(info) + "\n", encoding="utf-8")
    print(json.dumps(info))


# --- PLATEAU LOD3 tran 3D Tiles -------------------------------------------------------------

def tileset_leaves(tileset: dict) -> list[str]:
    def walk(node):
        children = node.get("children", [])
        if node.get("content") and not children:
            yield node["content"]["uri"]
        for child in children:
            yield from walk(child)
    return list(walk(tileset["root"]))


def tile_function(attributes: dict):
    function = attributes.get("tran:function")
    if isinstance(function, list):
        function = function[0] if function else None
    return function


def read_b3dm(path: Path):
    """ECEF triangles (n, 3, 3) and the tran:function of each, from one b3dm tile."""
    import DracoPy  # noqa: PLC0415

    data = Path(path).read_bytes()
    _, _, _, ftj, ftb, btj, btb = struct.unpack("<4s6I", data[:28])
    feature_table = json.loads(data[28:28 + ftj])
    batch_table = json.loads(data[28 + ftj + ftb:28 + ftj + ftb + btj])
    glb = data[28 + ftj + ftb + btj + btb:]
    json_length = struct.unpack("<I", glb[12:16])[0]
    gltf = json.loads(glb[20:20 + json_length])
    offset = 20 + json_length
    binary_length = struct.unpack("<I", glb[offset:offset + 4])[0]
    binary = glb[offset + 8:offset + 8 + binary_length]
    center = np.array(gltf.get("extensions", {}).get("CESIUM_RTC", {}).get(
        "center", feature_table.get("RTC_CENTER", [0, 0, 0])))
    functions = [tile_function(a) for a in batch_table["attributes"]]
    triangles, tags = [], []
    for node in gltf.get("nodes", []):
        if "mesh" not in node:
            continue
        matrix = np.array(node.get("matrix", np.eye(4).ravel(order="F").tolist())).reshape(4, 4, order="F")
        for primitive in gltf["meshes"][node["mesh"]]["primitives"]:
            draco = primitive["extensions"]["KHR_draco_mesh_compression"]
            view = gltf["bufferViews"][draco["bufferView"]]
            start = view.get("byteOffset", 0)
            mesh = DracoPy.decode(binary[start:start + view["byteLength"]])
            points = np.asarray(mesh.points, dtype=float)
            batch = np.asarray(mesh.get_attribute_by_unique_id(draco["attributes"]["_BATCHID"])["data"]).ravel()
            faces = np.asarray(mesh.faces, dtype=int)
            p = (np.c_[points, np.ones(len(points))] @ matrix.T)[:, :3]
            p = np.c_[p[:, 0], -p[:, 2], p[:, 1]] + center  # glTF y-up -> 3D Tiles z-up
            triangles.append(p[faces])
            tags += [functions[int(batch[f[0]])] for f in faces]
    return (np.concatenate(triangles) if triangles else np.zeros((0, 3, 3))), tags


def command_plateau_fetch(args) -> int:
    for url in args.tileset_url:
        name = Path(urllib.parse.urlparse(url).path).parent.name
        out = args.out / name
        fetch(url, out / "tileset.json")
        leaves = tileset_leaves(json.loads((out / "tileset.json").read_text(encoding="utf-8")))
        base = url.rsplit("/", 1)[0]
        for uri in leaves:
            if not (out / uri).is_file():
                fetch(f"{base}/{uri}", out / uri)
        print(f"{name}: {len(leaves)} leaf tiles")
    return 0


def command_plateau_cache(args) -> int:
    triangles, n_tiles = [], 0
    for tile in sorted(args.tiles.glob("*/data/**/*.b3dm")):
        tri, tags = read_b3dm(tile)
        n_tiles += 1
        if not len(tri):
            continue
        keep = np.isin(np.array(tags), ROAD_FUNCTIONS)
        if keep.any():
            triangles.append(tri[keep])
    triangles = np.concatenate(triangles)
    np.save(args.out, triangles)
    print(f"tiles {n_tiles} road triangles {len(triangles)}")
    return 0


def rasterize_road(triangles_local):
    """Heights of every 1 m grid node inside each triangle: {(x, y): [z, ...]}."""
    nodes: dict[tuple[int, int], list[float]] = {}
    for a, b, c in triangles_local:
        lo = np.floor(np.minimum(np.minimum(a, b), c)[:2])
        hi = np.ceil(np.maximum(np.maximum(a, b), c)[:2])
        xs, ys = np.meshgrid(np.arange(lo[0], hi[0] + 1), np.arange(lo[1], hi[1] + 1))
        p = np.c_[xs.ravel(), ys.ravel()]
        v0, v1, v2 = c[:2] - a[:2], b[:2] - a[:2], p - a[:2]
        d00, d01, d11 = v0 @ v0, v0 @ v1, v1 @ v1
        den = d00 * d11 - d01 * d01
        if abs(den) < 1e-12:
            continue
        d02, d12 = v2 @ v0, v2 @ v1
        u = (d11 * d02 - d01 * d12) / den
        w = (d00 * d12 - d01 * d02) / den
        inside = (u >= -1e-9) & (w >= -1e-9) & (u + w <= 1 + 1e-9)
        if not inside.any():
            continue
        z = a[2] + u[inside] * (c[2] - a[2]) + w[inside] * (b[2] - a[2])
        for (x, y), zz in zip(p[inside].astype(int), z):
            nodes.setdefault((x, y), []).append(zz)
    return nodes


def command_plateau_map(args) -> int:
    from scipy.spatial import cKDTree  # noqa: PLC0415

    sols, enu, track = solution_track(args.solution)
    tree = cKDTree(track[:, :2])
    all_triangles = np.load(args.triangles)
    distance, _ = tree.query(enu(all_triangles.mean(axis=1))[:, :2])
    near = all_triangles[distance < 80.0]
    local = enu(near.reshape(-1, 3)).reshape(-1, 3, 3)
    nodes = rasterize_road(local)
    xy = np.array(list(nodes.keys()), dtype=float)
    heights = list(nodes.values())
    spread = np.array([max(v) - min(v) for v in heights])
    median = np.array([float(np.median(v)) for v in heights])
    distance, _ = tree.query(xy)
    keep = (spread <= MULTILEVEL_SPREAD_M) & (distance <= CORRIDOR_M)
    local_xyz = np.c_[xy[keep], median[keep]]
    _, _, map_height = ecef_to_llh(enu.to_ecef(local_xyz))
    offset, sigma, n_fix = offset_and_sigma(sols, track, local_xyz[:, :2], map_height)
    info = {"source": "PLATEAU LOD3 tran", "points": int(keep.sum()),
            "dropped_multilevel": int(((spread > MULTILEVEL_SPREAD_M) & (distance <= CORRIDOR_M)).sum()),
            "fix_epochs_on_map": n_fix, "offset_m": round(offset, 3), "sigma_mad_m": round(sigma, 3),
            "triangles": int(len(local))}
    write_map(args.out, enu, local_xyz, info)
    return 0


# --- GSI DEM5A ------------------------------------------------------------------------------

def dem_tile(lat: float, lon: float, zoom: int = 15) -> tuple[int, int]:
    n = 2 ** zoom
    x = int((lon + 180) / 360 * n)
    y = int((1 - math.asinh(math.tan(math.radians(lat))) / math.pi) / 2 * n)
    return x, y


def decode_dem_png(rgb) -> np.ndarray:
    """GSI elevation PNG: x = 2^16 R + 2^8 G + B; 0.01 m units, two's complement at 2^23, 2^23 = no data."""
    v = rgb[:, :, 0].astype(np.int64) * 65536 + rgb[:, :, 1].astype(np.int64) * 256 + rgb[:, :, 2].astype(np.int64)
    height = np.where(v < 2 ** 23, v * 0.01, (v - 2 ** 24) * 0.01)
    height[v == 2 ** 23] = np.nan
    return height


def solution_extent(paths):
    sols = [s for path in paths for s in read_libgnss_pos(path)
            if np.all(np.isfinite(s.ecef)) and np.linalg.norm(s.ecef) > 6e6]
    lat = np.array([s.lat_deg for s in sols])
    lon = np.array([s.lon_deg for s in sols])
    # Drop gross SPP outliers before taking the extent.
    keep = (np.abs(lat - np.median(lat)) < 0.2) & (np.abs(lon - np.median(lon)) < 0.2)
    return lat[keep], lon[keep]


def command_dem_fetch(args) -> int:
    lat, lon = solution_extent(args.solution)
    tiles = {dem_tile(a + dl, b + dm) for a, b in zip(lat, lon)
             for dl in (-0.0004, 0, 0.0004) for dm in (-0.0005, 0, 0.0005)}
    for x, y in sorted(tiles):
        path = args.out / f"15_{x}_{y}.png"
        if not path.is_file():
            fetch(DEM_URL.format(x=x, y=y), path)
    print(f"{len(tiles)} DEM tiles")
    return 0


def command_dem_geoid(args) -> int:
    lat, lon = solution_extent(args.solution)
    s, n, w, e = lat.min() - 0.002, lat.max() + 0.002, lon.min() - 0.002, lon.max() + 0.002
    rows = []
    for la, lo in ((s, w), (s, e), (n, w), (n, e), ((s + n) / 2, (w + e) / 2)):
        query = urllib.parse.urlencode({"outputType": "json", "latitude": f"{la:.3f}", "longitude": f"{lo:.3f}"})
        with urllib.request.urlopen(f"{GEOID_URL}?{query}", timeout=60) as response:
            rows.append((round(la, 3), round(lo, 3), float(json.loads(response.read())["OutputData"]["geoidHeight"])))
        time.sleep(1.0)
    args.out.write_text("".join(f"{a} {b} {c}\n" for a, b, c in rows), encoding="utf-8")
    print(rows)
    return 0


def command_dem_map(args) -> int:
    from PIL import Image  # noqa: PLC0415
    from scipy.spatial import cKDTree  # noqa: PLC0415

    geoid = np.loadtxt(args.geoid)
    lat_c, lon_c = float(np.round(geoid[:, 0].mean(), 2)), float(np.round(geoid[:, 1].mean(), 2))
    design = np.c_[np.ones(len(geoid)), geoid[:, 0] - lat_c, geoid[:, 1] - lon_c]
    plane = np.linalg.lstsq(design, geoid[:, 2], rcond=None)[0]
    pixels = []
    for tile in sorted(args.tiles.glob("15_*_*.png")):
        _, x, y = tile.stem.split("_")
        x, y = int(x), int(y)
        height = decode_dem_png(np.asarray(Image.open(tile).convert("RGB")))
        n = 2 ** 15
        j, i = np.meshgrid(np.arange(256), np.arange(256))
        lon = (x + (j + 0.5) / 256) / n * 360 - 180
        lat = np.degrees(np.arctan(np.sinh(np.pi * (1 - 2 * (y + (i + 0.5) / 256) / n))))
        pixels.append(np.c_[lat.ravel(), lon.ravel(), height.ravel()])
    pixels = np.concatenate(pixels)
    pixels = pixels[np.isfinite(pixels[:, 2])]
    sols, enu, track = solution_track(args.solution)
    tree = cKDTree(track[:, :2])
    pixel_xy = enu(llh_to_ecef(pixels[:, 0], pixels[:, 1], np.zeros(len(pixels))))[:, :2]
    distance, _ = tree.query(pixel_xy)
    pixels, pixel_xy = pixels[distance < 45], pixel_xy[distance < 45]
    lo = np.floor(track[:, :2].min(axis=0)) - CORRIDOR_M
    hi = np.ceil(track[:, :2].max(axis=0)) + CORRIDOR_M
    gx, gy = np.meshgrid(np.arange(lo[0], hi[0] + 1), np.arange(lo[1], hi[1] + 1))
    grid = np.c_[gx.ravel(), gy.ravel()]
    distance, _ = tree.query(grid)
    grid = grid[distance <= CORRIDOR_M]
    near, index = cKDTree(pixel_xy).query(grid, k=4)
    ok = near[:, 0] < 4.0
    weight = 1 / np.maximum(near[ok], 0.05)
    orthometric = (pixels[index[ok], 2] * weight).sum(axis=1) / weight.sum(axis=1)
    grid = grid[ok]
    lat, lon, _ = ecef_to_llh(enu.to_ecef(np.c_[grid, np.zeros(len(grid))]))
    height = orthometric + plane[0] + plane[1] * (lat - lat_c) + plane[2] * (lon - lon_c)
    offset, sigma, n_fix = offset_and_sigma(sols, track, grid, height)
    info = {"source": "GSI DEM5A", "points": int(len(grid)), "fix_epochs_on_map": n_fix,
            "offset_m": round(offset, 3), "sigma_mad_m": round(sigma, 3),
            "geoid_plane": [round(float(v), 4) for v in plane], "geoid_plane_center": [lat_c, lon_c]}
    write_llh(args.out, lat, lon, height, info)
    return 0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="command", required=True)
    p = sub.add_parser("plateau-fetch", help="download every leaf tile of PLATEAU tilesets")
    p.add_argument("--tileset-url", nargs="+", required=True)
    p.add_argument("--out", type=Path, required=True)
    p.set_defaults(func=command_plateau_fetch)
    p = sub.add_parser("plateau-cache", help="decode the tiles once; save road triangles (ECEF)")
    p.add_argument("--tiles", type=Path, required=True)
    p.add_argument("--out", type=Path, required=True)
    p.set_defaults(func=command_plateau_cache)
    p = sub.add_parser("plateau-map", help="road-height map for one run from cached triangles")
    p.add_argument("--triangles", type=Path, required=True)
    p.add_argument("--solution", type=Path, required=True, help="the run's own gnss_solve .pos")
    p.add_argument("--out", type=Path, required=True)
    p.set_defaults(func=command_plateau_map)
    p = sub.add_parser("dem-fetch", help="download the GSI DEM5A z15 tiles under the runs' solutions")
    p.add_argument("--solution", type=Path, nargs="+", required=True)
    p.add_argument("--out", type=Path, required=True)
    p.set_defaults(func=command_dem_fetch)
    p = sub.add_parser("dem-geoid", help="GSI geoid heights at the corners and centre of the runs' solutions")
    p.add_argument("--solution", type=Path, nargs="+", required=True)
    p.add_argument("--out", type=Path, required=True)
    p.set_defaults(func=command_dem_geoid)
    p = sub.add_parser("dem-map", help="height map for one run from DEM tiles and geoid points")
    p.add_argument("--tiles", type=Path, required=True)
    p.add_argument("--geoid", type=Path, required=True, help="rows of 'lat lon geoid_height'")
    p.add_argument("--solution", type=Path, required=True)
    p.add_argument("--out", type=Path, required=True)
    p.set_defaults(func=command_dem_map)
    args = parser.parse_args(argv)
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
