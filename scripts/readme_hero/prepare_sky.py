"""Raycast a GT-anchored, model-only zenith fisheye and a stable satellite cohort.

Dependencies are preparation-only: numpy, Pillow, trimesh, embreex, DracoPy.
Runtime rendering consumes the frozen sprite/JSON without a geometry engine.
"""

import argparse
import csv
import hashlib
import io
import json
import math
import struct
import urllib.request
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from urllib.parse import urljoin

import numpy as np
import trimesh
from PIL import Image
from trimesh.ray.ray_pyembree import RayMeshIntersector

ROOT = Path(__file__).resolve().parents[2]
KOTO = "https://assets.cms.plateau.reearth.io/assets/69/e06917-ca70-4b93-810b-765fb29ce233/13108_koto-ku_pref_2025_citygml_1_op_bldg_3dtiles_13108_koto-ku_lod2_no_texture/tileset.json"
p = argparse.ArgumentParser(description=__doc__)
p.add_argument("--trace", type=Path, default=ROOT / "docs/odaiba_hero_trace.json")
p.add_argument("--satellites", type=Path, required=True)
p.add_argument(
    "--cache-dir", type=Path, default=ROOT / "output/readme-hero/resource-cache"
)
p.add_argument("--output-json", type=Path, default=ROOT / "docs/odaiba_hero_sky.json")
p.add_argument("--output-sprite", type=Path, default=ROOT / "docs/odaiba_hero_sky.png")
a = p.parse_args()
s = json.loads(a.trace.read_text())
building_models = [s["tilesetUrl"], KOTO] + [
    "https://assets.cms.plateau.reearth.io/assets/92/43303e-6eb5-44ad-b65f-e21e63bdf98a/13102_chuo-ku_pref_2025_citygml_1_op_bldg_3dtiles_13102_chuo-ku_lod2_no_texture/tileset.json",
    "https://assets.cms.plateau.reearth.io/assets/be/403449-37a8-4dbb-a756-c3d4e6639f5d/13109_shinagawa-ku_pref_2025_citygml_1_op_bldg_3dtiles_13109_shinagawa-ku_lod2_no_texture/tileset.json",
]
# Bridges (decks, elevated roads and walkways) block the sky as solidly as
# buildings but are a separate PLATEAU class. Without them the deck crossing
# the road near TOW 273710, where the receiver drops from 21 to 8 reported
# satellites, would stay model LOS. Same 2025 release, LOD2, same four wards.
BRIDGE_MODELS = [
    "https://assets.cms.plateau.reearth.io/assets/4c/2e16d4-56fd-492f-a67b-bbb38e8ef0d0/13103_minato-ku_pref_2025_citygml_1_op_brid_3dtiles_lod2/tileset.json",
    "https://assets.cms.plateau.reearth.io/assets/e5/a12f70-6426-49c1-94d3-0f8299e0d53f/13108_koto-ku_pref_2025_citygml_1_op_brid_3dtiles_lod2/tileset.json",
    "https://assets.cms.plateau.reearth.io/assets/2f/77dc90-7824-4349-924a-f5f862c430d4/13102_chuo-ku_pref_2025_citygml_1_op_brid_3dtiles_lod2/tileset.json",
    "https://assets.cms.plateau.reearth.io/assets/46/3c7ed9-452b-4210-980f-5cdb3bc9b3c3/13109_shinagawa-ku_pref_2025_citygml_1_op_brid_3dtiles_lod2/tileset.json",
]
models = building_models + BRIDGE_MODELS
model_class = {**{u: "bldg" for u in building_models}, **{u: "brid" for u in BRIDGE_MODELS}}
cache = a.cache_dir
cache.mkdir(parents=True, exist_ok=True)
records = []


def fetch(url):
    key = hashlib.sha256(url.encode()).hexdigest()
    path = cache / key
    meta = cache / (key + ".json")
    if not path.exists():
        req = urllib.request.Request(
            url,
            headers={
                "User-Agent": "gnssplusplus-library model zenith visualization (https://github.com/rsasaki0109/gnssplusplus-library)"
            },
        )
        with urllib.request.urlopen(req, timeout=60) as response:
            blob = response.read()
            kind = response.headers.get("Content-Type", "application/octet-stream")
        path.write_bytes(blob)
        meta.write_text(json.dumps({"url": url, "type": kind}))
    blob = path.read_bytes()
    return blob


with a.satellites.open(newline="") as handle:
    sat_rows = list(csv.DictReader(handle))
if not sat_rows:
    raise RuntimeError("No native satellite geometry")
sat_by_tow = {}
for r in sat_rows:
    sat_by_tow.setdefault(round(float(r["tow"]), 3), []).append(r)
ref = [r for r in s["reference"] if s["start"] <= r[0] <= s["end"]]
if len(ref) != len(sat_by_tow):
    raise RuntimeError("Each selected reference epoch needs native satellite geometry")
origin = np.array([float(sat_rows[0]["receiver_" + v]) for v in "xyz"])


def rotation(lat, lon):
    lat, lon = np.radians([lat, lon])
    sl, cl = np.sin(lat), np.cos(lat)
    so, co = np.sin(lon), np.cos(lon)
    return np.array([[-so, co, 0], [-sl * co, -sl * so, cl], [cl * co, cl * so, sl]])


R = rotation(ref[0][2], ref[0][1])
radius = 2000.0
min_el = 10.0
# Conservatively expand a bounding rectangle around the entire receiver route.
dlat = np.degrees(radius / 6370000)
dlon = dlat / math.cos(math.radians(ref[0][2]))
bounds = np.radians(
    [
        min(r[1] for r in ref) - dlon,
        min(r[2] for r in ref) - dlat,
        max(r[1] for r in ref) + dlon,
        max(r[2] for r in ref) + dlat,
    ]
)
urls = []
url_class = []
max_height = -math.inf
min_height = math.inf
for model in models:
    doc = json.loads(fetch(model))
    maximum = doc["root"]["boundingVolume"]["region"][5]
    max_height = max(max_height, maximum)
    min_height = min(min_height, doc["root"]["boundingVolume"]["region"][4])

    def collect(node, inherited="REPLACE"):
        if "transform" in node:
            raise RuntimeError(
                "Tile transforms require explicit handling; refusing to guess"
            )
        region = node["boundingVolume"].get("region")
        if region is None:
            raise RuntimeError("Unsupported tile bound")
        if (
            region[2] < bounds[0]
            or region[0] > bounds[2]
            or region[3] < bounds[1]
            or region[1] > bounds[3]
        ):
            return
        refine = node.get("refine", inherited)
        children = node.get("children", [])
        if "content" in node and (not children or refine == "ADD"):
            uri = node["content"].get("uri", node["content"].get("url"))
            if not uri.endswith((".b3dm", ".glb")):
                raise RuntimeError("Expected pinned b3dm or glb model")
            urls.append(urljoin(model, uri))
            url_class.append(model_class[model])
        for child in children:
            collect(child, refine)

    collect(doc["root"])
# Above 10 degrees, even the tallest building in these datasets cannot block
# a ray beyond this range. This does not cover unmodeled objects or other wards.
if (max_height - min(r[3] for r in ref)) / math.tan(math.radians(min_el)) >= radius:
    raise RuntimeError("Range insufficient for model height envelope")
print("Fetching", len(urls), "leaf tiles from the modeled wards", flush=True)
blobs = list(ThreadPoolExecutor(4).map(fetch, urls))
meshes = []
face_class = []
yup = np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]])
for url, kind, blob in zip(urls, url_class, blobs):
    if url.endswith(".glb"):
        # 3D Tiles 1.1 content: the glTF itself, without a b3dm feature table.
        if blob[:4] != b"glTF" or struct.unpack("<I", blob[8:12])[0] != len(blob):
            raise RuntimeError("Invalid glb")
        feature, glb = {}, blob
    else:
        magic, version, length, ftj, ftb, btj, btb = struct.unpack("<4s6I", blob[:28])
        if magic != b"b3dm" or version != 1 or length != len(blob):
            raise RuntimeError("Invalid b3dm")
        feature = json.loads(blob[28 : 28 + ftj])
        glb = blob[28 + ftj + ftb + btj + btb :]
    json_length = struct.unpack("<I", glb[12:16])[0]
    gltf = json.loads(glb[20 : 20 + json_length])
    # Trimesh decodes Draco and node transforms, but not CESIUM_RTC.
    center = np.array(
        gltf.get("extensions", {})
        .get("CESIUM_RTC", {})
        .get("center", feature.get("RTC_CENTER", [0, 0, 0]))
    )
    loaded = trimesh.load_scene(
        io.BytesIO(glb), file_type="glb", process=False
    ).to_mesh()
    ecef = loaded.vertices @ yup.T + center
    # A wrong Y-up/RTC convention can place buildings below the ellipsoid.
    # Verify the decoded geometry against the published height envelope.
    x, y, z = ecef.T
    horizontal = np.hypot(x, y)
    lat = np.arctan2(z, horizontal * (1 - 6.69437999014e-3))
    for _ in range(8):
        prime = 6378137 / np.sqrt(1 - 6.69437999014e-3 * np.sin(lat) ** 2)
        lat = np.arctan2(z + 6.69437999014e-3 * prime * np.sin(lat), horizontal)
    heights = horizontal / np.cos(lat) - prime
    if (
        not np.isfinite(heights).all()
        or heights.min() < min_height - 1
        or heights.max() > max_height + 1
    ):
        raise RuntimeError("Decoded mesh violates the published height envelope")
    vertices = (ecef - origin) @ R.T
    meshes.append(trimesh.Trimesh(vertices=vertices, faces=loaded.faces, process=False))
    face_class += [kind] * len(loaded.faces)
    records.append(
        {
            "url": url,
            "class": kind,
            "sha256": hashlib.sha256(blob).hexdigest(),
            "triangles": len(loaded.faces),
        }
    )
mesh = trimesh.util.concatenate(meshes)
if len(face_class) != len(mesh.faces):
    raise RuntimeError("Face classes do not follow the concatenated mesh")
ray = RayMeshIntersector(mesh)
print("Built BVH:", len(mesh.faces), "triangles", flush=True)

SIZE = 256
CENTER = 128
FISH_RADIUS = 112
Y, X = np.mgrid[:SIZE, :SIZE]
dx = (X - CENTER) / FISH_RADIUS
dy = (CENTER - Y) / FISH_RADIUS
rho = np.sqrt(dx * dx + dy * dy)
inside = rho <= 1
az = np.arctan2(dx[inside], dy[inside])
el = (1 - rho[inside]) * math.pi / 2
local = np.column_stack([np.sin(az) * np.cos(el), np.cos(az) * np.cos(el), np.sin(el)])
columns = 16
sheet = Image.new(
    "RGB", (columns * SIZE, math.ceil(len(ref) / columns) * SIZE), (12, 17, 26)
)
frames = []
for index, r in enumerate(ref):
    tow = round(r[0] % 604800, 3)
    observed = sat_by_tow[tow]
    receiver = np.array([float(observed[0]["receiver_" + v]) for v in "xyz"])
    orig = (receiver - origin) @ R.T
    current = rotation(r[2], r[1])
    directions = local @ current @ R.T
    points, indices, triangles = ray.intersects_location(
        np.tile(orig, (len(directions), 1)), directions, multiple_hits=False
    )
    distances = np.linalg.norm(points - orig, axis=1)
    valid = (distances > 0.05) & (distances <= radius)
    indices = indices[valid]
    triangles = triangles[valid]
    image = np.full((SIZE, SIZE, 3), (12, 17, 26), dtype=np.uint8)
    image[inside] = [161, 198, 225]
    shaded = np.tile(np.array([82, 100, 122]), (len(indices), 1))
    if len(indices):
        light = np.array([0.3, -0.4, 0.866])
        shade = 0.65 + 0.35 * np.abs(mesh.face_normals[triangles] @ light)
        shaded = np.clip(shaded * shade[:, None], 0, 255).astype(np.uint8)
    inner = image[inside]
    inner[indices] = shaded
    image[inside] = inner
    # Low elevations stay explicitly unassessed, including in the sky image.
    low = inside & (rho > (90 - min_el) / 90)
    image[low] = (image[low].astype(float) * 0.55).astype(np.uint8)
    sheet.paste(
        Image.fromarray(image), ((index % columns) * SIZE, (index // columns) * SIZE)
    )
    sv = []
    for sat in observed:
        pos = np.array([float(sat["satellite_" + v]) for v in "xyz"])
        direction = pos - receiver
        direction /= np.linalg.norm(direction)
        direction = direction @ R.T
        loc, _, hit = ray.intersects_location(
            np.array([orig]), np.array([direction]), multiple_hits=False
        )
        blocked = bool(
            len(loc) and 0.05 < float(np.linalg.norm(loc[0] - orig)) <= radius
        )
        eligible = float(sat["el_deg"]) >= min_el and int(sat["health"]) == 0
        label = ("NLOS" if blocked else "LOS") if eligible else "unassessed"
        x, y = float(sat["fish_x"]), float(sat["fish_y"])
        # Independent equidistant formula verifies native fisheye conventions.
        angle = math.radians(float(sat["az_deg"]))
        rad = FISH_RADIUS * (90 - float(sat["el_deg"])) / 90
        if (
            abs(x - (CENTER + rad * math.sin(angle))) > 1e-7
            or abs(y - (CENTER - rad * math.cos(angle))) > 1e-7
        ):
            raise RuntimeError("Fisheye axes do not agree")
        sv.append(
            {
                "id": sat["satellite"],
                "observed": bool(int(sat["observed"])),
                "az_deg": float(sat["az_deg"]),
                "el_deg": float(sat["el_deg"]),
                "x": x,
                "y": y,
                "model_status": label,
                "hit_distance_m": (
                    float(np.linalg.norm(loc[0] - orig)) if blocked else None
                ),
                "hit_class": face_class[int(hit[0])] if blocked else None,
            }
        )
    frames.append({"time": r[0], "satellites": sv})
    if index % 25 == 0:
        print("sky", index + 1, "/", len(ref), flush=True)
a.output_sprite.parent.mkdir(parents=True, exist_ok=True)
sheet.save(a.output_sprite)
meta = {
    "schema": "odaiba_gt_zenith.v2",
    "cohort_ids": sorted({r["satellite"] for r in sat_rows}),
    "satellite_cohort": "Fixed union of IDs reported at least once in the selected clip; every ID propagated at every reference epoch, independent of observation availability",
    "projection": "ideal equidistant, North up; native projFisheye",
    "sprite": {"cell_size": SIZE, "columns": columns, "frames": len(frames)},
    "frames": frames,
    "model": {
        "urls": models,
        "classes": {"bldg": building_models, "brid": BRIDGE_MODELS},
        "wards": ["Minato", "Koto", "Chuo", "Shinagawa"],
        "year": 2025,
        "radius_m": radius,
        "classification_min_elevation_deg": min_el,
        "maximum_model_height_ellipsoidal_m": max_height,
        "geometry": "most-detailed REPLACE leaves; ADD content retained; b3dm and glb content; glTF Y-up to Z-up plus CESIUM_RTC",
        "triangles": len(mesh.faces),
        "tiles": records,
    },
    "interpretation": "Model-predicted building and bridge blockage from published GT ECEF; not measured LOS, not historical imagery, no multipath simulation, not used by either solver. <10deg or unhealthy ephemeris is unassessed; clear rays refer only to these modeled wards/range.",
    "inputs": {
        "trace_sha256": hashlib.sha256(a.trace.read_bytes()).hexdigest(),
        "satellite_geometry_csv_sha256": hashlib.sha256(
            a.satellites.read_bytes()
        ).hexdigest(),
    },
    "sprite_sha256": hashlib.sha256(a.output_sprite.read_bytes()).hexdigest(),
}
a.output_json.write_text(json.dumps(meta, separators=(",", ":")) + "\n")
print("Saved sky sprite and metadata", a.output_sprite.stat().st_size, flush=True)
