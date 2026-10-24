# README: real Odaiba trajectories in a 3D city

The opening GIF replays **fresh libgnss++ and RTKLIB solutions** from the public
UrbanNav Tokyo Odaiba observations. The orange path is RTKLIB `demo5 b34k`;
the mint path is libgnss++. Buildings are the official Project PLATEAU Koto-ku
2025 LOD2 model, with an OpenStreetMap road basemap.

## What is shown

- GPS week **2032**, TOW **273385.1–273415.1 s**: a 30-second recorded window,
  played in 12 seconds (2.5×), 96 frames at 8 fps, 960 × 420 pixels.
- The interval was chosen from reference motion alone: the first continuously
  moving 30-second window, starting at least 10 seconds into the reference,
  with forward 1.5-second speed above 2 m/s. It was not selected by comparative
  positioning error. All 301 solution epochs in this interval are FLOAT for
  RTKLIB and FIXED for libgnss++; FIXED is a solver state, not an accuracy guarantee.
- Both panes use the **same reference-driven camera**. Camera positions and
  headings come from reference, never from either solver. Reference is used
  after solving for visualization and scoring, not as a solver input.
- Horizontal coordinates and status codes are unmodified solver outputs.
  Recorded paths ahead of the playback cursor are visible: this is an offline
  replay, not a prediction or live-navigation demo. Gaps over 0.5 s are not joined;
  a missing current solution is labelled `NO SOLUTION`.
- This is a **horizontal visualization** on a flat schematic ground plane at
  42 m ellipsoidal height, with the paths drawn just above it. Estimated heights
  are not visualized; buildings retain their published geometry. The scene
  cannot measure vertical accuracy or building collisions. The 2025 map/model
  is context for the older drive, not a reconstruction of its collection date.

The trace and basemap are retained in [odaiba_hero_trace.json](odaiba_hero_trace.json)
and [odaiba_hero_basemap.png](odaiba_hero_basemap.png). Source hashes, commands,
solver versions and full-run results are recorded in
[odaiba_hero_provenance.json](odaiba_hero_provenance.json).
The basemap is a small, cached patch assembled from 20 OSM zoom-18 tiles; the
renderer uses the identical image in both panes, avoiding unequal streaming detail.

## Re-render the recorded trace

Run from the repository root on Linux with Python, npm, and Chromium installed
(the tested Chromium path is `/usr/bin/chromium`). No Cesium ion account is needed.
The renderer uses the inherited HTTPS proxy and TLS trust for public PLATEAU
assets, caches downloaded resources, and fails if scene resources do not load.

```bash
mkdir -p output/readme-hero
python3 -m venv --system-site-packages output/readme-hero/venv
. output/readme-hero/venv/bin/activate
python -m pip install numpy==2.3.5 Pillow==12.3.0 playwright==1.62.0
npm pack cesium@1.123.1 --cache output/readme-hero/npm-cache \
  --pack-destination output/readme-hero
mkdir -p output/readme-hero/cesium
tar -xzf output/readme-hero/cesium-1.123.1.tgz \
  -C output/readme-hero/cesium --strip-components=1
python scripts/readme_hero/render.py \
  --cesium-dir output/readme-hero/cesium/Build/Cesium \
  --output output/readme-hero/rebuilt.gif
```

GIF palette entries are reserved for the two method colors. Frame delays alternate
in centiseconds so 8 fps is actually 12 seconds rather than silently rounded faster.
Rasterization and live PLATEAU assets can change; a rebuild is not promised to be
byte-identical across browser versions. The raw trace snapshot is deterministic.

## Re-run both solvers from public observations

Build `gnss_solve` using the repository's normal CMake instructions. Build
RTKLIB `demo5 b34k` at commit `55a0f2c742a605a3d27a90b17556b2cfdf98ad50`
from [rtklibexplorer/RTKLIB](https://github.com/rtklibexplorer/RTKLIB).
The tested libgnss++ solver sources are the `develop` base
`a40998e2f8e7f312ef9f6b4a21b3eb8b52350330`; this PR changes no solver code.

```bash
# Reads only the four required members, not the entire 4.45 GB ZIP.
# ZIP CRCs are checked; an existing differing input is never overwritten.
python scripts/readme_hero/fetch_inputs.py

python apps/gnss.py odaiba-benchmark \
  --dataset-dir data/driving/Tokyo_Data/Odaiba \
  --rtklib-bin /path/to/RTKLIB/app/consapp/rnx2rtkp/gcc/rnx2rtkp \
  --base-position surveyed --mode kinematic --segment-epochs 0 \
  --lib-pos output/readme-hero/libgnss.pos \
  --lib-kml output/readme-hero/libgnss.kml \
  --rtklib-pos output/readme-hero/rtklib.pos \
  --comparison-png output/readme-hero/comparison.png \
  --scorecard-png output/readme-hero/scorecard.png \
  --social-card-png output/readme-hero/social.png \
  --summary-json output/readme-hero/summary.json

python scripts/readme_hero/prepare_trace.py \
  --reference-csv data/driving/Tokyo_Data/Odaiba/reference.csv \
  --lib-pos output/readme-hero/libgnss.pos \
  --rtklib-pos output/readme-hero/rtklib.pos \
  --start-tow 273385.1 --duration 30 \
  --output output/readme-hero/trace.json
```

Both solvers read the same rover/base/navigation files and use the same surveyed
base ECEF coordinate `(-3961904.9530, 3348993.7578, 3698211.7553)` m.
The default profile and GLONASS AR off match the existing Odaiba benchmark.
The freshly regenerated trace was checked against the tracked snapshot for exact
JSON data equality. Full-run scoring reproduced 1,298 vs 205 FIX epochs and
4.777 vs 25.622 m horizontal P95 on each solver's matched epochs. The GIF's short
window is illustrative; see [benchmarks](benchmarks.md) for the full context,
matching populations and limitations.

## Sources and attribution

- Drive/reference: [UrbanNavDataset](https://github.com/IPNL-POLYU/UrbanNavDataset#tokyo-dataset),
  public Tokyo archive linked by the upstream project. Raw data is downloaded to
  ignored `data/`; it is not added to this PR.
- Buildings: **国土交通省 Project PLATEAU**, 江東区 2025 建築物モデル LOD2,
  rendered with a uniform color. The exact tileset URL is in the trace and provenance;
  discover datasets through the [official catalog](https://api.plateauview.mlit.go.jp/datacatalog/plateau-datasets).
  See the [PLATEAU site policy](https://www.mlit.go.jp/plateau/site-policy/).
- Map image: **© OpenStreetMap contributors**,
  [copyright and attribution](https://www.openstreetmap.org/copyright).
  Map imagery and city-model derivatives retain their source licensing and are
  not relicensed under the repository's code license.
- Renderer: [CesiumJS](https://github.com/CesiumGS/cesium), Apache-2.0.
  Cesium is fetched from npm for rendering; its distribution is not vendored here.
