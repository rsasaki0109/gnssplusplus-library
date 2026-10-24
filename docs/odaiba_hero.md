# README: three real trajectories in one 3D scene

The opening GIF overlays **Ground Truth, libgnss++, and RTKLIB** in a single
PLATEAU/OpenStreetMap scene. The camera follows Ground Truth position and measured
heading, so deviations are visible relative to one reference instead of being
recentered independently in separate panes.

## Legend and measurements

- **White dashed:** Ground Truth (Applanix reference).
- **Mint solid:** libgnss++. Mint and white overlap where the estimate agrees
  with the reference; their horizontal coordinates are never offset for legibility.
- **Orange solid:** RTKLIB `demo5 b34k`.

The legend shows each method's current solution state and horizontal error for the
same reference epoch (within 1 ms timestamp tolerance), plus its median horizontal error over the
**same 171 exactly matched reference/solver epochs**. Selected-window median H is
**0.21 m for libgnss++ versus 7.50 m for RTKLIB**; P95 is 0.65 versus 9.86 m.
The native solution is mostly FLOAT here: solver status and actual reference error
are separate measurements.

## Explicitly selected showcase

This is intentionally a **favorable illustrative segment**, selected to make the
solver differences visible. It is not a representative or independent benchmark.
The choice is reproducible with `prepare_trace.py --select-showcase`:

1. Scan nominal 20-second windows on a two-second grid. Align the start to the
   first exact common epoch within one second after each grid point, so both
   current-position markers are available at the opening frame.
2. Require at least 80% exact common-epoch coverage, no common gap above 2.501 s,
   reference motion above 1 m/s (forward 1.5-second displacement), and native
   common-epoch horizontal P95 at most 1 m.
3. Rank eligible windows by descending RTKLIB common-epoch median H; break ties
   with the earliest start. These filters and the ranking deliberately favor
   showing the native solver's advantage; the full-run results remain linked below.

This selects GPS week **2032**, TOW **273700.0–273720.0 s**. There are 201 reference
samples, 191 native solutions (189 FLOAT, two SPP), and 173 RTKLIB solutions
(101 FLOAT, 72 DGPS). Missing positions remain missing. Gaps over 0.5 s are never
joined, and a missing current position is labelled `NO SOLUTION`; no gap filling,
error scaling, or horizontal shifts are used.

The 20-second drive is played in **10 seconds (2×)**: 80 frames at 8 fps,
**960 × 480 pixels**. The camera stays 32 m behind and 8 m above the reference
rendering plane, using measured reference heading even near turns. Ground Truth
is supplied only to the visualization/scoring step, never to either solver.

## Interpretation, data and rendering

This is an offline **horizontal visualization**, with prerecorded forward paths
visible. A flat schematic ground plane at 42 m ellipsoidal height places all paths
at 42.45 m; estimated heights are not visualized. Buildings retain their published
geometry. This is not a vertical-accuracy or building-collision measurement.
The official **Minato-ku 2025 PLATEAU LOD2** model and current OSM map are context
for the older drive, not a reconstruction of its collection date.

The measured trace, heading, errors and selection criteria are retained in
[odaiba_hero_trace.json](odaiba_hero_trace.json). The shared OSM snapshot is
[odaiba_hero_basemap.png](odaiba_hero_basemap.png), assembled from 20 zoom-18 tiles.
Input/solution hashes, full-run results and rendering details are in
[odaiba_hero_provenance.json](odaiba_hero_provenance.json).

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

GIF palette entries are reserved for the method colors and white reference. Frame delays alternate
in centiseconds so 8 fps is actually 10 seconds rather than silently rounded faster.
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
  --select-showcase --duration 20 \
  --output output/readme-hero/trace.json
```

Both solvers read the same rover/base/navigation files and use the same surveyed
base ECEF coordinate `(-3961904.9530, 3348993.7578, 3698211.7553)` m.
The default profile and GLONASS AR off match the existing Odaiba benchmark.
The selected trace is reproducible from the retained solver outputs and exact
reference epochs; both window selection and all displayed errors are checked. Full-run scoring reproduced 1,298 vs 205 FIX epochs and
4.777 vs 25.622 m horizontal P95 on each solver's matched epochs. The GIF's deliberately favorable short
window is illustrative; see [benchmarks](benchmarks.md) for the full context,
matching populations and limitations.

## Sources and attribution

- Drive/reference: [UrbanNavDataset](https://github.com/IPNL-POLYU/UrbanNavDataset#tokyo-dataset),
  public Tokyo archive linked by the upstream project. Raw data is downloaded to
  ignored `data/`; it is not added to this PR.
- Buildings: **国土交通省 Project PLATEAU**, 港区 2025 建築物モデル LOD2,
  rendered with a uniform color. The exact tileset URL is in the trace and provenance;
  discover datasets through the [official catalog](https://api.plateauview.mlit.go.jp/datacatalog/plateau-datasets).
  See the [PLATEAU site policy](https://www.mlit.go.jp/plateau/site-policy/).
- Map image: **© OpenStreetMap contributors**,
  [copyright and attribution](https://www.openstreetmap.org/copyright).
  Map imagery and city-model derivatives retain their source licensing and are
  not relicensed under the repository's code license.
- Renderer: [CesiumJS](https://github.com/CesiumGS/cesium), Apache-2.0.
  Cesium is fetched from npm for rendering; its distribution is not vendored here.
