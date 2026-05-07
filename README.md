# libgnss++ — Modern C++ GNSS/RTK/PPP/CLAS Toolkit

Native non-GUI GNSS stack in modern C++17 with built-in `SPP`, `RTK`, `PPP`, `CLAS/MADOCA`, `RTCM`, `UBX`, and direct `QZSS L6` handling.

The point of this repo is simple: ship a usable GNSS toolchain without depending on an external RTKLIB runtime.

If this repo is useful, star it.

![UrbanNav Odaiba social card](docs/driving_odaiba_social_card.png)

Contribution and PR workflow: [CONTRIBUTING.md](CONTRIBUTING.md)
Architecture notes: [docs/architecture.md](docs/architecture.md)
Documentation index: [docs/index.md](docs/index.md)

## CLAS Performance vs CLASLIB

QZSS CLAS (Centimeter-Level Augmentation Service) PPP from raw L6 binary, 2019-08-27 static dataset (TRM59800.80 antenna), 1 hour (3599 epochs):

| Metric | gnssplusplus `--claslib-parity` | CLASLIB |
|--------|-----------------------------:|--------:|
| Matched fixed epochs | **3594 / 3599 (99.86%)** | 3594 / 3599 (99.86%) |
| **RMS 3D (fixed-only)** | **3.57 mm** | 7.29 mm |
| 3D bias (mean offset) | **1.66 mm** | 4.84 mm |
| RMS East | **1.15 mm** | 1.52 mm |
| RMS North | **1.21 mm** | 0.92 mm |
| RMS Up | **3.15 mm** | 7.07 mm |
| Mean E / N / U | -0.72 / +0.93 / +1.17 mm | +0.65 / -0.59 / +4.76 mm |
| First fix epoch | epoch 6 | epoch 6 |
| CLASLIB runtime link | not required | required |
| Parity depth | 17 helpers at 1e-6 m vs CLASLIB oracle | reference |

| CLASLIB 2D | gnssplusplus 2D |
|---|---|
| ![CLASLIB 2D](docs/clas_claslib_2d.png) | ![gnssplusplus 2D](docs/clas_native_2d.png) |

gnssplusplus achieves **51% lower RMS 3D** and **~3x tighter 3D bias** than upstream CLASLIB on the same 1-hour window while keeping the same fix rate, **with no CLASLIB runtime dependency on the default path**. The `ClasnatParity` GoogleTest suite pins 17 core helpers (`windupcorr`, `antmodel`, `ionmapf`, `prectrop`, `corrmeas`, `satpos_ssr`, `tidedisp`, `eph2clk`, `eph2pos`, `geodist`, `satantoff`, `compensatedisp`, `trop_grid_data`, `filter`, `lambda`, `tropmodel`, `stec_grid_data`) to 1e-6 m parity against the CLASLIB C source.

Opt-ins:

- `-DCLASLIB_PARITY_LINK=ON` + `--claslib-bridge`: delegate to upstream CLASLIB `postpos()` linked as a static library (oracle mode)
- `--legacy-strict-parity`: iter13-era non-native strict OSR path (regression reference)

See `docs/clas_port_architecture.md` for the port design and `docs/clas_validated_datasets.md` for the validated dataset set.

## RTK Performance vs RTKLIB (demo5)

The primary public RTK benchmark is
[taroz/PPC-Dataset](https://github.com/taroz/PPC-Dataset): urban Tokyo/Nagoya
vehicle runs with survey-grade receiver observations, reference-station
observations, broadcast navigation data, and trajectory truth. The comparison
below solves the same public rover/base/nav observations with gnssplusplus and
RTKLIB `demo5`. It is **not** a proprietary receiver-engine comparison.
UrbanNav Tokyo Odaiba is dominated by the explicit `--preset odaiba` opt-in
profile across Fix count, Fix rate, Hmed, Hp95, and Vp95.

On PPC Tokyo and Nagoya, the current gnssplusplus `develop` branch dominates
RTKLIB `demo5` on positioned-epoch precision and Fix rate with **no Phase 2
opt-in flags**. Positioning rate is tracked as a separate first-class metric:
the PPC coverage profile keeps valid SPP/float fallback epochs and now exceeds
RTKLIB `demo5` on Positioning rate for all six public Tokyo/Nagoya runs.
UrbanNav Tokyo Odaiba is kept as an independent public urban stress smoke:
gnssplusplus wins Fix count, Hp95, and Vp95 there, while the Hmed gap closes to
9 cm when wide-lane AR is explicitly enabled.

All runs below use `--mode kinematic --preset low-cost --match-tolerance-s
0.25`. The coverage profile additionally uses `--no-arfilter` plus the default
low-speed non-FIX drift guard and SPP height-step guard, the default FLOAT
bridge-tail guard, and `--ratio 2.4`. The kinematic post-filter cascade was
removed in PR #36 (single-epoch height-step drop only), so
`--no-kinematic-post-filter` is no longer required for coverage parity with the
default profile.

### Benchmark Scope

| Dataset | Role | Receiver/input basis | Comparison target |
|---|---|---|---|
| PPC Tokyo/Nagoya | Primary public moving-RTK sign-off | Septentrio mosaic-X5 rover RINEX plus Trimble Alloy/NetR9 base RINEX/nav and `reference.csv` truth | gnssplusplus vs RTKLIB `demo5` on the same observations |
| UrbanNav Tokyo Odaiba | External urban stress smoke | Public Odaiba rover/base/nav and Applanix reference | gnssplusplus vs RTKLIB `demo5`; not a receiver-engine benchmark |

`ppc-demo` summaries record this under `receiver_observation_provenance`,
including the rover/base receiver and antenna model. `receiver_engine_solution_available`
is intentionally `false` for PPC because the benchmark target is the open
observation solve against reference truth.

The checked-in scorecard is generated from `gnss ppc-coverage-matrix` output,
so it shows Positioning-rate wins first and keeps Fix-rate, PPC official
distance-ratio score, and P95 horizontal-error deltas visible in the same view.

![PPC RTK coverage scorecard](docs/ppc_rtk_demo5_scorecard.png)

### PPC Tokyo precision profile (kinematic, low-cost preset, no Phase 2 flags)

This fixed-output table is the precision-oriented view. The coverage table
below is the sign-off view for no-solution gaps and fallback-positioned epochs.

| Run  | gnssplusplus Fix / rate | RTKLIB Fix / rate | Hmed (m)              | Vp95 (m)               |
|------|------------------------:|------------------:|:---------------------:|:----------------------:|
| run1 | **3572 / 81.26%**       | 2418 / 30.52%     | **0.037** vs 1.567 (42×) | **1.259** vs 36.703 (29×) |
| run2 | **4674 / 80.12%**       | 2127 / 27.58%     | **0.016** vs 0.835 (52×) | **0.313** vs 42.624 (136×) |
| run3 | **7516 / 86.84%**       | 5778 / 40.55%     | **0.012** vs 0.666 (56×) | **0.137** vs 24.521 (179×) |

### PPC coverage profile (GNSS-only fallback epochs retained)

<!-- PPC_COVERAGE_MATRIX:START -->
| Run | gnssplusplus Positioning | RTKLIB Positioning | Delta | gnssplusplus Fix | RTKLIB Fix | PPC official score | RTKLIB official score | Official delta | P95 H delta |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| Tokyo run1 | **90.0%** | 66.3% | **+23.7 pp** | **54.4%** | 30.5% | **34.9%** | 0.0% | **+34.9 pp** | +3.39 m |
| Tokyo run2 | **95.3%** | 84.3% | **+11.0 pp** | **64.1%** | 27.6% | **69.0%** | 16.9% | **+52.1 pp** | -18.51 m |
| Tokyo run3 | **95.7%** | 93.1% | **+2.5 pp** | **63.0%** | 40.5% | **60.6%** | 35.6% | **+25.0 pp** | -0.24 m |
| Nagoya run1 | **88.8%** | 65.8% | **+23.0 pp** | **64.5%** | 33.8% | **49.5%** | 22.4% | **+27.1 pp** | -23.78 m |
| Nagoya run2 | **85.6%** | 69.8% | **+15.8 pp** | **51.4%** | 18.8% | **20.9%** | 11.0% | **+9.9 pp** | -27.24 m |
| Nagoya run3 | **93.8%** | 67.7% | **+26.1 pp** | **27.1%** | 13.9% | **27.4%** | 7.6% | **+19.7 pp** | -5.37 m |

Across these six public runs, the coverage profile averages **+17.0 pp**
Positioning-rate lead, **+28.1 pp** PPC official-score lead, and
**-11.96 m** P95 horizontal-error delta versus RTKLIB `demo5`.
<!-- PPC_COVERAGE_MATRIX:END -->

### PPC realtime status profile (deployable Wrong/FIX sign-off)

The current PPC status-correctness profile is
`--realtime-profile sigma-demote`. It keeps the benchmark on the same public
Tokyo/Nagoya rover/base/nav observations and uses PPC `reference.csv` only after
the run for scoring and Wrong-FIX labeling. The runtime selector itself is
deployable-only: it uses solver diagnostics such as NIS per observation, ratio,
residual/jump guards, and emitted status, not reference truth.

On the six public PPC runs, the recommended `nis2-ratio4` profile reaches
**64.686%** weighted PPC official score with **6.188x** minimum realtime factor.
Its purpose is not to maximize the number of emitted FIX epochs; it minimizes
the measured Wrong/FIX rate while staying realtime:

| Profile | Weighted official | FIX epochs | Wrong FIX | Wrong/FIX | Min realtime |
|---|---:|---:|---:|---:|---:|
| sigma profile, no status demotion | 64.750% | 41608 | 3078 | 7.398% | 6.883x |
| `sigma-demote` (`--demote-fixed-status-nis-per-obs 2`) | 64.707% | 29705 | 563 | 1.895% | 7.046x |
| `sigma-demote` + ratio status demotion (`--demote-fixed-status-max-ratio 4`) | 64.686% | 27395 | 470 | 1.716% | 6.188x |

This is a public-run replay result, not a hidden/private PPC leaderboard
submission. The RTKLIB `demo5` comparison remains the open-observation baseline
shown above; do not describe this as a proprietary receiver-engine comparison.
Full rejected candidates and the Wrong-FIX audit are recorded in
[`docs/ppc_realtime_gate_existing_outputs.md`](docs/ppc_realtime_gate_existing_outputs.md).

### PPC tuning history and diagnostics

The README keeps the sign-off view above. The long exploratory trail is split
into focused notes:

- [Benchmarks](docs/benchmarks.md): PPC coverage matrix, historical selector
  experiments, IMU/dropout bridge diagnostics, scorecards, and reproduction
  commands.
- [PPC realtime gate outputs](docs/ppc_realtime_gate_existing_outputs.md):
  accepted `sigma-demote nis2-ratio4` result plus rejected real-time gate candidates.
- [PPC smoother sanity report](docs/ppc_smoother_oracle_report.md): smoother
  and oracle/reference-selected checks kept as diagnostics only.

PPC2024's official score is a distance ratio with 3D error <= 50 cm. The
published first-place result was 78.7% Public / 85.6% Private in
[PPC2024 results](https://taroz.net/data/PPC2024_results.pdf). The tables above
use the same score definition on public open runs, but they are local replays,
not official Kaggle submissions or hidden Private split results.

### PPC Nagoya (same preset)

| Run  | Fix delta    | rate delta    | Hmed delta     |
|------|-------------:|--------------:|---------------:|
| run1 | **+1743**    | **+58.03 pp** | **9× better**  |
| run2 | **+1735**    | **+64.00 pp** | **10× better** |
| run3 | **+154**     | **+50.16 pp** | **44× better** |

### UrbanNav Tokyo Odaiba (kinematic, low-cost preset)

| Config                                                                     | Fix              | Rate        | Hmed (m)           | Hp95 (m)    | Vp95 (m)    |
|----------------------------------------------------------------------------|-----------------:|------------:|:------------------:|:-----------:|:-----------:|
| RTKLIB demo5                                                               | 595              | 7.22%       | **0.707**          | 27.878      | 45.212      |
| gnssplusplus default                                                       | **1268** (+673)  | **36.98%**  | 1.707              | **19.585**  | **25.495**  |
| gnssplusplus `--preset odaiba`                                             | **735** (+140)   | **32.81%**  | **0.698**          | **19.976**  | **26.440**  |

PPC Tokyo + Nagoya need no Phase 2 flags. On Odaiba, `--preset odaiba` is the
explicit all-metric demo5-beating profile.

| RTKLIB 2D | libgnss++ 2D |
|---|---|
| ![RTKLIB 2D](docs/driving_odaiba_comparison_rtklib_2d.png) | ![libgnss++ 2D](docs/driving_odaiba_comparison_libgnss_2d.png) |

## Phase 2 opt-in tuning gates

The default RTK pipeline is the benchmark path. Additional gates are default-off
unless explicitly enabled for diagnostics or dataset-specific profiles such as
Odaiba.

Common entry points:

- Use `--realtime-profile sigma-demote` for the current PPC deployable
  Wrong/FIX sign-off.
- Use `--preset odaiba` for the Odaiba all-metric demo5-beating tradeoff.
- See [RTK tuning gates](docs/rtk_tuning_gates.md) for the full flag table and
  historical sweep context.

## Docs

- Public site: <https://rsasaki0109.github.io/gnssplusplus-library/>
- [Documentation index](docs/index.md)
- [Architecture notes](docs/architecture.md)
- [Reference analyses](docs/references/index.md)
- [Contribution workflow](CONTRIBUTING.md)

Local docs site:

```bash
python3 -m pip install -r requirements-docs.txt
python3 -m mkdocs serve
```

## Docker

Build the runtime image:

```bash
docker build -t libgnsspp:latest .
```

Pull the published image:

```bash
docker pull ghcr.io/rsasaki0109/gnssplusplus-library:develop
```

Run the CLI against a mounted workspace or dataset directory:

```bash
docker run --rm -it \
  -v "$PWD:/workspace" \
  libgnsspp:latest \
  solve --rover /workspace/data/rover_kinematic.obs \
  --base /workspace/data/base_kinematic.obs \
  --nav /workspace/data/navigation_kinematic.nav \
  --out /workspace/output/docker_rtk.pos
```

Serve the local web UI from inside the container:

```bash
docker run --rm -it \
  -p 8085:8085 \
  -v "$PWD:/workspace" \
  libgnsspp:latest \
  web --host 0.0.0.0 --port 8085 --root /workspace
```

The image installs the `gnss` dispatcher, Python helpers, and `libgnsspp` Python package, but it does not embed the repo sample datasets. Mount your source tree or your own dataset directory.

Run the web UI with Compose:

```bash
docker compose up gnss-web
```

Override the image if you want a local or tagged build:

```bash
LIBGNSSPP_IMAGE=ghcr.io/rsasaki0109/gnssplusplus-library:v0.1.0 docker compose up gnss-web
```

## What You Get

- Native solvers: `SPP`, `RTK`, `PPP`, `CLAS-style PPP`
- Native protocols: `RINEX`, `RTCM`, `UBX`, direct `QZSS L6`
- Raw/log tooling: `NMEA`, `NovAtel`, `SBP`, `SBF`, `Trimble`, `SkyTraq`, `BINEX`
- Product tooling: `fetch-products`, `ionex-info`, `dcb-info`
- Analysis tooling: `visibility`, `visibility-plot`, and `moving-base-plot` for az/el/SNR exports plus moving-base/visibility PNG quick-looks
- Moving-base tooling: `moving-base-prepare` plus `moving-base-signoff` for real bag/replay/live validation, including optional commercial receiver side-by-side summaries
- One CLI entrypoint: `gnss spp`, `solve`, `ppp`, `visibility`, `stream`, `convert`, `live`, `rcv`
- Local web UI: `gnss web` for benchmark snapshots, live/moving-base/PPP-product sign-offs, 2D trajectories, visibility views, artifact bundles, receiver status, and artifact links
- Built-in sign-off scripts and checked-in benchmark artifacts
- CMake install/export, Python bindings, and ROS2 playback node

![Feature overview](docs/libgnsspp_feature_overview.png)

## Quick Start

### Build

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
```

### First solutions

```bash
python3 apps/gnss.py spp \
  --obs data/rover_static.obs \
  --nav data/navigation_static.nav \
  --out output/spp_solution.pos

python3 apps/gnss.py solve \
  --rover data/short_baseline/TSK200JPN_R_20240010000_01D_30S_MO.rnx \
  --base data/short_baseline/TSKB00JPN_R_20240010000_01D_30S_MO.rnx \
  --nav data/short_baseline/BRDC00IGS_R_20240010000_01D_MN.rnx \
  --mode static \
  --out output/rtk_solution.pos

python3 apps/gnss.py ppp \
  --static \
  --obs data/rover_static.obs \
  --nav data/navigation_static.nav \
  --out output/ppp_solution.pos

python3 apps/gnss.py visibility \
  --obs data/rover_static.obs \
  --nav data/navigation_static.nav \
  --csv output/visibility.csv \
  --summary-json output/visibility_summary.json \
  --max-epochs 60

python3 apps/gnss.py replay \
  --rover-rinex data/rover_kinematic.obs \
  --base-rinex data/base_kinematic.obs \
  --nav-rinex data/navigation_kinematic.nav \
  --mode moving-base \
  --out output/moving_base_replay.pos \
  --max-epochs 20
```

### Inspect receiver logs

```bash
python3 apps/gnss.py ubx-info \
  --input logs/session.ubx \
  --decode-observations

python3 apps/gnss.py sbf-info \
  --input logs/session.sbf \
  --decode-pvt \
  --decode-lband \
  --decode-p2pp
```

### Useful commands

| Command | Purpose |
|---|---|
| `gnss spp` | Batch SPP from rover/nav RINEX |
| `gnss solve` | Batch RTK from rover/base/nav RINEX |
| `gnss ppp` | Batch PPP from rover RINEX plus nav or precise products |
| `gnss fetch-products` | Fetch and cache `SP3`/`CLK`/`IONEX`/`DCB` products |
| `gnss visibility` | Export azimuth/elevation/SNR visibility rows and summary JSON |
| `gnss stream` | Inspect and relay RTCM over file, NTRIP, TCP, or serial |
| `gnss ubx-info` / `gnss sbf-info` | Inspect receiver logs |
| `gnss ppc-rtk-signoff` | PPC Tokyo/Nagoya RTK sign-off profiles |
| `gnss ppc-coverage-matrix` | Full six-run PPC matrix with JSON/Markdown summaries |
| `gnss web` | Local browser UI for summary JSON, live/moving-base/PPP-product sign-offs, `.pos` trajectories, moving-base/visibility plots and histories, receiver status, and artifact/provenance links |

See all commands and options:

```bash
python3 apps/gnss.py --help
```

More CLI details are in [Interfaces](docs/interfaces.md).

### Local web UI

```bash
python3 apps/gnss.py web \
  --port 8085 \
  --rcv-status output/receiver.status.json
```

Then open `http://127.0.0.1:8085` to inspect Odaiba metrics, live/moving-base/PPP-product sign-offs, 2D trajectories, moving-base and visibility plots, moving-base history, PPC summaries, receiver status, and linked artifact bundles in a browser. The PPP products table links directly to fetched products, MALIB `.pos`, comparison CSV/PNG artifacts, and dataset provenance.

Long-running dashboard commands can also read TOML config files. See
`configs/web.example.toml`, `configs/live_signoff.example.toml`,
`configs/moving_base_signoff.example.toml`, `configs/ppc_rtk_signoff.example.toml`,
and `configs/ppp_products_ppc.example.toml`, then pass `--config-toml <file>`.

Container form:

```bash
docker run --rm -it -p 8085:8085 -v "$PWD:/workspace" \
  libgnsspp:latest web --host 0.0.0.0 --port 8085 --root /workspace
```

### Real moving-base sign-off

`gnss solve`, `gnss replay`, and `gnss live` accept `--mode moving-base`. For
real moving-base datasets, use `gnss moving-base-signoff` with a reference CSV
carrying per-epoch base/rover ECEF coordinates. Example TOML files live under
`configs/`; detailed commands are in [Quick Start](docs/quickstart.md).

### Product-driven PPP

Use `gnss fetch-products`, `gnss ppp-static-signoff`,
`gnss ppp-kinematic-signoff`, and `gnss ppp-products-signoff` for SP3/CLK/IONEX
/ DCB driven runs. See [Quick Start](docs/quickstart.md) and
`configs/ppp_products_ppc.example.toml` for full examples.

## Reproduce Benchmarks

`PPC-Dataset` can be verified directly from an extracted dataset tree. The
current sign-off command is:

```bash
python3 apps/gnss.py ppc-rtk-signoff \
  --dataset-root /datasets/PPC-Dataset \
  --city tokyo \
  --realtime-profile sigma-demote \
  --rtklib-bin /path/to/rnx2rtkp \
  --summary-json output/ppc_tokyo_run1_rtk_signoff.json
```

Full replay, RTKLIB comparison, and historical diagnostic commands are in
[PPC reproduction commands](docs/ppc_reproduction.md). Dataset source:
[taroz/PPC-Dataset](https://github.com/taroz/PPC-Dataset).

Other benchmark artifacts and checked sign-offs are documented in
[Benchmarks](docs/benchmarks.md) and [Validation](docs/validation.md).

## Install And Package

```bash
cmake --install build --prefix /opt/libgnsspp
```

Installed layout includes:

- `bin/gnss`, native binaries, Python wrappers, sign-off scripts, asset
  generators, CMake/pkg-config exports, and the `libgnsspp` Python package.

Examples:

```bash
# pkg-config
pkg-config --cflags --libs libgnsspp

# source the installed dispatcher
/opt/libgnsspp/bin/gnss --help
```

## Python And ROS2

Python bindings expose:

- RINEX/header inspection, `.pos` statistics, coordinate helpers, and file-based
  `SPP`, `PPP`, and `RTK` solve helpers.

ROS2 support includes a playback node that publishes `.pos` files as:

- `sensor_msgs/NavSatFix`, `geometry_msgs/PoseStamped`, `nav_msgs/Path`, and
  solution status / satellite-count telemetry.

## Tests And Dogfooding

Run the full non-GUI regression set:

```bash
ctest --test-dir build --output-on-failure
```

Important checks already covered in-tree:

- solver/unit tests, live realtime/error-handling regression, benchmark/image
  generation tests, installed-prefix packaging smoke tests, Python bindings, and
  ROS2 node smoke tests.

## Data

Bundled samples live under `data/`; generated benchmark outputs live under
`output/` and `docs/`.

## Scope

This repo is intentionally focused on a strong non-GUI GNSS stack.

It already covers native `RTK`, `PPP`, `CLAS`, `RTCM`, `UBX`, `QZSS L6`,
installed CLI tooling, benchmarks, sign-off scripts, and README asset
generation.

It is still not marketed as a perfect RTKLIB drop-in replacement. The remaining gaps are about scope breadth, not the core non-GUI workflow shipped here.

## License

MIT License. See [LICENSE](LICENSE).
