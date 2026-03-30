# Quick Start

## Build

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
```

## Docker

```bash
docker build -t libgnsspp:latest .
docker pull ghcr.io/rsasaki0109/gnssplusplus-library:develop
docker run --rm -it -v "$PWD:/workspace" libgnsspp:latest --help
docker compose up gnss-web
```

The container installs the `gnss` dispatcher and Python helpers under `/opt/libgnsspp`. Sample datasets are not embedded, so mount your source tree or your own dataset directory into `/workspace`.

## First solutions

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

## Main commands

| Command | Purpose |
|---|---|
| `gnss spp` | Batch SPP from rover/nav RINEX |
| `gnss solve` | Batch RTK from rover/base/nav RINEX |
| `gnss ppp` | Batch PPP from rover RINEX plus nav or precise products |
| `gnss visibility` | Export azimuth/elevation/SNR visibility rows and summary JSON from rover/nav RINEX |
| `gnss visibility-plot` | Render a visibility CSV into a polar/elevation PNG quick-look |
| `gnss moving-base-plot` | Render a moving-base solution/reference pair into a baseline/heading PNG quick-look |
| `gnss moving-base-prepare` | Extract rover/base UBX plus reference CSV from a ROS2 moving-base bag |
| `gnss moving-base-signoff` | Validate a real moving-base replay/live dataset against per-epoch base/rover reference coordinates |
| `gnss scorpion-moving-base-signoff` | One-command prepare + BRDC fetch + replay validation for the public SCORPION bag |
| `gnss fetch-products` | Fetch and cache `SP3`/`CLK`/`IONEX`/`DCB` files from local or remote sources |
| `gnss ppp-products-signoff` | Run static, kinematic, or PPC PPP sign-off with fetched product presets or templates, plus comparison CSV/PNG artifacts |
| `gnss stream` | Inspect and relay RTCM over file, NTRIP, TCP, or serial |
| `gnss convert` | Convert RTCM or UBX into simple RINEX outputs |
| `gnss ubx-info` | Inspect `NAV-PVT`, `RAWX`, `SFRBX` from file or serial |
| `gnss ionex-info` | Inspect `IONEX` headers, maps, and auxiliary DCB blocks |
| `gnss dcb-info` | Inspect `Bias-SINEX` or auxiliary DCB products |
| `gnss qzss-l6-info` | Inspect direct QZSS L6 frames and export Compact SSR payloads |
| `gnss web` | Local browser UI for summary JSON, live/moving-base/PPP-product sign-offs, trajectories, moving-base/visibility plots, receiver status, and artifact links |
| `gnss ppc-rtk-signoff` | Fixed RTK sign-off profiles for PPC Tokyo/Nagoya |

## Local web UI

```bash
python3 apps/gnss.py web \
  --port 8085 \
  --rcv-status output/receiver.status.json
```

Open `http://127.0.0.1:8085` to inspect benchmark tables, live/moving-base/PPP-product sign-offs, receiver status, moving-base/visibility plots, moving-base history, and linked artifacts/provenance. PPP-product rows include direct links to fetched products, MALIB `.pos`, comparison CSV/PNG, and dataset reference files.

You can also store the same arguments in `configs/web.example.toml` and run:

```bash
python3 apps/gnss.py web --config-toml configs/web.example.toml
```

Docker form:

```bash
docker run --rm -it -p 8085:8085 -v "$PWD:/workspace" \
  libgnsspp:latest web --host 0.0.0.0 --port 8085 --root /workspace
```

## Real moving-base sign-off

`moving-base-signoff` is for external real datasets. Provide recorded `replay` or `live` inputs plus a reference CSV with per-epoch base/rover ECEF coordinates.

Typical replay preparation flow:

```bash
python3 apps/gnss.py moving-base-prepare \
  --input /datasets/moving_base/2023-06-14T174658Z.zip \
  --rover-ubx-out output/moving_base_rover.ubx \
  --base-ubx-out output/moving_base_base.ubx \
  --reference-csv output/moving_base_reference.csv

python3 apps/gnss.py fetch-products \
  --date 2023-06-14 \
  --preset brdc-nav

python3 apps/gnss.py scorpion-moving-base-signoff \
  --input /datasets/moving_base/2023-06-14T174658Z.zip \
  --summary-json output/scorpion_moving_base_summary.json

python3 apps/gnss.py moving-base-signoff \
  --config-toml configs/moving_base_signoff.example.toml
```

## Product-driven PPP

```bash
python3 apps/gnss.py fetch-products \
  --date 2024-01-02 \
  --preset igs-final \
  --preset ionex \
  --preset dcb \
  --summary-json output/products.json

python3 apps/gnss.py ppp-static-signoff \
  --fetch-products \
  --product-date 2024-01-02 \
  --product sp3=https://cddis.nasa.gov/archive/gnss/products/{gps_week}/COD0OPSFIN_{yyyy}{doy}0000_01D_05M_ORB.SP3.gz \
  --product clk=https://cddis.nasa.gov/archive/gnss/products/{gps_week}/COD0OPSFIN_{yyyy}{doy}0000_01D_30S_CLK.CLK.gz \
  --product ionex=https://cddis.nasa.gov/archive/gnss/products/ionex/{yyyy}/{doy}/COD0OPSFIN_{yyyy}{doy}0000_01D_01H_GIM.INX.gz \
  --product dcb=https://cddis.nasa.gov/archive/gnss/products/bias/{yyyy}/CAS0MGXRAP_{yyyy}{doy}0000_01D_01D_DCB.BSX.gz \
  --summary-json output/ppp_static_summary.json

python3 apps/gnss.py ppp-kinematic-signoff \
  --max-epochs 120 \
  --require-common-epoch-pairs-min 120 \
  --require-reference-fix-rate-min 95 \
  --require-converged \
  --require-convergence-time-max 300 \
  --require-mean-error-max 7 \
  --require-p95-error-max 7 \
  --require-max-error-max 7 \
  --require-mean-sats-min 18 \
  --require-ppp-solution-rate-min 100

python3 apps/gnss.py ppp-products-signoff \
  --config-toml configs/ppp_products_ppc.example.toml
```

## Local docs

```bash
python3 -m pip install -r requirements-docs.txt
python3 -m mkdocs serve
```
