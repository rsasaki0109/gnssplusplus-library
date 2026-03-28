# Quick Start

## Build

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
```

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
```

## Main commands

| Command | Purpose |
|---|---|
| `gnss spp` | Batch SPP from rover/nav RINEX |
| `gnss solve` | Batch RTK from rover/base/nav RINEX |
| `gnss ppp` | Batch PPP from rover RINEX plus nav or precise products |
| `gnss stream` | Inspect and relay RTCM over file, NTRIP, TCP, or serial |
| `gnss convert` | Convert RTCM or UBX into simple RINEX outputs |
| `gnss ubx-info` | Inspect `NAV-PVT`, `RAWX`, `SFRBX` from file or serial |
| `gnss qzss-l6-info` | Inspect direct QZSS L6 frames and export Compact SSR payloads |
| `gnss web` | Local browser UI for summary JSON, live sign-offs, trajectories, and receiver status |
| `gnss ppc-rtk-signoff` | Fixed RTK sign-off profiles for PPC Tokyo/Nagoya |

## Local web UI

```bash
python3 apps/gnss.py web \
  --port 8085 \
  --rcv-status output/receiver.status.json
```

Open `http://127.0.0.1:8085`.

## Local docs

```bash
python3 -m pip install -r requirements-docs.txt
python3 -m mkdocs serve
```
