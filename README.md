# libgnss++ — Modern C++ GNSS/RTK/PPP/CLAS Toolkit

Modern C++17 GNSS toolkit with native `SPP`, `RTK`, `PPP`, `CLAS/MADOCA`, `RTCM`, `UBX`, and direct `QZSS L6` support.

The goal is simple: ship a usable non-GUI GNSS stack without depending on an external RTKLIB runtime.

If this repo saves you time, consider starring it.

![Twitter-ready UrbanNav Odaiba social card](docs/driving_odaiba_social_card.png)

![libgnss++ feature overview](docs/libgnsspp_feature_overview.png)

## Why It Exists

- One native stack for `RTK`, `PPP`, `CLAS`, `RTCM`, `UBX`, and `QZSS L6`
- First-class non-GUI tooling: `gnss spp`, `solve`, `ppp`, `stream`, `convert`, `live`, `rcv`
- Checked-in benchmark artifacts and sign-off scripts instead of vague claims
- CMake install/export, Python bindings, and ROS2 playback in-tree

### UrbanNav Tokyo Odaiba

Open driving dataset: [UrbanNav Tokyo Odaiba](https://github.com/IPNL-POLYU/UrbanNavDataset) (`2018-12-19`, Trimble rover/base, ~`170 m` baseline).

Current checked-in Odaiba snapshot:
- **All matched epochs**: libgnss++ leads on matched-epoch count (`11637 vs 8241`), fix rate (`8.11% vs 7.22%`), p95 horizontal error (`7.58 m vs 27.88 m`), and max horizontal error (`61.82 m vs 82.87 m`).
- **Common epochs only**: horizontal median is within `30 mm` of RTKLIB (`0.733 m vs 0.704 m`), median vertical error is slightly better (`0.571 m vs 0.575 m`), and p95 tail is much better (`5.94 m vs 27.67 m`).
- **Machine-readable artifact**: the benchmark writes `output/odaiba_summary.json`.

| RTKLIB 2D | libgnss++ 2D |
|---|---|
| ![RTKLIB 2D trajectory](docs/driving_odaiba_comparison_rtklib_2d.png) | ![libgnss++ 2D trajectory](docs/driving_odaiba_comparison_libgnss_2d.png) |

More detail:
- [Full comparison figure](docs/driving_odaiba_comparison.png)
- [Detailed scorecard](docs/driving_odaiba_scorecard.png)
- [Summary JSON](output/odaiba_summary.json)

## Benchmark Snapshot

Tested on open datasets with dual-frequency observations.

| Dataset | Baseline | libgnss++ Fix Rate | RTKLIB Fix Rate | libgnss++ RMS (h) | RTKLIB RMS (h) |
|---------|----------|--------------------|-----------------|-------------------|----------------|
| **Kinematic** | 1.2 km | **100%** | 98.3% | **12 mm** | 8 mm |
| **Short static** | 36 m | **99%** | 100% | 90 mm | — |
| **Long static** | 3.3 km | 52% | 60.8% | 104 mm | 14 mm* |

\* The checked long-static RTKLIB output shows a large height offset relative to the RINEX header position; libgnss++ can stay much closer to the approximate header position on that sample.

### Kinematic Reference Comparison

![Kinematic RTK: libgnss++ vs RTKLIB](docs/kinematic_comparison.png)

## What Is Already Strong

- **Instant first fix** on the checked benchmark datasets
- **No RTKLIB runtime dependency**
- **Modern C++ core** with self-contained `LAMBDA`, troposphere, ionosphere, and solver logic
- **Wide protocol coverage** for a non-GUI toolchain
- **Sign-off scripts** for short-baseline RTK, kinematic RTK, static PPP, kinematic PPP, and Odaiba

## Architecture

```text
libgnss++/
  core/         constants, coordinates, types, observation, navigation, solution
  models/       troposphere, ionosphere
  algorithms/   spp, rtk, ppp, kalman, lambda
  io/           rinex, rtcm, ubx, ntrip, solution_writer
  apps/         native CLI tools and benchmark/sign-off entrypoints
  python/       pybind11 bindings
  ros2/         solution playback node
```

## Quick Start

### Build

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
```

### First Solutions

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

### C++ API

```cpp
#include <libgnss++/gnss.hpp>

int main() {
    libgnss::RTKProcessor rtk;
    libgnss::RTKProcessor::RTKConfig config;
    config.ambiguity_ratio_threshold = 3.0;
    rtk.setRTKConfig(config);

    libgnss::io::RINEXReader rover, base, nav_reader;
    rover.open("rover.obs");
    base.open("base.obs");
    nav_reader.open("navigation.nav");

    libgnss::NavigationData nav;
    nav_reader.readNavigationData(nav);

    libgnss::io::RINEXReader::RINEXHeader base_header;
    base.readHeader(base_header);
    rtk.setBasePosition(base_header.approximate_position);

    libgnss::ObservationData rover_obs, base_obs;
    while (rover.readObservationEpoch(rover_obs) && base.readObservationEpoch(base_obs)) {
        auto solution = rtk.processRTKEpoch(rover_obs, base_obs, nav);
        if (solution.status == libgnss::SolutionStatus::FIXED) {
            // cm-level position available
        }
    }
}
```

## Command-Line Tools

`libgnss++` ships with a small command suite built around project-native names instead of copying RTKLIB app names.

For cross-platform usage, the canonical entry point is the Python dispatcher:

```bash
python3 apps/gnss.py <command> ...
```

On Windows, the same interface is intended to be:

```powershell
py apps\gnss.py <command> ...
```

or:

```powershell
apps\gnss.cmd <command> ...
```

The `./apps/gnss_*` launchers remain as Unix convenience wrappers, but they are not the long-term portable interface.

### Core commands

| Command | Purpose |
|---------|---------|
| `gnss spp` | Batch SPP post-processing from rover/nav RINEX |
| `gnss solve` | Batch RTK post-processing from RINEX rover/base/nav |
| `gnss ppp` | Batch PPP post-processing from rover RINEX plus precise SP3/CLK products |
| `gnss nav-products` | Generate simple SP3/CLK files from observed epochs plus broadcast nav |
| `gnss rinex-info` | Inspect RINEX headers and optionally count epochs / ephemerides |
| `gnss stream` | Read RTCM from file or NTRIP, print message summaries, and relay frames |
| `gnss ubx-info` | Inspect UBX NAV/RAWX/SFRBX logs or serial streams and optionally export RAWX epochs to RINEX |
| `gnss nmea-info` | Inspect NMEA GGA/RMC logs or serial streams and print decoded position summaries |
| `gnss novatel-info` | Inspect NovAtel ASCII/Binary BESTPOS/BESTVEL logs or serial streams |
| `gnss sbp-info` | Inspect Swift Binary Protocol GPS_TIME/POS_LLH/VEL_NED logs or serial streams |
| `gnss sbf-info` | Inspect Septentrio SBF PVTGeodetic/LBandTrackerStatus/P2PPStatus logs or serial streams |
| `gnss trimble-info` | Inspect Trimble GSOF GENOUT Type 1/2/8 packets from file or serial input |
| `gnss skytraq-info` | Inspect SkyTraq binary epoch/raw/rawx/ack logs or serial streams |
| `gnss binex-info` | Inspect BINEX big-endian regular-CRC metadata/navigation/prototyping records from file or serial input |
| `gnss qzss-l6-info` | Inspect direct QZSS L6 250-byte frames and optionally export data-part, subframe, subtype `10` service-info packets, Compact SSR message, or sampled correction CSV from subtype `1/2/3/4/5/6/7/8/9/11/12` |
| `gnss convert` | Convert RTCM or UBX input into simple RINEX files and export UBX SFRBX frame metadata |
| `gnss replay` | Replay rover/base data from RINEX, UBX, or RTCM through the RTK solver |
| `gnss stats` | Text summary for `.pos` results |
| `gnss plot` | Time-series and ENU plots for one or two `.pos` files |
| `gnss compare` | Compare libgnss++ output against RTKLIB output |
| `gnss trackplot` | 2D trajectory comparison with status colors |
| `gnss rtklib2pos` | Convert raw RTKLIB `.pos` into libgnss++ `.pos` format |
| `gnss pos2kml` | Convert libgnss++ or raw RTKLIB `.pos` into KML |
| `gnss driving-compare` | Build the README comparison figure from solution files |
| `gnss scorecard` | Build the Odaiba scorecard image |
| `gnss social-card` | Build a Twitter-ready Odaiba social card image |
| `gnss odaiba-benchmark` | Run the full Odaiba benchmark pipeline end-to-end |
| `gnss odaiba-scan` | Scan Odaiba in fixed epoch windows and dump per-window metrics |
| `gnss short-baseline-signoff` | Run the mixed-GNSS Tsukuba short-baseline static sign-off and emit summary JSON |
| `gnss rtk-kinematic-signoff` | Run the bundled mixed-GNSS RTK kinematic sign-off and emit summary JSON |
| `gnss ppp-static-signoff` | Run the bundled static PPP sign-off and emit summary JSON |
| `gnss ppp-kinematic-signoff` | Run the bundled kinematic PPP sign-off against an RTK reference and emit summary JSON |
| `gnss clas-ppp` | Run PPP with a named CLAS/MADOCA correction profile over RTCM, compact sampled transport, or raw QZSS L6 subtype `1/2/3/4/5/6/7/8/9/11/12` input and emit summary JSON |
| `gnss ros2-solution-node` | Publish a `.pos` solution file as ROS2 `NavSatFix`, `PoseStamped`, `Path`, status, and satellite-count topics |

`gnss_compare`, `gnss_plot`, `gnss_trackplot`, and `gnss_pos2kml` accept raw RTKLIB `.pos` output directly.

`gnss solve` now exposes `--iono est` in addition to `off/iflc`, which enables a float RTK path with DD ionosphere states in the Kalman filter. The current `EST` path is intended as a robustness mode and regression target rather than a fixed-ambiguity mode.

```bash
python3 apps/gnss.py solve \
  --data-dir data/driving \
  --out output/rtk_solution.pos \
  --kml output/rtk_solution.kml \
  --mode kinematic
```

```bash
python3 apps/gnss.py ppp \
  --static \
  --obs data/rover_static.obs \
  --nav data/navigation_static.nav \
  --sp3 precise.sp3 \
  --clk precise.clk \
  --antex igs20.atx \
  --blq station.blq \
  --out output/ppp_solution.pos
```

For synthetic or dry-run regression inputs without atmospheric delay, add `--no-estimate-troposphere`.

For precise-product ambiguity fixing experiments, add `--enable-ar`. The current PPP-AR path is validated on synthetic precise-product regressions, and the bundled real static sample now has a `20 epoch` regression slice that reaches `PPP fixed solutions = 2` with generated `SP3/CLK` and `--ar-ratio-threshold 1.5`.

For broadcast-nav PPP experiments with external orbit/clock corrections, `gnss ppp` accepts either a simple SSR CSV file via `--ssr` or sampled RTCM SSR input via `--ssr-rtcm`. `--ssr-rtcm` can point at a local file or an `ntrip://`, `tcp://`, or `serial://` RTCM source. The CSV format is `week,tow,sat,dx,dy,dz,dclock_m`. The RTCM path now decodes SSR `1/2/3/4/5/6` payloads for GPS/GLONASS/Galileo/QZSS/BeiDou through `RTCMProcessor::decodeSSRCorrections()`. Orbit/clock payloads are converted into sampled PPP corrections, matching SSR high-rate clock payloads are folded into the sampled clock correction, SSR URA is carried into PPP measurement weighting, and SSR code-bias is applied to PPP pseudorange observations.

You can also point it at explicit files and override solver knobs:

```bash
python3 apps/gnss.py solve \
  --rover data/driving/rover.obs \
  --base data/driving/base.obs \
  --nav data/driving/navigation.nav \
  --out output/custom_solution.pos \
  --format llh \
  --ratio 3.0 \
  --min-ar-sats 5 \
  --elevation-mask-deg 15 \
  --base-ecef -3962108.7 3381309.5 3668678.8
```

```bash
python3 apps/gnss.py stream --input correction.rtcm3 --output relay.rtcm3 --limit 100
python3 apps/gnss.py stream --input serial:///dev/ttyUSB1?baud=115200 --limit 10
python3 apps/gnss.py stream --input tcp://127.0.0.1:9000 --limit 10
python3 apps/gnss.py stream --input correction.rtcm3 --output serial:///dev/ttyUSB1?baud=115200 --limit 10
python3 apps/gnss.py stream --input correction.rtcm3 --output tcp://127.0.0.1:9000 --limit 10
python3 apps/gnss.py stream --input ntrip://user:pass@caster.example.com:2101/MOUNT --decode-observations --limit 10
python3 apps/gnss.py ubx-info --input logs/session.ubx --decode-nav --decode-observations --obs-rinex-out output/session.obs
python3 apps/gnss.py ubx-info --input serial:///dev/ttyACM0?baud=115200 --decode-observations --limit 10
python3 apps/gnss.py nmea-info --input logs/session.nmea --decode-gga --decode-rmc
python3 apps/gnss.py nmea-info --input serial:///dev/ttyUSB0?baud=9600 --limit 10
python3 apps/gnss.py novatel-info --input logs/novatel.log --decode-bestpos --decode-bestvel
python3 apps/gnss.py novatel-info --input logs/novatel.bin --decode-bestpos --decode-bestvel
python3 apps/gnss.py novatel-info --input serial:///dev/ttyUSB0?baud=115200 --limit 10
python3 apps/gnss.py sbp-info --input logs/session.sbp --decode-time --decode-pos --decode-vel
python3 apps/gnss.py sbp-info --input serial:///dev/ttyUSB0?baud=115200 --limit 10
python3 apps/gnss.py sbf-info --input logs/session.sbf --decode-pvt --decode-lband --decode-p2pp
python3 apps/gnss.py sbf-info --input serial:///dev/ttyUSB0?baud=115200 --limit 10
python3 apps/gnss.py trimble-info --input logs/session.gsof --decode-time --decode-llh --decode-vel
python3 apps/gnss.py trimble-info --input serial:///dev/ttyUSB0?baud=115200 --limit 10
python3 apps/gnss.py skytraq-info --input logs/session.stq --decode-epoch --decode-raw --decode-ack
python3 apps/gnss.py skytraq-info --input serial:///dev/ttyUSB0?baud=115200 --limit 10
python3 apps/gnss.py binex-info --input logs/session.bnx --decode-metadata --decode-nav --decode-proto
python3 apps/gnss.py binex-info --input serial:///dev/ttyUSB0?baud=115200 --limit 10
python3 apps/gnss.py qzss-l6-info --input logs/qzss_l6.bin --show-preview --extract-data-parts output/qzss_l6_frames.csv --extract-subframes output/qzss_l6_subframes.csv --extract-compact-messages output/qzss_l6_messages.csv --extract-compact-corrections output/qzss_l6_compact.csv --gps-week 2200
python3 apps/gnss.py qzss-l6-info --input serial:///dev/ttyUSB0?baud=115200 --limit 10
python3 apps/gnss.py nav-products --obs data/rover_static.obs --nav data/navigation_static.nav --sp3-out output/static_products.sp3 --clk-out output/static_products.clk --max-epochs 60
python3 apps/gnss.py ppp-static-signoff --enable-ar --generate-products --ar-ratio-threshold 1.5 --require-mean-error-max 5.0 --require-max-error-max 6.0 --require-ppp-fixed-epochs-min 1 --require-ppp-solution-rate-min 100
python3 apps/gnss.py convert --format ubx --input logs/session.ubx --obs-out output/session.obs
python3 apps/gnss.py convert --format ubx --input logs/session.ubx --nav-out output/session.nav  # GPS/QZSS/Galileo/GLONASS/BeiDou RXM-SFRBX
python3 apps/gnss.py convert --format ubx --input logs/session.ubx --sfrbx-out output/session_sfrbx.csv
python3 apps/gnss.py convert --format ubx --input serial:///dev/ttyACM0?baud=115200 --obs-out output/stream.obs --limit 10
python3 apps/gnss.py convert --format rtcm --input tcp://127.0.0.1:9000 --obs-out output/correction.obs --limit 10
python3 apps/gnss.py convert --format rtcm --input correction.rtcm3 --obs-out output/correction.obs --nav-out output/correction.nav
python3 apps/gnss.py replay --rover-rinex data/rover_kinematic.obs --base-rinex data/base_kinematic.obs --nav-rinex data/navigation_kinematic.nav --out output/replay.pos
python3 apps/gnss.py live --rover-ubx serial:///dev/ttyACM0 --rover-ubx-baud 115200 --base-rtcm correction.rtcm3 --nav-rinex data/driving/navigation.nav --out output/live.pos
python3 apps/gnss.py live --rover-rtcm tcp://127.0.0.1:9001 --base-rtcm tcp://127.0.0.1:9000 --nav-rinex data/driving/navigation.nav --out output/live.pos
python3 apps/gnss.py rcv start --config configs/live.example.conf --status-out output/receiver_status.json --log-out output/receiver.log
python3 apps/gnss.py rcv status --status-out output/receiver_status.json --wait-seconds 5
python3 apps/gnss.py rcv status --status-out output/receiver_status.json --tail-log-lines 20
python3 apps/gnss.py rcv restart --config configs/live.example.conf --status-out output/receiver_status.json --wait-seconds 1
python3 apps/gnss.py rcv reload --status-out output/receiver_status.json --wait-seconds 1
python3 apps/gnss.py rcv stop --status-out output/receiver_status.json
python3 apps/gnss.py rinex-info --count-records data/driving/rover.obs
python3 apps/gnss.py stats output/rtk_solution.pos
python3 apps/gnss.py compare output/rtk_solution.pos output/driving_rtklib_rtk.pos
python3 apps/gnss.py plot output/rtk_solution.pos output/driving_rtklib_rtk.pos
python3 apps/gnss.py trackplot output/rtk_solution.pos output/driving_rtklib_rtk.pos
python3 apps/gnss.py rtklib2pos output/driving_rtklib_rtk.pos output/driving_rtklib_converted.pos
python3 apps/gnss.py pos2kml output/driving_rtklib_rtk.pos output/driving_rtklib_rtk.kml --status non-spp
python3 apps/gnss.py driving-compare --lib-pos output/rtk_solution.pos --rtklib-pos output/driving_rtklib_rtk.pos --reference-csv data/driving/Tokyo_Data/Odaiba/reference.csv --output docs/driving_odaiba_comparison.png
python3 apps/gnss.py scorecard --lib-pos output/rtk_solution.pos --rtklib-pos output/driving_rtklib_rtk.pos --reference-csv data/driving/Tokyo_Data/Odaiba/reference.csv --output docs/driving_odaiba_scorecard.png
python3 apps/gnss.py social-card --lib-pos output/rtk_solution.pos --rtklib-pos output/driving_rtklib_rtk.pos --reference-csv data/driving/Tokyo_Data/Odaiba/reference.csv --output docs/driving_odaiba_social_card.png
python3 apps/gnss.py odaiba-benchmark --rtklib-bin /path/to/rnx2rtkp --summary-json output/odaiba_summary.json
python3 apps/gnss.py odaiba-benchmark --rtklib-bin /path/to/rnx2rtkp --malib-bin /path/to/malib/rnx2rtkp --summary-json output/odaiba_summary.json
python3 apps/gnss.py odaiba-scan --glonass-ar autocal --window-size 1000 --step 1000 --output-csv output/odaiba_window_scan.csv
python3 apps/gnss.py short-baseline-signoff --summary-json output/short_baseline_summary.json --max-epochs 120 --require-fix-rate-min 95 --require-mean-error-max 0.15 --require-max-error-max 0.6 --require-mean-sats-min 14
python3 apps/gnss.py rtk-kinematic-signoff --summary-json output/rtk_kinematic_summary.json --max-epochs 120 --require-valid-epochs-min 120 --require-fix-rate-min 95 --require-mean-error-max 3.0 --require-max-error-max 3.0 --require-mean-sats-min 25
python3 apps/gnss.py ppp-static-signoff --summary-json output/ppp_static_summary.json --max-epochs 120 --require-valid-epochs-min 120 --require-mean-error-max 1.5 --require-max-error-max 1.5 --require-mean-sats-min 6.0 --require-ppp-solution-rate-min 100
python3 apps/gnss.py ppp-kinematic-signoff --summary-json output/ppp_kinematic_summary.json --max-epochs 120 --require-common-epoch-pairs-min 120 --require-reference-fix-rate-min 95 --require-mean-error-max 75 --require-p95-error-max 145 --require-max-error-max 190 --require-mean-sats-min 25 --require-ppp-solution-rate-min 100
python3 apps/gnss.py ppp-static-signoff --malib-bin /path/to/MALIB/rnx2rtkp --summary-json output/ppp_static_summary.json
python3 apps/gnss.py ppp-kinematic-signoff --malib-bin /path/to/MALIB/rnx2rtkp --summary-json output/ppp_kinematic_summary.json
python3 apps/gnss.py clas-ppp --profile madoca --obs data/rover_static.obs --nav data/navigation_static.nav --ssr-rtcm ntrip://caster/MOUNT --out output/madoca_ppp.pos --summary-json output/madoca_ppp_summary.json
python3 apps/gnss.py clas-ppp --profile clas --obs data/rover_static.obs --nav data/navigation_static.nav --compact-ssr corrections.compact.csv --out output/clas_ppp.pos --summary-json output/clas_ppp_summary.json
python3 apps/gnss.py clas-ppp --profile clas --obs data/rover_static.obs --nav data/navigation_static.nav --qzss-l6 logs/qzss_l6.bin --qzss-gps-week 2200 --out output/clas_l6.pos --summary-json output/clas_l6_summary.json
```

Use the benchmark as a sign-off gate with explicit thresholds:

```bash
python3 apps/gnss.py odaiba-benchmark \
  --rtklib-bin /path/to/rnx2rtkp \
  --summary-json output/odaiba_summary.json \
  --require-all-epochs-min 11000 \
  --require-common-epoch-pairs-min 8000 \
  --require-lib-all-p95-h-max 8.0 \
  --require-lib-common-median-h-max 0.8 \
  --require-lib-common-p95-h-max 6.5
```

`gnss stream` now covers the low-level RTCM ingest and relay path from file, NTRIP, serial, or TCP sources, and can relay frames back out to file, serial, or TCP sinks. `gnss convert` covers the first practical `convbin`-style export path for RTCM/UBX logs, `gnss nmea-info` adds a lightweight decoder family for `NMEA GGA/RMC` file or serial inspection, `gnss novatel-info` adds a proprietary raw-family inspect path for `NovAtel ASCII/Binary BESTPOS/BESTVEL`, `gnss sbp-info` adds Swift Binary Protocol `GPS_TIME / POS_LLH / VEL_NED`, `gnss sbf-info` adds Septentrio SBF `PVTGeodetic / LBandTrackerStatus / P2PPStatus` file or serial inspection, `gnss trimble-info` adds Trimble GSOF `Type 1/2/8` GENOUT packet inspection for `file / serial`, `gnss skytraq-info` adds SkyTraq binary `0xDC/0xDD/0xE5/0x83/0x84` epoch/raw/rawx/ack inspection for `file / serial`, `gnss binex-info` adds BINEX `0x00/0x01/0x7F` metadata/navigation/prototyping record inspection for `file / serial`, `gnss qzss-l6-info` adds direct QZSS L6 frame inspection plus bit-accurate 5-part subframe export, Compact SSR message inventory, and minimal sampled correction export for 250-byte messages, `gnss nav-products` adds a small bridge from real observation epochs plus broadcast nav into minimal `SP3/CLK` files, `gnss replay` provides an offline rover/base replay solver path, `gnss live` covers RTCM or UBX rover input against RTCM base corrections with inline station/nav metadata recovery, short-gap base interpolation, and short hold-last-base operation for real-time correction lag, and `gnss rcv` adds an `rtkrcv`-style config-file wrapper with foreground `run` plus `start/restart/reload/status/stop` control backed by JSON status snapshots, waitable status polling, optional log-tail inspection, and an interactive `console` command loop for the common receiver operations. `scripts/run_odaiba_comparison.sh` remains as a compatibility wrapper, but it now delegates to `gnss odaiba-benchmark`.

`gnss odaiba-benchmark` emits a machine-readable summary JSON alongside the PNG artifacts so the checked-in benchmark snapshot can be compared without scraping console output. If `--malib-bin` or an existing `--malib-pos` is provided, the same summary also records MALIB metrics for regression tests.

`gnss short-baseline-signoff` does the same for the mixed-GNSS Tsukuba short-baseline dataset using rover/base header coordinates as the static reference.

`gnss rtk-kinematic-signoff` does the same for the bundled mixed-GNSS `rover_kinematic/base_kinematic/navigation_kinematic` sample. That dataset is effectively static in practice, so the sign-off compares the kinematic RTK solution against the rover header position and keeps a stable regression gate on fix rate, mean/max drift, and satellite count.

`gnss ppp-static-signoff` does the same for the bundled sample static dataset using the rover header coordinates as the reference.

`gnss ppp-kinematic-signoff` uses the bundled `rover_kinematic/base_kinematic/navigation_kinematic` sample and compares broadcast-nav kinematic PPP against a matching RTK reference trajectory generated from the same data. The bundled `kinematic` sample is still much weaker than RTK in absolute accuracy, so this sign-off is intended as a regression gate rather than a final-quality benchmark.

Both PPP sign-off entrypoints can optionally run MALIB `rnx2rtkp` on the same epoch window by passing `--malib-bin /path/to/MALIB/rnx2rtkp`. The summary JSON then records both the libgnss++ metrics and the MALIB sidecar metrics using the bundled `scripts/malib_ppp_static.conf` / `scripts/malib_ppp_kinematic.conf` broadcast-nav templates.

The current broadcast-nav PPP internals are now partially aligned to MALIB on the bundled samples: the static `120 epoch` sign-off still lands at `1.432 m mean/max` versus MALIB `2.155 / 3.136 m`, while the kinematic `120 epoch` sign-off improved from `10863.754 / 19023.914 / 23488.517 m` (`mean / p95 / max`) to `69.684 / 142.042 / 187.010 m` after matching RTKLIB-style measurement variance, preserving the bundled broadcast-nav ambiguity handling, and splitting the receiver clock state so GLONASS can absorb its own broadcast PPP time offset. This is a large regression improvement, but it is still not MALIB-equivalent on the bundled kinematic sample (`19.302 / 20.708 / 21.711 m`).

`gnss clas-ppp` is the named correction entry point for CLAS/MADOCA-style PPP runs. It can wrap `gnss ppp --ssr-rtcm` for RTCM-carried SSR, `--compact-ssr` for a compact sampled correction source, or `--qzss-l6` for direct raw QZSS L6 frames. The compact format is `week,tow,system,prn,dx,dy,dz,dclock_m[,high_rate_clock_m][,ura_sigma_m=<m>][,cbias:<id>=<m>...][,pbias:<id>=<m>...][,atmos_<name>=<value>...]`; the wrapper expands it into a temporary sampled SSR CSV and records the selected profile, transport, and correction encoding in the summary JSON. The current direct L6 path decodes subtype `1/2/3/4/5/6/7/8/9/11/12` in-tree and feeds orbit/clock, URA, code-bias, phase-bias, STEC polynomial, gridded atmospheric metadata, and reference-grid atmospheric corrections into the same sampled correction path, while subtype `10` service information can be reassembled by `gnss qzss-l6-info`.

The core library now also carries a precise-products-backed `PPP_FLOAT` path. `SP3`/`CLK` loading, interpolation, static/kinematic `gnss ppp` modes, `gnss ppp-static-signoff`, `gnss ppp-kinematic-signoff`, and synthetic regression coverage in `tests/test_ppp.cpp` are in place. The PPP troposphere model now uses in-tree Niell hydrostatic/wet mapping, a latitude/season/height zenith climatology seed, and a full GPT-style modeled slant delay for the `estimate_troposphere=false` path instead of the old blended secant approximation, while keeping the RTK/SPP short-baseline path unchanged. Dual-frequency kinematic PPP cycle-slip detection now uses `loss_of_lock + geometry-free + Melbourne-Wubbena` checks instead of relying on loss-of-lock alone. The PPP geophysical path now also applies an in-tree approximate solid-Earth tide correction driven by Sun/Moon ephemeris, receiver ANTEX PCO via `gnss ppp --antex`, and site-specific ocean loading from `gnss ppp --blq station.blq` using BLQ harmonic coefficients selected by the observation `MARKER NAME` or `--ocean-loading-station`. The bundled static sample now signs off at `PPP solution rate = 100%` with `mean/max position error = 1.432 m`. `gnss nav-products` now also lets the bundled real static sample exercise the precise-products path with generated `SP3/CLK`, and the current stable regression runs `120/120` valid epochs with `0` fallback epochs. Real-data PPP-AR is now signed off in `gnss ppp-static-signoff`: the bundled `120 epoch` static sample can run with `--enable-ar --generate-products --ar-ratio-threshold 1.5` and currently passes at `PPP fixed epochs >= 1`, `PPP solution rate = 100%`, `mean error <= 5.0 m`, `max error <= 6.0 m`, and `fallback = 0`. Minimal RTCM SSR `1/2/3/4/5/6` decoding and `RTCM SSR file/NTRIP/TCP/serial -> sampled PPP correction` application are now in place, `gnss clas-ppp` now supports RTCM-carried, compact sampled, and direct QZSS L6 subtype `1/2/3/4/5/6/7/8/9/11/12` CLAS/MADOCA transport/application paths, and `gnss qzss-l6-info` covers direct QZSS L6 frame ingest at the header/data-part level plus 5-part subframe assembly/export, subtype `10` service-information packet assembly/export, Compact SSR message/correction export, subtype `8/9/12` atmospheric export, and direct atmospheric application into PPP on the sampled SSR path using the official CLAS grid definition.

### Requirements

- C++17 compiler (GCC 7+, Clang 6+)
- CMake 3.14+
- Eigen3
- Python 3 development headers
- pybind11

### Install And Package

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
cmake --install build --prefix /opt/libgnsspp

# pkg-config
PKG_CONFIG_PATH=/opt/libgnsspp/lib/pkgconfig pkg-config --cflags --libs libgnsspp

# CMake package
cmake -S consumer_app -B consumer_build -DCMAKE_PREFIX_PATH=/opt/libgnsspp

# API docs (if Doxygen is installed)
cmake --build build --target doc_doxygen

# package artifacts
cd build && cpack -G TGZ && cpack -G DEB
```

The install prefix preserves the current dispatcher layout:
- `bin/` for `gnss`, compiled CLI tools, and Python helper entrypoints
- `lib/pythonX.Y/site-packages/libgnsspp/` for the minimal Python bindings package
- `tools/` for plotting/stat/compare helpers
- `scripts/` for benchmark/report generators
- `configs/` for sample live receiver configs

The current `libgnsspp` Python package now covers inspection, coordinate conversion, common post-processing helpers, and file-based `SPP/PPP/RTK` solving helpers:

```python
import libgnsspp

header = libgnsspp.read_rinex_header("data/rover_static.obs")
epochs = libgnsspp.read_rinex_observation_epochs("data/rover_static.obs", max_epochs=2)
spp = libgnsspp.solve_spp_file("data/rover_static.obs", "data/navigation_static.nav", max_epochs=3)
ppp = libgnsspp.solve_ppp_file("data/rover_static.obs", "data/navigation_static.nav", max_epochs=5)
rtk = libgnsspp.solve_rtk_file(
    "data/short_baseline/TSK200JPN_R_20240010000_01D_30S_MO.rnx",
    "data/short_baseline/TSKB00JPN_R_20240010000_01D_30S_MO.rnx",
    "data/short_baseline/BRDC00IGS_R_20240010000_01D_MN.rnx",
    max_epochs=10,
)
solution = libgnsspp.load_solution("output/rtk_solution.pos")
last = solution.last_solution()
fixed_only = solution.filter_by_status(libgnsspp.SolutionStatus.PPP_FIXED)
ecef = libgnsspp.geodetic_deg_to_ecef((35.0, 139.0, 10.0))
```

If ROS2 Jazzy-compatible dependencies are installed, the build also produces `gnss_solution_node`, which replays a `.pos` file onto ROS2 `NavSatFix`, `PoseStamped`, `Path`, solution-status, and satellite-count topics:

```bash
python3 apps/gnss.py ros2-solution-node --ros-args -p solution_file:=output/rtk_solution.pos -p max_messages:=1
```

### Run Tests

```bash
# Unit tests
./build/tests/run_tests

# Regression tests (requires test data in data/)
bash tests/run_regression.sh
```

## Analysis Tools

```bash
# Generate a libgnss++ RTK solution from RINEX
python3 apps/gnss.py solve --data-dir data/driving --out output/rtk_solution.pos

# Inspect and relay RTCM
python3 apps/gnss.py stream --input correction.rtcm3 --output relay.rtcm3 --limit 100

# Inspect UBX and export RAWX to a simple RINEX observation file
python3 apps/gnss.py ubx-info --input logs/session.ubx --decode-nav --decode-observations --obs-rinex-out output/session.obs

# Convert RTCM or UBX into simple RINEX files
python3 apps/gnss.py convert --format ubx --input logs/session.ubx --obs-out output/session.obs
python3 apps/gnss.py convert --format ubx --input logs/session.ubx --nav-out output/session.nav
python3 apps/gnss.py convert --format rtcm --input correction.rtcm3 --obs-out output/correction.obs --nav-out output/correction.nav

# Replay rover/base observations through the RTK solver
python3 apps/gnss.py replay --rover-rinex data/rover_kinematic.obs --base-rinex data/base_kinematic.obs --nav-rinex data/navigation_kinematic.nav --out output/replay.pos

# Quick statistics
python3 apps/gnss.py stats output/rtk_solution.pos

# Visualization (matplotlib)
python3 apps/gnss.py plot output/rtk_solution.pos

# Compare with RTKLIB
python3 apps/gnss.py compare output/rtk_solution.pos output/driving_rtklib_rtk.pos

# 2D trajectory plot
python3 apps/gnss.py trackplot output/rtk_solution.pos output/driving_rtklib_rtk.pos

# Convert raw RTKLIB output into libgnss++ POS format
python3 apps/gnss.py rtklib2pos output/driving_rtklib_rtk.pos output/driving_rtklib_converted.pos

# Convert POS to KML
python3 apps/gnss.py pos2kml output/rtk_solution.pos output/rtk_solution_from_pos.kml

# Regenerate README comparison assets directly
python3 apps/gnss.py driving-compare --lib-pos output/rtk_solution.pos --rtklib-pos output/driving_rtklib_rtk.pos --reference-csv data/driving/Tokyo_Data/Odaiba/reference.csv --output docs/driving_odaiba_comparison.png
python3 apps/gnss.py scorecard --lib-pos output/rtk_solution.pos --rtklib-pos output/driving_rtklib_rtk.pos --reference-csv data/driving/Tokyo_Data/Odaiba/reference.csv --output docs/driving_odaiba_scorecard.png
python3 apps/gnss.py social-card --lib-pos output/rtk_solution.pos --rtklib-pos output/driving_rtklib_rtk.pos --reference-csv data/driving/Tokyo_Data/Odaiba/reference.csv --output docs/driving_odaiba_social_card.png

# Regenerate the UrbanNav Odaiba comparison image
python3 apps/gnss.py odaiba-benchmark --rtklib-bin /path/to/rnx2rtkp
python3 apps/gnss.py odaiba-benchmark --rtklib-bin /path/to/rnx2rtkp --malib-bin /path/to/malib/rnx2rtkp

# Scan Odaiba by window to find weak multi-GNSS segments
python3 apps/gnss.py odaiba-scan --glonass-ar autocal --window-size 1000 --step 1000 --output-csv output/odaiba_window_scan.csv
```

## Test Data

| Directory | Description | Source |
|-----------|-------------|--------|
| `data/` (symlinks) | Active test dataset | — |
| `data/rover_static.obs` | Static 3.3 km baseline | Sample |
| `data/rover_kinematic.obs` | Kinematic 1.2 km | [geofis/ppk](https://github.com/geofis/ppk) |
| `data/short_baseline/` | Static 36 m (Tsukuba IGS) | BKG GNSS Data Center |
| `data/driving/` | Urban driving 6.5 km (Tokyo Odaiba) | [UrbanNavDataset](https://github.com/IPNL-POLYU/UrbanNavDataset) |

## Reference Links

- [JAXA-SNU/MALIB](https://github.com/JAXA-SNU/MALIB) — RTKLIB fork focused on MADOCA-PPP, PPP-AR, and L6E-enabled real-time/post-process workflows. This is the current external reference for Phase 5 PPP / CLAS-MADOCA work.
- [QZSS-Strategy-Office/claslib](https://github.com/QZSS-Strategy-Office/claslib) — official CLAS reference implementation. `data/clas_grid.def` is mirrored from here and drives the nearest-grid atmospheric application path for direct QZSS L6 subtype `12`.
- [QZSS L6 Signal Interface Specification IS-QZSS-L6-004](https://qzss.go.jp/en/technical/download/pdf/ps-is-qzss/is-qzss-l6-004.pdf) — official direct QZSS L6 header/data-part structure used for `gnss qzss-l6-info`
- [Swift Navigation libsbp navigation docs](https://swift-nav.github.io/libsbp/c/build/docs/html/group__navigation.html) — official SBP `GPS_TIME / POS_LLH / VEL_NED` layouts used for `gnss sbp-info`
- [Septentrio AsteRx SB3 Pro+ Reference Guide](https://www.septentrio.com/system/files/support/asterx_sb3_pro_firmware_v4.10.1_reference_guide.pdf) — official SBF block layout used for `gnss sbf-info` (`PVTGeodetic`, `LBandTrackerStatus`, `P2PPStatus`)
- [Trimble R780 GSOF Messages Overview](https://help.fieldsystems.trimble.com/r780/gsof-messages-overview.htm) — official Trimble GSOF message index used for `gnss trimble-info` (`Type 1/2/8`)
- [SkyTraq AN0028 Binary Messages Of SkyTraq Venus 8 GNSS Receiver](https://www.skytraq.com.tw/homesite/AN0028_1.4.44.pdf) — official SkyTraq binary epoch/raw/ack message framing used for `gnss skytraq-info`
- [SkyTraq AN0039 Raw Measurement Data Extension Of SkyTraq Phoenix GNSS Receiver](https://www.skytraq.com.tw/homesite/AN0039.pdf) — official SkyTraq `RAWX` layout used for `gnss skytraq-info`

## Algorithm Overview

### RTK Processing Flow

1. **SPP** — initial position via weighted least squares
2. **SD bias init** — single-difference carrier phase bias from phase−code
3. **DD observation model** — double-difference with geodist + Sagnac + troposphere
4. **Kalman filter** — Eigen-based EKF with SD ambiguity states
5. **LAMBDA** — integer least-squares ambiguity resolution (LD + reduction + search)
6. **WL-NL AR** — wide-lane/narrow-lane for long baselines (Melbourne-Wubbena + IF combination)
7. **Fix-and-hold** — maintain integer fix across epochs with direct state constraint

### Key Design Decisions

- **SD parameterization** — single-difference ambiguity states (DD formed in observation model)
- **Separate rover/base satellite positions** — computed from respective pseudoranges (matches RTKLIB satposs)
- **Analytical Sagnac** — `geodist()` with `OMGE*(rs[0]*rr[1]-rs[1]*rr[0])/c` (no rotation matrix)
- **Position-based hold validation** — accept low-ratio fixes when position is consistent with last fix

## License

MIT License — see LICENSE file.
