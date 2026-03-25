# libgnss++ — Modern C++ GNSS Positioning Library

Self-contained RTK/SPP positioning library in modern C++17. No RTKLIB runtime dependency.

## Performance vs RTKLIB

Tested on 3 open datasets with GPS L1+L2 dual-frequency observations.

| Dataset | Baseline | libgnss++ Fix Rate | RTKLIB Fix Rate | libgnss++ RMS (h) | RTKLIB RMS (h) |
|---------|----------|--------------------|-----------------|--------------------|-----------------|
| **Kinematic** | 1.2 km | **100%** | 98.3% | **12 mm** | 8 mm |
| **Short static** | 36 m | **99%** | 100% | 90 mm | — |
| **Long static** | 3.3 km | 52% | 60.8% | 104 mm | 14 mm* |

\* RTKLIB の long static 結果は RINEX header 概略位置と比較すると高さ方向に 2.2m のオフセットがあり、libgnss++ の方が概略位置に近い結果 (16cm) を示す場合がある。

### Kinematic Result Comparison

![Kinematic RTK: libgnss++ vs RTKLIB](docs/kinematic_comparison.png)

### Urban Driving Result Comparison

Open driving dataset: [UrbanNav Tokyo Odaiba](https://github.com/IPNL-POLYU/UrbanNavDataset) (2018-12-19, Trimble rover/base, ~170 m baseline). The checked-in benchmark artifacts compare the current libgnss++ Odaiba pipeline against a checked-in RTKLIB forward-kinematic baseline on the same open dataset.

![UrbanNav Odaiba benchmark scorecard](docs/driving_odaiba_scorecard.png)

Current Odaiba snapshot: on all matched epochs, libgnss++ leads on matched-epoch count, fix rate, p95 horizontal error, and max horizontal error. On common epochs only, the horizontal median is within 27 mm of RTKLIB and the vertical median is slightly better for libgnss++.

![Urban driving comparison on UrbanNav Tokyo Odaiba](docs/driving_odaiba_comparison.png)

**Highlights:**
- Instant first fix (epoch 1) on all datasets
- Kinematic fix rate exceeds RTKLIB (100% vs 98.3%)
- Zero external GNSS library dependency (RTKLIB not required at runtime)
- Wide-lane/Narrow-lane AR enables long baseline ambiguity resolution

## Features

- **RTK positioning** with carrier phase ambiguity resolution (LAMBDA)
- **SPP positioning** with Klobuchar ionosphere and Saastamoinen troposphere
- **Wide-lane / Narrow-lane AR** for long baseline ionosphere mitigation
- **Fix-and-hold** ambiguity maintenance with position-based validation
- **RINEX 2/3 reader** (GPS observation, navigation, multi-GNSS header parsing)
- **Eigen-based Kalman filter** (no C array marshalling)
- **Zero external GNSS dependency** (self-contained LAMBDA, troposphere, ionosphere models)

## Architecture

```
libgnss++/
  core/         constants, coordinates, types, observation, navigation, solution
  models/       troposphere (Saastamoinen), ionosphere (Klobuchar)
  algorithms/   spp, rtk, kalman, lambda
  io/           rinex, solution_writer
```

## Quick Start

```cpp
#include <libgnss++/gnss.hpp>

int main() {
    // Setup RTK processor
    libgnss::RTKProcessor rtk;
    libgnss::RTKProcessor::RTKConfig config;
    config.ambiguity_ratio_threshold = 3.0;
    rtk.setRTKConfig(config);

    // Load data
    libgnss::io::RINEXReader rover, base, nav_reader;
    rover.open("rover.obs");
    base.open("base.obs");
    nav_reader.open("navigation.nav");

    libgnss::NavigationData nav;
    nav_reader.readNavigationData(nav);

    // Set base position from RINEX header
    libgnss::io::RINEXReader::RINEXHeader base_header;
    base.readHeader(base_header);
    rtk.setBasePosition(base_header.approximate_position);

    // Process epoch by epoch
    libgnss::ObservationData rover_obs, base_obs;
    while (rover.readObservationEpoch(rover_obs) && base.readObservationEpoch(base_obs)) {
        auto solution = rtk.processRTKEpoch(rover_obs, base_obs, nav);
        if (solution.status == libgnss::SolutionStatus::FIXED) {
            // cm-level position available
        }
    }
}
```

## Building

```bash
mkdir build && cd build
cmake ..
make -j$(nproc)
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
| `gnss solve` | Batch RTK post-processing from RINEX rover/base/nav |
| `gnss rinex-info` | Inspect RINEX headers and optionally count epochs / ephemerides |
| `gnss stream` | Read RTCM from file or NTRIP, print message summaries, and relay frames |
| `gnss ubx-info` | Inspect UBX NAV/RAWX logs or serial streams and optionally export RAWX epochs to RINEX |
| `gnss convert` | Convert RTCM, NTRIP, or UBX file/serial input into simple RINEX observation/navigation files |
| `gnss replay` | Replay rover/base data from RINEX, UBX, or RTCM through the RTK solver |
| `gnss stats` | Text summary for `.pos` results |
| `gnss plot` | Time-series and ENU plots for one or two `.pos` files |
| `gnss compare` | Compare libgnss++ output against RTKLIB output |
| `gnss trackplot` | 2D trajectory comparison with status colors |
| `gnss rtklib2pos` | Convert raw RTKLIB `.pos` into libgnss++ `.pos` format |
| `gnss pos2kml` | Convert libgnss++ or raw RTKLIB `.pos` into KML |
| `gnss driving-compare` | Build the README comparison figure from solution files |
| `gnss scorecard` | Build the Odaiba scorecard image |
| `gnss odaiba-benchmark` | Run the full Odaiba benchmark pipeline end-to-end |
| `gnss odaiba-scan` | Scan Odaiba in fixed epoch windows and dump per-window metrics |

`gnss_compare`, `gnss_plot`, `gnss_trackplot`, and `gnss_pos2kml` accept raw RTKLIB `.pos` output directly.

```bash
python3 apps/gnss.py solve \
  --data-dir data/driving \
  --out output/rtk_solution.pos \
  --kml output/rtk_solution.kml \
  --mode kinematic
```

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
python3 apps/gnss.py stream --input ntrip://user:pass@caster.example.com:2101/MOUNT --decode-observations --limit 10
python3 apps/gnss.py ubx-info --input logs/session.ubx --decode-nav --decode-observations --obs-rinex-out output/session.obs
python3 apps/gnss.py ubx-info --input serial:///dev/ttyACM0?baud=115200 --decode-observations --limit 10
python3 apps/gnss.py convert --format ubx --input logs/session.ubx --obs-out output/session.obs
python3 apps/gnss.py convert --format ubx --input serial:///dev/ttyACM0?baud=115200 --obs-out output/stream.obs --limit 10
python3 apps/gnss.py convert --format rtcm --input correction.rtcm3 --obs-out output/correction.obs --nav-out output/correction.nav
python3 apps/gnss.py replay --rover-rinex data/rover_kinematic.obs --base-rinex data/base_kinematic.obs --nav-rinex data/navigation_kinematic.nav --out output/replay.pos
python3 apps/gnss.py live --rover-ubx serial:///dev/ttyACM0 --rover-ubx-baud 115200 --base-rtcm correction.rtcm3 --nav-rinex data/driving/navigation.nav --out output/live.pos
python3 apps/gnss.py rcv start --config configs/live.example.conf --status-out output/receiver_status.json --log-out output/receiver.log
python3 apps/gnss.py rcv status --status-out output/receiver_status.json --wait-seconds 5
python3 apps/gnss.py rcv restart --config configs/live.example.conf --status-out output/receiver_status.json --wait-seconds 1
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
python3 apps/gnss.py odaiba-benchmark --rtklib-bin /path/to/rnx2rtkp
python3 apps/gnss.py odaiba-scan --glonass-ar autocal --window-size 1000 --step 1000 --output-csv output/odaiba_window_scan.csv
```

`gnss stream` now covers the low-level RTCM ingest and relay path from file, NTRIP, or serial sources, `gnss convert` covers the first practical `convbin`-style export path for RTCM/UBX logs, `gnss replay` provides an offline rover/base replay solver path, `gnss live` covers RTCM or UBX rover input against RTCM base corrections with inline station/nav metadata recovery, short-gap base interpolation, and short hold-last-base operation for real-time correction lag, and `gnss rcv` adds an `rtkrcv`-style config-file wrapper with foreground `run` plus `start/restart/status/stop` control backed by JSON status snapshots and waitable status polling. `scripts/run_odaiba_comparison.sh` remains as a compatibility wrapper, but it now delegates to `gnss odaiba-benchmark`.

### Requirements

- C++17 compiler (GCC 7+, Clang 6+)
- CMake 3.14+
- Eigen3

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

# Regenerate the UrbanNav Odaiba comparison image
python3 apps/gnss.py odaiba-benchmark --rtklib-bin /path/to/rnx2rtkp

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
