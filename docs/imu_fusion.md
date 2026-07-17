# GNSS/IMU Loosely-Coupled Fusion (`gnss fuse`)

Loosely-coupled GNSS/IMU integration: a 15-state error-state EKF
(position, velocity, attitude, accel bias, gyro bias) propagates 100 Hz
strapdown mechanization and corrects with the library's own RTK (or SPP)
solutions plus Doppler-derived velocity. Design insights are borrowed from
[inuex35/tightly-coupled-gnss-imu-fgo](https://github.com/inuex35/tightly-coupled-gnss-imu-fgo)
(two-phase pipeline, ZUPT/NHC constraints, lever-arm handling), re-expressed
as a pure C++/Eigen ESKF with no external estimator dependency.

## Usage

```bash
gnss fuse \
  --rover data/PPC-Dataset/tokyo/run1/rover.obs \
  --base  data/PPC-Dataset/tokyo/run1/base.obs \
  --nav   data/PPC-Dataset/tokyo/run1/base.nav \
  --imu   data/PPC-Dataset/tokyo/run1/imu.csv \
  --lever-arm 0.31,0,0.55 \
  --preset low-cost \
  --out output/fused.pos
```

Omitting `--base` falls back to SPP as the GNSS half. `--attitude-csv`
exports per-epoch roll/pitch/yaw for analysis.

Key flags: `--lever-arm x,y,z` (IMU→antenna, body FLU), `--zupt`/`--no-zupt`
(default on), `--nhc`/`--no-nhc` (default off), `--imu-grade`,
`--max-position-nis`/`--max-velocity-nis`/`--max-consecutive-gate-rejections`
(innovation gating), heading-alignment window/threshold knobs (see `--help`).

## IMU input

`imu.csv` is parsed by header name (whitespace/case/punctuation-insensitive),
matching the convention of `scripts/analyze_ppc_imu_coverage.py`:
GPS TOW + week, accel in m/s², angular rate in deg/s (converted to rad/s at
load). Body-axis convention is explicit (`ImuAxisConvention`), defaulting to
FLU/Z-up, which is what the PPC-Dataset IMU actually logs (verified: stationary
Acc Z ≈ +9.81, despite the upstream README saying FRD).

## Architecture

| Piece | Location |
|---|---|
| IMU CSV loader | `include/libgnss++/io/imu.hpp`, `src/io/imu.cpp` |
| Strapdown mechanization, attitude | `src/fusion/mechanization.cpp`, `attitude.cpp` |
| Process noise / discretization | `src/fusion/fusion_process_noise.cpp` |
| Measurement models (pos/vel/ZUPT/NHC, lever arm) | `src/fusion/fusion_measurement.cpp` |
| Dense Joseph-form update + NIS gate | `src/fusion/fusion_update.cpp` |
| Static + heading alignment | `src/fusion/fusion_initialization.cpp` |
| Per-epoch driver | `src/fusion/fusion_processor.cpp` (`LooseCouplingProcessor`) |
| Doppler LS velocity (feeds SPP/RTK solutions) | `src/algorithms/spp_velocity.cpp` |
| CLI | `apps/gnss_fuse.cpp` (registered as `fuse` in `apps/gnss.py`) |

The fusion deliberately does not reuse `algorithms/kalman.hpp`: its
RTKLIB-style "active state" convention (`x[i] != 0`) is incompatible with an
error state that is legitimately zero after injection.

Heading alignment latches only after a trailing window of GNSS course
samples agrees within a circular-scatter tolerance (a single-epoch latch was
shown to catch reversing vehicles ~180° wrong on PPC nagoya). The reported
"heading converged" status requires alignment plus healthy recent
velocity-innovation NIS.

## Validation snapshot (PPC Tokyo/Nagoya, `--preset low-cost`)

Scored against the 5 Hz Applanix reference. Representative results:

| Run | Availability (RTK→fused) | H-RMSE (RTK→fused) | Roll/Pitch RMSE | Yaw median err |
|---|---|---|---|---|
| tokyo/run1 | 84.8% → 99.7% | 5.7 m → 7.4 m | 0.5° / 1.9° | 0.9° |
| nagoya/run2 | 76.8% → 99.9% | 6.3 m → 14.2 m | 1.1° / 1.7° | 1.7° |

Fusion trades some average accuracy (IMU dead-reckoning drift during long
urban outages) for near-total availability: ~99% of reference epochs get a
solution vs 69–92% for RTK alone, with ~94–100% of RTK outage epochs bridged.
Fixed-epoch accuracy is not degraded.

Known limits: consumer/tactical-grade dead-reckoning drifts tens of meters
through multi-minute outages; vertical error remains weaker than horizontal;
the NIS gate defaults were tuned on PPC data. The RTK-hosted tightly-coupled
path is specified in [`tight_coupling.md`](tight_coupling.md). The older
`include/libgnss++/fusion/dd_imu_bridge.hpp` declarations are retained only as
the historical parallel-filter sketch that preceded that design.
