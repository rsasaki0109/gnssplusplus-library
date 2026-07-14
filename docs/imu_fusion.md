# GNSS/IMU Fusion (`gnss fuse`)

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

Add `--tight-dd-imu` (requires `--base`) to augment the propagated INS state
with real rover/base double-difference rows and innovation-gated soft recovery.
Code DD is committed; carrier/ambiguity candidates are evaluated in shadow and
fall back to code-only without partial AR. `--tight-dd-carrier-experimental`
commits carrier, ambiguity, and partial-AR updates for research ablations. It
is deliberately separate because PPC long-prefix validation found that
apparently accepted carrier updates could make the covariance indefinite.
Without either flag the established loose-coupling path is unchanged.

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
| Tight DD row adapter | `RTKProcessor::formTightlyCoupledObservations` |
| Augmented INS/ambiguity update | `src/fusion/dd_imu_bridge.cpp` |
| Doppler LS velocity (feeds SPP/RTK solutions) | `src/algorithms/spp_velocity.cpp` |
| CLI | `apps/native/gnss_fuse.cpp` (registered as `fuse` in `apps/gnss.py`) |

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
the NIS gate defaults were tuned on PPC data. The reusable Stage-2 core now
lives in `fusion/dd_imu_bridge`: it accepts a mechanized 15-state INS state,
maintains live DD ambiguity states and cross-covariances, applies joint code and
carrier updates, performs quality-ordered partial LAMBDA, and replaces hard SPP
reseed with a gated soft update/covariance inflation. The opt-in
`gnss fuse --tight-dd-imu` epoch loop evaluates the existing RTK DD machinery
at the propagated INS position, converts the ECEF Jacobian into the fixed local
ENU frame, and applies the bridge update while reporting row, gate, PAR, and
soft-reset diagnostics. The validation table above predates that opt-in path
and remains loose-coupling evidence only.

A C++20/MSVC real-data integration smoke on the first 50 Tokyo/run1 epochs
verified the operational committed-carrier path, but a 600-epoch replay exposed
catastrophic carrier-tail instability (p95 above 149 km). The default was
therefore changed to safe shadow-carrier evaluation. On the same 600 epochs,
baseline versus safe-shadow p50/p95/p99 ECEF error was
1.171/3.480/27.031 m versus 1.123/3.480/30.447 m; 301 epochs used the
carrier-to-code fallback, partial AR and soft resets were both zero. This is
near-neutral safety evidence, not adoption evidence; use
`experiments/run_tight_dd_imu_ablation.py` for the full-six and blocked-span
comparison.
