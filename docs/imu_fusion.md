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
  --lever-arm 0.31,0,-0.55 \
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
(default on), `--zupt-gnss-speed-gate`/`--no-zupt-gnss-speed-gate` (default
on), and `--zupt-gnss-speed-threshold-mps` (default 0.5). A last-known GNSS
velocity above the threshold latches ZUPT suppression until a later low-speed
GNSS velocity releases it; stale/missing velocity does not silently re-enable
ZUPT after a moving latch. If no GNSS velocity has ever been available, the
legacy IMU-only detector is preserved. The default-on gate is a safety guard
against the IMU variance/median detector mistaking smooth constant-speed
motion for a stop. `--nhc`/`--no-nhc` (default off), `--imu-grade`,
`--max-position-nis`/`--max-velocity-nis`/`--max-consecutive-gate-rejections`
(innovation gating), heading-alignment window/threshold knobs (see `--help`).

When a GNSS Doppler velocity update is rejected repeatedly, the default
`--max-consecutive-velocity-gate-rejections 3` recovery can re-anchor only the
IMU-origin velocity from the same finite, positive-semidefinite Doppler
covariance. The antenna measurement is converted with
`v_imu = v_antenna - R(omega x lever_arm)`; position, attitude, and biases are
left unchanged, velocity cross-covariances are cleared, and the correction is
bounded by `--max-gnss-velocity-reanchor-mps` (default 20 m/s). A malformed
covariance or an over-bound correction cannot trigger the recovery. This is a
bounded deterministic recovery for velocity-gate lockout, not an ungated EKF
retry; set the patience to 0 to disable it.

Position-gate recovery follows the same separation: only a run of trusted
`FIXED` rejections can trigger the position-only re-anchor, and a `FLOAT` or
SPP epoch resets that patience counter. The application default is 30
consecutive `FIXED` epochs. The default position correction bound is unlimited
(`+infinity`), because a genuine outage can leave the IMU origin tens or
hundreds of metres from a returning, accurate FIX; an application that has an
independent maximum-jump contract can set a finite
`max_fixed_position_reanchor_m` in the library configuration. The reset still
requires finite target/covariance data and changes only position (including
its covariance block); it never retries a rejected EKF update or overwrites
velocity, attitude, or biases.

## Position frame contract

The loose ESKF nominal `position_enu` and `velocity_enu` are always the IMU
origin. `LooseCouplingProcessor::toPositionSolution()` preserves that
IMU-origin contract for internal consumers, including tight-coupling
propagation, priors, and integrity evidence. The GNSS/RTK input and the PPC
`reference.csv` are antenna-frame positions.

External fused `.pos` and KML products emitted by `gnss fuse` use
`toAntennaPositionSolution()`: position is `p_imu + R_body_to_enu * lever_arm`
and velocity is `v_imu + R_body_to_enu * (omega_body x lever_arm)`, then both
are rotated to ECEF. Their position and velocity covariance are the matching
`H P H^T` projections, including the lever-arm attitude Jacobian. The
velocity projection treats the latest bias-corrected gyro sample as an
observed input and therefore does not include a separate gyro measurement
noise term. Do not change internal propagated-solution callers to the
antenna method without separately auditing their frame contract.
The `urban-bridge-score` JSON summary records the same declaration under
`coordinate_frame_contract` so archived metrics remain auditable.

## IMU input

`imu.csv` is parsed by header name (whitespace/case/punctuation-insensitive),
matching the convention of `scripts/analysis/analyze_ppc_imu_coverage.py`:
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

`gnss fuse --help` intentionally shows only everyday inputs, outputs, and
presets. The accepted tuning and research flags remain backward-compatible
and are listed under `gnss fuse --help-advanced`. Prefer `--navi776-tc`
instead of spelling out its component tight-coupling flags.

For repeatable runs, put defaults in a flat TOML `[gnss_fuse]` table and pass
`--config <path>`. Keys may use `snake_case` or `kebab-case`; vector values
such as `lever_arm` and `base_ecef` use three-element arrays. Command-line
options are applied after the file regardless of where `--config` appears, so
they are reliable one-run overrides. See `configs/examples/fuse.example.toml`.

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

The Tokyo/run1 3500-epoch ZUPT A/B confirmed 21 false applications while the
last GNSS speed was 0.529–4.014 m/s (tow 187483.0–187491.0). The moving latch
is therefore retained for safety, although this short A/B did not improve the
trajectory score: gate off/on max bridge was 81.938/102.780 m and fixed-P95
regression was 36.044/36.209 m. This is a safety hardening result, not an
accuracy-pass result; the full run1 gate was not opened from this slice.

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
`scripts/experiments/run_tight_dd_imu_ablation.py` for the full-six and blocked-span
comparison.

The complementary RTK-hosted tightly-coupled path is specified in
[`tight_coupling.md`](tight_coupling.md). It keeps baseline, ambiguity, and AR
ownership in `RTKProcessor`, and feeds opt-in INS propagation and measurement
updates into that existing lifecycle.
