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

The RTK half takes three wrong-fix guards from `gnss solve`, all off by
default:

- `--hold-kalman-update` applies the held integers as a Kalman
  pseudo-observation instead of overwriting the ambiguities.
- `--hold-kalman-update-validated-only` (with `--hold-kalman-update`) applies it
  only after a validated ambiguity-resolution FIX, not after the hold-fix
  fallback.
- `--min-fix-satellites <k>` refuses an ambiguity-resolution FIX with fewer
  than k non-reference satellites.

With `--navi776-tc` on the six PPC runs, all three together cut the wrong-FIX
rate from 7.08% to 2.00% of FIX. FIX within 0.5 m fall by 0.91 pp of epochs,
but FIX within 0.25 m rise from 62.93% to 65.90%
([measurement](ppc_rtk_vs_gnss_imu_fusion.md#kf-hold-only-after-validated-fixes)).
On UrbanNav Tokyo they also lower the wrong-FIX rate. On Odaiba, however, they
hold one wrong fix for 13 s, 12 m off horizontally and 44 m in height
([UrbanNav check](ppc_rtk_vs_gnss_imu_fusion.md#urbannav-out-of-sample-check)).

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

With `--navi776-tc` (or `--tc-velocity-states`), use the RTK stream written by
`--rtk-pos-out`. The fused `--out` stream can be kilometres off on the PPC
Tokyo runs, because the loose ESKF takes a wrong RTK velocity state
([known issue](navi776_techniques.md#known-issue-the-velocity-state-2026-10-31)).
Without any tight-coupling flag, PPC nagoya/run1 is kilometres off too: there
the loose ESKF's gyro bias estimate grows to 0.23 rad/s and its heading does
not converge. `--consider-before-heading-latch` fixes most of that
([below](#the-loose-eskf-before-the-heading-latch)).

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

### The loose ESKF before the heading latch

Until the heading latches, the yaw σ is 180°, so the error-state
linearization about the nominal yaw is not valid. By default the GNSS, ZUPT and
NHC updates of that phase still correct roll, pitch and both IMU biases. The
latch then reopens only the yaw, and the tilt and biases learned before it
stay, with a small and wrong covariance.

- **What goes wrong.** On 7 of the 11 runs below, the gyro bias estimate
  reaches 0.09–0.23 rad/s, and the heading then drifts. On tokyo/run1,
  tokyo/run2 and nagoya/run1 the jump comes within seconds of the latch. This
  is the mechanism found for the online PVA filter
  ([candidate v11](online_pva_candidate_v11.md)).
- **What the flag does.** `--consider-before-heading-latch` makes those
  pre-latch updates Schmidt consider updates for the attitude and both biases:
  position and velocity are corrected, the attitude and biases are left
  unchanged, and the covariance stays consistent (Joseph form). After the
  latch every update is the plain update. The option is the one
  `velocity_consistency_v10` of `gnss pva-evaluate` sets; `gnss fuse` exposes
  it as an opt-in.
- **nagoya/run1 starts by reversing.** The car backs out of a parking space
  for about 8 s at up to 1.2 m/s, and the course samples agree, so the
  heading latches 180° off. The flag does not fix this wrong latch: with it,
  nagoya/run1 still reaches 0.117 rad/s and a p90 of 72.5 m. The heading-latch direction test of
  `velocity_consistency_v5` flips nagoya/run1 correctly but also flips the
  correct forward latches on Odaiba and HK Deep-Urban-1, so it is not
  exposed in `gnss fuse`.

Fused `--out` 3D error, p50 / p90 (m), one run each, the same RTK flags
(`--preset low-cost --ratio 2.4 --max-subset-ar-drop-steps 18
--rtk-snr-weighting --no-arfilter`) and no tight coupling. The largest gyro
bias estimate of the run is in parentheses (rad/s).

| Run | Default | `--consider-before-heading-latch` |
| --- | ---: | ---: |
| PPC tokyo/run1 | 2.43 / 60.3 (0.161) | 0.12 / 9.4 (0.012) |
| PPC tokyo/run2 | 0.12 / 1.6 (0.088) | 0.10 / 1.6 (0.010) |
| PPC tokyo/run3 | 0.09 / 2.3 (0.158) | 0.08 / 2.3 (0.008) |
| PPC nagoya/run1 | 2,044 / 13,274 (0.229) | 1.60 / 72.5 (0.117) |
| PPC nagoya/run2 | 0.63 / 15.4 (0.025) | 0.63 / 15.2 (0.024) |
| PPC nagoya/run3 | 1.86 / 16.7 (0.013) | 1.81 / 16.7 (0.021) |
| UrbanNav Odaiba | 0.39 / 17.8 (0.152) | 0.27 / 17.8 (0.005) |
| UrbanNav Shinjuku | 0.53 / 19.5 (0.097) | 0.32 / 15.5 (0.024) |
| UrbanNav HK Deep-Urban-1 | 5.98 / 57.5 (0.101) | 3.21 / 56.4 (0.006) |
| UrbanNav HK Harsh-Urban-1 | 66.2 / 479 (0.023) | 140.7 / 265 (0.028) |
| UrbanNav HK Medium-Urban-1 | 4,659 / 39,329 (0.050) | 48.8 / 193 (0.038) |

- The PPC runs use the FLU lever arms, UrbanNav a zero lever arm, and
  UrbanNav Tokyo `--base-ecef -3961904.9530 3348993.7578 3698211.7553`.
  The Hong Kong error is to the raw truth, about 0.6 m from the antenna.
- The RTK stream (`--rtk-pos-out`) is byte-identical with and without the
  flag on all 11 runs.
- HK Harsh-Urban-1 is worse at p50. Its RTK stream fixes 2.5% of the
  epochs and 92% of those fixes are wrong, so its fused output is unusable
  either way.
- All 11 runs are development data: PPC and UrbanNav Tokyo were used to find
  the mechanism, and the table was measured after it. There is no holdout, so
  the default stays off.
- **Do not combine it with `--tc-closed-loop` or `--navi776-tc`.** The
  closed-loop INS starts from the loose ESKF's attitude and biases, so the
  flag changes the RTK stream. On one run per PPC run under `--navi776-tc`,
  correct FIX fall from 68.21% to 65.10% and the wrong-FIX rate rises from
  6.41% to 9.53%.

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
near-neutral safety evidence, not adoption evidence. The full-six and
blocked-span ablation driver used for this comparison was never committed to
the repository, so there is no in-tree reproduction script for it.

The complementary RTK-hosted tightly-coupled path is specified in
[`tight_coupling.md`](tight_coupling.md). It keeps baseline, ambiguity, and AR
ownership in `RTKProcessor`, and feeds opt-in INS propagation and measurement
updates into that existing lifecycle.
