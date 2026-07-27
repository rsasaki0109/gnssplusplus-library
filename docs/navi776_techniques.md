# navi.776 techniques (Motooka 2026, GSDC 2023-24 winner)

Introduction of three techniques from "Optimized GNSS/INS Integration with
Time Synchronization for High-Accuracy Smartphone Positioning" (NAVIGATION
Vol. 73, navi.776) into the KF/RTK path. Branch:
`feature/navi776-gnss-ins-techniques`. All knobs default-OFF and
bit-identical when off (verified: tokyo run1 first 1200 epochs, md5-equal
.pos against the pre-change binary).

Cycle-slip detection and NIS gating from the paper were not ported — this
library's existing implementations are already a superset.

## A. Innovation-based adaptive measurement variance

`RTKConfig::enable_adaptive_measurement_noise` (+ `--rtk-adaptive-noise`
on gnss_solve / gnss_fuse / ppc-demo).

Paper recursion `R <- a*R + (1-a)*(vv' - HPH')`, diagonal only, applied at
the **single-difference** level: the per-satellite EWMA of
`v^2 - HPH_ii - ref_var` replaces the model satellite variance fed into
`buildDoubleDifferenceCovariance`, clamped to
`[min_scale, max_scale] * model_variance` (model = varerr x SNR x NLOS, so
elevation trends stay the backbone). Reference-side variance stays at the
model value, keeping the DD block's correlated part known and the block PD
by construction. Keying is per satellite/frequency/kind, so the memory
survives reference switches. alpha: phase 0.9, code 0.5 (paper values).
Slip resets the phase entry; reacquisition clears the tracker; gate-rejected
epochs never adapt.

### Gate A result (2026-07-27, same binary, full runs, canonical flags*)

| run | variant | fix% | 50cm-matched% | official% | p95_h m |
|---|---|---|---|---|---|
| tokyo1 | OFF | 86.71 | 82.63 | 68.97 | 1.381 |
| tokyo1 | ON (paper defaults) | 84.78 | 84.38 | 69.33 | 1.891 |
| tokyo1 | ON2 (max-scale 4) | 84.37 | 82.89 | 68.90 | 1.857 |
| tokyo3 | OFF | 81.82 | 87.09 | 77.54 | 0.883 |
| tokyo3 | ON | **86.12** | **90.37** | 77.36 | 0.998 |
| tokyo3 | ON2 | 85.45 | 89.45 | 77.23 | 1.147 |
| nagoya1 | OFF | 86.42 | 80.34 | 54.12 | 0.755 |
| nagoya1 | ON | 76.15 | 79.95 | 51.44 | 1.869 |
| nagoya1 | ON2 | 75.91 | 79.81 | 51.17 | 1.848 |

\* `ppc-demo --solver rtk --preset low-cost --ratio 2.4
--max-subset-ar-drop-steps 18 --rtk-snr-weighting --no-arfilter
--max-epochs -1`

**Verdict: mixed-negative as a blanket default — flag stays OFF.**
tokyo3 is a clear win (fix +4.30 pp, 50cm +3.28 pp), tokyo1 trades fix for
50cm score, nagoya1 collapses (fix −10.3 pp, p95_h +147%). The one
permitted retune (max_variance_scale 25 → 4) did not rescue nagoya and
slightly hurt tokyo, so the ceiling is not the driver: on the 9.4 km
MEIJOBASE baseline the innovation stream contains real DD model error
(ionosphere), which the tracker faithfully absorbs into R, loosening the
filter exactly where the model needs to stay strict. The mechanism matches
the paper's context — short smartphone baselines with device-quality noise
as the dominant innovation source. Usable as an opt-in for short-baseline
urban runs; do not enable on long baselines.

### A3 follow-up: baseline-length gate (`adaptive_noise_max_baseline_m`)

`--rtk-adaptive-noise-max-baseline <m>` (0 = no gate): adaptation is
active only while the float baseline is at or below the threshold,
evaluated on the prior state each epoch. With the gate at 1000 m
(same binary, full runs, vs the same OFF baselines as gate A):

| run | variant | fix% | 50cm-matched% | official% | p95_h m | wall s |
|---|---|---|---|---|---|---|
| tokyo1 | OFF | 86.71 | 82.63 | 68.97 | 1.381 | 73.5 |
| tokyo1 | gated ON | **87.16** | 82.26 | 68.38 | 1.381 | 83.6 |
| tokyo3 | OFF | 81.82 | 87.09 | 77.54 | 0.883 | 218.1 |
| tokyo3 | gated ON | **83.09** | **88.57** | 77.56 | 0.880 | 234.1 |
| nagoya1 | gated ON | bit-identical to OFF (md5-equal .pos) | | | | |

**Verdict: PASSES the gate bar on all three runs** — fix +0.45 pp
(tokyo1) / +1.27 pp (tokyo3) / unchanged (nagoya1), p95_h flat
everywhere, nagoya1 exactly neutralized. The gate also fires on tokyo
SPP-reseed excursions (transient float baselines > 1 km), which
incidentally stops the tracker from learning during resets — that is why
gated tokyo1 beats ungated ON on fix by 2.4 pp. Cost: the ungated 50cm
gains shrink (tokyo1 +1.75 -> -0.37 pp; tokyo3 +3.28 -> +1.48 pp), and
wall +7-14%% from the extra bookkeeping. Recommended opt-in:
`--rtk-adaptive-noise --rtk-adaptive-noise-max-baseline 1000`.
The feature remains default-OFF pending a user decision on making this
combination a preset.

Other follow-up candidates (not pursued): phase-only adaptation,
per-system alpha.

## B. SD Doppler observation rows (velocity observability)

`RTKConfig::enable_doppler_measurement_rows` (+ gnss_fuse
`--tc-doppler-rows`, `--tc-doppler-sigma`; requires `--tc-velocity-states`).

Rover-only between-satellite SD Doppler rows in the RTK measurement update,
same observation model as FGO's opt-in `SingleDifferenceDopplerFactor`
(sigma 0.2 m/s). Receiver clock drift cancels in the between-satellite
difference; rows touch only the M4 velocity tail states (force-active mask
already in place), never ambiguities/AR/slip/locks. Doppler rows carry a
per-row outlier threshold (m/s domain) instead of the metre-domain scalar.
Rows are skipped until the first INS position/velocity time update
initializes the velocity covariance.

### Gate B result (2026-07-27, same binary, full runs)

Both arms `--tc-closed-loop --tc-velocity-states` + canonical RTK knobs +
per-city lever arm; scored on the `--rtk-pos-out` stream. OFF numbers
reproduce docs/tight_coupling.md's M4 table exactly (77.36/75.75/7.53 on
tokyo1), confirming measurement consistency.

| run | variant | fix% | 50cm-matched% | official% | p95_h m |
|---|---|---|---|---|---|
| tokyo1 | OFF (M4) | 77.36 | 75.75 | 70.07 | 7.53 |
| tokyo1 | ON (sigma 0.2) | 78.24 | 74.17 | 71.36 | 10.05 |
| tokyo1 | ON2 (sigma 0.5) | **78.39** | **76.20** | **72.66** | 8.23 |
| tokyo3 | OFF | 78.52 | 79.23 | 74.69 | 4.69 |
| tokyo3 | ON | 76.11 | 72.80 | 69.96 | 11.01 |
| tokyo3 | ON2 | 79.50 | 77.53 | 73.85 | 6.97 |
| nagoya1 | OFF | 78.27 | 74.39 | 55.46 | 9.66 |
| nagoya1 | ON | 78.14 | 69.26 | 53.30 | 10.37 |
| nagoya1 | ON2 | 73.27 | 63.81 | 45.91 | 9.92 |

**Verdict: mixed-negative as a blanket default — flag stays OFF.**
The paper sigma (0.2 m/s) over-constrains automotive urban Doppler:
50cm/p95 regress everywhere. The retune (sigma 0.5) turns tokyo1 into a
near-clean win (fix +1.03 pp, 50cm +0.46 pp, official +2.59 pp; only p95_h
+9% fails the bar) and improves tokyo3 fix (+0.98 pp) at small accuracy
cost, but nagoya1 regresses badly under both sigmas. Pattern mirrors gate
A: on the 9.4 km baseline the extra velocity coupling propagates
model-error-driven velocity into the position/ambiguity block through the
INS reanchor loop. Usable as an opt-in on short-baseline runs (sigma 0.5
recommended over the paper's 0.2); do not enable on long baselines.

## C. Offline GNSS-IMU time-offset search

Machinery: `ImuSeries::shiftTime` (week-rollover-safe), `gnss_fuse
--imu-time-offset <s>` (applied at load, 0.0 = guarded exact no-op, OFF
bit-identity verified), per-run `imu_time_offset_score:` stdout line
(J(dt) = mean squared Kalman position correction accumulated ONLY on
INS-time-update epochs), and `scripts/search_imu_time_offset.py`
(coarse 100 ms -> fine 20 ms, coverage guard, boundary warning;
`--coarse-fine off` gives the paper-faithful 101-candidate sweep).

### Gate C result (2026-07-27, 3000-epoch prefixes, closed-loop coupling)

| window | argmin dt | J spread across candidates | shape |
|---|---|---|---|
| tokyo run1 | -0.32 s | 0.063 vs 0.16 m^2 | bimodal noise (two solution branches, no convexity) |
| tokyo run3 | -0.30 s | 0.0143-0.0166 m^2 (1.16x) | flat |
| nagoya run1 | -0.90 s | 0.504-0.665 m^2 (1.32x) | flat, argmin near boundary |

**Verdict: null result — no reliable time offset detectable; the applied
offset stays 0.0.** The pre-registered acceptance criteria (locally convex
J around the argmin, two-window agreement) fail on every window: the
tokyo1 curve is a bimodal fix-branch lottery, tokyo3/nagoya1 are flat to
within run-to-run noise. Two causes, both expected in hindsight: (1) the
PPC-Dataset's Septentrio mosaic-X5 + tactical IMU logging is
hardware-synchronized, so the true offset is ~0 and there is no signal to
find; (2) the M3 closed loop re-anchors the INS at RTK's own posterior
every epoch, so a mistimed IMU only perturbs one ~0.2 s mechanization
interval per update — J barely responds even to a +/-1 s shift. The
paper's method needs an unanchored GNSS/INS EKF over device-quality data
(its smartphone context) to make J sharply dt-sensitive.

### ESKF / precomputed-solution extension (2026-07-27)

The adapter and non-RTK scoring path are now implemented:

- `gnss_fuse --gnss-pos <solution.pos>` feeds a precomputed LibGNSS++ or
  RTKLIB position/velocity stream directly to `LooseCouplingProcessor`; raw
  rover/base observations and navigation data are not required.
- `Solution::loadFromFile()` accepts RTKLIB calendar-time LLH rows and
  converts their full NEU position/velocity covariance to ECEF.
- `--imu-format rtklibexplorer-sf` loads the upstream GPST-referenced Unix
  timestamps, converts acceleration g to m/s2, and preserves gyro rad/s.
  `--imu-misalignment-rpy-deg` applies upstream `Euler_to_CTM` followed by
  FRD-to-FLU conversion.
- In this path, J uses only the accepted GNSS **position-update injection**,
  captured before the same-epoch velocity update. This avoids contaminating
  the position-correction objective with velocity-update cross-covariance.
  `--imu-time-offset-score-start-epoch` permits a warm-up prefix followed by
  a disjoint scoring window.
- `search_imu_time_offset.py` accepts `--gnss-pos`, `--imu`, the format and
  mounting arguments while retaining its original RTK-data-dir interface.

Validation used upstream `rtklibexplorer/GNSS_IMU` at commit
`a6d74e83f35636859c2cb9a3a397df39bd044a07`. These logs are on a common
GPST axis, but upstream documents a fixed logging-delay correction of
-0.125 s for `drive_0708` and 0 s for `walk_0827`, providing known targets.

| run / scoring window | expected dt | grid | argmin dt | result |
|---|---:|---:|---:|---|
| drive epochs 0-999 | -0.125 s | 50 ms | -0.100 s | pass, 25 ms error |
| drive epochs 1000-1999 | -0.125 s | 50 ms | -0.250 s | fail, route-dependent minimum |
| walk epochs 0-267 | 0.000 s | 50 ms | 0.000 s | pass |
| walk epochs 268-535 | 0.000 s | 50 ms | 0.000 s | pass |
| walk full run | 0.000 s | 50 ms | 0.000 s | pass |

The walk two-window acceptance criterion passes exactly, and the drive first
window recovers the documented nonzero delay within 25 ms. The drive second
window does not reproduce it: J instead bottoms near -0.25 s. Therefore the
extension is a **partial positive** demonstrating that the machinery can
recover both zero and nonzero known offsets, but the original safeguards
remain mandatory: never accept an offset without local shape and independent
window agreement. The drive failure also shows that an ESKF correction-norm
objective can remain motion/model dependent even without RTK re-anchoring.

Example (drive first window):

```bash
python scripts/search_imu_time_offset.py \
  --gnss-pos ../data/rtklibexplorer_gnss_imu/drive_0708/gnss_1934_sf.pos \
  --imu ../data/rtklibexplorer_gnss_imu/drive_0708/imu_1934_sf.csv \
  --imu-format rtklibexplorer-sf \
  --imu-misalignment-rpy-deg=179.71,-6.60,185.35 \
  --lever-arm 0,0.05,0 --coupling-flags= \
  --offset-min -0.2 --offset-max 0 --offset-step 0.05 \
  --coarse-fine off --max-epochs 1000 \
  --out-dir output/navi776_eskf_offset_drive_w1
```

## Combined configuration (A3 gate + B sigma 0.5 on the M4 closed loop)

`--tc-closed-loop --tc-velocity-states --tc-doppler-rows
--tc-doppler-sigma 0.5 --rtk-adaptive-noise
--rtk-adaptive-noise-max-baseline 1000`, same binary, full runs, scored
on the `--rtk-pos-out` stream (OFF arms rerun; they reproduced the gate B
OFF numbers exactly):

| run | variant | fix% | 50cm-matched% | official% | p95_h m |
|---|---|---|---|---|---|
| tokyo1 | OFF (M4) | 77.36 | 75.75 | 70.07 | 7.53 |
| tokyo1 | combined | **79.19** | **76.00** | **72.24** | **7.38** |
| tokyo2 | OFF (M4) | 77.58 | 82.39 | 84.03 | 2.95 |
| tokyo2 | combined | **85.09** | **84.85** | **84.57** | 3.18 |
| tokyo3 | OFF | 78.52 | 79.23 | 74.69 | 4.69 |
| tokyo3 | combined | 78.69 | 77.07 | 72.68 | 7.68 |
| nagoya1 | OFF / combined (md5 equal) | 78.27 | 74.39 | 55.46 | 9.66 |
| nagoya2 | OFF / combined (md5 equal) | 54.18 | 53.85 | 39.59 | 27.81 |

tokyo1 is a clean all-metric win (fix +1.83 pp, official +2.16 pp, p95_h
improved) — the best tokyo1 tight-coupling result recorded in this repo.
Tokyo2 is a strong fix/threshold win (fix +7.50 pp, 50cm +2.46 pp,
official +0.55 pp), but p95_h regresses by 0.24 m (8.0%). Its parallel
A/B wall time was 702.3 s OFF vs 833.9 s ON (+18.8%), also failing the
pre-registered <=+5% wall bar; nagoya2, where the gates disarm, was
414.6 s vs 417.0 s (+0.58%). Tokyo3 fails (accuracy/p95 regress, dominated by the Doppler-row
component, which was already negative on tokyo3 accuracy in gate B).

Across all three short-baseline Tokyo runs, fix rate improves every time
(+1.83/+7.50/+0.17 pp), while 50cm and official improve on run1/run2 but
regress on run3, and p95 improves only on run1. Across both 9.4 km Nagoya
runs, the scored RTK stream is bit-identical. Verdict: the result is not a
tokyo1-only accident and is a promising short-baseline fix-rate booster,
but it remains a run-dependent opt-in rather than a blanket preset because
tail accuracy and active-path runtime are not consistently safe. The
adaptive-noise gate composes cleanly with Doppler rows (the DOPPLER-kind
adaptation path exercised here for the first time, no instability observed).

The whole combo is exposed as one flag: `gnss_fuse --navi776-tc`
(closed loop + velocity states + Doppler rows sigma 0.5 gated 1000 m +
adaptive noise gated 1000 m; flags given after it override). The Doppler
rows carry their own baseline gate (`--tc-doppler-max-baseline`,
`RTKConfig::doppler_row_max_baseline_m`), so on nagoya run1 (9.4 km)
both components disarm and the `--navi776-tc` RTK stream is
**bit-identical to the M4 OFF baseline (md5-equal .pos)** — the combo is
provably harmless on long baselines and needs no per-run opt-out. Nagoya
run2 independently reproduces this result (both RTK files MD5
`30d61358688fd1a34f5f71b14c3e2803`; every score identical).
