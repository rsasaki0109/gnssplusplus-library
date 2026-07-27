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

Follow-up candidates (not pursued now): baseline-length gate on the
adaptation (mirror `snr_min_baseline_m`), phase-only adaptation, per-system
alpha.

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

### Gate B result

_pending_

## C. Offline GNSS-IMU time-offset search

_pending — `ImuSeries::shiftTime` + `gnss_fuse --imu-time-offset` +
`scripts/search_imu_time_offset.py`, J(dt) = mean squared KF position
correction over INS-time-update epochs, coarse (100 ms) -> fine (20 ms)._
