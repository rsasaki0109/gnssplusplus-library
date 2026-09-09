# Phase 55 Pixel5 ADR-uncertainty identifiability audit

Phase 55 is a truth-free audit of the one raw physical factor selected by
Phase 54: Android per-satellite `AccumulatedDeltaRangeUncertaintyMeters`.  It
tests whether the source-defined per-pair uncertainty predicts the existing
eligible TDCP closure residual across four route-disjoint Pixel5 captures.
No correction, solver rerun, truth read, navigation read, coordinate input, or
metric payload read was allowed.

The freeze was pushed before any Phase 55 raw read in `575ae30`; its SHA-256
is `cc303962265d5814f613a06db793e181ebaf5a9a2727d7903ef31ee65741cf2a`.
The evaluator, focused tests, CMake registration, and manifest were then
sealed and pushed in `534b350`; the manifest SHA-256 is
`19c844e3a9c9c3b5d34b3a7b90f1639574c499a07aa8d87f0a440da6d4e36e8a`.

## Fixed raw-only contract

The Android API defines `AccumulatedDeltaRangeUncertaintyMeters` as the
absolute, single-sided 1-sigma uncertainty of one accumulated delta range, in
meters.  The official reference is
[`GnssMeasurement.getAccumulatedDeltaRangeUncertaintyMeters()`](https://developer.android.com/reference/android/location/GnssMeasurement#getAccumulatedDeltaRangeUncertaintyMeters()).
The audited pair quantity is fixed with no coefficient, clip, or tuning:

```text
u_pair_m = sqrt(u_prev_m^2 + u_current_m^2)
```

For each same-system/SVID/signal adjacent raw pair, the closure residual is

```text
r_pair = (signed_ADR_k - signed_ADR_prev)
         - 0.5 * (PseudorangeRate_prev + PseudorangeRate_k) * dt_seconds
```

Pixel5 keeps the adapter ADR sign.  Ordinary pairs require `0 < dt <= 1.5 s`,
unchanged hardware-clock discontinuity/segment, finite in-range Phase 25
raw-clock pseudorange, clear existing code/C/N0/multipath masks, and ADR
`VALID` with `RESET`/`CYCLE_SLIP` clear.  Pairs missing a positive finite ADR
uncertainty remain ordinary closure accounting but are excluded from the
uncertainty relation; they are never imputed.  The signed residual is centered
by the endpoint `utcTimeMillis + HardwareClockDiscontinuityCount` median of
ordinary pairs, and all relation gates use the absolute centered residual.

The evaluator reports route, signal/frequency, satellite, ADR-state, fixed
uncertainty-bin, and route-quartile groups.  Its predeclared gates require four
routes, at least 1,000 ordinary and uncertainty pairs per route, at least five
satellites, at least three populated fixed bins with 50 pairs each, routewise
Spearman at least 0.20, q4/q1 p95 excess at least 0.02 m and ratio at least
1.25, calibration bounds, and fixed leave-one-route-out materiality.  All
thresholds were sealed before raw input.

## One-shot observations

The four pinned raw files were each read exactly once in one process.  All
routes had the uncertainty header and every ordinary pair had a finite
positive uncertainty, but the population occupied only two of the four fixed
bins (`<=0.01 m` and `0.01–0.1 m`).  Every route contained only one signal /
frequency group, `GALILEO:GAL_E1:1575420000 Hz`.

| Route | Raw rows | Epochs | Ordinary / uncertainty pairs | Satellites | Unsupported rows (informational) | Spearman | q4/q1 p95 excess (m) | q4/q1 ratio | Populated bins | Bin monotonicity |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|
| MTV-a | 87,705 | 2,159 | 7,508 / 7,508 | 7 | 266 | 0.064250 | 0.012871 | 1.246912 | 2 | pass |
| MTV-h | 112,833 | 3,140 | 8,763 / 8,763 | 6 | 435 | 0.116044 | 0.005792 | 1.150682 | 2 | pass |
| LAX-t | 51,243 | 1,466 | 4,176 / 4,176 | 10 | 247 | 0.057336 | 0.009497 | 1.177785 | 2 | **fail** |
| MTV-u | 35,810 | 1,102 | 3,728 / 3,728 | 11 | 137 | 0.159487 | 0.036263 | 2.009300 | 2 | pass |

The route medians of absolute centered closure residual were 0.0306591 m,
0.0168202 m, 0.0354249 m, and 0.0318009 m (aggregate median 0.0312300 m;
route-median MAD 0.00238289 m).  Pairwise route-median distances are retained
in the machine-readable result; they are descriptive only and are not a
correction signal.  Normalized residual medians were 1.77587, 0.880089,
1.89538, and 1.29510.

The fixed-bin gate fails on all routes because only two bins are populated.
The routewise relation gate also fails: all four Spearman values are below
0.20, and the q4/q1 p95 excess and ratio miss the frozen thresholds on MTV-a,
MTV-h, and LAX-t.  Leave-one-route-out materiality happens to pass its less
stringent fixed rule, but it cannot override the routewise and population
failures.  Presentation-integrity checks all pass: pair-reason, state,
signal, satellite, fixed-bin, and quartile counts sum exactly; all four route
medians and all four LOO folds are retained; aggregate recomputation is exact.

The pinned source audit also finds that the current Android adapter parses ADR
meters/state but does not parse `AccumulatedDeltaRangeUncertaintyMeters`, the
`Observation` model does not retain it, and current FGO uses fixed
`tdcp_sigma_m`.  Thus no native sigma-floor implementation was attempted.

## Decision

The result is **no-go-adr-uncertainty-not-identifiable**.  The strongest
finding is that the field is available and complete in these raw captures, but
its pair uncertainty has insufficient fixed-bin support and no stable
routewise material relation to the centered TDCP closure residual.  No native
TDCP sigma floor is authorized.  Phase 43 remains the champion and the Phase
51 option remains experimental.  The `0.782` target is not evaluated without
truth.

The exactly one next source-supported raw factor is
**Android per-satellite `BiasUncertaintyNanos` / receiver-clock uncertainty
relationship**.  This phase does not begin that audit.

## Read accounting and artifacts

The read accounting is one process with four raw GNSS reads (one per route),
and zero raw IMU, truth, navigation, solver, trajectory, coordinate, MAT,
validation/holdout, archive, rematerialization, device-WLS,
`SvPosition`/`SvElevation`, Kaggle/token, or prior Phase 53/54 metric-payload
reads.

The machine-readable record is
[`smartphone_r5_phase55_pixel5_adr_uncertainty_result_v1.json`](records/smartphone_r5_phase55_pixel5_adr_uncertainty_result_v1.json).
The raw audit output is under
`output/smartphone-r5/phase55-pixel5-adr-uncertainty-v1/`:

| Artifact | Bytes | SHA-256 |
|---|---:|---|
| `phase55_pixel5_adr_uncertainty.json` | 2,495,725 | `9e354875018f3ee39542576f10086d90d9795d6bcb29ce8391999eafc3048462` |
| `phase55_pixel5_adr_uncertainty.routes.json` | 2,483,676 | `635b19da4f587523fbf1f6bb5992797b9ef811293aa6241984d8daf15573282b` |
| `phase55_pixel5_adr_uncertainty.manifest.json` | 1,645 | `d70ce015063d60caf795040bb6ef7966383d2319a3f4f5635026552b913f737e` |

The evaluator source SHA-256 is
`65d2af7c26ca17c9654c10588e8e4b78964e06eff4dc58fa80b598e322440314`;
focused test SHA-256 is
`1a39091c984108d583cf95fbdb91a6521f9d36280b0bae2dde5aa4a22b80abf2`; and
the sealed `tests/CMakeLists.txt` SHA-256 is
`1a3bb0567131a05588d37501dc0f9a32caa1ab9b8dbd610799303c4a389821bb`.
The focused Python suite passed 7 tests, `py_compile` passed, and
`--verify-freeze` passed before the raw audit without raw/truth reads.  CMake
registers `python_smartphone_phase55_pixel5_adr_uncertainty_audit_tests`.
