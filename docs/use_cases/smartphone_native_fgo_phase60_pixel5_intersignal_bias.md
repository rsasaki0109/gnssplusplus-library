# Phase60 Pixel5 inter-signal-bias audit

Phase60 is a truth-free, raw-only audit of Android
`FullInterSignalBiasNanos` and `SatelliteInterSignalBiasNanos` on four
route-disjoint Pixel5 recordings.  The audit was sealed before the raw read,
used one evaluator process, and read each `device_gnss.csv` exactly once.
There was no solver or trajectory rerun and no truth, navigation, IMU,
coordinate, WLS, MAT, validation, holdout, archive, Kaggle, or token access.

## Frozen source contract

The source contract is Android API 30's
[GnssMeasurement reference](https://developer.android.com/reference/android/location/GnssMeasurement).
Both fields are signed nanoseconds (`double`) and Android documents the
subtraction sign:

```
corrected pseudorange = raw pseudorange - bias
bias_m = 299792458.0 * bias_nanos * 1e-9
```

`FullInterSignalBiasNanos` is the complete estimated receiver-plus-space
inter-signal bias.  `SatelliteInterSignalBiasNanos` is its space-segment
component.  Therefore `Full - Satellite` is the receiver-side remainder;
`Full + Satellite` is not a valid correction and was prohibited.  The pair
observable was restricted to the same UTC epoch, constellation, and SVID with
two distinct supported signal/frequency groups.  The sign was not fitted from
the recordings.

The raw CSV headers contain both fields in all four routes, but the current
adapter does not parse them, `Observation` does not retain them, and the FGO
path does not consume them.  Existing generic signal-bias states are estimator
state machinery, not evidence of raw Android ISB consumption.  No native
implementation was therefore attempted.

## Result

The frozen result is **no-go** (`phase60-no-go-intersignal-bias-not-identifiable`).
Every route has zero finite Full values, zero finite Satellite values, and zero
pairs with both finite values.  This makes signed materiality, the
Full-minus-Satellite decomposition, temporal stability, and non-common
same-satellite evidence unidentifiable.  Headers, raw proxy coverage, pair
counts, signal composition, source-sign/decomposition accounting, raw hashes,
and presentation integrity passed; the finite-coverage, materiality, and
temporal-stability gates failed.

| route | raw rows | adopted proxy rows | satellites | ISB groups | same-epoch/SVID pairs | finite Full | finite Satellite |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| MTV-a | 87,705 | 83,612 | 37 | 13 | 20,864 | 0 | 0 |
| MTV-h | 112,833 | 108,722 | 31 | 11 | 34,383 | 0 | 0 |
| LAX-t | 51,243 | 50,706 | 29 | 12 | 15,602 | 0 | 0 |
| MTV-u | 35,810 | 35,391 | 26 | 10 | 10,783 | 0 | 0 |

The exactly one next factor is `raw Android pseudorange code tracking /
multipath residual calibration not already masked by the current adapter`.
It is not started by this phase.  Phase43 remains champion; Phase51 and its
later native experiments remain experimental.  The `0.782` target was not
evaluated because Phase60 contains no accuracy truth.

## Frozen gates and accounting

The AND gates were fixed before the first raw read: four routes; exact input
hash/byte accounting; at least 1,000 adopted proxy rows and five satellites
per route; both headers present with at least 80% finite values; at least 100
same-epoch/same-SVID pairs, three paired satellites, two signal-frequency
groups, and two signal families per route; at least 0.03 m signed/remainder
and pair-bias materiality (0.02 m pair correction shift); finite temporal
coverage of at least 80% with at most 5% unexplained jumps; four leave-one-route
out folds; and exact presentation/read accounting.  Unsupported signal rows
were informational and not a gate; no missing ISB value was imputed.

Raw read accounting is: `raw_device_gnss.csv` reads **4 total / 1 per route**,
single process; truth **0**, navigation **0**, solver **0**, trajectory **0**,
IMU **0**, coordinate/WLS/SvPosition/SvElevation **0**, enriched pseudorange
**0**, MAT **0**, validation/holdout **0**, archive reopen/rematerialization
**0**, and Kaggle/token **0**.

The machine-readable result is
[Phase60 result](records/smartphone_r5_phase60_pixel5_intersignal_bias_result_v1.json).
The pre-read freeze is
[Phase60 freeze](records/smartphone_r5_phase60_pixel5_intersignal_bias_freeze_v1.json)
and the sealed evaluator contract is the
[Phase60 evaluator manifest](records/smartphone_r5_phase60_pixel5_intersignal_bias_evaluator_manifest_v1.json).
The immutable output manifest records these artifact hashes:

| artifact | bytes | SHA-256 |
| --- | ---: | --- |
| result | 63,637 | `583671d40e58606587ca52fc8e42dda4a2371e953ddb8eb7e43313a77bc14742` |
| routes | 49,274 | `f4fa9a77c842821e10bf7d7bf153a35bf0903719be9fc6ea85ee0b5c4d6cce3a` |
| events | 161 | `d7582e786e2608ae8a9e8ca4083ccdce94c7624018e860bf977a189b8386a8a1` |
| output manifest | 1,698 | `ba38d8e2e4aa37f1d56ca8ee6791857c798d59e9c1841561a80df0cbb60efc94` |

The freeze SHA-256 is
`d4c0a0ea383801b77cd96083a04dd15c79a6061e79be5e70f5e0859d2340e4a0`, and
the evaluator-manifest SHA-256 is
`e6da7fde7c4dd7cea460a0a314ec988add894ded6c1f6957dbda59e0f5575365`.
