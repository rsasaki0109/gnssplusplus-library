# Phase61 Pixel5 C/N0 pseudorange CMC audit

Phase61 is a truth-free, raw-only audit of a single source-supported factor:
Android `Cn0DbHz` pseudorange code-tracking/multipath residual calibration.
It is explicitly separate from the Phase47 mask/integrity result and from the
optional upstream p85 SNR path.  The four route-disjoint Pixel5 raw
`device_gnss.csv` files were read once each by one evaluator process.  No
truth, navigation, solver, trajectory, IMU, coordinate, WLS, enriched
pseudorange, MAT, validation, holdout, archive, Kaggle, token, or previous
metric payload was read.

## Frozen observable and source contract

The raw pseudorange is the Phase25 integer/Decimal Android-clock
reconstruction.  For each `(system,SVID,signal)` continuous ADR arc, the
fixed observable is

```text
cmc_m = P_raw_m - AccumulatedDeltaRangeMeters_signed_m
residual_m = abs(cmc_m - median_arc_cmc_m)
```

Only arcs with at least two valid rows are eligible; singleton rows are
reported as exclusions and are not imputed.  Existing code/C/N0/multipath and
ADR-state masks are retained.  Pixel5 is not in the adapter's ADR sign
negation list, so the signed Android ADR value is preserved.  Same-epoch/HCDC
centering is diagnostic only and is never applied as a correction.

The source-supported shape was fixed before raw access:

```text
shape(Cn0DbHz) = 10^(-(Cn0DbHz-40)/20)
sigma_model_m = alpha_m * shape(Cn0DbHz)
```

Each leave-one-route-out fold fits `alpha_m` as the median of
`abs(centered CMC)/shape` on its three training routes.  No truth, sweep,
clip, alternate shape, or post-read tuning is used.  If authorized in a
future phase, the only candidate scope would be an FGO undifferenced
pseudorange floor `max(existing_sigma_m, sigma_model_m)`, coefficient one and
no upper cap; no SPP, TDCP, Doppler, or correction is authorized by this
audit.  Phase43's current pseudorange path is `3.0 m / sin(elevation)^1.0`
with upstream p85 quality disabled.

## One-shot result

The pre-read freeze was pushed at `65a8fb9` (SHA-256
`d80122d87f2f6e4529483e0f319951ea1f199e46120bd0e785a2adb5eac9685d`).  The
evaluator, focused tests, CMake registration, and manifest were sealed before
the raw read; the final pre-read reseal is `8d0a449`, and the evaluator
manifest SHA-256 is
`8d4a704d88b6c310e09f541c94f2b820b8c3250809a0ab3c9738f9d7a3818520`.

The result is **no-go**:
`no-go-cn0-pseudorange-calibration-not-identifiable`.

| route | raw rows | eligible CMC rows | singleton rows | satellites | signal families | C/N0 Spearman | median abs CMC (m) | p95 abs CMC (m) |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| MTV-a | 87,705 | 8,111 | 286 | 7 | 1 | -0.223342 | 0.886164 | 3.467414 |
| MTV-h | 112,833 | 9,337 | 257 | 6 | 1 | -0.203747 | 0.967753 | 3.937635 |
| LAX-t | 51,243 | 4,695 | 320 | 10 | 1 | -0.256808 | 0.752618 | 3.099244 |
| MTV-u | 35,810 | 4,169 | 249 | 11 | 1 | -0.168379 | 0.752588 | 3.261637 |

All eligible rows had finite positive C/N0.  However, LAX-t and MTV-u are
below the frozen 5,000 CMC-row minimum; every route has only one supported
signal family (`GALILEO:GAL_E1`), below the two-family composition gate; and
the routewise Spearman threshold (`<= -0.25`) fails for MTV-a, MTV-h, and
MTV-u.  Fixed-bin monotonicity fails in all four routes (median in MTV-a and
MTV-u; p95 in MTV-h and LAX-t as well).  Leave-one-route-out direction is
negative in only one of four folds.  The fitted model remains below the
configured 3.0 m pseudorange base sigma in every route, so its conservative
raw-only impact proxy is zero in every fold.

The strongest finding is a modest high-C/N0 reduction in CMC residual in
some routes, but it is not route-stable, cannot be separated from the
single-family composition, and cannot affect the configured pseudorange
factor under the frozen impact proxy.  Therefore no native C++ correction or
implementation stage is authorized.  Phase43 remains the champion and
Phase51 remains experimental.  Phase61 selects no second factor; the next
factor must be chosen in a separately frozen phase.  The `0.782` target is
not evaluated without accuracy truth.

## Accounting and artifacts

Raw accounting is one process, four raw GNSS reads (one per route), truth 0,
navigation 0, solver/trajectory 0, IMU 0, coordinate/WLS/SvPosition/
SvElevation 0, enriched pseudorange 0, MAT 0, validation/holdout 0, archive
reopen/rematerialization 0, Kaggle/token 0, and Phase43/47/60 metric payload
reads 0.  Unsupported signal rows are informational only; no raw values are
imputed.

The machine-readable result is
[`smartphone_r5_phase61_pixel5_cn0_pseudorange_calibration_result_v1.json`](records/smartphone_r5_phase61_pixel5_cn0_pseudorange_calibration_result_v1.json).
The freeze and evaluator manifest are
[`smartphone_r5_phase61_pixel5_cn0_pseudorange_calibration_freeze_v1.json`](records/smartphone_r5_phase61_pixel5_cn0_pseudorange_calibration_freeze_v1.json)
and
[`smartphone_r5_phase61_pixel5_cn0_pseudorange_calibration_evaluator_manifest_v1.json`](records/smartphone_r5_phase61_pixel5_cn0_pseudorange_calibration_evaluator_manifest_v1.json).

The immutable raw-only output under
`output/smartphone-r5/phase61-pixel5-cn0-pseudorange-calibration-v1/` has:

| artifact | bytes | SHA-256 |
| --- | ---: | --- |
| result | 84,200 | `d37d883ed148a831c9482094e929da986429eb9acbf76f83a29a4dc6a4947587` |
| routes | 71,276 | `fa3258e0b171492c59ab038177f79a7675a2b1ea509407193640b7f53e8d1694` |
| events | 29,164,429 | `eb83c126142998358a66be10f59747d800499aebbb15a67bfacda2a277e8dad` |
| output manifest | 460 | `090cce32cad3e6c4557d7244fa890fa0dc49b1cda00b5168d2111d6df32b7a81` |

The focused Python suite passed 5 tests, `py_compile` passed, and freeze and
manifest verification passed before raw access.  CMake registers
`python_smartphone_phase61_pixel5_cn0_pseudorange_calibration_tests`.
