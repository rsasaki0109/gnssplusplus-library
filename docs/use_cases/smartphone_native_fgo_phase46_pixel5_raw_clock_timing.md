# Phase 46 Pixel5 raw Android receiver-clock timing audit

Phase 46 audits the one raw factor named by Phase 45: Android GNSS receiver
clock/timing residual.  It is a diagnostic only; it does not modify a
candidate, rerun the solver, fit a correction, or read truth.  The exact four
Pixel5 `device_gnss.csv` inputs, equations, thresholds, and one-read contract
were frozen in the [Phase 46 freeze](records/smartphone_r5_phase46_pixel5_raw_clock_timing_freeze_v1.json)
before the first raw payload read.  The source/test/freeze hashes were then
sealed in the [evaluator manifest](records/smartphone_r5_phase46_pixel5_raw_clock_timing_evaluator_manifest_v1.json)
(`089db81`).

## Fixed raw clock contract

One evaluator process read each pinned raw CSV exactly once.  No ground truth,
Phase 45 truth-derived residual/ENU payload, archive, rematerialization,
validation/holdout, MAT, device-WLS, precomputed coordinate, Kaggle/token, or
native solver path was opened.  Only `MessageType=Raw` rows were retained.

For each row, the exact clock intermediate was

```
gps_hardware_time_ns = integer(TimeNanos) - integer(FullBiasNanos) - Decimal(BiasNanos)
```

The UTC comparison used the fixed dataset-era 18-second GPS--UTC offset and
the GPS epoch Unix offset:

```
utc_gps_residual_ms = gps_hardware_time_ns / 1e6
                     + 315964800000 - 18000 - utcTimeMillis
```

This follows Android's documented `TimeNanos - (FullBiasNanos + BiasNanos)`
definition and its `HardwareClockDiscontinuityCount` boundary semantics
([`GnssClock`](https://developer.android.com/reference/android/location/GnssClock)).
The fixed leap offset is used only for this audit comparison; it is not added
to the Phase 25 pseudorange equation.  The integer `TimeNanos - FullBiasNanos`
subtraction was also compared with float64 to quantify precision loss.

Rows were grouped by first-seen `utcTimeMillis`.  A new Phase 25 segment was
declared only for a HardwareClockDiscontinuityCount change or a preceding
`TimeNanos` gap strictly greater than one second.  A FullBias change alone was
therefore an uncaptured change, not silently promoted to a segment boundary.
The audit reports segment-base versus per-row receiver TOW and its
c-scaled pseudorange-equivalent effect without using coordinates or truth.

## One-shot result

| Route | Raw rows / epochs | Clock median (ms) | Detrended max (ms) | Drift (ppm) | Uncaptured FullBias changes | Base-vs-row failures | Max base-vs-row effect |
|---|---:|---:|---:|---:|---:|---:|---:|
| MTV-a | 87,705 / 2,159 | 0.248528 | 0.012085 | 0.047444 | 2,152 | 2,158 | 29,421.032 m |
| MTV-h | 112,833 / 3,140 | 0.304506 | 0.004003 | 0.001122 | 2,940 | 3,139 | 2,485.879 m |
| LAX-t | 51,243 / 1,466 | 0.269683 | 0.002844 | 0.005804 | 1,398 | 1,465 | 2,852.225 m |
| MTV-u | 35,810 / 1,102 | 0.930999 | 0.002457 | 0.021190 | 1,088 | 1,101 | 6,268.361 m |

All four routes had finite required timing, complete raw epoch-domain
coverage, monotonic/non-conflicting UTC keys, zero inferred missing epochs,
zero `TimeNanos` gaps above one second, and zero HCDC transitions.  The
UTC/GPS residual and drift gates therefore pass comfortably: the largest
detrended residual is 0.012085 ms against a 2 ms limit, and the largest drift
is 0.047444 ppm against a 1000 ppm limit.

The failure is the same on every route: `FullBiasNanos` changes on essentially
every subsequent epoch while the HCDC count stays constant and the 1-second
boundary is not crossed.  The resulting segment-base/per-row differences reach
98,138 ns (29.421 km at c) on MTV-a.  This is not satellite geometry: exact
same-epoch receiver-clock spread is 0 m, every same-epoch clock-field spread
is zero, and available constellation/signal group median spread is zero.  The
large value is a common receiver-clock gauge, while float64 subtraction can
lose up to 256 ns.

The [machine-readable result record](records/smartphone_r5_phase46_pixel5_raw_clock_timing_result_v1.json)
and [output manifest](../../output/smartphone-r5/phase46-pixel5-raw-clock-timing-v1/phase46_pixel5_raw_clock_timing.manifest.json)
capture all per-route, aggregate, event-table, and read-accounting hashes.

## Gates and decision

The gates were immutable before the read:

| Gate | Limit | Observation | Result |
|---|---:|---:|---|
| Route count | 4 | 4 | pass |
| Raw input identity / finite timing / epoch coverage | exact / finite / 1.0 | exact / finite / 1.0 | pass |
| UTC key order | 0 conflicts/nonmonotonic | 0 | pass |
| Detrended UTC/GPS residual | <=2 ms | 0.012085 ms max | pass |
| Affine drift | <=1000 ppm | 0.047444 ppm max | pass |
| HCDC/FullBias changes without explicit boundary | 0 | 7,578 FullBias changes | fail |
| Uncaptured non-common-mode TOW effect | <=10 m | 0 m | pass |
| Base-vs-row FullBias identity | 0 failures | 7,863 | fail |

Decision: **No-Go for receiver-clock correction.**  The only large structure is
common-mode and fails the frozen base-vs-row/boundary contract; it is not an
identified, deployable horizontal-geometry mechanism.  No correction was
implemented.

If another phase is explicitly authorized, the single next raw physical factor
is **satellite-specific raw code propagation/multipath residual**.  Android
defines `ReceivedSvTimeNanos` as the received satellite time and exposes
per-satellite `State` and `MultipathIndicator` fields ([`GnssMeasurement`](https://developer.android.com/reference/android/location/GnssMeasurement)).
That factor must first show non-common-mode, geometry-changing evidence from
raw fields before any implementation is considered.  The Phase 46 result does
not authorize that work.

## Artifacts and verification

The one-shot command was:

```bash
python3 apps/commands/benchmarks/gnss_smartphone_phase46_pixel5_raw_clock_timing_audit.py
```

It produced 15,441 event rows.  The output artifact hashes are:

| Artifact | SHA-256 |
|---|---|
| `phase46_pixel5_raw_clock_timing.json` | `98e77fee8ff0f3135f7ba0bdbefdd97709085c8ba26a2e1d5f54837cdae23eba` |
| `phase46_pixel5_raw_clock_timing.routes.json` | `7e83aea173369019de9660392dbfcab87a619bf65b7fb71e56b7db2cd48c7b69` |
| `phase46_pixel5_raw_clock_timing.events.json` | `8b887add559ebc1a47d3ce7fb5e3f5873098dad610355f7e810054180b44c00e` |
| `phase46_pixel5_raw_clock_timing.manifest.json` | `df0425fb8f1e334cc6dd2d0f52d2c2ff5705617ef3490fe4316c6be501db3a06` |

Focused tests are registered as
`python_smartphone_phase46_pixel5_raw_clock_timing_audit_tests` in CMake.
