# Phase 47 Pixel5 raw code propagation/multipath audit

Phase 47 audits the single raw physical factor selected by Phase 46:
satellite-specific raw code propagation/multipath residual.  It is a
truth-free, audit-only phase.  The evaluator opened the four pinned Pixel5
`device_gnss.csv` files once each in one process, used no navigation file or
coordinate, and did not rerun a solver or fit/apply a correction.

The freeze was committed and pushed before any Phase 47 raw read in
`1a9cb50`.  The evaluator, focused tests, CMake registration, and manifest
were then committed and pushed in `bbb934f`.  The frozen input and evaluator
hashes are recorded below and in the machine-readable result.

## Fixed raw-only contract

Raw pseudorange follows the Phase 25 long-double contract.  The evaluator
subtracts integer `TimeNanos - segment_base_FullBiasNanos` before converting to
Decimal, subtracts `BiasNanos` and per-signal `TimeOffsetNanos`, maps the
constellation transmit time (including BeiDou +14 s and the Phase 25
GLONASS day mapping), unwraps to the nearest half-week, and multiplies by the
speed of light.  No `SvPosition`, `SvElevation`, navigation, WLS, or truth is
used.

Same first-seen `utcTimeMillis` epoch, constellation, and SVID pairs are
formed for GPS L1CA--L5, Galileo E1--E5a, and BeiDou B1I--B2A.  The primary
minus secondary code difference cancels receiver-clock and geometry common
terms to first order.  The frequency-squared ionosphere-consistency
combination is reported but is explicitly ionosphere-confounded and is not a
correction.  For valid ADR arcs, CMC is `P - signed ADR`; arcs reset on HCDC,
the frozen 1.5 s gap, cycle slip, ADR invalidity, or missing P/ADR.

The existing adapter masks are reported without adding a new mask:
`MultipathIndicator == 1`, C/N0 below 20 dB-Hz, invalid Android code/time
state, and the existing pseudorange range check.  Android documents
`ReceivedSvTimeNanos`, `State`, and `MultipathIndicator` as per-measurement
fields ([GnssMeasurement](https://developer.android.com/reference/android/location/GnssMeasurement)); the
repository adapter contract is pinned to
`src/io/android_raw_gnss.cpp` and the Phase 25 record.  The current FGO
observation path was statically checked to iterate eligible parsed
observations and construct pseudorange factors for the declared pair
signals.  The regressed Phase 13 Pixel7pro upstream-quality candidate is
policy-only and is not re-proposed.

## One-shot result

The machine-readable output is
`output/smartphone-r5/phase47-pixel5-raw-code-multipath-v1/`.  Its artifact
hashes are:

| Artifact | SHA-256 |
|---|---|
| `phase47_pixel5_raw_code_multipath.json` | `a060ca1ed8f5316111f610af5bd7b3d324e44581ea05d334de0ba3d6e636a802` |
| `phase47_pixel5_raw_code_multipath.routes.json` | `d2ba58672cd5525a235f45986a5c13743a801e442d5a52ed9fc24e1564b91fdc` |
| `phase47_pixel5_raw_code_multipath.events.json` | `e61bf8ab26e823c3797752755de8ac9d7921fc483efd03ca53fb6da4b02bd55d` |
| `phase47_pixel5_raw_code_multipath.manifest.json` | `2cb86fcf0a4dc24291dc292ed9226504cad0a30249f5009db89c5162f0e0fcd5` |

All four routes met raw domain coverage (1.0), had finite reconstructed P,
and had monotonic non-repeated UTC keys.  Pair coverage exceeded the frozen
minimum on every route:

| Route | Raw rows | Epochs | Pair satellites (max) | Pair epochs (max) | Pair clean retention | Existing-mask clean raw retention |
|---|---:|---:|---:|---:|---:|---:|
| MTV-a | 87,705 | 2,159 | 7 | 12,855 | 99.386% | 95.333% |
| MTV-h | 112,833 | 3,140 | 7 | 19,488 | 99.520% | 96.386% |
| LAX-t | 51,243 | 1,466 | 11 | 11,081 | 99.211% | 98.952% |
| MTV-u | 35,810 | 1,102 | 11 | 7,731 | 99.612% | 98.830% |

The strongest pair-level raw metric was Galileo E1--E5a flagged-minus-clean
p95 excess.  It was materially large in all four routes (flagged p95 at
least 21.885 m, ratio at least 1.698x clean), and the same threshold decision
held in every leave-one-route-out fold.  Same-epoch non-common spread and
satellite median dispersion were present in all routes; the static FGO
signal-adoption and impacted-row gate passed.  MultipathIndicator itself was
zero in every raw row, so this result does not duplicate an already-observed
`MultipathIndicator == 1` population.

The audit is nevertheless fail-closed for evaluator integrity.  The emitted
route table's `cmc_diagnostics.groups` reused the final inner arc list for
`raw_cmc_m`/`count` (for example, `count:39` despite the stored centered CMC
count being much larger), and its aggregate
`pair_code_difference_route_medians_m` presentation field collapsed four route
medians to one value.  These are required diagnostic fields, not cosmetic
metadata.  The CMC per-signal raw distributions and that aggregate field are
therefore quarantined; no raw payload was reopened and the evaluator was not
rerun.

| Route | Canonical pair | Flagged p95 (m) | Clean p95 (m) | Flagged/clean | Excess (m) | Satellite median range (m) |
|---|---|---:|---:|---:|---:|---:|
| MTV-a | GAL E1--E5a | 45.419 | 9.593 | 4.734x | 35.825 | 4.197 |
| MTV-h | GAL E1--E5a | 77.406 | 12.591 | 6.148x | 64.815 | 4.197 |
| LAX-t | GPS L1CA--L5 | 41.611 | 19.187 | 2.169x | 22.424 | 10.493 |
| MTV-u | GAL E1--E5a | 35.076 | 14.390 | 2.438x | 20.686 | 24.883 |

The frozen route-center dispersion gate is the only failed numerical gate in
the descriptive pair summary: canonical
route excesses are **35.825, 64.815, 22.424, and 20.686 m**, with route MAD
**7.570 m** against the **2.0 m** limit.  Thus the flagged-vs-clean effect is
not a stable common four-route mechanism even though each route independently
passes materiality, retention, non-common-mode, and LOO threshold checks.
CMC diagnostics, per-signal/satellite summaries, C/N0, state,
`ReceivedSvTimeUncertaintyNanos`, pseudorange-rate uncertainty, ADR buckets,
arc events, and pair outlier persistence are in the route/event tables.

Decision: **No-Go for evaluator integrity, and therefore no deployable raw
code/multipath correction or new weighting.**  The strongest finding is the
stale-loop/aggregate-field defect above; the route-dependent flagged code
dispersion is descriptive only and fails the frozen route-center gate as
well.  The exact one next source-supported raw physical factor is **raw
Android per-satellite `ReceivedSvTimeUncertaintyNanos`/code-tracking
residual**.  It is not implemented or read in Phase 47.  The 0.782 target is
not evaluated without truth.

## Accounting and verification

The one-shot read accounting is four raw CSV reads total, one per route;
truth, Phase45 payload, Phase46 payload, BRDC navigation, archive,
rematerialization, solver, validation/holdout, MAT, WLS, `SvPosition`,
`SvElevation`, precomputed coordinates, and Kaggle/token access are all zero.
The output event table contains 125,801 raw-only events.  Focused tests are
registered as
`python_smartphone_phase47_pixel5_raw_code_multipath_audit_tests`.

The machine-readable result record marks this as
`no-go-evaluator-integrity-failure` and preserves the immutable output hashes
for auditability; it explicitly records that the raw input was not reopened.

The evaluator manifest pins freeze SHA
`bd9c3b0649b2068286de44bfd36171d09b46ff3e6e760d02067e225f5a13d5f9`, source
SHA `dba0adc17fbdc04c3ce6031fece8a28d4218a680a719cfffcbe6366c2b8f5b72`, and
focused-test SHA
`e08da5015619280fa139bc59fa290b12262182f36803ccbbae8e181b69e1f7df`.
