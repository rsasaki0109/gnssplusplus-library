# Phase 48 Pixel5 raw code-rate uncertainty audit

Phase 48 audits the single raw factor named by Phase 47: per-satellite
Android `ReceivedSvTimeUncertaintyNanos` and its code-tracking residual.  This
is a truth-free, audit-only phase.  The four frozen Pixel5 raw
`device_gnss.csv` files were each read once in one evaluator process; no
truth, Phase 45 payload, Phase 47 metric payload, navigation, coordinates,
solver, validation, archive, or correction path was opened.

The freeze was pushed before any Phase 48 raw read in `4b7f6cb`.  The
evaluator, focused tests, CMake registration, and manifest were then sealed
and pushed in `cbab201`.  The evaluator manifest pins source SHA
`31ce6b4c342373bed95e4977523ace7e7df7e7f59b1469c78dd12ea5c504aaaf`, focused
test SHA `780e5ffe9dd99f333b55f0fe4250f56dc1fdca2223214f909e3fc92355b53e94`,
and CMake SHA
`4292450abf35401509b84cd6a83a1890f4e815e580801468b2622aac9ccc187b`.

## Fixed raw-only contract

Raw pseudorange uses the Phase 25 integer/Decimal segment-base contract.  A
transition is a same-satellite, same-signal consecutive raw pair with
`0 < dt <= 1.5 s`, unchanged HardwareClockDiscontinuityCount, valid Android
code/transmit state, finite in-range reconstructed P, and the existing
MultipathIndicator/CN0 masks.  The fixed residual, using the Phase 41 direct
range-rate sign contract, is

```
e_k = (P_k - P_prev)
      - 0.5*(PseudorangeRateMetersPerSecond_prev
             + PseudorangeRateMetersPerSecond_k)*dt
```

The endpoint `utcTimeMillis` + HCDC group median is subtracted to remove
receiver-clock common mode; both raw and centered residuals remain in the
route table.  `ReceivedSvTimeUncertaintyNanos` is converted to
`c*nanos/1e9` metres.  `PseudorangeRateUncertaintyMetersPerSecond` is reported
separately, and normalized absolute residual/sigma is reported only for
positive uncertainty.  Ordered buckets are `<=10 ns`, `10–100 ns`, and
`>100 ns`, with fixed log bins and quartiles.  The materiality comparison
uses the frozen `>10 ns` pool (the latter two ordered buckets) against
`<=10 ns`.

The existing adapter masks are only reported: `MultipathIndicator == 1`,
C/N0 below 20 dB-Hz, invalid state bits, and out-of-range P.  The static
source contract is pinned to the adapter, observation header, and FGO source;
the Phase 13 upstream-quality result remains policy metadata and is not
re-proposed.  Android's per-measurement uncertainty field is documented in
[`GnssMeasurement`](https://developer.android.com/reference/android/location/GnssMeasurement).

## One-shot result

The machine-readable output is
`output/smartphone-r5/phase48-pixel5-raw-code-rate-uncertainty-v1/`.  The
result record is
[`smartphone_r5_phase48_pixel5_raw_code_rate_uncertainty_result_v1.json`](records/smartphone_r5_phase48_pixel5_raw_code_rate_uncertainty_result_v1.json).

| Route | Raw rows | Epochs | Transitions | Satellites | Residual median abs (m) | Residual p95 abs (m) | Spearman | `<=10 ns` / `10–100 ns` / `>100 ns` | `>10 ns` p95 excess / ratio | Low fraction |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| MTV-a | 87,705 | 2,159 | 13,431 | 7 | 1.346 | 8.152 | 0.444 | 2,625 / 10,806 / 0 | 6.034 m / 3.104x | 0.195 |
| MTV-h | 112,833 | 3,140 | 15,630 | 7 | 1.388 | 9.730 | 0.467 | 3,513 / 12,117 / 0 | 8.015 m / 3.654x | 0.225 |
| LAX-t | 51,243 | 1,466 | 11,732 | 11 | 1.842 | 11.581 | 0.476 | 1,449 / 10,283 / 0 | 9.315 m / 4.036x | 0.124 |
| MTV-u | 35,810 | 1,102 | 9,612 | 11 | 1.746 | 10.889 | 0.498 | 1,836 / 7,776 / 0 | 9.018 m / 4.019x | 0.191 |

The raw association is positive in every route, with all four Spearman values
above the frozen 0.35 threshold and stable positive direction in every LOO
fold.  However, only two of the three ordered buckets are populated in every
route (`>100 ns` is empty), and the low/base population is only 0.124–0.225
against the frozen 0.30 minimum.  Only one signal family is present per
route, so the within-signal composition requirement of two passing families
also fails.

The aggregate route median absolute residuals are **1.346134** (MTV-a),
**1.388083** (MTV-h), **1.841842** (LAX-t), and **1.746348 m** (MTV-u), with
aggregate median **1.567215 m**, MAD **0.200107 m**, and all six pairwise
route distances retained.  Presentation-integrity checks pass: ordered
bucket counts sum to each route transition count, all four route medians are
retained in frozen order, and aggregate median/MAD recompute exactly.

## Static adoption and decision

The pinned source audit found no `ReceivedSvTimeUncertaintyNanos` parser in the
current Android adapter, no retained observation field, and no current FGO
sigma consumption.  The existing adapter-mask contract is present.  Thus,
even though the raw diagnostic association is material, it is not an
available estimator input.

Decision: **No-Go for an uncertainty sigma-floor mechanism.**  The strongest
finding is that the field is observable in raw data but absent from the
current adapter/observation/FGO path; sparse ordered buckets, insufficient
low/base retention, and single-family composition independently fail the
frozen identification gates.  No correction or weighting was implemented.

The exactly one next source-supported raw physical factor is **raw Android
per-satellite carrier-phase ADR cycle-slip/lock-loss residual**.  It is not
audited or implemented here.  The `0.782` target is not evaluated without
truth.  If a later phase passes all gates, it may authorize only the concept
`max(existing_sigma, c*ReceivedSvTimeUncertaintyNanos)`; Phase 48 does not
implement it.

## Accounting, artifacts, and verification

Read accounting is four raw CSV reads total (one per route, one process),
with truth, Phase 45 payload, Phase 47 metric payload, BRDC navigation,
solver/trajectory reruns, correction implementations, validation/holdout,
archive reopen, rematerialization, MAT, device-WLS/precomputed coordinates,
and Kaggle/token access all zero.  The event table contains 21,246 raw-only
events.  Focused tests are registered as
`python_smartphone_phase48_pixel5_raw_code_rate_uncertainty_audit_tests`.
Their presentation-integrity cases explicitly reject the stale inner-loop
count and collapsed four-route aggregate patterns found in Phase 47.

| Artifact | Bytes | SHA-256 |
|---|---:|---|
| `phase48_pixel5_raw_code_rate_uncertainty.json` | 37,455 | `4f2faef123f0770baf93004fb89bb7e9cfa8803be10d04ee8dc92d6df63f99e3` |
| `phase48_pixel5_raw_code_rate_uncertainty.routes.json` | 26,050 | `6528a988ad27144f547785c580b97b7abff19d1dfe6f04ecbb71bf80761ebea1` |
| `phase48_pixel5_raw_code_rate_uncertainty.events.json` | 5,754,552 | `99fec40ea301de34cea28a570baa6e4ef02657e991b1f66ea13e108435eeca51` |
| `phase48_pixel5_raw_code_rate_uncertainty.manifest.json` | 1,838 | `c7afaa9e329eb5a5e0bffa36d82f9f43eb2748bfd5636e62d630803468fd143c` |

The evaluator manifest SHA is
`93f68d54fba0288e2ff358e496bc2e2d300343548a74e5ab0e468d0ab12df053`, and the
freeze SHA is
`e8bbfddda9670b492c3a938109c92333fd8432c564c7a6837874d9a11194b454`.
