# Phase138 affine-TDCP structural launch-free audit

Execution label: Luna Max  
Phase: 138  
Scope: contract qualification only; no raw, solver, truth, MAT, PDC,
precomputed-coordinate, accuracy, or Kaggle access.

## Decision and pins

The sole candidate remains the already frozen Phase138 adapter
`phase138-affine-tdcp-anchor-range-constant-v1`, from design freeze
`49be79247b4d82bf7a64c7b63e542c5a6dfff2c0`, implemented by
`c5783d7e323b0c4593958a4e210284cf9f9fc726`.  The preceding Phase135 Doppler
source correction is pinned to `f9a1fc9403e7072435a30d9e06aa8ea59493cdc5`.
This audit adds no algorithm or configuration change.

The structural recipe is exactly:

* Phase135 official affine P/D/ordinary-TDCP family: on.
* Phase138 anchor-range constant: on.
* Phase118 official TDCP Huber mapping: on; fixed TDCP sigma is 0.03 m and
  Highway k is 0.5.
* Phase107 native raw-base compensation and source miss mask: on, exactly
  once; preserve-additional-frequency-bands: off.
* Phase117, Phase120, and Phase126--134 selectors: off.
* Source C7/D meter-state handoff, CCDD, QR (`MULTIFRONTAL_QR` /
  `EliminateQR`), IMU, filtering, LM schedule, and Pixel5 final offset: fixed
  at the prior recipe.

The only planned route order is MTV-A
(`2021-03-16-18-59-us-ca-mtv-a/pixel5`) followed by LAX-T
(`2022-04-01-18-22-us-ca-lax-t/pixel5`), one eventual native invocation per
route.  This document does not authorize those invocations.

## Source-backed Phase138 boundary

For each retained adjacent same-signal TDCP pair, the Phase135 affine factor
uses the previous endpoint fixed-initial LOS and the Phase118 prepared carrier
delta.  Phase138 changes only the measurement constant:

```
tdcp_phase138 = tdcp_native - (rho_current_initial - rho_previous_initial)
```

Both `rho` values must come from the same source geodist helper used for the
affine LOS, with the same endpoint epoch/satellite state and one explicit
Sagnac term.  The factor keys, Jacobian, pair admission, sigma, robust loss,
and factor count are unchanged.  The native implementation records the range
validation and adjustment counts, the exact-one application pass, the frozen
equation, and the single-Sagnac representation in the Phase138 summary.

The launch-free validator therefore requires, for every route:

* Phase135, Phase138, and Phase118 are enabled exactly once; all other listed
  selectors are absent/off.
* Every admitted P, D, and ordinary TDCP row becomes exactly one affine factor;
  legacy P/D/TDCP counts are zero and the Pose3-X bridge is complete.
* `range_constants_validated == tdcp_measurements_adjusted ==
  affine_tdcp_factor_count == ordinary_tdcp_admitted_rows`; the adjustment
  pass is exactly one and `adjusted_exactly_once` plus
  `factor_count_unchanged` are true.
* The Phase138 equation and fixed-initial-endpoint geometry metadata are exact;
  source geometry is finite, endpoint/satellite-state provenance is the same
  source path as the affine LOS, and the Sagnac representation is single.
  Missing/nonfinite/mismatched endpoint or state evidence is fail-closed.
* Raw-base correction is applied once with conserved source-miss accounting;
  raw/zero/held/nearest/extrapolated/fallback corrections are forbidden.
* C7 has seven metre-valued components, D is metre/second with exact retained
  key alignment, the main solver is QR, both GNSS-first and main have finite
  strict cost decrease and at least one accepted iteration, and no retry or
  fallback occurs.
* Output is finite, earth-valid, epoch-complete, and has exactly one Pixel5
  final-output offset.  Only an opaque solution hash/row seal may cross the
  structural boundary; coordinate rows are never read or displayed.

The endpoint/satellite-state condition is deliberately not satisfied by a
caller-provided count alone: the pinned native source must contain the two
Phase135 geodist evaluations around the Phase138 helper, the exact geometry
representation string, and the source-state fields.  A later raw summary
missing any of this evidence fails closed rather than being inferred.

## Qualification boundary and accounting

The launch-free artifacts contain only tracked source, freeze/manifest JSON,
placeholder argv, and in-memory synthetic summaries.  Before independent
authorization, all raw phone GNSS/IMU, broadcast navigation, raw-base RINEX,
solution-coordinate, truth, MAT/PDC/precomputed, solver, accuracy, and Kaggle
read counters are zero.  No input is materialized, no native process is
launched, and no solution content is opened.  Structural failure is sealed as
failure; rerun, fallback, repair, tuning, and sweep are disallowed.

The next boundary is a separate independent raw authorization commit that
must pin this audit, the Phase138 freeze, implementation, runner/manifest,
focused tests, and target binary.  Only after that authorization may the two
raw-only routes be materialized and attempted once, in the fixed order.  Truth
and accuracy authorization remains a separate later boundary.

