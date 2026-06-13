# CLAS DD PPP-RTK Filter A3

A3 adds fixed-candidate validation and fix-and-hold feedback to the gated
`GNSS_PPP_CLAS_DD_FILTER=1` CLAS DD scaffold.  The default CLAS and MADOCA
paths remain outside this code path and were verified byte-identical to A0.

## Implemented

- `DdFilterScaffold::conditionFixedSnapshot()` now treats LAMBDA acceptance as
  a candidate only.  The conditioned fixed snapshot is re-evaluated through the
  DD row builder and must pass phase postfit RMS, max residual, chi guard,
  ratio, PDOP, min-fix, and reference-continuity gates before PPP_FIXED is
  published.
- The persistent DD state is no longer overwritten by every fixed snapshot.
  After a validated fix, `processFloatUpdate()` applies CLASLIB `holdamb()`-
  style pseudo-measurement rows on DD ambiguity differences using
  `varholdamb = 0.001 cyc^2`.  Hold rows use the validated DD integers and the
  current float DD ambiguity states.
- Ambiguity AR eligibility now tracks CLASLIB-style lock count and AR elevation
  mask semantics: new or reset ambiguity states start at `-arlockcnt`, and
  phase rows increment lock only after an accepted DD float update.
- `buildDdMeasurementSystem()` now exposes reference-satellite groups and row
  elevations.  The scaffold tracks per-system/frequency phase references; a
  reference switch clears held ambiguity feedback and rejects the fixed
  candidate for that epoch rather than carrying a stale DD datum.
- `DdUpdateDiagnostics` now surfaces fixed reject reason, postfit residual
  statistics, PDOPs, reference-change count, consecutive fix count, and hold
  rows/applied state.  `GNSS_PPP_CLAS_DD_DIAG=/path.csv` writes the same fields
  for tuning reports.
- Added `PPPClasDdTest.PostfitValidationRejectsLargePhaseRms`.

The chi statistic in the native scaffold is not numerically identical to
CLASLIB `filter2()` because the active covariance and DD row covariance are
assembled through the local Eigen measurement system.  A finite high chi guard
is kept, while the measured A3 decision gate is dominated by phase postfit RMS
and max residual.

## A2 -> A3 Oracle Result

2019-08-27 CLAS, same-epoch
`/tmp/s31_ref/claslib_oracle_xyz_full.pos`, `--ar-ratio-threshold 2.0`,
3599 matched epochs.

| Path | Fixed epochs | Fixed 3D mean | Fixed Up mean | All-epoch 3D mean |
| --- | ---: | ---: | ---: | ---: |
| A2 | 1073 | 1.342069 m | 0.709820 m | 1.161354 m |
| A3 | 768 | 1.246752 m | 0.378185 m | 1.050019 m |

A3 remains above the native fixed-count floor of 301, improves fixed-only 3D
and Up means versus A2, and improves all-epoch 3D mean versus A2.

## Gate Counts

Final gate-on diagnostic counts from `/tmp/a3_clas_dd_diag.csv`:

| Gate | Rejected epochs |
| --- | ---: |
| Ratio | 1252 |
| Postfit RMS | 56 |
| Postfit max residual | 6 |
| PDOP | 0 |
| Min-fix | 0 |
| Reference change | 0 |
| Insufficient ambiguities | 552 |

Hold feedback was applied on 768 epochs.

## A4 Entry Points

- Add QZSS J01-J03 row-set parity to `buildDdMeasurementSystem()` once the DD
  scaffold consumes their corrections with the same row admission as CLASLIB.
- Keep reference continuity diagnostics enabled while adding QZSS rows; QZSS
  reference switches can silently change the DD datum if held ambiguities are
  not reset or withheld.
- Re-run the same A3 oracle table with QZSS rows admitted, then split reject
  counts by constellation/frequency to verify whether QZSS improves the float
  datum or introduces postfit residual outliers.
