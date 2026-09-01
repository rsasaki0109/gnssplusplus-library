# Phase 45 Pixel 5 residual identifiability diagnostic

Phase 45 tests whether the sealed Phase 44 four-route Pixel 5 development
cohort supports a common residual model.  It is an evaluation-only diagnostic:
the Phase 43 candidate bytes remain unchanged, the native solver is not run,
and no truth-derived correction is fitted, written, selected, or deployed.

The first freeze was committed as
`smartphone_r5_phase45_pixel5_residual_diagnostic_freeze_v1.json`.  Its
successor v2 corrected the exact-model coverage wording but still repeated
truth-row coverage in the gate criterion.  v3 is the operative freeze and
pins both predecessors.  v3 treats all candidate keys being exactly matched
inside the prediction/raw-UTC domain as the gate; complete read coverage of
the pinned truth materialization is reported separately.  The one missing
truth key on MTV-a and MTV-u is the known Phase 43 raw-UTC warm-up exclusion,
not an interpolation or edge-fill opportunity.

## One-shot contract

After the evaluator manifest was committed and pushed, the evaluator read
each sealed candidate submission, each Phase 44 materialized truth CSV, and
each pinned raw `device_imu.csv` exactly once in one process.  It did not open
or rematerialize the dataset archive.  The truth path was never used to
produce an inference coordinate.  Candidate rows are matched to truth only
on exact `(phone, UnixTimeMillis)` keys.

For every exact match, the evaluator computes prediction-minus-truth latitude
and longitude displacement in a fixed local ENU frame at the first matched
truth coordinate.  The submission has no altitude; `up` therefore remains
`null`.  It reports component medians, unscaled MAD, ordinary and component
3-MAD-inlier covariance, first/last 25% shifts, four chronological quartiles,
normalized linear time drift, prediction-only speed bins, and raw-only
`UncalAccel`/`UncalMag` gravity and tilt-compensated heading groups.  Pairwise
route-median ENU distances are included.

The leave-one-route-out (LOO) diagnostic subtracts the component-wise median
of the other three route medians only from residual vectors in the held route.
Its P50/P95 score is reported for transfer diagnosis; no corrected coordinate
is written and no native process consumes it.  The full-cohort common median
diagnostic is likewise offline only.

The predeclared gates are all required: four exact Pixel 5 routes and complete
prediction-domain coverage; route-center component MAD/radius; prefix-tail
stability; raw orientation independence; LOO improvement on every route;
LOO macro improvement; a 0.10 m individual-worsening limit; and the separate
0.782 m full-cohort reachability check.  Truth-row coverage and warm-up
exclusions are recorded independently from prediction-domain coverage.

Reproduce only the frozen one-shot command (the output directory must not
already contain a result):

```bash
PYTHONHASHSEED=0 python3 apps/commands/benchmarks/gnss_smartphone_phase45_pixel5_residual_diagnostic.py score
```

The focused test is registered as
`python_smartphone_phase45_pixel5_residual_diagnostic_tests`.

## Result

The checked-in result is
`smartphone_r5_phase45_pixel5_residual_diagnostic_result_v1.json`; the
atomic output is under
`output/smartphone-r5/phase45-pixel5-residual-diagnostic-v1/`.

The diagnostic is `no-go-residual-not-identifiable`.  All four routes have
complete prediction-domain coverage and complete pinned truth-row reads, but
the residual centers are not common (`north` route-median MAD 1.348603 m and
maximum center distance 2.830536 m), the MTV-h prefix-to-tail shift is
4.492411 m, the heading groups on MTV-h differ by 2.810526 m, and three LOO
held routes worsen.  The LOO macro changes from 3.533210 m to 4.248509 m
(−0.715299 m improvement), with maximum individual worsening 2.298791 m.
The full-cohort common-median diagnostic reaches 3.229678 m macro and
3.220895 m on the target MTV-h route, so the aspirational 0.782 m possibility
is explicitly not reached.

| route | median ENU (E,N) m | MAD (E,N) m | prefix→tail shift m | matched/candidate | truth rows | LOO score m | LOO Δ m |
|---|---:|---:|---:|---:|---:|---:|---:|
| MTV-a / Pixel 5 | (0.454259, 0.299114) | (0.829571, 0.442753) | 2.671926 | 2158/2158 | 2159 (1 warm-up) | 4.418367 | −2.298791 |
| MTV-h / Pixel 5 | (1.292189, 0.275718) | (2.037893, 0.608306) | 4.492411 | 3139/3139 | 3139 | 4.079825 | −0.470767 |
| LAX-t / Pixel 5 | (−1.706457, 2.996320) | (0.455745, 0.193815) | 1.228298 | 1465/1465 | 1465 | 4.891380 | −0.387283 |
| MTV-u / Pixel 5 | (1.104483, 2.986060) | (0.597337, 0.878315) | 2.380510 | 1101/1101 | 1102 (1 warm-up) | 3.604466 | +0.295643 |

The strongest residual structure is LOO common-median non-transfer: the
common vector learned on three routes makes the MTV-a, MTV-h, and LAX-t held
scores worse.  The single next source-supported raw physical factor is the
Android raw GNSS receiver clock/timing residual (`TimeNanos`, `FullBiasNanos`,
`BiasNanos`, and `utcTimeMillis`), to be audited truth-free under a new freeze.
This phase implements no such factor and does not authorize validation or
holdout access.
