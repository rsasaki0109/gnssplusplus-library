# Phase 44 Pixel 5 development accuracy

Phase 44 scores the already sealed Phase 43 fallback-seed candidate on four
route-disjoint Pixel 5 development identities.  It is an evaluation-only
step: the native solver and binary are not rerun, candidate/control bytes are
not rewritten, and no validation or holdout truth is opened.

The immutable pre-truth contract is the [Phase 44 freeze](records/smartphone_r5_phase44_pixel5_development_accuracy_freeze_v1.json).
The source and focused-test hashes were committed in the
[evaluator manifest](records/smartphone_r5_phase44_pixel5_development_accuracy_evaluator_manifest_v1.json)
before any Phase 44 truth payload was opened.  The evaluator reads the
existing MTV-a truth once and materializes/opens/reads each added-route
`ground_truth.csv` member once, in one process (four route reads total).

## Fixed scoring contract

The candidate and each valid control are parsed once from the exact Phase 43
run-1 artifact pinned by the freeze.  Timestamps match only on the exact
`(phone, UnixTimeMillis)` key; no nearest timestamp, interpolation, edge hold,
extrapolation, or coordinate synthesis is allowed.  Missing truth keys remain
uncovered.  Every matched row uses spherical Haversine horizontal distance
with radius `6,371,008.8 m`.  P50 and P95 use linear interpolation at rank
`(n - 1) * q`, and the local Kaggle diagnostic is `(P50 + P95) / 2`.

The one missing key on MTV-a (`2158/2159`) and the one on MTV-u (`1101/1102`)
are the leading raw-UTC warm-up epochs excluded by the sealed Phase43 native
`raw_utc_key_contract` (`warmup_epoch_excluded: true`); they are not filled or
nearest-matched by this evaluator.

Per route the result reports mean, P50, P95, maximum, exact truth coverage,
maximum speed, and the count of transitions strictly above 70 m/s.  The
Pixel 5 macro score is the arithmetic mean of the four route scores.  MTV-h's
Phase 43 flag-off control failed closed and has no output, so it is excluded
from the valid-control comparison while the candidate is evaluated against
the absolute gate.

## Reproduction

After the freeze and evaluator manifest commits, run the one-shot score:

```bash
PYTHONHASHSEED=0 python3 apps/commands/benchmarks/gnss_smartphone_phase44_pixel5_development_accuracy.py score
```

The result is written to
`output/smartphone-r5/phase44-pixel5-development-accuracy-v1/`.  The focused
contract test is registered as
`python_smartphone_phase44_pixel5_development_accuracy_tests`.

## Sealed result

The [result record](records/smartphone_r5_phase44_pixel5_development_accuracy_result_v1.json)
is `no-go-development-absolute-gate`.  All candidate rows are finite and no
prediction transition exceeds 70 m/s, but the target score, two additional
route scores, macro score, and two exact-coverage checks fail the predeclared
absolute gates:

| route | candidate mean m | P50 m | P95 m | max m | score m | exact coverage | over70 |
|---|---:|---:|---:|---:|---:|---:|---:|
| MTV-a / Pixel 5 | 1.395945 | 1.292875 | 2.947137 | 3.787958 | 2.120006 | 2158/2159 | 0 |
| MTV-h / Pixel 5 | 2.586677 | 2.428867 | 4.785064 | 5.161751 | 3.606966 | 3139/3139 | 0 |
| LAX-t / Pixel 5 | 3.815338 | 3.405107 | 5.618205 | 9.376928 | 4.511656 | 1465/1465 | 0 |
| MTV-u / Pixel 5 | 3.245134 | 3.399597 | 4.414721 | 4.897524 | 3.907159 | 1101/1102 | 0 |
| **Pixel 5 macro** | — | — | — | — | **3.536447** | — | **0** |

The target absolute limits were score ≤3.0 m, P95 ≤5.0 m, and over70 = 0;
the four-route macro limit was ≤2.0 m and each route score had to be ≤3.0 m.
The target route score was `3.6069657083 m`, so the aspirational `0.782 m`
target was not met and is recorded without adjustment.  No validation was
opened even though the result is complete, and no post-truth tuning was
performed.

Truth accounting is four route reads total (existing 1; added archive
members materialized/opened/read 3), with zero validation, holdout, MAT,
device-WLS, precomputed-coordinate, Kaggle, or token access.
