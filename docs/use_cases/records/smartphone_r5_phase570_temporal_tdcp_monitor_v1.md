# Phase570: adjacent TDCP post-fit residual diagnostic

Read aggregate summaries only; no truth/coordinates/new score. Current
baseline development counts:

| Route | TDCP built | Pair candidates | Missing previous | Code/phase jump rejects | Median arc epochs | Normalized residual RMS |
|---|---:|---:|---:|---:|---:|---:|
| H (535) | 69270 | 71499 | 5701 | 2229 | 5 | 4.72955 |
| A (568) | 46482 | 48239 | 5949 | 1757 | 4 | 6.24483 |
| U (480 baseline) | 17289 | 17906 | 2469 | 617 | 4 | 5.26440 |
| LAX (476 baseline) | 24964 | 25915 | 3653 | 951 | 4 | 6.49098 |

Missing-previous counts are not a subset of the pair-candidate denominator.
These large normalized RMS values do not by themselves justify sigma tuning.
Prior frequency-correlated nuisance experiments differ from temporal shared
endpoint correlation; do not conflate them or revive the rejected ionosphere
state from these counts.

Added read-only adjacent residual moments helper and wired it to the existing
native app post-fit TDCP reconstruction. Same satellite/signal, exact shared
epoch endpoint required. Gaps and invalid residuals break linkage. Numerically
centered paired moments report Pearson correlation (unavailable for too few
pairs or zero variance). Metres, not sigma-normalized; no clipping, fitting,
noise-model/graph modification, or saved residual series. Joint-ionosphere
lane does not emit this statistic because reconstruction omits its terms.

Two standalone gtests passed for alternating/constant sequences and
gap/key/invalid/break handling. Registered in CMake. Native app build started,
session 26554; completion not yet established at record creation. No raw run
with this monitor yet, full CTest unrun. Resume the same live build.

Interpretation limits: pooled post-fit correlation is not raw measurement
noise covariance. Fitting, stream offsets and common-model errors affect it.
It cannot supply an inferred whitening parameter directly. Next gate is
native build and frozen output-identity replay, then use the statistic to
decide whether a temporal-noise model merits a separately specified test.
