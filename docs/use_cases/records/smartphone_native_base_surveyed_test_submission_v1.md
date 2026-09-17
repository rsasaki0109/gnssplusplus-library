# Native FGO + base-surveyed: first GSDC test submission (v1)

First Kaggle submission of the **native (truth-free) GNSS/IMU FGO lane with
the base-surveyed correction** on the GSDC 2023 test set.

## Result

| metric | value |
| --- | --- |
| competition | `smartphone-decimeter-2023` |
| submission_ref | 56309958 |
| public_score | 3.460 |
| private_score | **3.344** |
| status | COMPLETE |

Prior best on this account is the upstream MAT import at 0.782 (external
precomputed results, not our method). The previous native-lane submission
scored 4.276, so base-surveyed on 8 of 40 trips improved the native lane by
**0.93 m** on the private leaderboard.

## Composition

- 71936 keys, exactly the `sample_submission.csv` key set.
- 8 Pixel5 trips: our native FGO recipe + base-surveyed (16330 keys).
- 32 trips: unchanged from `native-fgo-test-v5` (the frozen Eigen lane).

Successful trips: `mtv-g/pixel5`, `sjc-r/pixel5`, `lax-n/pixel5`,
`lax-i/pixel5`, `lax-x/pixel5`, `sjc-q/pixel5`, `mtv-de1/pixel5`,
`sjc-be2/pixel5`.

## Method

Per test route: base station from the route's `*_rnx*.obs` name and the year
from the route date, looked up in `base/base_position.csv` (surveyed ECEF);
added

```
--native-base-pseudorange-compensation --native-base-pseudorange-source-miss-mask \
--native-base-rinex <base.obs> --native-base-rinex-sha256 <sha> \
--native-base-position-ecef X Y Z
```

on top of the phase171 Android-raw native FGO recipe. Submission keys follow
the sample; unresolvable epochs keep the v5/sample coordinates.

## Findings / blockers

- **base-surveyed transfers to the test set**: replacing 8 Pixel5 trips
  moved the private score 4.276 -> 3.344.
- **Pixel5-pinned flags block the other phones**: `phase197`,
  `native-source-tdcp-meter-sigma`, `phase217`, `phase213` require
  `phoneFromDatasetId == "pixel5"`. Dropping them lets non-Pixel5 routes
  start but the reduced recipe fails on most of them.
- **Initialisation failures dominate**: of 40 test trips only ~11 produced
  output. Non-Pixel5 mostly fail with `GNSS-first initialization unavailable`
  or `Phase165 raw-P graph handoff ... epoch-count-mismatch` (rc=1); Samsung
  routes abort (rc=-6). Adding `--android-include-first-native-epoch
  --native-sparse-p-staging` (relaxed from the LAX-T-only pin) does not fix
  them.

## Not a held-out claim

The 8 Pixel5 trips are in the test split but the recipe was developed on the
four dev routes; per-phone generalisation and the dev-vs-private aggregation
gap remain open. Dev-route base-surveyed results are in
`smartphone_base_surveyed_route_results_v1.md`.
