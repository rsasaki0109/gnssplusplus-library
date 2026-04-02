# Decisions

## D-0001: Move PPP-AR work into an experiment lane first

Status: accepted

`PPP-AR` changes are no longer treated as a single implementation path.
Instead, new approaches must first be comparable inside `experiments/ppp_ar/`
before they are promoted into the stable solver path.

Why:

- `PPP-AR` still has unresolved accuracy gaps.
- The repo now has enough tooling to compare multiple approaches on the same
  CLAS input set.
- Premature abstraction made it too easy to hide solver-specific tradeoffs.

## D-0002: Use CLASLIB boundaries as the primary reference for CLAS PPP work

Status: accepted

For CLAS-oriented PPP experiments, the primary upstream reference is the
`CLASLIB` boundary between:

- Compact SSR decode
- `SSR2OSR` style observation-space correction generation
- PPP / PPP-AR solver stages

This does not mean mirroring `CLASLIB` wholesale. It means the experiments
should first test whether a correction-first pipeline materially improves:

- solution rate
- fixed epochs
- 3D error
- structural modularity

## D-0003: Keep the stable core and experiment lane separate

Status: accepted

Stable code remains under:

- `src/algorithms/`
- `include/libgnss++/algorithms/`
- `apps/`

Experimental work lives under:

- `experiments/`

Promotion rule:

- an experiment only graduates after it wins or ties on the shared metrics,
  and its structural heuristics are not worse than the current baseline.

## D-0004: Compare strategies with one shared interface

Status: accepted

Every `PPP-AR` experiment arm must share:

- the same observation file
- the same navigation file
- the same CLAS correction source
- the same reference position
- the same epoch limit
- the same metrics output format

This is enforced by `experiments/ppp_ar/run_experiments.py`.

## D-0005: Keep OSR pipeline arms in experiments until they beat the baseline on accuracy and fixed-epoch behavior

Status: accepted

The first seeded CLAS sweep is treated as a decision checkpoint, not a
promotion trigger.

Current interpretation:

- `OSR Float Pipeline` improves mean/p95 3D error over the `IFLC` control arm.
- `OSR AR Pipeline` matches that improvement and also produces fixed epochs.
- `OSR AR Pipeline` is now treated as a promotion candidate, not yet as the
  stable default.

Immediate rule:

- keep `IFLC Float Baseline` as the stable fallback
- keep `OSR Float Pipeline` inside `experiments/` as a trial candidate
- keep `OSR AR Pipeline` inside `experiments/` as a promotion candidate
- only promote to the stable default after repeated wins across a shared CLAS
  experiment suite and acceptable structural heuristics

## D-0006: Use repeated suite wins, not a single attractive dataset, as the promotion trigger

Status: accepted

The first two-case CLAS suite changed the interpretation of the seeded arms:

- `CLAS 2019-08-27` still favors the `OSR` arms.
- `CLAS 2018-11-24` regresses badly for the same arms.
- The suite result therefore downgrades both `OSR` arms from immediate
  promotion to `extended_trial`.

Immediate rule:

- do not promote an arm because one dataset looks strong
- keep the control arm stable until the candidate wins across the full suite
- treat failures in additional cases as design feedback, not as noise to hide

## D-0007: Keep CLAS atmosphere-token selection policy in the experiment lane

Status: accepted

The 2018/2019 CLAS suite was re-run with selector-only variants:

- `grid-first`
- `grid-guarded`
- `balanced`
- `freshness-first`

Outcome:

- `grid-guarded` and `balanced` collapse to the same result as `grid-first`
  on the current suite.
- `freshness-first` improves solution rate on the 2018 case, but it does so by
  switching to spatially mismatched networks and creates catastrophic 3D error.
- none of the selector-only arms move from `extended_trial` to promotion.

Immediate rule:

- keep selector policy configurable, but experimental
- do not promote a selector-only change into the stable PPP path
- shift the next experiment axis toward:
  - compact-SSR to expanded-SSR correctness
  - OSR builder behavior
  - CLAS observation/update boundaries

## D-0008: Keep CLAS epoch-boundary fallback policy in the experiment lane

Status: accepted

The next CLAS suite was re-run with explicit epoch-boundary arms:

- `strict-osr`
- `hybrid-standard-ppp`

Outcome:

- both `strict` and `hybrid` arms hold `100%` solution rate on the current
  2018/2019 suite, but both regress mean/p95 3D error relative to the control.
- both `strict` and `hybrid` arms produce `0` fixed epochs across the suite.
- `clas_hybrid_fallback_epochs` stays at `0` in every case, which means the
  current CLAS epoch path is not failing closed into the fallback boundary.
- the dominant error therefore still sits inside the accepted CLAS update path,
  not in the simple choice between `seed-only` and `standard-PPP fallback`.

Immediate rule:

- keep epoch-boundary policy configurable, but experimental
- do not promote `strict` or `hybrid` CLAS boundary arms into the stable PPP path
- shift the next experiment axis toward:
  - compact-SSR to expanded-SSR correctness
  - OSR builder / correction application semantics
  - CLAS measurement-update quality inside accepted epochs

## D-0009: Do not promote reduced CLAS correction-application semantics

Status: accepted

The next CLAS suite was re-run with accepted-path application variants:

- `full-osr`
- `orbit-clock-bias`
- `orbit-clock-only`

Outcome:

- both reduced-semantics arms hold `100%` solution rate on the current
  2018/2019 suite, but both are worse than `full-osr` on mean/p95 3D error.
- the degradation appears in both the `2019` and `2018` cases.
- `AR` does not recover under either reduced-semantics arm; fixed epochs stay `0`.
- this means the current failure is not explained by “too much CLAS
  atmosphere/bias application” inside the accepted update path.

Immediate rule:

- keep `full-osr` as the best current CLAS-style accepted-path experiment arm
- keep `orbit-clock-bias` and `orbit-clock-only` as negative-control experiments
- do not promote reduced correction semantics into the stable PPP path
- shift the next experiment axis toward:
  - compact-SSR to expanded-SSR correctness
  - OSR value construction before the measurement builder
  - CLAS phase-bias / continuity semantics before residual formation

## D-0010: Do not promote reduced CLAS phase-bias / continuity semantics

Status: accepted

The next CLAS suite was re-run with phase-continuity variants:

- `full-repair`
- `raw-phase-bias`
- `no-phase-bias`

Outcome:

- `raw-phase-bias` is effectively identical to the current `strict` CLAS arm on
  both the `2019` and `2018` cases.
- `no-phase-bias` moves the result by only a few centimeters on the current
  suite and does not recover fixed epochs.
- the `2018` failure remains catastrophic for every arm, which means the
  dominant error does not sit in the continuity-repair layer alone.

Immediate rule:

- keep `full-repair` as the best current CLAS-style continuity semantics
- keep `raw-phase-bias` and `no-phase-bias` as negative-control experiments
- do not promote reduced phase-continuity semantics into the stable PPP path
- shift the next experiment axis toward:
  - compact-SSR to expanded-SSR correctness
  - OSR value construction before residual formation
  - CLAS phase-bias value construction and continuity timing before repair

## D-0011: Do not promote clock-bound expanded-SSR timing policies

Status: accepted

The next CLAS suite was re-run with expanded-SSR timing variants:

- `lag-tolerant`
- `clock-bound-phase-bias`
- `clock-bound-atmos-and-phase-bias`

Outcome:

- `clock-bound-phase-bias` is effectively identical to the current `strict`
  CLAS arm on both the `2019` and `2018` cases.
- `clock-bound-atmos-and-phase-bias` improves mean/p95 3D error on the current
  suite, but it does so by collapsing solution rate from `99.58%` to `12.08%`.
- `AR` still does not recover under either timing arm; fixed epochs stay `0`.
- the dominant error therefore does not sit in simple carry timing alone.

Immediate rule:

- keep `lag-tolerant` as the best current timing policy inside the CLAS
  experiment lane
- keep `clock-bound-phase-bias` and `clock-bound-atmos-and-phase-bias` as
  negative-control timing experiments
- do not promote stricter timing binding into the stable PPP path
- shift the next experiment axis toward:
  - compact-SSR to expanded-SSR correctness
  - OSR value construction before residual formation
  - phase-bias value construction and continuity timing before repair

## D-0012: Keep compact-to-expanded row-emission policy in the experiment lane

Status: accepted

The next CLAS suite was re-run with expanded-SSR source variants:

- `lag-tolerant-union`
- `orbit-or-clock-only`
- `orbit-and-clock-only`

Outcome:

- `orbit-or-clock-only` is strictly worse than the current source policy on
  both the `2019` and `2018` cases.
- `orbit-and-clock-only` reduces the `2018` error tail relative to the current
  strict CLAS arm, but it still regresses badly against the stable control and
  also makes the `2019` case worse.
- `AR` does not recover under the source-policy arms in any stable way; only
  one arm produces a few fixed epochs and it does so while degrading the float
  solution further.
- the dominant failure therefore does not sit in simple zero-row emission alone.

Immediate rule:

- keep `lag-tolerant-union` as the current source policy inside the CLAS
  experiment lane
- keep `orbit-or-clock-only` and `orbit-and-clock-only` as negative-control
  source experiments
- do not promote stricter compact-to-expanded row emission into the stable PPP
  path
- shift the next experiment axis toward:
  - expanded SSR value construction before residual formation
  - compact subtype merge semantics before row materialization
  - phase-bias value construction and continuity timing before repair

## D-0013: Do not promote simple compact atmosphere-merge carry policies

Status: accepted

The next CLAS suite was re-run with compact atmosphere-merge variants:

- `stec-coeff-carry`
- `no-carry`
- `network-locked-stec-coeff-carry`

Outcome:

- after re-running the suite with the correct `2018-11-24` reference position,
  both `no-carry` and `network-locked-stec-coeff-carry` are effectively
  identical to the current `strict` CLAS arm on both the `2019` and `2018`
  cases.
- none of the atmosphere-merge arms improve solution rate, mean/p95 3D error,
  or fixed-epoch behavior relative to the current strict OSR pipeline.
- `AR` still does not recover under any of the merge-policy variants; fixed
  epochs stay `0`.
- the dominant failure therefore does not sit in simple subtype-8 STEC
  coefficient carry-forward alone.

Immediate rule:

- keep `stec-coeff-carry` as the current compact atmosphere-merge policy inside
  the CLAS experiment lane
- keep `no-carry` and `network-locked-stec-coeff-carry` as negative-control
  merge experiments
- do not promote alternative compact atmosphere carry policies into the stable
  PPP path
- shift the next experiment axis toward:
  - expanded SSR value construction before residual formation
  - compact subtype merge semantics beyond simple STEC coefficient carry
  - phase-bias value construction and continuity timing before repair

## D-0014: Do not promote residual-only or polynomial-only expanded atmosphere values

Status: accepted

The next CLAS suite was re-run with expanded value-construction variants:

- `full-composed`
- `residual-only`
- `polynomial-only`

Outcome:

- `polynomial-only` is effectively identical to the current `strict` CLAS arm
  on both the `2019` and `2018` cases.
- `residual-only` reduces mean 3D error relative to the current `strict` arm,
  but it does so by collapsing solution rate from `100%` to `45%` across the
  suite.
- `AR` does not recover under any of the value-construction arms; fixed epochs
  stay `0`.
- the dominant failure therefore does not sit in a simple “use only polynomial
  terms” or “use only residual terms” split.

Immediate rule:

- keep `full-composed` as the current expanded atmosphere-value construction
  policy inside the CLAS experiment lane
- keep `residual-only` and `polynomial-only` as negative-control value
  experiments
- do not promote reduced value-construction semantics into the stable PPP path
- shift the next experiment axis toward:
  - expanded SSR value construction details beyond all-or-nothing polynomial vs residual splits
  - compact subtype merge semantics beyond simple STEC coefficient carry
  - phase-bias value construction and continuity timing before repair

## D-0015: Do not promote simple compact subtype-precedence merge policies

Status: accepted

The next CLAS suite was re-run with compact subtype-precedence variants:

- `union`
- `gridded-priority`
- `combined-priority`

Outcome:

- both `gridded-priority` and `combined-priority` are effectively identical to
  the current `strict` CLAS arm on both the `2019` and `2018` cases.
- neither subtype-precedence arm improves solution rate, mean/p95 3D error, or
  fixed-epoch behavior relative to the current strict OSR pipeline.
- `AR` still does not recover under any of the subtype-precedence variants;
  fixed epochs stay `0`.
- the dominant failure therefore does not sit in simple “subtype 9 beats 8” or
  “subtype 12 replaces 8/9” precedence rules inside one pending compact epoch.

Immediate rule:

- keep `union` as the current compact subtype-merge policy inside the CLAS
  experiment lane
- keep `gridded-priority` and `combined-priority` as negative-control subtype
  experiments
- do not promote alternative subtype-precedence rules into the stable PPP path
- shift the next experiment axis toward:
  - expanded SSR value construction details inside subtype-12 atmospheric rows
  - compact subtype merge semantics beyond simple family replacement
  - phase-bias value construction and continuity timing before repair

## D-0016: Do not promote alternative expanded residual-sampling semantics

Status: accepted

The next CLAS suite was re-run with expanded residual-sampling variants:

- `indexed-or-mean`
- `indexed-only`
- `mean-only`

Outcome:

- `indexed-only` is effectively identical to the current `strict` CLAS arm on
  both the `2019` and `2018` cases.
- `mean-only` changes the `2019` case by only a few tenths of a millimeter and
  leaves the `2018` failure untouched.
- neither residual-sampling arm improves solution rate, mean/p95 3D error, or
  fixed-epoch behavior relative to the current strict OSR pipeline.
- `AR` still does not recover under any of the residual-sampling variants;
  fixed epochs stay `0`.
- the dominant failure therefore does not sit in the simple choice between
  indexed residual sampling and mean fallback once the expanded atmosphere
  values are already built.

Immediate rule:

- keep `indexed-or-mean` as the current expanded residual-sampling policy
  inside the CLAS experiment lane
- keep `indexed-only` and `mean-only` as negative-control residual-sampling
  experiments
- do not promote alternative residual-sampling semantics into the stable PPP
  path
- shift the next experiment axis toward:
  - subtype-12 value construction details before residual sampling
  - phase-bias value construction and continuity timing before repair
  - compact-to-expanded SSR semantics that are upstream of residual sampling

## D-0017: Do not promote reduced subtype-12 value-construction semantics

Status: accepted

The next CLAS suite was re-run with subtype-12 surface-term variants:

- `full`
- `planar`
- `offset-only`

Outcome:

- `planar` is effectively identical to the current `strict` CLAS arm on both
  the `2019` and `2018` cases.
- `offset-only` is also effectively identical to the current `strict` CLAS arm
  on both suite cases.
- neither subtype-12 arm improves solution rate, mean/p95 3D error, or
  fixed-epoch behavior relative to the current strict OSR pipeline.
- `AR` still does not recover under any of the subtype-12 variants; fixed
  epochs stay `0`.
- the dominant failure therefore does not sit in the simple choice between
  keeping subtype-12 constant-only, planar, or higher-order surface terms once
  the compact rows are already accepted into the expanded SSR lane.

Immediate rule:

- keep `full` as the current subtype-12 value-construction policy inside the
  CLAS experiment lane
- keep `planar` and `offset-only` as negative-control subtype-12 experiments
- do not promote reduced subtype-12 value-construction semantics into the
  stable PPP path
- shift the next experiment axis toward:
  - phase-bias value construction and continuity timing before repair
  - compact-to-expanded SSR semantics that are upstream of subtype-12 surface construction
  - source-row / subtype-12 row correctness before residual formation

## D-0018: Do not promote split phase-continuity semantics

Status: accepted

The next CLAS suite was re-run with phase-continuity split variants:

- `full-repair`
- `sis-continuity-only`
- `repair-only`
- `raw-phase-bias`
- `no-phase-bias`

Outcome:

- `sis-continuity-only` is effectively identical to the current `strict` CLAS
  arm on both the `2019` and `2018` cases.
- `repair-only` is also effectively identical to the current `strict` CLAS arm
  on both suite cases.
- `raw-phase-bias` remains identical to those split variants, and
  `no-phase-bias` only moves the result by a few millimeters.
- neither isolating SIS continuity deltas nor isolating repair-offset
  accumulation improves solution rate, mean/p95 3D error, or fixed-epoch
  behavior relative to the current strict OSR pipeline.
- `AR` still does not recover under any of the split continuity variants; fixed
  epochs stay `0`.
- the dominant failure therefore does not sit in the simple choice between
  SIS-delta injection and repair-offset accumulation once the same phase-bias
  values have already been accepted into the expanded SSR lane.

Immediate rule:

- keep `full-repair` as the current continuity policy inside the CLAS
  experiment lane
- keep `sis-continuity-only`, `repair-only`, `raw-phase-bias`, and
  `no-phase-bias` as negative-control continuity experiments
- do not promote split continuity semantics into the stable PPP path
- shift the next experiment axis toward:
  - phase-bias value construction and reference-time semantics before SIS/repair
    injection
  - compact-to-expanded SSR semantics that are upstream of the accepted
    phase-bias values
  - source-row correctness before phase-bias continuity state is updated

## D-0019: Do not promote split phase-bias value semantics

Status: accepted

The next CLAS suite was re-run with phase-bias value variants:

- `full`
- `phase-bias-only`
- `compensation-only`

Outcome:

- `phase-bias-only` is effectively identical to the current `strict` CLAS arm
  on both the `2019` and `2018` cases.
- `compensation-only` moves the `2019` case by only a few millimeters and
  leaves the `2018` failure untouched.
- neither value split improves solution rate, mean/p95 3D error, or fixed-epoch
  behavior relative to the current strict OSR pipeline.
- `AR` still does not recover under any of the split value variants; fixed
  epochs stay `0`.
- the dominant failure therefore does not sit in the simple choice between
  compact phase-bias values and derived phase-compensation values once both are
  accepted under the same continuity policy.

Immediate rule:

- keep `full` as the current phase-bias value-construction policy inside the
  CLAS experiment lane
- keep `phase-bias-only` and `compensation-only` as negative-control
  phase-bias value experiments
- do not promote split phase-bias value semantics into the stable PPP path
- shift the next experiment axis toward:
  - phase-bias reference-time semantics before SIS/repair injection
  - compact-to-expanded SSR semantics that are upstream of accepted phase-bias
    values
  - source-row correctness before phase-bias continuity state is updated

## D-0020: Do not promote alternate phase-bias reference-time semantics

Status: accepted

The next CLAS suite was re-run with phase-bias reference-time variants:

- `phase-bias-reference`
- `clock-reference`
- `observation-epoch`

Outcome:

- `clock-reference` is effectively identical to the current `strict` CLAS arm
  on both the `2019` and `2018` cases.
- `observation-epoch` is also effectively identical to the current `strict`
  CLAS arm on both suite cases.
- suite metrics stay unchanged across all three variants:
  - solution rate `50.0%`
  - mean 3D `4.898 m`
  - p95 `5.134 m`
  - fixed epochs `0`
- ambiguity fixing still does not recover under any of the alternate
  reference-time variants.
- the dominant failure therefore does not sit in the simple choice between
  binding accepted phase-bias semantics to the compact phase-bias message time,
  the selected clock epoch, or the observation epoch.

Immediate rule:

- keep `phase-bias-reference` as the current reference-time policy inside the
  CLAS experiment lane
- keep `clock-reference` and `observation-epoch` as negative-control
  reference-time experiments
- do not promote alternate phase-bias reference-time semantics into the stable
  PPP path
- shift the next experiment axis toward:
  - compact-to-expanded SSR semantics upstream of accepted phase-bias values
  - source-row correctness before phase-bias continuity state is updated
  - phase-bias continuity-state construction beyond simple time rebinding

## D-0021: Do not promote alternate compact phase-bias merge semantics

Status: accepted

The next CLAS suite was re-run with compact phase-bias merge variants:

- `latest-union`
- `message-reset`
- `selected-mask-prune`

Outcome:

- `message-reset` is effectively identical to the current `strict` CLAS arm on
  both the `2019` and `2018` cases.
- `selected-mask-prune` is also effectively identical to the current `strict`
  CLAS arm on both suite cases.
- suite metrics stay unchanged across all three variants:
  - solution rate `50.0%`
  - mean 3D `4.898 m`
  - p95 `5.134 m`
  - fixed epochs `0`
- ambiguity fixing still does not recover under either compact phase-bias merge
  variant.
- the dominant failure therefore does not sit in the simple choice between
  union-carrying stale subtype-5/6 phase-bias rows, resetting them per
  message, or pruning them to the selected subtype-6 satellite mask before
  expanded SSR materialization.

Immediate rule:

- keep `latest-union` as the current compact phase-bias merge policy inside the
  CLAS experiment lane
- keep `message-reset` and `selected-mask-prune` as negative-control compact
  merge experiments
- do not promote alternate compact phase-bias merge semantics into the stable
  PPP path
- shift the next experiment axis toward:
  - compact-to-expanded SSR semantics upstream of accepted phase-bias values
  - source-row correctness before phase-bias continuity state is updated
  - accepted phase-bias value construction beyond simple compact merge policy

## D-0022: Do not promote alternate compact phase-bias source precedence

Status: accepted

The next CLAS suite was re-run with compact phase-bias source-precedence
variants:

- `arrival-order`
- `subtype5-priority`
- `subtype6-priority`

Outcome:

- `subtype5-priority` is effectively identical to the current `strict` CLAS arm
  on both the `2019` and `2018` cases.
- `subtype6-priority` is also effectively identical to the current `strict`
  CLAS arm on both suite cases.
- suite metrics stay unchanged across all three variants:
  - solution rate `50.0%`
  - mean 3D `4.898 m`
  - p95 `5.134 m`
  - fixed epochs `0`
- ambiguity fixing still does not recover under either alternate source
  precedence variant.
- the dominant failure therefore does not sit in the simple choice between
  following raw arrival order, forcing dedicated subtype-5 rows to win, or
  forcing subtype-6 code/phase-bias rows to win before expanded SSR
  materialization.

Immediate rule:

- keep `arrival-order` as the current compact phase-bias source precedence
  inside the CLAS experiment lane
- keep `subtype5-priority` and `subtype6-priority` as negative-control source
  precedence experiments
- do not promote alternate compact phase-bias source precedence into the stable
  PPP path
- shift the next experiment axis toward:
  - compact-to-expanded SSR semantics upstream of accepted phase-bias values
  - source-row correctness before phase-bias continuity state is updated
  - accepted phase-bias value construction beyond simple source precedence

## D-0023: Do not promote alternate network phase-bias composition semantics

Status: accepted

The next CLAS suite was re-run with network phase-bias composition variants:

- `direct-values`
- `base-plus-network`
- `base-only-if-present`

Outcome:

- `base-plus-network` is effectively identical to the current `strict` CLAS arm
  on both the `2019` and `2018` cases.
- `base-only-if-present` is also effectively identical to the current `strict`
  CLAS arm on both suite cases.
- suite metrics stay unchanged across all three variants:
  - solution rate `50.0%`
  - mean 3D `4.898 m`
  - p95 `5.134 m`
  - fixed epochs `0`
- ambiguity fixing still does not recover under either alternate network
  phase-bias composition variant.
- the dominant failure therefore does not sit in the simple choice between
  using subtype-6 network phase-bias rows directly, adding them on top of base
  rows, or replacing them with base rows before expanded SSR materialization.

Immediate rule:

- keep `direct-values` as the current network phase-bias composition policy
  inside the CLAS experiment lane
- keep `base-plus-network` and `base-only-if-present` as negative-control
  materialization experiments
- do not promote alternate network phase-bias composition semantics into the
  stable PPP path
- shift the next experiment axis toward:
  - compact-to-expanded SSR semantics that are upstream of accepted
    phase-bias values
  - source-row correctness before phase-bias continuity state is updated
  - code/phase bias bank materialization semantics beyond simple base-plus-network

## D-0024: Do not promote alternate compact phase-bias bank lookup semantics

Status: accepted

The next CLAS suite was re-run with compact phase-bias bank-lookup variants:

- `pending-epoch`
- `same-30s-bank`
- `close-30s-bank`
- `latest-preceding-bank`

Outcome:

- under the `base-plus-network` materialization lane, `same-30s-bank`,
  `close-30s-bank`, and `latest-preceding-bank` are all effectively identical
  to the current `strict` CLAS arm on both the `2019` and `2018` cases.
- suite metrics stay unchanged across the current bank-lookup variants:
  - solution rate `50.0%`
  - mean 3D `4.898 m`
  - p95 `5.134 m`
  - fixed epochs `0`
- ambiguity fixing still does not recover under any of the alternate bank
  lookup semantics.
- together with `D-0023`, this means the dominant failure does not sit in the
  simple choice between staying inside the pending epoch, requiring the exact
  30-second anchor, using the closest preceding 30-second bank, or reusing the
  latest preceding base bank before expanded SSR materialization.

Immediate rule:

- keep `pending-epoch` as the current compact phase-bias bank default inside
  the CLAS experiment lane
- keep `same-30s-bank`, `close-30s-bank`, and `latest-preceding-bank` as
  negative-control materialization experiments
- do not promote alternate compact phase-bias bank lookup semantics into the
  stable PPP path
- shift the next experiment axis toward:
  - compact-to-expanded SSR semantics that are upstream of accepted
    phase-bias values
  - source-row correctness before phase-bias continuity state is updated
  - row/value construction beyond simple bank lookup semantics
