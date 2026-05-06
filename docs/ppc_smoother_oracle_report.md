# PPC Smoother Oracle Report

This note records the 2026-05-05 PPC smoother-stack sweep results. The headline
numbers are useful for local research, but they are not private-score-equivalent:
the final segment combiner uses PPC `reference.csv` to choose the best candidate
per official scoring segment.

## Headline

The best local full-reference ceiling found in this sweep is:

| stack | weighted PPC official score |
|---|---:|
| `next_interp_natural_n60_age30_a12p0_l6p0_sim` | **91.712675%** |
| `next_interp_akima_n60_age30_a12p0_l6p0_sim` | 91.710637% |
| `next_interp_pchip_n60_age30_a12p0_l6p0_sim` | 91.689011% |
| `next_interp_linear_extrap_n60_age30_a12p0_l6p0_sim` | 91.665764% |
| `v237_sim` baseline | 91.647619% |

The best smoother stack is only +0.065056 pp above `v237_sim`, and only
+0.002038 pp above the previous `akima` interpolation stack.

## Current Best Breakdown

`next_interp_natural_n60_age30_a12p0_l6p0_sim` per-run scores:

| run | score | vs `v237_sim` |
|---|---:|---:|
| `tokyo_run1` | 93.585367% | +0.000000 pp |
| `tokyo_run2` | 96.077700% | +0.019488 pp |
| `tokyo_run3` | 95.425733% | +0.048142 pp |
| `nagoya_run1` | 86.869362% | +0.000000 pp |
| `nagoya_run2` | 76.277661% | +0.229824 pp |
| `nagoya_run3` | 86.362224% | +0.301058 pp |

The largest local oracle movements are `nagoya_run2` and `nagoya_run3`.

The best-stack 2D trajectories below are colored by GNSS solution status:

![PPC smoother best-stack 2D trajectory colored by GNSS status](ppc_smoother_status_trajectories.png)

For comparison, the `bsr_demo5_cont` baseline with the same status colors:

![PPC demo5-cont 2D trajectory colored by GNSS status](ppc_smoother_status_trajectories_demo5.png)

## Mode Checks

The current ceiling depends on the regressor-gated anchor and loss masks. More
deployable anchor or loss choices did not improve the local ceiling:

| geometry | anchor mode | loss mode | weighted score | vs best |
|---|---|---|---:|---:|
| natural n60/a12/l6 | regressor | regressor | **91.712675%** | baseline |
| natural n60/a12/l6 | regressor | learned `proba < 0.3` | 91.698204% | -0.014471 pp |
| natural n60/a12/l6 | regressor | internal `ratio < 6` | 91.676510% | -0.036165 pp |
| natural n60/a12/l6 | learned `proba >= 0.4` | regressor | 91.671050% | -0.041625 pp |
| natural n60/a12/l6 | internal quality | regressor | 91.647619% | -0.065056 pp |

The `internal quality` anchor check used `ratio >= 10`, post-suppression RMS
`<= 0.05 m`, and at least 8 satellites.

## Blind Review

The local ceiling above is oracle-only. It is produced by a segment selector
that scores each candidate against `reference.csv` and then chooses the
reference-best candidate for the official segment. That is not deployable on a
private test set.

Truth-blind leave-one-run-out checks were run on the akima stack:

| check | holdout gain |
|---|---:|
| oracle-selected 17-candidate categorical rules | 0.000000 pp |
| candidate-label categorical rules | 0.000000 pp |
| restricted numeric quality probe | 0.000000 pp |
| learned GBDT selector, all rows | 0.000000 pp |
| learned GBDT selector, changed rows only | 0.000000 pp |
| full 485-candidate active-pool categorical streaming LOO | 0.000000 pp |

The full active-pool categorical rule repeatedly learned
`status_transition == NO_SOLUTION->status_5`, but the holdout gain was exactly
zero in all six folds. The natural stack was not separately blind-validated; its
increment over akima is only +0.002038 pp, so the akima blind failure is the
right caution signal.

## Deployable-Only Check

The selector tooling now supports `--selection-mode priority_first`, which fixes
selection to candidate rules plus priority order. In that mode, the PPC reference
is still used to evaluate the final output, but it is not used to choose the
candidate for a segment, and the reference-scored non-regression gate is skipped.

A fully internal smoother probe on the natural n60/a12/l6 geometry used:

- anchor: `status == FIXED`, `ratio >= 10`, post-suppression RMS `<= 0.05 m`,
  and at least 8 satellites
- loss: non-FIX or FIXED with `ratio < 6`

That deployable-only fixed check produced no public-score gain over the fixed
baseline:

| check | weighted PPC official score |
|---|---:|
| `v237_sim` baseline | 91.647619% |
| deployable internal smoother fixed recipe | 91.647619% |

The distance-weighted aggregate printed by the probe was likewise unchanged:
91.616709% -> 91.616709% (delta +0.000 m).

## Takeaway

The smoother stack is a useful local oracle ceiling exercise, but it is saturated
for practical purposes:

- `natural` is the best interpolation geometry, but only barely beats `akima`.
- Switching anchor or loss gates away from the regressor setup lowers the score.
- Existing truth-blind LOO checks do not preserve the local oracle gains.
- The deployable-only fixed internal smoother check preserves no gain over
  `v237_sim`.

Do not treat the 91.712675% number as a PPC private-score estimate. Use it as an
upper-bound diagnostic for future selector research.

## Tooling Added

The sweep uses these scripts:

- `scripts/run_ppc_natural_spline_loss_interp_probe.py`
- `scripts/analyze_ppc_profile_segment_delta.py --write-all-segments`
- `scripts/analyze_ppc_segment_selector_loro_blind.py`
- `scripts/analyze_ppc_segment_selector_loro_blind_pandas.py`
- `scripts/analyze_ppc_segment_selector_loro_blind_stream.py`
- `scripts/analyze_ppc_segment_selector_loro_learned.py`
- `scripts/apply_ppc_multi_candidate_selector.py --selection-mode priority_first`
