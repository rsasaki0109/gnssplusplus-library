# Multi-epoch ambiguity-resolution shadow audit

## Scope

This experiment asks whether repeated integer candidates on an uninterrupted
double-difference ambiguity arc provide a safe, truth-free way to recover
FLOAT epochs. It is deliberately independent of the shipped estimator: the
shadow cannot add graph factors, pin ambiguities, change the selected partial
AR subset, or alter the reported position and FIX/FLOAT status.

The implementation is not a TDCP-specific change. It uses the fixed-lag
ambiguity marginal already produced by tightly coupled carrier-phase/IMU FGO.
An ambiguity enters the counterfactual subset only after its integer candidate
is identical for three consecutive epochs on the same internal arc index.
At least four such ambiguities are required. The current joint covariance is
then reduced to that subset and a fresh two-candidate LAMBDA search reports:

- persistent ambiguity count and minimum history support;
- ratio and bootstrapped success-rate lower bound;
- whether the fresh search agrees with the historical integers;
- counterfactual ECEF position and its float/IMU-prediction separation.

Enable it in the parity harness with `--multiepoch-ar-shadow`. The default is
off.

## Research and OSS audit

- The ION GNSS+ 2024 multi-epoch FGO-RTK study compares single-epoch AR,
  ambiguity merging, and multi-epoch AR with single-differenced ambiguity
  constraints. It motivates keeping arc identity and window history explicit,
  rather than treating repeated solution labels as independent votes:
  [Li et al., 2024](https://www.ion.org/publications/abstract.cfm?articleID=19805).
- Integer-aperture theory requires controlling the probability of an incorrect
  integer output, not merely maximizing availability. A repeated candidate is
  therefore telemetry, not sufficient evidence for promotion:
  [Teunissen, 2003](https://doi.org/10.1007/s00190-002-0299-9).
- RTKLIB delays ambiguity holding until a configurable number of consecutive
  validated fixes and resets that evidence when FIX is lost. This supports
  tracking persistence separately from the per-epoch ratio test:
  [RTKLIB `rtkpos.c`](https://github.com/tomojitakasu/RTKLIB/blob/master/src/rtkpos.c).
- GICI-LIB uses partial ambiguity resolution and validates the constrained
  graph by rejecting an integer update when the range cost worsens. This
  supports requiring an independent post-fix witness before activation:
  [GICI-LIB ambiguity resolution](https://github.com/chichengcn/gici-open/blob/master/src/gnss/ambiguity_resolution.cpp).
- GraphGNSSLib demonstrates windowed DD pseudorange/carrier/Doppler FGO with
  LAMBDA, but does not supply a production-grade integer-aperture activation
  rule that can replace dataset validation here:
  [GraphGNSSLib](https://github.com/weisongwen/GraphGNSSLib).

The audited stash was not applied wholesale. Its public LAMBDA diagnostics,
stale-key filtering, bootstrapped success rate, and solution-separation ideas
are already present on `develop` in a more complete form. Adaptive covariance
scaling and held-only anchor mutation were excluded from this isolated
multi-epoch experiment.

## Validation protocol

The shipped Tokyo preset is replayed on PPC Tokyo runs 1, 2, and 3. The
baseline and shadow must have identical status, ECEF position, ratio, fixed
ambiguity count, and AR outcome at every epoch. Candidate correctness is used
only offline: a counterfactual candidate is classed correct when its 3D error
is below 0.5 m. Activation may be considered only if a truth-free rule adds no
wrong fixes on any run; otherwise the feature remains monitor-only.

## Results

The full shadow replays produced the following counterfactual candidates.
“FLOAT correct/wrong” counts only epochs where the shipped solution remained
FLOAT, so these are the potential additions to fix rate.

| Run | Epochs | All candidates correct/wrong | FLOAT correct/wrong | FLOAT ratio > 3 correct/wrong |
|---|---:|---:|---:|---:|
| Tokyo run1 | 11,905 | 4,608 / 2,859 | 293 / 1,024 | 87 / 170 |
| Tokyo run2 | 9,147 | 7,139 / 255 | 114 / 164 | 57 / 45 |
| Tokyo run3 | 15,294 | 10,785 / 384 | 413 / 233 | 17 / 29 |

The baseline and shadow CSVs have the same row count on every run. Status,
ECEF position, reported ratio, fixed ambiguity count, and AR outcome are
identical in every one of the 36,346 rows. A separate 500-epoch current-build
A/B replay also had zero differing rows. This confirms that the implementation
is diagnostic-only.

Bootstrapped success rate does not rescue the rule: at a threshold of 0.999 it
accepted 1,314/1,317, 273/278, and 645/646 FLOAT candidates on runs 1, 2, and
3 respectively, including the wrong integer basins. Exact integers can remain
stable for many epochs when the underlying urban measurement model is biased.

A threshold grid was selected using runs 1 and 2 only, then checked on run 3.
The best zero-wrong training family required ratio above 20, float separation
at most 0.05 m, IMU-prediction separation at most 0.1 m, and a fresh SPP
witness within 5 m (the default three-epoch/four-ambiguity requirements still
apply). It recovered only three correct epochs in the training runs and three
correct epochs on held-out run 3. Six epochs over 36,346 is about **0.0165
percentage points**, far below a meaningful fix-rate improvement, and the
threshold was selected after a grid search. It is not promoted to a shipping
gate.

The diagnostic also has measurable cost when enabled. Solver wall time was
463.520→566.505 s, 584.568→543.092 s, and 844.915→930.754 s for runs 1–3;
summed wall time increased by about 7.8% (individual runs include normal timing
noise). Since the default is off, shipped runtime is unchanged.

## Decision

Keep `--multiepoch-ar-shadow` monitor-only. Do not increase fix rate from
temporal integer agreement, ratio, or BSR alone. The next activation experiment
should add an independent observation-domain witness, such as the constrained
graph range-cost check used by GICI-LIB or a held-out carrier residual that was
not part of the LAMBDA marginal. Any future gate must again be selected without
run3 truth and then validated on a held-out course.

## Held-out carrier witness follow-up

The follow-up implements the second experiment without changing estimator
output. For each multi-epoch candidate, it passes only the ambiguities in the
persistent subset to the existing surplus-satellite validator. Carrier rows
outside that subset are therefore re-differenced against an alternate
reference and evaluated at the candidate position without contributing an
integer or covariance entry to the reduced LAMBDA search. The CSV fields are:

- `multiepoch_ar_surplus_eval`: an independent pool was large enough to decide;
- `multiepoch_ar_surplus_pass`: every required held-out check passed;
- `multiepoch_ar_surplus_level`: constellation fallback, 0 for the strongest
  GQEBR pool through 5 for GQ;
- `multiepoch_ar_surplus_used`: number of held-out satellites in the deciding
  pool.

This follows the fixed-hypothesis validation direction used by
[GICI-LIB](https://github.com/chichengcn/gici-open/blob/master/src/gnss/ambiguity_resolution.cpp),
which rejects a constrained update when range cost increases, and
[RTKLIB](https://github.com/tomojitakasu/RTKLIB/blob/master/src/rtkpos.c),
which recomputes and validates fixed-solution post-fit residuals. The important
difference is explicit sample splitting: the new verdict uses carrier rows
whose ambiguity was not fixed by the multi-epoch search.

The shipping preset was replayed over all of Tokyo runs 1 and 2. Correct and
wrong below classify counterfactual FLOAT candidates by 3D error at or below,
or above, 0.5 m. The shadow continued to leave status, ECEF position, ratio,
fixed ambiguity count, and AR outcome unchanged.

| FLOAT candidate filter | Run1 correct / wrong | Run2 correct / wrong |
|---|---:|---:|
| All multi-epoch candidates | 293 / 1,024 | 114 / 164 |
| Held-out verdict available | 264 / 856 | 102 / 144 |
| Held-out verdict passed | 118 / 230 | 34 / 33 |
| Strongest GQEBR pool passed | 100 / 184 | 25 / 27 |
| Run1-selected gate | **45 / 0** | **3 / 15** |

The run1-selected gate required the strongest GQEBR pool, ratio above 50, and
candidate/IMU-prediction separation at most 0.1 m. It looked safe in development
but failed immediately on run2, where 15 of 18 accepted candidates were wrong
and the maximum candidate error was 1.755 m. An exploratory run1+2 grid adding
fresh-SPP separation found no non-empty zero-wrong rule. Run3 remains an
unconsumed holdout for this follow-up because the candidate already failed its
validation course.

Decision: retain the held-out verdict as diagnostic evidence, but do not add a
multi-epoch rescue switch or change the shipping FIX rate. Independent carrier
rows can share the same urban multipath basin and are not, by themselves, a
safe integer-aperture witness.
