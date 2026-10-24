# Iterated RTK update: repeated rows and covariance

The RTK epoch runs `updateFilter()` more than once (`kf_iterations`, capped
at two passes in kinematic mode once a previous solution exists). Each later
pass rebuilt the rows at the updated state and applied them as a new Kalman
update to the previous pass's **posterior** state and covariance. Relinearizing
the same measurements this way counts them again. This was the open hypothesis
in the [handoff](claude_handoff_gnss_fix.md); it is now reproduced, but fixing
it alone does not pass the full-data adoption gate. The default is unchanged.

## Reproduction

`RTKIteratedUpdateTest.RelinearizationFromPriorDoesNotRecountEpochMeasurements`
(`tests/test_rtk_legacy.cpp`) uses the noise-free synthetic scene with an
unbiased SPP seed (atmosphere models off, as the scene has none), so every
code and phase row passes the prefit gate on the first pass and the problem is
effectively linear. Three static epochs are processed with one pass and with
the default iterated passes.

Before any change, the iterated run's position variances and every
double-difference ambiguity variance were 0.50-0.51 of the single-pass values
(the first form of the test failed with ratio 0.5023-0.5054). The second pass
added no information but halved the covariance. The test now pins this legacy
behaviour at 0.5 and requires the opt-in mode to match the single pass within
1e-3, with the same DD ambiguity estimates.

The first pass is not redundant: with a biased seed (atmosphere models on, about
4.7 m prefit RMS) the 3 m prefit gate suppresses 14 of 18 rows and only the
relinearized pass admits them. A correct fix therefore has to keep the pass,
not drop it.

## Opt-in implementation

`--kf-relinearize-from-prior` (`RTKConfig::kf_relinearize_from_epoch_prior`)
snapshots the epoch prior (x0, P0) before the first pass. A later pass:

1. builds rows at the current iterate x_i and applies the ordinary outlier
   thresholds (code/phase 3 m, Doppler its own threshold) to those residuals;
2. re-expresses each kept row about the prior, r0' = r_i + H (x_i - x0);
3. applies them once to (x0, P0) with no second gate.

In a linear model this reproduces the first-pass solution (iterated EKF /
Gauss-Newton). Residual RMS/max diagnostics keep their meaning (residuals at
the pass's linearization point after its gate); NIS is that of the update
actually applied. A failed pass leaves the previous iterate, as before. With
the option off, the solver, POS and telemetry are unchanged.

## Public-data evaluation: NO_GO for default

Same inputs, surveyed base, exact week/TOW matching and scoring-only reference
as the [continuity investigation](urbannav_doppler_consensus.md). The decision
rule (the [fix-recovery adoption gate](fix_recovery_guard.md) applied per run)
was written down before the replay. Nothing was retuned afterwards.

`scripts/analysis/evaluate_urbannav_fix_continuity.py` replayed the baseline
(`35654026`) and the experiment as the default ([patch](urbannav_iterated_update_experiment.patch),
SHA-256 equal to the replay manifest's source diff). The
[audit](urbannav_iterated_update_evaluation.json) holds all eight runs,
hashes, argv, recovery and censoring.

| Drive | Recipe | Version | FIX | FIX >0.5 / >2 / >3 m | Missing | H P95 m | Common-epoch H P95 m |
| --- | --- | --- | ---: | ---: | ---: | ---: | ---: |
| Odaiba | default | before | 1,298 | 0 / 0 / 0 | 1,426 | 4.7772 | 4.7265 |
| Odaiba | default | after | 1,298 | 0 / 0 / 0 | 1,515 | 5.1399 | 5.1181 |
| Odaiba | age5_doppler3 | before | 4,256 | 157 / 3 / 0 | 1,480 | 4.8963 | 4.8057 |
| Odaiba | age5_doppler3 | after | 4,297 | 103 / 3 / 0 | 1,562 | 5.2171 | 5.2072 |
| Shinjuku | default | before | 122 | 0 / 0 / 0 | 3,234 | 8.3821 | 7.9090 |
| Shinjuku | default | after | 122 | 0 / 0 / 0 | 3,451 | 1.9650 | 1.8222 |
| Shinjuku | age5_doppler3 | before | 1,949 | 13 / 1 / 1 | 3,332 | 8.3432 | 7.9090 |
| Shinjuku | age5_doppler3 | after | 3,972 | 516 / 125 / 15 | 3,581 | 1.8668 | 1.5853 |

`age5_doppler3` = `--max-fixed-anchor-age 5 --max-fixed-doppler-consensus 3`,
not a recommended recipe. The evaluator's `baseline_correct_fix_lost` counts
correct FIX epochs of the *default-recipe* baseline that a run no longer has;
by that measure nothing is lost, and the default FIX populations are the same
size. Against the same-recipe baseline, however, the age5_doppler3 experiment
loses 15 (Odaiba) and 309 (Shinjuku) correct FIX epochs at 0.5 m, and 23 and
253 at 2 m ([hold follow-up audit](urbannav_hold_update_evaluation.json)). But missing output rises
in all four runs, horizontal P95 worsens on both Odaiba recipes, Odaiba
age5_doppler3 recovery P95 at 0.5 m rises from 186.7 s to 298.5 s, and
Shinjuku age5_doppler3 wrong FIX grows by an order of magnitude. Shinjuku's
default P95 falls from 8.38 m to 1.97 m, but that does not offset the failed
conditions. **Decision: keep the option off by default.**

The opt-in build was then replayed on all eight recipes: with the flag off every
POS is byte-identical to the baseline, and with the flag on every POS is
byte-identical to the experiment run above.

## What the wrong FIX populations look like

Joining the `--debug-epoch-log` of the age5_doppler3 runs with the reference
(scoring only):

| Run | FIX >0.5 m: hold / AR | AR pairs median (≤6 share) | AR ratio median | AR float→fixed median m |
| --- | ---: | ---: | ---: | ---: |
| Odaiba before, wrong | 74 / 83 | 6 (93%) | 6.4 | 0.92 |
| Odaiba before, correct | 946 / 3,153 | 18 (2%) | 27.9 | 0.01 |
| Shinjuku before, wrong | 4 / 9 | 8 (44%) | 5.6 | 1.46 |
| Shinjuku after, wrong | 234 / 282 | 10 (27%) | 12.4 | 0.03 |

With the legacy covariance most wrong AR fixes are low-redundancy candidates
whose integer solution moves the position about a metre from the float while
passing the ratio test. That integer correction is larger than the claimed
precision. With the corrected covariance, the Shinjuku wrong population changes
character: the float itself already sits in the wrong basin (0.03 m from the
fixed solution) and nearly half are hold fixes. Within the 283274-283289
interval, 145 consecutive wrong AR epochs agree with the Doppler track, because
the track re-anchors on each accepted FIX. This is a relative check and cannot
detect a persistent basin error.

These are descriptive observations from already-inspected data, not validated
gates. A pair-count or float/fixed-separation threshold chosen from this table
would be the same kind of post-hoc tuning already rejected. The overconfident
legacy covariance is the calibration that the existing ratio/jump gates have
been working with. Making it correct changes which candidates reach them.

## Next questions (unverified)

- Hold-state evolution under the corrected covariance: `applyHoldAmbiguity()`
  sets satellite ambiguities to the held integers, zeroes their cross
  covariance (including with position) and sets 0.001 cycle². This is not a
  conditional update, and it does not move the position. Whether it locks
  wrong basins in the Shinjuku "after" run needs its own reproduction.
- Candidate acceptance with honest Qb: compare the LAMBDA success-rate and the
  float→fixed Mahalanobis distance for correct vs wrong candidates before
  proposing any frozen rule.
