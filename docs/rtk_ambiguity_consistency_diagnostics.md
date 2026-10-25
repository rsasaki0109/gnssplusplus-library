# Float-ambiguity consistency at accepted AR fixes

Follow-up to the [iterated update](rtk_iterated_update_covariance.md) and
[hold update](rtk_hold_ambiguity_update.md) rounds. Neither correcting the
covariance (R = `--kf-relinearize-from-prior`) nor the hold (H/HR) stopped
wrong AR candidates. This round only **measures** what the acceptance
statistics see. No solver behaviour changes.

## Diagnostics added

`--lambda-shadow-candidates N` already runs a read-only LAMBDA search on the
full DD set. It reports the bootstrapped success rate (BSR) and the best and
second costs `db' Qb⁻¹ db`. Four `--debug-epoch-log` columns are appended:

| Column | Meaning |
| --- | --- |
| `lambda_shadow_max_abs_z` | max over pairs of `|float − best integer| / sqrt(Qb_ii)` |
| `lambda_shadow_max_z_pair` | that pair, `REF-SAT/Lf` |
| `lambda_shadow_max_z_lock_count` | its `min(ref, sat)` lock count in epochs |
| `lambda_shadow_pairs_abs_z_gt3` | number of pairs with `|z| > 3` |

`RTKLambdaShadowTest.ReportsPerPairIntegerResidualOfBestCandidate` checks
them on the noise-free scene (all |z| < 3). With the shadow on, base/R/HR
age5_doppler3 POS files are byte-identical to the runs without it. Without the
flag, the default and age5_doppler3 POS files are byte-identical to develop.

## Accepted AR fixes, correct vs wrong

age5_doppler3, hold fixes excluded, reference used only to label 3D ≤ 0.5 m as
correct. Full record: [JSON](urbannav_ambiguity_consistency_diagnostics.json).

| Run | Group | AR FIX | BSR p10 | cost/pair p50 | χ² p<0.001 | max\|z\| p50 | any \|z\|>3 | max-z pair lock ≤10 | pairs p50 |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| Odaiba base | correct | 3,153 | 1.00 | 4.7 | 66% | 4.0 | 62% | 3% | 19 |
| Odaiba base | wrong | 83 | 1.00 | 77.1 | 98% | 10.2 | 88% | 0% | 6 |
| Shinjuku base | correct | 977 | 1.00 | 64.3 | 99% | 6.6 | 92% | 8% | 14 |
| Shinjuku base | wrong | 9 | 0.78 | 24.4 | 89% | 6.6 | 67% | 22% | 8 |
| Odaiba R | correct | 3,137 | 1.00 | 1.9 | 48% | 2.7 | 47% | 3% | 19 |
| Odaiba R | wrong | 75 | 1.00 | 33.9 | 99% | 2.8 | 48% | 0% | 6 |
| Shinjuku R | correct | 2,258 | 1.00 | 20.8 | 83% | 4.0 | 78% | 5% | 16 |
| Shinjuku R | wrong | 282 | 1.00 | 28.0 | 99% | 5.0 | 83% | 7% | 10 |
| Odaiba HR | correct | 3,174 | 1.00 | 88.8 | 100% | 16.1 | 97% | 1% | 18 |
| Shinjuku HR | correct | 3,315 | 1.00 | 136.7 | 100% | 19.4 | 98% | 3% | 17 |

## What this shows

1. **The model-based statistics are saturated.** BSR is about 1 for correct
   and wrong candidates alike. A consistent model would give a best cost per
   pair of about 1. Even correct fixes reach 2-137, and the χ² test of the
   best candidate rejects most correct fixes. Qb is overconfident by far more
   than the factor 2 removed by R, so ratio/BSR/χ² rules on this Qb cannot
   separate the populations. On Shinjuku R, correct and wrong are almost
   indistinguishable on every column.
2. **The inconsistency sits on long-locked ambiguities.** The pair with the
   largest |z| has a lock of at most 10 epochs in only 0-22% of cases (median
   lock 49-198 epochs). These are not fresh ambiguities with a large prior.
   Long-tracked SD ambiguities drift off integer by more than their shrinking
   variance allows: an undetected bias or slip, or multipath absorbed into the
   ambiguity, while process noise is 1e-8 cycle²/s.
3. **Wrong fixes are low-redundancy epochs, not subset picks.** The median
   selected/full pair ratio is 1.0. Wrong AR fixes come from epochs whose
   *full* set has 4-10 pairs, against 14-19 for correct fixes. With few pairs,
   one or two biased long-locked ambiguities can still pass the ratio test.
4. **The Kalman hold adds overconfidence.** Under HR, correct fixes have
   median |z| 16-19 and nearly all have pairs with |z| > 3. The pseudo-
   observation is applied again at every FIX epoch, so it plausibly counts the
   same integer information repeatedly. This is an observation and has not
   been reproduced.

The reference labels make this descriptive. It does not validate a gate.

## Next questions (unverified)

- Why long-locked ambiguities leave the integer grid. Compare the max-|z| pair
  with the slip detectors' near-misses (GF / Doppler / code-minus-phase
  margins), half-cycle flags and elevation. Separate undetected slips from slow
  biases before changing any detector or the ambiguity process noise.
- Whether repeated hold pseudo-observations (HR) are a second repeated count.
  Reproduce with a synthetic scene first, as for the iterated update.
- Only then propose one frozen acceptance rule. It should use redundancy or
  consistency evidence that does not depend on this Qb.
