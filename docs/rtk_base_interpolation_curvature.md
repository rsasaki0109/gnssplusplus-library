# Base interpolation dropped the range curvature

This is a follow-up to the [ambiguity consistency diagnostics](rtk_ambiguity_consistency_diagnostics.md).
That round found long-locked ambiguities drifting off integer. This round
looked for the cause. It found a defect in how `gnss_solve` and
`gnss_fuse` align the 1 Hz base to a faster rover. The defect is real, but
it is **not** the main source of the drift. The fix misses the frozen default
gate by noise-level margins, so it is the opt-in `gnss_solve
--base-interp-curvature`. **Defaults are unchanged.** Record:
[JSON](urbannav_base_interpolation_evaluation.json).

## The defect

`interpolateBaseEpoch()` in `apps/native/rtk_base_epoch_align.hpp` is meant
to interpolate the residual to the modeled range, not the raw observable.
However, it also interpolated the modeled range linearly between the two
base epochs:

```
target_m = before_m + alpha (after_m - before_m)
L_target = (target_m + linear(L lambda - modeled)) / lambda
         = linear(L)                       (raw linear interpolation)
```

The result therefore drops the range curvature. `gnss_live` and
`examples/rtk_positioning` evaluate the modeled range at the target time, but
the shared helper did not.

The error is zero at a base epoch and largest mid-interval. A 1 Hz truth
analysis therefore cannot see it, while the solver meets it at most rover
epochs. With a 10 Hz Trimble rover, 11,159 of 12,399 Odaiba epochs and 18,710
of 20,791 Shinjuku epochs are interpolated.

On the Odaiba base (geometry only, el ≥ 10°, no GLONASS):

| Quantity | p50 | p95 | max |
| --- | ---: | ---: | ---: |
| undifferenced, mid-interval | 2.4 mm | 14.7 mm | 20.9 mm |
| DD, all rover offsets | 4.9 mm | 12.4 mm | 27.2 mm |

Half of the interpolated DD values are off by more than 5 mm (0.03 L1
cycles).

## Fix and tests

`interpolateBaseEpoch(..., model_range_at_target)` (`gnss_solve
--base-interp-curvature`) applies the fix. It was built in two steps:

1. **Evaluate the modeled range at the target time** (`e109f77`).
   `RtkBaseAlignmentTest.InterpolationReproducesObservationsGeneratedAtTargetTime`
   checks it on the synthetic scene. The interpolated observation must match
   one generated at the target time. The worst error is 15.2 mm on develop
   and below 5e-8 m after the fix.
2. **Use one ephemeris for all three epochs** (`99fd099`). The real-data
   check below showed that step 1 alone is not enough. With independent
   ephemeris selection, a broadcast set switch inside the gap puts a
   decimetre orbit step into the observation. The helper now uses the
   ephemeris selected for the target time.
   `RtkBaseAlignmentTest.InterpolationIsNotShiftedByAnEphemerisSwitchInsideTheGap`
   adds a re-referenced ephemeris 0.8 s after toe. The error is 30–61 mm
   after step 1 and below 1 mm after step 2.

`RtkBaseAlignmentTest.DefaultInterpolationIsLinearInTheRawObservable` pins
the default to the legacy behaviour.

## Real-data check

Each real base epoch *t* is predicted from the real epochs *t−1* and *t+1*
(α = 0.5 over a 2 s gap) and compared with the phase observed at *t*. The
comparison uses between-satellite single differences (base clock removed),
el ≥ 15°, and excludes GLONASS and LLI-flagged rows. No reference trajectory
is involved.

| Base | Version | p50 | p95 | p99 | max | > 5 cm |
| --- | --- | ---: | ---: | ---: | ---: | ---: |
| Odaiba | develop | 22.0 mm | 52.7 mm | 56.1 mm | 69.4 mm | 5,002 |
| Odaiba | step 1 | 1.8 mm | 5.9 mm | 8.7 mm | 68.2 mm | 20 |
| Odaiba | **step 2** | 1.8 mm | 5.9 mm | 8.6 mm | 19.4 mm | 0 |
| Shinjuku | develop | 7.4 mm | 59.1 mm | 69.9 mm | 86.1 mm | 10,061 |
| Shinjuku | step 1 | 1.7 mm | 5.5 mm | 7.9 mm | 423.4 mm | 42 |
| Shinjuku | **step 2** | 1.7 mm | 5.5 mm | 7.7 mm | 17.9 mm | 0 |

The step 1 outliers (0.36–0.42 m) sit on J02 at TOW 282600–282601 and on C06
at 282614, which is where the broadcast set switches.

## What the long-lock steps are

The reference trajectory is used here for analysis only. On Odaiba, the truth
DD at the 1 Hz base epochs shows 11 times with persistent whole-cycle steps
(≥ 0.85 cycle, held for 5 s). At 9 of them the solver logged LLI slips or
ambiguity resets in the same second. At the two C04 B2I steps (+1 cycle at
273595, −1 cycle at 273604) no detector fired.

Missed slips therefore exist but are rare. They cannot explain why 62–97% of
correct fixes carry a pair with |z| > 3.

## Frozen evaluation

The rule was frozen before replay:
[JSON](urbannav_base_interpolation_evaluation.json) `frozen_rule_written_before_replay`.
Each run compares step 2 with develop `66383b5` under the same recipe.

| Rover | Drive | FIX | wrong >0.5/2/3 m | ref-correct lost | missing | H P95 m |
| --- | --- | ---: | ---: | ---: | ---: | ---: |
| Trimble | Odaiba | 1,298 → 1,298 | 0/0/0 → 0/0/0 | 0 | 1,426 → **1,429** | 4.7772 → 4.7769 |
| Trimble | Shinjuku | 122 → 122 | 0/0/0 → 0/0/0 | 0 | 3,234 → **3,238** | 8.3821 → 8.3662 |
| u-blox | Odaiba | 461 → 461 | 0/0/0 → 0/0/0 | 0 | 922 → 920 | 6.7021 → 6.6742 |
| u-blox | Shinjuku | 1 → 20 | 1/1/0 → 0/0/0 | — | 2,374 → 2,372 | 5.7359 → **5.7476** |

The table shows the default recipe. In the u-blox Shinjuku run, develop's
only FIX is the gross one. Its "right-censored" flag is false only because
develop never reaches a correct FIX.

HK Deep-Urban-1 (1 Hz rover and 1 Hz base, no interpolated epoch) is
byte-identical. HK Harsh-Urban-1 has one interpolated epoch, and two FLOAT
rows move by a few centimetres. The rule assumed HK would be untouched; that
assumption did not hold for Harsh.

**Gate: not passed.** The failures are missing epochs +3 and +4 on Trimble
(the individual missing epochs swap in both directions), +1.2 cm horizontal
P95 on u-blox Shinjuku, and HK Harsh. No wrong-FIX population grows under the
default recipe.

The opt-in recipes are descriptive only:

| Run | wrong >0.5/2/3 m | wrong episodes | ref-correct lost |
| --- | ---: | ---: | ---: |
| Trimble Odaiba age5_doppler3 | 157/3/0 → 121/0/0 | 37 → 23 | 2.2% |
| Trimble Shinjuku age5_doppler3 | 13/1/1 → 23/6/6 | 8 → 9 | 5.8% |
| Trimble Odaiba CD | 6/0/0 → 53/0/0 | 4 → 3 | 3.0% |
| Trimble Shinjuku CD | 4/0/0 → 10/0/0 | 3 → 5 | 0.4% |
| u-blox Odaiba age5_doppler3 / CD | 1/0/0 → 3/0/0 / 0 → 0 | | 2.6% / 0.2% |

The two large changes are single episodes:

- **Odaiba CD.** A 5 s span at TOW 273907–273912 is FIX in both runs. Its
  error is 7–34 cm on develop and 56–66 cm after the fix, so it crosses the
  0.5 m label.
- **Shinjuku age5_doppler3.** The new 22 m FIX at 282868 sits on a FLOAT
  solution that is 22 m off in both runs. A 4-pair AR passes with ratio 3.02
  after the fix and found no candidate on develop. The CD recipe's
  10-ambiguity floor rejects it.

**Mechanism.** These are age5_doppler3 accepted AR fixes, hold fixes
excluded:

| Drive | Group | cost/pair p50 | max\|z\| p50 | any \|z\|>3 |
| --- | --- | --- | --- | --- |
| Odaiba | correct | 4.73 → 4.44 | 4.02 → 3.99 | 62% → 60% |
| Shinjuku | correct | 64.3 → 68.4 | 6.57 → 6.21 | 92% → 93% |

The curvature error was a real systematic error of 1–3 cm, but it is not what
makes Qb overconfident.

## Decision

The frozen rule gives **NO_GO for a default change**, so the fix is
**opt-in** (`--base-interp-curvature`). Confirmed bugs that missed the gate
in earlier rounds were handled the same way: the iterated-update double
count and the hold overwrite.

The gate misses are noise-level, and no wrong-FIX population grows under the
default recipe. Two things argue against a default change:

- the opt-in recipes, including the hero clip's CD, move by single episodes;
- recorded results such as [the hero clip note](rtk_hero_clip_fix.md) would
  no longer reproduce.

Without the flag, `gnss_solve` POS is byte-identical to develop `66383b5` on
all 14 runs of this evaluation. With the flag, it is byte-identical to the
step 2 runs scored above. `gnss_fuse` keeps the default.

Not changed:

- `gnss_live` and `examples/rtk_positioning` (the AGENTS.md refactor-gate
  tool) evaluate the target range with independent ephemeris selection.
- `rtk_base_alignment::holdBaseEpoch` does the same for its two epochs.

All three can still see a set switch inside the gap.

Next: the remaining overconfidence on long-locked ambiguities. Candidate
causes are multipath absorbed into the float ambiguity, and the 1e-8
cycle²/s ambiguity process noise. Separate them on the truth DD at 10 Hz,
using the corrected interpolation as the base, before proposing any rule.

## Reproduce

1. Build develop `66383b5` in a worktree and this branch, both Release.
   The candidate is this branch with `--base-interp-curvature`.
2. For each rover ∈ {`rover_trimble.obs`, `rover_ublox.obs`}, drive ∈
   {Odaiba, Shinjuku} and recipe, run the native argv listed in the JSON
   `native_argv`. Run HK Deep/Harsh as well; their data are prepared as in
   [the HK holdout note](online_pva_default_switch_holdout_v2.md).
3. Score the Trimble runs with `evaluate_urbannav_fix_continuity.score()`.
   Score the u-blox runs on the 0.2 s grid with 5 ms matching, as in the
   [hero clip note](rtk_hero_clip_fix.md).
4. `run_tests --gtest_filter='RtkBaseAlignmentTest.*'`.
