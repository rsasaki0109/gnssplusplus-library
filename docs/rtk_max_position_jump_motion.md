# The max position jump check is motion-blind too

The [hero clip note](rtk_hero_clip_fix.md) showed two motion-blind
validation checks and added a Doppler-propagated reference to one of them
(`--fix-history-motion-compensation`, D). This round traces what is left of
the CD recipe's losses and wrong fixes. It finds the same defect in the
other check, and adds the same propagation there as the opt-in
`--max-position-jump-motion-compensation` (M). **Defaults are unchanged.**
Record: [JSON](urbannav_max_position_jump_motion_evaluation.json).

The reference trajectory is used only for scoring and 0.5 m labels.

## Combining the correctness fixes with CD does not help

Before any change, the recipe CDRI was frozen. CDRI is CD plus the two
confirmed-bug opt-ins:

- R, `--kf-relinearize-from-prior` ([iterated update](rtk_iterated_update_covariance.md));
- I, `--base-interp-curvature` ([base interpolation](rtk_base_interpolation_curvature.md)).

| Trimble wrong >0.5 m | CD | CD+R | CD+I | CDRI |
| --- | ---: | ---: | ---: | ---: |
| Odaiba | 6 | 43 | 53 | 38 |
| Shinjuku | 4 | 8 | 10 | 10 |

CDRI fails the gate on both Trimble drives. The u-blox holdout has no wrong
FIX under any variant.

## Where CD loses correct fixes

All 37 default-correct epochs that CD loses on Odaiba are in one span,
TOW 273505.9–273510.7. After the FIX at 273505.7, the vehicle moves at about
13 m/s. The candidates' distance from that FIX grows 1.33 m every 0.1 s.

`validateFixedSolution()` compares the candidate with the last FIX against
the absolute `max_position_jump_m` (5 m) for as long as the anchor is usable,
which is 5 s with `--max-fixed-anchor-age 5`. After 0.4 s every consistent
candidate is rejected:

- 30 of the 37 epochs fail with `max_position_jump`;
- the first 7 fail the 10-ambiguity floor (`low_ratio_weak_integer_system`).

M propagates the anchor with D's rule unchanged: the last FIX plus the
latest Doppler velocity × dt, with dt and the velocity age both at most 2 s.
The 5 m limit is unchanged.

`RTKFixHistoryJumpTest.MotionCompensationKeepsConsistentFixesUnderMaxPositionJump`
uses 13 m/s with a FIX 1.1 s old:

- the legacy check rejects the consistent 14.3 m candidate;
- M accepts it;
- M still rejects a candidate 6 m off the motion;
- beyond 2 s, M falls back to the stationary anchor.

## Frozen evaluation

The rule was frozen before the replay
([JSON](urbannav_max_position_jump_motion_evaluation.json) `frozen_rule_cdm`).
Without M, the build reproduces develop `f48dcd9`'s default and CD POS
byte-identically on all six runs.

| Rover | Drive | Recipe | FIX | wrong >0.5/2/3 m | default-correct lost | missing | H P95 m |
| --- | --- | --- | ---: | ---: | ---: | ---: | ---: |
| Trimble | Odaiba | default | 1,298 | 0/0/0 | — | 1,426 | 4.777 |
| Trimble | Odaiba | M | 1,298 | 0/0/0 | 0 | 1,426 | 4.777 |
| Trimble | Odaiba | CD | 4,338 | 6/0/0 | 37 (2.85%) | 1,419 | 4.854 |
| Trimble | Odaiba | **CDM** | 4,450 | 6/0/0 | 7 (0.54%) | 1,419 | 4.854 |
| Trimble | Shinjuku | default | 122 | 0/0/0 | — | 3,234 | 8.382 |
| Trimble | Shinjuku | M | 122 | 0/0/0 | 0 | 3,234 | 8.382 |
| Trimble | Shinjuku | CD | 2,845 | 4/0/0 | 0 | 3,273 | 8.387 |
| Trimble | Shinjuku | **CDM** | 2,877 | 4/0/0 | 0 | 3,272 | 8.387 |
| u-blox | Odaiba | default | 461 | 0/0/0 | — | 922 | 6.702 |
| u-blox | Odaiba | **M** | 617 | 0/0/0 | 0 | 922 | 6.702 |
| u-blox | Odaiba | CD | 854 | 0/0/0 | 4 | 921 | 6.700 |
| u-blox | Odaiba | **CDM** | 871 | 0/0/0 | 4 | 921 | 6.700 |
| u-blox | Shinjuku | default / M | 1 | 1/1/0 | — | 2,374 | 5.736 |
| u-blox | Shinjuku | CD / CDM | 16 | 0/0/0 | 0 | 2,345 | 5.844 |

Notes on the table:

- "M" outside u-blox Odaiba is byte-identical to the default.
- The recovery P95 and the right-censoring flags (0.5 m) are unchanged by M
  on every run.
- HK (descriptive: development data since online PVA holdout v3, with the
  truth biased up to about 1 m): M takes HK Deep from 6 to 17 FIX, none
  beyond 2 m, and leaves HK Harsh identical.

**Mechanism prediction (frozen): met.** Under CDM, 30 of the 37 CD losses are
correct fixes again. The remaining 7 are the floor's, and CDM loses no new
epoch.

**Gate:**

- **CDM** fails: it inherits CD's wrong fixes on Trimble.
- **M alone** passes every item listed in the frozen rule on all four Tokyo
  runs. It does not meet the clause of the
  [recovery guard gate](fix_recovery_guard.md) that at least one wrong-FIX
  population must *decrease*: none changes. It also changes only one of the
  four runs. M therefore stays opt-in, and switching it on by default is
  left to the maintainer.

## CD's remaining wrong fixes

The 10 wrong epochs (6 Odaiba, 4 Shinjuku) are not isolated jumps. Each sits
at the end of a FIX streak whose error grows by 0.07–0.15 m per epoch to
0.5–0.9 m. Seven of the ten are hold fixes, all directly after a FIX epoch.
The next FLOAT is back near the reference, so the drift is in the FIX
solution, not in the reference.

Every step stays under the motion-compensated 0.2 m history check. The
Doppler consensus re-anchors at every accepted FIX, so the accumulated drift
is never compared with anything.

A 1 s comparison with the integrated SPP Doppler velocity overlaps between
the two groups on Shinjuku:

| Drive | correct fixes p95 / p99 | wrong fixes |
| --- | --- | --- |
| Odaiba | 0.10 / 0.26 m | 0.30–0.74 m |
| Shinjuku | 0.28 / 0.56 m | 0.26–0.73 m |

No rule is proposed from it.

## Reproduce

Use the native argv of [the base interpolation record](urbannav_base_interpolation_evaluation.json)
with the recipe flags in the JSON. Score the Trimble runs with
`evaluate_urbannav_fix_continuity.score()` and the u-blox runs on the 0.2 s
grid with 5 ms matching. Run the test with
`run_tests --gtest_filter='RTKFixHistoryJumpTest.*'`.
