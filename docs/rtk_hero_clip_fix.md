# Why the hero clip never shows FIX, and what changes it

PR #639 adds a Ground Truth zenith inset to the Odaiba README GIF (GPS week
2032, TOW 273700–273720). The inset shows about 29 model-LOS satellites and
20 received, yet libgnss++ stays FLOAT for the whole clip. This note explains
that mismatch with solver telemetry. It replays opt-in recipes on both
UrbanNav Tokyo drives, using the Trimble rover as development data and the
previously unused **u-blox rover as holdout**. **Defaults are unchanged.**
Record: [JSON](urbannav_hero_clip_fix_evaluation.json).

## Two rejections, both motion-blind

1. **Stale FIX anchor.** In 173 of the clip's 201 epochs an AR candidate
   exists (median 17 DD pairs, 0.28 m from the float). `max_position_jump`
   (5 m) rejects every one of them. It compares the candidate with the
   last FIX, 179–199 s and 338–415 m away, because
   `max_fixed_anchor_age_s = 0` never expires the anchor.
2. **Stationary 0.2 m history check.** Once the anchor is allowed to
   expire, `exceedsFixHistoryJump()` takes over. After three consecutive
   FIXes it rejects a candidate more than 0.2 m from the previous FIX,
   ignoring elapsed time. At 10 Hz this rejects anything moving faster than
   2 m/s, so the clip alternates `.FFF.FFF.FFF…`. Labelled offline against the
   reference, the candidates it rejects are 96–100% correct (median
   4–8 cm) on both drives.

## Opt-in pieces

| Flag | Effect |
| --- | --- |
| `--max-fixed-anchor-age 5 --max-fixed-doppler-consensus 3` | existing; expire the anchor, check the Doppler track |
| `--low-ratio-guard-threshold 1e9 --low-ratio-min-fixed-ambiguities 10` | existing guard used as "reject AR candidates fixing < 10 DD ambiguities" |
| `--hold-fix-unchanged-set` | existing; no hold with unvalidated rounded pairs |
| `--fix-history-motion-compensation` | **new**: history reference = last FIX + latest Doppler velocity × dt (both ≤ 2 s old); the 0.2 m tolerance is unchanged |

`RTKFixHistoryJumpTest.MotionCompensationKeepsConsistentMovingFixes` covers
three cases at 6 m/s and 10 Hz: the legacy check rejects the 0.6 m step, the
compensated check accepts it, and a candidate 0.5 m off the motion is still
rejected. Without the new flag, the build reproduces the develop default POS
byte-identically on both receivers and drives.

The 10-ambiguity floor was chosen **after** seeing the Trimble diagnostics
(wrong AR median 6 pairs, correct 14–19). Each recipe was written down before
its replay, and nothing was retuned afterwards. The u-blox drives were not
used for any choice.

## Results (vs the default recipe)

Recipes: C = anchor 5 s + Doppler 3 m + 10-ambiguity floor; C2 = C + hold
unchanged-set; D = motion compensation only; CD = C + motion compensation.

| Rover | Drive | Recipe | FIX | wrong >0.5/2/3 m | default-correct lost | missing | H P95 m |
| --- | --- | --- | ---: | ---: | ---: | ---: | ---: |
| Trimble | Odaiba | default | 1,298 | 0/0/0 | — | 1,426 | 4.777 |
| Trimble | Odaiba | age5_doppler3 | 4,256 | 157/3/0 | 0 | 1,480 | 4.896 |
| Trimble | Odaiba | C | 3,884 | 52/0/0 | 63 | 1,421 | 4.855 |
| Trimble | Odaiba | C2 | 3,641 | 1/0/0 | 297 | 1,425 | 4.904 |
| Trimble | Odaiba | D | 1,454 | 0/0/0 | 0 | 1,425 | 4.777 |
| Trimble | Odaiba | **CD** | 4,338 | 6/0/0 | 37 | 1,419 | 4.854 |
| Trimble | Shinjuku | default | 122 | 0/0/0 | — | 3,234 | 8.382 |
| Trimble | Shinjuku | age5_doppler3 | 1,949 | 13/1/1 | 0 | 3,332 | 8.343 |
| Trimble | Shinjuku | C | 2,328 | 0/0/0 | 0 | 3,294 | 8.388 |
| Trimble | Shinjuku | D | 122 | 0/0/0 | 0 | 3,233 | 8.382 |
| Trimble | Shinjuku | **CD** | 2,845 | 4/0/0 | 0 | 3,273 | 8.387 |
| u-blox | Odaiba | default | 461 | 0/0/0 | — | 922 | 6.702 |
| u-blox | Odaiba | C | 648 | 0/0/0 | 0 | 921 | 6.700 |
| u-blox | Odaiba | D | 645 | 0/0/0 | 5 | 923 | 6.704 |
| u-blox | Odaiba | **CD** | 854 | 0/0/0 | 4 | 921 | 6.700 |
| u-blox | Shinjuku | default | 1 | 1/1/0 | — | 2,374 | 5.736 |
| u-blox | Shinjuku | C | 15 | 0/0/0 | 0 | 2,347 | 5.846 |
| u-blox | Shinjuku | **CD** | 16 | 0/0/0 | 0 | 2,345 | 5.844 |

The u-blox epoch tags carry a constant +1 ms (Odaiba) / +2–3 ms (Shinjuku)
offset. Those runs are therefore scored on the 0.2 s reference grid: a
solution is matched within 5 ms and compared with the reference interpolated
to its time. Missing solutions stay missing. The Trimble runs use the
repository evaluator's exact 1 ms matching.

**Hero clip (Odaiba TOW 273700–273720):** default 0 FIX. C2: 112/191, still
`.FFF.`. **CD: 151/191 FIX, continuous**, 3D error median 2.1 cm, p95 5.1 cm,
max 0.49 m. The remaining non-FIX span is a genuine short outage (SPP/FLOAT).

## Decision

No recipe passes the strict default gate:

- **D** is neutral on Trimble, but fails on the holdout by 5/461 (1.08%)
  correct-FIX loss, +1 missing and +2 mm P95.
- **CD** is clean on the holdout and removes the default's one gross u-blox
  FIX. On Trimble, however, it adds 6 (Odaiba) and 4 (Shinjuku) wrong FIX
  epochs at 0.5–2 m and loses 2.9% of Odaiba's default-correct FIX epochs.

All pieces stay opt-in. Rendering the hero with CD would be an explicitly
labelled opt-in recipe, not the default solver.
