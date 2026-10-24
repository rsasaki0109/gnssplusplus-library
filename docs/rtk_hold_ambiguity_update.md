# Hold-ambiguity update: overwrite vs conditional update

Follow-up to the [iterated-update covariance](rtk_iterated_update_covariance.md)
investigation. With an honest filter covariance, about half of Shinjuku's wrong
FIX epochs were hold fixes whose float already sat in the wrong basin. The
handoff therefore listed `applyHoldAmbiguity()` as the next unverified suspect.
This note reproduces how it differs from a conditional update and evaluates a
conditional update. **The default is unchanged.**

## What the legacy hold does

After an accepted FIX (`consecutive_fix_count_ >= min_hold_count`), for every
selected DD pair the legacy code:

- writes `x_sat = x_ref - N_held` into the satellite's SD ambiguity;
- zeroes every covariance entry of that satellite, including its correlation
  with position;
- sets its variance to 0.001 cycle².

That is not a measurement update. The position does not move toward the
fixed solution, and the information that relates the ambiguity to position is
discarded, not conditioned. RTKLIB `holdamb` instead applies the held
integers as pseudo-observations, `z = N_held`, `h(x) = x_ref - x_sat`,
`R = 0.001 I`, in an ordinary Kalman update.

`RTKHoldAmbiguityUpdateTest.LegacyHoldOverwritesInsteadOfConditioningTheState`
(`tests/test_rtk_legacy.cpp`) builds a correlated position/ambiguity state.
It shows that the legacy hold leaves the position unchanged and zeroes the
ambiguity-position covariance, while the exact Kalman reference moves the
position and keeps it non-zero. The existing hold-fix tests still pin the
legacy overwrite.

## Opt-in conditional update

`--hold-kalman-update` (`RTKConfig::hold_ambiguity_kalman_update`) applies
the same pairs (same system, index and nonzero-state guards) as one
`kalmanFilter()` update with `R = 0.001 I`. The variance was not changed or
tuned. The test requires state and covariance to match the exact update to
1e-9. With the option off, all eight default/age5_doppler3 POS files are
byte-identical to develop `1fb1b6e`. With only `--kf-relinearize-from-prior`,
they are byte-identical to the previous round's experiment.

## Full-drive replay: NO_GO

The decision rule was frozen before replay (the same per-run adoption gate as
before). `H` = `--hold-kalman-update`; `HR` = H plus
`--kf-relinearize-from-prior`; `R` = relinearize only. All runs use the same
native argv as `scripts/analysis/evaluate_urbannav_fix_continuity.py` plus
the flags. The [audit](urbannav_hold_update_evaluation.json) records
hashes and argv.

| Drive | Recipe | Version | FIX | FIX >0.5 / >2 / >3 m | of which hold | Missing | H P95 m | Correct (≤0.5 m) lost vs same-recipe baseline |
| --- | --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Odaiba | default | baseline / H / R / HR | 1,298 | 0 / 0 / 0 | 0 | 1,426 / 1,426 / 1,515 / 1,515 | 4.7772 / 4.7772 / 5.1399 / 5.1399 | 0 |
| Shinjuku | default | baseline / H / R / HR | 122 | 0 / 0 / 0 | 0 | 3,234 / 3,234 / 3,451 / 3,451 | 8.3821 / 8.3821 / 1.9650 / 1.9650 | 0 |
| Odaiba | age5_doppler3 | baseline | 4,256 | 157 / 3 / 0 | 74 | 1,480 | 4.8963 | — |
| Odaiba | age5_doppler3 | H | 4,373 | 110 / 3 / 0 | 7 | 1,578 | 6.2332 | 11 |
| Odaiba | age5_doppler3 | R | 4,297 | 103 / 3 / 0 | 28 | 1,562 | 5.2171 | 15 |
| Odaiba | age5_doppler3 | HR | 4,317 | 73 / 5 / 2 | 7 | 1,603 | 6.2321 | 39 |
| Shinjuku | age5_doppler3 | baseline | 1,949 | 13 / 1 / 1 | 4 | 3,332 | 8.3432 | — |
| Shinjuku | age5_doppler3 | H | 1,912 | 19 / 1 / 1 | 1 | 3,331 | 8.3432 | 112 |
| Shinjuku | age5_doppler3 | R | 3,972 | 516 / 125 / 15 | 234 | 3,581 | 1.8668 | 309 |
| Shinjuku | age5_doppler3 | HR | 4,227 | 383 / 188 / 24 | 98 | 3,569 | 1.8587 | 87 |

With the default recipe, H changes some POS bytes but none of the FIX, missing,
P95 or correct-FIX statistics. H fails the gate on
Odaiba age5_doppler3 (missing +98, P95 +1.34 m) and Shinjuku age5_doppler3
(wrong >0.5 m 13→19, correct loss 112/1,936 = 5.8%). HR fails everywhere
R failed, and adds >2/>3 m errors on both drives. **Decision: keep the option
off.** Nothing was retuned.

## What this establishes

- The legacy overwrite does propagate wrong basins through the hold path. The
  conditional update cuts wrong hold FIX epochs from 74 to 7 (Odaiba, legacy
  covariance) and from 234 to 98 (Shinjuku, honest covariance).
- It is not the main source of wrong FIXes. Wrong **AR** candidates remain or
  grow (Odaiba 83→103 under H; Shinjuku 282→285 under HR), and gross errors
  increase in HR. Moving the position on hold also changes later acceptance
  decisions, and the P95/missing changes follow from that.
- Together with the previous round: correcting the covariance (R) or the hold
  (H) alone, or both (HR), does not stop the existing ratio/jump/Doppler gates
  from accepting wrong low-redundancy AR candidates. The next question is
  candidate acceptance itself, studied against the honest covariance of R/HR.
  Any rule must be frozen before replay.
