# Hold-fix NIS limit on `--preset urban`; why urban publishes fewer epochs

This note follows the [phase audit note](rtk_phase_audit.md). It adds one opt-in
option and evaluates it on `--preset urban` (the CDME recipe). It also explains
why the preset publishes fewer epochs than the default. **No default and no
preset change.** Record: [JSON](hold_fix_nis_evaluation.json).

The reference trajectory is used only for scoring and for the analysis-only
phase audit.

## Most of the preset's genuine wrong FIX are hold fixes

The urban debug logs cover the ten Tokyo and PPC runs. Fixes are labelled by
the reference and by the phase audit; reference-inconsistent epochs are left
out.

- **Hold fixes are the main source.** They account for 183 of the 256 genuine
  wrong FIX (>0.5 m), although they are only about 5% of all fixes.
- **The float update is already inconsistent.**
  - 107 of those 183 follow a float update whose NIS per observation exceeds 10.
  - Correct fixes have a NIS p50 of 0.54 and a p99 of 10.2.
  - The epochs the audit marks `reference_inconsistent` have a NIS p99 of 2.4.
    That is independent support that those fixes were right.
- **The hold path skips the existing gate.** `max_fixed_update_nis_per_observation`
  already rejects AR candidates after such an update. `tryHoldFix()` never
  checks it.
- **Example: Shinjuku TOW 281650.1.**
  1. The NIS per observation jumps from 1 to 52.5.
  2. LAMBDA fails.
  3. 64 hold fixes, 0.63–0.75 m off, follow.

## Change

The new option is `hold_fix_max_update_nis_per_observation`
(`--hold-fix-max-update-nis-per-obs <v>`, default 0 = off). It rejects a hold
fix with reason `hold_fix_update_nis` when the epoch's float update NIS per
observation exceeds `v`. The held integers are left as they were.

`RTKHoldFixDefaultTest.HoldFixIgnoresAnInconsistentFloatUpdateUnlessItsNisLimitIsSet`
reproduces the problem and the fix:

- **Legacy:** the hold fix is accepted after an update with NIS 52.5.
- **Limit 10:** the same hold fix is rejected.
- **NIS 2:** the hold fix is accepted, at the same position as without the
  limit.

The evaluated value is 10. It is the value of the earlier AR-only gate
experiments ([PPC realtime gates](ppc_realtime_gate_existing_outputs.md),
`fixed_nis10`); it was not tuned here. The rule discloses that the distributions
above were seen before the rule was frozen.

## Result: UH = `--preset urban` + `--hold-fix-max-update-nis-per-obs 10`

Without the flag, default and urban POS are byte-identical to develop on every
run.

| Run | RTKLIB: FIX / wrong >0.5 m | U: FIX / wrong >0.5 m (audited) | UH: FIX / wrong >0.5 m (audited) | U-correct lost |
| --- | --- | --- | --- | ---: |
| Trimble Odaiba | 5839 / 19 | 7467 / 9 | 7440 / 7 | 0.34% |
| Trimble Shinjuku | 6313 / 128 | 11491 / 242 (119) | 11440 / 173 (50) | 0.91% |
| u-blox Odaiba | 1857 / 92 | 3130 / 0 | 3130 / 0 | 0 |
| u-blox Shinjuku | 792 / 93 | 3582 / 10 | 3621 / 9 | 0.03% |
| PPC tokyo/run1 | 995 / 123 | 4328 / 24 | 4329 / 20 | 3.44% |
| PPC tokyo/run2 | 1151 / 15 | 6669 / 5 | 6517 / 5 | 3.45% |
| PPC tokyo/run3 | 6675 / 85 | 10910 / 35 | 10770 / 18 | 1.60% |
| PPC nagoya/run1 | 1577 / 81 | 5280 / 19 | 5145 / 15 | 2.53% |
| PPC nagoya/run2 | 2133 / 239 | 4404 / 26 (17) | 4395 / 23 (14) | 0.14% |
| PPC nagoya/run3 | 291 / 92 | 1069 / 18 | 951 / 4 | 9.90% |

- **Wrong FIX fall.** Over the ten runs, audited wrong FIX >0.5 m fall from 256
  to 142 (−44.5%). Wrong FIX beyond 2 m and 3 m are unchanged, and no run gains
  a wrong FIX.
- **Phase-confirmed wrong FIX fall from 133 to 36.** On Shinjuku they go from
  69 to 3.
- **Fix count falls by 1.0%** (58330 → 57738). RTKLIB has 27623.

The frozen gates came out as follows:

- **A** (more FIX and no more wrong FIX than RTKLIB): passes on all ten runs
  with audited labels. Raw labels pass on 9; Trimble Shinjuku has 173 against
  128.
- **W** (wrong FIX not increased): passes on every run.
- **L** (U-correct loss ≤ 1%): fails on five PPC runs, with losses of 1.6–9.9%.
  The direct estimate predicted only nagoya/run3. The likely reason, not yet
  verified, is that a rejected hold fix resets the run of consecutive fixes, so
  the hold path stays closed for a while afterwards.
- **M** (missing epochs and Hp95 not increased): fails on four runs:
  - Shinjuku: 27 more missing epochs, Hp95 +0.33 m;
  - u-blox Shinjuku: one more missing epoch;
  - PPC tokyo/run3: Hp95 3.10 → 4.12 m;
  - PPC nagoya/run1: Hp95 +0.04 m.

**Decision under the frozen rule.** A and W pass, but L and M do not, so the
trade goes to the maintainer. The option stays opt-in, and `--preset urban` is
unchanged.

## Variant: keep the hold path open after a NIS-only rejection

In the UH debug logs, few of the lost correct fixes are direct rejections:
3 of 104 on nagoya/run3 and 8 of 230 on tokyo/run2.

- **nagoya/run3:** 97 of the 104 are U hold fixes that UH never attempted. The
  rejected epoch was FLOAT, which restarted the fix run and closed the hold
  path.
- **tokyo/run2:** most are AR fixes that stopped resolving.

The opt-in `hold_fix_update_nis_keeps_hold_run`
(`--hold-fix-update-nis-keeps-hold-run`) keeps that run for the hold-fix
fallback and the hold. The run ends when any of these happens:

- a fix extends the run past it;
- the hold is rejected for any other reason;
- a reacquisition reset;
- an SPP fallback;
- a FLOAT epoch without that rejection.

`consecutive_fix_count_` and its other uses are unchanged. Two tests cover it:
`NisRejectedHoldKeepsTheHoldPathOpenOnlyWhenAsked` and
`KeptHoldRunEndsAtAnyOtherHoldRejection`.

UH2 = UH + the flag, under a new frozen rule:

| | U | UH | UH2 |
| --- | ---: | ---: | ---: |
| FIX (10 runs) | 58330 | 57738 | 57771 |
| Audited wrong >0.5 m | 256 | 142 | 181 |
| Phase-confirmed wrong | 133 | 36 | 58 |
| U-correct loss on nagoya/run1 / run3 | – | 2.53% / 9.90% | 0.46% / 9.90% |
| U-correct loss on tokyo/run1 / run2 / run3 | – | 3.44 / 3.45 / 1.60% | 2.81 / 2.45 / 1.60% |

- **Where it helps:** fewer lost fixes on nagoya/run1 and tokyo/run1–2.
- **What it costs:** it gives back part of the wrong-FIX reduction. On
  Shinjuku, audited wrong FIX rise from 50 under UH to 84 under UH2.
- **nagoya/run3 does not move, for a different reason:**
  1. At TOW 553961.2 the float update NIS per observation is 1559, yet the
     legacy hold keeps fixing correctly.
  2. UH2 rejects that hold for its NIS, so the inconsistent update reaches the
     float.
  3. The next hold candidate is more than 1 m off and fails the hold jump check,
     which ends the kept run.
  4. 108 correct hold epochs are lost.
- **What the NIS cannot tell apart:** a NIS spike can come from the
  measurements while the held integers are right, so it does not separate:
  - a hold that is wrong because it overwrote a stale float (Shinjuku 281650);
  - a correct hold that is riding through bad measurements (nagoya/run3).

Gates A and W pass. L fails on PPC tokyo/run1–3 and nagoya/run3, and M on
Shinjuku, u-blox Shinjuku and tokyo/run3. P1 (nagoya/run3 loss below 5%) and P2
(audited total at most 160) are not met. **Neither variant is added to
`--preset urban`; both stay opt-in.**

## Why `--preset urban` publishes fewer epochs than the default

`gnss solve` runs a kinematic post-filter by default. Once a FIX has been
written, it drops any epoch whose height differs from the last written epoch by
more than max(1 m, 4 m/s × dt). Under the preset the last written epoch is
mostly an accurate FIX, so the code-biased FLOAT and SPP epochs after a FIX
streak are dropped. The solver did produce them: the urban debug logs show
FLOAT, and sometimes FIX, at the epochs that only the default publishes.

Rerunning urban without the post-filter restores these epochs:

| Run | Restored | FLOAT / SPP / FIX | Restored H p50 | Restored \|U\| p50 / p95 |
| --- | ---: | --- | ---: | --- |
| Trimble Odaiba | 636 | 412 / 218 / 6 | 2.93 m | 4.73 / 33.6 m |
| Trimble Shinjuku | 1252 | 790 / 380 / 82 | 2.46 m | 5.67 / 51.1 m |
| PPC tokyo/run3 | 647 | 378 / 213 / 56 | 3.20 m | 5.24 / 36.8 m |

- **The extra missing epochs are mostly what the filter is meant to remove.**
  They are FLOAT and SPP epochs with metre-level height errors.
- **It also removes correct fixes.** It drops correct FIX epochs whose height
  is more than 1 m from a code-biased FLOAT anchor: 71 on Shinjuku, 54 on PPC
  tokyo/run3 and 6 on Odaiba. It drops 13 wrong FIX epochs as well.
- **The missing-epoch gate is misleading here.** Counting missing epochs
  penalises the preset for dropping outliers.
- **No change is proposed here.** Letting a FIX epoch through against a FLOAT
  anchor needs its own evaluation.

## Reproduce

1. Build this branch.
2. Run `--preset urban` with and without `--hold-fix-max-update-nis-per-obs 10`,
   using the argv of the [CDMH note](rtk_cdmh_ppc_evaluation.md).
3. Score with the same matching, and audit with
   `scripts/analysis/truth_dd/phase_audit.py`.
4. For the missing-epoch analysis, add `--no-kinematic-post-filter` to the urban
   run.
