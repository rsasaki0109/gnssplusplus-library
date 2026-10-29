# Phase-fit condition for the hold-fix NIS limit on `--preset urban`

This note follows the [hold-fix NIS note](rtk_hold_fix_nis.md). It adds one
opt-in option that applies the hold-fix NIS limit only to a hold whose position
also misfits the carrier phases. **No default and no preset change.** Record:
[JSON](hold_fix_phase_fit_evaluation.json).

The reference trajectory and the truth DD export are used only for scoring, for
the analysis-only [phase audit](rtk_phase_audit.md) and for the motivation
below. Nothing here is a solver input.

## The NIS limit removes wrong holds, and correct ones with them

Under U (`--preset urban`) the ten runs have 2849 hold fixes: 2651 correct and
183 genuine wrong FIX (>0.5 m). The NIS limit of 10 (UH) removes 107 of the 183,
but also 238 correct holds, 12% of the correct FIX of nagoya/run3.

The phase audit's ambiguity function separates them better. AFn is computed at
the solver position from the truth DD export (elevation ≥ 15°, no LLI) and
maximised within ±0.10 m:

- **Wrong holds misfit.** 129 of the 156 wrong holds with at least 10 DD have
  AFn < 0.5.
- **Correct holds that misfit are already off.** 213 correct holds have
  AFn < 0.5. Their error is 0.20–0.47 m (p5–p95; median 0.31 m), so they are
  in a wrong-integer basin but below the 0.5 m label.

Direct effect on the U holds, before any knock-on effect:

| Rule | Wrong holds removed (of 183) | Correct removed <0.1 m / 0.1–0.5 m | Worst run |
| --- | ---: | --- | --- |
| NIS > 10 (UH) | 107 | 8 / 230 | nagoya/run3 11.99% |
| AFn < 0.5 | 129 | 4 / 209 | tokyo/run2 1.86% |
| NIS > 10 and (AFn < 0.5 or < 10 DD) | 103 | 0 / 47 | tokyo/run2 0.39% |

## Change

The new option is `hold_fix_update_nis_max_phase_fit`
(`--hold-fix-update-nis-max-phase-fit <af>`, default 0 = off). It works only
together with `--hold-fix-max-update-nis-per-obs`:

- When the update NIS exceeds that limit, `tryHoldFix()` computes the phase fit
  of the hold position over the hold's DD pairs.
- The fit is the mean of cos(2π r/λ) of the integer-free DD phase residual r.
  It takes its maximum on a ±0.10 m grid with 0.02 m steps.
- The NIS rejection then applies only when the fit is below `af`, or unknown
  because fewer than 10 pairs are usable.
- The fit is written to the debug epoch log as `hold_fix_phase_fit`.

The window, the 10 pairs and the value 0.5 are `NEIGHBOURHOOD`, `MIN_DD` and
`AF_INCONSISTENT` of `scripts/analysis/truth_dd/phase_audit.py`, and 10 is the
NIS value of UH. None was tuned here. The rule discloses that the AND of NIS
and phase misfit was chosen after seeing the distributions above on the same
ten runs.

Two tests cover the option:

- `HoldPhaseFitIsHighOnlyNearAPositionThatFitsThePhases`
- `NisLimitSparesOnlyAHoldWhosePositionFitsThePhases`

## Result: UHP = UH + `--hold-fix-update-nis-max-phase-fit 0.5`

Without the flag, `--preset urban` POS is byte-identical to develop on every run.
UH reproduces the NIS note exactly.

| Run | U: FIX / audited wrong | UH: FIX / audited wrong / U-correct lost | UHP: FIX / audited wrong / U-correct lost |
| --- | --- | --- | --- |
| Trimble Odaiba | 7467 / 9 | 7440 / 7 / 0.34% | 7440 / 7 / 0.34% |
| Trimble Shinjuku | 11491 / 119 | 11440 / 50 / 0.91% | 11520 / 53 / **0.18%** |
| u-blox Odaiba | 3130 / 0 | 3130 / 0 / 0 | 3130 / 0 / 0 |
| u-blox Shinjuku | 3582 / 3 | 3621 / 2 / 0.03% | 3582 / 3 / 0 |
| PPC tokyo/run1 | 4328 / 24 | 4329 / 20 / 3.44% | 4319 / 22 / 2.09% |
| PPC tokyo/run2 | 6669 / 5 | 6517 / 5 / 3.45% | 6550 / 5 / 2.21% |
| PPC tokyo/run3 | 10910 / 35 | 10770 / 18 / 1.60% | 10811 / 18 / 1.22% |
| PPC nagoya/run1 | 5280 / 19 | 5145 / 15 / 2.53% | 5253 / 19 / **0.55%** |
| PPC nagoya/run2 | 4404 / 17 | 4395 / 14 / 0.14% | 4395 / 14 / 0.14% |
| PPC nagoya/run3 | 1069 / 18 | 951 / 4 / 9.90% | 957 / 4 / 9.32% |

The audited counts here also audit u-blox Shinjuku (10 → 3 under U), so the U and
UH totals are 7 below those of the NIS note.

| Ten runs | U | UH | UHP |
| --- | ---: | ---: | ---: |
| FIX | 58330 | 57738 | 57957 |
| Audited wrong >0.5 m | 249 | 135 | 145 |
| Phase-confirmed wrong | 133 | 36 | 36 |
| U-correct FIX lost | – | 923 | 548 |

- **The wrong-FIX reduction of UH is kept.** Phase-confirmed wrong FIX fall from
  133 to 36, as under UH. Audited wrong FIX are 145 against UH's 135.
- **Lost correct FIX fall by 41%** (923 → 548).
- **Trimble Shinjuku no longer pays for it.** Missing epochs are 3332 (UH 3363)
  and Hp95 is 8.338 m (UH 8.670).

The frozen gates came out as follows:

- **A**: passes on all ten runs with audited labels. Raw labels fail on Trimble
  Shinjuku (176 against RTKLIB's 128), as under UH.
- **W**: passes on every run.
- **L** (U-correct loss ≤ 1%): fails on tokyo/run1 2.09%, tokyo/run2 2.21%,
  tokyo/run3 1.22% and nagoya/run3 9.32%.
- **M**: fails only on tokyo/run3. Hp95 goes 3.10 → 4.12 m, as under UH. UHP
  publishes 228 more epochs there: FLOAT and SPP after a FIX streak, which U's
  kinematic post-filter dropped.

The predictions missed. The direct effect suggested at most 0.4% loss per run and
less than 1% on nagoya/run3.

**Decision under the frozen rule.** A and W pass, but L and M do not. The option
stays opt-in, and `--preset urban` is unchanged.

## Where the remaining loss comes from

- **Tokyo PPC: later AR fixes.** Most lost epochs are U AR fixes within 0.1 m:
  72 of 90 on run1, 133 of 147 on run2 and 87 of 133 on run3. They come up to
  hundreds of epochs after a hold rejection. The float that was not held fails
  `lambda_not_fixed` or the position jump check.
- **nagoya/run3: a correct rejection closes the hold path.**
  1. Under U, the hold at TOW 553961.2 (float update NIS per observation 1651)
     is 0.87 m off. AFn is 0.35 at the hold and 0.91 at the reference.
  2. U's holds stay 0.9–1.0 m off until 553963.8.
  3. They come back to about 0.2 m at 553964.0.
  4. UHP rejects the 553961.2 hold (in-solver phase fit 0.37). The FLOAT epoch
     closes the hold path, and the 98 later U holds are lost: 6 within 0.1 m,
     65 within 0.1–0.2 m and 27 within 0.2–0.5 m.

**Correction to the NIS note.** That note says the legacy hold at nagoya/run3
TOW 553961.2 "keeps fixing correctly". Under U it is 0.87 m off, and the holds
stay wrong for 2.8 s. The NIS spike there marks a wrong hold, not a correct hold
riding through bad measurements. Correct holds after a NIS above 10 do exist:
for example 553958.2–553961.0 on the same run, with NIS 10.7–11.6, phase fit
0.89–0.92 and errors of 0.17–0.18 m. UHP keeps those.

## Variant UHP2: keep the hold path open as well

UHP2 = UHP + `--hold-fix-update-nis-keeps-hold-run`, under a new frozen rule
written after the UHP replay.

| | UHP | UHP2 |
| --- | ---: | ---: |
| Audited wrong >0.5 m | 145 | 179 |
| Phase-confirmed wrong | 36 | 58 |
| U-correct lost | 548 | 508 |
| nagoya/run3 loss | 9.32% | 9.32% |

- **nagoya/run3 does not move.** The next hold candidates are about 1 m off and
  fail the hold jump check, which ends the kept run.
- **Wrong FIX return on Shinjuku.** Audited wrong FIX rise from 53 to 87, and
  phase-confirmed from 3 to 25.

UHP2 is not recommended.

## Reproduce

1. Build this branch. Export the truth DD of each run as in the
   [phase audit note](rtk_phase_audit.md).
2. Run U, UH, UHP and UHP2 with the argv of the
   [CDMH note](rtk_cdmh_ppc_evaluation.md) and `--debug-epoch-log <csv>`.
3. Score Trimble and PPC with
   `scripts/analysis/evaluate_urbannav_fix_continuity.py`'s `score()`. u-blox
   uses the reference interpolated to each solution epoch.
4. Audit each POS with `scripts/analysis/truth_dd/phase_audit.py`.
