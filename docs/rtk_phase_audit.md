# Carrier-phase audit of wrong-FIX labels; CDMEH is NO_GO

This note follows the [Doppler consensus age note](rtk_doppler_consensus_age.md).
There, CDME (CDM + `--max-fixed-doppler-consensus-age 5`) beat RTKLIB demo5 on 9
of 10 runs. The exception was Trimble Shinjuku: 242 wrong FIX at 0.5–2 m
against RTKLIB's 128. This round asks whether those labels are right, then
replays one more recipe. **No solver code and no default change.** Record:
[JSON](phase_audit_cdmeh_evaluation.json).

The reference trajectory is used only for scoring and for the analysis-only
audit; it is never a solver input.

## The largest Shinjuku episode is a reference offset

From TOW 283198 to 283247 the vehicle is stopped (reference speed 0). CDME
fixes with a steady +0.41 m north offset from the reference during the whole
stop. After it moves off, the offset grows to 0.5–0.7 m (TOW 283255–283265),
and later to 1.3 m (283284–283289).

The DD carrier phases decide which position is wrong. Their ambiguity
function

    AF(x) = mean over DD pairs of cos(2π · (DD phase − DD range(x)) / λ)

is 1 when every DD is an integer number of cycles from x, and about 0 for a
position that does not fit the phases. Sampled every 6 s during the stop, with
19–30 DD pairs:

- AF at the reference is −0.01 to 0.14;
- AF at the FIX position is 0.84–0.96;
- a ±1 m horizontal grid search peaks at the FIX position, horizontally within
  2 cm.

As a control, at TOW 283168–283174 the FIX is within 0.08 m of the reference,
and AF at the reference is 0.92 and 0.76.

## The audit

`scripts/analysis/truth_dd/phase_audit.py` applies this test to every FIX epoch
more than 0.5 m from the reference. It reads the output of
`scripts/analysis/truth_dd/export_truth_dd.cpp`. AFn is the best AF within
±0.10 m of a point, evaluated at both the reference and the solver position.
Each epoch gets one label:

| Label | Condition |
| --- | --- |
| `undetermined_few_dd` | fewer than 10 DD pairs (rover LLI clear, elevation ≥ 15°) |
| `reference_inconsistent` | AFn(solver) ≥ 0.8 and AFn(reference) < 0.5 |
| `wrong_fix_confirmed` | AFn(reference) ≥ 0.8 and AFn(solver) < 0.5 |
| `undetermined` | anything else |

The rule and three predictions were frozen before any full run.
`tests/test_phase_audit.py` checks the labels on synthetic scenes.

Controls are correct FIX epochs within 0.10 m of the reference. Both AFn are at
least 0.8 for 85–100% of controls on the Trimble drives. On PPC it is 67–100%;
the lowest rates are nagoya/run1 and a 6-epoch sample on nagoya/run3. No
control epoch is labelled `reference_inconsistent` on any run.

Labels for the CDME round's runs (u-blox and HK are not audited):

| Run | Solver | Wrong >0.5 m | Reference inconsistent | Confirmed | Undetermined (few DD) |
| --- | --- | ---: | ---: | ---: | ---: |
| Trimble Shinjuku | RTKLIB | 128 | 0 | 21 | 30 (77) |
| Trimble Shinjuku | CDME | 242 | **123** | 69 | 34 (16) |
| Trimble Odaiba | RTKLIB | 19 | 0 | 2 | 3 (14) |
| Trimble Odaiba | CDME | 9 | 0 | 1 | 6 (2) |
| PPC, 6 runs | RTKLIB | 635 | 1 | 129 | 417 (88) |
| PPC, 6 runs | CDME | 127 | 9 | 63 | 49 (6) |

- **Only Shinjuku has a reference problem of this size.** On PPC only
  nagoya/run2 has any `reference_inconsistent` epochs: 9 for CDME and 1 for
  RTKLIB.
- **None of RTKLIB's 112 Shinjuku wrong FIX beyond 3 m is
  `reference_inconsistent`**: 21 are confirmed, the rest undetermined.
- **Among CDME's Shinjuku epochs, 123 of 242 (50.8%) are
  `reference_inconsistent`.** They lie in TOW 283255–283265 and 283284–283289.

With audited wrong counts (raw wrong minus `reference_inconsistent`, for both
solvers), **CDME beats RTKLIB on all 10 runs**. On Trimble Shinjuku it has
11491 FIX and 119 / 0 / 0 wrong FIX (>0.5 / 2 / 3 m), against RTKLIB's 6313 and
128 / 112 / 112. The raw labels still give 9 of 10.

## The genuine CDME episode: the legacy hold overwrite

On Shinjuku, 64 of CDME's 69 confirmed wrong epochs are one episode, TOW
281650.1–281656.4 (0.63–0.75 m):

1. **281649.6–281650.0: correct fixes from a stale float.** These are correct
   subset AR fixes (19 of 25 pairs, ratio 3.0). The float is 0.99 m off, yet its
   position covariance trace is 1.8e-4 m².
2. **The legacy hold leaves the state inconsistent.** At the 5th consecutive
   fix (`min_hold_count`), it overwrites the held SD ambiguities and zeroes
   their cross-covariances. The float position stays 1 m off.
3. **281650.1: the solver falls back to hold fixes.**
   - The float NIS per observation jumps from 1 to 52.5, and LAMBDA fails.
   - The hold fix lands 0.65 m from the last FIX, inside the 1 m hold jump
     limit.
   - It is 0.667 m from a 0.1 s old Doppler track, inside the 3 m limit.
   - Hold fixes continue for 6.6 s.

With `--hold-kalman-update` the state stays consistent. These epochs then
become correct AR fixes (ratio 34–40, 26 pairs).

## CDMEH = CDME + `--hold-kalman-update`: NO_GO

Every flag already exists. The rule was frozen before the full replay. It was
written after one exploratory Shinjuku Trimble run, which the rule discloses.
The binary is develop `2219d1d`. Default, CDM and CDME reproduce the CDME
round byte for byte.

FIX and wrong FIX (>0.5 / 2 / 3 m), 0.11 s matching:

| Run | RTKLIB | CDME | CDMEH |
| --- | --- | --- | --- |
| Trimble Odaiba | 5839 / 19, 4, 4 | 7467 / 9, 0, 0 | 7510 / 8, 0, 0 |
| Trimble Shinjuku | 6313 / 128, 112, 112 | 11491 / 242, 0, 0 | 12361 / 190, 0, 0 |
| u-blox Odaiba | 1857 / 92, 69, 44 | 3130 / 0 | 3148 / 3, 0, 0 |
| u-blox Shinjuku | 792 / 93, 81, 81 | 3582 / 10, 0, 0 | 4048 / 57, 0, 0 |
| PPC tokyo/run1 | 995 / 123, 105, 104 | 4328 / 24, 1, 1 | 6306 / **129, 7, 5** |
| PPC tokyo/run2 | 1151 / 15, 13, 13 | 6669 / 5, 0, 0 | 6845 / 9, 0, 0 |
| PPC tokyo/run3 | 6675 / 85, 48, 44 | 10910 / 35, 0, 0 | 11350 / 8, 0, 0 |
| PPC nagoya/run1 | 1577 / 81, 77, 76 | 5280 / 19, 0, 0 | 4820 / 14, 0, 0 |
| PPC nagoya/run2 | 2133 / 239, 229, 223 | 4404 / 26, 0, 0 | 3909 / 33, 2, 0 |
| PPC nagoya/run3 | 291 / 92, 82, 75 | 1069 / 18, 2, 0 | 1206 / 13, 2, 0 |

- **Where the Kalman hold helps:**
  - Shinjuku Trimble confirmed wrong FIX fall from 69 to 3.
  - PPC tokyo/run3 confirmed wrong FIX fall from 28 to 3.
  - PPC nagoya/run3 raw wrong FIX fall from 18 to 13.
- **Where it hurts:**
  - PPC tokyo/run1 rises to 129 / 7 / 5. That is worse than RTKLIB at 0.5 m.
  - u-blox Shinjuku rises from 10 to 57.
- **Gate A** (more FIX and no more wrong FIX than RTKLIB) holds on 8 of 10 runs
  raw and 9 of 10 audited.
- **Gate C′** (against CDME) fails on all 10 Tokyo and PPC runs; only HK
  passes. CDME-correct loss is 0.67–14.52%, and missing epochs rise on 6 of
  10 runs.
- **Gate B** (against default) fails, as for CDM and CDME.

Predictions: Odaiba FIX ±5% and nagoya/run3 met; gate A ≥ 9 of 10 raw and
loss ≤ 1% not met. **Decision: NO_GO.** CDMEH is not proposed, and
`--hold-kalman-update` stays opt-in.

## Where this leaves the goal

- **The goal is met with audited labels.** Under the frozen audit, the existing
  opt-in CDME has more FIX and no more wrong FIX than RTKLIB on all 10 runs.
  The Shinjuku shortfall was mostly reference error.
- **CDME is still not a default candidate.** Against CDM and default it adds
  confirmed wrong FIX: Shinjuku 69, PPC tokyo/run3 28 and nagoya/run3 16. It
  also adds missing epochs.
- **Next.** The Shinjuku episode points at the legacy hold overwrite with an
  overconfident, stale float. A narrower fix than the full Kalman hold, keeping
  the state consistent when the held integers move it, needs its own
  reproduction test and frozen evaluation.

## Reproduce

1. Build develop `2219d1d` and `export_truth_dd` as in the
   [long-lock note](rtk_long_lock_overconfidence.md).
2. Export the truth DD for each Trimble drive and each PPC run. PPC uses the
   header base position.
3. Run default, CDM, CDME and CDMEH with the argv of the
   [CDMH note](rtk_cdmh_ppc_evaluation.md). Run RTKLIB demo5 b34k with
   `scripts/rtklib_odaiba.conf`.
4. Audit each run:

   ```sh
   python3 scripts/analysis/truth_dd/phase_audit.py <reference.csv> <truth_dd.csv> out.json \
     CDME=lib:<cdme.pos> RTKLIB=rtklib:<rtklib.pos>
   ```

The JSON records both frozen rules, the binary and truth DD hashes, every
label count, and the gates.
