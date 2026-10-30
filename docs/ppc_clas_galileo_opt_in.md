# CLAS Galileo rows (opt-in)

`GNSS_PPP_CLAS_GALILEO=1` adds Galileo E1/E5a rows to the kinematic CLAS
PPP-RTK filter. The default is off, so the published six-run gate (GPS+QZSS
only, no FIX above 3 m) is unchanged.

On the six PPC runs the opt-in raises the FIX rate from 27.23% to 35.87%, and
the share of epochs with a FIX within 0.3 m from 17.26% to 24.53%. It also
adds 76 FIX epochs above 3 m, 69 of them in one 35 s window of tokyo/run1.
Use it when more accurate fixes are worth an occasional large wrong fix.

Adding `GNSS_PPP_CLAS_FIRST_FIX_AR_ALPHA=1` gives a conservative Galileo
profile. The ratio test that starts a FIX segment then uses MRTKLIB's 1%
significance level instead of 10%. On the six runs this removes every FIX
above 3 m and cuts FIX above 1 m from 179 (default) to 102. FIX within 0.3 m
are 19.28% of epochs, against 17.26% for the default. The cost is the FIX
rate: 25.70% against 27.23%. tokyo/run2 drops to 19.99%, below the 21.7%
MRTKLIB v0.4.2 target, so this profile also stays opt-in; see
[Stricter first fix](#stricter-first-fix).

## Why Galileo was missing

- **The filter.** The native filter follows MRTKLIB v0.5.1, whose CLAS
  residual path never forms a Galileo row on this data.
- **The cause: bias identity.**
  - CLAS broadcasts the Galileo code and phase biases for E1 B+C (`L1X`) and
    E5a I+Q (`L5X`).
  - The PPC receiver reports `C1C`/`L1C` and `C5Q`/`L5Q`.
  - The exact signal lookup therefore finds no code bias, and the cell is
    dropped.
- **MRTKLIB's fix.** MRTKLIB v0.6.5 added a pass that takes both biases from
  the first CSSR signal on the same frequency band (`clas_osr_corrmeas()`
  pass 2, `code_on_same_freq()`).
- **What this means on the six PPC runs.** MRTKLIB v0.7.9 run on them, scored
  the same way as native:

| | FIX | FIX within 0.3 m | FIX above 1 m | FIX above 3 m |
| --- | ---: | ---: | ---: | ---: |
| MRTKLIB v0.7.9, GPS+Galileo+QZSS | 43.13% | 28.15% | 2464 | 1245 |
| MRTKLIB v0.7.9, GPS+QZSS (`systems = ["GPS", "QZSS"]`) | 29.92% | 19.17% | 1462 | 839 |
| native, default (GPS+QZSS) | 27.23% | 17.26% | 179 | 0 |

The percentages are shares of scored epochs.

- **Same constellations.** With GPS+QZSS on both sides, native is close to
  MRTKLIB on accurate FIX and has far fewer wrong ones.
- **Galileo is the lever.** In MRTKLIB it is what turns 19% accurate FIX into
  28%.
- **Reproduction check.** The MRTKLIB runs reproduce its own published
  v0.7.7 table; tokyo/run2, for example, is 4392 FIX, 48.3%, 2.546 m RMS2D.

## Change

Everything below is active only with `GNSS_PPP_CLAS_GALILEO=1`.

1. **Galileo rows.**
   - The kinematic parity path no longer skips Galileo.
   - This covers bias-state seeding, code and phase rows, the outage
     lifecycle and the fixed-solution validation rows.
   - Galileo forms its own DD group on E1 and on E5a.
2. **Same-band bias fallback (Galileo).**
   - When no CSSR signal matches the observation code, the code and phase
     biases come from the same-band signal (`sameBandBiasCode()`).
   - On tokyo/run2 at TOW 177600 the resulting E04/E10/E11/E19/E25 phase and
     code biases equal MRTKLIB's `OSRRES` trace values exactly.
3. **Full resets start from the pre-FDE seed.**
   - The native seed is solved again with RAIM FDE whenever it is more than
     10 m from the filter. That FDE subset can be several metres worse than
     the full solution.
   - On tokyo/run1, around TOW 187955, the full solution was 2–5 m off and
     the FDE seed 12–15 m off.
   - With Galileo rows, a reset from the FDE seed re-converged onto that
     offset. The FDE seed is still used for the maxdiff bookkeeping.
   - MRTKLIB runs FDE only when `valsol()` fails.
4. **Fresh-position code gate.**
   - While the position variance is above 100 m² (just after a reset), DD
     code rows whose innovation exceeds 20 m are dropped. 20 m is claslib's
     SPP exclusion threshold.
   - At that stage the innovation test cannot reject a NLOS row. On tokyo/run1,
     E04 at −20/−25 m and G07 at −28 m pulled a re-initialized position by
     10–26 m.

Only the fallback helper is new in the bias identity header
(`rtklibCodeBand()`, `sameBandBiasCode()`); `PPPMultifrequencyTest.FindsClasGalileoBiasOnTheSameFrequencyBand`
covers it. With the flag unset all six `.pos` files are byte-identical to the
current default.

## Results

Scoring as in the [validation note](ppc_clas_validation.md). The table
compares the default with `GNSS_PPP_CLAS_GALILEO=1`.

| Run | FIX | FIX within 0.3 m (of epochs) | FIX RMS2D | > 1 m | > 3 m | max FIX |
| --- | --- | --- | --- | --- | --- | --- |
| tokyo/run1 | 11.916 → 23.618% | 11.04 → 22.87% | 0.345 → 2.397 m | 83 → 79 | 0 → 69 | 1.96 → 22.66 m |
| tokyo/run2 | 25.358 → 37.925% | 22.68 → 36.86% | 0.333 → 0.201 m | 21 → 4 | 0 → 2 | 1.33 → 6.03 m |
| tokyo/run3 | 38.765 → 48.717% | 37.51 → 46.63% | 0.193 → 0.363 m | 49 → 195 | 0 → 0 | 2.99 → 2.67 m |
| nagoya/run1 | 44.854 → 46.319% | 8.10 → 7.68% | 0.447 → 0.390 m | 26 → 2 | 0 → 0 | 1.04 → 1.09 m |
| nagoya/run2 | 24.941 → 31.424% | 0.00 → 0.04% | 0.547 → 1.399 m | 0 → 46 | 0 → 1 | 0.77 → 70.27 m |
| nagoya/run3 | 10.041 → 15.167% | 6.93 → 10.49% | 0.304 → 0.814 m | 0 → 63 | 0 → 4 | 0.58 → 6.80 m |
| **Six runs** | **27.23 → 35.87%** | **17.26 → 24.53%** | **0.361 → 1.072 m** | **179 → 389** | **0 → 76** | |

What each part contributes (six runs):

| Profile | FIX | FIX within 0.3 m | FIX RMS2D | > 1 m | > 3 m |
| --- | ---: | ---: | ---: | ---: | ---: |
| default | 27.23% | 17.26% | 0.361 m | 179 | 0 |
| Galileo rows and bias fallback | 35.25% | 23.76% | 1.160 m | 989 | 105 |
| + pre-FDE reset seed | 35.49% | 24.04% | 1.019 m | 401 | 89 |
| + fresh-position code gate (the opt-in) | 35.87% | 24.53% | 1.072 m | 389 | 76 |

What the results show:

- **Galileo accuracy is not the problem.** At accurate FIX epochs on
  tokyo/run1, the Galileo DD code residuals have a per-satellite median of
  0.2 m. GPS is at 0.28 m.
- **The wrong fixes start just after a reset.**
  - The float is still metres off, typically 1–9 s after a full reset.
  - The first ratio is usually barely above the threshold (3.0–5.8) with 6–11
    ambiguities.
  - SPP and filter are often offset in the same direction, so the 10 m
    maxdiff check does not fire.
- **The tokyo/run1 window (TOW 187936–187972).** The gate recovers the float
  once. The next 15-epoch FLOAT reset then wipes that recovered float, and the
  re-initialization is pulled away again by rows below the 20 m gate.

These two patterns were the open work before the profile could become a
default. All six runs were used both to find the mechanisms and to measure
them; no held-out CLAS kinematic data with truth is available here.

## Stricter first fix

`GNSS_PPP_CLAS_FIRST_FIX_AR_ALPHA` sets the MRTKLIB F-test significance level,
in percent, for a fix that would start a FIX segment, that is, when the
previous epoch published no FIX. The accepted levels are the six rows of
MRTKLIB's `qf` table (`pos2-aralpha`): 0.1, 0.5, 1, 5, 10 and 20. A fix that
continues a segment keeps the clas.toml 10%. Unset, or a level outside the
table, leaves every fix at 10% and the output byte-identical.

**Why the first fix.** The wrong fixes were traced back to their reset on
all six runs.

- **The reset.** A full reset (FLOATCNT or maxdiffp) re-initializes the
  filter from an SPP seed that is several metres off. On tokyo/run1 at TOW
  187925.4, maxdiffp replaced a filter 1.0 m from the truth with a seed
  10.8 m off.
- **The float.** Within 1–3 s the float settles 3–20 m from the truth. The
  code errors at 5 Hz are correlated in time, but the filter averages them
  as white noise, so its covariance already looks converged.
- **The first fix.** The first fix then comes at a ratio of 3.0–5.8 with
  7–11 ambiguities, just above the 10% threshold (2.2–2.8). Fix-and-hold then
  keeps it.

**No per-epoch test separated them.** These quantities were compared between
the first fixes of correct and wrong FIX segments:

- the bootstrapped success rate of the accepted candidate: 0.90–0.998 for
  wrong fixes, and as low as 0.6 for correct ones;
- the fixed-failure-rate ratio test (Hou et al. 2016, Pf = 0.1%);
- the filter–SPP distance;
- the size of the float update.

A stricter significance level for every first fix was the only change that
removed the wrong fixes without removing most correct ones.

Results with `GNSS_PPP_CLAS_GALILEO=1 GNSS_PPP_CLAS_FIRST_FIX_AR_ALPHA=1`,
against the default:

| Run | FIX | FIX within 0.3 m (of epochs) | FIX RMS2D | > 1 m | > 3 m | max FIX |
| --- | --- | --- | --- | --- | --- | --- |
| tokyo/run1 | 11.916 → 18.921% | 11.04 → 18.89% | 0.345 → 0.058 m | 83 → 0 | 0 → 0 | 1.96 → 0.39 m |
| tokyo/run2 | 25.358 → 19.991% | 22.68 → 19.45% | 0.333 → 0.119 m | 21 → 0 | 0 → 0 | 1.33 → 0.47 m |
| tokyo/run3 | 38.765 → 44.090% | 37.51 → 43.31% | 0.193 → 0.172 m | 49 → 102 | 0 → 0 | 2.99 → 1.34 m |
| nagoya/run1 | 44.854 → 19.631% | 8.10 → 2.14% | 0.447 → 0.386 m | 26 → 0 | 0 → 0 | 1.04 → 0.56 m |
| nagoya/run2 | 24.941 → 23.469% | 0.00 → 0.03% | 0.547 → 0.464 m | 0 → 0 | 0 → 0 | 0.77 → 0.77 m |
| nagoya/run3 | 10.041 → 9.871% | 6.93 → 8.98% | 0.304 → 0.247 m | 0 → 0 | 0 → 0 | 0.58 → 0.49 m |
| **Six runs** | **27.23 → 25.70%** | **17.26 → 19.28%** | **0.361 → 0.253 m** | **179 → 102** | **0 → 0** | |

What the table shows:

- **Accuracy.** No run has a FIX above 1.34 m, and FIX RMS2D falls on every
  run. tokyo/run3 is the exception for FIX above 1 m: 102 against 49, all
  between 1.0 and 1.34 m. The other runs have none.
- **FIX rate.**
  - tokyo/run1 and tokyo/run3 gain FIX.
  - tokyo/run2 and nagoya/run1 lose many. Their fixes come mostly in short
    segments right after a reset.
  - tokyo/run2 (19.99%) and nagoya/run2 (23.47%) are measured against the
    MRTKLIB v0.4.2 targets of 21.7% and 23.4%. tokyo/run2 misses its target,
    so the hard sign-off is not met.
- **Without Galileo.** The same setting on the GPS+QZSS default halves the
  FIX rate to 13.77%. The fewer ambiguities leave no headroom over the
  stricter thresholds, so use it only with `GNSS_PPP_CLAS_GALILEO=1`.

**How 1% was chosen.** The level was chosen on these same six runs, among
MRTKLIB's table values:

| First-fix level (Galileo on) | FIX | FIX within 0.3 m | > 1 m | > 3 m |
| --- | ---: | ---: | ---: | ---: |
| 10% (the opt-in) | 35.87% | 24.53% | 389 | 76 |
| 5% | 33.52% | 24.11% | 236 | 22 |
| 1% | 25.70% | 19.28% | 102 | 0 |
| 0.5% | 23.16% | 17.60% | 101 | 0 |

1% is the least strict level without a FIX above 3 m.

## What did not work

These variants were tried with Galileo rows on; none of them is in the code.

While looking for the stricter first fix (six runs, Galileo rows on):

- **A 10 m fresh-position code gate** (claslib's `maxdiffp` instead of 20 m).
  FIX above 3 m rise from 76 to 107. Removing one wrong fix shifts the reset
  timing and creates others.
- **No AR for 5 s after a full reset.** FIX collapse to 2.87%. The 15-epoch
  FLOAT reset (3 s at 5 Hz) leaves no float older than 5 s.
- **A 75-epoch FLOAT reset** (15 s at 5 Hz). FIX above 3 m rise to 108 and FIX
  above 1 m to 721.
- **Code variance inflated 5x or 25x**, to model the time correlation. FIX
  above 3 m: 27 and 133.
- **1% for every fix**, not only the first. FIX 24.69%, and nagoya/run1 drops
  to 11.58%, below its 17.0% target.
- **1% only until the first fix after a reset.** Re-fixes after a lost fix
  then bring back 32 FIX above 3 m.
- **1% only when the candidate contains Galileo ambiguities.**
  - FIX 27.99%, and tokyo/run2 is back at 24.46%.
  - Two FIX above 3 m remain. One of them is a single 70 m epoch on
    nagoya/run2, after an 8 s SPP-only stretch re-initialized the filter from
    a seed 68 m off.
- **Keeping the filter position across a FLOAT reset.** No improvement.

- **Galileo in the float only, not in AR.** Accurate FIX reach only 19.26%,
  and tokyo/run3 still has 22 FIX above 3 m.
- **FDE only when the seed fails validation (as MRTKLIB does).**
  - With Galileo: FIX above 3 m drop from 105 to 36, but FIX above 1 m rise
    to 1197.
  - Without Galileo, FIX drop to 25.29% and FIX above 1 m rise to 890.
- **The pre-FDE reset seed without Galileo.** nagoya/run2 loses 320 FIX. The
  seed change is therefore tied to the Galileo flag.

The PPC IMU did not help either: as the reset position, as the time update,
or as a check of the first fix. See
[CLAS with the PPC IMU](ppc_clas_imu_experiments.md).

## Reproduce

```sh
python3 scripts/experiments/ppc/generate_ppc_clas_scorecard.py \
  --dataset-root data/PPC-Dataset --work-dir output/ppc_clas_galileo \
  --l6-cache output/ppc_clas_full/l6_cache --configs parity
# with GNSS_PPP_CLAS_GALILEO=1 exported for the opt-in, and
# GNSS_PPP_CLAS_FIRST_FIX_AR_ALPHA=1 added for the stricter first fix
```

The MRTKLIB comparison used MRTKLIB v0.7.9 `mrtk post` with
`conf/benchmark/clas.toml` and the city configuration, the concatenated
hourly L6 files of each run, and `-ts`/`-te` 60 s outside the run window. The
GPS+QZSS run adds a configuration with `[positioning] systems = ["GPS",
"QZSS"]`.
