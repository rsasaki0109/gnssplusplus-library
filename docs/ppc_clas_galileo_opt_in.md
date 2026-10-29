# CLAS Galileo rows (opt-in)

`GNSS_PPP_CLAS_GALILEO=1` adds Galileo E1/E5a rows to the kinematic CLAS
PPP-RTK filter. The default is off, so the published six-run gate (GPS+QZSS
only, no FIX above 3 m) is unchanged.

On the six PPC runs the opt-in raises the FIX rate from 27.23% to 35.87%, and
the share of epochs with a FIX within 0.3 m from 17.26% to 24.53%. It also
adds 76 FIX epochs above 3 m, 69 of them in one 35 s window of tokyo/run1.
Use it when more accurate fixes are worth an occasional large wrong fix.

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

These two patterns are the open work before the profile can become a
default. All six runs were used both to find the mechanisms and to measure
them; no held-out CLAS kinematic data with truth is available here.

## What did not work

These variants were tried with Galileo rows on; none of them is in the code.

- **Galileo in the float only, not in AR.** Accurate FIX reach only 19.26%,
  and tokyo/run3 still has 22 FIX above 3 m.
- **FDE only when the seed fails validation (as MRTKLIB does).**
  - With Galileo: FIX above 3 m drop from 105 to 36, but FIX above 1 m rise
    to 1197.
  - Without Galileo, FIX drop to 25.29% and FIX above 1 m rise to 890.
- **The pre-FDE reset seed without Galileo.** nagoya/run2 loses 320 FIX. The
  seed change is therefore tied to the Galileo flag.

## Reproduce

```sh
python3 scripts/experiments/ppc/generate_ppc_clas_scorecard.py \
  --dataset-root data/PPC-Dataset --work-dir output/ppc_clas_galileo \
  --l6-cache output/ppc_clas_full/l6_cache --configs parity
# with GNSS_PPP_CLAS_GALILEO=1 exported for the opt-in
```

The MRTKLIB comparison used MRTKLIB v0.7.9 `mrtk post` with
`conf/benchmark/clas.toml` and the city configuration, the concatenated
hourly L6 files of each run, and `-ts`/`-te` 60 s outside the run window. The
GPS+QZSS run adds a configuration with `[positioning] systems = ["GPS",
"QZSS"]`.
