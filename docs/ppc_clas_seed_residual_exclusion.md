# CLAS seed: claslib large-residual exclusion

The kinematic CLAS PPP-RTK path now gives its SPP seed the pseudorange
large-residual exclusion of claslib (`exclsat()`). MRTKLIB v0.7.4 applies the
same exclusion to its private CLAS seed copy, with the same parameters: 20 m
and more than 7 rows. On the six PPC runs FIX rises from 25.117% to 27.232%.
There are still no FIX epochs above 3 m, and FIX RMS2D goes from 0.359 m to
0.361 m. Set `GNSS_PPP_CLAS_SEED_RESIDUAL_EXCLUSION_M=0` to restore the
previous seed.

## Why

The kinematic CLAS filter takes its position from the SPP seed in two cases:

- after every full state reset;
- when the filter is more than 10 m from the seed for 6 epochs in a row
  (MRTKLIB `maxdiffp` / `poserrcnt`).

On PPC tokyo/run2 the baseline was reset this way 286 times. For each reset
the filter position before the reset was compared with the seed, against the
reference trajectory:

| At the 286 maxdiffp resets | Count |
| --- | ---: |
| Filter closer to the truth than the seed | 202 |
| Seed closer to the truth than the filter | 84 |
| Filter horizontal error, median | 2.5 m |
| Seed horizontal error, median | 5.7 m |

So most resets replaced a better filter with a worse seed. After the reset the
filter needs new locks and a new float before AR can run again.

The native seed followed MRTKLIB v0.5.1 `pntpos()`, which has no `exclsat()`.
Upstream claslib does have it, and MRTKLIB v0.7.4 (PR #248) restored it for
the CLAS seed only (`mrtk_rtkpos.c` `rtkpos()`, `mrtk_spp.c` `exclsat()`).

## Change

- **`SPPConfig`**
  - Two fields: `large_residual_exclusion_m` (default 0, off) and
    `large_residual_exclusion_min_rows` (default 7).
  - From the second iteration, the solver looks at the previous iteration's
    residuals. It does so only while that iteration's row count is above the
    minimum.
  - The row count includes MRTKLIB's rank-constraint rows: one per unused
    time system of `NT = 7`.
  - If the residual RMS is within the threshold, every satellite whose
    residual exceeds it is dropped for the rest of the solve, including the
    final validation.
  - If the RMS is above the threshold, nothing is dropped. This is the same as
    `exclsat()`.
- **CLAS seed**
  - The kinematic CLAS parity seed sets 20 m and a minimum of 7, so it needs
    more than 7 rows.
  - Other SPP users keep the exclusion off, so their output is unchanged.
- **Opt-out**
  - `GNSS_PPP_CLAS_SEED_RESIDUAL_EXCLUSION_M` sets the seed threshold.
  - A value of 0 or below turns the exclusion off. The output then matches the
    previous baseline byte for byte; this was checked on tokyo/run2.
- **Tests**
  - `SPPLargeResidualExclusionTest.*`.

The parameters come from claslib and MRTKLIB and were not tuned on PPC.

## Results

Scoring as in the [validation note](ppc_clas_validation.md): raw PPC
reference, first 60 matched epochs discarded, status 6 is FIX. The baseline
`.pos` files are byte-identical to the previously published candidate
manifest.

| Run | FIX | FIX RMS2D | > 1 m | > 3 m | FIX < 0.3 m | lost / gained |
| --- | --- | --- | --- | --- | --- | --- |
| tokyo/run1 | 10.982 → 11.916% | 0.351 → 0.345 m | 81 → 83 | 0 → 0 | 1209 → 1310 | 4 / 115 |
| tokyo/run2 | 21.859 → 25.358% | 0.329 → 0.333 m | 21 → 21 | 0 → 0 | 1418 → 2062 | 29 / 347 |
| tokyo/run3 | 38.378 → 38.765% | 0.191 → 0.193 m | 49 → 49 | 0 → 0 | 5688 → 5716 | 18 / 77 |
| nagoya/run1 | 36.923 → 44.854% | 0.451 → 0.447 m | 18 → 26 | 0 → 0 | 533 → 611 | 2 / 600 |
| nagoya/run2 | 23.948 → 24.941% | 0.554 → 0.547 m | 0 → 0 | 0 → 0 | 0 → 0 | 12 / 105 |
| nagoya/run3 | 9.010 → 10.041% | 0.304 → 0.304 m | 0 → 0 | 0 → 0 | 338 → 356 | 0 / 53 |
| **Six runs** | **25.117 → 27.232%** | **0.359 → 0.361 m** | **169 → 179** | **0 → 0** | **9186 → 10055** | **65 / 1297** |

What the table shows:

- **FIX rate.** Every run gains FIX, and no run's maximum FIX error changes.
- **Nagoya accuracy.** Nagoya FIX sit at the known CLAS offset of that area,
  which is why nagoya/run2 has no FIX below 0.3 m in either column. Of the 600
  FIX gained on nagoya/run1, 517 are between 0.3 and 0.5 m and 8 are above
  1 m.
- **Epochs that are FIX in both runs.** On tokyo/run2, 404 of them move by more
  than 0.1 m, all closer to the truth: the median error goes from 0.388 m to
  0.176 m.
- **p68 and TTFF.**
  - p68 improves on tokyo/run1, tokyo/run2 (0.206 → 0.178 m), nagoya/run1 and
    nagoya/run2.
  - It is equal on tokyo/run3 and 3 mm worse on nagoya/run3.
  - TTFF does not change on any run.

The mechanism holds on tokyo/run2:

- maxdiffp resets: 291 → 259 (the debug log also counts resets of epochs outside
  the scored window);
- FLOAT-count resets: 320 → 296;
- maxdiff epochs in which AR is suppressed: 2369 → 2161.

## What did not work

These variants were run on all six runs before the port, while looking for
the cause of the post-reset gaps. None of them is in the code.

- **Run AR in maxdiff-only epochs, as MRTKLIB does.**
  - Native suppresses AR when the filter is more than 10 m from the seed.
  - Allowing it adds 652 FIX, but FIX above 3 m go from 0 to 4 (all on
    tokyo/run3) and FIX above 1 m from 169 to 330.
  - The new FIX start a few seconds after a reset, from a float 2–5 m off
    that the covariance calls converged.
- **Lower the hold-continuation track record**
  (`GNSS_PPP_CLAS_HOLD_CONT_MIN_TRACK`, default 60).
  - With 10: +479 FIX and no FIX above 3 m, but tokyo/run2 trades 113 FIX
    below 0.3 m for 234 FIX at 0.5–1 m. Over the six runs, FIX below 0.3 m
    rise by only 35.
  - With 0: one FIX above 3 m and 105 more above 1 m.
- **UrbanNav Tokyo as held-out data.**
  - The QZSS archive has the 2018-12-19 L6 files.
  - The parity profile solves only 13% of those epochs as PPP (the rest are
    SPP), so the data cannot test this change yet.

## MRTKLIB v0.7.7

The README compares against the MRTKLIB v0.4.2 article. MRTKLIB's own
`docs/reference/benchmark.md` now records v0.7.7 on the same six runs with
the same 60-epoch skip.

MRTKLIB notes that until its harness fix (#316 / #317), its runs that cross
a UTC hour used only the first L6 file. This affects tokyo/run3, nagoya/run1
and nagoya/run3, so the v0.4.2 values for those three runs are not
comparable.

| Run | MRTKLIB v0.7.7 FIX | its FIX < 30 cm | native FIX (this change) | native FIX < 30 cm |
| --- | ---: | ---: | ---: | ---: |
| tokyo/run1 | 32.4% | 88.0% | 11.9% | 92.6% |
| tokyo/run2 | 48.3% | 80.5% | 25.4% | 89.5% |
| tokyo/run3 | 58.4% | 93.6% | 38.8% | 96.8% |
| nagoya/run1 | 53.9% | 13.3% | 44.9% | 18.1% |
| nagoya/run2 | 34.0% | 2.5% | 24.9% | 0.0% |
| nagoya/run3 | 14.8% | 71.9% | 10.0% | 69.0% |

How the two compare:

- **Tokyo.** MRTKLIB v0.7.7 fixes far more epochs, and still has more FIX
  below 30 cm per scored epoch. On tokyo/run2 that is 38.9% against 22.7%.
- **Misfix tail.** MRTKLIB carries a heavier tail: its tokyo/run2 FIX 95th
  percentile is 7.8 m, while the native maximum is 1.3 m.

The changes between v0.5.1, which the native port follows, and v0.7.7 are the
next leads:

- **v0.6.5 OSR fixes.** E5a/L5 corrections, the same-frequency code fallback
  and per-epoch OSR clearing. Native still leaves Galileo out of the CLAS
  filter, as v0.5.1 does on this data.
- **v0.6.10 enhanced seed.** C/N0 weighting and TDCP jump rejection
  (`cn0+tdcp`). MRTKLIB reports +1.0 pp mean FIX on PPC.
- **v0.7.4 claslib sync.** Correction-stream timing, BackupCSSR/BackupGrid,
  the ionosphere Gauss-Markov decay, and the claslib slot layout with Galileo
  rows.

## Reproduce

```sh
python3 scripts/experiments/ppc/generate_ppc_clas_scorecard.py \
  --dataset-root data/PPC-Dataset --work-dir output/ppc_clas_full \
  --l6-cache output/ppc_clas_full/l6_cache --configs parity
# the previous seed:
GNSS_PPP_CLAS_SEED_RESIDUAL_EXCLUSION_M=0 python3 scripts/experiments/ppc/generate_ppc_clas_scorecard.py ...
```

Then score as in the [validation note](ppc_clas_validation.md).
