# Design-slice FLOAT root-cause analysis (Tokyo run1 epochs 5000--5499)

## Summary

Every mechanism proposed to recover the design-slice FLOAT epochs was tested
and falsified. The 167 FLOAT epochs are the result of the existing safety
mechanisms (CMC level exclusion and the surplus-quality nsat floor) correctly
blocking wrong fixes. Relaxing any of them turns a large number of epochs
FIXED at ~2.9 km horizontal error.

| Proposed recovery | Result |
|---|---|
| Active selective arc-restart | no carrier arc to restart (CP-hold-only or build-time-excluded) |
| `--low-count-ar` (min 4 / 3, ratio 0) | 48 LAMBDA attempts, 0 accepted |
| `surplus_rescue_quality_pass` nsat/ddpr floor relaxation | 6 epochs FIXED, all wrong (~2.9 km) |
| `--cmc-level 2.0` (relax CMC exclusion) | 40 epochs FIXED, all wrong (~2.9 km) |

## The 167 FLOAT epochs decompose as

- 120 CP-hold epochs (carrier suppressed by the existing FSM) -- ar_outcome 4.
- 45 LAMBDA-insufficient epochs (eligible candidate count 2--5, below the
  per-epoch floor of 6) -- ar_outcome 5, `lambda_attempts == 0`.
- 2 other (ar_outcome 8).

## Root cause of the 45 LAMBDA-insufficient epochs

The eligible ambiguity count collapses at slice epoch 334 (TOW 188537.6):
`E36->E09` flips from eligible (amb 32) to build-time-excluded, dropping the
count from 5 to 2, and the epoch loses FIX. The excluded rows are dominated
by a few satellites:

| satellite | excluded rows |
|---|---:|
| C13 | 433 |
| C40 | 254 |
| C38 | 209 |
| J03 | 141 |
| E36 | 102 |

These rows are excluded by the CMC (code-minus-carrier) level screen
(`code_minus_carrier_level_threshold_m`, profile `--cmc-level 0.75`): a
(satellite, signal) whose CMC drifts more than 0.75 m from its EWMA baseline
is removed from LAMBDA at build time (fgo.cpp `cmc_level_exclude_this_epoch`,
fgo_gtsam_backend.cpp `BuildTimeExcluded`). The `amb_excl_one_band` counter is
0 -- this is CMC exclusion, not the one-band-per-satellite rule.

## Why relaxing the gates produces wrong fixes

- **CMC relaxation (`--cmc-level 2.0`)**: FIX 333 -> 373, but all 40 new fixes
  are wrong at ~2.9 km. The excluded carrier rows carry contaminated
  ambiguity (multipath-corrupted code-carrier level), and admitting them makes
  LAMBDA select self-consistent-but-wrong integers. The screen exists
  precisely to prevent this.
- **`surplus_rescue_quality_pass` nsat/ddpr floor relaxation**: 6 epochs
  FIXED, all wrong. The floors encode the independent-satellite coverage and
  code-residual bounds that distinguish a correct sparse-geometry integer
  from a wrong one; ratio and FFRT alone cannot (they validate internal
  consistency only).
- **`--low-count-ar`**: even with min_candidates=3 and ratio 0, 0 accepted.
  The surviving candidates do not pass the surplus quality gate; and the 8
  "clean" epochs (335--367, ratio 10^6, FFRT pass) are also wrong integers --
  their float solutions sit at 166 m fixed offset from truth.

## Conclusion

The design-slice FLOAT rate is dominated by epochs whose float solutions are
already biased (50--500 m from truth for 166/167 FLOAT epochs), which is the
downstream symptom of carrier rows being CMC-excluded (or held by CP-hold).
Restoring those rows to AR, lowering the candidate floor, or relaxing the
surplus quality gates all produce wrong FIX at kilometre scale. The safety
mechanisms are functioning as intended.

Any future attempt to raise FIX rate on this slice must improve the QUALITY
of the float ambiguity estimate itself (so the excluded/held epochs carry a
correct float solution before AR), which is a distinct research thread from
the quarantine/arc-restart work and from threshold relaxation. No sealed
run2/run3 holdouts are spent on any of the falsified directions.
