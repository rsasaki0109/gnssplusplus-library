# Frozen holdout results: re-aligning the output filter before the heading latch

The [contract](smartpnt_prelatch_realign_holdout.md) was committed before any
estimator ran on its data: locally as `ee17ebc` at 22:28:24 UTC on
2026-11-01, on `cd77ecf` (`develop`). The run was started right after that
commit, in the same second. These are the results of its one run.

**Decision: pass.** One run is better, none is worse, and the RTK streams are
byte-identical on all five.

## Run

- **When:** 2026-11-01, 22:28:24–22:31:46 UTC, the ten commands of the
  contract, once each.
- **Binary:** SHA256 `d2240233…c3d9`, the one the contract names.
- **Data:** the converted files of the contract, unchanged.
- **Every arm exited 0.**

## Results

Fused `--out` 3D p50 / p90 (m), scored by
`scripts/analysis/score_fused_output.py`. Both arms match the same number of
epochs on each run.

| Run | Matched epochs | Control | Candidate | p50 | p90 | Run outcome | RTK stream |
| --- | ---: | ---: | ---: | --- | --- | --- | --- |
| Data03 | 9,531 | 2.762 / 7.572 | 2.762 / 7.572 | equal | equal | equal | identical |
| Data04 | 6,392 | 0.0296 / 2.365 | 0.0295 / 2.344 | equal | equal | equal | identical |
| Data05 | 1,881 | 0.0774 / 1.194 | 0.0737 / 1.195 | lower | equal | **better** | identical |
| Data17 | 5,461 | 0.1229 / 1.283 | 0.1229 / 1.283 | equal | equal | equal | identical |
| Data18 | 9,559 | 0.2195 / 3.201 | 0.2195 / 3.201 | equal | equal | equal | identical |

## Reported, not in the rule

- **The effect is small here.** The pass rests on one run: Data05's p50
  drops by 3.7 mm (4.8%), outside the 1% band.
- **Not byte-identical.** The other four runs are equal within the band, but
  their fused streams are not byte-identical: the candidate re-aligned on
  every run.
- **What the holdout does not show.** None of these runs has a start-up
  transient of the Data15 size, so the holdout does not test the large
  failure that motivated the change. It shows no harm on new data, not the
  Data15 repair.

## After scoring: why the effect is small

The five runs are development data from here on. This look came after the
decision and does not change it.

- **Size of the transient.** Gyro mean of the first 2 s against the mean of
  the stand-still before the first motion, as in the contract's
  development scan:

  | Run | Stand-still from the first IMU sample | Tilt rate | Tilt by the first motion |
  | --- | ---: | ---: | ---: |
  | Data03 | 3,021 s | 0.0046 °/s | 13.8° |
  | Data04 | 1,526 s | 0.0015 °/s | 2.2° |
  | Data05 | 372 s | 0.0067 °/s | 2.5° |
  | Data17 | 377 s | 0.0011 °/s | 0.4° |
  | Data18 | 379 s | 0.0012 °/s | 0.5° |

  Data15 had 0.086 °/s.
- **Data03.** Its IMU starts 2,547 s before the first GNSS epoch, so its IMU
  stand-still is 3,021 s, not the 474 s of the truth. Its first 2 s also
  carry a −0.28 °/s z-gyro transient. That one is about the yaw axis, which
  the heading latch overrides.
- **Re-alignments made by the candidate** (output filter): Data03 2,015,
  Data04 1,514, Data05 362, Data17 370, Data18 372.

## What follows

- **A PR may make the re-alignment** part of the consider update before the
  latch, without a new option, with tests, this record and the development
  table. Merging it remains the maintainer's decision.
- **The runs are not repeated, and no other data is added to compensate.**
  SmartPNT-POS Data03, 04, 05, 17 and 18 are development data from now on.
