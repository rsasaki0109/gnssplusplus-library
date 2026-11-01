# Frozen holdout results: the reverse-start check of the heading latch

The [contract](smartpnt_reverse_start_holdout.md) was committed before any
estimator ran on its data.

- **Locally first:** as `7181811` at 03:52:29 UTC on 2026-11-01, on `ebb5fdb`
  (the head of #697, which adds the option). That was 15 s before the first
  run.
- **Then on `develop`:** #697 was squash-merged as `894ccc1`, whose tree is
  identical to `ebb5fdb`. The commit was replayed onto `develop` unchanged as
  `8d4e372`; `git range-diff` reports no change.

These are the results of its one run.

- **(a) under `--navi776-tc`: fail.**
- **(b) without tight coupling: fail.**

`--heading-latch-direction-regression` stays an opt-in.

## Run

- **When:** 2026-11-01, 03:52:44–04:02:23 UTC, the 32 commands of the
  contract, once each.
- **Binary:** SHA256 `67b18893…a9220`, the one the contract names.
- **Data:** the converted files of the contract, unchanged.
- **Every arm exited 0.** RTK streams are identical between control and
  candidate in all 16 pairs.

## Results

Fused `--out` 3D p50 / p90 (m), control → candidate. For forward-class runs
without a flip, the candidate's `--out` is identical to the control's.

| Run | Class | (a) flips (slope) | (a) `--out` | (b) flips (slope) | (b) `--out` |
| --- | --- | --- | --- | --- | --- |
| Data01 | reverse | 0 (none) | 545 / 186,987, identical | 0 (none) | 5,547 / 28,814, identical |
| Data14 | reverse | 1 (−1.27) | **405 / 27,223 → 0.90 / 21.5** | 1 (−1.06) | **0.62 / 4,301 → 0.53 / 2.0** |
| Data02 | forward | 0 (0.96) | 0.221 / 12.6 | 0 (1.00) | 0.116 / 14.9 |
| Data13 | forward | 0 (20.3) | 973,382 / 28,039,416 | **1 (−5.18)** | 202,006 / 1,837,462 → 250,993 / 3,505,693 |
| Data15 | forward | **1 (−5.35)** | 776,534 / 26,124,910 → 681 / 578,898 | 0 (0.99) | 0.598 / 10.5 |
| Data16 | forward | 0 (0.99) | 0.191 / 76.4 | 0 (0.86) | 0.166 / 77.0 |
| Data19 | forward | 0 (1.01) | 1.262 / 1.8 | 0 (0.99) | 0.018 / 0.1 |
| Data20 | forward | 0 (0.83) | 0.186 / 3.3 | 0 (1.11) | 0.112 / 0.2 |

- **(a) fails** because the forward-class run Data15 flips. Data14 flips
  and is better.
- **(b) fails** because the forward-class run Data13 flips. Data14 flips
  and is better.

## After scoring: what happened

The eight runs are development data from here on. This look came after the
decisions and does not change them. Diagnostic reruns of the four arms below,
with `LIBGNSS_DEBUG_HEADING=1`, reproduce the scored `--out` streams byte for
byte.

- **Data14 works as designed.** The car reverses out at about 1.2 m/s.
  - The latch at TOW 439005 takes the course, 177° from the truth heading.
  - It has 2 intervals, so the decision waits for the end of the segment.
    At the stop 14 s later, the slope is −1.27 over 16 intervals, and the
    flip makes the heading right.
- **The two wrong flips latch in forward motion that follows a slow reverse
  or a stop.**
  - **Data15, (a):**
    - The car reverses at about 1 m/s until TOW 178300 and slows to
      0.10 m/s (truth) at 178301.
    - It then drives forward. The latch at 178308 is in that forward
      motion and takes the right course.
    - The six intervals behind the decision straddle the turn-around, with
      no stop detected between, and give −5.35.
  - **Data13, (b):** The car stands still from TOW 363882 to 363888 and then
    drives forward. The latch at 363893 is right, and the slope of its five
    intervals is −5.18.
  - **Shared signature.** In both cases the slope is far from the −1 of a
    real reverse. The development and holdout reverse starts gave −0.81 to
    −1.27; these two gave about −5. A segment end that the filter does not
    see (GNSS speed not below 0.2 m/s, no stationary IMU sample) lets
    intervals of opposite directions into one regression.
- **Both wrong flips are on runs whose `--out` is unusable either way.**
  Those outputs are hundreds of kilometres off in both arms (Data13 in both
  modes, Data15 under `--navi776-tc`). That is a separate problem these runs
  show. The rule counts the flips regardless, as it should.
- **Data01 never latches.** Its rover file has no Doppler observations, so
  there is no GNSS velocity, and neither filter aligns the heading in either
  mode. The run tests nothing about the check.

## What follows

- **Nothing changes.** `--heading-latch-direction-regression` stays an opt-in
  in both modes.
- **The runs are not repeated, and no other data is added to compensate.**
  The eight runs are development data from now on.
- **A next version would need its own holdout on new data.** Two changes that
  the findings above suggest:
  - accept a flip only for a slope near −1 (−5.35 and −5.18 against
    −0.81 … −1.27);
  - end a segment on a GNSS speed minimum, not only below 0.2 m/s.

  Both were chosen after seeing these runs.
