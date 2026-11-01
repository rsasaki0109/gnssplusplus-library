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
  - **Correction, for Data13:** its slope came from a conversion error,
    not from a missed segment end. See
    [the section below](#later-a-conversion-error-behind-data13).
- **Both wrong flips are on runs whose `--out` is unusable either way.**
  Those outputs are hundreds of kilometres off in both arms (Data13 in both
  modes, Data15 under `--navi776-tc`). That is a separate problem these runs
  show. The rule counts the flips regardless, as it should.
- **Data01 never latches.** Its rover file has no Doppler observations, so
  there is no GNSS velocity, and neither filter aligns the heading in either
  mode. The run tests nothing about the check.

## Later: a conversion error behind Data13

This was found after the record above, the same day. It changes neither
decision.

- **Cause.** The Data13 IMR file starts with 0.55 s in which two record
  streams interleave 0.17 ms apart (TOW 362320.00–362320.55, the first
  second of the run).
  - Each record holds a full 5 ms period of increments.
  - The converter divided each by its measured interval. So 111 rate samples
    were about 29 times too large, at the start of the static alignment.
- **Fix.** The converter now drops a record that is less than a quarter
  period behind the kept one. The manifest counts these as
  `duplicate_records_dropped`.
  - Of the eleven runs converted for the two SmartPNT holdouts, only Data13's
    `imu.csv` changes (111 records dropped). The other ten are byte-identical.
  - That includes Data11 and Data12. Their raw IMR files were fetched again,
    and the SHA256 matches the earlier record.
  - Data04 (2 ms) and Data16 (3 ms) each have one timestamp phase shift.
    Such a record holds a full period like any other, and it is kept as
    before.
- **Data13 on the fixed conversion.** Same binary (SHA256 `67b18893…a9220`),
  same four commands. Fused `--out` 3D p50 / p90 (m):

  | Mode | Arm | Holdout conversion | Fixed conversion |
  | --- | --- | --- | --- |
  | (a) | control | 973,382 / 28,039,416 | 0.159 / 0.79 |
  | (a) | candidate | identical to control, 0 flips (20.3) | identical to control, 0 flips (0.73) |
  | (b) | control | 202,006 / 1,837,462 | 0.102 / 0.66 |
  | (b) | candidate | 250,993 / 3,505,693, 1 flip (−5.18) | identical to control, 0 flips (0.74) |

  - **(b) RTK stream:** identical to the holdout's. Without tight coupling,
    the RTK half does not use the IMU.
  - **(a) RTK stream:** changes, 3D p50 / p90 0.110 / 3.26 m → 0.101 /
    0.59 m.
- **The (b) flip on Data13 came from the corrupted IMU input.** On the fixed
  conversion the slope is 0.74 and nothing flips. The shared signature
  above is left as written; for the wrong flips it now rests on Data15
  alone.
- **This does not turn (b) into a pass.**
  - On the fixed conversion, no forward-class run of (b) would flip. So (b)
    would meet the rule.
  - But the fix was found by looking at the scored runs, and the eight runs
    are development data now.
  - The contract rules out re-scoring. Making the check a default still needs
    its own holdout on new data.
  - (a) fails on Data15 either way.
- **Data15 under `--navi776-tc` stays unexplained.**
  - Its IMR file has no short records.
  - The car stands still for the first 539 s. Under the consider update
    before the latch (which the output filter of (a) uses), the pitch
    drifts during that time. In a diagnostic run without tight coupling,
    with `--consider-before-heading-latch`, it reaches 14.8° by TOW 177900;
    the truth is 2.0°, and the same run without the consider update gives
    2.06°.
  - The cause of that drift is not established.

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
