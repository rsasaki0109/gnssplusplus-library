# Frozen holdout contract: default switch to `velocity_consistency_v9` (v2)

This contract is frozen in a commit before any candidate replay of the holdout
data, and before any estimator of this repository has been scored on it. It
decides whether the online PVA production default may be proposed to change to
`velocity_consistency_v9`. The development record for that candidate is
[online_pva_candidate_v10.md](online_pva_candidate_v10.md) and its
[results](online_pva_candidate_v10_results.md).

The candidate was built on the six PPC-Dataset runs and diagnosed on UrbanNav
Tokyo (Odaiba, Shinjuku), which was the holdout of
[contract v1](online_pva_default_switch_holdout_v1.md) and is development data
since. This contract evaluates the candidate, unchanged, on data that played no
part in its development: UrbanNav Hong Kong, Deep-Urban-1 and Harsh-Urban-1.

Go here does **not** switch the default by itself. Go on this population is
one of two conditions (see [Decision](#decision-and-the-later-population)). A
separate, reviewed PR would propose the switch to the maintainer. No-Go keeps
the default and is recorded like every earlier No-Go.

## Why the gates differ from contracts v1-v10

The per-scenario gates of contracts v1-v10 (and of holdout v1) require every
statistic to be at most 1.01 x the control on each of 3 scenarios of each run.
That is below the control's own scenario-to-scenario noise. UrbanNav Odaiba
control RTK position P95 is 47.84 / 28.45 / 26.79 m in the normal, GNSS-outage
and IMU-gap scenarios, which differ only by a 4-10 s disturbance: the control
moves by 21 m between scenarios. The candidate `velocity_consistency_v9` was
28.28 / 28.41 / 28.46 m. A 1.01x gate fails on the third scenario (26.79 m
against 28.46 m), a difference far smaller than the control's own variation
across the three.

The gates below pool the three scenario replays of a run before comparing, and
give each statistic the margin that its role needs: strict for the quantities
the candidate exists to improve, 10 % for the secondary ones. They were chosen
before any holdout data was scored, from this reasoning only; no new number was
computed to set them. The numbers in the rationale are the recorded v10 results.

## Holdout data

UrbanNav Hong Kong (IPNL-POLYU/UrbanNavDataset, Dropbox downloads), two runs:

| Run | Contract name | Date (GPST) | Truth span (GPS TOW, week 2158) | Converted rover epochs |
|---|---|---|---:|---:|
| Deep-Urban-1 | `HKDeepUrban1_novatel` | 2021-05-21 | 455342 - 456880 s | 1,492 (of 1,493) |
| Harsh-Urban-1 | `HKHarshUrban1_novatel` | 2021-05-18 | 184488 - 186799 s | 2,269 (of 3,307) |

- **Instruments:**
  - Rover: NovAtel Flexpak6, 1 Hz on integer GPS seconds, converted to RINEX
    3.03 by RTKCONV demo5 b33c in GPS time. GPS L1 C/A and L2 P(Y), GLONASS
    L1 C/A and L2 P, BeiDou B1I and B2I. No Galileo. Pseudorange, carrier
    phase, Doppler and C/N0 on each signal. No event records.
  - Base: HKSC (Hong Kong Lands Department, Leica GR50, 1 Hz, GPS, GLONASS,
    Galileo and BeiDou), RINEX 3.02, hourly files from
    `rinex.geodetic.gov.hk/rinex3/2021/<doy>/HKSC/1s/`. The header position
    (-2414266.9197, 5386768.9868, 2407460.0314) m is the base position.
  - Navigation: the HKSC daily broadcast files GN, RN, EN and CN of the same
    day (`HKSC00HKG_R_2021<doy>0000_01D_{GN,RN,EN,CN}.rnx.gz`).
  - IMU: Xsens, about 400 Hz (median step 2.5 ms), ROS bag CSV with Unix UTC
    `header.stamp`.
  - Truth: 1 Hz post-processed text file with UTC and GPS time, latitude and
    longitude as D M S, ellipsoidal height, body-frame velocity and
    acceleration, roll, pitch, heading and a quality flag Q.
- **Difficulty (truth and rover files only):**
  - Baselines to HKSC from the truth positions: 4.75 / 5.32 / 5.63 km
    (min / median / max) for Deep, 2.70 / 2.79 / 3.15 km for Harsh.
  - The vehicle is stopped (horizontal speed < 0.2 m/s) in 37 % of the Deep
    rows and 61 % of the Harsh rows. Maximum speed 11.1 and 10.5 m/s.
  - The rover file has epoch gaps (no observations): 9 gaps of more than 1 s
    in Deep (largest 13 s) and 11 in Harsh (largest 11 s). The replay's rover
    gap limit is 2 s, which 7 and 8 of them exceed.
  - Median satellites per rover epoch: 11 (Deep), 9 (Harsh).
  - Truth quality Q (converted rows): Deep 1/2/3/4/5 = 517/757/103/87/28,
    Harsh 1/2/3/4/5/6 = 46/1202/433/340/246/2. The meaning of Q is not
    documented in the files; no row is excluded by it.
- **Independence from development data:** a different city, year, vehicle, rover
  receiver (NovAtel Flexpak6), IMU (Xsens, about 400 Hz) and base network (HKSC).
  No run overlaps PPC or UrbanNav Tokyo.
- **Data access:** the data has no stated licence and is not redistributed. The
  repository records SHA256 of each raw and converted file, not the files.

## What was looked at before this freeze

All of this was done on the raw or converted files, never with an estimator,
and nothing compares an estimate to the truth.

- **Headers, rates and gaps** of the rover, base and IMU files; constellations
  and signals; satellites per epoch; the base hour covering each run;
  whether every rover epoch has an exact base epoch.
- **Truth-vs-truth:**
  - UTC and GPS time columns (offset exactly 18 s).
  - Rates, gaps, quality counts, speeds, baseline lengths.
  - **The velocity frame.** The 24 proper signed axis permutations of the body
    velocity were rotated with roll, pitch and heading to ENU and compared to
    the velocity obtained by differentiating the truth positions.
- **IMU-vs-truth:**
  - **The IMU frame.** The 24 proper signed axis permutations of the 1 s mean
    accelerometer against the specific force implied by the truth (forward
    acceleration, speed x yaw rate, gravity with truth roll and pitch).
  - Gyro z against the truth heading rate, as a lag scan.
  - At-rest gravity, the roll and pitch implied by the accelerometer at rest,
    and the difference between the bag time and the sensor stamp (median 0.3 ms;
    the sensor stamp is used).
- **An earlier survey** (not part of this holdout) also downloaded and read at
  format level the Medium-Urban-1 and a tunnel run (GNSS coverage, truth
  header, Medium-Urban-1 IMU z axis and gyro-z lag), a 2019 and a 2020 pilot
  set and further HKSC hours. None of them is converted or used. They must not
  be added to this population later.
- **The pipeline smoke** defined below.

**Not run before this freeze:** no estimator of this repository
(`gnss_pva_replay` with any candidate, `gnss pva-evaluate`, `gnss solve`, the
examples) was run on this data, except the single control-only smoke below. No
accuracy metric was computed. `gnss_pva_metrics` was not run on this data.

### The frames, and a correction to the survey

An earlier format survey recorded the Xsens axes as "already FLU". It had only
checked the z axis (gravity and gyro z). The 24-permutation check shows they
are **not** FLU:

| Check | Run | Best convention | RMS best | RMS next | RMS worst |
|---|---|---|---:|---:|---:|
| Truth body velocity to FRD | Deep | forward = y, right = x, down = -z | 0.095 m/s | 0.245 | 11.76 |
| | Harsh | same | 0.153 m/s | 0.267 | 11.27 |
| Xsens accelerometer to FLU | Deep | forward = +y, left = -x, up = +z | 0.069 m/s^2 | 0.556 (identity) | 11.31 |
| | Harsh | same | 0.053 m/s^2 | 0.378 (identity) | 11.31 |

- **Truth body frame:** x right, y forward, z up. Differentiated-position
  ENU velocity versus the rotated body velocity: RMS E/N/U 0.054 / 0.050 /
  0.026 m/s (Deep) and 0.041 / 0.052 / 0.090 m/s (Harsh), on the converted rows.
- **Xsens frame:** x right, y forward, z up. The same orientation as the truth
  body frame. The slopes of raw y on the truth forward acceleration and of raw x
  on the left acceleration are +1.04 / -1.03 (Deep) and +1.07 / -1.05 (Harsh).
  Gyro y follows the truth roll rate (r = 0.82, 0.68) and gyro x the pitch
  rate (r = 0.52, 0.42). The converter writes FLU = (y, -x, z).
- **Heading versus course over ground:** median -1.35 deg (Deep) and
  -1.37 deg (Harsh), moving rows only. That is a constant offset of the truth
  heading to the direction of travel, not corrected.
- **Boresight:** at rest, the roll and pitch of the accelerometer differ from
  the truth by +1.8 / +0.5 deg (Deep) and +0.2 / -1.1 deg (Harsh), medians. They
  are not corrected. The lever arm and the boresight are the declared zero
  assumptions of the replay, shared by control and candidate.
- **Time:** the IMU stamp minus truth time that maximises the correlation of the
  1 s integrals of gyro z with the truth heading change is +0.01 s (Deep) and
  0.00 s (Harsh); every lag within 5e-4 of the best correlation lies in
  [-0.05, +0.07] s and [-0.04, +0.04] s. The correlation is 0.99999. This is
  information only. **No time offset is applied.** (UrbanNav Tokyo had about
  0.15 s.)

The two axis checks, the heading-versus-course offset and the lag scan are
reproduced by `scripts/analysis/check_urbannav_hk_frames.py --truth ... --imu ...`.
The boresight, at-rest gravity and stamp checks were ad hoc and are not in the
script.

## Conversion to the PPC run layout (fixed here, no tuning)

`scripts/convert_urbannav_hk_to_ppc_layout.py` writes, per run, a directory
`<root>/urbannav/<Run>_novatel/` holding the PPC file set, and a manifest
`<Run>_novatel.manifest.json` beside it with the SHA256 of every raw input and
output. Only the NovAtel rover is used.

1. **Time.** GPS time throughout. The converter checks for every truth row that
   `UTCTime - 315964800 + 18` equals the truth week and time of week exactly,
   and converts the IMU `header.stamp` (Unix UTC) with the same 18 s.
2. **Epoch rule.** A rover epoch is kept if and only if it has an exact truth
   row and an exact IMU grid sample (below). Nothing else decides. Deep: 1 of 1,493
   epochs is dropped (the last, 456880 s, which has a truth row but the IMU
   ends at 456879.06 s). Harsh: 1,038 of 3,307 are dropped, all outside the
   truth span.
3. **`rover.obs`:** the NovAtel RINEX, the header and the kept epoch records
   byte-preserved. Event records (epoch flag other than 0 or 1) are refused.
4. **`base.obs`:** the HKSC hourly file covering the run (Compact RINEX
   expanded with the `hatanaka` 2.8.1 package, which is lossless), header
   byte-preserved, and the epoch records from the first to the last kept rover
   epoch byte-preserved. Everything else is dropped:
   - Epochs before the first rover epoch: the replay reads every base epoch up
     to the first rover epoch before it starts and buffers at most 16. The Tokyo
     base files begin at the first rover epoch. Deep drops
     1,746 epochs before and 320 after; Harsh 888 and 400.
   - Event records: the closing "header information follows" block (flag 4)
     that every hourly file carries. The replay's RINEX reader treats one as the
     end of the file.
   - The header's `TIME OF FIRST/LAST OBS` is therefore stale and unused.
   One hourly file covers each run, so no concatenation is done (the converter
   concatenates several files only if given).
5. **`base.nav`:** the day's GN, RN, EN and CN files merged into one RINEX 3.02
   mixed file. A new version/type line, the original `PGM / RUN BY / DATE` line, a
   comment, the `IONOSPHERIC CORR` and `TIME SYSTEM CORR` lines of all four
   (the GPS Klobuchar parameters GPSA/GPSB are the ones the reader uses), the GPS
   `LEAP SECONDS` line, then the record bodies of G, R, E, C unchanged.
   Deep: 206 / 402 / 1,709 / 433 records; Harsh: 203 / 409 / 1,736 / 427
   (G / R / E / C). The `MM` file of the same directory is meteorological data,
   not navigation.
6. **`imu.csv`:** PPC header and units. The steps, in order:
   1. **Axes.** Xsens (x right, y forward, z up) to FLU: x -> y, y -> -x,
      z -> z, for the accelerometer and the gyro (evidence above).
   2. **Units.** Gyro rad/s -> deg/s. The accelerometer is m/s^2 already.
   3. **Resampling.** The raw samples are on no fixed grid, and the replay needs
      an IMU sample at each rover epoch within 1e-6 s. They are resampled by
      linear interpolation onto the grid t = k x 0.01 s (100 Hz) of GPS time of
      week, the rate of the PPC `imu.csv`.
      - A grid point is written when it lies after one raw sample and at or
        before the next, and those two samples are at most 0.1 s apart.
      - No extrapolation, and no time offset.
      - The raw rate is about 400 Hz, so the 100 Hz samples are point
        samples of the interpolated signal, not 4-sample means. This follows
        the Tokyo converter and is the same for control and candidate.
      - No raw pair is over 0.1 s in either run (largest step 9.4 and 12.3 ms),
        so no grid point is skipped.
7. **`reference.csv`:** PPC header names plus a trailing `Truth Quality (Q)`
   column, rows at kept rover epochs only.
   - `Latitude (deg)`, `Longitude (deg)`: exact decimal degrees from D M S
     (sign of the degree token), ten decimals. Height, roll, pitch and heading
     are the raw text.
   - `ECEF X/Y/Z`: WGS84 from latitude, longitude and ellipsoidal height. The
     datum of the truth is not documented; WGS84 is assumed.
   - `East/North/Up Velocity (m/s)`: the body velocity as FRD = (y, x, -z),
     rotated by the 3-2-1 attitude (heading clockwise from north) to NED, then
     ENU = (E, N, -D). The same rotation as `gnss_pva_metrics` uses for its
     scene labels, and verified above.
   - Truth rows without a kept rover epoch are dropped (Deep 47, Harsh 43), which
     the scorer's turn label feels only as a missing yaw-rate sample; no gate
     uses it.

**Offline checks of the converted directories** (no estimator, run by the
converter and stored in the manifest):

| Run | Rover epochs | Without truth row | Without IMU grid sample | Without exact base epoch | Reference rows without rover epoch |
|---|---:|---:|---:|---:|---:|
| `HKDeepUrban1_novatel` | 1,492 | 0 | 0 | 0 | 0 |
| `HKHarshUrban1_novatel` | 2,269 | 0 | 0 | 1 (185501 s) | 0 |

The rover is 1 Hz and the base is 1 Hz on the same integer seconds, so every
rover epoch except one has an exact base epoch; the replay's SPP fallback
covers the missing one, in control and candidate alike. (UrbanNav Tokyo had a
5 Hz rover on a 1 Hz base, where only every fifth epoch had one.)

## Replay configuration (fixed here)

`gnss_pva_replay` accepts the `urbannav/<run>` layout since contract v1, and
this layout works for a 1 Hz rover without a code change. No source file of the
replay or the processor is changed by this contract.

- **Lever arm.** (0, 0, 0) m in FLU, the declared assumption for UrbanNav. It
  is applied to Hong Kong too. No documented antenna-IMU lever arm exists for
  this data. Control and candidate use the same zero.
- **Everything else** is the configuration the replay already uses: the base
  position from the base RINEX header, RTK settings, initialization and time
  handling. The processor limits that matter here: 2 s rover gap, 0.1 s IMU gap,
  16 pending base epochs, 10,000 pending IMU samples.
- **PPC is unaffected.** The tokyo and nagoya branches are unchanged and PPC
  outputs stay bit-identical (gate 7). The replay binary built from the freeze
  tree has SHA256 `5cf2b2c60d17b24a25ed46d3d69b593fdf0481263043e2c8e6cc83fda928f5ad`,
  byte-identical to the binary of the v10 comparison.

## Population and comparison

- **Population:** 2 runs (`HKDeepUrban1_novatel`, `HKHarshUrban1_novatel`) x 3
  scenarios.
  - Normal.
  - GNSS outage, 60-70 s (`--start-s 60 --duration-s 10`).
  - IMU gap, 60-64 s (`--start-s 60 --duration-s 4`).
- That is 6 control replays (`--candidate none`, the current production default)
  and 6 candidate replays (`--candidate velocity_consistency_v9`), built from the
  same binary and interleaved on the same quiet host, at most 3 at once. Each
  replay is `gnss pva-evaluate --run-dir <root>/urbannav/<Run>_novatel
  --replay-binary <binary> --scenario <scenario> --candidate <candidate>`, a full
  run (`--max-epochs 0`).
- The comparison is one invocation of the existing comparator with the new gate
  set, over the two runs:

  ```bash
  python3 scripts/analysis/compare_online_pva.py --gate-set holdout_v2 \
      --baseline-dir <none>/normal --candidate-dir <cand>/normal \
      --baseline-scenario-dir <none>/scenarios --candidate-scenario-dir <cand>/scenarios \
      --runs HKDeepUrban1_novatel HKHarshUrban1_novatel --output-dir <new>
  ```

  The scenario directories are named `<Run>_novatel-gnss_outage` and
  `<Run>_novatel-imu_gap`. `--gate-set default` is the gate set of contracts
  v1-v10, unchanged and bit-identical in output. `holdout_v2` takes the contract
  and candidate defaults of this document.
- **Pooling.** For each run and each arm, the scored epochs of the three scenario
  replays are pooled (1,492 x 3 and 2,269 x 3 epochs). Statistics are computed on
  the pooled set, on two cohorts:
  - **All-output:** every epoch where the arm has the metric.
  - **Common-valid:** the epochs where both arms have the metric, matched by
    scenario and elapsed time.

## Acceptance (the frozen gates, applied per run)

Each run has 39 gates, 78 for this population. `x` is the control, `y` the
candidate, both pooled as above. A tolerance of 1e-9 is added to every ratio
gate.

| Gate | Statistic | Condition |
|---|---|---|
| **H1** primary, no worse | fused position and rotation: RMSE and P95, both cohorts (8 gates) | y <= 1.00 x |
| **H2** secondary non-inferiority | RTK position, RTK velocity, fused velocity: RMSE and P95, both cohorts (12 gates) | y <= 1.10 x |
| **H3** tail safety | fused position and rotation: P99, all-output cohort (2 gates) | y <= 1.25 x |
| **H4** coverage | RTK, fused, RTK velocity, fused velocity, attitude and heading availability, pooled (6 gates) | y >= x - 0.005 |
| **H5** timing | normal scenario: first fresh attitude, first heading latch. GNSS-outage and IMU-gap scenarios: GNSS-update, fresh-attitude and heading recovery (8 gates) | y <= x + 1.0 s |
| **H6** processor | processing P95 of each scenario replay (3 gates) | y <= 2 x |
| **H7** integrity | missing metrics, scenarios or replays, truth mismatches, input-hash mismatches, changed outputs | any one fails the population |
| **Gate 7** parity | candidate `none` is bit-identical to develop `c37979c3` in every deterministic CSV field on the 18 PPC runs | separate check, `scripts/analysis/check_pva_default_parity.py` |

Details of the rules:

- **Metrics.** RMSE, P95 and P99 are those of `gnss_pva_metrics.stats` (absolute
  values, linear interpolation between order statistics) on the pooled values.
  A cohort with no values for a metric is a missing metric and fails.
- **Rotation** is the full-rotation error and exists from the first heading latch.
  Coverage is counted over all emitted epochs.
- **H4 coverage** is pooled over the epochs of the three replays. Hiding a bad
  epoch cannot improve H1-H3 on the common-valid cohort, and costs coverage.
- **H5 nulls are censored, never zero.** A null (never recovered) for the
  candidate where the control is non-null fails. A null control passes. Both null
  passes. A control value of 0.0 s is a value. The comparison has 1e-6 s of
  tolerance.
- **H6** is a host-contention guard: a failure caused by a contended host is
  repeated unchanged on a quiet host and recorded as such; no input, binary or
  setting changes.
- **H7 details.** The comparator rejects the population as failed, with No-Go and
  no gate table, if:
  - a replay is missing, did not pass, is not a full run, or its `errors.csv`,
    `score.json` or native CSV hash differs from the manifest;
  - the two arms differ in raw input hashes, scenario, scenario window, epoch
    count, start time, base position, lever arm, navigation policy or emitted
    timestamps, or a match fraction is not 1;
  - the control is not candidate `none` or the candidate is not
    `velocity_consistency_v9`;
  - the scenario of a directory is not the one its name says, or the window is
    not 60 + 10 s (GNSS outage) or 60 + 4 s (IMU gap);
  - the three scenarios of one run are not on the same raw inputs;
  - not every replay records one and the same binary SHA256;
  - `errors.csv` disagrees with `score.json` on the count of any gated metric.

**Go for a population only if every gate passes on every run.** Otherwise record
No-Go with the failed gates, and keep the default. Do not change the candidate,
the conversion, the configuration, the comparator or any threshold after seeing
any holdout result.

## Decision and the later population

- This population is Hong Kong. The decision in this contract is **Go or No-Go for
  the Hong Kong population**.
- A **Meijo University / Chiba Institute of Technology Odaiba** dataset may be
  added later as a second population under the same gate set. It needs its own
  addendum document, committed and frozen before the first run of any estimator
  on that data, in the same way as this contract. The addendum fixes the data,
  the conversion (as a script with synthetic-fixture tests), the replay layout
  and lever arm, the run list and the smoke. It may not change H1-H7, gate 7, the
  candidate, the comparator or any threshold of this contract. The comparator
  already applies the gate set to any run list (`--runs`).
- **Proposal rule.** A PR proposing `velocity_consistency_v9` as the online PVA
  default requires **Go on Hong Kong and Go on that later population**. Go on
  Hong Kong alone does not permit it. No-Go on either keeps the default. A No-Go on
  Hong Kong ends the evaluation of this candidate: a later population is not run
  to compensate.
- **The data is consumed once scored.** After the first scored replay, Hong Kong is
  development data for any later candidate, exactly as Tokyo became after holdout
  v1. A new candidate needs a new contract and new data.

## Pipeline smoke allowed before the candidate run

- **Allowed run:** one bounded control-only replay on each converted directory,
  `--candidate none --max-epochs 300`, normal scenario.
- **What it checks:** that the conversion and the layout run: the replay state is
  `passed` and fused and RTK availability are nonzero.
- **What was read:** only the replay state, the fused and RTK availability
  (`fused_status > 0`, `rtk_status > 0`) and the match fraction (the share of
  emitted epochs whose exact time has a truth row, by timestamps only). The scorer
  was not run, no error metric was computed, and no estimate was compared with
  the truth.
- **If the smoke fails for a format reason,** the converter may be fixed to meet
  this contract's specification. The specification itself does not change. The fix
  and its reason are recorded.

### Change before the freeze: base trimmed to the rover span

The first smoke on the first draft of the converter failed in both directories:
`gnss_pva_replay: pending base capacity exceeded`, before any epoch was
processed. The draft copied the HKSC hourly file unchanged, which begins up to 29
minutes before the first rover epoch, and the replay buffers 16 base epochs at
most. The converter now keeps the base epochs from the first to the last kept
rover epoch (item 4 of the conversion) and drops event records. Commit
`919a5532`. A format reason only; no estimate was read.

### Final pre-freeze smoke

The frozen converter and a replay binary built from the freeze tree, 300 epochs,
normal scenario. Only the state, availability and match fraction were read.

| Directory | State | Fused availability | RTK availability | Match fraction |
|---|---|---:|---:|---:|
| `HKDeepUrban1_novatel` | passed | 0.977 | 0.940 | 1.0 |
| `HKHarshUrban1_novatel` | passed | 1.0 | 0.737 | 1.0 |

## Implementation check of the comparator on development data

The `holdout_v2` gate set was run on the existing development comparison
outputs (24 control and 24 candidate replays of `none` against
`velocity_consistency_v9`, the six PPC runs and UrbanNav Tokyo Odaiba and
Shinjuku). It is an **implementation check only**. The thresholds were fixed
above before it ran and were not changed after seeing it. It was also checked
that the default gate set reproduces the recorded decisions of the v10 results
exactly (PPC 0 of 558 failing, UrbanNav 2 of 186).

All 8 runs pass all 39 gates. Worst candidate/control ratio over the gates of
each hypothesis, and the worst deltas (H1-H3: 1.00 / 1.10 / 1.25 allowed):

| Run | Gates passed | H1 | H2 | H3 | H4 max loss (<= 0.005) | H5 max delta (<= 1.0 s) | H6 max ratio (<= 2) |
|---|---:|---:|---:|---:|---:|---:|---:|
| tokyo1 | 39/39 | 0.062 | 0.680 | 0.070 | 0 | 0.00 s | 1.89 |
| tokyo2 | 39/39 | 0.035 | 0.441 | 0.039 | 0 | 0.00 s | 1.43 |
| tokyo3 | 39/39 | 0.216 | 0.772 | 0.256 | 0 | 0.00 s | 1.22 |
| nagoya1 | 39/39 | 0.238 | 0.904 | 0.478 | 0 | 0.00 s | 1.23 |
| nagoya2 | 39/39 | 0.109 | 0.850 | 0.199 | 0 | 0.00 s | 1.61 |
| nagoya3 | 39/39 | 0.287 | 0.983 | 0.281 | 0 | 0.00 s | 1.30 |
| Odaiba (Trimble) | 39/39 | 0.997 | 0.998 | 1.000 | 0.0009 | 0.00 s | 1.18 |
| Shinjuku (Trimble) | 39/39 | 0.080 | 0.656 | 0.071 | -0.0003 | 0.00 s | 1.17 |

The binary of those replays is the one named above. The pooled RMSE and attitude
coverage of two runs were recomputed independently from the per-scenario
`score.json` files and agree to all printed digits. Decision file SHA256
`5ba4db7cff8178949c61bab883c8a6a43556b02a51d24a44c8aea9a928ed04ce`.

## Frozen files

| File | SHA256 |
|---|---|
| `scripts/analysis/compare_online_pva.py` | `65a0c628fc6c90173893f92181291f23a88cb1690809070ef2809f04552c9a93` |
| `scripts/convert_urbannav_hk_to_ppc_layout.py` | `9f480081df586461f81c7632e219ad2c3327e520a2d10d50e8a3531be7bb4f4b` |
| `scripts/analysis/check_urbannav_hk_frames.py` | `b5af3d656197fbdc40045ec06ff66ca05b4691053bf639138b106284e94df9c4` |
| `scripts/convert_urbannav_to_ppc_layout.py` (shared helpers, unchanged) | `70b81a91205a18b8d71405b42a0270f331cdcf595eaa95b3fb2b25288850bbed` |
| `apps/commands/benchmarks/gnss_pva_metrics.py` (scorer, unchanged) | `2d767c3d6e612e83ad61dd5f4f01d524a83470b73577a1b8817cd53076fa2957` |
| `apps/commands/benchmarks/gnss_pva_evaluate.py` (unchanged) | `1654796dc688edcf82567b17a96d8b08636fbcfde57e22b03e9c329871c0e94b` |
| `apps/native/gnss_pva_replay.cpp` (unchanged) | `4fcf913a932047fded8601bf87cec4244e8179c36f906f2bd54c9e011183f00e` |

## Data fingerprints

Raw inputs (survey download names; the zip archives contain the other sensors'
files too, of which only the NovAtel rover set is used):

| Run | File | Bytes | SHA256 |
|---|---|---:|---|
| Deep | `UrbanNav-HK-Deep-Urban-1.novatel.flexpak6.obs` (in `gnss.zip`, 48,954,448 B, `2ce5ffdc984c6a814faa2c2be6c470289e869457d89f9d7a67eeedef3b171efd`) | 2,277,793 | `2c772cc9f84a67c49c454ab35b091faaeadf1cb6805864648237b7db01a143ed` |
| Deep | truth text, saved as `gt_deep.txt` | 274,298 | `cb35bc9ebfb171147d290672a46b9d7426475501c922d84ae81edde92c3ba272` |
| Deep | `xsense_imu_deep_urban1.csv` | 214,803,817 | `40ade891c52bdc0a262a92eae6d4b1376a5cee7d0b485fb729a51248737228f0` |
| Deep | `HKSC00HKG_R_20211410600_01H_01S_MO.crx.gz` | 2,421,672 | `d72aaf02812b957f7c993e6f2dadfb3470ac84e05a6717df49e0ecae4c68e74c` |
| Deep | `HKSC00HKG_R_20211410000_01D_GN.rnx.gz` | 29,290 | `35da059024049aa73c1cfa1b52be71b7d29efaae36e71df40d5eade53e68c96b` |
| Deep | `HKSC00HKG_R_20211410000_01D_RN.rnx.gz` | 28,701 | `5edf87f01701240d13b05267839456d44a7098f1c8ed8e8a2f4e7fccfaf56f65` |
| Deep | `HKSC00HKG_R_20211410000_01D_EN.rnx.gz` | 121,556 | `811487f6a17b8952fa54cf45a4ef16eba8ceb6f15e1ed8c335fac75300a91241` |
| Deep | `HKSC00HKG_R_20211410000_01D_CN.rnx.gz` | 63,248 | `9e260d38f50ac455066fbf782e3ec12af901a952ca42a91d29ae2c28c12b72c8` |
| Harsh | `UrbanNav-HK-Harsh-Urban-1.novatel.flexpak6.obs` (in `gnss.zip`, 97,401,705 B, `e5d5ae3a16d47cf3c94f25d7694e91903bbd06f6855959ab0f60c97273ed18e4`) | 4,154,166 | `7a9a63e22f944ab098d923a4f6f3397cf8996906c812479204f644fdd7fd232b` |
| Harsh | truth text, saved as `gt_harsh.txt` | 411,890 | `a05edc4641623a3ddf5d04fe9f36262bf7ae76ce9c87953d5c7840790324a317` |
| Harsh | `xsense_imu_harsh_1.csv` | 472,829,946 | `06abcc84660299e86125bfd243d63b10dd70e1bc2ac9d6270f4d1464ae1bd528` |
| Harsh | `HKSC00HKG_R_20211380300_01H_01S_MO.crx.gz` | 2,020,548 | `310dbd6e5004b782358b6deba922de63e0b9cb59130d60c247a8c9d5e746d6f4` |
| Harsh | `HKSC00HKG_R_20211380000_01D_GN.rnx.gz` | 29,221 | `771bb2e1b5a0f4b5dfd35f231968cd5a0d7c9ec5e91d00b1e63e1a2e012f676b` |
| Harsh | `HKSC00HKG_R_20211380000_01D_RN.rnx.gz` | 28,984 | `1193350f0906629b7ab55564172fb7547a0b50d08f6632cdb7b866d3ed21c1b4` |
| Harsh | `HKSC00HKG_R_20211380000_01D_EN.rnx.gz` | 124,076 | `41014d67675a3763c1e5367b0d956103e726edf89228e87cdd56eaf4ec4e2b98` |
| Harsh | `HKSC00HKG_R_20211380000_01D_CN.rnx.gz` | 62,144 | `3fe545aff4651259850e8dd1da68f05f8c210a9afe750acb6d0529b297bc5f8a` |

The decompressed hourly base files have SHA256
`806df3fc5e7ae9e7e7191c3c4963984a1adc6cc258a3c0d75322eff96f6feb00` (Deep, 28,128,185 B)
and `6f5865879a7ad5f38ad3631a825c2cec1a6578ddd217d91c055d72b09b251a54` (Harsh,
22,743,578 B). The truth text files are the ground-truth files of the dataset
README (`UrbanNav_whampoa_raw.txt` for Deep-Urban-1 and
`UrbanNav_mongkok_GT_part_raw.txt` for Harsh-Urban-1); the mapping to the local
names rests on the survey's record and on the matching time spans, and the files
themselves are pinned by SHA256.

Converted files, from the converter at the freeze (the manifests also hold the
raw hashes and the converter hash):

| Directory | File | Bytes | SHA256 |
|---|---|---:|---|
| `HKDeepUrban1_novatel` | `rover.obs` | 2,274,676 | `f2aaad09de3c4304c5f601275a2b8641824ee19f59f979aeded210d04108b5a9` |
| | `base.obs` | 12,035,738 | `b8b705402526ef93bf3a21e95e5fcef2de78e0497f4e5610e3ee5f995c4e79fe` |
| | `base.nav` | 1,568,049 | `101abd626a7f73a9d0f599bfd89d7cb61d70005287d020b0d18a8b6a3258ce1c` |
| | `imu.csv` | 14,452,277 | `bc574a4ac16db30f0bf57c1608a00e8998874f799c18b4bca8794a2e95df35fe` |
| | `reference.csv` | 240,647 | `a00ac4f07929580344c797106808f824265fdfd230bfc52f998f7345b3191325` |
| | manifest | 5,624 | `903700138cc6d93199602b851d1f7e2f37a65e754b99e061b7906c72b09b3ad4` |
| `HKHarshUrban1_novatel` | `rover.obs` | 2,746,140 | `f8724beac4dec51bf305331b9399a228ba9b94f4e5d9c16847b03d8416b79082` |
| | `base.obs` | 14,433,422 | `38ff39dd67e525a36b41bb12dd5d3dbbc1d6b3f1865d5a6ba48c1906193ce4ae` |
| | `base.nav` | 1,580,728 | `8080e8cb21bb997665323d7595c03eb2b6d5f56ddb42f5921905975d4bfd3d17` |
| | `imu.csv` | 31,661,266 | `8ad2b2e42405da71dd775590efef993f0cb77bded29615644b7c524a5fc0ec6b` |
| | `reference.csv` | 364,589 | `6b6b250a1e7dd7b1c422f274c0f6aa2ae852e8258aa9812840a81df6799a1364` |
| | manifest | 5,671 | `c594a723e5096a41403ed7e9633cbf0a20c8961812a97e7a5f57260f0d345421` |

The converted `imu.csv` has 153,693 (Deep) and 336,761 (Harsh) rows.

## Known limitations of the data (stated before the run, not corrected)

These are untested by construction, since no estimator has been run, and they
affect control and candidate alike.

- **Lever arm and boresight:** declared zero. The truth reference point is not
  documented. The truth heading differs from the course by about -1.4 deg.
- **Datum:** the truth frame and the HKSC header position may differ by
  decimetres. The truth ECEF is computed as WGS84.
- **BeiDou B1I labelling:** the rover file labels B1I as `2I` (RINEX 3.02/3.03
  convention) and the base as `1I` (3.04), which the reader maps to B1C. BeiDou
  carrier phase can then pair between rover and base on B2I only. Observation
  codes are not rewritten.
- **GLONASS channels:** the rover header lists one `GLONASS SLOT / FRQ #` entry
  (R15 in Deep, R10 in Harsh); the base header lists all 24.
- **No Galileo at the rover.**
- **IMU resampling:** 400 Hz to 100 Hz by point-sampled linear interpolation.
- **Truth quality:** variable (Q 1 to 6). In deep urban canyons the truth itself
  is degraded.
- **Scorer:** the manifest of `gnss pva-evaluate` carries the fixed text
  `dataset_role: development/regression; no heldout claim`; it is the tool's
  constant and has no meaning for this contract.

## Reported in addition (not gates)

- Absolute errors by scenario, run and cohort, with the warning that the zero
  lever arm, the unknown truth reference point, the datum and the boresight bias
  them by up to about 1 m. The pooled statistics of H1-H4 are in the decision
  file for both arms.
- Errors split by the truth quality Q.
- The IMU-versus-truth time offset (about 0.00 s here) and the heading-versus-course
  offset (about -1.4 deg). Neither is corrected.
- Per-run behaviour of the candidate: direction-test flips, rover-gap RTK-only
  resets (the 7 and 8 rover gaps over 2 s), gyro-bias seeds and the processor
  counters of `replay.json`.
- Per-scenario values of every H1-H3 statistic, for comparison with the pooled
  ones.
- Processor P95 ratios and the host state.
