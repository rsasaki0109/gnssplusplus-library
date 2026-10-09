# Frozen velocity-consistency candidate v7 (candidate name `velocity_consistency_v6`)

Freeze this contract before running the six-run comparison. Document numbering
follows the contract revision (seventh contract); the candidate it defines is
named `velocity_consistency_v6`. All earlier candidates
(`vehicle_nhc_latched_v1`, `velocity_consistency_v1` to `_v5`) and their records
are unchanged and stay selectable. Development data only; no application
holdout is reopened. The production default stays unchanged even if this
candidate passes.

Control: `develop` default configuration, candidate `none`. It is built from
the candidate tree and run on the same host, in the same interleaved queue as
the candidate. This is the quiet-host procedure of
[online_pva_candidate_v5_results.md](online_pva_candidate_v5_results.md).

Gate 7 checks that candidate `none` from the candidate tree is unchanged:
every deterministic CSV field is bit-identical to candidate `none` built from
develop `c37979c3`, on all 18 runs. The recorded
[v6 results](online_pva_candidate_v6_results.md) used that same reference.

## What this candidate answers

With `velocity_consistency_v5`, every run is at 1.7-3.7 deg rotation RMSE in
every scenario except one: Tokyo 2 IMU gap, at 18.4 deg RMSE and 55.6 deg P95.
That value is unchanged from v3/v4. This candidate answers only that failure.

## Diagnosis

The diagnosis uses v5 replays of the six IMU-gap scenarios. Truth is used for
offline scoring only.

1. **The IMU gap recreates the fused filter, and the new filter re-runs a
   static alignment while the vehicle moves.**
   - IMU samples are removed over 60-64 s.
   - `OnlineRtkImuProcessor` fires `imu_stale_reset` at 60.2 s and recreates
     every filter (`max_imu_gap_s` 0.1 s).
   - The new `LooseCouplingProcessor` initializes from the first
     `align_static_window_s` = 2 s of IMU after the gap, 64-66 s, with no
     stationarity test.
   - `alignStatic` sets gyro bias := window-mean gyro and accel bias := mean
     minus g along up, with velocity 0 (`fusion_initialization.cpp`).
2. **In Tokyo 2 the window is a turn.**
   - Truth yaw rate is +0.096 rad/s at 1.2 m/s.
   - The fused gyro z bias goes from -0.0114 rad/s, the pre-gap filter value,
     to +0.0875 rad/s. The true stationary bias is about -0.0004 rad/s.
   - The re-latch at 75.4 s is correct (3.1 deg at 76 s).
   - The 0.09 rad/s bias error then turns the heading at about 5 deg/s:
     65 deg at 86 s, 82 deg at 165 s, 37 deg at 230 s, 4.6 deg at 290 s.
   - 93 % of the scenario's squared rotation error lies in 104-300 s, and
     6 % in 66-104 s.
3. **Across the six runs, the size of the window-mean minus pre-gap gyro bias
   difference orders the post-gap rotation peaks.**

   | Run | |difference| (rad/s) |
   |---|---:|
   | Tokyo 2 | 0.099 |
   | Tokyo 3 | 0.034 |
   | Nagoya 1 | 0.034 |
   | Nagoya 2 | 0.018 |
   | Nagoya 3 | 0.008 |
   | Tokyo 1 | 0.005 |

   The pre-gap filter bias is closer to the stationary truth bias than the
   window mean in five of six runs. Nagoya 2 is the exception: 0.014 vs
   0.0045 rad/s.
4. **Counterfactuals.** These are scratch builds on Tokyo 2 IMU gap only;
   rotation RMSE / P95 in deg.

   | Variant | RMSE | P95 |
   |---|---:|---:|
   | Control (as v5) | 18.4 | 55.6 |
   | X1: gyro bias seeded with the pre-gap value, fused and RTK-prior filters | 5.2 | 3.0 |
   | X2: X1 + accel bias seeded | 5.5 | 3.0 |
   | X3: the loose filter is not reset across the gap | 8.9 | 23.4 |

   - The window-mean gyro bias is the driver.
   - Accel-bias seeding adds nothing.
   - Not resetting carries the pre-gap heading error and an unbridged 4 s
     turn.

## Design (one change, no new constants)

`LooseCouplingProcessor` gets `seedGyroBiasForNextInitialization(Vector3d)`.
A seed set this way replaces the window-mean gyro bias in the next
`initializeFromStaticWindow()`, right after `alignStatic`. It is then cleared.

- `alignStatic`'s attitude depends only on the mean specific force, and its
  accel bias only on the mean specific force, so neither depends on the gyro
  bias.
- Covariances are unchanged: the gyro-bias sigma stays
  `kInitialGyroBiasSigmaRadps`. The seed is not trusted more than a window
  mean.
- Nothing else in the filter changes.

`OnlineRtkImuProcessor::Config::carry_gyro_bias_across_reset` (bool, default
false). When it is false, the code is bit-identical to v5.

When it is true, every reset after construction that recreates the fused
filter `fusion_` takes the old filter's nominal gyro bias, if that filter was
initialized, and seeds the new `fusion_` with it. The resets that recreate
`fusion_` are `imu_gap_reset`, `imu_stale_reset`, and `rover_gap_reset` when
(i) is off.

- The RTK-prior filter `prior_fusion_` is recreated as before, without a seed,
  so the RTK columns stay as in v5.
- X1 seeded both filters, so its numbers are an indication, not a prediction.
- With (i) on, a rover-only gap does not recreate `fusion_`, so it is not
  affected.

Why it is sound:
- The IMU, its mounting and its power are continuous across a data gap in
  this replay and in a live stream. A seconds-long hole in the data does not
  change the sensor's bias.
- The pre-gap filter estimate is the product of the whole preceding run of
  updates. A 2 s window-mean has no way to tell a turn from a bias.
- Keeping the same covariance means the filter can correct the seed exactly as
  fast as it would have corrected a window mean.

Rejected alternatives:
- **(a) Use the window mean only when the window passes the ZUPT stationarity
  test.** By the same window statistics, it would still take the window mean
  in Nagoya 1 and Nagoya 2 sub-windows, and it needs a decision rule over
  sub-windows that is a new choice.
- **(b) Do not recreate the fused filter across short IMU gaps** (X3). This is
  worse on the one measured run, and it needs a new horizon constant.
- **(c) Also seed the accel bias** (X2). It gives no benefit.
- **(d) Seed the RTK-prior filter too.** It would change the RTK columns,
  which v3-v5 keep isolated from the fused output.

## Candidate `velocity_consistency_v6` (one frozen change set, no tuning)

* everything in `velocity_consistency_v5`
  ([contract](online_pva_candidate_v6.md)), unchanged;
* (j) `carry_gyro_bias_across_reset = true`.

CLI: `gnss pva-evaluate|gnss_pva_replay --candidate velocity_consistency_v6`.

- No change to sensor axes, lever arms, time, initialization window, truth
  alignment, RTK settings or any threshold.
- Candidate `none` and `velocity_consistency_v1` to `_v5` code paths are
  unchanged. The option defaults to false.
- Unit tests cover:
  - the OFF default;
  - the seed replacing the window mean, with attitude and accel bias
    unchanged;
  - no seed when the old filter was never initialized;
  - the RTK-prior filter not being seeded.

Expected consequences, stated before the comparison:

- The normal and GNSS-outage scenarios of all six runs are bit-identical to
  v5. They recreate no fused filter: Nagoya 1's rover gap keeps it under (i).
- The six IMU-gap scenarios change from the first post-gap initialization
  (66.0 s).
- Tokyo 2 IMU gap is expected to improve strongly.
- Tokyo 3 and Nagoya 1 IMU gap are expected to improve somewhat.
- Nagoya 2 IMU gap may be slightly worse than v5, because its window mean was
  closer to the true bias. Against the control (104.1 deg RMSE) it has a wide
  margin.

## Disclosure of development use

All six runs were used before this freeze:

- the v5 normal and scenario replays (quiet-host set of the v6 results) for
  the diagnosis;
- three scratch counterfactual builds on Tokyo 2 IMU gap only (X1-X3 above);
- offline window statistics for the six IMU-gap scenarios from `imu.csv`,
  truth and the v5 outputs.

Before the freeze, the candidate implementation is run only through unit tests
and one bounded prefix: Tokyo 2 IMU gap, 600 epochs, to confirm that the seed
is applied at the post-gap initialization. No full replay of the candidate is
run before the freeze. (j) introduces no constant.

What that prefix showed:

- The seed is applied at the post-gap initialization (tow 177066). Gyro bias
  z is -0.0114 rad/s (seed) instead of +0.0875 rad/s (window mean).
- Rotation error at 76 / 86 / 100 / 110 s:
  - v6: 1.3 / 13.9 / 23.4 / 33.8 deg
  - v5: 3.1 / 64.6 / 15.0 / 27.0 deg
  - v6 is worse than v5 at 100-110 s within the prefix. That matches X1's
    timeline, and nothing was changed in response.
- Candidates `none` and `velocity_consistency_v5` from the candidate tree
  match the v5-study outputs in every field except `processing_ms` on the same
  600-epoch prefix.

## Acceptance (unchanged from `online_pva_candidate_v6.md`, no relaxation)

Population: six full normal runs, plus fixed 60-70 s GNSS removal and 60-64 s
IMU gap on each. That is 18 control and 18 candidate replays, with identical
raw input hashes and replay contract. Gates, per run/scenario unless stated:

1. RTK/fused position, RTK/fused velocity and full-rotation RMSE and P95 each
   <= 1.01 x control, on both the all-output and common-valid cohorts.
2. Coverage (RTK, fused, velocity, attitude, heading availability) loses at
   most 0.1 percentage point.
3. First fresh attitude and first heading latch no later. Scenario GNSS
   update, fresh-attitude and heading recovery no later (null is censored,
   never zero).
4. Processor P95 <= 2 x control, same host, same interleaved queue.
5. At least one normal-run full-rotation RMSE/P95 improvement.
6. Missing metrics, scenarios, truth mismatches or input-hash mismatches fail.
7. Candidate `none` built from the candidate tree is bit-identical to
   candidate `none` from develop `c37979c3` in every deterministic CSV field
   (all but `processing_ms`) on all 18 runs.

Go only if every gate passes. Otherwise record No-Go with the failed gates,
keep the defaults, and do not tune this candidate after seeing the results.

Additionally reported, not as gates:
- v5 -> v6 per run and scenario;
- Tokyo 2 IMU-gap rotation timeline;
- the seeded and window-mean gyro bias at every seeded initialization.
