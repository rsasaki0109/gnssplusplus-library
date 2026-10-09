# Frozen velocity-consistency candidate v6 (candidate name `velocity_consistency_v5`)

Freeze this contract before running the six-run comparison. Document numbering
follows the contract revision (sixth contract); the candidate it defines is
named `velocity_consistency_v5`. All earlier candidates
(`vehicle_nhc_latched_v1`, `velocity_consistency_v1` to `_v4`) and their records
are unchanged and stay selectable. Development data only; no application
holdout is reopened. The production default stays unchanged even if this
candidate passes.

Control: `develop` default configuration, candidate `none`, built from the
candidate tree and run on the same host in the same interleaved queue as the
candidate (the quiet-host procedure of
[online_pva_candidate_v5_results.md](online_pva_candidate_v5_results.md)).
Gate 7 checks that candidate `none` from the candidate tree is unchanged: every
deterministic CSV field is bit-identical to candidate `none` built from the
develop commit this branch starts from, on all 18 runs.

## What this candidate answers

`velocity_consistency_v4` passes every gate, but Nagoya 1 keeps a full-rotation
RMSE of 31.1 deg in the normal run, 45.8 deg in the GNSS-outage run and 14.1 deg in the IMU-gap run. The other runs are at 1.7-2.9 deg in all three scenarios, except Tokyo 2 IMU gap at 18.4 deg (not addressed
here). This candidate answers only the Nagoya 1 attitude failure.

## Diagnosis (v4 replays of the six normal runs, offline truth used for scoring only)

1. **Reverse start, no direction test at the heading latch.** Truth
   (`reference.csv`) shows Nagoya 1 reversing from 47.6 to 56.4 s. The
   vehicle faces 178.7 deg, with forward speed down to -1.18 m/s and 7.1 m of
   travel. `HeadingAlignmentTracker` latches at 52.0 s once three RTK
   velocities of at least 1.0 m/s agree. `tryAlignHeading` then sets the body
   +X axis to the GNSS course over ground, -2.8 deg, which is the direction of
   travel. That is 178.4 deg from the true heading. Nothing tests whether the
   vehicle moves forward or backward. The error decays only through ordinary
   position and velocity updates: 178 deg at 52 s, -160 at 80 s, -109 at
   100 s, -36 at 120 s, -4 at 132 s.
   - `heading_converged` stays true for 94 % of 60-120 s.
   - The gyro z bias absorbs the error: about +0.024 rad/s at 120-200 s
     against roughly -0.003 rad/s at rest. This makes the heading drift
     again during the stop at 160-188 s.
   - Rotation RMSE in 60 s bins is 156 / 128 / 23 / 52 / 31 / 13 deg for the
     first six minutes, then 0.8-9 deg. The 52-120 s interval alone carries
     83 % of the run's squared rotation error, and 52-227 s carries 94 %.
2. **Rover gap re-initializes the inertial filters while moving.**
   - The rover observation file has a 10 s hole, 217.2-227.2 s, while the
     vehicle runs at 12.1 m/s. The IMU stream is continuous (largest
     dt 0.01 s).
   - `OnlineRtkImuProcessor` treats a rover gap > `max_rover_gap_s` (2 s) like
     an IMU gap and recreates every filter. The new loose filter performs a
     *static* alignment on the first 2 s of the IMU backlog: velocity := 0 at
     12 m/s, biases := window means. It then re-latches the heading 1.2 s
     later mid-turn.
   - Rotation RMSE over 227.2-345 s is 25.3 deg, peaking at 55 deg. That is
     about 5 % of the run's squared error.
   - This is the only rover-gap reset in the twelve normal replays
     (none and v4). Tokyo 1 has a 2.0 s gap, which is not > 2.0 s.
3. **Attribution by counterfactual.** These are diagnostic scratch builds,
   not candidates. Values are heading-error RMS over the whole run.
   - A higher latch speed that avoids the reversing segment: 31.1 -> 7.8 deg.
   - That plus no rover-gap reset: 1.9 deg.
   - Removing the rover-gap reset alone makes the run worse (105 deg). The
     wrong pre-reset heading then survives, whereas the reset accidentally
     replaced it.
   - So the two changes must be evaluated together.

## Design (two changes, one candidate; no new constants)

**(h) Direction test at the heading latch.**
`LooseCouplingProcessor::Config::heading_latch_direction_test`, default false.
When false, the code is bit-identical to v4.

When true, the filter keeps a signed longitudinal body velocity `v_long`,
integrated from the IMU. It also keeps a flag `v_long_valid`.

- **Each propagated IMU sample:**
  `v_long += a_x * dt`, where
  `a_x = [ (accel_raw - accel_bias) + R_bn^T * (0, 0, -g) ]_x`. This is the
  body-frame forward kinematic acceleration. It depends only on roll and pitch,
  which the static alignment observes, not on the unknown yaw. The term
  `(omega x v)_x = omega_y v_z - omega_z v_y` is neglected, because a road
  vehicle's lateral and vertical body velocity is about 0.
- **Stationary reset:** whenever the existing ZUPT stationarity condition
  holds on a sample, `v_long := 0` and `v_long_valid := true`. The condition is
  `gnssSpeedGateAllowsZupt() && detectStationary()`, with the same thresholds
  and window as ZUPT. It is evaluated whether or not `zupt_enable` is set.
- **Invalidation:** `v_long_valid` is false after (re)initialization until the
  condition first holds. The static-alignment window is not evidence of rest
  after a reset that happens while moving.
- **At the latch:** when `HeadingAlignmentTracker` is ready and
  `v_long_valid` holds, `v_long < 0` means the vehicle moves backward. The
  latched heading is then the tracker's mean course + 180 deg. Otherwise
  (`v_long >= 0` or not valid), the latch is unchanged.

The test is a sign test between two hypotheses, forward (`v_long = +speed`)
and backward (`v_long = -speed`). Their likelihoods are symmetric, so the
decision boundary is 0 and there is no margin constant.

- The latch time is unchanged, so gate 3 (first heading latch no later) is
  not affected.
- Velocity re-anchoring on the latch and the latched sigma are unchanged.
- The latch is the only point where (h) acts. Later epochs and later
  generations' latches use the same rule.

Error budget at the Nagoya 1 latch: about 4.4 s of integration since the last
stationary sample. An accel-bias error of 0.01 m/s^2 gives 0.04 m/s, and a
0.5 deg pitch error gives 0.09 m/s x 4.4 s = 0.38 m/s. Both are small against
the 1.1 m/s speed to be discriminated. Over long moving intervals the
integrator drifts. It is used only at a latch, and a latch needs at least
1.0 m/s.

**(i) A rover-only gap keeps the inertial filters.**
`OnlineRtkImuProcessor::Config::rover_gap_keeps_inertial_filters`, default
false. When false, the code is bit-identical to v4.

When true, a rover gap > `max_rover_gap_s` recreates the RTK filter, the
tight-coupling filter and the isolated RTK-prior filter `prior_fusion_`
exactly as before. Only the loose fused filter `fusion_` keeps its state.
The RTK side therefore behaves exactly as in v4: (h) is not applied to the
prior filter, and (i) recreates it as before. As a result, the RTK columns
are expected to be bit-identical to v4. The
existing IMU-gap checks still recreate everything when the IMU itself has a
gap > `max_imu_gap_s` or is stale. They run after this check in
`processRover` and are unchanged.

- Such a reset is reported as reason `rover_gap_rtk_reset` with its own
  diagnostics counter.
- `reset_generation` is not incremented, because the fused filter instance
  and its state continue.
- After the gap, the existing v4 rules handle recovery: the post-gap
  position re-anchor (1.0 s horizon) and the v2 consecutive-rejection
  re-anchors.

Why it is sound: the gap is GNSS-only, and the INS mechanization stayed
continuous through it. Re-initializing turned 10 s of valid inertial
propagation into a false static alignment at 12 m/s. Keeping the filter costs
at most a stale position, which v4's post-gap re-anchor already treats as an
unverified prior.

Rejected alternatives:
- **A higher latch speed** (the counterfactual). It delays the first latch in
  every run, which fails gate 3, and it does not detect reversing at speed.
- **Deferring the latch until forward motion** also fails gate 3, on Nagoya 1.
- **A running yaw-consistency monitor with a 180 deg alternative hypothesis.**
  It is costlier (a second filter), has no information at constant cruise,
  and earlier heading-recovery attempts thrashed on this run (comments in
  `fusion_processor.cpp`, `heading_recovery_*`).
- **(i) alone** makes Nagoya 1 worse (counterfactual above).

## Candidate `velocity_consistency_v5` (one frozen change set, no tuning)

* everything in `velocity_consistency_v4`
  ([contract](online_pva_candidate_v5.md)), unchanged;
* (h) `heading_latch_direction_test = true` on the fused loose filter (not on
  the isolated RTK-prior filter, whose output never reaches the fused
  columns);
* (i) `rover_gap_keeps_inertial_filters = true`.

CLI: `gnss pva-evaluate|gnss_pva_replay --candidate velocity_consistency_v5`.

- No change to sensor axes, lever arms, time, initialization window, truth
  alignment, RTK settings or any threshold.
- Candidate `none` and `velocity_consistency_v1` to `_v4` code paths are
  unchanged. Both options default to false; unit tests cover the OFF defaults
  and both trigger paths.

Expected consequences, stated before the comparison:

- Nagoya 1 latches at 52.0 s with a heading near 177 deg instead of 357 deg.
- The 227.2 s rover gap no longer re-initializes the fused filter.
- The other five runs are expected unchanged, if `v_long >= 0` at their first
  latch and no rover gap > 2 s occurs. An offline estimate of the IMU forward
  acceleration sign in the 4 s before the first latch gives:

  | Run | ax minus rest, m/s^2 | GNSS speed slope, m/s^2 |
  |---|---:|---:|
  | Tokyo 1 | +0.38 | +0.34 |
  | Tokyo 2 | +0.39 | +0.32 |
  | Tokyo 3 | +0.41 | +0.32 |
  | Nagoya 1 | -0.23 | +0.13 |
  | Nagoya 2 | +0.19 | +0.16 |
  | Nagoya 3 | +0.39 | +0.35 |

- In the GNSS-outage and IMU-gap scenarios, the IMU-gap reset at 60 s still
  recreates every filter, as designed. For Nagoya 1 IMU gap this removes the
  accidental cure of the reverse-start error in v4. Whether the re-latch after
  that moving reset is tested depends on whether the vehicle stops first.

## Disclosure of development use

All six runs were used before this freeze:

- the v4 normal and scenario replays (quiet-host set of the v5 results) for
  the diagnosis and attribution above;
- diagnostic scratch binaries of Nagoya 1 only, for the counterfactuals: no
  rover-gap reset, latch speed 2.5 m/s, and both;
- the offline sign table above, from `imu.csv` and RTK speeds, for all six
  runs.

Before the freeze, the candidate implementation is run only through unit tests
and bounded Nagoya 1 prefixes:

- 400 epochs, to confirm (h) fires at the first latch;
- 1,200 epochs, to confirm the (i) path is taken at 227.2 s.

No full replay of the candidate is run before the freeze. (h) and (i)
introduce no constant: the stationarity test is the existing ZUPT condition,
and the decision boundary is the sign.

## Acceptance (unchanged from `online_pva_candidate_v5.md`, no relaxation)

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
4. Processor P95 <= 2 x control, same host and same interleaved queue.
5. At least one normal-run full-rotation RMSE/P95 improvement.
6. Missing metrics, scenarios, truth mismatches or input-hash mismatches fail.
7. Candidate `none` built from the candidate tree is bit-identical to
   candidate `none` from the base develop commit in every deterministic CSV
   field (all but `processing_ms`) on all 18 runs.

Go only if every gate passes. Otherwise record No-Go with the failed gates,
keep the defaults, and do not tune this candidate after seeing the results.

Additionally reported (not gates):
- v4 -> v5 per run and scenario;
- Nagoya 1 rotation error per 60 s bin;
- the `v_long` sign at every latch in every replay;
- every `rover_gap_rtk_reset`.
