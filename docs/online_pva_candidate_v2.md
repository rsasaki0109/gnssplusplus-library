# Frozen velocity-consistency candidate v2

Freeze this contract before running the six-run comparison. Control: `develop`
at `c2eb06f7` (includes the SPP cold-start fix, PR #562), default configuration,
candidate `none`, built Release from that source. Development data only; no
application holdout is reopened. `vehicle_nhc_latched_v1` and its records are
unchanged and remain a recorded No-Go.

## Diagnosis this candidate answers

Uncommitted env-gated experiments (pre-#562 build, SPP seed workaround) traced
the large attitude and velocity errors to three causes that this candidate
addresses and one it does not:

1. The GNSS-course heading latch rotates only the attitude. Velocity, its
   covariance and its cross-covariances were produced while yaw was arbitrary.
   The next velocity updates then have NIS/observation of about 5-19 and, with
   the NIS gates at their default of 0.0 (disabled), are absorbed into yaw and
   the z gyro bias (up to +10 deg/s). The velocity-gate-lockout re-anchor
   (`max_consecutive_velocity_gate_rejections = 3`) never fires because the
   gates are off.
2. In the default `tight_time_update` path the RTK filter carries a velocity
   state that is the tight INS prediction (`rtk_ins_time_update.cpp`). That
   state is returned as `out.rtk.velocity_ecef`, fed back into `tight_->reanchor`
   and into the loose filter: a self-confirming loop in which velocity diverged
   to 15-28 m/s. Re-anchoring with an independent Doppler velocity removed it.
3. (Fixed upstream by #562.) SPP cold-start lockout.
4. Not addressed: Nagoya run1 reverse start and `rover_gap_reset` alignment
   while moving; the gyro bias learned before the latch is not reset.

## Candidate `velocity_consistency_v1` (one frozen change set, no tuning)

All three components are opt-in and fixed; every default stays as on develop.

* (a) At the heading latch, if `LooseCouplingProcessor::Config::
  reanchor_velocity_on_heading_latch` is set, call the existing
  `reanchorVelocityFromGnssSolution()` with that epoch's GNSS antenna velocity.
  The velocity state becomes the GNSS velocity minus the lever-arm term using
  the NEW attitude, `P_vv` becomes the GNSS covariance plus the lever-arm
  attitude term, and all velocity cross-covariances are cleared. Chosen over
  rotating the velocity by the latch yaw delta because an ENU velocity is
  invariant to a yaw rotation; what is inconsistent is the velocity estimate
  and its correlations built under the wrong yaw, and the existing reset is
  the audited, bounded (20 m/s) operation for that. The same measurement
  already updated the state this epoch, so no information is double counted
  beyond the cleared correlations. If the reset is refused the state is left as is.
* (b) If `OnlineRtkImuProcessor::Config::independent_doppler_velocity` is set
  (and `tight_time_update` is on), the loose filter and `tight_->reanchor` use a
  Doppler least-squares velocity and covariance from
  `spp_velocity::solveVelocityFromObservations` at the RTK position, sigma = the
  RTK processor's Doppler sigma (0.5 m/s), never the RTK filter's own velocity
  state. If it cannot be solved the epoch carries no GNSS velocity (no tight
  re-anchor; the existing failed-anchor bootstrap applies). The exported
  `out.rtk` result is not modified.
* (c) `max_position_update_nis_per_observation = 9.0` and
  `max_velocity_update_nis_per_observation = 9.0`. Fixed from the diagnosis; a
  looser 25 lost the effect on two runs in diagnosis, which is recorded as
  brittleness, not tuned here.

CLI: `gnss pva-evaluate|gnss_pva_replay --candidate velocity_consistency_v1`.
No change to sensor axes, lever arms, time, initialization window, truth
alignment, RTK settings or any other threshold.

## Disclosure of development use

Before freezing, the implementation was run only for wiring on the first 1200
epochs (120 s) of Tokyo run1 (control vs candidate fused position RMSE
125.65 -> 1.14 m, rotation RMSE 102.5 -> 4.5 deg). No parameter was adjusted
after seeing it. The earlier diagnosis used all six runs on a different build.

## Acceptance (from `online_pva_development_plan.md`, same comparator as v1)

Population: six full normal runs, plus fixed 60-70 s GNSS removal and 60-64 s
IMU gap on each (18 control and 18 candidate replays), identical raw input
hashes and replay contract. Gates, per run/scenario unless stated:

1. RTK/fused position, RTK/fused velocity and full-rotation RMSE and P95 each
   <= 1.01 x control, on both the all-output and common-valid cohorts.
2. Coverage (RTK, fused, velocity, attitude, heading availability) loses at
   most 0.1 percentage point.
3. First fresh attitude and first heading latch no later; scenario GNSS update,
   fresh-attitude and heading recovery no later (null is censored, never zero).
4. Processor P95 <= 2 x control (host contention is reported, not hidden).
5. At least one normal-run full-rotation RMSE/P95 improvement.
6. Missing metrics, scenarios, truth mismatches or input-hash mismatches fail.
7. Candidate `none` built from the candidate tree is bit-identical to the
   control build in every deterministic CSV field (all but `processing_ms`) on
   all 18 runs. (The v1 pre-latch parity gate does not apply: this candidate
   acts before the latch.)

Go only if every gate passes. Otherwise record No-Go with the failed gates, keep
the defaults, and do not tune this candidate after seeing the results.
