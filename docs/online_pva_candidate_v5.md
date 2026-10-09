# Frozen velocity-consistency candidate v5 (candidate name `velocity_consistency_v4`)

Freeze this contract before running the six-run comparison. Document numbering
follows the contract revision (fifth contract); the candidate it defines is
named `velocity_consistency_v4`. `vehicle_nhc_latched_v1`,
`velocity_consistency_v1` ([contract](online_pva_candidate_v2.md)),
`velocity_consistency_v2` ([contract](online_pva_candidate_v3.md)) and
`velocity_consistency_v3` ([contract](online_pva_candidate_v4.md)) and their
records are unchanged and stay selectable. Development data only; no
application holdout is reopened. The production default stays unchanged even
if this candidate passes.

Control: `develop` default configuration, candidate `none`. The recorded control
is develop `c2eb06f7` (`pva_velcons_20261009/ctl`). The recorded set is accepted
as the control only if candidate `none` built from the candidate tree is
bit-identical to it in every deterministic CSV field on all 18 runs (gate 7).
The same check on the merge of develop `526cccaf` (PR #565-#567) into this
branch (`68c7faa3`) was 18/18 identical before this candidate was written (see
the results record).

## What this candidate answers

`velocity_consistency_v3` failed 4 of 558 gates, all
`scenario.recovery_gnss_update_s` after the 10 s GNSS outage (Tokyo 2, Tokyo 3,
Nagoya 2: 0 -> 0.2 s; Nagoya 1: 0 -> 18 s). Everything else passed. This
candidate changes only what is needed to answer that one failure.

## Diagnosis (instrumented replay of v3, outage 60-70 s, six runs)

The first epoch after the outage (t = 70.0 s) is an RTK FLOAT epoch. The
fusion's position NIS gate (9 per observation) sees: innovation = RTK FLOAT
minus propagated INS position, innovation covariance = INS position covariance +
reported FLOAT covariance. ENU errors against `reference.csv` (offline scoring,
never an estimator input); z = error / reported sigma:

| Run | INS error E/N/U m | INS sigma m | INS z | FLOAT error E/N/U m | FLOAT sigma m | FLOAT z | NIS/obs | v3 |
|---|---|---|---|---|---|---|---:|---|
| Tokyo 1 | -2.8 / 0.3 / 0.9 | 1.02 / 0.97 / 0.35 | -2.8 / 0.3 / 2.6 | 0.08 / 0.08 / -0.18 | 0.037 / 0.032 / 0.099 | 2.2 / 2.6 / -1.8 | 6.8 | accepted |
| Tokyo 2 | 5.6 / -4.3 / 1.5 | 0.98 / 0.76 / 0.31 | 5.7 / -5.7 / 4.8 | -0.70 / 0.08 / 11.45 | 0.055 / 0.019 / 0.591 | -12.6 / 4.4 / 19.4 | 88.9 | rejected |
| Tokyo 3 | 1.4 / -1.9 / 9.3 | 1.40 / 1.50 / 1.64 | 1.0 / -1.3 / 5.7 | -0.01 / 0.04 / -0.17 | 0.075 / 0.034 / 0.128 | -0.1 / 1.2 / -1.3 | 11.7 | rejected |
| Nagoya 1 | 32.9 / -14.2 / 0.8 | 1.04 / 1.22 / 0.47 | 31.6 / -11.6 / 1.8 | 0.04 / 0.23 / 0.23 | 0.038 / 0.045 / 0.102 | 1.1 / 5.1 / 2.3 | 387.6 | rejected |
| Nagoya 2 | -1.9 / 10.6 / 0.3 | 1.14 / 0.65 / 0.22 | -1.7 / 16.3 / 1.3 | 0.03 / -0.04 / -0.06 | 0.005 / 0.006 / 0.011 | 7.3 / -6.1 / -5.9 | 100.0 | rejected |
| Nagoya 3 | -0.7 / -3.4 / 1.2 | 1.30 / 1.69 / 0.85 | -0.5 / -2.0 / 1.5 | -0.60 / 0.16 / 1.61 | 0.79 / 1.35 / 13.3 | -0.8 / 0.1 / 0.1 | 1.1 | accepted |

(INS error is the fused position at the last outage epoch for the accepted
runs and at 70.0 for the rejected ones.)

1. The INS covariance is the dominant cause. In all four rejected runs the
   propagated INS position is 4.8 to 31.6 sigma from the truth (10 s unaided;
   the 15-state covariance carries only IMU noise, bias random walk and the
   attitude it believes: sigma about 1 m, attitude sigma 1-4 deg, while the
   unmodelled terms that actually drive the drift - attitude/heading error
   already present at the outage start, the roughly 150 deg heading error of
   Nagoya 1's reverse start, and the SPP-biased vertical state of Tokyo 3 - are not in
   it). The FLOAT itself is sub-decimetre in all but Tokyo 2 (z within 3 in
   Tokyo 3, 1.1-5.1 in Nagoya 1).
2. The FLOAT reported covariance is also optimistic where the RTK filter has
   not reconverged: Tokyo 2's first post-outage FLOAT is 19 sigma off in height
   (11.45 m, equal to the SPP vertical bias of this data set; the reported
   sigma is 0.59 m) and Nagoya 2's is 6-7 sigma (millimetre sigma). Making the
   FLOAT sigma honest cannot remove the rejections: Nagoya 1 (z 31.6) and
   Nagoya 2 (16.3) would still fail on the INS side.
3. Process noise cannot be the remedy. To pass the gate at 70.0 the INS
   position variance would need to be larger by NIS/9 = 1.3 (Tokyo 3), 9.9
   (Tokyo 2), 11 (Nagoya 2) and 43 (Nagoya 1) after 10 s, but only after a
   gap: the same inflation at the 0.2 s fusion cadence would make the INS
   prior meaningless in normal operation. The error is not a time-uniform
   noise deficit but a covariance that is unverified when GNSS has been absent.
4. Nagoya 1's 18 s: from 70.0 every precise (FLOAT) update is rejected (INS 35 m
   away, sigma 1.2 m), SPP updates are rejected too (NIS 47), and the
   FLOAT/FIXED re-anchor of v2 needs 30 consecutive FLOAT rejections.

## Design: trust the measurement over an unverified prior (one change)

New default-OFF `LooseCouplingProcessor::Config::position_reanchor_after_gnss_gap_s`
(0 = off, bit-identical). When it is `> 0` and a FLOAT/FIXED position update is
rejected by the NIS gate (or its innovation covariance is invalid), and the last
accepted GNSS position update of any class (SPP, FLOAT, FIXED or a re-anchor)
is older than this horizon, the position-only re-anchor that v2 already uses
(`reanchorPositionFromFixedSolution`) is applied at once: position := the
lever-arm-compensated GNSS antenna position, position covariance := GNSS
covariance + lever-arm attitude term, position cross-covariances cleared;
velocity, attitude and biases untouched. The clock restarts, so an immediately
following rejection is steady state again.

Why this is covariance-consistent: while GNSS keeps being accepted, every
update verifies the prior's covariance (a rejection then means the measurement
is the outlier, and the NIS gate stays in force exactly as before). After a gap
longer than the horizon nothing has verified the prior since, its covariance
is an unchecked extrapolation, and the test statistic of a gate built on it has
lost its meaning; a rejection is then evidence against the prior. The
measurement it is preferred to carries a reported covariance that
`ReportedCovarianceMode::SPP_CONSISTENCY_SCALED` (candidate v3, e) has made
consistent with the same epoch's SPP, and the re-anchor adopts exactly that
covariance, so the state is as uncertain as the measurement says (Tokyo 2's
height is taken with sigma 0.59 m, not with the prior's). It is not an
unconditional ungated update: it needs a NIS rejection, only applies to the
precise class, and respects `max_fixed_position_reanchor_m`.

Rejected alternatives, with reasons from the table above: (i) inflate the FLOAT
covariance - cannot fix Nagoya 1/2, the INS side fails; (ii) larger process
noise - needs x1.3 to x43 after 10 s only (point 3); (iii) scale the whole
prior by NIS/9 at a rejection (covariance matching) - turns the gate into a
robust update that no longer rejects: with sigma_prior 1 m, sigma_FLOAT 0.1 m
and a 100 m outlier the gain is 1/(1 + 11) and the state is pulled 8 m toward
the outlier; the re-anchor takes the measurement's own covariance and is only
used after a gap; (iv) lower the number of
consecutive rejections of v2 - a pure count, and it would defeat the gate in the
steady-state wrong-FLOAT regime v3 depends on (Tokyo 1, 57-66 s).

Constant: horizon = `float_reanchor_max_coarse_age_s` = 1.0 s, the repository's
existing "GNSS position is no longer current" horizon (v2), not a new value
and not tuned on this problem. Data support: in the six normal runs accepted
position updates arrive every 0.2 s, 0.4-1.0 s apart 65-198 times per run
(rejected epochs), and more than 1.0 s apart 1-7 times per run (Tokyo 1 7, Tokyo 2 2, Tokyo 3 1, Nagoya 1 6, Nagoya 2
7, Nagoya 3 1, including the real outages of up to 15, 34 and 94 s), so the
rule can fire only at those few places, and only if the first precise update
after them is NIS-rejected. The 10 s outage is 10x the horizon and 50x the
update interval, so the exact horizon does not decide the outage scenarios; it
matters only for dropouts of 1-10 s in normal runs.

Not covered, by design: a rejected SPP first update after a gap (the coarse
class is never re-anchored), and a gap closed by an accepted update.

## Candidate `velocity_consistency_v4` (one frozen change set, no tuning)

* everything in `velocity_consistency_v3` ([contract](online_pva_candidate_v4.md):
  RTK reported covariance mode 3, isolated RTK prior, NIS gates 9, FLOAT/FIXED
  re-anchor after 30 rejections), unchanged;
* (g) `position_reanchor_after_gnss_gap_s = float_reanchor_max_coarse_age_s`
  (1.0 s).

CLI: `gnss pva-evaluate|gnss_pva_replay --candidate velocity_consistency_v4`.
No change to sensor axes, lever arms, time, initialization window, truth
alignment, RTK settings or any other threshold. Candidate `none` and
`velocity_consistency_v1/v2/v3` code paths are unchanged (the new branch is
disabled when the option is 0; unit tests cover the OFF default and the trigger
logic).

Expected consequence, stated before the comparison: in Tokyo 2 the re-anchor
adopts the 11.45 m height error of a just-reconverging FLOAT whose sigma is
optimistic (point 2); the fused height then follows the reconverging FLOAT with
the same optimism as the control did at that epoch, so Tokyo 2's outage-run
fused position is expected to be worse than v3's and similar to the control's
for tens of seconds. The gates compare with the control.

## Disclosure of development use

All six runs were used before this freeze: the instrumented 500-epoch outage
replay of v3 for the diagnosis table (innovation, INS and reported covariance
at each position update; offline errors from `errors.csv`), and 600-epoch
GNSS-outage replays (60-70 s) of this candidate on all six runs to confirm that
the first post-outage update is applied (recovery 0.0 s on all six; Tokyo 2 as
expected above), and the full normal replays of this candidate on all six runs
(to check that it does not disturb normal operation): the fused output is
bit-identical to `velocity_consistency_v3` on Tokyo 1-3 and Nagoya 3 and
differs only after the first real GNSS dropouts of Nagoya 1 (954 s) and Nagoya 2
(1322 s); the RTK columns are untouched. The full outage and IMU-gap scenarios
of this candidate were not run before the freeze. The normal-run full replays of
v3 had been used earlier (v4 contract). No constant was chosen or changed after
seeing results: the horizon is the existing v2 constant; 3, 9, 30 and 1.0 s are
the earlier contracts' values.

## Acceptance (unchanged from `online_pva_candidate_v4.md`, no relaxation)

Population: six full normal runs, plus fixed 60-70 s GNSS removal and 60-64 s
IMU gap on each (18 control and 18 candidate replays), identical raw input
hashes and replay contract. Gates, per run/scenario unless stated:

1. RTK/fused position, RTK/fused velocity and full-rotation RMSE and P95 each
   <= 1.01 x control, on both the all-output and common-valid cohorts.
2. Coverage (RTK, fused, velocity, attitude, heading availability) loses at
   most 0.1 percentage point.
3. First fresh attitude and first heading latch no later; scenario GNSS update,
   fresh-attitude and heading recovery no later (null is censored, never zero).
4. Processor P95 <= 2 x control (host contention is reported, not hidden; a
   contemporaneous candidate-`none` run is reported alongside).
5. At least one normal-run full-rotation RMSE/P95 improvement.
6. Missing metrics, scenarios, truth mismatches or input-hash mismatches fail.
7. Candidate `none` built from the candidate tree is bit-identical to the
   control in every deterministic CSV field (all but `processing_ms`) on all 18
   runs.

Go only if every gate passes. Otherwise record No-Go with the failed gates, keep
the defaults, and do not tune this candidate after seeing the results.
Additionally reported (not gates): fused position error around the outage, and
per normal run the first epoch at which the candidate output differs from
`velocity_consistency_v3`.
