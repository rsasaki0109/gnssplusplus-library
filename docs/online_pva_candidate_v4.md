# Frozen velocity-consistency candidate v4 (candidate name `velocity_consistency_v3`)

Freeze this contract before running the six-run comparison. Document numbering
follows the contract revision (fourth contract); the candidate it defines is
named `velocity_consistency_v3`. `vehicle_nhc_latched_v1`,
`velocity_consistency_v1` ([contract](online_pva_candidate_v2.md)) and
`velocity_consistency_v2` ([contract](online_pva_candidate_v3.md), open PR) and
their records are unchanged. Development data only; no application holdout is
reopened. The production default stays unchanged even if this candidate passes.

Control: `develop` default configuration, candidate `none`. The recorded control
is develop `c2eb06f7` (`pva_velcons_20261009/ctl`); develop is now `526cccaf`.
The recorded set is accepted as the control only if candidate `none` built from
the candidate tree is bit-identical to it in every deterministic CSV field on
all 18 runs (gate 7).

## Root cause answered by this candidate

Earlier candidates worked around the fusion's rejection of the RTK FLOAT stream
(declared sigma 0.1 m, actual error 10-150 m). The upstream cause is in the
RTK processor, not in the fusion:

1. `RTKProcessor::generateSolution` (`src/algorithms/rtk.cpp:238` before this
   change) sets `solution.position_covariance = 0.01 * I` for every FLOAT and
   FIXED solution. It is a constant, not the filter covariance. It is also the
   only place a DD RTK covariance is produced (`rtk.cpp:389` is the safe-float
   continuity path).
2. Why FLOAT can be 10-150 m off while the filter looks healthy (Tokyo 1, 57 s
   onward, debug columns of the online replay): every measurement row is
   zeroed by the outlier suppression (`rtk_filter.cpp:458`, 30 m threshold;
   `rtk_update.cpp:256`). Suppressed rows = 2 x observations (two iterations),
   NIS 0.000, maximum prefit residual 25 m growing to 113 m, so the epoch is a
   measurement-free propagation of the INS prior. The prior is the tight
   predictor that is re-anchored from this very posterior every exact-base epoch
   (`online_rtk_imu.cpp:174,231`), a self-confirming loop; the position drifts at
   the wrong INS velocity (33 -> 125 m in 8 s) and is still emitted as FLOAT
   because the only divergence gates compare with SPP at 150 m
   (`rtk_epoch.cpp:360`).
3. A second, smaller effect: each epoch runs up to two measurement iterations
   that re-apply the same rows to the already updated covariance
   (`rtk_epoch.cpp:308-313`), so the Kalman marginal is over-confident by up to
   the iteration count even when nothing else is wrong.
4. Remaining after both are handled (not addressed): in converged epochs the
   filter marginal (cm-level) is still 3-6x too small versus decimetre errors
   (time-correlated code multipath is modelled as white). Fixing that needs a
   measurement-model change inside the RTK filter and is out of scope.

Measured on the six online replays (control, FLOAT epochs only; error versus the
offline reference, ENU z = error / reported sigma): 284-908 FLOAT epochs per
run, reported sigma 0.14 m horizontal, RMS z 35-547 per axis, 8-61 % of epochs
inside 3 sigma per axis, NEES within the chi-square 99.73 % bound for 7-30 %.
SPP epochs are consistent (RMS z 0.3-2.3) and FIXED epochs are consistent.

## Design: report an honest covariance, do not touch the filter

New default-OFF `RTKConfig::reported_covariance_mode`
(`LEGACY_FIXED_SIGMA` = 0, bit-identical). Only
`PositionSolution::position_covariance` of FLOAT epochs changes; no state,
position, ambiguity or status is modified, so the batch RTK `.pos` stream is
independent of the mode. Mode `SPP_CONSISTENCY_SCALED` (3) reports

  C = s * v * P1,

* `P1`: Kalman position marginal after the first measurement iteration (a
  single application of the epoch's rows),
* `v = max(1, NIS_per_observation)` of that pass (innovation variance factor),
* `s >= 1`: smallest factor for which the FLOAT-minus-SPP difference `d` is
  consistent with its own covariance, `d' (s v P1 + C_spp)^-1 d <= 3` (the
  expected value of a 3-dof statistic; `src/algorithms/rtk_covariance_consistency.cpp`).
  The SPP solution is the one the epoch already computes for its gates
  (`rtk_epoch.cpp:158`); no extra SPP run. No evidence (no SPP, non-finite)
  gives `s = 1`. FIXED is unchanged.

Modes 1 (last-iteration marginal) and 2 (`P1 * v`, no SPP check) exist for
ablation and are not part of the candidate. CLI for the batch solver:
`gnss_solve --rtk-reported-covariance legacy|filter|first-pass|spp-consistency`,
`--rtk-covariance-log FILE`.

## Candidate `velocity_consistency_v3` (one frozen change set, no tuning)

* (e) RTK reported covariance mode 3 (above).
* (f) `OnlineRtkImuProcessor::Config::rtk_prior_fusion`: the RTK filter's INS
  prior is bootstrapped from an isolated loose-coupling filter built from the
  control fusion configuration and fed the legacy RTK covariance
  (`rtk_reported_covariance_replaced` => `0.01 * I`). The fused output filter
  can then gate, re-anchor and use the honest covariance without changing the
  RTK filter's own output. This is the decoupling recommended by the v3 record
  ("keep the control's tight predictor for the RTK prior and run the corrected
  state as a separate output"); it also forgoes any benefit the RTK filter
  would get from a better fused state.
* (c) `max_position_update_nis_per_observation = 9`,
  `max_velocity_update_nis_per_observation = 9` (as v1/v2).
* (d) `float_position_reanchor_after_rejections = 30`,
  `float_reanchor_max_coarse_age_s = 1.0` (as v2; its code is cherry-picked
  from `f91eb44b`).

Not included: (a) heading-latch velocity re-anchor and (b) independent Doppler
velocity. (b) changed the RTK filter's prior and caused the RTK-gate failures
of v1/v2. (a) is dropped on development evidence (below): with an honest
covariance it changes rotation by up to 2.2 deg, mostly for the worse, and does not
help position.

CLI: `gnss pva-evaluate|gnss_pva_replay --candidate velocity_consistency_v3`.
No change to sensor axes, lever arms, time, initialization window, truth
alignment, RTK settings or any other threshold. Candidate `none` code paths are
unchanged (all new branches are disabled by default).

## Disclosure of development use

All six normal runs were used before this freeze, with an instrumented replay
(per-epoch RTK status, covariance, SPP scale, update diagnostics; offline
scoring against `reference.csv`, never fed to the estimator). Normal-run
ablations on the full runs (fused position/velocity/rotation and RTK columns),
all from this tree: covariance mode 0/1/2/3 alone; mode 3 + {c}, {a,c},
{a,b,c}, {a,b,c,d}; mode 1 + {a,c}; with the isolated RTK prior: mode 3 +
{c}, {a,c}, {c,d}, {a,c,d}. Selection from them: honest covariance is what
removes the attitude failure (rotation 105 -> 4-8 deg on three runs with
covariance alone); {c} makes it uniform (1.7-2.9 deg on five runs); {d} is kept
because without it Nagoya 1 fused position RMSE (54.6 m) is above the control
(49.3 m) while with it 17.7 m; {a} and {b} are dropped as above. The outage and
IMU-gap scenarios were not run on this candidate before the freeze. No constant
was changed after seeing results: 3 (statistic target), 9, 30 and 1.0 s are the
repository's or the earlier contracts' values.

## Acceptance (unchanged from `online_pva_candidate_v3.md`, no relaxation)

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

Additionally reported (not gates): covariance consistency of the RTK FLOAT
stream before and after (this document's measurement), and the batch RTK
(`gnss_solve`, PPC native-replay recipe) `.pos` output with the mode OFF and
mode 3 against the unmodified binary.
