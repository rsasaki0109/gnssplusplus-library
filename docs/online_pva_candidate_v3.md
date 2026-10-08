# Frozen velocity-consistency candidate v3 (candidate name `velocity_consistency_v2`)

Freeze this contract before running the six-run comparison. Document numbering
follows the contract revision: this is the third contract; the candidate it
defines is named `velocity_consistency_v2` and is `velocity_consistency_v1`
plus one position-gate recovery. `velocity_consistency_v1`
([contract](online_pva_candidate_v2.md), [No-Go record](online_pva_candidate_v2_results.md))
and `vehicle_nhc_latched_v1` are unchanged. Development data only; no
application holdout is reopened. The production default stays unchanged, and
stays unchanged even if this candidate passes (a Go would only be reported).

Control: `develop` default configuration, candidate `none`, Release, replay
binary built from the committed candidate tree. The v1 control was recorded
from develop `c2eb06f7`; develop is now `526cccaf` (adds an unrelated,
default-OFF SPP barometer path). The control is accepted as that recorded set
only if candidate `none` built from this tree is bit-identical to it in every
deterministic CSV field on all 18 runs (gate 7); otherwise a control is
regenerated from the unmodified `526cccaf` and the difference is reported.

## Why v1 failed, by mechanism

All findings below come from a debug log of the v1 candidate (temporary
`LIBGNSS_DEBUG_POS` print of status, gate result, NIS per observation,
rejection counter, declared sigma and innovation at every GNSS position update
on all six normal runs; the print is not committed).

1. The position NIS gate (9 per observation) rejects every precise update. In
   all six runs the RTK FLOAT stream (declared sigma 0.1 m, 1-2 Hz) is rejected
   by the gate for the largest part of the run (one rejection streak that is
   still open at the end of the run in each run, lengths 626-1876 FLOAT epochs;
   FLOAT epochs accepted: 52 of 1398 on Nagoya 1). Only the coarse SPP updates
   (declared sigma 6.4 m, 5-10 Hz) pass, so the fused position is an
   SPP-driven solution with a 10-15 m bias.
2. Why the existing recovery cannot fire:
   `fusion_processor.cpp` counts consecutive rejections in one counter shared
   by every solution class. Any accepted SPP update resets it
   (`applyUpdateAndInject`, line 189), every non-FIXED epoch zeroes it
   (line 504-505), and the re-anchor needs a FIXED solution with 30
   consecutive rejections (line 528, `max_consecutive_gate_rejections = 30`).
   On Nagoya 1 the logged counter never exceeds 1, and only 13 epochs in
   1520 s are FIXED. The recovery is structurally unreachable on a
   FLOAT-dominated run.
3. Nagoya 1 then loses the coarse path too. From about 500 s the state drifts
   from the FLOAT solution (innovation 38 m growing to 148 m) while the
   fused velocity state is near zero (558-669 s) and then non-zero; at about 700-715 s
   the state moves beyond the SPP gate (innovation 120 m against a 6.4 m sigma:
   NIS per observation about 117) and from then on no position update of any
   class is accepted (accepted-update share 0.00 from 780 s). The offset stays
   at 120 m, then 170 m, to the end of the run.
4. The reported FLOAT can itself be wrong. On Nagoya 1, between about 495 s and
   735 s the exported RTK FLOAT solution, still declaring sigma 0.1 m, is 21,
   then 41 ... 150 m from the reference (it recovers to 0.5-3 m at 735 s). The
   same RTK output exists in the control. A recovery that trusts any rejected
   FLOAT therefore follows this drift: a first diagnostic version without a
   consistency check (re-anchor after 30 FLOAT rejections) reduced Nagoya 1
   fused position RMSE only 119.0 -> 59.6 m (control 49.3 m), with the fused
   state tracking the drifting FLOAT to 215 m.
5. Gates trade position against attitude (known from the v1 record): the
   lever-arm cross-covariance lets a large position innovation rotate the
   attitude. A re-anchor that clears only the position block and its
   cross-covariances does not.

RTK-filter regression (2-9% P95, N1 RTK position 25.7 -> 30.2 m): the
mechanism is component (b), not the gates. With `tight_time_update` the
fusion's tight predictor supplies the RTK filter's external position/velocity
time update (`online_rtk_imu.cpp:171-175`), and that predictor is re-anchored
each epoch from `gnss_input` (`:218`). Component (b) changes the velocity that
re-anchors it, so the RTK prior changes and the exported RTK result changes. The
quantity that moves is the epoch population: the number of epochs where the RTK
filter emits FLOAT instead of falling back to SPP rises (control -> v1 FLOAT
epochs: Tokyo 3 494 -> 2934, Tokyo 1 660 -> 2257, Tokyo 2 775 -> 1726, Nagoya 1
713 -> 1387, Nagoya 2 908 -> 1771, Nagoya 3 284 -> 921; SPP-fallback-epoch RMSE
changes by at most 4%), and the all-epoch RMSE/P95 mixes in the FLOAT
epochs' own error (Tokyo 3: FLOAT RMSE 47.5 -> 3.8 m; Nagoya 1: 19.8 -> 41.6 m,
because the RTK FLOAT that drifts to 150 m is now emitted instead of replaced
by SPP). Without (b) (components a and c only, diagnostic, not a candidate) the
RTK result still differs from the control through the tight-filter bootstrap
from the fusion state, and RTK velocity is worse (Tokyo 1 RTK velocity RMSE
2.10 -> 2.78 m/s, P95 3.29 -> 4.60). There is no isolated change that keeps
the RTK output of the control while fixing the fused state, since the RTK
prior is the fused/tight state by construction. This candidate therefore does
not try to change the RTK path: component (b) is kept as in v1, the RTK
comparison is run unchanged, and an RTK-gate failure is expected to recur and
is reported as such. The fusion-position recovery below does not touch the RTK
filter: the RTK FLOAT epoch counts of v1 and the final v2 diagnostic are
identical on four runs and differ by 1 and 2 epochs on Nagoya 2 and 3.

## Candidate `velocity_consistency_v2` (one frozen change set, no tuning)

Components (a) heading-latch velocity re-anchor, (b) independent Doppler
velocity for the loose filter and tight re-anchor, and (c) position and
velocity NIS gates of 9.0 per observation are exactly as frozen in
`online_pva_candidate_v2.md`. New:

* (d) `LooseCouplingProcessor::Config::float_position_reanchor_after_rejections
  = 30` (default 0 = off) and `float_reanchor_max_coarse_age_s = 1.0`.
  A separate counter counts consecutive gate rejections of FLOAT/FIXED
  position updates only. SPP/DGPS epochs neither advance nor reset it; a
  FLOAT/FIXED update accepted by the gate resets it. At 30 the existing
  position-only re-anchor (`reanchorPositionFromFixedSolution`: position set
  from the lever-arm-compensated GNSS antenna position, covariance = the
  solution's covariance plus the lever-arm attitude term, all position
  cross-covariances cleared, attitude/velocity/biases untouched) is applied
  from that FLOAT/FIXED solution, then the counter restarts. The re-anchor is
  refused (and retried on the next FLOAT/FIXED epoch) unless the FLOAT/FIXED
  antenna position is consistent with the latest coarse (non-FLOAT/FIXED)
  position no older than 1.0 s: the difference, against the sum of both
  covariances plus the displacement variance (|velocity| * age)^2, must have
  NIS per axis <= `max_position_update_nis_per_observation` (9.0, the
  existing gate value; no new threshold). With no recent coarse position the
  re-anchor is not taken.
  A re-anchor sets `gnss_position_updated` for that epoch, so the accepted-update
  share counts re-anchors.

Constants, from the diagnosis, fixed before the first v2 replay:

* 30 rejections is the repository's own patience for the FIXED re-anchor
  (`max_consecutive_gate_rejections = 30`), so a recovered FLOAT is trusted on
  the same evidence. Over the six v1 runs the logged FLOAT/FIXED rejection
  streaks number 41: 6 are still open at the end of the run (626-1876 epochs,
  the lockouts), 35 end in an accepted update, and of those 30 last at most 22
  epochs and 5 last 30, 36, 173, 809 and 950 epochs (the last three are
  lockouts that ended by chance). So 30 does not fire on the common transient
  rejections but bounds a lockout to about 30 FLOAT epochs (20-30 s at
  1-1.5 Hz).
* 1.0 s is the base-station cadence of the PPC logs (one coarse update per
  rover epoch lies between two exact-base epochs, i.e. at most one base interval
  old), the displacement term makes the test insensitive to the age inside it.
* The cross-check bound reuses the gate value 9.0 because this is the same
  question the gate asks (is this measurement consistent with an independent
  prediction), now with the coarse solution as the prediction; with the coarse
  sigma of 6.4 m it accepts a FLOAT up to about 33 m from the coarse position
  and rejects the 140 m drifting float.

CLI: `gnss pva-evaluate|gnss_pva_replay --candidate velocity_consistency_v2`.
No change to sensor axes, lever arms, time, initialization window, truth
alignment, RTK settings or any other threshold. Candidate `none` and
`velocity_consistency_v1` code paths are unchanged (the new branch is
disabled when `float_position_reanchor_after_rejections <= 0`).

Known limits, not addressed: (i) a run whose exact base is present at every
rover epoch produces no coarse epochs, so the cross-check never allows (d) to
act and the lockout persists; the PPC logs (1 Hz base, 5-10 Hz rover) do not
show this; (ii) a FLOAT drifting less than about 30 m from the coarse position
is followed; (iii) the 10-15 m SPP bias that the fused state carries before the
first re-anchor and Nagoya 1's reverse start remain; (iv) the RTK-filter
feedback above.

## Disclosure of development use

All six normal runs were used for diagnosis before this freeze, so the
comparison is development evidence, not a holdout: (1) the v1 candidate with the
debug print on all six runs; (2) a first v2 version with N = 30 and no
cross-check on Nagoya 1 and Tokyo 3 (Nagoya 1 fused position 59.6 m, Tokyo 3
3.1 m): this showed the drifting-FLOAT failure and is why the cross-check was
added; (3) the final candidate (N = 30, 1.0 s, cross-check at 9.0) on all six
normal runs, and the same without (b) on all six normal runs, to decide
whether (b) causes the RTK change (it does, and removing it is worse for RTK
velocity). No constant was changed after any of these results: 30 and 1.0 s
were set before the first v2 replay, and the cross-check reuses the existing
gate value. The scenario replays (outage, IMU gap) were not run on v2 before
this freeze.

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
   control in every deterministic CSV field (all but `processing_ms`) on all 18
   runs. (The v1-only pre-latch parity gate does not apply: this candidate acts
   before the latch.)

Go only if every gate passes. Otherwise record No-Go with the failed gates, keep
the defaults, and do not tune this candidate after seeing the results.

An informational ablation on the six normal runs follows the decision and is
never used to change the frozen candidate: v1, v1 + recovery (this candidate),
and this candidate without component (b).
