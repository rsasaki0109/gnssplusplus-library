# Frozen velocity-consistency candidate v11 (candidate name `velocity_consistency_v10`)

Freeze this contract before running the comparison. The candidate it defines
is `velocity_consistency_v10`. Document numbering follows the contract
revision (eleventh contract), one ahead of the candidate name. All earlier
candidates and records are unchanged and stay selectable. The production
default stays unchanged even if this candidate passes.

## What this candidate answers

`velocity_consistency_v9` ([results](online_pva_candidate_v10_results.md))
passes every PPC gate, but its post-hoc findings show an early attitude loss
of about 180 deg that the relative gates could not see, because the control
fails the same way:

- **Odaiba:** the candidate's rotation error exceeds 90 deg on about half of
  the epochs, from 29.6 s, before and after the BeiDou reader fix.
- **Shinjuku:** with the reader fix, the candidate starts doing the same from
  25.4 s (rotation RMSE 3.8 -> 105 deg).

A post-hoc diagnosis (scratch branch `v9-flip-diag`, commit `aeff5222`; truth
used only to score) found the mechanism. It was run on full replays of all 8
normal scenarios and the 6 UrbanNav scenario runs.

- **Pre-latch updates corrupt tilt and bias.**
  - Until the first heading latch the yaw is unobservable (sigma 180 deg) and
    the error-state linearization about an arbitrary nominal yaw is invalid.
  - Every position, velocity and ZUPT update in that phase nevertheless
    spreads its innovation into roll/pitch and into the gyro and accelerometer
    biases.
  - The latch then resets only the yaw. The tilt and bias learned in that
    regime stay, with a tight and wrong covariance.
- **Odaiba.** The corrupted tilt makes the v5 direction test see the wrong
  sign of the body-forward velocity, so it flips a correct latch by 180 deg.
- **Shinjuku.** The latch is correct, and the filter then drifts away from it
  because of the contaminated tilt and bias.
- **Not an axis or time convention issue.** IMU-versus-truth sign checks agree
  with the axes in use, and IMU time shifts of +-0.16 s leave Odaiba just as
  broken.
- **Counterfactual.** Making the pre-latch updates Schmidt-Kalman consider
  updates for the attitude and both biases (mask `0x7fc0`, arm `fixC`)
  removed the early loss on Odaiba and Shinjuku in that diagnosis.

## Candidate `velocity_consistency_v10`

* Everything in `velocity_consistency_v9`
  ([contract](online_pva_candidate_v10.md)), unchanged.
* Plus one new default-off option,
  `LooseCouplingProcessor::Config::consider_attitude_and_biases_before_heading_latch`
  (bool, default `false`), set to `true` on the fused filter only. It is set
  after the RTK-prior snapshot, so the isolated prior filter keeps the v7
  settings (this is where the diagnosis arm set it).

**Behavior of the option.** While the filter's heading is not latched
(`heading_aligned_` false), every measurement update of the loose filter
leaves the attitude (error states 6-8), the accelerometer bias (9-11) and the
gyro bias (12-14) unchanged: their gain rows are zero. The covariance is
updated in Joseph form, which is valid for any gain, so the consider states
keep their variance and their cross-covariances with position and velocity
are updated consistently (Schmidt-Kalman consider update). Position and
velocity are corrected as before. After the latch every update is the plain
update, bit for bit.

**Which updates.** The mask is applied inside the single function through
which the filter applies updates (`applyUpdateAndInject`), so it covers
exactly: the GNSS position update, the GNSS velocity update, the ZUPT update
and the NHC update (NHC is disabled in this replay, so it does not occur). There is no exemption for ZUPT. It does not touch the re-anchors
(they overwrite states and covariance, they are not Kalman updates), the
tight-coupling processor, or the RTK filter. `fusion_update::applyDenseUpdate`
takes the mask as a trailing parameter that defaults to "no mask", so every
other caller is bit-identical.

**Why Schmidt consider.** A state that is unobservable before the latch must
not absorb residuals. The residuals of this phase are caused by the unknown
yaw, so they say nothing about roll, pitch or the sensor biases. Holding those
states and letting only position and velocity move keeps what the alignment
and the static window knew, without inventing information. No new constant:
the mask is the attitude and both bias blocks of the existing 15-state layout.

CLI: `gnss pva-evaluate|gnss_pva_replay --candidate velocity_consistency_v10`.
`replay.json` records the v8 options and counters, the v9 flag
`reanchor_velocity_on_heading_latch`, and
`consider_attitude_and_biases_before_heading_latch`.

## Population, comparison and acceptance

As [online_pva_candidate_v10.md](online_pva_candidate_v10.md), plus gate 8:

- **Runs:** the six PPC runs plus UrbanNav Odaiba and Shinjuku (Trimble), with
  the existing UrbanNav conversion used for v9.
- **Scenarios:** normal, GNSS outage 60-70 s and IMU gap 60-64 s.
- **Replays:** 24 control (`none`) and 24 candidate replays from one binary,
  interleaved, at most 3 at once.
- **UrbanNav inputs are read with the RINEX 3.00-3.02 BeiDou B1I fix**
  (develop `e389c8b3`, in `a7a5d5bf`). UrbanNav control values therefore
  differ from every earlier record, including the control columns of the v9
  results. Control and candidate use the same reader. PPC (RINEX 3.04) is
  unaffected.
- **Gates 1-7** per run/scenario, unchanged. Gate 7 is the bit-identity of the
  control `none` of the candidate binary against `none` built from develop
  `a7a5d5bf`, checked on the 18 PPC control replays (every CSV field except
  `processing_ms`).
- **Gate 8, attitude integrity (absolute, not relative to the control).** For
  each candidate replay, every run and scenario: the fraction of scored epochs
  with `rotation_deg > 90` deg must be at most 0.01. A scored epoch is an
  epoch of `errors.csv` with a `rotation_deg` value. The comparison is
  strictly above 90 deg, and the control's value is recorded for information
  only. It is computed with
  `scripts/analysis/compare_online_pva.py --attitude-integrity --candidate-name velocity_consistency_v10 --contract docs/online_pva_candidate_v11.md`.
  Without that flag the comparator output is unchanged.

Go only if all eight gates pass on all 24 run/scenarios. Go does not change
the default. It permits a separate, reviewed default-switch proposal, which
needs a fresh holdout because all eight runs are development data. Nothing
is tuned after the results are seen.

## Expected consequences, stated before the comparison

These come from the diagnosis counterfactuals, which are disclosed below as
already seen on full runs.

- **UrbanNav (Odaiba, Shinjuku):** rotation about 2-3 deg, and no epochs above
  90 deg.
- **Nagoya 1:** rotation RMSE 1.37 -> about 2.9 deg against v9, still far
  below the control.
- **Gate 8 (every run/scenario):** expected to pass.
- **Known risk, not tuned:** the per-scenario 1.01x relative gates may still
  fail on UrbanNav. The control has large scenario-to-scenario variance there
  (see the post-hoc findings in
  [online_pva_candidate_v10_results.md](online_pva_candidate_v10_results.md)).
  If that happens it is a No-Go under this contract.

## Disclosure of development use

- All eight runs were used in earlier contracts and are development data.
- The diagnosis (`v9-flip-diag`, `aeff5222`) ran full counterfactuals on all
  8 normal scenarios and on the 6 UrbanNav scenario runs. Besides the arm
  above it ran other arms (a post-latch direction verification, skipping the
  pre-latch velocity update, narrower masks, a ZUPT exemption, IMU time
  shifts). None of those is in this candidate.
- This candidate's mask and flag placement are those of arm `fixC`. Before
  this freeze the implementation was checked for bit-identity only:
  - Candidate `velocity_consistency_v10` equals the `fixC` outputs
    (`pva.csv`, every field except `processing_ms`) on full Odaiba (Trimble)
    and PPC Tokyo 1 normal.
  - Candidate `none` and `velocity_consistency_v9` equal a binary built from
    develop `a7a5d5bf` on full PPC Tokyo 1 normal.
  - A 600-epoch prefix of PPC Tokyo 1 was run with the candidate, and only
    counts were read: RTK status FLOAT 6, FIXED 594; `replay.json` records
    both flags.
- No other candidate run was made before the freeze. The comparison itself
  has not been run.
