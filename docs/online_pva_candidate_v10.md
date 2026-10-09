# Frozen velocity-consistency candidate v10 (candidate name `velocity_consistency_v9`)

Freeze this contract before running the comparison. The candidate it defines
is `velocity_consistency_v9`. All earlier candidates and records are
unchanged. The production default stays unchanged even if this candidate
passes.

## What this candidate answers

`velocity_consistency_v8` ([results](online_pva_candidate_v9_results.md))
fails 6 PPC gates and 2 UrbanNav gates. Five of the six PPC failures are the
attitude of Tokyo 1 and Tokyo 2, which is lost from the first heading latch.

A post-hoc isolation study (scratch builds; truth used only to score) found
the mechanism.

- **Which option.** Option (n), the epoch SPP velocity as the fused filter's
  velocity input, alone reproduces the loss. Removing it from v8 restores the
  attitude of v7. Options (k), (l) and (m) are not involved.
- **Not the velocity values.** The SPP and the all-rows Doppler LS
  velocities agree to 0.01-0.07 m/s on Tokyo 1 at 0-30 s. Epoch, frame,
  static bias and the direction test are ruled out.
- **The velocity covariance is the cause.**
  - The LS covariance counts every signal row as independent: about 79 rows
    against about 31 satellites. Its sigma is about 1.6 x smaller than the SPP
    sigma, while the actual errors are equal.
  - Swapping only the covariance moves the failure: SPP velocity with LS
    covariance is healthy, and LS velocity with SPP covariance loses attitude.
- **Why v7 survived the latch.**
  - The heading latch rotates only the attitude. The first fused velocity
    updates after it have a large NIS.
  - With the over-tight LS covariance, that NIS exceeds the gate of 9 three
    times in a row (Tokyo 1: 9.5, 9.8, 13.1). This triggers the existing
    3-rejection velocity re-anchor, which happens to restore consistency.
  - With the honest SPP covariance the NIS is 7-8, the updates are accepted,
    and no re-anchor happens. The inconsistent velocity/attitude
    cross-covariance then drives the heading and gyro bias away.
- **A direct test.** Enabling the existing option
  `reanchor_velocity_on_heading_latch` restores the attitude with (n) on. This
  option was introduced by `velocity_consistency_v1` precisely for this
  situation: "the latch rotates only the attitude; velocity and every
  velocity/attitude/bias correlation were produced by a filter running with
  an arbitrary pre-latch yaw". On 600-epoch prefixes it gave Tokyo 1 rotation
  3.5 deg and Tokyo 2 rotation 1.6 deg.

## Candidate `velocity_consistency_v9` (no new option or constant)

* Everything in `velocity_consistency_v8`
  ([contract](online_pva_candidate_v9.md)), unchanged.
* Plus `LooseCouplingProcessor::Config::reanchor_velocity_on_heading_latch = true`
  on the fused filter only. It is set after the RTK-prior snapshot, so the
  isolated prior filter keeps the v7 settings.

CLI: `gnss pva-evaluate|gnss_pva_replay --candidate velocity_consistency_v9`.
`replay.json` records the v8 options and counters, plus this flag.

Rejected alternative: keep (n) for the exported RTK velocity only and restore
the LS velocity as the fusion input. On Tokyo 1, Tokyo 2 and Shinjuku this
gave v7's attitude and v8's RTK velocity gain. It was rejected because it
keeps the over-tight LS covariance. The attitude would still depend on that
covariance tripping a rejection gate after the latch, which is not a designed
mechanism. If this candidate is No-Go, that alternative is the fallback under
its own contract.

## Population, comparison and acceptance (unchanged)

These are as in [online_pva_candidate_v9.md](online_pva_candidate_v9.md):

- **Runs:** the six PPC runs plus UrbanNav Odaiba and Shinjuku (Trimble).
- **Scenarios:** normal, GNSS outage 60-70 s and IMU gap 60-64 s.
- **Replays:** 24 control and 24 candidate replays from one binary,
  interleaved, at most 3 at once.
- **Gates:** gates 1-7 per run/scenario. Gate 7 is checked on the 18 PPC
  control replays against develop `c37979c3`.

Go only if every gate passes on all 24 run/scenarios. Go does not change the
default; it permits a separate, reviewed default-switch proposal. Nothing is
tuned after the results are seen.

## Expected consequences, stated before the comparison

- **Tokyo 1 and Tokyo 2:** attitude is restored.
- **Other runs:** the latch re-anchor also changes them, so their rotation
  results can move in either direction against v8.
- **Odaiba IMU gap:** the RTK position P95 failure of v8 (26.8 -> 28.5 m) is
  not addressed and may persist.

## Disclosure of development use

- All eight runs were used in earlier contracts.
- The isolation study ran full Tokyo 1 and Tokyo 2 ablations, the
  export-only variant on Tokyo 1, Tokyo 2 and Shinjuku, and the latch
  re-anchor on Tokyo 1 and Tokyo 2 prefixes.
- Before this freeze the candidate is run only on a 600-epoch prefix of PPC
  Tokyo 1, and only counts are read:
  - RTK status: FLOAT 6, FIXED 594.
  - `replay.json` records the flag.
- On the same prefix, `none` and `velocity_consistency_v8` match the v8
  comparison outputs in every field except `processing_ms`.
