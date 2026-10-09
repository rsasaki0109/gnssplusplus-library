# Frozen velocity-consistency candidate v9 (candidate name `velocity_consistency_v8`)

Freeze this contract before running the comparison. The candidate it defines
is `velocity_consistency_v8`. All earlier candidates and records are
unchanged. The production default stays unchanged even if this candidate
passes.

## What this candidate answers

`velocity_consistency_v7` ([results](online_pva_candidate_v8_results.md))
passes every PPC position, fused-velocity, rotation and recovery gate. It
still fails 19 PPC gates and 38 UrbanNav gates.

A post-hoc diagnosis of its outputs used instrumented scratch replays. Truth
was used only offline. The diagnosis found four separate mechanisms.

1. **A 9.6 km FLOAT seeded at the base coordinates (Shinjuku, t = 1705.4 s).**
   - What happened: a 2.8 s rover gap reset the RTK filter. SPP was invalid
     and no trusted anchor existed. When four DD satellites returned, the
     filter initialised its position from the last fallback,
     `rover_pos = base_position_` (`rtk_fallback.cpp`).
   - The result was a FLOAT at the base station: prefit residual RMS 8.6 km
     and error 9,606 m. This single epoch is 97 % of Shinjuku's RTK MSE.
     Without it the RTK RMSE is 18 m.
   - Why it got through: the fused NIS gate rejected it (34,141 per
     observation). The v4 post-gap re-anchor then adopted it anyway, and the
     fused output stayed 9.6 km off for 6 epochs. That event alone makes the
     fused RMSE 231 m.
2. **Post-gap re-anchors onto FLOAT solutions the RTK filter itself flags as
   inconsistent.** At Shinjuku 1446.6 s the re-anchor adopted a FLOAT whose
   prefit residual RMS was 16 m. The preset's own float prefit gate is 4 m RMS
   / 10 m max. The fused error then sat at 30-40 m for about 15 s.
3. **Lost RTK availability on UrbanNav (0.2-0.4 percentage points).**
   - All 64 lost epochs are SPP outputs blanked inside `fallback_spp`
     (`rtk_epoch.cpp`). The rule blanks an SPP with <= 5 satellites that is
     > 25 m from the last trusted position, and it has no age limit.
   - The trusted anchor was a median 10 s old, and up to 109 s.
   - The control never reaches this path on its plain-SPP epochs. The hold
     sends those epochs through `processRTKEpoch`, so the candidate does.
   - The sibling trusted-jump rule in the same function already limits itself
     to an anchor age of <= 3 s.
4. **Exported RTK velocity P95, and the Nagoya 2 first heading latch
   (+0.2 s).**
   - Correction to the v8 results text: the exported RTK velocity is the RTK
     filter's velocity state on every candidate epoch, not the independent
     Doppler velocity. `independent_doppler_velocity` changes only the fusion
     input.
   - The independent Doppler least-squares velocity
     (`spp_velocity::solveVelocityFromObservations`) uses every Doppler row:
     all signals, no elevation mask, no outlier rejection.
   - The SPP processor's velocity uses one signal per satellite, the 15 deg
     mask and its pseudorange outlier rejection. On the same epochs it is
     better at P95: Tokyo 1 2.10 vs 2.99 m/s, Nagoya 1 2.22 vs 3.23,
     Shinjuku 1.30 vs 2.74.
   - The Nagoya 2 latch difference is the 1.0 m/s speed threshold crossed one
     epoch later by the LS velocity (0.01-0.03 m/s difference).
   - On the control's SPP epochs the control's fusion input is that SPP
     velocity.

## Design (four default-OFF options; no new constant)

- **(k) Reject a FLOAT initialised at the base coordinates.**
  - Option: `RTKConfig::reject_float_seeded_at_base`, default false.
  - When true and the filter (re)initialisation took its rover position from
    the final `base_position_` fallback, because no SPP, fix, receiver or last
    solution position was available, the epoch takes the existing
    `fallback_spp` path instead of emitting the FLOAT.
  - The filter stays initialised, so the next epoch proceeds normally.
- **(l) Do not re-anchor onto a FLOAT that fails the float prefit gate.**
  - The RTK processor sets `PositionSolution::float_prefit_gate_exceeded`
    (default false). It is true when the solution's own update prefit
    residual exceeds the configured `max_float_prefit_residual_rms_m` /
    `max_float_prefit_residual_max_m`. These are the low-cost preset values,
    4 m / 10 m, and they are 0 (disabled) in the default configuration.
  - New fusion option `LooseCouplingProcessor::Config::reanchor_requires_prefit_gate_pass`,
    default false. When true, every re-anchor refuses such a solution: the
    post-gap re-anchor (v4) and the 30-rejection FLOAT/FIXED re-anchor (v2).
    The NIS gate then stands as the decision.
- **(m) Bound the SPP blanking by the trusted anchor's age.**
  - Option: `RTKConfig::spp_fallback_blank_max_anchor_age_s`, default 0, which
    means no limit, as today.
  - When > 0, the `fallback_spp` blanking rule applies only if the trusted
    anchor is no older than this.
  - Value: 3.0 s, the horizon of the sibling trusted-jump rule in the same
    function.
- **(n) Use the epoch SPP velocity as the independent velocity.**
  - Option: `OnlineRtkImuProcessor::Config::independent_velocity_from_epoch_spp`,
    default false. It requires `independent_doppler_velocity`.
  - When true, the independent velocity fed to the fusion and tight
    re-anchor is the SPP processor's velocity solved in the same RTK epoch.
    `RTKProcessor` exposes it read-only, with its covariance. It replaces the
    all-rows Doppler LS. If that velocity is not valid, the epoch carries no
    GNSS velocity, as today.
  - The same velocity, when valid, is also exported as the RTK output
    velocity. Otherwise the exported velocity is unchanged.

## Candidate `velocity_consistency_v8`

- It includes everything in `velocity_consistency_v7`
  ([contract](online_pva_candidate_v8.md)), unchanged.
- It adds (k), (l) with the low-cost preset values already in the RTK
  configuration, (m) at 3.0 s, and (n).
- CLI: `gnss pva-evaluate|gnss_pva_replay --candidate velocity_consistency_v8`.

## Population, comparison and acceptance

These are unchanged from [online_pva_candidate_v8.md](online_pva_candidate_v8.md):

- Runs: six PPC runs, plus UrbanNav Odaiba and Shinjuku with the Trimble rover.
- Scenarios: normal, GNSS outage 60-70 s and IMU gap 60-64 s.
- Replays: 24 control and 24 candidate replays from one binary, interleaved,
  at most 3 at once.
- Gates 1-7 apply per run/scenario. Gate 7 is checked on the 18 PPC control
  replays against develop `c37979c3`.

Go only if every gate passes on all 24 run/scenarios. Go does not change the
default. It permits a separate, reviewed default-switch proposal. Nothing is
tuned after the results are seen.

## Expected consequences, stated before the comparison

- **Shinjuku 1705.4 s:** the base-seeded FLOAT is not emitted, so the fused
  9.6 km excursion disappears. The diagnosis counterfactual of (k)+(l) gave
  Shinjuku fused RMSE 11.8 m and RTK RMSE 18.0 m.
- **Availability with (m):**
  - Shinjuku is estimated at -0.05 pp against the control (passes).
  - Odaiba is estimated at -0.13 pp (fails the 0.1 pp gate). Only 3 of its
    12 lost epochs have an anchor older than 3 s.
  - This is a known expected failure. The horizon is not changed to make the
    gate pass.
- **Velocity with (n):** the fusion velocity on SPP epochs equals the
  control's, and the exported RTK velocity P95 is expected below the
  control's. The Nagoya 2 latch is expected to coincide with the control's.
- **Risk:** (l) can delay recovery after a real outage when the first FLOAT
  has a large prefit residual. Only Tokyo 1 GNSS outage was tested in the
  diagnosis.

## Disclosure of development use

- All eight runs were used in earlier contracts.
- The diagnosis ran instrumented v7 replays on Shinjuku, Odaiba, Tokyo 1 and
  Nagoya 1.
- It ran three counterfactuals on Shinjuku normal: two jump guards and
  (k)+(l), plus (k)+(l) on Tokyo 1 GNSS outage.
- It made offline velocity comparisons on Tokyo 1, Tokyo 3, Nagoya 1 and
  Shinjuku.
- Before this freeze the candidate is run only on 600-epoch prefixes of PPC
  Tokyo 1 and UrbanNav Shinjuku. These confirm that the options are active,
  and only counts are read.

## Implementation notes recorded before the freeze

- **(l) Prefit flag scope.** The flag is computed from the float update's
  prefit residual for every emitted solution, FIXED included. The re-anchor
  refusal therefore also covers flagged FIXED solutions on the v2 and v4
  paths. The older FIXED-patience re-anchor (`max_consecutive_gate_rejections`)
  is not gated.
- **(k) Seed tracking.** It is per epoch. The base-fallback seed flag is
  re-evaluated at every re-seed and is cleared by any other seed source. A
  rejected epoch keeps the filter initialised. STATIC and moving-base modes
  never set it.
- **(m) Age boundary.** An anchor exactly 3.0 s old still blanks.
- **(n) Velocity routing.**
  - The epoch SPP velocity and covariance come from the SPP processor, with
    the existing 0.5 m/s Doppler sigma.
  - It also applies on missing-base epochs, where it equals the SPP-fallback
    output.
  - The isolated RTK-prior filter receives the same independent velocity as
    the fused filter, as in v7 where it received the LS velocity.
- **Diagnostics.** `replay.json` for this candidate records the four options
  and these counters:
  - base-seed rejections;
  - age-limited blanks;
  - prefit-flagged epochs;
  - fusion re-anchor refusals;
  - SPP-velocity exports.
- **Tests.**
  - `run_tests`: 1,343 pass.
  - `gnss_online_tests`: 37/37.
  - The Python PVA, comparison and converter tests pass.
- **OFF parity.** On 600-epoch prefixes, `none` and `velocity_consistency_v7`
  are unchanged against the v7 comparison outputs: Tokyo 1, and v7 on
  Shinjuku.
- **Pre-freeze prefixes of the candidate (600 epochs, counts only).**

  | Prefix | RTK FIXED / FLOAT / SPP | Prefit-flagged epochs | Re-anchor refusals | Base-seed rejections | Age-limited blanks | SPP-velocity exports |
  |---|---|---|---|---|---|---|
  | Tokyo 1 | 594 / 6 / 0 | 0 | 0 | 0 | 0 | 600 |
  | Shinjuku | 477 / 114 / 9 | 43 | 26 | 0 | 0 | 600 |

