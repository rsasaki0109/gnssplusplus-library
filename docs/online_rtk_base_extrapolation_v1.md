# Frozen candidate contract: online RTK base-epoch extrapolation (`rtk_base_extrapolation_v1`)

This contract is frozen in a commit before any comparison replay. It applies
to the online RTK/IMU processor used by `gnss_pva_replay` and by the online
PVA product path: `OnlineRtkImuProcessor`.

## Problem (diagnosis, offline truth used only for scoring)

`OnlineRtkImuProcessor::processRover` runs differential RTK only when a base
epoch exists at the rover time within 1e-6 s. Otherwise it falls back to
single-point positioning, and it discards past base epochs as "expired". The
code comment explains this as avoiding a stale base "stamped with the rover
time".

Every dataset here has a 1 Hz base and a 5 Hz rover, so 4 of every 5 rover
epochs have no differential solution:

| Run (control, normal) | Exact-base epochs | RTK status SPP / FLOAT / FIXED |
|---|---:|---|
| PPC Tokyo 1 | 2,388 of 11,928 (20 %) | 11,171 / 660 / 15 |
| UrbanNav Odaiba | about 20 % | 6 FIXED |

- **Effect on RTK.** The carrier-phase filter never stays converged, and the
  exported RTK position RMSE is large: 14-46 m on PPC and 200 m on Odaiba.
- **Effect on the batch product.** The batch product `gnss solve` aligns the
  base by interpolation and fixes about 9,000 epochs on the same run. That
  interpolation needs the next base epoch, so it is not causal.
- **Existing real-time option.** The live app `gnss live` has a "hold" option
  that relabels the last base epoch with the rover time, without correcting the
  satellite motion. That is a range error of up to about 800 m/s x age.

## Design (one change, no new constant)

`OnlineRtkImuProcessor::Config::base_extrapolation_max_age_s`, default 0 (off).
With 0 the processor is bit-identical to today.

When it is > 0 and no exact base epoch exists for a rover epoch, the processor
takes the **latest base epoch at or before the rover time**. Base epochs
already passed are kept as the "latest past" instead of being discarded. If
that epoch's age is <= `base_extrapolation_max_age_s`, it is aligned to the
rover time with a **geometry-corrected zero-order hold**. For each signal of
each satellite:

```
modeled(t)  = |sat(t - tau) - base| + Saastamoinen(base, el)
            (sat state from the causal broadcast ephemerides of this epoch)
P_target    = modeled(t_rover) + (P_base   - modeled(t_base))
L_target*l  = modeled(t_rover) + (L_base*l - modeled(t_base))   (only if no LLI/loss of lock)
```

- **Formula.** This is the existing `interpolateBaseEpoch` model of the batch
  path (`apps/native/rtk_base_epoch_align.hpp`) with the "after" epoch removed.
  The residual (clock, atmosphere, multipath) is held, and the satellite motion
  is modelled.
- **Doppler** is copied. **SNR, LLI, code** are copied from the base epoch.
- **Fallbacks** (unchanged behaviour):
  - A satellite whose modeled range fails, or whose elevation is <= 0.05 rad,
    is omitted.
  - If no signal survives, the epoch takes the existing SPP fallback.
  - If the age exceeds the limit, the epoch also takes the existing SPP
    fallback.
- **Base position** is the configured base position, the RINEX header in the
  replay. It is the same position the RTK filter already uses.
- **Helper location.** The helper becomes library code shared with the batch
  path. The batch path's numerical behaviour must not change.
- **Causality.** Only base epochs received at or before the rover time are
  used. Receipt and ephemeris causality of the replay are unchanged.
- **Rover time.** The result is stamped with the rover time and passed to
  `processRTKEpoch` exactly like an exact base epoch.

**Constant.** `base_extrapolation_max_age_s = 2.0 s`.
- This is the existing `kMaxInterpolationGapSeconds` of the batch base
  interpolation, the repository's horizon for aligning a base epoch.
- It is not tuned. With a 1 Hz base, the age of a held epoch is at most 0.8 s,
  or 1.8 s when one base epoch is missing.

### Why it is sound

- **What cancels.** Between-satellite single differences remove the base clock.
  Holding the code/phase residual for <= 2 s keeps the base clock drift, which
  is common to all satellites. It also keeps the atmospheric and multipath
  change at the base over that interval: millimetres to centimetres for phase at
  a static base.
- **What does not cancel** is the satellite motion. It changes the range by up
  to about 800 m/s x age, and the model removes it to the accuracy of the
  broadcast orbit.
- **Precedent.** This is the standard real-time-kinematic treatment of a
  lower-rate base, with an age limit (RTKLIB's "age of differential").

## Candidate `rtk_base_extrapolation_v1`

- The current production default (candidate `none`) plus
  `base_extrapolation_max_age_s = 2.0`.
- Nothing else changes: no fusion option, no RTK setting.
- CLI: `gnss pva-evaluate|gnss_pva_replay --candidate rtk_base_extrapolation_v1`.

## Population and comparison

- **Runs:** the six PPC runs plus UrbanNav Tokyo Odaiba and Shinjuku. UrbanNav
  uses the Trimble rover, converted with the frozen
  `scripts/convert_urbannav_to_ppc_layout.py`.
- **Scenarios:** normal, GNSS outage 60-70 s, IMU gap 60-64 s.
- **Replays:** 24 control (`none`) and 24 candidate replays from one binary,
  interleaved on the same quiet host, at most 3 at once.
- **Comparator:** `compare_online_pva.py`, run once for PPC (default runs) and
  once for UrbanNav (`--runs Odaiba_trimble Shinjuku_trimble`).

## Acceptance (frozen gates, per run/scenario, both comparator invocations)

1. RTK and fused position, RTK and fused velocity, and full-rotation error each
   <= 1.01 x control, for RMSE and for P95. This holds on both the all-output and
   the common-valid cohorts.
2. Coverage loses at most 0.1 percentage point.
3. The first fresh attitude and the first heading latch come no later. Scenario
   recovery also comes no later. A null is censored, never zero.
4. Processor P95 <= 2 x control.
5. At least one normal-run full-rotation RMSE/P95 improvement.
6. Missing metrics, scenarios, truth mismatches or input-hash mismatches fail.
7. Candidate `none` built from the candidate tree is bit-identical to candidate
   `none` from develop `c37979c3` in every deterministic CSV field on the 18 PPC
   runs.

Go only if every gate passes on all 24 run/scenarios. Otherwise record No-Go
with the failed gates. Go does not change the default; it permits a separate,
reviewed default-switch proposal. Nothing is tuned after the results are seen.

## Disclosure of development use

- **Diagnosis inputs.** The diagnosis used existing control outputs of PPC Tokyo
  1 and UrbanNav Odaiba (exact-base share and RTK status counts) and the
  batch `gnss solve` result on Tokyo 1.
- **Earlier use of UrbanNav.** The two UrbanNav runs were also the holdout for
  the v6 default switch, which is now development data. They were not used to
  design this change.
- **Pre-freeze runs.** The candidate implementation is run only through unit
  tests and two bounded prefixes before the freeze:
  - PPC Tokyo 1, 600 epochs;
  - UrbanNav Odaiba, 600 epochs.
  These only confirm that extrapolated epochs are produced and the RTK
  status mix changes.

## Reported in addition (not gates)

- Per run: exact, extrapolated and SPP-fallback epoch counts.
- RTK status counts (SPP, FLOAT, FIXED) for control and candidate.
- Fix rate and the RTK FIXED-epoch 3D error P95.
