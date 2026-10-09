# Frozen candidate contract: online RTK with the product RTK configuration (`rtk_online_product_v1`)

This contract is frozen in a commit before any comparison replay.

It follows [online_rtk_base_extrapolation_v1_results.md](online_rtk_base_extrapolation_v1_results.md),
a No-Go. A post-hoc isolation study then showed why the online RTK/IMU path
falls far behind the batch product `gnss solve --preset low-cost`. The study
used scratch builds only, on PPC Tokyo 1 and Nagoya 2.

## Diagnosis (isolation study, truth used only for scoring)

1. **The default RTK configuration rejects almost every correct kinematic
   fix.**
   - `RTKConfig` defaults to a static 5 m fixed-position jump limit:
     `max_position_jump_rate_mps = 0`.
   - The documented `low-cost` preset uses a motion-aware limit instead:
     30 m/s with a 5 m minimum. It also uses float prefit-residual resets and
     the AR filter.
   - Library-only RTK on Tokyo 1 with causal base hold:
     - default config: 772 fixes (6.5 % of reference epochs), H P50 0.61 m;
     - low-cost: 8,116 fixes (67.9 %), H P50 0.07 m.
   - The motion-aware jump gate alone gives 5,738 fixes. Removing it from
     low-cost leaves 337.
2. **Base alignment is secondary.**
   - Interpolation, which is non-causal, against the causal hold of
     `rtk_base_extrapolation_v1`:
     - Tokyo 1: 8,269 / 7,895 fixes (H P50 0.029 / 0.041 m).
     - Nagoya 2: 4,544 / 4,155 fixes.
   - Both figures are from the batch pipeline.
3. **The online RTK velocity is a self-confirming loop.**
   - With `enable_velocity_states` the RTK output velocity is the RTK filter's
     velocity state. Only the tight INS time update writes it, and the DD
     update does not measure it.
   - That velocity is fed back to the loose filter and to the tight re-anchor.
   - RTK velocity RMSE on epochs with that update:
     - 4.1 m/s already in the control, which applies it only on the 20 % of
       epochs with an exact base;
     - 12.7 m/s once every epoch is differential.
   - The existing option `independent_doppler_velocity` from
     `velocity_consistency_v1` breaks the loop by using a Doppler
     least-squares velocity. It brings RTK velocity RMSE to 1.75 m/s on
     Tokyo 1 with hold and low-cost.
4. **The app-level output guards change only which non-FIX epochs are
   output**, not the fixed set. The two largest, stationary-drift and
   float-bridge-tail, use future epochs and are excluded from an online path.

## Design (combination of existing, documented pieces; no new constant)

- **(a) Library RTK preset.**
  - `include/libgnss++/algorithms/rtk_presets.hpp` and
    `src/algorithms/rtk_presets.cpp` provide
    `applyRtkPreset(RTKConfig&, const std::string& name)`.
  - They carry the same numeric tables as the app helper
    `applyRtkConfigPreset` (`apps/native/rtk_base_epoch_align.hpp`) for
    `low-cost`, plus the product default `max_baseline_length = 20000` m from
    `gnss solve`.
  - A unit test asserts the library table equals the app table field by field.
  - `gnss solve` and the apps are not changed.
- **(b) Online preset option.**
  - `OnlineRtkImuProcessor::Config::rtk_preset` (string, default empty = none)
    is applied to the RTK filter configuration in `recreateRtkFilter()`.
  - Empty means bit-identical to today.
  - The isolated RTK-prior filter and the fusion configuration are untouched.
- **(c) Base hold.** `base_extrapolation_max_age_s = 2.0` (existing option and
  constant, `rtk_base_extrapolation_v1`).
- **(d) Velocity loop.** `independent_doppler_velocity = true` (existing
  option, `velocity_consistency_v1`).

## Candidate `rtk_online_product_v1`

- It is the current production default (candidate `none`) plus (b)
  `rtk_preset = "low-cost"`, (c) and (d).
- It carries no fusion option of v2-v6. This answers whether the online RTK
  input alone repairs the default.
- CLI: `gnss pva-evaluate|gnss_pva_replay --candidate rtk_online_product_v1`.

## Population, comparison and acceptance

The population, the comparator invocations and the gates are those of
[online_rtk_base_extrapolation_v1.md](online_rtk_base_extrapolation_v1.md),
unchanged:

- six PPC runs plus UrbanNav Odaiba and Shinjuku (Trimble, frozen converter);
- the normal, GNSS outage 60-70 s and IMU gap 60-64 s scenarios;
- 24 control and 24 candidate replays from one binary, interleaved, at most
  3 at a time;
- gates 1-7 per run/scenario, with gate 7 checked on the 18 PPC control
  replays against develop `c37979c3`.

Go only if every gate passes on all 24 run/scenarios. Go does not change the
default. It permits a separate, reviewed default-switch proposal. Nothing is
tuned after the results are seen.

## Expected consequences, stated before the comparison

- **RTK position.** FIXED share near the causal batch level, about 65-70 % of
  PPC Tokyo 1 epochs, and RTK horizontal P50 at the centimetre level.
- **RTK velocity.** About 1.3-1.8 m/s, the Doppler LS level.
- **Fused output.** The fused output may still be poor: the study prototype
  had a fused horizontal P50 of 1.3-3.0 m. Gate 1 may therefore fail on fused
  metrics wherever the control happens to be better.
- **Wrong fixes.**
  - Hold roughly doubles wrong fixes over 0.5 m against interpolation.
  - With the INS prior, Nagoya 2 reached 604 wrong fixes over 0.5 m in the
    study, against 64 in the batch.
  - RTK P95 can therefore regress on some runs. These are known risks, not
    reasons to tune.

## Disclosure of development use

- **Before this freeze:** the isolation study ran scratch builds on PPC Tokyo 1
  and Nagoya 2, including a scratch replay of this exact combination on
  Tokyo 1 (fixes 8,832, RTK H P50 0.047 m, RTK velocity 1.75 m/s).
- **Earlier contracts:** all eight runs were used in earlier contracts.
- **Pre-freeze candidate runs:** unit tests and one 600-epoch prefix on PPC
  Tokyo 1. The prefix confirms that the preset, hold and independent Doppler
  velocity are active. Only status counts are read.

## Implementation notes recorded before the freeze

- **Library presets.** `applyRtkPreset` carries every preset the app helper
  supports: survey, low-cost, odaiba and moving-base. `""` and `"none"` are
  no-ops, as in the app helper.
  - `max_baseline_length = 20000` is added for low-cost and for odaiba, which
    is low-cost plus wide-lane AR. Only low-cost is used here.
  - A unit test compares all 16 fields touched by the library and app tables
    for every preset.
- **Unknown preset names** are rejected in the processor constructor.
- **Pre-freeze prefix.** PPC Tokyo 1, normal scenario, 600 epochs; only counts
  were read:
  - RTK status: SPP 0 / FLOAT 6 / FIXED 594;
  - base epochs: 120 exact, 480 extrapolated;
  - `replay.json` confirms the preset, the hold and the independent Doppler
    velocity.
- **Parity on 600-epoch prefixes of Tokyo 1 / Nagoya 1.** Candidates `none` and
  `rtk_base_extrapolation_v1` are unchanged against their earlier outputs.

## Reported in addition (not gates)

- RTK status counts (SPP, FLOAT, FIXED).
- RTK FIXED share of reference epochs.
- RTK horizontal P50/P95.
- Wrong fixes over 0.1 m and over 0.5 m.
- RTK and fused velocity RMSE.
