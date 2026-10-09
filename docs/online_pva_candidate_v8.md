# Frozen velocity-consistency candidate v8 (candidate name `velocity_consistency_v7`)

Freeze this contract before running the comparison. The candidate it defines
is `velocity_consistency_v7`. All earlier candidates and their records are
unchanged. The production default stays unchanged even if this candidate
passes.

## What this candidate answers

Two earlier results point the same way.

- **`velocity_consistency_v6`** fixes the fusion-layer failures found on PPC:
  the reverse start, the moving resets and the gyro bias after an IMU gap.
  It still runs on the online RTK input, which delivered differential
  solutions on only 20 % of epochs and fixed 13-49 epochs per run
  ([online_rtk_base_extrapolation_v1.md](online_rtk_base_extrapolation_v1.md)).
- **`rtk_online_product_v1`** repairs that RTK input
  ([results](online_rtk_product_config_v1_results.md)):
  - RTK fixes rise to 2,900-10,400 per run.
  - Fused position RMSE improves 2-8 x on seven of eight runs.
  - It fails where v6's fusion fixes are absent: Nagoya 1 rotation from the
    reverse start, and Nagoya 2 rotation.
  - Its fused filter diverges on Shinjuku, because it has no NIS gate and no
    re-anchor against a burst of bad RTK input.

This candidate asks one question: does the repaired RTK input plus the v6
fusion set pass where each alone failed?

## Candidate `velocity_consistency_v7` (one frozen change set, no new option or constant)

* Everything in `velocity_consistency_v6`
  ([contract](online_pva_candidate_v7.md)), unchanged:
  - the isolated RTK-prior filter (the control fusion configuration);
  - RTK reported covariance mode `SPP_CONSISTENCY_SCALED`;
  - NIS gates of 9;
  - the FLOAT/FIXED re-anchor after 30 rejections, and the post-gap re-anchor
    (1.0 s);
  - the heading-latch direction test;
  - the rover-gap RTK-only reset;
  - the gyro-bias carry.
* Plus the RTK input of `rtk_online_product_v1`
  ([contract](online_rtk_product_config_v1.md)), unchanged:
  - `rtk_preset = "low-cost"` (library preset);
  - `base_extrapolation_max_age_s = 2.0` (causal base hold);
  - `independent_doppler_velocity = true`.
* The preset applies to the RTK filter only. The RTK-prior and fused filters
  keep the configurations they have in v6.
* The only code change is the candidate wiring in `gnss_pva_replay` and
  `gnss pva-evaluate`. `replay.json` now records the preset fields for any
  candidate with a preset; for `rtk_online_product_v1` the content is
  unchanged.

## Population, comparison and acceptance (unchanged)

The population, comparator invocations and gates are those of
[online_rtk_product_config_v1.md](online_rtk_product_config_v1.md):

- **Runs:** six PPC runs plus UrbanNav Odaiba and Shinjuku (Trimble).
- **Scenarios:** normal, GNSS outage 60-70 s and IMU gap 60-64 s.
- **Replays:** 24 control (`none`) and 24 candidate replays from one binary,
  interleaved, at most 3 at once.
- **Gates:** gates 1-7 per run/scenario. Gate 7 is checked on the 18 PPC
  control replays against develop `c37979c3`.

Go only if every gate passes on all 24 run/scenarios. Go does not change the
default; it permits a separate, reviewed default-switch proposal. Nothing is
tuned after the results are seen.

## Expected consequences, stated before the comparison

- RTK columns: as in `rtk_online_product_v1`, with this caveat. In
  `rtk_online_product_v1` the RTK filter's INS prior came from the fused
  filter, while v3-v6 isolate it in the RTK-prior filter. The RTK columns can
  therefore differ from `rtk_online_product_v1`.
- Nagoya 1 rotation: improves, through the v5 direction test.
- Shinjuku: is expected not to diverge, because v3's NIS gates and re-anchors
  bound the effect of bad RTK epochs. It is the main risk.
- Earlier v4 limit: v6 rejects SPP-only epochs after the outage, so the
  post-outage recovery delay seen on UrbanNav should disappear, because the
  base hold leaves no SPP-only epochs.

## Disclosure of development use

- All eight runs were used in earlier contracts, including the full 48-replay
  comparisons of v6 on PPC and of `rtk_online_product_v1` on all eight runs.
- Before this freeze the candidate is run only on one 600-epoch prefix of PPC
  Tokyo 1. That run is used to confirm the wiring, and only status counts are
  read.

## Pre-freeze check (600-epoch prefix of PPC Tokyo 1; counts only)

Candidate `velocity_consistency_v7`:

| Item | Count |
|---|---:|
| RTK status FLOAT | 6 |
| RTK status FIXED | 594 |
| Extrapolated-base epochs | 480 |

`replay.json` records the preset and the independent Doppler velocity.

The same prefix confirms that the wiring change leaves earlier candidates
untouched. Candidates `none`, `velocity_consistency_v6` and
`rtk_online_product_v1` match their earlier outputs in every field except
`processing_ms`.

## Reported in addition (not gates)

- v6 -> v7 and `rtk_online_product_v1` -> v7 per run.
- RTK status counts.
- Direction-test flips, rover-gap resets and gyro seeds.
