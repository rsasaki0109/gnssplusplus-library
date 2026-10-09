# Holdout result: default switch to `velocity_consistency_v6` is No-Go (4 of 186 gates)

The frozen contract is [online_pva_default_switch_holdout_v1.md](online_pva_default_switch_holdout_v1.md).

- **Freeze commit:** `90d6921d`. Nothing in the frozen set was changed after the
  results were seen: not the candidate, the conversion, the configuration or the
  gates.
- **Contract SHA256:** `123caf558a5a214c6902b04d3cce79e2a782ce783a2fc97a914591304658b412`.
- **Machine-readable record:**
  [online_pva_decision_holdout_v1.json](online_pva_decision_holdout_v1.json).

**The production default stays unchanged.**

## Decision

**No-Go: 4 of 186 gates fail** over 6 run/scenarios. The population is UrbanNav
Tokyo Odaiba and Shinjuku, Trimble rover, in the normal, GNSS outage 60-70 s and
IMU gap 60-64 s scenarios.

The other gates all pass:

- **Gate 4, timing:** processor P95 candidate/control is 0.965 / 1.026 / 1.073
  (min / mean / max).
- **Gate 5, improvement:** present.
- **Gate 7, parity:** candidate `none` from this tree is bit-identical to develop
  `c37979c3` in every deterministic field on all 18 PPC runs (175,902 rows).
- **Replays:** all 12 holdout replays passed, with complete truth matches and
  identical input hashes.
- **Replay binary SHA256:** `8fa5d784...`.

| Run/scenario | Failed gate | Control | v6 |
|---|---|---:|---:|
| Odaiba normal | all-output rotation RMSE | 105.6 deg | 109.3 deg |
| Odaiba normal | common-valid rotation RMSE | 105.6 deg | 109.3 deg |
| Odaiba GNSS outage | scenario GNSS-update recovery | 0.0 s | 0.8 s |
| Shinjuku GNSS outage | scenario GNSS-update recovery | 0.0 s | 0.8 s |

## Full results, control -> v6

Values are RMSE / P95.

| Run/scenario | Fused pos m | Fused vel m/s | Rotation deg | RTK pos m |
|---|---|---|---|---|
| Odaiba normal | 81.2 -> 16.7 / 157.1 -> 39.7 | 6.26 -> 1.92 / 13.96 -> 3.37 | **105.6 -> 109.3** / 171.4 -> 172.1 | 199.9 / 47.8 (same) |
| Odaiba GNSS outage | 68.9 -> 14.8 / 156.5 -> 36.7 | 6.73 -> 1.76 / 12.85 -> 3.30 | 106.1 -> 105.3 / 171.6 -> 170.9 | 200.0 / 28.4 (same) |
| Odaiba IMU gap | 57.6 -> 9.7 / 157.2 -> 26.0 | 5.47 -> 0.70 / 11.42 -> 1.14 | 52.8 -> 10.7 / 101.1 -> 17.1 | 199.4 / 26.8 (same) |
| Shinjuku normal | 82.8 -> 47.9 / 175.1 -> 127.1 | 9.27 -> 0.76 / 20.95 -> 1.53 | 68.7 -> 8.5 / 152.6 -> 20.2 | 27.6 / 65.6 (same) |
| Shinjuku GNSS outage | 79.9 -> 17.9 / 171.6 -> 46.0 | 9.25 -> 0.79 / 21.15 -> 1.50 | 95.0 -> 11.3 / 169.4 -> 25.6 | 28.3 / 65.9 (same) |
| Shinjuku IMU gap | 108.3 -> 17.3 / 213.3 -> 44.9 | 10.39 -> 0.68 / 23.65 -> 1.39 | 93.1 -> 6.0 / 168.3 -> 6.6 | 28.5 / 70.3 (same) |

Reading the table:

- **Large improvements.** Fused position and velocity improve in all six
  run/scenarios. Rotation improves strongly in four of them, for example
  Shinjuku normal 68.7 -> 8.5 deg.
- **The current default is as broken on the holdout as on PPC:** rotation
  RMSE is 53-106 deg.
- **The failures are narrow, but real:**
  1. **Odaiba normal and GNSS-outage attitude.** Neither control nor v6 is
     usable here (about 105 deg RMSE). v6 is 3.5 % worse in the normal run.
     - The first heading latch, at 10.4 s, is roughly correct: course -38.8 deg
       against a true heading of about -27 deg. The vehicle is moving forward
       (`v_long` +0.66 m/s), so there is no flip.
     - The error then wanders: 76 deg at 60 s, 119 deg at 120 s.
     - The pre-gap fused gyro z bias is +0.044 rad/s.
     - In the IMU-gap scenario the fused filter is recreated at 64 s, and v6
       then reaches 10.7 deg.
     - Odaiba's RTK RMSE is 200 m against a P95 of 48 m, so a few epochs have
       very large errors. Whether these outliers, the declared zero lever arm,
       or the about 0.15 s IMU-versus-truth time offset drives the drift was not
       investigated. The investigation would itself use this data.
  2. **Recovery after the GNSS outage is 0.8 s later.**
     - The UrbanNav base is 1 Hz, so after the outage the epochs at 70.0-70.6 s
       have no exact base, and the RTK filter falls back to SPP.
     - The control applies those SPP positions. v6 rejects them with its NIS
       gate, and its post-gap re-anchor covers only FLOAT/FIXED.
     - The v6 contract named this limit: "a rejected SPP first update after a
       gap is not re-anchored".
     - The first exact-base epoch, 70.8 s, is applied. PPC's 5 Hz base never
       exercised this case.

## v6 behaviour on the holdout (reported, not gated)

- **Direction test:** no flip in any replay.
  - First latch `v_long`: +0.66 m/s (Odaiba), +0.70 m/s (Shinjuku).
  - The re-latches after the IMU reset were not tested (`v_long` invalid while
    moving), as designed.
- **Rover gaps:** Shinjuku has 4 `rover_gap_rtk_reset` per replay, so 4 rover
  gaps of more than 2 s were bridged without re-initializing the fused filter.
  Odaiba has none.
- **Gyro-bias seed at the IMU-gap re-initialization (z, rad/s):**

  | Run | Seed | Window mean |
  |---|---:|---:|
  | Odaiba | +0.0444 | +0.0001 |
  | Shinjuku | +0.0238 | -0.0650 |

  The Odaiba seed carried the drifted pre-gap estimate.
- **Determinism:** a debug rerun of the six v6 replays is identical (6/6).

## What this means

- The six-run development result (Go, 0 of 558) did not fully generalise.
- Under the 1 Hz-base condition the v4 post-gap rule leaves the SPP class
  uncovered.
- On one run, both configurations lose attitude, and v6 does not help.
- Per the contract, the default stays as it is.
- v6 remains opt-in, and its large fused-output improvements on 4 of 6 holdout
  run/scenarios are recorded here.
- A follow-up candidate would have to address both findings under a new
  contract. That contract would need data it was not designed on: these two
  runs are now development data.

## Limits

- **Population:** two runs, one receiver, one base at 1 Hz with 4-10 km
  baselines.
- **Truth reference point:** unknown.
- **Lever arm:** declared zero.
- **IMU-versus-truth time offset:** about 0.15 s, not corrected.
- **Absolute errors:** biased by the three items above, by up to about 1 m.
  Control and candidate share all of them.
- **Data access:** the UrbanNav data has no stated licence and is not
  redistributed. Converted-file SHA256 are in the converter manifests, which
  were produced by `scripts/convert_urbannav_to_ppc_layout.py` at the freeze
  commit.
