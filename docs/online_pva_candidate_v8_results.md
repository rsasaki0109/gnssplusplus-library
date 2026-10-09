# Velocity-consistency candidate v8 results: No-Go (19/558 PPC, 38/186 UrbanNav), closest yet

The frozen contract is [online_pva_candidate_v8.md](online_pva_candidate_v8.md).

- **Freeze:** commit `97e2380e`, before any comparison replay.
- **Contract SHA256:** `1408c66c0cdad273139c70404793cee759ef4173a5f5e7d58e28c19bc1df6159`.
- **Default and tuning:** the default is unchanged, and nothing was tuned after
  the results were seen.
- **Machine-readable record:**
  [online_pva_decision_v8.json](online_pva_decision_v8.json).

## Run note

An earlier attempt at the comparison was interrupted.

- Twenty-two candidate replays failed to start because the
  `gnss pva-evaluate` candidate list was temporarily switched away from the
  candidate tree. The replay binary itself was not affected.
- The whole 48-replay comparison was rerun from a worktree of the frozen
  commit, interleaved as specified. Only that rerun is reported. Nothing from
  the failed attempt is used.

## Decision

**No-Go.** The candidate fails 19 of 558 gates on PPC and 38 of 186 on
UrbanNav.

- **Replays:** 48 replays from one binary (SHA256 `d8fd7bbd...`, built from
  the frozen tree). All passed.
- **Gate 7:** control `none` is bit-identical to develop `c37979c3` on the 18
  PPC runs (175,902 rows).
- **Processor P95:** candidate/control is 1.07 / 1.35 / 1.82 (min / mean /
  max).

### Normal runs, control -> candidate

Values are RMSE / P95, except fused velocity, which is RMSE only.

| Run | RTK pos m | Fused pos m | Fused vel m/s | Rotation deg |
|---|---|---|---|---|
| Tokyo 1 | 32.2 / 61.7 -> 21.0 / 35.3 | 71.7 / 155.2 -> **12.7 / 33.6** | 7.05 -> **0.89** | 105.1 / 171.1 -> **1.9 / 2.5** |
| Tokyo 2 | 19.5 / 29.7 -> 6.3 / 10.6 | 74.2 / 179.5 -> **4.2 / 8.8** | 6.85 -> **0.51** | 100.3 / 170.0 -> **1.4 / 1.9** |
| Tokyo 3 | 18.6 / 31.9 -> 9.0 / 22.5 | 41.9 / 115.0 -> **5.9 / 14.7** | 3.22 -> **0.20** | 63.5 / 144.4 -> **1.6 / 2.4** |
| Nagoya 1 | 25.7 / 29.0 -> 21.2 / 27.5 | 49.3 / 118.1 -> **14.1 / 25.1** | 4.61 -> **1.41** | 88.2 / 168.7 -> **2.2 / 3.4** |
| Nagoya 2 | 37.7 / 81.8 -> 30.6 / 72.5 | 81.0 / 168.0 -> **21.4 / 53.3** | 3.73 -> **0.61** | 18.5 / 39.8 -> **2.8 / 5.3** |
| Nagoya 3 | 43.7 / 101.1 -> 36.0 / 99.2 | 58.9 / 128.1 -> **34.1 / 94.6** | 5.54 -> **0.30** | 105.0 / 170.9 -> **1.9 / 4.4** |
| Odaiba | 199.9 / 47.8 -> 199.1 / 27.9 | 81.2 / 157.1 -> **12.5 / 31.7** | 6.26 -> **1.13** | 105.6 / 171.4 -> 104.4 / 170.7 |
| Shinjuku | 27.6 / 65.6 -> 97.9 / 37.1 | 82.8 / 175.1 -> 231.4 / **24.3** | 9.27 -> **0.51** | 68.7 / 152.6 -> **12.6 / 30.0** |

**On PPC every rotation, position, fused-velocity and recovery gate passes in
all 18 run/scenarios.** Rotation is 1.4-2.8 deg RMSE on all six runs, against
18-105 deg for the control.

## Failed gates

The common-valid cohort fails the same gates as the all-output cohort.

- **RTK velocity P95.** PPC: Tokyo 1, Tokyo 3 (all scenarios), Tokyo 2 IMU gap,
  and Nagoya 1 (all scenarios).
  - The candidate's RTK velocity is the independent Doppler least-squares
    velocity on every epoch.
  - It is 3-31 % worse at P95 than the control's mixture: Doppler LS on SPP
    epochs, filter state on the 20 % differential epochs. For example,
    Tokyo 1 goes 3.29 -> 3.39 m/s.
- **Nagoya 2 first heading latch.** 41.8 -> 42.0 s in all three scenarios.
- **UrbanNav RTK availability.** 0.2-0.4 percentage points lower (gate 0.1) on
  every Odaiba and Shinjuku scenario.
- **Shinjuku (all scenarios).** The RTK output has rare very large errors:
  RMSE 98 m against P95 37 m.
  - The fused output follows a few of them: RMSE 231 m against P95 24 m.
  - It no longer diverges as it did in `rtk_online_product_v1`.
  - RTK velocity and the first heading latch also fail here.
- **Odaiba IMU gap.** RTK position P95 is 26.8 -> 27.9 m.
- **Odaiba normal and GNSS outage attitude.** It stays unusable (104 deg), as in
  every earlier candidate and the control.

## Reading

- **What the combination fixes.** The RTK input of `rtk_online_product_v1` and
  the fusion set of `velocity_consistency_v6` fix each other's failures on all
  PPC accuracy gates.
- **What remains:**
  1. **RTK velocity definition.** The Doppler LS velocity is slightly worse at
     P95 than the filter-state velocity it replaced on 20 % of control epochs.
  2. **Rare RTK position outliers on UrbanNav Shinjuku.** They pass the fused
     NIS gate often enough to move the fused RMSE. Odaiba shows the same
     pattern: RTK RMSE 199 m against P95 28-48 m in both arms.
  3. **A small RTK availability loss on UrbanNav.**
  4. **Odaiba attitude.**
- A follow-up needs a new frozen contract, and all eight runs are development
  data for it.
