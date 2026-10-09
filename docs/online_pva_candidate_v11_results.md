# Velocity-consistency candidate v11 results: Go (PPC 0/576, UrbanNav 0/192)

The frozen contract is [online_pva_candidate_v11.md](online_pva_candidate_v11.md).

- **Freeze:** commit `6f2fab94`, before any comparison replay.
- **Contract SHA256:** `f685b873c2d34e4f0d77c7962063cea1ff63b61511c3f620fee6c06104eb140c`.
- **Default:** unchanged. Go permits a separate, reviewed default-switch
  proposal.
- **Tuning:** nothing was tuned after the results were seen.
- **Machine-readable record:**
  [online_pva_decision_v11.json](online_pva_decision_v11.json).

## Decision

**Go.** Every gate passes on all 24 run/scenarios.

- **PPC:** 0 of 576 gates fail. That is the 558 earlier gates plus gate 8 on
  18 run/scenarios.
- **UrbanNav:** 0 of 192 gates fail.
- **Targeted rotation improvement:** present in both comparator outputs.
- **Gate 8 (attitude integrity, absolute):** the candidate has no epoch with
  rotation error above 90 deg in any run/scenario. The largest rotation error
  anywhere is 38.5 deg (Shinjuku IMU gap). The control fraction above 90 deg is
  0-55 %, depending on the run/scenario.
- **Gate 7:** control `none` is bit-identical to develop `a7a5d5bf` on the 18 PPC
  run/scenarios: 175,902 rows, every deterministic CSV field.

Run details:

- **Replays:** 48 replays from one binary (SHA256 `854bdd42...`, built from the
  frozen worktree), interleaved control/candidate, at most 3 at once. All
  passed and none were rerun.
- **Processor P95, candidate/control:** min 1.11, mean 1.36, max 1.92. The
  limit is 2.
- **UrbanNav inputs** are read with the RINEX 3.00-3.02 BeiDou B1I fix (#577).
  UrbanNav control values therefore differ from the records of v10 and earlier.

## Normal runs, control -> candidate

Values are RMSE / P95, except fused velocity, which is RMSE.

| Run | RTK pos m | RTK vel m/s | Fused pos m | Fused vel m/s | Rotation deg |
|---|---|---|---|---|---|
| Tokyo 1 | 32.2/61.7 -> 21.1/34.7 | 2.10/3.29 -> 1.17/2.22 | 71.7/155.2 -> **4.0/6.2** | 7.05 -> **0.18** | 105.1/171.1 -> **1.7/2.4** |
| Tokyo 2 | 19.5/29.7 -> 6.3/10.7 | 2.26/3.71 -> 0.75/1.59 | 74.2/179.5 -> **2.4/5.1** | 6.85 -> **0.10** | 100.3/170.0 -> **1.4/1.9** |
| Tokyo 3 | 18.6/31.9 -> 12.0/24.0 | 2.10/1.76 -> 0.66/1.30 | 41.9/115.0 -> **10.4/9.0** | 3.22 -> **0.15** | 63.5/144.4 -> **1.6/2.6** |
| Nagoya 1 | 25.7/29.0 -> 20.9/26.7 | 2.51/4.26 -> 1.28/2.36 | 49.3/118.2 -> **12.5/11.3** | 4.61 -> **0.88** | 88.2/168.7 -> **2.9/3.0** |
| Nagoya 2 | 37.7/81.8 -> 30.9/71.7 | 2.30/5.09 -> 1.49/2.82 | 81.0/168.0 -> **7.3/17.1** | 3.73 -> **0.33** | 18.5/39.8 -> **2.1/4.3** |
| Nagoya 3 | 43.7/101.1 -> 37.0/99.5 | 2.47/5.13 -> 1.50/3.31 | 58.9/128.1 -> **13.6/19.9** | 5.54 -> **0.20** | 105.0/170.9 -> **1.8/2.8** |
| Odaiba | 199.4/27.7 -> 199.0/26.7 | 3.06/7.17 -> 0.38/0.51 | 82.7/211.7 -> **11.2/12.6** | 6.96 -> **0.21** | 105.7/171.8 -> **2.1/3.3** |
| Shinjuku | 27.0/64.9 -> 16.6/31.7 | 2.17/4.99 -> 0.76/0.75 | 136.3/323.8 -> **7.0/11.2** | 24.41 -> **0.28** | 99.7/170.5 -> **2.2/3.7** |

## Against velocity_consistency_v9 (rotation, normal scenario)

- **PPC** (RINEX 3.04, comparable):
  - Tokyo 1-3 and Nagoya 2-3 are equal or better.
  - Nagoya 1 is worse, 1.37 -> 2.90 deg RMSE. The contract predicted this.
    The cost sits in the first ~100 s, and the run stays far below the
    control's 88 deg.
- **UrbanNav:** the earlier v9 records were made before the reader fix, so
  they are not comparable. On the same fixed input, v9 lost the attitude on
  Odaiba and Shinjuku (about 105 deg RMSE). v10 holds it at 2.1 and 2.2 deg.

## Remaining limits

- **Development data only.** All eight runs are development data, and the
  diagnosis ran full counterfactuals on them.
- **Odaiba RTK position RMSE** is about 199 m in both arms. The cause is a few
  very large RTK outliers, which this candidate does not address.
- **A default switch** needs its own reviewed proposal. The proposal should
  include one run on data never used in development. The Hong Kong conversion
  prepared for holdout v2 has not been run with any estimator.
