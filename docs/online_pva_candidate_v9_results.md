# Velocity-consistency candidate v9 results: No-Go (6/558 PPC, 2/186 UrbanNav)

The frozen contract is [online_pva_candidate_v9.md](online_pva_candidate_v9.md).

- **Freeze:** commit `c06567c1`, together with the implementation, before any
  comparison replay.
- **Contract SHA256:** `8313064b98440acbcd69e2d35923071da73785bc1925b6a8821969c452c4381d`.
- **Default:** unchanged.
- **Tuning:** nothing was tuned after the results were seen.
- **Machine-readable record:**
  [online_pva_decision_v9.json](online_pva_decision_v9.json).

## Decision

**No-Go.** 6 of 558 PPC gates fail and 2 of 186 UrbanNav gates fail. This is
the fewest failures of any candidate so far.

- **Replays:** 48 replays from one binary (SHA256 `53135341...`, built from the
  frozen worktree). They were interleaved, and all passed.
- **Gate 7:** the control `none` is bit-identical to develop `c37979c3` on the
  18 PPC runs (175,902 rows).
- **Processor P95:** candidate / control is 0.99 / 1.31 / 1.88 (min / mean /
  max).

| Run/scenario | Failed gate (both cohorts) | Control | Candidate |
|---|---|---:|---:|
| Tokyo 1 | rotation RMSE | 105.1 deg | 106.3 deg |
| Tokyo 2 | rotation RMSE | 100.3 deg | 103.9 deg |
| Tokyo 2 GNSS outage | rotation P95 | 68.1 deg | 73.4 deg |
| Odaiba IMU gap | RTK position P95 | 26.8 m | 28.5 m |

## Normal runs: control / v7 / v8

RTK and fused values are RMSE / P95. Rotation is RMSE.

| Run | RTK pos m | RTK vel m/s | Fused pos m | Rotation deg |
|---|---|---|---|---|
| Tokyo 1 | 32.2/61.7 / 21.0/35.3 / 21.1/34.7 | 2.10/3.29 / 1.75/3.39 / **1.17/2.22** | 71.7/155.2 / 12.7/33.6 / **10.8/17.2** | 105.1 / 1.9 / **106.3** |
| Tokyo 2 | 19.5/29.7 / 6.3/10.6 / 6.3/10.6 | 2.26/3.71 / 1.08/2.20 / **0.75/1.59** | 74.2/179.5 / 4.2/8.8 / **3.6/7.5** | 100.3 / 1.4 / **103.9** |
| Tokyo 3 | 18.6/31.9 / 9.0/22.5 / 12.0/23.9 | 2.10/1.76 / 0.97/1.98 / **0.66/1.30** | 41.9/115.0 / 5.9/14.7 / 10.4/9.0 | 63.5 / 1.6 / 1.6 |
| Nagoya 1 | 25.7/29.0 / 21.2/27.5 / 20.9/26.7 | 2.51/4.26 / 2.02/4.55 / **1.28/2.36** | 49.3/118.1 / 14.1/25.1 / **12.5/11.3** | 88.2 / 2.2 / **1.8** |
| Nagoya 2 | 37.7/81.8 / 30.6/72.5 / 30.9/71.7 | 2.30/5.09 / 1.69/4.11 / **1.49/2.82** | 81.0/168.0 / 21.4/53.3 / **7.3/17.1** | 18.5 / 2.8 / 2.8 |
| Nagoya 3 | 43.7/101.1 / 36.0/99.2 / 37.0/99.5 | 2.47/5.13 / 1.62/3.78 / **1.50/3.31** | 58.9/128.1 / 34.1/94.6 / **14.2/19.9** | 105.0 / 1.9 / **1.5** |
| Odaiba | 199.9/47.8 / 199.1/27.9 / 199.0/28.3 | 3.84/9.31 / 2.20/4.96 / 2.32/5.00 | 81.2/157.1 / 12.5/31.7 / 19.6/57.2 | 105.6 / 104.4 / 105.6 |
| Shinjuku | 27.6/65.6 / 97.9/37.1 / **18.2/38.5** | 1.96/3.96 / 4.47/10.75 / **1.02/1.31** | 82.8/175.1 / 231.4/24.3 / **7.3/13.7** | 68.7 / 12.6 / **8.8** |

## Reading (post-hoc; not used to change the candidate)

### What worked

- **Shinjuku is resolved.** The 9.6 km base-seeded FLOAT is gone. RTK RMSE
  falls from 97.9 to 18.2 m, fused RMSE from 231 to 7.3 m, and rotation is
  8.8 deg.
- **RTK velocity.** Every RTK velocity P95 gate passes. The epoch SPP velocity
  is better than the control's on every run.
- **UrbanNav availability.** It passes on every scenario, including Odaiba,
  against the expectation stated in the contract.
- **Nagoya 2.** The first heading latch matches the control.
- **Fused position.** It improves further on five of eight runs.

### What regressed

- **Tokyo 1 and Tokyo 2 attitude.** It is lost, where v7 had 1.9 and 1.4 deg.
- **How it fails on Tokyo 1:**
  - The heading error is -16.7 deg at 16 s, -44 deg at 20 s, +94 deg at 60 s
    and +161 deg at 120 s. It keeps wandering for the rest of the run.
  - That is a drifting heading from the first latch on, not a 180 deg flip.
  - Fused position and velocity stay good meanwhile: fused velocity RMSE
    1.16 m/s.
- **Likely cause.** The only v8 change that touches the attitude path from the
  start is (n). With it, the epoch SPP velocity and its covariance become the
  fused and RTK-prior filters' velocity measurement. Isolating it needs a new
  diagnosis.

### Remaining failure

- **Odaiba IMU gap:** RTK P95 is 6 % worse (26.8 -> 28.5 m).

## Next

- A follow-up should isolate (n) on Tokyo 1 and Tokyo 2.
- It should then decide under a new frozen contract whether the SPP velocity
  should feed only the exported RTK velocity, which fixes the velocity gates,
  and not the fusion input.
- All eight runs are development data.
