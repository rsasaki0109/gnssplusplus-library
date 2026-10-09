# Online RTK with the product RTK configuration: No-Go, with a large RTK and fused-position gain

The frozen contract is [online_rtk_product_config_v1.md](online_rtk_product_config_v1.md).

- **Freeze:** `977b24b1`, together with the implementation, before any
  comparison replay.
- **Contract SHA256:** `25ba00c696f28b5d818e788e23830198cf70a0cf6c15e19b84ec59c05be548aa`.
- **Default:** unchanged. The candidate stays opt-in.
- **Tuning:** nothing was tuned after the results were seen.
- **Machine-readable record:**
  [online_rtk_product_config_decision_v1.json](online_rtk_product_config_decision_v1.json).

## Decision

**No-Go.** The candidate fails 39 of 558 gates on PPC (18 run/scenarios) and 58
of 186 on UrbanNav (6).

- **Replays:** 48 replays from one binary (SHA256 `f2d4a648...`), run
  interleaved on the quiet host. All passed with full truth matches.
- **Gate 7:** candidate `none` from this tree is bit-identical to develop
  `c37979c3` on all 18 PPC runs (175,902 rows).
- **Processor P95:** candidate/control is 1.04 / 1.37 / 1.76 (min / mean /
  max).

## Normal runs, control -> candidate (RMSE unless stated)

| Run | RTK pos m (RMSE / P95) | RTK FIXED epochs | RTK vel m/s | Fused pos m (RMSE / P95) | Fused vel m/s | Rotation deg |
|---|---|---|---|---|---|---|
| Tokyo 1 | 32.2 / 61.7 -> **21.0 / 35.3** | 15 -> 8,832 | 2.10 -> 1.75 | 71.7 / 155.2 -> **15.8 / 40.4** | 7.05 -> 3.20 | 105.1 -> 105.7 |
| Tokyo 2 | 19.5 / 29.7 -> **6.3 / 10.6** | 23 -> 7,085 | 2.26 -> 1.08 | 74.2 / 179.5 -> **10.2 / 10.2** | 6.85 -> 1.10 | 100.3 -> **7.0** |
| Tokyo 3 | 18.6 / 31.9 -> **9.0 / 22.5** | 36 -> 10,376 | 2.10 -> 0.97 | 41.9 / 115.0 -> **8.5 / 20.3** | 3.22 -> 1.24 | 63.5 -> **9.7** |
| Nagoya 1 | 25.7 / 29.0 -> 21.2 / 27.5 | 13 -> 4,493 | 2.51 -> 2.02 | 49.3 / 118.1 -> **13.8 / 32.6** | 4.61 -> 3.41 | 88.2 -> 105.1 |
| Nagoya 2 | 37.7 / 81.8 -> 30.6 / 72.5 | 49 -> 4,593 | 2.30 -> 1.69 | 81.0 / 168.0 -> **25.0 / 67.0** | 3.73 -> 2.99 | 18.5 -> 41.6 |
| Nagoya 3 | 43.7 / 101.1 -> 36.0 / 99.2 | 22 -> 2,937 | 2.47 -> 1.62 | 58.9 / 128.1 -> **35.4 / 96.0** | 5.54 -> 2.53 | 105.0 -> **17.0** |
| Odaiba | 199.9 / 47.8 -> 199.1 / 27.9 | 6 -> 3,715 | 3.84 -> 2.20 | 81.2 / 157.1 -> **22.2 / 58.4** | 6.26 -> 2.94 | 105.6 -> **35.7** |
| Shinjuku | 27.6 / 65.6 -> 97.9 / 37.1 | 47 -> 7,232 | 1.96 -> 4.47 | 82.8 / 175.1 -> **26,656 / 52,928 (diverged)** | 9.27 -> 1,441 | 68.7 -> 67.6 |

- **RTK.** The online RTK now fixes on 2,900-10,400 epochs per run, against
  13-49 for the control.
  - RTK position RMSE improves on all six PPC runs.
  - RTK velocity RMSE improves everywhere except Shinjuku.
- **Fused output.**
  - Fused position RMSE improves 2-8x on seven of eight runs.
  - Rotation improves strongly on Tokyo 2, Tokyo 3, Nagoya 3 and Odaiba.

## Failed gates

**PPC, all-output cohort.** The common-valid cohort fails the same gates.

| Run(s) | Failed gate | Control -> candidate |
|---|---|---|
| Tokyo 1, Tokyo 3 (all scenarios), Tokyo 2 IMU gap, Nagoya 1 (all scenarios) | RTK velocity P95 | 3-27 % worse, for example Tokyo 1 3.29 -> 3.39 m/s |
| Nagoya 1 (all three scenarios) | Rotation | 88 -> 105 deg RMSE |
| Nagoya 2 | Rotation | 18.5 -> 41.6 deg RMSE |
| Tokyo 2 IMU gap | Rotation | 102.7 -> 104.9 deg RMSE |
| Tokyo 3 GNSS outage | Fused velocity P95 | 1.40 -> 2.78 m/s |
| Nagoya 2 (all scenarios) | First heading latch | 41.8 -> 42.0 s |

Nagoya 1's rotation failure is the reverse start that `velocity_consistency_v5`
fixed. This candidate carries none of the v2-v6 fusion fixes.

**UrbanNav.**

- 50 of the 58 failures are Shinjuku: the fused filter diverges.
- The others are coverage, first-heading-latch and velocity gates.
- RTK availability drops by about 0.2-0.5 percentage points on both runs.
- The Shinjuku RTK position RMSE rises from 27.6 to 97.9 m, while the RTK P95
  falls from 65.6 to 37.1 m.

## Reading

- **The online RTK is no longer the bottleneck.** The main reason the online
  RTK lagged the batch product was the default RTK configuration, its static
  5 m fixed-jump limit. The product preset, a causal base hold and an
  independent Doppler velocity close most of that gap.
- **What remains is in the fusion layer.**
  - The rotation failures are those v5/v6 already address: Nagoya 1's reverse
    start, Nagoya 2's attitude, and the IMU-gap gyro bias.
  - The Shinjuku divergence shows that the fused filter has no protection
    against a burst of bad RTK input. Its RTK RMSE grew while its P95 shrank:
    a few epochs with very large errors.
- **Next candidate:** this RTK input combined with the v6 fusion set, plus a
  causal RTK output guard. That needs a new frozen contract. All eight runs are
  now development data for it.
