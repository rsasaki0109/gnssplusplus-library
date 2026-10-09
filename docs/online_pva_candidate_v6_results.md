# Velocity-consistency candidate v6 results: Go (0 of 558 gates)

`velocity_consistency_v5` was evaluated against the frozen contract in
[online_pva_candidate_v6.md](online_pva_candidate_v6.md).

- Freeze commit `1c7aff4a`, committed with the implementation before any
  comparison replay.
- Contract SHA256 `d45c575019333b2918482e307a64950b3d60f3aff03d6918620ce8bb73dd79a2`.
- Production defaults are unchanged; the candidate stays an explicit opt-in
  (`--candidate velocity_consistency_v5`).
- Machine-readable record:
  [online_pva_decision_v6.json](online_pva_decision_v6.json).

## Decision

**Go, 0 of 558 gates fail.** The comparison covers 18 run/scenarios and 36
replays. Every replay is truth-matched with identical input hashes, and the
targeted rotation improvement is present.

- **Tree:** `1c7aff4a`, clean. Release build, GCC 13.3.0. Replay binary
  SHA256 `c11a3d96...`.
- **Control:** candidate `none` from the same binary.
- **Host:** the same quiet 4-vCPU cloud host as the v5 timing
  re-measurement. At most 3 replays ran at once, with none/v5 adjacent in the
  queue. Load average median 2.91, max 3.0.
- **Gate 4:** processor P95, candidate / control, is 0.979 / 1.055 / 1.163
  (min / mean / max).
- **Gate 7:** candidate `none` from this tree is bit-identical to candidate
  `none` from develop `c37979c3` in every field except `processing_ms`. That
  holds on all 18 runs, 175,902 rows.
- **Determinism:** a second v5 pass with `LIBGNSS_DEBUG_HEADING=1`, used for
  the latch table below, gives identical output on 18/18.

## Where v5 differs from v4

- **Only Nagoya 1 changes.** In the other five runs, every field except
  `processing_ms` is bit-identical to v4 in all three scenarios. Their first
  latch has a valid `v_long >= 0`, and they have no rover gap > 2 s.
- **RTK columns are bit-identical to v4 in all 18 run/scenarios,** as the
  contract expected. (i) recreates the RTK side exactly as before, and (h) is
  not applied to the RTK-prior filter.
- **Nagoya 1 first differs at the 52.0 s latch.**

Nagoya 1, v4 -> v5. Fused position and rotation figures are RMSE / P95.

| Scenario | Fused pos m | Fused vel RMSE m/s | Rotation deg |
|---|---|---|---|
| normal | 13.1 -> 13.2 / 24.3 -> 24.7 | 1.02 -> 0.93 | 31.1 -> **3.7** / 61.3 -> **5.1** |
| GNSS outage 60-70 s | 13.4 -> 13.2 / 24.5 -> 24.6 | 1.13 -> 0.93 | 45.8 -> **3.0** / 153.3 -> **4.1** |
| IMU gap 60-64 s | 13.1 -> 13.2 / 24.3 -> 24.3 | 0.96 -> 0.92 | 14.1 -> **2.3** / 16.5 -> **3.1** |

(Control `none`: rotation 88.2 / 88.7 / 83.0 deg RMSE.)

Nagoya 1 attitude is now on par with the other runs (1.7-2.9 deg RMSE).
Fused position RMSE/P95 is 0.1-0.4 m worse than v4 in the normal run. It
stays far below the control (49.3 / 118.1 m), and the gates compare with the
control.

Nagoya 1 normal, rotation RMSE per 60 s bin (deg), v4 -> v5:

| 0-60 s | 60-120 s | 120-180 s | 180-240 s | 240-300 s | 300-360 s | 360-420 s | 420-480 s |
|---|---|---|---|---|---|---|---|
| 156.4 -> 18.8 | 128.0 -> 14.9 | 22.7 -> 4.2 | 52.4 -> 4.4 | 30.6 -> 2.4 | 13.0 -> 1.1 | 4.6 -> 1.0 | 9.2 -> 0.8 |

- After 360 s, no 60 s bin exceeds 1.8 deg.
- The first two bins include the yaw-unaligned epochs before the 52.0 s
  latch.
- GNSS outage, fused position error at 69.8 / 70.0 / 70.2 / 72 / 80 / 100 s:
  - v4: 34.6 / 0.3 / 1.2 / 2.2 / 3.1 / 2.8 m
  - v5: 15.8 / 0.3 / 0.4 / 0.7 / 0.2 / 0.6 m

## Latch direction test at every fused-filter latch

| Replay(s) | Latch tow | v_long m/s | valid | flipped |
|---|---:|---:|---|---|
| Nagoya 1 (all three scenarios) | 550432 | -0.61 | yes | **yes** (course -2.8 -> 177.2 deg) |
| Nagoya 1 IMU gap, re-latch | 550446 | -0.10 | no | no |
| Nagoya 2 (all three) | 555762 | +0.44 | yes | no |
| Nagoya 2 IMU gap, re-latch | 555786 | +0.23 | no | no |
| Nagoya 3 (all three) | 553836 | +1.11 | yes | no |
| Nagoya 3 IMU gap, re-latch | 553866 | +0.01 | no | no |
| Tokyo 1 (all three) | 187485 | +0.69 | yes | no |
| Tokyo 1 IMU gap, re-latch | 187553 | +0.53 | yes (stopped after the reset) | no |
| Tokyo 2 (all three) | 177023 | +1.18 | yes | no |
| Tokyo 2 IMU gap, re-latch | 177075 | +0.40 | no | no |
| Tokyo 3 (all three) | 179498 | +1.07 | yes | no |
| Tokyo 3 IMU gap, re-latch | 179526 | +0.01 | no | no |

After a reset that happens while moving, the test is skipped as designed
(`valid` no) until the vehicle stops. The smallest valid margin on a forward
latch is +0.44 m/s (Nagoya 2). Valid `|v_long|` at the first latch is
0.44-1.18 m/s, against GNSS speeds of about 1.0-1.4 m/s. Part of the speed
is lost to bias and tilt errors, which is consistent with the error budget
in the contract.

`rover_gap_rtk_reset`: exactly one per Nagoya 1 replay, at 227.2 s (all three
scenarios). There is none in the other runs, and `reset_generation` does not
advance for it. The IMU-gap scenarios still have their `imu_stale_reset` at
60.2 s, as designed.

## Validation

- **Unit tests:** `run_tests` 1,386 cases: 1,328 pass, 58 skipped (missing
  fixtures), 0 failed. This includes the new `FusionProcessorSyntheticTest`
  direction-test cases: OFF default, reverse flips, forward does not, no
  stationary sample means no flip, tilted mount.
- **Online tests:** `gnss_online_tests` `OnlineRtkImuTest.*` 17/17. This
  includes the new rover-gap cases: OFF default, RTK-only reset keeps the
  fused filter, OFF gives the old reset, an IMU gap still resets everything.
- **Python:** `tests/test_pva_comparison.py` passes.
- **Not run:** the broader `ctest` lanes needing matplotlib, mkdocs or an
  installed Python package.

## Limits

- Six development runs, all used in the diagnosis and in the comparison. There
  is no holdout. Only one run (Nagoya 1) exercises either change. The flip
  fired once and the rover-gap path once, so the benefit rests on a single
  event of each kind.
- The direction test is a sign test with a 0.44 m/s smallest observed margin.
  A reverse start after a long moving interval with no stationary sample is
  not tested (`valid` false). Such a start keeps the v4 behaviour.
- Tokyo 2 IMU-gap rotation (18.4 deg) is unchanged. Its cause is the static
  re-alignment while turning after the IMU reset, which is a separate
  candidate.
- FLOAT covariance optimism after reconvergence, the Tokyo 2 post-outage
  ~30 s, is unchanged from v4.
