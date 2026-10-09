# Velocity-consistency candidate v7 results: Go (0 of 558 gates)

`velocity_consistency_v6` was evaluated against the frozen contract in
[online_pva_candidate_v7.md](online_pva_candidate_v7.md).

- **Freeze:** commit `62ffab3c`. The implementation is in `18cdaa5a`. Both came
  before any comparison replay.
- **Contract SHA256:** `fd481b42cf8a1c8dcb7e4deaea0c49086a05fa79b0e921a92b842a0880e3e50d`.
- **Defaults:** production defaults are unchanged. The candidate stays an
  explicit opt-in (`--candidate velocity_consistency_v6`).
- **Machine-readable record:**
  [online_pva_decision_v7.json](online_pva_decision_v7.json).

## Decision

**Go, 0 of 558 gates fail.** The comparison covers 18 run/scenarios and 36
replays. Every replay is truth-matched, and the input hashes are identical. The
targeted rotation improvement is present.

- **Build:** tree `62ffab3c`, clean. Release build, GCC 13.3.0. Replay binary
  SHA256 `9a1ecce1...`.
- **Control:** candidate `none` from the same binary.
- **Host:** the quiet 4-vCPU cloud host. At most 3 jobs ran at once, with each
  none job next to its v6 job in the queue. 1-minute load average: median 3.0,
  max 3.4.
- **Gate 4:** processor P95, candidate / control, min / mean / max is 0.947 /
  1.039 / 1.107.
- **Gate 7:** candidate `none` from this tree is bit-identical to candidate
  `none` from develop `c37979c3` in every field except `processing_ms`. That
  holds on all 18 runs, 175,902 rows.
- **Determinism:** a second v6 pass of the six IMU-gap scenarios with
  `LIBGNSS_DEBUG_HEADING=1` gives identical output (6/6). That pass is also the
  source of the seed table.

## Where v6 differs from v5

The predictions in the contract hold:

- **Normal and GNSS-outage scenarios:** bit-identical to v5 in all 12, in every
  field except `processing_ms`. No fused filter is recreated in them.
- **IMU-gap scenarios:** all six change from the post-gap initialization
  onward. Their RTK columns are bit-identical to v5, because the RTK-prior
  filter is not seeded.

IMU gap 60-64 s, v5 -> v6. Each cell is RMSE / P95. The control column is
candidate `none`.

| Run | Rotation deg | Fused pos m | Control rotation deg |
|---|---|---|---|
| Tokyo 1 | 1.8 -> 1.8 / 3.1 -> 3.1 | 15.0 -> 15.0 / 32.2 -> 32.2 | 104.9 / 171.3 |
| **Tokyo 2** | **18.4 -> 3.8 / 55.6 -> 3.3** | 11.8 -> 11.7 / 20.1 -> 20.2 | 102.7 / 170.6 |
| Tokyo 3 | 2.0 -> 1.7 / 3.3 -> 3.0 | 14.1 -> 14.1 / 26.7 -> 26.7 | 104.6 / 171.2 |
| Nagoya 1 | 2.3 -> 2.3 / 3.1 -> 3.5 | 13.2 -> 13.2 / 24.3 -> 24.4 | 83.0 / 167.4 |
| Nagoya 2 | 2.3 -> 2.6 / 4.6 -> 4.7 | 15.1 -> 15.1 / 32.7 -> 32.8 | 104.1 / 170.7 |
| Nagoya 3 | 2.8 -> 2.8 / 5.4 -> 5.4 | 32.8 -> 32.8 / 61.4 -> 61.4 | 106.7 / 170.3 |

Tokyo 2 IMU gap, rotation error (deg), v5 -> v6:

| t (s) | 76 | 86 | 100 | 110 | 130 | 165 | 230 | 290 |
|---|---|---|---|---|---|---|---|---|
| v5 -> v6 | 3.1 -> 1.3 | 64.6 -> 13.9 | 15.0 -> 23.4 | 27.0 -> 33.8 | 59.7 -> 1.0 | 82.0 -> 1.9 | 37.1 -> 1.6 | 4.6 -> 0.2 |

- The long 104-300 s drift that carried 93% of the v5 error is gone.
- A shorter excursion remains around 95-115 s, with a maximum of 35.0 deg after
  66 s. The pre-freeze prefix and counterfactual X1 had already shown it.
- Nagoya 2 is slightly worse, as predicted, because its window mean was closer
  to the true bias. Nagoya 1's rotation P95 is 0.4 deg worse.
- Both are far inside the control gates.

## Seed at every seeded initialization (gyro bias z, rad/s)

| Run (IMU gap) | Init tow | Seed (pre-gap) | Window mean (replaced) |
|---|---:|---:|---:|
| Tokyo 1 | 187536 | +0.0017 | +0.0010 |
| Tokyo 2 | 177066 | -0.0114 | +0.0875 |
| Tokyo 3 | 179526 | +0.0027 | +0.0355 |
| Nagoya 1 | 550446 | +0.0105 | -0.0226 |
| Nagoya 2 | 555786 | -0.0168 | +0.0012 |
| Nagoya 3 | 553866 | -0.0019 | -0.0001 |

- Exactly one seeded initialization happens per IMU-gap replay.
- None happens in the normal or GNSS-outage replays.

## Validation

- `run_tests`: 1,331 pass, 0 failed. This includes the new
  `FusionProcessorSyntheticTest` gyro-seed cases: OFF default, the seed
  replaces only the gyro bias, and the seed is consumed.
- `gnss_online_tests`: 24/24. This includes the new carry cases: OFF default,
  carry on imu_gap, imu_stale and rover_gap, no seed from an uninitialized
  filter, and the RTK-prior filter not seeded.
- `tests/test_pva_comparison.py` and `tests/test_pva.py` pass.
- The broader `ctest` lanes that need matplotlib, mkdocs or an installed Python
  package were not run.

## Limits

- The six runs are development data, all used in the diagnosis and the
  comparison. There is no holdout.
- The IMU gap is synthetic: 4 s, the same sensor before and after. A real
  outage with a power cycle or a temperature change could change the bias. The
  seed keeps the initial bias sigma, so the filter can move it, but no such
  case was tested.
- A second reset before the new filter initializes loses the seed, by design.
  This was not observed.
- The 95-115 s excursion in Tokyo 2 (up to 35 deg) is not explained. It is
  unchanged in kind from counterfactual X1, where gyro bias wanders while
  turning before and after the latch.
