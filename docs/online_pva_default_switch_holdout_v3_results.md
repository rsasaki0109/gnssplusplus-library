# Holdout contract v3 results: `velocity_consistency_v10` on UrbanNav Hong Kong

Contract: [online_pva_default_switch_holdout_v3.md](online_pva_default_switch_holdout_v3.md)
(frozen at commit `a23e8311`). Decision file:
[online_pva_decision_holdout_v3.json](online_pva_decision_holdout_v3.json), as
produced by the frozen comparator.

## Decision: No-Go

**4 of 84 gates fail** (2 on each run). Per the contract, the default stays
`none`, the evaluation of `velocity_consistency_v10` on this data ends, and the
result is not rerun. Nothing was changed after seeing the result: candidate,
conversion, configuration, comparator, thresholds and binary are those of the
freeze. The decision file reads `state: passed`, `adoption: No-Go`,
`default_changed: false`, `failed_gates: 4`, `gates: 84`, `runs: 2`.

Failed gates:

| Run | Gate | Control | Candidate | Threshold |
|---|---|---:|---:|---|
| `HKDeepUrban1_novatel` | H4 `rtk_available` (pooled coverage) | 0.831546 | 0.822833 | cand >= ctl - 0.005 (delta -0.0087) |
| `HKDeepUrban1_novatel` | H4 `rtk_velocity_available` (pooled coverage) | 0.829535 | 0.822833 | cand >= ctl - 0.005 (delta -0.0067) |
| `HKHarshUrban1_novatel` | H5 `normal.initial.first_heading_s` | 2.0 s | 22.0 s | cand <= ctl + 1.0 s (delta +20.0 s) |
| `HKHarshUrban1_novatel` | H2 `all.rtk_position_m.rmse` (all-output cohort) | 747.609 m | 1094.983 m | cand <= 1.10 x ctl (ratio 1.4647) |

All H1 (primary), H3 (tail), H6 (processor), H7 (integrity) and H8 (attitude
integrity) gates pass on both runs; the other 80 of the 84 gates pass (40 of 42
on each run).

## Executed procedure

- **Date and tree.** 2026-10-10; repository HEAD `a23e8311` (the freeze commit),
  clean tracked tree, branch `claude/eager-gauss-kckwh1`. Each replay manifest
  records `repository.commit` `a23e83111a93...` and an empty tracked diff.
- **Frozen-file check before starting.** The SHA256 of the ten frozen files of
  the contract's table and of the ten converted data files and two manifests
  of the contract's data-fingerprint table were recomputed and matched.
- **Binary.** `gnss_pva_replay`, SHA256
  `440b24ce68c15f488dd80ae959b4d87a2db4fe3db5d764df3358899df4906fa1`
  (verified equal to the contract before the first replay; not rebuilt). All
  12 replay manifests record this SHA256, and so does the decision file
  (`binary_sha256`).
- **Replays.** 12 full runs (`--max-epochs 0`): 6 `--candidate none` and 6
  `--candidate velocity_consistency_v10`, via `python3 apps/gnss.py pva-evaluate
  --run-dir <root>/urbannav/<Run>_novatel --replay-binary <binary> --candidate
  <arm> --output-dir <out> --scenario <s> [--start-s 60 --duration-s 10 (gnss_outage) |
  --start-s 60 --duration-s 4 (imu_gap)] --max-epochs 0`. Driven by `xargs -P 3`
  over a job list interleaved control, candidate for each run and scenario (Deep
  normal, Deep gnss_outage, Deep imu_gap, Harsh normal, Harsh gnss_outage, Harsh
  imu_gap), at most 3 concurrent. All 12 exited with status 0. Start and end
  times and commands are in `driver.log` (scratch). The 12 replays ran from
  06:09:55 to 06:10:27 UTC (32 s in total); each replay's `wall_s` was 5.0 to
  6.1 s (control) and 7.2 to 8.9 s (candidate).
- **Epochs.** 1,492 per Deep replay, 2,269 per Harsh replay; match fraction 1.0
  in every replay; `reference_used_for_estimation: false`.
- **Comparator** (`scripts/analysis/compare_online_pva.py`, SHA256 `5b3b54e8...`
  as frozen), run once, exit status 0:

  ```
  python3 scripts/analysis/compare_online_pva.py --gate-set holdout_v3 \
      --baseline-dir <none>/normal --candidate-dir <cand>/normal \
      --baseline-scenario-dir <none>/scenarios --candidate-scenario-dir <cand>/scenarios \
      --runs HKDeepUrban1_novatel HKHarshUrban1_novatel --output-dir <decision>
  ```

  The decision JSON is stored unchanged as `docs/online_pva_decision_holdout_v3.json`
  (43,125 bytes, SHA256
  `515984b548424677bb128bd5b75ed814860ad360abfe74003b1524c4110d54ab`). It embeds
  the absolute scratch paths of the replay manifests, with their hashes. The
  contract path and hash recorded in it: `docs/online_pva_default_switch_holdout_v3.md`,
  SHA256 `bb488787443d11e9d7b6c87cae607df3f5376b7ffa9c7eff7a8ecc2ac5f4f7ce`.
- **Replay outputs** (`errors.csv`, `score.json`, `manifest.json`, `replay/pva.csv`,
  `replay/replay.json`) are in the session scratch directory, not in the
  repository, under
  `.../scratchpad/holdout_v3_run/{none,cand}/{normal/<Run>_novatel,scenarios/<Run>_novatel-<scenario>}`.
  The decision JSON pins each by SHA256.

## Host load

| Time (UTC) | `uptime` load average (1, 5, 15 min) | Note |
|---|---|---|
| 06:09:34 (before the checks) | 0.13, 0.98, 0.94 | the only other process above 1 % CPU was the agent harness (3 %) |
| 06:09:55 (immediately before the first replay) | 0.09, 0.92, 0.92 | |
| 06:10:27 (immediately after the last replay) | 1.16, 1.10, 0.98 | |

The host has 4 vCPUs (Intel Xeon 2.1 GHz, 15 GB RAM, Linux 6.18.44), a
Firecracker VM. Nothing else heavy was running. At most 3 replays ran at once.
The H6 processor ratios are 1.17 to 1.44 (below), well under 2, so no H6 repeat
was needed or run. No gate was repeated.

## H7: integrity

**Passed.** The comparator produced a gate table (a failed H7 returns No-Go with
no gate table). All 12 replays are present, passed and full, with `errors.csv`,
`score.json` and native CSV hashes equal to their manifests. Both arms have the
same raw input hashes, scenario, window, epoch count, start time, base position,
lever arm (0, 0, 0), navigation policy and timestamps, and match fraction 1. The
control is candidate `none` and the candidate is `velocity_consistency_v10`. One
binary SHA256 is recorded by all replays. (The comparator checks these
conditions itself; this list restates the contract's H7 rules, and the
outcome is that none was violated. Start times: GPS TOW 455346 s for Deep and
184488 s for Harsh in all six replays of each run.)

## H8: attitude integrity (fraction of scored epochs with `rotation_deg` > 90 deg)

Absolute gate on the candidate (<= 0.01); the control's fraction is information
only. **All 6 candidate replays pass with 0 epochs above 90 deg** (no scored epoch of any candidate replay is above 90 deg).

| Run | Scenario | Control fraction (epochs) | Candidate fraction (epochs) | Candidate <= 0.01 |
|---|---|---:|---:|---|
| `HKDeepUrban1_novatel` | normal | 0.427820 (569/1330) | 0 (0/1484) | pass |
| `HKDeepUrban1_novatel` | gnss_outage | 0.496241 (660/1330) | 0 (0/1484) | pass |
| `HKDeepUrban1_novatel` | imu_gap | 0.441950 (571/1292) | 0 (0/1446) | pass |
| `HKHarshUrban1_novatel` | normal | 0.549799 (1093/1988) | 0 (0/2247) | pass |
| `HKHarshUrban1_novatel` | gnss_outage | 0.596076 (1185/1988) | 0 (0/2247) | pass |
| `HKHarshUrban1_novatel` | imu_gap | 0.499748 (990/1981) | 0 (0/2240) | pass |

The control loses attitude by more than 90 deg in 43 to 60 % of its scored
epochs on this data; the candidate in none of them.

## Gate tables (42 gates per run)

Ratio is candidate / control for H1, H2, H3 and H6; delta is candidate minus
control for H4 (coverage) and H5 (seconds); H8 is absolute. The pooled values
are those of the decision file: the three scenario replays of a run pooled (4,476
epochs for Deep, 6,807 for Harsh), "all" is the all-output cohort and "common"
the common-valid cohort. An H5 value of `null` means never recovered.

### HKDeepUrban1_novatel: 40 of 42 gates pass

| Hyp | Gate (statistic) | Control | Candidate | Ratio / delta | Threshold | Pass |
|---|---|---:|---:|---:|---|---|
| H1 | `H1.all.fused_position_m.rmse` | 2246.15 | 316.824 | 0.1411x | cand <= 1.00 * ctl | pass |
| H1 | `H1.all.fused_position_m.p95` | 5476.18 | 425.291 | 0.0777x | cand <= 1.00 * ctl | pass |
| H1 | `H1.all.rotation_deg.rmse` | 99.4153 | 4.26928 | 0.0429x | cand <= 1.00 * ctl | pass |
| H1 | `H1.all.rotation_deg.p95` | 170.075 | 7.58766 | 0.0446x | cand <= 1.00 * ctl | pass |
| H1 | `H1.common.fused_position_m.rmse` | 2246.15 | 108.095 | 0.0481x | cand <= 1.00 * ctl | pass |
| H1 | `H1.common.fused_position_m.p95` | 5476.18 | 65.1418 | 0.0119x | cand <= 1.00 * ctl | pass |
| H1 | `H1.common.rotation_deg.rmse` | 99.4153 | 4.1122 | 0.0414x | cand <= 1.00 * ctl | pass |
| H1 | `H1.common.rotation_deg.p95` | 170.075 | 7.71412 | 0.0454x | cand <= 1.00 * ctl | pass |
| H2 | `H2.all.rtk_position_m.rmse` | 765.587 | 34.0268 | 0.0444x | cand <= 1.10 * ctl | pass |
| H2 | `H2.all.rtk_position_m.p95` | 107.982 | 36.2624 | 0.3358x | cand <= 1.10 * ctl | pass |
| H2 | `H2.all.rtk_velocity_mps.rmse` | 24.767 | 1.08059 | 0.0436x | cand <= 1.10 * ctl | pass |
| H2 | `H2.all.rtk_velocity_mps.p95` | 43.2947 | 1.36482 | 0.0315x | cand <= 1.10 * ctl | pass |
| H2 | `H2.all.fused_velocity_mps.rmse` | 221.985 | 13.4995 | 0.0608x | cand <= 1.10 * ctl | pass |
| H2 | `H2.all.fused_velocity_mps.p95` | 174.39 | 41.1583 | 0.2360x | cand <= 1.10 * ctl | pass |
| H2 | `H2.common.rtk_position_m.rmse` | 41.6951 | 34.0649 | 0.8170x | cand <= 1.10 * ctl | pass |
| H2 | `H2.common.rtk_position_m.p95` | 63.7525 | 36.0305 | 0.5652x | cand <= 1.10 * ctl | pass |
| H2 | `H2.common.rtk_velocity_mps.rmse` | 24.9792 | 0.917892 | 0.0367x | cand <= 1.10 * ctl | pass |
| H2 | `H2.common.rtk_velocity_mps.p95` | 43.3791 | 1.14016 | 0.0263x | cand <= 1.10 * ctl | pass |
| H2 | `H2.common.fused_velocity_mps.rmse` | 221.985 | 11.7932 | 0.0531x | cand <= 1.10 * ctl | pass |
| H2 | `H2.common.fused_velocity_mps.p95` | 174.39 | 16.1198 | 0.0924x | cand <= 1.10 * ctl | pass |
| H3 | `H3.all.fused_position_m.p99` | 10213.7 | 1897.09 | 0.1857x | cand <= 1.25 * ctl | pass |
| H3 | `H3.all.rotation_deg.p99` | 178.697 | 12.3121 | 0.0689x | cand <= 1.25 * ctl | pass |
| H4 | `H4.coverage.rtk_available` | 0.831546 | 0.822833 | -0.0087 | cand >= ctl - 0.005 | **FAIL** |
| H4 | `H4.coverage.fused_available` | 0.917113 | 0.998883 | +0.0818 | cand >= ctl - 0.005 | pass |
| H4 | `H4.coverage.rtk_velocity_available` | 0.829535 | 0.822833 | -0.0067 | cand >= ctl - 0.005 | **FAIL** |
| H4 | `H4.coverage.fused_velocity_available` | 0.917113 | 0.998883 | +0.0818 | cand >= ctl - 0.005 | pass |
| H4 | `H4.coverage.attitude_available` | 0.917113 | 0.998883 | +0.0818 | cand >= ctl - 0.005 | pass |
| H4 | `H4.coverage.heading_available` | 0.882931 | 0.986148 | +0.1032 | cand >= ctl - 0.005 | pass |
| H5 | `normal.initial.first_fresh_s` | 0 | 0 | +0.0000 | cand <= ctl + 1.0 s; a null cand fails if ctl is non-null | pass |
| H5 | `normal.initial.first_heading_s` | 8 | 8 | +0.0000 | cand <= ctl + 1.0 s; a null cand fails if ctl is non-null | pass |
| H5 | `gnss_outage.scenario.recovery_gnss_update_s` | 0 | 0 | +0.0000 | cand <= ctl + 1.0 s; a null cand fails if ctl is non-null | pass |
| H5 | `gnss_outage.scenario.recovery_fresh_attitude_s` | 0 | 0 | +0.0000 | cand <= ctl + 1.0 s; a null cand fails if ctl is non-null | pass |
| H5 | `gnss_outage.scenario.recovery_heading_s` | 0 | 0 | +0.0000 | cand <= ctl + 1.0 s; a null cand fails if ctl is non-null | pass |
| H5 | `imu_gap.scenario.recovery_gnss_update_s` | 2 | 2 | +0.0000 | cand <= ctl + 1.0 s; a null cand fails if ctl is non-null | pass |
| H5 | `imu_gap.scenario.recovery_fresh_attitude_s` | 2 | 2 | +0.0000 | cand <= ctl + 1.0 s; a null cand fails if ctl is non-null | pass |
| H5 | `imu_gap.scenario.recovery_heading_s` | 35 | 35 | +0.0000 | cand <= ctl + 1.0 s; a null cand fails if ctl is non-null | pass |
| H6 | `normal.processing.p95_ms` | 6.34112 | 8.05113 | 1.2697x | cand <= 2 * ctl; host contention | pass |
| H6 | `gnss_outage.processing.p95_ms` | 6.10516 | 7.18338 | 1.1766x | cand <= 2 * ctl; host contention | pass |
| H6 | `imu_gap.processing.p95_ms` | 6.08143 | 7.08781 | 1.1655x | cand <= 2 * ctl; host contention | pass |
| H8 | `H8.normal.attitude_integrity.rotation_gt_90deg_fraction` | 0.42782 | 0 | absolute | cand fraction of scored epochs with rotation_deg > 90 <= 0.01 (absolute; ctl is information only) | pass |
| H8 | `H8.gnss_outage.attitude_integrity.rotation_gt_90deg_fraction` | 0.496241 | 0 | absolute | cand fraction of scored epochs with rotation_deg > 90 <= 0.01 (absolute; ctl is information only) | pass |
| H8 | `H8.imu_gap.attitude_integrity.rotation_gt_90deg_fraction` | 0.44195 | 0 | absolute | cand fraction of scored epochs with rotation_deg > 90 <= 0.01 (absolute; ctl is information only) | pass |

### HKHarshUrban1_novatel: 40 of 42 gates pass

| Hyp | Gate (statistic) | Control | Candidate | Ratio / delta | Threshold | Pass |
|---|---|---:|---:|---:|---|---|
| H1 | `H1.all.fused_position_m.rmse` | 5978.49 | 223.116 | 0.0373x | cand <= 1.00 * ctl | pass |
| H1 | `H1.all.fused_position_m.p95` | 4313.41 | 384.345 | 0.0891x | cand <= 1.00 * ctl | pass |
| H1 | `H1.all.rotation_deg.rmse` | 108.393 | 7.79808 | 0.0719x | cand <= 1.00 * ctl | pass |
| H1 | `H1.all.rotation_deg.p95` | 171.608 | 15.888 | 0.0926x | cand <= 1.00 * ctl | pass |
| H1 | `H1.common.fused_position_m.rmse` | 5978.49 | 203.684 | 0.0341x | cand <= 1.00 * ctl | pass |
| H1 | `H1.common.fused_position_m.p95` | 4313.41 | 173.176 | 0.0401x | cand <= 1.00 * ctl | pass |
| H1 | `H1.common.rotation_deg.rmse` | 108.345 | 7.91318 | 0.0730x | cand <= 1.00 * ctl | pass |
| H1 | `H1.common.rotation_deg.p95` | 171.531 | 16.1137 | 0.0939x | cand <= 1.00 * ctl | pass |
| H2 | `H2.all.rtk_position_m.rmse` | 747.609 | 1094.98 | 1.4646x | cand <= 1.10 * ctl | **FAIL** |
| H2 | `H2.all.rtk_position_m.p95` | 187.83 | 134.327 | 0.7152x | cand <= 1.10 * ctl | pass |
| H2 | `H2.all.rtk_velocity_mps.rmse` | 10.7823 | 0.930056 | 0.0863x | cand <= 1.10 * ctl | pass |
| H2 | `H2.all.rtk_velocity_mps.p95` | 20.119 | 1.43977 | 0.0716x | cand <= 1.10 * ctl | pass |
| H2 | `H2.all.fused_velocity_mps.rmse` | 274.557 | 5.89097 | 0.0215x | cand <= 1.10 * ctl | pass |
| H2 | `H2.all.fused_velocity_mps.p95` | 480.04 | 10.1231 | 0.0211x | cand <= 1.10 * ctl | pass |
| H2 | `H2.common.rtk_position_m.rmse` | 700.33 | 700.224 | 0.9998x | cand <= 1.10 * ctl | pass |
| H2 | `H2.common.rtk_position_m.p95` | 151.694 | 133.921 | 0.8828x | cand <= 1.10 * ctl | pass |
| H2 | `H2.common.rtk_velocity_mps.rmse` | 10.821 | 0.813627 | 0.0752x | cand <= 1.10 * ctl | pass |
| H2 | `H2.common.rtk_velocity_mps.p95` | 20.119 | 1.24062 | 0.0617x | cand <= 1.10 * ctl | pass |
| H2 | `H2.common.fused_velocity_mps.rmse` | 274.557 | 5.37545 | 0.0196x | cand <= 1.10 * ctl | pass |
| H2 | `H2.common.fused_velocity_mps.p95` | 480.04 | 4.99263 | 0.0104x | cand <= 1.10 * ctl | pass |
| H3 | `H3.all.fused_position_m.p99` | 39825.3 | 1228.81 | 0.0309x | cand <= 1.25 * ctl | pass |
| H3 | `H3.all.rotation_deg.p99` | 178.244 | 18.2086 | 0.1022x | cand <= 1.25 * ctl | pass |
| H4 | `H4.coverage.rtk_available` | 0.732187 | 0.733069 | +0.0009 | cand >= ctl - 0.005 | pass |
| H4 | `H4.coverage.fused_available` | 0.912443 | 0.999265 | +0.0868 | cand >= ctl - 0.005 | pass |
| H4 | `H4.coverage.rtk_velocity_available` | 0.732187 | 0.733069 | +0.0009 | cand >= ctl - 0.005 | pass |
| H4 | `H4.coverage.fused_velocity_available` | 0.912443 | 0.999265 | +0.0868 | cand >= ctl - 0.005 | pass |
| H4 | `H4.coverage.attitude_available` | 0.912443 | 0.999265 | +0.0868 | cand >= ctl - 0.005 | pass |
| H4 | `H4.coverage.heading_available` | 0.875129 | 0.989276 | +0.1141 | cand >= ctl - 0.005 | pass |
| H5 | `normal.initial.first_fresh_s` | 0 | 0 | +0.0000 | cand <= ctl + 1.0 s; a null cand fails if ctl is non-null | pass |
| H5 | `normal.initial.first_heading_s` | 2 | 22 | +20.0000 | cand <= ctl + 1.0 s; a null cand fails if ctl is non-null | **FAIL** |
| H5 | `gnss_outage.scenario.recovery_gnss_update_s` | 0 | 0 | +0.0000 | cand <= ctl + 1.0 s; a null cand fails if ctl is non-null | pass |
| H5 | `gnss_outage.scenario.recovery_fresh_attitude_s` | 0 | 0 | +0.0000 | cand <= ctl + 1.0 s; a null cand fails if ctl is non-null | pass |
| H5 | `gnss_outage.scenario.recovery_heading_s` | 0 | 0 | +0.0000 | cand <= ctl + 1.0 s; a null cand fails if ctl is non-null | pass |
| H5 | `imu_gap.scenario.recovery_gnss_update_s` | 2 | 2 | +0.0000 | cand <= ctl + 1.0 s; a null cand fails if ctl is non-null | pass |
| H5 | `imu_gap.scenario.recovery_fresh_attitude_s` | 2 | 2 | +0.0000 | cand <= ctl + 1.0 s; a null cand fails if ctl is non-null | pass |
| H5 | `imu_gap.scenario.recovery_heading_s` | 4 | 4 | +0.0000 | cand <= ctl + 1.0 s; a null cand fails if ctl is non-null | pass |
| H6 | `normal.processing.p95_ms` | 4.30588 | 6.18432 | 1.4363x | cand <= 2 * ctl; host contention | pass |
| H6 | `gnss_outage.processing.p95_ms` | 4.39878 | 5.63892 | 1.2819x | cand <= 2 * ctl; host contention | pass |
| H6 | `imu_gap.processing.p95_ms` | 4.69823 | 5.89218 | 1.2541x | cand <= 2 * ctl; host contention | pass |
| H8 | `H8.normal.attitude_integrity.rotation_gt_90deg_fraction` | 0.549799 | 0 | absolute | cand fraction of scored epochs with rotation_deg > 90 <= 0.01 (absolute; ctl is information only) | pass |
| H8 | `H8.gnss_outage.attitude_integrity.rotation_gt_90deg_fraction` | 0.596076 | 0 | absolute | cand fraction of scored epochs with rotation_deg > 90 <= 0.01 (absolute; ctl is information only) | pass |
| H8 | `H8.imu_gap.attitude_integrity.rotation_gt_90deg_fraction` | 0.499748 | 0 | absolute | cand fraction of scored epochs with rotation_deg > 90 <= 0.01 (absolute; ctl is information only) | pass |

## Observations on the four failures

These are descriptions of the produced outputs, written after the result. They
change no gate and propose nothing.

- **Deep, H4 RTK availability.** The pooled RTK availability of the candidate is
  0.8228 against 0.8315 for the control (RTK velocity 0.8228 against 0.8295),
  a loss of 0.0087 and 0.0067 against an allowed 0.005. By scenario the number of epochs with an RTK
  position is 1,244 / 1,234 / 1,244 for the control and 1,231 / 1,221 / 1,231
  for the candidate (normal / gnss_outage / imu_gap), a difference of 13
  epochs in each (of 1,492). The candidate's fused, velocity, attitude and
  heading availability are higher than the control's (0.9989 against 0.9171
  fused, 0.9861 against 0.8829 heading).
- **Harsh, H5 first heading latch (normal scenario).** The control latches its
  first heading at 2.0 s and the candidate at 22.0 s. In the same replay the
  control has 8 full resets on rover gaps (`rover_gap_reset`) and rotation error
  above 90 deg in 55 % of scored epochs; the candidate has none above 90 deg.
  The gate counts only the time of the first latch. The Deep run's first latch
  is 8.0 s in both arms.
- **Harsh, H2 RTK position RMSE (all-output cohort).** The RMSE is 747.6 m
  (control) against 1,095.0 m (candidate); the P95 is 187.8 m against 134.3 m,
  and the common-valid cohort RMSE is 700.33 m against 700.22 m (ratio 1.0002).
  The two cohorts differ by the epochs where only the candidate has an RTK
  position: in the normal replay 22 such epochs, of which three (elapsed 826,
  829 and 830 s) have RTK position errors of about 19.9 km. The control has the
  same size of error at the adjacent epochs 822 and 823 s (18.3 and 19.9 km), with nearly the
  same values in both arms, and no RTK output at 826, 829, 830 s. The RTK
  position is therefore the same large-error solution in both arms; the
  difference is which epochs the arm outputs it for. (Checked on the normal
  replay only, after the result; the other two scenarios show the same RMSE
  difference: 749.5 against 1,097.7 m and 746.1 against 1,093.5 m.)
  The common-valid RTK velocity and the fused position, velocity and rotation
  gates all pass by wide margins.

## Reported in addition (not gates)

### Absolute errors, with a bias warning

The tables below give absolute errors per scenario, run and cohort. They are
biased by up to about 1 m by the zero lever arm, the unknown truth reference
point, the WGS84 datum assumption and the unmodelled boresight (known
limitations of the contract); at the metre level they should not be read as
accuracy. Both arms are far from sub-metre on this data: the **control's** pooled
fused-position RMSE is 2,246 m (Deep) and 5,978 m (Harsh), its RTK-position P95
108 m and 188 m, and its rotation RMSE 99 and 108 deg. The candidate's pooled
fused-position RMSE is 317 m (Deep) and 223 m (Harsh), with RTK-position P95 of
36 m and 134 m and rotation RMSE of 4.3 and 7.8 deg. The values in the gate
tables above are the pooled H1 to H4 statistics for both arms; the per-scenario
values follow.

#### Pooled H1 summary (from the decision file)

| Run | Cohort | Metric | Control RMSE | Candidate RMSE | Control P95 | Candidate P95 |
|---|---|---|---:|---:|---:|---:|
| Deep | all | fused position (m) | 2246.15 | 316.82 | 5476.18 | 425.29 |
| Deep | all | rotation (deg) | 99.42 | 4.27 | 170.07 | 7.59 |
| Deep | common | fused position (m) | 2246.15 | 108.10 | 5476.18 | 65.14 |
| Deep | common | rotation (deg) | 99.42 | 4.11 | 170.07 | 7.71 |
| Harsh | all | fused position (m) | 5978.49 | 223.12 | 4313.41 | 384.35 |
| Harsh | all | rotation (deg) | 108.39 | 7.80 | 171.61 | 15.89 |
| Harsh | common | fused position (m) | 5978.49 | 203.68 | 4313.41 | 173.18 |
| Harsh | common | rotation (deg) | 108.34 | 7.91 | 171.53 | 16.11 |

### Per-scenario values of the H1-H3 statistics

Recomputed from each replay's `errors.csv` with the scorer's `stats` (absolute
values, linear interpolation between order statistics), the same functions and
cohort definitions as the comparator; not part of any gate. Units: metres,
degrees, m/s. "-" means no value.

#### HKDeepUrban1_novatel: per-scenario statistics, all-output cohort

| Scenario | Metric | n ctl | n cand | RMSE ctl | RMSE cand | P95 ctl | P95 cand | P99 ctl | P99 cand |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|
| normal | fused_position_m | 1370 | 1492 | 2244.988 | 317.671 | 5475.932 | 423.117 | 10078.504 | 1891.889 |
| normal | rotation_deg | 1330 | 1484 | 96.945 | 4.490 | 169.317 | 7.976 | 178.632 | 12.981 |
| normal | rtk_position_m | 1244 | 1231 | 764.647 | 33.981 | 111.746 | 36.186 | 5366.923 | 107.672 |
| normal | rtk_velocity_mps | 1241 | 1231 | 24.809 | 1.079 | 43.275 | 1.351 | 45.076 | 3.958 |
| normal | fused_velocity_mps | 1370 | 1492 | 221.863 | 13.553 | 173.595 | 41.256 | 618.090 | 60.705 |
| gnss_outage | fused_position_m | 1370 | 1492 | 2244.717 | 312.599 | 5475.932 | 419.510 | 10078.504 | 1859.454 |
| gnss_outage | rotation_deg | 1330 | 1484 | 103.467 | 4.083 | 170.803 | 6.755 | 178.719 | 11.965 |
| gnss_outage | rtk_position_m | 1234 | 1221 | 767.609 | 34.119 | 105.032 | 36.262 | 5367.972 | 107.715 |
| gnss_outage | rtk_velocity_mps | 1231 | 1221 | 24.861 | 1.084 | 43.300 | 1.365 | 45.090 | 3.959 |
| gnss_outage | fused_velocity_mps | 1370 | 1492 | 221.851 | 13.326 | 173.595 | 40.315 | 618.090 | 59.912 |
| imu_gap | fused_position_m | 1365 | 1487 | 2248.752 | 320.166 | 5476.069 | 424.842 | 10088.903 | 1906.977 |
| imu_gap | rotation_deg | 1292 | 1446 | 97.656 | 4.223 | 169.469 | 7.494 | 178.656 | 12.624 |
| imu_gap | rtk_position_m | 1244 | 1231 | 764.517 | 33.981 | 104.363 | 36.186 | 5366.923 | 107.672 |
| imu_gap | rtk_velocity_mps | 1241 | 1231 | 24.631 | 1.079 | 43.275 | 1.351 | 45.076 | 3.958 |
| imu_gap | fused_velocity_mps | 1365 | 1487 | 222.242 | 13.619 | 174.037 | 41.632 | 618.401 | 60.696 |

#### HKDeepUrban1_novatel: per-scenario statistics, common-valid cohort (both arms have the metric, matched by scenario and elapsed time)

| Scenario | Metric | n | RMSE ctl | RMSE cand | P95 ctl | P95 cand |
|---|---|---:|---:|---:|---:|---:|
| normal | fused_position_m | 1370 | 2244.988 | 108.421 | 5475.932 | 64.035 |
| normal | rotation_deg | 1330 | 96.945 | 4.339 | 169.317 | 8.016 |
| normal | rtk_position_m | 1218 | 43.234 | 34.018 | 78.906 | 35.659 |
| normal | rtk_velocity_mps | 1218 | 25.021 | 0.917 | 43.334 | 1.136 |
| normal | fused_velocity_mps | 1370 | 221.863 | 11.838 | 173.595 | 15.899 |
| gnss_outage | fused_position_m | 1370 | 2244.717 | 106.926 | 5475.932 | 63.626 |
| gnss_outage | rotation_deg | 1330 | 103.467 | 3.936 | 170.803 | 6.815 |
| gnss_outage | rtk_position_m | 1208 | 40.985 | 34.159 | 50.366 | 35.924 |
| gnss_outage | rtk_velocity_mps | 1208 | 25.075 | 0.920 | 43.366 | 1.139 |
| gnss_outage | fused_velocity_mps | 1370 | 221.851 | 11.663 | 173.595 | 15.810 |
| imu_gap | fused_position_m | 1365 | 2248.752 | 108.932 | 5476.069 | 64.649 |
| imu_gap | rotation_deg | 1292 | 97.656 | 4.049 | 169.469 | 7.559 |
| imu_gap | rtk_position_m | 1218 | 40.816 | 34.018 | 49.798 | 35.659 |
| imu_gap | rtk_velocity_mps | 1218 | 24.841 | 0.917 | 43.334 | 1.136 |
| imu_gap | fused_velocity_mps | 1365 | 222.242 | 11.878 | 174.037 | 16.072 |

#### HKHarshUrban1_novatel: per-scenario statistics, all-output cohort

| Scenario | Metric | n ctl | n cand | RMSE ctl | RMSE cand | P95 ctl | P95 cand | P99 ctl | P99 cand |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|
| normal | fused_position_m | 2072 | 2269 | 5953.212 | 224.203 | 4425.268 | 386.806 | 39198.883 | 1208.766 |
| normal | rotation_deg | 1988 | 2247 | 108.713 | 6.693 | 171.661 | 14.959 | 178.200 | 17.608 |
| normal | rtk_position_m | 1663 | 1667 | 747.230 | 1093.777 | 187.325 | 134.403 | 2716.647 | 248.178 |
| normal | rtk_velocity_mps | 1663 | 1667 | 11.613 | 0.946 | 20.122 | 1.440 | 21.105 | 3.740 |
| normal | fused_velocity_mps | 2072 | 2269 | 273.195 | 5.892 | 387.529 | 10.344 | 1526.814 | 33.683 |
| gnss_outage | fused_position_m | 2072 | 2269 | 6059.777 | 227.766 | 6330.566 | 388.106 | 39198.883 | 1212.038 |
| gnss_outage | rotation_deg | 1988 | 2247 | 111.599 | 6.316 | 172.129 | 13.139 | 178.616 | 16.115 |
| gnss_outage | rtk_position_m | 1653 | 1655 | 749.487 | 1097.738 | 190.404 | 134.562 | 2716.698 | 248.319 |
| gnss_outage | rtk_velocity_mps | 1653 | 1655 | 11.559 | 0.923 | 20.122 | 1.439 | 21.114 | 3.572 |
| gnss_outage | fused_velocity_mps | 2072 | 2269 | 279.732 | 5.894 | 555.276 | 10.068 | 1526.814 | 33.811 |
| imu_gap | fused_position_m | 2067 | 2264 | 5921.475 | 217.238 | 3453.823 | 304.424 | 39250.230 | 1211.355 |
| imu_gap | rotation_deg | 1981 | 2240 | 104.746 | 9.892 | 171.045 | 17.701 | 178.184 | 36.260 |
| imu_gap | rtk_position_m | 1668 | 1668 | 746.120 | 1093.450 | 186.061 | 133.914 | 2716.622 | 248.166 |
| imu_gap | rtk_velocity_mps | 1668 | 1668 | 8.973 | 0.920 | 16.892 | 1.414 | 19.522 | 3.536 |
| imu_gap | fused_velocity_mps | 2067 | 2264 | 270.654 | 5.887 | 358.547 | 9.918 | 1527.116 | 33.753 |

#### HKHarshUrban1_novatel: per-scenario statistics, common-valid cohort (both arms have the metric, matched by scenario and elapsed time)

| Scenario | Metric | n | RMSE ctl | RMSE cand | P95 ctl | P95 cand |
|---|---|---:|---:|---:|---:|---:|
| normal | fused_position_m | 2072 | 5953.212 | 205.758 | 4425.268 | 199.342 |
| normal | rotation_deg | 1968 | 108.668 | 6.762 | 171.518 | 15.788 |
| normal | rtk_position_m | 1645 | 699.759 | 699.657 | 145.391 | 133.935 |
| normal | rtk_velocity_mps | 1645 | 11.655 | 0.813 | 20.122 | 1.237 |
| normal | fused_velocity_mps | 2072 | 273.195 | 5.379 | 387.529 | 4.977 |
| gnss_outage | fused_position_m | 2072 | 6059.777 | 207.862 | 6330.566 | 199.872 |
| gnss_outage | rotation_deg | 1968 | 111.584 | 6.362 | 172.025 | 14.268 |
| gnss_outage | rtk_position_m | 1635 | 701.895 | 701.797 | 148.371 | 133.989 |
| gnss_outage | rtk_velocity_mps | 1635 | 11.600 | 0.815 | 20.122 | 1.244 |
| gnss_outage | fused_velocity_mps | 2072 | 279.732 | 5.380 | 555.276 | 4.977 |
| imu_gap | fused_position_m | 2067 | 5921.475 | 197.264 | 3453.823 | 128.481 |
| imu_gap | rotation_deg | 1961 | 104.657 | 10.090 | 170.947 | 17.892 |
| imu_gap | rtk_position_m | 1647 | 699.344 | 699.227 | 151.694 | 133.421 |
| imu_gap | rtk_velocity_mps | 1647 | 9.002 | 0.812 | 16.892 | 1.217 |
| imu_gap | fused_velocity_mps | 2067 | 270.654 | 5.368 | 358.547 | 4.960 |

### Errors split by truth quality Q (pooled over the three scenario replays, all-output cohort)


#### HKDeepUrban1_novatel

| Q | arm | epochs | fused pos RMSE | fused pos P95 | rotation RMSE | rotation P95 | RTK pos n | RTK pos RMSE | RTK pos P95 |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|
| 1 | control | 1551 | 1324.85 | 3830.56 | 83.73 | 165.34 | 1487 | 538.23 | 26.81 |
| 1 | candidate | 1551 | 42.01 | 26.05 | 3.22 | 6.75 | 1472 | 3.21 | 5.84 |
| 2 | control | 2271 | 2990.08 | 8543.06 | 108.60 | 172.21 | 1599 | 1046.33 | 184.15 |
| 2 | candidate | 2271 | 439.29 | 1081.00 | 4.51 | 7.84 | 1578 | 50.90 | 85.05 |
| 3 | control | 309 | 1285.79 | 807.92 | 103.78 | 163.42 | 291 | 17.89 | 36.12 |
| 3 | candidate | 309 | 158.87 | 445.72 | 5.33 | 12.25 | 288 | 18.67 | 37.10 |
| 4 | control | 261 | 16.94 | 31.24 | 105.29 | 169.15 | 261 | 15.04 | 35.95 |
| 4 | candidate | 261 | 2.74 | 5.59 | 5.51 | 8.10 | 261 | 11.50 | 23.15 |
| 5 | control | 84 | 18.82 | 31.57 | 85.07 | 156.98 | 84 | 17.79 | 34.13 |
| 5 | candidate | 84 | 5.17 | 9.02 | 5.12 | 8.13 | 84 | 17.39 | 30.79 |

#### HKHarshUrban1_novatel

| Q | arm | epochs | fused pos RMSE | fused pos P95 | rotation RMSE | rotation P95 | RTK pos n | RTK pos RMSE | RTK pos P95 |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|
| 1 | control | 138 | 609.26 | 1222.46 | 84.10 | 149.99 | 129 | 14.45 | 26.58 |
| 1 | candidate | 138 | 212.20 | 816.35 | 5.71 | 10.46 | 129 | 5.94 | 7.95 |
| 2 | control | 3606 | 3771.47 | 2976.04 | 107.88 | 172.03 | 2601 | 472.85 | 166.23 |
| 2 | candidate | 3606 | 257.33 | 500.17 | 6.64 | 12.46 | 2572 | 737.85 | 127.46 |
| 3 | control | 1299 | 5650.44 | 7306.52 | 107.36 | 167.99 | 764 | 1694.97 | 134.08 |
| 3 | candidate | 1299 | 247.13 | 383.27 | 8.70 | 17.28 | 779 | 2422.91 | 127.05 |
| 4 | control | 1020 | 7744.88 | 9135.17 | 109.68 | 171.18 | 748 | 71.91 | 229.48 |
| 4 | candidate | 1020 | 110.52 | 287.35 | 7.99 | 15.35 | 767 | 57.67 | 140.29 |
| 5 | control | 738 | 10586.79 | 37840.45 | 114.49 | 172.08 | 736 | 84.94 | 236.07 |
| 5 | candidate | 738 | 51.61 | 117.34 | 10.76 | 17.01 | 737 | 97.64 | 235.59 |
| 6 | control | 6 | 213.85 | 340.73 | 137.74 | 166.87 | 6 | 4.56 | 5.78 |
| 6 | candidate | 6 | 4.28 | 4.63 | 10.38 | 17.42 | 6 | 5.32 | 6.30 |

### Per-replay processor P95, rotation > 90 deg, and replay counters

| Run | Scenario | Arm | epochs | proc P95 ms | rot>90 frac | score heading_err>90 | score heading_err>150 | reset_count | rtk_base_seed_rejections | rtk_spp_blank_age_limited | rtk_float_prefit_gate_exceeded | fusion_reanchor_prefit_refusals | epoch_spp_velocity_exports | consider flag |
|---|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|
| HKDeepUrban1_novatel | normal | control | 1492 | 6.3411 | 0.427820 (569/1330) | 497 | 161 | 7 | n/a | n/a | n/a | n/a | n/a | n/a |
| HKDeepUrban1_novatel | normal | candidate | 1492 | 8.0511 | 0.000000 (0/1484) | 0 | 0 | 0 | 25 | 2 | 196 | 69 | 1211 | True |
| HKDeepUrban1_novatel | gnss_outage | control | 1492 | 6.1052 | 0.496241 (660/1330) | 588 | 208 | 7 | n/a | n/a | n/a | n/a | n/a | n/a |
| HKDeepUrban1_novatel | gnss_outage | candidate | 1492 | 7.1834 | 0.000000 (0/1484) | 0 | 0 | 0 | 25 | 2 | 196 | 71 | 1201 | True |
| HKDeepUrban1_novatel | imu_gap | control | 1492 | 6.0814 | 0.441950 (571/1292) | 499 | 164 | 8 | n/a | n/a | n/a | n/a | n/a | n/a |
| HKDeepUrban1_novatel | imu_gap | candidate | 1492 | 7.0878 | 0.000000 (0/1446) | 0 | 0 | 1 | 25 | 2 | 196 | 73 | 1211 | True |
| HKHarshUrban1_novatel | normal | control | 2269 | 4.3059 | 0.549799 (1093/1988) | 1005 | 317 | 8 | n/a | n/a | n/a | n/a | n/a | n/a |
| HKHarshUrban1_novatel | normal | candidate | 2269 | 6.1843 | 0.000000 (0/2247) | 0 | 0 | 0 | 16 | 0 | 534 | 184 | 1620 | True |
| HKHarshUrban1_novatel | gnss_outage | control | 2269 | 4.3988 | 0.596076 (1185/1988) | 1019 | 314 | 8 | n/a | n/a | n/a | n/a | n/a | n/a |
| HKHarshUrban1_novatel | gnss_outage | candidate | 2269 | 5.6389 | 0.000000 (0/2247) | 0 | 0 | 0 | 16 | 0 | 531 | 184 | 1609 | True |
| HKHarshUrban1_novatel | imu_gap | control | 2269 | 4.6982 | 0.499748 (990/1981) | 916 | 282 | 9 | n/a | n/a | n/a | n/a | n/a | n/a |
| HKHarshUrban1_novatel | imu_gap | candidate | 2269 | 5.8922 | 0.000000 (0/2240) | 0 | 0 | 1 | 16 | 1 | 537 | 180 | 1620 | True |

The meaning of the truth quality Q is not documented in the dataset files (see
the contract); the split above is descriptive only and no row was excluded by it.

### Other reported items

- **Processor P95** per replay is in the table above and in the H6 gates: the
  candidate / control ratios are 1.27 / 1.18 / 1.17 (Deep normal, gnss_outage,
  imu_gap) and 1.44 / 1.28 / 1.25 (Harsh), all below the allowed 2. The
  candidate's absolute P95 is 5.6 to 8.1 ms per epoch.
- **Behaviour of the candidate per run** (`replay.json`, table above). The consider-update
  option flag (`consider_attitude_and_biases_before_heading_latch`) is true in all 6
  candidate replays. Rover gaps over 2 s: the control records 7 (Deep) and 8
  (Harsh) `rover_gap_reset` full resets in the normal replay (`reset_count`
  7 / 8; 7 / 7 / 8 / 8 / 8 / 9 across the six control replays in the order Deep normal, gnss_outage, imu_gap, Harsh normal, gnss_outage, imu_gap), and the candidate
  records 7 and 8 `rover_gap_rtk_reset` RTK-only resets with `reset_count` 0 in the
  normal replays (the candidate's imu_gap replays have `reset_count` 1). Direction-test flips and gyro-bias
  seeds are not emitted in the `replay.json`, `score.json` or `manifest.json`
  of these runs, so they are not reported. The other candidate counters
  (`rtk_base_seed_rejections`, `rtk_float_prefit_gate_exceeded`,
  `fusion_reanchor_prefit_refusals`, `epoch_spp_velocity_exports`) are in the
  table above; the control's `replay.json` does not carry them.
- **IMU-versus-truth time offset** (about 0.00 s) and **heading-versus-course
  offset** (about -1.4 deg): neither was corrected, as frozen. Not re-measured here.
- **One epoch without an exact base epoch** (Harsh, 185501 s): the control
  records one `missing_exact_base` row in the normal replay; the candidate's
  normal replay records none.
- **Bound on the claim.** This is one city, one receiver, one IMU, one base
  network and two runs. The contract's Decision section applies: the result is
  No-Go, the default stays `none`, the evaluation of `velocity_consistency_v10`
  on this data ends and is not rerun, no later population is run to compensate,
  and the Hong Kong data is now development data for any later candidate.
  Nothing in this record proposes a change to the candidate, the gates or the
  data.

## Where the numbers come from

- Decision: `docs/online_pva_decision_holdout_v3.json` (SHA256
  `515984b548424677bb128bd5b75ed814860ad360abfe74003b1524c4110d54ab`).
- Per-replay `score.json` and `errors.csv`: the 12 replay directories pinned in the
  decision file (path and SHA256 of each manifest; each manifest pins its
  `errors.csv`, `score.json` and native CSV).
- The "reported in addition" tables were computed from those `errors.csv`,
  `reference.csv` (the Q column) and `replay.json` files by a one-off script;
  they are not produced by the frozen comparator and carry no gate.
