# Velocity-consistency candidate v3 results: No-Go

`velocity_consistency_v2` fails the frozen contract in
[online_pva_candidate_v3.md](online_pva_candidate_v3.md) (committed as
`81ebce80`, with the candidate code in `f91eb44b`, before the comparison).
Production defaults are unchanged; the candidate remains an explicit opt-in
(`--candidate velocity_consistency_v2`) for reproducibility. Machine-readable
gates: [online_pva_decision_v3.json](online_pva_decision_v3.json).

Control is the recorded develop `c2eb06f7` default set of
[v1](online_pva_candidate_v2_results.md). Candidate `none` built from the
candidate tree (develop `526cccaf` plus this change) is bit-identical to it in
all 60 deterministic CSV fields on all 18 runs, so no control was regenerated.
Candidate replay binary SHA256
`a8cddc82f86123f8287d392c9554967960bbd8ebe0d006f3b948c6adbbe9c399` (also used
for candidate `none`), repository commit `81ebce80`, clean tree. 36 full
replays (six runs x normal, GNSS outage 60-70 s, IMU gap 60-64 s, each control
and candidate), all truth-matched, identical input hashes.

## Decision

558 gates, 55 failed (42 excluding timing). Every fused position, fused
velocity and full-rotation RMSE/P95 gate passes in all 18 runs, on both
cohorts; coverage loses at most 0.04 pp; first fresh attitude is unchanged;
the targeted rotation improvement holds. Failed:

* RTK-filter (exported RTK result, not the fused one) position RMSE/P95 and
  velocity P95: 28 position gates (14 per cohort) and 8 velocity gates (4 per
  cohort), in Tokyo 2 IMU gap, Tokyo 3 normal/outage/gap and Nagoya 1/2/3.
  Tokyo 3 outage RTK position RMSE 16.8 -> 25.9 m, Nagoya 1 25.7 -> 30.2 m.
* First heading 41.8 -> 42.0 s on Nagoya 2 (one 0.2 s epoch, all three
  scenarios).
* GNSS-update recovery after the 10 s outage: 0.0 -> 0.2 s in Tokyo 2 and
  Nagoya 2, and 0.0 -> 26.0 s in Nagoya 1 (v1: 142.8 s).
* Processor P95 in 13 of 18 replays (e.g. 6.3 -> 16.6 ms, Tokyo 1), host
  contention. The host load average was about 16 (other jobs) when the
  candidate and a fresh candidate-`none` run were made, and the fresh
  candidate `none` is as slow as the candidate (Tokyo 1 17.2 ms against 16.6 ms,
  Nagoya 1 15.6 against 15.2). Re-scoring against that contemporaneous
  candidate-`none` set, which is bit-identical to the control in every
  deterministic field, gives 42 failed gates and no timing failure (output
  directory `decision_vs_none`). The decision is No-Go either way; the
  recorded-control figure above is the contract result.

## Full-run results (normal), control -> candidate

| Run | Fused position RMSE / P95 (m) | Fused velocity RMSE / P95 (m/s) | Rotation RMSE / P95 (deg) |
| --- | ---: | ---: | ---: |
| Tokyo 1 | 71.7 -> 17.3 / 155.2 -> 33.9 | 7.05 -> 1.04 / 14.62 -> 1.30 | 105.1 -> 2.4 / 171.1 -> 3.9 |
| Tokyo 2 | 74.2 -> 6.7 / 179.5 -> 14.5 | 6.85 -> 0.34 / 14.14 -> 0.79 | 100.3 -> 1.7 / 170.0 -> 2.8 |
| Tokyo 3 | 41.9 -> 3.1 / 115.0 -> 7.3 | 3.22 -> 0.25 / 8.22 -> 0.50 | 63.5 -> 1.7 / 144.4 -> 2.8 |
| Nagoya 1 | 49.3 -> 24.5 / 118.1 -> 54.5 | 4.61 -> 1.24 / 9.84 -> 1.73 | 88.2 -> 41.7 / 168.7 -> 129.5 |
| Nagoya 2 | 81.0 -> 35.4 / 168.0 -> 83.9 | 3.73 -> 0.58 / 9.96 -> 1.13 | 18.5 -> 3.1 / 39.8 -> 6.2 |
| Nagoya 3 | 58.9 -> 25.2 / 128.1 -> 58.8 | 5.54 -> 0.64 / 11.72 -> 1.08 | 105.0 -> 3.1 / 170.9 -> 6.4 |

Availability: fused 99.81-99.93% unchanged; heading availability changes by at
most 0.04 pp. Outage and IMU-gap tables, with the RTK columns and recovery
times, are in [online_pva_candidate_v3_scenarios.md](online_pva_candidate_v3_scenarios.md).
Against v1 the recovery improves position on five runs (Tokyo 3 14.3 -> 3.1 m,
Nagoya 1 119.0 -> 24.5 m) and worsens it on Nagoya 2 (18.7 -> 35.4 m, P95
33.8 -> 83.9), with small rotation losses against v1 (Nagoya 3 RMSE 2.55 ->
3.08 deg, Nagoya 2 P95 5.5 -> 6.2 deg; not gated against v1). The cause of the
Nagoya 2 loss was not isolated (a FLOAT inside the 33 m cross-check bound is
followed). Nagoya 1 rotation is still 41.7 deg (the reverse start is not
addressed).

## What the failures are

1. Nagoya 1 lockout fixed, not eliminated. The 120-170 m offset of v1 is gone
   (fused position 24.5 m, below the control's 49.3 m, tail 0.2-1 m from about
   750 s). After the 10 s outage the state is rejected by both gate classes and
   the FLOAT streak needs 30 epochs, which is the 26.0 s above: bounded, but
   later than the control's 0.0 s, so the contract's recovery gate fails by
   design of the patience.
2. The RTK-filter gates fail for the reason already derived before freezing: the
   fusion's tight predictor supplies the RTK filter's time-update prior
   (`online_rtk_imu.cpp:171-175`) and component (b) changes the velocity that
   re-anchors it (`:218`), so the exported RTK result and its FLOAT-versus-SPP
   epoch mix change. The recovery added here does not touch that path: RTK
   columns of v1 and this candidate agree (FLOAT epoch counts equal on four
   runs, 1 and 2 epochs different on Nagoya 2 and 3). The ablation below shows
   that dropping (b) does not restore the control's RTK output either and makes
   RTK velocity worse, so this is a property of the feedback structure, not
   something this candidate fixes.
3. Nagoya 2 heading 41.8 -> 42.0 s: the heading tracker consumes the same
   `gnss_input` velocity (`fusion_processor.cpp` heading tracker), which (b)
   replaces by the Doppler solution; the latch consensus is reached one epoch
   later. Inferred from the code path, not isolated by an experiment.
4. Tokyo 2 / Nagoya 2 outage update recovery 0.0 -> 0.2 s: with the debug print
   (v1 candidate, Tokyo 2, outage) the first post-outage epoch is a FLOAT
   update rejected by the position gate (NIS per observation 196, innovation
   6.8 m against a declared 0.1 m sigma); a coarse update is accepted at the
   next epoch. The new recovery is not involved (the rejection streak was 1 epoch).

## Ablation (informational, normal runs, not used to change the candidate)

Same six normal runs and binary; `recovery, no (b)` is the frozen code with
`independent_doppler_velocity` disabled by a temporary, uncommitted environment
switch (patch `replay_ablation_env.patch` in the output directory, source
commit `f91eb44b`, whose output is bit-identical to the committed binary for the
full candidate).

| Run | Variant | Fused pos RMSE/P95 m | Fused vel RMSE/P95 m/s | Rot RMSE/P95 deg | RTK pos RMSE/P95 m | RTK vel RMSE/P95 m/s |
|---|---|---|---|---|---|---|
| Tokyo 1 | control | 71.65/155.23 | 7.05/14.62 | 105.07/171.11 | 32.20/61.67 | 2.10/3.29 |
| Tokyo 1 | v1 | 25.92/48.06 | 1.20/1.26 | 2.91/6.03 | 27.37/55.22 | 1.40/2.78 |
| Tokyo 1 | v1+recovery | 17.29/33.92 | 1.04/1.30 | 2.39/3.86 | 27.37/55.22 | 1.40/2.80 |
| Tokyo 1 | recovery, no (b) | 18.23/46.58 | 0.46/1.02 | 2.39/5.09 | 30.08/60.88 | 2.78/4.60 |
| Tokyo 2 | control | 74.18/179.52 | 6.85/14.14 | 100.30/170.00 | 19.54/29.67 | 2.26/3.71 |
| Tokyo 2 | v1 | 13.83/26.99 | 0.38/0.78 | 1.42/2.57 | 12.50/21.10 | 0.92/2.12 |
| Tokyo 2 | v1+recovery | 6.66/14.52 | 0.34/0.79 | 1.67/2.83 | 12.50/21.10 | 0.92/2.11 |
| Tokyo 2 | recovery, no (b) | 10.80/20.77 | 0.26/0.52 | 1.89/2.96 | 14.82/24.90 | 1.96/2.47 |
| Tokyo 3 | control | 41.86/115.03 | 3.22/8.22 | 63.45/144.41 | 18.61/31.88 | 2.10/1.76 |
| Tokyo 3 | v1 | 14.30/26.19 | 0.28/0.58 | 1.73/2.97 | 15.18/29.86 | 0.82/1.78 |
| Tokyo 3 | v1+recovery | 3.11/7.34 | 0.25/0.50 | 1.69/2.78 | 15.18/29.86 | 0.82/1.78 |
| Tokyo 3 | recovery, no (b) | 14.13/26.72 | 0.23/0.50 | 1.73/2.99 | 17.33/31.02 | 1.40/1.55 |
| Nagoya 1 | control | 49.33/118.15 | 4.61/9.84 | 88.17/168.65 | 25.74/29.04 | 2.51/4.26 |
| Nagoya 1 | v1 | 118.98/178.29 | 1.30/1.61 | 35.36/103.01 | 30.19/34.53 | 1.39/2.72 |
| Nagoya 1 | v1+recovery | 24.53/54.49 | 1.24/1.73 | 41.72/129.45 | 30.19/34.53 | 1.39/2.72 |
| Nagoya 1 | recovery, no (b) | 53.32/87.27 | 1.01/1.25 | 35.08/66.68 | 28.66/31.91 | 1.61/3.21 |
| Nagoya 2 | control | 80.99/167.96 | 3.73/9.96 | 18.48/39.82 | 37.69/81.81 | 2.30/5.09 |
| Nagoya 2 | v1 | 18.74/33.80 | 0.57/1.08 | 3.15/5.50 | 37.08/83.80 | 1.48/3.04 |
| Nagoya 2 | v1+recovery | 35.35/83.85 | 0.58/1.13 | 3.10/6.24 | 37.11/83.88 | 1.48/3.04 |
| Nagoya 2 | recovery, no (b) | 28.51/49.57 | 0.58/1.35 | 3.41/4.80 | 45.70/114.79 | 1.99/4.20 |
| Nagoya 3 | control | 58.88/128.08 | 5.54/11.72 | 104.97/170.93 | 43.70/101.09 | 2.47/5.13 |
| Nagoya 3 | v1 | 31.78/66.92 | 0.58/0.94 | 2.55/4.54 | 44.95/109.03 | 1.56/3.45 |
| Nagoya 3 | v1+recovery | 25.15/58.76 | 0.64/1.08 | 3.08/6.36 | 45.00/109.24 | 1.56/3.45 |
| Nagoya 3 | recovery, no (b) | 29.97/61.39 | 0.37/0.86 | 3.03/5.82 | 44.15/101.24 | 1.78/3.59 |

Reading: the recovery is what separates v1 from this candidate in fused
position (largest on Nagoya 1 and Tokyo 3); component (b) is the source of the
RTK-output change in both directions (RTK velocity RMSE 2.1-2.5 -> 0.8-1.6 m/s
with it, to 1.4-2.8 without it; RTK position RMSE better in the three Tokyo runs
and Nagoya 2 with it, worse in Nagoya 1 and 3). Without (b) the fused velocity
is better or equal on all six runs, and the fused position is worse on Tokyo 2
(10.8 against 6.7 m), Tokyo 3 (14.1 against 3.1), Nagoya 1 and 3 and better on
Nagoya 2. No variant is RTK-gate-clean on all six runs (only Tokyo 3 without
(b) is within 1% of the control on RTK position and velocity RMSE/P95). The
ablation was not run on the outage or IMU-gap scenarios.

## Post-hoc and development diagnostics

* First version of the recovery, N = 30 without the coarse cross-check (not
  the frozen candidate, Nagoya 1 and Tokyo 3 only): Nagoya 1 fused position
  RMSE 59.6 m (control 49.3, v1 119.0), because the FLOAT it re-anchored to was
  itself 21-150 m off between about 495 s and 735 s; Tokyo 3 3.1 m. The
  cross-check was added from that observation before the freeze.
* The recovery has not been exercised on a deployment without coarse epochs
  between base epochs (known limit i of the contract).

## Validation

New unit tests (`FusionProcessorSyntheticTest.FloatGateRecovery*`, four cases):
default off keeps FLOAT rejected; a FLOAT re-anchor fires at the configured
count despite interleaved accepted coarse updates, changes only the position
state (velocity, attitude, gyro bias unchanged; position cross-covariances
cleared; position variance equal to the FLOAT covariance) and restarts the
counter; a drifting FLOAT 140 m from the coarse position is refused; no recent
coarse epoch refuses; an accepted FLOAT resets the streak. A mutation test
(consistency check bypassed) makes three of the four fail. Comparator test for
the candidate name and the absent v1 parity gate; defaults test extended with
the new field. Release build of the full tree (Python bindings off):
`run_tests` passes 1,512 cases (67 skipped), `gnss_online_tests` 13/13,
`python_pva_tests` and `python_pva_comparison_tests` pass. The broader `ctest`
lanes that failed environmentally in the v1 record were not re-run here.
Outputs are under `rtklib_v2_ws_output/pva_v3_20261009/` in the local workspace
(`none`, `cand`, `abl_v1`, `diag`, `decision`, `decision_vs_none`).

## Next experiment (needs a new contract)

Do not reuse this candidate. The fused-state problem (lockout) is now handled,
and what blocks adoption is the contract's RTK-output gates and the
post-outage first-update gate. A defensible follow-up has to decouple what the
RTK filter sees from what the fused filter corrects (for example keep the
control's tight predictor for the RTK prior and run the corrected state as a
separate output) or change the contract to score the fused solution only; that
is a change of the gates, which this experiment may not do. Nagoya 1's reverse
start (rotation 41.7 deg) and the gyro bias learned before the latch remain
open.
