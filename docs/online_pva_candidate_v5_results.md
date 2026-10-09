# Velocity-consistency candidate v5 results: Go on the contemporaneous basis (0 of 558 gates), 5 timing gates fail against the recorded control

`velocity_consistency_v4` was evaluated against the frozen contract in
[online_pva_candidate_v5.md](online_pva_candidate_v5.md) (freeze commit
`066e4a60`, committed before any comparison replay; contract SHA256
`fbe3903c41b2755bbfcaae86b1fd1efae47f7c010900a653630ba048a83ea276`). Production
defaults are unchanged; the candidate stays an explicit opt-in
(`--candidate velocity_consistency_v4`). Machine-readable gates:
[online_pva_decision_v5.json](online_pva_decision_v5.json) (compact; the 1.6 MB
full decisions are in the local output directory, hashes recorded there).

## Decision

558 gates over 18 run/scenarios, replay binary SHA256
`001bc66bc98709b1c8d556d7a5ae12edbbc5f4d5cd3398afc92b821bb80bd4cc` (clean
tree), 36 replays plus 18 contemporaneous candidate-`none` replays, all
truth-matched, identical input hashes.

| Basis | Result |
|---|---|
| Candidate vs **contemporaneous candidate-`none`** (same binary, run side by side under the same host load; bit-identical to the control, gate 7) | **Go, 0 of 558 gates fail**, targeted rotation improvement present |
| Candidate vs the **recorded control** (develop `c2eb06f7` set, produced at an earlier, lighter load) | No-Go as written: 5 of 558 gates fail, all `processing.p95_ms` (Tokyo 1 12.6 vs 6.3 ms, Tokyo 2 16.1 vs 7.7, Tokyo 3 IMU gap 19.5 vs 8.1, Nagoya 1 IMU gap 12.9 vs 6.2, Nagoya 2 IMU gap 12.0 vs 5.7) |

Gate 4 asks for candidate P95 <= 2 x control with host contention reported, not
hidden. The host was loaded by other jobs (load average 11-20) during this
comparison: candidate-`none` itself is 1.49-2.30x (mean 1.79) slower than the
recorded control. The candidate's own cost against the same-binary
contemporaneous `none` is 1.01-1.11x (mean 1.06), i.e. the isolated second fusion
filter of v3 plus the new branch cost about 6 % at P95. Every non-timing gate
(position, velocity and rotation RMSE/P95, RTK columns, coverage, first
fresh-attitude/heading, scenario recovery, parity) passes against both
bases, including the four `scenario.recovery_gnss_update_s` gates that failed in v3.
Read plainly: the candidate passes every accuracy and recovery gate; the strict
timing gate against the recorded control is not met on a loaded host and is
met against the same-load reference. No-Go/Go on this development data is
not an adoption claim: the six runs were also the development data, there is no
holdout, and the production default is unchanged.

Candidate `none` built from the candidate tree is bit-identical to the recorded
control in every deterministic CSV field on all 18 runs (gate 7, 175,902 rows).

## What changed against v3

Only the post-outage recovery. Normal runs are bit-identical to v3 on Tokyo
1-3 and Nagoya 3; Nagoya 1 and Nagoya 2 differ only after their real GNSS
dropouts (954 s and 1322 s): fused position RMSE/P95 17.7/32.8 -> 13.1/24.3 m
and 22.5/53.9 -> 21.3/50.1 m. The RTK columns are unchanged everywhere (the RTK
filter is isolated from the fused output).

The first GNSS position update after the outage is now applied at once on all
six runs (0.0 s; v3: 0.2 s on Tokyo 2/Tokyo 3/Nagoya 2, 18.0 s on Nagoya 1).
Fused position error (m) around the outage:

Fused position error (m) at t = 69.8, 70.0, 70.2, 72.0, 80.0, 100.0 s (outage 60-70 s)
| Run | control | v3 | v4 |
|---|---|---|---|
| tokyo1 | 161.1 / 8.7 / 7.7 / 6.0 / 1.7 / 3.0 | 3.0 / 0.1 / 0.1 / 0.2 / 0.4 / 0.2 | 3.0 / 0.1 / 0.1 / 0.2 / 0.4 / 0.2 |
| tokyo2 | 17.1 / 10.0 / 10.1 / 10.0 / 4.6 / 2.6 | 7.1 / 7.3 / 5.7 / 3.0 / 2.8 / 0.7 | 7.1 / 11.5 / 11.5 / 11.6 / 12.5 / 0.8 |
| tokyo3 | 23.8 / 0.6 / 0.9 / 1.9 / 1.4 / 0.8 | 9.6 / 9.6 / 9.5 / 5.1 / 0.4 / 0.3 | 9.6 / 0.2 / 0.1 / 0.6 / 0.6 / 0.2 |
| nagoya1 | 39.3 / 0.9 / 0.8 / 2.0 / 3.1 / 7.3 | 34.6 / 35.8 / 37.0 / 38.1 / 39.1 / 2.0 | 34.6 / 0.3 / 1.2 / 2.2 / 3.1 / 2.8 |
| nagoya2 | 8.3 / 0.3 / 0.4 / 0.4 / 0.6 / 0.5 | 10.5 / 10.8 / 9.8 / 7.9 / 4.1 / 0.2 | 10.5 / 0.1 / 0.3 / 1.9 / 0.3 / 0.2 |
| nagoya3 | 333.9 / 11.0 / 8.3 / 13.8 / 5.1 / 11.7 | 3.7 / 1.6 / 1.6 / 1.3 / 1.6 / 0.8 | 3.7 / 1.6 / 1.6 / 1.3 / 1.6 / 0.8 |

Reading: Nagoya 1/Nagoya 2/Tokyo 3 recover at the first FLOAT (Nagoya 1 from 35.8
m to 0.3 m; v3 stayed 36-39 m off until the 30-rejection re-anchor at 88 s).
**Tokyo 2 is worse than v3 for about 30 s**: the first post-outage FLOAT is a
reconverging solution whose height is 11.45 m off (equal to this data set's SPP
vertical bias) with a reported sigma of 0.59 m (19 sigma, contract point 2). The
re-anchor adopts it, the fused height follows the reconverging FLOAT at 11-12
m (v3: 3 m, because the NIS gate rejected the optimistic FLOAT) until the
30-rejection re-anchor at about 100 s. That equals the control's behaviour at
that epoch (10 m) and the outage-run RMSE barely moves (8.5 -> 8.7 m) because
the window is 30 s of a 1,800 s run, but it is a real cost of trusting a FLOAT
that is still optimistic after a reconvergence; fixing it needs an honest
reconverging-FLOAT covariance in the RTK filter (out of scope, see
[rtk_float_covariance.md](rtk_float_covariance.md) limits). Nagoya 1/Nagoya 2
outage rotation regressed slightly against v3 (42.6 -> 45.8 deg and 2.0 -> 2.5
deg RMSE; still far below the control's 88.7 and 90.8).

## Full-run results, control -> candidate

### NORMAL  (control -> candidate velocity_consistency_v4)
| Run | Fused pos RMSE/P95 m | Fused vel RMSE/P95 m/s | Rot RMSE/P95 deg | RTK pos RMSE/P95 m | RTK vel RMSE/P95 m/s | fused / heading avail % | first fresh/heading s |
|---|---|---|---|---|---|---|---|
| Tokyo 1 | 71.7 -> 14.4 / 155.2 -> 29.7 | 7.05 -> 0.30 / 14.62 -> 0.58 | 105.1 -> 1.8 / 171.1 -> 3.0 | 32.2 -> 32.2 / 61.7 -> 61.7 | 2.10 -> 2.10 / 3.29 -> 3.29 | 99.92/99.36 -> 99.92/99.36 | 2.0/15.2 -> 2.0/15.2 |
| Tokyo 2 | 74.2 -> 10.3 / 179.5 -> 19.7 | 6.85 -> 0.28 / 14.14 -> 0.51 | 100.3 -> 2.0 / 170.0 -> 2.2 | 19.5 -> 19.5 / 29.7 -> 29.7 | 2.26 -> 2.26 / 3.71 -> 3.71 | 99.89/98.72 -> 99.89/98.72 | 2.0/23.4 -> 2.0/23.4 |
| Tokyo 3 | 41.9 -> 14.1 / 115.0 -> 26.6 | 3.22 -> 0.23 / 8.22 -> 0.51 | 63.5 -> 1.7 / 144.4 -> 2.8 | 18.6 -> 18.6 / 31.9 -> 31.9 | 2.10 -> 2.10 / 1.76 -> 1.76 | 99.93/98.77 -> 99.93/98.77 | 2.0/37.6 -> 2.0/37.6 |
| Nagoya 1 | 49.3 -> 13.1 / 118.1 -> 24.3 | 4.61 -> 1.02 / 9.84 -> 1.24 | 88.2 -> 31.1 / 168.7 -> 61.3 | 25.7 -> 25.7 / 29.0 -> 29.0 | 2.51 -> 2.51 / 4.26 -> 4.26 | 99.82/96.50 -> 99.82/96.50 | 2.0/52.0 -> 2.0/52.0 |
| Nagoya 2 | 81.0 -> 21.3 / 168.0 -> 50.1 | 3.73 -> 0.46 / 9.96 -> 0.92 | 18.5 -> 2.4 / 39.8 -> 4.4 | 37.7 -> 37.7 / 81.8 -> 81.8 | 2.30 -> 2.30 / 5.09 -> 5.09 | 99.89/97.79 -> 99.89/97.79 | 2.0/41.8 -> 2.0/41.8 |
| Nagoya 3 | 58.9 -> 31.9 / 128.1 -> 61.4 | 5.54 -> 0.35 / 11.72 -> 0.85 | 105.0 -> 2.9 / 170.9 -> 5.5 | 43.7 -> 43.7 / 101.1 -> 101.1 | 2.47 -> 2.47 / 5.13 -> 5.13 | 99.81/96.54 -> 99.81/96.54 | 2.0/36.0 -> 2.0/36.0 |

### GNSS OUTAGE 60-70 s  (control -> candidate velocity_consistency_v4)
| Run | Fused pos RMSE/P95 m | Fused vel RMSE/P95 m/s | Rot RMSE/P95 deg | RTK pos RMSE/P95 m | RTK vel RMSE/P95 m/s | fused / heading avail % | first fresh/heading s |
|---|---|---|---|---|---|---|---|
| Tokyo 1 | 55.1 -> 14.1 / 130.9 -> 27.9 | 6.11 -> 0.28 / 11.78 -> 0.56 | 104.9 -> 1.7 / 171.2 -> 2.8 | 29.7 -> 29.7 / 58.9 -> 58.9 | 2.64 -> 2.64 / 3.89 -> 3.89 | 99.92/99.36 -> 99.92/99.36 | 2.0/15.2 -> 2.0/15.2 |
| Tokyo 2 | 72.5 -> 8.7 / 187.3 -> 18.9 | 4.82 -> 0.27 / 11.98 -> 0.63 | 28.5 -> 2.7 / 68.1 -> 2.8 | 19.4 -> 19.4 / 26.4 -> 26.4 | 2.42 -> 2.42 / 4.99 -> 4.99 | 99.89/98.72 -> 99.89/98.72 | 2.0/23.4 -> 2.0/23.4 |
| Tokyo 3 | 27.2 -> 14.4 / 31.9 -> 26.7 | 1.95 -> 0.22 / 1.40 -> 0.48 | 12.7 -> 1.6 / 40.6 -> 2.9 | 16.8 -> 16.8 / 30.9 -> 30.9 | 1.07 -> 1.07 / 1.55 -> 1.55 | 99.93/98.77 -> 99.93/98.77 | 2.0/37.6 -> 2.0/37.6 |
| Nagoya 1 | 53.2 -> 13.4 / 131.9 -> 24.5 | 4.87 -> 1.13 / 10.91 -> 1.50 | 88.7 -> 45.8 / 168.8 -> 153.3 | 26.0 -> 26.0 / 29.3 -> 29.3 | 2.76 -> 2.76 / 4.39 -> 4.39 | 99.82/96.50 -> 99.82/96.50 | 2.0/52.0 -> 2.0/52.0 |
| Nagoya 2 | 55.2 -> 12.9 / 133.2 -> 30.9 | 4.30 -> 0.39 / 9.76 -> 0.73 | 90.8 -> 2.5 / 167.5 -> 3.5 | 37.2 -> 37.2 / 85.7 -> 85.7 | 4.24 -> 4.24 / 13.91 -> 13.91 | 99.89/97.79 -> 99.89/97.79 | 2.0/41.8 -> 2.0/41.8 |
| Nagoya 3 | 75.2 -> 31.7 / 171.7 -> 61.4 | 5.75 -> 0.36 / 10.27 -> 0.85 | 103.2 -> 2.8 / 171.3 -> 5.4 | 45.8 -> 45.8 / 103.7 -> 103.7 | 2.47 -> 2.47 / 6.22 -> 6.22 | 99.81/96.54 -> 99.81/96.54 | 2.0/36.0 -> 2.0/36.0 |

Recovery s (GNSS update / fresh attitude / heading), control -> candidate:
- tokyo1: 0.0 / 0.0 / 0.0 -> 0.0 / 0.0 / 0.0
- tokyo2: 0.0 / 0.0 / 0.0 -> 0.0 / 0.0 / 0.0
- tokyo3: 0.0 / 0.0 / 0.0 -> 0.0 / 0.0 / 0.0
- nagoya1: 0.0 / 0.0 / 0.0 -> 0.0 / 0.0 / 0.0
- nagoya2: 0.0 / 0.0 / 0.0 -> 0.0 / 0.0 / 0.0
- nagoya3: 0.0 / 0.0 / 0.0 -> 0.0 / 0.0 / 0.0

### IMU GAP 60-64 s  (control -> candidate velocity_consistency_v4)
| Run | Fused pos RMSE/P95 m | Fused vel RMSE/P95 m/s | Rot RMSE/P95 deg | RTK pos RMSE/P95 m | RTK vel RMSE/P95 m/s | fused / heading avail % | first fresh/heading s |
|---|---|---|---|---|---|---|---|
| Tokyo 1 | 63.4 -> 15.0 / 148.0 -> 32.2 | 5.83 -> 0.29 / 11.39 -> 0.61 | 104.9 -> 1.8 / 171.3 -> 3.1 | 31.2 -> 31.2 / 63.3 -> 63.3 | 2.36 -> 2.36 / 3.72 -> 3.72 | 99.67/98.39 -> 99.67/98.39 | 2.0/15.2 -> 2.0/15.2 |
| Tokyo 2 | 52.2 -> 11.8 / 111.5 -> 20.1 | 6.07 -> 0.40 / 12.06 -> 0.93 | 102.7 -> 18.4 / 170.6 -> 55.6 | 14.0 -> 14.0 / 23.6 -> 23.6 | 1.66 -> 1.66 / 2.21 -> 2.21 | 99.57/97.89 -> 99.57/97.89 | 2.0/23.4 -> 2.0/23.4 |
| Tokyo 3 | 59.3 -> 14.1 / 90.2 -> 26.7 | 6.60 -> 0.22 / 12.34 -> 0.48 | 104.6 -> 2.0 / 171.2 -> 3.3 | 17.3 -> 17.3 / 31.0 -> 31.0 | 1.53 -> 1.53 / 1.74 -> 1.74 | 99.75/98.57 -> 99.75/98.57 | 2.0/37.6 -> 2.0/37.6 |
| Nagoya 1 | 55.8 -> 13.1 / 130.6 -> 24.3 | 4.91 -> 0.96 / 11.22 -> 1.07 | 83.0 -> 14.1 / 167.4 -> 16.5 | 27.6 -> 27.6 / 30.8 -> 30.8 | 2.57 -> 2.57 / 4.34 -> 4.34 | 99.43/96.09 -> 99.43/96.09 | 2.0/52.0 -> 2.0/52.0 |
| Nagoya 2 | 64.2 -> 15.1 / 146.2 -> 32.7 | 4.41 -> 0.34 / 9.38 -> 0.70 | 104.1 -> 2.3 / 170.7 -> 4.6 | 38.3 -> 38.3 / 85.0 -> 85.0 | 4.34 -> 4.34 / 14.59 -> 14.59 | 99.59/97.46 -> 99.59/97.46 | 2.0/41.8 -> 2.0/41.8 |
| Nagoya 3 | 88.6 -> 32.8 / 192.6 -> 61.4 | 7.26 -> 0.41 / 14.31 -> 0.87 | 106.7 -> 2.8 / 170.3 -> 5.4 | 43.8 -> 43.8 / 100.7 -> 100.7 | 2.04 -> 2.04 / 3.82 -> 3.82 | 99.25/95.94 -> 99.25/95.94 | 2.0/36.0 -> 2.0/36.0 |

Recovery s (GNSS update / fresh attitude / heading), control -> candidate:
- tokyo1: 2.0 / 2.0 / 19.4 -> 2.0 / 2.0 / 19.4
- tokyo2: 2.0 / 2.0 / 11.4 -> 2.0 / 2.0 / 11.4
- tokyo3: 2.0 / 2.0 / 2.4 -> 2.0 / 2.0 / 2.4
- nagoya1: 2.0 / 2.0 / 2.4 -> 2.0 / 2.0 / 2.4
- nagoya2: 2.0 / 2.0 / 2.4 -> 2.0 / 2.0 / 2.4
- nagoya3: 2.0 / 2.0 / 2.4 -> 2.0 / 2.0 / 2.4


## v3 -> v4

### NORMAL  (v3 -> v4), fused position / velocity / rotation RMSE
| Run | Fused pos RMSE/P95 m | Fused vel RMSE/P95 m/s | Rot RMSE/P95 deg | GNSS-update recovery s |
|---|---|---|---|---|
| tokyo1 | 14.4 -> 14.4 / 29.7 -> 29.7 | 0.30 -> 0.30 / 0.58 -> 0.58 | 1.8 -> 1.8 / 3.0 -> 3.0 | - |
| tokyo2 | 10.3 -> 10.3 / 19.7 -> 19.7 | 0.28 -> 0.28 / 0.51 -> 0.51 | 2.0 -> 2.0 / 2.2 -> 2.2 | - |
| tokyo3 | 14.1 -> 14.1 / 26.6 -> 26.6 | 0.23 -> 0.23 / 0.51 -> 0.51 | 1.7 -> 1.7 / 2.8 -> 2.8 | - |
| nagoya1 | 17.7 -> 13.1 / 32.8 -> 24.3 | 1.02 -> 1.02 / 1.25 -> 1.24 | 31.1 -> 31.1 / 61.3 -> 61.3 | - |
| nagoya2 | 22.5 -> 21.3 / 53.9 -> 50.1 | 0.46 -> 0.46 / 0.88 -> 0.92 | 2.5 -> 2.4 / 4.4 -> 4.4 | - |
| nagoya3 | 31.9 -> 31.9 / 61.4 -> 61.4 | 0.35 -> 0.35 / 0.85 -> 0.85 | 2.9 -> 2.9 / 5.5 -> 5.5 | - |

### GNSS OUTAGE 60-70 s  (v3 -> v4), fused position / velocity / rotation RMSE
| Run | Fused pos RMSE/P95 m | Fused vel RMSE/P95 m/s | Rot RMSE/P95 deg | GNSS-update recovery s |
|---|---|---|---|---|
| tokyo1 | 14.1 -> 14.1 / 27.9 -> 27.9 | 0.28 -> 0.28 / 0.56 -> 0.56 | 1.7 -> 1.7 / 2.8 -> 2.8 | 0.0 -> 0.0 |
| tokyo2 | 8.5 -> 8.7 / 18.9 -> 18.9 | 0.27 -> 0.27 / 0.63 -> 0.63 | 2.6 -> 2.7 / 2.5 -> 2.8 | 0.2 -> 0.0 |
| tokyo3 | 14.4 -> 14.4 / 26.7 -> 26.7 | 0.22 -> 0.22 / 0.48 -> 0.48 | 1.6 -> 1.6 / 2.9 -> 2.9 | 0.2 -> 0.0 |
| nagoya1 | 18.4 -> 13.4 / 36.5 -> 24.5 | 1.13 -> 1.13 / 1.30 -> 1.50 | 42.6 -> 45.8 / 146.0 -> 153.3 | 18.0 -> 0.0 |
| nagoya2 | 13.2 -> 12.9 / 30.9 -> 30.9 | 0.38 -> 0.39 / 0.70 -> 0.73 | 2.0 -> 2.5 / 2.8 -> 3.5 | 0.2 -> 0.0 |
| nagoya3 | 31.7 -> 31.7 / 61.4 -> 61.4 | 0.36 -> 0.36 / 0.85 -> 0.85 | 2.8 -> 2.8 / 5.4 -> 5.4 | 0.0 -> 0.0 |

### IMU GAP 60-64 s  (v3 -> v4), fused position / velocity / rotation RMSE
| Run | Fused pos RMSE/P95 m | Fused vel RMSE/P95 m/s | Rot RMSE/P95 deg | GNSS-update recovery s |
|---|---|---|---|---|
| tokyo1 | 15.0 -> 15.0 / 32.2 -> 32.2 | 0.29 -> 0.29 / 0.61 -> 0.61 | 1.8 -> 1.8 / 3.1 -> 3.1 | 2.0 -> 2.0 |
| tokyo2 | 11.8 -> 11.8 / 20.1 -> 20.1 | 0.40 -> 0.40 / 0.93 -> 0.93 | 18.4 -> 18.4 / 55.6 -> 55.6 | 2.0 -> 2.0 |
| tokyo3 | 14.1 -> 14.1 / 26.7 -> 26.7 | 0.22 -> 0.22 / 0.48 -> 0.48 | 2.0 -> 2.0 / 3.3 -> 3.3 | 2.0 -> 2.0 |
| nagoya1 | 17.6 -> 13.1 / 33.0 -> 24.3 | 0.96 -> 0.96 / 1.07 -> 1.07 | 14.1 -> 14.1 / 16.5 -> 16.5 | 2.0 -> 2.0 |
| nagoya2 | 15.1 -> 15.1 / 32.7 -> 32.7 | 0.34 -> 0.34 / 0.70 -> 0.70 | 2.3 -> 2.3 / 4.6 -> 4.6 | 2.0 -> 2.0 |
| nagoya3 | 32.8 -> 32.8 / 61.4 -> 61.4 | 0.41 -> 0.41 / 0.87 -> 0.87 | 2.8 -> 2.8 / 5.4 -> 5.4 | 2.0 -> 2.0 |


## Validation

New unit tests (`FusionProcessorSyntheticTest.PostGapReanchor*`: OFF default,
re-anchor only after the horizon and only for a NIS-rejected FLOAT/FIXED, state
effects limited to position and its covariance, steady-state rejections and
coarse-class rejections untouched, accepted updates unaffected), the OFF default
in `OnlineRtkImuTest.VelocityConsistencyCandidateDefaultsAreOff` and the
comparator test for the new candidate name. Release build, Python bindings off:
`run_tests` 1,591 cases, 1,524 pass, 67 skipped (missing fixtures), 0 failed;
`gnss_online_tests` 13/13; `test_pva.py` and `test_pva_comparison.py` pass.
Broader `ctest` lanes were not run.

## Limits and unverified

* Six development runs, all used in development and in the comparison; no holdout.
* The rule fires only for a NIS-rejected FLOAT/FIXED after a gap > 1.0 s; a
  rejected first SPP update is not re-anchored, and a mis-reported FLOAT (Tokyo
  2) is trusted. Whether the horizon matters for dropouts of 1-10 s in normal
  runs was only observed (two real cases, both improved), not swept.
* Timing is host-load dependent; the contemporaneous-`none` comparison is the
  meaningful one but the contract's literal gate refers to the recorded control.
* Nagoya 1 reverse start (rotation 31-46 deg) and Tokyo 2 IMU-gap rotation
  (18.4 deg) remain, as in v3.
* Diagnostic instrumentation (per-update innovation/covariance dump) was a
  scratch copy of the fusion source and is not committed.

## Quiet-host re-measurement of the timing gate (2026-10-09, cloud)

The five gate-4 failures above came from a host loaded by other jobs. Gate 4
was re-measured on an otherwise idle 4-vCPU cloud host (Intel Xeon 2.10 GHz),
running control and candidate side by side. Same frozen contract (SHA256
`fbe3903c...`, freeze `066e4a60` before HEAD) and the same comparator
(`compare_online_pva.py --candidate-name velocity_consistency_v4 --contract
docs/online_pva_candidate_v5.md`), with no threshold changes. Compact record:
[online_pva_decision_v5_quiet_host.json](online_pva_decision_v5_quiet_host.json).

* Tree: develop `c37979c3`, clean. Release build, GCC 13.3.0, Python bindings
  and tests off. Replay binary SHA256 `d853ff83...` (different toolchain, so a
  different hash from the local `001bc66b...`).
* Control: develop default configuration (`--candidate none`) from the same
  binary. The changes between the freeze and `c37979c3` are only the default-OFF
  CLAS PAR frequency gate (`ppp_ar.cpp`, `ppp_env_overrides.*`), which is not on
  the PVA path.
* 36 full replays: 6 runs x {normal, GNSS outage 60-70 s, IMU gap 60-64 s} x
  {none, v4}. All 36 passed. At most 3 ran at once, with each none job queued
  next to its v4 job. 1-minute load average median 2.96, max 3.07, nothing
  else running.
* Gate 7 check: all 180 RMSE/P95 values (fused/RTK position and velocity, and
  rotation, 18 run-scenarios x none/v4) equal the recorded tables above at
  printed precision. So this host reproduces the recorded control and
  candidate. A bit-level CSV comparison with the local recorded set was not
  possible here, because those CSVs are not available in the cloud.

**Result: Go, 0 of 558 gates fail** (18 run/scenarios, targeted rotation
improvement present). Processor P95, candidate / same-host control:
min 0.956, mean 1.033, max 1.128 (gate allows 2.0). The five runs that failed
against the recorded control:

| Run | recorded control ms | quiet-host control ms | quiet-host v4 ms | v4 / quiet control | v4 / recorded control |
|---|---:|---:|---:|---:|---:|
| tokyo1 | 6.27 | 5.92 | 6.07 | 1.03 | 0.97 |
| tokyo2 | 7.74 | 7.57 | 7.54 | 1.00 | 0.97 |
| tokyo3-imu_gap | 8.14 | 10.11 | 10.20 | 1.01 | 1.25 |
| nagoya1-imu_gap | 6.24 | 5.56 | 5.75 | 1.03 | 0.92 |
| nagoya2-imu_gap | 5.75 | 5.93 | 6.17 | 1.04 | 1.07 |

The earlier failures were host contention, not candidate cost. On a quiet host
the candidate costs about 3% at P95 (the earlier contemporaneous estimate was
6% under load). With this, every gate in the contract passes. The
production default is still unchanged. Whether to switch it is a separate
decision: these six runs are development data with no holdout, and Tokyo 2
is worse than v3 for about 30 s after the outage (see above).
