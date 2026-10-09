# Velocity-consistency candidate v4 results: No-Go (4 of 558 gates)

`velocity_consistency_v3` fails the frozen contract in
[online_pva_candidate_v4.md](online_pva_candidate_v4.md) (committed as
`5a6f48ab`, code including the cherry-picked `f91eb44b`, before the comparison).
Production defaults are unchanged; the candidate stays an explicit opt-in
(`--candidate velocity_consistency_v3`). Machine-readable gates:
[online_pva_decision_v4.json](online_pva_decision_v4.json) (compact; the 1.6 MB
full decision is in the local output directory, hash recorded there).

Control is the recorded develop `c2eb06f7` default set. Candidate `none` built
from the candidate tree is bit-identical to it in every deterministic CSV field
on all 18 runs (gate 7 passes), so no control was regenerated. Replay binary
SHA256 `31e2859ae164c6b1a9a2bcd512778119b6420c363ed0abd71902f2f8ce045708`
(also used for the contemporaneous candidate-`none` set), clean tree. 36 replays
(six runs x normal, GNSS outage 60-70 s, IMU gap 60-64 s, control and
candidate), all truth-matched, identical input hashes.

## Decision

558 gates over 18 run/scenarios. Against the recorded control 21 fail: 17 are
the processor P95 timing gate and 4 the GNSS-update recovery gate. The host was
loaded by other jobs (load average 4-16); against the contemporaneous
candidate-`none` run of the same binary, which is bit-identical to the control,
no timing gate fails (candidate/none P95 ratio 0.92-1.61, mean 1.17; the
isolated second fusion filter costs a few ms) and 4 gates fail:

| Run | Gate | Control -> candidate |
|---|---|---|
| Tokyo 2, GNSS outage | first GNSS position update after the outage | 0.0 -> 0.2 s |
| Tokyo 3, GNSS outage | same | 0.0 -> 0.2 s |
| Nagoya 2, GNSS outage | same | 0.0 -> 0.2 s |
| Nagoya 1, GNSS outage | same | 0.0 -> 18.0 s |

Everything else passes in all 18 run/scenarios, including what failed in v1/v2:
RTK position/velocity RMSE and P95 (all-output and common-valid cohorts) are
bit-identical to the control (the RTK columns below do not move), fused
position, velocity and rotation RMSE/P95 are all at or below the control,
coverage and first-heading/fresh-attitude times are unchanged, and the targeted
rotation improvement holds on all six normal runs.

## Why the recovery gate fails

The first epoch after the outage (t = 70.0 s) is an RTK FLOAT epoch. The
control applies it (no gate), the candidate's position NIS gate (9 per
observation) rejects it after 10 s of unaided propagation, and the next SPP epoch
(0.2 s later) is accepted. Nagoya 1 is rejected for 18 s: FLOAT is rejected
and the FLOAT/FIXED re-anchor (d) needs 30 consecutive FLOAT rejections. This is
the cost of enabling gate (c) and of the optimism that remains in the converged
FLOAT sigma (see [rtk_float_covariance.md](rtk_float_covariance.md)); the
contract does not allow trading it against the large gain elsewhere and the
candidate was not tuned after the result.

## Full-run results, control -> candidate

### NORMAL  (control -> candidate velocity_consistency_v3)
| Run | Fused pos RMSE/P95 m | Fused vel RMSE/P95 m/s | Rot RMSE/P95 deg | RTK pos RMSE/P95 m | RTK vel RMSE/P95 m/s | fused / heading avail % | first fresh/heading s |
|---|---|---|---|---|---|---|---|
| Tokyo 1 | 71.7 -> 14.4 / 155.2 -> 29.7 | 7.05 -> 0.30 / 14.62 -> 0.58 | 105.1 -> 1.8 / 171.1 -> 3.0 | 32.2 -> 32.2 / 61.7 -> 61.7 | 2.10 -> 2.10 / 3.29 -> 3.29 | 99.92/99.36 -> 99.92/99.36 | 2.0/15.2 -> 2.0/15.2 |
| Tokyo 2 | 74.2 -> 10.3 / 179.5 -> 19.7 | 6.85 -> 0.28 / 14.14 -> 0.51 | 100.3 -> 2.0 / 170.0 -> 2.2 | 19.5 -> 19.5 / 29.7 -> 29.7 | 2.26 -> 2.26 / 3.71 -> 3.71 | 99.89/98.72 -> 99.89/98.72 | 2.0/23.4 -> 2.0/23.4 |
| Tokyo 3 | 41.9 -> 14.1 / 115.0 -> 26.6 | 3.22 -> 0.23 / 8.22 -> 0.51 | 63.5 -> 1.7 / 144.4 -> 2.8 | 18.6 -> 18.6 / 31.9 -> 31.9 | 2.10 -> 2.10 / 1.76 -> 1.76 | 99.93/98.77 -> 99.93/98.77 | 2.0/37.6 -> 2.0/37.6 |
| Nagoya 1 | 49.3 -> 17.7 / 118.1 -> 32.8 | 4.61 -> 1.02 / 9.84 -> 1.25 | 88.2 -> 31.1 / 168.7 -> 61.3 | 25.7 -> 25.7 / 29.0 -> 29.0 | 2.51 -> 2.51 / 4.26 -> 4.26 | 99.82/96.50 -> 99.82/96.50 | 2.0/52.0 -> 2.0/52.0 |
| Nagoya 2 | 81.0 -> 22.5 / 168.0 -> 53.9 | 3.73 -> 0.46 / 9.96 -> 0.88 | 18.5 -> 2.5 / 39.8 -> 4.4 | 37.7 -> 37.7 / 81.8 -> 81.8 | 2.30 -> 2.30 / 5.09 -> 5.09 | 99.89/97.79 -> 99.89/97.79 | 2.0/41.8 -> 2.0/41.8 |
| Nagoya 3 | 58.9 -> 31.9 / 128.1 -> 61.4 | 5.54 -> 0.35 / 11.72 -> 0.85 | 105.0 -> 2.9 / 170.9 -> 5.5 | 43.7 -> 43.7 / 101.1 -> 101.1 | 2.47 -> 2.47 / 5.13 -> 5.13 | 99.81/96.54 -> 99.81/96.54 | 2.0/36.0 -> 2.0/36.0 |

### GNSS OUTAGE 60-70 s  (control -> candidate velocity_consistency_v3)
| Run | Fused pos RMSE/P95 m | Fused vel RMSE/P95 m/s | Rot RMSE/P95 deg | RTK pos RMSE/P95 m | RTK vel RMSE/P95 m/s | fused / heading avail % | first fresh/heading s |
|---|---|---|---|---|---|---|---|
| Tokyo 1 | 55.1 -> 14.1 / 130.9 -> 27.9 | 6.11 -> 0.28 / 11.78 -> 0.56 | 104.9 -> 1.7 / 171.2 -> 2.8 | 29.7 -> 29.7 / 58.9 -> 58.9 | 2.64 -> 2.64 / 3.89 -> 3.89 | 99.92/99.36 -> 99.92/99.36 | 2.0/15.2 -> 2.0/15.2 |
| Tokyo 2 | 72.5 -> 8.5 / 187.3 -> 18.9 | 4.82 -> 0.27 / 11.98 -> 0.63 | 28.5 -> 2.6 / 68.1 -> 2.5 | 19.4 -> 19.4 / 26.4 -> 26.4 | 2.42 -> 2.42 / 4.99 -> 4.99 | 99.89/98.72 -> 99.89/98.72 | 2.0/23.4 -> 2.0/23.4 |
| Tokyo 3 | 27.2 -> 14.4 / 31.9 -> 26.7 | 1.95 -> 0.22 / 1.40 -> 0.48 | 12.7 -> 1.6 / 40.6 -> 2.9 | 16.8 -> 16.8 / 30.9 -> 30.9 | 1.07 -> 1.07 / 1.55 -> 1.55 | 99.93/98.77 -> 99.93/98.77 | 2.0/37.6 -> 2.0/37.6 |
| Nagoya 1 | 53.2 -> 18.4 / 131.9 -> 36.5 | 4.87 -> 1.13 / 10.91 -> 1.30 | 88.7 -> 42.6 / 168.8 -> 146.0 | 26.0 -> 26.0 / 29.3 -> 29.3 | 2.76 -> 2.76 / 4.39 -> 4.39 | 99.82/96.50 -> 99.82/96.50 | 2.0/52.0 -> 2.0/52.0 |
| Nagoya 2 | 55.2 -> 13.2 / 133.2 -> 30.9 | 4.30 -> 0.38 / 9.76 -> 0.70 | 90.8 -> 2.0 / 167.5 -> 2.8 | 37.2 -> 37.2 / 85.7 -> 85.7 | 4.24 -> 4.24 / 13.91 -> 13.91 | 99.89/97.79 -> 99.89/97.79 | 2.0/41.8 -> 2.0/41.8 |
| Nagoya 3 | 75.2 -> 31.7 / 171.7 -> 61.4 | 5.75 -> 0.36 / 10.27 -> 0.85 | 103.2 -> 2.8 / 171.3 -> 5.4 | 45.8 -> 45.8 / 103.7 -> 103.7 | 2.47 -> 2.47 / 6.22 -> 6.22 | 99.81/96.54 -> 99.81/96.54 | 2.0/36.0 -> 2.0/36.0 |

Recovery s (GNSS update / fresh attitude / heading), control -> candidate:
- Tokyo 1: 0.0 / 0.0 / 0.0 -> 0.0 / 0.0 / 0.0
- Tokyo 2: 0.0 / 0.0 / 0.0 -> 0.2 / 0.0 / 0.0
- Tokyo 3: 0.0 / 0.0 / 0.0 -> 0.2 / 0.0 / 0.0
- Nagoya 1: 0.0 / 0.0 / 0.0 -> 18.0 / 0.0 / 0.0
- Nagoya 2: 0.0 / 0.0 / 0.0 -> 0.2 / 0.0 / 0.0
- Nagoya 3: 0.0 / 0.0 / 0.0 -> 0.0 / 0.0 / 0.0

### IMU GAP 60-64 s  (control -> candidate velocity_consistency_v3)
| Run | Fused pos RMSE/P95 m | Fused vel RMSE/P95 m/s | Rot RMSE/P95 deg | RTK pos RMSE/P95 m | RTK vel RMSE/P95 m/s | fused / heading avail % | first fresh/heading s |
|---|---|---|---|---|---|---|---|
| Tokyo 1 | 63.4 -> 15.0 / 148.0 -> 32.2 | 5.83 -> 0.29 / 11.39 -> 0.61 | 104.9 -> 1.8 / 171.3 -> 3.1 | 31.2 -> 31.2 / 63.3 -> 63.3 | 2.36 -> 2.36 / 3.72 -> 3.72 | 99.67/98.39 -> 99.67/98.39 | 2.0/15.2 -> 2.0/15.2 |
| Tokyo 2 | 52.2 -> 11.8 / 111.5 -> 20.1 | 6.07 -> 0.40 / 12.06 -> 0.93 | 102.7 -> 18.4 / 170.6 -> 55.6 | 14.0 -> 14.0 / 23.6 -> 23.6 | 1.66 -> 1.66 / 2.21 -> 2.21 | 99.57/97.89 -> 99.57/97.89 | 2.0/23.4 -> 2.0/23.4 |
| Tokyo 3 | 59.3 -> 14.1 / 90.2 -> 26.7 | 6.60 -> 0.22 / 12.34 -> 0.48 | 104.6 -> 2.0 / 171.2 -> 3.3 | 17.3 -> 17.3 / 31.0 -> 31.0 | 1.53 -> 1.53 / 1.74 -> 1.74 | 99.75/98.57 -> 99.75/98.57 | 2.0/37.6 -> 2.0/37.6 |
| Nagoya 1 | 55.8 -> 17.6 / 130.6 -> 33.0 | 4.91 -> 0.96 / 11.22 -> 1.07 | 83.0 -> 14.1 / 167.4 -> 16.5 | 27.6 -> 27.6 / 30.8 -> 30.8 | 2.57 -> 2.57 / 4.34 -> 4.34 | 99.43/96.09 -> 99.43/96.09 | 2.0/52.0 -> 2.0/52.0 |
| Nagoya 2 | 64.2 -> 15.1 / 146.2 -> 32.7 | 4.41 -> 0.34 / 9.38 -> 0.70 | 104.1 -> 2.3 / 170.7 -> 4.6 | 38.3 -> 38.3 / 85.0 -> 85.0 | 4.34 -> 4.34 / 14.59 -> 14.59 | 99.59/97.46 -> 99.59/97.46 | 2.0/41.8 -> 2.0/41.8 |
| Nagoya 3 | 88.6 -> 32.8 / 192.6 -> 61.4 | 7.26 -> 0.41 / 14.31 -> 0.87 | 106.7 -> 2.8 / 170.3 -> 5.4 | 43.8 -> 43.8 / 100.7 -> 100.7 | 2.04 -> 2.04 / 3.82 -> 3.82 | 99.25/95.94 -> 99.25/95.94 | 2.0/36.0 -> 2.0/36.0 |

Recovery s (GNSS update / fresh attitude / heading), control -> candidate:
- Tokyo 1: 2.0 / 2.0 / 19.4 -> 2.0 / 2.0 / 19.4
- Tokyo 2: 2.0 / 2.0 / 11.4 -> 2.0 / 2.0 / 11.4
- Tokyo 3: 2.0 / 2.0 / 2.4 -> 2.0 / 2.0 / 2.4
- Nagoya 1: 2.0 / 2.0 / 2.4 -> 2.0 / 2.0 / 2.4
- Nagoya 2: 2.0 / 2.0 / 2.4 -> 2.0 / 2.0 / 2.4
- Nagoya 3: 2.0 / 2.0 / 2.4 -> 2.0 / 2.0 / 2.4


Availability is unchanged to the printed precision. Nagoya 1 rotation is still
31-43 deg (reverse start, not addressed by any candidate) but is better than the
control's 88 deg; Tokyo 2 IMU-gap rotation 102.7 -> 18.4 deg is the only
remaining non-converged gap case besides Nagoya 1.

## Ablation used to choose the components (development, normal runs, not gated)

Fused position RMSE (m) / rotation RMSE (deg). `iso` = isolated RTK prior.
Covariance mode 3 everywhere except `none`.

| Variant | Tokyo 1 | Tokyo 2 | Tokyo 3 | Nagoya 1 | Nagoya 2 | Nagoya 3 |
|---|---|---|---|---|---|---|
| control | 71.7 / 105 | 74.2 / 100 | 41.9 / 63 | 49.3 / 88 | 81.0 / 18.5 | 58.9 / 105 |
| covariance only | 36.2 / 105 | 19.6 / 7.6 | 16.3 / 8.1 | 23.3 / 106 | 23.8 / 4.2 | 31.9 / 5.1 |
| + c | 12.5 / 1.8 | 10.0 / 3.0 | 14.3 / 1.9 | 54.0 / 32 | 46.1 / 3.2 | 32.3 / 2.9 |
| + a,c | 12.2 / 1.9 | 10.0 / 3.0 | 13.8 / 1.9 | 54.0 / 37 | 17.3 / 3.0 | 32.3 / 3.5 |
| + a,b,c | 25.5 / 3.2 | 7.8 / 1.7 | 13.3 / 1.7 | 118 / 45 | 16.3 / 4.0 | 28.9 / 2.8 |
| + a,b,c,d | 20.3 / 2.4 | 6.4 / 1.7 | 5.2 / 1.6 | 29.7 / 40 | 14.5 / 4.0 | 26.5 / 2.5 |
| iso + c | 14.7 / 1.8 | 11.6 / 2.0 | 14.4 / 1.7 | 54.6 / 31 | 38.2 / 2.2 | 31.9 / 2.9 |
| iso + c,d (candidate) | 14.4 / 1.8 | 10.3 / 2.0 | 14.1 / 1.7 | 17.7 / 31 | 22.6 / 2.5 | 31.9 / 2.9 |

Reading: the honest covariance alone removes the attitude collapse on three runs
(rotation 63-105 -> 5-8 deg on Tokyo 2, Tokyo 3, Nagoya 3; Nagoya 2 18.5 -> 4.2)
and roughly halves position error, but not Tokyo 1 and Nagoya 1. Without isolation the
RTK columns move (without b: Tokyo 2 RTK position 19.5 -> 17.7, Tokyo 3 18.6 ->
21.0 with a, Nagoya 2 +15 % with covariance alone, RTK velocity 2.10 -> 2.8 on
Tokyo 1), because the fused attitude bootstraps the RTK prior; (b) moves them
further both ways. With isolation they are identical by construction. Component
(a) is neutral to harmful once the covariance is honest; (d) matters for
Nagoya 1 position only.

## Limits and unverified

* Isolation (f) forgoes any RTK benefit from the better fused state; whether the
  RTK filter should consume the corrected state was not evaluated and would need
  its own RTK-gate decision.
* The remaining optimism of the converged FLOAT sigma (RMS z about 4-6) is not
  fixed; the post-outage rejection above follows from it.
* Nagoya 1 reverse start and Tokyo 2 IMU-gap rotation remain.
* Only the six development runs; no holdout. Timing is host-load dependent; the
  contemporaneous-`none` comparison is the meaningful one and it is the same
  binary, so it measures only the candidate's own cost.
* The ablation grid above used a scratch instrumented copy of the replay with
  environment switches (not committed). Its `iso + c,d` normal-run output is
  bit-identical, in every deterministic CSV field, to the committed
  `velocity_consistency_v3` normal-run output on all six runs.

## Validation

New unit tests: `RTKCovarianceConsistencyTest` (4 cases: consistent FLOAT is not
inflated; inflated FLOAT reaches the 3-dof statistic exactly and reports a sigma
comparable to the disagreement; scale grows with disagreement and shrinks with
SPP uncertainty; invalid input reports no evidence), the OFF defaults of the new
RTK and online options, and the comparator test for the new candidate name. The
v2 `FloatGateRecovery*` fusion tests come with the cherry-picked recovery.
Release build, Python bindings off: `run_tests` 1,583 cases, 1,516 pass, 67
skipped (missing fixtures), 0 failed; `gnss_online_tests` 13/13;
`test_pva.py` and `test_pva_comparison.py` pass. The RTK-level behaviour of the
modes is covered by the 36-replay and batch comparisons above, not by a unit
test (the bundled kinematic fixtures are absent in this checkout).
Broader `ctest` lanes were not run.
