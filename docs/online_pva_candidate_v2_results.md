# Velocity-consistency candidate v2 results: No-Go

`velocity_consistency_v1` fails the frozen contract in
[online_pva_candidate_v2.md](online_pva_candidate_v2.md) (committed before the
comparison). Production defaults are unchanged; the candidate remains an
explicit opt-in (`--candidate velocity_consistency_v1`) for reproducibility.
Machine-readable gates: [online_pva_decision_v2.json](online_pva_decision_v2.json).

Control is develop `c2eb06f7` (default, Release, native replay SHA256
`99c56dcdf357d0a53d065245acbd513d54befb8eb18e3f8b8d2477fb8a02af5f`); candidate
binary `ff02d2cfdec99465ece42c498e61fc3582cb067b7800bbbe571939b625681d86`.
36 full replays (six runs x normal, GNSS outage 60-70 s, IMU gap 60-64 s, each
control and candidate), all truth-matched, identical input hashes. Candidate
`none` from the candidate tree is bit-identical to the control in all 60
deterministic CSV fields on all 18 runs.

## Decision

558 gates, 54 failed (24 per cohort on all-output and common-valid, plus 6
timing gates). Passing: coverage (no loss over 0.1 pp), first fresh attitude,
processor P95 (<= 2 x control; normal 5.5-11.5 ms, host contention not
controlled), and the targeted rotation improvement. Failed:

* Nagoya 1, all three scenarios: fused position RMSE 49.3 -> 119.0 m (P95
  118.1 -> 178.3), and RTK position 25.7 -> 30.2 m. The GNSS-outage scenario's
  GNSS-update recovery is also later (0.0 -> 142.8 s).
* RTK position/velocity P95 regress by 2-9% (the exported RTK filter result,
  not the fused one) in Tokyo 2 IMU gap, Tokyo 3 normal/outage/gap and Nagoya
  1/3. Tokyo 3 outage RTK position RMSE 16.8 -> 25.9 m.
* Timing: Nagoya 2 first heading 41.8 -> 42.0 s (one 0.2 s epoch later, all
  three scenarios) and GNSS-update recovery 0.0 -> 0.2 s in Tokyo 2/Nagoya 2
  outage.

Every other fused position/velocity/rotation gate passes.

## Full-run results (normal), control -> candidate

| Run | Fused position RMSE / P95 (m) | Fused velocity RMSE / P95 (m/s) | Rotation RMSE / P95 (deg) |
| --- | ---: | ---: | ---: |
| Tokyo 1 | 71.7 -> 25.9 / 155.2 -> 48.1 | 7.05 -> 1.20 / 14.62 -> 1.26 | 105.1 -> 2.9 / 171.1 -> 6.0 |
| Tokyo 2 | 74.2 -> 13.8 / 179.5 -> 27.0 | 6.85 -> 0.38 / 14.14 -> 0.78 | 100.3 -> 1.4 / 170.0 -> 2.6 |
| Tokyo 3 | 41.9 -> 14.3 / 115.0 -> 26.2 | 3.22 -> 0.28 / 8.22 -> 0.58 | 63.5 -> 1.7 / 144.4 -> 3.0 |
| Nagoya 1 | 49.3 -> 119.0 / 118.1 -> 178.3 | 4.61 -> 1.30 / 9.84 -> 1.61 | 88.2 -> 35.4 / 168.7 -> 103.0 |
| Nagoya 2 | 81.0 -> 18.7 / 168.0 -> 33.8 | 3.73 -> 0.57 / 9.96 -> 1.08 | 18.5 -> 3.1 / 39.8 -> 5.5 |
| Nagoya 3 | 58.9 -> 31.8 / 128.1 -> 66.9 | 5.54 -> 0.58 / 11.72 -> 0.94 | 105.0 -> 2.5 / 170.9 -> 4.5 |

Availability: fused 99.81-99.93% unchanged; heading availability changes by at
most 0.04 pp (Nagoya 1, 96.50 -> 96.46). The GNSS-outage and IMU-gap tables
(same metrics for the other 12 replays, plus recovery times) are in
[online_pva_candidate_v2_scenarios.md](online_pva_candidate_v2_scenarios.md).

## What the failure is

The attitude and velocity fixes work: five of six runs improve position,
velocity and rotation by 2-70x, and the GNSS-latch pathology is gone (rotation
1-6 deg RMSE where the control sat near 100 deg). The contract fails on
Nagoya 1 and on small RTK-filter regressions. On Nagoya 1 the candidate holds
the attitude (rotation 1-5 deg after 300 s) but the fused position locks out:
the fraction of epochs with an accepted GNSS position update is 0.81-0.83
before 700 s, 0.07 in 700-780 s and 0.00 from 780 s to the end, leaving a
~170 m offset. This is a position NIS-gate lockout (the gate rejects every update while the
prediction is wrong; the trusted-FIXED re-anchor needs 30 consecutive FIXED
epochs, which a mostly-FLOAT run never supplies).

Post-hoc diagnostics, not part of the contract and not used to change the
candidate (temporary, uncommitted env switches on the same code; one replay
each, full run):

| Nagoya 1 normal | Fused position RMSE (m) | Fused velocity RMSE (m/s) | Rotation RMSE (deg) |
| --- | ---: | ---: | ---: |
| Control | 49.3 | 4.61 | 88.2 |
| Candidate | 119.0 | 1.30 | 35.4 |
| Candidate, position gate off | 42.9 | 1.57 | 105.9 |
| Candidate, velocity gate off | 95.5 | 2.20 | 107.1 |

Removing the position gate removes the lockout (position beats the control) but
also removes the attitude gain, so on Nagoya 1 the two gates together are what
keeps the attitude from collapsing; and with the position gate off Tokyo 3
changes from 14.3 to 4.1 m position and 1.7 to 4.7 deg rotation. The gates are
therefore a position/attitude trade-off, not a free improvement. This is one
replay per variant and is evidence about mechanism, not a decision.

## Validation

Candidate wiring: unit tests `OnlineRtkImuTest.VelocityConsistencyCandidateDefaultsAreOff`
and `HeadingLatchReanchorsVelocityAndClearsItsCrossCovariance` (new), comparator
tests for the candidate name and the v1-only parity gate. The independent-Doppler
path (component b) has no synthetic unit test; it is exercised only by the
real-data replays above. Release build of the full tree (Python bindings off): `run_tests` passes
1,495 cases (67 skipped), `gnss_online_tests` 13/13, `python_pva_tests` and
`python_pva_comparison_tests` pass. `ctest` passes 146 of 152 lanes. The six
failures are environmental or already failing and not tied to this change:
two smartphone lanes (known on develop), `python_packaging_tests` and
`python_trajectory_bundle_tests` (need the Python binding that this build
disabled), `python_ros2_node_tests` (missing `liblibstatistics_collector.so`),
and `python_cli_tests` (9 failures after pointing `GNSSPP_BUILD_DIR` at the
build: missing `gnss_live`/`gnss_ppp`/station binaries and one QZSS L6 serial
case). These were not re-run on an unmodified develop tree. Outputs are under `rtklib_v2_ws_output/pva_velcons_20261009/`
in the local workspace (`ctl`, `cand`, `off`, `decision`).

## Next experiment (needs a new contract)

Do not reuse this candidate. A defensible follow-up needs a position gate that
cannot lock out (for example gating FIXED positions only, or a
FLOAT-capable re-anchor after a bounded rejection streak) while keeping the
velocity gate and (a)/(b) with its own frozen
thresholds and a component ablation (a, b, c separately) on the same 18 runs.
Nagoya 1's reverse start and the gyro-bias state learned before the latch
remain open.
