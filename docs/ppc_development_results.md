# PPC development results — 2026-10-06

The four requested implementation and evaluation deliverables are complete.
The FIX-recovery feature fails its default-adoption gate and remains off.
Evidence uses the existing six PPC development runs; it is not new held-out
validation. No push, pull request or merge was performed.

| Deliverable | Result | Instructions and recorded evidence |
| --- | --- | --- |
| Raw native reproduction | Full six-run RTK/fused/coupled-RTK replay and repeat pass; all 18 POS streams match in every numeric field including status. Historical 26 tiers remain score-only. | [Command/provenance](ppc_native_replay.md), [repeatability](ppc_native_repeatability_verification.json) |
| Wrong FIX classification and recovery | Runtime-only, optional detector/reset/recovery implemented; same-binary full six-run OFF/ON evaluation returns **NO_GO**. Default remains OFF. | [Usage and decision](fix_recovery_guard.md), [paired evaluation](ppc_fix_recovery_evaluation.json), [full default parity](ppc_native_default_parity_verification.json) |
| Received-event RTK/IMU | Typed API and installed stdin executable pass causal real-position prefix checks, missing/late base, IMU outage/reinitialization and delayed rover cases. | [Input protocol/reproduction](online_rtk_imu.md), [600-epoch verification](ppc_online_verification.json) |
| Public Python observation analysis | CSV/JSON and 33 per-satellite plots generated from 300 Tokyo 1 epochs (9,432 rows), with explicit units, clock handling, missing-data behavior and slip evidence. | [API/example](python_observation_analysis.md), [verification](ppc_python_observation_verification.json) |

## Integration and scope

Main implementation branch: `feat/ppc-reproduction-integrity-online-tools`.
The separate development worktree is `E:/gnsspp-goal-development`. The first
full replay is frozen at `b34cf3e7`; the recovery experiment at `8235764f`.
Each manifest archives actual source contents, binaries, build settings,
inputs, arguments and output hashes. Later documentation and regression fixes
do not rewrite these experiment records.

All six default-off RTK streams match the original native baseline, despite
the comparison builds using different optional GTSAM configurations. The final
integrated GTSAM-enabled RTK/fused/coupled-RTK smoke also matches all three
original 120-epoch streams numerically:
[integrated smoke record](ppc_integrated_smoke_verification.json). This bounded
smoke does not replace the full 18-stream repeat proof.

The online verifier uses simulated, explicitly declared reception times from
raw RINEX/IMU inputs. Normal typed processing yields 600 valid RTK-stream
positions (including SPP fallbacks), 590 fresh fused positions and 103 tight
time updates. Its 300-epoch prefix matches all numerical/metadata fields
except measured processing time. The GPS-only RTCM CLI emits all 600 rows
while stdin remains open and has 28 tight updates; 492 RTK fields are NONE.
The results establish causality and recovery mechanics, not full-run accuracy,
live network performance or integrity. Existing batch preprocessing retains
its future-input dependencies and is not advertised as causal.

The observation residual is a corrected-code geometric diagnostic after
receiver-clock fitting, not the solver's admitted postfit residual. Carrier
continuity and common-clock flags retain their source evidence and limitations.
The analyzer never reads the PPC reference trajectory.

## Regression checks

The Windows Release build at `E:/gnsspp-goal-build` builds all default targets,
including libraries, apps, examples, Python bindings and native tests. The
GTSAM-enabled main build at `E:/gnsspp-build-integrity-covariance` also rebuilds
`gnss_solve`, `gnss_fuse` and `gnss_online` successfully.

- All C++ CTest suites pass. The main `run_tests` suite has 1,339 cases:
  1,274 pass and 65 skip for unavailable fixtures/backends. Separate suites
  include nine online processor tests, four historical RTCM time-context tests
  and seven recovery guard tests, all passing.
- CLI: 223 cases total, 158 passed and 65 skipped for unavailable fixtures/
  platform features. Benchmark scripts: 181 total, 180 passed and one skipped.
- Public bindings: four passing cases, eight skipped historical-data cases.
  The PPC extension example is independently exercised on actual observations.
- Observation analyzer: 14 passing cases; integrity audit: seven; native replay:
  12; existing PPC reproduction: 32. CMake install/packaging tests pass.
- The full CTest run registers 137 lanes: 128 pass and nine pre-existing
  smartphone environment/fixture lanes fail. Failed lanes were rerun serially.
  This is a partial broad-suite pass, not a green whole-repository result.
- ROS2 dependencies and its runtime node are unavailable on this Windows build;
  no ROS2 runtime verification is claimed. Historical RTK fixtures were not
  replaced with PPC inputs.

The nine remaining CTest failures are outside these four deliverables and
have no source changes in this branch:

| Lanes | Observed cause |
| --- | --- |
| `python_smartphone_native_fgo_v2_1_confirmation_tests`, `python_smartphone_native_fgo_v3_heading_optional_tests` | Immutable SHA-256 contracts see Windows CRLF checkout bytes; the unchanged Git blobs match the recorded LF hashes. The sealed records and hash expectations were preserved. |
| `python_smartphone_raw_quality_control_tests`, `python_smartphone_wls_stability_selector_eval_tests`, `python_smartphone_wls_residual_tests`, `python_smartphone_wls_residual_v2_tests`, `python_smartphone_wls_multi_phone_ensemble_tests`, `python_smartphone_wls_test_batch_tests` | Existing scripts import the POSIX-only Python `resource` module, unavailable in native Windows Python. |
| `python_smartphone_native_fgo_pdc_factor_hypotheses_tests` | Pre-existing sealed artifact `output/smartphone-r5/native-fgo-pdc-factor-audit-v1/factor_audit.json` is absent in this worktree. |

Logs and their hashes are retained in
[ppc_development_regression_verification.json](ppc_development_regression_verification.json).
Windows process probing, test path/dispatcher portability, normalized rendered
HTML hashing and a sibling-module import collision were corrected during
regression checks. The package and CLI checks were rerun after those corrections.
Closed holdouts and the paused smartphone lane were not materialized or tuned.
