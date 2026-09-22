# Fixed-lag covariance and recovery validation

Status: **complete** (2026-09-22). All six existing PPC runs and all 42 native
comparisons completed successfully. No positioning-accuracy improvement is claimed.
Baseline: `eff1a3c`; candidate branch: `feat/fixed-lag-covariance-holdout`.

PR integration note: the tested change was committed as `038e13e` and then
cherry-picked onto `develop` at `34858d1` on `fix/fixed-lag-covariance-recovery`.
The four scoring tests pass on that branch. Native test and replay evidence
below describes the original frozen source, not the integrated PR head;
native integration validation remains pending for the draft PR.

The user explicitly cancelled unused-data validation. This supersedes that
clause of the registered goal. Only existing PPC development data are used;
no new dataset acquisition or held-out evaluation is in scope. The user
subsequently authorized publishing a PR; merging remains out of scope.

## Scope and implementation

1. Update workspace entry points and verify build/data locations. The workspace
   `README.md` now identifies repositories, worktrees and current local paths;
   `HANDOFF_TO_CODEX.md` labels the July handoff as historical.
2. Provide covariance for the **reported ECEF antenna position**. The default-off
   `FGOConfig::compute_fixed_lag_position_covariance` and parity CLI option
   `--fixed-lag-covariance` capture FLOAT covariance at each actual position
   revision, including the final window/tail snapshot. FIX output uses its
   integer-conditioned covariance; held and reoptimized FIX positions carry
   their own graph marginal. Pose covariance is projected through rotation
   and the lever arm. Missing, nonfinite or materially indefinite covariance
   remains NaN. Only tiny numerical negative eigenvalues are clipped to zero.
3. Correct recovery semantics: require consecutive clean FIX evidence, reset
   interrupted streaks, reject continuing hard-suspect evidence, handle a
   one-epoch recovery configuration, require positive covariance and ignore
   reset-generation changes from unavailable estimates.
4. Validate existing PPC runs with frozen settings, record accuracy, wrong FIX,
   recovery delay, P95, missing epochs, covariance coverage and processing cost.

`--dump-shadow-csv` exports native statuses and full-precision positions without
reference matching. `--dump-csv` is a separate reference-matched scoring aid.
The shadow schema preserves SPP/FLOAT/FIXED as native numeric values 1/3/4 and
includes `solution_latency_s`, `reset_generation` and
`causal_provenance_verified`. Small positive traces are not rounded to zero.

## Causality and accuracy limits

The parity harness's initial static leveling and heading latch can use later
observations. Their known lookahead is now included in exported latency,
without changing the solution. Smoothing latency alone is insufficient:
`buildSecondaryCodeTable` scans whole rover/base files and base matching can
interpolate a future base epoch. The batch export therefore explicitly sets
`causal_provenance_verified=0`.

The updated consumer treats unverified, blank or malformed provenance as
absent, including reset-generation metadata. It also withholds estimates
before their declared latency has elapsed. Legacy CSVs without the provenance
column retain their existing producer contract. These checks do not certify
the native RTK application's entire preprocessing chain for online operation.
A streaming frontend and prefix-invariance audit remain separate future work.

Conditional FIX covariance assumes the accepted integer hypothesis and model;
it does not estimate the probability that those integers are correct. All 1,002 wrong FIX epochs across the six runs have trace below
4 m². Small covariance alone is therefore unsuitable as an accuracy check. No threshold was tuned to
hide these failures. Covariance remains opt-in and this batch FGO shadow is
**not promoted** to real-time recovery authority.

RTK and FGO also consume the same GNSS observations. Agreement between them
is a consistency check, not independent ground truth or evidence excluding
errors common to both estimators.

## Frozen comparison rules

- Compare baseline, candidate covariance OFF and candidate covariance ON with
  identical inputs and shipping settings. Preserve epoch grid, positions,
  FIX status, ratio and fixed-count columns. Compare full-precision OFF/ON
  shadow positions/statuses as a stronger invariant.
- Use 3D error **>2 m** for wrong FIX; also report the fraction below 0.5 m,
  horizontal P95, correct FIX count and missing/unmatched inputs.
- Recovery delay is the time from the last wrong FIX in an event to the next
  FIX with 3D error <=2 m. Report unrecovered events instead of dropping them.
- Promotion requires no increase in wrong FIX or missing outputs and no
  decrease in correct FIX. Report per-run runtime/P95 tradeoffs regardless.
- Count missing epochs against native reader/processed **inputs**, before
  problem construction and output filtering. FGO uses its logged rover-input
  count; RTK sums exact-base, interpolated-base and skipped rover counters.
  The metric is `missing_or_unmatched_epochs`, not a claim of solver failure.
- Truth is used by the offline scorer, not by the native RTK command. The
  parity smoke without `--ref` must produce byte-identical shadow output.

The FGO runner pins lag 5 s, multi-frequency, partial AR, hold, elevation 25°,
SNR 30, tactical IMU, CMC 0.75, CP hold residual 2 m, exception recovery,
DDPR anchor, FDE, elevation-dependent variance, and demotion distances/settings
5/25/5. Explicit existing project lever arms are Tokyo `(0.31,0,0.55)` and
Nagoya `(0.593,-0.670,-1.216)`, identical in each pair. Exact argv is saved.
RTK uses the frozen `low-cost` preset and default residual integrity policy.

## Completed development results

These are existing development runs, not generalization/held-out evidence.
All displayed FGO values are unchanged across baseline/OFF/ON. All completed
pairs pass the exported-column and full-precision position/status invariants.

| FGO run | Inputs / matched outputs | Wrong / correct FIX | <0.5 m of outputs | Horizontal P95 m | Recovery P95 s / unrecovered |
| --- | ---: | ---: | ---: | ---: | ---: |
| Tokyo1 | 11,928 / 11,905 | 388 / 5,536 | 49.845% | 16.284 | 64.68 / 2 |
| Tokyo2 | 9,151 / 9,147 | 226 / 6,424 | 77.949% | 3.392 | 53.68 / 0 |
| Tokyo3 | 15,301 / 15,294 | 112 / 10,851 | 68.040% | 5.981 | 143.32 / 0 |
| Nagoya1 | 7,602 / 7,529 | 37 / 3,039 | 46.062% | 17.352 | 84.16 / 0 |
| Nagoya2 | 9,451 / 9,425 | 178 / 3,408 | 34.801% | 43.577 | 186.96 / 0 |
| Nagoya3 | 5,201 / 5,201 | 61 / 1,456 | 15.401% | 14.811 | 57.25 / 2 |

| FGO run | Positive covariance / outputs | Positive FIX covariance / FIX | Solver OFF / ON seconds | Missing/unmatched inputs |
| --- | ---: | ---: | ---: | ---: |
| Tokyo1 | 11,905 / 11,905 | 5,924 / 5,924 | 90.808 / 347.087 | 23 |
| Tokyo2 | 9,130 / 9,147 | 6,650 / 6,650 | 92.190 / 390.292 | 4 |
| Tokyo3 | 15,294 / 15,294 | 10,963 / 10,963 | 138.694 / 656.100 | 7 |
| Nagoya1 | 7,529 / 7,529 | 3,076 / 3,076 | 72.458 / 340.629 | 73 |
| Nagoya2 | 9,185 / 9,425 | 3,586 / 3,586 | 71.636 / 279.719 | 26 |
| Nagoya3 | 5,201 / 5,201 | 1,517 / 1,517 | 32.262 / 112.649 | 0 |

Tokyo2's 17 missing covariance rows are FLOAT, near logged singular/update
failures at solution indices 6227–6245 (two rows inside that interval are
valid). Both scoring and shadow CSVs retain missing values. The per-run
`covariance_missing_epochs.json` records indices/times and observed log context;
no per-call covariance exception reason is exported. Nagoya2 has another
240 missing covariance rows, all FLOAT; these remain missing rather than being
replaced by unjustified values. All FIX rows across all six runs have positive
covariance. None of the six runs has a native NONE or nonfinite position output.

For RTK, baseline/candidate **without shadow** are POS-byte identical, and the
candidate **with explicitly unverified shadow** is POS-byte identical to its
no-shadow output. Thus the provenance gate adds no positioning loss or claimed
accuracy benefit on these runs.

| RTK run | Inputs / matched outputs | Missing/unmatched | Wrong / correct FIX | <0.5 m of outputs | Horizontal P95 m | Recovery P95 s / unrecovered |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Tokyo1 | 11,928 / 10,062 | 1,866 | 282 / 7,893 | 81.067% | 2.256 | 93.68 / 0 |
| Tokyo2 | 9,151 / 8,383 | 768 | 45 / 7,101 | 88.572% | 1.609 | 39.92 / 0 |
| Tokyo3 | 15,301 / 13,539 | 1,762 | 14 / 10,910 | 87.939% | 1.188 | 42.00 / 0 |
| Nagoya1 | 7,602 / 6,715 | 887 | 5 / 4,697 | 73.254% | 1.429 | 46.54 / 0 |
| Nagoya2 | 9,451 / 6,683 | 2,768 | 26 / 4,462 | 74.233% | 5.943 | 290.78 / 0 |
| Nagoya3 | 5,201 / 3,275 | 1,926 | 139 / 2,092 | 70.137% | 10.636 | 128.29 / 0 |

The legacy consumer ignores the new provenance column. Its shadow-enabled
outputs fail the no-loss criterion and must not be presented as a causal
accuracy baseline:

| Legacy RTK with unverified shadow | Wrong / correct FIX | Missing/unmatched | Shadow samples granted authority |
| --- | ---: | ---: | ---: |
| Tokyo1 | 278 / 6,782 | 1,892 | 5,392 |
| Tokyo2 | 43 / 6,854 | 773 | 6,502 |
| Tokyo3 | 4 / 10,078 | 1,760 | 10,904 |
| Nagoya1 | 0 / 4,213 | 905 | 3,047 |
| Nagoya2 | 14 / 2,876 | 2,563 | 2,953 |
| Nagoya3 | 139 / 1,921 | 1,932 | 966 |

Post-filtering can change the epoch grid and positions as well as status;
paired FIX-demotion counts alone do not describe the entire effect.
Timing is local development evidence, with RTK replay sometimes overlapping
FGO processing, not an isolated hardware benchmark.

## Tests and build limitations

| Check | Result |
| --- | --- |
| Integrity manager/health/realtime suites | 47 passed; four new failures reproduced before the fix |
| New covariance tests | 6 passed: PSD, numerical lever-arm Jacobian, window/tail, frozen FIX, report-only replacement, reoptimized marginal |
| Complete GTSAM test file | Candidate 177/179; baseline 171/173, same two existing failures |
| Native CLI tests | 4 passed on baseline and final candidate |
| Comparison/scoring tests | 4 passed, including input-based missing denominator |
| Controlled native shadow-contract cases | 7 passed |

The two existing GTSAM failures are
`GnssFirstStagingRemainsCholeskyWhenSelectorIsCarried` and
`CapturesUnanchoredComponentZeroColumnAndExactKeyReference`; the latter expects
a type name without MSVC's `class` prefix. The all-app dependencies of
`gnss_run_tests` fail on unrelated Windows issues: `unistd.h` in
`gnss_pos_vel_pdc`/`gnss_smartphone_fgo`, and MSVC nesting depth in `gnss_fuse`.
Focused test executables link the same built libraries and inherit the
repository's existing GTSAM `/FORCE:MULTIPLE` link requirement. This is not a
full-project CTest pass.

The final 60-epoch smoke preserves baseline/OFF/ON output and, without `--ref`,
produces byte-identical shadow output. Known initial lookahead is 11.8 s.
Controlled contract tests cover unverified/blank/malformed provenance, future
availability, SPP, an asserted verified marker and legacy CSV format. All
seven POS outputs match the plain smoke. The asserted marker is only a test
fixture: it admits one sample after initialization lookahead elapses, and does
not assert that the batch PPC producer is causally verified. A separate future-
latency/generation-999 case exposes neither the sample nor its generation.
Reports distinguish finite `independent_position_valid` telemetry from
`shadow_health_qualified` authority.

## Reproduction and evidence

Local data: `E:/rtklib_v2_ws_data/PPC-Dataset`; GTSAM 4.3 navigation installation:
`E:/gtsam/install`; compiler: MSVC 14.44. Python bindings are disabled. The old
MSVC 14.38 build cache and non-GTSAM binaries are not used as verification.

Baseline worktree: `E:/gnsspp-integrity-baseline-source`, detached at `eff1a3c`;
build: `E:/gnsspp-build-integrity-baseline`. Candidate build:
`E:/gnsspp-build-integrity-covariance`. Baseline solver libraries are unchanged.
Two CLI-only patches are applied equally:
[explicit lever arm](use_cases/records/fixed_lag_covariance_baseline_calibration.patch)
and [MSVC parser nesting](use_cases/records/fixed_lag_covariance_baseline_msvc.patch).
Canonical `gnss_fgo_parity` and `gnss_solve` targets build successfully.

From the repository root in PowerShell (use new output directories for a new
inference run; `--resume` only accepts completed, hash-verified matching runs):

```powershell
$env:PATH = 'E:/gtsam/install/bin;C:/vcpkg/installed/x64-windows/bin;' + $env:PATH
python scripts/analysis/run_fixed_lag_covariance_regression.py --baseline-bin E:/gnsspp-build-integrity-baseline/apps/Release/gnss_fgo_parity.exe --candidate-bin E:/gnsspp-build-integrity-covariance/apps/Release/gnss_fgo_parity.exe --dataset-root E:/rtklib_v2_ws_data/PPC-Dataset --output-dir E:/rtklib_v2_ws_output/integrity_covariance/full_final --resume
python scripts/analysis/run_integrity_covariance_replay.py --baseline-bin E:/gnsspp-build-integrity-baseline/apps/Release/gnss_solve.exe --candidate-bin E:/gnsspp-build-integrity-covariance/apps/Release/gnss_solve.exe --dataset-root E:/rtklib_v2_ws_data/PPC-Dataset --shadow-root E:/rtklib_v2_ws_output/integrity_covariance/full_final --output-dir E:/rtklib_v2_ws_output/integrity_covariance/realtime_final --resume
```

Evidence root: `E:/rtklib_v2_ws_output/integrity_covariance/`.

- `full_final/{city}_run{n}`: FGO CSVs, truth-free shadows, logs, run records,
  comparisons and per-run covariance accuracy/missing-value audits.
- `realtime_final/{city}_run{n}`: four RTK variants, POS/telemetry/logs and metrics.
- `smoke_final`, `realtime_smoke_final`, `shadow_contract_checks`: final smoke
  and controlled native input-contract evidence.
- `source_manifest_final.json`: frozen inference source hashes;
  `source_manifest_reporting.json`: subsequent report-only corrections.
- `build_manifest.json`: compiler/CMake settings, executable/library/GTSAM DLL
  hashes and the focused test CMake projects. Each `.run.json` also records
  argv, binary/input/output hashes, PID/state and wall time.
- `input_epoch_inventory.json`: direct RINEX epoch counts and reference row/
  GPS-week counts for all six runs. Tokyo1 and Nagoya1 have more reference
  rows than rover observations; these denominators are deliberately distinct.
- `evidence_audit.json`: final audit of all 42 completed runs: binary/input/output
  hashes, input denominators, solution invariants, POS byte equality, provenance
  rejection and unchanged inference source hashes. No runs remain pending.
- [Portable validation record](use_cases/records/fixed_lag_covariance_validation.json):
  all six result pairs, covariance audits, source/build manifests, input inventory
  and evidence hashes, retained in the repository without raw observations.
- Test evidence in `E:/rtklib_v2_ws_tmp/`: `integrity_unit_before.json` (8 pass,
  4 fail in the reproduction subset), `integrity_unit_after.json` (47 pass),
  `fgo_all_tests.*`, `fgo_baseline_all_tests.*`, `integrity_final_cli_tests.log`.

Earlier `smoke_paired`/`full_paired` artifacts predate the causality correction;
one ON run was deliberately interrupted and marked accordingly. They are not
final validation evidence. An early report also used output count as a missing-
epoch denominator. The corrected scorer uses input counts and is regression
tested; native inference did not change.

## Completion decision

All four scope items are complete: workspace entry points, covariance matching
the reported position, recovery/availability semantics, and the six-run frozen
development comparison. Both replay processes exited successfully. Hash-checked
resume regenerated every report with the corrected input denominator without
rerunning inference. The final audit covers all 18 FGO and 24 RTK executions.

The covariance implementation passes its correctness checks and preserves
positions, but costs 3.49–4.73 times the OFF solver time on these runs. It stays
default-off. The recovery state-machine fixes pass focused tests; real-data
accuracy improvement is not established because this batch shadow cannot be
granted causal authority. All legacy shadow-enabled runs lose correct FIX
epochs and fail the declared no-loss gate. No promotion or threshold tuning
was performed. Unused-data validation remains cancelled.
