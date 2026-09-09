# Phase221: enable explicit native motion plus Doppler combination

Primary agent only; no subagents, native real-data solve, truth/candidate
payload access, MAT input, or submission in this implementation phase.

Removed only the Phase213/217 mutual exclusion in the CLI and GTSAM
backend. Both existing selectors remain default-off. Other admission guards,
factor implementations, noise, robust thresholds, seeds, and timing are
unchanged. The backend error text now names duplicate position motion rather
than claiming all Doppler combinations are forbidden.

The existing Phase171 handoff fixture's combined case now reaches optimize
instead of expecting an exception. It checks both dedicated factor counts,
convergence, and exported finite clock states through the existing assertions.
Standalone cases and malformed Phase213-row cases remain present. Malformed
rows with both selectors on are not a separate fixture in this change.

Replaced the obsolete CLI paired-selector rejection case with a test proving
the pair reaches the downstream pinned raw-recipe guard. This is selector
admission evidence, not a raw-data success claim. Corrected its motion test
module docstring.

Verification:

- Build session 89693 completed with exit 0 for gnss_fgo_imu_no_base and
  gnss_run_tests, with GTSAM enabled.
- Updated Phase217 plus Phase213 CLI tests: 13/13 passed.
- Phase209 plus Phase205 CLI guards: 9/9 passed.
- Focused Phase171/204/205/208/213/216 and IMU time-index tests: 23/23 passed.
- Existing Android/clock/Phase143/164/165/167/194/raw-P/source-clock/stop
  regression selection: 85/85 passed.
- git diff --check passed. This is 108 selected C++ and 22 CLI tests,
  not full CTest. An initial CLI command named nonexistent files and ran
  zero tests; the corrected commands above are the reported evidence.

Next freeze a raw-only H combined experiment using the Phase218 recipe plus
the explicit Phase213 main Doppler flag. Do not change any other solver
setting; validate current source/binary pins and new output-directory identity
before one invocation. Expected graph: 252384 factors, 15700 values,
3139 motion factors and 66685 dedicated main Doppler factors. Accuracy is
unmeasured for the combination; current H best remains 1.2689788175187473 m.
