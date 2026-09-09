# Phase285 completion inspection

Native session 45664 exited 0; PID 3237261 is terminal. One native invocation
took 289.1342649429571 s. No rerun or scoring occurred.

Summary shows pool 105188, old P 101916, new P 102050, recovered 687,
removed 553, unchanged 101363. Both conservation identities hold. Actual
P count in epochs.pseudorange_factors is 102050; total graph factors 252518
equals 252384 - 101916 + 102050. Main converged in 40 iterations.

Initial structural-record generation aborted on an overly broad baseline
comparison: epochs includes pseudorange_factors, which intentionally changed.
Inspection confirmed all other fields of epochs are unchanged, including
problem/output 3140 and TDCP 69270. GNSS-first, IMU initialization/noise/time
offset, output contract and raw UTC contract compare equal to Phase234.
The full comparison must retain epochs=false and explicitly account for
only the expected P-count difference, not relabel the entire object equal.

That aborted record attempt read candidate bytes for hashing once and lines
once (3140 including header), without interpreting coordinates. No structural
JSON was written and no truth was read. Further payload opens must be counted
honestly. Next finish a structural record with the corrected comparison scope
and full termination/coverage evidence before freezing an evaluator.
