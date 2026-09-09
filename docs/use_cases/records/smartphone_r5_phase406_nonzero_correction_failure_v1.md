# Phase406 — nonzero correction control exposed unresolved behavior

Phase405 app/library build completed; fresh CLI tests passed 27/27. Strengthened
the native paired-graph fixture by injecting .03 m and alpha*.03 m into its
two synthetic TDCP measurements, requiring a nonzero optimized correction.
Freshly compiled/linked native focused suite: 1/2 passed, 1 failed at that
assertion. Exported L1 correction was exactly zero despite reported convergence.
Existing pair/state count, identity and RMS equality assertions did not fail.

Therefore do not count zero-state diagnostics equality as validation of an
active correction. It may reflect solver/fixture sensitivity, premature
termination, insertion or export behavior; cause is not established. Scalar
factor and small paired graph tests from Phase400 are narrower and cannot
overrule this full backend failure.

Added iteration/initial/final-cost context to the failing assertion after the
run; this last test-only edit is not recompiled yet. Next compile the current
test source, link rebuilt libraries, and rerun the exact Phase171 main test to
inspect aggregate solver diagnostics. The last failing binary is
`/tmp/gnss_phase406_backend_tests`; no build sessions remain live.

No raw solve, truth, MAT or persisted trajectory input. CLI remains unexposed.
Do not weaken the nonzero assertion or claim precision improvement. Goal
remains active; this is a diagnosable test failure, not an external blocker.
