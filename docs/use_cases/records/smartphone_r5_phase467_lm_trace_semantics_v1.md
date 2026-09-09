# Phase467 — linear-solve counter semantics

Inspected cached GTSAM 4.3a LevenbergMarquardtOptimizer.cpp: tryLambda sets
systemSolvedSuccessfully=false only when solve(dampedSystem,params) throws
IndeterminantLinearSystemException. Its six-column summary prints iteration,
newError, costChange, lambda, that boolean, elapsed time. Native parseAttempts
reads the same order. Thus the fifth column is not nonlinear step acceptance.

Added a test-only failure-count accessor and extended the existing nonfinite
trace regression test. Standalone native_lm_trace_audit.cpp compiled against
the actual internal parser and linked GTSAM, returning attempts=2 and
linear_failures=1 for one failed linear solve plus one solved row with NaN
nonlinear cost. Exit 0; git diff --check passed. The expanded backend gtest
itself has not been rebuilt/run; do not claim a full backend-suite pass.

No column-order bug was found. Existing H/U/LAX failure counts are consistent
with actual caught linear-system exceptions under this output format, not
just nonfinite nonlinear costs. Historical full captured traces were not
recovered in this phase, so this is format validation, not independent
recounting of those runs. All recorded baseline main solves converged.

The LAX termination metadata uses MULTIFRONTAL_QR/COLAMD, isotropic damping
(diagonal_damping=false), initial lambda 1e-5, final approximately 1e-9,
lower bound zero. Do not change damping from counters alone or mistake fewer
retries for better positioning. A useful next diagnostic would capture the
actual failing key/lambda and whether retries change the accepted solution;
only then decide if scaling, gauge handling or solver configuration needs
an implementation change. Preserve the constrained C7 semantics established
in Phase465, and do not relax sensor quality masks to suppress exceptions.

No inference rerun, truth access, MAT, scoring or submission. Production
optimizer behavior unchanged; overall accuracy/leaderboard goal unmet.
