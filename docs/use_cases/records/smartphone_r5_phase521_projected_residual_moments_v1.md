# Phase521 — projected residual moments without unstable scalar fitting

Added scalarResidualProjection to pseudorange_position_information.hpp. It
jointly whitens coefficient and residual, projects both against the same
nuisance SVD space, and returns information a'a, cross moment a'r and residual
energy r'r. Existing information routines are unchanged. No fitted correction
is returned or applied. Local model convention r + a*delta + B*n implies
delta = -a'r/a'a only when identifiable; downstream diagnostics must not divide
by numerically null information or interpret a large noisy fit as ionosphere.

Two tests added to the existing registered position-information test file:
weighted synthetic signed correction recovery despite nuisance contamination;
pure nuisance residual elimination, fully unobservable case and bad dimension
rejection. Fresh standalone compile/run exited 0, all 11 tests passed (2 new,
9 existing). git diff --check on both edited files passed. Full CTest not run.

```sh
c++ -std=c++17 -O0 -Iinclude -I/usr/include/eigen3 tests/test_pseudorange_position_information.cpp -lgtest -lgtest_main -lpthread -o /tmp/phase521_residual_projection
/tmp/phase521_residual_projection
```

Integration still outstanding: Phase519 actual-factor diagnostic currently
discards residual after validating it. Next retain that same-run scalar only
in the diagnostic row, compute IRLS projected residual moments, and report
aggregate residual/coefficient alignment and temporal support without feeding
any result into the graph. A scalar correlation or temporal persistence alone
cannot distinguish ionosphere from multipath or model mismatch. No raw replay,
truth, MAT, saved positioning input, production model change or score claim
in this phase. No active native/build job remains. Overall goal still unmet.
