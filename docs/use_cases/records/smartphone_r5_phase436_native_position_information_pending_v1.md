# Phase436 — native nominal P-information integration, pending build

Connected the Phase435 primitive after optimization in the non-affine C7
Phase171 IMU main branch. Group actual admitted pseudorange factors by epoch;
form ECEF antenna-position range Jacobians from same-run optimized antenna
positions and existing rotated satellite positions. Reuse native C7 component
mapping/selectors and each factor's nominal metre sigma. No saved positions,
truth, priors or diagnostic feedback into inference.

Aggregate epoch count, numerical rank-deficient epoch count and minimum
eigenvalue in m^-2. Rank threshold is max(1e-12, max_eigenvalue*1e-10), an
explicit numerical diagnostic convention, not a physical accuracy threshold.
Empty P epochs yield zero information. Invalid geometry/eigensystems reject.
This measures unconstrained-clock, nominal single-epoch P support, not robust
weighted information or full FGO observability. Minimum over a route cannot
describe typical geometry; further distributions may be needed for diagnosis.

Added repeated-LOS and planar-geometry controls: primitive suite now passes
7/7 tests. Fresh backend test object `/tmp/gnss_phase436_backend_tests.o`
compiled successfully with epoch-count/range assertions; it has NOT been
linked/run against the new library yet. Native build remains live in session
29563. Do not restart on a polling timeout or use an old binary as validation.
No new raw run or accuracy evaluation. Full CTest not run.
