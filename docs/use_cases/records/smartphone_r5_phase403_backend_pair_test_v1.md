# Phase403 — backend paired-state test executed

Phase402 library/app build completed successfully. Test object compilation
also completed; linked against the rebuilt libraries with local GTSAM.
Fresh focused `FGOGtsamPhase171NoDopplerImuMainTest.*` passed 2/2 after
explicitly adding a call through the new branch (Phase184 OFF, native ECEF
staging, Phase213/217 and robust main ON, other ablations OFF).

The enabled synthetic branch asserts convergence, one residual state, two
wrapped TDCP factors, two inserted TDCP measurements, finite postfit RMS,
and invalid zero-prior rejection. It uses same-run native staging exports.
No source/truth trajectory or raw data evaluation was performed.

Initial explicit-branch run failed only the invalid-prior test, because the
test called construction rather than `optimizeProblem`, where validation
resides. Corrected the test and recompiled/relinked before passing. Phase402's
wording that constructor validates this selector was inaccurate: validation
is at the optimization entry and backend. Do not count the earlier passing
legacy-only run as proof that the new branch executed.

Remaining before public/raw use: reject fixed-lag/other unsupported dispatch
paths at the optimization entry, prevent main flag copying into GNSS-first,
carry in-memory residual corrections into app diagnostics, establish prior
scale and exact raw timing provenance, and verify disabled raw-output identity.
CLI is still unexposed, defaults OFF. Full CTest and accuracy are unverified.
No .782-class or leaderboard completion claim; goal remains active.
