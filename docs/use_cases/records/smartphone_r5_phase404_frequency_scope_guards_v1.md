# Phase404 — frequency-state dispatch and staging isolation

Extended `optimizeProblem` entry checks to reject fixed-lag, missing IMU/Pose3,
missing TDCP/C7, affine variants and source/official atmosphere-bypass modes
when the research residual-frequency state is requested. Added explicit
no-GTSAM-build rejection instead of letting another backend ignore the option.
The Android CLI's main-to-GNSS-first configuration copy now clears both the
flag and prior sigma before constructing the staging processor.

Fresh app build passed. Fresh test object compiled and linked against the
rebuilt library; focused native Phase171 main suite passed 2/2, including
explicit paired-state branch and fixed-lag/no-IMU/source-resL/zero-prior rejection.
No-GTSAM compile path was source-inspected only, not built. Staging flag reset
is source-level evidence; no enabled raw CLI run exists yet.

No CLI option exposed, no raw solve, truth read, MAT or saved-position input,
and no new accuracy score. Remaining before raw experiment: in-memory
per-factor residual correction export for app diagnostic consistency, exact
timing provenance checks and frozen physically justified prior scale. Full
CTest, disabled-mode raw replay and performance validation remain unrun.
The native-only performance objective remains active and unmet.
