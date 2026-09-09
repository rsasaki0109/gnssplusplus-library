# Phase429 — optimized IMU bias aggregate diagnostic, pending build

Rechecked Phase204/207/265 against current native code before proposing bias
changes. Prior removal already gave no H score improvement; source-count
random-walk scaling also did not improve its historical recipe. Do not repeat
either as a newly supported candidate or infer equivalence across recipes.

Added post-solve-only diagnostics to the native GTSAM backend: number of
optimized IMU bias states, maximum accelerometer bias norm (m/s²), maximum
gyro bias norm (rad/s). Read actual optimized ConstantBias values after solve;
reject nonfinite norms. No measurement, graph factor, prior, initialization or
optimizer setting changes. No bias series is serialized or reused by inference.
The app currently places these aggregates in its tdcp_contract diagnostic
object. Counts and finite norms are asserted in the existing paired synthetic
backend test. Maximum norms alone cannot establish drift, observability or
the correct bias covariance; no tuning decision follows from them alone.

Build `cmake --build build --target gnss_fgo_imu_no_base -j1` is live in session
52357, and fresh backend test compilation is live in session 40745 targeting
`/tmp/gnss_phase429_backend_tests.o`. Both were verified live, not restarted.
`git diff --check` passed. Compilation/test execution success is not yet proven.
Do not link older objects compiled against the preceding FGOResult layout.

After verification, raw diagnostic runs must use the retained baseline recipe
(frequency states OFF), not promote the rejected candidate. Pin new sources
and binary in fresh manifests and require baseline candidate SHA identity.
No additional truth read or accuracy score is required for this diagnosis.
