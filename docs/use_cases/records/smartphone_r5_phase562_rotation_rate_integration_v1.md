# Phase562: opt-in Doppler rotation-rate graph integration

Added `--native-doppler-rotation-rate`, default off. CLI scope requires raw
Phase171 IMU main, ECEF Doppler GNSS-first, Phase213 main, joint ionosphere
off. Backend also rejects use outside dedicated ECEF-D or Phase213 graphs.
The setting is copied to GNSS-first and retained in main. Enabled-only
summary metadata identifies both stages.

Unlike the standalone Phase561 ECEF research factor constructor, integration
uses retained row LOS rather than recomputing LOS from an updated seed.
`fromLos` computes the same model directly from unit receiver-to-satellite
LOS and already rotated satellite state. Existing row transfer/unit-LOS
validation remains unchanged. Only at insertion, the velocity coefficient
is replaced by -u/(1-k), and the receiver-only measurement by
observed - u.dot(rotated_velocity)/(1-k) + satellite_clock_drift_mps.
The existing linear source-clock factor classes can represent these
coefficients without modification; they do not normalize their input.
Thus the separate Phase561 class is a research test oracle, not the class
inserted by the backend. Main rotates the coefficient ECEF to ENU without
normalization; GNSS-first retains ECEF. Noise and clock coefficient stay
unchanged. No changes to P/TDCP admission or saved-position inference.

Standalone tests: `/tmp/phase562_rotation_factor`, 5/5 pass, including new
retained-LOS model parity and zero-LOS rejection. This does not yet test
the complete integrated graph or establish accuracy improvement.

Native build started with `cmake --build build --target gnss_fgo_imu_no_base -j2`.
At this record's creation session 52901 was live, compiling GTSAM backend
and fixed-lag sources; completion has not been verified. Resume that handle
or check actual process state, do not start a duplicate build.

Next gate: finish native build; negative CLI smoke; freeze current source,
binary, raw inputs and exact baseline/candidate commands before any new
accuracy read. First disabled raw replay must match the historical baseline.
Then one fixed enabled candidate, no parameter sweep. H/U/LAX are already
explored development routes, not held-out evaluation or leaderboard proof.
