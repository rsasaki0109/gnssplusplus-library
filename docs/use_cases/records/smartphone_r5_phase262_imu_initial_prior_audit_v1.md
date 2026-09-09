# Phase262 IMU initial Values versus priors

Source inspection only; no raw, truth, MAT or saved positioning payloads.

Native `fgo_gtsam_backend.cpp` IMU branch initializes every bias state from
`problem.imu.init_accel_bias/init_gyro_bias`, uses the same bias to construct
preintegrated measurements, and adds a finite first-bias prior at that
value (lines 1511–1516 at inspection). This block is active for Phase171;
there is no Phase171 exemption. The CLI supplies static-alignment estimates
and sigmas 0.1 m/s² and 0.01 rad/s respectively.

The same block also inserts a first-pose prior (roll/pitch 0.05 rad, yaw
5 degrees, translation 1e6 m) and first-velocity prior (0.5 m/s). These
are distinct from the bias prior and must not be bundled into one change.

Cached source `fgo_gnss_imu.m` initializes all bias states to zero and adds
per-epoch priors with infinite sigmas for bias, pose, position, velocity,
clock and drift. Infinite sigma is not a finite pull toward the initial
value. Initial Values, preintegration linearization bias, and finite
prior means are different roles in the native implementation.

`alignStatic` sets gyro bias to the initial-window gyro mean and estimates
accel bias from mean acceleration and gravity. The CLI leveling gate checks
acceleration magnitude and variation; it does not independently establish
zero angular motion. This is a potential source of a biased initial gyro
estimate, not evidence that the H window actually turns.

Decision: do not call a zero-bias replacement an initialization-only
ablation: it changes both the preintegration linearization point and the
finite prior's target. First add a default-off, Phase171-only option that
omits the first-bias prior while preserving initial Values, preintegration,
pose/velocity priors, and the CombinedImuFactor noise model. Test graph
count decrement exactly one, unchanged non-bias priors, convergence and
failure behavior on synthetic motion. Removing the bias prior might reduce
observability; do not add hidden stabilization or fallback if it fails.
Only after these checks should a single frozen raw comparison be considered.

This would test one graph constraint difference, not reproduce the entire
source graph or prove an accuracy improvement. Keep reference defaults.
