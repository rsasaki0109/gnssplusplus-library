# Phase208 source IMU factor construction test

Primary-agent work; no subagents, raw runs, truth reads, or MAT input.
Production code and the best recipe are unchanged.

Added `FGOGtsamPhase208SourceImuFactorTest` to
`tests/test_fgo_gtsam_backend.cpp`. It constructs an actual GTSAM ImuFactor
keyed to the ending bias and a separate BetweenFactor<ConstantBias> with
`sqrt(N) * sigma` noise. Twenty varying synthetic measurements at 0.05 s
intervals give a one-second interval; these are not H observations or H noise
parameters.

Verified controls:

- A terminal state predicted with the ending bias has near-zero motion
  residual; substituting a distinct starting bias does not.
- All six ending-bias Jacobian columns agree with central finite differences
  (epsilon 1e-6, matrix error below 1e-7).
- The actual two-factor graph error agrees with the independently calculated
  half squared normalized bias difference when the motion residual is zero.
- Motion and evolution factors use the intended distinct bias keys.

Fresh `gnss_run_tests` build succeeded. The new test and Phase204/205 tests
passed **7/7**. This checks construction, residuals, derivative and likelihood
accounting, not solver convergence, graph observability, or raw-data accuracy.

## Implementation boundary for the next experiment

Cached `fgo_gnss_imu.m` lines 294–299 specify the ending-bias motion factor
and sample-count bias evolution. Motion insertion is gated by
`dtgps < prm.time_diff_th`; cached `parameters.m` sets that threshold to 1.5 s.
The source initial pose, position, velocity, clock, drift and bias priors
(lines 181–186) all use infinite sigmas. Native first-state pose/velocity/bias
priors are finite. Therefore adopting just the two-factor construction must
not be described as full source parity, and removing priors must be a separate
observability-audited change, not an incidental part of this experiment.

Next: add an opt-in native two-factor branch with genuine standard IMU
preintegration (not a sliced Combined covariance). Preserve current sample
integration schedule and initial priors, retain -20 ms UTC offset, reject
incompatible selectors and empty/invalid IMU intervals, and expose actual
motion/evolution factor counts. Test the production branch before a single
raw comparison. Do not mix in the rejected Phase201 schedule change.
