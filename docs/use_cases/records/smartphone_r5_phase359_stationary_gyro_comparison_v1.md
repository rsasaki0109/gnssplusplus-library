# Phase359 — stationary gyro comparison

Primary-agent raw-only diagnostic; no truth, MAT, saved positioning input,
solver execution, accuracy score or submission. Native libraries and raw H
GNSS/IMU inputs are unchanged from Phase358 / Phase357.

`scripts/native_initial_imu_motion_audit.cpp --stop-comparison` (flag after
the two raw input paths) was compiled and executed successfully. Source
SHA256: `2bb734ad5542a2699c0e1bfef3781f6e06898e074a16d6b64716dbd172ffeb7b`.
The existing upstream_stop::detect defaults were used over the full loaded
IMU stream. Only its IMU mask was consumed; a sample-time query satisfies
the API's epoch argument, and no GNSS epoch-stop mapping is claimed.

Before execution, the comparison was defined as nonoverlapping 250-sample
blocks wholly admitted by the stop mask, resetting on a rejected sample
or a time gap exceeding 0.1 s. Require at least two blocks. Remainders are
discarded. No tuning from truth or positioning errors was performed.

- Total admitted stop samples: 52530 / 166502.
- Admitted samples within initial alignment window: **0 / 250**.
- Complete stationary blocks: 199.
- Norm of mean of stationary-block gyro means: 0.0010385541493464401 rad/s.
- Initial mean versus stationary mean vector distance: 0.0010657961381611441 rad/s.
- RMS block-mean scatter about stationary mean: 0.00045154079270274649 rad/s.
- Maximum block-mean distance from initial mean: 0.0015489783290716817 rad/s.

This strengthens the evidence that the acceleration-only initial gate does
not select a source-classified stationary window on H. It does not prove
the stationary aggregate is true sensor bias: persistent small rotations,
temperature drift and classifier false positives remain possible. The
norm comparison is invariant to the fixed orthonormal mounting rotation.

Next candidate is a raw-stop-supported gyro initializer, not a zero-bias
replacement or an accuracy-tuned offset. Before any H run, test synthetic
known bias with an initially turning interval, later stationary support,
insufficient support and gaps. Change gyro initialization only, retaining
accelerometer initialization and all covariance settings. Explicitly record
that changing init_gyro_bias affects initial Values, preintegration and the
first gyro-bias prior mean together; it is not merely an initial-Values
change. Keep it default-off, reject inadequate support without fallback,
and do not claim improvement before a frozen evaluation. Operational
baseline and the unachieved full objective remain unchanged.
