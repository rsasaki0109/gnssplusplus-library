# Phase358 — raw initial-window angular-motion witness

Primary agent only. No solver changes, truth reads, saved positioning inputs,
MAT payloads, accuracy evaluation or submission. The native raw IMU loader
and raw GNSS UTC/GPS mapping were used with the H fallback offset of -20 ms.
Inputs are the H GNSS and IMU files pinned by the Phase357 manifest.

Diagnostic source: `scripts/native_initial_imu_motion_audit.cpp`, SHA256
`3d4b1ff80a1a4c1f56df5b9e46b33a5b4a4f01f9095145035ca63622907be2bf`.
Compiled against the existing native libraries; compilation and execution
returned zero. The diagnostic was run twice, first without and then with
the threshold count; common aggregates were identical. No accuracy score
was computed on either run.

For the first 250 time-sorted synchronized samples, matching the CLI's
initial alignment window (166502 total loaded samples):

- Gyro vector-mean norm: 0.00092480843298136869 rad/s.
- Gyro RMS norm: 0.013695098019710323 rad/s.
- Maximum gyro norm: 0.083657505991593606 rad/s.
- 7 / 250 samples reached or exceeded 0.05 rad/s, the source
  `parameters.m` stop_gyro_max threshold.

The CLI gates this window on acceleration magnitude/variation but does not
apply the gyro stop criterion before alignStatic. The diagnostic computes
rotation-invariant norms, so the fixed native mounting rotation is not
needed. This is not the complete source stop detector: moving standard
deviations and their thresholds were not evaluated here.

The window is not uniformly below the source angular-rate stop limit.
However, its vector mean is small compared with its RMS. These measurements
do not identify true sensor bias, establish sustained turning, or prove
that initialization causes the positioning error. In particular, replacing
the bias with zero or selecting a window from H truth is not justified.

Next discriminating check: compare the same raw initial-window mean with
means over intervals admitted by the full existing native source-stop
detector, requiring sufficient support and reporting scatter, before
proposing a changed initialization rule. Keep the operational recipe and
its finite priors unchanged until that evidence and synthetic controls
justify an isolated experiment. H remains reused development data; the
0.782 / leaderboard objective is unachieved.
