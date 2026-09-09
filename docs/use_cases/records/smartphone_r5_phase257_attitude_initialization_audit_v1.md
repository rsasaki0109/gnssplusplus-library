# Phase257 native/source attitude initialization audit

Read-only source audit; no MAT payload, saved trajectory, or truth read.

Three separable differences were located:

1. Cached `gsdc2023/functions/vel2rpy.m` smooths velocity, rejects low-speed
   headings, and fills **all** missing headings with nearest neighbors.
   Native `src/fusion/fusion_initialization.cpp` lines 136–145 instead
   linearly interpolates interior gaps. Existing tests explicitly preserve
   that historical behavior. Do not rewrite them to claim source parity.
2. Cached `fgo_gnss_imu.m` constructs a Pose3 for each velocity-derived
   attitude (lines 145–149). Native CLI computes the complete sequence but
   uses only `rpy_rad.front()` at line 5609. The backend lines 1382–1388
   initializes subsequent attitudes by integrated gyro rotations. Thus an
   isolated interior-fill change cannot alter the currently consumed first
   attitude; it is not a justified standalone H accuracy experiment.
3. The source initializes bias to zero, while native uses static-alignment
   bias estimates. Keep this separate from an attitude-only experiment.

The source's local eul2rotm implementation uses Rx*Ry*Rz, while native
initialization uses Rz*Ry*Rx. For the velocity-derived sequence, roll and
pitch are both zero, so this is not a difference in the proposed yaw-only
initialization. Do not generalize that equivalence to arbitrary attitudes.

Next implementation: default-off, native same-run per-epoch velocity-heading
attitude seeds. Add an explicit nearest-fill option without changing the
legacy helper behavior. Carry a validated, exact epoch-sized sequence in
the in-memory IMU input and select it only for Phase171. Validate finite
proper rotations and reject missing/misaligned sequences; no saved seed
input and no fallback to gyro seeds when explicitly requested. Preserve
translation, velocity, clocks, bias initialization, factors, and solver
settings. Test turning motion and low-speed gaps before a single raw H run.

This is a candidate initialization hypothesis, not an established accuracy
improvement. H reference remains 1.0769392017393964 m; the target is unmet.
