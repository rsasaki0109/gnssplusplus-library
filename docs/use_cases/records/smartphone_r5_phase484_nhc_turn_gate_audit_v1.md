# Phase484 — NHC turn-gate counterexamples

Re-read current application mounting, fixed-lag admission, and internal
imuWindowStats implementation. Extended native_nhc_frame_audit.cpp and
compiled against the installed GTSAM and current internal header. Exit 0.
No route, truth, MAT, or saved position input was used; production behavior
and baseline settings are unchanged.

Measured synthetic cases:

- Jointly rotating pose and forward velocity by 0.4 rad yields exactly zero
  lateral/vertical residual. Turning is not itself lateral slip in body axes.
- Empty IMU window returns n=0 and yaw_rate_abs=0.
- Two body-z gyro samples +0.4 and -0.4 rad/s return yaw_rate_abs=0,
  despite median gyro norm 0.4 rad/s. Signed averaging hides reversal.

Current fixed-lag nhc_candidate checks speed and mean yaw but does not require
a nonempty IMU window. With use_zupt=false and adequate seed speed, both
counterexamples pass the yaw gate at its default 0.2 rad/s threshold. This
is an admission weakness, not evidence that these cases occur on a particular
route or that the existing batch baseline is affected: batch does not wire NHC.

For the batch candidate, do not copy this admission gate unchanged. Require
valid finite, time-covered IMU support and use a non-cancelling angular-motion
statistic. Specify whether body-z angular rate or full angular speed is intended;
the latter conservatively excludes roll/pitch as well as yaw. Keep the mounting
assumption explicit. Synthetic covariance confirms factor frame mathematics,
not the physical calibration of the phone mount. Turn exclusion is a conservative
model-validity choice, not proof that all turning violates NHC. Real-data
coverage diagnostics must precede accuracy claims. Overall goal remains unmet.
