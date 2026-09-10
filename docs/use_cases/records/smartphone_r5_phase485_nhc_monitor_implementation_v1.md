# Phase485 — native batch NHC coverage monitor (not a factor experiment)

Added native_nhc_gate.hpp with finite, ordered, at-least-two-sample support,
bounded endpoint/interior gaps, explicit speed threshold and peak bias-corrected
full angular-speed threshold. Peak norm cannot cancel a reversal and includes
roll/pitch; this is not a yaw-only measurement. Added four gtests to the canonical
test target; direct current-source compilation with system gtest passed all four.
Coverage includes missing windows, missing endpoints/interior samples, duplicate
times, nonfinite input, speed boundary, reversal and roll. Not full-suite evidence.

Added default-off --native-nhc-monitor to the Android raw Phase171 ECEF-D CLI.
Batch monitor executes only in the main stage, using transformed/synchronized
problem.imu.samples_body_flu and same-invocation GNSS-first velocity seeds. It
does not read saved coordinates or truth and adds no factors or seed changes.
Thresholds frozen before route inspection: min horizontal speed 2 m/s, peak
angular speed <=0.2 rad/s, every IMU gap <=0.05 s. First epoch excluded (no
preceding interval). Aggregate stderr reports intervals, supported, admitted
and factors_added=0. These thresholds are a conservative candidate policy,
not calibrated vehicle-mount evidence or an accuracy result.

Main executable build started after edits. Real-route coverage and baseline
output invariance remain to be verified after successful build. No production
NHC constraint is enabled by this change. No scoring or submission occurred.
