# Phase575: retain optional raw ADR uncertainty in native observations

Added `source_adr_uncertainty_m` and availability flag to Observation.
Android reader consumes optional AccumulatedDeltaRangeUncertaintyMeters
without unit conversion or ADR sign multiplication. Finite positive values
retained exactly; absent/blank/zero/negative/nonfinite/malformed text becomes
unavailable. Malformed optional text was previously ignored; it must not
change admission now. Missing metadata stores zero with availability false,
not a zero-noise measurement. Carrier quality/slip gates remain separate;
metadata availability alone does not authorize a valid carrier endpoint.

No FGO sigma, weights, factors, admission or covariance selector changed.
New loader fixture covers eight optional-value cases and checks unchanged
row count/carrier metres and uncertainty availability. Test translation unit
compiled separately to `/tmp/phase575_android_tests.o`; runtime tests pending
completion of native library rebuild, not claimed passing yet.

Native rebuild `cmake --build build --target gnss_fgo_imu_no_base -j2` live
session 12146. Observation layout changed, so do not link the new test object
against old pre-rebuild libraries or invoke stale native binaries as current.
Resume this build; don't start duplicates. Disk at start about 1.4 GB free.

After build: link/run focused Android loader tests, add explicit slip/reset
availability coverage if needed, then raw disabled-path identity gate before
any covariance integration. Full CTest/unrelated binary rebuild not done.
No truth, new accuracy evaluation, submission or goal completion this turn.
