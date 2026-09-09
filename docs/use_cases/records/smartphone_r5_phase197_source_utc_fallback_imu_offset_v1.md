# Smartphone R5 Phase197: opt-in UTC-fallback IMU time offset

Status: implementation and synthetic verification complete.  No native H run,
truth read, accuracy score, MAT input, or persisted trajectory input was used
in this phase.

## Bounded change

The default-off selector
`--native-phase197-source-utc-fallback-imu-offset` is restricted to the
Pixel5 Phase171 raw-P/ECEF-Doppler GNSS-first plus Pose3/IMU lane with explicit
Android UTC wall-clock fallback.  The application asks the loader to apply the
selector only when that fallback branch is actually selected; anchored
GNSS-elapsed-time input is never shifted.  The loader rejects a nonzero offset
without the explicit selector and fails closed on signed-timestamp overflow or
underflow.

The source-compatible fallback value is `-20 ms` (from the cached
`parameters.m`/`imuprocessing.m` source contract).  It is applied to the
validated affine UTC-to-GPS mapping input, not by subtracting a fixed value
from GPS time.  Raw UTC keys, elapsed-clock values, pairing bounds, and
interpolation remain unchanged.  The summary reports requested, fallback
actual, configured, and effective offset values separately, including an
explicit source branch and unchanged-clock markers.

This is deliberately an offset-only ablation.  It does not change the loader's
existing alignment coefficient, pairing, interpolation, or endpoint policy,
and therefore does not claim complete cached-source timing parity.

## Verification

- `cmake --build build --target gnss_run_tests gnss_fgo_imu_no_base -j8`
  passed (exit code 0).
- Fresh focused native regression filter: 85/85 tests passed, including the
  real Android CSV/mapping fallback loader test, anchor-precedence and invalid
  mapping/underflow cases, Phase194 noise isolation, and Phase164/165/167/171
  graph regressions.
- Python source/guard tests passed: Phase17 family 8/8, Phase18 family 35/35,
  Phase194 4/4, and Phase197 4/4.
- `git diff --check` passed.

No raw execution is authorized or pinned by this record; a future run must
freeze a separate manifest and keep Phase194 noise selection off when testing
this offset-only selector.
