# Native Android IMU vertical slice

This development-only slice connects raw GSDC 'device_imu.csv' to the
existing no-base native FGO entrypoint. It is deliberately small: it proves
the raw parser, clock preservation, gyro-anchored acceleration pairing,
explicit handset mounting, and the existing GTSAM 'CombinedImuFactor' path.
It does not claim upstream full-route equivalence or a quality improvement.
Production defaults and the retained v5 lane are unchanged.

## Phase 6 anchored raw-only contract (development opt-in)

The current native raw invocation accepts `device_gnss.csv` directly; it does
not require or generate an adapter RINEX file:

~~~sh
LD_LIBRARY_PATH=/home/sasaki/.local/lib:$LD_LIBRARY_PATH \
  build/apps/gnss_fgo_imu_no_base \
  --android-gnss <route>/<phone>/device_gnss.csv \
  --android-imu <route>/<phone>/device_imu.csv \
  --nav <route>/brdc.nav --out <run>/native/submission.csv \
  --summary-json <run>/native/summary.json --dataset-id <route>/<phone> \
  --max-epochs 30
~~~

`--android-gnss` supplies both the raw GNSS observation epochs and the clock
anchors. In this mode `--obs` is rejected, so no Python/RINEX or device-WLS
coordinate is part of inference. `--all-epochs` is available for a truth-free
full-route structural run; it does not turn the existing short-window IMU
initializer into a quality-promotion claim.

The Phase 6 synchronizer ports the pinned `imuprocessing.m` clock contract:
`ChipsetElapsedRealtimeNanos -> utcTimeMillis` is piecewise-linear with linear
extrapolation at the two ends, `UncalGyro` is the common time grid, and
accelerometer values are linearly interpolated on the elapsed clock. The
fixed upstream `sync_coefficient=0.5` is recorded and applied to the IMU
covariance in the existing GTSAM backend. Fewer than two valid GNSS anchors,
non-monotonic anchors, or any non-finite mapped time fail closed before the
IMU file is accepted. The terminal positive `dt` is duplicated for the
preintegration input, matching `imuprocessing.m`. The low-level parser's legacy direct-UTC mode remains
available only for isolated compatibility tests; the native raw entrypoint
always requires the GNSS anchor file.

## Raw contract

The C++ loader is 'libgnss::loadAndroidImuCsv' in
'src/io/imu.cpp', declared in 'include/libgnss++/io/imu.hpp'. It accepts only
the Android CSV columns needed by 'UncalAccel' and 'UncalGyro':
'MessageType', 'utcTimeMillis', 'elapsedRealtimeNanos',
'MeasurementX/Y/Z', and 'BiasX/Y/Z'.

- 'UncalAccel' remains metres/second² and 'UncalGyro' remains radians/second.
- UTC milliseconds are converted to GPST with the explicit GSDC-2023
  18-second offset after the Phase 6 elapsed-clock mapping. The direct UTC
  mode remains only as a low-level compatibility path and is not accepted by
  the native raw entrypoint.
- The first row for each UTC timestamp in each supported stream is retained,
  matching the pinned upstream 'unique' behavior.
- Gyro elapsed time is the anchor. Interior accelerometer samples are linearly
  interpolated on 'elapsedRealtimeNanos'; an endpoint is nearest-held only
  when its offset is at most 25 ms. No extrapolation is performed.
- The original gyro 'elapsedRealtimeNanos' is preserved in each 'ImuSample'.
  The native Phase 6 raw entrypoint reads the matching GNSS
  'ChipsetElapsedRealtimeNanos'/'utcTimeMillis' anchors and reports exact,
  interpolated, and extrapolated anchor counts plus the mapped time range.
- Raw axes are not relabelled by the loader. The no-base entrypoint applies the
  pinned taroz mounting rotation explicitly:
  'Rz(-94°) * Ry(178°) * Rx(-85°)'.
- MATLAB container paths are rejected before opening. The raw lane never
  reads or generates them, and no truth file is needed for a truth-free run.

The upstream specification evidence and exact SHA-256 values are recorded in
[smartphone_r5_native_fgo_android_imu_gap_audit_v1.json](records/smartphone_r5_native_fgo_android_imu_gap_audit_v1.json).
The formal pre-run freeze, fixed parameters, route identity, and commands are
in [smartphone_r5_native_fgo_android_imu_raw_freeze_v1.json](records/smartphone_r5_native_fgo_android_imu_raw_freeze_v1.json).

## Truth-free smoke

The current Phase 6 path is the direct raw command shown above. Its summary
is authoritative structural evidence: it must show `inputs.observation=null`,
an `android_gnss` input, `gnss_elapsed_anchor_applied=true`, finite output,
and no base/double-difference factors.

For historical Phase 5 parser compatibility, a bounded adapter RINEX can
still be produced from raw GNSS only:

~~~sh
PYTHONPATH=apps/commands python3 apps/gnss.py smartphone-gnss-adapter \
  --device-gnss <route>/<phone>/device_gnss.csv \
  --output-dir <run>/adapter600 \
  --dataset-id <route>/<phone> --device-model pixel7pro \
  --source-url https://www.kaggle.com/competitions/smartphone-decimeter-2023 \
  --source-terms "GSDC competition terms; local truth-free smoke" \
  --role development --truth-free --skip-epochs 0 --max-epochs 600
~~~

The historical Phase 5 C++ invocation using that adapter was:

~~~sh
LD_LIBRARY_PATH=/home/sasaki/.local/lib:$LD_LIBRARY_PATH \
  build/apps/gnss_fgo_imu_no_base \
  --obs <run>/adapter600/rover.obs --nav <route>/brdc.nav \
  --android-imu <route>/<phone>/device_imu.csv \
  --out <run>/native/submission.csv \
  --summary-json <run>/native/summary.json \
  --dataset-id <route>/<phone> --skip-epochs 200 --max-epochs 30
~~~

The summary is the authoritative structural evidence: it must show finite
output, no base/double-difference factors, a finite converged graph, and
nonzero 'imu_intervals'. If leveling or heading is not observable in the
bounded window, the entrypoint falls back to the exact existing native
no-IMU path and labels that fallback. This is fail-closed behavior, not a
promotion.

Focused parser and contract tests are part of 'run_tests':

~~~sh
LD_LIBRARY_PATH=/home/sasaki/.local/lib:$LD_LIBRARY_PATH \
  build/tests/run_tests \
  --gtest_filter='AndroidImuCsvTest.*:ImuAxisConventionTest.*:ImuCsvTest.*'
~~~

The prior raw Pixel smoke is sealed under
'output/smartphone-r5/phase5-android-imu-smoke' (ignored local output):
30 epochs, 29 CombinedImu intervals, 327 graph factors, convergence in 12
iterations, 166,363 aligned gyro rows, median pair offset 4.138795 ms, and
maximum pair offset 4.695516 ms. It has no truth score. The machine-readable
run record is
[smartphone_r5_native_fgo_android_imu_raw_truth_free_run_v1.json](records/smartphone_r5_native_fgo_android_imu_raw_truth_free_run_v1.json).

The Phase 6 anchored smoke, when generated, belongs under
'output/smartphone-r5/phase6-android-anchor-smoke' and must be recorded with
the Phase 6 freeze/run records; it is still a structural run, not a score.
The frozen contract is
[smartphone_r5_native_fgo_android_imu_gnss_anchor_freeze_v1.json](records/smartphone_r5_native_fgo_android_imu_gnss_anchor_freeze_v1.json),
and the full-route truth-free result is
[smartphone_r5_native_fgo_android_imu_gnss_anchor_full_truth_free_run_v1.json](records/smartphone_r5_native_fgo_android_imu_gnss_anchor_full_truth_free_run_v1.json).

## Remaining work

The exact anchor mapping is now implemented, but full-route multi-pass
initialization, stop factors, ordinary TDCP, and upstream optimizer settings
remain separate gaps. No holdout or validation result may be inferred from
this short structural smoke.
