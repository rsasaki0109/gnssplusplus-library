# Native Android IMU vertical slice

This development-only slice connects raw GSDC 'device_imu.csv' to the
existing no-base native FGO entrypoint. It is deliberately small: it proves
the raw parser, clock preservation, gyro-anchored acceleration pairing,
explicit handset mounting, and the existing GTSAM 'CombinedImuFactor' path.
It does not claim upstream full-route equivalence or a quality improvement.
Production defaults and the retained v5 lane are unchanged.

## Raw contract

The C++ loader is 'libgnss::loadAndroidImuCsv' in
'src/io/imu.cpp', declared in 'include/libgnss++/io/imu.hpp'. It accepts only
the Android CSV columns needed by 'UncalAccel' and 'UncalGyro':
'MessageType', 'utcTimeMillis', 'elapsedRealtimeNanos',
'MeasurementX/Y/Z', and 'BiasX/Y/Z'.

- 'UncalAccel' remains metres/second² and 'UncalGyro' remains radians/second.
- UTC milliseconds are converted to GPST with the explicit GSDC-2023
  18-second offset.
- The first row for each UTC timestamp in each supported stream is retained,
  matching the pinned upstream 'unique' behavior.
- Gyro elapsed time is the anchor. Interior accelerometer samples are linearly
  interpolated on 'elapsedRealtimeNanos'; an endpoint is nearest-held only
  when its offset is at most 25 ms. No extrapolation is performed.
- The original gyro 'elapsedRealtimeNanos' is preserved in each 'ImuSample'.
  Upstream's additional GNSS-observation elapsed-to-UTC interpolation and
  'sync_coefficient=0.5' are not silently approximated; the run summary
  reports 'gnss_elapsed_anchor_applied=false' until that stage is separately
  implemented and frozen.
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

First produce a bounded adapter RINEX file from raw GNSS only:

~~~sh
PYTHONPATH=apps/commands python3 apps/gnss.py smartphone-gnss-adapter \
  --device-gnss <route>/<phone>/device_gnss.csv \
  --output-dir <run>/adapter600 \
  --dataset-id <route>/<phone> --device-model pixel7pro \
  --source-url https://www.kaggle.com/competitions/smartphone-decimeter-2023 \
  --source-terms "GSDC competition terms; local truth-free smoke" \
  --role development --truth-free --skip-epochs 0 --max-epochs 600
~~~

Then invoke the C++ entrypoint with the raw IMU directly:

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

The current raw Pixel smoke is sealed under
'output/smartphone-r5/phase5-android-imu-smoke' (ignored local output):
30 epochs, 29 CombinedImu intervals, 327 graph factors, convergence in 12
iterations, 166,363 aligned gyro rows, median pair offset 4.138795 ms, and
maximum pair offset 4.695516 ms. It has no truth score. The machine-readable
run record is
[smartphone_r5_native_fgo_android_imu_raw_truth_free_run_v1.json](records/smartphone_r5_native_fgo_android_imu_raw_truth_free_run_v1.json).

## Remaining work

The next exact component is GNSS-anchor synchronization: expose the raw
GNSS 'elapsedRealtimeNanos'/UTC mapping to the native IMU builder and apply
the upstream coefficient under a new freeze. Full-route multi-pass
initialization, stop factors, ordinary TDCP, and upstream optimizer settings
remain separate gaps. No holdout or validation result may be inferred from
this short structural smoke.
