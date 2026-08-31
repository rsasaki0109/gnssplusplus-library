# Raw Android GNSS-first / IMU multipass (development candidate)

This candidate is opt-in and raw-only. It reads `device_gnss.csv`,
`device_imu.csv`, and `brdc.nav`; it does not read truth while generating an
output, and it does not use a device-WLS coordinate as an initializer.
Production RTK/SPP/FGO defaults are unchanged.

The GNSS-first pass uses the GTSAM batch Levenberg--Marquardt backend with
explicit ENU velocity and receiver clock-range-rate states. Its optimized
velocity states feed the upstream-compatible `vel2rpy` initialization:
centered window 20, 0.5 m/s 3-D speed threshold, raw-course interior linear
fill, endpoint nearest fill, then +180 degrees and wrapping. The second pass
uses the existing Pose3/velocity/IMU-bias graph and CombinedImuFactor chain,
with the frozen Android mounting and elapsed-realtime GNSS-anchor contract.
Invalid alignment, nonfinite states, or solver failure fail closed to the
native v1 route path.

The freeze, truth-free artifact hashes, and the single permitted development
train evaluation attempt are recorded in:

- `docs/use_cases/records/smartphone_r5_native_fgo_android_imu_gnss_first_multipass_freeze_v1.json`
- `docs/use_cases/records/smartphone_r5_native_fgo_android_imu_gnss_first_multipass_freeze_v1_manifest.json`
- `docs/use_cases/records/smartphone_r5_native_fgo_android_imu_gnss_first_multipass_result_v1.json`

## Reproduction

Build and run the truth-free full route with the Release binary:

```sh
cmake --build build --config Release -j2
LD_LIBRARY_PATH=/home/sasaki/.local/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH} \
  build/apps/gnss_fgo_imu_no_base \
  --android-gnss <route>/pixel7pro/device_gnss.csv \
  --android-imu <route>/pixel7pro/device_imu.csv \
  --nav <route>/brdc.nav \
  --out <output>/submission.csv \
  --summary-json <output>/summary.json \
  --dataset-id <route>/pixel7pro --all-epochs
```

The sealed full-route artifact produced 1,097 finite rows with no fallback,
1,096 IMU intervals, 4,695 GNSS-first Doppler factors, and 1,097 exported
GNSS-first velocity states. The strict score attempt was a No-Go before metric
publication because this candidate output did not satisfy the evaluator's
exact key-set contract (1,097 extra keys); no retry, key remap, truth reread,
or post-truth tuning was performed. Validation and holdout remain unopened.
