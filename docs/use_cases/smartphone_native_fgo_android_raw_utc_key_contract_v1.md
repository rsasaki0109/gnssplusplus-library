# Raw Android UTC-key contract (development candidate)

This opt-in candidate repairs the output-key mismatch observed in the Phase 7
raw-only run. `AndroidRawGnssResult` retains the integer `utcTimeMillis` for
each selected observation epoch, without reconstructing keys through GPST
floating-point arithmetic. With `--all-epochs --android-raw-utc-keys`, the
first raw epoch is treated as a declared GNSS/IMU warm-up and omitted; the
remaining raw keys are the output target. Native solution states are matched
to those keys within 2 ms. Missing target states use same-trip ECEF linear
interpolation, and finite nearest-edge hold is the only edge fallback. Device
WLS, truth, sample coordinates, imported result files, and all `.mat` files
are forbidden.

The warm-up choice is deliberately marked weak: the corrected Doppler state
needs a preceding epoch and the IMU graph begins with an interval, but the
pinned upstream `fgo_gnss.m` exposes configurable `IdxStart`/`IdxEnd` rather
than proving a universal one-row exclusion. Therefore this contract remains
opt-in and is not a production/default change.

## Reproduction

```sh
cmake --build build --config Release --target gnss_fgo_imu_no_base gnss_run_tests -j2
LD_LIBRARY_PATH=/home/sasaki/.local/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH} \
  build/apps/gnss_fgo_imu_no_base \
  --android-gnss <route>/pixel7pro/device_gnss.csv \
  --android-imu <route>/pixel7pro/device_imu.csv \
  --nav <route>/brdc.nav --out <output>/submission.csv \
  --summary-json <output>/summary.json \
  --dataset-id <route>/pixel7pro --all-epochs --android-raw-utc-keys
```

The frozen development route produced 1,383/1,383 finite ordered keys:
1,096 exact native states and 287 ECEF interpolations, with a maximum
interpolation bracket of 49 s. The single permitted development-train score
was 248.986414294 m (haversine/linear variant), so the candidate is No-Go and
must not proceed to validation or holdout. The sealed details are in
`docs/use_cases/records/smartphone_r5_native_fgo_android_raw_utc_key_contract_freeze_v1.json`,
`docs/use_cases/records/smartphone_r5_native_fgo_android_raw_utc_key_artifact_seal_v1.json`,
and
`docs/use_cases/records/smartphone_r5_native_fgo_android_raw_utc_key_contract_result_v1.json`.
The focused Android raw-key tests passed 9/9; the Release build and serial
CTest suite passed 131/131. The complete candidate/output/evaluation hashes
and the final No-Go decision are sealed in
`docs/use_cases/records/smartphone_r5_native_fgo_android_raw_utc_key_contract_result_v1.json`.
