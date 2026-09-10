# Native raw PDC+IMU ordinary TDCP candidate (Phase 10)

This is a development-only opt-in lane. It consumes only raw Android
`device_gnss.csv`, `device_imu.csv`, and broadcast `brdc.nav`; it does not use
MAT files, ground truth during inference, device-WLS coordinates, sample
coordinates, or a precomputed trajectory. The production FGO defaults are
unchanged.

The candidate is frozen and recorded in
[`smartphone_r5_phase10_native_pdc_imu_tdcp_freeze_v1.json`](records/smartphone_r5_phase10_native_pdc_imu_tdcp_freeze_v1.json)
with manifest hash `7db7faed45ea196821413c7cad4977955f1ab1a600451a8938b45a94b39347d9`.
It enables the existing ordinary time-differenced carrier factor only with
`--native-pdc-imu-tdcp`. Each factor pairs the same satellite and signal in
adjacent epochs after the Android ADR-to-metres conversion. The fixed gates
are finite `0 < dt <= 2 s`, no loss-of-lock/slip, finite differences, and
`abs(delta_carrier - delta_code) <= 10 m`. No standalone ambiguity, base, or
double-difference factor is enabled.

On the declared development route, the truth-free run built and inserted
15,485 TDCP factors from 16,527 candidate pairs (2,360 missing-previous and
1,042 code-phase-jump rejections), with 2,065 arcs (median length 4 epochs,
maximum 170). Final TDCP residual RMS was 0.0251430123 m (0.838100409 sigma),
the IMU graph converged in 12 iterations without fallback, and output had
1,383/1,383 raw UTC target keys with zero interpolation. Wall time was 14.76 s
and peak RSS 157,316 kB.

After the freeze and artifact hash seal, the declared development truth was
read once. The four local metric variants were all below the fixed Phase9
reference: WGS84/Vincenty linear 3.913768484 m, WGS84 nearest-rank 3.914159025
m, Haversine linear 3.913807300 m, and Haversine nearest-rank 3.914202589 m
(mean 3.913984349 m; coverage 1.0). The score record is
[`smartphone_r5_phase10_native_pdc_imu_tdcp_score_result_v1.json`](records/smartphone_r5_phase10_native_pdc_imu_tdcp_score_result_v1.json).
This is one route and does not authorize validation, holdout, or Kaggle use;
the attribution remains conditional on the inherited Phase9 PDC/IMU bridge
and is not a standalone ambiguity-solution claim.

Reproduce the truth-free artifact after building:

```sh
cmake --build build --parallel 4 --target gnss_fgo_imu_no_base run_tests
LD_LIBRARY_PATH=/home/sasaki/.local/lib:$LD_LIBRARY_PATH \
  build/apps/gnss_fgo_imu_no_base \
  --android-gnss <device_gnss.csv> --android-imu <device_imu.csv> \
  --nav <brdc.nav> --out <submission.csv> --summary-json <summary.json> \
  --dataset-id <route>/<phone> --all-epochs --android-raw-utc-keys \
  --native-pdc-imu-tdcp
```

The candidate remains opt-in and development-only; it does not claim the
0.782-class target or alter the native production/default lane.

After the once-scored artifact was sealed, a separate correctness audit added
clock-discontinuity rejection to the pair helper. The route has zero such
discontinuities, and a fresh raw-only run produced the byte-identical
submission (`5a367a306123a4557f42e2b59a4760bc4eb2edfd1d36492bcab6ab2c88e240bb`),
so the original truth score was reused without reopening truth. The scope and
hashes are recorded in
[`smartphone_r5_phase10_native_pdc_imu_tdcp_clock_gate_recovery_v1.json`](records/smartphone_r5_phase10_native_pdc_imu_tdcp_clock_gate_recovery_v1.json).
