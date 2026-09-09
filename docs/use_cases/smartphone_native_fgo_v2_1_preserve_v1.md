# Smartphone native FGO v2.1: preserved v1 graph plus IMU

This is an isolated, development-only candidate. It does not change the
production `gnss_fgo` defaults or submit to Kaggle.

The candidate keeps the frozen native v1 pseudorange, ordinary TDCP, position
motion, receiver-clock motion, and Huber settings, then adds the existing
GTSAM `CombinedImuFactor` Pose3/velocity/bias chain. It has no base, double-
difference, single-difference Doppler/TDCP, ambiguity, or truth factor. The
Android MAT/IMU frame and UTC-to-GPST conversion are inherited from the v2.0
no-base adapter. A fixed truth-free GNSS-speed/IMU-activity offset grid is
used only when its confidence gate passes; both permitted runs rejected the
estimate and used zero offset.

The freeze and sealed run evidence are:

- `docs/use_cases/records/smartphone_r5_gsdc2023_native_fgo_v2_1_preserve_v1_freeze.json`
- `docs/use_cases/records/smartphone_r5_gsdc2023_native_fgo_v2_1_preserve_v1_run.json`
- `docs/use_cases/records/smartphone_r5_gsdc2023_native_fgo_v2_1_preserve_v1_run_manifest.json`

## Reproduction

Build the opt-in target in Release mode:

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --config Release --target gnss_fgo_imu_v21 -j2
```

Run the truth-free structural smoke (the paths below are already sealed
artifacts):

```sh
LD_LIBRARY_PATH=/home/sasaki/.local/lib:$LD_LIBRARY_PATH \
  build/apps/gnss_fgo_imu_v21 \
  --obs output/smartphone-r5/native-fgo-v2-mat-no-base-v1/smoke/route-2021/rover.obs \
  --nav output/smartphone-r5/native-fgo-v2-mat-no-base-v1/smoke/route-2021/brdc.nav \
  --imu output/smartphone-r5/native-fgo-v2-mat-no-base-v1/smoke/route-2021/imu_processed.csv \
  --out output/smartphone-r5/native-fgo-v2-1-preserve-v1/smoke/route-2021/submission.csv \
  --summary-json output/smartphone-r5/native-fgo-v2-1-preserve-v1/smoke/route-2021/run_summary.json \
  --dataset-id 2021-03-16-18-59-us-ca-mtv-a/pixel5 --skip-epochs 200 --max-epochs 30
```

The fixed train short-window command is recorded verbatim in the run record.
It produced 30 finite rows, 444 pseudorange factors, 206 ordinary TDCP
factors inserted, 29 position/clock motion pairs, 29 CombinedImu factors,
zero forbidden factors, and a converged graph. Truth-free output hashes were
sealed before the one permitted read-only train comparison; this v2.1 phase
did not materialize or open validation/holdout members.

## Result

On the fixed 30-epoch Pixel 7 Pro train window, v2.1 reached WGS84 horizontal
P50/P95 of 1.549/1.704 m, versus the already sealed exact-v1 short-window
reference 1.461/2.001 m. Availability and truth coverage were 1.0 and H P95
improved, but H P50 regressed. The frozen all-metrics non-regression gate
therefore yields **No-Go**. The candidate is retained for auditability only;
it is not expanded to other routes and no validation/holdout evaluation is
authorized by this record. Vertical error could not be compared because this
research output intentionally has the four-column position schema.

Run the full regression suite after rebuilding:

```sh
LD_LIBRARY_PATH=/home/sasaki/.local/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH} \
  ctest --test-dir build -j1 --output-on-failure
git diff --check
```
