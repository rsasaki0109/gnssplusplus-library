# Smartphone native FGO v2: MATLAB-free no-base entry point

This is an opt-in research path. It does not change the production `gnss_fgo`
defaults and it does not use a base RINEX, validation/holdout data, or Kaggle
scores.

The frozen contract is recorded in
`docs/use_cases/records/smartphone_r5_gsdc2023_native_fgo_v2_mat_no_base_freeze.json`.
The entry point reads a RINEX observation file, broadcast navigation, and the
explicit `loadImuCsv()` CSV schema:

```sh
LD_LIBRARY_PATH=/home/sasaki/.local/lib:$LD_LIBRARY_PATH \
  build/apps/gnss_fgo_imu_no_base \
  --obs <adapter/rover.obs> --nav <brdc.nav> --imu <imu_processed.csv> \
  --out <submission.csv> --summary-json <run_summary.json> \
  --dataset-id <route/phone> --skip-epochs 200 --max-epochs 30
```

The short-window limit is deliberately 10--30 epochs. `--skip-epochs` only
selects a truth-free diagnostic window; the frozen examples use 200 so the
initial 250-sample stationary window is retained while GNSS course becomes
observable. The solver builds no-base undifferenced pseudorange factors and
the existing GTSAM backend adds `Pose3`, velocity, bias, and real
`CombinedImuFactor` states. A failed gravity/course/finite/solver gate falls
back to the same no-base problem without IMU and marks that decision in the
atomic JSON summary.

## Public MAT conversion

`apps/commands/benchmarks/gnss_smartphone_phone_data_mat.py` is a deterministic
schema adapter for public MATLAB v5 `phone_data.mat` files. It extracts only
numeric `acc.utcms/xyz` and `gyro.utcms/xyz`; MATLAB `gt.Gtime` MCOS opaque
objects are not decoded. Gyro timestamps are the anchors, acceleration is
nearest-neighbour paired within 25 ms, and UTC milliseconds are converted to
GPST with the declared 18-second offset. Android gyro input is rad/s and the
CSV loader contract is deg/s, so the conversion is performed exactly once.
The converter never reads `truth`, `nav`, or position labels. Its output still
has explicit raw axes; the native entry point applies the frozen upstream
`RzRyRx([-85,178,-94] deg)` mounting rotation before gravity/course
initialization.

The upstream MATLAB graph additionally contains custom Doppler/TDCP factors
from `gtsam_gnss`. Those factors were audited (MIT, hash-pinned) but are not
silently inserted here: the current native no-base GTSAM backend does not expose
their graph insertion contract. The v2 path therefore claims only the
CombinedImuFactor integration, not parity with the complete MATLAB graph.

The sealed run record is
`docs/use_cases/records/smartphone_r5_gsdc2023_native_fgo_v2_mat_no_base_smoke.json`.
It records the truth-free structural smoke, the bounded train comparison, all
source/upstream hashes, and the performance No-Go decision for this short
window. No validation or holdout was materialized or opened.
