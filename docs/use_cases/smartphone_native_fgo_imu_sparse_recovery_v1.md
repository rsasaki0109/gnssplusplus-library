# Native FGO sparse-epoch recovery candidate (Phase 9)

This is a separately frozen, development-only coverage candidate. It is not
the dedicated `gnss_pos_vel_pdc` state bridge: it keeps the existing raw
Android `FGOProcessor` problem builder and GTSAM Pose3/velocity/bias IMU graph,
retains epochs below the normal four-satellite factor floor, and enables the
existing secondary-frequency eligibility switch for raw P/D factors. The
production defaults and the retained native PDC/v5 lanes are unchanged.

The candidate was frozen before truth access in
`docs/use_cases/records/smartphone_r5_gsdc2023_native_fgo_imu_sparse_recovery_freeze_v1.json`
(record SHA256 `b852c28124e397c1a9a5f303e2d5655ba85e463a1fceb35528004c7b6ff80879`).
Only `device_gnss.csv`, `device_imu.csv`, and `brdc.nav` were allowed as
inference inputs. It produced 1,384 graph/output epochs, 1,383 exact raw UTC
target keys, 1,383 IMU intervals, and no fallback or interpolation. The raw
artifact is sealed in
`docs/use_cases/records/smartphone_r5_gsdc2023_native_fgo_imu_sparse_recovery_artifact_v1.json`.

The one permitted development-truth evaluation was run only after that seal.
It scored 1,383/1,383 keys, but the fixed WGS84/Vincenty linear diagnostic was
926.257651641 m (Haversine linear 924.182497976 m), versus the fixed gate
68.047583894 m and the earlier raw UTC FGO lane's 248.986414294 m. The
candidate is therefore No-Go and was not opened on validation or holdout. A
70 m/s diagnostic also observed a 183.4677 m/s adjacent transition; it was not
truth-repaired. Full details and the single truth-read accounting are in the
result record and manifest.

Reproduction after rebuilding the Release target (raw inputs only):

```sh
env LD_LIBRARY_PATH=/home/sasaki/.local/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH} \
  build/apps/gnss_fgo_imu_no_base \
  --android-gnss <device_gnss.csv> --android-imu <device_imu.csv> \
  --nav <brdc.nav> --out <submission.csv> --summary-json <summary.json> \
  --dataset-id 2023-05-24-20-26-us-ca-sjc-ge2/pixel7pro \
  --all-epochs --android-raw-utc-keys --fgo-imu-sparse-recovery
```

The next accuracy experiment must be a new freeze for an in-memory
`gnss_pos_vel_pdc` state/solution bridge; this coverage result must not be
reinterpreted as evidence that such a bridge exists.
