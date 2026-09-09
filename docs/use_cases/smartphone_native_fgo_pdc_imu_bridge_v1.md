# Native raw PDC state bridge with IMU (Phase 9)

This is an opt-in development lane. It consumes only Android
`device_gnss.csv`, Android `device_imu.csv`, and broadcast `brdc.nav`. It does
not read or generate MATLAB files, sample coordinates, device-WLS columns,
precomputed trajectories, or truth during inference. The frozen contract and
hash chain are in
`records/smartphone_r5_gsdc2023_native_fgo_pdc_imu_bridge_freeze_v1.json` and
its manifest.

The bridge solves an in-memory per-epoch P+D state (ECEF position, five clock
biases, ECEF velocity, and clock range-rate) with the existing native PDC
temporal equations. Its finite states are initial values only. The normal
GTSAM graph then consumes each raw P/D row once and adds the existing Pose3,
velocity, bias, and CombinedImuFactor chain; no second PDC measurement prior
is inserted. Carrier/TDCP and base/DD factors are disabled for this raw phone
lane.

The first structural run on
`2023-05-24-20-26-us-ca-sjc-ge2/pixel7pro` used the fixed recipe from the
freeze record. It produced 27,462 pseudorange rows, 27,440 Doppler rows,
1,383 temporal intervals, 207 finite PDC state seeds, and 1,383 CombinedImu
intervals. The PDC solve converged in 48 iterations and the final FGO solve
converged in 7 iterations; output had 1,383 finite raw-UTC target epochs,
with wall time 10.83 s and peak RSS 155,764 kB. PDC-to-final-FGO seed
displacement was P50 461.0862621490895 m and maximum 984.7570120024196 m.

The dedicated `gnss_pos_vel_pdc` executable still has a wider measurement
contract (15,704 P, 15,704 D, and 13,464 TDCP rows in the corresponding
diagnostic), while this bridge currently uses the FGO-filtered P/D rows and no
TDCP. Therefore this lane is not claimed as exact dedicated-PDC parity. The
The sealed development-train score is WGS84 linear diagnostic 5.927331890307588
m, below the frozen 68.0475838937948 m gate, with zero missing, extra, or
non-finite output keys. This is one route only and does not authorize validation
or holdout promotion. Production defaults are unchanged.

The score is not an isolated attribution of the PDC seed. The bridge run also
uses a different raw factor-selection contract (`spp_model_intersystem_bias`
disabled, sparse epoch retention, and 27,462 P rows versus 4,874 in the
Phase8 comparator), and the final FGO moves from the finite PDC seeds by P50
461.0862621490895 m (maximum 984.7570120024196 m). The score record therefore
labels its comparison as a Phase8 legacy factor-selection comparator; a same-
factor/no-seed ablation was intentionally not rerun after the sealed truth
evaluation.

Reproduce the structural run after building with:

```sh
cmake --build build --parallel 4 --target gnss_fgo_imu_no_base run_tests
LD_LIBRARY_PATH=/home/sasaki/.local/lib:$LD_LIBRARY_PATH \
  build/tests/run_tests --gtest_filter=PdcStateBridgeTest.*
LD_LIBRARY_PATH=/home/sasaki/.local/lib:$LD_LIBRARY_PATH \
  build/apps/gnss_fgo_imu_no_base \
  --android-gnss <device_gnss.csv> --android-imu <device_imu.csv> \
  --nav <brdc.nav> --out <submission.csv> --summary-json <summary.json> \
  --all-epochs --android-raw-utc-keys --native-pdc-state-bridge
```
