# Smartphone native FGO v3: heading-optional initialization

This is a development-only research lane. It preserves the v2.1 graph
(undifferenced pseudorange, ordinary TDCP, position/clock motion, Huber loss,
and `CombinedImuFactor`) and changes only the initialization failure policy.
Static gravity aligns roll/pitch. A multi-epoch GNSS course is used for yaw only
when the frozen speed, path, span, and circular-scatter gates pass. Otherwise a
finite `pi`-radian yaw prior controls the Pose3 gauge and yaw remains jointly
estimated by the graph. Non-finite initialization, solver conditioning failure,
or innovation failure disables IMU and runs the exact v1 preserve-TDCP/motion
fallback. Production defaults are unchanged.

The pinned upstream comparison is MIT `taroz/gsdc2023` commit
`29923f9f370f09ebc00f96d8cca375007a18e7d5`: `fgo_gnss_imu.m` uses
`vel2rpy` with a 0.5 m/s threshold and nearest-filled low-speed headings,
whereas this lane explicitly keeps weak yaw broad and finite. The source and
line evidence are recorded in
`docs/use_cases/records/smartphone_r5_gsdc2023_native_fgo_v3_heading_optional_freeze.json`.

## Reproduction

The identity and parameters were frozen before materializing the two new
route/device payloads in the freeze record. Rebuild with:

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --target gnss_fgo_imu_v3_heading_optional -j2
python3 tests/test_smartphone_native_fgo_v3_heading_optional.py
```

After freeze, the truth-free route command is:

```sh
LD_LIBRARY_PATH=/home/sasaki/.local/lib:$LD_LIBRARY_PATH \
  build/apps/gnss_fgo_imu_v3_heading_optional \
  --obs <adapter/rover.obs> --nav <inputs/brdc.nav> \
  --imu <inputs/imu_processed.csv> --out <candidate/submission.csv> \
  --summary-json <candidate/run_summary.json> \
  --dataset-id <route/phone> --skip-epochs 200 --max-epochs 30
```

Truth-free materialization, adapter output, candidate/detail output, and the
exact-v1 reference are sealed under
`output/smartphone-r5/native-fgo-v3-heading-optional-v1`. The two weak-heading
structural smokes were `2021-07-19-20-49-us-ca-mtv-a/pixel5` at
`--skip-epochs 950` and `2023-09-05-23-07-us-ca-routen/pixel5` at
`--skip-epochs 0`: unchanged v2.1 failed its heading gate, while v3 produced
finite 30-row output with 29 CombinedImuFactors and the broad yaw prior.

The first authorized train-truth evaluation was attempted only after those
artifacts were sealed. Its scorer stopped on an empty matched-distance
percentile (`TypeError: None + None`) after reading each declared truth member
once. It was sealed as No-Go without reopening truth, changing code/parameters,
or opening validation/holdout. See:

- `docs/use_cases/records/smartphone_r5_gsdc2023_native_fgo_v3_heading_optional_freeze.json`
- `docs/use_cases/records/smartphone_r5_gsdc2023_native_fgo_v3_heading_optional_truth_free_run.json`
- `docs/use_cases/records/smartphone_r5_gsdc2023_native_fgo_v3_heading_optional_train_score_failure.json`

The scorer failure is an evaluation/schema blocker, not evidence of an
algorithmic improvement. No promotion or validation access is authorized until
a separately approved evaluator audit.

## Evaluator recovery

The separately authorized recovery changed only null handling and diagnostic
schema normalization in the evaluator; the v3 source, Release binary, profile,
and sealed route outputs remained byte-identical. The same two materialized
truth files were read once more as the explicitly recorded recovery read, with
identical SHA256 values before and after. The recovery report is:

- `output/smartphone-r5/native-fgo-v3-heading-optional-v1/evaluator-recovery-v1/train_evaluation_recovery.json`
- `docs/use_cases/records/smartphone_r5_gsdc2023_native_fgo_v3_heading_optional_evaluator_recovery_result.json`

Aggregate H-P95 improved from 2.7133 m to 2.3981 m and the four-diagnostic mean
from 2.3449 m to 2.2556 m, but the route gate failed: the 2021 route regressed
V-P95 and the 2023 route regressed H-P50. The candidate is therefore No-Go;
fresh validation remains unopened.

The unrelated GTSAM parity failures were independently reproduced in a clean
HEAD worktree. A current backend clock-motion addition was found to create an
unanchored clock gauge for DD-only problems with no clock state. Requiring
`need_clock_states` fixes the defect; the two exact tests pass again without
weakening thresholds. Evidence is recorded in
`docs/use_cases/records/gnss_fgo_gtsam_backend_parity_recovery.json`.
