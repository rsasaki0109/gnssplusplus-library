# Phase171 same-run raw-P GNSS-first to IMU-main handoff

Status: implementation and synthetic validation complete; no real route run was
performed in Phase171.

## Contract

`--native-phase171-raw-p-no-doppler-imu-main` is default-off and requires the
Phase165 raw-P graph plus the Phase167 GNSS-first 1000-iteration selector.  The
application runs the raw-P Point3/velocity graph and the Pose3/IMU main graph in
one process.  Only the in-memory finite result is handed over: retained epoch
order, ECEF position/clock, C7 clock vector, and D state are coverage-checked;
no serialized seed, second SPP solve, raw-D fallback, or fabricated observation
is allowed.

The main graph uses the existing source C0/D meter factors, P rows, TDCP rows,
motion/clock-motion factors, and IMU preintegration.  Empty generic Doppler is
admitted only by the dedicated Phase171 configuration.  ECEF velocity is
converted to ENU once for IMU initialization.  C7 components absent from the
retained observations receive the existing explicit weak numerical gauge; this
is an initialization/gauge device, not a measured ISB or an observational-rank
claim.  Missing, nonfinite, or misaligned C7/D handoff data fails closed.

The Phase171 main pass retains the Phase143 source termination sidecar and its
configured/effective distinction (configured 12 in the main pass, effective
1000 from the official budget).  The GNSS-first staging pass remains the
dedicated 1000-iteration raw-P graph.  The stage and main inherit the required
direct-observable-quality contract, but are not claimed to have identical
measurement-quality/diagnostic settings or costs; Phase171 is an integration
boundary test, not an accuracy or leaderboard result.

## Evidence

Source sites:

- `apps/native/gnss_fgo_imu_no_base.cpp`: selector admission, same-invocation
  staging copy, exact handoff validation, ECEF-to-ENU velocity handoff, and
  no-fallback guards.
- `src/algorithms/fgo_gtsam_backend.cpp` and
  `src/algorithms/fgo_gtsam_internal.hpp`: dedicated empty-D admission, C0/D
  factor path, and absent-clock numerical gauge diagnostics.
- `include/libgnss++/algorithms/fgo_config.hpp` and `fgo.hpp`: default-off
  selector and diagnostic fields.

Synthetic test coverage is in
`tests/test_fgo_gtsam_backend.cpp`:

- `SameRunC7DHandOffAdmitsEmptyDopplerAndGaugesAbsentClockSlots` first solves
  the raw-P stage, copies its finite position/clock/C7/D exports, performs the
  single ECEF-to-ENU velocity conversion, inserts a physically consistent TDCP
  row, and solves the Pose3/IMU graph with empty generic Doppler.
- `RejectsMissingNonfiniteClockHandoffAndLeavesLegacySelectorOff` checks
  missing D, nonfinite C, and the default-off legacy boundary.

Commands and results:

```text
cmake --build build --target gnss_run_tests -j2        PASS
cmake --build build --target gnss_fgo_imu_no_base -j2 PASS
LD_LIBRARY_PATH=/home/sasaki/.local/lib build/tests/run_tests \
  --gtest_filter='*Phase93*:*Phase101*:*Phase135*:*Phase164*:*Phase165*:*Phase167*:*Phase171*'
  28/28 PASS
LD_LIBRARY_PATH=/home/sasaki/.local/lib build/tests/run_tests \
  --gtest_filter='FGOGtsamPhase171NoDopplerImuMainTest.*'
  2/2 PASS
```

A full Phase171 selector argv with nonexistent GNSS/IMU/nav paths reached raw
GNSS input opening and returned `failed to convert raw Android GNSS`; it did not
enter the Phase143 affine/raw-base admission gate.  This was a parser/admission
smoke only, not a data read or solver run.  No raw GNSS/IMU/nav, truth, MAT,
Kaggle, or real FGO execution occurred in this phase.

## Limits and next boundary

The synthetic tests prove the native handoff/topology and fail-closed checks,
not full Android parsing, route coverage, numerical accuracy, or IMU behavior
on H170 data.  A future real run must pin a new command/input/binary manifest
and independently assess the main IMU result; this record does not authorize or
claim that run.
