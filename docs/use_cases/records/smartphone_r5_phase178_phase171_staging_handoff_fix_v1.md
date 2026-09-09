# Phase178 Phase171 raw-P staging handoff admission fix

Status: implementation and validation complete; no raw route was run in
Phase178.

## Proven Phase177 failure

After the Phase176 velocity-motion-factor correction, the single pinned
Phase177 MTV-H invocation failed closed before summary serialization with:

```text
native source GNSS-first meter-state handoff requires either the staged Point3/velocity raw-D initializer or the main Pose3+IMU optimized-D handoff
```

The backend guard at `src/algorithms/fgo_gtsam_backend.cpp:347-355` treats
`use_native_source_clock_c0d_gnss_first_meter_state_handoff=true` on a no-IMU
staging config as requiring the legacy raw-D initializer.  The Phase171
staging graph is instead the dedicated raw-P Point3/V/C7/D graph.  Its own
backend admission path at `src/algorithms/fgo_gtsam_backend.cpp:121-206`
validates the raw-P recipe and its C7/D exports are controlled by the source
C0/D factor and epoch-vector parity, not by the legacy handoff selector.

## Minimal correction

In the Phase171 GNSS-first staging copy only,
`use_native_source_clock_c0d_gnss_first_meter_state_handoff` is now `false`.
The main Pose3+IMU config remains `true` and consumes the validated same-run
C7/D handoff.  No raw-D initializer, backend guard relaxation, numerical
tolerance, factor weight, default selector, or input was changed.

## Validation

- `python3 -m unittest -v tests/test_smartphone_phase178_phase171_staging_handoff.py tests/test_smartphone_phase176_phase171_final_config.py tests/test_smartphone_phase173_phase171_raw_seed_routing.py`: 7/7 passed.
- `LD_LIBRARY_PATH=/home/sasaki/.local/lib:$LD_LIBRARY_PATH build/tests/run_tests --gtest_filter='RawPSeedTest.*:RawPSeedAdapterTest.*:FGOGtsamPhase164RawNoDopplerGraphTest.*:FGOGtsamPhase165RawNoDopplerGraphTest.*:FGOGtsamPhase171NoDopplerImuMainTest.*:FGOGtsamPhase167RawNoDopplerTerminationTest.*'`: 40/40 passed.
- `cmake --build build --target gnss_fgo_imu_no_base -j2`: passed.
- Rebuilt executable SHA-256:
  `4b75cb79e65be240219ba07c515b0d8d184db913d70fdf25798bfa247057a76d`
  (4,036,072 bytes).

The static staging test and library synthetic graph tests do not prove a full
raw H GNSS+IMU run.  A new manifest and one fresh pinned invocation are needed
to evaluate the corrected application path.
