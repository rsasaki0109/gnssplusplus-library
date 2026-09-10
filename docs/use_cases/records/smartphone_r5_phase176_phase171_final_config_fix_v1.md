# Phase176 Phase171 final GNSS-first configuration fix

Status: implementation and focused validation complete; no raw route was run in
Phase176.

## Proven failure

The single pinned Phase175 MTV-H invocation (result commit
`bf15b9794dbecc25e61a9419513cd03f57ef6848`) aborted before native summary
serialization with:

```text
Phase164 requires the complete Point3/V/C7/D raw-P graph recipe and an empty generic Doppler family
```

The Phase171 branch in
`apps/native/gnss_fgo_imu_no_base.cpp` correctly enabled
`gnss_first_config.use_velocity_motion_factors = true`, but the common
post-branch default immediately assigned `false`.  The Phase164 admission guard
at `src/algorithms/fgo_gtsam_backend.cpp:126-154` requires that field to remain
true.  This was an application staging overwrite, not a solver or accuracy
finding.

## Minimal correction

The common assignment is now:

```cpp
gnss_first_config.use_velocity_motion_factors = phase171_imu_main;
```

Thus the dedicated Phase171 staging recipe retains the required factor, while
legacy GNSS-first paths retain their historical false setting.  No backend
guard, numerical tolerance, factor weight, input, or default selector was
changed.

## Validation

- `python3 -m unittest -v tests/test_smartphone_phase176_phase171_final_config.py tests/test_smartphone_phase173_phase171_raw_seed_routing.py`: 5/5 passed.
- The Phase176 test checks both the final application staging segment (including
  absence of a later false overwrite) and the backend's required guard.
- `cmake --build build --target gnss_fgo_imu_no_base -j2`: passed.
- Built executable SHA-256:
  `6a1253d471f77d9688380d00c709cc8d99ac524ddb709c0d4add6ac83d903dbc`
  (4,036,072 bytes).

The tests are launch-free source regressions; they do not prove a full raw H
GNSS+IMU route.  A new manifest and one fresh pinned run are required before
claiming that the Phase175 admission failure is resolved.
