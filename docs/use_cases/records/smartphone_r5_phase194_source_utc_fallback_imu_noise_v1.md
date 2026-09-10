# Smartphone R5 Phase194: source UTC-fallback IMU measurement-noise parity

Status: implementation and synthetic verification complete; no native H run,
truth read, or accuracy score was performed in this phase.

## Bounded change

The new default-off selector
`--native-phase194-source-utc-fallback-imu-noise` is accepted only for the
Pixel5 Phase171 raw-P/ECEF-Doppler GNSS-first + Pose3/IMU lane with explicit
Android UTC wall-clock fallback.  The policy is applied only after the native
Android loader reports `utc_wall_clock_fallback_applied=true`.

When applied, the source coefficient-1.0 measurement densities are selected:

- accelerometer white-noise sigma: `0.05` (covariance diagonal `0.0025`),
- gyroscope white-noise sigma: `0.001` (covariance diagonal `0.000001`).

The existing loader alignment coefficient remains `0.5` and the existing
selector-off densities remain `0.025`/`0.0005`.  Bias random-walk densities
(`0.00025`/`0.0000005`), integration sigma (`0.05`), timing/mapping, and all
factor/solver settings are unchanged.  The GTSAM backend squares only the two
selected white-noise densities when constructing the corresponding
`PreintegrationCombinedParams` covariance blocks.

The normal summary reports selector requested/applied state, actual fallback
state, source branch, both sigmas and covariance diagonals, alignment
coefficient, and explicit unchanged bias/integration markers.

## Verification

- `cmake --build build --target gnss_fgo_imu_no_base gnss_run_tests -j2` passed.
- Focused C++ filter: 9/9 passed, including the actual covariance construction
  test and existing Phase164/167/171 graph regressions.
- Phase194 source/guard/summary contracts: 4/4 Python tests passed.
- Phase193 evaluator contracts: 11/11 Python tests passed.
- `git diff --check` passed.

No H rerun is pinned or authorized from this record; the Phase193 H score and
the Phase192 native output remain unchanged and are not re-read here.
