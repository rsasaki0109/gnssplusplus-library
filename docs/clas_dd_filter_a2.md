# CLAS DD PPP-RTK Filter A2

A2 adds gated DD-native LAMBDA conditioning to the A1 double-differenced
CLAS scaffold and removes the A1 static position anchor pseudo-measurement.
The path remains behind `GNSS_PPP_CLAS_DD_FILTER=1`; the default CLAS and
MADOCA paths do not call this code.

## Implemented

- `DdFilterScaffold::processFloatUpdate()` now attempts integer fixing after a
  successful DD float update.
- The ambiguity vector is built from the same phase rows selected by
  `buildDdMeasurementSystem()`, using `StateLayout::ambiguityIndex()` for the
  reference and target satellites in each DD group.
- DD ambiguity covariance and ambiguity-vs-head covariance are extracted through
  `rtk_measurement::buildAmbiguityTransform()` on the DD scaffold state and
  covariance.  The old PPP ambiguity maps in `ppp_ar.cpp` and `ppp_wlnl.cpp`
  are not used.
- The full DD candidate vector is attempted first.  If the full vector does not
  pass the ratio gate, A2 attempts the highest available per-frequency DD
  subset from the same row builder groups.  This is a minimal PAR bridge for
  CLASLIB's `posopt7=on` oracle configuration; post-fit fixed validation and
  hold policy remain A3.
- On acceptance, the DD head state is conditioned with
  `xa = x - Qab * Qb^-1 * (b_float - b_fixed)`, the head covariance is
  downdated with `Pa = Paa - Qab * Qb^-1 * Qab'`, and compatible
  single-satellite ambiguity values are restored into the fixed snapshot.
- Accepted fixed snapshots are fed back into the DD scaffold so subsequent
  epochs carry the accepted ambiguity datum.  No hold pseudo-measurements are
  added in A2.
- The A1 anchor row was removed: no `appendStaticPositionAnchor()`, no
  anchor variance constant, and no static distance fallback guard remain.  The
  only stabilization retained is the initial DD position covariance
  (`0.003 m^2`) used when seeding the DD state from the first native float.

## Anchor Removal Measurement

2019-08-27 CLAS, same-epoch `/tmp/s31_ref/claslib_oracle_xyz_full.pos` oracle,
`--ar-ratio-threshold 2.0`, 3599 matched epochs.

| Path | Fixed epochs | Fixed 3D mean / p95 | Fixed Up mean / p95 | All 3D mean / p95 | LAMBDA accepted / rejected | Mean attempted ratio |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| A1 anchor, no A2 LAMBDA | 0 | n/a | n/a | 1.721532 / 3.335225 m | 0 / 0 | n/a |
| A2 with A1 anchor still present | 169 | 2.138678 / 3.870272 m | 1.785294 / 2.749376 m | 1.721540 / 3.335829 m | 169 / 2420 | 1.450616 |
| A2 final, anchor removed | 1073 | 1.342069 / 2.868478 m | 0.709820 / 1.918809 m | 1.161354 / 2.914994 m | 1073 / 1576 | 84.636654 |

The final no-anchor path fixes more epochs than the native baseline fixed count
(301) and improves the A1 float oracle mean/p95 3D and Up errors, but it does
not yet reach CLASLIB/native fixed accuracy.  That remaining gap is the A3
hold/validation target.

## A3 Entry Points

- `DdFilterScaffold::conditionFixedSnapshot()`: add CLASLIB-style post-fix
  residual validation around the accepted `fixed_snapshot_` before feedback.
- `DdFilterScaffold::processFloatUpdate()`: add holdamb-style pseudo-row
  feedback only after min-fix, ratio, PDOP, and postfit gates pass.
- `buildDdMeasurementSystem()`: preserve reference-satellite continuity and
  expose reference changes so A3 can reset or withhold ambiguity feedback on
  unstable groups.
- `DdUpdateDiagnostics`: surface accepted/rejected reasons and subset type if
  A3 needs per-epoch validation telemetry.
