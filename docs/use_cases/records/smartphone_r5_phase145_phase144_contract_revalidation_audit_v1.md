# Phase145 Phase144 contract revalidation audit

Execution label: Luna Max  
Phase: 145  
Status: metadata-only revalidation; historical Phase144 remains immutable.

## Finding

The Phase144 `configured_max_iterations: expected 1000, got 12` failure is a
validator contract error.  It is not selector loss, a serializer truncation,
or a runtime iteration regression.

The native application initializes the IMU/Pose3 main configuration with
`config.max_iterations = 12` at
`apps/native/gnss_fgo_imu_no_base.cpp:9437-9440`.  The Phase143 selector is
admitted and copied into the library configuration at `:9710-9716`, but does
not rewrite that pre-backend configuration field.  The backend computes the
effective optimizer bound at
`src/algorithms/fgo_gtsam_backend.cpp:2236-2239` using
`src/algorithms/fgo_gtsam_internal.hpp:249-254`, where the enabled main graph
maps configured `12` to effective `1000`.  The native report then copies the
pre-override value and `params.getMaxIterations()` at
`src/algorithms/fgo_gtsam_internal.hpp:361-367`; the report is serialized
without inference at `src/algorithms/fgo_gtsam_backend.cpp:2290-2297` and
`apps/native/gnss_fgo_imu_no_base.cpp:9012-9021`.

The Phase144 validator incorrectly required `configured_max_iterations ==
1000` for both stages at
`apps/commands/benchmarks/gnss_smartphone_phase144_telemetry_serializer_structural.py:334-341`.
Its stage call also asks for `gnss_first`, while the native report uses the
canonical source label `gnss-first` (`fgo_gtsam_backend.cpp:2292` and the
Phase143 validator).  The Phase144 serializer is therefore faithful to the
native report.

## Sealed metadata evidence

The Phase145 lane reads only the Phase144 result metadata and the two
`native_summary.json` paths named by that result.  It does not open the raw
GNSS/IMU/navigation/base paths listed in the argv metadata, solution rows, or
any truth/MAT/PDC/Kaggle artifact.

| route | main configured/effective | main accepted / branch | GNSS-first configured/effective | GNSS-first accepted / branch |
|---|---:|---|---:|---|
| MTV-A | 12 / 1000 | 25 / `outer_convergence_tolerance` | 1000 / 1000 | 111 / `outer_convergence_tolerance` |
| LAX-T | 12 / 1000 | 31 / `outer_convergence_tolerance` | 1000 / 1000 | 116 / `outer_convergence_tolerance` |

All four reports have zero rejected outer iterations, finite costs with strict
decrease, complete native traces, valid configuration, no fallback, and
accepted iterations below the effective cap.  Main uses
`MULTIFRONTAL_QR`/`EliminateQR`; GNSS-first uses
`MULTIFRONTAL_CHOLESKY`/`EliminatePreferCholesky`; both use `COLAMD`.

## Nearby termination-field audit

`maximum_lambda` is an observed peak, not the configured upper bound.  The
native telemetry initializes it from the initial/final lambda and raises it
from observed trial lambdas at
`src/algorithms/fgo_gtsam_internal.hpp:644-695`; the configured bound is
reported separately through `params.getlambdaUpperBound()` at
`fgo_gtsam_internal.hpp:387-393`.  The sealed values are therefore valid:
MTV-A main/GNSS-first `1e-5`/`1e-5`, LAX-T main/GNSS-first `1`/`1e-5`, all
within the configured `[0, 100000]` range.  Requiring the observed field to
equal `100000` would be another semantic mismatch.

The Phase145 validator requires the native stage labels, the exact frozen
configured/effective split, the original tolerances, lambda policy, damping,
solver/elimination/order, count conservation, branch/cap relationship,
finite monotonic costs, no-fallback and complete-trace markers.  It delegates
all unchanged Phase144 equation, factor-family, clock, raw-base, bridge,
offset, output, policy, and duplicate-free JSON gates to the immutable
Phase144 validator on a detached in-memory copy.  Only the two known Phase144
termination naming/value assumptions are adapted in that copy; source
summaries are never overwritten.

## Phase145 implementation and regression boundary

Validator:

`apps/commands/benchmarks/gnss_smartphone_phase145_phase144_contract_revalidation.py`

Focused tests cover acceptance of the native `12 -> 1000` split and rejection
of effective `12`, missing/invalid fields, an inactive selector, and recursive
duplicate JSON keys.  Existing Phase144 artifacts remain untouched.  The
revalidation result is a separate Phase145 record and reports zero raw reads,
solver invocations, solution-coordinate reads, truth/MAT/PDC reads, accuracy
calculations, and Kaggle access.
