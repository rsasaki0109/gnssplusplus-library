# Smartphone R5 Phase90: source C0/D velocity-handoff blocker audit

## Decision

Phase89 remains **NO-GO**. All four authorized Phase89 runs stopped at the
GNSS-first velocity-only handoff, before the C0/D IMU graph was built and
before any C0/D active solve was attempted.

The primary blocker is an invalid raw-Doppler velocity handoff contract. The
GNSS-first path rebuilds a separate FGO problem and the handoff validator
compares its exported solution and velocity vector sizes with the main
problem's epoch count without a time-keyed alignment step. It then requires
every velocity to be finite, at most 70 m/s, and present for every main-problem
epoch. A finite but poorly observed four-row solve fails the physical gate.

There is also a secondary initialization inconsistency. The Android adapter and
EpochSeed already carry DriftNanosPerSecond*c/1e9 in metres/second, but the
no-bridge Pose3+IMU backend initializes every C0/D drift state to zero. The
official source initializes its D vector from the source dclk sequence. A raw-D
seed is therefore source-aligned as an initializer, but it cannot make an
invalid direct-WLS velocity sequence valid.

The Phase89 numbers do not support blaming the C0/D equation or its
seconds/metres adaptation for the 6,985.21 m/s MTV-u value: C0/D is disabled in
the separate GNSS-first initializer, and all four Phase89 attempts stopped
before the C0/D active solve. The direct-WLS failure independently exhibits
the same large velocity/clock-rate states before C0/D is constructed.

Exactly one next candidate is frozen below for implementation and synthetic
testing. It is not an execution authorization. No raw route was rerun for this
audit.

## Sealed Phase89 evidence

The only Phase89 result used is the sealed structural result
docs/use_cases/records/smartphone_r5_phase89_source_clock_c0d_diagnostic_result_v1.json.
Its four authorized runs each invoked the GNSS-first optimizer once and
invoked the C0/D active solve zero times:

| route | handoff result | velocity gate evidence |
|---|---|---|
| 2021-03-16-18-59-us-ca-mtv-a/pixel5 | failed closed | 2159/2159, nonfinite 0, over-70 10, maximum 163.184 m/s |
| 2021-08-24-20-32-us-ca-mtv-h/pixel5 | failed closed | GNSS-first velocity count does not match observation epochs |
| 2022-04-01-18-22-us-ca-lax-t/pixel5 | failed closed | 1466/1466, nonfinite 0, over-70 19, maximum 112.537 m/s |
| 2023-03-08-21-34-us-ca-mtv-u/pixel5 | failed closed | 720/720, nonfinite 0, over-70 5, maximum 6985.21 m/s |

Every route has summary_published=false, active_c0d_solve_invocations=0, and
no active-solve telemetry. The result records zero truth/MAT/Kaggle or
precomputed-coordinate reads and no accuracy score.

The result's failed gates are consequently structural: active-solve telemetry,
exact termination/counter consistency, finite costs, at least one iteration,
strict cost decrease, C0/D invariant proof, raw-UTC/domain proof, finite
output-coordinate/speed proof, and no-bridge proof could not be established for
a solve that never reached the C0/D graph. This audit does not convert those
failures into an accuracy claim.

## Code path and count mismatch

The native entry point first builds the main raw problem. For
--native-gnss-first-velocity-only-handoff it then creates a second processor
and calls buildPseudorangeProblem(epochs, nav) again for the GNSS-first
initializer (apps/native/gnss_fgo_imu_no_base.cpp:4274-4311). The validator
receives that second result but the original main problem:

* validateGnssFirstVelocityOnlyHandoff requires both
  gnss_first_result.solution.solutions.size() and
  gnss_first_result.epoch_velocities_ecef_mps.size() to equal
  problem.epochs.size() (apps/native/gnss_fgo_imu_no_base.cpp:985-1010);
* the exported GNSS-only velocity vector is created from the initializer's own
  num_epochs (src/algorithms/fgo_gtsam_backend.cpp:1579-1587); and
* the problem builder may retain or drop an epoch at its measurement-count gate
  (src/algorithms/fgo_problems.cpp:897-912) and drops an epoch's first
  corrected-Doppler row when no adjacent epoch can identify receiver drift
  (src/algorithms/fgo_problems.cpp:951-969).

This is a fail-closed safety check, not evidence that the raw rows have a
common epoch identity. The handoff has no UTC-key or timestamp alignment
between the two independently built vectors. MTV-h's count error is therefore
an invalid handoff contract: a velocity sequence from one problem instance
cannot be indexed into another instance merely because both were built from
the same input vector. A future implementation may add a same-run,
source-epoch alignment check, but this freeze does not authorize silently
dropping, padding, reordering, or interpolating a velocity state.

The sealed result reports the count error but does not publish the two vector
sizes, so it cannot identify whether the solution vector or velocity vector was
short. The source has a concrete velocity-vector path consistent with that
error: the GTSAM backend enables GNSS velocity states only when the rebuilt
problem has nonempty undifferenced-Doppler factors
(src/algorithms/fgo_gtsam_backend.cpp:120-125). If the rebuilt problem's
quality, elevation, or adjacent-epoch gates reject all eligible Doppler rows,
the backend can still retain pseudorange epochs while exporting no velocity
states; the validator then compares that empty/short vector with the main
problem's epoch count. This is a source-supported failure path, not a claim
about an unrecorded MTV-h row count.

The GNSS-first candidate explicitly disables the C0/D factor and active
diagnostic while it builds the initializer
(apps/native/gnss_fgo_imu_no_base.cpp:4279-4293). Count and velocity failures
thus happen before C0/D is in the graph.

## Why the finite velocities become physically impossible

The corrected undifferenced-Doppler row shared by the local WLS initializer and
the GNSS-only graph is

    residual_mps = los.dot(v_receiver_ecef) + receiver_clock_rate_mps

with four unknowns [vx, vy, vz, clock_rate] and design row [los, 1]
(include/libgnss++/algorithms/doppler_velocity_wls.hpp:17-34,72-87). The
solver accepts a minimum of four rows, performs fixed robust iterations, and
then applies predeclared physical gates of 70 m/s for velocity, 2,000 m/s for
clock rate, and residual/conditioning gates
(include/libgnss++/algorithms/doppler_velocity_wls.hpp:36-46,147-175,231-274).

The Phase40 sealed direct-WLS result independently confirms this failure mode
on MTV-h: 4,724 corrected Doppler rows, 1,181 epochs with at least four rows,
144 insufficient epochs, zero valid/propagated estimates, 1,181
velocity-bound failures, 577 clock-rate-bound failures, and first solved state
velocity=8919.7537472980548 m/s, |clock_rate|=7314.3850372387406 m/s, reason
physical-gate. The state is finite and reached the algebraic solve; the
physical safety gate—not C0/D or a coordinate bridge—rejected it.

An in-memory synthetic sensitivity check using the same [los,1] equation
illustrates why a four-row solve can produce this class of finite state. Four
unit LOS rows with condition number 77548.66158491153 (well below the fixed
1e8 limit) and a one-metre/second perturbation to one row produced
|v|=13986.494034351917 m/s and |clock_rate|=13995.233564872227 m/s with zero
post-fit residual. This is not a route measurement and does not identify the
exact MTV-u LOS geometry; it establishes the supported mechanism that a
near-common-mode LOS set and a four-unknown exact solve can pass rank/condition
checks while requiring the physical gate to reject a huge state. MTV-u's
6985.21 m/s value is therefore best classified as a finite, poorly observed
raw-Doppler state exposed by the handoff, not as a C0/D seconds-vs-metres
result.

## Raw Android drift and official source parity

The native raw adapter parses optional Android DriftNanosPerSecond and converts
it to metres/second as drift_nanos_per_second*SPEED_OF_LIGHT/1e9, preserving
NaN when absent (src/io/android_raw_gnss.cpp:317-327,350-422,783-800). The
problem builder copies that value without deriving it from a coordinate or
clock difference (src/algorithms/fgo_problems.cpp:344-392).

The pinned official source makes the same conversion in
output/reproducibility-cache/gsdc2023/functions/gnsslog2obs.m:103-118 and uses
the resulting dclk as the D initial vector in
output/reproducibility-cache/gsdc2023/fgo_gnss_imu.m:40-55,102-106,325-355.
The official preprocessing file replaces D with a residual-derived estimate
for a declared phone list (output/reproducibility-cache/gsdc2023/preprocessing.m:130-172);
pixel5 is not in that list. A Phase90 raw-only candidate must not reproduce
that coordinate/baseline-dependent replacement.

In the current local no-bridge C0/D path, C states are initialized from the
problem clock seed, while D states use an optional PDC seed and otherwise zero
(src/algorithms/fgo_gtsam_backend.cpp:461-483,267-280). The PDC branch is
forbidden by the Phase89/Phase90 contract. The raw drift field is therefore
available but unused as a D initializer in the relevant path.

The pinned official C0/D factor is metre-valued:

    (C2-C1) - (D1+D2)*dt/2

with Jacobian [-1,+1,-dt/2,-dt/2]
(output/reproducibility-cache/gtsam_gnss/src/ClockFactor_CCDD.h:39-63).
The local adaptation deliberately preserves the equation in seconds/m/s as
(c2-c1)-((d1+d2)*dt/(2*C_LIGHT)), with sigma 0.1/C_LIGHT
(src/algorithms/fgo_gtsam_internal.hpp:858-972 and
src/algorithms/fgo_gtsam_backend.cpp:982-1009). This adaptation has a large
C-versus-D column scale ratio, as previously audited in Phase87, but it is not
exercised by the Phase89 handoff failures. Phase90 preserves it.

## Root-cause classification

| hypothesis | finding | status |
|---|---|---|
| Missing/weak integration test | Phase89 added all-epoch velocity, finite-cost, iteration, strict-decrease, and telemetry gates; prior tests did not reach this boundary. | Contributing process gap, now closed by fail-closed evaluation; not the numerical cause. |
| C0/D seconds-vs-metres conditioning | Real conditioning risk in the later C0/D graph, but C0/D is disabled in GNSS-first and was never attempted in Phase89. | Not the Phase89 6985.21 m/s cause; preserve unchanged. |
| Inconsistent clock/drift initialization/handoff | D is zero in the no-bridge IMU graph although raw D is available. | Secondary contributor; source-aligned raw D initialization is supported. |
| Raw-Doppler velocity observability and handoff contract | Four-unknown per-epoch states can be finite but physically huge; separate problem rebuilds are not epoch-key aligned; direct-WLS coverage is zero on the sealed MTV-h probe. | Primary supported blocker. |

The route records do not include singular values or per-epoch LOS matrices, so
the near-common-mode/conditioning explanation for any individual route is an
inference from the shared equation, finite physical-gate failures, and the
synthetic sensitivity check. It is not presented as an uninspected raw-data
measurement.

## Exactly one next candidate freeze

The evidence supports one narrowly scoped, source-aligned implementation
candidate:

    phase90_source_clock_c0d_raw_drift_direct_wls_handoff

The candidate may:

1. initialize each C0/D D state from the same-run raw
   EpochSeed.receiver_clock_drift_mps value (already in m/s), requiring a
   finite, one-to-one value for every retained problem epoch and failing closed
   otherwise;
2. use the existing direct FGOProblem::doppler_velocity_wls_estimates
   velocity-only handoff, with its current four-row WLS equations, physical
   bounds, residual/condition gates, bounded completion rules, and all-epoch
   coverage validator unchanged; and
3. expose only implementation/test diagnostics needed to prove D seed count,
   source unit, state provenance, and unchanged C0/D row accounting.

The implementation must preserve the source C0/D equation, Jacobian, 0.1 m
source sigma and current seconds conversion; direct raw P+D quality; the
optimizer algorithm; and the no-PDC/no-coordinate-copy boundary. It may not
relax the 70 m/s or 2,000 m/s gates, clip or replace rejected WLS states,
infer D from coordinates, use the historical residual-derived phone list, add
a base/coordinate bridge, read precomputed coordinates, or score accuracy.

This candidate is intentionally not expected to pass the current MTV-h
direct-WLS evidence automatically: Phase40 shows that the unmodified direct
sequence currently has zero valid coverage. Implementation and synthetic tests
must first prove that invalid coverage still fails closed. A separate
post-implementation execution manifest is required before any raw route can be
run; this Phase90 audit/freeze authorizes no route execution.

## Exact read and execution accounting

| item | count/setting |
|---|---:|
| sealed Phase89 result reads | 1 |
| sealed Phase89 freeze/manifest reads | 2 |
| sealed Phase39 result read | 1 |
| sealed Phase40 result read | 1 |
| sealed Phase87 audit and Phase88 freeze context reads | 2 |
| repository source/header files read or hashed | 10 |
| official pinned source/GTSAM files read or hashed | 9 |
| in-memory synthetic sensitivity checks | 1 |
| native solver invocations/reruns | 0 |
| raw Android GNSS reads | 0 |
| raw Android IMU reads | 0 |
| broadcast navigation reads | 0 |
| base RINEX reads | 0 |
| truth/MAT/Phase82/precomputed-coordinate reads | 0 |
| validation-holdout reads | 0 |
| Kaggle/token access | 0 |
| accuracy scoring/route selection | false |

Repository evidence files:

* apps/native/gnss_fgo_imu_no_base.cpp
* src/algorithms/fgo_problems.cpp
* src/algorithms/fgo_gtsam_backend.cpp
* src/algorithms/fgo_gtsam_internal.hpp
* include/libgnss++/algorithms/doppler_velocity_wls.hpp
* include/libgnss++/algorithms/doppler_contract.hpp
* include/libgnss++/algorithms/fgo.hpp
* include/libgnss++/core/observation.hpp
* src/io/android_raw_gnss.cpp
* include/libgnss++/core/solution.hpp

Official pinned evidence files:

* output/reproducibility-cache/gsdc2023/functions/gnsslog2obs.m
* output/reproducibility-cache/gsdc2023/preprocessing.m
* output/reproducibility-cache/gsdc2023/fgo_gnss_imu.m
* output/reproducibility-cache/gsdc2023/parameters.m
* output/reproducibility-cache/gtsam_gnss/src/ClockFactor_CCDD.h
* output/reproducibility-cache/gtsam_gnss/src/DopplerFactor_VD.h
* output/reproducibility-cache/gtsam-4.3a/gtsam/nonlinear/LevenbergMarquardtOptimizer.cpp
* output/reproducibility-cache/gtsam-4.3a/gtsam/nonlinear/NonlinearOptimizer.cpp
* output/reproducibility-cache/gtsam-4.3a/gtsam/nonlinear/internal/LevenbergMarquardtState.h

## SHA-256 evidence pins

| artifact | SHA-256 |
|---|---|
| Phase89 result | 9c61651f83a9b349866299b746c9e66f0ec8385b447a674692bab66d8b3c2f94 |
| Phase89 freeze | b379d508129712107f1e582b68cf0f0072c96d49dd2de181593fbfd89639140b |
| Phase89 manifest | 60a79956d693f154ab453d60109b66bf8ec041c8934da95625653b33831a0e85 |
| Phase39 result | dc8afbe58d89ef6453082942a3291924917e2a6c225d532cbcf87347b7b97fde |
| Phase40 result | ea1d92b2f69bf6810260e17d05ae4f3c0276e1ab7be6cec84a1f35b0fd5607b1 |
| apps/native/gnss_fgo_imu_no_base.cpp | a8511d42148dbff1d3833efd7ca3ff95aa575e955ea9da71e5db28b4aa59a6d3 |
| src/algorithms/fgo_problems.cpp | 725167ccc21a62e61c4da9852726ed5cafbfc05ab794eaa76c4641676f576245 |
| src/algorithms/fgo_gtsam_backend.cpp | 3935f1efa9a7fda4700188a3c57c0d466fc808f61c13d467e1954fe385990f7d |
| src/algorithms/fgo_gtsam_internal.hpp | 7221aaa53360a81aeea3f44ee293e7e1b08d30d368ac18cd7b839fbeb34c6beb |
| include/libgnss++/algorithms/doppler_velocity_wls.hpp | ef888b3e6c2f15d0dd035c0905bafdee463e346c549f81cf892ec30c1c4dcde0 |
| src/io/android_raw_gnss.cpp | 1bb9369e7db651e53fddab2a92c2f217512667bb349f70aa52934aeb80cbf387 |
| official gnsslog2obs.m | 665ce43d1fc3c5a9c3b3a6fd76f81539126e3ab897fbba484ab0ccd18964acff |
| official preprocessing.m | 976629d187e7fab5868eb8e5676a4d40520eb23db1254f7320d3b4270d7dffcf |
| official fgo_gnss_imu.m | c70090ccb8b27fc8ac7fd2929e2f995a14cfe7f089bb1fef067370e22051c3e3 |
| official parameters.m | 518925e9c75c7a14fceb5cd99432fe883311e0d64c72b94396744ac765120f52 |
| official ClockFactor_CCDD.h | 7c174217218aa0059155e1e5a7182c69d56a3cd389d0ebd95c242d6d8c65b9fc |
| official DopplerFactor_VD.h | de2c11d06ff860a95785c84620e5ed57dcfd5ca24995a0e642e9a5984fe093e1 |
| official GTSAM LevenbergMarquardtOptimizer.cpp | 834f732ac4ad4130b2a719a3181e5dfcd2ecdf584ece6f0c76bdf5f390e10cbc |
| official GTSAM NonlinearOptimizer.cpp | 594fd9bf3d48199f00d05794913df6c84d300664458d609fcdf0fb6ed3215a35 |
| official GTSAM LevenbergMarquardtState.h | 16964863621ca3aaadf6413c86adf202d566a4548a1a5a01a66b2aae286e7287 |
