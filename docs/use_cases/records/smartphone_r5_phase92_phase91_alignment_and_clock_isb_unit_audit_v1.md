# Smartphone R5 Phase92: Phase91 alignment and clock/ISB unit audit

## Decision

Phase91 remains **NO-GO**.  The sealed Phase91 result is a structural
diagnostic only; it does not authorize accuracy scoring, route selection, or a
submission.  This audit makes two findings:

1. The MTV-h and MTV-u Phase91 failures are an implementation alignment bug.
   The validator compared retained FGO epochs with the full, pre-filter raw
   epoch vectors and required equal total counts.  The correct contract is an
   exact retained-epoch-to-raw-key mapping: raw epochs filtered out before the
   retained problem are allowed to be unmatched, while every retained epoch
   must map exactly once, in source order, to its raw UTC key and raw drift.
   This is a correction to Phase91 implementation alignment, not a gate
   relaxation.
2. The raw-D initializer did not produce an active solve.  MTV-a and LAX-t
   both reached the main graph, made ten inner LM lambda attempts, accepted no
   outer iteration, and returned unchanged cost.  Their C0/D row proxy was
   about `5.996e8` because C was in seconds while D was in metres/second.
   The evidence supports source-meter clock/ISB state parity as the one next
   implementation candidate.  It is a conditioning/unit-consistency
   hypothesis, not a promise of convergence or accuracy.

No raw Android, native route, truth, MAT, Kaggle, token, validation-holdout,
Phase82, base, or precomputed-coordinate read was made for this audit.  No
implementation or raw execution is authorized by this record; the separately
sealed Phase92 freeze authorizes implementation only.

## Sealed Phase91 evidence

The sole route-result artifact used for run evidence was:

`docs/use_cases/records/smartphone_r5_phase91_source_clock_c0d_gnss_first_raw_drift_d_initializer_result_v1.json`

SHA-256: `9929e285b58b5d9a65350ba1a3d497c736968bb896d72dbe5fb014534b6d58c8`.

The two routes that published a main-graph summary had complete retained raw-D
coverage and a valid same-run GNSS-first handoff, but neither had active LM
progress:

| route | retained epochs | C0/D rows | raw D range (m/s) | inner lambda attempts | accepted outer iterations | graph iterations | initial cost | final cost | conditioning proxy |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| `2021-03-16-18-59-us-ca-mtv-a/pixel5` | 2159 | 2158 | -28.379243850708011 .. -0.68522566556930542 | 10 | 0 | 0 | 162235823.07086429 | 162235823.07086429 | 599584915.42414284 |
| `2022-04-01-18-22-us-ca-lax-t/pixel5` | 1466 | 1465 | -9.870448112487793 .. 8.2797584533691424 | 10 | 0 | 0 | 284569697.45147896 | 284569697.45147896 | 599584915.40669262 |

Both had finite initial/final costs, `termination_trace_complete=false`, and
`termination_branch_reason="no_progress_unclassified"`.  Their maximum
whitened C column norm was `2997924580`; the maximum whitened D column norms
were `5.000000004802132` and `5.0000000049476512`.  Thus the published
conditioning proxy is the measured ratio of those diagnostic columns, not a
claim about the full graph condition number.

The other two routes failed closed before main-graph construction with the
same sealed native-log error, `retained epoch counts are not identical`:

* `2021-08-24-20-32-us-ca-mtv-h/pixel5`
* `2023-03-08-21-34-us-ca-mtv-u/pixel5`

Their failure occurred in the Phase91 GNSS-first alignment gate, so it is not
evidence of a C0/D solve or a route accuracy result.  The Phase91 record
reports four authorized attempts, four GNSS-first invocations, two main C0/D
invocations, raw device GNSS/IMU/navigation reads only, and zero truth/MAT/
Kaggle/precomputed-coordinate/accuracy reads.

## Retained-epoch alignment finding

The Phase91 raw entry point snapshots `android_raw_epoch_times` and
`android_gnss.epoch_utc_time_millis` immediately after raw conversion
(`apps/native/gnss_fgo_imu_no_base.cpp:3938-3962`).  The loader guarantees that
those two vectors are parallel to the full `observations.epochs` vector and
have equal size (`src/io/android_raw_gnss.cpp:1042-1056`,
`include/libgnss++/io/android_raw_gnss.hpp:122-136`).  They are therefore
**pre-filter source vectors**, not necessarily the retained main-graph
sequence.

The main problem is built later from `epochs`
(`apps/native/gnss_fgo_imu_no_base.cpp:4093-4109,4379-4381`).  In
`buildPseudorangeProblem`, the builder creates an `EpochSeed` for each input
epoch and carries the raw `receiver_clock_drift_mps` into that seed
(`src/algorithms/fgo_problems.cpp:344-392`).  It can skip an epoch when no
finite seed is available (`:377-390`) and, unless sparse retention is enabled,
can skip an epoch below the usable-measurement floor (`:897-912`).  The
resulting `problem.epochs` is consequently an ordered retained subset of the
input vector.  Dropping short raw epochs is normal filtering behavior and is
not coordinate substitution.

The current Phase91 validator receives the retained main/GNSS-first/result
times but the unfiltered raw vectors
(`apps/native/gnss_fgo_imu_no_base.cpp:1049-1077,4621-4627`).  Its
`counts_match` predicate explicitly requires
`raw_epoch_count == main_epoch_count` and
`raw_utc_key_count == main_epoch_count`
(`include/libgnss++/algorithms/source_clock_c0d_initializer.hpp:120-125`).
Only after that check does it compare timestamps by equal vector index
(`:137-158`), and it reports `retained epoch counts are not identical`
(`:171-197`).  This is exactly the wrong cardinality relation when the raw
source has filtered-out epochs.  The GNSS-first candidate also starts from the
same `problem` instance in the standard mode (`apps/native/gnss_fgo_imu_no_base.cpp:4590-4594`),
so the mismatch is between retained problem epochs and the full raw vectors,
not proof of two different coordinate or solver inputs.

The required correction is strict and does not weaken coverage:

* carry an immutable raw source identity for every retained `EpochSeed` (the
  exact raw source index and integer UTC millisecond key, with the GNSS week/TOW
  identity); or carry an equivalent in-memory retained-key map created at the
  raw staging boundary;
* require each retained main epoch, GNSS-first epoch, and GNSS-first solution to
  have identical time/key identity, unique source index, and unchanged source
  order;
* look up `receiver_clock_drift_mps` by that exact retained raw key and require
  one finite value for every retained epoch;
* permit only raw epochs that were filtered out to remain unmatched; and
* reject missing keys, duplicate keys, reordering, nearest-time matches,
  interpolation, padding, truncation, zero/hold fallback, and any WLS,
  residual, velocity-difference, or coordinate inference.

The future implementation must therefore replace the Phase91 full-vector
count equality with a retained-key mapping.  It must not simply change the
gate to accept a short vector or index the first `N` raw drifts.

## Why raw-D initialization did not solve the zero-step

The Phase91 change initialized only the main graph's per-epoch D state from
finite raw Android drift.  It did not change the main C state, factor units,
Jacobian, sigma, or LM algorithm.  The sealed MTV-a and LAX-t measurements
show that complete finite D coverage is compatible with zero accepted LM
steps.  A route can therefore pass raw-D provenance and still fail the active
solve gates.

The installed official GTSAM source explains the telemetry semantics:

* `LevenbergMarquardtState::increaseLambda` increments the inner-attempt
  counter, while `decreaseLambda` creates the next state with
  `iterations + 1` (`output/reproducibility-cache/gtsam-4.3a/gtsam/nonlinear/internal/LevenbergMarquardtState.h:42-94`).
* `tryLambda` updates values only on a successful model-fidelity step; an
  unsuccessful solve/step increases lambda and eventually returns at the
  lambda upper bound (`.../LevenbergMarquardtOptimizer.cpp:121-269`).
* `defaultOptimize` uses the outer `iterations()` count for its loop and can
  return normally after `iterate()` without an accepted outer step
  (`.../NonlinearOptimizer.cpp:62-117`).

The repository telemetry wrapper preserves those semantics: it maps accepted
outer iterations from `optimizer_.iterations()`, counts the summary lambda
trials, and marks an incomplete trace as incomplete
(`src/algorithms/fgo_gtsam_internal.hpp:88-208`).  The backend currently sets
`diagnostics.converged` from “no caught exception”
(`src/algorithms/fgo_gtsam_backend.cpp:1271-1283`); Phase86/91 active gates
correctly require more than that bit.  Therefore Phase91's ten attempts,
zero accepted iterations, unchanged costs, and unclassified incomplete
termination are a real no-progress solve, not a hidden successful iteration.

## GTSAM batch clock and ISB unit audit

The local native Eigen state already uses metre-valued clock and constellation
bias columns: the initial clock column is `receiver_clock_bias_m`, clock priors
use `clock_motion_sigma_m`, and output divides the optimized metre state by
`SPEED_OF_LIGHT` to return API seconds (`src/algorithms/fgo.cpp:232-264,494-531,2273-2287`).
The GTSAM batch backend has a different convention:

| GTSAM key/state | Current batch unit and initialization | Current batch consumers | Required parity behavior |
|---|---|---|---|
| `clockKey('c')`, one per retained epoch | seconds; `receiver_clock_bias_m / C_LIGHT` at `src/algorithms/fgo_gtsam_backend.cpp:539-552` | all plain/ISB pseudorange factors, stock undifferenced carrier factor, `TimeDifferencedCarrierFactorArm`, legacy clock `BetweenFactor`, C0/D, clock prior, output and residual diagnostics | store C in metres; use direct metre terms/Jacobians in every enabled factor; divide by C_LIGHT only at the public `PositionSolution` boundary |
| `isbKey('i')`, one global node per non-GPS clock group | seconds; initialized to `0.0` at `:555-563` | every `PseudorangeFactorISB*` and residual RMS recomputation | store constellation ISB in metres; direct metre term/Jacobian and metre prior/output diagnostics |
| `signalBiasKey('f')`, static secondary receiver IFB | already metres; initialized/prior/output directly in metres (`:565-575,1193-1200,1542-1556`) | signal-bias pseudorange variants | preserve metre unit and factor coefficient `+1`; it is not the C/ISB conversion target |
| `residualIonosphereKey('j')` | metres | optional residual-ionosphere pseudorange variants | preserve metres and coefficient; not an ISB |
| `dopplerClockDriftKey('d')`, one per epoch | metres/second; raw-D, Doppler, and broad prior use direct m/s (`src/algorithms/fgo_gtsam_backend.cpp:328-349,761-789,1231-1244`) | undifferenced Doppler and C0/D | preserve m/s; no C_LIGHT conversion |

The batch clock/ISB factor coverage is complete in
`src/algorithms/fgo_gtsam_backend.cpp` and
`src/algorithms/fgo_gtsam_internal.hpp`:

* Plain and ISB pseudorange factors are emitted for Point3 and Pose3/lever-arm
  paths, with optional signal-bias and residual-ionosphere variants
  (`fgo_gtsam_backend.cpp:623-758`; factor equations and Jacobians in
  `fgo_gtsam_internal.hpp:1000-1070,1072-1353,1367-1743`).  Every current
  base-clock and ISB derivative is `C_LIGHT`; signal-bias and ionosphere
  derivatives are already `1` and their metre coefficient respectively.
* The optional stock `gtsam::CarrierPhaseFactor` is constructed with a
  seconds clock (`fgo_gtsam_backend.cpp:797-814`); the installed API documents
  its receiver clock as seconds and carrier ambiguity as metres
  (`/home/sasaki/.local/include/gtsam/navigation/CarrierPhaseFactor.h:20-79`).
  The ordinary Pose3 TDCP analogue also multiplies both clocks by C_LIGHT
  (`fgo_gtsam_internal.hpp:1745-1824`).  The parity implementation must use a
  metre-aware equivalent or an explicit, audited boundary conversion whenever
  either path is enabled; it may not leave a mixed-unit key.
* The current source C0/D factor is explicitly seconds/m/s:
  `error=(c2-c1)-((d1+d2)*dt/(2*C_LIGHT))`, Jacobian
  `[-1,+1,-dt/(2*C_LIGHT),-dt/(2*C_LIGHT)]`
  (`fgo_gtsam_internal.hpp:858-972`).  The backend supplies sigma
  `0.1/C_LIGHT` and computes its conditioning proxy with the same conversion
  (`fgo_gtsam_backend.cpp:1003-1078`).
* Legacy clock motion uses a seconds `BetweenFactor<double>` with
  `clock_motion_sigma_m/C_LIGHT` (`fgo_gtsam_backend.cpp:1003-1088`), and the
  optional clock prior uses `clock_prior_sigma_m/C_LIGHT`
  (`:1186-1191`).
* Output currently reads `optimized.at<double>(clockKey(i))` as API seconds
  (`:1591-1606`).  Pseudorange residual diagnostics add C_LIGHT times the
  clock and ISB states (`:1717-1755`).  Both are conversion boundaries that
  must be changed together for the opt-in metre state.
* Double-difference pseudorange/carrier factors do not consume these
  undifferenced clock or ISB keys; the fixed-lag backend is DD/IMU-only and
  has no `clockKey`/`isbKey` batch state.  Those factors remain outside this
  unit conversion.

The official pinned source agrees with the metre convention.  `GobsPhone.m`
labels `clk` as metres and `dclk` as metres/second
(`output/reproducibility-cache/gsdc2023/functions/GobsPhone.m:9-16`),
`gnsslog2obs.m` computes both with C_LIGHT (`:103-118`), and
`fgo_gnss_imu.m` passes `c_ini=clk` and `d_ini=dclk` directly into the graph
(`:40-61,102-106,180-218`).  Its `ClockFactor_CCDD.h` uses

```
(C2-C1) - (D1+D2)*dt/2              [metres]
[-1,+1,-dt/2,-dt/2]                 [metre Jacobian]
```

with the source clock sigma `0.1 m` from `parameters.m:130-133`.
The official `PseudorangeFactor_XC.h` and `DopplerFactor_VD.h` also use direct
clock and drift vector terms and unit Jacobians, rather than a C_LIGHT factor.

This is a source-aligned parity conversion, not sigma tuning.  At a one-second
edge, the current local C0/D row has

```
sigma_s = 0.1 / C_LIGHT
whitened C column = C_LIGHT / 0.1 = 2,997,924,580
whitened D column = (1 / 2) / 0.1 = 5
column ratio ~= 599,584,916
```

which matches the two Phase91 proxies to the recorded `dt` precision.  With C
and ISB in metres, the equivalent row has sigma `0.1 m`, C columns of `10`,
and D columns near `5` at one second.  This removes the artificial C_LIGHT
scale from the C0/D row while preserving the physical equation exactly after
the common metre conversion.  It does not by itself prove that the full graph
will converge; active-solve gates remain mandatory.

## Root-cause classification

| hypothesis | evidence-backed classification |
|---|---|
| Missing/weak integration test | A contributing process gap in earlier phases: factor tests did not require an accepted LM step. Phase86/91 added the active telemetry, finite-cost, `iterations >= 1`, and strict-decrease gates. It is not the numerical mechanism of the Phase91 zero-step. |
| Clock seconds versus metre conditioning | A leading mechanistic contributor supported by source parity and the measured C0/D column proxy of about `5.996e8`. The Phase91 D initializer left this mismatch untouched. It is the precise next implementation hypothesis, not a route-level proof of sole causality. |
| Inconsistent clock/drift initialization/handoff | Real but insufficient by itself: Phase91 supplied finite raw D on MTV-a/LAX-t while C remained seconds and both still accepted zero outer steps. The retained-key mismatch is a separate implementation alignment bug. |
| Raw-Doppler velocity observability or direct-WLS | Not implicated by the two Phase91 main-graph runs: direct-WLS and velocity-only routes are forbidden by the Phase91 contract, and those runs had valid same-run GNSS-first position/clock/velocity staging. |

The supported conclusion is: Phase91 exposed a retained-key alignment defect on
two routes and, on the two routes that reached the graph, exposed a no-progress
LM solve whose strongest actionable code-level contributor is the local
seconds/m/s adaptation against the official metre-valued C/ISB state.  No
accuracy or source-sigma conclusion follows.

## Phase92 implementation-only candidate

The separate Phase92 freeze authorizes exactly one opt-in candidate:

`phase92_source_meter_clock_state_parity_with_retained_raw_d_alignment`

planned selector: `--native-source-clock-c0d-meter-state-parity`.

The candidate is limited to the existing same-run native GNSS-first in-memory
raw GNSS -> GNSS-only FGO -> GNSS+IMU Pose3 batch path.  It must preserve direct
raw P+D quality and the existing C0/D row, edge/phone/clock-jump rules, source
sigma, optimizer, and no-coordinate-copy boundary.  It must not enable PDC,
direct-WLS, velocity-only, base, external/precomputed coordinate, truth, MAT,
Kaggle, or accuracy paths.

The one authorized implementation consists of:

1. Correct the Phase91 retained-key raw drift alignment exactly as specified
   above.  This is necessary source identity plumbing, not a relaxed count or
   coverage gate.
2. In the opt-in main GTSAM batch graph only, represent each receiver C_i and
   each global constellation ISB_i in metres.  Keep D_i in metres/second,
   signal IFB and residual-ionosphere states in metres, and all raw P/D
   measurements and physical sigmas unchanged.
3. Use the metre-equivalent official rows consistently:

   ```
   pseudorange: range + C_i + ISB_group + signal_bias + ionosphere - P
   C0/D:        (C_{i+1}-C_i) - (D_i+D_{i+1})*dt/2       [m]
   C0/D H:      [-1,+1,-dt/2,-dt/2]
   C0/D sigma:  0.1 m
   ```

   The metre row is the official seconds row multiplied by C_LIGHT; it is not
   a new equation or a changed sigma.  Adjust all clock priors, clock-motion
   rows, pseudorange/TDCP/carrier factor boundaries, residual diagnostics, and
   public output conversion consistently so no enabled batch path mixes C
   seconds with C/ISB metres.  Keep `PositionSolution.receiver_clock_bias` in
   its existing public seconds convention by dividing the final C_i by
   C_LIGHT exactly once.
4. Keep the selector default-off and leave the legacy GTSAM/Eigen behavior
   unchanged when it is absent.  Do not use magnitude-based unit guessing.

Required implementation tests must prove retained-key acceptance and strict
  rejection of missing/nonfinite/duplicate/reordered/misaligned keys; raw D
  finite full retained coverage with no fallback; metre C/ISB and m/s D factor
  equations/Jacobians/sigmas; all factor/prior/output conversions; default-off
  behavior; and no PDC/direct-WLS/velocity-only/external coordinate path.
  A later, separately sealed pre-raw manifest/evaluator must retain active
  telemetry completeness, finite costs, `iterations >= 1`, and strict
  `final_cost < initial_cost` gates.  This Phase92 audit/freeze authorizes no
  raw execution.

## Read and execution accounting

| item | count/setting |
|---|---:|
| sealed Phase91 result reads | 1 |
| sealed Phase91 freeze/manifest contract reads | 2 |
| official pinned source files read or hashed | 11 |
| installed GTSAM API header read or hashed | 1 |
| repository source/header files read or hashed | 10 |
| native solver invocations/reruns | 0 |
| raw Android GNSS reads | 0 |
| raw Android IMU reads | 0 |
| broadcast navigation reads | 0 |
| base RINEX reads | 0 |
| truth/MAT/Phase82/precomputed-coordinate reads | 0 |
| validation-holdout reads | 0 |
| Kaggle/token access | 0 |
| synthetic solver tests | 0 |
| accuracy scoring/route selection | false |

Repository evidence files:

* `apps/native/gnss_fgo_imu_no_base.cpp`
* `src/algorithms/fgo_problems.cpp`
* `src/algorithms/fgo.cpp`
* `src/algorithms/fgo_gtsam_backend.cpp`
* `src/algorithms/fgo_gtsam_internal.hpp`
* `include/libgnss++/algorithms/fgo.hpp`
* `include/libgnss++/algorithms/fgo_config.hpp`
* `include/libgnss++/algorithms/source_clock_c0d_initializer.hpp`
* `include/libgnss++/io/android_raw_gnss.hpp`
* `src/io/android_raw_gnss.cpp`

Official pinned evidence files:

* `output/reproducibility-cache/gsdc2023/fgo_gnss_imu.m`
* `output/reproducibility-cache/gsdc2023/functions/gnsslog2obs.m`
* `output/reproducibility-cache/gsdc2023/functions/GobsPhone.m`
* `output/reproducibility-cache/gsdc2023/parameters.m`
* `output/reproducibility-cache/gtsam_gnss/src/ClockFactor_CCDD.h`
* `output/reproducibility-cache/gtsam_gnss/src/PseudorangeFactor_XC.h`
* `output/reproducibility-cache/gtsam_gnss/src/DopplerFactor_VD.h`
* `output/reproducibility-cache/gtsam-4.3a/gtsam/nonlinear/LevenbergMarquardtOptimizer.cpp`
* `output/reproducibility-cache/gtsam-4.3a/gtsam/nonlinear/NonlinearOptimizer.cpp`
* `output/reproducibility-cache/gtsam-4.3a/gtsam/nonlinear/internal/LevenbergMarquardtState.h`
* `output/reproducibility-cache/gtsam-4.3a/gtsam/nonlinear/NonlinearOptimizerParams.h`

Installed API evidence:

* `/home/sasaki/.local/include/gtsam/navigation/CarrierPhaseFactor.h`

## SHA-256 evidence pins

| artifact | SHA-256 |
|---|---|
| sealed Phase91 result | `9929e285b58b5d9a65350ba1a3d497c736968bb896d72dbe5fb014534b6d58c8` |
| Phase91 implementation commit | `0b9c4ebc5704428421f0bfc1cad569d834690b6d` |
| Phase91 execution freeze | `456634b0a41da74de557185620bcb4d8c7b0572ebb7f3f1e415d8b8c82aa6fa4` |
| Phase91 execution manifest | `2078a4e2c3b963f07744c435ec303ea848bc372c8efd5003f6cb7b3d4b4ce988` |
| `apps/native/gnss_fgo_imu_no_base.cpp` | `825d3234a018cfac33e2e2b4afb1f4f2b00e1e339627f2eb995b7769ee125ad6` |
| `src/algorithms/fgo_problems.cpp` | `725167ccc21a62e61c4da9852726ed5cafbfc05ab794eaa76c4641676f576245` |
| `src/algorithms/fgo.cpp` | `4776896c345153d94700ad88f8651d9a30e1ee4bbb6c916a5c95df0053506f06` |
| `src/algorithms/fgo_gtsam_backend.cpp` | `66b575f5c05fb89ae611f3e7c8f01d4028d5d416023e16ea51d62d4819eb7a9a` |
| `src/algorithms/fgo_gtsam_internal.hpp` | `7221aaa53360a81aeea3f44ee293e7e1b08d30d368ac18cd7b839fbeb34c6beb` |
| `include/libgnss++/algorithms/source_clock_c0d_initializer.hpp` | `0431987c902c84909ddf34650f2ab306d3dc2c63f2afd288516ccadafcdbffba` |
| official `fgo_gnss_imu.m` | `c70090ccb8b27fc8ac7fd2929e2f995a14cfe7f089bb1fef067370e22051c3e3` |
| official `gnsslog2obs.m` | `665ce43d1fc3c5a9c3b3a6fd76f81539126e3ab897fbba484ab0ccd18964acff` |
| official `GobsPhone.m` | `11d43c1e76370b9d393075ec88071585efb3efb5d2fa97b3be58c0e1bd196986` |
| official `parameters.m` | `518925e9c75c7a14fceb5cd99432fe883311e0d64c72b94396744ac765120f52` |
| official `ClockFactor_CCDD.h` | `7c174217218aa0059155e1e5a7182c69d56a3cd389d0ebd95c242d6d8c65b9fc` |
| official `PseudorangeFactor_XC.h` | `7f467698f2239818724eba4485b830fbc543586bbee06a21b8b2c03ae5b56415` |
| official `DopplerFactor_VD.h` | `de2c11d06ff860a95785c84620e5ed57dcfd5ca24995a0e642e9a5984fe093e1` |
| pinned GTSAM `LevenbergMarquardtOptimizer.cpp` | `834f732ac4ad4130b2a719a3181e5dfcd2ecdf584ece6f0c76bdf5f390e10cbc` |
| pinned GTSAM `NonlinearOptimizer.cpp` | `594fd9bf3d48199f00d05794913df6c84d300664458d609fcdf0fb6ed3215a35` |
| pinned GTSAM `LevenbergMarquardtState.h` | `16964863621ca3aaadf6413c86adf202d566a4548a1a5a01a66b2aae286e7287` |
| pinned GTSAM `NonlinearOptimizerParams.h` | `0cf55b15a4d04aa1364b63bdae659fc65f568a8e08898a104a28d7fdc6ca07a5` |
| installed `CarrierPhaseFactor.h` | `938a5423662b779cded2cd646f6f0156223a6427a1a2e5d5491038209922b269` |
