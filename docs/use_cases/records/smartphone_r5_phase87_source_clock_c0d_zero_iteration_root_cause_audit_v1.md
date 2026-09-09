# Smartphone R5 Phase87: source ClockFactor C0/D zero-iteration root-cause audit

## Decision

Phase86 remains **NO-GO** for active-solve promotion.  The eight immutable
Phase85-v2 summaries are a converged-only diagnostic: every graph reports
`iterations=0`, finite `initial_cost`, and `final_cost == initial_cost`.

The evidence supports this classification:

1. **Immediate telemetry cause:** the GTSAM LM wrapper records “converged” as
   “`optimize()` returned without throwing”, not as a successful optimizer
   step.  GTSAM can return normally after a zero-progress first iteration with
   `iterations=0` and unchanged error.  Thus the Phase85 `converged=true` bit
   is not evidence that an active step was accepted.
2. **Mechanistic contributors:** the no-bridge Phase85 handoff initializes the
   C clock states from the GNSS-first clock seed but initializes every C0/D
   drift state to zero; and the local adaptation mixes clock seconds with
   drift metres/second in a seconds residual.  At one-second edges this gives
   a C-versus-D Jacobian-column ratio of about `5.99584916e8` (normal-diagonal
   ratio about `3.5950207149472704e17`).  This is a severe conditioning risk
   when the initial C/D relation is inconsistent.
3. **Test/evaluator gap:** the existing C0/D tests prove the equation,
   Jacobians, units, edge gates, and graph row accounting, but do not require
   a successful LM iteration or a strict cost decrease.  Phase86 correctly
   added those active-solve gates and rejected the old result.

The exact internal LM stop branch (indeterminate linear solve, unsuccessful
model step/lambda exhaustion, or the small-cost-change stop) is not observable
in the sealed Phase85 summaries.  The audit therefore does not claim one of
those branches as route-specific fact.  A diagnostic-only implementation is
authorized by the separate Phase88 freeze to expose that branch on a future
raw-only run.

No source sigma tuning, accuracy claim, truth comparison, MAT/Kaggle access, or
native/raw route rerun was performed for this audit.

## Sealed Phase85/86 evidence

The Phase85-v2 aggregate is the only route-result artifact used here.  Its
eight candidate summaries have the following graph telemetry; repeats are
byte-identical in the Phase86 result:

| route | run | graph factors | values | C0/D factors | iterations | initial cost | final cost |
|---|---:|---:|---:|---:|---:|---:|---:|
| `2021-03-16-18-59-us-ca-mtv-a/pixel5` | 1 | 143414 | 10799 | 2158 | 0 | 181489255.2266004 | 181489255.2266004 |
| `2021-03-16-18-59-us-ca-mtv-a/pixel5` | 2 | 143414 | 10799 | 2158 | 0 | 181489255.2266004 | 181489255.2266004 |
| `2021-08-24-20-32-us-ca-mtv-h/pixel5` | 1 | 213184 | 15703 | 3139 | 0 | 299479705.9119013 | 299479705.9119013 |
| `2021-08-24-20-32-us-ca-mtv-h/pixel5` | 2 | 213184 | 15703 | 3139 | 0 | 299479705.9119013 | 299479705.9119013 |
| `2022-04-01-18-22-us-ca-lax-t/pixel5` | 1 | 88025 | 7333 | 1465 | 0 | 192965599.96515816 | 192965599.96515816 |
| `2022-04-01-18-22-us-ca-lax-t/pixel5` | 2 | 88025 | 7333 | 1465 | 0 | 192965599.96515816 | 192965599.96515816 |
| `2023-03-08-21-34-us-ca-mtv-u/pixel5` | 1 | 63471 | 5513 | 1101 | 0 | 62260446.071378246 | 62260446.071378246 |
| `2023-03-08-21-34-us-ca-mtv-u/pixel5` | 2 | 63471 | 5513 | 1101 | 0 | 62260446.071378246 | 62260446.071378246 |

The Phase86 result reports these failed aggregate gates:

- `sealed_phase85_v2_artifact_hashes_exact` (the historical freeze pin is
  one character short: it ends `...cc4a`, while the immutable aggregate on
  disk is the 64-character SHA ending `...cc4a6`);
- `graph_iterations_at_least_one` for all eight runs;
- `final_cost_strictly_less_than_initial_cost` for all eight runs; and
- `all_gates_anded` as the derived fail-closed gate.

The malformed historical hash is an artifact-integrity failure, not a claim
about the optimizer.  The two active-solve failures are independent of it.

## GTSAM termination semantics

The official pinned GTSAM sources establish the meaning of the telemetry:

- `LevenbergMarquardtState` starts with `iterations=0`; `increaseLambda()`
  increments only `totalNumberInnerIterations`, while `decreaseLambda()` (the
  accepted-step path) creates the next state with `iterations + 1`
  (`output/reproducibility-cache/gtsam-4.3a/gtsam/nonlinear/internal/LevenbergMarquardtState.h:42-58,68-94`).
- `tryLambda()` only calls `decreaseLambda()` after a successful damped step
  with adequate model fidelity.  An indeterminate solve, no model decrease,
  or a small nonlinear cost change leaves the outer iteration counter at zero;
  repeated unsuccessful trials can terminate at the lambda upper bound
  (`output/reproducibility-cache/gtsam-4.3a/gtsam/nonlinear/LevenbergMarquardtOptimizer.cpp:144-221,243-269`).
- `defaultOptimize()` performs one `iterate()` and then checks convergence.
  With unchanged error, `absoluteDecrease=0` and `relativeDecrease=0`, which
  satisfy the configured positive tolerances even though no accepted step was
  made (`output/reproducibility-cache/gtsam-4.3a/gtsam/nonlinear/NonlinearOptimizer.cpp:62-109,182-230`).
- The local backend sets `maxIterations`, absolute/relative tolerances, and
  initial/final costs, then sets `solved=true` unless `optimize()` throws.  It
  assigns that boolean directly to `diagnostics.converged`; it does not check
  `optimizer.iterations()` or cost decrease
  (`src/algorithms/fgo_gtsam_backend.cpp:1157-1186`).

A synthetic installed-GTSAM factor with a finite constant residual and a
zero Jacobian reproduced the exact semantic trap without opening any route
data: GTSAM printed `converged`, then reported
`initial=5000 final=5000 iterations=0`.  The local Eigen backend already guards
against this interpretation by treating `iter==0` as not converged
(`src/algorithms/fgo.cpp:1318-1337`), highlighting that the gap is in the
GTSAM wrapper/evaluator contract rather than the C0/D equation unit test.

## C0/D units, equation, and conditioning

The official source uses a meter-valued clock state and meter/second drift:

- `fgo_gnss_imu.m` initializes `c` and `d` independently and inserts both
  into every epoch (`output/reproducibility-cache/gsdc2023/fgo_gnss_imu.m:102-106,169-186`).
- Its pseudorange factor adds `c` to a metre-valued range residual and its
  Doppler factor adds `d` to a metre/second-valued rate residual
  (`output/reproducibility-cache/gtsam_gnss/src/PseudorangeFactor_XC.h:48-64`,
  `output/reproducibility-cache/gtsam_gnss/src/DopplerFactor_VD.h:45-54`).
- The source C0/D row is
  `c2-c1-(d1+d2)*dt/2`, with Jacobian
  `[-1,+1,-dt/2,-dt/2]`
  (`output/reproducibility-cache/gtsam_gnss/src/ClockFactor_CCDD.h:39-63`)
  and `sigma_motion_clk=0.1` in `parameters.m:130-133`.

The local graph deliberately stores `c` in seconds and `d` in metres/second,
so it adapts the same physical row as
`(c2-c1)-(d1+d2)*dt/(2*C_LIGHT)` with `sigma=0.1/C_LIGHT`
(`src/algorithms/fgo_gtsam_internal.hpp:692-697,756-760`; factor insertion at
`src/algorithms/fgo_gtsam_backend.cpp:976-983`).  That conversion is
dimensionally coherent, but it is badly scaled against the metre-valued
state blocks.  For `C_LIGHT=299792458`:

| `dt` | absolute drift Jacobian | clock/drift column ratio | normal-diagonal ratio |
|---:|---:|---:|---:|
| 1.0 s | `1.6678204759907602e-09` | `599584916` | `3.5950207149472704e17` |
| 1.5 s | `2.5017307139861402e-09` | `399723277.33333337` | `1.5977869844210096e17` |

The converted scalar noise is `3.3356409519815207e-10` seconds.  The official
meter row at `dt=1` has drift columns `-0.5` and therefore does not introduce
this cross-block scale ratio.  This is evidence for a conditioning contributor,
not permission to change the frozen sigma or silently change state units.

## Initialization and handoff

The Phase85 command used the no-bridge C0/D recipe.  In the backend:

- C states are inserted from the optional PDC seed when present, otherwise
  from the problem clock-bias metre value and divided by `C_LIGHT`
  (`src/algorithms/fgo_gtsam_backend.cpp:464-476`).
- D states are inserted from an optional PDC clock-rate seed, otherwise zero
  (`src/algorithms/fgo_gtsam_backend.cpp:261-274`).  With
  `native_pdc_state_bridge=false`, the Phase85 recipe provides no PDC seed, so
  every initial D state is zero.
- The raw parser does carry Android clock drift in metres/second
  (`src/io/android_raw_gnss.cpp:573-592,783-800`), and the problem builder
  carries it into `EpochSeed` (`src/algorithms/fgo_problems.cpp:364-391`), but
  that value is not used as the C0/D state initializer in this no-bridge path.
- Phase85 did not select the velocity-only handoff.  The historical handoff
  copies GNSS-first positions and receiver clocks into the following in-memory
  problem, converting the GNSS-first clock back to the internal metre marker
  (`apps/native/gnss_fgo_imu_no_base.cpp:4251-4270`).  The separate velocity-only
  branch explicitly leaves raw SPP position/clock seeds untouched
  (`apps/native/gnss_fgo_imu_no_base.cpp:4246-4255`).

Therefore the observed recipe starts with a GNSS-first C trajectory but a
zero D trajectory in the no-bridge branch.  A synthetic 30-metre adjacent C
  mismatch with zero D would be a 300-sigma C0/D residual under the frozen
`0.1 m` source noise; this illustrates the scale of the initialization
problem, not a claim about any uninspected route residual.

## What the existing tests do and do not establish

`tests/test_fgo_gtsam_backend.cpp:4235-4271` verifies the source residual,
Jacobian conversion, and seconds noise; `:4273-4320` verifies edge/configuration
gates; and `:4322` onward verifies synthetic batch row accounting.  None of
these tests asserts that GTSAM accepts an outer LM step, that `iterations>=1`,
or that `final_cost < initial_cost`.  The old Phase85 evaluator consequently
treated the wrapper’s no-exception `converged=true` as sufficient.  Phase86’s
active-solve reclassification fixed the evaluation contract, but did not and
should not rewrite the sealed Phase85 artifact.

## Root-cause verdict and next authorized action

The precise supported verdict is **GTSAM no-progress telemetry masked by a
weak convergence contract, with C/D handoff inconsistency and seconds/meter
conditioning as the leading mechanistic contributors**.  The sealed artifacts
cannot distinguish the individual internal LM stop branch, so that branch must
be exposed before selecting a corrective implementation.

Phase88 therefore freezes exactly one source-aligned raw-only candidate:
`phase88_source_clock_c0d_active_solve_observability_conditioning_diagnostic`.
It may add active-solve telemetry only (accepted outer iterations, total inner
lambda attempts, maximum lambda, linear-solve/indeterminate status, initial
and final costs, and C/D whitened Jacobian-column norms/conditioning proxy).
It must preserve the source C0/D equation, current sigma, raw P+D quality
contract, and measurement factors.  It must require the velocity-only
GNSS-first handoff, keep the PDC bridge disabled, forbid coordinate copying or
precomputed coordinates, and make no accuracy/Kaggle/truth decision.  Meter-
valued state parity and sigma tuning are not part of this one candidate; they
require a later evidence-based freeze.

## Exact read and execution accounting

| item | count/setting |
|---|---:|
| Phase86 freeze logical reads | 1 |
| Phase86 sealed result logical reads | 1 |
| Phase85-v2 aggregate logical reads | 1 |
| Phase85 candidate summary/submission reads by this audit | 0 |
| repository source/header/test files read or hashed | 11 |
| official pinned source/GTSAM files read or hashed | 9 |
| synthetic tests | 2 (installed-GTSAM semantic reproduction; numeric conditioning calculation) |
| native solver invocations/reruns | 0 |
| raw Android GNSS/IMU reads | 0 |
| broadcast navigation reads | 0 |
| base RINEX reads | 0 |
| truth/MAT/Phase82 coordinate/precomputed-coordinate reads | 0 |
| validation holdout reads | 0 |
| Kaggle/token access | 0 |
| accuracy scoring or route selection | false |

The repository files used for source evidence were:
`apps/native/gnss_fgo_imu_no_base.cpp`,
`include/libgnss++/algorithms/fgo.hpp`,
`include/libgnss++/algorithms/fgo_config.hpp`,
`src/algorithms/fgo.cpp`, `src/algorithms/fgo_gtsam_backend.cpp`,
`src/algorithms/fgo_gtsam_internal.hpp`, `src/algorithms/fgo_problems.cpp`,
`src/io/android_raw_gnss.cpp`, `include/libgnss++/core/observation.hpp`,
`include/libgnss++/core/solution.hpp`, and
`tests/test_fgo_gtsam_backend.cpp`.

The official pinned files used were:
`output/reproducibility-cache/gsdc2023/fgo_gnss_imu.m`,
`output/reproducibility-cache/gsdc2023/parameters.m`,
`output/reproducibility-cache/gtsam_gnss/src/ClockFactor_CCDD.h`,
`output/reproducibility-cache/gtsam_gnss/src/PseudorangeFactor_XC.h`,
`output/reproducibility-cache/gtsam_gnss/src/DopplerFactor_VD.h`,
`output/reproducibility-cache/gtsam-4.3a/gtsam/nonlinear/LevenbergMarquardtOptimizer.cpp`,
`output/reproducibility-cache/gtsam-4.3a/gtsam/nonlinear/NonlinearOptimizer.cpp`,
`output/reproducibility-cache/gtsam-4.3a/gtsam/nonlinear/NonlinearOptimizerParams.h`,
and `output/reproducibility-cache/gtsam-4.3a/gtsam/nonlinear/internal/LevenbergMarquardtState.h`.

## SHA-256 evidence pins

| artifact | SHA-256 |
|---|---|
| Phase86 freeze | `9b13d6a5118c734521e69d350c9478f0844c5bbf9047c5e057bf4f5a427ace46` |
| Phase86 sealed result | `603b3fe3cf6cecd751f9240673a335ee884b711bc923f08acc3ab0fcb7134503` |
| Phase85-v2 aggregate (actual) | `2ac9919e57bb18e52ccb81bc73471af017af77fe3dad65d5750f919d5b1cc4a6` |
| Phase85-v2 output manifest | `5bb7bdaeb0f1fb2c00d7fe1354cd0cda1c289b2b664840b4b39135380574c53c` |
| `src/algorithms/fgo_gtsam_backend.cpp` | `4075156d10189f7f16df8e3f570106c73e0635fa2a99ef084f34295fda88afbd` |
| `src/algorithms/fgo_gtsam_internal.hpp` | `90db67fe9dd8d032c3d28d37de6c161966eb3cd5ca8fb873d98a47fa6181859b` |
| `apps/native/gnss_fgo_imu_no_base.cpp` | `89688fc5ce16402c66a15c6f373185fbcc3e4a1b1c6d9b7e7b691bf95b051189` |
| official `fgo_gnss_imu.m` | `c70090ccb8b27fc8ac7fd2929e2f995a14cfe7f089bb1fef067370e22051c3e3` |
| official `parameters.m` | `518925e9c75c7a14fceb5cd99432fe883311e0d64c72b94396744ac765120f52` |
| official `ClockFactor_CCDD.h` | `7c174217218aa0059155e1e5a7182c69d56a3cd389d0ebd95c242d6d8c65b9fc` |
| official GTSAM `LevenbergMarquardtOptimizer.cpp` | `834f732ac4ad4130b2a719a3181e5dfcd2ecdf584ece6f0c76bdf5f390e10cbc` |
| official GTSAM `NonlinearOptimizer.cpp` | `594fd9bf3d48199f00d05794913df6c84d300664458d609fcdf0fb6ed3215a35` |
| official GTSAM `LevenbergMarquardtState.h` | `16964863621ca3aaadf6413c86adf202d566a4548a1a5a01a66b2aae286e7287` |
