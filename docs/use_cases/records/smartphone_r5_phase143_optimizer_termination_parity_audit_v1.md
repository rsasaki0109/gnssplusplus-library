# Phase143 optimizer termination parity audit

Execution label: Luna Max  
Phase: 143  
Scope: read-only optimizer/source parity audit and one candidate design. No
raw phone GNSS/IMU, navigation, base, truth, solution-coordinate, solver,
MAT/PDC/precomputed-coordinate, accuracy, or Kaggle payload was read or
executed. Phase142 accuracy was not reopened or re-evaluated.

## Decision

The official MATLAB GNSS and GNSS+IMU entry points both construct
gtsam.LevenbergMarquardtParams, set base verbosity to TERMINATION, and
explicitly set maxIterations to 1000. The native Phase142 IMU main graph
sets config.max_iterations to 12, while its separate GNSS-first initializer
sets gnss_first_config.max_iterations to 1000. The native main configuration
has no --max-iterations override in the IMU application; the value is
assigned at the graph entry point.

Both inspected sealed structural routes independently show main
accepted_iterations: 12 and terminal_branch: maximum_outer_iterations. The
same hard-cap branch is present in the Phase112 structural record. This is
direct evidence that the current 12 is a native main-stage experiment cap,
not an observed convergence tolerance. GNSS-first accepts substantially more
iterations in the same records, so the cap is not a process-wide or solver
failure limit.

Exactly one source-backed candidate is therefore frozen for a future
implementation: an opt-in selector that changes only the Phase142 main LM
iteration bound from 12 to the official value 1000. It does not change the
GNSS-first stage (already 1000), tolerances, lambda policy, ordering, QR
branch, graph, factors, values, noises, filters, initialization, raw-base
path, C7/D state, TDCP, IMU, Pixel5 offset, output, fallback, or legacy
defaults. This audit does not authorize implementation, raw input, solver,
truth, or accuracy execution.

## Authoritative source evidence

| Question | Evidence | Finding |
| --- | --- | --- |
| Official GNSS optimizer | output/reproducibility-cache/gsdc2023/fgo_gnss.m:203-207 | Constructs LM params, calls setVerbosity(TERMINATION), then setMaxIterations(1000). |
| Official GNSS+IMU optimizer | output/reproducibility-cache/gsdc2023/fgo_gnss_imu.m:325-329 | Same construction and explicit maxIterations 1000. |
| Official LM values not explicitly set in MATLAB | Both official source files contain no relative/absolute tolerance, lambda, diagonal damping, linear solver, or ordering setter | Those values are inherited from the linked GTSAM constructor/defaults. |
| Official/default tolerance, ordering, solver | gtsam/nonlinear/NonlinearOptimizerParams.h:42-47,97-108 | Relative 1e-5, absolute 1e-5, total error 0, COLAMD, and MULTIFRONTAL_CHOLESKY; absent ordering means COLAMD. |
| Official/default LM policy | gtsam/nonlinear/LevenbergMarquardtParams.h:49-81 | Legacy lambda initial 1e-5, factor 10, lower 0, upper 1e5, model-fidelity floor 1e-3, diagonal damping false, fixed lambda factor true. |
| Native main bound | apps/native/gnss_fgo_imu_no_base.cpp:9154-9158 | Phase142 IMU/Pose3 main config explicitly sets max_iterations to 12. |
| Native GNSS-first bound | apps/native/gnss_fgo_imu_no_base.cpp:9968-9972 | Separate Point3/velocity GNSS-first config explicitly sets max_iterations to 1000 and identifies the upstream bound. |
| Native main tolerances | src/algorithms/fgo_gtsam_backend.cpp:2207-2217 | Max comes from config; zero config thresholds map to absolute 1e-10 and relative 1e-8. No lambda, diagonal, or ordering mutation occurs here. |
| Native Phase99 solver | src/algorithms/fgo_gtsam_backend.cpp:91-114,2218-2226; src/algorithms/fgo_gtsam_internal.hpp:120-129 | Phase99 selects MULTIFRONTAL_QR/EliminateQR only for the meter-state main graph; GNSS-first and legacy remain multifrontal Cholesky. |
| Native diagnostic verbosity | src/algorithms/fgo_gtsam_backend.cpp:2207-2210; src/algorithms/fgo_gtsam_internal.hpp:435-445 | Normal native LM is silent; an existing environment/diagnostic path may request SUMMARY/TRYLAMBDA and changes verbosity only. |
| Iteration-cap semantics | gtsam/nonlinear/NonlinearOptimizer.cpp:62-117 | The loop continues only while iterations is below maxIterations and convergence is false; maximum iterations is a distinct exit. |
| Convergence semantics | gtsam/nonlinear/NonlinearOptimizer.cpp:182-230 | Relative/absolute decrease and total error are checked after an iteration. |
| Native termination classification | src/algorithms/fgo_gtsam_internal.hpp:467-579 | The authoritative wrapper labels a solve maximum_outer_iterations when accepted iterations reach optimizer.params.maxIterations; tolerance, small-cost, maximum-lambda, exception, and no-progress branches are separate. |

The official setVerbosity(TERMINATION) is the base
NonlinearOptimizerParams verbosity setter. The official MATLAB code does not
call setVerbosityLM, so it does not establish a different lambda-trial
policy. The native Phase142 path deliberately preserves its local tolerances
(1e-8/1e-10) and QR selector; parity evidence supports changing only the
explicitly documented iteration bound in this phase.

## Sealed structural evidence

The current structural evidence is read from sealed metadata, not solution
rows:

| Record | Route | GNSS-first | Main | Interpretation |
| --- | --- | ---: | ---: | --- |
| Phase141 structural result, commit 9b2427bb03ca14df5bf3c3668d46d3638b03388a, JSON SHA-256 33d67248b9423a841ebb29b6373eed042d4da6b15924df710dcc37f18b73ff13 | MTV-A | 111 accepted; finite strict decrease | 12 accepted; finite strict decrease; maximum_outer_iterations | Main stopped at the configured cap. |
| Same sealed result | LAX-T | 116 accepted; finite strict decrease | 12 accepted; finite strict decrease; maximum_outer_iterations | Same cap behavior on an independent route. |
| Phase112 structural result, commit dfa9c3a186c9d8633dc572192b513b584f18800a, JSON SHA-256 087b1bf51b4e584b86a7b558560df4ceb78e19b01e572f2d60aeef4c3d759ba0 | MTV-A | 218 accepted; terminal trace complete | 12 accepted; maximum_outer_iterations; 20 inner lambda attempts | The 12-cap predates Phase143 and is main-stage specific. |
| Same Phase112 sealed result | LAX-T | 234 accepted; terminal trace complete | 12 accepted; maximum_outer_iterations; 20 inner lambda attempts | Not a route-specific early stop. |

The normalized Phase141 record is authoritative for this comparison because
its native active-solve report is already sealed. Its main entries also show
MULTIFRONTAL_QR/EliminateQR, finite initial/final costs, and no fallback. No
coordinates or accuracy values are interpreted here. The current audit HEAD
is 53e90683df43e76f6c4757d9d114f943cdfac326; the Phase142 truth-only result
commit is not reopened.

## Effective parameter comparison

| Parameter | Official MATLAB call plus local GTSAM defaults | Native Phase142 main | Phase143 candidate when on | Phase143 invariant |
| --- | --- | --- | --- | --- |
| maxIterations | Explicit 1000 | 12 | 1000 | Main only; GNSS-first remains 1000. |
| Relative error tolerance | Inherited 1e-5 | Explicit fallback 1e-8 | 1e-8 | Unchanged; no official-tolerance substitution. |
| Absolute error tolerance | Inherited 1e-5 | Explicit fallback 1e-10 | 1e-10 | Unchanged. |
| Total error tolerance | Default 0 | Default 0 | 0 | Unchanged. |
| Lambda initial/factor/lower/upper | 1e-5/10/0/1e5 | Same constructor defaults | Same | No lambda tuning or schedule change. |
| Model fidelity / damping | 1e-3 / diagonal damping false | Same | Same | No damping change. |
| Linear solver | Unspecified in MATLAB, local default Cholesky | Phase99 main MULTIFRONTAL_QR | QR | QR is a frozen Phase99 setting, not a Phase143 change. |
| Elimination/order | Default EliminatePreferCholesky and COLAMD | EliminateQR, COLAMD | EliminateQR, COLAMD | No ordering change. |
| Verbosity | Base TERMINATION; LM-specific default silent | Silent unless existing diagnostic/env trace | Existing value | Termination telemetry is structured, not a verbosity/algorithm change. |

The table separates what the official entry point explicitly requests from
what the native Phase142 recipe already freezes. The candidate ports only one
scalar request: the official main-stage iteration budget.

## Frozen candidate boundary

Candidate ID: phase143-official-main-lm-termination-budget-v1  
Proposed selector: --native-phase143-official-main-lm-termination-budget  
Default: off  
Scope: Phase142/Phase99 meter-state Pose3+IMU main batch LM call only

When the selector is on, the native config passed to the existing GTSAM LM
backend must have exactly maxIterations == 1000 for that main call. The
GNSS-first call remains exactly as currently configured (1000). When it is
off, the existing main value remains exactly 12. The selector must be
rejected if it is requested outside the Phase142 meter-state/Phase99 recipe,
if another experiment selector changes the graph or optimizer, or if the
effective parameter report cannot prove the exact value.

No partial mode is allowed. The following are fixed and must compare equal to
the selector-off Phase142 recipe: graph/factor and Values construction, initial
state and handoff, C7/D/CCDD, QR/EliminateQR, COLAMD ordering, relative and
absolute tolerances, total error tolerance, lambda values and update policy,
diagonal damping, noises/sigmas, filters, raw-base correction, TDCP/IMU
behavior, Pixel5 offset, output mapping, fallback policy, and all other
selectors. There is no retry, sweep, tolerance change, score-based choice, or
alternative solver.

The future native diagnostic must publish one authoritative, versioned
termination object for each stage, with no solution values or coordinate rows.
Required fields are:

* selector state and effective max_iterations;
* attempted, accepted outer iterations, and total inner lambda attempts;
* finite initial/final cost and strict-progress result;
* exact termination branch (maximum_outer_iterations,
  outer_convergence_tolerance, small_cost_change, maximum_lambda,
  no_inner_iteration, exception, or no_progress_unclassified);
* effective relative/absolute/total tolerances;
* initial/final/maximum lambda, lambda factor and bounds, model-fidelity
  threshold, and diagonal-damping state;
* linear solver, elimination function, ordering type/presence, and existing
  stage identity; and
* no-fallback and telemetry-complete markers sourced from the native active
  solve report.

Missing, duplicate, renamed, inferred, non-finite, or conflicting fields are
fail-closed. In particular, accepted_iterations may not be reconstructed from
a generic iteration field or a process return code. A future structural
validator must require the main branch to be either a native tolerance/LM
termination branch under the exact 1000 bound or an explicitly reported
exception/failure; it must never relabel a cap as convergence.

## Risks and next boundary

The candidate is source-backed but not an accuracy claim. Raising a hard cap
can increase runtime and can change the final native state; it may still stop
at a tolerance or lambda branch before 1000. The sealed Phase112/Phase141
records justify the cap diagnosis but do not predict the candidate result.
Qualification must therefore be launch-free first, followed by a new
independent structural authorization and one-shot raw execution if approved.
Truth/accuracy requires a separate later authorization. No historical result
may be rerun or repaired.

## Source and artifact hashes

These hashes bind the read-only evidence used by this audit:

* apps/native/gnss_fgo_imu_no_base.cpp — 912b8f6cfca175bf5a91ab63d1d4803f33a1acbc89438c785b42fec469957181
* src/algorithms/fgo_gtsam_backend.cpp — fdf1e9b19300c52ed50f363e96f3b9df5515c82b8a1e23a1718cc978626a23
* src/algorithms/fgo_gtsam_internal.hpp — fc328bf55099841b6478df9e0f82af72eb9e0f9365c04e86eb618ae9d91cd8a6
* include/libgnss++/algorithms/fgo.hpp — da16e074a70bd5a813146ff0a32ee3b8706856807f828af2b1806b474de0c85f
* output/reproducibility-cache/gsdc2023/fgo_gnss.m — 5368e4056f70f448b728792c5e4c124b7f2afc1e00653492b807083bd3ccf0c3
* output/reproducibility-cache/gsdc2023/fgo_gnss_imu.m — c70090ccb8b27fc8ac7fd2929e2f995a14cfe7f089bb1fef067370e22051c3e3
* output/reproducibility-cache/gtsam-4.3a/gtsam/nonlinear/LevenbergMarquardtParams.h — dd9348c08a96f0dffb2cf6b6f24eab18d4d32280a21e333a1e38725b254ec3f2
* output/reproducibility-cache/gtsam-4.3a/gtsam/nonlinear/NonlinearOptimizerParams.h — 0cf55b15a4d04aa1364b63bdae659fc65f568a8e08898a104a28d7fdc6ca07a5
* output/reproducibility-cache/gtsam-4.3a/gtsam/nonlinear/NonlinearOptimizer.cpp — 594fd9bf3d48199f00d05794913df6c84d300664458d609fcdf0fb6ed3215a35
* output/reproducibility-cache/gtsam-4.3a/gtsam/nonlinear/LevenbergMarquardtOptimizer.cpp — 834f732ac4ad4130b2a719a3181e5dfcd2ecdf584ece6f0c76bdf5f390e10cbc

The audit file is intentionally committed before the machine-readable freeze
so the freeze can pin its full commit and SHA-256.
