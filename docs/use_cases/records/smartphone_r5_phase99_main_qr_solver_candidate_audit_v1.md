# Phase99 main meter-state QR solver candidate audit

- Execution label: `Luna Max`
- Audit date: `2026-09-03`
- Status: `sealed-read-only-audit-qualified-for-one-candidate-freeze`
- Scope: the sealed Phase80/98 structural records for `2021-03-16-18-59-us-ca-mtv-a/pixel5` (MTV-A) and `2022-04-01-18-22-us-ca-lax-t/pixel5` (LAX-T).
- Question: whether the Phase93/98 opt-in main meter-state graph has a sufficiently supported solver-path hypothesis for one future `MULTIFRONTAL_QR` candidate, while leaving the GNSS-first graph and the legacy default unchanged.

This is a source and sealed-artifact audit. It did not read raw device files,
broadcast navigation data, truth, MAT binaries, base data, or precomputed
coordinates; it did not invoke the native solver, rerun a route, calculate
accuracy, or publish a solution. The QR candidate is frozen for a later
implementation/qualification decision, not declared successful here.

## Decision

The MTV-A/LAX-T structural evidence gate is **qualified for exactly one
solver-path candidate freeze**, but it is not a solution or root-cause proof.
Both routes have the same earliest observed main-stage boundary: the first
damped linear solve under the existing multifrontal Cholesky path. In the
same sealed runs, GNSS-first meter C0/D optimization made finite strict
progress and exported a complete exact-key D sequence. The Phase80 comparator
without the Phase98 CCDD meter-state recipe accepted 12 main iterations with
strict cost decrease on both route identities.

The candidate is:

`phase99-main-meter-state-multifrontal-qr-branch-only`

For a future implementation, this candidate changes only the main
meter-state optimizer's effective linear solver enum from
`MULTIFRONTAL_CHOLESKY` to `MULTIFRONTAL_QR`. It preserves multifrontal mode,
the implicit COLAMD ordering, graph/factors/Values, equation and units,
factor sigmas, filtering, initialization, C/D handoff, LM damping and trial
policy, and fail-closed behavior. It does not change GNSS-first. It is not
implemented, and this audit authorizes no raw or solver execution.

The evidence does **not** establish that CCDD alone caused a rank defect. The
Phase80 and Phase98 graphs are not an append-only factor change: the legacy
clock row was replaced by CCDD, C/ISB units changed, signal-bias states were
removed, and retained observation populations differ. QR is therefore a
bounded numerical hypothesis for the observed factorization boundary, not a
proven correction.

## Read accounting

| Activity | Count |
|---|---:|
| Native solver invocations | `0` |
| Raw `device_gnss.csv` reads | `0` |
| Raw `device_imu.csv` reads | `0` |
| Broadcast navigation reads | `0` |
| Truth reads | `0` |
| MAT binaries read/generated | `0` |
| Base RINEX reads | `0` |
| Precomputed-coordinate reads | `0` |
| Kaggle/token access | `0` |
| Accuracy calculations/submission | `0` |
| Route reruns/fallbacks | `0` |
| New large artifacts | `0` |

Only committed source/record files, the two small sealed Phase80 summaries,
the Phase98 aggregate structural result, and the two official `.m` source
files were inspected. No `.mat` payload was opened.

## Authorities

| Item | Pinned authority |
|---|---|
| Current source/result tip | Phase98 result commit `1e843beb5dedf94a6339b46b163800c387f00637` |
| Phase98 implementation | commit `b61e6db05364bebd63508e93506168c097236a0e`; native source SHA-256 `eb50ebe8bc084e2eceba81cab72b42390cc3ebbbdfd1a4e483d3c4635a1a4afc` for `apps/native/gnss_fgo_imu_no_base.cpp`, `aa0e1b35803b24f9d19924e73745427f0639911fc5872248bbc8afb1f9971f9f` for `src/algorithms/fgo_gtsam_backend.cpp`, and `65d13487e5e89835b92f48cb9f9ef30e184b58ebd84b03176ee3384ff017b95f` for `src/algorithms/fgo_gtsam_internal.hpp` |
| Phase98 freeze | commit `3dbfff6b87fc4434fec39baebd1f78b047804dd4`; freeze SHA-256 `971d1d67d976b74cb89efbc32fd04504b26e07b386a3d7923df358dd5c9c8591` |
| Phase98 manifest | `docs/use_cases/records/smartphone_r5_phase98_solver_rank_diagnostic_execution_manifest_v1.json`; SHA-256 `4d12781e1642ae1e3bbfb69975c6dcf24b3376c0b908ea0837e7aaffc3a25289` |
| Phase98 sealed structural result | `docs/use_cases/records/smartphone_r5_phase98_solver_rank_diagnostic_structural_result_v1.json`; SHA-256 `8948a006aa4d02b40418c9eb1bc78ad711c9432154eba6ad69c70359e6b5b7be`; result commit `1e843beb5dedf94a6339b46b163800c387f00637` |
| Phase98 prior audit | commit `8517380eb205a6fb30bfb88bccf7de3dc6f87847`; audit SHA-256 `f0c970a8ddc0f0a9ec5a9ab4fd8ea80d91eff9e37db91599e9336ffd7d7f8919` |
| Phase80 MTV-A summary | `output/smartphone-r5/phase80-source-exact-direct-observable-quality-structural-v2/2021-03-16-18-59-us-ca-mtv-a/pixel5/candidate/run1/summary.json`; SHA-256 `ff0e1be40bd12f5fabd9fa8e0571e61049e1e23c34c347618e35f9b68a5f5181` |
| Phase80 LAX-T summary | `output/smartphone-r5/phase80-source-exact-direct-observable-quality-structural-v2/2022-04-01-18-22-us-ca-lax-t/pixel5/candidate/run1/summary.json`; SHA-256 `599a399debe0800ed4fc794b3b39ffb31e384b233a3fe08338a60f4176b21a9a` |
| Phase80 source/freeze | source commit `f28aebaf1b41df3ee5e363a8ad94987247321048`; freeze SHA-256 `9ebdde21fe7fd955029243d3bdbba3a9f9d3832388d67480760394177c8f685e`; manifest SHA-256 `6d20541560b726ed2409cd05902e5105874ac72e113af0c3812f029130df310f` |
| Official GNSS source | `output/reproducibility-cache/gsdc2023/fgo_gnss.m`, SHA-256 `5368e4056f70f448b728792c5e4c124b7f2afc1e00653492b807083bd3ccf0c3` |
| Official GNSS/IMU source | `output/reproducibility-cache/gsdc2023/fgo_gnss_imu.m`, SHA-256 `c70090ccb8b27fc8ac7fd2929e2f995a14cfe7f089bb1fef067370e22051c3e3` |
| Official CCDD factor | `output/reproducibility-cache/gtsam_gnss/src/ClockFactor_CCDD.h`, SHA-256 `7c174217218aa0059155e1e5a7182c69d56a3cd389d0ebd95c242d6d8c65b9fc` |

## MTV-A/LAX-T structural gate

The table compares sealed structural telemetry only. Costs are optimizer
costs, not accuracy or truth scores.

| Route | Phase80 no-C0D main | Phase98 meter C0/D GNSS-first | Phase98 meter C0/D main |
|---|---|---|---|
| MTV-A | `143414` factors, `10799` Values; `12` accepted iterations; cost `155976340.16237149 -> 28128.450524519692` | `167` accepted iterations; cost `5618000.786274777 -> 21298.114523521894`; `2158` C0D rows; D `2159/2159`, finite and exact-key | `120093` factors, `10798` Values; `0` accepted; cost unchanged at `79358354.39651252`; ten trials, `1e-5 -> 1e5` |
| LAX-T | `88025` factors, `7333` Values; `12` accepted iterations; cost `190528071.24200463 -> 22206.857308543094` | `262` accepted iterations; cost `6419784.856579111 -> 12818.288538454106`; `1465` C0D rows; D `1466/1466`, finite and exact-key | `61082` factors, `7332` Values; `0` accepted; cost unchanged at `166205998.11910567`; ten trials, `1e-5 -> 1e5` |

Additional sealed Phase98 facts for both main routes:

- Initial graph cost and first linearization were observed and finite.
- Every classified factor-family contribution was finite.
- All ten LM trials reported `indeterminate_linear_system`; the terminal
  branch was `maximum_lambda`. No finite delta or accepted main step was
  observed.
- The existing solver telemetry was `MULTIFRONTAL_CHOLESKY`, branch
  `multifrontal`, elimination `EliminatePreferCholesky`, ordering `COLAMD`,
  no explicit ordering, and `diagonal_damping=false`.
- The C0/D conditioning proxies were approximately `1.9999999980791472`
  (MTV-A) and `1.9999999980209395` (LAX-T). These are not full
  normal-matrix condition numbers and do not prove rank.
- Phase98 reported no nearby key because the pinned LM catch boundary did not
  retain the exception object. The Phase97 large sidecar likewise did not
  establish dense numerical rank; its one anchored component and absence of
  reporting-level zero columns are structural observations only.

These facts qualify a solver-path hypothesis. They do not establish that QR
will accept a step, that the graph is full rank, or that a route is accurate.

## Why the comparison is not an append-only CCDD test

The sealed Phase80 and Phase98 graph recipes differ in several coupled ways:

- Phase80 used adjacent legacy clock rows; Phase98 used the official CCDD
  row with the meter-state C/ISB convention.
- Phase98 uses C and ISB in metres, D in metres/second, and `dt` in seconds;
  Phase80's legacy receiver-clock state was seconds-valued while D remained
  metres/second.
- Phase80 retained signal-bias state families and quality-anchor/base
  compensation settings; Phase98's no-base direct recipe does not retain
  those signal-bias states and consumes different observation populations.
- The route totals consequently differ in factors and Values. The concrete
  COLAMD permutation can therefore differ even though the ordering policy is
  the same.

Thus the evidence supports “the CCDD meter-state recipe plus its changed
graph reaches a Cholesky failure boundary” rather than “one positive
semidefinite row made an otherwise identical matrix singular.” That
distinction is why QR is frozen as one bounded candidate, not as a proven
cause or guaranteed fix.

## Local GTSAM semantics

The following claims were checked against the pinned local GTSAM 4.3a source;
no external solver was run.

| Source | Observation |
|---|---|
| `gtsam/nonlinear/NonlinearOptimizerParams.h`, SHA-256 `0cf55b15a4d04aa1364b63bdae659fc65f568a8e08898a104a28d7fdc6ca07a5` | `MULTIFRONTAL_CHOLESKY` and `MULTIFRONTAL_QR` are supported enum values; both are multifrontal. The default is Cholesky. Cholesky maps to `EliminatePreferCholesky`; QR maps to `EliminateQR`. |
| `gtsam/nonlinear/LevenbergMarquardtParams.h`, SHA-256 `dd9348c08a96f0dffb2cf6b6f24eab18d4d32280a21e333a1e38725b254ec3f2` | Default `orderingType` is `COLAMD`; `EnsureHasOrdering` materializes that ordering when no explicit ordering is supplied. Legacy LM values include initial lambda `1e-5`, factor `10`, upper bound `1e5`, `minModelFidelity=1e-3`, and `diagonalDamping=false`. |
| `gtsam/nonlinear/NonlinearOptimizer.cpp`, SHA-256 `594fd9bf3d48199f00d05794913df6c84d300664458d609fcdf0fb6ed3215a35` | In multifrontal mode, the same `GaussianFactorGraph` and ordering are passed to `gfg.optimize`; the selected elimination function is the parameter-dependent part. |
| `gtsam/inference/EliminateableFactorGraph-inst.h`, SHA-256 `e652ffda77bb22c58ef55607bb566296f97060d694db449dafacb1bdbafe7845` | Multifrontal elimination builds the same variable index/order/tree and invokes the supplied elimination function. A Cholesky-to-QR enum switch does not itself select sequential elimination or a new ordering. |
| `gtsam/linear/HessianFactor.cpp`, SHA-256 `9ad05f58d0f77faea406b6d86a909e8c829bfb55ff05fd5c92d70c58938cded0` | `EliminatePreferCholesky` uses Cholesky for ordinary unconstrained noise models and uses QR when constrained noise requires it; explicitly selecting QR uses the QR path. |
| `gtsam/linear/JacobianFactor.cpp`, SHA-256 `1b5eef269cab6999f580b40c0d6ba65a732a3c51206b523744182a60dd5b0eee` | The supported `EliminateQR` path operates on Jacobian factors and still has explicit insufficient-row/indeterminate checks. QR is not a guarantee of solvability. |
| `gtsam/nonlinear/LevenbergMarquardtOptimizer.cpp`, SHA-256 `834f732ac4ad4130b2a719a3181e5dfcd2ecdf584ece6f0c76bdf5f390e10cbc` | Each trial builds the damped system with existing LM settings, calls `solve`, catches `IndeterminantLinearSystemException`, and increases lambda until a step succeeds or the upper bound is reached. The current catch discards the exception object. |
| `gtsam/linear/linearExceptions.h`, SHA-256 `15d3c2e1ae816f688e2d90cd37212cb7559330862cf6dd061462e967a2ffa418` | The exception represents an indeterminate linear system and carries a nearby key; that key is a detection location affected by elimination order, not proof of the original rank defect. |

The candidate therefore changes a real factorization/elimination branch. It
does not merely add telemetry, and it must be evaluated as an algorithmic
candidate with its own implementation and qualification. QR can avoid a
Cholesky positive-definiteness requirement and may expose a usable least-
squares step for a numerically semidefinite/ill-conditioned linearization;
that is the numerical rationale, not a result observed in this audit.

## LM, equation, units, and handoff invariants

The native main solve is assembled at
`src/algorithms/fgo_gtsam_backend.cpp:1401-1423`. The future branch may set
only the main meter-state invocation's `linearSolverType`; it must not set an
explicit ordering. The existing implicit COLAMD order, LM lambda sequence,
trial count, max iterations, acceptance test, and terminal handling remain
the contract.

The official CCDD factor and native implementation retain:

\[
r=(C_2-C_1)-\frac{(D_1+D_2)\,dt}{2},\qquad
J_{C_1,C_2,D_1,D_2}=[-1,+1,-dt/2,-dt/2].
\]

The frozen invariant is C/ISB in metres, D in metres/second, `dt` in
seconds, official CCDD sigma `0.1 m`, retained raw D initialization, exact
retained-key alignment, finite/full optimized C/D, and the existing
fail-closed/no-fallback contract. No factor, filter, residual, prior,
equation, unit, sigma, or initialization change is included in the candidate.

## Official MATLAB staging consistency

The official source is a staging reference, not an execution input here.

- `fgo_gnss.m:34-48` initializes or reloads position, velocity, `clk`, and
  `dclk`; `:103-120` inserts `x`, `v`, `c`, and `d`; `:170-176` adds the
  `ClockFactor_CCDD` row; and `:203-214` uses
  `gtsam.LevenbergMarquardtParams` and `gtsam.LevenbergMarquardtOptimizer`.
- `fgo_gnss.m:220-229` exports `clkest` and `dclkest` together with position
  and velocity in `result_gnss.mat`.
- `fgo_gnss_imu.m:40-61` loads that GNSS-first result and uses `posest`,
  `velest`, `clkest`, and `dclkest` as the next-stage initialization;
  `:165-187` inserts the corresponding Pose3/position/velocity/clock/drift/
  bias Values; `:270-276` adds CCDD; and `:325-335` uses the same GTSAM LM
  optimizer family.

The MATLAB source does not explicitly request QR, so it is not evidence that
QR will succeed. A main-only QR branch does not contradict its state staging:
the GNSS-first solver and the C/D/position/velocity handoff remain unchanged.

## Candidate boundary and exclusions

Exactly one candidate is frozen in the companion JSON. Its required future
implementation boundary is:

1. Add one opt-in selector whose scope is the Phase93/98 main meter-state
   graph only; the selector must not affect the GNSS-first call.
2. Immediately before that main optimizer is constructed, select
   `gtsam::NonlinearOptimizerParams::MULTIFRONTAL_QR` in the existing params
   object. Preserve the current empty explicit ordering so GTSAM materializes
   the same COLAMD policy.
3. Keep the existing LM optimizer, graph, Values, factors, CCDD equation,
   units, sigma, filters, retained D seed, exact-key handoff, and output
   gates. If the QR path fails a structural gate or throws, fail closed; do
   not retry Cholesky or publish a solution.

The following are explicitly not frozen candidates: sequential QR, changing
ordering, adding damping or changing lambda/iteration settings, changing
CCDD sigma/equation/units, changing filters or initialization, changing
factor families, switching GNSS-first to QR, and a Cholesky-then-QR fallback.

## Next boundary

The next authorized action is implementation of this single opt-in branch,
focused synthetic/regression coverage, and a source/diff qualification that
proves legacy default and GNSS-first invariants. A new pre-raw manifest,
evaluator, and independent authorization would be required before any raw
execution. This Phase99 audit/freeze itself authorizes none of those runs and
makes no accuracy, truth, MAT, base, precomputed-coordinate, or Kaggle claim.
