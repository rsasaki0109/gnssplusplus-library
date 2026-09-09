# Phase98 solver/rank audit

- Execution label: `Luna Max`
- Audit date: `2026-09-03`
- Status: `sealed-read-only-static-and-artifact-audit`
- Scope: the sealed Phase80 active-main records and the sealed Phase97
  singular-main records for `2021-03-16-18-59-us-ca-mtv-a/pixel5` (MTV-A) and
  `2022-04-01-18-22-us-ca-lax-t/pixel5` (LAX-T).
- This turn performed no raw-input read, native solver run, truth/MAT/Kaggle
  access, accuracy calculation, or solution publication. No exact Phase97
  incidence list was copied or regenerated; the existing 173,215,107-byte
  Phase97 result remains the only such artifact.

This record separates sealed observations from source-derived conclusions. It
does not promote a solution or identify a unique numerical root cause where
the available solver boundary does not expose one.

## Decision

The earliest common observed Phase97 failure boundary is the first main-stage
damped linear-system solve. It is not established that the C0/D row itself
lowered rank:

1. Phase80's different no-C0D main graph accepted 12 outer iterations and
   strictly reduced cost for both route identities.
2. Phase97's GNSS-first meter C0/D stage made finite strict progress and
   exported every retained D key. Its same-run main graph then evaluated and
   linearized, but all ten observed LM trials failed before a finite delta,
   and no main outer step was accepted.
3. The Phase80-to-Phase97 change is not “append one PSD factor to the same
   matrix.” The legacy clock row was replaced by CCDD, direct code/Doppler
   populations changed, Phase80 signal-bias states were removed, MTV-A's ISB
   count changed, the C state changed from seconds to metres, and the concrete
   COLAMD permutation can therefore change with the graph topology.
4. The Phase97 sidecar found one anchored structural component, no exact or
   reporting-level zero columns, and finite factor-family linearizations, but
   deliberately did not compute dense rank for its large matrices. Its rank
   status is therefore unknown, not full-rank and not proven singular.

The sole candidate frozen after this audit is an observation-only solver-path
and exception-boundary diagnostic. It records the configured solver and
ordering, pinned LM damping, existing trial branch, and an actual nearby key
only when the exception object is naturally available. It does not switch to
QR, run a second elimination, modify the graph, or rerun a route. The freeze
JSON is a separate commit and authorizes no execution.

## Read accounting

| Activity | Count |
|---|---:|
| Native solver invocations | `0` |
| Raw `device_gnss.csv` reads | `0` |
| Raw `device_imu.csv` reads | `0` |
| Broadcast navigation reads | `0` |
| Truth reads | `0` |
| MAT reads/generated | `0` |
| Kaggle/token access | `0` |
| Base RINEX reads | `0` |
| Precomputed-coordinate reads | `0` |
| Accuracy calculations/submission | `0` |
| Route reruns/fallbacks | `0` |
| New large incidence artifacts | `0` |

Only committed source/record files, the already-sealed Phase80 summaries, and
the already-sealed Phase97 result metadata were inspected. Historical Phase80
base-RINEX provenance is described below only as a configuration difference;
it was not read by this audit.

## Authorities

| Item | Commit or digest |
|---|---|
| Phase80 source/freeze authority | source commit `f28aebaf1b41df3ee5e363a8ad94987247321048`; freeze `docs/use_cases/records/smartphone_r5_phase80_source_exact_direct_p_quality_freeze_v1.json`, SHA-256 `9ebdde21fe7fd955029243d3bdbba3a9f9d3832388d67480760394177c8f685e` |
| Phase80 manifest | `docs/use_cases/records/smartphone_r5_phase80_source_exact_direct_p_quality_structural_manifest_v1.json`, SHA-256 `6d20541560b726ed2409cd05902e5105874ac72e113af0c3812f029130df310f` |
| Phase80 MTV-A sealed summary | `output/smartphone-r5/phase80-source-exact-direct-observable-quality-structural-v2/2021-03-16-18-59-us-ca-mtv-a/pixel5/candidate/run1/summary.json`, SHA-256 `ff0e1be40bd12f5fabd9fa8e0571e61049e1e23c34c347618e35f9b68a5f5181` |
| Phase80 LAX-T sealed summary | `output/smartphone-r5/phase80-source-exact-direct-observable-quality-structural-v2/2022-04-01-18-22-us-ca-lax-t/pixel5/candidate/run1/summary.json`, SHA-256 `599a399debe0800ed4fc794b3b39ffb31e384b233a3fe08338a60f4176b21a9a` |
| Phase97 implementation | commit `e912d4571ba684453a0bc6fa73cdd0b99d9ffc86`; native algorithm, solver/filter/LM, equation, units, and sigma were declared unchanged by its sealed manifest |
| Phase97 freeze | commit `587ef8db667b47ec539aba9d0a76d43698d8bc53`; freeze SHA-256 `07723240dc56ae80c3d21e25b783fb086c5e9ea4af47e62cd56fc5b81980d174` |
| Phase97 execution | result commit `dd32a32ecc2960322511ecf6753431bda64c603a`; result SHA-256 `abf4027bec33d51b54fcfa2c3def55ff9ea22568fca7eeeb83049271cf2efecc`; file size `173215107` bytes |
| Current native backend | `src/algorithms/fgo_gtsam_backend.cpp`, SHA-256 `26f5822d5ed31b8a1b739aecf94afe17ec71ac8847d02a8bac901f28f9f12379` |
| Current native LM/diagnostic wrapper | `src/algorithms/fgo_gtsam_internal.hpp`, SHA-256 `4ba74c23855472a785f3d2ad57a2c85fd80e233b85a54a2c230f2b5ef6e905f8` |
| Native route/config assembly | `apps/native/gnss_fgo_imu_no_base.cpp`, SHA-256 `c0c49d35df4eb0abf56c2ef64d0beb80c0c6bc16023b987a74bb2d42cd77b55a` |
| FGO configuration defaults | `include/libgnss++/algorithms/fgo_config.hpp`, SHA-256 `0bb86f2b9755f7de68c1274f4eebb99bf441f597514c5a421b2c25cc8906f6e9` |
| Pinned GTSAM solver params | `output/reproducibility-cache/gtsam-4.3a/gtsam/nonlinear/NonlinearOptimizerParams.h`, SHA-256 `0cf55b15a4d04aa1364b63bdae659fc65f568a8e08898a104a28d7fdc6ca07a5`; `LevenbergMarquardtParams.h`, SHA-256 `dd9348c08a96f0dffb2cf6b6f24eab18d4d32280a21e333a1e38725b254ec3f2` |
| Pinned GTSAM LM path | `LevenbergMarquardtOptimizer.cpp`, SHA-256 `834f732ac4ad4130b2a719a3181e5dfcd2ecdf584ece6f0c76bdf5f390e10cbc`; `NonlinearOptimizer.cpp`, SHA-256 `594fd9bf3d48199f00d05794913df6c84d300664458d609fcdf0fb6ed3215a35` |
| Pinned GTSAM elimination/order path | `Ordering.h`, SHA-256 `7ab402edf858426a05dcd122914f9d380609d34f95d92ceefc843c36ecf5766c`; `EliminateableFactorGraph-inst.h`, SHA-256 `e652ffda77bb22c58ef55607bb566296f97060d694db449dafacb1bdbafe7845`; `GaussianFactorGraph.cpp`, SHA-256 `c4c0fd3543dd7a35eca722429f188e08bc96e31d877109b92b2c9ac599039d45` |
| Pinned GTSAM exception/elimination details | `linearExceptions.h`, SHA-256 `15d3c2e1ae816f688e2d90cd37212cb7559330862cf6dd061462e967a2ffa418`; `HessianFactor.cpp`, SHA-256 `9ad05f58d0f77faea406b6d86a909e8c829bfb55ff05fd5c92d70c58938cded0`; `linearAlgorithms-inst.h` was inspected read-only |
| Official CCDD reference | `output/reproducibility-cache/gtsam_gnss/src/ClockFactor_CCDD.h`, SHA-256 `7c174217218aa0059155e1e5a7182c69d56a3cd389d0ebd95c242d6d8c65b9fc` |

## Sealed route behavior

The Phase80 rows below are the active main graph summaries. The Phase97 rows
are the exact aggregate fields in the sealed result; its full keyed incidence
payload is intentionally not reproduced here.

| Route | Phase80 graph factors / values | Phase80 main iterations and cost | Phase97 graph factors / values | Phase97 main accepted / cost | Phase97 first-stage C0/D |
|---|---:|---|---:|---|---|
| MTV-A | `143414 / 10799` | `12`; `155976340.16237149 → 28128.450524519692` | `120093 / 10798` | `0`; `79358354.39651252 → 79358354.39651252`; 10 trials, lambda `1e-5 → 1e5` | accepted `167`; `5618000.786274777 → 21298.114523521894`; C0/D rows `2158`; D `2159/2159`, finite and exact-key |
| LAX-T | `88025 / 7333` | `12`; `190528071.24200463 → 22206.857308543094` | `61082 / 7332` | `0`; `166205998.11910567 → 166205998.11910567`; 10 trials, lambda `1e-5 → 1e5` | accepted `262`; `6419784.856579111 → 12818.288538454106`; C0/D rows `1465`; D `1466/1466`, finite and exact-key |

Phase80's GNSS-first staging also progressed (`192` iterations on MTV-A and
`163` on LAX-T), but it was a no-C0D staging graph. Phase97's staging graph
was the meter C0/D Point3+velocity graph. On both routes it passed the
finite/strict-progress handoff checks before the main failure.

Phase97 recorded one anchored component and zero exact/near-zero keyed
columns in each main graph. Its rank records were
`rows=165425, columns=36706` for MTV-A and `rows=91861, columns=24924` for
LAX-T, with status `rank_revealing_unavailable_due_to_size`; the structural
support pass did not establish numerical rank or nullity. The C0/D aggregate
conditioning proxies (`1.9999999980791472` and `1.9999999980209395`) are not
complete-normal-matrix condition numbers.

## Exact configuration and graph-set comparison

### Route/config contract

Both manifests select direct source observable quality and the explicit
no-PDC spelling. Both main paths use `Pose3 + IMU`, 12 main LM iterations,
undifferenced corrected Doppler, ordinary TDCP, robust loss, position/motion
rows, inter-system bias states, and no standalone carrier-phase factors or
double-difference factors. The route-specific source-quality Huber settings
are preserved in Phase80 and inherited by Phase97's direct-quality path.

The material differences are:

| Setting or side effect | Phase80 active main | Phase97 singular main |
|---|---|---|
| Direct quality / PDC | `--native-source-direct-observable-quality` plus `--native-pdc-imu-tdcp-no-bridge`; no PDC bridge | Same direct/no-bridge flags |
| Source C0/D | selector off; adjacent `BetweenFactor<double>` legacy clock rows | `--native-source-clock-c0d-factor` and meter-state parity on; adjacent official CCDD rows replace legacy rows |
| C0/D units | receiver C is stored/whitened in seconds; D is m/s | receiver C and ISB are metres; D is m/s; `dt` is seconds; CCDD residual is metres |
| GNSS-first stage | Point3 + velocity, no C0/D; direct Doppler staging | Point3 + velocity, C0/D meter rows; D starts from retained raw EpochSeed drift, then optimized C/D and same-run position/clock/velocity are handed off |
| Pseudorange input path | source direct quality plus Phase78 base-RINEX pseudorange compensation and miss mask; summary still has `base_factors=false` because no base graph factors are added | no base-RINEX compensation, no external/base input |
| Quality anchor | `--native-quality-anchor` and fallback recovery on | off |
| Signal-bias state | `--native-signal-bias-states` on; static `f` states and their priors/factors | off; no `f` states |
| TDCP / carrier | TDCP on; standalone carrier off | TDCP on; standalone carrier off |
| IMU / Doppler | Combined IMU factors and direct receiver-only Doppler; D exists from this Doppler path | Same families; C0/D additionally couples pre-existing D states to C |
| Main damping/solver flags | no native override | no native override; diagnostic wrapper changes only `verbosityLM` to `TRYLAMBDA` to capture existing trials |

The Phase97 CLI also enables phase96/97 telemetry selectors and the
GNSS-first handoff selector. Those selectors do not add a graph factor or
change an LM parameter. The Phase97 main `ordering_context` value
`initial Values::keys() ascending` is the sidecar's output enumeration, not
the optimizer's elimination permutation.

### Values and variable families

For both route identities, the active IMU graph has five per-epoch values:
`x=Pose3` (tangent dimension 6), `v=Vector3` (3), `b=ConstantBias` (6),
`c=scalar` (1), and `d=scalar` (1). Thus each epoch contributes 17 tangent
dimensions. The following dimensions are derived from the sealed Values count
and pinned GTSAM value dimensions; they are not a new runtime dump.

| Route | N | Phase80 extras | Phase80 Values / tangent dim | Phase97 extras | Phase97 Values / tangent dim |
|---|---:|---|---:|---|---:|
| MTV-A | `2159` | 2 global ISB + 2 signal-bias `f` | `10799 / 36707` | 3 global ISB; no `f` | `10798 / 36706` |
| LAX-T | `1466` | 2 global ISB + 1 signal-bias `f` | `7333 / 24925` | 2 global ISB; no `f` | `7332 / 24924` |

Therefore Phase97 did not introduce the per-epoch D family: Phase80 already
had D because direct Doppler factors were active. MTV-A gained one global ISB
state but lost two signal-bias states; LAX-T lost one signal-bias state with
the same ISB count. The net Values count decreased by one on each route; no
new isolated variable family is proven by this comparison.

### Factor-family counts

The Phase80 insertion breakdown is derived from its sealed graph summary and
the source's fixed `N-1` temporal loops. The Phase97 breakdown is the exact
sidecar classifier aggregate. Phase97's `imu_preintegration_bias` bucket
includes the first bias prior; its `priors` bucket therefore contains the
first pose/velocity priors and broad V/D priors.

| Family | Phase80 MTV-A | Phase80 LAX-T | Phase97 MTV-A | Phase97 LAX-T |
|---|---:|---:|---:|---:|
| IMU/preintegration (+ Phase97 first-bias prior) | `2158` | `1465` | `2159` | `1466` |
| GNSS code | `56547` | `41935` | `57281` | `30798` |
| GNSS Doppler | `29588` | `13795` | `20748` | `8942` |
| TDCP (`other` in Phase97 classifier) | `46482` | `24964` | `31269` | `14012` |
| Position motion | `2158` | `1465` | `2158` | `1465` |
| Clock temporal row | `2158` legacy | `1465` legacy | `2158` CCDD | `1465` CCDD |
| Broad V/D priors | `4318` | `2932` | included in `priors` | included in `priors` |
| First pose/velocity/bias priors | `3` | `3` | bias included above; 2 in `priors` | bias included above; 2 in `priors` |
| Signal-bias priors | `2` | `1` | `0` | `0` |
| Phase97 classifier `priors` total | not separately serialized | not separately serialized | `4320` | `2934` |
| Graph-factor total | `143414` | `88025` | `120093` | `61082` |

No IMU, code, Doppler, TDCP, or position-motion family disappeared entirely.
Their populations changed because Phase80's base-compensated quality recipe,
signal-bias eligibility, and Phase97's no-base direct recipe do not consume
the same retained observation rows. The legacy clock family was replaced by
the same-count CCDD family, rather than CCDD being appended to an unchanged
graph. Signal-bias factors/states/priors were removed. C0/D factors were
added as a named family, but they also changed the C state units and replaced
the legacy clock equation.

### Priors and physical units

The native source inserts a finite first Pose3, first velocity, and first IMU
bias prior. It also inserts broad `1e6`-unit priors for every velocity and D
state when direct Doppler/IMU-D is active. Their nominal diagonal contribution
is `1e-12`; they are numerical gauge guards, not truth or trajectory input.
There is no default explicit C prior or global ISB prior. Phase80's legacy
clock row converts its metre sigma to the seconds-valued clock state. Phase97
uses the official meter-state CCDD row with ordinary sigma `0.1 m`, C/ISB in
metres and D in metres/second. Phase80's static signal-bias states have a
`1000 m` prior; Phase97 removes that family.

The official CCDD equation and Jacobian are unchanged:

\[
r=(C_2-C_1)-\frac{(D_1+D_2)\,dt}{2},\qquad
J_{[C_1,C_2,D_1,D_2]}=[-1,+1,-dt/2,-dt/2].
\]

The native source implements the same equation at
`src/algorithms/fgo_gtsam_internal.hpp:1230–1283`, with the meter-state
scale selected only by the Phase92/93 opt-in flag. No equation, factor sigma,
filter, or LM tuning change is inferred or proposed here.

## LM and sparse-elimination audit

### Effective path in both main runs

The Phase80 source constructs `gtsam::LevenbergMarquardtParams` and calls the
pinned `gtsam::LevenbergMarquardtOptimizer` directly. Phase97 constructs the
same optimizer inside `GtsamLmActiveSolveOptimizer`; its
`diagnosticParams` copies the params and changes only `verbosityLM` when
telemetry is enabled. The native backend does not set `linearSolverType`,
`orderingType`, or an explicit `Ordering` in either path.

The pinned defaults and call path are exact in the local source:

- `NonlinearOptimizerParams.h:47` defaults `orderingType` to `COLAMD`.
- `NonlinearOptimizerParams.h:98–108` declares the solver enum and defaults
  `linearSolverType` to `MULTIFRONTAL_CHOLESKY`.
- `LevenbergMarquardtParams.h:61–82` sets legacy LM defaults: initial lambda
  `1e-5`, multiplicative factor `10`, upper bound `1e5`, lower bound `0`,
  minimum model fidelity `1e-3`, fixed lambda factor, and
  `diagonalDamping=false`.
- `fgo_gtsam_backend.cpp:1400–1410` (and the Phase80 source at its equivalent
  solve block) sets the main maximum to 12, absolute tolerance to `1e-10`
  when the config value is zero, and relative tolerance to `1e-8` when zero.
- `LevenbergMarquardtOptimizer.cpp:49–63` calls `EnsureHasOrdering`; the
  empty explicit ordering is consequently materialized as a COLAMD ordering.
- `NonlinearOptimizer.cpp:132–153` dispatches the default multifrontal mode to
  `GaussianFactorGraph::optimize(ordering, getEliminationFunction())`.
- `NonlinearOptimizerParams.h:137–145` maps both Cholesky enum variants to
  `EliminatePreferCholesky` and both QR variants to `EliminateQR`.
- `GaussianFactorGraph.cpp:316–318` sends the multifrontal call through
  `eliminateMultifrontal(ordering, function)`. `EliminatePreferCholesky`
  chooses Cholesky for ordinary unconstrained noise models and only falls back
  to QR when constrained noise requires it.

Thus the solver *type and ordering policy* are unchanged between the sealed
Phase80 and Phase97 main paths: multifrontal, COLAMD, and prefer-Cholesky.
The actual COLAMD permutation can differ because the factor/key graph differs.
Phase97's `TRYLAMBDA` output is observability only. Its sealed trial lambdas
`1e-5, 1e-4, ..., 1e4` and terminal `maximum_lambda` agree with the pinned
LM control flow, not with a changed damping policy.

### QR alternatives are not source-preserving diagnostics

The pinned API supports these alternatives:

| Parameter | Pinned operation | Audit decision |
|---|---|---|
| `MULTIFRONTAL_CHOLESKY` | multifrontal elimination with `EliminatePreferCholesky` | current default path |
| `MULTIFRONTAL_QR` | multifrontal elimination with `EliminateQR` | rejected as a diagnostic switch: changes the linear solve path |
| `SEQUENTIAL_CHOLESKY` | sequential elimination with `EliminatePreferCholesky` | rejected: changes elimination topology/path |
| `SEQUENTIAL_QR` | sequential elimination with `EliminateQR` | rejected: changes both factorization and elimination path |

Changing any of the latter three settings would be an algorithm/solver
candidate, not a source-preserving observation of the Phase97 failure. It
could change whether a trial is accepted and therefore cannot be folded into
this read-only audit or the existing Phase97 authorization.

### Nearby-variable availability

The public exception does retain a nearby key. The pinned
`linearExceptions.h:67–100` documents that the key is where the problem was
discovered, is ordering-dependent, and is not necessarily the origin. The
actual sparse path can throw it in at least two relevant places:

- `HessianFactor.cpp:459–483` catches a Cholesky failure and throws
  `IndeterminantLinearSystemException(keys.front())` during clique
  elimination.
- `linearAlgorithms-inst.h:95–100` throws with the front key of a conditional
  if back-substitution produces NaNs.

The public graph API exposes `eliminateMultifrontal` and
`eliminateSequential` with an explicit ordering and elimination function
(`EliminateCholesky`, `EliminateQR`, or the prefer-Cholesky function). A
separate call on a copied linearized graph could therefore catch an actual
exception object and read `nearbyVariable()`. It would nevertheless be a
second sparse elimination/linear solve, not a passive read of the existing LM
trial. The Phase97 freeze explicitly forbids an additional optimizer or linear
solver, and no such call was made here.

More importantly, pinned `LevenbergMarquardtOptimizer::tryLambda` catches the
exception at `LevenbergMarquardtOptimizer.cpp:153–160` as
`const IndeterminantLinearSystemException&` without binding or exporting the
object. The Phase97 wrapper consequently records the honest
`nearby_variable_unavailable` status. A guessed key would turn a boundary
observation into an unsupported root-cause claim.

## Positive-semidefinite factor premise

For a fixed variable set and fixed existing factors, a weighted linearized
least-squares system has

\[
H=A^T W A,\qquad H' = H + B^T W_B B,
\]

with `W` and `W_B` positive semidefinite. Therefore

\[
x^T H'x=x^T Hx+\|W_B^{1/2}Bx\|^2\ge x^T Hx,
\]

and `ker(H') = ker(H) ∩ ker(W_B^{1/2}B)`, so rank cannot decrease when only a
PSD factor is appended to the same columns. The ordinary `0.1 m` CCDD noise
model satisfies this factor-level premise.

The sealed runs do not satisfy the *same-columns/same-factors* premise:

- Phase80 has signal-bias `f` states and priors; Phase97 removes them.
- Phase80 has a legacy clock row; Phase97 replaces it with CCDD rather than
  appending a row to the old graph.
- MTV-A has two Phase80 ISB states and three Phase97 ISB states; LAX-T keeps
  two. Per-epoch D already existed in Phase80, so C0/D did not merely append a
  new D family.
- Code and Doppler retained-row counts differ, and Phase80's base-RINEX
  compensation changes code residual inputs. The observed graph factor
  totals are consequently different.
- C is scaled from seconds to metres in the Phase97 meter-state graph. The
  factor equation is source-aligned, but the numerical columns and COLAMD
  graph are not the Phase80 columns.

The correct conclusion is therefore “the graph/scale/solver-boundary
differences warrant diagnosis,” not “CCDD made a previously full-rank matrix
rank-deficient.” No sealed artifact identifies a key collision, isolated
component, globally zero column, or a C/D/ISB gauge as the cause.

## Candidate comparison and freeze boundary

| Candidate | Evidence and effect | Decision |
|---|---|---|
| Switch the main LM to `MULTIFRONTAL_QR` or `SEQUENTIAL_QR` | The pinned API supports it, but it changes `getEliminationFunction()` and/or the elimination traversal; trial acceptance may change | Reject for this audit; not source-preserving |
| Run a second Cholesky/QR sparse elimination on the captured linear graph to obtain rank or `nearbyVariable()` | The public API can expose the key at an elimination boundary, but this is another linear solve/factorization and its key is ordering-dependent | Reject for this audit and existing Phase97 freeze; no run |
| Record the existing solver enum/elimination branch, ordering policy/context, LM damping, and naturally available exception metadata at the existing boundary | Tests the unresolved solver-path hypothesis without changing graph, Values, factors, ordering, LM parameters, or trial control flow; preserves explicit unavailable status | **Freeze exactly this one candidate** |

The candidate's implementation boundary is observation only:

1. Record the effective `linearSolverType`, multifrontal/sequential branch,
   `EliminatePreferCholesky`/`EliminateQR` selection, `orderingType`, whether
   an explicit ordering exists, and a compact ordering digest/count. Do not
   re-emit the exact incidence list.
2. Record the pinned LM lambda initial/factor/lower/upper values,
   `diagonalDamping`, model-fidelity threshold, and the already existing first
   ten trial branches.
3. At a naturally reachable exception boundary, record exception class,
   stage, message, and `nearbyVariable()` Symbol character/index. If the
   pinned LM catch remains the source boundary, record
   `nearby_variable_unavailable`; do not reconstruct a key or perform a second
   elimination.
4. Reuse the existing initial-linearization keyed rank status and zero/near-zero
   summary as metadata only. No additional rank decomposition, dense matrix,
   external solver, or matrix modification is allowed.
5. Preserve the official CCDD equation/Jacobian, meter C/ISB and m/s D units,
   `0.1 m` sigma, filters, priors, LM control flow, legacy defaults, no-PDC
   policy, fail-closed behavior, and no solution/accuracy publication.

This candidate is default-off, not implemented by this audit, not raw-run
authorized, and not solver-rerun authorized. Its exact policy is sealed in
`smartphone_r5_phase98_solver_rank_diagnostic_freeze_v1.json` in a separate
commit after this audit commit.

## Release boundary

Phase98 establishes no solver correction, alternate factorization result,
solution, accuracy value, or promotion. Any implementation or execution of
the single observation candidate requires a separately pinned implementation,
qualification, and authorization. QR switching and extra sparse elimination
remain explicitly outside this freeze.
