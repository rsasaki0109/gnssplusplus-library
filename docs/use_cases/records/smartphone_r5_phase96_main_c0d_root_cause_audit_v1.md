# Phase96 main C0/D no-progress root-cause audit

- Execution label: `Luna Max`
- Audit status: `sealed-read-only-source-and-artifact-audit`
- Audit date: `2026-09-03`
- Scope: the two Phase95 routes whose GNSS-first result reached the exact-key,
  finite optimized-D handoff: `2021-03-16-18-59-us-ca-mtv-a/pixel5` (MTV-A)
  and `2022-04-01-18-22-us-ca-lax-t/pixel5` (LAX-T).
- MTV-H and MTV-U are explicitly outside this audit and candidate freeze. No
  MTV-H/U correction, route selection, or route-specific tuning is proposed.
- Raw execution in this audit: **0**. No raw GNSS, raw IMU, or broadcast
  navigation file was opened; no truth, MAT, Kaggle/token, base,
  precomputed-coordinate, or accuracy artifact was read.
- Native solver invocations in this audit: **0**. No route was rerun.
- Source changes in this audit: **0**. The only changes authorized after this
  record are the separate, diagnostic-only freeze record.

This record separates facts forced by the sealed artifacts and pinned source
from inferences that cannot be proved without new telemetry. It is a failure
analysis, not a convergence, accuracy, promotion, or submission result.

## Decision

The strongest reproducible statement is:

1. The GNSS-first Point3/velocity C0/D graph completed a finite strict-cost
   progress solve on both in-scope routes and exported a full, finite D vector
   with exact retained-key alignment.
2. The same-run main meter-state graph was constructed and returned finite
   structural state coverage, but accepted zero outer LM steps on both routes;
   its reported initial and final nonlinear costs were equal, and its lambda
   reached the reported maximum after ten inner attempts.
3. The sealed main result does not identify whether those attempts were
   rejected because of an indeterminate linear system, a non-positive
   linearized reduction, a non-decreasing nonlinear cost, insufficient model
   fidelity, a small-change branch, or another exception/termination path.
4. The current source has the local quantities needed to distinguish those
   cases, but the public FGO result and Phase95 artifact expose only aggregate
   costs/counters. The requested factor-family costs, per-variable/family
   gradient and normal-diagonal norms, and first-ten predicted-versus-actual
   LM trial records therefore cannot be reconstructed from the sealed record.

The sole defensible next candidate is an opt-in, source-aligned diagnostic
telemetry surface that observes the existing main graph and pinned LM
`tryLambda` path without changing graph factors, C0/D units or sigma, LM
parameters, acceptance rules, fallback policy, or route set. This candidate is
frozen separately and is **not implemented or authorized to run** by this
audit.

### Minimal-cause answer

The minimum cause that the evidence supports is not a particular factor or
solver exception; it is **main-stage LM made no accepted progress for all ten
existing lambda attempts under the meter-state C0/D graph**. GNSS-first
progress, exact C/D handoff, finite state coverage, and graph construction are
already established for MTV-A and LAX-T. The next smaller claim—such as “the
CCDD Jacobian is singular,” “Doppler dominates,” or “a nonfinite trial caused
the rejection”—is not supported because the corresponding family and
per-trial telemetry is absent. Phase80's no-C0D success localizes the contrast
to the C0/D-enabled main experiment, but does not identify the mathematical
failure mechanism.

## Authority and hashes

| Item | Pin |
|---|---|
| Phase95 corrected structural result | `docs/use_cases/records/smartphone_r5_phase95_raw_input_path_corrected_structural_result_v1.json`, SHA-256 `beba25d4b0d910064d76bea7aa6b7a884937ca01c3b00cf0f7f9f4b6267ea281`, commit `f120e68` |
| Phase95 corrected structural markdown | `docs/use_cases/records/smartphone_r5_phase95_raw_input_path_corrected_structural_result_v1.md`, SHA-256 `1d4cd803e216b23d3c467c1e2ece3c287dfad0171fd43a1fff30c8829395181c` |
| Phase95 corrected evaluator | `apps/commands/benchmarks/gnss_smartphone_phase95_raw_input_path_corrected.py`, SHA-256 `ca02b61b91255b6a82d756feb3f42e1d40ce71dcea433aee319b68ac8ef16c6f` |
| Phase95 corrected execution wrapper | `apps/commands/benchmarks/gnss_smartphone_phase95_raw_input_path_corrected_execute.py`, SHA-256 `5ac858cf156555652f6feae60c71a4237269d4b4c2cb70c1adf5a9e08f10e883` |
| Phase95 implementation metadata | sealed result implementation commit `24f3329841f29d18b0b92c5476887464d8bf8be6`, binary SHA-256 `76f6cfe06becadb464bbda3b085ecde93efc803bb74188448ba4abe2caabb621` |
| Phase92 structural result | `docs/use_cases/records/smartphone_r5_phase92_source_meter_clock_state_parity_structural_result_v1.json`, SHA-256 `8488306b3ec61d0360418de73fa1596271597b6971d77de676fcd279d7e1b01c` |
| Phase92 alignment/unit audit | `docs/use_cases/records/smartphone_r5_phase92_phase91_alignment_and_clock_isb_unit_audit_v1.md`, SHA-256 `45b15439063eaa823f786df15155c83b92e65939812e1d18698cf1812d0f5271` |
| Phase80 structural manifest | `docs/use_cases/records/smartphone_r5_phase80_source_exact_direct_p_quality_structural_manifest_v1.json`, SHA-256 `6d20541560b726ed2409cd05902e5105874ac72e113af0c3812f029130df310f` |
| Phase80 MTV-A main summary | sealed run-1 summary, SHA-256 `ff0e1be40bd12f5fabd9fa8e0571e61049e1e23c34c347618e35f9b68a5f5181` |
| Phase80 LAX-T main summary | sealed run-1 summary, SHA-256 `599a399debe0800ed4fc794b3b39ffb31e384b233a3fe08338a60f4176b21a9a` |
| Official GNSS-only staging source | `output/reproducibility-cache/gsdc2023/fgo_gnss.m`, SHA-256 `5368e4056f70f448b728792c5e4c124b7f2afc1e00653492b807083bd3ccf0c3` |
| Official GNSS+IMU staging source | `output/reproducibility-cache/gsdc2023/fgo_gnss_imu.m`, SHA-256 `c70090ccb8b27fc8ac7fd2929e2f995a14cfe7f089bb1fef067370e22051c3e3` |
| Main graph backend source | `src/algorithms/fgo_gtsam_backend.cpp`, SHA-256 `cafefea8c0a982a58dd860468f433176bab055690a3f41c996882adc5c57500a` |
| Main C0/D/LM support source | `src/algorithms/fgo_gtsam_internal.hpp`, SHA-256 `2e2b6ac4e9e01d1c82b423774c53512d8217e4f04b3a2aeb00b99ea018aeaea8` |
| FGO public result/diagnostics | `include/libgnss++/algorithms/fgo.hpp`, SHA-256 `56d4dffb567cbc0b9fd96e2cbeef6dae080aadc510601145396de0cf39db4af3` |
| Native entry point | `apps/native/gnss_fgo_imu_no_base.cpp`, SHA-256 `524a8c0e4703a0ea533a21a4775e82719abaab2d2229a725ccf272cf2ab5430a` |
| Pinned GTSAM LM source | `output/reproducibility-cache/gtsam-4.3a/gtsam/nonlinear/LevenbergMarquardtOptimizer.cpp`, SHA-256 `834f732ac4ad4130b2a719a3181e5dfcd2ecdf584ece6f0c76bdf5f390e10cbc` |
| Official CCDD source | `output/reproducibility-cache/gtsam_gnss/src/ClockFactor_CCDD.h`, SHA-256 `7c174217218aa0059155e1e5a7182c69d56a3cd389d0ebd95c242d6d8c65b9fc` |

The Phase95 sealed result records four historical native invocations and zero
truth/MAT/Kaggle/base/precomputed-coordinate/accuracy activity. Those counts
belong to Phase95; this Phase96 audit adds no invocation and does not reopen
the input files behind them.

## Route evidence

The following values are copied from the sealed Phase95 telemetry. They are
costs, counts, and predicates only; no solution coordinate or raw observation
is published here.

| Route | GNSS-first accepted / cost (initial -> final) | GNSS-first D coverage | Main C0/D factors | Main accepted / cost (initial -> final) | Main lambda / attempts | C/D proxy | Main structural outcome |
|---|---:|---:|---:|---:|---:|---:|---|
| MTV-A | `167`; `5618000.786274777 -> 21298.114523521894` | `2159/2159`, finite, exact-key `true` | `2158` | `0`; `79358354.39651252 -> 79358354.39651252` | `1e-5 -> 1e5` / `10` | `1.9999999980791472` | position/clock/velocity/D full and finite; all positions earth-valid; coverage-contract fail-closed |
| LAX-T | `262`; `6419784.856579111 -> 12818.288538454106` | `1466/1466`, finite, exact-key `true` | `1465` | `0`; `166205998.11910567 -> 166205998.11910567` | `1e-5 -> 1e5` / `10` | `1.9999999980209395` | position/clock/velocity/D full and finite; all positions earth-valid; coverage-contract fail-closed |

For both routes, Phase95 reports `active_solve_attempted=true`, finite main
costs, `terminal_branch="no_progress_unclassified"`, and
`termination_trace_complete=false`. It reports empty exception type/message.
The equality of the printed costs is a sealed fact. The final lambda matching
the reported maximum is also a fact, but it is not substituted for the sealed
`no_progress_unclassified` branch because the trace was incomplete.

### Phase80 comparison

The sealed Phase80 no-C0D reference summaries show the following historical
main solves for the same two route identities:

| Route | Graph factors / values / IMU intervals | Iterations | Main cost (initial -> final) | Converged |
|---|---:|---:|---:|---:|
| MTV-A | `143414 / 10799 / 2158` | `12` | `155976340.16237149 -> 28128.450524519692` | `true` |
| LAX-T | `88025 / 7333 / 1465` | `12` | `190528071.24200463 -> 22206.857308543094` | `true` |

This is a useful sealed behavioral contrast, not a claim that Phase80 is a
single-variable ablation of every Phase95 configuration field. It supports
investigating the C0/D-enabled main solve while leaving the exact rejection
mechanism unresolved.

### Separate classification of the excluded Phase95 routes

The following sealed facts explain why MTV-H and MTV-U are not pooled with the
two successful-handoff routes and are not part of the candidate freeze:

| Route | Earliest sealed boundary | Evidence | Classification |
|---|---|---|---|
| MTV-H (`2021-08-24-20-32-us-ca-mtv-h/pixel5`) | GNSS-first backend admission | retained epochs `1247`, retained undifferenced Doppler factors `0`, eligible C0/D pairs `1041`, D initializer `1247/1247` finite; guard failed on `source_clock_c0d_problem_path_unavailable` and `gnss_first_problem.undifferenced_doppler_factors.empty()` | No GNSS-first solve or main handoff occurred. This is an admission/data-population failure, not the in-scope main all-trial-rejection case. |
| MTV-U (`2023-03-08-21-34-us-ca-mtv-u/pixel5`) | Main coverage contract after handoff and main solve | GNSS-first accepted `1000` with strict finite progress; main accepted `0`, cost unchanged at `15619537525702.566`, C0/D factors `671`, D and velocity `720/720` finite, but earth-valid positions `564/720` (`156` out of Earth) | Main no-progress is present, but an independent position/earth-validity failure is also present. It is excluded to avoid mixing that failure mode into the MTV-A/LAX-T diagnosis. |

The MTV-H `0` Doppler count and MTV-U `564/720` earth-valid count are copied
from sealed Phase95 telemetry. They are not inferred from input files and do
not authorize opening or rerunning either route.

## Source/control-flow audit

### Main graph construction and C0/D contract

At `src/algorithms/fgo_gtsam_backend.cpp:40-51`, the backend selects the
Pose3/IMU or Point3/velocity path and marks the C0/D problem path. The C0/D
configuration guard at `:63-68` rejects an invalid candidate rather than
falling back to the legacy clock row. The meter-state and exact handoff
preconditions are checked at `:70-109`.

The main graph inserts the existing observation and state families in the
same solve. Relevant construction boundaries are:

- base clock and ISB state insertion at `:596-643`;
- undifferenced pseudorange/code factors at `:692-840`;
- undifferenced Doppler/velocity and D factors at `:842-880`;
- Pose3/position motion factors at `:1080-1097`;
- clock motion and eligible source C0/D rows at `:1099-1195`;
- optional position, clock, signal-bias, ionosphere, velocity, and D priors
  at `:1272-1355`.

The source C0/D insertion uses the official row and ordinary meter sigma
`0.1 m` at `:1148-1158`; the code records only aggregate factor count and a
two-column whitened norm proxy (`:1159-1183`). It records aggregate graph
factor/value counts at `:1357-1358`, then only aggregate nonlinear
`graph.error(initial)` and `graph.error(optimized)` at `:1373-1392`.

The official cached source factor at
`output/reproducibility-cache/gtsam_gnss/src/ClockFactor_CCDD.h:47-61` is

```text
error = (C2-C1) - (D1+D2)*dt/2
Jacobian = [-1, +1, -dt/2, -dt/2]
```

The class receives a noise model but does not select a sigma. The active
backend supplies the frozen ordinary sigma `0.1 m`; no unit, equation, sigma,
filter, or LM change is indicated by this audit.

### GNSS-first to main handoff boundary

The native entry point configures the GNSS-first Point3/velocity graph at
`apps/native/gnss_fgo_imu_no_base.cpp:5652-5700`, calls its optimizer at
`:5725-5792`, and records the returned staged telemetry at `:5793-5809`.
Exact retained-key and optimized-D checks run at `:5810-5854`; only after
those checks does the app copy same-run position/clock state and hand off the
optimized D vector at `:5905-5932`.

The main optimizer is called at `:6012-6017`. Its result is summarized before
the Phase93 structural gate at `:6066-6120`. The gate requires, among other
predicates, nonempty full finite D/position-clock/velocity coverage, an active
C0/D solve, accepted outer iterations greater than zero, finite costs, and
strict final-cost decrease. Therefore the Phase95 records prove that the
in-scope routes passed the handoff predicates and reached the main C0/D solve;
they do not prove a particular numerical C/D value or a particular LM
rejection reason.

### Official source staging, state keys, and ordering

The official `fgo_gnss.m` source stages GNSS first. At
`output/reproducibility-cache/gsdc2023/fgo_gnss.m:34-47`, it initializes or
loads position/velocity/clock/drift, with `clkest` and `dclkest` coming from
the GNSS-only result on the non-initial path. It inserts per-epoch `x`, `v`,
`c`, and `d` states and their (infinite-sigma) initial priors at `:107-120`,
then adds pseudorange and Doppler factors at `:123-152`, motion and CCDD
clock factors at `:154-178`, and optional TDCP factors at `:179-201`. The
single LM optimizer is called at `:203-218`, and both `clkest` and `dclkest`
are exported at `:220-224` and saved with the GNSS result at `:251-253`.

The official `fgo_gnss_imu.m` source consumes that GNSS-first result at
`:40-60`: it loads `posest`, `velest`, `clkest`, and `dclkest`, derives the
initial attitude from velocity, and retains the prior IMU attitude when the
reset policy is off. It inserts `p`, `x`, `v`, `c`, `d`, and zero `b` states in
that order at `:169-187`; adds the Pose3-to-Point3 and pseudorange/Doppler
families at `:189-221`; adds motion/CCDD and IMU preintegration/bias factors
at `:252-300`; then TDCP at `:301-323`. It optimizes once at `:325-340` and
exports position, velocity, C, D, and IMU bias at `:342-356`.

The native key helpers preserve the same semantic order: `x` position,
`v` velocity, `c` per-epoch base clock, `i` global ISB, `d` per-epoch drift,
`p` as the Pose3 value held in the position slot on the IMU path, and `b`
per-epoch IMU bias (`src/algorithms/fgo_gtsam_internal.hpp:528-555`). In the
native Phase93/95 meter-state selector, C and ISB are metres, D is metres per
second, `dt` is seconds, and CCDD residual/sigma are metres; the legacy
selector retains its seconds clock convention. The custom factor argument
order at `:965-1003` is `[C1,C2,D1,D2]`, with Jacobian
`[-1,+1,-dt/2,-dt/2]` after the meter-state scale is applied. No key-order or
unit mismatch is exposed by the sealed handoff predicates; the unresolved
issue is main numerical progress after that handoff.

The native IMU initializer follows the same staged boundary: same-run
GNSS-first position/clock and optimized D are checked and copied before the
main graph (`apps/native/gnss_fgo_imu_no_base.cpp:5810-5932`), while the IMU
velocity/attitude/bias state construction and CombinedImuFactor insertion are
in `src/algorithms/fgo_gtsam_backend.cpp:418-583`. The main graph therefore
receives the intended state families and exact handoff, not a raw/zero/WLS D
fallback. This is a static consistency finding, not a claim about any
unpublished state values.

### Static factor-family and Jacobian map

The requested family map is structurally identifiable even though its numeric
initial costs and norms are not serialized:

| Family | Native construction / key blocks | Jacobian or residual contract |
|---|---|---|
| GNSS code | `fgo_gtsam_backend.cpp:692-840`; position `x` or Pose3 `p`, base `c`, optional ISB/signal-bias/ionosphere | range plus clock/optional bias minus measurement; meter-state clock derivative is the explicit state scale |
| GNSS Doppler | `:842-880`; velocity `v` and drift `d` | receiver LOS velocity plus clock-rate term against Doppler residual; D is `[m/s]` |
| IMU/preintegration/bias | `:418-583`; Pose3 `p`, velocity `v`, bias `b` | CombinedImuFactor over adjacent `p/v/b`; dropout uses existing velocity and bias continuity factors |
| Priors | `:547-565`, `:1272-1355`; first-state pose/velocity/bias and optional position/clock/signal/ionosphere/velocity/D priors | existing configured noise models; no new prior is introduced by this audit |
| Clock CCDD | `:1099-1185`; `c_i,c_{i+1},d_i,d_{i+1}` | source-exact CCDD row in `[C1,C2,D1,D2]` order, ordinary sigma `0.1 m` in meter mode |

This map does not imply that any family dominates the cost or normal matrix.
Only an observation pass over the already-built graph can establish those
numeric contributions. Such an observation pass is the single diagnostic
candidate frozen below; it must not create a counterfactual graph or solve.

### What the active LM actually computes

The pinned GTSAM source linearizes once per outer iteration and computes
`hessianDiagonal()` for diagonal damping at
`LevenbergMarquardtOptimizer.cpp:273-305`. In the exact `tryLambda` path:

- the damped system is built at `:141-143`;
- `solve` catches `IndeterminantLinearSystemException` only at `:153-160`;
- old/new linearized errors and `linearizedCostChange` are computed at
  `:168-177`;
- the tentative nonlinear error and actual `costChange` are computed at
  `:187-200`;
- `modelFidelity` and successful-step acceptance are computed at `:201-208`;
- the small relative-cost-change stop is checked at `:212-220`;
- the existing SUMMARY line publishes only `cost`, `cost_change`, `lambda`,
  and the solve-success bit at `:224-241`;
- failed trials increase lambda and either retry or stop at the upper bound at
  `:243-269`.

The library wrapper at
`src/algorithms/fgo_gtsam_internal.hpp:95-249` is aggregate-only. Its
`Attempt` record has just `outer_iteration`, `new_error`, `cost_change`,
`lambda`, and `system_solved_successfully` (`:114-120`); `parseAttempts`
consumes the six SUMMARY columns (`:133-148`). `finalize` derives aggregate
counts and a branch label (`:151-207`), but it does not retain predicted or
linearized cost, model fidelity, rejection reason, variable key, or factor
family. The backend copies those aggregate fields only at
`fgo_gtsam_backend.cpp:1393-1435`.

## Requested evidence: what is and is not established

| Requested evidence | Sealed Phase95/Phase92/Phase80 surface | Pinned source capability | Audit conclusion |
|---|---|---|---|
| Main initial cost by GNSS code, GNSS Doppler, IMU/preintegration/bias, priors, and clock CCDD | Main total initial cost and C0/D factor count only; no family totals | Existing factor insertion sites and `factor.error(initial)` can be observed, but no family labels or totals are emitted | **Unavailable in sealed artifacts**; total cost is known, decomposition is not |
| Gradient norms by variable/family | No gradient fields | Initial `linearize()` creates the Gaussian graph, but no per-key/family gradient summary is serialized | **Unavailable** |
| Normal diagonal norms by variable/family | Only C/D whitened column maxima and their ratio; no normal diagonal | `hessianDiagonal()` exists for damping, then is discarded; no family/key publication | **Unavailable**; C/D proxy near `2` is not a full normal-matrix condition number |
| First ten trial predicted vs actual cost | Main `inner_lambda_attempts=10`, but no trial rows in the sealed Phase95 summary and `termination_trace_complete=false` | `linearizedCostChange`, `newError`, `costChange`, and `modelFidelity` are local in `tryLambda`; current six-column parser drops the first, third, and reason-level distinctions | **Unavailable** |
| Rejection reason per trial | No reason field; exception type/message empty | Source branches distinguish indeterminate solve, negative linearized change, nonlinear/model-fidelity rejection, small-change stop, and max-lambda stop, but current wrapper does not capture them | **Unavailable** |
| Nonfinite/indeterminate/linear-solver exceptions | Costs and exported structural states are finite; no exception type/message; no per-trial indeterminate count in the Phase95 route record | `IndeterminantLinearSystemException` is collapsed into a solve-success bit in the local trace; other exceptions reach the backend `std::exception` catch | No such event is proved, and no complete count/type is available |

The absence of a published field is not evidence that the corresponding
condition occurred. In particular, finite initial/final costs do not exclude a
nonfinite intermediate quantity that was not serialized, and the incomplete
trace does not identify the ten trial outcomes.

## Fact versus inference

### Facts

- Both in-scope GNSS-first solves attempted the active C0/D path, accepted
  many outer iterations, strictly decreased finite cost, and returned a full
  finite optimized-D sequence aligned to retained keys.
- Both main C0/D solves were attempted and returned finite structural output
  coverage, with C0/D factors present and meter-state parity enabled.
- Both main solves accepted zero outer iterations, kept equal initial/final
  costs, reached the reported maximum lambda after ten inner attempts, and
  were sealed with `no_progress_unclassified` plus an incomplete trace.
- No route-local Phase95 exception type/message was recorded for either route.
- Phase80's historical no-C0D main reference accepted twelve iterations and
  strictly reduced its aggregate cost on both in-scope routes.

### Inferences permitted by those facts

- The failure boundary is the main C0/D-enabled LM progress/termination path,
  after handoff and main graph construction. This is a stage localization,
  not a proof of one mathematical cause.
- The current C/D column proxy being approximately `2` shows that the two
  reported whitened column maxima have similar scale. It cannot establish
  rank, full conditioning, gradient alignment, factor-family dominance, or a
  linear solver failure.
- The difference from the Phase80 reference justifies collecting diagnostic
  evidence around the C0/D-enabled main graph; it does not justify changing
  C0/D sigma, LM damping, filters, priors, solver, or handoff semantics.

### Not established

The sealed artifacts do not establish whether the first ten trials had
negative linearized reductions, non-decreasing nonlinear costs, low model
fidelity, indeterminate solves, nonfinite intermediate values, or another
branch. Any one of those as the root cause would be speculation until the
requested telemetry exists.

## Candidate comparison and freeze boundary

Only one candidate is eligible for a Phase96 freeze:

**Source-aligned main C0/D diagnostic telemetry (opt-in, not implemented).**

The candidate must:

1. Apply only to the two in-scope Phase95 successful-handoff identities, MTV-A
   and LAX-T. MTV-H/U are excluded.
2. Keep the legacy selector-off path byte-for-byte behaviorally unchanged and
   keep the candidate default disabled. A selector may enable observation only;
   it must not authorize raw execution by itself.
3. At the existing main graph construction boundary, retain non-semantic
   family labels for already-created factors and report finite/nonfinite
   per-family initial error for exactly these buckets: GNSS code,
   GNSS Doppler, IMU/preintegration/bias, priors, clock CCDD, and `other`.
   The `other` bucket and a total/reconciliation field are required so the
   report cannot silently omit factors. No raw residual, coordinate, state
   vector, or solution row may be published.
4. At the one existing initial linearization, report only aggregate finite
   norms/counts by factor family and variable bucket (position, velocity,
   IMU-bias, clock-C, clock-ISB, clock-D, and `other`): whitened gradient
   norm and normal-diagonal summary. No additional optimizer or solve may be
   invoked, and no damping diagonal may be changed.
5. Instrument the exact pinned `tryLambda` locals for the first ten existing
   trials, without changing control flow: outer/trial index, lambda, old and
   new linearized cost, predicted/linearized reduction, tentative nonlinear
   cost, actual reduction, model fidelity when defined, solve status, and an
   exact source-branch rejection/termination category. Record whether a
   value is unavailable or nonfinite rather than substituting a number.
6. Record nonfinite classifications and exception counts/types at graph
   evaluation, linearization, solve, retract, and nonlinear-cost boundaries.
   Distinguish `IndeterminantLinearSystemException`, standard exceptions, and
   unknown exceptions. Preserve the existing fail-closed behavior.
7. Publish the diagnostic object on early failure and before any solution or
   accuracy lane. The object must be safe to emit when no `FGOResult` exists,
   and must not turn a failed route into a pass.
8. Keep official CCDD equation/Jacobian, meter C/ISB and metre-per-second D
   units, ordinary sigma `0.1 m`, existing filters, existing LM parameters,
   no-PDC/no-external-input policy, no fallback, and exact route policy
   unchanged.

The safe implementation boundary is observation at existing factor insertion,
linearization, and `tryLambda` locals. It is not a second graph evaluation
with a new optimizer, a counterfactual solve, a retry, a tuning sweep, or a
factor removal. This freeze does not authorize implementation tests against
raw routes; a later authorization would have to be separately sealed.

### Alternatives rejected, not frozen

- Removing or disabling C0/D to reproduce the Phase80 result would change the
  algorithm and would answer a different experiment.
- Changing sigma, LM/lambda limits, tolerances, priors, filters, or adding a
  retry/fallback/PDC bridge is prohibited by the phase boundary and has no
  supporting root-cause evidence.
- Rerunning either route, adding MTV-H/U, or reading truth/MAT/Kaggle data is
  outside this audit.

## Read accounting and release boundary

| Activity | Phase96 audit value |
|---|---:|
| Native solver invocations | `0` |
| Raw GNSS reads | `0` |
| Raw IMU reads | `0` |
| Broadcast navigation reads | `0` |
| Truth reads | `0` |
| MAT reads/generated | `0` |
| Kaggle/token access | `0` |
| Base RINEX reads | `0` |
| Precomputed-coordinate reads | `0` |
| Accuracy calculations/submission | `0` |
| Route reruns/fallbacks | `0` |

This audit and its separate freeze record authorize no raw run, no solution
publication, no accuracy evaluation, and no submission release.
