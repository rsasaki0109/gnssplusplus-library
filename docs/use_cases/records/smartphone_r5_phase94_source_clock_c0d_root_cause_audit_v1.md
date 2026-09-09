# Phase94 source-clock C0/D root-cause audit

- Execution label: `Luna Max`
- Audit status: `sealed-read-only-source-and-artifact-audit`
- Audit date: `2026-09-02`
- Scope: Phase93 sealed result `71f05e4`, its pinned runner and implementation, plus sealed Phase92/Phase80 evidence.
- Raw execution in this audit: **0**. No raw GNSS/IMU/navigation file was reopened, and no truth, MAT, Kaggle/token, base, precomputed-coordinate, or accuracy artifact was read.
- Source changes in this audit: **0**.

This record separates observations that are forced by the source/control flow from explanations that cannot be distinguished from the sealed Phase93 failure artifacts. It is a root-cause audit, not a promotion or accuracy result.

## Authority and evidence

| Item | Pin |
|---|---|
| Phase93 sealed structural result | `docs/use_cases/records/smartphone_r5_phase93_source_staging_clock_state_handoff_structural_result_v1.json`, SHA-256 `cc5eb2a56ee4239a9e25eca04d4573fd8fcfc0cc071f0cb3a507f33ba7f1ca59`, commit `71f05e4` |
| Phase93 structural result markdown | `docs/use_cases/records/smartphone_r5_phase93_source_staging_clock_state_handoff_structural_result_v1.md`, SHA-256 `8fcfa78d2054f9f19db977c344c232838c182fffee08e57f7f8fe8933953c870` |
| Phase93 execution wrapper | `apps/commands/benchmarks/gnss_smartphone_phase93_source_staging_clock_state_handoff_execute.py`, SHA-256 `96f841081788de1c3b7aea1b7983ff5f77bbf04a6cc69e63264e822791d9a920`, commit `b2a5d47` |
| Phase93 evaluator | `apps/commands/benchmarks/gnss_smartphone_phase93_source_staging_clock_state_handoff.py`, SHA-256 `0bf2113fc484a72e7bcea2765381ded93fa8df930c5f95e1585fc03c2185f3ed` |
| Phase93 implementation | commit `2613adb12fba80c468f8c6945ba360a126129f58`, binary pin from sealed result `99bdb24a06f6056eebd4dd0ceb969e4d7e14095ff7d9802c89676221a2232a82` |
| Phase93 source freeze | `docs/use_cases/records/smartphone_r5_phase93_source_staging_clock_state_handoff_freeze_v1.json`, SHA-256 `34ca3a2c6beecdb912aed81c469eca8a45d32e80bad69e9fc5fa12ea1b49ab88` |
| Phase92 structural result | `docs/use_cases/records/smartphone_r5_phase92_source_meter_clock_state_parity_structural_result_v1.json`, SHA-256 `8488306b3ec61d0360418de73fa1596271597b6971d77de676fcd279d7e1b01c` |
| Phase92 alignment/unit audit | `docs/use_cases/records/smartphone_r5_phase92_phase91_alignment_and_clock_isb_unit_audit_v1.md`, SHA-256 `45b15439063eaa823f786df15155c83b92e65939812e1d18698cf1812d0f5271` |
| Phase80 structural evidence | sealed artifact referenced by Phase93/Phase81 records; no raw input reopened |

The sealed Phase93 result records exactly four native invocations, one per route, with raw GNSS/IMU/broadcast navigation reads only. Its read accounting records zero truth, MAT, Kaggle/token, base, precomputed-coordinate, accuracy, and rerun activity. The preserved route stderr files are the only route-local execution evidence used below.

## Observed Phase93 outcome

| Route | Return | Preserved native evidence | Output/summary |
|---|---:|---|---|
| `2021-03-16-18-59-us-ca-mtv-a/pixel5` | `1` | `coverage=true`, `position_clock=true`, `velocity=true`, C0/D factor count `2158`, accepted outer iterations `0`, initial and final cost both `7.93584e+07` | No output rows; no summary |
| `2021-08-24-20-32-us-ca-mtv-h/pixel5` | `-6` | uncaught `std::invalid_argument`: `native source ClockFactor_CCDD C0/D requires direct observable quality, no PDC bridge, and an approved GTSAM batch path` | No output rows; no summary |
| `2022-04-01-18-22-us-ca-lax-t/pixel5` | `1` | `coverage=true`, `position_clock=true`, `velocity=true`, C0/D factor count `1465`, accepted outer iterations `0`, initial and final cost both `1.66206e+08` | No output rows; no summary |
| `2023-03-08-21-34-us-ca-mtv-u/pixel5` | `1` | `coverage=true`, `position_clock=false`, `velocity=true`, C0/D factor count `671`, accepted outer iterations `0`, initial and final cost both `1.56195e+13` | No output rows; no summary |

The numbers in this table are preserved native failure-line values, not reconstructed from raw data. The Phase93 result's `failure_evidence` marks all stage telemetry as not published because every route failed before the native summary publication point.

## 1. Route2 guard: exact argv and configuration flow

The sealed route2 command contains the following non-path argument vector, in this order:

```text
build/apps/gnss_fgo_imu_no_base
--dataset-id 2021-08-24-20-32-us-ca-mtv-h/pixel5
--android-gnss <manifest device_gnss.csv>
--android-imu <manifest device_imu.csv>
--nav <manifest brdc.nav>
--all-epochs
--android-raw-utc-keys
--android-raw-clock-only
--android-utc-wall-clock-fallback
--native-pdc-imu-tdcp-no-bridge
--native-source-direct-observable-quality
--native-source-clock-c0d-factor
--native-source-clock-c0d-meter-state-parity
--native-source-clock-c0d-active-solve-diagnostic
--native-source-clock-c0d-gnss-first-meter-state-handoff
--out <phase93 structural_output.csv>
--summary-json <phase93 summary.json>
```

The three angle-bracket values above are role placeholders for the exact manifest-pinned raw paths; this audit does not reopen them.

The CLI accepts this combination in `apps/native/gnss_fgo_imu_no_base.cpp:615-806`. In particular, it verifies direct observable quality, no PDC bridge, GTSAM, C0/D, meter-state parity, active-solve diagnostics, raw Android UTC keys, all epochs, and the Phase93 handoff selector. It rejects the raw-D-only selector, velocity-only/direct-WLS handoffs, legacy upstream quality, base/external inputs, and other optional candidates.

The main config is created at `apps/native/gnss_fgo_imu_no_base.cpp:4406-4426` and `:4511-4552`:

- GTSAM, Pose3, IMU, corrected undifferenced Doppler, direct observable quality, no PDC bridge;
- C0/D factor, meter-state parity, active solve diagnostics, and Phase93 handoff all enabled;
- the ordinary C0/D equation and sigma remain the frozen source contract.

For the first raw GNSS pass, the app copies that config and then makes the source-staging graph Point3 plus explicit velocity states at `:4817-4865`: `use_imu=false`, `use_pose3_state=false`, `use_velocity_states=true`, C0/D/meter/active/raw-D enabled, and the frozen GNSS-first iteration bound of `1000`. The first optimizer call is `gnss_first_processor.optimizeProblem(gnss_first_problem)` at `:4885`; IMU construction and the main optimizer call occur later at `:5055-5071`.

At `src/algorithms/fgo_gtsam_backend.cpp:40-68`, the backend computes:

```text
use_imu = use_pose3 && use_imu_config && problem.imu.valid && num_epochs >= 2
use_gnss_velocity_states = !use_imu && !use_pose3 && use_velocity_states
                            && !problem.undifferenced_doppler_factors.empty()
source_clock_c0d_problem_path = use_imu || use_gnss_velocity_states
```

The observed exception is the C0/D configuration guard at `:63-68`. The approved GNSS-first branch in `nativeSourceClockC0DBackendConfigurationAllowed`, `src/algorithms/fgo_gtsam_internal.hpp:928-949`, requires the handoff selector, Point3/no-IMU mode, velocity states, direct quality, undifferenced Doppler enabled, motion and clock-motion factors, no PDC bridge/fixed lag, and a nonempty phone identity.

Facts forced by this flow:

1. The exception is from the first GNSS-first optimizer call, not from output writing. That call is earlier than IMU construction and the main optimizer.
2. The CLI/config spelling itself is not the rejection: the CLI validation had already accepted it, and the exception is the backend's runtime guard.
3. With the static route configuration, the only route-data-dependent terms in the guard are the constructed GNSS-first problem's `undifferenced_doppler_factors.empty()` and `num_epochs < 2`. The route2 failure occurs before a result exists, so no native count was serialized.
4. The most likely branch is an empty retained undifferenced-D population after the direct-quality/geometry/epoch filtering. This is an inference, not a sealed count. The alternative (or conjunction) is fewer than two retained problem epochs. The sealed stderr cannot distinguish these disjuncts, and the audit does not reopen raw input to do so.

The Phase81 reclassification's large route2 D counts are not proof against this inference: that sealed artifact came from a different Phase80 candidate command with quality-anchor, signal-bias, and base-related switches. It is useful as historical context, but it is not an exact Phase93 problem-builder trace.

## 2. Why GNSS-first telemetry is `n/a` for every route

The native report object does receive GNSS-first fields at `apps/native/gnss_fgo_imu_no_base.cpp:4887-4903`, and `makeSummary` would serialize them at `:3800-3921`. However, that serialization is reachable only after the main graph, output validation, and final summary call at `:5400-5409`.

- Route2 throws in `FGOProcessor::optimizeProblem` before returning a `FGOResult`; the app has no exception boundary around that first call, so it terminates before any `imu_report` fields can be written.
- Routes1/3/4 return at the Phase93 optimized-D/progress contract at `:5099-5144`, before output CSV and `makeSummary`.
- The runner `apps/commands/benchmarks/gnss_smartphone_phase93_source_staging_clock_state_handoff_execute.py:611-650` first requires a zero return code, then reads a native summary. A nonzero route is recorded as fail-closed and has no parsed stage diagnostics.

Therefore `gnss_first: n/a` is a publication/control-flow fact, not evidence that GNSS-first was inactive on routes1/3/4. It is also not evidence of a GNSS-first numerical failure on route2; route2 failed before a result was available.

## 3. Earliest evidence for route1/3/4 main `accepted=0`

The observed message is emitted only by the final Phase93 contract at `:5119-5144`. The source order gives a stronger stage boundary than the sparse stderr:

- If exact retained-key handoff validation failed, the app would set `phase91_handoff_ok=false` and return at `:4961-4971` with `GNSS-first initialization unavailable` / `native GNSS-first candidate failed closed`.
- For the Phase93 selector, that same pre-main gate additionally requires C0/D enabled, meter parity enabled, a nonzero GNSS-first C0/D factor count, finite active-solve costs, accepted GNSS-first outer iterations greater than zero, finite initial/final cost, and strict GNSS-first cost decrease (`:4923-4947`).
- `deriveGnssFirstVelocities` must also pass before the app proceeds (`:4950-4960`).
- If IMU construction failed, the candidate returns at `:5062-5068`; if the main result were empty, non-converged, or had zero IMU intervals, it returns at `:5070-5084`.

Thus, for routes1/3/4, the earliest known failure is after the GNSS-first optimized C/D handoff, after IMU graph construction, and after a normal main `FGOResult` return. The native failure line proves the main C0/D active solve had zero accepted outer iterations and unchanged finite initial/final costs. It does not publish the active branch reason, lambda trace, or conditioning proxy.

| Route | Handoff reached? | GNSS-first active solve passed pre-main gate? | Main C0/D evidence | Main output coverage evidence |
|---|---|---|---|---|
| MTV-a | Yes, by source-order implication | Yes, including exact-key validation and optimized-D export validation | `2158` factors; accepted `0`; `79358400 == 79358400` | D full/finite, velocity full/finite, position+clock full/finite |
| LAX-t | Yes, by source-order implication | Yes, including exact-key validation and optimized-D export validation | `1465` factors; accepted `0`; `166206000 == 166206000` | D full/finite, velocity full/finite, position+clock full/finite |
| MTV-u | Yes, by source-order implication | Yes, including exact-key validation and optimized-D export validation | `671` factors; accepted `0`; `15619500000000 == 15619500000000` | D full/finite, velocity full/finite; position+clock predicate false |

“C/D values” are therefore established only as structural properties: the optimized GNSS-first D vector was accepted for handoff and the route1/3/4 main result had finite full D coverage according to the native failure evidence. No numeric C or D vector was published. The main initial graph had a finite error (the printed initial cost), but no persisted artifact exposes every initial state value.

The Phase92 sealed audit supplies the relevant comparison: the meter-state C0/D main graph had full raw-D coverage but zero accepted outer iterations and unchanged costs on its published MTV-a/LAX-t summaries, with conditioning proxy near `2`; the no-C0D Phase80 main graph accepted `12` iterations and strictly decreased cost on all four routes. This isolates the remaining failure to the C0/D-enabled main solve/progress path, but does not justify changing sigma, lambda, or the optimizer.

## 4. Route4 `position_clock=false`

The exact predicate is `apps/native/gnss_fgo_imu_no_base.cpp:5105-5112`:

```text
result.solution.solutions.size() == problem.epochs.size()
&& every solution has earth-valid finite ECEF position
&& every receiver_clock_bias is finite
```

The sealed line proves this conjunction was false while the optimized-D and velocity conjunctions were true. Because the app returns immediately at `:5129-5144`, before output alignment and summary publication, the sealed artifacts cannot distinguish a solution-vector length mismatch from one or more invalid/out-of-Earth/nonfinite position-clock entries. The `671` C0/D factor count records factor eligibility and is not, by itself, the cause of the position-clock predicate. Any more specific route4 claim would be speculation.

## Candidate comparison and decision boundary

Three bounded candidates were considered:

| Candidate | What it would change | Evidence fit | Decision |
|---|---|---|---|
| A. Stage-local C0/D admission and failure telemetry | Add an opt-in, raw-free stage report/preflight that records GNSS-first retained epoch count, undifferenced-D count, each backend-admission boolean, exception branch, handoff coverage, and every main progress/coverage subpredicate before fail-closed return. Do not bypass any guard or alter the graph. | Directly closes the evidence gap responsible for route2 and route4 `n/a`; preserves source equation, units, sigma, and optimizer. | **Freeze** |
| B. Retain sparse epochs or relax the direct-quality D screen | Change which epochs/D rows reach the GNSS-first graph to avoid the route2 admission failure. | Could affect the suspected route2 disjunct, but the exact disjunct is not known; it changes measurement inclusion and has no sealed proof as the source-exact fix. | Do not freeze |
| C. Change LM/lambda/sigma, add a prior, or add a fallback/bridge | Attempt to force main progress or bypass a failed stage. | Not supported by the evidence; conflicts with the no-tuning, no-PDC, and fail-closed contract. | Reject |

Candidate A is the only defensible next step under the present evidence. It is diagnostic-only: it is not a convergence claim, does not authorize raw execution, and must not convert route2's guard failure into a pass. The separate freeze record names the exact implementation boundary; no code is changed by this audit.

## Read accounting and release boundary

This audit performed no native solver invocation and no raw input read. It read only pinned source text, sealed Phase93 stderr/result artifacts, and sealed Phase92/Phase80 evidence. Truth/MAT/Kaggle/token/base/precomputed-coordinate/accuracy reads remain zero. No raw rerun, accuracy evaluation, promotion, or submission release is authorized by this record.
