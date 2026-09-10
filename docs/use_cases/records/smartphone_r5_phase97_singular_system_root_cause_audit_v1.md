# Phase97 singular-system root-cause audit

- Execution label: `Luna Max`
- Audit status: `sealed-read-only-static-and-artifact-audit`
- Audit date: `2026-09-03`
- Scope: the two Phase96 primary routes whose GNSS-first C0/D solve reached a
  finite, strict-progress, exact-key D handoff: `2021-03-16-18-59-us-ca-mtv-a/pixel5`
  (MTV-A) and `2022-04-01-18-22-us-ca-lax-t/pixel5` (LAX-T).
- Phase80 is used only as a sealed no-C0D behavioral/source comparison. It is
  not treated as a one-variable ablation of Phase96.
- Raw input files were not opened. Truth, MAT, Kaggle/token, base,
  precomputed-coordinate, solution, and accuracy artifacts were not opened or
  generated. No native solver was rerun.

This audit distinguishes sealed facts from static inferences. It does not
claim convergence, accuracy, a valid solution, a promotion, or a root cause
that the available telemetry cannot prove.

## Decision

No unique algorithmic correction is established. The earliest reliable
failure boundary is the first main-stage damped linear-system solve:

1. On both routes, GNSS-first accepted finite strict-progress steps and
   exported every retained D state with exact key alignment.
2. The same-run main C0/D graph was built and its initial linearization was
   observed. Main state coverage was finite and complete, including C, D,
   velocity, and position predicates.
3. The pinned LM path reported ten unsuccessful trials. The first nine were
   recorded as an indeterminate linear system; the tenth reached the maximum
   lambda stop. No outer step was accepted and the reported nonlinear cost did
   not decrease.
4. The sidecar does not contain the actual exception's
   `IndeterminantLinearSystemException::nearbyVariable()` key. The current
   wrapper catches the pinned LM trace's missing solve delta and synthesizes a
   type/category record after the pinned optimizer has already discarded the
   exception object. Therefore no particular key, factor, disconnected
   component, zero column, or gauge is proven to be the cause.

The sole defensible next step is one opt-in, read-only diagnostic candidate:
exact active-key/value descriptors, factor incidence and connected-component
accounting, keyed zero/near-zero column and normal-diagonal buckets, and the
actual nearby key when the solver boundary makes it available. It must observe
the existing graph and first ten existing LM trials without changing the
algorithm or invoking another solve. This candidate is frozen in a separate
JSON commit; it is not implemented or authorized to run by this audit.

## Read accounting and authority

### This Phase97 audit

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

Only source files and already-sealed Phase80/Phase96 records were inspected.
The Phase80 records themselves document a historical base-RINEX compensation
configuration; that provenance is retained below as a comparability warning,
not as a Phase97 input or read.

### Pinned records and source hashes

| Item | Authority |
|---|---|
| Phase96 sealed result | `6039aa35b948bfd8f73c1398ce592dc7c9914360`; `docs/use_cases/records/smartphone_r5_phase96_main_c0d_diagnostic_structural_result_v1.json`; SHA-256 `e51628fceba06be72026fdef358f60f183a7a837b4b78b3851f934a62f4d7955` |
| Phase96 diagnostic implementation | commit `bcdcd952943623650cdf77410c17e9ec87f45d7f` |
| Phase96 telemetry freeze | `docs/use_cases/records/smartphone_r5_phase96_main_c0d_diagnostic_telemetry_freeze_v1.json`; SHA-256 `9605f3ce32ad08ae3f17254834df50a65cd93918cee593b487a02ad7b97ebc65` |
| Phase80 sealed source | commit `f28aebaf1b41df3ee5e363a8ad94987247321048` |
| Phase80 structural manifest | `docs/use_cases/records/smartphone_r5_phase80_source_exact_direct_p_quality_structural_manifest_v1.json`; SHA-256 `6d20541560b726ed2409cd05902e5105874ac72e113af0c3812f029130df310f` |
| Phase80 MTV-A summary | SHA-256 `ff0e1be40bd12f5fabd9fa8e0571e61049e1e23c34c347618e35f9b68a5f5181` |
| Phase80 LAX-T summary | SHA-256 `599a399debe0800ed4fc794b3b39ffb31e384b233a3fe08338a60f4176b21a9a` |
| Native graph backend | `src/algorithms/fgo_gtsam_backend.cpp`; SHA-256 `077afd8df8745fe0c01fc52d35c33ac5908b44185e27d55cdb8c19eac7f6cdc7` |
| Native graph/key and Phase96 sidecar support | `src/algorithms/fgo_gtsam_internal.hpp`; SHA-256 `7691fa988b6236c14152cd7d9d66aba66fc2879df2861174a5cbe87b3bbef1cb` |
| Public FGO diagnostics | `include/libgnss++/algorithms/fgo.hpp`; SHA-256 `623dc927266bc82e944b2e9965f3dfa35a2b749c892beeae9ae39fd71d83feef` |
| Pinned LM | `output/reproducibility-cache/gtsam-4.3a/gtsam/nonlinear/LevenbergMarquardtOptimizer.cpp`; SHA-256 `834f732ac4ad4130b2a719a3181e5dfcd2ecdf584ece6f0c76bdf5f390e10cbc` |
| Pinned indeterminate exception | `output/reproducibility-cache/gtsam-4.3a/gtsam/linear/linearExceptions.h`; SHA-256 `15d3c2e1ae816f688e2d90cd37212cb7559330862cf6dd061462e967a2ffa418` |
| Pinned Symbol | `output/reproducibility-cache/gtsam-4.3a/gtsam/inference/Symbol.h`; SHA-256 `a0ad984b092195e51a62cd2ea994cedb9698d8a1750ab6086fe9ec0ae438faaa` |
| Pinned Values implementation | `output/reproducibility-cache/gtsam-4.3a/gtsam/nonlinear/Values.cpp`; SHA-256 `ab72606c6545bdaf017cf09c6748dfc3d7e530068ab72e4002cc1c681da40dee` |
| Official GNSS staging | `output/reproducibility-cache/gsdc2023/fgo_gnss.m`; SHA-256 `5368e4056f70f448b728792c5e4c124b7f2afc1e00653492b807083bd3ccf0c3` |
| Official GNSS+IMU staging | `output/reproducibility-cache/gsdc2023/fgo_gnss_imu.m`; SHA-256 `c70090ccb8b27fc8ac7fd2929e2f995a14cfe7f089bb1fef067370e22051c3e3` |
| Official CCDD source | `output/reproducibility-cache/gtsam_gnss/src/ClockFactor_CCDD.h`; SHA-256 `7c174217218aa0059155e1e5a7182c69d56a3cd389d0ebd95c242d6d8c65b9fc` |

## Sealed Phase96 evidence

### Route-level boundary

| Route | GNSS-first accepted / cost | D handoff | Main graph factors / Values | Main initial → final cost | Main accepted | LM trials / lambda | Result |
|---|---:|---|---:|---:|---:|---|---|
| MTV-A | `167`; `5618000.786274777 → 21298.114523521894` | `2159/2159`, finite, exact-key | `120093 / 10798` | `79358354.39651252 → 79358354.39651252` | `0` | `10`; `1e-5 → 1e5` | finite structural output, fail-closed main progress gate |
| LAX-T | `262`; `6419784.856579111 → 12818.288538454106` | `1466/1466`, finite, exact-key | `61082 / 7332` | `166205998.11910567 → 166205998.11910567` | `0` | `10`; `1e-5 → 1e5` | finite structural output, fail-closed main progress gate |

Both routes have `active_solve_attempted=true`, finite main initial/final
costs, a complete first-ten trace, and a `maximum_lambda` terminal branch in
the Phase96 sidecar. The C/D whitened-column conditioning proxies are
`1.9999999980791472` (MTV-A) and `1.9999999980209395` (LAX-T). That proxy is
only the ratio of two aggregate CCDD column maxima; it is not a rank or
condition-number certificate for the complete normal matrix.

### Main factor-family accounting

These are the exact Phase96 sidecar aggregates. The family classifier is a
read-only type-name classification; in particular the IMU family includes the
first ConstantBias prior, while `other` contains the native TDCP type.

| Route | Family | Factors | Initial cost |
|---|---|---:|---:|
| MTV-A | priors | `4320` | `20.947145425123253` |
| MTV-A | imu_preintegration_bias | `2159` | `71870632.76589131` |
| MTV-A | gnss_code | `57281` | `2867835.4121781313` |
| MTV-A | gnss_doppler | `20748` | `184918.08783213163` |
| MTV-A | other (TDCP in this graph) | `31269` | `4434607.865105846` |
| MTV-A | motion | `2158` | `335.2447769589361` |
| MTV-A | clock_ccdd | `2158` | `4.073586009709632` |
| LAX-T | priors | `2934` | `79.64480174936614` |
| LAX-T | imu_preintegration_bias | `1466` | `161869880.55325297` |
| LAX-T | gnss_code | `30798` | `1283708.6345118012` |
| LAX-T | gnss_doppler | `8942` | `298736.64600214903` |
| LAX-T | other (TDCP in this graph) | `14012` | `2753401.9343244997` |
| LAX-T | motion | `1465` | `188.05748984930577` |
| LAX-T | clock_ccdd | `1465` | `2.64872289226184` |

Each route's family-cost sum exactly reconciles to its graph initial cost in
the sealed JSON. All listed factor errors were finite. This proves that the
initial graph evaluation and the recorded initial linearization reached the
sidecar, but it does not prove that the assembled linear system is full rank.

### First ten trials and exception boundary

The two routes have the same pinned trace shape:

| Trial indices | Lambda sequence | Linear solve status | Candidate/cost fields | Rejection/termination |
|---|---|---|---|---|
| `0..8` | `1e-5, 1e-4, ..., 1e3` | `indeterminate_linear_system` | candidate false; predicted and actual reductions unavailable | `indeterminate_linear_system` |
| `9` | `1e4` | `indeterminate_linear_system` | candidate false; predicted and actual reductions unavailable | `maximum_lambda_stop` |

The sealed exception aggregate is count `10`, stage `lm_trial`, classification
`indeterminate_linear_system`, type label
`IndeterminantLinearSystemException`, message
`pinned LM trial did not solve the damped linear system`. This label is
consistent with the pinned LM catch path, but the actual exception key is not
present. It must not be interpreted as a nearby-key diagnosis.

The pinned source explains the information loss:

- `LevenbergMarquardtOptimizer::tryLambda` builds the damped system, calls
  `solve`, and catches `IndeterminantLinearSystemException` at lines
  `141–160` without binding the exception object.
- All predicted/actual cost and model-fidelity values are computed only after
  a successful solve at lines `162–220`.
- The current wrapper's `PendingPhase96Trial` marks a trial indeterminate when
  no `linear delta norm` line is parsed, and `finalize` emits the synthetic
  type/message at `src/algorithms/fgo_gtsam_internal.hpp:397–415`.
- The pinned exception class stores `j_` and exposes
  `nearbyVariable()` (`linearExceptions.h:94–100`), but the current catch does
  not retain it. GTSAM documents that the nearby key is where the issue was
  discovered, not necessarily where it originated, and that elimination
  ordering affects it.

Thus the earliest *observed* mathematical boundary is “damped linear system
did not solve”; the source does not let this audit distinguish rank
deficiency, indefiniteness, numerical conditioning, or a key discovered late
in elimination.

## Key namespace and Values audit

### Native key construction

The native C++ helpers at
`src/algorithms/fgo_gtsam_internal.hpp:804–831` use disjoint Symbol
characters:

| Symbol | Helper | Native type/role | Phase96 active? |
|---|---|---|---|
| `x` | `positionKey(epoch)` | per-epoch `Point3` in Point3 path or `Pose3` in IMU path | yes |
| `v` | `velocityKey(epoch)` | per-epoch `Vector3` velocity | yes |
| `b` | `biasKey(epoch)` | per-epoch `imuBias::ConstantBias` | yes |
| `c` | `clockKey(epoch)` | per-epoch scalar receiver clock C | yes |
| `d` | `dopplerClockDriftKey(epoch)` | per-epoch scalar receiver clock drift D | yes |
| `i` | `isbKey(ordinal)` | one global scalar ISB per non-GPS clock group | yes, route-dependent count |
| `f` | `signalBiasKey(ordinal)` | static receiver signal-bias scalar | no |
| `j` | `residualIonosphereKey(epoch)` | per-epoch ionosphere scalar | no |
| `a` | `ambiguityKey(index)` | ambiguity scalar | no |
| `z` | `dummyAmbiguityKey()` | shared dummy ambiguity scalar | no |

Pinned `Symbol::operator==` compares both character and index
(`Symbol.h:90–103`). Therefore `x0`, `v0`, `b0`, `c0`, and `d0` are distinct
keys; equal epoch indices across families do not collide. Pinned `Values::insert`
uses `emplace` and throws `ValuesKeyAlreadyExists` on a duplicate
(`Values.cpp:155–159`). The sealed Phase96 graph was observed and did not
report that exception. Static evidence therefore rejects a namespace
collision as the presently supported cause, while a future key dump should
still verify duplicate insertion attempts and value types explicitly.

The official MATLAB staging source uses a separate `p` Pose3 key in
`fgo_gnss_imu.m`, while the native C++ IMU backend stores the Pose3 in its
`positionKey` (`x`) slot. This source-language distinction is important: the
Phase80 and Phase96 native Values counts cannot be compared as if they were a
literal dump of the MATLAB `p/x/v/c/d/b` namespace.

### Values cardinality and derived tangent dimensions

For the active native Phase96 configuration, the insertion code creates five
per-epoch values `x,v,b,c,d`, plus global `i` values. No `f,j,a,z` state is
enabled by the Phase96 route configuration. The sealed counts imply:

| Route | Epochs N | `5N` per-epoch keys | Values count | Derived ISB count | Derived tangent dimension |
|---|---:|---:|---:|---:|---:|
| MTV-A | `2159` | `10795` | `10798` | `3` | `17N + 3 = 36706` |
| LAX-T | `1466` | `7330` | `7332` | `2` | `17N + 2 = 24924` |

The dimensions are derived, not serialized fields: `Pose3` contributes 6,
`Vector3` 3, `ConstantBias` 6, and each scalar C/D/ISB 1, so each epoch is
17 tangent dimensions. Pinned `Values::dim()` sums the inserted value
dimensions (`Values.cpp:242–248`). The record contains only
`graph_value_count`, not `Values::dim()`, so these dimensions are an audit
calculation rather than a sealed runtime measurement.

## Factor incidence and connectivity

The current backend's insertion sites define the following key blocks:

| Family | Source boundary | Key incidence in the active Phase96 IMU graph |
|---|---|---|
| GNSS code | `fgo_gtsam_backend.cpp:703–851` | `x_i,c_i`, and `i_k` for non-GPS code |
| GNSS Doppler | `:853–881` | `v_i,d_i` |
| IMU/preintegration | `:578–593` | `x_i,v_i,x_{i+1},v_{i+1},b_i,b_{i+1}`; the first bias prior is classified into this family |
| TDCP (`other`) | `:923–960` | `x_i,c_i,x_{i+1},c_{i+1}` |
| Position motion | `:1082–1107` | `x_i,x_{i+1}` |
| Clock C0/D | `:1110–1195` | `c_i,c_{i+1},d_i,d_{i+1}` |
| Priors | `:558–576`, `:1283–1365` | first `x_0,v_0,b_0`; broad `v_i,d_i`; optional C prior is disabled by the default zero clock-prior sigma |

Under the sealed full-coverage counts, this incidence pattern forms an
apparent temporal chain: `x` is connected to IMU, code/TDCP, and motion;
`v` to IMU and Doppler; `d` to Doppler, C0/D, and its broad prior; `c` to
code/TDCP/C0D; and global `i` to code across epochs. That is a structural
inference from source and aggregate counts, not a runtime connected-component
certificate. Phase96 did not serialize each factor's `keys()`, per-key degree,
or the graph's component labels. A missing epoch key, a factor referencing a
nonexistent key, or an unexpectedly isolated global state therefore cannot be
excluded by the current record.

GTSAM's public graph/factor API can expose factor keys and key vectors, and
`Values::keys()`, `dims()`, and `dim()` are read-only interfaces. Those exact
descriptors are part of the single diagnostic candidate below.

## Prior, gauge, and zero/near-zero audit

### Anchoring present in source

- `x_0` has a finite first-state Pose3 prior. Its translation sigma is
  `1e6`, so it is a weak finite anchor; attitude has the configured finite
  initialization sigmas.
- `v_0` and `b_0` have first-state priors.
- Every `v_i` and `d_i` receives a broad `1e6` sigma prior when the direct
  Doppler/IMU-D path is active. Its nominal diagonal contribution is
  `1e-12`; it is a numerical gauge guard, not a measurement.
- There is no default explicit C prior (`clock_prior_sigma_m == 0`), and no
  default ISB prior. C and ISB are instead connected through code/TDCP/C0D
  incidence. D is connected through Doppler, C0/D, and its broad prior.
- The official CCDD row is unchanged: in meter state,
  `(C2-C1)-((D1+D2)*dt/2)` with Jacobian order
  `[-1,+1,-dt/2,-dt/2]` for `[C1,C2,D1,D2]`, ordinary sigma `0.1 m`.
  `SourceClockC0DFactor` records this at
  `fgo_gtsam_internal.hpp:1227–1280`; insertion is at
  `fgo_gtsam_backend.cpp:1160–1169`.

These anchors show that some finite priors exist, but they do not establish
full rank of the coupled system. A missing C/ISB anchor is not automatically
a defect because observation factors can anchor a state relative to geometry.

### Aggregate zero/near-zero buckets already sealed

Phase96 reports family-by-variable aggregate minima, not key-indexed columns.
Representative values are:

| Route | Bucket | Reported minimum | Interpretation supported by the record |
|---|---|---:|---|
| MTV-A | `gnss_code/position` normal diagonal | `0` | a zero entry in some factor block; not a globally zero `x_i` column |
| MTV-A | `other/position` normal diagonal | `0` | a zero entry in some TDCP block; not a disconnected position key |
| LAX-T | `gnss_code/position` normal diagonal | `0` | same aggregate limitation |
| LAX-T | `other/position` normal diagonal | `0` | same aggregate limitation |
| MTV-A | `gnss_doppler/velocity` normal diagonal | `8.000526572996408e-11` | small block contribution; key and full-column sum unavailable |
| LAX-T | `gnss_doppler/velocity` normal diagonal | `1.5082617738627254e-9` | small block contribution; key and full-column sum unavailable |
| MTV-A | `clock_ccdd/clock_c` normal diagonal | `100` | finite CCDD block minimum |
| MTV-A | `clock_ccdd/clock_d` normal diagonal | `24.99999995197868` | finite CCDD block minimum |
| LAX-T | `clock_ccdd/clock_c` normal diagonal | `100` | finite CCDD block minimum |
| LAX-T | `clock_ccdd/clock_d` normal diagonal | `24.999999950523488` | finite CCDD block minimum |
| Both | broad `priors/clock_d` normal diagonal | `1e-12` | expected broad-prior contribution |

The sidecar's `normal_diagonal_min` is the minimum over each family/block
contribution. A zero block entry can arise from a factor's geometry while the
same variable receives nonzero contributions elsewhere. Consequently these
values cannot identify a zero column, a rank defect, or the nearby exception
key. No per-key normal diagonal, rank-revealing factorization, singular-value
summary, incidence degree, or component count was sealed.

For completeness, the complete set of emitted family/bucket minima is shown
below. These remain aggregate contributions, not per-key columns; `—` means
that the bucket was not emitted for that route.

| Family / variable bucket | MTV-A minimum | LAX-T minimum |
|---|---:|---:|
| `priors / position` | `1e-12` | `1e-12` |
| `priors / velocity` | `1e-12` | `1e-12` |
| `priors / clock_d` | `1e-12` | `1e-12` |
| `imu_preintegration_bias / imu_bias` | `100` | `100` |
| `imu_preintegration_bias / position` | `391.83302750453686` | `391.83246853429927` |
| `imu_preintegration_bias / velocity` | `1691.3967164254907` | `1692.0689827503556` |
| `gnss_code / position` | `0` | `0` |
| `gnss_code / clock_c` | `1.2056204080108645e-05` | `1.2539322865173465e-05` |
| `gnss_code / clock_isb` | `1.2056204080108645e-05` | `1.2539322865173465e-05` |
| `gnss_doppler / velocity` | `8.000526572996408e-11` | `1.5082617738627254e-09` |
| `gnss_doppler / clock_d` | `0.044953059394387346` | `0.01655738996843737` |
| `other / position` | `0` | `0` |
| `other / clock_c` | `14.171115452392971` | `5.623743268771576` |
| `motion / position` | `0.00039999999999999677` | `0.00039999999999999476` |
| `clock_ccdd / clock_c` | `100` | `100` |
| `clock_ccdd / clock_d` | `24.99999995197868` | `24.999999950523488` |

No `signal_bias`, `ambiguity`, or unclassified variable bucket was emitted for
either Phase96 route, consistent with the active Values-set audit above.

## Phase80 versus Phase96: the “factor addition cannot worsen rank” premise

The premise is valid only for adding a positive-semidefinite factor to the
same variables and retaining the same other factors. Phase96 is not that
operation.

### Actual native variable-set difference

| Route | Phase80 no-C0D Values | Phase80 active set | Phase96 C0/D Values | Phase96 active set |
|---|---:|---|---:|---|
| MTV-A | `10799` | per-epoch `x,v,b,c,d` + 2 global ISB + 2 signal-bias `f` states | `10798` | per-epoch `x,v,b,c,d` + 3 global ISB; no `f/j/a/z` |
| LAX-T | `7333` | per-epoch `x,v,b,c,d` + 2 global ISB + 1 signal-bias `f` state | `7332` | per-epoch `x,v,b,c,d` + 2 global ISB; no `f/j/a/z` |

The Phase80 signal-bias count is sealed in its summaries; the remaining extra
Values after the five per-epoch families are the global ISB states under the
native insertion rules. Key IDs were not serialized, so this decomposition is
source-plus-summary derived. Phase80 and Phase96 therefore do not even have
the same scalar variable set: Phase96 removes `f` states and changes the ISB
count for MTV-A.

### Actual native factor-set difference

| Route | Phase80 no-C0D graph | Phase96 C0/D graph |
|---|---|---|
| MTV-A | `143414` factors: IMU `2158`, direct Doppler `29588`, pseudorange `56547`, TDCP `46482`, position motion `2158`, legacy clock rows `2158`, broad V/D priors `4318`, first-state priors `3`, signal-bias priors `2` | `120093` factors: priors `4320`, IMU/bias family `2159`, code `57281`, Doppler `20748`, TDCP/other `31269`, motion `2158`, C0/D `2158` |
| LAX-T | `88025` factors: IMU `1465`, direct Doppler `13795`, pseudorange `41935`, TDCP `24964`, position motion `1465`, legacy clock rows `1465`, broad V/D priors `2932`, first-state priors `3`, signal-bias prior `1` | `61082` factors: priors `2934`, IMU/bias family `1466`, code `30798`, Doppler `8942`, TDCP/other `14012`, motion `1465`, C0/D `1465` |

Phase80's sealed manifest also records source-quality, signal-bias, and
historical base-pseudorange-compensation settings that are not part of the
Phase96 direct no-base graph. The aggregate P/D/TDCP rows are consequently
not one-to-one matched observations. C0/D replaces the legacy clock motion
row in the Phase96 branch; it is not merely appended to an unchanged Phase80
graph. The comparison is therefore evidence for a changed graph/variable
contract, not a proof that C0/D itself made a previously full-rank matrix
singular.

### What can and cannot be concluded

Facts:

- namespace characters are disjoint and duplicate insertion would throw;
- both Phase96 graphs were built, evaluated, and linearized;
- all sealed Phase96 factor-family errors and aggregate Jacobian blocks were
  finite;
- both main solves failed at the linear-system progress boundary for all ten
  trials;
- Phase80's different no-C0D graph accepted 12 iterations and strictly
  reduced cost on both route identities.

Permitted inference:

- the failure is localized after handoff, during main LM solve/progress;
- the Phase80 contrast justifies graph-structure diagnosis;
- the CCDD aggregate proxy near `2` is insufficient to characterize complete
  rank or conditioning.

Not established:

- a key collision;
- an isolated variable or disconnected component;
- a globally zero/near-zero column;
- a specific C, D, ISB, Pose3, velocity, or bias gauge;
- CCDD equation/unit/sigma error;
- whether the pinned exception arose from rank deficiency, indefiniteness,
  poor conditioning, ordering, or another numerical condition;
- whether the first ten trials would have had finite predicted/actual costs,
  because no trial solved the system and those fields are unavailable.

## Candidate comparison and freeze boundary

No algorithmic correction is frozen. Three possible actions were compared:

| Candidate | Why it might appear attractive | Decision |
|---|---|---|
| Remove/disable C0/D or restore the Phase80 variable set | Could reproduce the historical solved graph | Reject: changes the experiment and cannot identify the Phase96 failure mechanism |
| Change C0/D sigma, LM damping/limits, priors, filters, solver, or add a retry/fallback | Could make a solve pass | Reject: tuning/algorithm change is outside this audit and has no causal evidence |
| Exact key/incidence/connectivity/rank-revealing observation of the existing graph and LM boundary | Directly tests the unresolved hypotheses without changing the solve | **Freeze exactly this one candidate** |

The frozen candidate must remain default-off and diagnostic-only. Its required
read-only payload is:

1. Every active `Values` key's Symbol character/index, concrete value type,
   dimension, total `Values::size()`, total `Values::dim()`, and duplicate or
   missing-key/type-mismatch diagnostics.
2. Every existing factor's family, exact key list, finite status, and
   per-key/per-family incidence degree. Report graph components from that same
   key incidence; do not build a counterfactual graph.
3. Prior/gauge anchor labels keyed to the exact variables, including weak
   broad priors and absent explicit anchors.
4. Keyed Jacobian-column and normal-diagonal summaries, with exact-zero and
   reporting-only near-zero buckets. Bucket thresholds may describe output
   only; they must not alter damping, filtering, factor weights, or solving.
5. The actual `IndeterminantLinearSystemException::nearbyVariable()` key when
   the pinned solver boundary can bind it. If the pinned LM catch remains
   unmodified, emit an explicit `nearby_variable_unavailable` status rather
   than inventing a key. Preserve the documented fact that this key is a
   detection location, not necessarily the origin.
6. The first ten existing LM trials' source branch, lambda, solve status,
   predicted/actual values when defined, finite status, and exception class;
   no additional solve, retry, damping policy, or termination decision.
7. A failure-safe diagnostic sidecar before any solution/accuracy lane. It
   must never publish state vectors, coordinates, raw observations, truth,
   or a passing result for a failed solve.

The implementation boundary is observation at existing graph construction,
linearization, and pinned `tryLambda` locals. It must preserve the official
CCDD equation/Jacobian, meter C/ISB and metre-per-second D units, ordinary
sigma `0.1 m`, filters, priors, LM parameters, no-PDC/no-external-input
policy, no-fallback policy, and legacy-default behavior. No raw execution is
authorized by this audit or the separate freeze JSON.

## Release boundary

This record establishes neither a solution nor a correction. Any future
implementation, test, or raw diagnostic run requires a separately pinned
artifact and authorization. The only candidate permitted by the Phase97
freeze is the observation-only key/connectivity/rank diagnostic described
above.
