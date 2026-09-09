# Phase141 native telemetry schema completeness audit

Execution label: Luna Max  
Phase: 141  
Scope: read-only source/schema audit and a telemetry-only bridge design.  No
raw GNSS/IMU/navigation/base payload, solver, solution-coordinate, truth,
MAT/PDC/precomputed-coordinate, accuracy, or Kaggle content was read or
executed.

## Decision

The Phase139 structural result is historical and remains `NO-GO` and
immutable.  Its two native runs already contain useful scalar diagnostics, but
the Phase138/139 validator consumes a *normalized wrapper schema* that is not
the native schema.  The wrapper currently fills missing values from return
codes, selector state, geometry labels, or generic iteration counts.  That is
not sufficient for a fail-closed structural gate: an absent accepted-iteration
count must not be inferred from `iterations`, and an absent finite/transactional
or output field must not be inferred from a successful process return.

One candidate is frozen for implementation: an opt-in native
`phase141_telemetry` object emitted at the existing summary boundary.  It is a
diagnostic snapshot only.  It copies each value once from an authoritative
internal report (`ImuBuildReport`, `FGOResult::diagnostics`,
`BasePseudorangeCompensationReport`, `UpstreamPositionOffsetReport`, and the
existing output/alignment reports) and carries an explicit semantic equation
ID/AST beside the unchanged display expression.  The wrapper/validator will
validate this object; it will not synthesize or overwrite it.  Missing,
duplicated, renamed, non-finite, or conflicting values fail closed.

The candidate does not alter factors, equations, units, sigma, filtering,
initialization, graph topology, solver selection, LM behavior, output rows, or
the legacy/default path.  It is not raw-execution authorization.

## Evidence and source map

| Required structural fact | Native source of truth | Current representation / defect |
| --- | --- | --- |
| Route and selector isolation | `apps/native/gnss_fgo_imu_no_base.cpp:6113-6208` (`Options`, selected solver) | Root selector booleans exist, but no versioned complete telemetry envelope binds them to the normalized route summary. |
| Phase135 enabled/configuration | `include/libgnss++/algorithms/fgo.hpp:1226-1234`; `makeSummary:6219-6268` | Native emits enabled/configuration/counts, but not an explicit transactional or finite-Jacobian report. |
| Phase135 P/D/TDCP admitted and affine counts | `FGODiagnostics` fields and backend insertion counters; `makeSummary:6232-6239` | Native names only `*_factors_inserted`; validator requires `admitted_rows`, so wrapper has to invent equality. |
| Phase135 key order and source geometry | backend affine insertion path (`src/algorithms/fgo_gtsam_backend.cpp`) and native fixed key list at `makeSummary:6245-6253` | Key list is display metadata; per-family `key_order_exact`, `finite_values`, and `source_geometry_same_path` are absent and wrapper hard-codes them. |
| Phase135 legacy counts | `FGODiagnostics`/problem factor vectors | No native legacy-count object is emitted; wrapper hard-codes zero. |
| Pose3-X bridge exact keys | `FGODiagnostics.phase135_pose3_x_bridge_factors` and backend bridge insertion | Native emits only a count; `keys_exact` is absent and wrapper hard-codes true. |
| Phase138 dependency/configuration | `fgo.hpp:1238-1247`; `makeSummary:6270-6310` | Enabled/configuration/counts are present; dependency is derived from the command by wrapper rather than a native admission report. |
| Phase138 equation semantics | `src/algorithms/fgo_gtsam_backend.cpp:121-127`; `tdcp_contract.hpp:35-36`; native summary `makeSummary:6288-6289` | Native emits RHS-only `tdcp_native-(rho_current_initial-rho_previous_initial)`. The frozen validator demands the full assignment. Phase140 proved this is representation-only, but current schema lacks `semantic_id`, `representation`, and AST/tokens. |
| Phase138 endpoint/satellite state, finite adjusted values, transaction | backend constant/adjustment path (`fgo_gtsam_backend.cpp:1580-1679`) | Counts and `adjusted_exactly_once` exist; `same_endpoint_epoch_and_satellite_state`, `finite_adjusted_measurements`, `no_raw_or_zero_fallback`, and `transactional` are absent and wrapper derives them. |
| Phase138 legacy TDCP count | graph/factor insertion diagnostics | Not emitted; wrapper supplies zero. |
| Raw-base exactly-once/conservation | `BasePseudorangeCompensationReport` at `gnss_fgo_imu_no_base.cpp:3802-3917`, native objects at `6395-6748` | The native report contains detailed counts/flags, but normalized `raw_base` is a wrapper subset and does not preserve a one-to-one authority binding. |
| C7 metre units / C7 mapping / D alignment | native C0/D report `makeSummary:7068-7245`, `FGODiagnostics` C0/D fields, Phase91/93/101 `ImuBuildReport` fields | Native uses names such as `internal_clock_state_unit`, `epoch_vector_dimension`, and raw-D initializer fields; normalized `clock.c_units`, `clock.d_units`, and exact alignment are absent. |
| GNSS-first attempted/accepted iterations and costs | `ImuBuildReport:2263-2369`; populated at `gnss_fgo_imu_no_base.cpp:9643-9659`; summary `7898-7917` | Native has `attempted`, `iterations`, and costs, plus C0/D accepted count. It does not expose normalized `accepted_iterations` or `no_fallback`; wrapper falls back from C0/D or generic iterations. |
| Main attempted/accepted iterations and costs | `FGOResult::FGODiagnostics` at `fgo.hpp:1322-1360,1484-1508`; main result around `9873-9900`; `makeSummary:7068-7245` | Main initial/final costs and C0/D accepted count exist, but no explicit main `attempted`, `accepted_iterations`, or `no_fallback` field is emitted. Phase139 explicitly records the accepted field as absent. |
| Solver type/elimination | `FGODiagnostics.selected_linear_solver_type`, `selected_elimination_function` (`fgo.hpp:1502-1508`), root summary `6204-6217` | Values exist at root but not under the normalized `solver` object and are not bound to stage telemetry. |
| Output finite/earth-valid/expected coverage | output checks `gnss_fgo_imu_no_base.cpp:10405-10431`; `Phase94MainTelemetry:2836-2994`; raw UTC report | Native output contract reports finite/atomic writing and raw UTC counts, but no complete scalar output gate object. Wrapper infers `earth_valid`, coverage, and opaque seal from return code/path. |
| Pixel5 offset exactly once | `UpstreamPositionOffsetReport:3778-3787`, sole application `10250-10355` | Native reports `applied` and corrected epochs; no explicit application-pass count. Wrapper coerces an application count. |
| Failure/fallback/read policy | `makeSummary:6117-6122`, stage diagnostics, wrapper accounting | Native has `fallback` and truth-free fields, while wrapper owns path/read accounting. They must remain separate; native cannot claim raw read counts that the wrapper did not observe. |

## Sealed historical evidence

The only sealed run inspected for this audit is commit
`02b3c3042e131e0c08403be738e76050f8b887c4`, result JSON SHA-256
`a772fe23935376cddb95f4e0c6cc1ec8954bf9c875f5577504ed84179004a748`.
Its sidecar SHA-256 is
`8533f455888f8a09a1129aca6df1a0efcf03e127709db9bbe0834413b978a153`.
Both MTV-A and LAX-T records contain the RHS-only Phase138 display text and
the same representation-only validator failure.  They also record native
main costs and generic `iterations`, but no normalized main accepted-iteration
field.  This audit does not reinterpret coordinates or reopen any payload.

The Phase140 semantic audit/freeze (`5bcd50a25d9b45612805a78b6a54f09719394fff`
and `539de2a82cc153d197e39728229c1d46e9edee67`) proves the equation's RHS
semantics, but explicitly forbids inferring the missing main gate.  Therefore
the result stays NO-GO.

## Frozen candidate schema boundary

The native object is versioned as
`smartphone-r5-native-fgo-phase141-telemetry.v1` and has one copy/sync marker.
It must contain these authoritative groups:

* `equation`: semantic ID, declared representation (`full-assignment` or
  `rhs-only-native-diagnostic`), unchanged display expression, exact AST and
  token tuple.  Whitespace normalization is the only permitted textual
  normalization.
* `phase135`: enabled/configuration/transactional/finite-Jacobian/fixed-LOS/
  single-Sagnac flags; geometry and Sagnac counts; P/D/ordinary-TDCP admitted
  and affine-inserted counts with key order and finite/source-path flags;
  legacy family counts; Pose3-X bridge count and exact-key flag.
* `phase138`: dependency/configuration/transaction/exactly-once flags,
  endpoint/satellite-state/finiteness/no-fallback flags, display equation and
  semantic equation object, range/adjustment/affine/legacy counts and pass
  count.
* `clock` and `solver`: C/D units, C7 mapping, exact retained-D alignment;
  selected linear solver/elimination; GNSS-first and main attempted,
  accepted-iteration, initial/final-cost, finite/strict-progress, terminal and
  no-fallback fields.  Accepted count is taken from the stage's authoritative
  active C0/D solve report; it is never inferred from generic `iterations`.
* `raw_base` and `offset`: authoritative exactly-once, conservation, and
  failure fields from the existing reports, including an explicit offset
  application pass count.
* `output` and `policy`: scalar finite/earth-valid/expected-coverage and
  opaque-seal flags, Pixel5 application count, truth/coordinate-publication
  false.  No coordinate rows or solution values are serialized.

The wrapper may validate these fields and add its independent read-accounting
record, but may not copy an old field into a differently named field, derive a
boolean from a return code, or replace the native object.  A duplicate
`phase141_telemetry` object, duplicate key, changed field name, mismatched
source report, non-finite value, or missing required field is NO-GO.

The legacy summary objects remain byte/semantics-compatible when the selector
is off.  The candidate is default-off and only changes diagnostics when
explicitly selected.  It does not authorize raw or solver execution; a future
launch-free manifest and a new independent authorization are required.

## Required qualification

Focused schema tests must instantiate a complete real-shaped native summary,
accept the declared full and RHS-only equation representations, and reject
missing/misnamed/duplicate/conflicting fields, non-finite costs, wrong AST or
tokens, inferred accepted counts, wrapper overwrite, and selector leakage.
The launch-free validator must report zero raw, solver, truth, solution-row,
MAT/PDC/precomputed, accuracy, and Kaggle reads.  Full C++ qualification is
required after the telemetry-only implementation.  The historical Phase139
artifact must not be modified or re-run.
