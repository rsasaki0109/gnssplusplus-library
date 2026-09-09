# Phase144 native telemetry serializer audit

Status: read-only source/metadata audit.  The immutable Phase143 raw result
`6ff8ac1d980153ee1dd3036295f3d92ba4f0cdfc` remains `NO-GO`; this audit does
not reopen raw GNSS, IMU, navigation, or base payloads, does not parse a
solution row, and does not start a solver.

## Evidence

The two native structural summary files produced by the sealed Phase143 run
were inspected as JSON bytes only for schema/duplicate-key evidence:

| route | bytes | SHA-256 | duplicate JSON paths |
|---|---:|---|---|
| MTV-A | 38,815 | `4ce97e7b6e95450ef2b7c54e89047c597c32f8a032f72bd5922e4120d814f061` | `/gnss_first/failure` |
| LAX-T | 38,100 | `c5e443b1fed5af0335bab2e3468b656c1fdbc43ce97222f2a4b65806c06cc846` | `/gnss_first/failure` |

The recursive duplicate detector found exactly one repeated key path in each
summary.  At `apps/native/gnss_fgo_imu_no_base.cpp:8490-8507`, the ordinary
top-level `gnss_first` object emits `failure` from
`ImuBuildReport::gnss_first_failure`; each selected handoff branch then emits
another `failure` at `:8520-8720`.  The values are authoritative reports, but
the JSON object is invalid because the key is emitted twice.  A last-wins JSON
reader hides this defect; Phase144 remains fail-closed on duplicates.

The same summaries show the native Phase143 termination sidecar is complete:
both stages carry attempted/accepted/rejected counts, costs, finite/strict
progress flags, termination branch, solver/elimination, lambda policy, and
configuration validity.  The Phase143 result's wrapper could not consume these
fields because duplicate-key rejection occurred first.

## Schema and semantic gaps

The actual top-level summary has the following representation gaps against the
Phase138/139 structural contracts:

| required evidence | actual representation | classification |
|---|---|---|
| one `gnss_first.failure` | base failure plus branch failure | duplicate/fatal |
| Phase138 full assignment display | `tdcp_native-(rho_current_initial-rho_previous_initial)` | semantically equivalent RHS, display-divergent |
| Phase138 semantic equation object | absent from the normal Phase143 summary; only optional Phase141 envelope advertises an AST | missing |
| P/D/TDCP admitted rows, affine insertion, key order, finite values, source geometry | factor counts and internal booleans are present, but the complete family objects are only in the optional Phase141 envelope | represented by separate fields |
| Sagnac evaluation count | `geometry_rows_validated` | misnamed/derived representation |
| accepted/rejected outer iterations and terminal reason | generic `gnss_first.iterations`/`graph.iterations`; authoritative values are in `phase143_termination` | authoritative sidecar, generic fields insufficient |
| bridge/factor-count aggregate | individual p135 counts and optional Phase141 `bridge`/`factor_counts` | represented by separate fields |
| C7/D, base, offset, output guards | native report fields are present | complete for the frozen scalar subset |

The Phase138 equation is mathematically the same right-hand side as the
frozen assignment, but it lacks the left-hand-side assignment and canonical
spacing.  The serializer must therefore preserve the diagnostic RHS internally
while emitting this exact display and AST at the schema boundary:

`tdcp_phase138 = tdcp_native - (rho_current_initial - rho_previous_initial)`

AST: `assign(tdcp_phase138, sub(tdcp_native, sub(rho_current_initial,
rho_previous_initial)))`.  The canonical token tuple is
`ASSIGN, tdcp_phase138, SUB, GROUP_OPEN, tdcp_native, SUB, GROUP_OPEN,
rho_current_initial, SUB, rho_previous_initial, GROUP_CLOSE, GROUP_CLOSE`.

## One candidate

Freeze exactly one default-off telemetry candidate:

`--native-phase144-telemetry-schema`

When selected together with the existing Phase135/138/118/143 recipe, the
native summary serializer will:

1. emit each object key once, with one authoritative `failure` and a distinct
   `handoff_failure` where branch-specific evidence exists;
2. emit the canonical Phase138 display, semantic ID, AST, full token tuple,
   RHS token tuple, and representation at the normal summary boundary;
3. expose authoritative GNSS-first/main attempted and accepted counts, initial
   and final costs, solver/elimination, factor-family counts, bridge, C7/D,
   base, offset, and output guards in deterministic field order; and
4. mark `native_phase144_telemetry_schema=true` so a launch-free validator can
   require the repaired schema.

All values are copied once from existing native reports.  No graph, factor,
equation evaluation, unit, sigma, filter, LM schedule, ordering, initialization,
or output row changes are authorized.  The default and all earlier selectors
remain unchanged; malformed, missing, duplicate, or semantically wrong fields
fail closed.

## Implementation/qualification boundary

The implementation is limited to the native summary writer and its CLI marker,
plus schema tests.  A later launch-free Phase144 runner will pin this source,
the target binary, and a complete synthetic summary validator.  It will report
zero raw/solver/truth/solution reads.  Any future raw run requires a new
independent authorization after that qualification; the historical Phase143
result is never rewritten or re-run.

