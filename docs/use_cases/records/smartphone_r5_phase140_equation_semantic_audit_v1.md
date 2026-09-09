# Phase140 sealed-equation semantic audit

Execution label: Luna Max  
Phase: 140  
Scope: read-only audit and post-run revalidation design.  No raw payload,
solver, truth, solution-coordinate, MAT/PDC/precomputed-coordinate, accuracy,
or Kaggle content was read or executed in this phase.

## Decision

The Phase139 failure is a representation mismatch, not a mutation of the
Phase138 TDCP right-hand side.  Both routes in the sealed result contain the
same native text:

```
tdcp_native-(rho_current_initial-rho_previous_initial)
```

The frozen contract names the complete assignment:

```
tdcp_phase138 = tdcp_native - (rho_current_initial - rho_previous_initial)
```

The native text is therefore the exact RHS-only representation of the frozen
assignment.  It omits the LHS label and insignificant whitespace only.  It
does not commute the subtraction, change either sign, remove either range
term, or add another term.  The historical Phase139 result remains immutable
and remains recorded as `NO-GO` because its launch-time validator required the
full textual assignment; this audit does not rewrite or re-run it.

## Pinned evidence

| Evidence | Location / pin | Observation |
| --- | --- | --- |
| Sealed structural result | `02b3c3042e131e0c08403be738e76050f8b887c4`; JSON SHA-256 `a772fe23935376cddb95f4e0c6cc1ec8954bf9c875f5577504ed84179004a748` | MTV-A and LAX-T each record the same RHS-only string and the same expected/full-string validator failure. |
| Sealed result sidecar | `docs/use_cases/records/smartphone_r5_phase139_affine_tdcp_structural_raw_result_v1.md`; SHA-256 `8533f455888f8a09a1129aca6df1a0efcf03e127709db9bbe0834413b978a153` | Describes the mismatch as representation-only and preserves route scalar telemetry and opaque hashes. |
| Frozen equation | `apps/commands/benchmarks/gnss_smartphone_phase138_affine_tdcp_structural.py:623-626`; Phase138 freeze `f1f5fef2ae0ccdcbae94be86e6af813b65ec499b` | Validator requires `tdcp_phase138 = tdcp_native - (rho_current_initial - rho_previous_initial)`. |
| Native metadata | `src/algorithms/fgo.cpp:205-213`, `src/algorithms/fgo_gtsam_backend.cpp:121-127` | Enabled Phase138 summaries emit `tdcp_native-(rho_current_initial-rho_previous_initial)` and the fixed-endpoint/single-Sagnac geometry label. |
| Native arithmetic | `include/libgnss++/algorithms/tdcp_contract.hpp:27-37`; source SHA-256 `f4423e1944fd58f28ad779fa6a6ba87919e358360cdf1ddda59b0af20a954d57` | `range_delta_m = current_initial_range_m - previous_initial_range_m`, then `tdcp_m = tdcp_native_m - range_delta_m`, with finite checks. |
| Native call boundary | `src/algorithms/fgo_gtsam_backend.cpp:1580-1612,1637-1641,1664-1679` | The same endpoint geometry computes the constant, the adjusted measurement is inserted once, and count conservation is fail-closed. |

The result JSON failure entries are at `routes[0].failure` and
`routes[1].failure`; both are byte-for-byte the same expected-versus-actual
comparison.  No coordinate field was interpreted.  The sealed sidecar's
aggregate accounting is historical evidence only: raw phone GNSS/IMU,
broadcast navigation, raw-base, and native solver counts are two each, while
truth, solution-coordinate, MAT/PDC/precomputed, accuracy, Kaggle, and
rerun/fallback/repair counts are zero.

## Formal token and AST comparison

The semantic contract is defined without algebraic simplification.  Lexing
may remove ASCII whitespace only; it must preserve identifiers, operators,
and grouping.  The complete frozen expression has this AST:

```text
ASSIGN(
  lhs=tdcp_phase138,
  rhs=SUB(
    tdcp_native,
    SUB(rho_current_initial, rho_previous_initial)))
```

The exact token tuples are:

```text
full:
[ASSIGN, tdcp_phase138, SUB, GROUP_OPEN, tdcp_native, SUB, GROUP_OPEN,
 rho_current_initial, SUB, rho_previous_initial, GROUP_CLOSE, GROUP_CLOSE]

rhs-only:
[tdcp_native, SUB, GROUP_OPEN, rho_current_initial, SUB,
 rho_previous_initial, GROUP_CLOSE]
```

The actual native text tokenizes to the `rhs-only` tuple.  Prepending the
contract's typed LHS and assignment produces exactly the `full` tuple.  The
following are deliberately not equivalent and must be rejected: swapping
`rho_current_initial` and `rho_previous_initial`, replacing either `SUB` with
`ADD`, changing the outer subtraction, dropping a range term or grouping,
adding any term, changing `tdcp_phase138`, or using an unknown identifier.
Whitespace normalization alone must never accept those variants.

This distinction also prevents a dangerous false positive: a native field
must declare (or be bound by its sealed schema to) `representation=rhs-only`,
not merely contain a string that happens to look like a RHS.  The current
historical field is the pinned
`phase138_measurement_equation` field emitted by the pinned native source;
future summaries should carry the representation and semantic ID explicitly.

## Unit/sign and geometry proof

The helper's `current - previous` order is visible at
`tdcp_contract.hpp:35`; line 36 subtracts that delta from `tdcp_native_m`.
Thus the expression is metre-valued TDCP native measurement minus the
source-consistent initial geometric range change.  The backend evaluates the
previous and current endpoint ranges through the same Phase135 geometry path
before invoking the helper (`fgo_gtsam_backend.cpp:1582-1607`).  The frozen
geometry label records one RTKLIB geodist/Sagnac convention.  No source or
sealed evidence indicates a changed unit, sign, endpoint, satellite state, or
Sagnac application.

## Frozen revalidation boundary (design only)

Because the semantic comparison is positive, one metadata-only candidate is
safe to freeze.  It is not implemented or launched in Phase140.  A future
validator may read only:

1. the immutable Phase139 structural result/sidecar scalar summaries;
2. the sealed native summary scalar fields and their recorded byte/hash
   digests; and
3. the pinned semantic-equation schema in the accompanying Phase140 freeze.

It must never open raw inputs, invoke the solver, read solution coordinate
rows, read truth/MAT/PDC/precomputed artifacts, calculate accuracy, or contact
Kaggle.  It may classify the immutable Phase139 run `GO` only if every
required scalar field is present and passes; absent, ambiguous, or altered
metadata is `NO-GO`.

The validator must recheck, per route, the complete Phase138 structural gate:

* exact selector isolation, Phase135 all-affine family counts, and zero legacy
  factors;
* semantic equation ID plus the exact full/RHS token form, fixed-endpoint
  satellite-state provenance, one Sagnac representation, finite adjusted
  measurements, exact range/adjustment/factor counts, one application pass,
  and transactional/exactly-once flags;
* Phase107 raw-base exactly-once and conservation flags, with no raw/zero/
  hold/nearest/extrapolated fallback;
* seven metre-valued C components, finite metre/second D, exact retained-key
  alignment, QR/`EliminateQR`, accepted iterations, finite strict cost
  decrease, and no fallback; and
* finite earth-valid expected output coverage, exactly one Pixel5 final-output
  offset, opaque solution seal only, and zero forbidden reads.

The native summary digest is an integrity witness, not permission to read its
coordinate or solution payload.  The validator must bind each scalar summary
to the sealed route/hash and reject duplicate routes, changed hashes, missing
fields, non-finite numbers, extra equation terms, or any read-accounting
violation.  It must not infer an absent accepted-iteration field from a zero
return code or from a cost decrease.

## Required focused tests and accounting

The next implementation/qualification boundary must add synthetic tests for:

* full assignment and pinned RHS-only text accepted only with their declared
  representations;
* optional-whitespace equivalence;
* commuted range terms, wrong signs, missing/extra terms, wrong LHS,
  malformed grouping, unknown identifiers, and representation mismatch
  rejected;
* exact semantic-ID/AST/token-tuple serialization and hash binding;
* route/hash/summary scalar gate revalidation, missing-field fail-closed,
  non-finite and duplicate-summary rejection; and
* legacy/default selector isolation.

Phase140 accounting is zero for raw reads, solver invocations, truth reads,
solution-coordinate reads, MAT/PDC/precomputed reads, accuracy calculations,
Kaggle/token access, materialization, and reruns.  Only tracked source,
sealed-result metadata, and native/freeze source text were inspected.  Any
future validator implementation or execution requires a new independent
authorization and a separate result seal; this audit authorizes neither.
