# Smartphone R5 Phase134 native-summary bridge structural contract audit

- Execution label: `Luna Max`
- Audit date: `2026-09-04`
- Candidate: `phase134-native-phase131-summary-diagnostics-bridge-v1`
- Scope: launch-free qualification of the already implemented telemetry-only
  bridge.  This audit does not authorize raw materialization, native solver
  execution, solution-row access, truth/accuracy evaluation, or publication.

## Decision

Exactly one structural candidate is retained: synchronize the Phase131
canonical-correction diagnostics that are already present in
`BasePseudorangeCompensationReport` into the `FGOProblemDiagnostics` object
used by the native top-level summary.  The synchronization is a value copy,
performed exactly once after the existing base report is populated and before
summary serialization.  A second synchronization fails closed.

The candidate is telemetry-only.  It does not change observations,
corrections, factor construction, state values, equations, units, sigma,
filtering, robust kernels, C7/D/C0D, QR/LM, initialization, output rows, or
legacy/default behavior.  Existing structural gates remain gates; the bridge
does not turn a zero or missing counter into a positive result.

## Pins and source evidence

| artifact | pinned value |
|---|---|
| Phase134 candidate freeze | `41a9fa8dd52878cd7992313f151014dc9cca0fd8` |
| candidate freeze JSON SHA-256 | `6c9bb8715e465876c7b95096130c3af4c5f9bfaabb038edd3812c71f500e95a0` |
| implementation commit | `130a7f8f12e191cc12ebd0ff66ad591d732775f7` |
| native app source SHA-256 | `efddfed4104d71db3df13a9b4cd0240bd603e4c78a3be313750dfe96c9273769` |
| FGO diagnostics header SHA-256 | `4f2df010f8b4c73bb1ca83d17811db7672c31d154507f758dc940ee1124b22ae` |
| Phase133 wrapper SHA-256 | `37b02693ae183edee8435a4247ef474f67e50ecb65773901ebad593900f92fab` |
| target binary SHA-256 | `3965852271023671cd0e9c6ea3e779ab6f67f4e883d724ccacadf28bd8b2fb79` |

The implementation commit changes only the diagnostics value object, native
summary writer/bridge, the wrapper's separate native/normalized summary
paths, and tests.  The solver and factor implementation directories and the
algorithm configuration/model headers are unchanged by the candidate.

## Exact bridge contract

The native model report is the authoritative source for the following
Phase131 fields:

- enabled/configuration-valid/configuration-failure;
- canonical, rejected, unknown-band, conflict, duplicate, stream, selected
  stream, and merged-stream counts;
- canonical failure-reason map;
- canonicalization attempts and resolver call count;
- source-miss matching mode/key and original, retained, dropped, matched,
  finite, build, application-pass, and corrected-row conservation fields;
- factor/signal consistency, applied, exactly-once, and duplicate-rejection
  markers.

The destination is `FGOProblemDiagnostics`, and the native top-level
`phase131_canonical_correction_band_key` object is serialized from that
destination.  The explicit invariant is:

```
canonicalization_attempt_rows = canonical_rows + canonical_rejected_rows
resolver_call_count        = canonicalization_attempt_rows
top_level_phase131         = exact value copy of base-report Phase131 fields
diagnostics_bridge.count    = 1 and diagnostics_bridge.exactly_once = true
```

The existing native nested base-compensation report remains present and is
not rewritten.  The wrapper gives the native process
`native_summary.json` and writes its derived/normalized view separately as
`structural_summary.json`.  The native artifact is byte-exactly retained and
its byte count/hash are recorded.  A same-path copy or a second preservation
attempt fails closed.

## Frozen structural matrix

The future independent authorization is limited to MTV-A then LAX-T, one
run per route, with no control rerun, fallback, repair, or sweep.  The
Phase118, 126, 127, 128, 129, and 131 native selectors are each forwarded
exactly once.  Phase130 remains the historical runner-only selector and is
forwarded zero times because the native binary does not own it.  Phase117,
Phase120, and additional-frequency selectors remain off.

For each authorized route the validator must require:

1. native and normalized summary paths are distinct;
2. native summary bytes/hash are retained before normalization;
3. bridge source, count, and exactly-once marker are valid;
4. resolver/attempt count is positive and satisfies the exact equation above;
5. every base-report canonical/reject/conservation field equals the native
   top-level field or its declared nested conservation counterpart;
6. no raw/zero correction fallback, duplicate bridge, or fabricated counter;
7. the pre-existing algorithmic structural gates (finite progress, strict
   cost decrease, coverage, factor/C7/D/QR/IMU/TDCP/base/offset markers) are
   evaluated unchanged.

The solution file is only an opaque hash/row-count artifact if a later
authorization permits a run.  No solution coordinate row is opened here.

## Qualification plan and read boundary

Launch-free tests use only synthetic JSON objects, temporary non-payload
files, static source text, and sealed metadata.  They cover nonzero and zero
bridge propagation, exact base/top-level copy, resolver/attempt mismatch,
conservation mismatch, duplicate synchronization/preservation, distinct
native versus normalized paths, forbidden payload lineage, and selector
isolation.  The full C++ suite and the focused Python suite are build/test
qualification only.

Before any raw run, a new manifest and pre-raw seal must pin this audit,
freeze, implementation, runner, validator, tests, and target binary.  A new
independent authorization is mandatory; the historical Phase133
authorization/result cannot be reused after the native binary change.

During this audit and launch-free qualification:

| resource/action | count |
|---|---:|
| raw phone GNSS/IMU, broadcast nav, raw-base payload/header | 0 |
| native solver/binary invocation | 0 |
| solution coordinate rows | 0 |
| truth/accuracy/MAT/PDC/precomputed coordinates | 0 |
| Kaggle/token access | 0 |
| rerun/fallback/repair/sweep | 0 |

The next boundary is contract/runner/manifest validation only.  Structural
raw execution and all truth/accuracy lanes remain unauthorized.
