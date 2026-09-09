# Phase126 pre-raw accounting seal

- Execution label: `Luna Max`
- Phase: `126`
- Runner/validator/manifest commit: `40f1d93c0b7dbf9b92353df9e283cd0ad03de9b4`
- Freeze commit: `5d00daf56afb6fe6b6e1f01b4a1213c287f671db`
- Implementation commit: `9e9972667ee009e1bcd0dd1ea4732ae45415c3ce`
- Matrix: `MTV-A -> LAX-T`, exactly one planned run per route.

This is a launch-free pre-authorization seal.  Only tracked source, the
Phase126 design/audit/freeze records, and sealed Phase112/117/118 metadata
were inspected.  No raw phone/base member was materialized, probed, hashed,
or opened; no native application or solver was started; and no truth,
solution, coordinate, MAT, PDC, accuracy, or Kaggle lane was entered.

The runner contains placeholders rather than input paths and has no child
process or payload operation.  Post-authorization RINEX/header/signal
inventory is a mandatory gate, not a pre-raw action.  Unknown signal mapping,
GLONASS channel/frequency, station-reference semantics, satellite state,
atmosphere domain, or finite/in-domain value fails closed before any route
launch.  No partial A/B/C stream may be applied, and no fallback or rerun is
authorized.

## Accounting

The machine-readable companion records every counter as zero.  In particular,
`raw_base_rinex_reads`, `raw_base_header_reads`, and `raw_base_hash_reads`
are zero; the sealed base SHA values in the contract remain metadata only.
`solution_rows_opened` and `solution_coordinate_interpretations` are zero,
and the solution remains opaque/withheld.

The Phase126 structural validator and its facade passed the focused
launch-free tests (9 tests) and Python compilation.  These checks validate
only source/manifest invariants; they do not constitute raw or solver
authorization.  An independent authorization commit is the next boundary.
