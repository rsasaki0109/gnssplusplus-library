# Phase384 — evaluation-domain mismatch caught before scoring

Phase382/383 emitted all 1466 native raw epochs because their argv inherited
Phase249 U's --android-include-first-native-epoch. Phase146's pinned LAX-T
truth metadata has 1465 rows. Its historical output metadata has 1466
newlines including header, i.e. 1465 data rows. No truth payload was opened
to discover this discrepancy. No accuracy score was computed.

The raw alignment serializer defaults to first_output=1 and excludes only
the first raw epoch. The explicit include-first flag changes first_output
to 0; it does not change solver construction. Hence complete 1466-row
native output is not itself an exact evaluation-domain match. Earlier
structural success claims apply to raw coverage, not truth-key coverage.

Next freeze an evaluation-only projection for BOTH candidates: preserve
the original 1466-row artifacts and omit exactly the first raw timestamp,
using raw identity and the historical serializer convention, not error
magnitude. No other rows may be selected, interpolated or filled. Pin both
original and projected hashes, then require the unchanged metric kernel's
exact key-domain check against the 1465-row truth. If any other mismatch
exists, fail closed with no retry. Row counts alone do not prove that the
missing truth key is the first raw key; the exact join must establish it.

Such a projection is evaluation/output processing only; never feed either
candidate or projected coordinates back into inference. No native rerun is
needed merely to alter serializer coverage. This is not route-specific
accuracy selection, heldout validation or evidence of achieving the goal.
