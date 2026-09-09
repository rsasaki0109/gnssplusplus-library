# Phase281: pure grouped P reselector

Added pseudorange_remasking::select(FGOProblem const&), which computes
corrected_P - range(updated_epoch_position) - updated_meter_clock over
the retained pool, centers by system/band median, applies the unchanged
L1/L5 20/15 m thresholds, and returns pool indices and recovered/removed/
unchanged counts. It does not mutate the problem or measurements.

Checks include nonempty inputs, finite state/measurement/sigma, increasing
epoch times, metre-clock marker, valid epoch indices, supported bands,
unique pool identities, and old selection being a unique pool subset.
There is no file interface or saved positioning input.

Build 7096 completed with exit 0. Four selected tests passed: the new
reselector test, two pool tests and the earlier admission-predicate test.
The new synthetic GPS group has four observations, one 25 m code error,
and an old accepted subset excluding one good row. Selection recovers that
good row, removes the bad row and retains two; duplicate pool identity is
rejected. Input spot-checks confirm no mutation. This is not an end-to-end
raw-to-main recovery test, full CTest or real-data accuracy evidence.

Before integration, extend tests for separate groups, odd/even medians,
invalid state/old subset, and unchanged measurement identity. Exact raw
epoch alignment and same-run state ownership must be enforced by the CLI
caller. A public value object cannot establish arbitrary caller provenance.
Keep reselector default-unconnected until those gates are tested; then
apply atomically after validated GNSS-first and before the main graph.
