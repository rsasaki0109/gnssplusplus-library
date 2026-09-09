# Phase324: correction transaction failure and missing-epoch smoothing

No truth, candidate trajectory, or raw input was read in this audit. Inspected
the existing algorithm source and native correction implementation.

## Reproduced and fixed transaction defect

`source_pseudorange_miss_mask::applyImpl` modified caller-owned factors inside
the loop before its final accounting check/vector swap. A synthetic callback
that succeeds once and throws on the second call left the first caller factor
corrected and marked applied. The new test failed on both invariants before
the fix. Work now proceeds on a per-factor copy; exceptions propagate without
partially changing the caller's vector. This does not establish that a callback
exception caused Phase319's regression: that run succeeded without this event.
Reports after exceptions remain incomplete and must not be treated as success.
After the fix, `gnss_run_tests` built and all 36 base-compensation tests passed.
This is not full CTest or a real-data byte-identity gate. No inference rerun or
score change is claimed for this exception-safety correction.

## Smoothing discrepancy to address next

`correct_pseudorange.m` smooths an epoch-indexed satellite residual column and
interpolates on the base observation time vector. Native `Model::build` appends
only finite, admitted residual rows into each stream, then smooths that compact
vector. Thus missing epoch positions no longer consume window width. Native
interpolation also operates on compact stream timestamps, not a dense grid.

A synthetic test makes the consequence explicit: with a 3-sample window,
`[0, NaN, NaN, 90]` has endpoint means 0 and 90, while compacting to `[0, 90]`
makes both means 45. The mean helper already handles missing values; its caller
must preserve the base epoch grid. This is a concrete algorithm discrepancy,
not proof of its size or effect on the measured H positioning score.

Next implement an explicit dense-epoch correction path that preserves missing
samples through smoothing/interpolation, including all-missing windows and
start/end support. Do not merely filter missing outputs afterward and bridge
long NaN intervals. Keep historical compact behavior available for existing
frozen runs; no truth-derived window tuning or coordinate offset is justified.
