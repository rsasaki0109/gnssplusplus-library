# Phase341: diagnostic FGO admission removes low-elevation difference overlap

Extended the Phase340 diagnostic to build native FGO pseudorange admission
from raw phone observations and navigation in the same process. SPP seeds
are ephemeral raw-derived values, never loaded/saved receiver solutions.
Enabled shared rover epoch states, multi-constellation, multi-frequency DD,
upstream observable quality and source-complete zero explicit group delay.
Other FGO settings remain defaults. Match admitted factors by exact GPS
week/TOW, satellite and signal to the raw phone difference queries.

Successful standalone C++17/O1 build and one raw pass exited 0. All Phase339
and Phase340 statistics repeated exactly. New results:

- diagnostic FGO pseudorange factors: 101536
- admitted finite difference queries: 81362
- admitted queries with difference >1e-6 m: 0
- maximum admitted difference: 0 m

Thus the 676 changed raw queries do not survive this diagnostic FGO
admission. This materially weakens the low-elevation troposphere hypothesis;
do not implement/promote an unclamped atmosphere change as an accuracy fix
on this evidence. The diagnostic is not the full Phase326/329 operational
recipe (factor count differs), does not apply the production correction
model transaction, and cannot prove zero exposure for every recipe. It
nevertheless shows that raw overlap alone was misleading for solver impact.
No optimization, truth/candidate reads, accuracy score or production change.

Executable: `/dev/shm/gnss-test-build-recovery.BJvQt9/fgo_atmosphere_overlap_audit`.
Inputs: same three raw H paths as Phase340. This remains an exploratory
diagnostic, not a hash-frozen production reproduction. Next investigation
should prioritize remaining correction-reference or measurement-model
differences rather than tune the low-elevation discrepancy against H truth.
