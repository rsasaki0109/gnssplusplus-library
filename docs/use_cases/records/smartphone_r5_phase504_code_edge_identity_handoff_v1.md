# Phase504 — same-row diagnostic handoff to builder

Reusable codeEdgeCandidates helper passed two direct gtests (exact neighbors,
missing witness, clock coverage/break, duplicate identity) and reproduced H
426 candidate count in the native raw audit. It is a diagnostic candidate
extractor, not a proof of clean code or an admission rule.

Added default-false native_code_edge_diagnostic_candidate to Observation.
For raw all-epoch Phase171 with skip=0, app computes candidates on the raw
epoch vector with its aligned hardware-clock counts before moving epochs.
It marks the uniquely matched satellite/signal row in place; subsequent row
copies carry the diagnostic flag. There is no serialized candidate input,
nearest-time matching or post-filter row-index join. The selector's unique
pair checks exclude ambiguous rows.

Shadow builder stores this flag and its input epoch index. It now reports
candidate geometry/quality pass, candidate residual pass against unchanged
medians, and candidate residual pass whose input epoch is in the actual
problem_to_input_epoch mapping. Shadow rows never enter graph or medians.
Counts are pending an actual replay; do not claim intersection size yet.

Header change triggers dependent rebuild (session 5222), in progress. Old
standalone binaries/objects may have stale Observation layout; rebuild before
using them as evidence for current code. git diff --check clean. No truth
evaluation or promotion; default measurement selection unchanged.
