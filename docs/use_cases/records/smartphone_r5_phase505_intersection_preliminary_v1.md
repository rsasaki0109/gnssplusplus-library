# Phase505 — preliminary exact same-row intersection

Live native run PID 3801727 / session 44009 emitted:

- raw candidates: 426
- candidate shadow geometry/quality pass: 420
- baseline-median residual pass: 243
- residual pass with retained problem epoch: 243
- factors added: 0

Unlike Phase503's broad 807-row pool, these are the same-row label intersection.
Runtime remains active; output/graph/handoff invariance verification must wait
for completion. No clean-observation claim, truth scoring or readmission.

Implementation audit for a possible later bounded experiment: shadow rows
currently lack the full native P-factor geometry/clock/provenance fields, so
they cannot be turned into factors by attaching only a residual and epoch.
A future pool must reuse the existing complete factor construction, leave
ordinary epoch admission/GPS clock-jump bookkeeping untouched, exclude the
pool from baseline residual medians and GNSS-first, then insert only selected
complete factors into the main graph with exact retained epoch mapping.
Do not rebuild medians or estimate corrections from these candidate residuals.

243 candidates are an experiment scope, not evidence that readmission helps.
Temporal self-consistency can preserve persistent bias, as tested explicitly.
Keep defaults off and require single frozen-run structural checks before any
additional development evaluation. Current sources remain pinned unchanged
until the live diagnostic run terminates.
