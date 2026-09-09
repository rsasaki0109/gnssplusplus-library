# Phase502 — downstream residual diagnostic for masked code

Current builder excludes P-D-masked rows before constructing P factors and
computing whole-route system/band residual medians. Adding those rows into
the normal pool just to audit them would change the medians and confound
the experiment.

Added a separate shadow list for Phase171 builder P-D-masked rows that reach
geometry/correction evaluation and pass SNR/elevation/finite upstream sigma.
It stores only group/signal/native seed residual, never contributes to graph,
clock-jump GPS tracking, seed inputs or median estimation. After ordinary
medians are frozen, counts shadow rows passing the ordinary centered-residual
threshold and unavailable residual/median cases. Aggregate stderr only.

Scope is all eligible masked code rows, not the 426 outside-edge-supported
identities from the raw audit. These lists are not yet intersected. Counts
refer to rows at this builder location, not proof every row's epoch is later
retained or every downstream condition passed. Explicit identity/epoch mapping
is required before readmission. No production selection change.

Build started (session 59914); git diff --check clean. No raw native replay,
output-invariance verification or truth evaluation of this diagnostic yet.
Goal remains unmet; next verify diagnostic behavior then exact candidate scope.
