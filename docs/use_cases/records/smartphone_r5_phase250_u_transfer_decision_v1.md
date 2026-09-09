# Phase250 complete U development transfer

Frozen evaluator commit 138354b; 13 selected metadata/synthetic tests pass.
One score, candidate/truth each read once in this evaluation. Phase247
previously read U truth once and failed before computing a score; these
are separate immutable attempts, not a claim of only one lifetime read.

U score: 1.305940315797287 m, P50 1.0548006190679335 m,
P95 1.5570800125266404 m. All 1102 prediction/truth keys match, including
the first epoch; finite/Earth-valid, no over-70-m/s violations,
interpolation, hold or offset reapplication.

The coverage defect is resolved for this run. The 0.782 m objective is
not met. H reference remains 1.0769392017393964 m; do not compare route
scores as an improvement delta, or call either route heldout. U was used
historically and is explicitly development. No leaderboard claim.

Retain estimator settings; do not tune thresholds to U. Next investigate
the still-open source affine versus native nonlinear TDCP geometry with
an independent synthetic equation/Jacobian oracle, including endpoint
satellite motion and clock terms. Historical Phase135/138 are constrained
recipes and cannot be assumed compatible with the current native lane.
Use raw-only same-run seeds if a new implementation is justified.
