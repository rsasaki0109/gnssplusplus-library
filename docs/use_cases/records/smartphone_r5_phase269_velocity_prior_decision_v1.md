# Phase269: first velocity prior omission decision

Frozen evaluation commit: `474b49d`. The metadata and scoring-kernel test
selection passed 25 tests; this is not a full CTest run.

One authorized candidate read, one truth read, and one score calculation
completed. All 3,139 H development keys matched exactly, without interpolation
or edge hold. All integrity gates passed; the 0.782 m gate failed.

- Route score: 1.0769276297134343 m.
- P50: 0.8369149295600797 m; P95: 1.3169403298667886 m.
- Delta against Phase235: -0.000011572025962136578 m (about -0.012 mm).

This is a numerically lower development scalar, not evidence of a meaningful
or generalizable accuracy improvement. Do not promote velocity-prior omission
on this evidence alone; retain the Phase234 recipe as the operational baseline.
The new candidate is the lowest observed H scalar, but H has been repeatedly
used for development and is not heldout or leaderboard evidence.

No native rerun, MAT access, precomputed positioning input, submission, or
Kaggle-token access occurred during evaluation. Candidate positioning output
was consumed only by the frozen evaluator, never as solver input.

The bias and velocity prior ablations have not explained the remaining gap.
Next investigate a concrete observation/model discrepancy through raw-data
or synthetic evidence before choosing another accuracy experiment. Do not
remove the initial pose prior without an observability/nullspace analysis;
do not start a prior-strength sweep from these tiny scalar changes.
