# Phase207 decision: do not promote bias-density-only change

Primary-agent work, without subagents. Evaluator and candidate metadata were
frozen in commit `654635e` before the single authorized evaluation.
Metadata/authorization tests plus the unchanged synthetic metric suite passed
13/13. The native solver was not rerun for evaluation.

The Phase206 native raw-only candidate scored **1.2756036693774828 m** on the
already-used H development/train route, against Phase199's
**1.2751561666667786 m**. Delta: **+0.00044750271070426173 m** (worse).
P50: 0.7559639190497712 m; P95: 1.7952434197051945 m.
All 3139 candidate/truth keys matched exactly, all coordinates passed finite
and Earth-range checks, and no interpolation, edge hold, offset reapplication,
or over-70-m/s segment was reported. Candidate and truth were each read once;
the metric was calculated once.

Do not promote Phase205 into the best recipe. Retain the default-off option
and synthetic covariance tests as evidence, not as an accuracy improvement.
The sub-millimetre score difference establishes no generalization or
statistical-significance claim. Phase198/199 remains the best measured recipe.
The raw-only native 0.782-class and leaderboard objective is **not achieved**.

## Next algorithmic investigation

The Phase204 audit identified differences that density scaling alone does not
remove: source ending-bias ImuFactor versus native starting-bias
CombinedImuFactor, motion/bias covariance correlations, and the finite native
initial bias prior. The next useful experiment is an opt-in source factor
formulation, with synthetic residual/Jacobian/covariance tests before raw
execution. Keep the legacy integration schedule and the established -20 ms
offset fixed so schedule changes do not confound that comparison. Do not call
this full source parity while scheduling and initial-prior differences remain.

H remains development data. Broader route-grouped validation, with historical
truth-use auditing, is still required before any heldout or leaderboard claim.
