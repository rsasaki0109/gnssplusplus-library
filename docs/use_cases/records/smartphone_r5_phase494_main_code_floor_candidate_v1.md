# Phase494 — stage-isolated main code uncertainty floor

Added default-off --native-main-code-uncertainty-floor. Scope requires raw
Android Phase171 ECEF-D; forbids combining with the builder-level floor,
batch NHC or frequency residual states. It does not set the builder floor
config flag. The modification is after same-run GNSS-first, IMU preparation
and main row selection, immediately before main preflight and solve.

Frozen policy: each available positive finite source uncertainty greater
than the existing sigma replaces sigma, with no multiplier or clipping.
Missing/invalid uncertainty leaves sigma unchanged; invalid existing sigma
fails closed. Only sigma and its applied metadata flag change. Diagnostic
reports retained/changed counts and stage scope. Counterfactual monitor is
suppressed in this mode because post-floor ratios would mislead.

Backend constructs main P noise directly from factor.sigma_m (around line
1804), so this affects the actual C7 likelihood, not merely metadata. Robust
loss remains enabled with its existing normalized threshold: increasing sigma
also expands the physical residual scale at which robust downweighting starts.
This is part of the candidate, not a claim of pure covariance-only behavior.

Expected H affected count is 101916 based on the Phase493 raw diagnostic.
GNSS-first equality, unchanged observation/graph counts, convergence and
stage isolation must be verified on the actual candidate before scoring.
Main rebuild started; no candidate run or truth evaluation yet. Default off,
no promotion, no .782/leaderboard achievement claim.
