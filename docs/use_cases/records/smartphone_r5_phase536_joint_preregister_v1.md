# Phase536 — first joint residual-ionosphere experiment, frozen before scoring

One candidate: segment anchor sigma 3 m, random-walk density 0.02 m/sqrt(s),
gap split 1.5 s. These are engineering trial assumptions, not measured noise,
an upstream parity claim or an optimum. The anchor allows meter-scale residual
vertical L1 error; the walk permits 0.02 m standard deviation per second and
approximately 0.155 m over a minute. Gap policy matches the existing 1.5 s
motion continuity threshold. No truth/leaderboard sweep selects these values.

Use the exact Phase535 raw H recipe, adding only the joint CLI switch.
GNSS-first must be unchanged. Wrap every accepted code/ordinary TDCP factor
once; add one scalar per epoch and one anchor/walk prior per epoch. H expects
3140 extra states and 3140 extra factors, with 101916 code and 69270 TDCP
bindings. Original initial data-only diagnostic rows are not joint posterior
statistics. Existing legacy residual-ionosphere counters are not this lane.

First gate: current binary disabled replay output/graph/GNSS-first identity.
Candidate gates: successful finite solve, complete bindings, expected graph
increments, identical GNSS-first and epoch coverage, finite solved-state and
correction diagnostics, and both numerical lambda-floor stage contracts.
Numerical success is not an accuracy claim. Inspect state magnitudes before
evaluation; do not silently clip. Do not modify candidate priors using H truth.

If valid, evaluate this single frozen candidate against the matched baseline
using the established evaluation-only boundary; check U and LAX transfer with
the same priors before any promotion. These already explored routes are
development checks, not an untouched generalization test. A later untouched
evaluation and actual leaderboard evidence remain necessary for the goal.
No MAT, saved positioning input or truth-dependent inference is permitted.
