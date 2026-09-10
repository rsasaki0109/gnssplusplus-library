# Phase320: paired native candidate regresses on H development data

The frozen Phase319 candidate was evaluated exactly once after the Phase320
manifest/evaluator freeze (`1affa3e7`). Pre-truth verification passed; all ten
existing Phase203 evaluator tests passed. Candidate and truth were each read
once, with 3139 exact matched rows and no interpolation, hold, or offset
reapplication. No solver was invoked during evaluation.

- P50: 2.025001392280304 m.
- P95: 2.478068941513089 m.
- Score: 2.2515351668966965 m.
- Operational baseline: 1.0769392017393964 m.
- Delta: +1.1745959651573001 m (worse).

Reject promotion of the paired configuration. Preserve the existing base-OFF
operational recipe and retain this failed experiment as evidence. The 0.782
gate failed; H is reused development data, not a heldout or leaderboard test.
No submission was made. Native convergence and raw-only in-memory operation
are established, but neither improved positioning accuracy nor source parity
follows from those facts.

This was a compound change: common transmission states, broadcast record
selection, zero explicit group delay, and base correction plus missing-stream
masking. The score alone cannot identify which caused the regression. Do not
attribute it to IMU, source station coordinates, or a particular constellation
without evidence. Do not retune correction magnitude or coordinates using H
truth, repeat this score, or restore missing corrections via extrapolation.

Next investigate without truth: trace the paired rover/base residual equations
and group-delay conventions, verify correction sign/interpolation against
synthetic known residuals, and quantify the post-mask constellation/clock
observability. Design any subsequent ablation before its evaluation rather
than adjusting parameters to this score. Raw header station-reference and
native orbit/rotation differences remain unresolved sources of uncertainty.
