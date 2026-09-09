# Phase541 — fixed joint model fails U transfer

After finite-state/provenance verification and magnitude review, froze U
candidate and matched baseline hashes, then ran the existing exact-key scorer
once per output. Truth provenance and 1102 rows inherited from Phase247;
baseline recipe from Phase480 includes the first native epoch. No solver or
parameter changes from H; evaluation-only reads, not inference inputs.

U development (P50+P95)/2: baseline 1.3059603092036585 m; joint candidate
2.1108052382243216 m; delta +0.8048449290206632 m. Candidate P50
2.000191977304861 m, P95 2.221418499143782 m. Full 1102/1102 exact matches,
finite and zero over-70-m/s events. Numerical validity did not ensure accuracy.

Reject promotion of this fixed shared-ionosphere model: H gained 1.74 cm but
U degraded 80.5 cm. Large U fitted state may be absorbing other errors, but
this is a hypothesis, not a demonstrated causal attribution. Do not sweep
priors against U truth to rescue it. Preserve negative result.

LAX session 38740 / native PID 3941214 remains the already-launched fixed
experiment; inspect authoritative live state before waiting or verification.
Next finish that run, then revisit observability/model separation using raw
measurements and synthetic controls rather than truth-dependent tuning.
Goal remains unmet; no promotion, submission or leaderboard claim.
