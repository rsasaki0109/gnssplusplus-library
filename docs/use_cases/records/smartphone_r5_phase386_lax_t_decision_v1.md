# Phase386 — LAX-T code-gate alternative not promoted

Both evaluation conditions frozen in 7638d459 before scoring. Each
candidate and truth was read once per score (two truth reads total).
1465 keys match exactly for both predeclared first-epoch projections.
No interpolation, hold, additional exclusion, offset reapplication or
submission. Original 1466-row native outputs remain preserved.

Sparse-staging baseline: 3.1024422817253834 m (P50 2.8641256260803294,
P95 3.340758937370437). Code-gate OFF: 3.1102744912936076 m (P50
2.885132326693796, P95 3.335416655893419). Delta +0.007832209568224169 m.

Code-gate removal worsens H and LAX-T and improves U under the tested fixed
rule. Do not promote globally or use route-ID score-based switching. The
0.782/LB objective is unmet. All three routes are reused development.

Sparse staging removes a whole-route admission failure and preserves exact
coverage, but successful convergence is not adequate accuracy. The roughly
3.1 m LAX-T error is materially larger than the gate's millimetre-level
effect. Prioritize diagnosing this raw-native baseline transfer weakness,
not further code-gate threshold tuning. Historical Phase146 used another
recipe and cannot isolate the cause or serve as inference input. Compare
configuration/measurement provenance, not historical coordinates or
truth-derived offsets. Keep production defaults unchanged.
