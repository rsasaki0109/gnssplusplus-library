# Phase539 — runtime diagnosis from completed H telemetry

Read Phase536 native_summary.json main phase143_termination: 56 attempted,
56 accepted, zero rejected outer iterations; 56 total inner lambda attempts,
56 parsed trials and 56 native inner iterations. Terminated through
outer_convergence_tolerance, with strict cost decrease and finite costs.
Therefore repeated failed LM trials do not explain the 12.8x wall time.

Solver is MULTIFRONTAL_QR / EliminateQR with automatic COLAMD and no explicit
ordering, no diagonal damping and no fallback. This identifies the relevant
linear-solver path but does not prove fill-in is the cause. New wrappers add
scalar keys, including two endpoint keys to each TDCP factor, and delegate
base factor Jacobians. Both wrappers also allocate base Jacobian vectors even
for error-only calls. Profiling is required before attributing the slowdown
to either allocation overhead, linearization, ordering or elimination.

No pinned native source/binary was changed during U transfer. U session 81310,
PID 3932176 remains live at this audit. Do not rebuild/restart that experiment
to investigate performance. First preserve transfer output, then benchmark
an explicit diagnostic run without changing noise/priors or using truth.
