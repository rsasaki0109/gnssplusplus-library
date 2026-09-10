# Phase426 — paired frequency residual-state candidate not promoted

Phase422/423 diagnostic-only raw replays completed with exit 0. Both verified
the original Phase416/417 candidate SHA, exact raw output keys, convergence,
one inserted Gaussian prior per frequency state, two wrapped scalar TDCP
factors per state, and corrected app/backend TDCP RMS agreement within 1e-7 m.
H elapsed 279.6905650959816 s; LAX-T 150.29556158394553 s. These timings are
not a controlled speed benchmark.

Phase425 frozen development evaluation then completed once for each route.
Manifest SHA256 `b63cabab93e6f9cf57d19ed28445fed37f5f16e3d979f5c9f86f6a39225eaabd`.
Each truth payload was read once by the unchanged scoring kernel. No native
inference was invoked during evaluation; LAX-T used the predeclared first-row
projection only for evaluation, never as inference input.

| Route | Baseline m | Frequency candidate m | Delta m |
|---|---:|---:|---:|
| H | 1.0769392017393964 | 1.1480541172596557 | +0.07111491552025928 |
| LAX-T | 3.1024422817253834 | 3.105028107445106 | +0.0025858257197226564 |

Metric is (P50 + P95)/2; lower is better. Exact evaluation-domain coverage and
finite-position gates passed. Both routes regress. Do not promote this fixed
0.01 m prior candidate, switch it per route, or claim improvement from lower
fitted TDCP residuals. Existing default remains OFF. No post-score prior sweep
is authorized by this experiment design. The result does not disprove every
frequency-dependent error model, but rejects this prospective candidate.

These repeatedly used development routes are not heldout/LB evidence; the
0.782-class and leaderboard objective remains unmet. Next investigate a
distinct, raw-only source of error using non-truth diagnostics and historical
negative results before proposing another fixed candidate. Preserve all
experiment records and original inference outputs; no submission was made.
