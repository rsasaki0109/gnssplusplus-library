# Phase473 — corrected lambda diagnostic verified

Phase472 completed once, exit 0, 230.802035050001 seconds. Session 78369
closed. Manifest SHA256:
ecd7f3619ca8119345f4fd2c0dfe87b245acb03f4daaccb6d2b461383432878f.
verify_native_imu_bias_diagnostic.py 472 passed pins, convergence, baseline
output identity, all preceding diagnostics and the failed-lambda check.
Output SHA remains d2c619121951d1a322ab39e15fcf16dbf7a6d61d2c651d03a4f68ea357c61e7d.

Two aggregates in the frozen two-stage execution order:

| Stage | Failed linear trials | Min lambda | Max lambda |
|---|---:|---:|---:|
| GNSS-first | 80 | 1e-9 | 1e-9 |
| Main | 91 | 1e-9 | 1e-9 |

Main count agrees with summary's existing 91 failures. Nearby variable keys
remain unavailable. The diagnostic bug from Phase469 is resolved for this
actual raw execution. No scoring/truth read, no accuracy improvement claim.

This supports a bounded numerical-efficiency candidate: an opt-in minimum
lambda of 1e-8 (the next existing factor-of-ten level above every observed
failure), leaving factors, constraints, masks and all other solver settings
unchanged. This is prospective development evidence, not an untouched test.
Do not infer fewer retries or identical results before running it. First
require default-off regression coverage, then one pinned LAX comparison of
output identity, costs, iterations, failure count and runtime. If identical,
there is no need to read truth; runtime needs repeated controlled timing
before a speed claim. Extend to H/U before any general promotion.

This numerical candidate does not replace the original accuracy objective.
The 0.782-class/LB target remains unmet; an identical-output speedup alone
cannot complete the goal. No production setting changed in this phase.
