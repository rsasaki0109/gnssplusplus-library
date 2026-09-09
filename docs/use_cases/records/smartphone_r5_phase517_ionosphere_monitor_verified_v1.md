# Phase517 — H diagnostic replay verified

Phase516 runner session 98914 terminated with exit 0; native PID 3836972
completed after 413.36027169495355 seconds. Do not rerun that output directory.
Output SHA256 4a0c4ef8822c72aa9134368cd57fe769817895e63b5dbe925783a9c8d091fa8e
matches the floor-only baseline exactly. No coordinate parsing or truth read.

New scripts/verify_phase516_ionosphere_monitor.py exited 0. Checked current
source, binary and raw input hashes, completed return code, exact output hash,
entire graph and gnss_first summary equality versus Phase505, plus both
phase143 termination stage contracts. Monitor parser passed a valid fixture
and six invalid controls (missing, duplicate, invalid count, NaN, fraction
outside range and zero epochs). Initial verifier expectation of 3139 confused
IMU intervals with problem epochs; corrected from authoritative baseline
summary to 3140, not by weakening the monitor validity check.

Aggregate result:

- 3140 measured epochs, 0 invalid.
- Median remaining/total coefficient information: 0.00483834 (~0.484%).
- Median projected scalar information: 0.895282 per square metre.
- Graph unchanged: 252384 factors, 15700 values, 43 main iterations.

Interpretation boundary: most of this coefficient column is explained by
local position/C7 nuisance columns, but nonzero remaining information exists.
This is not evidence that residual ionosphere is zero, not a prediction of
position accuracy, and not full time-coupled observability. Uses Gaussian
nominal sigmas rather than Huber weights, stored SPP-elevation coefficients
and current same-run handoff position Jacobians. A scalar reciprocal-information
uncertainty would only apply to that simplified local linear model, not the
whole FGO or a physically validated ionosphere estimate.

No legacy guard removed, no new ionosphere states/factors inserted, no accuracy
evaluation or promotion. Next examine robust-weight sensitivity and whether
the remaining component corresponds to consistent raw residual structure before
integrating a new C7-aware factor. H is reused development, not heldout data.
Overall .782/LB objective remains unmet; no live native/build job remains here.
