# Phase520 — actual-factor initial IRLS information verified

Phase519 native PID 3846318 and runner session 68532 completed, exit 0,
412.077510530944 seconds. Output SHA256:
4a0c4ef8822c72aa9134368cd57fe769817895e63b5dbe925783a9c8d091fa8e.
Exact baseline output equality holds. No live native job remains from this run.

Added scripts/verify_phase519_ionosphere_monitor.py; execution exited 0.
Checks frozen source/binary/raw-input hashes, output hash, full graph and
GNSS-first summary equality versus Phase505, both termination stage contracts,
one finite monotone-information monitor, 3140 problem epochs and all 101916
retained code rows. Added tests/test_ionosphere_irls_monitor.py: 2 tests passed,
including nine invalid fixtures. No coordinate interpretation or truth access.

Actual initial graph diagnostics:

- Epochs 3140; rows 101916; invalid 0.
- Downweighted rows 98767.
- Median nominal projected information 0.895282078029 m^-2.
- Median IRLS projected information 0.0802815904466 m^-2.

These use the same rows, actual Pose3/C7 Jacobians and stored residual-ionosphere
coefficients. Nominal median agrees with Phase516 at its printed precision.
The IRLS median is much smaller but positive; the ratio of these medians is
not the median per-epoch ratio. Downweighted counts refer to the initial
handoff state and Huber threshold, not a ground-truth bad-observation label.
Neither statistic is a whole-graph accuracy bound or exact robust Hessian.

Decision: an independent per-epoch ionosphere correction based solely on code
would have limited local support in this initial robust model. Do not loosen
Huber or prior strength based solely on these reused H statistics. Before
adding a state, inspect the corresponding projected residual and temporal
consistency (or optimized-state diagnostic) to distinguish actual residual
structure from a merely available degree of freedom. No production correction,
accuracy evaluation or candidate promotion occurred. The overall goal remains
unmet and active; all data used for inference stayed raw and same-run in memory.
