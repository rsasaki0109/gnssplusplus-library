# Phase470 — Phase469 diagnostic failure and correction

Phase469 native replay completed once: exit 0, 218.31619336793665 seconds,
session 90724 closed. Verifier passed all source/input/binary pins, baseline
output SHA, convergence and earlier diagnostics, then failed the required
failed-lambda aggregate check. No such aggregate was emitted. Do not call
this a successful new diagnostic experiment or repeat Phase469 automatically.

Cause verified in native and GTSAM sources: active_solve_diagnostic enables
Phase96 TRYLAMBDA verbosity; GTSAM prints the six-column SUMMARY only when
verbosity equals SUMMARY, not for TRYLAMBDA. The new range accumulator used
only SUMMARY attempts. The existing count of 91 failures comes instead from
the full parsed Phase96 trial list, so it remains nonzero and valid while
the added accumulator remained empty. The prior SUMMARY format test did not
cover the runtime-selected verbosity. This is a diagnostic wiring failure,
not a new positioning or optimizer failure.

Corrected accumulator to use the same selected trial source as the existing
failure counter: full Phase96 trials when enabled, otherwise SUMMARY rows.
No double counting; log emission occurs after the selected accumulation.
This source correction is not yet rebuilt or replayed. Require an end-to-end
TRYLAMBDA diagnostic regression before another raw run. The previous manifest
pins describe the failed-diagnostic build and must remain unchanged.

No truth read, scoring, MAT or model/optimizer-parameter change. Accuracy goal
still unmet. The baseline-identical run does not establish improvement.
