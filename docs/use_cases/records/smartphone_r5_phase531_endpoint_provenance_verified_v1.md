# Phase531 — H TDCP endpoint coverage and invariance verified

Phase530 native PID 3878588 / session 47812 completed exit 0 in
412.6292914269725 seconds. No live job remains from that run. Output SHA256
4a0c4ef8822c72aa9134368cd57fe769817895e63b5dbe925783a9c8d091fa8e matches baseline.

Added verify_phase530_ionosphere_monitor.py. Passed source/binary/raw pins,
output identity, full graph/GNSS-first summary equality against Phase505 and
both termination stage contracts. Endpoint marker equals all 69270 built
TDCP factors, zero invalid, coefficient range .998451 to 5.31903. 507 valid
edges lack at least one exact retained code key. This confirms coverage is
not contingent on code-factor admission; it does not certify physical accuracy.
Prior IRLS/residual diagnostics also match their printed baseline values.
Endpoint parser passed one valid and seven invalid synthetic controls.

No truth or accuracy evaluation, no MAT/saved positioning input, no new
state/factor enabled. Existing behavior remains verified for this H replay,
not automatically every route or DD/legacy constructor (which retains zero).
Next connect the separately tested wrappers/shared temporal states in a
dedicated default-off main graph, with explicit parameter and scope contract.
GNSS-first must remain unchanged and original observations must be replaced,
not duplicated. Overall .782/LB goal is active and still unproven.
