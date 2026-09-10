# Phase528 — explicit temporal topology for shared residual states

Added ionosphere_temporal_plan.hpp: pure plan over ordered retained epoch times
and exact caller-supplied reset flags. One anchor per segment; adjacent epochs
within supplied max gap receive zero-mean random-walk edges with sigma equal
to supplied density times sqrt(dt). Gap or reset starts a new segment. Bad
time order is rejected even on a reset, not disguised as a valid new segment.
No noise density/max gap defaults, fitted corrections, state values or graph
insertion supplied by this helper. Caller still owns anchor sigma and reason
for resetting. A hardware reset is not proof that physical ionosphere jumps.

Added/registered test_ionosphere_temporal_plan.cpp. Fresh standalone binary
/tmp/phase528_temporal_plan passed 3/3: exact edge/anchor cardinality, gap and
reset boundaries, sqrt-time density, week rollover, empty input and malformed
configuration/time rejection. Full CTest not run; no production replay.

This is a topology primitive, not yet the actual GTSAM temporal factors.
Integration must specify physical prior parameters and handle TDCP edges that
span any chosen segment boundary without silently dropping or duplicating raw
measurements. Key mapping must be exact retained indices, not timestamp-nearest
joins. Do not use an independent anchor per band/satellite for this vertical
shared-state model.

Next raw provenance: prepareCarrierObservationsForReceiver already computes
geometry elevation and frequency before corrected carrier construction;
PreparedCarrierObservation stores elevation. Propagate endpoint coefficients
through the actual TDCP assembly, not accepted code-factor lookup (carrier
rows can survive code rejection). This work and full graph default-off wiring
remain outstanding. No truth/MAT/saved positioning input or accuracy claim.
Overall objective remains active and unmet.
