# Phase529 — raw carrier endpoint coefficient propagation

Added residual_ionosphere_coefficient to PreparedCarrierObservation and two
endpoint coefficient fields to FGO TimeDifferencedCarrierFactor. Both actual
carrier preparation paths calculate the coefficient from their own geometry
elevation and row_frequency_hz using the existing shared contract. Ordinary
TDCP assembly copies coefficients from the exact previous/current carrier
objects, not a lookup into accepted pseudorange factors. This covers code-
rejected carrier observations without nearest-time joins or saved trajectories.

Legacy/DD constructors retain default zero (unavailable); do not claim those
paths support the new joint model. Invalid physical inputs may produce NaN
via the helper; the future opt-in model must explicitly validate availability
and fail/report rather than silently invent a coefficient. Existing solver
does not consume these new fields. Measurement values, selection gates and
noise are unchanged by these edits; runtime invariance is still unproven.

Targeted diff check passed. Native target build started session 9037 and was
confirmed live compiling fgo.cpp/fgo_problems.cpp at record creation; this is
not a successful-build claim. Continue polling the same build, do not restart
on timeout. Remaining disk reported 1.7 GB: avoid large new debug/test builds
or downloads, and never delete unrelated user artifacts to make space.

Next verify compile completion, endpoint coverage on a raw replay and baseline
output identity before enabling any wrapped factor/state. Full model wiring,
parameter freeze and performance evaluation remain outstanding. No truth,
MAT or saved positioning input; overall goal remains active and unmet.
