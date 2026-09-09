# Phase526 — two-endpoint residual-ionosphere TDCP factor

Implemented code_ionosphere::TdcpEndpointsFactor wrapping an existing scalar
TDCP factor, sharing original geometry, C7 units, Jacobians and robust noise.
For prediction-minus-measurement residual add a_prev*I_prev - a_curr*I_curr;
I is vertical residual AFTER the existing atmospheric correction, not total
ionosphere. Different endpoint mapping factors are explicit. A constant
vertical residual need not cancel when elevation changes. No graph insertion.

Actual C7 point-TDCP fixture checks zero-state original residual/robust cost
and all original Jacobians, both new analytical/numerical derivatives, unequal
mapping with constant vertical state, clone equality and key/coefficient guards.
Added to existing registered test_tdcp_residual_state_factor.cpp. Fresh build
and run /tmp/phase526_tdcp_endpoints passed 6/6 (five existing, one new).
Full CTest not run. Actual Pose3 wrapper coverage for this new two-endpoint
class and joint code+TDCP synthetic recovery are still outstanding.

Integration requirements, not yet implemented:

- One vertical residual state per retained main epoch, shared across code
  and TDCP, never independent states for each band or duplicated observations.
- Code coefficient positive; TDCP has positive previous and negative current
  endpoint coefficients. Both must use a consistent frozen geometry convention.
- The current TimeDifferencedCarrierFactor struct has no endpoint ionosphere
  coefficients: supply them from same-run raw carrier preparation, including
  rows without accepted code; do not nearest-match code rows or saved states.
- A temporal prior/reset contract must specify gap/discontinuity handling and
  one segment anchor. No arbitrary per-epoch prior tuning from H diagnostics.
- Preserve current C7/Phase171 guards until the dedicated default-off path is
  complete; leave GNSS-first unchanged. Zero state equality is necessary but
  not sufficient for complete graph equivalence or improved positioning.

No raw run, truth/MAT/saved-position input or score claim. Overall goal active.
