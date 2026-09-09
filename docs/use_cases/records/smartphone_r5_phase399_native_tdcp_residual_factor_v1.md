# Phase399 — native GTSAM residual-state factor prototype

Added `tdcp_residual_state_factor.hpp`: a scalar NoiseModelFactor wrapper
that reuses the original TDCP factor's noise model, unwhitened residual and
original-key Jacobians, then appends -alpha*u and a scalar Jacobian -alpha.
The original factor is held read-only. Invalid scalar dimensions, missing
noise, nonpositive/nonfinite coefficient, key collisions, nonfinite state or
overflow fail explicitly. Clone and equality include wrapper semantics.

No backend/CLI selector calls this wrapper yet. It is not a new default and
does not change existing output. Graph integration must replace, not append
to, a selected original factor, and add one prior per shared paired state.
The wrapper itself cannot validate satellite/epoch identity or prior scale.

Fresh standalone GTSAM-enabled build and execution passed 3/3 tests using
the actual `TimeDifferencedCarrierFactorSourceClockPoint` native factor:
zero-u residual/cost/noise/Jacobian identity, nonzero-u sign, numerical state
derivative, linearization, clone/equality, bad key/coefficient/state checks.
Only synthetic geometry, no raw, truth, MAT or saved positions used. This is
not a complete graph solve or Pose3-arm Jacobian test; those remain required.

Test source registered in existing CMake list, guarded for optional GTSAM.
Full CTest and real-data accuracy remain unrun. Next add Pose3/C7 and paired
graph optimization controls, then exact same-satellite/endpoint pairing and
an explicit default-off backend path. Do not pick a nuisance-prior scale
from reused development truth. Objective remains active and unachieved.
