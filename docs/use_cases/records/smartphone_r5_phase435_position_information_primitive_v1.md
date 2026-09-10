# Phase435 — clock-projected pseudorange information primitive

Added Eigen-only diagnostic `clockProjectedInformation(A,B,sigma)`: whiten
position/clock Jacobians by positive finite nominal metre sigma, project A
out of the numerically observed column space of B by SVD, return AᵀA after
projection. Absent/redundant clock slots do not get artificial priors/damping.
Empty observation sets return zero information; invalid/nonfinite inputs fail.

Standalone C++ tests passed 5/5, registered in tests/CMakeLists.txt. Controls
cover analytic shared-clock projection, inverse-square sigma scaling, fully
independent clocks removing all information, absent/redundant slots, empty
and invalid inputs, and heterogeneous-sigma C7 clock-gauge invariance.

Native source audit confirms the active pseudorange branch uses component
mapping from raw_p_seed::c7ClockComponentFor and sourceClockComponentJacobian:
C[0] plus selected component, using assignment for component 0, not doubling.
Pose3 uses antenna-position/lever-arm geometry. A production diagnostic must
reuse those exact mappings and same-run antenna locations, not saved positions.

Not yet connected to native factors. Nominal single-epoch P information is
not full FGO observability or posterior covariance: it omits robust residual
weights, temporal clock constraints, TDCP, Doppler and IMU information. Label
it accordingly and do not infer a new factor/noise change from algebra alone.
No raw execution, truth evaluation or performance claim in this phase.
