# Phase216 Pose3 XXVV motion building block — synthetically verified

Primary-agent work. No raw run or truth access. The best recipe remains
Phase198/199; no new factor is inserted into a production graph yet.

Implemented `MotionFactorPose3XXVV` in `fgo_gtsam_internal.hpp` for
`translation(P2)-translation(P1)-(V1+V2)*dt/2`, matching the existing Point3
XXVV residual. Pose Jacobians use the actual GTSAM translation Jacobians,
not identity blocks that would be wrong for nonidentity pose rotation.
Velocities and positions are in the same ENU frame. The intended lane must
require zero lever arm; this factor does not model an antenna offset.
Invalid nonpositive/nonfinite duration throws; nonfinite states return
nonfinite error rather than a fabricated residual.

Added `FGOGtsamPhase216Pose3MotionTest`: nonidentity rotations, a constant
acceleration/trapezoidal-motion zero-residual control, equivalence with the
existing Point3 factor, numerical derivatives for all pose and velocity
columns, an inconsistent-velocity residual/likelihood check at Street sigma
0.05 m, and invalid-duration checks.

Build session **6128** completed successfully. The fresh binary passed
**6/6** tests: Phase216 Pose3 residual/Jacobians, Phase164 observability
controls and Phase171 IMU handoff tests. The derivative check covers all
12 pose tangent and 6 velocity columns with nonidentity rotations. This is
not a full CTest result or proof of an XXVV-augmented main graph's rank.
The factor is not yet inserted in production. Next add an opt-in
main graph branch with explicit counts and source time-gap gating. For the
first isolated raw comparison, retain Phase198 best legacy IMU and keep the
new main Doppler option OFF; do not change this choice after seeing truth.
The source main-D plus XXVV combination can be separately evaluated later.

This is a factor implementation and pending test, not accuracy improvement,
full source parity, or completion of the raw-only performance objective.
