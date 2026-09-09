# Phase252 affine Pose3 TDCP factor

Added SourceAffineTdcpPoseFactor, not yet connected to production graphs.
ECEF previous LOS and endpoint antenna anchors are fixed constructor data;
the caller must provide carrier delta minus endpoint anchor-range delta.
Native Pose3 antenna positions and Jacobians use the existing LeverArm
conversion, including optional navigation-to-ECEF transformation.
Clock difference uses C[0] only. Invalid geometry throws; invalid clock
vectors or antenna positions return nonfinite residuals.

Build session 87558 completed. Fourteen selected tests passed across
Phase252/251/138/135. The new test independently constructs antenna anchors
from nested Pose3 transforms, uses nonidentity poses and frame rotation,
nonzero lever arm, and checks all 12 pose and 14 clock derivative columns
against central differences. Pose step is 1e-3 (radians for rotation,
metres for translation), tolerance 2e-6 to accommodate ECEF roundoff;
clock tolerance 1e-10. It also checks zero LOS rejection and NaN clock.

No full CTest or route accuracy claim. No raw/truth/MAT inputs used.
Next add default-off TDCP-only graph selection with exact same-run anchor
provenance and explicit Point3/Pose3 branches. Preserve all non-TDCP
factors and historical Phase135/138 recipes. Integration remains unproven.
