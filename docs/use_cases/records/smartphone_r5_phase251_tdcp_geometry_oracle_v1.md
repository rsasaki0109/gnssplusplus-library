# Phase251 TDCP geometry oracle

Added a synthetic comparison of actual Phase135 affine and native
source-clock Point3 TDCP factors. Build session 71681 completed; 13 tests
passed across Phase251 geometry, Phase138 TDCP and Phase135 affine suites.
No raw/truth/MAT payloads or native route solves.

The new test isolates a common Cartesian frame without Sagnac/atmosphere.
It uses moving satellite endpoints, nonzero C[0] change, and the explicit
endpoint anchor-range difference in the affine measurement constant.
Both residuals agree at their anchors. Previous-position and clock
Jacobians agree; current-position Jacobians differ because source uses
the previous LOS while nonlinear geometry uses the current endpoint LOS.
A transverse 100 m receiver perturbation produces a nonzero residual
difference checked against independently computed Euclidean ranges.

This is a model difference, not a bug proof or accuracy improvement.
It does not validate whole-route affine performance, rotated Pose3
Jacobians, lever arms, or Sagnac conventions in a new combined lane.

Next design a TDCP-only default-off affine factor selection compatible
with current main Pose3 and GNSS-first Point3 state types. Preserve P/D,
noise, masks, and endpoint anchor-range constants. Use only same-run
native initialization, and test nonidentity Pose3/frame Jacobians before
any raw experiment. Do not turn on historical Phase135 wholesale: that
would change other measurement families and historical recipe guards.
