# Phase518 — actual GTSAM Huber diagnostic controls

Inspected makeNoise and PseudorangeFactorSourceClockArm: sigma is clamped to
at least 1e-9, scalar base Gaussian is optionally wrapped by Huber, and residual
is antenna range + C7 clock projection - corrected code. A diagnostic using
upstream_seed_residual_m would be stale after GNSS-first and lacks full C7.
Do not use that field as the main-stage robust residual. Pose/lever-arm and
clock initialization must match the actual graph when constructing weights.

Added tests/test_ionosphere_huber_information.cpp, using linked GTSAM Huber
weight/loss (not a guessed reimplementation), registered under GTSAM_FOUND:

1. Weight consumes standardized residual; effective sigma is sigma/sqrt(weight).
2. For fixed linearized coefficients/nuisance columns, reducing row weights
   does not increase projected absolute information (tested on a fixture).
   This does not assert monotonicity of projected/total information ratio.
3. Huber tail exact loss curvature is zero while its IRLS weight stays positive.
   Therefore the proposed weighted information must be labelled a local IRLS
   approximation, not exact robust Hessian or calibrated posterior uncertainty.

Standalone compile succeeded. First execution lacked transitive metis runtime
library path and exited 127 before tests; rerun with preserved LD_LIBRARY_PATH
prefixed by /home/sasaki/.local/lib exited 0 and passed all 3 tests.
Full CTest not run. No app/solver behaviour changed in this phase, no raw run,
no truth/MAT/saved positioning input, no accuracy or robustness claim.

Next obtain residuals using the actual same-run initial graph pose/clock state,
not just SPP metadata, then compare nominal and IRLS information on identical
retained rows. Phase516 remains the verified nominal reference. Goal active.
