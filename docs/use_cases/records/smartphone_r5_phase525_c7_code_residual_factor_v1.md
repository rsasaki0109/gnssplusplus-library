# Phase525 — native C7-compatible code residual factor

Implemented code_ionosphere::ResidualStateFactor in
include/libgnss++/algorithms/code_ionosphere_state_factor.hpp. It wraps one
existing scalar code factor, shares its exact noise model, geometry/clock
residual and Jacobians, and appends +alpha*vertical_L1_residual and Jacobian
+alpha. This sign is code delay, unlike the negative carrier sign. No original
and wrapped factors may coexist as independent observations in a later graph.
Rejects null/non-scalar factors, nonpositive/nonfinite coefficients, key
collision and nonfinite/overflowing state residuals. Clone/equality supported.

Tests use the ACTUAL PseudorangeFactorSourceClockArm with nonzero lever arm,
nontrivial rotation and seven-component clock, and actual scalar Huber noise.
Zero state preserves residual, original Jacobians, noise pointer and robust
cost. Positive delay sign and numerical state derivative checked; invalid
coefficient/key/state controls and clone equality checked. 3/3 passed in a
fresh standalone GTSAM build. Initial test compilation failed from mixed
float/double NaN/Inf initializer types; explicitly converted and rebuilt.
Registered test under GTSAM_FOUND. Full CTest not run.

No selector, priors, graph insertion or native replay yet. Existing model and
guards remain unchanged. This is a factor building block, not a complete
ionosphere model: a physical shared state also needs opposite-sign carrier
coupling consistent with the already-corrected TDCP observable, and a defined
temporal prior/reset contract. A code-only experiment must be labelled as
such, not claimed as full ionosphere recovery. Next specify and test state
identity/prior/coupling, then integrate default-off without changing GNSS-first
or importing saved trajectories. No truth/MAT access or accuracy claim; goal
active and unmet.

Standalone build used -std=c++17 -O0 -DGNSSPP_HAS_GTSAM, include directories
include, /usr/include/eigen3, /home/sasaki/.local/include, and libraries gtsam,
gtest, gtest_main, pthread. Binary /tmp/phase525_code_ionosphere_factor ran with
LD_LIBRARY_PATH prefixed by /home/sasaki/.local/lib preserving the prior value.
