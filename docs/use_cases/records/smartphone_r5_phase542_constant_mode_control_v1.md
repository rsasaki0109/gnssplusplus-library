# Phase542 — constant-mode synthetic control

Added tests/test_joint_ionosphere_constant_mode.cpp; standalone compile and
/tmp/phase542_constant_mode passed 2/2 tests. Not registered in CMake during
the frozen transfer run; no native source or binary changes.

Actual temporalPlan for 100 contiguous epochs yields one anchor and 99 walk
edges. A constant state incurs zero walk penalty regardless of walk density;
only the single anchor penalizes it. Equal endpoint mapping makes its TDCP
correction zero, while differing endpoint mapping constrains it. A separate
analytic Gaussian toy with fixed geometry/clocks shows repeated code bias
aligned with the ionosphere coefficient can overwhelm one anchor.

This is an identifiability counterexample, not a reconstruction of U. The
real graph has robust losses, evolving mapping and free geometry/clock states.
It does not prove the fitted U state is code bias or justify truth-tuned
anchor/density changes. Independent raw-observable evidence is needed to
separate atmospheric and other systematic errors before another model trial.
