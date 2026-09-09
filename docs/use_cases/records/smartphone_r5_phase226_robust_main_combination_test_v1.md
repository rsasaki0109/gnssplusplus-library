# Phase226: robust main combined-graph test

Primary agent. Added a final default-false robust_main parameter to the
existing synthetic Phase171 handoff fixture and an additional combined
Phase184 + ECEF-D stage + main Doppler + Pose3 motion case with that parameter
true. The existing OFF case and all earlier calls remain unchanged.

Build session 35591 completed with exit 0. The focused Phase171, TDCP mapping,
Pose3 motion and Doppler-frame selection passed 10/10 C++ tests on the freshly
built test binary. The added subcase reaches the existing assertions for
convergence, selected k=0.2, factor counts and finite exported clock states.
git diff --check passed. Production solver code did not change in this phase.

Scope: robust loss is enabled in the main graph for the added case. The
synthetic stage retains its existing non-robust configuration. This is not
a numerical outlier-weight oracle or a claim that both synthetic stages now
match every production setting. The actual production threshold consumer
was inspected in Phase224; no real-data accuracy inference follows from
these tests. Full CTest was not run.

Next freeze a single raw H source-k experiment from Phase222 plus Phase184.
Its initial GNSS solve and resulting seeds may change; do not use the old
unchanged-GNSS-first gate. Preserve all other settings and evaluate only after
structural verification. No MAT, raw-data solve, truth or candidate payload
read occurred during this test phase. Current H best remains 1.2680574850266653 m.
