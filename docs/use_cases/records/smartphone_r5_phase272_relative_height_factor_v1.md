# Phase272: relative local-up factor

Implemented internal `RelativeHeightPoseFactor`, with residual
`up_ecef dot (antenna_ecef_2 - antenna_ecef_1)`. The fixed unit up axis
defines one common tangent frame, not a separate geodetic height at each
endpoint. Both Pose3 Jacobians use the existing LeverArm chain rule.
Constructor checks finite unit axis and scalar noise dimension.

Build session 11575 completed with exit 0. Sixteen selected tests passed:
the new Phase272 test, Phase252 affine Pose3 test, two relative-height
selector tests, and twelve FusionInitializationTest tests. This is not
full CTest coverage.

The new test uses a rotated navigation-to-ECEF frame, nonzero lever arm,
different endpoint attitudes, and central differences for all twelve pose
coordinates (step 1e-3, tolerance 2e-6). It verifies local antenna-height
residual and invariance under horizontal translation of one endpoint.
Invalid zero up axis and non-scalar noise are rejected.

No graph integration or production-default change yet. Source vector-noise
versus scalar robust-loss equivalence still needs a direct test, followed
by same-run provenance and insertion coverage tests. No raw run, truth or
MAT payload access, saved positioning input, or accuracy claim.
