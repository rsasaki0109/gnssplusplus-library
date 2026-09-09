# Phase253 TDCP-only affine integration worklog

Status: implementation and validation in progress; no accuracy claim.

The default-off `--native-tdcp-only-affine-geometry` selector connects the
existing affine Point3 factor and Phase252 Pose3 factor to ordinary TDCP.
Each graph anchors at its own actual initial Values: native raw-stage
Point3 values, or main-stage antenna positions from Pose3 and LeverArm.
The main initialization remains the same-run in-memory native handoff.
No saved positioning or MAT input is introduced.

Only TDCP geometry is changed. Previous-endpoint LOS is used at both
endpoints; the carrier measurement is reduced by the endpoint anchor-range
difference. Satellite positions and plain-range convention are inherited
from the native endpoint factors. This is not a claim of source raw-geodist
or single-Sagnac parity. Source resL selection and metre sigma remain
independent selectors; historical Phase135/138 recipes are unchanged.

Main and GNSS-first inserted-factor counters are exposed in the summary.
Non-GTSAM selection throws rather than silently ignoring the option.

Validation so far:

- CLI admission tests plus existing metre-sigma CLI tests: 14 passed.
- Initial build failed because a plainRange call omitted its third argument;
  corrected to explicitly pass nullptr for an unused Jacobian.
- The integration fixture initially had no stage TDCP observations; corrected
  to add one synthetic stage observation and check actual insertion before
  testing the main graph. The corrected fixture passed the fresh build/run.
- Build session 66810 completed successfully. Subsequent incremental build
  session 62938 also completed, including the changed fgo.cpp and CLI.
- Twenty-one focused C++ tests passed: Phase253/252/251, Phase171, and
  historical Phase135 affine-family and Phase138 tests. The same-run handoff
  fixture exercised one inserted affine factor in each stage with main
  Doppler, motion, and robust loss enabled. This is not full CTest coverage.

- Latest CLI regressions (affine geometry, metre sigma, resL): 22 passed.

Next: freeze a raw-only H comparison against the
Phase234/235 recipe with only this geometry selector changed. Do not read
truth or launch an accuracy experiment until the implementation tests pass.
