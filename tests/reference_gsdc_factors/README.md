# Pinned GSDC factor comparison

This optional standalone suite includes the original taroz/gtsam_gnss headers
from commit `e679b72b620fc6800723578e4077dd157587d129` and compares their actual
evaluators with the native backend. Configuration rejects another commit or
modified reference headers. No reference implementation is copied into the repo.

Configure with installed GTSAM and GTest packages and the pinned checkout:

```sh
cmake -S tests/reference_gsdc_factors -B build-reference \
  -DTAROZ_GTSAM_GNSS=/path/to/gtsam_gnss \
  -DGTSAM_DIR=/path/to/gtsam/cmake -DGTest_DIR=/path/to/gtest/cmake
cmake --build build-reference --config Release
ctest --test-dir build-reference -C Release --output-on-failure
```

Ensure the corresponding shared-library directories are on the runtime search
path (PATH on Windows). The suite checks unwhitened residuals and analytic
Jacobians for 15 clock states, 15 motion states, 12 Doppler states, 21 optional affine pseudorange states, and three optional
affine TDCP states. A sixth test checks that the default nonlinear pseudorange
model agrees at its anchor but differs away from that anchor; its success is
not an equivalence claim. The affine classes are optional and are not selected
by the submitted default recipe. Doppler
measurements are explicitly converted from source residual-at-origin to native
absolute projected measurement before comparison. Both velocity and LOS use
the same frame.

These tests do not run MATLAB, full trajectory estimation, observation
preprocessing, factor-selection policies, whitening, or optimizer convergence.
They do not cover all P/TDCP variants, IMU, or height factors and make no accuracy
claim. They are separate from the ordinary repository suite because the pinned
external checkout is an explicit dependency.
