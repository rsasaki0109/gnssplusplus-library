# Phase533 — default-off native joint main selector wiring

Implemented use_native_joint_ionosphere with three explicit positive config
parameters (anchor sigma, random-walk density, max gap); no usable defaults.
CLI: --native-joint-ionosphere ANCHOR_SIGMA_M DENSITY_M_SQRT_S MAX_GAP_S.
Duplicate selector and malformed/nonfinite/nonpositive values rejected.
CLI restricts raw Phase171 ECEF-D all-epoch scope and excludes simultaneous
NHC, readmission, code floors and prior TDCP frequency-state experiments.
GNSS-first copied config explicitly clears the new enable bit.

Library optimizeProblem guard requires GTSAM batch Pose3/valid IMU/Phase171
same-run handoff, positive parameters, no legacy residual-ionosphere state,
no frequency residual state or Phase135 geometry. Existing legacy guards
remain intact. Backend collects exact inserted code and TDCP graph indices,
requires full nonempty coverage, stages the Phase532 plan, replaces original
factor slots and appends only priors/zero states before graph diagnostics and
solve. Prints state/prior/code/TDCP counts and all parameter values.

State key j matches the legacy ionosphere family; mutual exclusion and the
plan's graph/Values collision checks prevent coexistence. Gap-only process
segmentation is intentional: receiver clock jumps alone do not imply physical
ionosphere jumps. Accepted TDCP observations remain present across any process
segment boundary. No independently calibrated physical prior is claimed.

Build started session 44986, confirmed live compiling dependencies; completion
is NOT yet proven. Targeted diff check passed. Disk remaining ~1.3 GB: no large
downloads/debug builds or unrelated deletions. Next finish build, test CLI and
library scope rejection, add optimized-state sanity reporting and freeze one
candidate parameter set BEFORE any accuracy evaluation. New selector has not
been run on raw data, and disabled-path invariance after this wiring remains
unverified. No truth/MAT/saved positioning input. Goal active and unmet.
