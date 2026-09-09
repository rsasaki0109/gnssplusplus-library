# Phase263 first IMU bias prior ablation

Default-off `--native-omit-first-imu-bias-prior` selects omission of the
single first-bias prior only. The native CLI requires Pixel5 Phase171
ECEF-D and UTC fallback without Phase135. GNSS-first copied config resets
the selector to false. Public optimizer rejects non-GTSAM/non-Pose3/
non-IMU/non-Phase171 selection.

The backend retains initial bias Values and preintegration bias, first
pose and velocity priors, and all CombinedImuFactors. It records inserted
and omitted first-bias prior counts. The counters are exposed in the
summary tdcp_contract block, without changing TDCP mathematics.

Added synthetic integration case compares the selected graph with the
same problem/config except selector off, checking convergence and exactly
one fewer factor plus unchanged IMU and TDCP counts. This does not by
itself prove full rank for arbitrary motion or exact preservation of all
unobserved states. Existing solver failure behavior remains unchanged;
no hidden stabilizing factor is inserted by this option.

Build session 25980 completed both targets successfully. Twenty focused C++
tests passed (Phase263/259/171 and FusionInitializationTest), including
convergence and exactly one fewer graph factor in the selected synthetic
case. CLI tests for bias-prior omission, heading and metre sigma passed
18/18. No full CTest or arbitrary-motion observability claim.
Raw-data accuracy remains pending. No raw or truth
payload read, MAT access, saved-position input, or submission in this work.
