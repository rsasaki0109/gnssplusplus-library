# Phase259 per-epoch heading integration

Default-off CLI `--native-epoch-heading-attitude-seeds` requires Pixel5
Phase171 ECEF-D, UTC fallback, and no Phase135. The same-run GNSS velocity
sequence is converted by velocityToRpy with nearest interior filling, and
all rotations plus exact epoch times are carried in FGOProblem. The copied
GNSS-first config explicitly disables this main-only selector.

The public optimizer rejects unsupported backend/state configurations,
missing rotation/time entries, nonmatching times, nonfinite matrices,
nonorthogonal matrices, and reflections. The IMU backend replaces only its
attitude initialization sequence and counts inserted seeds. Bias, noise,
factors, clocks, and velocities are not intentionally changed. With a
nonzero lever arm, body translation is still derived from the antenna
position and the selected rotation, as in the existing initialization.

CLI diagnostics expose requested/inserted fields in the existing summary's
tdcp_contract block (placement is historical convenience, not a TDCP factor
change). Failed IMU initialization cannot fall back because Phase171 is
required and already fails closed.

Validation: CLI heading and metre-sigma tests passed 13/13. Build session
75070 completed both native CLI and C++ test targets successfully.
Twenty-three focused C++ tests passed across Phase259, Phase171,
FusionInitializationTest, and Phase256/253/252/251. No full CTest claim.
Added rejection tests and a synthetic two-epoch graph insertion case with
varying yaw. This graph fixture does not itself validate velocity-to-heading
provenance; the helper has separate tests and CLI code must also be checked
on a frozen raw run. No accuracy claim yet.

Remaining: check exact inserted count and
unchanged initial GNSS stage in one frozen H comparison. Preserve Phase234
reference settings otherwise. No truth/MAT/saved-position input was read
for this implementation work.
