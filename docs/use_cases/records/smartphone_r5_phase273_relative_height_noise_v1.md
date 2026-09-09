# Phase273: scalar/vector relative-height robust noise

Compared actual GTSAM Robust(Huber(0.5), Diagonal([Inf, Inf, 0.1]))
with Robust(Huber(0.5), Isotropic(1, 0.1)). Synthetic finite horizontal
residuals 12345 and -9876 accompany nine vertical residuals spanning zero,
both signs, quadratic region, transition, and linear region.

Squared Mahalanobis distance, loss, normalized-domain robust weight, and
robust-whitened vertical residual agree; horizontal whitened components
are zero. This verifies the tested local noise equivalence, not full
source graph or trajectory equivalence.

Initial build 66055 succeeded but the first test run failed because it
passed raw metre residuals directly to Robust::weight. Loss and whitened
residual comparisons passed. Corrected the test to pass unweightedWhiten
residuals to weight, without changing production code or tolerances.
Rebuild 80550 succeeded; all four selected tests passed (Phase273,
Phase272, and two selector tests). No full CTest claim.

Integration inspection: fgo_gtsam_backend.cpp currently computes the IMU
epoch_stop mask before graph insertion. Relative-height exclusion must use
this mask, not upstream_stop_velocity_gate_passed, because the source
relative-height branch checks !stop at both endpoints independently of
the separate stop-velocity prior gate. The existing stop_velocity_seeds_nav
is populated from same-run GNSS-first velocities by the CLI. Any new caller
must require its complete finite coverage rather than inherit the stop
factor's graph-velocity fallback. Candidate positions must likewise come
from the validated same-run handoff, before main optimization.

No raw run, accuracy evaluation, MAT access, saved positioning input,
submission, or production-default change. Graph integration is next.
