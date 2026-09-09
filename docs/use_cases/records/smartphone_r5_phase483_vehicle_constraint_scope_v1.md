# Phase483 — batch vehicle-constraint candidate scope

Code audit: smartphone CLI fixes antenna lever arm to zero; no measured
alternative mounting distance is available in the inspected inputs. Do not
tune an arbitrary lever arm against route scores. Existing NonHolonomicFactor
is wired in fgo_gtsam_fixed_lag.cpp, not the smartphone batch backend. Its
residual is body lateral/vertical velocity, using Pose3 rotation and nav-frame
velocity. Current CLI uses explicit fixed taroz mounting Rz(-94)Ry(178)Rx(-85)
degrees; raw phone axes are not vehicle axes by definition.

Compiled scripts/native_nhc_frame_audit.cpp against the actual factor. At
20 m/s straight motion, yaw errors 0/1/5 degrees give lateral residuals
0/-0.349048/-1.74311 m/s, or 0/1.16349/5.81038 nominal sigmas at the existing
0.3 m/s setting. Analytic residual/sign checks passed, exit 0. No real data,
truth, MAT or saved-position inference inputs were used.

This quantifies frame sensitivity, not proof that actual mounting is wrong
or that a jointly optimized NHC factor will fail: orientation is a state,
and the full IMU/GNSS graph may constrain it. Before adding batch NHC, make
the vehicle/sensor alignment assumption explicit and test both orientation
correction and genuine side-slip/turn cases. Reuse the actual factor with
an opt-in scope, raw speed/turn gates and no GT-derived heading. A factor that
simply forces the phone axes to follow GNSS velocity without validated frame
handling is not an acceptable implementation.

NHC remains an untested batch accuracy hypothesis. Do not advertise an
improvement or enable the fixed-lag flag expecting it to affect batch FGO.
Numerical-floor experiment is complete only as a three-route feasibility
check; overall accuracy/leaderboard goal remains active and unmet.
