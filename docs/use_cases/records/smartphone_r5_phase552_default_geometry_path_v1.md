# Phase552 — default Pose3/C7 geometry path

Traced backend code insertion to PseudorangeFactorSourceClockArm, supplied
factor.satellite_position_ecef (the already rotated position), and ordinary
TDCP insertion to TimeDifferencedCarrierFactorSourceClockArm with the rotated
previous/current endpoints. Both factor implementations evaluate plainRange
at the lever-arm antenna position, not the source geodist/Sagnac helper.
Preparation's earthRotationCorrected rotates once using geometric travel time
at the seed. The separate unrotated source fields are used by optional affine
paths, not the inspected default Pose3/C7 branches.

No double Earth-rotation application found in these branches. The rotation
angle is frozen at seed construction, not recomputed at every optimized pose;
that approximation is distinct from double correction and has not been
quantified here. No claim about all alternative branches or Doppler paths.
No native source edits, accuracy evaluation or new candidate launch.

Do not add a second Sagnac correction to these factors. A future exact
geometry change must compare against the current rotation approximation with
synthetic receiver perturbations before spending another raw full-run budget.
