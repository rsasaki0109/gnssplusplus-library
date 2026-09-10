# Phase266 remaining first-state constraints

Read-only code audit; no raw/truth or saved positioning payload access.

Native IMU graph first-pose prior has roll/pitch sigma 0.05 rad, yaw sigma
5 degrees, and translation sigma 1e6 m in the smartphone CLI. The first
velocity prior has sigma 0.5 m/s and targets `imu.init_velocity_nav`.
The CLI sets that target to `velocity_heading.smoothed_velocity_enu.front()`;
it is therefore a smoothed native GNSS velocity, not automatically the
first unsmoothed velocity used elsewhere in the handoff. This establishes
a difference in target construction, not its magnitude or harm on H.

The Phase213 Doppler factors constrain ENU velocity and clock drift,
without a Pose3 key. The production CLI's lever arm is zero. GNSS position
factors and translation/velocity motion constraints therefore do not
directly observe yaw through an antenna lever arm. IMU acceleration/motion
can provide heading information, but stationary gravity does not determine
yaw. A whole-pose-prior removal cannot be justified by assuming all
attitudes are observable on every route. The old DD-oriented comment above
the prior block is not evidence for the no-base Phase171 observability.

Do not bundle removal of pose and velocity priors. Before any further raw
run, test a velocity-prior-only omission on a synthetic graph with the
existing pose/bias priors and dedicated Doppler factors retained. Require
exactly one removed factor and no replacement constraint or fallback.
Include a case where smoothed initial velocity differs from the velocity
state seed so the test does not merely exercise a zero residual.

Defer pose-prior removal until a separate nullspace/observability test
covers stationary and moving segments. No accuracy improvement is claimed;
the reference remains 1.0769392017393964 m on repeatedly used H development
data. The target remains unmet.
