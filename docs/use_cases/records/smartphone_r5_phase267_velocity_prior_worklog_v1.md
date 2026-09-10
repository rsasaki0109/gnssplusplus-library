# Phase267 first velocity prior ablation

Added default-off `--native-omit-first-imu-velocity-prior`. CLI requires
Pixel5 Phase171 ECEF-D, Phase213 main Doppler, UTC fallback and no Phase135.
The public optimizer also requires GTSAM Pose3 IMU Phase171 with Phase213.
The GNSS-first config explicitly disables the main-only selector.

Only the first-velocity prior insertion is conditional. Initial Values,
IMU preintegration, pose and bias priors remain unchanged. Inserted and
omitted velocity-prior counters are exposed in the existing summary.

The synthetic integration case offsets imu.init_velocity_nav by
(0.4, -0.2, 0.1) m/s in both candidate and reference problems so the prior
target differs from the ordinary same-run velocity seed. It checks
convergence, exactly one fewer graph factor, retained bias prior, and
unchanged IMU/Doppler factor counts. This is not a general observability
proof for all possible Doppler geometry or motion.

CLI velocity/bias/metre-sigma tests passed 19/19. Build session 58134
completed both targets successfully. Twenty-one focused C++ tests passed
across Phase267/263/259/171 and FusionInitializationTest, including the
offset-target graph comparison. No full CTest claim.
Real-data accuracy is unverified. No raw/truth/MAT
payload or saved-position input read for this implementation work.

Next freeze one H
run against Phase234 with only the velocity-prior selector added; require
GNSS-first and initialization aggregate equality, one fewer graph factor,
and unchanged exact output alignment before scoring. Keep all defaults.
