# Phase212: prioritize missing final-graph observations

Read-only algorithm audit by the primary agent. No raw run, truth read,
coordinate interpretation, MAT input, solver modification or new accuracy
claim. Worktree before this record was clean at `1127a17`.

## Evidence and priority

Cached `output/reproducibility-cache/gsdc2023/fgo_gnss_imu.m` inserts
`DopplerFactor_VD` in the final GNSS+IMU graph (lines 212–216), using velocity,
clock drift, LOS and source residual. It also inserts `MotionFactor_XXVV`
for consecutive epochs below the time-gap threshold (around lines 267–270).
`parameters.m` sets Street motion sigma to 0.05 m (lines 135–141).

The best native Phase198/199 recipe has **66685 Doppler factors in GNSS-first
and zero in the final graph**. Phase171's backend guard explicitly rejects
enabled generic Doppler or nonempty Doppler rows
(`src/algorithms/fgo_gtsam_backend.cpp`, around lines 102–126). The final
same-run optimized velocity/drift values are initialization, not persistent
Doppler likelihood terms. Thus successful ECEF-D initialization does not
establish source-like final-graph observation coverage.

The backend's dedicated raw-seed branch inserts XXVV motion factors around
lines 2363–2379. The current CLI main configuration disables position motion
factors (`apps/native/gnss_fgo_imu_no_base.cpp`, around line 11335). IMU
preintegration provides a different motion constraint; it is not evidence
that the source XXVV likelihood is present.

The backend already has an ENU Doppler insertion path and
`UndifferencedDopplerVelocityFactorSourceClock`, with vector-valued clock drift
support (around lines 1926–2010). The source-like Phase135 affine family is a
separate, currently incompatible option. Reuse mathematically verified
components only; do not enable an incompatible historical flag wholesale.

## Next bounded implementation

Prioritize a dedicated **opt-in final-graph Doppler lane**, before further
bias-noise or prior sweeps on H. Keep Phase171 no-D default semantics and
tests intact; do not merely remove its rejection guards. The new lane must:

1. Carry corrected Doppler rows from raw same-run construction with exact
   epoch/source identity, never from saved velocity/position outputs.
2. Transform LOS ECEF to the existing IMU ENU frame once, verify residual sign
   and m/s clock drift units, and retain source quality/robust-noise contracts.
3. Add likelihood terms to the final graph, with actual row/factor counters,
   while preserving the GNSS-first initialization and the best legacy IMU
   option. Do not combine the rejected Phase201/205/209 changes.
4. Test residual/Jacobian signs, frame rotation and malformed joins on
   synthetic controls, then test actual graph construction and convergence.
5. Freeze a single raw comparison before truth. Add XXVV separately after
   this isolated observation-family test, rather than confounding both.

This is a code-supported hypothesis about missing constraints, not proof
that adding Doppler improves H or reaches 0.782 m.

## Other differences and prohibited shortcuts

Source initial pose/position/velocity/clock/drift/bias priors use infinite
sigmas (`fgo_gnss_imu.m` lines 181–186). Native first-state priors include
roll/pitch 0.05 rad, yaw 5 degrees, velocity 0.5 m/s, accel bias 0.1 and gyro
bias 0.01 (`gnss_fgo_imu_no_base.cpp` lines 5625–5629). Removing them requires
an observability audit; it is not the next numerical tuning step.

Source height logic has both a `posgt`-dependent branch and a same-run
trajectory loop-height branch (around lines 226–249). **Do not port the
ground-truth-dependent branch into inference.** Any future loop-height
constraint must derive candidates solely from raw same-run estimates and
must not consume a saved trajectory. Neither branch is authorized by this
audit. H remains reused development/train, not heldout or leaderboard proof.
