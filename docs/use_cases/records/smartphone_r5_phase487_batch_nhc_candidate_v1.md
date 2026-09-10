# Phase487 — default-off native batch NHC candidate

Implemented --native-batch-nhc / FGOConfig.use_native_batch_nhc, independent
of fixed-lag use_nhc. Scope requires GTSAM batch Pose3, valid IMU, Phase171
main and same-invocation GNSS-first handoff. CLI additionally requires Android
raw Phase171 ECEF-D. The copied GNSS-first config explicitly disables it.
No saved positions or MAT input is introduced.

Before scoring, freeze candidate policy: previous-interval gate from Phase486
(speed >=2 m/s, peak angular speed <=0.2 rad/s, maximum gap .05 s), existing
NonHolonomicFactor at the interval's terminal epoch, diagonal lateral/vertical
sigmas .3/.2 m/s with block Huber 1.345. Reuses body-to-ENU Pose3 and ENU
velocity keys. Vehicle lever arm zero is an approximation, not measured
calibration. Fixed source mounting remains an assumption to validate through
the complete raw experiment; not inferred from truth. Insertions increment
diagnostics.nhc_epochs and aggregate stderr factors_added.

Extended scripts/native_nhc_frame_audit.cpp with an actual robust-factor
joint pose/velocity LM solve. Synthetic prior yaw .05 rad becomes .00110026
rad with velocity change 2.44499e-7 m/s; cost decreases and assertions pass,
exit 0. This tests joint optimization and frame handling, not route accuracy.
Earlier frame covariance and turn-reversal counterexample checks also pass.

Main build in progress when recording this implementation. No native candidate
route or truth evaluation has run yet. Prior Phase486 monitor-only output
invariance does not establish this changed factor graph's accuracy. Default
remains off and overall .782 / leaderboard goal is unmet.
