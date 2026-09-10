# Phase274: relative-height integration (in progress)

Default-off config and CLI `--native-relative-height-pairs` insert scalar
local-up factors in the main graph only. GNSS-first config explicitly
disables the option. Selection reads epoch positions and full finite
stop_velocity_seeds_nav, with raw IMU epoch_stop flags, before optimization.
It does not use the generic position-seed fallback or stop-velocity gate.
Public admission requires GTSAM/Pose3/IMU/Phase171, stop detection and the
GNSS-first clock handoff; backend validates complete stop coverage.

CLI requires Pixel5 Phase171 ECEF-D, UTC fallback, stop constraints and no
Phase135. Summary reports requested state, selected/inserted counts and
fixed convention/threshold/noise parameters. No external height reference.

Build 24454 failed on an incorrect poseKey identifier; corrected to the
existing positionKey used for Pose3. Build 44294 then completed successfully.
Seven selected C++ tests passed: Phase274 guard, Phase273 noise, Phase272
Jacobian, two pair-selector tests, and two Phase171 main integration tests.
The extended synthetic main fixture uses deliberately fabricated 101 m/s
pair-selection velocities, separate from its same-run stage velocities,
to cross the threshold in a tiny fixture. This proves positive insertion,
matching pair/factor counts and convergence with the selector on/off; it
does NOT prove production seed provenance or physical validity of a pair.

CLI build session 55872 completed with exit 0. Six new CLI admission tests
and eight existing metre-sigma CLI tests passed (14 total).
No full CTest, real raw run, truth read, MAT
payload access, saved coordinate input or accuracy claim. Before freezing
a raw experiment: audit exact same-run epoch/velocity
handoff ownership, then commit the implementation and pin the binary.
