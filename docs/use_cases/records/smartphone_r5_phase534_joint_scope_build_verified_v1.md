# Phase534 — joint selector build and rejection controls

Phase533 native build session 44986 completed exit 0. Added registered
test_joint_ionosphere_scope.cpp and linked it against the freshly built native
libraries. /tmp/phase534_joint_scope passed 2/2 tests: default off with zero
unset parameters, plus twelve invalid configurations rejected with the
dedicated joint guard message (not an unrelated later empty-problem error).
Includes wrong backend, missing IMU/Pose3/valid input, fixed lag, missing
Phase171/handoff, legacy/frequency-state conflict and bad explicit parameters.

Actual CLI rejected nine malformed argument controls: missing one/all values,
zero/negative/nonfinite values, trailing nonnumeric text and duplicate flag.
Help displays all three parameter units. CLI controls do not independently
prove complete valid-path activation because no valid raw job was launched.
The library guard controls establish scoped rejection before graph building.

No raw run or accuracy evaluation. Full CTest not run. Optimized-state sanity
reporting and disabled-path invariance after new wiring remain outstanding.
No physical prior parameters selected/frozen yet. Next add finite/range
diagnostics for the actual optimized shared states before a raw candidate,
then freeze and run one explicit main-only experiment. No MAT, truth or saved
positioning input; goal remains active and unmet. No active build/test jobs.
