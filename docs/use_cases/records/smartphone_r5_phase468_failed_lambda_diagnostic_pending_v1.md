# Phase468 — failure-lambda diagnostic implementation pending build

Existing Phase98 logs report nearby_variable_unavailable. Inspection of
GTSAM tryLambda confirms its internal exception catch discards the exception
object, so the current wrapper cannot recover a nearby key from SUMMARY or
TRYLAMBDA text. No key is guessed and GTSAM is not patched/replaced.

Added aggregate stderr output in native optimizer telemetry finalization:
native-lm-failed-lambda count/min/max, with nearby_key explicitly unavailable.
Only failed parsed linear trials with finite nonnegative lambda contribute.
No factors, optimizer parameters, trial decisions or state values change.
The aggregate does not establish the failure cause and does not assign stages
by itself; a prospective replay must distinguish GNSS-first/main log order
using the surrounding stage records and validate counts against summaries.

Build started with cmake --build build --target gnss_fgo_imu_no_base -j1,
session 78858. Completion is not yet established at record creation. No
raw replay was launched; no new source/binary pins were frozen. Next poll
the same build handle, then verify diagnostic behavior before a single frozen
baseline replay. Do not reuse historical manifests after source modifications.

git diff --check passed. Disk free approximately 6.2 GB; avoid duplicate
large debug builds. No truth/MAT/solution input/submission. Goal unmet.
