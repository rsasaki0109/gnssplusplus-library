# Phase490 — NHC does reach position, but response is small on H

Compared frozen Phase479 floor-only and Phase488 floor+NHC output CSVs
evaluation-only, by exact 3139 keys and pinned SHA256. No truth read or
accuracy recomputation. scripts/compare_phase490_nhc_outputs.py exit 0;
aggregate JSON is smartphone_r5_phase490_nhc_output_response_v1.json.

Position separations: mean 0.00843192148043339 m, P50
0.005408557596357283 m, P95 0.025001738666844447 m, maximum
0.031033846089703197 m. Thus NHC is not disconnected from position; this
run changes positions at millimetre-to-centimetre scale. Output difference
alone cannot say whether attitude absorbed most of the constraint or whether
the initial trajectory already satisfied it.

Code confirms Phase217 inserts MotionFactorPose3XXVV on consecutive pose and
velocity states (sigma .05 m), in addition to IMU. NHC itself uses only pose
rotation and velocity, with zero direct translation Jacobian. A uniform
translation of the trajectory leaves NHC unchanged; it cannot independently
anchor an absolute positioning offset. Do not conclude this proves the H
error is a uniform offset, or infer a truth-fitted correction.

With exactly matched keys, triangle inequality bounds each distance-to-truth
change by the maximum 0.031034 m output separation, and likewise bounds each
error percentile and their mean. This fixed NHC candidate cannot bridge the
roughly .295 m historical H score gap to .782, regardless of error direction.
No gate/noise sweep is justified from this result. Keep candidate default off.
Next prioritize raw absolute GNSS modelling/admission evidence over stronger
vehicle pseudo-measurements; need new evidence rather than another H score.
