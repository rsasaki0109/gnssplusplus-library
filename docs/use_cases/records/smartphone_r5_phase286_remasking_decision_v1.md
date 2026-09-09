# Phase286: residual-only re-admission decision

Frozen evaluator commit d1724f4; 31 selected metadata/scoring tests passed.
One candidate read, one truth read, one score. All 3139 exact keys and
integrity gates passed; no interpolation or hold. The 0.782 m gate failed.

Score 1.0781431256597123 m versus operational 1.0769392017393964 m:
delta +0.0012039239203158747 m. P50 0.846386404936933 m;
P95 1.3098998463824914 m. P95 improved but P50 worsened enough that the
predeclared aggregate did not improve. Do not select a different metric
after the result. Retain Phase234 operational settings; re-masking OFF.

Recovery of 687 observations and removal of 553 demonstrates an operative
admission change, not better positioning. The small scalar regression does
not prove equivalence, significance, or explain the remaining accuracy gap.
Do not sweep thresholds or chain more same-route re-masking passes from
this result. Review broader pipeline evidence before another candidate.

H is repeatedly used development data, not heldout or leaderboard evidence.
No native rerun, MAT payload, precomputed solver input, Kaggle submission or
token access during evaluation. The overall objective remains unachieved.
