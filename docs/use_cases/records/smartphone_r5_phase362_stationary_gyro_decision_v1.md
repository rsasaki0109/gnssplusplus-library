# Phase362 — stationary gyro initialization not promoted

Frozen evaluation commit: `a5e6d217`. Evaluator metadata verification and
10 synthetic metric tests passed before one candidate read, one truth read
and one score. No native rerun or submission.

H reused-development score: **1.0770097254022988 m**.
P50: 0.8368958723547287 m; P95: 1.3171235784498687 m.
Operational baseline: 1.0769392017393964 m.
Delta: +0.00007052366290238865 m (worse, approximately 0.071 mm).
All 3139 keys matched exactly; no interpolation, edge hold or offset
reapplication. Both native stages converged; GNSS-first aggregates matched
baseline, with unchanged graph factor and value counts.

Do not promote this option. The difference is practically negligible and
does not support initial gyro bias as the explanation of the target gap on
H. Raw evidence that the initial window is not classified stationary does
not imply that replacing its mean improves a jointly optimized bias graph.
Do not sweep block lengths, stop thresholds or bias blends against H truth.
Keep the default-off helper and synthetic tests for reproducibility.

Next work should move away from this initialization hypothesis, toward
measurement-model evidence or route-grouped development transfer. Any
additional evaluation split must be checked against prior truth-access
history before being called held out. No fresh validation or leaderboard
claim follows from this result; the 0.782-class / LB objective is unmet.
