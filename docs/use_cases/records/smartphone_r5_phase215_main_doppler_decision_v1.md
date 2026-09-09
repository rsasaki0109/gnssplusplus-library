# Phase215: main Doppler alone does not improve aggregate H accuracy

Primary agent; no subagents. Evaluator freeze commit `e9af3d0` precedes one
authorized score. Metadata/authorization plus unchanged synthetic metric
tests passed 13/13. No native rerun for evaluation.

H development score: **1.276650597650404 m**, against Phase199 best
**1.2751561666667786 m**; delta **+0.0014944309836253389 m** (worse).
P50: 0.7693956402821466 m (baseline 0.7553341850450441 m).
P95: 1.783905555018661 m (baseline 1.7949781482885132 m).
P95 improves but P50 degrades; do not promote using only the favorable metric.
All 3139 candidate/truth keys matched, finite/Earth-range checks passed,
over-70-m/s segments were zero, and no interpolation/hold/offset reapplication
occurred. Candidate/truth reads and score calculation were each one.

Keep Phase198/199 as accuracy best and Phase213 default-off. The observed
154.548 s runtime is shorter than the previous single raw comparisons, but
is not a repeatability benchmark. Do not claim a general speedup or combine
this runtime advantage with an unsupported accuracy improvement.

The missing-final-D hypothesis was tested: it changes the graph but alone
does not close the roughly 0.493 m target gap. Next isolate the source XXVV
position/velocity motion likelihood identified in Phase212. Port the actual
Pose3 translation residual and Jacobians, preserve the original priors and
IMU schedule, and test synthetic constant-velocity and inconsistent-velocity
controls before raw execution. Freeze whether main Doppler is enabled before
truth; do not choose a combination after inspecting the new candidate score.
No XXVV implementation or raw invocation is claimed by this record.

H remains reused development/train, not heldout or leaderboard proof.
0.782-class native raw-only performance and the leaderboard objective remain
unachieved; broader route-grouped validation is still required.
