# Phase219: native main motion development result

Primary agent only. Phase218 completed once, return code 0, in
1366.1967291979818 seconds. No native rerun was performed for evaluation.
Evaluation code and manifest were frozen in commit 052a186 before truth access;
13 metadata/authorization and inherited synthetic metric tests passed.

The H development scalar improved from 1.2751561666667786 m to
1.2689788175187473 m (delta -0.006177349148031253 m). P50 is
0.7606401104947066 m, worse than the previous 0.7553341850450441 m;
P95 is 1.7773175245427881 m, better than 1.7949781482885132 m.
All 3139 rows match exact UTC keys, are finite and Earth-valid, and have
zero over-70-m/s transitions. No interpolation, hold, or offset reapplication.
Candidate and truth payloads were each read once for one score calculation.

Decision: Phase218 is the new best H development recipe by the frozen scalar,
not a demonstrated general improvement. Keep production defaults unchanged.
The raw-only native solver used no MAT or precomputed positioning inputs;
the saved candidate was used only by evaluation. H has been repeatedly used
for development and is not heldout or leaderboard proof. The 0.782 m target
is not achieved.

Next: examine the combined source main Doppler plus motion graph as an explicit
new ablation, including compatibility tests before enabling the combination.
The isolated Doppler ablation worsened the scalar slightly; its interaction
with motion is untested. Do not reuse saved trajectories as solver inputs.
Broader route validation remains required before generalization or leaderboard
claims. One observed runtime is not a speed benchmark.
