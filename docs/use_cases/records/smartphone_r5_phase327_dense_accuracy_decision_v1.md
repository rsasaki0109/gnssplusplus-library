# Phase326/327: dense smoothing corrects support but not H regression

Frozen Phase326 raw GNSS/IMU run completed once in 242.250 seconds, exit 0.
GNSS-first and main converged and the in-memory handoff aligned. Base model
built once; corrections applied once. Before correction, P count was 101847,
unchanged from Phase319. Dense smoothing retained 81591 rather than 81394
factors: 197 additional finite-correction rows. Missing streams remained 19163;
unavailable callbacks fell from 1290 to 1093. Count conservation passed.
The callback counter is not an independent proof of out-of-domain timing:
it also includes NaN barriers inside the dense grid.

Candidate frozen at bc1d6541: 3139 exact UTC rows, 251174 bytes, SHA-256
`e3bf7cf230288c926804d894ddb940573198123272b1a7dc87ea1e3cfd3d2114`.
Evaluator frozen at 438a90bd after static checks and ten kernel tests. Each
candidate/truth payload read once, one score, no interpolation or rescore.

- P50: 2.0253460613959198 m.
- P95: 2.478079488116469 m.
- Score: 2.2517127747561947 m.
- Earlier compact paired score: 2.2515351668966965 m.
- Operational base-OFF score: 1.0769392017393964 m.

Do not promote. The grid-shape fix changed correction support but did not
resolve the large paired regression on H. The tiny score difference is not
evidence of a useful performance change. Dense smoothing remains a tested
explicit algorithm mode; the operational base-OFF recipe remains unchanged.
This does not establish that all source smoothing/interpolation details match.

Next separate the effect of finite-correction row selection from subtraction
of correction values, keeping the candidate's state/bias settings fixed in a
preregistered diagnostic ablation. Also continue source-equation/reference
checks without truth-derived coordinate or correction adjustments. Do not
tune moving-mean width based on these scores or repeat existing evaluations.
H remains development data, not heldout/LB proof. No MAT, saved-positioning
inference input, or submission; 0.782 and leaderboard goals remain unachieved.
