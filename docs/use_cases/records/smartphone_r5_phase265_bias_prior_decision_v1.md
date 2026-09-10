# Phase265 first-bias prior decision

Phase264 completed once in 315.41625128197484 seconds. The first-bias
prior was omitted exactly once; graph count was 252,383 versus reference
252,384. GNSS-first, IMU initialization, epoch and output aggregate
contracts matched Phase234. No MAT or saved positioning solver input.

Twenty-three metadata/evaluator tests passed before evaluation freeze
13902d0. Candidate and truth were read once each and scored once. All 3,139
keys matched, both domains had full coverage, finite Earth-position gates
passed and no speed exceeded 70 m/s. No interpolation, hold or offset
reapplication was used.

- P50: 0.8369083196573837 m.
- P95: 1.3169700838214091 m.
- Score: 1.0769392017393964 m; delta versus reference: 0.0 m.

These metric values equal the reference, but candidate hashes differ;
do not infer identical trajectories or all residuals. The native main
final cost changed from 319301.9783692179 to 319301.86922738806. No accuracy
improvement established. Keep the prior enabled by default.

The single H development ablation does not establish that bias priors are
irrelevant on other routes or that removing all initial priors is safe.
The target remains unmet and no leaderboard submission was made.

Next inspect active first-pose and first-velocity constraints, and their
gauge/observability roles, before deciding whether another prior ablation
is justified. Do not infer that the other priors share this outcome or
sweep their sigmas against H truth. Recent heading/bias tests have not
explained the roughly 0.295 m gap; prioritize a demonstrable model or
observation-admission difference over unsupported initialization tuning.
