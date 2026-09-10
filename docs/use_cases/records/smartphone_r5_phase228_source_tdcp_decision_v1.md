# Phase228 source TDCP development decision

Evaluation frozen at 77e15d8 before truth access; 13 tests passed. Primary
agent, one candidate read, one truth read, one score, no native rerun.

H scalar worsened from 1.2680574850266653 to 1.3270306231389604 m,
delta +0.0589731381122951 m. P50 improved from 0.7706427231288419 to
0.6914426977553225 m, while P95 worsened from 1.7654722469244886 to
1.9626185485225982 m. All 3139 rows match exactly and are finite/Earth-valid;
zero over-70-m/s transitions, no interpolation, hold or offset reapplication.

Do not promote Phase227 by selecting the favorable median: the frozen scalar
is worse. Keep Phase222/223 as H development best and production defaults
unchanged. Source k=0.2 affected both stages; this result cannot isolate main
weighting from initialization. It does not invalidate the source algorithm
as a whole. No MAT or saved positioning input was used in the native solve.

Next audit source observation sigma and resL corrections before further
Huber tuning: the current fixed 0.03 m sigma is not proven equivalent to the
source observation-specific sigma. Avoid a truth-guided k sweep. Broader
route validation and split/leakage checks remain necessary; repeated H
development is not heldout or leaderboard proof. The goal is not achieved.
