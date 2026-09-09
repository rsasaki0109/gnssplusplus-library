# Phase277: relative-height development decision

Evaluation frozen at 0378adb; 30 metadata/scoring tests passed. One candidate
read, one truth read, one score calculation. All 3,139 keys matched exactly
without interpolation or hold, and all integrity gates passed. The 0.782 m
gate failed. H is repeatedly used development data, not leaderboard proof.

- Score: 1.0824359657527727 m.
- P50: 0.8460043723150807 m; P95: 1.3188675591904646 m.
- Delta from operational Phase235 baseline: +0.005496764013376287 m.

Do not promote: the isolated 7,266-pair relative-height addition did not
improve H. Keep it default-off and retain the Phase234 operational recipe.
Do not sweep proximity, speed-sum, sigma, or Huber thresholds in response.
The result does not establish whether pair geometry, road levels, existing
vertical errors, or model interaction caused the regression. Inferring a
cause from this scalar alone would be unsupported.

Next return to raw observation/model evidence rather than another prior or
height parameter sweep. Native raw-input feasibility is demonstrated for
this option, not the requested accuracy or leaderboard rank. No MAT payload,
precomputed solver input, native rerun, submission or token access occurred
during the evaluation. No generalization or statistical significance claim.
