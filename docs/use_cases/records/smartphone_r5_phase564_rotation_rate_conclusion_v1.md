# Phase564: rotation-rate correction is flat on H development

Both raw-only runs completed successfully, source/binary/raw pins unchanged.
Candidate session 10547 terminal, exit 0, 192.10262325406075 seconds;
baseline 194.5893999011023 seconds. Candidate output SHA:
`2d19cdcb5f63dd155dcc52b02eee40b30ad1e80f57929f4f99501bf5d3fd2f87`.
Structural verifier passed both stages. Same 3,140 modeled epochs,
101,916 P factors, 69,270 TDCP rows and 66,685 D factors in each graph;
no epoch-counter changes. Enabled summary present, finite converged graphs,
same-run handoff valid. Full CTest not run.

Separate evaluation manifest frozen after structural verification, before
truth read; single evaluation attempt completed using unchanged legacy
metric kernel. Artifacts are Phase564 accuracy manifest/result/attempt JSON
beside this record. Evaluation-only reads: candidate once, baseline once,
truth twice. No output coordinates fed to inference.

H development metric (P50+P95)/2 in metres:

- Baseline: 1.0769180514591334
- Rotation-rate candidate: 1.0769450401046536
- Delta: +0.000026988645520198418 m (about 0.027 mm worse)
- Candidate P50: 0.836929693962997; P95: 1.3169603862463102
- Matched rows: 3,139; prediction/truth domain coverage both 1.

Conclusion: numerically flat, no positioning improvement demonstrated.
Do not promote the flag, tune its magnitude, or claim progress toward .782
from this result. Keep default off. This model is only a partial physical
derivative and should not be described as a fully validated observable fix.
No U/LAX sweep justified solely by this H result, and no submission made.
Next prioritize a materially larger raw-observation/model limitation rather
than sub-millimetre correction variants. This is development evidence only,
not held-out or leaderboard proof. The full goal remains unmet.
