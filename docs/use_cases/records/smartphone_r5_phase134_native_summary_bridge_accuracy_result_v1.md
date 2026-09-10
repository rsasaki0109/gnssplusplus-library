# Phase134 truth-only accuracy result

- Status: `no-go-phase134-truth-only-accuracy`
- Candidate: `phase134-native-phase131-summary-diagnostics-bridge-v1`
- Evaluation: exactly once per route, MTV-A then LAX-T, after independent authorization.
- Candidate and truth coordinate rows are omitted; release and Kaggle submission remain unauthorized.

| Route | Score (m) | P50 (m) | P95 (m) | Exact coverage | Finite/Earth-valid | >70 m/s | Route gate |
|---|---:|---:|---:|---:|---|---:|---|
| MTV-A | 1.0130009613 | 0.8492265624 | 1.1767753602 | 1.0 | True | 0 | NO-GO: regresses Phase112 |
| LAX-T | 0.6188718940 | 0.4653652055 | 0.7723785825 | 1.0 | True | 0 | GO |

Macro: `0.8159364277 m` (unweighted mean in the fixed MTV-A→LAX-T order).

Promotion gate: strict `candidate_macro_score_m < 0.782`; result `False`.
Failed gates: MTV-A no-regression versus Phase112 and strict macro threshold.

Baselines: Phase112 `0.8318381724 m`; Phase118 `0.8141981503 m`; Phase120
`0.8170323381 m`.

Read accounting: candidate reads 2, truth reads 2, accuracy calculations 2;
native solver 0, raw/base 0, MAT/PDC/precomputed coordinates 0, Kaggle/token 0,
reruns/fallbacks 0. Candidate and truth payload hashes were checked only after
authorization; no coordinate rows were written to this artifact.
