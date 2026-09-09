# Phase120 truth-only accuracy result

Status: NO-GO; the strict promotion gate failed closed.  Coordinates and
coordinate rows are omitted, and release, validation, and Kaggle submission
remain unauthorized.

Evaluation was one isolated truth-only process, in the fixed order MTV-A then
LAX-T, after the independent authorization commit.  No native solver, raw
GNSS/IMU/navigation, raw-base, MAT, PDC, precomputed-coordinate, or Kaggle
lane was used.

| Route | Score (m) | P50 (m) | P95 (m) | Prediction rows | Truth rows | Domain coverage | Finite | >70 m/s | Route gate |
|---|---:|---:|---:|---:|---:|---:|---|---:|---|
| MTV-A | 1.0080567510569665 | 0.8369792217794683 | 1.1791342803344647 | 2158 | 2159 | 1.0 | true | 0 | NO-GO (regressed vs Phase112) |
| LAX-T | 0.6260079250519545 | 0.4658974769535745 | 0.7861183731503345 | 1465 | 1465 | 1.0 | true | 0 | GO |

The unweighted two-route macro is `0.8170323380544604 m`; the strict gate is
`candidate_macro_score_m < 0.782`, so it failed.  The sealed comparison
macros are Phase112 `0.8318381724000121 m` and Phase118
`0.814198150322117 m`.

Read accounting: candidate payload reads 2, truth reads 2 (one per route),
candidate coordinate interpretations 2, and accuracy calculations 2.  Native
solver invocations, raw GNSS/IMU/navigation reads, raw-base reads,
MAT/PDC/precomputed-coordinate reads, Kaggle/token access, reruns, and
fallbacks were all zero.  The JSON result contains only opaque hashes,
metadata, metrics, gate booleans, and accounting.
