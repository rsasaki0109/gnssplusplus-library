# Phase134 isolated truth-only accuracy lane audit

Status: launch-free audit; no candidate payload, truth payload, raw input, native
solver, MAT, PDC, precomputed coordinate, or Kaggle access.

## Scope and sealed input

This lane evaluates only the two opaque solution seals produced by the
Phase134 structural result (`f658d52ce666882c1516d9ede8d396885d8adbd6`).  The
structural result is the sole candidate provenance.  Its per-route solution
SHA-256 and row count are metadata only; no solution path is opened or
interpreted during this audit.  The fixed route order is:

1. `2021-03-16-18-59-us-ca-mtv-a/pixel5` (MTV-A)
2. `2022-04-01-18-22-us-ca-lax-t/pixel5` (LAX-T)

The structural result SHA-256 is
`426adb4d951d226a4e99e36813e0873fd0b10ec47a6b62a10f2df4d67327f0fa`.
The candidate is one existing Phase134 run per route, with no repair,
replacement, rerun, or solver invocation in this lane.

## Metric and parser parity

The evaluator must reuse the Phase118/Phase120 metric contract and the pinned
Phase82/Phase76 parser/scorer.  The authoritative source is
`apps/commands/benchmarks/gnss_smartphone_phase120_tdcp_equation_normalization_accuracy.py`
(commit `7e91c25e10b53f252b978542d3ec59f57d845a2a`, SHA-256
`22c0f3eb6c3624d81a3e7662828a1eb299a464c352250c16c101ae64eadab0cf`), with
the same contract present in the pinned Phase118 evaluator.  No metric
semantic change is admitted:

- candidate header: `phone,UnixTimeMillis,LatitudeDegrees,LongitudeDegrees`;
- exact integer join key `(phone, UnixTimeMillis)`;
- duplicate prediction/truth keys and prediction keys absent from truth fail
  closed;
- candidate prediction-domain coverage must be exactly `1.0`;
- only the pinned leading MTV-A warm-up truth key may be absent; no fill,
  interpolation, nearest, hold, extrapolation, or repair;
- per-row spherical Haversine distance with earth radius `6371008.8` m;
- P50/P95 use linear interpolation at rank `(n - 1) * q`;
- route scalar is `(P50 + P95) / 2` m;
- macro is the unweighted mean in the fixed MTV-A then LAX-T order;
- all candidate coordinates and derived values must be finite and
  earth-valid, and transition speed must have zero samples above 70 m/s.

The evaluator reads truth only in one independently authorized truth-only
process, after candidate hash/metadata verification.  It records route
metrics and opaque hashes only; candidate or truth coordinate rows are never
written to a result.

## Fixed baselines and promotion rule

Sealed aggregate baselines are retained for comparison only:

| lane | MTV-A (m) | LAX-T (m) | macro (m) |
|---|---:|---:|---:|
| Phase112 same pipeline | 0.9976852530 | 0.6659910918 | 0.8318381724 |
| Phase118 champion | 1.0028266778 | 0.6255696228 | 0.8141981503 |
| Phase120 | not used for route promotion | not used for route promotion | 0.8170323381 |

The only promotion comparator is strict `candidate_macro_score_m < 0.782`;
equality fails.  A truth-only accuracy GO does not authorize release,
validation, or Kaggle submission.

## Pixel5 offset and leakage boundary

Phase134 structural output metadata confirms the official Pixel5 position
offset was applied exactly once in the native output path (`0.316227766` m,
corrected epochs equal the problem epoch count).  The evaluator must not apply
or estimate any offset.  Solver/raw/base reruns, MAT/PDC/precomputed phone
coordinates, truth reads before authorization, post-truth tuning, fallback,
repair, and publication are all fail-closed violations.

## Frozen authorization boundary

Before a future independent authorization, all of the following remain zero:

```text
candidate paths materialized       0
candidate payload/coordinate reads 0
truth paths materialized           0
truth reads                        0
accuracy calculations              0
native solver invocations          0
raw GNSS/IMU/navigation reads      0
raw base RINEX reads               0
MAT/PDC/precomputed reads          0
Kaggle/token access                0
reruns/fallbacks/repairs           0
```

The next boundary is an independent authorization commit pinning this audit,
the accuracy freeze, manifest, evaluator/tests, the Phase134 structural result,
and the sealed truth metadata.  Only after that commit may exactly two
candidate payload reads, exactly two truth reads, and exactly two score
calculations occur, in the fixed route order.  If any pin, schema, hash, row
count, or gate fails, the evaluator must stop without a retry.

