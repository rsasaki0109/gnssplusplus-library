# Phase142 / Phase141 isolated accuracy-lane audit

Status: read-only audit; no candidate, truth, raw, native-solver, MAT, PDC,
precomputed-coordinate, accuracy, or Kaggle payload was read.

## Scope and immutable evidence

This lane evaluates only the opaque candidate seals from the immutable Phase141
structural result:

| item | value |
|---|---|
| structural result commit | `9b2427bb03ca14df5bf3c3668d46d3638b03388a` |
| structural result path | `docs/use_cases/records/smartphone_r5_phase141_telemetry_schema_raw_result_v1.json` |
| structural result SHA-256 | `33d67248b9423a841ebb29b6373eed042d4da6b15924df710dcc37f18b73ff13` |
| route order | MTV-A, then LAX-T |
| runs | exactly one per route, already sealed |

The Phase141 result is structural-only and explicitly reports
`coordinate_rows_interpreted=false`.  The candidate seals are metadata, not
coordinate input:

| route | opaque SHA-256 | bytes | newline count | expected prediction rows |
|---|---|---:|---:|---:|
| MTV-A | `e1f8211b16a413622c95903e73d5b5f17fff0ee9acdb28bdce5bdd15708b229c` | 172694 | 2159 | 2158 |
| LAX-T | `85f5610f3deffae9912ed9ea026be2b4a4e0824064516e6c93e591255c1f4fcd` | 117254 | 1466 | 1465 |

No solution path is materialized or opened by this audit.  The Phase141
structural telemetry already records one Pixel5 output-offset application per
route; Phase142 must treat that as an immutable boundary and must not apply an
offset again.

## Metric and parser parity

Phase142 reuses the Phase137/134/118 metric implementation and does not copy or
reinterpret it.  The pinned contract is:

* required candidate header: `phone,UnixTimeMillis,LatitudeDegrees,LongitudeDegrees`;
* exact integer `(phone, UnixTimeMillis)` joins, with duplicate or extra
  prediction keys failing closed;
* only the exact declared leading warm-up truth omission is allowed; no
  interpolation, nearest, hold, extrapolation, or fill;
* spherical Haversine distance with Earth radius `6371008.8` metres;
* linear percentile interpolation at rank `(n - 1) * q`;
* route scalar `(P50 + P95) / 2` metres and an unweighted macro in fixed
  MTV-A → LAX-T order;
* all candidate values and derived metrics finite, Earth-valid, and prediction
  domain coverage exactly `1.0`; transition speed over 70 m/s exactly zero;
* promotion is strict `macro < 0.782` metres; equality fails.

The evaluator will pin source hashes for the Phase134 metric evaluator, the
Phase118 evaluator, the Phase82 scorer, and the Phase76 truth parser.  Their
truth-cohort paths and hashes are deferred metadata only.  Truth is read by a
separate, independently authorized evaluator process, never by the solver or
this audit.

## Baselines and decision boundary

The sealed comparison baselines are Phase112 macro `0.8318381724` metres and
Phase118 champion macro `0.8141981503` metres.  They are informational
references; no baseline payload is reopened and no tuning or sweep is
authorized.  A Phase142 candidate can be promoted only when both route
evaluations pass every finite/full-alignment/domain/velocity guard and the
unweighted macro is strictly below `0.782` metres.

## Launch-free accounting and authorization boundary

This audit's accounting is all zero: candidate paths materialized `0`, opaque
solution payload reads `0`, candidate coordinate interpretations `0`, truth
reads `0`, accuracy calculations `0`, raw GNSS/IMU/navigation/base reads `0`,
native solver invocations `0`, MAT/PDC/precomputed-coordinate reads `0`, and
Kaggle/token accesses `0`.  No files are copied, transformed, repaired,
published, or rerun.

The next artifacts are deliberately separate: a machine-readable freeze, a
launch-free evaluator/manifest/tests commit, and a pre-truth accounting seal.
An independent truth-only authorization must be committed and verified after
those pins are complete.  Only then may exactly two candidate payload reads,
exactly two truth reads, and exactly two score calculations occur, in MTV-A →
LAX-T order.  A failed gate is final for that authorization; there is no
fallback, repair, rerun, or Kaggle submission.

