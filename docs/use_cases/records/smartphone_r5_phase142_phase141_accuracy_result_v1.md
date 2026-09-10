# Phase142 isolated truth-only accuracy result

Status: `NO-GO` (fail-closed).  This is the single authorized evaluation in
MTV-A → LAX-T order.  No rerun, repair, fallback, publication, solver run,
raw/base read, MAT/PDC/precomputed-coordinate read, or Kaggle access occurred.

## Pins

| artifact | commit / SHA-256 |
|---|---|
| independent truth authorization | `ffe7e5981598d906c328752088de19c46a3119da` / `b8258e17b040f92d0d935661f5f19bdc301d48fe6be9e0758a45e0bb5e0ba643` |
| structural result | `9b2427bb03ca14df5bf3c3668d46d3638b03388a` / `33d67248b9423a841ebb29b6373eed042d4da6b15924df710dcc37f18b73ff13` |
| evaluator/manifest/tests | `adbce9bd266922bb584ffb40db7134790c71be07` |
| result JSON | `6c20c63523b12417e00f7faca63d60f7cbfc93100d053c113a324a83cd44d138` |

Candidate solution contents and truth coordinate rows are not reproduced here;
only the evaluator's opaque hash/row metadata is sealed in the JSON artifact.

## Scores and gates

| route | score (m) | finite/full/domain/velocity |
|---|---:|---|
| MTV-A | 0.9985004195844207 | pass |
| LAX-T | 0.6299720596873719 | pass |
| unweighted macro | **0.8142362396358963** | strict `< 0.782`: fail |

Both routes had exact prediction-domain coverage `1.0`, finite/Earth-valid
values, and zero transitions over 70 m/s.  Promotion is not authorized because
the strict macro threshold was not met.  Kaggle submission remains forbidden.

## Read accounting

Exactly two candidate reads, two truth reads, and two accuracy calculations were
performed (one per route).  Native solver, raw GNSS/IMU/navigation, raw base,
MAT/PDC/precomputed-coordinate, Kaggle, rerun, fallback, and repair counts are
all zero.  The JSON result contains no solution or truth coordinate rows and
release/submission remains unauthorized.

