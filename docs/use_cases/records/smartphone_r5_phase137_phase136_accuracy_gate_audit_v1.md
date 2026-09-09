# Phase137 isolated accuracy-gate audit

## Scope and decision boundary

Phase137 evaluates only the two opaque solution artifacts sealed by the
Phase136/135 structural run.  This audit is launch-free: it reads tracked
source and sealed metadata, but does not materialize or open a candidate
solution, official truth, raw GNSS/IMU/navigation/base payload, MAT/PDC data,
or Kaggle artifact.  It does not launch the native solver, repair a solution,
re-run a route, or publish coordinates.

The source structural result is commit
`65b24a939a373421015f07b119fdbec04d5c6ebb`, JSON SHA-256
`d79dbac9dba4ce352d1d2b23f92d87fa44057221822a67f94c494f41f789c2a7`.
It records one successful Phase135 affine + Phase118 Huber structural run for
each route, with Pixel5 final-output offset applied exactly once.  The sealed
opaque candidate metadata is:

| route | candidate rows | problem epochs | bytes | opaque SHA-256 | coordinate rows read |
| --- | ---: | ---: | ---: | --- | ---: |
| MTV-A | 2158 | 2159 | 172694 | `dad4b5ee82942e11b051155f76fd28fe59c21e2928caba0d48a8955a6024005a` | 0 |
| LAX-T | 1465 | 1466 | 117254 | `df022cad3fdd68a3de2a6b2eceac0c0520a5a22fda61c76edf997efde964b519` | 0 |

The rows and bytes above are sealed metadata only; no solution content is
interpreted by this audit.  The candidate is exactly one immutable output per
route, evaluated in the fixed order MTV-A then LAX-T.

## Metric and gate parity

The evaluator must reuse the Phase134/Phase118 metric path and the pinned
Phase82/Phase76 parser semantics without modification.  The contract is:

* candidate and truth CSVs use the existing `phone,UnixTimeMillis,
  LatitudeDegrees,LongitudeDegrees` schema;
* exact integer `(phone, UnixTimeMillis)` joins are required, with duplicate or
  extra prediction keys failing closed;
* the one pinned leading warm-up truth key may be absent; no interpolation,
  nearest, edge hold, extrapolation, or fill is allowed;
* each row uses spherical Haversine distance with Earth radius 6371008.8 m;
  route score is `(P50 + P95) / 2`; macro is the unweighted mean in route
  order;
* all candidate values and derived metrics must be finite and Earth-valid,
  prediction-domain coverage must equal 1.0, and transition speed must have
  zero over-70-m/s rows;
* promotion is strict `candidate_macro_score_m < 0.782`; equality fails.

Pinned informational baselines, not tuning targets, are Phase112 macro
`0.8318381724` and Phase118 champion macro `0.8141981503`.  The Phase118
route scores are 1.0028266778 (MTV-A) and 0.6255696228 (LAX-T).  The evaluator
must not compare, alter, or re-run any baseline payload.

The Pixel5 offset is already present in the Phase136 output.  The evaluator
must not apply it again; double application is fail-closed.  Candidate hash
and metadata verification must precede any truth path materialization.  The
single truth-only evaluator process may then read exactly one candidate and
one truth payload per route and perform exactly two score calculations.

## Freeze recommendation

Freeze exactly one candidate: `phase137-phase136-official-affine-opaque-
truth-only-accuracy-v1`.  It is default to no evaluation until an independent
truth-only authorization commit pins this audit, freeze, evaluator, manifest,
tests, Phase136 result, and the existing metric references.  A failing gate
must be sealed unchanged; there is no retry, repair, fallback, route tuning,
or Kaggle submission authorization.

The planned pre-authorization accounting is zero for candidate/truth path
materialization, candidate or truth payload reads, coordinate interpretation,
accuracy calculations, native solver/raw/base reads, MAT/PDC/precomputed
coordinate reads, Kaggle access, reruns, and fallbacks.  The future authorized
lane is limited to candidate reads 2, truth reads 2, and accuracy calculations
2, with native/raw/base/MAT/PDC/Kaggle reads remaining zero.

## Pinned source references

* Phase134 evaluator: `apps/commands/benchmarks/gnss_smartphone_phase134_native_summary_bridge_accuracy.py`, SHA-256
  `b554e2c07bedffcb53751e4e6351bcf5006fe99370dee34a16cc4f3eb81b145a`,
  commit `b59ca771...` (the current full commit is recorded in the freeze).
* Phase118 evaluator: SHA-256
  `a1b2fac0a5615e47a11e7e4b171ced77d12818b66c598a531a4c48d2bf7ff829`,
  commit `c29e1700922260a77f047db8c82692f9d2b13603`.
* Phase82 scorer: SHA-256
  `232ba0ab1fa7ebde6ddfaaf851d190d1ed32377ea056ab6c31e244139c2977fb`,
  commit `be45879297a2a00a8574c77b71be0a5b721ff8c9`.
* Phase76 truth parser: SHA-256
  `513e5970ee58846963f077b9960944e5fbf5f3535e5be24ca6e72493785f7761`,
  commit `7a8e77ce03f6f4354cf2f541098a53628faa23d5`.

This document is an audit only.  Truth scoring and any release decision remain
outside its authority boundary.
