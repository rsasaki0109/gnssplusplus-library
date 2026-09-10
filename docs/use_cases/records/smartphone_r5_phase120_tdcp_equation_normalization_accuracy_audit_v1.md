# Phase120 isolated truth-only accuracy audit

Status: audit only; no truth, candidate coordinate rows, raw input, native
solver, MAT, PDC, precomputed-coordinate, Kaggle, or accuracy-evaluator
execution was performed while preparing this audit.

## Scope and decision

The single candidate is the already sealed Phase120 structural result
`6a050b1041c29a484aa791a39a0edcd533640f94`.  Only its two opaque solution
seals are in scope, in the fixed order MTV-A then LAX-T.  No solution row is
used by this audit.  The later evaluator may read exactly one candidate file
and one official truth file per route only after an independent authorization
commit.

The Phase120 structural result is a structural GO, with `truth_reads=0`,
`accuracy_scored=false`, and `solution_output_published=false`.  Its candidate
outputs are therefore eligible for a truth-only evaluation contract but are
not promoted or published by this audit.

## Immutable metric evidence

The metric is reused bit-for-bit from the sealed Phase118 truth-only
evaluator, which in turn pins the Phase102/103/112 contract and the Phase82
scorer plus Phase76 parser:

| Item | Immutable evidence |
|---|---|
| Phase118 evaluator source | commit `c29e1700922260a77f047db8c82692f9d2b13603`; SHA-256 `a1b2fac0a5615e47a11e7e4b171ced77d12818b66c598a531a4c48d2bf7ff829` |
| Phase118 accuracy manifest | commit `c29e1700922260a77f047db8c82692f9d2b13603`; SHA-256 `b33b276c32bbbb5e2a8778a9259cd9e8daf92d874e67aa01dc27a20fb46b186b` |
| Phase118 metric freeze | commit `ef8d11bc4fa64e2ed6cd5fface133979655953ff`; SHA-256 `860a7f382199958f8dd2c43ba697d4c5cde1bc550d5747028e4c6f73be346760` |
| Phase118 sealed aggregate | commit `6b37f92077a8ede43cb24e7a5cada580feccea5c`; SHA-256 `918597aab19f462a2aadfff606f461ce47f48f2fdcd997e5fdeb89ab459a0164` |
| Phase82 scorer source | SHA-256 `232ba0ab1fa7ebde6ddfaaf851d190d1ed32377ea056ab6c31e244139c2977fb` |
| Phase76 parser source | SHA-256 `513e5970ee58846963f077b9960944e5fbf5f3535e5be24ca6e72493785f7761` |

The exact contract is:

- CSV header `phone,UnixTimeMillis,LatitudeDegrees,LongitudeDegrees`;
- exact integer key `(phone, UnixTimeMillis)`, exact intersection only;
- duplicate keys and extra prediction keys fail closed;
- only the pinned leading MTV-A warm-up truth key may be absent;
- spherical Haversine distance with Earth radius `6371008.8 m`;
- route scalar `(P50 + P95) / 2` in metres, with linear percentile rank
  `(n - 1) * q`;
- transition speed guard `over_70_mps_count == 0`;
- unweighted arithmetic macro over exactly MTV-A then LAX-T;
- strict promotion comparator `candidate_macro_score_m < 0.782` (equality
  fails).

No metric, join, row, offset, or aggregation change is proposed.

## Candidate evidence (metadata only)

The Phase120 result JSON has SHA-256
`4c783ed063cc4f1bb3530826a40607eab6d8aa3c9716a8a8bedc8abebcdb48f5` and
records two opaque seals:

| Route | Opaque output path | SHA-256 | Bytes | Rows |
|---|---|---|---:|---:|
| MTV-A | `output/smartphone-r5/phase120-tdcp-equation-normalization-v1/2021-03-16-18-59-us-ca-mtv-a__pixel5/withheld_solution_output.csv` | `8fa9ab82dcddd80cb2dc04da69cdb797f737634daeb6fc6bd94f23564d039dad` | 172694 | 2158 |
| LAX-T | `output/smartphone-r5/phase120-tdcp-equation-normalization-v1/2022-04-01-18-22-us-ca-lax-t__pixel5/withheld_solution_output.csv` | `994f0891c151451de879daf12ddee7d3d1be853cf4c82ffe883456af07339146` | 117254 | 1465 |

These values are opaque metadata copied from the sealed structural result;
the candidate files were not opened or interpreted during this audit.

## Baselines and gates

The sealed comparison aggregates are Phase112 same-pipeline macro
`0.8318381724000121 m` and Phase118 macro `0.814198150322117 m`.  The latter
route scores are `1.002826677831249 m` (MTV-A) and `0.625569622812985 m`
(LAX-T).  They are comparison evidence only, not tuning inputs.

The one frozen evaluation candidate must require both routes finite and
earth-valid, exact candidate-domain coverage, zero over-70-m/s transitions,
the pinned warm-up policy, no route regression versus the sealed Phase112
baseline, and strict macro `<0.782 m`.  A failed route or missing payload is
fail-closed; there is no repair, replacement, rerun, fallback, or release.

## Read boundary

Before independent authorization, all candidate/truth path materialization,
candidate payload reads, truth reads, coordinate interpretation, native
invocations, raw/base reads, accuracy calculations, MAT/PDC/precomputed
coordinate reads, Kaggle/token access, reruns, and fallbacks are zero.  The
future evaluator is one isolated process, reads candidate metadata/hash and
parses one candidate per route before reading one truth file per route, and
emits only route metrics, strict gate booleans, opaque hashes, and accounting;
coordinate rows are not written to its result.  Truth remains unavailable to
the solver and to this audit/freeze lane.

## Single candidate freeze recommendation

Freeze exactly one default execution-free candidate:
`phase120-official-tdcp-resl-atmosphere-cancellation-truth-only-accuracy-v1`.
It reuses the Phase118 evaluator's metric/schema/join/aggregation unchanged
and changes only the sealed candidate result source.  The next implementation
boundary is a launch-free Phase120 evaluator adapter plus manifest and
synthetic tests; an independent truth-only authorization is required before
any payload path is materialized.
