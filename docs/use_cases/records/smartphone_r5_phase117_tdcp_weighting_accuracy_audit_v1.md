# Phase117 official TDCP SNR/type weighting truth-only accuracy audit

- Execution label: `Luna Max`
- Audit date: `2026-09-03`
- Status: `sealed-read-only-audit-qualified-for-phase117-truth-only-freeze`
- Scope: the two completed Phase117 raw/base structural runs, MTV-A and
  LAX-T. No native solver, raw GNSS/IMU/navigation, base RINEX, or truth
  payload is read by this audit.

## Structural authority

Phase117 result commit `b850fc4da7aeba552213230dacac790c4780c561` is a sealed
structural GO. It records exactly one native invocation per route, return code
zero, official TDCP SNR/type weighting enabled with source-derived finite
metre sigmas, unchanged ordinary-TDCP factor count, exact raw-base correction,
complete C7/D handoff, QR main progress, finite expected output coverage, and
no solver fallback. Its result SHA-256 is
`2517dcc805146dc34e790c78a392caf000c79eaba837fe099b6d522c147e9cf2`.
The pre-raw manifest and independent raw authorization are pinned by the
structural result as `676f5653a3f3ed95302da2eaec03d38e5757334076cf33134a4df5f8b0a682ec`
and `5bccd316f942ea7bb8bf2e0e69da627b92fd2dd76f1af4d75ad628e053fba202`.

The candidate CSVs were opened only by the structural wrapper for opaque
byte/hash and row-count sealing. This audit does not parse a header, timestamp,
latitude, longitude, or coordinate value. The sealed candidate metadata is:

| route | opaque candidate path | bytes | SHA-256 | rows |
|---|---|---:|---|---:|
| MTV-A | `output/smartphone-r5/phase117-tdcp-weighting-v1/2021-03-16-18-59-us-ca-mtv-a__pixel5/withheld_solution_output.csv` | 172694 | `4cbf4fae577146ac96d2f0bc324126e3d98a4a76c5ba8987384797a5b6a788cc` | 2158 |
| LAX-T | `output/smartphone-r5/phase117-tdcp-weighting-v1/2022-04-01-18-22-us-ca-lax-t__pixel5/withheld_solution_output.csv` | 117254 | `aaef4f13e5a7365e5dc3e7a71c046e6878e21dd1c016b0f3e514156d4f70895c` | 1465 |

The files remain withheld, unmodified, and unpublished. Candidate CSV parsing
and coordinate interpretation are deferred to one authorized truth-only
evaluator subprocess, after this freeze and a separate authorization.

## Metric and promotion boundary

Phase117 accuracy will reuse the already sealed Phase102/103/112 metric
contract and pinned Phase82 parser/scorer. No solver or TDCP implementation is
part of the accuracy lane. The exact contract is:

- integer `(phone, UnixTimeMillis)` keys and exact intersection only;
- duplicate or extra prediction keys fail closed; only the exact pinned MTV-A
  leading warm-up truth key may be absent;
- spherical Haversine distance with radius `6371008.8 m`;
- linear percentile rank `(n - 1) * q`, route scalar `(P50 + P95) / 2`, and
  unweighted macro in fixed MTV-A then LAX-T order;
- prediction-domain coverage exactly `1.0`, finite/Earth-valid coordinates,
  and zero prediction transitions above `70 m/s`.

The primary comparison anchor is the sealed Phase112 same-route pipeline,
whose candidate macro is `0.8318381724000121 m`; the strict promotion gate is
an explicit macro `<= 0.782 m` AND predicate. Phase112's sealed result also
records the same route order and metric definition. Accuracy GO, if achieved,
does not authorize release, validation, or Kaggle submission.

The official truth metadata is inherited from the sealed Phase112 truth
cohort and is not opened by this audit or the solver:

| route | truth path metadata | bytes | SHA-256 | rows |
|---|---|---:|---:|---:|
| MTV-A | `output/smartphone-r5/phase29-no-bridge-train-eval-v1/truth/2021-03-16-18-59-us-ca-mtv-a/pixel5/ground_truth.csv` | 210009 | `7c84ed6a80b1bbb08c0ffad57493513833b9d5474e22a43c5a44da82824ee22d` | 2159 |
| LAX-T | `output/smartphone-r5/phase44-pixel5-development-accuracy-v1/truth/2022-04-01-18-22-us-ca-lax-t/pixel5/ground_truth.csv` | 143441 | `29e0861dd1ecb8865c10adab69396d98ed96618e8877b09d04aa8d671edf79e8` | 1465 |

After authorization, one evaluator may read each pinned truth file exactly
once, only after candidate hash/schema/finite/Earth-valid preflight. The
result will contain route metrics, gate booleans, opaque hashes, and read
accounting only; no solution or truth coordinate rows.

## Read accounting for this audit

| activity | count |
|---|---:|
| native solver invocations | `0` |
| raw GNSS/IMU/navigation or base RINEX reads | `0` |
| truth payload reads | `0` |
| candidate coordinate interpretations | `0` |
| candidate opaque metadata/hash reads | `0` |
| MAT/precomputed phone-coordinate/PDC reads | `0` |
| accuracy calculations | `0` |
| Kaggle/token access | `0` |
| reruns/fallbacks | `0` |

Exactly one candidate is frozen for the next boundary:
`phase117-official-tdcp-snr-type-weighting-truth-only-accuracy-v1`. The next
artifacts are a launch-free evaluator/manifest/tests contract, a new
independent truth-only authorization, and one sealed accuracy result. Any
schema, alignment, finite, speed, metric, leakage, or gate failure is
fail-closed; there is no repair, retry, fallback, rerun, or publication.
