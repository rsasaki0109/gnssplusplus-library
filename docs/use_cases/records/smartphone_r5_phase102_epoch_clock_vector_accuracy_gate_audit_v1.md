# Phase102 epoch-clock-vector accuracy-gate audit

- Execution label: `Luna Max`
- Audit date: `2026-09-03`
- Status: `sealed-read-only-audit-qualified-for-one-candidate-freeze`
- Scope: only the Phase101 structural-GO routes `2021-03-16-18-59-us-ca-mtv-a/pixel5` (MTV-A) and `2022-04-01-18-22-us-ca-lax-t/pixel5` (LAX-T).
- Candidate: one fresh Phase101 source-parity solution per route, using the
  raw-only GNSS-first epoch-local C/ISB vector, D handoff, and Phase99 QR
  main branch already sealed by Phase101.

This is a read-only audit. It inspected committed source and sealed record
metadata only. It did not open raw GNSS, IMU, or navigation bytes; open a
truth file; invoke the native solver; generate or read a solution CSV; read
MAT, base, PDC, or precomputed coordinates; calculate accuracy; or access
Kaggle/tokens. The companion freeze authorizes neither raw execution nor
truth evaluation.

## Decision

Freeze exactly one opt-in candidate:

`phase102-phase101-epoch-clock-vector-accuracy-v1`

The candidate is a fresh, one-run-per-route Phase101 solution. The sealed
Phase101 structural output is qualification evidence only and is not reused
as the accuracy candidate. The evaluator must read the official truth files
only after both native processes exit and after the candidate solution hash
and metadata have been sealed. No solution rows may be published in the
result artifact.

## Evidence and comparison baselines

The structural authority is Phase101 result `59a60fb`. It records exact
retained-key C7 and D coverage, finite handoff, GNSS-first progress, and
Phase99 QR main progress for both routes; its withheld solution rows are not
accuracy evidence.

The two numeric comparison baselines are immutable sealed candidate scores,
using the same route identity, exact-key matching, Haversine distance,
percentile interpolation, and route scalar:

| Route | Phase82 same-route direct-quality score (m) | Phase100 QR scalar-clock score (m) |
|---|---:|---:|
| MTV-A | `1.1139384500152307` | `1.147436714982201` |
| LAX-T | `0.9389644134001871` | `3.3204375254720286` |

The Phase82 two-route arithmetic mean is `1.0264514317077089` and the
Phase100 QR two-route mean is `2.2339371202271145`. The official Phase82
four-route aggregate is not substituted for either two-route baseline.
Phase82 and Phase100 candidate rows/truth bytes remain sealed artifacts and
are not read by the future native solver. Their aggregate values are used
only by the isolated evaluator-side comparison.

## Exact metric and alignment contract

The evaluator reuses the Phase100/Phase82 contract without changing its
metric behavior:

- Candidate header is exactly `phone,UnixTimeMillis,LatitudeDegrees,LongitudeDegrees`.
- Truth is read with `csv.DictReader`; required fields are
  `UnixTimeMillis`, `LatitudeDegrees`, and `LongitudeDegrees`; an optional
  truth `phone` must equal the declared route, while absent `phone` is
  assigned to it.
- The identity key is `(phone, UnixTimeMillis)` with exact integer
  timestamps. Duplicate candidate/truth keys, candidate keys absent from
  truth, or any non-exact alignment fail closed.
- Prediction-domain coverage is matched prediction keys divided by
  prediction keys and must equal exactly `1.0`. Only the exact pinned leading
  warm-up truth key may be missing; no interpolation, nearest matching,
  fill, extrapolation, or edge hold is allowed.
- Row errors use spherical Haversine distance with Earth radius `6371008.8`
  metres. P50/P95 use linear interpolation at rank `(n - 1) * q`, and the
  route scalar is `(P50 + P95) / 2` metres.
- The macro is the unweighted arithmetic mean of exactly MTV-A then LAX-T.
  No four-route aggregate is mixed into this macro.
- Every coordinate, distance, speed, and reported metric must be finite and
  Earth-valid. Ordered prediction transition speed must have
  `over_70_mps_count == 0`.

Expected alignment is fixed at 2159 problem epochs/2158 prediction rows and
2159 truth rows with one pinned leading warm-up key for MTV-A, and 1466
problem epochs/1465 prediction rows and 1465 truth rows with no missing key
for LAX-T. Exact key alignment, strict timestamp order, no duplicates, and
no prediction extras are mandatory.

## Raw-only and evaluator boundary

The native command may receive only the Phase95-materialized
`device_gnss.csv`, `device_imu.csv`, and broadcast `brdc.nav` paths, plus the
already sealed Phase101 selectors. It may not receive a truth path or truth
environment variable, MAT/base/PDC/precomputed coordinates, an archive,
token, or Kaggle path. Raw content may not be copied or transformed.

The wrapper runs each native route exactly once, seals solution hash/bytes/
rows/header metadata after native exit, and then starts one separate
evaluator subprocess. That evaluator reads each pinned truth exactly once,
after all native runs and the solution pre-truth checks. It also reads only
sealed Phase82/Phase100 aggregate metadata for comparisons. No native launch
is permitted after truth access. Any predicate failure is preserved as a
no-go; there is no retry, fallback, tuning, route selection, or rerun.

## Promotion gates

Promotion is the conjunction of all gates below. The evaluator must record
every boolean and the exact route/macro values, including the strict gate
result; a failure is not repaired or softened.

| Gate | Required value |
|---|---|
| Candidate route set | exactly MTV-A then LAX-T |
| Fresh runs | exactly one Phase101 raw-only run per route |
| Output integrity | exact header, expected rows, exact keys, strict timestamps, no duplicates/extras |
| Prediction coverage | exactly `1.0` per route |
| Finite/Earth-valid | true for all evaluated rows and derived metrics |
| Continuity | `over_70_mps_count == 0` per route |
| Phase43 improvement | at least `0.05 m` per route and no route regression |
| Phase43 macro improvement | at least `0.1 m` |
| Absolute route limit | every route scalar at most `3.0 m` |
| Absolute macro limit | two-route macro at most `2.0 m` |
| Strict macro limit | two-route macro at most `0.782 m` |
| Phase82 comparison | no route scalar regression against same-route Phase82 candidate |
| Phase100 comparison | no route scalar regression against same-route Phase100 QR scalar-clock candidate |

The `0.782 m` strict macro result is an explicit AND gate and must be
reported even if a less strict gate already fails. A structural or accuracy
GO authorizes neither release, validation, nor Kaggle submission.

## Read accounting for this audit

| Activity | Count |
|---|---:|
| Native solver invocations | `0` |
| Raw GNSS/IMU/navigation reads | `0` |
| Truth-file reads | `0` |
| MAT/base/PDC/precomputed-coordinate reads | `0` |
| Accuracy calculations | `0` |
| Kaggle/token access | `0` |
| New solution or large output artifacts | `0` |

## Next boundary

The companion freeze JSON fixes the candidate, two baselines, metric,
alignment, leakage boundary, and strict gates. The next permissible commits
are the pre-raw evaluator/manifest/tests, an independent one-shot
authorization, and then exactly two fresh raw-only native runs followed by
one isolated truth evaluator. This audit does not itself authorize any of
those actions.
